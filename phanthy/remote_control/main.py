#!/usr/bin/env python3
"""
drivers/phanthy/remote_control/main.py — Phanthy Remote Control MCP 驱动。

架构：
  - MCP JSON-RPC 2.0: 同步 HTTPServer（主线程）
  - WebSocket /ws/mic: asyncio 线程（接收浏览器 PCM-16k 流）
  - ROS2 spin: daemon 线程（spin_once 循环）

环境变量：
    CONFIG_PATH    — config.yaml 路径
    AGENT_CORE_URL — Agent Core 地址（默认 http://localhost:15678）
"""

import json
import os
import signal
import sys
import threading
from http.server import BaseHTTPRequestHandler, HTTPServer
from pathlib import Path

import yaml

import rclpy
import rclpy.executors

from plugins import RemoteMessagePlugin, RemoteAudioPlugin

# ── Config ────────────────────────────────────────────────────────────────────


def _load_config() -> dict:
    config_path = os.environ.get("CONFIG_PATH", str(Path(__file__).parent / "config.yaml"))
    with open(config_path) as f:
        return yaml.safe_load(f)


# ── Bundle ────────────────────────────────────────────────────────────────────


class RemoteControlBundle:
    def __init__(self, namespace: str, executor):
        self._plugins = [
            RemoteMessagePlugin(namespace, executor),
            RemoteAudioPlugin(namespace, executor),
        ]

    def start_all(self):
        for p in self._plugins:
            p.start()
        print("[bundle] All plugins started")

    def stop_all(self):
        for p in self._plugins:
            p.stop()
        print("[bundle] All plugins stopped")

    def get_all_tools(self) -> list:
        tools = []
        for p in self._plugins:
            if hasattr(p, "get_tools"):
                tools.extend(p.get_tools())
            else:
                tools.append(p.get_tool())
        return tools

    def dispatch(self, tool_name: str, args: dict) -> dict | None:
        for p in self._plugins:
            plugin_tools = p.get_tools() if hasattr(p, "get_tools") else [p.get_tool()]
            for tool_def in plugin_tools:
                if tool_def["name"] == tool_name:
                    action = args.pop("action", None)
                    if not action:
                        return {"error": "Missing required parameter: action"}
                    return p.dispatch(action, args)
        return None


# ── MCP HTTP server (sync, same pattern as Unitree) ──────────────────────────

_bundle: RemoteControlBundle | None = None


def _make_handler():
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, fmt, *args):
            print(f"[mcp] {self.address_string()} {fmt % args}")

        def _send(self, status: int, body: str):
            encoded = body.encode()
            self.send_response(status)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(encoded)))
            self.send_header("Access-Control-Allow-Origin", "*")
            self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
            self.send_header("Access-Control-Allow-Headers", "Content-Type, Accept")
            self.end_headers()
            self.wfile.write(encoded)

        def do_OPTIONS(self):
            self.send_response(204)
            self.send_header("Access-Control-Allow-Origin", "*")
            self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
            self.send_header("Access-Control-Allow-Headers", "Content-Type, Accept")
            self.end_headers()

        def do_POST(self):
            length = int(self.headers.get("Content-Length", 0))
            raw = self.rfile.read(length)
            try:
                rpc = json.loads(raw)
            except Exception:
                self._send(400, json.dumps({"jsonrpc": "2.0", "id": None,
                                            "error": {"code": -32700, "message": "Parse error"}}))
                return

            rid = rpc.get("id")
            method = rpc.get("method", "")
            params = rpc.get("params") or {}

            if rid is None:
                self.send_response(202)
                self.end_headers()
                return

            def ok(result):
                self._send(200, json.dumps({"jsonrpc": "2.0", "id": rid, "result": result}, ensure_ascii=False))

            def err(code, msg):
                self._send(200, json.dumps({"jsonrpc": "2.0", "id": rid,
                                            "error": {"code": code, "message": msg}}))

            try:
                if method == "initialize":
                    ok({
                        "protocolVersion": "2024-11-05",
                        "capabilities": {"tools": {}},
                        "serverInfo": {"name": "phanthy-remote-control", "version": "1.0.0"},
                    })
                elif method == "tools/list":
                    ok({"tools": _bundle.get_all_tools()})
                elif method == "tools/call":
                    name = params.get("name", "")
                    args = params.get("arguments") or {}
                    result = _bundle.dispatch(name, dict(args))
                    if result is None:
                        err(-32601, f"Unknown tool: {name}")
                    else:
                        ok({"content": [{"type": "text", "text": json.dumps(result, ensure_ascii=False)}]})
                else:
                    err(-32601, f"Method not found: {method}")
            except Exception as e:
                err(-32603, str(e))

    return Handler


# ── WebSocket mic server (asyncio thread) ────────────────────────────────────



# ── Registration heartbeat ───────────────────────────────────────────────────


def _start_registration(mcp_port: int, name: str, category: str):
    """Register with agent-core, heartbeat every 30s."""
    import urllib.request as _urllib

    agent_core_url = os.environ.get("AGENT_CORE_URL", "http://localhost:15678")
    payload = json.dumps({
        "name": name,
        "url": f"http://localhost:{mcp_port}/mcp",
        "category": category,
    }).encode()

    def _run():
        import time as _t
        while True:
            try:
                req = _urllib.Request(
                    f"{agent_core_url}/api/mcp",
                    data=payload,
                    headers={"Content-Type": "application/json"},
                    method="POST",
                )
                with _urllib.urlopen(req, timeout=3):
                    pass
                _t.sleep(30)
            except Exception as e:
                print(f"[register] failed: {e}, retrying in 5s")
                _t.sleep(5)

    threading.Thread(target=_run, daemon=True, name="register").start()


# ── Entry point ──────────────────────────────────────────────────────────────


def main():
    global _bundle

    cfg = _load_config()
    mcp_port = int(cfg.get("mcp_port", 15710))
    namespace = cfg.get("ros_namespace", "phanthy/remote_control").replace("/", "_")

    # Init ROS2
    rclpy.init()
    executor = rclpy.executors.MultiThreadedExecutor()

    # Init bundle
    _bundle = RemoteControlBundle(namespace, executor)
    _bundle.start_all()

    # ROS2 spin thread (spin_once loop, same as Unitree driver)
    def _spin():
        while rclpy.ok():
            executor.spin_once(timeout_sec=0.1)

    threading.Thread(target=_spin, daemon=True, name="ros2_spin").start()

    # Start registration heartbeat
    _start_registration(mcp_port, "Phanthy Remote Control", "driver")

    # MCP HTTP server (main thread, sync)
    server = HTTPServer(("", mcp_port), _make_handler())
    print(f"[main] Phanthy Remote Control driver started")
    print(f"[main] MCP endpoint: http://localhost:{mcp_port}/mcp")

    def _shutdown(signum, frame):
        print(f"\n[main] signal {signum}, shutting down")
        _bundle.stop_all()
        threading.Thread(target=server.shutdown, daemon=True).start()

    signal.signal(signal.SIGINT, _shutdown)
    signal.signal(signal.SIGTERM, _shutdown)

    try:
        server.serve_forever()
    finally:
        _bundle.stop_all()
        executor.shutdown()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
