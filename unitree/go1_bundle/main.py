#!/usr/bin/env python3
"""
go1_bundle/main.py — Unitree Go1 (EDU) 状态 + 基础控制驱动入口（原始 unitree_legged_sdk）。

一个驱动 = 一个 MCP server。本 bundle 是从完整 go1 驱动中**切出的最小蓝本**，当前聚合
3 张状态卡（loco_state / battery / obstacle_range）+ 1 张控制卡（spin）。每张卡是一个
**自包含的 .py 文件**，main.py 按 config.yaml 里启用的卡名**自动 import 同名模块**并装配
（约定：config key == 模块名 == 文件名 == 卡名）。因此新增一张卡 = 新建 `<卡名>.py` +
在 config.yaml 打开它，**不用改本文件**。

所有卡共享同一个 raw SDK client（唯一 UDP 收发线程 → snapshot()）；状态卡只读 snapshot 的
不同切片，控制卡经该 client 的下发原语发 HighCmd。装了 rclpy 时状态卡发 ROS2 topic 在画布
渲染，否则走 MCP action=info。固定 HIGHLEVEL（读 HighState / 下发 HighCmd）。
无 robot_interface / 无硬件时自动 STUB（server 仍能起、注册、列 tool）。

用法： python3 main.py [networkInterface]   环境变量： CONFIG_PATH / AGENT_CORE_URL
详见同目录 CONTRIBUTING.md。
"""

from __future__ import annotations

import importlib
import json
import os
import re
import signal
import socket
import sys
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import yaml

try:
    import rclpy
    import rclpy.executors
    _HAS_ROS2 = True
except Exception:
    _HAS_ROS2 = False


def _load_config() -> dict:
    config_path = os.environ.get("CONFIG_PATH", str(Path(__file__).parent / "config.yaml"))
    with open(config_path) as f:
        return yaml.safe_load(f)


def _resolve_namespace(cfg: dict) -> str:
    ns = cfg.get("ros_namespace", "").strip()
    if ns:
        return re.sub(r"[^a-zA-Z0-9_]", "_", ns)
    return re.sub(r"[^a-zA-Z0-9_]", "_", socket.gethostname())


class Go1Bundle:
    """按 config 装配启用的卡片；对外提供 tools 列表与 dispatch。

    装配规则：遍历 config.plugins 里 enabled 的每个卡名 → import_module(卡名) →
    调用其 make_plugin(plugin_config, namespace, executor, client)。卡名即模块名。
    """

    def __init__(self, cfg, namespace, executor, client):
        self._plugins = []
        pc = cfg.get("plugins", {}) or {}
        for card, conf in pc.items():
            if not (isinstance(conf, dict) and conf.get("enabled")):
                continue
            try:
                mod = importlib.import_module(card)
                self._plugins.append(mod.make_plugin(conf, namespace, executor, client))
            except Exception as e:
                print(f"[bundle] 卡 '{card}' 加载失败，跳过: {e}", flush=True)
        print(f"[bundle] {len(self._plugins)} plugins: {[type(p).__module__ for p in self._plugins]}", flush=True)

    def start_all(self):
        for i, p in enumerate(self._plugins):
            try:
                p.start()
            except Exception as e:
                print(f"[bundle] Plugin {i} ({type(p).__module__}) start() FAILED: {e}", flush=True)
        print(f"[bundle] All {len(self._plugins)} plugins started", flush=True)

    def stop_all(self):
        for p in self._plugins:
            try:
                p.stop()
            except Exception:
                pass
        print("[bundle] All plugins stopped")

    def get_all_tools(self):
        tools = []
        for p in self._plugins:
            tools.extend(p.get_tools() if hasattr(p, "get_tools") else [p.get_tool()])
        return tools

    def dispatch(self, tool_name, args):
        for p in self._plugins:
            for tool_def in (p.get_tools() if hasattr(p, "get_tools") else [p.get_tool()]):
                if tool_def["name"] == tool_name:
                    if tool_def["type"] == "resource":
                        return p.dispatch(tool_name, args)
                    action = args.pop("action", tool_name)
                    args["_tool_name"] = tool_name
                    return p.dispatch(action, args)
        return None


_bundle = None


def make_handler():
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, fmt, *args):
            msg = fmt % args
            if '"POST /mcp' in msg and "200" in msg:
                return
            print(f"[mcp] {self.address_string()} {msg}")

        def _send(self, status, body):
            enc = body.encode()
            self.send_response(status)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(enc)))
            self.send_header("Access-Control-Allow-Origin", "*")
            self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
            self.send_header("Access-Control-Allow-Headers", "Content-Type, Accept")
            self.end_headers()
            self.wfile.write(enc)

        def do_GET(self):
            self.send_response(404)
            self.end_headers()

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
                self._send(200, json.dumps({"jsonrpc": "2.0", "id": rid, "result": result}))

            def err(code, msg):
                self._send(200, json.dumps({"jsonrpc": "2.0", "id": rid,
                                            "error": {"code": code, "message": msg}}))
            try:
                if method == "initialize":
                    ok({"protocolVersion": "2024-11-05", "capabilities": {"tools": {}},
                        "serverInfo": {"name": "go1-bundle", "version": "1.0.0"}})
                elif method == "tools/list":
                    ok({"tools": _bundle.get_all_tools()})
                elif method == "tools/call":
                    name = params.get("name", "")
                    args = params.get("arguments") or {}
                    result = _bundle.dispatch(name, args)
                    if result is None:
                        err(-32601, f"Unknown tool: {name}")
                    else:
                        ok({"content": [{"type": "text", "text": json.dumps(result)}]})
                else:
                    err(-32601, f"Method not found: {method}")
            except Exception as e:
                err(-32603, str(e))

    return Handler


def _start_registration(mcp_port, name, category):
    import urllib.request as _urllib
    import ssl as _ssl
    agent_core_url = os.environ.get("AGENT_CORE_URL", "https://localhost:15678")
    # 与 agent-core 同机 → localhost；跨机须设 MCP_ADVERTISE_HOST
    advertise_host = os.environ.get("MCP_ADVERTISE_HOST", "localhost")
    payload = json.dumps({"name": name, "url": f"http://{advertise_host}:{mcp_port}/mcp", "category": category}).encode()
    _ctx = _ssl.create_default_context()
    _ctx.check_hostname = False
    _ctx.verify_mode = _ssl.CERT_NONE

    def _run():
        import time as _t
        while True:
            try:
                req = _urllib.Request(f"{agent_core_url}/api/mcp", data=payload,
                                      headers={"Content-Type": "application/json"}, method="POST")
                with _urllib.urlopen(req, timeout=3, context=_ctx):
                    pass
                _t.sleep(30)
            except Exception as e:
                print(f"[register] failed: {e}, retrying in 5s")
                _t.sleep(5)

    threading.Thread(target=_run, daemon=True, name="register").start()


def main():
    global _bundle
    network_iface = sys.argv[1] if len(sys.argv) >= 2 else ""
    cfg = _load_config()
    namespace = _resolve_namespace(cfg)
    mcp_port = int(cfg.get("mcp_port", 15717))

    print(f"[bundle] namespace={namespace} mcp_port={mcp_port} control_level=HIGHLEVEL")

    from go1_sdk_client import Go1HighSdkClient
    client = Go1HighSdkClient(network_iface=network_iface)
    client.start()
    print(f"[bundle] raw SDK client started ({'live' if client.available else 'STUB'})")

    executor = None
    if _HAS_ROS2:
        try:
            rclpy.init()
            executor = rclpy.executors.MultiThreadedExecutor()
            print("[bundle] ROS2 executor ready")
        except Exception as e:
            print(f"[bundle] ROS2 init 失败，状态卡走 MCP 轮询: {e}")
    else:
        print("[bundle] 未检测到 rclpy，状态卡走 MCP 轮询")

    _bundle = Go1Bundle(cfg, namespace, executor, client)
    _bundle.start_all()

    if executor is not None:
        def _spin():
            while rclpy.ok():
                executor.spin_once(timeout_sec=0.1)
        threading.Thread(target=_spin, daemon=True, name="bundle_spin").start()

    _start_registration(mcp_port, cfg.get("name", "Unitree Go1 (Bundle)"), "driver")

    server = ThreadingHTTPServer(("", mcp_port), make_handler())
    print(f"[bundle] MCP server → http://localhost:{mcp_port}")

    def _shutdown(signum, frame):
        print(f"[bundle] signal {signum}, shutting down")
        _bundle.stop_all()
        client.stop()
        threading.Thread(target=server.shutdown, daemon=True).start()

    signal.signal(signal.SIGTERM, _shutdown)
    signal.signal(signal.SIGINT, _shutdown)
    try:
        server.serve_forever()
    finally:
        _bundle.stop_all()
        client.stop()
        if executor is not None:
            executor.shutdown()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
