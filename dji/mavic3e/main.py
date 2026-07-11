#!/usr/bin/env python3
"""
dji/mavic3e/main.py — DJI Mavic 3E 无人机设备 bundle 统一入口。

读取 config.yaml，按插件配置加载插件，聚合成一个 MCP HTTP server 对外暴露。
通过 bridge_client 与 psdk_bridge (C 进程) 通信，或在 mock 模式下模拟响应。

用法：
    python3 main.py

环境变量：
    CONFIG_PATH — config.yaml 路径（默认同目录下）
    AGENT_CORE_URL — Agent Core 地址（默认 https://localhost:15678）
"""

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


# ── Config ────────────────────────────────────────────────────────────────────

def _load_config() -> dict:
    config_path = os.environ.get("CONFIG_PATH", str(Path(__file__).parent / "config.yaml"))
    with open(config_path) as f:
        return yaml.safe_load(f)


def _resolve_namespace(cfg: dict) -> str:
    ns = cfg.get("ros_namespace", "").strip()
    if ns:
        return re.sub(r"[^a-zA-Z0-9_]", "_", ns)
    return re.sub(r"[^a-zA-Z0-9_]", "_", socket.gethostname())


# ── Bundle ────────────────────────────────────────────────────────────────────

class Mavic3EDeviceBundle:
    def __init__(self, cfg: dict, namespace: str, executor, bridge):
        self._plugins: list = []
        self._bridge = bridge
        self._namespace = namespace
        plugins_cfg = cfg.get("plugins", {})

        if plugins_cfg.get("telemetry", {}).get("enabled", False):
            from device import TelemetryPlugin
            self._plugins.append(TelemetryPlugin(
                plugins_cfg["telemetry"], namespace, executor, bridge))
            print("[bundle] TelemetryPlugin loaded")

        if plugins_cfg.get("camera_stream", {}).get("enabled", False):
            from device import CameraStreamPlugin
            self._plugins.append(CameraStreamPlugin(
                plugins_cfg["camera_stream"], namespace, executor, bridge))
            print("[bundle] CameraStreamPlugin loaded")

        if plugins_cfg.get("perception", {}).get("enabled", False):
            from device import PerceptionPlugin
            self._plugins.append(PerceptionPlugin(
                plugins_cfg["perception"], namespace, executor, bridge))
            print("[bundle] PerceptionPlugin loaded")

        if plugins_cfg.get("hms", {}).get("enabled", False):
            from device import HmsPlugin
            self._plugins.append(HmsPlugin(
                plugins_cfg["hms"], namespace, executor, bridge))
            print("[bundle] HmsPlugin loaded")

        if plugins_cfg.get("flight", {}).get("enabled", False):
            from device import FlightPlugin
            self._plugins.append(FlightPlugin(
                plugins_cfg["flight"], namespace, executor, bridge))
            print("[bundle] FlightPlugin loaded")

        if plugins_cfg.get("camera", {}).get("enabled", False):
            from device import CameraPlugin
            self._plugins.append(CameraPlugin(
                plugins_cfg["camera"], namespace, executor, bridge))
            print("[bundle] CameraPlugin loaded")

        if plugins_cfg.get("gimbal", {}).get("enabled", False):
            from device import GimbalPlugin
            self._plugins.append(GimbalPlugin(
                plugins_cfg["gimbal"], namespace, executor, bridge))
            print("[bundle] GimbalPlugin loaded")

        if plugins_cfg.get("waypoint", {}).get("enabled", False):
            from device import WaypointPlugin
            self._plugins.append(WaypointPlugin(
                plugins_cfg["waypoint"], namespace, executor, bridge))
            print("[bundle] WaypointPlugin loaded")

        if plugins_cfg.get("speaker", {}).get("enabled", False):
            from device import SpeakerPlugin
            self._plugins.append(SpeakerPlugin(
                plugins_cfg["speaker"], namespace, executor, bridge))
            print("[bundle] SpeakerPlugin loaded")

        if plugins_cfg.get("power", {}).get("enabled", False):
            from device import PowerPlugin
            self._plugins.append(PowerPlugin(
                plugins_cfg["power"], namespace, executor, bridge))
            print("[bundle] PowerPlugin loaded")

    def start_all(self) -> None:
        for i, p in enumerate(self._plugins):
            try:
                p.start()
            except Exception as e:
                print(f"[bundle] Plugin {i} ({type(p).__name__}) start() FAILED: {e}", flush=True)
                import traceback
                traceback.print_exc()
        print(f"[bundle] All {len(self._plugins)} plugins started", flush=True)

    def stop_all(self) -> None:
        for p in self._plugins:
            try:
                p.stop()
            except Exception:
                pass
        self._bridge.stop()
        print("[bundle] All plugins stopped")

    def get_all_tools(self) -> list:
        tools = [self._model_tool()]
        for p in self._plugins:
            if hasattr(p, "get_tools"):
                tools.extend(p.get_tools())
            else:
                tools.append(p.get_tool())
        return tools

    def _model_tool(self) -> dict:
        return {
            "name": "model",
            "type": "resource",
            "description": "DJI Mavic 3E aircraft metadata (cameras, gimbal range, specs)",
            "inputSchema": {"type": "object", "properties": {}},
        }

    def dispatch(self, tool_name: str, args: dict) -> dict | None:
        if tool_name == "model":
            info_path = Path(__file__).parent / "resource" / "mavic3e_info.json"
            return json.loads(info_path.read_text())
        for p in self._plugins:
            plugin_tools = p.get_tools() if hasattr(p, "get_tools") else [p.get_tool()]
            for tool_def in plugin_tools:
                if tool_def["name"] == tool_name:
                    if tool_def["type"] == "resource":
                        return p.dispatch(tool_name, args)
                    action = args.pop("action", tool_name)
                    args["_tool_name"] = tool_name
                    return p.dispatch(action, args)
        return None


# ── MCP HTTP server ───────────────────────────────────────────────────────────

_bundle: Mavic3EDeviceBundle | None = None


def make_handler():
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, fmt, *args):
            msg = fmt % args
            if '"POST /mcp' in msg and "200" in msg:
                return
            print(f"[mcp] {self.address_string()} {msg}")

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
                self._send(400, json.dumps({
                    "jsonrpc": "2.0", "id": None,
                    "error": {"code": -32700, "message": "Parse error"},
                }))
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
                self._send(200, json.dumps({
                    "jsonrpc": "2.0", "id": rid,
                    "error": {"code": code, "message": msg},
                }))

            try:
                if method == "initialize":
                    ok({
                        "protocolVersion": "2024-11-05",
                        "capabilities": {"tools": {}},
                        "serverInfo": {"name": "dji-mavic3e-bundle", "version": "1.0.0"},
                    })
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


# ── Device wait (container startup) ───────────────────────────────────────

def _wait_for_device(device_path: str, timeout: int = 30):
    """Wait for E-Port USB device to appear (handles container startup race condition)."""
    import time as _t
    start = _t.time()
    while not os.path.exists(device_path):
        elapsed = _t.time() - start
        if elapsed > timeout:
            print(f"[bundle] WARNING: {device_path} not found after {timeout}s — "
                  "E-Port may not be connected")
            return
        if int(elapsed) % 5 == 0 and int(elapsed) > 0:
            print(f"[bundle] waiting for {device_path}... ({int(elapsed)}s)")
        _t.sleep(1)
    print(f"[bundle] {device_path} detected")


# ── Entry point ───────────────────────────────────────────────────────────────

def _start_registration(mcp_port: int, name: str, category: str):
    """Register this driver with agent-core in a background thread, then heartbeat every 30s."""
    import urllib.request as _urllib
    import ssl as _ssl

    agent_core_url = os.environ.get("AGENT_CORE_URL", "https://localhost:15678")
    payload = json.dumps({
        "name": name,
        "url": f"http://localhost:{mcp_port}/mcp",
        "category": category,
    }).encode()
    _ctx = _ssl.create_default_context()
    _ctx.check_hostname = False
    _ctx.verify_mode = _ssl.CERT_NONE

    def _run():
        import time as _t
        while True:
            try:
                req = _urllib.Request(
                    f"{agent_core_url}/api/mcp", data=payload,
                    headers={"Content-Type": "application/json"}, method="POST",
                )
                with _urllib.urlopen(req, timeout=3, context=_ctx):
                    pass
                _t.sleep(30)
            except Exception as e:
                print(f"[register] failed: {e}, retrying in 5s")
                _t.sleep(5)

    threading.Thread(target=_run, daemon=True, name="register").start()


def main():
    global _bundle

    cfg = _load_config()
    namespace = _resolve_namespace(cfg)
    mcp_port = int(cfg.get("mcp_port", 15702))
    psdk_cfg = cfg.get("psdk_bridge", {})
    mock_mode = psdk_cfg.get("mock_mode", True)

    print(f"[bundle] namespace={namespace} mcp_port={mcp_port} mock={mock_mode}")

    # Bridge client — communicates with C psdk_bridge or runs in mock mode
    from bridge_client import BridgeClient

    if not mock_mode:
        # Wait for E-Port device to appear (USB hotplug in container)
        uart_dev = psdk_cfg.get("uart_dev", "/dev/ttyACM0")
        _wait_for_device(uart_dev)

    bridge = BridgeClient(mock_mode=mock_mode)
    print(f"[bundle] BridgeClient initialized (mock={mock_mode})")

    # ROS2
    rclpy.init()
    executor = rclpy.executors.MultiThreadedExecutor()

    _bundle = Mavic3EDeviceBundle(cfg, namespace, executor, bridge)
    _bundle.start_all()

    def _spin():
        while rclpy.ok():
            executor.spin_once(timeout_sec=0.1)

    spin_thread = threading.Thread(target=_spin, daemon=True, name="bundle_spin")
    spin_thread.start()

    _start_registration(mcp_port, cfg.get("name", "DJI Mavic 3E"), "driver")

    server = ThreadingHTTPServer(("", mcp_port), make_handler())
    print(f"[bundle] MCP server → http://localhost:{mcp_port}")

    def _shutdown(signum, frame):
        print(f"[bundle] signal {signum}, shutting down")
        _bundle.stop_all()
        threading.Thread(target=server.shutdown, daemon=True).start()

    signal.signal(signal.SIGTERM, _shutdown)
    signal.signal(signal.SIGINT, _shutdown)

    try:
        server.serve_forever()
    finally:
        _bundle.stop_all()
        executor.shutdown()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
