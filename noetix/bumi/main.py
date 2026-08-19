#!/usr/bin/env python3
"""
drivers/noetix/bumi/main.py — Noetix Bumi-EDU 设备 bundle 统一入口。

读取 config.yaml，按插件配置加载插件，聚合成一个 MCP HTTP server 对外暴露。
驱动启动时自动 start 所有插件，关闭时自动 stop。

用法：
    python3 main.py

环境变量：
    CONFIG_PATH — config.yaml 路径（默认同目录下）
"""

import json
import os
import re
import signal
import socket
import subprocess
import sys
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import yaml

import rclpy
import rclpy.executors


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

class BumiDeviceBundle:
    def __init__(self, cfg: dict, namespace: str, executor, high_ctrl, media_ctrl):
        self._plugins: list = []
        plugins_cfg = cfg.get("plugins", {})

        if plugins_cfg.get("state", {}).get("enabled", False) and high_ctrl is not None:
            from device import StatePlugin
            self._plugins.append(StatePlugin(plugins_cfg["state"], namespace, executor, high_ctrl))
            print("[bundle] StatePlugin loaded")

        if plugins_cfg.get("loco", {}).get("enabled", False) and high_ctrl is not None:
            from device import LocoPlugin
            self._plugins.append(LocoPlugin(plugins_cfg["loco"], namespace, executor, high_ctrl))
            print("[bundle] LocoPlugin loaded")

        if plugins_cfg.get("mic", {}).get("enabled", False) and media_ctrl is not None:
            from device import MicPlugin
            self._plugins.append(MicPlugin(plugins_cfg["mic"], namespace, executor, media_ctrl))
            print("[bundle] MicPlugin loaded")

        if plugins_cfg.get("speaker", {}).get("enabled", False) and media_ctrl is not None:
            from device import SpeakerPlugin
            self._plugins.append(SpeakerPlugin(plugins_cfg["speaker"], namespace, executor, media_ctrl))
            print("[bundle] SpeakerPlugin loaded")

        if plugins_cfg.get("camera", {}).get("enabled", False):
            from device import CameraPlugin
            self._plugins.append(CameraPlugin(plugins_cfg["camera"], namespace, executor))
            print("[bundle] CameraPlugin loaded")

        if plugins_cfg.get("motion_state", {}).get("enabled", False) and high_ctrl is not None:
            from device import MotionStatePlugin
            self._plugins.append(MotionStatePlugin(
                plugins_cfg["motion_state"], namespace, executor, high_ctrl))
            print("[bundle] MotionStatePlugin loaded")

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
            p.stop()
        print("[bundle] All plugins stopped")

    def get_all_tools(self) -> list:
        tools = []
        for p in self._plugins:
            if hasattr(p, 'get_tools'):
                tools.extend(p.get_tools())
            else:
                tools.append(p.get_tool())
        return tools

    def dispatch(self, tool_name: str, args: dict) -> dict | None:
        for p in self._plugins:
            plugin_tools = p.get_tools() if hasattr(p, 'get_tools') else [p.get_tool()]
            for tool_def in plugin_tools:
                if tool_def["name"] == tool_name:
                    if tool_def["type"] == "resource":
                        return p.dispatch(tool_name, args)
                    action = args.pop("action", tool_name)
                    args['_tool_name'] = tool_name
                    result = p.dispatch(action, args)
                    return result
        return None


# ── MCP HTTP server ───────────────────────────────────────────────────────────

_bundle: BumiDeviceBundle | None = None


def make_handler():
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, fmt, *args):
            msg = fmt % args
            if '"POST /mcp' in msg and '200' in msg:
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
                self._send(400, json.dumps({"jsonrpc": "2.0", "id": None,
                                             "error": {"code": -32700, "message": "Parse error"}}))
                return

            rid    = rpc.get("id")
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
                    ok({
                        "protocolVersion": "2024-11-05",
                        "capabilities": {"tools": {}},
                        "serverInfo": {"name": "bumi-driver", "version": "1.0.0"},
                    })
                elif method == "tools/list":
                    ok({"tools": _bundle.get_all_tools()})
                elif method == "tools/call":
                    name   = params.get("name", "")
                    args   = params.get("arguments") or {}
                    result = _bundle.dispatch(name, args)
                    if result is None:
                        err(-32601, f"Unknown tool: {name}")
                    else:
                        ok({"content": [{"type": "text", "text": json.dumps(result)}]})
                else:
                    err(-32601, f"Method not found: {method}")
            except Exception as e:
                import traceback
                traceback.print_exc()
                err(-32603, str(e))

    return Handler


# ── Entry point ───────────────────────────────────────────────────────────────


def _start_registration(mcp_port: int, name: str, category: str):
    """Register this driver with agent-core in a background thread, then heartbeat every 30s."""
    import urllib.request as _urllib
    import ssl as _ssl
    agent_core_url = os.environ.get("AGENT_CORE_URL", "https://localhost:15678")
    payload = json.dumps({
        "name": name,
        "url":  f"http://localhost:{mcp_port}/mcp",
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

    cfg       = _load_config()
    namespace = _resolve_namespace(cfg)
    mcp_port  = int(cfg.get("mcp_port", 15704))

    print(f"[bundle] namespace={namespace} mcp_port={mcp_port}")

    # ── Initialize Noetix SDK ──
    high_ctrl = None
    media_ctrl = None

    try:
        sys.path.insert(0, "/work/noetix_sdk_bumi/build")
        from highcontrol_py import HighController
        high_ctrl = HighController.instance()

        # init() may block indefinitely if robot is unreachable (GIL held in C++).
        # Use subprocess probe to check if DDS connection is possible first.
        probe_code = (
            "import sys; sys.path.insert(0, '/work/noetix_sdk_bumi/build'); "
            "from highcontrol_py import HighController; "
            "ctrl = HighController.instance(); ctrl.init(); "
            "print('OK')"
        )
        try:
            result = subprocess.run(
                [sys.executable, "-c", probe_code],
                capture_output=True, text=True, timeout=10,
            )
            if "OK" in result.stdout:
                # Probe succeeded, safe to init in main process
                high_ctrl.init()
                print("[bundle] HighController initialized")
            else:
                print(f"[bundle] WARNING: HighController probe failed: {result.stderr.strip()}")
                high_ctrl = None
        except subprocess.TimeoutExpired:
            print("[bundle] WARNING: HighController.init() timed out (robot unreachable?)")
            print("[bundle] MCP server will start without robot connection")
            high_ctrl = None
    except Exception as e:
        print(f"[bundle] WARNING: HighController init failed: {e}")
        high_ctrl = None

    if high_ctrl is not None:
        try:
            from mediacontrol_py import MediaController
            media_ctrl = MediaController.instance()

            probe_code2 = (
                "import sys; sys.path.insert(0, '/work/noetix_sdk_bumi/build'); "
                "from mediacontrol_py import MediaController; "
                "ctrl = MediaController.instance(); ctrl.init(); "
                "print('OK')"
            )
            result = subprocess.run(
                [sys.executable, "-c", probe_code2],
                capture_output=True, text=True, timeout=10,
            )
            if "OK" in result.stdout:
                media_ctrl.init()
                import time
                time.sleep(5)  # Wait for data sync per SDK docs
                print("[bundle] MediaController initialized")
            else:
                print(f"[bundle] MediaController probe failed: {result.stderr.strip()}")
                media_ctrl = None
        except subprocess.TimeoutExpired:
            print("[bundle] MediaController timed out")
            media_ctrl = None
        except Exception as e:
            print(f"[bundle] MediaController unavailable: {e}")
            media_ctrl = None

    # ── ROS2 ──
    rclpy.init()
    executor = rclpy.executors.MultiThreadedExecutor()

    _bundle = BumiDeviceBundle(cfg, namespace, executor, high_ctrl, media_ctrl)
    _bundle.start_all()

    def _spin():
        while rclpy.ok():
            executor.spin_once(timeout_sec=0.1)

    spin_thread = threading.Thread(target=_spin, daemon=True, name="bundle_spin")
    spin_thread.start()

    _start_registration(mcp_port, "bumi-driver", "driver")

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
