#!/usr/bin/env python3
"""
drivers/booster/k1/main.py — Booster K1 设备 bundle 统一入口。

读取 config.yaml，按插件配置加载插件，聚合成一个 MCP HTTP server 对外暴露。
驱动启动时自动 start 所有插件，关闭时自动 stop。

与 unitree/g1 的差异：K1 只有一个官方 SDK 入口 —— boosteros.robots.booster.BoosterRobot。
官方文档明确要求同一台机器人只创建一个实例（每个实例各开一条 ROS 节点/订阅/控制发布通道，
多开会导致指令冲突），所以这里构造单个 `robot`，所有插件共享它，而不是像 G1 那样为每个
能力分别持有一个 SDK client。

用法：
    python3 main.py

环境变量：
    CONFIG_PATH — config.yaml 路径（默认同目录下）
"""

from __future__ import annotations

try:
    from common import logsafe
    logsafe.install()
except ImportError as _e:  # running outside the container image
    import sys as _sys
    _sys.stderr.write(f"[bundle] logsafe unavailable ({_e}); stdout unprotected\n")

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

class K1DeviceBundle:
    def __init__(self, cfg: dict, namespace: str, executor, robot):
        self._plugins: list = []
        plugins_cfg = cfg.get("plugins", {})

        if plugins_cfg.get("state", {}).get("enabled", False):
            from device import StatePlugin
            self._plugins.append(StatePlugin(plugins_cfg["state"], namespace, executor, robot))
            print("[bundle] StatePlugin loaded")

        if plugins_cfg.get("camera", {}).get("enabled", False):
            from device import CameraPlugin
            self._plugins.append(CameraPlugin(plugins_cfg["camera"], namespace, executor, robot))
            print("[bundle] CameraPlugin loaded")

        if plugins_cfg.get("loco", {}).get("enabled", False):
            from device import LocoPlugin
            self._plugins.append(LocoPlugin(plugins_cfg["loco"], namespace, executor, robot))
            print("[bundle] LocoPlugin loaded")

        if plugins_cfg.get("upper_body", {}).get("enabled", False):
            from device import UpperBodyPlugin
            self._plugins.append(UpperBodyPlugin(plugins_cfg["upper_body"], namespace, executor, robot))
            print("[bundle] UpperBodyPlugin loaded")

        if plugins_cfg.get("action", {}).get("enabled", False):
            from device import ActionPlugin
            self._plugins.append(ActionPlugin(plugins_cfg["action"], namespace, executor, robot))
            print("[bundle] ActionPlugin loaded")

        if plugins_cfg.get("audio", {}).get("enabled", False):
            from device import AudioPlugin
            self._plugins.append(AudioPlugin(plugins_cfg["audio"], namespace, executor, robot))
            print("[bundle] AudioPlugin loaded")

        if plugins_cfg.get("speech", {}).get("enabled", False):
            from device import SpeechPlugin
            self._plugins.append(SpeechPlugin(plugins_cfg["speech"], namespace, executor, robot))
            print("[bundle] SpeechPlugin loaded")

        if plugins_cfg.get("detection", {}).get("enabled", False):
            from device import DetectionPlugin
            self._plugins.append(DetectionPlugin(plugins_cfg["detection"], namespace, executor, robot))
            print("[bundle] DetectionPlugin loaded")

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
                    args['_tool_name'] = tool_name  # let multi-tool plugins know which tool was called
                    result = p.dispatch(action, args)
                    return result
        return None


# ── MCP HTTP server ───────────────────────────────────────────────────────────

_bundle: K1DeviceBundle | None = None


def make_handler():
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, fmt, *args):
            msg = fmt % args
            if '"POST /mcp' in msg and '200' in msg:
                return
            # Escape and cap: msg embeds the raw request line, which on host
            # networking is remote-controlled bytes going straight into the
            # Docker log framer (log injection / control-byte corruption).
            safe = msg.encode("unicode_escape").decode("ascii")[:200]
            print(f"[mcp] {self.address_string()} {safe}")

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
                        "serverInfo": {"name": "k1-device-bundle", "version": "1.0.0"},
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
                    pass  # heartbeat ok, suppress log
                _t.sleep(30)
            except Exception as e:
                print(f"[register] failed: {e}, retrying in 5s")
                _t.sleep(5)
    threading.Thread(target=_run, daemon=True, name="register").start()


def _connect_robot(cfg: dict):
    """Construct the single shared BoosterRobot instance.

    boosteros raises LocoClientInitError when no robot/virtual-robot is
    discoverable within `timeout`. Real deploys can start this container
    before the robot's motion service is up, and this environment has no
    K1/Booster Studio reachable at all — either way the MCP server must still
    come up and answer tools/list, just with every plugin reporting a
    disconnected state instead of crashing the process.
    """
    from boosteros.robots.booster import BoosterRobot
    try:
        robot = BoosterRobot(
            timeout=float(cfg.get("connect_timeout", 5.0)),
            callback_workers=int(cfg.get("callback_workers", 4)),
        )
        print(f"[bundle] BoosterRobot connected: {robot.robot_info}")
        return robot
    except Exception as e:
        print(f"[bundle] BoosterRobot connect FAILED ({e}); "
              f"plugins will start in a disconnected state", flush=True)
        return None


def main():
    global _bundle

    cfg       = _load_config()
    namespace = _resolve_namespace(cfg)
    mcp_port  = int(cfg.get("mcp_port", 15705))

    print(f"[bundle] namespace={namespace} mcp_port={mcp_port}")

    robot = _connect_robot(cfg)

    # ROS2 — used only for this driver's own outbound topics (imu/battery/
    # joints/...); boosteros manages its own internal rclpy node/executor for
    # talking to the robot.
    rclpy.init()
    executor = rclpy.executors.MultiThreadedExecutor()

    _bundle = K1DeviceBundle(cfg, namespace, executor, robot)
    _bundle.start_all()

    def _spin():
        while rclpy.ok():
            executor.spin_once(timeout_sec=0.1)

    spin_thread = threading.Thread(target=_spin, daemon=True, name="bundle_spin")
    spin_thread.start()

    _start_registration(mcp_port, cfg.get("name", "Booster K1 Bundle"), "driver")

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
