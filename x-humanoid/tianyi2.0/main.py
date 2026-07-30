#!/usr/bin/env python3
"""
x-humanoid/tianyi2.0/main.py — 天轶2.0 Pro 设备 bundle 统一入口。

读取 config.yaml，按插件配置加载插件，聚合成一个 MCP HTTP server 对外暴露。
驱动启动时自动 start 所有插件，关闭时自动 stop。

双 Domain 模式：
  - domain 0: 订阅/发布到天轶本体控制器 (192.168.41.1)
  - domain 42: 发布传感器数据给 Agent Core

用法：
    python3 main.py

环境变量：
    CONFIG_PATH — config.yaml 路径（默认同目录下）
    SLAMTEC_URL — Slamtec底盘API地址（默认 http://192.168.11.1:1448）
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

import rclpy
import rclpy.executors
from rclpy.context import Context


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


# ── Dual Domain ROS2 Init ────────────────────────────────────────────────────

class DualDomainROS2:
    """Manages two ROS2 contexts: domain 0 (tianyi) and domain 42 (agent-core)."""

    def __init__(self):
        # Domain 0: connect to tianyi body controller
        self.ctx_tianyi = Context()
        rclpy.init(context=self.ctx_tianyi, domain_id=0)
        self.executor_tianyi = rclpy.executors.MultiThreadedExecutor(context=self.ctx_tianyi)

        # Domain 42: publish to agent-core
        self.ctx_core = Context()
        rclpy.init(context=self.ctx_core, domain_id=42)
        self.executor_core = rclpy.executors.MultiThreadedExecutor(context=self.ctx_core)

        self._spin_threads = []

    def start_spin(self):
        """Start spinning both executors in background threads."""
        def _spin(executor, name):
            try:
                while rclpy.ok(context=executor.context):
                    executor.spin_once(timeout_sec=0.1)
            except Exception:
                pass
            print(f"[ros2] {name} spin exited")

        t1 = threading.Thread(target=_spin, args=(self.executor_tianyi, "domain0"), daemon=True)
        t2 = threading.Thread(target=_spin, args=(self.executor_core, "domain42"), daemon=True)
        t1.start()
        t2.start()
        self._spin_threads = [t1, t2]

    def shutdown(self):
        self.executor_tianyi.shutdown()
        self.executor_core.shutdown()
        rclpy.shutdown(context=self.ctx_tianyi)
        rclpy.shutdown(context=self.ctx_core)


# ── Bundle ────────────────────────────────────────────────────────────────────

class TianyiDeviceBundle:
    def __init__(self, cfg: dict, namespace: str, ros2: DualDomainROS2, slamtec_client):
        self._plugins: list = []
        plugins_cfg = cfg.get("plugins", {})

        if plugins_cfg.get("state", {}).get("enabled", False):
            from device import StatePlugin
            self._plugins.append(StatePlugin(plugins_cfg["state"], namespace, ros2))
            print("[bundle] StatePlugin loaded")

        if plugins_cfg.get("camera", {}).get("enabled", False):
            from device import CameraPlugin
            self._plugins.append(CameraPlugin(plugins_cfg["camera"], namespace, ros2))
            print("[bundle] CameraPlugin loaded")

        for config_name, class_name in (("imu", "ImuPlugin"),
                                        ("camera_depth", "DepthCameraPlugin"),
                                        ("camera_pointcloud", "PointCloudPlugin")):
            if plugins_cfg.get(config_name, {}).get("enabled", False):
                from device import ImuPlugin, DepthCameraPlugin, PointCloudPlugin
                plugin_class = {"ImuPlugin": ImuPlugin, "DepthCameraPlugin": DepthCameraPlugin,
                                "PointCloudPlugin": PointCloudPlugin}[class_name]
                self._plugins.append(plugin_class(plugins_cfg[config_name], namespace, ros2))
                print(f"[bundle] {class_name} loaded")

        if plugins_cfg.get("asr", {}).get("enabled", False):
            from device import AsrPlugin
            self._plugins.append(AsrPlugin(plugins_cfg["asr"], namespace, ros2))
            print("[bundle] AsrPlugin loaded")

        if plugins_cfg.get("nav_state", {}).get("enabled", False):
            from device import NavStatePlugin
            self._plugins.append(NavStatePlugin(plugins_cfg["nav_state"], namespace, ros2, slamtec_client))
            print("[bundle] NavStatePlugin loaded")

        if plugins_cfg.get("head", {}).get("enabled", False):
            from device import HeadPlugin
            self._plugins.append(HeadPlugin(plugins_cfg["head"], namespace, ros2))
            print("[bundle] HeadPlugin loaded")

        if plugins_cfg.get("arm", {}).get("enabled", False):
            from device import ArmPlugin
            self._plugins.append(ArmPlugin(plugins_cfg["arm"], namespace, ros2))
            print("[bundle] ArmPlugin loaded")

        if plugins_cfg.get("waist", {}).get("enabled", False):
            from device import WaistPlugin
            self._plugins.append(WaistPlugin(plugins_cfg["waist"], namespace, ros2))
            print("[bundle] WaistPlugin loaded")

        if plugins_cfg.get("leg", {}).get("enabled", False):
            from device import LegPlugin
            self._plugins.append(LegPlugin(plugins_cfg["leg"], namespace, ros2))
            print("[bundle] LegPlugin loaded")

        if plugins_cfg.get("hand", {}).get("enabled", False):
            from device import HandPlugin
            self._plugins.append(HandPlugin(plugins_cfg["hand"], namespace, ros2))
            print("[bundle] HandPlugin loaded")

        if plugins_cfg.get("tts", {}).get("enabled", False):
            from device import TtsPlugin
            self._plugins.append(TtsPlugin(plugins_cfg["tts"], namespace, ros2))
            print("[bundle] TtsPlugin loaded")

        if plugins_cfg.get("nav", {}).get("enabled", False):
            from device import NavPlugin
            self._plugins.append(NavPlugin(plugins_cfg["nav"], namespace, ros2, slamtec_client))
            print("[bundle] NavPlugin loaded")

        if plugins_cfg.get("chat", {}).get("enabled", False):
            from device import ChatPlugin
            self._plugins.append(ChatPlugin(plugins_cfg["chat"], namespace, ros2))
            print("[bundle] ChatPlugin loaded")

        if plugins_cfg.get("lyre_event", {}).get("enabled", False):
            from device import LyreEventPlugin
            self._plugins.append(LyreEventPlugin(plugins_cfg["lyre_event"], namespace, ros2))
            print("[bundle] LyreEventPlugin loaded")

        if plugins_cfg.get("llm", {}).get("enabled", False):
            from device import LlmPlugin
            self._plugins.append(LlmPlugin(plugins_cfg["llm"], namespace, ros2))
            print("[bundle] LlmPlugin loaded")

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

_bundle: TianyiDeviceBundle | None = None
_server_name = "tianyi2-device-bundle"


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
                        "serverInfo": {"name": _server_name, "version": "1.0.0"},
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
    global _bundle, _server_name

    cfg       = _load_config()
    namespace = _resolve_namespace(cfg)
    mcp_port  = int(cfg.get("mcp_port", 15708))
    _server_name = cfg.get("mcp_server_name", "tianyi2-device-bundle")

    print(f"[bundle] namespace={namespace} mcp_port={mcp_port}")

    # Slamtec HTTP client
    slamtec_url = os.environ.get("SLAMTEC_URL", cfg.get("slamtec", {}).get("base_url", "http://192.168.11.1:1448"))
    from nav_client import SlamtecClient
    slamtec_client = SlamtecClient(slamtec_url)
    print(f"[bundle] Slamtec client → {slamtec_url}")

    # Dual-domain ROS2
    ros2 = DualDomainROS2()
    ros2.start_spin()
    print("[bundle] Dual-domain ROS2 initialized (domain 0 + domain 42)")

    _bundle = TianyiDeviceBundle(cfg, namespace, ros2, slamtec_client)
    _bundle.start_all()

    _start_registration(mcp_port, cfg.get("name", "Tianyi 2.0 Pro"), "driver")

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
        ros2.shutdown()


if __name__ == "__main__":
    main()
