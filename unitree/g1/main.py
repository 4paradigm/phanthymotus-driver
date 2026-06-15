#!/usr/bin/env python3
"""
drivers/unitree/g1/main.py — Unitree G1 设备 bundle 统一入口。

读取 config.yaml，按插件配置加载插件，聚合成一个 MCP HTTP server 对外暴露。
驱动启动时自动 start 所有插件，关闭时自动 stop。

MCP 工具命名规则：直接使用 tool name（mic, tts, led, loco, loco_state, arm, state）

用法：
    python3 main.py <networkInterface>

环境变量：
    CONFIG_PATH — config.yaml 路径（默认同目录下）
"""

import json
import os
import re
import signal
import socket
import sys
import threading
from http.server import BaseHTTPRequestHandler, HTTPServer
from pathlib import Path

import yaml

import rclpy
import rclpy.executors

from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.g1.audio.g1_audio_client import AudioClient
from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient
from unitree_sdk2py.g1.arm.g1_arm_action_client import G1ArmActionClient
from unitree_sdk2py.g1.slam.slam_client import SlamClient
from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import MotionSwitcherClient


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

class G1DeviceBundle:
    def __init__(self, cfg: dict, namespace: str, executor,
                 audio_client: AudioClient,
                 loco_client: LocoClient,
                 arm_client: G1ArmActionClient,
                 slam_client: SlamClient,
                 msc_client: MotionSwitcherClient):
        self._plugins: list = []
        plugins_cfg = cfg.get("plugins", {})

        if plugins_cfg.get("mic", {}).get("enabled", False):
            from device import MicPlugin
            self._plugins.append(MicPlugin(plugins_cfg["mic"], namespace, executor))
            print("[bundle] MicPlugin loaded")

        if plugins_cfg.get("tts", {}).get("enabled", False):
            from device import NativeTtsPlugin
            self._plugins.append(NativeTtsPlugin(plugins_cfg["tts"], namespace, executor, audio_client))
            print("[bundle] NativeTtsPlugin loaded")

        if plugins_cfg.get("speaker", {}).get("enabled", False):
            from device import SpeakerPlugin
            self._plugins.append(SpeakerPlugin(plugins_cfg["speaker"], namespace, executor, audio_client))
            print("[bundle] SpeakerPlugin loaded")

        if plugins_cfg.get("led", {}).get("enabled", False):
            from device import LedPlugin
            self._plugins.append(LedPlugin(plugins_cfg["led"], namespace, executor, audio_client))
            print("[bundle] LedPlugin loaded")

        if plugins_cfg.get("loco", {}).get("enabled", False):
            from device import LocoStatePlugin, LocoPlugin
            self._plugins.append(LocoStatePlugin(plugins_cfg["loco"], namespace, executor))
            self._plugins.append(LocoPlugin(plugins_cfg["loco"], namespace, executor, loco_client))
            print("[bundle] LocoStatePlugin + LocoPlugin loaded")

        if plugins_cfg.get("arm", {}).get("enabled", False):
            from device import ArmActionPlugin
            self._plugins.append(ArmActionPlugin(plugins_cfg["arm"], namespace, executor, arm_client))
            print("[bundle] ArmActionPlugin loaded")

        if plugins_cfg.get("asr", {}).get("enabled", False):
            from device import AsrPlugin
            self._plugins.append(AsrPlugin(plugins_cfg["asr"], namespace, executor))
            print("[bundle] AsrPlugin loaded")

        if plugins_cfg.get("state", {}).get("enabled", False):
            from device import StatePlugin
            self._plugins.append(StatePlugin(plugins_cfg["state"], namespace, executor))
            print("[bundle] StatePlugin loaded")

        if plugins_cfg.get("camera", {}).get("enabled", False):
            from device import RealSensePlugin
            self._plugins.append(RealSensePlugin(plugins_cfg["camera"], namespace, executor))
            print("[bundle] RealSensePlugin loaded")

        if plugins_cfg.get("lidar", {}).get("enabled", False):
            from device import LidarPlugin
            self._plugins.append(LidarPlugin(plugins_cfg["lidar"], namespace, executor))
            print("[bundle] LidarPlugin loaded")

        if plugins_cfg.get("slam", {}).get("enabled", False):
            from device import SpatialPlugin
            self._plugins.append(SpatialPlugin(plugins_cfg["slam"], namespace, executor, slam_client))
            print("[bundle] SpatialPlugin loaded")

        if plugins_cfg.get("motion_switcher", {}).get("enabled", False):
            from device import MotionSwitcherPlugin
            self._plugins.append(MotionSwitcherPlugin(plugins_cfg["motion_switcher"], namespace, executor, msc_client))
            print("[bundle] MotionSwitcherPlugin loaded")

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
                    if tool_def["type"] == "sensor":
                        return {"info": "Sensor is active, data flows via ROS2 topic. No callable actions."}
                    if tool_def["type"] == "resource":
                        return p.dispatch(tool_name, args)
                    action = args.pop("action", tool_name)
                    return p.dispatch(action, args)
        return None


# ── MCP HTTP server ───────────────────────────────────────────────────────────

_bundle: G1DeviceBundle | None = None


def make_handler():
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
                        "serverInfo": {"name": "g1-device-bundle", "version": "2.0.0"},
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
    agent_core_url = os.environ.get("AGENT_CORE_URL", "http://localhost:15678")
    payload = json.dumps({
        "name": name,
        "url":  f"http://localhost:{mcp_port}/mcp",
        "category": category,
    }).encode()
    def _run():
        import time as _t
        while True:
            try:
                req = _urllib.Request(
                    f"{agent_core_url}/api/mcp", data=payload,
                    headers={"Content-Type": "application/json"}, method="POST",
                )
                with _urllib.urlopen(req, timeout=3):
                    print(f"[register] heartbeat ok → {agent_core_url}")
                _t.sleep(30)
            except Exception as e:
                print(f"[register] failed: {e}, retrying in 5s")
                _t.sleep(5)
    threading.Thread(target=_run, daemon=True, name="register").start()


def main():
    global _bundle

    if len(sys.argv) < 2:
        print(f"Usage: python3 {sys.argv[0]} <networkInterface>")
        sys.exit(1)

    network_iface = sys.argv[1]
    cfg           = _load_config()
    namespace     = _resolve_namespace(cfg)
    mcp_port      = int(cfg.get("mcp_port", 15701))

    print(f"[bundle] namespace={namespace} mcp_port={mcp_port}")

    # DDS init
    ChannelFactoryInitialize(0, network_iface)
    print(f"[bundle] DDS initialized on interface: {network_iface}")

    # AudioClient (shared by tts + led)
    audio_client = AudioClient()
    audio_client.SetTimeout(10.0)
    audio_client.Init()
    print("[bundle] AudioClient ready")

    # LocoClient (locomotion control)
    loco_client = LocoClient()
    loco_client.SetTimeout(10.0)
    loco_client.Init()
    print("[bundle] LocoClient ready")

    # G1ArmActionClient (arm gestures)
    arm_client = G1ArmActionClient()
    arm_client.SetTimeout(10.0)
    arm_client.Init()
    print("[bundle] G1ArmActionClient ready")

    # SlamClient (SLAM navigation)
    slam_client = SlamClient()
    slam_client.SetTimeout(5.0)
    slam_client.Init()
    print("[bundle] SlamClient ready")

    # MotionSwitcherClient
    msc_client = MotionSwitcherClient()
    msc_client.SetTimeout(5.0)
    msc_client.Init()
    print("[bundle] MotionSwitcherClient ready")

    # ROS2
    rclpy.init()
    executor = rclpy.executors.MultiThreadedExecutor()

    _bundle = G1DeviceBundle(cfg, namespace, executor, audio_client, loco_client, arm_client, slam_client, msc_client)
    _bundle.start_all()

    def _spin():
        while rclpy.ok():
            executor.spin_once(timeout_sec=0.1)

    spin_thread = threading.Thread(target=_spin, daemon=True, name="bundle_spin")
    spin_thread.start()

    _start_registration(mcp_port, cfg.get("name", "Unitree G1"), "driver")

    server = HTTPServer(("", mcp_port), make_handler())
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
