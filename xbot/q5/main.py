#!/usr/bin/env python3
"""
xbot/q5/main.py — Q5 轮式人形机器人驱动卡片统一入口。

读取 config.yaml，按插件配置加载插件，聚合成一个 MCP HTTP server 对外暴露。
驱动启动时自动 start 所有插件，关闭时自动 stop。

ROS2 Domain ID = 211（与 Q5 本体控制器统一）

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

class Q5DeviceBundle:
    def __init__(self, cfg: dict, namespace: str, executor):
        self._cfg = cfg
        self._plugins: list = []
        plugins_cfg = cfg.get("plugins", {})

        if plugins_cfg.get("state", {}).get("enabled", False):
            from device import StatePlugin
            self._plugins.append(StatePlugin(plugins_cfg["state"], namespace, executor))
            print("[bundle] StatePlugin loaded")

        if plugins_cfg.get("imu", {}).get("enabled", False):
            from device import ImuPlugin
            self._plugins.append(ImuPlugin(plugins_cfg["imu"], namespace, executor))
            print("[bundle] ImuPlugin loaded")

        if plugins_cfg.get("battery", {}).get("enabled", False):
            from device import BatteryPlugin
            self._plugins.append(BatteryPlugin(plugins_cfg["battery"], namespace, executor))
            print("[bundle] BatteryPlugin loaded")

        if plugins_cfg.get("faults", {}).get("enabled", False):
            from device import FaultsPlugin
            self._plugins.append(FaultsPlugin(plugins_cfg["faults"], namespace, executor))
            print("[bundle] FaultsPlugin loaded")

        if plugins_cfg.get("loco", {}).get("enabled", False):
            from device import LocoPlugin
            self._plugins.append(LocoPlugin(plugins_cfg["loco"], namespace, executor))
            print("[bundle] LocoPlugin loaded")

        if plugins_cfg.get("joint_servo", {}).get("enabled", False):
            from device import JointServoPlugin
            self._plugins.append(JointServoPlugin(plugins_cfg["joint_servo"], namespace, executor))
            print("[bundle] JointServoPlugin loaded")

        if plugins_cfg.get("hand", {}).get("enabled", False):
            from device import HandPlugin
            self._plugins.append(HandPlugin(plugins_cfg["hand"], namespace, executor))
            print("[bundle] HandPlugin loaded")

        if plugins_cfg.get("hand_state", {}).get("enabled", False):
            from device import HandStatePlugin
            self._plugins.append(HandStatePlugin(plugins_cfg["hand_state"], namespace, executor))
            print("[bundle] HandStatePlugin loaded")

        if plugins_cfg.get("head", {}).get("enabled", False):
            from device import HeadPlugin
            self._plugins.append(HeadPlugin(plugins_cfg["head"], namespace, executor))
            print("[bundle] HeadPlugin loaded")

        if plugins_cfg.get("head_gesture", {}).get("enabled", False):
            from device import HeadGesturePlugin
            self._plugins.append(HeadGesturePlugin(plugins_cfg["head_gesture"], namespace, executor))
            print("[bundle] HeadGesturePlugin loaded")

        if plugins_cfg.get("arm", {}).get("enabled", False):
            from device import ArmPlugin
            self._plugins.append(ArmPlugin(plugins_cfg["arm"], namespace, executor))
            print("[bundle] ArmPlugin loaded")

        if plugins_cfg.get("arm_gesture", {}).get("enabled", False):
            from device import ArmGesturePlugin
            self._plugins.append(ArmGesturePlugin(plugins_cfg["arm_gesture"], namespace, executor))
            print("[bundle] ArmGesturePlugin loaded")

        if plugins_cfg.get("motion", {}).get("enabled", False):
            from device import MotionPlugin
            self._plugins.append(MotionPlugin(plugins_cfg["motion"], namespace, executor))
            print("[bundle] MotionPlugin loaded")

        if plugins_cfg.get("gesture", {}).get("enabled", False):
            from device import GesturePlugin
            self._plugins.append(GesturePlugin(plugins_cfg["gesture"], namespace, executor))
            print("[bundle] GesturePlugin loaded")

        if plugins_cfg.get("audio", {}).get("enabled", False):
            from device import AudioPlugin
            self._plugins.append(AudioPlugin(plugins_cfg["audio"], namespace, executor))
            print("[bundle] AudioPlugin loaded")

        if plugins_cfg.get("speaker", {}).get("enabled", False):
            from device import SpeakerPlugin
            self._plugins.append(SpeakerPlugin(plugins_cfg["speaker"], namespace, executor))
            print("[bundle] SpeakerPlugin loaded")

        if plugins_cfg.get("led", {}).get("enabled", False):
            from device import LedPlugin
            self._plugins.append(LedPlugin(plugins_cfg["led"], namespace, executor))
            print("[bundle] LedPlugin loaded")

        if plugins_cfg.get("nav", {}).get("enabled", False):
            from device import NavPlugin
            self._plugins.append(NavPlugin(plugins_cfg["nav"], namespace, executor))
            print("[bundle] NavPlugin loaded")

        if plugins_cfg.get("teleop", {}).get("enabled", False):
            from device import TeleopPlugin
            self._plugins.append(TeleopPlugin(plugins_cfg["teleop"], namespace, executor))
            print("[bundle] TeleopPlugin loaded")

        if plugins_cfg.get("odom", {}).get("enabled", False):
            from device import OdomPlugin
            self._plugins.append(OdomPlugin(plugins_cfg["odom"], namespace, executor))
            print("[bundle] OdomPlugin loaded")

        if plugins_cfg.get("simple_actions", {}).get("enabled", False):
            from device import SimpleActionsPlugin
            self._plugins.append(SimpleActionsPlugin(plugins_cfg["simple_actions"], namespace, executor))
            print("[bundle] SimpleActionsPlugin loaded")

        if plugins_cfg.get("simple_trajectory", {}).get("enabled", False):
            from device import SimpleTrajectoryPlugin
            self._plugins.append(SimpleTrajectoryPlugin(plugins_cfg["simple_trajectory"], namespace, executor))
            print("[bundle] SimpleTrajectoryPlugin loaded")

        if plugins_cfg.get("behavior", {}).get("enabled", False):
            from device import BehaviorPlugin
            self._plugins.append(BehaviorPlugin(plugins_cfg["behavior"], namespace, executor))
            print("[bundle] BehaviorPlugin loaded")

        if plugins_cfg.get("grasp", {}).get("enabled", False):
            from device import GraspObjectPlugin
            self._plugins.append(GraspObjectPlugin(plugins_cfg["grasp"], namespace, executor))
            print("[bundle] GraspObjectPlugin loaded")

        if plugins_cfg.get("motion_action", {}).get("enabled", False):
            from device import MotionActionPlugin
            self._plugins.append(MotionActionPlugin(plugins_cfg["motion_action"], namespace, executor))
            print("[bundle] MotionActionPlugin loaded")

        if plugins_cfg.get("camera", {}).get("enabled", False):
            from device import CameraPlugin
            self._plugins.append(CameraPlugin(plugins_cfg["camera"], namespace, executor))
            print("[bundle] CameraPlugin loaded")

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
                    if tool_def.get("type") == "resource":
                        return p.dispatch(tool_name, args)
                    default_action = tool_def.get("default_action", "start")
                    action = args.pop("action", default_action)
                    args['_tool_name'] = tool_name
                    result = p.dispatch(action, args)
                    return result
        return None


# ── MCP HTTP server ───────────────────────────────────────────────────────────

_bundle: Q5DeviceBundle | None = None


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
                        "serverInfo": {"name": _bundle._cfg.get("name", "q5-device-bundle"), "version": "1.0.0"},
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
                        tool_result = {
                            "content": [{
                                "type": "text",
                                "text": json.dumps(result, ensure_ascii=False),
                            }],
                        }
                        if (isinstance(result, dict)
                                and (result.get("state") == "error"
                                     or "error" in result)):
                            tool_result["isError"] = True
                        ok(tool_result)
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
    mcp_port  = int(cfg.get("mcp_port", 15708))

    print(f"[bundle] namespace={namespace} mcp_port={mcp_port}")

    # ROS2 init — Domain ID 211 for Q5
    DOMAIN_ID = 211
    ctx = rclpy.context.Context()
    rclpy.init(context=ctx, domain_id=DOMAIN_ID)
    executor = rclpy.executors.MultiThreadedExecutor(context=ctx)
    print(f"[bundle] ROS2 initialized on domain {DOMAIN_ID}")

    _bundle = Q5DeviceBundle(cfg, namespace, executor)
    _bundle.start_all()

    def _spin():
        while rclpy.ok(context=ctx):
            executor.spin_once(timeout_sec=0.1)

    spin_thread = threading.Thread(target=_spin, daemon=True, name="bundle_spin")
    spin_thread.start()

    _start_registration(mcp_port, cfg.get("name", "XBot Q5"), "driver")

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
        rclpy.shutdown(context=ctx)


if __name__ == "__main__":
    main()
