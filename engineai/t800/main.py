#!/usr/bin/env python3
"""EngineAI T800 MCP driver entrypoint."""

from __future__ import annotations

import json
import os
import re
import signal
import socket
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import rclpy
import rclpy.executors
import yaml
from rclpy.context import Context


def _load_config() -> dict:
    path = os.environ.get("CONFIG_PATH", str(Path(__file__).parent / "config.yaml"))
    with open(path, encoding="utf-8") as handle:
        return yaml.safe_load(handle)


def _resolve_namespace(config: dict) -> str:
    value = str(config.get("ros_namespace", "")).strip() or socket.gethostname()
    return re.sub(r"[^a-zA-Z0-9_]", "_", value)


def _configure_cyclonedds(config: dict) -> str:
    interface = os.environ.get("NETWORK_INTERFACE") or str(config["ros"].get("robot_interface", "eth0"))
    if not re.fullmatch(r"[a-zA-Z0-9_.:-]+", interface):
        raise ValueError(f"invalid robot network interface: {interface}")
    os.environ.setdefault(
        "CYCLONEDDS_URI",
        "<CycloneDDS><Domain><General><Interfaces>"
        f"<NetworkInterface name='{interface}'/>"
        "</Interfaces></General></Domain></CycloneDDS>",
    )
    return interface


class DualDomainROS2:
    def __init__(self, robot_domain_id: int, core_domain_id: int):
        self.ctx_robot = Context()
        rclpy.init(context=self.ctx_robot, domain_id=robot_domain_id)
        self.executor_robot = rclpy.executors.MultiThreadedExecutor(context=self.ctx_robot)

        self.ctx_core = Context()
        rclpy.init(context=self.ctx_core, domain_id=core_domain_id)
        self.executor_core = rclpy.executors.MultiThreadedExecutor(context=self.ctx_core)
        self._threads: list[threading.Thread] = []

    def start_spin(self) -> None:
        def spin(executor, context, label):
            try:
                while rclpy.ok(context=context):
                    executor.spin_once(timeout_sec=0.1)
            except Exception as exc:
                print(f"[ros2] {label} executor stopped: {exc}", flush=True)

        for executor, context, label in (
            (self.executor_robot, self.ctx_robot, "robot"),
            (self.executor_core, self.ctx_core, "core"),
        ):
            thread = threading.Thread(target=spin, args=(executor, context, label), daemon=True)
            thread.start()
            self._threads.append(thread)

    def shutdown(self) -> None:
        self.executor_robot.shutdown()
        self.executor_core.shutdown()
        if rclpy.ok(context=self.ctx_robot):
            rclpy.shutdown(context=self.ctx_robot)
        if rclpy.ok(context=self.ctx_core):
            rclpy.shutdown(context=self.ctx_core)


class T800DeviceBundle:
    def __init__(self, config: dict, namespace: str, ros2: DualDomainROS2):
        from device import (
            DancePlugin,
            GesturePlugin,
            JointBridgePlugin,
            JointOverridePlugin,
            JointPlanPlugin,
            LedPlugin,
            LocomotionPlugin,
            MotionModePlugin,
            MotorPowerPlugin,
            NativeNodeControlPlugin,
            NativeSdkPlugin,
            SafetyControlPlugin,
            StatePlugin,
            TtsPlugin,
        )
        from virtual_gamepad import VirtualGamepadPlugin

        self._plugins: list = []
        plugins = config.get("plugins", {})

        state = StatePlugin(config, namespace, ros2)
        if plugins.get("state", {}).get("enabled", True):
            self._plugins.append(state)

        plugin_types = (
            ("locomotion", LocomotionPlugin, (config, namespace, ros2, state)),
            ("motion_mode", MotionModePlugin, (config, namespace, ros2, state)),
            ("joint_plan", JointPlanPlugin, (config, namespace, ros2, state)),
            ("joint_override", JointOverridePlugin, (config, namespace, ros2, state)),
            ("joint_bridge", JointBridgePlugin, (config, namespace, ros2, state)),
            ("led", LedPlugin, (config, namespace, ros2)),
            ("tts", TtsPlugin, (config, namespace, ros2)),
            ("motor_power", MotorPowerPlugin, (config, namespace, ros2)),
            ("native_node_control", NativeNodeControlPlugin, (config, namespace, ros2)),
            ("safety", SafetyControlPlugin, (config, namespace, ros2, state)),
        )
        instances = {}
        for key, cls, args in plugin_types:
            if plugins.get(key, {}).get("enabled", False):
                instance = cls(*args)
                instances[key] = instance
                self._plugins.append(instance)

        if plugins.get("dance", {}).get("enabled", True) and "motion_mode" in instances:
            instance = DancePlugin(instances["motion_mode"], state)
            instances["dance"] = instance
            self._plugins.append(instance)

        if plugins.get("gesture", {}).get("enabled", True) and "joint_plan" in instances:
            instance = GesturePlugin(instances["joint_plan"])
            instances["gesture"] = instance
            self._plugins.append(instance)

        virtual_gamepad_config = plugins.get("virtual_gamepad", {})
        if virtual_gamepad_config.get("enabled", False):
            instance = VirtualGamepadPlugin(virtual_gamepad_config, namespace, ros2)
            instances["virtual_gamepad"] = instance
            self._plugins.append(instance)

        if "safety" in instances:
            instances["safety"].set_controls(
                [
                    instances[key]
                    for key in ("locomotion", "joint_override", "joint_bridge", "virtual_gamepad", "gesture")
                    if key in instances
                ]
            )

        native_config = plugins.get("native_sdk", {})
        if native_config.get("enabled", False):
            self._plugins.append(NativeSdkPlugin(native_config, namespace, ros2))

    def start_all(self) -> None:
        for plugin in self._plugins:
            try:
                plugin.start()
                print(f"[bundle] {type(plugin).__name__} started", flush=True)
            except Exception as exc:
                print(f"[bundle] {type(plugin).__name__} start failed: {exc}", flush=True)

    def stop_all(self) -> None:
        for plugin in reversed(self._plugins):
            try:
                plugin.stop()
            except Exception as exc:
                print(f"[bundle] {type(plugin).__name__} stop failed: {exc}", flush=True)

    def get_all_tools(self) -> list[dict]:
        tools: list[dict] = []
        for plugin in self._plugins:
            tools.extend(plugin.get_tools() if hasattr(plugin, "get_tools") else [plugin.get_tool()])
        return tools

    def dispatch(self, tool_name: str, arguments: dict) -> dict | None:
        for plugin in self._plugins:
            tools = plugin.get_tools() if hasattr(plugin, "get_tools") else [plugin.get_tool()]
            for definition in tools:
                if definition["name"] != tool_name:
                    continue
                args = dict(arguments)
                if definition["type"] == "resource":
                    return plugin.dispatch(tool_name, args)
                action = args.pop("action", tool_name)
                args["_tool_name"] = tool_name
                return plugin.dispatch(action, args)
        return None


_bundle: T800DeviceBundle | None = None


def make_handler():
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, fmt, *args):
            msg = fmt % args
            if '"POST /mcp' not in msg or "200" not in msg:
                print(f"[mcp] {self.address_string()} {msg}")

        def _send_json(self, status: int, payload: dict) -> None:
            body = json.dumps(payload, ensure_ascii=False).encode()
            self.send_response(status)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Access-Control-Allow-Origin", "*")
            self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
            self.send_header("Access-Control-Allow-Headers", "Content-Type, Accept")
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            if self.path == "/health":
                self._send_json(200, {"state": "running", "driver": "engineai-t800"})
            else:
                self._send_json(404, {"error": "not found"})

        def do_OPTIONS(self):
            self.send_response(204)
            self.send_header("Access-Control-Allow-Origin", "*")
            self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
            self.send_header("Access-Control-Allow-Headers", "Content-Type, Accept")
            self.end_headers()

        def do_POST(self):
            if self.path != "/mcp":
                self._send_json(404, {"error": "not found"})
                return
            length = int(self.headers.get("Content-Length", 0))
            try:
                rpc = json.loads(self.rfile.read(length))
            except Exception:
                self._send_json(400, {"jsonrpc": "2.0", "id": None,
                                      "error": {"code": -32700, "message": "Parse error"}})
                return
            request_id = rpc.get("id")
            if request_id is None:
                self.send_response(202)
                self.end_headers()
                return

            def ok(result):
                self._send_json(200, {"jsonrpc": "2.0", "id": request_id, "result": result})

            def error(code, message):
                self._send_json(200, {"jsonrpc": "2.0", "id": request_id,
                                      "error": {"code": code, "message": message}})

            method = rpc.get("method", "")
            params = rpc.get("params") or {}
            try:
                if method == "initialize":
                    ok({"protocolVersion": "2024-11-05", "capabilities": {"tools": {}},
                        "serverInfo": {"name": "engineai-t800-device-bundle", "version": "1.0.0"}})
                elif method == "tools/list":
                    ok({"tools": _bundle.get_all_tools()})
                elif method == "tools/call":
                    name = params.get("name", "")
                    result = _bundle.dispatch(name, params.get("arguments") or {})
                    if result is None:
                        error(-32601, f"Unknown tool: {name}")
                    else:
                        ok({"content": [{"type": "text", "text": json.dumps(result, ensure_ascii=False)}]})
                else:
                    error(-32601, f"Method not found: {method}")
            except (TypeError, ValueError) as exc:
                error(-32602, str(exc))
            except Exception as exc:
                import traceback
                traceback.print_exc()
                error(-32603, str(exc))

    return Handler


def _start_registration(port: int, config: dict) -> None:
    import ssl
    import time
    import urllib.request

    agent_core_url = os.environ.get("AGENT_CORE_URL", "https://localhost:15678")
    payload = json.dumps({
        "id": "engineai-t800-driver",
        "name": config.get("name", "EngineAI T800 Development Edition"),
        "url": f"http://localhost:{port}/mcp",
        "transport": "http",
        "category": "driver",
    }).encode()
    context = ssl.create_default_context()
    context.check_hostname = False
    context.verify_mode = ssl.CERT_NONE

    def register_loop():
        while True:
            try:
                request = urllib.request.Request(
                    f"{agent_core_url}/api/mcp", data=payload,
                    headers={"Content-Type": "application/json"}, method="POST"
                )
                with urllib.request.urlopen(request, timeout=3, context=context):
                    pass
                time.sleep(30)
            except Exception as exc:
                print(f"[register] failed: {exc}; retrying in 5s", flush=True)
                time.sleep(5)

    threading.Thread(target=register_loop, daemon=True, name="register").start()


def main() -> None:
    global _bundle
    config = _load_config()
    namespace = _resolve_namespace(config)
    port = int(config.get("mcp_port", 15708))
    robot_domain = int(config["ros"].get("robot_domain_id", 69))
    core_domain = int(config["ros"].get("core_domain_id", 42))
    interface = _configure_cyclonedds(config)
    print(f"[bundle] namespace={namespace} domains={robot_domain}->{core_domain} port={port} interface={interface}")

    ros2 = DualDomainROS2(robot_domain, core_domain)
    ros2.start_spin()
    _bundle = T800DeviceBundle(config, namespace, ros2)
    _bundle.start_all()
    _start_registration(port, config)

    server = ThreadingHTTPServer(("", port), make_handler())
    print(f"[bundle] MCP server http://localhost:{port}/mcp", flush=True)

    stopping = threading.Event()

    def shutdown(signum, _frame):
        if stopping.is_set():
            return
        stopping.set()
        print(f"[bundle] signal {signum}; stopping", flush=True)
        _bundle.stop_all()
        threading.Thread(target=server.shutdown, daemon=True).start()

    signal.signal(signal.SIGTERM, shutdown)
    signal.signal(signal.SIGINT, shutdown)
    try:
        server.serve_forever()
    finally:
        _bundle.stop_all()
        ros2.shutdown()


if __name__ == "__main__":
    main()
