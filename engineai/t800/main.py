#!/usr/bin/env python3
"""EngineAI T800 MCP driver entrypoint."""

from __future__ import annotations

# Make every log line one atomic, control-character-free write, so concurrent
# writers cannot tear a Docker log record. Must run before anything prints.
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
import threading
import time
import uuid
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse

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
    namespace = re.sub(r"[^a-zA-Z0-9_]", "_", value).strip("_")
    if not namespace:
        return "t800"
    if not re.match(r"[a-zA-Z_]", namespace):
        namespace = f"t800_{namespace}"
    return namespace


def _configure_cyclonedds(config: dict) -> str:
    interface = os.environ.get("NETWORK_INTERFACE") or str(config["ros"].get("robot_interface", "eth1"))
    if not re.fullmatch(r"[a-zA-Z0-9_.:-]+", interface):
        raise ValueError(f"invalid robot network interface: {interface}")
    available = {name for _index, name in socket.if_nameindex()}
    if available and interface not in available:
        names = ", ".join(sorted(available))
        raise ValueError(
            f"robot network interface {interface!r} does not exist; available interfaces: {names}"
        )
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
    _MOTION_OUTPUT_TOOLS = frozenset({
        "loco", "motion_mode", "dance", "joint_plan", "gesture",
        "joint_override", "joint_bridge", "virtual_gamepad", "gait",
        "motion_recorder", "head", "speaker",
    })
    _SAFE_WHILE_MOTION_SETTLING = frozenset({
        "start", "stop", "info", "status", "list", "stop_move",
        "stop_dance", "stop_gesture", "stop_playback", "cancel",
        "release", "stop_command", "record_stop",
    })

    def __init__(self, config: dict, namespace: str, ros2: DualDomainROS2):
        from device import (
            DancePlugin,
            GaitPlugin,
            GesturePlugin,
            HeadActuatorPlugin,
            JointBridgePlugin,
            JointOverridePlugin,
            JointPlanPlugin,
            HeartbeatStatusPlugin,
            LedPlugin,
            LocomotionPlugin,
            ControlledSpatialPlugin,
            MotionCommandTracePlugin,
            MotionEventsPlugin,
            MotionInterruptGroup,
            MotionRecorderPlugin,
            MicPlugin,
            MotionModePlugin,
            MotorPowerPlugin,
            NativeInterfaceProbePlugin,
            NativeNodeControlPlugin,
            NativeSdkPlugin,
            SafetyControlPlugin,
            SpeakerPlugin,
            StatePlugin,
            TtsPlugin,
            VisionPlugin,
            _t800_acp_preflight,
            _t800_acp_status,
        )
        from virtual_gamepad import VirtualGamepadPlugin

        self._plugins: list = []
        self._active_plugins: list = []
        self._startup_errors: dict[str, str] = {}
        self._started = False
        self._acp_status = _t800_acp_status
        acp_status = _t800_acp_preflight()
        if acp_status["state"] == "error":
            error = acp_status["last_error"]
            print(
                f"[bundle] ACP configuration error: {error}",
                flush=True,
            )
        plugins = config.get("plugins", {})
        motion_interrupt_group = MotionInterruptGroup()
        self._motion_interrupt_group = motion_interrupt_group

        motion_events = None
        if plugins.get("motion_events", {}).get("enabled", False):
            motion_events = MotionEventsPlugin(config, namespace, ros2)
        self._motion_events = motion_events

        state = StatePlugin(config, namespace, ros2, motion_events=motion_events)
        if plugins.get("state", {}).get("enabled", True):
            self._plugins.append(state)
        if motion_events is not None:
            self._plugins.append(motion_events)

        plugin_types = (
            ("heartbeat_status", HeartbeatStatusPlugin, (config, namespace, ros2)),
            ("motion_command_trace", MotionCommandTracePlugin, (config, namespace, ros2)),
            ("native_interface_probe", NativeInterfaceProbePlugin, (config, namespace, ros2)),
            ("locomotion", LocomotionPlugin, (config, namespace, ros2, state)),
            ("motion_mode", MotionModePlugin, (config, namespace, ros2, state)),
            ("joint_plan", JointPlanPlugin, (config, namespace, ros2, state)),
            ("joint_override", JointOverridePlugin, (config, namespace, ros2, state)),
            ("joint_bridge", JointBridgePlugin, (config, namespace, ros2, state)),
            ("led", LedPlugin, (config, namespace, ros2)),
            ("tts", TtsPlugin, (config, namespace, ros2)),
            ("mic", MicPlugin, (config, namespace, ros2)),
            ("speaker", SpeakerPlugin, (config, namespace, ros2)),
            ("vision", VisionPlugin, (config, namespace, ros2)),
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

        for key in ("mic", "vision"):
            provider = instances.get(key)
            if provider is not None and hasattr(provider, "health_sources"):
                state.register_health_provider(key, provider.health_sources)

        locomotion = instances.get("locomotion")
        if locomotion is not None:
            locomotion.set_interrupt_group(motion_interrupt_group)
            motion_interrupt_group.register(
                "locomotion", locomotion.halt, locomotion.motion_active
            )
        speaker = instances.get("speaker")
        if speaker is not None:
            speaker.set_interrupt_group(motion_interrupt_group)
            motion_interrupt_group.register(
                "speaker", speaker.halt, speaker.motion_active
            )

        if plugins.get("dance", {}).get("enabled", True) and "motion_mode" in instances:
            instance = DancePlugin(instances["motion_mode"], state)
            instances["dance"] = instance
            self._plugins.append(instance)

        if plugins.get("gesture", {}).get("enabled", True) and "joint_plan" in instances:
            instance = GesturePlugin(instances["joint_plan"])
            instance.set_interrupt_group(motion_interrupt_group)
            motion_interrupt_group.register(
                "gesture", instance.halt, instance.motion_active
            )
            instances["gesture"] = instance
            self._plugins.append(instance)

        # Gait selector — delegates to the public Native SDK motion-state API.
        if (
            plugins.get("gait", {}).get("enabled", False)
            and "motion_mode" in instances
        ):
            instance = GaitPlugin(config, instances["motion_mode"], state)
            instances["gait"] = instance
            self._plugins.append(instance)

        # Motion recorder — record and replay joint trajectories
        motion_recorder = None
        if plugins.get("motion_recorder", {}).get("enabled", False):
            motion_recorder = MotionRecorderPlugin(config, namespace, ros2)
            instances["motion_recorder"] = motion_recorder
            self._plugins.append(motion_recorder)
            if "joint_plan" in instances:
                motion_recorder.set_joint_plan(instances["joint_plan"])
            motion_recorder.set_reset_controls(state, instances.get("motion_mode"))
            motion_recorder.set_interrupt_group(motion_interrupt_group)
            motion_interrupt_group.register(
                "motion_recorder",
                motion_recorder.interrupt_motion,
                motion_recorder.motion_active,
            )

        head_config = plugins.get("head", {})
        if head_config.get("enabled", True) and "joint_plan" in instances:
            instance = HeadActuatorPlugin(head_config, instances["joint_plan"], state)
            instance.set_interrupt_group(motion_interrupt_group)
            motion_interrupt_group.register(
                "head", instance.halt, instance.motion_active
            )
            instances["head"] = instance
            self._plugins.append(instance)
        virtual_gamepad_config = plugins.get("virtual_gamepad", {})
        if virtual_gamepad_config.get("enabled", False):
            instance = VirtualGamepadPlugin(virtual_gamepad_config, namespace, ros2)
            instances["virtual_gamepad"] = instance
            self._plugins.append(instance)

        controlled_spatial_config = plugins.get("controlled_spatial", {})
        if controlled_spatial_config.get("enabled", False):
            try:
                instance = ControlledSpatialPlugin(controlled_spatial_config, namespace, ros2)
                instances["controlled_spatial"] = instance
                self._plugins.append(instance)
            except Exception as exc:
                print(f"[bundle] ControlledSpatialPlugin init failed, controlled_spatial disabled: {exc}", flush=True)

        if plugins.get("controlled_spatial_map", {}).get("enabled", False):
            try:
                from controlled_spatial_map import make_plugin as make_map_plugin
                map_cfg = dict(plugins["controlled_spatial_map"])
                self._plugins.append(make_map_plugin(map_cfg, namespace, ros2))
                print("[bundle] ControlledSpatialMapPlugin loaded")
            except Exception as e:
                print(f"[bundle] ControlledSpatialMapPlugin load skipped: {e}", flush=True)

        if "safety" in instances:
            instances["safety"].set_controls(
                [
                    instances[key]
                    for key in (
                        "locomotion", "joint_override", "joint_bridge",
                        "virtual_gamepad", "gesture", "motion_recorder", "head",
                        "speaker",
                    )
                    if key in instances
                ]
            )

        native_config = plugins.get("native_sdk", {})
        if native_config.get("enabled", False):
            self._plugins.append(NativeSdkPlugin(native_config, namespace, ros2))

    def start_all(self) -> None:
        if self._started:
            return
        self._active_plugins = []
        self._startup_errors = {}
        for plugin in self._plugins:
            try:
                plugin.start()
                print(f"[bundle] {type(plugin).__name__} started", flush=True)
            except Exception as exc:
                plugin_name = type(plugin).__name__
                self._startup_errors[plugin_name] = str(exc)
                print(f"[bundle] {type(plugin).__name__} start failed: {exc}", flush=True)
                try:
                    plugin.stop()
                except Exception as stop_exc:
                    print(f"[bundle] {plugin_name} cleanup failed: {stop_exc}", flush=True)
            else:
                self._active_plugins.append(plugin)
        self._started = True

    def stop_all(self) -> None:
        if not self._started:
            return
        # Phase 1: stop every physical output before any potentially slow
        # resource teardown (for example speaker process shutdown).
        for plugin in self._active_plugins:
            halt = getattr(plugin, "halt", None)
            if not callable(halt):
                continue
            try:
                halt()
            except Exception as exc:
                print(f"[bundle] {type(plugin).__name__} halt failed: {exc}", flush=True)
        # Phase 2: release resources in reverse construction order so dependent
        # plugins are torn down before the services they reference.
        for plugin in reversed(self._active_plugins):
            try:
                plugin.stop()
            except Exception as exc:
                print(f"[bundle] {type(plugin).__name__} stop failed: {exc}", flush=True)
        self._active_plugins = []
        self._started = False

    def get_all_tools(self) -> list[dict]:
        tools: list[dict] = []
        for plugin in self._active_plugins:
            tools.extend(plugin.get_tools() if hasattr(plugin, "get_tools") else [plugin.get_tool()])
        return tools

    def health(self) -> dict:
        configured_tools = 0
        for plugin in self._plugins:
            definitions = plugin.get_tools() if hasattr(plugin, "get_tools") else [plugin.get_tool()]
            configured_tools += len(definitions)
        active_tools = len(self.get_all_tools())
        acp_status_fn = getattr(
            self,
            "_acp_status",
            lambda: {"state": "ready", "configured": True, "last_error": None},
        )
        acp_status = acp_status_fn()
        if not self._started:
            state = "starting"
        elif self._startup_errors and not self._active_plugins:
            state = "failed"
        elif self._startup_errors:
            state = "degraded"
        elif acp_status.get("state") == "error":
            state = "degraded"
        else:
            state = "running"
        return {
            "state": state,
            "driver": "engineai-t800",
            "configured_plugins": len(self._plugins),
            "active_plugins": len(self._active_plugins),
            "configured_tools": configured_tools,
            "active_tools": active_tools,
            "startup_errors": dict(self._startup_errors),
            "acp": acp_status,
        }

    def dispatch(self, tool_name: str, arguments: dict) -> dict | None:
        for plugin in self._active_plugins:
            tools = plugin.get_tools() if hasattr(plugin, "get_tools") else [plugin.get_tool()]
            for definition in tools:
                if definition["name"] != tool_name:
                    continue
                args = dict(arguments)
                if definition["type"] == "resource":
                    result = plugin.dispatch(tool_name, args)
                    if self._motion_events is not None:
                        self._motion_events.record_tool_call(tool_name, tool_name, args, result)
                    return result
                action = args.pop("action", tool_name)
                args["_tool_name"] = tool_name
                interrupt_group = getattr(self, "_motion_interrupt_group", None)
                if (
                    interrupt_group is not None
                    and definition["type"] == "actuator"
                    and tool_name in self._MOTION_OUTPUT_TOOLS
                ):
                    interrupt_actions = {
                        binding.get("action")
                        for hook_id, binding in definition["inputSchema"]
                        .get("x-hooks", {}).items()
                        if hook_id.startswith("on_interrupt")
                    }
                    safe_action = (
                        action in interrupt_actions
                        or action in self._SAFE_WHILE_MOTION_SETTLING
                    )
                    if (
                        tool_name == "gesture"
                        and action == "stop"
                        and bool(args.get("reset_after", False))
                    ):
                        safe_action = False
                    if not safe_action:
                        blocking_outputs = interrupt_group.blocking_outputs()
                        if blocking_outputs:
                            return {
                                "error": "motion output is active or still settling",
                                "tool": tool_name,
                                "action": action,
                                "blocking_outputs": blocking_outputs,
                            }
                try:
                    result = plugin.dispatch(action, args)
                except Exception as exc:
                    if self._motion_events is not None:
                        self._motion_events.record_exception(tool_name, action, args, exc)
                    raise
                if self._motion_events is not None:
                    self._motion_events.record_tool_call(tool_name, action, args, result)
                return result
        return None


_bundle: T800DeviceBundle | None = None


def make_handler():
    sse_sessions: dict[str, "Handler"] = {}
    sse_sessions_lock = threading.Lock()

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, fmt, *args):
            msg = fmt % args
            if '"POST /mcp' not in msg or "200" not in msg:
                # Escape and cap: msg embeds the raw request line, which on host
                # networking is remote-controlled bytes going straight into the
                # Docker log framer (log injection / control-byte corruption).
                safe = msg.encode("unicode_escape").decode("ascii")[:200]
                print(f"[mcp] {self.address_string()} {safe}")

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

        def _send_sse(self, event: str, data: str) -> bool:
            try:
                payload = f"event: {event}\ndata: {data}\n\n".encode()
                self.wfile.write(payload)
                self.wfile.flush()
                return True
            except (BrokenPipeError, ConnectionResetError, OSError):
                return False

        def do_GET(self):
            parsed = urlparse(self.path)
            if parsed.path == "/mcp/sse":
                session_id = uuid.uuid4().hex
                self.send_response(200)
                self.send_header("Content-Type", "text/event-stream")
                self.send_header("Cache-Control", "no-cache")
                self.send_header("Connection", "keep-alive")
                self.send_header("Access-Control-Allow-Origin", "*")
                self.end_headers()
                with sse_sessions_lock:
                    sse_sessions[session_id] = self
                endpoint = f"/mcp/messages?session_id={session_id}"
                try:
                    if not self._send_sse("endpoint", endpoint):
                        return
                    while self._send_sse("ping", "{}"):
                        time.sleep(15)
                finally:
                    with sse_sessions_lock:
                        sse_sessions.pop(session_id, None)
                return
            if parsed.path == "/health":
                if _bundle is None:
                    self._send_json(503, {"state": "starting", "driver": "engineai-t800"})
                    return
                payload = _bundle.health()
                self._send_json(200 if payload.get("state") == "running" else 503, payload)
            else:
                self._send_json(404, {"error": "not found"})

        def do_OPTIONS(self):
            self.send_response(204)
            self.send_header("Access-Control-Allow-Origin", "*")
            self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
            self.send_header("Access-Control-Allow-Headers", "Content-Type, Accept")
            self.end_headers()

        def do_POST(self):
            parsed = urlparse(self.path)
            if parsed.path not in ("/mcp", "/mcp/messages"):
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

            session_id = parse_qs(parsed.query).get("session_id", [""])[0]
            with sse_sessions_lock:
                sse_client = sse_sessions.get(session_id)

            def response(payload: dict) -> None:
                if sse_client is None:
                    self._send_json(200, payload)
                    return
                self.send_response(202)
                self.send_header("Access-Control-Allow-Origin", "*")
                self.end_headers()
                if not sse_client._send_sse("message", json.dumps(payload, ensure_ascii=False)):
                    with sse_sessions_lock:
                        sse_sessions.pop(session_id, None)

            def ok(result):
                response({"jsonrpc": "2.0", "id": request_id, "result": result})

            def error(code, message):
                response({"jsonrpc": "2.0", "id": request_id,
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

    server = ThreadingHTTPServer(("", port), make_handler())
    print(f"[bundle] MCP server http://localhost:{port}/mcp", flush=True)
    _start_registration(port, config)

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
