#!/usr/bin/env python3
"""Small shared runtime for vendor ROS2/DDS MCP drivers.

The vendor-specific modules own all robot contracts.  This module only owns the
common dual-domain ROS lifecycle, MCP transport, registration and tool dispatch.
It deliberately imports ROS lazily so HTTP and schema contracts can be tested on
development machines without ROS installed.
"""

from __future__ import annotations

import dataclasses
import json
import os
import re
import signal
import socket
import threading
from enum import Enum
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any, Callable, Iterable


def load_config(driver_file: str) -> dict:
    import yaml

    default = Path(driver_file).with_name("config.yaml")
    path = Path(os.environ.get("CONFIG_PATH", str(default)))
    with path.open(encoding="utf-8") as handle:
        return yaml.safe_load(handle) or {}


def resolve_namespace(config: dict) -> str:
    raw = os.environ.get("ROS_NAMESPACE") or str(config.get("ros_namespace", "")).strip() or socket.gethostname()
    return re.sub(r"[^a-zA-Z0-9_]", "_", raw)


def configure_cyclonedds(config: dict) -> str:
    ros = config.get("ros", {})
    interface = os.environ.get("NETWORK_INTERFACE") or str(ros.get("robot_interface", "eth0"))
    if not re.fullmatch(r"[a-zA-Z0-9_.:-]+", interface):
        raise ValueError(f"invalid robot network interface: {interface}")
    configured_uri = ros.get("cyclonedds_uri")
    if configured_uri:
        os.environ.setdefault("CYCLONEDDS_URI", str(configured_uri))
        return interface
    os.environ.setdefault(
        "CYCLONEDDS_URI",
        "<CycloneDDS><Domain><General><Interfaces>"
        f"<NetworkInterface name='{interface}'/>"
        "</Interfaces></General></Domain></CycloneDDS>",
    )
    return interface


class DualDomainROS2:
    """Separate vendor and Agent Core DDS domains with independent executors."""

    def __init__(self, robot_domain_id: int, core_domain_id: int):
        import rclpy
        import rclpy.executors
        from rclpy.context import Context

        self.rclpy = rclpy
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
                while self.rclpy.ok(context=context):
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
        if self.rclpy.ok(context=self.ctx_robot):
            self.rclpy.shutdown(context=self.ctx_robot)
        if self.rclpy.ok(context=self.ctx_core):
            self.rclpy.shutdown(context=self.ctx_core)


def jsonable(value: Any) -> Any:
    """Convert ROS messages, numpy values and SDK records to JSON-safe values."""
    if value is None or isinstance(value, (str, int, float, bool)):
        return value
    if isinstance(value, Enum):
        return value.name
    if dataclasses.is_dataclass(value):
        return jsonable(dataclasses.asdict(value))
    if isinstance(value, dict):
        return {str(key): jsonable(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [jsonable(item) for item in value]
    if hasattr(value, "tolist"):
        return jsonable(value.tolist())
    slots = getattr(value, "__slots__", None)
    if slots:
        return {name.lstrip("_"): jsonable(getattr(value, name)) for name in slots if hasattr(value, name)}
    if hasattr(value, "__dict__"):
        return {key: jsonable(item) for key, item in vars(value).items() if not key.startswith("_")}
    return str(value)


def action_schema(actions: dict[str, tuple[list[str], str]], properties: dict[str, dict]) -> dict:
    props = {"action": {"type": "string", "enum": list(actions), "description": "Action to perform"}}
    props.update(properties)
    return {
        "type": "object",
        "properties": props,
        "required": ["action"],
        "x-action-params": {
            name: {"params": params, "description": description}
            for name, (params, description) in actions.items()
        },
    }


def tool(
    name: str,
    kind: str,
    description: str,
    schema: dict | None = None,
    *,
    topic_out: list[dict] | None = None,
) -> dict:
    result = {
        "name": name,
        "type": kind,
        "multiInstance": False,
        "description": description,
        "inputSchema": schema or {"type": "object", "properties": {}},
    }
    if topic_out:
        result["topic_out"] = topic_out
    return result


class DriverBundle:
    def __init__(self, plugins: Iterable[Any]):
        self.plugins = list(plugins)

    def start_all(self) -> None:
        for plugin in self.plugins:
            try:
                plugin.start()
                print(f"[bundle] {type(plugin).__name__} started", flush=True)
            except Exception as exc:
                print(f"[bundle] {type(plugin).__name__} start failed: {exc}", flush=True)

    def stop_all(self) -> None:
        for plugin in reversed(self.plugins):
            try:
                plugin.stop()
            except Exception as exc:
                print(f"[bundle] {type(plugin).__name__} stop failed: {exc}", flush=True)

    def get_all_tools(self) -> list[dict]:
        definitions: list[dict] = []
        for plugin in self.plugins:
            definitions.extend(plugin.get_tools() if hasattr(plugin, "get_tools") else [plugin.get_tool()])
        return definitions

    def dispatch(self, tool_name: str, arguments: dict) -> dict | None:
        for plugin in self.plugins:
            definitions = plugin.get_tools() if hasattr(plugin, "get_tools") else [plugin.get_tool()]
            if not any(definition["name"] == tool_name for definition in definitions):
                continue
            args = dict(arguments)
            action = tool_name if next(d for d in definitions if d["name"] == tool_name)["type"] == "resource" else args.pop("action", tool_name)
            args["_tool_name"] = tool_name
            return plugin.dispatch(action, args)
        return None


def make_handler(bundle_getter: Callable[[], DriverBundle], server_name: str, driver_id: str):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, fmt, *args):
            msg = fmt % args
            if '"POST /mcp' not in msg or "200" not in msg:
                print(f"[mcp] {self.address_string()} {msg}")

        def send_json(self, status: int, payload: dict) -> None:
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
                self.send_json(200, {"state": "running", "driver": driver_id})
            else:
                self.send_json(404, {"error": "not found"})

        def do_OPTIONS(self):
            self.send_response(204)
            self.send_header("Access-Control-Allow-Origin", "*")
            self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
            self.send_header("Access-Control-Allow-Headers", "Content-Type, Accept")
            self.end_headers()

        def do_POST(self):
            if self.path != "/mcp":
                self.send_json(404, {"error": "not found"})
                return
            try:
                length = int(self.headers.get("Content-Length", 0))
                rpc = json.loads(self.rfile.read(length))
            except Exception:
                self.send_json(400, {"jsonrpc": "2.0", "id": None, "error": {"code": -32700, "message": "Parse error"}})
                return
            request_id = rpc.get("id")
            if request_id is None:
                self.send_response(202)
                self.end_headers()
                return

            def ok(result):
                self.send_json(200, {"jsonrpc": "2.0", "id": request_id, "result": result})

            def error(code, message):
                self.send_json(200, {"jsonrpc": "2.0", "id": request_id, "error": {"code": code, "message": message}})

            method = rpc.get("method", "")
            params = rpc.get("params") or {}
            try:
                bundle = bundle_getter()
                if method == "initialize":
                    ok({"protocolVersion": "2024-11-05", "capabilities": {"tools": {}},
                        "serverInfo": {"name": server_name, "version": "1.0.0"}})
                elif method == "tools/list":
                    ok({"tools": bundle.get_all_tools()})
                elif method == "tools/call":
                    name = params.get("name", "")
                    result = bundle.dispatch(name, params.get("arguments") or {})
                    if result is None:
                        error(-32601, f"Unknown tool: {name}")
                    else:
                        ok({"content": [{"type": "text", "text": json.dumps(jsonable(result), ensure_ascii=False)}]})
                else:
                    error(-32601, f"Method not found: {method}")
            except (TypeError, ValueError) as exc:
                error(-32602, str(exc))
            except Exception as exc:
                import traceback
                traceback.print_exc()
                error(-32603, str(exc))

    return Handler


def start_registration(port: int, config: dict, driver_id: str) -> None:
    import ssl
    import time
    import urllib.request

    url = os.environ.get("AGENT_CORE_URL", "https://localhost:15678")
    payload = json.dumps({
        "id": driver_id,
        "name": config.get("name", driver_id),
        "url": f"http://localhost:{port}/mcp",
        "transport": "http",
        "category": "driver",
    }).encode()
    context = ssl.create_default_context()
    context.check_hostname = False
    context.verify_mode = ssl.CERT_NONE

    def loop():
        while True:
            try:
                request = urllib.request.Request(f"{url}/api/mcp", data=payload, headers={"Content-Type": "application/json"}, method="POST")
                with urllib.request.urlopen(request, timeout=3, context=context):
                    pass
                time.sleep(30)
            except Exception as exc:
                print(f"[register] failed: {exc}; retrying in 5s", flush=True)
                time.sleep(5)

    threading.Thread(target=loop, daemon=True, name="register").start()


def run_driver(
    driver_file: str,
    driver_id: str,
    server_name: str,
    build_plugins: Callable[[dict, str, DualDomainROS2], Iterable[Any]],
) -> None:
    config = load_config(driver_file)
    namespace = resolve_namespace(config)
    interface = configure_cyclonedds(config)
    ros_cfg = config.get("ros", {})
    port = int(config["mcp_port"])
    robot_domain = int(ros_cfg.get("robot_domain_id", 0))
    core_domain = int(ros_cfg.get("core_domain_id", 42))
    print(f"[bundle] {driver_id} namespace={namespace} domains={robot_domain}->{core_domain} interface={interface} port={port}")

    ros2 = DualDomainROS2(robot_domain, core_domain)
    ros2.start_spin()
    bundle = DriverBundle(build_plugins(config, namespace, ros2))
    bundle.start_all()
    start_registration(port, config, driver_id)
    server = ThreadingHTTPServer(("", port), make_handler(lambda: bundle, server_name, driver_id))
    stopping = threading.Event()

    def shutdown(signum, _frame):
        if stopping.is_set():
            return
        stopping.set()
        print(f"[bundle] signal {signum}; stopping", flush=True)
        bundle.stop_all()
        threading.Thread(target=server.shutdown, daemon=True).start()

    signal.signal(signal.SIGTERM, shutdown)
    signal.signal(signal.SIGINT, shutdown)
    try:
        server.serve_forever()
    finally:
        bundle.stop_all()
        ros2.shutdown()
