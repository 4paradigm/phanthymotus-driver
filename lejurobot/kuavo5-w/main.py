#!/usr/bin/env python3
"""MCP endpoint and isolated ROS 2 runtime for the ROS 1 Noetic adapter."""
from common import logsafe
logsafe.install()

import json
import os
import re
import signal
import socket
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parent
PROFILE = Path("/opt/phanthy-motus/dds-local.xml")
DRIVER_ID = "lejurobot-kuavo5-w-driver"
_bundle = None


def _load_config():
    with Path(os.environ.get("CONFIG_PATH", ROOT / "config.yaml")).open(encoding="utf-8") as stream:
        return yaml.safe_load(stream) or {}


def _namespace(config):
    value = config.get("ros_namespace", "").strip() or socket.gethostname()
    value = re.sub(r"[^a-zA-Z0-9_]", "_", value)
    return f"robot_{value}" if not value or value[0].isdigit() else value


def _configure_dds():
    # Fail before creating ANY participant: a missing Docker bind mount can be a directory.
    if not PROFILE.is_file():
        raise RuntimeError(f"DDS loopback profile must be a file: {PROFILE}")
    configured = os.environ.get("FASTRTPS_DEFAULT_PROFILES_FILE", str(PROFILE))
    if configured != str(PROFILE):
        raise RuntimeError("this adapter requires the fleet dds-local.xml profile")
    rmw = os.environ.get("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp")
    if rmw != "rmw_fastrtps_cpp" or "FASTDDS_BUILTIN_TRANSPORTS" in os.environ:
        raise RuntimeError("use rmw_fastrtps_cpp with dds-local.xml; remove FASTDDS_BUILTIN_TRANSPORTS")
    os.environ.update(ROS_DOMAIN_ID="42", RMW_IMPLEMENTATION=rmw,
                      FASTRTPS_DEFAULT_PROFILES_FILE=str(PROFILE))


def _register(config, stopped):
    import ssl
    import urllib.request
    context = ssl.create_default_context()
    context.check_hostname = False
    context.verify_mode = ssl.CERT_NONE
    url = os.environ.get("AGENT_CORE_URL", "https://localhost:15678").rstrip("/")
    payload = json.dumps({"id": DRIVER_ID, "name": config.get("name", "Leju Kuavo 5-W Driver"),
        "url": f"http://localhost:{int(config.get('mcp_port', 15747))}/mcp",
        "transport": "http", "category": "driver"}).encode()

    def loop():
        while not stopped.is_set():
            delay = 30
            try:
                request = urllib.request.Request(f"{url}/api/mcp", data=payload,
                    headers={"Content-Type": "application/json"}, method="POST")
                with urllib.request.urlopen(request, timeout=5, context=context):
                    pass
            except Exception as exc:
                print(f"[register] failed: {exc}", flush=True)
                delay = 5
            stopped.wait(delay)
    thread = threading.Thread(target=loop, daemon=True, name="register")
    thread.start()
    return thread


def _handler():
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, fmt, *args):
            safe = (fmt % args).encode("unicode_escape").decode("ascii")[:200]
            print(f"[mcp] {safe}", flush=True)

        def _reply(self, status, payload):
            body = json.dumps(payload, ensure_ascii=False, allow_nan=False).encode()
            self.send_response(status)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            self.wfile.write(body)

        def _error(self, code, message, rid=None, status=200):
            self._reply(status, {"jsonrpc": "2.0", "id": rid,
                                 "error": {"code": code, "message": message}})

        def do_POST(self):
            if self.path.split("?", 1)[0] != "/mcp":
                self._error(-32601, "Not found", status=404)
                return
            try:
                length = int(self.headers.get("Content-Length", 0))
                if not 0 < length <= 65536:
                    self._error(-32600, "Body must contain 1..65536 bytes", status=413)
                    return
                rpc = json.loads(self.rfile.read(length),
                                 parse_constant=lambda v: (_ for _ in ()).throw(ValueError(v)))
            except (ValueError, UnicodeError):
                self._error(-32700, "Parse error", status=400)
                return
            if (not isinstance(rpc, dict) or rpc.get("jsonrpc") != "2.0"
                    or not isinstance(rpc.get("method"), str)
                    or not isinstance(rpc.get("params", {}), dict)):
                self._error(-32600, "Invalid Request", status=400)
                return
            if "id" not in rpc:
                self.send_response(202)
                self.send_header("Content-Length", "0")
                self.end_headers()
                return
            rid, method, params = rpc["id"], rpc["method"], rpc.get("params", {})
            if method == "initialize":
                result = {"protocolVersion": "2024-11-05", "capabilities": {"tools": {}},
                          "serverInfo": {"name": "lejurobot-kuavo5-w", "version": "0.1.0"}}
            elif method == "tools/list":
                result = {"tools": _bundle.get_all_tools()}
            elif method == "tools/call":
                if not isinstance(params.get("name"), str) or not isinstance(params.get("arguments", {}), dict):
                    self._error(-32602, "name must be a string and arguments an object", rid)
                    return
                try:
                    value = _bundle.dispatch(params["name"], params.get("arguments", {}))
                except Exception as exc:
                    value = {"state": "error", "error": f"{type(exc).__name__}: {exc}"}
                if value is None:
                    self._error(-32601, f"Unknown tool: {params['name']}", rid)
                    return
                result = {"content": [{"type": "text", "text": json.dumps(value, ensure_ascii=False)}]}
                if value.get("error") or value.get("state") in ("error", "rejected"):
                    result["isError"] = True
            else:
                self._error(-32601, f"Method not found: {method}", rid)
                return
            self._reply(200, {"jsonrpc": "2.0", "id": rid, "result": result})

        def do_OPTIONS(self):
            self.send_response(204)
            self.send_header("Access-Control-Allow-Origin", "*")
            self.send_header("Access-Control-Allow-Methods", "POST, OPTIONS")
            self.send_header("Access-Control-Allow-Headers", "Content-Type")
            self.send_header("Content-Length", "0")
            self.end_headers()
    return Handler


def main():
    global _bundle
    _configure_dds()
    # Import only after DDS settings are finalized.
    import rclpy
    from rclpy.node import Node
    from device import KuavoBundle
    from rosbridge_client import RosbridgeClient

    config = _load_config()
    namespace = _namespace(config)
    ros1 = RosbridgeClient(os.environ.get("ROSBRIDGE_URL", config.get("rosbridge_url", "ws://127.0.0.1:9090")))
    stopped = threading.Event()
    rclpy.init(args=None, domain_id=42)
    node = Node("kuavo5w_driver")
    server, ros2_thread = None, None
    try:
        _bundle = KuavoBundle(config, namespace, ros1, node)
        server = ThreadingHTTPServer(("", int(config.get("mcp_port", 15747))), _handler())
        ros2_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True, name="ros2-domain-42")
        ros2_thread.start()
        ros1.start()
        _register(config, stopped)
        print(f"[bundle] MCP :{server.server_port}/mcp namespace={namespace} ROS1={ros1.url} ROS2=42", flush=True)

        def shutdown(signum, _frame):
            print(f"[bundle] signal {signum}: shutting down", flush=True)
            stopped.set()
            threading.Thread(target=server.shutdown, daemon=True).start()
        signal.signal(signal.SIGTERM, shutdown)
        signal.signal(signal.SIGINT, shutdown)
        server.serve_forever()
    finally:
        stopped.set()
        if _bundle is not None:
            _bundle.close()
        if server is not None:
            server.server_close()
        ros1.close()
        node.destroy_node()
        rclpy.shutdown()
        if ros2_thread is not None:
            ros2_thread.join(timeout=2)


if __name__ == "__main__":
    main()
