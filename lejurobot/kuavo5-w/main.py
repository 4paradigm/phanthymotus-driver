#!/usr/bin/env python3
"""MCP endpoint and isolated ROS 2 runtime for the ROS 1 Noetic adapter."""
from common import logsafe
logsafe.install()

import json
import ipaddress
import os
import re
import signal
import socket
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parent
PROFILE = Path("/opt/phanthy-motus/dds-local.xml")
DRIVER_ID = "lejurobot-kuavo5-w-driver"
MCP_HOST = "127.0.0.1"
_bundle = None


class _McpRequestLog:
    """One bounded sampler shared by all HTTP connections/worker threads."""
    TOOLS = frozenset(('loco', 'arm', 'leg', 'waist', 'head',
                       'joints', 'imu', 'loco_state', 'model',
                       'battery', 'laser_scan', 'camera_left', 'camera_right', 'camera_head', 'camera_depth'))
    STATES = frozenset(('ready', 'idle', 'running', 'executing', 'paused',
                        'completed', 'error', 'rejected', 'disconnected'))

    def __init__(self):
        self._lock = threading.Lock()
        self._buckets = {}
        self._states = {}

    def emit(self, message, error=False, event=None):
        outcome = 'error' if error else 'ok'
        with self._lock:
            now = time.monotonic()
            bucket = self._buckets.setdefault(outcome, {'at': None, 'suppressed': 0})
            transition = False
            if (event and isinstance(event[0], str) and isinstance(event[1], str)
                    and event[0] in self.TOOLS and event[1] in self.STATES):
                transition = self._states.get(event[0]) != event[1]
                self._states[event[0]] = event[1]
            if transition or bucket['at'] is None or now - bucket['at'] >= 5.:
                detail = f' tool={event[0]} state={event[1]}' if transition else ''
                print(f"[mcp] outcome={outcome}{detail} {message} suppressed={bucket['suppressed']}", flush=True)
                bucket.update(at=now, suppressed=0)
            else:
                bucket['suppressed'] += 1


_request_log = _McpRequestLog()


def _load_config():
    with Path(os.environ.get("CONFIG_PATH", ROOT / "config.yaml")).open(encoding="utf-8") as stream:
        return yaml.safe_load(stream) or {}


def _rosbridge_url(config):
    # Compose passes empty when no explicit KUAVO_ROSBRIDGE_URL is set.
    return os.environ.get('ROSBRIDGE_URL') or config.get('rosbridge_url', 'ws://127.0.0.1:9090')


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
        def _local_request(self):
            # Agent Core is a server-side, loopback caller; browsers call Core.
            try:
                local = ipaddress.ip_address(self.client_address[0]).is_loopback
            except ValueError:
                local = False
            hosts = self.headers.get_all('Host', [])
            allowed = {f'localhost:{self.server.server_port}', f'127.0.0.1:{self.server.server_port}'}
            if (not local or self.headers.get_all('Origin', []) or len(hosts) != 1
                    or hosts[0].lower() not in allowed):
                self._error(-32600, 'MCP requires a loopback server-side request', status=403)
                return False
            return True

        def log_message(self, fmt, *args):
            safe = (fmt % args)[:200].encode("unicode_escape").decode("ascii")[:200]
            error = getattr(self, '_rpc_failed', False) or getattr(self, '_http_status', 0) >= 400
            _request_log.emit(safe, error, getattr(self, '_tool_event', None))

        def log_request(self, code='-', size='-'):
            self._http_status = int(code) if str(code).isdigit() else 0
            super().log_request(code, size)

        def log_error(self, fmt, *args):
            self._rpc_failed = True
            self.log_message(fmt, *args)

        def _reply(self, status, payload):
            body = json.dumps(payload, ensure_ascii=False, allow_nan=False).encode()
            self._rpc_failed = 'error' in payload or payload.get('result', {}).get('isError', False)
            self.send_response(status)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def _error(self, code, message, rid=None, status=200):
            self._reply(status, {"jsonrpc": "2.0", "id": rid,
                                 "error": {"code": code, "message": message}})

        def do_POST(self):
            if not self._local_request():
                return
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
                # Polling is sampled; actual card state changes are distinct
                # diagnostic events. Never key sampler storage by user input.
                if params.get('arguments', {}).get('action', 'start') not in ('info', 'get_current_mode'):
                    self._tool_event = (params['name'], value.get('state'))
            else:
                self._error(-32601, f"Method not found: {method}", rid)
                return
            self._reply(200, {"jsonrpc": "2.0", "id": rid, "result": result})

        def do_OPTIONS(self):
            if self._local_request():
                self._error(-32600, 'Browser access is not supported; use Agent Core', status=403)
    return Handler


def _mcp_server(port):
    return ThreadingHTTPServer((MCP_HOST, port), _handler())


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
    ros1 = RosbridgeClient(_rosbridge_url(config))
    stopped = threading.Event()
    rclpy.init(args=None, domain_id=42)
    node = Node("kuavo5w_driver")
    server, ros2_thread = None, None
    try:
        _bundle = KuavoBundle(config, namespace, ros1, node,
                              media_factory=lambda: RosbridgeClient(_rosbridge_url(config)))
        server = _mcp_server(int(config.get("mcp_port", 15747)))
        ros2_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True, name="ros2-domain-42")
        ros2_thread.start()
        ros1.start()
        _bundle.start_background()
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
