#!/usr/bin/env python3
"""RealMan RM75-6F-V driver entry point."""

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


# ── MCP HTTP server ───────────────────────────────────────────────────────────

_bundle = None


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

            rid = rpc.get("id")
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
                        "serverInfo": {"name": "realman-rm75-driver", "version": "1.0.0"},
                    })
                elif method == "tools/list":
                    ok({"tools": _bundle.get_all_tools()})
                elif method == "tools/call":
                    name = params.get("name", "")
                    args = params.get("arguments") or {}
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


def _start_registration(mcp_port: int, name: str, category: str):
    import urllib.request as _urllib
    import ssl as _ssl
    agent_core_url = os.environ.get("AGENT_CORE_URL", "https://localhost:15678")
    payload = json.dumps({
        "name": name,
        "url": f"http://localhost:{mcp_port}/mcp",
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

    cfg = _load_config()
    namespace = _resolve_namespace(cfg)
    mcp_port = int(cfg.get("mcp_port", 15718))

    print(f"[bundle] namespace={namespace} mcp_port={mcp_port}")

    # ROS2
    rclpy.init()
    executor = rclpy.executors.MultiThreadedExecutor()

    ros2_mock = type("Ros2Mock", (), {"executor_core": executor})()

    from device import build_plugins
    plugins = build_plugins(cfg, namespace, ros2_mock)

    class Bundle:
        def get_all_tools(self):
            tools = []
            for p in plugins:
                if hasattr(p, 'get_tools'):
                    tools.extend(p.get_tools())
                else:
                    tools.append(p.get_tool())
            return tools

        def dispatch(self, tool_name: str, args: dict):
            for p in plugins:
                if hasattr(p, 'get_tools'):
                    tool_defs = p.get_tools()
                else:
                    tool_defs = [p.get_tool()]
                for tool_def in tool_defs:
                    if tool_def["name"] == tool_name:
                        action = args.pop("action", tool_name)
                        args['_tool_name'] = tool_name
                        return p.dispatch(action, args)
            return None

    bundle = Bundle()
    _bundle = bundle

    for i, p in enumerate(plugins):
        try:
            p.start()
        except Exception as e:
            print(f"[bundle] Plugin {i} ({type(p).__name__}) start() FAILED: {e}", flush=True)
            import traceback
            traceback.print_exc()

    print(f"[bundle] All {len(plugins)} plugins started", flush=True)

    def _spin():
        while rclpy.ok():
            executor.spin_once(timeout_sec=0.1)

    spin_thread = threading.Thread(target=_spin, daemon=True, name="bundle_spin")
    spin_thread.start()

    _start_registration(mcp_port, cfg.get("name", "RealMan RM75-6F-V"), "driver")

    server = ThreadingHTTPServer(("", mcp_port), make_handler())
    print(f"[bundle] MCP server -> http://localhost:{mcp_port}")

    def _shutdown(signum, frame):
        print(f"[bundle] signal {signum}, shutting down")
        for p in plugins:
            p.stop()
        threading.Thread(target=server.shutdown, daemon=True).start()

    signal.signal(signal.SIGTERM, _shutdown)
    signal.signal(signal.SIGINT, _shutdown)

    try:
        server.serve_forever()
    finally:
        for p in plugins:
            p.stop()
        executor.shutdown()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
