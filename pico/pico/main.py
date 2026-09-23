#!/usr/bin/env python3
"""Independent PICO MCP Driver; importing this module opens no sockets."""
from __future__ import annotations
import ipaddress
import json
import os
import signal
import ssl
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlsplit
from urllib.request import Request, ProxyHandler, HTTPSHandler, build_opener
import yaml
from common.ext_vr.plugin import ExtVrPlugin


def make_handler(plugin):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass

        def send_json(self, status, value):
            data = json.dumps(value, ensure_ascii=False, allow_nan=False).encode()
            self.send_response(status)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)

        def do_POST(self):
            if self.path != "/mcp":
                self.send_json(404, {})
                return
            if (
                self.headers.get("Origin") is not None
                or not ipaddress.ip_address(self.client_address[0]).is_loopback
            ):
                self.send_json(403, {"error": "local_mcp_only"})
                return
            rid = None
            try:
                length = int(self.headers.get("Content-Length", "0"))
                if not 0 < length <= 65536:
                    raise ValueError("invalid_request_size")
                message = json.loads(self.rfile.read(length))
                if not isinstance(message, dict):
                    raise ValueError("invalid_request")
                rid = message.get("id")
                method = message.get("method")
                params = message.get("params") or {}
                if method == "initialize":
                    result = {
                        "protocolVersion": "2024-11-05",
                        "capabilities": {"tools": {}},
                        "serverInfo": {"name": "pico-driver", "version": "1.0.0"},
                    }
                elif method == "tools/list":
                    result = {"tools": plugin.get_tools()}
                elif method == "tools/call":
                    if params.get("name") != "teleop_device":
                        raise ValueError("unknown_tool")
                    args = params.get("arguments") or {}
                    value = plugin.dispatch(args.get("action", "info"), args)
                    if value is None:
                        raise ValueError("unknown_action")
                    result = {
                        "content": [
                            {
                                "type": "text",
                                "text": json.dumps(
                                    value, ensure_ascii=False, allow_nan=False
                                ),
                            }
                        ],
                        "isError": bool(value.get("error")),
                    }
                elif method == "notifications/initialized":
                    self.send_json(200, {})
                    return
                else:
                    raise ValueError("unknown_method")
                self.send_json(200, {"jsonrpc": "2.0", "id": rid, "result": result})
            except Exception as exc:
                self.send_json(
                    200,
                    {
                        "jsonrpc": "2.0",
                        "id": rid,
                        "error": {"code": -32000, "message": str(exc)[:240]},
                    },
                )

    return Handler


def register(stop, config):
    agent = os.environ.get("AGENT_CORE_URL", "https://127.0.0.1:15678").rstrip("/")
    context = (
        ssl._create_unverified_context()
        if urlsplit(agent).hostname in ("127.0.0.1", "localhost", "::1")
        else ssl.create_default_context()
    )
    opener = build_opener(ProxyHandler({}), HTTPSHandler(context=context))
    payload = json.dumps(
        {
            "id": "pico-driver",
            "name": "PICO",
            "url": f"http://127.0.0.1:{config['mcp_port']}/mcp",
            "transport": "http",
            "category": "driver",
        }
    ).encode()
    headers = {"Content-Type": "application/json"}
    if os.environ.get("ACCESS_TOKEN"):
        headers["Authorization"] = "Bearer " + os.environ["ACCESS_TOKEN"]
    while not stop.is_set():
        try:
            with opener.open(
                Request(
                    agent + "/api/mcp", data=payload, headers=headers, method="POST"
                ),
                timeout=5,
            ) as response:
                response.read(65536)
            stop.wait(30)
        except Exception:
            print("[pico] Core registration unavailable; retrying", flush=True)
            stop.wait(5)


def main():
    from common import logsafe

    logsafe.install()
    from identity import prepare_config, validate_dds_profile
    import rclpy
    from rclpy.context import Context
    from rclpy.executors import MultiThreadedExecutor

    config = yaml.safe_load(
        Path(
            os.environ.get("CONFIG_PATH", Path(__file__).with_name("config.yaml"))
        ).read_text()
    )
    validate_dds_profile(
        os.environ.get(
            "FASTRTPS_DEFAULT_PROFILES_FILE", "/opt/phanthy-motus/dds-local.xml"
        )
    )
    device_config = prepare_config(config["pico"])
    context = Context()
    rclpy.init(context=context, domain_id=42)
    executor = MultiThreadedExecutor(num_threads=2, context=context)
    ros_thread = threading.Thread(target=executor.spin, name="pico-dds", daemon=True)
    ros_thread.start()
    plugin = ExtVrPlugin(device_config, config.get("namespace", "pico"), executor)
    stop = threading.Event()
    server = ThreadingHTTPServer(
        ("127.0.0.1", int(config.get("mcp_port", 15742))), make_handler(plugin)
    )
    threading.Thread(
        target=register, args=(stop, config), name="pico-registration", daemon=True
    ).start()

    def shutdown(*_):
        stop.set()
        threading.Thread(target=server.shutdown, daemon=True).start()

    signal.signal(signal.SIGTERM, shutdown)
    signal.signal(signal.SIGINT, shutdown)
    try:
        server.serve_forever()
    finally:
        stop.set()
        plugin.close()
        server.server_close()
        executor.shutdown()
        ros_thread.join(3)
        context.shutdown()


if __name__ == "__main__":
    main()
