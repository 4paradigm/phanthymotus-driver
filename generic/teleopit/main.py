#!/usr/bin/env python3
"""Teleopit simulation cards, served through the existing MCP/Core contracts."""

from __future__ import annotations

import sys
from pathlib import Path

# Repository checkout and image layout both put common/ above this directory.
for _parent in Path(__file__).resolve().parents:
    if (_parent / "common" / "vendor_runtime.py").is_file():
        sys.path.insert(0, str(_parent))
        break

from common import logsafe

logsafe.install()

import argparse
import os
import signal
import threading
from http.server import ThreadingHTTPServer

from common.vendor_runtime import (
    DriverBundle, load_config, make_handler, resolve_namespace, start_registration,
)

from plugin import CoreTopicPublisher, TeleopitPlugin


DRIVER_ID = "teleopit-driver"
SERVER_NAME = "teleopit-simulation"


def build_bundle(config: dict, manager=None, publisher=None) -> DriverBundle:
    """Build a ROS/Teleopit-import-free bundle when collaborators are supplied."""
    if manager is None:
        from manager import SimulationManager
        manager = SimulationManager(config.get("teleopit", {}))
    plugin = TeleopitPlugin(config, resolve_namespace(config), manager, publisher)
    bundle = DriverBundle([plugin])
    bundle.manager = manager
    bundle.plugin = plugin
    return bundle


def create_server(config: dict, bundle: DriverBundle, host=None, port=None) -> ThreadingHTTPServer:
    base = make_handler(lambda: bundle, SERVER_NAME, DRIVER_ID)

    class Handler(base):
        def log_message(self, fmt, *args):
            msg = fmt % args
            if '"POST /mcp' not in msg or "200" not in msg:
                safe = msg.encode("unicode_escape").decode("ascii")[:200]
                print(f"[mcp] {self.address_string()} {safe}", flush=True)

        def do_POST(self):
            # The shared handler remains the owner of JSON-RPC framing. Bound
            # request size here because this card needs no large input payloads.
            try:
                length = int(self.headers.get("Content-Length", "0"))
            except ValueError:
                length = -1
            if length <= 0 or length > 65536:
                self.close_connection = True
                self.send_json(413 if length > 65536 else 400,
                               {"error": "Content-Length must be between 1 and 65536"})
                return
            self.connection.settimeout(5)
            super().do_POST()

    bind_host = host if host is not None else os.environ.get("MCP_BIND_HOST", config.get("bind_host", "127.0.0.1"))
    bind_port = int(port if port is not None else config.get("mcp_port", 15719))
    return ThreadingHTTPServer((bind_host, bind_port), Handler)


def main(argv=None) -> None:
    parser = argparse.ArgumentParser(description="Teleopit G1 29DoF 仿真 Driver（无真机输出）")
    parser.add_argument("--no-ros", action="store_true", help="仅提供 MCP，不发布 Core 状态/画面话题")
    parser.add_argument("--no-register", action="store_true", help="不自动向 Agent Core 注册")
    parser.add_argument("--host", help="MCP 监听地址，默认 127.0.0.1")
    parser.add_argument("--port", type=int, help="MCP 监听端口，默认 15719")
    args = parser.parse_args(argv)
    config = load_config(__file__)
    namespace = resolve_namespace(config)
    publisher = None
    if not args.no_ros:
        try:
            publisher = CoreTopicPublisher(namespace, config.get("ros", {}).get("core_domain_id", 42))
        except Exception as exc:
            print(f"[teleopit] ROS 2 unavailable; MCP remains available: {exc}", flush=True)
    bundle = build_bundle(config, publisher=publisher)
    server = None
    stopping = threading.Event()

    def shutdown(signum, _frame):
        if stopping.is_set():
            return
        stopping.set()
        print(f"[teleopit] signal {signum}; stopping", flush=True)
        threading.Thread(target=server.shutdown, daemon=True).start()

    try:
        server = create_server(config, bundle, args.host, args.port)
        bundle.start_all()
        if not args.no_register:
            start_registration(server.server_address[1], config, DRIVER_ID)
        signal.signal(signal.SIGTERM, shutdown)
        signal.signal(signal.SIGINT, shutdown)
        print(f"[teleopit] MCP={server.server_address} namespace={namespace} hardware_output=false", flush=True)
        server.serve_forever()
    finally:
        bundle.stop_all()
        bundle.manager.close()
        if server is not None:
            server.server_close()
        if publisher is not None:
            publisher.close()


if __name__ == "__main__":
    main()
