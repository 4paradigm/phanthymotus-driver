#!/usr/bin/env python3
"""
drivers/pnpbotics/adam/main.py — PNPbotics Adam MCP HTTP server.

Reads config.yaml, initializes DDS + gRPC + ROS2, loads plugins, and exposes
them as MCP tools via HTTP JSON-RPC 2.0.

Usage:
    python3 main.py <networkInterface>

Environment variables:
    CONFIG_PATH — config.yaml path (default: same directory)
    AGENT_CORE_URL — Agent Core URL (default: https://localhost:15678)
    GRPC_HOST — gRPC host override (default from config.yaml)
    GRPC_PORT — gRPC port override (default from config.yaml)
"""

from __future__ import annotations

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
                        "serverInfo": {"name": "adam-device-bundle", "version": "1.0.0"},
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
                import traceback
                traceback.print_exc()
                err(-32603, str(e))

    return Handler


# ── Registration & Heartbeat ──────────────────────────────────────────────────

def _start_registration(mcp_port: int, name: str, category: str):
    """Register this driver with Agent Core, then heartbeat every 30s."""
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


# ── Entry point ───────────────────────────────────────────────────────────────

def main():
    global _bundle

    network_iface = sys.argv[1] if len(sys.argv) > 1 else None
    cfg           = _load_config()
    namespace     = _resolve_namespace(cfg)
    mcp_port      = int(cfg.get("mcp_port", 15702))
    variant       = cfg.get("variant", "sp")

    print(f"[adam] namespace={namespace} variant={variant} mcp_port={mcp_port}")

    # DDS init (pnd_sdk_python) — MUST be before rclpy.init() to avoid CycloneDDS/FastDDS conflict
    dds_domain_id = int(cfg.get("dds_domain_id", 0))
    try:
        from pndbotics_sdk_py.core.channel import ChannelFactoryInitialize
        if network_iface:
            ChannelFactoryInitialize(dds_domain_id, network_iface)
        else:
            ChannelFactoryInitialize(dds_domain_id)
        print(f"[adam] DDS initialized (domain={dds_domain_id}, iface={network_iface or 'auto'})")
    except ImportError:
        print("[adam] WARNING: pndbotics_sdk_py not found, DDS features disabled")
    except Exception as e:
        print(f"[adam] WARNING: DDS init failed: {e}")

    # Pre-create DDS subscribers before rclpy.init() to avoid CycloneDDS/FastDDS conflict
    dds_lowstate_sub = None
    dds_handstate_sub = None
    dds_hand_pub = None
    dds_hand_sub = None
    try:
        from pndbotics_sdk_py.core.channel import ChannelSubscriber, ChannelPublisher
        from pndbotics_sdk_py.idl.pnd_adam.msg.dds_ import LowState_, HandState_, HandCmd_

        dds_lowstate_sub = ChannelSubscriber("rt/lowstate", LowState_)
        dds_lowstate_sub.Init()
        dds_handstate_sub = ChannelSubscriber("rt/handstate", HandState_)
        dds_handstate_sub.Init()
        dds_hand_pub = ChannelPublisher("rt/handcmd", HandCmd_)
        dds_hand_pub.Init()
        dds_hand_sub = ChannelSubscriber("rt/handstate", HandState_)
        dds_hand_sub.Init()
        print("[adam] DDS subscribers/publishers created")
    except Exception as e:
        print(f"[adam] WARNING: DDS channels failed: {e}")

    # gRPC client
    grpc_host = os.environ.get("GRPC_HOST", cfg.get("grpc_host", "localhost"))
    grpc_port = int(os.environ.get("GRPC_PORT", cfg.get("grpc_port", 6666)))
    from grpc_client import AdamGrpcClient
    grpc_client = AdamGrpcClient(grpc_host, grpc_port)
    grpc_client.connect()
    print(f"[adam] gRPC client → {grpc_host}:{grpc_port}")

    # ROS2 — init after DDS to avoid CycloneDDS participant conflict
    import rclpy
    import rclpy.executors
    rclpy.init()
    executor = rclpy.executors.MultiThreadedExecutor()
    print("[adam] ROS2 initialized")

    # Load plugins (pass pre-created DDS channels)
    from device import AdamDeviceBundle
    _bundle = AdamDeviceBundle(cfg, namespace, executor, grpc_client,
                               dds_lowstate_sub=dds_lowstate_sub,
                               dds_handstate_sub=dds_handstate_sub,
                               dds_hand_pub=dds_hand_pub,
                               dds_hand_sub=dds_hand_sub)
    _bundle.start_all()
    print(f"[adam] Bundle loaded ({len(_bundle.get_all_tools())} tools)")

    # ROS2 spin thread
    def _spin():
        while rclpy.ok():
            executor.spin_once(timeout_sec=0.1)

    spin_thread = threading.Thread(target=_spin, daemon=True, name="ros2_spin")
    spin_thread.start()

    # Register with Agent Core
    driver_name = cfg.get("name", "PNPbotics Adam")
    _start_registration(mcp_port, driver_name, "driver")

    # MCP HTTP server
    server = ThreadingHTTPServer(("", mcp_port), make_handler())
    print(f"[adam] MCP server → http://localhost:{mcp_port}")

    def _shutdown(signum, frame):
        print(f"[adam] signal {signum}, shutting down")
        _bundle.stop_all()
        grpc_client.close()
        threading.Thread(target=server.shutdown, daemon=True).start()

    signal.signal(signal.SIGTERM, _shutdown)
    signal.signal(signal.SIGINT, _shutdown)

    try:
        server.serve_forever()
    finally:
        _bundle.stop_all()
        grpc_client.close()
        executor.shutdown()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
