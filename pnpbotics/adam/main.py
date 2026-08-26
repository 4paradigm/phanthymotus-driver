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
            # Escape and cap: msg embeds the raw request line, which on host
            # networking is remote-controlled bytes going straight into the
            # Docker log framer (log injection / control-byte corruption).
            safe = msg.encode("unicode_escape").decode("ascii")[:200]
            print(f"[mcp] {self.address_string()} {safe}")

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
    except Exception as exc:
        print(f"[adam] WARNING: DDS init unavailable, DDS features disabled: {exc}")

    # Pre-create DDS channels before rclpy.init() to avoid CycloneDDS/FastDDS
    # participant conflicts. One shared rt/handstate reader feeds both the
    # hand-state sensor card and the hand actuator's partial-command logic.
    dds_lowstate_sub = None
    dds_handstate_sub = None
    dds_hand_pub = None
    plugins_cfg = cfg.get("plugins", {})
    need_lowstate = plugins_cfg.get("state", {}).get("enabled", True)
    need_handstate = (
        plugins_cfg.get("hand_state", {}).get("enabled", True)
        or plugins_cfg.get("hand", {}).get("enabled", True)
    )
    need_hand_pub = plugins_cfg.get("hand", {}).get("enabled", True)
    try:
        from pndbotics_sdk_py.core.channel import ChannelSubscriber, ChannelPublisher
        from pndbotics_sdk_py.idl.pnd_adam.msg.dds_ import LowState_, HandState_, HandCmd_

        def _init_channel(label, factory):
            channel = None
            try:
                channel = factory()
                channel.Init()
                print(f"[adam] DDS channel ready: {label}")
                return channel
            except Exception as exc:
                if channel is not None:
                    try:
                        channel.Close()
                    except Exception:
                        pass
                print(f"[adam] WARNING: DDS channel unavailable ({label}): {exc}")
                return None

        if need_lowstate:
            dds_lowstate_sub = _init_channel(
                "rt/lowstate reader",
                lambda: ChannelSubscriber("rt/lowstate", LowState_),
            )
        if need_handstate:
            dds_handstate_sub = _init_channel(
                "rt/handstate reader",
                lambda: ChannelSubscriber("rt/handstate", HandState_),
            )
        if need_hand_pub:
            dds_hand_pub = _init_channel(
                "rt/handcmd writer",
                lambda: ChannelPublisher("rt/handcmd", HandCmd_),
            )
    except Exception as exc:
        print(f"[adam] WARNING: DDS channel setup failed: {exc}")

    # gRPC client
    grpc_host = os.environ.get("GRPC_HOST", cfg.get("grpc_host", "localhost"))
    grpc_port = int(os.environ.get("GRPC_PORT", cfg.get("grpc_port", 6666)))
    from grpc_client import AdamGrpcClient
    grpc_client = AdamGrpcClient(grpc_host, grpc_port)
    grpc_client.connect()
    print(f"[adam] gRPC client → {grpc_host}:{grpc_port}")

    # ROS2 — init after DDS to avoid CycloneDDS participant conflict.
    # The MCP server can still expose DDS-only cards when ROS2 is unavailable.
    rclpy = None
    executor = None
    ros2_enabled = False
    _rclpy = None
    try:
        import rclpy as _rclpy
        import rclpy.executors
        _rclpy.init()
        rclpy = _rclpy
        executor = rclpy.executors.MultiThreadedExecutor()
        ros2_enabled = True
        print("[adam] ROS2 initialized")
    except Exception as exc:
        print(f"[adam] WARNING: ROS2 unavailable; ROS2 cards disabled: {exc}")
        if _rclpy is not None:
            try:
                _rclpy.shutdown()
            except Exception:
                pass
        rclpy = None

    # Load plugins (pass pre-created DDS channels)
    from device import AdamDeviceBundle
    _bundle = AdamDeviceBundle(cfg, namespace, executor, grpc_client,
                               dds_lowstate_sub=dds_lowstate_sub,
                               dds_handstate_sub=dds_handstate_sub,
                               dds_hand_pub=dds_hand_pub,
                               ros2_enabled=ros2_enabled)
    _bundle.start_all()
    print(f"[adam] Bundle loaded ({len(_bundle.get_all_tools())} tools)")

    # ROS2 spin thread
    spin_thread = None
    if ros2_enabled:
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
        _bundle.close_all()
        for label, channel in (
            ("rt/lowstate reader", dds_lowstate_sub),
            ("rt/handcmd writer", dds_hand_pub),
        ):
            if channel is not None:
                try:
                    channel.Close()
                except Exception as exc:
                    print(f"[adam] WARNING: DDS channel close failed ({label}): {exc}", flush=True)
        grpc_client.close()
        if executor is not None:
            executor.shutdown()
        if rclpy is not None:
            try:
                rclpy.shutdown()
            except Exception:
                pass
        if spin_thread is not None:
            spin_thread.join(1.0)


if __name__ == "__main__":
    main()
