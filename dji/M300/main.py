#!/usr/bin/env python3
"""
dji/M300/main.py — DJI Matrice 300 RTK 无人机设备 bundle 统一入口。

读取 config.yaml，按插件配置加载插件，聚合成一个 MCP HTTP server 对外暴露。
通过 bridge_client 与 psdk_bridge (C 进程) 通信，或在 mock 模式下模拟响应。

用法：
    python3 main.py

环境变量：
    CONFIG_PATH — config.yaml 路径（默认同目录下）
    AGENT_CORE_URL — Agent Core 地址（默认 https://localhost:15678）
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


# ── Bundle ────────────────────────────────────────────────────────────────────

class M300DeviceBundle:
    def __init__(self, cfg: dict, namespace: str, executor, bridge):
        self._plugins: list = []
        self._bridge = bridge
        self._namespace = namespace
        plugins_cfg = cfg.get("plugins", {})

        if plugins_cfg.get("telemetry", {}).get("enabled", False):
            from device import TelemetryPlugin
            self._plugins.append(TelemetryPlugin(
                plugins_cfg["telemetry"], namespace, executor, bridge))
            print("[bundle] TelemetryPlugin loaded")

        if plugins_cfg.get("camera_stream", {}).get("enabled", False):
            from device import CameraStreamPlugin
            self._plugins.append(CameraStreamPlugin(
                plugins_cfg["camera_stream"], namespace, executor, bridge))
            print("[bundle] CameraStreamPlugin loaded")

        if plugins_cfg.get("perception", {}).get("enabled", False):
            from device import PerceptionPlugin
            self._plugins.append(PerceptionPlugin(
                plugins_cfg["perception"], namespace, executor, bridge))
            print("[bundle] PerceptionPlugin loaded")

        if plugins_cfg.get("hms", {}).get("enabled", False):
            from device import HmsPlugin
            self._plugins.append(HmsPlugin(
                plugins_cfg["hms"], namespace, executor, bridge))
            print("[bundle] HmsPlugin loaded")

        if plugins_cfg.get("flight", {}).get("enabled", False):
            from device import FlightPlugin
            self._plugins.append(FlightPlugin(
                plugins_cfg["flight"], namespace, executor, bridge))
            print("[bundle] FlightPlugin loaded")

        if plugins_cfg.get("camera", {}).get("enabled", False):
            from device import CameraPlugin
            self._plugins.append(CameraPlugin(
                plugins_cfg["camera"], namespace, executor, bridge))
            print("[bundle] CameraPlugin loaded")

        if plugins_cfg.get("gimbal", {}).get("enabled", False):
            from device import GimbalPlugin
            self._plugins.append(GimbalPlugin(
                plugins_cfg["gimbal"], namespace, executor, bridge))
            print("[bundle] GimbalPlugin loaded")

        if plugins_cfg.get("waypoint", {}).get("enabled", False):
            from device import WaypointPlugin
            self._plugins.append(WaypointPlugin(
                plugins_cfg["waypoint"], namespace, executor, bridge))
            print("[bundle] WaypointPlugin loaded")

        if plugins_cfg.get("time_sync", {}).get("enabled", False):
            from device import TimeSyncPlugin
            self._plugins.append(TimeSyncPlugin(
                plugins_cfg["time_sync"], namespace, executor, bridge))
            print("[bundle] TimeSyncPlugin loaded")

    def start_all(self) -> None:
        for i, p in enumerate(self._plugins):
            try:
                p.start()
            except Exception as e:
                print(f"[bundle] Plugin {i} ({type(p).__name__}) start() FAILED: {e}", flush=True)
                import traceback
                traceback.print_exc()
        print(f"[bundle] All {len(self._plugins)} plugins started", flush=True)

    def stop_all(self) -> None:
        for p in self._plugins:
            try:
                p.stop()
            except Exception:
                pass
        self._bridge.stop()
        print("[bundle] All plugins stopped")

    def get_all_tools(self) -> list:
        tools = [self._model_tool()]
        for p in self._plugins:
            if hasattr(p, "get_tools"):
                tools.extend(p.get_tools())
            else:
                tools.append(p.get_tool())
        return tools

    def _model_tool(self) -> dict:
        return {
            "name": "model",
            "type": "resource",
            "description": "DJI Matrice 300 RTK aircraft metadata (payload, gimbal range, specs)",
            "inputSchema": {"type": "object", "properties": {}},
        }

    def dispatch(self, tool_name: str, args: dict) -> dict | None:
        if tool_name == "model":
            info_path = Path(__file__).parent / "resource" / "m300_info.json"
            info = json.loads(info_path.read_text())
            # Merge dynamic aircraft info from bridge
            try:
                resp = self._bridge.get_aircraft_info()
                if resp and resp.get("ok"):
                    info["aircraft_info"] = resp["data"]
            except Exception:
                pass
            return info
        for p in self._plugins:
            plugin_tools = p.get_tools() if hasattr(p, "get_tools") else [p.get_tool()]
            for tool_def in plugin_tools:
                if tool_def["name"] == tool_name:
                    if tool_def["type"] == "resource":
                        return p.dispatch(tool_name, args)
                    action = args.pop("action", tool_name)
                    args["_tool_name"] = tool_name
                    return p.dispatch(action, args)
        return None


# ── MCP HTTP server ───────────────────────────────────────────────────────────

_bundle: M300DeviceBundle | None = None


def make_handler():
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, fmt, *args):
            msg = fmt % args
            if '"POST /mcp' in msg and "200" in msg:
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
                self._send(400, json.dumps({
                    "jsonrpc": "2.0", "id": None,
                    "error": {"code": -32700, "message": "Parse error"},
                }))
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
                self._send(200, json.dumps({
                    "jsonrpc": "2.0", "id": rid,
                    "error": {"code": code, "message": msg},
                }))

            try:
                if method == "initialize":
                    ok({
                        "protocolVersion": "2024-11-05",
                        "capabilities": {"tools": {}},
                        "serverInfo": {"name": "dji-m300-bundle", "version": "1.0.0"},
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


# ── Entry point ───────────────────────────────────────────────────────────────


def _start_registration(mcp_port: int, name: str, category: str):
    """Register this driver with agent-core in a background thread, then heartbeat every 30s."""
    import urllib.request as _urllib
    import ssl as _ssl

    # Drivers share the Nano network namespace with Agent Core.  Use the IPv4
    # loopback address explicitly: on Nano images that have IPv6 disabled,
    # ``localhost`` may resolve to ``::1`` first and urllib fails with
    # EADDRNOTAVAIL before it attempts IPv4.
    agent_core_url = os.environ.get("AGENT_CORE_URL", "https://127.0.0.1:15678")
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
    mcp_port = int(cfg.get("mcp_port", 15702))
    psdk_cfg = cfg.get("psdk_bridge", {})

    # This deployment is directly connected to the M300 PSDK port.  Its DJI
    # USB CDC device (VID:PID 2ca3:001f) is UART0 at /dev/ttyACM0.  Do not
    # select the optional E-Port development-kit FTDI node (/dev/ttyUSB0).
    uart0_dev = psdk_cfg.get("uart0_dev", "/dev/ttyACM0")
    uart1_dev = psdk_cfg.get("uart1_dev", "/dev/ttyACM0")
    missing = [dev for dev in (uart0_dev, uart1_dev) if not os.path.exists(dev)]
    if missing:
        print("[bundle] ERROR: M300 E-Port UARTs are incomplete: "
              f"missing {', '.join(missing)}. Driver will not start.")
        print("[bundle] Expected UART0=/dev/ttyACM0 (DJI USB CDC); "
              "ttyUSB0 is only used with the optional E-Port dev kit.")
        # Keep process alive so container doesn't restart-loop, but don't serve
        import time as _t
        while True:
            _t.sleep(60)

    print(f"[bundle] namespace={namespace} mcp_port={mcp_port} "
          f"uart0={uart0_dev} uart1={uart1_dev}")

    # Start psdk_bridge C process as subprocess
    import subprocess as _sp
    bridge_bin = "/usr/local/bin/psdk_bridge"
    if not os.path.exists(bridge_bin):
        bridge_bin = "/work/psdk_bridge/build/psdk_bridge"
    socket_path = "/tmp/psdk_bridge.sock"

    bridge_proc = _sp.Popen(
        [bridge_bin, socket_path,
         psdk_cfg.get("app_id", ""),
         psdk_cfg.get("app_key", ""),
         psdk_cfg.get("app_license", ""),
         uart0_dev,
         str(psdk_cfg.get("uart0_baud_rate", 460800)),
         uart1_dev],
        stdout=sys.stdout, stderr=sys.stderr,
    )
    print(f"[bundle] psdk_bridge started (pid={bridge_proc.pid})")

    # Wait for socket to appear (PSDK handshake takes 10-20s)
    import time as _t
    for i in range(300):  # 30 seconds max
        if os.path.exists(socket_path):
            break
        if i % 50 == 0 and i > 0:
            print(f"[bundle] waiting for psdk_bridge socket... ({i//10}s)")
        _t.sleep(0.1)
    else:
        print("[bundle] ERROR: psdk_bridge socket not ready after 30s, exiting")
        bridge_proc.terminate()
        sys.exit(1)

    # Give bridge a moment to accept connections
    _t.sleep(1)

    # Bridge client — connects to psdk_bridge C process (retry up to 5 times)
    from bridge_client import BridgeClient
    bridge = None
    for attempt in range(5):
        try:
            bridge = BridgeClient(
                socket_path=socket_path,
                mock_mode=False,
            )
            break
        except Exception as e:
            print(f"[bundle] BridgeClient connect attempt {attempt+1} failed: {e}")
            _t.sleep(2)
    if bridge is None:
        print("[bundle] ERROR: cannot connect to psdk_bridge, exiting")
        bridge_proc.terminate()
        sys.exit(1)
    print(f"[bundle] BridgeClient connected (uart0={uart0_dev} uart1={uart1_dev})")

    # ROS2
    rclpy.init()
    executor = rclpy.executors.MultiThreadedExecutor()

    _bundle = M300DeviceBundle(cfg, namespace, executor, bridge)
    _bundle.start_all()

    def _spin():
        while rclpy.ok():
            executor.spin_once(timeout_sec=0.1)

    spin_thread = threading.Thread(target=_spin, daemon=True, name="bundle_spin")
    spin_thread.start()

    _start_registration(mcp_port, cfg.get("name", "DJI Matrice 300 RTK"), "driver")

    server = ThreadingHTTPServer(("", mcp_port), make_handler())
    print(f"[bundle] MCP server → http://localhost:{mcp_port}")

    def _shutdown(signum, frame):
        print(f"[bundle] signal {signum}, shutting down")
        _bundle.stop_all()
        bridge_proc.terminate()
        threading.Thread(target=server.shutdown, daemon=True).start()

    signal.signal(signal.SIGTERM, _shutdown)
    signal.signal(signal.SIGINT, _shutdown)

    try:
        server.serve_forever()
    finally:
        _bundle.stop_all()
        bridge_proc.terminate()
        bridge_proc.wait(timeout=3)
        executor.shutdown()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
