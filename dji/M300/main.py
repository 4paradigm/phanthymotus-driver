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
import time
import uuid
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse

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
        tools = []
        for p in self._plugins:
            if hasattr(p, "get_tools"):
                tools.extend(p.get_tools())
            else:
                tools.append(p.get_tool())
        return tools

    def dispatch(self, tool_name: str, args: dict) -> dict | None:
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
    sse_sessions: dict[str, "Handler"] = {}
    sse_sessions_lock = threading.Lock()

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

        def _send_sse(self, event: str, data: str) -> bool:
            try:
                payload = f"event: {event}\ndata: {data}\n\n".encode()
                self.wfile.write(payload)
                self.wfile.flush()
                return True
            except (BrokenPipeError, ConnectionResetError, OSError):
                return False

        def do_GET(self):
            if urlparse(self.path).path == "/mcp/sse":
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
                    # Do not let an idle proxy tear down the discovery stream.
                    while self._send_sse("ping", "{}"):
                        time.sleep(15)
                finally:
                    with sse_sessions_lock:
                        sse_sessions.pop(session_id, None)
                return
            self.send_response(404)
            self.end_headers()

        def do_OPTIONS(self):
            self.send_response(204)
            self.send_header("Access-Control-Allow-Origin", "*")
            self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
            self.send_header("Access-Control-Allow-Headers", "Content-Type, Accept")
            self.end_headers()

        def do_POST(self):
            parsed = urlparse(self.path)
            if parsed.path not in ("/mcp", "/mcp/messages"):
                self.send_response(404)
                self.end_headers()
                return
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

            session_id = parse_qs(parsed.query).get("session_id", [""])[0]
            with sse_sessions_lock:
                sse_client = sse_sessions.get(session_id)

            def response(payload: dict):
                if sse_client is not None:
                    # Legacy MCP sends responses on the SSE stream and only
                    # requires acknowledgement of the POST request.
                    self.send_response(202)
                    self.end_headers()
                    if not sse_client._send_sse("message", json.dumps(payload)):
                        with sse_sessions_lock:
                            sse_sessions.pop(session_id, None)
                else:
                    self._send(200, json.dumps(payload))

            if rid is None:
                self.send_response(202)
                self.end_headers()
                return

            def ok(result):
                response({"jsonrpc": "2.0", "id": rid, "result": result})

            def err(code, msg):
                response({
                    "jsonrpc": "2.0", "id": rid,
                    "error": {"code": code, "message": msg},
                })

            try:
                if method == "initialize":
                    ok({
                        "protocolVersion": "2024-11-05",
                        "capabilities": {"tools": {}},
                        "serverInfo": {"name": "dji-m300-bundle", "version": "1.0.0"},
                    })
                elif method == "tools/list":
                    all_tools = _bundle.get_all_tools()
                    visible = [t for t in all_tools if not t.get("hidden")]
                    ok({"tools": visible})
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

    bridge_stopping = threading.Event()

    def _watch_bridge() -> None:
        exit_code = bridge_proc.wait()
        if not bridge_stopping.is_set():
            print(f"[bundle] FATAL: psdk_bridge exited ({exit_code}); restarting container", flush=True)
            os._exit(1)

    threading.Thread(target=_watch_bridge, daemon=True, name="bridge_watchdog").start()

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
        bridge_stopping.set()
        _bundle.stop_all()
        bridge_proc.terminate()
        threading.Thread(target=server.shutdown, daemon=True).start()

    signal.signal(signal.SIGTERM, _shutdown)
    signal.signal(signal.SIGINT, _shutdown)

    try:
        server.serve_forever()
    finally:
        bridge_stopping.set()
        _bundle.stop_all()
        bridge_proc.terminate()
        bridge_proc.wait(timeout=3)
        executor.shutdown()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
