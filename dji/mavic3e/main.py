#!/usr/bin/env python3
"""
dji/mavic3e/main.py — DJI Mavic 3E 无人机设备 bundle 统一入口。

读取 config.yaml，按插件配置加载插件，聚合成一个 MCP HTTP server 对外暴露。
通过 bridge_client 与 psdk_bridge (C 进程) 通信，或在 mock 模式下模拟响应。

用法：
    python3 main.py

环境变量：
    CONFIG_PATH — config.yaml 路径（默认同目录下）
    AGENT_CORE_URL — Agent Core 地址（默认 https://localhost:15678）
"""

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

class Mavic3EDeviceBundle:
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

        if plugins_cfg.get("speaker", {}).get("enabled", False):
            from device import SpeakerPlugin
            self._plugins.append(SpeakerPlugin(
                plugins_cfg["speaker"], namespace, executor, bridge))
            print("[bundle] SpeakerPlugin loaded")

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
            "description": "DJI Mavic 3E aircraft metadata (cameras, gimbal range, specs)",
            "inputSchema": {"type": "object", "properties": {}},
        }

    def dispatch(self, tool_name: str, args: dict) -> dict | None:
        if tool_name == "model":
            info_path = Path(__file__).parent / "resource" / "mavic3e_info.json"
            return json.loads(info_path.read_text())
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

_bundle: Mavic3EDeviceBundle | None = None


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
                        "serverInfo": {"name": "dji-mavic3e-bundle", "version": "1.0.0"},
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


# ── Device auto-detect (container startup) ────────────────────────────────

# Known E-Port USB devices (VID, PID or None for any PID)
_EPORT_USB_IDS = [
    ("2ca3", None),    # DJI direct (any PID)
    ("0403", "6001"),  # FTDI FT232R (E-Port dev board)
    ("0403", "6010"),  # FTDI FT2232 (dual-port variant)
    ("0403", "6014"),  # FTDI FT232H
]


def _detect_uart_device(timeout: int = 30) -> str | None:
    """Auto-detect E-Port serial device by scanning /dev/ttyUSB* and /dev/ttyACM*.
    Matches by USB VID/PID whitelist from sysfs. Returns device path or None."""
    import glob
    import time as _t

    start = _t.time()
    while True:
        candidates = sorted(glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*"))
        for dev in candidates:
            vid, pid = _get_usb_ids(dev)
            if not vid:
                continue
            for known_vid, known_pid in _EPORT_USB_IDS:
                if vid == known_vid and (known_pid is None or pid == known_pid):
                    print(f"[bundle] E-Port detected: {dev} (VID={vid} PID={pid})")
                    return dev

        elapsed = _t.time() - start
        if elapsed > timeout:
            print(f"[bundle] WARNING: no E-Port device found after {timeout}s")
            return None
        if int(elapsed) % 5 == 0 and int(elapsed) > 0:
            print(f"[bundle] waiting for E-Port device... ({int(elapsed)}s)")
        _t.sleep(1)


def _get_usb_ids(device_path: str) -> tuple[str, str]:
    """Read USB VID/PID from sysfs for a tty device."""
    dev_name = os.path.basename(device_path)
    # Try /sys/class/tty/<dev>/device/../idVendor
    for base in [f"/sys/class/tty/{dev_name}/device/..",
                 f"/sys/class/tty/{dev_name}/device/../.."]:
        vid_path = f"{base}/idVendor"
        pid_path = f"{base}/idProduct"
        try:
            with open(vid_path) as f:
                vid = f.read().strip()
            with open(pid_path) as f:
                pid = f.read().strip()
            if vid:
                return vid, pid
        except (FileNotFoundError, PermissionError):
            continue
    return "", ""


# ── Entry point ───────────────────────────────────────────────────────────────

def _configure_usb_gadget():
    """Configure USB gadget for DJI PSDK USB Bulk mode.
    Sets up FunctionFS bulk endpoints required for liveview/perception."""
    import subprocess as _sp2
    import time as _t2

    setup_script = "/deploy/setup_usb_bulk.sh"
    if not os.path.exists(setup_script):
        setup_script = os.path.join(os.path.dirname(__file__), "deploy", "setup_usb_bulk.sh")

    if not os.path.exists(setup_script):
        print("[usb] setup_usb_bulk.sh not found, skipping USB Bulk config")
        return

    # Step 1: Run gadget config script (creates FFS functions, mounts)
    print("[usb] running setup_usb_bulk.sh...")
    try:
        result = _sp2.run(["bash", setup_script], capture_output=True, text=True, timeout=15)
        if result.stdout:
            for line in result.stdout.strip().split("\n"):
                print(f"[usb] {line}")
        if result.returncode != 0:
            print(f"[usb] WARNING: setup failed (rc={result.returncode})")
            if result.stderr:
                for line in result.stderr.strip().split("\n")[-5:]:
                    print(f"[usb] {line}")
            return
    except Exception as e:
        print(f"[usb] WARNING: setup error: {e}")
        return

    # Step 2: Launch startup_bulk daemons (must stay alive — keep ep0 open)
    startup_bin = "/usr/local/bin/startup_bulk"
    if not os.path.exists(startup_bin):
        print("[usb] startup_bulk not found, skipping")
        return

    # Kill old instances
    _sp2.run(["pkill", "-f", "startup_bulk"], capture_output=True)
    _t2.sleep(0.5)

    bulk_procs = []
    for i in range(1, 4):
        proc = _sp2.Popen(
            [startup_bin, f"/dev/usb-ffs/bulk{i}"],
            stdout=sys.stdout, stderr=sys.stderr,
        )
        bulk_procs.append(proc)
        _t2.sleep(1)
    print(f"[usb] startup_bulk launched for bulk1/2/3 (pids: {[p.pid for p in bulk_procs]})")

    # Step 3: Bind UDC (after startup_bulk has opened ep0)
    _t2.sleep(1)
    try:
        udc_name = open("/sys/class/udc/" + os.listdir("/sys/class/udc/")[0] + "/../../../UDC", "r")
    except Exception:
        pass
    gadget_udc = "/sys/kernel/config/usb_gadget/l4t/UDC"
    if os.path.exists(gadget_udc):
        udc_name = os.listdir("/sys/class/udc/")[0]
        with open(gadget_udc, "w") as f:
            f.write(udc_name)
        print(f"[usb] UDC bound: {udc_name}")
    else:
        print("[usb] WARNING: cannot find gadget UDC file")


def _start_registration(mcp_port: int, name: str, category: str):
    """Register this driver with agent-core in a background thread, then heartbeat every 30s."""
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
    mcp_port = int(cfg.get("mcp_port", 15702))
    psdk_cfg = cfg.get("psdk_bridge", {})

    # Auto-detect UART device
    uart_dev = psdk_cfg.get("uart_dev", "auto")
    detected_dev = None
    if uart_dev == "auto":
        detected_dev = _detect_uart_device(timeout=10)
    elif os.path.exists(uart_dev):
        detected_dev = uart_dev

    # No device = don't start (avoid mock data misleading users)
    if not detected_dev:
        print(f"[bundle] ERROR: No E-Port device found. Driver will not start.")
        print(f"[bundle] Connect E-Port dev board and restart container.")
        # Keep process alive so container doesn't restart-loop, but don't serve
        import time as _t
        while True:
            _t.sleep(60)

    print(f"[bundle] namespace={namespace} mcp_port={mcp_port} uart={detected_dev}")

    # Start psdk_bridge C process as subprocess
    import subprocess as _sp
    bridge_bin = "/usr/local/bin/psdk_bridge"
    if not os.path.exists(bridge_bin):
        bridge_bin = "/work/psdk_bridge/build/psdk_bridge"
    socket_path = "/tmp/psdk_bridge.sock"

    # Configure USB gadget VID/PID for DJI compatibility (RTL8152)
    # DJI Mavic 3E only recognizes specific USB network adapters
    _configure_usb_gadget()

    bridge_proc = _sp.Popen(
        [bridge_bin, socket_path,
         psdk_cfg.get("app_id", ""),
         psdk_cfg.get("app_key", ""),
         psdk_cfg.get("app_license", ""),
         detected_dev,
         str(psdk_cfg.get("baud_rate", 921600))],
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
    print(f"[bundle] BridgeClient connected (uart={detected_dev})")

    # ROS2
    rclpy.init()
    executor = rclpy.executors.MultiThreadedExecutor()

    _bundle = Mavic3EDeviceBundle(cfg, namespace, executor, bridge)
    _bundle.start_all()

    def _spin():
        while rclpy.ok():
            executor.spin_once(timeout_sec=0.1)

    spin_thread = threading.Thread(target=_spin, daemon=True, name="bundle_spin")
    spin_thread.start()

    _start_registration(mcp_port, cfg.get("name", "DJI Mavic 3E"), "driver")

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
