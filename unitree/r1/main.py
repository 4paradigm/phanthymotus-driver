#!/usr/bin/env python3
"""
drivers/unitree/r1/main.py — Unitree R1-EDU 设备 bundle 统一入口。

读取 config.yaml，按插件配置加载插件，聚合成一个 MCP HTTP server 对外暴露。
驱动启动时自动 start 所有插件，关闭时自动 stop。

MCP 工具命名规则：直接使用 tool name（mic, tts, led, loco, loco_state, state）

用法：
    python3 main.py <networkInterface>

环境变量：
    CONFIG_PATH — config.yaml 路径（默认同目录下）
"""

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

import rclpy
import rclpy.executors

from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.h2.loco.h2_loco_client import LocoClient
from unitree_sdk2py.g1.audio.g1_audio_client import AudioClient
from rpc_proxy import RpcProxy
try:
    from common import lifecycle as _lifecycle
except ImportError:  # a checkout rather than the container image, where
    # common/ is copied in beside this file. Load-bearing, so it resolves the
    # repo root rather than degrading to a no-op the way logsafe does.
    import sys as _sys, pathlib as _pathlib
    _sys.path.insert(0, str(_pathlib.Path(__file__).resolve().parents[2]))
    from common import lifecycle as _lifecycle


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

class R1DeviceBundle:
    def __init__(self, cfg: dict, namespace: str, executor,
                 audio_client,
                 loco_client):
        self._plugins: list = []
        plugins_cfg = cfg.get("plugins", {})

        if plugins_cfg.get("mic", {}).get("enabled", False):
            from device import MicPlugin
            self._plugins.append(MicPlugin(plugins_cfg["mic"], namespace, executor))
            print("[bundle] MicPlugin loaded")

        if plugins_cfg.get("tts", {}).get("enabled", False):
            from device import NativeTtsPlugin
            self._plugins.append(NativeTtsPlugin(plugins_cfg["tts"], namespace, executor, audio_client))
            print("[bundle] NativeTtsPlugin loaded")

        if plugins_cfg.get("speaker", {}).get("enabled", False):
            from device import SpeakerPlugin
            self._plugins.append(SpeakerPlugin(plugins_cfg["speaker"], namespace, executor, audio_client))
            print("[bundle] SpeakerPlugin loaded")

        if plugins_cfg.get("led", {}).get("enabled", False):
            from device import LedPlugin
            self._plugins.append(LedPlugin(plugins_cfg["led"], namespace, executor, audio_client))
            print("[bundle] LedPlugin loaded")

        if plugins_cfg.get("loco", {}).get("enabled", False):
            from device import LocoStatePlugin, LocoPlugin
            self._plugins.append(LocoStatePlugin(plugins_cfg["loco"], namespace, executor))
            loco = LocoPlugin(plugins_cfg["loco"], namespace, executor, loco_client)
            self._plugins.append(loco)
            print("[bundle] LocoStatePlugin + LocoPlugin loaded")

            # The streaming counterpart to `loco`. Constructed here, right after
            # it, because it needs a reference to `loco` to arbitrate over the
            # chassis — and it must exist before SmartMotionPlugin below, which
            # looks it up by PREFIX and would otherwise interrupt only half the
            # ways this robot can be moving.
            if plugins_cfg.get("loco_servo", {}).get("enabled", False):
                from loco_servo import LocoServoPlugin
                self._plugins.append(LocoServoPlugin(
                    plugins_cfg["loco_servo"], namespace, executor, loco_client,
                    loco_plugin=loco,
                ))
                print("[bundle] LocoServoPlugin loaded")

        if plugins_cfg.get("state", {}).get("enabled", False):
            from device import StatePlugin
            self._plugins.append(StatePlugin(plugins_cfg["state"], namespace, executor))
            print("[bundle] StatePlugin loaded")

        if plugins_cfg.get("asr", {}).get("enabled", False):
            from device import AsrPlugin
            self._plugins.append(AsrPlugin(plugins_cfg["asr"], namespace, executor))
            print("[bundle] AsrPlugin loaded")

        if plugins_cfg.get("camera", {}).get("enabled", False):
            from device import CameraPlugin
            self._plugins.append(CameraPlugin(plugins_cfg["camera"], namespace, executor))
            print("[bundle] CameraPlugin loaded")

        if plugins_cfg.get("vision_capture", {}).get("enabled", False):
            from vision_capture import VisionCapturePlugin
            self._plugins.append(VisionCapturePlugin(
                plugins_cfg["vision_capture"], namespace, executor))
            print("[bundle] VisionCapturePlugin loaded")

        if plugins_cfg.get("ext_mic", {}).get("enabled", False):
            from ext_devices import ExtMicPlugin
            self._plugins.append(ExtMicPlugin(plugins_cfg["ext_mic"], namespace, executor))
            print("[bundle] ExtMicPlugin loaded")

        if plugins_cfg.get("ext_camera", {}).get("enabled", False):
            from ext_devices import ExtCameraPlugin
            self._plugins.append(ExtCameraPlugin(plugins_cfg["ext_camera"], namespace, executor))
            print("[bundle] ExtCameraPlugin loaded")

        # SmartMotion 统一打断控制（放在最后，需要引用其他 plugin）
        if plugins_cfg.get("smart_motion", {}).get("enabled", True):
            from device import SmartMotionPlugin
            speaker_plugin = next((p for p in self._plugins if getattr(p, 'PREFIX', '') == 'speaker'), None)
            loco_plugin = next((p for p in self._plugins if getattr(p, 'PREFIX', '') == 'loco'), None)
            servo_plugin = next((p for p in self._plugins if getattr(p, 'PREFIX', '') == 'locoservo'), None)
            self._plugins.append(SmartMotionPlugin(
                plugins_cfg.get("smart_motion", {}), namespace, executor,
                speaker_plugin=speaker_plugin,
                loco_plugin=loco_plugin,
                loco_servo_plugin=servo_plugin,
            ))
            print(f"[bundle] SmartMotionPlugin loaded (speaker={'yes' if speaker_plugin else 'no'}, "
                  f"loco={'yes' if loco_plugin else 'no'}, "
                  f"loco_servo={'yes' if servo_plugin else 'no'})")

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
            p.stop()
        print("[bundle] All plugins stopped")

    def get_all_tools(self) -> list:
        tools = []
        for p in self._plugins:
            if hasattr(p, 'get_tools'):
                tools.extend(p.get_tools())
            else:
                tools.append(p.get_tool())
        return tools

    def dispatch(self, tool_name: str, args: dict) -> dict | None:
        for p in self._plugins:
            plugin_tools = p.get_tools() if hasattr(p, 'get_tools') else [p.get_tool()]
            for tool_def in plugin_tools:
                if tool_def["name"] == tool_name:
                    if tool_def["type"] == "resource":
                        return p.dispatch(tool_name, args)
                    action = args.pop("action", tool_name)
                    args['_tool_name'] = tool_name
                    result = p.dispatch(action, args)
                    # A plugin that only knows its own verbs declines these
                    # rather than failing at them — see common/lifecycle.py.
                    if action in _lifecycle.LIFECYCLE_ACTIONS and _lifecycle.is_declined(result):
                        return _lifecycle.reply(action)
                    return result
        return None


# ── MCP HTTP server ───────────────────────────────────────────────────────────

_bundle: R1DeviceBundle | None = None


def make_handler():
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, fmt, *args):
            msg = fmt % args
            if '"POST /mcp' in msg and '200' in msg:
                return
            # The dashboard probes for an SSE endpoint this server does not serve,
            # every couple of seconds. A 404 there is expected, not news, and it
            # drowns out the posture/error lines in the driver log.
            if '"GET /mcp/sse' in msg and '404' in msg:
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
                        "serverInfo": {"name": "r1-device-bundle", "version": "1.0.0"},
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
                err(-32603, str(e))

    return Handler


# ── Entry point ───────────────────────────────────────────────────────────────


def _start_registration(mcp_port: int, name: str, category: str):
    """Register this driver with agent-core in a background thread, then heartbeat every 30s."""
    import urllib.request as _urllib
    import ssl as _ssl
    agent_core_url = os.environ.get("AGENT_CORE_URL", "https://localhost:15678")
    payload = json.dumps({
        "name": name,
        "url":  f"http://localhost:{mcp_port}/mcp",
        "category": category,
    }).encode()
    # agent-core uses self-signed cert, skip verification for localhost
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

    network_iface = sys.argv[1] if len(sys.argv) >= 2 else ""
    cfg           = _load_config()
    namespace     = _resolve_namespace(cfg)
    mcp_port      = int(cfg.get("mcp_port", 15702))

    print(f"[bundle] namespace={namespace} mcp_port={mcp_port}")

    # DDS init — retried in the background, not attempted once and abandoned.
    #
    # On r1_sz this bundle started three seconds before eth10 existed, the one
    # attempt failed, and every state topic on that robot stayed empty from boot
    # while the cards publishing them looked healthy on the canvas. The
    # interface list is recomputed on every attempt, because the thing being
    # waited for is an interface that does not exist yet. See common/dds_link.py.
    from common import dds_link as _dds_link

    _link = _dds_link.install(network_iface)
    # A short wait so the common case — the interface is already there — still
    # looks synchronous in the log and cards subscribe before the first tool
    # call. Failing this wait is not fatal: the link keeps trying, and anything
    # registered through `on_ready` subscribes the moment it succeeds.
    if not _link.wait(5.0):
        print("[bundle] DDS 还没连上，后台继续重试 —— MCP 照常启动，"
              "机器人状态话题在连上之前是空的", flush=True)

    # NOTE: this used to redirect fd 1 to /dev/null and move the real log pipe
    # to a dup'd high fd, to "suppress C++ layer stdout". That was wrong on both
    # counts. The noisy output was Python `print()` in the vendored SDK, not C++,
    # and it was silenced only by an accident of fd inheritance: RpcProxy starts
    # below, spawns, and inherited fd 1 == /dev/null. Meanwhile the shuffle broke
    # the "fd 1 is the docker log" invariant, left two buffered writers on one
    # pipe, and cost every other subprocess its stdout. The SDK prints are now
    # gated at source (UNITREE_RPC_DEBUG), so no redirection is needed.

    # RPC Proxy — runs LocoClient + AudioClient in a subprocess to avoid GIL contention.
    # The main process has many threads (ROS2 executor, camera, mic) which starve
    # CycloneDDS listener callbacks, causing RPC response timeouts (3104).
    # Use the same interface that succeeded for main process DDS — the link
    # reports which one that was, which is not necessarily the configured name
    # (the fallback scan may have found another, or auto-detect may have won).
    rpc_iface = _link.interface if _link.ready else network_iface
    rpc_proxy = RpcProxy(network_iface=rpc_iface)
    print("[bundle] RpcProxy subprocess started")

    # Aliases for plugin compatibility
    audio_client = rpc_proxy
    loco_client = rpc_proxy

    # ROS2
    rclpy.init()
    executor = rclpy.executors.MultiThreadedExecutor()

    _bundle = R1DeviceBundle(cfg, namespace, executor, audio_client, loco_client)
    _bundle.start_all()

    def _spin():
        while rclpy.ok():
            executor.spin_once(timeout_sec=0.1)

    spin_thread = threading.Thread(target=_spin, daemon=True, name="bundle_spin")
    spin_thread.start()

    _start_registration(mcp_port, cfg.get("name", "Unitree R1"), "driver")

    server = ThreadingHTTPServer(("", mcp_port), make_handler())
    print(f"[bundle] MCP server → http://localhost:{mcp_port}")

    def _shutdown(signum, frame):
        print(f"[bundle] signal {signum}, shutting down")
        _bundle.stop_all()
        threading.Thread(target=server.shutdown, daemon=True).start()

    signal.signal(signal.SIGTERM, _shutdown)
    signal.signal(signal.SIGINT, _shutdown)

    try:
        server.serve_forever()
    finally:
        _bundle.stop_all()
        executor.shutdown()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
