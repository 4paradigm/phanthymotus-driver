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
            self._plugins.append(LocoPlugin(plugins_cfg["loco"], namespace, executor, loco_client))
            print("[bundle] LocoStatePlugin + LocoPlugin loaded")

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
                    # Sensors are always-on; start/stop are no-ops
                    if tool_def["type"] == "sensor" and action in ("start", "stop"):
                        return {"state": "running" if action == "start" else "idle"}
                    # Actuators: start/stop from canvas are no-ops (canvas lifecycle, not robot motion)
                    if tool_def["type"] == "actuator" and action in ("start", "stop"):
                        return {"state": "ready" if action == "start" else "idle"}
                    result = p.dispatch(action, args)
                    if result is not None:
                        return result
                    # Fallback for unhandled actions on sensors
                    if tool_def["type"] == "sensor":
                        return {"state": "running"}
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
    agent_core_url = os.environ.get("AGENT_CORE_URL", "http://localhost:15678")
    payload = json.dumps({
        "name": name,
        "url":  f"http://localhost:{mcp_port}/mcp",
        "category": category,
    }).encode()
    def _run():
        import time as _t
        while True:
            try:
                req = _urllib.Request(
                    f"{agent_core_url}/api/mcp", data=payload,
                    headers={"Content-Type": "application/json"}, method="POST",
                )
                with _urllib.urlopen(req, timeout=3):
                    pass
                _t.sleep(30)
            except Exception as e:
                print(f"[register] failed: {e}, retrying in 5s")
                _t.sleep(5)
    threading.Thread(target=_run, daemon=True, name="register").start()


def main():
    global _bundle

    if len(sys.argv) < 2:
        print(f"Usage: python3 {sys.argv[0]} <networkInterface>")
        sys.exit(1)

    network_iface = sys.argv[1]
    cfg           = _load_config()
    namespace     = _resolve_namespace(cfg)
    mcp_port      = int(cfg.get("mcp_port", 15702))

    print(f"[bundle] namespace={namespace} mcp_port={mcp_port}")

    # DDS init — try specified interface, fallback to interface holding 192.168.123.164
    dds_ok = False
    ifaces_to_try = [network_iface]
    # Detect interface for 192.168.123.x subnet as fallback
    try:
        import netifaces
        for iface_name in netifaces.interfaces():
            addrs = netifaces.ifaddresses(iface_name).get(netifaces.AF_INET, [])
            for addr in addrs:
                if addr["addr"].startswith("192.168.123."):
                    if iface_name not in ifaces_to_try:
                        ifaces_to_try.append(iface_name)
    except ImportError:
        pass
    ifaces_to_try.append("")  # auto-detect as last resort

    for iface in ifaces_to_try:
        try:
            ChannelFactoryInitialize(0, iface)
            print(f"[bundle] DDS initialized on interface: {iface or '(auto)'}")
            dds_ok = True
            break
        except Exception as e:
            print(f"[bundle] DDS init failed on '{iface}': {e}")
    if not dds_ok:
        print("[bundle] WARNING: DDS unavailable — robot communication disabled, MCP server still starting")

    # RPC Proxy — runs LocoClient + AudioClient in a subprocess to avoid GIL contention.
    # The main process has many threads (ROS2 executor, camera, mic) which starve
    # CycloneDDS listener callbacks, causing RPC response timeouts (3104).
    # Use the same interface that succeeded for main process DDS.
    rpc_iface = iface if dds_ok else network_iface
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
