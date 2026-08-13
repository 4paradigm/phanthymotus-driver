#!/usr/bin/env python3
"""
xbot/q5/main.py — Q5 轮式人形机器人驱动卡片统一入口。

读取 config.yaml，按插件配置加载插件，聚合成一个 MCP HTTP server 对外暴露。
驱动启动时自动 start 所有插件，关闭时自动 stop。

ROS2 Domain ID = 211（与 Q5 本体控制器统一）

用法：
    python3 main.py

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
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import yaml

import rclpy
import rclpy.executors

# ── Q5 Bridge Worker (subprocess DDS bridge) ──────────────────────────────────
# Spawned as a subprocess with Domain 42/FastDDS to publish sensor snapshots
# to Agent Core ROS2 domain. Mirrors G1's safety_harness.py pattern.
import q5_bridge_worker as bridge_worker_mod


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

class Q5DeviceBundle:
    def __init__(self, cfg: dict, namespace: str, executor, sdk_client=None):
        self._cfg = cfg
        self._sdk_client = sdk_client
        self._plugins: list = []
        self._failed_plugins: list = []
        plugins_cfg = cfg.get("plugins", {})

        # Only verified Q5 cards are shipped in this bundle.
        _PLUGIN_MAP = [
            ("base_drive", "BaseDrivePlugin"),
            ("arm_control", "ArmControlPlugin"),
            ("hand_control", "HandControlPlugin"),
            ("hand_gesture", "HandGesturePlugin"),
            ("head_control", "HeadControlPlugin"),
            ("state", "StatePlugin"),
            ("battery", "BatteryPlugin"),
            ("audio", "AudioPlugin"),
            ("mic", "MicPlugin"),
            ("speaker", "SpeakerPlugin"),
            ("camera_rgb", "CameraRgbPlugin"),
            ("camera_depth", "CameraDepthPlugin"),
        ]

        for key, cls_name in _PLUGIN_MAP:
            if not plugins_cfg.get(key, {}).get("enabled", False):
                continue
            try:
                import device
                cls = getattr(device, cls_name)
                plugin = cls(plugins_cfg[key], namespace, executor, self._sdk_client)
                self._plugins.append(plugin)
                print(f"[bundle] {cls_name} loaded")
            except Exception as e:
                print(f"[bundle] {cls_name} FAILED to load: {e}", flush=True)
                import traceback
                traceback.print_exc()
                self._failed_plugins.append((key, cls_name, str(e)))

        if self._failed_plugins:
            print(f"[bundle] WARNING: {len(self._failed_plugins)} plugin(s) failed to load: "
                  f"{', '.join(f'{n}({k})' for k, n, _ in self._failed_plugins)}", flush=True)
        print(f"[bundle] {len(self._plugins)}/{len(self._plugins)+len(self._failed_plugins)} plugins loaded successfully")

    def start_all(self) -> None:
        for i, p in enumerate(self._plugins):
            start = getattr(p, "start", None)
            if not callable(start):
                continue
            try:
                start()
            except Exception as e:
                print(f"[bundle] Plugin {i} ({type(p).__name__}) start() FAILED: {e}", flush=True)
                import traceback
                traceback.print_exc()
        print(f"[bundle] All {len(self._plugins)} plugins started", flush=True)

    def stop_all(self) -> None:
        for p in self._plugins:
            stop = getattr(p, "stop", None)
            if not callable(stop):
                continue
            try:
                stop()
            except Exception:
                pass
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
                    if tool_def.get("type") == "resource":
                        return p.dispatch(tool_name, args)
                    default_action = tool_def.get("default_action", "start")
                    action = args.pop("action", default_action)
                    args['_tool_name'] = tool_name
                    result = p.dispatch(action, args)
                    return result
        return None


# ── MCP HTTP server ───────────────────────────────────────────────────────────

_bundle: Q5DeviceBundle | None = None


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
                        "serverInfo": {"name": _bundle._cfg.get("name", "q5-device-bundle"), "version": "1.0.0"},
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
                        tool_result = {
                            "content": [{
                                "type": "text",
                                "text": json.dumps(result, ensure_ascii=False),
                            }],
                        }
                        if (isinstance(result, dict)
                                and (result.get("state") == "error"
                                     or "error" in result)):
                            tool_result["isError"] = True
                        ok(tool_result)
                else:
                    err(-32601, f"Method not found: {method}")
            except Exception as e:
                import traceback
                traceback.print_exc()
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

    cfg       = _load_config()
    namespace = _resolve_namespace(cfg)
    mcp_port  = int(cfg.get("mcp_port", 15708))

    print(f"[bundle] namespace={namespace} mcp_port={mcp_port}")

    # ROS2 init — Domain ID 211 for Q5 (set via ROS_DOMAIN_ID env var)
    rclpy.init()
    executor = rclpy.executors.MultiThreadedExecutor()
    print(f"[bundle] ROS2 initialized on domain {os.environ.get('ROS_DOMAIN_ID', '?')}")

    # Shared read-only ROS2 client — subscribes to all sensor topics
    from q5_sdk_client import Q5SdkClient
    sdk_client = Q5SdkClient(cfg.get("joint_state_position_unit", "radians"))
    sdk_client.start(executor)
    print(f"[bundle] SDK client started ({'live' if sdk_client.available else 'STUB'})")

    # Bridge Worker subprocess — publishes sensor snapshots to Domain 42/FastDDS
    bridge_worker = bridge_worker_mod.BridgeWorker(namespace, debug=False)
    bridge_worker.start()
    sdk_client.publish_media = bridge_worker.push_media
    sdk_client.publish_audio = bridge_worker.push_audio
    sdk_client.configure_speaker = bridge_worker.configure_speaker
    sdk_client.pop_speaker_chunk = bridge_worker.pop_speaker_chunk
    print("[bundle] BridgeWorker subprocess started (Domain 42/FastDDS)")

    # Background thread: periodically push full_snapshot to bridge worker
    # This replaces the old q5_sensor_bridge.py HTTP polling with direct multiprocessing.Queue
    _bridge_running = True

    def _bridge_pusher():
        """Push sensor snapshots to bridge subprocess at ~10Hz."""
        import time as _time
        while _bridge_running and sdk_client.available:
            try:
                full_snap = sdk_client.full_snapshot()
                bridge_worker.push_snapshot(full_snap)
            except Exception as e:
                print(f"[bundle] bridge push error: {e}", flush=True)
            _time.sleep(0.1)  # 10Hz

    _bridge_pusher_thread = threading.Thread(target=_bridge_pusher, daemon=True, name="bridge_pusher")
    _bridge_pusher_thread.start()

    # 等待一下让SDK客户端初始化并获取一些数据
    print("[bundle] Waiting for sensor data...")
    time.sleep(2)

    # 输出诊断信息
    if sdk_client.available:
        diag = sdk_client.get_diagnostic_info()
        print(f"[bundle] SDK diagnostic info:")
        print(f"  - available: {diag['available']}")
        print(f"  - running: {diag['running']}")
        print(f"  - lifecycle_state: {diag['lifecycle_state']}")
        print(f"  - lifecycle_source: {diag['lifecycle_source']}")
        print(f"  - joint_data: available={diag['joint_data']['available']}, fresh={diag['joint_data']['fresh']}, count={diag['joint_data']['joint_count']}")
        if diag['joint_data']['age_ms'] is not None:
            print(f"  - joint_data age: {diag['joint_data']['age_ms']}ms")
        print(f"  - node_initialized: {diag['node_initialized']}")
        print(f"  - lifecycle_service_available: {diag['lifecycle_service_available']}")
    else:
        print("[bundle] SDK client is not available")

    _bundle = Q5DeviceBundle(cfg, namespace, executor, sdk_client)
    _bundle.start_all()

    def _spin():
        while rclpy.ok():
            executor.spin_once(timeout_sec=0.1)

    spin_thread = threading.Thread(target=_spin, daemon=True, name="bundle_spin")
    spin_thread.start()

    _start_registration(mcp_port, cfg.get("name", "XBot Q5"), "driver")

    server = ThreadingHTTPServer(("", mcp_port), make_handler())
    print(f"[bundle] MCP server → http://localhost:{mcp_port}")

    def _shutdown(signum, frame):
        print(f"[bundle] signal {signum}, shutting down")
        global _bridge_running
        _bridge_running = False
        bridge_worker.shutdown()
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
