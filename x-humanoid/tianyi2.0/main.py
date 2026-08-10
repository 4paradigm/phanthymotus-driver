#!/usr/bin/env python3
"""
x-humanoid/tianyi2.0/main.py — 天轶2.0 Pro 设备 bundle 统一入口。

读取 config.yaml，按插件配置加载插件，聚合成一个 MCP HTTP server 对外暴露。
驱动启动时自动 start 所有插件，关闭时自动 stop。

双 Domain 模式：
  - domain 0: 订阅/发布到天轶本体控制器 (192.168.41.1)
  - domain 42: 发布传感器数据给 Agent Core

用法：
    python3 main.py

环境变量：
    CONFIG_PATH — config.yaml 路径（默认同目录下）
    SLAMTEC_URL — Slamtec底盘API地址（默认 http://192.168.11.1:1448）
"""

import json
import os
import re
import signal
import socket
import subprocess
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import yaml

import rclpy
import rclpy.executors
from rclpy.context import Context


# ── Config ────────────────────────────────────────────────────────────────────

def _load_config() -> dict:
    config_path = os.environ.get("CONFIG_PATH", str(Path(__file__).parent / "config.yaml"))
    with open(config_path) as f:
        return yaml.safe_load(f)


def _ensure_lyre_audio_mode():
    """Switch host lyre service to audio mode (ASR + TTS, no dialogue) if not already."""
    _nsenter = ["nsenter", "-t", "1", "-m", "-u", "-i", "-n", "-p", "--"]
    target = "audio"
    try:
        result = subprocess.run(
            _nsenter + ["cat", "/home/nvidia/data/param/lyre_launch_mode"],
            capture_output=True, text=True, timeout=5)
        current = result.stdout.strip()
        if current == target:
            print(f"[lyre] already in {target} mode")
            return
        subprocess.run(
            _nsenter + ["bash", "-c", f"echo {target} > /home/nvidia/data/param/lyre_launch_mode"],
            check=True, timeout=5)
        subprocess.run(
            _nsenter + ["systemctl", "restart", "lyre"],
            check=True, timeout=15)
        print(f"[lyre] switched from {current!r} to {target} mode, restarted")
        time.sleep(3)
    except Exception as e:
        print(f"[lyre] WARNING: could not switch to {target} mode: {e}")


def _ensure_audio_sender(cfg: dict):
    """Ensure audio_sender.py TCP server is running on remote hosts for network mic sources."""
    ext_mic_cfg = cfg.get("plugins", {}).get("ext_mic", {})
    if not ext_mic_cfg.get("enabled", False):
        return
    sources = ext_mic_cfg.get("network_sources", [])

    for src in sources:
        url = src.get("url", "")
        if not url.startswith("tcp://"):
            continue
        ssh_host = src.get("ssh_host")
        if not ssh_host:
            continue

        ssh_user = src.get("ssh_user", "ubuntu")
        ssh_pass = src.get("ssh_password", "")
        card = src.get("card", 1)
        sender_path = src.get("sender_path", "/home/ubuntu/audio_sender.py")
        port = url.rsplit(":", 1)[-1].split("/")[0] if ":" in url else "9800"
        name = src.get("name", ssh_host)

        def _ssh(cmd: str, timeout: int = 10) -> subprocess.CompletedProcess:
            return subprocess.run(
                ["sshpass", "-p", ssh_pass, "ssh",
                 "-o", "StrictHostKeyChecking=no", "-o", f"ConnectTimeout=3",
                 f"{ssh_user}@{ssh_host}", cmd],
                capture_output=True, text=True, timeout=timeout)

        # Check if already running
        try:
            result = _ssh("pgrep -f audio_sender.py")
            if result.returncode == 0 and result.stdout.strip():
                pid = result.stdout.strip().splitlines()[0]
                print(f"[audio_sender] {name}: already running on {ssh_host} (pid={pid})")
                continue
        except Exception as e:
            print(f"[audio_sender] {name}: WARNING: SSH check failed: {e}")
            continue

        # Deploy audio_sender.py if not present
        try:
            result = _ssh(f"test -f {sender_path} && echo EXISTS")
            if "EXISTS" not in (result.stdout or ""):
                local_src = str(Path(__file__).parent / "audio_sender.py")
                subprocess.run(
                    ["sshpass", "-p", ssh_pass, "scp",
                     "-o", "StrictHostKeyChecking=no",
                     local_src, f"{ssh_user}@{ssh_host}:{sender_path}"],
                    check=True, timeout=15)
                print(f"[audio_sender] {name}: deployed to {ssh_host}:{sender_path}")
        except Exception as e:
            print(f"[audio_sender] {name}: WARNING: deploy failed: {e}")
            continue

        # Start in background
        try:
            _ssh(f"nohup python3 {sender_path} --port {port} --card {card} "
                 f"> /tmp/audio_sender.log 2>&1 &")
            time.sleep(1)
            result = _ssh("pgrep -f audio_sender.py")
            if result.returncode == 0 and result.stdout.strip():
                pid = result.stdout.strip().splitlines()[0]
                print(f"[audio_sender] {name}: started on {ssh_host} (pid={pid}, port={port}, card={card})")
            else:
                print(f"[audio_sender] {name}: WARNING: did not start, check {ssh_host}:/tmp/audio_sender.log")
        except Exception as e:
            print(f"[audio_sender] {name}: WARNING: start failed: {e}")


def _resolve_namespace(cfg: dict) -> str:
    ns = cfg.get("ros_namespace", "").strip()
    if ns:
        return re.sub(r"[^a-zA-Z0-9_]", "_", ns)
    return re.sub(r"[^a-zA-Z0-9_]", "_", socket.gethostname())


# ── Dual Domain ROS2 Init ────────────────────────────────────────────────────

class DualDomainROS2:
    """Manages two ROS2 contexts: domain 0 (tianyi) and domain 42 (agent-core)."""

    def __init__(self):
        # Domain 0: connect to tianyi body controller
        # Use lyre's DDS profile so we can discover topics on 192.168.41.x / 127.0.0.1
        dds_profile = "/work/dds_profile.xml"
        if os.path.exists(dds_profile):
            os.environ["FASTRTPS_DEFAULT_PROFILES_FILE"] = dds_profile
            print(f"[ros2] domain0: using DDS profile {dds_profile}")
        self.ctx_tianyi = Context()
        rclpy.init(context=self.ctx_tianyi, domain_id=0)
        self.executor_tianyi = rclpy.executors.MultiThreadedExecutor(context=self.ctx_tianyi)

        # Domain 42: publish to agent-core (no DDS profile — use all interfaces)
        os.environ.pop("FASTRTPS_DEFAULT_PROFILES_FILE", None)
        self.ctx_core = Context()
        rclpy.init(context=self.ctx_core, domain_id=42)
        self.executor_core = rclpy.executors.MultiThreadedExecutor(context=self.ctx_core)

        self._spin_threads = []

    def start_spin(self):
        """Start spinning both executors in background threads."""
        def _spin(executor, name):
            try:
                while rclpy.ok(context=executor.context):
                    executor.spin_once(timeout_sec=0.1)
            except Exception:
                pass
            print(f"[ros2] {name} spin exited")

        t1 = threading.Thread(target=_spin, args=(self.executor_tianyi, "domain0"), daemon=True)
        t2 = threading.Thread(target=_spin, args=(self.executor_core, "domain42"), daemon=True)
        t1.start()
        t2.start()
        self._spin_threads = [t1, t2]

    def shutdown(self):
        self.executor_tianyi.shutdown()
        self.executor_core.shutdown()
        rclpy.shutdown(context=self.ctx_tianyi)
        rclpy.shutdown(context=self.ctx_core)


# ── Bundle ────────────────────────────────────────────────────────────────────

class TianyiDeviceBundle:
    def __init__(self, cfg: dict, namespace: str, ros2: DualDomainROS2, slamtec_client):
        self._cfg = cfg
        self._plugins: list = []
        plugins_cfg = cfg.get("plugins", {})

        if plugins_cfg.get("state", {}).get("enabled", False):
            from device import StatePlugin
            self._plugins.append(StatePlugin(plugins_cfg["state"], namespace, ros2))
            print("[bundle] StatePlugin loaded")

        if plugins_cfg.get("camera", {}).get("enabled", False):
            from device import CameraPlugin
            self._plugins.append(CameraPlugin(plugins_cfg["camera"], namespace, ros2))
            print("[bundle] CameraPlugin loaded")

        for config_name, class_name in (("imu", "ImuPlugin"),
                                        ("camera_depth", "DepthCameraPlugin"),
                                        ("camera_pointcloud", "PointCloudPlugin")):
            if plugins_cfg.get(config_name, {}).get("enabled", False):
                from device import ImuPlugin, DepthCameraPlugin, PointCloudPlugin
                plugin_class = {"ImuPlugin": ImuPlugin, "DepthCameraPlugin": DepthCameraPlugin,
                                "PointCloudPlugin": PointCloudPlugin}[class_name]
                self._plugins.append(plugin_class(plugins_cfg[config_name], namespace, ros2))
                print(f"[bundle] {class_name} loaded")

        if plugins_cfg.get("asr", {}).get("enabled", False):
            from device import AsrPlugin
            self._plugins.append(AsrPlugin(plugins_cfg["asr"], namespace, ros2))
            print("[bundle] AsrPlugin loaded")

        if plugins_cfg.get("nav_state", {}).get("enabled", False):
            from device import NavStatePlugin
            self._plugins.append(NavStatePlugin(plugins_cfg["nav_state"], namespace, ros2, slamtec_client))
            print("[bundle] NavStatePlugin loaded")

        if plugins_cfg.get("power_board", {}).get("enabled", False):
            from device import PowerBoardStatePlugin
            self._plugins.append(PowerBoardStatePlugin(plugins_cfg["power_board"], namespace, ros2))
            print("[bundle] PowerBoardStatePlugin loaded")

        if plugins_cfg.get("motors", {}).get("enabled", False):
            from device import MotorStatePlugin
            self._plugins.append(MotorStatePlugin(plugins_cfg["motors"], namespace, ros2))
            print("[bundle] MotorStatePlugin loaded")

        if plugins_cfg.get("hand_state", {}).get("enabled", False):
            from device import HandStatePlugin
            self._plugins.append(HandStatePlugin(plugins_cfg["hand_state"], namespace, ros2))
            print("[bundle] HandStatePlugin loaded")

        if plugins_cfg.get("remote_event", {}).get("enabled", False):
            from device import RemoteStatePlugin
            self._plugins.append(RemoteStatePlugin(plugins_cfg["remote_event"], namespace, ros2))
            print("[bundle] RemoteStatePlugin loaded")

        if plugins_cfg.get("head", {}).get("enabled", False):
            from device import HeadPlugin
            self._plugins.append(HeadPlugin(plugins_cfg["head"], namespace, ros2))
            print("[bundle] HeadPlugin loaded")

        if plugins_cfg.get("head_gesture", {}).get("enabled", False):
            from device import HeadGesturePlugin
            self._plugins.append(HeadGesturePlugin(
                plugins_cfg["head_gesture"], namespace, ros2))
            print("[bundle] HeadGesturePlugin loaded")

        if plugins_cfg.get("arm", {}).get("enabled", False):
            from device import ArmPlugin
            self._plugins.append(ArmPlugin(plugins_cfg["arm"], namespace, ros2))
            print("[bundle] ArmPlugin loaded")

        if plugins_cfg.get("arm_gesture", {}).get("enabled", False):
            from device import ArmGesturePlugin
            self._plugins.append(ArmGesturePlugin(
                plugins_cfg["arm_gesture"], namespace, ros2))
            print("[bundle] ArmGesturePlugin loaded")

        if plugins_cfg.get("waist", {}).get("enabled", False):
            from device import WaistPlugin
            self._plugins.append(WaistPlugin(plugins_cfg["waist"], namespace, ros2))
            print("[bundle] WaistPlugin loaded")

        if plugins_cfg.get("hand", {}).get("enabled", False):
            from device import HandPlugin
            self._plugins.append(HandPlugin(plugins_cfg["hand"], namespace, ros2))
            print("[bundle] HandPlugin loaded")

        if plugins_cfg.get("tts", {}).get("enabled", False):
            from device import TtsPlugin
            self._plugins.append(TtsPlugin(plugins_cfg["tts"], namespace, ros2))
            print("[bundle] TtsPlugin loaded")

        if plugins_cfg.get("voice_play", {}).get("enabled", False):
            from device import VoicePlayActuatorPlugin
            self._plugins.append(VoicePlayActuatorPlugin(plugins_cfg["voice_play"], namespace, ros2))
            print("[bundle] VoicePlayActuatorPlugin loaded")

        if plugins_cfg.get("nav", {}).get("enabled", False):
            from device import NavPlugin
            self._plugins.append(NavPlugin(plugins_cfg["nav"], namespace, ros2, slamtec_client))
            print("[bundle] NavPlugin loaded")

        if plugins_cfg.get("chat", {}).get("enabled", False):
            from device import ChatPlugin
            self._plugins.append(ChatPlugin(plugins_cfg["chat"], namespace, ros2))
            print("[bundle] ChatPlugin loaded")

        if plugins_cfg.get("voice_chat", {}).get("enabled", False):
            from device import VoiceChatActuatorPlugin
            self._plugins.append(VoiceChatActuatorPlugin(plugins_cfg["voice_chat"], namespace, ros2))
            print("[bundle] VoiceChatActuatorPlugin loaded")
        if plugins_cfg.get("controlled_spatial", {}).get("enabled", False):
            from controlled_spatial import ControlledSpatialPlugin
            self._plugins.append(ControlledSpatialPlugin(plugins_cfg["controlled_spatial"], namespace, ros2, slamtec_client))
            print("[bundle] ControlledSpatialPlugin loaded")

        if plugins_cfg.get("health_check", {}).get("enabled", False):
            from device import RobotFaultsPlugin
            self._plugins.append(RobotFaultsPlugin(plugins_cfg["health_check"], namespace, ros2, slamtec_client))
            print("[bundle] RobotFaultsPlugin loaded (health_check)")

        if plugins_cfg.get("laser_scan", {}).get("enabled", False):
            from device import LaserScanPlugin
            self._plugins.append(LaserScanPlugin(plugins_cfg["laser_scan"], namespace, ros2, slamtec_client))
            print("[bundle] LaserScanPlugin loaded")

        if plugins_cfg.get("chassis_raw", {}).get("enabled", False):
            from device import ChassisRawPlugin
            self._plugins.append(ChassisRawPlugin(plugins_cfg["chassis_raw"], namespace, ros2, slamtec_client))
            print("[bundle] ChassisRawPlugin loaded")

        if plugins_cfg.get("ext_mic", {}).get("enabled", False):
            from ext_devices import ExtMicPlugin
            self._plugins.append(ExtMicPlugin(plugins_cfg["ext_mic"], namespace, ros2.executor_core))
            print("[bundle] ExtMicPlugin loaded")

        if plugins_cfg.get("light", {}).get("enabled", False):
            from light import LightPlugin
            self._plugins.append(LightPlugin(plugins_cfg["light"], namespace, ros2))
            print("[bundle] LightPlugin loaded")

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
                    default_action = tool_def.get("default_action", "start")
                    action = args.pop("action", default_action)
                    args['_tool_name'] = tool_name
                    result = p.dispatch(action, args)
                    return result
        return None


# ── MCP HTTP server ───────────────────────────────────────────────────────────

_bundle: TianyiDeviceBundle | None = None


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
                        "serverInfo": {"name": _bundle._cfg.get("name", "tianyi2-device-bundle"), "version": "1.0.0"},
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
    mcp_port  = int(cfg.get("mcp_port", 15707))

    print(f"[bundle] namespace={namespace} mcp_port={mcp_port}")

    # Slamtec HTTP client
    slamtec_url = os.environ.get("SLAMTEC_URL", cfg.get("slamtec", {}).get("base_url", "http://192.168.11.1:1448"))
    from nav_client import SlamtecClient
    slamtec_client = SlamtecClient(slamtec_url)
    print(f"[bundle] Slamtec client → {slamtec_url}")

    # Ensure host lyre service is in audio mode (ASR + TTS, no built-in dialogue)
    _ensure_lyre_audio_mode()

    # Ensure TCP audio sender is running on host for DJI wireless mic
    _ensure_audio_sender(cfg)

    # Dual-domain ROS2
    ros2 = DualDomainROS2()
    ros2.start_spin()
    print("[bundle] Dual-domain ROS2 initialized (domain 0 + domain 42)")

    _bundle = TianyiDeviceBundle(cfg, namespace, ros2, slamtec_client)
    _bundle.start_all()

    _start_registration(mcp_port, cfg.get("name", "Tianyi 2.0 Pro"), "driver")

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
        ros2.shutdown()


if __name__ == "__main__":
    main()
