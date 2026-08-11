#!/usr/bin/env python3
"""
x-humanoid/bumi/main.py — Bumi Edu Max 设备 bundle 统一入口。

读取 config.yaml，按插件配置加载插件，聚合成一个 MCP HTTP server 对外暴露。
驱动启动时自动 start 所有插件，关闭时自动 stop。

通信架构：
  - Bumi SDK (C++ pybind11): HighController / LowController / MediaController
    通过 CycloneDDS 与运控板通信（500Hz 状态推送 + 控制指令下发）
  - Agent Core: 通过 HTTP JSON-RPC (MCP) 暴露工具，传感器数据通过 topic 输出

用法：
    python3 main.py

环境变量：
    CONFIG_PATH — config.yaml 路径（默认同目录下）
    AGENT_CORE_URL — Agent Core 注册地址（默认 https://localhost:15678）
"""

import json
import os
import queue as _queue
import re
import signal
import socket
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

# ── SSE event bus (thread-safe) ─────────────────────────────────────────────

_sse_clients: list[_queue.Queue] = []
_sse_lock = threading.Lock()


def sse_push(event: dict):
    """线程安全地广播 SSE 事件到所有连接的客户端。"""
    data = json.dumps(event, ensure_ascii=False)
    with _sse_lock:
        dead = []
        for q in _sse_clients:
            try:
                q.put_nowait(data)
            except _queue.Full:
                dead.append(q)
        for q in dead:
            _sse_clients.remove(q)


import yaml

from device import set_publish_fn

# Wire SSE publish function so camera plugins can push data to Agent Core
set_publish_fn(sse_push)

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


# ── Bumi SDK 初始化 ──────────────────────────────────────────────────────────


class BumiSDK:
    """初始化 Bumi SDK 的三个 Controller 单例，供所有插件共享。"""

    def __init__(self, cfg: dict):
        self._cfg = cfg
        self._high = None
        self._low = None
        self._media = None
        self._aolion = None

        # 配置 CycloneDDS
        dds_cfg = cfg.get("dds", {})
        dds_file = dds_cfg.get("config_file", "config/dds.xml")
        dds_path = Path(__file__).parent / dds_file
        if dds_path.exists():
            os.environ["CYCLONEDDS_URI"] = f"file://{dds_path}"
            print(f"[sdk] CycloneDDS config → {dds_path}")

        # 加载 pybind11 库
        sdk_cfg = cfg.get("sdk", {})
        build_path = sdk_cfg.get("build_path", str(Path(__file__).parent / "build"))
        if build_path not in sys.path:
            sys.path.insert(0, build_path)

    def init_high_controller(self):
        """初始化 HighController（高层动作控制）。"""
        if self._high is not None:
            return self._high
        try:
            from highcontrol_py import HighController
            self._high = HighController.instance()
            self._high.init()
            print("[sdk] HighController initialized")
        except ImportError as e:
            print(f"[sdk] WARNING: highcontrol_py import failed ({e}), running in stub mode")
            self._high = None
        return self._high

    def init_low_controller(self):
        """初始化 LowController（底层电机控制，可选）。"""
        if self._low is not None:
            return self._low
        try:
            from lowcontrol_py import LowController
            self._low = LowController.instance()
            self._low.init()
            print("[sdk] LowController initialized")
        except ImportError as e:
            print(f"[sdk] WARNING: lowcontrol_py import failed ({e}), running in stub mode")
            self._low = None
        return self._low

    def init_media_controller(self):
        """初始化 MediaController（语音/音视频系统）。"""
        if self._media is not None:
            return self._media
        try:
            from mediacontrol_py import MediaController
            self._media = MediaController.instance()
            self._media.init()
            print("[sdk] MediaController initialized")
        except ImportError as e:
            print(f"[sdk] WARNING: mediacontrol_py import failed ({e}), running in stub mode")
            self._media = None
        return self._media

    def init_aolion_driver(self):
        """初始化手柄直连驱动（有算力板时手柄插算力板 USB）。"""
        if self._aolion is not None:
            return self._aolion
        joy_cfg = self._cfg.get("joystick_direct", {})
        if not joy_cfg.get("enabled", False):
            return None
        try:
            from highcontrol_py import AoLionDriver
            self._aolion = AoLionDriver()
            port = joy_cfg.get("port", "/dev/input/js0")
            baudrate = joy_cfg.get("baudrate", 115200)
            self._aolion.init(port, baudrate)
            print(f"[sdk] AoLionDriver initialized on {port}")
        except ImportError as e:
            print(f"[sdk] WARNING: AoLionDriver import failed ({e})")
            self._aolion = None
        except Exception as e:
            print(f"[sdk] WARNING: AoLionDriver init failed ({e})")
            self._aolion = None
        return self._aolion

    @property
    def high(self):
        return self._high

    @property
    def low(self):
        return self._low

    @property
    def media(self):
        return self._media

    @property
    def aolion(self):
        return self._aolion


# ── Bundle ────────────────────────────────────────────────────────────────────


class BumiDeviceBundle:
    """按 config.yaml 加载所有插件，聚合成 MCP 工具集。"""

    def __init__(self, cfg: dict, namespace: str, sdk: BumiSDK):
        self._cfg = cfg
        self._ns = namespace
        self._sdk = sdk
        self._plugins: list = []
        plugins_cfg = cfg.get("plugins", {})

        # 导入所有插件类 (一次导入，避免重复 import)
        from device import (
            StatePlugin, CameraPlugin, DepthCameraPlugin, PointCloudPlugin,
            RemoteEventPlugin, MotorsPlugin, JoystickDirectPlugin, AsrPlugin,
            StandPlugin, WalkPlugin, ArmGesturePlugin, DancePlugin,
            TeachPlugin, FallRecoveryPlugin, TtsPlugin, VoicePlayPlugin,
            ChatPlugin, VoiceChatPlugin, RLPolicyPlugin,
        )
        from media_plugins import (
            MediaSystemStatusPlugin, MediaSystemErrorPlugin,
            MicrophonePlugin, SpeakerAudioPlugin,
            VideoCapturePlugin, VideoDesensedPlugin,
            WakewordPlugin, VolumePlugin, TimeoutConfigPlugin,
            BeepSwitchPlugin, AudioRoutingPlugin,
            AudioCaptureControlPlugin, AudioPlaybackControlPlugin,
            VideoCaptureControlPlugin,
            ExternalAudioInputPlugin, ExternalAudioOutputPlugin,
            ExternalVideoInputPlugin,
        )

        def _load(mapping: list[tuple[str, type]]):
            """按 config_key → class 映射加载已启用的插件。"""
            for config_key, cls in mapping:
                if plugins_cfg.get(config_key, {}).get("enabled", False):
                    self._plugins.append(cls(plugins_cfg[config_key], namespace, sdk))
                    print(f"[bundle] {cls.__name__} loaded")

        # ── 传感器插件 ──
        _load([
            ("state", StatePlugin),
            ("camera_head", CameraPlugin),
            ("camera_depth", DepthCameraPlugin),
            ("camera_pointcloud", PointCloudPlugin),
            ("remote_event", RemoteEventPlugin),
            ("motors", MotorsPlugin),
            ("joystick_direct", JoystickDirectPlugin),
            ("asr", AsrPlugin),
        ])

        # ── MediaController 传感器插件 ──
        _load([
            ("media_system_status", MediaSystemStatusPlugin),
            ("media_system_error", MediaSystemErrorPlugin),
            ("microphone", MicrophonePlugin),
            ("speaker_audio", SpeakerAudioPlugin),
            ("video_capture", VideoCapturePlugin),
            ("video_desensed", VideoDesensedPlugin),
        ])

        # ── 执行器插件 ──
        _load([
            ("stand", StandPlugin),
            ("walk", WalkPlugin),
            ("arm_gesture", ArmGesturePlugin),
            ("dance", DancePlugin),
            ("teach", TeachPlugin),
            ("fall_recovery", FallRecoveryPlugin),
            ("tts", TtsPlugin),
            ("voice_play", VoicePlayPlugin),
            ("chat", ChatPlugin),
            ("voice_chat", VoiceChatPlugin),
        ])

        # ── MediaController 执行器插件 ──
        _load([
            ("wakeword", WakewordPlugin),
            ("volume", VolumePlugin),
            ("timeout_config", TimeoutConfigPlugin),
            ("beep_switch", BeepSwitchPlugin),
            ("audio_routing", AudioRoutingPlugin),
            ("audio_capture_control", AudioCaptureControlPlugin),
            ("audio_playback_control", AudioPlaybackControlPlugin),
            ("video_capture_control", VideoCaptureControlPlugin),
            ("external_audio_input", ExternalAudioInputPlugin),
            ("external_audio_output", ExternalAudioOutputPlugin),
            ("external_video_input", ExternalVideoInputPlugin),
        ])

        # ── 可选扩展插件 ──
        if plugins_cfg.get("rl_policy", {}).get("enabled", False):
            self._plugins.append(RLPolicyPlugin(plugins_cfg["rl_policy"], namespace, sdk))
            print("[bundle] RLPolicyPlugin loaded")

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
                    default_action = tool_def.get("default_action", "start")
                    action = args.pop("action", default_action)
                    args["_tool_name"] = tool_name
                    result = p.dispatch(action, args)
                    return result
        return None


# ── MCP HTTP server ───────────────────────────────────────────────────────────

_bundle: BumiDeviceBundle | None = None


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
            if self.path.split("?")[0] == "/sse":
                self.send_response(200)
                self.send_header("Content-Type", "text/event-stream")
                self.send_header("Cache-Control", "no-cache")
                self.send_header("Connection", "keep-alive")
                self.send_header("Access-Control-Allow-Origin", "*")
                self.end_headers()

                client_queue = _queue.Queue(maxsize=64)
                with _sse_lock:
                    _sse_clients.append(client_queue)
                try:
                    while True:
                        try:
                            data = client_queue.get(timeout=30)
                            self.wfile.write(f"data: {data}\n\n".encode())
                            self.wfile.flush()
                        except _queue.Empty:
                            self.wfile.write(b": ping\n\n")
                            self.wfile.flush()
                except (BrokenPipeError, ConnectionResetError, OSError):
                    pass
                finally:
                    with _sse_lock:
                        if client_queue in _sse_clients:
                            _sse_clients.remove(client_queue)
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
            length = int(self.headers.get("Content-Length", 0))
            raw = self.rfile.read(length)
            try:
                rpc = json.loads(raw)
            except Exception:
                self._send(400, json.dumps({"jsonrpc": "2.0", "id": None,
                                            "error": {"code": -32700, "message": "Parse error"}}))
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
                self._send(200, json.dumps({"jsonrpc": "2.0", "id": rid,
                                            "error": {"code": code, "message": msg}}))

            try:
                if method == "initialize":
                    ok({
                        "protocolVersion": "2024-11-05",
                        "capabilities": {"tools": {}},
                        "serverInfo": {
                            "name": _bundle._cfg.get("name", "bumi-device-bundle"),
                            "version": "1.0.0",
                        },
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
    mcp_port = int(cfg.get("mcp_port", 15709))

    print(f"[bundle] namespace={namespace} mcp_port={mcp_port}")

    # 初始化 Bumi SDK
    sdk = BumiSDK(cfg)
    sdk.init_high_controller()
    sdk.init_media_controller()
    sdk.init_aolion_driver()

    _bundle = BumiDeviceBundle(cfg, namespace, sdk)
    _bundle.start_all()

    _start_registration(mcp_port, cfg.get("name", "Bumi Edu Max"), "driver")

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


if __name__ == "__main__":
    main()
