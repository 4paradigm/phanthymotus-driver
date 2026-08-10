#!/usr/bin/env python3
"""
drivers/unitree/g1/main.py — Unitree G1 设备 bundle 统一入口。

读取 config.yaml，按插件配置加载插件，聚合成一个 MCP HTTP server 对外暴露。
驱动启动时自动 start 所有插件，关闭时自动 stop。

MCP 工具命名规则：直接使用 tool name（mic, tts, led, loco, loco_state, arm, state）

用法：
    python3 main.py <networkInterface>

环境变量：
    CONFIG_PATH — config.yaml 路径（默认同目录下）
"""

from __future__ import annotations

import json
import os
import re
import signal
import socket
import ssl
import sys
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import rclpy
import rclpy.executors
import yaml
from rpc_proxy import RpcProxy
from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import (
    MotionSwitcherClient,
)
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.g1.arm.g1_arm_action_client import G1ArmActionClient
from unitree_sdk2py.g1.audio.g1_audio_client import AudioClient
from unitree_sdk2py.g1.slam.slam_client import SlamClient


# Kept lightweight until teleop is explicitly enabled. Ordinary legacy plugin
# exceptions must not be mistaken for teleoperation protocol errors.
class _TeleopDisabledProtocolError(Exception):
    pass


class _TeleopDisabledRtcError(Exception):
    pass


TeleopProtocolError = _TeleopDisabledProtocolError
RtcRequestError = _TeleopDisabledRtcError


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


def _best_effort_cleanup(label: str, operation) -> bool:
    """Run one shutdown operation without starving later safety cleanup."""

    try:
        operation()
    except Exception as exc:  # noqa: BLE001 -- shutdown must isolate each owner
        print(
            f"[bundle] {label} cleanup FAILED: {type(exc).__name__}: {exc}",
            flush=True,
        )
        import traceback

        traceback.print_exc()
        return False
    return True


# ── Bundle ────────────────────────────────────────────────────────────────────

class G1DeviceBundle:
    def __init__(self, cfg: dict, namespace: str, executor,
                 audio_client: AudioClient,
                 loco_client: RpcProxy,
                 arm_client: G1ArmActionClient,
                 slam_client: SlamClient,
                 msc_client: MotionSwitcherClient,
                 smart_motion=None,
                 network_iface: str = "eth0",
                 teleop_service=None):
        self._plugins: list = []
        self._smart_motion = smart_motion
        self._teleop_service = teleop_service
        self._stop_lock = threading.Lock()
        self._stopped = False
        plugins_cfg = cfg.get("plugins", {})
        if (
            self._teleop_service is not None
            and plugins_cfg.get("arm", {}).get("enabled") is True
        ):
            raise ValueError(
                "teleoperation and ArmActionPlugin cannot share G1 arm authority"
            )

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
            from device import LocoPlugin, LocoStatePlugin
            self._plugins.append(LocoStatePlugin(plugins_cfg["loco"], namespace, executor))
            self._plugins.append(LocoPlugin(plugins_cfg["loco"], namespace, executor, loco_client, slam_client=slam_client, smart_motion=smart_motion))
            print("[bundle] LocoStatePlugin + LocoPlugin loaded")

        if (
            plugins_cfg.get("arm", {}).get("enabled", False)
            and self._teleop_service is None
        ):
            from device import ArmActionPlugin
            self._plugins.append(ArmActionPlugin(plugins_cfg["arm"], namespace, executor, arm_client))
            print("[bundle] ArmActionPlugin loaded")

        if plugins_cfg.get("asr", {}).get("enabled", False):
            from device import AsrPlugin
            self._plugins.append(AsrPlugin(plugins_cfg["asr"], namespace, executor))
            print("[bundle] AsrPlugin loaded")

        if plugins_cfg.get("state", {}).get("enabled", False):
            from device import StatePlugin
            self._plugins.append(StatePlugin(plugins_cfg["state"], namespace, executor))
            print("[bundle] StatePlugin loaded")

        if plugins_cfg.get("camera", {}).get("enabled", False):
            from device import RealSensePlugin
            self._plugins.append(RealSensePlugin(plugins_cfg["camera"], namespace, executor))
            print("[bundle] RealSensePlugin loaded")

        if plugins_cfg.get("lidar", {}).get("enabled", False):
            from device import LidarPlugin
            self._plugins.append(LidarPlugin(plugins_cfg["lidar"], namespace, executor))
            print("[bundle] LidarPlugin loaded")

        if plugins_cfg.get("slam", {}).get("enabled", False):
            from device import SpatialPlugin
            self._plugins.append(SpatialPlugin(plugins_cfg["slam"], namespace, executor, slam_client, smart_motion=smart_motion))
            print("[bundle] SpatialPlugin loaded")

        if plugins_cfg.get("controlled_spatial", {}).get("enabled", False):
            from controlled_spatial import ControlledSpatialPlugin
            controlled_cfg = dict(plugins_cfg["controlled_spatial"])
            controlled_cfg["network_iface"] = network_iface
            self._plugins.append(ControlledSpatialPlugin(controlled_cfg, namespace, executor, slam_client, smart_motion=smart_motion))
            print("[bundle] ControlledSpatialPlugin loaded")

        if plugins_cfg.get("controlled_spatial_map", {}).get("enabled", False):
            try:
                from controlled_spatial_map import ControlledSpatialMapPlugin
                map_cfg = dict(plugins_cfg["controlled_spatial_map"])
                map_cfg["network_iface"] = network_iface
                self._plugins.append(ControlledSpatialMapPlugin(map_cfg, namespace, executor))
                print("[bundle] ControlledSpatialMapPlugin loaded")
            except Exception as e:
                print(f"[bundle] ControlledSpatialMapPlugin load skipped: {e}", flush=True)
                import traceback
                traceback.print_exc()

        if plugins_cfg.get("motion_switcher", {}).get("enabled", False):
            from device import MotionSwitcherPlugin
            self._plugins.append(MotionSwitcherPlugin(plugins_cfg["motion_switcher"], namespace, executor, msc_client))
            print("[bundle] MotionSwitcherPlugin loaded")

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
            self._plugins.append(SmartMotionPlugin(
                plugins_cfg.get("smart_motion", {}), namespace, executor,
                speaker_plugin=speaker_plugin,
                loco_plugin=loco_plugin,
            ))
            print(f"[bundle] SmartMotionPlugin loaded (speaker={'yes' if speaker_plugin else 'no'}, loco={'yes' if loco_plugin else 'no'})")

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
        with self._stop_lock:
            if self._stopped:
                return
            self._stopped = True
            if self._teleop_service is not None:
                _best_effort_cleanup(
                    "teleop service",
                    self._teleop_service.close,
                )
            for index, plugin in enumerate(self._plugins):
                _best_effort_cleanup(
                    f"plugin {index} ({type(plugin).__name__})",
                    plugin.stop,
                )
            print("[bundle] All plugins stopped", flush=True)

    def get_all_tools(self) -> list:
        tools = []
        for p in self._plugins:
            if hasattr(p, 'get_tools'):
                tools.extend(p.get_tools())
            else:
                tools.append(p.get_tool())
        if self._teleop_service is not None:
            tools.extend(self._teleop_service.get_tools())
        return tools

    def dispatch(self, tool_name: str, args: dict) -> dict | None:
        if self._teleop_service is not None:
            if tool_name in ("teleop_session", "teleop_state"):
                return self._teleop_service.dispatch(tool_name, args)
            if tool_name == "arm":
                return {
                    "ok": False,
                    "code": "teleop_arm_unavailable",
                    "error": "arm gestures are unavailable while teleoperation is enabled",
                }
        for p in self._plugins:
            plugin_tools = p.get_tools() if hasattr(p, 'get_tools') else [p.get_tool()]
            for tool_def in plugin_tools:
                if tool_def["name"] == tool_name:
                    if tool_def["type"] == "resource":
                        return p.dispatch(tool_name, args)
                    action = args.pop("action", tool_name)
                    args['_tool_name'] = tool_name  # let multi-tool plugins know which tool was called
                    result = p.dispatch(action, args)
                    return result
        return None


# ── MCP HTTP server ───────────────────────────────────────────────────────────

_bundle: G1DeviceBundle | None = None
_teleop_service = None


def make_handler():
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, fmt, *args):
            # Suppress routine request logs (info/heartbeat); only log errors and tool calls
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
            self.send_header("Access-Control-Allow-Headers", "Content-Type, Accept, Authorization")
            self.end_headers()
            self.wfile.write(encoded)

        def do_GET(self):
            if self.path == "/health" and _teleop_service is not None:
                if not _teleop_service.authorized(self.headers.get("Authorization")):
                    self._send(401, json.dumps({"error": "unauthorized"}))
                    return
                self._send(200, json.dumps(_teleop_service.health()))
                return
            self.send_response(404)
            self.end_headers()

        def do_OPTIONS(self):
            self.send_response(204)
            self.send_header("Access-Control-Allow-Origin", "*")
            self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
            self.send_header("Access-Control-Allow-Headers", "Content-Type, Accept, Authorization")
            self.end_headers()

        def do_POST(self):
            if _teleop_service is not None and not _teleop_service.authorized(
                self.headers.get("Authorization")
            ):
                self._send(401, json.dumps({"error": "unauthorized"}))
                return
            length = int(self.headers.get("Content-Length", 0))
            raw = self.rfile.read(length)
            try:
                rpc = json.loads(raw)
            except Exception:
                self._send(400, json.dumps({"jsonrpc": "2.0", "id": None,
                                             "error": {"code": -32700, "message": "Parse error"}}))
                return

            if self.path == "/offer":
                if _teleop_service is None:
                    self._send(404, json.dumps({"error": "teleop unavailable"}))
                    return
                try:
                    self._send(200, json.dumps(_teleop_service.accept_offer(rpc)))
                except RtcRequestError as exc:
                    self._send(
                        getattr(exc, "status", 400),
                        json.dumps({"error": {"code": getattr(exc, "code", "rtc_error"), "message": str(exc)}}),
                    )
                return
            if self.path != "/mcp":
                self._send(404, json.dumps({"error": "not found"}))
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

            def err(code, msg, data=None):
                error = {"code": code, "message": msg}
                if data is not None:
                    error["data"] = data
                self._send(200, json.dumps({
                    "jsonrpc": "2.0",
                    "id": rid,
                    "error": error,
                }))

            try:
                if method == "initialize":
                    ok({
                        "protocolVersion": "2024-11-05",
                        "capabilities": {"tools": {}},
                        "serverInfo": {"name": "g1-device-bundle", "version": "2.0.0"},
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
            except TeleopProtocolError as e:
                protocol_code = str(getattr(e, "code", "invalid_arguments"))[:64]
                rpc_code = (
                    -32601
                    if protocol_code in {"unknown_tool", "unknown_action"}
                    else -32602
                )
                err(rpc_code, str(e), {"code": protocol_code})
            except Exception as e:
                err(-32603, str(e))

    return Handler


# ── Entry point ───────────────────────────────────────────────────────────────


def build_registration_ssl_context(registration: dict) -> ssl.SSLContext:
    """Pin registration TLS to the deployed Agent Core certificate."""

    if registration.get("verify_tls", True) is not True:
        raise ValueError("G1 Driver registration requires TLS verification")
    ca_file = Path(
        str(registration.get("ca_file", "/etc/motus-core-certs/cert.pem"))
    )
    if not ca_file.is_file():
        raise ValueError(f"Agent Core pinned CA file is missing: {ca_file}")
    try:
        context = ssl.create_default_context(
            ssl.Purpose.SERVER_AUTH,
            cafile=str(ca_file),
        )
    except (OSError, ssl.SSLError) as exc:
        raise ValueError(f"Agent Core pinned CA file is invalid: {ca_file}") from exc
    # Core's deployed certificate is issued to `phanthy-motus`, while this
    # host-network Driver registers through localhost. Chain verification is
    # mandatory; only the DNS-name check is disabled.
    context.check_hostname = False
    context.verify_mode = ssl.CERT_REQUIRED
    return context


def _start_registration(
    mcp_port: int,
    name: str,
    category: str,
    *,
    cfg: dict,
    teleop_service=None,
):
    """Register this Driver with pinned TLS and its per-Driver Bearer."""

    import urllib.request as _urllib
    teleop_cfg = cfg.get("teleop", {})
    registration = (
        teleop_cfg.get("registration", {})
        if isinstance(teleop_cfg, dict)
        else {}
    )
    if not isinstance(registration, dict):
        raise ValueError("teleop.registration config must be an object")
    agent_core_url = str(
        registration.get(
            "agent_core_url",
            os.environ.get("AGENT_CORE_URL", "https://localhost:15678"),
        )
    ).rstrip("/")
    if not agent_core_url.startswith("https://"):
        raise ValueError("Agent Core registration URL must use https")
    advertise_url = str(
        registration.get("advertise_url", f"http://localhost:{mcp_port}/mcp")
    )
    driver_id = (
        teleop_service.runtime.driver_id
        if teleop_service is not None
        else os.environ.get("MOTUS_DRIVER_ID", "unitree-g1")
    )
    robot_id = (
        teleop_service.runtime.robot_id
        if teleop_service is not None
        else os.environ.get("MOTUS_ROBOT_ID", driver_id)
    )
    payload_value = {
        "id": driver_id,
        "robot_id": robot_id,
        "name": name,
        "url": advertise_url,
        "transport": "http",
        "category": category,
    }
    payload = json.dumps(payload_value).encode()
    headers = {"Content-Type": "application/json"}
    token = (
        teleop_service.driver_token
        if teleop_service is not None
        else os.environ.get("MOTUS_DRIVER_TOKEN")
    )
    if token:
        headers["Authorization"] = f"Bearer {token}"
    if teleop_service is not None and not token:
        raise ValueError("teleoperation registration requires MOTUS_DRIVER_TOKEN")
    try:
        context = build_registration_ssl_context(registration)
    except ValueError as exc:
        if teleop_service is not None:
            teleop_service.update_registration_status(
                state="tls_error",
                last_error=str(exc),
            )
            raise
        print(f"[register] disabled: {exc}")
        return {"state": "disabled_tls", "error": str(exc)[:256]}

    def _run():
        while True:
            if teleop_service is not None:
                status = teleop_service.registration_status
                teleop_service.update_registration_status(
                    state="registering",
                    attempts=int(status["attempts"]) + 1,
                    last_error=None,
                )
            try:
                req = _urllib.Request(
                    f"{agent_core_url}/api/mcp", data=payload,
                    headers=headers, method="POST",
                )
                with _urllib.urlopen(req, timeout=3, context=context) as response:
                    http_status = int(getattr(response, "status", 200))
                    response_body = response.read(64 * 1024 + 1)
                if len(response_body) > 64 * 1024:
                    raise ValueError("registration response exceeds 64 KiB")
                if teleop_service is not None:
                    try:
                        decoded = json.loads(response_body)
                        data = decoded["data"]
                        trusted = (
                            isinstance(decoded, dict)
                            and isinstance(data, dict)
                            and data.get("id") == driver_id
                            and data.get("trust_state") == "trusted"
                        )
                    except (KeyError, TypeError, ValueError, json.JSONDecodeError):
                        trusted = False
                    if trusted:
                        status = teleop_service.registration_status
                        teleop_service.update_registration_status(
                            state="registered",
                            successes=int(status["successes"]) + 1,
                            last_http_status=http_status,
                            last_error=None,
                        )
                        wait_seconds = 30.0
                    else:
                        teleop_service.update_registration_status(
                            state="trust_error",
                            last_http_status=http_status,
                            last_error="Agent Core did not confirm the requested trusted identity",
                        )
                        wait_seconds = 5.0
                else:
                    wait_seconds = 30.0
            except Exception as e:
                print(f"[register] failed: {e}, retrying in 5s")
                if teleop_service is not None:
                    status_code = getattr(e, "code", None)
                    teleop_service.update_registration_status(
                        state=("http_error" if status_code is not None else "connection_error"),
                        last_http_status=(
                            int(status_code) if isinstance(status_code, int) else None
                        ),
                        last_error=f"{type(e).__name__}: {e}",
                    )
                wait_seconds = 5.0
            if teleop_service is not None:
                if teleop_service.registration_wait(wait_seconds):
                    return
            else:
                threading.Event().wait(wait_seconds)

    if teleop_service is not None:
        teleop_service.launch_registration_worker(_run)
    else:
        threading.Thread(target=_run, daemon=True, name="g1-register").start()
    return {
        "endpoint": f"{agent_core_url}/api/mcp",
        "payload": payload_value,
        "tls_verify_mode": context.verify_mode,
    }


def main():
    global _bundle, _teleop_service, TeleopProtocolError, RtcRequestError

    if len(sys.argv) < 2:
        print(f"Usage: python3 {sys.argv[0]} <networkInterface>")
        sys.exit(1)

    network_iface = sys.argv[1]
    cfg           = _load_config()
    namespace     = _resolve_namespace(cfg)
    mcp_port      = int(cfg.get("mcp_port", 15701))
    _bundle = None
    _teleop_service = None
    loco_client = None
    executor = None
    smart_motion = None
    server = None
    ros_initialized = False

    try:
        print(f"[bundle] namespace={namespace} mcp_port={mcp_port}")

        # DDS init
        ChannelFactoryInitialize(0, network_iface)
        print(f"[bundle] DDS initialized on interface: {network_iface}")

        teleop_cfg = cfg.get("teleop")
        teleop_requested = (
            isinstance(teleop_cfg, dict) and teleop_cfg.get("enabled") is True
        )
        if teleop_requested:
            from teleop.factory import build_g1_teleop_service, project_preflight_error
            from teleop.protocol import ProtocolError as _TeleopProtocolError
            from teleop.rtc import RtcRequestError as _RtcRequestError

            TeleopProtocolError = _TeleopProtocolError
            RtcRequestError = _RtcRequestError
            requested_mode = str(teleop_cfg.get("mode", "shadow"))
            try:
                _teleop_service = build_g1_teleop_service(cfg)
                if _teleop_service is None:
                    raise RuntimeError(
                        "teleop was enabled but its factory returned no service"
                    )
            except Exception as exc:
                print(
                    "[teleop-preflight] "
                    + json.dumps(
                        project_preflight_error(exc, mode=requested_mode),
                        allow_nan=False,
                        separators=(",", ":"),
                        sort_keys=True,
                    ),
                    flush=True,
                )
                return 1
            print(
                "[teleop-preflight] "
                + json.dumps(
                    _teleop_service.preflight_status(),
                    allow_nan=False,
                    separators=(",", ":"),
                    sort_keys=True,
                ),
                flush=True,
            )
            print(
                f"[bundle] G1 teleoperation initialized "
                f"(mode={_teleop_service.runtime.mode}, "
                f"profile={_teleop_service.runtime.profile_id})"
            )

        # Suppress C++ layer stdout (ClientStub recv/future logs) while keeping
        # Python print working. C++ writes directly to fd 1.
        original_fd = os.dup(1)
        devnull = os.open(os.devnull, os.O_WRONLY)
        os.dup2(devnull, 1)
        os.close(devnull)
        sys.stdout = os.fdopen(original_fd, "w", buffering=1)

        # AudioClient (shared by tts + led)
        audio_client = AudioClient()
        audio_client.SetTimeout(10.0)
        audio_client.Init()
        print("[bundle] AudioClient ready")

        # LocoClient (locomotion control) — via subprocess proxy to avoid GIL contention
        loco_client = RpcProxy(network_iface)
        print("[bundle] LocoClient ready (subprocess proxy)")

        # Firmware arm gestures and teleoperation are mutually exclusive owners.
        arm_client = None
        if (
            _teleop_service is None
            and cfg.get("plugins", {}).get("arm", {}).get("enabled", False)
        ):
            arm_client = G1ArmActionClient()
            arm_client.SetTimeout(10.0)
            arm_client.Init()
            print("[bundle] G1ArmActionClient ready")

        # SlamClient (SLAM navigation)
        slam_client = SlamClient()
        slam_client.SetTimeout(5.0)
        slam_client.Init()
        print("[bundle] SlamClient ready")

        # MotionSwitcherClient
        msc_client = MotionSwitcherClient()
        msc_client.SetTimeout(5.0)
        msc_client.Init()
        print("[bundle] MotionSwitcherClient ready")

        # ROS2
        rclpy.init()
        ros_initialized = True
        executor = rclpy.executors.MultiThreadedExecutor()

        # Safety Harness (SmartMotion) — independent subprocess
        harness_cfg = cfg.get("safety_harness", {})
        if harness_cfg.get("enabled", True):
            from safety_harness import SmartMotionProxy

            smart_motion = SmartMotionProxy(namespace, harness_cfg, network_iface)
            print("[bundle] SmartMotion safety harness active (subprocess)")

        _bundle = G1DeviceBundle(
            cfg,
            namespace,
            executor,
            audio_client,
            loco_client,
            arm_client,
            slam_client,
            msc_client,
            smart_motion=smart_motion,
            network_iface=network_iface,
            teleop_service=_teleop_service,
        )
        _bundle.start_all()

        def _spin():
            while rclpy.ok():
                executor.spin_once(timeout_sec=0.1)

        spin_thread = threading.Thread(
            target=_spin,
            daemon=True,
            name="bundle_spin",
        )
        spin_thread.start()

        _start_registration(
            mcp_port,
            cfg.get("name", "Unitree G1"),
            "driver",
            cfg=cfg,
            teleop_service=_teleop_service,
        )

        server = ThreadingHTTPServer(("", mcp_port), make_handler())
        print(f"[bundle] MCP server → http://localhost:{mcp_port}")
        shutdown_requested = threading.Event()

        def _shutdown(signum, frame):
            del frame
            print(f"[bundle] signal {signum}, shutting down")
            if shutdown_requested.is_set():
                return
            shutdown_requested.set()
            threading.Thread(target=server.shutdown, daemon=True).start()

        signal.signal(signal.SIGTERM, _shutdown)
        signal.signal(signal.SIGINT, _shutdown)
        server.serve_forever()
    finally:
        # The teleop service owns the sole possible arm publisher. Close it
        # independently and before every legacy subsystem, even when bundle or
        # server construction never completed.
        if _teleop_service is not None:
            _best_effort_cleanup("teleop service", _teleop_service.close)
        if _bundle is not None:
            _best_effort_cleanup("device bundle", _bundle.stop_all)
        if smart_motion is not None:
            _best_effort_cleanup("SmartMotion", smart_motion.shutdown)
        if loco_client is not None:
            _best_effort_cleanup("LocoClient", loco_client.stop)
        if server is not None:
            _best_effort_cleanup("MCP server", server.server_close)
        if executor is not None:
            _best_effort_cleanup("ROS executor", executor.shutdown)
        if ros_initialized:
            _best_effort_cleanup("ROS", rclpy.shutdown)


if __name__ == "__main__":
    raise SystemExit(main())
