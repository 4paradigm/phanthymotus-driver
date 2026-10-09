"""
ext_devices.py — Go1 外部设备通信卡合集(actuator)。

合并了以下外部设备卡（每张卡行为不变）：
  - beep: 头部扬声器蜂鸣控制（HTTP 到 Nano beep_adapter :18082）
  - speaker: 头部扬声器音频流播放（ROS2 订阅 + TCP 二进制帧到 Nano speaker_adapter）
  - face_light: 面部灯带静态/逐灯/定时灯效（统一官方 SDK）
  - system_health: 机器人整体健康检查（CPU/内存/磁盘/电池/MQTT）


这些卡不共享 SDK client 控制通路，各自通过独立协议(Nano HTTP / ROS2+TCP / MQTT / UDP)通信。
"""

from __future__ import annotations

import json
import math
import os
import re
import signal
import ssl
import socket
import struct
import subprocess
import threading
import time
import urllib.parse
import urllib.request
from uuid import uuid4

try:
    import paho.mqtt.client as mqtt
    _HAS_MQTT = True
except Exception:
    _HAS_MQTT = False

try:
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import String
    _HAS_ROS2 = True
    _QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                      history=HistoryPolicy.KEEP_LAST, depth=200,
                      durability=DurabilityPolicy.VOLATILE)
    _ALARM_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                            history=HistoryPolicy.KEEP_LAST, depth=200,
                            durability=DurabilityPolicy.VOLATILE)
    _MIC_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                          history=HistoryPolicy.KEEP_LAST, depth=50,
                          durability=DurabilityPolicy.VOLATILE)
except Exception:
    _HAS_ROS2 = False

# ============================================================================
# beep — 头部扬声器蜂鸣控制
# ============================================================================

CARD_BEEP = "beep"


def _now_ms():
    return int(time.time() * 1000)


def _fail_beep(action, request_id, code, message, retryable=False, details=None):
    return {"ok": False, "card": CARD_BEEP, "action": action, "request_id": request_id,
            "code": code, "message": message, "details": details or {},
            "retryable": retryable, "timestamp_ms": _now_ms()}


class _BeepAdapterClient:
    def __init__(self, config: dict):
        self.base_url = (config.get("adapter_url")
                         or "http://%s:%s/v1" % (config.get("adapter_host", "192.168.123.13"),
                                                 config.get("adapter_port", 18082)))
        self.base_url = self.base_url.rstrip("/")
        self.timeout = float(config.get("rpc_timeout_sec", 2.0))

    def request(self, path: str, payload: dict) -> dict:
        data = json.dumps(payload).encode("utf-8")
        req = urllib.request.Request(self.base_url + path, data=data,
                                     headers={"Content-Type": "application/json"}, method="POST")
        try:
            with urllib.request.urlopen(req, timeout=self.timeout) as r:
                return json.loads(r.read().decode("utf-8"))
        except (OSError, ValueError) as exc:
            raise ConnectionError(str(exc)) from exc


class BeepPlugin:
    def __init__(self, plugin_config, namespace, executor):
        self._config = plugin_config or {}
        self._client = _BeepAdapterClient(self._config)
        self._alarm_state = None
        self._alarm_pub = None
        if _HAS_ROS2 and executor is not None:
            try:
                self._alarm_node = Node("go1_%s_alarms" % CARD_BEEP)
                self._alarm_pub = self._alarm_node.create_publisher(
                    String, "/%s/state/device_alarms" % namespace, _QOS)
                executor.add_node(self._alarm_node)
            except Exception as e:
                print(f"[{CARD_BEEP}] ROS2 告警不可用（不影响 beep）: {e}", flush=True)
                self._alarm_pub = None

    def _alarm(self, code, message, retryable):
        if self._alarm_pub is None or self._alarm_state == code:
            return
        self._alarm_state = code
        now = _now_ms()
        self._alarm_pub.publish(String(data=json.dumps({
            "alarm_id": "%s-%s-001" % (CARD_BEEP, code), "active": True, "severity": "error",
            "card": CARD_BEEP, "code": code, "message": message, "first_seen_ms": now,
            "last_seen_ms": now, "recovered_at_ms": None, "retryable": retryable, "details": {}})))

    def _clear_alarm(self):
        if self._alarm_pub is None or not self._alarm_state:
            return
        code, self._alarm_state, now = self._alarm_state, None, _now_ms()
        self._alarm_pub.publish(String(data=json.dumps({
            "alarm_id": "%s-%s-001" % (CARD_BEEP, code), "active": False, "severity": "error",
            "card": CARD_BEEP, "code": code, "message": "condition recovered", "first_seen_ms": now,
            "last_seen_ms": now, "recovered_at_ms": now, "retryable": False, "details": {}})))

    def _call(self, action, args) -> dict:
        request_id = args.get("request_id")
        payload = {k: v for k, v in args.items() if not k.startswith("_")}
        payload["action"], payload["request_id"] = action, request_id
        try:
            result = self._client.request("/%s/actions" % CARD_BEEP, payload)
        except ConnectionError:
            self._alarm("COMMUNICATION_ERROR", "beep adapter is unreachable", True)
            return _fail_beep(action, request_id, "COMMUNICATION_ERROR",
                              "beep adapter is unreachable", True)
        if result.get("ok"):
            self._clear_alarm()
        else:
            self._alarm(result.get("code", "INTERNAL_ERROR"),
                        result.get("message", "beep adapter request failed"),
                        result.get("retryable", False))
        return result

    def get_tool(self):
        return {"name": CARD_BEEP, "type": "actuator", "multiInstance": False,
          "description": "Go1 头部扬声器蜂鸣控制：播放蜂鸣音、调节音量",
          "inputSchema": {"type": "object",
            "properties": {
              "action": {"type": "string", "enum": ["beep", "set_volume", "get_volume"],
                         "description": "要执行的蜂鸣操作"},
              "request_id": {"type": "string"},
              "duration_sec": {"type": "number", "minimum": 0.1, "maximum": 10,
                               "description": "蜂鸣时长（秒，0.1–10）"},
              "frequency_hz": {"type": "number", "minimum": 100, "maximum": 8000,
                               "description": "蜂鸣频率（Hz，100–8000，默认 1000）"},
              "volume_percent": {"type": "integer", "minimum": 0, "maximum": 100,
                                 "description": "音量百分比 0–100（set_volume 用）"}},
            "required": ["action"],
            "x-action-params": {
              "beep": {"params": ["duration_sec", "frequency_hz"],
                       "description": "播放指定时长和频率的蜂鸣音"},
              "set_volume": {"params": ["volume_percent"], "description": "设置扬声器音量 0–100%"},
              "get_volume": {"params": [], "description": "读取当前扬声器音量"}}}}

    def start(self):
        pass

    def stop(self):
        pass

    def dispatch(self, action, args):
        rid = args.get("request_id")
        if action == "beep":
            dur = args.get("duration_sec", 0.3)
            freq = args.get("frequency_hz", 1000)
            if not isinstance(dur, (int, float)) or isinstance(dur, bool) or not 0 < dur <= 10:
                return _fail_beep(action, rid, "INVALID_ARGUMENT", "duration_sec must be a number in (0, 10]")
            if not isinstance(freq, (int, float)) or isinstance(freq, bool) or not 100 <= freq <= 8000:
                return _fail_beep(action, rid, "INVALID_ARGUMENT", "frequency_hz must be a number in [100, 8000]")
            return self._call(action, args)
        if action == "set_volume" and (type(args.get("volume_percent")) is not int
                                        or not 0 <= args["volume_percent"] <= 100):
            return _fail_beep(action, rid, "INVALID_ARGUMENT", "volume_percent must be an integer from 0 to 100")
        if action not in ("set_volume", "get_volume"):
            return _fail_beep(action, rid, "INVALID_ARGUMENT", "unsupported beep action")
        return self._call(action, args)


def make_beep(plugin_config, namespace, executor, client):
    return BeepPlugin(plugin_config, namespace, executor)


# ============================================================================
# speaker — 头部扬声器音频流播放
# ============================================================================

CARD_SPEAKER = "speaker"
_FRAME_PCM = 0x01


def _sr_ch_from_format(fmt: str):
    f = (fmt or "").lower()
    sr = 48000 if "48k" in f else (8000 if "8k" in f else 16000)
    ch = 2 if "stereo" in f else 1
    return sr, ch


class _TcpLink:
    def __init__(self, host: str, port: int, connect_timeout: float = 2.0):
        self._host = host
        self._port = port
        self._timeout = connect_timeout
        self._sock: socket.socket | None = None
        self._lock = threading.Lock()

    def _ensure(self) -> socket.socket:
        if self._sock is not None:
            return self._sock
        s = socket.create_connection((self._host, self._port), timeout=self._timeout)
        s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        s.settimeout(5.0)
        self._sock = s
        return s

    def send_pcm(self, sr: int, ch: int, pcm: bytes) -> None:
        frame_size = 8 + len(pcm)
        header = struct.pack(">IHBb", frame_size, sr, ch, _FRAME_PCM)
        frame = header + pcm
        with self._lock:
            try:
                sock = self._ensure()
                sock.sendall(frame)
            except (OSError, socket.error) as exc:
                self._close_unlocked()
                raise ConnectionError(str(exc)) from exc

    def close(self) -> None:
        with self._lock:
            self._close_unlocked()

    def _close_unlocked(self) -> None:
        s, self._sock = self._sock, None
        if s is not None:
            try:
                s.close()
            except Exception:
                pass


class _HttpCtrlClient:
    def __init__(self, config: dict):
        self.base_url = (config.get("adapter_url")
                         or "http://%s:%s/v1" % (config.get("adapter_host", "192.168.123.13"),
                                                  config.get("adapter_port", 18083)))
        self.base_url = self.base_url.rstrip("/")
        self.timeout = float(config.get("rpc_timeout_sec", 5.0))

    def request(self, path: str, payload: dict) -> dict:
        data = json.dumps(payload).encode("utf-8")
        req = urllib.request.Request(self.base_url + path, data=data,
                                     headers={"Content-Type": "application/json"}, method="POST")
        try:
            with urllib.request.urlopen(req, timeout=self.timeout) as r:
                return json.loads(r.read().decode("utf-8"))
        except (OSError, ValueError) as exc:
            raise ConnectionError(str(exc)) from exc


class SpeakerPlugin:
    def __init__(self, plugin_config, namespace, executor):
        self._config = plugin_config or {}
        self._ns = namespace
        self._executor = executor
        self._topic = self._config.get("mic_topic", "/remote_control/mic")
        self._max_buffer_bytes = int(self._config.get("max_buffer_bytes", 3200))
        self._playing = False
        self._alive = True
        self._buf = bytearray()
        self._buf_lock = threading.Lock()
        self._data_event = threading.Event()
        self._sr, self._ch = 16000, 1
        _host = self._config.get("adapter_host", "192.168.123.13")
        _stream_port = int(self._config.get("stream_port", 18084))
        self._tcp = _TcpLink(_host, _stream_port)
        self._http = _HttpCtrlClient(self._config)
        self._node = None
        self._sub = None
        self._alarm_pub = None
        self._alarm_state = None
        if _HAS_ROS2 and executor is not None:
            try:
                self._node = Node("go1_%s" % CARD_SPEAKER)
                self._alarm_pub = self._node.create_publisher(
                    String, "/%s/state/device_alarms" % namespace, _ALARM_QOS)
                try:
                    from audio_msgs.msg import AudioChunk
                    self._sub = self._node.create_subscription(
                        AudioChunk, self._topic, self._on_audio, _MIC_QOS)
                    print(f"[{CARD_SPEAKER}] subscribed {self._topic} (idle until play)", flush=True)
                except Exception as e:
                    print(f"[{CARD_SPEAKER}] audio_msgs 不可用，无法订阅 {self._topic}: {e}"
                          f"（需 source /ros_ws/install/setup.bash）", flush=True)
                    self._sub = None
                executor.add_node(self._node)
            except Exception as e:
                print(f"[{CARD_SPEAKER}] ROS2 不可用: {e}", flush=True)
                self._node = None
                self._alarm_pub = None
                self._sub = None
        self._writer_thread = threading.Thread(target=self._writer_loop, name="go1_speaker_writer", daemon=True)
        self._writer_thread.start()

    def _alarm(self, code, message, retryable):
        if self._alarm_pub is None or self._alarm_state == code:
            return
        self._alarm_state = code
        now = _now_ms()
        self._alarm_pub.publish(String(data=json.dumps({
            "alarm_id": "%s-%s-001" % (CARD_SPEAKER, code), "active": True, "severity": "error",
            "card": CARD_SPEAKER, "code": code, "message": message, "first_seen_ms": now,
            "last_seen_ms": now, "recovered_at_ms": None, "retryable": retryable, "details": {}})))

    def _clear_alarm(self):
        if self._alarm_pub is None or not self._alarm_state:
            return
        code, self._alarm_state, now = self._alarm_state, None, _now_ms()
        self._alarm_pub.publish(String(data=json.dumps({
            "alarm_id": "%s-%s-001" % (CARD_SPEAKER, code), "active": False, "severity": "error",
            "card": CARD_SPEAKER, "code": code, "message": "condition recovered", "first_seen_ms": now,
            "last_seen_ms": now, "recovered_at_ms": now, "retryable": False, "details": {}})))

    def _play(self):
        if not _HAS_ROS2 or self._node is None:
            return _fail_speaker("play", None, "PRECONDITION_FAILED",
                                 "ROS2 unavailable in driver (need rclpy + executor)")
        if self._sub is None:
            return _fail_speaker("play", None, "PRECONDITION_FAILED",
                                 "not subscribed — audio_msgs missing; source /ros_ws/install/setup.bash in the driver image")
        with self._buf_lock:
            self._buf = bytearray()
        self._playing = True
        print(f"[{CARD_SPEAKER}] play → forwarding {self._topic} to speaker (TCP binary)", flush=True)
        return {"ok": True, "card": CARD_SPEAKER, "action": "play", "state": "running",
                "topic_in": self._topic, "timestamp_ms": _now_ms()}

    def _pause(self):
        self._playing = False
        with self._buf_lock:
            self._buf = bytearray()
        self._tcp.close()
        try:
            self._http.request("/speaker/actions", {"action": "stop", "card": CARD_SPEAKER})
        except Exception:
            pass
        print(f"[{CARD_SPEAKER}] pause", flush=True)
        return {"ok": True, "card": CARD_SPEAKER, "action": "pause", "state": "idle", "timestamp_ms": _now_ms()}

    def _on_audio(self, msg) -> None:
        if not self._playing:
            return
        try:
            data = bytes(msg.data)
        except Exception:
            return
        if not data:
            return
        fmt = getattr(msg, "format", "") or ""
        if fmt:
            self._sr, self._ch = _sr_ch_from_format(fmt)
        with self._buf_lock:
            self._buf.extend(data)
            over = len(self._buf) - self._max_buffer_bytes
            if over > 0:
                del self._buf[:over]
        self._data_event.set()

    def _writer_loop(self) -> None:
        while self._alive:
            self._data_event.wait(timeout=1.0)
            self._data_event.clear()
            if not self._playing:
                continue
            while True:
                with self._buf_lock:
                    if not self._buf:
                        break
                    chunk = bytes(self._buf)
                    self._buf = bytearray()
                try:
                    self._tcp.send_pcm(self._sr, self._ch, chunk)
                    self._clear_alarm()
                except ConnectionError:
                    self._alarm("COMMUNICATION_ERROR", "speaker adapter is unreachable", True)
                    break

    def get_tool(self):
        return {"name": CARD_SPEAKER, "type": "actuator", "multiInstance": False,
          "description": "Go1 头部扬声器：播放操作员远程麦克风音频流",
          "topic_in": [{"format": "audio/pcm-16k"}],
          "inputSchema": {"type": "object",
            "properties": {
              "action": {"type": "string",
                         "enum": ["set_volume", "get_volume"],
                         "description": "要执行的扬声器操作"},
              "request_id": {"type": "string"},
              "volume_percent": {"type": "integer", "minimum": 0, "maximum": 100,
                                 "description": "音量百分比 0–100（set_volume 用）"}},
            "required": ["action"],
            "x-action-params": {
              "set_volume": {"params": ["volume_percent"], "description": "设置扬声器音量 0–100%"},
              "get_volume": {"params": [], "description": "读取当前扬声器音量"}}}}

    def start(self):
        pass

    def stop(self):
        self._alive = False
        self._playing = False
        self._data_event.set()
        self._tcp.close()

    def _call_adapter(self, action, args) -> dict:
        rid = args.get("request_id")
        payload = {k: v for k, v in args.items() if not k.startswith("_")}
        payload["action"], payload["card"] = action, CARD_SPEAKER
        try:
            result = self._http.request("/speaker/actions", payload)
        except ConnectionError:
            self._alarm("COMMUNICATION_ERROR", "speaker adapter is unreachable", True)
            return _fail_speaker(action, rid, "COMMUNICATION_ERROR", "speaker adapter is unreachable", True)
        if result.get("ok"):
            self._clear_alarm()
        else:
            self._alarm(result.get("code", "INTERNAL_ERROR"),
                        result.get("message", "speaker adapter request failed"),
                        result.get("retryable", False))
        return result

    def dispatch(self, action, args):
        rid = args.get("request_id")
        if action in ("play", "start"):
            return self._play()
        if action in ("pause", "stop"):
            return self._pause()
        if action == "set_volume":
            if type(args.get("volume_percent")) is not int or not 0 <= args["volume_percent"] <= 100:
                return _fail_speaker(action, rid, "INVALID_ARGUMENT", "volume_percent must be an integer from 0 to 100")
            return self._call_adapter(action, args)
        if action == "get_volume":
            return self._call_adapter(action, args)
        return _fail_speaker(action, rid, "INVALID_ARGUMENT", "unsupported speaker action")


def _fail_speaker(action, request_id, code, message, retryable=False, details=None):
    return {"ok": False, "card": CARD_SPEAKER, "action": action, "request_id": request_id,
            "code": code, "message": message, "details": details or {},
            "retryable": retryable, "timestamp_ms": _now_ms()}


def make_speaker(plugin_config, namespace, executor, client):
    return SpeakerPlugin(plugin_config, namespace, executor)


# ============================================================================
# face_light — 面部灯带颜色控制
# ============================================================================

CARD_FACE_LIGHT = "face_light"
_PRESETS = {"red": (255, 0, 0), "green": (0, 255, 0), "blue": (0, 0, 255),
            "yellow": (255, 255, 0), "cyan": (0, 255, 255), "magenta": (255, 0, 255),
            "white": (255, 255, 255), "off": (0, 0, 0)}
_FACE_BLACK = ((0, 0, 0),) * 12
_FACE_EFFECTS = ("blink", "breathe", "fade", "chase")
# Official LED.bmp: sides are the viewer's sides when facing the dog, NOT body sides.
_FACE_LED_MAP = [{"index": i, "viewer_side": "left" if i < 6 else "right",
                  "row_from_top": i % 6} for i in range(12)]


def _env_face(action, ok, **extra):
    d = {"ok": ok, "action": action, "card": CARD_FACE_LIGHT,
         "control_level": "HIGHLEVEL", "timestamp_ms": _now_ms(),
         "state_source": "software_record", "hardware_verified": False}
    d.update(extra)
    return d


def _face_rgb(value, label="RGB"):
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        raise ValueError(f"{label} must contain exactly three RGB integers")
    if any(type(v) is not int or not 0 <= v <= 255 for v in value):
        raise ValueError(f"{label} channels must be integers in 0..255 (no bool/coercion)")
    return tuple(value)


def _face_seconds(value, label, minimum, maximum):
    if type(value) not in (int, float) or not math.isfinite(value) or not minimum <= value <= maximum:
        raise ValueError(f"{label} must be a finite number in {minimum}..{maximum} seconds")
    return float(value)


def _face_effect_frame(effect, rgb, target, period, elapsed):
    """Pure renderer; period is one complete cycle, chase traverses indices 0..11."""
    phase = (elapsed % period) / period
    if effect == "chase":
        frame = list(_FACE_BLACK)
        frame[min(11, int(phase * 12))] = rgb
        return tuple(frame)
    if effect == "blink":
        color = rgb if phase < 0.5 else (0, 0, 0)
    elif effect == "breathe":
        scale = (1 - math.cos(2 * math.pi * phase)) / 2
        color = tuple(round(c * scale) for c in rgb)
    elif effect == "fade":
        weight = min(1.0, max(0.0, elapsed / period))  # One-way transition, then hold target
        color = tuple(round(a + (b - a) * weight) for a, b in zip(rgb, target))
    else:
        raise ValueError(f"unknown effect {effect}")
    return (color,) * 12


class _FaceSdkBackend:
    """One SDK helper process; bounded IPC, no MQTT fallback or UDP reimplementation."""
    name = "sdk"
    per_led = True

    def __init__(self, executable, exclusive, sdk_dir=None):
        self.executable = executable
        self.exclusive = exclusive
        self.sdk_dir = sdk_dir
        self.process = None
        self._pending = b""

    IO_TIMEOUT = 1.0

    @property
    def connected(self):
        # Process readiness only; the official SDK has no hardware acknowledgement.
        return self.process is not None and self.process.poll() is None

    def _read(self, timeout=1):
        import select
        deadline = time.monotonic() + timeout
        while b"\n" not in self._pending:
            remaining = deadline - time.monotonic()
            if remaining <= 0 or not select.select([self.process.stdout], [], [], max(0, remaining))[0]:
                raise RuntimeError("face light SDK adapter response timed out")
            data = os.read(self.process.stdout.fileno(), 4096)
            if not data:
                raise RuntimeError("face light SDK adapter exited or closed stdout")
            self._pending += data
            if len(self._pending) > 4096:
                raise RuntimeError("face light SDK adapter returned an oversized response")
        line, self._pending = self._pending.split(b"\n", 1)
        return line.decode("utf-8", errors="replace")

    def start(self):
        if self.exclusive is not True:
            raise RuntimeError("SDK requires sdk_exclusive=true after the existing faceLightMqtt writer is stopped; do not stop faceLightServer")
        try:
            environment = os.environ.copy()
            if self.sdk_dir:
                environment["FACE_LIGHT_SDK_DIR"] = self.sdk_dir
            self.process = subprocess.Popen([self.executable], stdin=subprocess.PIPE,
                                            stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, bufsize=0,
                                            env=environment, start_new_session=True)
            # Raw nonblocking writes cannot stall when the helper stops reading.
            os.set_blocking(self.process.stdin.fileno(), False)
            self._pending = b""
            response = self._read(timeout=15)
            if response != "READY face-light-v1":
                raise RuntimeError(response if response.startswith("ERROR ") else
                                   "face light SDK adapter protocol mismatch")
        except Exception:
            self.close()
            raise

    def _write_request(self, payload, deadline):
        import select
        pending = memoryview(payload)
        fd = self.process.stdin.fileno()
        while pending:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise RuntimeError("face light SDK adapter request write timed out")
            try:
                if not select.select([], [fd], [], remaining)[1]:
                    raise RuntimeError("face light SDK adapter request write timed out")
                sent = os.write(fd, pending)
            except (BlockingIOError, InterruptedError):
                continue  # Retry within the same deadline, including partial writes.
            if sent <= 0:
                raise RuntimeError("face light SDK adapter closed stdin")
            pending = pending[sent:]

    def write(self, frame):
        if not self.connected:
            raise RuntimeError("face light SDK adapter is not running")
        try:
            payload = (" ".join(str(v) for rgb in frame for v in rgb) + "\n").encode("ascii")
            deadline = time.monotonic() + self.IO_TIMEOUT
            self._write_request(payload, deadline)
            response = self._read(timeout=max(0, deadline - time.monotonic()))
            if response != "SENT":
                raise RuntimeError(response)
        except Exception:
            # Kill a stalled helper before another command can write/replay old work.
            self.close()
            raise

    def close(self):
        process, self.process = self.process, None
        self._pending = b""
        if process is None:
            return
        try:
            try:
                process.stdin.close()  # EOF: run SDK destructor
            except OSError:
                pass
            try:
                process.wait(timeout=0.2)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(process.pid, signal.SIGTERM)
                except ProcessLookupError:
                    pass
                try:
                    process.wait(timeout=0.2)
                except subprocess.TimeoutExpired:
                    try:
                        os.killpg(process.pid, signal.SIGKILL)
                    except ProcessLookupError:
                        pass
                    process.wait()
        finally:
            process.stdout.close()


_FACE_SDK_EXECUTABLE = "/deploy/face_light/run_sdk.sh"
_FACE_SDK_DIR = "/opt/phanthy-motus/data/go1/faceLightSDK_Nano"

_FACE_CONFIG_DEFAULTS = {
    "backend": "sdk",
    "sdk_executable": _FACE_SDK_EXECUTABLE, "sdk_exclusive": True,
    "sdk_dir": _FACE_SDK_DIR,
}


def _face_backend(config):
    backend = config["backend"]
    if backend == "mqtt":
        raise ValueError("face_light now uses the official SDK; select backend=sdk, mount the SDK and stop the old MQTT writer before confirming sdk_exclusive")
    if backend != "sdk":
        raise ValueError("face_light backend must be sdk; simulated is not a deployment option")
    if type(config["sdk_exclusive"]) is not bool:
        raise ValueError("sdk_exclusive must be a boolean")
    for key in ("sdk_dir", "sdk_executable"):
        fixed_path = _FACE_SDK_DIR if key == "sdk_dir" else _FACE_SDK_EXECUTABLE
        if config[key] != fixed_path:
            raise ValueError(f"{key} is fixed to {fixed_path}; remote path overrides are not allowed")
    if backend == "sdk":
        return _FaceSdkBackend(_FACE_SDK_EXECUTABLE, config["sdk_exclusive"], _FACE_SDK_DIR)
    raise ValueError("unsupported face_light backend")


def _face_acp_notify(action_id, status, result):
    """Report software completion; socket delivery is not hardware feedback."""
    url = os.environ.get("AGENT_CORE_URL", "https://localhost:15678").rstrip("/")
    payload = json.dumps({"action_id": action_id, "status": status, "result": result,
                          "tool": CARD_FACE_LIGHT, "ts": time.time()}).encode("utf-8")
    request = urllib.request.Request(url + "/api/acp/complete", data=payload,
                                     headers={"Content-Type": "application/json"}, method="POST")
    context = ssl.create_default_context()
    parsed_url = urllib.parse.urlsplit(url)
    if parsed_url.scheme == "https" and parsed_url.hostname in ("localhost", "127.0.0.1"):
        context.check_hostname = False
        context.verify_mode = ssl.CERT_NONE
    try:
        with urllib.request.urlopen(request, timeout=3, context=context):
            pass
    except Exception as exc:
        # Bounded callback failure must not prevent preemption or shutdown.
        print(f"[face_light] ACP callback failed for {action_id}: {exc}", flush=True)


class FaceLightPlugin:
    PREFIX = CARD_FACE_LIGHT

    def __init__(self, plugin_config, namespace, executor, client):
        c = plugin_config or {}
        self._config = {key: c.get(key, value) for key, value in _FACE_CONFIG_DEFAULTS.items()}
        self._config_error = None
        try:
            self._backend = _face_backend(self._config)
        except ValueError as exc:
            # Keep the card discoverable so persisted config cannot abort the bundle.
            # This inert fixed-path SDK backend cannot start until config is repaired.
            self._config_error = str(exc)
            self._backend = _FaceSdkBackend(_FACE_SDK_EXECUTABLE, True, _FACE_SDK_DIR)
        # Operations lock serializes lifecycle + commands; worker takes only state lock.
        self._ops = threading.RLock()
        self._lock = threading.RLock()
        self._thread = None
        self._cancel = None
        self._active = False
        self._mode = "error" if self._config_error else "stopped"
        self._colors = _FACE_BLACK
        self._last_sent = None
        self._last_error = self._config_error
        self._completion_slots = threading.BoundedSemaphore(8)
        self._completion_threads = set()

    def _report_completion(self, action_id, status, result):
        def notify():
            try:
                _face_acp_notify(action_id, status, result)
            finally:
                with self._lock:
                    self._completion_threads.discard(threading.current_thread())
                self._completion_slots.release()

        reporter = threading.Thread(target=notify, name="go1-face-light-acp", daemon=True)
        with self._lock:
            self._completion_threads.add(reporter)
        try:
            reporter.start()
        except Exception as exc:
            with self._lock:
                self._completion_threads.discard(reporter)
            self._completion_slots.release()
            print(f"[face_light] ACP reporter failed for {action_id}: {exc}", flush=True)

    def _cancel_effect(self):
        # Called under _ops. No joins while holding worker's lock.
        with self._lock:
            if self._cancel:
                self._cancel.set()
            worker = self._thread
        if worker:
            worker.join()  # backend writes are bounded; guarantees no later old write
        with self._lock:
            self._thread = self._cancel = None

    def _write(self, frame):
        # All callers hold _lock. Do not claim the frame was applied on hardware.
        self._backend.write(frame)
        self._colors = frame
        self._last_sent = _now_ms()
        self._last_error = None

    COMPLETION_SHUTDOWN_TIMEOUT = 0.5

    def start(self):
        with self._ops, self._lock:
            if self._config_error:
                return _env_face("start", False, state="idle", available=False,
                                 code="INVALID_ARGUMENT", message=self._config_error)
            if not self._active:
                try:
                    self._backend.start()
                    self._active = True
                    self._mode = "ready"
                    self._last_error = None
                except Exception as exc:
                    self._last_error = str(exc)
                    self._mode = "error"
                    return _env_face("start", False, state="idle", available=False,
                                     code="NOT_AVAILABLE", message=str(exc))
            return _env_face("start", True, state="ready", connected=self._backend.connected,
                             simulated=self._backend.name == "simulated")

    def stop(self):
        with self._ops:
            self._cancel_effect()
            with self._lock:
                error = None
                try:
                    if self._active:
                        self._write(_FACE_BLACK)
                except Exception as exc:
                    error = str(exc)
                finally:
                    self._active = False
                    self._mode = "stopped"
                    try:
                        self._backend.close()
                    except Exception as exc:
                        error = error or str(exc)
                    self._last_error = error
                if error:
                    result = _env_face("stop", False, state="idle", code="NOT_AVAILABLE", message=error)
                else:
                    result = _env_face("stop", True, state="idle")
                reporters = list(self._completion_threads)
            # Lights are already off and the backend is closed before HTTP cleanup.
            # Reporters never write LEDs or take _ops; joining cannot delay the black frame.
            deadline = time.monotonic() + self.COMPLETION_SHUTDOWN_TIMEOUT
            for reporter in reporters:
                reporter.join(timeout=max(0, deadline - time.monotonic()))
            with self._lock:
                pending = sum(reporter.is_alive() for reporter in self._completion_threads)
            result["completion_pending"] = pending
            result["completion_cleanup_complete"] = pending == 0
            return result

    def _run_effect(self, cancel, effect, rgb, target, period, duration, started, action_id):
        # First frame was sent by dispatch. Absolute elapsed time avoids timing drift.
        status = "cancelled"
        result = _env_face(effect, False, code="CANCELLED", message="Effect interrupted")
        try:
            while not cancel.wait(min(0.05, period / 24, max(0.001, duration - (time.monotonic() - started)))):
                with self._lock:
                    if cancel.is_set() or not self._active:
                        return
                    elapsed = time.monotonic() - started
                    if elapsed >= duration:
                        self._write((target,) * 12 if effect == "fade" else _FACE_BLACK)
                        self._mode = "static" if effect == "fade" else "off"
                        status = "completed"
                        result = _env_face(effect, True, mode=self._mode,
                                           end_behavior="hold_target" if effect == "fade" else "off",
                                           simulated=self._backend.name == "simulated")
                        return
                    self._write(_face_effect_frame(effect, rgb, target, duration if effect == "fade" else period, elapsed))
        except Exception as exc:
            with self._lock:
                self._last_error = str(exc)
                self._mode = "error"
            status = "error"
            result = _env_face(effect, False, code="NOT_AVAILABLE", message=str(exc))
        finally:
            # Transport POST runs separately from the worker joined by preemption.
            result["timestamp_ms"] = _now_ms()
            self._report_completion(action_id, status, result)

    def _configure(self, args):
        # Agent Core sends action=config before start and before ordinary calls.
        # Replayed identical configs must not interrupt a running effect.
        if not isinstance(args, dict):
            return _env_face("config", False, code="INVALID_ARGUMENT", message="config must be an object")
        with self._ops:
            candidate = dict(self._config)
            candidate.update({key: args[key] for key in _FACE_CONFIG_DEFAULTS if key in args})
            try:
                backend = _face_backend(candidate)
            except ValueError as exc:
                return _env_face("config", False, code="INVALID_ARGUMENT", message=str(exc))
            if candidate == self._config:
                return _env_face("config", True, state="ready" if self._active else "idle", changed=False)
            stopped = self.stop()
            with self._lock:
                self._config = candidate
                self._config_error = None
                self._backend = backend
                self._colors = _FACE_BLACK
                self._last_sent = None
                return _env_face("config", True, state="idle", changed=True, needs_start=True,
                                 previous_shutdown_ok=stopped["ok"],
                                 message="Configuration changed; start the card to apply it")

    def _info(self):
        with self._lock:
            per_led = self._backend.per_led
            effects = list(_FACE_EFFECTS)
            connected = self._backend.connected
            available = self._active and connected
            reason = None
            if not available:
                reason = self._config_error or self._last_error or (
                    "SDK adapter is not running; stop/start after checking SDK setup"
                    if self._active else "face_light is stopped; prepare the official SDK and call start")
            return _env_face("info", True, state="ready" if available else "idle",
                             available=available, unavailable_reason=reason,
                             availability_source="software_sdk_process", config_valid=self._config_error is None,
                             mode=self._mode, running=bool(self._thread and self._thread.is_alive()),
                             connected=connected, backend=self._backend.name,
                             simulated=self._backend.name == "simulated", last_error=self._last_error,
                             last_sent_timestamp_ms=self._last_sent,
                             colors=[list(c) for c in self._colors] if self._last_sent else None,
                             capabilities={"uniform_rgb": True, "per_led": per_led,
                                           "set_leds": per_led, "effects": effects,
                                           "hardware_feedback": False, "official_sdk_integrated": self._backend.name == "sdk"},
                             led_map=[dict(item) for item in _FACE_LED_MAP],
                             led_map_source="Unitree Go1_Edu.md LED.bmp; not verified on this robot")

    def get_tool(self):
        # Canvas uses description as placeholder; keep the channel hint short.
        rgb_channel = {"type": "integer", "minimum": 0, "maximum": 255, "default": 0, "description": "0"}
        rgb_array = {"type": "array", "minItems": 3, "maxItems": 3,
                     "items": {"type": "integer", "minimum": 0, "maximum": 255}}
        actions = {"set_color": ["r", "g", "b"], "preset": ["name"], "off": [],
                   "set_led": ["index", "r", "g", "b"], "set_leds": ["color_format", "colors"],
                   "blink": ["r", "g", "b", "period_s", "duration_s"],
                   "breathe": ["r", "g", "b", "period_s", "duration_s"],
                   "fade": ["r", "g", "b", "to_r", "to_g", "to_b", "duration_s"],
                   "chase": ["r", "g", "b", "period_s", "duration_s"], "info": []}
        descriptions = {"set_color": "Persistent uniform RGB until next command",
                        "preset": "Persistent named color", "off": "Cancel effect and turn all LEDs off",
                        "set_led": "Set SDK index 0..11; others retain last software frame (initially off); per-LED backend required",
                        "set_leds": "12 RGB hex colors in index order, separated by spaces (e.g. FF0000); legacy RGB arrays accepted",
                        # hex format:FF0000 00FF00 0000FF FF0000 00FF00 0000FF FF0000 00FF00 0000FF FF0000 00FF00 0000FF
                        # RGB array format:[[255,0,0],[0,255,0],[0,0,255],[255,0,0],[0,255,0],[0,0,255],[255,0,0],[0,255,0],[0,0,255],[255,0,0],[0,255,0],[0,0,255]]
                        "blink": "Blink uniform RGB on/off; period_s per full cycle, auto-off after duration_s",
                        "breathe": "Breathe by scaling RGB; period_s per full cycle, auto-off after duration_s",
                        "fade": "Transition RGB to target RGB over duration_s, then hold target until next command",
                        "chase": "One LED traverses 0..11 per period_s; auto-off after duration_s; per-LED backend required",
                        "info": "Software-recorded status and backend capabilities; no hardware feedback"}
        return {"name": CARD_FACE_LIGHT, "type": "actuator", "multiInstance": False,
                "description": "Go1 face_light: persistent RGB, 12 LED control and internal timed effects. "
                               "Official SDK only; check info.available and unavailable_reason before control. "
                               "SDK readiness is software-recorded, not hardware display feedback.",
                "inputSchema": {"type": "object", "required": ["action"],
                                "x-completion": {"actions": list(_FACE_EFFECTS), "timeout": 3610},
                                "x-resource": "face_light",
                                "properties": {"action": {"type": "string", "enum": list(actions)},
                                               **{k: dict(rgb_channel) for k in ("r", "g", "b", "to_r", "to_g", "to_b")},
                                               "name": {"type": "string", "enum": list(_PRESETS), "default": "off"},
                                               "index": {"type": "integer", "minimum": 0, "maximum": 11, "default": 0,
                                                         "description": "Facing dog: viewer left top-bottom 0..5; right 6..11"},
                                               "color_format": {"type": "string", "enum": ["hex", "rgb_array"],
                                                                "default": "hex", "description": "hex: RGB hex colors; rgb_array: JSON list of 12 RGB triples"},
                                               "colors": {"anyOf": [
                                                   {"type": "string"},
                                                   {"type": "array", "minItems": 12, "maxItems": 12, "items": rgb_array}],
                                                          "default": " ".join(["000000"] * 12),
                                                          "description": "12 RGB hex colors (0..11), separated by spaces; FF0000=red, 000000=off"},
                                               "period_s": {"type": "number", "minimum": 0.2, "maximum": 3600, "default": 2,
                                                            "description": "2"},
                                               "duration_s": {"type": "number", "minimum": 0.05, "maximum": 3600, "default": 5,
                                                              "description": "5"}},
                                "x-action-params": {a: {"params": p, "description": descriptions[a]} for a, p in actions.items()}},
                "topic_out": []}

    def dispatch(self, action, args):
        if action == "config":
            return self._configure(args)
        if action == "start":
            if isinstance(args, dict) and any(key in args for key in _FACE_CONFIG_DEFAULTS):
                configured = self._configure(args)
                if not configured["ok"]:
                    return configured
            return self.start()
        if action == "stop":
            return self.stop()
        if action == "info":
            return self._info()
        if action not in ("set_color", "preset", "off", "set_led", "set_leds", *_FACE_EFFECTS):
            return None
        try:
            if not isinstance(args, dict):
                raise ValueError("arguments must be an object")
            rgb = _face_rgb([args.get(k, 0) for k in ("r", "g", "b")])
            target = _face_rgb([args.get(k, 0) for k in ("to_r", "to_g", "to_b")], "target RGB")
            if action == "preset":
                name = args.get("name", "off")
                if not isinstance(name, str) or name.lower() not in _PRESETS:
                    raise ValueError(f"unknown preset; choose {', '.join(_PRESETS)}")
                rgb = _PRESETS[name.lower()]
            if action == "set_led":
                index = args.get("index", 0)
                if type(index) is not int or not 0 <= index < 12:
                    raise ValueError("index must be an integer in 0..11")
            if action == "set_leds":
                color_format = args.get("color_format")
                if color_format is not None and color_format not in ("hex", "rgb_array"):
                    raise ValueError("color_format must be hex or rgb_array")
                colors = args.get("colors", [[0, 0, 0] for _ in range(12)]
                                  if color_format == "rgb_array" else " ".join(["000000"] * 12))
                if color_format == "rgb_array" and isinstance(colors, str):
                    try:
                        colors = json.loads(colors)
                    except (ValueError, TypeError) as exc:
                        raise ValueError("colors must be a JSON list of 12 RGB triples in rgb_array mode") from exc
                if color_format == "hex" and not isinstance(colors, str):
                    raise ValueError("colors must be RGB hex text in hex mode")
                if isinstance(colors, str) and color_format != "rgb_array":
                    tokens = colors.replace(",", " ").split()
                    if len(tokens) != 12:
                        raise ValueError("colors must contain exactly 12 RGB hex colors in index order")
                    colors = []
                    for i, token in enumerate(tokens):
                        if not re.fullmatch(r"#?[0-9a-fA-F]{6}", token):
                            raise ValueError(f"colors[{i}] must be six RGB hex digits, e.g. FF0000 or #FF0000")
                        token = token.lstrip("#")
                        colors.append([int(token[j:j + 2], 16) for j in (0, 2, 4)])
                if not isinstance(colors, (list, tuple)) or len(colors) != 12:
                    raise ValueError("colors must contain exactly 12 RGB triples in SDK index order")
                frame = tuple(_face_rgb(c, f"colors[{i}]") for i, c in enumerate(colors))
            if action in _FACE_EFFECTS:
                period = _face_seconds(args.get("period_s", 2), "period_s", 0.2, 3600)
                duration = _face_seconds(args.get("duration_s", 5), "duration_s", 0.05, 3600)
        except ValueError as exc:
            return _env_face(action, False, code="INVALID_ARGUMENT", message=str(exc))
        with self._ops:
            if self._config_error:
                return _env_face(action, False, code="INVALID_ARGUMENT", message=self._config_error)
            reserved = False
            if action in _FACE_EFFECTS:
                reserved = self._completion_slots.acquire(blocking=False)
                if not reserved:
                    return _env_face(action, False, code="RESOURCE_BUSY",
                                     message="Completion callbacks are still pending; retry later")
            self._cancel_effect()
            with self._lock:
                if not self._active:
                    if reserved:
                        self._completion_slots.release()
                    return _env_face(action, False, code="NOT_AVAILABLE", message="face_light is stopped; start it first")
                if action == "set_led":
                    changed = list(self._colors)
                    changed[index] = rgb
                    frame = tuple(changed)
                elif action != "set_leds":
                    frame = (_FACE_BLACK if action == "off" else
                             _face_effect_frame(action, rgb, target, period, 0) if action in _FACE_EFFECTS else (rgb,) * 12)
                frame_sent = False
                try:
                    self._write(frame)
                    frame_sent = True
                    self._mode = action if action in _FACE_EFFECTS else ("off" if frame == _FACE_BLACK else "static")
                    if action in _FACE_EFFECTS:
                        action_id = f"face_light_{action}_{uuid4().hex}"
                        self._cancel = threading.Event()
                        self._thread = threading.Thread(target=self._run_effect, name="go1-face-light",
                                                        args=(self._cancel, action, rgb, target, period, duration, time.monotonic(), action_id),
                                                        daemon=True)
                        self._thread.start()
                except Exception as exc:
                    message = str(exc)
                    cleanup = {}
                    if reserved:
                        self._completion_slots.release()
                        self._thread = self._cancel = None
                        if frame_sent:
                            # No accepted action/worker owns this frame. Clear it before
                            # returning failure, under the same serialized write lock.
                            try:
                                self._write(_FACE_BLACK)
                                cleanup["initial_frame_cleanup_ok"] = True
                            except Exception as clear_exc:
                                cleanup["initial_frame_cleanup_ok"] = False
                                message += f"; initial effect frame cleanup failed: {clear_exc}"
                                self._active = False
                                try:
                                    self._backend.close()
                                except Exception as close_exc:
                                    message += f"; SDK close failed: {close_exc}"
                    self._mode = "error"
                    self._last_error = message
                    return _env_face(action, False, code="NOT_AVAILABLE", message=message, **cleanup)
                applied = {"colors": [list(c) for c in frame]}
                if action == "set_color":
                    applied.update(dict(zip(("r", "g", "b"), rgb)))
                elif action in ("off", "preset"):
                    applied["rgb"] = list(frame[0])
                    if action == "preset":
                        applied["name"] = name.lower()
                if action in _FACE_EFFECTS:
                    applied.update(duration_s=duration, end_behavior="hold_target" if action == "fade" else "off")
                    if action != "fade":
                        applied["period_s"] = period
                return _env_face(action, True, applied=applied, simulated=self._backend.name == "simulated",
                                 **({"action_id": action_id} if action in _FACE_EFFECTS else {}),
                                 delivery="simulated" if self._backend.name == "simulated" else "sdk_udp_socket_sent")


def make_face_light(plugin_config, namespace, executor, client):
    return FaceLightPlugin(plugin_config, namespace, executor, client)


# ============================================================================
# system_health — 机器人整体健康检查
# ============================================================================

CARD_SYS_HEALTH = "system_health"
_MARK = {"OK": "[OK]", "WARNING": "[WARN]", "CRITICAL": "[CRIT]", "INFO": "[i]", "UNKNOWN": "[?]"}
_RK = {"OK": 0, "INFO": 0, "UNKNOWN": 0, "WARNING": 1, "CRITICAL": 2}


def _run(cmd, timeout=3):
    out = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                         timeout=timeout, universal_newlines=True)
    return out.stdout.strip()


class SysHealthPlugin:
    def __init__(self, plugin_config, namespace, executor, client):
        self._client = client
        c = plugin_config or {}
        self._host = c.get("mqtt_host", "localhost")
        self._port = int(c.get("mqtt_port", 1883))
        self._mqtt = None
        self._bms = None
        self._lock = threading.Lock()

    def start(self):
        if self._mqtt is not None:
            return
        if not _HAS_MQTT:
            return
        try:
            self._mqtt = mqtt.Client()
            self._mqtt.on_message = self._on_msg
            self._mqtt.connect(self._host, self._port, 60)
            self._mqtt.subscribe("bms/state")
            self._mqtt.loop_start()
            print("[system_health] MQTT connected (subscribed bms/state)", flush=True)
        except Exception as e:
            print(f"[system_health] MQTT connect failed: {e}", flush=True)
            self._mqtt = None

    def stop(self):
        try:
            if self._mqtt:
                self._mqtt.loop_stop()
                self._mqtt.disconnect()
        except Exception:
            pass
        finally:
            self._mqtt = None

    def _on_msg(self, cl, userdata, msg):
        with self._lock:
            self._bms = bytes(msg.payload)

    def get_tool(self):
        return {"name": CARD_SYS_HEALTH, "type": "actuator", "multiInstance": False, "description":
                ("Get the robot's overall status / health info in one call — use this to answer "
                 "'how is the robot / robot status / is anything wrong'. Checks the compute board "
                 "(CPU temp/load, memory, disk, power throttle, network, key process) and robot subsystems "
                 "(battery, comm link, motion state); returns per-item OK/WARNING/CRITICAL + overall verdict."),
                "inputSchema": {"type": "object",
                                "properties": {"action": {"type": "string", "enum": ["robot_info"],
                                                           "description": "Get robot overall status / health info"}},
                                "required": ["action"]}}

    def dispatch(self, action, args):
        if action == "start":
            self.start()
            return {"state": "ready"}
        if action == "info":
            return {"state": "ready"}
        if action == "stop":
            self.stop()
            return {"state": "idle"}
        if action not in ("robot_info", "diagnose"):
            return None

        report, problems, counts, worst = {}, [], {}, [0]

        def add(label, status, disp):
            report[label] = f"{_MARK.get(status, '')} {disp}"
            counts[status] = counts.get(status, 0) + 1
            if _RK.get(status, 0) > worst[0]:
                worst[0] = _RK.get(status, 0)
            if status in ("WARNING", "CRITICAL"):
                problems.append(f"{_MARK[status]} {label}: {disp}")

        # compute board
        try:
            temps = []
            for z in os.listdir("/sys/class/thermal"):
                if z.startswith("thermal_zone"):
                    try:
                        with open(f"/sys/class/thermal/{z}/temp") as f:
                            temps.append(int(f.read().strip()) / 1000.0)
                    except Exception:
                        pass
            if temps:
                t = round(max(temps), 1)
                add("cpu_temp", "OK" if t < 70 else ("WARNING" if t < 80 else "CRITICAL"),
                    f"{t}C" + ("" if t < 80 else " hot"))
            else:
                add("cpu_temp", "UNKNOWN", "read failed")
        except Exception as e:
            add("cpu_temp", "UNKNOWN", str(e))

        try:
            with open("/proc/loadavg") as f:
                load1 = float(f.read().split()[0])
            n = os.cpu_count() or 1
            r = load1 / n
            add("cpu_load", "OK" if r < 0.8 else ("WARNING" if r < 1.2 else "CRITICAL"),
                f"{load1} / {n}cores" + (" overloaded" if r >= 0.8 else ""))
        except Exception as e:
            add("cpu_load", "UNKNOWN", str(e))

        try:
            mi = {}
            with open("/proc/meminfo") as f:
                for line in f:
                    k, _, v = line.partition(":")
                    if v:
                        mi[k] = int(v.strip().split()[0])
            total, avail = mi.get("MemTotal", 0), mi.get("MemAvailable", 0)
            up = round(100 * (total - avail) / total, 1) if total else 0
            add("memory", "OK" if up < 80 else ("WARNING" if up < 92 else "CRITICAL"),
                f"{up}% used (avail {avail // 1024}M / total {total // 1024}M)")
        except Exception as e:
            add("memory", "UNKNOWN", str(e))

        try:
            vfs = os.statvfs("/")
            total = vfs.f_blocks * vfs.f_frsize
            free = vfs.f_bavail * vfs.f_frsize
            up = round(100 * (total - free) / total, 1) if total else 0
            add("disk", "OK" if up < 85 else ("WARNING" if up < 95 else "CRITICAL"),
                f"{up}% used (free {free // (1024 ** 3)}G)")
        except Exception as e:
            add("disk", "UNKNOWN", str(e))

        try:
            raw = _run(["vcgencmd", "get_throttled"])
            val = raw.split("=")[-1] if "=" in raw else raw
            flags = int(val, 16) if val.startswith("0x") else 0
            if flags == 0:
                add("power", "OK", "normal")
            else:
                add("power", "CRITICAL" if (flags & 0x5) else "WARNING", f"{val} undervolt/throttle")
        except Exception:
            add("power", "UNKNOWN", "vcgencmd unavailable")

        try:
            ips = _run(["hostname", "-I"]).split()
            add("network", "OK" if ips else "CRITICAL", f"{len(ips)} IP" if ips else "no IP")
        except Exception as e:
            add("network", "UNKNOWN", str(e))

        try:
            running = bool(_run(["pgrep", "-x", "Legged_sport"]))
            add("sport_process", "OK" if running else "WARNING",
                "Legged_sport running" if running else "not running")
        except Exception as e:
            add("sport_process", "UNKNOWN", str(e))

        # robot subsystems
        try:
            snap = self._client.snapshot() if self._client else {"fresh": False}
            fresh = bool(snap.get("fresh", False))
            if not fresh:
                add("robot_comm", "WARNING", "no fresh HighState (std SDK cannot read this dog / lying / STUB)")
            else:
                add("robot_comm", "OK", f"HighState fresh (mode={snap.get('mode_name', '?')})")
            add("motion_mode", "INFO", str(snap.get("mode_name", "unknown")))
        except Exception as e:
            add("robot_comm", "UNKNOWN", str(e))

        try:
            with self._lock:
                raw = self._bms
            if not raw or len(raw) < 8:
                add("battery", "UNKNOWN", "no bms/state (MQTT) yet")
            else:
                soc = raw[3]
                cur = struct.unpack_from("<i", raw, 4)[0]
                add("battery", "OK" if soc > 30 else ("WARNING" if soc > 15 else "CRITICAL"),
                    f"{soc}% {'charging' if cur > 0 else 'discharging'} {abs(cur)}mA")
        except Exception as e:
            add("battery", "UNKNOWN", str(e))

        overall = ["OK", "WARNING", "CRITICAL"][worst[0]]
        return {
            "ok": True, "action": action, "card": CARD_SYS_HEALTH,
            "control_level": "HIGHLEVEL", "timestamp_ms": int(time.time() * 1000),
            "overall": overall,
            "summary": f"{sum(counts.values())} checks: {counts.get('OK', 0)} OK / "
                       f"{counts.get('WARNING', 0)} warn / {counts.get('CRITICAL', 0)} crit",
            "problems": problems if problems else ["none, all OK"],
            "report": report,
        }


def make_system_health(plugin_config, namespace, executor, client):
    return SysHealthPlugin(plugin_config, namespace, executor, client)
