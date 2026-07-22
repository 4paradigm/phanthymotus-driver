"""
ext_devices.py — Go1 外部设备通信卡合集(actuator)。

合并了以下外部设备卡（每张卡行为不变）：
  - beep: 头部扬声器蜂鸣控制（HTTP 到 Nano beep_adapter :18082）
  - speaker: 头部扬声器音频流播放（ROS2 订阅 + TCP 二进制帧到 Nano speaker_adapter）
  - face_light: 面部灯带颜色控制（MQTT face_light/color）
  - system_health: 机器人整体健康检查（CPU/内存/磁盘/电池/MQTT）


这些卡不共享 SDK client 控制通路，各自通过独立协议(Nano HTTP / ROS2+TCP / MQTT / UDP)通信。
"""

from __future__ import annotations

import json
import os
import socket
import struct
import subprocess
import threading
import time
import urllib.request

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


def _env_face(action, ok, **extra):
    d = {"ok": ok, "action": action, "card": CARD_FACE_LIGHT,
         "control_level": "HIGHLEVEL", "timestamp_ms": int(time.time() * 1000)}
    d.update(extra)
    return d


class FaceLightPlugin:
    def __init__(self, plugin_config, namespace, executor, client):
        c = plugin_config or {}
        self._host = c.get("mqtt_host", "localhost")
        self._port = int(c.get("mqtt_port", 1883))
        self._client = None

    def start(self):
        if not _HAS_MQTT:
            print("[face_light] paho-mqtt 未安装,灯带不可用(模拟)", flush=True)
            return
        try:
            self._client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION1)
            self._client.connect(self._host, self._port, 60)
            self._client.loop_start()
            print(f"[{CARD_FACE_LIGHT}] MQTT 已连接 → {self._host}:{self._port}", flush=True)
        except Exception as e:
            print(f"[{CARD_FACE_LIGHT}] MQTT 连接失败: {e}", flush=True)
            self._client = None

    def stop(self):
        try:
            if self._client:
                self._pub(0, 0, 0)
                time.sleep(0.1)
                self._client.loop_stop()
                self._client.disconnect()
        except Exception:
            pass

    def _pub(self, r, g, b):
        if self._client is None:
            return False
        self._client.publish("face_light/color", bytes([r & 0xFF, g & 0xFF, b & 0xFF]))
        return True

    def get_tool(self):
        return {
            "name": CARD_FACE_LIGHT, "type": "actuator", "multiInstance": False,
            "description": ("Go1 face LED strip STATIC color via MQTT — set a persistent RGB/preset color and hold it "
                            "(steady-state indication; stays until changed or off). For blink/breathe/fade/timed effects, "
                            "additional effects cards can be added in the future."),
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {"type": "string", "enum": ["set_color", "preset", "off"],
                               "description": "Light action"},
                    "r": {"type": "integer", "description": "Red 0-255"},
                    "g": {"type": "integer", "description": "Green 0-255"},
                    "b": {"type": "integer", "description": "Blue 0-255"},
                    "name": {"type": "string",
                             "description": "Preset: red/green/blue/yellow/cyan/magenta/white/off"},
                },
                "required": ["action"],
                "x-action-params": {
                    "set_color": {"params": ["r", "g", "b"], "description": "Set RGB (0-255 each)"},
                    "preset": {"params": ["name"], "description": "Preset color by name"},
                    "off": {"params": [], "description": "Turn light off"},
                },
            },
            "topic_out": [],
        }

    def dispatch(self, action, args):
        if action == "start":
            return {"state": "ready"}
        if action == "stop":
            return {"state": "idle"}
        if action == "info":
            return {"state": "ready"}

        if self._client is None:
            return _env_face(action, False, code="NOT_AVAILABLE",
                             message="MQTT not connected (paho missing or broker unreachable)")

        if action == "off":
            self._pub(0, 0, 0)
            return _env_face("off", True, applied={"rgb": [0, 0, 0]})
        if action == "preset":
            nm = str(args.get("name", "off")).lower()
            rgb = _PRESETS.get(nm)
            if rgb is None:
                return _env_face("preset", False, code="INVALID_ARG",
                                 message=f"unknown preset {nm}; available: {', '.join(_PRESETS)}")
            self._pub(*rgb)
            return _env_face("preset", True, applied={"name": nm, "rgb": list(rgb)})
        if action == "set_color":
            r = max(0, min(255, int(args.get("r", 0))))
            g = max(0, min(255, int(args.get("g", 0))))
            b = max(0, min(255, int(args.get("b", 0))))
            self._pub(r, g, b)
            return _env_face("set_color", True, applied={"r": r, "g": g, "b": b})
        return None


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
        if action in ("start", "info"):
            return {"state": "ready"}
        if action == "stop":
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
