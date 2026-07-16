"""
test_speaker.py — Go1 头部扬声器音频流播放卡（自包含，一卡一文件）。

约定见 CONTRIBUTING.md：卡名 == 模块名 == 文件名 == config.yaml 里的 key == MCP 工具名 == "test_speaker"。
main.py 会按 config 自动 import 本模块并调用 make_plugin()，无需改 main.py。
（文件名带 test_ 前缀：与将来正式的 speaker 卡区分，先作开发/验证用。）

功能：订阅 agent core 发布的 remote_mic 音频流并在 Go1 头部扬声器播放。
数据来源（实机核实）：浏览器麦克风 → WebSocket /ws/mic(agent core :15678) → agent core 逐块封成
audio_msgs/msg/AudioChunk（format="pcm_16k_16bit_mono"、data=裸 PCM 字节）发布到 ROS2 topic
`/remote_control/mic`（BEST_EFFORT）。本卡订阅该 topic，攒批后转发到 Go1 Head Nano 的 speaker_adapter 播放。

架构：
  /remote_control/mic ──ROS2订阅──▶ test_speaker(驱动容器/Pi) ──HTTP攒批──▶ speaker_adapter(Nano:18083) ──aplay──▶ 扬声器
  ┌─ Pi 驱动容器 ─────────────────────┐   HTTP/JSON     ┌─ Head Nano (192.168.123.13) ─────┐
  │ test_speaker.py: Plugin           │ ── POST ──────▶ │ speaker_adapter.py  :18083        │
  │  · 订阅 /remote_control/mic        │  /v1/speaker/   │  · 常驻流式 aplay(S16_LE 16k)     │
  │  · 攒 ~batch_ms 的 PCM 批量转发    │    actions      │  · set/get 音量（amixer）         │
  └────────────────────────────────────┘                 └───────────────────────────────────┘

语义：start=开始订阅并播放（一直播到 stop）；stop=退订并停播。容器起来后 Plugin.start() 会自动订阅一次
（best-effort），所以"启动新容器即生效"；仍可用 start/stop 手动开关。

规范（与 beep.py 一致）：
  - **无输出 topic**（本卡只订阅 topic_in，不发布任何 data/流 topic，get_tool 不含 topic_out）。
  - 报警走共享的 device_alarms topic（adapter 不可达时发一条，恢复后清除）——与 beep 相同。
  - dispatch 返回 plain dict；未知 action 返回 None。

部署前提：驱动容器须能 import audio_msgs（Dockerfile CMD 已 source /ros_ws/install/setup.bash）；
与 agent core 同 ROS_DOMAIN_ID(=42)、同 rmw。缺 rclpy/audio_msgs 时优雅降级（start 返回 PRECONDITION_FAILED）。
"""

from __future__ import annotations

import base64
import json
import threading
import time
import urllib.request

try:
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import String
    _HAS_ROS2 = True
    # 告警 topic QoS（与 beep 一致）。
    _ALARM_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                            history=HistoryPolicy.KEEP_LAST, depth=200,
                            durability=DurabilityPolicy.VOLATILE)
    # 订阅 mic 的 QoS：必须 BEST_EFFORT 才能和 agent core 的 BEST_EFFORT 发布者兼容。
    _MIC_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                          history=HistoryPolicy.KEEP_LAST, depth=50,
                          durability=DurabilityPolicy.VOLATILE)
except Exception:
    _HAS_ROS2 = False

CARD = "test_speaker"      # 卡名 = 模块名 = 文件名 = config key = MCP 工具名


def _now_ms() -> int:
    return int(time.time() * 1000)


def _failure(action, request_id, code, message, retryable=False, details=None) -> dict:
    return {"ok": False, "card": CARD, "action": action, "request_id": request_id,
            "code": code, "message": message, "details": details or {},
            "retryable": retryable, "timestamp_ms": _now_ms()}


def _sr_ch_from_format(fmt: str):
    """从 AudioChunk.format（如 "pcm_16k_16bit_mono"）解析采样率/声道，取不到给 16k/mono 缺省。"""
    f = (fmt or "").lower()
    sr = 48000 if "48k" in f else (8000 if "8k" in f else 16000)
    ch = 2 if "stereo" in f else 1
    return sr, ch


class _SpeakerAdapterClient:
    """访问 Nano 上 speaker_adapter 的最小 JSON-over-HTTP 客户端（只打固定端点）。"""

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


class Plugin:
    """test_speaker 卡：订阅 /remote_control/mic，攒批把 PCM 转发给 Nano speaker_adapter 播放。"""

    def __init__(self, plugin_config, namespace, executor):
        self._config = plugin_config or {}
        self._ns = namespace
        self._executor = executor
        self._client = _SpeakerAdapterClient(self._config)
        self._topic = self._config.get("mic_topic", "/remote_control/mic")
        self._batch_ms = float(self._config.get("forward_batch_ms", 80))
        # 转发跟不上时的缓冲上限（默认 ~2s@16k/16bit/mono），超出丢最旧的以保持准实时。
        self._max_buffer_bytes = int(self._config.get("max_buffer_bytes", 64000))

        self._state = "idle"
        self._lock = threading.Lock()      # 保护订阅/线程生命周期
        self._buf = bytearray()
        self._buf_lock = threading.Lock()  # 保护 PCM 缓冲
        self._sr, self._ch = 16000, 1
        self._sub = None
        self._forward_thread = None
        self._running = False

        # ROS2 节点（供订阅 + 告警）：无 rclpy / executor 时全程降级。
        self._node = None
        self._alarm_pub = None
        self._alarm_state = None
        if _HAS_ROS2 and executor is not None:
            try:
                self._node = Node("go1_%s" % CARD)
                self._alarm_pub = self._node.create_publisher(
                    String, "/%s/state/device_alarms" % namespace, _ALARM_QOS)
                executor.add_node(self._node)
            except Exception as e:  # noqa: BLE001
                print(f"[{CARD}] ROS2 不可用: {e}", flush=True)
                self._node = None
                self._alarm_pub = None

    # ── 告警（best-effort；无 publisher 时静默）─────────────────────────────
    def _alarm(self, code, message, retryable):
        if self._alarm_pub is None or self._alarm_state == code:
            return
        self._alarm_state = code
        now = _now_ms()
        self._alarm_pub.publish(String(data=json.dumps({
            "alarm_id": "%s-%s-001" % (CARD, code), "active": True, "severity": "error",
            "card": CARD, "code": code, "message": message, "first_seen_ms": now,
            "last_seen_ms": now, "recovered_at_ms": None, "retryable": retryable, "details": {}})))

    def _clear_alarm(self):
        if self._alarm_pub is None or not self._alarm_state:
            return
        code, self._alarm_state, now = self._alarm_state, None, _now_ms()
        self._alarm_pub.publish(String(data=json.dumps({
            "alarm_id": "%s-%s-001" % (CARD, code), "active": False, "severity": "error",
            "card": CARD, "code": code, "message": "condition recovered", "first_seen_ms": now,
            "last_seen_ms": now, "recovered_at_ms": now, "retryable": False, "details": {}})))

    # ── 订阅生命周期 ─────────────────────────────────────────────────────────
    def _start_sub(self) -> dict:
        with self._lock:
            if self._state == "running":
                return {"ok": True, "card": CARD, "action": "start", "state": "running",
                        "topic_in": self._topic, "timestamp_ms": _now_ms()}
            if not _HAS_ROS2 or self._node is None:
                return _failure("start", None, "PRECONDITION_FAILED",
                                "ROS2 unavailable in driver (need rclpy + executor)")
            try:
                from audio_msgs.msg import AudioChunk
            except Exception:
                return _failure("start", None, "PRECONDITION_FAILED",
                                "audio_msgs not importable — source /ros_ws/install/setup.bash in the driver image")
            try:
                self._sub = self._node.create_subscription(
                    AudioChunk, self._topic, self._on_audio, _MIC_QOS)
            except Exception as e:  # noqa: BLE001
                return _failure("start", None, "INTERNAL_ERROR", "failed to subscribe %s: %s" % (self._topic, e))

            with self._buf_lock:
                self._buf = bytearray()
            self._running = True
            self._forward_thread = threading.Thread(target=self._forward_loop, name="go1_speaker_fwd", daemon=True)
            self._forward_thread.start()
            self._state = "running"
            print(f"[{CARD}] subscribed {self._topic} → speaker_adapter", flush=True)
            return {"ok": True, "card": CARD, "action": "start", "state": "running",
                    "topic_in": self._topic, "timestamp_ms": _now_ms()}

    def _stop_sub(self) -> dict:
        with self._lock:
            self._running = False
            th = self._forward_thread
            self._forward_thread = None
        if th is not None:
            th.join(timeout=2.0)
        with self._lock:
            if self._sub is not None and self._node is not None:
                try:
                    self._node.destroy_subscription(self._sub)
                except Exception:
                    pass
            self._sub = None
            self._state = "idle"
        with self._buf_lock:
            self._buf = bytearray()
        # 通知 Nano 停播（失败不阻塞）。
        try:
            self._client.request("/speaker/actions", {"action": "stop", "card": CARD})
        except Exception:
            pass
        print(f"[{CARD}] unsubscribed {self._topic}", flush=True)
        return {"ok": True, "card": CARD, "action": "stop", "state": "idle", "timestamp_ms": _now_ms()}

    def _on_audio(self, msg) -> None:
        """ROS2 订阅回调（executor 线程）：把 AudioChunk 的 PCM 追加进缓冲，超上限丢最旧（保准实时）。"""
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

    def _forward_loop(self) -> None:
        """攒 batch_ms 的 PCM 一次性 base64 转发给 speaker_adapter，解耦 ROS 回调与 HTTP 延迟。"""
        while self._running:
            time.sleep(self._batch_ms / 1000.0)
            with self._buf_lock:
                if not self._buf:
                    continue
                chunk = bytes(self._buf)
                self._buf = bytearray()
            payload = {"action": "play", "card": CARD,
                       "pcm_base64": base64.b64encode(chunk).decode(),
                       "sample_rate": self._sr, "channels": self._ch}
            try:
                r = self._client.request("/speaker/actions", payload)
                if r.get("ok"):
                    self._clear_alarm()
                else:
                    self._alarm(r.get("code", "INTERNAL_ERROR"),
                                r.get("message", "speaker adapter play failed"),
                                r.get("retryable", False))
            except ConnectionError:
                self._alarm("COMMUNICATION_ERROR", "speaker adapter is unreachable", True)

    # ── 插件契约 ───────────────────────────────────────────────────────────
    def get_tool(self):
        # 动作名避开平台保留的系统生命周期动作 {start,stop,info,config}（那些不会渲染成前端按钮）：
        # 用 play/pause 作为用户可点的“开播/停播”。仍保留 start/stop 在 enum 里以兼容平台生命周期调用，
        # 但 x-action-params（决定前端按钮）只暴露 play/pause/set_volume/get_volume。
        return {"name": CARD, "type": "actuator", "multiInstance": False,
          "description": ("Go1 head speaker — plays the operator's remote microphone stream "
                          "(subscribes ROS2 /remote_control/mic, PCM-16k) on the on-board speaker "
                          "(action card, no output topics). play begins playing the mic stream until pause."),
          "inputSchema": {"type": "object",
            "properties": {
              "action": {"type": "string",
                         "enum": ["play", "pause", "set_volume", "get_volume", "start", "stop"],
                         "description": "Speaker action to perform"},
              "request_id": {"type": "string"},
              "volume_percent": {"type": "integer", "minimum": 0, "maximum": 100,
                                 "description": "Volume 0–100% (set_volume)"}},
            "required": ["action"],
            "x-action-params": {
              "play":       {"params": [], "description": "Start playing the remote mic stream on the speaker"},
              "pause":      {"params": [], "description": "Stop playing the mic stream (unsubscribe)"},
              "set_volume": {"params": ["volume_percent"], "description": "Set speaker volume 0–100%"},
              "get_volume": {"params": [], "description": "Read current speaker volume"}}}}

    def start(self):
        # 平台生命周期钩子：主进程装配时调用。这里不自动订阅——平台在注册后会把卡置为 idle（会调 stop），
        # 由用户在 15678 点 play 开播。避免与平台生命周期打架。
        pass

    def stop(self):
        try:
            self._stop_sub()
        except Exception:
            pass

    def _call_adapter(self, action, args) -> dict:
        rid = args.get("request_id")
        payload = {k: v for k, v in args.items() if not k.startswith("_")}
        payload["action"], payload["card"] = action, CARD
        try:
            result = self._client.request("/speaker/actions", payload)
        except ConnectionError:
            self._alarm("COMMUNICATION_ERROR", "speaker adapter is unreachable", True)
            return _failure(action, rid, "COMMUNICATION_ERROR", "speaker adapter is unreachable", True)
        if result.get("ok"):
            self._clear_alarm()
        else:
            self._alarm(result.get("code", "INTERNAL_ERROR"),
                        result.get("message", "speaker adapter request failed"),
                        result.get("retryable", False))
        return result

    def dispatch(self, action, args):
        rid = args.get("request_id")
        # play(用户按钮) 与 start(平台生命周期) 都开播；pause 与 stop 都停播。
        if action in ("play", "start"):
            return self._start_sub()
        if action in ("pause", "stop"):
            return self._stop_sub()
        if action == "set_volume":
            if type(args.get("volume_percent")) is not int or not 0 <= args["volume_percent"] <= 100:
                return _failure(action, rid, "INVALID_ARGUMENT", "volume_percent must be an integer from 0 to 100")
            return self._call_adapter(action, args)
        if action == "get_volume":
            return self._call_adapter(action, args)
        return _failure(action, rid, "INVALID_ARGUMENT", "unsupported test_speaker action")


def make_plugin(plugin_config, namespace, executor, client):
    """main.py 装配入口。test_speaker 不用共享 SDK client（HighState），故忽略 client。"""
    return Plugin(plugin_config, namespace, executor)
