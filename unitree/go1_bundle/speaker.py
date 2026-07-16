"""
speaker.py — Go1 头部扬声器音频流播放卡（自包含，一卡一文件）。

约定见 CONTRIBUTING.md：卡名 == 模块名 == 文件名 == config.yaml 里的 key == MCP 工具名 == "speaker"。
main.py 会按 config 自动 import 本模块并调用 make_plugin()，无需改 main.py。

功能：订阅 agent core 发布的 remote_mic 音频流并在 Go1 头部扬声器播放。
数据来源（实机核实）：浏览器麦克风 → WebSocket /ws/mic(agent core :15678) → agent core 逐块封成
audio_msgs/msg/AudioChunk（format="pcm_16k_16bit_mono"、data=裸 PCM 字节）发布到 ROS2 topic
`/remote_control/mic`（topic_out format = "audio/pcm-16k"）。本卡以 topic_in(audio/pcm-16k) 声明输入口，
在 15678 画布上与 remote_mic 连线；play 时开始把收到的 PCM 攒批转发到 Head Nano 的 speaker_adapter 播放。

架构：
  /remote_control/mic ──ROS2订阅──▶ speaker(驱动容器/Pi) ──HTTP攒批──▶ speaker_adapter(Nano:18083) ──aplay──▶ 扬声器

生命周期/线程安全（重要）：订阅在 __init__ **只建一次、运行期永不 destroy**——因为本卡的 node 挂在
主进程的 MultiThreadedExecutor 上，运行期 destroy_subscription 会与 executor 的 spin 撞车报
`InvalidHandle: cannot use Destroyable`（会连累整个驱动的 spin 线程崩掉）。故 play/pause 只翻
`_playing` 标志：not playing 时回调直接丢帧、转发线程空转，绝不动 ROS 句柄。

规范（与 beep.py 一致）：
  - **无输出 topic**（只有 topic_in，不发布任何 data/流 topic）。
  - 报警走共享的 device_alarms topic（adapter 不可达时发一条，恢复后清除）。
  - 用户按钮用非保留名 play/pause（平台把 start/stop/info/config 当系统动作、不渲染成按钮）。

部署前提：驱动容器须能 import audio_msgs（Dockerfile CMD 已 source /ros_ws/install/setup.bash）；
与 agent core 同 ROS_DOMAIN_ID(=42)、同 rmw。缺 rclpy/audio_msgs 时优雅降级（play 返回 PRECONDITION_FAILED）。
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
    _ALARM_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                            history=HistoryPolicy.KEEP_LAST, depth=200,
                            durability=DurabilityPolicy.VOLATILE)
    # 订阅 mic 的 QoS：必须 BEST_EFFORT 才能和 agent core 的 BEST_EFFORT 发布者兼容。
    _MIC_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                          history=HistoryPolicy.KEEP_LAST, depth=50,
                          durability=DurabilityPolicy.VOLATILE)
except Exception:
    _HAS_ROS2 = False

CARD = "speaker"      # 卡名 = 模块名 = 文件名 = config key = MCP 工具名


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
    """speaker 卡：订阅 /remote_control/mic，play 时把 PCM 攒批转发给 Nano speaker_adapter 播放。"""

    def __init__(self, plugin_config, namespace, executor):
        self._config = plugin_config or {}
        self._ns = namespace
        self._executor = executor
        self._client = _SpeakerAdapterClient(self._config)
        self._topic = self._config.get("mic_topic", "/remote_control/mic")
        self._batch_ms = float(self._config.get("forward_batch_ms", 80))
        # 转发跟不上时的缓冲上限（默认 ~2s@16k/16bit/mono），超出丢最旧的以保持准实时。
        self._max_buffer_bytes = int(self._config.get("max_buffer_bytes", 64000))

        self._playing = False              # play/pause 标志（不碰 ROS 句柄）
        self._alive = True                 # 进程存活标志（Plugin.stop 置 False）
        self._buf = bytearray()
        self._buf_lock = threading.Lock()
        self._sr, self._ch = 16000, 1

        # ROS2 节点：订阅 mic + 发告警。**订阅只建一次，运行期永不 destroy**（见模块 docstring）。
        self._node = None
        self._sub = None
        self._alarm_pub = None
        self._alarm_state = None
        if _HAS_ROS2 and executor is not None:
            try:
                self._node = Node("go1_%s" % CARD)
                self._alarm_pub = self._node.create_publisher(
                    String, "/%s/state/device_alarms" % namespace, _ALARM_QOS)
                try:
                    from audio_msgs.msg import AudioChunk
                    self._sub = self._node.create_subscription(
                        AudioChunk, self._topic, self._on_audio, _MIC_QOS)
                    print(f"[{CARD}] subscribed {self._topic} (idle until play)", flush=True)
                except Exception as e:  # noqa: BLE001
                    print(f"[{CARD}] audio_msgs 不可用，无法订阅 {self._topic}: {e}"
                          f"（需 source /ros_ws/install/setup.bash）", flush=True)
                    self._sub = None
                executor.add_node(self._node)
            except Exception as e:  # noqa: BLE001
                print(f"[{CARD}] ROS2 不可用: {e}", flush=True)
                self._node = None
                self._alarm_pub = None
                self._sub = None

        # 转发线程常驻（进程生命周期）：not playing / 无数据时空转，绝不动 ROS 句柄。
        self._fwd_thread = threading.Thread(target=self._forward_loop, name="go1_speaker_fwd", daemon=True)
        self._fwd_thread.start()

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

    # ── 播放开关（只翻标志，不动订阅）─────────────────────────────────────────
    def _play(self) -> dict:
        if not _HAS_ROS2 or self._node is None:
            return _failure("play", None, "PRECONDITION_FAILED",
                            "ROS2 unavailable in driver (need rclpy + executor)")
        if self._sub is None:
            return _failure("play", None, "PRECONDITION_FAILED",
                            "not subscribed — audio_msgs missing; source /ros_ws/install/setup.bash in the driver image")
        with self._buf_lock:
            self._buf = bytearray()
        self._playing = True
        print(f"[{CARD}] play → forwarding {self._topic} to speaker", flush=True)
        return {"ok": True, "card": CARD, "action": "play", "state": "running",
                "topic_in": self._topic, "timestamp_ms": _now_ms()}

    def _pause(self) -> dict:
        self._playing = False
        with self._buf_lock:
            self._buf = bytearray()
        try:
            self._client.request("/speaker/actions", {"action": "stop", "card": CARD})
        except Exception:
            pass
        print(f"[{CARD}] pause", flush=True)
        return {"ok": True, "card": CARD, "action": "pause", "state": "idle", "timestamp_ms": _now_ms()}

    def _on_audio(self, msg) -> None:
        """ROS2 订阅回调（executor 线程）：仅在 playing 时缓冲 PCM，超上限丢最旧（保准实时）。"""
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

    def _forward_loop(self) -> None:
        """常驻转发线程：playing 且有数据时，攒 batch_ms 的 PCM 一次性 base64 发给 speaker_adapter。"""
        while self._alive:
            time.sleep(self._batch_ms / 1000.0)
            if not self._playing:
                continue
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
        # topic_in(audio/pcm-16k)：与 remote_mic 的 topic_out 同格式，供 15678 画布连线。无 topic_out。
        # 播放不做成按钮：开启智能控制(project start)时平台会对本卡调 action=start(带连线解析出的 input_topic)
        #   → dispatch 映射到开播；停止智能控制调 stop → 停播。用户卡片按钮只保留音量(set/get_volume)。
        return {"name": CARD, "type": "actuator", "multiInstance": False,
          "description": ("Go1 head speaker — plays the operator's remote microphone stream "
                          "(audio/pcm-16k from remote_mic) on the on-board speaker. Wire remote_mic → "
                          "this card and start the project; it plays until the project stops. No output topics."),
          "topic_in": [{"format": "audio/pcm-16k"}],
          "inputSchema": {"type": "object",
            "properties": {
              "action": {"type": "string",
                         "enum": ["set_volume", "get_volume"],
                         "description": "Speaker action to perform"},
              "request_id": {"type": "string"},
              "volume_percent": {"type": "integer", "minimum": 0, "maximum": 100,
                                 "description": "Volume 0–100% (set_volume)"}},
            "required": ["action"],
            "x-action-params": {
              "set_volume": {"params": ["volume_percent"], "description": "Set speaker volume 0–100%"},
              "get_volume": {"params": [], "description": "Read current speaker volume"}}}}

    def start(self):
        pass   # 由用户在 15678 点 play 开播；不自动订阅播放（订阅已在 __init__ 建好，此处不动）。

    def stop(self):
        # 主进程关闭钩子：停播 + 停转发线程。**不 destroy 订阅**（交给 rclpy.shutdown 统一回收，避免 spin 撞车）。
        self._alive = False
        self._playing = False

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
            return self._play()
        if action in ("pause", "stop"):
            return self._pause()
        if action == "set_volume":
            if type(args.get("volume_percent")) is not int or not 0 <= args["volume_percent"] <= 100:
                return _failure(action, rid, "INVALID_ARGUMENT", "volume_percent must be an integer from 0 to 100")
            return self._call_adapter(action, args)
        if action == "get_volume":
            return self._call_adapter(action, args)
        return _failure(action, rid, "INVALID_ARGUMENT", "unsupported speaker action")


def make_plugin(plugin_config, namespace, executor, client):
    """main.py 装配入口。speaker 不用共享 SDK client（HighState），故忽略 client。"""
    return Plugin(plugin_config, namespace, executor)
