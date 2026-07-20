"""
speaker.py — Go1 头部扬声器音频流播放卡（自包含，一卡一文件）。

约定见 CONTRIBUTING.md：卡名 == 模块名 == 文件名 == config.yaml 里的 key == MCP 工具名 == "speaker"。
main.py 会按 config 自动 import 本模块并调用 make_plugin()，无需改 main.py。

功能：订阅 agent core 发布的 remote_mic 音频流并在 Go1 头部扬声器播放。
数据来源（实机核实）：浏览器麦克风 → WebSocket /ws/mic(agent core :15678) → agent core 逐块封成
audio_msgs/msg/AudioChunk（format="pcm_16k_16bit_mono"、data=裸 PCM 字节）发布到 ROS2 topic
`/remote_control/mic`（topic_out format = "audio/pcm-16k"）。本卡以 topic_in(audio/pcm-16k) 声明输入口，
在 15678 画布上与 remote_mic 连线；play 时开始把收到的 PCM 转发到 Head Nano 的 speaker_adapter 播放。

架构（v2 低延迟）：
  /remote_control/mic ─ROS2─▶ speaker(Pi) ─TCP二进制帧─▶ speaker_adapter(Nano:18084) ─aplay─▶ 扬声器
  控制命令(volume/stop) ─HTTP JSON─▶ speaker_adapter(Nano:18083)

v2 改进（相比 v1 HTTP+base64 攒批）：
  1. PCM 走持久 TCP 连接 + 长度前缀二进制帧，去掉 base64（省 33% 带宽）和 JSON/HTTP 开销
  2. ROS2 回调通过 Event 即刻唤醒 writer 线程，去掉 40ms sleep 攒批
  3. 参数调优：缓冲上限收紧(3200B≈100ms)、ALSA buffer 75ms/period 10ms

生命周期/线程安全（重要）：订阅在 __init__ **只建一次、运行期永不 destroy**——因为本卡的 node 挂在
主进程的 MultiThreadedExecutor 上，运行期 destroy_subscription 会与 executor 的 spin 撞车报
`InvalidHandle: cannot use Destroyable`（会连累整个驱动的 spin 线程崩掉）。故 play/pause 只翻
`_playing` 标志：not playing 时回调直接丢帧、writer 线程空转，绝不动 ROS 句柄。

规范（与 beep.py 一致）：
  - **无输出 topic**（只有 topic_in，不发布任何 data/流 topic）。
  - 报警走共享的 device_alarms topic（adapter 不可达时发一条，恢复后清除）。
  - 用户按钮用非保留名 play/pause（平台把 start/stop/info/config 当系统动作、不渲染成按钮）。

部署前提：驱动容器须能 import audio_msgs（Dockerfile CMD 已 source /ros_ws/install/setup.bash）；
与 agent core 同 ROS_DOMAIN_ID(=42)、同 rmw。缺 rclpy/audio_msgs 时优雅降级（play 返回 PRECONDITION_FAILED）。
"""

from __future__ import annotations

import json
import socket
import struct
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

# TCP 二进制帧类型
_FRAME_PCM = 0x01


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


# ── 持久 TCP 连接（二进制 PCM 帧）────────────────────────────────────────────

class _TcpLink:
    """到 Nano speaker_adapter TCP 流端口的持久连接。懒建连 + 断线自动重连。
    帧格式：[4B frame_size(含帧头)][2B sample_rate][1B channels][1B type][...PCM...]"""

    def __init__(self, host: str, port: int, connect_timeout: float = 2.0):
        self._host = host
        self._port = port
        self._timeout = connect_timeout
        self._sock: socket.socket | None = None
        self._lock = threading.Lock()   # 保护 _sock 的读写

    def _ensure(self) -> socket.socket:
        if self._sock is not None:
            return self._sock
        s = socket.create_connection((self._host, self._port), timeout=self._timeout)
        s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        s.settimeout(5.0)   # 发送超时
        self._sock = s
        return s

    def send_pcm(self, sr: int, ch: int, pcm: bytes) -> None:
        """发送一帧 PCM。连接失败/发送失败抛 ConnectionError，调用方处理。"""
        frame_size = 8 + len(pcm)
        header = struct.pack(">IHBx", frame_size, sr, ch)  # x = padding/type=0x01 位置
        # 把 type 写进 padding 位
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


# ── HTTP 客户端（控制命令：音量/stop/info）────────────────────────────────────

class _HttpCtrlClient:
    """访问 Nano 上 speaker_adapter 的 HTTP 端点（仅低频控制命令）。"""

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


# ── 插件主体 ─────────────────────────────────────────────────────────────────

class Plugin:
    """speaker 卡：订阅 /remote_control/mic，play 时把 PCM 通过持久 TCP 即刻推给 Nano speaker_adapter 播放。"""

    def __init__(self, plugin_config, namespace, executor):
        self._config = plugin_config or {}
        self._ns = namespace
        self._executor = executor
        self._topic = self._config.get("mic_topic", "/remote_control/mic")

        # 低延迟：Event 驱动推式转发 + 小缓冲上限(~100ms@16k)。积压超上限丢最旧 → 延迟钉在低位。
        self._max_buffer_bytes = int(self._config.get("max_buffer_bytes", 3200))

        self._playing = False              # play/pause 标志（不碰 ROS 句柄）
        self._alive = True                 # 进程存活标志（Plugin.stop 置 False）
        self._buf = bytearray()
        self._buf_lock = threading.Lock()
        self._data_event = threading.Event()   # ROS2 回调放完数据 set → writer 线程即刻醒
        self._sr, self._ch = 16000, 1

        # TCP 链路（PCM 数据流）+ HTTP 客户端（控制命令）
        _host = self._config.get("adapter_host", "192.168.123.13")
        _stream_port = int(self._config.get("stream_port", 18084))
        self._tcp = _TcpLink(_host, _stream_port)
        self._http = _HttpCtrlClient(self._config)

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

        # Writer 线程常驻（进程生命周期）：由 Event 驱动，not playing / 无数据时空转，绝不动 ROS 句柄。
        self._writer_thread = threading.Thread(target=self._writer_loop, name="go1_speaker_writer", daemon=True)
        self._writer_thread.start()

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
        print(f"[{CARD}] play → forwarding {self._topic} to speaker (TCP binary)", flush=True)
        return {"ok": True, "card": CARD, "action": "play", "state": "running",
                "topic_in": self._topic, "timestamp_ms": _now_ms()}

    def _pause(self) -> dict:
        self._playing = False
        with self._buf_lock:
            self._buf = bytearray()
        self._tcp.close()   # 断开 TCP 流连接（adapter 侧检测到断开会回到 accept）
        try:
            self._http.request("/speaker/actions", {"action": "stop", "card": CARD})
        except Exception:
            pass
        print(f"[{CARD}] pause", flush=True)
        return {"ok": True, "card": CARD, "action": "pause", "state": "idle", "timestamp_ms": _now_ms()}

    # ── ROS2 音频回调 + Event 驱动 writer 线程 ────────────────────────────────

    def _on_audio(self, msg) -> None:
        """ROS2 订阅回调（executor 线程）：仅在 playing 时缓冲 PCM，超上限丢最旧，set Event 唤醒 writer。"""
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
        self._data_event.set()   # 即刻唤醒 writer 线程

    def _writer_loop(self) -> None:
        """Event 驱动的 writer 线程：ROS2 回调 set Event → 立即醒来 drain buffer → TCP 二进制帧发送。
        相比 v1 的 time.sleep(40ms) 攒批，延迟从 ~30ms 均值降到 <1ms。"""
        while self._alive:
            # 等 Event（timeout 仅为检测 _alive 退出，正常路径几乎不触发超时）
            self._data_event.wait(timeout=1.0)
            self._data_event.clear()
            if not self._playing:
                continue
            # 内层循环：一次性发完所有积压（如果 _on_audio 在发送期间又来了数据，会在下一轮 drain）
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
                    break   # 断线后不紧循环重试，等下一次 Event 再连

    # ── 插件契约 ───────────────────────────────────────────────────────────
    def get_tool(self):
        # topic_in(audio/pcm-16k)：与 remote_mic 的 topic_out 同格式，供 15678 画布连线。无 topic_out。
        # 播放不做成按钮：开启智能控制(project start)时平台会对本卡调 action=start(带连线解析出的 input_topic)
        #   → dispatch 映射到开播；停止智能控制调 stop → 停播。用户卡片按钮只保留音量(set/get_volume)。
        return {"name": CARD, "type": "actuator", "multiInstance": False,
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
        pass   # 由用户在 15678 点 play 开播；不自动订阅播放（订阅已在 __init__ 建好，此处不动）。

    def stop(self):
        # 主进程关闭钩子：停播 + 停 writer 线程 + 关 TCP。**不 destroy 订阅**（交给 rclpy.shutdown 统一回收）。
        self._alive = False
        self._playing = False
        self._data_event.set()   # 唤醒 writer 线程让它检测 _alive=False 退出
        self._tcp.close()

    def _call_adapter(self, action, args) -> dict:
        rid = args.get("request_id")
        payload = {k: v for k, v in args.items() if not k.startswith("_")}
        payload["action"], payload["card"] = action, CARD
        try:
            result = self._http.request("/speaker/actions", payload)
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
