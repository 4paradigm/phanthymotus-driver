#!/usr/bin/env python3
"""
engineai/t800/plugins/peripherals.py — T800 开发版外设卡片（LED / 麦克风 / 扬声器）。

数据流:
  LedPlugin      (actuator): dispatch → /hardware/led_control (domain 69, LedControl)
  MicPlugin      (sensor):   ALSA/sounddevice 采集 → /{ns}/mic/audio (domain 42,
                             audio_msgs/AudioChunk, format "audio/pcm-16k")
  SpeakerPlugin  (actuator): 本地 sounddevice 播放 WAV 文件 / base64 PCM，无 ROS 话题

音频协议硬约束（perception 层 ASR）: 16kHz 单声道 PCM_S16_LE，
chunk 必须 ≥1024 字节（512 samples）—— 攒够 512 samples 才发布一个
AudioChunk，低于该值会被 ASR 的 VAD 静默丢弃。

模块级只 import 标准库；rclpy / audio_msgs / sounddevice / numpy 均在函数内
延迟导入并 try/except 容错 —— 本机无 ROS2 环境且无声卡时模块可被纯 import 测试。
"""

from __future__ import annotations

import base64
import struct
import threading
import time


def _resample_linear(samples, src_rate, dst_rate):
    """线性插值重采样（纯 Python，不依赖 numpy）。48k→16k 时为 3 倍抽取。"""
    n_in = len(samples)
    if n_in < 2:
        return samples
    n_out = max(1, int(n_in * dst_rate / src_rate))
    step = (n_in - 1) / float(n_out - 1) if n_out > 1 else 0.0
    out = []
    for i in range(n_out):
        pos = i * step
        i0 = int(pos)
        frac = pos - i0
        i1 = i0 + 1 if i0 + 1 < n_in else i0
        out.append(int(round(samples[i0] * (1.0 - frac) + samples[i1] * frac)))
    return out


# ══════════════════════════════════════════════════════════════════════════════
# LedPlugin (actuator) — 机身 LED 灯效, tool name="led"
# ══════════════════════════════════════════════════════════════════════════════

class LedPlugin:
    """T800 机身 LED 灯效控制 (actuator, tool name="led")。

    发布到 (domain 69): /hardware/led_control (interface_protocol/LedControl, 默认 QoS depth 10)
    模式枚举: 11 种灯效中文名 → LedControl.color 常量 0x1~0xb；reset 发送 0 还原内置灯光。
    """
    PREFIX = "led"

    # 中文名灯效 → LedControl.color 常量（官方 LedControl.msg）
    _LED_MODES = {
        "红闪": 0x1, "绿闪": 0x2, "蓝闪": 0x3, "白闪": 0x4,
        "白常亮": 0x5, "绿常亮": 0x6, "白呼吸": 0x7, "白流水": 0x8,
        "红呼吸": 0x9, "橙闪": 0xa, "橙常亮": 0xb,
    }

    def __init__(self, plugin_config: dict, namespace: str, ros2):
        self._plugin_config = plugin_config or {}
        self._ns = namespace
        self._ros2 = ros2
        self._pub_node = None
        self._pub = None
        self._current_mode = None  # 当前灯效中文名（None = 未设置/已还原）
        try:
            self._pub_node = ros2.make_node_t800("t800_led_pub")
        except Exception as e:  # noqa: BLE001
            print(f"[LedPlugin] WARNING: 无 ROS2 环境 ({e})，stub 模式")
            self._pub_node = None

    def get_tool(self) -> dict:
        return {
            "name": "led",
            "type": "actuator",
            "multiInstance": False,
            "description": (
                "T800 机身 LED 灯效控制 — 11 种模式枚举"
                "（红闪/绿闪/蓝闪/白闪/白常亮/绿常亮/白呼吸/白流水/红呼吸/橙闪/橙常亮），"
                "reset 还原内置灯光。"
            ),
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {
                        "type": "string",
                        "enum": ["set", "reset"],
                        "description": "要执行的操作",
                    },
                    "mode": {
                        "type": "string",
                        "enum": list(self._LED_MODES),
                        "description": "灯效模式（中文名）",
                    },
                },
                "required": ["action"],
                "x-action-params": {
                    "set": {"params": ["mode"], "description": "设置 LED 灯效模式"},
                    "reset": {"params": [], "description": "还原内置灯光"},
                },
            },
        }

    def start(self) -> None:
        """创建 LED 控制话题发布者（幂等）。"""
        try:
            from rclpy.qos import QoSProfile
            from ros2 import T800_TOPICS
            if self._pub_node is None:
                self._pub_node = self._ros2.make_node_t800("t800_led_pub")
            if self._pub is None:
                from interface_protocol.msg import LedControl
                topic = T800_TOPICS.get("led", "/hardware/led_control")
                self._pub = self._pub_node.create_publisher(
                    LedControl, topic, QoSProfile(depth=10))  # 官方示例默认 QoS
        except Exception as e:  # noqa: BLE001
            print(f"[LedPlugin] WARNING: 发布者创建失败 ({e})，stub 模式")

    def stop(self) -> None:
        pass

    def _publish(self, color: int) -> None:
        if self._pub is None:
            print("[LedPlugin] 发布者不可用（stub 模式），忽略 LED 指令")
            return
        try:
            from interface_protocol.msg import LedControl
            msg = LedControl()
            msg.color = int(color)
            self._pub.publish(msg)
        except Exception as e:  # noqa: BLE001
            print(f"[LedPlugin] 发布失败: {e}")

    def dispatch(self, action: str, args: dict) -> dict | None:
        if action == "start":
            self.start()
            return {"state": "ready"}
        if action == "stop":
            return {"state": "idle"}
        if action == "info":
            return {"state": "ready", "mode": self._current_mode,
                    "modes": list(self._LED_MODES)}
        if action == "set":
            mode = args.get("mode", "")
            color = self._LED_MODES.get(mode)
            if color is None:
                return {"state": "error", "error": "INVALID_MODE",
                        "message": f"未知灯效: {mode}，可选: {'/'.join(self._LED_MODES)}"}
            self._publish(color)
            self._current_mode = mode
            return {"state": "ok", "mode": mode, "color": color}
        if action == "reset":
            self._publish(0)
            self._current_mode = None
            return {"state": "ok", "mode": "还原"}
        return {"state": "error", "error": "INVALID_ARGUMENT",
                "message": f"未知 action: {action}"}


# ══════════════════════════════════════════════════════════════════════════════
# MicPlugin (sensor) — 内置麦克风采集, tool name="mic", readOnly
# ══════════════════════════════════════════════════════════════════════════════

class MicPlugin:
    """T800 内置麦克风采集 → domain42 AudioChunk (audio/pcm-16k)。

    采集: sounddevice.InputStream 请求 16kHz 单声道 int16；
          设备不支持 16k 时回退 48k + 手动线性重采样到 16k。
    缓冲: 内部攒够 512 samples (1024 字节) 才发布一个 AudioChunk
          —— ASR VAD 硬约束，低于 1024 字节的 chunk 会被静默丢弃。
    发布: (domain 42) /{ns}/mic/audio (audio_msgs/AudioChunk)。
    无声卡/无 sounddevice 时 start 返回 error，dispatch 以中文 message 提示。
    """
    PREFIX = "mic"
    _CHUNK_SAMPLES = 512   # 每 chunk 采样数 (16kHz)
    _CHUNK_BYTES = 1024    # 1024 字节
    _FLUSH_INTERVAL = 0.05  # 缓冲线程轮询周期 (s)

    def __init__(self, plugin_config: dict, namespace: str, ros2):
        self._plugin_config = plugin_config or {}
        self._ns = namespace
        self._ros2 = ros2
        self._topic = f"/{namespace}/mic/audio"
        self._running = False
        self._stream = None
        self._sd = None
        self._sd_error = ""
        self._sr = 16000
        self._resampling = False
        self._buf_lock = threading.Lock()
        self._buf = bytearray()          # int16 LE 样本缓冲
        self._pub_node = None
        self._pub = None
        self._AudioChunk = None
        self._Header = None
        self._thread = None
        self._samples_published = 0
        try:
            from audio_msgs.msg import AudioChunk
            from std_msgs.msg import Header
            from ros2 import QOS_CORE
            self._AudioChunk = AudioChunk
            self._Header = Header
            self._pub_node = ros2.make_node_core("t800_mic_pub")
            self._pub = self._pub_node.create_publisher(AudioChunk, self._topic, QOS_CORE)
        except Exception as e:  # noqa: BLE001
            print(f"[MicPlugin] WARNING: 无 ROS2 环境 ({e})，stub 模式")
            self._AudioChunk = None
            self._Header = None
            self._pub_node = None
            self._pub = None

    def get_tool(self) -> dict:
        return {
            "name": "mic",
            "type": "sensor",
            "multiInstance": False,
            "readOnly": True,
            "description": (
                "T800 内置麦克风 — 16kHz 单声道 PCM_S16_LE 采集，"
                "重采样并缓冲为 1024 字节/块发布到 " + self._topic +
                " (audio/pcm-16k)，满足 perception ASR 协议"
            ),
            "inputSchema": {"type": "object", "properties": {}},
            "topic_out": [{"topic": self._topic, "format": "audio/pcm-16k"}],
        }

    def start(self) -> str:
        """打开采集流并开始发布。返回 "ok" 表示成功，否则返回中文错误描述。"""
        if self._stream is not None:
            return "ok"
        try:
            import sounddevice as sd
            self._sd = sd
        except Exception as e:  # noqa: BLE001
            return f"sounddevice 不可用: {e}"
        # 首选 16kHz；失败回退 48kHz + 手动线性重采样到 16kHz
        last_err = None
        for sr, resampling in ((16000, False), (48000, True)):
            try:
                self._stream = self._sd.InputStream(
                    samplerate=sr, channels=1, dtype="int16",
                    blocksize=sr // 20, callback=self._on_audio)
                # 先设置采样率/重采样标志再 start()：声卡线程可能在 start 后立刻触发
                # 首个回调，若标志未就绪会把 48kHz 样本当作 16k 直接缓冲发布
                self._sr = sr
                self._resampling = resampling
                self._stream.start()
                break
            except Exception as e:  # noqa: BLE001
                self._stream = None
                last_err = e
        if self._stream is None:
            return f"打开声卡失败（无可用录音设备）: {last_err}"
        self._running = True
        self._thread = threading.Thread(target=self._flush_loop, daemon=True)
        self._thread.start()
        print(f"[MicPlugin] 采集启动 sr={self._sr} resample={self._resampling}")
        return "ok"

    def stop(self) -> None:
        """停止采集并释放声卡资源。"""
        self._running = False
        if self._stream is not None:
            try:
                self._stream.stop()
                self._stream.close()
            except Exception as e:  # noqa: BLE001
                print(f"[MicPlugin] 关闭采集流失败: {e}")
            self._stream = None

    def _on_audio(self, indata, frames, time_info, status):
        """sounddevice 采集回调（音频线程内执行，只缓冲不发布）。"""
        try:
            samples = indata[:, 0].tolist()  # 单声道 int16 样本
            if self._resampling:
                samples = _resample_linear(samples, self._sr, 16000)
            with self._buf_lock:
                self._buf.extend(struct.pack(f"<{len(samples)}h", *samples))
        except Exception as e:  # noqa: BLE001
            print(f"[MicPlugin] 采集回调错误: {e}")

    def _flush_loop(self) -> None:
        """后台线程：把缓冲的样本按 1024 字节一块切出来发布。"""
        while self._running:
            try:
                if self._pub is None or self._AudioChunk is None:
                    time.sleep(self._FLUSH_INTERVAL)
                    continue
                with self._buf_lock:
                    buf = self._buf
                    self._buf = bytearray()
                while len(buf) >= self._CHUNK_BYTES:
                    chunk = bytes(buf[:self._CHUNK_BYTES])
                    buf = buf[self._CHUNK_BYTES:]
                    self._publish_chunk(chunk)
                if buf:
                    # 不足一个 chunk 的残尾写回缓冲，跨轮继续累积（避免采样丢失）
                    with self._buf_lock:
                        self._buf = buf + self._buf
            except Exception as e:  # noqa: BLE001
                print(f"[MicPlugin] 缓冲线程错误: {e}")
            time.sleep(self._FLUSH_INTERVAL)

    def _publish_chunk(self, chunk: bytes) -> None:
        try:
            msg = self._AudioChunk()
            msg.header = self._Header()
            msg.header.stamp = self._pub_node.get_clock().now().to_msg()
            msg.format = "audio/pcm-16k"
            msg.data = list(chunk)
            self._pub.publish(msg)
            self._samples_published += len(chunk) // 2
        except Exception as e:  # noqa: BLE001
            print(f"[MicPlugin] 发布失败: {e}")

    def dispatch(self, action: str, args: dict) -> dict | None:
        if action == "start":
            err = self.start()
            if err == "ok":
                return {"state": "running", "message": "麦克风采集已启动"}
            return {"state": "error", "error": "AUDIO_CAPTURE_FAILED", "message": err}
        if action == "stop":
            self.stop()
            return {"state": "idle"}
        if action == "info":
            return {"state": "running" if self._running else "idle",
                    "topic_out": [{"topic": self._topic, "format": "audio/pcm-16k"}],
                    "samples_published": self._samples_published}
        return {"state": "error", "error": "INVALID_ARGUMENT",
                "message": f"未知 action: {action}"}


# ══════════════════════════════════════════════════════════════════════════════
# SpeakerPlugin (actuator) — 扬声器播放, tool name="speaker"
# ══════════════════════════════════════════════════════════════════════════════

class SpeakerPlugin:
    """T800 扬声器播放 (actuator, tool name="speaker")。

    用 sounddevice 在后台线程播放 WAV 文件或 base64 编码的 PCM_S16_LE 数据；
    无 ROS 话题。本机开发无 T800 声卡时返回 error 容错。
    """
    PREFIX = "speaker"

    def __init__(self, plugin_config: dict, namespace: str, ros2):
        self._plugin_config = plugin_config or {}
        self._ns = namespace
        self._ros2 = ros2
        self._lock = threading.Lock()
        self._state = "idle"          # idle / playing
        self._current = None          # 当前播放来源（文件路径 或 "pcm-data"）
        self._sd = None               # sounddevice 模块（延迟导入）
        self._sd_error = ""

    def get_tool(self) -> dict:
        return {
            "name": "speaker",
            "type": "actuator",
            "multiInstance": False,
            "description": "T800 扬声器 — 播放 WAV 文件 / base64 PCM 音频数据",
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {
                        "type": "string",
                        "enum": ["play_file", "play_wav", "stop", "list_devices"],
                        "description": "要执行的操作",
                    },
                    "path": {
                        "type": "string",
                        "description": "要播放的 WAV 文件路径",
                    },
                    "data": {
                        "type": "string",
                        "description": "base64 编码的 PCM_S16_LE 音频数据",
                    },
                    "sample_rate": {
                        "type": "integer",
                        "description": "PCM 采样率 (Hz)，默认 16000",
                    },
                },
                "required": ["action"],
                "x-action-params": {
                    "play_file": {"params": ["path"], "description": "播放本地 WAV 文件"},
                    "play_wav": {"params": ["data", "sample_rate"],
                                 "description": "播放 base64 编码的 PCM_S16_LE 音频"},
                    "stop": {"params": [], "description": "停止当前播放"},
                    "list_devices": {"params": [], "description": "列出可用音频输出设备"},
                },
            },
        }

    def start(self) -> None:
        pass

    def stop(self) -> None:
        """停止当前播放。"""
        if self._sd is not None:
            try:
                self._sd.stop()
            except Exception:  # noqa: BLE001
                pass
        with self._lock:
            self._state = "idle"
            self._current = None

    def _get_sd(self):
        """延迟导入 sounddevice；失败返回 None 并记录原因。"""
        if self._sd is None:
            try:
                import sounddevice as sd
                self._sd = sd
            except Exception as e:  # noqa: BLE001
                self._sd_error = f"sounddevice 不可用: {e}"
                return None
        return self._sd

    def _start_play(self, sd, data, sr, source) -> None:
        """后台线程播放（sd.play 非阻塞，wait 在后台线程等待结束）。"""
        def worker():
            try:
                with self._lock:
                    self._state = "playing"
                    self._current = source
                sd.play(data, sr)
                sd.wait()
            except Exception as e:  # noqa: BLE001
                print(f"[SpeakerPlugin] 播放错误: {e}")
            finally:
                with self._lock:
                    self._state = "idle"
                    self._current = None
        threading.Thread(target=worker, daemon=True).start()

    def _play_file(self, path: str) -> dict:
        sd = self._get_sd()
        if sd is None:
            return {"state": "error", "error": "AUDIO_UNAVAILABLE",
                    "message": self._sd_error}
        try:
            import wave
            import numpy as np
        except ImportError as e:  # noqa: BLE001
            return {"state": "error", "error": "AUDIO_UNAVAILABLE",
                    "message": f"缺少播放依赖 (numpy): {e}"}
        try:
            with wave.open(path, "rb") as wf:
                sr = wf.getframerate()
                channels = wf.getnchannels()
                sampwidth = wf.getsampwidth()
                frames = wf.readframes(wf.getnframes())
            if sampwidth != 2:
                return {"state": "error", "error": "UNSUPPORTED_FORMAT",
                        "message": "仅支持 16bit PCM WAV"}
            data = np.frombuffer(frames, dtype=np.int16)
            if channels > 1:
                # 多声道取平均混为单声道
                data = data.reshape(-1, channels).mean(axis=1).astype(np.int16)
            if len(data) == 0:
                return {"state": "error", "error": "LOAD_FAILED",
                        "message": f"文件无音频数据: {path}"}
        except Exception as e:  # noqa: BLE001
            return {"state": "error", "error": "LOAD_FAILED",
                    "message": f"读取 WAV 失败: {e}"}
        self._start_play(sd, data, sr, path)
        return {"state": "playing", "file": path, "sample_rate": sr}

    def _play_wav(self, b64data: str, sample_rate) -> dict:
        sd = self._get_sd()
        if sd is None:
            return {"state": "error", "error": "AUDIO_UNAVAILABLE",
                    "message": self._sd_error}
        if not b64data:
            return {"state": "error", "error": "INVALID_ARGUMENT",
                    "message": "缺少 data 参数（base64 编码的 PCM 数据）"}
        try:
            import numpy as np
        except ImportError as e:  # noqa: BLE001
            return {"state": "error", "error": "AUDIO_UNAVAILABLE",
                    "message": f"缺少播放依赖 (numpy): {e}"}
        try:
            pcm = base64.b64decode(b64data)
            sr = int(sample_rate or 16000)
            if sr <= 0:
                raise ValueError("sample_rate 必须为正整数")
            data = np.frombuffer(pcm, dtype=np.int16)
            if len(data) == 0:
                return {"state": "error", "error": "INVALID_ARGUMENT",
                        "message": "解码后无音频数据"}
        except Exception as e:  # noqa: BLE001
            return {"state": "error", "error": "INVALID_ARGUMENT",
                    "message": f"解码 PCM 失败: {e}"}
        self._start_play(sd, data, sr, "pcm-data")
        return {"state": "playing", "sample_rate": sr, "samples": int(len(data))}

    def _list_devices(self) -> dict:
        sd = self._get_sd()
        if sd is None:
            return {"state": "error", "error": "AUDIO_UNAVAILABLE",
                    "message": self._sd_error}
        try:
            devices = sd.query_devices()
            rows = [
                {
                    "index": i,
                    "name": dev.get("name", "?"),
                    "max_output_channels": int(dev.get("max_output_channels", 0)),
                }
                for i, dev in enumerate(devices)
            ]
            return {"devices": rows, "count": len(rows)}
        except Exception as e:  # noqa: BLE001
            return {"state": "error", "error": "AUDIO_UNAVAILABLE",
                    "message": f"查询设备失败: {e}"}

    def dispatch(self, action: str, args: dict) -> dict | None:
        if action == "start":
            return {"state": "ready"}
        if action == "stop":
            self.stop()
            return {"state": "idle"}
        if action == "info":
            with self._lock:
                state = self._state
                current = self._current
            return {"state": state, "current": current}
        if action == "play_file":
            path = args.get("path", "")
            if not path:
                return {"state": "error", "error": "INVALID_ARGUMENT",
                        "message": "缺少 path 参数"}
            return self._play_file(path)
        if action == "play_wav":
            return self._play_wav(args.get("data", ""), args.get("sample_rate"))
        if action == "list_devices":
            return self._list_devices()
        return {"state": "error", "error": "INVALID_ARGUMENT",
                "message": f"未知 action: {action}"}
