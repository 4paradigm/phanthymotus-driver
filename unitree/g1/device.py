#!/usr/bin/env python3
"""
drivers/unitree/g1/device.py — Unitree G1 设备插件（重构版）。

设计原则：
  - 一个设备 = 一个 tool，tool schema 含 type 字段（sensor / actuator）
  - sensor：只读声明，驱动启动时自动 start，数据通过 ROS2 topic 输出
  - actuator：单 tool + action 参数分发操作
  - start/stop 不暴露给 LLM，由驱动生命周期管理

插件：
  MicPlugin          (sensor)    — UDP multicast → ROS2 topic
  NativeTtsPlugin    (actuator)  — G1 内置 TTS + 音量控制
  LedPlugin          (actuator)  — LED 灯带控制
  LocoStatePlugin    (sensor)    — DDS SportModeState → ROS2 topic
  LocoPlugin         (actuator)  — 运动控制
  ArmActionPlugin    (actuator)  — 手臂动作
  StatePlugin        (sensor)    — DDS LowState → IMU/battery ROS2 topic
"""

import json
import math
import queue
import socket
import struct
import subprocess
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import Header, String
from audio_msgs.msg import AudioChunk

from unitree_sdk2py.g1.audio.g1_audio_client import AudioClient
from pointcloud_utils import gravity_align_inplace

# ── 常量 ──────────────────────────────────────────────────────────────────────

MIC_GROUP_IP = "239.168.123.161"
MIC_PORT     = 5555
MIC_RATE     = 16000          # Hz
CHUNK_BYTES  = 1024           # bytes per ROS2 publish (~32ms at 16kHz/16bit/mono)

_LOW_LAT_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=200,
    durability=DurabilityPolicy.VOLATILE,
)


# ── 工具函数 ──────────────────────────────────────────────────────────────────

def _get_local_ip() -> str:
    """返回本机在 192.168.123.x 网段的 IP；失败则用 UDP trick 兜底。"""
    try:
        import netifaces
        for iface in netifaces.interfaces():
            addrs = netifaces.ifaddresses(iface).get(netifaces.AF_INET, [])
            for addr in addrs:
                if addr["addr"].startswith("192.168.123."):
                    return addr["addr"]
    except ImportError:
        pass
    try:
        s = socket.socket(socket.AF_DGRAM)
        s.connect(("192.168.123.1", 1))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except Exception:
        return ""


# ── MicPlugin (sensor) ───────────────────────────────────────────────────────

class _MicNode(Node):
    def __init__(self, topic: str):
        super().__init__("g1_mic")
        self._topic  = topic
        self._pub    = self.create_publisher(AudioChunk, topic, _LOW_LAT_QOS)
        self._sock:   socket.socket | None = None
        self._thread: threading.Thread | None = None
        self.state   = "idle"
        self._packet_count = 0
        self._last_packet_ts = 0.0
        self.get_logger().info(f"MicNode ready — topic: {topic}")

    def start_capture(self) -> str:
        if self._sock is not None:
            return self._topic
        local_ip = _get_local_ip()
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind(("", MIC_PORT))
        mreq = struct.pack(
            "4s4s",
            socket.inet_aton(MIC_GROUP_IP),
            socket.inet_aton(local_ip) if local_ip else b"\x00\x00\x00\x00",
        )
        sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)
        sock.settimeout(0.5)
        self._sock   = sock
        self._packet_count = 0
        self._thread = threading.Thread(target=self._pump, daemon=True)
        self._thread.start()
        self.get_logger().info(f"Capture started — multicast {MIC_GROUP_IP}:{MIC_PORT}")
        return self._topic

    def stop_capture(self) -> None:
        if self._sock:
            try:
                self._sock.close()
            except Exception:
                pass
            self._sock = None
        self.state = "idle"
        self.get_logger().info("Capture stopped")

    def _pump(self) -> None:
        buf = bytearray()
        while self._sock is not None:
            try:
                data, _ = self._sock.recvfrom(65536)
            except socket.timeout:
                continue
            except OSError:
                break
            self._packet_count += 1
            self._last_packet_ts = time.monotonic()
            buf.extend(data)
            while len(buf) >= CHUNK_BYTES:
                chunk = bytes(buf[:CHUNK_BYTES])
                buf   = buf[CHUNK_BYTES:]
                try:
                    msg = AudioChunk()
                    msg.header = Header()
                    msg.header.stamp = self.get_clock().now().to_msg()
                    msg.format = "audio/pcm-16k"
                    msg.data   = list(chunk)
                    self._pub.publish(msg)
                except Exception as e:
                    self.get_logger().error(f"[mic] publish error: {e}")
                    break


class MicPlugin:
    PREFIX = "mic"

    def __init__(self, plugin_config: dict, namespace: str, executor, audio_client=None):
        self._topic = f"/{namespace}/mic/audio"
        self._node  = _MicNode(self._topic)
        executor.add_node(self._node)

    def get_tool(self) -> dict:
        return {
            "name": "mic",
            "type": "sensor",
            "multiInstance": False,
            "description": f"G1 microphone — captures UDP multicast audio (PCM-16 16kHz mono) and publishes to ROS2 topic {self._topic}",
            "inputSchema": {"type": "object", "properties": {}},
            "topic_out": [{"topic": self._topic, "format": "audio/pcm-16k"}],
        }

    def start(self) -> None:
        self._node.start_capture()  # start capture early but no self-check here

    def stop(self) -> None:
        self._node.stop_capture()

    def dispatch(self, action: str, args: dict) -> dict | None:
        if action == "start":
            # Start capture if not already running
            self._node.start_capture()
            # Self-check: verify full pipeline (multicast → ROS2 publish → subscribable)
            state, message = self._self_check()
            return {"state": state, "message": message} if message else {"state": state}
        if action == "stop":
            return {"state": "idle"}
        if action == "info":
            last_ago = int((time.monotonic() - self._node._last_packet_ts) * 1000) if self._node._last_packet_ts > 0 else -1
            return {
                "state": self._node.state,
                "topic_out": [{"topic": self._topic, "format": "audio/pcm-16k"}],
                "packets": self._node._packet_count,
                "last_packet_ago_ms": last_ago,
            }
        return None

    def _self_check(self) -> tuple[str, str]:
        """Verify mic pipeline: multicast receiving + ROS2 topic subscribable.

        Check 1: multicast packets arriving (in-process).
        Check 2: ROS2 topic receivable from a subprocess (avoids same-process
                 FastDDS intra-participant matching issues).
        """
        import time as _t

        # Check 1: multicast receiving
        if self._node._packet_count == 0:
            deadline = _t.monotonic() + 3.0
            while _t.monotonic() < deadline and self._node._packet_count == 0:
                _t.sleep(0.1)
        if self._node._packet_count == 0:
            self._node.state = "error"
            return "error", "no multicast packets received in 3s"

        # Check 2: ROS2 topic receivable — use subprocess to avoid same-process DDS issues
        check_script = (
            "import sys, rclpy, time;"
            "from rclpy.node import Node;"
            "from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy;"
            "from audio_msgs.msg import AudioChunk;"
            "rclpy.init();"
            "n = Node('_mic_check');"
            "qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,"
            "history=HistoryPolicy.KEEP_LAST, depth=10, durability=DurabilityPolicy.VOLATILE);"
            "ok = [False];"
            "n.create_subscription(AudioChunk, sys.argv[1], lambda m: ok.__setitem__(0, True), qos);"
            "dl = time.monotonic() + 3.0;"
            "\nwhile time.monotonic() < dl and not ok[0]: rclpy.spin_once(n, timeout_sec=0.1)\n"
            "rclpy.shutdown();"
            "sys.exit(0 if ok[0] else 1)"
        )
        try:
            result = subprocess.run(
                ["python3", "-c", check_script, self._topic],
                timeout=5,
                capture_output=True,
            )
            if result.returncode != 0:
                self._node.state = "error"
                return "error", "topic published but not receivable via ROS2"
        except (subprocess.TimeoutExpired, Exception) as e:
            self._node.state = "error"
            return "error", f"ROS2 subscribe check failed: {e}"

        self._node.state = "running"
        return "running", ""


# ── NativeTtsPlugin (actuator) ───────────────────────────────────────────────

class NativeTtsPlugin:
    PREFIX = "tts"

    def __init__(self, plugin_config: dict, namespace: str, executor, audio_client: AudioClient):
        self._client = audio_client

    def get_tool(self) -> dict:
        return {
            "name": "tts",
            "type": "actuator",
            "multiInstance": False,
            "description": "G1 on-board TTS engine — synthesize text to robot speech, control volume",
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {
                        "type": "string",
                        "enum": ["speak", "get_volume", "set_volume"],
                        "description": "Action to perform",
                    },
                    "text":   {"type": "string",  "description": "Text to speak"},
                    "voice":  {"type": "integer", "description": "Voice ID (default 0)"},
                    "volume": {"type": "integer", "description": "Volume 0-100"},
                },
                "required": ["action"],
                "x-action-params": {
                    "speak":      {"params": ["text", "voice"],  "description": "Synthesize text to speech on the robot"},
                    "get_volume": {"params": [],                 "description": "Get current speaker volume"},
                    "set_volume": {"params": ["volume"],         "description": "Set speaker volume (0-100)"},
                },
            },
        }

    def start(self) -> None:
        pass

    def stop(self) -> None:
        pass

    def dispatch(self, action: str, args: dict) -> dict | None:
        if action == "start":
            return {"state": "ready"}
        if action == "stop":
            return {"state": "idle"}
        if action == "speak":
            text  = args.get("text", "")
            voice = int(args.get("voice", 0))
            ret   = self._client.TtsMaker(text, voice)
            return {"ret": ret, "text": text}
        elif action == "get_volume":
            ret = self._client.GetVolume()
            return {"ret": ret}
        elif action == "set_volume":
            vol = int(args.get("volume", 50))
            ret = self._client.SetVolume(vol)
            return {"ret": ret, "volume": vol}
        return None


# ── SpeakerPlugin (actuator) ─────────────────────────────────────────────────

APP_NAME = "g1_speaker"


class _SpeakerNode(Node):
    PREFILL = 3       # buffer 3 chunks (~300ms) before starting playback
    MERGE_BYTES = 9600  # merge into ~300ms blocks before calling PlayStream

    def __init__(self, audio_client: AudioClient):
        super().__init__("g1_speaker")
        self._client = audio_client
        self._topic: str | None = None
        self._sub    = None
        self._idx    = 0
        self.state   = "idle"
        self._buf = queue.Queue()
        self._draining = threading.Event()
        self._drain_thread: threading.Thread | None = None
        self._last_chunk_time = 0.0
        self._flush_timer = None
        # 打断/暂停控制
        self._lock = threading.Lock()
        self._interrupt_flag = threading.Event()
        self._pause_event = threading.Event()
        self._pause_event.set()  # 初始为非暂停状态
        self._muted = False  # interrupt 后静默，丢弃后续 chunks 直到新 utterance
        # Clear stale PlayStream session from previous container run (MCU keeps state across reboot)
        self._client.PlayStop(APP_NAME)
        self.get_logger().info("SpeakerNode ready")

    def start_play(self, topic: str) -> str:
        if self._sub is not None:
            if self._topic == topic:
                self.get_logger().info(f"[speaker] already subscribing {topic}, skip")
                return self._topic
            # topic changed — stop old subscription first
            self.get_logger().info(f"[speaker] topic changed {self._topic} → {topic}, re-subscribing")
            self.stop_play()
        self._topic = topic
        self._muted = False  # 新 start 时清除静默
        self.get_logger().info(f"[speaker] creating subscription: topic={topic}, msg_type=AudioChunk, qos=LOW_LAT")
        self._sub = self.create_subscription(
            AudioChunk, topic, self._on_chunk, _LOW_LAT_QOS,
        )
        self.state = "ready"
        self.get_logger().info(f"[speaker] subscription created, waiting for chunks on {topic}")
        return topic

    def stop_play(self) -> None:
        if self._flush_timer is not None:
            self._flush_timer.cancel()
            self.destroy_timer(self._flush_timer)
            self._flush_timer = None
        if self._sub is not None:
            self.destroy_subscription(self._sub)
            self._sub = None
        self._draining.clear()
        self._pause_event.set()  # 确保 drain thread 不会卡在 pause wait
        self._interrupt_flag.set()  # 确保 drain thread 退出
        if self._drain_thread is not None:
            self._drain_thread.join(timeout=2)
            self._drain_thread = None
        self._interrupt_flag.clear()
        # flush remaining buffer
        while not self._buf.empty():
            try:
                self._buf.get_nowait()
            except queue.Empty:
                break
        try:
            self._client.PlayStop(APP_NAME)
        except Exception as e:
            self.get_logger().warn(f"PlayStop error: {e}")
        self.state = "idle"
        self.get_logger().info("Speaker stopped")

    def interrupt(self) -> dict:
        """立即中止播放：清空 buffer，停止 SDK，保持 subscription。"""
        with self._lock:
            self._interrupt_flag.set()
            # 清空 buffer
            while not self._buf.empty():
                try:
                    self._buf.get_nowait()
                except queue.Empty:
                    break
            # 停止 SDK 播放
            try:
                self._client.PlayStop(APP_NAME)
            except Exception as e:
                self.get_logger().warn(f"[speaker] interrupt PlayStop error: {e}")
            # 等 drain thread 退出
            if self._drain_thread is not None and self._drain_thread.is_alive():
                self._drain_thread.join(timeout=1)
                self._drain_thread = None
            self._interrupt_flag.clear()
            self._pause_event.set()
            self._draining.clear()
            self._muted = True  # 静默：丢弃后续 TTS chunks 直到新 utterance
            self.state = "ready"
        self.get_logger().info("[speaker] interrupted — buffer cleared, muted until new utterance")
        return {"state": "ready", "action": "interrupted"}

    def pause(self) -> dict:
        """暂停播放：停止 SDK，保留 buffer 中未播放的内容。"""
        with self._lock:
            if self.state not in ("playing", "ready"):
                return {"state": self.state, "error": "not playing"}
            self._pause_event.clear()  # drain thread 将阻塞在 wait()
            try:
                self._client.PlayStop(APP_NAME)
            except Exception as e:
                self.get_logger().warn(f"[speaker] pause PlayStop error: {e}")
            self.state = "paused"
        self.get_logger().info(f"[speaker] paused — buffer size={self._buf.qsize()}")
        return {"state": "paused", "buffer_chunks": self._buf.qsize()}

    def resume(self) -> dict:
        """恢复播放：从 buffer 中剩余内容继续。"""
        with self._lock:
            if self.state != "paused":
                return {"state": self.state, "error": "not paused"}
            self.state = "playing"
            self._pause_event.set()  # 唤醒 drain thread
            # 如果 drain thread 已经退出了（pause 时退出），重新启动
            if self._drain_thread is None or not self._drain_thread.is_alive():
                if not self._buf.empty():
                    self._start_drain()
        self.get_logger().info("[speaker] resumed")
        return {"state": "playing"}

    # EOF magic: 8 bytes (4 samples [1,-1,1,-1])，标记 utterance 结束
    AUDIO_EOF_MAGIC = b'\x01\x00\xff\xff\x01\x00\xff\xff'

    def _on_chunk(self, msg: AudioChunk) -> None:
        pcm = bytes(msg.data)
        now = time.monotonic()
        self._idx += 1

        # 检测 EOF magic：utterance 结束标记
        if len(pcm) == 8 and pcm == self.AUDIO_EOF_MAGIC:
            if self._muted:
                self._muted = False
                self.get_logger().info("[speaker] unmuted — received EOF marker")
            return  # EOF 不入 buffer、不播放

        # Muted 状态：interrupt 后丢弃来自旧 utterance 的 chunks
        if self._muted:
            self._last_chunk_time = now
            return  # 丢弃，等 EOF 到达

        self._buf.put(pcm)
        self._last_chunk_time = now
        # 更新状态：收到 chunk 时如果是 ready，变为 playing
        if self.state == "ready":
            self.state = "playing"
        if not self._draining.is_set() and self.state == "playing" and self._buf.qsize() >= self.PREFILL:
            self._start_drain()
        elif not self._draining.is_set() and self.state == "playing" and self._flush_timer is None:
            # start a flush timer — if no more chunks arrive, drain what we have
            self._flush_timer = self.create_timer(0.2, self._check_flush)

    def _start_drain(self) -> None:
        if self._flush_timer is not None:
            self._flush_timer.cancel()
            self.destroy_timer(self._flush_timer)
            self._flush_timer = None
        self._draining.set()
        self._drain_thread = threading.Thread(target=self._drain, daemon=True)
        self._drain_thread.start()

    def _check_flush(self) -> None:
        """Timer callback: if no new chunks for 300ms and buffer non-empty, start drain."""
        if self._flush_timer is not None:
            self._flush_timer.cancel()
            self.destroy_timer(self._flush_timer)
            self._flush_timer = None
        if not self._draining.is_set() and not self._buf.empty() and self.state == "playing":
            idle = time.monotonic() - self._last_chunk_time
            if idle >= 0.15:
                self._start_drain()

    def _drain(self) -> None:
        play_idx = 0
        merged = b''
        empty_count = 0
        while self._draining.is_set():
            # 检查 interrupt
            if self._interrupt_flag.is_set():
                return
            # 检查 pause（阻塞等待 resume 或 interrupt）
            if not self._pause_event.wait(timeout=0.1):
                continue  # 还在 paused，循环检查 interrupt

            try:
                pcm = self._buf.get(timeout=0.1)
                merged += pcm
                empty_count = 0
            except queue.Empty:
                empty_count += 1
                if merged and empty_count >= 2:
                    play_idx += 1
                    self._play_merged(merged, play_idx)
                    merged = b''
                elif not merged and empty_count >= 3:
                    break
                continue
            if len(merged) >= self.MERGE_BYTES:
                play_idx += 1
                self._play_merged(merged, play_idx)
                merged = b''
        if merged and not self._interrupt_flag.is_set():
            play_idx += 1
            self._play_merged(merged, play_idx)
        self._draining.clear()
        # 播放完毕，回到 ready（如果没有被 interrupt/stop）
        if self.state == "playing":
            self.state = "ready"
        self.get_logger().info("[speaker] drain finished")

    def _play_merged(self, pcm: bytes, idx: int) -> None:
        # 播放前再次检查 interrupt
        if self._interrupt_flag.is_set():
            return
        duration = len(pcm) / 32000
        t0 = time.monotonic()
        try:
            code, data = self._client.PlayStream(APP_NAME, "0", pcm)
            if code != 0:
                self.get_logger().error(f"[speaker] PlayStream error code={code}, data={data}")
        except Exception as e:
            self.get_logger().error(f"[speaker] PlayStream error: {e}")
        elapsed = time.monotonic() - t0
        remaining = duration - elapsed - 0.08
        if remaining > 0 and not self._interrupt_flag.is_set():
            time.sleep(remaining)


class SpeakerPlugin:
    PREFIX = "speaker"

    def __init__(self, plugin_config: dict, namespace: str, executor, audio_client: AudioClient):
        self._node = _SpeakerNode(audio_client)
        executor.add_node(self._node)

    def get_tool(self) -> dict:
        return {
            "name": "speaker",
            "type": "actuator",
            "multiInstance": False,
            "description": "G1 speaker — subscribes to ROS2 topic and streams PCM-16k audio to robot speaker",
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {
                        "type": "string",
                        "enum": ["start", "stop", "info"],
                        "description": "Action to perform",
                    },
                    "input_topic": {
                        "type": "string",
                        "description": "ROS2 topic to subscribe for PCM audio (provided by canvas connection)",
                    },
                },
                "required": ["action"],
            },
            "topic_in": [{"format": "audio/pcm-16k"}],
        }

    def start(self) -> None:
        pass  # startup sound is played on first dispatch(start) when project starts

    def _play_startup_sound(self) -> None:
        """Play startup PCM by directly calling PlayStream in small blocks with pacing."""
        import pathlib
        pcm_path = pathlib.Path(__file__).parent / 'resource' / 'startup_beep.pcm'
        try:
            pcm = pcm_path.read_bytes()
            block_size = 9600  # ~300ms per block
            for offset in range(0, len(pcm), block_size):
                block = pcm[offset:offset + block_size]
                code, _ = self._node._client.PlayStream(APP_NAME, "0", block)
                if code != 0:
                    self._node.get_logger().warn(f"[speaker] startup sound stopped at offset {offset}: code={code}")
                    return
                duration = len(block) / 32000
                remaining = duration - 0.08
                if remaining > 0:
                    time.sleep(remaining)
            self._node.get_logger().info(f"[speaker] startup sound OK ({len(pcm)} bytes)")
        except Exception as e:
            self._node.get_logger().warn(f"[speaker] startup sound error: {e}")

    def stop(self) -> None:
        self._node.stop_play()

    def dispatch(self, action: str, args: dict) -> dict | None:
        if action != "info":
            self._node.get_logger().info(f"[speaker] dispatch action={action}, args={args}")
        if action in ("start", "play"):
            topic = args.get("input_topic", "")
            if not topic:
                return {"error": "Missing input_topic"}
            # Always stop first to ensure clean restart
            self._node.stop_play()
            # Play startup sound synchronously before starting subscription
            self._play_startup_sound()
            topic = self._node.start_play(topic)
            return {"state": "ready", "topic": topic}
        elif action == "stop":
            self._node.stop_play()
            return {"state": "idle"}
        elif action == "info":
            return {
                "state": self._node.state,
                "topic": self._node._topic,
                "buffer_chunks": self._node._buf.qsize(),
            }
        return None


# ── SmartMotionPlugin (controller) ─────────────────────────────────────────

class SmartMotionPlugin:
    """统一打断/暂停控制卡片。协调 speaker + loco 的中止和暂停。"""
    PREFIX = "smart_motion"

    def __init__(self, plugin_config: dict, namespace: str, executor,
                 speaker_plugin=None, loco_plugin=None):
        self._speaker = speaker_plugin
        self._loco = loco_plugin

    def get_tool(self) -> dict:
        return {
            "name": "smart_motion",
            "type": "actuator",
            "multiInstance": False,
            "description": "SmartMotion — 统一运动/输出控制，提供打断、暂停、恢复能力",
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {
                        "type": "string",
                        "enum": ["interrupt_all", "interrupt_speak", "interrupt_motion",
                                 "pause_speak", "resume_speak", "status"],
                        "description": "Action to perform",
                    },
                },
                "required": ["action"],
                "x-action-params": {
                    "interrupt_all":    {"params": [], "description": "中止所有输出（语音+动作同时停止）"},
                    "interrupt_speak":  {"params": [], "description": "中止语音播放，清空待播队列"},
                    "interrupt_motion": {"params": [], "description": "停止机器人当前运动"},
                    "pause_speak":      {"params": [], "description": "暂停语音播放（保留未播内容，可恢复）"},
                    "resume_speak":     {"params": [], "description": "恢复之前暂停的语音播放"},
                    "status":           {"params": [], "description": "查询当前输出状态（语音/运动）"},
                },
            },
        }

    def start(self) -> None:
        pass

    def stop(self) -> None:
        pass

    def dispatch(self, action: str, args: dict) -> dict | None:
        if action == "start":
            return {"state": "ready"}
        if action == "stop":
            return {"state": "idle"}
        if action == "interrupt_all":
            r1 = self._do_interrupt_speak()
            r2 = self._do_interrupt_motion()
            return {"speak": r1, "motion": r2}
        elif action == "interrupt_speak":
            return self._do_interrupt_speak()
        elif action == "interrupt_motion":
            return self._do_interrupt_motion()
        elif action == "pause_speak":
            if self._speaker:
                return self._speaker._node.pause()
            return {"error": "no speaker plugin"}
        elif action == "resume_speak":
            if self._speaker:
                return self._speaker._node.resume()
            return {"error": "no speaker plugin"}
        elif action == "status":
            return {
                "speak": self._speaker.dispatch("info", {}) if self._speaker else None,
                "motion": self._loco.dispatch("info", {}) if self._loco else None,
            }
        return None

    def _do_interrupt_speak(self) -> dict | None:
        if self._speaker:
            return self._speaker._node.interrupt()
        return {"error": "no speaker plugin"}

    def _do_interrupt_motion(self) -> dict | None:
        if self._loco:
            return self._loco.dispatch("stop_move", {})
        return {"error": "no loco plugin"}


# ── LedPlugin (actuator) ─────────────────────────────────────────────────────

class LedPlugin:
    PREFIX = "led"

    def __init__(self, plugin_config: dict, namespace: str, executor, audio_client: AudioClient):
        self._client = audio_client

    def get_tool(self) -> dict:
        return {
            "name": "led",
            "type": "actuator",
            "multiInstance": False,
            "description": "G1 LED strip control — set RGB color or turn off",
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {
                        "type": "string",
                        "enum": ["set", "off"],
                        "description": "Action to perform",
                    },
                    "r": {"type": "integer", "description": "Red 0-255"},
                    "g": {"type": "integer", "description": "Green 0-255"},
                    "b": {"type": "integer", "description": "Blue 0-255"},
                },
                "required": ["action"],
                "x-action-params": {
                    "set": {"params": ["r", "g", "b"], "description": "Set LED strip to specified RGB color"},
                    "off": {"params": [],              "description": "Turn off LED strip"},
                },
            },
        }

    def start(self) -> None:
        pass

    def stop(self) -> None:
        pass

    def dispatch(self, action: str, args: dict) -> dict | None:
        if action == "start":
            return {"state": "ready"}
        if action == "stop":
            return {"state": "idle"}
        if action == "set":
            r   = int(args.get("r", 0))
            g   = int(args.get("g", 0))
            b   = int(args.get("b", 0))
            ret = self._client.LedControl(r, g, b)
            return {"ret": ret, "r": r, "g": g, "b": b}
        elif action == "off":
            ret = self._client.LedControl(0, 0, 0)
            return {"ret": ret}
        return None


# ── LocoStatePlugin (sensor) ─────────────────────────────────────────────────

class _LocoStateNode(Node):
    """Subscribes to DDS odommodestate + sportmodestate and republishes as JSON to ROS2."""

    _ODOM_INTERVAL = 0.1  # 10 Hz throttle

    def __init__(self, odom_topic: str, motion_topic: str):
        super().__init__("g1_loco_state")
        self._odom_pub   = self.create_publisher(String, odom_topic,   _LOW_LAT_QOS)
        self._motion_pub = self.create_publisher(String, motion_topic, _LOW_LAT_QOS)
        self._last_state: dict = {}
        self._lock       = threading.Lock()
        self._last_odom_time: float = 0.0

        try:
            from unitree_sdk2py.core.channel import ChannelSubscriber
            from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_
            odom_sub = ChannelSubscriber("rt/odommodestate", SportModeState_)
            odom_sub.Init(self._on_odom, 10)
            self.get_logger().info(f"LocoStateNode subscribed rt/odommodestate → {odom_topic}")
        except Exception as e:
            self.get_logger().warn(f"LocoStateNode: failed to subscribe rt/odommodestate: {e}")

        try:
            from unitree_sdk2py.core.channel import ChannelSubscriber
            from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_
            sport_sub = ChannelSubscriber("rt/sportmodestate", SportModeState_)
            sport_sub.Init(self._on_motion, 10)
            self.get_logger().info(f"LocoStateNode subscribed rt/sportmodestate → {motion_topic}")
        except Exception as e:
            self.get_logger().warn(f"LocoStateNode: failed to subscribe rt/sportmodestate: {e}")

    def _format_state(self, msg) -> dict:
        imu = msg.imu_state
        return {
            "mode":          msg.mode,
            "gait_type":     msg.gait_type,
            "body_height":   msg.body_height,
            "position":      list(msg.position),
            "velocity":      list(msg.velocity),
            "yaw_speed":     msg.yaw_speed,
            "foot_force":    list(msg.foot_force),
            "imu": {
                "quaternion":    list(imu.quaternion),
                "gyroscope":     list(imu.gyroscope),
                "accelerometer": list(imu.accelerometer),
                "rpy":           list(imu.rpy),
            },
        }

    def _on_odom(self, msg) -> None:
        now = time.monotonic()
        if now - self._last_odom_time < self._ODOM_INTERVAL:
            return
        self._last_odom_time = now

        state = self._format_state(msg)
        with self._lock:
            self._last_state = state
        out = String()
        out.data = json.dumps(state)
        self._odom_pub.publish(out)

    def _on_motion(self, msg) -> None:
        state = self._format_state(msg)
        out = String()
        out.data = json.dumps(state)
        self._motion_pub.publish(out)

    def get_last_state(self) -> dict:
        with self._lock:
            return dict(self._last_state)


class LocoStatePlugin:
    PREFIX = "loco_state"

    def __init__(self, plugin_config: dict, namespace: str, executor):
        self._odom_topic   = f"/{namespace}/loco/state"
        self._motion_topic = f"/{namespace}/loco/motion_state"
        self._node = _LocoStateNode(self._odom_topic, self._motion_topic)
        executor.add_node(self._node)

    def get_tools(self) -> list:
        return [self._odom_tool(), self._motion_tool()]

    def _odom_tool(self) -> dict:
        return {
            "name": "loco_state",
            "type": "sensor",
            "multiInstance": False,
            "description": f"G1 locomotion state (always active) — mode, velocity, position, body_height, foot_force, IMU. Publishes at 10Hz to {self._odom_topic}",
            "inputSchema": {"type": "object", "properties": {}},
            "topic_out": [{"topic": self._odom_topic, "format": "data/json"}],
        }

    def _motion_tool(self) -> dict:
        return {
            "name": "loco_motion_state",
            "type": "sensor",
            "multiInstance": False,
            "description": f"G1 sport mode state (only active when standing/walking) — same fields as loco_state but from motion controller. Publishes to {self._motion_topic}",
            "inputSchema": {"type": "object", "properties": {}},
            "topic_out": [{"topic": self._motion_topic, "format": "data/json"}],
        }

    def start(self) -> None:
        pass  # DDS subscription starts in __init__

    def stop(self) -> None:
        pass

    def dispatch(self, action: str, args: dict) -> dict | None:
        if action == "start":
            return {"state": "running"}
        if action == "stop":
            return {"state": "idle"}
        if action == "info":
            tool_name = args.get('_tool_name', '')
            if tool_name == 'loco_motion_state':
                return {"state": "running", "topic_out": [{"topic": self._motion_topic, "format": "data/json"}]}
            return {"state": "running", "topic_out": [{"topic": self._odom_topic, "format": "data/json"}]}
        return None


# ── LocoPlugin (actuator) ────────────────────────────────────────────────────

class LocoPlugin:
    PREFIX = "loco"

    def __init__(self, plugin_config: dict, namespace: str, executor, loco_client, slam_client=None, smart_motion=None):
        self._client = loco_client
        self._slam_client = slam_client
        self._smart_motion = smart_motion
        self._namespace = namespace
        self._move_timer: threading.Timer | None = None

    def get_tools(self) -> list:
        tools = [self._loco_tool(), self._switch_mode_tool(), self._switch_mode_expert_tool()]
        if self._smart_motion:
            tools.append(self._motion_events_tool())
        return tools

    def _motion_events_tool(self) -> dict:
        topic = f"/{self._namespace}/safety/motion_events"
        return {
            "name": "motion_events",
            "type": "sensor",
            "multiInstance": False,
            "description": f"SmartMotion safety harness events — motion_start/stop/decelerate/resume, nav_start/paused/resumed/stopped, safety_stop (tilt/foot_airborne/comm_timeout/overheat). Publishes to {topic}",
            "inputSchema": {"type": "object", "properties": {}},
            "topic_out": [{"topic": topic, "format": "data/json"}],
        }

    def _loco_tool(self) -> dict:
        return {
            "name": "loco",
            "type": "actuator",
            "multiInstance": False,
            "description": "G1 locomotion control — move, stop, set height, get state, wave/shake hand",
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {
                        "type": "string",
                        "enum": ["move", "stop_move", "set_stand_height", "get_fsm_id", "get_fsm_mode", "get_balance_mode", "get_swing_height", "get_stand_height", "get_phase", "wave_hand", "shake_hand"],
                        "description": "Action to perform",
                    },
                    "vx":         {"type": "number", "description": "Forward velocity m/s [-1, 1]"},
                    "vy":         {"type": "number", "description": "Lateral velocity m/s [-1, 1]"},
                    "vyaw":       {"type": "number", "description": "Yaw rotation rad/s [-2, 2]"},
                    "duration":   {"type": "number", "description": "Move duration in seconds. 0 or negative = move until explicit stop (default 0)"},
                    "height":     {"type": "number", "description": "Normalized height 0.0-1.0"},
                    "turn":       {"type": "boolean", "description": "Turn while waving (default false)"},
                },
                "required": ["action"],
                "x-action-params": {
                    "move":             {"params": ["vx", "vy", "vyaw", "duration"], "description": "Move with specified velocities. duration>0 for timed move, 0 or negative for continuous until stop."},
                    "stop_move":        {"params": [],                                 "description": "Stop all movement immediately"},
                    "set_stand_height": {"params": ["height"],                         "description": "Set the robot's standing height (0.0-1.0)"},
                    "get_fsm_id":       {"params": [],                                 "description": "Get current FSM state ID"},
                    "get_fsm_mode":     {"params": [],                                 "description": "Get current FSM mode"},
                    "get_balance_mode": {"params": [],                                 "description": "Get current balance mode"},
                    "get_swing_height": {"params": [],                                 "description": "Get current swing height"},
                    "get_stand_height": {"params": [],                                 "description": "Get current stand height"},
                    "get_phase":        {"params": [],                                 "description": "Get current gait phase (deprecated)"},
                    "wave_hand":        {"params": ["turn"],                           "description": "Perform a waving hand gesture"},
                    "shake_hand":       {"params": [],                                 "description": "Perform a handshake gesture"},
                },
            },
        }

    # ── FSM state groups for safety checks ──────────────────────────────────────
    _GROUND_STATES = {0, 1}            # zero_torque, damp — lying on ground
    _LOW_STATES = {2, 702}             # squat, prep — stable low stance
    _STANDING_STATES = {500, 501, 801} # normal_loco, 3dof_waist, run — active balance
    _UNSAFE_STATES = {3, 706}          # sit, balance_stand — not directly switchable

    def _switch_mode_tool(self) -> dict:
        return {
            "name": "switch_mode",
            "type": "actuator",
            "multiInstance": False,
            "description": "G1 safe locomotion mode switch. "
                           "lie2standup=安全起立(ground→主运控), standup2lie=安全躺下(standing→阻尼), "
                           "standup2squat=站到蹲(standing→下蹲), squat2standup=蹲到站(下蹲→主运控), "
                           "damp=阻尼(ground only), zero_torque=零力矩(ground only), "
                           "emergency_stop=紧急阻尼(any state), get_current_mode=查询当前状态",
            "inputSchema": {
                "type": "object",
                "properties": {
                    "mode": {
                        "type": "string",
                        "enum": ["lie2standup", "standup2lie", "standup2squat", "squat2standup",
                                 "damp", "zero_torque", "emergency_stop", "get_current_mode"],
                        "description": "Target mode",
                    },
                },
                "required": ["mode"],
            },
        }

    def _switch_mode_expert_tool(self) -> dict:
        return {
            "name": "switch_mode_expert",
            "type": "actuator",
            "multiInstance": False,
            "description": "G1 locomotion mode switch — directly set FSM mode ID (EXPERT ONLY, bypasses safety checks, robot may fall!). IDs: 0=zero_torque, 1=damp, 2=squat, 3=sit, 4=lock_stand, 500=normal_loco, 501=3dof_waist, 702=lie_to_stand, 706=balance_squat, 801=run_loco",
            "inputSchema": {
                "type": "object",
                "properties": {
                    "fsm_id": {
                        "type": "integer",
                        "description": "FSM mode ID",
                    },
                },
                "required": ["fsm_id"],
            },
        }

    def start(self) -> None:
        pass

    def stop(self) -> None:
        if self._move_timer:
            self._move_timer.cancel()
            self._move_timer = None
        self._client.StopMove()

    def _auto_stop(self):
        """Timer 回调：自动停止运动"""
        self._move_timer = None
        self._client.StopMove()

    def dispatch(self, action: str, args: dict) -> dict | None:
        if action == "start":
            return {"state": "ready"}
        if action == "stop":
            return {"state": "idle"}
        if action == "info":
            tool_name = args.get("_tool_name", "motion_events")
            if tool_name == "motion_events" and self._smart_motion:
                topic = f"/{self._namespace}/safety/motion_events"
                return {"state": "running", "topic_out": [{"topic": topic, "format": "data/json"}]}
            return None
        if action == "move":
            vx   = float(args.get("vx",   0))
            vy   = float(args.get("vy",   0))
            vyaw = float(args.get("vyaw", 0))
            duration = float(args.get("duration", 0))

            # Route through SmartMotion safety harness
            if self._smart_motion:
                return self._smart_motion.move(vx, vy, vyaw, duration)

            # Fallback: direct control (no safety harness)
            vx   = max(-1.0, min(1.0, vx))
            vy   = max(-1.0, min(1.0, vy))
            vyaw = max(-2.0, min(2.0, vyaw))

            if self._move_timer:
                self._move_timer.cancel()
                self._move_timer = None

            if duration > 0:
                # G1 SetVelocity duration has known bugs — use Timer fallback
                ret = self._client.Move(vx, vy, vyaw, True)
                self._move_timer = threading.Timer(duration, self._auto_stop)
                self._move_timer.start()
            else:
                # Continuous move until explicit stop
                ret = self._client.Move(vx, vy, vyaw, True)

            return {"ret": ret, "vx": vx, "vy": vy, "vyaw": vyaw, "duration": duration}
        elif action == "stop_move":
            # Route through SmartMotion safety harness
            if self._smart_motion:
                return self._smart_motion.stop()

            # Fallback: direct control
            if self._move_timer:
                self._move_timer.cancel()
                self._move_timer = None
            if self._slam_client:
                try:
                    self._slam_client.PauseNav()
                except Exception:
                    pass
            ret = self._client.StopMove()
            return {"ret": ret}
        elif action == "switch_mode":
            mode = args.get("mode", "")
            code, current_fsm = self._client.GetFsmId()

            if code != 0:
                return {"error": f"Cannot read current FSM state (code={code}). Aborting for safety."}

            if mode == "emergency_stop":
                ret = self._client.Damp()
                return {"ret": ret, "mode": "emergency_stop",
                        "warning": "Emergency damp executed regardless of state"}

            elif mode == "get_current_mode":
                FSM_DESCRIPTIONS = {
                    0: "lying down, zero torque (零力矩, no resistance)",
                    1: "lying down, damping (阻尼, resists movement)",
                    2: "squatting (下蹲, position hold, stable)",
                    3: "sitting (落座, needs external support, unstable)",
                    500: "standing, normal locomotion (主运控, balanced)",
                    501: "standing, 3DOF waist locomotion (balanced)",
                    702: "prep stance (预备模式, stable low stance)",
                    706: "balance stand (过渡态, intermediate)",
                    801: "standing, running gait (跑步运控, balanced)",
                }
                desc = FSM_DESCRIPTIONS.get(current_fsm, f"unknown state")
                return {"fsm_id": current_fsm, "description": desc}

            elif mode == "lie2standup":
                if current_fsm in self._STANDING_STATES:
                    return {"info": "Robot is already standing", "fsm_id": current_fsm}
                if current_fsm in self._UNSAFE_STATES:
                    return {"error": f"Robot is in unsafe state (FSM={current_fsm}). Use emergency_stop first."}
                # From ground: damp → lie2stand(702) → auto到500
                if current_fsm in self._GROUND_STATES:
                    steps = []
                    if current_fsm == 0:
                        steps.append(("Damp", 1, "damp"))
                    steps.append(("Lie2StandUp", 500, "lie2standup"))
                    return self._run_fsm_sequence(steps)
                # From low states (squat/prep): start(500)
                if current_fsm in self._LOW_STATES:
                    steps = [("Start", 500, "start")]
                    return self._run_fsm_sequence(steps)
                return {"error": f"Cannot stand up from FSM={current_fsm}"}

            elif mode == "standup2lie":
                if current_fsm in self._GROUND_STATES:
                    return {"info": "Robot is already on the ground", "fsm_id": current_fsm}
                if current_fsm in self._STANDING_STATES:
                    # Stop movement first, wait for stabilization
                    self._client.StopMove()
                    import time as _time; _time.sleep(1.0)
                    steps = [("StandUp2Squat", 2, "standup2squat"), ("Damp", 1, "damp")]
                    return self._run_fsm_sequence(steps)
                if current_fsm in self._LOW_STATES:
                    steps = [("Damp", 1, "damp")]
                    return self._run_fsm_sequence(steps)
                return {"error": f"Cannot lie down from FSM={current_fsm}. Use emergency_stop if needed."}

            elif mode == "standup2squat":
                if current_fsm in self._GROUND_STATES or current_fsm in self._LOW_STATES:
                    return {"info": "Robot is already in low/ground state", "fsm_id": current_fsm}
                if current_fsm in self._STANDING_STATES:
                    self._client.StopMove()
                    import time as _time; _time.sleep(1.0)
                    steps = [("StandUp2Squat", 2, "standup2squat")]
                    return self._run_fsm_sequence(steps)
                return {"error": f"Cannot squat from FSM={current_fsm}. Use emergency_stop if needed."}

            elif mode == "squat2standup":
                if current_fsm in self._STANDING_STATES:
                    return {"info": "Robot is already standing", "fsm_id": current_fsm}
                if current_fsm in self._LOW_STATES:
                    steps = [("Start", 500, "start")]
                    return self._run_fsm_sequence(steps)
                if current_fsm in self._GROUND_STATES:
                    return {"error": f"Robot is on ground (FSM={current_fsm}). Use lie2standup instead."}
                return {"error": f"Cannot stand from FSM={current_fsm}"}

            elif mode in ("damp", "zero_torque"):
                if current_fsm in self._STANDING_STATES or current_fsm in self._LOW_STATES:
                    return {"error": f"Cannot enter {mode} from upright/low state (FSM={current_fsm}). "
                                     f"Robot will collapse. Use standup2lie first."}
                if current_fsm in self._UNSAFE_STATES:
                    return {"error": f"Cannot enter {mode} from unsafe state (FSM={current_fsm}). "
                                     f"Use emergency_stop first."}
                fn = self._client.ZeroTorque if mode == "zero_torque" else self._client.Damp
                ret = fn()
                return {"ret": ret, "mode": mode}

            else:
                return {"error": f"Unknown mode: {mode}. Available: lie2standup, standup2lie, "
                                 f"standup2squat, squat2standup, damp, zero_torque, "
                                 f"emergency_stop, get_current_mode"}
        elif action == "switch_mode_expert":
            fid = int(args.get("fsm_id", 0))
            ret = self._client.SetFsmId(fid)
            return {"ret": ret, "fsm_id": fid}
        elif action == "set_stand_height":
            h = max(0.0, min(1.0, float(args.get("height", 0.5))))
            ret = self._client.SetStandHeight(h)
            return {"ret": ret, "height": h}
        elif action == "get_fsm_id":
            code, fsm_id = self._client.GetFsmId()
            return {"ret": code, "fsm_id": fsm_id}
        elif action == "get_fsm_mode":
            code, fsm_mode = self._client.GetFsmMode()
            return {"ret": code, "fsm_mode": fsm_mode}
        elif action == "get_balance_mode":
            code, balance_mode = self._client.GetBalanceMode()
            return {"ret": code, "balance_mode": balance_mode}
        elif action == "get_swing_height":
            code, swing_height = self._client.GetSwingHeight()
            return {"ret": code, "swing_height": swing_height}
        elif action == "get_stand_height":
            code, stand_height = self._client.GetStandHeight()
            return {"ret": code, "stand_height": stand_height}
        elif action == "get_phase":
            code, phase = self._client.GetPhase()
            return {"ret": code, "phase": phase}
        elif action == "wave_hand":
            turn = bool(args.get("turn", False))
            ret  = self._client.WaveHand(turn)
            return {"ret": ret, "turn": turn}
        elif action == "shake_hand":
            ret = self._client.ShakeHand()
            return {"ret": ret}
        return None

    # ── FSM sequence helper ───────────────────────────────────────────────────

    def _run_fsm_sequence(self, steps: list) -> dict:
        """Execute FSM sequence in subprocess (no GIL contention).
        steps = [(method_name, target_fsm_id_to_poll, step_name), ...]"""
        result = self._client.RunFsmSequence(steps, interval=1.0, step_timeout=15.0)
        if result is None:
            return {"error": "RPC timeout during sequence execution"}
        return result


# ── FallRecoveryPlugin (sensor + actuator) ─────────────────────────────────────

class _FallRecoveryNode(Node):
    def __init__(self, namespace: str, imu_topic: str, foot_force_topic: str, events_topic: str, config: dict):
        super().__init__("g1_fall_recovery")
        self._namespace = namespace
        self._imu_topic = imu_topic
        self._foot_force_topic = foot_force_topic
        self._events_topic = events_topic
        self._fall_angle_threshold_deg = float(config.get("fall_angle_threshold_deg", 50))
        self._fall_confirm_duration = float(config.get("fall_confirm_duration", 0.5))
        self._foot_force_threshold = float(config.get("foot_force_threshold", 10))
        self._lock = threading.Lock()
        self._state = {
            "is_fallen": False,
            "fall_direction": "none",
            "confidence": 0.0,
            "imu_rpy": {"roll": 0.0, "pitch": 0.0, "yaw": 0.0},
            "foot_force": [],
            "last_fall_time": 0.0,
        }
        self._candidate_start = 0.0
        self._last_direction = "none"
        self._event_pub = self.create_publisher(String, events_topic, _LOW_LAT_QOS)

        try:
            self._imu_sub = self.create_subscription(String, imu_topic, self._on_imu, _LOW_LAT_QOS)
            print(f"[fall_recovery] IMU subscribed: {imu_topic}", flush=True)
        except Exception as e:
            print(f"[fall_recovery] IMU subscribe failed: {e}", flush=True)

        try:
            self._foot_force_sub = self.create_subscription(String, foot_force_topic, self._on_loco_state, _LOW_LAT_QOS)
            print(f"[fall_recovery] foot_force subscribed: {foot_force_topic}", flush=True)
        except Exception as e:
            print(f"[fall_recovery] foot_force subscribe failed: {e}", flush=True)

    def _on_imu(self, msg) -> None:
        try:
            data = json.loads(msg.data)
            rpy = data.get("rpy") or []
            if len(rpy) < 3:
                return
            roll, pitch, yaw = [math.degrees(float(v)) for v in rpy[:3]]
            with self._lock:
                self._state["imu_rpy"] = {
                    "roll": round(roll, 2),
                    "pitch": round(pitch, 2),
                    "yaw": round(yaw, 2),
                }
            self._update_detection()
        except Exception as e:
            print(f"[fall_recovery] IMU parse failed: {e}", flush=True)

    def _on_loco_state(self, msg) -> None:
        try:
            data = json.loads(msg.data)
            forces = data.get("foot_force") or []
            with self._lock:
                self._state["foot_force"] = [float(v) for v in forces]
            self._update_detection()
        except Exception as e:
            print(f"[fall_recovery] foot_force parse failed: {e}", flush=True)

    def _update_detection(self) -> None:
        now_mono = time.monotonic()
        event = None
        with self._lock:
            imu_rpy = dict(self._state["imu_rpy"])
            forces = list(self._state["foot_force"])
            roll = float(imu_rpy.get("roll", 0.0))
            pitch = float(imu_rpy.get("pitch", 0.0))
            direction = self._classify_direction(roll, pitch)
            angle_ok = direction != "none"
            foot_ok = len(forces) >= 4 and all(float(f) < self._foot_force_threshold for f in forces[:4])

            if angle_ok and foot_ok:
                if self._candidate_start == 0.0 or direction != self._last_direction:
                    self._candidate_start = now_mono
                    self._last_direction = direction
                confirmed = now_mono - self._candidate_start >= self._fall_confirm_duration
                confidence = self._confidence(roll, pitch, forces)
                if confirmed:
                    was_fallen = bool(self._state["is_fallen"])
                    self._state.update({
                        "is_fallen": True,
                        "fall_direction": direction,
                        "confidence": confidence,
                    })
                    if not was_fallen:
                        self._state["last_fall_time"] = time.time()
                        event = {"type": "fall_detected", "direction": direction, "confidence": confidence}
                else:
                    self._state["confidence"] = confidence
            else:
                self._candidate_start = 0.0
                self._last_direction = "none"
                self._state.update({
                    "is_fallen": False,
                    "fall_direction": "none",
                    "confidence": 0.0,
                })

        if event:
            self.publish_event(event["type"], {k: v for k, v in event.items() if k != "type"})

    def _classify_direction(self, roll: float, pitch: float) -> str:
        if max(abs(roll), abs(pitch)) < self._fall_angle_threshold_deg:
            return "none"
        if abs(pitch) >= abs(roll):
            return "prone" if pitch > 0 else "supine"
        return "right" if roll > 0 else "left"

    def _confidence(self, roll: float, pitch: float, forces: list) -> float:
        angle_score = min(1.0, max(abs(roll), abs(pitch)) / max(self._fall_angle_threshold_deg, 1.0))
        force_score = 1.0 if len(forces) >= 4 and all(float(f) < self._foot_force_threshold for f in forces[:4]) else 0.0
        return round(min(1.0, 0.7 * angle_score + 0.3 * force_score), 3)

    def get_status(self) -> dict:
        with self._lock:
            return {
                "is_fallen": bool(self._state["is_fallen"]),
                "fall_direction": self._state["fall_direction"],
                "confidence": float(self._state["confidence"]),
                "imu_rpy": dict(self._state["imu_rpy"]),
                "foot_force": list(self._state["foot_force"]),
                "last_fall_time": float(self._state["last_fall_time"]),
            }

    def publish_event(self, event_type: str, data: dict | None = None) -> None:
        try:
            event = {"type": event_type, "timestamp": time.time()}
            if data:
                event.update(data)
            msg = String()
            msg.data = json.dumps(event)
            self._event_pub.publish(msg)
            print(f"[fall_recovery] event: {event_type} | {json.dumps(data or {})}", flush=True)
        except Exception as e:
            print(f"[fall_recovery] event publish failed: {e}", flush=True)


class FallRecoveryPlugin:
    PREFIX = "fall_recovery"

    def __init__(self, plugin_config: dict, namespace: str, executor, loco_plugin=None):
        self._namespace = namespace
        self._config = plugin_config
        self._loco_plugin = loco_plugin
        self._recovery_timeout = float(plugin_config.get("recovery_timeout", 10))
        self._imu_topic = self._resolve_topic(namespace, plugin_config.get("imu_topic", "state/imu"))
        self._foot_force_topic = self._resolve_topic(namespace, plugin_config.get("foot_force_topic", "loco/state"))
        self._events_topic = f"/{namespace}/fall_recovery/events"
        self._node = _FallRecoveryNode(namespace, self._imu_topic, self._foot_force_topic, self._events_topic, plugin_config)
        executor.add_node(self._node)

    def _resolve_topic(self, namespace: str, topic: str) -> str:
        topic = str(topic).strip()
        if topic.startswith("/"):
            return topic
        return f"/{namespace}/{topic.lstrip('/')}"

    def get_tools(self) -> list:
        return [self._status_tool(), self._recover_tool(), self._fall_event_tool()]

    def _status_tool(self) -> dict:
        return {
            "name": "fall_status",
            "type": "sensor",
            "multiInstance": False,
            "description": "G1 fall detection status — fallen state, posture direction, confidence, IMU RPY and foot force.",
            "inputSchema": {"type": "object", "properties": {}},
        }

    def _recover_tool(self) -> dict:
        return {
            "name": "fall_recover",
            "type": "actuator",
            "multiInstance": False,
            "description": "G1 one-shot fall recovery. Uses existing safe switch_mode lie2standup sequence.",
            "inputSchema": {
                "type": "object",
                "properties": {
                    "direction": {
                        "type": "string",
                        "enum": ["auto", "prone", "supine", "left", "right"],
                        "description": "Fall direction. auto uses current detection result.",
                        "default": "auto",
                    },
                    "force": {
                        "type": "boolean",
                        "description": "Execute recovery even if no fall is detected.",
                        "default": False,
                    },
                },
            },
        }

    def _fall_event_tool(self) -> dict:
        return {
            "name": "fall_events",
            "type": "sensor",
            "multiInstance": False,
            "description": f"G1 fall recovery events — fall_detected, recovery_started, recovery_completed, recovery_failed. Publishes to {self._events_topic}",
            "inputSchema": {"type": "object", "properties": {}},
            "topic_out": [{"topic": self._events_topic, "format": "data/json"}],
        }

    def start(self) -> None:
        pass

    def stop(self) -> None:
        pass

    def dispatch(self, action: str, args: dict) -> dict | None:
        if action == "start":
            return {"state": "running"}
        if action == "stop":
            return {"state": "idle"}
        if action == "info":
            tool_name = args.get("_tool_name", "fall_status")
            if tool_name == "fall_events":
                return {"state": "running", "topic_out": [{"topic": self._events_topic, "format": "data/json"}]}
            return self._node.get_status()
        if action == "fall_status":
            return self._node.get_status()
        if action == "fall_recover":
            return self._recover(args)
        return None

    def _recover(self, args: dict) -> dict:
        status = self._node.get_status()
        direction_arg = str(args.get("direction", "auto"))
        force = bool(args.get("force", False))
        direction = status["fall_direction"] if direction_arg == "auto" else direction_arg

        if direction not in ("prone", "supine", "left", "right"):
            direction = "none"
        if not force and not status["is_fallen"]:
            return {"success": False, "state": "not_fallen", "status": status, "error": "Robot is not detected as fallen. Set force=true to recover anyway."}
        if not self._loco_plugin:
            self._node.publish_event("recovery_failed", {"reason": "missing_loco_plugin", "direction": direction})
            return {"success": False, "state": "failed", "direction": direction, "error": "LocoPlugin is unavailable"}

        try:
            self._node.publish_event("recovery_started", {"direction": direction, "forced": force})
            result = self._loco_plugin.dispatch("switch_mode", {"mode": "lie2standup"})
            if isinstance(result, dict) and "error" in result:
                self._node.publish_event("recovery_failed", {"direction": direction, "error": result["error"]})
                return {"success": False, "state": "failed", "direction": direction, "progress": "lie2standup", "result": result, "timeout": self._recovery_timeout}
            self._node.publish_event("recovery_completed", {"direction": direction})
            return {"success": True, "state": "completed", "direction": direction, "progress": "lie2standup", "result": result, "timeout": self._recovery_timeout}
        except Exception as e:
            print(f"[fall_recovery] recovery failed: {e}", flush=True)
            self._node.publish_event("recovery_failed", {"direction": direction, "error": str(e)})
            return {"success": False, "state": "failed", "direction": direction, "error": str(e)}


class ContinuousGaitPlugin:
    PREFIX = "continuous_gait"

    def __init__(self, plugin_config: dict, namespace: str, executor, loco_client):
        self._client = loco_client

    def get_tool(self) -> dict:
        return {
            "name": "continuous_gait",
            "type": "actuator",
            "multiInstance": False,
            "description": "G1 continuous gait mode — enable/disable continuous gait, query balance mode",
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {
                        "type": "string",
                        "enum": ["enable", "disable", "get_state"],
                        "description": "Action to perform",
                    },
                },
                "required": ["action"],
                "x-action-params": {
                    "enable":    {"params": [], "description": "Enable continuous gait mode (balance mode 1)"},
                    "disable":   {"params": [], "description": "Disable continuous gait mode (balance mode 0)"},
                    "get_state": {"params": [], "description": "Get current balance mode (0=standard gait, 1=continuous gait)"},
                },
            },
        }

    def start(self) -> None:
        pass

    def stop(self) -> None:
        pass

    def dispatch(self, action: str, args: dict) -> dict | None:
        if action == "start":
            return {"state": "ready"}
        if action == "stop":
            return {"state": "idle"}
        if action == "enable":
            ret = self._client.ContinuousGait(True)
            return {"ret": ret, "continuous_gait": True}
        elif action == "disable":
            ret = self._client.ContinuousGait(False)
            return {"ret": ret, "continuous_gait": False}
        elif action == "get_state":
            code, mode = self._client.GetBalanceMode()
            return {"ret": code, "balance_mode": mode,
                    "description": "standard gait" if mode == 0 else "continuous gait" if mode == 1 else f"unknown mode {mode}"}
        return None


# ── BmsControlPlugin (actuator) ───────────────────────────────────────────────

class BmsControlPlugin:
    PREFIX = "bms_control"

    def __init__(self, plugin_config: dict, namespace: str, executor):
        from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber
        from unitree_sdk2py.idl.unitree_hg.msg.dds_ import BmsCmd_, BmsState_

        self._bms_cmd_type = BmsCmd_
        self._command_topic = plugin_config.get("command_topic", "rt/lf/bmscmd")
        self._last_state: dict | None = None
        self._lock = threading.Lock()

        self._publisher = ChannelPublisher(self._command_topic, BmsCmd_)
        self._publisher.Init()
        self._subscriber = ChannelSubscriber("rt/lf/bmsstate", BmsState_)
        self._subscriber.Init(self._on_state, 10)

    def get_tool(self) -> dict:
        return {
            "name": "bms_control",
            "type": "actuator",
            "multiInstance": False,
            "description": "G1 BMS control — query battery state or send a raw BMS command byte",
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {
                        "type": "string",
                        "enum": ["get_state", "get_soc", "get_soh", "get_voltage", "get_current",
                                 "get_temperatures", "get_cell_voltages", "get_cycle_count",
                                 "get_health_summary", "send_command"],
                        "description": "Action to perform",
                    },
                    "cmd": {
                        "type": "integer",
                        "minimum": 0,
                        "maximum": 255,
                        "description": "Raw BMS command byte (0-255)",
                    },
                },
                "required": ["action"],
                "x-action-params": {
                    "get_state":          {"params": [],      "description": "Get full BMS state"},
                    "get_soc":            {"params": [],      "description": "Get state of charge (%)"},
                    "get_soh":            {"params": [],      "description": "Get state of health (%)"},
                    "get_voltage":        {"params": [],      "description": "Get total battery voltage (V)"},
                    "get_current":        {"params": [],      "description": "Get current (A) and direction"},
                    "get_temperatures":   {"params": [],      "description": "Get all temperature sensors"},
                    "get_cell_voltages":  {"params": [],      "description": "Get all cell voltages (V)"},
                    "get_cycle_count":    {"params": [],      "description": "Get charge cycle count"},
                    "get_health_summary": {"params": [],      "description": "Get battery health summary"},
                    "send_command":       {"params": ["cmd"], "description": "Send raw BMS command byte"},
                },
            },
        }

    def start(self) -> None:
        pass

    def stop(self) -> None:
        pass

    def _on_state(self, msg) -> None:
        state = {
            "version_high":     int(msg.version_high),
            "version_low":      int(msg.version_low),
            "fn":               int(msg.fn),
            "soc":              int(msg.soc),
            "soh":              int(msg.soh),
            "current":          int(msg.current),
            "voltage":          [int(v) for v in msg.bmsvoltage if v > 0],
            "cell_vol":         [int(v) for v in msg.cell_vol if v > 0],
            "temperature":      [int(t) for t in msg.temperature if t > 0],
            "cycle":            int(msg.cycle),
            "manufacturer_date": int(msg.manufacturer_date),
            "bmsstate":         [int(v) for v in msg.bmsstate],
        }
        with self._lock:
            self._last_state = state

    def dispatch(self, action: str, args: dict) -> dict | None:
        if action == "start":
            return {"state": "ready"}
        if action == "stop":
            return {"state": "idle"}
        if action.startswith("get_"):
            with self._lock:
                state = dict(self._last_state) if self._last_state is not None else None
            if state is None:
                return {"error": "BMS state unavailable: no rt/lf/bmsstate message received"}

            if action == "get_state":
                return state
            if action == "get_soc":
                return {"soc": state["soc"], "unit": "%"}
            if action == "get_soh":
                return {"soh": state["soh"], "unit": "%"}
            if action == "get_voltage":
                voltage = state["voltage"][0] / 1000.0 if state["voltage"] else 0.0
                return {"voltage": voltage, "unit": "V"}
            if action == "get_current":
                current = state["current"] / 1000.0
                direction = "charging" if current < 0 else "discharging" if current > 0 else "idle"
                return {"current": current, "unit": "A", "direction": direction}
            if action == "get_temperatures":
                temperatures = state["temperature"]
                return {
                    "temperatures": temperatures,
                    "count": len(temperatures),
                    "max": max(temperatures) if temperatures else 0,
                    "min": min(temperatures) if temperatures else 0,
                    "avg": sum(temperatures) / len(temperatures) if temperatures else 0.0,
                    "unit": "°C",
                }
            if action == "get_cell_voltages":
                voltages = [value / 1000.0 for value in state["cell_vol"]]
                maximum = max(voltages) if voltages else 0.0
                minimum = min(voltages) if voltages else 0.0
                return {
                    "voltages": voltages,
                    "count": len(voltages),
                    "max": maximum,
                    "min": minimum,
                    "delta": maximum - minimum,
                    "unit": "V",
                }
            if action == "get_cycle_count":
                return {"cycle": state["cycle"], "unit": "次"}
            if action == "get_health_summary":
                soc = state["soc"]
                soh = state["soh"]
                temperatures = state["temperature"]
                status = "critical" if soc <= 10 or soh <= 60 else "warning" if soc <= 20 or soh <= 80 else "good"
                return {
                    "soc": soc,
                    "soh": soh,
                    "voltage": state["voltage"][0] / 1000.0 if state["voltage"] else 0.0,
                    "current": state["current"] / 1000.0,
                    "temperature_avg": sum(temperatures) / len(temperatures) if temperatures else 0.0,
                    "cycle_count": state["cycle"],
                    "status": status,
                }
        if action == "send_command":
            cmd = args.get("cmd")
            if isinstance(cmd, bool) or not isinstance(cmd, int) or not 0 <= cmd <= 255:
                return {"error": "cmd must be an integer between 0 and 255"}
            message = self._bms_cmd_type(cmd=cmd, reserve=[0] * 40)
            published = self._publisher.Write(message)
            return {"published": published, "cmd": cmd, "command_topic": self._command_topic}
        return None


# ── AsrPlugin (sensor) ───────────────────────────────────────────────────────

class _AsrNode(Node):
    """Subscribes to DDS rt/audio_msg (String_) and republishes ASR results to ROS2."""

    def __init__(self, topic: str):
        super().__init__("g1_asr")
        self._topic = topic
        self._pub = self.create_publisher(String, topic, _LOW_LAT_QOS)
        self._last_index: int = -1

        try:
            from unitree_sdk2py.core.channel import ChannelSubscriber
            from unitree_sdk2py.idl.std_msgs.msg.dds_ import String_
            sub = ChannelSubscriber("rt/audio_msg", String_)
            sub.Init(self._on_msg, 10)
            self.get_logger().info(f"AsrNode subscribed rt/audio_msg → {topic}")
        except Exception as e:
            self.get_logger().warn(f"AsrNode: failed to subscribe rt/audio_msg: {e}")

    def _on_msg(self, msg) -> None:
        try:
            payload = json.loads(msg.data_)
        except (json.JSONDecodeError, AttributeError):
            return
        # Deduplicate by index
        idx = payload.get("index", -1)
        if idx == self._last_index:
            return
        self._last_index = idx

        out = String()
        out.data = json.dumps(payload)
        self._pub.publish(out)


class AsrPlugin:
    PREFIX = "asr"

    def __init__(self, plugin_config: dict, namespace: str, executor):
        self._topic = f"/{namespace}/asr/text"
        self._node = _AsrNode(self._topic)
        executor.add_node(self._node)

    def get_tool(self) -> dict:
        return {
            "name": "asr",
            "type": "sensor",
            "multiInstance": False,
            "description": (
                "G1 built-in ASR — offline speech recognition results "
                "(text, angle, confidence, emotion). "
                f"Publishes to {self._topic}"
            ),
            "inputSchema": {"type": "object", "properties": {}},
            "topic_out": [{"topic": self._topic, "format": "data/json"}],
        }

    def start(self) -> None:
        pass  # Passive DDS subscription, started in __init__

    def stop(self) -> None:
        pass

    def dispatch(self, action: str, args: dict) -> dict | None:
        if action == "start":
            return {"state": "running"}
        if action == "stop":
            return {"state": "idle"}
        if action == "info":
            return {"state": "running", "topic_out": [{"topic": self._topic, "format": "data/json"}]}
        return None


# ── ArmActionPlugin (actuator) ───────────────────────────────────────────────

_ARM_ACTION_MAP = {
    "release arm":    99,
    "two-hand kiss":  11,
    "left kiss":      12,
    "right kiss":     13,
    "hands up":       15,
    "clap":           17,
    "high five":      18,
    "hug":            19,
    "heart":          20,
    "right heart":    21,
    "reject":         22,
    "right hand up":  23,
    "x-ray":          24,
    "face wave":      25,
    "high wave":      26,
    "shake hand":     27,
}
_ARM_ID_MAP = {v: k for k, v in _ARM_ACTION_MAP.items()}


class ArmActionPlugin:
    PREFIX = "arm"

    def __init__(self, plugin_config: dict, namespace: str, executor, arm_client):
        self._client = arm_client

    def get_tool(self) -> dict:
        return {
            "name": "arm",
            "type": "actuator",
            "multiInstance": False,
            "description": f"G1 arm gestures — execute predefined actions. Available: {', '.join(_ARM_ACTION_MAP)}",
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {
                        "type": "string",
                        "enum": ["execute", "release", "list"],
                        "description": "Action to perform",
                    },
                    "gesture":    {"type": "string",  "description": f"Gesture name: {', '.join(_ARM_ACTION_MAP)}"},
                    "action_id":  {"type": "integer", "description": "Gesture ID (alternative to gesture name)"},
                },
                "required": ["action"],
                "x-action-params": {
                    "execute": {"params": ["gesture", "action_id"], "description": "Execute a predefined arm gesture by name or ID"},
                    "release": {"params": [],                       "description": "Release arm to relaxed state"},
                    "list":    {"params": [],                       "description": "List all available arm gestures with IDs"},
                },
            },
        }

    def start(self) -> None:
        pass

    def stop(self) -> None:
        pass

    def dispatch(self, action: str, args: dict) -> dict | None:
        if action == "start":
            return {"state": "ready"}
        if action == "stop":
            return {"state": "idle"}
        if action == "list":
            return {"actions": [{"id": v, "name": k} for k, v in _ARM_ACTION_MAP.items()]}
        elif action == "execute":
            action_id = None
            if "action_id" in args:
                action_id = int(args["action_id"])
            elif "gesture" in args:
                action_id = _ARM_ACTION_MAP.get(args["gesture"].lower().strip())
                if action_id is None:
                    return {"error": f"Unknown gesture: {args['gesture']}. Available: {list(_ARM_ACTION_MAP)}"}
            else:
                return {"error": "Provide 'gesture' name or 'action_id'"}
            ret = self._client.ExecuteAction(action_id)
            return {"ret": ret, "action_id": action_id, "gesture": _ARM_ID_MAP.get(action_id, "unknown")}
        elif action == "release":
            ret = self._client.ExecuteAction(99)
            return {"ret": ret, "action_id": 99, "gesture": "release arm"}
        return None


# ── StatePlugin (sensor) ─────────────────────────────────────────────────────

class _LowStateNode(Node):
    """Subscribes to DDS rt/lowstate + rt/lf/bmsstate and republishes to ROS2."""

    _JOINTS_INTERVAL = 0.1   # 10 Hz throttle for joints
    _IMU_INTERVAL    = 0.05  # 20 Hz throttle for IMU
    _BMS_INTERVAL    = 1.0   # 1 Hz throttle for BMS
    _MAINBOARD_INTERVAL = 2.0  # 0.5 Hz throttle for mainboard

    def __init__(self, imu_topic: str, battery_topic: str, joints_topic: str, mainboard_topic: str):
        super().__init__("g1_low_state")
        self._imu_pub       = self.create_publisher(String, imu_topic,       _LOW_LAT_QOS)
        self._battery_pub   = self.create_publisher(String, battery_topic,   _LOW_LAT_QOS)
        self._joints_pub    = self.create_publisher(String, joints_topic,    _LOW_LAT_QOS)
        self._mainboard_pub = self.create_publisher(String, mainboard_topic, _LOW_LAT_QOS)
        self._last_imu:     dict = {}
        self._last_battery: dict = {}
        self._lock = threading.Lock()
        self._last_joints_time:    float = 0.0
        self._last_imu_time:       float = 0.0
        self._last_bms_time:       float = 0.0
        self._last_mainboard_time: float = 0.0

        try:
            from unitree_sdk2py.core.channel import ChannelSubscriber
            from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_
            sub = ChannelSubscriber("rt/lowstate", LowState_)
            sub.Init(self._on_state, 10)
            self.get_logger().info(f"LowStateNode subscribed rt/lowstate → {imu_topic}, {joints_topic}")
        except Exception as e:
            self.get_logger().warn(f"LowStateNode: failed to subscribe rt/lowstate: {e}")

        try:
            from unitree_sdk2py.core.channel import ChannelSubscriber
            from unitree_sdk2py.idl.unitree_hg.msg.dds_ import BmsState_
            bms_sub = ChannelSubscriber("rt/lf/bmsstate", BmsState_)
            bms_sub.Init(self._on_bms, 10)
            self.get_logger().info(f"LowStateNode subscribed rt/lf/bmsstate → {battery_topic}")
        except Exception as e:
            self.get_logger().warn(f"LowStateNode: failed to subscribe rt/lf/bmsstate: {e}")

        try:
            from unitree_sdk2py.core.channel import ChannelSubscriber
            from unitree_sdk2py.idl.unitree_hg.msg.dds_ import MainBoardState_
            mb_sub = ChannelSubscriber("rt/lf/mainboardstate", MainBoardState_)
            mb_sub.Init(self._on_mainboard, 10)
            self.get_logger().info(f"LowStateNode subscribed rt/lf/mainboardstate → {mainboard_topic}")
        except Exception as e:
            self.get_logger().warn(f"LowStateNode: failed to subscribe rt/lf/mainboardstate: {e}")

    def _on_state(self, msg) -> None:
        now = time.monotonic()

        # IMU: throttle to 20 Hz
        if now - self._last_imu_time >= self._IMU_INTERVAL:
            self._last_imu_time = now
            imu = msg.imu_state
            imu_data = {
                "quaternion":    list(imu.quaternion),
                "gyroscope":     list(imu.gyroscope),
                "accelerometer": list(imu.accelerometer),
                "rpy":           list(imu.rpy),
                "temperature":   float(imu.temperature),
            }
            with self._lock:
                self._last_imu = imu_data

            imu_out = String()
            imu_out.data = json.dumps(imu_data)
            self._imu_pub.publish(imu_out)

        # Joints: throttle to 10 Hz
        now = time.monotonic()
        if now - self._last_joints_time >= self._JOINTS_INTERVAL:
            self._last_joints_time = now
            joints = []
            for i, m in enumerate(msg.motor_state):
                joints.append({
                    "idx": i,
                    "q": round(float(m.q), 4),
                    "dq": round(float(m.dq), 4),
                    "tau": round(float(m.tau_est), 3),
                    "temp": list(m.temperature),
                })
            joints_out = String()
            joints_out.data = json.dumps({"joints": joints, "imu_quat": list(msg.imu_state.quaternion)})
            self._joints_pub.publish(joints_out)

    def _on_bms(self, msg) -> None:
        now = time.monotonic()
        if now - self._last_bms_time < self._BMS_INTERVAL:
            return
        self._last_bms_time = now

        bms_data = {
            "soc":         int(msg.soc),
            "soh":         int(msg.soh),
            "current":     int(msg.current),
            "voltage":     [int(v) for v in msg.bmsvoltage if v > 0],
            "cell_vol":    [int(v) for v in msg.cell_vol if v > 0],
            "temperature": [int(t) for t in msg.temperature if t > 0],
            "cycle":       int(msg.cycle),
        }
        with self._lock:
            self._last_battery = bms_data

        bat_out = String()
        bat_out.data = json.dumps(bms_data)
        self._battery_pub.publish(bat_out)

    def _on_mainboard(self, msg) -> None:
        now = time.monotonic()
        if now - self._last_mainboard_time < self._MAINBOARD_INTERVAL:
            return
        self._last_mainboard_time = now

        mb_data = {
            "temperature": [int(t) for t in msg.temperature if t > 0],
            "fan_state":   [int(f) for f in msg.fan_state],
            "value":       [round(float(v), 2) for v in msg.value if v != 0.0],
            "state":       [int(s) for s in msg.state if s > 0],
        }
        mb_out = String()
        mb_out.data = json.dumps(mb_data)
        self._mainboard_pub.publish(mb_out)


class StatePlugin:
    PREFIX = "state"

    def __init__(self, plugin_config: dict, namespace: str, executor):
        self._imu_topic       = f"/{namespace}/state/imu"
        self._battery_topic   = f"/{namespace}/state/battery"
        self._joints_topic    = f"/{namespace}/state/joints"
        self._mainboard_topic = f"/{namespace}/state/mainboard"
        self._node = _LowStateNode(self._imu_topic, self._battery_topic, self._joints_topic, self._mainboard_topic)
        executor.add_node(self._node)

    def get_tools(self) -> list:
        return [self._imu_tool(), self._battery_tool(), self._joints_tool(), self._mainboard_tool(), self._model_tool()]

    def _imu_tool(self) -> dict:
        return {
            "name": "imu",
            "type": "sensor",
            "multiInstance": False,
            "description": f"G1 IMU sensor — quaternion, gyroscope, accelerometer, rpy, temperature. Publishes to {self._imu_topic}",
            "inputSchema": {"type": "object", "properties": {}},
            "topic_out": [{"topic": self._imu_topic, "format": "data/json"}],
        }

    def _battery_tool(self) -> dict:
        return {
            "name": "battery",
            "type": "sensor",
            "multiInstance": False,
            "description": f"G1 BMS battery — SOC%, SOH%, current(mA), voltage, cell voltages, temperature, charge cycles. Publishes at 1Hz to {self._battery_topic}",
            "inputSchema": {"type": "object", "properties": {}},
            "topic_out": [{"topic": self._battery_topic, "format": "data/json"}],
        }

    def _joints_tool(self) -> dict:
        return {
            "name": "joints",
            "type": "sensor",
            "multiInstance": False,
            "description": f"G1 joint states — 35 motors with position(q), velocity(dq), torque(tau), temperature. Publishes at 10Hz to {self._joints_topic}",
            "inputSchema": {"type": "object", "properties": {}},
            "topic_out": [{"topic": self._joints_topic, "format": "sensor/skeleton"}],
        }

    def _mainboard_tool(self) -> dict:
        return {
            "name": "mainboard",
            "type": "sensor",
            "multiInstance": False,
            "description": f"G1 mainboard state — temperature, fan state, system values. Publishes at 0.5Hz to {self._mainboard_topic}",
            "inputSchema": {"type": "object", "properties": {}},
            "topic_out": [{"topic": self._mainboard_topic, "format": "data/json"}],
        }

    def _model_tool(self) -> dict:
        return {
            "name": "model",
            "type": "resource",
            "multiInstance": False,
            "description": "G1 robot URDF model for 3D visualization — kinematic chain with joint origins, axes, and limits",
            "inputSchema": {"type": "object", "properties": {}},
        }

    def start(self) -> None:
        pass  # DDS subscription starts in __init__

    def stop(self) -> None:
        pass

    def dispatch(self, action: str, args: dict) -> dict | None:
        if action == "start":
            return {"state": "running"}
        if action == "stop":
            return {"state": "idle"}
        if action == "info":
            tool_name = args.get('_tool_name', '')
            topic_map = {
                'imu':       (self._imu_topic,      'data/json'),
                'battery':   (self._battery_topic,  'data/json'),
                'joints':    (self._joints_topic,   'sensor/skeleton'),
                'mainboard': (self._mainboard_topic,'data/json'),
            }
            if tool_name in topic_map:
                topic, fmt = topic_map[tool_name]
                return {"state": "running", "topic_out": [{"topic": topic, "format": fmt}]}
            return {"state": "running"}
        if action == "model":
            from pathlib import Path
            urdf_path = Path(__file__).parent / "resource" / "g1_model.urdf"
            if urdf_path.exists():
                return {"urdf": urdf_path.read_text()}
            return {"error": "URDF model file not found"}
        return None


# ── GripperStatePlugin (sensor) ──────────────────────────────────────────────

class _GripperStateNode(Node):
    """Subscribes to left/right gripper DDS state and republishes JSON to ROS2."""

    def __init__(self, topic: str, plugin_config: dict):
        super().__init__("g1_gripper_state")
        self._publisher = self.create_publisher(String, topic, _LOW_LAT_QOS)
        self._min_angle = float(plugin_config.get("min_angle_rad", 0.0))
        self._max_angle = float(plugin_config.get("max_angle_rad", 1.5))
        self._grasp_tau_threshold = float(plugin_config.get("grasp_tau_threshold", 0.1))
        self._publish_interval = 1.0 / float(plugin_config.get("publish_hz", 20))
        self._last_publish_time = {"left": 0.0, "right": 0.0}
        self._subscribers = []

        from unitree_sdk2py.core.channel import ChannelSubscriber
        from unitree_sdk2py.idl.unitree_hg.msg.dds_ import HandState_

        for side, config_key, default_topic in (
            ("left", "left_dds_topic", "rt/hand/left_state"),
            ("right", "right_dds_topic", "rt/hand/right_state"),
        ):
            dds_topic = plugin_config.get(config_key, default_topic)
            try:
                subscriber = ChannelSubscriber(dds_topic, HandState_)
                subscriber.Init(lambda msg, side=side: self._on_state(side, msg), 10)
                self._subscribers.append(subscriber)
                self.get_logger().info(f"GripperStateNode subscribed {dds_topic} → {topic}")
            except Exception as e:
                self.get_logger().warn(f"GripperStateNode: failed to subscribe {dds_topic}: {e}")

    def _on_state(self, side: str, msg) -> None:
        now = time.monotonic()
        if now - self._last_publish_time[side] < self._publish_interval:
            return
        self._last_publish_time[side] = now

        # TODO: Confirm motor index 0 and whether larger angles mean a more closed gripper.
        motor = msg.motor_state[0]
        position_rad = float(motor.q)
        position_pct = (position_rad - self._min_angle) / (self._max_angle - self._min_angle) * 100.0
        position_pct = max(0.0, min(100.0, position_pct))
        tau_est = float(motor.tau_est)
        data = {
            "side": side,
            "position_pct": round(position_pct, 2),
            "position_rad": position_rad,
            "tau_est": tau_est,
            "temperature": [int(value) for value in motor.temperature],
            "object_grasped": abs(tau_est) >= self._grasp_tau_threshold,
            "error_code": [int(value) for value in msg.error],
            "power_v": float(msg.power_v),
            "power_a": float(msg.power_a),
        }
        output = String()
        output.data = json.dumps(data)
        self._publisher.publish(output)


class GripperStatePlugin:
    PREFIX = "gripper_state"

    def __init__(self, plugin_config: dict, namespace: str, executor):
        self._topic = f"/{namespace}/gripper/state"
        self._node = _GripperStateNode(self._topic, plugin_config)
        executor.add_node(self._node)

    def get_tool(self) -> dict:
        return {
            "name": "gripper_state",
            "type": "sensor",
            "multiInstance": False,
            "description": f"G1 gripper state — position, torque, temperature, grasp detection, errors, and power. Publishes to {self._topic}",
            "inputSchema": {"type": "object", "properties": {}},
            "topic_out": [{"topic": self._topic, "format": "data/json"}],
        }

    def start(self) -> None:
        pass  # DDS subscriptions start in __init__

    def stop(self) -> None:
        pass

    def dispatch(self, action: str, args: dict) -> dict | None:
        if action == "start":
            return {"state": "running"}
        if action == "stop":
            return {"state": "idle"}
        if action == "info":
            return {"state": "running", "topic_out": [{"topic": self._topic, "format": "data/json"}]}
        return None


# ── GripperControlPlugin (actuator) ──────────────────────────────────────────

class GripperControlPlugin:
    PREFIX = "gripper_control"

    def __init__(self, plugin_config: dict, namespace: str, executor):
        self._dds_topics = {
            "left": plugin_config.get("left_dds_topic", "rt/hand/left_cmd"),
            "right": plugin_config.get("right_dds_topic", "rt/hand/right_cmd"),
        }
        self._min_angle = float(plugin_config.get("min_angle_rad", 0.0))
        self._max_angle = float(plugin_config.get("max_angle_rad", 1.5))
        self._motor_mode = int(plugin_config.get("motor_mode", 10))
        self._default_kp = float(plugin_config.get("default_kp", 20.0))
        self._default_kd = float(plugin_config.get("default_kd", 1.0))
        self._force_pct = float(plugin_config.get("default_force_pct", 50))
        self._max_tau = float(plugin_config.get("max_tau", 0.5))
        self._publishers = {}
        self._last_position_pct = {"left": 0.0, "right": 0.0}

    def get_tool(self) -> dict:
        return {
            "name": "gripper_control",
            "type": "actuator",
            "multiInstance": False,
            "description": "G1 parallel gripper control — open, close, set position or force, and stop",
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {
                        "type": "string",
                        "enum": ["open", "close", "set_position", "set_force", "stop"],
                        "description": "Action to perform",
                    },
                    "side": {
                        "type": "string",
                        "enum": ["left", "right", "both"],
                        "default": "both",
                        "description": "Gripper side to control",
                    },
                    "position_pct": {
                        "type": "number",
                        "minimum": 0,
                        "maximum": 100,
                        "description": "Opening position, where 0% is open and 100% is closed",
                    },
                    "force_pct": {
                        "type": "number",
                        "minimum": 0,
                        "maximum": 100,
                        "description": "Gripping force limit percentage",
                    },
                },
                "required": ["action"],
                "x-action-params": {
                    "open": {"params": ["side"], "description": "Fully open the selected gripper"},
                    "close": {"params": ["side", "force_pct"], "description": "Close the selected gripper, optionally setting the force limit"},
                    "set_position": {"params": ["side", "position_pct"], "description": "Set gripper position from 0% open to 100% closed"},
                    "set_force": {"params": ["side", "force_pct"], "description": "Set the gripping force limit"},
                    "stop": {"params": ["side"], "description": "Stop by holding the last commanded position"},
                },
            },
        }

    def start(self) -> None:
        pass  # DDS publishers are initialized lazily on the first action

    def stop(self) -> None:
        pass

    def _get_publisher(self, side: str):
        if side not in self._publishers:
            from unitree_sdk2py.core.channel import ChannelPublisher
            from unitree_sdk2py.idl.unitree_hg.msg.dds_ import HandCmd_
            publisher = ChannelPublisher(self._dds_topics[side], HandCmd_)
            publisher.Init()
            self._publishers[side] = publisher
        return self._publishers[side]

    def _send_position(self, side: str, position_pct: float) -> dict:
        from unitree_sdk2py.idl.default import unitree_hg_msg_dds__HandCmd_

        # TODO: Confirm motor index 0, mode 10, and whether larger angles mean a more closed gripper.
        position_rad = self._min_angle + position_pct / 100.0 * (self._max_angle - self._min_angle)
        message = unitree_hg_msg_dds__HandCmd_()
        motor = message.motor_cmd[0]
        motor.mode = self._motor_mode
        motor.q = position_rad
        motor.dq = 0.0
        motor.tau = 0.0
        motor.kp = self._default_kp
        motor.kd = self._default_kd
        published = self._get_publisher(side).Write(message)
        self._last_position_pct[side] = position_pct
        return {
            "side": side,
            "position_pct": position_pct,
            "position_rad": position_rad,
            "published": bool(published),
        }

    def dispatch(self, action: str, args: dict) -> dict | None:
        if action == "start":
            return {"state": "ready"}
        side = args.get("side", "both")
        if side not in ("left", "right", "both"):
            return {"error": "side must be left, right, or both"}
        sides = ("left", "right") if side == "both" else (side,)

        if action == "set_force":
            if "force_pct" not in args:
                return {"error": "force_pct is required"}
            force_pct = float(args["force_pct"])
            if not 0.0 <= force_pct <= 100.0:
                return {"error": "force_pct must be between 0 and 100"}
            self._force_pct = force_pct
            # TODO: Apply the force limit once the HandCmd_ force-control semantics are confirmed on hardware.
            return {"side": side, "force_pct": force_pct, "max_tau": self._max_tau, "applied": False}

        if action == "open":
            position_pct = 0.0
        elif action == "close":
            if "force_pct" in args:
                force_pct = float(args["force_pct"])
                if not 0.0 <= force_pct <= 100.0:
                    return {"error": "force_pct must be between 0 and 100"}
                self._force_pct = force_pct
            position_pct = 100.0
        elif action == "set_position":
            if "position_pct" not in args:
                return {"error": "position_pct is required"}
            position_pct = float(args["position_pct"])
            if not 0.0 <= position_pct <= 100.0:
                return {"error": "position_pct must be between 0 and 100"}
        elif action == "stop":
            # TODO: Replace last-target hold with measured-position hold if hardware testing requires it.
            return {"results": [self._send_position(item, self._last_position_pct[item]) for item in sides]}
        else:
            return None

        results = [self._send_position(item, position_pct) for item in sides]
        return {"results": results, "force_pct": self._force_pct}


# ── HeadControlPlugin (actuator) ─────────────────────────────────────────────

class HeadControlPlugin:
    PREFIX = "head_control"

    def __init__(self, plugin_config: dict, namespace: str, executor):
        self._motor_index = int(plugin_config.get("motor_index", 23))
        self._min_angle_rad = math.radians(float(plugin_config.get("min_angle_deg", -90.0)))
        self._max_angle_rad = math.radians(float(plugin_config.get("max_angle_deg", 90.0)))
        self._motor_mode = int(plugin_config.get("motor_mode", 10))
        self._default_kp = float(plugin_config.get("default_kp", 20.0))
        self._default_kd = float(plugin_config.get("default_kd", 1.0))
        self._lowcmd_topic = plugin_config.get("lowcmd_topic", "rt/lowcmd")
        self._lowstate_topic = plugin_config.get("lowstate_topic", "rt/lowstate")
        self._shake_amplitude_deg = float(plugin_config.get("shake_amplitude_deg", 30.0))
        self._shake_speed_hz = float(plugin_config.get("shake_speed_hz", 1.5))
        self._shake_times = int(plugin_config.get("shake_times", 3))
        self._publisher = None
        self._animation_stop = threading.Event()
        self._animation_thread = None
        self._state_lock = threading.Lock()
        self._current_angle_rad = None
        self._last_commanded_angle_rad = 0.0

        try:
            from unitree_sdk2py.core.channel import ChannelSubscriber
            from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_
            self._subscriber = ChannelSubscriber(self._lowstate_topic, LowState_)
            self._subscriber.Init(self._on_lowstate, 10)
        except Exception as e:
            self._subscriber = None
            print(f"[head_control] LowState subscription unavailable: {e}")

    def get_tool(self) -> dict:
        return {
            "name": "head_control",
            "type": "actuator",
            "multiInstance": False,
            "description": "G1 head yaw control — set angle, recenter, shake head, or read current angle",
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {
                        "type": "string",
                        "enum": ["set_angle", "recenter", "shake_head", "get_angle"],
                        "description": "Action to perform",
                    },
                    "angle_deg": {
                        "type": "number",
                        "minimum": math.degrees(self._min_angle_rad),
                        "maximum": math.degrees(self._max_angle_rad),
                        "description": "Target yaw angle in degrees; negative is left and positive is right",
                    },
                    "speed": {
                        "type": "number",
                        "exclusiveMinimum": 0,
                        "description": "Shake frequency in Hz",
                    },
                    "times": {
                        "type": "integer",
                        "minimum": 1,
                        "description": "Number of shake cycles",
                    },
                },
                "required": ["action"],
                "x-action-params": {
                    "set_angle": {"params": ["angle_deg"], "description": "Set head yaw angle"},
                    "recenter": {"params": [], "description": "Return head yaw to zero"},
                    "shake_head": {"params": ["speed", "times"], "description": "Shake head asynchronously, then return to the starting angle"},
                    "get_angle": {"params": [], "description": "Read the current head yaw angle from LowState"},
                },
            },
        }

    def start(self) -> None:
        pass  # DDS publisher is initialized lazily on the first motion command

    def stop(self) -> None:
        self._animation_stop.set()

    def _on_lowstate(self, msg) -> None:
        if self._motor_index >= len(msg.motor_state):
            return
        with self._state_lock:
            self._current_angle_rad = float(msg.motor_state[self._motor_index].q)

    def _get_publisher(self):
        if self._publisher is None:
            from unitree_sdk2py.core.channel import ChannelPublisher
            from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_
            self._publisher = ChannelPublisher(self._lowcmd_topic, LowCmd_)
            self._publisher.Init()
        return self._publisher

    def _send_position(self, angle_rad: float) -> dict:
        from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_

        angle_rad = max(self._min_angle_rad, min(self._max_angle_rad, angle_rad))
        message = unitree_hg_msg_dds__LowCmd_()
        motor = message.motor_cmd[self._motor_index]
        # TODO: Confirm motor index 23, position mode 10, angle direction, and physical limits on hardware.
        motor.mode = self._motor_mode
        motor.q = angle_rad
        motor.dq = 0.0
        motor.tau = 0.0
        motor.kp = self._default_kp
        motor.kd = self._default_kd
        # TODO: Confirm whether LowCmd CRC must be calculated before publishing.
        published = self._get_publisher().Write(message)
        with self._state_lock:
            self._last_commanded_angle_rad = angle_rad
        return {
            "angle_deg": math.degrees(angle_rad),
            "angle_rad": angle_rad,
            "published": bool(published),
        }

    def _cancel_animation(self) -> None:
        self._animation_stop.set()

    def _start_shake(self, speed_hz: float, times: int) -> dict:
        self._cancel_animation()
        stop_event = threading.Event()
        self._animation_stop = stop_event
        with self._state_lock:
            origin = self._current_angle_rad
            if origin is None:
                origin = self._last_commanded_angle_rad

        amplitude = math.radians(self._shake_amplitude_deg)
        duration = times / speed_hz

        def animate():
            started = time.monotonic()
            while not stop_event.is_set():
                elapsed = time.monotonic() - started
                if elapsed >= duration:
                    break
                angle = origin + amplitude * math.sin(2.0 * math.pi * speed_hz * elapsed)
                self._send_position(angle)
                stop_event.wait(0.02)
            if not stop_event.is_set():
                self._send_position(origin)

        self._animation_thread = threading.Thread(target=animate, daemon=True, name="head_shake")
        self._animation_thread.start()
        return {
            "state": "shaking",
            "speed_hz": speed_hz,
            "times": times,
            "amplitude_deg": self._shake_amplitude_deg,
            "return_angle_deg": math.degrees(origin),
        }

    def dispatch(self, action: str, args: dict) -> dict | None:
        if action == "start":
            return {"state": "ready"}
        if action == "set_angle":
            if "angle_deg" not in args:
                return {"error": "angle_deg is required"}
            angle_deg = float(args["angle_deg"])
            if not math.degrees(self._min_angle_rad) <= angle_deg <= math.degrees(self._max_angle_rad):
                return {"error": f"angle_deg must be between {math.degrees(self._min_angle_rad)} and {math.degrees(self._max_angle_rad)}"}
            self._cancel_animation()
            return self._send_position(math.radians(angle_deg))
        if action == "recenter":
            self._cancel_animation()
            return self._send_position(0.0)
        if action == "shake_head":
            speed_hz = float(args.get("speed", self._shake_speed_hz))
            times = int(args.get("times", self._shake_times))
            if speed_hz <= 0:
                return {"error": "speed must be greater than 0"}
            if times < 1:
                return {"error": "times must be at least 1"}
            return self._start_shake(speed_hz, times)
        if action == "get_angle":
            with self._state_lock:
                angle_rad = self._current_angle_rad
            if angle_rad is None:
                return {"error": "No LowState angle received yet"}
            return {"angle_deg": math.degrees(angle_rad), "angle_rad": angle_rad}
        return None


# ── LidarPlugin (sensor) ─────────────────────────────────────────────────────

LIDAR_CLOUD_INTERVAL = 0.05      # 20 Hz max (source is ~10Hz, allow headroom)


class _LidarNode(Node):
    """Subscribes to DDS utlidar PointCloud2 and republishes with gravity alignment."""

    def __init__(self, cloud_topic: str):
        super().__init__("g1_lidar")
        from std_msgs.msg import UInt8MultiArray
        self._cloud_pub = self.create_publisher(UInt8MultiArray, cloud_topic, _LOW_LAT_QOS)
        self._last_cloud_time: float = 0.0
        self._imu_roll:  float = 0.0
        self._imu_pitch: float = 0.0

        # Diagnostics (printed every N frames)
        self._cb_count: int = 0         # total DDS callbacks received
        self._cb_accepted: int = 0      # passed throttle
        self._cb_dropped: int = 0       # queue full
        self._cb_first_time: float = 0.0
        self._worker_count: int = 0
        self._worker_total_ms: float = 0.0

        # Worker thread for point cloud processing (keeps DDS receive thread unblocked)
        self._cloud_queue: queue.Queue = queue.Queue(maxsize=10)
        self._worker = threading.Thread(target=self._process_loop, daemon=True, name="lidar_worker")
        self._worker.start()

        # Subscribe DDS PointCloud2
        try:
            from unitree_sdk2py.core.channel import ChannelSubscriber
            from unitree_sdk2py.idl.sensor_msgs.msg.dds_ import PointCloud2_
            self._cloud_sub = ChannelSubscriber("rt/utlidar/cloud_livox_mid360", PointCloud2_)
            self._cloud_sub.Init(self._on_cloud, 1)  # queueLen=1: use BQueue to avoid blocking DDS receive thread (which delays RPC responses)
            self.get_logger().info(f"LidarNode subscribed rt/utlidar/cloud_livox_mid360 → {cloud_topic}")
        except Exception as e:
            self.get_logger().warn(f"LidarNode: failed to subscribe cloud: {e}")

        # Subscribe DDS Livox IMU for gravity alignment (co-located with lidar)
        try:
            from unitree_sdk2py.core.channel import ChannelSubscriber
            from unitree_sdk2py.idl.sensor_msgs.msg.dds_ import Imu_
            self._livox_imu_sub = ChannelSubscriber("rt/utlidar/imu_livox_mid360", Imu_)
            self._livox_imu_sub.Init(self._on_livox_imu, 10)
            self.get_logger().info("LidarNode subscribed rt/utlidar/imu_livox_mid360 for gravity alignment")
        except Exception as e:
            self.get_logger().warn(f"LidarNode: failed to subscribe Livox IMU: {e}")

    def _on_cloud(self, msg) -> None:
        """DDS callback — throttle and enqueue for worker thread.
        Runs directly in CycloneDDS receive thread (queueLen=0).
        """
        self._cb_count += 1
        now = time.monotonic()
        if now - self._last_cloud_time < LIDAR_CLOUD_INTERVAL:
            return
        self._last_cloud_time = now
        self._cb_accepted += 1

        if self._cb_first_time == 0.0:
            self._cb_first_time = now

        point_step = msg.point_step
        total_points = msg.width * msg.height
        data = msg.data if isinstance(msg.data, (bytes, bytearray)) else bytes(msg.data)

        # Non-blocking put; drop frame if worker is busy
        try:
            self._cloud_queue.put_nowait((point_step, total_points, data,
                                         self._imu_roll, self._imu_pitch))
        except queue.Full:
            self._cb_dropped += 1

        # Print stats every 2000 accepted frames (~200s at 10Hz)
        if self._cb_accepted % 2000 == 0:
            elapsed = now - self._cb_first_time
            avg_hz = self._cb_accepted / elapsed if elapsed > 0 else 0
            print(
                f"[lidar:stats] received={self._cb_count} accepted={self._cb_accepted} "
                f"dropped={self._cb_dropped} avg_hz={avg_hz:.1f} "
                f"worker_avg={self._worker_total_ms/max(self._worker_count,1):.1f}ms",
                flush=True
            )

    def _process_loop(self) -> None:
        """Worker thread: gravity alignment + publish (off the DDS receive thread)."""
        import array as _array
        from std_msgs.msg import UInt8MultiArray
        while True:
            item = self._cloud_queue.get()
            if item is None:
                break
            point_step, total_points, data, roll, pitch = item
            t0 = time.monotonic()

            # Apply gravity alignment (returns bytearray, avoids extra copy)
            data = gravity_align_inplace(data, point_step, total_points, roll, pitch)

            # Publish — pre-allocate buffer to avoid header + data concat copy
            header = struct.pack('<II', point_step, total_points)
            buf = bytearray(8 + len(data))
            buf[:8] = header
            buf[8:] = data
            ros_msg = UInt8MultiArray()
            ros_msg.data = _array.array('B', buf)
            self._cloud_pub.publish(ros_msg)

            elapsed_ms = (time.monotonic() - t0) * 1000
            self._worker_count += 1
            self._worker_total_ms += elapsed_ms

    def _on_livox_imu(self, msg) -> None:
        """Compute roll/pitch from Livox IMU accelerometer (co-located with lidar, inverted mount)."""
        import math
        try:
            acc = msg.linear_acceleration
            ax, ay, az = float(acc.x), float(acc.y), float(acc.z)
            # Livox is mounted inverted: flip y,z to get upright-equivalent frame
            self._imu_roll = math.atan2(-ay, -az)
            self._imu_pitch = math.atan2(-ax, math.sqrt(ay * ay + az * az))
        except Exception:
            pass


class LidarPlugin:
    PREFIX = "lidar"

    def __init__(self, plugin_config: dict, namespace: str, executor):
        self._cloud_topic = f"/{namespace}/lidar/cloud"
        self._node = _LidarNode(self._cloud_topic)
        executor.add_node(self._node)

    def get_tools(self) -> list:
        return [self._cloud_tool()]

    def _cloud_tool(self) -> dict:
        return {
            "name": "lidar_cloud",
            "type": "sensor",
            "multiInstance": False,
            "description": f"Livox Mid-360 full point cloud passthrough at 10Hz. Binary format: [uint32 point_step][uint32 total_points][raw PointCloud2 bytes]. Publishes to {self._cloud_topic}",
            "inputSchema": {"type": "object", "properties": {}},
            "topic_out": [{"topic": self._cloud_topic, "format": "sensor/pointcloud"}],
            "configSchema": {
                "type": "object",
                "properties": {},
            },
        }

    def start(self) -> None:
        pass  # DDS subscription starts in __init__

    def stop(self) -> None:
        pass

    def dispatch(self, action: str, args: dict) -> dict | None:
        if action == "start":
            return {"state": "running"}
        if action == "stop":
            return {"state": "idle"}
        if action == "info":
            return {"state": "running", "topic_out": [{"topic": self._cloud_topic, "format": "sensor/pointcloud"}]}
        return None


# ── SpatialPlugin (actuator + sensor) ────────────────────────────────────────

import math
import os
import sqlite3

SPATIAL_POS_INTERVAL = 0.1      # 10 Hz pos_tag publish
SPATIAL_TRAJ_INTERVAL = 3.0     # trajectory sample every 3s
SPATIAL_TRAJ_MIN_DIST = 0.3     # or if moved > 0.3m


class _SpatialDB:
    """SQLite storage for maps, POIs, and trajectory."""

    def __init__(self, db_path: str):
        os.makedirs(os.path.dirname(db_path) if os.path.dirname(db_path) else '.', exist_ok=True)
        self._conn = sqlite3.connect(db_path, check_same_thread=False)
        self._conn.row_factory = sqlite3.Row
        self._migrate()

    def _migrate(self):
        c = self._conn.cursor()
        c.executescript("""
            CREATE TABLE IF NOT EXISTS maps (
                name TEXT PRIMARY KEY,
                pcd_path TEXT NOT NULL,
                created_at REAL DEFAULT (strftime('%s','now')),
                last_used_at REAL DEFAULT 0
            );
            CREATE TABLE IF NOT EXISTS poi (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                name TEXT NOT NULL,
                description TEXT DEFAULT '',
                x REAL NOT NULL, y REAL NOT NULL, yaw REAL DEFAULT 0,
                map_name TEXT NOT NULL,
                created_at REAL DEFAULT (strftime('%s','now')),
                UNIQUE(name, map_name)
            );
            CREATE TABLE IF NOT EXISTS trajectory (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                x REAL NOT NULL, y REAL NOT NULL, yaw REAL DEFAULT 0,
                ts REAL NOT NULL
            );
            CREATE TABLE IF NOT EXISTS meta (
                key TEXT PRIMARY KEY,
                value TEXT
            );
        """)
        self._conn.commit()

    def get_last_used_map(self) -> str | None:
        row = self._conn.execute(
            "SELECT value FROM meta WHERE key='last_used_map'"
        ).fetchone()
        return row['value'] if row else None

    def set_last_used_map(self, name: str):
        self._conn.execute(
            "INSERT OR REPLACE INTO meta (key, value) VALUES ('last_used_map', ?)", (name,)
        )
        self._conn.execute(
            "UPDATE maps SET last_used_at = strftime('%s','now') WHERE name = ?", (name,)
        )
        self._conn.commit()

    def add_map(self, name: str, pcd_path: str):
        self._conn.execute(
            "INSERT OR REPLACE INTO maps (name, pcd_path) VALUES (?, ?)", (name, pcd_path)
        )
        self._conn.commit()

    def list_maps(self) -> list[dict]:
        rows = self._conn.execute("SELECT name, pcd_path, created_at, last_used_at FROM maps ORDER BY last_used_at DESC").fetchall()
        return [dict(r) for r in rows]

    def get_map(self, name: str) -> dict | None:
        row = self._conn.execute("SELECT * FROM maps WHERE name = ?", (name,)).fetchone()
        return dict(row) if row else None

    def add_poi(self, name: str, x: float, y: float, yaw: float, map_name: str, description: str = ""):
        self._conn.execute(
            "INSERT OR REPLACE INTO poi (name, description, x, y, yaw, map_name) VALUES (?, ?, ?, ?, ?, ?)",
            (name, description, x, y, yaw, map_name)
        )
        self._conn.commit()

    def delete_poi(self, name: str, map_name: str) -> bool:
        cur = self._conn.execute("DELETE FROM poi WHERE name = ? AND map_name = ?", (name, map_name))
        self._conn.commit()
        return cur.rowcount > 0

    def list_pois(self, map_name: str) -> list[dict]:
        rows = self._conn.execute(
            "SELECT name, description, x, y, yaw FROM poi WHERE map_name = ? ORDER BY name",
            (map_name,)
        ).fetchall()
        return [dict(r) for r in rows]

    def find_poi(self, query: str, map_name: str) -> dict | None:
        row = self._conn.execute(
            "SELECT name, description, x, y, yaw FROM poi WHERE map_name = ? AND name LIKE ?",
            (map_name, f"%{query}%")
        ).fetchone()
        return dict(row) if row else None

    def add_trajectory(self, x: float, y: float, yaw: float, ts: float):
        self._conn.execute(
            "INSERT INTO trajectory (x, y, yaw, ts) VALUES (?, ?, ?, ?)", (x, y, yaw, ts)
        )
        self._conn.commit()

    def prune_trajectory(self, keep_seconds: float = 3600):
        """Keep only last N seconds of trajectory."""
        self._conn.execute(
            "DELETE FROM trajectory WHERE ts < ?", (time.time() - keep_seconds,)
        )
        self._conn.commit()


def _bearing_label(dx: float, dy: float) -> str:
    """Convert delta (x=forward, y=left) to bearing label."""
    angle = math.atan2(dy, dx)  # radians, 0=forward, pi/2=left
    deg = math.degrees(angle)
    if -22.5 <= deg < 22.5:
        return "front"
    elif 22.5 <= deg < 67.5:
        return "left_front"
    elif 67.5 <= deg < 112.5:
        return "left"
    elif 112.5 <= deg < 157.5:
        return "left_behind"
    elif -67.5 <= deg < -22.5:
        return "right_front"
    elif -112.5 <= deg < -67.5:
        return "right"
    elif -157.5 <= deg < -112.5:
        return "right_behind"
    else:
        return "behind"


class _SlamInfoNode(Node):
    """Subscribes to rt/slam_info, rt/slam_key_info, and mapping point clouds.
    Maintains a 3D voxel map buffer and publishes full map at 1Hz."""

    import numpy as np

    VOXEL_SIZE = 0.05            # 5cm voxel grid for deduplication
    MAP_PUBLISH_INTERVAL = 1.0   # 1 Hz full map publish
    MAP_SAVE_INTERVAL = 5.0      # auto-save PCD every 5s
    MAX_SEND_POINTS = 50000      # max points per publish (downsample if exceeded)
    RECENT_CLOUD_MAX = 50000     # recent cloud ring buffer capacity
    KF_DIST_THRESH = 2.0         # keyframe every 2m movement
    KF_YAW_THRESH = 0.52         # or 30° rotation

    def __init__(self, pos_tag_topic: str, mapping_topic: str, db: _SpatialDB, sc_mgr=None, slam_cloud_topic: str | None = None):
        super().__init__("g1_spatial")
        self._db = db
        self._sc_mgr = sc_mgr  # ScanContextManager (optional)
        self._auto_mapping_cb = None  # set by SpatialPlugin: called once on first localization
        self._pos_tag_pub = self.create_publisher(String, pos_tag_topic, _LOW_LAT_QOS)

        from std_msgs.msg import UInt8MultiArray
        self._mapping_pub = self.create_publisher(UInt8MultiArray, mapping_topic, _LOW_LAT_QOS)

        # slam_cloud: real-time SLAM point cloud passthrough (standard coordinate system)
        self._slam_cloud_pub = None
        self._last_slam_cloud_time: float = 0.0
        SLAM_CLOUD_INTERVAL = 0.2  # 5Hz
        self._slam_cloud_interval = SLAM_CLOUD_INTERVAL
        if slam_cloud_topic:
            self._slam_cloud_pub = self.create_publisher(UInt8MultiArray, slam_cloud_topic, _LOW_LAT_QOS)

        self._current_pose: dict | None = None
        self._map_status: str = "idle"    # idle | mapping | localized
        self._nav_status: dict | None = None
        self._nav_target_name: str | None = None
        self._active_map: str | None = None
        self._lock = threading.Lock()

        self._last_pub_time: float = 0.0
        self._last_traj_time: float = 0.0
        self._last_traj_pose: tuple = (0.0, 0.0)

        # 3D voxel map buffer: dict[(ix,iy,iz)] → (x, y, z)
        self._map_buffer: dict[tuple, tuple] = {}
        self._map_buffer_lock = threading.Lock()
        self._map_buffer_dirty = False  # set True when new points added, False after save

        # Cloud processing queue + background thread (decouples DDS callback from heavy processing)
        self._cloud_queue = queue.Queue(maxsize=50)
        self._cloud_processor_running = True
        self._cloud_processor_thread = threading.Thread(
            target=self._cloud_processor_loop, daemon=True, name="cloud_processor"
        )
        self._cloud_processor_thread.start()

        # Recent cloud ring buffer for discover_map fingerprinting
        self._recent_cloud = _SlamInfoNode.np.zeros((self.RECENT_CLOUD_MAX, 3), dtype=_SlamInfoNode.np.float32)
        self._recent_cloud_count = 0
        self._recent_cloud_write_idx = 0

        # Keyframe tracking for Scan Context
        self._last_kf_pose: tuple = (0.0, 0.0, 0.0)  # (x, y, yaw)

        # Watchdog: detect when point cloud stops arriving
        self._last_cloud_time: float = 0.0
        self._watchdog_cb = None  # set by SpatialPlugin: called when cloud stops
        self._watchdog_timer: threading.Timer | None = None
        self._watchdog_running = False
        self.WATCHDOG_TIMEOUT = 3.0  # seconds without data before triggering restart

        # 1Hz full map publish timer
        self._last_map_publish_time: float = 0.0

        # Auto-save PCD timer
        self._last_map_save_time: float = 0.0
        self._pcd_save_dir: str | None = None  # set by SpatialPlugin when active map is set
        self._save_timer: threading.Timer | None = None
        self._save_timer_running = False

        # Subscribe DDS topics (store refs to prevent GC from killing subscriptions)
        self._dds_subs = []
        try:
            from unitree_sdk2py.core.channel import ChannelSubscriber
            from unitree_sdk2py.idl.std_msgs.msg.dds_ import String_
            info_sub = ChannelSubscriber("rt/slam_info", String_)
            info_sub.Init(self._on_slam_info, 10)
            self._dds_subs.append(info_sub)
            self.get_logger().info("SpatialNode subscribed rt/slam_info")
        except Exception as e:
            self.get_logger().warn(f"SpatialNode: failed to subscribe rt/slam_info: {e}")

        try:
            from unitree_sdk2py.core.channel import ChannelSubscriber
            from unitree_sdk2py.idl.std_msgs.msg.dds_ import String_
            key_sub = ChannelSubscriber("rt/slam_key_info", String_)
            key_sub.Init(self._on_slam_key_info, 10)
            self._dds_subs.append(key_sub)
            self.get_logger().info("SpatialNode subscribed rt/slam_key_info")
        except Exception as e:
            self.get_logger().warn(f"SpatialNode: failed to subscribe rt/slam_key_info: {e}")

        # Subscribe mapping point clouds (both mapping and relocation modes)
        try:
            from unitree_sdk2py.core.channel import ChannelSubscriber
            from unitree_sdk2py.idl.sensor_msgs.msg.dds_ import PointCloud2_
            map_cloud_sub = ChannelSubscriber("rt/unitree/slam_mapping/points", PointCloud2_)
            map_cloud_sub.Init(self._on_mapping_cloud, 10)
            self._dds_subs.append(map_cloud_sub)
            self.get_logger().info("SpatialNode subscribed rt/unitree/slam_mapping/points")
        except Exception as e:
            self.get_logger().warn(f"SpatialNode: failed to subscribe mapping points: {e}")

        try:
            from unitree_sdk2py.core.channel import ChannelSubscriber
            from unitree_sdk2py.idl.sensor_msgs.msg.dds_ import PointCloud2_
            reloc_cloud_sub = ChannelSubscriber("rt/unitree/slam_relocation/points", PointCloud2_)
            reloc_cloud_sub.Init(self._on_mapping_cloud, 10)
            self._dds_subs.append(reloc_cloud_sub)
            self.get_logger().info("SpatialNode subscribed rt/unitree/slam_relocation/points")
        except Exception as e:
            self.get_logger().warn(f"SpatialNode: failed to subscribe relocation points: {e}")

    def _on_slam_info(self, msg) -> None:
        try:
            data = json.loads(msg.data)
        except Exception:
            return

        msg_type = data.get("type", "")

        if msg_type == "pos_info" or msg_type == "mapping_info":
            pose_data = data.get("data", {}).get("currentPose")
            if pose_data:
                q_x = float(pose_data.get("q_x", 0.0))
                q_y = float(pose_data.get("q_y", 0.0))
                q_z = float(pose_data.get("q_z", 0.0))
                q_w = float(pose_data.get("q_w", 1.0))
                yaw = math.atan2(
                    2 * (q_w * q_z + q_x * q_y),
                    1 - 2 * (q_y * q_y + q_z * q_z),
                )
                with self._lock:
                    prev_status = self._map_status
                    self._current_pose = {
                        "x": pose_data["x"],
                        "y": pose_data["y"],
                        "yaw": round(yaw, 3),
                    }
                    if msg_type == "pos_info":
                        self._map_status = "localized"
                    elif msg_type == "mapping_info":
                        self._map_status = "mapping"

                # Auto-transition: localized → mapping (always be mapping)
                # Only fire if transitioning TO localized from a non-mapping state
                if msg_type == "pos_info" and prev_status != "mapping" and prev_status != "localized" and self._auto_mapping_cb:
                    self.get_logger().info("[slam_info] Localized! Triggering auto StartMapping...")
                    try:
                        self._auto_mapping_cb()
                    except Exception as e:
                        self.get_logger().warn(f"[slam_info] auto-mapping callback failed: {e}")

            # Trajectory recording
            self._maybe_record_trajectory()
            # Publish pos_tag
            self._maybe_publish_pos_tag()

        elif msg_type == "ctrl_info":
            ctrl = data.get("data", {})
            progress = ctrl.get("progress", {})
            with self._lock:
                self._nav_status = {
                    "target": self._nav_target_name,
                    "progress": progress.get("completion_percentage", 0),
                    "eta_seconds": progress.get("last_time", 0),
                    "is_arrived": ctrl.get("is_arrived", False),
                    "obstacle": ctrl.get("obsInfo", {}).get("state", False),
                    "is_paused": ctrl.get("stateMachine", {}).get("isPause", False),
                }
                if ctrl.get("is_arrived"):
                    self._nav_status = None
                    self._nav_target_name = None

    def _on_slam_key_info(self, msg) -> None:
        try:
            data = json.loads(msg.data)
        except Exception:
            return
        if data.get("type") == "task_result":
            is_arrived = data.get("data", {}).get("is_arrived", False)
            if is_arrived:
                with self._lock:
                    self._nav_status = None
                    self._nav_target_name = None

    def _on_mapping_cloud(self, msg) -> None:
        """DDS callback: fast enqueue only. Processing happens in separate thread."""
        # Quick extract raw data and enqueue — don't block DDS thread
        try:
            data = bytes(msg.data)
            if len(data) < msg.point_step:
                return

            # Real-time slam_cloud passthrough (throttled)
            if self._slam_cloud_pub is not None:
                now = time.monotonic()
                if now - self._last_slam_cloud_time >= self._slam_cloud_interval:
                    self._last_slam_cloud_time = now
                    self._publish_slam_cloud(msg.fields, msg.point_step, msg.width * msg.height, data)

            self._cloud_queue.put_nowait((msg.fields, msg.point_step, msg.width * msg.height, data))
        except Exception:
            pass  # queue full, drop frame

    def _publish_slam_cloud(self, fields, point_step: int, total_points: int, data: bytes) -> None:
        """Parse SLAM PointCloud2, transform to standard coords, publish as sensor/pointcloud binary."""
        np = _SlamInfoNode.np
        num_points = min(total_points, 20000)
        if len(data) < num_points * point_step:
            num_points = len(data) // point_step
        if num_points == 0:
            return

        # Parse field offsets
        field_map = {}
        for f in fields:
            field_map[f.name] = f.offset
        x_off = field_map.get("x", 0)
        y_off = field_map.get("y", 4)
        z_off = field_map.get("z", 8)

        # Extract x, y, z via numpy
        raw = np.frombuffer(data, dtype=np.uint8, count=num_points * point_step)
        raw = raw.reshape(num_points, point_step)
        sx = raw[:, x_off:x_off+4].view(np.float32).ravel()
        sy = raw[:, y_off:y_off+4].view(np.float32).ravel()
        sz = raw[:, z_off:z_off+4].view(np.float32).ravel()

        # Filter invalid
        valid = (
            np.isfinite(sx) & np.isfinite(sy) & np.isfinite(sz) &
            (np.abs(sx) < 50) & (np.abs(sy) < 50) & (np.abs(sz) < 20)
        )
        sx, sy, sz = sx[valid], sy[valid], sz[valid]
        n = len(sx)
        if n == 0:
            return

        # Transform to standard display coordinates:
        # SLAM output is already in standard coordinate system, pass through directly
        out = np.column_stack([sx, sy, sz]).astype(np.float32)

        # Pack binary: [uint32 point_step=12][uint32 total_points][float32 x,y,z × N]
        header = struct.pack('<II', 12, n)
        from std_msgs.msg import UInt8MultiArray
        ros_msg = UInt8MultiArray()
        ros_msg.data = list(header + out.tobytes())
        self._slam_cloud_pub.publish(ros_msg)

    def _cloud_processor_loop(self):
        """Background thread: processes queued point clouds at its own pace."""
        np = _SlamInfoNode.np
        while self._cloud_processor_running:
            try:
                item = self._cloud_queue.get(timeout=1.0)
            except Exception:
                continue

            fields, point_step, total_points, data = item
            if total_points == 0:
                continue

            # Parse fields
            field_map = {}
            for f in fields:
                field_map[f.name] = (f.offset, f.datatype)
            x_off = field_map.get("x", (0, 7))[0]
            y_off = field_map.get("y", (4, 7))[0]
            z_off = field_map.get("z", (8, 7))[0]

            # Numpy vectorized parsing
            num_points = min(total_points, 20000)
            if len(data) < num_points * point_step:
                num_points = len(data) // point_step

            # Build structured dtype for the point layout
            # Extract x, y, z using byte offsets directly
            raw = np.frombuffer(data, dtype=np.uint8, count=num_points * point_step)
            raw = raw.reshape(num_points, point_step)

            x = raw[:, x_off:x_off+4].view(np.float32).ravel()
            y = raw[:, y_off:y_off+4].view(np.float32).ravel()
            z = raw[:, z_off:z_off+4].view(np.float32).ravel()

            # Filter invalid (NaN and out of range)
            valid = (
                np.isfinite(x) & np.isfinite(y) & np.isfinite(z) &
                (np.abs(x) < 50) & (np.abs(y) < 50) & (np.abs(z) < 20)
            )
            x, y, z = x[valid], y[valid], z[valid]

            if len(x) == 0:
                continue

            pts_arr = np.column_stack([x, y, z]).astype(np.float32)

            # Merge into voxel map buffer (deduplication)
            voxel_size = self.VOXEL_SIZE
            # Vectorized voxel key computation
            ix = (pts_arr[:, 0] / voxel_size).astype(np.int32)
            iy = (pts_arr[:, 1] / voxel_size).astype(np.int32)
            iz = (pts_arr[:, 2] / voxel_size).astype(np.int32)

            with self._map_buffer_lock:
                prev_size = len(self._map_buffer)
                for j in range(len(pts_arr)):
                    key = (int(ix[j]), int(iy[j]), int(iz[j]))
                    if key not in self._map_buffer:
                        self._map_buffer[key] = (float(pts_arr[j, 0]), float(pts_arr[j, 1]), float(pts_arr[j, 2]))
                new_size = len(self._map_buffer)
                if new_size > prev_size:
                    self._map_buffer_dirty = True

            self._last_cloud_time = time.monotonic()
            new_points = new_size - prev_size
            self.get_logger().info(
                f"[mapping_cloud] frame: {len(pts_arr)} pts parsed, "
                f"+{new_points} new voxels, total={new_size}"
            )

            # Update recent cloud ring buffer
            n = len(pts_arr)
            start = self._recent_cloud_write_idx
            cap = self.RECENT_CLOUD_MAX
            if n <= cap:
                end = start + n
                if end <= cap:
                    self._recent_cloud[start:end] = pts_arr
                else:
                    first = cap - start
                    self._recent_cloud[start:cap] = pts_arr[:first]
                    self._recent_cloud[0:n - first] = pts_arr[first:]
                self._recent_cloud_write_idx = (start + n) % cap
                self._recent_cloud_count = min(self._recent_cloud_count + n, cap)
            else:
                self._recent_cloud[:] = pts_arr[-cap:]
                self._recent_cloud_write_idx = 0
                self._recent_cloud_count = cap

            # Keyframe + publish
            self._maybe_add_keyframe(pts_arr)
            self._maybe_publish_full_map()

    def _maybe_record_trajectory(self):
        with self._lock:
            if self._current_pose is None:
                return
            x, y = self._current_pose["x"], self._current_pose["y"]
            yaw = self._current_pose["yaw"]

        now = time.time()
        dx = x - self._last_traj_pose[0]
        dy = y - self._last_traj_pose[1]
        dist = math.sqrt(dx * dx + dy * dy)

        if (now - self._last_traj_time >= SPATIAL_TRAJ_INTERVAL) or (dist >= SPATIAL_TRAJ_MIN_DIST):
            self._last_traj_time = now
            self._last_traj_pose = (x, y)
            self._db.add_trajectory(x, y, yaw, now)

    def _maybe_add_keyframe(self, pts_arr) -> None:
        """Generate Scan Context keyframe if robot moved/rotated enough."""
        if self._sc_mgr is None:
            return
        with self._lock:
            if self._current_pose is None or self._active_map is None:
                return
            x, y = self._current_pose["x"], self._current_pose["y"]
            yaw = self._current_pose["yaw"]
            active_map = self._active_map

        lx, ly, lyaw = self._last_kf_pose
        dx = x - lx
        dy = y - ly
        dist = math.sqrt(dx * dx + dy * dy)
        dyaw = abs(yaw - lyaw)
        if dyaw > math.pi:
            dyaw = 2 * math.pi - dyaw

        if dist >= self.KF_DIST_THRESH or dyaw >= self.KF_YAW_THRESH:
            sc = self._sc_mgr.make_scan_context(pts_arr)
            self._sc_mgr.add_keyframe(active_map, sc, (x, y, 0.0))
            self._last_kf_pose = (x, y, yaw)

    def _maybe_publish_full_map(self) -> None:
        """Publish the full 3D voxel map at 1Hz."""
        np = _SlamInfoNode.np
        now = time.monotonic()
        if now - self._last_map_publish_time < self.MAP_PUBLISH_INTERVAL:
            return
        self._last_map_publish_time = now

        with self._lock:
            pose = self._current_pose
        robot_x = pose["x"] if pose else 0.0
        robot_y = pose["y"] if pose else 0.0
        # The mapping renderer already negates the packet yaw after mapping
        # SLAM +Y to Three.js -Z, so publish the display-frame value.
        robot_yaw = -pose["yaw"] if pose else 0.0

        # Extract points from voxel buffer
        with self._map_buffer_lock:
            if not self._map_buffer:
                return
            all_points = list(self._map_buffer.values())

        pts = np.array(all_points, dtype=np.float32)
        num_points = len(pts)

        # Downsample if too many
        if num_points > self.MAX_SEND_POINTS:
            indices = np.random.choice(num_points, self.MAX_SEND_POINTS, replace=False)
            pts = pts[indices]
            num_points = self.MAX_SEND_POINTS

        # Pack binary: [float32 robot_x, robot_y, robot_yaw][uint8 flags][uint32 N][float32 x,y,z × N]
        # flags: bit0=full_map(1), bit1=has_z(1) → flags = 0x03
        flags = 0x03
        header = struct.pack('<fffBI', robot_x, robot_y, robot_yaw, flags, num_points)
        body = pts.tobytes()

        from std_msgs.msg import UInt8MultiArray
        ros_msg = UInt8MultiArray()
        ros_msg.data = list(header + body)
        self._mapping_pub.publish(ros_msg)

    def _maybe_save_pcd(self) -> None:
        """Auto-save map buffer to PCD file. Called by a recurring timer thread."""
        np = _SlamInfoNode.np

        if not self._pcd_save_dir:
            self.get_logger().debug("[save_pcd] no save dir set")
            self._schedule_save_timer()
            return

        with self._lock:
            active_map = self._active_map
        if not active_map:
            self.get_logger().debug("[save_pcd] no active map")
            self._schedule_save_timer()
            return

        with self._map_buffer_lock:
            if not self._map_buffer or not self._map_buffer_dirty:
                self._schedule_save_timer()
                return
            all_points = list(self._map_buffer.values())
            self._map_buffer_dirty = False

        if len(all_points) < 10:
            self._schedule_save_timer()
            return

        # Write PCD file (ASCII format for simplicity and compatibility)
        pcd_path = os.path.join(self._pcd_save_dir, f"{active_map}.pcd")
        os.makedirs(os.path.dirname(pcd_path), exist_ok=True)
        try:
            pts = np.array(all_points, dtype=np.float32)
            num = len(pts)
            with open(pcd_path, 'w') as f:
                f.write("# .PCD v0.7 - Point Cloud Data\n")
                f.write("VERSION 0.7\n")
                f.write("FIELDS x y z\n")
                f.write("SIZE 4 4 4\n")
                f.write("TYPE F F F\n")
                f.write("COUNT 1 1 1\n")
                f.write(f"WIDTH {num}\n")
                f.write("HEIGHT 1\n")
                f.write("VIEWPOINT 0 0 0 1 0 0 0\n")
                f.write(f"POINTS {num}\n")
                f.write("DATA ascii\n")
                for i in range(num):
                    f.write(f"{pts[i,0]:.4f} {pts[i,1]:.4f} {pts[i,2]:.4f}\n")
            self.get_logger().info(f"Auto-saved PCD: {pcd_path} ({num} points)")
        except Exception as e:
            self.get_logger().warn(f"Failed to save PCD: {e}")

        self._schedule_save_timer()

    def _schedule_save_timer(self):
        """Schedule the next PCD auto-save."""
        if not self._save_timer_running:
            return
        self._save_timer = threading.Timer(self.MAP_SAVE_INTERVAL, self._maybe_save_pcd)
        self._save_timer.daemon = True
        self._save_timer.start()

    def _start_save_timer(self):
        """Start the recurring PCD save timer."""
        self._save_timer_running = True
        self._schedule_save_timer()

    def _stop_save_timer(self):
        """Stop the recurring PCD save timer."""
        self._save_timer_running = False
        if self._save_timer:
            self._save_timer.cancel()
            self._save_timer = None
        self._stop_watchdog()

    def _start_watchdog(self):
        """Start the cloud watchdog that detects when point cloud stops arriving."""
        self._watchdog_running = True
        self._last_cloud_time = time.monotonic()
        self._schedule_watchdog()

    def _stop_watchdog(self):
        """Stop the cloud watchdog."""
        self._watchdog_running = False
        if self._watchdog_timer:
            self._watchdog_timer.cancel()
            self._watchdog_timer = None

    def _schedule_watchdog(self):
        if not self._watchdog_running:
            return
        self._watchdog_timer = threading.Timer(1.0, self._check_watchdog)
        self._watchdog_timer.daemon = True
        self._watchdog_timer.start()

    def _check_watchdog(self):
        """Check if point cloud has stopped arriving. If so, trigger restart."""
        if not self._watchdog_running:
            return
        now = time.monotonic()
        elapsed = now - self._last_cloud_time
        if self._last_cloud_time > 0 and elapsed > self.WATCHDOG_TIMEOUT and self._watchdog_cb:
            self.get_logger().warn(
                f"[watchdog] No point cloud for {elapsed:.1f}s, triggering restart"
            )
            self._last_cloud_time = now  # reset to avoid repeated triggers
            try:
                self._watchdog_cb()
            except Exception as e:
                self.get_logger().warn(f"[watchdog] restart callback failed: {e}")
        self._schedule_watchdog()

    def set_pcd_save_dir(self, path: str):
        """Set the directory for auto-saving PCD files and start the save timer."""
        self._pcd_save_dir = path
        self._start_save_timer()

    def load_pcd_to_buffer(self, pcd_path: str) -> None:
        """Load a PCD file into the voxel map buffer."""
        np = _SlamInfoNode.np
        if not os.path.exists(pcd_path):
            self.get_logger().warn(f"PCD file not found: {pcd_path}")
            return

        points = self._parse_pcd(pcd_path)
        if points is None or len(points) == 0:
            return

        voxel_size = self.VOXEL_SIZE
        with self._map_buffer_lock:
            for i in range(len(points)):
                ix = int(points[i, 0] / voxel_size)
                iy = int(points[i, 1] / voxel_size)
                iz = int(points[i, 2] / voxel_size)
                self._map_buffer[(ix, iy, iz)] = (points[i, 0], points[i, 1], points[i, 2])

        self.get_logger().info(f"Loaded {len(points)} points from PCD, buffer size: {len(self._map_buffer)}")

    def clear_map_buffer(self) -> None:
        """Clear the voxel map buffer."""
        with self._map_buffer_lock:
            self._map_buffer.clear()

    def get_recent_cloud(self):
        """Return recent cloud points as Nx3 numpy array (for discover fingerprinting)."""
        np = _SlamInfoNode.np
        count = min(self._recent_cloud_count, self.RECENT_CLOUD_MAX)
        if count == 0:
            return None
        return self._recent_cloud[:count].copy()

    @staticmethod
    def _parse_pcd(path: str):
        """Parse ASCII/binary PCD file, extract x,y,z columns. Returns Nx3 numpy array."""
        np = _SlamInfoNode.np
        try:
            with open(path, 'rb') as f:
                header_lines = []
                while True:
                    line = f.readline()
                    if not line:
                        return None
                    line_str = line.decode('ascii', errors='ignore').strip()
                    header_lines.append(line_str)
                    if line_str.startswith('DATA'):
                        break

                # Parse header
                fields = []
                num_points = 0
                data_type = "ascii"
                field_sizes = []
                field_types = []
                for hl in header_lines:
                    parts = hl.split()
                    if parts[0] == "FIELDS":
                        fields = parts[1:]
                    elif parts[0] == "SIZE":
                        field_sizes = [int(s) for s in parts[1:]]
                    elif parts[0] == "TYPE":
                        field_types = parts[1:]
                    elif parts[0] == "POINTS":
                        num_points = int(parts[1])
                    elif parts[0] == "DATA":
                        data_type = parts[1].lower()

                if num_points == 0:
                    return None

                # Find x, y, z field indices
                try:
                    xi = fields.index("x")
                    yi = fields.index("y")
                    zi = fields.index("z")
                except ValueError:
                    return None

                if data_type == "ascii":
                    points = []
                    for _ in range(num_points):
                        line = f.readline().decode('ascii', errors='ignore').strip()
                        if not line:
                            break
                        vals = line.split()
                        if len(vals) <= max(xi, yi, zi):
                            continue
                        x = float(vals[xi])
                        y = float(vals[yi])
                        z = float(vals[zi])
                        if x != x or y != y or z != z:
                            continue
                        points.append((x, y, z))
                    return np.array(points, dtype=np.float32) if points else None

                elif data_type == "binary":
                    point_size = sum(field_sizes)
                    raw = f.read(num_points * point_size)
                    if len(raw) < num_points * point_size:
                        num_points = len(raw) // point_size

                    # Compute byte offsets for x, y, z
                    offsets = [0]
                    for s in field_sizes[:-1]:
                        offsets.append(offsets[-1] + s)

                    x_off = offsets[xi]
                    y_off = offsets[yi]
                    z_off = offsets[zi]

                    points = np.zeros((num_points, 3), dtype=np.float32)
                    for i in range(num_points):
                        base = i * point_size
                        points[i, 0] = struct.unpack_from('<f', raw, base + x_off)[0]
                        points[i, 1] = struct.unpack_from('<f', raw, base + y_off)[0]
                        points[i, 2] = struct.unpack_from('<f', raw, base + z_off)[0]

                    # Filter NaN
                    valid = ~np.isnan(points).any(axis=1)
                    return points[valid]

        except Exception:
            return None

    def _maybe_publish_pos_tag(self):
        now = time.monotonic()
        if now - self._last_pub_time < SPATIAL_POS_INTERVAL:
            return
        self._last_pub_time = now

        with self._lock:
            pose = self._current_pose
            map_status = self._map_status
            nav_status = dict(self._nav_status) if self._nav_status else None
            active_map = self._active_map

        if pose is None:
            return

        # Compute nearby tags
        tags_in_range = []
        if active_map:
            pois = self._db.list_pois(active_map)
            for poi in pois:
                dx = poi["x"] - pose["x"]
                dy = poi["y"] - pose["y"]
                dist = math.sqrt(dx * dx + dy * dy)
                if dist <= 20.0:  # only show within 20m
                    # Transform to robot frame for bearing
                    cos_yaw = math.cos(-pose["yaw"])
                    sin_yaw = math.sin(-pose["yaw"])
                    rx = dx * cos_yaw - dy * sin_yaw
                    ry = dx * sin_yaw + dy * cos_yaw
                    tags_in_range.append({
                        "name": poi["name"],
                        "dist": round(dist, 2),
                        "bearing": _bearing_label(rx, ry),
                    })
            tags_in_range.sort(key=lambda t: t["dist"])

        nearest = tags_in_range[0] if tags_in_range else None

        output = {
            "pose": pose,
            "nearest_tag": nearest,
            "tags_in_range": tags_in_range[:5],
            "map_status": map_status,
            "nav_status": nav_status,
        }

        out = String()
        out.data = json.dumps(output)
        self._pos_tag_pub.publish(out)

    def get_pose(self) -> dict | None:
        with self._lock:
            return dict(self._current_pose) if self._current_pose else None

    def set_active_map(self, name: str | None):
        with self._lock:
            self._active_map = name

    def set_map_status(self, status: str):
        with self._lock:
            self._map_status = status

    def set_nav_target(self, name: str | None):
        with self._lock:
            self._nav_target_name = name


class SpatialPlugin:
    PREFIX = "spatial"

    def __init__(self, plugin_config: dict, namespace: str, executor, slam_client, smart_motion=None):
        self._client = slam_client
        self._smart_motion = smart_motion
        self._map_dir = plugin_config.get("map_dir", "/home/unitree")
        os.makedirs(self._map_dir, exist_ok=True)
        db_path = plugin_config.get("db_path", os.path.join(os.path.dirname(__file__), "resource", "spatial.db"))
        self._db = _SpatialDB(db_path)

        # Scan Context manager for auto-discover
        sc_db_path = os.path.join(os.path.dirname(db_path), "scan_context.db")
        from scan_context import ScanContextManager
        self._sc_mgr = ScanContextManager(sc_db_path)

        self._pos_tag_topic = f"/{namespace}/spatial/pos_tag"
        self._mapping_topic = f"/{namespace}/spatial/mapping"
        self._slam_cloud_topic = f"/{namespace}/spatial/slam_cloud"
        self._node = _SlamInfoNode(self._pos_tag_topic, self._mapping_topic, self._db, self._sc_mgr, slam_cloud_topic=self._slam_cloud_topic)
        self._node.set_active_map(self._db.get_last_used_map())
        self._node.set_pcd_save_dir(self._map_dir)
        self._node._auto_mapping_cb = self._on_localized
        self._node._watchdog_cb = None
        # Watchdog disabled — SLAM mapping cloud may not always be available
        # self._node._start_watchdog()
        executor.add_node(self._node)

    def get_tools(self) -> list:
        return [self._spatial_tool(), self._pos_tag_tool(), self._mapping_tool(), self._slam_cloud_tool()]

    def _pos_tag_tool(self) -> dict:
        return {
            "name": "pos_tag",
            "type": "sensor",
            "multiInstance": False,
            "description": f"Spatial position + nearest tags — current pose, nearby POIs with distance/bearing, map/nav status. 10Hz to {self._pos_tag_topic}",
            "inputSchema": {"type": "object", "properties": {}},
            "topic_out": [{"topic": self._pos_tag_topic, "format": "data/json"}],
        }

    def _mapping_tool(self) -> dict:
        return {
            "name": "slam_mapping",
            "type": "sensor",
            "multiInstance": False,
            "description": f"SLAM 3D mapping visualization — full 3D point cloud map with robot position. Binary format: [float32 robot_x,y,yaw][uint8 flags][uint32 N][float32 x,y,z × N]. 1Hz to {self._mapping_topic}",
            "inputSchema": {"type": "object", "properties": {}},
            "topic_out": [{"topic": self._mapping_topic, "format": "sensor/mapping"}],
        }

    def _slam_cloud_tool(self) -> dict:
        return {
            "name": "slam_cloud",
            "type": "sensor",
            "multiInstance": False,
            "description": f"Real-time SLAM point cloud at 5Hz in standard coordinate system. Binary format: [uint32 point_step=12][uint32 total_points][float32 x,y,z × N]. Subscribes rt/unitree/slam_mapping/points, transforms and publishes to {self._slam_cloud_topic}",
            "inputSchema": {"type": "object", "properties": {}},
            "topic_out": [{"topic": self._slam_cloud_topic, "format": "sensor/pointcloud"}],
        }

    def _spatial_tool(self) -> dict:
        return {
            "name": "spatial",
            "type": "actuator",
            "multiInstance": False,
            "description": "Spatial intelligence — place tagging, navigation. Mapping is always active automatically.",
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {
                        "type": "string",
                        "enum": ["start_mapping", "stop_mapping",
                                 "tag_place", "untag_place", "list_tags",
                                 "navigate_to_tag", "navigate_to_pose",
                                 "pause_nav", "resume_nav", "stop_nav"],
                        "description": "Action to perform",
                    },
                    "name":        {"type": "string", "description": "POI tag name"},
                    "description": {"type": "string", "description": "POI description"},
                    "tag_name":    {"type": "string", "description": "Target tag name for navigation"},
                    "x":           {"type": "number", "description": "Target X coordinate (meters)"},
                    "y":           {"type": "number", "description": "Target Y coordinate (meters)"},
                    "yaw":         {"type": "number", "description": "Target yaw (radians)"},
                    "map_name":    {"type": "string", "description": "Map name (for start/stop mapping)"},
                },
                "required": ["action"],
                "x-action-params": {
                    "start_mapping":    {"params": ["map_name"],            "description": "Start SLAM mapping (optional map_name)"},
                    "stop_mapping":     {"params": [],                      "description": "Stop mapping and save the map"},
                    "tag_place":        {"params": ["name", "description"], "description": "Tag current position with a name"},
                    "untag_place":      {"params": ["name"],               "description": "Remove a place tag"},
                    "list_tags":        {"params": [],                     "description": "List all tags with relative positions"},
                    "navigate_to_tag":  {"params": ["tag_name"],           "description": "Navigate to a tagged place"},
                    "navigate_to_pose": {"params": ["x", "y", "yaw"],     "description": "Navigate to coordinates (advanced)"},
                    "pause_nav":        {"params": [],                     "description": "Pause navigation"},
                    "resume_nav":       {"params": [],                     "description": "Resume navigation"},
                    "stop_nav":         {"params": [],                     "description": "Stop and cancel navigation"},
                },
            },
        }

    def _on_localized(self):
        """Called by _SlamInfoNode when SLAM reports localization success.
        Automatically transitions to mapping mode to keep building the map."""
        if self._node._map_status == "mapping":
            return
        print("[SpatialPlugin] _on_localized: SLAM localized, starting mapping to extend map", flush=True)
        code, resp = self._client.StartMapping()
        print(f"[SpatialPlugin] StartMapping after localization → code={code}", flush=True)
        if code == 0:
            self._node.set_map_status("mapping")
            if not self._node._active_map:
                map_name = f"map_{int(time.time())}"
                pcd_path = f"{self._map_dir}/{map_name}.pcd"
                self._node.set_active_map(map_name)
                self._db.add_map(map_name, pcd_path)
                print(f"[SpatialPlugin] Created new map for post-localization mapping: {map_name}", flush=True)
        else:
            print(f"[SpatialPlugin] StartMapping failed after localization: code={code}", flush=True)

    def _on_cloud_timeout(self):
        """Called by watchdog when no point cloud received for WATCHDOG_TIMEOUT seconds.
        Re-subscribe DDS topics (subscription may have been lost)."""
        print("[SpatialPlugin] _on_cloud_timeout: re-subscribing DDS mapping topics", flush=True)
        try:
            from unitree_sdk2py.core.channel import ChannelSubscriber
            from unitree_sdk2py.idl.sensor_msgs.msg.dds_ import PointCloud2_
            # Re-create subscriptions
            sub1 = ChannelSubscriber("rt/unitree/slam_mapping/points", PointCloud2_)
            sub1.Init(self._node._on_mapping_cloud, 10)
            sub2 = ChannelSubscriber("rt/unitree/slam_relocation/points", PointCloud2_)
            sub2.Init(self._node._on_mapping_cloud, 10)
            # Replace old refs
            self._node._dds_subs = [s for s in self._node._dds_subs
                                    if not hasattr(s, '_topic') or 'points' not in str(getattr(s, '_topic', ''))]
            self._node._dds_subs.extend([sub1, sub2])
            print("[SpatialPlugin] DDS re-subscribed successfully", flush=True)
        except Exception as e:
            print(f"[SpatialPlugin] DDS re-subscribe failed: {e}", flush=True)

    def start(self) -> None:
        """Auto-start mapping on plugin start. Always mapping, always observing."""
        print("[SpatialPlugin] start() called, scheduling auto-mapping in 3s", flush=True)
        def _auto_start():
            time.sleep(3)
            try:
                self._do_auto_mapping()
            except Exception as e:
                print(f"[SpatialPlugin] auto-mapping failed: {e}")
                import traceback
                traceback.print_exc()
        threading.Thread(target=_auto_start, daemon=True).start()

    def stop(self) -> None:
        self._node._stop_save_timer()

    def _do_auto_mapping(self) -> dict:
        """Auto-mapping logic based on current SLAM state:
        - mapping (code 3104 = already mapping) → just ensure active_map is set, don't interrupt
        - localized → StartMapping to extend
        - idle → fingerprint match, then StartMapping
        """
        with self._node._lock:
            status = self._node._map_status
        print(f"[SpatialPlugin] _do_auto_mapping: current status={status}", flush=True)

        if status == "mapping":
            # SLAM is already in mapping mode (started automatically by robot)
            # Don't call StartMapping again (code 3104) — just ensure we track it
            print("[SpatialPlugin] SLAM already mapping, just ensuring active_map is set", flush=True)
            if not self._node._active_map:
                map_name = f"map_{int(time.time())}"
                pcd_path = f"{self._map_dir}/{map_name}.pcd"
                self._node.set_active_map(map_name)
                self._db.add_map(map_name, pcd_path)
                print(f"[SpatialPlugin] Created map entry: {map_name}", flush=True)
            return {"status": "already_mapping", "map_name": self._node._active_map}

        if status == "localized":
            # SLAM finished relocation but not mapping — start mapping to extend
            print("[SpatialPlugin] SLAM localized, calling StartMapping to extend", flush=True)
            code, resp = self._client.StartMapping()
            print(f"[SpatialPlugin] StartMapping() → code={code}", flush=True)
            if code == 0:
                self._node.set_map_status("mapping")
                if not self._node._active_map:
                    map_name = f"map_{int(time.time())}"
                    pcd_path = f"{self._map_dir}/{map_name}.pcd"
                    self._node.set_active_map(map_name)
                    self._db.add_map(map_name, pcd_path)
                    print(f"[SpatialPlugin] Created map entry: {map_name}", flush=True)
                return {"status": "continued", "map_name": self._node._active_map}
            print(f"[SpatialPlugin] StartMapping failed: code={code}, trying fingerprint path", flush=True)
            # Fall through to fingerprint path if StartMapping fails

        # SLAM not localized (idle) — try fingerprint matching
        recent_cloud = self._node.get_recent_cloud()
        cloud_size = len(recent_cloud) if recent_cloud is not None else 0
        print(f"[SpatialPlugin] Fingerprint path: recent_cloud={cloud_size} points")

        if recent_cloud is not None and cloud_size >= 100:
            current_sc = self._sc_mgr.make_scan_context(recent_cloud)
            match = self._sc_mgr.query(current_sc)
            print(f"[SpatialPlugin] Fingerprint query: {match}")

            if match:
                map_name = match["map_name"]
                map_info = self._db.get_map(map_name)
                if map_info:
                    pcd_path = map_info["pcd_path"]
                    print(f"[SpatialPlugin] Matched map '{map_name}', trying InitPose + StartMapping")
                    code, resp = self._client.InitPose(0, 0, 0, 0, 0, 0, 1.0, pcd_path)
                    print(f"[SpatialPlugin] InitPose → code={code}")
                    if code == 0:
                        code2, _ = self._client.StartMapping()
                        print(f"[SpatialPlugin] StartMapping after InitPose → code={code2}")
                        if code2 == 0:
                            self._node.load_pcd_to_buffer(pcd_path)
                            self._node.set_map_status("mapping")
                            self._node.set_active_map(map_name)
                            self._db.set_last_used_map(map_name)
                            return {"status": "found", "map_name": map_name, "pose": match["pose"]}

        # No match or no data — start fresh
        print("[SpatialPlugin] No match, starting new map")
        code, resp = self._client.StartMapping()
        print(f"[SpatialPlugin] StartMapping() → code={code}")
        if code == 0:
            map_name = f"map_{int(time.time())}"
            pcd_path = f"{self._map_dir}/{map_name}.pcd"
            self._node.clear_map_buffer()
            self._node.set_map_status("mapping")
            self._node.set_active_map(map_name)
            self._db.add_map(map_name, pcd_path)
            print(f"[SpatialPlugin] Started new map: {map_name}")
            return {"status": "new", "map_name": map_name}
        return {"error": f"StartMapping failed, code={code}"}

    def dispatch(self, action: str, args: dict) -> dict | None:
        if action == "start":
            return {"state": "ready"}
        if action == "stop":
            return {"state": "idle"}
        if action == "info":
            tool_name = args.get('_tool_name', '')
            if tool_name == 'mapping':
                return {"state": "running", "topic_out": [{"topic": self._mapping_topic, "format": "sensor/mapping"}]}
            return {"state": "running", "topic_out": [{"topic": self._pos_tag_topic, "format": "data/json"}]}
        if action == "start_mapping":
            map_name = args.get("map_name", f"map_{int(time.time())}")
            code, resp = self._client.StartMapping()
            if code == 0:
                pcd_path = f"{self._map_dir}/{map_name}.pcd"
                self._node.clear_map_buffer()
                self._node.set_map_status("mapping")
                self._node.set_active_map(map_name)
                self._db.add_map(map_name, pcd_path)
                return {"status": "mapping", "map_name": map_name}
            return {"error": f"StartMapping failed, code={code}", "response": resp}

        elif action == "stop_mapping":
            active_map = self._node._active_map
            if not active_map:
                return {"error": "No active map"}
            pcd_path = f"{self._map_dir}/{active_map}.pcd"
            # Save current buffer to PCD before stopping
            self._node._maybe_save_pcd()
            code, resp = self._client.StopMapping(pcd_path)
            self._node.set_map_status("idle")
            if code == 0:
                return {"status": "stopped", "map_name": active_map, "pcd_path": pcd_path}
            return {"error": f"StopMapping failed, code={code}", "response": resp}

        elif action == "tag_place":
            name = args.get("name", "")
            if not name:
                return {"error": "name is required"}
            pose = self._node.get_pose()
            if not pose:
                return {"error": "No current pose available (SLAM not running?)"}
            active_map = self._node._active_map or "default"
            desc = args.get("description", "")
            self._db.add_poi(name, pose["x"], pose["y"], pose["yaw"], active_map, desc)
            return {"status": "tagged", "name": name, "pose": pose, "map": active_map}

        elif action == "untag_place":
            name = args.get("name", "")
            active_map = self._node._active_map or "default"
            if self._db.delete_poi(name, active_map):
                return {"status": "deleted", "name": name}
            return {"error": f"Tag '{name}' not found in map '{active_map}'"}

        elif action == "list_tags":
            active_map = self._node._active_map or "default"
            pois = self._db.list_pois(active_map)
            pose = self._node.get_pose()
            result = []
            for poi in pois:
                entry = {"name": poi["name"], "description": poi["description"], "x": poi["x"], "y": poi["y"]}
                if pose:
                    dx = poi["x"] - pose["x"]
                    dy = poi["y"] - pose["y"]
                    dist = math.sqrt(dx * dx + dy * dy)
                    cos_yaw = math.cos(-pose["yaw"])
                    sin_yaw = math.sin(-pose["yaw"])
                    rx = dx * cos_yaw - dy * sin_yaw
                    ry = dx * sin_yaw + dy * cos_yaw
                    entry["distance"] = round(dist, 2)
                    entry["bearing"] = _bearing_label(rx, ry)
                result.append(entry)
            return {"tags": result, "map": active_map}

        elif action == "navigate_to_tag":
            tag_name = args.get("tag_name", "")
            active_map = self._node._active_map or "default"
            poi = self._db.find_poi(tag_name, active_map)
            if not poi:
                return {"error": f"Tag '{tag_name}' not found", "available": [p["name"] for p in self._db.list_pois(active_map)]}
            yaw = poi.get("yaw", 0)

            # Route through SmartMotion safety harness
            if self._smart_motion:
                result = self._smart_motion.navigate_to(poi["x"], poi["y"], yaw, tag_name)
                if "error" not in result:
                    self._node.set_nav_target(tag_name)
                return result

            # Fallback: direct control
            q_z = math.sin(yaw / 2)
            q_w = math.cos(yaw / 2)
            code, resp = self._client.NavigateTo(poi["x"], poi["y"], 0, 0, 0, q_z, q_w)
            if code == 0:
                self._node.set_nav_target(tag_name)
                return {"status": "navigating", "target": tag_name, "pose": {"x": poi["x"], "y": poi["y"], "yaw": yaw}}
            return {"error": f"NavigateTo failed, code={code}", "response": resp}

        elif action == "navigate_to_pose":
            x = float(args.get("x", 0))
            y = float(args.get("y", 0))
            yaw = float(args.get("yaw", 0))

            # Route through SmartMotion safety harness
            if self._smart_motion:
                result = self._smart_motion.navigate_to(x, y, yaw)
                if "error" not in result:
                    self._node.set_nav_target(f"({x:.1f}, {y:.1f})")
                return result

            # Fallback: direct control
            q_z = math.sin(yaw / 2)
            q_w = math.cos(yaw / 2)
            code, resp = self._client.NavigateTo(x, y, 0, 0, 0, q_z, q_w)
            if code == 0:
                self._node.set_nav_target(f"({x:.1f}, {y:.1f})")
                return {"status": "navigating", "target_pose": {"x": x, "y": y, "yaw": yaw}}
            return {"error": f"NavigateTo failed, code={code}", "response": resp}

        elif action == "pause_nav":
            if self._smart_motion:
                return self._smart_motion.pause_nav()
            code, resp = self._client.PauseNav()
            return {"status": "paused"} if code == 0 else {"error": f"PauseNav failed, code={code}"}

        elif action == "resume_nav":
            if self._smart_motion:
                return self._smart_motion.resume_nav()
            code, resp = self._client.ResumeNav()
            return {"status": "resumed"} if code == 0 else {"error": f"ResumeNav failed, code={code}"}

        elif action == "stop_nav":
            if self._smart_motion:
                result = self._smart_motion.stop_nav()
                self._node.set_nav_target(None)
                return result
            self._client.PauseNav()
            self._node.set_nav_target(None)
            return {"status": "stopped"}

        return None


# ── MotionSwitcherPlugin (actuator) ──────────────────────────────────────────

class MotionSwitcherPlugin:
    PREFIX = "motion_switcher"

    def __init__(self, plugin_config: dict, namespace: str, executor, msc_client):
        self._client = msc_client

    def get_tool(self) -> dict:
        return {
            "name": "motion_switcher",
            "type": "actuator",
            "multiInstance": False,
            "description": (
                "G1 high-level motion mode switcher — check current mode, select mode "
                "(ai/normal/advanced), or release mode for low-level control. "
                "Modes: ai=AI locomotion, normal=normal locomotion, advanced=advanced mode. "
                "release frees control for lowcmd/dex3."
            ),
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {
                        "type": "string",
                        "enum": ["check", "select", "release", "set_silent", "get_silent"],
                        "description": "Action to perform",
                    },
                    "mode": {
                        "type": "string",
                        "enum": ["ai", "normal", "advanced"],
                        "description": "Mode to select (for 'select' action)",
                    },
                    "silent": {
                        "type": "boolean",
                        "description": "Silent flag (for 'set_silent' action)",
                    },
                },
                "required": ["action"],
                "x-action-params": {
                    "check":      {"params": [],         "description": "Check current motion mode"},
                    "select":     {"params": ["mode"],   "description": "Select a motion mode (ai/normal/advanced)"},
                    "release":    {"params": [],         "description": "Release current mode for low-level control"},
                    "set_silent": {"params": ["silent"], "description": "Set silent mode on/off"},
                    "get_silent": {"params": [],         "description": "Get current silent mode status"},
                },
            },
        }

    def start(self) -> None:
        pass

    def stop(self) -> None:
        pass

    def dispatch(self, action: str, args: dict) -> dict | None:
        if action == "start":
            return {"state": "ready"}
        if action == "stop":
            return {"state": "idle"}
        if action == "check":
            if code != 0:
                return {"error": f"CheckMode failed, code={code}"}
            return {"mode": result}
        elif action == "select":
            mode = args.get("mode", "normal")
            code, _ = self._client.SelectMode(mode)
            if code != 0:
                return {"error": f"SelectMode failed, code={code}"}
            return {"ret": code, "selected": mode}
        elif action == "release":
            code, _ = self._client.ReleaseMode()
            if code != 0:
                return {"error": f"ReleaseMode failed, code={code}"}
            return {"ret": code, "released": True}
        elif action == "set_silent":
            # SetSilent/GetSilent may not be exposed by the client—fallback gracefully
            silent = bool(args.get("silent", True))
            try:
                code, _ = self._client.SetSilent()
                return {"ret": code, "silent": silent}
            except Exception as e:
                return {"error": str(e)}
        elif action == "get_silent":
            try:
                code, _ = self._client.GetSilent()
                return {"ret": code}
            except Exception as e:
                return {"error": str(e)}
        return None


# ── RealSensePlugin (sensor) ─────────────────────────────────────────────────

RS_COLOR_W, RS_COLOR_H, RS_COLOR_FPS = 1920, 1080, 15
RS_DEPTH_W, RS_DEPTH_H, RS_DEPTH_FPS = 640, 480, 15
RS_JPEG_QUALITY  = 80
RS_DIST_INTERVAL = 0.1  # 10 Hz for distance JSON


class RealSensePlugin:
    PREFIX = "camera"

    def __init__(self, plugin_config: dict, namespace: str, executor):
        self._namespace   = namespace
        self._color_topic = f"/{namespace}/camera/rgb"
        self._depth_topic = f"/{namespace}/camera/depth"
        self._dist_topic  = f"/{namespace}/camera/distance"
        self._proc = None

    def get_tools(self) -> list:
        return [self._color_tool(), self._depth_tool(), self._dist_tool()]

    def _color_tool(self) -> dict:
        return {
            "name": "camera_rgb",
            "type": "sensor",
            "multiInstance": False,
            "description": f"RealSense color camera — {RS_COLOR_W}x{RS_COLOR_H} JPEG @ {RS_COLOR_FPS}fps. Publishes CompressedImage to {self._color_topic}",
            "inputSchema": {"type": "object", "properties": {}},
            "topic_out": [{"topic": self._color_topic, "format": "image/jpeg"}],
        }

    def _depth_tool(self) -> dict:
        return {
            "name": "camera_depth",
            "type": "sensor",
            "multiInstance": False,
            "description": f"RealSense depth camera — {RS_DEPTH_W}x{RS_DEPTH_H} 16UC1 (z16, mm) @ {RS_DEPTH_FPS}fps. Publishes to {self._depth_topic}",
            "inputSchema": {"type": "object", "properties": {}},
            "topic_out": [{"topic": self._depth_topic, "format": "image/depth-z16"}],
        }

    def _dist_tool(self) -> dict:
        return {
            "name": "camera_distance",
            "type": "sensor",
            "multiInstance": False,
            "description": f"RealSense center-point distance(m) + fps. Publishes at 10Hz to {self._dist_topic}",
            "inputSchema": {"type": "object", "properties": {}},
            "topic_out": [{"topic": self._dist_topic, "format": "data/json"}],
        }

    def start(self) -> None:
        import multiprocessing as mp
        if self._proc is not None and self._proc.is_alive():
            return
        ctx = mp.get_context("spawn")
        self._proc = ctx.Process(
            target=run_realsense_process, args=(self._namespace,),
            name="realsense", daemon=True,
        )
        self._proc.start()
        print(f"[bundle] RealSense capture forked → pid={self._proc.pid}")

    def stop(self) -> None:
        if self._proc is not None and self._proc.is_alive():
            self._proc.terminate()
            self._proc.join(timeout=3.0)
            if self._proc.is_alive():
                self._proc.kill()
                self._proc.join(timeout=2.0)
        self._proc = None

    def dispatch(self, action: str, args: dict) -> dict | None:
        if action == "start":
            return {"state": "running"}
        if action == "stop":
            return {"state": "idle"}
        if action == "info":
            tool_name = args.get('_tool_name', '')
            if tool_name == 'camera_depth':
                return {"state": "running", "topic_out": [{"topic": self._depth_topic, "format": "image/depth-z16"}]}
            if tool_name == 'camera_distance':
                return {"state": "running", "topic_out": [{"topic": self._dist_topic, "format": "data/json"}]}
            return {"state": "running", "topic_out": [{"topic": self._color_topic, "format": "image/jpeg"}]}
        return None


def run_realsense_process(namespace: str) -> None:
    """RealSense subprocess entry — independent GIL for full 1080p@15fps throughput.

    All heavy imports (cv2, numpy, pyrealsense2, sensor_msgs) happen here
    so the main process is not affected if these packages are missing.
    """
    import os
    import cv2
    import numpy as np
    import pyrealsense2 as rs
    import rclpy
    from rclpy.node import Node as _Node
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
    from std_msgs.msg import String as _String
    from sensor_msgs.msg import Image, CompressedImage

    _QOS = QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        durability=DurabilityPolicy.VOLATILE,
    )

    class _RealSenseNode(_Node):
        def __init__(self, color_topic, depth_topic, dist_topic):
            super().__init__("g1_realsense")
            self._color_pub = self.create_publisher(CompressedImage, color_topic, _QOS)
            self._depth_pub = self.create_publisher(Image, depth_topic, _QOS)
            self._dist_pub  = self.create_publisher(_String, dist_topic, _QOS)

            self._pipeline = None
            self._last_ts        = 0.0
            self._last_dist_time = 0.0

            self._depth_q = queue.Queue(maxsize=1)
            self._depth_worker = None

            self._color_q = queue.Queue(maxsize=1)
            self._color_worker = None

            self._worker_stop = threading.Event()

            self.get_logger().info(
                f"RealSenseNode ready — color:{color_topic} depth:{depth_topic} dist:{dist_topic}"
            )

        def start_capture(self):
            if self._pipeline is not None:
                return
            ctx = rs.context()
            devs = ctx.query_devices()
            if len(devs) == 0:
                self.get_logger().warn("RealSenseNode: no device connected")
                return
            serial = devs[0].get_info(rs.camera_info.serial_number)

            pipeline = rs.pipeline()
            config = rs.config()
            config.enable_device(serial)
            config.enable_stream(rs.stream.depth, RS_DEPTH_W, RS_DEPTH_H, rs.format.z16, RS_DEPTH_FPS)
            config.enable_stream(rs.stream.color, RS_COLOR_W, RS_COLOR_H, rs.format.bgr8, RS_COLOR_FPS)

            self._worker_stop.clear()
            self._depth_worker = threading.Thread(target=self._depth_loop, name="rs_depth", daemon=True)
            self._depth_worker.start()
            self._color_worker = threading.Thread(target=self._color_loop, name="rs_color", daemon=True)
            self._color_worker.start()

            pipeline.start(config, self._on_frame)
            self._pipeline = pipeline
            self.get_logger().info(f"RealSense capture started — device {serial}")

        def stop_capture(self):
            if self._pipeline is not None:
                try:
                    self._pipeline.stop()
                except Exception:
                    pass
                self._pipeline = None
            self._worker_stop.set()
            if self._depth_worker is not None:
                self._depth_worker.join(timeout=2.0)
                self._depth_worker = None
            if self._color_worker is not None:
                self._color_worker.join(timeout=2.0)
                self._color_worker = None
            self.get_logger().info("RealSense capture stopped")

        def _depth_loop(self):
            while not self._worker_stop.is_set():
                try:
                    depth_np, stamp = self._depth_q.get(timeout=0.5)
                except queue.Empty:
                    continue
                try:
                    msg = Image()
                    msg.header.stamp = stamp
                    msg.header.frame_id = "camera_depth_optical_frame"
                    msg.height = depth_np.shape[0]
                    msg.width  = depth_np.shape[1]
                    msg.encoding = "16UC1"
                    msg.is_bigendian = 0
                    msg.step = depth_np.shape[1] * 2
                    msg.data = depth_np.tobytes()
                    self._depth_pub.publish(msg)
                except Exception as e:
                    self.get_logger().error(f"[realsense] depth publish error: {e}")

        def _color_loop(self):
            while not self._worker_stop.is_set():
                try:
                    color_np, stamp = self._color_q.get(timeout=0.5)
                except queue.Empty:
                    continue
                try:
                    ok, jpg = cv2.imencode(".jpg", color_np, [cv2.IMWRITE_JPEG_QUALITY, RS_JPEG_QUALITY])
                    if ok:
                        cmsg = CompressedImage()
                        cmsg.header.stamp = stamp
                        cmsg.header.frame_id = "camera_color_optical_frame"
                        cmsg.format = "jpeg"
                        cmsg.data = jpg.tobytes()
                        self._color_pub.publish(cmsg)
                except Exception as e:
                    self.get_logger().error(f"[realsense] color publish error: {e}")

        def _on_frame(self, frame):
            try:
                if not frame.is_frameset():
                    return
                fs = frame.as_frameset()
                color_frame = fs.get_color_frame()
                depth_frame = fs.get_depth_frame()
                stamp = self.get_clock().now().to_msg()

                if color_frame:
                    color_np = np.asanyarray(color_frame.get_data())
                    try:
                        self._color_q.get_nowait()
                    except queue.Empty:
                        pass
                    self._color_q.put((color_np, stamp))

                dist = 0.0
                if depth_frame:
                    dist = depth_frame.get_distance(
                        depth_frame.get_width() // 2,
                        depth_frame.get_height() // 2,
                    )
                    depth_np = np.array(depth_frame.get_data())
                    try:
                        self._depth_q.get_nowait()
                    except queue.Empty:
                        pass
                    self._depth_q.put((depth_np, stamp))

                now = time.monotonic()
                if now - self._last_dist_time >= RS_DIST_INTERVAL:
                    fps = 1.0 / (now - self._last_ts) if self._last_ts > 0 else 0.0
                    self._last_dist_time = now
                    self._last_ts = now
                    out = _String()
                    out.data = json.dumps({"distance_m": round(dist, 3), "fps": round(fps, 1)})
                    self._dist_pub.publish(out)
                else:
                    self._last_ts = now
            except Exception as e:
                self.get_logger().error(f"[realsense] frame error: {e}")

    color_topic = f"/{namespace}/camera/rgb"
    depth_topic = f"/{namespace}/camera/depth"
    dist_topic  = f"/{namespace}/camera/distance"

    rclpy.init()
    node = _RealSenseNode(color_topic, depth_topic, dist_topic)
    node.start_capture()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    print(f"[realsense-proc] started — {color_topic} (pid={os.getpid()})", flush=True)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    except Exception as e:
        # SIGTERM shuts rclpy down asynchronously. Its executor reports that
        # expected exit as ExternalShutdownException on Humble.
        if rclpy.ok():
            print(f"[realsense-proc] executor stopped: {e}", flush=True)
    finally:
        node.stop_capture()
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception:
                pass
        print("[realsense-proc] stopped", flush=True)
