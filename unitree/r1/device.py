#!/usr/bin/env python3
"""
drivers/unitree/r1/device.py — Unitree R1-EDU 设备插件。

设计原则：
  - 一个设备 = 一个 tool，tool schema 含 type 字段（sensor / actuator）
  - sensor：只读声明，驱动启动时自动 start，数据通过 ROS2 topic 输出
  - actuator：单 tool + action 参数分发操作
  - start/stop 不暴露给 LLM，由驱动生命周期管理

插件：
  MicPlugin          (sensor)    — UDP multicast → ROS2 topic
  NativeTtsPlugin    (actuator)  — R1 内置 TTS + 音量控制
  SpeakerPlugin      (actuator)  — PCM 音频流播放
  LedPlugin          (actuator)  — LED 灯带控制
  LocoStatePlugin    (sensor)    — DDS OdomModeState → ROS2 topic
  LocoPlugin         (actuator)  — 运动控制 (H2 LocoClient)
  StatePlugin        (sensor)    — DDS LowState → IMU/battery/joints ROS2 topic
  AsrPlugin          (sensor)    — DDS ASR results → ROS2 topic
  CameraPlugin       (sensor)    — GStreamer H.264 RTP → MJPEG ROS2 topic
"""

import json
import queue
import socket
import struct
import subprocess
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import String
from audio_msgs.msg import AudioChunk

from unitree_sdk2py.g1.audio.g1_audio_client import AudioClient

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
        super().__init__("r1_mic")
        self._topic  = topic
        self._pub    = self.create_publisher(AudioChunk, topic, _LOW_LAT_QOS)
        self._sock:   socket.socket | None = None
        self._thread: threading.Thread | None = None
        self.state   = "idle"
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
        self.state   = "running"
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
                data = self._sock.recv(4096)
                buf.extend(data)
            except socket.timeout:
                continue
            except OSError:
                break
            while len(buf) >= CHUNK_BYTES:
                chunk = bytes(buf[:CHUNK_BYTES])
                del buf[:CHUNK_BYTES]
                msg = AudioChunk()
                msg.format = "pcm_16k_16bit_mono"
                msg.data = list(chunk)
                self._pub.publish(msg)


class MicPlugin:
    PREFIX = "mic"

    def __init__(self, plugin_config: dict, namespace: str, executor):
        self._topic = f"/{namespace}/mic/audio"
        self._node = _MicNode(self._topic)
        executor.add_node(self._node)

    def get_tool(self) -> dict:
        return {
            "name": "mic",
            "type": "sensor",
            "multiInstance": False,
            "description": f"R1 4-mic array — noise-reduced PCM 16kHz/16bit/mono. Publishes to {self._topic}",
            "inputSchema": {"type": "object", "properties": {}},
            "topic_out": [{"topic": self._topic, "format": "audio/pcm-16k"}],
        }

    def start(self) -> None:
        self._node.start_capture()

    def stop(self) -> None:
        self._node.stop_capture()

    def dispatch(self, action: str, args: dict) -> dict | None:
        if action == "info":
            return {"state": self._node.state, "topic_out": [{"topic": self._topic, "format": "audio/pcm-16k"}]}
        return None


# ── NativeTtsPlugin (actuator) ────────────────────────────────────────────────

class NativeTtsPlugin:
    PREFIX = "tts"

    def __init__(self, plugin_config: dict, namespace: str, executor, audio_client: AudioClient):
        self._client = audio_client

    def get_tool(self) -> dict:
        return {
            "name": "tts",
            "type": "actuator",
            "multiInstance": False,
            "description": "R1 on-board TTS engine — synthesize text to robot speech, control volume",
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {
                        "type": "string",
                        "enum": ["speak", "get_volume", "set_volume"],
                        "description": "Action to perform",
                    },
                    "text":   {"type": "string",  "description": "Text to speak"},
                    "voice":  {"type": "integer", "description": "Voice ID: 0=Chinese, 1=English"},
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

APP_NAME = "r1_speaker"


class _SpeakerNode(Node):
    PREFILL = 20      # buffer 20 chunks (~2s) before starting playback
    MERGE_BYTES = 64000  # merge into ~2s blocks before calling PlayStream

    def __init__(self, audio_client: AudioClient):
        super().__init__("r1_speaker")
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
        self.get_logger().info("SpeakerNode ready")

    def start_play(self, topic: str) -> str:
        if self._sub is not None:
            if self._topic == topic:
                return self._topic
            self.stop_play()
        self._topic = topic
        self._sub = self.create_subscription(
            AudioChunk, topic, self._on_chunk, _LOW_LAT_QOS,
        )
        self.state = "playing"
        self.get_logger().info(f"[speaker] subscribed to {topic}")
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
        if self._drain_thread is not None:
            self._drain_thread.join(timeout=2)
            self._drain_thread = None
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

    def _on_chunk(self, msg: AudioChunk) -> None:
        pcm = bytes(msg.data)
        self._idx += 1
        self._buf.put(pcm)
        self._last_chunk_time = time.monotonic()
        if not self._draining.is_set() and self._buf.qsize() >= self.PREFILL:
            self._start_drain()
        elif not self._draining.is_set() and self._flush_timer is None:
            self._flush_timer = self.create_timer(0.5, self._check_flush)

    def _start_drain(self) -> None:
        if self._flush_timer is not None:
            self._flush_timer.cancel()
            self.destroy_timer(self._flush_timer)
            self._flush_timer = None
        self._draining.set()
        self._drain_thread = threading.Thread(target=self._drain, daemon=True)
        self._drain_thread.start()

    def _check_flush(self) -> None:
        if self._flush_timer is not None:
            self._flush_timer.cancel()
            self.destroy_timer(self._flush_timer)
            self._flush_timer = None
        if not self._draining.is_set() and not self._buf.empty():
            idle = time.monotonic() - self._last_chunk_time
            if idle >= 0.3:
                self._start_drain()

    def _drain(self) -> None:
        play_idx = 0
        merged = b''
        while self._draining.is_set():
            try:
                pcm = self._buf.get(timeout=0.3)
                merged += pcm
            except queue.Empty:
                if merged:
                    play_idx += 1
                    self._play_merged(merged, play_idx)
                    merged = b''
                else:
                    break
                continue
            if len(merged) >= self.MERGE_BYTES:
                play_idx += 1
                self._play_merged(merged, play_idx)
                merged = b''
        if merged:
            play_idx += 1
            self._play_merged(merged, play_idx)
        self._draining.clear()

    def _play_merged(self, pcm: bytes, idx: int) -> None:
        duration = len(pcm) / 32000  # seconds (16kHz, 16-bit mono)
        t0 = time.monotonic()
        try:
            code, data = self._client.PlayStream(APP_NAME, str(idx), pcm)
            if code != 0:
                self.get_logger().error(f"[speaker] PlayStream error code={code}")
        except Exception as e:
            self.get_logger().error(f"[speaker] PlayStream error: {e}")
        elapsed = time.monotonic() - t0
        remaining = duration - elapsed
        if remaining > 0:
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
            "description": "R1 speaker — subscribes to ROS2 topic and streams PCM-16k audio to robot speaker",
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
        pass

    def stop(self) -> None:
        self._node.stop_play()

    def dispatch(self, action: str, args: dict) -> dict | None:
        if action in ("start", "play"):
            topic = args.get("input_topic", "")
            if not topic:
                return {"error": "Missing input_topic"}
            topic = self._node.start_play(topic)
            return {"state": "playing", "topic": topic}
        elif action == "stop":
            self._node.stop_play()
            return {"state": "idle"}
        elif action == "info":
            return {"state": self._node.state, "topic": self._node._topic}
        return None


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
            "description": "R1 RGB LED strip control — set color or turn off",
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
    """Subscribes to DDS rt/odommodestate (IMUState_) and republishes as JSON to ROS2."""

    _ODOM_INTERVAL = 0.1  # 10 Hz throttle

    def __init__(self, odom_topic: str):
        super().__init__("r1_loco_state")
        self._odom_pub = self.create_publisher(String, odom_topic, _LOW_LAT_QOS)
        self._last_state: dict = {}
        self._lock = threading.Lock()
        self._last_odom_time: float = 0.0

        try:
            from unitree_sdk2py.core.channel import ChannelSubscriber
            from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_
            self._odom_sub = ChannelSubscriber("rt/odommodestate", SportModeState_)
            self._odom_sub.Init(self._on_odom, 10)
            self.get_logger().info(f"LocoStateNode subscribed rt/odommodestate → {odom_topic}")
        except Exception as e:
            self.get_logger().warn(f"LocoStateNode: failed to subscribe rt/odommodestate: {e}")

    def _on_odom(self, msg) -> None:
        now = time.monotonic()
        if now - self._last_odom_time < self._ODOM_INTERVAL:
            return
        self._last_odom_time = now

        try:
            imu = msg.imu_state
            state = {
                "mode":          msg.mode,
                "gait_type":     msg.gait_type,
                "body_height":   msg.body_height,
                "position":      list(msg.position),
                "velocity":      list(msg.velocity),
                "yaw_speed":     msg.yaw_speed,
                "imu": {
                    "quaternion":    list(imu.quaternion),
                    "gyroscope":     list(imu.gyroscope),
                    "accelerometer": list(imu.accelerometer),
                    "rpy":           list(imu.rpy),
                },
            }
        except AttributeError:
            # Fallback if message type differs
            state = {"raw": str(msg)}

        with self._lock:
            self._last_state = state
        out = String()
        out.data = json.dumps(state)
        self._odom_pub.publish(out)


class LocoStatePlugin:
    PREFIX = "loco_state"

    def __init__(self, plugin_config: dict, namespace: str, executor):
        self._odom_topic = f"/{namespace}/loco/state"
        self._node = _LocoStateNode(self._odom_topic)
        executor.add_node(self._node)

    def get_tool(self) -> dict:
        return {
            "name": "loco_state",
            "type": "sensor",
            "multiInstance": False,
            "description": f"R1 locomotion state (always active) — mode, velocity, position, body_height, IMU. Publishes at 10Hz to {self._odom_topic}",
            "inputSchema": {"type": "object", "properties": {}},
            "topic_out": [{"topic": self._odom_topic, "format": "data/json"}],
        }

    def start(self) -> None:
        pass

    def stop(self) -> None:
        pass

    def dispatch(self, action: str, args: dict) -> dict | None:
        if action == "info":
            return {"state": "running", "topic_out": [{"topic": self._odom_topic, "format": "data/json"}]}
        return None


# ── LocoPlugin (actuator) ────────────────────────────────────────────────────

class LocoPlugin:
    """R1 locomotion control via H2 LocoClient RPC.

    FSM IDs: 0=zero_torque, 1=damp, 4=locked_stand, 811=walk/run
    """
    PREFIX = "loco"

    def __init__(self, plugin_config: dict, namespace: str, executor, loco_client):
        self._client = loco_client
        self._namespace = namespace
        self._move_timer: threading.Timer | None = None

    def get_tools(self) -> list:
        return [self._loco_tool(), self._switch_mode_tool(), self._switch_mode_expert_tool()]

    def _loco_tool(self) -> dict:
        return {
            "name": "loco",
            "type": "actuator",
            "multiInstance": False,
            "description": "R1 locomotion control — move, stop, set height, wave/shake hand via SetTaskId",
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {
                        "type": "string",
                        "enum": ["move", "stop_move", "set_stand_height", "wave_hand", "shake_hand"],
                        "description": "Action to perform",
                    },
                    "vx":         {"type": "number", "description": "Forward velocity m/s [-1, 1]"},
                    "vy":         {"type": "number", "description": "Lateral velocity m/s [-1, 1]"},
                    "vyaw":       {"type": "number", "description": "Yaw rotation rad/s [-2, 2]"},
                    "duration":   {"type": "number", "description": "Move duration in seconds. -1 = move until explicit stop (default -1)"},
                    "height":     {"type": "number", "description": "Normalized height 0.0-1.0"},
                    "turn":       {"type": "boolean", "description": "Turn while waving (default false)"},
                },
                "required": ["action"],
                "x-action-params": {
                    "move":             {"params": ["vx", "vy", "vyaw", "duration"], "description": "Move with specified velocities. duration>0 for timed move, -1 for continuous until stop."},
                    "stop_move":        {"params": [],                                 "description": "Stop all movement immediately"},
                    "set_stand_height": {"params": ["height"],                         "description": "Set the robot's standing height (0.0-1.0)"},
                    "wave_hand":        {"params": ["turn"],                           "description": "Perform a waving hand gesture via SetTaskId"},
                    "shake_hand":       {"params": [],                                 "description": "Perform a handshake gesture via SetTaskId"},
                },
            },
        }

    def _switch_mode_tool(self) -> dict:
        return {
            "name": "switch_mode",
            "type": "actuator",
            "multiInstance": False,
            "description": "R1 locomotion mode switch — change posture/locomotion mode by name. damp=阻尼, start=主运控, zero_torque=零力矩, stand_up=起立, squat=下蹲, high_stand=最高站, low_stand=最低站, balance_stand=平衡站立, continuous_gait=持续踏步, stop_gait=停止踏步",
            "inputSchema": {
                "type": "object",
                "properties": {
                    "mode": {
                        "type": "string",
                        "enum": ["damp", "start", "zero_torque", "stand_up", "squat",
                                 "balance_stand", "continuous_gait", "stop_gait",
                                 "high_stand", "low_stand"],
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
            "description": "R1 locomotion mode switch — directly set FSM mode ID (expert use only). IDs: 0=zero_torque, 1=damp, 4=locked_stand, 811=walk/run",
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
        if action == "info":
            return None
        if action == "move":
            vx   = max(-1.0, min(1.0, float(args.get("vx",   0))))
            vy   = max(-1.0, min(1.0, float(args.get("vy",   0))))
            vyaw = max(-2.0, min(2.0, float(args.get("vyaw", 0))))
            duration = float(args.get("duration", -1))

            if self._move_timer:
                self._move_timer.cancel()
                self._move_timer = None

            ret = self._client.Move(vx, vy, vyaw, True)

            if duration > 0:
                self._move_timer = threading.Timer(duration, self._auto_stop)
                self._move_timer.start()

            return {"ret": ret, "vx": vx, "vy": vy, "vyaw": vyaw, "duration": duration}
        elif action == "stop_move":
            if self._move_timer:
                self._move_timer.cancel()
                self._move_timer = None
            ret = self._client.StopMove()
            return {"ret": ret}
        elif action == "switch_mode":
            mode = args.get("mode", "")
            mode_dispatch = {
                "damp":            lambda: self._client.Damp(),
                "start":           lambda: self._client.Start(),
                "zero_torque":     lambda: self._client.ZeroTorque(),
                "stand_up":        lambda: self._client.StandUp(),
                "squat":           lambda: self._client.Squat(),
                "balance_stand":   lambda: self._client.BalanceStand(),
                "continuous_gait": lambda: self._client.Move(0, 0, 0, True),
                "stop_gait":       lambda: self._client.StopMove(),
                "high_stand":      lambda: self._client.HighStand(),
                "low_stand":       lambda: self._client.LowStand(),
            }
            fn = mode_dispatch.get(mode)
            if fn is None:
                return {"error": f"Unknown mode: {mode}. Available: {list(mode_dispatch.keys())}"}
            ret = fn()
            return {"ret": ret, "mode": mode}
        elif action == "switch_mode_expert":
            fid = int(args.get("fsm_id", 0))
            ret = self._client.SetFsmId(fid)
            return {"ret": ret, "fsm_id": fid}
        elif action == "set_stand_height":
            h = max(0.0, min(1.0, float(args.get("height", 0.5))))
            ret = self._client.SetStandHeight(h)
            return {"ret": ret, "height": h}
        elif action == "wave_hand":
            turn = bool(args.get("turn", False))
            ret = self._client.WaveHand(turn)
            return {"ret": ret, "turn": turn}
        elif action == "shake_hand":
            ret = self._client.ShakeHand()
            return {"ret": ret}
        return None


# ── AsrPlugin (sensor) ───────────────────────────────────────────────────────

class _AsrNode(Node):
    """Subscribes to DDS rt/audio_msg (String_) and republishes ASR results to ROS2."""

    def __init__(self, topic: str):
        super().__init__("r1_asr")
        self._topic = topic
        self._pub = self.create_publisher(String, topic, _LOW_LAT_QOS)
        self._last_index: int = -1

        try:
            from unitree_sdk2py.core.channel import ChannelSubscriber
            from unitree_sdk2py.idl.std_msgs.msg.dds_ import String_
            self._asr_sub = ChannelSubscriber("rt/audio_msg", String_)
            self._asr_sub.Init(self._on_msg, 10)
            self.get_logger().info(f"AsrNode subscribed rt/audio_msg → {topic}")
        except Exception as e:
            self.get_logger().warn(f"AsrNode: failed to subscribe rt/audio_msg: {e}")

    def _on_msg(self, msg) -> None:
        try:
            payload = json.loads(msg.data_)
        except (json.JSONDecodeError, AttributeError):
            return
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
                "R1 built-in ASR — offline speech recognition results "
                "(text, angle/DOA, confidence, speaker_id, emotion). "
                f"Publishes to {self._topic}"
            ),
            "inputSchema": {"type": "object", "properties": {}},
            "topic_out": [{"topic": self._topic, "format": "data/json"}],
        }

    def start(self) -> None:
        pass

    def stop(self) -> None:
        pass

    def dispatch(self, action: str, args: dict) -> dict | None:
        if action == "info":
            return {"state": "running", "topic_out": [{"topic": self._topic, "format": "data/json"}]}
        return None


# ── StatePlugin (sensor) ─────────────────────────────────────────────────────

class _LowStateNode(Node):
    """Subscribes to DDS rt/lowstate + rt/lf/bmsstate + rt/lf/mainboardstate and republishes to ROS2."""

    _JOINTS_INTERVAL = 0.1     # 10 Hz
    _IMU_INTERVAL    = 0.05    # 20 Hz
    _BMS_INTERVAL    = 1.0     # 1 Hz
    _MAINBOARD_INTERVAL = 2.0  # 0.5 Hz

    def __init__(self, imu_topic: str, battery_topic: str, joints_topic: str, mainboard_topic: str):
        super().__init__("r1_low_state")
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
            self._lowstate_sub = ChannelSubscriber("rt/lowstate", LowState_)
            self._lowstate_sub.Init(self._on_state, 10)
            self.get_logger().info(f"LowStateNode subscribed rt/lowstate → {imu_topic}, {joints_topic}")
        except Exception as e:
            self.get_logger().warn(f"LowStateNode: failed to subscribe rt/lowstate: {e}")

        try:
            from unitree_sdk2py.core.channel import ChannelSubscriber
            from unitree_sdk2py.idl.unitree_hg.msg.dds_ import BmsState_
            self._bms_sub = ChannelSubscriber("rt/lf/bmsstate", BmsState_)
            self._bms_sub.Init(self._on_bms, 10)
            self.get_logger().info(f"LowStateNode subscribed rt/lf/bmsstate → {battery_topic}")
        except Exception as e:
            self.get_logger().warn(f"LowStateNode: failed to subscribe rt/lf/bmsstate: {e}")

        try:
            from unitree_sdk2py.core.channel import ChannelSubscriber
            from unitree_sdk2py.idl.unitree_hg.msg.dds_ import MainBoardState_
            self._mainboard_sub = ChannelSubscriber("rt/lf/mainboardstate", MainBoardState_)
            self._mainboard_sub.Init(self._on_mainboard, 10)
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
        return [self._imu_tool(), self._battery_tool(), self._joints_tool(), self._mainboard_tool()]

    def _imu_tool(self) -> dict:
        return {
            "name": "imu",
            "type": "sensor",
            "multiInstance": False,
            "description": f"R1 IMU sensor — quaternion, gyroscope, accelerometer, rpy, temperature. Publishes to {self._imu_topic}",
            "inputSchema": {"type": "object", "properties": {}},
            "topic_out": [{"topic": self._imu_topic, "format": "data/json"}],
        }

    def _battery_tool(self) -> dict:
        return {
            "name": "battery",
            "type": "sensor",
            "multiInstance": False,
            "description": f"R1 BMS battery — SOC%, SOH%, current(mA), voltage, cell voltages, temperature, charge cycles. Publishes at 1Hz to {self._battery_topic}",
            "inputSchema": {"type": "object", "properties": {}},
            "topic_out": [{"topic": self._battery_topic, "format": "data/json"}],
        }

    def _joints_tool(self) -> dict:
        return {
            "name": "joints",
            "type": "sensor",
            "multiInstance": False,
            "description": f"R1 joint states — 35 motor slots (26 active DoF) with position(q), velocity(dq), torque(tau), temperature. Publishes at 10Hz to {self._joints_topic}",
            "inputSchema": {"type": "object", "properties": {}},
            "topic_out": [{"topic": self._joints_topic, "format": "sensor/skeleton"}],
        }

    def _mainboard_tool(self) -> dict:
        return {
            "name": "mainboard",
            "type": "sensor",
            "multiInstance": False,
            "description": f"R1 mainboard state — temperature, fan state, system values. Publishes at 0.5Hz to {self._mainboard_topic}",
            "inputSchema": {"type": "object", "properties": {}},
            "topic_out": [{"topic": self._mainboard_topic, "format": "data/json"}],
        }

    def start(self) -> None:
        pass

    def stop(self) -> None:
        pass

    def dispatch(self, action: str, args: dict) -> dict | None:
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
        return None


# ── CameraPlugin (sensor) ────────────────────────────────────────────────────

class _CameraNode(Node):
    """Receives H.264 RTP video from R1 via GStreamer and publishes MJPEG frames to ROS2."""

    def __init__(self, main_topic: str, left_topic: str, right_topic: str):
        super().__init__("r1_camera")
        from std_msgs.msg import UInt8MultiArray
        self._main_pub  = self.create_publisher(UInt8MultiArray, main_topic, _LOW_LAT_QOS)
        self._left_pub  = self.create_publisher(UInt8MultiArray, left_topic, _LOW_LAT_QOS)
        self._right_pub = self.create_publisher(UInt8MultiArray, right_topic, _LOW_LAT_QOS)
        self._procs: list[subprocess.Popen] = []
        self._threads: list[threading.Thread] = []
        self.state = "idle"
        self.get_logger().info(f"CameraNode ready — main:{main_topic} left:{left_topic} right:{right_topic}")

    def start_capture(self) -> None:
        if self.state == "running":
            return
        self.state = "running"

        # Main camera (port 5001, 1280x720)
        self._start_stream(5001, self._main_pub, "main")
        # Left stereo (port 5002, 544x448)
        self._start_stream(5002, self._left_pub, "left")
        # Right stereo (port 5003, 544x448)
        self._start_stream(5003, self._right_pub, "right")

        self.get_logger().info("Camera capture started (3 streams)")

    def _start_stream(self, port: int, publisher, name: str) -> None:
        """Launch a GStreamer subprocess to decode H.264 RTP and output JPEG frames."""
        cmd = [
            "gst-launch-1.0", "-q",
            "udpsrc", f"port={port}", "!",
            "application/x-rtp,media=video,clock-rate=90000,encoding-name=H264,payload=96", "!",
            "rtph264depay", "!",
            "avdec_h264", "!",
            "videoconvert", "!",
            "jpegenc", "quality=75", "!",
            "fdsink", "fd=1",
        ]
        try:
            proc = subprocess.Popen(
                cmd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
            )
            self._procs.append(proc)
            t = threading.Thread(target=self._read_frames, args=(proc, publisher, name), daemon=True)
            t.start()
            self._threads.append(t)
        except FileNotFoundError:
            self.get_logger().error(f"gst-launch-1.0 not found — camera stream {name} disabled")

    def _read_frames(self, proc: subprocess.Popen, publisher, name: str) -> None:
        """Read JPEG frames from GStreamer stdout and publish to ROS2.

        GStreamer jpegenc outputs a continuous stream of JPEG data.
        We detect JPEG boundaries via SOI (FFD8) and EOI (FFD9) markers.
        """
        from std_msgs.msg import UInt8MultiArray
        buf = bytearray()
        CHUNK = 65536
        while self.state == "running" and proc.poll() is None:
            data = proc.stdout.read(CHUNK)
            if not data:
                break
            buf.extend(data)
            # Extract complete JPEG frames
            while True:
                soi = buf.find(b'\xff\xd8')
                if soi == -1:
                    buf.clear()
                    break
                eoi = buf.find(b'\xff\xd9', soi + 2)
                if eoi == -1:
                    # Trim everything before SOI
                    if soi > 0:
                        del buf[:soi]
                    break
                frame = bytes(buf[soi:eoi + 2])
                del buf[:eoi + 2]
                msg = UInt8MultiArray()
                msg.data = list(frame)
                publisher.publish(msg)

    def stop_capture(self) -> None:
        self.state = "idle"
        for proc in self._procs:
            try:
                proc.terminate()
                proc.wait(timeout=3)
            except Exception:
                try:
                    proc.kill()
                except Exception:
                    pass
        self._procs.clear()
        self._threads.clear()
        self.get_logger().info("Camera capture stopped")


class CameraPlugin:
    PREFIX = "camera"

    def __init__(self, plugin_config: dict, namespace: str, executor):
        self._main_topic  = f"/{namespace}/camera/main"
        self._left_topic  = f"/{namespace}/camera/left"
        self._right_topic = f"/{namespace}/camera/right"
        self._node = _CameraNode(self._main_topic, self._left_topic, self._right_topic)
        executor.add_node(self._node)

    def get_tools(self) -> list:
        return [self._main_tool(), self._left_tool(), self._right_tool()]

    def _main_tool(self) -> dict:
        return {
            "name": "camera_main",
            "type": "sensor",
            "multiInstance": False,
            "description": f"R1 main camera (1280x720 @ 30fps) — H.264 decoded to MJPEG. Publishes to {self._main_topic}",
            "inputSchema": {"type": "object", "properties": {}},
            "topic_out": [{"topic": self._main_topic, "format": "video/mjpeg"}],
        }

    def _left_tool(self) -> dict:
        return {
            "name": "camera_left",
            "type": "sensor",
            "multiInstance": False,
            "description": f"R1 left stereo camera (544x448) — H.264 decoded to MJPEG. Publishes to {self._left_topic}",
            "inputSchema": {"type": "object", "properties": {}},
            "topic_out": [{"topic": self._left_topic, "format": "video/mjpeg"}],
        }

    def _right_tool(self) -> dict:
        return {
            "name": "camera_right",
            "type": "sensor",
            "multiInstance": False,
            "description": f"R1 right stereo camera (544x448) — H.264 decoded to MJPEG. Publishes to {self._right_topic}",
            "inputSchema": {"type": "object", "properties": {}},
            "topic_out": [{"topic": self._right_topic, "format": "video/mjpeg"}],
        }

    def start(self) -> None:
        self._node.start_capture()

    def stop(self) -> None:
        self._node.stop_capture()

    def dispatch(self, action: str, args: dict) -> dict | None:
        if action == "info":
            tool_name = args.get('_tool_name', '')
            topic_map = {
                'camera_main':  (self._main_topic,  'video/mjpeg'),
                'camera_left':  (self._left_topic,  'video/mjpeg'),
                'camera_right': (self._right_topic, 'video/mjpeg'),
            }
            if tool_name in topic_map:
                topic, fmt = topic_map[tool_name]
                return {"state": self._node.state, "topic_out": [{"topic": topic, "format": fmt}]}
            return {"state": self._node.state}
        return None
