#!/usr/bin/env python3
"""
drivers/unitree/go2/device.py — Unitree Go2 四足机器狗设备插件。

设计原则：
  - 一个设备 = 一个 tool，tool schema 含 type 字段（sensor / actuator）
  - sensor：只读声明，驱动启动时自动 start，数据通过 ROS2 topic 输出
  - actuator：单 tool + action 参数分发操作
  - start/stop 不暴露给 LLM，由驱动生命周期管理

插件：
  MicPlugin              (sensor)    — UDP multicast → ROS2 topic
  SpeakerPlugin          (actuator)  — PCM 音频流通过 DDS AudioData_ 播放
  VuiPlugin              (actuator)  — 音量/亮度控制 (VuiClient)
  LocoStatePlugin        (sensor)    — DDS SportModeState → ROS2 topic
  LocoPlugin             (actuator)  — 运动控制 (SportClient, 4 tools)
  ObstaclesAvoidPlugin   (actuator)  — 自主避障 (ObstaclesAvoidClient)
  CameraPlugin           (sensor)    — GStreamer H.264 UDP multicast → MJPEG ROS2 topic
  StatePlugin            (sensor)    — DDS LowState → IMU/battery/joints ROS2 topic
  MotionSwitcherPlugin   (actuator)  — 运控模式切换 (MotionSwitcherClient)
  AsrPlugin              (sensor)    — DDS ASR results → ROS2 topic
"""

import json
import multiprocessing
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
        super().__init__("go2_mic")
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
            "description": f"Go2 microphone — PCM 16kHz/16bit/mono. Publishes to {self._topic}",
            "inputSchema": {"type": "object", "properties": {}},
            "topic_out": [{"topic": self._topic, "format": "audio/pcm-16k"}],
        }

    def start(self) -> None:
        self._node.start_capture()

    def stop(self) -> None:
        self._node.stop_capture()

    def dispatch(self, action: str, args: dict) -> dict | None:
        if action == "start":
            return {"state": "running"}
        if action == "stop":
            return {"state": "idle"}
        if action == "info":
            return {"state": self._node.state, "topic_out": [{"topic": self._topic, "format": "audio/pcm-16k"}]}
        return None


# ── SpeakerPlugin (actuator) ─────────────────────────────────────────────────

APP_NAME = "go2_speaker"
_SPEAKER_MERGE_MS = 200  # merge PCM to at least 200ms before sending


def _speaker_worker(pcm_queue: multiprocessing.Queue, network_iface: str):
    """Subprocess: receives PCM-16k chunks, resamples to 44.1k WAV, sends via AudioHub megaphone."""
    import base64
    import io
    import wave
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelPublisher
    from unitree_sdk2py.idl.unitree_api.msg.dds_ import (
        Request_, RequestHeader_, RequestIdentity_, RequestLease_, RequestPolicy_,
    )

    ChannelFactoryInitialize(0, network_iface)
    pub = ChannelPublisher("rt/api/audiohub/request", Request_)
    pub.Init()
    time.sleep(0.3)

    def _send(api_id, parameter_dict):
        import json as _json
        identity = RequestIdentity_(id=int(time.time() * 1000) % 2147483648, api_id=api_id)
        lease = RequestLease_(id=0)
        policy = RequestPolicy_(priority=0, noreply=False)
        header = RequestHeader_(identity=identity, lease=lease, policy=policy)
        req = Request_(header=header, parameter=_json.dumps(parameter_dict), binary=[])
        pub.Write(req)

    def _send_wav(wav_data: bytes):
        b64_data = base64.b64encode(wav_data).decode('utf-8')
        chunk_size = 4096
        chunks = [b64_data[i:i+chunk_size] for i in range(0, len(b64_data), chunk_size)]
        total = len(chunks)
        for i, chunk in enumerate(chunks, 1):
            _send(4003, {
                "current_block_size": len(chunk),
                "block_content": chunk,
                "current_block_index": i,
                "total_block_number": total,
            })
            time.sleep(0.01)

    def _pcm_to_wav(pcm: bytes) -> bytes:
        """Wrap PCM-16k into WAV (no resampling — megaphone plays at 16kHz)."""
        buf = io.BytesIO()
        with wave.open(buf, 'wb') as wf:
            wf.setnchannels(1)
            wf.setsampwidth(2)
            wf.setframerate(16000)
            wf.writeframes(pcm)
        return buf.getvalue()

    # Enter megaphone mode
    _send(4001, {})
    time.sleep(0.3)
    print("[SpeakerWorker] ready, megaphone active", flush=True)

    merge_bytes = int(16000 * 2 * _SPEAKER_MERGE_MS / 1000)  # bytes for MERGE_MS at 16kHz/16bit/mono
    merged = b''
    megaphone_active = True

    while True:
        try:
            item = pcm_queue.get(timeout=0.2)
        except Exception:
            # Timeout — flush any accumulated audio then exit megaphone to stop looping
            if merged:
                duration = len(merged) / 32000
                wav = _pcm_to_wav(merged)
                _send_wav(wav)
                merged = b''
                time.sleep(duration)
            if megaphone_active:
                _send(4002, {})
                megaphone_active = False
            continue

        if item is None:
            # Flush remaining and exit
            if merged:
                duration = len(merged) / 32000
                wav = _pcm_to_wav(merged)
                _send_wav(wav)
                time.sleep(duration)
            break

        # Re-enter megaphone if we exited due to silence
        if not megaphone_active:
            _send(4001, {})
            time.sleep(0.1)
            megaphone_active = True

        merged += item
        if len(merged) >= merge_bytes:
            duration = len(merged) / 32000
            wav = _pcm_to_wav(merged)
            _send_wav(wav)
            merged = b''
            # Wait for playback before sending next block
            time.sleep(duration)

    # Exit megaphone mode
    _send(4002, {})
    print("[SpeakerWorker] exited", flush=True)


class _SpeakerNode(Node):
    """Subscribes to ROS2 audio topic and forwards PCM to speaker subprocess."""

    def __init__(self, network_iface: str):
        super().__init__("go2_speaker")
        self._network_iface = network_iface
        self._topic: str | None = None
        self._sub = None
        self.state = "idle"
        self._pcm_q: multiprocessing.Queue | None = None
        self._proc: multiprocessing.Process | None = None

        self.get_logger().info("SpeakerNode ready (independent subprocess)")

    def start_play(self, topic: str) -> str:
        if self._sub is not None:
            if self._topic == topic:
                return self._topic
            self.stop_play()
        # Start speaker subprocess
        ctx = multiprocessing.get_context("spawn")
        self._pcm_q = ctx.Queue()
        self._proc = ctx.Process(
            target=_speaker_worker,
            args=(self._pcm_q, self._network_iface),
            daemon=True,
        )
        self._proc.start()
        self._topic = topic
        self._sub = self.create_subscription(
            AudioChunk, topic, self._on_chunk, _LOW_LAT_QOS,
        )
        self.state = "playing"
        self.get_logger().info(f"[speaker] subscribed to {topic}, subprocess started")
        return topic

    def stop_play(self) -> None:
        if self._sub is not None:
            self.destroy_subscription(self._sub)
            self._sub = None
        if self._pcm_q is not None:
            self._pcm_q.put(None)  # signal subprocess to exit
            self._pcm_q = None
        if self._proc is not None:
            self._proc.join(timeout=5)
            if self._proc.is_alive():
                self._proc.terminate()
            self._proc = None
        self.state = "idle"

    def _on_chunk(self, msg: AudioChunk) -> None:
        if self._pcm_q is not None:
            self._pcm_q.put(bytes(msg.data))


class SpeakerPlugin:
    PREFIX = "speaker"

    def __init__(self, plugin_config: dict, namespace: str, executor, rpc_proxy, network_iface: str = "eth0"):
        self._node = _SpeakerNode(network_iface)
        executor.add_node(self._node)

    def get_tool(self) -> dict:
        return {
            "name": "speaker",
            "type": "actuator",
            "multiInstance": False,
            "description": "Go2 speaker — subscribes to ROS2 topic and streams PCM-16k audio to robot speaker via AudioHub megaphone (independent subprocess)",
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


# ── VuiPlugin (actuator) ─────────────────────────────────────────────────────

class VuiPlugin:
    """Go2 volume/brightness control via VuiClient RPC."""
    PREFIX = "vui"

    def __init__(self, plugin_config: dict, namespace: str, executor, rpc_proxy):
        self._proxy = rpc_proxy

    def get_tool(self) -> dict:
        return {
            "name": "vui",
            "type": "actuator",
            "multiInstance": False,
            "description": "Go2 volume and LED brightness control — set/get volume (0-10) and brightness (0-10)",
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {
                        "type": "string",
                        "enum": ["set_volume", "get_volume", "set_brightness", "get_brightness"],
                        "description": "Action to perform",
                    },
                    "level": {"type": "integer", "description": "Volume or brightness level (0-10)"},
                },
                "required": ["action"],
                "x-action-params": {
                    "set_volume":     {"params": ["level"], "description": "Set speaker volume (0-10)"},
                    "get_volume":     {"params": [],        "description": "Get current speaker volume"},
                    "set_brightness": {"params": ["level"], "description": "Set LED brightness (0-10)"},
                    "get_brightness": {"params": [],        "description": "Get current LED brightness"},
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
        if action == "set_volume":
            level = max(0, min(10, int(args.get("level", 5))))
            ret = self._proxy.Vui_SetVolume(level)
            return {"ret": ret, "volume": level}
        elif action == "get_volume":
            code, vol = self._proxy.Vui_GetVolume()
            return {"ret": code, "volume": vol}
        elif action == "set_brightness":
            level = max(0, min(10, int(args.get("level", 5))))
            ret = self._proxy.Vui_SetBrightness(level)
            return {"ret": ret, "brightness": level}
        elif action == "get_brightness":
            code, bright = self._proxy.Vui_GetBrightness()
            return {"ret": code, "brightness": bright}
        return None


# ── LocoStatePlugin (sensor) ─────────────────────────────────────────────────

class _LocoStateNode(Node):
    """Subscribes to DDS rt/sportmodestate and republishes at 10Hz to ROS2."""

    _THROTTLE_INTERVAL = 0.1  # 10Hz

    def __init__(self, topic: str):
        super().__init__("go2_loco_state")
        self._topic = topic
        self._pub = self.create_publisher(String, topic, _LOW_LAT_QOS)
        self._last_pub_time = 0.0

        try:
            from unitree_sdk2py.core.channel import ChannelSubscriber
            from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_
            self._sub = ChannelSubscriber("rt/sportmodestate", SportModeState_)
            self._sub.Init(self._on_state, 10)
            self.get_logger().info(f"LocoStateNode subscribed rt/sportmodestate → {topic}")
        except Exception as e:
            self.get_logger().warn(f"LocoStateNode: failed to subscribe: {e}")

    def _on_state(self, msg) -> None:
        now = time.monotonic()
        if now - self._last_pub_time < self._THROTTLE_INTERVAL:
            return
        self._last_pub_time = now

        try:
            imu = msg.imu_state
            data = {
                "mode": int(msg.error_code) if hasattr(msg, 'error_code') else 0,
                "position": [round(float(p), 4) for p in msg.position],
                "velocity": [round(float(v), 4) for v in msg.velocity],
                "body_height": round(float(msg.body_height), 4),
                "yaw_speed": round(float(msg.yaw_speed), 4),
                "imu": {
                    "quaternion": [round(float(q), 5) for q in imu.quaternion],
                    "gyroscope": [round(float(g), 4) for g in imu.gyroscope],
                    "accelerometer": [round(float(a), 4) for a in imu.accelerometer],
                    "rpy": [round(float(r), 4) for r in imu.rpy],
                    "temperature": int(imu.temperature),
                },
            }
            out = String()
            out.data = json.dumps(data)
            self._pub.publish(out)
        except Exception as e:
            self.get_logger().error(f"LocoState publish error: {e}")


class LocoStatePlugin:
    PREFIX = "loco_state"

    def __init__(self, plugin_config: dict, namespace: str, executor):
        self._topic = f"/{namespace}/loco/state"
        self._node = _LocoStateNode(self._topic)
        executor.add_node(self._node)

    def get_tool(self) -> dict:
        return {
            "name": "loco_state",
            "type": "sensor",
            "multiInstance": False,
            "description": f"Go2 locomotion state (always active) — mode, velocity, position, body_height, yaw_speed, IMU. Publishes at 10Hz to {self._topic}",
            "inputSchema": {"type": "object", "properties": {}},
            "topic_out": [{"topic": self._topic, "format": "data/json"}],
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
            return {"state": "running", "topic_out": [{"topic": self._topic, "format": "data/json"}]}
        return None


# ── LocoPlugin (actuator) ────────────────────────────────────────────────────

# Go2 state machine codes for reference
_GO2_STATE_CODES = {
    100: "agile", 1001: "damping", 1002: "stand_lock", 1013: "balance_stand",
    1015: "regular_walk", 1016: "regular_run", 1017: "economy_mode",
    2007: "dodge_avoid", 2008: "bound", 2009: "jump_run", 2010: "classic",
    2011: "handstand", 2012: "front_flip", 2013: "back_flip", 2014: "left_flip",
    2016: "cross_step", 2017: "upright",
}


class LocoPlugin:
    """Go2 locomotion control via SportClient RPC — exposes 4 tools."""
    PREFIX = "loco"

    def __init__(self, plugin_config: dict, namespace: str, executor, rpc_proxy):
        self._proxy = rpc_proxy

    def get_tools(self) -> list:
        return [self._loco_tool(), self._switch_gait_tool(), self._gesture_tool(), self._acrobatics_tool()]

    def _loco_tool(self) -> dict:
        return {
            "name": "loco",
            "type": "actuator",
            "multiInstance": False,
            "description": "Go2 basic locomotion control — move, stop, stand, euler, speed level, auto recovery. Move command persists ~1 second and must be re-issued for continuous motion.",
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {
                        "type": "string",
                        "enum": ["move", "stop_move", "balance_stand", "stand_up", "stand_down",
                                 "recovery_stand", "damp", "euler", "speed_level",
                                 "switch_joystick", "auto_recovery_set", "auto_recovery_get"],
                        "description": "Action to perform",
                    },
                    "vx":      {"type": "number", "description": "Forward velocity m/s [-2.5, 3.8]"},
                    "vy":      {"type": "number", "description": "Lateral velocity m/s [-1.0, 1.0]"},
                    "vyaw":    {"type": "number", "description": "Yaw rotation rad/s [-4.0, 4.0]"},
                    "roll":    {"type": "number", "description": "Roll angle rad [-0.75, 0.75]"},
                    "pitch":   {"type": "number", "description": "Pitch angle rad [-0.75, 0.75]"},
                    "yaw":     {"type": "number", "description": "Yaw angle rad [-0.6, 0.6]"},
                    "level":   {"type": "integer", "description": "Speed level: -1=slow, 0=normal, 1=fast"},
                    "flag":    {"type": "boolean", "description": "Enable/disable flag"},
                },
                "required": ["action"],
                "x-action-params": {
                    "move":              {"params": ["vx", "vy", "vyaw"],    "description": "Move with velocity (persists ~1s, re-issue for continuous)"},
                    "stop_move":         {"params": [],                       "description": "Stop all movement immediately"},
                    "balance_stand":     {"params": [],                       "description": "Enter balance stand mode"},
                    "stand_up":          {"params": [],                       "description": "Stand up tall (0.33m)"},
                    "stand_down":        {"params": [],                       "description": "Lie down"},
                    "recovery_stand":    {"params": [],                       "description": "Recover from fallen state to standing"},
                    "damp":              {"params": [],                       "description": "Emergency stop — all motors enter damping state"},
                    "euler":             {"params": ["roll", "pitch", "yaw"], "description": "Set body attitude (radians)"},
                    "speed_level":       {"params": ["level"],                "description": "Set speed gear: -1=slow, 0=normal, 1=fast"},
                    "switch_joystick":   {"params": ["flag"],                 "description": "Enable/disable native remote control"},
                    "auto_recovery_set": {"params": ["flag"],                 "description": "Enable/disable auto-flip recovery (disable when carrying payload)"},
                    "auto_recovery_get": {"params": [],                       "description": "Query auto-recovery status"},
                },
            },
        }

    def _switch_gait_tool(self) -> dict:
        return {
            "name": "switch_gait",
            "type": "actuator",
            "multiInstance": False,
            "description": "Go2 gait mode switching — AI agile/classic, economic, running, walking, and special gaits. Some modes cause motor overheating.",
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {
                        "type": "string",
                        "enum": ["free_walk", "free_bound", "free_jump", "free_avoid",
                                 "classic_walk", "walk_upright", "cross_step",
                                 "trot_run", "static_walk", "economic_gait", "switch_avoid_mode"],
                        "description": "Gait to switch to",
                    },
                    "flag": {"type": "boolean", "description": "Enable/disable (for gaits that support toggle)"},
                },
                "required": ["action"],
                "x-action-params": {
                    "free_walk":         {"params": [],       "description": "AI Agile mode (default) — strong terrain adaptability, stairs, gravel, grass, wet surfaces"},
                    "free_bound":        {"params": ["flag"], "description": "Bounding gait (legs together running). flag=true to enter, false to exit to FreeWalk"},
                    "free_jump":         {"params": ["flag"], "description": "Jump-running gait. flag=true to enter, false to exit to FreeWalk"},
                    "free_avoid":        {"params": ["flag"], "description": "Dodge/avoidance mode — obstacle avoidance while moving. flag=true to enter, false to exit to FreeWalk"},
                    "classic_walk":      {"params": ["flag"], "description": "AI Classic gait — stable and elegant, terrain adaptive. flag=true to enter, false to exit to FreeWalk"},
                    "walk_upright":      {"params": ["flag"], "description": "⚠️ Rear-leg upright walking (MOTORS OVERHEAT EASILY). flag=true to enter, false to exit"},
                    "cross_step":        {"params": ["flag"], "description": "⚠️ Cross-step gait (MOTORS OVERHEAT EASILY). flag=true to enter, false to exit"},
                    "trot_run":          {"params": [],       "description": "Regular running mode — max 3.7m/s (⚠️ DANGEROUS, no terrain adaptation)"},
                    "static_walk":       {"params": [],       "description": "Regular walking mode — elegant but no terrain adaptation"},
                    "economic_gait":     {"params": [],       "description": "Economy/battery-saving mode — ~4h on single battery"},
                    "switch_avoid_mode": {"params": [],       "description": "Disable front/rear obstacle dodging when joystick not pushed (in avoid mode)"},
                },
            },
        }

    def _gesture_tool(self) -> dict:
        return {
            "name": "gesture",
            "type": "actuator",
            "multiInstance": False,
            "description": "Go2 gesture and performance actions — greetings, dances, poses",
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {
                        "type": "string",
                        "enum": ["hello", "stretch", "content", "heart", "scrape",
                                 "sit", "rise_sit", "pose", "dance1", "dance2"],
                        "description": "Gesture action to perform",
                    },
                    "flag": {"type": "boolean", "description": "Enable/disable (for pose)"},
                },
                "required": ["action"],
                "x-action-params": {
                    "hello":    {"params": [],       "description": "Wave/greeting gesture"},
                    "stretch":  {"params": [],       "description": "Stretching pose"},
                    "content":  {"params": [],       "description": "Happy/content gesture"},
                    "heart":    {"params": [],       "description": "Heart shape gesture"},
                    "scrape":   {"params": [],       "description": "Chinese New Year greeting bow (拜年)"},
                    "sit":      {"params": [],       "description": "Sit down"},
                    "rise_sit": {"params": [],       "description": "Stand up from sitting"},
                    "pose":     {"params": ["flag"], "description": "Strike a pose (flag=true) or recover (flag=false)"},
                    "dance1":   {"params": [],       "description": "Dance routine 1"},
                    "dance2":   {"params": [],       "description": "Dance routine 2"},
                },
            },
        }

    def _acrobatics_tool(self) -> dict:
        return {
            "name": "acrobatics",
            "type": "actuator",
            "multiInstance": False,
            "description": "Go2 acrobatic tricks — ⚠️ DANGEROUS, ensure clear space and soft ground. Flips and jumps may damage the robot.",
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {
                        "type": "string",
                        "enum": ["front_flip", "back_flip", "left_flip",
                                 "front_jump", "front_pounce", "hand_stand"],
                        "description": "Acrobatic trick to perform",
                    },
                    "flag": {"type": "boolean", "description": "Enable/disable (for hand_stand)"},
                },
                "required": ["action"],
                "x-action-params": {
                    "front_flip":   {"params": [],       "description": "⚠️ DANGEROUS: Forward flip"},
                    "back_flip":    {"params": [],       "description": "⚠️ DANGEROUS: Backward flip"},
                    "left_flip":    {"params": [],       "description": "⚠️ DANGEROUS: Left flip"},
                    "front_jump":   {"params": [],       "description": "Forward jump"},
                    "front_pounce": {"params": [],       "description": "Forward pounce"},
                    "hand_stand":   {"params": ["flag"], "description": "⚠️ Inverted handstand walking (MOTORS OVERHEAT). flag=true to enter, false to exit"},
                },
            },
        }

    def start(self) -> None:
        pass

    def stop(self) -> None:
        self._proxy.StopMove()

    def dispatch(self, action: str, args: dict) -> dict | None:
        if action == "start":
            return {"state": "ready"}
        if action == "stop":
            return {"state": "idle"}

        # ── loco tool actions ──
        if action == "move":
            vx   = max(-2.5, min(3.8, float(args.get("vx",   0))))
            vy   = max(-1.0, min(1.0, float(args.get("vy",   0))))
            vyaw = max(-4.0, min(4.0, float(args.get("vyaw", 0))))
            ret = self._proxy.Move(vx, vy, vyaw)
            return {"ret": ret, "vx": vx, "vy": vy, "vyaw": vyaw}
        elif action == "stop_move":
            ret = self._proxy.StopMove()
            return {"ret": ret}
        elif action == "balance_stand":
            ret = self._proxy.BalanceStand()
            return {"ret": ret}
        elif action == "stand_up":
            ret = self._proxy.StandUp()
            return {"ret": ret}
        elif action == "stand_down":
            ret = self._proxy.StandDown()
            return {"ret": ret}
        elif action == "recovery_stand":
            ret = self._proxy.RecoveryStand()
            return {"ret": ret}
        elif action == "damp":
            ret = self._proxy.Damp()
            return {"ret": ret}
        elif action == "euler":
            roll  = max(-0.75, min(0.75, float(args.get("roll",  0))))
            pitch = max(-0.75, min(0.75, float(args.get("pitch", 0))))
            yaw   = max(-0.6,  min(0.6,  float(args.get("yaw",   0))))
            ret = self._proxy.Euler(roll, pitch, yaw)
            return {"ret": ret, "roll": roll, "pitch": pitch, "yaw": yaw}
        elif action == "speed_level":
            level = max(-1, min(1, int(args.get("level", 0))))
            ret = self._proxy.SpeedLevel(level)
            return {"ret": ret, "level": level}
        elif action == "switch_joystick":
            flag = bool(args.get("flag", True))
            ret = self._proxy.SwitchJoystick(flag)
            return {"ret": ret, "flag": flag}
        elif action == "auto_recovery_set":
            flag = bool(args.get("flag", True))
            ret = self._proxy.AutoRecoverySet(flag)
            return {"ret": ret, "enabled": flag}
        elif action == "auto_recovery_get":
            code, enabled = self._proxy.AutoRecoveryGet()
            return {"ret": code, "enabled": enabled}

        # ── switch_gait tool actions ──
        elif action == "free_walk":
            ret = self._proxy.FreeWalk()
            return {"ret": ret, "gait": "free_walk"}
        elif action == "free_bound":
            flag = bool(args.get("flag", True))
            ret = self._proxy.FreeBound(flag)
            return {"ret": ret, "gait": "free_bound", "flag": flag}
        elif action == "free_jump":
            flag = bool(args.get("flag", True))
            ret = self._proxy.FreeJump(flag)
            return {"ret": ret, "gait": "free_jump", "flag": flag}
        elif action == "free_avoid":
            flag = bool(args.get("flag", True))
            ret = self._proxy.FreeAvoid(flag)
            return {"ret": ret, "gait": "free_avoid", "flag": flag}
        elif action == "classic_walk":
            flag = bool(args.get("flag", True))
            ret = self._proxy.ClassicWalk(flag)
            return {"ret": ret, "gait": "classic_walk", "flag": flag}
        elif action == "walk_upright":
            flag = bool(args.get("flag", True))
            ret = self._proxy.WalkUpright(flag)
            return {"ret": ret, "gait": "walk_upright", "flag": flag}
        elif action == "cross_step":
            flag = bool(args.get("flag", True))
            ret = self._proxy.CrossStep(flag)
            return {"ret": ret, "gait": "cross_step", "flag": flag}
        elif action == "trot_run":
            ret = self._proxy.TrotRun()
            return {"ret": ret, "gait": "trot_run"}
        elif action == "static_walk":
            ret = self._proxy.StaticWalk()
            return {"ret": ret, "gait": "static_walk"}
        elif action == "economic_gait":
            ret = self._proxy.EconomicGait()
            return {"ret": ret, "gait": "economic_gait"}
        elif action == "switch_avoid_mode":
            ret = self._proxy.SwitchAvoidMode()
            return {"ret": ret}

        # ── gesture tool actions ──
        elif action == "hello":
            ret = self._proxy.Hello()
            return {"ret": ret}
        elif action == "stretch":
            ret = self._proxy.Stretch()
            return {"ret": ret}
        elif action == "content":
            ret = self._proxy.Content()
            return {"ret": ret}
        elif action == "heart":
            ret = self._proxy.Heart()
            return {"ret": ret}
        elif action == "scrape":
            ret = self._proxy.Scrape()
            return {"ret": ret}
        elif action == "sit":
            ret = self._proxy.Sit()
            return {"ret": ret}
        elif action == "rise_sit":
            ret = self._proxy.RiseSit()
            return {"ret": ret}
        elif action == "pose":
            flag = bool(args.get("flag", True))
            ret = self._proxy.Pose(flag)
            return {"ret": ret, "flag": flag}
        elif action == "dance1":
            ret = self._proxy.Dance1()
            return {"ret": ret}
        elif action == "dance2":
            ret = self._proxy.Dance2()
            return {"ret": ret}

        # ── acrobatics tool actions ──
        elif action == "front_flip":
            ret = self._proxy.FrontFlip()
            return {"ret": ret}
        elif action == "back_flip":
            ret = self._proxy.BackFlip()
            return {"ret": ret}
        elif action == "left_flip":
            ret = self._proxy.LeftFlip()
            return {"ret": ret}
        elif action == "front_jump":
            ret = self._proxy.FrontJump()
            return {"ret": ret}
        elif action == "front_pounce":
            ret = self._proxy.FrontPounce()
            return {"ret": ret}
        elif action == "hand_stand":
            flag = bool(args.get("flag", True))
            ret = self._proxy.HandStand(flag)
            return {"ret": ret, "flag": flag}

        return None


# ── ObstaclesAvoidPlugin (actuator) ──────────────────────────────────────────

class ObstaclesAvoidPlugin:
    """Go2 dedicated obstacle avoidance — velocity/incremental/absolute position control with auto-avoidance."""
    PREFIX = "obstacles_avoid"

    def __init__(self, plugin_config: dict, namespace: str, executor, rpc_proxy):
        self._proxy = rpc_proxy

    def get_tool(self) -> dict:
        return {
            "name": "obstacles_avoid",
            "type": "actuator",
            "multiInstance": False,
            "description": "Go2 obstacle avoidance — move with intelligent collision avoidance. Supports velocity, incremental position, and absolute position control. Must call take_control before API movement.",
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {
                        "type": "string",
                        "enum": ["enable", "disable", "get_status", "take_control",
                                 "move", "move_to_increment", "move_to_absolute"],
                        "description": "Action to perform",
                    },
                    "vx":  {"type": "number", "description": "X velocity m/s [-1.5, 1.5] or X position m [-2.0, 2.0]"},
                    "vy":  {"type": "number", "description": "Y velocity m/s [-1.0, 1.0] or Y position m [-2.0, 2.0]"},
                    "vyaw": {"type": "number", "description": "Yaw velocity rad/s [-1.57, 1.57] or Yaw angle rad"},
                },
                "required": ["action"],
                "x-action-params": {
                    "enable":            {"params": [],                 "description": "Enable obstacle avoidance"},
                    "disable":           {"params": [],                 "description": "Disable obstacle avoidance"},
                    "get_status":        {"params": [],                 "description": "Get obstacle avoidance status"},
                    "take_control":      {"params": [],                 "description": "Take over joystick velocity control (must call before API movement)"},
                    "move":              {"params": ["vx", "vy", "vyaw"], "description": "Move with obstacle avoidance (velocity mode)"},
                    "move_to_increment": {"params": ["vx", "vy", "vyaw"], "description": "Move by incremental position with avoidance"},
                    "move_to_absolute":  {"params": ["vx", "vy", "vyaw"], "description": "Move to absolute world position with avoidance"},
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
            ret = self._proxy.OA_SwitchSet(True)
            return {"ret": ret, "enabled": True}
        elif action == "disable":
            ret = self._proxy.OA_SwitchSet(False)
            return {"ret": ret, "enabled": False}
        elif action == "get_status":
            code, enabled = self._proxy.OA_SwitchGet()
            return {"ret": code, "enabled": enabled}
        elif action == "take_control":
            ret = self._proxy.OA_UseRemoteCommandFromApi(True)
            return {"ret": ret, "api_control": True}
        elif action == "move":
            vx   = max(-1.5,  min(1.5,  float(args.get("vx",   0))))
            vy   = max(-1.0,  min(1.0,  float(args.get("vy",   0))))
            vyaw = max(-1.57, min(1.57, float(args.get("vyaw", 0))))
            ret = self._proxy.OA_Move(vx, vy, vyaw)
            return {"ret": ret, "vx": vx, "vy": vy, "vyaw": vyaw, "mode": "velocity"}
        elif action == "move_to_increment":
            x    = max(-2.0,  min(2.0,  float(args.get("vx",   0))))
            y    = max(-2.0,  min(2.0,  float(args.get("vy",   0))))
            yaw  = max(-1.57, min(1.57, float(args.get("vyaw", 0))))
            ret = self._proxy.OA_MoveToIncrementPosition(x, y, yaw)
            return {"ret": ret, "x": x, "y": y, "yaw": yaw, "mode": "increment"}
        elif action == "move_to_absolute":
            x   = float(args.get("vx",   0))
            y   = float(args.get("vy",   0))
            yaw = float(args.get("vyaw", 0))
            ret = self._proxy.OA_MoveToAbsolutePosition(x, y, yaw)
            return {"ret": ret, "x": x, "y": y, "yaw": yaw, "mode": "absolute"}
        return None


# ── CameraPlugin (sensor) ────────────────────────────────────────────────────

class _CameraNode:
    """Manages a subprocess that receives Go2 H.264 UDP multicast video and publishes MJPEG frames."""

    def __init__(self, topic: str, stream_addr: str, stream_port: int):
        self._topic = topic
        self._stream_addr = stream_addr
        self._stream_port = stream_port
        self._proc = None
        self.state = "idle"

    def start_capture(self) -> None:
        if self.state == "running":
            return
        import multiprocessing as mp
        ctx = mp.get_context("spawn")
        self._proc = ctx.Process(
            target=_run_camera_process,
            args=(self._topic, self._stream_addr, self._stream_port),
            name="go2_camera",
            daemon=True,
        )
        self._proc.start()
        self.state = "running"
        print(f"[camera] subprocess started → pid={self._proc.pid}", flush=True)

    def stop_capture(self) -> None:
        if self._proc is not None and self._proc.is_alive():
            self._proc.terminate()
            self._proc.join(timeout=3.0)
            if self._proc.is_alive():
                self._proc.kill()
                self._proc.join(timeout=2.0)
        self._proc = None
        self.state = "idle"
        print("[camera] subprocess stopped", flush=True)


def _run_camera_process(topic: str, stream_addr: str, stream_port: int) -> None:
    """Camera subprocess — receives Go2 H264 UDP multicast, decodes to JPEG, publishes to ROS2."""
    import subprocess as _subprocess
    import threading as _threading
    import rclpy
    from rclpy.node import Node as _Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
    from sensor_msgs.msg import CompressedImage as _CompressedImage

    _QOS = QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        durability=DurabilityPolicy.VOLATILE,
    )

    rclpy.init()
    node = _Node("go2_camera")
    pub = node.create_publisher(_CompressedImage, topic, _QOS)

    # GStreamer pipeline for Go2 front camera (UDP multicast H264)
    cmd = [
        "gst-launch-1.0", "-q",
        "udpsrc", f"address={stream_addr}", f"port={stream_port}", "!",
        "application/x-rtp,media=video,clock-rate=90000,encoding-name=H264,payload=96", "!",
        "rtph264depay", "!",
        "avdec_h264", "!",
        "videoconvert", "!",
        "jpegenc", "quality=75", "!", "fdsink", "fd=1",
    ]

    try:
        proc = _subprocess.Popen(cmd, stdout=_subprocess.PIPE, stderr=_subprocess.DEVNULL)
    except FileNotFoundError:
        node.get_logger().error("gst-launch-1.0 not found — camera stream disabled")
        node.destroy_node()
        rclpy.shutdown()
        return

    node.get_logger().info(f"Camera capture started: {stream_addr}:{stream_port} → {topic}")

    buf = bytearray()
    CHUNK = 65536
    try:
        while proc.poll() is None:
            data = proc.stdout.read(CHUNK)
            if not data:
                break
            buf.extend(data)
            while True:
                soi = buf.find(b'\xff\xd8')
                if soi == -1:
                    buf.clear()
                    break
                eoi = buf.find(b'\xff\xd9', soi + 2)
                if eoi == -1:
                    if soi > 0:
                        del buf[:soi]
                    break
                frame = bytes(buf[soi:eoi + 2])
                del buf[:eoi + 2]
                msg = _CompressedImage()
                msg.header.stamp = node.get_clock().now().to_msg()
                msg.format = "jpeg"
                msg.data = frame
                pub.publish(msg)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            proc.terminate()
            proc.wait(timeout=2)
        except Exception:
            try:
                proc.kill()
            except Exception:
                pass
        node.destroy_node()
        rclpy.shutdown()


class CameraPlugin:
    PREFIX = "camera"

    def __init__(self, plugin_config: dict, namespace: str, executor, rpc_proxy=None):
        self._topic = f"/{namespace}/camera/front"
        self._stream_addr = plugin_config.get("stream_addr", "230.1.1.1")
        self._stream_port = int(plugin_config.get("stream_port", 1720))
        self._node = _CameraNode(self._topic, self._stream_addr, self._stream_port)
        self._proxy = rpc_proxy

    def get_tool(self) -> dict:
        return {
            "name": "camera_front",
            "type": "sensor",
            "multiInstance": False,
            "description": f"Go2 front camera (1280x720 @ 15fps) — H.264 UDP multicast decoded to MJPEG. Also supports on-demand JPEG snapshot. Publishes to {self._topic}",
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {
                        "type": "string",
                        "enum": ["snapshot"],
                        "description": "Optional action (snapshot returns single JPEG frame via VideoClient)",
                    },
                },
            },
            "topic_out": [{"topic": self._topic, "format": "image/jpeg"}],
        }

    def start(self) -> None:
        self._node.start_capture()

    def stop(self) -> None:
        self._node.stop_capture()

    def dispatch(self, action: str, args: dict) -> dict | None:
        if action == "start":
            return {"state": "running"}
        if action == "stop":
            return {"state": "idle"}
        if action == "info":
            return {"state": self._node.state, "topic_out": [{"topic": self._topic, "format": "image/jpeg"}]}
        if action == "snapshot":
            if self._proxy is None:
                return {"error": "VideoClient not available"}
            result = self._proxy.Video_GetImageSample()
            if result is None:
                return {"error": "Failed to capture snapshot"}
            return {"snapshot": "captured", "size": len(result) if result else 0}
        return None


# ── StatePlugin (sensor) ─────────────────────────────────────────────────────

# Go2 motor index → joint name mapping (12 motors: 3 per leg)
_GO2_JOINT_NAMES = [
    'FR_hip_joint', 'FR_thigh_joint', 'FR_calf_joint',    # 0-2
    'FL_hip_joint', 'FL_thigh_joint', 'FL_calf_joint',    # 3-5
    'RR_hip_joint', 'RR_thigh_joint', 'RR_calf_joint',    # 6-8
    'RL_hip_joint', 'RL_thigh_joint', 'RL_calf_joint',    # 9-11
]


class _LowStateNode(Node):
    """Subscribes to DDS rt/lowstate (Go2 LowState_) and republishes to ROS2."""

    _JOINTS_INTERVAL = 0.1     # 10 Hz
    _IMU_INTERVAL    = 0.05    # 20 Hz
    _BMS_INTERVAL    = 1.0     # 1 Hz

    def __init__(self, imu_topic: str, battery_topic: str, joints_topic: str):
        super().__init__("go2_low_state")
        self._imu_pub     = self.create_publisher(String, imu_topic,     _LOW_LAT_QOS)
        self._battery_pub = self.create_publisher(String, battery_topic, _LOW_LAT_QOS)
        self._joints_pub  = self.create_publisher(String, joints_topic,  _LOW_LAT_QOS)
        self._last_joints_time: float = 0.0
        self._last_imu_time:    float = 0.0
        self._last_bms_time:    float = 0.0

        try:
            from unitree_sdk2py.core.channel import ChannelSubscriber
            from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowState_
            self._lowstate_sub = ChannelSubscriber("rt/lowstate", LowState_)
            self._lowstate_sub.Init(self._on_state, 10)
            self.get_logger().info(f"LowStateNode subscribed rt/lowstate → {imu_topic}, {joints_topic}")
        except Exception as e:
            self.get_logger().warn(f"LowStateNode: failed to subscribe rt/lowstate: {e}")

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
                "temperature":   int(imu.temperature),
            }
            imu_out = String()
            imu_out.data = json.dumps(imu_data)
            self._imu_pub.publish(imu_out)

        # Joints: throttle to 10 Hz
        if now - self._last_joints_time >= self._JOINTS_INTERVAL:
            self._last_joints_time = now
            joints = []
            for i, m in enumerate(msg.motor_state):
                if i >= len(_GO2_JOINT_NAMES):
                    break
                name = _GO2_JOINT_NAMES[i]
                joints.append({
                    "idx": i,
                    "name": name,
                    "q": round(float(m.q), 4),
                    "dq": round(float(m.dq), 4),
                    "tau": round(float(m.tau_est), 3),
                    "temperature": int(m.temperature) if hasattr(m, 'temperature') else 0,
                })
            joints_out = String()
            joints_out.data = json.dumps({"joints": joints})
            self._joints_pub.publish(joints_out)

        # Battery: throttle to 1 Hz (from bms_state field if available)
        if now - self._last_bms_time >= self._BMS_INTERVAL:
            if hasattr(msg, 'bms_state'):
                self._last_bms_time = now
                bms = msg.bms_state
                bms_data = {
                    "soc": int(bms.soc) if hasattr(bms, 'soc') else 0,
                    "current": int(bms.current) if hasattr(bms, 'current') else 0,
                    "cycle": int(bms.cycle) if hasattr(bms, 'cycle') else 0,
                }
                bat_out = String()
                bat_out.data = json.dumps(bms_data)
                self._battery_pub.publish(bat_out)


class StatePlugin:
    PREFIX = "state"

    def __init__(self, plugin_config: dict, namespace: str, executor):
        self._imu_topic     = f"/{namespace}/state/imu"
        self._battery_topic = f"/{namespace}/state/battery"
        self._joints_topic  = f"/{namespace}/state/joints"
        self._node = _LowStateNode(self._imu_topic, self._battery_topic, self._joints_topic)
        executor.add_node(self._node)

    def get_tools(self) -> list:
        return [self._imu_tool(), self._battery_tool(), self._joints_tool()]

    def _imu_tool(self) -> dict:
        return {
            "name": "imu",
            "type": "sensor",
            "multiInstance": False,
            "description": f"Go2 IMU sensor — quaternion, gyroscope, accelerometer, rpy, temperature. Publishes at 20Hz to {self._imu_topic}",
            "inputSchema": {"type": "object", "properties": {}},
            "topic_out": [{"topic": self._imu_topic, "format": "data/json"}],
        }

    def _battery_tool(self) -> dict:
        return {
            "name": "battery",
            "type": "sensor",
            "multiInstance": False,
            "description": f"Go2 BMS battery — SOC%, current, charge cycles. Publishes at 1Hz to {self._battery_topic}",
            "inputSchema": {"type": "object", "properties": {}},
            "topic_out": [{"topic": self._battery_topic, "format": "data/json"}],
        }

    def _joints_tool(self) -> dict:
        return {
            "name": "joints",
            "type": "sensor",
            "multiInstance": False,
            "description": f"Go2 joint states — 12 motors (4 legs x 3 joints: hip/thigh/calf) with position(q), velocity(dq), torque(tau), temperature. Publishes at 10Hz to {self._joints_topic}",
            "inputSchema": {"type": "object", "properties": {}},
            "topic_out": [{"topic": self._joints_topic, "format": "sensor/skeleton"}],
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
            tool_name = args.get('_tool_name', '')
            topic_map = {
                'imu':     (self._imu_topic,     'data/json'),
                'battery': (self._battery_topic, 'data/json'),
                'joints':  (self._joints_topic,  'sensor/skeleton'),
            }
            if tool_name in topic_map:
                topic, fmt = topic_map[tool_name]
                return {"state": "running", "topic_out": [{"topic": topic, "format": fmt}]}
            return {"state": "running"}
        return None


# ── MotionSwitcherPlugin (actuator) ──────────────────────────────────────────

class MotionSwitcherPlugin:
    """Go2 motion mode switching — check/select/release motion mode, silent mode."""
    PREFIX = "motion_switcher"

    def __init__(self, plugin_config: dict, namespace: str, executor, rpc_proxy):
        self._proxy = rpc_proxy

    def get_tool(self) -> dict:
        return {
            "name": "motion_switcher",
            "type": "actuator",
            "multiInstance": False,
            "description": "Go2 motion mode switcher — check current mode, switch between motion modes (mcf/normal/advanced/ai), release for low-level control",
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {
                        "type": "string",
                        "enum": ["check_mode", "select_mode", "release_mode"],
                        "description": "Action to perform",
                    },
                    "name": {"type": "string", "description": "Motion mode name to switch to (mcf for firmware>=1.1.6, or normal/advanced/ai)"},
                },
                "required": ["action"],
                "x-action-params": {
                    "check_mode":   {"params": [],       "description": "Check current form (0=standard, 1=wheel-leg) and motion mode"},
                    "select_mode":  {"params": ["name"], "description": "Switch to specified motion mode"},
                    "release_mode": {"params": [],       "description": "Release motion mode (required before low-level motor control)"},
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
        if action == "check_mode":
            code, data = self._proxy.MSC_CheckMode()
            return {"ret": code, "data": data}
        elif action == "select_mode":
            name = args.get("name", "mcf")
            code, data = self._proxy.MSC_SelectMode(name)
            return {"ret": code, "mode": name}
        elif action == "release_mode":
            code, data = self._proxy.MSC_ReleaseMode()
            return {"ret": code}
        return None


# ── AsrPlugin (sensor) ───────────────────────────────────────────────────────

class _AsrNode(Node):
    """Subscribes to DDS rt/audio_msg (String_) and republishes ASR results to ROS2."""

    def __init__(self, topic: str):
        super().__init__("go2_asr")
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
                "Go2 built-in ASR — offline speech recognition results "
                "(text, angle/DOA, confidence). "
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
        if action == "start":
            return {"state": "running"}
        if action == "stop":
            return {"state": "idle"}
        if action == "info":
            return {"state": "running", "topic_out": [{"topic": self._topic, "format": "data/json"}]}
        return None
