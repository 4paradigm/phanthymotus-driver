#!/usr/bin/env python3
"""
drivers/noetix/bumi/device.py — Noetix Bumi-EDU 设备插件实现。

插件列表：
  - StatePlugin: joints (21-DOF skeleton), imu, battery, model (URDF resource)
  - LocoPlugin: loco (move/stop), switch_mode (mode transitions)
  - MicPlugin: 8ch mic capture → mono PCM 16kHz
  - SpeakerPlugin: audio playback via MediaController
  - CameraPlugin: Realsense D435i color + depth
  - MotionStatePlugin: motion/safety state and change history
  - ArmPlugin: guarded Bumi-EDU dual-arm control
  - MediaSystemPlugin: MediaController status/configuration
  - DiagnosticsPlugin: low-risk read-only self-test
"""

import json
import math
import os
import struct
import subprocess
import threading
import time
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from typing import Any
from uuid import uuid4

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import String
from audio_msgs.msg import AudioChunk
from sensor_msgs.msg import CompressedImage, Image as SensorImage


_LOW_LAT_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=200,
    durability=DurabilityPolicy.VOLATILE,
)

# ── Joint Mapping ─────────────────────────────────────────────────────────────
# SDK motor_id order → URDF joint names (must match URDF exactly for skeleton renderer)

_BUMI_JOINT_NAMES = [
    # 0-3: left arm
    'l_arm_pitch_joint', 'l_arm_roll_joint', 'l_arm_yaw_joint', 'l_elbow_pitch_joint',
    # 4-9: left leg
    'l_leg_pitch_joint', 'l_leg_roll_joint', 'l_leg_yaw_joint',
    'l_knee_pitch_joint', 'l_ankle_pitch_joint', 'l_ankle_roll_joint',
    # 10-13: right arm
    'r_arm_pitch_joint', 'r_arm_roll_joint', 'r_arm_yaw_joint', 'r_elbow_pitch_joint',
    # 14-19: right leg
    'r_leg_pitch_joint', 'r_leg_roll_joint', 'r_leg_yaw_joint',
    'r_knee_pitch_joint', 'r_ankle_pitch_joint', 'r_ankle_roll_joint',
    # 20: waist
    'waist_yaw_joint',
]

# ── ControlCmd Mapping ────────────────────────────────────────────────────────
# Lazy-loaded from highcontrol_py.ControlCmd enum at runtime

_MODE_TO_CMD_NAME = {
    "walk": "WALK",
    "swing": "SWING",       # 挥手
    "shake": "SHAKE",       # 握手
    "cheer": "CHEER",       # 欢呼
    "run": "RUN",           # 预留
    "enable": "START",      # 使能/失能
    "ready": "SWITCH",      # 准备模式
    "start_teach": "STARTTEACH",
    "save_teach": "SAVETEACH",
    "end_teach": "ENDTEACH",
    "play_teach": "PLAYTEACH",
    "dance": "DANCE",
    "fall_to_stand": "FALLTOSTAND",
    "stand_to_fall": "STANDTOFALL",
    "dance1": "DANCE1",
    "dance2": "DANCE2",
    "tear": "TEAR",         # 擦眼泪
}

_ControlCmd = None  # Lazy-loaded enum module


def _get_control_cmd(name: str):
    """Get ControlCmd enum value by name."""
    global _ControlCmd
    if _ControlCmd is None:
        from highcontrol_py import ControlCmd
        _ControlCmd = ControlCmd
    return getattr(_ControlCmd, name)


def _get_default_cmd():
    """Get DEFAULT command."""
    return _get_control_cmd("DEFAULT")

_WORKMODE_NAMES = {
    0: "enabled", 1: "ready", 2: "walking", 5: "dance",
    8: "greet", 9: "shake", 10: "cheer", 11: "start_teach",
    12: "end_teach", 14: "save_teach_1", 23: "play_teach",
    26: "protection", 27: "fall_to_stand", 28: "stand_to_fall",
    29: "save_teach_2", 30: "disabled", 31: "dance1", 32: "dance2", 33: "tear",
}


# ── StatePlugin (sensor, multi-tool) ─────────────────────────────────────────

class _BumiStateNode(Node):
    """Polls Noetix SDK HighController for state data and republishes to ROS2."""

    _JOINTS_INTERVAL = 0.1     # 10 Hz
    _IMU_INTERVAL    = 0.05    # 20 Hz
    _BMS_INTERVAL    = 1.0     # 1 Hz

    def __init__(self, namespace: str, high_ctrl):
        super().__init__("bumi_state")
        self._high_ctrl = high_ctrl
        self._imu_topic     = f"/{namespace}/state/imu"
        self._battery_topic = f"/{namespace}/state/battery"
        self._joints_topic  = f"/{namespace}/state/joints"

        self._imu_pub     = self.create_publisher(String, self._imu_topic,     _LOW_LAT_QOS)
        self._battery_pub = self.create_publisher(String, self._battery_topic, _LOW_LAT_QOS)
        self._joints_pub  = self.create_publisher(String, self._joints_topic,  _LOW_LAT_QOS)

        self._last_imu: dict = {}
        self._last_battery: dict = {}
        self._lock = threading.Lock()
        self._running = False
        self._thread: threading.Thread | None = None

    def start_polling(self):
        self._running = True
        self._thread = threading.Thread(target=self._poll_loop, daemon=True, name="bumi_state_poll")
        self._thread.start()

    def stop_polling(self):
        self._running = False

    def _poll_loop(self):
        last_joints_time = 0.0
        last_imu_time = 0.0
        last_bms_time = 0.0

        while self._running:
            try:
                now = time.monotonic()

                # IMU: 20 Hz
                if now - last_imu_time >= self._IMU_INTERVAL:
                    last_imu_time = now
                    imu = self._high_ctrl.get_imu_data()
                    imu_data = {
                        "quaternion":    [imu.ori[i] for i in range(4)],
                        "angular_vel":   [imu.angular_vel[i] for i in range(3)],
                        "linear_acc":    [imu.linear_acc[i] for i in range(3)],
                    }
                    with self._lock:
                        self._last_imu = imu_data
                    msg = String()
                    msg.data = json.dumps(imu_data)
                    self._imu_pub.publish(msg)

                # Joints: 10 Hz
                if now - last_joints_time >= self._JOINTS_INTERVAL:
                    last_joints_time = now
                    joint_state = self._high_ctrl.get_joint_state()
                    joints = []
                    for i in range(21):
                        js = joint_state[i]
                        joints.append({
                            "idx": i,
                            "name": _BUMI_JOINT_NAMES[i],
                            "q": round(float(js.pos), 4),
                            "dq": round(float(js.vel), 4),
                            "tau": round(float(js.tau), 3),
                            "temp": int(js.temperature),
                        })
                    imu = self._high_ctrl.get_imu_data()
                    workmode = self._high_ctrl.get_mode()
                    joints_out = String()
                    joints_out.data = json.dumps({
                        "joints": joints,
                        "imu_quat": [float(imu.ori[3]), float(imu.ori[0]), float(imu.ori[1]), float(imu.ori[2])],  # SDK [x,y,z,w] → renderer [w,x,y,z]
                        "workmode": workmode,
                    })
                    self._joints_pub.publish(joints_out)

                # Battery: 1 Hz
                if now - last_bms_time >= self._BMS_INTERVAL:
                    last_bms_time = now
                    bms = self._high_ctrl.get_robot_bms_data()
                    bms_data = {
                        "soc": int(bms.battery_soc),
                        "soh": int(bms.battery_soh),
                        "temperature": int(bms.battery_temp),
                        "alarm": int(bms.battery_alarm),
                    }
                    with self._lock:
                        self._last_battery = bms_data
                    msg = String()
                    msg.data = json.dumps(bms_data)
                    self._battery_pub.publish(msg)

                time.sleep(0.02)  # 50 Hz poll loop
            except Exception as e:
                self.get_logger().warn(f"State poll error: {e}")
                time.sleep(0.5)


class StatePlugin:
    PREFIX = "state"

    def __init__(self, plugin_config: dict, namespace: str, executor, high_ctrl):
        self._namespace = namespace
        self._high_ctrl = high_ctrl
        self._node = _BumiStateNode(namespace, high_ctrl)
        executor.add_node(self._node)

    def get_tools(self) -> list:
        ns = self._namespace
        return [
            {
                "name": "imu",
                "type": "sensor",
                "multiInstance": False,
                "description": f"Bumi IMU — quaternion, angular velocity, linear acceleration. Publishes at 20Hz to /{ns}/state/imu",
                "inputSchema": {"type": "object", "properties": {}},
                "topic_out": [{"topic": f"/{ns}/state/imu", "format": "data/json"}],
            },
            {
                "name": "battery",
                "type": "sensor",
                "multiInstance": False,
                "description": f"Bumi battery — SOC%, SOH%, temperature, alarm. Publishes at 1Hz to /{ns}/state/battery",
                "inputSchema": {"type": "object", "properties": {}},
                "topic_out": [{"topic": f"/{ns}/state/battery", "format": "data/json"}],
            },
            {
                "name": "joints",
                "type": "sensor",
                "multiInstance": False,
                "description": f"Bumi joint states — 21 DOF with position(q rad), velocity(dq), torque(tau), temperature. Publishes at 10Hz to /{ns}/state/joints",
                "inputSchema": {"type": "object", "properties": {}},
                "topic_out": [{"topic": f"/{ns}/state/joints", "format": "sensor/skeleton"}],
            },
            {
                "name": "model",
                "type": "resource",
                "multiInstance": False,
                "description": "Bumi URDF model for 3D skeleton visualization — 21-DOF kinematic chain",
                "inputSchema": {"type": "object", "properties": {}},
            },
        ]

    def start(self) -> None:
        self._node.start_polling()

    def stop(self) -> None:
        self._node.stop_polling()

    def dispatch(self, action: str, args: dict) -> dict | None:
        if action == "start":
            return {"state": "running"}
        if action == "stop":
            return {"state": "idle"}
        if action == "info":
            return {"state": "running"}
        if action == "model":
            urdf_path = Path(__file__).parent / "resource" / "bumi_model.urdf"
            if urdf_path.exists():
                return {"urdf": urdf_path.read_text()}
            return {"error": "URDF model file not found"}
        return None


# ── LocoPlugin (actuator, multi-tool) ────────────────────────────────────────

class LocoPlugin:
    PREFIX = "loco"

    def __init__(self, plugin_config: dict, namespace: str, executor, high_ctrl):
        self._high_ctrl = high_ctrl
        self._namespace = namespace
        self._lock = threading.Lock()
        self._last_cmd_time: float = 0.0
        self._move_thread: threading.Thread | None = None
        self._move_stop_event = threading.Event()

    def get_tools(self) -> list:
        return [self._loco_tool(), self._switch_mode_tool()]

    def _loco_tool(self) -> dict:
        return {
            "name": "loco",
            "type": "actuator",
            "multiInstance": False,
            "description": "Bumi locomotion — move with velocity commands or stop.",
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {
                        "type": "string",
                        "enum": ["move", "stop_move"],
                    },
                    "vx": {
                        "type": "number",
                        "description": "Forward velocity [-1, 1] (>0 forward)",
                        "minimum": -1, "maximum": 1,
                    },
                    "vy": {
                        "type": "number",
                        "description": "Lateral velocity [-1, 1] (>0 left)",
                        "minimum": -1, "maximum": 1,
                    },
                    "vyaw": {
                        "type": "number",
                        "description": "Turning velocity [-1, 1] (>0 left turn)",
                        "minimum": -1, "maximum": 1,
                    },
                    "duration": {
                        "type": "number",
                        "description": "Duration in seconds (0 = continuous until stop_move)",
                        "minimum": 0,
                    },
                },
                "required": ["action"],
                "x-action-params": {
                    "move": {
                        "params": ["vx", "vy", "vyaw", "duration"],
                        "description": "Move with specified velocities. Requires walking mode.",
                    },
                    "stop_move": {
                        "params": [],
                        "description": "Stop all movement immediately.",
                    },
                },
            },
            "topic_out": [],
        }

    def _switch_mode_tool(self) -> dict:
        return {
            "name": "switch_mode",
            "type": "actuator",
            "multiInstance": False,
            "description": "Bumi mode switching — switch between walking, gestures, dance, teach modes.",
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {
                        "type": "string",
                        "enum": ["switch", "get_mode"],
                    },
                    "mode": {
                        "type": "string",
                        "enum": list(_MODE_TO_CMD_NAME.keys()),
                        "description": "Target mode to switch to.",
                    },
                    "index": {
                        "type": "integer",
                        "description": "Teach file index (for save_teach/play_teach)",
                        "minimum": 0,
                    },
                },
                "required": ["action"],
                "x-action-params": {
                    "switch": {
                        "params": ["mode", "index"],
                        "description": "Switch robot to specified mode. Edge-triggered (sends once).",
                    },
                    "get_mode": {
                        "params": [],
                        "description": "Get current workmode.",
                    },
                },
            },
            "topic_out": [],
        }

    def start(self) -> None:
        pass

    def stop(self) -> None:
        self._stop_move()

    def dispatch(self, action: str, args: dict) -> dict | None:
        if action == "start":
            return {"state": "ready"}
        if action == "stop":
            self._stop_move()
            return {"state": "idle"}

        tool_name = args.pop('_tool_name', '')

        if action == "move":
            return self._do_move(args)
        if action == "stop_move":
            return self._stop_move()
        if action == "switch":
            return self._do_switch(args)
        if action == "get_mode":
            return self._do_get_mode()
        return None

    def _publish_cmd(self, x: float, y: float, z: float, action_cmd, index: int = 0):
        """Send command with rate limiting (≥2ms between calls). action_cmd is ControlCmd enum."""
        with self._lock:
            now = time.monotonic()
            elapsed = now - self._last_cmd_time
            if elapsed < 0.002:
                time.sleep(0.002 - elapsed)
            self._high_ctrl.publish_cmd(x, y, z, action_cmd, index)
            self._last_cmd_time = time.monotonic()

    def _do_move(self, args: dict) -> dict:
        # Check if in walking mode
        mode = self._high_ctrl.get_mode()
        if mode == 26:
            return {"error": "Robot in protection mode, cannot move"}
        if mode not in (2, 0, 1):
            return {"error": f"Cannot move in workmode {mode} ({_WORKMODE_NAMES.get(mode, 'unknown')}). Switch to walk mode first."}

        vx = float(args.get("vx", 0))
        vy = float(args.get("vy", 0))
        vyaw = float(args.get("vyaw", 0))
        duration = float(args.get("duration", 0))

        # Stop any existing move thread
        self._move_stop_event.set()
        if self._move_thread and self._move_thread.is_alive():
            self._move_thread.join(timeout=1)

        self._move_stop_event.clear()
        default_cmd = _get_default_cmd()

        if duration > 0:
            # Timed move
            def _move_timed():
                end_time = time.monotonic() + duration
                while not self._move_stop_event.is_set() and time.monotonic() < end_time:
                    self._publish_cmd(vx, vy, vyaw, default_cmd, 0)
                    time.sleep(0.02)  # 50 Hz
                # Stop
                self._publish_cmd(0, 0, 0, default_cmd, 0)

            self._move_thread = threading.Thread(target=_move_timed, daemon=True, name="bumi_move")
            self._move_thread.start()
            return {"state": "moving", "vx": vx, "vy": vy, "vyaw": vyaw, "duration": duration}
        else:
            # Continuous move with 5s watchdog
            def _move_continuous():
                watchdog_end = time.monotonic() + 5.0
                while not self._move_stop_event.is_set() and time.monotonic() < watchdog_end:
                    self._publish_cmd(vx, vy, vyaw, default_cmd, 0)
                    time.sleep(0.02)
                self._publish_cmd(0, 0, 0, default_cmd, 0)

            self._move_thread = threading.Thread(target=_move_continuous, daemon=True, name="bumi_move")
            self._move_thread.start()
            return {"state": "moving", "vx": vx, "vy": vy, "vyaw": vyaw, "duration": "continuous (5s watchdog)"}

    def _stop_move(self) -> dict:
        self._move_stop_event.set()
        if self._move_thread and self._move_thread.is_alive():
            self._move_thread.join(timeout=1)
        if self._high_ctrl is not None:
            self._publish_cmd(0, 0, 0, _get_default_cmd(), 0)
        return {"state": "stopped"}

    def _do_switch(self, args: dict) -> dict:
        mode_str = args.get("mode", "")
        index = int(args.get("index", 0))

        if mode_str not in _MODE_TO_CMD_NAME:
            return {"error": f"Unknown mode: {mode_str}. Available: {list(_MODE_TO_CMD_NAME.keys())}"}

        # Safety: check protection mode
        current_mode = self._high_ctrl.get_mode()
        if current_mode == 26:
            return {"error": "Robot in protection mode. Cannot switch modes."}

        cmd_enum = _get_control_cmd(_MODE_TO_CMD_NAME[mode_str])

        # Edge-trigger: send action once, then DEFAULT
        self._publish_cmd(0, 0, 0, cmd_enum, index)
        time.sleep(0.003)  # ≥2ms
        self._publish_cmd(0, 0, 0, _get_default_cmd(), 0)

        # Wait briefly and check new mode
        time.sleep(0.1)
        new_mode = self._high_ctrl.get_mode()

        return {
            "state": "switched",
            "requested": mode_str,
            "workmode": new_mode,
            "workmode_name": _WORKMODE_NAMES.get(new_mode, "unknown"),
        }

    def _do_get_mode(self) -> dict:
        mode = self._high_ctrl.get_mode()
        return {
            "workmode": mode,
            "workmode_name": _WORKMODE_NAMES.get(mode, "unknown"),
        }


# ── MicPlugin (sensor, subprocess) ────────────────────────────────────────────

def _mic_subprocess(namespace: str):
    """Mic capture subprocess — polls MediaController, publishes AudioChunk."""
    import os as _os
    _os.environ.setdefault('CYCLONEDDS_URI', 'file:///work/noetix_sdk_bumi/config/dds.xml')
    import sys as _sys
    _sys.path.insert(0, '/work/noetix_sdk_bumi/build')
    import time as _time
    import struct as _struct
    import numpy as _np

    import rclpy as _rclpy
    from rclpy.node import Node as _Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
    from audio_msgs.msg import AudioChunk as _AudioChunk

    _QOS = QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
        depth=200,
        durability=DurabilityPolicy.VOLATILE,
    )

    from mediacontrol_py import MediaController
    media_ctrl = MediaController.instance()
    media_ctrl.init()
    _time.sleep(3)

    _rclpy.init()
    node = _Node("bumi_mic_sub")
    topic = f"/{namespace}/mic/audio"
    pub = node.create_publisher(_AudioChunk, topic, _QOS)

    print(f"[mic_subprocess] publishing to {topic}", flush=True)

    frame_count = 0
    t_start = _time.monotonic()
    buffer = _np.array([], dtype=_np.int16)
    MIN_CHUNK_SAMPLES = 512  # 1024 bytes = 32ms @ 16kHz

    while True:
        try:
            audio = media_ctrl.get_audio_capture_data()
            if audio.channels == 0 or len(audio.audio_data) == 0:
                _time.sleep(0.005)
                continue

            # Downmix 8ch → mono (channel 0) using numpy for speed
            samples = _np.array(audio.audio_data, dtype=_np.int16)
            mono = samples[::audio.channels]

            # SDK returns low-amplitude signal (~8-bit dynamic range in 16-bit container)
            # Apply moderate gain to reach usable 16-bit level without clipping
            mono = _np.clip(mono.astype(_np.int32) * 50, -32768, 32767).astype(_np.int16)

            # Accumulate until we have enough for a proper chunk
            buffer = _np.concatenate([buffer, mono])

            if len(buffer) >= MIN_CHUNK_SAMPLES:
                msg = _AudioChunk()
                msg.format = "pcm_16k_16bit_mono"
                msg.data = buffer.tobytes()
                pub.publish(msg)
                buffer = _np.array([], dtype=_np.int16)

                frame_count += 1
                if frame_count % 200 == 0:
                    elapsed = _time.monotonic() - t_start
                    print(f"[mic_subprocess] {frame_count} chunks, {frame_count/elapsed:.1f} chunks/s", flush=True)
        except Exception as e:
            print(f"[mic_subprocess] error: {e}", flush=True)
            _time.sleep(0.5)


class MicPlugin:
    PREFIX = "mic"

    def __init__(self, plugin_config: dict, namespace: str, executor, media_ctrl):
        self._namespace = namespace
        self._topic = f"/{namespace}/mic/audio"
        self._proc: subprocess.Popen | None = None

    def get_tool(self) -> dict:
        return {
            "name": "mic",
            "type": "sensor",
            "multiInstance": False,
            "description": f"Bumi microphone — 8ch array, outputs mono PCM 16kHz 16bit. Publishes to {self._topic}",
            "inputSchema": {"type": "object", "properties": {}},
            "topic_out": [{"topic": self._topic, "format": "audio/pcm-16k"}],
        }

    def start(self) -> None:
        import sys
        self._proc = subprocess.Popen(
            [sys.executable, "-c",
             f"import sys; sys.path.insert(0, '/work'); from device import _mic_subprocess; _mic_subprocess({self._namespace!r})"],
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
        )
        # Forward subprocess stdout in background
        def _fwd():
            for line in self._proc.stdout:
                print(line.decode(errors='replace').rstrip(), flush=True)
        threading.Thread(target=_fwd, daemon=True).start()

    def stop(self) -> None:
        if self._proc:
            self._proc.terminate()
            self._proc = None

    def dispatch(self, action: str, args: dict) -> dict | None:
        if action == "start":
            return {"state": "running", "topic_out": [{"topic": self._topic, "format": "audio/pcm-16k"}]}
        if action == "stop":
            return {"state": "idle"}
        if action == "info":
            return {"state": "running" if self._proc and self._proc.poll() is None else "idle"}
        return None


# ── SpeakerPlugin (actuator) ─────────────────────────────────────────────────

class SpeakerPlugin:
    PREFIX = "speaker"

    def __init__(self, plugin_config: dict, namespace: str, executor, media_ctrl):
        self._media_ctrl = media_ctrl
        self._namespace = namespace
        self._node = Node("bumi_speaker")
        self._executor = executor
        executor.add_node(self._node)
        self._playing = False
        self._sub = None

    def get_tool(self) -> dict:
        return {
            "name": "speaker",
            "type": "actuator",
            "multiInstance": False,
            "description": "Bumi speaker — play audio from ROS2 topic on robot speaker, volume control, wake/sleep.",
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {
                        "type": "string",
                        "enum": ["play", "stop", "get_volume", "set_volume", "wakeup", "sleep"],
                    },
                    "input_topic": {
                        "type": "string",
                        "description": "ROS2 topic to subscribe for PCM audio data",
                    },
                    "volume": {
                        "type": "integer",
                        "description": "Volume level 0-200",
                        "minimum": 0, "maximum": 200,
                    },
                },
                "required": ["action"],
                "x-action-params": {
                    "play": {
                        "params": ["input_topic"],
                        "description": "Subscribe to audio topic and play through robot speaker",
                    },
                    "stop": {
                        "params": [],
                        "description": "Stop audio playback",
                    },
                    "get_volume": {
                        "params": [],
                        "description": "Get current volume (0-200)",
                    },
                    "set_volume": {
                        "params": ["volume"],
                        "description": "Set volume (0-200)",
                    },
                    "wakeup": {
                        "params": [],
                        "description": "Wake up robot audio agent",
                    },
                    "sleep": {
                        "params": [],
                        "description": "Put robot audio agent to sleep",
                    },
                },
            },
            "topic_in": [{"format": "audio/pcm-16k"}],
            "topic_out": [],
        }

    def start(self) -> None:
        pass

    def stop(self) -> None:
        self._playing = False

    def dispatch(self, action: str, args: dict) -> dict | None:
        args.pop('_tool_name', None)

        if action == "start":
            return {"state": "ready"}
        if action == "stop":
            self._playing = False
            self._media_ctrl.pause_audio_playback()
            return {"state": "idle"}
        if action == "play":
            return self._do_play(args)
        if action == "get_volume":
            vol = self._media_ctrl.get_volume()
            return {"volume": vol}
        if action == "set_volume":
            vol = int(args.get("volume", 100))
            self._media_ctrl.set_volume(vol)
            return {"volume": vol, "state": "set"}
        if action == "wakeup":
            self._media_ctrl.wakeup()
            return {"state": "awake"}
        if action == "sleep":
            self._media_ctrl.sleep()
            return {"state": "sleeping"}
        return None

    def _do_play(self, args: dict) -> dict:
        input_topic = args.get("input_topic", "")
        if not input_topic:
            return {"error": "input_topic is required"}

        self._playing = True
        self._media_ctrl.resume_audio_playback()

        # Subscribe to the audio topic
        def _on_audio(msg):
            if not self._playing:
                return
            try:
                import base64
                data = json.loads(msg.data)
                pcm_bytes = base64.b64decode(data["data"])
                # Convert mono to stereo (duplicate channel) for MediaController (2ch required)
                mono_samples = struct.unpack(f'<{len(pcm_bytes)//2}h', pcm_bytes)
                stereo_samples = []
                for s in mono_samples:
                    stereo_samples.extend([s, s])  # duplicate L=R

                # Create AudioStream and publish
                from mediacontrol_py import AudioStream
                stream = AudioStream()
                stream.channels = 2
                stream.sample_rate = 16000
                stream.format = 2
                stream.audio_data = stereo_samples
                self._media_ctrl.publish_external_audio_playback_stream(stream)
            except Exception as e:
                self._node.get_logger().warn(f"Speaker playback error: {e}")

        if self._sub is not None:
            self._node.destroy_subscription(self._sub)
        self._sub = self._node.create_subscription(String, input_topic, _on_audio, _LOW_LAT_QOS)

        return {"state": "playing", "input_topic": input_topic}


# ── CameraPlugin (sensor, subprocess) ────────────────────────────────────────

def _camera_subprocess(namespace: str):
    """Camera subprocess — captures Realsense D435i color+depth, publishes to ROS2."""
    import time as _time
    import numpy as _np

    import rclpy as _rclpy
    from rclpy.node import Node as _Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
    from sensor_msgs.msg import CompressedImage as _CompressedImage
    from sensor_msgs.msg import Image as _SensorImage

    _QOS = QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        durability=DurabilityPolicy.VOLATILE,
    )

    import pyrealsense2 as rs
    import cv2

    # Try turbojpeg for faster encoding, fallback to cv2
    try:
        from turbojpeg import TurboJPEG, TJPF_BGR
        _tj = TurboJPEG()
        def encode_jpeg(bgr_image):
            return _tj.encode(bgr_image, pixel_format=TJPF_BGR, quality=80)
        print("[camera_subprocess] using TurboJPEG encoder", flush=True)
    except Exception:
        def encode_jpeg(bgr_image):
            _, buf = cv2.imencode('.jpg', bgr_image, [cv2.IMWRITE_JPEG_QUALITY, 80])
            return buf.tobytes()
        print("[camera_subprocess] using cv2 JPEG encoder", flush=True)

    _rclpy.init()
    node = _Node("bumi_camera_sub")
    color_topic = f"/{namespace}/camera/color"
    depth_topic = f"/{namespace}/camera/depth"
    color_pub = node.create_publisher(_CompressedImage, color_topic, _QOS)
    depth_pub = node.create_publisher(_CompressedImage, depth_topic, _QOS)

    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

    try:
        pipeline.start(config)
    except Exception as e:
        print(f"[camera_subprocess] Realsense pipeline start failed: {e}", flush=True)
        return

    print(f"[camera_subprocess] publishing color→{color_topic} depth→{depth_topic}", flush=True)

    frame_count = 0
    t_start = _time.monotonic()
    try:
        while True:
            t0 = _time.monotonic()
            frames = pipeline.wait_for_frames(timeout_ms=1000)
            t_wait = _time.monotonic() - t0

            color_frame = frames.get_color_frame()
            if color_frame:
                color_image = _np.asanyarray(color_frame.get_data())
                t1 = _time.monotonic()
                jpeg_bytes = encode_jpeg(color_image)
                t_enc = _time.monotonic() - t1
                msg = _CompressedImage()
                msg.header.stamp = node.get_clock().now().to_msg()
                msg.format = "jpeg"
                msg.data = jpeg_bytes
                color_pub.publish(msg)

            depth_frame = frames.get_depth_frame()
            if depth_frame:
                depth_image = _np.asanyarray(depth_frame.get_data())
                import zlib as _zlib
                compressed = _zlib.compress(depth_image.tobytes(), 1)
                msg = _CompressedImage()
                msg.header.stamp = node.get_clock().now().to_msg()
                msg.format = "16UC1; compressedDepth zlib"
                msg.data = compressed
                depth_pub.publish(msg)

            frame_count += 1
            # Log every 300 frames (~15s at 20fps)
            if frame_count % 300 == 0:
                elapsed = _time.monotonic() - t_start
                fps = frame_count / elapsed
                print(f"[camera_subprocess] {frame_count} frames, {fps:.1f} fps, last: wait={t_wait*1000:.1f}ms enc={t_enc*1000:.1f}ms", flush=True)

            _time.sleep(0.001)  # yield CPU
    except Exception as e:
        print(f"[camera_subprocess] error: {e}", flush=True)
    finally:
        pipeline.stop()


class CameraPlugin:
    PREFIX = "camera"

    def __init__(self, plugin_config: dict, namespace: str, executor):
        self._namespace = namespace
        self._color_topic = f"/{namespace}/camera/color"
        self._depth_topic = f"/{namespace}/camera/depth"
        self._proc: subprocess.Popen | None = None

    def get_tools(self) -> list:
        return [
            {
                "name": "camera",
                "type": "sensor",
                "multiInstance": False,
                "description": f"Bumi Realsense D435i color camera — 640x480 JPEG @ 30fps. Publishes to {self._color_topic}",
                "inputSchema": {"type": "object", "properties": {}},
                "topic_out": [{"topic": self._color_topic, "format": "image/jpeg"}],
            },
            {
                "name": "depth",
                "type": "sensor",
                "multiInstance": False,
                "description": f"Bumi Realsense D435i depth camera — 640x480 zlib-compressed Z16 @ 30fps. Publishes to {self._depth_topic}",
                "inputSchema": {"type": "object", "properties": {}},
                "topic_out": [{"topic": self._depth_topic, "format": "image/depth-zlib"}],
            },
        ]

    def start(self) -> None:
        import sys
        self._proc = subprocess.Popen(
            [sys.executable, "-c",
             f"import sys; sys.path.insert(0, '/work'); from device import _camera_subprocess; _camera_subprocess({self._namespace!r})"],
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
        )
        def _fwd():
            for line in self._proc.stdout:
                print(line.decode(errors='replace').rstrip(), flush=True)
        threading.Thread(target=_fwd, daemon=True).start()

    def stop(self) -> None:
        if self._proc:
            self._proc.terminate()
            self._proc = None

    def dispatch(self, action: str, args: dict) -> dict | None:
        if action == "start":
            return {"state": "running"}
        if action == "stop":
            return {"state": "idle"}
        if action == "info":
            tool_name = args.get('_tool_name', '')
            if tool_name == "camera":
                return {"state": "running", "topic_out": [{"topic": self._color_topic, "format": "image/jpeg"}]}
            if tool_name == "depth":
                return {"state": "running", "topic_out": [{"topic": self._depth_topic, "format": "image/depth-zlib"}]}
            return {"state": "running"}
        return None


# ── Higher-level Bumi cards ──────────────────────────────────────────────────
#
# These cards intentionally live in device.py with the base device plugins so
# the Bumi bundle has a single implementation module.


_WORKMODE_NAMES = {
    0: "enabled", 1: "ready", 2: "walking", 5: "dance",
    8: "greet", 9: "shake", 10: "cheer", 11: "start_teach",
    12: "end_teach", 14: "save_teach_1", 23: "play_teach",
    26: "protection", 27: "fall_to_stand", 28: "stand_to_fall",
    29: "save_teach_2", 30: "disabled", 31: "dance1", 32: "dance2",
    33: "tear",
}

_MOTOR_ERROR_NAMES = {
    0x02: "overcurrent",
    0x03: "undervoltage",
    0x04: "encoder_error",
    0x06: "brake_voltage_high",
    0x07: "driver_error",
    0x08: "overvoltage",
    0x09: "undervoltage",
    0x0A: "overcurrent",
    0x0B: "mos_overtemperature",
    0x0C: "coil_overtemperature",
    0x0D: "communication_lost",
    0x0E: "overload",
}

_JOINT_NAMES_BY_ID = [
    "l_arm_pitch_joint", "l_arm_roll_joint", "l_arm_yaw_joint", "l_elbow_pitch_joint",
    "l_leg_pitch_joint", "l_leg_roll_joint", "l_leg_yaw_joint",
    "l_knee_pitch_joint", "l_ankle_pitch_joint", "l_ankle_roll_joint",
    "r_arm_pitch_joint", "r_arm_roll_joint", "r_arm_yaw_joint", "r_elbow_pitch_joint",
    "r_leg_pitch_joint", "r_leg_roll_joint", "r_leg_yaw_joint",
    "r_knee_pitch_joint", "r_ankle_pitch_joint", "r_ankle_roll_joint",
    "waist_yaw_joint",
]


def _wall_time_ms() -> int:
    return int(time.time() * 1000)


def _enum_name(value: Any) -> str:
    """Return a stable readable value for pybind enums and ordinary values."""
    name = getattr(value, "name", None)
    if name:
        return str(name)
    text = str(value)
    return text.rsplit(".", 1)[-1] if "." in text else text


def _finite_number(value: Any, field: str) -> float:
    if isinstance(value, bool):
        raise ValueError(f"{field} must be a number")
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{field} must be a number") from exc
    if not math.isfinite(result):
        raise ValueError(f"{field} must be finite")
    return result


def _header_payload(header: Any) -> dict:
    if header is None:
        return {}
    return {
        "message_id": getattr(header, "message_id", None),
        "source_timestamp_us": getattr(header, "timestamp_us", None),
        "sn": getattr(header, "sn", None),
    }


def _quaternion_xyzw_to_rpy(quaternion: list[float]) -> list[float] | None:
    """Convert the SDK's documented [x, y, z, w] quaternion to roll/pitch/yaw."""
    norm = math.sqrt(sum(value * value for value in quaternion))
    if norm < 1e-12:
        return None
    x, y, z, w = (value / norm for value in quaternion)
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = 2.0 * (w * y - z * x)
    pitch = math.copysign(math.pi / 2.0, sinp) if abs(sinp) >= 1.0 else math.asin(sinp)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return [round(roll, 6), round(pitch, 6), round(yaw, 6)]


class _MotionStateNode(Node):
    def __init__(self, namespace: str, high_ctrl, interval_s: float,
                 history_size: int, activity_velocity_threshold: float):
        super().__init__("bumi_motion_state")
        self._high_ctrl = high_ctrl
        self._topic = f"/{namespace}/motion/state"
        self._pub = self.create_publisher(String, self._topic, 10)
        self._interval_s = interval_s
        self._activity_velocity_threshold = activity_velocity_threshold
        self._history = deque(maxlen=history_size)
        self._snapshot: dict = {
            "state": "no_data", "fresh": False, "sample_age_ms": None,
            "reason": "waiting_for_high_controller",
        }
        self._sampled_monotonic: float | None = None
        self._last_signature: dict | None = None
        self._event_sequence = 0
        self._read_failed = False
        self._lock = threading.Lock()
        self._running = False
        self._thread = None

    @property
    def topic(self) -> str:
        return self._topic

    def start_polling(self):
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True, name="bumi_motion_state")
        self._thread.start()

    def stop_polling(self):
        self._running = False
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=1.0)

    def snapshot(self, detail: str = "summary") -> dict:
        with self._lock:
            result = dict(self._snapshot)
            sampled_monotonic = self._sampled_monotonic
        result["sample_age_ms"] = (
            max(0, int((time.monotonic() - sampled_monotonic) * 1000))
            if sampled_monotonic is not None else None
        )
        result["fresh"] = bool(
            result.get("state") not in ("error", "no_data")
            and result["sample_age_ms"] is not None
            and result["sample_age_ms"] <= max(1000, int(self._interval_s * 5_000))
        )
        if detail == "joints" and result.get("state") not in ("error", "no_data"):
            return {
                "state": result["state"],
                "fresh": result["fresh"],
                "sample_age_ms": result["sample_age_ms"],
                "source": result.get("source"),
                "joint_states": result.get("joint_states", []),
            }
        result.pop("joint_states", None)
        return result

    def history(self, limit: int) -> list[dict]:
        with self._lock:
            events = [dict(event) for event in list(self._history)[-limit:]]
        now = time.monotonic()
        for event in events:
            recorded = event.pop("_recorded_monotonic", None)
            event["age_ms"] = max(0, int((now - recorded) * 1000)) if recorded else None
        return events

    def clear_history(self):
        with self._lock:
            self._history.clear()

    def _append_event(self, event: str, **data):
        self._event_sequence += 1
        self._history.append({
            "sequence": self._event_sequence,
            "event": event,
            "_recorded_monotonic": time.monotonic(),
            **data,
        })

    def _record_changes(self, payload: dict, signature: dict):
        previous = self._last_signature
        if previous is None:
            self._last_signature = signature
            return
        if signature["workmode"] != previous["workmode"]:
            self._append_event(
                "workmode_changed",
                previous={"code": previous["workmode"], "name": previous["workmode_name"]},
                current={"code": signature["workmode"], "name": signature["workmode_name"]},
            )
        if signature["protection"] != previous["protection"]:
            self._append_event(
                "protection_changed",
                previous=previous["protection"], current=signature["protection"],
            )
        if signature["activity"] != previous["activity"]:
            self._append_event(
                "motion_started" if signature["activity"] == "moving" else "motion_stopped",
                previous=previous["activity"], current=signature["activity"],
                max_abs_joint_velocity=payload["joint_motion"]["max_abs_velocity"],
                activity_velocity_threshold=self._activity_velocity_threshold,
            )
        if signature["motor_faults"] != previous["motor_faults"]:
            self._append_event(
                "motor_faults_changed", faults=payload["motor_faults"],
            )
        self._last_signature = signature

    def _loop(self):
        while self._running:
            try:
                payload = self._read_once()
                signature = {
                    "workmode": payload["workmode"]["code"],
                    "workmode_name": payload["workmode"]["name"],
                    "protection": payload["workmode"]["protection"],
                    "activity": payload["activity"],
                    "motor_faults": tuple(
                        (item["motor_id"], item["error"]) for item in payload["motor_faults"]),
                }
                with self._lock:
                    if self._read_failed:
                        self._append_event("state_read_recovered")
                        self._read_failed = False
                    self._record_changes(payload, signature)
                    self._snapshot = payload
                    self._sampled_monotonic = time.monotonic()
                msg = String()
                msg.data = json.dumps(payload, ensure_ascii=False)
                self._pub.publish(msg)
                time.sleep(self._interval_s)
            except Exception as exc:
                error = {
                    "state": "error", "fresh": False, "sample_age_ms": None,
                    "reason": str(exc),
                }
                with self._lock:
                    self._snapshot = error
                    self._sampled_monotonic = None
                    if not self._read_failed:
                        self._append_event("state_read_error", reason=str(exc))
                        self._read_failed = True
                time.sleep(max(0.5, self._interval_s))

    def _read_once(self) -> dict:
        mode = int(self._high_ctrl.get_mode())
        imu = self._high_ctrl.get_imu_data()
        raw_joint_state = self._high_ctrl.get_joint_state()
        if len(raw_joint_state) != 21:
            raise RuntimeError(f"HighController returned {len(raw_joint_state)} joints, expected 21")

        quaternion = [float(imu.ori[index]) for index in range(4)]
        angular_velocity = [float(imu.angular_vel[index]) for index in range(3)]
        linear_acceleration = [float(imu.linear_acc[index]) for index in range(3)]
        joint_states = []
        faults = []
        unrecognized_statuses = []
        for index, joint in enumerate(raw_joint_state):
            motor_id = int(getattr(joint, "motor_id", index))
            error = int(getattr(joint, "error", 0))
            documented_fault = error in _MOTOR_ERROR_NAMES
            item = {
                "motor_id": motor_id,
                "joint": _JOINT_NAMES_BY_ID[index],
                "position": round(float(joint.pos), 6),
                "velocity": round(float(joint.vel), 6),
                "torque": round(float(joint.tau), 6),
                "temperature": int(joint.temperature),
                "error": error,
                "fault": documented_fault,
                "error_documented": error == 0 or documented_fault,
            }
            joint_states.append(item)
            if documented_fault:
                faults.append({
                    "motor_id": motor_id, "joint": _JOINT_NAMES_BY_ID[index],
                    "error": error,
                    "error_name": _MOTOR_ERROR_NAMES[error],
                    "temperature": int(joint.temperature),
                })
            elif error:
                unrecognized_statuses.append({
                    "motor_id": motor_id,
                    "joint": _JOINT_NAMES_BY_ID[index],
                    "raw_error": error,
                })

        absolute_velocities = [abs(item["velocity"]) for item in joint_states]
        max_velocity = max(absolute_velocities)
        most_active_index = absolute_velocities.index(max_velocity)
        moving = [item for item in joint_states
                  if abs(item["velocity"]) >= self._activity_velocity_threshold]

        return {
            "state": "completed",
            "fresh": True,
            "source": "Noetix HighController/CycloneDDS",
            "activity": "moving" if moving else "stationary",
            "workmode": {
                "code": mode, "name": _WORKMODE_NAMES.get(mode, "unknown"),
                "protection": mode == 26,
            },
            "body_motion": {
                "orientation": {
                    "quaternion_xyzw": [round(value, 8) for value in quaternion],
                    "roll_pitch_yaw_rad": _quaternion_xyzw_to_rpy(quaternion),
                },
                "angular_velocity": {
                    "xyz": [round(value, 6) for value in angular_velocity],
                    "magnitude": round(math.sqrt(sum(value * value for value in angular_velocity)), 6),
                },
                "linear_acceleration": {
                    "xyz": [round(value, 6) for value in linear_acceleration],
                    "magnitude": round(math.sqrt(sum(value * value for value in linear_acceleration)), 6),
                },
            },
            "joint_motion": {
                "joint_count": len(joint_states),
                "activity_velocity_threshold": self._activity_velocity_threshold,
                "moving_joint_count": len(moving),
                "moving_joints": [item["joint"] for item in moving],
                "max_abs_velocity": round(max_velocity, 6),
                "mean_abs_velocity": round(sum(absolute_velocities) / len(absolute_velocities), 6),
                "most_active_joint": {
                    "motor_id": joint_states[most_active_index]["motor_id"],
                    "joint": joint_states[most_active_index]["joint"],
                    "velocity": joint_states[most_active_index]["velocity"],
                },
            },
            "motor_faults": faults,
            "unrecognized_motor_statuses": unrecognized_statuses,
            "joint_states": joint_states,
        }


class MotionStatePlugin:
    PREFIX = "motion_state"

    def __init__(self, plugin_config: dict, namespace: str, executor, high_ctrl):
        interval = _finite_number(plugin_config.get("poll_interval_s", 0.5), "poll_interval_s")
        if not 0.02 <= interval <= 2.0:
            raise ValueError("poll_interval_s must be in [0.02, 2.0]")
        history_size = int(plugin_config.get("history_size", 100))
        if not 1 <= history_size <= 1000:
            raise ValueError("history_size must be in [1, 1000]")
        activity_threshold = _finite_number(
            plugin_config.get("activity_velocity_threshold", 0.15),
            "activity_velocity_threshold",
        )
        if not 0.001 <= activity_threshold <= 10.0:
            raise ValueError("activity_velocity_threshold must be in [0.001, 10.0]")
        self._node = _MotionStateNode(
            namespace, high_ctrl, interval, history_size, activity_threshold)
        executor.add_node(self._node)

    def get_tool(self) -> dict:
        return {
            "name": "motion_state", "type": "sensor", "multiInstance": False,
            "description": "Bumi whole-body motion telemetry — body orientation/dynamics, joint activity and motion events.",
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {
                        "type": "string",
                        "enum": ["snapshot", "history", "clear_history"],
                        "description": "操作：snapshot 获取当前运动遥测；history 查询近期运动事件；clear_history 清空内存中的事件记录。",
                    },
                    "detail": {
                        "type": "string", "enum": ["summary", "joints", "none"],
                        "default": "summary",
                        "description": "snapshot 时选择 summary（默认，整机运动摘要）或 joints（仅 21 个关节明细）；history 时必须选择 none；clear_history 无需设置。",
                    },
                    "limit": {
                        "type": "integer", "minimum": 1, "maximum": 100,
                        "default": 20,
                        "description": "仅用于 history，最多返回最近多少条事件；范围 1～100，未填写时默认 20。事件不足时返回全部已有事件。",
                    },
                },
                "required": ["action"],
                "x-action-params": {
                    "snapshot": {"params": ["detail"], "description": "获取当前运动遥测；summary 返回整机摘要，joints 仅返回 21 个关节完整明细。"},
                    "history": {"params": ["detail", "limit"], "description": "查询近期运动事件；detail 必须选择 none，limit 可选且默认返回 20 条。"},
                    "clear_history": {"params": [], "description": "清空内存中的运动事件记录；不需要任何参数。"},
                },
            },
            "topic_out": [{"topic": self._node.topic, "format": "data/json"}],
        }

    def start(self):
        self._node.start_polling()

    def stop(self):
        self._node.stop_polling()

    def dispatch(self, action: str, args: dict) -> dict:
        if action in ("start", "snapshot", "info"):
            detail = args.get("detail", "summary")
            if detail not in ("summary", "joints"):
                return {
                    "state": "error",
                    "error": "snapshot requires detail=summary or detail=joints; none is only for history",
                }
            return self._node.snapshot(detail)
        if action == "stop":
            return {"state": "idle"}
        if action == "history":
            if args.get("detail") != "none":
                return {
                    "state": "error",
                    "error": "history requires detail=none; select none in the detail field",
                }
            try:
                limit = int(args.get("limit", 20))
            except (TypeError, ValueError):
                return {"state": "error", "error": "limit must be an integer"}
            if not 1 <= limit <= 100:
                return {"state": "error", "error": "limit must be in [1, 100]"}
            return {"state": "completed", "events": self._node.history(limit)}
        if action == "clear_history":
            self._node.clear_history()
            return {"state": "completed"}
        return {"state": "error", "error": f"unknown action: {action}"}


@dataclass(frozen=True)
class _ArmJoint:
    name: str
    lower: float
    upper: float


_LEFT_ARM = (
    _ArmJoint("l_arm_pitch_joint", -2.36, 2.36),
    _ArmJoint("l_arm_roll_joint", -0.14, 1.94),
    _ArmJoint("l_arm_yaw_joint", -1.57, 1.57),
    _ArmJoint("l_elbow_pitch_joint", -2.26, 0.0),
)
_RIGHT_ARM = (
    _ArmJoint("r_arm_pitch_joint", -2.36, 2.36),
    _ArmJoint("r_arm_roll_joint", -1.94, 0.14),
    _ArmJoint("r_arm_yaw_joint", -1.57, 1.57),
    _ArmJoint("r_elbow_pitch_joint", -2.26, 0.0),
)


class ArmPlugin:
    """Guarded Bumi-EDU arm card backed by the whole-body LowController."""

    PREFIX = "arm"

    def __init__(self, plugin_config: dict, namespace: str, executor, low_ctrl, high_ctrl=None):
        self._ctrl = low_ctrl
        self._high_ctrl = high_ctrl
        self._write_enabled = plugin_config.get("write_enabled", False) is True
        self._speed_limit = plugin_config.get("verified_speed_limit_rad_s")
        self._update_hz = plugin_config.get("verified_trajectory_update_hz")
        self._kp = plugin_config.get("verified_joint_kp")
        self._kd = plugin_config.get("verified_joint_kd")
        self._position_tolerance = plugin_config.get("verified_position_tolerance_rad")
        self._feedback_timeout = plugin_config.get("verified_feedback_timeout_s")
        self._max_action_duration = plugin_config.get("verified_max_action_duration_s")
        self._takeover_verified = plugin_config.get("high_low_arbitration_verified", False) is True
        self._recovery_verified = plugin_config.get("low_control_recovery_verified", False) is True
        self._lock = threading.Lock()
        self._cancel = threading.Event()
        self._thread = None
        self._status = {"state": "idle", "write_enabled": self._write_enabled}
        self._joint_indices = self._resolve_arm_indices()

    def _resolve_arm_indices(self) -> dict[str, int]:
        result = {}
        for joint in (*_LEFT_ARM, *_RIGHT_ARM):
            try:
                index = int(self._ctrl.getJointsIndex(joint.name))
            except Exception:
                index = -1
            result[joint.name] = index
        return result

    def get_tool(self) -> dict:
        return {
            "name": "arm", "type": "actuator", "multiInstance": False,
            "description": "Bumi-EDU guarded 8-DOF dual-arm control. LowController write is disabled until a verified whole-body hold profile is configured.",
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {"type": "string", "enum": ["get_state", "get_limits", "move", "status", "cancel"]},
                    "left_positions": {"type": "array", "items": {"type": "number"}, "minItems": 4, "maxItems": 4,
                                       "description": "Radians: [shoulder_pitch, shoulder_roll, shoulder_yaw, elbow_pitch]"},
                    "right_positions": {"type": "array", "items": {"type": "number"}, "minItems": 4, "maxItems": 4,
                                        "description": "Radians: [shoulder_pitch, shoulder_roll, shoulder_yaw, elbow_pitch]"},
                    "speed": {"type": "number", "exclusiveMinimum": 0, "description": "Target joint speed in rad/s; bounded by verified driver configuration"},
                },
                "required": ["action"],
                "x-action-params": {
                    "get_state": {"params": [], "description": "Read current dual-arm motor state"},
                    "get_limits": {"params": [], "description": "Read URDF-derived arm joint limits"},
                    "move": {"params": ["left_positions", "right_positions", "speed"], "description": "Move one or both arms to target angles"},
                    "status": {"params": [], "description": "Get current arm action status"},
                    "cancel": {"params": [], "description": "Cancel target interpolation while retaining the last commanded hold"},
                },
            },
        }

    def start(self):
        pass

    def stop(self):
        self._cancel.set()
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=1.0)

    def dispatch(self, action: str, args: dict) -> dict:
        if action in ("start", "info", "status"):
            with self._lock:
                return dict(self._status)
        if action == "stop":
            self.stop()
            return {"state": "idle"}
        if action == "get_limits":
            return {"state": "completed", "unit": "rad", "source": "resource/bumi_model.urdf",
                    "left": [joint.__dict__ for joint in _LEFT_ARM],
                    "right": [joint.__dict__ for joint in _RIGHT_ARM]}
        if action == "get_state":
            return self._get_state()
        if action == "cancel":
            self._cancel.set()
            with self._lock:
                action_id = self._status.get("action_id")
                self._status = {"state": "cancelled", "action_id": action_id,
                                "note": "interpolation cancelled; LowController retains its last command"}
                return dict(self._status)
        if action == "move":
            return self._move(args)
        return {"state": "error", "error": f"unknown action: {action}"}

    def _get_state(self) -> dict:
        try:
            states = self._ctrl.get_joint_state()
            arms = {}
            for side, joints in (("left", _LEFT_ARM), ("right", _RIGHT_ARM)):
                values = []
                for joint in joints:
                    index = self._joint_indices[joint.name]
                    if not 0 <= index < len(states):
                        values.append({"name": joint.name, "available": False})
                        continue
                    state = states[index]
                    values.append({
                        "name": joint.name, "motor_id": int(getattr(state, "motor_id", index)),
                        "position": float(state.pos), "velocity": float(state.vel),
                        "torque": float(state.tau), "temperature": int(state.temperature),
                        "error": int(state.error), "available": True,
                    })
                arms[side] = values
            return {"state": "completed", "observed_at_ms": _wall_time_ms(), "source": "Noetix LowController/CycloneDDS", **arms}
        except Exception as exc:
            return {"state": "error", "error": str(exc)}

    def _validate_pose(self, values: Any, joints: tuple[_ArmJoint, ...], field: str) -> list[float] | None:
        if values is None:
            return None
        if not isinstance(values, (list, tuple)) or len(values) != 4:
            raise ValueError(f"{field} must contain exactly 4 values")
        result = [_finite_number(value, field) for value in values]
        violations = [
            {"joint": joint.name, "value": value, "lower": joint.lower, "upper": joint.upper}
            for joint, value in zip(joints, result) if not joint.lower <= value <= joint.upper
        ]
        if violations:
            raise ValueError(f"{field} exceeds URDF limits: {violations}")
        return result

    def _validated_write_profile(self) -> tuple[float, float, float, float, float, list[float], list[float]]:
        if not self._write_enabled:
            raise ValueError("arm write control is disabled; set arm.write_enabled only after the LowController takeover procedure is verified")
        if not self._takeover_verified or not self._recovery_verified:
            raise ValueError(
                "arm write control requires high_low_arbitration_verified=true and "
                "low_control_recovery_verified=true")
        speed_limit = _finite_number(self._speed_limit, "verified_speed_limit_rad_s")
        update_hz = _finite_number(self._update_hz, "verified_trajectory_update_hz")
        position_tolerance = _finite_number(self._position_tolerance, "verified_position_tolerance_rad")
        feedback_timeout = _finite_number(self._feedback_timeout, "verified_feedback_timeout_s")
        max_action_duration = _finite_number(
            self._max_action_duration, "verified_max_action_duration_s")
        if (speed_limit <= 0 or not 1 <= update_hz <= 500
                or position_tolerance <= 0 or feedback_timeout <= 0
                or max_action_duration <= 0):
            raise ValueError("verified speed, update, tolerance or feedback timeout values are invalid")
        if not isinstance(self._kp, list) or not isinstance(self._kd, list) or len(self._kp) != 21 or len(self._kd) != 21:
            raise ValueError("verified_joint_kp and verified_joint_kd must each contain 21 values")
        kp = [_finite_number(value, "verified_joint_kp") for value in self._kp]
        kd = [_finite_number(value, "verified_joint_kd") for value in self._kd]
        if any(value < 0 for value in (*kp, *kd)):
            raise ValueError("verified_joint_kp/kd cannot contain negative values")
        if any(not 0 <= index < 21 for index in self._joint_indices.values()):
            raise ValueError("one or more Bumi arm joints could not be resolved by LowController")
        return (speed_limit, update_hz, position_tolerance, feedback_timeout,
                max_action_duration, kp, kd)

    def _move(self, args: dict) -> dict:
        try:
            left = self._validate_pose(args.get("left_positions"), _LEFT_ARM, "left_positions")
            right = self._validate_pose(args.get("right_positions"), _RIGHT_ARM, "right_positions")
            if left is None and right is None:
                raise ValueError("left_positions or right_positions is required")
            (speed_limit, update_hz, position_tolerance, feedback_timeout,
             max_action_duration, kp, kd) = self._validated_write_profile()
            speed = _finite_number(args.get("speed"), "speed")
            if not 0 < speed <= speed_limit:
                raise ValueError(f"speed must be in (0, {speed_limit}] rad/s")
        except ValueError as exc:
            return {"state": "error", "error": str(exc), "code": "arm_validation_failed"}

        with self._lock:
            if self._thread and self._thread.is_alive():
                return {"state": "error", "error": "another arm action is running", "code": "arm_busy"}
            action_id = f"bumi_arm_{uuid4().hex[:12]}"
            self._cancel.clear()
            self._status = {"state": "accepted", "action_id": action_id}
            self._thread = threading.Thread(
                target=self._run_move,
                args=(action_id, left, right, speed, update_hz,
                      position_tolerance, feedback_timeout,
                      max_action_duration, kp, kd),
                daemon=True, name="bumi_arm_move",
            )
            self._thread.start()
            return dict(self._status)

    def _run_move(self, action_id, left, right, speed, update_hz,
                  position_tolerance, feedback_timeout, max_action_duration, kp, kd):
        try:
            from lowcontrol_py import MotorCmd
            if self._high_ctrl is not None and int(self._high_ctrl.get_mode()) == 26:
                raise RuntimeError("robot is in protection mode")
            states = self._ctrl.get_joint_state()
            if len(states) != 21:
                raise RuntimeError(f"LowController returned {len(states)} joints, expected 21")
            motor_errors = [
                {"motor_id": int(getattr(item, "motor_id", index)), "error": int(item.error)}
                for index, item in enumerate(states) if int(item.error)
            ]
            if motor_errors:
                raise RuntimeError(f"motor errors present: {motor_errors}")
            try:
                bms = self._ctrl.get_robot_bms_data()
                if int(bms.battery_alarm):
                    raise RuntimeError(f"battery alarm present: {int(bms.battery_alarm)}")
            except AttributeError:
                pass
            start = [float(item.pos) for item in states]
            target = list(start)
            target_indices = []
            for pose, joints in ((left, _LEFT_ARM), (right, _RIGHT_ARM)):
                if pose is None:
                    continue
                for value, joint in zip(pose, joints):
                    index = self._joint_indices[joint.name]
                    target[index] = value
                    target_indices.append(index)
            duration = max(abs(goal - initial) for goal, initial in zip(target, start)) / speed
            if duration > max_action_duration:
                raise RuntimeError(
                    f"planned arm trajectory duration {duration:.3f}s exceeds verified "
                    f"maximum {max_action_duration:.3f}s")
            steps = max(1, int(math.ceil(duration * update_hz)))
            with self._lock:
                self._status = {"state": "running", "action_id": action_id, "steps": steps}
            for step in range(1, steps + 1):
                if self._cancel.is_set():
                    return
                ratio = step / steps
                commands = []
                for index in range(21):
                    cmd = MotorCmd()
                    cmd.motor_id = index
                    cmd.pos = start[index] + (target[index] - start[index]) * ratio
                    cmd.vel = 0.0
                    cmd.tau = 0.0
                    cmd.kp = kp[index]
                    cmd.kd = kd[index]
                    commands.append(cmd)
                self._ctrl.set_joint(commands)
                time.sleep(1.0 / update_hz)
            deadline = time.monotonic() + feedback_timeout
            max_error = float("inf")
            while not self._cancel.is_set() and time.monotonic() < deadline:
                actual = self._ctrl.get_joint_state()
                if len(actual) != 21:
                    raise RuntimeError(f"LowController returned {len(actual)} joints during feedback")
                feedback_errors = [
                    {"motor_id": int(getattr(item, "motor_id", index)), "error": int(item.error)}
                    for index, item in enumerate(actual) if int(item.error)
                ]
                if feedback_errors:
                    raise RuntimeError(f"motor errors during motion: {feedback_errors}")
                max_error = max(abs(float(actual[i].pos) - target[i]) for i in target_indices)
                if max_error <= position_tolerance:
                    break
                time.sleep(min(0.05, 1.0 / update_hz))
            if self._cancel.is_set():
                return
            if max_error > position_tolerance:
                raise RuntimeError(
                    f"arm feedback timeout: max error {max_error:.6f} rad exceeds "
                    f"verified tolerance {position_tolerance:.6f} rad")
            with self._lock:
                self._status = {"state": "completed", "action_id": action_id,
                                "feedback_verified": True,
                                "max_position_error_rad": max_error}
        except Exception as exc:
            with self._lock:
                self._status = {"state": "error", "action_id": action_id, "error": str(exc)}


class MediaSystemPlugin:
    PREFIX = "media_system"
    _SET_INTERVAL_S = 0.5
    _BOOL_FIELDS = {
        "audio_cue": ("set_audio_cue_enable", "get_audio_cue_enable"),
        "internal_capture_audio_to_agent": ("set_internal_capture_audio_data_to_agent_enable", "get_internal_capture_audio_data_to_agent_enable"),
        "external_audio_to_agent": ("set_external_custom_audio_data_to_agent_enable", "get_external_custom_audio_data_to_agent_enable"),
        "internal_agent_audio_to_playback": ("set_internal_agent_audio_data_to_playback_enable", "get_internal_agent_audio_data_to_playback_enable"),
        "external_audio_to_playback": ("set_external_custom_audio_data_to_playback_enable", "get_external_custom_audio_data_to_playback_enable"),
        "internal_video_to_agent": ("set_internal_capture_video_data_to_agent_enable", "get_internal_capture_video_data_to_agent_enable"),
        "external_video_to_agent": ("set_external_custom_video_data_to_agent_enable", "get_external_custom_video_data_to_agent_enable"),
        "external_audio_use_internal_3a": ("set_external_custom_audio_data_to_agent_use_internal_3a", "get_external_custom_audio_data_to_agent_use_internal_3a"),
    }

    def __init__(self, plugin_config: dict, namespace: str, executor, media_ctrl):
        self._ctrl = media_ctrl
        self._set_lock = threading.Lock()
        self._last_set = 0.0

    def get_tool(self) -> dict:
        config_props = {
            "timeout_ms": {
                "type": "integer", "minimum": 0,
                "description": "媒体 Agent 的超时配置，单位毫秒；必须为非负整数。",
            },
            "wakeup_response": {
                "type": "string", "maxLength": 256,
                "description": "唤醒后使用的回复文本，不是唤醒词；最长 256 个字符。",
            },
            "sleep_response": {
                "type": "string", "maxLength": 256,
                "description": "进入休眠时使用的回复文本；最长 256 个字符。",
            },
            "audio_cue": {
                "type": "boolean",
                "description": "是否启用媒体系统提示音。",
            },
            "internal_capture_audio_to_agent": {
                "type": "boolean",
                "description": "是否将机器人内部采集的音频发送给媒体 Agent。",
            },
            "external_audio_to_agent": {
                "type": "boolean",
                "description": "是否允许外部程序提供的自定义音频发送给媒体 Agent。",
            },
            "internal_agent_audio_to_playback": {
                "type": "boolean",
                "description": "是否将媒体 Agent 内部生成的音频发送到机器人播放端。",
            },
            "external_audio_to_playback": {
                "type": "boolean",
                "description": "是否允许外部程序提供的自定义音频发送到机器人播放端。",
            },
            "internal_video_to_agent": {
                "type": "boolean",
                "description": "是否将机器人内部采集的视频发送给媒体 Agent。",
            },
            "external_video_to_agent": {
                "type": "boolean",
                "description": "是否允许外部程序提供的自定义视频发送给媒体 Agent。",
            },
            "external_audio_use_internal_3a": {
                "type": "boolean",
                "description": "外部自定义音频发送给 Agent 前是否使用机器人内部 3A 音频处理。",
            },
        }
        return {
            "name": "media_system", "type": "actuator", "multiInstance": False,
            "description": "管理 Bumi 媒体 Agent：查询状态和配置、控制唤醒/休眠/重启，以及暂停或恢复麦克风、扬声器和视频采集通道。",
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {
                        "type": "string",
                        "enum": ["status", "get_config", "set_config", "get_wake_words", "wakeup", "sleep", "restart", "pause", "resume"],
                        "description": "status=查媒体 Agent 状态；get/set_config=查/改配置；get_wake_words=查唤醒词；wakeup/sleep/restart=唤醒/休眠/重启媒体 Agent；pause/resume=暂停/恢复 stream 指定的媒体通道。",
                    },
                    "config": {
                        "type": "object", "properties": config_props,
                        "additionalProperties": False, "minProperties": 1,
                        "description": "set_config 专用，请填写 JSON，例如 {\"audio_cue\":true,\"timeout_ms\":30000}；可填一个或多个字段，未填写项保持不变。",
                    },
                    "stream": {
                        "type": "string",
                        "enum": ["audio_capture", "audio_playback", "video_capture", "all"],
                        "description": "仅用于 pause/resume：audio_capture=麦克风采集，audio_playback=扬声器播放，video_capture=视频采集，all=以上全部。",
                    },
                },
                "required": ["action"],
                "x-action-params": {
                    "status": {"params": [], "description": "查询媒体 Agent 当前状态、状态变化原因和系统错误。"},
                    "get_config": {"params": [], "description": "读取卡片支持的全部媒体配置。"},
                    "set_config": {"params": ["config"], "description": "修改 config 中填写的一个或多个配置，并回读修改后的完整配置。"},
                    "get_wake_words": {"params": [], "description": "读取当前唤醒词；不能通过此操作修改唤醒词。"},
                    "wakeup": {"params": [], "description": "唤醒 Bumi 媒体 Agent，使其进入可语音交互状态；返回 accepted 后调用 status 确认。"},
                    "sleep": {"params": [], "description": "让 Bumi 媒体 Agent 休眠，语音交互将暂停；返回 accepted 后调用 status 确认。"},
                    "restart": {"params": [], "description": "重启 Bumi 媒体 Agent，期间音视频交互会暂时中断；之后调用 status 确认恢复。"},
                    "pause": {"params": ["stream"], "description": "暂停 Bumi 的 stream 通道：麦克风采集、扬声器播放、视频采集或全部。"},
                    "resume": {"params": ["stream"], "description": "恢复之前暂停的 Bumi stream 通道，使相应采集或播放继续。"},
                },
            },
        }

    def start(self):
        pass

    def stop(self):
        pass

    def dispatch(self, action: str, args: dict) -> dict:
        try:
            if action in ("start", "info", "status"):
                return self._status()
            if action == "stop":
                return {"state": "idle"}
            if action == "get_config":
                return {"state": "completed", "config": self._get_config()}
            if action == "get_wake_words":
                return {"state": "completed", "wake_words": str(self._ctrl.get_wakeup_words())}
            if action == "set_config":
                return self._set_config(args.get("config"))
            if action in ("wakeup", "sleep", "restart"):
                self._rate_limited_call(getattr(self._ctrl, action))
                return {"state": "accepted", "requested": action, "media_status": self._status()}
            if action in ("pause", "resume"):
                return self._stream_control(action, args.get("stream"))
            return {"state": "error", "error": f"unknown action: {action}"}
        except Exception as exc:
            return {"state": "error", "error": str(exc)}

    def _status(self) -> dict:
        status = self._ctrl.get_system_status()
        error = self._ctrl.get_system_error()
        return {
            "state": "completed", "observed_at_ms": _wall_time_ms(),
            "source": "Noetix MediaController/CycloneDDS",
            "status": _enum_name(status.value), "reason": _enum_name(status.reason),
            "status_header": _header_payload(getattr(status, "header", None)),
            "error": {"code": int(error.code), "message": str(error.message),
                      "header": _header_payload(getattr(error, "header", None))},
        }

    def _get_config(self) -> dict:
        config = {
            "timeout_ms": int(self._ctrl.get_timeout()),
            "wakeup_response": str(self._ctrl.get_wakeup_response_words()),
            "sleep_response": str(self._ctrl.get_sleep_response_words()),
        }
        for name, (_, getter) in self._BOOL_FIELDS.items():
            config[name] = bool(getattr(self._ctrl, getter)())
        return config

    def _rate_limited_call(self, fn, *args):
        with self._set_lock:
            wait = self._SET_INTERVAL_S - (time.monotonic() - self._last_set)
            if wait > 0:
                time.sleep(wait)
            result = fn(*args)
            self._last_set = time.monotonic()
            return result

    def _set_config(self, config: Any) -> dict:
        if isinstance(config, str):
            try:
                config = json.loads(config)
            except json.JSONDecodeError as exc:
                return {
                    "state": "error",
                    "error": (
                        "config must be valid JSON, for example "
                        "{\"audio_cue\":true,\"timeout_ms\":30000}; "
                        f"parse error: {exc.msg}"
                    ),
                }
        if not isinstance(config, dict) or not config:
            return {
                "state": "error",
                "error": (
                    "config must be a non-empty JSON object, for example "
                    "{\"audio_cue\":true,\"timeout_ms\":30000}"
                ),
            }
        allowed = {"timeout_ms", "wakeup_response", "sleep_response", *self._BOOL_FIELDS.keys()}
        unknown = sorted(set(config) - allowed)
        if unknown:
            return {"state": "error", "error": f"unsupported config fields: {unknown}"}
        setters = []
        for name, value in config.items():
            if name in self._BOOL_FIELDS:
                if type(value) is not bool:
                    return {"state": "error", "error": f"{name} must be boolean"}
                setters.append((getattr(self._ctrl, self._BOOL_FIELDS[name][0]), value))
            elif name == "timeout_ms":
                if isinstance(value, bool) or not isinstance(value, int) or value < 0:
                    return {"state": "error", "error": "timeout_ms must be a non-negative integer"}
                setters.append((self._ctrl.set_timeout, value))
            else:
                if not isinstance(value, str) or len(value) > 256:
                    return {"state": "error", "error": f"{name} must be a string of at most 256 characters"}
                setter = self._ctrl.set_wakeup_response_words if name == "wakeup_response" else self._ctrl.set_sleep_response_words
                setters.append((setter, value))
        for setter, value in setters:
            self._rate_limited_call(setter, value)
        return {"state": "completed", "config": self._get_config()}

    def _stream_control(self, action: str, stream: Any) -> dict:
        if stream not in ("audio_capture", "audio_playback", "video_capture", "all"):
            return {"state": "error", "error": "stream must be audio_capture, audio_playback, video_capture or all"}
        prefix = "pause" if action == "pause" else "resume"
        selected = ("audio_capture", "audio_playback", "video_capture") if stream == "all" else (stream,)
        for item in selected:
            self._rate_limited_call(getattr(self._ctrl, f"{prefix}_{item}"))
        return {"state": "completed", "action": action, "streams": list(selected)}


class DiagnosticsPlugin:
    PREFIX = "diagnostics"

    def __init__(self, plugin_config: dict, namespace: str, executor, high_ctrl=None,
                 media_ctrl=None, low_ctrl=None, plugins: list | None = None):
        self._high = high_ctrl
        self._media = media_ctrl
        self._low = low_ctrl
        self._plugins = plugins or []
        self._last_report = None

    def get_tool(self) -> dict:
        return {
            "name": "diagnostics", "type": "sensor", "multiInstance": False,
            "description": "Bumi low-risk self-test — motion controller, media controller, microphone and camera process health.",
            "inputSchema": {
                "type": "object",
                "properties": {"action": {"type": "string", "enum": ["quick_check", "motion_check", "media_check", "vision_check", "full_check", "report"]}},
                "required": ["action"],
                "x-action-params": {
                    "quick_check": {"params": [], "description": "Run read-only controller and process checks"},
                    "motion_check": {"params": [], "description": "Check HighController, mode, BMS and motor errors without moving"},
                    "media_check": {"params": [], "description": "Check MediaController and microphone process without playback"},
                    "vision_check": {"params": [], "description": "Check camera process health without changing capture"},
                    "full_check": {"params": [], "description": "Run every low-risk read-only check"},
                    "report": {"params": [], "description": "Return the most recent diagnostic report"},
                },
            },
        }

    def start(self):
        pass

    def stop(self):
        pass

    def dispatch(self, action: str, args: dict) -> dict:
        if action in ("start", "info", "report"):
            return self._last_report or {"state": "no_data", "reason": "no diagnostic check has run"}
        if action == "stop":
            return {"state": "idle"}
        groups = {
            "quick_check": ("motion", "media", "vision"),
            "full_check": ("motion", "media", "vision"),
            "motion_check": ("motion",),
            "media_check": ("media",),
            "vision_check": ("vision",),
        }.get(action)
        if groups is None:
            return {"state": "error", "error": f"unknown action: {action}"}
        checks = []
        if "motion" in groups:
            checks.extend(self._motion_checks())
        if "media" in groups:
            checks.extend(self._media_checks())
        if "vision" in groups:
            checks.extend(self._vision_checks())
        result = "failed" if any(item["status"] == "failed" for item in checks) else (
            "warning" if any(item["status"] == "warning" for item in checks) else "passed")
        self._last_report = {"state": "completed", "result": result,
                             "observed_at_ms": _wall_time_ms(), "checks": checks}
        return self._last_report

    @staticmethod
    def _check(name: str, status: str, message: str, **data) -> dict:
        return {"name": name, "status": status, "message": message, **data}

    def _motion_checks(self) -> list[dict]:
        if self._high is None:
            return [self._check("high_controller", "failed", "HighController unavailable")]
        checks = []
        try:
            mode = int(self._high.get_mode())
            checks.append(self._check("high_controller", "passed", "state reads succeeded", workmode=mode,
                                      workmode_name=_WORKMODE_NAMES.get(mode, "unknown")))
            if mode == 26:
                checks.append(self._check("protection_mode", "failed", "robot is in protection mode"))
            bms = self._high.get_robot_bms_data()
            alarm = int(bms.battery_alarm)
            checks.append(self._check("bms_alarm", "failed" if alarm else "passed",
                                      "battery alarm present" if alarm else "no battery alarm", alarm=alarm))
            states = self._high.get_joint_state()
            errors = [{
                "motor_id": int(getattr(item, "motor_id", index)),
                "error": int(item.error),
                "error_name": _MOTOR_ERROR_NAMES[int(item.error)],
            } for index, item in enumerate(states)
                if int(item.error) in _MOTOR_ERROR_NAMES]
            unrecognized = [{
                "motor_id": int(getattr(item, "motor_id", index)),
                "raw_error": int(item.error),
            } for index, item in enumerate(states)
                if int(item.error) and int(item.error) not in _MOTOR_ERROR_NAMES]
            checks.append(self._check("motor_errors", "failed" if errors else "passed",
                                      f"{len(errors)} motor error(s)" if errors else "no motor errors", errors=errors))
            if unrecognized:
                checks.append(self._check(
                    "unrecognized_motor_statuses", "warning",
                    f"{len(unrecognized)} undocumented non-zero motor status value(s); not classified as faults",
                    statuses=unrecognized,
                ))
        except Exception as exc:
            checks.append(self._check("high_controller_read", "failed", str(exc)))
        if self._low is None:
            checks.append(self._check(
                "low_controller", "failed", "LowController unavailable; arm card cannot be loaded"))
        else:
            try:
                low_states = self._low.get_joint_state()
                checks.append(self._check(
                    "low_controller", "passed" if len(low_states) == 21 else "failed",
                    f"received {len(low_states)} joint states", joint_count=len(low_states)))
            except Exception as exc:
                checks.append(self._check("low_controller", "failed", str(exc)))
        return checks

    def _media_checks(self) -> list[dict]:
        checks = []
        if self._media is None:
            checks.append(self._check("media_controller", "failed", "MediaController unavailable"))
        else:
            try:
                status = self._media.get_system_status()
                error = self._media.get_system_error()
                code = int(error.code)
                checks.append(self._check("media_controller", "failed" if code else "passed",
                                          f"status={_enum_name(status.value)} error={code}",
                                          status_value=_enum_name(status.value), error_code=code,
                                          error_message=str(error.message)))
            except Exception as exc:
                checks.append(self._check("media_controller", "failed", str(exc)))
        mic = next((item for item in self._plugins if getattr(item, "PREFIX", "") == "mic"), None)
        if mic is not None:
            alive = bool(getattr(mic, "_proc", None) and mic._proc.poll() is None)
            checks.append(self._check("microphone_process", "passed" if alive else "failed",
                                      "process running" if alive else "process not running"))
        return checks

    def _vision_checks(self) -> list[dict]:
        camera = next((item for item in self._plugins if getattr(item, "PREFIX", "") == "camera"), None)
        if camera is None:
            return [self._check("camera_process", "warning", "camera plugin not loaded")]
        proc = getattr(camera, "_proc", None)
        alive = bool(proc and proc.poll() is None)
        return [self._check("camera_process", "passed" if alive else "failed",
                            "process running" if alive else "process not running")]
