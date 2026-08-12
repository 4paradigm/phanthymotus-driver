#!/usr/bin/env python3
"""
drivers/noetix/bumi/device.py — Noetix Bumi-EDU 设备插件实现。

插件列表：
  - StatePlugin: joints (21-DOF skeleton), imu, battery, joy, model (URDF resource)
  - LocoPlugin: loco (move/stop), switch_mode (mode transitions)
  - MicPlugin: 8ch mic capture → mono PCM 16kHz
  - SpeakerPlugin: audio playback via MediaController
  - CameraPlugin: Realsense D435i color + depth
  - MediaSystemPlugin: media_system (wakeup/sleep/restart/status/errors)
  - MediaConfigPlugin: media_config (volume/timeout/enables/3A/pause/resume)
  - WakeupWordsPlugin: wakeup_words (wakeup/response/sleep response words)
  - AudioPlaybackPlugin: audio_playback (agent→speaker playback stream monitor)
  - VideoPlugin: video_capture (internal camera), video_external (external video push)
  - LowCmdPlugin: low_cmd (LowController set_joint, subprocess)
  - RlPolicyPlugin: rl_policy (ONNX RL walking, subprocess)
"""

import json
import os
import struct
import subprocess
import threading
import time
from pathlib import Path

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
    _JOY_INTERVAL    = 0.1     # 10 Hz

    def __init__(self, namespace: str, high_ctrl):
        super().__init__("bumi_state")
        self._high_ctrl = high_ctrl
        self._imu_topic     = f"/{namespace}/state/imu"
        self._battery_topic = f"/{namespace}/state/battery"
        self._joints_topic  = f"/{namespace}/state/joints"
        self._joy_topic     = f"/{namespace}/state/joy"

        self._imu_pub     = self.create_publisher(String, self._imu_topic,     _LOW_LAT_QOS)
        self._battery_pub = self.create_publisher(String, self._battery_topic, _LOW_LAT_QOS)
        self._joints_pub  = self.create_publisher(String, self._joints_topic,  _LOW_LAT_QOS)
        self._joy_pub     = self.create_publisher(String, self._joy_topic,      _LOW_LAT_QOS)

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
        last_joy_time = 0.0

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

                # Joy: 10 Hz
                if now - last_joy_time >= self._JOY_INTERVAL:
                    last_joy_time = now
                    joy = self._high_ctrl.from_dds_get_joydata()
                    joy_data = {
                        "buttons": [int(joy.btnA[i]) for i in range(4)],
                        "axes": {
                            "lx": round(float(joy.lx), 4),
                            "rx": round(float(joy.rx), 4),
                            "ly": round(float(joy.ly), 4),
                            "ry": round(float(joy.ry), 4),
                        },
                    }
                    msg = String()
                    msg.data = json.dumps(joy_data)
                    self._joy_pub.publish(msg)

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
                "name": "joy",
                "type": "sensor",
                "multiInstance": False,
                "description": f"Bumi joystick — 4 buttons + 4 axes (lx,rx,ly,ry). Publishes at 10Hz to /{ns}/state/joy",
                "inputSchema": {"type": "object", "properties": {}},
                "topic_out": [{"topic": f"/{ns}/state/joy", "format": "data/json"}],
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


# ── MediaSystemPlugin (system) ───────────────────────────────────────────────

class MediaSystemPlugin:
    """MediaController system control — wakeup/sleep/restart, status, errors."""

    PREFIX = "media_system"

    def __init__(self, plugin_config: dict, namespace: str, executor, media_ctrl):
        self._media_ctrl = media_ctrl

    def get_tool(self) -> dict:
        return {
            "name": "media_system",
            "type": "system",
            "multiInstance": False,
            "description": "Bumi media system — wakeup/sleep/restart audio agent, query work status, system status, and errors.",
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {
                        "type": "string",
                        "enum": ["read", "wakeup", "sleep", "restart"],
                    },
                },
                "required": ["action"],
                "x-action-params": {
                    "read": {"params": [], "description": "Query work status, system status, and error codes"},
                    "wakeup": {"params": [], "description": "Wake up audio agent"},
                    "sleep": {"params": [], "description": "Put audio agent to sleep"},
                    "restart": {"params": [], "description": "Restart audio agent"},
                },
            },
            "topic_in": [],
            "topic_out": [],
        }

    def start(self) -> None:
        pass

    def stop(self) -> None:
        pass

    def dispatch(self, action: str, args: dict) -> dict | None:
        args.pop('_tool_name', None)

        if action == "start":
            return {"state": "ready"}
        if action == "stop":
            return {"state": "idle"}
        if action == "read":
            return {
                "work_status": self._media_ctrl.get_work_status(),
                "system_status": self._media_ctrl.get_system_status(),
                "system_error": self._media_ctrl.get_system_error(),
            }
        if action == "wakeup":
            self._media_ctrl.wakeup()
            return {"state": "awake"}
        if action == "sleep":
            self._media_ctrl.sleep()
            return {"state": "sleeping"}
        if action == "restart":
            self._media_ctrl.restart()
            return {"state": "restarting"}
        return None


# ── MediaConfigPlugin (system) ──────────────────────────────────────────────

class MediaConfigPlugin:
    """MediaController configuration — volume, timeout, audio cue, 6 routing
    enables, 3A processing, pause/resume."""

    PREFIX = "media_config"

    def __init__(self, plugin_config: dict, namespace: str, executor, media_ctrl):
        self._media_ctrl = media_ctrl
        self._last_set_ms = 0

    def get_tool(self) -> dict:
        return {
            "name": "media_config",
            "type": "system",
            "multiInstance": False,
            "description": "Bumi media configuration — volume, timeout, audio cue, 6 audio/video routing enables, 3A processing, pause/resume playback.",
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {
                        "type": "string",
                        "enum": [
                            "read", "set_volume", "set_timeout", "set_audio_cue",
                            "set_mic_to_agent", "set_agent_to_speaker",
                            "set_ext_audio_3a", "pause_playback", "resume_playback",
                        ],
                    },
                    "volume": {"type": "integer", "description": "0-200", "minimum": 0, "maximum": 200},
                    "timeout_ms": {"type": "integer", "description": "Sleep timeout in ms"},
                    "enable": {"type": "boolean", "description": "true=on, false=off"},
                },
                "required": ["action"],
                "x-action-params": {
                    "read": {"params": [], "description": "Read all config values"},
                    "set_volume": {"params": ["volume"], "description": "Set speaker volume (0-200)"},
                    "set_timeout": {"params": ["timeout_ms"], "description": "Set auto-sleep timeout (ms)"},
                    "set_audio_cue": {"params": ["enable"], "description": "Toggle system audio cue/beep"},
                    "set_mic_to_agent": {"params": ["enable"], "description": "Toggle internal mic capture → agent"},
                    "set_agent_to_speaker": {"params": ["enable"], "description": "Toggle agent audio → speaker playback"},
                    "set_ext_audio_3a": {"params": ["enable"], "description": "Toggle external audio 3A (noise reduction/echo cancel)"},
                    "pause_playback": {"params": [], "description": "Pause audio playback"},
                    "resume_playback": {"params": [], "description": "Resume audio playback"},
                },
            },
            "topic_in": [],
            "topic_out": [],
        }

    def start(self) -> None:
        pass

    def stop(self) -> None:
        pass

    def dispatch(self, action: str, args: dict) -> dict | None:
        args.pop('_tool_name', None)

        if action == "start":
            return {"state": "ready"}
        if action == "stop":
            return {"state": "idle"}
        if action == "read":
            return {
                "volume": self._media_ctrl.get_volume(),
                "timeout_ms": self._media_ctrl.get_timeout(),
                "audio_cue_enable": self._media_ctrl.get_audio_cue_enable(),
                "mic_to_agent": self._media_ctrl.get_internal_capture_audio_data_to_agent_enable(),
                "agent_to_speaker": self._media_ctrl.get_internal_agent_audio_data_to_playback_enable(),
                "ext_audio_3a": self._media_ctrl.get_external_custom_audio_data_to_agent_use_internal_3a(),
            }
        if action == "pause_playback":
            self._media_ctrl.pause_audio_playback()
            return {"state": "paused"}
        if action == "resume_playback":
            self._media_ctrl.resume_audio_playback()
            return {"state": "resumed"}

        # Rate-limit all set actions to >=500ms
        now_ms = int(time.time() * 1000)
        elapsed = now_ms - self._last_set_ms
        if self._last_set_ms > 0 and elapsed < 500:
            return {"error": f"rate limited: {elapsed}ms since last set, need >=500ms"}
        self._last_set_ms = now_ms

        if action == "set_volume":
            vol = int(args.get("volume", 100))
            self._media_ctrl.set_volume(vol)
            return {"volume": vol, "state": "set"}
        if action == "set_timeout":
            timeout_ms = int(args.get("timeout_ms", 30000))
            self._media_ctrl.set_timeout(timeout_ms)
            return {"timeout_ms": timeout_ms, "state": "set"}
        if action == "set_audio_cue":
            enable = bool(args.get("enable", True))
            self._media_ctrl.set_audio_cue_enable(enable)
            return {"audio_cue_enable": enable, "state": "set"}
        if action == "set_mic_to_agent":
            enable = bool(args.get("enable", True))
            self._media_ctrl.set_internal_capture_audio_data_to_agent_enable(enable)
            return {"mic_to_agent": enable, "state": "set"}
        if action == "set_agent_to_speaker":
            enable = bool(args.get("enable", True))
            self._media_ctrl.set_internal_agent_audio_data_to_playback_enable(enable)
            return {"agent_to_speaker": enable, "state": "set"}
        if action == "set_ext_audio_3a":
            enable = bool(args.get("enable", True))
            self._media_ctrl.set_external_custom_audio_data_to_agent_use_internal_3a(enable)
            return {"ext_audio_3a": enable, "state": "set"}
        return None


# ── WakeupWordsPlugin (system) ───────────────────────────────────────────────

class WakeupWordsPlugin:
    """MediaController wakeup/response words configuration."""

    PREFIX = "wakeup_words"

    def __init__(self, plugin_config: dict, namespace: str, executor, media_ctrl):
        self._media_ctrl = media_ctrl

    def get_tool(self) -> dict:
        return {
            "name": "wakeup_words",
            "type": "system",
            "multiInstance": False,
            "description": "Bumi wakeup words — configure wakeup word, response word, and sleep response word.",
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {
                        "type": "string",
                        "enum": ["read", "set_wakeup_word", "set_response_word", "set_sleep_response_word"],
                    },
                    "word": {"type": "string", "description": "Wakeup/response phrase"},
                },
                "required": ["action"],
                "x-action-params": {
                    "read": {"params": [], "description": "Read current wakeup, response, and sleep response words"},
                    "set_wakeup_word": {"params": ["word"], "description": "Set wakeup word phrase"},
                    "set_response_word": {"params": ["word"], "description": "Set response word phrase"},
                    "set_sleep_response_word": {"params": ["word"], "description": "Set sleep response word phrase"},
                },
            },
            "topic_in": [],
            "topic_out": [],
        }

    def start(self) -> None:
        pass

    def stop(self) -> None:
        pass

    def dispatch(self, action: str, args: dict) -> dict | None:
        args.pop('_tool_name', None)

        if action == "start":
            return {"state": "ready"}
        if action == "stop":
            return {"state": "idle"}
        if action == "read":
            return {
                "wakeup_word": self._media_ctrl.get_wakeup_words(),
                "response_word": self._media_ctrl.get_response_words(),
                "sleep_response_word": self._media_ctrl.get_sleep_response_words(),
            }
        if action == "set_wakeup_word":
            word = str(args.get("word", ""))
            self._media_ctrl.set_wakeup_words(word)
            return {"wakeup_word": word, "state": "set"}
        if action == "set_response_word":
            word = str(args.get("word", ""))
            self._media_ctrl.set_response_words(word)
            return {"response_word": word, "state": "set"}
        if action == "set_sleep_response_word":
            word = str(args.get("word", ""))
            self._media_ctrl.set_sleep_response_words(word)
            return {"sleep_response_word": word, "state": "set"}
        return None


# ── AudioPlaybackPlugin (sensor) ─────────────────────────────────────────────

class AudioPlaybackPlugin:
    """Monitor agent→speaker playback stream from MediaController."""

    PREFIX = "audio_playback"

    def __init__(self, plugin_config: dict, namespace: str, executor, media_ctrl):
        self._media_ctrl = media_ctrl
        self._namespace = namespace
        self._topic = f"/{namespace}/audio/playback"
        self._node = Node("bumi_audio_playback")
        executor.add_node(self._node)
        self._pub = None
        self._running = False
        self._thread: threading.Thread | None = None

    def get_tool(self) -> dict:
        return {
            "name": "audio_playback",
            "type": "sensor",
            "multiInstance": False,
            "description": f"Bumi audio playback monitor — agent→speaker PCM stream. Publishes to {self._topic}. Silent when no audio playing.",
            "inputSchema": {"type": "object", "properties": {}},
            "topic_out": [{"topic": self._topic, "format": "audio/pcm-16k"}],
        }

    def start(self) -> None:
        self._pub = self._node.create_publisher(String, self._topic, _LOW_LAT_QOS)
        self._running = True
        self._thread = threading.Thread(target=self._poll_loop, daemon=True, name="bumi_audio_playback")
        self._thread.start()

    def stop(self) -> None:
        self._running = False

    def _poll_loop(self):
        while self._running:
            try:
                audio = self._media_ctrl.get_agent_audio_playback_data()
                if audio.channels == 0 or len(audio.audio_data) == 0:
                    time.sleep(0.01)
                    continue
                samples = list(audio.audio_data)
                msg = String()
                msg.data = json.dumps({
                    "channels": audio.channels,
                    "sample_rate": audio.sample_rate,
                    "format": audio.format,
                    "data": __import__('base64').b64encode(
                        struct.pack(f'<{len(samples)}h', *samples)
                    ).decode(),
                })
                self._pub.publish(msg)
            except Exception:
                time.sleep(0.01)

    def dispatch(self, action: str, args: dict) -> dict | None:
        if action == "start":
            return {"state": "running", "topic_out": [{"topic": self._topic, "format": "audio/pcm-16k"}]}
        if action == "stop":
            return {"state": "idle"}
        if action == "info":
            return {"state": "running" if self._running else "idle"}
        return None


# ── VideoPlugin (sensor + actuator) ───────────────────────────────────────────

class VideoPlugin:
    """Internal camera capture (sensor) + external video push (actuator)."""

    PREFIX = "video"

    def __init__(self, plugin_config: dict, namespace: str, executor, media_ctrl):
        self._media_ctrl = media_ctrl
        self._namespace = namespace
        self._capture_topic = f"/{namespace}/video/capture"
        self._node = Node("bumi_video")
        executor.add_node(self._node)
        self._pub = None
        self._running = False
        self._thread: threading.Thread | None = None

    def get_tools(self) -> list:
        return [
            {
                "name": "video_capture",
                "type": "sensor",
                "multiInstance": False,
                "description": f"Bumi internal camera video capture. Publishes to {self._capture_topic}. May have no data if no internal camera.",
                "inputSchema": {"type": "object", "properties": {}},
                "topic_out": [{"topic": self._capture_topic, "format": "image/jpeg"}],
            },
            {
                "name": "video_external",
                "type": "actuator",
                "multiInstance": False,
                "description": "Bumi external video push — publish external video stream to robot AI agent.",
                "inputSchema": {
                    "type": "object",
                    "properties": {
                        "action": {
                            "type": "string",
                            "enum": ["publish", "stop"],
                        },
                        "input_topic": {
                            "type": "string",
                            "description": "ROS2 topic to subscribe for video frames",
                        },
                    },
                    "required": ["action"],
                    "x-action-params": {
                        "publish": {"params": ["input_topic"], "description": "Subscribe to video topic and push to robot agent"},
                        "stop": {"params": [], "description": "Stop external video push"},
                    },
                },
                "topic_in": [{"format": "image/jpeg"}],
                "topic_out": [],
            },
        ]

    def start(self) -> None:
        self._pub = self._node.create_publisher(String, self._capture_topic, _LOW_LAT_QOS)
        self._running = True
        self._thread = threading.Thread(target=self._poll_loop, daemon=True, name="bumi_video_capture")
        self._thread.start()

    def stop(self) -> None:
        self._running = False

    def _poll_loop(self):
        while self._running:
            try:
                video = self._media_ctrl.get_capture_video_data()
                if video.width == 0 or len(video.video_data) == 0:
                    time.sleep(0.05)
                    continue
                msg = String()
                msg.data = json.dumps({
                    "width": video.width,
                    "height": video.height,
                    "format": video.format,
                    "data": __import__('base64').b64encode(bytes(video.video_data)).decode(),
                })
                self._pub.publish(msg)
            except Exception:
                time.sleep(0.05)

    def dispatch(self, action: str, args: dict) -> dict | None:
        tool_name = args.pop('_tool_name', '')

        if action == "start":
            return {"state": "running"}
        if action == "stop":
            if tool_name == "video_external":
                return {"state": "idle"}
            return {"state": "idle"}
        if action == "info":
            return {"state": "running" if self._running else "idle"}

        if tool_name == "video_external":
            if action == "publish":
                input_topic = args.get("input_topic", "")
                if not input_topic:
                    return {"error": "input_topic is required"}
                self._media_ctrl.set_external_custom_video_data_to_agent_enable(True)

                def _on_video(msg):
                    try:
                        data = json.loads(msg.data)
                        from mediacontrol_py import VideoStream
                        stream = VideoStream()
                        stream.width = data.get("width", 640)
                        stream.height = data.get("height", 480)
                        stream.format = 1  # YUV422
                        import base64 as _b64
                        stream.video_data = list(_b64.b64decode(data["data"]))
                        self._media_ctrl.publish_external_video_stream(stream)
                    except Exception:
                        pass

                sub = self._node.create_subscription(String, input_topic, _on_video, _LOW_LAT_QOS)
                return {"state": "publishing", "input_topic": input_topic}
            if action == "stop":
                self._media_ctrl.set_external_custom_video_data_to_agent_enable(False)
                return {"state": "stopped"}
        return None


# ── LowCmdPlugin (actuator, subprocess) ──────────────────────────────────────

def _lowcmd_subprocess(namespace: str):
    """LowController subprocess — set_joint for 21 motors at 500Hz.

    Runs in a separate process because LowController and HighController
    cannot coexist in the same Python process (type registration conflict).
    """
    import os as _os
    _os.environ.setdefault('CYCLONEDDS_URI', 'file:///work/noetix_sdk_bumi/config/dds.xml')
    import sys as _sys
    _sys.path.insert(0, '/work/noetix_sdk_bumi/build')
    import time as _time

    from lowcontrol_py import LowController, MotorCmd

    ctrl = LowController.instance()
    ctrl.init()
    _time.sleep(2)

    import rclpy as _rclpy
    from rclpy.node import Node as _Node
    from std_msgs.msg import String as _String

    _rclpy.init()
    node = _Node("bumi_lowcmd_sub")

    # Joint name → hardware index mapping
    joint_names = [
        'leg_l1', 'leg_r1', 'waist_1', 'leg_l2', 'leg_r2',
        'arm_l1', 'arm_r1', 'leg_l3', 'leg_r3', 'arm_l2', 'arm_r2',
        'leg_l4', 'leg_r4', 'arm_l3', 'arm_r3', 'leg_l5', 'leg_r5',
        'arm_l4', 'arm_r4', 'leg_l6', 'leg_r6',
    ]

    cmd_topic = f"/{namespace}/low_cmd"
    last_cmd = None

    def _on_cmd(msg):
        nonlocal last_cmd
        try:
            data = json.loads(msg.data)
            cmds = []
            for i in range(21):
                mc = MotorCmd()
                mc.motor_id = i
                mc.kp = float(data.get("kp", 0))
                mc.kd = float(data.get("kd", 0))
                mc.q = float(data.get("q", 0))
                mc.dq = float(data.get("dq", 0))
                mc.tau = float(data.get("tau", 0))
                cmds.append(mc)
            last_cmd = cmds
        except Exception as e:
            print(f"[lowcmd_subprocess] error: {e}", flush=True)

    sub = node.create_subscription(_String, cmd_topic, _on_cmd, _LOW_LAT_QOS)
    print(f"[lowcmd_subprocess] listening on {cmd_topic}", flush=True)

    rate = _rclpy.rate.Rate(500)  # 500 Hz
    while _rclpy.ok():
        if last_cmd is not None:
            ctrl.set_joint(last_cmd)
        rate.spin_once()

    ctrl = None


class LowCmdPlugin:
    """Low-level motor control via LowController (subprocess)."""

    PREFIX = "low_cmd"

    def __init__(self, plugin_config: dict, namespace: str, executor):
        self._namespace = namespace
        self._cmd_topic = f"/{namespace}/low_cmd"
        self._proc: subprocess.Popen | None = None
        self._node = Node("bumi_lowcmd")
        executor.add_node(self._node)
        self._pub = self._node.create_publisher(String, self._cmd_topic, _LOW_LAT_QOS)

    def get_tool(self) -> dict:
        return {
            "name": "low_cmd",
            "type": "actuator",
            "multiInstance": False,
            "description": "Bumi low-level motor control — set 21 joint PD targets (kp,kd,q,dq,tau) at 500Hz via LowController subprocess.",
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {
                        "type": "string",
                        "enum": ["send", "stop"],
                    },
                    "kp": {"type": "number", "description": "Position gain (default 0)"},
                    "kd": {"type": "number", "description": "Velocity gain (default 0)"},
                    "q": {"type": "number", "description": "Target position in rad (default 0)"},
                    "dq": {"type": "number", "description": "Target velocity in rad/s (default 0)"},
                    "tau": {"type": "number", "description": "Feedforward torque in Nm (default 0)"},
                },
                "required": ["action"],
                "x-action-params": {
                    "send": {"params": ["kp", "kd", "q", "dq", "tau"], "description": "Send PD command to all 21 joints"},
                    "stop": {"params": [], "description": "Stop sending commands (zero all targets)"},
                },
            },
            "topic_out": [{"topic": self._cmd_topic, "format": "data/json"}],
        }

    def start(self) -> None:
        import sys
        self._proc = subprocess.Popen(
            [sys.executable, "-c",
             f"import sys; sys.path.insert(0, '/work'); from device import _lowcmd_subprocess; _lowcmd_subprocess({self._namespace!r})"],
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
        args.pop('_tool_name', None)

        if action == "start":
            return {"state": "ready"}
        if action == "stop":
            msg = String()
            msg.data = json.dumps({"kp": 0, "kd": 0, "q": 0, "dq": 0, "tau": 0})
            self._pub.publish(msg)
            return {"state": "stopped"}
        if action == "send":
            cmd = {
                "kp": float(args.get("kp", 0)),
                "kd": float(args.get("kd", 0)),
                "q": float(args.get("q", 0)),
                "dq": float(args.get("dq", 0)),
                "tau": float(args.get("tau", 0)),
            }
            msg = String()
            msg.data = json.dumps(cmd)
            self._pub.publish(msg)
            return {"state": "sent", "cmd": cmd}
        return None


# ── RlPolicyPlugin (actuator, subprocess) ────────────────────────────────────

def _rlpolicy_subprocess(namespace: str):
    """RL policy walking subprocess — ONNX model inference + state machine.

    Runs in a separate process because LowController and HighController
    cannot coexist in the same Python process.
    """
    import os as _os
    _os.environ.setdefault('CYCLONEDDS_URI', 'file:///work/noetix_sdk_bumi/config/dds.xml')
    import sys as _sys
    _sys.path.insert(0, '/work/noetix_sdk_bumi/build')
    import time as _time
    import numpy as _np

    from lowcontrol_py import LowController, MotorCmd

    ctrl = LowController.instance()
    ctrl.init()
    _time.sleep(2)

    import rclpy as _rclpy
    from rclpy.node import Node as _Node
    from std_msgs.msg import String as _String

    _rclpy.init()
    node = _Node("bumi_rl_sub")

    # Joint names (SDK order)
    joint_names = [
        'leg_l1', 'leg_r1', 'waist_1', 'leg_l2', 'leg_r2',
        'arm_l1', 'arm_r1', 'leg_l3', 'leg_r3', 'arm_l2', 'arm_r2',
        'leg_l4', 'leg_r4', 'arm_l3', 'arm_r3', 'leg_l5', 'leg_r5',
        'arm_l4', 'arm_r4', 'leg_l6', 'leg_r6',
    ]

    # Map joint name → hardware index
    joint_indices = {}
    for name in joint_names:
        joint_indices[name] = ctrl.getJointsIndex(name)

    # Load ONNX model
    model_path = "/work/noetix_sdk_bumi/models/policy.onnx"
    try:
        import onnxruntime as ort
        session = ort.InferenceSession(model_path)
        print(f"[rl_subprocess] ONNX model loaded: {model_path}", flush=True)
    except Exception as e:
        print(f"[rl_subprocess] ONNX load failed: {e}", flush=True)
        return

    # State machine
    mode_ = "DEFAULT"
    cmd_topic = f"/{namespace}/rl_cmd"
    target_vel = {"vx": 0.0, "vy": 0.0, "vyaw": 0.0}
    running = True

    def _on_cmd(msg):
        nonlocal mode_, target_vel, running
        try:
            data = json.loads(msg.data)
            action = data.get("action", "")
            if action == "start":
                mode_ = "LIE"
            elif action == "stop":
                mode_ = "DEFAULT"
                target_vel = {"vx": 0.0, "vy": 0.0, "vyaw": 0.0}
            elif action == "stand":
                mode_ = "STAND"
            elif action == "lie":
                mode_ = "LIE"
            elif action == "walk":
                mode_ = "USERMODE"
                target_vel = {
                    "vx": float(data.get("vx", 0)),
                    "vy": float(data.get("vy", 0)),
                    "vyaw": float(data.get("vyaw", 0)),
                }
        except Exception as e:
            print(f"[rl_subprocess] cmd error: {e}", flush=True)

    sub = node.create_subscription(_String, cmd_topic, _on_cmd, _LOW_LAT_QOS)
    print(f"[rl_subprocess] listening on {cmd_topic}", flush=True)

    # Default PD gains
    kp_stiff = [100.0] * 21
    kd_stiff = [3.0] * 21
    kp_soft = [20.0] * 21
    kd_soft = [1.0] * 21

    # Default standing pose
    default_pos = [0.0] * 21

    rate = _rclpy.rate.Rate(100)  # 100 Hz inference
    while _rclpy.ok():
        try:
            if mode_ == "DEFAULT":
                # Soft PD to default pose
                cmds = []
                for i in range(21):
                    mc = MotorCmd()
                    mc.motor_id = i
                    mc.kp = kp_soft[i]
                    mc.kd = kd_soft[i]
                    mc.q = default_pos[i]
                    mc.dq = 0.0
                    mc.tau = 0.0
                    cmds.append(mc)
                ctrl.set_joint(cmds)

            elif mode_ in ("LIE", "STAND", "USERMODE"):
                # Get joint state
                joint_state = ctrl.get_joint_state()
                imu = ctrl.get_imu_data()

                # Build observation vector
                joint_pos = []
                joint_vel = []
                for i in range(21):
                    joint_pos.append(float(joint_state[i].pos))
                    joint_vel.append(float(joint_state[i].vel))

                # Projected gravity from IMU
                quat = [float(imu.ori[i]) for i in range(4)]
                # Simple projected gravity calculation
                projected_gravity = [
                    2 * (quat[0] * quat[2] + quat[1] * quat[3]),
                    2 * (quat[1] * quat[2] - quat[0] * quat[3]),
                    1 - 2 * (quat[0] * quat[0] + quat[1] * quat[1]),
                ]

                # Fall protection
                if projected_gravity[2] >= -0.3:
                    mode_ = "DEFAULT"
                    continue

                # Build obs: projected_gravity(3) + joint_pos(21) + joint_vel(21) + velocity(3)
                vx = max(-1.5, min(1.5, target_vel["vx"]))
                vy = max(-1.0, min(1.0, target_vel["vy"]))
                vyaw = max(-1.0, min(1.0, target_vel["vyaw"]))

                obs = _np.array(
                    projected_gravity + joint_pos + joint_vel + [vx, vy, vyaw],
                    dtype=_np.float32,
                ).reshape(1, -1)

                # ONNX inference
                outputs = session.run(None, {"obs": obs})
                action = outputs[0][0]

                # Apply action
                cmds = []
                for i in range(21):
                    mc = MotorCmd()
                    mc.motor_id = i
                    mc.kp = kp_stiff[i]
                    mc.kd = kd_stiff[i]
                    mc.q = float(action[i])
                    mc.dq = 0.0
                    mc.tau = 0.0
                    cmds.append(mc)
                ctrl.set_joint(cmds)

        except Exception as e:
            print(f"[rl_subprocess] loop error: {e}", flush=True)

        rate.spin_once()

    ctrl = None


class RlPolicyPlugin:
    """RL policy walking control via ONNX model (subprocess)."""

    PREFIX = "rl_policy"

    def __init__(self, plugin_config: dict, namespace: str, executor):
        self._namespace = namespace
        self._cmd_topic = f"/{namespace}/rl_cmd"
        self._proc: subprocess.Popen | None = None
        self._node = Node("bumi_rl_policy")
        executor.add_node(self._node)
        self._pub = self._node.create_publisher(String, self._cmd_topic, _LOW_LAT_QOS)

    def get_tool(self) -> dict:
        return {
            "name": "rl_policy",
            "type": "actuator",
            "multiInstance": False,
            "description": "Bumi RL policy walking — ONNX model-based reinforcement learning locomotion. State machine: DEFAULT→LIE→STAND→USERMODE(walk). Fall protection auto-recovers.",
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {
                        "type": "string",
                        "enum": ["read", "start", "stop", "stand", "lie", "walk"],
                    },
                    "vx": {"type": "number", "description": "Forward velocity [-1.5, 1.5]", "minimum": -1.5, "maximum": 1.5},
                    "vy": {"type": "number", "description": "Lateral velocity [-1.0, 1.0]", "minimum": -1, "maximum": 1},
                    "vyaw": {"type": "number", "description": "Turning velocity [-1.0, 1.0]", "minimum": -1, "maximum": 1},
                },
                "required": ["action"],
                "x-action-params": {
                    "read": {"params": [], "description": "Check if RL subprocess is running"},
                    "start": {"params": [], "description": "Start RL policy (enter LIE state)"},
                    "stop": {"params": [], "description": "Stop RL policy (return to DEFAULT)"},
                    "stand": {"params": [], "description": "Stand up from lying"},
                    "lie": {"params": [], "description": "Lie down from standing"},
                    "walk": {"params": ["vx", "vy", "vyaw"], "description": "Walk with velocity commands"},
                },
            },
            "topic_out": [{"topic": self._cmd_topic, "format": "data/json"}],
        }

    def start(self) -> None:
        import sys
        self._proc = subprocess.Popen(
            [sys.executable, "-c",
             f"import sys; sys.path.insert(0, '/work'); from device import _rlpolicy_subprocess; _rlpolicy_subprocess({self._namespace!r})"],
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
        )
        def _fwd():
            for line in self._proc.stdout:
                print(line.decode(errors='replace').rstrip(), flush=True)
        threading.Thread(target=_fwd, daemon=True).start()

    def stop(self) -> None:
        if self._proc:
            # Send stop command first
            msg = String()
            msg.data = json.dumps({"action": "stop"})
            self._pub.publish(msg)
            time.sleep(0.1)
            self._proc.terminate()
            self._proc = None

    def dispatch(self, action: str, args: dict) -> dict | None:
        args.pop('_tool_name', None)

        if action == "start":
            return {"state": "ready"}
        if action == "stop":
            return {"state": "idle"}
        if action == "read":
            return {"state": "running" if self._proc and self._proc.poll() is None else "idle"}
        if action == "start":
            msg = String()
            msg.data = json.dumps({"action": "start"})
            self._pub.publish(msg)
            return {"state": "starting"}
        if action == "stop":
            msg = String()
            msg.data = json.dumps({"action": "stop"})
            self._pub.publish(msg)
            return {"state": "stopped"}
        if action == "stand":
            msg = String()
            msg.data = json.dumps({"action": "stand"})
            self._pub.publish(msg)
            return {"state": "standing"}
        if action == "lie":
            msg = String()
            msg.data = json.dumps({"action": "lie"})
            self._pub.publish(msg)
            return {"state": "lying"}
        if action == "walk":
            cmd = {
                "action": "walk",
                "vx": float(args.get("vx", 0)),
                "vy": float(args.get("vy", 0)),
                "vyaw": float(args.get("vyaw", 0)),
            }
            msg = String()
            msg.data = json.dumps(cmd)
            self._pub.publish(msg)
            return {"state": "walking", "cmd": cmd}
        return None
