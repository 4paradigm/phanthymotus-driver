#!/usr/bin/env python3
"""
drivers/noetix/bumi/device.py — Noetix Bumi-EDU 设备插件实现。

插件列表：
  - StatePlugin: joints (21-DOF skeleton), motor_health, imu, battery, model (URDF resource)
  - LocoPlugin: loco (move/stop), switch_mode (mode transitions)
  - MicPlugin: 8ch mic capture → mono PCM 16kHz
  - SpeakerPlugin: audio playback via MediaController
  - CameraPlugin: Realsense D435i color + depth
  - VideoPlugin: external video push + desensed video read (auto-start desensed)
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

# Motor-health uses the names defined by the Bumi SDK documentation. Keep this
# mapping separate from the skeleton names above: the latter must match the
# bundled URDF, while the health card is an API-facing diagnostic contract.
_MOTOR_HEALTH_JOINT_NAMES = {
    0: "arm_l1_joint",
    1: "arm_l2_joint",
    2: "arm_l3_joint",
    3: "arm_l4_joint",
    4: "leg_l1_joint",
    5: "leg_l2_joint",
    6: "leg_l3_joint",
    7: "leg_l4_joint",
    8: "leg_l5_joint",
    9: "leg_l6_joint",
    10: "arm_r1_joint",
    11: "arm_r2_joint",
    12: "arm_r3_joint",
    13: "arm_r4_joint",
    14: "leg_r1_joint",
    15: "leg_r2_joint",
    16: "leg_r3_joint",
    17: "leg_r4_joint",
    18: "leg_r5_joint",
    19: "leg_r6_joint",
    20: "waist_1_joint",
}

_MOTOR_ERROR_MESSAGES = {
    0x02: "Motor over current",
    0x03: "Motor under voltage",
    0x04: "Encoder error",
    0x06: "Brake voltage over",
    0x07: "DRV driver error",
    0x08: "Over voltage",
    0x09: "Under voltage",
    0x0A: "Over current",
    0x0B: "MOS over temperature",
    0x0C: "Coil over temperature",
    0x0D: "Communication lost",
    0x0E: "Overload",
}

# The Bumi guide lists 02H, 03H, 04H and 06H-0EH as motor error codes.  The
# robot reports 01H for an otherwise healthy motor, so it must be treated like
# the SDK's zero-value/no-error state rather than surfaced as an unknown fault.
_MOTOR_HEALTH_NORMAL_CODES = {0x00, 0x01}


def _build_motor_health(joint_state) -> dict:
    """Convert the SDK's 21 MotorState values to the motor_health schema."""
    faults = []
    for index, motor in enumerate(joint_state):
        motor_id = int(getattr(motor, "motor_id", index))
        error = int(getattr(motor, "error", 0))
        if error in _MOTOR_HEALTH_NORMAL_CODES:
            continue

        faults.append({
            "joint": _MOTOR_HEALTH_JOINT_NAMES.get(motor_id, f"motor_{motor_id}"),
            "motor_id": motor_id,
            "code": f"0x{error:02X}",
            "message": _MOTOR_ERROR_MESSAGES.get(error, "Unknown motor error"),
            "temperature": int(getattr(motor, "temperature", 0)),
        })

    return {
        "healthy": not faults,
        "fault_count": len(faults),
        "faults": faults,
    }

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
        self._motor_health_topic = f"/{namespace}/state/motor_health"

        self._imu_pub     = self.create_publisher(String, self._imu_topic,     _LOW_LAT_QOS)
        self._battery_pub = self.create_publisher(String, self._battery_topic, _LOW_LAT_QOS)
        self._joints_pub  = self.create_publisher(String, self._joints_topic,  _LOW_LAT_QOS)
        self._motor_health_pub = self.create_publisher(String, self._motor_health_topic, _LOW_LAT_QOS)

        self._last_imu: dict = {}
        self._last_battery: dict = {}
        self._last_motor_health: dict = {
            "healthy": True,
            "fault_count": 0,
            "faults": [],
        }
        self._lock = threading.Lock()
        self._running = False
        self._thread: threading.Thread | None = None

    def start_polling(self):
        self._running = True
        self._thread = threading.Thread(target=self._poll_loop, daemon=True, name="bumi_state_poll")
        self._thread.start()

    def stop_polling(self):
        self._running = False

    def get_motor_health(self) -> dict:
        with self._lock:
            return {
                "healthy": self._last_motor_health["healthy"],
                "fault_count": self._last_motor_health["fault_count"],
                "faults": [dict(fault) for fault in self._last_motor_health["faults"]],
            }

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

                    motor_health = _build_motor_health(joint_state)
                    with self._lock:
                        self._last_motor_health = motor_health
                    motor_health_out = String()
                    motor_health_out.data = json.dumps(motor_health)
                    self._motor_health_pub.publish(motor_health_out)

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
                "name": "motor_health",
                "type": "sensor",
                "multiInstance": False,
                "description": f"Bumi motor health — active motor faults with joint, error code, message, and temperature. Publishes at 10Hz to /{ns}/state/motor_health",
                "inputSchema": {"type": "object", "properties": {}},
                "topic_out": [{"topic": f"/{ns}/state/motor_health", "format": "data/json"}],
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
        if action == "motor_health":
            return self._node.get_motor_health()
        if action == "info":
            if args.get("_tool_name") == "motor_health":
                return {
                    "state": "running",
                    "topic_out": [{
                        "topic": f"/{self._namespace}/state/motor_health",
                        "format": "data/json",
                    }],
                }
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

# ── VideoPlugin (actuator ×1 + sensor ×1 + actuator ×1) ──────────────────────
#
# 算力板 Bumi 的视频数据流（参考 SDK demo test_media.py / example_media.cpp）：
#
#   external-video: 摄像头/USB → YUYV帧 → publish_external_video_stream() → 运控板 Agent
#                              │
#   desensed-video: 音视频交互系统加工脱敏 → get_video_capture_desensed_data()
#                   （官方：带算力板需持续推外部视频流才有脱敏流；实测不推也可能出运控板相机画面）
#
#   get_video_capture_data() / pause_video_capture() / resume_video_capture()
#   仅不带算力板的 Bumi 可用，本插件不封装。

def _yuv422_to_rgb(yuv_data: bytes, width: int, height: int):
    """Convert YUYV/YUV422 raw bytes to RGB numpy array."""
    import numpy as np
    yuv = np.frombuffer(yuv_data, dtype=np.uint8).reshape((height, width, 2))
    y = yuv[:, :, 0].astype(np.float32)
    u = yuv[:, ::2, 1].astype(np.float32)
    v = yuv[:, 1::2, 1].astype(np.float32)
    u = np.repeat(np.repeat(u, 2, axis=1), 1, axis=0)[:height, :width]
    v = np.repeat(np.repeat(v, 2, axis=1), 1, axis=0)[:height, :width]
    y = y - 16
    u = u - 128
    v = v - 128
    r = (y + 1.402 * v).clip(0, 255).astype(np.uint8)
    g = (y - 0.344136 * u - 0.714136 * v).clip(0, 255).astype(np.uint8)
    b = (y + 1.772 * u).clip(0, 255).astype(np.uint8)
    return np.stack([r, g, b], axis=2)


def _encode_jpeg(bgr_image) -> bytes:
    """Encode BGR numpy array to JPEG bytes. TurboJPEG preferred, cv2 fallback."""
    try:
        from turbojpeg import TurboJPEG, TJPF_BGR
        _tj = TurboJPEG()
        return _tj.encode(bgr_image, pixel_format=TJPF_BGR, quality=80)
    except Exception:
        import cv2
        _, buf = cv2.imencode('.jpg', bgr_image, [cv2.IMWRITE_JPEG_QUALITY, 80])
        return buf.tobytes()


def _frame_to_yuyv(bgr_frame) -> bytes:
    """Convert OpenCV BGR frame to YUYV bytes (test_media.py publish_video pattern)."""
    import numpy as np
    import cv2
    h, w = bgr_frame.shape[:2]
    yuv = cv2.cvtColor(bgr_frame, cv2.COLOR_BGR2YUV)
    out = np.zeros((h, w, 2), dtype=np.uint8)
    out[:, 0::2, 0] = yuv[:, 0::2, 0]   # Y even
    out[:, 0::2, 1] = yuv[:, 0::2, 1]   # U
    out[:, 1::2, 0] = yuv[:, 1::2, 0]   # Y odd
    out[:, 1::2, 1] = yuv[:, 1::2, 2]   # V
    return out.tobytes()


def _decode_video_frame(data: bytes, width: int, height: int):
    """Decode raw video frame data (YUV422 or RGB) to BGR for JPEG encoding."""
    import numpy as np
    import cv2

    if len(data) < 100:
        return None
    try:
        from io import BytesIO
        from PIL import Image
        img = Image.open(BytesIO(data))
        rgb = np.array(img.convert("RGB"))
        return cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
    except Exception:
        pass

    if len(data) == width * height * 2:
        try:
            rgb = _yuv422_to_rgb(data, width, height)
            return cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        except Exception:
            pass

    if len(data) == width * height * 3:
        try:
            rgb = np.frombuffer(data, dtype=np.uint8).reshape((height, width, 3))
            return cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        except Exception:
            pass

    return None


# ═══════════════════════════════════════════════════════════════════════════════
# Subprocess: USB camera → YUYV → publish_external_video_stream() + ROS2 JPEG
# ═══════════════════════════════════════════════════════════════════════════════

def _external_video_subprocess(namespace: str, camera_id: int):
    """Open USB camera, push YUYV frames to robot AI agent, also publish JPEG to ROS2.

    This is the PRIMARY video path for 算力板 Bumi.
    Follows test_media.py publish_video() pattern exactly.
    """
    import os as _os
    _os.environ.setdefault('CYCLONEDDS_URI', 'file:///work/noetix_sdk_bumi/config/dds.xml')
    import sys as _sys
    _sys.path.insert(0, '/work/noetix_sdk_bumi/build')
    import time as _time
    import cv2
    import numpy as _np

    import rclpy as _rclpy
    from rclpy.node import Node as _Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
    from sensor_msgs.msg import CompressedImage as _CompressedImage

    _QOS = QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        durability=DurabilityPolicy.VOLATILE,
    )

    from mediacontrol_py import MediaController, VideoStream

    # ── Init MediaController ──
    media = MediaController.instance()
    if not media.init():
        print("[external_video] MediaController.init() FAILED", flush=True)
        return
    _time.sleep(3)

    # ── Enable external video routing to agent ──
    media.set_external_custom_video_data_to_agent_enable(True)
    _time.sleep(0.2)

    # ── Open USB camera (auto-detect: explicit id → video4/2/0 → any /dev/videoN) ──
    if camera_id >= 0:
        candidate_ids = [camera_id] + [i for i in (4, 2, 0) if i != camera_id]
    else:
        candidate_ids = [4, 2, 0]
    import os as __os
    for d in range(10):
        if __os.path.exists(f"/dev/video{d}") and d not in candidate_ids:
            candidate_ids.append(d)

    cap = None
    used_id = -1
    for cid in candidate_ids:
        c = cv2.VideoCapture(cid)
        if c.isOpened():
            cap = c
            used_id = cid
            break
        c.release()

    if cap is None:
        print(f"[external_video] Cannot open camera (tried: {candidate_ids})", flush=True)
        for d in range(10):
            p = f"/dev/video{d}"
            if __os.path.exists(p):
                print(f"    available: {p}", flush=True)
        return

    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"[external_video] camera /dev/video{used_id}: {w}x{h}", flush=True)

    # ── ROS2 publisher for external consumers ──
    _rclpy.init()
    node = _Node("bumi_external_video")
    topic = f"/{namespace}/video/external"
    pub = node.create_publisher(_CompressedImage, topic, _QOS)

    print(f"[external_video] publishing YUYV→agent + JPEG→{topic}", flush=True)

    fc = 0
    t_start = _time.monotonic()
    try:
        while cap.isOpened():
            ret, frame = cap.read()
            if not ret:
                _time.sleep(0.01)
                continue

            # ── Push YUYV to agent ──
            yuyv = _frame_to_yuyv(frame)
            vs = VideoStream()
            vs.width = w
            vs.height = h
            vs.format = 0   # 0 = YUYV (SDK demo convention)
            vs.fps = 30
            vs.timestamp_us = int(_time.time() * 1e6)
            vs.video_data = list(yuyv)
            media.publish_external_video_stream(vs)

            # ── Also publish JPEG to ROS2 ──
            jpeg = _encode_jpeg(frame)
            msg = _CompressedImage()
            msg.header.stamp = node.get_clock().now().to_msg()
            msg.format = "jpeg"
            msg.data = jpeg
            pub.publish(msg)

            fc += 1
            if fc % 300 == 0:
                elapsed = _time.monotonic() - t_start
                print(f"[external_video] {fc} frames, {fc / elapsed:.1f} fps", flush=True)

            _time.sleep(0.001)  # yield
    except Exception as e:
        print(f"[external_video] error: {e}", flush=True)
    finally:
        cap.release()


# ═══════════════════════════════════════════════════════════════════════════════
# Subprocess: poll get_video_capture_desensed_data() → ROS2 JPEG
# ═══════════════════════════════════════════════════════════════════════════════

def _desensed_video_subprocess(namespace: str):
    """Poll the desensed (privacy-masked) video stream, publish as JPEG to ROS2.

    Reads get_video_capture_desensed_data() — the stream the robot's audio/video
    interaction system produces by desensitizing its current video input. Per the
    SDK docs, on a 算力板 Bumi this needs an active external video push
    (external-video). In practice a frame may also appear without a push, when
    set_internal_capture_video_data_to_agent_enable(True) routes the control-board
    camera through. Follows test_media.py desensed_video() pattern.
    """
    import os as _os
    _os.environ.setdefault('CYCLONEDDS_URI', 'file:///work/noetix_sdk_bumi/config/dds.xml')
    import sys as _sys
    _sys.path.insert(0, '/work/noetix_sdk_bumi/build')
    import time as _time

    import rclpy as _rclpy
    from rclpy.node import Node as _Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
    from sensor_msgs.msg import CompressedImage as _CompressedImage

    _QOS = QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        durability=DurabilityPolicy.VOLATILE,
    )

    from mediacontrol_py import MediaController
    media = MediaController.instance()
    if not media.init():
        print("[desensed_video] MediaController.init() FAILED", flush=True)
        return
    _time.sleep(3)

    _rclpy.init()
    node = _Node("bumi_desensed_video")
    topic = f"/{namespace}/video/desensed"
    pub = node.create_publisher(_CompressedImage, topic, _QOS)

    # Enable routing so desensed frames are produced
    media.set_internal_capture_video_data_to_agent_enable(True)

    print(f"[desensed_video] waiting for desensed frames → {topic}", flush=True)
    print(f"[desensed_video] (desensed stream; per SDK docs needs external video push)", flush=True)

    frame_count = 0
    t_start = _time.monotonic()
    no_data_count = 0

    while True:
        try:
            vs = media.get_video_capture_desensed_data()
            if vs.width == 0 or len(vs.video_data) == 0:
                no_data_count += 1
                if no_data_count == 1:
                    print("[desensed_video] no desensed data yet (is external video pushing?)", flush=True)
                _time.sleep(0.05)
                continue
            no_data_count = 0

            raw = bytes(vs.video_data)
            bgr = _decode_video_frame(raw, vs.width, vs.height)
            if bgr is None:
                _time.sleep(0.02)
                continue

            jpeg_bytes = _encode_jpeg(bgr)
            msg = _CompressedImage()
            msg.header.stamp = node.get_clock().now().to_msg()
            msg.format = "jpeg"
            msg.data = jpeg_bytes
            pub.publish(msg)

            frame_count += 1
            if frame_count % 300 == 0:
                elapsed = _time.monotonic() - t_start
                print(f"[desensed_video] {frame_count} frames, {frame_count / elapsed:.1f} fps", flush=True)

            _time.sleep(0.005)
        except Exception as e:
            print(f"[desensed_video] error: {e}", flush=True)
            _time.sleep(0.5)


# ═══════════════════════════════════════════════════════════════════════════════
# VideoPlugin class
# ═══════════════════════════════════════════════════════════════════════════════

class VideoPlugin:
    """Video for 算力板 Bumi via MediaController.

    Tools:
      external-video (actuator) — push video + routing config
      desensed-video (sensor)   — read the system's desensed (privacy-masked) video stream
    """

    PREFIX = "video"

    def __init__(self, plugin_config: dict, namespace: str, executor, media_ctrl):
        self._namespace = namespace
        self._media_ctrl = media_ctrl
        self._node = Node("bumi_video")
        executor.add_node(self._node)

        self._external_topic = f"/{namespace}/video/external"
        self._desensed_topic = f"/{namespace}/video/desensed"
        self._camera_color_topic = f"/{namespace}/camera/color"

        # external-video subprocess state
        self._proc_external: subprocess.Popen | None = None
        # desensed-video subprocess state
        self._proc_desensed: subprocess.Popen | None = None
        # push_from_topic state (in-process)
        self._pushing = False
        self._sub = None

    # ── Tool definitions ──────────────────────────────────────────────────

    def get_tools(self) -> list:
        return [
            self._external_video_tool(),
            {
                "name": "desensed-video",
                "type": "sensor",
                "multiInstance": False,
                "description": (
                    f"Bumi desensed (privacy-masked) video stream from the robot's audio/video system. "
                    f"Per SDK docs, needs an active external video push. Publishes JPEG to {self._desensed_topic}"
                ),
                "inputSchema": {"type": "object", "properties": {}},
                "topic_out": [{"topic": self._desensed_topic, "format": "image/jpeg"}],
            },
        ]

    def _external_video_tool(self) -> dict:
        return {
            "name": "external-video",
            "type": "actuator",
            "multiInstance": False,
            "description": (
                "Push external video to Bumi AI agent for recognition. "
                "Subscribes to CameraPlugin color topic, converts to YUYV, "
                f"calls publish_external_video_stream(). Also publishes JPEG to {self._external_topic}"
            ),
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {
                        "type": "string",
                        "enum": ["push_from_topic", "push_from_camera", "stop_push",
                                    "agent_enable", "agent_disable",
                                    "agent_external_enable", "agent_external_disable",
                                    "get_status"],
                    },
                    "camera_id": {
                        "type": "integer",
                        "description": "USB camera device ID (omit or -1 = auto-detect video4→2→0→any)",
                    },
                },
                "required": ["action"],
                "x-action-params": {
                    "push_from_camera": {
                        "params": ["camera_id"],
                        "description": "Open specified USB camera, convert frames to YUYV, push to agent.",
                    },
                    "push_from_topic": {
                        "params": [],
                        "description": f"Subscribe to {self._camera_color_topic} from CameraPlugin, convert to YUYV, push to agent.",
                    },
                    "stop_push": {
                        "params": [],
                        "description": "Stop all external video push.",
                    },
                    "agent_enable": {
                        "params": [],
                        "description": "Enable internal video → robot AI agent.",
                    },
                    "agent_disable": {
                        "params": [],
                        "description": "Disable internal video → robot AI agent.",
                    },
                    "agent_external_enable": {
                        "params": [],
                        "description": "Enable external video → robot AI agent.",
                    },
                    "agent_external_disable": {
                        "params": [],
                        "description": "Disable external video → robot AI agent.",
                    },
                    "get_status": {
                        "params": [],
                        "description": "Get current video routing configuration status.",
                    },
                },
            },
            "topic_in": [{"format": "image/jpeg"}],
            "topic_out": [],
        }

    # ── Lifecycle ─────────────────────────────────────────────────────────

    def start(self) -> None:
        # Auto-start desensed-video subprocess (reads the desensed stream)
        self._start_desensed()
        # Enable internal capture → agent (also drives the desensed stream)
        self._media_ctrl.set_internal_capture_video_data_to_agent_enable(True)

    def stop(self) -> None:
        self._stop_external()
        self._stop_desensed()
        self._pushing = False

    # ── Dispatch ──────────────────────────────────────────────────────────

    def dispatch(self, action: str, args: dict) -> dict | None:
        tool_name = args.pop('_tool_name', '')

        # ── external-video actions ──
        if tool_name == "external-video":
            if action == "push_from_camera":
                return self._do_push_from_camera(args)
            if action == "push_from_topic":
                return self._do_push_from_topic(args)
            if action == "stop_push":
                return self._do_stop_external_push()
            if action == "agent_enable":
                self._media_ctrl.set_internal_capture_video_data_to_agent_enable(True)
                return {"state": "enabled", "internal_video_to_agent": True}
            if action == "agent_disable":
                self._media_ctrl.set_internal_capture_video_data_to_agent_enable(False)
                return {"state": "disabled", "internal_video_to_agent": False}
            if action == "agent_external_enable":
                self._media_ctrl.set_external_custom_video_data_to_agent_enable(True)
                return {"state": "enabled", "external_video_to_agent": True}
            if action == "agent_external_disable":
                self._media_ctrl.set_external_custom_video_data_to_agent_enable(False)
                return {"state": "disabled", "external_video_to_agent": False}
            if action == "get_status":
                return {
                    "internal_video_to_agent": self._media_ctrl.get_internal_capture_video_data_to_agent_enable(),
                    "external_video_to_agent": self._media_ctrl.get_external_custom_video_data_to_agent_enable(),
                }
            if action == "start":
                return {"state": "ready"}
            if action == "stop":
                self._stop_external()
                return {"state": "idle"}
            if action == "info":
                running = self._proc_external is not None and self._proc_external.poll() is None
                return {
                    "state": "running" if running else "idle",
                    "topic_out": [{"topic": self._external_topic, "format": "image/jpeg"}] if running else [],
                }

        # ── desensed-video actions ──
        if tool_name == "desensed-video":
            if action == "start":
                return self._start_desensed()
            if action == "stop":
                return self._stop_desensed()
            if action == "info":
                running = self._proc_desensed is not None and self._proc_desensed.poll() is None
                return {
                    "state": "running" if running else "idle",
                    "topic_out": [{"topic": self._desensed_topic, "format": "image/jpeg"}] if running else [],
                }

        return None

    # ── external-video: push_from_camera (subprocess) ─────────────────────

    def _do_push_from_camera(self, args: dict) -> dict:
        camera_id = int(args.get("camera_id", -1))  # -1 = auto-detect (video4→2→0→any /dev/videoN)

        # Stop any existing push
        self._do_stop_external_push()

        import sys
        self._proc_external = subprocess.Popen(
            [sys.executable, "-c",
             f"import sys; sys.path.insert(0, '/work'); from device import _external_video_subprocess; _external_video_subprocess({self._namespace!r}, {camera_id})"],
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
        )
        def _fwd():
            for line in self._proc_external.stdout:
                print(line.decode(errors='replace').rstrip(), flush=True)
        threading.Thread(target=_fwd, daemon=True).start()

        return {
            "state": "pushing",
            "camera_id": camera_id,
            "topic_out": [{"topic": self._external_topic, "format": "image/jpeg"}],
        }

    def _stop_external(self) -> None:
        if self._proc_external:
            self._proc_external.terminate()
            self._proc_external = None

    # ── external-video: push_from_topic (in-process, like SpeakerPlugin) ──

    def _do_push_from_topic(self, args: dict) -> dict:
        import cv2
        import numpy as np
        import time as _t

        # Default to CameraPlugin color topic
        input_topic = args.get("input_topic", self._camera_color_topic)

        # Stop camera subprocess if running (only one push active at a time)
        self._stop_external()

        self._media_ctrl.set_external_custom_video_data_to_agent_enable(True)
        self._pushing = True

        from mediacontrol_py import VideoStream

        def _on_frame(msg):
            if not self._pushing:
                return
            try:
                raw = np.frombuffer(msg.data, dtype=np.uint8)
                bgr = cv2.imdecode(raw, cv2.IMREAD_COLOR)
                if bgr is None:
                    return
                h, w = bgr.shape[:2]

                yuyv_bytes = _frame_to_yuyv(bgr)
                vs = VideoStream()
                vs.width = w
                vs.height = h
                vs.format = 0
                vs.fps = 30
                vs.timestamp_us = int(_t.time() * 1e6)
                vs.video_data = list(yuyv_bytes)
                self._media_ctrl.publish_external_video_stream(vs)
            except Exception as e:
                self._node.get_logger().warn(f"Video push_from_topic error: {e}")

        if self._sub is not None:
            self._node.destroy_subscription(self._sub)
        from sensor_msgs.msg import CompressedImage
        self._sub = self._node.create_subscription(CompressedImage, input_topic, _on_frame, _LOW_LAT_QOS)

        return {"state": "pushing", "input_topic": input_topic}

    def _do_stop_external_push(self) -> dict:
        self._stop_external()
        self._pushing = False
        if self._sub is not None:
            self._node.destroy_subscription(self._sub)
            self._sub = None
        return {"state": "stopped"}

    # ── desensed-video (subprocess) ───────────────────────────────────────

    def _start_desensed(self) -> dict:
        if self._proc_desensed and self._proc_desensed.poll() is None:
            return {"state": "already_running", "topic_out": [{"topic": self._desensed_topic, "format": "image/jpeg"}]}
        import sys
        self._proc_desensed = subprocess.Popen(
            [sys.executable, "-c",
             f"import sys; sys.path.insert(0, '/work'); from device import _desensed_video_subprocess; _desensed_video_subprocess({self._namespace!r})"],
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
        )
        def _fwd():
            for line in self._proc_desensed.stdout:
                print(line.decode(errors='replace').rstrip(), flush=True)
        threading.Thread(target=_fwd, daemon=True).start()
        return {"state": "running", "topic_out": [{"topic": self._desensed_topic, "format": "image/jpeg"}]}

    def _stop_desensed(self) -> dict:
        if self._proc_desensed:
            self._proc_desensed.terminate()
            self._proc_desensed = None
        return {"state": "stopped"}
