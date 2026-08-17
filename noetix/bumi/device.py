#!/usr/bin/env python3
"""
drivers/noetix/bumi/device.py — Noetix Bumi-EDU 设备插件实现。
插件列表：
  - StatePlugin: joints (21-DOF skeleton), imu, battery, model (URDF resource)
  - LocoPlugin: locomotion, stand-up/prone storage, semantic actions and action recording
  - ArmPlugin: bilateral 8-DOF arm position control through LowController
  - MicPlugin: 8ch mic capture → mono PCM 16kHz
  - SpeakerPlugin: audio playback via MediaController
  - CameraPlugin: Realsense D435i color + depth
  - MotionStatePlugin: combined whole-body motion state
"""

import json
import math
import os
import struct
import subprocess
import threading
import time
from pathlib import Path
from typing import Any

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

# LowController motor IDs and mechanical limits documented by Noetix. Card
# inputs use degrees; MotorCmd.pos is always sent in radians.
_BUMI_ARM_JOINTS = {
    "left": (
        (0, "shoulder_pitch", -135.0, 135.0),
        (1, "shoulder_roll", -8.0, 111.0),
        (2, "shoulder_yaw", -90.0, 90.0),
        (3, "elbow_pitch", -129.0, 0.0),
    ),
    "right": (
        (10, "shoulder_pitch", -135.0, 135.0),
        (11, "shoulder_roll", -111.0, 8.0),
        (12, "shoulder_yaw", -90.0, 90.0),
        (13, "elbow_pitch", -129.0, 0.0),
    ),
}

# Fixed gains from the vendor Bumi policy configuration, reordered into motor
# ID order. They are intentionally not exposed through the MCP schema.
_BUMI_LOWCONTROL_KP = (
    12.0, 12.0, 12.0, 12.0,
    60.0, 60.0, 60.0, 45.0, 10.0, 10.0,
    12.0, 12.0, 12.0, 12.0,
    60.0, 60.0, 60.0, 45.0, 10.0, 10.0,
    53.0,
)
_BUMI_LOWCONTROL_KD = (
    0.4, 0.4, 0.4, 0.4,
    3.0, 3.0, 2.5, 2.0, 0.5, 0.5,
    0.4, 0.4, 0.4, 0.4,
    3.0, 3.0, 2.5, 2.0, 0.5, 0.5,
    3.4,
)

# ── ControlCmd Mapping ────────────────────────────────────────────────────────
# Lazy-loaded from highcontrol_py.ControlCmd enum at runtime

_POSTURE_ACTIONS = {
    "stand_up": ("FALLTOSTAND", {27}),
    "lie_prone": ("STANDTOFALL", {28, 30}),
}

_PRESET_ACTIONS = {
    "wave": ("SWING", {8}),
    "handshake": ("SHAKE", {9}),
    "cheer": ("CHEER", {10}),
    "dance_1": ("DANCE", {5}),
    "dance_2": ("DANCE1", {31}),
    "dance_3": ("DANCE2", {32}),
    "wipe_tears": ("TEAR", {33}),
    "reset": ("WALK", {2}),
}

_TEACHING_ACTIONS = {
    "start_recording": ("STARTTEACH", {11}),
    # ENDTEACH is deprecated. SAVETEACH finishes the recording and saves it.
    "finish_and_save_recording": ("SAVETEACH", {12, 14, 29}),
    "play_recording": ("PLAYTEACH", {23}),
}

_SEMANTIC_ACTION_WORKMODES = {5, 8, 9, 10, 31, 32, 33}
_TEAR_AUTO_EXIT_S = 5.0
_PLAYBACK_MOVING_THRESHOLD = 0.15
_PLAYBACK_STATIONARY_THRESHOLD = 0.15
_PLAYBACK_STATIONARY_CONFIRM_S = 3.0
_PLAYBACK_MOTION_START_TIMEOUT_S = 10.0
_PLAYBACK_MAX_MONITOR_S = 120.0

# Face-up stand-up safety gate, calibrated from the supplied Bumi samples.
# Linear acceleration is expressed in the robot IMU frame.  Comparing its
# normalized direction avoids depending on the exact measured gravity value.
_STAND_POSE_SAMPLE_COUNT = 5
_STAND_POSE_SAMPLE_INTERVAL_S = 0.05
_FACE_UP_GRAVITY_DIRECTION = (0.98480989, 0.00318972, 0.17360677)
_FACE_UP_MAX_GRAVITY_ANGLE_DEG = 8.0
_STAND_POSE_ACCELERATION_RANGE = (9.2, 10.4)
_STAND_POSE_MAX_ANGULAR_VELOCITY = 0.10
_STAND_POSE_MAX_JOINT_VELOCITY = 0.15
_STAND_POSE_MAX_JOINT_VELOCITY_SPIKE = 0.30

# Median joint references from the verified face-up sample.  Tolerances are
# deliberately wider than its sensor noise, while still rejecting the supplied
# bent-leg sample and grossly misplaced limbs.  IMU and joint checks must both
# pass; joint positions alone are not treated as proof of physical pose.
_FACE_UP_JOINT_REFERENCES = (
    0.37175, 0.02994, -0.07954, -0.53407,
    -0.14630, -0.00973, -0.00858, -0.03567, -0.29445, 0.00682,
    0.40684, -0.09136, 0.07916, -0.53845,
    -0.15660, -0.01545, 0.09556, -0.04482, -0.25210, -0.01301,
    0.01850,
)
_FACE_UP_JOINT_TOLERANCES = (
    0.65, 0.45, 0.45, 0.55,
    0.40, 0.30, 0.35, 0.40, 0.60, 0.60,
    0.65, 0.45, 0.45, 0.55,
    0.40, 0.30, 0.35, 0.40, 0.60, 0.60,
    0.45,
)

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


# ── ArmPlugin (actuator, LowController) ──────────────────────────────────────

# LowController arm card. This is intentionally separate from LocoPlugin even
# though both are actuator plugins: they use different vendor control layers.
class ArmPlugin:
    """Smooth bilateral arm position control using the 21-motor API."""

    PREFIX = "arm"
    _DEFAULT_SPEED_DEG_S = 20.0
    _MIN_SPEED_DEG_S = 10.0
    _MAX_SPEED_DEG_S = 30.0
    _CONTROL_INTERVAL_S = 0.01  # SDK send_thread republishes at 500 Hz.
    _MIN_TRAJECTORY_S = 0.5
    _FEEDBACK_TIMEOUT_S = 2.0
    _POSITION_TOLERANCE_DEG = 5.0
    _STATIONARY_VELOCITY_THRESHOLD = 0.20
    _STATIONARY_SAMPLE_COUNT = 5

    def __init__(self, plugin_config: dict, namespace: str, executor, low_ctrl,
                 high_ctrl=None):
        self._low_ctrl = low_ctrl
        self._high_ctrl = high_ctrl
        self._move_lock = threading.Lock()
        self._stop_event = threading.Event()
        self._motor_cmd_cls = None
        self._initialization_error: str | None = None

    def get_tool(self) -> dict:
        left_limits = [[item[2], item[3]] for item in _BUMI_ARM_JOINTS["left"]]
        right_limits = [[item[2], item[3]] for item in _BUMI_ARM_JOINTS["right"]]
        return {
            "name": "arm",
            "type": "actuator",
            "multiInstance": False,
            "description": (
                "Move Bumi's left arm, right arm, or both arms to specified joint angles "
                "through LowController. Provide at least one four-angle array in degrees, "
                "ordered as [shoulder pitch, shoulder roll, shoulder yaw, elbow pitch]. "
                "The driver validates every documented joint limit and moves smoothly; "
                "KP, KD, and torque are fixed internally and cannot be supplied by the user. "
                "The command is accepted only when HighController reports workmode=2 "
                "(walking) and five state samples confirm that Bumi is stationary. "
                "LowController takes control of all 21 motors even for an arm-only move. "
                "Per the vendor safety instructions, test only with the robot secured by a "
                "load-bearing safety hanger, and do not call high-level motion cards at the "
                "same time."
            ),
            "inputSchema": {
                "type": "object",
                "properties": {
                    "left_positions": {
                        "type": "array",
                        "items": {"type": "number"},
                        "minItems": 4,
                        "maxItems": 4,
                        "description": (
                            "Optional left-arm target angles in degrees: [shoulder_pitch, "
                            "shoulder_roll, shoulder_yaw, elbow_pitch]. Per-joint ranges are "
                            f"{left_limits}. Omit this field to keep the left arm at its "
                            "measured starting angles. Example: [0, 20, 0, -45]."
                        ),
                    },
                    "right_positions": {
                        "type": "array",
                        "items": {"type": "number"},
                        "minItems": 4,
                        "maxItems": 4,
                        "description": (
                            "Optional right-arm target angles in degrees: [shoulder_pitch, "
                            "shoulder_roll, shoulder_yaw, elbow_pitch]. Per-joint ranges are "
                            f"{right_limits}. Omit this field to keep the right arm at its "
                            "measured starting angles. Example: [0, -20, 0, -45]."
                        ),
                    },
                    "speed_deg_s": {
                        "type": "number",
                        "minimum": self._MIN_SPEED_DEG_S,
                        "maximum": self._MAX_SPEED_DEG_S,
                        "default": self._DEFAULT_SPEED_DEG_S,
                        "description": (
                            "Maximum joint speed in degrees per second. Range 10-30; default "
                            "20. The minimum keeps worst-case moves within the MCP request window."
                        ),
                    },
                },
                "anyOf": [
                    {"required": ["left_positions"]},
                    {"required": ["right_positions"]},
                ],
            },
            "topic_out": [],
        }

    def start(self) -> None:
        try:
            from lowcontrol_py import MotorCmd
            self._motor_cmd_cls = MotorCmd
            self._verify_joint_mapping()
        except Exception as exc:
            self._initialization_error = str(exc)
            print(f"[ArmPlugin] unavailable: {exc}", flush=True)

    def stop(self) -> None:
        # Request the interpolation loop to stop at its latest smoothly reached
        # position. Do not enqueue a damping command here because it could
        # unexpectedly release a standing robot.
        self._stop_event.set()

    def dispatch(self, action: str, args: dict) -> dict:
        if action in ("start", "info"):
            return {
                "state": "ready" if self._initialization_error is None else "error",
                "control": "LowController 21-motor position command",
                "supported_joints": 8,
                "initialization_error": self._initialization_error,
            }
        if action == "stop":
            self._stop_event.set()
            return {"state": "stopping"}
        if action not in ("arm", "move"):
            return {"state": "error", "error": f"unknown arm operation: {action}"}
        return self._move(args)

    def _verify_joint_mapping(self) -> None:
        expected = {
            "arm_l1_joint": 0, "arm_l2_joint": 1,
            "arm_l3_joint": 2, "arm_l4_joint": 3,
            "arm_r1_joint": 10, "arm_r2_joint": 11,
            "arm_r3_joint": 12, "arm_r4_joint": 13,
        }
        mismatches = []
        for name, expected_id in expected.items():
            actual_id = int(self._low_ctrl.getJointsIndex(name))
            if actual_id != expected_id:
                mismatches.append(
                    {"joint": name, "expected_id": expected_id, "actual_id": actual_id})
        if mismatches:
            raise RuntimeError(f"LowController arm joint mapping mismatch: {mismatches}")

    @staticmethod
    def _decode_positions(value: Any, field_name: str) -> tuple[Any, dict | None]:
        if isinstance(value, str):
            try:
                value = json.loads(value)
            except json.JSONDecodeError as exc:
                return None, {
                    "state": "error", "command_sent": False,
                    "error": f"{field_name} must be a valid JSON array: {exc}",
                }
        if not isinstance(value, (list, tuple)) or len(value) != 4:
            return None, {
                "state": "error", "command_sent": False,
                "error": f"{field_name} must contain exactly four joint angles",
            }
        converted = []
        for index, item in enumerate(value):
            if isinstance(item, bool) or not isinstance(item, (int, float)):
                return None, {
                    "state": "error", "command_sent": False,
                    "error": f"{field_name}[{index}] must be a finite number",
                }
            number = float(item)
            if not math.isfinite(number):
                return None, {
                    "state": "error", "command_sent": False,
                    "error": f"{field_name}[{index}] must be a finite number",
                }
            converted.append(number)
        return converted, None

    def _validate_request(self, args: dict) -> tuple[dict, float] | dict:
        requested: dict[str, list[float]] = {}
        violations = []
        for side in ("left", "right"):
            field_name = f"{side}_positions"
            if field_name not in args or args[field_name] is None:
                continue
            values, error = self._decode_positions(args[field_name], field_name)
            if error is not None:
                return error
            requested[side] = values
            for value, (_, joint_name, lower, upper) in zip(
                    values, _BUMI_ARM_JOINTS[side]):
                if value < lower or value > upper:
                    violations.append({
                        "side": side,
                        "joint": joint_name,
                        "value_deg": value,
                        "minimum_deg": lower,
                        "maximum_deg": upper,
                    })

        if not requested:
            return {
                "state": "error", "command_sent": False,
                "error": "Provide left_positions, right_positions, or both",
            }
        if violations:
            return {
                "state": "error", "command_sent": False,
                "error": "One or more arm targets exceed the documented joint limits",
                "violations": violations,
            }

        speed = args.get("speed_deg_s", self._DEFAULT_SPEED_DEG_S)
        if isinstance(speed, bool) or not isinstance(speed, (int, float)):
            return {
                "state": "error", "command_sent": False,
                "error": "speed_deg_s must be a number from 10 to 30",
            }
        speed = float(speed)
        if not math.isfinite(speed) or not self._MIN_SPEED_DEG_S <= speed <= self._MAX_SPEED_DEG_S:
            return {
                "state": "error", "command_sent": False,
                "error": "speed_deg_s must be between 10 and 30",
            }
        return requested, speed

    def _new_commands(self, positions: list[float]) -> list:
        commands = []
        for motor_id, position in enumerate(positions):
            cmd = self._motor_cmd_cls()
            cmd.pos = float(position)
            cmd.vel = 0.0
            cmd.tau = 0.0
            cmd.kp = _BUMI_LOWCONTROL_KP[motor_id]
            cmd.kd = _BUMI_LOWCONTROL_KD[motor_id]
            cmd.motor_id = motor_id
            commands.append(cmd)
        return commands

    @staticmethod
    def _read_states(low_ctrl):
        states = low_ctrl.get_joint_state()
        if len(states) != 21:
            raise RuntimeError(f"expected 21 joint states, received {len(states)}")
        motor_ids = [int(states[i].motor_id) for i in range(21)]
        if motor_ids != list(range(21)):
            raise RuntimeError(
                f"LowController joint state is not ready; motor IDs are {motor_ids}")
        positions = [float(states[i].pos) for i in range(21)]
        velocities = [float(states[i].vel) for i in range(21)]
        if not all(math.isfinite(value) for value in positions + velocities):
            raise RuntimeError("LowController returned a non-finite joint state")
        return states, positions

    def _check_stationary_walking(self) -> dict | None:
        maximum_velocity = 0.0
        for sample_index in range(self._STATIONARY_SAMPLE_COUNT):
            mode = int(self._high_ctrl.get_mode())
            if mode == 26:
                return {
                    "state": "error", "command_sent": False,
                    "current_workmode": 26,
                    "current_workmode_name": "protection",
                    "protection": True,
                    "error": (
                        "Robot is in protection mode. Restart and inspect it before "
                        "attempting LowController motion."
                    ),
                }
            if mode != 2:
                return {
                    "state": "error", "command_sent": False,
                    "current_workmode": mode,
                    "current_workmode_name": _WORKMODE_NAMES.get(mode, "unknown"),
                    "required_workmode": 2,
                    "required_workmode_name": "walking",
                    "error": (
                        "Arm movement requires workmode=2 (walking). The card does not "
                        "switch modes automatically."
                    ),
                }
            states = self._high_ctrl.get_joint_state()
            if len(states) != 21:
                return {
                    "state": "error", "command_sent": False,
                    "error": f"HighController returned {len(states)} joints; expected 21",
                }
            sample_maximum = max(abs(float(state.vel)) for state in states)
            if not math.isfinite(sample_maximum):
                return {
                    "state": "error", "command_sent": False,
                    "error": "HighController returned a non-finite joint velocity",
                }
            maximum_velocity = max(maximum_velocity, sample_maximum)
            if sample_index + 1 < self._STATIONARY_SAMPLE_COUNT:
                time.sleep(0.05)
        if maximum_velocity >= self._STATIONARY_VELOCITY_THRESHOLD:
            return {
                "state": "error", "command_sent": False,
                "maximum_joint_velocity_rad_s": round(maximum_velocity, 4),
                "stationary_threshold_rad_s": self._STATIONARY_VELOCITY_THRESHOLD,
                "error": (
                    "The robot is in walking mode but is not stationary. Stop locomotion, "
                    "wait for the body to settle, and retry."
                ),
            }
        return None

    def _move(self, args: dict) -> dict:
        if self._initialization_error is not None or self._motor_cmd_cls is None:
            return {
                "state": "error", "command_sent": False,
                "error": "LowController arm control is unavailable",
                "detail": self._initialization_error,
            }
        validated = self._validate_request(args)
        if isinstance(validated, dict):
            return validated
        requested, speed_deg_s = validated

        if self._high_ctrl is None:
            return {
                "state": "error", "command_sent": False,
                "error": (
                    "Cannot verify walking and stationary prerequisites because "
                    "HighController is unavailable; no LowController command was sent."
                ),
            }
        try:
            prerequisite_error = self._check_stationary_walking()
        except Exception as exc:
            return {
                "state": "error", "command_sent": False,
                "error": f"Failed to verify arm movement prerequisites: {exc}",
            }
        if prerequisite_error is not None:
            return prerequisite_error

        if not self._move_lock.acquire(blocking=False):
            return {
                "state": "error", "command_sent": False,
                "error": "Another arm movement is already running",
            }

        command_sent = False
        try:
            self._stop_event.clear()
            low_states, start_positions = self._read_states(self._low_ctrl)
            documented_faults = []
            for index, state in enumerate(low_states):
                error = int(getattr(state, "error", 0))
                if error in _MOTOR_ERROR_NAMES:
                    documented_faults.append({
                        "motor_id": int(getattr(state, "motor_id", index)),
                        "joint": _BUMI_JOINT_NAMES[index],
                        "error": error,
                        "error_name": _MOTOR_ERROR_NAMES[error],
                    })
            if documented_faults:
                return {
                    "state": "error", "command_sent": False,
                    "error": "Documented motor faults are active; arm movement was blocked",
                    "motor_faults": documented_faults,
                }
            target_positions = list(start_positions)
            target_ids = []
            for side, values in requested.items():
                for value_deg, (motor_id, _, _, _) in zip(
                        values, _BUMI_ARM_JOINTS[side]):
                    target_positions[motor_id] = math.radians(value_deg)
                    target_ids.append(motor_id)

            max_delta_deg = max(
                abs(math.degrees(target_positions[i] - start_positions[i]))
                for i in target_ids
            )
            # Cosine easing has a peak velocity of pi/2 times its average.
            duration_s = max(
                self._MIN_TRAJECTORY_S,
                max_delta_deg * math.pi / (2.0 * speed_deg_s),
            )
            commands = self._new_commands(start_positions)
            started_at = time.monotonic()
            next_update = started_at

            while True:
                if self._stop_event.is_set():
                    return {
                        "state": "error",
                        "command_sent": command_sent,
                        "error": "Arm movement was interrupted before reaching its target",
                        "control_state": (
                            "LowController retains its latest 21-motor command; keep the "
                            "safety hanger attached."
                        ),
                    }
                elapsed = time.monotonic() - started_at
                progress = min(1.0, elapsed / duration_s)
                blend = 0.5 - 0.5 * math.cos(math.pi * progress)
                for motor_id in range(21):
                    commands[motor_id].pos = (
                        start_positions[motor_id]
                        + (target_positions[motor_id] - start_positions[motor_id]) * blend
                    )
                self._low_ctrl.set_joint(commands)
                command_sent = True
                if progress >= 1.0:
                    break
                next_update += self._CONTROL_INTERVAL_S
                time.sleep(max(0.0, next_update - time.monotonic()))

            feedback_deadline = time.monotonic() + self._FEEDBACK_TIMEOUT_S
            measured_positions = start_positions
            errors_deg = {}
            while True:
                _, measured_positions = self._read_states(self._low_ctrl)
                errors_deg = {
                    f"{side}_{joint_name}": round(math.degrees(
                        measured_positions[motor_id] - target_positions[motor_id]), 3)
                    for side, values in requested.items()
                    for _, (motor_id, joint_name, _, _) in zip(
                        values, _BUMI_ARM_JOINTS[side])
                }
                if max(abs(value) for value in errors_deg.values()) <= self._POSITION_TOLERANCE_DEG:
                    break
                if time.monotonic() >= feedback_deadline:
                    break
                time.sleep(0.05)
            max_error_deg = max(abs(value) for value in errors_deg.values())
            measured = {
                side: [
                    round(math.degrees(measured_positions[motor_id]), 3)
                    for motor_id, _, _, _ in _BUMI_ARM_JOINTS[side]
                ]
                for side in requested
            }
            completed = max_error_deg <= self._POSITION_TOLERANCE_DEG
            return {
                "state": "completed" if completed else "error",
                "command_sent": True,
                "requested_positions_deg": requested,
                "measured_positions_deg": measured,
                "joint_errors_deg": errors_deg,
                "max_abs_error_deg": round(max_error_deg, 3),
                "completion_tolerance_deg": self._POSITION_TOLERANCE_DEG,
                "speed_deg_s": speed_deg_s,
                "trajectory_duration_s": round(duration_s, 3),
                "control_state": (
                    "LowController continues holding the latest complete 21-motor "
                    "position command after this call returns."
                ),
                "message": (
                    "Arm movement completed and measured feedback is within tolerance."
                    if completed else
                    "The trajectory was sent, but arm feedback did not reach the target "
                    "within 5 degrees before the 2-second feedback timeout. Keep the "
                    "safety hanger attached and inspect the robot before retrying."
                ),
            }
        except Exception as exc:
            return {
                "state": "error",
                "command_sent": command_sent,
                "error": f"LowController arm movement failed: {exc}",
                "safety": (
                    "Keep the robot supported by the safety hanger and stop other "
                    "motion commands before inspection."
                ),
            }
        finally:
            self._move_lock.release()


# ── LocoPlugin (actuator, multi-tool) ───────────────────────────────

class LocoPlugin:
    PREFIX = "loco"

    def __init__(self, plugin_config: dict, namespace: str, executor, high_ctrl):
        self._high_ctrl = high_ctrl
        self._namespace = namespace
        self._lock = threading.Lock()
        self._action_lock = threading.Lock()
        self._last_cmd_time: float = 0.0
        self._move_thread: threading.Thread | None = None
        self._move_stop_event = threading.Event()
        self._control_period = 0.01       # 100 Hz, matching the vendor demo
        self._control_preroll_s = 0.3     # refresh an already active DDS writer path
        self._control_cold_preroll_s = 3.0
        self._control_channel_warmed = False
        self._auto_exit_lock = threading.Lock()
        self._auto_exit_timers: dict[str, threading.Timer] = {}
        self._playback_monitor_stop: threading.Event | None = None
        self._playback_monitor_thread: threading.Thread | None = None

    def get_tools(self) -> list:
        return [
            self._loco_tool(),
            self._stand_up_lie_prone_tool(),
            self._semantic_action_tool(),
            self._action_recording_tool(),
        ]

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

    def _stand_up_lie_prone_tool(self) -> dict:
        return {
            "name": "stand_up_lie_prone",
            "type": "actuator",
            "multiInstance": False,
            "description": "让 Bumi 从仰面平躺自主起身，或从正常站立姿态趴下收纳。stand_up 会先用 IMU 和 21 个关节状态检查仰面方向、静止状态及四肢姿态，不通过时不发送任何控制命令；通过后在必要时自动使能，并直接执行自主起身，不会进入仅适用于人工扶站的准备模式。传感器无法检查地面、脚下异物和周围空间，用户仍须完成 action 描述中的现场安全检查。",
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {
                        "type": "string",
                        "enum": list(_POSTURE_ACTIONS),
                        "description": "stand_up=自主起身：仅限机器人面朝上平躺、四肢自然放置、双腿伸直、脚底无异物，并在平坦防滑地面留出至少 3m×3m 无人无障碍空间；lie_prone=趴下收纳：仅限机器人已稳定站立，并在平坦防滑地面留出至少 3m×3m 无人无障碍空间。",
                    },
                },
                "required": ["action"],
                "x-action-params": {
                    "stand_up": {
                        "params": [],
                        "description": "仅从 disabled/enabled 状态自主起身。卡片先自动检查机器人是否静止、躯干是否仰面及 21 个关节是否接近安全平躺姿态；检查失败时不会使能或起身。检查通过后直接调用 FALLTOSTAND，不会调用用于人工扶站的 SWITCH/ready。用户仍须确认平坦防滑地面、脚底无异物且周围 3m×3m 安全。",
                    },
                    "lie_prone": {
                        "params": [],
                        "description": "仅从 walking 状态趴下收纳。调用前必须由用户确认机器人稳定站立且周围 3m×3m 安全；其他工作模式不会发送动作命令。",
                    },
                },
            },
            "topic_out": [],
        }

    def _semantic_action_tool(self) -> dict:
        return {
            "name": "semantic_action", "type": "actuator", "multiInstance": False,
            "description": "执行 Bumi 出厂预设的挥手、握手、欢呼、三种舞蹈和擦眼泪动作。卡片会自动进入动作所需的行走模式。执行前必须确认机器人已正常站立、双脚着地，地面平坦防滑且周围无人和障碍物；舞蹈建议至少留出 3m×3m 空间。",
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {
                        "type": "string", "enum": list(_PRESET_ACTIONS),
                        "description": "wave=挥手；handshake=握手；cheer=欢呼；dance_1/dance_2/dance_3=三种出厂舞蹈；wipe_tears=擦眼泪并在定时结束后自动返回 walking；reset=终止/退出当前语义动作并返回 walking 模式。",
                    },
                },
                "required": ["action"],
                "x-action-params": {
                    name: {"params": [], "description": description}
                    for name, description in {
                        "wave": "挥手。确认机器人稳定站立，手臂摆动范围内无人和障碍物。",
                        "handshake": "握手。确认机器人稳定站立，人员不要拉扯机器人手臂。",
                        "cheer": "欢呼。确认机器人稳定站立，肢体活动范围内无人和障碍物。",
                        "dance_1": "执行舞蹈 1。机器人属于盲舞，至少留出 3m×3m 平坦防滑空间。",
                        "dance_2": "执行舞蹈 2。机器人属于盲舞，至少留出 3m×3m 平坦防滑空间。",
                        "dance_3": "执行舞蹈 3。机器人属于盲舞，至少留出 3m×3m 平坦防滑空间。",
                        "wipe_tears": "执行擦眼泪动作，固定 5 秒后自动返回 walking。确认机器人稳定站立且手臂周围无障碍物。",
                        "reset": "结束当前语义动作并返回 workmode=2（walking），用于动作后复位。",
                    }.items()
                } | {
                    "wipe_tears": {
                        "params": [],
                        "description": "执行擦眼泪动作，固定 5 秒后自动返回 walking，无需填写时长或调用 reset。",
                    },
                },
            },
            "topic_out": [],
        }

    def _action_recording_tool(self) -> dict:
        return {
            "name": "action_recording", "type": "actuator", "multiInstance": False,
            "description": "录制、结束并保存或播放 Bumi 示教动作。start_recording 和 play_recording 会自动进入所需行走模式；finish_and_save_recording 只能在已开始录制后使用；play_recording 会根据关节运动状态判断播放结束并自动返回 walking。",
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {
                        "type": "string", "enum": list(_TEACHING_ACTIONS),
                        "description": "start_recording=开始录制示教；finish_and_save_recording=结束当前录制并保存；play_recording=播放已保存动作，检测到动作完成后自动返回 walking。",
                    },
                    "recording_id": {
                        "type": "integer", "minimum": 0, "maximum": 65535,
                        "description": "动作记录编号，范围 0～65535。结束并保存、播放时必须填写；开始录制时无需填写。保存与播放同一动作时使用相同编号。",
                    },
                },
                "required": ["action"],
                "x-action-params": {
                    "start_recording": {"params": [], "description": "自动准备模式后开始示教录制。确认机器人稳定站立；缓慢引导关节，禁止强推至机械限位。"},
                    "finish_and_save_recording": {"params": ["recording_id"], "description": "结束当前示教并保存到 recording_id。若尚未开始录制，则不会发送命令。"},
                    "play_recording": {"params": ["recording_id"], "description": "自动准备并播放 recording_id；卡片根据 workmode 和 21 个关节速度推断动作完成，随后自动返回 walking，无需填写时长或手动停止。"},
                },
            },
            "topic_out": [],
        }

    def start(self) -> None:
        pass

    def stop(self) -> None:
        self._cancel_all_auto_exits()
        self._cancel_playback_monitor()
        self._stop_move()

    def dispatch(self, action: str, args: dict) -> dict | None:
        if action == "start":
            return {"state": "ready"}
        if action == "stop":
            self._stop_move()
            return {"state": "idle"}

        tool_name = args.pop('_tool_name', '')

        if tool_name == "loco" and action == "move":
            return self._do_move(args)
        if tool_name == "loco" and action == "stop_move":
            return self._stop_move()
        if tool_name == "stand_up_lie_prone" and action in _POSTURE_ACTIONS:
            return self._do_posture_action(action, args)
        if tool_name == "semantic_action" and action in _PRESET_ACTIONS:
            return self._do_preset_action(action, args)
        if tool_name == "action_recording" and action in _TEACHING_ACTIONS:
            return self._do_teaching_action(action, args)
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
        mode = int(self._high_ctrl.get_mode())
        if mode == 26:
            return {"state": "error", "error": "Robot in protection mode, cannot move"}
        if mode != 2:
            return {
                "state": "error",
                "error": (
                    f"movement requires workmode=2 (walking); current mode is "
                    f"{mode} ({_WORKMODE_NAMES.get(mode, 'unknown')}). "
                    "Use switch_mode switch=walk first."
                ),
            }

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

    def _do_posture_action(self, action: str, args: dict) -> dict:
        safety = self._safety_requirements(action)
        current_mode = int(self._high_ctrl.get_mode())
        if current_mode == 26:
            return self._protection_error(action, [], current_mode, safety)
        allowed_modes = {0, 30} if action == "stand_up" else {2}
        if current_mode not in allowed_modes:
            return {
                "state": "error", "command_sent": False,
                "requested_action": action,
                "current_workmode": current_mode,
                "current_workmode_name": _WORKMODE_NAMES.get(current_mode, "unknown"),
                "allowed_workmodes": [
                    {"code": mode, "name": _WORKMODE_NAMES.get(mode, "unknown")}
                    for mode in sorted(allowed_modes)
                ],
                "error": (
                    "stand_up is allowed only from disabled or enabled mode after the robot has been placed face-up. It is blocked from ready, walking, and action modes to prevent a standing robot from collapsing."
                    if action == "stand_up" else
                    "lie_prone is allowed only from walking mode after stable standing has been confirmed."
                ),
                "safety_requirements": safety,
            }
        pose_check = None
        if action == "stand_up":
            pose_check = self._check_face_up_stand_pose()
            if not pose_check["safe_to_stand"]:
                return {
                    "state": "error",
                    "command_sent": False,
                    "requested_action": action,
                    "current_workmode": current_mode,
                    "current_workmode_name": _WORKMODE_NAMES.get(current_mode, "unknown"),
                    "error": "Automatic pose check failed. No enable, prepare, or stand-up command was sent.",
                    "pose_check": pose_check,
                    "message": "Place the robot face-up with both legs straight and all limbs naturally positioned, wait until it is still, then try again.",
                    "safety_requirements": safety,
                }
        prepared = (
            self._prepare_stand_up(action)
            if action == "stand_up" else
            self._prepare_workmode(2, action)
        )
        if prepared["state"] == "error":
            prepared["safety_requirements"] = safety
            if pose_check is not None:
                prepared["pose_check"] = pose_check
            return prepared
        command_name, expected_modes = _POSTURE_ACTIONS[action]
        result = self._trigger_user_action(
            action, command_name, expected_modes, prepared["steps"], safety)
        if pose_check is not None:
            result["pose_check"] = pose_check
            result["pose_verification"] = (
                "IMU direction, stillness, and median joint positions passed the automatic "
                "face-up check. Floor condition, objects under the feet, and surrounding "
                "clearance still require visual confirmation by the user."
            )
        return result

    def _prepare_stand_up(self, requested_action: str) -> dict:
        """Reach enabled mode without entering ready, which assumes assisted standing."""
        self._move_stop_event.set()
        if self._move_thread and self._move_thread.is_alive():
            self._move_thread.join(timeout=1)

        steps = []
        mode = int(self._high_ctrl.get_mode())
        if mode == 26:
            return self._protection_error(requested_action, steps, mode)
        if mode == 30:
            mode = self._run_preparation_step("enable", "START", {0}, steps)
            if mode == 26:
                return self._protection_error(requested_action, steps, mode)
            if mode != 0:
                return self._preparation_error(
                    requested_action, steps, mode,
                    "The robot did not enter enabled mode. FALLTOSTAND was not sent.")
        if mode != 0:
            return self._preparation_error(
                requested_action, steps, mode,
                "Autonomous stand-up requires disabled or enabled mode. FALLTOSTAND was not sent.")
        return {"state": "completed", "steps": steps, "workmode": mode}

    def _check_face_up_stand_pose(self) -> dict:
        """Fail-closed sensor check before any stand-up preparation command."""
        acceleration_samples = []
        angular_speed_samples = []
        joint_velocity_samples = []
        joint_position_samples = []
        documented_faults = []

        try:
            for sample_index in range(_STAND_POSE_SAMPLE_COUNT):
                imu = self._high_ctrl.get_imu_data()
                joints = self._high_ctrl.get_joint_state()
                if len(joints) != len(_BUMI_JOINT_NAMES):
                    raise RuntimeError(
                        f"HighController returned {len(joints)} joints; expected "
                        f"{len(_BUMI_JOINT_NAMES)}")

                acceleration = [float(imu.linear_acc[index]) for index in range(3)]
                angular_velocity = [float(imu.angular_vel[index]) for index in range(3)]
                positions = [float(joint.pos) for joint in joints]
                velocities = [abs(float(joint.vel)) for joint in joints]
                values = acceleration + angular_velocity + positions + velocities
                if not all(math.isfinite(value) for value in values):
                    raise RuntimeError("IMU or joint state contains a non-finite value")

                acceleration_samples.append(acceleration)
                angular_speed_samples.append(
                    math.sqrt(sum(value * value for value in angular_velocity)))
                joint_velocity_samples.append(max(velocities))
                joint_position_samples.append(positions)

                for index, joint in enumerate(joints):
                    error = int(getattr(joint, "error", 0))
                    if error in _MOTOR_ERROR_NAMES:
                        documented_faults.append({
                            "motor_id": int(getattr(joint, "motor_id", index)),
                            "joint": _BUMI_JOINT_NAMES[index],
                            "error": error,
                            "error_name": _MOTOR_ERROR_NAMES[error],
                        })
                if sample_index + 1 < _STAND_POSE_SAMPLE_COUNT:
                    time.sleep(_STAND_POSE_SAMPLE_INTERVAL_S)
        except Exception as exc:
            return {
                "safe_to_stand": False,
                "sample_count": len(acceleration_samples),
                "failed_checks": ["sensor_data_available"],
                "error": str(exc),
                "meaning": "The sensor state could not be verified, so stand-up was blocked.",
            }

        gravity_angles = []
        acceleration_magnitudes = []
        for acceleration in acceleration_samples:
            magnitude = math.sqrt(sum(value * value for value in acceleration))
            acceleration_magnitudes.append(magnitude)
            if magnitude < 1e-9:
                gravity_angles.append(180.0)
                continue
            direction = [value / magnitude for value in acceleration]
            cosine = sum(
                value * reference
                for value, reference in zip(direction, _FACE_UP_GRAVITY_DIRECTION)
            )
            gravity_angles.append(math.degrees(math.acos(max(-1.0, min(1.0, cosine)))))

        median_positions = []
        for joint_index in range(len(_BUMI_JOINT_NAMES)):
            values = sorted(sample[joint_index] for sample in joint_position_samples)
            median_positions.append(values[len(values) // 2])

        out_of_pose_joints = []
        for index, (position, reference, tolerance) in enumerate(zip(
                median_positions, _FACE_UP_JOINT_REFERENCES,
                _FACE_UP_JOINT_TOLERANCES)):
            deviation = abs(position - reference)
            if deviation > tolerance:
                out_of_pose_joints.append({
                    "motor_id": index,
                    "joint": _BUMI_JOINT_NAMES[index],
                    "position_rad": round(position, 5),
                    "reference_rad": reference,
                    "deviation_rad": round(deviation, 5),
                    "maximum_deviation_rad": tolerance,
                })

        max_gravity_angle = max(gravity_angles)
        min_acceleration = min(acceleration_magnitudes)
        max_acceleration = max(acceleration_magnitudes)
        max_angular_speed = max(angular_speed_samples)
        sorted_joint_velocities = sorted(joint_velocity_samples)
        median_joint_velocity = sorted_joint_velocities[
            len(sorted_joint_velocities) // 2]
        max_joint_velocity = sorted_joint_velocities[-1]
        checks = {
            "face_up_orientation": max_gravity_angle <= _FACE_UP_MAX_GRAVITY_ANGLE_DEG,
            "gravity_like_acceleration": (
                min_acceleration >= _STAND_POSE_ACCELERATION_RANGE[0]
                and max_acceleration <= _STAND_POSE_ACCELERATION_RANGE[1]
            ),
            "body_stationary": max_angular_speed <= _STAND_POSE_MAX_ANGULAR_VELOCITY,
            "joints_stationary": (
                median_joint_velocity <= _STAND_POSE_MAX_JOINT_VELOCITY
                and max_joint_velocity <= _STAND_POSE_MAX_JOINT_VELOCITY_SPIKE
            ),
            "joint_pose": not out_of_pose_joints,
            "no_documented_motor_fault": not documented_faults,
        }
        failed_checks = [name for name, passed in checks.items() if not passed]
        return {
            "safe_to_stand": not failed_checks,
            "sample_count": _STAND_POSE_SAMPLE_COUNT,
            "sample_window_s": round(
                (_STAND_POSE_SAMPLE_COUNT - 1) * _STAND_POSE_SAMPLE_INTERVAL_S, 2),
            "checks": checks,
            "failed_checks": failed_checks,
            "measurements": {
                "maximum_gravity_direction_error_deg": round(max_gravity_angle, 3),
                "acceleration_magnitude_range_m_s2": [
                    round(min_acceleration, 3), round(max_acceleration, 3)],
                "maximum_angular_velocity_rad_s": round(max_angular_speed, 4),
                "median_maximum_joint_velocity_rad_s": round(
                    median_joint_velocity, 4),
                "maximum_joint_velocity_rad_s": round(max_joint_velocity, 4),
                "out_of_pose_joints": out_of_pose_joints,
                "documented_motor_faults": documented_faults,
            },
            "thresholds": {
                "maximum_gravity_direction_error_deg": _FACE_UP_MAX_GRAVITY_ANGLE_DEG,
                "acceleration_magnitude_m_s2": list(_STAND_POSE_ACCELERATION_RANGE),
                "maximum_angular_velocity_rad_s": _STAND_POSE_MAX_ANGULAR_VELOCITY,
                "median_maximum_joint_velocity_rad_s": _STAND_POSE_MAX_JOINT_VELOCITY,
                "maximum_joint_velocity_spike_rad_s": (
                    _STAND_POSE_MAX_JOINT_VELOCITY_SPIKE),
                "joint_position_rule": (
                    "The five-sample median of every joint must be within its calibrated "
                    "per-joint maximum deviation. Failed joints include their limits."
                ),
            },
            "meaning": (
                "All available sensor checks passed. This is a safety gate, not proof of "
                "a safe environment; visually confirm the floor, feet, and 3 m x 3 m clearance."
                if not failed_checks else
                "One or more sensor checks failed, so no stand-up preparation or action command was sent."
            ),
        }

    def _do_preset_action(self, action: str, args: dict) -> dict:
        safety = self._safety_requirements(action)
        if action == "reset":
            self._cancel_auto_exit("wipe_tears")
            return self._do_semantic_reset(safety)
        prepared = self._prepare_workmode(2, action)
        if prepared["state"] == "error":
            prepared["safety_requirements"] = safety
            return prepared
        command_name, expected_modes = _PRESET_ACTIONS[action]
        result = self._trigger_user_action(
            action, command_name, expected_modes, prepared["steps"], safety)
        if action == "wipe_tears" and result.get("confirmed_started"):
            self._schedule_auto_walk_exit(
                "wipe_tears", 33, _TEAR_AUTO_EXIT_S, safety)
            result.update({
                "auto_return_to_walk": True,
                "auto_return_after_s": _TEAR_AUTO_EXIT_S,
                "auto_return_condition": (
                    "WALK will be sent only if the robot is still in tear mode when the timer expires."
                ),
            })
        return result

    def _do_teaching_action(self, action: str, args: dict) -> dict:
        safety = self._safety_requirements(action)
        recording_id = None
        if action in ("finish_and_save_recording", "play_recording"):
            if "recording_id" not in args:
                return {
                    "state": "error", "command_sent": False,
                    "error": f"{action} requires recording_id in the range 0 to 65535",
                    "safety_requirements": safety,
                }
            try:
                recording_id = int(args["recording_id"])
            except (TypeError, ValueError):
                return {"state": "error", "command_sent": False,
                        "error": "recording_id must be an integer", "safety_requirements": safety}
            if not 0 <= recording_id <= 65535:
                return {"state": "error", "command_sent": False,
                        "error": "recording_id must be in the range 0 to 65535", "safety_requirements": safety}
        if action == "finish_and_save_recording":
            mode = int(self._high_ctrl.get_mode())
            if mode == 26:
                return self._protection_error(action, [], mode, safety)
            if mode != 11:
                return {
                    "state": "error", "command_sent": False,
                    "requested_action": action,
                    "current_workmode": mode,
                    "current_workmode_name": _WORKMODE_NAMES.get(mode, "unknown"),
                    "error": "No action recording is currently active. Call start_recording first, guide the action, and then finish and save it.",
                    "safety_requirements": safety,
                }
            steps = []
        else:
            prepared = self._prepare_workmode(2, action)
            if prepared["state"] == "error":
                prepared["safety_requirements"] = safety
                return prepared
            steps = prepared["steps"]

        command_name, expected_modes = _TEACHING_ACTIONS[action]
        result = self._trigger_user_action(
            action, command_name, expected_modes, steps, safety,
            index=recording_id or 0, recording_id=recording_id)
        if action == "play_recording" and result.get("confirmed_started"):
            self._schedule_playback_completion_monitor(safety)
            result.update({
                "auto_return_to_walk": True,
                "completion_detection": "inferred_from_workmode_and_joint_velocity",
                "auto_return_condition": (
                    "After joint motion has been observed, WALK is sent when all joints "
                    "remain below the motion_state activity threshold for a sustained "
                    "3-second score while the robot is still in play_teach mode; isolated "
                    "encoder-noise spikes reduce the score instead of resetting it."
                ),
                "completion_detection_note": (
                    "The SDK exposes no explicit playback-completion event; completion is "
                    "inferred from the 21 reported joint velocities."
                ),
            })
        return result

    def _do_semantic_reset(self, safety: str) -> dict:
        self._move_stop_event.set()
        if self._move_thread and self._move_thread.is_alive():
            self._move_thread.join(timeout=1)
        current_mode = int(self._high_ctrl.get_mode())
        if current_mode == 26:
            return self._protection_error("reset", [], current_mode, safety)

        if current_mode == 2:
            return {
                "state": "completed",
                "command_sent": False,
                "requested_action": "reset",
                "confirmed": True,
                "workmode": 2,
                "workmode_name": "walking",
                "preparation_steps": [],
                "safety_requirements": safety,
                "message": "The robot is already in walking mode; no command was sent.",
            }

        if current_mode == 23:
            return {
                "state": "error", "command_sent": False,
                "requested_action": "reset",
                "current_workmode": current_mode,
                "current_workmode_name": "play_teach",
                "error": "semantic_action.reset does not control action recording playback. Wait for the automatic playback timer to return the robot to walking mode.",
                "safety_requirements": safety,
            }

        if current_mode not in _SEMANTIC_ACTION_WORKMODES:
            return {
                "state": "error", "command_sent": False,
                "requested_action": "reset",
                "current_workmode": current_mode,
                "current_workmode_name": _WORKMODE_NAMES.get(current_mode, "unknown"),
                "allowed_workmodes": sorted(_SEMANTIC_ACTION_WORKMODES),
                "error": "reset is allowed only while a semantic action is active. It will not enable the robot or enter ready/walking mode from disabled, enabled, ready, prone, or unknown physical states.",
                "safety_requirements": safety,
            }

        return self._send_walk_exit("reset", safety)

    def _schedule_auto_walk_exit(self, key: str, expected_mode: int,
                                 delay_s: float, safety: str) -> None:
        self._cancel_auto_exit(key)

        def _auto_exit() -> None:
            with self._auto_exit_lock:
                if self._auto_exit_timers.get(key) is not timer:
                    return
                self._auto_exit_timers.pop(key, None)
            current_mode = int(self._high_ctrl.get_mode())
            if current_mode != expected_mode:
                print(
                    f"[loco] {key} auto-return skipped: workmode={current_mode} "
                    f"({_WORKMODE_NAMES.get(current_mode, 'unknown')})",
                    flush=True,
                )
                return
            result = self._send_walk_exit(f"{key}_auto_return", safety)
            print(
                f"[loco] {key} auto-return result: {json.dumps(result)}",
                flush=True,
            )

        timer = threading.Timer(delay_s, _auto_exit)
        timer.daemon = True
        with self._auto_exit_lock:
            self._auto_exit_timers[key] = timer
        timer.start()

    def _schedule_playback_completion_monitor(self, safety: str) -> None:
        self._cancel_playback_monitor()
        stop_event = threading.Event()

        def _monitor() -> None:
            started_at = time.monotonic()
            motion_seen = False
            stationary_score_s = 0.0
            exit_reason = None

            while not stop_event.wait(0.05):
                now = time.monotonic()
                mode = int(self._high_ctrl.get_mode())
                if mode == 2:
                    print(
                        "[loco] play_recording completed: firmware already returned to walking",
                        flush=True,
                    )
                    return
                if mode == 26:
                    print(
                        "[loco] play_recording monitor stopped: robot entered protection mode",
                        flush=True,
                    )
                    return
                if mode != 23:
                    print(
                        f"[loco] play_recording monitor stopped: workmode={mode} "
                        f"({_WORKMODE_NAMES.get(mode, 'unknown')})",
                        flush=True,
                    )
                    return

                try:
                    joint_states = self._high_ctrl.get_joint_state()
                    max_velocity = max(
                        abs(float(state.vel)) for state in joint_states)
                except Exception as exc:
                    print(
                        f"[loco] play_recording completion sample failed: {exc}",
                        flush=True,
                    )
                    continue

                if max_velocity >= _PLAYBACK_MOVING_THRESHOLD:
                    motion_seen = True
                    stationary_score_s = max(0.0, stationary_score_s - 0.25)
                elif motion_seen and max_velocity <= _PLAYBACK_STATIONARY_THRESHOLD:
                    # Disabled Bumi joints occasionally report isolated velocity
                    # spikes around the activity threshold. Accumulate a sustained
                    # stationary score instead of resetting a multi-second timer
                    # after every single noisy sample.
                    stationary_score_s = min(
                        _PLAYBACK_STATIONARY_CONFIRM_S,
                        stationary_score_s + 0.05,
                    )
                    if stationary_score_s >= _PLAYBACK_STATIONARY_CONFIRM_S:
                        exit_reason = "joint_motion_completed"
                        break

                elapsed = now - started_at
                if not motion_seen and elapsed >= _PLAYBACK_MOTION_START_TIMEOUT_S:
                    exit_reason = "no_joint_motion_detected"
                    break
                if elapsed >= _PLAYBACK_MAX_MONITOR_S:
                    exit_reason = "maximum_monitor_time_reached"
                    break

            if stop_event.is_set() or exit_reason is None:
                return
            if int(self._high_ctrl.get_mode()) != 23:
                return
            result = self._send_walk_exit("play_recording_auto_return", safety)
            result["completion_inference"] = exit_reason
            print(
                f"[loco] play_recording auto-return result: {json.dumps(result)}",
                flush=True,
            )

        monitor_thread = threading.Thread(
            target=_monitor,
            daemon=True,
            name="bumi_playback_completion_monitor",
        )
        with self._auto_exit_lock:
            self._playback_monitor_stop = stop_event
            self._playback_monitor_thread = monitor_thread
        monitor_thread.start()

    def _cancel_playback_monitor(self) -> None:
        with self._auto_exit_lock:
            stop_event = self._playback_monitor_stop
            self._playback_monitor_stop = None
            self._playback_monitor_thread = None
        if stop_event is not None:
            stop_event.set()

    def _cancel_auto_exit(self, key: str) -> None:
        with self._auto_exit_lock:
            timer = self._auto_exit_timers.pop(key, None)
        if timer is not None:
            timer.cancel()

    def _cancel_all_auto_exits(self) -> None:
        with self._auto_exit_lock:
            timers = list(self._auto_exit_timers.values())
            self._auto_exit_timers.clear()
        for timer in timers:
            timer.cancel()

    def _send_walk_exit(self, requested_action: str, safety: str) -> dict:
        observed = self._send_edge_and_wait(
            _get_control_cmd("WALK"), {2, 26}, timeout_s=3.0)
        if observed == 26:
            return self._protection_error(
                requested_action, [], observed, safety, command_sent=True)
        confirmed = observed == 2
        return {
            "state": "completed" if confirmed else "accepted",
            "command_sent": True,
            "requested_action": requested_action,
            "confirmed": confirmed,
            "workmode": observed,
            "workmode_name": _WORKMODE_NAMES.get(observed, "unknown"),
            "preparation_steps": [],
            "safety_requirements": safety,
            "message": (
                "The active action was exited and walking mode was confirmed."
                if confirmed else
                "The WALK exit command was sent, but walking mode was not observed within 3 seconds."
            ),
        }

    def _prepare_workmode(self, target_mode: int, requested_action: str) -> dict:
        """Automatically reach ready(1) or walking(2) through documented steps."""
        self._move_stop_event.set()
        if self._move_thread and self._move_thread.is_alive():
            self._move_thread.join(timeout=1)

        steps = []
        mode = int(self._high_ctrl.get_mode())
        if mode == 26:
            return self._protection_error(requested_action, steps, mode)

        stable_modes = {0, 1, 2, 30}
        if mode not in stable_modes:
            mode = self._wait_for_workmode(stable_modes, timeout_s=15.0)
            steps.append({
                "step": "wait_for_current_action",
                "result_workmode": mode,
                "result_workmode_name": _WORKMODE_NAMES.get(mode, "unknown"),
            })
            if mode == 26:
                return self._protection_error(requested_action, steps, mode)
            if mode not in stable_modes:
                return self._preparation_error(
                    requested_action, steps, mode,
                    "The current robot action has not finished. No further mode transition was sent; wait for the action to finish and try again.")

        if mode == 30:
            mode = self._run_preparation_step("enable", "START", {0}, steps)
            if mode == 26:
                return self._protection_error(requested_action, steps, mode)
            if mode != 0:
                return self._preparation_error(requested_action, steps, mode, "The robot did not enter enabled mode.")

        if target_mode == 1 and mode == 2:
            mode = self._run_preparation_step("prepare", "SWITCH", {1}, steps)
        elif mode == 0:
            mode = self._run_preparation_step("prepare", "SWITCH", {1}, steps)

        if mode == 26:
            return self._protection_error(requested_action, steps, mode)
        if target_mode == 1:
            if mode != 1:
                return self._preparation_error(requested_action, steps, mode, "The robot did not enter the ready mode required for standing up.")
            return {"state": "completed", "steps": steps, "workmode": mode}

        if mode == 1:
            mode = self._run_preparation_step("enter_walking", "WALK", {2}, steps)
        if mode == 26:
            return self._protection_error(requested_action, steps, mode)
        if mode != 2:
            return self._preparation_error(requested_action, steps, mode, "The robot did not enter the walking mode required for this action.")
        return {"state": "completed", "steps": steps, "workmode": mode}

    def _run_preparation_step(self, step: str, command_name: str,
                              expected_modes: set[int], steps: list[dict]) -> int:
        # START is a toggle, so it must never be retried.  On the tested Bumi,
        # however, the disabled state reached after STANDTOFALL can ignore a
        # START edge until it has first received a sustained neutral stream.
        # Prime that transition every time instead of only once per process.
        neutral_preroll_s = (
            self._control_cold_preroll_s if command_name == "START" else None)
        observed = self._send_edge_and_wait(
            _get_control_cmd(command_name), expected_modes | {26}, timeout_s=3.0,
            preroll_override_s=neutral_preroll_s)
        step_result = {
            "step": step,
            "command": command_name,
            "expected_workmodes": sorted(expected_modes),
            "observed_workmode": observed,
            "observed_workmode_name": _WORKMODE_NAMES.get(observed, "unknown"),
            "confirmed": observed in expected_modes,
        }
        if neutral_preroll_s is not None:
            step_result["neutral_preroll_s"] = neutral_preroll_s
        steps.append(step_result)
        return observed

    def _trigger_user_action(self, requested_action: str, command_name: str,
                             expected_modes: set[int], preparation_steps: list[dict],
                             safety_requirements: str, index: int = 0,
                             recording_id: int | None = None) -> dict:
        observed = self._send_edge_and_wait(
            _get_control_cmd(command_name), expected_modes | {26}, index=index, timeout_s=3.0)
        if observed == 26:
            return self._protection_error(
                requested_action, preparation_steps, observed, safety_requirements,
                command_sent=True)
        confirmed = observed in expected_modes
        result = {
            "state": "running" if confirmed else "accepted",
            "command_sent": True,
            "requested_action": requested_action,
            "confirmed_started": confirmed,
            "workmode": observed,
            "workmode_name": _WORKMODE_NAMES.get(observed, "unknown"),
            "preparation_steps": preparation_steps,
            "safety_requirements": safety_requirements,
            "pose_verification": "The SDK exposes only workmode and cannot verify the robot's physical pose. The user must check the pose and surrounding area.",
            "message": (
                "The target action mode was observed and the action is running. This response does not mean the physical action has completed."
                if confirmed else
                "The command was sent, but the target action mode was not observed within 3 seconds. Check the robot and motion_state."
            ),
        }
        if recording_id is not None:
            result["recording_id"] = recording_id
        return result

    @staticmethod
    def _preparation_error(requested_action: str, steps: list[dict],
                           mode: int, message: str) -> dict:
        return {
            "state": "error", "command_sent": bool(steps),
            "requested_action": requested_action,
            "current_workmode": mode,
            "current_workmode_name": _WORKMODE_NAMES.get(mode, "unknown"),
            "preparation_steps": steps,
            "error": message,
            "message": "The requested action was not sent. Check the robot pose, floor, and surrounding clearance before trying again.",
        }

    @staticmethod
    def _protection_error(requested_action: str, steps: list[dict], mode: int,
                          safety_requirements: str | None = None,
                          command_sent: bool = False) -> dict:
        result = {
            "state": "error",
            "command_sent": command_sent or bool(steps),
            "requested_action": requested_action,
            "current_workmode": mode,
            "current_workmode_name": "protection",
            "protection": True,
            "preparation_steps": steps,
            "error": "The robot has entered protection mode and the action cannot continue.",
            "recovery": "Stop operating and restart the robot. Before restarting, place it face-up on a flat, non-slip floor with its limbs naturally positioned and no objects under its feet. Clear at least a 3 m x 3 m area, then run stand_up.",
        }
        if safety_requirements:
            result["safety_requirements"] = safety_requirements
        return result

    @staticmethod
    def _safety_requirements(action: str) -> str:
        if action == "stand_up":
            return "Use only when the robot is lying face-up with its limbs naturally positioned, legs straight, no objects under its feet, on a flat non-slip floor, with at least a clear 3 m x 3 m area."
        if action == "lie_prone":
            return "Use only when the robot is standing normally and steadily on a flat non-slip floor, with at least a clear 3 m x 3 m area."
        if action in {"dance_1", "dance_2", "dance_3", "play_recording"}:
            return "Use only when the robot is standing normally and steadily with both feet on a flat non-slip floor, with at least a clear 3 m x 3 m area."
        if action == "start_recording":
            return "Make sure the robot is standing steadily on a flat non-slip floor under supervision. Guide joints slowly; never force, twist quickly, or exceed mechanical limits."
        if action == "finish_and_save_recording":
            return "Use only after start_recording has been called and action guidance is finished. Do not move the robot while the recording is being saved."
        if action == "reset":
            return "Keep the robot standing with both feet on a flat non-slip floor and keep people and obstacles outside its movement range while returning to walking mode."
        return "Use only when the robot is standing normally and steadily with both feet on a flat non-slip floor, with no people or obstacles in its movement range."

    def _wait_for_workmode(self, expected_modes: set[int], timeout_s: float) -> int:
        deadline = time.monotonic() + timeout_s
        observed = int(self._high_ctrl.get_mode())
        while observed not in expected_modes and observed != 26 and time.monotonic() < deadline:
            time.sleep(0.05)
            observed = int(self._high_ctrl.get_mode())
        return observed

    def _send_edge_and_wait(self, cmd_enum, expected_modes: set[int],
                            index: int = 0, timeout_s: float = 2.0,
                            preroll_override_s: float | None = None) -> int:
        """Prime DDS, send one action edge, then maintain neutral control frames."""
        with self._action_lock:
            default_cmd = _get_default_cmd()

            # The vendor examples keep publishing at 100 Hz. Use a longer
            # DEFAULT-only pre-roll for a new DDS writer and whenever the caller
            # explicitly requests it for a state transition such as START.
            # Never retry the action itself: START is a toggle and a delayed
            # duplicate could immediately disable the robot again.
            if preroll_override_s is not None:
                preroll_s = preroll_override_s
            else:
                preroll_s = (
                    self._control_cold_preroll_s
                    if not self._control_channel_warmed
                    else self._control_preroll_s
                )
            if preroll_override_s is not None:
                print(
                    f"[loco] priming control transition with DEFAULT for "
                    f"{preroll_s:.1f}s",
                    flush=True,
                )
            elif preroll_s == self._control_cold_preroll_s:
                print(
                    f"[loco] priming new control channel with DEFAULT for {preroll_s:.1f}s",
                    flush=True,
                )

            # A newly started DDS writer can lose its first control sample on
            # some Bumi firmware. Neutral frames warm the command path without
            # changing workmode or requesting motion.
            preroll_deadline = time.monotonic() + preroll_s
            while time.monotonic() < preroll_deadline:
                self._publish_cmd(0, 0, 0, default_cmd, 0)
                time.sleep(self._control_period)
            self._control_channel_warmed = True

            # Actions such as START and PLAYTEACH are edge-triggered. Never
            # retry them automatically: START is a toggle and repeated actions
            # can reverse a transition or restart an active policy.
            self._publish_cmd(0, 0, 0, cmd_enum, index)
            time.sleep(self._control_period)

            deadline = time.monotonic() + timeout_s
            observed = int(self._high_ctrl.get_mode())
            while observed not in expected_modes and time.monotonic() < deadline:
                self._publish_cmd(0, 0, 0, default_cmd, 0)
                time.sleep(self._control_period)
                observed = int(self._high_ctrl.get_mode())
            return observed

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
                 activity_velocity_threshold: float):
        super().__init__("bumi_motion_state")
        self._high_ctrl = high_ctrl
        self._topic = f"/{namespace}/motion/state"
        self._pub = self.create_publisher(String, self._topic, 10)
        self._interval_s = interval_s
        self._activity_velocity_threshold = activity_velocity_threshold
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

    def _loop(self):
        while self._running:
            try:
                payload = self._read_once()
                msg = String()
                msg.data = json.dumps(payload, ensure_ascii=False)
                self._pub.publish(msg)
                time.sleep(self._interval_s)
            except Exception as exc:
                error = {
                    "state": "error", "fresh": False,
                    "reason": str(exc),
                }
                msg = String()
                msg.data = json.dumps(error, ensure_ascii=False)
                self._pub.publish(msg)
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
            "activity_description": (
                "at least one joint velocity reached the configured activity threshold"
                if moving else
                "all joint velocities are below the configured activity threshold"
            ),
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
            "joint_states": joint_states,
        }


class MotionStatePlugin:
    PREFIX = "motion_state"

    def __init__(self, plugin_config: dict, namespace: str, executor, high_ctrl):
        interval = _finite_number(plugin_config.get("poll_interval_s", 0.5), "poll_interval_s")
        if not 0.02 <= interval <= 2.0:
            raise ValueError("poll_interval_s must be in [0.02, 2.0]")
        activity_threshold = _finite_number(
            plugin_config.get("activity_velocity_threshold", 0.15),
            "activity_velocity_threshold",
        )
        if not 0.001 <= activity_threshold <= 10.0:
            raise ValueError("activity_velocity_threshold must be in [0.001, 10.0]")
        self._node = _MotionStateNode(namespace, high_ctrl, interval, activity_threshold)
        executor.add_node(self._node)

    def get_tool(self) -> dict:
        return {
            "name": "motion_state", "type": "sensor", "multiInstance": False,
            "description": "Bumi 整机运动状态：持续输出工作模式、保护状态、运动判断、IMU 姿态与动态、关节运动统计、已确认的电机故障，以及全部 21 个关节的位置、速度、力矩、温度和原始错误值。不包含电池信息，也不控制机器人。",
            "inputSchema": {"type": "object", "properties": {}},
            "topic_out": [{"topic": self._node.topic, "format": "data/json"}],
        }

    def start(self):
        self._node.start_polling()

    def stop(self):
        self._node.stop_polling()

    def dispatch(self, action: str, args: dict) -> dict | None:
        if action == "start":
            return {"state": "running"}
        if action == "stop":
            return {"state": "idle"}
        return None
