#!/usr/bin/env python3
"""
drivers/noetix/bumi/device.py — Noetix Bumi-EDU 设备插件实现。
插件列表：
  - StatePlugin: joints (21-DOF skeleton), imu, battery, model (URDF resource)
  - LocoPlugin: locomotion, stand-up/prone storage, semantic actions and action recording
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

_POSTURE_ACTIONS = {
    "stand_up": ("FALLTOSTAND", {27}),
    # disabled(30) is the terminal state, not proof that STANDTOFALL started.
    "lie_prone": ("STANDTOFALL", {28}),
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
_ACTION_MOTION_SAMPLE_INTERVAL_S = 0.05
_LIE_PRONE_MOTION_START_TIMEOUT_S = 4.0
_LIE_PRONE_MIN_JOINT_DISPLACEMENT_RAD = 0.15
_LIE_PRONE_MIN_MOVED_JOINT_COUNT = 3
_WIPE_TEARS_MOTION_START_TIMEOUT_S = 3.0
_WIPE_TEARS_MIN_ARM_DISPLACEMENT_RAD = 0.08
_ARM_JOINT_INDICES = tuple(range(4)) + tuple(range(10, 14))
_PLAYBACK_MOVING_THRESHOLD = 0.15
_PLAYBACK_STATIONARY_THRESHOLD = 0.15
_PLAYBACK_STATIONARY_CONFIRM_S = 3.0
_PLAYBACK_MOTION_START_TIMEOUT_S = 10.0
_PLAYBACK_MAX_MONITOR_S = 120.0
_PLAYBACK_VELOCITY_WINDOW_SIZE = 5
_PLAYBACK_MOTION_RESET_THRESHOLD = 0.20
_PLAYBACK_MIN_JOINT_DISPLACEMENT_RAD = 0.08
_WALK_EXIT_MAX_ATTEMPTS = 2
_WALK_EXIT_CONFIRM_TIMEOUT_S = 3.0

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
_STAND_ENABLE_MODE_TIMEOUT_S = 10.0
_STAND_ENABLE_MAX_ATTEMPTS = 2
_STAND_ENABLE_SETTLE_TIMEOUT_S = 12.0
_STAND_ENABLE_MIN_DWELL_S = 3.0
_STAND_ENABLE_STATIONARY_CONFIRM_S = 2.0
_STAND_ENABLE_SAMPLE_INTERVAL_S = 0.05
_STAND_ENABLE_VELOCITY_WINDOW_SIZE = 5

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


def _acp_notify(action_id: str, status: str, result: dict,
                tool: str) -> None:
    """Report an asynchronous physical-action terminal state to Agent Core."""
    import json as _json
    import os as _os
    import ssl as _ssl
    import sys as _sys
    import urllib.request as _urllib

    agent_core_url = _os.environ.get(
        "AGENT_CORE_URL", "https://localhost:15678")
    context = _ssl.create_default_context()
    context.check_hostname = False
    context.verify_mode = _ssl.CERT_NONE
    payload = _json.dumps({
        "action_id": action_id,
        "status": status,
        "result": result,
        "tool": tool,
        "ts": time.time(),
    }).encode()
    try:
        request = _urllib.Request(
            f"{agent_core_url}/api/acp/complete",
            data=payload,
            headers={"Content-Type": "application/json"},
            method="POST",
        )
        with _urllib.urlopen(request, timeout=5, context=context):
            pass
        print(
            f"[ACP] {status}: action_id={action_id} tool={tool}",
            flush=True,
        )
    except Exception as exc:
        print(
            f"[ACP] callback failed for {action_id}: {exc}",
            file=_sys.stderr,
            flush=True,
        )


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


# ── LocoPlugin (actuator, multi-tool) ───────────────────────────────

class LocoPlugin:
    PREFIX = "loco"
    _MOVE_MIN_DURATION_S = 1.0
    _MOVE_DEFAULT_DURATION_S = 2.0
    _MOVE_MAX_DURATION_S = 10.0
    _MOVE_MIN_NONZERO_BY_AXIS = {
        "vx": 0.5,
        "vy": 0.6,
        "vyaw": 0.5,
    }
    _MOVE_CONFIRM_TIMEOUT_S = 2.0
    # Stationary feedback on the tested robot can peak around 0.16 rad/s, while
    # a real low-amplitude gait can start below the former fixed 0.30 rad/s
    # threshold. Keep an absolute noise floor and require a rise over the
    # command-specific stationary baseline instead of using 0.30 unconditionally.
    _MOVE_CONFIRM_MIN_JOINT_VELOCITY = 0.22
    _MOVE_CONFIRM_BASELINE_MARGIN = 0.10

    def __init__(self, plugin_config: dict, namespace: str, executor, high_ctrl):
        self._high_ctrl = high_ctrl
        self._namespace = namespace
        self._lock = threading.Lock()
        self._action_lock = threading.Lock()
        # Serializes the complete bounded-move session setup, including ACP
        # binding.  The per-frame publish lock above is intentionally separate.
        self._move_session_lock = threading.RLock()
        self._last_cmd_time: float = 0.0
        self._move_thread: threading.Thread | None = None
        self._move_observation: dict | None = None
        self._move_stop_event = threading.Event()
        self._control_period = 0.01       # 100 Hz, matching the vendor demo
        self._control_preroll_s = 0.3     # refresh an already active DDS writer path
        self._control_cold_preroll_s = 3.0
        self._control_channel_warmed = False
        self._auto_exit_lock = threading.Lock()
        self._auto_exit_timers: dict[str, threading.Timer] = {}
        self._auto_exit_action_ids: dict[str, str] = {}
        self._playback_monitor_stop: threading.Event | None = None
        self._playback_monitor_thread: threading.Thread | None = None
        self._playback_action_id: str | None = None
        self._acp_lock = threading.Lock()
        self._active_acp: dict[str, dict] = {}
        self._lifecycle_stop_event = threading.Event()

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
            "description": (
                "Move Bumi with bounded HighController walking commands or stop an active "
                "move. The card uses only the vendor HighController; it does not initialize "
                "LowController. move is accepted only when workmode=2 (walking). Every move "
                "sends a fresh WALK trigger for this bounded command, then follows the vendor "
                "example by continuously sending DEFAULT with the requested velocity. Every "
                "move automatically sends zero velocity after at "
                "most 10 seconds. Values are normalized SDK commands, not metres per second. "
                "Before moving, confirm that Bumi is standing steadily on a flat non-slip floor "
                "and that its path is clear."
            ),
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {
                        "type": "string",
                        "enum": ["move", "stop_move"],
                        "description": (
                            "move=send a time-limited walking command; "
                            "stop_move=immediately send zero walking velocity and requires no "
                            "other parameters."
                        ),
                    },
                    "vx": {
                        "type": "number",
                        "default": 0.0,
                        "description": "default: 0, [-1.0,-0.5]U[0.5,1.0]",
                        "minimum": -1, "maximum": 1,
                    },
                    "vy": {
                        "type": "number",
                        "default": 0.0,
                        "description": "default: 0, [-1.0,-0.6]U[0.6,1.0]",
                        "minimum": -1, "maximum": 1,
                    },
                    "vyaw": {
                        "type": "number",
                        "default": 0.0,
                        "description": "default: 0, [-1.0,-0.5]U[0.5,1.0]",
                        "minimum": -1, "maximum": 1,
                    },
                    "duration": {
                        "type": "number",
                        "default": self._MOVE_DEFAULT_DURATION_S,
                        "description": "default: 2.0, [1.0,10.0] seconds",
                        "minimum": self._MOVE_MIN_DURATION_S,
                        "maximum": self._MOVE_MAX_DURATION_S,
                    },
                },
                "required": ["action"],
                "x-completion": {
                    "actions": ["move"],
                    "timeout": 20,
                },
                "x-action-params": {
                    "move": {
                        "params": ["vx", "vy", "vyaw", "duration"],
                        "description": (
                            "Move with normalized HighController commands for 1-10 seconds. "
                            "At least one velocity must be non-zero and the robot must already "
                            "be upright, stable and in walking mode (motion_state must report "
                            "workmode.code=2). If it is not, the card returns an error without "
                            "sending WALK or velocity commands. Non-zero vx and vy together "
                            "request diagonal translation. For the first low-risk test, use "
                            "forward=0.5, lateral=0, turn=0 and duration=2 in a clear area."
                        ),
                    },
                    "stop_move": {
                        "params": [],
                        "description": (
                            "Stop the active loco command by sending zero forward, lateral and "
                            "turning velocity. No additional parameters are used."
                        ),
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
            "description": "让 Bumi 从仰面平躺自主起身，或从正常站立姿态趴下收纳。stand_up 会先用 IMU 和 21 个关节状态检查仰面方向、静止状态及四肢姿态，不通过时不发送任何控制命令；通过后在必要时自动使能，并直接执行自主起身，不会进入仅适用于人工扶站的准备模式。lie_prone 只有在进入动作模式且关节位移确认身体实际开始运动后才返回 running；未启动时会返回电池 SOC 与 alarm 诊断。传感器无法检查地面、脚下异物和周围空间，用户仍须完成 action 描述中的现场安全检查。",
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
                "x-completion": {
                    "actions": ["stand_up", "lie_prone"],
                    "timeout": 120,
                },
                "x-action-params": {
                    "stand_up": {
                        "params": [],
                        "description": "仅从 disabled/enabled 状态自主起身。卡片先自动检查机器人是否静止、躯干是否仰面及 21 个关节是否接近安全平躺姿态；检查失败时不会使能或起身。检查通过后直接调用 FALLTOSTAND，不会调用用于人工扶站的 SWITCH/ready。用户仍须确认平坦防滑地面、脚底无异物且周围 3m×3m 安全。",
                    },
                    "lie_prone": {
                        "params": [],
                        "description": "仅从 walking 状态趴下收纳。调用前必须由用户确认机器人稳定站立且周围 3m×3m 安全；其他工作模式不会发送动作命令。进入动作模式后还会检查全身关节是否产生实际位移，未启动时返回电池诊断。",
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
                "x-completion": {
                    "actions": ["wipe_tears"],
                    "timeout": 30,
                },
                "x-action-params": {
                    name: {"params": [], "description": description}
                    for name, description in {
                        "wave": "挥手。确认机器人稳定站立，手臂摆动范围内无人和障碍物。",
                        "handshake": "握手。确认机器人稳定站立，人员不要拉扯机器人手臂。",
                        "cheer": "欢呼。确认机器人稳定站立，肢体活动范围内无人和障碍物。",
                        "dance_1": "执行舞蹈 1。机器人属于盲舞，至少留出 3m×3m 平坦防滑空间。",
                        "dance_2": "执行舞蹈 2。机器人属于盲舞，至少留出 3m×3m 平坦防滑空间。",
                        "dance_3": "执行舞蹈 3。机器人属于盲舞，至少留出 3m×3m 平坦防滑空间。",
                        "wipe_tears": "执行擦眼泪动作。检测到手臂实际运动后开始 5 秒计时并自动返回 walking；未启动时返回电池诊断。确认机器人稳定站立且手臂周围无障碍物。",
                        "reset": "结束当前语义动作并返回 workmode=2（walking），用于动作后复位。",
                    }.items()
                } | {
                    "wipe_tears": {
                        "params": [],
                        "description": "执行擦眼泪动作。手臂关节位移确认动作启动后固定 5 秒自动返回 walking；未启动时返回电池 SOC 与 alarm，无需填写时长或调用 reset。",
                    },
                },
            },
            "topic_out": [],
        }

    def _action_recording_tool(self) -> dict:
        return {
            "name": "action_recording", "type": "actuator", "multiInstance": False,
            "description": "录制、结束并保存或播放 Bumi 示教动作。start_recording 和 play_recording 会自动进入所需行走模式；finish_and_save_recording 只能在已开始录制后使用；play_recording 会用关节位移和滚动速度中位数判断播放结束，并通过最多两次受保护的 WALK 尝试自动返回 walking。",
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
                "x-completion": {
                    "actions": ["play_recording"],
                    "timeout": 150,
                },
                "x-action-params": {
                    "start_recording": {"params": [], "description": "自动准备模式后开始示教录制。确认机器人稳定站立；缓慢引导关节，禁止强推至机械限位。"},
                    "finish_and_save_recording": {"params": ["recording_id"], "description": "结束当前示教并保存到 recording_id。若尚未开始录制，则不会发送命令。"},
                    "play_recording": {"params": ["recording_id"], "description": "自动准备并播放 recording_id；卡片根据 workmode、21 个关节位移和滚动速度中位数推断动作完成，随后通过带确认的 WALK 退出自动返回 walking，无需填写时长或手动停止。"},
                },
            },
            "topic_out": [],
        }

    def start(self) -> None:
        self._lifecycle_stop_event.clear()

    def stop(self) -> None:
        self._lifecycle_stop_event.set()
        self._cancel_all_auto_exits()
        self._cancel_playback_monitor()
        self._stop_move()
        self._cancel_all_acp("Bumi actuator plugin stopped")

    def dispatch(self, action: str, args: dict) -> dict | None:
        if action == "start":
            self._lifecycle_stop_event.clear()
            return {"state": "ready"}
        if action == "stop":
            self._lifecycle_stop_event.set()
            self._cancel_all_auto_exits()
            self._cancel_playback_monitor()
            self._stop_move()
            self._cancel_all_acp("Bumi actuator plugin stopped")
            return {"state": "idle"}

        tool_name = args.pop('_tool_name', '')

        if tool_name == "loco" and action == "move":
            with self._move_session_lock:
                result = self._do_move(args)
                if result.get("state") == "running":
                    action_id, _ = self._register_acp("loco", action)
                    result["action_id"] = action_id
                    move_thread = self._move_thread
                    move_observation = self._move_observation
                    threading.Thread(
                        target=self._complete_move_acp,
                        args=(
                            action_id, move_thread, move_observation,
                            dict(result)),
                        daemon=True,
                        name=f"bumi_move_acp_{action_id[-8:]}",
                    ).start()
                return result
        if tool_name == "loco" and action == "stop_move":
            return self._stop_move()
        if tool_name == "stand_up_lie_prone" and action in _POSTURE_ACTIONS:
            return self._start_posture_acp(action, args)
        if tool_name == "semantic_action" and action in _PRESET_ACTIONS:
            result = self._do_preset_action(action, args)
            if (action == "wipe_tears" and result.get("state") == "running"
                    and result.get("confirmed_started")):
                action_id, _ = self._register_acp(
                    "semantic_action", action)
                result["action_id"] = action_id
                self._schedule_auto_walk_exit(
                    "wipe_tears", 33, _TEAR_AUTO_EXIT_S,
                    self._safety_requirements(action), action_id=action_id)
            return result
        if tool_name == "action_recording" and action in _TEACHING_ACTIONS:
            result = self._do_teaching_action(action, args)
            if (action == "play_recording" and result.get("state") == "running"
                    and result.get("confirmed_started")):
                action_id, _ = self._register_acp(
                    "action_recording", action)
                result["action_id"] = action_id
                self._schedule_playback_completion_monitor(
                    self._safety_requirements(action), action_id=action_id)
            return result
        return None

    def _register_acp(self, tool: str, action: str,
                      action_id: str | None = None) -> tuple[str, threading.Event]:
        action_id = action_id or f"bumi_{action}_{uuid4().hex[:8]}"
        cancel_event = threading.Event()
        with self._acp_lock:
            self._active_acp[action_id] = {
                "tool": tool,
                "action": action,
                "cancel_event": cancel_event,
            }
        return action_id, cancel_event

    def _finish_acp(self, action_id: str, status: str, result: dict) -> bool:
        with self._acp_lock:
            active = self._active_acp.pop(action_id, None)
        if active is None:
            return False
        _acp_notify(action_id, status, result, active["tool"])
        return True

    def _cancel_all_acp(self, reason: str) -> None:
        with self._acp_lock:
            active_items = list(self._active_acp.items())
            self._active_acp.clear()
        for action_id, active in active_items:
            active["cancel_event"].set()
            _acp_notify(
                action_id,
                "cancelled",
                {"action": active["action"], "reason": reason},
                active["tool"],
            )

    def _complete_move_acp(self, action_id: str,
                           move_thread: threading.Thread | None,
                           observation: dict | None,
                           start_result: dict) -> None:
        if move_thread is not None:
            move_thread.join()
        # The MCP response must reach Agent Core before a very short move can
        # complete and post its terminal callback.
        time.sleep(0.1)
        observation = dict(observation or {})
        reason = observation.get("stop_reason", "unknown")
        if reason == "duration_elapsed":
            status = "completed"
        elif reason == "stop_requested":
            status = "cancelled"
        else:
            status = "error"
        terminal_result = {
            **start_result,
            "state": status,
            "reason": reason,
            **observation,
        }
        self._finish_acp(action_id, status, terminal_result)

    def _start_posture_acp(self, action: str, args: dict) -> dict:
        safety = self._safety_requirements(action)
        current_mode = int(self._high_ctrl.get_mode())
        mode_error = self._posture_workmode_error(
            action, current_mode, safety)
        if mode_error is not None:
            return mode_error

        pose_check = None
        if action == "stand_up":
            pose_check = self._check_face_up_stand_pose()
            if not pose_check["safe_to_stand"]:
                return {
                    "state": "error",
                    "command_sent": False,
                    "requested_action": action,
                    "current_workmode": current_mode,
                    "current_workmode_name": _WORKMODE_NAMES.get(
                        current_mode, "unknown"),
                    "error": (
                        "Automatic pose check failed. No enable or stand-up "
                        "command was sent."
                    ),
                    "pose_check": pose_check,
                    "message": (
                        "Place the robot face-up with both legs straight and all limbs "
                        "naturally positioned, wait until it is still, then try again."
                    ),
                    "safety_requirements": safety,
                }

        action_id, cancel_event = self._register_acp(
            "stand_up_lie_prone", action)
        threading.Thread(
            target=self._run_posture_acp,
            args=(action_id, cancel_event, action, dict(args)),
            daemon=True,
            name=f"bumi_posture_acp_{action_id[-8:]}",
        ).start()
        response = {
            "state": "accepted",
            "action_id": action_id,
            "requested_action": action,
            "safety_requirements": safety,
            "message": (
                "The workmode and available pose checks passed. Preparation, action startup "
                "and completion monitoring will continue asynchronously. Agent Core will "
                "keep the actuator completion barrier until the action completes, fails or "
                "is cancelled."
            ),
        }
        if action == "stand_up":
            response["user_guidance"] = (
                "Please wait patiently for about 20 seconds while Bumi enables its motors, "
                "settles its joints, and starts standing up. Do not send another stand_up "
                "request while this action is still running. If the robot still does not "
                "respond after this action reports an error, check its face-up pose, battery, "
                "workmode, and surrounding clearance before retrying."
            )
        return response

    def _run_posture_acp(self, action_id: str, cancel_event: threading.Event,
                         action: str, args: dict) -> None:
        # Leave enough time for Agent Core to register the action_id returned by
        # the MCP response before an immediate validation failure is reported.
        if cancel_event.wait(0.25):
            return
        try:
            result = self._do_posture_action(action, args)
            print(
                "[posture] startup_result "
                + json.dumps(result, ensure_ascii=True, separators=(",", ":")),
                flush=True,
            )
            if cancel_event.is_set():
                return
            if (result.get("state") == "error"
                    or not result.get("confirmed_started")):
                self._finish_acp(action_id, "error", result)
                return

            active_mode = 27 if action == "stand_up" else 28
            deadline = time.monotonic() + 90.0
            while not cancel_event.wait(0.1):
                mode = int(self._high_ctrl.get_mode())
                if mode == 26:
                    result.update({
                        "state": "error",
                        "final_workmode": mode,
                        "final_workmode_name": "protection",
                        "error": "The robot entered protection mode before the posture action completed.",
                    })
                    self._finish_acp(action_id, "error", result)
                    return
                if mode != active_mode:
                    result.update({
                        "state": "completed",
                        "final_workmode": mode,
                        "final_workmode_name": _WORKMODE_NAMES.get(mode, "unknown"),
                        "completion_detection": "workmode_left_posture_action",
                    })
                    self._finish_acp(action_id, "completed", result)
                    return
                if time.monotonic() >= deadline:
                    result.update({
                        "state": "error",
                        "error": "The posture action did not leave its active workmode within 90 seconds.",
                    })
                    self._finish_acp(action_id, "error", result)
                    return
        except Exception as exc:
            self._finish_acp(action_id, "error", {
                "requested_action": action,
                "error": str(exc),
            })

    def _publish_cmd(self, x: float, y: float, z: float, action_cmd, index: int = 0):
        """Send command with rate limiting (≥2ms between calls). action_cmd is ControlCmd enum."""
        with self._lock:
            now = time.monotonic()
            elapsed = now - self._last_cmd_time
            if elapsed < 0.002:
                time.sleep(0.002 - elapsed)
            self._high_ctrl.publish_cmd(x, y, z, action_cmd, index)
            self._last_cmd_time = time.monotonic()

    def _sample_max_joint_velocity(self) -> float:
        joints = self._high_ctrl.get_joint_state()
        if len(joints) != len(_BUMI_JOINT_NAMES):
            raise RuntimeError(
                f"HighController returned {len(joints)} joints; "
                f"expected {len(_BUMI_JOINT_NAMES)}")
        velocities = [abs(float(joint.vel)) for joint in joints]
        if not all(math.isfinite(value) for value in velocities):
            raise RuntimeError("HighController returned a non-finite joint velocity")
        return max(velocities)

    def _read_battery_status(self) -> dict:
        """Return only the battery percentage and documented BMS alarm value."""
        try:
            bms = self._high_ctrl.get_robot_bms_data()
            return {
                "soc_percent": int(bms.battery_soc),
                "alarm": int(bms.battery_alarm),
            }
        except Exception:
            return {
                "soc_percent": None,
                "alarm": None,
            }

    def _attach_battery_failure_diagnostic(self, result: dict) -> dict:
        battery = self._read_battery_status()
        result["battery_status"] = battery
        alarm = battery.get("alarm")
        if alarm:
            result["battery_alarm_present"] = True
            result["low_battery_may_have_prevented_action"] = True
            result["battery_diagnosis"] = (
                "The BMS reports a non-zero battery alarm. Low charge or another battery "
                "condition may have prevented the firmware from starting the physical action. "
                "Charge and inspect Bumi before retrying. The SDK does not document the alarm "
                "bit meanings, so this driver cannot identify the exact battery fault."
            )
        elif alarm == 0:
            result["battery_alarm_present"] = False
            result["low_battery_may_have_prevented_action"] = None
            result["battery_diagnosis"] = (
                "The BMS did not report a battery alarm when the action failure was checked. "
                "Review the returned SOC value: the SDK does not document a low-SOC action "
                "threshold, so this driver cannot confirm or exclude low charge as the cause."
            )
        else:
            result["battery_alarm_present"] = None
            result["low_battery_may_have_prevented_action"] = None
            result["battery_diagnosis"] = (
                "Battery state could not be read, so low charge could not be evaluated."
            )
        return result

    def _capture_joint_positions(self, indices: tuple[int, ...]) -> list[float]:
        joints = self._high_ctrl.get_joint_state()
        if len(joints) != len(_BUMI_JOINT_NAMES):
            raise RuntimeError(
                f"HighController returned {len(joints)} joints; "
                f"expected {len(_BUMI_JOINT_NAMES)}")
        positions = [float(joints[index].pos) for index in indices]
        if not all(math.isfinite(value) for value in positions):
            raise RuntimeError("HighController returned a non-finite joint position")
        return positions

    def _confirm_joint_displacement(
            self, baseline: list[float], indices: tuple[int, ...],
            minimum_displacement_rad: float, timeout_s: float,
            active_modes: set[int], minimum_moved_joint_count: int = 1) -> dict:
        """Confirm physical action start independently from workmode feedback."""
        deadline = time.monotonic() + timeout_s
        peak_displacement = 0.0
        most_moved_index = indices[0]
        maximum_moved_joint_count = 0
        observed_mode = int(self._high_ctrl.get_mode())
        sample_count = 0
        error = None

        while time.monotonic() < deadline:
            observed_mode = int(self._high_ctrl.get_mode())
            if observed_mode == 26 or observed_mode not in active_modes:
                break
            try:
                current = self._capture_joint_positions(indices)
            except Exception as exc:
                error = str(exc)
                break
            sample_count += 1
            moved_joint_count = 0
            for list_index, (start, now) in enumerate(zip(baseline, current)):
                displacement = abs(now - start)
                if displacement >= minimum_displacement_rad:
                    moved_joint_count += 1
                if displacement > peak_displacement:
                    peak_displacement = displacement
                    most_moved_index = indices[list_index]
            maximum_moved_joint_count = max(
                maximum_moved_joint_count, moved_joint_count)
            if moved_joint_count >= minimum_moved_joint_count:
                return {
                    "confirmed": True,
                    "sample_count": sample_count,
                    "observed_workmode": observed_mode,
                    "observed_workmode_name": _WORKMODE_NAMES.get(
                        observed_mode, "unknown"),
                    "peak_joint_displacement_rad": round(peak_displacement, 5),
                    "most_moved_joint": _BUMI_JOINT_NAMES[most_moved_index],
                    "minimum_required_displacement_rad": minimum_displacement_rad,
                    "moved_joint_count": moved_joint_count,
                    "minimum_required_moved_joint_count": (
                        minimum_moved_joint_count),
                }
            time.sleep(_ACTION_MOTION_SAMPLE_INTERVAL_S)

        return {
            "confirmed": False,
            "sample_count": sample_count,
            "observed_workmode": observed_mode,
            "observed_workmode_name": _WORKMODE_NAMES.get(observed_mode, "unknown"),
            "peak_joint_displacement_rad": round(peak_displacement, 5),
            "most_moved_joint": _BUMI_JOINT_NAMES[most_moved_index],
            "minimum_required_displacement_rad": minimum_displacement_rad,
            "moved_joint_count": maximum_moved_joint_count,
            "minimum_required_moved_joint_count": minimum_moved_joint_count,
            "error": error,
        }

    def _do_move(self, args: dict) -> dict:
        values = {}
        limits = {
            "vx": (-1.0, 1.0, 0.0),
            "vy": (-1.0, 1.0, 0.0),
            "vyaw": (-1.0, 1.0, 0.0),
            "duration": (
                self._MOVE_MIN_DURATION_S,
                self._MOVE_MAX_DURATION_S,
                self._MOVE_DEFAULT_DURATION_S,
            ),
        }
        for field, (minimum, maximum, default) in limits.items():
            value = args.get(field, default)
            if isinstance(value, bool) or not isinstance(value, (int, float)):
                return {
                    "state": "error", "command_sent": False,
                    "error": f"{field} must be a number from {minimum} to {maximum}",
                }
            value = float(value)
            if not math.isfinite(value) or value < minimum or value > maximum:
                return {
                    "state": "error", "command_sent": False,
                    "error": f"{field} must be a finite number from {minimum} to {maximum}",
                    "received": args.get(field),
                }
            values[field] = value

        vx, vy, vyaw = values["vx"], values["vy"], values["vyaw"]
        duration = values["duration"]
        for field, value in (("vx", vx), ("vy", vy), ("vyaw", vyaw)):
            minimum_nonzero = self._MOVE_MIN_NONZERO_BY_AXIS[field]
            if value != 0.0 and abs(value) < minimum_nonzero:
                return {
                    "state": "error",
                    "command_sent": False,
                    "error": (
                        f"{field} must be 0 or have an absolute value from "
                        f"{minimum_nonzero} to 1.0"
                    ),
                    "received": args.get(field),
                    "allowed_ranges": [
                        [-1.0, -minimum_nonzero],
                        [0.0, 0.0],
                        [minimum_nonzero, 1.0],
                    ],
                }
        if vx == 0.0 and vy == 0.0 and vyaw == 0.0:
            return {
                "state": "error", "command_sent": False,
                "error": (
                    "move requires at least one non-zero velocity; use stop_move to send "
                    "a zero-velocity stop command"
                ),
            }

        mode = int(self._high_ctrl.get_mode())
        if mode == 26:
            return {
                "state": "error", "command_sent": False,
                "current_workmode": 26,
                "current_workmode_name": "protection",
                "protection": True,
                "error": (
                    "The robot is in protection mode. No movement command was sent; stop "
                    "operation and inspect the robot before following the documented restart "
                    "procedure."
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
                    "loco.move can run only when motion_state reports workmode.code=2 "
                    "(walking). No WALK trigger or velocity command was sent."
                ),
                "why_not_automatic": (
                    "The card does not automatically enable, stand up or enter walking because "
                    "HighController cannot verify the physical pose, floor or surrounding space."
                ),
                "recovery_steps": [
                    "Place Bumi upright with both feet stable on a flat non-slip floor.",
                    "Keep people and obstacles out of the intended path.",
                    "Use a documented supported control to enter walking mode.",
                    "Call motion_state and confirm workmode.code=2 before retrying loco.move.",
                ],
                "message": (
                    "Prepare the robot safely, enter walking mode, verify workmode.code=2 with "
                    "motion_state, and then retry. Never force walking from a lying pose."
                ),
            }

        # Stop the previous bounded command before reusing the shared event.
        self._move_stop_event.set()
        if self._move_thread and self._move_thread.is_alive():
            self._move_thread.join(timeout=1)
            if self._move_thread.is_alive():
                return {
                    "state": "error", "command_sent": False,
                    "error": "The previous loco command did not stop within 1 second",
                }

        # Capture the stationary baseline before WALK carries the requested
        # velocity. Sampling afterwards would insert an avoidable gap between
        # the edge and the DEFAULT velocity stream.
        try:
            baseline_samples = []
            for sample_index in range(3):
                baseline_samples.append(self._sample_max_joint_velocity())
                if sample_index < 2:
                    time.sleep(0.05)
            baseline_joint_velocity = max(baseline_samples)
        except Exception as exc:
            return {
                "state": "error", "command_sent": False,
                "error": f"Cannot verify pre-move joint feedback: {exc}",
                "message": "No walking activation or velocity command was sent.",
            }

        # Trigger WALK for every bounded move. The previous implementation kept
        # a process-local "armed" flag after sending zero velocity, but that flag
        # cannot prove that the firmware still accepts this writer's next
        # command. A fresh, non-toggle WALK edge makes separate MCP calls
        # deterministic while DEFAULT remains the continuous velocity frame,
        # matching the vendor example and the successful host-side A/B test.
        activation_status = {}
        observed = self._send_edge_and_wait(
            _get_control_cmd("WALK"), {2, 26}, timeout_s=1.0,
            command_values=(vx, vy, vyaw),
            required_mode_before_send=2,
            send_status=activation_status)
        walking_policy_refreshed = bool(
            activation_status.get("command_sent"))
        if observed == 26:
            return self._protection_error(
                "move", [], observed,
                "Keep the robot standing on a flat non-slip floor with a clear path.",
                command_sent=walking_policy_refreshed)
        if not walking_policy_refreshed:
            return {
                "state": "error", "command_sent": False,
                "current_workmode": observed,
                "current_workmode_name": _WORKMODE_NAMES.get(
                    observed, "unknown"),
                "error": (
                    "The WALK activation edge was not sent because the driver stopped or "
                    "workmode left walking during command preparation. No velocity stream "
                    "was started."
                ),
            }
        if observed != 2:
            return {
                "state": "error",
                "command_sent": walking_policy_refreshed,
                "current_workmode": observed,
                "current_workmode_name": _WORKMODE_NAMES.get(observed, "unknown"),
                "error": (
                    "The WALK activation edge was sent, but workmode=2 was not observed. "
                    "No velocity stream was started."
                ),
            }

        self._move_stop_event.clear()
        default_cmd = _get_default_cmd()

        # Send the first frame synchronously so command_sent=True is truthful.
        self._publish_cmd(vx, vy, vyaw, default_cmd, 0)

        move_observation = {"stop_reason": "duration_elapsed"}
        self._move_observation = move_observation

        def _move_timed():
            end_time = time.monotonic() + duration
            next_send = time.monotonic() + self._control_period
            try:
                while (not self._move_stop_event.is_set()
                       and time.monotonic() < end_time):
                    current_mode = int(self._high_ctrl.get_mode())
                    if current_mode != 2:
                        move_observation["stop_reason"] = "workmode_left_walking"
                        move_observation["observed_workmode"] = current_mode
                        break
                    time.sleep(max(0.0, next_send - time.monotonic()))
                    if self._move_stop_event.is_set() or time.monotonic() >= end_time:
                        if self._move_stop_event.is_set():
                            move_observation["stop_reason"] = "stop_requested"
                        break
                    current_mode = int(self._high_ctrl.get_mode())
                    if current_mode != 2:
                        move_observation["stop_reason"] = "workmode_left_walking"
                        move_observation["observed_workmode"] = current_mode
                        break
                    self._publish_cmd(vx, vy, vyaw, default_cmd, 0)
                    next_send += self._control_period
                if (self._move_stop_event.is_set()
                        and move_observation["stop_reason"] == "duration_elapsed"):
                    move_observation["stop_reason"] = "stop_requested"
            finally:
                self._publish_cmd(0, 0, 0, default_cmd, 0)
                print(
                    f"[loco] velocity stream stopped: "
                    f"{json.dumps(move_observation, ensure_ascii=False)}",
                    flush=True,
                )

        self._move_thread = threading.Thread(
            target=_move_timed, daemon=True, name="bumi_move")
        self._move_thread.start()

        confirmation_threshold = max(
            self._MOVE_CONFIRM_MIN_JOINT_VELOCITY,
            baseline_joint_velocity + self._MOVE_CONFIRM_BASELINE_MARGIN,
        )
        confirmation_deadline = time.monotonic() + min(
            self._MOVE_CONFIRM_TIMEOUT_S, duration)
        observed_peak_joint_velocity = baseline_joint_velocity
        confirmation_samples = 0
        confirmation_error = None
        while (self._move_thread.is_alive()
               and time.monotonic() < confirmation_deadline):
            try:
                observed_peak_joint_velocity = max(
                    observed_peak_joint_velocity,
                    self._sample_max_joint_velocity(),
                )
                confirmation_samples += 1
                if observed_peak_joint_velocity >= confirmation_threshold:
                    break
            except Exception as exc:
                confirmation_error = str(exc)
                break
            time.sleep(0.05)

        confirmed_started = (
            confirmation_error is None
            and observed_peak_joint_velocity >= confirmation_threshold
        )
        if not confirmed_started:
            self._move_stop_event.set()
            self._move_thread.join(timeout=1)
            observed_mode = int(self._high_ctrl.get_mode())
            return {
                "state": "error",
                "command_sent": True,
                "confirmed_started": False,
                "workmode": observed_mode,
                "workmode_name": _WORKMODE_NAMES.get(observed_mode, "unknown"),
                "normalized_command": {
                    "forward": vx, "lateral": vy, "turn": vyaw},
                "duration_s": duration,
                "walking_policy_refreshed": walking_policy_refreshed,
                "baseline_max_joint_velocity_rad_s": round(
                    baseline_joint_velocity, 4),
                "observed_peak_joint_velocity_rad_s": round(
                    observed_peak_joint_velocity, 4),
                "motion_confirmation_threshold_rad_s": round(
                    confirmation_threshold, 4),
                "confirmation_samples": confirmation_samples,
                "confirmation_error": confirmation_error,
                "error": (
                    "The HighController command stream was sent, but joint feedback did not "
                    "confirm that locomotion started. Zero velocity was sent."
                ),
                "possible_causes": [
                    "The requested normalized velocity may be inside a firmware startup "
                    "deadband.",
                    "Another Bumi app, driver container, SDK process, or remote controller is "
                    "publishing zero velocity and overriding this driver.",
                    "The firmware reports walking mode but is not accepting this SDK client's "
                    "velocity stream.",
                ],
            }

        return {
            "state": "running",
            "command_sent": True,
            "confirmed_started": True,
            "workmode": mode,
            "workmode_name": "walking",
            "normalized_command": {"forward": vx, "lateral": vy, "turn": vyaw},
            "duration_s": duration,
            "control_rate_hz": round(1.0 / self._control_period),
            "walking_policy_refreshed": walking_policy_refreshed,
            "baseline_max_joint_velocity_rad_s": round(
                baseline_joint_velocity, 4),
            "observed_peak_joint_velocity_rad_s": round(
                observed_peak_joint_velocity, 4),
            "motion_confirmation_threshold_rad_s": round(
                confirmation_threshold, 4),
            "message": (
                "Joint feedback confirms that locomotion started. The bounded walking command "
                "is running; zero velocity will be sent when duration_s expires, stop_move is "
                "called, or workmode leaves walking."
            ),
        }

    def _stop_move(self) -> dict:
        with self._move_session_lock:
            self._move_stop_event.set()
            was_running = bool(
                self._move_thread and self._move_thread.is_alive())
            if self._move_thread and self._move_thread.is_alive():
                self._move_thread.join(timeout=1)
            if self._high_ctrl is not None:
                self._publish_cmd(0, 0, 0, _get_default_cmd(), 0)
            mode = (
                int(self._high_ctrl.get_mode())
                if self._high_ctrl is not None else None)
            return {
                "state": "completed",
                "command_sent": self._high_ctrl is not None,
                "was_running": was_running,
                "workmode": mode,
                "workmode_name": _WORKMODE_NAMES.get(mode, "unknown"),
                "message": "Zero forward, lateral and turning velocity was sent.",
            }

    def _do_posture_action(self, action: str, args: dict) -> dict:
        safety = self._safety_requirements(action)
        current_mode = int(self._high_ctrl.get_mode())
        mode_error = self._posture_workmode_error(
            action, current_mode, safety)
        if mode_error is not None:
            return mode_error
        motion_indices = tuple(range(len(_BUMI_JOINT_NAMES)))
        motion_baseline = None
        if action == "lie_prone":
            try:
                motion_baseline = self._capture_joint_positions(motion_indices)
            except Exception as exc:
                return {
                    "state": "error",
                    "command_sent": False,
                    "requested_action": action,
                    "error": f"Cannot capture pre-action joint state: {exc}",
                    "message": "No STANDTOFALL command was sent.",
                    "safety_requirements": safety,
                }
        pose_check = None
        if action == "stand_up":
            # Re-check in the worker because the robot may have been moved
            # after the synchronous MCP validation but before commands begin.
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
        if action == "lie_prone" and not result.get("confirmed_started"):
            result["state"] = "error"
            result["error"] = (
                "STANDTOFALL was sent, but workmode=28 was not observed, so the "
                "lie-prone action did not confirm startup."
            )
        if (action == "lie_prone" and result.get("confirmed_started")
                and motion_baseline is not None):
            motion_confirmation = self._confirm_joint_displacement(
                motion_baseline, motion_indices,
                _LIE_PRONE_MIN_JOINT_DISPLACEMENT_RAD,
                _LIE_PRONE_MOTION_START_TIMEOUT_S, {28},
                _LIE_PRONE_MIN_MOVED_JOINT_COUNT)
            result["physical_motion_confirmation"] = motion_confirmation
            if not motion_confirmation["confirmed"]:
                observed_mode = motion_confirmation["observed_workmode"]
                if observed_mode == 26:
                    protection_result = self._protection_error(
                        action, prepared["steps"], observed_mode, safety,
                        command_sent=True)
                    protection_result["physical_motion_confirmation"] = (
                        motion_confirmation)
                    return self._attach_battery_failure_diagnostic(
                        protection_result)
                recovery_result = None
                if observed_mode == 28:
                    recovery_result = self._send_walk_exit(
                        "lie_prone_start_failed", safety)
                result.update({
                    "state": "error",
                    "confirmed_started": False,
                    "mode_start_confirmed": True,
                    "physical_motion_confirmed": False,
                    "current_workmode": observed_mode,
                    "current_workmode_name": _WORKMODE_NAMES.get(
                        observed_mode, "unknown"),
                    "error": (
                        "STANDTOFALL mode was observed, but whole-body joint feedback did not "
                        "confirm that the physical lie-prone action started."
                    ),
                    "message": (
                        "The robot may have rejected or aborted the physical action. Check the "
                        "battery result, charge Bumi if an alarm is present, and inspect the "
                        "standing pose and floor before retrying."
                    ),
                })
                if recovery_result is not None:
                    result["walking_recovery"] = recovery_result
                return self._attach_battery_failure_diagnostic(result)
            else:
                result["physical_motion_confirmed"] = True
        if pose_check is not None:
            result["pose_check"] = pose_check
            result["pose_verification"] = (
                "IMU direction, stillness, and median joint positions passed the automatic "
                "face-up check. Floor condition, objects under the feet, and surrounding "
                "clearance still require visual confirmation by the user."
            )
        return result

    def _posture_workmode_error(self, action: str, current_mode: int,
                                safety: str) -> dict | None:
        if current_mode == 26:
            return self._protection_error(
                action, [], current_mode, safety)
        allowed_modes = {0, 30} if action == "stand_up" else {2}
        if current_mode in allowed_modes:
            return None
        return {
            "state": "error",
            "command_sent": False,
            "requested_action": action,
            "current_workmode": current_mode,
            "current_workmode_name": _WORKMODE_NAMES.get(
                current_mode, "unknown"),
            "allowed_workmodes": [
                {"code": mode, "name": _WORKMODE_NAMES.get(mode, "unknown")}
                for mode in sorted(allowed_modes)
            ],
            "error": (
                "stand_up is allowed only when the robot is face-up on the floor in "
                "disabled or enabled mode. It is blocked while the robot is already "
                "standing, ready, walking, or executing another action."
                if action == "stand_up" else
                "lie_prone is allowed only from walking mode after stable standing has "
                "been confirmed."
            ),
            "message": (
                "Do not call stand_up while Bumi is already standing. To lie down, "
                "use lie_prone after confirming stable walking mode."
                if action == "stand_up" else
                "Enter walking mode with Bumi standing steadily before requesting "
                "lie_prone."
            ),
            "safety_requirements": safety,
        }

    def _prepare_stand_up(self, requested_action: str) -> dict:
        """Reach enabled mode without entering ready, which assumes assisted standing."""
        # The tested firmware can consume the first START edge only as a late
        # wake-up when stale walking velocity remains on the control path. The
        # previously verified workaround is an explicit stop_move zero frame,
        # followed by the sustained START preroll below. START itself remains a
        # single edge because repeating this toggle could disable the robot.
        stop_result = self._stop_move()
        mode = stop_result.get("workmode")
        if mode is None:
            mode = int(self._high_ctrl.get_mode())
        steps = [{
            "step": "clear_motion",
            "command": "DEFAULT",
            "zero_velocity_sent": bool(stop_result.get("command_sent")),
            "observed_workmode": mode,
            "observed_workmode_name": _WORKMODE_NAMES.get(mode, "unknown"),
            "confirmed": bool(stop_result.get("command_sent")),
        }]
        if mode == 26:
            return self._protection_error(requested_action, steps, mode)
        if mode == 30:
            for attempt in range(1, _STAND_ENABLE_MAX_ATTEMPTS + 1):
                if mode != 30:
                    break
                mode = self._run_preparation_step(
                    f"enable_attempt_{attempt}", "START", {0}, steps,
                    observation_timeout_s=_STAND_ENABLE_MODE_TIMEOUT_S,
                    required_mode_before_send=30)
                if mode == 0 or mode == 26:
                    break
            if mode == 26:
                return self._protection_error(requested_action, steps, mode)
            if mode != 0:
                return self._preparation_error(
                    requested_action, steps, mode,
                    "The robot did not enter enabled mode after two guarded START attempts. "
                    "FALLTOSTAND was not sent.")
        if mode != 0:
            return self._preparation_error(
                requested_action, steps, mode,
                "Autonomous stand-up requires disabled or enabled mode. FALLTOSTAND was not sent.")
        settle_result = self._wait_for_enabled_stable()
        steps.append(settle_result)
        mode = settle_result["observed_workmode"]
        if mode == 26:
            return self._protection_error(requested_action, steps, mode)
        if not settle_result["confirmed"]:
            return self._preparation_error(
                requested_action, steps, mode,
                "Enabled mode was observed, but the mode and joint feedback did not become "
                "stable before the stand-up timeout. FALLTOSTAND was not sent.")
        return {"state": "completed", "steps": steps, "workmode": mode}

    def _wait_for_enabled_stable(self) -> dict:
        """Wait for motor-enable initialization to settle before FALLTOSTAND."""
        started_at = time.monotonic()
        deadline = started_at + _STAND_ENABLE_SETTLE_TIMEOUT_S
        stable_since = None
        velocity_window = []
        sample_count = 0
        peak_joint_velocity = 0.0
        last_window_median = None
        observed_mode = int(self._high_ctrl.get_mode())
        error = None

        while time.monotonic() < deadline:
            observed_mode = int(self._high_ctrl.get_mode())
            if observed_mode != 0:
                error = (
                    "The robot left enabled mode while waiting for motor initialization "
                    "to settle."
                )
                break
            try:
                max_velocity = self._sample_max_joint_velocity()
            except Exception as exc:
                error = f"Cannot read joint feedback while enabled mode settles: {exc}"
                break

            sample_count += 1
            peak_joint_velocity = max(peak_joint_velocity, max_velocity)
            velocity_window.append(max_velocity)
            if len(velocity_window) > _STAND_ENABLE_VELOCITY_WINDOW_SIZE:
                velocity_window.pop(0)

            now = time.monotonic()
            window_stationary = False
            if len(velocity_window) == _STAND_ENABLE_VELOCITY_WINDOW_SIZE:
                ordered = sorted(velocity_window)
                last_window_median = ordered[len(ordered) // 2]
                window_stationary = (
                    last_window_median <= _STAND_POSE_MAX_JOINT_VELOCITY
                    and max(velocity_window)
                    <= _STAND_POSE_MAX_JOINT_VELOCITY_SPIKE
                )
            if window_stationary:
                if stable_since is None:
                    stable_since = now
            else:
                stable_since = None

            dwell_s = now - started_at
            stable_s = 0.0 if stable_since is None else now - stable_since
            if (dwell_s >= _STAND_ENABLE_MIN_DWELL_S
                    and stable_s >= _STAND_ENABLE_STATIONARY_CONFIRM_S):
                return {
                    "step": "settle_enabled",
                    "command": None,
                    "observed_workmode": observed_mode,
                    "observed_workmode_name": _WORKMODE_NAMES.get(
                        observed_mode, "unknown"),
                    "confirmed": True,
                    "sample_count": sample_count,
                    "settle_elapsed_s": round(dwell_s, 3),
                    "stationary_confirmed_s": round(stable_s, 3),
                    "peak_abs_joint_velocity_rad_s": round(
                        peak_joint_velocity, 4),
                    "final_window_median_velocity_rad_s": round(
                        last_window_median, 4),
                    "meaning": (
                        "Enabled mode remained active and joint feedback settled before "
                        "FALLTOSTAND was sent."
                    ),
                }
            time.sleep(_STAND_ENABLE_SAMPLE_INTERVAL_S)

        elapsed_s = time.monotonic() - started_at
        return {
            "step": "settle_enabled",
            "command": None,
            "observed_workmode": observed_mode,
            "observed_workmode_name": _WORKMODE_NAMES.get(
                observed_mode, "unknown"),
            "confirmed": False,
            "sample_count": sample_count,
            "settle_elapsed_s": round(elapsed_s, 3),
            "peak_abs_joint_velocity_rad_s": round(peak_joint_velocity, 4),
            "final_window_median_velocity_rad_s": (
                None if last_window_median is None
                else round(last_window_median, 4)
            ),
            "error": error or (
                "Enabled mode did not produce sufficiently stable joint feedback before "
                "the settle timeout."
            ),
            "thresholds": {
                "minimum_enabled_dwell_s": _STAND_ENABLE_MIN_DWELL_S,
                "stationary_confirmation_s": (
                    _STAND_ENABLE_STATIONARY_CONFIRM_S),
                "median_maximum_joint_velocity_rad_s": (
                    _STAND_POSE_MAX_JOINT_VELOCITY),
                "maximum_joint_velocity_spike_rad_s": (
                    _STAND_POSE_MAX_JOINT_VELOCITY_SPIKE),
            },
        }

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
        motion_baseline = None
        if action == "wipe_tears":
            try:
                # Capture after preparation so enable/ready/walking transitions
                # cannot be mistaken for physical TEAR motion.
                motion_baseline = self._capture_joint_positions(_ARM_JOINT_INDICES)
            except Exception as exc:
                return {
                    "state": "error",
                    "command_sent": bool(prepared["steps"]),
                    "requested_action": action,
                    "preparation_steps": prepared["steps"],
                    "error": f"Cannot capture pre-action arm state: {exc}",
                    "message": "No TEAR command was sent.",
                    "safety_requirements": safety,
                }
        command_name, expected_modes = _PRESET_ACTIONS[action]
        result = self._trigger_user_action(
            action, command_name, expected_modes, prepared["steps"], safety)
        if action == "wipe_tears" and not result.get("confirmed_started"):
            result["state"] = "error"
            result["error"] = (
                "TEAR was sent, but workmode=33 was not observed, so the wipe-tears "
                "action did not confirm startup."
            )
        if action == "wipe_tears" and result.get("confirmed_started"):
            motion_confirmation = self._confirm_joint_displacement(
                motion_baseline, _ARM_JOINT_INDICES,
                _WIPE_TEARS_MIN_ARM_DISPLACEMENT_RAD,
                _WIPE_TEARS_MOTION_START_TIMEOUT_S, {33})
            result["physical_motion_confirmation"] = motion_confirmation
            if not motion_confirmation["confirmed"]:
                observed_mode = motion_confirmation["observed_workmode"]
                if observed_mode == 26:
                    protection_result = self._protection_error(
                        action, prepared["steps"], observed_mode, safety,
                        command_sent=True)
                    protection_result["physical_motion_confirmation"] = (
                        motion_confirmation)
                    return self._attach_battery_failure_diagnostic(
                        protection_result)
                recovery_result = None
                if observed_mode == 33:
                    recovery_result = self._send_walk_exit(
                        "wipe_tears_start_failed", safety)
                result.update({
                    "state": "error",
                    "confirmed_started": False,
                    "mode_start_confirmed": True,
                    "physical_motion_confirmed": False,
                    "auto_return_to_walk": False,
                    "current_workmode": observed_mode,
                    "current_workmode_name": _WORKMODE_NAMES.get(
                        observed_mode, "unknown"),
                    "error": (
                        "TEAR mode was observed, but arm joint feedback did not confirm that "
                        "the physical wipe-tears action started."
                    ),
                    "message": (
                        "The robot may have rejected the physical action. Check the battery "
                        "result and charge Bumi before retrying if a battery alarm is present."
                    ),
                })
                if recovery_result is not None:
                    result["walking_recovery"] = recovery_result
                return self._attach_battery_failure_diagnostic(result)
            result["physical_motion_confirmed"] = True
            result.update({
                "auto_return_to_walk": True,
                "auto_return_after_s": _TEAR_AUTO_EXIT_S,
                "auto_return_condition": (
                    "If the robot is still in tear mode when the timer expires, the card "
                    "uses at most two guarded WALK attempts and stops as soon as walking "
                    "mode is observed."
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
            recording_id_value = args["recording_id"]
            if type(recording_id_value) is not int:
                return {"state": "error", "command_sent": False,
                        "error": "recording_id must be an integer", "safety_requirements": safety}
            recording_id = recording_id_value
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
            result.update({
                "auto_return_to_walk": True,
                "completion_detection": "inferred_from_workmode_and_joint_velocity",
                "auto_return_condition": (
                    "After joint motion or displacement has been observed, a rolling "
                    "five-sample velocity median must remain stationary for a sustained "
                    "3-second score while the robot is still in play_teach mode. The card "
                    "then uses at most two guarded WALK attempts to confirm walking mode."
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
                                 delay_s: float, safety: str,
                                 action_id: str | None = None) -> None:
        self._cancel_auto_exit(key)

        def _auto_exit() -> None:
            with self._auto_exit_lock:
                if self._auto_exit_timers.get(key) is not timer:
                    return
                self._auto_exit_timers.pop(key, None)
                timer_action_id = self._auto_exit_action_ids.pop(key, None)
            current_mode = int(self._high_ctrl.get_mode())
            if current_mode != expected_mode:
                print(
                    f"[loco] {key} auto-return skipped: workmode={current_mode} "
                    f"({_WORKMODE_NAMES.get(current_mode, 'unknown')})",
                    flush=True,
                )
                if timer_action_id is not None:
                    status = "completed" if current_mode == 2 else "error"
                    self._finish_acp(timer_action_id, status, {
                        "action": key,
                        "final_workmode": current_mode,
                        "final_workmode_name": _WORKMODE_NAMES.get(
                            current_mode, "unknown"),
                        "reason": (
                            "firmware_already_returned_to_walking"
                            if current_mode == 2 else
                            "workmode_changed_before_auto_return"
                        ),
                    })
                return
            result = self._send_walk_exit(f"{key}_auto_return", safety)
            print(
                f"[loco] {key} auto-return result: {json.dumps(result)}",
                flush=True,
            )
            if timer_action_id is not None:
                status = "completed" if result.get("confirmed") else "error"
                self._finish_acp(timer_action_id, status, result)

        timer = threading.Timer(delay_s, _auto_exit)
        timer.daemon = True
        with self._auto_exit_lock:
            self._auto_exit_timers[key] = timer
            if action_id is not None:
                self._auto_exit_action_ids[key] = action_id
        timer.start()

    def _schedule_playback_completion_monitor(
            self, safety: str, action_id: str | None = None) -> None:
        self._cancel_playback_monitor()
        stop_event = threading.Event()

        def _monitor() -> None:
            # Avoid posting completion before Agent Core has registered the
            # action_id from the MCP response for a very short recording.
            if stop_event.wait(0.25):
                return
            started_at = time.monotonic()
            motion_seen = False
            stationary_score_s = 0.0
            exit_reason = None
            velocity_window = []
            initial_positions = None
            peak_joint_velocity = 0.0
            peak_joint_displacement = 0.0
            final_window_median_velocity = None
            sample_count = 0

            while not stop_event.wait(0.05):
                now = time.monotonic()
                mode = int(self._high_ctrl.get_mode())
                if mode == 2:
                    print(
                        "[loco] play_recording completed: firmware already returned to walking",
                        flush=True,
                    )
                    if action_id is not None:
                        self._finish_acp(action_id, "completed", {
                            "action": "play_recording",
                            "final_workmode": 2,
                            "final_workmode_name": "walking",
                            "reason": "firmware_returned_to_walking",
                        })
                    return
                if mode == 26:
                    print(
                        "[loco] play_recording monitor stopped: robot entered protection mode",
                        flush=True,
                    )
                    if action_id is not None:
                        self._finish_acp(action_id, "error", {
                            "action": "play_recording",
                            "final_workmode": 26,
                            "final_workmode_name": "protection",
                            "error": "The robot entered protection mode during playback.",
                        })
                    return
                if mode != 23:
                    print(
                        f"[loco] play_recording monitor stopped: workmode={mode} "
                        f"({_WORKMODE_NAMES.get(mode, 'unknown')})",
                        flush=True,
                    )
                    if action_id is not None:
                        self._finish_acp(action_id, "error", {
                            "action": "play_recording",
                            "final_workmode": mode,
                            "final_workmode_name": _WORKMODE_NAMES.get(
                                mode, "unknown"),
                            "error": "Playback left play_teach mode unexpectedly.",
                        })
                    return

                try:
                    joint_states = self._high_ctrl.get_joint_state()
                    if len(joint_states) != len(_BUMI_JOINT_NAMES):
                        raise RuntimeError(
                            f"HighController returned {len(joint_states)} joints; expected "
                            f"{len(_BUMI_JOINT_NAMES)}")
                    positions = [float(state.pos) for state in joint_states]
                    max_velocity = max(
                        abs(float(state.vel)) for state in joint_states)
                    if initial_positions is None:
                        initial_positions = positions
                    max_displacement = max(
                        abs(position - initial)
                        for position, initial in zip(
                            positions, initial_positions))
                except Exception as exc:
                    print(
                        f"[loco] play_recording completion sample failed: {exc}",
                        flush=True,
                    )
                    continue

                sample_count += 1
                peak_joint_velocity = max(peak_joint_velocity, max_velocity)
                peak_joint_displacement = max(
                    peak_joint_displacement, max_displacement)
                velocity_window.append(max_velocity)
                if len(velocity_window) > _PLAYBACK_VELOCITY_WINDOW_SIZE:
                    velocity_window.pop(0)
                if len(velocity_window) < _PLAYBACK_VELOCITY_WINDOW_SIZE:
                    continue
                ordered_velocities = sorted(velocity_window)
                median_velocity = ordered_velocities[
                    len(ordered_velocities) // 2]
                final_window_median_velocity = median_velocity

                if (median_velocity >= _PLAYBACK_MOVING_THRESHOLD
                        or max_displacement
                        >= _PLAYBACK_MIN_JOINT_DISPLACEMENT_RAD):
                    motion_seen = True
                if (motion_seen
                        and median_velocity <= _PLAYBACK_STATIONARY_THRESHOLD):
                    stationary_score_s = min(
                        _PLAYBACK_STATIONARY_CONFIRM_S,
                        stationary_score_s + 0.05,
                    )
                    if stationary_score_s >= _PLAYBACK_STATIONARY_CONFIRM_S:
                        exit_reason = "joint_motion_completed"
                        break
                elif (motion_seen and median_velocity
                      >= _PLAYBACK_MOTION_RESET_THRESHOLD):
                    # The rolling median requires a majority of recent samples
                    # to be moving, so an isolated encoder spike cannot erase
                    # the stationary score.
                    stationary_score_s = 0.0

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
            result["playback_observation"] = {
                "sample_count": sample_count,
                "peak_abs_joint_velocity_rad_s": round(
                    peak_joint_velocity, 4),
                "peak_joint_displacement_rad": round(
                    peak_joint_displacement, 4),
                "final_window_median_velocity_rad_s": (
                    None if final_window_median_velocity is None
                    else round(final_window_median_velocity, 4)
                ),
                "stationary_score_s": round(stationary_score_s, 3),
                "velocity_window_size": _PLAYBACK_VELOCITY_WINDOW_SIZE,
            }
            print(
                f"[loco] play_recording auto-return result: {json.dumps(result)}",
                flush=True,
            )
            if action_id is not None:
                completed = (
                    exit_reason == "joint_motion_completed"
                    and result.get("confirmed")
                )
                if exit_reason == "no_joint_motion_detected":
                    result["error"] = (
                        "No joint motion was detected after playback was triggered."
                    )
                elif exit_reason == "maximum_monitor_time_reached":
                    result["error"] = (
                        "Playback did not complete within the 120-second monitor limit."
                    )
                elif not result.get("confirmed"):
                    result["error"] = (
                        "Playback motion ended, but walking-mode recovery was not confirmed."
                    )
                self._finish_acp(
                    action_id, "completed" if completed else "error", result)

        monitor_thread = threading.Thread(
            target=_monitor,
            daemon=True,
            name="bumi_playback_completion_monitor",
        )
        with self._auto_exit_lock:
            self._playback_monitor_stop = stop_event
            self._playback_monitor_thread = monitor_thread
            self._playback_action_id = action_id
        monitor_thread.start()

    def _cancel_playback_monitor(self) -> None:
        with self._auto_exit_lock:
            stop_event = self._playback_monitor_stop
            action_id = self._playback_action_id
            self._playback_monitor_stop = None
            self._playback_monitor_thread = None
            self._playback_action_id = None
        if stop_event is not None:
            stop_event.set()
        if action_id is not None:
            self._finish_acp(action_id, "cancelled", {
                "action": "play_recording",
                "reason": "playback monitor cancelled",
            })

    def _cancel_auto_exit(self, key: str) -> None:
        with self._auto_exit_lock:
            timer = self._auto_exit_timers.pop(key, None)
            action_id = self._auto_exit_action_ids.pop(key, None)
        if timer is not None:
            timer.cancel()
        if action_id is not None:
            self._finish_acp(action_id, "cancelled", {
                "action": key,
                "reason": "automatic return timer cancelled",
            })

    def _cancel_all_auto_exits(self) -> None:
        with self._auto_exit_lock:
            timers = list(self._auto_exit_timers.values())
            action_ids = list(self._auto_exit_action_ids.items())
            self._auto_exit_timers.clear()
            self._auto_exit_action_ids.clear()
        for timer in timers:
            timer.cancel()
        for key, action_id in action_ids:
            self._finish_acp(action_id, "cancelled", {
                "action": key,
                "reason": "automatic return timers cancelled",
            })

    def _send_walk_exit(self, requested_action: str, safety: str) -> dict:
        source_mode = int(self._high_ctrl.get_mode())
        if source_mode == 26:
            return self._protection_error(
                requested_action, [], source_mode, safety,
                command_sent=False)
        if source_mode == 2:
            return {
                "state": "completed",
                "command_sent": False,
                "requested_action": requested_action,
                "confirmed": True,
                "workmode": 2,
                "workmode_name": "walking",
                "exit_attempts": [],
                "safety_requirements": safety,
                "message": (
                    "The robot had already returned to walking mode; no WALK command "
                    "was sent."
                ),
            }

        attempts = []
        observed = source_mode
        for attempt in range(1, _WALK_EXIT_MAX_ATTEMPTS + 1):
            if observed != source_mode:
                break
            send_status = {}
            observed = self._send_edge_and_wait(
                _get_control_cmd("WALK"), {2, 26},
                timeout_s=_WALK_EXIT_CONFIRM_TIMEOUT_S,
                required_mode_before_send=source_mode,
                send_status=send_status)
            attempt_result = {
                "attempt": attempt,
                "source_workmode": source_mode,
                "source_workmode_name": _WORKMODE_NAMES.get(
                    source_mode, "unknown"),
                "command_sent": bool(send_status.get("command_sent")),
                "observed_workmode": observed,
                "observed_workmode_name": _WORKMODE_NAMES.get(
                    observed, "unknown"),
                "confirmed": observed == 2,
            }
            if send_status.get("blocked_by_mode_change"):
                attempt_result["send_skipped_reason"] = (
                    "Workmode changed before WALK was sent. The retry was skipped."
                )
            attempts.append(attempt_result)
            print(
                "[loco] walk_exit_attempt "
                + json.dumps(
                    attempt_result, ensure_ascii=True,
                    separators=(",", ":")),
                flush=True,
            )
            if observed in {2, 26}:
                break

        if observed == 26:
            result = self._protection_error(
                requested_action, attempts, observed, safety,
                command_sent=any(
                    item["command_sent"] for item in attempts))
            result["exit_attempts"] = attempts
            return result
        confirmed = observed == 2
        return {
            "state": "completed" if confirmed else "error",
            "command_sent": any(
                item["command_sent"] for item in attempts),
            "requested_action": requested_action,
            "confirmed": confirmed,
            "workmode": observed,
            "workmode_name": _WORKMODE_NAMES.get(observed, "unknown"),
            "preparation_steps": [],
            "exit_attempts": attempts,
            "safety_requirements": safety,
            "message": (
                "The active action was exited and walking mode was confirmed."
                if confirmed else
                "Walking mode was not confirmed during the guarded WALK exit. Check "
                "exit_attempts, robot workmode, and battery before retrying or restarting "
                "the robot."
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
                              expected_modes: set[int], steps: list[dict],
                              observation_timeout_s: float | None = None,
                              required_mode_before_send: int | None = None) -> int:
        # START is a toggle, so every edge must be guarded by the current mode.
        # On the tested Bumi, the disabled state reached after STANDTOFALL can
        # ignore a START edge until it has first received a sustained neutral
        # stream. Prime each guarded attempt instead of only once per process.
        neutral_preroll_s = (
            self._control_cold_preroll_s if command_name == "START" else None)
        if observation_timeout_s is None:
            observation_timeout_s = 6.0 if command_name == "START" else 3.0
        send_status = {}
        observed = self._send_edge_and_wait(
            _get_control_cmd(command_name), expected_modes | {26},
            timeout_s=observation_timeout_s,
            preroll_override_s=neutral_preroll_s,
            required_mode_before_send=required_mode_before_send,
            send_status=send_status)
        step_result = {
            "step": step,
            "command": command_name,
            "command_sent": bool(send_status.get("command_sent")),
            "expected_workmodes": sorted(expected_modes),
            "observed_workmode": observed,
            "observed_workmode_name": _WORKMODE_NAMES.get(observed, "unknown"),
            "confirmed": observed in expected_modes,
            "observation_timeout_s": observation_timeout_s,
        }
        if neutral_preroll_s is not None:
            step_result["neutral_preroll_s"] = neutral_preroll_s
        if send_status.get("blocked_by_mode_change"):
            step_result["send_skipped_reason"] = (
                "Workmode changed before the edge command was sent; the command was "
                "skipped to avoid reversing the transition."
            )
        steps.append(step_result)
        print(
            "[posture] preparation_step "
            + json.dumps(step_result, ensure_ascii=True, separators=(",", ":")),
            flush=True,
        )
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
        if not confirmed:
            result["physical_motion_confirmed"] = False
            self._attach_battery_failure_diagnostic(result)
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
                            preroll_override_s: float | None = None,
                            command_values: tuple[float, float, float] | None = None,
                            required_mode_before_send: int | None = None,
                            send_status: dict | None = None) -> int:
        """Prime DDS, send one action edge, then maintain neutral control frames."""
        with self._action_lock:
            default_cmd = _get_default_cmd()
            if send_status is not None:
                send_status.clear()
                send_status["command_sent"] = False

            # The vendor examples keep publishing at 100 Hz. Use a longer
            # DEFAULT-only pre-roll for a new DDS writer and whenever the caller
            # explicitly requests it for a state transition such as START.
            # This method sends at most one action edge. A higher-level caller
            # may make a bounded retry only while required_mode_before_send is
            # continuously confirmed, preventing a delayed START transition
            # from being toggled back to disabled.
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
                if self._lifecycle_stop_event.is_set():
                    return int(self._high_ctrl.get_mode())
                observed = int(self._high_ctrl.get_mode())
                if observed == 26:
                    if send_status is not None:
                        send_status["blocked_by_protection"] = True
                    return observed
                if required_mode_before_send is not None:
                    if observed != required_mode_before_send:
                        if send_status is not None:
                            send_status["blocked_by_mode_change"] = True
                        return observed
                self._publish_cmd(0, 0, 0, default_cmd, 0)
                time.sleep(self._control_period)
            self._control_channel_warmed = True

            if self._lifecycle_stop_event.is_set():
                return int(self._high_ctrl.get_mode())
            observed = int(self._high_ctrl.get_mode())
            if observed == 26:
                if send_status is not None:
                    send_status["blocked_by_protection"] = True
                return observed
            if required_mode_before_send is not None:
                if observed != required_mode_before_send:
                    if send_status is not None:
                        send_status["blocked_by_mode_change"] = True
                    return observed

            # Actions such as START and PLAYTEACH are edge-triggered. Send this
            # edge exactly once; any bounded retry is a separate guarded call.
            command_x, command_y, command_z = command_values or (0.0, 0.0, 0.0)
            self._publish_cmd(command_x, command_y, command_z, cmd_enum, index)
            if send_status is not None:
                send_status["command_sent"] = True
            time.sleep(self._control_period)

            deadline = time.monotonic() + timeout_s
            observed = int(self._high_ctrl.get_mode())
            while (observed not in expected_modes and observed != 26
                   and time.monotonic() < deadline):
                if self._lifecycle_stop_event.is_set():
                    return observed
                observed = int(self._high_ctrl.get_mode())
                if observed == 26 or observed in expected_modes:
                    return observed
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
