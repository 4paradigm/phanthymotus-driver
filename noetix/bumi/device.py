#!/usr/bin/env python3
"""
drivers/noetix/bumi/device.py — Noetix Bumi-EDU 设备插件实现。

插件列表：
  - StatePlugin: joints (21-DOF skeleton), imu, battery, model (URDF resource)
  - LocoPlugin: locomotion, stand-up/lie-down, semantic actions, action recording and debug workmode
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

# ── ControlCmd Mapping ────────────────────────────────────────────────────────
# Lazy-loaded from highcontrol_py.ControlCmd enum at runtime

_POSTURE_ACTIONS = {
    "stand_up": ("FALLTOSTAND", {27}),
    "lie_down": ("STANDTOFALL", {28, 30}),
}

_PRESET_ACTIONS = {
    "wave": ("SWING", {8}),
    "handshake": ("SHAKE", {9}),
    "cheer": ("CHEER", {10}),
    "dance_1": ("DANCE", {5}),
    "dance_2": ("DANCE1", {31}),
    "dance_3": ("DANCE2", {32}),
    "wipe_tears": ("TEAR", {33}),
}

_TEACHING_ACTIONS = {
    "start_recording": ("STARTTEACH", {11}),
    # ENDTEACH is deprecated. SAVETEACH finishes the recording and saves it.
    "finish_and_save_recording": ("SAVETEACH", {12, 14, 29}),
    "play_recording": ("PLAYTEACH", {23}),
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
        return [
            self._loco_tool(),
            self._stand_up_lie_down_tool(),
            self._semantic_action_tool(),
            self._action_recording_tool(),
            self._debug_workmode_tool(),
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

    def _stand_up_lie_down_tool(self) -> dict:
        return {
            "name": "stand_up_lie_down",
            "type": "actuator",
            "multiInstance": False,
            "description": "让 Bumi 从仰面平躺自主起身，或从正常站立姿态躺下收纳。卡片会自动完成内部使能/准备/行走模式切换；SDK 无法确认真实姿态，用户必须按 action 描述摆放机器人。错误姿态可能触发保护模式。",
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {
                        "type": "string",
                        "enum": list(_POSTURE_ACTIONS),
                        "description": "stand_up=自主起身：仅限机器人面朝上平躺、四肢自然放置、双腿伸直、脚底无异物，并在平坦防滑地面留出至少 3m×3m 无人无障碍空间；lie_down=躺下收纳：仅限机器人已稳定站立，并在平坦防滑地面留出至少 3m×3m 无人无障碍空间。",
                    },
                },
                "required": ["action"],
                "x-action-params": {
                    "stand_up": {
                        "params": [],
                        "description": "从仰面平躺自主起身。调用即会自动准备内部模式并执行；调用前必须完成姿态和 3m×3m 环境检查。",
                    },
                    "lie_down": {
                        "params": [],
                        "description": "从稳定站立姿态躺下收纳。调用即会自动准备内部模式并执行；调用前必须确认地面和 3m×3m 环境安全。",
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
                        "description": "wave=挥手；handshake=握手；cheer=欢呼；dance_1/dance_2/dance_3=三种出厂舞蹈；wipe_tears=擦眼泪。选择后立即执行。",
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
                        "wipe_tears": "执行擦眼泪动作。确认机器人稳定站立且手臂周围无障碍物。",
                    }.items()
                },
            },
            "topic_out": [],
        }

    def _action_recording_tool(self) -> dict:
        return {
            "name": "action_recording", "type": "actuator", "multiInstance": False,
            "description": "录制、结束并保存或播放 Bumi 示教动作。start_recording 和 play_recording 会自动进入所需行走模式；finish_and_save_recording 只能在已开始录制后使用。示教时不得强推关节至限位，播放前须确认机器人稳定站立且周围空间安全。",
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {
                        "type": "string", "enum": list(_TEACHING_ACTIONS),
                        "description": "start_recording=开始录制示教；finish_and_save_recording=结束当前录制并保存；play_recording=播放已保存的示教动作。",
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
                    "play_recording": {"params": ["recording_id"], "description": "自动准备模式并播放 recording_id。确认该编号存在，机器人稳定站立，周围无人和障碍物。"},
                },
            },
            "topic_out": [],
        }

    def _debug_workmode_tool(self) -> dict:
        return {
            "name": "debug_workmode", "type": "actuator", "multiInstance": False,
            "description": "仅供开发者实机调试 Bumi 基础工作模式。直接发送 enable、disable、ready 或 walk，不自动补齐前置步骤。执行前必须由调试人员确认机器人姿态、地面、支撑和周围空间安全。",
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {
                        "type": "string",
                        "enum": ["enable", "disable", "ready", "walk"],
                        "description": "enable=失能状态下使能；disable=进入失能；ready=切换准备模式；walk=切换行走模式。卡片不会自动执行缺失的前置模式。",
                    },
                },
                "required": ["action"],
                "x-action-params": {
                    "enable": {"params": [], "description": "仅在 workmode=30（失能）时发送 START；已使能时不会重复发送。"},
                    "disable": {"params": [], "description": "从非失能状态发送 START 进入失能；已失能时不会重复发送。"},
                    "ready": {"params": [], "description": "直接发送 SWITCH 并等待 workmode=1；不会自动使能。"},
                    "walk": {"params": [], "description": "直接发送 WALK 并等待 workmode=2；不会自动进入 enable 或 ready。"},
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

        if tool_name == "loco" and action == "move":
            return self._do_move(args)
        if tool_name == "loco" and action == "stop_move":
            return self._stop_move()
        if tool_name == "stand_up_lie_down" and action in _POSTURE_ACTIONS:
            return self._do_posture_action(action)
        if tool_name == "semantic_action" and action in _PRESET_ACTIONS:
            return self._do_preset_action(action)
        if tool_name == "action_recording" and action in _TEACHING_ACTIONS:
            return self._do_teaching_action(action, args)
        if tool_name == "debug_workmode" and action in {"enable", "disable", "ready", "walk"}:
            return self._do_debug_workmode(action)
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

    def _do_posture_action(self, action: str) -> dict:
        safety = self._safety_requirements(action)
        target_mode = 1 if action == "stand_up" else 2
        prepared = self._prepare_workmode(target_mode, action)
        if prepared["state"] == "error":
            prepared["safety_requirements"] = safety
            return prepared
        command_name, expected_modes = _POSTURE_ACTIONS[action]
        return self._trigger_user_action(
            action, command_name, expected_modes, prepared["steps"], safety)

    def _do_preset_action(self, action: str) -> dict:
        safety = self._safety_requirements(action)
        prepared = self._prepare_workmode(2, action)
        if prepared["state"] == "error":
            prepared["safety_requirements"] = safety
            return prepared
        command_name, expected_modes = _PRESET_ACTIONS[action]
        return self._trigger_user_action(
            action, command_name, expected_modes, prepared["steps"], safety)

    def _do_teaching_action(self, action: str, args: dict) -> dict:
        safety = self._safety_requirements(action)
        recording_id = None
        if action in ("finish_and_save_recording", "play_recording"):
            if "recording_id" not in args:
                return {
                    "state": "error", "command_sent": False,
                    "error": f"{action} 必须填写 recording_id（0～65535）",
                    "safety_requirements": safety,
                }
            try:
                recording_id = int(args["recording_id"])
            except (TypeError, ValueError):
                return {"state": "error", "command_sent": False,
                        "error": "recording_id 必须是整数", "safety_requirements": safety}
            if not 0 <= recording_id <= 65535:
                return {"state": "error", "command_sent": False,
                        "error": "recording_id 必须在 0～65535 范围内", "safety_requirements": safety}

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
                    "error": "当前没有正在进行的示教录制；请先调用 start_recording，完成动作引导后再结束并保存",
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
        return self._trigger_user_action(
            action, command_name, expected_modes, steps, safety,
            index=recording_id or 0, recording_id=recording_id)

    def _do_debug_workmode(self, action: str) -> dict:
        current_mode = int(self._high_ctrl.get_mode())
        if action == "enable":
            if current_mode == 0:
                return self._debug_mode_result(action, current_mode, True, False)
            if current_mode != 30:
                return {
                    "state": "error", "command_sent": False,
                    "requested_action": action,
                    "current_workmode": current_mode,
                    "current_workmode_name": _WORKMODE_NAMES.get(current_mode, "unknown"),
                    "error": "enable 仅允许从 workmode=30（失能）执行；当前状态下不发送 START，避免其切换语义造成意外失能",
                }
            command_name, expected_modes = "START", {0}
        elif action == "disable":
            if current_mode == 30:
                return self._debug_mode_result(action, current_mode, True, False)
            command_name, expected_modes = "START", {30}
        elif action == "ready":
            if current_mode == 1:
                return self._debug_mode_result(action, current_mode, True, False)
            command_name, expected_modes = "SWITCH", {1}
        else:
            if current_mode == 2:
                return self._debug_mode_result(action, current_mode, True, False)
            command_name, expected_modes = "WALK", {2}

        self._move_stop_event.set()
        if self._move_thread and self._move_thread.is_alive():
            self._move_thread.join(timeout=1)
        observed = self._send_edge_and_wait(
            _get_control_cmd(command_name), expected_modes | {26}, timeout_s=3.0)
        if observed == 26:
            return self._protection_error(action, [], observed, command_sent=True)
        return self._debug_mode_result(
            action, observed, observed in expected_modes, True,
            command_name=command_name, expected_modes=expected_modes)

    @staticmethod
    def _debug_mode_result(action: str, observed: int, confirmed: bool,
                           command_sent: bool, command_name: str | None = None,
                           expected_modes: set[int] | None = None) -> dict:
        result = {
            "state": "completed" if confirmed else "accepted",
            "requested_action": action,
            "command_sent": command_sent,
            "confirmed": confirmed,
            "workmode": observed,
            "workmode_name": _WORKMODE_NAMES.get(observed, "unknown"),
            "message": (
                "已确认目标工作模式" if confirmed and command_sent else
                "机器人已经处于目标工作模式，未重复发送命令" if confirmed else
                "命令已发送，但 3 秒内未观察到目标工作模式"
            ),
        }
        if command_name:
            result["command"] = command_name
            result["expected_workmodes"] = sorted(expected_modes or set())
        return result

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
                    "机器人当前动作尚未结束，未继续切换内部模式；请等待动作完成后重试")

        if mode == 30:
            mode = self._run_preparation_step("enable", "START", {0}, steps)
            if mode == 26:
                return self._protection_error(requested_action, steps, mode)
            if mode != 0:
                return self._preparation_error(requested_action, steps, mode, "机器人未能进入使能状态")

        if target_mode == 1 and mode == 2:
            mode = self._run_preparation_step("prepare", "SWITCH", {1}, steps)
        elif mode == 0:
            mode = self._run_preparation_step("prepare", "SWITCH", {1}, steps)

        if mode == 26:
            return self._protection_error(requested_action, steps, mode)
        if target_mode == 1:
            if mode != 1:
                return self._preparation_error(requested_action, steps, mode, "机器人未能进入起身所需的准备状态")
            return {"state": "completed", "steps": steps, "workmode": mode}

        if mode == 1:
            mode = self._run_preparation_step("enter_walking", "WALK", {2}, steps)
        if mode == 26:
            return self._protection_error(requested_action, steps, mode)
        if mode != 2:
            return self._preparation_error(requested_action, steps, mode, "机器人未能进入动作所需的行走状态")
        return {"state": "completed", "steps": steps, "workmode": mode}

    def _run_preparation_step(self, step: str, command_name: str,
                              expected_modes: set[int], steps: list[dict]) -> int:
        observed = self._send_edge_and_wait(
            _get_control_cmd(command_name), expected_modes | {26}, timeout_s=3.0)
        steps.append({
            "step": step,
            "command": command_name,
            "expected_workmodes": sorted(expected_modes),
            "observed_workmode": observed,
            "observed_workmode_name": _WORKMODE_NAMES.get(observed, "unknown"),
            "confirmed": observed in expected_modes,
        })
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
            "pose_verification": "SDK 仅提供 workmode，不能确认机器人真实姿态；姿态和周围环境必须由用户检查",
            "message": (
                "已观察到目标动作模式，动作正在执行；此返回不代表物理动作已经完成"
                if confirmed else
                "命令已发送，但 3 秒内未观察到目标动作模式；请检查机器人实际状态和 motion_state"
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
            "message": "目标动作未发送；请检查机器人姿态、地面和周围空间后重试",
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
            "error": "机器人已进入保护模式，动作无法继续",
            "recovery": "请停止操作并重启机器人；重启前将机器人面朝上平躺于平坦防滑地面，四肢自然放置，脚底无异物，并确保周围至少 3m×3m 无人和障碍物，再执行 stand_up",
        }
        if safety_requirements:
            result["safety_requirements"] = safety_requirements
        return result

    @staticmethod
    def _safety_requirements(action: str) -> str:
        if action == "stand_up":
            return "只能在机器人面朝上平躺、四肢自然放置、双腿伸直、脚底无异物，地面平坦防滑且周围至少 3m×3m 无人无障碍物时使用"
        if action == "lie_down":
            return "只能在机器人正常稳定站立，地面平坦防滑且周围至少 3m×3m 无人无障碍物时使用"
        if action in {"dance_1", "dance_2", "dance_3", "play_recording"}:
            return "只能在机器人正常稳定站立、双脚着地，地面平坦防滑且周围至少 3m×3m 无人无障碍物时使用"
        if action == "start_recording":
            return "确认机器人稳定站立、地面平坦防滑且有人看护；缓慢引导关节，禁止强推、快速扭转或越过机械限位"
        if action == "finish_and_save_recording":
            return "仅在已经调用 start_recording 且示教动作已完成时使用；保存期间不要继续搬动机器人"
        return "只能在机器人正常稳定站立、双脚着地，地面平坦防滑且动作范围内无人和障碍物时使用"

    def _wait_for_workmode(self, expected_modes: set[int], timeout_s: float) -> int:
        deadline = time.monotonic() + timeout_s
        observed = int(self._high_ctrl.get_mode())
        while observed not in expected_modes and observed != 26 and time.monotonic() < deadline:
            time.sleep(0.05)
            observed = int(self._high_ctrl.get_mode())
        return observed

    def _send_edge_and_wait(self, cmd_enum, expected_modes: set[int],
                            index: int = 0, timeout_s: float = 2.0) -> int:
        """Send one event command, release with DEFAULT, then observe feedback."""
        self._publish_cmd(0, 0, 0, cmd_enum, index)
        # The vendor demo runs a 10 ms command loop. This also exceeds the
        # documented minimum 2 ms interval without repeatedly firing the event.
        time.sleep(0.01)
        self._publish_cmd(0, 0, 0, _get_default_cmd(), 0)
        deadline = time.monotonic() + timeout_s
        observed = int(self._high_ctrl.get_mode())
        while observed not in expected_modes and time.monotonic() < deadline:
            time.sleep(0.05)
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
