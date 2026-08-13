#!/usr/bin/env python3
"""
drivers/noetix/bumi/device.py — Noetix Bumi-EDU 设备插件实现。

插件列表：
  - StatePlugin: joints (21-DOF skeleton), imu, battery, model (URDF resource)
  - LocoPlugin: loco (move/stop), switch_mode (mode transitions)
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

_MODE_TO_CMD_NAME = {
    "enable": "START",      # 仅从失能模式进入使能模式
    "disable": "START",     # 从任意非失能模式进入失能模式
    "ready": "SWITCH",      # 准备模式
    "walk": "WALK",
    "swing": "SWING",       # 挥手
    "shake": "SHAKE",       # 握手
    "cheer": "CHEER",       # 欢呼
    "start_teach": "STARTTEACH",
    "save_teach": "SAVETEACH",
    "play_teach": "PLAYTEACH",
    "dance": "DANCE",
    "fall_to_stand": "FALLTOSTAND",
    "stand_to_fall": "STANDTOFALL",
    "dance1": "DANCE1",
    "dance2": "DANCE2",
    "tear": "TEAR",         # 擦眼泪
}

_MODE_EXPECTED_WORKMODES = {
    "enable": {0},
    "disable": {30},
    "ready": {1},
    "walk": {2},
    "swing": {8},
    "shake": {9},
    "cheer": {10},
    "start_teach": {11},
    # SAVETEACH reports exit-teach/save stages according to the vendor docs.
    "save_teach": {12, 14, 29},
    "play_teach": {23},
    "dance": {5},
    "fall_to_stand": {27, 2},
    "stand_to_fall": {28, 30},
    "dance1": {31},
    "dance2": {32},
    "tear": {33},
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
            "description": "Bumi 运控模式管理。切换前检查厂商规定的直接前置状态，但不会替用户自动补齐状态链；前置状态不符时返回安全操作顺序，便于用户先确认机器人姿态和环境。实际发送模式命令前会停止本卡片的速度命令。",
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {
                        "type": "string",
                        "enum": ["switch", "get_mode"],
                        "description": "switch=切换或触发 mode；get_mode=只查询当前模式。enable 和 disable 请在 switch 的 mode 中选择。",
                    },
                    "mode": {
                        "type": "string",
                        "enum": list(_MODE_TO_CMD_NAME.keys()),
                        "description": "仅 switch 使用。enable=从失能进入使能；disable=进入失能；ready=准备；walk=走路；swing=挥手；shake=握手；cheer=欢呼；start_teach/save_teach/play_teach=开始/保存/播放示教；dance/dance1/dance2=三种舞蹈；fall_to_stand=倒地起身；stand_to_fall=起身倒地；tear=擦眼泪。run 当前不可用，end_teach 已弃用。",
                    },
                    "index": {
                        "type": "integer",
                        "description": "仅 save_teach/play_teach 使用的示教文件编号，范围 0～65535；其他模式无需填写。",
                        "minimum": 0, "maximum": 65535,
                    },
                },
                "required": ["action"],
                "x-action-params": {
                    "switch": {
                        "params": ["mode", "index"],
                        "description": "先检查当前 workmode 是否是目标模式的直接前置状态；不符合时不发送命令，而是返回 required_sequence。符合时只发送一次命令并用 DEFAULT 释放。",
                    },
                    "get_mode": {
                        "params": [],
                        "description": "读取当前 workmode 编号、名称以及当前是否处于失能/保护状态。",
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

    def _do_switch(self, args: dict) -> dict:
        mode_str = args.get("mode", "")

        if mode_str not in _MODE_TO_CMD_NAME:
            return {
                "state": "error",
                "error": f"Unknown mode: {mode_str}. Available: {list(_MODE_TO_CMD_NAME.keys())}",
            }

        if mode_str in ("save_teach", "play_teach") and "index" not in args:
            return {"state": "error", "error": f"index is required for {mode_str}"}
        try:
            index = int(args.get("index", 0))
        except (TypeError, ValueError):
            return {"state": "error", "error": "index must be an integer"}
        if not 0 <= index <= 65535:
            return {"state": "error", "error": "index must be in [0, 65535]"}

        current_mode = int(self._high_ctrl.get_mode())
        if mode_str == "disable":
            return self._switch_disable(current_mode)
        if current_mode == 26:
            return {"state": "error", "error": "Robot in protection mode. Cannot switch modes."}
        if mode_str == "enable":
            return self._switch_enable(current_mode)

        persistent_modes = {"ready": 1, "walk": 2}
        if mode_str in persistent_modes and current_mode == persistent_modes[mode_str]:
            return self._already_in_mode(mode_str, current_mode)

        allowed_predecessors = {
            "ready": {0, 2},
            "walk": {1},
            "fall_to_stand": {1},
            "save_teach": {11},
        }
        required_modes = allowed_predecessors.get(mode_str, {2})
        if current_mode not in required_modes:
            return self._prerequisite_error(mode_str, current_mode, required_modes)

        # Prevent the 50 Hz movement publisher from racing with an edge-triggered
        # mode command and overwriting it with DEFAULT. This happens only after
        # the prerequisite check succeeds, so a rejected request changes nothing.
        self._stop_move()

        cmd_enum = _get_control_cmd(_MODE_TO_CMD_NAME[mode_str])
        observed = self._send_edge_and_wait(
            cmd_enum, _MODE_EXPECTED_WORKMODES[mode_str], index=index, timeout_s=3.0)
        transition_steps = [{
            "command": _MODE_TO_CMD_NAME[mode_str],
            "expected_workmodes": sorted(_MODE_EXPECTED_WORKMODES[mode_str]),
            "observed_workmode": observed,
        }]
        confirmed = observed in _MODE_EXPECTED_WORKMODES[mode_str]
        return {
            "state": "completed" if confirmed else "accepted",
            "requested": mode_str,
            "confirmed": confirmed,
            "workmode": observed,
            "workmode_name": _WORKMODE_NAMES.get(observed, "unknown"),
            "transition_steps": transition_steps,
            "message": (
                "target mode was observed"
                if confirmed else
                "command was sent, but the expected mode was not observed before timeout"
            ),
        }

    def _switch_enable(self, current_mode: int) -> dict:
        if current_mode == 0:
            return self._already_in_mode("enable", current_mode)
        if current_mode != 30:
            return self._prerequisite_error("enable", current_mode, {30})
        observed = self._send_edge_and_wait(_get_control_cmd("START"), {0})
        confirmed = observed == 0
        return {
            "state": "completed" if confirmed else "accepted",
            "requested": "enable",
            "confirmed": confirmed,
            "changed": confirmed,
            "workmode": observed,
            "workmode_name": _WORKMODE_NAMES.get(observed, "unknown"),
            "message": "enable confirmed" if confirmed else "START sent but enabled mode was not observed",
        }

    def _switch_disable(self, current_mode: int) -> dict:
        if current_mode == 30:
            return self._already_in_mode("disable", current_mode)
        self._stop_move()
        observed = self._send_edge_and_wait(_get_control_cmd("START"), {30})
        confirmed = observed == 30
        return {
            "state": "completed" if confirmed else "accepted",
            "requested": "disable",
            "confirmed": confirmed,
            "changed": confirmed,
            "workmode": observed,
            "workmode_name": _WORKMODE_NAMES.get(observed, "unknown"),
            "message": "disable confirmed" if confirmed else "START sent but disabled mode was not observed",
        }

    @staticmethod
    def _already_in_mode(requested: str, current_mode: int) -> dict:
        return {
            "state": "completed",
            "requested": requested,
            "confirmed": True,
            "changed": False,
            "workmode": current_mode,
            "workmode_name": _WORKMODE_NAMES.get(current_mode, "unknown"),
            "message": "robot is already in the requested mode; no command was sent",
        }

    @staticmethod
    def _prerequisite_error(requested: str, current_mode: int,
                            required_modes: set[int]) -> dict:
        if requested == "enable":
            sequence = ["disable", "enable"]
        elif requested == "ready":
            sequence = ["enable", "ready"] if current_mode == 30 else ["ready"]
        elif requested == "walk":
            sequence = (
                ["enable", "ready", "walk"] if current_mode == 30 else
                ["ready", "walk"] if current_mode == 0 else
                ["walk"]
            )
        elif requested == "fall_to_stand":
            sequence = (
                ["enable", "ready", "fall_to_stand"] if current_mode == 30 else
                ["ready", "fall_to_stand"]
            )
        elif requested == "save_teach":
            sequence = ["start_teach", "save_teach"]
        else:
            sequence = (
                ["enable", "ready", "walk", requested] if current_mode == 30 else
                ["ready", "walk", requested] if current_mode == 0 else
                ["walk", requested]
            )
        return {
            "state": "error",
            "error": "current workmode does not satisfy the requested mode prerequisite; no command was sent",
            "requested": requested,
            "current_workmode": current_mode,
            "current_workmode_name": _WORKMODE_NAMES.get(current_mode, "unknown"),
            "required_current_workmodes": [
                {"code": mode, "name": _WORKMODE_NAMES.get(mode, "unknown")}
                for mode in sorted(required_modes)
            ],
            "required_sequence": sequence,
            "safety_note": "verify robot pose, support, ground and surrounding clearance before executing each step",
        }

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

    def _do_get_mode(self) -> dict:
        mode = int(self._high_ctrl.get_mode())
        return {
            "workmode": mode,
            "workmode_name": _WORKMODE_NAMES.get(mode, "unknown"),
            "enabled": mode != 30,
            "protection": mode == 26,
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
            "motor_fault_summary": {
                "has_documented_fault": bool(faults),
                "documented_fault_count": len(faults),
                "meaning": "only Noetix-documented motor error codes are classified as faults",
            },
            "joint_field_help": {
                "position": "joint position in radians",
                "velocity": "joint angular velocity in radians per second",
                "torque": "raw joint torque feedback from HighController",
                "temperature": "motor temperature reported by HighController",
                "error": "raw SDK motor status/error value; use fault=true only for a documented fault",
            },
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
