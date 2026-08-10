#!/usr/bin/env python3
"""
xbot/q5/device.py — Q5 轮式人形机器人设备插件。

设计原则：
  - 一个设备 = 一个 tool（或 multi-tool plugin）
  - sensor：只读，驱动启动时自动 start，数据通过 ROS2 topic 输出
  - actuator：单 tool + action 参数分发操作
  - start/stop 不暴露给 LLM，由驱动生命周期管理

ROS2 Domain ID = 211

插件列表：
  StatePlugin        (sensor, multi-tool) — 关节/动态关节/MPC 位姿/机器人状态
  ImuPlugin          (sensor)             — IMU 姿态（从 dynamic_joint_states 提取）
  BatteryPlugin      (sensor)             — 电池状态
  FaultsPlugin       (sensor)             — 故障诊断
  LocoPlugin         (actuator)           — 底盘 TwistStamped 控制
  JointServoPlugin   (actuator)           — 整机关节位置控制 (HybridJointCommand)
  HandPlugin         (actuator)           — 灵巧手控制 (HybridJointCommand)
  HandStatePlugin    (sensor)             — 灵巧手状态
  HeadPlugin         (actuator)           — 头部 2DOF 控制
  HeadGesturePlugin  (actuator)           — 头部语义动作（点头/摇头等）
  ArmPlugin          (actuator)           — 双臂 2×7DOF 控制
  ArmGesturePlugin   (actuator)           — 双臂语义动作（挥手/敬礼等）
  MotionPlugin       (actuator)           — 运动管理（MotionRequest）
  GesturePlugin      (actuator)           — 手势播放控制
  AudioPlugin        (actuator)           — 音频播放（audio_player Action）
  SpeakerPlugin      (actuator)           — 语音播报（AudioPlay mode=2 ITEM）
  LedPlugin          (actuator)           — LED 灯带控制
  NavPlugin          (actuator)           — 导航控制
  TeleopPlugin       (actuator)           — 遥控/手柄控制
  OdomPlugin         (sensor)             — 底盘里程计
  SimpleActionsPlugin (actuator)          — 预定义简单动作 (SimpleActions Action)
  CameraPlugin       (sensor)             — 相机（预留）
"""

import json
import math
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.action import ActionClient
from std_msgs.msg import String, UInt8
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState, JointState, Joy
from control_msgs.msg import DynamicJointState
from xbot_common_interfaces.msg import (
    HybridJointCommand, RobotStatus, MotionStatus,
    ServoPose, ChannelsMsg, HandXd12, Imu, FaultArray,
)
from xbot_common_interfaces.srv import (
    DynamicLaunch, MotionRequest,
    SetVolume, StringMessage,
)
from std_srvs.srv import Trigger
from xbot_common_interfaces.action import (
    AudioPlay, SimpleTrajectory, Behavior, GraspObject, Motion, SimpleActions,
)


_LOW_LAT_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=200,
    durability=DurabilityPolicy.VOLATILE,
)

_RELIABLE_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    durability=DurabilityPolicy.VOLATILE,
)

# ── 关节名称 ──────────────────────────────────────────────────────────────────

# 真机 /joint_states 实际输出 34 个关节，分为：
#   底盘驱动 2 + 腿部 3 + 腰部 1 + 头部 2 + 左臂 7 + 右臂 7 + 左手 6 + 右手 6 = 34
# 注意：关节名与手册不同，以真机为准
_JOINTS_WR1 = [
    # 底盘驱动轮 2
    "left_drv_wheel_joint",
    "right_drv_wheel_joint",
    # 腿部 3 (Q5 轮式人形，hip + knee + ankle)
    "hip_joint",
    "knee_joint",
    "ankle_joint",
    # 腰部 1
    "waist_yaw_joint",
    # 头部 2
    "neck_pitch_joint",
    "neck_yaw_joint",
    # 左臂 7
    "left_shoulder_pitch_joint",
    "left_shoulder_roll_joint",
    "left_arm_yaw_joint",
    "left_elbow_pitch_joint",
    "left_elbow_yaw_joint",
    "left_wrist_pitch_joint",
    "left_wrist_roll_joint",
    # 右臂 7
    "right_shoulder_pitch_joint",
    "right_shoulder_roll_joint",
    "right_arm_yaw_joint",
    "right_elbow_pitch_joint",
    "right_elbow_yaw_joint",
    "right_wrist_pitch_joint",
    "right_wrist_roll_joint",
]

# 灵巧手关节（真机实际名称，每只手 6 个关节）
_JOINTS_HAND_L = [
    "left_hand_thumb_bend_joint",
    "left_hand_thumb_rota_joint1",
    "left_hand_index_joint1",
    "left_hand_mid_joint1",
    "left_hand_ring_joint1",
    "left_hand_pinky_joint1",
]

_JOINTS_HAND_R = [
    "right_hand_thumb_bend_joint",
    "right_hand_thumb_rota_joint1",
    "right_hand_index_joint1",
    "right_hand_mid_joint1",
    "right_hand_ring_joint1",
    "right_hand_pinky_joint1",
]

_JOINTS_HAND = _JOINTS_HAND_L + _JOINTS_HAND_R

# 双臂目标位姿坐标系
_SERVO_FRAME = "base_link"


# ── 工具函数 ──────────────────────────────────────────────────────────────────

def _deg2rad(deg: float) -> float:
    return deg * math.pi / 180.0


def _rad2deg(rad: float) -> float:
    return rad * 180.0 / math.pi


def _clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, float(value)))


def _make_hybrid_command(joint_names: list, positions: list,
                         velocities: list | None = None,
                         kps: list | None = None,
                         kds: list | None = None,
                         node: Node = None) -> HybridJointCommand:
    """Build a HybridJointCommand message."""
    msg = HybridJointCommand()
    if node is not None:
        msg.header.stamp = node.get_clock().now().to_msg()
    msg.joint_name = list(joint_names)
    msg.position = list(positions)
    msg.velocity = velocities or [0.0] * len(positions)
    msg.feedforward = [0.0] * len(positions)
    msg.kp = kps or [0.0] * len(positions)
    msg.kd = kds or [0.0] * len(positions)
    return msg


# ── 基类 ──────────────────────────────────────────────────────────────────────

class _Q5Node(Node):
    """Base node for Q5 plugins."""

    def __init__(self, name: str):
        super().__init__(f"q5_{name}")
        self.state = "idle"


# ═══════════════════════════════════════════════════════════════════════════════
# Sensor Plugins
# ═══════════════════════════════════════════════════════════════════════════════

# ── StatePlugin (multi-tool: state, joint_state, servo_pose) ──────────────────

class StatePlugin:
    """整机状态传感器 — joint_states / dynamic_joint_states / servo_pose / xbot_state."""

    def __init__(self, plugin_config: dict, namespace: str, executor):
        self._ns = namespace
        self._node = _Q5Node("state")
        executor.add_node(self._node)

        # Publishers
        self._pub_joint = self._node.create_publisher(JointState, f"/{self._ns}/joint_states", _RELIABLE_QOS)
        self._pub_dynjoint = self._node.create_publisher(DynamicJointState, f"/{self._ns}/dynamic_joint_states", _RELIABLE_QOS)
        self._pub_servo = self._node.create_publisher(ServoPose, f"/{self._ns}/servo_pose", _RELIABLE_QOS)
        self._pub_robot_status = self._node.create_publisher(RobotStatus, f"/{self._ns}/robot_status", _RELIABLE_QOS)

        # Subscribers
        self._sub_joint = self._node.create_subscription(
            JointState, "/joint_states", lambda m: self._pub_joint.publish(m), _RELIABLE_QOS)
        self._sub_dynjoint = self._node.create_subscription(
            DynamicJointState, "/dynamic_joint_states", lambda m: self._pub_dynjoint.publish(m), _RELIABLE_QOS)
        self._sub_servo = self._node.create_subscription(
            ServoPose, "/servo_poses", lambda m: self._pub_servo.publish(m), _RELIABLE_QOS)
        self._sub_status = self._node.create_subscription(
            RobotStatus, "/xbot_state", self._on_robot_status, _RELIABLE_QOS)

        # /get_servo_poses service is not available on the real machine (empty output from ros2 service type).
        # Servo pose data is obtained from /servo_poses topic subscription instead.
        self._robot_status = None
        self._state = "idle"

    def get_tool(self) -> dict:
        return {
            "name": "state",
            "type": "sensor",
            "multiInstance": False,
            "description": "Q5 robot state — joint_states, dynamic_joint_states, servo_pose, robot_status published on ROS2 topics",
            "inputSchema": {"type": "object", "properties": {
                "action": {"type": "string", "enum": ["start", "stop", "info"], "default": "info"},
            }},
            "default_action": "info",
        }

    def start(self) -> None:
        self._state = "active"

    def stop(self) -> None:
        self._state = "idle"

    def dispatch(self, action: str, args: dict) -> dict | None:
        if action in ("start", "info"):
            return {
                "state": "active",
                "topics": {
                    "joint_states": f"/{self._ns}/joint_states",
                    "dynamic_joint_states": f"/{self._ns}/dynamic_joint_states",
                    "servo_pose": f"/{self._ns}/servo_pose",
                    "robot_status": f"/{self._ns}/robot_status",
                },
            }
        return None

    def _on_robot_status(self, msg: RobotStatus):
        self._robot_status = msg


# ── ImuPlugin (sensor) ───────────────────────────────────────────────────────

class ImuPlugin:
    """IMU 姿态数据 — xbot_common_interfaces/msg/Imu。

    真机确认：ros2 topic list -t | grep -i imu 无输出，
    说明真机上没有独立的 IMU topic。Imu 数据可能通过其他渠道获取
    （如 dynamic_joint_states 或内部 SDK）。此插件订阅 /imu 作为预留，
    真机上可能收不到数据。
    """

    def __init__(self, plugin_config: dict, namespace: str, executor):
        self._ns = namespace
        self._node = _Q5Node("imu")
        executor.add_node(self._node)

        self._pub = self._node.create_publisher(Imu, f"/{self._ns}/imu/data", _LOW_LAT_QOS)
        # 真机上未发现 IMU topic，订阅 /imu 作为预留
        self._sub = self._node.create_subscription(
            Imu, "/imu", self._on_imu, _LOW_LAT_QOS)
        self._state = "idle"
        self._last_data = None

    def get_tool(self) -> dict:
        return {
            "name": "imu",
            "type": "sensor",
            "multiInstance": False,
            "description": "Q5 IMU — angular velocity, linear acceleration, orientation (roll/pitch/yaw)",
            "inputSchema": {"type": "object", "properties": {
                "action": {"type": "string", "enum": ["start", "stop", "info"], "default": "info"},
            }},
            "default_action": "info",
        }

    def start(self) -> None:
        self._state = "active"

    def stop(self) -> None:
        self._state = "idle"

    def dispatch(self, action: str, args: dict) -> dict | None:
        if action in ("start", "info"):
            info = {"state": "active", "topic": f"/{self._ns}/imu/data"}
            if self._last_data:
                info["data"] = self._last_data
            return info
        if action == "stop":
            return {"state": "idle"}
        return None

    def _on_imu(self, msg: Imu):
        data = {
            "angular_vel": {"x": msg.angular_vel_x, "y": msg.angular_vel_y, "z": msg.angular_vel_z},
            "linear_acc": {"x": msg.linear_acc_x, "y": msg.linear_acc_y, "z": msg.linear_acc_z},
            "orientation": {"w": msg.orientation_w, "x": msg.orientation_x,
                            "y": msg.orientation_y, "z": msg.orientation_z},
            "euler": {"roll": msg.roll, "pitch": msg.pitch, "yaw": msg.yaw},
        }
        self._last_data = data
        self._pub.publish(msg)


# ── BatteryPlugin (sensor) ───────────────────────────────────────────────────

class BatteryPlugin:
    """电池状态 — /battery_state (BatteryState)。"""

    def __init__(self, plugin_config: dict, namespace: str, executor):
        self._ns = namespace
        self._node = _Q5Node("battery")
        executor.add_node(self._node)

        self._pub = self._node.create_publisher(BatteryState, f"/{self._ns}/battery_state", _RELIABLE_QOS)
        self._sub = self._node.create_subscription(
            BatteryState, "/battery_state", self._on_battery, _RELIABLE_QOS)

        self._state = "idle"
        self._battery_info = None

    def get_tool(self) -> dict:
        return {
            "name": "battery",
            "type": "sensor",
            "multiInstance": False,
            "description": "Q5 battery state — voltage, current, temperature, SOC, status",
            "inputSchema": {"type": "object", "properties": {
                "action": {"type": "string", "enum": ["start", "stop", "info"], "default": "info"},
            }},
            "default_action": "info",
        }

    def start(self) -> None:
        self._state = "active"

    def stop(self) -> None:
        self._state = "idle"

    def dispatch(self, action: str, args: dict) -> dict | None:
        if action in ("start", "info"):
            info = {"state": "active", "topic": f"/{self._ns}/battery_state"}
            if self._battery_info:
                info.update({
                    "voltage": self._battery_info.voltage,
                    "current": self._battery_info.current,
                    "percentage": self._battery_info.percentage,
                    "temperature": self._battery_info.temperature,
                    "power_supply_status": self._battery_info.power_supply_status,
                })
            return info
        if action == "stop":
            return {"state": "idle"}
        return None

    def _on_battery(self, msg: BatteryState):
        self._battery_info = msg


# ── FaultsPlugin (sensor) ────────────────────────────────────────────────────

class FaultsPlugin:
    """故障诊断 — 订阅 /fault_array (FaultArray) 和 /fault_aggregator/highest_level (UInt8)。

    真机上 /fault_array 和 /fault_array_agg 均为 xbot_common_interfaces/msg/FaultArray 类型。
    /fault_aggregator/highest_level 是 UInt8 类型，作为故障等级指示。
    """

    def __init__(self, plugin_config: dict, namespace: str, executor):
        self._ns = namespace
        self._node = _Q5Node("faults")
        executor.add_node(self._node)

        self._pub = self._node.create_publisher(String, f"/{self._ns}/faults", _LOW_LAT_QOS)
        self._sub = self._node.create_subscription(
            FaultArray, "/fault_array", self._on_faults, _LOW_LAT_QOS)
        # /fault_aggregator/highest_level is UInt8 (confirmed on real machine)
        self._sub_level = self._node.create_subscription(
            UInt8, "/fault_aggregator/highest_level", self._on_fault_level, _LOW_LAT_QOS)

        self._state = "idle"
        self._faults = []
        self._highest_level = 0

    def get_tool(self) -> dict:
        return {
            "name": "faults",
            "type": "sensor",
            "multiInstance": False,
            "description": "Q5 fault diagnostics — highest fault level and fault array",
            "inputSchema": {"type": "object", "properties": {
                "action": {"type": "string", "enum": ["start", "stop", "info"], "default": "info"},
            }},
            "default_action": "info",
        }

    def start(self) -> None:
        self._state = "active"

    def stop(self) -> None:
        self._state = "idle"

    def dispatch(self, action: str, args: dict) -> dict | None:
        if action in ("start", "info"):
            return {
                "state": "active" if self._highest_level == 0 else "warning",
                "topic": f"/{self._ns}/faults",
                "highest_level": self._highest_level,
                "fault_count": len(self._faults),
                "faults": self._faults,
            }
        if action == "stop":
            return {"state": "idle"}
        return None

    def _on_faults(self, msg: FaultArray):
        faults = []
        for f in msg.faults:
            faults.append({
                "fault_code": f.fault_code,
                "name": f.name,
                "level": f.level,
                "message": f.message,
                "joint_name": f.joint_name,
                "fault_type": f.fault_type,
                "is_active": f.is_active,
            })
        self._faults = faults
        self._pub.publish(String(data=json.dumps({"fault_count": len(faults), "faults": faults})))

    def _on_fault_level(self, msg: UInt8):
        self._highest_level = msg.data


# ── HandStatePlugin (sensor) ─────────────────────────────────────────────────

class HandStatePlugin:
    """灵巧手状态 — /hand_sensor (HandXd12)。

    HandXd12 字段为 float32 数组：拇指/食指 3 个关节，中指/无名指/小指各 2 个关节：
      lefthumb[3], leftindex[3], leftmid[2], leftring[2], leftpinky[2],
      righthumb[3], rightindex[3], rightmid[2], rightring[2], rightpinky[2]
    """

    def __init__(self, plugin_config: dict, namespace: str, executor):
        self._ns = namespace
        self._node = _Q5Node("hand_state")
        executor.add_node(self._node)

        self._pub = self._node.create_publisher(String, f"/{self._ns}/hand_sensor", _LOW_LAT_QOS)
        self._sub = self._node.create_subscription(
            HandXd12, "/hand_sensor", self._on_hand_sensor, _LOW_LAT_QOS)

        self._state = "idle"
        self._last_data = None

    def get_tool(self) -> dict:
        return {
            "name": "hand_state",
            "type": "sensor",
            "multiInstance": False,
            "description": "Q5 hand sensor — 10 finger joint position arrays via HandXd12 "
                           "(thumb/index: 3 joints, mid/ring/pinky: 2 joints each)",
            "inputSchema": {"type": "object", "properties": {
                "action": {"type": "string", "enum": ["start", "stop", "info"], "default": "info"},
            }},
            "default_action": "info",
        }

    def start(self) -> None:
        self._state = "active"

    def stop(self) -> None:
        self._state = "idle"

    def dispatch(self, action: str, args: dict) -> dict | None:
        if action in ("start", "info"):
            info = {"state": "active", "topic": f"/{self._ns}/hand_sensor"}
            if self._last_data:
                info["data"] = self._last_data
            return info
        if action == "stop":
            return {"state": "idle"}
        return None

    def _on_hand_sensor(self, msg: HandXd12):
        # HandXd12 fields are float32 arrays: thumb/index have 3 joints, mid/ring/pinky have 2
        data = {
            "left": {
                "thumb": list(msg.lefthumb),
                "index": list(msg.leftindex),
                "mid": list(msg.leftmid),
                "pinky": list(msg.leftpinky),
                "ring": list(msg.leftring),
            },
            "right": {
                "thumb": list(msg.righthumb),
                "index": list(msg.rightindex),
                "mid": list(msg.rightmid),
                "pinky": list(msg.rightpinky),
                "ring": list(msg.rightring),
            },
        }
        self._last_data = data
        self._pub.publish(String(data=json.dumps(data)))


# ── OdomPlugin (sensor) ──────────────────────────────────────────────────────

class OdomPlugin:
    """底盘里程计 — /wr1_base_drive_controller/odom。"""

    def __init__(self, plugin_config: dict, namespace: str, executor):
        self._ns = namespace
        self._node = _Q5Node("odom")
        executor.add_node(self._node)

        self._pub = self._node.create_publisher(Odometry, f"/{self._ns}/odom", _RELIABLE_QOS)
        self._sub = self._node.create_subscription(
            Odometry, "/wr1_base_drive_controller/odom",
            lambda m: self._pub.publish(m), _RELIABLE_QOS)

        self._state = "idle"

    def get_tool(self) -> dict:
        return {
            "name": "odom",
            "type": "sensor",
            "multiInstance": False,
            "description": "Q5 chassis odometry — position, velocity, attitude from wheel encoders",
            "inputSchema": {"type": "object", "properties": {
                "action": {"type": "string", "enum": ["start", "stop", "info"], "default": "info"},
            }},
            "default_action": "info",
        }

    def start(self) -> None:
        self._state = "active"

    def stop(self) -> None:
        self._state = "idle"

    def dispatch(self, action: str, args: dict) -> dict | None:
        if action in ("start", "info"):
            return {"state": "active", "topic": f"/{self._ns}/odom"}
        if action == "stop":
            return {"state": "idle"}
        return None


# ═══════════════════════════════════════════════════════════════════════════════
# Actuator Plugins
# ═══════════════════════════════════════════════════════════════════════════════

# ── LocoPlugin (actuator) — chassis cmd_vel ──────────────────────────────────

class LocoPlugin:
    """底盘运动控制 — TwistStamped on /wr1_base_drive_controller/cmd_vel。"""

    def __init__(self, plugin_config: dict, namespace: str, executor):
        self._ns = namespace
        self._node = _Q5Node("loco")
        executor.add_node(self._node)

        self._pub = self._node.create_publisher(
            TwistStamped, "/wr1_base_drive_controller/cmd_vel", _RELIABLE_QOS)

        self._state = "idle"

    def get_tool(self) -> dict:
        return {
            "name": "loco",
            "type": "actuator",
            "multiInstance": False,
            "description": "Q5 chassis motion control — velocity commands (vx, vyaw). TwistStamped: linear.x = forward speed (m/s), angular.z = rotation speed (rad/s).",
            "inputSchema": {"type": "object", "properties": {
                "action": {"type": "string", "enum": ["start", "stop", "move", "stop_move", "info"], "default": "move"},
                "vx": {"type": "number", "description": "Forward velocity (m/s), range [-0.5, 0.5]"},
                "vyaw": {"type": "number", "description": "Angular velocity (rad/s), range [-1.0, 1.0]"},
                "duration": {"type": "number", "description": "Move duration in seconds (0 = continuous)"},
            }},
            "default_action": "move",
        }

    def start(self) -> None:
        self._state = "active"

    def stop(self) -> None:
        self._publish_stop()
        self._state = "idle"

    def dispatch(self, action: str, args: dict) -> dict | None:
        if action == "move":
            vx = _clamp(args.get("vx", 0.0), -0.5, 0.5)
            vyaw = _clamp(args.get("vyaw", 0.0), -1.0, 1.0)
            duration = args.get("duration", 0)
            self._publish_velocity(vx, vyaw)
            if duration > 0:
                threading.Thread(target=self._stop_after, args=(duration,), daemon=True).start()
            return {"state": "ok", "vx": vx, "vyaw": vyaw, "duration": duration}
        if action == "stop_move":
            self._publish_stop()
            return {"state": "ok", "message": "chassis stopped"}
        if action in ("start", "info"):
            return {"state": "active", "topic": "/wr1_base_drive_controller/cmd_vel"}
        if action == "stop":
            return {"state": "idle"}
        return None

    def _publish_velocity(self, vx: float, vyaw: float):
        msg = TwistStamped()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.twist.linear.x = vx
        msg.twist.angular.z = vyaw
        self._pub.publish(msg)

    def _publish_stop(self):
        self._publish_velocity(0.0, 0.0)

    def _stop_after(self, duration: float):
        time.sleep(duration)
        self._publish_stop()


# ── JointServoPlugin (actuator) — joint position control ─────────────────────

class JointServoPlugin:
    """关节伺服控制 — HybridJointCommand on /wr1_controller/commands。

    支持两种模式：
      - "joint" 模式：直接控制单个/多个关节位置（弧度）
      - "servo" 模式：通过 ServoPose 控制双臂+头部目标位姿
    """

    def __init__(self, plugin_config: dict, namespace: str, executor):
        self._ns = namespace
        self._node = _Q5Node("joint_servo")
        executor.add_node(self._node)

        self._pub = self._node.create_publisher(
            HybridJointCommand, "/wr1_controller/commands", _RELIABLE_QOS)

        # Service client for startup chain
        self._srv_dynamic_launch = self._node.create_client(DynamicLaunch, "/dynamic_launch")
        self._srv_ready = self._node.create_client(Trigger, "/ready_service")
        self._srv_activate = self._node.create_client(Trigger, "/activate_service")
        self._srv_deactivate = self._node.create_client(Trigger, "/deactivate_service")

        self._state = "idle"
        self._initialized = False

    def get_tool(self) -> dict:
        return {
            "name": "joint_servo",
            "type": "actuator",
            "multiInstance": False,
            "description": "Q5 joint servo control — position control for 22 main joints via HybridJointCommand. Also supports ServoPose-based arm+head pose control.",
            "inputSchema": {"type": "object", "properties": {
                "action": {"type": "string", "enum": ["start", "stop", "set_joint", "set_poses", "home", "info"], "default": "set_joint"},
                "joint_name": {"type": "string", "description": "Joint name (joint mode)"},
                "position": {"type": "number", "description": "Target position in radians (joint mode)"},
                "left_pose": {"type": "object", "description": "Left arm pose {x, y, z, qx, qy, qz, qw} (servo mode)"},
                "right_pose": {"type": "object", "description": "Right arm pose {x, y, z, qx, qy, qz, qw} (servo mode)"},
                "head_pose": {"type": "object", "description": "Head pose {x, y, z, qx, qy, qz, qw} (servo mode)"},
                "duration": {"type": "number", "description": "Move duration in seconds"},
            }},
            "default_action": "set_joint",
        }

    def start(self) -> None:
        self._state = "active"
        self._ensure_ready()

    def stop(self) -> None:
        self._publish_zero()
        self._deactivate()
        self._state = "idle"

    def dispatch(self, action: str, args: dict) -> dict | None:
        if action == "set_joint":
            self._ensure_ready()
            name = args.get("joint_name")
            pos = args.get("position", 0.0)
            if not name:
                return {"state": "error", "message": "joint_name required"}
            if name not in _JOINTS_WR1:
                return {"state": "error", "message": f"Unknown joint: {name}. Valid: {_JOINTS_WR1}"}
            pos = _clamp(pos, -6.28, 6.28)
            cmd = _make_hybrid_command([name], [pos], kps=[500.0], kds=[5.0])
            cmd.header.stamp = self._node.get_clock().now().to_msg()
            self._pub.publish(cmd)
            return {"state": "ok", "joint": name, "position": pos}
        if action == "set_poses":
            self._ensure_ready()
            return self._set_poses(args)
        if action == "home":
            self._ensure_ready()
            return self._home()
        if action in ("start", "info"):
            return {"state": "active", "topic": "/wr1_controller/commands"}
        if action == "stop":
            return {"state": "idle"}
        return None

    def _ensure_ready(self):
        """Execute the startup chain: dynamic_launch → ready → activate."""
        if self._initialized:
            return
        try:
            # dynamic_launch: app_name (留空), sync_control, launch_mode
            # 本机带 XHand 灵巧手 → launch_mode='pos'；无手机型用 'no_hand_pos'
            if self._srv_dynamic_launch.is_ready():
                req = DynamicLaunch.Request()
                req.app_name = ""
                req.sync_control = True
                req.launch_mode = "pos"
                future = self._srv_dynamic_launch.call_async(req)
                rclpy.spin_until_future_complete(self._node, future, timeout_sec=5.0)
                self._node.get_logger().info(f"dynamic_launch done")
        except Exception as e:
            self._node.get_logger().warn(f"dynamic_launch failed: {e}")

        try:
            if self._srv_ready.is_ready():
                req = Trigger.Request()
                future = self._srv_ready.call_async(req)
                rclpy.spin_until_future_complete(self._node, future, timeout_sec=5.0)
                self._node.get_logger().info(f"ready_service: {future.result().success}")
        except Exception as e:
            self._node.get_logger().warn(f"ready_service failed: {e}")

        try:
            if self._srv_activate.is_ready():
                req = Trigger.Request()
                future = self._srv_activate.call_async(req)
                rclpy.spin_until_future_complete(self._node, future, timeout_sec=5.0)
                self._node.get_logger().info(f"activate_service: {future.result().success}")
        except Exception as e:
            self._node.get_logger().warn(f"activate_service failed: {e}")

        self._initialized = True

    def _publish_zero(self):
        """Publish zero command to stop all joints."""
        cmd = _make_hybrid_command(_JOINTS_WR1, [0.0] * len(_JOINTS_WR1),
                                    kps=[0.0] * len(_JOINTS_WR1),
                                    kds=[0.0] * len(_JOINTS_WR1))
        cmd.header.stamp = self._node.get_clock().now().to_msg()
        self._pub.publish(cmd)

    def _deactivate(self):
        """Call /deactivate_service for clean shutdown."""
        try:
            if self._srv_deactivate.is_ready():
                req = Trigger.Request()
                future = self._srv_deactivate.call_async(req)
                rclpy.spin_until_future_complete(self._node, future, timeout_sec=3.0)
                self._node.get_logger().info(f"deactivate_service: {future.result().success}")
        except Exception as e:
            self._node.get_logger().warn(f"deactivate_service failed: {e}")

    def _set_poses(self, args: dict) -> dict:
        """Set arm/head target poses via ServoPose message on /servo_poses."""
        pose_msg = ServoPose()
        # ServoPose has NO top-level header; each pose (left/right/head) has its own header
        stamp = self._node.get_clock().now().to_msg()

        for side in ("left", "right", "head"):
            pose_key = f"{side}_pose"
            if pose_key in args:
                p = args[pose_key]
                if side == "left":
                    pose_msg.left_pose.header.stamp = stamp
                    pose_msg.left_pose.header.frame_id = p.get("frame_id", _SERVO_FRAME)
                    pose_msg.left_pose.position.x = _clamp(p.get("x", 0.0), -1.0, 1.0)
                    pose_msg.left_pose.position.y = _clamp(p.get("y", 0.0), -1.0, 1.0)
                    pose_msg.left_pose.position.z = _clamp(p.get("z", 0.0), -1.0, 1.0)
                    pose_msg.left_pose.orientation.x = _clamp(p.get("qx", 0.0), -1.0, 1.0)
                    pose_msg.left_pose.orientation.y = _clamp(p.get("qy", 0.0), -1.0, 1.0)
                    pose_msg.left_pose.orientation.z = _clamp(p.get("qz", 0.0), -1.0, 1.0)
                    pose_msg.left_pose.orientation.w = _clamp(p.get("qw", 1.0), -1.0, 1.0)
                elif side == "right":
                    pose_msg.right_pose.header.stamp = stamp
                    pose_msg.right_pose.header.frame_id = p.get("frame_id", _SERVO_FRAME)
                    pose_msg.right_pose.position.x = _clamp(p.get("x", 0.0), -1.0, 1.0)
                    pose_msg.right_pose.position.y = _clamp(p.get("y", 0.0), -1.0, 1.0)
                    pose_msg.right_pose.position.z = _clamp(p.get("z", 0.0), -1.0, 1.0)
                    pose_msg.right_pose.orientation.x = _clamp(p.get("qx", 0.0), -1.0, 1.0)
                    pose_msg.right_pose.orientation.y = _clamp(p.get("qy", 0.0), -1.0, 1.0)
                    pose_msg.right_pose.orientation.z = _clamp(p.get("qz", 0.0), -1.0, 1.0)
                    pose_msg.right_pose.orientation.w = _clamp(p.get("qw", 1.0), -1.0, 1.0)
                elif side == "head":
                    pose_msg.head_pose.header.stamp = stamp
                    pose_msg.head_pose.header.frame_id = p.get("frame_id", _SERVO_FRAME)
                    pose_msg.head_pose.position.x = _clamp(p.get("x", 0.0), -1.0, 1.0)
                    pose_msg.head_pose.position.y = _clamp(p.get("y", 0.0), -1.0, 1.0)
                    pose_msg.head_pose.position.z = _clamp(p.get("z", 0.0), -1.0, 1.0)
                    pose_msg.head_pose.orientation.x = _clamp(p.get("qx", 0.0), -1.0, 1.0)
                    pose_msg.head_pose.orientation.y = _clamp(p.get("qy", 0.0), -1.0, 1.0)
                    pose_msg.head_pose.orientation.z = _clamp(p.get("qz", 0.0), -1.0, 1.0)
                    pose_msg.head_pose.orientation.w = _clamp(p.get("qw", 1.0), -1.0, 1.0)

        # Publish ServoPose on /servo_poses (not on /wr1_controller/commands)
        if not hasattr(self, '_pub_servo'):
            self._pub_servo = self._node.create_publisher(ServoPose, "/servo_poses", _RELIABLE_QOS)
        self._pub_servo.publish(pose_msg)
        return {"state": "ok", "poses": list(args.keys())}

    def _home(self) -> dict:
        """Return all joints to home position (0)."""
        self._ensure_ready()
        self._publish_zero()
        return {"state": "ok", "message": "joints homed to zero"}


# ── HandPlugin (actuator) — hand control ──────────────────────────────────────

class HandPlugin:
    """灵巧手控制 — HybridJointCommand on /hand_controller/commands。

    真机有左右各 6 个手部关节，共 12 DoF。
    通过 hand 参数控制单手，或不指定则控制双手。
    """

    def __init__(self, plugin_config: dict, namespace: str, executor):
        self._ns = namespace
        self._node = _Q5Node("hand")
        executor.add_node(self._node)

        self._pub = self._node.create_publisher(
            HybridJointCommand, "/hand_controller/commands", _RELIABLE_QOS)

        self._joint_names_left = _JOINTS_HAND_L
        self._joint_names_right = _JOINTS_HAND_R
        self._joint_names = _JOINTS_HAND  # all 12
        self._doF = len(_JOINTS_HAND)

        self._state = "idle"

    def get_tool(self) -> dict:
        return {
            "name": "hand",
            "type": "actuator",
            "multiInstance": False,
            "description": f"Q5 dexterous hand — {self._doF} DoF (6 left + 6 right) via HybridJointCommand. "
                           f"Joints: {self._joint_names}",
            "inputSchema": {"type": "object", "properties": {
                "action": {"type": "string", "enum": ["start", "stop", "set_joint", "grip", "open", "info"], "default": "set_joint"},
                "joint_name": {"type": "string", "description": "Hand joint name"},
                "position": {"type": "number", "description": "Target position (radians)"},
                "hand": {"type": "string", "enum": ["left", "right"], "description": "Which hand"},
            }},
            "default_action": "set_joint",
        }

    def start(self) -> None:
        self._state = "active"

    def stop(self) -> None:
        self._publish_zero()
        self._state = "idle"

    def dispatch(self, action: str, args: dict) -> dict | None:
        if action == "set_joint":
            name = args.get("joint_name")
            pos = _clamp(args.get("position", 0.0), -1.57, 1.57)
            if name not in self._joint_names:
                return {"state": "error", "message": f"Unknown hand joint: {name}. Valid: {self._joint_names}"}
            cmd = _make_hybrid_command([name], [pos], kps=[200.0], kds=[2.0])
            cmd.header.stamp = self._node.get_clock().now().to_msg()
            self._pub.publish(cmd)
            return {"state": "ok", "joint": name, "position": pos}
        if action == "grip":
            return self._grip(close=True, hand=args.get("hand"))
        if action == "open":
            return self._grip(close=False, hand=args.get("hand"))
        if action in ("start", "info"):
            return {"state": "active", "doF": self._doF,
                    "joints_left": self._joint_names_left,
                    "joints_right": self._joint_names_right}
        if action == "stop":
            return {"state": "idle"}
        return None

    def _grip(self, close: bool, hand: str | None = None) -> dict:
        """Close or open hand fingers. If hand specified, only that hand; otherwise both."""
        target = -0.3 if close else 0.0
        names = []
        positions = []
        if hand == "left":
            joint_list = self._joint_names_left
        elif hand == "right":
            joint_list = self._joint_names_right
        else:
            joint_list = self._joint_names
        for name in joint_list:
            if "bend" in name or "joint1" in name:
                names.append(name)
                positions.append(target)
        cmd = _make_hybrid_command(names, positions, kps=[200.0] * len(names), kds=[2.0] * len(names))
        cmd.header.stamp = self._node.get_clock().now().to_msg()
        self._pub.publish(cmd)
        return {"state": "ok", "action": "grip" if close else "open", "joints": len(names)}

    def _publish_zero(self):
        cmd = _make_hybrid_command(self._joint_names, [0.0] * self._doF,
                                    kps=[0.0] * self._doF, kds=[0.0] * self._doF)
        cmd.header.stamp = self._node.get_clock().now().to_msg()
        self._pub.publish(cmd)


# ── HeadPlugin (actuator) — head 2DOF ────────────────────────────────────────

class HeadPlugin:
    """头部控制 — 通过 HybridJointCommand 控制 neck_pitch_joint / neck_yaw_joint。"""

    def __init__(self, plugin_config: dict, namespace: str, executor):
        self._ns = namespace
        self._node = _Q5Node("head")
        executor.add_node(self._node)

        self._pub = self._node.create_publisher(
            HybridJointCommand, "/wr1_controller/commands", _RELIABLE_QOS)

        self._state = "idle"

    def get_tool(self) -> dict:
        return {
            "name": "head",
            "type": "actuator",
            "multiInstance": False,
            "description": "Q5 head control — 2 DoF (pitch, yaw). Position in radians.",
            "inputSchema": {"type": "object", "properties": {
                "action": {"type": "string", "enum": ["start", "stop", "set", "look_at", "info"], "default": "set"},
                "joint_name": {"type": "string", "enum": ["neck_pitch_joint", "neck_yaw_joint"]},
                "position": {"type": "number", "description": "Target position (radians)"},
                "angle": {"type": "number", "description": "Angle in degrees (look_at mode)"},
            }},
            "default_action": "set",
        }

    def start(self) -> None:
        self._state = "active"

    def stop(self) -> None:
        self._publish_zero()
        self._state = "idle"

    def dispatch(self, action: str, args: dict) -> dict | None:
        if action == "set":
            name = args.get("joint_name", "neck_pitch_joint")
            if name not in ("neck_pitch_joint", "neck_yaw_joint"):
                return {"state": "error", "message": f"Unknown head joint: {name}"}
            pos = _clamp(args.get("position", 0.0), -0.52, 0.52)  # ~±30°
            cmd = _make_hybrid_command([name], [pos], kps=[500.0], kds=[5.0])
            cmd.header.stamp = self._node.get_clock().now().to_msg()
            self._pub.publish(cmd)
            return {"state": "ok", "joint": name, "position": pos}
        if action == "look_at":
            return self._look_at(args)
        if action in ("start", "info"):
            return {"state": "active", "joints": ["neck_pitch_joint", "neck_yaw_joint"]}
        if action == "stop":
            return {"state": "idle"}
        return None

    def _look_at(self, args: dict) -> dict:
        """Look at angle in degrees (yaw)."""
        angle_deg = _clamp(args.get("angle", 0.0), -45, 45)
        angle_rad = _deg2rad(angle_deg)
        cmd = _make_hybrid_command(["neck_yaw_joint"], [angle_rad], kps=[500.0], kds=[5.0])
        cmd.header.stamp = self._node.get_clock().now().to_msg()
        self._pub.publish(cmd)
        return {"state": "ok", "angle_deg": angle_deg}

    def _publish_zero(self):
        cmd = _make_hybrid_command(["neck_pitch_joint", "neck_yaw_joint"], [0.0, 0.0],
                                    kps=[0.0, 0.0], kds=[0.0, 0.0])
        cmd.header.stamp = self._node.get_clock().now().to_msg()
        self._pub.publish(cmd)


# ── ArmPlugin (actuator) — arm 2×7DOF via ServoPose ──────────────────────────

class ArmPlugin:
    """双臂控制 — 通过 ServoPose 设置左右臂目标位姿。"""

    def __init__(self, plugin_config: dict, namespace: str, executor):
        self._ns = namespace
        self._node = _Q5Node("arm")
        executor.add_node(self._node)

        self._pub = self._node.create_publisher(
            ServoPose, "/servo_poses", _RELIABLE_QOS)

        self._state = "idle"

    def get_tool(self) -> dict:
        return {
            "name": "arm",
            "type": "actuator",
            "multiInstance": False,
            "description": "Q5 arm control — 2×7 DoF via ServoPose. Set left/right arm end-effector poses.",
            "inputSchema": {"type": "object", "properties": {
                "action": {"type": "string", "enum": ["start", "stop", "set_pose", "home", "info"], "default": "set_pose"},
                "arm": {"type": "string", "enum": ["left", "right"]},
                "x": {"type": "number"}, "y": {"type": "number"}, "z": {"type": "number"},
                "qx": {"type": "number"}, "qy": {"type": "number"}, "qz": {"type": "number"}, "qw": {"type": "number"},
            }},
            "default_action": "set_pose",
        }

    def start(self) -> None:
        self._state = "active"

    def stop(self) -> None:
        self._state = "idle"

    def dispatch(self, action: str, args: dict) -> dict | None:
        if action == "set_pose":
            return self._set_pose(args)
        if action == "home":
            return self._home()
        if action in ("start", "info"):
            return {"state": "active", "topic": "/servo_poses"}
        if action == "stop":
            return {"state": "idle"}
        return None

    def _set_pose(self, args: dict) -> dict:
        arm = args.get("arm", "left")
        msg = ServoPose()
        stamp = self._node.get_clock().now().to_msg()

        pose_data = {
            "x": _clamp(args.get("x", 0.0), -1.0, 1.0),
            "y": _clamp(args.get("y", 0.0), -1.0, 1.0),
            "z": _clamp(args.get("z", 0.0), -1.0, 1.0),
            "qx": _clamp(args.get("qx", 0.0), -1.0, 1.0),
            "qy": _clamp(args.get("qy", 0.0), -1.0, 1.0),
            "qz": _clamp(args.get("qz", 0.0), -1.0, 1.0),
            "qw": _clamp(args.get("qw", 1.0), -1.0, 1.0),
        }

        if arm == "left":
            msg.left_pose.header.stamp = stamp
            msg.left_pose.header.frame_id = _SERVO_FRAME
            msg.left_pose.position.x = pose_data["x"]
            msg.left_pose.position.y = pose_data["y"]
            msg.left_pose.position.z = pose_data["z"]
            msg.left_pose.orientation.x = pose_data["qx"]
            msg.left_pose.orientation.y = pose_data["qy"]
            msg.left_pose.orientation.z = pose_data["qz"]
            msg.left_pose.orientation.w = pose_data["qw"]
        else:
            msg.right_pose.header.stamp = stamp
            msg.right_pose.header.frame_id = _SERVO_FRAME
            msg.right_pose.position.x = pose_data["x"]
            msg.right_pose.position.y = pose_data["y"]
            msg.right_pose.position.z = pose_data["z"]
            msg.right_pose.orientation.x = pose_data["qx"]
            msg.right_pose.orientation.y = pose_data["qy"]
            msg.right_pose.orientation.z = pose_data["qz"]
            msg.right_pose.orientation.w = pose_data["qw"]

        self._pub.publish(msg)
        return {"state": "ok", "arm": arm, "pose": pose_data}

    def _home(self) -> dict:
        """Send arms to home pose (neutral)."""
        msg = ServoPose()
        stamp = self._node.get_clock().now().to_msg()
        msg.left_pose.header.stamp = stamp
        msg.left_pose.header.frame_id = _SERVO_FRAME
        msg.right_pose.header.stamp = stamp
        msg.right_pose.header.frame_id = _SERVO_FRAME
        msg.left_pose.orientation.w = 1.0
        msg.right_pose.orientation.w = 1.0
        self._pub.publish(msg)
        return {"state": "ok", "message": "arms homed"}


# ── HeadGesturePlugin (actuator) ─────────────────────────────────────────────

class HeadGesturePlugin:
    """头部语义动作 — 点头/摇头/左右观察等。"""

    GESTURES = {
        "nod": {"joint": "neck_pitch_joint", "position": -0.17, "desc": "点头 (Yes)"},
        "shake": {"joint": "neck_yaw_joint", "position": -0.17, "desc": "摇头 (No)"},
        "look_left": {"joint": "neck_yaw_joint", "position": -0.35, "desc": "向左看"},
        "look_right": {"joint": "neck_yaw_joint", "position": 0.35, "desc": "向右看"},
        "up": {"joint": "neck_pitch_joint", "position": 0.35, "desc": "抬头看上方"},
        "down": {"joint": "neck_pitch_joint", "position": -0.35, "desc": "低头看下方"},
        "reset": {"joint": None, "position": 0.0, "desc": "头部回中"},
    }

    def __init__(self, plugin_config: dict, namespace: str, executor):
        self._ns = namespace
        self._node = _Q5Node("head_gesture")
        executor.add_node(self._node)

        self._pub = self._node.create_publisher(
            HybridJointCommand, "/wr1_controller/commands", _RELIABLE_QOS)
        self._state = "idle"

    def get_tool(self) -> dict:
        return {
            "name": "head_gesture",
            "type": "actuator",
            "multiInstance": False,
            "description": "Q5 head gestures — nod (yes), shake (no), look_left, look_right, up, down, reset",
            "inputSchema": {"type": "object", "properties": {
                "action": {"type": "string", "enum": ["start", "stop", "gesture", "info"], "default": "gesture"},
                "gesture": {"type": "string", "enum": list(HeadGesturePlugin.GESTURES.keys()), "description": "Gesture name"},
            }},
            "default_action": "gesture",
        }

    def start(self) -> None:
        self._state = "active"

    def stop(self) -> None:
        self._publish_zero()
        self._state = "idle"

    def dispatch(self, action: str, args: dict) -> dict | None:
        if action == "gesture":
            name = args.get("gesture", "nod")
            if name not in self.GESTURES:
                return {"state": "error", "message": f"Unknown gesture: {name}"}
            g = self.GESTURES[name]
            if g["joint"] is None:
                self._publish_zero()
                return {"state": "ok", "gesture": name, "description": g["desc"]}
            cmd = _make_hybrid_command([g["joint"]], [g["position"]], kps=[500.0], kds=[5.0])
            cmd.header.stamp = self._node.get_clock().now().to_msg()
            self._pub.publish(cmd)
            # Auto-reset after 1s
            threading.Thread(target=self._reset, args=(g["joint"],), daemon=True).start()
            return {"state": "ok", "gesture": name, "description": g["desc"]}
        if action in ("start", "info"):
            return {"state": "active", "gestures": list(self.GESTURES.keys())}
        if action == "stop":
            return {"state": "idle"}
        return None

    def _reset(self, joint: str):
        time.sleep(1.0)
        self._publish_zero()

    def _publish_zero(self):
        cmd = _make_hybrid_command(["neck_pitch_joint", "neck_yaw_joint"], [0.0, 0.0],
                                    kps=[0.0, 0.0], kds=[0.0, 0.0])
        cmd.header.stamp = self._node.get_clock().now().to_msg()
        self._pub.publish(cmd)


# ── ArmGesturePlugin (actuator) ──────────────────────────────────────────────

class ArmGesturePlugin:
    """双臂语义动作 — 挥手/敬礼/欢迎等。"""

    GESTURES = {
        "wave": {"desc": "挥手打招呼", "pose": {"x": 0.3, "y": 0.2, "z": 0.5, "qw": 1.0}},
        "salute": {"desc": "敬礼", "pose": {"x": 0.15, "y": 0.1, "z": 0.45, "qw": 0.7}},
        "welcome": {"desc": "欢迎", "pose": {"x": 0.25, "y": -0.15, "z": 0.45, "qw": 1.0}},
        "point": {"desc": "指方向", "pose": {"x": 0.35, "y": 0.0, "z": 0.45, "qw": 1.0}},
        "home": {"desc": "双臂回中"},
    }

    def __init__(self, plugin_config: dict, namespace: str, executor):
        self._ns = namespace
        self._node = _Q5Node("arm_gesture")
        executor.add_node(self._node)

        self._pub = self._node.create_publisher(
            ServoPose, "/servo_poses", _RELIABLE_QOS)
        self._state = "idle"

    def get_tool(self) -> dict:
        return {
            "name": "arm_gesture",
            "type": "actuator",
            "multiInstance": False,
            "description": "Q5 arm gestures — wave (hello), salute, welcome, point, home",
            "inputSchema": {"type": "object", "properties": {
                "action": {"type": "string", "enum": ["start", "stop", "gesture", "info"], "default": "gesture"},
                "gesture": {"type": "string", "enum": list(ArmGesturePlugin.GESTURES.keys()), "description": "Gesture name"},
            }},
            "default_action": "gesture",
        }

    def start(self) -> None:
        self._state = "active"

    def stop(self) -> None:
        self._state = "idle"

    def dispatch(self, action: str, args: dict) -> dict | None:
        if action == "gesture":
            name = args.get("gesture", "wave")
            if name not in self.GESTURES:
                return {"state": "error", "message": f"Unknown gesture: {name}"}
            g = self.GESTURES[name]
            msg = ServoPose()
            stamp = self._node.get_clock().now().to_msg()

            if name == "home":
                msg.left_pose.header.stamp = stamp
                msg.left_pose.header.frame_id = _SERVO_FRAME
                msg.right_pose.header.stamp = stamp
                msg.right_pose.header.frame_id = _SERVO_FRAME
                msg.left_pose.orientation.w = 1.0
                msg.right_pose.orientation.w = 1.0
            else:
                p = g["pose"]
                # Left arm
                msg.left_pose.header.stamp = stamp
                msg.left_pose.header.frame_id = _SERVO_FRAME
                msg.left_pose.position.x = p.get("x", 0)
                msg.left_pose.position.y = p.get("y", 0)
                msg.left_pose.position.z = p.get("z", 0)
                msg.left_pose.orientation.w = p.get("qw", 1.0)
                # Right arm (mirrored y)
                msg.right_pose.header.stamp = stamp
                msg.right_pose.header.frame_id = _SERVO_FRAME
                msg.right_pose.position.x = p.get("x", 0)
                msg.right_pose.position.y = -p.get("y", 0)
                msg.right_pose.position.z = p.get("z", 0)
                msg.right_pose.orientation.w = p.get("qw", 1.0)

            self._pub.publish(msg)
            return {"state": "ok", "gesture": name, "description": g["desc"]}
        if action in ("start", "info"):
            return {"state": "active", "gestures": list(self.GESTURES.keys())}
        if action == "stop":
            return {"state": "idle"}
        return None


# ── MotionPlugin (actuator) — motion_manager ──────────────────────────────────

class MotionPlugin:
    """运动管理 — /motion_manager/motion_request (MotionRequest)。"""

    def __init__(self, plugin_config: dict, namespace: str, executor):
        self._ns = namespace
        self._node = _Q5Node("motion")
        executor.add_node(self._node)

        self._pub = self._node.create_publisher(MotionStatus, f"/{self._ns}/motion_status", _LOW_LAT_QOS)
        self._sub = self._node.create_subscription(
            MotionStatus, "/motion_manager/motion_status",
            lambda m: self._pub.publish(m), _LOW_LAT_QOS)

        self._srv = self._node.create_client(MotionRequest, "/motion_manager/motion_request")
        self._srv.wait_for_service(timeout_sec=2.0)

        self._state = "idle"

    def get_tool(self) -> dict:
        return {
            "name": "motion",
            "type": "actuator",
            "multiInstance": False,
            "description": "Q5 motion manager — execute pre-defined motions via MotionRequest service.",
            "inputSchema": {"type": "object", "properties": {
                "action": {"type": "string", "enum": ["start", "stop", "execute", "status", "info"], "default": "execute"},
                "motion_name": {"type": "string", "description": "Motion name to execute"},
                "priority": {"type": "string", "enum": ["LOW", "MEDIUM", "HIGH", "SYSTEM", "EMERGENCY"],
                             "description": "Motion priority"},
            }},
            "default_action": "execute",
        }

    def start(self) -> None:
        self._state = "active"

    def stop(self) -> None:
        self._state = "idle"

    def dispatch(self, action: str, args: dict) -> dict | None:
        if action == "execute":
            return self._execute(args)
        if action == "status":
            return {"state": "active", "topic": f"/{self._ns}/motion_status"}
        if action in ("start", "info"):
            return {"state": "active"}
        if action == "stop":
            return {"state": "idle"}
        return None

    def _execute(self, args: dict) -> dict:
        if not self._srv.is_ready():
            return {"state": "error", "message": "motion_request service not available"}

        priority_map = {"LOW": 0, "MEDIUM": 20, "HIGH": 50, "SYSTEM": 80, "EMERGENCY": 99}
        prio_str = args.get("priority", "HIGH")
        prio_val = priority_map.get(prio_str, 50)

        req = MotionRequest.Request()
        req.motion_name = args.get("motion_name", "")
        req.motion_priority.priority = prio_val
        future = self._srv.call_async(req)
        try:
            rclpy.spin_until_future_complete(self._node, future, timeout_sec=5.0)
            resp = future.result()
            return {
                "state": "ok" if resp.result.error_code == 0 else "error",
                "error_code": resp.result.error_code,
                "description": resp.result.error_description,
            }
        except Exception as e:
            return {"state": "error", "message": str(e)}


# ── GesturePlugin (actuator) — gesture playback ──────────────────────────────

class GesturePlugin:
    """手势播放 — /gesture/upper_limb_play (topic) + stop_play/is_play (services)。"""

    def __init__(self, plugin_config: dict, namespace: str, executor):
        self._ns = namespace
        self._node = _Q5Node("gesture")
        executor.add_node(self._node)

        self._pub = self._node.create_publisher(String, "/gesture/upper_limb_play", _RELIABLE_QOS)

        self._srv_stop = self._node.create_client(Trigger, "/gesture/stop_play")
        self._srv_is = self._node.create_client(Trigger, "/gesture/is_play")

        self._state = "idle"

    def get_tool(self) -> dict:
        return {
            "name": "gesture",
            "type": "actuator",
            "multiInstance": False,
            "description": "Q5 gesture playback — play pre-recorded upper limb gestures by name, stop, and check status.",
            "inputSchema": {"type": "object", "properties": {
                "action": {"type": "string", "enum": ["start", "stop", "play", "stop_gesture", "is_playing", "info"], "default": "play"},
                "gesture_name": {"type": "string", "description": "Gesture name to play"},
            }},
            "default_action": "play",
        }

    def start(self) -> None:
        self._state = "active"

    def stop(self) -> None:
        self._stop()
        self._state = "idle"

    def dispatch(self, action: str, args: dict) -> dict | None:
        if action == "play":
            name = args.get("gesture_name", "")
            if not name:
                return {"state": "error", "message": "gesture_name required"}
            self._pub.publish(String(data=name))
            return {"state": "ok", "gesture": name}
        if action == "stop_gesture":
            self._stop()
            return {"state": "ok", "message": "gesture stopped"}
        if action == "is_playing":
            return self._is_playing()
        if action in ("start", "info"):
            return {"state": "active"}
        if action == "stop":
            return {"state": "idle"}
        return None

    def _stop(self):
        if not self._srv_stop.is_ready():
            return
        req = Trigger.Request()
        future = self._srv_stop.call_async(req)
        try:
            rclpy.spin_until_future_complete(self._node, future, timeout_sec=2.0)
        except Exception:
            pass

    def _is_playing(self) -> dict:
        if not self._srv_is.is_ready():
            return {"state": "error", "message": "is_play service not available"}
        req = Trigger.Request()
        future = self._srv_is.call_async(req)
        try:
            rclpy.spin_until_future_complete(self._node, future, timeout_sec=2.0)
            resp = future.result()
            return {"state": "ok", "playing": resp.success}
        except Exception as e:
            return {"state": "error", "message": str(e)}


# ── AudioPlugin (actuator) — audio playback action ───────────────────────────

class AudioPlugin:
    """音频播放 — /audio_player/play (xbot_common_interfaces/action/AudioPlay)
    + audio_player/set_volume (SetVolume) + audio_player/stop_play (Trigger)
    + audio_player/is_play (Trigger)。
    """

    def __init__(self, plugin_config: dict, namespace: str, executor):
        self._ns = namespace
        self._node = _Q5Node("audio")
        executor.add_node(self._node)

        # Action server on real machine: /audio_player/play (type: AudioPlay)
        self._action_client = ActionClient(self._node, AudioPlay, "/audio_player/play")

        self._srv_volume = self._node.create_client(SetVolume, "audio_player/set_volume")
        self._srv_stop = self._node.create_client(Trigger, "audio_player/stop_play")
        self._srv_is_play = self._node.create_client(Trigger, "audio_player/is_play")

        self._state = "idle"
        self._device = plugin_config.get("device", "plughw:2,0")

    def get_tool(self) -> dict:
        return {
            "name": "audio",
            "type": "actuator",
            "multiInstance": False,
            "description": "Q5 audio playback — play audio via /audio_player/play (AudioPlay action). "
                           "Supports 3 modes: by id, by file path, or by item (JSON). "
                           "Volume control via set_volume, stop via stop_play, "
                           "check play status via is_play.",
            "inputSchema": {"type": "object", "properties": {
                "action": {"type": "string",
                            "enum": ["start", "stop", "play", "set_volume", "stop_audio", "is_play", "info"],
                            "default": "play"},
                "mode": {"type": "integer", "enum": [0, 1, 2],
                         "description": "0=by id, 1=by path, 2=by item (JSON)"},
                "id": {"type": "integer", "description": "Audio ID (mode=0, must be >0)"},
                "path": {"type": "string", "description": "Audio file path (mode=1)"},
                "item": {"type": "string",
                         "description": "JSON item string (mode=2), e.g. {\"file_name\":\"x.wav\",\"text\":\"hello\"}"},
                "force_play": {"type": "boolean", "description": "true=interrupt current audio, false=default"},
                "timeout": {"type": "integer", "description": "Playback timeout in seconds"},
                "volume": {"type": "integer", "description": "Volume level 0-100 (for set_volume action)"},
            }},
            "default_action": "play",
        }

    def start(self) -> None:
        self._state = "active"

    def stop(self) -> None:
        self._stop()
        self._state = "idle"

    def dispatch(self, action: str, args: dict) -> dict | None:
        if action == "play":
            return self._play(args)
        if action == "set_volume":
            return self._set_volume(int(args.get("volume", 50)))
        if action == "stop_audio":
            self._stop()
            return {"state": "ok", "message": "audio stopped"}
        if action == "is_play":
            return self._is_play()
        if action in ("start", "info"):
            return {"state": "active", "device": self._device,
                    "action_server": "/audio_player/play",
                    "modes": {"0": "by id", "1": "by path", "2": "by item (JSON)"}}
        if action == "stop":
            return {"state": "idle"}
        return None

    def _play(self, args: dict) -> dict:
        if not self._action_client.wait_for_server(timeout_sec=3.0):
            return {"state": "error", "message": "audio_play_action server not available"}
        goal = AudioPlay.Goal()
        mode = int(args.get("mode", 1))
        goal.mode = mode
        goal.force_play = bool(args.get("force_play", False))
        goal.id = int(args.get("id", 0))
        goal.path = str(args.get("path", ""))
        goal.item = str(args.get("item", ""))
        goal.timeout = int(args.get("timeout", 0))
        future = self._action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self._node, future, timeout_sec=5.0)
        goal_handle = future.result()
        if not goal_handle.accepted:
            return {"state": "error", "message": "audio play goal rejected"}
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self._node, result_future, timeout_sec=10.0)
        result = result_future.result()
        return {"state": "ok" if result.success else "error",
                "message": result.message,
                "mode": mode}

    def _set_volume(self, volume: int) -> dict:
        volume = max(0, min(100, volume))
        if not self._srv_volume.is_ready():
            return {"state": "error", "message": "set_volume service not available"}
        req = SetVolume.Request()
        req.volume = volume
        future = self._srv_volume.call_async(req)
        try:
            rclpy.spin_until_future_complete(self._node, future, timeout_sec=2.0)
            resp = future.result()
            # SetVolume response: resp.result (bool), may not have message field
            return {"state": "ok" if resp.result else "error", "volume": volume}
        except Exception as e:
            return {"state": "error", "message": str(e)}

    def _stop(self):
        if not self._srv_stop.is_ready():
            return
        req = Trigger.Request()
        future = self._srv_stop.call_async(req)
        try:
            rclpy.spin_until_future_complete(self._node, future, timeout_sec=2.0)
        except Exception:
            pass

    def _is_play(self) -> dict:
        """查询当前是否正在播放音频 (audio_player/is_play, Trigger 类型)。"""
        if not self._srv_is_play.is_ready():
            return {"state": "error", "message": "is_play service not available"}
        req = Trigger.Request()
        future = self._srv_is_play.call_async(req)
        try:
            rclpy.spin_until_future_complete(self._node, future, timeout_sec=2.0)
            resp = future.result()
            # Trigger response: resp.success (bool)
            return {"state": "ok", "is_playing": resp.success}
        except Exception as e:
            return {"state": "error", "message": str(e)}


# ── SpeakerPlugin (actuator) ──────────────────────────────────────────────────

class SpeakerPlugin:
    """语音播报 — 使用本地 sherpa-onnx TTS 生成音频，通过 pyaudio 直接播放。

    流程：text → sherpa-onnx TTS → 16kHz PCM → pyaudio 播放

    依赖：sherpa_onnx, pyaudio
    模型文件：model-steps-3.onnx, vocos-16khz-univ.onnx, lexicon.txt, tokens.txt, espeak-ng-data/

    注：真机上 /audio_player/play action 没有 server（0 servers），
    audio_player/set_volume 和 is_play 服务也不可用。
    因此音量控制通过 pyaudio 软件音量实现，播放通过 pyaudio 直接输出。
    """

    def __init__(self, plugin_config: dict, namespace: str, executor):
        self._ns = namespace
        self._node = _Q5Node("speaker")
        executor.add_node(self._node)

        self._state = "idle"
        self._volume = int(plugin_config.get("volume", 50))
        self._device_name = plugin_config.get("device", None)  # e.g. "plughw:2,0"

        # Playback state
        self._is_playing = False
        self._play_thread = None
        self._stop_flag = threading.Event()

        # TTS adapter (lazy init — sherpa-onnx is heavy)
        self._tts_config = {
            "model_dir": plugin_config.get("model_dir", "/models/sherpa-onnx/tts"),
            "speaker_id": int(plugin_config.get("speaker_id", 0)),
            "speed": float(plugin_config.get("speed", 1.0)),
        }
        self._tts = None
        self._tts_error = None

    def _ensure_tts(self):
        """Lazy-initialize the sherpa-onnx TTS engine on first use."""
        if self._tts is not None or self._tts_error is not None:
            return
        try:
            import os
            import sherpa_onnx

            model_dir = self._tts_config["model_dir"]
            acoustic_model = os.path.join(model_dir, "model-steps-3.onnx")
            vocoder = os.path.join(model_dir, "vocos-16khz-univ.onnx")
            lexicon = os.path.join(model_dir, "lexicon.txt")
            tokens = os.path.join(model_dir, "tokens.txt")
            data_dir = os.path.join(model_dir, "espeak-ng-data")
            if not os.path.isdir(data_dir):
                data_dir = ""

            rule_fsts = []
            for name in ("date-zh.fst", "number-zh.fst", "phone-zh.fst"):
                p = os.path.join(model_dir, name)
                if os.path.exists(p):
                    rule_fsts.append(p)

            cfg = sherpa_onnx.OfflineTtsConfig(
                model=sherpa_onnx.OfflineTtsModelConfig(
                    matcha=sherpa_onnx.OfflineTtsMatchaModelConfig(
                        acoustic_model=acoustic_model,
                        vocoder=vocoder,
                        lexicon=lexicon if os.path.exists(lexicon) else "",
                        tokens=tokens,
                        data_dir=data_dir,
                        length_scale=1.0 / self._tts_config["speed"],
                    ),
                    num_threads=2,
                    provider="cpu",
                ),
                rule_fsts=",".join(rule_fsts) if rule_fsts else "",
            )
            self._tts = sherpa_onnx.OfflineTts(cfg)
            self._node.get_logger().info(
                f"sherpa-onnx TTS loaded: model_dir={model_dir}, "
                f"speaker_id={self._tts_config['speaker_id']}, "
                f"speed={self._tts_config['speed']}")
        except Exception as e:
            self._tts_error = str(e)
            self._node.get_logger().error(f"Failed to init sherpa-onnx TTS: {e}")

    def _synthesize(self, text: str) -> bytes:
        """Synthesize text to 16kHz 16-bit mono PCM bytes."""
        import struct

        self._ensure_tts()
        if self._tts is None:
            raise RuntimeError(f"TTS engine not available: {self._tts_error}")

        audio = self._tts.generate(
            text,
            sid=self._tts_config["speaker_id"],
            speed=self._tts_config["speed"],
        )
        float_samples = audio.samples
        # Apply software volume (0-100 → 0.0-1.0)
        vol_scale = self._volume / 100.0
        pcm = struct.pack(
            f'<{len(float_samples)}h',
            *[int(max(-32768, min(32767, s * 32767 * vol_scale))) for s in float_samples],
        )
        return pcm

    def _play_pcm(self, pcm: bytes):
        """Play PCM bytes via pyaudio in a background thread."""
        import pyaudio

        def _play():
            try:
                pa = pyaudio.PyAudio()
                stream = pa.open(
                    format=pyaudio.paInt16,
                    channels=1,
                    rate=16000,
                    output=True,
                    output_device_index=self._get_device_index(pa),
                )
                # Write in chunks to allow stop
                chunk_size = 3200  # 100ms at 16kHz 16-bit
                self._is_playing = True
                for i in range(0, len(pcm), chunk_size):
                    if self._stop_flag.is_set():
                        break
                    stream.write(pcm[i:i + chunk_size])
                stream.stop_stream()
                stream.close()
                pa.terminate()
            except Exception as e:
                self._node.get_logger().error(f"pyaudio playback failed: {e}")
            finally:
                self._is_playing = False

        self._stop_flag.clear()
        self._play_thread = threading.Thread(target=_play, daemon=True, name="speaker_play")
        self._play_thread.start()

    def _get_device_index(self, pa) -> int | None:
        """Find the output device index by name, or None for default."""
        if not self._device_name:
            return None
        for i in range(pa.get_device_count()):
            info = pa.get_device_info_by_index(i)
            if self._device_name in info.get("name", "") and info.get("maxOutputChannels", 0) > 0:
                return i
        return None

    def get_tool(self) -> dict:
        return {
            "name": "speaker",
            "type": "actuator",
            "multiInstance": False,
            "description": "Q5 speaker — text-to-speech using local sherpa-onnx TTS engine. "
                           "Generates 16kHz PCM audio and plays via pyaudio directly. "
                           "Supports volume control (software) and stop.",
            "inputSchema": {"type": "object", "properties": {
                "action": {"type": "string",
                            "enum": ["start", "stop", "speak", "set_volume", "stop_audio", "is_playing", "info"],
                            "default": "speak"},
                "text": {"type": "string", "description": "Text to synthesize and play"},
                "volume": {"type": "integer", "description": "Volume level 0-100 (software volume)"},
            }},
            "default_action": "speak",
        }

    def start(self) -> None:
        self._state = "active"

    def stop(self) -> None:
        self._stop_flag.set()
        if self._play_thread:
            self._play_thread.join(timeout=2.0)
        self._state = "idle"

    def dispatch(self, action: str, args: dict) -> dict | None:
        if action == "speak":
            return self._speak(args)
        if action == "set_volume":
            self._volume = max(0, min(100, int(args.get("volume", 50))))
            return {"state": "ok", "volume": self._volume}
        if action == "stop_audio":
            self._stop_flag.set()
            if self._play_thread:
                self._play_thread.join(timeout=2.0)
            return {"state": "ok", "message": "audio stopped"}
        if action == "is_playing":
            return {"state": "ok", "is_playing": self._is_playing}
        if action in ("start", "info"):
            return {"state": "active",
                    "tts_engine": "sherpa-onnx Matcha",
                    "audio_output": "pyaudio (16kHz 16-bit mono)",
                    "model_dir": self._tts_config["model_dir"],
                    "volume": self._volume}
        if action == "stop":
            self._stop_flag.set()
            if self._play_thread:
                self._play_thread.join(timeout=2.0)
            return {"state": "idle"}
        return None

    def _speak(self, args: dict) -> dict:
        text = args.get("text", "")
        if not text:
            return {"state": "error", "message": "text required"}

        # Stop any current playback
        self._stop_flag.set()
        if self._play_thread:
            self._play_thread.join(timeout=2.0)
        self._stop_flag.clear()

        # Step 1: Synthesize PCM via local TTS
        try:
            pcm = self._synthesize(text)
        except Exception as e:
            return {"state": "error", "message": f"TTS synthesis failed: {e}"}

        if not pcm:
            return {"state": "error", "message": "TTS produced empty audio"}

        # Step 2: Play via pyaudio
        self._play_pcm(pcm)

        return {"state": "ok", "text": text,
                "pcm_bytes": len(pcm),
                "volume": self._volume}


# ── LedPlugin (actuator) ─────────────────────────────────────────────────────

class LedPlugin:
    """LED 控制 — /led_control (UInt8)。"""

    def __init__(self, plugin_config: dict, namespace: str, executor):
        self._ns = namespace
        self._node = _Q5Node("led")
        executor.add_node(self._node)

        self._pub = self._node.create_publisher(UInt8, "/led_control", _RELIABLE_QOS)
        self._state = "idle"

    def get_tool(self) -> dict:
        return {
            "name": "led",
            "type": "actuator",
            "multiInstance": False,
            "description": "Q5 LED control — UInt8 value (0=off, 1-255=brightness or color index).",
            "inputSchema": {"type": "object", "properties": {
                "action": {"type": "string", "enum": ["start", "stop", "set", "info"], "default": "set"},
                "value": {"type": "integer", "description": "LED value 0-255"},
            }},
            "default_action": "set",
        }

    def start(self) -> None:
        self._state = "active"

    def stop(self) -> None:
        self._publish(0)
        self._state = "idle"

    def dispatch(self, action: str, args: dict) -> dict | None:
        if action == "set":
            val = _clamp(int(args.get("value", 0)), 0, 255)
            self._publish(val)
            return {"state": "ok", "value": val}
        if action in ("start", "info"):
            return {"state": "active"}
        if action == "stop":
            return {"state": "idle"}
        return None

    def _publish(self, val: int):
        self._pub.publish(UInt8(data=val))


# ── NavPlugin (actuator) — navigation ────────────────────────────────────────

class NavPlugin:
    """导航控制 — /navigate/* services。

    真机确认的 service（ros2 service list | grep navigate）：
      /navigate/start_nav          — StringMessage
      /navigate/stop_nav           — Trigger
      /navigate/pause_nav_ctrl     — StringMessage
      /navigate/is_nav_executing   — Trigger
      /navigate/is_config_nav      — StringMessage
      /navigate/is_license_verify  — StringMessage (额外发现)
    """

    def __init__(self, plugin_config: dict, namespace: str, executor):
        self._ns = namespace
        self._node = _Q5Node("nav")
        executor.add_node(self._node)

        self._srv_start = self._node.create_client(StringMessage, "/navigate/start_nav")
        self._srv_stop = self._node.create_client(Trigger, "/navigate/stop_nav")
        self._srv_pause = self._node.create_client(StringMessage, "/navigate/pause_nav_ctrl")
        self._srv_is_exec = self._node.create_client(Trigger, "/navigate/is_nav_executing")
        self._srv_is_config = self._node.create_client(StringMessage, "/navigate/is_config_nav")

        self._state = "idle"

    def get_tool(self) -> dict:
        return {
            "name": "nav",
            "type": "actuator",
            "multiInstance": False,
            "description": "Q5 navigation control — start/stop/pause navigation, check execution status.",
            "inputSchema": {"type": "object", "properties": {
                "action": {"type": "string", "enum": ["start", "stop", "pause", "resume", "status", "info"], "default": "status"},
                "nav_mode": {"type": "string", "description": "Navigation mode/name (start action)"},
            }},
            "default_action": "status",
        }

    def start(self) -> None:
        self._state = "active"

    def stop(self) -> None:
        self._state = "idle"

    def dispatch(self, action: str, args: dict) -> dict | None:
        if action == "start":
            return self._start_nav(args.get("nav_mode", ""))
        if action == "stop":
            return self._stop_nav()
        if action == "pause":
            return self._pause_nav()
        if action == "resume":
            return self._pause_nav()  # toggle
        if action == "status":
            return self._status()
        if action in ("start", "info"):
            return {"state": "active"}
        if action == "stop":
            return {"state": "idle"}
        return None

    def _start_nav(self, mode: str) -> dict:
        if not self._srv_start.is_ready():
            return {"state": "error", "message": "start_nav service not available"}
        req = StringMessage.Request()
        req.data = mode
        future = self._srv_start.call_async(req)
        try:
            rclpy.spin_until_future_complete(self._node, future, timeout_sec=3.0)
            resp = future.result()
            return {"state": "ok" if resp.result else "error"}
        except Exception as e:
            return {"state": "error", "message": str(e)}

    def _stop_nav(self) -> dict:
        if not self._srv_stop.is_ready():
            return {"state": "error", "message": "stop_nav service not available"}
        req = Trigger.Request()
        future = self._srv_stop.call_async(req)
        try:
            rclpy.spin_until_future_complete(self._node, future, timeout_sec=3.0)
            resp = future.result()
            return {"state": "ok" if resp.success else "error"}
        except Exception as e:
            return {"state": "error", "message": str(e)}

    def _pause_nav(self) -> dict:
        if not self._srv_pause.is_ready():
            return {"state": "error", "message": "pause_nav_ctrl service not available"}
        req = StringMessage.Request()
        req.data = "pause"
        future = self._srv_pause.call_async(req)
        try:
            rclpy.spin_until_future_complete(self._node, future, timeout_sec=3.0)
            resp = future.result()
            return {"state": "ok" if resp.result else "error"}
        except Exception as e:
            return {"state": "error", "message": str(e)}

    def _status(self) -> dict:
        if not self._srv_is_exec.is_ready():
            return {"state": "error", "message": "is_nav_executing service not available"}
        req = Trigger.Request()
        future = self._srv_is_exec.call_async(req)
        try:
            rclpy.spin_until_future_complete(self._node, future, timeout_sec=2.0)
            resp = future.result()
            return {"state": "navigating" if resp.success else "idle", "executing": resp.success}
        except Exception as e:
            return {"state": "error", "message": str(e)}


# ── TeleopPlugin (actuator) — joystick/remote control ────────────────────────

class TeleopPlugin:
    """遥控/手柄控制 — 真机确认的 topic：
      /joy [sensor_msgs/msg/Joy]
      /send_remote/command [sensor_msgs/msg/Joy]
      /loco/remoteControl/radio [xbot_common_interfaces/msg/ChannelsMsg]
      /remote_control/trigger_play [std_msgs/msg/String]
    """

    def __init__(self, plugin_config: dict, namespace: str, executor):
        self._ns = namespace
        self._node = _Q5Node("teleop")
        executor.add_node(self._node)

        self._pub_joy = self._node.create_publisher(Joy, "/send_remote/command", _LOW_LAT_QOS)
        self._pub_radio = self._node.create_publisher(ChannelsMsg, "/loco/remoteControl/radio", _LOW_LAT_QOS)

        self._sub_joy = self._node.create_subscription(
            Joy, "/joy", self._on_joy, _LOW_LAT_QOS)

        self._state = "idle"

    def get_tool(self) -> dict:
        return {
            "name": "teleop",
            "type": "actuator",
            "multiInstance": False,
            "description": "Q5 teleoperation — joystick input forwarding and remote control channel messaging.",
            "inputSchema": {"type": "object", "properties": {
                "action": {"type": "string", "enum": ["start", "stop", "trigger", "info"], "default": "info"},
                "channels": {"type": "array", "items": {"type": "number"}, "description": "Channel values"},
            }},
            "default_action": "info",
        }

    def start(self) -> None:
        self._state = "active"

    def stop(self) -> None:
        self._state = "idle"

    def dispatch(self, action: str, args: dict) -> dict | None:
        if action == "trigger":
            channels = args.get("channels", [0.0] * 8)
            msg = ChannelsMsg()
            msg.channels = [float(c) for c in channels]
            self._pub_radio.publish(msg)
            return {"state": "ok", "channels": msg.channels}
        if action in ("start", "info"):
            return {"state": "active", "topics": {
                "joy_input": "/send_remote/command",
                "radio_control": "/loco/remoteControl/radio",
            }}
        if action == "stop":
            return {"state": "idle"}
        return None

    def _on_joy(self, msg: Joy):
        self._pub_joy.publish(msg)


# ── CameraPlugin (sensor) — placeholder ──────────────────────────────────────

class CameraPlugin:
    """相机 — 预留实现，待确认相机 ROS2 topic 名称后补充。"""

    def __init__(self, plugin_config: dict, namespace: str, executor):
        self._ns = namespace
        self._node = _Q5Node("camera")
        executor.add_node(self._node)
        self._state = "idle"

    def get_tool(self) -> dict:
        return {
            "name": "camera",
            "type": "sensor",
            "multiInstance": False,
            "description": "Q5 camera — placeholder.待确认相机 topic 后实现。",
            "inputSchema": {"type": "object", "properties": {
                "action": {"type": "string", "enum": ["start", "stop", "info"], "default": "info"},
            }},
            "default_action": "info",
        }

    def start(self) -> None:
        self._state = "active"

    def stop(self) -> None:
        self._state = "idle"

    def dispatch(self, action: str, args: dict) -> dict | None:
        if action in ("start", "info"):
            return {"state": "active", "note": "camera placeholder — awaiting topic confirmation"}
        if action == "stop":
            return {"state": "idle"}
        return None


# ── SimpleActionsPlugin (actuator) — predefined simple actions ───────────────

class SimpleActionsPlugin:
    """简单动作播放 — /simple_actions (xbot_common_interfaces/action/SimpleActions)。

    真机 Action server: /simple_actions。
    SimpleActions Goal: action_name (string), time_cost (float32)
    SimpleActions Result: result (int32: SUCCESS=0/DENIED=1/FAILED=2), message (string)
    SimpleActions Feedback: progress (float32)
    """

    def __init__(self, plugin_config: dict, namespace: str, executor):
        self._ns = namespace
        self._node = _Q5Node("simple_actions")
        executor.add_node(self._node)

        self._action_client = ActionClient(self._node, SimpleActions, "/simple_actions")
        self._state = "idle"

    def get_tool(self) -> dict:
        return {
            "name": "simple_actions",
            "type": "actuator",
            "multiInstance": False,
            "description": "Q5 simple actions — trigger pre-defined simple actions (greetings, poses, etc.) "
                           "via /simple_actions.",
            "inputSchema": {"type": "object", "properties": {
                "action": {"type": "string", "enum": ["start", "stop", "execute", "info"],
                           "default": "execute"},
                "action_name": {"type": "string", "description": "Name of the simple action to execute"},
                "time_cost": {"type": "number",
                              "description": "Time cost limit in seconds (0 = default)"},
            }},
            "default_action": "execute",
        }

    def start(self) -> None:
        self._state = "active"

    def stop(self) -> None:
        self._state = "idle"

    def dispatch(self, action: str, args: dict) -> dict | None:
        if action == "execute":
            return self._execute(args.get("action_name", ""),
                                 float(args.get("time_cost", 0.0)))
        if action in ("start", "info"):
            return {"state": "active", "action_server": "/simple_actions"}
        if action == "stop":
            return {"state": "idle"}
        return None

    def _execute(self, action_name: str, time_cost: float) -> dict:
        if not action_name:
            return {"state": "error", "message": "action_name required"}
        if not self._action_client.wait_for_server(timeout_sec=3.0):
            return {"state": "error", "message": "simple_actions server not available"}
        goal = SimpleActions.Goal()
        goal.action_name = action_name
        goal.time_cost = time_cost
        future = self._action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self._node, future, timeout_sec=5.0)
        goal_handle = future.result()
        if not goal_handle.accepted:
            return {"state": "error", "message": "simple action goal rejected"}
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self._node, result_future, timeout_sec=30.0)
        result = result_future.result()
        return {"state": "ok" if result.result == SimpleActions.Result.SUCCESS else "error",
                "action_name": action_name,
                "message": result.message}


# ── SimpleTrajectoryPlugin (actuator) — predefined trajectories ──────────────

class SimpleTrajectoryPlugin:
    """预定义轨迹 — action server /simple_trajectory。

    SimpleTrajectory Goal: traj_type (int32), duration
      traj_type 常量: ZERO=0, SIN_WAVE=1, LIFT_UP=2, MPC_INIT=14
    SimpleTrajectory Result: result (int32), message (string)
      result 常量: SUCCESS=0, DENIED=1, FAILED=2
    """

    _TYPE_NAMES = {0: "ZERO", 1: "SIN_WAVE", 2: "LIFT_UP", 14: "MPC_INIT"}

    def __init__(self, plugin_config: dict, namespace: str, executor):
        self._ns = namespace
        self._node = _Q5Node("simple_trajectory")
        executor.add_node(self._node)

        # Action server name needs confirmation on real machine
        srv_name = plugin_config.get("action_server", "/simple_trajectory")
        self._action_client = ActionClient(self._node, SimpleTrajectory, srv_name)
        self._srv_name = srv_name
        self._state = "idle"

    def get_tool(self) -> dict:
        return {
            "name": "simple_trajectory",
            "type": "actuator",
            "multiInstance": False,
            "description": "Q5 simple trajectory — trigger predefined trajectories "
                           "(ZERO=0, SIN_WAVE=1, LIFT_UP=2, MPC_INIT=14) "
                           "via SimpleTrajectory action.",
            "inputSchema": {"type": "object", "properties": {
                "action": {"type": "string", "enum": ["start", "stop", "execute", "info"],
                           "default": "execute"},
                "traj_type": {"type": "integer", "enum": [0, 1, 2, 14],
                              "description": "0=ZERO, 1=SIN_WAVE, 2=LIFT_UP, 14=MPC_INIT"},
                "duration": {"type": "number", "description": "Trajectory duration in seconds"},
            }},
            "default_action": "execute",
        }

    def start(self) -> None:
        self._state = "active"

    def stop(self) -> None:
        self._state = "idle"

    def dispatch(self, action: str, args: dict) -> dict | None:
        if action == "execute":
            return self._execute(int(args.get("traj_type", 3)),
                                 float(args.get("duration", 0.0)))
        if action in ("start", "info"):
            return {"state": "active", "action_server": self._srv_name,
                    "traj_types": self._TYPE_NAMES}
        if action == "stop":
            return {"state": "idle"}
        return None

    def _execute(self, traj_type: int, duration: float) -> dict:
        if not self._action_client.wait_for_server(timeout_sec=3.0):
            return {"state": "error", "message": f"{self._srv_name} not available"}
        goal = SimpleTrajectory.Goal()
        goal.traj_type = traj_type
        goal.duration = duration
        future = self._action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self._node, future, timeout_sec=5.0)
        goal_handle = future.result()
        if not goal_handle.accepted:
            return {"state": "error", "message": "trajectory goal rejected",
                    "traj_type": self._TYPE_NAMES.get(traj_type, str(traj_type))}
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self._node, result_future, timeout_sec=30.0)
        result = result_future.result()
        # result.result: int8 (DENIED/FAILED/SUCCESS), result.message: string
        return {"state": "ok" if result.result == SimpleTrajectory.Result.SUCCESS else "error",
                "traj_type": self._TYPE_NAMES.get(traj_type, str(traj_type)),
                "message": result.message}


# ── BehaviorPlugin (actuator) — behavior tree execution ─────────────────────

class BehaviorPlugin:
    """行为树执行 — action server 名称需在真机上确认。

    Behavior Goal: auto_start (bool), command (string)
    Behavior Result: result (int8), message (string)
      result 常量: CREATE_FAILED, EXECUTION_FAILED, INVALID_COMMAND, SUCCESS
    """

    def __init__(self, plugin_config: dict, namespace: str, executor):
        self._ns = namespace
        self._node = _Q5Node("behavior")
        executor.add_node(self._node)

        srv_name = plugin_config.get("action_server", "/behavior_action")
        self._action_client = ActionClient(self._node, Behavior, srv_name)
        self._srv_name = srv_name
        self._state = "idle"

    def get_tool(self) -> dict:
        return {
            "name": "behavior",
            "type": "actuator",
            "multiInstance": False,
            "description": "Q5 behavior execution — trigger behavior tree commands "
                           "via Behavior action.",
            "inputSchema": {"type": "object", "properties": {
                "action": {"type": "string", "enum": ["start", "stop", "execute", "info"],
                           "default": "execute"},
                "command": {"type": "string", "description": "Behavior command string"},
                "auto_start": {"type": "boolean", "description": "Auto-start behavior tree", "default": True},
            }},
            "default_action": "execute",
        }

    def start(self) -> None:
        self._state = "active"

    def stop(self) -> None:
        self._state = "idle"

    def dispatch(self, action: str, args: dict) -> dict | None:
        if action == "execute":
            return self._execute(args.get("command", ""),
                                 bool(args.get("auto_start", True)))
        if action in ("start", "info"):
            return {"state": "active", "action_server": self._srv_name}
        if action == "stop":
            return {"state": "idle"}
        return None

    def _execute(self, command: str, auto_start: bool) -> dict:
        if not command:
            return {"state": "error", "message": "command required"}
        if not self._action_client.wait_for_server(timeout_sec=3.0):
            return {"state": "error", "message": f"{self._srv_name} not available"}
        goal = Behavior.Goal()
        goal.command = command
        goal.auto_start = auto_start
        future = self._action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self._node, future, timeout_sec=5.0)
        goal_handle = future.result()
        if not goal_handle.accepted:
            return {"state": "error", "message": "behavior goal rejected"}
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self._node, result_future, timeout_sec=30.0)
        result = result_future.result()
        return {"state": "ok" if result.result == Behavior.Result.SUCCESS else "error",
                "message": result.message, "command": command}


# ── GraspObjectPlugin (actuator) — object grasping ──────────────────────────

class GraspObjectPlugin:
    """物体抓取 — action server 名称需在真机上确认。

    GraspObject Goal: arg (string), cmd (string)
    """

    def __init__(self, plugin_config: dict, namespace: str, executor):
        self._ns = namespace
        self._node = _Q5Node("grasp")
        executor.add_node(self._node)

        srv_name = plugin_config.get("action_server", "/grasp_object_action")
        self._action_client = ActionClient(self._node, GraspObject, srv_name)
        self._srv_name = srv_name
        self._state = "idle"

    def get_tool(self) -> dict:
        return {
            "name": "grasp",
            "type": "actuator",
            "multiInstance": False,
            "description": "Q5 object grasping — trigger grasp commands "
                           "via GraspObject action.",
            "inputSchema": {"type": "object", "properties": {
                "action": {"type": "string", "enum": ["start", "stop", "execute", "info"],
                           "default": "execute"},
                "cmd": {"type": "string", "description": "Grasp command (e.g. 'grasp', 'release')"},
                "arg": {"type": "string", "description": "Grasp argument (e.g. object name)"},
            }},
            "default_action": "execute",
        }

    def start(self) -> None:
        self._state = "active"

    def stop(self) -> None:
        self._state = "idle"

    def dispatch(self, action: str, args: dict) -> dict | None:
        if action == "execute":
            return self._execute(args.get("cmd", ""), args.get("arg", ""))
        if action in ("start", "info"):
            return {"state": "active", "action_server": self._srv_name}
        if action == "stop":
            return {"state": "idle"}
        return None

    def _execute(self, cmd: str, arg: str) -> dict:
        if not cmd:
            return {"state": "error", "message": "cmd required"}
        if not self._action_client.wait_for_server(timeout_sec=3.0):
            return {"state": "error", "message": f"{self._srv_name} not available"}
        goal = GraspObject.Goal()
        goal.cmd = cmd
        goal.arg = arg
        future = self._action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self._node, future, timeout_sec=5.0)
        goal_handle = future.result()
        if not goal_handle.accepted:
            return {"state": "error", "message": "grasp goal rejected"}
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self._node, result_future, timeout_sec=30.0)
        result = result_future.result()
        return {"state": "ok", "cmd": cmd, "arg": arg, "result": str(result)}


# ── MotionActionPlugin (actuator) — motion priority dispatch ────────────────

class MotionActionPlugin:
    """运动动作分发 — action server 名称需在真机上确认。

    Motion Goal: motion_name (string), priority (int8)
      priority 常量: EMERGENCY, HIGH, LOW, MEDIUM, SYSTEM
    """

    _PRIORITY_NAMES = {}

    def __init__(self, plugin_config: dict, namespace: str, executor):
        self._ns = namespace
        self._node = _Q5Node("motion_action")
        executor.add_node(self._node)

        srv_name = plugin_config.get("action_server", "/motion_action")
        self._action_client = ActionClient(self._node, Motion, srv_name)
        self._srv_name = srv_name
        self._state = "idle"
        # Populate priority names from constants if available
        try:
            self._PRIORITY_NAMES = {
                Motion.Goal.EMERGENCY: "EMERGENCY",
                Motion.Goal.HIGH: "HIGH",
                Motion.Goal.LOW: "LOW",
                Motion.Goal.MEDIUM: "MEDIUM",
                Motion.Goal.SYSTEM: "SYSTEM",
            }
        except AttributeError:
            self._PRIORITY_NAMES = {0: "EMERGENCY", 1: "HIGH", 2: "LOW", 3: "MEDIUM", 4: "SYSTEM"}

    def get_tool(self) -> dict:
        return {
            "name": "motion_action",
            "type": "actuator",
            "multiInstance": False,
            "description": "Q5 motion action — trigger named motions with priority "
                           "via Motion action.",
            "inputSchema": {"type": "object", "properties": {
                "action": {"type": "string", "enum": ["start", "stop", "execute", "info"],
                           "default": "execute"},
                "motion_name": {"type": "string", "description": "Name of the motion to trigger"},
                "priority": {"type": "integer", "description": "Priority level (EMERGENCY=0, HIGH=1, LOW=2, MEDIUM=3, SYSTEM=4)"},
            }},
            "default_action": "execute",
        }

    def start(self) -> None:
        self._state = "active"

    def stop(self) -> None:
        self._state = "idle"

    def dispatch(self, action: str, args: dict) -> dict | None:
        if action == "execute":
            return self._execute(args.get("motion_name", ""),
                                 int(args.get("priority", 3)))
        if action in ("start", "info"):
            return {"state": "active", "action_server": self._srv_name,
                    "priorities": self._PRIORITY_NAMES}
        if action == "stop":
            return {"state": "idle"}
        return None

    def _execute(self, motion_name: str, priority: int) -> dict:
        if not motion_name:
            return {"state": "error", "message": "motion_name required"}
        if not self._action_client.wait_for_server(timeout_sec=3.0):
            return {"state": "error", "message": f"{self._srv_name} not available"}
        goal = Motion.Goal()
        goal.motion_name = motion_name
        goal.priority = priority
        future = self._action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self._node, future, timeout_sec=5.0)
        goal_handle = future.result()
        if not goal_handle.accepted:
            return {"state": "error", "message": "motion goal rejected"}
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self._node, result_future, timeout_sec=30.0)
        result = result_future.result()
        return {"state": "ok", "motion_name": motion_name,
                "priority": self._PRIORITY_NAMES.get(priority, str(priority)),
                "result": str(result)}
