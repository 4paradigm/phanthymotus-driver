#!/usr/bin/env python3
"""
x-humanoid/tianyi2.0/device.py — 天轶2.0 Pro 设备插件。

设计原则：
  - 一个设备 = 一个 tool (或 multi-tool plugin)
  - sensor：只读，驱动启动时自动 start，数据通过 ROS2 topic 输出 (domain 42)
  - actuator：action 参数分发操作，通过 ROS2 发布指令到天轶 (domain 0)
  - resource：返回静态数据 (如 URDF)
  - 角度对外用度(degrees)，内部转弧度(rad)发送

双 Domain 模式：
  - domain 0 (ros2.ctx_tianyi): 订阅天轶本体话题、发布控制指令
  - domain 42 (ros2.ctx_core): 发布传感器数据给 Agent Core

插件列表：
  StatePlugin         (sensor, multi-tool) — 关节/电池/急停/力传感器/URDF
  CameraPlugin        (sensor)             — Orbbec 头部相机
  AsrPlugin           (sensor)             — 语音识别结果
  NavStatePlugin      (sensor)             — 底盘导航状态
  PowerBoardStatePlugin (sensor)          — 电源板MOS温度/电流/电压
  HeadPlugin          (actuator)           — 头部3DOF控制
  HeadGesturePlugin   (actuator)           — 点头/摇头/左右观察等语义动作
  ArmPlugin           (actuator)           — 双臂14DOF控制
  ArmGesturePlugin    (actuator)           — 挥手/敬礼/欢迎等语义动作
  WaistPlugin         (actuator)           — 腰部2DOF控制
  HandPlugin          (actuator)           — 灵巧手控制
  TtsPlugin           (actuator)           — 语音合成
  VoicePlayActuatorPlugin (actuator)      — 音频播放控制(文件/URL/TTS)
  NavPlugin           (actuator)           — 底盘导航控制
  HomePlugin          (actuator)           — 充电桩管理与回桩
  ChatPlugin          (actuator)           — 语音交互开关
  VoiceChatActuatorPlugin (actuator)      — 语音对话开关
  MotorStatePlugin    (sensor)             — 全身21电机状态(2Hz)
  HandStatePlugin     (sensor)             — 灵巧手状态(10Hz, tool name=hand_state)
  RemoteStatePlugin   (sensor)             — 遥控器SBUS事件(5Hz)
  HealthCheckPlugin   (actuator)           — 全身体检卡 (tool name=health_check)
  LaserScanPlugin     (sensor)             — 激光雷达原始点云
  ChassisRawPlugin    (actuator)           — 底盘速度控制
  ControlledSpatialPlugin (actuator)      — 空间控制(controlled_spatial)
  ExtMicPlugin        (actuator)           — 外部麦克风(ext_mic)
  LightPlugin         (actuator)           — 灯光控制
  StatePlugin      (sensor, multi-tool) — 关节/电池/急停/力传感器/URDF
  CameraPlugin     (sensor)             — Orbbec 头部相机
  AsrPlugin        (sensor)             — 语音识别结果
  NavStatePlugin   (sensor)             — 底盘导航状态
  HeadPlugin       (actuator)           — 头部3DOF控制
  HeadGesturePlugin (actuator)          — 点头/摇头/左右观察等语义动作
  ArmPlugin        (actuator)           — 双臂14DOF控制
  ArmGesturePlugin (actuator)           — 挥手/敬礼/欢迎等语义动作
  WaistPlugin      (actuator)           — 腰部2DOF控制
  HandPlugin       (actuator)           — 灵巧手控制
  TtsPlugin        (actuator)           — 语音合成
  NavPlugin        (actuator)           — 底盘导航控制
  ChatPlugin       (actuator)           — 语音交互开关
  ControlledSpatialPlugin (actuator)    — 人工控制建图与导航 (Slamtec REST API)
"""

from __future__ import annotations

import json
import math
import re as _re
import subprocess
import struct
import threading
import time
from pathlib import Path
from uuid import uuid4

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import String, Bool, UInt32MultiArray

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

# ── Motor ID → Joint Name 映射 ───────────────────────────────────────────────

_HEAD_JOINTS = {
    1: "head_roll_joint",
    2: "head_pitch_joint",
    3: "head_yaw_joint",
}

_ARM_LEFT_JOINTS = {
    11: "left_shoulder_pitch_joint",
    12: "left_shoulder_roll_joint",
    13: "left_shoulder_yaw_joint",
    14: "left_elbow_pitch_joint",
    15: "left_wrist_yaw_joint",
    16: "left_wrist_pitch_joint",
    17: "left_wrist_roll_joint",
}

_ARM_RIGHT_JOINTS = {
    21: "right_shoulder_pitch_joint",
    22: "right_shoulder_roll_joint",
    23: "right_shoulder_yaw_joint",
    24: "right_elbow_pitch_joint",
    25: "right_wrist_yaw_joint",
    26: "right_wrist_pitch_joint",
    27: "right_wrist_roll_joint",
}

# Rated motor currents from the Tianyi 2.0 joint specification table. These
# values are used as the current limits in CmdSetMotorPosition commands.
_RATED_MOTOR_CURRENT_A = {
    1: 5.0, 2: 5.0, 3: 5.0,
    11: 35.0, 12: 23.0, 13: 8.0, 14: 8.0,
    15: 8.0, 16: 5.0, 17: 5.0,
    21: 35.0, 22: 23.0, 23: 8.0, 24: 8.0,
    25: 8.0, 26: 5.0, 27: 5.0,
}

_WAIST_JOINTS = {
    31: "waist_yaw_joint",
    32: "waist_pitch_joint",
}

_LEG_JOINTS = {
    51: "hip_pitch_joint",
    52: "knee_pitch_joint",
}

_ALL_JOINTS = {**_HEAD_JOINTS, **_ARM_LEFT_JOINTS, **_ARM_RIGHT_JOINTS, **_WAIST_JOINTS, **_LEG_JOINTS}

# Inspire feedback is folded directly into the joints skeleton. The vendor
# state messages report one normalized value for each of these six channels.
_SKELETON_HAND_ORDER = ["little", "ring", "middle", "index", "thumb_bend", "thumb_rotation"]
_SKELETON_HAND_ALIASES = {
    "1": "little", "little": "little", "pinky": "little",
    "2": "ring", "ring": "ring",
    "3": "middle", "middle": "middle",
    "4": "index", "index": "index", "fore": "index",
    "5": "thumb_bend", "thumb": "thumb_bend", "thumb_flex": "thumb_bend",
    "6": "thumb_rotation", "thumb_rotation": "thumb_rotation", "thumb_rotate": "thumb_rotation",
}
_SKELETON_HAND_JOINTS = {
    side: {
        "little": f"{side}_hand_little_joint",
        "ring": f"{side}_hand_ring_joint",
        "middle": f"{side}_hand_middle_joint",
        "index": f"{side}_hand_index_joint",
        "thumb_bend": f"{side}_hand_thumb_bend_joint",
        "thumb_rotation": f"{side}_hand_thumb_rotation_joint",
    }
    for side in ("left", "right")
}
_SKELETON_HAND_MAX_BEND_RAD = math.pi / 2.0
_RIGHT_THUMB_ROTATION_OPEN_RAW = 0.48

# Calibrated full-height encoder values. Unlike process-startup auto-zeroing,
# these keep the high and low lift poses consistent after a driver restart.
_SKELETON_JOINT_OFFSET = {
    "waist_pitch_joint": 0.70,
    "hip_pitch_joint": -0.70,
    "knee_pitch_joint": 0.35,
}
_SKELETON_JOINT_GAIN = {
    "hip_pitch_joint": 2.44,
    "knee_pitch_joint": 1.37,
}

_MOTOR_ERROR_DESCRIPTIONS = {
    1: "motor_over_temperature",
    2: "motor_over_current",
    3: "motor_under_voltage",
    4: "mos_over_temperature",
    5: "motor_stall",
    6: "motor_over_voltage",
    7: "motor_phase_loss",
    8: "encoder_error",
    33072: "device_offline",
    33073: "joint_position_out_of_range",
}


def _deg2rad(deg: float) -> float:
    return deg * math.pi / 180.0


def _rad2deg(rad: float) -> float:
    return rad * 180.0 / math.pi


def _skeleton_hand_bend_rad(raw: float, side: str, finger: str) -> float:
    """Map Inspire's open-ratio feedback to the virtual skeleton angle."""
    if -0.05 <= raw <= 1.05:
        open_ratio = max(0.0, min(1.0, raw))
    else:
        open_ratio = max(0.0, min(100.0, raw)) / 100.0
    if side == "right" and finger == "thumb_rotation":
        open_ratio = max(0.0, min(1.0, open_ratio / _RIGHT_THUMB_ROTATION_OPEN_RAW))
    return (1.0 - open_ratio) * _SKELETON_HAND_MAX_BEND_RAD


def _clamp(value: float, lower: float, upper: float) -> float:
    """Clamp a numeric input to a safe, documented range."""
    return max(lower, min(upper, float(value)))


def _rpm2rads(rpm: float) -> float:
    return rpm * 2.0 * math.pi / 60.0


# ── 关节限位 (deg, rpm, A): motor_id → (min_deg, max_deg, max_spd_rpm, rated_current_a) ─

_JOINT_LIMITS = {
    # 腰部
    31: (-160,   180,   30,  31.0),
    32: (-45,    120,   37.5, 82.0),
    # 左腿
    51: (-40,    5,     37.5, 5.0),
    52: (-23,    20,    37.5, 5.0),
}

# ── 腿部升降标定点位 (实测, rad) ──
# 51(hip) + 52(knee) ≈ -0.35, 32(pitch) ≈ -51, 三电机联动保证平稳升降
_LEG_LEVELS = [
    {},  # 占位, level 从 1 开始
    {"level": 1, 51:  0.08709, 52: -0.35002, 32: -0.08704},   # 归零位
    {"level": 2, 51: -0.08720, 52: -0.26279, 32:  0.08728},
    {"level": 3, 51: -0.17443, 52: -0.17557, 32:  0.17449},
    {"level": 4, 51: -0.26170, 52: -0.08832, 32:  0.26174},
    {"level": 5, 51: -0.34893, 52: -0.00107, 32:  0.34897},
    {"level": 6, 51: -0.43613, 52:  0.08618, 32:  0.43620},
    {"level": 7, 51: -0.52336, 52:  0.17335, 32:  0.52342},
    {"level": 8, 51: -0.61061, 52:  0.26061, 32:  0.61062},
    {"level": 9, 51: -0.69785, 52:  0.34785, 32:  0.69785},   # 最高位
]


def _acp_notify(action_id: str, status: str, result: dict, tool: str = ""):
    """POST action completion to Agent Core (module-level ACP helper)."""
    import urllib.request as _urllib
    import ssl as _ssl
    import os as _os

    agent_core_url = _os.environ.get("AGENT_CORE_URL", "https://localhost:15678")
    ctx = _ssl.create_default_context()
    ctx.check_hostname = False
    ctx.verify_mode = _ssl.CERT_NONE
    payload = json.dumps({
        "action_id": action_id,
        "status": status,
        "result": result,
        "tool": tool,
        "ts": time.time(),
    }).encode()
    try:
        req = _urllib.Request(
            f"{agent_core_url}/api/acp/complete",
            data=payload,
            headers={"Content-Type": "application/json"},
            method="POST",
        )
        _urllib.urlopen(req, timeout=5, context=ctx)
    except Exception as e:
        import sys
        print(f"[ACP] callback failed for {action_id}: {e}", file=sys.stderr)


class _ActionSequence:
    """Run one cancellable actuator sequence at a time."""

    def __init__(self, name: str):
        self._name = name
        self._lock = threading.Lock()
        self._cancel_event: threading.Event | None = None
        self._thread: threading.Thread | None = None

    def start(self, worker, on_done=None) -> None:
        """Start a worker sequence. on_done(cancelled: bool) called when finished."""
        self.cancel()
        cancel_event = threading.Event()

        def _run():
            try:
                worker(cancel_event)
            except Exception as e:
                print(f"[{self._name}] sequence failed: {e}")
            finally:
                cancelled = cancel_event.is_set()
                with self._lock:
                    if self._cancel_event is cancel_event:
                        self._cancel_event = None
                        self._thread = None
                if on_done:
                    try:
                        on_done(cancelled)
                    except Exception as e:
                        print(f"[{self._name}] on_done callback failed: {e}")

        thread = threading.Thread(
            target=_run, name=f"{self._name}_sequence", daemon=True)
        with self._lock:
            self._cancel_event = cancel_event
            self._thread = thread
        thread.start()

    def cancel(self) -> bool:
        with self._lock:
            cancel_event = self._cancel_event
            thread = self._thread
        if cancel_event is None:
            return False
        cancel_event.set()
        if thread and thread is not threading.current_thread():
            thread.join(timeout=1.0)
        with self._lock:
            if self._cancel_event is cancel_event:
                self._cancel_event = None
                self._thread = None
        return True


class _JointCommandFeedback:
    """Shared status, safety preflight, and motion feedback for raw joint cards."""

    _STATUS_MAX_AGE = 2.0
    _FEEDBACK_TIMEOUT = 2.0
    _MOVE_THRESHOLD_RAD = _deg2rad(0.5)
    _TARGET_TOLERANCE_RAD = _deg2rad(3.0)

    def __init__(self, subsystem: str, status_topic: str):
        self._subsystem = subsystem
        self._status_topic = status_topic
        self._condition = threading.Condition()
        self._status: dict[int, dict] = {}
        self._status_seq = 0
        self._status_time: float | None = None
        self._power_status: dict = {}
        self._power_status_time: float | None = None

    def create_subscriptions(self, node: Node) -> None:
        from bodyctrl_msgs.msg import MotorStatusMsg, PowerBoardKeyStatus
        node.create_subscription(
            MotorStatusMsg, self._status_topic,
            self._on_motor_status, _RELIABLE_QOS)
        node.create_subscription(
            PowerBoardKeyStatus, "/power/board/key_status",
            self._on_power_status, _RELIABLE_QOS)

    def _on_motor_status(self, msg) -> None:
        now = time.monotonic()
        with self._condition:
            self._status = {
                int(motor.name): {
                    "pos": float(motor.pos),
                    "speed": float(motor.speed),
                    "current": float(motor.current),
                    "temperature": float(motor.temperature),
                    "error": int(motor.error),
                }
                for motor in msg.status
            }
            self._status_seq += 1
            self._status_time = now
            self._condition.notify_all()

    def _on_power_status(self, msg) -> None:
        now = time.monotonic()
        with self._condition:
            self._power_status = {
                "is_estop": bool(msg.is_estop.data),
                "is_remote_estop": bool(msg.is_remote_estop.data),
                "is_power_on": bool(msg.is_power_on.data),
            }
            self._power_status_time = now
            self._condition.notify_all()

    @staticmethod
    def _error(code: str, message: str, **details) -> dict:
        result = {"state": "error", "error": message, "code": code}
        result.update(details)
        return result

    def _faults(self, motor_ids: list[int]) -> list[dict]:
        faults = []
        for motor_id in motor_ids:
            status = self._status.get(motor_id)
            if status is None or status["error"] == 0:
                continue
            error_code = status["error"]
            faults.append({
                "motor_id": motor_id,
                "joint": _ALL_JOINTS.get(motor_id, f"motor_{motor_id}"),
                "error_code": error_code,
                "description": _MOTOR_ERROR_DESCRIPTIONS.get(
                    error_code, "unknown_vendor_error"),
            })
        return faults

    def preflight(self, publisher, motor_ids: list[int]) -> dict | None:
        if not publisher:
            return self._error(
                "publisher_not_initialized",
                f"{self._subsystem} command publisher is not initialized")
        now = time.monotonic()
        with self._condition:
            if self._status_time is None:
                return self._error(
                    f"{self._subsystem}_status_unavailable",
                    f"No {self._status_topic} received; "
                    f"{self._subsystem} controller may not be running",
                    diagnosis=[
                        "check robot body-control program",
                        "complete robot self-check and confirm Ready state",
                        f"check ROS_DOMAIN_ID and {self._status_topic}",
                    ],
                )
            status_age = now - self._status_time
            if status_age > self._STATUS_MAX_AGE:
                return self._error(
                    f"{self._subsystem}_status_stale",
                    f"{self._status_topic} is stale ({status_age:.2f}s)",
                    diagnosis=[
                        "check robot body-control program",
                        "check ROS communication",
                    ],
                )
            missing = [
                motor_id for motor_id in motor_ids
                if motor_id not in self._status
            ]
            if missing:
                return self._error(
                    f"{self._subsystem}_motors_missing",
                    f"Selected {self._subsystem} motors are missing from "
                    f"{self._status_topic}",
                    missing_motor_ids=missing,
                )
            faults = self._faults(motor_ids)
            if faults:
                return self._error(
                    f"{self._subsystem}_motor_fault",
                    f"Selected {self._subsystem} has active motor faults",
                    faults=faults,
                )
            if (self._power_status_time is not None
                    and now - self._power_status_time <= self._STATUS_MAX_AGE):
                if (self._power_status.get("is_estop")
                        or self._power_status.get("is_remote_estop")):
                    return self._error(
                        "emergency_stop_active",
                        "Physical or remote emergency stop is active",
                        power_status=dict(self._power_status),
                    )
                if not self._power_status.get("is_power_on", True):
                    return self._error(
                        "robot_power_off",
                        "Robot power board reports power off",
                        power_status=dict(self._power_status),
                    )
        return None

    def snapshot(self, motor_ids: list[int]) -> tuple[int, dict[int, float]]:
        with self._condition:
            return self._status_seq, {
                motor_id: self._status[motor_id]["pos"]
                for motor_id in motor_ids
                if motor_id in self._status
            }

    def wait_for_motion(
            self, targets: dict[int, float], baseline_seq: int,
            baseline: dict[int, float]) -> dict:
        motor_ids = list(targets)
        deadline = time.monotonic() + self._FEEDBACK_TIMEOUT
        received_new_status = False
        with self._condition:
            while True:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    break
                if self._status_seq <= baseline_seq:
                    self._condition.wait(remaining)
                    continue
                received_new_status = True
                faults = self._faults(motor_ids)
                if faults:
                    return self._error(
                        f"{self._subsystem}_motor_fault_after_command",
                        f"{self._subsystem.capitalize()} motor fault appeared "
                        "after command",
                        faults=faults,
                    )
                positions = {
                    motor_id: self._status[motor_id]["pos"]
                    for motor_id in motor_ids
                }
                moved = max(
                    abs(positions[motor_id] - baseline[motor_id])
                    for motor_id in motor_ids
                )
                target_error = max(
                    abs(positions[motor_id] - targets[motor_id])
                    for motor_id in motor_ids
                )
                if (moved >= self._MOVE_THRESHOLD_RAD
                        or target_error <= self._TARGET_TOLERANCE_RAD):
                    return {
                        "state": "moving",
                        "status_topic": self._status_topic,
                        "max_movement_deg": round(_rad2deg(moved), 2),
                        "max_target_error_deg": round(
                            _rad2deg(target_error), 2),
                    }
                self._condition.wait(0.05)
        if not received_new_status:
            return self._error(
                f"{self._subsystem}_feedback_timeout",
                f"Command was published but no new {self._status_topic} "
                "was received",
                diagnosis=[
                    f"check {self._subsystem} controller and ROS communication",
                    "confirm robot self-check completed and robot is Ready",
                ],
            )
        return self._error(
            f"{self._subsystem}_no_motion",
            f"Command was published and {self._subsystem} status updated, "
            "but no selected joint moved",
            diagnosis=[
                "robot may not be Ready or self-check may be incomplete",
                f"{self._subsystem} controller may be disabled or rejecting commands",
                f"another node may be publishing competing "
                f"/{self._subsystem}/cmd_pos commands",
            ],
        )


# ══════════════════════════════════════════════════════════════════════════════
# StatePlugin (sensor, multi-tool)
# ══════════════════════════════════════════════════════════════════════════════

class StatePlugin:
    """关节状态 + 电池 + 急停 + 力传感器 + URDF 模型"""

    def __init__(self, plugin_config: dict, namespace: str, ros2):
        self._ns = namespace
        self._ros2 = ros2
        self._running = False

        # Cached state
        self._joint_data = {}  # motor_id → {pos, speed, current, temp, error}
        self._hand_data = {"left": None, "right": None}
        self._battery = {}
        self._estop = {}
        self._force_left = {}
        self._force_right = {}
        self._lock = threading.Lock()
        self._joints_only = bool(plugin_config.get("joints_only", False))
        self._publish_joints_enabled = bool(plugin_config.get("publish_joints", True))
        self._last_joints_publish_at = 0.0
        self._joints_publish_interval = 1.0 / 30.0

        self._hand_left_topic = plugin_config.get("hand_left_topic", "/inspire_hand/state/left_hand")
        self._hand_right_topic = plugin_config.get("hand_right_topic", "/inspire_hand/state/right_hand")
        self._hand_stale_after_sec = float(plugin_config.get("hand_stale_after_sec", 1.0))

        # Topics for Agent Core (domain 42)
        self._topic_joints = f"/{namespace}/state/joints"
        self._topic_battery = f"/{namespace}/state/battery"
        self._topic_estop = f"/{namespace}/state/estop"
        self._topic_force = f"/{namespace}/state/force"

        # Subscriber node (domain 0 - tianyi)
        sub_name = "tianyi2_joints_sub" if self._joints_only else "tianyi2_state_sub"
        self._sub_node = Node(sub_name, context=ros2.ctx_tianyi)
        ros2.executor_tianyi.add_node(self._sub_node)

        # Publisher node (domain 42 - agent core)
        pub_name = "tianyi2_joints_pub" if self._joints_only else "tianyi2_state_pub"
        self._pub_node = Node(pub_name, context=ros2.ctx_core)
        ros2.executor_core.add_node(self._pub_node)

        self._pub_joints = (
            self._pub_node.create_publisher(String, self._topic_joints, _LOW_LAT_QOS)
            if self._publish_joints_enabled else None
        )
        self._pub_battery = None
        self._pub_estop = None
        self._pub_force = None
        if not self._joints_only:
            self._pub_battery = self._pub_node.create_publisher(String, self._topic_battery, _LOW_LAT_QOS)
            self._pub_estop = self._pub_node.create_publisher(String, self._topic_estop, _LOW_LAT_QOS)
            self._pub_force = self._pub_node.create_publisher(String, self._topic_force, _LOW_LAT_QOS)

        # URDF path
        self._urdf_path = Path(__file__).parent / "resource" / "tianyi2_model.urdf"

    def get_tools(self) -> list:
        return [
            {
                "name": "joints",
                "type": "sensor",
                "description": "天轶2.0 全身关节状态 — 身体关节与 Inspire 灵巧手实时 skeleton 渲染",
                "inputSchema": {"type": "object", "properties": {}},
                "topic_out": [{"topic": self._topic_joints, "format": "sensor/skeleton"}],
            },
            {
                "name": "battery",
                "type": "sensor",
                "description": "天轶2.0 电池状态 — 电压/电流/电量 (大电池 + 小电池)",
                "inputSchema": {"type": "object", "properties": {}},
                "topic_out": [{"topic": self._topic_battery, "format": "data/json"}],
            },
            {
                "name": "estop",
                "type": "sensor",
                "description": "天轶2.0 急停和电源状态 — 急停按钮/软急停/电源/工作时间",
                "inputSchema": {"type": "object", "properties": {}},
                "topic_out": [{"topic": self._topic_estop, "format": "data/json"}],
            },
            {
                "name": "force_sensor",
                "type": "sensor",
                "description": "天轶2.0 六维力传感器 — 双腕力/力矩 (左/右 各3力+3力矩)",
                "inputSchema": {"type": "object", "properties": {}},
                "topic_out": [{"topic": self._topic_force, "format": "data/json"}],
            },
            {
                "name": "model",
                "type": "resource",
                "description": "天轶2.0 URDF 骨架模型 — 用于3D可视化",
                "inputSchema": {"type": "object", "properties": {}},
            },
        ]

    def start(self):
        self._running = True
        try:
            from bodyctrl_msgs.msg import MotorStatusMsg, PowerBatteryStatus, PowerBoardKeyStatus
            from geometry_msgs.msg import WrenchStamped

            # Subscribe to motor status topics
            for topic in ["/head/status", "/arm/status", "/waist/status", "/leg/status"]:
                self._sub_node.create_subscription(
                    MotorStatusMsg, topic, self._on_motor_status, _RELIABLE_QOS)

            if not self._joints_only:
                # Battery
                self._sub_node.create_subscription(
                    PowerBatteryStatus, "/power/battery/status", self._on_battery, _RELIABLE_QOS)

                # E-stop
                self._sub_node.create_subscription(
                    PowerBoardKeyStatus, "/power/board/key_status", self._on_estop, _RELIABLE_QOS)

                # Force sensors (100Hz, throttle to 5Hz in callback)
                self._sub_node.create_subscription(
                    WrenchStamped, "/arm_6dof_left", self._on_force_left, _RELIABLE_QOS)
                self._sub_node.create_subscription(
                    WrenchStamped, "/arm_6dof_right", self._on_force_right, _RELIABLE_QOS)

            print("[StatePlugin] subscriptions created")
        except ImportError as e:
            print(f"[StatePlugin] WARNING: msg import failed ({e}), running in stub mode")

        try:
            from sensor_msgs.msg import JointState
            self._sub_node.create_subscription(
                JointState, self._hand_left_topic,
                lambda message: self._on_hand_state("left", message), _LOW_LAT_QOS)
            self._sub_node.create_subscription(
                JointState, self._hand_right_topic,
                lambda message: self._on_hand_state("right", message), _LOW_LAT_QOS)
            print(f"[StatePlugin] hand subscriptions created: {self._hand_left_topic}, {self._hand_right_topic}")
        except ImportError as e:
            print(f"[StatePlugin] WARNING: hand skeleton disabled ({e})")

        # Publish timer
        self._pub_thread = threading.Thread(target=self._publish_loop, daemon=True)
        self._pub_thread.start()

    def stop(self):
        self._running = False

    def _on_motor_status(self, msg):
        with self._lock:
            for s in msg.status:
                self._joint_data[s.name] = {
                    "pos": s.pos,
                    "speed": s.speed,
                    "current": s.current,
                    "temp": s.temperature,
                    "error": s.error,
                }
            self._publish_joints_locked()

    def _on_hand_state(self, side: str, msg) -> None:
        values = {finger: None for finger in _SKELETON_HAND_ORDER}
        velocities = {finger: None for finger in _SKELETON_HAND_ORDER}
        efforts = {finger: None for finger in _SKELETON_HAND_ORDER}
        names = list(msg.name or [])
        positions = list(msg.position or [])
        velocity_values = list(msg.velocity or [])
        effort_values = list(msg.effort or [])

        for index, finger in enumerate(_SKELETON_HAND_ORDER):
            if index < len(names):
                key = str(names[index]).strip().lower().replace("-", "_").replace(" ", "_")
                finger = _SKELETON_HAND_ALIASES.get(key, finger)
            if index < len(positions):
                values[finger] = float(positions[index])
            if index < len(velocity_values):
                velocities[finger] = float(velocity_values[index])
            if index < len(effort_values):
                efforts[finger] = float(effort_values[index])

        stamp = getattr(msg, "header", None)
        message_timestamp_ms = 0
        if stamp is not None:
            message_timestamp_ms = stamp.stamp.sec * 1000 + stamp.stamp.nanosec // 1_000_000
        with self._lock:
            self._hand_data[side] = {
                "values": values,
                "velocities": velocities,
                "efforts": efforts,
                "source_joint_names": names,
                "received_timestamp_ms": int(time.time() * 1000),
                "message_timestamp_ms": message_timestamp_ms,
            }
            self._publish_joints_locked()

    def _on_battery(self, msg):
        with self._lock:
            self._battery = {
                "master_voltage": msg.master_battery_voltage,
                "master_current": msg.master_battery_current,
                "master_power": msg.master_battery_power,
                "little_voltage": msg.little_battery_voltage,
                "little_current": msg.little_battery_current,
                "little_power": msg.little_battery_power,
                "battery_installed": msg.battery_installed,
                "battery_working": msg.battery_working,
            }

    def _on_estop(self, msg):
        with self._lock:
            self._estop = {
                "work_time": msg.work_time,
                "is_estop": msg.is_estop.data,
                "is_remote_estop": msg.is_remote_estop.data,
                "is_power_on": msg.is_power_on.data,
            }

    _force_last_pub = 0

    def _on_force_left(self, msg):
        now = time.time()
        if now - self._force_last_pub < 0.2:  # 5Hz throttle
            return
        with self._lock:
            self._force_left = {
                "fx": msg.wrench.force.x,
                "fy": msg.wrench.force.y,
                "fz": msg.wrench.force.z,
                "tx": msg.wrench.torque.x,
                "ty": msg.wrench.torque.y,
                "tz": msg.wrench.torque.z,
            }

    def _on_force_right(self, msg):
        with self._lock:
            self._force_right = {
                "fx": msg.wrench.force.x,
                "fy": msg.wrench.force.y,
                "fz": msg.wrench.force.z,
                "tx": msg.wrench.torque.x,
                "ty": msg.wrench.torque.y,
                "tz": msg.wrench.torque.z,
            }

    def _build_joints_payload_locked(self) -> dict:
        now_ms = int(time.time() * 1000)
        joints = []
        body_entries = []
        visual_q_by_name = {}

        for motor_id, data in self._joint_data.items():
            motor_key = int(motor_id) if str(motor_id).isdigit() else motor_id
            name = _ALL_JOINTS.get(motor_key, f"motor_{motor_id}")
            raw_q = data["pos"]
            q = (raw_q - _SKELETON_JOINT_OFFSET.get(name, 0.0)) * _SKELETON_JOINT_GAIN.get(name, 1.0)
            visual_q_by_name[name] = q
            body_entries.append({
                "idx": motor_id,
                "name": name,
                "q": q,
                "raw_q": raw_q,
                "dq": data["speed"],
                "current": data["current"],
                "temp": data["temp"],
            })

        if "hip_pitch_joint" in visual_q_by_name or "knee_pitch_joint" in visual_q_by_name:
            waist_q = -(
                visual_q_by_name.get("hip_pitch_joint", 0.0)
                + visual_q_by_name.get("knee_pitch_joint", 0.0)
            )
            for entry in body_entries:
                if entry["name"] == "waist_pitch_joint":
                    entry["q"] = waist_q
                    entry["source"] = "visual_compensation"
        joints.extend(body_entries)

        hands = {}
        stale_ms = int(self._hand_stale_after_sec * 1000)
        for side, hand_data in self._hand_data.items():
            if not hand_data:
                hands[side] = {"available": False, "fresh": False}
                continue

            age_ms = now_ms - hand_data["received_timestamp_ms"]
            fresh = age_ms <= stale_ms
            hands[side] = {
                "available": True,
                "fresh": fresh,
                "age_ms": age_ms,
                "source_joint_names": hand_data["source_joint_names"],
            }
            for finger, raw in hand_data["values"].items():
                if raw is None:
                    continue
                bend_q = _skeleton_hand_bend_rad(raw, side, finger)
                base_name = _SKELETON_HAND_JOINTS[side][finger]
                shared = {
                    "raw": round(raw, 4),
                    "velocity_raw": hand_data["velocities"].get(finger),
                    "effort_raw": hand_data["efforts"].get(finger),
                    "source": "inspire_hand",
                    "fresh": fresh,
                }
                q = -bend_q if finger in {"little", "ring", "middle", "index"} else bend_q
                joints.append({"idx": f"{side}_hand_{finger}", "name": base_name, "q": q, **shared})
                if finger in {"little", "ring", "middle", "index"}:
                    joints.append({
                        "idx": f"{side}_hand_{finger}_distal",
                        "name": base_name.replace("_joint", "_distal_joint"),
                        "q": -bend_q,
                        **shared,
                    })

        return {"joints": joints, "timestamp_ms": now_ms, "hands": hands}

    def _publish_joints_locked(self, force: bool = False) -> None:
        """Publish a fresh skeleton as soon as feedback arrives, capped at 30 Hz."""
        if not self._publish_joints_enabled or self._pub_joints is None or not self._joint_data:
            return
        now = time.monotonic()
        if not force and now - self._last_joints_publish_at < self._joints_publish_interval:
            return
        msg = String()
        msg.data = json.dumps(self._build_joints_payload_locked())
        self._pub_joints.publish(msg)
        self._last_joints_publish_at = now

    def _publish_loop(self):
        """Publish aggregated state at 10Hz for joints, 1Hz for battery/estop."""
        joint_counter = 0
        while self._running:
            time.sleep(0.1)  # 10Hz
            joint_counter += 1

            # Publish joints
            with self._lock:
                self._publish_joints_locked(force=True)

            # 1Hz for battery/estop/force
            if joint_counter % 10 == 0:
                with self._lock:
                    if self._battery:
                        msg = String()
                        msg.data = json.dumps(self._battery)
                        self._pub_battery.publish(msg)
                    if self._estop:
                        msg = String()
                        msg.data = json.dumps(self._estop)
                        self._pub_estop.publish(msg)

            # 5Hz for force
            if joint_counter % 2 == 0:
                with self._lock:
                    if self._force_left or self._force_right:
                        msg = String()
                        msg.data = json.dumps({"left": self._force_left, "right": self._force_right})
                        self._pub_force.publish(msg)

    def dispatch(self, action_or_tool: str, args: dict) -> dict:
        # Resource tool: model
        if action_or_tool == "model":
            try:
                urdf = self._urdf_path.read_text()
                return {"urdf": urdf}
            except FileNotFoundError:
                return {"error": "URDF file not found"}
        # Sensor tools return state
        if action_or_tool == "joints":
            with self._lock:
                return self._build_joints_payload_locked()
        if action_or_tool == "battery":
            with self._lock:
                return self._battery or {"state": "no_data"}
        if action_or_tool == "estop":
            with self._lock:
                return self._estop or {"state": "no_data"}
        if action_or_tool == "force_sensor":
            with self._lock:
                return {"left": self._force_left, "right": self._force_right}
        # start/stop/info
        if action_or_tool == "start":
            return {"state": "running"}
        if action_or_tool == "stop":
            return {"state": "idle"}
        if action_or_tool == "info":
            tool_name = args.get("_tool_name", "joints")
            topic_map = {
                "joints": self._topic_joints,
                "battery": self._topic_battery,
                "estop": self._topic_estop,
                "force_sensor": self._topic_force,
            }
            topic = topic_map.get(tool_name, self._topic_joints)
            fmt = "sensor/skeleton" if tool_name == "joints" else "data/json"
            return {"state": "running", "topic_out": [{"topic": topic, "format": fmt}]}
        return {"error": f"unknown action: {action_or_tool}"}


# ══════════════════════════════════════════════════════════════════════════════
# CameraPlugin (sensor)
# ══════════════════════════════════════════════════════════════════════════════

class CameraPlugin:
    """Orbbec 头部 RGB 相机 — 独立编码线程避免阻塞executor"""

    def __init__(self, plugin_config: dict, namespace: str, ros2):
        self._ns = namespace
        self._ros2 = ros2
        self._topic = f"/{namespace}/camera/head"
        self._running = False
        self._frame_queue = None  # Will hold latest frame only

        self._sub_node = Node("tianyi2_camera_sub", context=ros2.ctx_tianyi)
        ros2.executor_tianyi.add_node(self._sub_node)

        self._pub_node = Node("tianyi2_camera_pub", context=ros2.ctx_core)
        ros2.executor_core.add_node(self._pub_node)

    def get_tool(self) -> dict:
        return {
            "name": "camera_head",
            "type": "sensor",
            "description": "天轶2.0 头部相机 (Orbbec RGB) — 彩色图像流",
            "inputSchema": {"type": "object", "properties": {}},
            "topic_out": [{"topic": self._topic, "format": "image/jpeg"}],
        }

    def start(self):
        self._running = True

        # Ensure Orbbec camera service is running
        self._ensure_orbbec_service()

        try:
            from sensor_msgs.msg import Image, CompressedImage
            import numpy as np
            import cv2

            self._np = np
            self._cv2 = cv2
            self._latest_frame = None  # Only keep latest frame
            self._frame_lock = threading.Lock()

            # Publish JPEG as CompressedImage
            self._pub = self._pub_node.create_publisher(CompressedImage, self._topic, _LOW_LAT_QOS)

            # Subscribe - callback just grabs the frame, doesn't encode
            self._sub_node.create_subscription(
                Image, "/ob_camera_head/color/image_raw", self._on_image_grab, _RELIABLE_QOS)

            # Separate encoding thread - avoids blocking executor
            self._encode_thread = threading.Thread(target=self._encode_loop, daemon=True)
            self._encode_thread.start()

            print("[CameraPlugin] subscription + encode thread created")
        except ImportError as e:
            print(f"[CameraPlugin] WARNING: import failed ({e})")

    def _ensure_orbbec_service(self):
        """Configure and start the host's Orbbec service through ``nsenter``.

        The camera runs on the host because it owns the USB device.  Each
        container start therefore makes the vendor startup script idempotently
        request PointCloud2, accelerometer, and gyroscope streams before
        ensuring the service is active.  This is deliberately runtime setup,
        not a Docker build step: a Dockerfile cannot alter a new machine's
        systemd service or access its camera.
        """
        import subprocess
        try:
            changed = self._configure_orbbec_startup()
            # Use nsenter to run systemctl on host PID 1's namespace
            result = subprocess.run(
                ["nsenter", "-t", "1", "-m", "-u", "-i", "-n", "-p", "--",
                 "systemctl", "is-active", "orbbec_head.service"],
                capture_output=True, text=True, timeout=5)
            if result.stdout.strip() == "active" and not changed:
                print("[CameraPlugin] orbbec_head.service already active")
                return
            # Restart applies a changed host startup script; start handles an
            # inactive service without unnecessarily interrupting a live one.
            action = "restart" if result.stdout.strip() == "active" else "start"
            subprocess.run(
                ["nsenter", "-t", "1", "-m", "-u", "-i", "-n", "-p", "--",
                 "systemctl", action, "orbbec_head.service"],
                check=True, capture_output=True, text=True, timeout=15)
            print(f"[CameraPlugin] orbbec_head.service {action}ed via nsenter")
        except Exception as e:
            print(f"[CameraPlugin] WARNING: could not start orbbec service ({e})")

    @staticmethod
    def _configure_orbbec_startup():
        """Enable the Orbbec streams in the host vendor startup script.

        This only changes the ``headty`` launch command and is idempotent, so
        it is safe to execute every time a freshly-created driver container
        starts.  The host path matches Tianyi's factory service definition.
        """
        import subprocess

        script = "/home/nvidia/data/scripts/start_orbbec_camera.sh"
        launch = "ros2 launch orbbec_camera head_330_ty.launch.py"
        desired = (f"{launch} enable_point_cloud:=true "
                   "enable_accel:=true enable_gyro:=true")
        updater = (
            "from pathlib import Path\n"
            f"path = Path({script!r})\n"
            "lines = path.read_text().splitlines(keepends=True)\n"
            f"launch = {launch!r}\n"
            f"desired = {desired!r}\n"
            "for index, line in enumerate(lines):\n"
            "    if line.lstrip().startswith(launch):\n"
            "        updated = line[:len(line) - len(line.lstrip())] + desired + '\\n'\n"
            "        if line == updated:\n"
            "            print('unchanged')\n"
            "        else:\n"
            "            lines[index] = updated\n"
            "            path.write_text(''.join(lines))\n"
            "            print('changed')\n"
            "        break\n"
            "else:\n"
            "    raise SystemExit('headty Orbbec launch command not found')\n"
        )
        result = subprocess.run(
            ["nsenter", "-t", "1", "-m", "-u", "-i", "-n", "-p", "--",
             "python3", "-c", updater],
            check=True, capture_output=True, text=True, timeout=10)
        changed = result.stdout.strip() == "changed"
        if changed:
            print("[CameraPlugin] enabled Orbbec point cloud, accel, and gyro streams")
        return changed

    def stop(self):
        self._running = False

    def _on_image_grab(self, msg):
        """Callback: just grab the latest frame, don't encode here (non-blocking)."""
        if not self._running:
            return
        with self._frame_lock:
            self._latest_frame = msg

    def _encode_loop(self):
        """Separate thread: encode and publish the latest frame. Always processes newest, skips stale."""
        np = self._np
        cv2 = self._cv2
        from sensor_msgs.msg import CompressedImage

        while self._running:
            # Grab latest frame atomically
            with self._frame_lock:
                msg = self._latest_frame
                self._latest_frame = None  # Mark as consumed
            if msg is None:
                time.sleep(0.005)  # 5ms poll
                continue
            try:
                # Zero-copy: np.frombuffer on array.array directly (no bytes() copy)
                img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
                if msg.encoding == "rgb8":
                    img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
                _, jpeg = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 50])
                out = CompressedImage()
                out.format = "jpeg"
                out.data = bytes(jpeg)
                self._pub.publish(out)
            except Exception as e:
                print(f"[CameraPlugin] encode error: {e}", flush=True)

    def dispatch(self, action: str, args: dict) -> dict:
        if action == "start":
            return {"state": "running"}
        if action == "stop":
            return {"state": "idle"}
        if action == "info":
            return {"state": "running", "topic_out": [{"topic": self._topic, "format": "image/jpeg"}]}
        return {"state": "running"}


# ══════════════════════════════════════════════════════════════════════════════
# Additional sensors / indicators
# ══════════════════════════════════════════════════════════════════════════════

class _JsonSensor:
    """Small base class for a domain-0 subscription bridged as JSON on domain 42."""
    _format = "data/json"

    def _tool(self, name, description):
        return {"name": name, "type": "sensor", "description": description,
                "inputSchema": {"type": "object", "properties": {}},
                "topic_out": [{"topic": self._topic, "format": self._format}]}

    def dispatch(self, action, args):
        if action == "info":
            return {"state": "running" if self._running else "idle",
                    "topic_out": [{"topic": self._topic, "format": self._format}]}
        return {"state": "running" if self._running else "idle"}


class ImuPlugin(_JsonSensor):
    """Bridge the head Orbbec camera's accelerometer and gyroscope to Agent Core."""
    def __init__(self, plugin_config, namespace, ros2):
        self._running = False
        self._topic = f"/{namespace}/state/imu"
        self._last_pub = 0.0
        self._latest = {
            "available": False,
            "timestamp_ms": int(time.time() * 1000),
            "source": None,
            "reason": "waiting_for_upstream_imu",
        }
        self._subscriptions = []  # Keep rclpy subscriptions alive for the plugin lifetime.
        self._camera_acceleration = None
        self._camera_acceleration_timestamp_ms = None
        self._camera_angular_velocity = None
        self._camera_angular_velocity_timestamp_ms = None
        self._sub_node = Node("tianyi2_imu_sub", context=ros2.ctx_tianyi)
        self._pub_node = Node("tianyi2_imu_pub", context=ros2.ctx_core)
        ros2.executor_tianyi.add_node(self._sub_node)
        ros2.executor_core.add_node(self._pub_node)
        self._pub = self._pub_node.create_publisher(String, self._topic, _LOW_LAT_QOS)

    def get_tool(self):
        tool = self._tool("imu", "天轶2.0 头部 Orbbec IMU — 相机实际加速度与角速度")
        tool["multiInstance"] = False
        return tool

    def start(self):
        """Subscribe to the Orbbec streams enabled by ``orbbec_head.service``.

        Orbbec publishes acceleration and angular velocity separately as
        ``sensor_msgs/Imu``.  These sensor publishers use BEST_EFFORT QoS, so
        the domain-0 subscription must use the matching low-latency profile.
        Only fields physically supplied by either stream are forwarded to the
        Agent Core card; no synthetic orientation, Euler angle, or zero values
        are emitted.
        """
        from sensor_msgs.msg import Imu as RosImu
        self._running = True
        self._subscriptions = [
            self._sub_node.create_subscription(
                RosImu, "/ob_camera_head/accel/sample", self._on_camera_accel, _LOW_LAT_QOS),
            self._sub_node.create_subscription(
                RosImu, "/ob_camera_head/gyro/sample", self._on_camera_gyro, _LOW_LAT_QOS),
        ]
        # Match StatePlugin/force_sensor behaviour: publish a snapshot even
        # while the vendor IMU stream is unavailable, so Agent Core renders a
        # useful card state instead of an empty panel.
        self._pub_thread = threading.Thread(target=self._publish_snapshot_loop, daemon=True)
        self._pub_thread.start()
        print("[ImuPlugin] subscribed: /ob_camera_head/accel/sample, /ob_camera_head/gyro/sample")

    def stop(self):
        self._running = False

    def _publish_camera_imu(self):
        """Publish only the samples actually received from the camera."""
        if not self._running:
            return
        timestamps = [ts for ts in (
            self._camera_acceleration_timestamp_ms,
            self._camera_angular_velocity_timestamp_ms,
        ) if ts is not None]
        if not timestamps:
            return
        payload = {"available": True, "timestamp_ms": max(timestamps)}
        if self._camera_acceleration is not None:
            payload["linear_acceleration"] = self._camera_acceleration
        if self._camera_angular_velocity is not None:
            payload["angular_velocity"] = self._camera_angular_velocity
        self._latest = payload

        # The dashboard only needs the latest state.  Bound transport to 30 Hz
        # so a high-rate IMU cannot congest the domain-42 bridge.
        now = time.monotonic()
        if now - self._last_pub < 1.0 / 30.0:
            return
        self._last_pub = now
        out = String()
        out.data = json.dumps(self._latest)
        self._pub.publish(out)

    def _publish_snapshot_loop(self):
        while self._running:
            snapshot = dict(self._latest)
            snapshot["timestamp_ms"] = int(time.time() * 1000)
            out = String()
            out.data = json.dumps(snapshot)
            self._pub.publish(out)
            time.sleep(0.5)

    def _on_camera_accel(self, msg):
        self._camera_acceleration = {
            "x": msg.linear_acceleration.x,
            "y": msg.linear_acceleration.y,
            "z": msg.linear_acceleration.z,
        }
        self._camera_acceleration_timestamp_ms = (
            msg.header.stamp.sec * 1000 + msg.header.stamp.nanosec // 1_000_000)
        self._publish_camera_imu()

    def _on_camera_gyro(self, msg):
        self._camera_angular_velocity = {
            "x": msg.angular_velocity.x,
            "y": msg.angular_velocity.y,
            "z": msg.angular_velocity.z,
        }
        self._camera_angular_velocity_timestamp_ms = (
            msg.header.stamp.sec * 1000 + msg.header.stamp.nanosec // 1_000_000)
        self._publish_camera_imu()

    def dispatch(self, action, args):
        if action in ("read", "get", "imu"):
            # Keep sensor values at the top level like StatePlugin's
            # force_sensor, which Agent Core renders directly.
            return dict(self._latest)
        if action == "info":
            return {"state": "running" if self._running else "idle",
                    "topic_out": [{"topic": self._topic, "format": self._format}]}
        return {"state": "running" if self._running else "idle"}


class DepthCameraPlugin:
    """Bridge only the newest Orbbec Z16 frame to Agent Core."""
    def __init__(self, plugin_config, namespace, ros2):
        self._running = False; self._topic = f"/{namespace}/camera/head/depth"
        self._max_hz = max(1.0, min(float(plugin_config.get("hz", 8)), 15.0))
        self._last_published_at = 0.0
        self._forwarded_frames = 0
        self._latest_image = None
        self._latest_image_lock = threading.Lock()
        self._subscription = None
        self._publish_timer = None
        self._sub_node = Node("tianyi2_depth_sub", context=ros2.ctx_tianyi)
        self._pub_node = Node("tianyi2_depth_pub", context=ros2.ctx_core)
        ros2.executor_tianyi.add_node(self._sub_node); ros2.executor_core.add_node(self._pub_node)
    def get_tool(self):
        return {"name": "camera_depth", "type": "sensor", "description": "天轶2.0 Orbbec 头部深度图（Z16）",
                "inputSchema": {"type": "object", "properties": {}}, "topic_out": [{"topic": self._topic, "format": "image/depth-z16"}]}
    def start(self):
        from sensor_msgs.msg import Image
        import cv2
        import numpy as np
        self._cv2, self._np = cv2, np
        latest_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                                history=HistoryPolicy.KEEP_LAST, depth=1,
                                durability=DurabilityPolicy.VOLATILE)
        ingress_qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                                 history=HistoryPolicy.KEEP_LAST, depth=1,
                                 durability=DurabilityPolicy.VOLATILE)
        self._running = True
        self._pub = self._pub_node.create_publisher(Image, self._topic, latest_qos)
        self._subscription = self._sub_node.create_subscription(
            Image, "/ob_camera_head/depth/image_raw", self._on_image, ingress_qos)
        self._publish_timer = self._pub_node.create_timer(1.0 / 30.0, self._publish_latest)
        print(f"[DepthCameraPlugin] forwarding newest Z16 frame at <= {self._max_hz:g} Hz")
    def stop(self):
        self._running = False
        with self._latest_image_lock:
            self._latest_image = None
    def _on_image(self, msg):
        if not self._running or msg.encoding not in ("16UC1", "mono16"): return
        with self._latest_image_lock:
            self._latest_image = msg

    def _publish_latest(self):
        if not self._running:
            return
        with self._latest_image_lock:
            if time.monotonic() - self._last_published_at < 1.0 / self._max_hz:
                return
            msg, self._latest_image = self._latest_image, None
        if msg is None:
            return
        dashboard = self._to_dashboard_depth(msg)
        if dashboard is None:
            return
        from sensor_msgs.msg import Image
        out = Image()
        out.header = msg.header
        out.height, out.width, out.encoding = 480, 640, "16UC1"
        out.is_bigendian, out.step, out.data = 0, 1280, dashboard.tobytes()
        self._pub.publish(out)
        self._last_published_at = time.monotonic()
        self._forwarded_frames += 1

    def _to_dashboard_depth(self, msg):
        if msg.encoding not in ("16UC1", "mono16") or msg.is_bigendian:
            return None
        width, height, step = int(msg.width), int(msg.height), int(msg.step)
        if width <= 0 or height <= 0 or step < width * 2:
            return None
        raw = self._np.frombuffer(msg.data, dtype=self._np.uint8)
        needed = height * step
        if raw.size < needed:
            return None
        depth = raw[:needed].reshape(height, step)[:, :width * 2].view(self._np.uint16).reshape(height, width)
        if width == 640 and height == 480:
            return depth
        if width * 3 > height * 4:
            crop_width = height * 4 // 3
            left = (width - crop_width) // 2
            depth = depth[:, left:left + crop_width]
        elif width * 3 < height * 4:
            crop_height = width * 3 // 4
            top = (height - crop_height) // 2
            depth = depth[top:top + crop_height, :]
        return self._cv2.resize(depth, (640, 480), interpolation=self._cv2.INTER_NEAREST)
    def dispatch(self, action, args):
        return {"state": "running" if self._running else "idle", "topic_out": [{"topic": self._topic, "format": "image/depth-z16"}]}


class PointCloudPlugin:
    """Pack gravity-levelled, floor-referenced Orbbec points for Agent Core."""
    _format = "sensor/pointcloud"
    def __init__(self, plugin_config, namespace, ros2):
        self._running = False; self._topic = f"/{namespace}/camera/head/points"; self._last = 0.0; self._intrinsics = None
        self._floor_offset_m = max(-3.0, min(float(plugin_config.get("floor_offset_m", 1.50)), 3.0))
        self._gravity_world = None; self._gravity_lock = threading.Lock()
        self._sub_node = Node("tianyi2_points_sub", context=ros2.ctx_tianyi); self._pub_node = Node("tianyi2_points_pub", context=ros2.ctx_core)
        ros2.executor_tianyi.add_node(self._sub_node); ros2.executor_core.add_node(self._pub_node)
    def get_tool(self):
        return {"name": "camera_pointcloud", "type": "sensor", "description": "天轶2.0 Orbbec 头部彩色点云（限频、限点）", "inputSchema": {"type": "object", "properties": {}}, "topic_out": [{"topic": self._topic, "format": self._format}]}
    def start(self):
        from sensor_msgs.msg import PointCloud2, Image, CameraInfo, Imu
        from std_msgs.msg import UInt8MultiArray
        self._running = True; self._pub = self._pub_node.create_publisher(UInt8MultiArray, self._topic, _LOW_LAT_QOS)
        self._sub_node.create_subscription(PointCloud2, "/ob_camera_head/depth/points", self._on_cloud, _LOW_LAT_QOS)
        # Gemini 336 currently has its depth image enabled even when the vendor
        # point-cloud stream is disabled.  This fallback keeps the card live.
        self._sub_node.create_subscription(CameraInfo, "/ob_camera_head/depth/camera_info", self._on_info, _RELIABLE_QOS)
        self._sub_node.create_subscription(Image, "/ob_camera_head/depth/image_raw", self._on_depth, _LOW_LAT_QOS)
        self._sub_node.create_subscription(Imu, "/ob_camera_head/accel/sample", self._on_accel, _LOW_LAT_QOS)
    def stop(self): self._running = False
    def _on_accel(self, msg):
        """Estimate display-frame up from the camera's stationary IMU vector."""
        # Optical raw coordinates are (right, down, forward); the renderer's
        # world coordinates are (right, up, backward) before leveling.
        g = (msg.linear_acceleration.x, -msg.linear_acceleration.y, -msg.linear_acceleration.z)
        magnitude = math.sqrt(sum(v * v for v in g))
        if not 8.0 <= magnitude <= 11.5: return  # reject dynamic acceleration
        g = tuple(v / magnitude for v in g)
        with self._gravity_lock:
            previous = self._gravity_world
            if previous is None:
                self._gravity_world = g
            else:
                # Low-pass the gravity direction to avoid point-cloud wobble.
                mixed = tuple(0.95 * old + 0.05 * new for old, new in zip(previous, g))
                norm = math.sqrt(sum(v * v for v in mixed))
                self._gravity_world = tuple(v / norm for v in mixed)

    def _gravity_snapshot(self):
        with self._gravity_lock: return self._gravity_world

    @staticmethod
    def _to_renderer_frame(x, y, z, gravity=None, floor_offset_m=0.0):
        """Map optical points to the renderer and align camera up with world up."""
        # Renderer map is (input_y, -input_z, -input_x), so this packed form
        # yields world (x, -y, -z): right, up, backward from optical raw XYZ.
        wx, wy, wz = x, -y, -z
        if gravity is not None:
            gx, gy, gz = gravity
            # Rodrigues rotation taking measured up/gravity to world +Y.
            vx, vy, vz = -gz, 0.0, gx  # gravity × (0, 1, 0)
            sine_sq = vx * vx + vy * vy + vz * vz
            cosine = gy
            if sine_sq > 1e-8:
                cross_x = vy * wz - vz * wy
                cross_y = vz * wx - vx * wz
                cross_z = vx * wy - vy * wx
                cross2_x = vy * cross_z - vz * cross_y
                cross2_y = vz * cross_x - vx * cross_z
                cross2_z = vx * cross_y - vy * cross_x
                factor = (1.0 - cosine) / sine_sq
                wx += cross_x + factor * cross2_x
                wy += cross_y + factor * cross2_y
                wz += cross_z + factor * cross2_z
        wy += floor_offset_m
        # Invert the renderer map above to pack the leveled world point.
        return -wz, wx, -wy
    def _on_cloud(self, msg):
        if not self._running or time.monotonic() - self._last < 0.5: return
        fields = {f.name: f.offset for f in msg.fields}
        if not all(k in fields for k in ("x", "y", "z")): return
        self._last = time.monotonic(); raw = bytes(msg.data); count = min(msg.width * msg.height, 10000); gravity = self._gravity_snapshot()
        packed = bytearray(struct.pack("<II", 12, count))
        for i in range(count):
            base = i * msg.point_step
            x = struct.unpack_from("<f", raw, base + fields["x"])[0]
            y = struct.unpack_from("<f", raw, base + fields["y"])[0]
            z = struct.unpack_from("<f", raw, base + fields["z"])[0]
            packed.extend(struct.pack("<fff", *self._to_renderer_frame(
                x, y, z, gravity, self._floor_offset_m)))
        from std_msgs.msg import UInt8MultiArray
        out = UInt8MultiArray(); out.data = list(packed); self._pub.publish(out)
    def _on_info(self, msg): self._intrinsics = (msg.k[0], msg.k[4], msg.k[2], msg.k[5])
    def _on_depth(self, msg):
        if not self._running or self._intrinsics is None or time.monotonic() - self._last < 0.5: return
        if msg.encoding not in ("16UC1", "mono16"): return
        fx, fy, cx, cy = self._intrinsics
        if fx <= 0 or fy <= 0: return
        self._last = time.monotonic(); raw = bytes(msg.data); step = max(1, int(math.sqrt((msg.width * msg.height) / 10000))); gravity = self._gravity_snapshot()
        packed = bytearray(); count = 0
        for v in range(0, msg.height, step):
            for u in range(0, msg.width, step):
                d = struct.unpack_from("<H", raw, v * msg.step + u * 2)[0]
                if d == 0: continue
                z = d / 1000.0
                x, y = (u - cx) * z / fx, (v - cy) * z / fy
                packed.extend(struct.pack("<fff", *self._to_renderer_frame(
                    x, y, z, gravity, self._floor_offset_m))); count += 1
        if not count: return
        from std_msgs.msg import UInt8MultiArray
        out = UInt8MultiArray(); out.data = list(struct.pack("<II", 12, count) + packed); self._pub.publish(out)
    def dispatch(self, action, args): return {"state": "running" if self._running else "idle", "topic_out": [{"topic": self._topic, "format": self._format}]}


# ══════════════════════════════════════════════════════════════════════════════
# AsrPlugin (sensor)
# ══════════════════════════════════════════════════════════════════════════════

# ══════════════════════════════════════════════════════════════════════════════
# IPA phoneme matching for ASR KWS (ported from perception/plugins/asr.py)
# ══════════════════════════════════════════════════════════════════════════════

_ESPEAK_BACKENDS = {}  # lang -> EspeakBackend instance
_ESPEAK_SEP = None


def _get_espeak_backend(lang):
    global _ESPEAK_SEP
    if _ESPEAK_SEP is None:
        from phonemizer.separator import Separator
        _ESPEAK_SEP = Separator(phone=' ', word='  ', syllable='')
    if lang not in _ESPEAK_BACKENDS:
        from phonemizer.backend import EspeakBackend
        _ESPEAK_BACKENDS[lang] = EspeakBackend(lang, with_stress=False)
    return _ESPEAK_BACKENDS[lang]


def _phonemize_safe(text: str, lang: str) -> str:
    """Phonemize with persistent backend; auto-rebuild on failure."""
    backend = _get_espeak_backend(lang)
    try:
        return backend.phonemize([text], separator=_ESPEAK_SEP, strip=True)[0]
    except Exception:
        _ESPEAK_BACKENDS.pop(lang, None)
        backend = _get_espeak_backend(lang)
        return backend.phonemize([text], separator=_ESPEAK_SEP, strip=True)[0]


def _text_to_ipa(text: str) -> list:
    """Convert text to IPA phoneme sequence using persistent espeak-ng backend."""
    segments = []
    current = ''
    current_is_cjk = None
    for char in text:
        is_cjk = '\u4e00' <= char <= '\u9fff'
        if current_is_cjk is None:
            current_is_cjk = is_cjk
        if is_cjk != current_is_cjk:
            if current.strip():
                segments.append((current.strip(), current_is_cjk))
            current = ''
            current_is_cjk = is_cjk
        current += char
    if current.strip():
        segments.append((current.strip(), current_is_cjk))

    ipa_seq = []
    for seg_text, is_cjk in segments:
        lang = 'cmn' if is_cjk else 'en-us'
        try:
            ipa = _phonemize_safe(seg_text, lang)
            ipa = _re.sub(r'[0-9˥˦˧˨˩¹²³⁴⁵]', '', ipa)
            phones = [p for p in ipa.split() if p]
            ipa_seq.extend(phones)
        except Exception:
            ipa_seq.extend(list(seg_text))
    return ipa_seq


_SIMILAR_GROUPS = [
    {'t', 'd'},           # alveolar stops
    {'p', 'b'},           # bilabial stops
    {'k', 'g'},           # velar stops
    {'f', 'v'},           # labiodental fricatives
    {'s', 'z'},           # alveolar fricatives
    {'s.', 'z.'},         # retroflex fricatives
    {'ɕ', 'ʃ', 'ʂ'},     # postalveolar/retroflex sibilants
    {'tsh', 'dz'},        # affricates
    {'n', 'ŋ'},           # nasals
    {'l', 'r', 'ɹ'},      # liquids
    {'t', 'tsh'},         # stop ~ affricate
    {'f', 't'},           # common confusion in noisy env
    {'x', 'h'},           # velar/glottal fricatives
    {'ɑu', 'au', 'ɑo', 'ao'},  # diphthong variants
    {'ou', 'uo'},         # vowel variants
    {'i', 'i.'},          # apical vowel variant
    {'a', 'ɑ'},           # open vowels
    {'an', 'ɑn'},         # front nasal variants
    {'f', 'kh'},          # 范/康 confusion in noisy env
    {'f', 'x'},           # 范/欢 confusion
    {'ts.', 'tɕh'},       # retroflex/palatal affricate confusion
    {'ɑ', 'ɑu'},          # vowel truncation
    {'ai', 'a'},          # diphthong simplification
    {'aiɜ', 'ai', 'a'},   # diphthong variants
    {'iɜ', 'i'},          # rhotacized vowel
    {'əɜ', 'ə', 'e'},     # schwa variants
]


def _phoneme_sub_cost(a: str, b: str) -> float:
    """Substitution cost: 0 if same, 0.3 if similar, 1.0 otherwise."""
    if a == b:
        return 0
    for group in _SIMILAR_GROUPS:
        if a in group and b in group:
            return 0.3
    return 1.0


def _phoneme_edit_distance(seq1: list, seq2: list) -> float:
    """Normalized edit distance with phoneme similarity (0=match, 1=different)."""
    m, n = len(seq1), len(seq2)
    if m == 0 or n == 0:
        return 1.0
    dp = [[0] * (n + 1) for _ in range(m + 1)]
    for i in range(m + 1):
        dp[i][0] = i
    for j in range(n + 1):
        dp[0][j] = j
    for i in range(1, m + 1):
        for j in range(1, n + 1):
            cost = _phoneme_sub_cost(seq1[i - 1], seq2[j - 1])
            dp[i][j] = min(dp[i - 1][j] + 1, dp[i][j - 1] + 1, dp[i - 1][j - 1] + cost)
    return dp[m][n] / max(m, n)


def _find_keyword_in_ipa(text_ipa: list, keyword_ipa: list, threshold: float):
    """Sliding window search for keyword in text IPA. Returns (matched, end_position)."""
    kw_len = len(keyword_ipa)
    if kw_len == 0 or len(text_ipa) < kw_len:
        return False, -1
    best_dist = float('inf')
    best_end = -1
    for i in range(len(text_ipa) - kw_len + 1):
        window = text_ipa[i:i + kw_len]
        dist = _phoneme_edit_distance(window, keyword_ipa)
        if dist < best_dist:
            best_dist = dist
            best_end = i + kw_len
    return best_dist <= threshold, best_end


def _extract_after_keyword(text: str, keyword_text: str, end_pos: int) -> str:
    """Extract text after the matched keyword using IPA end_pos to locate cut point.

    end_pos is the IPA phoneme index where the keyword match ends.
    We map this back to the original text by counting phoneme-producing
    characters (CJK chars each produce ~3 phonemes, alpha chars ~1-2).
    """
    # Build a mapping: for each phoneme-producing character, how many IPA tokens it generates
    segments = []
    current = ''
    current_is_cjk = None
    for char in text:
        is_cjk = '\u4e00' <= char <= '\u9fff'
        is_alpha = char.isalpha()
        if not is_cjk and not is_alpha:
            continue  # skip punctuation/numbers for phoneme counting
        if current_is_cjk is None:
            current_is_cjk = is_cjk
        if is_cjk != current_is_cjk:
            if current.strip():
                segments.append((current.strip(), current_is_cjk))
            current = ''
            current_is_cjk = is_cjk
        current += char
    if current.strip():
        segments.append((current.strip(), current_is_cjk))

    # Count IPA tokens per character to find the text position for end_pos
    ipa_idx = 0
    char_idx = 0  # index into original text (counting all chars)
    phoneme_char_pos = 0  # index into phoneme-producing chars

    for seg_text, is_cjk in segments:
        lang = 'cmn' if is_cjk else 'en-us'
        try:
            ipa = _phonemize_safe(seg_text, lang)
            ipa = _re.sub(r'[0-9˥˦˧˨˩¹²³⁴⁵]', '', ipa)
            phones = [p for p in ipa.split() if p]
        except Exception:
            phones = list(seg_text)

        seg_ipa_count = len(phones)
        if ipa_idx + seg_ipa_count >= end_pos:
            # end_pos falls within this segment
            # Estimate character position within segment proportionally
            offset_in_seg = end_pos - ipa_idx
            chars_in_seg = len(seg_text)
            # For CJK, each char ≈ equal IPA tokens; for alpha, approximate
            if seg_ipa_count > 0:
                cut_chars = round(offset_in_seg * chars_in_seg / seg_ipa_count)
            else:
                cut_chars = chars_in_seg
            cut_chars = min(cut_chars, chars_in_seg)

            # Find the actual position in original text
            found = 0
            for i, c in enumerate(text):
                if '\u4e00' <= c <= '\u9fff' or c.isalpha():
                    found += 1
                if found >= phoneme_char_pos + cut_chars:
                    cut_idx = i + 1
                    remaining = text[cut_idx:]
                    remaining = remaining.lstrip('，。！？、；：,.!?;: ')
                    return remaining
            return ''
        ipa_idx += seg_ipa_count
        phoneme_char_pos += len(seg_text)

    return ''


class AsrPlugin:
    """语音识别结果 (lyre ASR)"""

    def __init__(self, plugin_config: dict, namespace: str, ros2):
        self._ns = namespace
        self._ros2 = ros2
        self._topic = f"/{namespace}/asr/text"
        self._running = False

        # KWS config (defaults: enabled, keyword=小范小范)
        self._kws_enabled = True
        self._kws_keyword = '小范小范'
        self._kws_keyword_ipa = None  # lazy init on first ASR callback
        self._kws_threshold = 0.3

        self._sub_node = Node("tianyi2_asr_sub", context=ros2.ctx_tianyi)
        ros2.executor_tianyi.add_node(self._sub_node)

        self._pub_node = Node("tianyi2_asr_pub", context=ros2.ctx_core)
        ros2.executor_core.add_node(self._pub_node)
        self._pub = self._pub_node.create_publisher(String, self._topic, _RELIABLE_QOS)

    def get_tool(self) -> dict:
        return {
            "name": "asr",
            "type": "sensor",
            "description": "天轶2.0 语音识别 (lyre ASR) — 实时语音转文字",
            "inputSchema": {"type": "object", "properties": {}},
            "configSchema": {
                "type": "object",
                "properties": {
                    "kws_enabled": {
                        "type": "boolean",
                        "description": "启用关键词唤醒（启用后仅包含唤醒词的语音会被转发）",
                        "default": True,
                        "scope": "shared",
                    },
                    "kws_keyword": {
                        "type": "string",
                        "description": "唤醒词文本（如'小范小范'、'hello robot'）",
                        "default": "小范小范",
                        "scope": "shared",
                        "x-show-when": {"kws_enabled": "true"},
                    },
                    "kws_threshold": {
                        "type": "number",
                        "description": "音素匹配阈值（0-1，越小越严格，推荐0.3）",
                        "default": 0.3,
                        "scope": "shared",
                        "x-show-when": {"kws_enabled": "true"},
                    },
                },
            },
            "topic_out": [{"topic": self._topic, "format": "data/json"}],
        }

    def start(self):
        self._running = True
        try:
            from lyre_msgs.msg import AsrIat
            self._sub_node.create_subscription(
                AsrIat, "/audio_asr/iat", self._on_asr, _RELIABLE_QOS)
            print("[AsrPlugin] subscription created")
        except ImportError:
            # Fallback: subscribe as String
            self._sub_node.create_subscription(
                String, "/audio_asr/iat", self._on_asr_string, _RELIABLE_QOS)
            print("[AsrPlugin] fallback to String subscription")

    def stop(self):
        self._running = False

    def _ensure_keyword_ipa(self):
        """Lazy-init keyword IPA on first use (phonemizer import is slow)."""
        if self._kws_keyword_ipa is None and self._kws_keyword:
            try:
                self._kws_keyword_ipa = _text_to_ipa(self._kws_keyword)
                print(f"[AsrPlugin] KWS keyword='{self._kws_keyword}' ipa={self._kws_keyword_ipa}")
            except Exception as e:
                print(f"[AsrPlugin] KWS ipa init failed: {e}")

    def _kws_filter(self, text: str) -> str | None:
        """Apply KWS filtering. Returns filtered text or None if rejected."""
        if not self._kws_enabled:
            return text
        self._ensure_keyword_ipa()
        if not self._kws_keyword_ipa:
            return text
        text_ipa = _text_to_ipa(text)
        matched, end_pos = _find_keyword_in_ipa(text_ipa, self._kws_keyword_ipa, self._kws_threshold)
        if not matched:
            return None
        remaining = _extract_after_keyword(text, self._kws_keyword, end_pos)
        if not remaining.strip():
            return None
        return remaining

    def _on_asr(self, msg):
        if not self._running:
            return
        text = msg.text
        if not text or not text.strip():
            return
        print(f"[AsrPlugin] IN: {text!r}", flush=True)
        text = self._kws_filter(text)
        if text is None:
            print(f"[AsrPlugin] KWS rejected", flush=True)
            return
        print(f"[AsrPlugin] OUT: {text!r}", flush=True)
        out = String()
        out.data = json.dumps({"id": msg.id, "text": text})
        self._pub.publish(out)

    def _on_asr_string(self, msg):
        if not self._running:
            return
        try:
            data = json.loads(msg.data)
            text = data.get("text", "")
        except (json.JSONDecodeError, AttributeError):
            text = msg.data if hasattr(msg, 'data') else str(msg)
            data = None
        if not text or not text.strip():
            return
        print(f"[AsrPlugin] IN(str): {text!r}", flush=True)
        filtered = self._kws_filter(text)
        if filtered is None:
            print(f"[AsrPlugin] KWS rejected", flush=True)
            return
        print(f"[AsrPlugin] OUT: {filtered!r}", flush=True)
        if data is not None:
            data["text"] = filtered
            out = String()
            out.data = json.dumps(data, ensure_ascii=False)
            self._pub.publish(out)
        else:
            out = String()
            out.data = filtered
            self._pub.publish(out)

    def dispatch(self, action: str, args: dict) -> dict:
        if action == "config":
            if 'kws_enabled' in args:
                self._kws_enabled = bool(args['kws_enabled'])
            if 'kws_keyword' in args:
                self._kws_keyword = args['kws_keyword']
                self._kws_keyword_ipa = _text_to_ipa(args['kws_keyword']) if args['kws_keyword'] else None
                print(f"[AsrPlugin] KWS keyword updated: '{self._kws_keyword}' ipa={self._kws_keyword_ipa}")
            if 'kws_threshold' in args:
                self._kws_threshold = float(args['kws_threshold'])
            return {"status": "configured", "kws_enabled": self._kws_enabled,
                    "kws_keyword": self._kws_keyword, "kws_threshold": self._kws_threshold}
        if action in ("start", "stop", "info"):
            return {"state": "running" if self._running else "idle",
                    "topic_out": [{"topic": self._topic, "format": "data/json"}]}
        return {"state": "running"}


# ══════════════════════════════════════════════════════════════════════════════
# PowerBoardStatePlugin (sensor) — 电源板状态卡
# ══════════════════════════════════════════════════════════════════════════════

def _temp_status(t_max: float) -> str:
    if t_max >= 75:
        return "critical"
    if t_max >= 65:
        return "hot"
    if t_max >= 55:
        return "warm"
    return "normal"


def _battery_status(power: float) -> str:
    if power < 10:
        return "critical"
    if power < 25:
        return "low"
    return "normal"


class PowerBoardStatePlugin:
    """天轶2.0 Pro 电源板状态: 1Hz。

    数据源: /power/board/status → bodyctrl_msgs/PowerStatus
    输出策略(与 plugins/power_board.py 老框架保持一致):
      - temp/current/voltage 的 max/min = 实时所有部位的聚合标量(不是历史值)
      - temp.status: normal(<55) / warm(55-65) / hot(65-75) / critical(>75)
      - battery.status: critical(<10) / low(<25) / normal(>=25)
      - 电流 0A 合法(无负载),电压 0V 异常标 unknown
    """

    def __init__(self, plugin_config: dict, namespace: str, ros2):
        self._ns = namespace
        self._ros2 = ros2
        self._topic = f"/{namespace}/state/power_board"
        self._running = False
        self._data = {}
        self._lock = threading.Lock()

        self._sub_node = Node("tianyi2_power_sub", context=ros2.ctx_tianyi)
        ros2.executor_tianyi.add_node(self._sub_node)

        self._pub_node = Node("tianyi2_power_pub", context=ros2.ctx_core)
        ros2.executor_core.add_node(self._pub_node)
        self._pub = self._pub_node.create_publisher(String, self._topic, _LOW_LAT_QOS)

    def get_tool(self) -> dict:
        return {
            "name": "power_board",
            "type": "sensor",
            "multiInstance": False,
            "readOnly": True,
            "description": (
                "天轶2.0 Pro 电源板状态(1Hz)。"
                "部位:waist/arm_a/arm_b/leg_a/leg_b(温度电压电流)+head(仅电流)+bus(母线电压)。"
                "temp.max/min = 当前所有部位 MOS 温度的实时最大/最小, temp.status: normal(<55)/warm/hot(>65)/critical(>75)。"
                "current 0A 合法(部位无负载);voltage 0V 标 unknown(未上报)。"
                "battery.power=电量%, battery.status: critical(<10)/low(<25)/normal(>=25), current 负值=放电。"
                "version.software/hardware 为字符串版本号。"
            ),
            "inputSchema": {"type": "object", "properties": {}},
            "topic_out": [{"topic": self._topic, "format": "data/json"}],
        }

    def start(self):
        self._running = True
        try:
            from bodyctrl_msgs.msg import PowerStatus
            self._sub_node.create_subscription(
                PowerStatus, "/power/board/status", self._on_power, _RELIABLE_QOS)
            print("[PowerBoardStatePlugin] subscription created")
        except ImportError as e:
            print(f"[PowerBoardStatePlugin] WARNING: import failed ({e}), running in stub mode")

        self._thread = threading.Thread(target=self._publish_loop, daemon=True)
        self._thread.start()
        print("[PowerBoardStatePlugin] publish started")

    def stop(self):
        self._running = False

    def _on_power(self, msg):
        try:
            def _num(field):
                v = getattr(msg, field, None)
                return float(v) if v is not None else None

            def _str(field):
                v = getattr(msg, field, None)
                return str(v) if v is not None else None

            temps = {
                "waist": _num("waist_temp"),
                "arm_a": _num("arm_a_temp"),
                "arm_b": _num("arm_b_temp"),
                "leg_a": _num("leg_a_temp"),
                "leg_b": _num("leg_b_temp"),
            }
            currents = {
                "waist": _num("waist_curr"),
                "arm_a": _num("arm_a_curr"),
                "arm_b": _num("arm_b_curr"),
                "leg_a": _num("leg_a_curr"),
                "leg_b": _num("leg_b_curr"),
                "head":  _num("head_curr"),
            }
            voltages = {
                "waist": _num("waist_volt"),
                "arm_a": _num("arm_a_volt"),
                "arm_b": _num("arm_b_volt"),
                "leg_a": _num("leg_a_volt"),
                "leg_b": _num("leg_b_volt"),
                "bus":   _num("bus_volt"),
            }

            def _aggregate(d: dict, keep_zero: bool):
                """实时聚合 max/min;keep_zero=False 时 0 视为未上报剔除。"""
                vals = [v for v in d.values() if v is not None and (keep_zero or v > 0)]
                return (max(vals) if vals else None, min(vals) if vals else None)

            t_max, t_min = _aggregate(temps, keep_zero=True)
            c_max, c_min = _aggregate(currents, keep_zero=True)
            v_max, v_min = _aggregate(voltages, keep_zero=False)

            # 电流 0A 合法(无负载)保留原值;电压 0V 异常标 unknown
            volt_out = {k: (v if v and v > 0 else "unknown") for k, v in voltages.items()}

            battery = {
                "voltage": _num("battery_voltage"),
                "current": _num("battery_current"),
                "power":   _num("battery_power"),
            }
            p = battery["power"]
            battery["status"] = _battery_status(p) if p is not None else "unknown"

            with self._lock:
                self._data = {
                    "temp": {**temps, "max": t_max, "min": t_min,
                             "status": _temp_status(t_max) if t_max is not None else "unknown"},
                    "current": {**currents, "max": c_max, "min": c_min},
                    "voltage": {**volt_out, "max": v_max, "min": v_min},
                    "version": {
                        "software": _str("software_version"),
                        "hardware": _str("hardware_version"),
                    },
                    "battery": battery,
                }
        except Exception as e:  # noqa: BLE001
            print(f"[PowerBoardStatePlugin] callback error: {e}")

    def _publish_loop(self):
        while self._running:
            time.sleep(1.0)  # 1Hz
            with self._lock:
                if not self._data:
                    continue
                payload = json.loads(json.dumps(self._data))  # deep copy
            payload["timestamp_ms"] = int(time.time() * 1000)
            msg = String()
            msg.data = json.dumps(payload)
            self._pub.publish(msg)

    def dispatch(self, action: str, args: dict) -> dict:
        if action in ("read", "get_power_board"):
            with self._lock:
                data = dict(self._data) if self._data else None
            if data is None:
                return {"state": "error", "error": "NO_FEEDBACK",
                        "message": "no fresh power_board state"}
            return data
        if action in ("start", "stop", "info"):
            return {"state": "running" if self._running else "idle",
                    "topic_out": [{"topic": self._topic, "format": "data/json"}]}
        return {"state": "running"}


# ══════════════════════════════════════════════════════════════════════════════
# NavStatePlugin (sensor)
# ══════════════════════════════════════════════════════════════════════════════

class NavStatePlugin:
    """底盘导航状态 — 位姿/速度 (轮询 Slamtec HTTP API)"""

    def __init__(self, plugin_config: dict, namespace: str, ros2, slamtec_client):
        self._ns = namespace
        self._ros2 = ros2
        self._slamtec = slamtec_client
        self._topic = f"/{namespace}/nav/state"
        self._running = False

        self._pub_node = Node("tianyi2_nav_state_pub", context=ros2.ctx_core)
        ros2.executor_core.add_node(self._pub_node)
        self._pub = self._pub_node.create_publisher(String, self._topic, _LOW_LAT_QOS)

    def get_tool(self) -> dict:
        return {
            "name": "nav_state",
            "type": "sensor",
            "description": "天轶2.0 底盘导航状态 — 位姿(x,y,yaw)/速度 (Slamtec底盘, 2Hz)",
            "inputSchema": {"type": "object", "properties": {}},
            "topic_out": [{"topic": self._topic, "format": "data/json"}],
        }

    def start(self):
        self._running = True
        self._thread = threading.Thread(target=self._poll_loop, daemon=True)
        self._thread.start()
        print("[NavStatePlugin] polling started")

    def stop(self):
        self._running = False

    def _poll_loop(self):
        while self._running:
            try:
                pose = self._slamtec.get_pose()
                speed = self._slamtec.get_speed()
                data = {"pose": pose, "speed": speed}
                msg = String()
                msg.data = json.dumps(data)
                self._pub.publish(msg)
            except Exception:
                pass
            time.sleep(0.5)  # 2Hz

    def dispatch(self, action: str, args: dict) -> dict:
        if action in ("start", "stop", "info"):
            return {"state": "running" if self._running else "idle",
                    "topic_out": [{"topic": self._topic, "format": "data/json"}]}
        return {"state": "running"}


# ═══════════════════════════════════════════════════════════════════════════════
# HeadPlugin (actuator)
# ═════════════════════════════════════════════════════════════════════════════════

class HeadPlugin:
    """头部3DOF位置控制 (roll/pitch/yaw)"""

    def __init__(self, plugin_config: dict, namespace: str, ros2):
        self._ns = namespace
        self._ros2 = ros2
        self._pub_node = Node("tianyi2_head_pub", context=ros2.ctx_tianyi)
        ros2.executor_tianyi.add_node(self._pub_node)
        self._publisher = None  # Lazy init

    def get_tool(self) -> dict:
        return {
            "name": "head",
            "type": "actuator",
            "description": "天轶2.0 头部控制 — 3DOF (yaw±90°, pitch±25°, roll±26°)",
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {"type": "string", "enum": ["move_pos", "look_at"],
                               "description": "控制动作"},
                    "yaw": {"type": "number", "description": "偏航角(度), 左正右负, 范围[-90, 90]"},
                    "pitch": {"type": "number", "description": "俯仰角(度), 下正上负, 范围[-25, 25]"},
                    "roll": {"type": "number", "description": "翻滚角(度), 范围[-26, 26]"},
                    "target": {"type": "string", "enum": ["forward", "left", "right", "up", "down"],
                               "description": "预设方向"},
                },
                "required": ["action"],
                "x-action-params": {
                    "move_pos": {"params": ["yaw", "pitch", "roll"],
                                 "description": "移动头部到指定角度(度)"},
                    "look_at": {"params": ["target"],
                                "description": "看向预设方向"},
                },
            },
        }

    def start(self):
        try:
            from bodyctrl_msgs.msg import CmdSetMotorPosition
            self._publisher = self._pub_node.create_publisher(
                CmdSetMotorPosition, "/head/cmd_pos", _RELIABLE_QOS)
            print("[HeadPlugin] publisher created")
        except ImportError as e:
            print(f"[HeadPlugin] WARNING: msg import failed ({e})")

    def stop(self):
        pass

    def dispatch(self, action: str, args: dict) -> dict:
        if action == "move_pos":
            yaw = args.get("yaw", 0)
            pitch = args.get("pitch", 0)
            roll = args.get("roll", 0)
            return self._send_head_pos(roll, pitch, yaw)
        elif action == "look_at":
            target = args.get("target", "forward")
            presets = {
                "forward": (0, 0, 0),
                "left": (45, 0, 0),
                "right": (-45, 0, 0),
                "up": (0, -20, 0),
                "down": (0, 20, 0),
            }
            yaw, pitch, roll = presets.get(target, (0, 0, 0))
            return self._send_head_pos(roll, pitch, yaw)
        elif action in ("start", "info"):
            return {"state": "ready"}
        elif action == "stop":
            return {"state": "idle"}
        return {"error": f"unknown action: {action}"}

    def _send_head_pos(self, roll_deg: float, pitch_deg: float, yaw_deg: float) -> dict:
        if not self._publisher:
            return {"error": "publisher not initialized"}
        try:
            from bodyctrl_msgs.msg import CmdSetMotorPosition, SetMotorPosition
            msg = CmdSetMotorPosition()
            cmds = []
            for motor_id, deg in [(1, roll_deg), (2, pitch_deg), (3, yaw_deg)]:
                cmd = SetMotorPosition()
                cmd.name = motor_id
                cmd.pos = _deg2rad(deg)
                cmd.spd = 1.0  # rad/s
                cmd.cur = _RATED_MOTOR_CURRENT_A[motor_id]
                cmds.append(cmd)
            msg.cmds = cmds
            self._publisher.publish(msg)
            return {"state": "moving", "yaw": yaw_deg, "pitch": pitch_deg, "roll": roll_deg}
        except Exception as e:
            return {"error": str(e)}


# ══════════════════════════════════════════════════════════════════════════════
# HeadGesturePlugin (actuator)
# ══════════════════════════════════════════════════════════════════════════════

class HeadGesturePlugin:
    """可取消的头部语义动作序列。"""

    _STATUS_MAX_AGE = 2.0
    _FEEDBACK_TIMEOUT = 2.0
    _MOVE_THRESHOLD_RAD = _deg2rad(0.5)
    _TARGET_TOLERANCE_RAD = _deg2rad(3.0)

    def __init__(self, plugin_config: dict, namespace: str, ros2):
        self._pub_node = Node("tianyi2_head_gesture_pub", context=ros2.ctx_tianyi)
        ros2.executor_tianyi.add_node(self._pub_node)
        self._publisher = None
        self._sequence = _ActionSequence("HeadGesturePlugin")
        self._feedback_condition = threading.Condition()
        self._head_status = {}
        self._head_status_seq = 0
        self._head_status_time = None
        self._power_status = {}
        self._power_status_time = None

    def get_tool(self) -> dict:
        return {
            "name": "head_gesture",
            "type": "actuator",
            "description": "天轶2.0 头部语义动作 — 点头、摇头、左右观察、歪头和回正",
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {
                        "type": "string",
                        "enum": ["nod", "shake", "scan", "tilt", "reset", "cancel"],
                        "default": "nod",
                        "description": "头部动作，可选[nod, shake, scan, tilt, reset, cancel]",
                    },
                    "cycles": {
                        "type": "integer", "minimum": 1, "maximum": 5,
                        "default": 2, "description": "循环次数，范围[1, 5]，默认2",
                    },
                    "nod_amplitude": {
                        "type": "number", "minimum": 5, "maximum": 20,
                        "default": 12,
                        "description": "点头向下幅度(度)，范围[5, 20]，默认12",
                    },
                    "shake_amplitude": {
                        "type": "number", "minimum": 5, "maximum": 45,
                        "default": 25,
                        "description": "摇头左右幅度(度)，范围[5, 45]，默认25",
                    },
                    "scan_amplitude": {
                        "type": "number", "minimum": 5, "maximum": 45,
                        "default": 25,
                        "description": "左右观察幅度(度)，范围[5, 45]，默认25",
                    },
                    "scan_hold": {
                        "type": "number", "minimum": 0.2, "maximum": 3.0,
                        "default": 1.0,
                        "description": "左右观察时每侧停留时间(秒)，范围[0.2, 3.0]，默认1.0",
                    },
                    "tilt_amplitude": {
                        "type": "number", "minimum": 5, "maximum": 20,
                        "default": 12,
                        "description": "歪头幅度(度)，范围[5, 20]，默认12",
                    },
                    "speed": {
                        "type": "number", "minimum": 5, "maximum": 60,
                        "default": 30,
                        "description": "动作速度(度/秒)，范围[5, 60]，默认30",
                    },
                    "side": {
                        "type": "string", "enum": ["left", "right"],
                        "default": "left",
                        "description": "歪头方向，可选[left, right]，默认left",
                    },
                    "hold": {
                        "type": "number", "minimum": 0.2, "maximum": 3.0,
                        "default": 0.8,
                        "description": "歪头保持时间(秒)，范围[0.2, 3.0]，默认0.8",
                    },
                },
                "required": ["action"],
                "x-completion": {
                    "actions": ["scan", "shake"],
                    "timeout": 30,
                },
                "x-action-params": {
                    "tilt": {"params": ["side", "tilt_amplitude", "speed", "hold"], "description": "向指定方向歪头、保持后回正"},
                    "reset": {"params": ["speed"], "description": "取消序列并将头部回正"},
                    "cancel": {"params": [], "description": "取消尚未发送的后续动作帧"},
                },
            },
        }

    def start(self):
        try:
            from bodyctrl_msgs.msg import (
                CmdSetMotorPosition, MotorStatusMsg, PowerBoardKeyStatus)
            self._publisher = self._pub_node.create_publisher(
                CmdSetMotorPosition, "/head/cmd_pos", _RELIABLE_QOS)
            self._pub_node.create_subscription(
                MotorStatusMsg, "/head/status",
                self._on_head_status, _RELIABLE_QOS)
            self._pub_node.create_subscription(
                PowerBoardKeyStatus, "/power/board/key_status",
                self._on_power_status, _RELIABLE_QOS)
            print("[HeadGesturePlugin] publisher and feedback subscriptions created")
        except ImportError as e:
            print(f"[HeadGesturePlugin] WARNING: msg import failed ({e})")

    def stop(self):
        self._sequence.cancel()

    def dispatch(self, action: str, args: dict) -> dict:
        if action in ("start", "info"):
            return {
                "state": "ready" if self._publisher else "idle",
                "feedback_supported": True,
                "feedback_topic": "/head/status",
            }
        if action == "cancel":
            return {"state": "cancelled", "cancelled": self._sequence.cancel()}
        if action == "reset":
            self._sequence.cancel()
            check = self._preflight()
            if check is not None:
                return check
            baseline_seq, baseline = self._feedback_snapshot()
            result = self._publish_pose(0, 0, 0, args.get("speed", 30))
            if "error" in result:
                return result
            return self._wait_for_head_feedback(
                (0, 0, 0), baseline_seq, baseline)
        if action not in ("nod", "shake", "scan", "tilt"):
            return {"error": f"unknown action: {action}"}
        if not self._publisher:
            return {"error": "publisher not initialized"}
        check = self._preflight()
        if check is not None:
            return check

        cycles = int(_clamp(args.get("cycles", 2), 1, 5))
        speed = _clamp(args.get("speed", 30), 5, 60)
        amplitude_specs = {
            "nod": ("nod_amplitude", 12, 20),
            "shake": ("shake_amplitude", 25, 45),
            "scan": ("scan_amplitude", 25, 45),
            "tilt": ("tilt_amplitude", 12, 20),
        }
        amplitude_key, amplitude_default, amplitude_max = amplitude_specs[action]
        amplitude = _clamp(
            args.get(amplitude_key, amplitude_default), 5, amplitude_max)

        frames: list[tuple[float, float, float, float]] = []
        if action == "nod":
            for _ in range(cycles):
                frames.extend([(0, amplitude, 0, amplitude / speed),
                               (0, 0, 0, amplitude / speed)])
        elif action == "shake":
            for _ in range(cycles):
                frames.extend([(amplitude, 0, 0, amplitude / speed),
                               (-amplitude, 0, 0, 2 * amplitude / speed)])
        elif action == "scan":
            scan_hold = _clamp(args.get("scan_hold", 1.0), 0.2, 3.0)
            for _ in range(cycles):
                frames.extend([(amplitude, 0, 0, amplitude / speed + scan_hold),
                               (0, 0, 0, amplitude / speed),
                               (-amplitude, 0, 0, amplitude / speed + scan_hold),
                               (0, 0, 0, amplitude / speed)])
        else:
            roll = amplitude if args.get("side", "left") == "left" else -amplitude
            hold = _clamp(args.get("hold", 0.8), 0.2, 3.0)
            frames.append((0, 0, roll, amplitude / speed + hold))
        frames.append((0, 0, 0, max(0.15, amplitude / speed)))

        def _worker(cancel_event: threading.Event):
            for yaw, pitch, roll, delay in frames:
                if cancel_event.is_set():
                    return
                result = self._publish_pose(yaw, pitch, roll, speed)
                if "error" in result or cancel_event.wait(max(0.15, delay)):
                    return

        baseline_seq, baseline = self._feedback_snapshot()
        # ACP: for long actions (scan, shake), return action_id and callback on done
        action_id = None
        on_done = None
        if action in ("scan", "shake"):
            from uuid import uuid4
            action_id = f"head_gesture_{action}_{uuid4().hex[:8]}"

            def on_done(cancelled):
                if cancelled:
                    _acp_notify(action_id, "cancelled", {"gesture": action}, "head_gesture")
                else:
                    _acp_notify(action_id, "completed", {"gesture": action, "cycles": cycles}, "head_gesture")

        self._sequence.start(_worker, on_done=on_done)
        first_target = frames[0][:3]
        feedback = self._wait_for_head_feedback(
            first_target, baseline_seq, baseline)
        if feedback.get("state") == "error":
            self._sequence.cancel()
            return feedback
        result = {
            "state": "running", "gesture": action, "cycles": cycles,
            "amplitude": amplitude, "speed": speed,
            "feedback_verified": True,
            "feedback": feedback,
        }
        if action_id:
            result["action_id"] = action_id
        return result

    def _on_head_status(self, msg):
        now = time.monotonic()
        with self._feedback_condition:
            self._head_status = {
                int(motor.name): {
                    "pos": float(motor.pos),
                    "speed": float(motor.speed),
                    "current": float(motor.current),
                    "temperature": float(motor.temperature),
                    "error": int(motor.error),
                }
                for motor in msg.status
            }
            self._head_status_seq += 1
            self._head_status_time = now
            self._feedback_condition.notify_all()

    def _on_power_status(self, msg):
        now = time.monotonic()
        with self._feedback_condition:
            self._power_status = {
                "is_estop": bool(msg.is_estop.data),
                "is_remote_estop": bool(msg.is_remote_estop.data),
                "is_power_on": bool(msg.is_power_on.data),
            }
            self._power_status_time = now
            self._feedback_condition.notify_all()

    def _error_result(self, code: str, message: str, **details) -> dict:
        result = {
            "state": "error",
            "error": message,
            "code": code,
        }
        result.update(details)
        return result

    def _active_motor_faults(self) -> list[dict]:
        faults = []
        for motor_id in _HEAD_JOINTS:
            status = self._head_status.get(motor_id)
            if status is None or status["error"] == 0:
                continue
            error_code = status["error"]
            faults.append({
                "motor_id": motor_id,
                "joint": _HEAD_JOINTS[motor_id],
                "error_code": error_code,
                "description": _MOTOR_ERROR_DESCRIPTIONS.get(
                    error_code, "unknown_vendor_error"),
            })
        return faults

    def _preflight(self) -> dict | None:
        if not self._publisher:
            return self._error_result(
                "publisher_not_initialized",
                "head command publisher is not initialized")
        now = time.monotonic()
        with self._feedback_condition:
            if self._head_status_time is None:
                return self._error_result(
                    "head_status_unavailable",
                    "No /head/status received; head controller may not be running",
                    diagnosis=[
                        "check robot body-control program",
                        "complete robot self-check and confirm Ready state",
                        "check ROS_DOMAIN_ID and /head/status",
                    ],
                )
            status_age = now - self._head_status_time
            if status_age > self._STATUS_MAX_AGE:
                return self._error_result(
                    "head_status_stale",
                    f"/head/status is stale ({status_age:.2f}s)",
                    diagnosis=[
                        "check robot body-control program",
                        "check ROS communication",
                    ],
                )
            missing = [
                motor_id for motor_id in _HEAD_JOINTS
                if motor_id not in self._head_status
            ]
            if missing:
                return self._error_result(
                    "head_motors_missing",
                    "Head motors are missing from /head/status",
                    missing_motor_ids=missing,
                )
            faults = self._active_motor_faults()
            if faults:
                return self._error_result(
                    "head_motor_fault", "Head has active motor faults",
                    faults=faults,
                )
            if (self._power_status_time is not None
                    and now - self._power_status_time <= self._STATUS_MAX_AGE):
                if (self._power_status.get("is_estop")
                        or self._power_status.get("is_remote_estop")):
                    return self._error_result(
                        "emergency_stop_active",
                        "Physical or remote emergency stop is active",
                        power_status=dict(self._power_status),
                    )
                if not self._power_status.get("is_power_on", True):
                    return self._error_result(
                        "robot_power_off", "Robot power board reports power off",
                        power_status=dict(self._power_status),
                    )
        return None

    def _feedback_snapshot(self) -> tuple[int, dict[int, float]]:
        with self._feedback_condition:
            return self._head_status_seq, {
                motor_id: self._head_status[motor_id]["pos"]
                for motor_id in _HEAD_JOINTS
                if motor_id in self._head_status
            }

    def _wait_for_head_feedback(
            self, target: tuple[float, float, float],
            baseline_seq: int, baseline: dict[int, float]) -> dict:
        yaw, pitch, roll = target
        targets = {
            1: _deg2rad(float(roll)),
            2: _deg2rad(float(pitch)),
            3: _deg2rad(float(yaw)),
        }
        deadline = time.monotonic() + self._FEEDBACK_TIMEOUT
        received_new_status = False
        with self._feedback_condition:
            while True:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    break
                if self._head_status_seq <= baseline_seq:
                    self._feedback_condition.wait(remaining)
                    continue
                received_new_status = True
                faults = self._active_motor_faults()
                if faults:
                    return self._error_result(
                        "head_motor_fault_after_command",
                        "Head motor fault appeared after command",
                        faults=faults,
                    )
                positions = {
                    motor_id: self._head_status[motor_id]["pos"]
                    for motor_id in _HEAD_JOINTS
                }
                moved = max(
                    abs(positions[motor_id] - baseline[motor_id])
                    for motor_id in _HEAD_JOINTS
                )
                target_error = max(
                    abs(positions[motor_id] - targets[motor_id])
                    for motor_id in _HEAD_JOINTS
                )
                if (moved >= self._MOVE_THRESHOLD_RAD
                        or target_error <= self._TARGET_TOLERANCE_RAD):
                    return {
                        "state": "moving",
                        "status_topic": "/head/status",
                        "max_movement_deg": round(_rad2deg(moved), 2),
                        "max_target_error_deg": round(
                            _rad2deg(target_error), 2),
                    }
                self._feedback_condition.wait(0.05)
        if not received_new_status:
            return self._error_result(
                "head_feedback_timeout",
                "Command was published but no new /head/status was received",
                diagnosis=[
                    "check head controller and ROS communication",
                    "confirm robot self-check completed and robot is Ready",
                ],
            )
        return self._error_result(
            "head_no_motion",
            "Command was published and head status updated, but no joint moved",
            diagnosis=[
                "robot may not be Ready or self-check may be incomplete",
                "head controller may be disabled or rejecting commands",
                "another node may be publishing competing /head/cmd_pos commands",
            ],
        )

    def _publish_pose(self, yaw_deg: float, pitch_deg: float,
                      roll_deg: float, speed_deg: float) -> dict:
        if not self._publisher:
            return {"error": "publisher not initialized"}
        try:
            from bodyctrl_msgs.msg import CmdSetMotorPosition, SetMotorPosition
            yaw_deg = _clamp(yaw_deg, -90, 90)
            pitch_deg = _clamp(pitch_deg, -25, 25)
            roll_deg = _clamp(roll_deg, -26, 26)
            speed_rad = _deg2rad(_clamp(speed_deg, 5, 60))
            msg = CmdSetMotorPosition()
            msg.cmds = []
            for motor_id, deg in [(1, roll_deg), (2, pitch_deg), (3, yaw_deg)]:
                cmd = SetMotorPosition()
                cmd.name = motor_id
                cmd.pos = _deg2rad(deg)
                cmd.spd = speed_rad
                cmd.cur = _RATED_MOTOR_CURRENT_A[motor_id]
                msg.cmds.append(cmd)
            self._publisher.publish(msg)
            return {"state": "moving", "yaw": yaw_deg, "pitch": pitch_deg, "roll": roll_deg}
        except Exception as e:
            return {"error": str(e)}


# ═══════════════════════════════════════════════════════════════════════════
# ArmPlugin (actuator)
# ════════════════════════════════════════════════════════════════════════════════

class ArmPlugin:
    """双臂14DOF控制 (位置模式 / 力位混合)"""

    # Raw-control defaults and a deliberately bounded tuning range. The
    # vendor-side controller applies them; this driver does not rate-limit the
    # position step.
    _DEFAULT_KP = 50.0
    _DEFAULT_KD = 20.0
    _KP_RANGE = (10.0, 200.0)
    _KD_RANGE = (5.0, 50.0)

    _JOINT_NAMES = [
        "shoulder_pitch", "shoulder_roll", "shoulder_yaw",
        "elbow_pitch", "wrist_yaw", "wrist_pitch", "wrist_roll",
    ]
    _LEFT_POSE_LIMITS = [
        (-170, 170), (-15, 150), (-170, 170), (-150, 15),
        (-170, 170), (-45, 60), (-95, 75),
    ]
    _RIGHT_POSE_LIMITS = [
        (-170, 170), (-150, 15), (-170, 170), (-150, 15),
        (-170, 170), (-45, 60), (-75, 95),
    ]

    def __init__(self, plugin_config: dict, namespace: str, ros2):
        self._ns = namespace
        self._ros2 = ros2
        self._pub_node = Node("tianyi2_arm_pub", context=ros2.ctx_tianyi)
        ros2.executor_tianyi.add_node(self._pub_node)
        self._pos_publisher = None
        self._ctrl_publisher = None
        self._feedback = _JointCommandFeedback("arm", "/arm/status")
        self._sequence = _ActionSequence("ArmPlugin")

    def get_tool(self) -> dict:
        return {
            "name": "arm",
            "type": "actuator",
            "description": (
                "控制左右手臂的各关节角度。不确定选哪个模式时，请使用move_pos："
                "它适合抬手、弯肘、摆姿势和回到初始位置。move_ctrl是高级调试模式，"
                "用于调整手臂保持姿势时有多用力、到位后是否容易晃动，以及已确认安全的"
                "轻推柔顺实验；它不是慢速模式，不适合普通动作或大幅移动。"
            ),
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {"type": "string", "enum": ["move_pos", "move_ctrl", "move_traj"],
                               "default": "move_pos",
                               "description": (
                                   "模式选择：抬手、弯肘、摆姿势、回零等普通操作选move_pos；"
                                   "只有需要调节手臂保持力度或减少晃动时才选move_ctrl；"
                                   "连贯多段动作（如太极、舞蹈）选move_traj"
                               )},
                    "waypoints": {
                        "type": "array",
                        "items": {
                            "type": "object",
                            "properties": {
                                "left_positions": {
                                    "type": "array", "items": {"type": "number", "minimum": -170, "maximum": 170},
                                    "minItems": 7, "maxItems": 7,
                                    "description": "左臂7关节角度(度)，顺序[肩pitch,肩roll,肩yaw,肘pitch,腕yaw,腕pitch,腕roll]，缺失时继承上一个路径点"
                                },
                                "right_positions": {
                                    "type": "array", "items": {"type": "number", "minimum": -170, "maximum": 170},
                                    "minItems": 7, "maxItems": 7,
                                    "description": "右臂7关节角度(度)，顺序同左臂，缺失时继承上一个路径点"
                                },
                                "time_from_start": {
                                    "type": "number", "minimum": 0,
                                    "description": "从轨迹开始到此点的秒数，必须递增"
                                },
                            },
                            "required": ["time_from_start"],
                        },
                        "minItems": 2,
                        "description": "轨迹路径点列表（≥2个），按time_from_start升序排列。每点可只指定left或right，缺失的一侧继承上一个路径点"
                    },
                    "left_positions": {
                        "type": "array", "items": {"type": "number", "minimum": -170, "maximum": 170},
                        "minItems": 7, "maxItems": 7,
                        "default": [0, 0, 0, 0, 0, 0, 0],
                        "description": "左臂实际7关节角度(度): [肩pitch, 肩roll, 肩yaw, 肘pitch, 腕yaw, 腕pitch, 腕roll]"
                    },
                    "right_positions": {
                        "type": "array", "items": {"type": "number", "minimum": -170, "maximum": 170},
                        "minItems": 7, "maxItems": 7,
                        "default": [0, 0, 0, 0, 0, 0, 0],
                        "description": "右臂实际7关节角度(度)，顺序同左臂；若要镜像左臂姿态，请将肩roll、肩yaw、腕yaw、腕roll（索引1/2/4/6）取反"
                    },
                    "speed": {"type": "number", "minimum": 0.2, "maximum": 1.5,
                              "default": 0.5,
                              "description": (
                                  "关节角速度，可设置范围[0.2,1.5]rad/s，推荐默认值0.5。"
                                  "move_pos：决定手臂移动到目标姿势有多快；常规动作用0.5，"
                                  "想要更慢更稳可降到0.3左右，想要更快更利落可提到0.8左右。"
                                  "move_traj：整条轨迹执行时关节移动的快慢，实际节奏由waypoints的"
                                  "time_from_start控制。move_ctrl不使用此参数（不能靠它减速）"
                              )},
                    "kp": {"type": "array", "items": {"type": "number", "minimum": 10, "maximum": 200},
                           "minItems": 7, "maxItems": 7,
                           "default": [50, 50, 50, 50, 50, 50, 50],
                           "description": (
                               "仅move_ctrl使用，可理解为关节偏离目标后“拉回去的力度”。"
                               "按[肩pitch,肩roll,肩yaw,肘pitch,腕yaw,腕pitch,腕roll]填写7项，"
                               "范围[10,200]，默认50，左右臂共用。调高会保持得更硬、更有力，"
                               "但可能动作突然或冲过目标；调低会更柔和，但手臂可能被负载压偏"
                           )},
                    "kd": {"type": "array", "items": {"type": "number", "minimum": 5, "maximum": 50},
                           "minItems": 7, "maxItems": 7,
                           "default": [20, 20, 20, 20, 20, 20, 20],
                           "description": (
                               "仅move_ctrl使用，可理解为关节的“减震力度”。关节顺序同KP，"
                               "共7项，范围[5,50]，默认20，左右臂共用。调高通常更不容易晃，"
                               "但反应可能变慢；调得太低，手臂到位后可能来回抖动"
                           )},
                },
                "required": ["action"],
                "x-action-params": {
                    "move_pos": {"params": ["left_positions", "right_positions", "speed"],
                                 "description": (
                                     "普通动作首选：填写左右臂目标角度和移动速度。适合抬手、弯肘、"
                                     "摆出指定姿势、回到初始位置，以及其他希望控制移动快慢的场景"
                                 )},
                    "move_ctrl": {"params": ["left_positions", "right_positions", "kp", "kd"],
                                  "description": (
                                      "高级调试模式：目标角度决定手臂想停在哪里，KP决定偏离后拉回的"
                                      "力度，KD决定减少晃动的力度。适用于手臂被负载压偏、到位后晃动，"
                                      "或已确认安全的轻推柔顺实验。它没有可设置的移动速度，同一角度在"
                                      "不同KP/KD下也可能停在不同位置；普通摆姿势或大幅移动请用move_pos"
                                  )},
                    "move_traj": {"params": ["waypoints", "speed"],
                                  "description": (
                                      "轨迹模式：一次下发完整路径点序列，驱动侧以50Hz线性插值连续下发"
                                      "CmdSetMotorPosition，消除多段move_pos之间的LLM调度停顿。适合太极、"
                                      "舞蹈、连贯手势等多段连续动作。参数格式："
                                      "waypoints为数组(至少2个点，按time_from_start升序)，每个点包含："
                                      "time_from_start(秒，从轨迹开始到此点的时间，必填且必须递增)、"
                                      "left_positions/right_positions(7关节角度，单位度，两者可省略其一，"
                                      "缺失的一侧自动继承上一个路径点；第一个点若缺失某侧则该侧默认全0)。"
                                      "关节顺序为[肩pitch,肩roll,肩yaw,肘pitch,腕yaw,腕pitch,腕roll]，"
                                      "负pitch=向前。speed为关节角速度[0.2,1.5]rad/s，推荐0.5。"
                                      "轨迹总时长>3秒时异步执行并返回action_id，≤3秒同步返回completed"
                                  )},
                },
            },
        }

    def start(self):
        try:
            from bodyctrl_msgs.msg import CmdSetMotorPosition, CmdMotorCtrl
            self._pos_publisher = self._pub_node.create_publisher(
                CmdSetMotorPosition, "/arm/cmd_pos", _RELIABLE_QOS)
            self._ctrl_publisher = self._pub_node.create_publisher(
                CmdMotorCtrl, "/arm/cmd_ctrl", _RELIABLE_QOS)
            self._feedback.create_subscriptions(self._pub_node)
            print("[ArmPlugin] publishers and feedback subscriptions created")
        except ImportError as e:
            print(f"[ArmPlugin] WARNING: msg import failed ({e})")

    def stop(self):
        pass

    def dispatch(self, action: str, args: dict) -> dict:
        if action == "move_pos":
            poses = self._requested_poses(args)
            speed = args.get("speed", 0.5)
            validated = self._validate_command(poses, speed=speed)
            if isinstance(validated, dict):
                return validated
            poses, speed = validated
            motor_ids = self._motor_ids()
            check = self._feedback.preflight(self._pos_publisher, motor_ids)
            if check is not None:
                return check
            baseline_seq, baseline = self._feedback.snapshot(motor_ids)
            result = self._send_pos(poses, speed)
            if "error" in result:
                return result
            feedback = self._feedback.wait_for_motion(
                self._target_positions(poses),
                baseline_seq, baseline)
            if feedback.get("state") == "error":
                return feedback
            result["feedback_verified"] = True
            result["feedback"] = feedback
            return result
        elif action == "move_traj":
            return self._handle_move_traj(args)
        elif action == "move_ctrl":
            poses = self._requested_poses(args)
            kp = args.get("kp", [self._DEFAULT_KP] * 7)
            kd = args.get("kd", [self._DEFAULT_KD] * 7)
            validated = self._validate_command(
                poses, kp=kp, kd=kd)
            if isinstance(validated, dict):
                return validated
            poses, kp, kd = validated
            motor_ids = self._motor_ids()
            check = self._feedback.preflight(self._ctrl_publisher, motor_ids)
            if check is not None:
                return check
            baseline_seq, baseline = self._feedback.snapshot(motor_ids)
            result = self._send_ctrl(poses, kp, kd)
            if "error" in result:
                return result
            feedback = self._feedback.wait_for_motion(
                self._target_positions(poses),
                baseline_seq, baseline)
            if feedback.get("state") == "error":
                return feedback
            result["feedback_verified"] = True
            result["feedback"] = feedback
            return result
        elif action in ("start", "info"):
            return {
                "state": "ready" if self._pos_publisher else "idle",
                "feedback_supported": True,
                "feedback_topic": "/arm/status",
                "right_arm_mirrored": False,
                "independent_bilateral_positions": True,
            }
        elif action == "stop":
            return {"state": "idle"}
        return {"error": f"unknown action: {action}"}

    @staticmethod
    def _requested_poses(args: dict) -> dict[str, object]:
        """Resolve visible per-arm fields, with legacy `positions` fallback."""
        legacy = args.get("positions", [0] * 7)
        return {
            "left": args.get("left_positions", legacy),
            "right": args.get("right_positions", legacy),
        }

    @staticmethod
    def _motor_ids() -> list[int]:
        return [*range(11, 18), *range(21, 28)]

    @classmethod
    def _pose_violations(cls, poses: dict[str, list[float]]) -> list[dict]:
        selected = [
            (arm_side, pose,
             cls._LEFT_POSE_LIMITS if arm_side == "left" else cls._RIGHT_POSE_LIMITS)
            for arm_side, pose in poses.items()
        ]
        violations = []
        for arm_side, pose, limits in selected:
            for index, (value, bounds) in enumerate(zip(pose, limits)):
                lower, upper = bounds
                if value < lower or value > upper:
                    violations.append({
                        "side": arm_side,
                        "joint": cls._JOINT_NAMES[index],
                        "value_deg": value,
                        "minimum_deg": lower,
                        "maximum_deg": upper,
                    })
        return violations

    @classmethod
    def _target_positions(
            cls, poses: dict[str, list[float]]) -> dict[int, float]:
        targets = {}
        if "left" in poses:
            targets.update({
                11 + index: _deg2rad(deg)
                for index, deg in enumerate(poses["left"])
            })
        if "right" in poses:
            targets.update({
                21 + index: _deg2rad(deg)
                for index, deg in enumerate(poses["right"])
            })
        return targets

    @staticmethod
    def _decode_array_argument(value, name: str):
        """Accept native arrays and JSON-array strings emitted by the dashboard."""
        if not isinstance(value, str):
            return value, None
        try:
            decoded = json.loads(value)
        except json.JSONDecodeError as exc:
            return None, {
                "state": "error",
                "error": f"{name} must be a valid JSON array",
                "code": f"invalid_arm_{name}",
                "parse_error": str(exc),
            }
        if not isinstance(decoded, list):
            return None, {
                "state": "error",
                "error": f"{name} JSON value must be an array",
                "code": f"invalid_arm_{name}",
            }
        return decoded, None

    @classmethod
    def _validate_command(
            cls, poses, speed=None, kp=None, kd=None):
        converted_poses = {}
        for arm_side, values in poses.items():
            name = f"{arm_side}_positions"
            values, error = cls._decode_array_argument(values, name)
            if error is not None:
                return error
            if not isinstance(values, (list, tuple)) or len(values) != 7:
                return {"state": "error",
                        "error": f"{name} must have exactly 7 values (degrees)",
                        "code": f"invalid_arm_{name}"}
            try:
                pose = [float(value) for value in values]
            except (TypeError, ValueError):
                return {"state": "error", "error": f"{name} must be numeric",
                        "code": f"invalid_arm_{name}"}
            if not all(math.isfinite(value) for value in pose):
                return {"state": "error", "error": f"{name} must be finite",
                        "code": f"invalid_arm_{name}"}
            converted_poses[arm_side] = pose
        violations = cls._pose_violations(converted_poses)
        if violations:
            return {"state": "error", "error": "Arm pose exceeds URDF joint limits",
                    "code": "arm_pose_out_of_range", "violations": violations}
        if speed is not None:
            try:
                speed = float(speed)
            except (TypeError, ValueError):
                return {"state": "error", "error": "speed must be numeric",
                        "code": "invalid_arm_speed"}
            if not math.isfinite(speed):
                return {"state": "error", "error": "speed must be finite",
                        "code": "invalid_arm_speed"}
            if speed < 0.2 or speed > 1.5:
                return {"state": "error", "error": "speed must be in [0.2, 1.5] rad/s",
                        "code": "arm_speed_out_of_range", "speed": speed}
            return converted_poses, speed
        for name, values, lower, upper in (
                ("kp", kp, *cls._KP_RANGE),
                ("kd", kd, *cls._KD_RANGE)):
            values, error = cls._decode_array_argument(values, name)
            if error is not None:
                return error
            if not isinstance(values, (list, tuple)) or len(values) != 7:
                return {"state": "error", "error": f"{name} must have exactly 7 values",
                        "code": f"invalid_arm_{name}"}
            try:
                converted = [float(value) for value in values]
            except (TypeError, ValueError):
                return {"state": "error", "error": f"{name} must be numeric",
                        "code": f"invalid_arm_{name}"}
            if not all(math.isfinite(value) for value in converted):
                return {"state": "error", "error": f"{name} must be finite",
                        "code": f"invalid_arm_{name}"}
            bad = [value for value in converted if value < lower or value > upper]
            if bad:
                return {"state": "error", "error": f"{name} values must be in [{lower}, {upper}]",
                        "code": f"arm_{name}_out_of_range", "values": bad}
            if name == "kp":
                kp = converted
            else:
                kd = converted
        return converted_poses, kp, kd

    def _send_pos(self, poses: dict[str, list[float]], speed: float) -> dict:
        if not self._pos_publisher:
            return {"error": "publisher not initialized"}
        try:
            from bodyctrl_msgs.msg import CmdSetMotorPosition, SetMotorPosition
            msg = CmdSetMotorPosition()
            cmds = []
            sides = [(11 if arm_side == "left" else 21, pose)
                     for arm_side, pose in poses.items()]

            for base_id, pose in sides:
                for i, deg in enumerate(pose):
                    cmd = SetMotorPosition()
                    motor_id = base_id + i
                    cmd.name = motor_id
                    cmd.pos = _deg2rad(deg)
                    cmd.spd = speed
                    cmd.cur = _RATED_MOTOR_CURRENT_A[motor_id]
                    cmds.append(cmd)

            msg.cmds = cmds
            self._pos_publisher.publish(msg)
            return {"state": "moving", "side": "both", "joints": len(cmds)}
        except Exception as e:
            return {"error": str(e)}

    def _handle_move_traj(self, args: dict) -> dict:
        waypoints_raw = args.get("waypoints")
        if waypoints_raw is None:
            return {"state": "error", "error": "waypoints is required for move_traj and must have at least 2 points",
                    "code": "invalid_arm_waypoints"}
        if isinstance(waypoints_raw, str):
            try:
                waypoints_raw = json.loads(waypoints_raw)
            except json.JSONDecodeError as exc:
                return {"state": "error", "error": f"waypoints must be valid JSON: {exc}",
                        "code": "invalid_arm_waypoints"}
        if not isinstance(waypoints_raw, list) or len(waypoints_raw) < 2:
            return {"state": "error", "error": "waypoints is required for move_traj and must have at least 2 points",
                    "code": "invalid_arm_waypoints"}
        speed = args.get("speed", 0.5)
        try:
            speed = float(speed)
        except (TypeError, ValueError):
            return {"state": "error", "error": "speed must be numeric"}
        if speed < 0.2 or speed > 1.5:
            return {"state": "error", "error": "speed must be in [0.2, 1.5] rad/s"}
        prev_left = None
        prev_right = None
        resolved = []
        for i, wp in enumerate(waypoints_raw):
            if not isinstance(wp, dict):
                return {"state": "error", "error": f"waypoint {i} must be an object"}
            tfs = wp.get("time_from_start")
            if tfs is None:
                return {"state": "error", "error": f"waypoint {i} missing time_from_start"}
            try:
                tfs = float(tfs)
            except (TypeError, ValueError):
                return {"state": "error", "error": f"waypoint {i} time_from_start must be numeric"}
            left = wp.get("left_positions")
            right = wp.get("right_positions")
            if left is not None:
                left, err = self._decode_array_argument(left, "left_positions")
                if err:
                    return err
                if not isinstance(left, (list, tuple)) or len(left) != 7:
                    return {"state": "error", "error": f"waypoint {i} left_positions must have 7 values"}
                left = [float(v) for v in left]
                prev_left = left
            else:
                left = prev_left
            if right is not None:
                right, err = self._decode_array_argument(right, "right_positions")
                if err:
                    return err
                if not isinstance(right, (list, tuple)) or len(right) != 7:
                    return {"state": "error", "error": f"waypoint {i} right_positions must have 7 values"}
                right = [float(v) for v in right]
                prev_right = right
            else:
                right = prev_right
            if left is None and right is None:
                return {"state": "error", "error": f"waypoint {i} has no arm target and no prior point to inherit"}
            if left is None:
                left = [0.0] * 7
            if right is None:
                right = [0.0] * 7
            poses = {"left": left, "right": right}
            violations = self._pose_violations(poses)
            if violations:
                return {"state": "error", "error": f"waypoint {i} pose exceeds URDF joint limits",
                        "violations": violations}
            resolved.append((tfs, left, right))
        if len(resolved) < 2:
            return {"state": "error", "error": "need at least 2 valid waypoints"}
        for i in range(1, len(resolved)):
            if resolved[i][0] <= resolved[i - 1][0]:
                return {"state": "error", "error": f"waypoint {i} time_from_start must be greater than previous"}
        motor_ids = self._motor_ids()
        check = self._feedback.preflight(self._pos_publisher, motor_ids)
        if check is not None:
            return check
        action_id = f"arm_traj_{uuid4().hex[:8]}"
        duration = resolved[-1][0]
        if duration <= 3.0:
            cancel_event = threading.Event()
            self._run_trajectory(resolved, speed, cancel_event)
            return {
                "state": "completed",
                "action_id": action_id,
                "waypoints": len(resolved),
                "duration_s": duration,
            }
        else:
            def _worker(cancel_event):
                self._run_trajectory(resolved, speed, cancel_event)
            self._sequence.start(_worker)
            return {
                "state": "moving",
                "action_id": action_id,
                "waypoints": len(resolved),
                "duration_s": duration,
                "async": True,
            }

    def _run_trajectory(self, resolved: list, speed: float, cancel_event: threading.Event):
        rate = 0.02
        total_time = resolved[-1][0]
        t = 0.0
        while t < total_time:
            if cancel_event.is_set():
                break
            idx = 0
            for i in range(len(resolved) - 1):
                if resolved[i][0] <= t <= resolved[i + 1][0]:
                    idx = i
                    break
            else:
                idx = len(resolved) - 2
            t0, left0, right0 = resolved[idx]
            t1, left1, right1 = resolved[idx + 1]
            dt = t1 - t0
            alpha = (t - t0) / dt if dt > 0 else 0.0
            alpha = max(0.0, min(1.0, alpha))
            interp_left = [left0[j] + (left1[j] - left0[j]) * alpha for j in range(7)]
            interp_right = [right0[j] + (right1[j] - right0[j]) * alpha for j in range(7)]
            poses = {"left": interp_left, "right": interp_right}
            self._send_pos(poses, speed)
            time.sleep(rate)
            t += rate

    def _send_ctrl(self, poses: dict[str, list[float]], kp: list, kd: list) -> dict:
        if not self._ctrl_publisher:
            return {"error": "publisher not initialized"}
        try:
            from bodyctrl_msgs.msg import CmdMotorCtrl, MotorCtrl
            msg = CmdMotorCtrl()
            cmds = []
            sides = [(11 if arm_side == "left" else 21, pose)
                     for arm_side, pose in poses.items()]

            for base_id, pose in sides:
                for i, deg in enumerate(pose):
                    cmd = MotorCtrl()
                    cmd.name = base_id + i
                    cmd.pos = _deg2rad(deg)
                    cmd.spd = 0.0
                    cmd.tor = 0.0
                    cmd.kp = kp[i] if i < len(kp) else self._DEFAULT_KP
                    cmd.kd = kd[i] if i < len(kd) else self._DEFAULT_KD
                    cmds.append(cmd)

            msg.cmds = cmds
            self._ctrl_publisher.publish(msg)
            return {"state": "moving", "side": "both",
                    "mode": "force_position"}
        except Exception as e:
            return {"error": str(e)}


# ══════════════════════════════════════════════════════════════════════════════
# ArmGesturePlugin (actuator)
# ══════════════════════════════════════════════════════════════════════════════

class ArmGesturePlugin:
    """可取消、带状态反馈和 URDF 限位检查的手臂语义动作序列。"""

    _STATUS_MAX_AGE = 2.0
    _FEEDBACK_TIMEOUT = 2.0
    _MOVE_THRESHOLD_RAD = _deg2rad(0.5)
    _TARGET_TOLERANCE_RAD = _deg2rad(3.0)
    _NEUTRAL = [0, 0, 0, 0, 0, 0, 0]
    _JOINT_NAMES = [
        "shoulder_pitch", "shoulder_roll", "shoulder_yaw",
        "elbow_pitch", "wrist_yaw", "wrist_pitch", "wrist_roll",
    ]
    # Limits in degrees, copied from resource/tianyi2_model.urdf. Keeping the
    # limits here makes a bad semantic pose fail before a motor command is sent.
    _LEFT_POSE_LIMITS = [
        (-170, 170), (-15, 150), (-170, 170), (-150, 15),
        (-170, 170), (-45, 60), (-95, 75),
    ]
    _RIGHT_POSE_LIMITS = [
        (-170, 170), (-150, 15), (-170, 170), (-150, 15),
        (-170, 170), (-45, 60), (-75, 95),
    ]
    # 角度顺序：肩 pitch、肩 roll、肩 yaw、肘 pitch、腕 yaw、腕 pitch、腕 roll。
    # 肘 pitch 使用负角度屈肘；右臂由 _publish_pose 按横向关节自动镜像。
    _GESTURES = {
        # In the URDF chain shoulder yaw rotates the elbow-pitch plane. The
        # shoulder and elbow angles place the wrist; wrist yaw/roll are used
        # only where the final palm orientation needs calibration.
        "salute": [-10, 90, 60, -110, 50, 0, 0],
        "welcome": [-10, 65, 75, -100, 0, 0, 0],
        "raise": [0, 130, 0, -15, 0, 0, 0],
        "shake_hands": [-55, 15, 5, -35, 0, 0, 0],
        "high_five": [-40, 40, -20, -80, 0, 0, 50],
    }
    _PREPARE_POSES = {
        # Flex the elbow while establishing the lifting plane instead of first
        # rotating a fully extended arm near the head.
        "salute": [-10, 40, 35, -45, 25, 0, 0],
        "welcome": [-10, 45, 45, -60, 0, 0, 0],
        "raise": [0, 75, 0, -30, 0, 0, 0],
        "shake_hands": [-30, 10, 0, -20, 0, 0, 0],
        "high_five": [-25, 25, -10, -45, 0, 0, 10],
    }

    def __init__(self, plugin_config: dict, namespace: str, ros2):
        self._pub_node = Node("tianyi2_arm_gesture_pub", context=ros2.ctx_tianyi)
        ros2.executor_tianyi.add_node(self._pub_node)
        self._publisher = None
        self._sequence = _ActionSequence("ArmGesturePlugin")
        self._feedback_condition = threading.Condition()
        self._arm_status = {}
        self._arm_status_seq = 0
        self._arm_status_time = None
        self._power_status = {}
        self._power_status_time = None

    def get_tool(self) -> dict:
        return {
            "name": "arm_gesture",
            "type": "actuator",
            "description": "天轶2.0 手臂语义动作 — 敬礼、欢迎、举手、握手、击掌和回正",
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {
                        "type": "string",
                        "enum": [
                            "salute", "welcome", "raise", "shake_hands",
                            "high_five", "reset", "cancel",
                        ],
                        "default": "welcome",
                        "description": "手臂动作，可选[salute, welcome, raise, shake_hands, high_five, reset, cancel]",
                    },
                    "side": {
                        "type": "string", "enum": ["left", "right", "both"],
                        "default": "right",
                        "description": "执行手臂，可选[left, right, both]，默认right",
                    },
                    "salute_side": {
                        "type": "string", "enum": ["left", "right"],
                        "default": "right",
                        "description": "敬礼手臂，可选[left, right]，默认right",
                    },
                    "cycles": {
                        "type": "integer", "minimum": 1, "maximum": 5,
                        "default": 2,
                        "description": "欢迎/握手摆动循环次数，范围[1, 5]，默认2",
                    },
                    "speed": {
                        "type": "number", "minimum": 0.2, "maximum": 1.5,
                        "default": 0.5,
                        "description": "关节速度(rad/s)，范围[0.2, 1.5]，默认0.5",
                    },
                },
                "required": ["action"],
                "x-completion": {
                    "actions": ["salute", "welcome", "shake_hands"],
                    "timeout": 30,
                },
                "x-action-params": {
                    "salute": {"params": ["salute_side", "speed"], "description": "抬起小臂、将手靠近额侧、停留后回正"},
                    "welcome": {"params": ["side", "cycles", "speed"], "description": "在身体侧上方抬起手掌并左右摆动后回正"},
                    "raise": {"params": ["side", "speed"], "description": "将手臂高举到头部上方后回正"},
                    "shake_hands": {"params": ["side", "cycles", "speed"], "description": "向前伸手并轻柔上下摆动，做出握手动作"},
                    "high_five": {"params": ["side", "speed"], "description": "将手掌伸到身体前方并保持在肩部附近，做出击掌等待姿势"},
                    "reset": {"params": ["side", "speed"], "description": "取消序列并回到中性姿态"},
                    "cancel": {"params": [], "description": "取消尚未发送的后续动作帧"},
                },
            },
        }

    def start(self):
        try:
            from bodyctrl_msgs.msg import (
                CmdSetMotorPosition, MotorStatusMsg, PowerBoardKeyStatus)
            self._publisher = self._pub_node.create_publisher(
                CmdSetMotorPosition, "/arm/cmd_pos", _RELIABLE_QOS)
            self._pub_node.create_subscription(
                MotorStatusMsg, "/arm/status",
                self._on_arm_status, _RELIABLE_QOS)
            self._pub_node.create_subscription(
                PowerBoardKeyStatus, "/power/board/key_status",
                self._on_power_status, _RELIABLE_QOS)
            print("[ArmGesturePlugin] publisher and feedback subscriptions created")
        except ImportError as e:
            print(f"[ArmGesturePlugin] WARNING: msg import failed ({e})")

    def stop(self):
        self._sequence.cancel()

    def dispatch(self, action: str, args: dict) -> dict:
        if action in ("start", "info"):
            return {
                "state": "ready" if self._publisher else "idle",
                "feedback_supported": True,
                "feedback_topic": "/arm/status",
            }
        if action == "cancel":
            return {"state": "cancelled", "cancelled": self._sequence.cancel()}
        if action == "salute":
            # The salute card exposes a dedicated left/right-only selector.
            # Keep accepting the old `side` argument for direct MCP callers.
            side = args.get("salute_side", args.get("side", "right"))
        else:
            side = args.get("side", "right")
        if side not in ("left", "right", "both"):
            return {"error": "side must be left, right or both"}
        if action == "salute" and side == "both":
            return {
                "state": "error",
                "error": "salute only supports one arm at a time to avoid head/arm interference",
                "code": "unsafe_bilateral_salute",
            }
        speed = _clamp(args.get("speed", 0.5), 0.2, 1.5)
        if action == "reset":
            self._sequence.cancel()
            check = self._preflight(side)
            if check is not None:
                return check
            baseline_seq, baseline = self._feedback_snapshot(side)
            result = self._publish_pose(side, self._NEUTRAL, speed)
            if "error" in result:
                return result
            return self._wait_for_arm_feedback(
                side, self._NEUTRAL, baseline_seq, baseline)
        if action not in self._GESTURES:
            return {"error": f"unknown action: {action}"}
        if not self._publisher:
            return {"error": "publisher not initialized"}
        check = self._preflight(side)
        if check is not None:
            return check

        pose = self._GESTURES[action]
        cycles = int(_clamp(args.get("cycles", 2), 1, 5))
        # Frame entries are (pose, hold_seconds, transition_ratio). A ratio
        # below 1 starts the next frame before the current target fully settles,
        # allowing the controller to blend adjacent salute stages.
        if action == "salute":
            frames = [
                (self._PREPARE_POSES[action], 0.0, 0.90),
                (pose, 1.1, 1.0),
            ]
        else:
            frames = [
                (self._PREPARE_POSES[action], 0.25, 0.90),
                (pose, 0.8, 0.90),
            ]
        if action == "shake_hands":
            for i in range(cycles * 2):
                handshake_pose = list(pose)
                # A small elbow sweep produces the handshake motion while the
                # wrist stays neutral and the arm remains extended forward.
                if i % 2 == 0:
                    handshake_pose[3] = -28
                else:
                    handshake_pose[3] = -42
                frames.append((handshake_pose, 0.30, 0.85))
        elif action == "welcome":
            # Keep shoulder yaw and the wrist fixed. In this URDF pose, changing
            # shoulder yaw moves the hand mostly forward/backward. A small elbow
            # pitch sweep instead produces about 10 cm of lateral hand travel
            # with little forward/backward or vertical displacement.
            for i in range(cycles * 2):
                welcome_pose = list(pose)
                if i % 2 == 0:
                    welcome_pose[3] = -110
                else:
                    welcome_pose[3] = -90
                frames.append((welcome_pose, 0.35, 0.85))
        frames.append((self._NEUTRAL, 1.0, 1.0))
        for frame, _, _ in frames:
            violations = self._pose_violations(side, frame)
            if violations:
                return self._error_result(
                    "arm_pose_out_of_range",
                    "Semantic arm pose exceeds URDF joint limits",
                    gesture=action,
                    violations=violations,
                )

        def _worker(cancel_event: threading.Event):
            previous = self._NEUTRAL
            for frame, hold, transition_ratio in frames:
                if cancel_event.is_set():
                    return
                result = self._publish_pose(side, frame, speed)
                max_delta_rad = max(
                    abs(_deg2rad(float(current) - float(old)))
                    for current, old in zip(frame, previous)
                )
                transition = max_delta_rad / speed if speed > 0 else 0
                previous = frame
                delay = max(0.12, transition * transition_ratio) + hold
                if "error" in result or cancel_event.wait(delay):
                    return

        baseline_seq, baseline = self._feedback_snapshot(side)
        # ACP: for long gestures, return action_id and callback on done
        action_id = None
        on_done = None
        if action in ("salute", "welcome", "shake_hands"):
            from uuid import uuid4
            action_id = f"arm_gesture_{action}_{uuid4().hex[:8]}"

            def on_done(cancelled):
                if cancelled:
                    _acp_notify(action_id, "cancelled", {"gesture": action, "side": side}, "arm_gesture")
                else:
                    _acp_notify(action_id, "completed", {"gesture": action, "side": side}, "arm_gesture")

        self._sequence.start(_worker, on_done=on_done)
        feedback = self._wait_for_arm_feedback(
            side, frames[0][0], baseline_seq, baseline)
        if feedback.get("state") == "error":
            self._sequence.cancel()
            return feedback
        result = {
            "state": "running", "gesture": action, "side": side,
            "cycles": cycles, "speed": speed,
            "feedback_verified": True,
            "feedback": feedback,
        }
        if action_id:
            result["action_id"] = action_id
        return result

    def _on_arm_status(self, msg):
        now = time.monotonic()
        with self._feedback_condition:
            self._arm_status = {
                int(motor.name): {
                    "pos": float(motor.pos),
                    "speed": float(motor.speed),
                    "current": float(motor.current),
                    "temperature": float(motor.temperature),
                    "error": int(motor.error),
                }
                for motor in msg.status
            }
            self._arm_status_seq += 1
            self._arm_status_time = now
            self._feedback_condition.notify_all()

    def _on_power_status(self, msg):
        now = time.monotonic()
        with self._feedback_condition:
            self._power_status = {
                "is_estop": bool(msg.is_estop.data),
                "is_remote_estop": bool(msg.is_remote_estop.data),
                "is_power_on": bool(msg.is_power_on.data),
            }
            self._power_status_time = now
            self._feedback_condition.notify_all()

    @staticmethod
    def _motor_ids(side: str) -> list[int]:
        motor_ids = []
        if side in ("left", "both"):
            motor_ids.extend(range(11, 18))
        if side in ("right", "both"):
            motor_ids.extend(range(21, 28))
        return motor_ids

    @staticmethod
    def _mirror_pose(left_pose: list[float]) -> list[float]:
        return [
            left_pose[0], -left_pose[1], -left_pose[2],
            left_pose[3], -left_pose[4], left_pose[5], -left_pose[6],
        ]

    @classmethod
    def _pose_violations(
            cls, side: str, left_pose: list[float]) -> list[dict]:
        if len(left_pose) != 7:
            return [{"side": side, "error": "pose_length", "actual": len(left_pose)}]
        selected = []
        if side in ("left", "both"):
            selected.append(("left", left_pose, cls._LEFT_POSE_LIMITS))
        if side in ("right", "both"):
            selected.append((
                "right", cls._mirror_pose(left_pose), cls._RIGHT_POSE_LIMITS))
        violations = []
        for arm_side, pose, limits in selected:
            for index, (value, bounds) in enumerate(zip(pose, limits)):
                lower, upper = bounds
                if float(value) < lower or float(value) > upper:
                    violations.append({
                        "side": arm_side,
                        "joint": cls._JOINT_NAMES[index],
                        "value_deg": float(value),
                        "minimum_deg": lower,
                        "maximum_deg": upper,
                    })
        return violations

    @classmethod
    def _target_positions(
            cls, side: str, left_pose: list[float]) -> dict[int, float]:
        right_pose = cls._mirror_pose(left_pose)
        targets = {}
        if side in ("left", "both"):
            targets.update({
                11 + index: _deg2rad(float(deg))
                for index, deg in enumerate(left_pose)
            })
        if side in ("right", "both"):
            targets.update({
                21 + index: _deg2rad(float(deg))
                for index, deg in enumerate(right_pose)
            })
        return targets

    def _error_result(self, code: str, message: str, **details) -> dict:
        result = {
            "state": "error",
            "error": message,
            "code": code,
        }
        result.update(details)
        return result

    def _active_motor_faults(self, motor_ids: list[int]) -> list[dict]:
        faults = []
        for motor_id in motor_ids:
            status = self._arm_status.get(motor_id)
            if status is None or status["error"] == 0:
                continue
            error_code = status["error"]
            faults.append({
                "motor_id": motor_id,
                "joint": _ALL_JOINTS.get(motor_id, f"motor_{motor_id}"),
                "error_code": error_code,
                "description": _MOTOR_ERROR_DESCRIPTIONS.get(
                    error_code, "unknown_vendor_error"),
            })
        return faults

    def _preflight(self, side: str) -> dict | None:
        if not self._publisher:
            return self._error_result(
                "publisher_not_initialized", "arm command publisher is not initialized")
        now = time.monotonic()
        motor_ids = self._motor_ids(side)
        with self._feedback_condition:
            if self._arm_status_time is None:
                return self._error_result(
                    "arm_status_unavailable",
                    "No /arm/status received; arm controller may not be running",
                    diagnosis=[
                        "check robot body-control program",
                        "complete robot self-check and confirm Ready state",
                        "check ROS_DOMAIN_ID and /arm/status",
                    ],
                )
            status_age = now - self._arm_status_time
            if status_age > self._STATUS_MAX_AGE:
                return self._error_result(
                    "arm_status_stale",
                    f"/arm/status is stale ({status_age:.2f}s)",
                    diagnosis=[
                        "check robot body-control program",
                        "check ROS communication",
                    ],
                )
            missing = [
                motor_id for motor_id in motor_ids
                if motor_id not in self._arm_status
            ]
            if missing:
                return self._error_result(
                    "arm_motors_missing",
                    "Selected arm motors are missing from /arm/status",
                    missing_motor_ids=missing,
                )
            faults = self._active_motor_faults(motor_ids)
            if faults:
                return self._error_result(
                    "arm_motor_fault", "Selected arm has active motor faults",
                    faults=faults,
                )
            if (self._power_status_time is not None
                    and now - self._power_status_time <= self._STATUS_MAX_AGE):
                if (self._power_status.get("is_estop")
                        or self._power_status.get("is_remote_estop")):
                    return self._error_result(
                        "emergency_stop_active",
                        "Physical or remote emergency stop is active",
                        power_status=dict(self._power_status),
                    )
                if not self._power_status.get("is_power_on", True):
                    return self._error_result(
                        "robot_power_off", "Robot power board reports power off",
                        power_status=dict(self._power_status),
                    )
        return None

    def _feedback_snapshot(self, side: str) -> tuple[int, dict[int, float]]:
        with self._feedback_condition:
            return self._arm_status_seq, {
                motor_id: self._arm_status[motor_id]["pos"]
                for motor_id in self._motor_ids(side)
                if motor_id in self._arm_status
            }

    def _wait_for_arm_feedback(
            self, side: str, target_pose: list[float],
            baseline_seq: int, baseline: dict[int, float]) -> dict:
        motor_ids = self._motor_ids(side)
        targets = self._target_positions(side, target_pose)
        deadline = time.monotonic() + self._FEEDBACK_TIMEOUT
        received_new_status = False
        with self._feedback_condition:
            while True:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    break
                if self._arm_status_seq <= baseline_seq:
                    self._feedback_condition.wait(remaining)
                    continue
                received_new_status = True
                faults = self._active_motor_faults(motor_ids)
                if faults:
                    return self._error_result(
                        "arm_motor_fault_after_command",
                        "Arm motor fault appeared after command",
                        faults=faults,
                    )
                positions = {
                    motor_id: self._arm_status[motor_id]["pos"]
                    for motor_id in motor_ids
                }
                moved = max(
                    abs(positions[motor_id] - baseline[motor_id])
                    for motor_id in motor_ids
                )
                target_error = max(
                    abs(positions[motor_id] - targets[motor_id])
                    for motor_id in motor_ids
                )
                if (moved >= self._MOVE_THRESHOLD_RAD
                        or target_error <= self._TARGET_TOLERANCE_RAD):
                    return {
                        "state": "moving",
                        "status_topic": "/arm/status",
                        "max_movement_deg": round(_rad2deg(moved), 2),
                        "max_target_error_deg": round(
                            _rad2deg(target_error), 2),
                    }
                self._feedback_condition.wait(0.05)
        if not received_new_status:
            return self._error_result(
                "arm_feedback_timeout",
                "Command was published but no new /arm/status was received",
                diagnosis=[
                    "check arm controller and ROS communication",
                    "confirm robot self-check completed and robot is Ready",
                ],
            )
        return self._error_result(
            "arm_no_motion",
            "Command was published and arm status updated, but no joint moved",
            diagnosis=[
                "robot may not be Ready or self-check may be incomplete",
                "arm controller may be disabled or rejecting commands",
                "another node may be publishing competing /arm/cmd_pos commands",
                "check joint load and mechanical interference; do not exceed the mapped vendor-rated current",
            ],
        )

    def _publish_pose(self, side: str, left_pose: list[float], speed: float) -> dict:
        if not self._publisher:
            return {"error": "publisher not initialized"}
        if len(left_pose) != 7:
            return {"error": "internal pose must have 7 values"}
        violations = self._pose_violations(side, left_pose)
        if violations:
            return self._error_result(
                "arm_pose_out_of_range",
                "Arm pose exceeds URDF joint limits",
                violations=violations,
            )
        try:
            from bodyctrl_msgs.msg import CmdSetMotorPosition, SetMotorPosition
            # Mirror the lateral axes for the right arm. All values remain within
            # the URDF limits used by the existing arm card.
            right_pose = self._mirror_pose(left_pose)
            selected = []
            if side in ("left", "both"):
                selected.append((11, left_pose))
            if side in ("right", "both"):
                selected.append((21, right_pose))
            msg = CmdSetMotorPosition()
            msg.cmds = []
            for base_id, pose in selected:
                for index, deg in enumerate(pose):
                    cmd = SetMotorPosition()
                    motor_id = base_id + index
                    cmd.name = motor_id
                    cmd.pos = _deg2rad(float(deg))
                    cmd.spd = speed
                    cmd.cur = _RATED_MOTOR_CURRENT_A[motor_id]
                    msg.cmds.append(cmd)
            self._publisher.publish(msg)
            return {"state": "moving", "side": side, "joints": len(msg.cmds)}
        except Exception as e:
            return {"error": str(e)}


# ══════════════════════════════════════════════════════════
# WaistPlugin (actuator)
# ══════════════════════════════════════════════════════════════════════════

class WaistPlugin:
    """腰部偏航 + 腿部升降 (三电机联动: hip+knee+pitch)

    调用格式:
      - 腰偏航: {"action": "move_waist", "yaw": 30, "speed": 0.5}
      - 腿升降: {"action": "move_leg", "height": 50, "speed": 0.5}
      - 腰归零: {"action": "set_zero_waist"}
      - 腿归零: {"action": "set_zero_leg"}

    height: 0=最低(归零位), 100=最高, 三电机线性插值联动
    """

    def __init__(self, plugin_config: dict, namespace: str, ros2):
        self._ns = namespace
        self._ros2 = ros2
        self._pub_node = Node("tianyi2_waist_cmd", context=ros2.ctx_tianyi)
        ros2.executor_tianyi.add_node(self._pub_node)
        self._pub_waist = None
        self._pub_leg = None

    def get_tool(self) -> dict:
        return {
            "name": "waist",
            "type": "actuator",
            "description": "天轶2.0 腰部偏航+腿部升降 — yaw (-120°~120°), height (0-100), 俯仰角已禁用",
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {"type": "string", "enum": ["move_waist", "move_leg", "set_zero_waist", "set_zero_leg"],
                               "description": "控制模式"},
                    "yaw": {"type": "number", "description": "腰偏航角(度), 范围[-120, 120], 默认0"},
                    "height": {"type": "number", "description": "腿部升降高度(0-100), 0=最低(归零位), 100=最高, 默认0"},
                    "speed": {"type": "number", "description": "运动速度(rad/s), 默认0.5"},
                },
                "required": ["action"],
                "x-action-params": {
                    "move_waist": {"params": ["yaw", "speed"],
                                 "description": "腰部偏航: 控制yaw角度(-120°~120°)"},
                    "move_leg": {"params": ["height", "speed"],
                                  "description": "腿部升降: 三电机联动, 线性插值, height 0-100"},
                    "set_zero_waist": {"params": [],
                                 "description": "腰部归零: yaw=0°"},
                    "set_zero_leg": {"params": [],
                                 "description": "腿部归零: height=0 (回到归零位)"},
                },
            },
        }

    def start(self):
        try:
            from bodyctrl_msgs.msg import CmdSetMotorPosition
            self._pub_waist = self._pub_node.create_publisher(CmdSetMotorPosition, "/waist/cmd_pos", _RELIABLE_QOS)
            self._pub_leg   = self._pub_node.create_publisher(CmdSetMotorPosition, "/leg/cmd_pos", _RELIABLE_QOS)
            print("[WaistPlugin] publishers created (/waist/cmd_pos, /leg/cmd_pos)")
        except ImportError as e:
            print(f"[WaistPlugin] WARNING: {e}")

    def stop(self):
        pass

    def dispatch(self, action: str, args: dict) -> dict:
        if action == "move_waist":
            return self._send_yaw(args.get("yaw", 0), args.get("speed", 0.5))
        if action == "move_leg":
            return self._send_leg_height(args.get("height", 0), args.get("speed", 0.5))
        if action == "set_zero_waist":
            return self._send_yaw(0)
        if action == "set_zero_leg":
            return self._send_leg_height(0)
        if action in ("start", "info"):
            return {"state": "ready"}
        if action == "stop":
            return {"state": "idle"}
        return {"ok": False, "code": "INVALID_ARGUMENT", "message": f"unknown action: {action}"}

    def _send_yaw(self, yaw_deg: float, speed_rad_s: float = 0.5) -> dict:
        if not self._pub_waist:
            return {"ok": False, "code": "COMMUNICATION_ERROR", "message": "publisher not ready"}
        try:
            from bodyctrl_msgs.msg import CmdSetMotorPosition, SetMotorPosition
            msg = CmdSetMotorPosition()
            mid = 31
            lim = _JOINT_LIMITS[mid]
            pos_deg = _clamp(yaw_deg, lim[0], lim[1])
            clamped = (pos_deg != yaw_deg)
            spd = _clamp(speed_rad_s, 0, _rpm2rads(lim[2]))
            cmd = SetMotorPosition()
            cmd.name = mid; cmd.pos = _deg2rad(pos_deg); cmd.spd = spd; cmd.cur = 5.0
            msg.cmds.append(cmd)
            if clamped:
                return {"ok": False, "code": "JOINT_LIMIT_VIOLATION",
                        "message": f"waist yaw out of range [{lim[0]}°, {lim[1]}°]"}
            self._pub_waist.publish(msg)
            return {"ok": True, "card": "waist", "action": "move_waist",
                    "applied": [{"name": _ALL_JOINTS[mid], "pos_deg": pos_deg, "spd_rad_s": spd}]}
        except Exception as e:
            return {"ok": False, "code": "COMMUNICATION_ERROR", "message": str(e)}

    def _send_leg_height(self, height: float, speed_rad_s: float = 0.5) -> dict:
        """三电机联动升降: height 0-100 线性插值, 基于实测端点。
        51(hip)+52(knee) → /leg/cmd_pos, 32(pitch) → /waist/cmd_pos.

        height=0   → 51= 0.087, 52=-0.350, 32=-0.087 (归零位)
        height=50  → 51=-0.305, 52=-0.001, 32= 0.305 (中间位)
        height=100 → 51=-0.698, 52= 0.348, 32= 0.698 (最高位)

        约束: pos51+pos52≈-0.35, pos32≈-pos51
        """
        if not self._pub_leg or not self._pub_waist:
            return {"ok": False, "code": "COMMUNICATION_ERROR", "message": "publisher not ready"}
        try:
            from bodyctrl_msgs.msg import CmdSetMotorPosition, SetMotorPosition

            # 线性插值: t ∈ [0, 1], 基于实测端点 (level 1 ↔ level 9)
            t = height / 100.0
            zero = _LEG_LEVELS[1]   # height=0
            maxv = _LEG_LEVELS[9]   # height=100

            # leg: 51(hip) + 52(knee) → /leg/cmd_pos
            msg_leg = CmdSetMotorPosition()
            results = []
            for mid in (51, 52):
                target_rad = zero[mid] + t * (maxv[mid] - zero[mid])
                lim = _JOINT_LIMITS[mid]
                lo_rad, hi_rad = _deg2rad(lim[0]), _deg2rad(lim[1])
                pos_rad = _clamp(target_rad, lo_rad, hi_rad)
                clamped = (pos_rad != target_rad)
                spd = _clamp(speed_rad_s, 0, _rpm2rads(lim[2]))
                cmd = SetMotorPosition()
                cmd.name = mid; cmd.pos = pos_rad; cmd.spd = spd; cmd.cur = 5.0
                msg_leg.cmds.append(cmd)
                results.append({"name": _ALL_JOINTS[mid], "pos_rad": round(pos_rad, 5)})
                if clamped:
                    return {"ok": False, "code": "JOINT_LIMIT_VIOLATION",
                            "message": f"leg {mid} target {target_rad:.5f} rad out of range"}
            self._pub_leg.publish(msg_leg)

            # waist: 32(pitch) → /waist/cmd_pos
            mid = 32
            target_rad = zero[mid] + t * (maxv[mid] - zero[mid])
            lim = _JOINT_LIMITS[mid]
            lo_rad, hi_rad = _deg2rad(lim[0]), _deg2rad(lim[1])
            pos_rad = _clamp(target_rad, lo_rad, hi_rad)
            clamped = (pos_rad != target_rad)
            spd = _clamp(speed_rad_s, 0, _rpm2rads(lim[2]))
            msg_waist = CmdSetMotorPosition()
            cmd = SetMotorPosition()
            cmd.name = mid; cmd.pos = pos_rad; cmd.spd = spd; cmd.cur = 5.0
            msg_waist.cmds.append(cmd)
            results.append({"name": _ALL_JOINTS[mid], "pos_rad": round(pos_rad, 5)})
            if clamped:
                return {"ok": False, "code": "JOINT_LIMIT_VIOLATION",
                        "message": f"waist {mid} target {target_rad:.5f} rad out of range"}
            self._pub_waist.publish(msg_waist)

            return {"ok": True, "card": "waist", "action": "move_leg", "height": height,
                    "applied": results}
        except Exception as e:
            return {"ok": False, "code": "COMMUNICATION_ERROR", "message": str(e)}


# ══════════════════════════════════════════════════════════════════════════════
# HandPlugin (actuator)
# ══════════════════════════════════════════════════════════════════════════════

class HandPlugin:
    """Inspire 灵巧手控制.

    预设手势 (action = thumbs_up / fist / victory / handshake / point / ok / open_palm):
        选择 side (left/right/both) 直接执行对应手势。
    set_fingers_raw (底层全量控指):
        选择 side 后逐指输入 0-100 百分比，全量下发（未填默认0=张开）。
    reset:
        先清除指定手所有手指关节错误锁，再执行力控校准（手指会自动运动）。
    """

    # Finger ID: 1=little, 2=ring, 3=middle, 4=index, 5=thumb_bend, 6=thumb_rotation
    _FINGER_NAMES = ["little", "ring", "middle", "index", "thumb_bend", "thumb_rotation"]

    # 0 表示张开，100 表示弯曲到握紧。顺序见 _FINGER_NAMES。
    _GESTURE_PRESETS = {
        "thumbs_up": [100, 100, 100, 100, 0, 0],
        "fist": [100, 100, 100, 100, 92, 0],
        "victory": [100, 100, 0, 0, 100, 0],
        "handshake": [50, 50, 50, 50, 0, 30],
        "point": [100, 100, 100, 0, 92, 0],
        "ok": [0, 0, 0, 60, 50, 50],
        "open_palm": [0, 0, 0, 0, 0, 0],
    }

    _GESTURE_LABELS = {
        "thumbs_up": "点赞",
        "fist": "握拳",
        "victory": "比耶",
        "handshake": "握手",
        "point": "指向",
        "ok": "ok",
        "open_palm": "张开手掌",
    }

    def __init__(self, plugin_config: dict, namespace: str, ros2):
        self._ns = namespace
        self._ros2 = ros2
        self._pub_node = Node("tianyi2_hand_pub", context=ros2.ctx_tianyi)
        ros2.executor_tianyi.add_node(self._pub_node)
        self._left_pub = None
        self._right_pub = None
        self._left_clear_error = None
        self._right_clear_error = None
        self._left_calibrate = None
        self._right_calibrate = None
        self._srv_timeout = plugin_config.get("call_timeout", 3.0)

    def get_tool(self) -> dict:
        _GESTURE_ACTIONS = list(self._GESTURE_PRESETS.keys())
        return {
            "name": "hand",
            "type": "actuator",
            "description": "天轶2.0 Inspire 灵巧手 — 预设手势 + 底层全量控指 + 重置",
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {"type": "string",
                               "enum": _GESTURE_ACTIONS + ["set_fingers_raw", "reset"],
                               "description": "控制模式: 预设手势(thumbs_up/fist/victory/handshake/point/ok/open_palm) | set_fingers_raw=底层全量控指 | reset=清除错误+力控校准"},
                    "side": {"type": "string", "enum": ["left", "right", "both"],
                             "description": "控制哪只手"},
                    "little": {"type": "number",
                               "description": "little finger (0=open, 100=closed)"},
                    "ring": {"type": "number",
                             "description": "ring finger (0=open, 100=closed)"},
                    "middle": {"type": "number",
                               "description": "middle finger (0=open, 100=closed)"},
                    "index": {"type": "number",
                              "description": "index finger (0=open, 100=closed)"},
                    "thumb_bend": {"type": "number",
                                   "description": "thumb bend (0=open, 100=closed)"},
                    "thumb_rotation": {"type": "number",
                                       "description": "thumb rotation"},
                },
                "required": ["action"],
                "x-action-params": {
                    **{g: {"params": ["side"],
                           "description": f"预设手势: {self._GESTURE_LABELS[g]}"}
                       for g in _GESTURE_ACTIONS},
                    "set_fingers_raw": {
                        "params": ["side", "little", "ring", "middle", "index", "thumb_bend", "thumb_rotation"],
                        "description": "底层全量控指: 逐指输入角度(0=张开,100=握紧), 不填默认0, 直接下发硬件",
                    },
                    "reset": {
                        "params": ["side"],
                        "description": "先清除手指关节错误锁，再执行力控校准零点（手指会自动运动）",
                    },
                },
            },
        }

    def start(self):
        try:
            from sensor_msgs.msg import JointState
            self._left_pub = self._pub_node.create_publisher(
                JointState, "/inspire_hand/ctrl/left_hand", _RELIABLE_QOS)
            self._right_pub = self._pub_node.create_publisher(
                JointState, "/inspire_hand/ctrl/right_hand", _RELIABLE_QOS)
            print("[HandPlugin] publishers created")
        except ImportError as e:
            print(f"[HandPlugin] WARNING: msg import failed ({e})")

        try:
            from bodyctrl_msgs.srv import SetClearError
            self._left_clear_error = self._pub_node.create_client(
                SetClearError, "/inspire_hand/set_clear_error/left_hand")
            self._right_clear_error = self._pub_node.create_client(
                SetClearError, "/inspire_hand/set_clear_error/right_hand")
            print("[HandPlugin] clear_error clients created")
        except ImportError as e:
            print(f"[HandPlugin] WARNING: clear_error service import failed ({e})")

        try:
            from bodyctrl_msgs.srv import SetGestureForceCalibration
            self._left_calibrate = self._pub_node.create_client(
                SetGestureForceCalibration, "/inspire_hand/set_gesture_force_calibration/left_hand")
            self._right_calibrate = self._pub_node.create_client(
                SetGestureForceCalibration, "/inspire_hand/set_gesture_force_calibration/right_hand")
            print("[HandPlugin] calibrate clients created")
        except ImportError as e:
            print(f"[HandPlugin] WARNING: calibrate service import failed ({e})")

    def stop(self):
        pass

    def dispatch(self, action: str, args: dict) -> dict:
        # ── 预设手势 (thumbs_up / fist / victory / handshake / point / ok / open_palm) ──
        if action in self._GESTURE_PRESETS:
            side = args.get("side", "both")
            if side not in ("left", "right", "both"):
                return {"error": "side must be left, right, or both"}
            result = self._send_angles(side, self._GESTURE_PRESETS[action])
            if "error" not in result:
                result["mode"] = "gesture"
                result["gesture"] = action
                result["gesture_label"] = self._GESTURE_LABELS[action]
            return result

        # ── 底层全量控指 ──
        elif action == "set_fingers_raw":
            side = args.get("side", "both")
            if side not in ("left", "right", "both"):
                return {"error": "side must be left, right, or both"}
            keys = ["little", "ring", "middle", "index", "thumb_bend", "thumb_rotation"]
            angles = []
            for k in keys:
                v = args.get(k)
                if v is None:
                    angles.append(0)
                else:
                    angles.append(max(0, min(100, int(v))))
            result = self._send_angles(side, angles)
            if "error" not in result:
                result["mode"] = "set_fingers_raw"
                result["angles"] = {k: a for k, a in zip(keys, angles)}
            return result

        # ── 重置: 先清除错误锁，再力控校准 ──
        elif action == "reset":
            side = args.get("side", "both")
            if side not in ("left", "right", "both"):
                return {"error": "side must be left, right, or both"}
            clear_result = self._clear_error(side)
            calib_result = self._calibrate(side)
            ok = clear_result.get("ok", False) and calib_result.get("ok", False)
            return {"ok": ok, "card": "hand", "action": "reset",
                    "clear_error": clear_result, "calibrate": calib_result}

        elif action in ("start", "info"):
            return {"state": "ready"}
        elif action == "stop":
            return {"state": "idle"}
        return {"error": f"unknown action: {action}"}

    def _send_angles(self, side: str, angles: list) -> dict:
        if not self._left_pub or not self._right_pub:
            return {"error": "publishers not initialized"}
        try:
            from sensor_msgs.msg import JointState
            # Angles are in percentage (0=open, 100=closed).
            # Hardware maps position 1.0 → open, 0.0 → closed, so invert.
            positions = [(100 - a) / 100.0 for a in angles]

            pubs = []
            if side in ("left", "both"):
                pubs.append(self._left_pub)
            if side in ("right", "both"):
                pubs.append(self._right_pub)

            for pub in pubs:
                msg = JointState()
                msg.name = [str(i + 1) for i in range(6)]
                msg.position = positions
                pub.publish(msg)

            return {"state": "moving", "side": side, "angles": angles}
        except Exception as e:
            return {"error": str(e)}

    def _clear_error(self, side: str) -> dict:
        """清除指定手的所有手指关节错误锁（文档 5.7.7）。"""
        sides = ["left", "right"] if side == "both" else [side]
        results = {}
        ok = True
        for s in sides:
            client = self._left_clear_error if s == "left" else self._right_clear_error
            if not client:
                results[s] = {"ok": False, "message": "client not initialized"}
                ok = False
                continue
            try:
                if not client.wait_for_service(timeout_sec=self._srv_timeout):
                    results[s] = {"ok": False, "message": "service not available"}
                    ok = False
                    continue
                req = client.srv_type.Request()
                resp = client.call(req)
                results[s] = {"ok": True, "accepted": resp.setclear_error_accepted}
            except Exception as e:
                results[s] = {"ok": False, "message": str(e)}
                ok = False
        return {"ok": ok, "card": "hand", "action": "clear_error", "results": results}


    def _calibrate(self, side: str) -> dict:
        """力控校准：手指自动运动以重新标定零点，修复编码器漂移。"""
        sides = ["left", "right"] if side == "both" else [side]
        results = {}
        ok = True
        for s in sides:
            client = self._left_calibrate if s == "left" else self._right_calibrate
            if not client:
                results[s] = {"ok": False, "message": "client not initialized"}
                ok = False
                continue
            try:
                if not client.wait_for_service(timeout_sec=self._srv_timeout):
                    results[s] = {"ok": False, "message": "service not available"}
                    ok = False
                    continue
                req = client.srv_type.Request()
                resp = client.call(req)
                results[s] = {"ok": True, "accepted": resp.calibration_accepted}
            except Exception as e:
                results[s] = {"ok": False, "message": str(e)}
                ok = False
        return {"ok": ok, "card": "hand", "action": "calibrate", "results": results}


# ══════════════════════════════════════════════════════════════════════════════
# TtsPlugin (actuator)
# ══════════════════════════════════════════════════════════════════════════════

class TtsPlugin:
    """语音合成 (lyre TTS)"""

    def __init__(self, plugin_config: dict, namespace: str, ros2):
        self._ns = namespace
        self._ros2 = ros2
        self._srv_node = Node("tianyi2_tts", context=ros2.ctx_tianyi)
        ros2.executor_tianyi.add_node(self._srv_node)
        self._play_client = None
        self._stop_client = None
        self._pause_client = None
        self._resume_client = None
        # 取消信号：interrupt 时 set，通知 _speak_segments 线程停止
        self._cancel_event: threading.Event | None = None
        # ACP: PlayEvent 订阅，用于判断播放真正完成
        self._play_event_sub = None
        self._play_progress_sub = None
        self._pending_play: dict[str, threading.Event] = {}  # sid → Event
        self._pending_play_status: dict[str, int] = {}  # sid → event_code
        self._pending_play_duration: dict[str, float] = {}  # sid → total duration (from progress)
        self._play_event_buffer: dict[str, int] = {}  # 缓存最近的 PlayEvent（防竞态）

    def get_tool(self) -> dict:
        return {
            "name": "tts",
            "type": "actuator",
            "description": "天轶2.0 语音合成 (TTS) — 文字转语音播放",
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {"type": "string", "enum": ["speak", "interrupt", "pause", "resume"],
                               "description": "控制动作"},
                    "text": {"type": "string", "description": "要播放的文本"},
                    "force": {"type": "boolean", "description": "是否强制播放(打断当前播放)", "default": False},
                },
                "required": ["action"],
                "x-completion": {"actions": ["speak"], "timeout": 180},
                "x-action-params": {
                    "speak": {"params": ["text", "force"], "description": "合成并播放文本"},
                    "interrupt": {"params": [], "description": "立即停止播放并丢弃剩余内容，无需再调 pause"},
                    "pause": {"params": [], "description": "暂停播放（可用 resume 恢复，不要与 interrupt 同时使用）"},
                    "resume": {"params": [], "description": "恢复被 pause 暂停的播放"},
                },
                "x-hooks": {
                    "on_interrupt_speak": {"action": "interrupt"},
                },
            },
        }

    def start(self):
        try:
            from lyre_msgs.srv import PlayText, PlayStop, PlayPause, PlayResume
            from lyre_msgs.msg import PlayEvent, PlayProgress
            self._play_client = self._srv_node.create_client(PlayText, "/audio_play/play_text")
            self._stop_client = self._srv_node.create_client(PlayStop, "/audio_play/stop")
            self._pause_client = self._srv_node.create_client(PlayPause, "/audio_play/pause")
            self._resume_client = self._srv_node.create_client(PlayResume, "/audio_play/resume")
            # 订阅播放事件 topic，用于判断播放真正完成
            self._play_event_sub = self._srv_node.create_subscription(
                PlayEvent, "/audio_play/event", self._on_play_event, 10)
            # 订阅播放进度 topic，获取精确总时长用于超时计算
            self._play_progress_sub = self._srv_node.create_subscription(
                PlayProgress, "/audio_play/progress", self._on_play_progress, 10)
            print("[TtsPlugin] service clients + event/progress subscriptions created")
        except ImportError as e:
            print(f"[TtsPlugin] WARNING: msg import failed ({e})")
            return

        # Health check: verify PlayEvent pipeline is working
        self._startup_error = self._lyre_health_check()

    def _lyre_health_check(self) -> str | None:
        """Call play_text and verify PlayEvent arrives. Returns error message or None."""
        import subprocess as _sp
        import time as _time

        for attempt in range(2):
            if attempt > 0:
                # Restart lyre via nsenter on second attempt
                print("[TtsPlugin] health check failed, restarting lyre...", flush=True)
                try:
                    _sp.run(["nsenter", "-t", "1", "-m", "-u", "-i", "-n", "-p", "--",
                             "systemctl", "restart", "lyre"],
                            capture_output=True, timeout=15)
                    _time.sleep(5)
                except Exception as e:
                    print(f"[TtsPlugin] lyre restart failed: {e}", flush=True)

            # Wait for service to be available (poll without spinning — executor thread handles it)
            service_ready = False
            deadline = _time.time() + 5
            while _time.time() < deadline:
                if self._play_client.service_is_ready():
                    service_ready = True
                    break
                _time.sleep(0.2)
            if not service_ready:
                print(f"[TtsPlugin] health check attempt {attempt+1}: play_text service not available", flush=True)
                continue

            # Send a silent test (single dot — minimal TTS)
            from lyre_msgs.srv import PlayText
            req = PlayText.Request()
            req.text = "."
            req.force = True
            future = self._play_client.call_async(req)

            # Wait for response (max 3s) — executor spin thread delivers it
            deadline = _time.time() + 3
            while not future.done() and _time.time() < deadline:
                _time.sleep(0.1)

            if not future.done():
                print(f"[TtsPlugin] health check attempt {attempt+1}: play_text call timeout", flush=True)
                continue

            resp = future.result()
            if resp is None or resp.code != 0:
                print(f"[TtsPlugin] health check attempt {attempt+1}: play_text returned error", flush=True)
                continue

            sid = resp.sid
            # Wait for PlayEvent with this sid (3s timeout)
            # The executor spin thread will call _on_play_event which populates _play_event_buffer
            deadline = _time.time() + 3
            while _time.time() < deadline:
                if sid in self._play_event_buffer:
                    break
                _time.sleep(0.1)

            if sid in self._play_event_buffer:
                # Cleanup test sid from buffers
                self._play_event_buffer.pop(sid, None)
                self._pending_play.pop(sid, None)
                self._pending_play_status.pop(sid, None)
                self._pending_play_duration.pop(sid, None)
                print(f"[TtsPlugin] health check passed (attempt {attempt+1})", flush=True)
                return None  # success
            else:
                print(f"[TtsPlugin] health check attempt {attempt+1}: PlayEvent not received for sid={sid}", flush=True)

        return "Lyre TTS PlayEvent 链路异常：播放成功但无法收到完成事件。已尝试重启 lyre 仍未恢复，请检查 lyre 服务状态。"

    # PlayEvent event codes
    _EVENT_NAMES = {0: "STARTED", 1: "COMPLETED", 2: "STOPPED", 3: "CANCELLED", 4: "FAILED"}

    def _on_play_event(self, msg):
        """PlayEvent callback: 播放完成/停止/失败时解锁对应的 pending."""
        # event: 0=STARTED, 1=COMPLETED, 2=STOPPED, 3=CANCELLED, 4=FAILED
        sid = msg.sid
        event_code = msg.event
        if event_code >= 1:
            # 缓存事件（防止 _wait_cb 还没注册 pending 时 miss）
            self._play_event_buffer[sid] = event_code
            # 如果已注册 pending，直接解锁
            if sid in self._pending_play:
                self._pending_play_status[sid] = event_code
                self._pending_play[sid].set()
            event_name = self._EVENT_NAMES.get(event_code, f"UNKNOWN({event_code})")
            print(f"[TtsPlugin] PlayEvent: sid={sid} {event_name}")

    def _on_play_progress(self, msg):
        """PlayProgress callback: 获取播放总时长，用于精确超时计算。"""
        sid = msg.sid
        duration = msg.duration
        if sid and duration > 0 and sid in self._pending_play:
            # 只更新一次（取第一次收到的 duration）
            if sid not in self._pending_play_duration:
                self._pending_play_duration[sid] = duration
                print(f"[TtsPlugin] PlayProgress: sid={sid} duration={duration:.1f}s")

    def stop(self):
        pass

    def dispatch(self, action: str, args: dict) -> dict:
        if action == "speak":
            text = args.get("text", "")
            force = args.get("force", False)
            if not text:
                return {"error": "text is required"}
            return self._speak(text, force)
        elif action == "interrupt":
            # 先通知后台线程停止循环
            ce = self._cancel_event
            if ce:
                ce.set()
            return self._call_empty_service(self._stop_client, "interrupt")
        elif action == "pause":
            return self._call_empty_service(self._pause_client, "pause")
        elif action == "resume":
            return self._call_empty_service(self._resume_client, "resume")
        elif action in ("start", "info"):
            if hasattr(self, '_startup_error') and self._startup_error:
                return {"state": "error", "message": self._startup_error}
            return {"state": "ready"}
        return {"error": f"unknown action: {action}"}

    def _speak(self, text: str, force: bool) -> dict:
        if not self._play_client:
            return {"error": "service client not initialized"}
        try:
            import uuid as _uuid
            action_id = f"tts-{_uuid.uuid4().hex[:8]}"

            # 分段：超过 280 字按标点切分，避免超长文本 PlayEvent 丢失
            segments = self._split_text(text, max_chars=280)

            # 取消旧的播放线程（如果有）— 同时对 lyre 发 stop
            old_ce = self._cancel_event
            if old_ce and not old_ce.is_set():
                old_ce.set()
                # 停止 lyre 当前播放，确保新请求能拿到 sid
                if self._stop_client:
                    try:
                        req = type(self._stop_client.srv_type.Request)()
                        self._stop_client.call_async(req)
                    except Exception:
                        pass
            cancel_event = threading.Event()
            self._cancel_event = cancel_event

            # Background thread: 逐段播放, 全部完成后 ACP callback
            threading.Thread(
                target=self._speak_segments,
                args=(segments, force, action_id, text, cancel_event),
                daemon=True,
            ).start()
            return {"state": "speaking", "action_id": action_id, "text": text[:50],
                    "segments": len(segments)}
        except Exception as e:
            return {"error": str(e)}

    @staticmethod
    def _split_text(text: str, max_chars: int = 280) -> list[str]:
        """按标点分段，每段不超过 max_chars 字。"""
        import re as _re
        sentences = _re.split(r'(?<=[。！？；\n])', text)
        segments = []
        current = ""
        for sent in sentences:
            if not sent:
                continue
            if len(current) + len(sent) > max_chars and current:
                segments.append(current)
                current = sent
            else:
                current += sent
        if current:
            segments.append(current)
        return segments if segments else [text]

    def _speak_segments(self, segments: list, force: bool, action_id: str, full_text: str, cancel_event: threading.Event):
        """顺序播放多段文本，全部完成后 ACP callback。支持 cancel_event 打断。"""
        import time as _t
        from lyre_msgs.srv import PlayText

        overall_status = "completed"
        sid = None  # 记录最后一个成功的 sid（用于 ACP callback）

        for i, seg_text in enumerate(segments):
            # 每段开始前检查取消
            if cancel_event.is_set():
                overall_status = "cancelled"
                print(f"[TtsPlugin] cancelled before seg {i+1}/{len(segments)}")
                break

            is_last = (i == len(segments) - 1)
            # 发送 PlayText service — 第一段用调用方指定的 force，后续段不 force
            req = PlayText.Request()
            req.text = seg_text
            req.force = force if i == 0 else False
            req.last = is_last
            future = self._play_client.call_async(req)

            # Phase 1: 等 service response（获取 sid）
            timeout_service = 10.0
            start = _t.time()
            while not future.done() and _t.time() - start < timeout_service:
                if cancel_event.is_set():
                    break
                _t.sleep(0.1)

            if cancel_event.is_set():
                overall_status = "cancelled"
                print(f"[TtsPlugin] cancelled during service wait seg {i+1}/{len(segments)}")
                break

            seg_sid = None
            if future.done():
                result = future.result()
                if result:
                    seg_sid = getattr(result, 'sid', None)
            if seg_sid:
                sid = seg_sid

            # Phase 2: 等 PlayEvent
            if seg_sid:
                buffered = self._play_event_buffer.pop(seg_sid, None)
                if buffered is not None:
                    # 被 interrupt 停止的情况
                    if buffered >= 2 and cancel_event.is_set():
                        overall_status = "cancelled"
                        print(f"[TtsPlugin] cancelled (buffered STOPPED/CANCELLED) seg {i+1}/{len(segments)}")
                        self._pending_play_duration.pop(seg_sid, None)
                        break
                    seg_status = "completed" if buffered == 1 else "error"
                    print(f"[TtsPlugin] seg {i+1}/{len(segments)} from buffer: sid={seg_sid} event={buffered}")
                else:
                    ev = threading.Event()
                    self._pending_play[seg_sid] = ev
                    _t.sleep(0.3)
                    reported_duration = self._pending_play_duration.get(seg_sid)
                    if reported_duration and reported_duration > 0:
                        play_timeout = reported_duration + 8.0
                    else:
                        # 更宽松: 2.5字/秒 + 10s buffer
                        play_timeout = len(seg_text) / 2.5 + 10.0
                    # 分段等待，每 0.5s 检查一次 cancel
                    waited = 0.0
                    timed_out = True
                    while waited < play_timeout:
                        if cancel_event.is_set():
                            break
                        chunk = min(0.5, play_timeout - waited)
                        if ev.wait(timeout=chunk):
                            timed_out = False
                            break
                        waited += chunk

                    if cancel_event.is_set():
                        overall_status = "cancelled"
                        self._pending_play.pop(seg_sid, None)
                        self._pending_play_status.pop(seg_sid, None)
                        self._pending_play_duration.pop(seg_sid, None)
                        self._play_event_buffer.pop(seg_sid, None)
                        print(f"[TtsPlugin] cancelled during PlayEvent wait seg {i+1}/{len(segments)}")
                        break
                    elif timed_out:
                        seg_status = "error"
                        print(f"[TtsPlugin] PlayEvent timeout: seg {i+1}/{len(segments)} sid={seg_sid}, waited {play_timeout:.0f}s (duration={reported_duration})")
                    else:
                        event_code = self._pending_play_status.get(seg_sid, 1)
                        # STOPPED(2) 或 CANCELLED(3) + cancel_event → 被打断
                        if event_code >= 2 and cancel_event.is_set():
                            overall_status = "cancelled"
                            self._pending_play.pop(seg_sid, None)
                            self._pending_play_status.pop(seg_sid, None)
                            self._pending_play_duration.pop(seg_sid, None)
                            self._play_event_buffer.pop(seg_sid, None)
                            print(f"[TtsPlugin] cancelled (PlayEvent STOPPED) seg {i+1}/{len(segments)}")
                            break
                        # event_code: 1=COMPLETED, 2=STOPPED (也算完成), 3=CANCELLED, 4=FAILED
                        seg_status = "completed" if event_code <= 2 else "error"
                        if event_code > 2:
                            event_name = self._EVENT_NAMES.get(event_code, f"UNKNOWN({event_code})")
                            print(f"[TtsPlugin] seg {i+1}/{len(segments)} failed: {event_name} (code={event_code})")
                        elif event_code == 2:
                            print(f"[TtsPlugin] seg {i+1}/{len(segments)} STOPPED (treated as completed)")
                    self._pending_play.pop(seg_sid, None)
                    self._pending_play_status.pop(seg_sid, None)
                self._pending_play_duration.pop(seg_sid, None)
                self._play_event_buffer.pop(seg_sid, None)

                if seg_status == "error" and not is_last:
                    # 某段 error 但还有后续段，继续播下一段
                    print(f"[TtsPlugin] seg {i+1} error, continuing to next segment")
                    continue
                elif seg_status == "error" and is_last:
                    overall_status = "error"
            elif not seg_sid:
                # 没拿到 sid，fallback 按字数估算（但也要检查 cancel）
                fallback_s = len(seg_text) / 2.5 + 5.0
                waited = 0.0
                while waited < fallback_s:
                    if cancel_event.is_set():
                        overall_status = "cancelled"
                        break
                    _t.sleep(min(0.5, fallback_s - waited))
                    waited += 0.5
                if cancel_event.is_set() and overall_status != "cancelled":
                    overall_status = "cancelled"
                if overall_status == "cancelled":
                    print(f"[TtsPlugin] cancelled during fallback wait seg {i+1}/{len(segments)}")
                    break
                print(f"[TtsPlugin] no sid from service seg {i+1}, fallback sleep {fallback_s:.0f}s")

        # ACP callback
        try:
            import urllib.request, ssl, json as _json, os as _os
            url = _os.environ.get("AGENT_CORE_URL", "https://localhost:15678")
            ctx = ssl.create_default_context()
            ctx.check_hostname = False
            ctx.verify_mode = ssl.CERT_NONE
            p = _json.dumps({"action_id": action_id, "status": overall_status,
                             "result": {"action": "speak", "sid": sid or "unknown",
                                        "segments": len(segments)}}).encode()
            r = urllib.request.Request(f"{url}/api/acp/complete", data=p,
                                      headers={"Content-Type": "application/json"}, method="POST")
            urllib.request.urlopen(r, timeout=5, context=ctx)
            print(f"[TtsPlugin] ACP {overall_status}: {action_id} ({len(segments)} segs, {len(full_text)} chars)")
        except Exception as e:
            print(f"[TtsPlugin] ACP callback failed: {e}")

    def _call_empty_service(self, client, action_name: str) -> dict:
        if not client:
            return {"error": f"{action_name} service client not initialized"}
        try:
            req = client.srv_type.Request()
            client.call_async(req)
            return {"state": action_name}
        except Exception as e:
            return {"error": str(e)}


# ══════════════════════════════════════════════════════════════════════════════
# VoicePlayActuatorPlugin (actuator) — 卡名: voice_play
# ══════════════════════════════════════════════════════════════════════════════

_URL_PRECHECK_TIMEOUT = 1.5


def _check_url_reachable(url: str) -> tuple[bool, str]:
    """对远端音频 URL 做 HEAD 预检。返回 (reachable, reason)。"""
    if not url:
        return False, "empty url"
    if not (url.startswith("http://") or url.startswith("https://")):
        return False, "url must start with http:// or https://"
    try:
        # -I = HEAD, -L 跟随重定向, --max-time 总超时, -sS 静默但显示错误
        r = subprocess.run(
            ["curl", "-I", "-L", "-sS", "--max-time", str(_URL_PRECHECK_TIMEOUT),
             "-o", "/dev/null", "-w", "%{http_code}", url],
            capture_output=True, text=True, timeout=_URL_PRECHECK_TIMEOUT + 0.5,
        )
    except subprocess.TimeoutExpired:
        return False, f"precheck timeout > {_URL_PRECHECK_TIMEOUT}s"
    except Exception as e:  # noqa: BLE001
        return False, f"precheck error: {e}"
    code = (r.stdout or "").strip()
    if code == "200":
        return True, "ok"
    return False, f"HTTP {code}" if code else "no response"


class VoicePlayActuatorPlugin:
    """音频播放控制 (lyre_msgs service) — 卡名: voice_play

    Actions:
      play_file  → /audio_play/play_file  (PlayFile)
      play_url   → /audio_play/play_url   (PlayUrl)
      play_text  → /audio_play/play_text  (PlayText)
      stop       → /audio_play/stop       (PlayStop)
      pause      → /audio_play/pause      (PlayPause)
      resume     → /audio_play/resume     (PlayResume)

    play_url 前会先做 1.5s HTTP HEAD 预检,不可达直接返回 URL_UNREACHABLE,
    不进入 service call 阶段,避免浪费 5s service 超时。
    """

    def __init__(self, plugin_config: dict, namespace: str, ros2):
        self._ns = namespace
        self._ros2 = ros2
        self._srv_node = Node("tianyi2_voice_play", context=ros2.ctx_tianyi)
        ros2.executor_tianyi.add_node(self._srv_node)
        self._clients = {}
        self._types = {}

    def get_tool(self) -> dict:
        return {
            "name": "voice_play",
            "type": "actuator",
            "description": (
                "天轶2.0 Pro 音频播放控制(本地文件/URL/TTS/停止/暂停/恢复),HIGHLEVEL,lyre_msgs service。"
                "play_url 前会先做 1.5s HTTP HEAD 预检, 不可达直接返回 URL_UNREACHABLE 不浪费 service 超时。"
                "stop/pause/resume 为快速控制(无参数), 服务超时 3s。"
            ),
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {
                        "type": "string",
                        "enum": ["play_file", "play_url", "play_text", "interrupt", "pause", "resume"],
                        "description": "控制模式",
                    },
                    "path": {"type": "string", "description": "本地音频文件绝对路径(play_file)"},
                    "url":  {"type": "string", "description": "远程音频文件URL(play_url, http(s)://)"},
                    "text": {"type": "string", "description": "TTS文本(play_text)"},
                    "force": {"type": "boolean", "description": "强制播放(停止当前任务立即播放,可选)"},
                },
                "required": ["action"],
                "x-completion": {
                    "actions": ["play_text", "play_file", "play_url"],
                    "timeout": 60
                },
                "x-action-params": {
                    "play_file": {"params": ["path", "force"], "description": "播放本地音频文件"},
                    "play_url":  {"params": ["url", "force"],  "description": "播放远程URL音频"},
                    "play_text": {"params": ["text", "force"], "description": "TTS合成并播放文本"},
                    "interrupt":  {"params": [],                 "description": "中止播放(不可恢复)"},
                    "pause":     {"params": [],                 "description": "暂停播放(可恢复)"},
                    "resume":    {"params": [],                 "description": "恢复暂停的播放"},
                },
            },
        }

    def start(self):
        try:
            from lyre_msgs.srv import PlayFile, PlayUrl, PlayText, PlayStop, PlayPause, PlayResume
            self._types = {
                "play_file": ("/audio_play/play_file", PlayFile),
                "play_url":  ("/audio_play/play_url",  PlayUrl),
                "play_text": ("/audio_play/play_text", PlayText),
                "interrupt":  ("/audio_play/stop",      PlayStop),
                "pause":     ("/audio_play/pause",     PlayPause),
                "resume":    ("/audio_play/resume",   PlayResume),
            }
            for key, (svc_name, _svc_type) in self._types.items():
                self._clients[key] = self._srv_node.create_client(self._types[key][1], svc_name)
            print(f"[VoicePlayActuatorPlugin] {len(self._clients)} service clients created")
        except ImportError as e:
            print(f"[VoicePlayActuatorPlugin] WARNING: lyre_msgs.srv import failed ({e})")

    def stop(self):
        pass

    def dispatch(self, action: str, args: dict) -> dict:
        if action in ("start", "info"):
            return {"state": "ready"}
        if action not in self._types:
            return {"ok": False, "code": "INVALID_ARGUMENT",
                    "message": f"unknown action: {action}",
                    "action": action, "timestamp_ms": int(time.time() * 1000)}
        client = self._clients.get(action)
        if client is None:
            return {"ok": False, "code": "NO_SERVICE",
                    "message": f"{action} service client not initialized",
                    "action": action, "timestamp_ms": int(time.time() * 1000)}

        # play_url 先做可达性预检,不可达直接返回,不进入 service call 阶段
        if action == "play_url":
            url = str(args.get("url", "") or "")
            reachable, reason = _check_url_reachable(url)
            if not reachable:
                return {"ok": False, "code": "URL_UNREACHABLE",
                        "message": f"url precheck failed: {reason}",
                        "url": url, "action": action,
                        "timestamp_ms": int(time.time() * 1000)}

        _, svc_type = self._types[action]
        req = svc_type.Request()
        # 公共字段:seq/last/force(不再传 sid — 讯飞服务端自动生成)
        force = bool(args.get("force", False))
        if hasattr(req, "seq"):
            req.seq = 0
        if hasattr(req, "last"):
            req.last = True
        if hasattr(req, "force"):
            req.force = force
        if action == "play_file":
            path = str(args.get("path", "") or "")
            if not path:
                return {"ok": False, "code": "INVALID_ARGUMENT",
                        "message": "path is required",
                        "action": action, "timestamp_ms": int(time.time() * 1000)}
            req.path = path
        elif action == "play_url":
            url = str(args.get("url", "") or "")
            if not url:
                return {"ok": False, "code": "INVALID_ARGUMENT",
                        "message": "url is required",
                        "action": action, "timestamp_ms": int(time.time() * 1000)}
            req.url = url
        elif action == "play_text":
            text = str(args.get("text", "") or "")
            if not text:
                return {"ok": False, "code": "INVALID_ARGUMENT",
                        "message": "text is required",
                        "action": action, "timestamp_ms": int(time.time() * 1000)}
            req.text = text
        # stop/pause/resume 无额外字段

        try:
            future = client.call_async(req)
            # play_file/play_url/play_text: 立即返回 action_id，后台等待完成后回调
            if action in ("play_file", "play_url", "play_text"):
                import uuid as _uuid
                action_id = f"voice-{_uuid.uuid4().hex[:8]}"
                # 后台线程等待 service response 后回调 Agent Core
                def _wait_and_callback(fut, aid, act):
                    try:
                        timeout = 60.0
                        start = time.time()
                        while not fut.done() and time.time() - start < timeout:
                            time.sleep(0.1)
                        if fut.done():
                            result = fut.result()
                            code = int(getattr(result, "code", 0)) if result else -1
                            status = "completed" if code == 0 else "error"
                        else:
                            status = "completed"  # timeout 也视为播完
                    except Exception:
                        status = "error"
                    # ACP callback
                    try:
                        import urllib.request as _urllib
                        import ssl as _ssl
                        import os as _os
                        agent_core_url = _os.environ.get("AGENT_CORE_URL", "https://localhost:15678")
                        ctx = _ssl.create_default_context()
                        ctx.check_hostname = False
                        ctx.verify_mode = _ssl.CERT_NONE
                        payload = json.dumps({"action_id": aid, "status": status,
                                              "result": {"action": act}}).encode()
                        _req = _urllib.Request(f"{agent_core_url}/api/acp/complete",
                                              data=payload, headers={"Content-Type": "application/json"},
                                              method="POST")
                        _urllib.urlopen(_req, timeout=3, context=ctx)
                        print(f"[VoicePlay] ACP complete: {aid} ({status})")
                    except Exception as e:
                        print(f"[VoicePlay] ACP callback failed: {e}")
                threading.Thread(target=_wait_and_callback,
                                 args=(future, action_id, action), daemon=True).start()
                return {
                    "ok": True, "code": 0,
                    "message": "submitted",
                    "action_id": action_id,
                    "action": action,
                    "timestamp_ms": int(time.time() * 1000),
                }
            # interrupt/pause/resume: 短超时等待确认
            timeout = 3.0
            start = time.time()
            while not future.done() and time.time() - start < timeout:
                time.sleep(0.05)
            result = future.result()
            if result is None:
                return {"ok": False, "code": "CALL_FAILED",
                        "message": f"{action} service call returned empty (timeout {timeout:.0f}s)",
                        "action": action, "timestamp_ms": int(time.time() * 1000)}
            code = int(getattr(result, "code", 0))
            return {
                "ok": code == 0,
                "code": code,
                "message": str(getattr(result, "message", "")),
                "action": action,
                "timestamp_ms": int(time.time() * 1000),
            }
        except Exception as e:
            return {"ok": False, "code": "COMMUNICATION_ERROR",
                    "message": str(e), "action": action,
                    "timestamp_ms": int(time.time() * 1000)}


# ══════════════════════════════════════════════════════════════════════════════
# NavPlugin (actuator)
# ══════════════════════════════════════════════════════════════════════════════

class NavPlugin:
    """底盘导航控制 — 自主导航/遥控/旋转"""

    _ACP_ACTIONS = frozenset(("move_to", "rotate", "rotate_to"))
    _POLL_INTERVAL = 1.0
    _STALL_TIMEOUT = 60.0

    def __init__(self, plugin_config: dict, namespace: str, ros2, slamtec_client):
        self._ns = namespace
        self._ros2 = ros2
        self._slamtec = slamtec_client
        self._poll_lock = threading.Lock()
        self._active_poll: str | None = None

        # cmd_vel publisher for direct velocity control (domain 0)
        self._vel_node = Node("tianyi2_nav_vel", context=ros2.ctx_tianyi)
        ros2.executor_tianyi.add_node(self._vel_node)
        self._vel_pub = None

    def get_tool(self) -> dict:
        return {
            "name": "nav",
            "type": "actuator",
            "description": "天轶2.0 底盘导航 — 自主导航到目标点/方向遥控/旋转 (Slamtec轮式底盘)",
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {"type": "string",
                               "enum": ["move_to", "move_by", "rotate", "rotate_to", "cancel", "get_pose"],
                               "description": "导航动作"},
                    "x": {"type": "number", "description": "目标x坐标(米)"},
                    "y": {"type": "number", "description": "目标y坐标(米)"},
                    "direction": {"type": "string",
                                  "enum": ["forward", "backward", "left", "right"],
                                  "description": "移动方向(move_by)"},
                    "angle": {"type": "number", "description": "旋转角度(度), 正=逆时针"},
                    "speed": {"type": "number", "description": "速度比例(0-1), 默认0.5"},
                    "vx": {"type": "number", "description": "前后速度(m/s), 正=前进"},
                    "vy": {"type": "number", "description": "左右速度(m/s), 正=左移"},
                    "vyaw": {"type": "number", "description": "旋转速度(rad/s), 正=逆时针"},
                },
                "required": ["action"],
                "x-completion": {
                    "actions": ["move_to", "rotate", "rotate_to"],
                    "timeout": 180,
                },
                "x-action-params": {
                    "move_to": {"params": ["x", "y", "speed"],
                                "description": "自主导航到目标点(带避障)，系统自动等待到达"},
                    "move_by": {"params": ["direction", "speed"],
                                "description": "方向遥控移动(不避障, 持续500ms)"},
                    "rotate": {"params": ["angle"],
                               "description": "原地旋转指定角度(度)，系统自动等待完成"},
                    "rotate_to": {"params": ["angle"],
                                  "description": "原地旋转到绝对角度(度)，系统自动等待完成"},
                    "cancel": {"params": [],
                             "description": "取消当前导航动作"},
                    "get_pose": {"params": [],
                                 "description": "获取当前位姿(x, y, yaw)"},
                },
                "x-hooks": {
                    "on_interrupt_motion": {"action": "cancel"},
                },
            },
        }

    def start(self):
        try:
            from geometry_msgs.msg import Twist
            self._vel_pub = self._vel_node.create_publisher(Twist, "/cmd_vel", _RELIABLE_QOS)
            print("[NavPlugin] cmd_vel publisher created")
        except ImportError as e:
            print(f"[NavPlugin] WARNING: msg import failed ({e})")

    def stop(self):
        pass

    def dispatch(self, action: str, args: dict) -> dict:
        if action == "move_to":
            x = args.get("x", 0)
            y = args.get("y", 0)
            speed = args.get("speed")
            result = self._slamtec.move_to(x, y, speed_ratio=speed)
            if result.get("error"):
                return {"state": "error", "api_result": result}
            action_id = self._start_poll(action, result, {"x": x, "y": y})
            resp = {"state": "navigating", "target": {"x": x, "y": y}, "api_result": result}
            if action_id:
                resp["action_id"] = action_id
            return resp

        elif action == "move_by":
            direction = args.get("direction", "forward")
            dir_map = {"forward": 0, "backward": 1, "right": 2, "left": 3}
            d = dir_map.get(direction, 0)
            result = self._slamtec.move_by(d)
            return {"state": "moving", "direction": direction, "api_result": result}

        elif action == "rotate":
            angle_deg = args.get("angle", 0)
            angle_rad = _deg2rad(angle_deg)
            result = self._slamtec.rotate(angle_rad)
            if result.get("error"):
                return {"state": "error", "api_result": result}
            action_id = self._start_poll(action, result, {"angle": angle_deg})
            resp = {"state": "rotating", "angle": angle_deg, "api_result": result}
            if action_id:
                resp["action_id"] = action_id
            return resp

        elif action == "rotate_to":
            angle_deg = args.get("angle", 0)
            angle_rad = _deg2rad(angle_deg)
            result = self._slamtec.rotate_to(angle_rad)
            if result.get("error"):
                return {"state": "error", "api_result": result}
            action_id = self._start_poll(action, result, {"angle": angle_deg})
            resp = {"state": "rotating_to", "angle": angle_deg, "api_result": result}
            if action_id:
                resp["action_id"] = action_id
            return resp

        elif action == "cancel":
            result = self._slamtec.cancel_current_action()
            with self._poll_lock:
                self._active_poll = None
            # Also stop cmd_vel
            if self._vel_pub:
                try:
                    from geometry_msgs.msg import Twist
                    self._vel_pub.publish(Twist())  # zero velocity
                except Exception:
                    pass
            return {"state": "stopped", "api_result": result}

        elif action == "get_pose":
            pose = self._slamtec.get_pose()
            return {"pose": pose}

        elif action in ("start", "info"):
            return {"state": "ready"}
        return {"error": f"unknown action: {action}"}

    def _start_poll(self, action: str, api_result: dict, context: dict) -> str | None:
        """Start ACP polling thread for a Slamtec action. Returns action_id or None."""
        from uuid import uuid4
        action_id = f"nav_{action}_{uuid4().hex[:8]}"
        with self._poll_lock:
            self._active_poll = action_id
        threading.Thread(
            target=self._poll_loop,
            args=(action_id, action, context),
            daemon=True,
        ).start()
        return action_id

    def _poll_loop(self, action_id: str, action: str, context: dict):
        """Poll Slamtec for action completion, then fire ACP callback."""
        t0 = time.time()
        last_pose = None
        last_move_time = time.time()

        while True:
            time.sleep(self._POLL_INTERVAL)
            with self._poll_lock:
                if self._active_poll != action_id:
                    return
            elapsed = time.time() - t0

            # Check if action is still running
            try:
                current = self._slamtec.get_nav_status()
            except Exception as exc:
                current = {"error": str(exc)}

            if current.get("error"):
                if elapsed > 180:
                    _acp_notify(action_id, "error", {
                        "action": action, "error": current["error"],
                        "elapsed": round(elapsed, 1), **context,
                    }, "nav")
                    return
                continue

            # Slamtec ActionState: 0=NewBorn, 1=Working, 3=Paused, 4=Done.
            status = current.get("action_state")
            if status == 4:
                result_code = current.get("result")
                if result_code == 0:
                    _acp_notify(action_id, "completed", {
                        "action": action, "elapsed": round(elapsed, 1), **context,
                    }, "nav")
                else:
                    _acp_notify(action_id, "error", {
                        "action": action, "error": f"result_code={result_code}",
                        "elapsed": round(elapsed, 1), **context,
                    }, "nav")
                return

            if status is None or status == -1:
                if elapsed <= 3.0:
                    continue
                _acp_notify(action_id, "error", {
                    "action": action, "error": "action_disappeared",
                    "elapsed": round(elapsed, 1), **context,
                }, "nav")
                return

            # Stall detection for position navigation only.
            if action == "move_to":
                try:
                    pose = self._slamtec.get_pose()
                    if last_pose:
                        dx = pose.get("x", 0) - last_pose.get("x", 0)
                        dy = pose.get("y", 0) - last_pose.get("y", 0)
                        if (dx * dx + dy * dy) > 0.01:  # moved > 0.1m
                            last_pose = pose
                            last_move_time = time.time()
                        elif time.time() - last_move_time > self._STALL_TIMEOUT:
                            _acp_notify(action_id, "error", {
                                "action": action, "error": "stall_timeout",
                                "elapsed": round(elapsed, 1), **context,
                            }, "nav")
                            return
                    else:
                        last_pose = pose
                        last_move_time = time.time()
                except Exception:
                    pass

            # Hard timeout
            if elapsed > 180:
                _acp_notify(action_id, "error", {
                    "action": action, "error": "timeout", "elapsed": 180, **context,
                }, "nav")
                return


# ══════════════════════════════════════════════════════════════════════════════
# HomePlugin (actuator)
# ══════════════════════════════════════════════════════════════════════════════

class HomePlugin:
    """充电桩管理与回桩控制。回桩动作通过 ACP 异步报告最终结果。"""

    _POLL_INTERVAL = 1.0
    _STALL_TIMEOUT = 60.0
    _ACTION_TIMEOUT = 180.0
    _MISSING_ACTION_TIMEOUT = 3.0

    def __init__(self, plugin_config: dict, namespace: str, ros2, slamtec_client):
        self._ns = namespace
        self._slamtec = slamtec_client
        self._poll_lock = threading.Lock()
        self._active_poll: str | None = None

    def get_tool(self) -> dict:
        return {
            "name": "home",
            "type": "actuator",
            "description": "天轶2.0 充电桩管理与回桩。简单流程：机器人停在充电桩对接位置并定位正常后执行 register_dock；新桩会自动设为当前回桩目标，随后可直接执行 go_home 回桩充电。要切换到已有充电桩时，先用 list_docks 获取 dock_id，再执行 set_dock。回桩前需加载地图并保持定位正常。",
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {"type": "string", "enum": [
                        "list_docks", "register_dock", "set_dock", "delete_dock",
                        "clear_docks", "get_dock", "go_home", "cancel",
                    ], "description": "充电桩管理或回桩动作"},
                    "display_name": {"type": "string", "description": "注册时使用的充电桩名称，例如 main_dock"},
                    "dock_id": {"type": "string", "description": "充电桩 UUID，由 list_docks 返回，用于选定或删除该充电桩"},
                    "pose": {"type": "object", "description": "直接设定当前充电桩的 Pose3D（x,y,z,yaw,pitch,roll），必须属于当前地图坐标系"},
                    "back_to_landing": {"type": "boolean", "description": "上桩失败后是否回到对接前的上桩点"},
                    "charging_retry_count": {"type": "integer", "description": "上桩失败时的额外重试次数"},
                    "move_mode": {"type": "integer", "enum": [0, 2], "description": "回桩路径模式：0 为自由导航，2 为轨道优先（需底盘固件支持）"},
                },
                "required": ["action"],
                "x-completion": {
                    "actions": ["go_home"],
                    "timeout": 180,
                },
                "x-action-params": {
                    "list_docks": {"params": [], "description": "列出当前地图已注册的全部充电桩，返回名称、dock_id 与位姿；可据此选择或删除充电桩"},
                    "register_dock": {"params": ["display_name"], "description": "将机器人当前定位位姿保存为一个新充电桩，并自动设为当前回桩目标。执行前应让机器人停在实际充电桩的对接位置并确认定位正常；成功后可直接执行 go_home"},
                    "set_dock": {"params": ["dock_id", "pose"], "description": "设置本次及后续回桩使用的当前目标。填写 dock_id 时读取已注册充电桩的位姿；也可直接填写 pose。二者任选其一，位姿必须匹配当前地图"},
                    "delete_dock": {"params": ["dock_id"], "description": "删除 list_docks 返回的指定充电桩记录，不会移动机器人"},
                    "clear_docks": {"params": [], "description": "删除全部已注册充电桩记录，不会移动机器人，操作不可恢复"},
                    "get_dock": {"params": [], "description": "读取当前回桩目标的位姿；返回为空或 404 表示尚未设置"},
                    "go_home": {"params": [], "description": "在当前地图中导航到已设置的充电桩并尝试对接充电。需地图已加载、机器人定位正常且当前充电桩已设置"},
                    "cancel": {"params": [], "description": "取消当前回桩请求；机器人将停止等待该回桩动作完成"},
                },
                "x-hooks": {"on_interrupt_motion": {"action": "cancel"}},
            },
        }

    def start(self):
        pass

    def stop(self):
        with self._poll_lock:
            self._active_poll = None

    @staticmethod
    def _error(result: dict) -> bool:
        return isinstance(result, dict) and bool(result.get("error"))

    def dispatch(self, action: str, args: dict) -> dict:
        if action == "list_docks":
            return {"docks": self._slamtec.get_home_docks()}
        if action == "register_dock":
            name = str(args.get("display_name", "")).strip()
            if not name:
                return {"error": "display_name is required"}
            dock = self._slamtec.register_home_dock(name)
            if self._error(dock):
                return {"state": "error", "dock": dock}
            pose = dock.get("pose") if isinstance(dock, dict) else None
            if not isinstance(pose, dict):
                return {
                    "state": "registered_not_selected",
                    "dock": dock,
                    "error": "registered dock response does not contain pose",
                }
            selected = self._slamtec.set_home_pose(pose)
            if self._error(selected):
                return {
                    "state": "registered_not_selected",
                    "dock": dock,
                    "pose": pose,
                    "selection_result": selected,
                }
            return {
                "state": "registered_and_selected",
                "dock": dock,
                "pose": pose,
                "selection_result": selected,
            }
        if action == "set_dock":
            dock_id = str(args.get("dock_id", "")).strip()
            pose = args.get("pose")
            if dock_id:
                docks = self._slamtec.get_home_docks()
                entries = docks.get("raw", docks) if isinstance(docks, dict) else docks
                if isinstance(entries, list):
                    selected = next((d for d in entries if str(d.get("id")) == dock_id), None)
                    if selected and selected.get("pose"):
                        pose = selected["pose"]
            if not isinstance(pose, dict):
                return {"error": "dock_id or pose is required"}
            return {"api_result": self._slamtec.set_home_pose(pose), "pose": pose}
        if action == "delete_dock":
            dock_id = str(args.get("dock_id", "")).strip()
            if not dock_id:
                return {"error": "dock_id is required"}
            return {"api_result": self._slamtec.delete_home_dock(dock_id)}
        if action == "clear_docks":
            return {"api_result": self._slamtec.clear_home_docks()}
        if action == "get_dock":
            return {"pose": self._slamtec.get_home_pose()}
        if action == "go_home":
            result = self._slamtec.go_home(
                back_to_landing=args.get("back_to_landing"),
                charging_retry_count=args.get("charging_retry_count"),
                move_mode=args.get("move_mode"),
            )
            if self._error(result):
                return {"state": "error", "api_result": result}
            action_id = self._start_poll(action, result, {})
            response = {"state": "going_home", "api_result": result}
            if action_id:
                response["action_id"] = action_id
            return response
        if action == "cancel":
            result = self._slamtec.cancel_action()
            with self._poll_lock:
                self._active_poll = None
            return {"state": "cancelled", "api_result": result}
        if action in ("start", "info"):
            return {"state": "ready"}
        return {"error": f"unknown action: {action}"}

    def _start_poll(self, action: str, api_result: dict, context: dict) -> str:
        from uuid import uuid4
        action_id = f"home_{action}_{uuid4().hex[:8]}"
        with self._poll_lock:
            self._active_poll = action_id
        threading.Thread(target=self._poll_loop, args=(action_id, action, context), daemon=True).start()
        return action_id

    def _poll_loop(self, action_id: str, action: str, context: dict):
        t0 = time.time()
        last_pose = None
        last_move_time = t0
        while True:
            time.sleep(self._POLL_INTERVAL)
            with self._poll_lock:
                if self._active_poll != action_id:
                    return
            elapsed = time.time() - t0
            try:
                current = self._slamtec.get_nav_status()
            except Exception as exc:
                current = {"error": str(exc)}
            if current.get("error"):
                if elapsed > self._ACTION_TIMEOUT:
                    _acp_notify(action_id, "error", {"action": action, "error": current["error"], **context}, "home")
                    return
                continue

            state = current.get("action_state")
            result_code = current.get("result")
            if state == 4:
                if result_code == 0:
                    _acp_notify(action_id, "completed", {"action": action, "elapsed": round(elapsed, 1), **context}, "home")
                else:
                    _acp_notify(action_id, "error", {"action": action, "error": current.get("reason") or f"result_code={result_code}", "elapsed": round(elapsed, 1), **context}, "home")
                return
            if state == 3:
                if elapsed > self._ACTION_TIMEOUT:
                    _acp_notify(action_id, "error", {"action": action, "error": "timeout", "elapsed": self._ACTION_TIMEOUT, **context}, "home")
                    return
                continue
            if state is None or state == -1:
                # A successful action is reported as Done/result=0. Never infer
                # success merely because the chassis no longer exposes an action.
                if elapsed > self._MISSING_ACTION_TIMEOUT:
                    _acp_notify(action_id, "error", {"action": action, "error": "action_disappeared", "elapsed": round(elapsed, 1), **context}, "home")
                    return
            if action == "go_home":
                try:
                    pose = self._slamtec.get_pose()
                    if last_pose is None:
                        last_pose = pose
                    dx = pose.get("x", 0) - last_pose.get("x", 0)
                    dy = pose.get("y", 0) - last_pose.get("y", 0)
                    if dx * dx + dy * dy > 0.01:
                        last_pose = pose
                        last_move_time = time.time()
                    elif time.time() - last_move_time > self._STALL_TIMEOUT:
                        _acp_notify(action_id, "error", {"action": action, "error": "stall_timeout", "elapsed": round(elapsed, 1), **context}, "home")
                        return
                except Exception:
                    pass
            if elapsed > self._ACTION_TIMEOUT:
                _acp_notify(action_id, "error", {"action": action, "error": "timeout", "elapsed": self._ACTION_TIMEOUT, **context}, "home")
                return


# ══════════════════════════════════════════════════════════════════════════════
# ChatPlugin (actuator)
# ══════════════════════════════════════════════════════════════════════════════

class ChatPlugin:
    """语音交互开关"""

    def __init__(self, plugin_config: dict, namespace: str, ros2):
        self._ns = namespace
        self._ros2 = ros2
        self._pub_node = Node("tianyi2_chat_pub", context=ros2.ctx_tianyi)
        ros2.executor_tianyi.add_node(self._pub_node)
        self._publisher = None

    def get_tool(self) -> dict:
        return {
            "name": "chat",
            "type": "actuator",
            "description": "天轶2.0 语音交互模式 — 开启/关闭内置语音对话功能",
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {"type": "string", "enum": ["enable", "disable"],
                               "description": "开启或关闭"},
                },
                "required": ["action"],
                "x-action-params": {
                    "enable": {"params": [], "description": "开启语音交互"},
                    "disable": {"params": [], "description": "关闭语音交互"},
                },
            },
        }

    def start(self):
        self._publisher = self._pub_node.create_publisher(Bool, "/audio_chat/enable", _RELIABLE_QOS)
        print("[ChatPlugin] publisher created")

    def stop(self):
        pass

    def dispatch(self, action: str, args: dict) -> dict:
        if action in ("enable", "disable"):
            if self._publisher:
                msg = Bool()
                msg.data = (action == "enable")
                self._publisher.publish(msg)
                return {"state": action + "d"}
            return {"error": "publisher not initialized"}
        elif action in ("start", "info"):
            return {"state": "ready"}
        elif action == "stop":
            return {"state": "idle"}
        return {"error": f"unknown action: {action}"}


# ══════════════════════════════════════════════════════════════════════════════
# VoiceChatActuatorPlugin (actuator) — 卡名: voice_chat
# ══════════════════════════════════════════════════════════════════════════════

class VoiceChatActuatorPlugin:
    """语音对话开关 (/audio_chat/enable std_msgs/Bool)"""

    def __init__(self, plugin_config: dict, namespace: str, ros2):
        self._ns = namespace
        self._ros2 = ros2
        self._pub_node = Node("tianyi2_voice_chat_pub", context=ros2.ctx_tianyi)
        ros2.executor_tianyi.add_node(self._pub_node)
        self._publisher = None

    def get_tool(self) -> dict:
        return {
            "name": "voice_chat",
            "type": "actuator",
            "description": "天轶2.0 Pro 语音对话开关(enable/disable),HIGHLEVEL,topic /audio_chat/enable std_msgs/Bool。",
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {"type": "string", "enum": ["enable", "disable"],
                               "description": "开启/关闭语音对话"},
                },
                "required": ["action"],
                "x-action-params": {
                    "enable":  {"params": [], "description": "开启语音对话"},
                    "disable": {"params": [], "description": "关闭语音对话"},
                },
            },
        }

    def start(self):
        self._publisher = self._pub_node.create_publisher(Bool, "/audio_chat/enable", _RELIABLE_QOS)
        print("[VoiceChatActuatorPlugin] publisher created")

    def stop(self):
        pass

    def dispatch(self, action: str, args: dict) -> dict:
        if action in ("start", "info"):
            return {"state": "ready"}
        if action in ("enable", "disable"):
            if not self._publisher:
                return {"ok": False, "code": "PRECONDITION_FAILED",
                        "message": "publisher not initialized"}
            msg = Bool()
            msg.data = (action == "enable")
            self._publisher.publish(msg)
            return {
                "ok": True,
                "code": 0,
                "message": "",
                "action": action,
                "value": msg.data,
                "timestamp_ms": int(time.time() * 1000),
            }
        if action == "stop":
            return {"state": "idle"}
        return {"ok": False, "code": "INVALID_ARGUMENT",
                "message": f"unknown action: {action}"}


# ══════════════════════════════════════════════════════════════════════════════
# MotorStatePlugin (sensor) — 全身21电机状态按部位聚合 (2Hz)
# ══════════════════════════════════════════════════════════════════════════════

_MOTOR_ERROR_DESCRIPTIONS = {
    0: "ok",
    1: "over_current",
    2: "over_temperature",
    3: "communication_lost",
    4: "encoder_error",
    5: "over_voltage",
    6: "under_voltage",
    7: "motor_stall",
    8: "phase_error",
}


def _describe_motor_error(code: int) -> str:
    return _MOTOR_ERROR_DESCRIPTIONS.get(int(code), f"code#{code}")


# 关节 ID → 语义化名称 (对齐 bodyctrl_msgs/MotorName 枚举)
_MOTOR_IDX_TO_NAME = {
    1: "head_roll", 2: "head_pitch", 3: "head_yaw",
    11: "left_shoulder_roll", 12: "left_shoulder_pitch", 13: "left_shoulder_yaw",
    14: "left_elbow", 15: "left_elbow_flex", 16: "left_wrist_angle", 17: "left_wrist_rotate",
    21: "right_shoulder_roll", 22: "right_shoulder_pitch", 23: "right_shoulder_yaw",
    24: "right_elbow", 25: "right_elbow_flex", 26: "right_wrist_angle", 27: "right_wrist_rotate",
    31: "waist_yaw", 32: "waist_roll", 33: "waist_extra",
    51: "left_hip", 52: "left_knee", 53: "left_ankle",
    54: "left_foot_roll", 55: "left_foot_pitch", 56: "left_foot_yaw",
    61: "right_hip", 62: "right_knee", 63: "right_ankle",
    64: "right_foot_roll", 65: "right_foot_pitch", 66: "right_foot_yaw",
}


def _split_arm(motors: list) -> tuple[list, list]:
    """按关节 ID 拆左右臂: 11-17 左, 21-27 右。"""
    left, right = [], []
    for m in motors:
        idx = m.get("idx", 0)
        if 11 <= idx <= 17:
            left.append(m)
        elif 21 <= idx <= 27:
            right.append(m)
    return left, right


class MotorStatePlugin:
    """天轶2.0 全身21电机状态 — 按部位聚合 (2Hz)。

    数据源 (domain 0):
      /head/status  → MotorStatusMsg (关节 1-3)
      /waist/status → MotorStatusMsg (关节 31-33)
      /arm/status   → MotorStatusMsg (关节 11-17 左 / 21-27 右)
      /leg/status   → MotorStatusMsg (关节 51-66)
    发布到 (domain 42): /{ns}/state/motors (std_msgs/String JSON)
    """

    def __init__(self, plugin_config: dict, namespace: str, ros2):
        self._ns = namespace
        self._ros2 = ros2
        self._topic = f"/{namespace}/state/motors"
        self._running = False
        self._lock = threading.Lock()
        self._latest = {"head": None, "waist": None, "arm": None, "leg": None}

        self._sub_node = Node("tianyi2_motors_sub", context=ros2.ctx_tianyi)
        ros2.executor_tianyi.add_node(self._sub_node)

        self._pub_node = Node("tianyi2_motors_pub", context=ros2.ctx_core)
        ros2.executor_core.add_node(self._pub_node)
        self._pub = self._pub_node.create_publisher(String, self._topic, _LOW_LAT_QOS)

    def get_tool(self) -> dict:
        return {
            "name": "motors",
            "type": "sensor",
            "multiInstance": False,
            "readOnly": True,
            "description": (
                "天轶2.0 全身21电机状态(按部位聚合, 2Hz)。"
                "部位: head(3DOF)/arm_left(7DOF)/arm_right(7DOF)/waist(2DOF)/leg(2DOF)。"
                "每关节: name=语义名, q=角度(rad), dq=速度(rad/s), current=电流(A), temp=温度(°C)。"
                "bodyctrl 不上报腰腿的 current/temp/dq → 标 unknown 或不出现。"
                "error=0 正常, 非0故障(此时额外输出 error_description)。"
            ),
            "inputSchema": {"type": "object", "properties": {}},
            "topic_out": [{"topic": self._topic, "format": "data/json"}],
        }

    def start(self):
        self._running = True
        try:
            from bodyctrl_msgs.msg import MotorStatusMsg
            topics = {
                "head": "/head/status",
                "waist": "/waist/status",
                "arm": "/arm/status",
                "leg": "/leg/status",
            }
            for key, topic in topics.items():
                self._sub_node.create_subscription(
                    MotorStatusMsg, topic,
                    lambda m, k=key: self._on_motor(k, m), _RELIABLE_QOS)
            print("[MotorStatePlugin] subscriptions created")
        except ImportError as e:
            print(f"[MotorStatePlugin] WARNING: import failed ({e}), stub mode")

        self._thread = threading.Thread(target=self._publish_loop, daemon=True)
        self._thread.start()
        print("[MotorStatePlugin] publish started")

    def stop(self):
        self._running = False

    def _on_motor(self, road: str, msg):
        try:
            status_list = getattr(msg, "status", [])
            motors = []
            for m in status_list:
                idx = int(getattr(m, "name", 0) or 0)
                err = int(getattr(m, "error", 0))
                pos_raw = float(getattr(m, "pos", 0))
                spd_raw = float(getattr(m, "speed", 0))
                cur_raw = float(getattr(m, "current", 0))
                tmp_raw = float(getattr(m, "temperature", 0))

                item = {
                    "idx": idx,
                    "name": _MOTOR_IDX_TO_NAME.get(idx, f"joint_{idx}"),
                    "q": round(pos_raw, 6),
                }
                if abs(spd_raw) > 0:
                    item["dq"] = round(spd_raw, 6)
                if abs(cur_raw) > 0:
                    item["current"] = round(cur_raw, 6)
                else:
                    item["current"] = "unknown"
                if tmp_raw > 0:
                    item["temp"] = tmp_raw
                else:
                    item["temp"] = "unknown"
                if err != 0:
                    item["error"] = err
                    item["error_description"] = _describe_motor_error(err)
                motors.append(item)
            with self._lock:
                self._latest[road] = motors
        except Exception as e:  # noqa: BLE001
            print(f"[MotorStatePlugin] callback error on {road}: {e}")

    @staticmethod
    def _part(joints, label: str = "") -> dict:
        block = {"count": len(joints) if joints else 0, "joints": joints or []}
        if label:
            block["label"] = label
        return block

    def _produce(self) -> dict | None:
        with self._lock:
            data = dict(self._latest)
        if not any(data.values()):
            return None
        arm_left, arm_right = _split_arm(data.get("arm") or [])
        return {
            "parts": {
                "head":      self._part(data.get("head"),   "head (3DOF)"),
                "arm_left":  self._part(arm_left,          "left arm (7DOF)"),
                "arm_right": self._part(arm_right,         "right arm (7DOF)"),
                "waist":     self._part(data.get("waist"), "waist (2DOF)"),
                "leg":       self._part(data.get("leg"),   "leg (2DOF)"),
            },
            "timestamp_ms": int(time.time() * 1000),
        }

    def _publish_loop(self):
        while self._running:
            time.sleep(0.5)  # 2Hz
            payload = self._produce()
            if payload is None:
                continue
            msg = String()
            msg.data = json.dumps(payload)
            self._pub.publish(msg)

    def dispatch(self, action: str, args: dict) -> dict:
        if action in ("read", "get_motors", "get_motor_state"):
            d = self._produce()
            if d is None:
                return {"state": "error", "error": "NO_FEEDBACK",
                        "message": "no fresh motor state"}
            return d
        if action in ("start", "stop", "info"):
            return {"state": "running" if self._running else "idle",
                    "topic_out": [{"topic": self._topic, "format": "data/json"}]}
        return {"state": "error", "error": "INVALID_ARGUMENT",
                "message": f"unknown action: {action}"}


# ══════════════════════════════════════════════════════════════════════════════
# HandStatePlugin (sensor) — Inspire 灵巧手状态 (10Hz), tool name="hand_state"
# 注意: 上游已有 HandPlugin (actuator, tool name="hand"), 故此处用 hand_state 避免冲突
# ══════════════════════════════════════════════════════════════════════════════

_HAND_FINGER_NAMES = {
    1: "pinky", 2: "ring", 3: "middle",
    4: "index", 5: "thumb_flex", 6: "thumb_rotate",
}
_HAND_FINGER_LABELS = {
    1: "pinky", 2: "ring", 3: "middle",
    4: "index", 5: "thumb flexion", 6: "thumb rotation",
}


def _hand_position_label(p: float) -> str:
    if p >= 0.95:
        return "fully_closed"
    if p >= 0.75:
        return "almost_closed"
    if p >= 0.25:
        return "half_closed"
    if p >= 0.05:
        return "almost_open"
    return "fully_open"


class HandStatePlugin:
    """天轶2.0 Pro Inspire 灵巧手状态 — 左右手各6指 (10Hz)。

    数据源 (domain 0):
      /inspire_hand/state/left_hand  → sensor_msgs/JointState
      /inspire_hand/state/right_hand → sensor_msgs/JointState
    发布到 (domain 42): /{ns}/state/hand
    """

    def __init__(self, plugin_config: dict, namespace: str, ros2):
        self._ns = namespace
        self._ros2 = ros2
        self._topic = f"/{namespace}/state/hand"
        self._running = False
        self._lock = threading.Lock()
        self._latest = {"left": None, "right": None}

        self._sub_node = Node("tianyi2_hand_sub", context=ros2.ctx_tianyi)
        ros2.executor_tianyi.add_node(self._sub_node)

        self._pub_node = Node("tianyi2_hand_pub2", context=ros2.ctx_core)
        ros2.executor_core.add_node(self._pub_node)
        self._pub = self._pub_node.create_publisher(String, self._topic, _LOW_LAT_QOS)

    def get_tool(self) -> dict:
        return {
            "name": "hand_state",
            "type": "sensor",
            "multiInstance": False,
            "readOnly": True,
            "description": (
                "Tianyi 2.0 Pro Inspire dexterous hand state (6 fingers per hand, 10Hz)."
                "Finger order: 1=pinky 2=ring 3=middle 4=index 5=thumb_flex 6=thumb_rotate."
                "position: 0=open 1=closed (normalized), effort: current (A), velocity: normalized speed."
                "Each finger has a position_label tag (fully_open/almost_open/half_closed/almost_closed/fully_closed)."
            ),
            "inputSchema": {"type": "object", "properties": {}},
            "topic_out": [{"topic": self._topic, "format": "data/json"}],
        }

    def start(self):
        self._running = True
        try:
            from sensor_msgs.msg import JointState
            topics = {
                "left": "/inspire_hand/state/left_hand",
                "right": "/inspire_hand/state/right_hand",
            }
            for key, topic in topics.items():
                self._sub_node.create_subscription(
                    JointState, topic,
                    lambda m, k=key: self._on_hand(k, m), _RELIABLE_QOS)
            print("[HandStatePlugin] subscriptions created")
        except ImportError as e:
            print(f"[HandStatePlugin] WARNING: import failed ({e}), stub mode")

        self._thread = threading.Thread(target=self._publish_loop, daemon=True)
        self._thread.start()
        print("[HandStatePlugin] publish started")

    def stop(self):
        self._running = False

    def _on_hand(self, side: str, msg):
        try:
            names = getattr(msg, "name", [])
            positions = getattr(msg, "position", [])
            velocities = getattr(msg, "velocity", [])
            efforts = getattr(msg, "effort", [])
            fingers = []
            for i, raw_name in enumerate(names):
                try:
                    fid = int(raw_name)
                except (TypeError, ValueError):
                    fid = i + 1
                item = {
                    "id": fid,
                    "name": _HAND_FINGER_NAMES.get(fid, f"finger_{fid}"),
                    "label": _HAND_FINGER_LABELS.get(fid, f"finger_{fid}"),
                    "position": round(float(positions[i]), 4) if i < len(positions) else 0.0,
                    "velocity": round(float(velocities[i]), 4) if i < len(velocities) else 0.0,
                    "effort": round(float(efforts[i]), 4) if i < len(efforts) else 0.0,
                }
                item["position_label"] = _hand_position_label(item["position"])
                fingers.append(item)
            with self._lock:
                self._latest[side] = fingers
        except Exception as e:  # noqa: BLE001
            print(f"[HandStatePlugin] callback error on {side}: {e}")

    @staticmethod
    def _hand_block(fingers) -> dict:
        if not fingers:
            return {"count": 0, "fingers": []}
        return {"count": len(fingers), "fingers": fingers}

    def _produce(self) -> dict | None:
        with self._lock:
            data = dict(self._latest)
        if not any(data.values()):
            return None
        return {
            "hands": {
                "left":  self._hand_block(data.get("left")),
                "right": self._hand_block(data.get("right")),
            },
            "timestamp_ms": int(time.time() * 1000),
        }

    def _publish_loop(self):
        while self._running:
            time.sleep(0.1)  # 10Hz
            payload = self._produce()
            if payload is None:
                continue
            msg = String()
            msg.data = json.dumps(payload)
            self._pub.publish(msg)

    def dispatch(self, action: str, args: dict) -> dict:
        if action in ("read", "get_hand_state"):
            d = self._produce()
            if d is None:
                return {"state": "error", "error": "NO_FEEDBACK",
                        "message": "no fresh hand state"}
            return d
        if action in ("start", "stop", "info"):
            return {"state": "running" if self._running else "idle",
                    "topic_out": [{"topic": self._topic, "format": "data/json"}]}
        return {"state": "error", "error": "INVALID_ARGUMENT",
                "message": f"unknown action: {action}"}


# ══════════════════════════════════════════════════════════════════════════════
# RemoteStatePlugin (sensor) — 遥控器SBUS事件 (5Hz)
# ══════════════════════════════════════════════════════════════════════════════

_REMOTE_KEY_NAMES = {
    1: ("a_up", 1), 2: ("a_down", 2),
    3: ("b_up", 3), 4: ("b_down", 4),
    5: ("c_up", 5), 6: ("c_down", 6),
    7: ("d_up", 7), 8: ("d_down", 8),
    9: ("e_up", 9), 10: ("e_mid", 10), 11: ("e_down", 11),
    12: ("f_up", 12), 13: ("f_mid", 13), 14: ("f_down", 14),
    15: ("g_left", 15), 16: ("g_mid", 16), 17: ("g_right", 17),
    18: ("h_left", 18), 19: ("h_mid", 19), 20: ("h_right", 20),
}

_REMOTE_KEY_LABELS = {
    "a_up": "A键按下", "a_down": "A键回弹",
    "b_up": "B键按下", "b_down": "B键回弹",
    "c_up": "C键按下", "c_down": "C键回弹",
    "d_up": "D键按下", "d_down": "D键回弹",
    "e_up": "E键上拨", "e_mid": "E键中位", "e_down": "E键下拨",
    "f_up": "F键上拨", "f_mid": "F键中位", "f_down": "F键下拨",
    "g_left": "G键左拨", "g_mid": "G键中位", "g_right": "G键右拨",
    "h_left": "H键左拨", "h_mid": "H键中位", "h_right": "H键右拨",
}


def _stick_pos(x: float, y: float) -> str:
    """摇杆方向标签 (|x|+|y| < 0.1 视为居中)。"""
    if abs(x) < 0.1 and abs(y) < 0.1:
        return "center"
    if abs(x) >= abs(y):
        return "right" if x > 0 else "left"
    return "forward" if y > 0 else "back"


class RemoteStatePlugin:
    """天轶2.0 遥控器SBUS事件 — 8按键 + 2摇杆 (5Hz)。

    数据源 (domain 0):
      /sbus_data       → sensor_msgs/Joy (12 轴摇杆)
      /sbus_data/event → bodyctrl_msgs/SbusData (按键事件)
    发布到 (domain 42): /{ns}/state/remote_event
    """

    def __init__(self, plugin_config: dict, namespace: str, ros2):
        self._ns = namespace
        self._ros2 = ros2
        self._topic = f"/{namespace}/state/remote_event"
        self._running = False
        self._lock = threading.Lock()
        self._latest_event = None
        self._prev_key_new = 0
        self._joy = {"x1": 0.0, "y1": 0.0, "x2": 0.0, "y2": 0.0}
        self._buttons = {k: 0 for k in ("a", "b", "c", "d", "e", "f", "g", "h")}

        self._sub_node = Node("tianyi2_remote_sub", context=ros2.ctx_tianyi)
        ros2.executor_tianyi.add_node(self._sub_node)

        self._pub_node = Node("tianyi2_remote_pub", context=ros2.ctx_core)
        ros2.executor_core.add_node(self._pub_node)
        self._pub = self._pub_node.create_publisher(String, self._topic, _LOW_LAT_QOS)

    def get_tool(self) -> dict:
        return {
            "name": "remote_event",
            "type": "sensor",
            "multiInstance": False,
            "readOnly": True,
            "description": (
                "天轶2.0 遥控器SBUS事件(43Hz 采样, 5Hz 心跳发布)。"
                "8 按键 A-H + 2 摇杆(左/右, 归一化 -1~+1)。"
                "buttons 字段每帧更新当前按键状态(button_a~button_h 0/1)。"
                "按键边沿事件在 event 字段附 button(如 a_up)+ button_id(1-20)+ label(中文); 摇杆在 joystick 字段。"
                "遥控器静止时 buttons 全 0, joystick 全 0, event 不出现(正常 idle 态)。"
            ),
            "inputSchema": {"type": "object", "properties": {}},
            "topic_out": [{"topic": self._topic, "format": "data/json"}],
        }

    def start(self):
        self._running = True
        try:
            from sensor_msgs.msg import Joy
            from bodyctrl_msgs.msg import SbusData
            self._sub_node.create_subscription(Joy, "/sbus_data", self._on_joy, _RELIABLE_QOS)
            self._sub_node.create_subscription(SbusData, "/sbus_data/event", self._on_event, _RELIABLE_QOS)
            print("[RemoteStatePlugin] subscriptions created")
        except ImportError as e:
            print(f"[RemoteStatePlugin] WARNING: import failed ({e}), stub mode")

        self._thread = threading.Thread(target=self._publish_loop, daemon=True)
        self._thread.start()
        print("[RemoteStatePlugin] publish started")

    def stop(self):
        self._running = False

    def _on_joy(self, msg):
        try:
            axes = list(getattr(msg, "axes", []))
            def _g(i):
                return round(float(axes[i]), 4) if len(axes) > i else 0.0
            with self._lock:
                self._joy = {"x1": _g(0), "y1": _g(1), "x2": _g(2), "y2": _g(3)}
        except Exception as e:  # noqa: BLE001
            print(f"[RemoteStatePlugin] joy callback error: {e}")

    def _on_event(self, msg):
        try:
            key_new = int(getattr(msg, "key_event_new", 0))
            with self._lock:
                for k in ("a", "b", "c", "d", "e", "f", "g", "h"):
                    self._buttons[k] = int(getattr(msg, f"button_{k}", 0))
            if key_new == self._prev_key_new or key_new == 0:
                return
            name_id = _REMOTE_KEY_NAMES.get(key_new)
            if not name_id:
                return
            button_name, button_id = name_id
            evt = {
                "event": "button",
                "button": button_name,
                "button_id": button_id,
                "label": _REMOTE_KEY_LABELS.get(button_name, button_name),
                "timestamp_ms": int(time.time() * 1000),
            }
            with self._lock:
                self._latest_event = evt
            self._prev_key_new = key_new
        except Exception as e:  # noqa: BLE001
            print(f"[RemoteStatePlugin] event callback error: {e}")

    def _produce(self) -> dict:
        with self._lock:
            evt = self._latest_event
            self._latest_event = None
            joy = dict(self._joy)
            btns = dict(self._buttons)
        out = {
            "state": "idle" if evt is None else "active",
            "joystick": {
                "left":  {"x": joy["x1"], "y": joy["y1"], "position": _stick_pos(joy["x1"], joy["y1"])},
                "right": {"x": joy["x2"], "y": joy["y2"], "position": _stick_pos(joy["x2"], joy["y2"])},
            },
            "buttons": btns,
            "timestamp_ms": int(time.time() * 1000),
        }
        if evt is not None:
            out["event"] = evt
        return out

    def _publish_loop(self):
        while self._running:
            time.sleep(0.2)  # 5Hz
            payload = self._produce()
            msg = String()
            msg.data = json.dumps(payload)
            self._pub.publish(msg)

    def dispatch(self, action: str, args: dict) -> dict:
        if action in ("read", "get_remote_event", "get_remote"):
            d = self._produce()
            if d is None:
                return {"state": "error", "error": "NO_FEEDBACK",
                        "message": "no fresh remote state"}
            return d
        if action in ("start", "stop", "info"):
            return {"state": "running" if self._running else "idle",
                    "topic_out": [{"topic": self._topic, "format": "data/json"}]}
        return {"state": "error", "error": "INVALID_ARGUMENT",
                "message": f"unknown action: {action}"}
class HealthCheckPlugin:
    """全身体检卡 — 底盘安全 + 身体电机/灵巧手/急停 + 电源 + 手部 + IMU + 自检状态
    综合诊断结论 + 可行建议。
    bodyctrl_msgs 不可用时身体部分自动降级为 unavailable。"""

    @staticmethod
    def _default_config() -> dict:
        return {
            "motor_status_topics": ["/head/status", "/arm/status", "/waist/status", "/leg/status"],
            "hand_error_topics": {
                "left": "/inspire_hand/error/left_hand",
                "right": "/inspire_hand/error/right_hand",
            },
            "hand_state_topics": {
                "left": "/inspire_hand/state/left_hand",
                "right": "/inspire_hand/state/right_hand",
            },
            "estop_topic": "/power/board/key_status",
            "power_board_topic": "/power/board/status",
            "imu_accel_topic": "/ob_camera_head/accel/sample",
            "imu_gyro_topic": "/ob_camera_head/gyro/sample",
            "self_check_topic": "/bodycontrol_state",
        }

    def __init__(self, plugin_config: dict, namespace: str, ros2, slamtec_client):
        cfg = self._default_config()
        cfg.update(plugin_config)
        self._ns = namespace
        self._ros2 = ros2
        self._slamtec = slamtec_client
        self._running = False

        # 身体故障源配置
        self._motor_topics = cfg["motor_status_topics"]
        self._hand_topics = cfg["hand_error_topics"]
        self._hand_state_topics = cfg["hand_state_topics"]
        self._estop_topic = cfg["estop_topic"]
        self._power_board_topic = cfg["power_board_topic"]
        self._imu_accel_topic = cfg["imu_accel_topic"]
        self._imu_gyro_topic = cfg["imu_gyro_topic"]
        self._self_check_topic = cfg["self_check_topic"]

        # 状态
        self._chassis_data = None
        self._body_faults = {}
        self._body_sources = {}
        self._body_available = False
        self._last_update_ms = None
        self._lock = threading.Lock()

        # 新增数据源状态
        self._power_board_data = None       # PowerStatus
        self._power_board_available = False
        self._hand_state_data = {"left": None, "right": None}  # JointState
        self._hand_state_available = False
        self._imu_data = {"accel": None, "gyro": None}  # Imu msg
        self._imu_available = False
        self._self_check_state = None       # NodeState
        self._self_check_available = False

        # 自检状态判定:
        #   开始: bodycontrol_state 1→0 (proc_manager 触发自检的瞬间, 同时禁用 TTS)
        #   完成: 短促提示音 (duration < 2000ms)
        self._self_check_started = False
        self._self_check_completed = False
        # 短提示音判定 (自检完成)
        self._short_prompt_threshold_ms = 2.0    # < 2.0s 视为短促提示音 (duration 单位是秒)

        # 音频进度 duration 跟踪 (sid -> duration)
        self._audio_event_available = False
        self._sid_duration = {}
        self._max_sid_track = 20

        # 订阅节点 (domain 0) — 接收身体故障
        self._sub_node = Node("tianyi2_health_check_sub", context=ros2.ctx_tianyi)
        ros2.executor_tianyi.add_node(self._sub_node)

    def get_tool(self) -> dict:
        return {
            "name": "health_check",
            "type": "actuator",
            "description": "天轶2.0 全身体检：汇总底盘、电机、手部、电源、急停、IMU、自检等所有子系统状态，给出故障原因和操作建议。应在以下场景调用：①Agent Core 发现机器人无法正常移动或操作（可能被急停锁住，也可能出现电机/电源故障），需要排查根因时；②用户说\"检查一下机器人\"\"体检\"\"状态怎么样\"\"有没有故障\"等类似话语时。急停激活时会提示用户拔掉急停按钮并按遥控器A键自检。",
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {"type": "string", "enum": ["summary"],
                               "default": "summary",
                               "description": "summary=返回可操作判定+各子系统一句话概括+建议"},
                },
                "required": ["action"],
                "x-action-params": {
                    "summary": {"params": [], "description": "返回: 自检状态/急停/各子系统就绪/可否操作及原因+建议"},
                },
            },
        }

    # ── start / stop ──────────────────────────────────────────────────────

    def start(self):
        self._running = True

        # 底盘采集线程 (HTTP, 5Hz)
        threading.Thread(target=self._chassis_poll_loop, daemon=True).start()

        # 身体 ROS2 订阅 (失败则降级)
        try:
            from bodyctrl_msgs.msg import MotorStatusMsg, PowerBoardKeyStatus
            for src_topic in self._motor_topics:
                self._sub_node.create_subscription(
                    MotorStatusMsg, src_topic,
                    lambda msg, t=src_topic: self._on_motor_status(msg, t),
                    _RELIABLE_QOS)
            for side, src_topic in self._hand_topics.items():
                self._sub_node.create_subscription(
                    UInt32MultiArray, src_topic,
                    lambda msg, h=side, t=src_topic: self._on_hand_error(msg, h, t),
                    _RELIABLE_QOS)
            self._sub_node.create_subscription(
                PowerBoardKeyStatus, self._estop_topic,
                self._on_estop, _RELIABLE_QOS)
            self._body_available = True
            print("[HealthCheckPlugin] body subscriptions created")
        except ImportError as e:
            print(f"[HealthCheckPlugin] bodyctrl_msgs not available ({e}), body disabled")

        # 电源板订阅 (PowerStatus)
        try:
            from bodyctrl_msgs.msg import PowerStatus
            self._sub_node.create_subscription(
                PowerStatus, self._power_board_topic,
                self._on_power_board, _RELIABLE_QOS)
            self._power_board_available = True
            print("[HealthCheckPlugin] power board subscription created")
        except ImportError:
            print("[HealthCheckPlugin] PowerStatus not available, power board disabled")

        # 手部状态订阅 (JointState) — 检测手部是否在线
        try:
            from sensor_msgs.msg import JointState
            for side, src_topic in self._hand_state_topics.items():
                self._sub_node.create_subscription(
                    JointState, src_topic,
                    lambda msg, s=side: self._on_hand_state(msg, s),
                    _RELIABLE_QOS)
            self._hand_state_available = True
            print("[HealthCheckPlugin] hand state subscriptions created")
        except ImportError:
            print("[HealthCheckPlugin] JointState not available, hand state disabled")

        # IMU 订阅
        try:
            from sensor_msgs.msg import Imu as RosImu
            self._sub_node.create_subscription(
                RosImu, self._imu_accel_topic,
                lambda msg: self._on_imu("accel", msg), _RELIABLE_QOS)
            self._sub_node.create_subscription(
                RosImu, self._imu_gyro_topic,
                lambda msg: self._on_imu("gyro", msg), _RELIABLE_QOS)
            self._imu_available = True
            print("[HealthCheckPlugin] IMU subscriptions created")
        except ImportError:
            print("[HealthCheckPlugin] sensor_msgs.Imu not available, IMU disabled")

        # 自检状态订阅 (NodeState)
        try:
            from bodyctrl_msgs.msg import NodeState
            self._sub_node.create_subscription(
                NodeState, self._self_check_topic,
                self._on_self_check, _RELIABLE_QOS)
            self._self_check_available = True
            print("[HealthCheckPlugin] self check subscription created")
        except ImportError:
            print("[HealthCheckPlugin] NodeState not available, self check disabled")

        # 音频进度订阅 (短促提示音 → 自检完成)
        try:
            from lyre_msgs.msg import PlayProgress
            self._sub_node.create_subscription(
                PlayProgress, "/audio_play/progress",
                self._on_play_progress, _LOW_LAT_QOS)
            self._audio_event_available = True
            print("[HealthCheckPlugin] audio progress subscription created")
        except ImportError:
            print("[HealthCheckPlugin] PlayProgress not available, audio progress disabled")

        # 完整数据发布线程 (1Hz → topic)
        print("[HealthCheckPlugin] started")

    def stop(self):
        self._running = False

    # ── 底盘 HTTP 轮询 (5Hz) ──────────────────────────────────────────────

    def _chassis_poll_loop(self):
        while self._running:
            try:
                data = self._slamtec.get_safety_status()
                if data:
                    with self._lock:
                        self._chassis_data = data
                        self._last_update_ms = int(time.time() * 1000)
            except Exception:
                pass
            time.sleep(0.2)  # 5Hz

    # ── 身体 ROS2 回调 ────────────────────────────────────────────────────

    def _on_motor_status(self, msg, source_topic: str):
        now_ms = int(time.time() * 1000)
        with self._lock:
            for motor in msg.status:
                motor_id = int(motor.name)
                fault_id = f"motor:{motor_id}"
                if int(motor.error) == 0:
                    self._body_faults.pop(fault_id, None)
                    continue
                self._body_faults[fault_id] = {
                    "fault_id": fault_id,
                    "category": "motor",
                    "motor_id": motor_id,
                    "component": _ALL_JOINTS.get(motor_id, f"motor_{motor_id}"),
                    "error_code": int(motor.error),
                    "error_desc": _MOTOR_ERROR_DESCRIPTIONS.get(int(motor.error), "unknown"),
                    "severity": "error",
                    "source_topic": source_topic,
                }
            self._body_sources[source_topic] = now_ms
            self._last_update_ms = now_ms

    def _on_hand_error(self, msg, side: str, source_topic: str):
        now_ms = int(time.time() * 1000)
        prefix = f"hand:{side}:"
        with self._lock:
            for key in [k for k in self._body_faults if k.startswith(prefix)]:
                self._body_faults.pop(key, None)
            for idx, code in enumerate(msg.data, start=1):
                code = int(code)
                if code == 0:
                    continue
                fid = f"{prefix}{idx}"
                self._body_faults[fid] = {
                    "fault_id": fid,
                    "category": "hand",
                    "component": f"{side}_hand_error_channel_{idx}",
                    "error_code": code,
                    "severity": "error",
                    "source_topic": source_topic,
                }
            self._body_sources[source_topic] = now_ms
            self._last_update_ms = now_ms

    def _on_estop(self, msg):
        now_ms = int(time.time() * 1000)
        states = {
            "physical": bool(msg.is_estop.data),
            "remote": bool(msg.is_remote_estop.data),
        }
        with self._lock:
            for kind, active in states.items():
                fid = f"estop:{kind}"
                if not active:
                    self._body_faults.pop(fid, None)
                    continue
                self._body_faults[fid] = {
                    "fault_id": fid,
                    "category": "estop",
                    "component": f"{kind}_estop",
                    "error_code": 1,
                    "severity": "fatal",
                    "source_topic": self._estop_topic,
                }
            self._body_sources[self._estop_topic] = now_ms
            self._last_update_ms = now_ms

    # ── 新增回调: 电源板 / 手部状态 / IMU / 自检 ──────────────────────────

    def _on_power_board(self, msg):
        now_ms = int(time.time() * 1000)
        with self._lock:
            self._power_board_data = msg
            self._last_update_ms = now_ms

    def _on_hand_state(self, msg, side: str):
        now_ms = int(time.time() * 1000)
        with self._lock:
            self._hand_state_data[side] = msg
            self._last_update_ms = now_ms

    def _on_imu(self, kind: str, msg):
        now_ms = int(time.time() * 1000)
        with self._lock:
            self._imu_data[kind] = msg
            self._last_update_ms = now_ms

    def _on_self_check(self, msg):
        """bodycontrol_state 回调。
        state 1→0: 自检触发 → 置 started, 禁用 TTS 防止音频误触发完成判定;
        state 0→1: 服务恢复就绪，不代表完成。"""
        now_ms = int(time.time() * 1000)
        with self._lock:
            prev_state = self._self_check_state.state if self._self_check_state is not None else None
            self._self_check_state = msg
            self._last_update_ms = now_ms
            # 自检开始: Running(1) → Initing(0)
            if prev_state == 1 and msg.state == 0:
                self._self_check_started = True
                self._self_check_completed = False
                print(f"[HealthCheckPlugin] self-check triggered (state 1→0)")

    def _on_play_progress(self, msg):
        """订阅 /audio_play/progress, 短促提示音 (duration < 2.0s) → 自检完成."""
        with self._lock:
            if msg.sid:
                self._sid_duration[msg.sid] = msg.duration
                if len(self._sid_duration) > self._max_sid_track:
                    keys = sorted(self._sid_duration.keys())
                    for k in keys[:len(keys) - self._max_sid_track]:
                        self._sid_duration.pop(k, None)
                if (self._self_check_started and not self._self_check_completed
                        and msg.duration is not None
                        and msg.duration < self._short_prompt_threshold_ms):
                    self._self_check_completed = True
                    self._self_check_started = False
                    print(f"[HealthCheckPlugin] self-check completed by short prompt "
                          f"(sid={msg.sid}, duration={msg.duration}ms)")

    # ── 按需查询执行器 ─────────────────────────────────────────────────────

    def _build_summary_locked(self) -> dict:
        """生成全身体检摘要，每次 dispatch 调用时读取最新快照。"""

        now_ms = int(time.time() * 1000)
        issues = []
        advice = []

        # ── 底盘 ──
        chassis_available = self._chassis_data is not None
        chassis_healthy = chassis_available and not self._chassis_data.get("has_error") \
                          and not self._chassis_data.get("has_fatal")
        chassis_lines = []
        if not chassis_available:
            chassis_lines.append("离线")
            issues.append("底盘通信离线，无法获取安全状态")
        elif not chassis_healthy:
            chassis_lines.append("异常")
            issues.append("底盘系统异常")
        else:
            chassis_lines.append("正常")
        if chassis_available:
            d = self._chassis_data
            if d.get("emergency_stop"):
                chassis_lines.append("急停!")
                issues.append("底盘急停已触发")
                advice.append("检查急停按钮状态，确认安全后解除急停")
            if d.get("lidar_disconnected"):
                chassis_lines.append("雷达离线")
                issues.append("激光雷达离线")
            if d.get("cliff"):
                issues.append("检测到跌落风险")
                advice.append("立即停止移动，检查地面环境")
            if d.get("collision"):
                issues.append("检测到碰撞")
                advice.append("检查碰撞方向，排查周围障碍物")

        # ── 身体电机/急停 ──
        body_faults = sorted(self._body_faults.values(), key=lambda f: f["fault_id"])
        body_lines = []
        if not self._body_available:
            body_lines.append("离线")
            issues.append("身体电机/急停数据无法获取 (bodyctrl_msgs 未导入)")
        elif not body_faults:
            body_lines.append("正常")
        else:
            for f in body_faults:
                line = f"{f['component']}({f.get('error_desc', f['error_code'])})"
                body_lines.append(line)
                issues.append(f"故障: {line}")
                if f.get("severity") == "fatal":
                    advice.append(f"严重故障 {f['component']}，请立即停止操作并检查硬件")

        # ── 电源板 ──
        power_available = self._power_board_available and self._power_board_data is not None
        power_lines = []
        if not self._power_board_available:
            power_lines.append("离线")
            issues.append("电源板数据不可用 (PowerStatus 未导入)")
        elif not power_available:
            power_lines.append("无数据")
        else:
            p = self._power_board_data
            # 检查各部位温度
            temps = {
                "腰部": p.waist_temp, "臂A": p.arm_a_temp, "臂B": p.arm_b_temp,
                "腿A": p.leg_a_temp, "腿B": p.leg_b_temp,
            }
            hot_zones = [f"{k}({v:.0f}°C)" for k, v in temps.items() if v > 75]
            warm_zones = [f"{k}({v:.0f}°C)" for k, v in temps.items() if 65 < v <= 75]
            if hot_zones:
                power_lines.append(f"过热:{','.join(hot_zones)}")
                issues.append(f"电源板MOS过热: {', '.join(hot_zones)}")
                advice.append("立即停止运行，检查散热及风扇状态")
            elif warm_zones:
                power_lines.append(f"温升:{','.join(warm_zones)}")
                issues.append(f"电源板MOS温度偏高: {', '.join(warm_zones)}")
            else:
                power_lines.append("温度正常")
            # 电池
            if p.battery_power < 10:
                power_lines.append(f"电量极低({p.battery_power:.0f}%)")
                issues.append(f"电池电量极低 ({p.battery_power:.0f}%)，随时可能断电")
                advice.append("立即充电，避免在低电量下操作运动关节")
            elif p.battery_power < 25:
                power_lines.append(f"电量偏低({p.battery_power:.0f}%)")
                issues.append(f"电池电量偏低 ({p.battery_power:.0f}%)")
            else:
                power_lines.append(f"电量({p.battery_power:.0f}%)")
            # 电压异常
            if p.bus_volt == 0:
                power_lines.append("母线电压异常")
                issues.append("母线电压为0V，电源板可能未正常工作")

        # ── 手部状态 ──
        hand_available = self._hand_state_available
        hand_lines = []
        if not hand_available:
            hand_lines.append("离线")
        else:
            for side in ("left", "right"):
                data = self._hand_state_data[side]
                if data is None:
                    hand_lines.append(f"{'左' if side=='left' else '右'}手无数据")
                    issues.append(f"{'左' if side=='left' else '右'}手传感器无数据")
                elif data.name and len(data.name) > 0:
                    hand_lines.append(f"{'左' if side=='left' else '右'}手在线")
                else:
                    hand_lines.append(f"{'左' if side=='left' else '右'}手无关节名")
                    issues.append(f"{'左' if side=='left' else '右'}手关节名称为空，可能未连接")

        # ── IMU ──
        imu_available = self._imu_available
        imu_lines = []
        if not imu_available:
            imu_lines.append("离线")
        else:
            accel_ok = self._imu_data["accel"] is not None
            gyro_ok = self._imu_data["gyro"] is not None
            if accel_ok and gyro_ok:
                imu_lines.append("在线")
            else:
                missing = []
                if not accel_ok:
                    missing.append("加速度")
                if not gyro_ok:
                    missing.append("角速度")
                imu_lines.append(f"部分缺失:{','.join(missing)}")
                issues.append(f"IMU部分数据缺失: {', '.join(missing)}")

        # ── 自检状态 (state 1→0=开始, 短提示音=完成) ──
        # 自检中 = 检测到开始信号 且 未检测到完成信号
        self_check_available = self._self_check_available and self._self_check_state is not None
        self_check_lines = []

        if self._self_check_started and not self._self_check_completed:
            self_check_lines.append("自检中")
            issues.append("自检进行中")
        else:
            self_check_lines.append("没有在自检")

        # ── 综合健康判定 ──
        has_issues = len(issues) > 0
        all_normal = (chassis_healthy and self._body_available and not body_faults
                      and power_available and not hot_zones and p.battery_power >= 25)
        healthy = all_normal and not has_issues

        # ── 人类可读总结 ──
        if healthy:
            summary_text = "机器人状态良好: 底盘正常, 身体关节无故障, 电源温度正常, 手部在线, IMU在线, 节点运行中。"
        else:
            snippet = "；".join(issues[:3])
            if len(issues) > 3:
                snippet += f" 等{len(issues)}个问题"
            summary_text = f"检测到异常: {snippet}。"
        if advice:
            summary_text += " " + " ".join(advice)

        return {
            "healthy": healthy,
            "summary_text": summary_text,
            "summary": ", ".join(
                ["底盘:" + ",".join(chassis_lines),
                 "身体:" + ",".join(body_lines),
                 "电源:" + ",".join(power_lines),
                 "手部:" + ",".join(hand_lines),
                 "IMU:" + ",".join(imu_lines),
                 "自检:" + ",".join(self_check_lines)]),
            "chassis": {
                "available": chassis_available,
                "healthy": chassis_healthy,
                "emergency_stop": self._chassis_data.get("emergency_stop", False) if chassis_available else None,
                "detail": ", ".join(chassis_lines),
            },
            "body": {
                "available": self._body_available,
                "fault_count": len(body_faults),
                "faults": body_faults if body_faults else [],
                "detail": ", ".join(body_lines),
            },
            "power_board": {
                "available": power_available,
                "detail": ", ".join(power_lines),
            },
            "hand_state": {
                "available": hand_available,
                "detail": ", ".join(hand_lines),
            },
            "imu": {
                "available": imu_available,
                "detail": ", ".join(imu_lines),
            },
            "self_check": {
                "available": self_check_available,
                "detail": ", ".join(self_check_lines),
            },
            "issues": issues,
            "advice": advice,
        }

    def dispatch(self, action: str, args: dict) -> dict:
        if action in ("start", "stop", "info"):
            return {"state": "running" if self._running else "idle"}
        with self._lock:
            full = self._build_summary_locked()
        return self._build_operability_summary(full)

    def _build_operability_summary(self, full: dict) -> dict:
        """从完整诊断数据中提取可操作判定 + 人话总结。"""

        body_data = full.get("body", {})
        chassis_data = full.get("chassis", {})
        power_board = full.get("power_board", {})
        hand_state = full.get("hand_state", {})
        imu_info = full.get("imu", {})
        sc_info = full.get("self_check", {})

        lines = []

        # ── 1. 急停 ──
        estop_active = chassis_data.get("emergency_stop", False)
        estop_faults = [f for f in body_data.get("faults", []) if f.get("category") == "estop"]
        estop_active = estop_active or bool(estop_faults)
        if estop_active:
            lines.append("⚠️  急停 — 已触发 (fatal)")
        else:
            lines.append("✅ 急停 — 未触发")

        # ── 2. 电机 ──
        motor_faults = [f for f in body_data.get("faults", []) if f.get("category") == "motor"]
        if motor_faults:
            by_desc = {}
            for f in motor_faults:
                d = f.get("error_desc", "unknown")
                by_desc[d] = by_desc.get(d, 0) + 1
            parts = [f"{c}个{cate}" for cate, c in by_desc.items()]
            lines.append(f"⚠️  电机 — {len(motor_faults)}项故障({', '.join(parts)})")
        elif body_data.get("fault_count", 0) == 0:
            lines.append("✅ 电机 — 正常")
        else:
            lines.append("⚠️  电机 — 无数据")

        # ── 3. 电源/电池 ──
        power_detail = power_board.get("detail", "未知")
        if "过热" in power_detail or "温升" in power_detail:
            lines.append(f"⚠️  电源 — {power_detail}")
        elif "电量极低" in power_detail:
            lines.append(f"⚠️  电池 — {power_detail}")
        elif "电量偏低" in power_detail:
            lines.append(f"⚠️  电池 — {power_detail}")
        else:
            lines.append(f"✅ 电源 — {power_detail}")

        # ── 4. 手部 ──
        hands_detail = hand_state.get("detail", "未知")
        if "离线" in hands_detail or "无数据" in hands_detail or "未连接" in hands_detail:
            lines.append(f"⚠️  手部 — {hands_detail}")
        else:
            lines.append(f"✅ 手部 — {hands_detail}")

        # ── 5. IMU ──
        imu_detail = imu_info.get("detail", "未知")
        if "离线" in imu_detail or "缺失" in imu_detail:
            lines.append(f"⚠️  IMU — {imu_detail}")
        else:
            lines.append(f"✅ IMU — {imu_detail}")

        # ── 6. 自检状态 (两态: 自检中 / 没有在自检) ──
        sc_detail = sc_info.get("detail", "未知")
        sc_label = sc_detail
        if sc_detail == "自检中":
            lines.append("🔄 自检 — 进行中")
        else:
            lines.append("✅ 自检 — 没有在自检")

        # ── 7. 底盘 ──
        chassis_detail = chassis_data.get("detail", "未知")
        if "离线" in chassis_detail or "异常" in chassis_detail:
            lines.append(f"⚠️  底盘 — {chassis_detail}")
        elif "急停" in chassis_detail:
            lines.append(f"⚠️  底盘 — {chassis_detail}")
        # 正常底盘不在主列表里重复显示, 省空间

        # ── 8. 判定 + 总结 ──
        issues = full.get("issues", [])
        advice_list = full.get("advice", [])

        fatal_items = [f for f in body_data.get("faults", []) if f.get("severity") == "fatal"]
        blocker_count = len(fatal_items) + (1 if estop_active else 0)
        if sc_detail == "自检中":
            blocker_count += 1

        if blocker_count > 0 or "电量极低" in power_detail:
            worst = [f.get("component", "未知") for f in fatal_items[:3]]
            worst_parts = worst + (["急停"] if estop_active and "急停" not in "_".join(worst) else [])
            worst_str = "、".join(worst_parts[:3]) if worst_parts else "未知"
            summary = f"发现{len(issues)}项问题, 最严重: {worst_str}, 禁止操作"
        else:
            summary = "状态良好, 可以操作"

        # 建议
        if estop_active:
            advice_list = ["请拔掉急停按钮，然后按下遥控器A键让机器人自检（自检完成后才能正常操作）"] + (advice_list or [])
        if advice_list:
            summary += "。建议: " + "；".join(advice_list[:2])

        return {
            "can_operate": blocker_count == 0,
            "issue_count": len(issues),
            "emergency_stop": estop_active,
            "subsystems": {
                "chassis": chassis_detail,
                "body": body_data.get("detail", "未知"),
                "power": power_detail,
                "hands": hands_detail,
                "imu": imu_detail,
                "node": sc_detail,
            },
            "issues": issues,
            "advice": advice_list,
            "summary_text": "\n".join(lines + [summary]),
        }


# ══════════════════════════════════════════════════════════════════════════════
# LaserScanPlugin (sensor)
# ══════════════════════════════════════════════════════════════════════════════

class LaserScanPlugin:
    """激光雷达原始数据 — 轮询 Slamtec 底盘 laserscan 端点 (5Hz)"""

    def __init__(self, plugin_config: dict, namespace: str, ros2, slamtec_client):
        self._ns = namespace
        self._ros2 = ros2
        self._slamtec = slamtec_client
        self._topic = f"/{namespace}/state/laser_scan"
        self._topic_viz = f"/{namespace}/state/laser_scan_viz"
        self._running = False
        self._latest_raw = None

        self._pub_node = Node("tianyi2_laserscan_pub", context=ros2.ctx_core)
        ros2.executor_core.add_node(self._pub_node)
        self._pub = self._pub_node.create_publisher(String, self._topic, _LOW_LAT_QOS)
        # 可视化图片发布
        try:
            from sensor_msgs.msg import CompressedImage
            self._pub_viz = self._pub_node.create_publisher(
                CompressedImage, self._topic_viz, _LOW_LAT_QOS)
        except ImportError:
            self._pub_viz = None

    def get_tool(self) -> dict:
        return {
            "name": "laser_scan",
            "type": "sensor",
            "description": "天轶2.0 激光雷达 — 5Hz 原始点云 + 1Hz 俯视点云图, 后台自动运行",
            "inputSchema": {
                "type": "object",
                "properties": {},
            },
            "topic_out": [{"topic": self._topic, "format": "data/json"},
                          {"topic": self._topic_viz, "format": "image/jpeg"}],
        }

    def start(self):
        self._running = True
        self._thread = threading.Thread(target=self._poll_loop, daemon=True)
        self._thread.start()
        print("[LaserScanPlugin] polling started")

    def stop(self):
        self._running = False

    def _poll_loop(self):
        tick = 0
        while self._running:
            try:
                data = self._slamtec.get_laser_scan()
                if data and "error" not in data:
                    # 始终发布完整原始数据到 topic
                    msg = String()
                    msg.data = json.dumps(data)
                    self._pub.publish(msg)

                    # 缓存点云供可视化
                    laser_points = data.get("laser_points", data.get("data", []))
                    self._latest_raw = laser_points
            except Exception:
                pass
            # 每 1 秒生成一张可视化图片发到 viz topic
            if self._pub_viz and self._latest_raw and tick % 5 == 0:
                self._publish_viz()
            tick += 1
            time.sleep(0.2)

    def _publish_viz(self):
        """生成俯视图 JPEG 发到 viz topic."""
        try:
            import cv2, numpy as np
            from sensor_msgs.msg import CompressedImage
        except ImportError:
            return
        pts = self._latest_raw
        if not pts:
            return
        size = 400
        scale = (size // 2) / 6.0
        img = np.full((size, size, 3), (20, 20, 20), dtype=np.uint8)
        for p in pts:
            if not p.get("valid"):
                continue
            dist = p["distance"]
            if dist > 6.0:
                continue
            angle = p["angle"]
            px = int(size // 2 + dist * np.sin(angle) * scale)
            py = int(size // 2 - dist * np.cos(angle) * scale)
            if 0 <= px < size and 0 <= py < size:
                ratio = min(dist / 3.0, 1.0)
                r = int(255 * (1.0 - ratio))
                b = int(255 * ratio)
                cv2.circle(img, (px, py), 1, (b, 0, r), -1)
        cv2.circle(img, (size // 2, size // 2), 6, (0, 255, 0), -1)
        cv2.line(img, (size // 2, size // 2), (size // 2, 10), (0, 255, 0), 1)
        for label, a_range, color in [
            ("F", (-0.52, 0.52), (255, 255, 0)),
            ("L", (1.04, 2.09), (0, 255, 255)),
            ("R", (-2.09, -1.04), (255, 0, 255)),
        ]:
            mid = (a_range[0] + a_range[1]) / 2
            lx = int(size // 2 + 3.5 * scale * np.sin(mid))
            ly = int(size // 2 - 3.5 * scale * np.cos(mid))
            cv2.putText(img, label, (lx - 8, ly + 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
        for r in [2, 4, 6]:
            cv2.circle(img, (size // 2, size // 2), int(r * scale), (50, 50, 50), 1)
            cv2.putText(img, f"{r}m", (size // 2 + 5, size // 2 - int(r * scale) + 12),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.35, (80, 80, 80), 1)
        _, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 80])
        viz_msg = CompressedImage()
        viz_msg.format = "jpeg"
        viz_msg.data = buf.tobytes()
        self._pub_viz.publish(viz_msg)

    def dispatch(self, action: str, args: dict) -> dict:
        return {"state": "running" if self._running else "idle",
                "topic_out": [{"topic": self._topic, "format": "data/json"},
                              {"topic": self._topic_viz, "format": "image/jpeg"}]}


# ══════════════════════════════════════════════════════════════════════════════
# ChassisRawPlugin (actuator)
# ══════════════════════════════════════════════════════════════════════════════

class ChassisRawPlugin:
    """底盘速度控制 — 固定速率 0.3 m/s, 200ms 刷新保持连续运动。
    duration=-1 持续运动。"""

    DIR_NAMES = ["forward", "backward", "right", "left"]
    _FIXED_V = 0.3          # m/s
    _FIXED_W = 1.0          # rad/s (≈ 57.3 °/s)

    def __init__(self, plugin_config: dict, namespace: str, ros2, slamtec_client):
        self._ns = namespace
        self._ros2 = ros2
        self._slamtec = slamtec_client
        self._max_vx = float(plugin_config.get("max_vx", 0.5))
        self._max_vyaw = float(plugin_config.get("max_vyaw", 1.0))
        self._max_duration = float(plugin_config.get("max_duration", 5.0))
        self._running = False
        self._gen = 0  # 世代计数器, 防止新旧线程并发

    def get_tool(self) -> dict:
        return {
            "name": "chassis_raw",
            "type": "actuator",
            "description": "天轶2.0 底盘速度控制 — 固定速率 0.3 m/s, 前进/后退/左转/右转, duration=-1 持续运动",
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {"type": "string",
                               "enum": ["move", "rotate", "brake"],
                               "description": "控制动作"},
                    "direction": {"type": "string",
                                  "enum": ["forward", "backward"],
                                  "description": "移动方向 (固定速率 0.3 m/s)"},
                    "rotation": {"type": "string",
                                 "enum": ["left", "right"],
                                 "description": "旋转方向"},
                    "angle": {"type": "number",
                              "description": "旋转角度(度), 负数为反向旋转, 编码器闭环精确到度"},
                    "duration": {"type": "number",
                                 "description": "持续时间(秒), -1=持续运动 (不填 angle 时生效)"},
                },
                "required": ["action"],
                "x-completion": {
                    "actions": ["move", "rotate"],
                    "timeout": 60
                },
                "x-action-params": {
                    "move":   {"params": ["direction", "duration"],
                               "description": "前进/后退, 固定速率 0.3 m/s, duration=-1 持续运动"},
                    "rotate": {"params": ["rotation", "angle", "duration"],
                               "description": "精确旋转: angle(度) 用 Slamtec RotateAction 闭环; duration 为持续旋转"},
                    "brake":  {"params": [],
                               "description": "立即停止移动"},
                },
            },
        }

    def start(self):
        print("[ChassisRawPlugin] started (Slamtec MoveByAction mode)")

    def stop(self):
        self._running = False
        try:
            self._slamtec.cancel_current_action()
        except Exception:
            pass
        print("[ChassisRawPlugin] stopped")

    def dispatch(self, action: str, args: dict) -> dict:
        if action == "move":
            direction_str = args.get("direction", "forward")
            if direction_str not in ("forward", "backward"):
                return {"error": "direction must be 'forward' or 'backward'"}
            d = 0 if direction_str == "forward" else 1
            try:
                dur = float(args.get("duration", 3.0))
            except (TypeError, ValueError):
                return {"error": "duration must be a number"}
            return self._start(d, dur)
        elif action == "rotate":
            rot = args.get("rotation", "left")
            if rot not in ("left", "right"):
                return {"error": "rotation must be 'left' or 'right'"}

            angle = args.get("angle")
            dur = args.get("duration")

            if angle is not None:
                # 精确旋转: 使用 Slamtec RotateAction (编码器闭环, 角度精确)
                # 负数角度 = 反向旋转: left -90° → 实际右转 90°
                try:
                    angle_deg = float(angle)
                except (TypeError, ValueError):
                    return {"error": "angle must be a number (degrees)"}
                angle_rad = math.radians(angle_deg)
                if rot == "right":
                    angle_rad = -angle_rad  # right 方向: +→顺时针(CW), -→逆时针(CCW)
                # left 方向: +→逆时针(CCW), -→顺时针(CW) — angle_rad 符号不变
                self._slamtec.rotate(angle_rad)
                return {"rotation": rot, "angle": angle_deg, "unit": "degree",
                        "rad": round(angle_rad, 4)}

            # 非精确模式: duration 或 -1 持续旋转 (fallback move_by)
            direction = 3 if rot == "left" else 2
            if dur is None:
                dur = 3.0
            try:
                dur = float(dur)
            except (TypeError, ValueError):
                return {"error": "duration must be a number"}
            return self._start(direction, dur)
        elif action == "brake":
            return self._do_stop()
        elif action in ("start", "info"):
            return {"state": "ready"}
        return {"error": f"unknown action: {action}"}

    def _start(self, direction: int, duration: float) -> dict:
        self._running = False
        self._gen += 1
        time.sleep(0.05)
        self._do_stop()
        time.sleep(0.05)

        continuous = (duration < 0)
        self._running = True
        gen = self._gen
        import uuid as _uuid
        action_id = f"move-{_uuid.uuid4().hex[:8]}"
        threading.Thread(
            target=self._move_loop, args=(direction, duration, continuous, gen, action_id), daemon=True
        ).start()

        return {"direction": self.DIR_NAMES[direction],
                "duration": duration, "continuous": continuous,
                "action_id": action_id,
                "speed": "0.3 m/s (fixed)" if direction < 2 else "1.0 rad/s (fixed)"}

    def _move_loop(self, direction: int, total_duration: float, continuous: bool, gen: int, action_id: str = ''):
        """运动控制 — 非持续模式发单次精确定时 move_by; 持续模式每 200ms 刷新。"""
        if not continuous:
            # 单次精确运动: move_by duration 单位是毫秒
            try:
                self._slamtec.move_by(direction, int(total_duration * 1000))
            except Exception as e:
                print(f"[ChassisRawPlugin] move_by error: {e}")
            time.sleep(total_duration + 0.3)  # 等运动完成+缓冲
            if self._running and self._gen == gen:
                self._do_stop()
            # ACP callback
            if action_id:
                self._acp_move_callback(action_id, "completed", total_duration)
            return

        # 持续模式: 每 200ms 刷新 300ms 运动指令保持连续
        step = 0.2
        while self._running and self._gen == gen:
            try:
                self._slamtec.move_by(direction, 300)
            except Exception as e:
                print(f"[ChassisRawPlugin] move_by error: {e}")
                break
            time.sleep(step)
        if self._running and self._gen == gen:
            self._do_stop()
        # ACP callback for continuous mode (stopped by brake)
        if action_id:
            self._acp_move_callback(action_id, "completed", -1)

    def _acp_move_callback(self, action_id: str, status: str, duration: float):
        """POST movement completion to Agent Core."""
        try:
            import urllib.request as _urllib
            import ssl as _ssl
            import os as _os
            agent_core_url = _os.environ.get("AGENT_CORE_URL", "https://localhost:15678")
            ctx = _ssl.create_default_context()
            ctx.check_hostname = False
            ctx.verify_mode = _ssl.CERT_NONE
            payload = json.dumps({"action_id": action_id, "status": status,
                                  "result": {"duration": duration}}).encode()
            req = _urllib.Request(f"{agent_core_url}/api/acp/complete",
                                 data=payload, headers={"Content-Type": "application/json"},
                                 method="POST")
            _urllib.urlopen(req, timeout=3, context=ctx)
            print(f"[ChassisRawPlugin] ACP complete: {action_id} ({status})")
        except Exception as e:
            print(f"[ChassisRawPlugin] ACP callback failed: {e}")

    def _do_stop(self) -> dict:
        self._running = False
        try:
            self._slamtec.cancel_current_action()
        except Exception:
            pass
        return {"vx": 0.0, "vyaw": 0.0, "state": "stopped"}

    @staticmethod
    def _clamp(value: float, low: float, high: float) -> float:
        return max(low, min(high, value))
