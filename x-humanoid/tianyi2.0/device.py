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
"""

from __future__ import annotations

import json
import math
import threading
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import String, Bool

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

_WAIST_JOINTS = {
    31: "waist_yaw_joint",
    32: "waist_pitch_joint",
}

_LEG_JOINTS = {
    51: "left_hip_pitch_joint",
    52: "left_knee_pitch_joint",
}

_ALL_JOINTS = {**_HEAD_JOINTS, **_ARM_LEFT_JOINTS, **_ARM_RIGHT_JOINTS, **_WAIST_JOINTS, **_LEG_JOINTS}

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


def _clamp(value: float, lower: float, upper: float) -> float:
    """Clamp a numeric input to a safe, documented range."""
    return max(lower, min(upper, float(value)))


class _ActionSequence:
    """Run one cancellable actuator sequence at a time."""

    def __init__(self, name: str):
        self._name = name
        self._lock = threading.Lock()
        self._cancel_event: threading.Event | None = None
        self._thread: threading.Thread | None = None

    def start(self, worker) -> None:
        self.cancel()
        cancel_event = threading.Event()

        def _run():
            try:
                worker(cancel_event)
            except Exception as e:
                print(f"[{self._name}] sequence failed: {e}")
            finally:
                with self._lock:
                    if self._cancel_event is cancel_event:
                        self._cancel_event = None
                        self._thread = None

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
        self._battery = {}
        self._estop = {}
        self._force_left = {}
        self._force_right = {}
        self._lock = threading.Lock()

        # Topics for Agent Core (domain 42)
        self._topic_joints = f"/{namespace}/state/joints"
        self._topic_battery = f"/{namespace}/state/battery"
        self._topic_estop = f"/{namespace}/state/estop"
        self._topic_force = f"/{namespace}/state/force"

        # Subscriber node (domain 0 - tianyi)
        self._sub_node = Node("tianyi2_state_sub", context=ros2.ctx_tianyi)
        ros2.executor_tianyi.add_node(self._sub_node)

        # Publisher node (domain 42 - agent core)
        self._pub_node = Node("tianyi2_state_pub", context=ros2.ctx_core)
        ros2.executor_core.add_node(self._pub_node)

        self._pub_joints = self._pub_node.create_publisher(String, self._topic_joints, _LOW_LAT_QOS)
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
                "description": "天轶2.0 全身关节状态 — 位置/速度/电流/温度 (头/臂/腰/腿 共21个关节)",
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

    def _publish_loop(self):
        """Publish aggregated state at 10Hz for joints, 1Hz for battery/estop."""
        joint_counter = 0
        while self._running:
            time.sleep(0.1)  # 10Hz
            joint_counter += 1

            # Publish joints
            with self._lock:
                if self._joint_data:
                    joints = []
                    for motor_id, data in self._joint_data.items():
                        name = _ALL_JOINTS.get(motor_id, f"motor_{motor_id}")
                        joints.append({
                            "idx": motor_id,
                            "name": name,
                            "q": data["pos"],
                            "dq": data["speed"],
                            "current": data["current"],
                            "temp": data["temp"],
                        })
                    payload = json.dumps({"joints": joints})
                    msg = String()
                    msg.data = payload
                    self._pub_joints.publish(msg)

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
                return {"joints": list(self._joint_data.values())}
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
        """Ensure orbbec_head.service is running. Use nsenter to access host systemd."""
        import subprocess
        try:
            # Use nsenter to run systemctl on host PID 1's namespace
            result = subprocess.run(
                ["nsenter", "-t", "1", "-m", "-u", "-i", "-n", "-p", "--",
                 "systemctl", "is-active", "orbbec_head.service"],
                capture_output=True, text=True, timeout=5)
            if result.stdout.strip() == "active":
                print("[CameraPlugin] orbbec_head.service already active")
                return
            # Start it
            subprocess.run(
                ["nsenter", "-t", "1", "-m", "-u", "-i", "-n", "-p", "--",
                 "systemctl", "start", "orbbec_head.service"],
                capture_output=True, text=True, timeout=10)
            print("[CameraPlugin] orbbec_head.service started via nsenter")
        except Exception as e:
            print(f"[CameraPlugin] WARNING: could not start orbbec service ({e})")

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
# AsrPlugin (sensor)
# ══════════════════════════════════════════════════════════════════════════════

class AsrPlugin:
    """语音识别结果 (lyre ASR)"""

    def __init__(self, plugin_config: dict, namespace: str, ros2):
        self._ns = namespace
        self._ros2 = ros2
        self._topic = f"/{namespace}/asr/text"
        self._running = False

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

    def _on_asr(self, msg):
        if not self._running:
            return
        out = String()
        out.data = json.dumps({"id": msg.id, "text": msg.text})
        self._pub.publish(out)

    def _on_asr_string(self, msg):
        if not self._running:
            return
        self._pub.publish(msg)

    def dispatch(self, action: str, args: dict) -> dict:
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
                cmd.cur = 3.0  # A (max current)
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
                        "enum": ["nod", "shake", "scan", "tilt", "reset", "stop"],
                        "default": "nod",
                        "description": "头部动作，可选[nod, shake, scan, tilt, reset, stop]",
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
                "x-action-params": {
                    "nod": {"params": ["cycles", "nod_amplitude", "speed"], "description": "向下点头后回正，不经过抬头姿态"},
                    "shake": {"params": ["cycles", "shake_amplitude", "speed"], "description": "在左右方向之间连续摇头后回正"},
                    "scan": {"params": ["cycles", "scan_amplitude", "speed", "scan_hold"], "description": "依次观察左侧并停留、回中、观察右侧并停留、回中"},
                    "tilt": {"params": ["side", "tilt_amplitude", "speed", "hold"], "description": "向指定方向歪头、保持后回正"},
                    "reset": {"params": ["speed"], "description": "取消序列并将头部回正"},
                    "stop": {"params": [], "description": "取消尚未发送的后续动作帧"},
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
        if action == "stop":
            return {"state": "stopped", "cancelled": self._sequence.cancel()}
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
        self._sequence.start(_worker)
        first_target = frames[0][:3]
        feedback = self._wait_for_head_feedback(
            first_target, baseline_seq, baseline)
        if feedback.get("state") == "error":
            self._sequence.cancel()
            return feedback
        return {
            "state": "running", "gesture": action, "cycles": cycles,
            "amplitude": amplitude, "speed": speed,
            "feedback_verified": True,
            "feedback": feedback,
        }

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
                cmd.cur = 3.0
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

    # Conservative raw-control starting gains. The vendor-side controller
    # applies these gains; this driver does not rate-limit the position step.
    _DEFAULT_KP = 10.0
    _DEFAULT_KD = 8.0
    _KP_RANGE = (5.0, 100.0)
    _KD_RANGE = (5.0, 30.0)

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

    def get_tool(self) -> dict:
        return {
            "name": "arm",
            "type": "actuator",
            "description": "天轶2.0 双臂控制 — 每臂7DOF (肩3+肘1+腕3), 位置/力位混合模式",
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {"type": "string", "enum": ["move_pos", "move_ctrl"],
                               "default": "move_pos",
                               "description": "控制模式"},
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
                              "description": "运动速度(rad/s), 范围[0.2, 1.5], 默认0.5"},
                    "kp": {"type": "array", "items": {"type": "number", "minimum": 5, "maximum": 100},
                           "minItems": 7, "maxItems": 7,
                           "default": [10, 10, 10, 10, 10, 10, 10],
                           "description": "位置增益(7个), 范围[5,100], 默认10；move_ctrl无速度限制，仅建议小角度低增益调试"},
                    "kd": {"type": "array", "items": {"type": "number", "minimum": 5, "maximum": 30},
                           "minItems": 7, "maxItems": 7,
                           "default": [8, 8, 8, 8, 8, 8, 8],
                           "description": "速度阻尼增益(7个), 范围[5,30], 默认8"},
                },
                "required": ["action"],
                "x-action-params": {
                    "move_pos": {"params": ["left_positions", "right_positions", "speed"],
                                 "description": "位置模式: 同时输入并执行左右臂两组实际关节角度"},
                    "move_ctrl": {"params": ["left_positions", "right_positions", "kp", "kd"],
                                  "description": "力位混合模式: 同时输入并执行左右臂实际位置和共用增益"},
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
                    cmd.name = base_id + i
                    cmd.pos = _deg2rad(deg)
                    cmd.spd = speed
                    cmd.cur = 5.0
                    cmds.append(cmd)

            msg.cmds = cmds
            self._pos_publisher.publish(msg)
            return {"state": "moving", "side": "both", "joints": len(cmds)}
        except Exception as e:
            return {"error": str(e)}

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
                            "high_five", "reset", "stop",
                        ],
                        "default": "welcome",
                        "description": "手臂动作，可选[salute, welcome, raise, shake_hands, high_five, reset, stop]",
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
                "x-action-params": {
                    "salute": {"params": ["salute_side", "speed"], "description": "抬起小臂、将手靠近额侧、停留后回正"},
                    "welcome": {"params": ["side", "cycles", "speed"], "description": "在身体侧上方抬起手掌并左右摆动后回正"},
                    "raise": {"params": ["side", "speed"], "description": "将手臂高举到头部上方后回正"},
                    "shake_hands": {"params": ["side", "cycles", "speed"], "description": "向前伸手并轻柔上下摆动，做出握手动作"},
                    "high_five": {"params": ["side", "speed"], "description": "将手掌伸到身体前方并保持在肩部附近，做出击掌等待姿势"},
                    "reset": {"params": ["side", "speed"], "description": "取消序列并回到中性姿态"},
                    "stop": {"params": [], "description": "取消尚未发送的后续动作帧"},
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
        if action == "stop":
            return {"state": "stopped", "cancelled": self._sequence.cancel()}
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
        self._sequence.start(_worker)
        feedback = self._wait_for_arm_feedback(
            side, frames[0][0], baseline_seq, baseline)
        if feedback.get("state") == "error":
            self._sequence.cancel()
            return feedback
        return {
            "state": "running", "gesture": action, "side": side,
            "cycles": cycles, "speed": speed,
            "feedback_verified": True,
            "feedback": feedback,
        }

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
                "the configured 5A maximum current may be insufficient; verify with the robot vendor before increasing it",
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
                    cmd.name = base_id + index
                    cmd.pos = _deg2rad(float(deg))
                    cmd.spd = speed
                    cmd.cur = 5.0
                    msg.cmds.append(cmd)
            self._publisher.publish(msg)
            return {"state": "moving", "side": side, "joints": len(msg.cmds)}
        except Exception as e:
            return {"error": str(e)}


# ══════════════════════════════════════════════════════════
# WaistPlugin (actuator)
# ══════════════════════════════════════════════════════════════════════════

class WaistPlugin:
    """腰部2DOF控制 (yaw/pitch)"""

    def __init__(self, plugin_config: dict, namespace: str, ros2):
        self._ns = namespace
        self._ros2 = ros2
        self._pub_node = Node("tianyi2_waist_pub", context=ros2.ctx_tianyi)
        ros2.executor_tianyi.add_node(self._pub_node)
        self._publisher = None

    def get_tool(self) -> dict:
        return {
            "name": "waist",
            "type": "actuator",
            "description": "天轶2.0 腰部控制 — 2DOF (yaw±160°, pitch -45°~120°)",
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {"type": "string", "enum": ["move_pos"],
                               "description": "控制动作"},
                    "yaw": {"type": "number", "description": "偏航角(度), 范围[-160, 180]"},
                    "pitch": {"type": "number", "description": "俯仰角(度), 范围[-45, 120]"},
                },
                "required": ["action"],
                "x-action-params": {
                    "move_pos": {"params": ["yaw", "pitch"],
                                 "description": "移动腰部到指定角度"},
                },
            },
        }

    def start(self):
        try:
            from bodyctrl_msgs.msg import CmdSetMotorPosition
            self._publisher = self._pub_node.create_publisher(
                CmdSetMotorPosition, "/waist/cmd_pos", _RELIABLE_QOS)
            print("[WaistPlugin] publisher created")
        except ImportError as e:
            print(f"[WaistPlugin] WARNING: msg import failed ({e})")

    def stop(self):
        pass

    def dispatch(self, action: str, args: dict) -> dict:
        if action == "move_pos":
            yaw = args.get("yaw", 0)
            pitch = args.get("pitch", 0)
            return self._send_pos(yaw, pitch)
        elif action in ("start", "info"):
            return {"state": "ready"}
        elif action == "stop":
            return {"state": "idle"}
        return {"error": f"unknown action: {action}"}

    def _send_pos(self, yaw_deg: float, pitch_deg: float) -> dict:
        if not self._publisher:
            return {"error": "publisher not initialized"}
        try:
            from bodyctrl_msgs.msg import CmdSetMotorPosition, SetMotorPosition
            msg = CmdSetMotorPosition()
            cmds = []
            for motor_id, deg in [(31, yaw_deg), (32, pitch_deg)]:
                cmd = SetMotorPosition()
                cmd.name = motor_id
                cmd.pos = _deg2rad(deg)
                cmd.spd = 0.5  # rad/s
                cmd.cur = 10.0  # A
                cmds.append(cmd)
            msg.cmds = cmds
            self._publisher.publish(msg)
            return {"state": "moving", "yaw": yaw_deg, "pitch": pitch_deg}
        except Exception as e:
            return {"error": str(e)}


# ══════════════════════════════════════════════════════════════════════════════
# HandPlugin (actuator)
# ══════════════════════════════════════════════════════════════════════════════

class HandPlugin:
    """Inspire灵巧手控制 — 6指位置/力/速度控制"""

    # 手指ID: 1=小指, 2=无名指, 3=中指, 4=食指, 5=拇指弯曲, 6=拇指旋转
    _FINGER_NAMES = ["little", "ring", "middle", "index", "thumb_bend", "thumb_rotation"]

    _GRASP_PRESETS = {
        "power": [100, 100, 100, 100, 100, 50],
        "pinch": [0, 0, 0, 80, 80, 60],
        "lateral": [100, 100, 100, 100, 0, 80],
        "tripod": [0, 0, 80, 80, 80, 50],
        "point": [0, 0, 0, 0, 100, 50],
    }

    def __init__(self, plugin_config: dict, namespace: str, ros2):
        self._ns = namespace
        self._ros2 = ros2
        self._pub_node = Node("tianyi2_hand_pub", context=ros2.ctx_tianyi)
        ros2.executor_tianyi.add_node(self._pub_node)
        self._left_pub = None
        self._right_pub = None

    def get_tool(self) -> dict:
        return {
            "name": "hand",
            "type": "actuator",
            "description": "天轶2.0 Inspire灵巧手 — 每手6指, 位置控制(0-100%: 0=张开, 100=握紧)",
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {"type": "string",
                               "enum": ["set_angle", "open", "close", "grasp"],
                               "description": "控制动作"},
                    "side": {"type": "string", "enum": ["left", "right", "both"],
                             "description": "控制哪只手"},
                    "angles": {"type": "array", "items": {"type": "number"},
                               "description": "6个手指位置(0-100%): [小指, 无名指, 中指, 食指, 拇指弯曲, 拇指旋转]"},
                    "grasp_type": {"type": "string",
                                   "enum": ["power", "pinch", "lateral", "tripod", "point"],
                                   "description": "预设抓取模式"},
                },
                "required": ["action"],
                "x-action-params": {
                    "set_angle": {"params": ["side", "angles"],
                                  "description": "设置手指角度(6个值, 0-100%)"},
                    "open": {"params": ["side"],
                             "description": "完全张开手"},
                    "close": {"params": ["side"],
                              "description": "完全握紧手"},
                    "grasp": {"params": ["side", "grasp_type"],
                              "description": "执行预设抓取动作"},
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

    def stop(self):
        pass

    def dispatch(self, action: str, args: dict) -> dict:
        side = args.get("side", "both")
        if action == "set_angle":
            angles = args.get("angles", [])
            if len(angles) != 6:
                return {"error": "angles must have exactly 6 values (0-100%)"}
            return self._send_angles(side, angles)
        elif action == "open":
            return self._send_angles(side, [0, 0, 0, 0, 0, 0])
        elif action == "close":
            return self._send_angles(side, [100, 100, 100, 100, 100, 50])
        elif action == "grasp":
            grasp_type = args.get("grasp_type", "power")
            angles = self._GRASP_PRESETS.get(grasp_type, self._GRASP_PRESETS["power"])
            return self._send_angles(side, angles)
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
            # Angles are in percentage (0-100), position field is percentage/100
            positions = [a / 100.0 for a in angles]

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

    def get_tool(self) -> dict:
        return {
            "name": "tts",
            "type": "actuator",
            "description": "天轶2.0 语音合成 (TTS) — 文字转语音播放",
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {"type": "string", "enum": ["speak", "stop", "pause", "resume"],
                               "description": "控制动作"},
                    "text": {"type": "string", "description": "要播放的文本"},
                    "force": {"type": "boolean", "description": "是否强制播放(打断当前播放)", "default": False},
                },
                "required": ["action"],
                "x-action-params": {
                    "speak": {"params": ["text", "force"], "description": "合成并播放文本"},
                    "stop": {"params": [], "description": "停止播放"},
                    "pause": {"params": [], "description": "暂停播放"},
                    "resume": {"params": [], "description": "恢复播放"},
                },
            },
        }

    def start(self):
        try:
            from lyre_msgs.srv import PlayText, PlayStop, PlayPause, PlayResume
            self._play_client = self._srv_node.create_client(PlayText, "/audio_play/play_text")
            self._stop_client = self._srv_node.create_client(PlayStop, "/audio_play/stop")
            self._pause_client = self._srv_node.create_client(PlayPause, "/audio_play/pause")
            self._resume_client = self._srv_node.create_client(PlayResume, "/audio_play/resume")
            print("[TtsPlugin] service clients created")
        except ImportError as e:
            print(f"[TtsPlugin] WARNING: msg import failed ({e})")

    def stop(self):
        pass

    def dispatch(self, action: str, args: dict) -> dict:
        if action == "speak":
            text = args.get("text", "")
            force = args.get("force", False)
            if not text:
                return {"error": "text is required"}
            return self._speak(text, force)
        elif action == "stop":
            return self._call_empty_service(self._stop_client, "stop")
        elif action == "pause":
            return self._call_empty_service(self._pause_client, "pause")
        elif action == "resume":
            return self._call_empty_service(self._resume_client, "resume")
        elif action in ("start", "info"):
            return {"state": "ready"}
        return {"error": f"unknown action: {action}"}

    def _speak(self, text: str, force: bool) -> dict:
        if not self._play_client:
            return {"error": "service client not initialized"}
        try:
            from lyre_msgs.srv import PlayText
            req = PlayText.Request()
            req.text = text
            req.force = force
            req.last = True
            future = self._play_client.call_async(req)
            # Non-blocking, just return immediately
            return {"state": "speaking", "text": text[:50]}
        except Exception as e:
            return {"error": str(e)}

    def _call_empty_service(self, client, action_name: str) -> dict:
        if not client:
            return {"error": f"{action_name} service client not initialized"}
        try:
            req = type(client.srv_type.Request)()
            client.call_async(req)
            return {"state": action_name}
        except Exception as e:
            return {"error": str(e)}


# ══════════════════════════════════════════════════════════════════════════════
# NavPlugin (actuator)
# ══════════════════════════════════════════════════════════════════════════════

class NavPlugin:
    """底盘导航控制 — 自主导航/遥控/旋转/回桩"""

    def __init__(self, plugin_config: dict, namespace: str, ros2, slamtec_client):
        self._ns = namespace
        self._ros2 = ros2
        self._slamtec = slamtec_client

        # cmd_vel publisher for direct velocity control (domain 0)
        self._vel_node = Node("tianyi2_nav_vel", context=ros2.ctx_tianyi)
        ros2.executor_tianyi.add_node(self._vel_node)
        self._vel_pub = None

    def get_tool(self) -> dict:
        return {
            "name": "nav",
            "type": "actuator",
            "description": "天轶2.0 底盘导航 — 自主导航到目标点/方向遥控/旋转/回桩充电 (Slamtec轮式底盘)",
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {"type": "string",
                               "enum": ["move_to", "move_by", "rotate", "rotate_to", "go_home", "stop", "get_pose"],
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
                "x-action-params": {
                    "move_to": {"params": ["x", "y", "speed"],
                                "description": "自主导航到目标点(带避障)"},
                    "move_by": {"params": ["direction", "speed"],
                                "description": "方向遥控移动(不避障, 持续500ms)"},
                    "rotate": {"params": ["angle"],
                               "description": "原地旋转指定角度(度)"},
                    "rotate_to": {"params": ["angle"],
                                  "description": "原地旋转到绝对角度(度)"},
                    "go_home": {"params": [],
                                "description": "自主导航回充电桩"},
                    "stop": {"params": [],
                             "description": "停止当前导航动作"},
                    "get_pose": {"params": [],
                                 "description": "获取当前位姿(x, y, yaw)"},
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
            return {"state": "navigating", "target": {"x": x, "y": y}, "api_result": result}

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
            return {"state": "rotating", "angle": angle_deg, "api_result": result}

        elif action == "rotate_to":
            angle_deg = args.get("angle", 0)
            angle_rad = _deg2rad(angle_deg)
            result = self._slamtec.rotate_to(angle_rad)
            return {"state": "rotating_to", "angle": angle_deg, "api_result": result}

        elif action == "go_home":
            result = self._slamtec.go_home()
            return {"state": "going_home", "api_result": result}

        elif action == "stop":
            result = self._slamtec.cancel_current_action()
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
