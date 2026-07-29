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
  ServiceStatePlugin (sensor)           — ROS2 服务节点运行状态
  MotorFaultsPlugin (sensor)            — 电机错误码汇总
  CameraPlugin     (sensor)             — Orbbec 头部相机
  AsrPlugin        (sensor)             — 语音识别结果
  NavStatePlugin   (sensor)             — 底盘导航状态
  StatusLightPlugin (actuator)          — 机器人状态灯事件
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


def _deg2rad(deg: float) -> float:
    return deg * math.pi / 180.0


def _rad2deg(rad: float) -> float:
    return rad * 180.0 / math.pi


def _stamp_to_ms(stamp) -> int | None:
    """Convert a ROS2 builtin_interfaces/Time-like object to milliseconds."""
    if stamp is None:
        return None
    try:
        return int(stamp.sec * 1000 + stamp.nanosec / 1_000_000)
    except (AttributeError, TypeError, ValueError):
        return None


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


# ══════════════════════════════════════════════════════════════════════════════════
# ServiceStatePlugin (sensor)
# ══════════════════════════════════════════════════════════════════════════════════

class ServiceStatePlugin:
    """Track the latest running/idle state for robot services or topics."""

    def __init__(self, plugin_config: dict, namespace: str, ros2):
        configured_topics = plugin_config.get("source_topics")
        if configured_topics is None:
            configured_topics = [plugin_config.get("source_topic", "")]
        self._source_topics = [
            str(topic).strip() for topic in configured_topics
            if str(topic).strip()
        ]
        self._topic = f"/{namespace}/state/service_state"
        self._running = False
        self._services = {}
        self._last_update_ms = None
        self._lock = threading.Lock()

        self._sub_node = Node(
            "tianyi2_service_state_sub", context=ros2.ctx_tianyi)
        ros2.executor_tianyi.add_node(self._sub_node)
        self._pub_node = Node(
            "tianyi2_service_state_pub", context=ros2.ctx_core)
        ros2.executor_core.add_node(self._pub_node)
        self._publisher = self._pub_node.create_publisher(
            String, self._topic, _RELIABLE_QOS)

    def get_tool(self) -> dict:
        return {
            "name": "service_state",
            "type": "sensor",
            "description": "天轶2.0 内部服务状态 — 按Topic汇总节点 running/idle 状态",
            "inputSchema": {"type": "object", "properties": {}},
            "topic_out": [{"topic": self._topic, "format": "data/json"}],
        }

    def start(self):
        self._running = True
        if not self._source_topics:
            print("[ServiceStatePlugin] source topics not configured")
            return
        try:
            from bodyctrl_msgs.msg import NodeState
            for source_topic in self._source_topics:
                self._sub_node.create_subscription(
                    NodeState, source_topic,
                    lambda msg, topic=source_topic: self._on_state(msg, topic),
                    _RELIABLE_QOS,
                )
        except ImportError as e:
            print(f"[ServiceStatePlugin] WARNING: msg import failed ({e}), running in stub mode")

    def stop(self):
        self._running = False

    def _on_state(self, msg, source_topic: str = ""):
        state_code = int(msg.state)
        state = {0: "idle", 1: "running"}.get(state_code, "unknown")
        now_ms = int(time.time() * 1000)
        service_key = f"{source_topic}:{msg.topic}" if source_topic else msg.topic
        with self._lock:
            self._services[service_key] = {
                "topic": msg.topic,
                "source_topic": source_topic,
                "state": state,
                "state_code": state_code,
                "timestamp_ms": _stamp_to_ms(msg.header.stamp) or now_ms,
            }
            self._last_update_ms = now_ms
            payload = self._build_payload_locked()
        if self._running:
            self._publisher.publish(String(data=json.dumps(payload)))

    def _build_payload_locked(self) -> dict:
        services = sorted(self._services.values(), key=lambda x: x["topic"])
        return {
            "available": self._last_update_ms is not None,
            "timestamp_ms": int(time.time() * 1000),
            "last_update_ms": self._last_update_ms,
            "service_count": len(services),
            "running_count": sum(s["state"] == "running" for s in services),
            "idle_count": sum(s["state"] == "idle" for s in services),
            "unknown_count": sum(s["state"] == "unknown" for s in services),
            "services": services,
        }

    def dispatch(self, action: str, args: dict) -> dict | None:
        if action == "service_state":
            with self._lock:
                return self._build_payload_locked()
        if action == "start":
            return {"state": "running"}
        if action == "stop":
            return {"state": "idle"}
        if action == "info":
            return {
                "state": "running",
                "source_topics": self._source_topics,
                "topic_out": [{"topic": self._topic, "format": "data/json"}],
            }
        return None


# ════════════════════════════════════════════════════════════════════════════════
# MotorFaultsPlugin (sensor)
# ═════════════════════════════════════════════════════════════════════════════════

class MotorFaultsPlugin:
    """Expose non-zero motor error codes without duplicating joint telemetry."""

    def __init__(self, plugin_config: dict, namespace: str, ros2):
        default_topics = [
            "/head/status", "/arm/status", "/waist/status", "/leg/status"]
        self._source_topics = plugin_config.get(
            "motor_status_topics", default_topics)
        self._topic = f"/{namespace}/state/motor_faults"
        self._running = False
        self._faults = {}
        self._last_update_ms = None
        self._lock = threading.Lock()

        self._sub_node = Node(
            "tianyi2_motor_faults_sub", context=ros2.ctx_tianyi)
        ros2.executor_tianyi.add_node(self._sub_node)
        self._pub_node = Node(
            "tianyi2_motor_faults_pub", context=ros2.ctx_core)
        ros2.executor_core.add_node(self._pub_node)
        self._publisher = self._pub_node.create_publisher(
            String, self._topic, _RELIABLE_QOS)

    def get_tool(self) -> dict:
        return {
            "name": "motor_faults",
            "type": "sensor",
            "description": "天轶2.0 电机故障汇总 — 仅输出非零电机错误码及关节名",
            "inputSchema": {"type": "object", "properties": {}},
            "topic_out": [{"topic": self._topic, "format": "data/json"}],
        }

    def start(self):
        self._running = True
        try:
            from bodyctrl_msgs.msg import MotorStatusMsg
            for source_topic in self._source_topics:
                self._sub_node.create_subscription(
                    MotorStatusMsg, source_topic,
                    lambda msg, topic=source_topic: self._on_status(msg, topic),
                    _RELIABLE_QOS,
                )
        except ImportError as e:
            print(f"[MotorFaultsPlugin] WARNING: msg import failed ({e}), running in stub mode")

    def stop(self):
        self._running = False

    def _on_status(self, msg, source_topic: str):
        now_ms = int(time.time() * 1000)
        with self._lock:
            for motor in msg.status:
                motor_id = int(motor.name)
                if int(motor.error) == 0:
                    self._faults.pop(motor_id, None)
                    continue
                self._faults[motor_id] = {
                    "motor_id": motor_id,
                    "joint_name": _ALL_JOINTS.get(
                        motor_id, f"motor_{motor_id}"),
                    "error_code": int(motor.error),
                    "source_topic": source_topic,
                    "timestamp_ms": _stamp_to_ms(msg.header.stamp) or now_ms,
                }
            self._last_update_ms = now_ms
            payload = self._build_payload_locked()
        if self._running:
            self._publisher.publish(String(data=json.dumps(payload)))

    def _build_payload_locked(self) -> dict:
        faults = sorted(
            self._faults.values(), key=lambda item: item["motor_id"])
        return {
            "available": self._last_update_ms is not None,
            "timestamp_ms": int(time.time() * 1000),
            "last_update_ms": self._last_update_ms,
            "healthy": self._last_update_ms is not None and not faults,
            "fault_count": len(faults),
            "summary": (
                "尚未收到电机状态数据"
                if self._last_update_ms is None
                else (f"检测到{len(faults)}个电机故障" if faults
                      else "当前没有非零电机错误码")
            ),
            "faults": faults,
        }

    def dispatch(self, action: str, args: dict) -> dict | None:
        if action == "motor_faults":
            with self._lock:
                return self._build_payload_locked()
        if action == "start":
            return {"state": "running"}
        if action == "stop":
            return {"state": "idle"}
        if action == "info":
            return {
                "state": "running",
                "source_topics": self._source_topics,
                "topic_out": [{"topic": self._topic, "format": "data/json"}],
            }
        return None


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


# ══════════════════════════════════════════════════════════════════════════════
# StatusLightPlugin (actuator)
# ══════════════════════════════════════════════════════════════════════════════

class StatusLightPlugin:
    """发布电源板定义的状态灯事件。

    PowerLightCtrl 消息只包含 cmd，灯效由电源板根据事件编号决定；
    因此这里不暴露硬件未定义的 RGB、亮度或闪烁频率参数。
    """

    _EVENTS = {
        "battery_supply": 1,
        "power_on_start": 2,
        "power_on_finish": 3,
        "service_start": 4,
        "service_finish": 5,
        "self_check_start": 6,
        "self_check_failed": 7,
        "self_check_success": 8,
        "fault_occur": 9,
        "fault_clear": 10,
        "voice_wakeup": 11,
        "voice_response": 12,
        "voice_exit": 13,
        "running_start": 14,
        "running_finish": 15,
        "power_off": 16,
        "warn_occur": 17,
        "warn_clear": 18,
    }

    def __init__(self, plugin_config: dict, namespace: str, ros2):
        self._pub_node = Node("tianyi2_status_light_pub", context=ros2.ctx_tianyi)
        ros2.executor_tianyi.add_node(self._pub_node)
        self._publisher = None

    def get_tool(self) -> dict:
        return {
            "name": "status_light",
            "type": "actuator",
            "description": "天轶2.0 状态灯控制 — 向电源板发送官方定义的灯光状态事件",
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {
                        "type": "string", "enum": ["set_event"],
                        "description": "发送状态灯事件",
                    },
                    "event": {
                        "type": "string", "enum": list(self._EVENTS),
                        "description": "PowerLightCtrl.msg 定义的状态事件",
                    },
                },
                "required": ["action"],
                "x-action-params": {
                    "set_event": {
                        "params": ["event"],
                        "description": "设置状态灯事件",
                    },
                },
            },
        }

    def start(self):
        try:
            from bodyctrl_msgs.msg import PowerLightCtrl
            self._publisher = self._pub_node.create_publisher(
                PowerLightCtrl, "/power/light/ctrl", _RELIABLE_QOS)
            print("[StatusLightPlugin] publisher created")
        except ImportError as e:
            print(f"[StatusLightPlugin] WARNING: msg import failed ({e})")

    def stop(self):
        pass

    def dispatch(self, action: str, args: dict) -> dict:
        if action == "set_event":
            event = str(args.get("event", "")).strip()
            if event not in self._EVENTS:
                return {"error": "event must be one of the documented PowerLightCtrl events"}
            if not self._publisher:
                return {"error": "publisher not initialized"}
            try:
                from bodyctrl_msgs.msg import PowerLightCtrl
                msg = PowerLightCtrl()
                msg.cmd = self._EVENTS[event]
                self._publisher.publish(msg)
                return {"state": "sent", "event": event, "cmd": msg.cmd}
            except Exception as e:
                return {"error": str(e)}
        if action in ("start", "info"):
            return {"state": "ready" if self._publisher else "idle"}
        if action == "stop":
            return {"state": "idle"}
        return {"error": f"unknown action: {action}"}


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

    def __init__(self, plugin_config: dict, namespace: str, ros2):
        self._pub_node = Node("tianyi2_head_gesture_pub", context=ros2.ctx_tianyi)
        ros2.executor_tianyi.add_node(self._pub_node)
        self._publisher = None
        self._sequence = _ActionSequence("HeadGesturePlugin")

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
                        "description": "头部动作",
                    },
                    "cycles": {
                        "type": "integer", "minimum": 1, "maximum": 5,
                        "default": 2, "description": "循环次数",
                    },
                    "amplitude": {
                        "type": "number", "minimum": 5, "maximum": 45,
                        "description": "动作幅度(度)，会再按各关节极限限幅",
                    },
                    "speed": {
                        "type": "number", "minimum": 5, "maximum": 60,
                        "default": 30, "description": "动作速度(度/秒)",
                    },
                    "side": {
                        "type": "string", "enum": ["left", "right"],
                        "default": "left", "description": "歪头方向",
                    },
                    "hold": {
                        "type": "number", "minimum": 0.2, "maximum": 3.0,
                        "default": 0.8, "description": "歪头保持时间(秒)",
                    },
                },
                "required": ["action"],
                "x-action-params": {
                    "nod": {"params": ["cycles", "amplitude", "speed"], "description": "连续点头后回正"},
                    "shake": {"params": ["cycles", "amplitude", "speed"], "description": "连续摇头后回正"},
                    "scan": {"params": ["cycles", "amplitude", "speed"], "description": "左右观察后回正"},
                    "tilt": {"params": ["side", "amplitude", "speed", "hold"], "description": "向指定方向歪头后回正"},
                    "reset": {"params": ["speed"], "description": "取消序列并将头部回正"},
                    "stop": {"params": [], "description": "取消尚未发送的后续动作帧"},
                },
            },
        }

    def start(self):
        try:
            from bodyctrl_msgs.msg import CmdSetMotorPosition
            self._publisher = self._pub_node.create_publisher(
                CmdSetMotorPosition, "/head/cmd_pos", _RELIABLE_QOS)
            print("[HeadGesturePlugin] publisher created")
        except ImportError as e:
            print(f"[HeadGesturePlugin] WARNING: msg import failed ({e})")

    def stop(self):
        self._sequence.cancel()

    def dispatch(self, action: str, args: dict) -> dict:
        if action in ("start", "info"):
            return {"state": "ready" if self._publisher else "idle"}
        if action == "stop":
            return {"state": "stopped", "cancelled": self._sequence.cancel()}
        if action == "reset":
            self._sequence.cancel()
            return self._publish_pose(0, 0, 0, args.get("speed", 30))
        if action not in ("nod", "shake", "scan", "tilt"):
            return {"error": f"unknown action: {action}"}
        if not self._publisher:
            return {"error": "publisher not initialized"}

        cycles = int(_clamp(args.get("cycles", 2), 1, 5))
        speed = _clamp(args.get("speed", 30), 5, 60)
        amplitude_default = 12 if action in ("nod", "tilt") else 25
        amplitude = _clamp(args.get("amplitude", amplitude_default), 5, 45)

        frames: list[tuple[float, float, float, float]] = []
        if action == "nod":
            amplitude = min(amplitude, 20)
            for _ in range(cycles):
                frames.extend([(0, amplitude, 0, amplitude / speed),
                               (0, -amplitude, 0, 2 * amplitude / speed)])
        elif action == "shake":
            amplitude = min(amplitude, 45)
            for _ in range(cycles):
                frames.extend([(amplitude, 0, 0, amplitude / speed),
                               (-amplitude, 0, 0, 2 * amplitude / speed)])
        elif action == "scan":
            amplitude = min(amplitude, 45)
            for _ in range(cycles):
                frames.extend([(amplitude, 0, 0, amplitude / speed),
                               (0, 0, 0, amplitude / speed),
                               (-amplitude, 0, 0, amplitude / speed),
                               (0, 0, 0, amplitude / speed)])
        else:
            amplitude = min(amplitude, 20)
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

        self._sequence.start(_worker)
        return {
            "state": "running", "gesture": action, "cycles": cycles,
            "amplitude": amplitude, "speed": speed,
        }

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

    def __init__(self, plugin_config: dict, namespace: str, ros2):
        self._ns = namespace
        self._ros2 = ros2
        self._pub_node = Node("tianyi2_arm_pub", context=ros2.ctx_tianyi)
        ros2.executor_tianyi.add_node(self._pub_node)
        self._pos_publisher = None
        self._ctrl_publisher = None

    def get_tool(self) -> dict:
        return {
            "name": "arm",
            "type": "actuator",
            "description": "天轶2.0 双臂控制 — 每臂7DOF (肩3+肘1+腕3), 位置/力位混合模式",
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {"type": "string", "enum": ["move_pos", "move_ctrl"],
                               "description": "控制模式"},
                    "side": {"type": "string", "enum": ["left", "right", "both"],
                             "description": "控制哪只手臂"},
                    "positions": {"type": "array", "items": {"type": "number"},
                                  "description": "7个关节角度(度): [肩pitch, 肩roll, 肩yaw, 肘pitch, 腕yaw, 腕pitch, 腕roll]"},
                    "speed": {"type": "number", "description": "运动速度(rad/s), 默认1.0"},
                    "kp": {"type": "array", "items": {"type": "number"},
                           "description": "位置增益(7个), 范围[0,2000]"},
                    "kd": {"type": "array", "items": {"type": "number"},
                           "description": "速度增益(7个), 范围[0,300]"},
                },
                "required": ["action"],
                "x-action-params": {
                    "move_pos": {"params": ["side", "positions", "speed"],
                                 "description": "位置模式: 移动手臂关节到指定角度(度)"},
                    "move_ctrl": {"params": ["side", "positions", "kp", "kd"],
                                  "description": "力位混合模式: 指定位置+增益"},
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
            print("[ArmPlugin] publishers created")
        except ImportError as e:
            print(f"[ArmPlugin] WARNING: msg import failed ({e})")

    def stop(self):
        pass

    def dispatch(self, action: str, args: dict) -> dict:
        if action == "move_pos":
            side = args.get("side", "left")
            positions = args.get("positions", [])
            speed = args.get("speed", 1.0)
            if len(positions) != 7:
                return {"error": "positions must have exactly 7 values (degrees)"}
            return self._send_pos(side, positions, speed)
        elif action == "move_ctrl":
            side = args.get("side", "left")
            positions = args.get("positions", [])
            kp = args.get("kp", [200] * 7)
            kd = args.get("kd", [20] * 7)
            if len(positions) != 7:
                return {"error": "positions must have exactly 7 values (degrees)"}
            return self._send_ctrl(side, positions, kp, kd)
        elif action in ("start", "info"):
            return {"state": "ready"}
        elif action == "stop":
            return {"state": "idle"}
        return {"error": f"unknown action: {action}"}

    def _send_pos(self, side: str, positions_deg: list, speed: float) -> dict:
        if not self._pos_publisher:
            return {"error": "publisher not initialized"}
        try:
            from bodyctrl_msgs.msg import CmdSetMotorPosition, SetMotorPosition
            msg = CmdSetMotorPosition()
            cmds = []
            sides = []
            if side in ("left", "both"):
                sides.append(("left", 11))
            if side in ("right", "both"):
                sides.append(("right", 21))

            for side_name, base_id in sides:
                for i, deg in enumerate(positions_deg):
                    cmd = SetMotorPosition()
                    cmd.name = base_id + i
                    cmd.pos = _deg2rad(deg)
                    cmd.spd = speed
                    cmd.cur = 5.0
                    cmds.append(cmd)

            msg.cmds = cmds
            self._pos_publisher.publish(msg)
            return {"state": "moving", "side": side, "joints": len(cmds)}
        except Exception as e:
            return {"error": str(e)}

    def _send_ctrl(self, side: str, positions_deg: list, kp: list, kd: list) -> dict:
        if not self._ctrl_publisher:
            return {"error": "publisher not initialized"}
        try:
            from bodyctrl_msgs.msg import CmdMotorCtrl, MotorCtrl
            msg = CmdMotorCtrl()
            cmds = []
            sides = []
            if side in ("left", "both"):
                sides.append(("left", 11))
            if side in ("right", "both"):
                sides.append(("right", 21))

            for side_name, base_id in sides:
                for i, deg in enumerate(positions_deg):
                    cmd = MotorCtrl()
                    cmd.name = base_id + i
                    cmd.pos = _deg2rad(deg)
                    cmd.spd = 0.0
                    cmd.tor = 0.0
                    cmd.kp = kp[i] if i < len(kp) else 200.0
                    cmd.kd = kd[i] if i < len(kd) else 20.0
                    cmds.append(cmd)

            msg.cmds = cmds
            self._ctrl_publisher.publish(msg)
            return {"state": "moving", "side": side, "mode": "force_position"}
        except Exception as e:
            return {"error": str(e)}


# ══════════════════════════════════════════════════════════════════════════════
# ArmGesturePlugin (actuator)
# ══════════════════════════════════════════════════════════════════════════════

class ArmGesturePlugin:
    """可取消的挥手、敬礼和欢迎手势序列。"""

    _NEUTRAL = [0, 0, 0, 0, 0, 0, 0]
    # 角度顺序：肩 pitch、肩 roll、肩 yaw、肘 pitch、腕 yaw、腕 pitch、腕 roll。
    _GESTURES = {
        "wave": [0, 55, 0, -85, 0, 0, 0],
        "salute": [-20, 45, -15, -100, 0, 15, 0],
        "welcome": [-20, 50, 0, -45, 0, 0, 0],
        "raise": [0, 45, 0, -20, 0, 0, 0],
    }

    def __init__(self, plugin_config: dict, namespace: str, ros2):
        self._pub_node = Node("tianyi2_arm_gesture_pub", context=ros2.ctx_tianyi)
        ros2.executor_tianyi.add_node(self._pub_node)
        self._publisher = None
        self._sequence = _ActionSequence("ArmGesturePlugin")

    def get_tool(self) -> dict:
        return {
            "name": "arm_gesture",
            "type": "actuator",
            "description": "天轶2.0 手臂语义动作 — 挥手、敬礼、欢迎、举手和回正",
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {
                        "type": "string",
                        "enum": ["wave", "salute", "welcome", "raise", "reset", "stop"],
                        "description": "手臂动作",
                    },
                    "side": {
                        "type": "string", "enum": ["left", "right", "both"],
                        "default": "right", "description": "执行手臂",
                    },
                    "cycles": {
                        "type": "integer", "minimum": 1, "maximum": 5,
                        "default": 2, "description": "挥手循环次数",
                    },
                    "speed": {
                        "type": "number", "minimum": 0.2, "maximum": 1.5,
                        "default": 0.5, "description": "关节速度(rad/s)",
                    },
                },
                "required": ["action"],
                "x-action-params": {
                    "wave": {"params": ["side", "cycles", "speed"], "description": "举手后挥手并回到中性姿态"},
                    "salute": {"params": ["side", "speed"], "description": "执行敬礼姿态并回正"},
                    "welcome": {"params": ["side", "speed"], "description": "执行欢迎展示姿态并回正"},
                    "raise": {"params": ["side", "speed"], "description": "举手并回正"},
                    "reset": {"params": ["side", "speed"], "description": "取消序列并回到中性姿态"},
                    "stop": {"params": [], "description": "取消尚未发送的后续动作帧"},
                },
            },
        }

    def start(self):
        try:
            from bodyctrl_msgs.msg import CmdSetMotorPosition
            self._publisher = self._pub_node.create_publisher(
                CmdSetMotorPosition, "/arm/cmd_pos", _RELIABLE_QOS)
            print("[ArmGesturePlugin] publisher created")
        except ImportError as e:
            print(f"[ArmGesturePlugin] WARNING: msg import failed ({e})")

    def stop(self):
        self._sequence.cancel()

    def dispatch(self, action: str, args: dict) -> dict:
        if action in ("start", "info"):
            return {"state": "ready" if self._publisher else "idle"}
        if action == "stop":
            return {"state": "stopped", "cancelled": self._sequence.cancel()}
        side = args.get("side", "right")
        if side not in ("left", "right", "both"):
            return {"error": "side must be left, right or both"}
        speed = _clamp(args.get("speed", 0.5), 0.2, 1.5)
        if action == "reset":
            self._sequence.cancel()
            return self._publish_pose(side, self._NEUTRAL, speed)
        if action not in self._GESTURES:
            return {"error": f"unknown action: {action}"}
        if not self._publisher:
            return {"error": "publisher not initialized"}

        pose = self._GESTURES[action]
        cycles = int(_clamp(args.get("cycles", 2), 1, 5))
        frames = [(pose, 0.8)]
        if action == "wave":
            for i in range(cycles * 2):
                wave_pose = list(pose)
                wave_pose[4] = 30 if i % 2 == 0 else -30
                frames.append((wave_pose, 0.6))
        frames.append((self._NEUTRAL, 1.0))

        def _worker(cancel_event: threading.Event):
            previous = self._NEUTRAL
            for frame, hold in frames:
                if cancel_event.is_set():
                    return
                result = self._publish_pose(side, frame, speed)
                max_delta_rad = max(
                    abs(_deg2rad(float(current) - float(old)))
                    for current, old in zip(frame, previous)
                )
                transition = max_delta_rad / speed if speed > 0 else 0
                previous = frame
                if "error" in result or cancel_event.wait(transition + hold):
                    return

        self._sequence.start(_worker)
        return {"state": "running", "gesture": action, "side": side,
                "cycles": cycles, "speed": speed}

    def _publish_pose(self, side: str, left_pose: list[float], speed: float) -> dict:
        if not self._publisher:
            return {"error": "publisher not initialized"}
        if len(left_pose) != 7:
            return {"error": "internal pose must have 7 values"}
        try:
            from bodyctrl_msgs.msg import CmdSetMotorPosition, SetMotorPosition
            # Mirror the lateral axes for the right arm. All values remain within
            # the URDF limits used by the existing arm card.
            right_pose = [left_pose[0], -left_pose[1], -left_pose[2],
                          left_pose[3], -left_pose[4], left_pose[5], -left_pose[6]]
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
