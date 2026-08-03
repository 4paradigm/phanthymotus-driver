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
  ChatPlugin          (actuator)           — 语音交互开关
  VoiceChatActuatorPlugin (actuator)      — 语音对话开关
  MotorStatePlugin    (sensor)             — 全身21电机状态(2Hz)
  HandStatePlugin     (sensor)             — 灵巧手状态(10Hz, tool name=hand_state)
  RemoteStatePlugin   (sensor)             — 遥控器SBUS事件(5Hz)
"""

from __future__ import annotations

import json
import math
import subprocess
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
      - units 字段附加单位说明
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
            payload["units"] = {
                "temp": "°C (MOS 管温度)",
                "current": "A (0=无负载, 合法)",
                "voltage": "V (unknown=未上报/异常)",
                "battery.power": "% (电量)",
                "battery.current": "A (负值=放电)",
            }
            payload["timestamp_ms"] = int(time.time() * 1000)
            payload["control_level"] = "ANY"
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

    # 手指ID: 1=小指, 2=无名指, 3=中指, 4=食指, 5=拇指弯曲, 6=拇指旋转
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
                               "description": "小指 (0=张开, 100=握紧)"},
                    "ring": {"type": "number",
                             "description": "无名指 (0=张开, 100=握紧)"},
                    "middle": {"type": "number",
                               "description": "中指 (0=张开, 100=握紧)"},
                    "index": {"type": "number",
                              "description": "食指 (0=张开, 100=握紧)"},
                    "thumb_bend": {"type": "number",
                                   "description": "拇指弯曲 (0=张开, 100=握紧)"},
                    "thumb_rotation": {"type": "number",
                                       "description": "拇指旋转"},
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
                        "enum": ["play_file", "play_url", "play_text", "stop", "pause", "resume"],
                        "description": "控制模式",
                    },
                    "path": {"type": "string", "description": "本地音频文件绝对路径(play_file)"},
                    "url":  {"type": "string", "description": "远程音频文件URL(play_url, http(s)://)"},
                    "text": {"type": "string", "description": "TTS文本(play_text)"},
                    "force": {"type": "boolean", "description": "强制播放(停止当前任务立即播放,可选)"},
                },
                "required": ["action"],
                "x-action-params": {
                    "play_file": {"params": ["path", "force"], "description": "播放本地音频文件"},
                    "play_url":  {"params": ["url", "force"],  "description": "播放远程URL音频"},
                    "play_text": {"params": ["text", "force"], "description": "TTS合成并播放文本"},
                    "stop":      {"params": [],                 "description": "停止播放(不可恢复)"},
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
                "stop":      ("/audio_play/stop",      PlayStop),
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
            return {"state": "ready", "control_level": "HIGHLEVEL"}
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
            # 等待 service 完成(3s 超时,本地调用,5s 太长)
            rclpy.spin_until_future_complete(self._srv_node, future, timeout_sec=3.0)
            result = future.result()
            if result is None:
                return {"ok": False, "code": "CALL_FAILED",
                        "message": f"{action} service call returned empty (timeout 3s)",
                        "action": action, "timestamp_ms": int(time.time() * 1000)}
            code = int(getattr(result, "code", 0))
            return {
                "ok": code == 0,
                "code": code,
                "message": str(getattr(result, "message", "")),
                "action": action,
                "control_level": "HIGHLEVEL",
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
            return {"state": "ready", "control_level": "HIGHLEVEL"}
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
                "control_level": "HIGHLEVEL",
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
                "head":      self._part(data.get("head"),   "头部(3DOF)"),
                "arm_left":  self._part(arm_left,          "左臂(7DOF)"),
                "arm_right": self._part(arm_right,         "右臂(7DOF)"),
                "waist":     self._part(data.get("waist"), "腰部(2DOF)"),
                "leg":       self._part(data.get("leg"),   "腿部(2DOF)"),
            },
            "units": {
                "q": "关节角度(rad)",
                "dq": "关节速度(rad/s), 未上报则不出现",
                "current": "电流(A), unknown=未上报",
                "temp": "温度(°C), unknown=未上报",
                "error": "故障码, 0=正常, 仅故障时出现",
            },
            "timestamp_ms": int(time.time() * 1000),
            "control_level": "ANY",
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
    1: "小指", 2: "无名指", 3: "中指",
    4: "食指", 5: "拇指弯曲", 6: "拇指旋转",
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
                "天轶2.0 Pro Inspire 灵巧手状态(左右手各6指, 10Hz)。"
                "手指顺序: 1=小指 2=无名指 3=中指 4=食指 5=拇指弯曲 6=拇指旋转。"
                "position: 0=张开 1=握紧(归一化), effort: 电流(A), velocity: 归一化速度。"
                "每指含 position_label 状态标签(fully_open/almost_open/half_closed/almost_closed/fully_closed)。"
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
                    "label": _HAND_FINGER_LABELS.get(fid, f"指{fid}"),
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
            "units": {
                "position": "0=张开, 1=握紧(归一化)",
                "velocity": "归一化速度, 0=静止",
                "effort": "电流(A)",
            },
            "timestamp_ms": int(time.time() * 1000),
            "control_level": "ANY",
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
            "control_level": "ANY",
        }
        if evt is not None:
            out["event"] = evt
        out["units"] = {
            "joystick": "归一化 -1.0 ~ +1.0 (position: center/left/right/forward/back)",
            "buttons": "A-H 8 按键状态值: -1=回弹/下拨, 0=中位, 1=按下/上拨",
        }
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
