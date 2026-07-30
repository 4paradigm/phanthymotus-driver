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
  ArmPlugin        (actuator)           — 双臂14DOF控制
  WaistPlugin      (actuator)           — 腰部2DOF控制
  LegPlugin        (actuator)           — 腿部2DOF控制
  HandPlugin       (actuator)           — 灵巧手控制
  TtsPlugin        (actuator)           — 语音合成
  NavPlugin        (actuator)           — 底盘导航控制
  ChatPlugin       (actuator)           — 语音交互开关
"""

import json
import math
import struct
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

# ── 关节限位 (deg, rpm, A): motor_id → (min_deg, max_deg, max_spd_rpm, rated_current_a) ─

_JOINT_LIMITS = {
    # 头部
    1:  (-26,    26,    64,  5.0),
    2:  (-25,    25,    64,  5.0),
    3:  (-90,    90,    64,  5.0),
    # 左臂
    11: (-170,   170,   88,  35.0),
    12: (-15,    150,   120, 23.0),
    13: (-170,   170,   73,  8.0),
    14: (-150,   15,    73,  8.0),
    15: (-170,   170,   146, 8.0),
    16: (-45,    60,    72,  5.0),
    17: (-95,    75,    72,  5.0),
    # 右臂
    21: (-170,   170,   88,  35.0),
    22: (-150,   15,    120, 23.0),
    23: (-170,   170,   73,  8.0),
    24: (-150,   15,    73,  8.0),
    25: (-170,   170,   146, 8.0),
    26: (-45,    60,    72,  5.0),
    27: (-75,    95,    72,  5.0),
    # 腰部
    31: (-160,   180,   30,  31.0),
    32: (-45,    120,   37.5, 82.0),
    # 左腿
    51: (-40,    5,     37.5, 5.0),
    52: (-23,    20,    37.5, 5.0),
}


def _rpm2rads(rpm: float) -> float:
    return rpm * 2.0 * math.pi / 60.0


def _clamp(val: float, lo: float, hi: float) -> tuple:
    """Clamp value to [lo, hi]; returns (clamped_value, was_clamped)."""
    if val < lo:
        return lo, True
    if val > hi:
        return hi, True
    return val, False


def _resolve_motor_id(identifier, valid_ids: list) -> int | None:
    """将关节标识符（int ID 或 str 名称）解析为 motor ID。"""
    if isinstance(identifier, int):
        return identifier if identifier in valid_ids else None
    if isinstance(identifier, str):
        s = identifier.strip().lower()
        for mid in valid_ids:
            name = _ALL_JOINTS.get(mid, "")
            if s == name or s == str(mid):
                return mid
    return None


def _deg2rad(deg: float) -> float:
    return deg * math.pi / 180.0


def _rad2deg(rad: float) -> float:
    return rad * 180.0 / math.pi


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


class HandStatePlugin(_JsonSensor):
    """Bridge both Inspire hands' feedback and error arrays."""
    def __init__(self, plugin_config, namespace, ros2):
        self._running, self._lock = False, threading.Lock()
        self._state = {"left": {}, "right": {}}
        self._topic = f"/{namespace}/state/hand"
        self._sub_node = Node("tianyi2_hand_state_sub", context=ros2.ctx_tianyi)
        self._pub_node = Node("tianyi2_hand_state_pub", context=ros2.ctx_core)
        ros2.executor_tianyi.add_node(self._sub_node); ros2.executor_core.add_node(self._pub_node)
        self._pub = self._pub_node.create_publisher(String, self._topic, _LOW_LAT_QOS)

    def get_tool(self): return self._tool("hand_state", "天轶2.0 灵巧手状态 — 双手六指位置、速度、电流与错误码")
    def start(self):
        from sensor_msgs.msg import JointState
        from std_msgs.msg import UInt32MultiArray
        self._running = True
        for side in ("left", "right"):
            self._sub_node.create_subscription(JointState, f"/inspire_hand/state/{side}_hand", lambda m, s=side: self._on_state(s, m), _RELIABLE_QOS)
            self._sub_node.create_subscription(UInt32MultiArray, f"/inspire_hand/error/{side}_hand", lambda m, s=side: self._on_error(s, m), _RELIABLE_QOS)
    def stop(self): self._running = False
    def _on_state(self, side, msg):
        with self._lock:
            self._state[side].update({"name": list(msg.name), "position": list(msg.position), "velocity": list(msg.velocity), "effort": list(msg.effort)})
        self._publish()
    def _on_error(self, side, msg):
        with self._lock: self._state[side]["errors"] = list(msg.data)
        self._publish()
    def _publish(self):
        if not self._running: return
        with self._lock: data = json.dumps(self._state)
        out = String(); out.data = data; self._pub.publish(out)


class DepthCameraPlugin:
    """Bridge only the newest Orbbec Z16 frame to Agent Core.

    Depth frames are large and the dashboard only needs the current image.  The
    domain-0 callback therefore never serializes or publishes a frame: it only
    replaces the pending frame.  A domain-42 timer publishes that newest frame,
    so a slow DDS consumer cannot make the input executor drain stale images.
    """
    def __init__(self, plugin_config, namespace, ros2):
        self._running = False; self._topic = f"/{namespace}/camera/head/depth"
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
        latest_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.VOLATILE,
        )
        # The vendor publisher is RELIABLE.  Keep that compatible policy at
        # ingress, but retain only one unread sample instead of the generic
        # ten-message sensor queue.
        ingress_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._running = True
        self._pub = self._pub_node.create_publisher(Image, self._topic, latest_qos)
        self._subscription = self._sub_node.create_subscription(
            Image, "/ob_camera_head/depth/image_raw", self._on_image, ingress_qos)
        # Publish on the domain-42 executor.  30 Hz is a ceiling, not a frame
        # rate target: the timer sends nothing until a newer source frame arrives.
        self._publish_timer = self._pub_node.create_timer(1.0 / 30.0, self._publish_latest)
    def stop(self):
        self._running = False
        with self._latest_image_lock:
            self._latest_image = None

    def _on_image(self, msg):
        if not self._running or msg.encoding not in ("16UC1", "mono16"): return
        # Never block the domain-0 executor on serialization or a slow
        # domain-42 consumer.  Replacing this reference deliberately drops any
        # stale frame that has not yet been published.
        with self._latest_image_lock:
            self._latest_image = msg

    def _publish_latest(self):
        if not self._running:
            return
        with self._latest_image_lock:
            msg = self._latest_image
            self._latest_image = None
        if msg is None:
            return
        # Rebuild the message for the independent domain-42 context.  This is
        # a raw Z16 copy; it does not convert or re-encode the depth image.
        from sensor_msgs.msg import Image
        out = Image()
        out.header = msg.header
        out.height, out.width, out.encoding = msg.height, msg.width, msg.encoding
        out.is_bigendian, out.step, out.data = msg.is_bigendian, msg.step, msg.data
        self._pub.publish(out)
        self._forwarded_frames += 1
        if self._forwarded_frames % 30 == 1:
            print(f"[DepthCameraPlugin] forwarded {self._forwarded_frames} latest Z16 frames")
    def dispatch(self, action, args):
        return {"state": "running" if self._running else "idle", "topic_out": [{"topic": self._topic, "format": "image/depth-z16"}]}


class PointCloudPlugin:
    """Pack and gravity-level Orbbec optical-frame points for Agent Core."""
    _format = "sensor/pointcloud"
    def __init__(self, plugin_config, namespace, ros2):
        self._running = False; self._topic = f"/{namespace}/camera/head/points"; self._last = 0.0; self._intrinsics = None
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
    def _to_renderer_frame(x, y, z, gravity=None):
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
            packed.extend(struct.pack("<fff", *self._to_renderer_frame(x, y, z, gravity)))
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
                packed.extend(struct.pack("<fff", *self._to_renderer_frame(x, y, z, gravity))); count += 1
        if not count: return
        from std_msgs.msg import UInt8MultiArray
        out = UInt8MultiArray(); out.data = list(struct.pack("<II", 12, count) + packed); self._pub.publish(out)
    def dispatch(self, action, args): return {"state": "running" if self._running else "idle", "topic_out": [{"topic": self._topic, "format": self._format}]}


class LightPlugin:
    """Safe semantic system-light control; no raw vendor command is exposed."""
    _commands = {"standby": 99, "service_wait": 20, "service_ready": 22, "warning": 12, "warning_clear": 13, "error": 10, "error_clear": 11}
    def __init__(self, plugin_config, namespace, ros2):
        self._pub_node = Node("tianyi2_light_pub", context=ros2.ctx_tianyi); ros2.executor_tianyi.add_node(self._pub_node); self._pub = None
    def get_tool(self):
        return {"name": "light", "type": "actuator", "description": "天轶2.0 系统状态灯效", "inputSchema": {"type": "object", "properties": {"action": {"type": "string", "enum": list(self._commands)}}, "required": ["action"], "x-action-params": {k: {"params": [], "description": k} for k in self._commands}}}
    def start(self):
        from bodyctrl_msgs.msg import LightCtrl
        self._pub = self._pub_node.create_publisher(LightCtrl, "/xsys/light/ctrl", _RELIABLE_QOS)
    def stop(self): pass
    def dispatch(self, action, args):
        if action not in self._commands: return {"error": f"unknown action: {action}"}
        from bodyctrl_msgs.msg import LightCtrl
        msg = LightCtrl(); msg.cmd = self._commands[action]; msg.caller_id = "phanthy-motus"; msg.caller_msg = f"Agent Core: {action}"; self._pub.publish(msg)
        return {"state": action}


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
# LyreEventPlugin (sensor) — lyre 语音包事件/状态采集
# ══════════════════════════════════════════════════════════════════════════════

class LyreEventPlugin:
    """lyre 语音包事件采集 — 唤醒词/ASR事件/播放事件/播放进度/TTS事件"""

    _ASR_EVENT_NAMES = {2: "error", 3: "state", 4: "wakeup", 5: "sleep",
                        6: "vad", 10: "pre_sleep", 13: "connected", 14: "disconnected"}
    _PLAY_EVENT_NAMES = {0: "started", 1: "completed", 2: "stopped", 3: "cancelled", 4: "failed"}

    def __init__(self, plugin_config: dict, namespace: str, ros2):
        self._ns = namespace
        self._ros2 = ros2
        self._running = False

        self._sub_node = Node("tianyi2_lyre_event_sub", context=ros2.ctx_tianyi)
        ros2.executor_tianyi.add_node(self._sub_node)

        self._pub_node = Node("tianyi2_lyre_event_pub", context=ros2.ctx_core)
        ros2.executor_core.add_node(self._pub_node)

        self._topics = {
            "lyre_keyword": f"/{namespace}/lyre/keyword",
            "lyre_asr_event": f"/{namespace}/lyre/asr_event",
            "lyre_play_event": f"/{namespace}/lyre/play_event",
            "lyre_play_progress": f"/{namespace}/lyre/play_progress",
            "lyre_tts_event": f"/{namespace}/lyre/tts_event",
        }
        self._pubs = {}

    def get_tools(self) -> list:
        return [
            {"name": "lyre_keyword", "type": "sensor",
             "description": "天轶2.0 语音唤醒关键词 — 检测到的唤醒词及声源角度",
             "inputSchema": {"type": "object", "properties": {}},
             "topic_out": [{"topic": self._topics["lyre_keyword"], "format": "data/json"}]},
            {"name": "lyre_asr_event", "type": "sensor",
             "description": "天轶2.0 ASR 状态事件 — 唤醒/休眠/VAD/连接状态",
             "inputSchema": {"type": "object", "properties": {}},
             "topic_out": [{"topic": self._topics["lyre_asr_event"], "format": "data/json"}]},
            {"name": "lyre_play_event", "type": "sensor",
             "description": "天轶2.0 音频播放事件 — 开始/完成/停止/取消/失败",
             "inputSchema": {"type": "object", "properties": {}},
             "topic_out": [{"topic": self._topics["lyre_play_event"], "format": "data/json"}]},
            {"name": "lyre_play_progress", "type": "sensor",
             "description": "天轶2.0 音频播放进度 — 当前位置和总时长(秒)",
             "inputSchema": {"type": "object", "properties": {}},
             "topic_out": [{"topic": self._topics["lyre_play_progress"], "format": "data/json"}]},
            {"name": "lyre_tts_event", "type": "sensor",
             "description": "天轶2.0 TTS 合成事件 — 合成开始/完成/停止/取消/失败",
             "inputSchema": {"type": "object", "properties": {}},
             "topic_out": [{"topic": self._topics["lyre_tts_event"], "format": "data/json"}]},
        ]

    def start(self):
        self._running = True
        for key, topic in self._topics.items():
            self._pubs[key] = self._pub_node.create_publisher(String, topic, _RELIABLE_QOS)

        try:
            from lyre_msgs.msg import AsrKeyword, AsrEvent, PlayEvent, PlayProgress, TtsEvent
            self._sub_node.create_subscription(
                AsrKeyword, "/audio_asr/keyword", self._on_keyword, _RELIABLE_QOS)
            self._sub_node.create_subscription(
                AsrEvent, "/audio_asr/event", self._on_asr_event, _RELIABLE_QOS)
            self._sub_node.create_subscription(
                PlayEvent, "/audio_play/event", self._on_play_event, _RELIABLE_QOS)
            self._sub_node.create_subscription(
                PlayProgress, "/audio_play/progress", self._on_play_progress, _RELIABLE_QOS)
            self._sub_node.create_subscription(
                TtsEvent, "/audio_tts/event", self._on_tts_event, _RELIABLE_QOS)
            print("[LyreEventPlugin] 5 subscriptions created")
        except ImportError:
            print("[LyreEventPlugin] WARNING: lyre_msgs not available, no subscriptions created")

    def stop(self):
        self._running = False

    def _publish(self, key: str, data: dict):
        if not self._running:
            return
        out = String()
        out.data = json.dumps(data)
        self._pubs[key].publish(out)

    def _on_keyword(self, msg):
        self._publish("lyre_keyword", {"keyword": msg.keyword, "angle": msg.angle})

    def _on_asr_event(self, msg):
        self._publish("lyre_asr_event", {
            "event": msg.event,
            "event_name": self._ASR_EVENT_NAMES.get(msg.event, f"unknown_{msg.event}"),
            "arg1": msg.arg1, "arg2": msg.arg2,
        })

    def _on_play_event(self, msg):
        self._publish("lyre_play_event", {
            "sid": msg.sid, "seq": msg.seq,
            "event": msg.event,
            "event_name": self._PLAY_EVENT_NAMES.get(msg.event, f"unknown_{msg.event}"),
            "message": msg.message,
        })

    def _on_play_progress(self, msg):
        self._publish("lyre_play_progress", {
            "sid": msg.sid, "seq": msg.seq,
            "position": msg.position, "duration": msg.duration,
        })

    def _on_tts_event(self, msg):
        self._publish("lyre_tts_event", {
            "sid": msg.sid, "seq": msg.seq,
            "event": msg.event,
            "event_name": self._PLAY_EVENT_NAMES.get(msg.event, f"unknown_{msg.event}"),
            "message": msg.message,
        })

    def dispatch(self, action: str, args: dict) -> dict:
        tool_name = args.get("_tool_name", "lyre_keyword")
        if tool_name not in self._topics:
            return {"error": f"unknown tool: {tool_name}"}
        if action in ("start", "stop", "info"):
            return {"state": "running" if self._running else "idle",
                    "topic_out": [{"topic": self._topics[tool_name], "format": "data/json"}]}
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
# HeadPlugin (actuator)
# ══════════════════════════════════════════════════════════════════════════════

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
# ArmPlugin (actuator)
# ══════════════════════════════════════════════════════════════════════════════

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
# WaistPlugin (actuator)
# ══════════════════════════════════════════════════════════════════════════════

class WaistPlugin:
    """腰部2DOF — move_pos / set_zero (不支持 KP/KD)

    调用格式:
      - 位置控制: {"action": "move_pos", "yaw": 30, "pitch": 10, "speed": 0.5, "current": 10.0}
      - 标零:   {"action": "set_zero"}  (等价于 move_pos yaw=0, pitch=0)
    """

    def __init__(self, plugin_config: dict, namespace: str, ros2):
        self._ns = namespace
        self._ros2 = ros2
        self._pub_node = Node("tianyi2_waist_cmd", context=ros2.ctx_tianyi)
        ros2.executor_tianyi.add_node(self._pub_node)
        self._pub_pos = None

    def get_tool(self) -> dict:
        return {
            "name": "waist",
            "type": "actuator",
            "description": "天轶2.0 腰部控制 — 2DOF (yaw: -160°~180°, pitch: -45°~120°), 位置控制/标零",
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {"type": "string", "enum": ["move_pos", "set_zero"],
                               "description": "控制模式"},
                    "yaw": {"type": "number", "description": "偏航角(度), 范围[-160, 180], 默认0"},
                    "pitch": {"type": "number", "description": "俯仰角(度), 范围[-45, 120], 默认0"},
                    "speed": {"type": "number", "description": "运动速度(rad/s), 默认0.5"},
                    "current": {"type": "number", "description": "最大电流(A), 默认10.0"},
                },
                "required": ["action"],
                "x-action-params": {
                    "move_pos": {"params": ["yaw", "pitch", "speed", "current"],
                                 "description": "位置模式: 移动腰部到指定角度(度)"},
                    "set_zero": {"params": [],
                                 "description": "标零: 等价于 move_pos yaw=0, pitch=0"},
                },
            },
        }

    def start(self):
        try:
            from bodyctrl_msgs.msg import CmdSetMotorPosition
            self._pub_pos  = self._pub_node.create_publisher(CmdSetMotorPosition, "/waist/cmd_pos", _RELIABLE_QOS)
            print("[WaistPlugin] publisher created (/waist/cmd_pos)")
        except ImportError as e:
            print(f"[WaistPlugin] WARNING: {e}")

    def stop(self):
        pass

    def dispatch(self, action: str, args: dict) -> dict:
        if action == "move_pos":
            return self._send_pos(
                args.get("yaw", 0), args.get("pitch", 0),
                args.get("speed", 0.5), args.get("current", 10.0))
        if action == "set_zero":
            return self._send_pos(0, 0)
        if action in ("start", "info"):
            return {"state": "ready"}
        if action == "stop":
            return {"state": "idle"}
        return {"ok": False, "code": "INVALID_ARGUMENT", "message": f"unknown action: {action}"}

    def _send_pos(self, yaw_deg: float, pitch_deg: float, speed_rad_s: float = 0.5, current_a: float = 10.0) -> dict:
        if not self._pub_pos:
            return {"ok": False, "code": "COMMUNICATION_ERROR", "message": "publisher not ready"}
        try:
            from bodyctrl_msgs.msg import CmdSetMotorPosition, SetMotorPosition
            msg = CmdSetMotorPosition()
            results = []
            for mid, deg in [(31, yaw_deg), (32, pitch_deg)]:
                lim = _JOINT_LIMITS[mid]
                pos_deg, clamped = _clamp(deg, lim[0], lim[1])
                # spd 使用 rad/s (修复: 之前错误地把 RPM 当 rad/s 传入)
                max_spd_rads = _rpm2rads(lim[2])
                spd, _ = _clamp(speed_rad_s, 0, max_spd_rads)
                cur, _ = _clamp(current_a, 0, lim[3])
                cmd = SetMotorPosition()
                cmd.name = mid
                cmd.pos = _deg2rad(pos_deg)
                cmd.spd = spd
                cmd.cur = cur
                msg.cmds.append(cmd)
                results.append({"name": _ALL_JOINTS[mid], "pos_deg": pos_deg, "spd_rad_s": spd, "cur_a": cur})
                if clamped:
                    return {"ok": False, "code": "JOINT_LIMIT_VIOLATION",
                            "message": f"waist joint {mid} ({_ALL_JOINTS[mid]}) pos_deg out of range [{lim[0]}°, {lim[1]}°]"}
            self._pub_pos.publish(msg)
            return {"ok": True, "card": "waist", "action": "move_pos", "applied": results}
        except Exception as e:
            return {"ok": False, "code": "COMMUNICATION_ERROR", "message": str(e)}



# ══════════════════════════════════════════════════════════════════════════════
# LegPlugin (actuator)
# ══════════════════════════════════════════════════════════════════════════════

class LegPlugin:
    """腿部2DOF — move_pos / set_zero (不支持 KP/KD)

    调用格式:
      - 位置控制: {"action": "move_pos", "hip": 30, "knee": 60, "speed": 0.5, "current": 10.0}
      - 标零:   {"action": "set_zero"}  (回到归零位姿 hip=5°, knee=-20°)
    """

    def __init__(self, plugin_config: dict, namespace: str, ros2):
        self._ns = namespace
        self._ros2 = ros2
        self._pub_node = Node("tianyi2_leg_cmd", context=ros2.ctx_tianyi)
        ros2.executor_tianyi.add_node(self._pub_node)
        self._pub_pos = None

    def get_tool(self) -> dict:
        return {
            "name": "leg",
            "type": "actuator",
            "description": "天轶2.0 腿部控制 — 2DOF (hip: -40°~5°, knee: -23°~20°), 位置控制/标零",
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {"type": "string", "enum": ["move_pos", "set_zero", "set_height"],
                               "description": "控制模式"},
                    "hip": {"type": "number", "description": "髋关节俯仰角(度), 范围[-40, 5], 默认0"},
                    "knee": {"type": "number", "description": "膝关节俯仰角(度), 范围[-23, 20], 默认0"},
                    "height": {"type": "number", "description": "腿高度(0-100), 0=归零最低, 100=最高"},
                    "speed": {"type": "number", "description": "运动速度(rad/s), 默认0.5"},
                    "current": {"type": "number", "description": "最大电流(A), 默认5.0"},
                },
                "required": ["action"],
                "x-action-params": {
                    "move_pos": {"params": ["hip", "knee", "speed"],
                                 "description": "位置模式: 移动腿部到指定角度(度)"},
                    "set_zero": {"params": [],
                                 "description": "标零: 回到归零位姿 (hip=5°, knee=-20°)"},
                    "set_height": {"params": ["height", "speed", "current"],
                                   "description": "高度模式: 0=归零最低, 100=最高, 线性插值hip/knee"},
                },
            },
        }

    def start(self):
        try:
            from bodyctrl_msgs.msg import CmdSetMotorPosition
            self._pub_pos = self._pub_node.create_publisher(CmdSetMotorPosition, "/leg/cmd_pos", _RELIABLE_QOS)
            print("[LegPlugin] publisher created (/leg/cmd_pos)")
        except ImportError as e:
            print(f"[LegPlugin] WARNING: {e}")

    def stop(self):
        pass

    def dispatch(self, action: str, args: dict) -> dict:
        if action == "move_pos":
            return self._send_pos(
                args.get("hip", 0), args.get("knee", 0),
                args.get("speed", 0.5), args.get("current", 5.0))
        if action == "set_zero":
            return self._send_pos(5.0, -20.0)
        if action == "set_height":
            return self._send_height(
                args.get("height", 0),
                args.get("speed", 0.5), args.get("current", 5.0))
        if action in ("start", "info"):
            return {"state": "ready"}
        if action == "stop":
            return {"state": "idle"}
        return {"ok": False, "code": "INVALID_ARGUMENT", "message": f"unknown action: {action}"}

    # 高度→关节角度映射（实测四点线性插值）
    # height 0 (归零):  hip=5°,  knee=-20°
    # height 100 (最高): hip=-40°, knee=20°
    @staticmethod
    def _height_to_angles(height: float) -> tuple:
        h = max(0.0, min(100.0, height)) / 100.0  # clamp to [0,1]
        hip_deg = 5.0 - 45.0 * h    # 5 → -40
        knee_deg = -20.0 + 40.0 * h  # -20 → 20
        return hip_deg, knee_deg

    def _send_height(self, height: float, speed_rad_s: float = 0.5, current_a: float = 5.0) -> dict:
        hip_deg, knee_deg = self._height_to_angles(height)
        result = self._send_pos(hip_deg, knee_deg, speed_rad_s, current_a)
        result["height"] = max(0.0, min(100.0, height))
        return result

    def _send_pos(self, hip_deg: float, knee_deg: float, speed_rad_s: float = 0.5, current_a: float = 5.0) -> dict:
        if not self._pub_pos:
            return {"ok": False, "code": "COMMUNICATION_ERROR", "message": "publisher not ready"}
        try:
            from bodyctrl_msgs.msg import CmdSetMotorPosition, SetMotorPosition
            msg = CmdSetMotorPosition()
            results = []
            for mid, deg in [(51, hip_deg), (52, knee_deg)]:
                lim = _JOINT_LIMITS[mid]
                pos_deg, clamped = _clamp(deg, lim[0], lim[1])
                max_spd_rads = _rpm2rads(lim[2])
                spd, _ = _clamp(speed_rad_s, 0, max_spd_rads)
                cur, _ = _clamp(current_a, 0, lim[3])
                cmd = SetMotorPosition()
                cmd.name = mid
                cmd.pos = _deg2rad(pos_deg)
                cmd.spd = spd
                cmd.cur = cur
                msg.cmds.append(cmd)
                results.append({"name": _ALL_JOINTS[mid], "pos_deg": pos_deg, "spd_rad_s": spd, "cur_a": cur})
                if clamped:
                    return {"ok": False, "code": "JOINT_LIMIT_VIOLATION",
                            "message": f"leg joint {mid} ({_ALL_JOINTS[mid]}) pos_deg out of range [{lim[0]}°, {lim[1]}°]"}
            self._pub_pos.publish(msg)
            return {"ok": True, "card": "leg", "action": "move_pos", "applied": results}
        except Exception as e:
            return {"ok": False, "code": "COMMUNICATION_ERROR", "message": str(e)}


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


# ══════════════════════════════════════════════════════════════════════════════
# LlmPlugin (actuator + sensor) — LLM 语音对话
# ══════════════════════════════════════════════════════════════════════════════

class LlmPlugin:
    """LLM 语音对话 — 向大模型提问并获取回复"""

    _LLM_EVENT_NAMES = {0: "started", 1: "completed", 2: "stopped", 3: "cancelled", 4: "failed"}

    def __init__(self, plugin_config: dict, namespace: str, ros2):
        self._ns = namespace
        self._ros2 = ros2
        self._running = False

        self._srv_node = Node("tianyi2_llm_srv", context=ros2.ctx_tianyi)
        ros2.executor_tianyi.add_node(self._srv_node)
        self._ask_client = None

        self._sub_node = Node("tianyi2_llm_sub", context=ros2.ctx_tianyi)
        ros2.executor_tianyi.add_node(self._sub_node)
        self._pub_node = Node("tianyi2_llm_pub", context=ros2.ctx_core)
        ros2.executor_core.add_node(self._pub_node)

        self._topic_rst = f"/{namespace}/lyre/llm_rst"
        self._topic_event = f"/{namespace}/lyre/llm_event"
        self._pub_rst = None
        self._pub_event = None

    def get_tools(self) -> list:
        return [
            {"name": "llm_ask", "type": "actuator",
             "description": "天轶2.0 LLM 语音对话 — 向大模型发送文本问题",
             "inputSchema": {
                 "type": "object",
                 "properties": {
                     "action": {"type": "string", "enum": ["ask"], "description": "发送问题"},
                     "text": {"type": "string", "description": "要发送的问题文本"},
                     "id": {"type": "string", "description": "可选标识符，关联 ASR 识别 ID"},
                 },
                 "required": ["action", "text"],
                 "x-action-params": {
                     "ask": {"params": ["text", "id"], "description": "向 LLM 提问"},
                 },
             }},
            {"name": "llm_rst", "type": "sensor",
             "description": "天轶2.0 LLM 语音对话结果 — 大模型返回的文本回复",
             "inputSchema": {"type": "object", "properties": {}},
             "topic_out": [{"topic": self._topic_rst, "format": "data/json"}]},
            {"name": "llm_event", "type": "sensor",
             "description": "天轶2.0 LLM 语音对话事件 — 开始/完成/停止/取消/失败",
             "inputSchema": {"type": "object", "properties": {}},
             "topic_out": [{"topic": self._topic_event, "format": "data/json"}]},
        ]

    def start(self):
        self._running = True
        self._pub_rst = self._pub_node.create_publisher(String, self._topic_rst, _RELIABLE_QOS)
        self._pub_event = self._pub_node.create_publisher(String, self._topic_event, _RELIABLE_QOS)

        try:
            from lyre_msgs.srv import LlmAsk
            from lyre_msgs.msg import LlmRst, LlmEvent
            self._ask_client = self._srv_node.create_client(LlmAsk, "/audio_llm/ask")
            self._sub_node.create_subscription(
                LlmRst, "/audio_llm/rst", self._on_llm_rst, _RELIABLE_QOS)
            self._sub_node.create_subscription(
                LlmEvent, "/audio_llm/event", self._on_llm_event, _RELIABLE_QOS)
            print("[LlmPlugin] service client + 2 subscriptions created")
        except ImportError as e:
            print(f"[LlmPlugin] WARNING: lyre_msgs import failed ({e})")

    def stop(self):
        self._running = False

    def dispatch(self, action: str, args: dict) -> dict:
        tool_name = args.get("_tool_name", "llm_ask")

        if tool_name == "llm_ask":
            if action == "ask":
                text = args.get("text", "")
                qid = args.get("id", "")
                if not text:
                    return {"error": "text is required"}
                return self._ask(text, qid)
            elif action in ("start", "info"):
                return {"state": "ready"}
            return {"error": f"unknown action: {action}"}

        elif tool_name == "llm_rst":
            if action in ("start", "stop", "info"):
                return {"state": "running" if self._running else "idle",
                        "topic_out": [{"topic": self._topic_rst, "format": "data/json"}]}
            return {"state": "running"}

        elif tool_name == "llm_event":
            if action in ("start", "stop", "info"):
                return {"state": "running" if self._running else "idle",
                        "topic_out": [{"topic": self._topic_event, "format": "data/json"}]}
            return {"state": "running"}

        return {"error": f"unknown tool: {tool_name}"}

    def _ask(self, text: str, qid: str) -> dict:
        if not self._ask_client:
            return {"error": "llm_ask service client not initialized"}
        try:
            from lyre_msgs.srv import LlmAsk
            req = LlmAsk.Request()
            req.text = text
            req.id = qid
            future = self._ask_client.call_async(req)
            return {"state": "asking", "text": text[:100]}
        except Exception as e:
            return {"error": str(e)}

    def _on_llm_rst(self, msg):
        if not self._running:
            return
        out = String()
        out.data = json.dumps({"sid": msg.sid, "seq": msg.seq, "last": msg.last, "text": msg.text})
        self._pub_rst.publish(out)

    def _on_llm_event(self, msg):
        if not self._running:
            return
        out = String()
        out.data = json.dumps({
            "sid": msg.sid, "seq": msg.seq,
            "event": msg.event,
            "event_name": self._LLM_EVENT_NAMES.get(msg.event, f"unknown_{msg.event}"),
            "message": msg.message,
        })
        self._pub_event.publish(out)
