#!/usr/bin/env python3
"""
engineai/t800/plugins/sensors.py — T800 开发版传感器转发卡片。

数据流（双 context 转发模式，参照 x-humanoid/tianyi2.0 MotorStatePlugin）:
  domain 69（机器人, CycloneDDS）订阅 →
    缓存最新一帧 → 定时 JSON 序列化 →
  domain 42（core, FastDDS）发布 std_msgs/String, format "data/json":

    /hardware/joint_state   → JointsStatePlugin → /{ns}/state/joints  (2Hz)
    /hardware/imu_info      → ImuPlugin         → /{ns}/state/imu     (10Hz)
    /hardware/power_info    → PowerPlugin       → /{ns}/state/power   (1Hz)
    /hardware/gamepad_keys  → GamepadPlugin     → /{ns}/state/gamepad (10Hz)

订阅 QoS 用 QOS_T800_BEST_EFFORT（官方 BEST_EFFORT/VOLATILE depth 1），
发布 QoS 用 QOS_CORE（core 域 depth 10 BEST_EFFORT），常量与话题表来自
同目录 ros2.py（Ros2Contexts / QOS_* / T800_TOPICS）。

模块级只 import 标准库；rclpy / interface_protocol / ros2 均在 __init__/start()
内延迟导入并 try/except 容错 —— 本机无 ROS2 环境时模块可被纯 import 测试。
"""

from __future__ import annotations

import json
import threading
import time

# ══════════════════════════════════════════════════════════════════════════════
# 常量表（官方索引 / 部位划分）
# ══════════════════════════════════════════════════════════════════════════════

# T800 开发版 25 自由度官方语义名（索引 0~24）
_JOINT_NAMES = [
    "J00_HIP_PITCH_L", "J01_HIP_ROLL_L", "J02_HIP_YAW_L", "J03_KNEE_PITCH_L",
    "J04_ANKLE_PITCH_L", "J05_ANKLE_ROLL_L", "J06_HIP_PITCH_R", "J07_HIP_ROLL_R",
    "J08_HIP_YAW_R", "J09_KNEE_PITCH_R", "J10_ANKLE_PITCH_R", "J11_ANKLE_ROLL_R",
    "J12_TORSO_YAW",
    "J13_SHOULDER_PITCH_L", "J14_SHOULDER_ROLL_L", "J15_SHOULDER_YAW_L",
    "J16_ELBOW_PITCH_L", "J17_ELBOW_YAW_L",
    "J18_SHOULDER_PITCH_R", "J19_SHOULDER_ROLL_R", "J20_SHOULDER_YAW_R",
    "J21_ELBOW_PITCH_R", "J22_ELBOW_YAW_R",
    "J23_HEAD_PITCH", "J24_HEAD_YAW",
]

# 部位划分: 左腿 0-5 / 右腿 6-11 / 腰 12 / 左臂 13-17 / 右臂 18-22 / 头 23-24
_JOINT_PARTS = [
    ("leg_left", [0, 1, 2, 3, 4, 5]),
    ("leg_right", [6, 7, 8, 9, 10, 11]),
    ("waist", [12]),
    ("arm_left", [13, 14, 15, 16, 17]),
    ("arm_right", [18, 19, 20, 21, 22]),
    ("head", [23, 24]),
]

_PART_LABELS = {
    "leg_left": "左腿 (6DOF)",
    "leg_right": "右腿 (6DOF)",
    "waist": "腰 (1DOF)",
    "arm_left": "左臂 (5DOF)",
    "arm_right": "右臂 (5DOF)",
    "head": "头 (2DOF)",
}

# 手柄按键/摇杆语义名（官方 GamepadKeys 常量表）
_GAMEPAD_DIGITAL_KEYS = [
    "LB", "RB", "A", "B", "X", "Y", "BACK", "START",
    "CROSS_X_UP", "CROSS_X_DOWN", "CROSS_Y_LEFT", "CROSS_Y_RIGHT",
]
_GAMEPAD_ANALOG_KEYS = [
    "LT", "RT", "LEFT_STICK_X", "LEFT_STICK_Y", "RIGHT_STICK_X", "RIGHT_STICK_Y",
]


class _JsonSensorBridge:
    """domain 69 订阅 → domain 42 JSON 转发的传感器公共基类。

    子类只需声明 _tool_name/_tool_description/_topics_key/_default_topic/
    _out_suffix/_node_tag/_interval 类属性，并实现 _import_msg_type/_on_msg/_produce。
    无 ROS2 环境时进入 stub 模式（节点/发布者为 None），模块仍可正常导入。
    """

    _format = "data/json"

    def __init__(self, plugin_config: dict, namespace: str, ros2):
        self._plugin_config = plugin_config or {}
        self._ns = namespace
        self._ros2 = ros2
        self._topic = f"/{namespace}/{self._out_suffix}"
        self._running = False
        self._lock = threading.Lock()
        self._latest = None            # 最新一帧解析后的 dict（None = 无数据）
        self._sub_node = None
        self._pub_node = None
        self._pub = None
        self._sub = None
        self._thread = None
        self._String = None
        try:
            from std_msgs.msg import String
            from ros2 import QOS_CORE
            self._String = String
            self._pub_node = ros2.make_node_core(f"t800_{self._node_tag}_pub")
            self._pub = self._pub_node.create_publisher(String, self._topic, QOS_CORE)
            self._sub_node = ros2.make_node_t800(f"t800_{self._node_tag}_sub")
        except Exception as e:  # noqa: BLE001
            print(f"[{self.__class__.__name__}] WARNING: 无 ROS2 环境 ({e})，stub 模式")
            self._String = None
            self._pub_node = None
            self._pub = None
            self._sub_node = None

    def get_tool(self) -> dict:
        return {
            "name": self._tool_name,
            "type": "sensor",
            "multiInstance": False,
            "readOnly": True,
            "description": self._tool_description,
            "inputSchema": {"type": "object", "properties": {}},
            "topic_out": [{"topic": self._topic, "format": self._format}],
        }

    def start(self) -> None:
        """创建订阅并启动转发线程（幂等，可重复调用）。"""
        self._running = True
        if self._sub is None and self._sub_node is not None:
            try:
                from ros2 import QOS_T800_BEST_EFFORT, T800_TOPICS
                msg_cls = self._import_msg_type()
                topic = T800_TOPICS.get(self._topics_key, self._default_topic)
                self._sub = self._sub_node.create_subscription(
                    msg_cls, topic, self._on_msg, QOS_T800_BEST_EFFORT)
                print(f"[{self.__class__.__name__}] 已订阅 {topic}")
            except Exception as e:  # noqa: BLE001
                print(f"[{self.__class__.__name__}] WARNING: 订阅创建失败 ({e})，stub 模式")
        if self._thread is None or not self._thread.is_alive():
            self._thread = threading.Thread(target=self._publish_loop, daemon=True)
            self._thread.start()

    def stop(self) -> None:
        self._running = False

    def _publish_loop(self) -> None:
        """定时把最新一帧数据序列化为 String JSON 发布到 core 域。"""
        while self._running:
            payload = self._produce()
            if payload is not None and self._pub is not None and self._String is not None:
                try:
                    msg = self._String()
                    msg.data = json.dumps(payload, ensure_ascii=False)
                    self._pub.publish(msg)
                except Exception as e:  # noqa: BLE001
                    print(f"[{self.__class__.__name__}] 发布失败: {e}")
            time.sleep(self._interval)

    def dispatch(self, action: str, args: dict) -> dict | None:
        if action == "start":
            self.start()
            return {"state": "running"}
        if action == "stop":
            self.stop()
            return {"state": "idle"}
        if action == "info":
            return {"state": "running" if self._running else "idle",
                    "topic_out": [{"topic": self._topic, "format": self._format}]}
        if action in ("read", "get"):
            payload = self._produce()
            if payload is None:
                return {"state": "error", "error": "NO_FEEDBACK",
                        "message": "尚未收到该传感器数据"}
            return payload
        return {"state": "error", "error": "INVALID_ARGUMENT",
                "message": f"未知 action: {action}"}

    # ── 子类实现 ───────────────────────────────────────────────────────────────
    def _import_msg_type(self):
        """延迟导入并返回 ROS2 消息类型。"""
        raise NotImplementedError

    def _on_msg(self, msg):
        """订阅回调：解析并缓存最新一帧。"""
        raise NotImplementedError

    def _produce(self) -> dict | None:
        """返回要发布的 JSON 字典；无数据返回 None。"""
        raise NotImplementedError


# ══════════════════════════════════════════════════════════════════════════════
# JointsStatePlugin (sensor) — 全身 25 关节状态, tool name="joints", 2Hz
# ══════════════════════════════════════════════════════════════════════════════

class JointsStatePlugin(_JsonSensorBridge):
    """T800 全身 25 关节状态 — 按部位聚合转发 (2Hz)。

    数据源 (domain 69): /hardware/joint_state → interface_protocol/JointState
    发布到 (domain 42): /{ns}/state/joints (std_msgs/String JSON, data/json)
    部位: leg_left(0-5)/leg_right(6-11)/waist(12)/arm_left(13-17)/arm_right(18-22)/head(23-24)
    """
    PREFIX = "joints"
    _tool_name = "joints"
    _out_suffix = "state/joints"
    _topics_key = "joint_state"
    _default_topic = "/hardware/joint_state"
    _node_tag = "joints"
    _interval = 0.5  # 2Hz
    _tool_description = (
        "T800 全身 25 关节状态（按部位聚合, 2Hz）。"
        "部位: leg_left(6DOF)/leg_right(6DOF)/waist(1DOF)/arm_left(5DOF)/arm_right(5DOF)/head(2DOF)。"
        "每关节: name=官方语义名(J00~J24), q=角度(rad), dq=速度(rad/s), tau=力矩(Nm)。"
    )

    def _import_msg_type(self):
        from interface_protocol.msg import JointState
        return JointState

    def _on_msg(self, msg):
        try:
            position = list(getattr(msg, "position", None) or [])
            velocity = list(getattr(msg, "velocity", None) or [])
            torque = list(getattr(msg, "torque", None) or [])
            if not position:
                return  # 空数组视为无数据
            with self._lock:
                self._latest = {
                    "position": position,
                    "velocity": velocity,
                    "torque": torque,
                }
        except Exception as e:  # noqa: BLE001
            print(f"[JointsStatePlugin] 回调解析失败: {e}")

    def _produce(self) -> dict | None:
        with self._lock:
            raw = self._latest
        if raw is None:
            return None
        position = raw["position"]
        velocity = raw["velocity"]
        torque = raw["torque"]
        n = len(position)
        parts = {}
        for part_key, idxs in _JOINT_PARTS:
            joints = []
            for idx in idxs:
                if idx >= n:
                    continue
                joints.append({
                    "idx": idx,
                    "name": _JOINT_NAMES[idx],
                    "q": round(position[idx], 6),
                    "dq": round(velocity[idx], 6) if idx < len(velocity) else 0.0,
                    "tau": round(torque[idx], 6) if idx < len(torque) else 0.0,
                })
            parts[part_key] = {
                "count": len(joints),
                "label": _PART_LABELS[part_key],
                "joints": joints,
            }
        return {"parts": parts, "timestamp_ms": int(time.time() * 1000)}


# ══════════════════════════════════════════════════════════════════════════════
# ImuPlugin (sensor) — 机身 IMU, tool name="imu", 10Hz
# ══════════════════════════════════════════════════════════════════════════════

class ImuPlugin(_JsonSensorBridge):
    """T800 机身 IMU — 四元数/RPY/加速度/角速度转发 (10Hz)。

    数据源 (domain 69): /hardware/imu_info → interface_protocol/ImuInfo
    发布到 (domain 42): /{ns}/state/imu (std_msgs/String JSON, data/json)
    """
    PREFIX = "imu"
    _tool_name = "imu"
    _out_suffix = "state/imu"
    _topics_key = "imu"
    _default_topic = "/hardware/imu_info"
    _node_tag = "imu"
    _interval = 0.1  # 10Hz
    _tool_description = (
        "T800 机身 IMU 姿态与运动信息 (10Hz)。"
        "quaternion=[w,x,y,z], rpy={roll,pitch,yaw}(rad), "
        "linear_acceleration={x,y,z}(m/s²), angular_velocity={x,y,z}(rad/s)。"
    )

    def _import_msg_type(self):
        from interface_protocol.msg import ImuInfo
        return ImuInfo

    def _on_msg(self, msg):
        try:
            payload = {
                "quaternion": [
                    round(msg.quaternion.w, 6), round(msg.quaternion.x, 6),
                    round(msg.quaternion.y, 6), round(msg.quaternion.z, 6),
                ],
                "rpy": {
                    "roll": round(msg.rpy.x, 6),
                    "pitch": round(msg.rpy.y, 6),
                    "yaw": round(msg.rpy.z, 6),
                },
                "linear_acceleration": {
                    "x": round(msg.linear_acceleration.x, 6),
                    "y": round(msg.linear_acceleration.y, 6),
                    "z": round(msg.linear_acceleration.z, 6),
                },
                "angular_velocity": {
                    "x": round(msg.angular_velocity.x, 6),
                    "y": round(msg.angular_velocity.y, 6),
                    "z": round(msg.angular_velocity.z, 6),
                },
            }
            with self._lock:
                self._latest = payload
        except Exception as e:  # noqa: BLE001
            print(f"[ImuPlugin] 回调解析失败: {e}")

    def _produce(self) -> dict | None:
        with self._lock:
            payload = self._latest
        if payload is None:
            return None
        return {**payload, "timestamp_ms": int(time.time() * 1000)}


# ══════════════════════════════════════════════════════════════════════════════
# PowerPlugin (sensor) — 电源状态, tool name="power", 1Hz
# ══════════════════════════════════════════════════════════════════════════════

class PowerPlugin(_JsonSensorBridge):
    """T800 电源状态 — 电量/电压/电流转发 (1Hz)。

    数据源 (domain 69): /hardware/power_info → interface_protocol/PowerInfo
    发布到 (domain 42): /{ns}/state/power (std_msgs/String JSON, data/json)
    """
    PREFIX = "power"
    _tool_name = "power"
    _out_suffix = "state/power"
    _topics_key = "power"
    _default_topic = "/hardware/power_info"
    _node_tag = "power"
    _interval = 1.0  # 1Hz
    _tool_description = (
        "T800 电源状态 (1Hz)。enable=电源使能, percentage=电量(%), "
        "voltage=电压(V), current=电流(A), current_limit=限流(A), error_code=故障码。"
    )

    def _import_msg_type(self):
        from interface_protocol.msg import PowerInfo
        return PowerInfo

    def _on_msg(self, msg):
        try:
            payload = {
                "enable": bool(getattr(msg, "enable", False)),
                "percentage": round(float(getattr(msg, "percentage", 0.0)), 4),
                "voltage": round(float(getattr(msg, "voltage", 0.0)), 4),
                "current": round(float(getattr(msg, "current", 0.0)), 4),
                "current_limit": round(float(getattr(msg, "current_limit", 0.0)), 4),
                "error_code": int(getattr(msg, "error_code", 0)),
            }
            with self._lock:
                self._latest = payload
        except Exception as e:  # noqa: BLE001
            print(f"[PowerPlugin] 回调解析失败: {e}")

    def _produce(self) -> dict | None:
        with self._lock:
            payload = self._latest
        if payload is None:
            return None
        return {**payload, "timestamp_ms": int(time.time() * 1000)}


# ══════════════════════════════════════════════════════════════════════════════
# GamepadPlugin (sensor) — 手柄状态, tool name="gamepad", 10Hz
# ══════════════════════════════════════════════════════════════════════════════

class GamepadPlugin(_JsonSensorBridge):
    """T800 手柄状态 — 数字按键 + 模拟摇杆转发 (10Hz)。

    数据源 (domain 69): /hardware/gamepad_keys → interface_protocol/GamepadKeys
    发布到 (domain 42): /{ns}/state/gamepad (std_msgs/String JSON, data/json)
    """
    PREFIX = "gamepad"
    _tool_name = "gamepad"
    _out_suffix = "state/gamepad"
    _topics_key = "gamepad"
    _default_topic = "/hardware/gamepad_keys"
    _node_tag = "gamepad"
    _interval = 0.1  # 10Hz
    _tool_description = (
        "T800 手柄状态 (10Hz)。digital_states: LB/RB/A/B/X/Y/BACK/START/十字键 (0/1), "
        "analog_states: LT/RT/左右摇杆 x/y (-1~1)。"
    )

    def _import_msg_type(self):
        from interface_protocol.msg import GamepadKeys
        return GamepadKeys

    def _on_msg(self, msg):
        try:
            digital = list(getattr(msg, "digital_states", None) or [])
            analog = list(getattr(msg, "analog_states", None) or [])
            payload = {
                "hardware_connected": bool(getattr(msg, "hardware_connected", False)),
                "digital_states": {
                    key: int(digital[i]) if i < len(digital) else 0
                    for i, key in enumerate(_GAMEPAD_DIGITAL_KEYS)
                },
                "analog_states": {
                    key: round(float(analog[i]), 6) if i < len(analog) else 0.0
                    for i, key in enumerate(_GAMEPAD_ANALOG_KEYS)
                },
            }
            with self._lock:
                self._latest = payload
        except Exception as e:  # noqa: BLE001
            print(f"[GamepadPlugin] 回调解析失败: {e}")

    def _produce(self) -> dict | None:
        with self._lock:
            payload = self._latest
        if payload is None:
            return None
        return {**payload, "timestamp_ms": int(time.time() * 1000)}
