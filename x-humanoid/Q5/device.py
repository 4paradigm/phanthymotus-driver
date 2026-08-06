#!/usr/bin/env python3
"""
x-humanoid/Q5/device.py — 星动纪元Q5机器人设备插件。

设计原则：
  - 一个设备 = 一个 tool (或 multi-tool plugin)
  - sensor：只读，驱动启动时自动 start，数据通过 ROS2 topic 输出 (domain 42)
  - actuator：action 参数分发操作，通过 ROS2 发布指令到Q5 (domain 211)
  - resource：返回静态数据 (如 URDF)
  - 角度对外用度(degrees)，内部转弧度(rad)发送

双 Domain 模式：
  - domain 211 (ros2.ctx_q5): 订阅Q5本体话题、发布控制指令
  - domain 42 (ros2.ctx_core): 发布传感器数据给 Agent Core

插件列表：
  SystemStatePlugin   (sensor, multi-tool) — 系统状态/CPU/异常信息
  BatteryPlugin       (sensor)             — 电池状态/电量/充放电状态
  EstopPlugin         (sensor)             — 急停按钮状态
  JointsPlugin        (sensor, multi-tool) — 全身关节状态/IMU
  CameraPlugin        (sensor)             — 头部RGB-D相机
  LidarPlugin         (sensor)             — 360°激光雷达点云
  MicPlugin           (sensor)             — 麦克风音频流
  AsrPlugin           (sensor)             — 语音识别结果
  ChassisPlugin       (actuator)           — 差速底盘运动控制(前后/旋转)
  HeadPlugin          (actuator)           — 头部2DOF控制(yaw/pitch)
  ArmPlugin           (actuator)           — 双臂14DOF控制
  WaistPlugin         (actuator)           — 腰部+腿部关节控制(旋转/升降)
  HandPlugin          (actuator)           — XHAND五指灵巧手控制
  TtsPlugin           (actuator)           — 语音合成/音量/音色控制
  LedPlugin           (actuator)           — 机身指示灯带控制
  ChatPlugin          (actuator)           — 大模型语音对话开关
  ActionPlayerPlugin  (actuator)           — 预设动作/语音回放
  NavPlugin           (actuator)           — 导览导航控制
  ModelPlugin         (resource)           — URDF骨架模型
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
from std_msgs.msg import String, Bool, Float32MultiArray, Int32MultiArray

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

# ── 关节映射 ───────────────────────────────────────────────
_HEAD_JOINTS = {
    1: "neck_yaw_joint",
    2: "neck_pitch_joint",
}

_ARM_LEFT_JOINTS = {
    11: "left_shoulder_pitch_joint",
    12: "left_shoulder_roll_joint",
    13: "left_arm_yaw_joint",
    14: "left_elbow_pitch_joint",
    15: "left_elbow_yaw_joint",
    16: "left_wrist_pitch_joint",
    17: "left_wrist_roll_joint",
}

_ARM_RIGHT_JOINTS = {
    21: "right_shoulder_pitch_joint",
    22: "right_shoulder_roll_joint",
    23: "right_arm_yaw_joint",
    24: "right_elbow_pitch_joint",
    25: "right_elbow_yaw_joint",
    26: "right_wrist_pitch_joint",
    27: "right_wrist_roll_joint",
}

_WAIST_LEG_JOINTS = {
    31: "waist_yaw_joint",
    32: "hip_joint",
    33: "knee_joint",
    34: "ankle_joint",
}

_CHASSIS_JOINTS = {
    41: "left_drv_wheel_joint",
    42: "right_drv_wheel_joint",
}

# 手部关节名（按手册定义）
# XHand Lite: 每手6关节
_HAND_JOINTS_LITE_LEFT = [
    "left_hand_thumb_bend_joint",
    "left_hand_thumb_rota_joint1",
    "left_hand_index_joint1",
    "left_hand_mid_joint1",
    "left_hand_ring_joint1",
    "left_hand_pinky_joint1",
]

_HAND_JOINTS_LITE_RIGHT = [
    "right_hand_thumb_bend_joint",
    "right_hand_thumb_rota_joint1",
    "right_hand_index_joint1",
    "right_hand_mid_joint1",
    "right_hand_ring_joint1",
    "right_hand_pinky_joint1",
]

# XHand1: 每手12关节（Lite基础上+6关节）
_HAND_JOINTS_XHAND1_LEFT = _HAND_JOINTS_LITE_LEFT + [
    "left_hand_thumb_rota_joint2",
    "left_hand_index_bend_joint",
    "left_hand_index_joint2",
    "left_hand_mid_joint2",
    "left_hand_ring_joint2",
    "left_hand_pinky_joint2",
]

_HAND_JOINTS_XHAND1_RIGHT = _HAND_JOINTS_LITE_RIGHT + [
    "right_hand_thumb_rota_joint2",
    "right_hand_index_bend_joint",
    "right_hand_index_joint2",
    "right_hand_mid_joint2",
    "right_hand_ring_joint2",
    "right_hand_pinky_joint2",
]

_HAND_JOINTS_ALL = {
    "lite": {
        "left": _HAND_JOINTS_LITE_LEFT,
        "right": _HAND_JOINTS_LITE_RIGHT,
    },
    "xhand1": {
        "left": _HAND_JOINTS_XHAND1_LEFT,
        "right": _HAND_JOINTS_XHAND1_RIGHT,
    },
}

# 预设手势 → 关节位置(rad)映射 (XHand Lite 6关节, XHand1仅前6关节参与预设)
# 关节顺序: [拇指弯曲, 拇指旋转1, 食指, 中指, 无名指, 小指]
_HAND_GESTURE_POSITIONS = {
    "open_palm":  [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    "fist":       [1.0, 1.0, 1.0, 1.0, 1.0, 1.0],
    "thumbs_up":  [0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
    "point":      [1.0, 0.0, 0.0, 1.0, 1.0, 1.0],
    "ok":         [0.0, 0.0, 0.0, 0.3, 1.0, 1.0],
    "victory":    [1.0, 1.0, 0.0, 0.0, 1.0, 1.0],
    "handshake":  [0.5, 0.5, 0.5, 0.5, 0.5, 0.5],
    "flat":       [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],  # 手掌伸平 = open_palm
}

_HAND_GESTURES = list(_HAND_GESTURE_POSITIONS.keys())


# ══════════════════════════════════════════════════════════════════════════════
# 插件基类
# ══════════════════════════════════════════════════════════════════════════════
class BasePlugin:
    def __init__(self, cfg: dict, namespace: str, ros2):
        self.cfg = cfg
        self.namespace = namespace
        self.ros2 = ros2
        self._running = False
        # 双域节点：sub_node接Q5(domain211)，pub_node发数据到core(domain42)
        self._sub_node = Node(f"q5_{self.__class__.__name__}_sub", context=ros2.ctx_q5)
        self._pub_node = Node(f"q5_{self.__class__.__name__}_pub", context=ros2.ctx_core)
        ros2.executor_q5.add_node(self._sub_node)
        ros2.executor_core.add_node(self._pub_node)

    def start(self):
        self._running = True

    def stop(self):
        self._running = False
        try:
            self._sub_node.destroy_node()
            self._pub_node.destroy_node()
        except Exception:
            pass

    def get_tool(self) -> dict:
        raise NotImplementedError

    def get_tools(self) -> list:
        return [self.get_tool()]

    def dispatch(self, action: str, args: dict) -> dict:
        raise NotImplementedError


# ══════════════════════════════════════════════════════════════════════════════
# SystemStatePlugin 系统状态
# ══════════════════════════════════════════════════════════════════════════════
class SystemStatePlugin(BasePlugin):
    def __init__(self, cfg, namespace, ros2):
        super().__init__(cfg, namespace, ros2)
        self._topic_state = f"/{namespace}/q5/system_state"
        self._topic_exception = f"/{namespace}/q5/exceptions"
        self._state = "INIT"
        self._cpu_usage = 0.0
        self._exceptions = []
        self._heartbeat_ts = 0.0
        self._publish_thread = None

    def get_tools(self) -> list:
        return [
            {
                "name": "system_state",
                "type": "sensor",
                "description": "Q5系统状态 — 机器人状态(INIT/IDLE/READY/ACTIVE/ERROR)、CPU使用率、运行模式",
                "inputSchema": {"type": "object", "properties": {}},
                "topic_out": [{"topic": self._topic_state, "format": "data/json"}],
            },
            {
                "name": "exceptions",
                "type": "sensor",
                "description": "Q5异常信息列表 — 模块/部位/等级/描述/解决方案，等级分为致命/严重/错误/警告/提示/信息",
                "inputSchema": {"type": "object", "properties": {}},
                "topic_out": [{"topic": self._topic_exception, "format": "data/json"}],
            },
        ]

    def start(self):
        super().start()
        self._pub_state = self._pub_node.create_publisher(String, self._topic_state, _RELIABLE_QOS)
        self._pub_exception = self._pub_node.create_publisher(String, self._topic_exception, _RELIABLE_QOS)
        # /xbot_state — Q5 实际系统状态 topic
        self._sub_node.create_subscription(String, "/xbot_state", self._on_xbot_state, _RELIABLE_QOS)
        # /system/heartbeat — Q5 心跳
        self._sub_node.create_subscription(String, "/system/heartbeat", self._on_heartbeat, _RELIABLE_QOS)
        # /system_monitor/status — Q5 系统监控
        self._sub_node.create_subscription(String, "/system_monitor/status", self._on_monitor_status, _RELIABLE_QOS)
        self._publish_thread = threading.Thread(target=self._publish_loop, daemon=True)
        self._publish_thread.start()

    def _on_xbot_state(self, msg):
        try:
            data = json.loads(msg.data)
            self._state = data.get("state", data.get("system_state", "UNKNOWN"))
            self._cpu_usage = float(data.get("cpu_usage", data.get("cpu", 0.0)))
            self._exceptions = data.get("exceptions", data.get("errors", []))
        except Exception:
            pass

    def _on_heartbeat(self, msg):
        self._heartbeat_ts = time.time()

    def _on_monitor_status(self, msg):
        try:
            data = json.loads(msg.data)
            if data.get("state"):
                self._state = data["state"]
            if data.get("cpu_usage") or data.get("cpu"):
                self._cpu_usage = float(data.get("cpu_usage", data.get("cpu", self._cpu_usage)))
            if data.get("exceptions") or data.get("errors"):
                self._exceptions = data.get("exceptions", data.get("errors", []))
        except Exception:
            pass

    def _publish_loop(self):
        while self._running:
            try:
                state_data = json.dumps({
                    "state": self._state,
                    "cpu_usage": self._cpu_usage,
                    "heartbeat_age_s": time.time() - self._heartbeat_ts if self._heartbeat_ts else -1,
                    "timestamp": time.time()
                })
                self._pub_state.publish(String(data=state_data))
                exc_data = json.dumps({"exceptions": self._exceptions, "timestamp": time.time()})
                self._pub_exception.publish(String(data=exc_data))
            except Exception:
                pass
            time.sleep(1.0)

    def dispatch(self, action: str, args: dict) -> dict:
        if action == "query_xbot_state":
            from std_srvs.srv import Trigger
            cli = self._sub_node.create_client(Trigger, "/query_xbot_state")
            if cli.wait_for_service(timeout_sec=3.0):
                future = cli.call_async(Trigger.Request())
                try:
                    rclpy.spin_until_future_complete(self._sub_node, future, timeout_sec=3.0)
                    resp = future.result()
                    if resp:
                        return {"ok": resp.success, "message": getattr(resp, "message", ""),
                                "current_state": self._state, "cpu_usage": self._cpu_usage}
                except Exception:
                    pass
                finally:
                    try:
                        self._sub_node.destroy_client(cli)
                    except Exception:
                        pass
            return {"error": "/query_xbot_state unavailable",
                    "current_state": self._state, "cpu_usage": self._cpu_usage}
        return {"state": self._state, "cpu_usage": self._cpu_usage,
                "exceptions_count": len(self._exceptions),
                "heartbeat_age_s": time.time() - self._heartbeat_ts if self._heartbeat_ts else -1}


# ══════════════════════════════════════════════════════════════════════════════
# BatteryPlugin 电池状态
# ══════════════════════════════════════════════════════════════════════════════
class BatteryPlugin(BasePlugin):
    def __init__(self, cfg, namespace, ros2):
        super().__init__(cfg, namespace, ros2)
        self._topic = f"/{namespace}/q5/battery"
        self._percent = 0
        self._voltage = 0.0
        self._current = 0.0
        self._temperature = 0.0
        self._charge = 0.0
        self._design_capacity = 0.0
        self._status = "UNKNOWN"  # charging/discharging/full
        self._led_color = "blue"

    def get_tool(self) -> dict:
        return {
            "name": "battery",
            "type": "sensor",
            "description": "Q5电池状态 — 电量百分比/电压/电流/充放电状态/灯带颜色/电池参数/固件版本，续航4小时以上，2.5小时快充",
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {
                        "type": "string",
                        "enum": ["status", "get_param", "get_version", "get_electric_state"],
                        "default": "status",
                        "description": "status=获取电池状态, get_param=获取电池参数, get_version=获取固件版本, get_electric_state=获取电源状态",
                    },
                },
                "required": [],
            },
            "topic_out": [{"topic": self._topic, "format": "data/json"}],
        }

    def _on_battery(self, msg):
        self._percent = int(msg.percent)
        self._voltage = float(msg.voltage)
        self._current = float(msg.current)
        self._status = msg.status
        # 对应灯带颜色逻辑
        if self._percent >= 96 and self._status == "charging":
            self._led_color = "green"
        elif self._percent >= 33:
            self._led_color = "blue"
        elif self._percent >= 21:
            self._led_color = "yellow"
        elif self._percent >= 9:
            self._led_color = "orange"
        else:
            self._led_color = "red"

    def _on_battery_str(self, msg):
        try:
            data = json.loads(msg.data)
            self._percent = int(data.get("percent", 0))
            self._voltage = float(data.get("voltage", 0.0))
            self._current = float(data.get("current", 0.0))
            self._status = data.get("status", "UNKNOWN")
        except Exception:
            pass

    def _on_battery_std(self, msg):
        """订阅标准 BatteryState 消息，补全 temperature/charge/design_capacity."""
        self._voltage = float(msg.voltage)
        self._current = float(msg.current)
        self._temperature = float(msg.temperature)
        self._charge = float(msg.charge)
        self._design_capacity = float(msg.design_capacity)
        if msg.power_supply_status == 0:
            self._status = "discharging"
        elif msg.power_supply_status == 1:
            self._status = "charging"
        elif msg.power_supply_status == 2:
            self._status = "full"
        if msg.percentage > 0:
            self._percent = int(msg.percentage * 100)

    def start(self):
        super().start()
        self._pub = self._pub_node.create_publisher(String, self._topic, _RELIABLE_QOS)
        # /battery_state — Q5 标准电池 topic (sensor_msgs/BatteryState)
        try:
            from sensor_msgs.msg import BatteryState
            self._sub_node.create_subscription(BatteryState, "/battery_state", self._on_battery_std, _RELIABLE_QOS)
        except ImportError:
            self._sub_node.create_subscription(String, "/battery_state", self._on_battery_str, _RELIABLE_QOS)
        threading.Thread(target=self._publish_loop, daemon=True).start()

    def _publish_loop(self):
        while self._running:
            try:
                data = json.dumps({
                    "percent": self._percent,
                    "voltage": self._voltage,
                    "current": self._current,
                    "temperature": self._temperature,
                    "charge": self._charge,
                    "design_capacity": self._design_capacity,
                    "status": self._status,
                    "led_color": self._led_color,
                    "timestamp": time.time()
                })
                self._pub.publish(String(data=data))
            except Exception:
                pass
            time.sleep(1.0)

    def _call_svc(self, svc_name, timeout=3.0):
        """Call a std_srvs/Trigger service and return result dict."""
        from std_srvs.srv import Trigger
        cli = self._sub_node.create_client(Trigger, svc_name)
        if not cli.wait_for_service(timeout_sec=timeout):
            return {"error": f"service {svc_name} not available"}
        future = cli.call_async(Trigger.Request())
        try:
            rclpy.spin_until_future_complete(self._sub_node, future, timeout_sec=timeout)
        except Exception:
            return {"error": f"service call to {svc_name} failed"}
        resp = future.result()
        try:
            self._sub_node.destroy_client(cli)
        except Exception:
            pass
        if resp is None:
            return {"error": f"service {svc_name} timed out"}
        return {"ok": resp.success, "message": getattr(resp, "message", "")}

    def dispatch(self, action: str, args: dict) -> dict:
        if action == "get_param":
            return self._call_svc("/get_battery_param")
        if action == "get_version":
            return self._call_svc("/Battery/GetVersion")
        if action == "get_electric_state":
            return self._call_svc("/Battery/ElectricState")
        return {
            "percent": self._percent,
            "voltage": self._voltage,
            "current": self._current,
            "temperature": self._temperature,
            "charge": self._charge,
            "design_capacity": self._design_capacity,
            "status": self._status,
            "led_color": self._led_color
        }


# ══════════════════════════════════════════════════════════════════════════════
# FaultPlugin 关节故障错误码诊断
# ══════════════════════════════════════════════════════════════════════════════
_FAULT_CODES = {
    0x0001: "MC_FOC_DURATION — 关节模组软件错误",
    0x0002: "MC_OVER_VOLT — 关节模组驱动器过压",
    0x0004: "MC_UNDER_VOLT — 关节模组驱动器欠压",
    0x0008: "MC_DRIVE_OVER_TEMP — 关节模组驱动器温度过高",
    0x0010: "MC_MOTOR_OVER_TEMP — 关节模组电机温度过高",
    0x0020: "MC_ENC_ERROR — 关节模组编码器故障",
    0x0040: "MC_BREAK_IN — 关节模组过流",
    0x0080: "MC_SW_ERROR — 关节模组内部状态错误",
    0x0100: "MC_STALL_ERROR — 关节模组堵转",
    0x0200: "MC_ECAT_ERROR — 关节模组EtherCAT通讯错误",
    0x0400: "MC_DRV_ERROR — 关节模组DRV错误",
}


def _decode_fault_mask(mask):
    """解析故障位掩码，返回描述列表"""
    if not mask or mask == 0:
        return []
    results = []
    for bit, desc in _FAULT_CODES.items():
        if mask & bit:
            results.append(desc)
    return results


class FaultPlugin(BasePlugin):
    """订阅关节故障状态，解析错误码位掩码"""

    def get_tool(self) -> dict:
        return {
            "name": "fault",
            "type": "sensor",
            "description": "查询Q5关节模组故障错误码 — 返回所有激活的故障及中文描述",
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {
                        "type": "string",
                        "enum": ["get_fault", "clear"],
                        "default": "get_fault",
                        "description": "get_fault获取当前故障状态，clear尝试清除故障（重置）",
                    },
                    "joint_id": {
                        "type": "number",
                        "description": "指定查询的关节ID（0=全部关节），默认0",
                    },
                },
                "required": ["action"],
            },
        }

    def __init__(self, cfg, namespace, ros2):
        super().__init__(cfg, namespace, ros2)
        self._fault_mask = 0
        self._joint_name = ""

    def dispatch(self, action: str, args: dict) -> dict:
        if action == "get_fault":
            joint_id = args.get("joint_id", 0)
            if joint_id != 0:
                return {"warning": "当前实现返回全部关节聚合故障状态，joint_id参数暂不过滤"}
            faults = _decode_fault_mask(self._fault_mask)
            return {
                "fault_mask": self._fault_mask,
                "joint": self._joint_name or "all",
                "faults": faults,
                "count": len(faults),
                "healthy": len(faults) == 0,
            }
        if action == "clear":
            # 调用 /clear_errors + /clear_hand_sensor 服务
            results = {}
            from std_srvs.srv import Trigger
            for svc in ["/clear_errors", "/clear_hand_sensor"]:
                cli = self._sub_node.create_client(Trigger, svc)
                if cli.wait_for_service(timeout_sec=2.0):
                    future = cli.call_async(Trigger.Request())
                    try:
                        rclpy.spin_until_future_complete(self._sub_node, future, timeout_sec=3.0)
                        resp = future.result()
                        results[svc] = {"ok": resp.success, "message": getattr(resp, "message", "")} if resp else {"error": "timeout"}
                    except Exception:
                        results[svc] = {"error": "call failed"}
                else:
                    results[svc] = {"error": "unavailable"}
                try:
                    self._sub_node.destroy_client(cli)
                except Exception:
                    pass
            return {"ok": True, "results": results}
        return {"error": f"unknown action: {action}"}

    def start(self):
        super().start()
        # /fault_array — Q5 故障列表 topic
        self._sub_node.create_subscription(String, "/fault_array", self._on_fault_array, _LOW_LAT_QOS)
        # /fault_aggregator/highest_level — Q5 最高故障等级
        self._sub_node.create_subscription(String, "/fault_aggregator/highest_level", self._on_highest_fault, _LOW_LAT_QOS)

    def _on_fault_array(self, msg):
        try:
            data = json.loads(msg.data)
            if isinstance(data, list):
                self._fault_mask = sum(data) if data else 0
            elif isinstance(data, dict):
                self._fault_mask = int(data.get("mask", data.get("fault", 0)))
                self._joint_name = data.get("name", data.get("joint_name", ""))
        except Exception:
            pass

    def _on_highest_fault(self, msg):
        try:
            data = json.loads(msg.data)
            level = data.get("level", data.get("highest_level", 0))
            if int(level) > 0 and self._fault_mask == 0:
                self._fault_mask = int(level)
        except Exception:
            pass


# ══════════════════════════════════════════════════════════════════════════════
# EstopPlugin 急停状态
# ══════════════════════════════════════════════════════════════════════════════
class EstopPlugin(BasePlugin):
    def __init__(self, cfg, namespace, ros2):
        super().__init__(cfg, namespace, ros2)
        self._topic = f"/{namespace}/q5/estop"
        self._estop_active = False
        self._estop_source = "unknown"

    def get_tool(self) -> dict:
        return {
            "name": "estop",
            "type": "sensor",
            "description": "Q5急停状态 — 机身背部硬件急停按钮/遥控器急停，按下时切断电机电源。数据来源: /xbot_state + /emergency_service",
            "inputSchema": {"type": "object", "properties": {}},
            "topic_out": [{"topic": self._topic, "format": "data/json"}],
        }

    def start(self):
        super().start()
        self._pub = self._pub_node.create_publisher(String, self._topic, _RELIABLE_QOS)
        # /xbot_state — Q5 实际包含 estop 状态
        self._sub_node.create_subscription(String, "/xbot_state", self._on_xbot_state, _RELIABLE_QOS)
        # /emergency_service — 机器人端急停服务状态
        self._sub_node.create_subscription(String, "/emergency_service", self._on_emergency, _RELIABLE_QOS)
        threading.Thread(target=self._publish_loop, daemon=True).start()

    def _on_xbot_state(self, msg):
        try:
            data = json.loads(msg.data)
            if "estop" in data:
                self._estop_active = bool(data["estop"])
                self._estop_source = "xbot_state"
        except Exception:
            pass

    def _on_emergency(self, msg):
        try:
            data = json.loads(msg.data)
            self._estop_active = data.get("emergency", data.get("active", False))
            self._estop_source = "emergency_service"
        except Exception:
            # Bool 消息
            if hasattr(msg, 'data'):
                self._estop_active = bool(msg.data)
                self._estop_source = "emergency_service"

    def _publish_loop(self):
        while self._running:
            try:
                data = json.dumps({
                    "estop_active": self._estop_active,
                    "source": self._estop_source,
                    "timestamp": time.time()
                })
                self._pub.publish(String(data=data))
            except Exception:
                pass
            time.sleep(0.5)

    def dispatch(self, action: str, args: dict) -> dict:
        return {"estop_active": self._estop_active, "source": self._estop_source}


# ══════════════════════════════════════════════════════════════════════════════
# JointsPlugin 关节状态
# ══════════════════════════════════════════════════════════════════════════════
class JointsPlugin(BasePlugin):
    def __init__(self, cfg, namespace, ros2):
        super().__init__(cfg, namespace, ros2)
        self._topic_joints = f"/{namespace}/q5/joints"
        self._topic_imu = f"/{namespace}/q5/imu"
        self._joint_states = {}
        self._imu_data = {"roll": 0.0, "pitch": 0.0, "yaw": 0.0, "accel": [0,0,9.8], "gyro": [0,0,0]}

    def get_tools(self) -> list:
        return [
            {
                "name": "joints",
                "type": "sensor",
                "description": "Q5全身关节状态 — 位置/速度/扭矩/温度(底盘2轮+双臂14关节+腰/膝/踝/髋+头部2关节+灵巧手10指共46DOF)",
                "inputSchema": {"type": "object", "properties": {}},
                "topic_out": [{"topic": self._topic_joints, "format": "sensor/skeleton"}],
            }
        ]

    def start(self):
        super().start()
        self._pub_joints = self._pub_node.create_publisher(String, self._topic_joints, _LOW_LAT_QOS)
        self._pub_imu = self._pub_node.create_publisher(String, self._topic_imu, _LOW_LAT_QOS)
        try:
            from sensor_msgs.msg import JointState
            self._sub_node.create_subscription(JointState, "/joint_states", self._on_joints, _LOW_LAT_QOS)
            from sensor_msgs.msg import Imu
            self._sub_node.create_subscription(Imu, "/imu/data", self._on_imu, _LOW_LAT_QOS)
        except ImportError:
            self._sub_node.create_subscription(String, "/joint_states", self._on_joints_str, _LOW_LAT_QOS)
            self._sub_node.create_subscription(String, "/imu/data", self._on_imu_str, _LOW_LAT_QOS)
        threading.Thread(target=self._publish_thread, daemon=True).start()

    def _on_joints(self, msg):
        for i, name in enumerate(msg.name):
            pos = math.degrees(msg.position[i]) if i < len(msg.position) else 0.0
            vel = math.degrees(msg.velocity[i]) if i < len(msg.velocity) else 0.0
            eff = msg.effort[i] if i < len(msg.effort) else 0.0
            self._joint_states[name] = {"position": pos, "velocity": vel, "effort": eff}

    def _on_joints_str(self, msg):
        try:
            self._joint_states = json.loads(msg.data)
        except Exception:
            pass

    def _on_imu(self, msg):
        q = msg.orientation
        # 四元数转欧拉角
        sinr_cosp = 2 * (q.w * q.x + q.y * q.z)
        cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y)
        roll = math.degrees(math.atan2(sinr_cosp, cosr_cosp))
        sinp = 2 * (q.w * q.y - q.z * q.x)
        pitch = math.degrees(math.asin(sinp) if abs(sinp) <= 1 else math.copysign(math.pi/2, sinp))
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = math.degrees(math.atan2(siny_cosp, cosy_cosp))
        self._imu_data = {
            "roll": roll, "pitch": pitch, "yaw": yaw,
            "accel": [msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z],
            "gyro": [math.degrees(msg.angular_velocity.x), math.degrees(msg.angular_velocity.y), math.degrees(msg.angular_velocity.z)]
        }

    def _on_imu_str(self, msg):
        try:
            self._imu_data = json.loads(msg.data)
        except Exception:
            pass

    def _publish_thread(self):
        while self._running:
            try:
                joints_data = json.dumps({"joints": self._joint_states, "timestamp": time.time()})
                self._pub_joints.publish(String(data=joints_data))
                imu_data = json.dumps({**self._imu_data, "timestamp": time.time()})
                self._pub_imu.publish(String(data=imu_data))
            except Exception:
                pass
            time.sleep(0.02)  # 50Hz

    def dispatch(self, action: str, args: dict) -> dict:
        return {"joints_count": len(self._joint_states)}


# ══════════════════════════════════════════════════════════════════════════════
# JointState 分类插件 — 共用 JointsPlugin 的关节数据，按部位过滤
# ══════════════════════════════════════════════════════════════════════════════

# ══════════════════════════════════════════════════════════════════════════════
# IMUPlugin IMU姿态数据
# ══════════════════════════════════════════════════════════════════════════════
class IMUPlugin(BasePlugin):
    def __init__(self, cfg, namespace, ros2):
        super().__init__(cfg, namespace, ros2)
        self._topic = f"/{namespace}/q5/imu"
        self._imu_data = {"roll": 0.0, "pitch": 0.0, "yaw": 0.0, "accel": [0,0,9.8], "gyro": [0,0,0]}

    def get_tool(self) -> dict:
        return {
            "name": "imu",
            "type": "sensor",
            "description": "Q5 IMU姿态数据 — 滚转/俯仰/偏航角、加速度、角速度",
            "inputSchema": {"type": "object", "properties": {}},
            "topic_out": [{"topic": self._topic, "format": "data/json"}],
        }

    def start(self):
        super().start()
        self._pub = self._pub_node.create_publisher(String, self._topic, _LOW_LAT_QOS)
        # /dynamic_joint_states — Q5 包含 IMU 数据的动态关节状态 topic
        self._sub_node.create_subscription(String, "/dynamic_joint_states", self._on_dynamic_state, _LOW_LAT_QOS)
        # /xbot_state — 可能包含 IMU 数据
        self._sub_node.create_subscription(String, "/xbot_state", self._on_xbot_imu, _LOW_LAT_QOS)
        threading.Thread(target=self._publish_loop, daemon=True).start()

    def _on_dynamic_state(self, msg):
        try:
            data = json.loads(msg.data)
            if "imu" in data:
                imu = data["imu"]
                self._imu_data = {
                    "roll": float(imu.get("roll", 0)), "pitch": float(imu.get("pitch", 0)),
                    "yaw": float(imu.get("yaw", 0)),
                    "accel": imu.get("accel", [0, 0, 9.8]),
                    "gyro": imu.get("gyro", [0, 0, 0])
                }
        except Exception:
            pass

    def _on_xbot_imu(self, msg):
        try:
            data = json.loads(msg.data)
            if "orientation" in data or "imu" in data:
                imu = data.get("imu", data)
                if "roll" in imu:
                    self._imu_data = {
                        "roll": float(imu.get("roll", 0)), "pitch": float(imu.get("pitch", 0)),
                        "yaw": float(imu.get("yaw", 0)),
                        "accel": imu.get("accel", [0, 0, 9.8]),
                        "gyro": imu.get("gyro", [0, 0, 0])
                    }
        except Exception:
            pass

    def _publish_loop(self):
        while self._running:
            try:
                imu_data = {**self._imu_data, "timestamp": time.time()}
                self._pub.publish(String(data=json.dumps(imu_data)))
            except Exception:
                pass
            time.sleep(0.02)  # 50Hz

    def dispatch(self, action: str, args: dict) -> dict:
        return {"imu": self._imu_data}


# ══════════════════════════════════════════════════════════════════════════════
# TemperaturePlugin 温度传感器 (Q5 /temperature topic)
# ══════════════════════════════════════════════════════════════════════════════
class TemperaturePlugin(BasePlugin):
    def __init__(self, cfg, namespace, ros2):
        super().__init__(cfg, namespace, ros2)
        self._topic = f"/{namespace}/q5/temperature"
        self._temperatures = {}  # joint_name → temp(℃)

    def get_tool(self) -> dict:
        return {
            "name": "temperature",
            "type": "sensor",
            "description": "Q5各关节/电机温度 — 数据来源 /temperature topic，含各部位温度(℃)",
            "inputSchema": {"type": "object", "properties": {}},
            "topic_out": [{"topic": self._topic, "format": "data/json"}],
        }

    def start(self):
        super().start()
        self._pub = self._pub_node.create_publisher(String, self._topic, _RELIABLE_QOS)
        self._sub_node.create_subscription(String, "/temperature", self._on_temperature, _RELIABLE_QOS)
        threading.Thread(target=self._publish_loop, daemon=True).start()

    def _on_temperature(self, msg):
        try:
            data = json.loads(msg.data)
            if isinstance(data, dict):
                self._temperatures = data
        except Exception:
            pass

    def _publish_loop(self):
        while self._running:
            try:
                data = json.dumps({"temperatures": self._temperatures, "timestamp": time.time()})
                self._pub.publish(String(data=data))
            except Exception:
                pass
            time.sleep(1.0)

    def dispatch(self, action: str, args: dict) -> dict:
        return {"temperatures": self._temperatures, "count": len(self._temperatures)}


# ══════════════════════════════════════════════════════════════════════════════
# 传感器插件（相机/雷达/麦克风/ASR）实现模板，对接对应SDK即可
# ══════════════════════════════════════════════════════════════════════════════
class CameraPlugin(BasePlugin):
    def __init__(self, cfg, namespace, ros2):
        super().__init__(cfg, namespace, ros2)
        self._color_topic = f"/camera/camera/color/image_raw"
        self._depth_topic = f"/camera/camera/depth/image_rect_raw"
        self._color_info_topic = f"/camera/camera/color/camera_info"
        self._depth_info_topic = f"/camera/camera/depth/camera_info"
        self._color_data = None
        self._depth_data = None
        self._color_info = {}
        self._depth_info = {}

    def get_tool(self) -> dict:
        return {
            "name": "camera_head",
            "type": "sensor",
            "description": "Q5头部RGB-D深度相机 — 彩色+深度图像流 + 相机内参/外参",
            "inputSchema": {"type": "object", "properties": {
                "action": {"type": "string", "enum": ["get_color", "get_depth", "get_color_info", "get_depth_info"],
                           "default": "get_color",
                           "description": "get_color=获取彩色图像(base64 JPEG)，get_depth=获取深度图像(raw)，get_color_info=获取彩色相机内参，get_depth_info=获取深度相机内参"},
            }},
            "topic_out": [{"topic": f"/{self.namespace}/q5/camera/color", "format": "image/jpeg"},
                          {"topic": f"/{self.namespace}/q5/camera/depth", "format": "image/raw"}],
        }

    def start(self):
        super().start()
        self._color_pub = self._pub_node.create_publisher(String,
            f"/{self.namespace}/q5/camera/color", _LOW_LAT_QOS)
        self._depth_pub = self._pub_node.create_publisher(String,
            f"/{self.namespace}/q5/camera/depth", _LOW_LAT_QOS)

        from sensor_msgs.msg import Image
        from std_msgs.msg import Header
        self._sub_node.create_subscription(Image, self._color_topic, self._on_color, _LOW_LAT_QOS)
        self._sub_node.create_subscription(Image, self._depth_topic, self._on_depth, _LOW_LAT_QOS)

        try:
            from sensor_msgs.msg import CameraInfo
            self._sub_node.create_subscription(CameraInfo, self._color_info_topic, self._on_color_info, _LOW_LAT_QOS)
            self._sub_node.create_subscription(CameraInfo, self._depth_info_topic, self._on_depth_info, _LOW_LAT_QOS)
        except ImportError:
            pass

        import threading
        threading.Thread(target=self._publish_loop, daemon=True).start()

    def _on_color(self, msg):
        try:
            import cv2
            import numpy as np
            step = msg.step if hasattr(msg, 'step') else msg.data.shape[1] if len(msg.data.shape) > 1 else msg.data.shape[0]
            arr = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(msg.height, msg.width, -1) if len(msg.data) > 0 else None
            if arr is not None and arr.shape[2] >= 3:
                ok, buf = cv2.imencode('.jpg', arr[:, :, 0:3])
                if ok:
                    self._color_data = buf.tobytes()
        except Exception:
            pass

    def _on_depth(self, msg):
        try:
            import numpy as np
            arr = np.frombuffer(bytes(msg.data), dtype=np.uint16).reshape(msg.height, msg.width) if len(msg.data) > 0 else None
            if arr is not None:
                self._depth_data = arr.tobytes()
        except Exception:
            pass

    def _on_color_info(self, msg):
        self._color_info = {
            "width": msg.width,
            "height": msg.height,
            "K": list(msg.k),
            "D": list(msg.d),
            "R": list(msg.r),
            "P": list(msg.p),
            "distortion_model": msg.distortion_model,
        }

    def _on_depth_info(self, msg):
        self._depth_info = {
            "width": msg.width,
            "height": msg.height,
            "K": list(msg.k),
            "D": list(msg.d),
            "R": list(msg.r),
            "P": list(msg.p),
            "distortion_model": msg.distortion_model,
        }

    def _publish_loop(self):
        import base64
        while self._running:
            try:
                if self._color_data:
                    self._color_pub.publish(String(data=base64.b64encode(self._color_data).decode('ascii')))
                if self._depth_data:
                    self._depth_pub.publish(String(data=base64.b64encode(self._depth_data).decode('ascii')))
            except Exception:
                pass
            time.sleep(0.1)  # 10Hz

    def dispatch(self, action: str, args: dict) -> dict:
        import base64
        if action == "get_color":
            if self._color_data is None:
                return {"error": "no color image received yet"}
            return {"ok": True, "format": "jpeg", "image": base64.b64encode(self._color_data).decode('ascii')}

        if action == "get_depth":
            if self._depth_data is None:
                return {"error": "no depth image received yet"}
            return {"ok": True, "format": "uint16_raw", "image": base64.b64encode(self._depth_data).decode('ascii')}

        if action == "get_color_info":
            if not self._color_info:
                return {"error": "no color camera info received yet"}
            return {"ok": True, **self._color_info}

        if action == "get_depth_info":
            if not self._depth_info:
                return {"error": "no depth camera info received yet"}
            return {"ok": True, **self._depth_info}

        return {"error": f"unknown action: {action}"}


class LidarPlugin(BasePlugin):
    def __init__(self, cfg, namespace, ros2):
        super().__init__(cfg, namespace, ros2)
        self._points = []

    def get_tool(self) -> dict:
        return {
            "name": "lidar",
            "type": "sensor",
            "description": "Q5 360°混合固态激光雷达点云数据 — 用于避障/导航",
            "inputSchema": {"type": "object", "properties": {}},
            "topic_out": [{"topic": f"/{self.namespace}/q5/lidar/points", "format": "pointcloud/pcd"}],
        }

    def start(self):
        super().start()
        self._pub = self._pub_node.create_publisher(String, f"/{self.namespace}/q5/lidar/points", _LOW_LAT_QOS)
        # /slam/map_cmap — Q5 激光雷达点云/slam地图 topic
        self._sub_node.create_subscription(String, "/slam/map_cmap", self._on_points_str, _LOW_LAT_QOS)

    def _on_points(self, msg):
        import struct
        import base64
        try:
            raw = bytes(msg.data)
            # 发布 base64 编码的点云数据到 Agent Core
            self._pub.publish(String(data=base64.b64encode(raw).decode('ascii')))
            self._points = [len(raw)]  # 只记录大小
        except Exception:
            pass

    def _on_points_str(self, msg):
        self._pub.publish(msg)  # 直通

    def dispatch(self, action: str, args: dict) -> dict:
        return {"state": "running", "last_points_size": sum(self._points)}


class MicPlugin(BasePlugin):
    """麦克风 — Q5 无 /audio/mic ROS2 topic，使用 sounddevice 直访声卡（见 AudioPlugin）。"""

    def __init__(self, cfg, namespace, ros2):
        super().__init__(cfg, namespace, ros2)
        self._audio_frames = []

    def get_tool(self) -> dict:
        return {
            "name": "mic",
            "type": "sensor",
            "description": "Q5麦克风阵列音频流 — 通过 sounddevice 直访声卡（无ROS2 topic）。录音请用 audio 卡片",
            "inputSchema": {"type": "object", "properties": {}},
            "topic_out": [{"topic": f"/{self.namespace}/q5/audio/mic", "format": "audio/pcm"}],
        }

    def start(self):
        super().start()
        self._pub = self._pub_node.create_publisher(String, f"/{self.namespace}/q5/audio/mic", _LOW_LAT_QOS)

    def dispatch(self, action: str, args: dict) -> dict:
        return {"state": "running", "note": "Q5麦克风通过 sounddevice 直访，录音请用 audio 卡片"}


class AsrPlugin(BasePlugin):
    def get_tool(self) -> dict:
        return {
            "name": "asr",
            "type": "sensor",
            "description": "Q5语音识别结果 — 实时语音转文字",
            "inputSchema": {"type": "object", "properties": {}},
            "topic_out": [{"topic": f"/{self.namespace}/q5/asr/result", "format": "data/json"}],
        }

    def start(self):
        super().start()
        self._pub = self._pub_node.create_publisher(String, f"/{self.namespace}/q5/asr/result", _RELIABLE_QOS)
        # /speech/sentence_topic — Q5 语音识别结果 topic
        self._sub_node.create_subscription(String, "/speech/sentence_topic", self._on_asr, _RELIABLE_QOS)

    def _on_asr(self, msg):
        try:
            data = json.loads(msg.data) if isinstance(msg.data, str) else {"text": str(msg.data)}
        except Exception:
            data = {"text": str(msg.data)}
        self._pub.publish(String(data=json.dumps({"text": data.get("text", ""), "timestamp": time.time()})))

    def dispatch(self, action: str, args: dict) -> dict:
        return {"state": "running"}


# ══════════════════════════════════════════════════════════════════════════════
# ChassisPlugin 底盘控制
# ══════════════════════════════════════════════════════════════════════════════
class ChassisPlugin(BasePlugin):
    def __init__(self, cfg, namespace, ros2):
        super().__init__(cfg, namespace, ros2)
        self._pub = None
        self._max_speed = 1.5  # m/s 最大前进速度

    def get_tool(self) -> dict:
        return {
            "name": "chassis",
            "type": "actuator",
            "description": "Q5差速底盘控制 — 前后移动、原地旋转、速度调节，最大速度1.5m/s",
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {"type": "string", "enum": ["move", "rotate", "stop", "set_speed"],
                               "description": "控制动作"},
                    "vx": {"type": "number", "description": "前后速度(m/s)，正=前进，负=后退，范围[-1.5, 1.5]"},
                    "vyaw": {"type": "number", "description": "旋转速度(rad/s)，正=逆时针，范围[-3.14, 3.14]"},
                    "direction": {"type": "string", "enum": ["forward", "backward", "left", "right", "stop"],
                                  "description": "预设移动方向"},
                    "speed": {"type": "number", "description": "速度比例(0-1)，默认0.5"},
                },
                "required": ["action"],
                "x-action-params": {
                    "move": {"params": ["vx", "vyaw"], "description": "按速度控制底盘运动"},
                    "rotate": {"params": ["angle", "speed"], "description": "原地旋转指定角度(度)"},
                    "stop": {"params": [], "description": "立即停止底盘运动"},
                    "set_speed": {"params": ["speed"], "description": "设置默认移动速度比例(0-1)"},
                },
            },
        }

    def start(self):
        super().start()
        try:
            from q5_msgs.msg import ChassisCmd
            self._pub = self._pub_node.create_publisher(ChassisCmd, "/chassis/cmd_vel", _LOW_LAT_QOS)
        except ImportError:
            self._pub = self._pub_node.create_publisher(Float32MultiArray, "/chassis/cmd_vel", _LOW_LAT_QOS)

        # 同时发布到本体底驱控制器话题
        from geometry_msgs.msg import TwistStamped
        self._pub_twist = self._pub_node.create_publisher(TwistStamped, "/wr1_base_drive_controller/cmd_vel", _LOW_LAT_QOS)

    def dispatch(self, action: str, args: dict) -> dict:
        if not self._pub:
            return {"error": "publisher not initialized"}
        vx = float(args.get("vx", 0.0))
        vyaw = float(args.get("vyaw", 0.0))
        speed_ratio = float(args.get("speed", 0.5))
        direction = args.get("direction", "")

        if action == "stop":
            vx = 0.0
            vyaw = 0.0
        elif action == "move" and direction:
            if direction == "forward":
                vx = self._max_speed * speed_ratio
            elif direction == "backward":
                vx = -self._max_speed * speed_ratio
            elif direction == "left":
                vyaw = 1.5 * speed_ratio
            elif direction == "right":
                vyaw = -1.5 * speed_ratio
            elif direction == "stop":
                vx = 0.0
                vyaw = 0.0
        elif action == "set_speed":
            self._max_speed = 1.5 * max(0.0, min(1.0, speed_ratio))
            return {"ok": True, "max_speed": self._max_speed}

        # 发送控制指令
        try:
            from q5_msgs.msg import ChassisCmd
            cmd = ChassisCmd()
            cmd.vx = vx
            cmd.vyaw = vyaw
            self._pub.publish(cmd)
        except ImportError:
            cmd = Float32MultiArray(data=[vx, 0.0, vyaw])
            self._pub.publish(cmd)

        # 同时发布 TwistStamped 到本体底驱控制器
        from geometry_msgs.msg import TwistStamped
        from std_msgs.msg import Header
        twist_stamped = TwistStamped()
        twist_stamped.header = Header()
        twist_stamped.header.stamp = self._pub_node.get_clock().now().to_msg()
        twist_stamped.header.frame_id = "base_link"
        twist_stamped.twist.linear.x = vx
        twist_stamped.twist.angular.z = vyaw
        self._pub_twist.publish(twist_stamped)

        return {"ok": True, "vx": vx, "vyaw": vyaw}


# ══════════════════════════════════════════════════════════════════════════════
# HeadPlugin 头部控制 → /wr1_controller/commands (HybridJointCommand)
# ══════════════════════════════════════════════════════════════════════════════
class HeadPlugin(BasePlugin):
    def __init__(self, cfg, namespace, ros2):
        super().__init__(cfg, namespace, ros2)
        self._pub = None
        # 头部关节限位（度）
        self._limits = {"yaw": (-60, 60), "pitch": (-30, 30)}

    def get_tool(self) -> dict:
        return {
            "name": "head",
            "type": "actuator",
            "description": "Q5头部2DOF控制 — yaw(neck_yaw)左右±60°, pitch(neck_pitch)上下±30°。当前仅位置模式，velocity/feedforward/kp/kd 不生效",
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {"type": "string", "enum": ["move_pos", "look_at", "reset"],
                               "description": "控制动作"},
                    "yaw": {"type": "number", "description": "偏航角(度), 左正右负, 范围[-60, 60]"},
                    "pitch": {"type": "number", "description": "俯仰角(度), 下正上负, 范围[-30, 30]"},
                    "target": {"type": "string", "enum": ["forward", "left", "right", "up", "down"],
                               "description": "预设方向"},
                },
                "required": ["action"],
                "x-action-params": {
                    "move_pos": {"params": ["yaw", "pitch"], "description": "移动头部到指定角度(度)"},
                    "look_at": {"params": ["target"], "description": "看向预设方向"},
                    "reset": {"params": [], "description": "头部回到正视前方零位"},
                },
            },
        }

    def start(self):
        super().start()
        from xbot_common_interfaces.msg import HybridJointCommand
        from std_msgs.msg import Header
        self._pub = self._pub_node.create_publisher(HybridJointCommand, "/wr1_controller/commands", _LOW_LAT_QOS)
        self._HybridJointCommand = HybridJointCommand
        self._Header = Header

    def _build_cmd(self, yaw_rad, pitch_rad):
        cmd = self._HybridJointCommand()
        cmd.header = self._Header()
        cmd.header.stamp = self._pub_node.get_clock().now().to_msg()
        cmd.header.frame_id = "base_link"
        cmd.joint_name = ["neck_yaw_joint", "neck_pitch_joint"]
        cmd.position = [float(yaw_rad), float(pitch_rad)]
        cmd.velocity = [0.3, 0.3]
        cmd.feedforward = [0.0, 0.0]
        cmd.kp = [50.0, 50.0]
        cmd.kd = [10.0, 10.0]
        return cmd

    def dispatch(self, action: str, args: dict) -> dict:
        if not self._pub:
            return {"error": "publisher not initialized"}
        yaw = float(args.get("yaw", 0.0))
        pitch = float(args.get("pitch", 0.0))
        target = args.get("target", "")

        if action == "reset":
            yaw = 0.0
            pitch = 0.0
        elif action == "look_at":
            if target == "left":
                yaw = -45
            elif target == "right":
                yaw = 45
            elif target == "up":
                pitch = -20
            elif target == "down":
                pitch = 20
            else:  # forward
                yaw = 0
                pitch = 0

        # 限位
        yaw = max(self._limits["yaw"][0], min(self._limits["yaw"][1], yaw))
        pitch = max(self._limits["pitch"][0], min(self._limits["pitch"][1], pitch))

        cmd = self._build_cmd(math.radians(yaw), math.radians(pitch))
        self._pub.publish(cmd)
        return {"ok": True, "yaw": yaw, "pitch": pitch}


# ══════════════════════════════════════════════════════════════════════════════
# ArmPlugin 双臂控制 → /wr1_controller/commands (HybridJointCommand)
# ══════════════════════════════════════════════════════════════════════════════
class ArmPlugin(BasePlugin):
    def __init__(self, cfg, namespace, ros2):
        super().__init__(cfg, namespace, ros2)
        self._pub = None
        self._joint_limits = (-170, 170)  # 大部分关节角度限位（度）

    def get_tool(self) -> dict:
        return {
            "name": "arm",
            "type": "actuator",
            "description": "Q5双臂14DOF控制 — 左右臂各7关节(肩pitch/roll/yaw、肘pitch/yaw、腕pitch/roll)，单臂额定负载2kg，最大5kg。当前仅位置模式，velocity/feedforward/kp/kd 不生效",
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {"type": "string", "enum": ["move_pos", "reset", "home"],
                               "default": "move_pos",
                               "description": "move_pos=控制关节角度，reset=双臂回到零位，home=标定当前位置为零位"},
                    "left_positions": {
                        "type": "array", "items": {"type": "number", "minimum": -170, "maximum": 170},
                        "minItems": 7, "maxItems": 7,
                        "default": [0, 0, 0, 0, 0, 0, 0],
                        "description": "左臂7关节角度(度): [肩pitch, 肩roll, 肩yaw, 肘pitch, 肘yaw, 腕pitch, 腕roll]"
                    },
                    "right_positions": {
                        "type": "array", "items": {"type": "number", "minimum": -170, "maximum": 170},
                        "minItems": 7, "maxItems": 7,
                        "default": [0, 0, 0, 0, 0, 0, 0],
                        "description": "右臂7关节角度(度)，顺序同左臂；镜像左臂姿态时肩roll/肩yaw/腕yaw/腕roll取反"
                    },
                    "velocity": {"type": "number", "default": 0.3,
                                 "description": "关节目标速度(rad/s)，默认0.3（当前仅位置模式，不生效）"},
                    "feedforward": {"type": "number", "default": 0.0,
                                    "description": "前馈力矩(N·m)，默认0.0（当前仅位置模式，不生效）"},
                    "kp": {"type": "number", "default": 85.0,
                           "description": "位置比例增益，默认85.0（当前仅位置模式，不生效）"},
                    "kd": {"type": "number", "default": 20.0,
                           "description": "速度微分增益，默认20.0（当前仅位置模式，不生效）"},
                },
                "required": ["action"],
                "x-action-params": {
                    "move_pos": {"params": ["left_positions", "right_positions", "velocity", "feedforward", "kp", "kd"],
                                 "description": "控制双臂各关节到指定角度(度)"},
                    "reset": {"params": [], "description": "双臂回到初始零位"},
                    "home": {"params": [], "description": "标定当前位置为双臂零位（调用 /set_zero_pos + /set_custom_home_position）"},
                },
            },
        }

    def start(self):
        super().start()
        from xbot_common_interfaces.msg import HybridJointCommand
        from std_msgs.msg import Header
        self._pub = self._pub_node.create_publisher(HybridJointCommand, "/wr1_controller/commands", _LOW_LAT_QOS)
        self._HybridJointCommand = HybridJointCommand
        self._Header = Header

    def _build_cmd(self, joint_names: list, positions_rad: list, velocity, feedforward, kp, kd):
        cmd = self._HybridJointCommand()
        cmd.header = self._Header()
        cmd.header.stamp = self._pub_node.get_clock().now().to_msg()
        cmd.header.frame_id = "base_link"
        cmd.joint_name = joint_names
        n = len(joint_names)
        cmd.position = [float(p) for p in positions_rad]
        cmd.velocity = [float(velocity)] * n
        cmd.feedforward = [float(feedforward)] * n
        cmd.kp = [float(kp)] * n
        cmd.kd = [float(kd)] * n
        return cmd

    def dispatch(self, action: str, args: dict) -> dict:
        if not self._pub:
            return {"error": "publisher not initialized"}

        velocity = float(args.get("velocity", 0.3))
        feedforward = float(args.get("feedforward", 0.0))
        kp = float(args.get("kp", 85.0))
        kd = float(args.get("kd", 20.0))

        if action == "home":
            # 调用 /set_zero_pos + /set_custom_home_position 标定当前位置为零位
            results = {}
            from std_srvs.srv import Trigger
            for svc in ["/set_zero_pos", "/set_custom_home_position"]:
                cli = self._sub_node.create_client(Trigger, svc)
                if cli.wait_for_service(timeout_sec=3.0):
                    future = cli.call_async(Trigger.Request())
                    try:
                        rclpy.spin_until_future_complete(self._sub_node, future, timeout_sec=5.0)
                        resp = future.result()
                        results[svc] = {"ok": resp.success} if resp else {"error": "timeout"}
                    except Exception:
                        results[svc] = {"error": "call failed"}
                else:
                    results[svc] = {"error": "unavailable"}
                try:
                    self._sub_node.destroy_client(cli)
                except Exception:
                    pass
            return {"ok": True, "action": "home", "results": results}
        elif action == "reset":
            left = [0.0] * 7
            right = [0.0] * 7
        else:
            left = [math.radians(max(self._joint_limits[0], min(self._joint_limits[1], float(x))))
                    for x in args.get("left_positions", [0] * 7)]
            right = [math.radians(max(self._joint_limits[0], min(self._joint_limits[1], float(x))))
                     for x in args.get("right_positions", [0] * 7)]

        # 合并关节名和位置
        joint_names = list(_ARM_LEFT_JOINTS.values()) + list(_ARM_RIGHT_JOINTS.values())
        positions_rad = left + right

        cmd = self._build_cmd(joint_names, positions_rad, velocity, feedforward, kp, kd)
        self._pub.publish(cmd)

        return {"ok": True,
                "left_positions": [math.degrees(p) for p in left],
                "right_positions": [math.degrees(p) for p in right],
                "velocity": velocity, "kp": kp, "kd": kd}


# ══════════════════════════════════════════════════════════════════════════════
# ArmServoPlugin 双臂笛卡尔位姿控制 (MPC ServoPose)
# ══════════════════════════════════════════════════════════════════════════════
class ArmServoPlugin(BasePlugin):
    def __init__(self, cfg, namespace, ros2):
        super().__init__(cfg, namespace, ros2)
        self._pub = None
        self._sub = None
        self._current_pose = {
            "left_pose": {"position": {"x": 0, "y": 0, "z": 0}, "orientation": {"x": 0, "y": 0, "z": 0, "w": 1}},
            "right_pose": {"position": {"x": 0, "y": 0, "z": 0}, "orientation": {"x": 0, "y": 0, "z": 0, "w": 1}},
            "head_pose": {"position": {"x": 0, "y": 0, "z": 0}, "orientation": {"x": 0, "y": 0, "z": 0, "w": 1}},
        }
        self._pose_has_data = False  # True when /get_pose has delivered real data

    def get_tool(self) -> dict:
        return {
            "name": "arm_servo",
            "type": "actuator",
            "description": (
                "Q5双臂笛卡尔位姿控制 (MPC) — 通过ServoPose控制左右手末端XYZ+四元数，框架base_link，内部MPC逆解算。"
                "使用前请先完成关节初始化并通过 mpc_controller.start_mpc 启动MPC算法；"
                "请先用 get_pose 读取当前位姿，以小增量逐步逼近目标，避免一次性发送大跨度的位姿变换，否则可能导致机械臂不稳定或触发安全保护。"
            ),
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {"type": "string", "enum": ["move_pose", "get_pose"],
                               "default": "move_pose",
                               "description": "move_pose=控制末端位姿（自动从小步长插值逼近），get_pose=读取当前位姿"},
                    "left_position": {
                        "type": "array", "items": {"type": "number"}, "minItems": 3, "maxItems": 3,
                        "default": [0.257, -0.346, 1.412],
                        "description": "左臂末端位置 [x, y, z] (m)，相对于base_link。请以当前位姿为起点，小增量逐步调整"
                    },
                    "left_orientation": {
                        "type": "array", "items": {"type": "number"}, "minItems": 4, "maxItems": 4,
                        "default": [0.612, -0.016, 0.79, -0.006],
                        "description": "左臂末端四元数 [x, y, z, w]"
                    },
                    "right_position": {
                        "type": "array", "items": {"type": "number"}, "minItems": 3, "maxItems": 3,
                        "default": [-0.018, -0.209, 0.778],
                        "description": "右臂末端位置 [x, y, z] (m)，相对于base_link。请以当前位姿为起点，小增量逐步调整"
                    },
                    "right_orientation": {
                        "type": "array", "items": {"type": "number"}, "minItems": 4, "maxItems": 4,
                        "default": [0.996, -0.011, 0.071, 0.046],
                        "description": "右臂末端四元数 [x, y, z, w]"
                    },
                    "head_position": {
                        "type": "array", "items": {"type": "number"}, "minItems": 3, "maxItems": 3,
                        "default": [-0.101, 0.0, 1.411],
                        "description": "头部末端位置 [x, y, z] (m)，相对于base_link"
                    },
                    "head_orientation": {
                        "type": "array", "items": {"type": "number"}, "minItems": 4, "maxItems": 4,
                        "default": [0.0, 0.0, 0.0, 1.0],
                        "description": "头部末端四元数 [x, y, z, w]"
                    },
                    "frame_id": {"type": "string", "default": "base_link",
                                 "description": "位姿参考坐标系，默认base_link"},
                    "rate": {"type": "number", "minimum": 1, "maximum": 200,
                             "default": 50, "description": "发布频率(Hz)，默认50"},
                    "max_step": {"type": "number", "default": 0.05,
                                 "description": "单步最大位移(m)，默认0.05。目标较远时自动分步插值逼近"},
                },
                "required": ["action"],
                "x-action-params": {
                    "move_pose": {"params": ["left_position", "left_orientation", "right_position", "right_orientation", "head_position", "head_orientation", "frame_id", "rate", "max_step"],
                                  "description": "发布ServoPose到/servo_poses，自动从当前位姿插值逼近目标"},
                    "get_pose": {"params": [], "description": "读取当前左右手+头部Cartesian位姿"},
                },
            },
        }

    def start(self):
        super().start()
        try:
            from xbot_common_interfaces.msg import ServoPose
            self._pub = self._pub_node.create_publisher(ServoPose, "/servo_poses", _LOW_LAT_QOS)
            self._sub = self._sub_node.create_subscription(ServoPose, "/get_pose", self._on_get_pose, _LOW_LAT_QOS)
        except ImportError:
            print("[arm_servo] ServoPose not available, using Float64MultiArray fallback")
            self._pub = self._pub_node.create_publisher(Float32MultiArray, "/servo_poses", _LOW_LAT_QOS)

    def _on_get_pose(self, msg):
        self._current_pose = {
            "left_pose": {
                "position": {"x": msg.left_pose.pose.position.x, "y": msg.left_pose.pose.position.y, "z": msg.left_pose.pose.position.z},
                "orientation": {"x": msg.left_pose.pose.orientation.x, "y": msg.left_pose.pose.orientation.y,
                                "z": msg.left_pose.pose.orientation.z, "w": msg.left_pose.pose.orientation.w},
            },
            "right_pose": {
                "position": {"x": msg.right_pose.pose.position.x, "y": msg.right_pose.pose.position.y, "z": msg.right_pose.pose.position.z},
                "orientation": {"x": msg.right_pose.pose.orientation.x, "y": msg.right_pose.pose.orientation.y,
                                "z": msg.right_pose.pose.orientation.z, "w": msg.right_pose.pose.orientation.w},
            },
            "head_pose": {
                "position": {"x": msg.head_pose.pose.position.x, "y": msg.head_pose.pose.position.y, "z": msg.head_pose.pose.position.z},
                "orientation": {"x": msg.head_pose.pose.orientation.x, "y": msg.head_pose.pose.orientation.y,
                                "z": msg.head_pose.pose.orientation.z, "w": msg.head_pose.pose.orientation.w},
            },
        }
        self._pose_has_data = True

    def dispatch(self, action: str, args: dict) -> dict:
        if action == "get_pose":
            return {"pose": self._current_pose, "has_data": self._pose_has_data}

        # move_pose — 前置检查 MPC 是否已启动
        try:
            from std_srvs.srv import Trigger
            mpc_ok = False
            mpc_cli = self._sub_node.create_client(Trigger, "/mpc/status")
            if mpc_cli.wait_for_service(timeout_sec=2):
                future = mpc_cli.call_async(Trigger.Request())
                import time as _time
                start = _time.time()
                while not future.done() and (_time.time() - start) < 3.0:
                    _time.sleep(0.05)
                if future.done() and future.result() and future.result().success:
                    mpc_ok = True
            try:
                self._sub_node.destroy_client(mpc_cli)
            except Exception:
                pass
            if not mpc_ok:
                print("[arm_servo] WARN: MPC 可能未启动，请先调用 mpc_controller.start_mpc。"
                      "位姿指令可能不会生效。")
        except Exception:
            pass

        # move_pose — 发布 ServoPose，自动从当前位姿插值逼近
        frame_id = args.get("frame_id", "base_link")
        rate_hz = int(args.get("rate", 50))
        max_step = float(args.get("max_step", 0.05))
        import threading

        # 解析目标位姿
        def _parse_pose(pos_key, ori_key):
            pos = [float(args.get(pos_key, [0, 0, 0])[i]) for i in range(3)]
            ori = [float(args.get(ori_key, [0, 0, 0, 1])[i]) for i in range(4)]
            return pos, ori

        target_left_pos, target_left_ori = _parse_pose("left_position", "left_orientation")
        target_right_pos, target_right_ori = _parse_pose("right_position", "right_orientation")
        target_head_pos, target_head_ori = _parse_pose("head_position", "head_orientation")

        # 检查是否已获取过当前位姿
        if not self._pose_has_data:
            print("[arm_servo] WARN: 尚未收到 /get_pose 数据，无法插值，将直接发送目标位姿。"
                  "请先调用 get_pose 确认位姿数据可用。")

        # 计算插值步数
        def _distance(a, b):
            return math.sqrt(sum((x - y) ** 2 for x, y in zip(a, b)))

        max_dist = max(
            _distance(target_left_pos,
                      [self._current_pose["left_pose"]["position"][k] for k in ["x", "y", "z"]])
            if self._pose_has_data else 0,
            _distance(target_right_pos,
                      [self._current_pose["right_pose"]["position"][k] for k in ["x", "y", "z"]])
            if self._pose_has_data else 0,
            _distance(target_head_pos,
                      [self._current_pose["head_pose"]["position"][k] for k in ["x", "y", "z"]])
            if self._pose_has_data else 0,
        )
        steps = max(1, int(math.ceil(max_dist / max_step))) if self._pose_has_data and max_dist > max_step else 1

        if steps > 1:
            print(f"[arm_servo] 目标距离 {max_dist:.3f}m > {max_step}m, 自动分 {steps} 步插值逼近")

        def _lerp(a, b, t):
            return [x + (y - x) * t for x, y in zip(a, b)]

        try:
            from xbot_common_interfaces.msg import ServoPose
            from geometry_msgs.msg import PoseStamped
            from rclpy.rate import Rate

            def _build_servo(pos_left, ori_left, pos_right, ori_right, pos_head, ori_head):
                servo = ServoPose()
                for limb, pos, ori in [("left", pos_left, ori_left),
                                       ("right", pos_right, ori_right),
                                       ("head", pos_head, ori_head)]:
                    ps = PoseStamped()
                    ps.header.frame_id = frame_id
                    ps.header.stamp = self._pub_node.get_clock().now().to_msg()
                    ps.pose.position.x = float(pos[0])
                    ps.pose.position.y = float(pos[1])
                    ps.pose.position.z = float(pos[2])
                    ps.pose.orientation.x = float(ori[0])
                    ps.pose.orientation.y = float(ori[1])
                    ps.pose.orientation.z = float(ori[2])
                    ps.pose.orientation.w = float(ori[3])
                    setattr(servo, f"{limb}_pose", ps)
                return servo

            # 起点：如果收到过位姿数据就用当前位姿，否则用目标（相当于直发）
            if self._pose_has_data:
                start_left_pos = [self._current_pose["left_pose"]["position"][k] for k in ["x", "y", "z"]]
                start_left_ori = [self._current_pose["left_pose"]["orientation"][k] for k in ["x", "y", "z", "w"]]
                start_right_pos = [self._current_pose["right_pose"]["position"][k] for k in ["x", "y", "z"]]
                start_right_ori = [self._current_pose["right_pose"]["orientation"][k] for k in ["x", "y", "z", "w"]]
                start_head_pos = [self._current_pose["head_pose"]["position"][k] for k in ["x", "y", "z"]]
                start_head_ori = [self._current_pose["head_pose"]["orientation"][k] for k in ["x", "y", "z", "w"]]
            else:
                start_left_pos = target_left_pos
                start_left_ori = target_left_ori
                start_right_pos = target_right_pos
                start_right_ori = target_right_ori
                start_head_pos = target_head_pos
                start_head_ori = target_head_ori

            def publish_loop():
                rate = Rate(frequency=rate_hz)
                for step_idx in range(steps):
                    if not self._running:
                        break
                    t_val = (step_idx + 1) / steps
                    cur_left_pos = _lerp(start_left_pos, target_left_pos, t_val)
                    cur_left_ori = start_left_ori if step_idx == steps - 1 else _lerp(start_left_ori, target_left_ori, t_val)
                    cur_right_pos = _lerp(start_right_pos, target_right_pos, t_val)
                    cur_right_ori = start_right_ori if step_idx == steps - 1 else _lerp(start_right_ori, target_right_ori, t_val)
                    cur_head_pos = _lerp(start_head_pos, target_head_pos, t_val)
                    cur_head_ori = start_head_ori if step_idx == steps - 1 else _lerp(start_head_ori, target_head_ori, t_val)

                    servo = _build_servo(cur_left_pos, cur_left_ori,
                                         cur_right_pos, cur_right_ori,
                                         cur_head_pos, cur_head_ori)

                    # 每步发送多帧让 MPC 收敛
                    frames_per_step = max(1, rate_hz // steps) if steps > 1 else 10
                    for _ in range(frames_per_step):
                        if not self._running:
                            break
                        try:
                            self._pub.publish(servo)
                        except Exception:
                            pass
                        rate.sleep()
            t = threading.Thread(target=publish_loop, daemon=True)
            t.start()
            return {"ok": True, "rate": rate_hz, "frame_id": frame_id,
                    "interpolated": steps > 1, "steps": steps, "max_step": max_step}

        except ImportError:
            # Fallback: Float64MultiArray (pos3 + ori4 = 7 per limb)
            from std_msgs.msg import Float64MultiArray
            from rclpy.rate import Rate

            def _build_f64(pos_left, ori_left, pos_right, ori_right, pos_head, ori_head):
                data = [0.0] * 21
                for base, pos, ori in [(0, pos_left, ori_left),
                                       (7, pos_right, ori_right),
                                       (14, pos_head, ori_head)]:
                    for i, v in enumerate(pos):
                        data[base + i] = v
                    for i, v in enumerate(ori):
                        data[base + 3 + i] = v
                return data

            def publish_loop():
                rate = Rate(frequency=rate_hz)
                for step_idx in range(steps):
                    if not self._running:
                        break
                    t_val = (step_idx + 1) / steps
                    cur_left_pos = _lerp(start_left_pos, target_left_pos, t_val)
                    cur_left_ori = start_left_ori if step_idx == steps - 1 else _lerp(start_left_ori, target_left_ori, t_val)
                    cur_right_pos = _lerp(start_right_pos, target_right_pos, t_val)
                    cur_right_ori = start_right_ori if step_idx == steps - 1 else _lerp(start_right_ori, target_right_ori, t_val)
                    cur_head_pos = _lerp(start_head_pos, target_head_pos, t_val)
                    cur_head_ori = start_head_ori if step_idx == steps - 1 else _lerp(start_head_ori, target_head_ori, t_val)

                    data = _build_f64(cur_left_pos, cur_left_ori,
                                     cur_right_pos, cur_right_ori,
                                     cur_head_pos, cur_head_ori)
                    frames_per_step = max(1, rate_hz // steps) if steps > 1 else 10
                    for _ in range(frames_per_step):
                        if not self._running:
                            break
                        try:
                            self._pub.publish(Float64MultiArray(data=data))
                        except Exception:
                            pass
                        rate.sleep()
            t = threading.Thread(target=publish_loop, daemon=True)
            t.start()
            return {"ok": True, "rate": rate_hz,
                    "interpolated": steps > 1, "steps": steps, "max_step": max_step}



# ══════════════════════════════════════════════════════════════════════════════
# WaistPlugin 腰/腿控制 → /wr1_controller/commands (HybridJointCommand)
# ══════════════════════════════════════════════════════════════════════════════
class WaistPlugin(BasePlugin):
    def get_tool(self) -> dict:
        return {
            "name": "waist",
            "type": "actuator",
            "description": "Q5腰部旋转+腿部升降控制 — waist_yaw±120°, 腿部hip/knee/ankle升降。当前仅位置模式，velocity 不生效",
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {"type": "string", "enum": ["move_waist", "move_height", "set_zero"],
                               "description": "控制模式"},
                    "yaw": {"type": "number", "description": "腰偏航角(度), 范围[-120, 120], 默认0"},
                    "height": {"type": "number", "description": "腿部升降高度(0-100), 0=最低, 100=最高, 默认0"},
                    "velocity": {"type": "number", "default": 0.3, "description": "关节目标速度(rad/s)（当前仅位置模式，不生效）"},
                },
                "required": ["action"],
                "x-action-params": {
                    "move_waist": {"params": ["yaw", "velocity"], "description": "腰部旋转到指定角度"},
                    "move_height": {"params": ["height", "velocity"], "description": "腿部升降到指定高度"},
                    "set_zero": {"params": [], "description": "腰部回正+腿部降到最低"},
                },
            },
        }

    def start(self):
        super().start()
        from xbot_common_interfaces.msg import HybridJointCommand
        from std_msgs.msg import Header
        self._pub = self._pub_node.create_publisher(HybridJointCommand, "/wr1_controller/commands", _LOW_LAT_QOS)
        self._HybridJointCommand = HybridJointCommand
        self._Header = Header

    def dispatch(self, action: str, args: dict) -> dict:
        yaw = math.radians(max(-120, min(120, float(args.get("yaw", 0.0)))))
        height_frac = max(0, min(100, float(args.get("height", 0.0)))) / 100.0
        velocity = float(args.get("velocity", 0.3))

        if action == "set_zero":
            yaw = 0.0
            height_frac = 0.0

        # waist_yaw + hip/knee/ankle 位置 (hip/knee/ankle 联动升降)
        hip_pos = height_frac * 1.2       # 髋关节 ~1.2rad 范围
        knee_pos = height_frac * (-1.5)   # 膝关节反向
        ankle_pos = height_frac * 0.6     # 踝关节微调

        cmd = self._HybridJointCommand()
        cmd.header = self._Header()
        cmd.header.stamp = self._pub_node.get_clock().now().to_msg()
        cmd.header.frame_id = "base_link"
        cmd.joint_name = ["waist_yaw_joint", "hip_joint", "knee_joint", "ankle_joint"]
        cmd.position = [float(yaw), float(hip_pos), float(knee_pos), float(ankle_pos)]
        cmd.velocity = [float(velocity)] * 4
        cmd.feedforward = [0.0] * 4
        cmd.kp = [80.0] * 4
        cmd.kd = [15.0] * 4
        self._pub.publish(cmd)

        return {"ok": True, "yaw": math.degrees(yaw), "height_pct": height_frac * 100}


# ══════════════════════════════════════════════════════════════════════════════
# HandPlugin 灵巧手控制
# ══════════════════════════════════════════════════════════════════════════════
class HandPlugin(BasePlugin):
    """XHAND灵巧手控制 — 通过 HybridJointCommand 发布到 /hand_controller/commands.

    兼容 XHand Lite (每手6关节) 和 XHand1 (每手12关节)。
    预设手势基于前6关节，XHand1的额外6关节保持0位置。
    """

    def __init__(self, cfg, namespace, ros2):
        super().__init__(cfg, namespace, ros2)
        self._hand_model = cfg.get("hand_model", "lite")  # "lite" 或 "xhand1"

    def get_tool(self) -> dict:
        return {
            "name": "hand",
            "type": "actuator",
            "description": "Q5 XHAND五指灵巧手控制 — 预设手势+逐指控制(rad)，通过 /hand_controller/commands 发布。当前仅位置模式，velocity/feedforward/kp/kd 不生效",
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {"type": "string", "enum": _HAND_GESTURES + ["set_fingers_raw", "reset", "pause_ee_retarget", "start_ee_retarget"],
                               "description": "预设手势(open_palm/fist/thumbs_up/point/ok/victory/handshake/flat) | set_fingers_raw=逐指控制(rad) | reset=张开回零 | pause/start_ee_retarget=暂停/恢复末端重定向"},
                    "side": {"type": "string", "enum": ["left", "right", "both"], "default": "right",
                             "description": "控制哪只手"},
                    "positions": {
                        "type": "array", "items": {"type": "number"},
                        "minItems": 6, "maxItems": 6,
                        "description": "逐指控制时6指目标位置(rad): [拇指弯曲, 拇指旋转, 食指, 中指, 无名指, 小指]"
                    },
                    "velocity": {"type": "number", "default": 0.3, "description": "关节目标速度(rad/s)（当前仅位置模式，不生效）"},
                    "feedforward": {"type": "number", "default": 350.0, "description": "前馈力矩(N·m)，灵巧手建议350（当前仅位置模式，不生效）"},
                    "kp": {"type": "number", "default": 100.0, "description": "位置比例增益（当前仅位置模式，不生效）"},
                    "kd": {"type": "number", "default": 0.0, "description": "速度微分增益（当前仅位置模式，不生效）"},
                },
                "required": ["action"],
                "x-action-params": {
                    **{g: {"params": ["side", "velocity", "feedforward", "kp", "kd"],
                           "description": f"预设手势: {g}"} for g in _HAND_GESTURES},
                    "set_fingers_raw": {"params": ["side", "positions", "velocity", "feedforward", "kp", "kd"],
                                       "description": "逐指精确控制(rad)，6指关节"},
                    "reset": {"params": ["side"], "description": "手指全部张开回零位"},
                },
            },
        }

    def start(self):
        super().start()
        from std_msgs.msg import Header
        from xbot_common_interfaces.msg import HybridJointCommand
        self._pub = self._pub_node.create_publisher(HybridJointCommand, "/hand_controller/commands", _LOW_LAT_QOS)
        self._HybridJointCommand = HybridJointCommand
        self._Header = Header

    def _build_cmd(self, side: str, positions: list, velocity: float, feedforward: float, kp: float, kd: float):
        """构建 HybridJointCommand，单次控制单手或双手."""
        model = self._hand_model
        joints_map = _HAND_JOINTS_ALL.get(model, _HAND_JOINTS_ALL["lite"])

        joint_names = []
        all_positions = []
        n = len(positions)  # 6 for preset gestures

        if side in ("left", "both"):
            left_joints = joints_map["left"]
            joint_names.extend(left_joints)
            all_positions.extend(positions)
            # XHand1 额外关节补零
            if model == "xhand1" and len(left_joints) > n:
                all_positions.extend([0.0] * (len(left_joints) - n))

        if side in ("right", "both"):
            right_joints = joints_map["right"]
            joint_names.extend(right_joints)
            all_positions.extend(positions)
            if model == "xhand1" and len(right_joints) > n:
                all_positions.extend([0.0] * (len(right_joints) - n))

        m = len(joint_names)
        cmd = self._HybridJointCommand()
        cmd.header = self._Header()
        cmd.header.stamp = self._pub_node.get_clock().now().to_msg()
        cmd.header.frame_id = "base_link"
        cmd.joint_name = joint_names
        cmd.position = [float(p) for p in all_positions]
        cmd.velocity = [float(velocity)] * m
        cmd.feedforward = [float(feedforward)] * m
        cmd.kp = [float(kp)] * m
        cmd.kd = [float(kd)] * m
        return cmd

    def dispatch(self, action: str, args: dict) -> dict:
        side = args.get("side", "right")
        velocity = float(args.get("velocity", 0.3))
        feedforward = float(args.get("feedforward", 350.0))
        kp = float(args.get("kp", 100.0))
        kd = float(args.get("kd", 0.0))

        if action in ("pause_ee_retarget", "start_ee_retarget"):
            from std_srvs.srv import Trigger
            svc_name = "/Pause_EE_Retarget" if action == "pause_ee_retarget" else "/Start_EE_Retarget"
            cli = self._sub_node.create_client(Trigger, svc_name)
            if not cli.wait_for_service(timeout_sec=3.0):
                return {"error": f"service {svc_name} not available"}
            future = cli.call_async(Trigger.Request())
            try:
                rclpy.spin_until_future_complete(self._sub_node, future, timeout_sec=3.0)
                resp = future.result()
                result = {"ok": resp.success, "message": getattr(resp, "message", "")} if resp else {"error": "timeout"}
            except Exception:
                result = {"error": "call failed"}
            try:
                self._sub_node.destroy_client(cli)
            except Exception:
                pass
            return {"action": action, **result}
        elif action in _HAND_GESTURE_POSITIONS:
            positions = list(_HAND_GESTURE_POSITIONS[action])
        elif action == "set_fingers_raw":
            positions = [float(x) for x in args.get("positions", [0.0] * 6)]
            if len(positions) != 6:
                return {"error": "positions 需要6个值: [拇指弯曲, 拇指旋转, 食指, 中指, 无名指, 小指]"}
        elif action == "reset":
            positions = [0.0] * 6
        else:
            return {"error": f"unknown action: {action}"}

        cmd = self._build_cmd(side, positions, velocity, feedforward, kp, kd)
        self._pub.publish(cmd)
        return {"ok": True, "action": action, "side": side, "positions": positions,
                "hand_model": self._hand_model}


# ══════════════════════════════════════════════════════════════════════════════
# HandLowlevelPlugin 灵巧手底层关节控制
# 手动指定关节名（完全控制），按手册定义
# ══════════════════════════════════════════════════════════════════════════════
class HandLowlevelPlugin(BasePlugin):
    """灵巧手底层 HybridJointCommand 控制 — 支持完整关节名列表，兼容 Lite/XHand1."""

    def __init__(self, cfg, namespace, ros2):
        super().__init__(cfg, namespace, ros2)
        self._hand_model = cfg.get("hand_model", "lite")

    def get_tool(self) -> dict:
        return {
            "name": "hand_low",
            "type": "actuator",
            "description": "Q5灵巧手底层关节控制 — 通过 HybridJointCommand 直接控制每指位置(rad)。当前仅位置模式，速度/前馈力矩/增益 均不生效",
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {"type": "string", "enum": ["set_joint", "reset"],
                               "description": "set_joint=设置关节目标值, reset=复位到零位"},
                    "side": {"type": "string", "enum": ["left", "right", "both"], "default": "right",
                             "description": "控制哪只手"},
                    "joint_names": {"type": "array", "items": {"type": "string"},
                                    "description": "自定义关节名列表，不填则根据 hand_model 自动选择"},
                    "positions": {"type": "array", "items": {"type": "number"},
                                  "description": "各关节目标位置(rad)，数量需与 joint_names 一致"},
                    "velocities": {"type": "array", "items": {"type": "number"},
                                   "description": "各关节目标速度(rad/s)，默认0.0（当前仅位置模式，不生效）"},
                    "feedforward": {"type": "array", "items": {"type": "number"},
                                    "description": "各关节前馈力矩(N·m)，默认350.0（当前仅位置模式，不生效）"},
                    "kp": {"type": "array", "items": {"type": "number"},
                           "description": "各关节位置比例增益，默认100.0（当前仅位置模式，不生效）"},
                    "kd": {"type": "array", "items": {"type": "number"},
                           "description": "各关节速度微分增益，默认0.0（当前仅位置模式，不生效）"},
                },
                "required": ["action"],
            },
        }

    def start(self):
        super().start()
        from std_msgs.msg import Header
        from xbot_common_interfaces.msg import HybridJointCommand
        self._pub = self._pub_node.create_publisher(HybridJointCommand, "/hand_controller/commands", _LOW_LAT_QOS)
        self._HybridJointCommand = HybridJointCommand
        self._Header = Header

    def _resolve_joints(self, side: str):
        """根据 hand_model 和 side 返回关节名列表."""
        joints_map = _HAND_JOINTS_ALL.get(self._hand_model, _HAND_JOINTS_ALL["lite"])
        joint_names = []
        if side in ("left", "both"):
            joint_names.extend(joints_map["left"])
        if side in ("right", "both"):
            joint_names.extend(joints_map["right"])
        return joint_names

    def _fill_array(self, values, target_len, default):
        """补齐或截断数组到目标长度."""
        if values is None:
            return [default] * target_len
        result = list(values)
        if len(result) < target_len:
            result.extend([default] * (target_len - len(result)))
        return result[:target_len]

    def dispatch(self, action: str, args: dict) -> dict:
        if not hasattr(self, '_HybridJointCommand'):
            return {"error": "HybridJointCommand not available"}
        side = args.get("side", "right")

        # 关节名：优先使用用户指定的，否则按 hand_model 自动选择
        joint_names = args.get("joint_names")
        if not joint_names:
            joint_names = self._resolve_joints(side)
        n = len(joint_names)

        if action == "reset":
            positions = [0.0] * n
            velocities = [0.0] * n
            feedforward = [0.0] * n
            kp = [100.0] * n
            kd = [0.0] * n
        else:
            positions = self._fill_array(args.get("positions"), n, 0.0)
            velocities = self._fill_array(args.get("velocities"), n, 0.0)
            feedforward = self._fill_array(args.get("feedforward"), n, 350.0)
            kp = self._fill_array(args.get("kp"), n, 100.0)
            kd = self._fill_array(args.get("kd"), n, 0.0)

        cmd = self._HybridJointCommand()
        cmd.header = self._Header()
        cmd.header.stamp = self._pub_node.get_clock().now().to_msg()
        cmd.header.frame_id = "base_link"
        cmd.joint_name = joint_names
        cmd.position = [float(p) for p in positions]
        cmd.velocity = [float(v) for v in velocities]
        cmd.feedforward = [float(f) for f in feedforward]
        cmd.kp = [float(k) for k in kp]
        cmd.kd = [float(k) for k in kd]
        self._pub.publish(cmd)

        return {"ok": True, "action": action, "side": side,
                "joint_count": n, "hand_model": self._hand_model}


# ══════════════════════════════════════════════════════════════════════════════
# HandSensorPlugin 手部传感器
# ══════════════════════════════════════════════════════════════════════════════
class HandSensorPlugin(BasePlugin):
    """订阅 /hand_sensor topic，获取灵巧手触觉+力+位置反馈."""

    def __init__(self, cfg, namespace, ros2):
        super().__init__(cfg, namespace, ros2)
        self._topic = f"/{namespace}/q5/hand_sensor"
        self._sensor_data = {}  # side → {pressures, forces, positions}

    def get_tool(self) -> dict:
        return {
            "name": "hand_sensor",
            "type": "sensor",
            "description": "Q5灵巧手传感器 — 指尖压力/关节力矩/触觉反馈。XHAND1带触觉，XHand Lite可能为空",
            "inputSchema": {"type": "object", "properties": {}},
            "topic_out": [{"topic": self._topic, "format": "data/json"}],
        }

    def start(self):
        super().start()
        self._pub = self._pub_node.create_publisher(String, self._topic, _RELIABLE_QOS)
        self._sub_node.create_subscription(String, "/hand_sensor", self._on_hand_sensor, _RELIABLE_QOS)
        threading.Thread(target=self._publish_loop, daemon=True).start()

    def _on_hand_sensor(self, msg):
        try:
            data = json.loads(msg.data)
            self._sensor_data = data
        except Exception:
            pass

    def _publish_loop(self):
        while self._running:
            try:
                data = json.dumps({"sensor": self._sensor_data, "timestamp": time.time()})
                self._pub.publish(String(data=data))
            except Exception:
                pass
            time.sleep(0.05)  # 20Hz

    def dispatch(self, action: str, args: dict) -> dict:
        return {"sensor_data": self._sensor_data}


class TtsPlugin(BasePlugin):
    def get_tool(self) -> dict:
        return {
            "name": "tts",
            "type": "actuator",
            "description": "Q5语音合成 — 文字转语音播放，通过 /speech/sentence_topic 发布。支持音量/音色/打断",
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {"type": "string", "enum": ["speak", "stop", "set_volume", "set_voice"],
                               "description": "控制动作"},
                    "text": {"type": "string", "description": "要播放的文本"},
                    "volume": {"type": "number", "minimum": 0, "maximum": 100, "default": 70, "description": "音量0-100"},
                    "voice": {"type": "string", "enum": ["male", "female", "xiaojuan"], "default": "female", "description": "音色选择"},
                    "force": {"type": "boolean", "default": False, "description": "是否打断当前播放"},
                },
                "required": ["action"],
                "x-action-params": {
                    "speak": {"params": ["text", "volume", "voice", "force"], "description": "合成并播放文本"},
                    "stop": {"params": [], "description": "停止播放"},
                    "set_volume": {"params": ["volume"], "description": "设置播放音量(0-100)"},
                    "set_voice": {"params": ["voice"], "description": "设置TTS音色"},
                },
            },
        }

    def start(self):
        super().start()
        # /speech/sentence_topic — Q5 语音合成 topic（与 ASR 共用）
        self._pub = self._pub_node.create_publisher(String, "/speech/sentence_topic", _RELIABLE_QOS)

    def dispatch(self, action: str, args: dict) -> dict:
        if action == "speak":
            text = args.get("text", "")
            force = args.get("force", False)
            payload = json.dumps({"text": text, "interrupt": force})
            self._pub.publish(String(data=payload))
        elif action in ("stop", "set_volume", "set_voice"):
            # speech topic 可能需要特定格式，保留 JSON
            payload = json.dumps({"action": action, **args})
            self._pub.publish(String(data=payload))
        else:
            return {"error": f"unknown action: {action}"}
        return {"ok": True, "action": action}


# ══════════════════════════════════════════════════════════════════════════════
# AudioPlayerPlugin 音频文件播放 (官方 /audio_player 接口)
# ══════════════════════════════════════════════════════════════════════════════
class AudioPlayerPlugin(BasePlugin):
    def __init__(self, cfg, namespace, ros2):
        super().__init__(cfg, namespace, ros2)
        self._action_client = None
        self._cli_is_play = None
        self._cli_stop_play = None
        self._cli_set_volume = None

    def get_tool(self) -> dict:
        return {
            "name": "audio_player",
            "type": "actuator",
            "description": "Q5音频文件播放 — 支持通过ID/路径/文件名播放已上传的音频，支持音量控制/查询/停止",
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {"type": "string", "enum": ["play", "is_play", "stop", "set_volume"],
                               "default": "play",
                               "description": "play=播放音频，is_play=查询播放状态，stop=停止播放，set_volume=设置音量"},
                    "mode": {"type": "number", "enum": [0, 1, 2, 3], "default": 3,
                             "description": "播放模式: 0=使用ID, 1=使用路径, 2=使用item(JSON), 3=使用文件名"},
                    "id": {"type": "number", "minimum": 1, "default": 0,
                           "description": "播放模式0时使用，音频ID(>0)"},
                    "path": {"type": "string", "default": "",
                             "description": "播放模式1时使用，音频完整路径"},
                    "item": {"type": "string", "default": "",
                             "description": "播放模式2时使用，JSON字符串如 {\"file_name\":\"aaa\",\"text\":\"xxx\"}"},
                    "file_name": {"type": "string", "default": "",
                                  "description": "播放模式3时使用，音频文件名如 hello.wav"},
                    "force_play": {"type": "boolean", "default": True,
                                   "description": "是否强制播放(中断当前音频)"},
                    "timeout": {"type": "number", "default": 0,
                                "description": "播放超时时间(秒)，0表示不限制"},
                    "volume": {"type": "number", "minimum": 0, "maximum": 100, "default": 70,
                               "description": "音量设置(0-100)"},
                },
                "required": ["action"],
                "x-action-params": {
                    "play": {"params": ["mode", "id", "path", "item", "file_name", "force_play", "timeout"],
                             "description": "播放音频文件"},
                    "is_play": {"params": [], "description": "查询是否正在播放音频"},
                    "stop": {"params": [], "description": "停止音频播放"},
                    "set_volume": {"params": ["volume"], "description": "设置音量(0-100)"},
                },
            },
        }

    def start(self):
        super().start()
        try:
            from xbot_common_interfaces.action import AudioPlay
            from rclpy.action import ActionClient
            self._action_client = ActionClient(self._sub_node, AudioPlay, "/audio_player/play")
        except ImportError:
            self._action_client = None

        from std_srvs.srv import Trigger
        self._cli_is_play = self._pub_node.create_client(Trigger, "/audio_player/is_play")
        self._cli_stop_play = self._pub_node.create_client(Trigger, "/audio_player/stop_play")

        try:
            from xbot_common_interfaces.srv import SetVolume
            self._cli_set_volume = self._pub_node.create_client(SetVolume, "/audio_player/set_volume")
        except ImportError:
            self._cli_set_volume = None

    def _call_service(self, client, service_name, request_cls, request=None):
        if not client.service_is_ready():
            return {"error": f"service {service_name} not ready"}
        if request is None:
            request = request_cls.Request() if hasattr(request_cls, "Request") else request_cls()
        future = client.call_async(request)
        try:
            rclpy.spin_until_future_complete(self._pub_node, future, timeout_sec=2.0)
        except Exception:
            return {"error": f"service call to {service_name} failed"}
        if future.result() is None:
            return {"error": f"service {service_name} timeout or None"}
        resp = future.result()
        return {"success": getattr(resp, "success", None), "message": getattr(resp, "message", "")}

    def dispatch(self, action: str, args: dict) -> dict:
        if action == "play":
            if self._action_client is None or not self._action_client.server_is_ready():
                return {"error": "AudioPlay action server not ready"}

            from xbot_common_interfaces.action import AudioPlay
            goal = AudioPlay.Goal()
            goal.mode = int(args.get("mode", 3))
            goal.force_play = args.get("force_play", True)
            goal.id = int(args.get("id", 0))
            goal.path = args.get("path", "")
            goal.item = args.get("item", "")
            goal.file_name = args.get("file_name", "")
            goal.timeout = float(args.get("timeout", 0))

            send_goal_future = self._action_client.send_goal_async(goal)
            rclpy.spin_until_future_complete(self._sub_node, send_goal_future, timeout_sec=2.0)
            goal_handle = send_goal_future.result()
            if not goal_handle or not goal_handle.accepted:
                return {"error": "AudioPlay goal rejected"}

            get_result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self._sub_node, get_result_future, timeout_sec=60.0)
            result = get_result_future.result().result if get_result_future.result() else None
            if result:
                return {"ok": True, "success": result.success, "message": result.message}
            return {"ok": True}

        if action == "is_play":
            return self._call_service(self._cli_is_play, "/audio_player/is_play", Trigger)

        if action == "stop":
            return self._call_service(self._cli_stop_play, "/audio_player/stop_play", Trigger)

        if action == "set_volume":
            if self._cli_set_volume is None:
                return {"error": "SetVolume service not available"}
            from xbot_common_interfaces.srv import SetVolume
            req = SetVolume.Request()
            req.volume = int(args.get("volume", 70))
            return self._call_service(self._cli_set_volume, "/audio_player/set_volume", SetVolume, req)

        return {"error": f"unknown action: {action}"}


# ══════════════════════════════════════════════════════════════════════════════
# AudioPlugin 声卡直接访问 (sounddevice 录音/播放)
# ══════════════════════════════════════════════════════════════════════════════
class AudioPlugin(BasePlugin):
    def __init__(self, cfg, namespace, ros2):
        super().__init__(cfg, namespace, ros2)
        self._input_device = 6   # Q5 默认麦克风设备索引
        self._output_device = 6  # Q5 默认扬声器设备索引

    def get_tool(self) -> dict:
        return {
            "name": "audio",
            "type": "actuator",
            "description": "Q5声卡直接访问 — 通过sounddevice进行麦克风录音和扬声器播放，支持采样率/通道数配置",
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {"type": "string", "enum": ["record", "play", "list_devices"],
                               "default": "record",
                               "description": "record=录音，play=播放音频，list_devices=列出可用声卡设备"},
                    "duration": {"type": "number", "minimum": 0.1, "default": 2.0,
                                  "description": "录音时长(秒)"},
                    "sample_rate": {"type": "number", "minimum": 8000, "default": 44100,
                                    "description": "采样率(Hz)，默认44100"},
                    "channels": {"type": "number", "minimum": 1, "default": 1,
                                 "description": "通道数，默认1"},
                    "amplify": {"type": "number", "minimum": 1, "default": 1.0,
                                "description": "音量放大倍数"},
                    "audio_data": {"type": "array", "items": {"type": "number"},
                                   "description": "播放音频数据(采样点数组)，record返回此数据"},
                    "input_device": {"type": "number", "default": 6,
                                     "description": "输入设备索引"},
                    "output_device": {"type": "number", "default": 6,
                                      "description": "输出设备索引"},
                },
                "required": ["action"],
                "x-action-params": {
                    "record": {"params": ["duration", "sample_rate", "channels", "amplify", "input_device"],
                               "description": "从麦克风录音，返回音频采样数据"},
                    "play": {"params": ["audio_data", "sample_rate", "output_device", "amplify"],
                             "description": "通过扬声器播放音频数据"},
                    "list_devices": {"params": [], "description": "列出系统所有可用声卡设备"},
                },
            },
        }

    def dispatch(self, action: str, args: dict) -> dict:
        import numpy as np
        try:
            import sounddevice as sd
        except ImportError:
            return {"error": "sounddevice library not installed"}

        if action == "list_devices":
            devices = sd.query_devices()
            result = []
            for i, dev in enumerate(devices):
                result.append({
                    "index": i,
                    "name": dev.get("name", ""),
                    "input_channels": dev.get("max_input_channels", 0),
                    "output_channels": dev.get("max_output_channels", 0),
                    "default_samplerate": dev.get("default_samplerate", 0),
                })
            return {"devices": result}

        if action == "record":
            duration = float(args.get("duration", 2.0))
            sample_rate = int(args.get("sample_rate", 44100))
            channels = int(args.get("channels", 1))
            amplify = float(args.get("amplify", 1.0))
            input_dev = int(args.get("input_device", self._input_device))

            sd.default.device = (input_dev, sd.default.device[1])
            print(f"[audio] Recording {duration}s @ {sample_rate}Hz, ch={channels}, dev={input_dev}")

            try:
                recording = sd.rec(int(sample_rate * duration),
                                   samplerate=sample_rate, channels=channels, dtype='int16')
                sd.wait()
            except Exception as e:
                return {"error": f"recording failed: {str(e)}"}

            # 放大并裁剪
            audio = recording * amplify
            audio = np.clip(audio, -32768, 32767).astype(np.int16)

            # 返回扁平化数组
            audio_list = audio.flatten().tolist()
            return {
                "ok": True,
                "duration": duration,
                "sample_rate": sample_rate,
                "channels": channels,
                "samples": len(audio_list),
                "audio_data": audio_list,
            }

        if action == "play":
            audio_data = args.get("audio_data")
            if audio_data is None:
                return {"error": "audio_data is required for play"}

            sample_rate = int(args.get("sample_rate", 44100))
            amplify = float(args.get("amplify", 1.0))
            output_dev = int(args.get("output_device", self._output_device))

            sd.default.device = (sd.default.device[0], output_dev)

            try:
                audio = np.array(audio_data, dtype=np.int16) * amplify
                audio = np.clip(audio, -32768, 32767).astype(np.int16)
                sd.play(audio, samplerate=sample_rate)
                sd.wait()
            except Exception as e:
                return {"error": f"playback failed: {str(e)}"}

            return {"ok": True, "duration": len(audio_data) / sample_rate if sample_rate else 0}

        return {"error": f"unknown action: {action}"}


class LedPlugin(BasePlugin):
    def get_tool(self) -> dict:
        return {
            "name": "led",
            "type": "actuator",
            "description": "Q5机身指示灯带控制 — RGB颜色/亮度/效果(常亮/呼吸/闪烁/彩虹)",
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {"type": "string", "enum": ["set_color", "set_rgb", "set_effect", "reset"],
                               "description": "set_color=预设颜色, set_rgb=自定义RGB, set_effect=灯光效果, reset=恢复默认电量指示"},
                    "color": {"type": "string", "enum": ["green", "blue", "yellow", "orange", "red", "white", "off"],
                              "description": "set_color 的预设颜色"},
                    "red": {"type": "number", "minimum": 0, "maximum": 255, "description": "红色分量(0-255)"},
                    "green": {"type": "number", "minimum": 0, "maximum": 255, "description": "绿色分量(0-255)"},
                    "blue": {"type": "number", "minimum": 0, "maximum": 255, "description": "蓝色分量(0-255)"},
                    "brightness": {"type": "number", "minimum": 0, "maximum": 255, "default": 255, "description": "亮度(0-255)"},
                    "effect": {"type": "string", "enum": ["solid", "breath", "blink", "rainbow"], "default": "solid",
                               "description": "灯光效果"},
                    "effect_speed": {"type": "number", "minimum": 0, "maximum": 1, "default": 0.5,
                                     "description": "效果速度(0-1)"},
                },
                "required": ["action"],
                "x-action-params": {
                    "set_color": {"params": ["color", "brightness", "effect", "effect_speed"], "description": "设置预设颜色+效果"},
                    "set_rgb": {"params": ["red", "green", "blue", "brightness", "effect", "effect_speed"], "description": "自定义RGB颜色"},
                    "set_effect": {"params": ["effect", "effect_speed"], "description": "仅修改灯光效果"},
                    "reset": {"params": [], "description": "恢复默认电量指示模式"},
                },
            },
        }

    # 预设颜色
    _COLOR_MAP = {
        "green":  (0, 255, 0),
        "blue":   (0, 0, 255),
        "yellow": (255, 255, 0),
        "orange": (255, 165, 0),
        "red":    (255, 0, 0),
        "white":  (255, 255, 255),
        "off":    (0, 0, 0),
    }

    _EFFECT_MAP = {"solid": 0, "breath": 1, "blink": 2, "rainbow": 3}

    def start(self):
        super().start()
        # /led_control — Q5 实际 LED 控制 topic
        try:
            from q5_msgs.msg import LedCmd
            self._pub = self._pub_node.create_publisher(LedCmd, "/led_control", _RELIABLE_QOS)
            self._LedCmd = LedCmd
        except ImportError:
            self._pub = self._pub_node.create_publisher(String, "/led_control", _RELIABLE_QOS)
            self._LedCmd = None

    def dispatch(self, action: str, args: dict) -> dict:
        effect_str = args.get("effect", "solid")
        effect = self._EFFECT_MAP.get(effect_str, 0)
        effect_speed = float(args.get("effect_speed", 0.5))

        if action == "reset":
            r, g, b = 0, 0, 255
            effect, effect_speed = 0, 0.5
        elif action == "set_color":
            color = args.get("color", "blue")
            r, g, b = self._COLOR_MAP.get(color, (0, 0, 255))
        elif action == "set_rgb":
            r = int(max(0, min(255, args.get("red", 0))))
            g = int(max(0, min(255, args.get("green", 0))))
            b = int(max(0, min(255, args.get("blue", 0))))
        elif action == "set_effect":
            r, g, b = 0, 0, 255  # 保持蓝色
        else:
            return {"error": f"unknown action: {action}"}

        brightness = int(max(0, min(255, args.get("brightness", 255))))

        if self._LedCmd is not None:
            cmd = self._LedCmd()
            cmd.red = r
            cmd.green = g
            cmd.blue = b
            cmd.brightness = brightness
            cmd.effect = effect
            cmd.effect_speed = effect_speed
            self._pub.publish(cmd)
        else:
            # fallback: String JSON
            self._pub.publish(String(data=json.dumps({
                "red": r, "green": g, "blue": b,
                "brightness": brightness, "effect": effect, "effect_speed": effect_speed,
            })))

        return {"ok": True, "rgb": [r, g, b], "brightness": brightness,
                "effect": effect_str, "effect_speed": effect_speed}


class ChatPlugin(BasePlugin):
    """Q5 大模型对话开关 — 通过 XOS 界面控制，无独立 ROS2 topic。

    Q5 实际未暴露 /chat/enable topic，对话功能需在 XOS 后台手动开启。
    此卡片发送的指令不会有实际效果，仅作为占位保留。
    """

    def get_tool(self) -> dict:
        return {
            "name": "chat",
            "type": "actuator",
            "description": "Q5大模型语音对话开关 — 通过XOS界面控制，无ROS2接口。本卡片仅作占位",
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {"type": "string", "enum": ["enable", "disable"], "description": "开启/关闭对话"},
                },
                "required": ["action"],
            },
        }

    def start(self):
        super().start()
        self._pub = None

    def dispatch(self, action: str, args: dict) -> dict:
        return {"ok": True, "state": "enabled" if action == "enable" else "disabled",
                "warning": "chat 功能通过 XOS 界面控制，此卡片调用无实际效果"}


class ActionPlayerPlugin(BasePlugin):
    """DEPRECATED: 手册只记录了 /gesture/upper_limb_play，请使用 gesture_player。
    该插件保留用于向后兼容，但 /action/cmd topic 未在手册中文档化。"""

    def get_tool(self) -> dict:
        return {
            "name": "action_player",
            "type": "actuator",
            "description": "DEPRECATED — Q5动作/语音回放。请使用 gesture_player（手册 /gesture/upper_limb_play 接口）。/action/cmd topic 手册未文档化",
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {"type": "string", "enum": ["play", "stop", "list"],
                               "description": "控制动作"},
                    "action_name": {"type": "string", "description": "要播放的动作名称"},
                    "play_voice": {"type": "boolean", "default": True, "description": "是否同时播放关联语音"},
                },
                "required": ["action"],
            },
        }

    def start(self):
        super().start()
        # TODO: /action/cmd 手册未文档化，需确认Q5固件是否支持
        self._pub = self._pub_node.create_publisher(String, "/action/cmd", _RELIABLE_QOS)

    def dispatch(self, action: str, args: dict) -> dict:
        self._pub.publish(String(data=json.dumps({"action": action, **args})))
        return {"ok": True, "deprecated": True, "migrate_to": "gesture_player"}


# ══════════════════════════════════════════════════════════════════════════════
# GesturePlayerPlugin 上肢动作回放 (官方 /gesture 接口)
# ══════════════════════════════════════════════════════════════════════════════
class GesturePlayerPlugin(BasePlugin):
    def __init__(self, cfg, namespace, ros2):
        super().__init__(cfg, namespace, ros2)
        self._pub = None
        self._cli_is_play = None
        self._cli_stop_play = None
        self._playing = False

    def get_tool(self) -> dict:
        return {
            "name": "gesture_player",
            "type": "actuator",
            "description": "Q5上肢录制动作回放 — 播放通过遥操作录制并上传到XOS的动作文件，支持查询/停止",
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {"type": "string", "enum": ["play", "is_play", "stop"],
                               "default": "play",
                               "description": "play=播放动作，is_play=查询是否正在播放，stop=停止播放"},
                    "action_name": {"type": "string", "description": "要播放的动作名称（需提前通过XOS上传）"},
                },
                "required": ["action"],
                "x-action-params": {
                    "play": {"params": ["action_name"], "description": "播放指定名称的上肢动作"},
                    "is_play": {"params": [], "description": "查询是否有动作正在播放"},
                    "stop": {"params": [], "description": "停止当前播放的动作"},
                },
            },
        }

    def start(self):
        super().start()
        self._pub = self._pub_node.create_publisher(String, "/gesture/upper_limb_play", _RELIABLE_QOS)
        from std_srvs.srv import Trigger
        self._cli_is_play = self._pub_node.create_client(Trigger, "/gesture/is_play")
        self._cli_stop_play = self._pub_node.create_client(Trigger, "/gesture/stop_play")

    def _call_trigger(self, client, service_name):
        if not client.service_is_ready():
            return {"playing": False, "error": f"service {service_name} not ready"}
        future = client.call_async(Trigger.Request())
        try:
            rclpy.spin_until_future_complete(self._pub_node, future, timeout_sec=2.0)
        except Exception:
            return {"playing": False, "error": f"service call to {service_name} failed"}
        if future.result() and future.result().success:
            return {"playing": future.result().success, "message": future.result().message}
        return {"playing": False, "error": f"service {service_name} returned error"}

    def dispatch(self, action: str, args: dict) -> dict:
        if action == "play":
            action_name = args.get("action_name", "")
            if not action_name:
                return {"error": "action_name is required for play"}
            self._pub.publish(String(data=action_name))
            self._playing = True
            return {"ok": True, "action_name": action_name}

        if action == "is_play":
            return self._call_trigger(self._cli_is_play, "/gesture/is_play")

        if action == "stop":
            result = self._call_trigger(self._cli_stop_play, "/gesture/stop_play")
            if result.get("playing") is False and "error" not in result:
                self._playing = False
            return result


class NavPlugin(BasePlugin):
    """Q5导航 — 对接 era_nav_msgs (https://github.com/roboterax/era_nav_msgs).

    导航功能需单独购买。该插件封装 era_nav_msgs 的核心导航 action/service：
      - NavigateToPose / NavigateToNamedPose / Dock / StartTour / Stop / GetRobotPose
    在未安装 era_nav_msgs 时回退到 /nav/cmd 字符串模式。
    """

    def __init__(self, cfg, namespace, ros2):
        super().__init__(cfg, namespace, ros2)
        self._nav_action = None   # era_nav_msgs action client
        self._nav_available = False

    def get_tool(self) -> dict:
        return {
            "name": "nav",
            "type": "actuator",
            "description": "Q5导览导航控制(era_nav_msgs) — 自主导航到预设导览点、回充电桩、避障，依赖 era_nav_msgs 包",
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {"type": "string", "enum": ["go_to_point", "go_home", "start_tour", "stop", "get_pose"],
                               "description": "导航动作"},
                    "point_name": {"type": "string", "description": "导览点名称"},
                    "x": {"type": "number", "description": "目标x坐标(米)"},
                    "y": {"type": "number", "description": "目标y坐标(米)"},
                    "yaw": {"type": "number", "description": "目标朝向(弧度)"},
                },
                "required": ["action"],
                "x-action-params": {
                    "go_to_point": {"params": ["point_name", "x", "y", "yaw"], "description": "导航到指定导览点或坐标"},
                    "go_home": {"params": [], "description": "自主导航回充电桩(Dock)"},
                    "start_tour": {"params": [], "description": "开始按预设路线导览"},
                    "stop": {"params": [], "description": "停止/取消当前导航任务"},
                    "get_pose": {"params": [], "description": "获取当前机器人位姿"},
                },
            },
        }

    def start(self):
        super().start()
        # 尝试加载 era_nav_msgs action 接口
        try:
            from era_nav_msgs.action import NavigateToPose
            from rclpy.action import ActionClient
            self._nav_action = ActionClient(self._sub_node, NavigateToPose, "/era_nav/navigate_to_pose")
            self._nav_available = self._nav_action.server_is_ready()
            if self._nav_available:
                print("[nav] era_nav_msgs NavigateToPose action available")
        except ImportError:
            print("[nav] era_nav_msgs not found, using /nav/cmd fallback")

        # Fallback: string-based command topic
        self._pub = self._pub_node.create_publisher(String, "/nav/cmd", _RELIABLE_QOS)
        # 订阅导航状态
        try:
            from nav_msgs.msg import Odometry
            self._sub_node.create_subscription(Odometry, "/odom", self._on_odom, _LOW_LAT_QOS)
        except ImportError:
            pass
        self._current_pose = {"x": 0.0, "y": 0.0, "yaw": 0.0}

    def _on_odom(self, msg):
        p = msg.pose.pose
        q = p.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self._current_pose = {
            "x": p.position.x,
            "y": p.position.y,
            "yaw": math.atan2(siny_cosp, cosy_cosp),
        }

    def dispatch(self, action: str, args: dict) -> dict:
        # 优先使用 era_nav_msgs
        if self._nav_available and action in ("go_to_point", "go_home"):
            try:
                from era_nav_msgs.action import NavigateToPose
                goal = NavigateToPose.Goal()
                if action == "go_home":
                    goal.mode = NavigateToPose.Goal.MODE_DOCK
                else:
                    goal.mode = NavigateToPose.Goal.MODE_NAMED if args.get("point_name") else NavigateToPose.Goal.MODE_XY
                    goal.point_name = args.get("point_name", "")
                    goal.x = float(args.get("x", 0.0))
                    goal.y = float(args.get("y", 0.0))
                    goal.yaw = float(args.get("yaw", 0.0))
                future = self._nav_action.send_goal_async(goal)
                rclpy.spin_until_future_complete(self._sub_node, future, timeout_sec=2.0)
                if future.result() and future.result().accepted:
                    return {"ok": True, "nav": "era_nav_msgs", "action": action, "goal_accepted": True}
                return {"error": "Nav goal rejected"}
            except Exception as e:
                return {"error": f"era_nav_msgs error: {e}"}

        if action == "get_pose":
            return {"ok": True, "pose": self._current_pose}

        # 尝试 /navigate/* 服务
        nav_svc_map = {
            "start_nav": "/navigate/start_nav",
            "stop_nav": "/navigate/stop_nav",
            "pause_nav_ctrl": "/navigate/pause_nav_ctrl",
            "is_config_nav": "/navigate/is_config_nav",
            "is_license_verify": "/navigate/is_license_verify",
            "is_nav_executing": "/navigate/is_nav_executing",
        }
        if action in nav_svc_map:
            from std_srvs.srv import Trigger
            cli = self._sub_node.create_client(Trigger, nav_svc_map[action])
            if cli.wait_for_service(timeout_sec=3.0):
                future = cli.call_async(Trigger.Request())
                try:
                    rclpy.spin_until_future_complete(self._sub_node, future, timeout_sec=5.0)
                    resp = future.result()
                    try:
                        self._sub_node.destroy_client(cli)
                    except Exception:
                        pass
                    if resp:
                        return {"ok": resp.success, "message": getattr(resp, "message", ""), "service": nav_svc_map[action]}
                except Exception:
                    pass

        # Fallback: publish JSON command to /nav/cmd
        self._pub.publish(String(data=json.dumps({"action": action, **args})))
        return {"ok": True, "nav": "fallback", "action": action}


class BagRecordPlugin(BasePlugin):
    """封装 ros2 bag record — 录制动作相关话题"""

    def get_tool(self) -> dict:
        return {
            "name": "bag_record",
            "type": "actuator",
            "description": "启动/停止ROS2 Bag录制，默认录制手关节控制和服务位姿话题",
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {
                        "type": "string",
                        "enum": ["start", "stop", "status"],
                        "description": "start启动录制，stop停止录制，status查看状态",
                    },
                    "topics": {
                        "type": "array",
                        "items": {"type": "string"},
                        "description": "要录制的话题列表，默认 ['/hand_controller/commands', '/servo_poses']",
                    },
                    "bag_name": {
                        "type": "string",
                        "description": "bag文件名（不含扩展名），默认使用的时间戳",
                    },
                },
                "required": ["action"],
            },
        }

    def dispatch(self, action: str, args: dict) -> dict:
        if action == "stop":
            if self._proc:
                self._proc.terminate()
                self._proc.wait()
                self._proc = None
            return {"ok": True, "message": "录制已停止"}
        if action == "status":
            running = self._proc is not None and self._proc.poll() is None
            return {"running": running}
        # start
        topics = args.get("topics", ["/hand_controller/commands", "/servo_poses"])
        bag_name = args.get("bag_name")
        cmd = ["ros2", "bag", "record", "-o", bag_name] + topics if bag_name else ["ros2", "bag", "record"] + topics
        self._proc = subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        return {"ok": True, "message": f"开始录制，PID: {self._proc.pid}"}

    def start(self):
        super().start()
        self._proc = None


class BagPlaybackPlugin(BasePlugin):
    """封装 ros2 bag play — 回放动作bag文件"""

    def get_tool(self) -> dict:
        return {
            "name": "bag_playback",
            "type": "actuator",
            "description": "播放指定的ROS2 Bag文件",
            "inputSchema": {
                "type": "object",
                "properties": {
                    "bag_path": {
                        "type": "string",
                        "description": "bag文件路径（.bag目录或 .db3 文件）",
                    },
                    "speed": {
                        "type": "number",
                        "description": "回放速度倍率，默认1.0",
                    },
                },
                "required": ["bag_path"],
            },
        }

    def dispatch(self, action: str, args: dict) -> dict:
        bag_path = args.get("bag_path")
        if not bag_path:
            return {"error": "缺少 bag_path"}
        speed = args.get("speed", 1.0)
        cmd = ["ros2", "bag", "play", bag_path, "--rate", str(speed)]
        proc = subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        return {"ok": True, "message": f"开始回放，PID: {proc.pid}", "bag_path": bag_path, "speed": speed}


class MpcControllerPlugin(BasePlugin):
    """封装 MPC 控制器和 SDK 的启停 — 优先使用 ROS2 service，回退到 q5_pub_client.py"""

    def __init__(self, cfg, namespace, ros2):
        super().__init__(cfg, namespace, ros2)
        self._mpc_start_cli = None
        self._mpc_stop_cli = None
        self._mpc_status_cli = None
        self._mpc_reset_cli = None
        self._sdk_start_cli = None
        self._sdk_stop_cli = None

    def get_tool(self) -> dict:
        return {
            "name": "mpc_controller",
            "type": "actuator",
            "description": "控制MPC算法和SDK的启动/停止/查询/复位",
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {
                        "type": "string",
                        "enum": ["start_mpc", "stop_mpc", "query_mpc", "reset_mpc", "reset_mmpc", "start_sdk", "stop_sdk"],
                        "description": "MPC/SDK操作: start_mpc/stop_mpc/query_mpc/reset_mpc/reset_mmpc/start_sdk/stop_sdk",
                    },
                    "mode": {
                        "type": "number", "enum": [0, 1], "default": 0,
                        "description": "reset_mpc 模式: 0=抬手, 1=放手",
                    },
                },
                "required": ["action"],
            },
        }

    def start(self):
        super().start()
        from std_srvs.srv import Trigger
        # 尝试连接 MPC services
        self._mpc_start_cli = self._sub_node.create_client(Trigger, "/mpc/start")
        self._mpc_stop_cli = self._sub_node.create_client(Trigger, "/mpc/stop")
        self._mpc_status_cli = self._sub_node.create_client(Trigger, "/mpc/status")
        self._mpc_reset_cli = self._sub_node.create_client(Trigger, "/mpc/reset")
        self._sdk_start_cli = self._sub_node.create_client(Trigger, "/sdk/start")
        self._sdk_stop_cli = self._sub_node.create_client(Trigger, "/sdk/stop")

    def _call_trigger(self, client, service_name, timeout=5.0):
        if client is None or not client.service_is_ready():
            return {"error": f"service {service_name} not available"}
        future = client.call_async(Trigger.Request())
        try:
            rclpy.spin_until_future_complete(self._sub_node, future, timeout_sec=timeout)
        except Exception:
            return {"error": f"service call to {service_name} failed"}
        if future.result() is None:
            return {"error": f"service {service_name} timed out"}
        resp = future.result()
        return {"ok": resp.success, "message": getattr(resp, "message", "")}

    def dispatch(self, action: str, args: dict) -> dict:
        service_map = {
            "start_mpc": (self._mpc_start_cli, "/mpc/start"),
            "stop_mpc": (self._mpc_stop_cli, "/mpc/stop"),
            "query_mpc": (self._mpc_status_cli, "/mpc/status"),
            "reset_mpc": (self._mpc_reset_cli, "/mpc/reset"),
            "start_sdk": (self._sdk_start_cli, "/sdk/start"),
            "stop_sdk": (self._sdk_stop_cli, "/sdk/stop"),
        }
        mmpc_reset_map = {"reset_mmpc": "/mobile_manipulator_mpc_reset"}

        if action in service_map:
            client, svc_name = service_map[action]
            if client is not None:
                return self._call_trigger(client, svc_name)

        if action in mmpc_reset_map:
            from std_srvs.srv import Trigger
            cli = self._sub_node.create_client(Trigger, mmpc_reset_map[action])
            if cli.wait_for_service(timeout_sec=3.0):
                result = self._call_trigger(cli, mmpc_reset_map[action])
                try:
                    self._sub_node.destroy_client(cli)
                except Exception:
                    pass
                return result
            return {"error": f"service {mmpc_reset_map[action]} not available"}

        # 回退到 q5_pub_client.py
        script = Path(__file__).parent / "q5_pub_client.py"
        if not script.exists():
            return {"error": "q5_pub_client.py not found and ROS2 service not available"}

        # 映射 action 到 --type/--cmd 参数
        type_cmd = {
            "start_mpc": ("mpc", "start"),
            "stop_mpc": ("mpc", "stop"),
            "query_mpc": ("mpc", "query"),
            "reset_mpc": ("mpc", "reset"),
            "start_sdk": ("sdk", "start"),
            "stop_sdk": ("sdk", "stop"),
        }
        ctrl_type, cmd = type_cmd.get(action, ("mpc", "start"))
        cli_args = ["python3", str(script), "--type", ctrl_type, "--cmd", cmd]
        if action == "reset_mpc":
            cli_args += ["--mode", str(args.get("mode", 0))]

        try:
            result = subprocess.run(cli_args, timeout=10, capture_output=True, text=True)
            if result.returncode == 0:
                return {"ok": True, "message": result.stdout.strip()[:200]}
            return {"error": result.stderr.strip()[:500] or result.stdout.strip()[:500]}
        except FileNotFoundError:
            return {"error": "python3 not found"}
        except subprocess.TimeoutExpired:
            return {"error": f"{action} 超时（10秒）"}


class ModelPlugin(BasePlugin):
    def get_tool(self) -> dict:
        return {
            "name": "model",
            "type": "resource",
            "description": "Q5 URDF骨架模型 — 用于3D可视化",
            "inputSchema": {"type": "object", "properties": {}},
        }

    def dispatch(self, action: str, args: dict) -> dict:
        # 返回URDF文件内容
        urdf_path = Path(__file__).parent / "resource" / "q5.urdf"
        if urdf_path.exists():
            return {"urdf": urdf_path.read_text()}
        return {"error": "URDF file not found"}


# ══════════════════════════════════════════════════════════════════════════════
# OdometryPlugin 里程计
# ══════════════════════════════════════════════════════════════════════════════
class OdometryPlugin(BasePlugin):
    """订阅 /wr1_base_drive_controller/odom 获取底盘里程计数据."""

    def __init__(self, cfg, namespace, ros2):
        super().__init__(cfg, namespace, ros2)
        self._topic = f"/{namespace}/q5/odometry"
        self._pose = {"x": 0.0, "y": 0.0, "yaw": 0.0}
        self._twist = {"vx": 0.0, "vy": 0.0, "vz": 0.0, "wx": 0.0, "wy": 0.0, "wz": 0.0}

    def get_tool(self) -> dict:
        return {
            "name": "odometry",
            "type": "sensor",
            "description": "Q5底盘里程计 — 位姿(x/y/yaw) + 速度(vx/vy/wz)，数据来源: /wr1_base_drive_controller/odom",
            "inputSchema": {"type": "object", "properties": {}},
            "topic_out": [{"topic": self._topic, "format": "data/json"}],
        }

    def start(self):
        super().start()
        self._pub = self._pub_node.create_publisher(String, self._topic, _RELIABLE_QOS)
        try:
            from nav_msgs.msg import Odometry
            self._sub_node.create_subscription(Odometry, "/wr1_base_drive_controller/odom", self._on_odom, _LOW_LAT_QOS)
        except ImportError:
            self._sub_node.create_subscription(String, "/wr1_base_drive_controller/odom", self._on_odom_str, _LOW_LAT_QOS)
        threading.Thread(target=self._publish_loop, daemon=True).start()

    def _on_odom(self, msg):
        p = msg.pose.pose
        q = p.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self._pose = {"x": p.position.x, "y": p.position.y, "yaw": math.atan2(siny_cosp, cosy_cosp)}
        t = msg.twist.twist
        self._twist = {"vx": t.linear.x, "vy": t.linear.y, "vz": t.linear.z,
                       "wx": t.angular.x, "wy": t.angular.y, "wz": t.angular.z}

    def _on_odom_str(self, msg):
        try:
            data = json.loads(msg.data)
            self._pose = data.get("pose", self._pose)
            self._twist = data.get("twist", self._twist)
        except Exception:
            pass

    def _publish_loop(self):
        while self._running:
            try:
                self._pub.publish(String(data=json.dumps({"pose": self._pose, "twist": self._twist, "timestamp": time.time()})))
            except Exception:
                pass
            time.sleep(0.05)

    def dispatch(self, action: str, args: dict) -> dict:
        return {"pose": self._pose, "twist": self._twist}


# ══════════════════════════════════════════════════════════════════════════════
# DiagnosticsPlugin 系统诊断
# ══════════════════════════════════════════════════════════════════════════════
class DiagnosticsPlugin(BasePlugin):
    """订阅 /diagnostics_agg, /diagnostics_nuc, /diagnostics_orin, /cpu_freq 获取整机诊断信息."""

    def __init__(self, cfg, namespace, ros2):
        super().__init__(cfg, namespace, ros2)
        self._topic = f"/{namespace}/q5/diagnostics"
        self._diag_agg = {}
        self._diag_nuc = {}
        self._diag_orin = {}
        self._cpu_freq = {}

    def get_tool(self) -> dict:
        return {
            "name": "diagnostics",
            "type": "sensor",
            "description": "Q5整机诊断汇总 — NUC/Orin诊断状态、CPU频率、故障汇总，数据来源: /diagnostics_agg + /diagnostics_nuc + /diagnostics_orin + /cpu_freq",
            "inputSchema": {"type": "object", "properties": {}},
            "topic_out": [{"topic": self._topic, "format": "data/json"}],
        }

    def start(self):
        super().start()
        self._pub = self._pub_node.create_publisher(String, self._topic, _RELIABLE_QOS)
        for topic, attr in [("/diagnostics_agg", "_diag_agg"), ("/diagnostics_nuc", "_diag_nuc"),
                            ("/diagnostics_orin", "_diag_orin"), ("/cpu_freq", "_cpu_freq")]:
            self._sub_node.create_subscription(String, topic,
                lambda msg, a=attr: self._on_diag(msg, a), _LOW_LAT_QOS)
        threading.Thread(target=self._publish_loop, daemon=True).start()

    def _on_diag(self, msg, attr_name):
        try:
            setattr(self, attr_name, json.loads(msg.data))
        except Exception:
            pass

    def _publish_loop(self):
        while self._running:
            try:
                self._pub.publish(String(data=json.dumps({
                    "diag_agg": self._diag_agg, "diag_nuc": self._diag_nuc,
                    "diag_orin": self._diag_orin, "cpu_freq": self._cpu_freq,
                    "timestamp": time.time(),
                })))
            except Exception:
                pass
            time.sleep(1.0)

    def dispatch(self, action: str, args: dict) -> dict:
        return {"diag_agg": self._diag_agg, "diag_nuc": self._diag_nuc,
                "diag_orin": self._diag_orin, "cpu_freq": self._cpu_freq}


# ══════════════════════════════════════════════════════════════════════════════
# JoystickPlugin 手柄/遥控器
# ══════════════════════════════════════════════════════════════════════════════
class JoystickPlugin(BasePlugin):
    """订阅 /joy 获取手柄/遥控器输入状态."""

    def __init__(self, cfg, namespace, ros2):
        super().__init__(cfg, namespace, ros2)
        self._topic = f"/{namespace}/q5/joystick"
        self._axes = []
        self._buttons = []

    def get_tool(self) -> dict:
        return {
            "name": "joystick",
            "type": "sensor",
            "description": "Q5手柄/遥控器状态 — 摇杆轴值+按键状态，数据来源: /joy",
            "inputSchema": {"type": "object", "properties": {}},
            "topic_out": [{"topic": self._topic, "format": "data/json"}],
        }

    def start(self):
        super().start()
        self._pub = self._pub_node.create_publisher(String, self._topic, _RELIABLE_QOS)
        try:
            from sensor_msgs.msg import Joy
            self._sub_node.create_subscription(Joy, "/joy", self._on_joy, _LOW_LAT_QOS)
        except ImportError:
            self._sub_node.create_subscription(String, "/joy", self._on_joy_str, _LOW_LAT_QOS)
        threading.Thread(target=self._publish_loop, daemon=True).start()

    def _on_joy(self, msg):
        self._axes = list(msg.axes)
        self._buttons = list(msg.buttons)

    def _on_joy_str(self, msg):
        try:
            data = json.loads(msg.data)
            self._axes = data.get("axes", [])
            self._buttons = data.get("buttons", [])
        except Exception:
            pass

    def _publish_loop(self):
        while self._running:
            try:
                self._pub.publish(String(data=json.dumps({"axes": self._axes, "buttons": self._buttons, "timestamp": time.time()})))
            except Exception:
                pass
            time.sleep(0.05)

    def dispatch(self, action: str, args: dict) -> dict:
        return {"axes": self._axes, "buttons": self._buttons}


# ══════════════════════════════════════════════════════════════════════════════
# TeleopPlugin 遥操作
# ══════════════════════════════════════════════════════════════════════════════
class TeleopPlugin(BasePlugin):
    """遥操作状态监控 + /teleoperation/service 启停控制."""

    def __init__(self, cfg, namespace, ros2):
        super().__init__(cfg, namespace, ros2)
        self._topic = f"/{namespace}/q5/teleop"
        self._state = {}
        self._health = {}
        self._calib_state = {}

    def get_tool(self) -> dict:
        return {
            "name": "teleop",
            "type": "sensor",
            "description": "Q5遥操作状态 — 遥操作运行状态、健康监控、标定状态；action: start/stop 启停遥操作服务",
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {"type": "string", "enum": ["status", "start", "stop"],
                               "default": "status",
                               "description": "status=获取遥操作状态, start/stop=启停遥操作服务"},
                },
                "required": [],
            },
            "topic_out": [{"topic": self._topic, "format": "data/json"}],
        }

    def start(self):
        super().start()
        self._pub = self._pub_node.create_publisher(String, self._topic, _RELIABLE_QOS)
        self._sub_node.create_subscription(String, "/teleop_state", self._on_teleop_state, _LOW_LAT_QOS)
        self._sub_node.create_subscription(String, "/teleoperation_health", self._on_teleop_health, _LOW_LAT_QOS)
        self._sub_node.create_subscription(String, "/teleoperation_calib_state", self._on_calib_state, _LOW_LAT_QOS)
        threading.Thread(target=self._publish_loop, daemon=True).start()

    def _on_teleop_state(self, msg):
        try:
            self._state = json.loads(msg.data)
        except Exception:
            pass

    def _on_teleop_health(self, msg):
        try:
            self._health = json.loads(msg.data)
        except Exception:
            pass

    def _on_calib_state(self, msg):
        try:
            self._calib_state = json.loads(msg.data)
        except Exception:
            pass

    def _publish_loop(self):
        while self._running:
            try:
                self._pub.publish(String(data=json.dumps({
                    "state": self._state, "health": self._health,
                    "calib_state": self._calib_state, "timestamp": time.time(),
                })))
            except Exception:
                pass
            time.sleep(0.2)

    def dispatch(self, action: str, args: dict) -> dict:
        if action in ("start", "stop"):
            from std_srvs.srv import Trigger
            cli = self._sub_node.create_client(Trigger, "/teleoperation/service")
            if not cli.wait_for_service(timeout_sec=3.0):
                return {"error": "/teleoperation/service not available"}
            future = cli.call_async(Trigger.Request())
            try:
                rclpy.spin_until_future_complete(self._sub_node, future, timeout_sec=5.0)
                resp = future.result()
                return {"ok": resp.success, "message": getattr(resp, "message", ""), "action": action} if resp else {"error": "timeout"}
            except Exception:
                return {"error": "call failed"}
            finally:
                try:
                    self._sub_node.destroy_client(cli)
                except Exception:
                    pass
        return {"state": self._state, "health": self._health, "calib_state": self._calib_state}


# ══════════════════════════════════════════════════════════════════════════════
# JointConfigPlugin 关节参数配置
# ══════════════════════════════════════════════════════════════════════════════
_JOINT_CONFIG_SERVICES = {
    "get_pos_kp": "获取位置比例增益",
    "set_pos_kp": "设置位置比例增益",
    "get_pos_ki": "获取位置积分增益",
    "set_pos_ki": "设置位置积分增益",
    "get_pos_kd": "获取位置微分增益",
    "set_pos_kd": "设置位置微分增益",
    "get_vel_kp": "获取速度比例增益",
    "set_vel_kp": "设置速度比例增益",
    "get_vel_ki": "获取速度积分增益",
    "set_vel_ki": "设置速度积分增益",
    "get_torque_factor": "获取力矩系数",
    "set_torque_factor": "设置力矩系数",
    "get_columb_friction": "获取库仑摩擦力",
    "set_columb_friction": "设置库仑摩擦力",
    "get_viscous_friction": "获取粘性摩擦力",
    "set_viscous_friction": "设置粘性摩擦力",
    "get_error_mask": "获取错误掩码",
    "set_error_mask": "设置错误掩码",
    "get_friction_enable": "获取摩擦力使能状态",
    "set_friction_enable": "设置摩擦力使能状态",
    "get_joint_pn": "获取关节PN号",
    "get_joint_sn": "获取关节SN号",
    "get_joints_version": "获取关节固件版本",
    "get_joints_boot_version": "获取关节Boot版本",
    "get_sdk_version": "获取SDK版本",
    "save_param": "保存参数到Flash",
    "set_def_param": "恢复默认参数",
    "update_joint": "更新单关节参数",
    "update_joints": "更新多关节参数",
}


class JointConfigPlugin(BasePlugin):
    """关节参数读写 — 封装 /get_* /set_* 等20+个关节配置服务."""

    def get_tool(self) -> dict:
        actions = list(_JOINT_CONFIG_SERVICES.keys())
        return {
            "name": "joint_config",
            "type": "actuator",
            "description": "Q5关节参数配置 — 读写KP/KD/KI/摩擦力/力矩系数等关节参数，支持20+个服务",
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {"type": "string", "enum": actions,
                               "description": "关节参数操作: " + ", ".join(actions)},
                    "joint_id": {"type": "number", "description": "关节ID（部分服务需要）"},
                    "value": {"type": "number", "description": "设置值（set类服务需要）"},
                },
                "required": ["action"],
            },
        }

    def dispatch(self, action: str, args: dict) -> dict:
        svc_name = f"/{action}"
        from std_srvs.srv import Trigger
        # 大多数关节参数服务使用 Trigger 协议；部分可能使用自定义 srv
        # 统一按 Trigger 尝试调用
        cli = self._sub_node.create_client(Trigger, svc_name)
        if not cli.wait_for_service(timeout_sec=2.0):
            return {"error": f"service {svc_name} not available", "action": action}
        future = cli.call_async(Trigger.Request())
        try:
            rclpy.spin_until_future_complete(self._sub_node, future, timeout_sec=3.0)
            resp = future.result()
            if resp:
                result = {"ok": resp.success, "message": getattr(resp, "message", ""), "service": svc_name}
            else:
                result = {"error": "timeout", "service": svc_name}
        except Exception:
            result = {"error": "call failed", "service": svc_name}
        try:
            self._sub_node.destroy_client(cli)
        except Exception:
            pass
        return result


# ══════════════════════════════════════════════════════════════════════════════
# MotionPlugin 运动管理器
# ══════════════════════════════════════════════════════════════════════════════
class MotionPlugin(BasePlugin):
    """运动管理器状态监控 + change_state/motion_request 控制."""

    def __init__(self, cfg, namespace, ros2):
        super().__init__(cfg, namespace, ros2)
        self._topic = f"/{namespace}/q5/motion"
        self._motion_status = {}
        self._transition_event = {}

    def get_tool(self) -> dict:
        return {
            "name": "motion",
            "type": "sensor",
            "description": "Q5运动管理器 — 状态监控+状态切换+运动请求，数据来源: /motion_manager/motion_status + /motion_manager/transition_event",
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {"type": "string", "enum": ["status", "change_state", "get_state", "get_available_states", "motion_request"],
                               "default": "status",
                               "description": "status=获取运动状态, change_state=切换状态, get_state=获取当前状态, get_available_states=获取可用状态列表, motion_request=发起运动请求"},
                    "target_state": {"type": "string", "description": "目标状态名（change_state时必填）"},
                    "request_name": {"type": "string", "description": "运动请求名（motion_request时必填）"},
                },
                "required": [],
            },
            "topic_out": [{"topic": self._topic, "format": "data/json"}],
        }

    def start(self):
        super().start()
        self._pub = self._pub_node.create_publisher(String, self._topic, _RELIABLE_QOS)
        self._sub_node.create_subscription(String, "/motion_manager/motion_status", self._on_status, _LOW_LAT_QOS)
        self._sub_node.create_subscription(String, "/motion_manager/transition_event", self._on_transition, _LOW_LAT_QOS)
        threading.Thread(target=self._publish_loop, daemon=True).start()

    def _on_status(self, msg):
        try:
            self._motion_status = json.loads(msg.data)
        except Exception:
            pass

    def _on_transition(self, msg):
        try:
            self._transition_event = json.loads(msg.data)
        except Exception:
            pass

    def _publish_loop(self):
        while self._running:
            try:
                self._pub.publish(String(data=json.dumps({
                    "status": self._motion_status, "transition": self._transition_event,
                    "timestamp": time.time(),
                })))
            except Exception:
                pass
            time.sleep(0.2)

    def dispatch(self, action: str, args: dict) -> dict:
        if action in ("change_state", "get_state", "get_available_states", "motion_request"):
            from std_srvs.srv import Trigger
            svc_map = {
                "change_state": "/motion_manager/change_state",
                "get_state": "/motion_manager/get_state",
                "get_available_states": "/motion_manager/get_available_states",
                "motion_request": "/motion_manager/motion_request",
            }
            svc = svc_map[action]
            cli = self._sub_node.create_client(Trigger, svc)
            if not cli.wait_for_service(timeout_sec=3.0):
                return {"error": f"service {svc} not available"}
            future = cli.call_async(Trigger.Request())
            try:
                rclpy.spin_until_future_complete(self._sub_node, future, timeout_sec=5.0)
                resp = future.result()
                result = {"ok": resp.success, "message": getattr(resp, "message", ""), "service": svc} if resp else {"error": "timeout"}
            except Exception:
                result = {"error": "call failed", "service": svc}
            try:
                self._sub_node.destroy_client(cli)
            except Exception:
                pass
            return result
        return {"status": self._motion_status, "transition": self._transition_event}


# ══════════════════════════════════════════════════════════════════════════════
# BrakePlugin 抱闸控制
# ══════════════════════════════════════════════════════════════════════════════
class BrakePlugin(BasePlugin):
    """关节抱闸控制 — /control_brake 服务."""

    def get_tool(self) -> dict:
        return {
            "name": "brake",
            "type": "actuator",
            "description": "Q5关节抱闸控制 — 通过 /control_brake 服务控制关节抱闸",
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {"type": "string", "enum": ["engage", "release", "status"],
                               "default": "status",
                               "description": "engage=抱闸锁定, release=释放抱闸, status=查询状态"},
                    "joint_id": {"type": "number", "description": "关节ID（0=全部）"},
                },
                "required": ["action"],
            },
        }

    def dispatch(self, action: str, args: dict) -> dict:
        from std_srvs.srv import Trigger
        cli = self._sub_node.create_client(Trigger, "/control_brake")
        if not cli.wait_for_service(timeout_sec=3.0):
            return {"error": "/control_brake service not available"}
        future = cli.call_async(Trigger.Request())
        try:
            rclpy.spin_until_future_complete(self._sub_node, future, timeout_sec=5.0)
            resp = future.result()
            return {"ok": resp.success, "message": getattr(resp, "message", ""), "action": action} if resp else {"error": "timeout"}
        except Exception:
            return {"error": "call failed"}
        finally:
            try:
                self._sub_node.destroy_client(cli)
            except Exception:
                pass


# ══════════════════════════════════════════════════════════════════════════════
# TfPlugin 坐标系变换
# ══════════════════════════════════════════════════════════════════════════════
class TfPlugin(BasePlugin):
    """订阅 /tf 获取坐标系变换树."""

    def __init__(self, cfg, namespace, ros2):
        super().__init__(cfg, namespace, ros2)
        self._topic = f"/{namespace}/q5/tf"
        self._transforms = []

    def get_tool(self) -> dict:
        return {
            "name": "tf",
            "type": "sensor",
            "description": "Q5坐标系变换 — 实时TF变换数据，数据来源: /tf",
            "inputSchema": {"type": "object", "properties": {}},
            "topic_out": [{"topic": self._topic, "format": "data/json"}],
        }

    def start(self):
        super().start()
        self._pub = self._pub_node.create_publisher(String, self._topic, _RELIABLE_QOS)
        try:
            from tf2_msgs.msg import TFMessage
            self._sub_node.create_subscription(TFMessage, "/tf", self._on_tf, _LOW_LAT_QOS)
        except ImportError:
            self._sub_node.create_subscription(String, "/tf", self._on_tf_str, _LOW_LAT_QOS)
        threading.Thread(target=self._publish_loop, daemon=True).start()

    def _on_tf(self, msg):
        tfs = []
        for t in msg.transforms:
            tfs.append({
                "child_frame_id": t.child_frame_id,
                "header_frame_id": t.header.frame_id,
                "translation": {"x": t.transform.translation.x, "y": t.transform.translation.y, "z": t.transform.translation.z},
                "rotation": {"x": t.transform.rotation.x, "y": t.transform.rotation.y, "z": t.transform.rotation.z, "w": t.transform.rotation.w},
            })
        self._transforms = tfs

    def _on_tf_str(self, msg):
        try:
            self._transforms = json.loads(msg.data)
        except Exception:
            pass

    def _publish_loop(self):
        while self._running:
            try:
                self._pub.publish(String(data=json.dumps({"transforms": self._transforms, "timestamp": time.time()})))
            except Exception:
                pass
            time.sleep(0.1)

    def dispatch(self, action: str, args: dict) -> dict:
        return {"transforms": self._transforms}


# ══════════════════════════════════════════════════════════════════════════════
# RemoteControlPlugin 遥控指令
# ══════════════════════════════════════════════════════════════════════════════
class RemoteControlPlugin(BasePlugin):
    """遥控指令收发 — /send_remote/command (发布) + /remote_control/trigger_play (订阅)."""

    def __init__(self, cfg, namespace, ros2):
        super().__init__(cfg, namespace, ros2)
        self._trigger = {}

    def get_tool(self) -> dict:
        return {
            "name": "remote_control",
            "type": "actuator",
            "description": "Q5遥控指令 — 发送遥控指令到 /send_remote/command，订阅 /remote_control/trigger_play 触发播放",
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {"type": "string", "enum": ["send", "trigger_status"],
                               "default": "send",
                               "description": "send=发送遥控指令, trigger_status=查询触发状态"},
                    "command": {"type": "string", "description": "遥控指令内容（send时必填）"},
                },
                "required": ["action"],
            },
        }

    def start(self):
        super().start()
        self._pub = self._pub_node.create_publisher(String, "/send_remote/command", _RELIABLE_QOS)
        self._sub_node.create_subscription(String, "/remote_control/trigger_play", self._on_trigger, _LOW_LAT_QOS)

    def _on_trigger(self, msg):
        try:
            self._trigger = json.loads(msg.data)
        except Exception:
            self._trigger = {"raw": msg.data}

    def dispatch(self, action: str, args: dict) -> dict:
        if action == "send":
            cmd = args.get("command", "")
            self._pub.publish(String(data=json.dumps({"command": cmd})))
            return {"ok": True, "command": cmd}
        return {"trigger": self._trigger}


# ══════════════════════════════════════════════════════════════════════════════
# PowerPlugin 电源/关机控制
# ══════════════════════════════════════════════════════════════════════════════
class PowerPlugin(BasePlugin):
    """电源管理 — /shutdown_service + /ethercat_emergency."""

    def get_tool(self) -> dict:
        return {
            "name": "power",
            "type": "actuator",
            "description": "Q5电源管理 — shutdown关机/restart重启/ethercat_emergency急停（请谨慎使用）",
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {"type": "string", "enum": ["shutdown", "ethercat_emergency"],
                               "description": "shutdown=关机, ethercat_emergency=EtherCAT急停"},
                },
                "required": ["action"],
            },
        }

    def dispatch(self, action: str, args: dict) -> dict:
        from std_srvs.srv import Trigger
        svc_map = {
            "shutdown": "/shutdown_service",
            "ethercat_emergency": "/ethercat_emergency",
        }
        svc = svc_map.get(action)
        if not svc:
            return {"error": f"unknown action: {action}"}
        cli = self._sub_node.create_client(Trigger, svc)
        if not cli.wait_for_service(timeout_sec=3.0):
            return {"error": f"service {svc} not available"}
        future = cli.call_async(Trigger.Request())
        try:
            rclpy.spin_until_future_complete(self._sub_node, future, timeout_sec=5.0)
            resp = future.result()
            return {"ok": resp.success, "message": getattr(resp, "message", ""), "service": svc} if resp else {"error": "timeout"}
        except Exception:
            return {"error": "call failed"}
        finally:
            try:
                self._sub_node.destroy_client(cli)
            except Exception:
                pass
