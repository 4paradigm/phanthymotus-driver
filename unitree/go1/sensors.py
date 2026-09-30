"""
sensors.py — Go1 状态/资源卡聚合（battery, imu, feet, swing_trajectory, fall_alarm, obstacle_range,
             remote_controller, udp_diagnostics, loco_state, odometry, joints, model）。

自包含：一张合并文件 = 多张卡片。main.py 按 config.yaml 里的卡名手动 import 并 make_plugin()。
每张卡保持独立的 CARD / Plugin / make_plugin，只是合并在同一文件。
"""

from __future__ import annotations

import json
import math
import struct
import threading
import time
from collections import deque
from pathlib import Path

from go1_sdk_client import JOINT_NAMES, parse_wireless_remote

try:
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
    from std_msgs.msg import String, UInt8MultiArray
    _HAS_ROS2 = True
    _QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                      history=HistoryPolicy.KEEP_LAST, depth=1,
                      durability=DurabilityPolicy.VOLATILE)
except Exception:
    _HAS_ROS2 = False


# ============================================================================
# battery.py — Go1 电池(BMS) 状态卡
# ============================================================================

_CARD_BATTERY = "battery"
_TOPIC_BATTERY = "/{ns}/state/battery"
_HZ_BATTERY = 1.0
_NODE_BATTERY = "go1_battery"
_DESC_BATTERY = "Go1 BMS — SOC%/current/cycles/temps/cell voltages"


def _build_battery(snap: dict) -> dict:
    d = {"timestamp_ms": int(time.time() * 1000),
         "control_level": snap.get("control_level", "HIGHLEVEL"),
         "fresh": bool(snap.get("fresh", False))}
    bat = snap.get("battery")
    if bat is None:
        d["available"] = False
        return d
    d.update(bat)
    return d


class BatteryPlugin:
    def __init__(self, plugin_config, namespace, executor, client):
        self._client = client
        self._topic = _TOPIC_BATTERY.format(ns=namespace)
        self._node = None
        if _HAS_ROS2 and executor is not None:
            try:
                self._node = Node(_NODE_BATTERY)
                self._pub = self._node.create_publisher(String, self._topic, _QOS)
                self._node.create_timer(1.0 / _HZ_BATTERY, self._tick)
                executor.add_node(self._node)
                self._node.get_logger().info(f"go1 battery → {self._topic} @ {_HZ_BATTERY}Hz")
            except Exception as e:  # noqa: BLE001
                print(f"[{_CARD_BATTERY}] ROS2 发布不可用，退回 MCP 轮询: {e}", flush=True)
                self._node = None

    def _tick(self):
        try:
            m = String()
            m.data = json.dumps(_build_battery(self._client.snapshot()))
            self._pub.publish(m)
        except Exception as e:  # noqa: BLE001
            self._node.get_logger().error(f"publish {self._topic} error: {e}")

    def get_tool(self):
        desc = _DESC_BATTERY + (f" — → {self._topic}" if self._node else " — poll via MCP action=info")
        return {"name": _CARD_BATTERY, "type": "sensor", "multiInstance": False, "description": desc,
                "inputSchema": {"type": "object", "properties": {}},
                "topic_out": ([{"topic": self._topic, "format": "data/json"}] if self._node else [])}

    def start(self): pass
    def stop(self): pass

    def dispatch(self, action, args):
        if action == "start": return {"state": "running"}
        if action == "stop": return {"state": "idle"}
        if action in ("info", "read", "get", _CARD_BATTERY):
            return {"state": "running", "data": _build_battery(self._client.snapshot()),
                    "topic_out": ([{"topic": self._topic, "format": "data/json"}] if self._node else [])}
        return None


def make_battery(plugin_config, namespace, executor, client):
    return BatteryPlugin(plugin_config, namespace, executor, client)


# ============================================================================
# imu.py — Go1 IMU 状态卡
# ============================================================================

_CARD_IMU = "imu"
_TOPIC_IMU = "/{ns}/state/imu"
_HZ_IMU = 20.0
_NODE_IMU = "go1_imu"
_DESC_IMU = "Go1 IMU — quaternion(wxyz)/gyro/accel/rpy/temp; attitude_may_drift on acceleration"


def _build_imu(snap: dict) -> dict:
    d = {"timestamp_ms": int(time.time() * 1000),
         "control_level": snap.get("control_level", "HIGHLEVEL"),
         "fresh": bool(snap.get("fresh", False))}
    imu = snap.get("imu")
    if imu is None:
        d["available"] = False
        return d
    d.update(imu)
    d["attitude_may_drift"] = True
    return d


class ImuPlugin:
    def __init__(self, plugin_config, namespace, executor, client):
        self._client = client
        self._topic = _TOPIC_IMU.format(ns=namespace)
        self._node = None
        if _HAS_ROS2 and executor is not None:
            try:
                self._node = Node(_NODE_IMU)
                self._pub = self._node.create_publisher(String, self._topic, _QOS)
                self._node.create_timer(1.0 / _HZ_IMU, self._tick)
                executor.add_node(self._node)
                self._node.get_logger().info(f"go1 imu → {self._topic} @ {_HZ_IMU}Hz")
            except Exception as e:  # noqa: BLE001
                print(f"[{_CARD_IMU}] ROS2 发布不可用，退回 MCP 轮询: {e}", flush=True)
                self._node = None

    def _tick(self):
        try:
            m = String()
            m.data = json.dumps(_build_imu(self._client.snapshot()))
            self._pub.publish(m)
        except Exception as e:  # noqa: BLE001
            self._node.get_logger().error(f"publish {self._topic} error: {e}")

    def get_tool(self):
        desc = _DESC_IMU + (f" — → {self._topic}" if self._node else " — poll via MCP action=info")
        return {"name": _CARD_IMU, "type": "sensor", "multiInstance": False, "description": desc,
                "inputSchema": {"type": "object", "properties": {}},
                "topic_out": ([{"topic": self._topic, "format": "data/json"}] if self._node else [])}

    def start(self): pass
    def stop(self): pass

    def dispatch(self, action, args):
        if action == "start": return {"state": "running"}
        if action == "stop": return {"state": "idle"}
        if action in ("info", "read", "get", _CARD_IMU):
            return {"state": "running", "data": _build_imu(self._client.snapshot()),
                    "topic_out": ([{"topic": self._topic, "format": "data/json"}] if self._node else [])}
        return None


def make_imu(plugin_config, namespace, executor, client):
    return ImuPlugin(plugin_config, namespace, executor, client)


# ============================================================================
# feet.py — Go1 足端状态卡
# ============================================================================

_CARD_FEET = "feet"
_TOPIC_FEET = "/{ns}/state/feet"
_HZ_FEET = 10.0
_NODE_FEET = "go1_feet"
_DESC_FEET = "Go1 feet — foot force raw[4]; foot position/speed to body (HIGHLEVEL only)"
_FOOT_ORDER = ["FR", "FL", "RR", "RL"]


def _build_feet(snap: dict) -> dict:
    d = {"timestamp_ms": int(time.time() * 1000),
         "control_level": snap.get("control_level", "HIGHLEVEL"),
         "fresh": bool(snap.get("fresh", False)),
         "order": _FOOT_ORDER}
    d["foot_force_raw"] = snap.get("foot_force")
    fp = snap.get("foot_pos")
    if fp is not None:
        d["position_to_body"] = fp
        d["speed_to_body"] = snap.get("foot_speed")
    return d


class FeetPlugin:
    def __init__(self, plugin_config, namespace, executor, client):
        self._client = client
        self._topic = _TOPIC_FEET.format(ns=namespace)
        self._node = None
        if _HAS_ROS2 and executor is not None:
            try:
                self._node = Node(_NODE_FEET)
                self._pub = self._node.create_publisher(String, self._topic, _QOS)
                self._node.create_timer(1.0 / _HZ_FEET, self._tick)
                executor.add_node(self._node)
                self._node.get_logger().info(f"go1 feet → {self._topic} @ {_HZ_FEET}Hz")
            except Exception as e:  # noqa: BLE001
                print(f"[{_CARD_FEET}] ROS2 发布不可用，退回 MCP 轮询: {e}", flush=True)
                self._node = None

    def _tick(self):
        try:
            m = String()
            m.data = json.dumps(_build_feet(self._client.snapshot()))
            self._pub.publish(m)
        except Exception as e:  # noqa: BLE001
            self._node.get_logger().error(f"publish {self._topic} error: {e}")

    def get_tool(self):
        desc = _DESC_FEET + (f" — → {self._topic}" if self._node else " — poll via MCP action=info")
        return {"name": _CARD_FEET, "type": "sensor", "multiInstance": False, "description": desc,
                "inputSchema": {"type": "object", "properties": {}},
                "topic_out": ([{"topic": self._topic, "format": "data/json"}] if self._node else [])}

    def start(self): pass
    def stop(self): pass

    def dispatch(self, action, args):
        if action == "start": return {"state": "running"}
        if action == "stop": return {"state": "idle"}
        if action in ("info", "read", "get", _CARD_FEET):
            return {"state": "running", "data": _build_feet(self._client.snapshot()),
                    "topic_out": ([{"topic": self._topic, "format": "data/json"}] if self._node else [])}
        return None


def make_feet(plugin_config, namespace, executor, client):
    return FeetPlugin(plugin_config, namespace, executor, client)


# ============================================================================
# fall_alarm.py — Go1 跌倒/侧翻告警卡
# ============================================================================

_CARD_FALL = "fall_alarm"
_TOPIC_FALL = "/{ns}/state/fall_alarm"
_HZ_FALL = 10.0
_NODE_FALL = "go1_fall_alarm"
_DESC_FALL = "Go1 fall/tilt alarm — ok/tilted/fallen from IMU roll/pitch magnitude + tilt angle"
_WARN_DEFAULT = 0.6
_FALL_DEFAULT = 1.2


def _build_fall(snap: dict, warn: float, fall: float) -> dict:
    d = {"timestamp_ms": int(time.time() * 1000),
         "control_level": snap.get("control_level", "HIGHLEVEL"),
         "fresh": bool(snap.get("fresh", False))}
    imu = snap.get("imu") or {}
    rpy = imu.get("rpy_rad") or [0.0, 0.0, 0.0]
    roll = float(rpy[0]) if len(rpy) > 0 else 0.0
    pitch = float(rpy[1]) if len(rpy) > 1 else 0.0
    tilt = max(abs(roll), abs(pitch))
    if tilt >= fall:
        status, hint = "fallen", "已跌倒/翻倒 —— 立即停止运动，需人工扶正后再操作"
    elif tilt >= warn:
        status, hint = "tilted", "机身明显倾斜 —— 谨慎，可能即将失稳"
    else:
        status, hint = "ok", "姿态正常"
    d.update({"status": status,
              "roll_rad": round(roll, 4), "pitch_rad": round(pitch, 4),
              "roll_deg": round(math.degrees(roll), 1), "pitch_deg": round(math.degrees(pitch), 1),
              "tilt_warn_rad": warn, "fall_rad": fall, "hint": hint})
    return d


class FallAlarmPlugin:
    def __init__(self, plugin_config, namespace, executor, client):
        self._client = client
        cfg = plugin_config or {}
        self._warn = float(cfg.get("tilt_warn_rad", _WARN_DEFAULT))
        self._fall = float(cfg.get("fall_rad", _FALL_DEFAULT))
        self._topic = _TOPIC_FALL.format(ns=namespace)
        self._node = None
        if _HAS_ROS2 and executor is not None:
            try:
                self._node = Node(_NODE_FALL)
                self._pub = self._node.create_publisher(String, self._topic, _QOS)
                self._node.create_timer(1.0 / _HZ_FALL, self._tick)
                executor.add_node(self._node)
                self._node.get_logger().info(f"go1 fall_alarm → {self._topic} @ {_HZ_FALL}Hz")
            except Exception as e:  # noqa: BLE001
                print(f"[{_CARD_FALL}] ROS2 发布不可用，退回 MCP 轮询: {e}", flush=True)
                self._node = None

    def _tick(self):
        try:
            m = String()
            m.data = json.dumps(_build_fall(self._client.snapshot(), self._warn, self._fall))
            self._pub.publish(m)
        except Exception as e:  # noqa: BLE001
            self._node.get_logger().error(f"publish {self._topic} error: {e}")

    def get_tool(self):
        desc = _DESC_FALL + (f" — → {self._topic}" if self._node else " — poll via MCP action=info")
        return {"name": _CARD_FALL, "type": "sensor", "multiInstance": False, "description": desc,
                "inputSchema": {"type": "object", "properties": {}},
                "topic_out": ([{"topic": self._topic, "format": "data/json"}] if self._node else [])}

    def start(self): pass
    def stop(self): pass

    def dispatch(self, action, args):
        if action == "start": return {"state": "running"}
        if action == "stop": return {"state": "idle"}
        if action in ("info", "read", "get", _CARD_FALL):
            return {"state": "running", "data": _build_fall(self._client.snapshot(), self._warn, self._fall),
                    "topic_out": ([{"topic": self._topic, "format": "data/json"}] if self._node else [])}
        return None


def make_fall_alarm(plugin_config, namespace, executor, client):
    return FallAlarmPlugin(plugin_config, namespace, executor, client)


# ============================================================================
# obstacle_range.py — Go1 超声波避障状态卡
# ============================================================================

_CARD_OBSTACLE = "obstacle_range"
_TOPIC_OBSTACLE = "/{ns}/state/obstacle_range"
_HZ_OBSTACLE = 10.0
_NODE_OBSTACLE = "go1_obstacle"
_DESC_OBSTACLE = "Go1 nearest-obstacle raw ranges[4] (direction/unit undocumented; read-only)"


def _build_obstacle(snap: dict) -> dict:
    d = {"timestamp_ms": int(time.time() * 1000),
         "control_level": snap.get("control_level", "HIGHLEVEL"),
         "fresh": bool(snap.get("fresh", False))}
    ro = snap.get("range_obstacle")
    d.update({"available": ro is not None, "range_raw": ro,
              "direction_mapping": "undocumented", "unit": "undocumented"})
    return d


class ObstacleRangePlugin:
    def __init__(self, plugin_config, namespace, executor, client):
        self._client = client
        self._topic = _TOPIC_OBSTACLE.format(ns=namespace)
        self._node = None
        if _HAS_ROS2 and executor is not None:
            try:
                self._node = Node(_NODE_OBSTACLE)
                self._pub = self._node.create_publisher(String, self._topic, _QOS)
                self._node.create_timer(1.0 / _HZ_OBSTACLE, self._tick)
                executor.add_node(self._node)
                self._node.get_logger().info(f"go1 obstacle → {self._topic} @ {_HZ_OBSTACLE}Hz")
            except Exception as e:  # noqa: BLE001
                print(f"[{_CARD_OBSTACLE}] ROS2 发布不可用，退回 MCP 轮询: {e}", flush=True)
                self._node = None

    def _tick(self):
        try:
            m = String()
            m.data = json.dumps(_build_obstacle(self._client.snapshot()))
            self._pub.publish(m)
        except Exception as e:  # noqa: BLE001
            self._node.get_logger().error(f"publish {self._topic} error: {e}")

    def get_tool(self):
        desc = _DESC_OBSTACLE + (f" — → {self._topic}" if self._node else " — poll via MCP action=info")
        return {"name": _CARD_OBSTACLE, "type": "sensor", "multiInstance": False, "description": desc,
                "inputSchema": {"type": "object", "properties": {}},
                "topic_out": ([{"topic": self._topic, "format": "data/json"}] if self._node else [])}

    def start(self): pass
    def stop(self): pass

    def dispatch(self, action, args):
        if action == "start": return {"state": "running"}
        if action == "stop": return {"state": "idle"}
        if action in ("info", "read", "get", _CARD_OBSTACLE):
            return {"state": "running", "data": _build_obstacle(self._client.snapshot()),
                    "topic_out": ([{"topic": self._topic, "format": "data/json"}] if self._node else [])}
        return None


def make_obstacle_range(plugin_config, namespace, executor, client):
    return ObstacleRangePlugin(plugin_config, namespace, executor, client)


# ============================================================================
# remote_controller.py — Go1 无线遥控器状态卡
# ============================================================================

_CARD_REMOTE = "remote_controller"
_TOPIC_REMOTE = "/{ns}/state/remote_controller"
_HZ_REMOTE = 10.0
_NODE_REMOTE = "go1_remote_controller"
_DESC_REMOTE = ("Go1 wireless remote — 16 buttons (R1/L1/start/select/R2/L2/F1/F2/A/B/X/Y/up/right/down/left) "
                "+ 5 axes (lx/rx/ry/ly/L2) from HighState.wirelessRemote[40]")


def _build_remote(snap: dict) -> dict:
    d = {"timestamp_ms": int(time.time() * 1000),
         "control_level": snap.get("control_level", "HIGHLEVEL"),
         "fresh": bool(snap.get("fresh", False))}
    raw = snap.get("wireless_remote")
    if raw is None:
        d["available"] = False
        return d
    d.update(parse_wireless_remote(raw))
    return d


class RemoteControllerPlugin:
    def __init__(self, plugin_config, namespace, executor, client):
        self._client = client
        self._topic = _TOPIC_REMOTE.format(ns=namespace)
        self._node = None
        if _HAS_ROS2 and executor is not None:
            try:
                self._node = Node(_NODE_REMOTE)
                self._pub = self._node.create_publisher(String, self._topic, _QOS)
                self._node.create_timer(1.0 / _HZ_REMOTE, self._tick)
                executor.add_node(self._node)
                self._node.get_logger().info(f"go1 remote_controller → {self._topic} @ {_HZ_REMOTE}Hz")
            except Exception as e:  # noqa: BLE001
                print(f"[{_CARD_REMOTE}] ROS2 发布不可用，退回 MCP 轮询: {e}", flush=True)
                self._node = None

    def _tick(self):
        try:
            m = String()
            m.data = json.dumps(_build_remote(self._client.snapshot()))
            self._pub.publish(m)
        except Exception as e:  # noqa: BLE001
            self._node.get_logger().error(f"publish {self._topic} error: {e}")

    def get_tool(self):
        desc = _DESC_REMOTE + (f" — → {self._topic}" if self._node else " — poll via MCP action=info")
        return {"name": _CARD_REMOTE, "type": "sensor", "multiInstance": False, "description": desc,
                "inputSchema": {"type": "object", "properties": {}},
                "topic_out": ([{"topic": self._topic, "format": "data/json"}] if self._node else [])}

    def start(self): pass
    def stop(self): pass

    def dispatch(self, action, args):
        if action == "start": return {"state": "running"}
        if action == "stop": return {"state": "idle"}
        if action in ("info", "read", "get", _CARD_REMOTE):
            return {"state": "running", "data": _build_remote(self._client.snapshot()),
                    "topic_out": ([{"topic": self._topic, "format": "data/json"}] if self._node else [])}
        return None


def make_remote_controller(plugin_config, namespace, executor, client):
    return RemoteControllerPlugin(plugin_config, namespace, executor, client)


# ============================================================================
# udp_diagnostics.py — Go1 UDP 通信健康状态卡
# ============================================================================

_CARD_UDP = "udp_diagnostics"
_TOPIC_UDP = "/{ns}/state/udp_diagnostics"
_HZ_UDP = 1.0
_NODE_UDP = "go1_udp_diagnostics"
_DESC_UDP = "Go1 UDP link health — send/recv counts and CRC/lose/flag error counters"


def _build_udp(client) -> dict:
    snap = client.snapshot()
    d = {"timestamp_ms": int(time.time() * 1000),
         "control_level": snap.get("control_level", "HIGHLEVEL"),
         "fresh": bool(snap.get("fresh", False))}
    diag = client.diagnostics()
    d.update(diag)
    return d


class UdpDiagnosticsPlugin:
    def __init__(self, plugin_config, namespace, executor, client):
        self._client = client
        self._topic = _TOPIC_UDP.format(ns=namespace)
        self._node = None
        if _HAS_ROS2 and executor is not None:
            try:
                self._node = Node(_NODE_UDP)
                self._pub = self._node.create_publisher(String, self._topic, _QOS)
                self._node.create_timer(1.0 / _HZ_UDP, self._tick)
                executor.add_node(self._node)
                self._node.get_logger().info(f"go1 udp_diagnostics → {self._topic} @ {_HZ_UDP}Hz")
            except Exception as e:  # noqa: BLE001
                print(f"[{_CARD_UDP}] ROS2 发布不可用，退回 MCP 轮询: {e}", flush=True)
                self._node = None

    def _tick(self):
        try:
            m = String()
            m.data = json.dumps(_build_udp(self._client))
            self._pub.publish(m)
        except Exception as e:  # noqa: BLE001
            self._node.get_logger().error(f"publish {self._topic} error: {e}")

    def get_tool(self):
        desc = _DESC_UDP + (f" — → {self._topic}" if self._node else " — poll via MCP action=info")
        return {"name": _CARD_UDP, "type": "sensor", "multiInstance": False, "description": desc,
                "inputSchema": {"type": "object", "properties": {}},
                "topic_out": ([{"topic": self._topic, "format": "data/json"}] if self._node else [])}

    def start(self): pass
    def stop(self): pass

    def dispatch(self, action, args):
        if action == "start": return {"state": "running"}
        if action == "stop": return {"state": "idle"}
        if action in ("info", "read", "get", _CARD_UDP):
            return {"state": "running", "data": _build_udp(self._client),
                    "topic_out": ([{"topic": self._topic, "format": "data/json"}] if self._node else [])}
        return None


def make_udp_diagnostics(plugin_config, namespace, executor, client):
    return UdpDiagnosticsPlugin(plugin_config, namespace, executor, client)


# ============================================================================
# loco_state.py — Go1 运动状态卡
# ============================================================================

_CARD_LOCO_STATE = "loco_state"
_TOPIC_LOCO_STATE = "/{ns}/loco/state"
_HZ_LOCO_STATE = 10.0
_NODE_LOCO_STATE = "go1_loco_state"
_DESC_LOCO_STATE = "Go1 locomotion state — mode/gait/odometry/velocity/body_height (HighState)"


def _build_loco_state(snap: dict) -> dict:
    d = {"timestamp_ms": int(time.time() * 1000),
         "control_level": snap.get("control_level", "HIGHLEVEL"),
         "fresh": bool(snap.get("fresh", False))}
    vel = snap.get("velocity") or [0.0, 0.0, 0.0]
    d.update({
        "mode": snap.get("mode", 0), "mode_name": snap.get("mode_name", "unknown"),
        "gait_type": snap.get("gait_type", 0), "gait_name": snap.get("gait_name", "unknown"),
        "foot_raise_height_m": snap.get("foot_raise_height", 0.0),
        "position_m": snap.get("position"), "body_height_m": snap.get("body_height", 0.0),
        "velocity_body_mps": {"forward": vel[0] if len(vel) > 0 else 0.0,
                              "lateral": vel[1] if len(vel) > 1 else 0.0},
        "velocity_index_2_raw": vel[2] if len(vel) > 2 else 0.0,
        "yaw_speed_rad_s": snap.get("yaw_speed", 0.0),
    })
    return d


class LocoStatePlugin:
    def __init__(self, plugin_config, namespace, executor, client):
        self._client = client
        self._topic = _TOPIC_LOCO_STATE.format(ns=namespace)
        self._node = None
        if _HAS_ROS2 and executor is not None:
            try:
                self._node = Node(_NODE_LOCO_STATE)
                self._pub = self._node.create_publisher(String, self._topic, _QOS)
                self._node.create_timer(1.0 / _HZ_LOCO_STATE, self._tick)
                executor.add_node(self._node)
                self._node.get_logger().info(f"go1 loco_state → {self._topic} @ {_HZ_LOCO_STATE}Hz")
            except Exception as e:  # noqa: BLE001
                print(f"[{_CARD_LOCO_STATE}] ROS2 发布不可用，退回 MCP 轮询: {e}", flush=True)
                self._node = None

    def _tick(self):
        try:
            m = String()
            m.data = json.dumps(_build_loco_state(self._client.snapshot()))
            self._pub.publish(m)
        except Exception as e:  # noqa: BLE001
            self._node.get_logger().error(f"publish {self._topic} error: {e}")

    def get_tool(self):
        desc = _DESC_LOCO_STATE + (f" — → {self._topic}" if self._node else " — poll via MCP action=info")
        return {"name": _CARD_LOCO_STATE, "type": "sensor", "multiInstance": False, "description": desc,
                "inputSchema": {"type": "object", "properties": {}},
                "topic_out": ([{"topic": self._topic, "format": "data/json"}] if self._node else [])}

    def start(self): pass
    def stop(self): pass

    def dispatch(self, action, args):
        if action == "start": return {"state": "running"}
        if action == "stop": return {"state": "idle"}
        if action in ("info", "read", "get", _CARD_LOCO_STATE):
            return {"state": "running", "data": _build_loco_state(self._client.snapshot()),
                    "topic_out": ([{"topic": self._topic, "format": "data/json"}] if self._node else [])}
        return None


def make_loco_state(plugin_config, namespace, executor, client):
    return LocoStatePlugin(plugin_config, namespace, executor, client)


# ============================================================================
# odometry.py — Go1 里程计卡
# ============================================================================

_CARD_ODOMETRY = "odometry"
_TOPIC_ODOMETRY = "/{ns}/state/odometry"
_HZ_ODOMETRY = 5.0
_NODE_ODOMETRY = "go1_odometry"
_DESC_ODOMETRY = ("Go1 odometry — position/yaw + displacement from origin; "
                  "action=read 读取。origin 取首帧。HIGHLEVEL only.")


def _ms() -> int:
    return int(time.time() * 1000)


class OdometryPlugin:
    def __init__(self, plugin_config, namespace, executor, client):
        self._client = client
        self._origin = None
        self._topic = _TOPIC_ODOMETRY.format(ns=namespace)
        self._node = None
        if _HAS_ROS2 and executor is not None:
            try:
                self._node = Node(_NODE_ODOMETRY)
                self._pub = self._node.create_publisher(String, self._topic, _QOS)
                self._node.create_timer(1.0 / _HZ_ODOMETRY, self._tick)
                executor.add_node(self._node)
                self._node.get_logger().info(f"go1 odometry → {self._topic} @ {_HZ_ODOMETRY}Hz")
            except Exception as e:  # noqa: BLE001
                print(f"[{_CARD_ODOMETRY}] ROS2 发布不可用，退回 MCP 轮询: {e}", flush=True)
                self._node = None

    def _build(self) -> dict:
        snap = self._client.snapshot()
        d = {"timestamp_ms": _ms(),
             "control_level": snap.get("control_level", "HIGHLEVEL"),
             "fresh": bool(snap.get("fresh", False))}
        pos = snap.get("position") or [0.0, 0.0, 0.0]
        imu = snap.get("imu") or {}
        rpy = imu.get("rpy_rad") or [0.0, 0.0, 0.0]
        yaw = float(rpy[2]) if len(rpy) > 2 else 0.0
        if self._origin is None:
            self._origin = [pos[0], pos[1]]
        dx = pos[0] - self._origin[0]
        dy = pos[1] - self._origin[1]
        d.update({"position_m": pos, "yaw_rad": round(yaw, 4),
                  "origin_m": list(self._origin),
                  "displacement_m": {"dx": round(dx, 3), "dy": round(dy, 3),
                                     "distance": round(math.hypot(dx, dy), 3)}})
        return d

    def _tick(self):
        try:
            m = String()
            m.data = json.dumps(self._build())
            self._pub.publish(m)
        except Exception as e:  # noqa: BLE001
            self._node.get_logger().error(f"publish {self._topic} error: {e}")

    def get_tool(self):
        desc = _DESC_ODOMETRY + (f" — → {self._topic}" if self._node else " — poll via MCP action=read")
        return {"name": _CARD_ODOMETRY, "type": "sensor", "multiInstance": False, "description": desc,
                "inputSchema": {"type": "object", "properties": {}},
                "topic_out": ([{"topic": self._topic, "format": "data/json"}] if self._node else [])}

    def start(self): pass
    def stop(self): pass

    def dispatch(self, action, args):
        if action == "start": return {"state": "running"}
        if action == "stop": return {"state": "idle"}
        if action in ("info", "read", "get", _CARD_ODOMETRY):
            return {"state": "running", "data": self._build(),
                    "topic_out": ([{"topic": self._topic, "format": "data/json"}] if self._node else [])}
        return None


def make_odometry(plugin_config, namespace, executor, client):
    return OdometryPlugin(plugin_config, namespace, executor, client)


# ============================================================================
# joints.py — Go1 12 腿关节状态卡
# ============================================================================

_CARD_JOINTS = "joints"
_TOPIC_JOINTS = "/{ns}/state/joints"
_HZ_JOINTS = 10.0
_NODE_JOINTS = "go1_joints"
_DESC_JOINTS = ("Go1 12 leg-joint states (q/dq/tau/temp) as skeleton; "
                "needs model card (URDF) to render as quadruped")


def _build_joints(snap: dict) -> dict:
    d = {"timestamp_ms": int(time.time() * 1000),
         "control_level": snap.get("control_level", "HIGHLEVEL"),
         "fresh": bool(snap.get("fresh", False)),
         "format": "sensor/skeleton"}
    js = snap.get("joints")
    if not js:
        d["available"] = False
        return d
    joints = []
    for j in js:
        i = int(j.get("i", 0))
        name = JOINT_NAMES[i] if 0 <= i < len(JOINT_NAMES) else f"joint_{i}"
        q = j.get("q", 0.0)
        joints.append({
            "idx": i, "name": name,
            "q": q, "q_rad": q,
            "dq": j.get("dq", 0.0),
            "tau": j.get("tau", 0.0),
            "ddq_rad_s2": j.get("ddq", 0.0),
            "mode": j.get("mode", 0),
            "temperature_c": j.get("temp", 0),
        })
    d["available"] = True
    d["joints"] = joints
    imu = snap.get("imu") or {}
    d["imu_quat"] = imu.get("quaternion_wxyz", [1.0, 0.0, 0.0, 0.0])
    return d


class JointsPlugin:
    def __init__(self, plugin_config, namespace, executor, client):
        self._client = client
        self._topic = _TOPIC_JOINTS.format(ns=namespace)
        self._node = None
        if _HAS_ROS2 and executor is not None:
            try:
                self._node = Node(_NODE_JOINTS)
                self._pub = self._node.create_publisher(String, self._topic, _QOS)
                self._node.create_timer(1.0 / _HZ_JOINTS, self._tick)
                executor.add_node(self._node)
                self._node.get_logger().info(f"go1 joints → {self._topic} @ {_HZ_JOINTS}Hz")
            except Exception as e:  # noqa: BLE001
                print(f"[{_CARD_JOINTS}] ROS2 发布不可用，退回 MCP 轮询: {e}", flush=True)
                self._node = None

    def _tick(self):
        try:
            m = String()
            m.data = json.dumps(_build_joints(self._client.snapshot()))
            self._pub.publish(m)
        except Exception as e:  # noqa: BLE001
            self._node.get_logger().error(f"publish {self._topic} error: {e}")

    def get_tool(self):
        desc = _DESC_JOINTS + (f" — → {self._topic}" if self._node else " — poll via MCP action=info")
        return {"name": _CARD_JOINTS, "type": "sensor", "multiInstance": False, "description": desc,
                "inputSchema": {"type": "object", "properties": {}},
                "topic_out": ([{"topic": self._topic, "format": "sensor/skeleton"}] if self._node else [])}

    def start(self): pass
    def stop(self): pass

    def dispatch(self, action, args):
        if action == "start": return {"state": "running"}
        if action == "stop": return {"state": "idle"}
        if action in ("info", "read", "get", _CARD_JOINTS):
            return {"state": "running", "data": _build_joints(self._client.snapshot()),
                    "topic_out": ([{"topic": self._topic, "format": "sensor/skeleton"}] if self._node else [])}
        return None


def make_joints(plugin_config, namespace, executor, client):
    return JointsPlugin(plugin_config, namespace, executor, client)


# ============================================================================
# model.py — Go1 骨骼模型卡 (resource)
# ============================================================================

_CARD_MODEL = "model"
_URDF_PATH = Path(__file__).parent / "resource" / "go1_model.urdf"


class ModelPlugin:
    def __init__(self, plugin_config, namespace, executor, client):
        pass

    def get_tool(self):
        return {"name": _CARD_MODEL, "type": "resource",
                "description": "Go1 quadruped URDF model for skeleton renderer",
                "inputSchema": {"type": "object", "properties": {}}}

    def start(self): pass
    def stop(self): pass

    def dispatch(self, tool_name, args):
        try:
            return {"urdf": _URDF_PATH.read_text()}
        except Exception as e:  # noqa: BLE001
            return {"error": f"go1_model.urdf not found: {e}"}


def make_model(plugin_config, namespace, executor, client):
    return ModelPlugin(plugin_config, namespace, executor, client)


# ============================================================================
# activity_monitor.py — Go1 活动度统计卡 (actuator, read-only)
# ============================================================================

_CARD_ACTIVITY = "activity_monitor"
_TOPIC_ACTIVITY = "/{ns}/state/activity"
_HZ_ACTIVITY_TOPIC = 0.2       # ROS2 发布频率 (5 秒一次)
_NODE_ACTIVITY = "go1_activity_monitor"
_DESC_ACTIVITY = (
    "Go1 activity statistics. A background sampler records horizontal speed "
    "and motion state from HighState. action=report returns both last_30s "
    "and since_start snapshots. Read-only, no robot control commands."
)

_HZ = 5.0
_RECENT_WINDOW_SEC = 30.0
_HISTORY_SEC = 86400.0
_MOVE_EPS = 0.05
_MAX_INTEGRATION_DT = 2.0


def _classify(moving_ratio):
    if moving_ratio < 0.1:
        return "idle"
    if moving_ratio < 0.5:
        return "light"
    return "active"


def _speed_mag(vel):
    """Return horizontal speed magnitude from a velocity dict/list."""
    if vel is None:
        return None
    try:
        if isinstance(vel, dict):
            vx = float(vel.get("vx", vel.get("x", 0.0)) or 0.0)
            vy = float(vel.get("vy", vel.get("y", 0.0)) or 0.0)
        elif isinstance(vel, (list, tuple)) and len(vel) >= 2:
            vx, vy = float(vel[0] or 0.0), float(vel[1] or 0.0)
        else:
            return None
        return math.hypot(vx, vy)
    except (TypeError, ValueError):
        return None


class ActivityMonitorPlugin:
    def __init__(self, plugin_config, namespace, executor, client):
        self._client = client
        c = plugin_config or {}
        self._hz = float(c.get("sample_hz", _HZ))
        self._recent_window_sec = float(c.get("recent_window_sec", _RECENT_WINDOW_SEC))
        self._history_sec = float(c.get("history_sec", _HISTORY_SEC))
        self._move_eps = float(c.get("move_eps", _MOVE_EPS))
        self._max_integration_dt = float(c.get("max_integration_dt", _MAX_INTEGRATION_DT))

        maxlen = max(2, int(self._hz * self._history_sec))
        self._buf = deque(maxlen=maxlen)
        self._lock = threading.Lock()
        self._running = False
        self._thread = None
        self._started_at = time.time()

        self._topic = _TOPIC_ACTIVITY.format(ns=namespace)
        self._node = None
        if _HAS_ROS2 and executor is not None:
            try:
                self._node = Node(_NODE_ACTIVITY)
                self._pub = self._node.create_publisher(String, self._topic, _QOS)
                self._node.create_timer(1.0 / _HZ_ACTIVITY_TOPIC, self._tick)
                executor.add_node(self._node)
                self._node.get_logger().info(
                    f"go1 activity_monitor → {self._topic} @ {_HZ_ACTIVITY_TOPIC}Hz")
            except Exception as e:  # noqa: BLE001
                print(f"[{_CARD_ACTIVITY}] ROS2 发布不可用，退回 MCP 轮询: {e}", flush=True)
                self._node = None

    def start(self):
        if self._running:
            return
        self._running = True
        self._started_at = time.time()
        self._thread = threading.Thread(target=self._sample_loop, daemon=True)
        self._thread.start()
        print(f"[activity_monitor] sampler started ({self._hz}Hz, "
              f"recent_window={self._recent_window_sec}s, history={self._history_sec}s)",
              flush=True)

    def stop(self):
        self._running = False
        t = self._thread
        if t and t.is_alive():
            t.join(timeout=2.0)
        self._thread = None

    def _sample_loop(self):
        period = 1.0 / self._hz if self._hz > 0 else 0.2
        while self._running:
            ts = time.time()
            try:
                snap = self._client.snapshot() if self._client else None
            except Exception:  # noqa: BLE001
                snap = None

            speed, effective_speed, moving, mode = None, 0.0, False, "unknown"
            if snap and snap.get("fresh", False):
                speed = _speed_mag(snap.get("velocity"))
                mode = str(snap.get("mode_name", "unknown"))
                if speed is not None:
                    moving = speed >= self._move_eps
                    effective_speed = speed if moving else 0.0

            if speed is not None:
                with self._lock:
                    self._buf.append((ts, speed, effective_speed, moving, mode))

            dt = time.time() - ts
            time.sleep(max(0.0, period - dt))

    def _select_samples(self, window_sec):
        now = time.time()
        with self._lock:
            samples = list(self._buf)
        if window_sec is None:
            return samples
        cutoff = now - window_sec
        return [s for s in samples if s[0] >= cutoff]

    def _compute_stats(self, samples, label):
        if len(samples) < 2:
            return {
                "range": label, "verdict": "no_data", "window_sec": 0.0,
                "sample_count": len(samples), "distance_m": 0.0,
                "moving_ratio": 0.0, "idle_sec": 0.0, "avg_speed": 0.0,
                "peak_speed": 0.0,
                "summary": "not enough fresh HighState samples yet",
            }

        t0, tN = samples[0][0], samples[-1][0]
        actual_window_sec = max(0.0, tN - t0)

        distance = 0.0
        moving_cnt = 0
        peak = 0.0

        for i, (ts, raw_speed, effective_speed, moving, mode) in enumerate(samples):
            peak = max(peak, raw_speed)
            if moving:
                moving_cnt += 1
            if i > 0:
                prev_ts, _pr, prev_eff, _pm, _pmd = samples[i - 1]
                dt = ts - prev_ts
                if 0 < dt <= self._max_integration_dt:
                    distance += 0.5 * (prev_eff + effective_speed) * dt

        n = len(samples)
        moving_ratio = moving_cnt / n
        idle_sec = round((1.0 - moving_ratio) * actual_window_sec, 1)
        avg_speed = distance / actual_window_sec if actual_window_sec > 0 else 0.0
        verdict = _classify(moving_ratio)

        return {
            "range": label, "verdict": verdict,
            "window_sec": round(actual_window_sec, 1), "sample_count": n,
            "distance_m": round(distance, 2),
            "moving_ratio": round(moving_ratio, 2), "idle_sec": idle_sec,
            "avg_speed": round(avg_speed, 3), "peak_speed": round(peak, 3),
            "summary": (f"{label}: {verdict}, moved ~{round(distance, 2)}m, "
                        f"{int(moving_ratio * 100)}% of time moving, "
                        f"idle {idle_sec}s, peak {round(peak, 2)}m/s"),
        }

    def _report(self):
        now_samples = self._select_samples(None)
        recent_samples = self._select_samples(self._recent_window_sec)
        current_mode = now_samples[-1][4] if now_samples else "unknown"
        return {
            "ok": True, "action": "report", "card": _CARD_ACTIVITY,
            "control_level": "HIGHLEVEL",
            "timestamp_ms": int(time.time() * 1000),
            "current_mode": current_mode, "move_eps": self._move_eps,
            "last_30s": self._compute_stats(recent_samples, "last_30s"),
            "since_start": self._compute_stats(now_samples, "since_start"),
        }

    def _build_for_topic(self):
        return self._report()

    def _tick(self):
        try:
            m = String()
            m.data = json.dumps(self._build_for_topic())
            self._pub.publish(m)
        except Exception as e:  # noqa: BLE001
            self._node.get_logger().error(f"publish {self._topic} error: {e}")

    def get_tool(self):
        desc = _DESC_ACTIVITY + (
            f" — → {self._topic}" if self._node else " — poll via MCP action=report")
        return {
            "name": _CARD_ACTIVITY, "type": "actuator",
            "multiInstance": False, "description": desc,
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {
                        "type": "string",
                        "enum": ["report"],
                        "description": "report = returns both last_30s and since_start activity stats",
                    }
                },
                "required": ["action"],
            },
            "topic_out": (
                [{"topic": self._topic, "format": "data/json"}] if self._node else []),
        }

    def dispatch(self, action, args):
        if action == "start":
            return {"state": "running"}
        if action == "stop":
            return {"state": "idle"}
        if action in ("info", "read", "get", _CARD_ACTIVITY):
            return {"state": "running", "data": self._report(),
                    "topic_out": ([{"topic": self._topic, "format": "data/json"}] if self._node else [])}
        if action == "report":
            return self._report()
        return None


def make_activity_monitor(plugin_config, namespace, executor, client):
    return ActivityMonitorPlugin(plugin_config, namespace, executor, client)


# ============================================================================
# swing_trajectory — Go1 足端轨迹卡
# ============================================================================

_SWING_FEET = ("FR", "FL", "RR", "RL")
_SWING_FORMAT = "data/json"


def _swing_vector(value):
    try:
        if isinstance(value, dict):
            values = [value[k] for k in ("x", "y", "z")]
        else:
            values = list(value)
        if len(values) != 3:
            return None
        values = [float(v) for v in values]
        return values if all(math.isfinite(v) for v in values) else None
    except (KeyError, TypeError, ValueError):
        return None


def _swing_world_point(body, position, quaternion):
    """p_world = p_body_odom + R_world_from_body(q_wxyz) p_body."""
    p = _swing_vector(position)
    try:
        q = [float(v) for v in quaternion]
        if len(q) != 4 or not all(math.isfinite(v) for v in q):
            return None
        norm = math.sqrt(sum(v * v for v in q))
        if not 0.5 <= norm <= 1.5:
            return None
        w, x, y, z = (v / norm for v in q)
    except (TypeError, ValueError):
        return None
    if p is None:
        return None
    bx, by, bz = body
    return [round(p[0] + (1 - 2 * (y*y + z*z))*bx + 2*(x*y - w*z)*by + 2*(x*z + w*y)*bz, 4),
            round(p[1] + 2*(x*y + w*z)*bx + (1 - 2*(x*x + z*z))*by + 2*(y*z - w*x)*bz, 4),
            round(p[2] + 2*(x*z - w*y)*bx + 2*(y*z + w*x)*by + (1 - 2*(x*x + y*y))*bz, 4)]


def _swing_pointcloud(data, frame):
    """Encode current/last swing paths for Agent Core's existing 3D point renderer.

    The renderer maps wire (x,y,z) to display (y,-z,-x), so the wire
    coordinates are remapped to show forward, left and up with up positive.
    Canonical robot coordinates remain in the JSON topic.
    """
    key = "body_xyz_m" if frame == "body" else "world_xyz_m"
    points = []
    for foot in _SWING_FEET:
        state = data["feet"][foot]
        path = state["active"] or state["last_completed"]
        previous = None
        for sample in path:
            point = _swing_vector(sample.get(key))
            if point is None:
                previous = None
                continue
            if previous is not None:
                distance = math.dist(previous, point)
                subdivisions = min(100, max(1, math.ceil(distance / 0.01)))
                for step in range(1, subdivisions):
                    fraction = step / subdivisions
                    points.append([a + (b - a) * fraction for a, b in zip(previous, point)])
            points.append(point)
            previous = point
    # One origin marker clears the preceding frame when there is no path.
    if not points:
        points = [[0.0, 0.0, 0.0]]
    points = points[:40000]
    payload = bytearray(struct.pack("<II", 12, len(points)))
    for x, y, z in points:
        payload.extend(struct.pack("<fff", -x, y, -z))
    return bytes(payload)


class SwingTrajectoryPlugin:
    def __init__(self, plugin_config, namespace, executor, client):
        cfg = plugin_config or {}
        self._client = client
        self._topic = f"/{namespace}/state/swing_trajectory"
        self._cloud_topics = {
            "body": f"/{namespace}/state/swing_trajectory_body_3d",
            "world": f"/{namespace}/state/swing_trajectory_world_3d",
        }
        self._hz = max(1.0, min(50.0, float(cfg.get("sample_hz", 20))))
        self._force = float(cfg.get("contact_force_threshold_raw", 20))
        self._hysteresis = max(0.0, float(cfg.get("force_hysteresis_raw", 10)))
        self._confirm = max(1, min(10, int(cfg.get("phase_confirm_samples", 2))))
        self._max_points = max(2, min(500, int(cfg.get("max_points_per_swing", 150))))
        self._max_age = max(0.05, float(cfg.get("max_sample_age_s", 0.5)))
        self._lock = threading.Lock()
        self._lifecycle_lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = None
        self._last_sample = None
        self._last_source_stamp = None
        self._last_signature = None
        self._last_status = "waiting_for_telemetry"
        self._feet = {name: {"phase": "unknown", "candidate": None, "candidate_count": 0,
                             "active": [], "last_completed": [], "swing_id": 0,
                             "swing_start_s": None} for name in _SWING_FEET}
        self._node = None
        if _HAS_ROS2 and executor is not None:
            try:
                self._node = Node("go1_swing_trajectory")
                qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                                 history=HistoryPolicy.KEEP_LAST, depth=1,
                                 durability=DurabilityPolicy.VOLATILE)
                self._pub = self._node.create_publisher(String, self._topic, qos)
                self._cloud_pubs = {
                    frame: self._node.create_publisher(UInt8MultiArray, topic, qos)
                    for frame, topic in self._cloud_topics.items()
                }
                executor.add_node(self._node)
            except Exception as exc:
                print(f"[swing_trajectory] ROS2 unavailable: {exc}", flush=True)
                self._node = None

    def get_tool(self):
        return {"name": "swing_trajectory", "type": "sensor", "multiInstance": False,
                "description": "Go1 four-foot swing trajectories: body frame and estimated odometry-world frame",
                "inputSchema": {"type": "object", "properties": {}},
                "topic_out": ([{"topic": self._topic, "format": _SWING_FORMAT}] + [
                    {"topic": topic, "format": "sensor/pointcloud"}
                    for topic in self._cloud_topics.values()] if self._node else [])}

    def start(self):
        with self._lifecycle_lock:
            if self._thread is None or not self._thread.is_alive():
                self._stop.clear()
                self._thread = threading.Thread(target=self._loop, name="go1_swing_trajectory", daemon=True)
                self._thread.start()

    def stop(self):
        with self._lifecycle_lock:
            self._stop.set()
            if self._thread is not None:
                self._thread.join(timeout=2)
                if not self._thread.is_alive():
                    self._thread = None

    def _loop(self):
        last_error = None
        repeated_errors = 0
        while not self._stop.is_set():
            try:
                self.process_sample(self._client.snapshot())
                if self._node is not None:
                    data = self._build()
                    msg = String()
                    msg.data = json.dumps(data)
                    self._pub.publish(msg)
                    for frame, publisher in self._cloud_pubs.items():
                        cloud = UInt8MultiArray()
                        cloud.data = _swing_pointcloud(data, frame)
                        publisher.publish(cloud)
                last_error = None
                repeated_errors = 0
            except Exception as exc:
                error = (type(exc).__name__, str(exc))
                repeated_errors = repeated_errors + 1 if error == last_error else 1
                last_error = error
                if repeated_errors == 1 or repeated_errors % 100 == 0:
                    print(f"[swing_trajectory] sample error ({repeated_errors}): {exc}", flush=True)
            self._stop.wait(1.0 / self._hz)

    def process_sample(self, snap, now=None):
        """Record changed snapshots without requiring changes to the shared SDK client."""
        now = time.monotonic() if now is None else now
        with self._lock:
            if not isinstance(snap, dict) or not snap.get("fresh"):
                self._last_status = "source_unavailable"
                return
            if snap.get("control_level") not in (None, "HIGHLEVEL") or snap.get("mode") != 2:
                for state in self._feet.values():
                    state.update(phase="unknown", candidate=None, candidate_count=0,
                                 active=[], swing_start_s=None)
                self._last_signature = None
                self._last_sample = None
                self._last_source_stamp = None
                self._last_status = "not_walking" if snap.get("mode") != 2 else "wrong_control_level"
                return
            positions = snap.get("foot_pos")
            forces = snap.get("foot_force")
            if not isinstance(positions, (list, tuple)) or len(positions) != 4 or not isinstance(forces, (list, tuple)) or len(forces) != 4:
                self._last_status = "foot_data_unavailable"
                return
            body = [_swing_vector(value) for value in positions]
            try:
                force = [float(v) for v in forces]
            except (TypeError, ValueError):
                self._last_status = "foot_data_invalid"
                return
            if any(v is None for v in body) or not all(math.isfinite(v) and v >= 0 for v in force):
                self._last_status = "foot_data_invalid"
                return
            imu = snap.get("imu") or {}
            quat = imu.get("quaternion_wxyz")
            world = ([_swing_world_point(v, snap.get("position"), quat) for v in body]
                     if imu.get("quaternion_wxyz_valid", True) else [None] * 4)
            source_stamp = snap.get("received_monotonic_s")
            if source_stamp is not None:
                try:
                    source_stamp = float(source_stamp)
                except (TypeError, ValueError):
                    source_stamp = float("nan")
                if not math.isfinite(source_stamp) or source_stamp > now + 0.1:
                    self._last_status = "source_timestamp_invalid"
                    return
                if now - source_stamp > self._max_age:
                    self._last_status = "source_stale"
                    return
            signature = json.dumps([force, body, snap.get("position"), quat],
                                   sort_keys=True, default=str)
            same_snapshot = (source_stamp == self._last_source_stamp if source_stamp is not None
                             else self._last_source_stamp is None and signature == self._last_signature)
            if same_snapshot and self._last_sample is not None and now - self._last_sample > self._max_age:
                return
            if not same_snapshot and self._last_sample is not None and now - self._last_sample > self._max_age:
                for state in self._feet.values():
                    state.update(phase="unknown", candidate=None, candidate_count=0,
                                 active=[], swing_start_s=None)
            if not same_snapshot:
                self._last_signature = signature
                self._last_source_stamp = source_stamp
                self._last_sample = source_stamp if source_stamp is not None else now
            sample_time = source_stamp if source_stamp is not None else now
            self._last_status = "ok" if all(v is not None for v in world) else "world_pose_unavailable"
            if same_snapshot:
                return
            for i, name in enumerate(_SWING_FEET):
                state = self._feet[name]
                f = force[i]
                started_swing = False
                desired = ("swing" if f <= self._force else
                           "stance" if f >= self._force + self._hysteresis else None)
                if desired is None:
                    state["candidate"] = None
                    state["candidate_count"] = 0
                elif desired != state["phase"]:
                    state["candidate_count"] = state["candidate_count"] + 1 if state["candidate"] == desired else 1
                    state["candidate"] = desired
                    if state["candidate_count"] >= self._confirm:
                        if state["phase"] == "swing" and desired == "stance":
                            state["last_completed"] = state["active"]
                            state["active"] = []
                        elif desired == "swing":
                            state["swing_id"] += 1
                            state["active"] = []
                            state["swing_start_s"] = sample_time
                            started_swing = True
                        state["phase"] = desired
                        state["candidate"] = None
                        state["candidate_count"] = 0
                else:
                    state["candidate"] = None
                    state["candidate_count"] = 0
                if state["phase"] == "swing" and (not same_snapshot or started_swing):
                    state["active"].append({"t_monotonic_s": round(sample_time, 4),
                                            "t_from_liftoff_s": round(sample_time - state["swing_start_s"], 4),
                                            "body_xyz_m": body[i], "world_xyz_m": world[i]})
                    if len(state["active"]) > self._max_points:
                        state["active"].pop(0)

    def _build(self):
        with self._lock:
            change_age = (None if self._last_sample is None else
                          max(0.0, time.monotonic() - self._last_sample))
            status = ("source_stale" if self._last_source_stamp is not None and
                      change_age is not None and change_age > self._max_age and
                      self._last_status in ("ok", "world_pose_unavailable") else
                      "source_unchanged" if self._last_source_stamp is None and change_age is not None and
                      change_age > self._max_age and self._last_status in ("ok", "world_pose_unavailable")
                      else self._last_status)
            feet = {name: {"phase": value["phase"], "swing_id": value["swing_id"],
                           "active": list(value["active"]),
                           "last_completed": list(value["last_completed"])}
                    for name, value in self._feet.items()}
        return {"timestamp_ms": int(time.time() * 1000), "control_level": "HIGHLEVEL",
                "status": status,
                "fresh": status in ("ok", "world_pose_unavailable"),
                "source_age_s": round(change_age, 3) if self._last_source_stamp is not None and change_age is not None else None,
                "source_freshness_confirmed": self._last_source_stamp is not None and status in ("ok", "world_pose_unavailable"),
                "observed_change_age_s": None if change_age is None else round(change_age, 3),
                "sample_time_basis": ("sdk_receive_monotonic" if self._last_source_stamp is not None
                                      else "card_observation_monotonic"),
                "body_frame": "body_instantaneous", "world_frame": "HighState_odometry_inertial_estimate",
                "world_assumption": "HighState.position and IMU quaternion axes/origin aligned; unverified on robot",
                "contact_force_threshold_raw": self._force, "feet": feet}

    def dispatch(self, action, args):
        if action == "start":
            self.start()
            return {"state": "running"}
        if action == "stop":
            self.stop()
            return {"state": "idle"}
        if action in ("info", "read", "get", "swing_trajectory"):
            return {"state": "running" if self._thread and self._thread.is_alive() else "idle",
                    "data": self._build(), "topic_out": self.get_tool()["topic_out"]}
        return None


def make_swing_trajectory(plugin_config, namespace, executor, client):
    return SwingTrajectoryPlugin(plugin_config, namespace, executor, client)
