"""
sensors.py — Go1 状态/资源卡聚合（battery, imu, feet, fall_alarm, obstacle_range,
             remote_controller, udp_diagnostics, loco_state, odometry, joints, model）。

自包含：一张合并文件 = 多张卡片。main.py 按 config.yaml 里的卡名手动 import 并 make_plugin()。
每张卡保持独立的 CARD / Plugin / make_plugin，只是合并在同一文件。
"""

from __future__ import annotations

import json
import math
import threading
import time
from collections import deque
from pathlib import Path

from go1_sdk_client import JOINT_NAMES, parse_wireless_remote

try:
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
    from std_msgs.msg import String
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
# battery_power / joint_power — read-only power, energy and runtime monitoring
# ============================================================================

_STALE_S = 2.5
_MAX_GAP_S = 2.5
_BMS_SLOT_COUNT = 10  # SDK array capacity, not the number of physical cells.
# Go1 six-cell layout observed in live HighState. The active channels match
# Table 9-3 (6 Cells) at https://www.ti.com/lit/ds/symlink/bq76930.pdf .
# This correspondence does not identify the installed BMS chip/firmware.
_GO1_CELL_INDICES = (0, 1, 4, 5, 6, 9)
_GO1_UNUSED_INDICES = (2, 3, 7, 8)
_MIN_RUNTIME_OBSERVATION_S = 60.0
_MIN_RUNTIME_SOC_DROP = 3.0
_RUNTIME_UPDATE_INTERVAL_S = 10.0


def _power_monitor_headers(data: dict) -> dict:
    """Compact ROS display; MCP info retains the complete diagnostic data."""
    status = "正常" if data.get("available") else f"数据不可用：{data.get('reason', 'unknown')}"
    return {"timestamp_ms": data["timestamp_ms"],
            "control_level": data["control_level"],
            "fresh": data["fresh"], "status": status}


def _battery_power_monitor(data: dict) -> dict:
    out = _power_monitor_headers(data)
    for field in ("power_w", "discharged_since_start_wh", "charged_since_start_wh",
                  "remaining_runtime_minutes"):
        # Keep stable keys: the latest-value renderer must clear old readings
        # when telemetry becomes unavailable rather than retaining stale values.
        out[field] = data.get(field)
    if data.get("available") and data.get("remaining_runtime_minutes") is None:
        reason = data.get("runtime_estimate_reason")
        note = {"insufficient_soc_history": "续航观察中",
                "soc_unavailable": "电量百分比不可用",
                "not_discharging": "非放电状态，暂无续航预测"}.get(reason, "续航暂不可用")
        out["status"] = f"数据正常；{note}"
    return out


def _joint_power_monitor(data: dict) -> dict:
    out = _power_monitor_headers(data)
    powers = {joint["name"]: joint["mechanical_power_w"]
              for joint in data.get("joints", [])} if data.get("available") else {}
    # A map renders as one row per joint, without torque/speed/index clutter.
    out["estimated_mechanical_power_w"] = {name: powers.get(name) for name in JOINT_NAMES}
    return out


def _sample_time(snap: dict, now: float) -> float | None:
    """Return a recent source timestamp; a read-time timestamp is not enough."""
    if not snap.get("fresh"):
        return None
    try:
        stamp = float(snap["sample_monotonic_s"])
    except (KeyError, TypeError, ValueError):
        return None
    if not math.isfinite(stamp) or stamp <= 0 or not -0.1 <= now - stamp <= _STALE_S:
        return None
    return stamp


def _battery_measurement(snap: dict) -> tuple[dict | None, str | None]:
    bat = snap.get("battery")
    if not isinstance(bat, dict):
        return None, "bms_unavailable"
    try:
        slots = [float(v) for v in bat["cell_voltage_mv"]]
        current_ma = float(bat["current_ma"])
        status = int(bat["status_code"])
    except (KeyError, TypeError, ValueError, OverflowError):
        return None, "invalid_bms_fields"
    if (len(slots) != _BMS_SLOT_COUNT or
            any(not math.isfinite(v) for v in slots) or
            not math.isfinite(current_ma) or abs(current_ma) > 200000):
        return None, "invalid_bms_values"
    cells = [slots[i] for i in _GO1_CELL_INDICES]
    unused = [slots[i] for i in _GO1_UNUSED_INDICES]
    if any(not 2000 <= v <= 5000 for v in cells):
        return None, "invalid_bms_values"
    # Shorted/unused channels had a 32 mV offset in the observed frames.
    # Require near-zero values in these fixed slots; never filter arbitrary
    # low readings out of the active cells or infer a new layout per sample.
    if any(not 0 <= v <= 100 for v in unused):
        return None, "unsupported_bms_cell_layout"
    direction = "discharge" if status == 1 else "charge" if status in (2, 3, 4) else "unknown"
    try:
        soc = float(bat["soc_percent"])
        soc = soc if math.isfinite(soc) and 0 <= soc <= 100 else None
    except (KeyError, TypeError, ValueError, OverflowError):
        soc = None
    voltage_v = sum(cells) / 1000.0
    power_w = voltage_v * abs(current_ma) / 1000.0
    return {"voltage_v": round(voltage_v, 3),
            "cell_count": len(cells),
            "cell_voltage_indices": list(_GO1_CELL_INDICES),
            "cell_voltage_mv": cells,
            "unused_cell_voltage_mv": unused,
            "voltage_source": "go1_six_cell_bms_slots",
            "current_a": round(current_ma / 1000.0, 3),
            "power_w": round(power_w, 3),
            "soc_percent": soc,
            "direction": direction,
            "signed_power_w": round(power_w if direction == "discharge" else -power_w, 3)
            if direction != "unknown" else None}, None


class BatteryPowerPlugin:
    """Integrate only consecutive, fresh BMS samples while the bundle runs."""

    def __init__(self, plugin_config, namespace, executor, client):
        self._client = client
        self._topic = f"/{namespace}/state/battery_power"
        self._node = None
        self._lock = threading.Lock()
        self._lifecycle_lock = threading.Lock()
        self._shutdown = threading.Event()
        self._thread = None
        self._last = None  # (source monotonic timestamp, unrounded W, direction)
        self._discharged_wh = 0.0
        self._charged_wh = 0.0
        self._covered_s = 0.0
        self._runtime_start = None  # (first continuous discharge timestamp, SOC%)
        self._runtime_last_soc = None
        self._runtime_updated_s = None
        self._latest = self._unavailable("no_sample_yet")
        if _HAS_ROS2 and executor is not None:
            try:
                self._node = Node("go1_battery_power")
                self._pub = self._node.create_publisher(String, self._topic, _QOS)
                self._node.create_timer(1.0, self._tick)
                executor.add_node(self._node)
            except Exception as e:
                print(f"[battery_power] ROS2 unavailable, MCP polling only: {e}", flush=True)
                self._node = None

    def _unavailable(self, reason: str) -> dict:
        return {"timestamp_ms": int(time.time() * 1000), "fresh": False,
                "control_level": "HIGHLEVEL",
                "available": False, "reason": reason,
                "discharged_since_start_wh": round(self._discharged_wh, 4),
                "charged_since_start_wh": round(self._charged_wh, 4),
                "covered_duration_s": round(self._covered_s, 2),
                "energy_scope": "driver_process_since_start",
                "remaining_runtime_minutes": None,
                "runtime_estimate_reason": reason}

    def _sample(self, snap: dict, now: float | None = None) -> dict:
        """One deterministic step, also used by the background sampler."""
        now = time.monotonic() if now is None else now
        with self._lock:
            stamp = _sample_time(snap, now)
            if stamp is None:
                self._last = None
                self._runtime_start = None
                self._runtime_last_soc = None
                self._latest = self._unavailable("stale_or_missing_sample")
                return dict(self._latest)
            measurement, reason = _battery_measurement(snap)
            if measurement is None:
                self._last = None
                self._runtime_start = None
                self._runtime_last_soc = None
                self._latest = self._unavailable(reason)
                return dict(self._latest)
            if self._last is not None and stamp == self._last[0]:
                return dict(self._latest)
            power = measurement["power_w"]
            direction = measurement["direction"]
            if self._last is not None and not 0 < stamp - self._last[0] <= _MAX_GAP_S:
                self._runtime_start = None
                self._runtime_last_soc = None
                if stamp < self._last[0]:
                    self._last = None
            if self._last is not None:
                previous_stamp, previous_power, previous_direction = self._last
                dt = stamp - previous_stamp
                if 0 < dt <= _MAX_GAP_S and direction == previous_direction:
                    energy_wh = (previous_power + power) * 0.5 * dt / 3600.0
                    if direction == "discharge":
                        self._discharged_wh += energy_wh
                        self._covered_s += dt
                    elif direction == "charge":
                        self._charged_wh += energy_wh
                        self._covered_s += dt
            if self._last is None or stamp > self._last[0]:
                self._last = (stamp, power, direction)
            soc = measurement["soc_percent"]
            runtime_minutes = None
            if direction != "discharge":
                self._runtime_start = None
                self._runtime_last_soc = None
                runtime_reason = "not_discharging"
            elif soc is None:
                self._runtime_start = None
                self._runtime_last_soc = None
                runtime_reason = "soc_unavailable"
            elif soc == 0:
                self._runtime_start = None
                self._runtime_last_soc = None
                runtime_minutes = 0.0
                runtime_reason = None
            else:
                if (self._runtime_start is None or
                        (self._runtime_last_soc is not None and
                         soc > self._runtime_last_soc)):
                    self._runtime_start = (stamp, soc)
                self._runtime_last_soc = soc
                start_stamp, start_soc = self._runtime_start
                observed_s = stamp - start_stamp
                drop = start_soc - soc
                if observed_s >= _MIN_RUNTIME_OBSERVATION_S and drop >= _MIN_RUNTIME_SOC_DROP:
                    runtime_minutes = round(soc * observed_s / drop / 60.0, 1)
                    runtime_reason = None
                else:
                    runtime_reason = "insufficient_soc_history"
            if runtime_minutes is not None:
                # Hold valid estimates between updates; unavailable states and
                # zero SOC still take effect immediately, without delaying Wh.
                previous_runtime = self._latest.get("remaining_runtime_minutes")
                if (soc != 0 and previous_runtime is not None and
                        self._runtime_updated_s is not None and
                        0 <= stamp - self._runtime_updated_s < _RUNTIME_UPDATE_INTERVAL_S):
                    runtime_minutes = previous_runtime
                else:
                    self._runtime_updated_s = stamp
            self._latest = {"timestamp_ms": int(time.time() * 1000),
                            "sample_monotonic_s": stamp,
                            "control_level": snap.get("control_level", "HIGHLEVEL"),
                            "fresh": True, "available": True,
                            **measurement,
                            "discharged_since_start_wh": round(self._discharged_wh, 4),
                            "charged_since_start_wh": round(self._charged_wh, 4),
                            "covered_duration_s": round(self._covered_s, 2),
                            "energy_scope": "driver_process_since_start",
                            "remaining_runtime_minutes": runtime_minutes,
                            "runtime_estimate_reason": runtime_reason,
                            "runtime_estimate_method": "observed_soc_decline",
                            "measurement_kind": "battery_electrical_estimate"}
            return dict(self._latest)

    def _run(self):
        while not self._shutdown.is_set():
            try:
                self._sample(self._client.snapshot())
            except Exception as e:
                with self._lock:
                    self._last = None
                    self._runtime_start = None
                    self._runtime_last_soc = None
                    self._latest = self._unavailable(f"snapshot_error: {e}")
            self._shutdown.wait(1.0)

    def _read_latest(self) -> dict:
        with self._lock:
            if (self._latest.get("available") and
                    _sample_time(self._latest, time.monotonic()) is None):
                return self._unavailable("stale_or_missing_sample")
            return dict(self._latest)

    def _tick(self):
        with self._lifecycle_lock:
            if self._shutdown.is_set():
                return
            try:
                msg = String()
                msg.data = json.dumps(_battery_power_monitor(self._read_latest()), ensure_ascii=False)
                self._pub.publish(msg)
            except Exception as e:
                self._node.get_logger().error(f"publish {self._topic} error: {e}")

    def get_tool(self):
        return {"name": "battery_power", "type": "sensor", "multiInstance": False,
                "description": ("Go1 BMS power, discharge/charge Wh and rough "
                                "remaining runtime from observed SOC decline; read-only"),
                "inputSchema": {"type": "object", "properties": {}},
                "topic_out": ([{"topic": self._topic, "format": "data/json"}] if self._node else [])}

    def start(self):
        with self._lifecycle_lock:
            if self._thread is None or not self._thread.is_alive():
                self._shutdown.clear()
                self._thread = threading.Thread(
                    target=self._run, daemon=True, name="go1_battery_power")
                self._thread.start()

    def stop(self):
        with self._lifecycle_lock:
            self._shutdown.set()
            if self._thread is not None:
                self._thread.join(timeout=2.5)
                if self._thread.is_alive():
                    raise RuntimeError("battery sampler did not stop")
            with self._lock:
                # Keep process totals, but never integrate across a pause or
                # reuse SOC history from before sampling was stopped.
                self._last = None
                self._runtime_start = None
                self._runtime_last_soc = None
                self._latest = self._unavailable("sampling_stopped")

    def dispatch(self, action, args):
        if action == "start":
            self.start()
            return {"state": "running"}
        if action == "stop":
            self.stop()
            return {"state": "idle"}
        if action in ("info", "read", "get", "battery_power"):
            with self._lifecycle_lock:
                running = (self._thread is not None and self._thread.is_alive()
                           and not self._shutdown.is_set())
                return {"state": "running" if running else "idle", "data": self._read_latest(),
                        "topic_out": ([{"topic": self._topic, "format": "data/json"}] if self._node else [])}
        return None


def make_battery_power(plugin_config, namespace, executor, client):
    return BatteryPowerPlugin(plugin_config, namespace, executor, client)


def _build_joint_power(snap: dict, now: float | None = None) -> dict:
    now = time.monotonic() if now is None else now
    out = {"timestamp_ms": int(time.time() * 1000),
           "control_level": snap.get("control_level", "HIGHLEVEL"),
           "fresh": False, "available": False,
           "measurement_kind": "estimated_joint_mechanical_power",
           "electrical_power_available": False}
    if _sample_time(snap, now) is None:
        out["reason"] = "stale_or_missing_sample"
        return out
    joints = snap.get("joints")
    if not isinstance(joints, list) or len(joints) < 12:
        out["reason"] = "incomplete_joint_state"
        return out
    result = []
    try:
        for i, joint in enumerate(joints[:12]):
            torque = float(joint["tau"])
            speed = float(joint["dq"])
            if not math.isfinite(torque) or not math.isfinite(speed):
                raise ValueError("non-finite joint data")
            result.append({"idx": i, "name": JOINT_NAMES[i],
                           "estimated_torque_nm": torque, "angular_speed_rad_s": speed,
                           "mechanical_power_w": round(torque * speed, 3)})
    except (KeyError, TypeError, ValueError):
        out["reason"] = "invalid_joint_state"
        return out
    powers = [joint["mechanical_power_w"] for joint in result]
    out.update({"fresh": True, "available": True, "joints": result,
                "positive_mechanical_power_w": round(sum(max(p, 0.0) for p in powers), 3),
                "negative_mechanical_power_w": round(sum(min(p, 0.0) for p in powers), 3)})
    return out


class JointPowerPlugin:
    def __init__(self, plugin_config, namespace, executor, client):
        self._client = client
        self._topic = f"/{namespace}/state/joint_power"
        self._node = None
        if _HAS_ROS2 and executor is not None:
            try:
                self._node = Node("go1_joint_power")
                self._pub = self._node.create_publisher(String, self._topic, _QOS)
                self._node.create_timer(0.1, self._tick)
                executor.add_node(self._node)
            except Exception as e:
                print(f"[joint_power] ROS2 unavailable, MCP polling only: {e}", flush=True)
                self._node = None

    def _tick(self):
        try:
            msg = String()
            msg.data = json.dumps(_joint_power_monitor(
                _build_joint_power(self._client.snapshot())), ensure_ascii=False)
            self._pub.publish(msg)
        except Exception as e:
            self._node.get_logger().error(f"publish {self._topic} error: {e}")

    def get_tool(self):
        return {"name": "joint_power", "type": "sensor", "multiInstance": False,
                "description": ("Go1 12 joints' estimated mechanical power "
                                "(tauEst*dq), not electrical power; read-only"),
                "inputSchema": {"type": "object", "properties": {}},
                "topic_out": ([{"topic": self._topic, "format": "data/json"}] if self._node else [])}

    def start(self): pass
    def stop(self): pass

    def dispatch(self, action, args):
        if action == "start": return {"state": "running"}
        if action == "stop": return {"state": "idle"}
        if action in ("info", "read", "get", "joint_power"):
            return {"state": "running", "data": _build_joint_power(self._client.snapshot()),
                    "topic_out": ([{"topic": self._topic, "format": "data/json"}] if self._node else [])}
        return None


def make_joint_power(plugin_config, namespace, executor, client):
    return JointPowerPlugin(plugin_config, namespace, executor, client)
