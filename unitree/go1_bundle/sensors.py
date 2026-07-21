"""
sensors.py — Go1 状态/资源卡聚合（battery, imu, feet, fall_alarm, obstacle_range,
             remote_controller, udp_diagnostics, loco_state, odometry, joints, model）。

自包含：一张合并文件 = 多张卡片。main.py 按 config.yaml 里的卡名手动 import 并 make_plugin()。
每张卡保持独立的 CARD / Plugin / make_plugin，只是合并在同一文件。
"""

from __future__ import annotations

import json
import math
import time
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
