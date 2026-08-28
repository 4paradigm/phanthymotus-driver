"""Compact Q5 battery status card backed by ``/battery_state``."""

from __future__ import annotations

import json
import math
import time

from sensor_contract import topic_out

try:
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import String

    _HAS_ROS2 = True
    _QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                      history=HistoryPolicy.KEEP_LAST, depth=1,
                      durability=DurabilityPolicy.VOLATILE)
except Exception:
    _HAS_ROS2 = False

CARD = "battery"
TYPE = "sensor"
TOPIC = "/{ns}/q5/battery"
FMT = "data/json"
HZ = 2.0
NODE = "q5_battery"
DESC = "Q5 电池：电量、状态、电压和温度"


def _percentage(raw):
    if isinstance(raw, bool) or not isinstance(raw, (int, float)) or not math.isfinite(raw):
        return None
    raw = float(raw)
    # ROS BatteryState uses 0..1; the observed Q5 message uses 0..100.
    return raw * 100.0 if 0.0 <= raw <= 1.0 else raw


def _level(percentage):
    if percentage is None:
        return "unknown"
    if percentage >= 99.5:
        return "full"
    if percentage <= 15.0:
        return "critical"
    if percentage <= 30.0:
        return "low"
    return "normal"


def build(snap: dict) -> dict:
    data = {
        "timestamp_ms": int(time.time() * 1000),
        "received_at_ms": snap.get("received_at_ms"),
        "fresh": bool(snap.get("fresh", False)),
        "available": bool(snap.get("available", False)),
        "age_ms": snap.get("age_ms"),
        "stale": bool(snap.get("stale", False)),
        "source_topic": "/battery_state",
    }
    if not data["available"]:
        data["message"] = "未收到 /battery_state 消息"
        return data

    percentage = _percentage(snap.get("percentage"))
    level = _level(percentage)
    data.update({
        "percentage": percentage,
        "level": level,
        "voltage_v": snap.get("voltage"),
        "temperature_c": snap.get("temperature"),
    })
    if not data["fresh"]:
        data["message"] = "电池消息已过期"
    elif level == "full":
        # The observed vendor status is 0/UNKNOWN with zero current even while
        # plugged in.  Report the actionable fact, not an invented charger state.
        data["message"] = "电量已满"
    elif level == "unknown":
        data["message"] = "电量百分比未上报"
    return data


class Plugin:
    def __init__(self, plugin_config, namespace, executor, client):
        self._client = client
        self._topic = TOPIC.format(ns=namespace)
        self._node = None
        self._pub = None
        if _HAS_ROS2 and executor is not None:
            try:
                self._node = Node(NODE)
                self._pub = self._node.create_publisher(String, self._topic, _QOS)
                self._node.create_timer(1.0 / HZ, self._tick)
                executor.add_node(self._node)
            except Exception as e:
                print(f"[{CARD}] ROS2 publisher unavailable: {e}", flush=True)
                self._node = None
                self._pub = None

    def _data(self):
        return build(self._client.sensor_snapshot("battery"))

    def _tick(self):
        msg = String()
        msg.data = json.dumps(self._data(), ensure_ascii=False)
        self._pub.publish(msg)

    def get_tool(self):
        return {
            "name": CARD, "type": TYPE, "multiInstance": False, "description": DESC,
            "inputSchema": {"type": "object", "properties": {
                "action": {"type": "string", "enum": ["info", "start", "stop"]},
            }, "required": ["action"], "additionalProperties": False},
            "topic_out": topic_out(self._topic, FMT),
        }

    def dispatch(self, action, args):
        if action == "start":
            return {"state": "running" if self._pub else "unavailable"}
        if action == "stop":
            return {"state": "idle"}
        if action in ("info", "read", "get", CARD):
            return {"state": "running" if self._pub else "unavailable", "data": self._data(),
                    "topic_out": topic_out(self._topic, FMT)}
        return None


def make_plugin(plugin_config, namespace, executor, client):
    return Plugin(plugin_config, namespace, executor, client)
