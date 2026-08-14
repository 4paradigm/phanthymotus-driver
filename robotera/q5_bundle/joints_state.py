"""Q5 complete joint-state card (read-only)."""

from __future__ import annotations

import json
import time

from sensor_contract import topic_out
from typing import Optional

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

CARD = "joints_state"
TYPE = "sensor"
TOPIC = "/{ns}/q5/joints_state"
FMT = "data/json"
HZ = 10.0
NODE = "q5_joints_state"
DESC = "Q5 完整关节状态：身体、左手、右手的实际 position、velocity、effort"


def _hand_side(name: str) -> Optional[str]:
    normalized = name.lower().replace("-", "_").replace(" ", "_")
    if "hand" not in normalized:
        return None
    if normalized.startswith(("left_", "l_")) or "_left_" in normalized:
        return "left"
    if normalized.startswith(("right_", "r_")) or "_right_" in normalized:
        return "right"
    return "unknown"


def _group(names, positions, velocities, efforts):
    return {
        "joint_count": len(names),
        "joint_names": names,
        "positions": {name: positions[name] for name in names if name in positions},
        "velocities": {name: velocities[name] for name in names if name in velocities},
        "efforts": {name: efforts[name] for name in names if name in efforts},
    }


def build(snap: dict) -> dict:
    data = {
        "timestamp_ms": int(time.time() * 1000),
        "received_at_ms": snap.get("received_at_ms"),
        "message_timestamp_ms": snap.get("message_timestamp_ms"),
        "fresh": bool(snap.get("fresh", False)),
        "available": bool(snap.get("available", False)),
        "age_ms": snap.get("age_ms"),
        "stale": bool(snap.get("stale", False)),
    }
    data["source_topic"] = "/joint_states"
    if not snap.get("fresh"):
        data["message"] = (
            "关节状态消息已过期" if snap.get("available", False)
            else "未收到 /joint_states 消息"
        )
        return data

    groups = {"body": [], "left_hand": [], "right_hand": [], "unknown_hand": []}
    for name in snap.get("joint_names", []):
        side = _hand_side(name)
        if side is None:
            groups["body"].append(name)
        elif side == "left":
            groups["left_hand"].append(name)
        elif side == "right":
            groups["right_hand"].append(name)
        else:
            groups["unknown_hand"].append(name)

    positions = snap.get("joints", {})
    velocities = snap.get("velocities", {})
    efforts = snap.get("efforts", {})
    data["joint_count"] = len(snap.get("joint_names", []))
    data["position_unit"] = snap.get("position_unit", "rad")
    data["groups"] = {
        group: _group(names, positions, velocities, efforts)
        for group, names in groups.items()
    }
    data["positions"] = dict(positions)
    data["velocities"] = dict(velocities)
    data["efforts"] = dict(efforts)
    return data


class Plugin:
    def __init__(self, plugin_config, namespace, executor, client):
        self._client = client
        self._topic = TOPIC.format(ns=namespace)
        self._node = None
        if _HAS_ROS2 and executor is not None:
            try:
                self._node = Node(NODE)
                self._pub = self._node.create_publisher(String, self._topic, _QOS)
                self._node.create_timer(1.0 / HZ, self._tick)
                executor.add_node(self._node)
            except Exception as e:
                print(f"[{CARD}] ROS2 发布不可用，退回 MCP 轮询: {e}", flush=True)
                self._node = None

    def _tick(self):
        msg = String()
        msg.data = json.dumps(build(self._client.snapshot()), ensure_ascii=False)
        self._pub.publish(msg)

    def get_tool(self):
        return {
            "name": CARD,
            "type": TYPE,
            "multiInstance": False,
            "description": DESC + (f" -> {self._topic}" if self._node else " — poll via MCP action=info"),
            "inputSchema": {
                "type": "object",
                "properties": {"action": {"type": "string", "enum": ["info", "start", "stop"]}},
                "required": ["action"],
                "additionalProperties": False,
            },
            "topic_out": topic_out(self._topic, FMT),
        }

    def start(self):
        pass

    def stop(self):
        pass

    def dispatch(self, action, args):
        if action == "start":
            return {"state": "running"}
        if action == "stop":
            return {"state": "idle"}
        if action in ("info", "read", "get", CARD):
            return {"state": "running", "data": build(self._client.snapshot()),
                    "topic_out": topic_out(self._topic, FMT)}
        return None


def make_plugin(plugin_config, namespace, executor, client):
    return Plugin(plugin_config, namespace, executor, client)
