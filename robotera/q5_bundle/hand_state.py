"""Q5 XHand Lite state card backed by the live joint snapshot."""

from __future__ import annotations

import json
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

CARD = "hand_state"
TYPE = "sensor"
TOPIC = "/{ns}/q5/hand_state"
FMT = "data/json"
HZ = 10.0
NODE = "q5_hand_state"
DESC = "Q5 XHand Lite 左右手关节状态：位置、速度、力矩"
LEFT_JOINTS = (
    "left_hand_thumb_bend_joint", "left_hand_thumb_rota_joint1",
    "left_hand_index_joint1", "left_hand_mid_joint1",
    "left_hand_ring_joint1", "left_hand_pinky_joint1",
)
RIGHT_JOINTS = tuple(name.replace("left_", "right_", 1) for name in LEFT_JOINTS)


def _side(names, positions, velocities, efforts):
    return {
        "joint_names": list(names),
        "joint_count": len(names),
        "positions": {name: positions[name] for name in names if name in positions},
        "velocities": {name: velocities[name] for name in names if name in velocities},
        "efforts": {name: efforts[name] for name in names if name in efforts},
        "complete": all(name in positions for name in names),
    }


def build(snap: dict) -> dict:
    data = {"timestamp_ms": int(time.time() * 1000),
            "received_at_ms": snap.get("received_at_ms"),
            "message_timestamp_ms": snap.get("message_timestamp_ms"),
            "fresh": bool(snap.get("fresh", False)),
            "available": bool(snap.get("available", False)),
            "age_ms": snap.get("age_ms"), "stale": bool(snap.get("stale", False)),
            "hand_model": "XHand Lite", "source_topic": "/joint_states"}
    if not snap.get("fresh"):
        data["message"] = (
            "手部关节状态消息已过期" if snap.get("available", False)
            else "未收到 /joint_states 消息"
        )
        return data
    positions, velocities, efforts = (snap.get("joints", {}), snap.get("velocities", {}), snap.get("efforts", {}))
    data["left"] = _side(LEFT_JOINTS, positions, velocities, efforts)
    data["right"] = _side(RIGHT_JOINTS, positions, velocities, efforts)
    data["hands_complete"] = data["left"]["complete"] and data["right"]["complete"]
    if not data["left"]["complete"] or not data["right"]["complete"]:
        data["message"] = "检测到的手部关节与 XHand Lite 布局不完整"
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

    def _tick(self):
        msg = String()
        msg.data = json.dumps(build(self._client.snapshot()), ensure_ascii=False)
        self._pub.publish(msg)

    def get_tool(self):
        return {"name": CARD, "type": TYPE, "multiInstance": False,
                "description": DESC,
                "inputSchema": {"type": "object", "properties": {"action": {"type": "string", "enum": ["info", "start", "stop"]}}, "required": ["action"], "additionalProperties": False},
                "topic_out": topic_out(self._topic, FMT)}

    def start(self):
        return {"state": "running" if self._pub else "unavailable"}

    def stop(self):
        return {"state": "idle"}

    def dispatch(self, action, args):
        if action == "start":
            return self.start()
        if action == "stop":
            return self.stop()
        if action in ("info", "read", "get", CARD):
            return {"state": "running" if self._pub else "unavailable", "data": build(self._client.snapshot()),
                    "topic_out": topic_out(self._topic, FMT)}
        return None


def make_plugin(plugin_config, namespace, executor, client):
    return Plugin(plugin_config, namespace, executor, client)
