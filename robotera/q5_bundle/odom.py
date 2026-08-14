# -*- coding: utf-8 -*-
# odom —— Q5 里程计 (nav_msgs/msg/Odometry)
# 自建 ROS2 Node 订阅 /wr1_base_drive_controller/odom，解析位置/姿态/速度。

from __future__ import annotations

import json
import time

from sensor_contract import topic_out

try:
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    from nav_msgs.msg import Odometry
    from std_msgs.msg import String

    _HAS_ROS2 = True
    _QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                      history=HistoryPolicy.KEEP_LAST, depth=1,
                      durability=DurabilityPolicy.VOLATILE)
except Exception:
    _HAS_ROS2 = False

CARD = "odom"
TYPE = "sensor"
SOURCE_TOPIC = "/wr1_base_drive_controller/odom"
TOPIC = "/{ns}/q5/odom"
FMT = "data/json"
HZ = 2.0
NODE = "q5_odom"
DESC = "Q5 里程计：底盘位置、姿态和速度"


def build(msg, received_at_ms) -> dict:
    now_ms = int(time.time() * 1000)
    if msg is None:
        return {"timestamp_ms": now_ms, "received_at_ms": received_at_ms,
                "fresh": False, "available": False, "source_topic": SOURCE_TOPIC,
                "message": "未收到里程计消息"}
    age_ms = None if received_at_ms is None else now_ms - received_at_ms
    p = msg.pose.pose
    t = msg.twist.twist
    return {"timestamp_ms": now_ms, "received_at_ms": received_at_ms,
            "age_ms": age_ms, "fresh": age_ms is not None and age_ms <= 5000,
            "available": True,
            "frame_id": msg.header.frame_id,
            "child_frame_id": msg.child_frame_id,
            "position": {"x": float(p.position.x), "y": float(p.position.y), "z": float(p.position.z)},
            "orientation": {"x": float(p.orientation.x), "y": float(p.orientation.y),
                            "z": float(p.orientation.z), "w": float(p.orientation.w)},
            "linear": {"x": float(t.linear.x), "y": float(t.linear.y), "z": float(t.linear.z)},
            "angular": {"x": float(t.angular.x), "y": float(t.angular.y), "z": float(t.angular.z)},
            "source_topic": SOURCE_TOPIC}


class Plugin:
    def __init__(self, plugin_config, namespace, executor, client):
        self._topic = TOPIC.format(ns=namespace)
        self._node = None
        self._pub = None
        self._last_msg = None
        self._received_at_ms = None
        if _HAS_ROS2 and executor is not None:
            try:
                self._node = Node(NODE)
                self._pub = self._node.create_publisher(String, self._topic, _QOS)
                self._node.create_subscription(Odometry, SOURCE_TOPIC, self._on_msg, _QOS)
                self._node.create_timer(1.0 / HZ, self._tick)
                executor.add_node(self._node)
            except Exception as e:
                print(f"[{CARD}] ROS2 subscription unavailable: {e}", flush=True)
                self._node = None
                self._pub = None

    def _on_msg(self, msg):
        self._last_msg = msg
        self._received_at_ms = int(time.time() * 1000)

    def _data(self):
        return build(self._last_msg, self._received_at_ms)

    def _tick(self):
        if self._pub is None:
            return
        msg = String()
        msg.data = json.dumps(self._data(), ensure_ascii=False)
        self._pub.publish(msg)

    def get_tool(self):
        return {"name": CARD, "type": TYPE, "multiInstance": False,
                "description": DESC + f" ({SOURCE_TOPIC})",
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
            return {"state": "running" if self._pub else "unavailable", "data": self._data(),
                    "topic_out": topic_out(self._topic, FMT)}
        return None


def make_plugin(plugin_config, namespace, executor, client):
    return Plugin(plugin_config, namespace, executor, client)
