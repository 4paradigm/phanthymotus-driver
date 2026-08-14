# -*- coding: utf-8 -*-
# heartbeat —— Q5 系统心跳 (std_msgs/msg/Header)
# 自建 ROS2 Node 订阅 /system/heartbeat，解析时间戳和 frame_id。

from __future__ import annotations

import json
import time

from sensor_contract import topic_out

try:
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import Header, String

    _HAS_ROS2 = True
    _QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                      history=HistoryPolicy.KEEP_LAST, depth=1,
                      durability=DurabilityPolicy.VOLATILE)
except Exception:
    _HAS_ROS2 = False

CARD = "heartbeat"
TYPE = "sensor"
SOURCE_TOPIC = "/system/heartbeat"
TOPIC = "/{ns}/q5/heartbeat"
FMT = "data/json"
HZ = 2.0
NODE = "q5_heartbeat"
DESC = "Q5 系统心跳：动态启动管理器存活监测"


def build(msg, received_at_ms) -> dict:
    now_ms = int(time.time() * 1000)
    if msg is None:
        return {"timestamp_ms": now_ms, "received_at_ms": received_at_ms,
                "fresh": False, "available": False, "source_topic": SOURCE_TOPIC,
                "message": "未收到心跳消息"}
    age_ms = None if received_at_ms is None else now_ms - received_at_ms
    return {"timestamp_ms": now_ms, "received_at_ms": received_at_ms,
            "age_ms": age_ms, "fresh": age_ms is not None and age_ms <= 5000,
            "available": True,
            "stamp_sec": int(msg.stamp.sec),
            "stamp_nanosec": int(msg.stamp.nanosec),
            "frame_id": str(msg.frame_id),
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
                self._node.create_subscription(Header, SOURCE_TOPIC, self._on_msg, _QOS)
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
