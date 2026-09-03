"""Q5 compact IMU (accelerometer + gyroscope) state card.

Publishes a normalized, self-describing JSON payload on
``/{ns}/q5/imu``.  The raw accelerometer/gyroscope vectors are already
subscribed by the Q5 SDK client (``q5_accel`` and ``q5_gyro``); this card is a
thin, stable view over the shared ``imu`` snapshot so it never depends on the
SDK's internal topic names or field layout.
"""

from __future__ import annotations

import json
import time

from sensor_contract import topic_out

try:
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import String

    _HAS_ROS2 = True
    _QOS = QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        durability=DurabilityPolicy.VOLATILE,
    )
except Exception:
    _HAS_ROS2 = False

CARD = "imu"
TYPE = "sensor"
TOPIC = "/{ns}/q5/imu"
FMT = "data/json"
HZ = 2.0
NODE = "q5_imu"
DESC = "Q5 IMU：线加速度（加速度计）和角速度（陀螺仪），2Hz 发布"


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

    def _data(self) -> dict:
        """Normalized, SDK-agnostic IMU snapshot.

        Every field is read with a default so a partial or changed SDK snapshot
        can never raise; an unread sensor simply reports ``available=False``.
        """
        snap = self._client.sensor_snapshot("imu")
        return {
            "available": bool(snap.get("available", False)),
            "fresh": bool(snap.get("fresh", False)),
            "age_ms": snap.get("age_ms"),
            "linear_acceleration": snap.get("linear_acceleration"),
            "angular_velocity": snap.get("angular_velocity"),
            "received_at_ms": snap.get("received_at_ms"),
            "message_timestamp_ms": snap.get("message_timestamp_ms"),
        }

    def _tick(self):
        if self._pub is None:
            return
        try:
            payload = json.dumps(self._data(), ensure_ascii=False)
        except Exception:
            return
        msg = String()
        msg.data = payload
        self._pub.publish(msg)

    def get_tool(self):
        return {
            "name": CARD,
            "type": TYPE,
            "multiInstance": False,
            "description": DESC,
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {
                        "type": "string",
                        "enum": ["info", "start", "stop"],
                        "description": "info=读一次; start=2Hz 发布; stop=停止发布。",
                    }
                },
                "required": ["action"],
                "additionalProperties": False,
            },
            "topic_out": topic_out(self._topic, FMT),
        }

    def dispatch(self, action, args):
        running = self._pub is not None
        if action == "start":
            return {"state": "running" if running else "unavailable"}
        if action == "stop":
            return {"state": "idle"}
        if action in ("info", "read", "get", "show", CARD):
            return {
                "state": "running" if running else "unavailable",
                "available": bool(self._data().get("available", False)),
                "fresh": bool(self._data().get("fresh", False)),
                "topic_out": topic_out(self._topic, FMT),
                "data": self._data(),
            }
        return None


def make_plugin(plugin_config, namespace, executor, client):
    return Plugin(plugin_config, namespace, executor, client)
