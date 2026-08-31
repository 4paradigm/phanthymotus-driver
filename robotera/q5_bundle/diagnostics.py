"""Q5 standard diagnostics card (read-only)."""

from __future__ import annotations

import json
import time

from sensor_contract import topic_out
from array import array

try:
    from diagnostic_msgs.msg import DiagnosticArray
    from rclpy.node import Node
    from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import String

    _HAS_ROS2 = True
    _QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                      history=HistoryPolicy.KEEP_LAST, depth=10)
except Exception:
    _HAS_ROS2 = False

CARD = "diagnostics"
TYPE = "sensor"
SOURCE_TOPIC = "/diagnostics_agg"
TOPIC = "/{ns}/q5/diagnostics"
FMT = "data/json"
HZ = 2.0
NODE = "q5_diagnostics"
DESC = "Q5 标准诊断数组：保留诊断级别、名称、硬件 ID 和键值"


def _jsonable(value):
    if value is None or isinstance(value, (bool, int, float, str)):
        return value
    if isinstance(value, (bytes, bytearray)):
        return list(value)
    if isinstance(value, array):
        return value.tolist()
    if isinstance(value, (list, tuple)):
        return [_jsonable(item) for item in value]
    fields = getattr(value, "get_fields_and_field_types", None)
    if callable(fields):
        return {name: _jsonable(getattr(value, name)) for name in fields()}
    return str(value)


def build(payload, received_at_ms, publisher_count: int | None = None):
    now_ms = int(time.time() * 1000)
    age_ms = None if received_at_ms is None else now_ms - received_at_ms
    fresh = age_ms is not None and age_ms <= 5000
    publisher_connected = publisher_count is not None and publisher_count > 0
    if payload is not None and fresh:
        source_state, message = "fresh", None
    elif payload is not None:
        source_state, message = "stale", "诊断消息已过期"
    elif publisher_count == 0:
        source_state, message = "diagnostics_publisher_not_running", "诊断发布器未运行"
    elif publisher_count is not None:
        source_state, message = "awaiting_diagnostic_event", "诊断发布器已连接，尚无诊断消息"
    else:
        source_state, message = "unknown", "未收到新鲜诊断消息"
    return {"timestamp_ms": now_ms, "received_at_ms": received_at_ms, "age_ms": age_ms,
            "fresh": fresh, "available": payload is not None,
            "has_diagnostic_message": payload is not None, "data": payload,
            "source_topic": SOURCE_TOPIC, "source_publisher_count": publisher_count,
            "publisher_connected": publisher_connected, "source_state": source_state,
            "message": message}


class Plugin:
    def __init__(self, plugin_config, namespace, executor, client):
        self._topic = TOPIC.format(ns=namespace)
        self._node = None
        self._pub = None
        self._payload = None
        self._received_at_ms = None
        if _HAS_ROS2 and executor is not None:
            try:
                self._node = Node(NODE)
                self._pub = self._node.create_publisher(String, self._topic, _QOS)
                self._node.create_subscription(DiagnosticArray, SOURCE_TOPIC, self._on_message, _QOS)
                self._node.create_timer(1.0 / HZ, self._tick)
                executor.add_node(self._node)
            except Exception as e:
                print(f"[{CARD}] ROS2 subscription unavailable: {e}", flush=True)
                self._node = None
                self._pub = None

    def _on_message(self, msg):
        self._payload = _jsonable(msg)
        self._received_at_ms = int(time.time() * 1000)

    def _publisher_count(self):
        if self._node is None:
            return None
        try:
            return len(self._node.get_publishers_info_by_topic(SOURCE_TOPIC))
        except Exception:
            return None

    def _data(self):
        return build(self._payload, self._received_at_ms, self._publisher_count())

    def _tick(self):
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
