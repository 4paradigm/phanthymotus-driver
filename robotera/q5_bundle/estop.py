"""Q5 emergency-stop state monitor (read-only)."""

from __future__ import annotations

import json
import time

from sensor_contract import topic_out

try:
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import String
    from xbot_common_interfaces.msg import RobotStatus

    _HAS_ROS2 = True
    _QOS = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                      history=HistoryPolicy.KEEP_LAST, depth=1,
                      durability=DurabilityPolicy.TRANSIENT_LOCAL)
except Exception:
    _HAS_ROS2 = False

CARD = "estop"
TYPE = "sensor"
SOURCE_TOPIC = "/xbot_state"
TOPIC = "/{ns}/q5/estop"
FMT = "data/json"
HZ = 2.0
NODE = "q5_estop"
DESC = "Q5 急停状态监测：只读，不提供解除急停或电源控制"
E_STOP = 7


def build(state: int | None, message: str | None, received_at_ms: int | None) -> dict:
    now_ms = int(time.time() * 1000)
    age_ms = None if received_at_ms is None else now_ms - received_at_ms
    fresh = age_ms is not None and age_ms <= 5000
    fsm_estop_reported = state == E_STOP
    # A retained or old E_STOP report must remain visible, but it cannot prove
    # that the robot is currently emergency-stopped once the source is stale.
    fsm_estop_detected = fresh and fsm_estop_reported
    if state is None or not fresh:
        status_message = "未收到新鲜 Q5 FSM 状态"
    elif fsm_estop_reported:
        status_message = "Q5 FSM 报告 E_STOP"
    else:
        status_message = None
    return {"timestamp_ms": now_ms, "received_at_ms": received_at_ms, "age_ms": age_ms,
            "fresh": fresh, "available": state is not None,
            "emergency_stop": fsm_estop_detected,
            "fsm_estop_detected": fsm_estop_detected,
            "fsm_estop_reported": fsm_estop_reported,
            "fsm_state": state, "fsm_message": message,
            "source_topic": SOURCE_TOPIC,
            "message": status_message}


class Plugin:
    def __init__(self, plugin_config, namespace, executor, client):
        self._topic = TOPIC.format(ns=namespace)
        self._node = None
        self._pub = None
        self._state = None
        self._message = None
        self._received_at_ms = None
        if _HAS_ROS2 and executor is not None:
            try:
                self._node = Node(NODE)
                self._pub = self._node.create_publisher(String, self._topic, _QOS)
                self._node.create_subscription(RobotStatus, SOURCE_TOPIC, self._on_state, _QOS)
                self._node.create_timer(1.0 / HZ, self._tick)
                executor.add_node(self._node)
            except Exception as e:
                print(f"[{CARD}] ROS2 subscription unavailable: {e}", flush=True)
                self._node = None
                self._pub = None

    def _on_state(self, msg):
        self._state = int(msg.state)
        self._message = str(msg.msg)
        self._received_at_ms = int(time.time() * 1000)

    def _data(self):
        return build(self._state, self._message, self._received_at_ms)

    def _tick(self):
        msg = String()
        msg.data = json.dumps(self._data(), ensure_ascii=False)
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
            return {"state": "running" if self._pub else "unavailable", "data": self._data(),
                    "topic_out": topic_out(self._topic, FMT)}
        return None


def make_plugin(plugin_config, namespace, executor, client):
    return Plugin(plugin_config, namespace, executor, client)
