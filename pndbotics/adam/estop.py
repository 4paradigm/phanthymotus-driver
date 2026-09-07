"""PNDbotics Adam emergency-stop state sensor (read-only)."""

from __future__ import annotations

import json
import threading
import time

try:
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import String

    HAS_ROS2 = True
    QOS = QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )
except Exception:
    HAS_ROS2 = False


CARD = "estop"
TOPIC = "/{namespace}/state/estop"
FORMAT = "data/json"
ESTOP_STATES = frozenset({"E_STOP", "ESTOP", "EMERGENCY_STOP"})


def _normalized_state(value) -> str | None:
    if not isinstance(value, str):
        return None
    normalized = value.strip().upper().replace("-", "_").replace(" ", "_")
    return normalized or None


def build(state: dict | None, received_at_ms: int | None, *, stale_after_ms: int = 5000) -> dict:
    """Build a fail-safe payload from a gRPC robot-state response."""
    now_ms = int(time.time() * 1000)
    age_ms = None if received_at_ms is None else max(0, now_ms - received_at_ms)
    fresh = age_ms is not None and age_ms <= stale_after_ms
    state = state or {}
    raw_state = state.get("fsm_state", state.get("mode"))
    normalized = _normalized_state(raw_state)
    detection_supported = normalized is not None
    reported = detection_supported and normalized in ESTOP_STATES
    detected = bool(fresh and reported)
    available = raw_state is not None and "error" not in state

    if not available:
        message = state.get("error") or "未收到机器人状态"
    elif not detection_supported:
        message = "当前 PND 接口仅返回数字 mode，无法可靠判断物理急停"
    elif not fresh:
        message = "机器人状态已过期，急停状态未知"
    elif reported:
        message = "Adam FSM 报告 E_STOP"
    else:
        message = None

    return {
        "timestamp_ms": now_ms,
        "received_at_ms": received_at_ms,
        "age_ms": age_ms,
        "fresh": fresh,
        "available": available,
        "detection_supported": detection_supported,
        "emergency_stop": detected if detection_supported and fresh else None,
        "fsm_estop_detected": detected,
        "fsm_estop_reported": bool(reported),
        "fsm_state": raw_state,
        "message": message,
    }


class Plugin:
    def __init__(self, plugin_config: dict, namespace: str, executor, grpc_client, **kwargs):
        self._grpc = grpc_client
        self._executor = executor
        self._topic = TOPIC.format(namespace=namespace)
        self._stale_after_ms = int(float(plugin_config.get("state_timeout_sec", 5.0)) * 1000)
        self._state = None
        self._received_at_ms = None
        self._active = False
        self._lock = threading.Lock()
        self._node = None
        self._pub = None

        if HAS_ROS2 and executor is not None:
            try:
                self._node = Node("adam_estop")
                self._pub = self._node.create_publisher(String, self._topic, QOS)
                rate = max(0.1, float(plugin_config.get("poll_rate_hz", 2.0)))
                self._node.create_timer(1.0 / rate, self._tick)
                executor.add_node(self._node)
            except Exception as exc:
                print(f"[estop] ROS2 publisher unavailable: {exc}", flush=True)
                self._node = None
                self._pub = None

    def _refresh(self):
        state = self._grpc.get_robot_state()
        with self._lock:
            self._state = state
            self._received_at_ms = int(time.time() * 1000)

    def _data(self, *, refresh: bool = False):
        if refresh:
            self._refresh()
        with self._lock:
            state = self._state
            received_at_ms = self._received_at_ms
        return build(state, received_at_ms, stale_after_ms=self._stale_after_ms)

    def _tick(self):
        if not self._active or self._pub is None:
            return
        self._refresh()
        msg = String()
        msg.data = json.dumps(self._data(), ensure_ascii=False)
        self._pub.publish(msg)

    def get_tool(self):
        return {
            "name": CARD,
            "type": "sensor",
            "description": "Adam 急停状态监测：只读，不提供解除急停或电源控制",
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {"type": "string", "enum": ["info", "start", "stop"]},
                },
                "required": ["action"],
                "additionalProperties": False,
            },
            "topic_out": [{"topic": self._topic, "format": FORMAT}],
        }

    def start(self):
        self._active = True
        return {"state": "running" if self._pub else "unavailable"}

    def stop(self):
        self._active = False
        return {"state": "idle"}

    def close(self):
        self.stop()
        if self._node is not None:
            try:
                self._executor.remove_node(self._node)
            except Exception:
                pass
            self._node.destroy_node()
            self._node = None
            self._pub = None

    def dispatch(self, action: str, args: dict):
        if action == "start":
            return self.start()
        if action == "stop":
            return self.stop()
        if action in ("info", "read", "get", CARD):
            return {
                "state": "running" if self._active and self._pub else "unavailable",
                "data": self._data(refresh=True),
                "topic_out": [{"topic": self._topic, "format": FORMAT}],
            }
        return None


def make_plugin(plugin_config: dict, namespace: str, executor, grpc_client):
    return Plugin(plugin_config, namespace, executor, grpc_client)
