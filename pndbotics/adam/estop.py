"""PNDbotics Adam physical emergency-stop sensor (read-only)."""

from __future__ import annotations

import json
import threading
import time
import urllib.error
import urllib.request

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


def build(state: dict | None, received_at_ms: int | None, *, stale_after_ms: int = 5000) -> dict:
    """Build a fail-safe payload from PAC physical power-state responses."""
    now_ms = int(time.time() * 1000)
    age_ms = None if received_at_ms is None else max(0, now_ms - received_at_ms)
    fresh = age_ms is not None and age_ms <= stale_after_ms
    state = state or {}
    actuator = state.get("actuator_status")
    rcu_power = state.get("rcu_power_enabled")
    signals = [value for value in (actuator, rcu_power) if isinstance(value, bool)]
    detection_supported = bool(signals)
    available = detection_supported
    detected = any(value is False for value in signals)
    disagreement = (
        isinstance(actuator, bool)
        and isinstance(rcu_power, bool)
        and actuator != rcu_power
    )

    if not available:
        message = state.get("error") or "未收到执行器供电状态"
    elif not fresh:
        message = "执行器供电状态已过期，急停状态未知"
    elif disagreement:
        message = "执行器与 RCU 电源状态不一致，按急停处理"
    elif detected:
        message = "执行器电源已断开，实体急停已触发"
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
        "actuator_status": actuator,
        "rcu_power_enabled": rcu_power,
        "fsm_state": state.get("fsm_state"),
        "message": message,
    }


class EStopPlugin:
    """Publishes the physical emergency-stop state without issuing commands."""

    def __init__(self, plugin_config: dict, namespace: str, executor, grpc_client, **kwargs):
        self._grpc = grpc_client
        self._executor = executor
        self._topic = TOPIC.format(namespace=namespace)
        self._stale_after_ms = int(float(plugin_config.get("state_timeout_sec", 5.0)) * 1000)
        self._pac_url = str(plugin_config.get("pac_url", "http://10.10.20.127:8626")).rstrip("/")
        self._http_timeout = max(0.1, float(plugin_config.get("http_timeout_sec", 2.0)))
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

    def _get_json(self, path: str) -> dict:
        request = urllib.request.Request(
            f"{self._pac_url}{path}", headers={"Accept": "application/json"}
        )
        with urllib.request.urlopen(request, timeout=self._http_timeout) as response:
            return json.loads(response.read().decode("utf-8"))

    def _read_pac_status(self) -> dict:
        state = {}
        errors = []
        try:
            payload = self._get_json("/robot_command/actuator_status/")
            value = (payload.get("data") or {}).get("actuator_status")
            if isinstance(value, bool):
                state["actuator_status"] = value
            else:
                errors.append("PAC 未返回 actuator_status")
        except (OSError, ValueError, urllib.error.URLError) as exc:
            errors.append(f"actuator_status: {exc}")

        try:
            payload = self._get_json("/rcu_settings/rcu16/status")
            value = ((payload.get("data") or {}).get("enable_states") or {}).get("power")
            if isinstance(value, bool):
                state["rcu_power_enabled"] = value
            else:
                errors.append("RCU 未返回 enable_states.power")
        except (OSError, ValueError, urllib.error.URLError) as exc:
            errors.append(f"rcu_power: {exc}")

        try:
            grpc_state = self._grpc.get_robot_state()
            state["fsm_state"] = grpc_state.get("fsm_state", grpc_state.get("mode"))
        except Exception:
            pass

        if errors:
            state["error"] = "; ".join(errors)
        return state

    def _refresh(self):
        try:
            state = self._read_pac_status()
        except Exception as exc:
            state = {"error": str(exc)}
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
            "description": "Adam 实体急停监测：只读，根据执行器与 RCU 电源状态判断",
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
