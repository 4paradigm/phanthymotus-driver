"""Unified Q5 operational health card (read-only)."""

from __future__ import annotations

import json
import time
from array import array

from sensor_contract import topic_out

try:
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import String
    from xbot_common_interfaces.msg import MotionStatus, RobotStatus, Temperature

    try:
        from xbot_common_interfaces.msg import FaultArray
    except Exception:
        FaultArray = None

    _HAS_ROS2 = True
    _VOLATILE_QOS = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                               history=HistoryPolicy.KEEP_LAST, depth=10,
                               durability=DurabilityPolicy.VOLATILE)
    _LATCHED_QOS = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                              history=HistoryPolicy.KEEP_LAST, depth=1,
                              durability=DurabilityPolicy.TRANSIENT_LOCAL)
except Exception:
    _HAS_ROS2 = False

CARD = "system_health"
TYPE = "sensor"
TOPIC = "/{ns}/q5/system_health"
FMT = "data/json"
HZ = 2.0
NODE = "q5_system_health"
DESC = "Q5 运行健康：运动状态、FSM 状态、温度和聚合故障"
STALE_THRESHOLD_MS = 5000
ROBOT_STATE_LABELS = {
    0: "INIT", 1: "SELF_TEST", 2: "IDLE", 3: "READY", 4: "ACTIVE",
    5: "SHUTDOWN", 6: "OTA", 7: "E_STOP", -1: "ERROR",
}
SOURCES = {
    "motion": "/motion_manager/motion_status",
    "robot": "/xbot_state",
    "temperature": "/temperature",
    "faults": "/fault_array_agg",
}
SOURCE_META = {
    # MotionStatus is currently observed as an event-driven topic. Its lack of
    # a recent event must not make the otherwise live robot health stale.
    "motion": {"required": False, "event_driven": True},
    "robot": {"required": True, "event_driven": False},
    "temperature": {"required": True, "event_driven": False},
    "faults": {"required": False, "event_driven": True},
}


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


def _source(payload, received_at_ms, source_topic: str, now_ms: int,
            publisher_count: int | None, event_driven: bool) -> dict:
    age_ms = None if received_at_ms is None else now_ms - received_at_ms
    fresh = age_ms is not None and age_ms <= STALE_THRESHOLD_MS
    if payload is not None and fresh:
        report_state = "fresh"
    elif payload is not None:
        report_state = "stale"
    elif publisher_count == 0:
        report_state = "publisher_not_running"
    elif event_driven and publisher_count is not None:
        report_state = "awaiting_event"
    elif payload is not None:
        report_state = "stale"
    else:
        report_state = "awaiting_message"
    return {
        "available": payload is not None,
        "fresh": fresh,
        "age_ms": age_ms,
        "received_at_ms": received_at_ms,
        "source_topic": source_topic,
        "source_publisher_count": publisher_count,
        "report_state": report_state,
        "event_driven": event_driven,
        "data": payload,
    }


def _temperature_summary(payload: dict | None) -> dict | None:
    if not isinstance(payload, dict):
        return None
    names, values = payload.get("name"), payload.get("temperature")
    if not isinstance(names, list) or not isinstance(values, list) or len(names) != len(values):
        return None
    readings = [(name, value) for name, value in zip(names, values)
                if isinstance(name, str) and isinstance(value, (int, float))]
    if not readings:
        return None
    maximum = max(readings, key=lambda item: item[1])
    minimum = min(readings, key=lambda item: item[1])
    return {
        "reading_count": len(readings),
        "maximum_celsius": maximum[1], "maximum_sensor": maximum[0],
        "minimum_celsius": minimum[1], "minimum_sensor": minimum[0],
    }


def build(snapshots: dict, publisher_counts: dict | None = None,
          now_ms: int | None = None) -> dict:
    now_ms = int(time.time() * 1000) if now_ms is None else now_ms
    publisher_counts = publisher_counts or {}
    result = {"timestamp_ms": now_ms}
    for name, source_topic in SOURCES.items():
        snapshot = snapshots.get(name, {})
        meta = SOURCE_META[name]
        result[name] = _source(snapshot.get("payload"), snapshot.get("received_at_ms"), source_topic,
                               now_ms, publisher_counts.get(name), meta["event_driven"])

    robot_data = result["robot"]["data"] or {}
    robot_state = robot_data.get("state") if isinstance(robot_data, dict) else None
    if isinstance(result["robot"]["data"], dict):
        result["robot"]["state_label"] = ROBOT_STATE_LABELS.get(robot_state, "UNKNOWN")
    available_sources = [name for name in SOURCES if result[name]["available"]]
    stale_sources = [name for name in SOURCES if result[name]["available"] and not result[name]["fresh"]]
    required_sources = [name for name, meta in SOURCE_META.items() if meta["required"]]
    missing_required_sources = [name for name in required_sources if not result[name]["available"]]
    stale_required_sources = [name for name in required_sources
                              if result[name]["available"] and not result[name]["fresh"]]
    event_sources_waiting = [name for name, meta in SOURCE_META.items()
                             if meta["event_driven"] and result[name]["report_state"] == "awaiting_event"]
    result["summary"] = {
        "available_sources": available_sources,
        "stale_sources": stale_sources,
        "robot_active": robot_state == 4,
        "required_sources": required_sources,
        "missing_required_sources": missing_required_sources,
        "stale_required_sources": stale_required_sources,
        "event_sources_waiting": event_sources_waiting,
        "all_required_sources_fresh": not missing_required_sources and not stale_required_sources,
    }
    result["available"] = bool(available_sources)
    result["fresh"] = result["summary"]["all_required_sources_fresh"]
    result["source_topics"] = dict(SOURCES)
    result["temperature_summary"] = _temperature_summary(result["temperature"]["data"])
    if not available_sources:
        result["message"] = "未收到 Q5 运行健康消息"
    elif missing_required_sources or stale_required_sources:
        result["message"] = "必需运行健康来源未就绪或已过期"
    elif event_sources_waiting:
        result["message"] = "必需健康遥测正常；部分事件式来源暂无新事件"
    else:
        result["message"] = "必需健康遥测正常"
    return result


class Plugin:
    def __init__(self, plugin_config, namespace, executor, client):
        self._topic = TOPIC.format(ns=namespace)
        self._node = None
        self._pub = None
        self._snapshots = {name: {"payload": None, "received_at_ms": None} for name in SOURCES}
        if _HAS_ROS2 and executor is not None:
            try:
                self._node = Node(NODE)
                self._pub = self._node.create_publisher(String, self._topic, _VOLATILE_QOS)
                self._node.create_subscription(MotionStatus, SOURCES["motion"], self._on_motion, _VOLATILE_QOS)
                self._node.create_subscription(RobotStatus, SOURCES["robot"], self._on_robot, _LATCHED_QOS)
                self._node.create_subscription(Temperature, SOURCES["temperature"], self._on_temperature, _VOLATILE_QOS)
                if FaultArray is not None:
                    self._node.create_subscription(FaultArray, SOURCES["faults"], self._on_faults, _VOLATILE_QOS)
                self._node.create_timer(1.0 / HZ, self._tick)
                executor.add_node(self._node)
            except Exception as e:
                print(f"[{CARD}] ROS2 subscriptions unavailable: {e}", flush=True)
                self._node = None
                self._pub = None

    def _set(self, name, payload):
        self._snapshots[name] = {"payload": payload, "received_at_ms": int(time.time() * 1000)}

    def _on_motion(self, msg):
        # Q5 releases observed in the field expose this as ``state``; older
        # interface builds called it ``status``.  Keep both keys in the JSON
        # contract so a vendor interface update cannot make health appear stale.
        value = getattr(msg, "state", getattr(msg, "status", None))
        state = int(value) if value is not None else None
        self._set("motion", {
            "state": state,
            "status": state,
            "message": str(getattr(msg, "msg", "")) or None,
        })

    def _on_robot(self, msg):
        self._set("robot", {"state": int(msg.state), "message": str(msg.msg)})

    def _on_temperature(self, msg):
        self._set("temperature", _jsonable(msg))

    def _on_faults(self, msg):
        self._set("faults", _jsonable(msg))

    def _data(self):
        publisher_counts = {}
        for name, source_topic in SOURCES.items():
            try:
                publisher_counts[name] = len(self._node.get_publishers_info_by_topic(source_topic))
            except Exception:
                publisher_counts[name] = None
        return build(self._snapshots, publisher_counts=publisher_counts)

    def _tick(self):
        msg = String()
        msg.data = json.dumps(self._data(), ensure_ascii=False)
        self._pub.publish(msg)

    def get_tool(self):
        return {
            "name": CARD,
            "type": TYPE,
            "multiInstance": False,
            "description": DESC,
            "inputSchema": {
                "type": "object",
                "properties": {"action": {"type": "string", "enum": ["info", "start", "stop"]}},
                "required": ["action"],
                "additionalProperties": False,
            },
            "topic_out": topic_out(self._topic, FMT),
        }

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
