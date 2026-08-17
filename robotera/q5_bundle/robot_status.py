# -*- coding: utf-8 -*-
# robot_status —— 聚合 robot_ready + system_health + diagnostics + estop
# 一个插件聚合五个维度：
#   1. 机器人业务状态 FSM (from /xbot_state → robot_ready / estop)
#   2. 运动管理器生命周期 (from /motion_manager/get_state → robot_ready)
#   3. 系统健康: motion + temperature + faults (from system_health)
#   4. 诊断数组 (from /diagnostics_agg → diagnostics)
#   5. 急停状态 (from /xbot_state → estop)

from __future__ import annotations

import json
import time
from array import array

from sensor_contract import topic_out

try:
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import String
    from xbot_common_interfaces.msg import RobotStatus, MotionStatus, Temperature

    try:
        from xbot_common_interfaces.msg import FaultArray
    except Exception:
        FaultArray = None

    from diagnostic_msgs.msg import DiagnosticArray
    from lifecycle_msgs.srv import GetState

    _HAS_ROS2 = True
    _VOLATILE_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                               history=HistoryPolicy.KEEP_LAST, depth=1,
                               durability=DurabilityPolicy.VOLATILE)
    _LATCHED_QOS = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                              history=HistoryPolicy.KEEP_LAST, depth=1,
                              durability=DurabilityPolicy.TRANSIENT_LOCAL)
    _DIAG_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                            history=HistoryPolicy.KEEP_LAST, depth=10)
    _MOTION_QOS = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                             history=HistoryPolicy.KEEP_LAST, depth=10,
                             durability=DurabilityPolicy.VOLATILE)
except Exception:
    _HAS_ROS2 = False

CARD = "robot_status"
TYPE = "sensor"
TOPIC = "/{ns}/q5/robot_status"
FMT = "data/json"
HZ = 2.0
NODE = "q5_robot_status"
DESC = "Q5 机器人综合状态：聚合 robot_ready + system_health + diagnostics + estop"
STALE_THRESHOLD_MS = 5000

ROBOT_STATE_LABELS = {
    0: "INIT", 1: "SELF_TEST", 2: "IDLE", 3: "READY", 4: "ACTIVE",
    5: "SHUTDOWN", 6: "OTA", 7: "E_STOP", -1: "ERROR",
}
CONTROL_READY_STATES = {3, 4}
E_STOP = 7


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


def _source_status(payload, received_at_ms, publisher_count: int | None,
                   now_ms: int) -> dict:
    age_ms = None if received_at_ms is None else now_ms - received_at_ms
    fresh = age_ms is not None and age_ms <= STALE_THRESHOLD_MS
    available = payload is not None
    if payload is not None and fresh:
        report_state = "fresh"
    elif payload is not None:
        report_state = "stale"
    elif publisher_count == 0:
        report_state = "publisher_not_running"
    elif publisher_count is not None:
        report_state = "awaiting_event"
    else:
        report_state = "awaiting_message"
    return {
        "available": available,
        "fresh": fresh,
        "age_ms": age_ms,
        "received_at_ms": received_at_ms,
        "publisher_count": publisher_count,
        "report_state": report_state,
        "data": payload,
    }


def build(fsm_state, fsm_message, fsm_received_at_ms,
          lifecycle_state, diag_payload, diag_received_at_ms, diag_publisher_count,
          motion_payload, motion_received_at_ms, motion_publisher_count,
          temp_payload, temp_received_at_ms, temp_publisher_count,
          fault_payload, fault_received_at_ms, fault_publisher_count,
          now_ms=None):
    now_ms = int(time.time() * 1000) if now_ms is None else now_ms

    # === Dimension 1: FSM (robot_ready) ===
    fsm_age = None if fsm_received_at_ms is None else now_ms - fsm_received_at_ms
    fsm_fresh = fsm_age is not None and fsm_age <= STALE_THRESHOLD_MS
    state_label = ROBOT_STATE_LABELS.get(fsm_state, "UNKNOWN")
    ready = fsm_fresh and fsm_state in CONTROL_READY_STATES

    # === Dimension 2: motion manager lifecycle (robot_ready) ===
    motion_manager_active = lifecycle_state == "active"

    # === Dimension 3: estop (estop) ===
    estop_active = fsm_fresh and fsm_state == E_STOP

    # === Dimension 4: diagnostics (diagnostics) ===
    diag_age = None if diag_received_at_ms is None else now_ms - diag_received_at_ms
    diag_fresh = diag_age is not None and diag_age <= STALE_THRESHOLD_MS
    diag_connected = diag_publisher_count is not None and diag_publisher_count > 0

    # === Dimension 5: system_health (motion, temperature, faults) ===
    motion_st = _source_status(motion_payload, motion_received_at_ms,
                               motion_publisher_count, now_ms)
    temp_st = _source_status(temp_payload, temp_received_at_ms,
                             temp_publisher_count, now_ms)
    fault_st = _source_status(fault_payload, fault_received_at_ms,
                              fault_publisher_count, now_ms)
    temp_summary = _temperature_summary(temp_payload)

    # === Composite message ===
    if not fsm_fresh:
        message = "机器人状态未知：未收到新鲜 /xbot_state"
    elif estop_active:
        message = "急停激活"
    elif not ready:
        message = f"状态: {state_label} | 运动管理器: {lifecycle_state.upper()}"
    else:
        message = f"状态: {state_label} | 运动管理器: {lifecycle_state.upper()}"

    return {
        "timestamp_ms": now_ms,

        # --- 综合 ---
        "fresh": fsm_fresh and temp_st["fresh"],
        "available": fsm_state is not None,
        "ready": ready and motion_manager_active,
        "message": message,

        # --- robot_ready (业务状态 + 运动管理器) ---
        "robot_status": {
            "state": state_label,
            "state_code": fsm_state,
            "fresh": fsm_fresh,
            "ready": ready,
            "message": fsm_message,
            "source": "/xbot_state",
        },
        "motion_manager": {
            "state": lifecycle_state,
            "active": motion_manager_active,
            "motion_ready": fsm_fresh and motion_manager_active,
            "source": "/motion_manager/get_state",
        },

        # --- estop (急停) ---
        "estop": {
            "active": estop_active,
            "reported": fsm_state == E_STOP,
            "fresh": fsm_fresh,
            "state_code": fsm_state,
            "message": "急停激活" if estop_active else None,
            "source": "/xbot_state",
        },

        # --- diagnostics (诊断数组) ---
        "diagnostics": {
            "fresh": diag_fresh,
            "available": diag_payload is not None,
            "connected": diag_connected,
            "publisher_count": diag_publisher_count,
            "age_ms": diag_age,
            "data": diag_payload if diag_fresh else None,
            "source": "/diagnostics_agg",
        },

        # --- system_health (运动 + 温度 + 故障) ---
        "system_health": {
            "motion": {
                **motion_st,
                "source": "/motion_manager/motion_status",
            },
            "temperature": {
                **temp_st,
                "summary": temp_summary,
                "source": "/temperature",
            },
            "faults": {
                **fault_st,
                "source": "/fault_array_agg",
            },
        },

        "source_topics": {
            "fsm": "/xbot_state",
            "motion_manager": "/motion_manager/get_state",
            "estop": "/xbot_state",
            "diagnostics": "/diagnostics_agg",
            "motion": "/motion_manager/motion_status",
            "temperature": "/temperature",
            "faults": "/fault_array_agg",
        },
    }


class Plugin:
    def __init__(self, plugin_config, namespace, executor, client):
        self._topic = TOPIC.format(ns=namespace)
        self._node = None
        self._pub = None

        # Dimension 1+3: FSM + estop (from /xbot_state)
        self._fsm_state = None
        self._fsm_message = None
        self._fsm_received_at_ms = None

        # Dimension 2: motion manager lifecycle
        self._lifecycle_state = "unknown"
        self._lifecycle_client = None
        self._lifecycle_request_type = None
        self._lifecycle_pending = False

        # Dimension 4: diagnostics
        self._diag_payload = None
        self._diag_received_at_ms = None

        # Dimension 5: system_health
        self._motion_payload = None
        self._motion_received_at_ms = None
        self._temp_payload = None
        self._temp_received_at_ms = None
        self._fault_payload = None
        self._fault_received_at_ms = None

        if _HAS_ROS2 and executor is not None:
            try:
                self._node = Node(NODE)
                self._pub = self._node.create_publisher(String, self._topic, _VOLATILE_QOS)

                # /xbot_state: FSM + estop
                self._node.create_subscription(RobotStatus, "/xbot_state", self._on_robot, _LATCHED_QOS)

                # /diagnostics_agg
                self._node.create_subscription(DiagnosticArray, "/diagnostics_agg", self._on_diag, _DIAG_QOS)

                # /motion_manager/motion_status
                self._node.create_subscription(MotionStatus, "/motion_manager/motion_status",
                                               self._on_motion, _MOTION_QOS)

                # /temperature
                self._node.create_subscription(Temperature, "/temperature",
                                               self._on_temperature, _MOTION_QOS)

                # /fault_array_agg (optional)
                if FaultArray is not None:
                    self._node.create_subscription(FaultArray, "/fault_array_agg",
                                                   self._on_faults, _MOTION_QOS)

                # motion_manager lifecycle
                self._lifecycle_client = self._node.create_client(GetState, "/motion_manager/get_state")
                self._lifecycle_request_type = GetState.Request
                self._node.create_timer(1.0, self._poll_lifecycle)

                self._node.create_timer(1.0 / HZ, self._tick)
                executor.add_node(self._node)
            except Exception as e:
                print(f"[{CARD}] ROS2 init failed: {e}", flush=True)
                self._node = None
                self._pub = None

    def _on_robot(self, msg):
        self._fsm_state = int(msg.state)
        self._fsm_message = str(msg.msg)
        self._fsm_received_at_ms = int(time.time() * 1000)

    def _on_diag(self, msg):
        self._diag_payload = _jsonable(msg)
        self._diag_received_at_ms = int(time.time() * 1000)

    def _on_motion(self, msg):
        value = getattr(msg, "state", getattr(msg, "status", None))
        state = int(value) if value is not None else None
        self._motion_payload = {
            "state": state,
            "status": state,
            "message": str(getattr(msg, "msg", "")) or None,
        }
        self._motion_received_at_ms = int(time.time() * 1000)

    def _on_temperature(self, msg):
        self._temp_payload = _jsonable(msg)
        self._temp_received_at_ms = int(time.time() * 1000)

    def _on_faults(self, msg):
        self._fault_payload = _jsonable(msg)
        self._fault_received_at_ms = int(time.time() * 1000)

    def _poll_lifecycle(self):
        if self._lifecycle_client is None or self._lifecycle_pending:
            return
        if not self._lifecycle_client.service_is_ready():
            self._lifecycle_state = "service_unavailable"
            return
        try:
            future = self._lifecycle_client.call_async(self._lifecycle_request_type())
            self._lifecycle_pending = True
            future.add_done_callback(self._on_lifecycle)
        except Exception:
            self._lifecycle_pending = False

    def _on_lifecycle(self, future):
        try:
            response = future.result()
            state = getattr(response, "current_state", None)
            label = str(getattr(state, "label", "unknown") or "unknown")
            self._lifecycle_state = label
        except Exception:
            pass
        finally:
            self._lifecycle_pending = False

    def _publisher_counts(self):
        if self._node is None:
            return None, None, None, None
        try:
            diag = len(self._node.get_publishers_info_by_topic("/diagnostics_agg"))
        except Exception:
            diag = None
        try:
            motion = len(self._node.get_publishers_info_by_topic("/motion_manager/motion_status"))
        except Exception:
            motion = None
        try:
            temp = len(self._node.get_publishers_info_by_topic("/temperature"))
        except Exception:
            temp = None
        try:
            fault = len(self._node.get_publishers_info_by_topic("/fault_array_agg"))
        except Exception:
            fault = None
        return diag, motion, temp, fault

    def _data(self):
        diag, motion, temp, fault = self._publisher_counts()
        return build(
            self._fsm_state, self._fsm_message, self._fsm_received_at_ms,
            self._lifecycle_state,
            self._diag_payload, self._diag_received_at_ms, diag,
            self._motion_payload, self._motion_received_at_ms, motion,
            self._temp_payload, self._temp_received_at_ms, temp,
            self._fault_payload, self._fault_received_at_ms, fault,
        )

    def _tick(self):
        if self._pub is None:
            return
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
