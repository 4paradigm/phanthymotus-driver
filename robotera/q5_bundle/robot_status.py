# -*- coding: utf-8 -*-
# robot_status —— 合并 robot_ready + estop + diagnostics
# 一个插件聚合三个维度：机器人业务状态(FSM)、急停状态、诊断数组。

from __future__ import annotations

import json
import time
from array import array

from sensor_contract import topic_out

try:
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import String
    from xbot_common_interfaces.msg import RobotStatus
    from diagnostic_msgs.msg import DiagnosticArray
    from lifecycle_msgs.srv import GetState

    _HAS_ROS2 = True
    _VOLATILE_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                               history=HistoryPolicy.KEEP_LAST, depth=1,
                               durability=DurabilityPolicy.VOLATILE)
    _LATCHED_QOS = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                              history=HistoryPolicy.KEEP_LAST, depth=1,
                              durability=DurabilityPolicy.TRANSIENT_LOCAL)
    _DIAG_QOS = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                            history=HistoryPolicy.KEEP_LAST, depth=10)
except Exception:
    _HAS_ROS2 = False

CARD = "robot_status"
TYPE = "sensor"
TOPIC = "/{ns}/q5/robot_status"
FMT = "data/json"
HZ = 2.0
NODE = "q5_robot_status"
DESC = "Q5 机器人综合状态：业务状态(FSM)、急停、诊断、运动管理器"
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


def build(fsm_state, fsm_message, fsm_received_at_ms,
          lifecycle_state, diag_payload, diag_received_at_ms,
          diag_publisher_count, now_ms=None):
    now_ms = int(time.time() * 1000) if now_ms is None else now_ms

    # --- 维度 1: 机器人业务状态 (FSM from /xbot_state) ---
    fsm_age = None if fsm_received_at_ms is None else now_ms - fsm_received_at_ms
    fsm_fresh = fsm_age is not None and fsm_age <= STALE_THRESHOLD_MS
    state_label = ROBOT_STATE_LABELS.get(fsm_state, "UNKNOWN")
    ready = fsm_fresh and fsm_state in CONTROL_READY_STATES

    # --- 维度 2: 急停状态 ---
    estop_active = fsm_fresh and fsm_state == E_STOP
    estop_reported = fsm_state == E_STOP

    # --- 维度 3: 诊断数组 ---
    diag_age = None if diag_received_at_ms is None else now_ms - diag_received_at_ms
    diag_fresh = diag_age is not None and diag_age <= STALE_THRESHOLD_MS
    diag_connected = diag_publisher_count is not None and diag_publisher_count > 0

    # --- 维度 4: 运动管理器生命周期 ---
    motion_manager_active = lifecycle_state == "active"

    # 综合消息
    if not fsm_fresh:
        message = "机器人状态未知：未收到新鲜 /xbot_state"
    elif estop_active:
        message = "急停激活"
    elif not ready:
        message = "状态: %s | 运动管理器: %s" % (state_label, lifecycle_state.upper())
    else:
        message = "状态: %s | 运动管理器: %s" % (state_label, lifecycle_state.upper())

    return {
        "timestamp_ms": now_ms,

        # 综合
        "fresh": fsm_fresh,
        "available": fsm_state is not None,
        "ready": ready,
        "message": message,

        # 业务状态
        "robot_state": state_label,
        "robot_state_code": fsm_state,
        "robot_state_fresh": fsm_fresh,
        "robot_state_age_ms": fsm_age,
        "fsm_message": fsm_message,

        # 急停
        "estop": {
            "active": estop_active,
            "reported": estop_reported,
        },

        # 运动管理器
        "motion_manager": {
            "state": lifecycle_state,
            "active": motion_manager_active,
            "motion_ready": fsm_fresh and motion_manager_active,
        },

        # 诊断
        "diagnostics": {
            "fresh": diag_fresh,
            "available": diag_payload is not None,
            "connected": diag_connected,
            "publisher_count": diag_publisher_count,
            "age_ms": diag_age,
            "data": diag_payload if diag_fresh else None,
        },

        "source_topics": {
            "fsm": "/xbot_state",
            "motion_manager": "/motion_manager/get_state",
            "diagnostics": "/diagnostics_agg",
        },
    }


class Plugin:
    def __init__(self, plugin_config, namespace, executor, client):
        self._topic = TOPIC.format(ns=namespace)
        self._node = None
        self._pub = None

        self._fsm_state = None
        self._fsm_message = None
        self._fsm_received_at_ms = None

        self._lifecycle_state = "unknown"
        self._lifecycle_client = None
        self._lifecycle_request_type = None
        self._lifecycle_pending = False

        self._diag_payload = None
        self._diag_received_at_ms = None

        if _HAS_ROS2 and executor is not None:
            try:
                self._node = Node(NODE)
                self._pub = self._node.create_publisher(String, self._topic, _VOLATILE_QOS)

                # /xbot_state: FSM state + estop (same RobotStatus msg)
                self._node.create_subscription(RobotStatus, "/xbot_state", self._on_robot, _LATCHED_QOS)

                # /diagnostics_agg
                self._node.create_subscription(DiagnosticArray, "/diagnostics_agg", self._on_diag, _DIAG_QOS)

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

    def _diag_publisher_count(self):
        if self._node is None:
            return None
        try:
            return len(self._node.get_publishers_info_by_topic("/diagnostics_agg"))
        except Exception:
            return None

    def _data(self):
        return build(self._fsm_state, self._fsm_message, self._fsm_received_at_ms,
                     self._lifecycle_state, self._diag_payload, self._diag_received_at_ms,
                     self._diag_publisher_count())

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
