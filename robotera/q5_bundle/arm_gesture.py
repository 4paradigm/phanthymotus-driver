"""Guarded Q5 semantic arm-gesture card.

Only maps named gestures to vendor-approved SimpleActions.  It does not accept
arbitrary joint targets and does not move when loaded, queried, or started.
"""

from __future__ import annotations

import threading
import time

from control_contract import q5_active_status, q5_is_control_ready

try:
    from rclpy.action import ActionClient
    from rclpy.node import Node
    from xbot_common_interfaces.action import SimpleActions

    _HAS_ROS2 = True
except Exception:
    _HAS_ROS2 = False

CARD = "arm_gesture"
TYPE = "actuator"
TOPIC = "/simple_actions"
NODE = "q5_arm_gesture"
DESC = "Q5 手臂语义姿势：归零、双手下垂、抬臂"
GESTURES = {
    "home": {"vendor_action": "zero", "time_cost_s": 4.0, "label": "归零姿势"},
    "hands_down": {"vendor_action": "initpose_handsdown", "time_cost_s": 4.0, "label": "双手自然下垂"},
    "lift_up": {"vendor_action": "lift_up", "time_cost_s": 4.0, "label": "抬臂"},
}


def _failure(code: str, message: str, **details) -> dict:
    return {"ok": False, "code": code, "message": message, "details": details}


class Plugin:
    def __init__(self, plugin_config, namespace, executor, client):
        del namespace
        self._client = client
        self._node = None
        self._action_client = None
        self._lock = threading.Lock()
        self._goal_handle = None
        self._goal_future = None
        self._result_future = None
        self._cancel_requested = False
        self._status = {"state": "idle", "gesture": None, "updated_at_ms": int(time.time() * 1000)}

        if _HAS_ROS2 and executor is not None:
            try:
                self._node = Node(NODE)
                self._action_client = ActionClient(self._node, SimpleActions, TOPIC)
                executor.add_node(self._node)
            except Exception as e:
                print(f"[{CARD}] ROS2 action client unavailable: {e}", flush=True)
                self._node = None
                self._action_client = None

    def get_tool(self):
        return {
            "name": CARD,
            "type": TYPE,
            "multiInstance": False,
            "description": DESC,
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {"type": "string", "enum": ["start", "run", "cancel", "info"]},
                    "gesture": {
                        "type": "string",
                        "enum": sorted(GESTURES),
                        "oneOf": [
                            {"const": name, "title": spec["label"]}
                            for name, spec in sorted(GESTURES.items())
                        ],
                    },
                },
                "required": ["action"],
                "additionalProperties": False,
                "x-action-params": {
                    "start": {"params": [], "description": "检查机器人与动作服务状态。"},
                    "run": {"params": ["gesture"], "description": "执行厂商白名单语义姿势。"},
                    "cancel": {"params": [], "description": "取消正在执行的姿势。"},
                    "info": {"params": [], "description": "查看当前状态和安全条件。"},
                },
            },
        }

    def _safety(self) -> dict:
        return {
            "action_server_available": self._action_client is not None,
            "lifecycle_state": self._client.get_lifecycle_state(),
            "joint_state_fresh": bool(self._client.snapshot().get("fresh", False)),
            "q5_fsm": q5_active_status(self._client),
            "topic": TOPIC,
            "allowed_gestures": {name: spec["label"] for name, spec in GESTURES.items()},
        }

    def _check_run(self, args):
        gesture = args.get("gesture")
        if gesture not in GESTURES:
            return _failure("GESTURE_NOT_ALLOWED", "Requested gesture is not in the approved Q5 gesture whitelist")
        safety = self._safety()
        if not safety["action_server_available"]:
            return _failure("ROS_UNAVAILABLE", "Q5 SimpleActions client is unavailable", safety=safety)
        if safety["lifecycle_state"] != "active":
            return _failure("LIFECYCLE_NOT_ACTIVE", "Q5 motion_manager must be active before gestures", safety=safety)
        ready, q5_fsm = q5_is_control_ready(self._client)
        if not ready:
            return _failure("Q5_FSM_NOT_READY", "Q5 /xbot_state must be fresh and READY or ACTIVE before gestures",
                            safety={**safety, "q5_fsm": q5_fsm})
        if not safety["joint_state_fresh"]:
            return _failure("JOINT_STATE_UNAVAILABLE", "Refusing gesture without fresh /joint_states", safety=safety)
        if not self._action_client.server_is_ready():
            return _failure("ACTION_SERVER_UNAVAILABLE", "Q5 /simple_actions server is not ready", safety=safety)
        return gesture

    def _set_status(self, state, gesture=None, **extra):
        self._status = {
            "state": state,
            "gesture": gesture,
            "updated_at_ms": int(time.time() * 1000),
            **extra,
        }

    def _on_goal_response(self, future):
        try:
            goal_handle = future.result()
            with self._lock:
                self._goal_future = None
                if goal_handle is None or not goal_handle.accepted:
                    self._set_status("rejected", self._status.get("gesture"))
                    return
                self._goal_handle = goal_handle
                self._set_status("executing", self._status.get("gesture"))
                self._result_future = goal_handle.get_result_async()
                self._result_future.add_done_callback(self._on_result)
                if self._cancel_requested:
                    goal_handle.cancel_goal_async()
        except Exception as e:
            with self._lock:
                self._goal_future = None
                self._set_status("error", self._status.get("gesture"), error=str(e))

    def _on_result(self, future):
        try:
            response = future.result()
            with self._lock:
                self._set_status("completed", self._status.get("gesture"),
                                 action_status=getattr(response, "status", None),
                                 cancelled=self._cancel_requested)
                self._goal_handle = None
                self._result_future = None
                self._cancel_requested = False
        except Exception as e:
            with self._lock:
                self._set_status("error", self._status.get("gesture"), error=str(e))
                self._goal_handle = None
                self._result_future = None
                self._cancel_requested = False

    def _cancel(self, reason: str) -> dict:
        with self._lock:
            goal_handle = self._goal_handle
            if goal_handle is None and self._goal_future is None:
                return {"ok": True, "state": "idle", "reason": reason}
            self._cancel_requested = True
            self._set_status("cancelling", self._status.get("gesture"), reason=reason)
            if goal_handle is not None:
                goal_handle.cancel_goal_async()
            return {"ok": True, "state": "cancelling", "reason": reason}

    def dispatch(self, action, args):
        if action == "start":
            return {"state": "ready" if self._action_client is not None else "unavailable", "safety": self._safety()}
        if action in ("cancel", "stop"):
            return self._cancel(action)
        if action == "info":
            with self._lock:
                status = dict(self._status)
            return {"ok": True, "status": status, "safety": self._safety()}
        if action != "run":
            return None

        gesture = self._check_run(args)
        if isinstance(gesture, dict):
            return gesture
        with self._lock:
            if self._goal_future is not None or self._goal_handle is not None:
                return _failure("ACTION_IN_PROGRESS", "A Q5 gesture is already active; cancel it before starting another")
            spec = GESTURES[gesture]
            goal = SimpleActions.Goal()
            goal.action_name = spec["vendor_action"]
            goal.time_cost = spec["time_cost_s"]
            self._cancel_requested = False
            self._set_status("submitting", gesture, vendor_action=goal.action_name, time_cost_s=goal.time_cost)
            self._goal_future = self._action_client.send_goal_async(goal)
            self._goal_future.add_done_callback(self._on_goal_response)
        return {"ok": True, "state": "submitting", "gesture": gesture,
                "vendor_action": spec["vendor_action"], "time_cost_s": spec["time_cost_s"], "cancellable": True}


def make_plugin(plugin_config, namespace, executor, client):
    return Plugin(plugin_config, namespace, executor, client)
