"""Guarded Q5 pre-defined action card.

Only vendor-documented actions are exposed.  This card never performs the
dynamic_launch/ready/activate sequence: an on-site operator must complete and
verify that sequence before explicitly unlocking this card.
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

CARD = "simple_action"
TYPE = "actuator"
TOPIC = "/simple_actions"
NODE = "q5_simple_action"
DESC = "Q5 预定义动作：标准初始姿势和抬臂"
DEFAULT_ACTIONS = {
    "zero": 4.0,
    "initpose_handsdown": 4.0,
    "lift_up": 4.0,
}
ACTION_LABELS = {
    "zero": "零位复位",
    "initpose_handsdown": "垂手初始姿势",
    "lift_up": "抬臂动作",
}


def _failure(code: str, message: str, **details) -> dict:
    return {"ok": False, "code": code, "message": message, "details": details}


class Plugin:
    def __init__(self, plugin_config, namespace, executor, client):
        self._client = client
        configured = plugin_config.get("actions", DEFAULT_ACTIONS)
        self._actions = {
            str(name): float(duration)
            for name, duration in configured.items()
            if str(name) in DEFAULT_ACTIONS and isinstance(duration, (int, float)) and duration > 0
        }
        if not self._actions:
            raise ValueError("simple_action requires at least one allowed vendor action")
        self._node = None
        self._action_client = None
        self._lock = threading.Lock()
        self._goal_handle = None
        self._goal_future = None
        self._result_future = None
        self._cancel_requested = False
        self._status = {"state": "idle", "action_name": None, "updated_at_ms": int(time.time() * 1000)}

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
                    "action": {"type": "string", "enum": ["start", "run", "cancel", "info"], "oneOf": [
                        {"const": "start", "title": "检查动作条件"},
                        {"const": "run", "title": "执行预设动作"},
                        {"const": "cancel", "title": "取消当前动作"},
                        {"const": "info", "title": "查看状态"},
                    ], "description": "仅限白名单动作。"},
                    "action_name": {"type": "string", "title": "预设动作", "enum": sorted(self._actions), "oneOf": [
                        {"const": name, "title": ACTION_LABELS.get(name, name)}
                        for name in sorted(self._actions)
                    ], "description": "时长由厂商定义。"},
                },
                "required": ["action"],
                "additionalProperties": False,
                "x-action-params": {
                    "start": {"params": [], "description": "检查动作服务和机器人状态。"},
                    "run": {"params": ["action_name"], "description": "执行一个白名单预设动作。"},
                    "cancel": {"params": [], "description": "请求取消正在执行的动作。"},
                    "info": {"params": [], "description": "查看动作执行状态和安全条件。"},
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
            "allowed_actions": dict(self._actions),
        }

    def _check_run(self, args):
        action_name = args.get("action_name")
        if action_name not in self._actions:
            return _failure("ACTION_NOT_ALLOWED", "Requested action is not in the approved Q5 action whitelist")
        status = self._safety()
        if not status["action_server_available"]:
            return _failure("ROS_UNAVAILABLE", "Q5 SimpleActions client is unavailable", status=status)
        if status["lifecycle_state"] != "active":
            return _failure("LIFECYCLE_NOT_ACTIVE", "Q5 motion_manager must be active before actions", status=status)
        q5_ready, q5_status = q5_is_control_ready(self._client)
        if not q5_ready:
            return _failure("Q5_FSM_NOT_READY", "Q5 /xbot_state must be fresh and READY or ACTIVE before actions",
                            status={**status, "q5_fsm": q5_status})
        if not status["joint_state_fresh"]:
            return _failure("JOINT_STATE_UNAVAILABLE", "Refusing action without fresh /joint_states")
        if not self._action_client.server_is_ready():
            return _failure("ACTION_SERVER_UNAVAILABLE", "Q5 /simple_actions server is not ready", status=status)
        return action_name

    def _set_status(self, state, action_name=None, **extra):
        self._status = {
            "state": state,
            "action_name": action_name,
            "updated_at_ms": int(time.time() * 1000),
            **extra,
        }

    def _on_goal_response(self, future):
        try:
            goal_handle = future.result()
            with self._lock:
                self._goal_future = None
                if goal_handle is None or not goal_handle.accepted:
                    self._set_status("rejected", self._status.get("action_name"))
                    return
                self._goal_handle = goal_handle
                self._set_status("executing", self._status.get("action_name"))
                self._result_future = goal_handle.get_result_async()
                self._result_future.add_done_callback(self._on_result)
                if self._cancel_requested:
                    goal_handle.cancel_goal_async()
        except Exception as e:
            with self._lock:
                self._goal_future = None
                self._set_status("error", self._status.get("action_name"), error=str(e))

    def _on_result(self, future):
        try:
            response = future.result()
            with self._lock:
                self._set_status(
                    "completed",
                    self._status.get("action_name"),
                    action_status=getattr(response, "status", None),
                    cancelled=self._cancel_requested,
                )
                self._goal_handle = None
                self._result_future = None
                self._cancel_requested = False
        except Exception as e:
            with self._lock:
                self._set_status("error", self._status.get("action_name"), error=str(e))
                self._goal_handle = None
                self._result_future = None
                self._cancel_requested = False

    def _cancel(self, reason: str) -> dict:
        with self._lock:
            goal_handle = self._goal_handle
            pending = self._goal_future is not None
            if goal_handle is None and not pending:
                return {"ok": True, "state": "idle", "reason": reason}
            self._cancel_requested = True
            self._set_status("cancelling", self._status.get("action_name"), reason=reason)
            if goal_handle is not None:
                goal_handle.cancel_goal_async()
            return {"ok": True, "state": "cancelling", "reason": reason}

    def start(self):
        return {"state": "ready" if self._action_client is not None else "unavailable"}

    def stop(self):
        self._cancel("driver_shutdown")

    def dispatch(self, action, args):
        if action == "start":
            return {**self.start(), "safety": self._safety()}
        if action in ("cancel", "stop"):
            return self._cancel(action)
        if action == "info":
            with self._lock:
                status = dict(self._status)
            return {"ok": True, "status": status, "safety": self._safety()}
        if action != "run":
            return None

        action_name = self._check_run(args)
        if isinstance(action_name, dict):
            return action_name
        with self._lock:
            if self._goal_future is not None or self._goal_handle is not None:
                return _failure("ACTION_IN_PROGRESS", "A Q5 action is already active; cancel it before starting another")
            goal = SimpleActions.Goal()
            goal.action_name = action_name
            goal.time_cost = self._actions[action_name]
            self._cancel_requested = False
            self._set_status("submitting", action_name, time_cost_s=goal.time_cost)
            self._goal_future = self._action_client.send_goal_async(goal)
            self._goal_future.add_done_callback(self._on_goal_response)
        return {"ok": True, "state": "submitting", "action_name": action_name,
                "time_cost_s": self._actions[action_name], "cancellable": True}


def make_plugin(plugin_config, namespace, executor, client):
    return Plugin(plugin_config, namespace, executor, client)
