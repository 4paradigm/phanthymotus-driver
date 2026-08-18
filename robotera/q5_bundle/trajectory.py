"""Q5 predefined-trajectory card — zero and lift-up positions.

Both trajectories go through the vendor ``/simple_actions`` action server,
matching the SDK ``TrajectoryController.set_zero_position`` and
``set_lift_up_position`` (which send ``action_name`` "zero"/"lift_up").
"""

from __future__ import annotations

import time

from rclpy.action import ActionClient
from rclpy.node import Node
from xbot_common_interfaces.action import SimpleActions

from control_contract import q5_active_status

CARD = "trajectory"
TYPE = "actuator"
DESC = "Q5 预设轨迹：零位复位、抬升位（走 vendor /simple_actions）"

TRAJECTORIES = ("zero", "lift_up")
TRAJECTORY_LABELS = {"zero": "零位复位", "lift_up": "抬升位"}


def _failure(code, message, **details):
    return {"ok": False, "code": code, "message": message, "details": details}


class Plugin:
    def __init__(self, plugin_config, namespace, executor, client):
        del plugin_config, namespace
        self._client = client
        self._node = Node("q5_trajectory")
        executor.add_node(self._node)
        self._actions = ActionClient(self._node, SimpleActions, "/simple_actions")

    def get_tool(self):
        return {"name": CARD, "type": TYPE, "multiInstance": False, "description": DESC,
                "inputSchema": {"type": "object", "properties": {
                    "action": {"type": "string", "enum": ["start", *TRAJECTORIES, "info"], "oneOf": [
                        {"const": "start", "title": "检查轨迹服务"},
                        *[{"const": name, "title": TRAJECTORY_LABELS[name]} for name in TRAJECTORIES],
                        {"const": "info", "title": "查看状态"},
                    ]},
                    "duration": {"type": "number", "title": "执行时长 (s)",
                                 "minimum": 0.5, "maximum": 30.0, "default": 4.0,
                                 "description": "轨迹执行时长，默认 4s"},
                }, "required": ["action"], "additionalProperties": False,
                "x-action-params": {
                    "start": {"params": [], "description": "检查 /simple_actions action server 是否可用。"},
                    "zero": {"params": ["duration"], "description": "所有关节回到零位（复位）。"},
                    "lift_up": {"params": ["duration"], "description": "机器人抬升到安全抬升位。"},
                    "info": {"params": [], "description": "查看轨迹服务状态与安全条件。"},
                }}}

    @staticmethod
    def _wait_future(future, timeout):
        deadline = time.monotonic() + timeout
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.02)
        return future.result() if future.done() else None

    def _status(self):
        return {
            "q5_fsm": q5_active_status(self._client),
            "simple_actions_available": self._actions.server_is_ready(),
        }

    def _run(self, name, duration):
        if not self._actions.wait_for_server(timeout_sec=5.0):
            return _failure("SIMPLE_ACTIONS_UNAVAILABLE", "Q5 /simple_actions is unavailable")
        goal = SimpleActions.Goal()
        goal.action_name, goal.time_cost = name, float(duration)
        handle = self._wait_future(self._actions.send_goal_async(goal), 8.0)
        if not handle or not handle.accepted:
            return _failure("GOAL_REJECTED", f"Q5 trajectory {name} goal rejected")
        result = self._wait_future(handle.get_result_async(), float(duration) + 10.0)
        message = getattr(result.result, "message", "timeout") if result else "timeout"
        success = bool(result and getattr(result.result, "result", 2) == 0)
        if success:
            return {"ok": True, "trajectory": name, "duration_s": float(duration), "message": message}
        return _failure("TRAJECTORY_FAILED", f"Q5 trajectory {name} failed", message=message)

    def start(self):
        return {"state": "ready", "status": self._status()}

    def stop(self):
        pass

    def dispatch(self, action, args):
        if action in ("start", "info"):
            return {"ok": True, "state": "ready", "status": self._status()}
        if action in TRAJECTORIES:
            duration = args.get("duration", 4.0)
            try:
                duration = float(duration)
            except (TypeError, ValueError):
                return _failure("INVALID_ARGUMENT", "duration must be numeric")
            if not 0.5 <= duration <= 30.0:
                return _failure("INVALID_ARGUMENT", "duration must be in [0.5, 30]")
            return self._run(action, duration)
        return None


def make_plugin(plugin_config, namespace, executor, client):
    return Plugin(plugin_config, namespace, executor, client)
