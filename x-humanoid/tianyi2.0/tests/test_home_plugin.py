"""Regression coverage for Tianyi's Slamtec home-dock card."""

from __future__ import annotations

import sys
import types
from pathlib import Path


def _load_device_module():
    """Load device.py without requiring ROS 2 in the unit-test environment."""
    rclpy = types.ModuleType("rclpy")
    rclpy.node = types.ModuleType("rclpy.node")
    rclpy.qos = types.ModuleType("rclpy.qos")

    class Node:
        def __init__(self, *args, **kwargs):
            pass

    class QoSProfile:
        def __init__(self, **kwargs):
            pass

    class Enum:
        BEST_EFFORT = RELIABLE = KEEP_LAST = VOLATILE = 0

    rclpy.node.Node = Node
    rclpy.qos.QoSProfile = QoSProfile
    rclpy.qos.ReliabilityPolicy = Enum
    rclpy.qos.HistoryPolicy = Enum
    rclpy.qos.DurabilityPolicy = Enum
    sys.modules.update({"rclpy": rclpy, "rclpy.node": rclpy.node, "rclpy.qos": rclpy.qos})

    std_msgs = types.ModuleType("std_msgs")
    std_msgs.msg = types.ModuleType("std_msgs.msg")
    for name in ("String", "Bool", "UInt32MultiArray"):
        setattr(std_msgs.msg, name, type(name, (), {}))
    sys.modules.update({"std_msgs": std_msgs, "std_msgs.msg": std_msgs.msg})

    module = types.ModuleType("tianyi_device_test")
    source = (Path(__file__).parents[1] / "device.py").read_text(encoding="utf-8")
    exec(compile(source, "device.py", "exec"), module.__dict__)
    return module


class _Client:
    def __init__(self, status):
        self.status = status
        self.calls = []

    def get_nav_status(self):
        return self.status

    def get_pose(self):
        return {"x": 0, "y": 0}

    def go_home(self, **kwargs):
        self.calls.append(("go_home", kwargs))
        return {"action_id": 1}

    def register_home_dock(self, display_name):
        self.calls.append(("register_home_dock", display_name))
        return {"id": "dock-new", "pose": {"x": 3, "y": 4, "yaw": 0.5}}

    def get_home_docks(self):
        return {"raw": [{"id": "dock-a", "pose": {"x": 1, "y": 2, "yaw": 0}}]}

    def set_home_pose(self, pose):
        self.calls.append(("set_home_pose", pose))
        return {"ok": True}

    def cancel_action(self):
        self.calls.append(("cancel",))
        return {"ok": True}


def _plugin(status):
    module = _load_device_module()
    module.time.sleep = lambda _: None
    client = _Client(status)
    return module, module.HomePlugin({}, "", None, client), client


def test_home_schema_owns_go_home():
    module, home, _ = _plugin({"action_state": 1, "result": 0})
    tool = home.get_tool()
    home_actions = tool["inputSchema"]["properties"]["action"]["enum"]
    assert "go_home" in home_actions
    assert "go_home_no_dock" not in home_actions
    assert all(step in tool["description"] for step in ("register_dock", "list_docks", "set_dock", "go_home"))
    set_dock = home.get_tool()["inputSchema"]["x-action-params"]["set_dock"]
    assert "二者任选其一" in set_dock["description"]
    register_dock = home.get_tool()["inputSchema"]["x-action-params"]["register_dock"]
    assert "自动设为当前回桩目标" in register_dock["description"]
    assert "go_home" not in module.NavPlugin({}, "", types.SimpleNamespace(ctx_tianyi=None, executor_tianyi=types.SimpleNamespace(add_node=lambda _: None)), _Client({})).get_tool()["inputSchema"]["properties"]["action"]["enum"]


def test_set_dock_resolves_pose_from_dock_id():
    _, home, client = _plugin({"action_state": 1, "result": 0})
    result = home.dispatch("set_dock", {"dock_id": "dock-a"})
    assert result["pose"] == {"x": 1, "y": 2, "yaw": 0}
    assert client.calls == [("set_home_pose", {"x": 1, "y": 2, "yaw": 0})]


def test_register_dock_selects_new_dock():
    _, home, client = _plugin({"action_state": 1, "result": 0})
    result = home.dispatch("register_dock", {"display_name": "main_dock"})
    assert result["state"] == "registered_and_selected"
    assert result["pose"] == {"x": 3, "y": 4, "yaw": 0.5}
    assert client.calls == [
        ("register_home_dock", "main_dock"),
        ("set_home_pose", {"x": 3, "y": 4, "yaw": 0.5}),
    ]


def test_home_poll_only_completes_on_done_success():
    module, home, _ = _plugin({"action_state": 4, "result": 0})
    calls = []
    module._acp_notify = lambda *args: calls.append(args)
    home._active_poll = "a"
    home._poll_loop("a", "go_home", {})
    assert calls[0][1] == "completed"


def test_home_poll_does_not_complete_when_action_disappears():
    module, home, _ = _plugin({"action_state": -1})
    calls = []
    module._acp_notify = lambda *args: calls.append(args)
    home._MISSING_ACTION_TIMEOUT = -1
    home._active_poll = "a"
    home._poll_loop("a", "go_home", {})
    assert calls[0][1] == "error"
    assert calls[0][2]["error"] == "action_disappeared"
