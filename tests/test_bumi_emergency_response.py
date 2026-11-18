"""ROS-free contract and lifecycle tests for Bumi emergency_response."""

from __future__ import annotations

import importlib
import json
import sys
import types
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[1]
BUMI = ROOT / "noetix" / "bumi"


class FakePublisher:
    def __init__(self): self.messages = []
    def publish(self, message): self.messages.append(json.loads(message.data))


class FakeNode:
    def __init__(self, _name): self.publisher = FakePublisher()
    def create_publisher(self, _message_type, _topic, _qos): return self.publisher


class FakeExecutor:
    def __init__(self): self.nodes = []
    def add_node(self, node): self.nodes.append(node)


class FakeString:
    def __init__(self): self.data = ""


class FakeLoco:
    def __init__(self): self.calls = []
    def dispatch(self, action, args):
        self.calls.append((action, args)); return {"state": "stopped"}


class FakeCapture:
    def __init__(self, ok=True): self.ok, self.calls = ok, []
    def dispatch(self, action, args):
        self.calls.append((action, args))
        if not self.ok: return {"ok": False, "code": "CAPTURE_FAILED"}
        return {"ok": True, "file_path": "/data/photo.jpg",
                "captured_at": "2026-11-18T12:00:00+08:00"}


@pytest.fixture
def module(monkeypatch):
    rclpy, rclpy_node = types.ModuleType("rclpy"), types.ModuleType("rclpy.node")
    rclpy_node.Node = FakeNode
    std_msgs, std_msgs_msg = types.ModuleType("std_msgs"), types.ModuleType("std_msgs.msg")
    std_msgs_msg.String = FakeString
    for name, value in (("rclpy", rclpy), ("rclpy.node", rclpy_node),
                        ("std_msgs", std_msgs), ("std_msgs.msg", std_msgs_msg)):
        monkeypatch.setitem(sys.modules, name, value)
    monkeypatch.syspath_prepend(str(BUMI)); sys.modules.pop("emergency_response", None)
    loaded = importlib.import_module("emergency_response")
    yield loaded
    sys.modules.pop("emergency_response", None)


def make_plugin(module, tmp_path, *, loco=None, capture=None):
    executor = FakeExecutor()
    plugin = module.EmergencyResponsePlugin(
        {"output_dir": str(tmp_path), "max_records": 20},
        "bumi_test", executor, loco, capture)
    assert executor.nodes == [plugin._node]
    return plugin


def engage(plugin):
    result = plugin.dispatch("engage", {"reason": "人工报告现场持续争执",
                                        "source": "manual", "location": "一楼综合窗口"})
    assert result["state"] == "engaged"
    return result


def test_tool_is_bumi_primitive_not_incident_workflow(module, tmp_path):
    tool = make_plugin(module, tmp_path).get_tool()
    actions = set(tool["inputSchema"]["properties"]["action"]["enum"])
    assert tool["name"] == "emergency_response" and tool["type"] == "actuator"
    assert actions == {"engage", "snapshot", "release", "info", "history"}
    assert "呼叫安保" in tool["description"]
    assert tool["topic_out"][0]["format"] == "data/json"


def test_engage_stops_motion_captures_and_publishes(module, tmp_path):
    loco, capture = FakeLoco(), FakeCapture()
    plugin = make_plugin(module, tmp_path, loco=loco, capture=capture)
    result = engage(plugin)
    assert loco.calls == [("stop_move", {})]
    assert capture.calls == [("capture_photo", {})]
    assert result["handoff"]["latest_photo"] == "/data/photo.jpg"
    assert result["record"]["schema"] == "motus.bumi-emergency-handoff/1"
    assert plugin._node.publisher.messages == [result["record"]]


def test_one_active_handoff_snapshot_and_release(module, tmp_path):
    plugin = make_plugin(module, tmp_path, capture=FakeCapture())
    handoff_id = engage(plugin)["handoff"]["handoff_id"]
    assert plugin.dispatch("engage", {"reason": "duplicate", "source": "system"})[
        "error"] == "handoff_already_active"
    assert plugin.dispatch("snapshot", {"handoff_id": handoff_id})["photo"]["ok"] is True
    assert plugin.dispatch("release", {"handoff_id": handoff_id, "confirm": False})[
        "error"] == "confirm_true_required"
    released = plugin.dispatch("release", {"handoff_id": handoff_id, "confirm": True})
    assert released["state"] == "released" and "does not start" in released["note"]
    assert plugin.dispatch("info", {})["state"] == "ready"


def test_camera_or_motion_failure_does_not_hide_handoff(module, tmp_path):
    result = engage(make_plugin(module, tmp_path))
    assert result["motion_stop"]["state"] == "unavailable"
    assert result["photo"]["code"] == "CAPTURE_UNAVAILABLE"
    assert result["handoff"]["latest_photo"] is None


def test_history_survives_restart_and_torn_line(module, tmp_path):
    plugin = make_plugin(module, tmp_path)
    handoff_id = engage(plugin)["handoff"]["handoff_id"]

    active_after_restart = make_plugin(module, tmp_path).dispatch("info", {})
    assert active_after_restart["state"] == "engaged"
    assert active_after_restart["active"]["handoff_id"] == handoff_id

    plugin.dispatch("release", {"handoff_id": handoff_id, "confirm": True})
    with plugin._journal.path.open("a", encoding="utf-8") as output: output.write("{torn")
    history = make_plugin(module, tmp_path).dispatch("history", {"limit": 10})["records"]
    assert [item["action"] for item in history] == ["release", "engage"]


def test_invalid_source_and_handoff_id_are_rejected(module, tmp_path):
    plugin = make_plugin(module, tmp_path)
    assert plugin.dispatch("engage", {"reason": "x", "source": "unknown"})[
        "state"] == "error"
    assert plugin.dispatch("snapshot", {"handoff_id": "bad"})["state"] == "error"
