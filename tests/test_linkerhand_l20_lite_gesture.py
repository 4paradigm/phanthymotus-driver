"""Offline checks for the separate gesture card; no serial device is opened."""

from __future__ import annotations

import sys
import threading
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
DRIVER = ROOT / "linkerhand" / "l20_lite"
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(DRIVER))

from common.vendor_runtime import DriverBundle, load_config
from gesture_device import ACTIONS, DEFAULT_POSES, GesturePlugin, NUMBER_POSES
from modbus_device import build_plugins


class FakeHand:
    def __init__(self, sides=None):
        self.config = {"side": "right", "serial_port": "/dev/fake"}
        self._bus_lock = threading.RLock()
        self.sides = sides or {"right": "/dev/fake"}

    def _discover(self):
        return dict(self.sides)


def config():
    return {
        "right_poses": DEFAULT_POSES,
        "left_poses": DEFAULT_POSES,
        "right_number_poses": NUMBER_POSES,
        "left_number_poses": NUMBER_POSES,
    }


def test_card_registered_without_changing_hand_schema():
    full = load_config(DRIVER / "config.yaml")
    plugins = build_plugins(full, "", None)
    tools = {item["name"]: item for item in DriverBundle(plugins).get_all_tools()}
    assert "hand" in tools and "hand_gesture" in tools
    assert "ok_gesture" not in tools["hand"]["inputSchema"]["properties"]["action"]["enum"]
    schema = tools["hand_gesture"]["inputSchema"]
    assert set(ACTIONS) <= set(schema["properties"]["action"]["enum"])
    assert set(ACTIONS) == set(schema["x-completion"]["actions"])
    assert tools["hand_gesture"]["multiInstance"] is True
    assert all(side in plugins[1]._calibration for side in ("left", "right"))
    assert schema["properties"]["start_num"]["description"] == "起始数字（0~9，0为握拳，默认 1）"
    assert schema["properties"]["end_num"]["description"] == "结束数字（0~9，0为握拳，默认 5；若小于起始数字则倒数）"
    assert schema["properties"]["hold_time_s"]["description"] == "手势停留展示时间（默认 1.0s）"


def test_motion_requires_confirmation_and_explicit_side_calibration():
    plugin = GesturePlugin(FakeHand(), config())
    for action in ACTIONS:
        with pytest.raises(ValueError, match="confirm_motion"):
            plugin.dispatch(action, {"instance_id": "one"})
    with pytest.raises(ValueError, match="torque"):
        plugin.dispatch("custom_grasp", {"confirm_motion": True, "torque": 256})
    uncalibrated = GesturePlugin(FakeHand({"left": "/dev/fake"}), {
        "right_poses": DEFAULT_POSES, "right_number_poses": NUMBER_POSES,
    })
    with pytest.raises(RuntimeError, match="no explicitly calibrated"):
        uncalibrated.dispatch("ok_gesture", {"confirm_motion": True})


def test_side_binding_is_read_only_and_rejects_duplicate():
    plugin = GesturePlugin(FakeHand({"left": "/dev/l", "right": "/dev/r"}), config())
    assert plugin.dispatch("config", {"instance_id": "one", "bound_side": "left"})["state"] == "configured"
    assert plugin.dispatch("info", {"instance_id": "one"})["serial_port"] == "/dev/l"
    with pytest.raises(ValueError, match="already bound"):
        plugin.dispatch("config", {"instance_id": "two", "bound_side": "left"})
    assert plugin.dispatch("config", {"instance_id": "two", "bound_side": "right"})["state"] == "configured"


def test_async_completion_uses_gesture_tool_and_colleague_pose(monkeypatch):
    plugin = GesturePlugin(FakeHand(), config())
    done = threading.Event()
    callbacks = []
    measured = []
    monkeypatch.setattr(plugin, "_connection", lambda *_: type("Connection", (), {
        "get": lambda self, side: type("Client", (), {
            "probe_positions": lambda self, side: None,
            "get_positions": lambda self: list(DEFAULT_POSES["ok_gesture"]),
            "set_speeds": lambda self, values: None,
            "set_torques": lambda self, values: None,
            "set_positions": lambda self, values: measured.append(list(values)),
        })(),
    })())
    monkeypatch.setattr(plugin, "_acp_callback", lambda *args: (callbacks.append(args), done.set()))
    monkeypatch.setattr(plugin, "_drive_and_wait", lambda client, target, speed, **kwargs:
                        (measured.append(list(target)), list(target))[1])
    accepted = plugin.dispatch("ok_gesture", {"instance_id": "one", "confirm_motion": True})
    assert accepted["state"] == "running"
    assert done.wait(3)
    assert callbacks[0][0] == accepted["action_id"]
    assert callbacks[0][1] == "completed"
    assert callbacks[0][2]["side"] == "right"
    assert measured[-1] == DEFAULT_POSES["ok_gesture"]


class ImmediateClient:
    def __init__(self):
        self.positions = list(NUMBER_POSES[0])
        self.commands = []
        self.torques = []

    def get_positions(self):
        return list(self.positions)

    def set_positions(self, values):
        self.positions = list(values)
        self.commands.append(list(values))

    def set_speeds(self, values):
        self.speeds = list(values)

    def set_torques(self, values):
        self.torques.append(list(values))


def test_count_and_grasp_preserve_colleague_targets_without_hardware(monkeypatch):
    plugin = GesturePlugin(FakeHand(), config())
    monkeypatch.setattr(plugin, "_wait", lambda seconds: None)
    client = ImmediateClient()
    counted = plugin._count(client, plugin._calibration["right"], {
        "speed": 85, "start_num": 2, "end_num": 0, "hold_time_s": 0.2,
    })
    assert counted["counted"] == [2, 1, 0]
    assert client.commands[-1] == NUMBER_POSES[0]
    client.positions = list(NUMBER_POSES[0])
    grasped = plugin._grasp(client, plugin._calibration["left"], {
        "speed": 45, "torque": 30,
    })
    assert DEFAULT_POSES["grasp_prep"] in client.commands
    assert DEFAULT_POSES["grasp_target"] in client.commands
    assert grasped["torque"] == 30
    assert grasped["contact_method"] == "position_stall_only"
    assert client.speeds[0] < client.speeds[2]  # Prototype's thumb-speed scaling.
