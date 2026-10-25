"""Gesture actions choose a physical hand without persistent card bindings."""

import sys
import threading
import time
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "linkerhand" / "l20"))

import gesture_device
from gesture_device import ACTIONS, GesturePlugin, NUMBER_POSES_20
from hand_device import claim_motion, release_motion


class Client:
    def __init__(self):
        self.positions = list(NUMBER_POSES_20[5])
        self.currents = []
        self.speeds = []

    def set_joint_positions(self, positions):
        self.positions = list(positions)

    def get_positions_vector(self):
        return list(self.positions)

    def set_currents(self, currents):
        self.currents.append(list(currents))

    def set_speeds(self, speeds):
        self.speeds.append(list(speeds))


class Controller:
    def __init__(self, side, channel):
        self.side = side
        self.channel = channel
        self._motion_lock = threading.RLock()
        self.client = Client()

    def get(self):
        return self.client

    def release_torque(self):
        self.client.set_currents([0] * 5)


class Hand:
    def __init__(self):
        self.config = {"can_channel": "can0"}
        self.controllers = {}
        self.calls = []
        self.results = []

    def get_controller_for_side(self, side, channel):
        self.calls.append((side, channel))
        return self.controllers.setdefault((side, channel), Controller(side, channel))

    def _acp_callback(self, action_id, status, result):
        self.results.append((action_id, status, result))


@pytest.fixture
def plugin(monkeypatch):
    monkeypatch.setattr(gesture_device, "verify_hand_connection", lambda _: None)
    return GesturePlugin(Hand(), {"motion_timeout_s": 0.01, "poll_interval_s": 0.001})


def wait_idle(plugin):
    deadline = time.monotonic() + 2
    while plugin._active and time.monotonic() < deadline:
        time.sleep(0.001)
    assert not plugin._active


def test_schema_requires_explicit_side_without_gear_binding(plugin):
    definition = plugin.get_tool()
    schema = definition["inputSchema"]
    assert "bound_side" not in definition.get("configSchema", {}).get("properties", {})
    assert schema["properties"]["bound_side"]["default"] == ""
    assert "x-hooks" not in schema
    for action in (*ACTIONS, "info", "stop"):
        assert "bound_side" in schema["x-action-params"][action]["params"]


@pytest.mark.parametrize("action", ["start", "config"])
def test_lifecycle_needs_neither_side_nor_hardware(plugin, action):
    assert plugin.dispatch(action, {"instance_id": "new-card"})["state"] in ("ready", "configured")
    assert not plugin.hand.calls
    assert not hasattr(plugin, "_bindings")


@pytest.mark.parametrize("side", [None, "", "LEFT", "other", False, 1])
def test_missing_or_invalid_side_does_not_open_hardware(plugin, side):
    with pytest.raises(ValueError, match="bound_side"):
        plugin.dispatch("number", {"bound_side": side, "number": 5, "confirm_motion": True})
    assert not plugin.hand.calls


@pytest.mark.parametrize("side", ["left", "right"])
@pytest.mark.parametrize("ratio,speed", [("1.0x", 85), ("1.5x", 128), ("2.0x", 170), ("2.5x", 212), ("3.0x", 255)])
def test_motion_without_instance_id_uses_requested_side(plugin, side, ratio, speed):
    result = plugin.dispatch("number", {"bound_side": side, "number": 5,
                                        "speed_ratio": ratio, "confirm_motion": True})
    wait_idle(plugin)
    assert result["bound_side"] == side
    assert plugin.hand.calls == [(side, "can0")]
    ctrl = plugin.hand.controllers[(side, "can0")]
    assert ctrl.client.speeds == [[speed] * 5]
    assert ctrl.client.currents[-1] == [0] * 5
    assert not vars(ctrl).get("_command_owner")


def test_prior_config_never_supplies_missing_side(plugin):
    plugin.dispatch("config", {"instance_id": "card", "bound_side": "left"})
    with pytest.raises(ValueError, match="bound_side"):
        plugin.dispatch("number", {"instance_id": "card", "number": 5, "confirm_motion": True})
    assert not plugin.hand.calls


def test_shared_can_configuration_is_read_each_request(plugin):
    plugin.hand.config["can_channel"] = "can1"
    plugin.dispatch("number", {"bound_side": "right", "number": 5, "confirm_motion": True})
    wait_idle(plugin)
    assert plugin.hand.calls == [("right", "can1")]


def test_two_hands_run_independently_and_stop_targets_one(plugin, monkeypatch):
    entered = {side: threading.Event() for side in ("left", "right")}
    finish = threading.Event()

    def blocked_pose(client, target, speed):
        side = next(side for (side, _), ctrl in plugin.hand.controllers.items() if ctrl.client is client)
        entered[side].set()
        while not finish.is_set():
            plugin._wait(0.001)
        return target

    monkeypatch.setattr(plugin, "_pose", blocked_pose)
    try:
        for side in ("left", "right"):
            plugin.dispatch("ok_gesture", {"bound_side": side, "confirm_motion": True})
            assert entered[side].wait(1)
        with pytest.raises(RuntimeError):
            plugin.dispatch("ok_gesture", {"bound_side": "left", "confirm_motion": True})
        assert plugin.dispatch("stop", {"instance_id": "unrelated-card"})["state"] == "idle"
        assert not any(active["stop_event"].is_set() for active in plugin._active.values())
        stopped = plugin.dispatch("stop", {"bound_side": "left"})
        assert stopped["state"] == "idle" and stopped["cancel_requested"]
        assert stopped["target_resolved"]
        assert not plugin._active[("right", "can0")]["stop_event"].is_set()
    finally:
        finish.set()
        wait_idle(plugin)


def test_id_only_stop_cancels_only_source_that_supplied_id(plugin, monkeypatch):
    entered = threading.Event()

    def blocked_pose(client, target, speed):
        entered.set()
        while True:
            plugin._wait(0.001)

    monkeypatch.setattr(plugin, "_pose", blocked_pose)
    plugin.dispatch("ok_gesture", {"bound_side": "left", "instance_id": "owner", "confirm_motion": True})
    assert entered.wait(1)
    plugin.dispatch("stop", {"instance_id": "other"})
    assert not plugin._active[("left", "can0")]["stop_event"].is_set()
    plugin.dispatch("stop", {"instance_id": "owner"})
    wait_idle(plugin)
    assert plugin._last_result["status"] == "cancelled"


def test_old_card_cleanup_cannot_unload_newer_device_command(plugin):
    plugin.dispatch("number", {"bound_side": "left", "instance_id": "old", "number": 5,
                               "hold_torque": 80, "confirm_motion": True})
    wait_idle(plugin)
    ctrl = plugin.hand.controllers[("left", "can0")]
    newer_owner = ("other-tool", "new-command")
    claim_motion(ctrl, newer_owner)
    release_motion(ctrl, newer_owner)
    before = list(ctrl.client.currents)
    plugin.dispatch("stop", {"instance_id": "old"})
    assert ctrl.client.currents == before


def test_stop_from_another_card_can_release_selected_gesture_hold(plugin):
    plugin.dispatch("number", {"bound_side": "left", "instance_id": "old", "number": 5,
                               "hold_torque": 80, "confirm_motion": True})
    wait_idle(plugin)
    ctrl = plugin.hand.controllers[("left", "can0")]
    plugin.dispatch("stop", {"bound_side": "left", "instance_id": "replacement"})
    assert ctrl.client.currents[-1] == [0] * 5


@pytest.mark.parametrize("failure", ["verify", "thread_start"])
def test_preworker_failure_preserves_existing_hold_for_explicit_stop(plugin, monkeypatch, failure):
    plugin.dispatch("number", {"bound_side": "left", "instance_id": "old", "number": 5,
                               "hold_torque": 80, "confirm_motion": True})
    wait_idle(plugin)
    target = ("left", "can0")
    previous_hold = plugin._held[target]
    ctrl = plugin.hand.controllers[target]
    before = list(ctrl.client.currents)

    def fail(*_):
        raise RuntimeError("simulated preworker failure")

    if failure == "verify":
        monkeypatch.setattr(gesture_device, "verify_hand_connection", fail)
    else:
        monkeypatch.setattr(threading.Thread, "start", fail)

    with pytest.raises(RuntimeError, match="simulated preworker failure"):
        plugin.dispatch("number", {"bound_side": "left", "number": 5, "confirm_motion": True})
    assert not plugin._active
    assert plugin._held[target] is previous_hold
    assert ctrl.client.currents == before
    assert vars(ctrl).get("_last_command_owner") == previous_hold["owner"]
    assert not vars(ctrl).get("_command_owner")
    plugin.dispatch("stop", {"bound_side": "left"})
    assert ctrl.client.currents[-1] == [0] * 5
    assert ctrl._holding_torque == 0
