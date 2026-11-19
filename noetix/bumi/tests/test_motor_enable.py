"""Standalone enable-only START semantics, cancellation and concurrent calls."""

import threading
from pathlib import Path

import pytest
import yaml

import importlib.util

from test_loco_destination import _load_device


class Controller:
    def __init__(self, mode=30, next_mode=0):
        self.mode = mode
        self.next_mode = next_mode
        self.commands = []

    def get_mode(self):
        return self.mode

    def publish_cmd(self, x, y, z, command, index):
        self.commands.append((x, y, z, command, index))
        if command == "START":
            self.mode = self.next_mode


@pytest.fixture
def card(monkeypatch):
    path = Path(__file__).resolve().parents[1] / "motor_enable.py"
    spec = importlib.util.spec_from_file_location("bumi_motor_enable_test", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setattr(module, "_get_control_cmd", lambda name: name)
    monkeypatch.setattr(module, "_get_default_cmd", lambda: "DEFAULT")
    ctrl = Controller()
    device, _ = _load_device(monkeypatch)
    monkeypatch.setattr(device, "_get_control_cmd", lambda name: name)
    monkeypatch.setattr(device, "_get_default_cmd", lambda: "DEFAULT")
    owner = device.LocoPlugin({}, "test", None, ctrl)
    plugin = module.MotorEnablePlugin({}, "test", None, ctrl, chassis_owner=owner)
    return module, plugin, ctrl


def call(plugin, action="enable", **args):
    return plugin.dispatch(action, {"_tool_name": "motor_enable", **args})


def test_enable_once_and_repeat_does_not_disable(card):
    _, plugin, ctrl = card
    result = call(plugin)
    assert result["state"] == "completed"
    assert result["confirmed"] is True
    assert result["workmode"] == 0
    assert ctrl.commands == [(0, 0, 0, "START", 0), (0, 0, 0, "DEFAULT", 0)]
    assert call(plugin)["command_sent"] is False
    assert len(ctrl.commands) == 2


@pytest.mark.parametrize("mode", [1, 2, 3, 5, 11, 23, 26, 27, 28, 99])
def test_other_modes_never_receive_start(card, mode):
    _, plugin, ctrl = card
    ctrl.mode = mode
    result = call(plugin)
    assert result["state"] == "error"
    assert result["command_sent"] is False
    assert result["workmode"] == mode
    assert ctrl.commands == []


def test_lifecycle_and_info_are_read_only(card):
    _, plugin, ctrl = card
    assert call(plugin, "start")["state"] == "ready"
    assert call(plugin, "info")["workmode"] == 30
    assert call(plugin, "stop")["state"] == "idle"
    assert call(plugin, "unsupported")["state"] == "error"
    assert ctrl.commands == []


def test_missing_or_failed_feedback_never_sends_start(card, monkeypatch):
    _, plugin, ctrl = card
    plugin._high_ctrl = None
    assert call(plugin)["command_sent"] is False
    plugin._high_ctrl = ctrl
    def fail():
        raise RuntimeError("DDS unavailable")
    monkeypatch.setattr(ctrl, "get_mode", fail)
    assert call(plugin)["command_sent"] is False
    assert call(plugin, "info")["state"] == "error"
    assert ctrl.commands == []


def test_mode_is_rechecked_before_publish(card, monkeypatch):
    _, plugin, ctrl = card
    modes = iter([30, 2])
    monkeypatch.setattr(ctrl, "get_mode", lambda: next(modes))
    assert call(plugin)["command_sent"] is False
    assert ctrl.commands == []


def test_protection_during_enable_is_not_success(card):
    _, plugin, ctrl = card
    ctrl.next_mode = 26
    result = call(plugin)
    assert result["code"] == "PROTECTION_ACTIVE"
    assert result["command_sent"] is True
    assert result["confirmed"] is False
    assert [c[3] for c in ctrl.commands] == ["START", "DEFAULT"]


def test_timeout_blocks_blind_retry(card, monkeypatch):
    module, plugin, ctrl = card
    ctrl.next_mode = 30
    clock = [100.0]
    monkeypatch.setattr(module.time, "monotonic", lambda: clock[0])
    monkeypatch.setattr(plugin._enable_cancel, "wait", lambda _: clock.__setitem__(0, clock[0] + 1))
    assert call(plugin)["code"] == "ENABLE_TIMEOUT"
    assert call(plugin)["code"] == "ENABLE_UNCONFIRMED"
    assert plugin._owner._prepare_workmode(1, "stand_up")["state"] == "error"
    assert [c[3] for c in ctrl.commands] == ["START", "DEFAULT"]
    ctrl.mode = 0  # Delayed acknowledgement: no second START is necessary.
    assert call(plugin)["confirmed"] is True
    assert plugin._owner._enable_unconfirmed is False
    assert len(ctrl.commands) == 2


@pytest.mark.parametrize("failed_command", ["START", "DEFAULT"])
def test_uncertain_delivery_is_not_retried(card, monkeypatch, failed_command):
    _, plugin, ctrl = card
    ctrl.next_mode = 30
    publish = ctrl.publish_cmd
    def fail(*args):
        publish(*args)
        if args[3] == failed_command:
            raise RuntimeError("DDS publish failed")
    monkeypatch.setattr(ctrl, "publish_cmd", fail)
    result = call(plugin)
    assert result["code"] == "ENABLE_SDK_ERROR"
    assert result["command_attempted"] is True
    assert result["command_sent"] is None
    assert call(plugin)["code"] == "ENABLE_UNCONFIRMED"
    assert len([c for c in ctrl.commands if c[3] == "START"]) == 1


def test_concurrent_enable_calls_are_serialized(card, monkeypatch):
    _, plugin, ctrl = card
    entered, release = threading.Event(), threading.Event()
    results = []
    publish = ctrl.publish_cmd
    def wait_before_publish(*args):
        if args[3] == "START":
            entered.set()
            assert release.wait(2)
        publish(*args)
    monkeypatch.setattr(ctrl, "publish_cmd", wait_before_publish)
    worker = threading.Thread(target=lambda: results.append(call(plugin)))
    worker.start()
    try:
        assert entered.wait(2)
        assert call(plugin)["code"] == "ENABLE_BUSY"
    finally:
        release.set()
        worker.join(2)
    assert not worker.is_alive()
    assert results[0]["state"] == "completed"
    assert [c[3] for c in ctrl.commands] == ["START", "DEFAULT"]


def test_enable_reserves_chassis_and_pauses_stream(card, monkeypatch):
    _, plugin, ctrl = card
    entered, release = threading.Event(), threading.Event()
    class Servo:
        paused = False
        owned = False
        def pause_for_explicit_command(self, reason):
            self.paused = True
            return True
        def set_explicit_velocity_owner(self, value):
            self.owned = value
    servo = Servo()
    plugin._owner.attach_servo(servo)
    publish = ctrl.publish_cmd
    def wait_before_publish(*args):
        if args[3] == "START":
            assert servo.paused and servo.owned
            entered.set()
            assert release.wait(2)
        publish(*args)
    monkeypatch.setattr(ctrl, "publish_cmd", wait_before_publish)
    worker = threading.Thread(target=lambda: call(plugin))
    worker.start()
    try:
        assert entered.wait(2)
        result = plugin._owner.dispatch("stand_up", {"_tool_name": "stand_up_lie_prone"})
        assert result["state"] == "error"
        assert result["command_sent"] is False
    finally:
        release.set()
        worker.join(2)
    assert not worker.is_alive()
    assert not servo.owned
    assert [c[3] for c in ctrl.commands] == ["START", "DEFAULT"]


def test_stop_after_start_releases_event_without_second_toggle(card, monkeypatch):
    _, plugin, ctrl = card
    ctrl.next_mode = 30
    publish = ctrl.publish_cmd
    def cancel(*args):
        publish(*args)
        if args[3] == "START":
            call(plugin, "stop")
    monkeypatch.setattr(ctrl, "publish_cmd", cancel)
    result = call(plugin)
    assert result["state"] == "cancelled"
    assert result["command_sent"] is True
    assert [c[3] for c in ctrl.commands] == ["START", "DEFAULT"]
    assert call(plugin)["code"] == "ENABLE_UNCONFIRMED"


def test_card_is_discoverable_in_enabled_bundle_and_manifest(card):
    _, plugin, _ = card
    tool = plugin.get_tool()
    assert tool["type"] == "actuator"
    assert tool["inputSchema"]["x-is-dangerous"] is True
    assert tool["inputSchema"]["x-action-params"]["enable"]["params"] == []
    assert tool["inputSchema"]["x-action-params"]["disable"]["params"] == []
    assert "confirm" not in tool["inputSchema"]["properties"]
    bumi = Path(__file__).resolve().parents[1]
    manifest = yaml.safe_load((bumi / "driver.yaml").read_text(encoding="utf-8"))
    assert {"name": "motor_enable", "type": "actuator"} in manifest["cards"]
    assert yaml.safe_load((bumi / "config.yaml").read_text(encoding="utf-8"))["plugins"]["motor_enable"]["enabled"] is True


def test_disable_from_enabled_sends_once_and_is_idempotent(card):
    _, plugin, ctrl = card
    ctrl.mode, ctrl.next_mode = 0, 30
    result = call(plugin, "disable")
    assert result["state"] == "completed"
    assert result["workmode"] == 30
    assert [c[3] for c in ctrl.commands] == ["START", "DEFAULT"]
    assert call(plugin, "disable")["command_sent"] is False
    assert len(ctrl.commands) == 2


def test_timeout_cannot_be_cleared_by_requesting_source_mode(card, monkeypatch):
    module, plugin, ctrl = card
    ctrl.next_mode = 30
    clock = [100.0]
    monkeypatch.setattr(module.time, "monotonic", lambda: clock[0])
    monkeypatch.setattr(plugin._enable_cancel, "wait", lambda _: clock.__setitem__(0, clock[0] + 1))
    assert call(plugin, "enable")["code"] == "ENABLE_TIMEOUT"
    assert call(plugin, "disable")["code"] == "ENABLE_UNCONFIRMED"
    assert [c[3] for c in ctrl.commands] == ["START", "DEFAULT"]
