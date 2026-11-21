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


class StatusObserver:
    """Simulate ongoing, locally received DDS samples without hardware."""
    def __init__(self, ctrl):
        self.ctrl = ctrl
        self.clock = 0
        self.age_ms = 0
        self.frozen = False
        self.sample = {"workmode": ctrl.mode, "age_ms": 0, "sequence": 0, "received_ns": 0}

    def snapshot(self):
        if not self.frozen:
            self.clock += 1
            self.sample = {"workmode": self.ctrl.mode, "age_ms": self.age_ms,
                           "sequence": self.clock, "received_ns": self.clock}
        return dict(self.sample)

    def mark(self):
        self.clock += 1
        return self.clock


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
    plugin._feedback = StatusObserver(ctrl)
    return module, plugin, ctrl


def call(plugin, action="enable", **args):
    if action == "enable" and "confirm" not in args:
        args["confirm"] = True
    return plugin.dispatch(action, {"_tool_name": "motor_enable", **args})


@pytest.mark.parametrize("args", [{}, {"confirm": False}, {"confirm": "true"}, {"confirm": 1}])
def test_enable_requires_explicit_boolean_confirmation(card, monkeypatch, args):
    _, plugin, ctrl = card
    def unexpected_reservation():
        raise AssertionError("unconfirmed request reserved chassis")
    monkeypatch.setattr(plugin._owner, "reserve_motor_enable", unexpected_reservation)
    result = plugin.dispatch("enable", args)
    assert result["code"] == "CONFIRM_REQUIRED"
    assert result["command_sent"] is False
    assert ctrl.commands == []


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
    status = call(plugin, "status")
    assert status["workmode"] == 30
    assert status["feedback_freshness"] == "recent"
    assert status["command_sent"] is False
    assert call(plugin, "stop")["state"] == "idle"
    assert call(plugin, "unsupported") is None
    assert ctrl.commands == []


def test_missing_or_failed_feedback_never_sends_start(card, monkeypatch):
    _, plugin, ctrl = card
    plugin._high_ctrl = None
    assert call(plugin)["command_sent"] is False
    plugin._high_ctrl = ctrl
    def fail():
        raise module.FeedbackUnavailable("DDS unavailable")
    module, _, _ = card
    monkeypatch.setattr(plugin, "_snapshot", fail)
    assert call(plugin)["command_sent"] is False
    assert call(plugin, "info")["state"] == "error"
    assert ctrl.commands == []


@pytest.mark.parametrize("age_ms", [1001, 10000])
def test_stale_status_does_not_authorize_start(card, age_ms):
    _, plugin, ctrl = card
    plugin._feedback.age_ms = age_ms
    assert call(plugin, "status")["feedback_freshness"] == "stale"
    assert call(plugin)["code"] == "FEEDBACK_UNAVAILABLE"
    assert ctrl.commands == []


def test_no_status_sample_does_not_authorize_start(card):
    _, plugin, ctrl = card
    plugin._feedback.sample = {"workmode": None, "age_ms": None,
                               "sequence": 0, "received_ns": None}
    plugin._feedback.frozen = True
    assert call(plugin)["code"] == "FEEDBACK_UNAVAILABLE"
    assert ctrl.commands == []


def test_pre_command_sample_cannot_clear_pending_guard(card):
    _, plugin, ctrl = card
    observer = plugin._feedback
    ctrl.mode = 0
    old_sample = observer.snapshot()
    plugin._owner._mode_toggle_target = 0
    plugin._owner._enable_unconfirmed = True
    plugin._owner._motor_enable_pending = True
    plugin._owner._motor_enable_marker = observer.mark()
    observer.frozen = True
    assert old_sample["received_ns"] < plugin._owner._motor_enable_marker
    assert call(plugin, "status")["enable_unconfirmed"] is True
    assert call(plugin)["code"] == "ENABLE_UNCONFIRMED"
    assert plugin._owner._prepare_workmode(1, "stand_up")["state"] == "error"
    assert ctrl.commands == []
    observer.frozen = False
    assert call(plugin, "status")["enable_unconfirmed"] is False


def test_failed_post_send_marker_cannot_reuse_old_marker(card, monkeypatch):
    _, plugin, ctrl = card
    plugin._owner._motor_enable_marker = -100
    def fail_mark():
        raise RuntimeError("clock unavailable")
    monkeypatch.setattr(plugin._feedback, "mark", fail_mark)
    result = call(plugin)
    assert result["code"] == "ENABLE_SDK_ERROR"
    assert plugin._owner._motor_enable_marker is None
    assert call(plugin, "status")["enable_unconfirmed"] is True
    assert [command[3] for command in ctrl.commands] == ["START", "DEFAULT"]


def test_stale_enabled_sample_after_start_does_not_confirm(card, monkeypatch):
    module, plugin, ctrl = card
    clock = [100.0]
    monkeypatch.setattr(module.time, "monotonic", lambda: clock[0])
    monkeypatch.setattr(plugin._enable_cancel, "wait", lambda _: clock.__setitem__(0, clock[0] + 1))
    publish = ctrl.publish_cmd
    def lose_feedback(*args):
        publish(*args)
        if args[3] == "START":
            plugin._feedback.age_ms = 2000
    monkeypatch.setattr(ctrl, "publish_cmd", lose_feedback)
    result = call(plugin)
    assert result["code"] == "ENABLE_TIMEOUT"
    assert result["feedback_freshness"] == "stale"
    assert plugin._owner._enable_unconfirmed is True
    assert call(plugin, "status")["enable_unconfirmed"] is True
    assert plugin._owner._prepare_workmode(1, "stand_up")["state"] == "error"
    assert [command[3] for command in ctrl.commands].count("START") == 1


def test_mode_is_rechecked_before_publish(card, monkeypatch):
    _, plugin, ctrl = card
    modes = iter([30, 2])
    monkeypatch.setattr(plugin, "require_recent_mode", lambda: next(modes))
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


def test_timeout_blocks_confirmed_retry_and_automatic_preparation(card, monkeypatch):
    module, plugin, ctrl = card
    ctrl.next_mode = 30
    clock = [100.0]
    monkeypatch.setattr(module.time, "monotonic", lambda: clock[0])
    monkeypatch.setattr(plugin._enable_cancel, "wait", lambda _: clock.__setitem__(0, clock[0] + 1))
    timeout = call(plugin)
    assert timeout["code"] == "ENABLE_TIMEOUT" and timeout["retry_available"] is False
    assert clock[0] == 104.0
    assert plugin._owner._prepare_workmode(1, "stand_up")["state"] == "error"
    commands = [c[3] for c in ctrl.commands]
    assert commands[0] == "START" and commands.count("START") == 1
    assert len(commands) > 2 and set(commands[1:]) == {"DEFAULT"}
    assert plugin.dispatch("enable", {})["code"] == "CONFIRM_REQUIRED"
    retry = call(plugin)
    assert retry["code"] == "ENABLE_UNCONFIRMED"
    assert retry["command_sent"] is False
    assert [c[3] for c in ctrl.commands] == commands


def test_default_refresh_stops_when_enabled_mode_is_observed(card, monkeypatch):
    module, plugin, ctrl = card
    ctrl.next_mode = 30
    clock = [100.0]
    waits = []
    monkeypatch.setattr(module.time, "monotonic", lambda: clock[0])
    def wait(delay):
        waits.append(delay)
        clock[0] += delay
    monkeypatch.setattr(plugin._enable_cancel, "wait", wait)
    publish = ctrl.publish_cmd
    def acknowledge_after_three_defaults(*args):
        publish(*args)
        if [command[3] for command in ctrl.commands].count("DEFAULT") == 3:
            ctrl.mode = 0
    monkeypatch.setattr(ctrl, "publish_cmd", acknowledge_after_three_defaults)
    result = call(plugin)
    assert result["state"] == "completed" and result["workmode"] == 0
    assert result["feedback_freshness"] == "recent"
    assert [command[3] for command in ctrl.commands] == ["START", "DEFAULT", "DEFAULT", "DEFAULT"]
    assert waits == [0.01, 0.01]


@pytest.mark.parametrize("action", ["info", "status"])
def test_info_reconciles_delayed_ack_after_timeout(card, monkeypatch, action):
    module, plugin, ctrl = card
    ctrl.next_mode = 30
    clock = [100.0]
    monkeypatch.setattr(module.time, "monotonic", lambda: clock[0])
    monkeypatch.setattr(plugin._enable_cancel, "wait", lambda _: clock.__setitem__(0, clock[0] + 1))
    assert call(plugin)["code"] == "ENABLE_TIMEOUT"
    commands = [c[3] for c in ctrl.commands]
    ctrl.mode = 0  # Delayed acknowledgement: no second START is necessary.
    info = call(plugin, action)
    assert info["workmode"] == 0
    assert info["enable_unconfirmed"] is False
    assert plugin._owner._enable_unconfirmed is False
    assert [c[3] for c in ctrl.commands] == commands


@pytest.mark.parametrize("action", ["info", "status"])
def test_info_does_not_clear_uncertainty_during_body_action(card, action):
    _, plugin, ctrl = card
    ctrl.mode = 0
    plugin._owner._enable_unconfirmed = True
    plugin._owner._mode_toggle_target = 0
    plugin._owner._motor_enable_pending = True
    plugin._owner._motor_enable_marker = -1
    plugin._owner._body_action = "another_action"
    assert call(plugin, action)["enable_unconfirmed"] is True
    assert ctrl.commands == []
    plugin._owner._body_action = None
    assert call(plugin, action)["enable_unconfirmed"] is False
    assert ctrl.commands == []


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
    if failed_command == "START":
        assert [c[3] for c in ctrl.commands] == ["START", "DEFAULT"]


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


@pytest.mark.parametrize("stop_via_dispatch", [True, False])
def test_stop_during_enable_initialization_is_not_lost(card, monkeypatch, stop_via_dispatch):
    _, plugin, ctrl = card
    clear_entered, release_clear = threading.Event(), threading.Event()
    stop_entered, stop_done = threading.Event(), threading.Event()
    reserve_entered, release_reserve = threading.Event(), threading.Event()
    results = []

    # Signal either a direct event set (old implementation) or an attempt to
    # take the new cancellation lock before allowing clear() to continue.
    original_lock = getattr(plugin, "_cancel_lock", threading.Lock())
    class ObservedLock:
        def __enter__(self):
            if threading.current_thread().name == "stop-request":
                stop_entered.set()
            original_lock.acquire()
            return self
        def __exit__(self, *_args):
            original_lock.release()
    monkeypatch.setattr(plugin, "_cancel_lock", ObservedLock(), raising=False)
    original_clear = plugin._enable_cancel.clear
    original_set = plugin._enable_cancel.set
    def pause_clear():
        clear_entered.set()
        assert release_clear.wait(2)
        original_clear()
    def observe_set():
        original_set()
        stop_entered.set()
    monkeypatch.setattr(plugin._enable_cancel, "clear", pause_clear)
    monkeypatch.setattr(plugin._enable_cancel, "set", observe_set)
    reserve = plugin._owner.reserve_motor_enable
    def pause_reservation():
        reserve_entered.set()
        assert release_reserve.wait(2)
        return reserve()
    monkeypatch.setattr(plugin._owner, "reserve_motor_enable", pause_reservation)

    def issue_stop():
        if stop_via_dispatch:
            call(plugin, "stop")
        else:
            plugin.stop()
        stop_done.set()

    worker = threading.Thread(target=lambda: results.append(call(plugin)))
    stopper = threading.Thread(target=issue_stop, name="stop-request")
    worker.start()
    try:
        assert clear_entered.wait(2)
        stopper.start()
        assert stop_entered.wait(2)
        release_clear.set()
        assert reserve_entered.wait(2)
        assert stop_done.wait(2)
    finally:
        release_clear.set()
        release_reserve.set()
        worker.join(2)
        if stopper.ident is not None:
            stopper.join(2)
    assert not worker.is_alive() and not stopper.is_alive()
    assert results[0]["state"] == "cancelled"
    assert results[0]["command_sent"] is False
    assert ctrl.commands == []


def test_stop_while_idle_does_not_cancel_next_enable(card):
    _, plugin, ctrl = card
    assert call(plugin, "stop")["state"] == "idle"
    assert call(plugin)["state"] == "completed"
    assert [c[3] for c in ctrl.commands] == ["START", "DEFAULT"]


def test_stop_during_final_mode_check_prevents_start(card, monkeypatch):
    _, plugin, ctrl = card
    entered, release = threading.Event(), threading.Event()
    results = []
    reads = [0]
    def get_mode():
        reads[0] += 1
        if reads[0] == 2:
            entered.set()
            assert release.wait(2)
        return 30
    monkeypatch.setattr(plugin, "require_recent_mode", get_mode)
    worker = threading.Thread(target=lambda: results.append(call(plugin)))
    worker.start()
    try:
        assert entered.wait(2)
        assert call(plugin, "stop")["state"] == "idle"
    finally:
        release.set()
        worker.join(2)
    assert not worker.is_alive()
    assert results[0]["state"] == "cancelled"
    assert results[0]["command_sent"] is False
    assert ctrl.commands == []


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
    assert plugin.PREFIX == "motor_enable"
    tool = plugin.get_tool()
    assert tool["type"] == "actuator"
    schema = tool["inputSchema"]
    action_params = schema["x-action-params"]
    assert set(action_params) == set(schema["properties"]["action"]["enum"])
    for action in ("status", "info", "start", "stop"):
        assert action_params[action]["params"] == []
        assert action_params[action].get("required", []) == []
    assert tool["inputSchema"]["x-is-dangerous"] is True
    assert tool["inputSchema"]["x-action-params"]["enable"]["params"] == ["confirm"]
    assert tool["inputSchema"]["x-action-params"]["enable"]["required"] == ["confirm"]
    assert "disable" not in tool["inputSchema"]["properties"]["action"]["enum"]
    assert tool["inputSchema"]["properties"]["confirm"]["type"] == "boolean"
    bumi = Path(__file__).resolve().parents[1]
    manifest = yaml.safe_load((bumi / "driver.yaml").read_text(encoding="utf-8"))
    assert {"name": "motor_enable", "type": "actuator"} in manifest["cards"]
    assert yaml.safe_load((bumi / "config.yaml").read_text(encoding="utf-8"))["plugins"]["motor_enable"]["enabled"] is True



def test_disable_is_rejected_without_sending_command(card):
    _, plugin, ctrl = card
    ctrl.mode = 0
    result = call(plugin, "disable")
    assert result is None
    assert ctrl.commands == []
