"""Walking-mode arbitration, feedback and failure paths without hardware."""

from __future__ import annotations

import importlib.util
import sys
from contextlib import contextmanager
from pathlib import Path
from types import SimpleNamespace

import pytest


BUMI = Path(__file__).resolve().parents[1] / "noetix" / "bumi"
SPEC = importlib.util.spec_from_file_location(
    "bumi_walking_mode_test", BUMI / "walking_mode.py")
MODULE = importlib.util.module_from_spec(SPEC)
sys.path.insert(0, str(BUMI))
try:
    SPEC.loader.exec_module(MODULE)
finally:
    sys.path.remove(str(BUMI))


class ControllerTripwire:
    def __getattr__(self, name):
        raise AssertionError(f"unexpected SDK access: {name}")


class ReadOnlyController(ControllerTripwire):
    mode = 2
    velocity = 0.0

    def get_mode(self):
        return self.mode

    def get_joint_state(self):
        return [SimpleNamespace(vel=self.velocity) for _ in range(21)]


class FakeLoco:
    reserved = False
    busy = False
    pauses = 0

    @contextmanager
    def reserve_walking_mode(self):
        if self.busy or self.reserved:
            raise RuntimeError("chassis busy")
        self.reserved = True
        try:
            yield
        finally:
            self.reserved = False

    def _preempt_servo(self, reason):
        assert self.reserved
        self.pauses += 1
        return True


@pytest.fixture
def card(monkeypatch):
    controller = ReadOnlyController()
    loco = FakeLoco()
    result = MODULE.WalkingModePlugin(
        {"app_host": "127.0.0.1"}, "bumi_test", object(), controller, loco)
    monkeypatch.setattr(result._cancel, "wait", lambda _timeout: False)

    class Client:
        commands = []
        status_override = None
        after_switch = None
        send_error = False

        def __init__(self, host, **kwargs):
            assert loco.reserved

        def __enter__(self):
            return self

        def __exit__(self, *_args):
            pass

        def get_status(self):
            if self.commands and self.after_switch is not None:
                if isinstance(self.after_switch, Exception):
                    raise self.after_switch
                return self.after_switch
            return controller.mode if self.status_override is None else self.status_override

        def set_walking_model(self, model):
            assert loco.reserved
            self.commands.append(model)
            if self.send_error:
                raise OSError("send interrupted")
            controller.mode = MODULE.MODEL_CODES[model]

    result.test_client = Client
    monkeypatch.setattr(MODULE, "WalkingModeClient", Client)
    return result


def test_schema_replaces_action_with_the_requested_mode_dropdown(card):
    tool = card.get_tool()
    assert tool["type"] == "actuator"
    assert tool["multiInstance"] is False
    schema = tool["inputSchema"]
    assert set(schema["properties"]) == {"walking_model"}
    assert schema["required"] == ["walking_model"]
    assert schema["properties"]["walking_model"]["enum"] == [
        "lower_body", "whole_body"]


@pytest.mark.parametrize("model", ["lower_body", "whole_body"])
def test_switch_requires_both_app_and_sdk_feedback_and_sends_only_once(card, model):
    card._high_ctrl.mode = 3 if model == "lower_body" else 2
    result = card.dispatch("walking_mode", {"walking_model": model})
    assert result["state"] == "completed"
    assert result["command_sent"] is True
    assert result["mode_verified"] is True
    assert result["walking_model"] == model
    assert result["observed_workmode"] == result["app_workmode"] == MODULE.MODEL_CODES[model]
    assert card.test_client.commands == [model]
    assert card._loco.pauses == 1
    assert card._loco.reserved is False


def test_same_mode_is_a_noop_without_pausing_servo(card):
    result = card.dispatch("walking_mode", {"walking_model": "lower_body"})
    assert result["state"] == "completed"
    assert result["command_sent"] is False
    assert result["mode_verified"] is True
    assert card.test_client.commands == []
    assert card._loco.pauses == 0


def test_unconfigured_host_uses_passive_discovery(card, monkeypatch):
    seen = []
    card._host = None
    monkeypatch.setattr(MODULE, "discover_host", lambda source, timeout: (
        seen.append((source, timeout)) or "127.0.0.1"))
    result = card.dispatch("walking_mode", {"walking_model": "whole_body"})
    assert result["state"] == "completed"
    assert seen == [("192.168.55.102", 3.0)]


@pytest.mark.parametrize("model", [None, "", "WALK", "RUN", 0, True, [], {}])
def test_invalid_choices_do_not_reach_sdk_or_network(card, model):
    card._high_ctrl = ControllerTripwire()
    result = card.dispatch("walking_mode", {"walking_model": model})
    assert result["code"] == "invalid_walking_model"
    assert result["command_sent"] is False
    assert card.test_client.commands == []


def test_lifecycle_and_info_never_touch_hardware(card):
    card._high_ctrl = ControllerTripwire()
    card.start()
    for action in ("start", "info"):
        result = card.dispatch(action, {})
        assert result["state"] == "ready"
        assert result["command_sent"] is False
    assert card.dispatch("stop", {}) == {"state": "idle", "command_sent": False}
    card.stop()
    assert card.dispatch("move", {"walking_model": "whole_body"}) is None


def test_request_does_not_become_a_claimed_hardware_acceptance(card):
    card.dispatch("walking_mode", {"walking_model": "whole_body"})
    result = card.dispatch("info", {})
    assert result["hardware_switch_verified"] is False
    assert "walking_model" not in result


@pytest.mark.parametrize("mode", [0, 1, 5, 26, 30])
def test_nonwalking_modes_cannot_enable_or_stand_up_the_robot(card, mode):
    card._high_ctrl.mode = mode
    result = card.dispatch("walking_mode", {"walking_model": "whole_body"})
    assert result["code"] == "invalid_workmode"
    assert result["command_sent"] is False
    assert card.test_client.commands == []
    assert card._loco.reserved is False


def test_feedback_disagreement_refuses_switch(card):
    card.test_client.status_override = 3
    result = card.dispatch("walking_mode", {"walking_model": "whole_body"})
    assert result["code"] == "feedback_mismatch"
    assert card.test_client.commands == []


def test_mode_changing_during_preflight_cannot_receive_the_switch(card, monkeypatch):
    def changing_joint_state():
        card._high_ctrl.mode = 26
        return [SimpleNamespace(vel=0.0) for _ in range(21)]

    monkeypatch.setattr(card._high_ctrl, "get_joint_state", changing_joint_state)
    result = card.dispatch("walking_mode", {"walking_model": "whole_body"})
    assert result["code"] == "workmode_changed"
    assert result["command_sent"] is False
    assert card.test_client.commands == []


@pytest.mark.parametrize("velocity", [0.15, 0.3, float("nan"), float("inf")])
def test_active_or_unreadable_joints_refuse_switch(card, velocity):
    card._high_ctrl.velocity = velocity
    result = card.dispatch("walking_mode", {"walking_model": "whole_body"})
    assert result["state"] == "error"
    assert result["command_sent"] is False
    assert card.test_client.commands == []


def test_busy_chassis_and_missing_loco_refuse_without_sdk_access(card):
    card._high_ctrl = ControllerTripwire()
    card._loco.busy = True
    result = card.dispatch("walking_mode", {"walking_model": "whole_body"})
    assert result["state"] == "error"
    assert result["command_sent"] is False
    card._loco = None
    assert card.dispatch("walking_mode", {"walking_model": "whole_body"})["code"] == "missing_arbitration"


def test_stop_prevents_later_execution_until_restarted(card):
    card.stop()
    card._high_ctrl = ControllerTripwire()
    result = card.dispatch("walking_mode", {"walking_model": "whole_body"})
    assert result["code"] == "stopped"
    assert result["command_sent"] is False


def test_send_failure_reports_possible_delivery_without_retrying(card):
    card.test_client.send_error = True
    result = card.dispatch("walking_mode", {"walking_model": "whole_body"})
    assert result["state"] == "error"
    assert result["command_sent"] is False
    assert result["command_may_have_been_sent"] is True
    assert result["mode_verified"] is False
    assert card.test_client.commands == ["whole_body"]
    assert card._loco.reserved is False


@pytest.mark.parametrize("failure", [26, OSError("connection lost")])
def test_protection_or_lost_feedback_does_not_claim_completion_or_retry(card, failure):
    card.test_client.after_switch = failure
    result = card.dispatch("walking_mode", {"walking_model": "whole_body"})
    assert result["state"] == "error"
    assert result["command_sent"] is True
    assert result["mode_verified"] is False
    assert card.test_client.commands == ["whole_body"]


def test_sdk_mode_alone_cannot_confirm_success(card, monkeypatch):
    card.test_client.after_switch = 2
    ticks = iter([0.0, 0.0, 10.0])
    monkeypatch.setattr(MODULE, "time", SimpleNamespace(monotonic=lambda: next(ticks)))
    result = card.dispatch("walking_mode", {"walking_model": "whole_body"})
    assert result["code"] == "mode_not_confirmed"
    assert result["observed_workmode"] == 3
    assert result["app_workmode"] == 2
    assert result["command_sent"] is True
    assert result["mode_verified"] is False
    assert card.test_client.commands == ["whole_body"]
