"""Startup settings reach the controller before any SDK connection or command."""

import time

import pytest

from test_tianyi_loco_servo import command, send, setup
from test_tianyi_loco_servo_process import (
    command as process_command, process_setup, send as process_send, wait_info,
)
from test_tianyi_sdk_worker import native_worker


SETTINGS = {
    "dry_run": False, "rotate_only": True, "require_standing": False,
    "min_vx": 0.02, "linear_acceleration": 0.4,
    "angular_acceleration": 1.0, "wz_limit": 0.5,
}


def test_core_first_start_applies_explicit_settings_before_opening_sdk(setup):
    card = setup.create(dry_run=True)
    info = card.dispatch("start", dict(SETTINGS))
    assert info["state"] == "running"
    for name, value in SETTINGS.items():
        assert info[name] == value
    assert card._backend.connected
    assert card._backend.messages == []  # Connecting does not command motion.
    setup.clock.advance()
    send(card, command(setup))
    info = card.dispatch("info", {})
    assert info["counts"]["accepted"] == info["counts"]["published"] == 1
    assert info["counts"]["dry_run_commands"] == 0
    assert card._backend.messages[-1][0] == 0  # Startup rotate_only is effective.
    assert card._backend.messages[-1][5] > 0


def test_core_start_can_explicitly_select_dry_run_over_yaml_false(setup):
    card = setup.create(dry_run=False)
    assert card.dispatch("start", {"dry_run": True})["dry_run"] is True
    assert not card._backend.connected
    setup.clock.advance()
    send(card, command(setup))
    assert card._counts["dry_run_commands"] == 1
    assert card._backend.messages == []


@pytest.mark.parametrize("saved", [False, True])
def test_omitted_start_option_keeps_saved_configuration(setup, saved):
    card = setup.create(dry_run=not saved)
    assert card.dispatch("config", {"dry_run": saved})["ok"]
    assert card.dispatch("start", {})["dry_run"] is saved


def test_start_options_on_running_paused_card_do_not_resume_it(setup):
    card = setup.create(dry_run=True)
    card.dispatch("start", {})
    assert card.dispatch("pause", {})["state"] == "paused"
    info = card.dispatch("start", {"dry_run": False})
    assert info["state"] == "paused" and info["dry_run"] is False
    setup.clock.advance()
    send(card, command(setup))
    assert card._backend.messages == []
    assert card._counts["accepted"] == 0


def test_running_start_honours_false_without_a_separate_config_call(setup):
    card = setup.create(dry_run=True)
    card.dispatch("start", {})
    info = card.dispatch("start", {"dry_run": False})
    assert info["state"] == "running" and info["dry_run"] is False
    setup.clock.advance()
    send(card, command(setup))
    assert card._counts["dry_run_commands"] == 0
    assert card._counts["published"] == 1 and card._backend.messages[-1][0] > 0


@pytest.mark.parametrize("bad", [
    {"dry_run": "false"}, {"rotate_only": 0}, {"require_standing": None},
    {"linear_acceleration": -1}, {"min_vx": 0.4}, {"wz_limit": 2},
])
def test_invalid_start_settings_do_not_open_resources_or_partially_configure(setup, bad):
    card = setup.create(dry_run=True)
    result = card.dispatch("start", {"dry_run": False, **bad})
    assert result["state"] == "error"
    assert card._backend is None and not card._nodes
    assert not card._running and card._settings["dry_run"] is True


def test_boolean_schema_defaults_follow_configuration_before_and_after_start(setup):
    card = setup.create(dry_run=False, rotate_only=True, require_standing=True)
    expected = {"dry_run": False, "rotate_only": True, "require_standing": True}
    form = card.get_tool()["configSchema"]["properties"]
    assert {name: form[name]["default"] for name in expected} == expected
    card.dispatch("config", {name: not value for name, value in expected.items()})
    form = card.get_tool()["configSchema"]["properties"]
    assert {name: form[name]["default"] for name in expected} == {
        name: not value for name, value in expected.items()}


def test_real_process_first_start_false_reaches_sdk_and_survives_stop_start(process_setup):
    card = process_setup.create(start=False, dry_run=True)
    info = card.dispatch("start", {"dry_run": False})
    assert info["state"] == "running" and info["dry_run"] is False
    assert info["backend"]["connected"] and info["counts"]["published"] == 0
    assert card._config["dry_run"] is False
    assert card.get_tool()["configSchema"]["properties"]["dry_run"]["default"] is False
    # The startup barrier can share a millisecond with this test's request.
    time.sleep(0.005)
    process_send(card, process_command())
    info = wait_info(card, lambda value: value["counts"]["accepted"] == 1)
    assert info["counts"]["dry_run_commands"] == 0
    assert info["backend"]["action_id"] != 0
    assert card.stop()["state"] == "idle"
    info = card.dispatch("start", {})
    assert info["state"] == "running" and info["dry_run"] is False
    assert info["backend"]["connected"]


def test_real_process_saved_false_is_not_overwritten_by_canvas_start_schema(process_setup):
    card = process_setup.create(start=False, dry_run=True)
    # Canvas caches the action schema before the user edits the gear form.
    tool = card.get_tool()
    schema = tool["inputSchema"]
    start_params = schema["x-action-params"]["start"]["params"]
    # Match the canvas action builder: include only defaults advertised by the
    # start action schema. Gear-form defaults live in configSchema and must not
    # leak back into this payload after action:config saved a different value.
    generated_start = {
        name: schema["properties"][name]["default"]
        for name in start_params
        if "default" in schema["properties"][name]
    }
    assert generated_start == {"input_topic": "/actucore/navi/cmd"}
    assert card.dispatch("config", {"dry_run": False})["ok"]
    info = card.dispatch("start", generated_start)
    assert info["state"] == "running" and info["dry_run"] is False
    assert card.get_tool()["configSchema"]["properties"]["dry_run"]["default"] is False
    assert "default" not in card.get_tool()["inputSchema"]["properties"]["dry_run"]


def test_real_process_dry_run_still_never_launches_sdk_when_explicitly_requested(process_setup, tmp_path):
    card = process_setup.create(start=False, dry_run=False,
                                sdk_binary=str(tmp_path / "missing-sdk"))
    info = card.dispatch("start", {"dry_run": True})
    assert info["state"] == "running" and info["dry_run"] is True
    assert info["backend"]["connected"] is False


@pytest.mark.parametrize("bad", [{"dry_run": 0}, {"rotate_only": "false"},
                                 {"linear_acceleration": -1}, {"wz_limit": 2}])
def test_facade_rejects_bad_start_settings_before_spawning_or_changing_epoch(process_setup, bad):
    card = process_setup.create(start=False, dry_run=True)
    before = card._epoch
    assert card.dispatch("start", {"dry_run": False, **bad})["state"] == "error"
    assert card._process is None and not card._nodes and card._epoch == before
    assert card._config["dry_run"] is True


def test_invalid_start_does_not_interrupt_an_existing_velocity_action(process_setup):
    card = process_setup.create()
    process_send(card, process_command())
    before = wait_info(card, lambda info: info["counts"]["accepted"] == 1)
    process, epoch = card._process, card._epoch
    assert card.dispatch("start", {"dry_run": "false"})["state"] == "error"
    assert card._process is process and process.poll() is None
    assert card._epoch == epoch and card._outbox.info()["active"]
    after = card.dispatch("info", {})
    assert after["state"] == "running" and after["dry_run"] is False
    assert after["backend"]["action_id"] == before["backend"]["action_id"]


def test_sdk_connect_failure_is_an_error_instead_of_a_successful_dry_run_fallback(process_setup, tmp_path):
    card = process_setup.create(start=False, dry_run=True,
                                sdk_binary=str(tmp_path / "missing-sdk"))
    result = card.dispatch("start", {"dry_run": False})
    assert result["state"] == "error"
    assert "error" in result
    assert card._outbox.info()["active"] is False
