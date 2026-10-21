"""101–107 registration, old-Core ordered slots, and reusable calibration windows.

All hardware and calibration confirmations here are synthetic test fixtures.
"""
import copy
import json
import time

import pytest

from test_bumi_exercise_routine import (
    BUMI, card, hardware, routine, finish, run, wait_for, plan,
)


def window(asset, seconds=0.025):
    return {"seconds": seconds, "source": "operator_confirmation",
            "robot_id": asset["robot_id"], "recording_id": asset["recording_id"],
            "confirmed_at": "2026-10-21T00:00:00Z"}


def register(card):
    result = card.dispatch("register_recordings", {})
    assert result["ok"], result
    return result


def calibrate(card, motion):
    result = card.dispatch("calibrate", {"motion": motion, "dry_run": "false", "confirm": "true"})
    assert result["ok"], result
    status = wait_for(card, "awaiting_confirmation")
    # Represents observing the *full* clip on the fake controller only.
    time.sleep(0.025)
    ack = card.dispatch("finish_calibration", {"job_id": status["job_id"], "confirm": "true"})
    assert ack["ok"], ack
    result = wait_for(card, {"completed", "error"})
    card._job["thread"].join(timeout=2)
    return result


def test_registration_retains_existing_101_and_is_idempotent(card, hardware):
    old = card._library["chest_expand"]
    old.update(recording_id=101, playback_window=window({**old, "recording_id": 101}))
    # Fixture edits must be reflected on disk, as an operator's existing library.
    card._library_path.write_text(json.dumps({"schema_version": 1, "motions": list(card._library.values())}))
    before = copy.deepcopy(old)
    result = register(card)
    assert result["existing"] == ["chest_expand"]
    assert result["added"] == [f"recording_{i}" for i in range(102, 108)]
    assert card._library["chest_expand"] == before
    for recording_id in range(102, 108):
        asset = card._library[f"recording_{recording_id}"]
        assert asset["recording_id"] == recording_id and not asset["verified"]
        assert asset["robot_id"] == "mock-robot" and "playback_window" not in asset
    assert len(register(card)["added"]) == 0
    assert not hardware[0].calls and hardware[0].reads == 0
    assert result["motion_sent"] is False


def test_registration_and_calibration_require_robot_specific_persistent_library(hardware):
    instance = routine.ExerciseRoutinePlugin({}, hardware[1])
    assert not instance.dispatch("register_recordings", {})["ok"]
    assert not instance.dispatch("calibrate", {"motion": "arms_up", "dry_run": False, "confirm": True})["ok"]
    assert not hardware[0].calls


def test_unverified_registered_actions_cannot_run_outside_calibration(card, hardware):
    register(card)
    result = card.dispatch("run", {"motion": "recording_102", "dry_run": False, "confirm": True})
    assert not result["ok"] and "尚未真机验证" in result["error"]
    assert not hardware[0].calls


def test_single_calibration_persists_only_after_confirmation_and_successful_exit(card, hardware):
    register(card)
    result = calibrate(card, "recording_102")
    assert result["state"] == "completed" and result["calibration_saved"]
    assert result["playback_exit_confirmed"] and not result["control_reserved"]
    saved = routine.load_library(card._library_path)["recording_102"]
    assert saved["verified"] and saved["duration_s"] is None
    assert saved["playback_window"]["seconds"] >= 0.025
    assert saved["playback_window"]["source"] == "operator_confirmation"
    # Reopening the driver must retain the window without a new measurement.
    reopened = routine.ExerciseRoutinePlugin(
        {"library_path": str(card._library_path), "robot_id": "mock-robot"}, hardware[1])
    preview = reopened.dispatch("validate", {"motion": "recording_102", "completion_strategy": "timed_exit",
                                             "dry_run": False})
    assert preview["ok"] and preview["steps"][0]["playback_s"] == saved["playback_window"]["seconds"]
    assert preview["steps"][0]["playback_window_source"] == "operator_calibration"


def test_ordered_multi_action_two_rounds_reuses_distinct_saved_windows_without_manual_seconds(card, hardware):
    register(card)
    # Deliberately non-sorted order: tests user-selected sequence, not ID sorting.
    for recording_id in (101, 103, 102):
        assert calibrate(card, f"recording_{recording_id}")["calibration_saved"]
    hardware[0].calls.clear()
    args = {"motion": "recording_101", "motion_2": "recording_103", "motion_3": "recording_102",
            "motion_4": "", "motion_5": " ", "rounds": "2", "play_count": "1",
            "rest_s": "0.01", "round_rest_s": "0.02", "completion_strategy": "timed_exit",
            "playback_s": "", "dry_run": "false", "confirm": "true"}
    preview = card.dispatch("validate", args)
    assert preview["ok"] and preview["total_steps"] == 3 and preview["total_plays"] == 6
    assert [s["recording_id"] for s in preview["steps"]] == [101, 103, 102]
    assert all(s["playback_window_source"] == "operator_calibration" for s in preview["steps"])
    assert card.dispatch("run", args)["ok"]
    result = wait_for(card, {"completed", "error"})
    card._job["thread"].join(timeout=2)
    commands = [(c[0], c[1]) for c in hardware[0].calls if c[0] != "DEFAULT"]
    assert commands == [(command, index) for index in [101, 103, 102] * 2
                        for command, index in [("PLAYTEACH", index), ("WALK", 0)]]
    assert result["exited_plays"] == 6 and result["completed_plays"] == 0
    assert not result["full_playback_confirmed"] and not result["control_reserved"]


def test_old_core_schema_exposes_seven_slots_and_confirmation_actions(card):
    register(card)
    schema = card.get_tool()["inputSchema"]
    for slot in ["motion", *[f"motion_{i}" for i in range(2, 8)]]:
        assert "recording_107" in schema["properties"][slot]["enum"]
        assert slot in schema["x-action-params"]["run"]["params"]
    assert {"continue", "calibrate", "finish_calibration", "register_recordings"} <= set(schema["properties"]["action"]["enum"])


def test_all_seven_actions_replay_in_selected_order_for_two_rounds(card, hardware):
    register(card)
    for recording_id in range(101, 108):
        assert calibrate(card, f"recording_{recording_id}")["calibration_saved"]
    order = [101, 103, 105, 102, 107, 104, 106]
    args = {("motion" if slot == 1 else f"motion_{slot}"): f"recording_{recording_id}"
            for slot, recording_id in enumerate(order, 1)}
    args.update(rounds="2", play_count="1", completion_strategy="timed_exit", playback_s="",
                dry_run="false", confirm="true")
    hardware[0].calls.clear()
    assert card.dispatch("run", args)["ok"]
    result = wait_for(card, {"completed", "error"})
    card._job["thread"].join(timeout=2)
    assert result["state"] == "completed" and result["exited_plays"] == 14
    assert [c[1] for c in hardware[0].calls if c[0] == "PLAYTEACH"] == order * 2
    assert [c[0] for c in hardware[0].calls if c[0] != "DEFAULT"] == ["PLAYTEACH", "WALK"] * 14
    assert result["playback_exit_confirmed"] and not result["control_reserved"]


def test_explicit_window_override_does_not_replace_saved_calibration(card):
    register(card)
    assert calibrate(card, "recording_102")["calibration_saved"]
    before = card._library_path.read_text()
    preview = card.dispatch("validate", {"motion": "recording_102", "completion_strategy": "timed_exit",
                                         "playback_s": "0.1", "dry_run": "false"})
    assert preview["ok"] and preview["steps"][0]["playback_s"] == 0.1
    assert preview["steps"][0]["playback_window_source"] == "explicit"
    assert card._library_path.read_text() == before


def test_calibration_disk_failure_does_not_claim_saved_or_mark_memory_verified(card, monkeypatch):
    register(card)
    def fail(_motions):
        raise OSError("read-only library volume")
    monkeypatch.setattr(card, "_persist_library", fail)
    result = calibrate(card, "recording_102")
    assert result["state"] == "error" and not result["calibration_saved"]
    assert not card._library["recording_102"]["verified"]
    assert result["playback_exit_confirmed"] and not result["control_reserved"]


@pytest.mark.parametrize("flags", [{}, {"dry_run": True, "confirm": True},
                                   {"dry_run": False, "confirm": False}])
def test_calibration_needs_explicit_real_start_confirmation(card, hardware, flags):
    register(card)
    assert not card.dispatch("calibrate", {"motion": "recording_102", **flags})["ok"]
    assert not hardware[0].calls


@pytest.mark.parametrize("cause", ["cancel", "exit_failure", "timeout", "protection"])
def test_aborted_calibration_never_saves_verified_or_window(card, hardware, cause):
    register(card)
    if cause == "timeout":
        card._library["recording_102"]["timeout_s"] = 0.02
    assert card.dispatch("calibrate", {"motion": "recording_102", "dry_run": False, "confirm": True})["ok"]
    status = wait_for(card, "awaiting_confirmation")
    if cause == "cancel":
        card.dispatch("cancel", {})
    elif cause == "exit_failure":
        hardware[0].exit_ok = False
        assert card.dispatch("finish_calibration", {"job_id": status["job_id"], "confirm": True})["ok"]
    elif cause == "protection":
        hardware[0].mode = 26
    result = wait_for(card, {"cancelled", "error"})
    card._job["thread"].join(timeout=2)
    saved = routine.load_library(card._library_path)["recording_102"]
    assert not saved["verified"] and "playback_window" not in saved
    assert not result["calibration_saved"]


def test_finish_calibration_rejects_old_job_or_ordinary_routine(card):
    register(card)
    assert card.dispatch("calibrate", {"motion": "recording_102", "dry_run": False, "confirm": True})["ok"]
    wait_for(card, "awaiting_confirmation")
    assert not card.dispatch("finish_calibration", {"job_id": "old-job", "confirm": True})["ok"]
    assert not card._job["ack"].is_set()
    card.dispatch("cancel", {})
    finish(card)
    run(card)
    status = wait_for(card, "awaiting_confirmation")
    assert not card.dispatch("finish_calibration", {"job_id": status["job_id"], "confirm": True})["ok"]
    finish(card)


@pytest.mark.parametrize("field,value", [("robot_id", "other"), ("recording_id", 999),
    ("source", "guess"), ("confirmed_at", ""), ("seconds", 0), ("seconds", True), ("seconds", float("nan"))])
def test_saved_window_evidence_cannot_be_reused_for_different_or_invalid_recording(card, hardware, field, value):
    asset = card._library["chest_expand"]
    asset["playback_window"] = {**window(asset), field: value}
    result = card.dispatch("run", {"motion": "chest_expand", "completion_strategy": "timed_exit",
                                   "dry_run": False, "confirm": True})
    assert not result["ok"] and not hardware[0].calls


def test_external_library_changes_are_not_overwritten_by_registration(card):
    data = json.loads(card._library_path.read_text())
    data["motions"][0]["name"] = "changed outside driver"
    card._library_path.write_text(json.dumps(data))
    result = card.dispatch("register_recordings", {})
    assert not result["ok"] and "外部修改" in result["error"]
    assert json.loads(card._library_path.read_text()) == data


def test_registration_while_playing_is_rejected_without_modifying_library(card):
    before = card._library_path.read_text()
    run(card)
    wait_for(card, "awaiting_confirmation")
    assert not card.dispatch("register_recordings", {})["ok"]
    assert card._library_path.read_text() == before
    finish(card)
