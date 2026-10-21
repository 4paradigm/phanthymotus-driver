"""Scheduler and real LocoPlugin arbitration against fake hardware; no ROS/SDK."""
from __future__ import annotations

import importlib.util
import json
from pathlib import Path
import sys
import threading
import time
from types import SimpleNamespace

import pytest

ROOT = Path(__file__).resolve().parents[1]
BUMI = ROOT / "noetix" / "bumi"
sys.path.insert(0, str(BUMI))
import exercise_routine as routine


def plan(*, rounds=1, count=1, rest=0, round_rest=0, motions=("chest_expand",)):
    return {"rounds": rounds, "round_rest_s": round_rest,
            "steps": [{"motion": m, "play_count": count, "rest_s": rest} for m in motions]}


def wait_for(card, state, timeout=2):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        result = card.dispatch("status", {})
        if result["state"] in (state if isinstance(state, set) else {state}):
            return result
        time.sleep(0.002)
    pytest.fail(f"waiting for {state}: {card.dispatch('status', {})}")


def finish(card):
    deadline = time.monotonic() + 3
    while time.monotonic() < deadline:
        result = card.dispatch("status", {})
        if result["state"] in routine.TERMINAL:
            card._job["thread"].join(timeout=1)
            return result
        if result["state"] == "awaiting_confirmation":
            assert card.dispatch("continue", {
                "job_id": result["job_id"], "confirmation_token": result["confirmation_token"],
                "confirm": True})["ok"]
        time.sleep(0.002)
    pytest.fail(f"routine did not finish: {result}")


class High:
    def __init__(self):
        self.mode = 2
        self.calls = []
        self.reads = 0
        self.read_error = False
        self.start_ok = True
        self.exit_ok = True

    def get_mode(self):
        self.reads += 1
        if self.read_error:
            raise RuntimeError("DDS read failed")
        return self.mode

    def publish_cmd(self, x, y, z, command, index):
        self.calls.append((command, index, x, y, z))
        if command == "PLAYTEACH" and self.start_ok:
            self.mode = 23
        elif command == "WALK" and self.exit_ok:
            self.mode = 2


@pytest.fixture
def hardware(monkeypatch):
    # Reuse the existing Bumi hardware-import stubs, but exercise the *real*
    # LocoPlugin dispatch, workmode guards and teaching actions.
    path = BUMI / "tests/test_mic_card.py"
    spec = importlib.util.spec_from_file_location("bumi_routine_import_stubs", path)
    helper = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(helper)
    device, _ = helper._load_device(monkeypatch)
    monkeypatch.setattr(device, "_get_control_cmd", lambda name: name)
    monkeypatch.setattr(device, "_get_default_cmd", lambda: "DEFAULT")
    high = High()
    loco = device.LocoPlugin({}, "test_bumi", None, high)

    def edge(command, expected_modes, index=0, timeout_s=2):
        loco._publish_cmd(0, 0, 0, command, index)
        loco._publish_cmd(0, 0, 0, "DEFAULT", 0)
        return high.get_mode()

    monkeypatch.setattr(loco, "_send_edge_and_wait", edge)
    return high, loco


@pytest.fixture
def card(tmp_path, hardware):
    library = json.loads((BUMI / "resource/exercise_motions.json").read_text())
    # These are *test fixtures*, not bundled hardware recordings or evidence.
    for index, motion in enumerate(library["motions"]):
        motion.update(verified=True, recording_id=41 + index, robot_id="mock-robot",
                      duration_s=0.02, timeout_s=2, verification_notes="FAKE HARDWARE TEST ONLY")
    path = tmp_path / "fake_library.json"
    path.write_text(json.dumps(library))
    terminal = []
    instance = routine.ExerciseRoutinePlugin(
        {"library_path": str(path), "robot_id": "mock-robot", "dry_run_clip_s": 0.01}, hardware[1],
        on_terminal=lambda *args: terminal.append(args))
    instance.terminal = terminal
    yield instance
    hardware[0].read_error = False
    hardware[0].exit_ok = True
    hardware[0].mode = 2
    instance.stop()


def run(card, p=None, **extra):
    return card.dispatch("run", {"plan": p or plan(), "dry_run": False, "confirm": True, **extra})


def confirm(card):
    status = wait_for(card, "awaiting_confirmation")
    return card.dispatch("continue", {"job_id": status["job_id"],
                                      "confirmation_token": status["confirmation_token"], "confirm": True})


def test_bundled_candidates_are_unbound_and_real_run_is_rejected(hardware):
    c = routine.ExerciseRoutinePlugin({}, hardware[1])
    info = c.dispatch("info", {})
    assert len(info["motions"]) == 3
    assert all(m["verified"] is False and m["recording_id"] is None for m in info["motions"])
    result = run(c)
    assert result["state"] == "error" and "不能用于真机" in result["error"]
    assert hardware[0].calls == [] and hardware[0].reads == 0


def test_validate_never_reads_or_commands_hardware(card, hardware):
    preview = card.dispatch("validate", {"plan": plan(), "dry_run": False})
    assert preview["ok"] and preview["estimated_duration_s"] == 0.02
    assert hardware[0].calls == [] and hardware[0].reads == 0


def test_pending_candidates_can_be_previewed_and_simulated_without_hardware():
    c = routine.ExerciseRoutinePlugin({"dry_run_clip_s": 0.001}, None, on_terminal=lambda *_args: None)
    preview = c.dispatch("validate", {"plan": plan()})
    assert preview["ok"] and preview["warnings"]
    assert preview["estimated_duration_s"] is None
    c.dispatch("run", {"plan": plan(rounds=2)})
    assert finish(c)["completed_plays"] == 2


@pytest.mark.parametrize("value", [True, 0, -1, 21, 1.2, "2", None])
def test_invalid_rounds(card, value):
    result = card.dispatch("validate", {"plan": plan(rounds=value)})
    assert not result["ok"]


@pytest.mark.parametrize("field,value", [
    ("play_count", True), ("play_count", 0), ("play_count", 21), ("play_count", 1.2),
    ("rest_s", -1), ("rest_s", float("nan")), ("rest_s", float("inf")),
    ("rest_s", True), ("rest_s", "1"), ("rest_s", 601),
    ("motion", "missing"), ("motion", []), ("typo", 1),
])
def test_invalid_steps(card, field, value):
    p = plan()
    p["steps"][0][field] = value
    assert card.dispatch("validate", {"plan": p})["state"] == "error"


@pytest.mark.parametrize("bad", [None, [], {}, {"steps": []}, {"steps": "x"},
                                    {"steps": [{}]}, {"steps": [None]}, {"steps": [], "typo": 1}])
def test_invalid_plan_shapes(card, bad):
    assert card.dispatch("validate", {"plan": bad})["state"] == "error"


@pytest.mark.parametrize("value", ["yes", "0", 1, None, {}])
def test_non_boolean_dry_run_is_rejected(card, value):
    assert card.dispatch("validate", {"plan": plan(), "dry_run": value})["state"] == "error"


def test_play_and_time_limits(card):
    assert not card.dispatch("validate", {"plan": plan(rounds=20, count=20)})["ok"]
    p = plan(rounds=2, motions=("chest_expand", "arms_up"))
    assert not card.dispatch("validate", {"plan": {**p, "round_rest_s": float("nan")}})["ok"]
    assert not card.dispatch("validate", {"plan": {**p, "steps": p["steps"] * 11}})["ok"]
    card._library["chest_expand"]["timeout_s"] = 600
    assert not card.dispatch("validate", {"plan": plan(count=6)})["ok"]


@pytest.mark.parametrize("field,value", [
    ("verified", False), ("recording_id", None), ("recording_id", True),
    ("recording_id", 65536), ("robot_id", "another-robot"),
    ("completion_strategy", "timer"), ("verification_notes", ""),
    ("timeout_s", None), ("timeout_s", 0), ("duration_s", float("nan")),
    ("internal_repetitions", True),
])
def test_invalid_physical_asset_is_rejected(card, hardware, field, value):
    card._library["chest_expand"][field] = value
    assert run(card)["state"] == "error"
    assert not hardware[0].calls


def test_real_run_requires_explicit_confirmation(card, hardware):
    assert run(card, confirm=False)["state"] == "error"
    assert not hardware[0].calls


def test_unsupported_pose_never_causes_automatic_enable_or_get_up(card, hardware):
    hardware[0].mode = 0
    assert run(card)["state"] == "error"
    assert not hardware[0].calls


def test_order_repeats_rounds_and_rest_boundaries(card, hardware, monkeypatch):
    rests = []
    def wait(job, seconds, **kwargs):
        card._guard(job)
        rests.append(seconds)
    monkeypatch.setattr(card, "_wait", wait)
    p = {"rounds": 2, "round_rest_s": 0.7, "steps": [
        {"motion": "chest_expand", "play_count": 2, "rest_s": 0.1},
        {"motion": "arms_up", "play_count": 1, "rest_s": 0.2},
        {"motion": "arms_forward_push", "play_count": 1, "rest_s": 0.9}]}
    preview = card.dispatch("validate", {"plan": p, "dry_run": False})
    assert preview["total_plays"] == 8
    assert preview["total_rest_s"] == pytest.approx(1.5)
    run(card, p)
    result = finish(card)
    assert result["state"] == "completed" and result["completed_plays"] == 8
    assert [c[1] for c in hardware[0].calls if c[0] == "PLAYTEACH"] == [41, 41, 42, 43] * 2
    assert rests == [0.1, 0.1, 0.2, 0.7, 0.1, 0.1, 0.2]
    assert result["playback_exit_confirmed"] is True
    assert len(card.terminal) == 1


def test_start_and_walking_mode_never_imply_physical_completion(card, hardware):
    run(card)
    wait_for(card, "awaiting_confirmation")
    assert not [c for c in hardware[0].calls if c[0] == "WALK"]
    hardware[0].mode = 2
    time.sleep(0.08)
    assert card.dispatch("status", {})["state"] == "awaiting_confirmation"
    assert finish(card)["state"] == "completed"


def test_duplicate_run_and_invalid_or_stale_confirmations(card):
    run(card)
    status = wait_for(card, "awaiting_confirmation")
    assert run(card)["state"] == "busy"
    assert card.dispatch("continue", {"confirm": True})["ok"] is False
    assert card.dispatch("continue", {"job_id": status["job_id"], "confirm": True,
                                      "confirmation_token": "old"})["ok"] is False
    assert confirm(card)["ok"]
    wait_for(card, routine.TERMINAL)
    assert card.dispatch("continue", {"job_id": status["job_id"], "confirm": True,
                                      "confirmation_token": status["confirmation_token"]})["ok"] is False
    assert card.dispatch("status", {"job_id": "unknown"})["ok"] is False


def test_stop_during_playback_exits_but_does_not_count_a_complete_clip(card, hardware):
    run(card, plan(count=2))
    wait_for(card, "awaiting_confirmation")
    assert card.dispatch("stop", {})["state"] == "stopping"
    result = wait_for(card, routine.TERMINAL)
    assert result["state"] == "cancelled" and result["completed_plays"] == 0
    assert result["playback_exit_confirmed"] is True
    assert len([c for c in hardware[0].calls if c[0] == "PLAYTEACH"]) == 1


def test_stop_during_rest_interrupts_wait_and_prevents_next_clip(card, hardware):
    run(card, plan(count=2, rest=5))
    confirm(card)
    status = wait_for(card, "resting")
    assert status["rest_remaining_s"] > 4
    start = time.monotonic()
    card.dispatch("stop", {})
    assert wait_for(card, routine.TERMINAL)["state"] == "cancelled"
    assert time.monotonic() - start < 0.5
    assert len([c for c in hardware[0].calls if c[0] == "PLAYTEACH"]) == 1


def test_stop_racing_exit_cannot_schedule_next_clip(card, hardware):
    run(card, plan(count=2))
    wait_for(card, "awaiting_confirmation")
    with hardware[1]._action_lock:
        confirm(card)
        card.dispatch("stop", {})
    assert wait_for(card, routine.TERMINAL)["state"] == "cancelled"
    assert len([c for c in hardware[0].calls if c[0] == "PLAYTEACH"]) == 1


def test_external_same_mode_playback_invalidates_old_job(card, hardware):
    run(card, plan(count=2))
    wait_for(card, "awaiting_confirmation")
    result = hardware[1].dispatch("play_recording", {"_tool_name": "action_recording", "recording_id": 99})
    assert result["confirmed_started"]
    status = wait_for(card, routine.TERMINAL)
    assert status["state"] == "cancelled" and status["cancel_reason"].endswith("play_recording")
    calls = [c[0:2] for c in hardware[0].calls if c[0] != "DEFAULT"]
    assert calls == [("PLAYTEACH", 41), ("WALK", 0), ("PLAYTEACH", 99)]
    time.sleep(0.08)
    assert hardware[0].mode == 23  # Old worker must not stop the NEW playback.
    assert [c[0:2] for c in hardware[0].calls if c[0] != "DEFAULT"] == calls


def test_external_lifecycle_stop_cancels_routine(card, hardware):
    run(card, plan(count=2))
    wait_for(card, "awaiting_confirmation")
    hardware[1].dispatch("stop", {"_tool_name": "semantic_action"})
    assert wait_for(card, routine.TERMINAL)["state"] == "cancelled"
    assert len([c for c in hardware[0].calls if c[0] == "PLAYTEACH"]) == 1


def test_streaming_writes_are_blocked_during_body_lease(card, hardware):
    run(card)
    wait_for(card, "awaiting_confirmation")
    before = list(hardware[0].calls)
    hardware[1].publish_velocity(1, 0, 0)
    hardware[1].publish_velocity(0, 0, 0)
    assert hardware[0].calls == before
    from loco_servo import LocoServoPlugin
    servo = LocoServoPlugin({}, "bumi", None, hardware[0], loco_plugin=hardware[1])
    assert "编排" in servo._chassis_conflict()


@pytest.mark.parametrize("problem", ["protection", "read_failure", "unexpected_mode"])
def test_bad_state_aborts_without_forcing_walk(card, hardware, problem):
    run(card, plan(count=2))
    wait_for(card, "awaiting_confirmation")
    if problem == "read_failure":
        hardware[0].read_error = True
    else:
        hardware[0].mode = 26 if problem == "protection" else 11
    status = wait_for(card, "error")
    assert status["completed_plays"] == 0 and status["error"]
    assert not [c for c in hardware[0].calls if c[0] == "WALK"]
    assert status["control_reserved"]  # An unconfirmed exit cannot silently release the body.


def test_unconfirmed_start_never_proceeds_to_another_clip(card, hardware):
    hardware[0].start_ok = False
    run(card, plan(count=2))
    assert wait_for(card, "error")["completed_plays"] == 0
    assert len([c for c in hardware[0].calls if c[0] == "PLAYTEACH"]) == 1


def test_unconfirmed_exit_blocks_following_motion_and_can_be_retried(card, hardware):
    run(card, plan(count=2))
    wait_for(card, "awaiting_confirmation")
    hardware[0].exit_ok = False
    confirm(card)
    status = wait_for(card, "error")
    assert status["playback_exit_confirmed"] is False and status["control_reserved"]
    result = hardware[1].dispatch("play_recording", {"_tool_name": "action_recording", "recording_id": 99})
    assert result["code"] == "ROUTINE_EXIT_UNCONFIRMED"
    assert [c[1] for c in hardware[0].calls if c[0] == "PLAYTEACH"] == [41]
    hardware[0].exit_ok = True
    card.dispatch("stop", {})
    assert not card.dispatch("status", {})["control_reserved"]
    card._job["thread"].join(timeout=1)
    assert len(card.terminal) == 1


def test_confirmation_timeout_is_error_not_success(card):
    card._library["chest_expand"]["timeout_s"] = 0.08
    run(card, plan(count=2))
    assert wait_for(card, "error")["completed_plays"] == 0


def test_task_deadline_is_enforced(card):
    run(card)
    wait_for(card, "awaiting_confirmation")
    card._job["deadline"] = time.monotonic() - 1
    assert "整套流程超时" in wait_for(card, "error")["error"]


def test_library_snapshot_cannot_change_running_plan(card, hardware):
    run(card, plan(count=2))
    wait_for(card, "awaiting_confirmation")
    card._library["chest_expand"]["recording_id"] = 99
    assert finish(card)["state"] == "completed"
    assert [c[1] for c in hardware[0].calls if c[0] == "PLAYTEACH"] == [41, 41]


def test_driver_shutdown_joins_worker_and_never_restores_plan(card, hardware):
    run(card, plan(count=2))
    wait_for(card, "awaiting_confirmation")
    result = card.stop()
    assert result["state"] == "cancelled"
    assert not card._job["thread"].is_alive()
    assert not hardware[1].is_motion_reserved()
    assert run(card)["state"] == "error"


def test_invalid_library_fails_closed(tmp_path):
    path = tmp_path / "broken.json"
    path.write_text('{"schema_version": 1, "motions": [{"id":"duplicate"}]}')
    c = routine.ExerciseRoutinePlugin({"library_path": str(path)}, None)
    assert c.dispatch("info", {})["library_error"]
    assert c.dispatch("run", {"plan": plan()})["state"] == "error"


def test_single_card_workbench_has_controls_and_body_resources(card):
    schema = card.get_tool()
    assert schema["type"] == "actuator" and not schema["multiInstance"]
    assert len(card.get_tools()) == 1
    spec = schema["inputSchema"]["x-workbench"]
    assert spec["protocol"] == "motus.workbench/1" and spec["entry"] == "ui"
    assert {"continue", "stop", "info", "run_plan"} <= set(spec["actions"])
    assert schema["inputSchema"]["x-completion"] == {"actions": ["run", "run_plan"], "timeout": 3615}
    assert set(schema["inputSchema"]["x-resource"]) == {"base", "leg", "arm_l", "arm_r"}
    assert schema["inputSchema"]["x-is-dangerous"] is True
    assert schema["inputSchema"]["properties"]["dry_run"]["default"] is True
    assert "plan" not in schema["inputSchema"]["x-action-params"]["run"]["params"]
    assert card.dispatch("start", {})["state"] == "ready"
    assert card.dispatch("ui", {})["protocol"] == spec["protocol"]


@pytest.mark.parametrize("field", ["rest_s", "round_rest_s"])
def test_enormous_numeric_input_is_a_parameter_error(card, field):
    p = plan()
    target = p if field == "round_rest_s" else p["steps"][0]
    target[field] = 10 ** 400
    assert card.dispatch("validate", {"plan": p})["state"] == "error"


def test_run_does_not_queue_behind_an_external_body_action(card, hardware):
    locked, release = threading.Event(), threading.Event()
    def other_action():
        with hardware[1]._action_lock:
            locked.set()
            release.wait(timeout=2)
    thread = threading.Thread(target=other_action)
    thread.start()
    assert locked.wait(timeout=1)
    try:
        start = time.monotonic()
        result = run(card)
        assert time.monotonic() - start < 0.2
        assert result["state"] == "error" and "其他动作" in result["error"]
        assert not hardware[0].calls
    finally:
        release.set()
        thread.join(timeout=1)


def test_external_command_before_worker_starts_invalidates_job(card, hardware, monkeypatch):
    entered, release = threading.Event(), threading.Event()
    original = card._worker
    def delayed(job):
        entered.set()
        release.wait(timeout=2)
        original(job)
    monkeypatch.setattr(card, "_worker", delayed)
    run(card)
    assert entered.wait(timeout=1)
    try:
        hardware[1].dispatch("stop_move", {"_tool_name": "loco"})
    finally:
        release.set()
    assert wait_for(card, "cancelled")["completed_plays"] == 0
    assert not [c for c in hardware[0].calls if c[0] == "PLAYTEACH"]


def test_unknown_direct_query_does_not_preempt_a_routine(card, hardware):
    run(card)
    wait_for(card, "awaiting_confirmation")
    before = list(hardware[0].calls)
    assert hardware[1].dispatch("info", {"_tool_name": "action_recording"}) is None
    assert hardware[0].calls == before
    assert card.dispatch("status", {})["state"] == "awaiting_confirmation"


@pytest.mark.parametrize("tool,action", [("loco", "stop_move"),
                                         ("stand_up_lie_prone", "lie_prone"),
                                         ("semantic_action", "wave"),
                                         ("action_recording", "start_recording")])
def test_each_direct_body_entry_preempts_old_scheduler(card, hardware, tool, action):
    run(card, plan(count=2))
    wait_for(card, "awaiting_confirmation")
    result = hardware[1].dispatch(action, {"_tool_name": tool})
    assert result is not None
    assert wait_for(card, "cancelled")["completed_plays"] == 0
    assert [c[1] for c in hardware[0].calls if c[0] == "PLAYTEACH"] == [41]


def test_failed_initial_state_read_is_structured_error(card, hardware):
    hardware[0].read_error = True
    result = run(card)
    assert result["state"] == "error" and "无法读取" in result["error"]
    assert not hardware[0].calls
    assert not hardware[1].is_motion_reserved()


def test_thread_start_failure_releases_body_and_shutdown_is_safe(card, hardware, monkeypatch):
    def fail(_thread):
        raise RuntimeError("test thread failure")
    monkeypatch.setattr(threading.Thread, "start", fail)
    result = run(card)
    assert result["state"] == "error" and "线程" in result["error"]
    assert "action_id" not in result
    assert not hardware[1].is_motion_reserved()
    assert card.stop()["state"] == "error"


def test_last_steps_repetitions_rest_only_between_plays_and_rounds(card, monkeypatch):
    rests = []
    monkeypatch.setattr(card, "_wait", lambda job, seconds, **kw: rests.append(seconds))
    run(card, plan(rounds=2, count=3, rest=0.1, round_rest=0.7))
    assert finish(card)["state"] == "completed"
    assert rests == [0.1, 0.1, 0.7, 0.1, 0.1]


def test_old_token_cannot_confirm_next_clip(card):
    run(card, plan(count=2))
    first = wait_for(card, "awaiting_confirmation")
    confirm(card)
    deadline = time.monotonic() + 1
    while time.monotonic() < deadline:
        second = card.dispatch("status", {})
        if second["state"] == "awaiting_confirmation" and second["confirmation_token"] != first["confirmation_token"]:
            break
        time.sleep(0.002)
    else:
        pytest.fail("second clip not reached")
    assert not card.dispatch("continue", {"job_id": first["job_id"], "confirm": True,
                                          "confirmation_token": first["confirmation_token"]})["ok"]
    assert finish(card)["state"] == "completed"



def test_new_clip_does_not_inherit_previous_exit_confirmation(card):
    run(card, plan(count=2))
    first = wait_for(card, "awaiting_confirmation")
    assert first["playback_exit_confirmed"] is None
    confirm(card)
    deadline = time.monotonic() + 1
    while time.monotonic() < deadline:
        second = card.dispatch("status", {})
        if second["state"] == "awaiting_confirmation" and second["play_index"] == 2:
            assert second["playback_exit_confirmed"] is None
            break
        time.sleep(0.002)
    else:
        pytest.fail("second clip not reached")
    assert finish(card)["playback_exit_confirmed"] is True


def test_stop_state_cannot_be_overwritten_by_a_late_worker_update(card):
    run(card)
    wait_for(card, "awaiting_confirmation")
    card.dispatch("stop", {})
    with card._lock:
        if not card._job["finished"]:
            card._update(card._job, state="running")
            assert card.dispatch("status", {})["state"] == "stopping"
    assert wait_for(card, routine.TERMINAL)["state"] == "cancelled"



@pytest.mark.parametrize("outcome", ["completed", "cancelled", "error"])
def test_acp_action_id_and_exactly_one_terminal_result(card, hardware, outcome):
    response = run(card)
    assert response["action_id"] == response["job_id"]
    wait_for(card, "awaiting_confirmation")
    if outcome == "completed":
        result = finish(card)
    elif outcome == "cancelled":
        card.dispatch("stop", {"_tool_name": "exercise_routine_control"})
        result = wait_for(card, "cancelled")
    else:
        hardware[0].mode = 26
        result = wait_for(card, "error")
    card._job["thread"].join(timeout=1)
    assert result["state"] == outcome
    assert len(card.terminal) == 1
    action_id, state, terminal = card.terminal[0]
    assert action_id == response["action_id"] and state == outcome
    assert terminal["action_id"] == action_id and terminal["state"] == outcome
    hardware[0].mode = 2
    card.dispatch("stop", {"_tool_name": "exercise_routine_control"})
    assert len(card.terminal) == 1


def test_operator_surface_cannot_start_new_motion_but_can_complete_lease(card, hardware):
    response = card.dispatch("run", {"_tool_name": "exercise_routine_control", "plan": plan(), "confirm": True})
    assert response["state"] == "error" and not hardware[0].calls
    response = run(card)
    status = wait_for(card, "awaiting_confirmation")
    control_status = card.dispatch("status", {"_tool_name": "exercise_routine_control"})
    assert control_status["action_id"] == response["action_id"]
    confirmed = card.dispatch("continue", {"_tool_name": "exercise_routine_control",
        "job_id": status["job_id"], "confirmation_token": status["confirmation_token"], "confirm": True})
    assert confirmed["ok"] and wait_for(card, "completed")["completed_plays"] == 1
    card._job["thread"].join(timeout=1)


def test_declared_interrupt_hook_cancels_pending_body_action(card):
    run(card, plan(count=2))
    wait_for(card, "awaiting_confirmation")
    hooks = card.get_tool()["inputSchema"]["x-hooks"]
    assert hooks["on_interrupt_motion"]["action"] == "stop"
    assert hooks["on_interrupt_all"]["action"] == "stop"
    card.dispatch(hooks["on_interrupt_motion"]["action"], {"_tool_name": "exercise_routine"})
    assert wait_for(card, "cancelled")["completed_plays"] == 0


def test_production_acp_transport_posts_correlated_terminal_once(card, monkeypatch):
    import io
    import urllib.request
    captured = []
    monkeypatch.setenv("AGENT_CORE_URL", "http://mock-agent-core.invalid:15678/")
    def urlopen(request, **kwargs):
        captured.append((request, kwargs))
        return io.BytesIO(b'{}')
    monkeypatch.setattr(urllib.request, "urlopen", urlopen)
    card._on_terminal = routine.notify_completion
    response = run(card)
    assert finish(card)["state"] == "completed"
    assert len(captured) == 1
    request, options = captured[0]
    payload = json.loads(request.data)
    assert request.full_url == "http://mock-agent-core.invalid:15678/api/acp/complete"
    assert request.method == "POST" and options["timeout"] == 5
    assert payload["action_id"] == response["action_id"]
    assert payload["status"] == "completed" and payload["tool"] == "exercise_routine"
    assert payload["result"]["completed_plays"] == 1


def test_acp_transport_failure_does_not_rewrite_terminal_or_retry(card, monkeypatch, capsys):
    import urllib.request
    attempts = []
    def unavailable(*args, **kwargs):
        attempts.append(1)
        raise OSError("mock Agent Core unavailable")
    monkeypatch.setattr(urllib.request, "urlopen", unavailable)
    card._on_terminal = routine.notify_completion
    run(card)
    assert finish(card)["state"] == "completed"
    assert attempts == [1]
    assert "ACP callback failed" in capsys.readouterr().out



@pytest.mark.parametrize("url,local_exception", [
    ("https://localhost:15678", True),
    ("https://localhost:15678/", True),
    ("https://core.example.invalid:15678", False),
    ("https://localhost:15679", False),
    ("https://localhost:15678/other", False),
    ("https://localhost.example.invalid:15678", False),
    ("http://localhost:15678", False),
])
def test_acp_tls_exception_is_only_for_exact_local_default(monkeypatch, url, local_exception):
    import io
    import ssl
    import urllib.request
    contexts = []
    exception_context = object()
    monkeypatch.setenv("AGENT_CORE_URL", url)
    monkeypatch.setattr(ssl, "_create_unverified_context", lambda: exception_context)
    def urlopen(_request, **options):
        contexts.append(options["context"])
        return io.BytesIO(b'{}')
    monkeypatch.setattr(urllib.request, "urlopen", urlopen)
    routine.notify_completion("tls-test", "completed", {})
    assert contexts == [exception_context if local_exception else None]


@pytest.mark.parametrize("canvas_text", [False, True])
@pytest.mark.parametrize("action", ["run", "run_plan"])
def test_http_fast_completion_waits_for_run_response_and_pending_registration(card, monkeypatch, canvas_text, action):
    """Force completion while the *real* MCP socket response write is blocked."""
    import ast
    import urllib.request
    from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
    source = ast.parse((BUMI / "main.py").read_text())
    selected = [node for node in source.body if isinstance(node, (ast.ClassDef, ast.FunctionDef))
                and node.name in {"BumiDeviceBundle", "make_handler"}]
    namespace = {"BaseHTTPRequestHandler": BaseHTTPRequestHandler, "json": json}
    tree = ast.Module(body=[ast.ImportFrom(module="__future__", names=[ast.alias(name="annotations")], level=0)] + selected,
                      type_ignores=[])
    exec(compile(ast.fix_missing_locations(tree), str(BUMI / "main.py"), "exec"), namespace)
    bundle = object.__new__(namespace["BumiDeviceBundle"])
    bundle._plugins = [card]
    namespace["_bundle"] = bundle
    blocked, release = threading.Event(), threading.Event()
    pending, received, client_results, client_errors = set(), [], [], []

    class CoreHandler(BaseHTTPRequestHandler):
        def log_message(self, *_args):
            pass
        def do_POST(self):
            payload = json.loads(self.rfile.read(int(self.headers["Content-Length"])))
            registered = payload["action_id"] in pending
            received.append((payload, registered))
            self.send_response(200 if registered else 409)
            self.end_headers()
            self.wfile.write(b'{}')

    class GatedHandler(namespace["make_handler"]()):
        def log_message(self, *_args):
            pass
        def _send(self, status, body):
            if json.loads(body).get("id") == "fast-run":
                blocked.set()
                if not release.wait(timeout=3):
                    raise OSError("test did not release response")
            super()._send(status, body)

    core = ThreadingHTTPServer(("127.0.0.1", 0), CoreHandler)
    mcp = ThreadingHTTPServer(("127.0.0.1", 0), GatedHandler)
    servers = [core, mcp]
    threads = [threading.Thread(target=s.serve_forever, kwargs={"poll_interval": 0.01}, daemon=True) for s in servers]
    for thread in threads:
        thread.start()
    monkeypatch.setenv("AGENT_CORE_URL", f"http://127.0.0.1:{core.server_port}")
    card._on_terminal = routine.notify_completion
    card._sim_clip_s = 0.001
    request = urllib.request.Request(f"http://127.0.0.1:{mcp.server_port}/mcp",
        data=json.dumps({"jsonrpc": "2.0", "id": "fast-run", "method": "tools/call",
                         "params": {"name": "exercise_routine", "arguments": {
                             "action": action, "dry_run": "true" if canvas_text else True,
                             "confirm": "false" if canvas_text else False,
                             "plan": json.dumps(plan()) if canvas_text else plan()}}}).encode(),
        headers={"Content-Type": "application/json"})
    def call():
        try:
            with urllib.request.urlopen(request, timeout=3) as response:
                message = json.loads(response.read())
            result = json.loads(message["result"]["content"][0]["text"])
            pending.add(result["action_id"])
            client_results.append(result)
        except Exception as exc:
            client_errors.append(exc)
    client = threading.Thread(target=call, daemon=True)
    try:
        client.start()
        assert blocked.wait(timeout=2)
        wait_for(card, "completed")
        assert not received  # Completion occurred before HTTP response; no ACP yet.
        assert not card._job["notification_done"].is_set()
        assert run(card)["state"] == "busy"  # Old publication cannot be orphaned by another job.
        release.set()
        client.join(timeout=2)
        assert not client_errors and len(client_results) == 1
        card._job["thread"].join(timeout=2)
        assert len(received) == 1 and received[0][1] is True
        assert received[0][0]["action_id"] == client_results[0]["action_id"]
        assert received[0][0]["status"] == "completed"
    finally:
        release.set()
        client.join(timeout=2)
        card.stop()
        for server in servers:
            server.shutdown()
            server.server_close()
        for thread in threads:
            thread.join(timeout=1)


def test_failed_response_cancels_task_without_sending_unregistered_acp(card):
    response = card.dispatch("run", {"plan": plan(count=2), "dry_run": False,
                                    "confirm": True, "_await_response_publication": True})
    wait_for(card, "awaiting_confirmation")
    card.response_published(response["action_id"], delivered=False)
    assert wait_for(card, "cancelled")["completed_plays"] == 0
    card._job["thread"].join(timeout=1)
    assert card.terminal == [] and card._job["notification_done"].is_set()


def test_shutdown_releases_worker_waiting_for_unpublished_http_response(card):
    card.dispatch("run", {"plan": plan(), "dry_run": True, "_await_response_publication": True})
    wait_for(card, "completed")
    assert not card._job["notification_done"].is_set()
    card.stop()
    assert not card._job["thread"].is_alive()
    assert card._job["notification_done"].is_set() and card.terminal == []



@pytest.mark.parametrize("text", ["true", " true ", "TRUE"])
def test_canvas_json_plan_and_boolean_text_validate(card, hardware, text):
    result = card.dispatch("validate", {"plan": json.dumps(plan()), "dry_run": text, "confirm": "false"})
    assert result["state"] == "validated" and result["dry_run"] is True
    assert hardware[0].calls == [] and hardware[0].reads == 0


def test_canvas_text_dry_run_executes_without_hardware(card, hardware):
    result = card.dispatch("run", {"plan": json.dumps(plan(rounds=2)), "dry_run": "true", "confirm": "false"})
    assert result["dry_run"] is True
    assert finish(card)["completed_plays"] == 2
    assert hardware[0].calls == [] and hardware[0].reads == 0


def test_blank_canvas_flags_default_to_dry_run_without_confirmation(card, hardware):
    result = card.dispatch("run", {"plan": json.dumps(plan()), "dry_run": " ", "confirm": ""})
    assert result["dry_run"] is True and finish(card)["state"] == "completed"
    assert not hardware[0].calls


def test_false_string_is_false_and_never_confirms_physical_motion(card, hardware):
    preview = card.dispatch("validate", {"plan": json.dumps(plan()), "dry_run": "false", "confirm": "false"})
    assert preview["ok"] and preview["dry_run"] is False
    result = card.dispatch("run", {"plan": json.dumps(plan()), "dry_run": "false", "confirm": "false"})
    assert result["state"] == "error" and not hardware[0].calls


@pytest.mark.parametrize("text", ["garbage", "[]", '"text"', "null", '{"rounds":1,}', "x" * 65537])
def test_bad_canvas_plan_text_is_rejected_before_hardware(card, hardware, text):
    result = card.dispatch("run", {"plan": text, "dry_run": "false", "confirm": "true"})
    assert result["state"] == "error" and not hardware[0].calls


def test_decoded_plan_still_rejects_unknown_keys_and_nested_wrong_types(card):
    for p in ({**plan(), "typo": 1}, plan(rounds="1"), plan(count=True)):
        assert not card.dispatch("validate", {"plan": json.dumps(p), "dry_run": "true"})["ok"]


def test_canvas_true_confirmation_only_applies_to_current_clip(card):
    run(card)
    status = wait_for(card, "awaiting_confirmation")
    args = {"_tool_name": "exercise_routine_control", "job_id": status["job_id"],
            "confirmation_token": status["confirmation_token"], "confirm": "false"}
    assert not card.dispatch("continue", args)["ok"]
    assert card.dispatch("continue", {**args, "confirm": "true"})["ok"]
    wait_for(card, "completed")
    card._job["thread"].join(timeout=1)


def test_blank_optional_canvas_job_id_queries_current_task(card):
    assert card.dispatch("status", {"job_id": " "})["state"] == "idle"
    run(card)
    status = wait_for(card, "awaiting_confirmation")
    assert card.dispatch("status", {"job_id": ""})["job_id"] == status["job_id"]


def test_flat_form_parameters_need_no_plan_and_validate_without_hardware(card, hardware):
    result = card.dispatch('validate', {'motion': 'chest_expand', 'rounds': '2',
        'play_count': '3', 'rest_s': '0.5', 'round_rest_s': '1', 'dry_run': 'false'})
    assert result['ok'] and result['rounds'] == 2 and result['total_plays'] == 6
    assert result['total_rest_s'] == 3
    assert not hardware[0].calls and hardware[0].reads == 0
    card.dispatch('run', {'motion': 'chest_expand', 'rounds': '2', 'dry_run': 'true'})
    assert finish(card)['exited_plays'] == 2
    assert not hardware[0].calls


@pytest.mark.parametrize('key,value', [('rounds', '1.5'), ('rounds', True), ('rounds', '21'),
    ('play_count', 0), ('play_count', 'false'), ('rest_s', 'NaN'), ('rest_s', True),
    ('rest_s', '-1'), ('round_rest_s', 'Infinity'), ('rounds', []), ('rest_s', {})])
def test_flat_form_rejects_invalid_numbers(card, hardware, key, value):
    result = card.dispatch('run', {'motion': 'chest_expand', key: value})
    assert result['state'] == 'error'
    assert not hardware[0].calls


def timed_plan(**kwargs):
    result = plan(**kwargs)
    for step in result['steps']:
        step['playback_s'] = 0.035
    return result


def test_timed_two_rounds_exit_each_clip_before_next_and_do_not_claim_physical_completion(card, hardware):
    preview = card.dispatch('validate_plan', {'plan': timed_plan(rounds=2, count=2, rest=0.01, round_rest=0.01),
                                             'completion_strategy': 'timed_exit', 'dry_run': False})
    assert preview['ok'] and preview['completion_strategy'] == 'timed_exit'
    assert preview['scheduled_duration_s'] == pytest.approx(0.17)
    result = run(card, timed_plan(rounds=2, count=2, rest=0.01, round_rest=0.01), completion_strategy='timed_exit')
    assert result['ok']
    # No human confirms a clip in this test: rounds advance using explicit windows.
    result = wait_for(card, 'completed')
    card._job['thread'].join(timeout=1)
    commands = [(c[0], c[1]) for c in hardware[0].calls if c[0] != 'DEFAULT']
    assert commands == [('PLAYTEACH', 41), ('WALK', 0)] * 4
    assert result['exited_plays'] == 4 and result['completed_plays'] == 0
    assert result['full_playback_confirmed'] is False
    assert result['playback_exit_confirmed'] is True and not result['control_reserved']
    assert len(card.terminal) == 1 and card.terminal[0][1] == 'completed'


def test_timed_mixed_sequence_uses_each_steps_window(card, hardware):
    p = timed_plan(rounds=2, motions=('chest_expand', 'arms_up'))
    p['steps'][1]['playback_s'] = 0.06
    run(card, p, completion_strategy='timed_exit')
    result = wait_for(card, 'completed')
    assert result['exited_plays'] == 4
    assert [c[1] for c in hardware[0].calls if c[0] == 'PLAYTEACH'] == [41, 42, 41, 42]
    assert [c[0] for c in hardware[0].calls if c[0] != 'DEFAULT'] == ['PLAYTEACH', 'WALK'] * 4


@pytest.mark.parametrize('value', [None, 0, -1, True, '10', float('nan'), float('inf'), 601])
def test_timed_window_requires_explicit_positive_finite_seconds(card, hardware, value):
    p = timed_plan()
    p['steps'][0]['playback_s'] = value
    result = run(card, p, completion_strategy='timed_exit')
    assert result['state'] == 'error' and not hardware[0].calls


def test_reported_ten_seconds_is_never_inferred_as_playback_window(card, hardware):
    card._library['chest_expand'].update(duration_s=None, reported_duration_s=10)
    result = run(card, completion_strategy='timed_exit')
    assert result['state'] == 'error' and 'playback_s' in result['error']
    assert not hardware[0].calls


def test_timed_mode_keeps_asset_validation_and_explicit_start_confirmation(card, hardware):
    card._library['chest_expand']['verified'] = False
    assert run(card, timed_plan(), completion_strategy='timed_exit')['state'] == 'error'
    card._library['chest_expand']['verified'] = True
    assert run(card, timed_plan(), completion_strategy='timed_exit', confirm='false')['state'] == 'error'
    assert not hardware[0].calls


@pytest.mark.parametrize('value', ['timer', 'auto', '', None, [], {}])
def test_unknown_completion_strategies_fail_without_motion(card, hardware, value):
    assert run(card, completion_strategy=value)['state'] == 'error'
    assert not hardware[0].calls


def test_flat_timed_form_parses_window_but_keeps_real_run_manual_confirmation(card, hardware):
    result = card.dispatch('run', {'motion': 'chest_expand', 'rounds': '2',
        'completion_strategy': 'timed_exit', 'playback_s': '0.1', 'dry_run': 'false', 'confirm': 'true'})
    assert result['ok'] and wait_for(card, 'completed')['exited_plays'] == 2
    assert [c[0] for c in hardware[0].calls if c[0] != 'DEFAULT'] == ['PLAYTEACH', 'WALK'] * 2


@pytest.mark.parametrize('phase', ['playing_window', 'resting'])
def test_stop_during_timed_window_or_rest_prevents_all_following_plays(card, hardware, phase):
    p = timed_plan(count=3, rest=3)
    if phase == 'playing_window':
        p['steps'][0]['playback_s'] = 3
    run(card, p, completion_strategy='timed_exit')
    wait_for(card, phase)
    assert card.dispatch('info', {})['busy'] is True
    card.dispatch('cancel', {})
    result = wait_for(card, 'cancelled')
    assert len([c for c in hardware[0].calls if c[0] == 'PLAYTEACH']) == 1
    assert not result['control_reserved']
    assert result['full_playback_confirmed'] is False


def test_failed_timed_exit_blocks_replay_and_preserves_ownership_for_stop_retry(card, hardware):
    hardware[0].exit_ok = False
    run(card, timed_plan(rounds=2), completion_strategy='timed_exit')
    result = wait_for(card, 'error')
    assert result['exited_plays'] == 0 and result['control_reserved'] is True
    assert card.dispatch('info', {})['busy'] is True
    assert len([c for c in hardware[0].calls if c[0] == 'PLAYTEACH']) == 1
    assert not result['full_playback_confirmed']
    hardware[0].exit_ok = True
    assert card.dispatch('stop', {})['control_reserved'] is False


def test_protection_during_timed_window_never_forces_walk_or_starts_another_round(card, hardware):
    p = timed_plan(rounds=2)
    p['steps'][0]['playback_s'] = 3
    run(card, p, completion_strategy='timed_exit')
    wait_for(card, 'playing_window')
    hardware[0].mode = 26
    assert wait_for(card, 'error')['exited_plays'] == 0
    assert [c[0] for c in hardware[0].calls if c[0] != 'DEFAULT'] == ['PLAYTEACH']


def test_workbench_entry_and_polling_have_no_hardware_side_effects(card, hardware):
    result = card.dispatch('ui', {})
    assert len(result['html']) < 250000 and '<input' in result['html'] and 'type="range"' in result['html']
    assert card.dispatch('info', {})['busy'] is False
    assert not hardware[0].calls and hardware[0].reads == 0


def test_same_card_can_query_and_confirm_each_round_without_manual_stop_calls(card, hardware):
    run(card, plan(rounds=2))
    for expected_round in [1, 2]:
        status = wait_for(card, 'awaiting_confirmation')
        assert status['round'] == expected_round
        assert card.dispatch('info', {})['current']['job_id'] == status['job_id']
        assert confirm(card)['ok']
        if expected_round == 1:
            deadline = time.monotonic() + 1
            while card.dispatch('status', {})['round'] != 2 and time.monotonic() < deadline:
                time.sleep(0.002)
    result = wait_for(card, 'completed')
    assert result['completed_plays'] == 2 and result['full_playback_confirmed'] is True
    assert [c[0] for c in hardware[0].calls if c[0] != 'DEFAULT'] == ['PLAYTEACH', 'WALK'] * 2
