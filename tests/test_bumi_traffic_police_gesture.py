"""Deterministic host-only gesture contracts through the real teaching helpers."""
import ast
import importlib.util
import io
import json
from pathlib import Path
import threading
import time
import types
import sys

import pytest
import yaml

from noetix.bumi.tests import test_loco_destination as harness

ROOT = Path(__file__).resolve().parents[1]
BUMI = ROOT / "noetix" / "bumi"


def call(plugin, action, **args):
    return plugin.dispatch(action, {"_tool_name": "traffic_police_gesture", **args})


def wait_until(check, timeout=1):
    deadline = time.monotonic()+timeout
    while time.monotonic() < deadline:
        if check():
            return
        time.sleep(.002)
    pytest.fail("condition did not become true")


@pytest.fixture
def robot(monkeypatch):
    module, _ = harness._load_device(monkeypatch)
    import observation_safety
    monkeypatch.setattr(observation_safety, "SETTLE_INTERVAL_S", .001)
    monkeypatch.setattr(observation_safety, "SETTLE_SAMPLES", 2)
    monkeypatch.setattr(module, "_get_control_cmd", lambda name: name)
    monkeypatch.setattr(module, "_get_default_cmd", lambda: "DEFAULT")
    events, callbacks = [], []
    monkeypatch.setattr(module, "_acp_notify", lambda *args: callbacks.append(args))

    class Controller:
        mode = 2
        fault = 0
        speed = 0.

        def get_mode(self):
            return self.mode

        def get_imu_data(self):
            return types.SimpleNamespace(ori=[0., 0., 0., 1.], angular_vel=[0., 0., self.speed])

        def get_joint_state(self):
            return [types.SimpleNamespace(error=self.fault, vel=self.speed) for _ in range(21)]

        def publish_cmd(self, *args):
            # The actual SDK isn't imported. A nonzero chassis command fails immediately.
            assert args[:3] == (0, 0, 0)
            events.append(("zero_velocity", args))

    ctrl = Controller()
    plugin = module.LocoPlugin({"traffic_police_gesture": {
        "stop_recording_id": 101, "go_recording_id": 202,
        "playback_timeout_s": 5}}, "host_test", None, ctrl)
    plugin._observation.stop_timeout_s = .025

    def edge(command, expected_modes, index=0, timeout_s=2):
        events.append((command, index))
        ctrl.mode = {"PLAYTEACH": 23, "WALK": 2, "STARTTEACH": 11,
                     "SAVETEACH": 12, "SWING": 8}.get(command, ctrl.mode)
        return ctrl.mode

    monkeypatch.setattr(plugin, "_send_edge_and_wait", edge)
    yield types.SimpleNamespace(plugin=plugin, ctrl=ctrl, events=events,
                                callbacks=callbacks, module=module, edge=edge)
    ctrl.mode, ctrl.fault, ctrl.speed = 2, 0, 0.
    plugin._move_thread = None
    plugin._rotate_state["state"] = "idle"
    plugin._traffic_gesture.shutdown()
    delivery_worker = plugin._traffic_gesture._delivery.worker
    if delivery_worker:
        delivery_worker.join(1)
        wait_until(lambda: not delivery_worker.is_alive())


@pytest.mark.parametrize("gesture,recording", [("stop", 101), ("go", 202)])
def test_business_gesture_maps_config_to_the_real_teaching_path(robot, monkeypatch, gesture, recording):
    plugin = robot.plugin
    teaching = plugin._do_teaching_action
    calls = []

    def shared(action, args):
        calls.append((action, dict(args), plugin._body_action))
        return teaching(action, args)

    monkeypatch.setattr(plugin, "_do_teaching_action", shared)
    result = call(plugin, gesture, execute=True)
    assert result["state"] == "running" and result["gesture"] == gesture
    assert result["recording_id"] == recording and result["action_id"]
    assert calls == [("play_recording", {"recording_id": recording}, f"traffic_police_gesture.{gesture}")]
    assert ("PLAYTEACH", recording) in robot.events
    assert plugin.is_moving() and plugin._body_action == f"traffic_police_gesture.{gesture}"
    assert not result["physical_completion_confirmed"] and not robot.callbacks


@pytest.mark.parametrize("gesture", ["stop", "go"])
def test_missing_recording_is_named_and_never_falls_back(robot, gesture):
    robot.plugin._traffic_gesture.config[gesture+"_recording_id"] = None
    result = call(robot.plugin, gesture, execute=True)
    assert result["state"] == "error" and result["reason"] == "gesture_not_configured"
    assert result["missing_config"] == [gesture+"_recording_id"]
    assert not robot.events and not robot.callbacks and robot.plugin._body_action is None


@pytest.mark.parametrize("bad", [-1, 65536, 1.5, True, "101", [], float("nan")])
def test_invalid_recording_ids_are_rejected_before_sdk_calls(robot, bad):
    robot.plugin._traffic_gesture.config["stop_recording_id"] = bad
    result = call(robot.plugin, "stop", execute=True)
    assert result["reason"] == "invalid_recording_id" and not robot.events


@pytest.mark.parametrize("mode", [0, 1, 5, 11, 23, 26, 30])
def test_nonstanding_semantic_teaching_and_fault_modes_are_not_auto_prepared(robot, mode):
    robot.ctrl.mode = mode
    result = call(robot.plugin, "go", execute=True)
    assert result["state"] == "error" and not robot.events
    assert robot.plugin._body_action is None


def test_body_action_reservation_is_never_bypassed(robot):
    robot.plugin._body_action = "semantic_action.wave"
    result = call(robot.plugin, "stop", execute=True)
    assert result["reason"] == "body_action_busy"
    assert robot.plugin._body_action == "semantic_action.wave" and not robot.events
    robot.plugin._body_action = None


def test_recording_body_action_reservation_is_never_bypassed(robot):
    robot.plugin._body_action = "action_recording.start_recording"
    assert call(robot.plugin, "go", execute=True)["reason"] == "body_action_busy"
    assert not robot.events
    robot.plugin._body_action = None


def test_inflight_rotation_is_refused_without_cancelling_it(robot):
    robot.plugin._rotate_state["state"] = "running"
    result = call(robot.plugin, "go", execute=True)
    assert result["reason"] == "rotation_busy"
    assert not robot.plugin._rotate_stop_event.is_set() and not robot.events
    robot.plugin._rotate_state["state"] = "idle"


def test_translation_in_progress_is_not_preempted(robot):
    robot.plugin._move_thread = types.SimpleNamespace(is_alive=lambda: True)
    assert call(robot.plugin, "stop", execute=True)["reason"] == "chassis_busy"
    assert not robot.events
    robot.plugin._move_thread = None


@pytest.mark.parametrize("state", ["running", "waiting", "holding"])
def test_servo_control_conflict_does_not_preempt_the_policy(robot, state):
    robot.plugin._servo = types.SimpleNamespace(_info=lambda: {"state": state})
    assert call(robot.plugin, "stop", execute=True)["reason"] == "servo_conflict"
    assert not robot.events
    robot.plugin._servo = None


@pytest.mark.parametrize("first,second", [("stop", "stop"), ("stop", "go"), ("go", "stop"), ("go", "go")])
def test_repeated_and_alternating_gestures_do_not_queue_or_overlap(robot, first, second):
    first_result = call(robot.plugin, first, execute=True)
    second_result = call(robot.plugin, second, execute=True)
    assert second_result["state"] == "error" and "action_id" not in second_result
    assert sum(event[0] == "PLAYTEACH" for event in robot.events) == 1
    assert call(robot.plugin, "info")["action_id"] == first_result["action_id"]
    assert not robot.callbacks


@pytest.mark.parametrize("first,second", [("stop", "stop"), ("stop", "go"), ("go", "stop")])
def test_cancel_then_next_gesture_is_a_new_operation(robot, first, second):
    old = call(robot.plugin, first, execute=True)
    ended = call(robot.plugin, "cancel")
    assert ended["state"] == "idle" and ended["playback_exit_confirmed"]
    assert robot.plugin._body_action is None and not robot.plugin.is_moving()
    new = call(robot.plugin, second, execute=True)
    assert new["action_id"] != old["action_id"]
    wait_until(lambda: bool(robot.callbacks))
    assert robot.callbacks[0][0] == old["action_id"] and robot.callbacks[0][1] == "cancelled"
    assert sum(event[0] == "PLAYTEACH" for event in robot.events) == 2


def test_info_has_no_motion_or_feedback_side_effects(robot, monkeypatch):
    result = call(robot.plugin, "info")
    assert result["state"] == "idle" and result["active_gesture"] is None
    assert result["stop_configured"] and result["go_configured"]
    call(robot.plugin, "stop", execute=True)
    before = list(robot.events)
    original_mode = robot.ctrl.get_mode
    monkeypatch.setattr(robot.ctrl, "get_mode", lambda: pytest.fail("info must not read or change the controller"))
    info = call(robot.plugin, "info")
    assert info["active_gesture"] == "stop" and info["busy"]
    assert robot.events == before
    monkeypatch.setattr(robot.ctrl, "get_mode", original_mode)


def test_lifecycle_stop_never_plays_a_parking_gesture(robot):
    assert call(robot.plugin, "stop")["state"] == "idle"
    assert not robot.events
    assert call(robot.plugin, "go", execute=False)["reason"] == "execute_required"
    call(robot.plugin, "go", execute=True)
    ended = call(robot.plugin, "stop", instance_id="canvas-card")
    assert ended["state"] == "idle"
    assert [event[0] for event in robot.events] == ["PLAYTEACH", "WALK"]


def test_gesture_stop_does_not_call_chassis_stop_move(robot, monkeypatch):
    monkeypatch.setattr(robot.plugin, "_stop_move", lambda: pytest.fail("parking gesture isn't stop_move"))
    assert call(robot.plugin, "stop", execute=True)["state"] == "running"


def test_plain_cancel_and_legacy_stop_playback_share_the_same_exit_path(robot, monkeypatch):
    call(robot.plugin, "stop", execute=True)
    teaching = robot.plugin._do_teaching_action
    calls = []

    def record(action, args):
        calls.append(action)
        return teaching(action, args)

    monkeypatch.setattr(robot.plugin, "_do_teaching_action", record)
    result = robot.plugin.dispatch("stop_playback", {"_tool_name": "action_recording"})
    assert result["state"] == "idle" and calls == ["stop_playback"]


def test_gesture_excludes_raw_base_and_observation_motion(robot):
    call(robot.plugin, "stop", execute=True)
    before = list(robot.events)
    for tool, action, params in (("loco", "move", {"vx": .5}),
                                 ("loco_relative_move", "rotate", {"angle_deg": 5}),
                                 ("loco_relative_move", "move", {"x_m": .5, "y_m": 0}),
                                 ("semantic_action", "wave", {}),
                                 ("action_recording", "start_recording", {})):
        assert robot.plugin.dispatch(action, {"_tool_name": tool, **params})["state"] == "error"
    with pytest.raises(RuntimeError):
        robot.plugin.publish_velocity(1, 0, 0)
    assert robot.plugin._observation.begin()["state"] == "error"
    assert robot.events == before and robot.plugin._body_action == "traffic_police_gesture.stop"


def test_observation_session_must_be_released_before_a_gesture(robot):
    sid = robot.plugin._observation.begin()["session_id"]
    assert call(robot.plugin, "stop", execute=True)["reason"] == "observation_busy"
    robot.plugin._observation.end(sid)
    assert call(robot.plugin, "stop", execute=True)["state"] == "running"


def test_playback_exception_returns_sync_error_without_orphaning_acp(robot, monkeypatch):
    teaching = robot.plugin._do_teaching_action

    def fail_after_submission(action, args):
        if action == "play_recording":
            teaching(action, args)
            raise RuntimeError("SDK reply failed after PLAYTEACH")
        return teaching(action, args)

    monkeypatch.setattr(robot.plugin, "_do_teaching_action", fail_after_submission)
    result = call(robot.plugin, "stop", execute=True)
    assert result["state"] == "error" and result["playback_exit_confirmed"]
    assert robot.plugin._body_action is None
    assert "action_id" not in result and not robot.callbacks
    assert [e[0] for e in robot.events] == ["PLAYTEACH", "WALK"]


def test_watchdog_start_failure_exits_without_creating_an_acp_pending(robot, monkeypatch):
    def failed_start(_thread):
        raise RuntimeError("cannot start watchdog")

    monkeypatch.setattr(threading.Thread, "start", failed_start)
    result = call(robot.plugin, "go", execute=True)
    assert result["state"] == "error" and result["playback_exit_confirmed"]
    assert "action_id" not in result and not robot.callbacks
    assert robot.plugin._body_action is None
    assert [e[0] for e in robot.events] == ["PLAYTEACH", "WALK"]


def test_sync_failure_with_unconfirmed_exit_keeps_body_but_no_orphan_pending(robot, monkeypatch):
    teaching = robot.plugin._do_teaching_action

    def fail_start_and_exit(action, args):
        if action == "play_recording":
            teaching(action, args)
            raise RuntimeError("lost playback acknowledgement")
        return {"state": "error", "confirmed": False}

    monkeypatch.setattr(robot.plugin, "_do_teaching_action", fail_start_and_exit)
    result = call(robot.plugin, "stop", execute=True)
    assert result["state"] == "locked" and "action_id" not in result
    assert robot.plugin._body_action == "traffic_police_gesture.stop" and not robot.callbacks
    monkeypatch.setattr(robot.plugin, "_do_teaching_action", teaching)
    assert call(robot.plugin, "cancel")["playback_exit_confirmed"]
    assert robot.plugin._body_action is None and not robot.callbacks


def test_cancel_refusal_locks_reservation_until_a_later_confirmed_exit(robot, monkeypatch):
    accepted = call(robot.plugin, "go", execute=True)

    def no_exit(command, modes, index=0, timeout_s=2):
        robot.events.append((command, index))
        return 23

    monkeypatch.setattr(robot.plugin, "_send_edge_and_wait", no_exit)
    refused = call(robot.plugin, "cancel")
    assert refused["state"] == "locked" and not refused["playback_exit_confirmed"]
    assert robot.plugin._body_action == "traffic_police_gesture.go"
    assert call(robot.plugin, "go", execute=True)["state"] == "error"
    wait_until(lambda: bool(robot.callbacks))
    assert len(robot.callbacks) == 1 and robot.callbacks[0][:2] == (accepted["action_id"], "error")
    monkeypatch.setattr(robot.plugin, "_send_edge_and_wait", robot.edge)
    assert call(robot.plugin, "cancel")["state"] == "idle"
    wait_until(lambda: bool(robot.callbacks))
    assert len(robot.callbacks) == 1


def test_unstable_feedback_does_not_release_the_body(robot):
    call(robot.plugin, "go", execute=True)
    robot.ctrl.speed = .5
    assert call(robot.plugin, "cancel")["state"] == "locked"
    assert robot.plugin.is_moving()
    robot.ctrl.speed = 0.
    assert call(robot.plugin, "cancel")["state"] == "idle"


def test_repeated_cancel_produces_only_one_acp_terminal(robot):
    accepted = call(robot.plugin, "stop", execute=True)
    assert call(robot.plugin, "cancel")["state"] == "idle"
    for _ in range(3):
        assert call(robot.plugin, "cancel")["state"] == "idle"
    wait_until(lambda: bool(robot.callbacks))
    assert len(robot.callbacks) == 1
    wait_until(lambda: bool(robot.callbacks))
    assert robot.callbacks[0][:2] == (accepted["action_id"], "cancelled")
    wait_until(lambda: bool(robot.callbacks))
    assert robot.callbacks[0][3] == "traffic_police_gesture"
    wait_until(lambda: bool(robot.callbacks))
    assert robot.callbacks[0][2]["physical_completion_confirmed"] is False


def test_sdk_return_to_walking_is_not_a_physical_completed_event(robot):
    call(robot.plugin, "go", execute=True)
    robot.ctrl.mode = 2
    wait_until(lambda: call(robot.plugin, "info")["gesture_state"] == "awaiting_verification")
    assert call(robot.plugin, "info")["state"] == "ready"
    assert not robot.callbacks and robot.plugin._body_action
    assert call(robot.plugin, "info")["physical_completion_available"] is False
    assert call(robot.plugin, "cancel")["state"] == "idle"
    assert all(callback[1] != "completed" for callback in robot.callbacks)


def test_start_and_info_separate_lifecycle_from_idle_without_sdk_calls(robot, monkeypatch):
    monkeypatch.setattr(robot.ctrl, "get_mode", lambda: pytest.fail("start/info must not query SDK"))
    monkeypatch.setattr(robot.plugin, "_do_teaching_action", lambda *args: pytest.fail("start must not play/stop"))
    before = call(robot.plugin, "info")
    assert before["state"] == "idle" and before["gesture_state"] == "idle" and not before["started"]
    for _ in range(2):
        started = call(robot.plugin, "start", instance_id="canvas")
        info = call(robot.plugin, "info")
        assert started["state"] == info["state"] == "ready"
        assert info["started"] and info["gesture_state"] == "idle" and not info["busy"]
        assert info["active_gesture"] is None and info["action_id"] is None
    assert not robot.events and not robot.callbacks and robot.plugin._body_action is None


@pytest.mark.parametrize("gesture", ["stop", "go"])
def test_unconfigured_recordings_do_not_block_start_or_info(robot, gesture):
    robot.plugin._traffic_gesture.config.update(stop_recording_id=None, go_recording_id=None)
    assert any(t["name"] == "traffic_police_gesture" for t in robot.plugin.get_tools())
    assert call(robot.plugin, "start")["state"] == "ready"
    info = call(robot.plugin, "info")
    assert info["state"] == "ready" and info["gesture_state"] == "idle"
    assert not info["stop_configured"] and not info["go_configured"]
    result = call(robot.plugin, gesture, execute=True)
    assert result["reason"] == "gesture_not_configured"
    assert result["missing_config"] == [gesture+"_recording_id"]
    assert call(robot.plugin, "info")["state"] == "ready"
    assert not robot.events and robot.plugin._body_action is None


@pytest.mark.parametrize("gesture", ["stop", "go"])
@pytest.mark.parametrize("execute", [False, None, 0, 1, "true", "false", {}, []])
def test_started_card_still_requires_json_boolean_true(robot, gesture, execute):
    call(robot.plugin, "start")
    result = call(robot.plugin, gesture, execute=execute)
    assert result["state"] == "error" and result["reason"] == "execute_required"
    assert not robot.events and not robot.callbacks and robot.plugin._body_action is None
    assert call(robot.plugin, "info")["gesture_state"] == "idle"


def test_missing_execute_is_cleanup_for_stop_and_refused_for_go(robot):
    call(robot.plugin, "start")
    assert call(robot.plugin, "go")["reason"] == "execute_required"
    assert call(robot.plugin, "stop", instance_id="canvas")["state"] == "idle"
    info = call(robot.plugin, "info")
    assert info["state"] == "idle" and not info["started"] and info["gesture_state"] == "idle"
    assert not robot.events


@pytest.mark.parametrize("gesture", ["stop", "go"])
def test_info_keeps_running_gesture_separate_from_card_readiness(robot, gesture):
    call(robot.plugin, "start")
    running = call(robot.plugin, gesture, execute=True)
    info = call(robot.plugin, "info")
    assert info["state"] == "ready" and info["started"]
    assert info["gesture_state"] == "running" and info["busy"]
    assert info["active_gesture"] == gesture and info["action_id"] == running["action_id"]
    assert robot.plugin._body_action == "traffic_police_gesture."+gesture
    assert not info["physical_completion_confirmed"]


def test_cancel_keeps_card_ready_and_preserves_the_last_exit_receipt(robot):
    call(robot.plugin, "start")
    running = call(robot.plugin, "go", execute=True)
    ended = call(robot.plugin, "cancel")
    assert ended["state"] == "idle" and ended["playback_exit_confirmed"]
    info = call(robot.plugin, "info")
    assert info["state"] == "ready" and info["started"]
    assert info["gesture_state"] == "idle" and not info["busy"] and info["action_id"] is None
    assert info["last_result"]["action_id"] == running["action_id"]
    assert info["last_result"]["physical_completion_confirmed"] is False
    wait_until(lambda: bool(robot.callbacks))
    assert robot.callbacks[0][1] == "cancelled"


def test_lifecycle_stop_disarms_card_and_restart_is_motion_free(robot):
    call(robot.plugin, "start")
    call(robot.plugin, "go", execute=True)
    assert call(robot.plugin, "stop", instance_id="canvas")["state"] == "idle"
    info = call(robot.plugin, "info")
    assert info["state"] == "idle" and not info["started"] and not info["busy"]
    assert info["gesture_state"] == "idle" and robot.plugin._body_action is None
    assert [event[0] for event in robot.events] == ["PLAYTEACH", "WALK"]
    before = list(robot.events)
    assert call(robot.plugin, "start")["state"] == "ready"
    assert robot.events == before


def test_ready_does_not_bypass_the_existing_body_reservation(robot):
    robot.plugin._body_action = "semantic_action.wave"
    try:
        assert call(robot.plugin, "start")["state"] == "ready"
        assert call(robot.plugin, "go", execute=True)["reason"] == "body_action_busy"
        assert robot.plugin._body_action == "semantic_action.wave" and not robot.events
    finally:
        robot.plugin._body_action = None


def test_locked_playback_remains_a_card_error_and_start_cannot_hide_it(robot, monkeypatch):
    call(robot.plugin, "start")
    call(robot.plugin, "stop", execute=True)
    teaching = robot.plugin._do_teaching_action
    monkeypatch.setattr(robot.plugin, "_do_teaching_action", lambda *args: {"state": "error", "confirmed": False})
    assert call(robot.plugin, "cancel")["state"] == "locked"
    info = call(robot.plugin, "info")
    assert info["state"] == "error" and info["started"] and info["gesture_state"] == "locked"
    assert info["error"] and info["last_error"] and info["busy"]
    before = list(robot.events)
    assert call(robot.plugin, "start")["state"] == "error" and robot.events == before
    assert robot.plugin._body_action == "traffic_police_gesture.stop"
    monkeypatch.setattr(robot.plugin, "_do_teaching_action", teaching)
    assert call(robot.plugin, "cancel")["playback_exit_confirmed"]
    assert call(robot.plugin, "info")["state"] == "ready"


def test_playback_failure_reports_last_error_without_unstarting_the_card(robot, monkeypatch):
    call(robot.plugin, "start")
    teaching = robot.plugin._do_teaching_action
    def failed(action, args):
        if action == "play_recording":
            teaching(action, args)
            raise RuntimeError("lost playback acknowledgement")
        return teaching(action, args)
    monkeypatch.setattr(robot.plugin, "_do_teaching_action", failed)
    assert call(robot.plugin, "go", execute=True)["state"] == "error"
    info = call(robot.plugin, "info")
    assert info["state"] == "ready" and info["started"] and info["gesture_state"] == "idle"
    assert info["last_result"]["state"] == "error" and info["last_error"]
    assert robot.plugin._body_action is None


@pytest.mark.parametrize("stop_count", [1, 2])
def test_pending_lifecycle_stop_cannot_be_rearmed_concurrently(robot, monkeypatch, stop_count):
    call(robot.plugin, "start")
    call(robot.plugin, "go", execute=True)
    entered, release = threading.Event(), threading.Event()
    teaching = robot.plugin._do_teaching_action
    def delayed(action, args):
        if action == "stop_playback":
            entered.set()
            assert release.wait(2)
        return teaching(action, args)
    monkeypatch.setattr(robot.plugin, "_do_teaching_action", delayed)
    results = []
    threads = [threading.Thread(target=lambda: results.append(call(robot.plugin, "stop", instance_id="canvas")))
               for _ in range(stop_count)]
    try:
        for thread in threads: thread.start()
        assert entered.wait(2)
        wait_until(lambda: robot.plugin._traffic_gesture._stop_requests == stop_count)
        info = call(robot.plugin, "info")
        assert info["state"] == "stopping" and info["gesture_state"] == "stopping"
        assert not info["started"] and info["busy"]
        assert call(robot.plugin, "start")["reason"] == "lifecycle_stopping"
        assert call(robot.plugin, "go", execute=True)["state"] == "error"
    finally:
        release.set()
        for thread in threads: thread.join(2)
    assert all(not thread.is_alive() for thread in threads)
    assert len(results) == stop_count and all(result["state"] == "idle" for result in results)
    assert call(robot.plugin, "info")["state"] == "idle"
    assert [event[0] for event in robot.events] == ["PLAYTEACH", "WALK"]
    wait_until(lambda: bool(robot.callbacks))
    assert len(robot.callbacks) == 1


def test_disabled_or_shutdown_card_cannot_claim_ready(robot):
    gesture = robot.plugin._traffic_gesture
    gesture.enabled = False
    assert gesture.start()["reason"] == "card_disabled"
    assert gesture.info()["state"] == "error" and not robot.events
    gesture.enabled = True
    call(robot.plugin, "start")
    gesture.shutdown()
    assert call(robot.plugin, "start")["reason"] == "shutting_down"
    info = call(robot.plugin, "info")
    assert info["state"] == "idle" and not info["started"] and info["closed"]
    assert not robot.events


def test_timeout_uses_the_existing_stop_playback_and_does_not_report_success(robot):
    robot.plugin._traffic_gesture.config["playback_timeout_s"] = 1
    call(robot.plugin, "stop", execute=True)
    wait_until(lambda: bool(robot.callbacks), timeout=2)
    wait_until(lambda: bool(robot.callbacks))
    assert robot.callbacks[0][1] == "error"
    wait_until(lambda: bool(robot.callbacks))
    assert robot.callbacks[0][2]["reason"] == "playback_timeout"
    assert call(robot.plugin, "info")["busy"] is False and ("WALK", 0) in robot.events


def test_shutdown_cancels_and_joins_before_releasing_body(robot):
    accepted = call(robot.plugin, "stop", execute=True)
    worker = robot.plugin._traffic_gesture.worker
    robot.plugin.stop()
    assert robot.plugin._body_action is None and not worker.is_alive()
    wait_until(lambda: bool(robot.callbacks))
    assert len(robot.callbacks) == 1 and robot.callbacks[0][0] == accepted["action_id"]
    wait_until(lambda: bool(robot.callbacks))
    assert robot.callbacks[0][1] == "cancelled"
    assert call(robot.plugin, "go", execute=True)["reason"] == "shutting_down"


def test_cancel_during_submission_latches_and_cannot_restart_after_exit(robot, monkeypatch):
    entered, release = threading.Event(), threading.Event()
    prepare = robot.plugin._prepare_workmode

    def delayed_prepare(*args):
        result = prepare(*args)
        entered.set()
        assert release.wait(.5)
        return result

    monkeypatch.setattr(robot.plugin, "_prepare_workmode", delayed_prepare)
    results, exits = [], []
    start = threading.Thread(target=lambda: results.append(call(robot.plugin, "stop", execute=True)))
    start.start()
    assert entered.wait(.5)
    cancel = threading.Thread(target=lambda: exits.append(call(robot.plugin, "cancel")))
    cancel.start()
    wait_until(lambda: robot.plugin._traffic_gesture.active["cancel"].is_set())
    release.set()
    start.join(.5)
    cancel.join(.5)
    assert not start.is_alive() and not cancel.is_alive()
    assert results[0]["reason"] == "gesture_cancelled_before_command"
    assert exits[0]["state"] == "idle" and not robot.events and not robot.callbacks


def test_mode_change_after_preflight_does_not_enable_or_prepare_the_robot(robot, monkeypatch):
    snapshot = robot.plugin._observation._snapshot

    def snapshot_then_disabled():
        result = snapshot()
        robot.ctrl.mode = 0
        return result

    monkeypatch.setattr(robot.plugin._observation, "_snapshot", snapshot_then_disabled)
    result = call(robot.plugin, "stop", execute=True)
    assert result["state"] == "error" and not robot.events
    assert robot.plugin._body_action is None


def test_raw_recording_semantic_and_reset_interfaces_keep_the_original_path(robot):
    plugin = robot.plugin
    assert plugin.dispatch("start_recording", {"_tool_name": "action_recording"})["state"] == "running"
    saved = plugin.dispatch("finish_and_save_recording", {"_tool_name": "action_recording", "recording_id": 7})
    assert saved["recording_id"] == 7 and ("SAVETEACH", 7) in robot.events
    robot.ctrl.mode = 2
    played = plugin.dispatch("play_recording", {"_tool_name": "action_recording", "recording_id": 7})
    assert played["state"] == "running" and "action_id" not in played
    assert plugin._body_action is None
    exited = plugin.dispatch("stop_playback", {"_tool_name": "action_recording"})
    assert exited["state"] == "completed" and exited["confirmed"]
    assert plugin.dispatch("wave", {"_tool_name": "semantic_action"})["state"] == "running"
    assert plugin.dispatch("reset", {"_tool_name": "semantic_action"})["state"] == "completed"


def test_card_schema_config_and_packaging_match_actual_registration(robot):
    tool = next(t for t in robot.plugin.get_tools() if t["name"] == "traffic_police_gesture")
    assert tool["type"] == "actuator" and not tool["multiInstance"]
    assert tool["inputSchema"]["properties"]["action"]["enum"] == ["stop", "go", "info", "cancel"]
    assert tool["inputSchema"]["x-completion"]["actions"] == ["stop", "go"]
    assert tool["inputSchema"]["x-completion"]["background"]
    assert tool["inputSchema"]["x-hooks"]["on_interrupt_all"] == {"action": "cancel"}
    config = yaml.safe_load((BUMI/"config.yaml").read_text())["plugins"]["traffic_police_gesture"]
    assert config["stop_recording_id"] is None and config["go_recording_id"] is None
    cards = yaml.safe_load((BUMI/"driver.yaml").read_text())["cards"]
    assert {"name": "traffic_police_gesture", "type": "actuator"} in cards
    assert {"name": "traffic_light", "type": "processor"} in cards
    assert "COPY traffic_police_gesture.py /work/traffic_police_gesture.py" in (BUMI/"Dockerfile").read_text()


def test_bundle_passes_gesture_config_to_the_existing_loco_plugin(monkeypatch):
    module, _ = harness._load_device(monkeypatch)
    monkeypatch.setitem(sys.modules, "device", module)
    monkeypatch.setitem(sys.modules, "rclpy.executors", types.ModuleType("rclpy.executors"))
    monkeypatch.setitem(sys.modules, "common", types.ModuleType("common"))
    spec = importlib.util.spec_from_file_location("bumi_gesture_bundle", BUMI/"main.py")
    main = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(main)
    config = {"plugins": {"loco": {"enabled": True}, "traffic_police_gesture": {
        "enabled": True, "stop_recording_id": 19, "go_recording_id": 20}}}
    bundle = main.BumiDeviceBundle(config, "test", None, object(), None)
    assert "traffic_police_gesture" in [t["name"] for t in bundle.get_all_tools()]
    loco = bundle._plugins[0]
    assert loco._traffic_gesture.config["stop_recording_id"] == 19
    assert bundle.dispatch("traffic_police_gesture", {"action": "info"})["stop_configured"]


def test_no_trajectory_or_motor_publisher_is_implemented_in_the_gesture_module():
    tree = ast.parse((BUMI/"traffic_police_gesture.py").read_text())
    attrs = {node.attr for node in ast.walk(tree) if isinstance(node, ast.Attribute)}
    assert "_do_teaching_action" in attrs and "_execute_body_action" in attrs
    assert not ({"publish_cmd", "_publish_cmd", "create_publisher"} & attrs)


def test_simultaneous_cancel_requests_send_one_exit_and_one_terminal(robot, monkeypatch):
    call(robot.plugin, "stop", execute=True)
    entered, release = threading.Event(), threading.Event()

    def delayed_exit(command, *args, **kwargs):
        if command == "WALK":
            entered.set()
            assert release.wait(.5)
        return robot.edge(command, *args, **kwargs)

    monkeypatch.setattr(robot.plugin, "_send_edge_and_wait", delayed_exit)
    results = []
    first = threading.Thread(target=lambda: results.append(call(robot.plugin, "cancel")))
    second = threading.Thread(target=lambda: results.append(call(robot.plugin, "cancel")))
    first.start()
    assert entered.wait(.5)
    second.start()
    release.set()
    first.join(.5)
    second.join(.5)
    assert len(results) == 2 and all(result["state"] == "idle" for result in results)
    assert sum(event[0] == "WALK" for event in robot.events) == 1
    wait_until(lambda: bool(robot.callbacks))
    assert len(robot.callbacks) == 1


def test_simultaneous_gestures_admit_exactly_one_body_action(robot, monkeypatch):
    entered, release = threading.Event(), threading.Event()

    def delayed_start(command, *args, **kwargs):
        if command == "PLAYTEACH":
            entered.set()
            assert release.wait(.5)
        return robot.edge(command, *args, **kwargs)

    monkeypatch.setattr(robot.plugin, "_send_edge_and_wait", delayed_start)
    first = []
    thread = threading.Thread(target=lambda: first.append(call(robot.plugin, "stop", execute=True)))
    thread.start()
    assert entered.wait(.5)
    assert call(robot.plugin, "go", execute=True)["reason"] == "gesture_busy"
    release.set()
    thread.join(.5)
    assert first[0]["state"] == "running"
    assert sum(event[0] == "PLAYTEACH" for event in robot.events) == 1


def test_cancel_exception_retains_the_same_owner_and_can_be_retried(robot, monkeypatch):
    call(robot.plugin, "go", execute=True)
    original = robot.plugin._do_teaching_action

    def broken_exit(action, args):
        raise RuntimeError("SDK unavailable")

    monkeypatch.setattr(robot.plugin, "_do_teaching_action", broken_exit)
    assert call(robot.plugin, "cancel")["state"] == "locked"
    assert robot.plugin._body_action == "traffic_police_gesture.go"
    monkeypatch.setattr(robot.plugin, "_do_teaching_action", original)
    assert call(robot.plugin, "cancel")["state"] == "idle"
    wait_until(lambda: bool(robot.callbacks))
    assert len(robot.callbacks) == 1 and robot.callbacks[0][1] == "error"


def test_unknown_recording_existence_is_not_claimed_as_a_success(robot, monkeypatch):
    def no_ack(command, *args, **kwargs):
        robot.events.append((command, kwargs.get("index", 0)))
        return 2

    monkeypatch.setattr(robot.plugin, "_send_edge_and_wait", no_ack)
    result = call(robot.plugin, "go", execute=True)
    assert result["state"] == "accepted" and result["confirmed_started"] is False
    assert result["physical_completion_confirmed"] is False
    assert robot.plugin._body_action and not robot.callbacks
    assert call(robot.plugin, "cancel")["state"] == "idle"
    wait_until(lambda: bool(robot.callbacks))
    assert robot.callbacks[0][1] == "cancelled"


def test_fault_before_playback_and_missing_feedback_never_send_commands(robot):
    robot.ctrl.fault = 999
    assert call(robot.plugin, "stop", execute=True)["reason"] == "unsafe_robot_state"
    robot.ctrl.fault = 0
    robot.ctrl.speed = float("nan")
    assert call(robot.plugin, "stop", execute=True)["reason"] == "unsafe_robot_state"
    assert not robot.events


def test_shutdown_failure_is_locked_and_not_reported_as_idle(robot, monkeypatch):
    call(robot.plugin, "go", execute=True)
    monkeypatch.setattr(robot.plugin, "_send_edge_and_wait", lambda *args, **kwargs: 23)
    result = robot.plugin._traffic_gesture.shutdown()
    assert result["state"] == "locked" and robot.plugin._body_action
    assert call(robot.plugin, "info")["busy"] is True
    assert call(robot.plugin, "go", execute=True)["state"] == "error"
    wait_until(lambda: bool(robot.callbacks))
    assert len(robot.callbacks) == 1 and robot.callbacks[0][1] == "error"


def test_real_mcp_handler_routes_business_stop_and_project_stop_differently(robot, monkeypatch):
    monkeypatch.setitem(sys.modules, "device", robot.module)
    monkeypatch.setitem(sys.modules, "rclpy.executors", types.ModuleType("rclpy.executors"))
    monkeypatch.setitem(sys.modules, "common", types.ModuleType("common"))
    spec = importlib.util.spec_from_file_location("bumi_gesture_http", BUMI/"main.py")
    main = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(main)
    bundle = main.BumiDeviceBundle.__new__(main.BumiDeviceBundle)
    bundle._plugins = [robot.plugin]
    monkeypatch.setattr(main, "_bundle", bundle)
    handler_type = main.make_handler()

    def rpc(method, params):
        body = json.dumps({"jsonrpc": "2.0", "id": 1, "method": method, "params": params}).encode()
        handler = object.__new__(handler_type)
        handler.path, handler.headers, handler.rfile = "/mcp", {"Content-Length": str(len(body))}, io.BytesIO(body)
        replies = []
        handler._send = lambda status, text: replies.append((status, json.loads(text)))
        handler.do_POST()
        assert replies[0][0] == 200 and "error" not in replies[0][1]
        return replies[0][1]["result"]

    assert any(tool["name"] == "traffic_police_gesture" for tool in rpc("tools/list", {})["tools"])

    def execute(args):
        result = rpc("tools/call", {"name": "traffic_police_gesture", "arguments": args})
        return json.loads(result["content"][0]["text"])

    assert execute({"action": "stop", "instance_id": "canvas"})["state"] == "idle"
    assert not robot.events
    assert execute({"action": "stop", "execute": True})["state"] == "running"
    assert execute({"action": "info"})["active_gesture"] == "stop"
    assert execute({"action": "stop", "instance_id": "canvas"})["state"] == "idle"
    assert [event[0] for event in robot.events] == ["PLAYTEACH", "WALK"]
