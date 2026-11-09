"""Exercise the real LocoPlugin dispatch/worker with a host-only controller fake."""
import threading
import time
import types

import pytest

from noetix.bumi.tests import test_loco_destination as harness


@pytest.fixture
def loco(monkeypatch):
    module, _ = harness._load_device(monkeypatch)
    import observation_safety
    monkeypatch.setattr(observation_safety, "SETTLE_INTERVAL_S", .001)
    monkeypatch.setattr(observation_safety, "SETTLE_SAMPLES", 2)
    monkeypatch.setattr(module, "_get_default_cmd", lambda: 0)
    callbacks = []
    monkeypatch.setattr(module, "_acp_notify", lambda *args: callbacks.append(args))

    class Controller:
        mode = 2
        error = 0
        speed = 0.
        sent = []

        def get_mode(self):
            return self.mode

        def get_imu_data(self):
            return types.SimpleNamespace(ori=[0., 0., 0., 1.], angular_vel=[0., 0., self.speed])

        def get_joint_state(self):
            return [types.SimpleNamespace(error=self.error, vel=self.speed) for _ in range(21)]

        def publish_cmd(self, *args):
            # A fake only. No SDK module is imported by this test.
            self.sent.append(args)

    ctrl = Controller()
    ctrl.sent = []
    plugin = module.LocoPlugin({}, "test", None, ctrl)
    plugin._observation.action_timeout_s = .2
    plugin._observation.stop_timeout_s = .08
    monkeypatch.setattr(plugin, "_do_rotate_impl", lambda args: {
        "state": "done", "turned_deg": args["angle_deg"], "elapsed_s": .001})
    yield plugin, ctrl, callbacks
    plugin._stop_move()
    plugin._wait_for_rotate_idle(.5)
    monitor = plugin._observation.monitor
    if monitor:
        monitor.join(.5)


def call(plugin, action, **args):
    return plugin.dispatch(action, {"_tool_name": "loco_relative_move", **args})


def wait_session(plugin, sid, state="ready"):
    deadline = time.monotonic()+1
    while time.monotonic() < deadline:
        result = call(plugin, "observation_status", session_id=sid)
        if result["state"] == state:
            return result
        time.sleep(.002)
    pytest.fail(f"session did not reach {state}: {result}")


def test_begin_inspects_without_motion_and_reserves_existing_card(loco):
    plugin, ctrl, _ = loco
    begun = call(plugin, "observation_begin")
    assert begun["state"] == "ready" and begun["stop_confirmed"]
    assert not ctrl.sent and plugin.is_moving()
    assert call(plugin, "observation_begin")["code"] == "SESSION_ACTIVE"
    ended = call(plugin, "observation_end", session_id=begun["session_id"])
    assert ended["state"] == "ended" and ended["released"] and not plugin.is_moving()


def test_small_turn_reuses_worker_acp_and_matching_terminal(loco):
    plugin, _, callbacks = loco
    sid = call(plugin, "observation_begin")["session_id"]
    accepted = call(plugin, "observation_rotate", session_id=sid, request_id="turn1", angle_deg=10)
    assert accepted["state"] == "running" and accepted["action_id"]
    status = wait_session(plugin, sid)
    assert status["stop_confirmed"] and status["action_id"] == accepted["action_id"]
    assert status["rotation_count"] == 1 and status["total_absolute_rotation_deg"] == 10
    assert status["observation_after_ns"] > 0
    assert len(callbacks) == 1 and callbacks[0][0] == accepted["action_id"]
    assert call(plugin, "status")["last_action_id"] == accepted["action_id"]


@pytest.mark.parametrize("angle", [10.01, -11, float("inf"), float("nan"), True, "10", 0])
def test_single_rotation_rejects_invalid_values_without_motion(loco, angle):
    plugin, ctrl, _ = loco
    sid = call(plugin, "observation_begin")["session_id"]
    result = call(plugin, "observation_rotate", session_id=sid, request_id="a", angle_deg=angle)
    assert result["code"] == "SINGLE_ROTATION_LIMIT" and not ctrl.sent


def test_alternating_signs_consume_absolute_budget_and_count(loco):
    plugin, _, _ = loco
    sid = call(plugin, "observation_begin")["session_id"]
    for index, angle in enumerate([10, -10, 10]):
        assert call(plugin, "observation_rotate", session_id=sid, request_id=str(index), angle_deg=angle)["state"] == "running"
        wait_session(plugin, sid)
    result = call(plugin, "observation_rotate", session_id=sid, request_id="4", angle_deg=-1)
    assert result["code"] == "ROTATION_COUNT_LIMIT"
    assert call(plugin, "observation_status", session_id=sid)["total_absolute_rotation_deg"] == 30


def test_total_budget_cannot_be_bypassed(loco):
    plugin, _, _ = loco
    sid = call(plugin, "observation_begin")["session_id"]
    plugin._observation.session["total_absolute_rotation_deg"] = 25
    assert call(plugin, "observation_rotate", session_id=sid, request_id="x", angle_deg=6)["code"] == "TOTAL_ROTATION_LIMIT"


@pytest.mark.parametrize("mode,error,speed", [(26, 0, 0), (1, 0, 0), (2, 999, 0), (2, 0, .5), (2, 0, float("nan"))])
def test_mode_fault_and_motion_feedback_refuse_begin(loco, mode, error, speed):
    plugin, ctrl, _ = loco
    ctrl.mode, ctrl.error, ctrl.speed = mode, error, speed
    assert call(plugin, "observation_begin")["code"] == "UNSAFE_STATE"
    assert not ctrl.sent


def test_fault_is_rechecked_before_each_rotation(loco):
    plugin, ctrl, _ = loco
    sid = call(plugin, "observation_begin")["session_id"]
    ctrl.error = 1
    assert call(plugin, "observation_rotate", session_id=sid, request_id="x", angle_deg=3)["code"] == "UNSAFE_STATE"
    assert not ctrl.sent


@pytest.mark.parametrize("kind", ["move", "rotate", "body", "servo"])
def test_begin_does_not_preempt_another_owner(loco, kind):
    plugin, ctrl, _ = loco
    if kind == "move":
        plugin._move_thread = types.SimpleNamespace(is_alive=lambda: True, join=lambda **kwargs: None)
    elif kind == "rotate":
        plugin._rotate_state["state"] = "running"
    elif kind == "body":
        plugin._body_action = "dance"
    else:
        plugin._servo = types.SimpleNamespace(_info=lambda: {"state": "running"})
    assert call(plugin, "observation_begin")["code"] == "CHASSIS_BUSY"
    assert not ctrl.sent
    plugin._move_thread, plugin._body_action, plugin._servo = None, None, None
    plugin._rotate_state["state"] = "idle"


def test_raw_rotation_translation_and_body_actions_are_blocked(loco):
    plugin, ctrl, _ = loco
    call(plugin, "observation_begin")
    for action, args in [("rotate", {"angle_deg": 1}), ("move", {"x_m": .1, "y_m": 0})]:
        assert call(plugin, action, **args)["state"] == "error"
    assert plugin.dispatch("move", {"_tool_name": "loco", "vx": .5})["state"] == "error"
    assert plugin.dispatch("dance_1", {"_tool_name": "semantic_action"})["state"] == "error"
    assert plugin._do_destination({"angle_deg": 1, "_observation_token": "forged"}, "rotate")["state"] == "error"
    with pytest.raises(RuntimeError):
        plugin.publish_velocity(1, 0, 0)
    assert not ctrl.sent


def test_retry_does_not_turn_twice(loco):
    plugin, _, _ = loco
    sid = call(plugin, "observation_begin")["session_id"]
    args = {"session_id": sid, "request_id": "same", "angle_deg": 5}
    call(plugin, "observation_rotate", **args)
    wait_session(plugin, sid)
    assert call(plugin, "observation_rotate", **args)["code"] == "DUPLICATE_REQUEST"


def test_stop_move_latches_cancel_and_end_confirms_stopped(loco, monkeypatch):
    plugin, _, _ = loco
    entered = threading.Event()

    def turn(args):
        entered.set()
        plugin._rotate_stop_event.wait(.5)
        return {"state": "stopped", "turned_deg": 0}

    monkeypatch.setattr(plugin, "_do_rotate_impl", turn)
    sid = call(plugin, "observation_begin")["session_id"]
    call(plugin, "observation_rotate", session_id=sid, request_id="x", angle_deg=5)
    assert entered.wait(.5)
    assert call(plugin, "observation_rotate", session_id=sid, request_id="y", angle_deg=5)["state"] == "error"
    stopped = call(plugin, "stop_move")
    assert stopped["observation"]["stop_confirmed"] and stopped["observation"]["released"]
    wait_session(plugin, sid, "ended")
    assert call(plugin, "observation_rotate", session_id=sid, request_id="z", angle_deg=5)["state"] == "error"
    ended = call(plugin, "observation_end", session_id=sid)
    assert ended["state"] == "ended" and ended["stop_confirmed"]


@pytest.mark.parametrize("failure", ["timeout", "exception", "lost_id", "query", "moving_after_done", "wrong_terminal"])
def test_failures_stop_and_lock_instead_of_continuing(loco, monkeypatch, failure):
    plugin, ctrl, _ = loco
    sid = call(plugin, "observation_begin")["session_id"]
    if failure == "timeout":
        monkeypatch.setattr(plugin, "_do_rotate_impl", lambda args: (
            plugin._rotate_stop_event.wait(.5) or {"state": "stopped"}))
    elif failure == "exception":
        def fail(args):
            raise RuntimeError("worker fault")
        monkeypatch.setattr(plugin, "_do_rotate_impl", fail)
    elif failure == "lost_id":
        monkeypatch.setattr(plugin, "_do_destination", lambda *args: {"state": "running"})
    elif failure == "query":
        monkeypatch.setattr(plugin, "_rotate_status", lambda: (_ for _ in ()).throw(RuntimeError("status failed")))
    elif failure == "moving_after_done":
        monkeypatch.setattr(plugin, "_do_rotate_impl", lambda args: (
            setattr(ctrl, "speed", .5) or {"state": "done", "turned_deg": 5}))
    elif failure == "wrong_terminal":
        monkeypatch.setattr(plugin, "_do_rotate_impl", lambda args: {"state": "timeout"})
    call(plugin, "observation_rotate", session_id=sid, request_id="x", angle_deg=5)
    locked = wait_session(plugin, sid, "locked")
    assert locked["cancelled"] and locked["error"]
    assert call(plugin, "observation_rotate", session_id=sid, request_id="y", angle_deg=5)["state"] == "error"
    monitor = plugin._observation.monitor
    if monitor:
        monitor.join(.6)
    assert ctrl.sent and all(command[:3] == (0, 0, 0) for command in ctrl.sent)
    if failure == "moving_after_done":
        ended = call(plugin, "observation_end", session_id=sid)
        assert ended["state"] == "locked" and not ended["released"] and not ended["stop_confirmed"]


def test_unknown_session_and_completed_session_ids_are_not_reused(loco):
    plugin, _, _ = loco
    assert call(plugin, "observation_status", session_id="bad")["code"] == "INVALID_SESSION"
    sid = call(plugin, "observation_begin")["session_id"]
    call(plugin, "observation_end", session_id=sid)
    assert call(plugin, "observation_rotate", session_id=sid, request_id="x", angle_deg=1)["code"] == "INVALID_SESSION"
    new = call(plugin, "observation_begin")
    assert new["session_id"] != sid


def test_end_cannot_release_a_worker_that_ignores_stop(loco, monkeypatch):
    plugin, _, _ = loco
    entered, release = threading.Event(), threading.Event()

    def stuck(args):
        entered.set()
        release.wait(.8)
        return {"state": "stopped", "turned_deg": 0}

    monkeypatch.setattr(plugin, "_do_rotate_impl", stuck)
    sid = call(plugin, "observation_begin")["session_id"]
    call(plugin, "observation_rotate", session_id=sid, request_id="x", angle_deg=5)
    assert entered.wait(.5)
    try:
        result = call(plugin, "observation_end", session_id=sid)
        assert result["state"] == "locked" and not result["released"]
        assert call(plugin, "observation_begin")["code"] == "SESSION_ACTIVE"
    finally:
        release.set()


def test_normal_raw_rotation_still_works_after_observation_release(loco):
    plugin, _, _ = loco
    sid = call(plugin, "observation_begin")["session_id"]
    call(plugin, "observation_end", session_id=sid)
    result = call(plugin, "rotate", angle_deg=20)
    assert result["state"] == "running"
    assert plugin._wait_for_rotate_idle(.5)


def test_concurrent_observation_admission_only_accepts_one(loco, monkeypatch):
    plugin, _, _ = loco
    gate = threading.Event()

    def turn(args):
        gate.wait(.5)
        return {"state": "done", "turned_deg": 5}

    monkeypatch.setattr(plugin, "_do_rotate_impl", turn)
    sid = call(plugin, "observation_begin")["session_id"]
    results = []
    start = threading.Barrier(3)

    def request(ident):
        start.wait()
        results.append(call(plugin, "observation_rotate", session_id=sid, request_id=ident, angle_deg=5))

    threads = [threading.Thread(target=request, args=(str(i),)) for i in range(2)]
    for thread in threads:
        thread.start()
    start.wait()
    for thread in threads:
        thread.join(.5)
    gate.set()
    assert sum(r["state"] == "running" for r in results) == 1
    assert call(plugin, "observation_status", session_id=sid)["rotation_count"] == 1


def test_lifecycle_stop_cleans_session_without_another_llm_turn(loco):
    plugin, _, _ = loco
    sid = call(plugin, "observation_begin")["session_id"]
    stopped = call(plugin, "stop")
    assert stopped["state"] == "idle" and stopped["observation"]["released"]
    assert call(plugin, "observation_status", session_id=sid)["stop_confirmed"]


def test_state_sensor_lifecycle_is_independent_of_observation_safety(monkeypatch):
    module, _ = harness._load_device(monkeypatch)
    sensor = module.StatePlugin.__new__(module.StatePlugin)
    assert sensor.dispatch("stop", {}) == {"state": "idle"}


def test_concurrent_end_calls_serialize_the_entire_release(loco, monkeypatch):
    plugin, _, _ = loco
    safety = plugin._observation
    sid = call(plugin, "observation_begin")["session_id"]
    entered, release, concurrent = threading.Event(), threading.Event(), threading.Event()
    verify = safety._stop_and_verify
    calls = []

    def slow_verify(session):
        calls.append(session["session_id"])
        if len(calls) > 1:
            concurrent.set()
        entered.set()
        release.wait(.5)
        return verify(session)

    monkeypatch.setattr(safety, "_stop_and_verify", slow_verify)
    results = []
    first = threading.Thread(target=lambda: results.append(call(plugin, "observation_end", session_id=sid)))
    second = threading.Thread(target=lambda: results.append(call(plugin, "observation_end", session_id=sid)))
    first.start()
    assert entered.wait(.5)
    second.start()
    try:
        assert not concurrent.wait(.03)
    finally:
        release.set()
        first.join(.5)
        second.join(.5)
    assert len(calls) == 1 and len(results) == 2
    assert all(result["released"] and result["stop_confirmed"] for result in results)
