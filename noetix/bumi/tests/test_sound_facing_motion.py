"""Exercise the production LocoPlugin, with FakeHighCtrl and no ACP network."""

import threading
import time
import types

import pytest

from test_loco_destination import FakeHighCtrl, _load_device, _rotate_plugin, wait_rotate


def setup(monkeypatch, mode=2):
    module, _ = _load_device(monkeypatch)
    acp = []
    monkeypatch.setattr(module, "_acp_notify", lambda *args: acp.append(args))
    fake = FakeHighCtrl(mode=mode)
    return _rotate_plugin(module, monkeypatch, fake), fake, acp


def test_no_owner_capability_can_be_supplied_through_mcp(monkeypatch):
    loco, fake, _ = setup(monkeypatch)
    owner = object()
    assert loco.reserve_relative_motion(owner)["state"] == "reserved"
    assert loco.dispatch("rotate", {"_tool_name": "loco_relative_move", "angle_deg": 40,
                                     "owner": owner})["state"] == "error"
    assert loco.rotate_owned(object(), 40)["state"] == "error"
    assert not fake.cmds
    assert loco.release_relative_motion(owner)["state"] == "released"


@pytest.mark.parametrize("action,tool,args", [
    ("move", "loco", {"vx": 0.5}), ("rotate", "loco_relative_move", {"angle_deg": 40}),
    ("move", "loco_relative_move", {"x_m": 1, "y_m": 0}),
    ("lie_prone", "stand_up_lie_prone", {}), ("wave", "semantic_action", {}),
    ("start_recording", "action_recording", {}),
])
def test_session_reserves_chassis_while_observing(monkeypatch, action, tool, args):
    loco, fake, _ = setup(monkeypatch)
    owner = object()
    loco.reserve_relative_motion(owner)
    out = loco.dispatch(action, {"_tool_name": tool, **args})
    assert out is None or out["state"] == "error"
    assert not fake.cmds and loco.is_moving()


def test_scoped_cancel_never_stops_other_owners_or_late_actions(monkeypatch):
    loco, fake, _ = setup(monkeypatch)
    owner = object()
    loco.reserve_relative_motion(owner)
    started = loco.rotate_owned(owner, 150)
    assert loco.cancel_owned_rotation(object(), started["action_id"])["state"] == "error"
    assert not loco._rotate_stop_event.is_set()
    assert loco.cancel_owned_rotation(owner, "old")["state"] == "unchanged"
    assert not loco._rotate_stop_event.is_set()
    assert loco.release_relative_motion(owner)["error"] == "rotation_not_terminal"
    loco.cancel_owned_rotation(owner, started["action_id"])
    assert wait_rotate(loco)["state"] == "stopped"
    assert fake.cmds[-1] == (0, 0, 0)
    assert loco.release_relative_motion(owner)["state"] == "released"
    before = list(fake.cmds)
    assert loco.cancel_owned_rotation(owner, started["action_id"])["state"] == "error"
    assert fake.cmds == before


@pytest.mark.parametrize("angle", [30, -30, 180, -180])
def test_internal_turn_uses_existing_imu_loop_and_local_receipt(monkeypatch, angle):
    loco, fake, acp = setup(monkeypatch)
    owner = object()
    loco.reserve_relative_motion(owner)
    started = loco.rotate_owned(owner, angle)
    assert started["state"] == "running"
    result = wait_rotate(loco)
    receipt = loco.owned_rotation_status(owner, started["action_id"])
    assert receipt["state"] == "terminal" and receipt["result"] == result
    assert result["state"] == "done" and abs(result["final_error_deg"]) <= 2
    assert any(cmd[2] * angle > 0 for cmd in fake.cmds)
    assert fake.cmds[-1] == (0, 0, 0) and acp == []
    assert loco.owned_rotation_status(owner, "old")["error"] == "unknown_action"
    receipt["result"]["state"] = "tampered"
    assert loco.owned_rotation_status(owner, started["action_id"])["result"]["state"] == "done"
    loco.release_relative_motion(owner)


@pytest.mark.parametrize("mode", [0, 1, 23, 26, 30])
def test_mode_refusals_no_motion_no_acp(monkeypatch, mode):
    loco, fake, acp = setup(monkeypatch, mode)
    owner = object()
    loco.reserve_relative_motion(owner)
    assert loco.relative_motion_health(owner)["state"] == "error"
    assert loco.rotate_owned(owner, 30)["state"] == "error"
    assert not fake.cmds and not acp
    loco.release_relative_motion(owner)


def test_original_per_turn_limit_is_still_enforced(monkeypatch):
    loco, fake, _ = setup(monkeypatch)
    owner = object()
    loco.reserve_relative_motion(owner)
    assert loco.rotate_owned(owner, 361)["state"] == "error"
    assert not fake.cmds


def test_active_other_controller_is_refused_without_stopping_it(monkeypatch):
    loco, fake, _ = setup(monkeypatch)
    loco._body_action = "other_card"
    assert loco.reserve_relative_motion(object())["error"] == "chassis_busy"
    assert loco._body_action == "other_card" and not fake.cmds
    loco._body_action = None
    loco._servo = types.SimpleNamespace(_info=lambda: {"state": "running"})
    assert loco.reserve_relative_motion(object())["error"] == "servo_not_idle"
    assert not fake.cmds


def test_global_manual_stop_revokes_ongoing_session_health(monkeypatch):
    loco, _, _ = setup(monkeypatch)
    owner = object()
    loco.reserve_relative_motion(owner)
    loco._stop_move()
    assert loco.relative_motion_health(owner)["error"] == "operator_interrupted"
    started = loco.rotate_owned(owner, 30)
    if started["state"] == "running":
        assert wait_rotate(loco)["state"] == "error"


def test_publisher_barrier_drains_inflight_servo_and_blocks_future_writes(monkeypatch):
    loco, fake, _ = setup(monkeypatch)
    entered, release, reserved = threading.Event(), threading.Event(), threading.Event()
    real_publish = fake.publish_cmd

    def slow_publish(*args):
        entered.set()
        assert release.wait(3)
        real_publish(*args)

    monkeypatch.setattr(fake, "publish_cmd", slow_publish)
    servo = threading.Thread(target=lambda: loco.publish_velocity(0, 0, 0.5))
    servo.start()
    assert entered.wait(2)
    owner = object()
    box = {}

    def reserve():
        box.update(loco.reserve_relative_motion(owner))
        reserved.set()

    reservation = threading.Thread(target=reserve)
    reservation.start()
    assert not reserved.wait(0.05)
    release.set()
    servo.join(2)
    reservation.join(2)
    assert not servo.is_alive() and not reservation.is_alive()
    assert box["state"] == "reserved" and len(fake.cmds) == 1
    loco.publish_velocity(0, 0, 0.8)
    loco.publish_velocity(0, 0, 0)
    assert len(fake.cmds) == 1
    loco.release_relative_motion(owner)
    loco.publish_velocity(0, 0, 0)
    assert len(fake.cmds) == 2


def test_cancel_during_acceptance_window_sticks(monkeypatch):
    loco, fake, acp = setup(monkeypatch)
    owner = object()
    loco.reserve_relative_motion(owner)
    entered, release = threading.Event(), threading.Event()
    real_preflight = loco._rotate_preflight

    def delayed(args):
        entered.set()
        assert release.wait(3)
        return real_preflight(args)

    monkeypatch.setattr(loco, "_rotate_preflight", delayed)
    result = {}
    worker = threading.Thread(target=lambda: result.update(loco.rotate_owned(owner, 30)))
    worker.start()
    assert entered.wait(2)
    # A stop before the ID is exposed must reject delayed acceptance, rather
    # than starting a worker and erasing its cancellation Event.
    loco._stop_move()
    release.set()
    worker.join(3)
    assert not worker.is_alive()
    assert result["state"] == wait_rotate(loco)["state"] == "stopped"
    assert all(cmd == (0, 0, 0) for cmd in fake.cmds) and not acp


def test_owner_interrupt_during_preflight_needs_no_action_id(monkeypatch):
    loco, fake, acp = setup(monkeypatch)
    owner = object();loco.reserve_relative_motion(owner)
    entered, release = threading.Event(), threading.Event()
    original = loco._rotate_preflight
    def delayed(args):
        entered.set();assert release.wait(2);return original(args)
    monkeypatch.setattr(loco, "_rotate_preflight", delayed)
    result = {}
    caller = threading.Thread(target=lambda: result.update(loco.rotate_owned(owner, 30)))
    caller.start();assert entered.wait(1)
    started = time.perf_counter()
    ack = loco.interrupt_owned_session(owner)
    assert time.perf_counter() - started < .2 and ack["state"] == "cancelling"
    assert ack["action_id"] is None
    release.set();caller.join(1)
    assert not caller.is_alive() and result["state"] == "stopped"
    assert not any(any(v for v in cmd) for cmd in fake.cmds) and not acp
    assert loco.rotate_owned(owner, 20)["error"] == "owned_session_interrupted"
    assert loco.interrupt_owned_session(object())["state"] == "unchanged"


def test_operator_expiry_during_sdk_preflight_cannot_start_a_worker(monkeypatch):
    from sound_facing import SoundFacing
    from sound_authorization_fixtures import approved_authority, approve_engine
    from test_sound_facing import Clock, feed
    loco, fake, acp = setup(monkeypatch)
    clock = Clock()
    engine = SoundFacing(loco, monotonic=clock.mono, wall_ns=clock.wall,
                         authority=approved_authority(clock))
    engine.start();approve_engine(engine, 1);engine.enable()
    original = loco._rotate_preflight
    def delayed(args):
        clock.advance(1)
        return original(args)
    monkeypatch.setattr(loco, "_rotate_preflight", delayed)
    feed(clock, engine)
    assert not engine.session_id and not engine.reserved
    assert not any(any(v for v in cmd) for cmd in fake.cmds) and not acp
    assert loco._relative_owner is None


def test_publisher_rechecks_authority_after_rate_limit_wait(monkeypatch):
    loco, fake, _ = setup(monkeypatch)
    owner, allowed = object(), [True]
    loco.reserve_relative_motion(owner, authorization=lambda: allowed[0])
    loco._last_cmd_time = time.monotonic()
    monkeypatch.setattr(time, "sleep", lambda _: allowed.__setitem__(0, False))
    with pytest.raises(RuntimeError, match="owned_authorization_invalid"):
        loco._publish_cmd(0, 0, .5, "fake-command")
    assert not fake.cmds


def test_reservation_rolls_back_if_second_servo_health_read_fails(monkeypatch):
    loco, _, _ = setup(monkeypatch)
    count = [0]

    def info():
        count[0] += 1
        if count[0] == 2:
            raise OSError("unavailable")
        return {"state": "idle"}

    loco._servo = types.SimpleNamespace(_info=info)
    with pytest.raises(OSError):
        loco.reserve_relative_motion(object())
    assert loco._relative_owner is None


def test_owned_imu_exception_still_brakes_and_yields_terminal_receipt(monkeypatch):
    loco, fake, acp = setup(monkeypatch)
    owner = object()
    loco.reserve_relative_motion(owner)
    monkeypatch.setattr(fake, "get_imu_data", lambda: (_ for _ in ()).throw(OSError()))
    started = loco.rotate_owned(owner, 30)
    result = wait_rotate(loco)
    assert result["state"] == "error" and result["stop_command_sent"] is True
    assert result["physical_stop_confirmed"] is False
    assert loco.owned_rotation_status(owner, started["action_id"])["state"] == "terminal"
    assert fake.cmds == [(0, 0, 0)] and not acp


def test_failed_sdk_brake_keeps_reservation(monkeypatch):
    loco, fake, acp = setup(monkeypatch)
    owner = object()
    loco.reserve_relative_motion(owner)
    monkeypatch.setattr(fake, "publish_cmd", lambda *args: (_ for _ in ()).throw(OSError()))
    loco.rotate_owned(owner, 30)
    result = wait_rotate(loco)
    assert result["state"] == "error" and result["stop_command_sent"] is False
    assert loco.release_relative_motion(owner)["error"] == "owned_stop_unconfirmed"
    assert loco.is_moving() and not acp


def test_internal_terminal_transition_has_no_missing_receipt_gap(monkeypatch):
    loco, _, _ = setup(monkeypatch)
    owner = object()
    loco.reserve_relative_motion(owner)
    monkeypatch.setattr(loco, "_do_rotate_impl", lambda args: {
        "state": "done", "requested_angle_deg": args["angle_deg"], "final_error_deg": 0})
    for _ in range(30):
        started = loco.rotate_owned(owner, 30)
        while True:
            receipt = loco.owned_rotation_status(owner, started["action_id"])
            assert receipt["state"] in ("running", "terminal")
            if receipt["state"] == "terminal":
                break
            time.sleep(0)
    loco.release_relative_motion(owner)
