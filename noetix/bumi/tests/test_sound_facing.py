"""Deterministic offline supervision tests. No ROS or hardware imports."""

import sys
import threading
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from sound_facing import CONVENTION, SoundFacing
from sound_authorization_fixtures import approved_authority, approve_engine


class Clock:
    now = 0.0
    epoch = 1_800_000_000_000_000

    def mono(self):
        return self.now

    def wall(self):
        return int(self.epoch + self.now * 1e6) * 1000

    def advance(self, seconds):
        self.now += seconds


class Motion:
    def __init__(self):
        self.owner = None
        self.calls = []
        self.cancelled = []
        self.receipt = None
        self.problem = None

    def reserve_relative_motion(self, owner, *, authorization=None):
        if self.owner is not None and self.owner is not owner:
            return {"state": "error", "error": "chassis_busy"}
        self.owner = owner
        self.authorization = authorization
        return {"state": "reserved"}

    def relative_motion_health(self, owner):
        return {"state": "error", "error": self.problem} if self.problem else {
            "state": "ready", "yaw_deg": 0.0, "stationary": True}

    def owned_motion_stability(self, owner):
        return {"state": "ready", "yaw_deg": 0.0, "stationary": True}

    def rotate_owned(self, owner, angle):
        assert self.owner is owner
        assert self.receipt is None
        self.calls.append(angle)
        action_id = str(len(self.calls))
        self.receipt = {"state": "running", "action_id": action_id}
        return self.receipt.copy()

    def owned_rotation_status(self, owner, action_id):
        return self.receipt.copy()

    def cancel_owned_rotation(self, owner, action_id):
        assert self.owner is owner
        self.cancelled.append(action_id)
        return {"state": "cancelling"}

    def interrupt_owned_session(self, owner):
        if self.owner is owner and self.receipt is not None:
            self.cancelled.append(self.receipt["action_id"])
        return {"state": "cancelling" if self.owner is owner else "unchanged"}

    def release_relative_motion(self, owner):
        assert owner is self.owner
        self.owner = None
        self.receipt = None
        return {"state": "released"}

    def finish(self, state="done", **extra):
        self.receipt = {"state": "terminal", "action_id": str(len(self.calls)),
                        "result": {"state": state, "requested_angle_deg": self.calls[-1],
                                   "final_error_deg": 0.0, "stop_command_sent": True, **extra}}


def observation(clock, angle=40, duration=0.2):
    end = clock.wall() // 1000
    return {"state": "fresh", "angle_deg": angle, "angle_convention": CONVENTION,
            "audio_window_start_us": end - round(duration * 1e6),
            "audio_window_end_us": end, "timestamp_ms": end // 1000}


def fixture(config=None, enable=True):
    clock, motion = Clock(), Motion()
    engine = SoundFacing(motion, config, monotonic=clock.mono, wall_ns=clock.wall,
                         authority=approved_authority(clock))
    engine.start()
    if enable:
        engine.enable()
    return clock, motion, engine.session_id, engine


def feed(clock, engine, angles=(40, 40, 40)):
    for angle in angles:
        clock.advance(0.25)
        engine.observe(observation(clock, angle))


def finish_and_settle(clock, motion, engine):
    motion.finish()
    engine.tick()
    assert engine.state == "settling"
    clock.advance(0.81)
    engine.observe({"state": "no_event"})
    engine.tick()
    assert engine.state in ("observing", "paused")
    motion.receipt = None


@pytest.mark.parametrize("angle,expected", [(0, None), (5, None), (11, None), (35, 35), (-40, -40), (180, 180), (-180, -180)])
def test_front_left_right_rear(angle, expected):
    clock, motion, _, engine = fixture()
    feed(clock, engine, (angle,) * 3)
    assert motion.calls == ([] if expected is None else [pytest.approx(expected)])


def test_boundary_crossing_uses_circular_mean():
    clock, motion, _, engine = fixture()
    feed(clock, engine, (179, -179, 178))
    assert len(motion.calls) == 1 and abs(motion.calls[0]) > 178


def test_many_turns_exceed_360_without_a_session_angle_cap():
    clock, motion, session, engine = fixture()
    for _ in range(12):
        feed(clock, engine, (90,) * 3)
        assert engine.state == "turning"
        finish_and_settle(clock, motion, engine)
    assert sum(motion.calls) == pytest.approx(1080)
    assert engine.turns == 12 and engine.session_id == session


def test_source_moves_left_then_right_and_hysteresis_blocks_fast_reversal():
    clock, motion, _, engine = fixture()
    feed(clock, engine)
    finish_and_settle(clock, motion, engine)
    feed(clock, engine, (-40,) * 3)
    assert motion.calls == [40]
    assert engine.reason == "reversal_hysteresis"
    for _ in range(5):
        clock.advance(0.25)
        engine.observe(observation(clock, -40))
    assert motion.calls == [40, -40]


@pytest.mark.parametrize("angles", [(40, -40) * 10, (5, 50, 100) * 5, (40, 55, 30) * 5])
def test_unstable_or_multiple_sources_never_turn(angles):
    clock, motion, _, engine = fixture()
    feed(clock, engine, angles)
    assert not motion.calls


def test_missing_event_discards_history_and_never_reuses_old_turn():
    clock, motion, _, engine = fixture()
    feed(clock, engine, (40, 40))
    engine.observe({"state": "no_event"})
    feed(clock, engine, (40,))
    assert not motion.calls
    engine.observe({"state": "stale"})
    clock.advance(0.5)
    engine.tick()
    assert not motion.calls


def test_pending_turn_blocks_all_other_observations_and_waits_for_terminal():
    clock, motion, _, engine = fixture()
    feed(clock, engine)
    feed(clock, engine, (-80,) * 20)
    assert motion.calls == [40] and engine.state == "turning"
    finish_and_settle(clock, motion, engine)
    assert motion.calls == [40]
    assert engine.state != "turning"


def test_old_window_published_after_turn_cannot_retrigger():
    clock, motion, _, engine = fixture()
    feed(clock, engine)
    old = observation(clock)
    finish_and_settle(clock, motion, engine)
    old["timestamp_ms"] = clock.wall() // 1_000_000
    engine.observe(old)
    assert len(motion.calls) == 1
    assert engine.reason in ("invalid_or_expired_observation", "before_observation_boundary")


@pytest.mark.parametrize("field,value", [
    ("angle_deg", True), ("angle_deg", float("nan")), ("angle_deg", float("inf")),
    ("angle_deg", 181), ("angle_deg", "45"), ("audio_window_start_us", 0),
    ("audio_window_end_us", 0), ("timestamp_ms", 0),
    ("audio_window_start_us", True), ("angle_convention", "right_positive"),
])
def test_invalid_data_never_moves(field, value):
    clock, motion, _, engine = fixture()
    for _ in range(10):
        clock.advance(0.25)
        data = observation(clock)
        data[field] = value
        engine.observe(data)
    assert not motion.calls


@pytest.mark.parametrize("kind", ["duplicate", "reordered", "future", "expired", "oversized_window", "missing_capture"])
def test_replay_and_clock_validation(kind):
    clock, motion, _, engine = fixture()
    clock.advance(0.25)
    old = observation(clock)
    engine.observe(old)
    for _ in range(5):
        clock.advance(0.1)
        data = old.copy() if kind in ("duplicate", "reordered") else observation(clock)
        if kind == "reordered":
            data["audio_window_start_us"] -= 1000
        elif kind == "future":
            data["audio_window_end_us"] += 1_000_000
        elif kind == "expired":
            data["audio_window_start_us"] -= 2_000_000
            data["audio_window_end_us"] -= 2_000_000
        elif kind == "oversized_window":
            data["audio_window_start_us"] -= 2_000_000
        elif kind == "missing_capture":
            del data["audio_window_start_us"]
        engine.observe(data)
    assert not motion.calls


def test_overlapping_windows_do_not_count_as_independent_evidence():
    clock, motion, _, engine = fixture()
    clock.advance(1)
    for _ in range(6):
        engine.observe(observation(clock, duration=0.5))
        clock.advance(0.1)
    assert not motion.calls


@pytest.mark.parametrize("state", ["error", "timeout", "stopped"])
def test_motion_failure_disarms_and_does_not_retry(state):
    clock, motion, session, engine = fixture()
    feed(clock, engine)
    motion.finish(state)
    engine.tick()
    feed(clock, engine, (80,) * 10)
    assert engine.state == "fault" and engine.session_id is None
    assert motion.calls == [40]


def test_success_without_correlated_valid_loop_result_is_a_fault():
    clock, motion, _, engine = fixture()
    feed(clock, engine)
    motion.finish(final_error_deg=30)
    engine.tick()
    assert engine.reason == "invalid_rotation_completion" and engine.session_id is None


def test_late_or_duplicate_receipt_cannot_complete_another_turn():
    clock, motion, _, engine = fixture()
    feed(clock, engine)
    motion.finish()
    old = motion.receipt.copy()
    finish_and_settle(clock, motion, engine)
    feed(clock, engine)
    motion.receipt = old
    engine.tick()
    assert engine.state == "fault" and motion.cancelled[-1] == "2"
    assert len(motion.calls) == 2


@pytest.mark.parametrize("failure", ["stream", "session", "mode", "downstream", "clock", "timeout"])
def test_watchdogs_cancel_only_owned_turn_and_revoke(failure):
    clock, motion, _, engine = fixture({"turn_timeout_s": 1, "session_timeout_s": 1})
    feed(clock, engine)
    if failure == "stream":
        clock.advance(1.6)
    elif failure == "session":
        clock.advance(1.01)
    elif failure == "mode":
        motion.problem = "protection_mode"
    elif failure == "downstream":
        motion.relative_motion_health = lambda owner: (_ for _ in ()).throw(OSError())
    elif failure == "clock":
        clock.epoch += 1_000_000
    else:
        clock.advance(1.1)
        engine.observe({"state": "no_event"})
    engine.tick()
    assert engine.state == "fault" and engine.session_id is None
    assert set(motion.cancelled) == {"1"}
    assert len(motion.calls) == 1


def test_disable_drains_before_reenable_and_is_idempotent():
    clock, motion, session, engine = fixture()
    feed(clock, engine)
    for _ in range(4):
        engine.disable()
        assert engine.enable().get("error") == "previous_motion_not_released"
    assert engine.session_id is None
    motion.finish("stopped")
    engine.tick()
    assert engine.reserved and engine.state == "stopping"
    clock.advance(0.81)
    engine.tick()
    assert not engine.reserved and engine.state == "idle"
    assert engine.enable()["error"] == "operator_authorization_required"
    approve_engine(engine)
    engine.enable()
    assert engine.session_id and engine.session_id != session


def test_start_config_restart_never_enable_tracking():
    clock, motion, _, engine = fixture(enable=False)
    assert engine.state == "idle"
    assert motion.owner is None
    engine.configure({"deadzone_deg": 15})
    engine.start()
    feed(clock, engine)
    assert engine.session_id is None and not motion.calls
    restarted = SoundFacing(motion)
    restarted.start()
    assert restarted.state == "idle" and restarted.session_id is None


def test_configuration_change_requires_new_explicit_enable():
    clock, motion, session, engine = fixture()
    engine.configure({"spread_deg": 10})
    feed(clock, engine)
    assert engine.session_id is None and motion.owner is None


@pytest.mark.parametrize("changes", [{"confirm_motion": True}, {"stable_windows": 1},
    {"stable_windows": True}, {"settle_s": 0}, {"deadzone_deg": float("nan")},
    {"release_deg": 20}, {"max_gap_s": 2, "stream_timeout_s": 1}])
def test_invalid_config_is_atomic(changes):
    _, _, _, engine = fixture()
    before = engine.snapshot()
    with pytest.raises(ValueError):
        engine.configure(changes)
    assert engine.snapshot() == before


def test_concurrent_enable_disable_has_one_owner_and_no_unsolicited_motion():
    _, motion, _, engine = fixture()
    jobs = [threading.Thread(target=engine.enable if i % 2 else engine.disable) for i in range(40)]
    for job in jobs:
        job.start()
    for job in jobs:
        job.join(2)
        assert not job.is_alive()
    engine.disable()
    assert motion.owner is None and not motion.calls


def test_acoustic_gap_discards_previous_evidence():
    clock, motion, _, engine = fixture()
    feed(clock, engine, (40, 40))
    clock.advance(1)
    feed(clock, engine, (40,))
    assert not motion.calls


def test_session_cannot_be_renewed_after_expiry():
    clock, motion, _, engine = fixture()
    clock.advance(31)
    engine.tick()
    assert engine.reason == "session_expired" and motion.owner is None


def test_failed_brake_retains_ownership_and_prevents_reenable():
    clock, motion, _, engine = fixture()
    feed(clock, engine)
    motion.finish("error", stop_command_sent=False)
    engine.tick()
    assert engine.reason == "owned_stop_unconfirmed" and engine.reserved
    engine.disable()
    assert engine.enable()["error"] == "previous_motion_not_released"
    assert motion.owner is engine.owner


def test_settling_checks_current_imu_quietness_before_reobserving():
    clock, motion, _, engine = fixture()
    feed(clock, engine)
    motion.finish()
    engine.tick()
    motion.relative_motion_health = lambda owner: {"state": "ready", "yaw_deg": 2, "stationary": False}
    clock.advance(0.9)
    engine.observe({"state": "no_event"})
    assert engine.state == "settling" and engine.reason == "settling_motion_detected"
    motion.relative_motion_health = lambda owner: {"state": "ready", "yaw_deg": 2, "stationary": True}
    clock.advance(0.9)
    engine.observe({"state": "no_event"})
    assert engine.state == "paused" and len(motion.calls) == 1


def test_settling_never_quiet_times_out_without_new_rotation():
    clock, motion, _, engine = fixture()
    feed(clock, engine)
    motion.finish()
    engine.tick()
    motion.relative_motion_health = lambda owner: {"state": "ready", "yaw_deg": 0, "stationary": False}
    for _ in range(14):
        clock.advance(0.5)
        engine.observe({"state": "no_event"})
    assert engine.reason == "settling_timeout" and engine.state == "fault"
    assert len(motion.calls) == 1


def test_angular_motion_noise_while_observing_cannot_trigger_turn():
    clock, motion, _, engine = fixture()
    motion.relative_motion_health = lambda owner: {"state": "ready", "yaw_deg": 5, "stationary": False}
    feed(clock, engine, (40,) * 10)
    assert not motion.calls and engine.reason == "robot_not_stationary"


def test_explicit_renew_keeps_same_session_but_cannot_revive_expired_session():
    clock, motion, session, engine = fixture({"session_timeout_s": 1})
    clock.advance(0.8)
    assert engine.renew(session)["session_id"] == session
    clock.advance(0.4)
    engine.observe({"state": "no_event"})
    assert engine.snapshot()["enabled"]
    before = engine._expires
    assert engine.renew("old")["error"] == "session_not_active_or_mismatched"
    assert engine._expires == before
    clock.advance(0.7)
    assert engine.renew(session)["error"] == "session_not_active_or_mismatched"
    assert engine.reason == "session_expired" and motion.owner is None


def test_sensors_info_start_and_idempotent_enable_do_not_renew():
    clock, motion, session, engine = fixture({"session_timeout_s": 1})
    deadline = engine._expires
    for _ in range(4):
        clock.advance(0.2)
        engine.observe({"state": "no_event"})
        engine.start()
        engine.snapshot()
        engine.enable()
        engine.configure({})
        assert engine._expires == deadline and engine.session_id == session
    clock.advance(0.3)
    engine.tick()
    assert engine.reason == "session_expired" and not motion.calls


def test_cancel_retains_ownership_until_continuous_imu_quiet():
    clock, motion, _, engine = fixture()
    feed(clock, engine)
    engine.disable()
    motion.finish("stopped")
    motion.owned_motion_stability = lambda owner: {"state": "ready", "yaw_deg": 10, "stationary": False}
    engine.tick()
    clock.advance(1)
    engine.tick()
    assert engine.reserved and engine.state == "stopping"
    motion.owned_motion_stability = lambda owner: {"state": "ready", "yaw_deg": 10, "stationary": True}
    engine.tick()
    clock.advance(0.81)
    engine.tick()
    assert engine.state == "idle" and not engine.reserved


def test_cancel_with_nonquiet_or_missing_imu_keeps_body_locked():
    clock, motion, _, engine = fixture()
    feed(clock, engine)
    engine.disable()
    motion.finish("stopped")
    motion.owned_motion_stability = lambda owner: {"state": "error", "error": "imu_unavailable"}
    engine.tick()
    clock.advance(6.1)
    engine.tick()
    assert engine.reason == "owned_stop_unconfirmed" and engine.reserved
    assert engine.enable()["error"] == "previous_motion_not_released"


def test_opposing_overlapping_window_invalidates_old_stability():
    clock, motion, _, engine = fixture()
    feed(clock, engine, (40, 40))
    clock.advance(0.05)
    engine.observe(observation(clock, -40))
    feed(clock, engine, (40,))
    assert not motion.calls


def test_maximum_quiet_interval_can_release_after_cancellation():
    clock, motion, _, engine = fixture({"settle_s": 5})
    feed(clock, engine)
    engine.disable()
    motion.finish("stopped")
    engine.tick()
    clock.advance(5.01)
    engine.tick()
    assert engine.state == "idle" and not engine.reserved


@pytest.mark.parametrize("shift_us", [-1_000_000, 1_000_000])
def test_real_clock_jump_retires_authorization_and_never_auto_recovers(shift_us):
    clock, motion, session, engine = fixture()
    feed(clock, engine)
    clock.epoch += shift_us
    engine.tick()
    assert engine.reason == "capture_clock_discontinuity"
    assert not engine.session_id and motion.cancelled
    clock.epoch -= shift_us
    engine.tick()
    assert engine.renew(session)["error"] == "session_not_active_or_mismatched"
    assert len(motion.calls) == 1


def test_sampling_scheduler_delay_is_recorded_and_fails_closed():
    clock, motion, _, engine = fixture()
    feed(clock, engine)
    original = engine._wall
    def delayed():
        clock.advance(.3)
        return original()
    engine._wall = delayed
    engine.tick()
    assert engine.reason == "capture_clock_sampling_delayed"
    assert engine.snapshot()["clock_diagnostics"]["sample_span_s"] >= .3 - 1e-8
    assert not engine.session_id and motion.cancelled
