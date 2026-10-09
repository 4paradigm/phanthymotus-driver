"""Strict velocity-stream safety, with fake clocks and bounded fake transports.

The AS2W adapter does not own these checks: every strict twist sink gets the
same tests, including faults and a vendor gate that outlives a frame's TTL.
"""
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from common.control import ControlSink, Outcome, Verdict


class Clock:
    def __init__(self):
        self.mono = 0
        self.wall = 1_000_000

    def advance(self, ms):
        self.mono += ms
        self.wall += ms


def descriptor():
    return {
        "control_interface": "motus.control/1", "mode": "twist", "dof": 6,
        "joint_names": ["vx", "vy", "vz", "wx", "wy", "wz"],
        "units": {"linear": "m/s", "angular": "rad/s"},
        "limits": {"lower": [-.3, -.2, 0, 0, 0, -.5],
                   "upper": [.3, .2, 0, 0, 0, .5],
                   "max_delta_per_step": [.03, .03, 1e-6, 1e-6, 1e-6, .05]},
        "rate": {"max_hz": 20, "expected_hz": 10, "watchdog_ms": 300,
                 "max_obs_age_ms": 500},
        "force_torque": None,
    }


def command(clock, **changes):
    value = dict(schema="motus.control/1", mode="twist", dof=6,
                 values=[.2, 0, 0, 0, 0, .4], source="navigation",
                 session_id="first", seq=1, priority=50, ttl_ms=200,
                 stamp_ms=clock.wall, obs_stamp_ms=clock.wall)
    value.update(changes)
    return value


def setup_sink(**kwargs):
    clock = Clock()
    applied, stopped = [], []
    sink = ControlSink(
        descriptor(), kwargs.pop("apply", lambda v, g: applied.append(v)),
        clock=lambda: clock.mono, wall_clock=lambda: clock.wall,
        on_watchdog=kwargs.pop("on_watchdog", lambda: stopped.append(sink.last_stop_reason)),
        strict_stream=True, **kwargs)
    sink.reset(initial_values=[0] * 6, minimum_stamp_ms=clock.wall)
    return sink, clock, applied, stopped


def test_first_twist_slews_from_explicit_rest_baseline():
    sink, clock, applied, _ = setup_sink()
    out = sink.submit(command(clock))
    assert out.verdict == Verdict.CLAMPED
    assert applied == [(0.03, 0, 0, 0, 0, .05)]


@pytest.mark.parametrize("change", [
    {"values": [float("nan"), 0, 0, 0, 0, 0]},
    {"values": [float("inf"), 0, 0, 0, 0, 0]},
    {"values": [.31, 0, 0, 0, 0, 0]},
    {"values": [0, 0, .01, 0, 0, 0]},
    {"dof": True}, {"dof": 6.0}, {"seq": True}, {"seq": -1}, {"seq": 2**63},
    {"ttl_ms": 301}, {"ttl_ms": float("inf")}, {"ttl_ms": 0},
    {"stamp_ms": float("nan")}, {"stamp_ms": 1_000_051},
    {"obs_stamp_ms": None}, {"obs_stamp_ms": True},
    {"obs_stamp_ms": float("inf")}, {"obs_stamp_ms": 1_000_051},
    {"obs_stamp_ms": 999_500}, {"source": "x" * 161},
    {"session_id": "x" * 161}, {"session_id": []},
    {"priority": float("nan")}, {"priority": 50.0},
    {"priority": True}, {"priority": -1}, {"priority": 101},
    {"gripper": float("nan")},
])
def test_strict_contract_refuses_malformed_or_unsafe_input(change):
    sink, clock, applied, _ = setup_sink()
    assert not sink.submit(command(clock, **change)).applied
    assert not applied
    assert sink.stats()["holder"] is None


def test_zero_twist_stops_immediately_without_slew_but_still_checks_source():
    sink, clock, applied, _ = setup_sink()
    sink.submit(command(clock))
    clock.advance(100)
    zero = [0] * 6
    assert not sink.submit(command(clock, source="other", values=zero)).applied
    assert sink.submit(command(clock, seq=2, values=zero)).applied
    assert applied[-1] == tuple(zero)
    clock.advance(500)
    assert sink.tick() is None  # no expired velocity after explicit zero
    assert sink.submit(command(clock, seq=3)).values[0] == .03


def test_resume_epoch_discards_old_packets_and_resets_sequence():
    sink, clock, applied, _ = setup_sink()
    previous = command(clock, seq=10)
    sink.submit(previous)
    sink.stop("pause")
    clock.advance(10)
    sink.reset(initial_values=[0] * 6, minimum_stamp_ms=clock.wall)
    assert sink.submit(previous).verdict == Verdict.DROPPED
    assert sink.submit(command(clock)).applied
    assert applied[-1][0] == .03


def test_session_change_requires_explicit_reset_even_with_increasing_sequence():
    sink, clock, _, _ = setup_sink()
    sink.submit(command(clock))
    assert not sink.submit(command(clock, seq=2, session_id="second")).applied
    assert sink.submit(command(clock, seq=2)).applied


def test_source_lease_and_sequence_only_advance_after_successful_application():
    sink, clock, applied, _ = setup_sink()
    assert sink.submit(command(clock)).applied
    assert not sink.submit(command(clock, source="other", seq=20)).applied
    assert sink.submit(command(clock, source="other", seq=1, priority=60)).applied
    assert len(applied) == 2


def test_rejected_raw_limit_cannot_steal_source_or_consume_its_sequence():
    sink, clock, _, _ = setup_sink()
    assert not sink.submit(command(clock, values=[1, 0, 0, 0, 0, 0])).applied
    assert sink.stats()["holder"] is None
    assert sink.submit(command(clock)).applied


def test_distinct_sources_are_bounded_until_reset():
    sink, clock, _, _ = setup_sink()
    for i in range(32):
        assert sink.submit(command(clock, source=f"source-{i}", priority=i)).applied
    assert not sink.submit(command(clock, source="overflow", priority=99)).applied
    sink.reset(initial_values=[0] * 6)
    assert sink.submit(command(clock, source="overflow", priority=99)).applied


def test_ttl_is_enforced_exactly_at_deadline_even_with_zero_monotonic_epoch():
    sink, clock, _, stopped = setup_sink()
    sink.submit(command(clock))
    clock.advance(199)
    assert sink.tick() is None
    clock.advance(1)
    assert sink.tick().verdict == Verdict.DROPPED
    assert len(stopped) == 1
    assert "deadline" in stopped[0]
    assert sink.tick() is None


def test_observation_expiry_stops_earlier_than_command_ttl():
    sink, clock, _, stopped = setup_sink()
    sink.submit(command(clock, obs_stamp_ms=clock.wall - 450))
    clock.advance(50)
    sink.tick()
    assert len(stopped) == 1


def test_wall_clock_rollback_cannot_extend_an_active_velocity():
    sink, clock, _, stopped = setup_sink()
    sink.submit(command(clock))
    clock.mono += 200
    clock.wall -= 60_000
    sink.tick()
    assert len(stopped) == 1


def test_slow_posture_gate_rechecks_freshness_before_actuator_write():
    sink, clock, applied, stopped = setup_sink()
    sink._before_apply = lambda *_: clock.advance(201)
    assert sink.submit(command(clock)).verdict == Verdict.DROPPED
    assert applied == []
    assert len(stopped) == 1


def test_gate_clock_rollback_still_uses_original_monotonic_deadline():
    sink, clock, applied, stopped = setup_sink()

    def gate(*_):
        clock.mono += 201
        clock.wall -= 20

    sink._before_apply = gate
    assert not sink.submit(command(clock)).applied
    assert not applied and len(stopped) == 1


def test_slow_successful_sdk_ack_stops_immediately_when_ttl_is_already_expired():
    sink, clock, applied, stopped = setup_sink()

    def apply(values, _):
        applied.append(values)
        clock.advance(201)

    sink._apply = apply
    assert sink.submit(command(clock)).verdict == Verdict.DROPPED
    assert len(applied) == len(stopped) == 1
    assert sink.stats()["active"] is False


@pytest.mark.parametrize("where", ["before_apply", "apply"])
def test_vendor_fault_stops_and_latches_without_reporting_applied(where):
    sink, clock, applied, stopped = setup_sink()

    def broken(*_):
        raise RuntimeError("SDK rejected command")

    setattr(sink, "_" + where, broken)
    out = sink.submit(command(clock))
    assert out.verdict == Verdict.ABORTED
    assert sink.aborted and "SDK rejected" in sink.fault_reason
    assert len(stopped) == 1
    assert not applied
    assert sink.submit(command(clock, seq=2)).verdict == Verdict.ABORTED
    assert len(stopped) == 1
    assert "applied" not in sink.stats()["counters"]


@pytest.mark.parametrize("where", ["before_apply", "apply"])
def test_adapter_generation_cancel_does_not_fault_or_advance_sink_state(where):
    sink, clock, _, stopped = setup_sink()
    setattr(sink, "_" + where, lambda *_: Outcome(Verdict.DROPPED, "generation changed"))
    assert sink.submit(command(clock)).verdict == Verdict.DROPPED
    assert not sink.aborted and not stopped
    assert sink.stats()["holder"] is None and not sink.stats()["active"]


@pytest.mark.parametrize("trigger", ["stop", "abort", "deadline"])
def test_failed_stop_latches_once_without_recursing(trigger):
    calls = []

    def broken():
        calls.append(1)
        raise RuntimeError("StopMove ret=3104")

    sink, clock, _, _ = setup_sink(on_watchdog=broken)
    if trigger == "deadline":
        sink.submit(command(clock))
        clock.advance(200)
        out = sink.tick()
    else:
        out = getattr(sink, trigger)("requested stop")
    assert out.verdict == Verdict.ABORTED
    assert calls == [1]
    assert sink.aborted and "StopMove ret=3104" in sink.fault_reason
    assert sink.tick() is None


def test_stop_retries_dont_clear_fault_and_reset_is_explicit():
    sink, clock, _, stopped = setup_sink()
    sink.abort("transport lost")
    assert sink.stop("retry stopping").verdict == Verdict.DROPPED
    assert sink.aborted and sink.fault_reason == "transport lost"
    assert stopped == ["transport lost", "retry stopping"]
    sink.reset(initial_values=[0] * 6, minimum_stamp_ms=clock.wall)
    assert not sink.aborted and not sink.fault_reason
    assert sink.submit(command(clock)).applied


def test_stop_preserves_source_sequence_and_success_sets_rest_baseline():
    sink, clock, _, _ = setup_sink()
    sink.submit(command(clock))
    sink.stop("hold")
    assert sink.stats()["holder"] == "navigation"
    assert not sink.submit(command(clock)).applied
    assert sink.submit(command(clock, seq=2)).values[0] == .03


def test_strict_mode_does_not_silently_apply_to_a_position_descriptor():
    d = descriptor()
    d["mode"] = "joint_position"
    with pytest.raises(ValueError, match="twist"):
        ControlSink(d, lambda *_: None, strict_stream=True)


@pytest.mark.parametrize("values,stamp", [([0], None), ([float("nan")] * 6, None),
                                           ([1] * 6, None), ([0] * 6, float("inf"))])
def test_bad_resume_baseline_or_epoch_does_not_clear_fault(values, stamp):
    sink, _, _, _ = setup_sink()
    sink.abort("fault")
    with pytest.raises(ValueError):
        sink.reset(initial_values=values, minimum_stamp_ms=stamp)
    assert sink.aborted


def test_legacy_watchdog_also_fires_when_first_apply_was_at_monotonic_zero():
    clock = Clock()
    stopped = []
    sink = ControlSink(descriptor(), lambda *_: None, clock=lambda: clock.mono,
                       wall_clock=lambda: clock.wall,
                       on_watchdog=lambda: stopped.append(1))
    sink.submit(command(clock))
    clock.advance(300)
    assert sink.tick().verdict == Verdict.DROPPED
    assert stopped == [1]


def test_default_policy_allows_equal_priority_source_takeover():
    # Strict streams keep an equal-priority competing source out. Existing
    # callers that omit strict_stream must retain their original tie policy.
    clock = Clock()
    applied = []
    sink = ControlSink(descriptor(), lambda values, _: applied.append(values),
                       clock=lambda: clock.mono, wall_clock=lambda: clock.wall)
    assert sink.submit(command(clock, source="first", priority=50)).applied
    assert sink.submit(command(clock, source="second", priority=50)).applied
    assert len(applied) == 2
    assert sink.stats()["holder"] == "second"


def test_strict_watchdog_before_skewed_ttl_stops_once_and_resumes_from_rest():
    sink, clock, applied, stopped = setup_sink()
    # Allowed future skew yields a 350 ms TTL deadline, after the 300 ms
    # watchdog. Both ways of stopping must clear the old velocity baseline.
    assert sink.submit(command(clock, stamp_ms=clock.wall + 50,
                               obs_stamp_ms=clock.wall + 50, ttl_ms=300)).applied
    assert applied[-1][0] == .03
    clock.advance(300)
    assert sink.tick().verdict == Verdict.DROPPED
    assert stopped == ["watchdog: no valid command for 300 ms"]
    assert sink.stats()["active"] is False
    clock.advance(50)
    assert sink.tick() is None
    assert sink.submit(command(clock, seq=2)).values[0] == .03
    assert len(stopped) == 1
