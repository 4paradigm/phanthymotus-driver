"""Compatibility tests for the shared, side-effect-free control checks."""
import copy
import math

import pytest

from common.control import ControlSink, Verdict, parse_descriptor
from common.control.sink import check_control_contract, check_control_freshness


NOW = 1000


def descriptor():
    return parse_descriptor({
        "control_interface": "motus.control/1", "mode": "twist", "dof": 2,
        "joint_names": ["vx", "wz"], "units": {"linear": "m/s"},
        "limits": {"lower": [-1, -1], "upper": [1, 1]},
        "rate": {"watchdog_ms": 100, "max_obs_age_ms": 50},
        "force_torque": None,
    })


def message(**changes):
    result = {"schema": "motus.control/1", "mode": "twist", "dof": 2,
              "values": [0.1, 0.0], "source": "test", "seq": 2,
              "stamp_ms": NOW, "ttl_ms": 100, "obs_stamp_ms": NOW,
              "priority": 0}
    result.update(changes)
    return result


def sink_with(d, calls):
    return ControlSink(d, lambda *args: calls.append(args),
                       clock=lambda: NOW, wall_clock=lambda: NOW)


@pytest.mark.parametrize("changes", [
    {}, {"schema": "wrong", "mode": "wrong"}, {"mode": "joint_position"},
    {"dof": 3}, {"values": [0.0]}, {"values": [math.nan, 0]},
    {"values": [math.inf, 0]}, {"values": [True, 0]}, {"source": ""},
])
def test_contract_matches_submit_and_does_not_mutate(changes):
    d = descriptor()
    msg = message(**changes)
    original = copy.deepcopy(msg)
    d_original = copy.deepcopy(d)
    calls = []
    sink = sink_with(d, calls)
    result = check_control_contract(d, msg)
    assert msg == original and d == d_original
    assert calls == [] and sink.stats()["counters"] == {}
    actual = sink.submit(msg)
    if result is None:
        assert actual.verdict == Verdict.APPLIED
    else:
        assert actual == result
        assert not calls


@pytest.mark.parametrize("changes", [
    {}, {"stamp_ms": NOW - 101}, {"stamp_ms": NOW - 100},
    {"stamp_ms": NOW + 100}, {"stamp_ms": math.nan}, {"ttl_ms": 0},
    {"ttl_ms": math.inf}, {"obs_stamp_ms": None},
    {"obs_stamp_ms": NOW - 51}, {"obs_stamp_ms": NOW - 50},
    {"obs_stamp_ms": NOW + 100}, {"obs_stamp_ms": math.inf},
    {"seq": 1}, {"seq": 0}, {"seq": True},
])
def test_freshness_matches_submit_without_committing_sequence(changes):
    d = descriptor()
    calls = []
    sink = sink_with(d, calls)
    assert sink.submit(message(seq=1)).applied
    calls.clear()
    msg = message(**changes)
    original = copy.deepcopy(msg)
    previous = {"test": 1}
    result = check_control_freshness(d, msg, NOW, previous)
    assert previous == {"test": 1} and msg == original and not calls
    actual = sink.submit(msg)
    if result is None:
        assert actual.verdict == Verdict.APPLIED
    else:
        assert actual == result
        assert not calls


def test_helper_success_does_not_commit_sink_or_source_state():
    d = descriptor()
    calls = []
    sink = sink_with(d, calls)
    msg = message()
    sequences = {}
    before = copy.deepcopy(sink.stats())
    assert check_control_contract(d, msg) is None
    assert check_control_freshness(d, msg, NOW, sequences) is None
    assert sequences == {} and sink.stats() == before and calls == []
    assert sink.submit(msg).applied
    assert sink.submit(msg).verdict == Verdict.DROPPED


def test_eef_pose_quaternion_contract_remains_shared():
    d = parse_descriptor({
        "control_interface": "motus.control/1", "mode": "eef_pose", "dof": 7,
        "joint_names": list("abcdefg"), "units": {"length": "m"},
        "limits": {"lower": [-1] * 7, "upper": [1] * 7},
        "rate": {"watchdog_ms": 100}, "force_torque": None,
    })
    msg = message(mode="eef_pose", dof=7, values=[0, 0, 0, 0, 0, 0, 0])
    calls = []
    result = check_control_contract(d, msg)
    assert result is not None and "unit quaternion" in result.reason
    assert sink_with(d, calls).submit(msg) == result
    assert not calls
