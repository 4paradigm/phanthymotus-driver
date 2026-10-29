"""Offline admission checks; no ROS or robot dependencies."""
import concurrent.futures
import math
import threading

import pytest

from unitree.g1.loco_admission import LocoAdmission
from unitree.g1.loco_descriptor import validate_loco_descriptor


NOW = 1000


def raw_descriptor():
    return {"control_interface": "motus.control/1", "mode": "twist", "dof": 6,
            "joint_names": ["vx", "vy", "vz", "wx", "wy", "wz"],
            "frame": "base_link", "units": {"linear": "m/s", "angular": "rad/s", "time": "s"},
            "limits": {"lower": [-1, -1, 0, 0, 0, -2],
                       "upper": [1, 1, 0, 0, 0, 2],
                       "max_delta_per_step": [0.1] * 6},
            "rate": {"watchdog_ms": 200, "max_obs_age_ms": 100,
                     "max_hz": 100, "expected_hz": 30}, "force_torque": None}


def make_gate(desc=None):
    if desc is None:
        desc = validate_loco_descriptor(raw_descriptor())
    return LocoAdmission(desc, session="session-1", source="planner",
                         max_command_age_ms=80, max_ttl_ms=100,
                         max_obs_age_ms=60, future_tolerance_ms=5,
                         max_session_grants=4, max_session_id_length=32)


def msg(**changes):
    data = {"schema": "motus.control/1", "mode": "twist", "dof": 6,
            "values": [0.1, 0, 0, 0, 0, 0.2], "source": "planner", "seq": 1,
            "stamp_ms": NOW, "obs_stamp_ms": NOW, "ttl_ms": 100}
    data.update(changes)
    return data


def check(gate, message, **kwargs):
    return gate.validate(message, session=kwargs.get("session", "session-1"),
                         now_ms=kwargs.get("now", NOW))


@pytest.mark.parametrize("changes", [
    {"schema": "wrong", "stamp_ms": None}, {"mode": "joint_position"},
    {"dof": 3}, {"values": [0] * 5}, {"source": "unknown"},
    {"values": [math.nan, 0, 0, 0, 0, 0]},
    {"values": [math.inf, 0, 0, 0, 0, 0]},
    {"values": [True, 0, 0, 0, 0, 0]},
    {"values": [0, 0, 1e-12, 0, 0, 0]},
    {"values": [0, 0, 0, -1e-12, 0, 0]},
    {"values": [0, 0, 0, 0, 1e-12, 0]},
    {"values": [1.01, 0, 0, 0, 0, 0]},
    {"values": [0, 0, 0, 0, 0, -2.01]},
    {"stamp_ms": NOW - 81}, {"stamp_ms": NOW + 6}, {"stamp_ms": None},
    {"ttl_ms": 101}, {"ttl_ms": 0}, {"ttl_ms": None}, {"ttl_ms": math.inf},
    {"obs_stamp_ms": None}, {"obs_stamp_ms": NOW - 61},
    {"obs_stamp_ms": NOW + 6}, {"obs_stamp_ms": math.nan},
    {"seq": True}, {"seq": 10**1000}, {"priority": math.nan}, {"gripper": math.inf},
])
def test_rejection_does_not_consume_sequence(changes):
    gate = make_gate()
    with pytest.raises(ValueError):
        check(gate, msg(**changes))
    assert check(gate, msg()) == (0.1, 0, 0, 0, 0, 0.2)


@pytest.mark.parametrize("field", ["stamp_ms", "ttl_ms", "obs_stamp_ms", "seq", "source"])
def test_missing_field_rejected_without_consuming_sequence(field):
    gate = make_gate()
    command = msg()
    del command[field]
    with pytest.raises(ValueError):
        check(gate, command)
    assert check(gate, msg())


@pytest.mark.parametrize("name,value", [
    ("max_command_age_ms", math.inf), ("max_ttl_ms", 0),
    ("max_obs_age_ms", True), ("future_tolerance_ms", -1),
    ("max_session_grants", 0), ("max_session_id_length", True),
])
def test_invalid_configuration_fails_closed(name, value):
    settings = dict(session="session-1", source="planner", max_command_age_ms=80,
                    max_ttl_ms=100, max_obs_age_ms=60, future_tolerance_ms=5,
                    max_session_grants=4, max_session_id_length=32)
    settings[name] = value
    with pytest.raises(ValueError):
        LocoAdmission(validate_loco_descriptor(raw_descriptor()), **settings)


def test_contract_precedes_freshness():
    with pytest.raises(ValueError, match="schema"):
        check(make_gate(), msg(schema="bad", stamp_ms=None))


def test_boundaries_and_replay():
    gate = make_gate()
    assert check(gate, msg(values=[1, -1, 0, 0, 0, 2], stamp_ms=NOW-60,
                           obs_stamp_ms=NOW-60)) == (1, -1, 0, 0, 0, 2)
    with pytest.raises(ValueError, match="seq"):
        check(gate, msg(seq=1))
    with pytest.raises(ValueError, match="seq"):
        check(gate, msg(seq=0))
    assert check(gate, msg(seq=2, stamp_ms=NOW+5, obs_stamp_ms=NOW+5))


def test_ttl_expiry_and_explicit_authorization():
    gate = make_gate()
    with pytest.raises(ValueError, match="expired"):
        check(gate, msg(stamp_ms=NOW-11, ttl_ms=10))
    check(gate, msg())
    with pytest.raises(ValueError, match="authorized"):
        check(gate, msg(seq=2), session="session-2")
    with pytest.raises(ValueError, match="authorized"):
        check(gate, msg(seq=2, source="other"))
    gate.authorize(session="session-2", source="other")
    assert check(gate, msg(source="other"), session="session-2")
    with pytest.raises(ValueError, match="authorized"):
        check(gate, msg(seq=3))


def test_observation_must_not_postdate_command():
    gate = make_gate()
    with pytest.raises(ValueError, match="observation is later"):
        check(gate, msg(stamp_ms=NOW-1, obs_stamp_ms=NOW))
    assert check(gate, msg(stamp_ms=NOW-1, obs_stamp_ms=NOW-1))
    assert check(gate, msg(seq=2, stamp_ms=NOW, obs_stamp_ms=NOW-1))


def test_session_ids_never_reused_and_failed_grants_preserve_state():
    gate = make_gate()
    check(gate, msg())
    with pytest.raises(ValueError, match="already used"):
        gate.authorize(session="session-1", source="planner")
    with pytest.raises(ValueError, match="seq"):
        check(gate, msg())
    gate.authorize(session="session-2", source="planner")
    check(gate, msg(), session="session-2")
    with pytest.raises(ValueError, match="already used"):
        gate.authorize(session="session-1", source="other")
    with pytest.raises(ValueError, match="seq"):
        check(gate, msg(), session="session-2")


def test_session_history_capacity_and_length_fail_closed():
    gate = make_gate()
    with pytest.raises(ValueError, match="too long"):
        gate.authorize(session="x" * 33, source="planner")
    for session in ("session-2", "session-3", "session-4"):
        gate.authorize(session=session, source="planner")
    with pytest.raises(ValueError, match="capacity"):
        gate.authorize(session="session-5", source="planner")
    with pytest.raises(ValueError, match="already used"):
        gate.authorize(session="session-1", source="planner")
    assert check(gate, msg(), session="session-4")


def test_concurrent_duplicate_session_grants_only_one_succeeds():
    gate = make_gate()
    barrier = threading.Barrier(12)
    def attempt(_):
        barrier.wait()
        try:
            gate.authorize(session="session-2", source="planner")
            return True
        except ValueError:
            return False
    with concurrent.futures.ThreadPoolExecutor(max_workers=12) as pool:
        assert sum(pool.map(attempt, range(12))) == 1
    assert check(gate, msg(), session="session-2")
    with pytest.raises(ValueError, match="seq"):
        check(gate, msg(), session="session-2")


def test_descriptor_and_message_are_snapshots():
    raw = raw_descriptor()
    parsed = validate_loco_descriptor(raw)
    gate = make_gate(parsed)
    raw["limits"]["upper"][0] = 999
    parsed.raw["limits"]["upper"][0] = 999
    command = msg()
    result = check(gate, command)
    command["values"][0] = 999
    assert result[0] == 0.1
    with pytest.raises(ValueError, match="bounds"):
        check(gate, msg(seq=2, values=[2, 0, 0, 0, 0, 0]))
    assert check(gate, msg(seq=2))


def test_descriptor_must_be_g1_validated_and_consistent():
    with pytest.raises(ValueError):
        make_gate(raw_descriptor())
    parsed = validate_loco_descriptor(raw_descriptor())
    parsed.raw["limits"]["upper"][0] = 5
    with pytest.raises(ValueError):
        make_gate(parsed)


@pytest.mark.parametrize("now", [math.nan, math.inf, True])
def test_invalid_clock_does_not_consume_seq(now):
    gate = make_gate()
    with pytest.raises(ValueError):
        check(gate, msg(), now=now)
    assert check(gate, msg())


def test_concurrent_same_sequence_only_one_succeeds():
    gate = make_gate()
    barrier = threading.Barrier(12)
    def attempt(_):
        barrier.wait()
        try:
            check(gate, msg())
            return True
        except ValueError:
            return False
    with concurrent.futures.ThreadPoolExecutor(max_workers=12) as pool:
        assert sum(pool.map(attempt, range(12))) == 1
