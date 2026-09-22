"""motus.odom/1 — the declaration, the sample, and the null-versus-zero rule.

The tests that matter here are the ones about `None`. Everything else in this
format is ordinary validation; the distinction between "measured zero" and "not
measured" is the reason the format exists, and it is the one a well-meaning
refactor erases (`or 0.0` is such a natural thing to type).
"""
from __future__ import annotations

import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from common.odom import (  # noqa: E402
    AXES,
    BODY_FRAME,
    SCHEMA,
    OdomError,
    axis,
    build_interface,
    build_sample,
    is_fresh,
    parse_interface,
)


# ── the axes are motus.control/1's twist axes ────────────────────────────────

def test_axis_order_matches_control_twist():
    """The single property this format is built on.

    `motus.control/1` documents twist as (vx, vy, vz, wx, wy, wz); a consumer
    subtracts measured from commanded index by index. If these ever diverge, a
    stuck-detector starts comparing forward speed against yaw rate and reports
    nonsense that looks like a robot fault.
    """
    from common.control.descriptor import MODES

    assert "twist" in MODES
    assert AXES == ("vx", "vy", "vz", "wx", "wy", "wz")


# ── null is not zero ─────────────────────────────────────────────────────────

def test_unmeasured_axes_survive_as_none():
    sample = build_sample(stamp_ms=1000, twist=[0.31, 0.02, None, None, None, -0.42])
    assert sample["twist"] == [0.31, 0.02, None, None, None, -0.42]
    assert axis(sample, "vz") is None
    assert axis(sample, "wz") == -0.42


def test_a_measured_zero_is_not_the_same_as_an_unmeasured_axis():
    """The whole point. A robot standing still and a robot with no odometry
    must not look alike, or a stuck-detector fires on every robot without it."""
    still = build_sample(stamp_ms=1000, twist=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    blind = build_sample(stamp_ms=1000, twist=[None] * 6)

    assert axis(still, "vx") == 0.0
    assert axis(blind, "vx") is None
    assert axis(still, "vx") != axis(blind, "vx")
    # and the difference must survive the wire
    import json
    assert json.loads(json.dumps(blind))["twist"] == [None] * 6


def test_axis_of_a_short_or_missing_twist_is_none_not_zero():
    assert axis({}, "vx") is None
    assert axis({"twist": [1.0]}, "wz") is None


def test_axis_rejects_a_name_that_is_not_an_axis():
    with pytest.raises(OdomError, match="yaw_speed"):
        axis(build_sample(stamp_ms=0, twist=[None] * 6), "yaw_speed")


def test_a_twist_of_the_wrong_width_is_refused():
    """Silently padding would put a value on the wrong axis, which is the
    single worst outcome available to this format."""
    with pytest.raises(OdomError, match="expected 6"):
        build_sample(stamp_ms=0, twist=[0.1, 0.2, 0.3])


# ── freshness ────────────────────────────────────────────────────────────────

def test_a_sample_without_a_stamp_is_never_fresh():
    assert is_fresh({"twist": [0.1] * 6}, now_ms=1000, max_age_ms=500) is False


def test_freshness_window():
    sample = build_sample(stamp_ms=1000, twist=[0.1] * 6)
    assert is_fresh(sample, now_ms=1400, max_age_ms=500) is True
    assert is_fresh(sample, now_ms=1600, max_age_ms=500) is False


def test_a_stamp_from_the_future_is_not_fresh():
    """Clock skew between a driver and a consumer is real, and a future stamp
    would otherwise stay 'fresh' forever."""
    sample = build_sample(stamp_ms=9000, twist=[0.1] * 6)
    assert is_fresh(sample, now_ms=1000, max_age_ms=500) is False


# ── the declaration ──────────────────────────────────────────────────────────

def test_a_built_interface_round_trips():
    raw = build_interface(provides=["vx", "vy", "wz"], rate_hz=10,
                          pose_drift="unbounded")
    parsed = parse_interface(raw)
    assert parsed.frame == BODY_FRAME
    assert parsed.has("vx") and not parsed.has("vz")
    assert parsed.rate_hz == 10.0
    assert parsed.pose_drift == "unbounded"
    assert parsed.usable_for_control is True


def test_a_world_frame_report_is_valid_but_not_usable_for_control():
    """World-frame odometry is real and worth publishing; comparing it against
    a body-frame command needs a rotation, which is a different feature. The
    consumer must be able to tell, rather than treating world as body."""
    parsed = parse_interface(build_interface(provides=["vx"], frame="world"))
    assert parsed.usable_for_control is False


def test_the_frame_must_be_stated():
    with pytest.raises(OdomError, match="frame"):
        parse_interface({"schema": SCHEMA, "provides": ["vx"]})


def test_an_unknown_axis_name_is_refused_with_the_real_list():
    with pytest.raises(OdomError, match="vyaw"):
        parse_interface({"schema": SCHEMA, "frame": "body", "provides": ["vyaw"]})


def test_the_wrong_schema_is_refused():
    with pytest.raises(OdomError, match="motus.odom/1"):
        parse_interface({"schema": "motus.control/1", "frame": "body",
                         "provides": []})


def test_build_interface_validates_what_it_builds():
    """A driver must fail in its own unit test, not on a robot."""
    with pytest.raises(OdomError):
        build_interface(provides=["vx"], pose_drift="probably fine")


# ── vendor extension ─────────────────────────────────────────────────────────

def test_vendor_fields_pass_through_untouched():
    """Adopting this format must cost a driver nothing it already reports."""
    vendor = {"mode": 811, "gait_type": 1, "body_height": 0.78,
              "nested": {"anything": [1, 2, 3]}}
    sample = build_sample(stamp_ms=1, twist=[None] * 6, vendor=vendor)
    assert sample["vendor"] == vendor


def test_a_core_consumer_is_unaffected_by_vendor_growth():
    plain = build_sample(stamp_ms=1, twist=[0.5] + [None] * 5)
    extended = build_sample(stamp_ms=1, twist=[0.5] + [None] * 5,
                            vendor={"something_new": 42})
    assert axis(plain, "vx") == axis(extended, "vx") == 0.5


def test_pose_is_absent_by_default_rather_than_invented():
    assert build_sample(stamp_ms=1, twist=[None] * 6)["pose"] is None
