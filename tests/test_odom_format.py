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
    mean_twist,
    parse_interface,
    resolve_stamp_ms,
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


# ── averaging a high-rate vendor topic down to the published rate ────────────
#
# The null-versus-zero rule has to survive this step too, and it is a step where
# treating None as 0 is unusually tempting: `sum(values) / len(values)` is the
# obvious thing to write and it turns an unmeasured axis into a measured one.

def test_mean_twist_averages_each_axis():
    assert mean_twist([[1.0, 0.0, None, None, None, 0.2],
                       [3.0, 0.0, None, None, None, 0.4]]) == \
        [2.0, 0.0, None, None, None, pytest.approx(0.3)]


def test_an_unmeasured_axis_stays_unmeasured():
    """Not 0.0 — the whole reason this format exists."""
    assert mean_twist([[1.0] + [None] * 5] * 3)[1:] == [None] * 5


def test_none_is_not_averaged_in_as_zero():
    """An axis reported intermittently averages over what arrived.

    Go1-shaped partial knowledge is the normal case, and counting the missing
    readings as zeros would report half the speed the robot actually had — a
    wrong number rather than a missing one, which a consumer cannot detect.
    """
    readings = [[2.0, None, None, None, None, None],
                [None, None, None, None, None, None],
                [4.0, None, None, None, None, None]]
    assert mean_twist(readings)[0] == 3.0


def test_an_empty_window_is_all_null():
    """Nothing arrived is not the same fact as the robot standing still."""
    assert mean_twist([]) == [None] * len(AXES)


def test_mean_twist_ignores_junk_without_dropping_the_axis():
    assert mean_twist([[1.0] + [None] * 5,
                       ["fast"] + [None] * 5,
                       [True] + [None] * 5,
                       [3.0] + [None] * 5])[0] == 2.0


def test_averaging_actually_suppresses_noise():
    """The reason to average rather than pick one reading of every fifty.

    A stuck-detector reads a threshold crossing, so what matters is not the mean
    over a long run but how far a single published sample can sit from the truth.
    """
    truth = 0.30
    noise = [+0.25, -0.25] * 25
    readings = [[truth + n] + [None] * 5 for n in noise]
    picked = readings[0][0]                       # what decimation published
    averaged = mean_twist(readings)[0]
    assert abs(averaged - truth) < abs(picked - truth)
    assert averaged == pytest.approx(truth)


# ── which clock a stamp came from ────────────────────────────────────────────

def test_a_plausible_vendor_stamp_is_quoted():
    """The spec asks for when the reading was taken, and normally we know."""
    received = 1_758_537_600_500
    stamp, provenance = resolve_stamp_ms(vendor_ms=received - 40,
                                         received_ms=received)
    assert stamp == received - 40
    assert provenance["stamp_source"] == "robot"
    assert provenance["stamp_skew_ms"] == 40


def test_a_boot_relative_clock_is_refused():
    """`sec` since power-on is a fine relative time and a nonsense absolute one.

    Quoting it would make every sample read as decades old to `is_fresh`, which
    is worse than the publish time it replaced: a consumer would stop the robot
    for blindness it does not have.
    """
    stamp, provenance = resolve_stamp_ms(vendor_ms=3_412_000,
                                         received_ms=1_758_537_600_500)
    assert stamp == 1_758_537_600_500
    assert provenance["stamp_source"] == "received"
    assert "not a wall clock" in provenance["stamp_reason"]


def test_a_skewed_vendor_clock_is_refused_and_measured():
    received = 1_758_537_600_500
    stamp, provenance = resolve_stamp_ms(vendor_ms=received - 41_000,
                                         received_ms=received)
    assert stamp == received
    assert provenance["stamp_source"] == "received"
    assert provenance["stamp_skew_ms"] == 41_000


def test_a_stamp_from_the_future_is_refused_too():
    """`is_fresh` rejects a negative age, so a fast clock reads as never fresh."""
    received = 1_758_537_600_500
    stamp, provenance = resolve_stamp_ms(vendor_ms=received + 41_000,
                                         received_ms=received)
    assert stamp == received
    assert provenance["stamp_skew_ms"] == -41_000


@pytest.mark.parametrize("bad", [None, True, "now", float("nan")])
def test_an_unusable_vendor_stamp_falls_back_and_says_so(bad):
    stamp, provenance = resolve_stamp_ms(vendor_ms=bad, received_ms=17)
    assert stamp == 17
    assert provenance["stamp_source"] == "received"
    assert provenance["stamp_reason"]


def test_the_fallback_is_still_fresh_against_our_own_clock():
    """The property that makes falling back the right call, not a shrug."""
    now = 1_758_537_600_500
    stamp, _ = resolve_stamp_ms(vendor_ms=3_412_000, received_ms=now)
    assert is_fresh(build_sample(stamp_ms=stamp, twist=[None] * 6),
                    now_ms=now + 20, max_age_ms=500)
