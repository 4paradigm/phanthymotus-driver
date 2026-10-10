"""Bumi's `motus.odom/1` — which axes it claims, and what the others report.

The whole risk in this file is one substitution. Bumi's SDK exposes an IMU,
joint states and a battery, and **no translational velocity anywhere**. Writing
`0.0` for `vx` instead of `None` would produce a perfectly well-formed sample
that says the robot measured itself standing still — and a consumer's stuck
detector ("commanded 0.3 m/s, measured nothing, therefore we have hit
something") would then fire on every step of a robot that simply cannot answer.

Nothing raises when that goes wrong, on either side. So it is asserted here,
which is why `odom_spec.py` does not import rclpy.

Run: PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest tests/test_bumi_odom.py -q
"""

from __future__ import annotations

import importlib.util
import sys
import time
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))


def _load():
    bundle = ROOT / "noetix" / "bumi"
    sys.path.insert(0, str(bundle))
    try:
        spec = importlib.util.spec_from_file_location(
            "bumi_odom_spec", bundle / "odom_spec.py")
        module = importlib.util.module_from_spec(spec)
        sys.modules["bumi_odom_spec"] = module
        spec.loader.exec_module(module)
        return module
    finally:
        sys.path.remove(str(bundle))


odom_spec = _load()

from common.odom import (  # noqa: E402
    AXES, axis, is_fresh, parse_interface)

NOW_MS = 1_800_000_000_000


# ── the declaration ──────────────────────────────────────────────────────────

def test_the_interface_is_valid():
    interface = parse_interface(odom_spec.interface())
    assert interface.frame == "body"
    assert interface.usable_for_control


def test_only_the_three_angular_axes_are_claimed():
    """Absent rather than listed-and-null, so a consumer can decide at start
    whether it can do its job instead of finding out at 10 Hz."""
    interface = parse_interface(odom_spec.interface())
    assert set(interface.provides) == {"wx", "wy", "wz"}
    for missing in ("vx", "vy", "vz"):
        assert not interface.has(missing)


def test_wz_is_claimed_because_it_is_the_axis_a_tracker_leans_on():
    """Self-rotation is the main reason a target moves across the image, so
    this is the half of the format that is actually available on Bumi."""
    assert parse_interface(odom_spec.interface()).has("wz")


def test_no_pose_is_declared_as_none_rather_than_unbounded():
    """`unbounded` would say a position is reported and cannot be trusted. Here
    there is no position at all, and a consumer may act differently on that."""
    assert parse_interface(odom_spec.interface()).pose_drift == "none"


def test_the_declared_rate_matches_the_publish_interval():
    """A declaration that outruns the publisher makes a consumer's freshness
    window too tight, and every sample reads as late."""
    interface = parse_interface(odom_spec.interface())
    assert interface.rate_hz == pytest.approx(1.0 / odom_spec.PUBLISH_S)


def test_the_publish_rate_clears_navis_observation_window():
    """navi's `max_obs_age_ms` is 500 ms. A 2 Hz publisher lands exactly on it,
    so half the samples read as stale and the tracker intermittently forgets the
    robot is turning — which is why this does not ride on the 2 Hz joint poll."""
    assert odom_spec.PUBLISH_S <= 0.25


# ── the sample ───────────────────────────────────────────────────────────────

def test_a_reading_leaves_the_translational_axes_none():
    """The one substitution this file exists to prevent."""
    row = odom_spec.reading([0.1, -0.2, 0.3])
    assert row[:3] == [None, None, None]
    assert row[3:] == [0.1, -0.2, 0.3]


def test_the_axis_order_is_the_control_order():
    """Commanded and measured have to line up index by index, or every
    comparison a consumer makes is between two different axes."""
    row = odom_spec.reading([1.0, 2.0, 3.0])
    assert len(row) == len(AXES)
    sample = odom_spec.sample([row], received_ms=NOW_MS)
    assert axis(sample, "wx") == 1.0
    assert axis(sample, "wy") == 2.0
    assert axis(sample, "wz") == 3.0


def test_a_published_sample_reports_null_speed_not_zero_speed():
    sample = odom_spec.sample([odom_spec.reading([0.0, 0.0, 0.5])],
                              received_ms=NOW_MS)
    assert sample["twist"][:3] == [None, None, None]
    assert axis(sample, "vx") is None
    assert axis(sample, "wz") == 0.5


def test_a_standing_still_robot_is_distinguishable_from_a_silent_one():
    """Both give an all-`None` vx. The difference has to be readable somewhere,
    and `vendor.samples` is it."""
    still = odom_spec.sample([odom_spec.reading([0.0, 0.0, 0.0])],
                             received_ms=NOW_MS)
    silent = odom_spec.sample([], received_ms=NOW_MS)
    assert still["vendor"]["samples"] == 1 and axis(still, "wz") == 0.0
    assert silent["vendor"]["samples"] == 0 and axis(silent, "wz") is None


def test_a_window_is_averaged_rather_than_decimated():
    """A consumer comparing a commanded rate against a measured one is reading a
    threshold crossing, and a noisy sample crosses thresholds it should not."""
    readings = [odom_spec.reading([0.0, 0.0, value])
                for value in (0.2, 0.4, 0.6, 0.8)]
    assert axis(odom_spec.sample(readings, received_ms=NOW_MS), "wz") == 0.5


def test_pose_is_null_because_there_is_none_to_report():
    assert odom_spec.sample([], received_ms=NOW_MS)["pose"] is None


def test_the_frame_is_body():
    """The most dangerous ambiguity in this format: a consumer that assumes
    wrong gets plausible numbers with a sign flipped."""
    assert odom_spec.sample([], received_ms=NOW_MS)["frame"] == "body"


def test_the_stamp_says_it_is_an_arrival_time():
    """The SDK's IMU struct carries no timestamp, so there is no robot clock to
    quote — and a consumer must be able to tell that from a quoted one."""
    sample = odom_spec.sample([], received_ms=NOW_MS)
    assert sample["stamp_ms"] == NOW_MS
    assert sample["vendor"]["stamp_source"] == "received"


def test_a_fresh_sample_reads_as_fresh():
    """`is_fresh` subtracts `stamp_ms` from the consumer's clock, so a sample
    stamped on arrival has to survive that round trip."""
    now_ms = int(time.time() * 1000)
    sample = odom_spec.sample([odom_spec.reading([0.0, 0.0, 0.1])],
                              received_ms=now_ms)
    assert is_fresh(sample, now_ms, 500)


def test_the_workmode_rides_along_in_vendor_not_in_the_core():
    """Vendor-specific things go in `vendor`, untouched — a consumer written
    against the core cannot be broken by a driver adding to it."""
    sample = odom_spec.sample([], received_ms=NOW_MS, workmode=2,
                              workmode_name="walking")
    assert sample["vendor"]["workmode_name"] == "walking"
    assert "workmode" not in sample


def test_an_unreadable_workmode_is_simply_absent():
    sample = odom_spec.sample([], received_ms=NOW_MS)
    assert "workmode" not in sample["vendor"]


def test_the_health_note_states_the_limitation_in_words():
    """`provides` is machine-readable and an operator does not read it. The
    reason a stuck detector cannot work on this robot has to be sayable."""
    assert "vx/vy/vz" in odom_spec.HEALTH_NOTE
