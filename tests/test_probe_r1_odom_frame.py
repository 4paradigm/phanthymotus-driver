"""The frame probe, against runs whose frame we chose.

`scripts/probe_r1_odom_frame.py` exists to settle a question nobody can answer by
reading code, and its answer will be used to change a declaration that a robot's
stuck detection depends on. So the analyser is tested the only way it can be:
generate a robot trajectory, express its velocity in a known frame, and check the
probe names that frame.

The indeterminate cases matter as much as the positive ones. A probe that reports
a verdict from a run carrying no information is worse than no probe — it would
produce a confident wrong answer from a robot walking in a straight line, which
is what a first attempt on hardware looks like.

Run:  PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest tests/test_probe_r1_odom_frame.py -q
"""
from __future__ import annotations

import importlib.util
import math
import os
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

# Loaded by path: `scripts/` is not a package, and importing the module must not
# require the SDK — everything the SDK touches is inside `_collect`.
_spec = importlib.util.spec_from_file_location(
    "probe_r1_odom_frame", ROOT / "scripts" / "probe_r1_odom_frame.py")
probe = importlib.util.module_from_spec(_spec)
sys.modules["probe_r1_odom_frame"] = probe
_spec.loader.exec_module(probe)


DT = 0.1


def walk(*, speed=0.5, yaw_rate=0.0, seconds=20.0, frame="body", lateral=0.0):
    """A robot walking with a constant body-frame twist, reported in `frame`.

    Returns rows shaped like the probe's collector: `(t, x, y, vx, vy, yaw)`,
    with `position` always world (as `rt/odommodestate` reports it) and the
    velocity pair expressed in whichever frame the caller asked for. That is
    precisely the ambiguity under test.
    """
    steps = int(seconds / DT)
    yaws = [step * DT * yaw_rate for step in range(steps)]
    # World-frame velocity of a body moving (speed, lateral) at each heading.
    world = [(speed * math.cos(a) - lateral * math.sin(a),
              speed * math.sin(a) + lateral * math.cos(a)) for a in yaws]

    # Trapezoid, because that is the integral the probe inverts: it differences
    # `position` and compares against the mean of the two bracketing velocities.
    # Advancing with forward Euler instead leaves a half-step bias of
    # `yaw_rate·dt/2` in every pair — 2% of the path here — which would show up as
    # a residual of the generator's making and get mistaken for the probe's.
    xs, ys = [0.0], [0.0]
    for (vx0, vy0), (vx1, vy1) in zip(world, world[1:]):
        xs.append(xs[-1] + (vx0 + vx1) / 2.0 * DT)
        ys.append(ys[-1] + (vy0 + vy1) / 2.0 * DT)

    rows = []
    for step in range(steps):
        reported = world[step] if frame == "world" else (speed, lateral)
        rows.append((step * DT, xs[step], ys[step],
                     reported[0], reported[1], yaws[step]))
    return rows


# ── the two answers it exists to tell apart ──────────────────────────────────

@pytest.mark.parametrize("frame", ["body", "world"])
def test_it_names_the_frame_the_run_was_generated_in(frame):
    result = probe.analyse(walk(frame=frame, yaw_rate=0.4))
    assert result["verdict"] == frame
    assert result["ratio"] > 1.5


def test_a_strafing_run_is_also_decided():
    """R1 can translate sideways, so vy is not always zero on a real run."""
    result = probe.analyse(walk(frame="world", yaw_rate=0.3, lateral=0.25))
    assert result["verdict"] == "world"


def test_the_residual_of_the_right_hypothesis_is_near_zero():
    """Not merely smaller — the arithmetic has to actually close.

    If both residuals are large the robot's `position` is not the integral of its
    `velocity` at all, and then the smaller of the two means nothing. This is the
    assertion that distinguishes "we identified the frame" from "we ranked two
    wrong models".
    """
    rows = walk(frame="body", yaw_rate=0.4)
    result = probe.analyse(rows)
    assert result["body_residual_m"] < 0.01 * result["travel_m"]


# ── the runs it must refuse to answer from ───────────────────────────────────

def test_a_straight_walk_cannot_decide():
    """At constant heading the two hypotheses are the same arithmetic.

    This is the run somebody will do first, and the one that would otherwise
    produce a confident answer worth nothing.
    """
    result = probe.analyse(walk(frame="body", yaw_rate=0.0))
    assert result["verdict"] == "indeterminate"
    assert "heading" in result["why"]


def test_a_stationary_robot_cannot_decide():
    result = probe.analyse(walk(speed=0.0, yaw_rate=0.4))
    assert result["verdict"] == "indeterminate"
    assert "moved" in result["why"]


def test_too_few_readings_cannot_decide():
    result = probe.analyse(walk(frame="body", yaw_rate=0.4, seconds=1.0))
    assert result["verdict"] == "indeterminate"
    assert "pairs" in result["why"]


def test_position_unrelated_to_velocity_is_indeterminate_not_a_verdict():
    """Neither hypothesis holding is a third outcome, and it is a real one.

    If `velocity` turned out to be a filtered estimate that does not integrate to
    the reported `position` — which is one of the things suspected of it — both
    residuals are large and similar, and the probe must not pick the luckier one.
    """
    rows = [(i * DT, 0.0, 0.0, 0.4, 0.1, i * 0.04)
            for i in range(300)]
    rows = [(t, 0.3 * math.sin(t), 0.3 * math.cos(t), vx, vy, yaw)
            for t, _, _, vx, vy, yaw in rows]
    result = probe.analyse(rows)
    assert result["verdict"] == "indeterminate"


# ── the discriminator itself ─────────────────────────────────────────────────

def test_yaw_spread_is_circular():
    """359° and 1° are 2° apart, not 358°."""
    near_zero = [math.radians(a) for a in (359.0, 0.0, 1.0)]
    assert probe._yaw_spread(near_zero) < math.radians(10)


def test_yaw_spread_sees_a_real_turn():
    assert probe._yaw_spread([0.0, 0.5, 1.0]) > probe.MIN_YAW_SPREAD_RAD


def test_long_gaps_between_readings_are_dropped():
    """A gap is not a sample pair: position moved by an unknown amount."""
    rows = walk(frame="body", yaw_rate=0.4)
    with_gap = rows[:100] + [(rows[100][0] + 30.0, *rows[100][1:])] + rows[101:]
    assert probe.analyse(with_gap)["verdict"] == "body"
