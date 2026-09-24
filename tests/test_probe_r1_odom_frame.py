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
    assert "blocks" in result["why"]


# ── position jitter, which is what the second hardware run tripped on ─────────

def jitter(rows, sigma=0.004, seed=7):
    """Add zero-mean noise to the reported position, as a real odometry does.

    `sigma` is the order measured on r1_sz: adjacent readings arrive ~7 ms apart
    and disagree by millimetres, which is the same size as the 2 mm of genuine
    displacement between them.
    """
    state = seed
    out = []
    for t, x, y, vx, vy, yaw in rows:
        # A small deterministic PRNG: the test must not depend on the platform's
        # `random` implementation, and a fixed sequence keeps a failure reportable.
        state = (state * 1103515245 + 12345) % (2 ** 31)
        nx = ((state / 2 ** 31) - 0.5) * 2 * sigma
        state = (state * 1103515245 + 12345) % (2 ** 31)
        ny = ((state / 2 ** 31) - 0.5) * 2 * sigma
        out.append((t, x + nx, y + ny, vx, vy, yaw))
    return out


@pytest.mark.parametrize("frame", ["body", "world"])
def test_position_jitter_does_not_collapse_the_verdict(frame):
    """The second hardware regression.

    A 14 m walk with a full circle of turning — about as informative as a run can
    be — returned indeterminate at ratio 1.30, because the estimator differenced
    adjacent samples whose displacement was the same size as the jitter on them.
    Summing per-pair error over thousands of pairs accumulates noise as fast as
    signal. Integrating over one-second blocks first is the fix, and this is the
    test that fails without it.
    """
    rows = republish(walk(frame=frame, yaw_rate=0.4, seconds=40.0))
    result = probe.analyse(jitter(rows))
    assert result["verdict"] == frame
    assert result["ratio"] > 1.5


def test_jitter_leaves_the_residual_well_under_the_path():
    rows = jitter(republish(walk(frame="body", yaw_rate=0.4, seconds=40.0)))
    result = probe.analyse(rows)
    assert abs(result["scale"]["body"]["rotation_deg"]) < probe.MAX_FRAME_ROTATION_DEG


def test_the_scale_fit_recovers_a_magnitude_error():
    """What turns "neither hypothesis fits" from a dead end into a lead.

    On r1_sz the better hypothesis left 34.6 m of residual against 9.6 m walked.
    If a single scalar makes the integral line up, the frame is settled and what
    is left is a magnitude disagreement — a unit, a rate assumption, or a velocity
    that is not the derivative of the reported position.
    """
    rows = republish(walk(frame="body", yaw_rate=0.4, seconds=40.0))
    crippled = [(t, x, y, vx * 0.4, vy * 0.4, yaw) for t, x, y, vx, vy, yaw in rows]
    fit = probe.analyse(crippled)["scale"]["body"]
    assert fit["k"] == pytest.approx(1 / 0.4, rel=0.02)
    assert fit["residual_m"] < 0.05 * probe.analyse(crippled)["travel_m"]


def test_the_scale_fit_does_not_rescue_a_wrong_frame():
    """A scalar must not be able to turn a rotation error into a fit.

    Otherwise the diagnostic would report that everything is explainable by a
    magnitude, and the frame question would become unanswerable in principle.
    """
    rows = republish(walk(frame="world", yaw_rate=0.4, seconds=40.0))
    result = probe.analyse(rows)
    assert result["scale"]["body"]["residual_m"] > 0.3 * result["travel_m"]
    assert result["scale"]["world"]["k"] == pytest.approx(1.0, rel=0.02)


def test_a_magnitude_error_does_not_block_the_frame_verdict():
    """The whole point of separating scale from rotation.

    This used to assert the opposite — that a magnitude disagreement made the frame
    undecidable — and that is what the script did for three hardware runs, because
    it ranked hypotheses by residual against a `position` that was itself wrong by
    a factor of four. The frame question is about direction, so a scale error must
    not touch it: here the velocity is 40% of the truth and the frame is still
    named, with the magnitude reported separately as its own finding.
    """
    rows = republish(walk(frame="body", yaw_rate=0.4, seconds=40.0))
    crippled = [(t, x, y, vx * 0.4, vy * 0.4, yaw) for t, x, y, vx, vy, yaw in rows]
    result = probe.analyse(crippled)
    assert result["verdict"] == "body"
    assert result["scale"]["body"]["k"] == pytest.approx(2.5, rel=0.02)
    assert "disagree about distance" in result["magnitude_note"]


def test_a_correct_run_gets_no_magnitude_note():
    """The note has to mean something, so it must not fire on agreement."""
    result = probe.analyse(republish(walk(frame="body", yaw_rate=0.4, seconds=40.0)))
    assert "magnitude_note" not in result


def test_the_verdict_does_not_depend_on_the_block_length():
    """The property that replaced a workaround.

    An earlier version of this test asserted the opposite — that shrinking the
    block collapsed the verdict — and that was true of the estimator it was
    written against, which summed the *magnitude* of each block's residual. Such a
    sum accumulates noise as fast as signal, so it needed long blocks to survive.

    The complex-gain fit is a least-squares estimator over vectors, so the noise
    cancels in the fit instead of accumulating, and the block length stops
    mattering: verdict and scale come out the same at 1 s and at 0.05 s. Pinned
    here because "this threshold is load-bearing" and "this threshold is
    irrelevant" are both worth knowing, and the first claim was wrong.
    """
    rows = jitter(republish(walk(frame="body", yaw_rate=0.4, seconds=40.0)))
    original = probe.BLOCK_S
    try:
        long_blocks = probe.analyse(rows)
        probe.BLOCK_S = 0.05
        short_blocks = probe.analyse(rows)
    finally:
        probe.BLOCK_S = original
    assert long_blocks["verdict"] == short_blocks["verdict"] == "body"
    assert short_blocks["scale"]["body"]["k"] == pytest.approx(
        long_blocks["scale"]["body"]["k"], rel=0.02)


def test_a_reading_may_carry_extra_columns():
    """A reading is a record with a stable prefix, not a fixed-width tuple.

    `yaw_speed` was added as a seventh column and `analyse` still unpacked six,
    which crashed *after* a robot had been walked and the readings saved — the one
    moment when a crash costs somebody a repeat of the physical experiment. The
    saved file survived it, which is the only reason it cost nothing.
    """
    rows = [(*row, 0.1) for row in republish(walk(frame="body", yaw_rate=0.4))]
    assert probe.analyse(rows)["verdict"] == "body"
    assert probe.kinematics(rows)["wz_turned_deg"] is not None


def test_kinematics_reports_each_source_separately():
    """The readout that does not presuppose which source is right.

    On r1_sz a measured 3 m straight walk produced 3.62 m from `velocity` and
    0.81 m from `position`; nothing internal to the robot could have told those
    apart, and the tape measure did.
    """
    rows = republish(walk(frame="body", yaw_rate=0.0, speed=0.5, seconds=20.0))
    k = probe.kinematics(rows)
    assert k["velocity"]["net_m"] == pytest.approx(10.0, rel=0.05)
    assert k["position"]["net_m"] == pytest.approx(10.0, rel=0.05)
    assert abs(k["heading_turned_deg"]) < 1.0


# ── blocks ───────────────────────────────────────────────────────────────────

def test_blocks_break_on_a_gap_rather_than_integrating_across_it():
    """Position moved by an unknown amount while nobody was listening."""
    rows = walk(frame="body", yaw_rate=0.4, seconds=6.0)
    with_gap = rows[:30] + [(r[0] + 30.0, *r[1:]) for r in rows[30:]]
    for block in probe._blocks(with_gap, probe.BLOCK_S):
        assert max(b[0] - a[0] for a, b in zip(block, block[1:])) < 0.5


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


# ── republication, which is what the first hardware run actually tripped on ──

def republish(rows, times=4):
    """Send each reading `times` times, as R1 does — measured at ~3.7x on r1_sz.

    The timestamps still advance, so a repeat looks exactly like "the robot did
    not move during this interval while claiming to be moving".
    """
    out = []
    t = 0.0
    for row in rows:
        for _ in range(times):
            out.append((t, *row[1:]))
            t += DT / times
    return out


@pytest.mark.parametrize("frame", ["body", "world"])
def test_a_republished_run_is_still_decided(frame):
    """The regression. On hardware this returned indeterminate from a real walk.

    Every duplicated pair contributes pure residual to both hypotheses, and at
    73% duplication that swamped the difference between them — ratio 1.22 against
    a threshold of 1.5, from a robot that was genuinely walking and turning.
    """
    result = probe.analyse(republish(walk(frame=frame, yaw_rate=0.4)))
    assert result["verdict"] == frame
    assert result["repeats"] > 0


def test_residuals_larger_than_the_path_are_the_tell():
    """What made the hardware run diagnosable, kept as an assertion.

    A correct analysis of a consistent run cannot produce more error than there
    was movement. If this ever holds again, something is fabricating residual.
    """
    result = probe.analyse(republish(walk(frame="body", yaw_rate=0.4)))
    assert result["body_residual_m"] < result["travel_m"]


def test_a_stationary_robot_keeps_its_velocity_noise():
    """Dedupe on the whole reading, not on position.

    A robot standing still republishes the same position with fresh velocity
    noise every time, and those readings are real evidence. Collapsing them by
    position alone would throw away the only samples such a run has.
    """
    rows = [(i * DT, 1.0, 2.0, 0.001 * (-1) ** i, 0.0, 0.5) for i in range(200)]
    kept, dropped = probe._dedupe(rows)
    assert dropped == 0
    assert len(kept) == 200


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
