#!/usr/bin/env python3
"""Which frame is R1's `rt/odommodestate.velocity` in? — read-only.

**This script sends nothing.** It subscribes to one DDS topic and prints
arithmetic. Somebody else has to move the robot while it runs; walking it by
remote or from the dashboard is enough, and so is being pushed.

── the question ─────────────────────────────────────────────────────────────

`unitree/r1/device.py` publishes `motus.odom/1` with `frame: "body"`, taken from
`rt/odommodestate`. But that topic's `position` is odometry-frame (world), and a
velocity sitting in the same message as a world-frame position is at least as
likely to be world-frame too. Rule 3 of the format exists for exactly this:

    Body versus world is the most dangerous ambiguity here [...] Guessing wrong
    gives plausible numbers with the lateral sign flipped whenever the robot is
    not facing along world x.

Nothing detects the mistake. `common/odom.py` takes the declared frame as given,
and so does the consumer (`actucore/plugins/navi/odom.py` only checks that the
sample *says* body). So the robot has to be asked.

── the method ───────────────────────────────────────────────────────────────

One message carries `position` (world), `velocity` (unknown frame) and
`imu.rpy[2]` (world yaw). Between two messages, `position` moves by `dp`. Then:

    velocity is world  ⟹  dp ≈ v · dt
    velocity is body   ⟹  dp ≈ R(yaw) · v · dt

Integrate both residuals over the run and compare. This uses only quantities the
robot already reports, so it needs no ruler, no calibration and no commands.

**Yaw has to vary, or the test cannot answer.** At yaw ≈ 0 the rotation is the
identity and the two hypotheses are the same arithmetic — a robot walking
straight ahead from its start pose satisfies both. So the run is rejected as
indeterminate unless the robot both travelled and turned, and the thresholds for
that are printed rather than hidden. An honest "cannot tell" is the point: the
alternative is a confident answer from a run that carried no information.

── running it ───────────────────────────────────────────────────────────────

Needs `unitree_sdk2py` and the robot's DDS, so the simplest place is inside the
driver container on the robot (the image flattens the bundle, so the script lands
at `/work/`, not `/work/scripts/`):

    ssh unitree@10.100.128.238          # r1_sz; r1_bj is unitree@10.100.130.6
    docker exec -it embodied-unitree-r1 python3 -u /work/probe_r1_odom_frame.py \
        --until-decisive --seconds 600

`--until-decisive` is the one to use when a person has to walk the robot: it waits
for the motion instead of for a clock, printing how much travel and heading change
it has so far, and stops as soon as the data can answer. Five runs on r1_sz were
taken with a fixed window and three of them caught a stationary robot.

Walk it **3 m or more with at least one substantial turn** — a straight line
cannot answer the question, for the reason given above. Every run is saved, so
re-analysis (`--load`) never costs another walk.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import statistics
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))


# Below these the run carries no usable information and the script says so
# rather than reporting whichever residual happened to come out smaller.
MIN_TRAVEL_M = 0.30
MIN_YAW_SPREAD_RAD = 0.35        # about 20 degrees of heading variation
# Enough motion that an undecided verdict is a statement about the robot rather
# than about the run. Used only by `--until-decisive`, to know when to stop
# waiting for a better walk than the one it already has.
AMPLE_TRAVEL_M = 3.0
# Blocks, not sample pairs — see `analyse`. One second is long enough that the
# robot's displacement dwarfs the jitter on its reported position, and short
# enough that a hundred of them fit in a walk somebody is willing to perform.
BLOCK_S = 1.0
MIN_BLOCKS = 10
# The winning hypothesis has to actually explain the path, not merely explain it
# less badly than the other one. Without this, two wrong models still produce a
# verdict as soon as one of them is 1.5x less wrong.
MAX_RESIDUAL_FRACTION = 0.35


def _collect(seconds: float, interface: str, until_decisive: bool = False) -> list:
    """Readings as `(t, x, y, vx, vy, yaw)`, until the window ends or data suffices."""
    from common.dds_link import candidate_interfaces
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
    from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_

    last_error = None
    for name in candidate_interfaces(interface):
        try:
            ChannelFactoryInitialize(0, name) if name else ChannelFactoryInitialize(0)
            print(f"DDS up on interface {name or '(auto)'}")
            break
        except Exception as exc:                                # noqa: BLE001
            last_error = exc
    else:
        raise SystemExit(f"could not initialise DDS: {last_error}")

    rows: list = []

    def on_msg(msg) -> None:
        try:
            imu = msg.imu_state
            rows.append((time.monotonic(),
                         float(msg.position[0]), float(msg.position[1]),
                         float(msg.velocity[0]), float(msg.velocity[1]),
                         float(imu.rpy[2])))
        except Exception:                                       # noqa: BLE001
            pass

    sub = ChannelSubscriber("rt/odommodestate", SportModeState_)
    sub.Init(on_msg, 10)

    deadline = time.monotonic() + seconds
    if not until_decisive:
        print(f"listening on rt/odommodestate for {seconds:.0f}s — "
              "move the robot now, and turn it at least once")
        while time.monotonic() < deadline:
            time.sleep(0.5)
            print(f"  {len(rows)} readings, {deadline - time.monotonic():.0f}s left",
                  end="\r", flush=True)
        print()
        return rows

    # **Wait for the robot to move rather than for a clock to run out.**
    #
    # A fixed window makes the measurement depend on somebody starting to walk
    # inside it, and that turned out to be the hardest part of taking it: of five
    # runs on r1_sz, three caught a stationary robot, each costing a round trip
    # and asking for another walk. Nothing about the physics needs a deadline —
    # the run is over when it carries enough travel and enough heading change to
    # decide, which is a property of the data and is already computed.
    #
    # So the deadline becomes a backstop and the gates become the exit condition.
    # Whoever is walking the robot can start whenever they like.
    print(f"listening on rt/odommodestate — walk the robot whenever you are "
          f"ready (giving up after {seconds:.0f}s if nothing happens)")
    while time.monotonic() < deadline:
        time.sleep(2.0)
        if len(rows) < 500:
            continue
        progress = analyse(list(rows))
        travel = progress["travel_m"]
        spread = math.degrees(progress.get("yaw_spread_rad") or 0.0)
        print(f"  {len(rows)} readings | travelled {travel:.2f}/{MIN_TRAVEL_M} m "
              f"| heading {spread:.0f}/{math.degrees(MIN_YAW_SPREAD_RAD):.0f}° "
              f"| {deadline - time.monotonic():.0f}s before giving up", flush=True)
        if progress["verdict"] != "indeterminate":
            print("  enough data to decide — stopping")
            break
        # Still indeterminate, but with plenty of motion in hand: the answer is
        # genuinely "neither frame explains this" or "too close to call", and more
        # walking will not change it. Stop rather than burn the backstop — the
        # verdict text says which of the two it is.
        if travel >= AMPLE_TRAVEL_M and spread >= math.degrees(MIN_YAW_SPREAD_RAD):
            print("  the run carries ample motion and still cannot decide — stopping")
            break
    return rows


def analyse(rows: list) -> dict:
    """Residuals for both hypotheses, plus whether the run can decide at all.

    Split out from collection so it is testable against synthetic rows — see
    tests/test_probe_r1_odom_frame.py, which generates a run in each frame and
    checks this picks the right one.
    """
    rows, repeats = _dedupe(rows)
    travel = 0.0
    world_err = []
    body_err = []
    yaws = [row[5] for row in rows]

    # **Integrate over blocks, do not difference adjacent samples.**
    #
    # The obvious estimator compares each consecutive pair: `dp` against `v·dt`.
    # It is wrong here, and the second hardware run is what showed it. R1's state
    # updates every ~7 ms, so an adjacent pair moves about 2 mm — the same order
    # as the jitter on the reported position. Summing the magnitude of the error
    # over nine thousand such pairs accumulates that noise linearly, and the
    # frame difference accumulates alongside it, so the ratio between the two
    # hypotheses collapses towards 1: measured 1.30 on a walk of 14.34 m with a
    # full circle of turning, which is about as informative a run as one can ask
    # for. Residuals of 23 m and 30 m against a 14 m path were the symptom.
    #
    # Over a one-second block the robot moves ~0.3 m while the jitter stays at
    # millimetres, so the same comparison carries two orders of magnitude more
    # signal. Nothing else changes: within a block the velocity is still
    # integrated sample by sample, with the heading applied per sample, because
    # the heading turns during the block and that is the whole discriminator.
    blocks = 0
    integrals = []
    for block in _blocks(rows, BLOCK_S):
        (t0, x0, y0, *_), (t1, x1, y1, *_) = block[0], block[-1]
        dpx, dpy = x1 - x0, y1 - y0
        travel += math.hypot(dpx, dpy)
        blocks += 1

        wx = wy = bx = by = 0.0
        for (ta, xa, ya, vxa, vya, yawa), (tb, xb, yb, vxb, vyb, yawb) in zip(block, block[1:]):
            dt = tb - ta
            # Midpoint of the two samples bracketing the interval: the trapezoid
            # rule, which is what a reported position is the integral of.
            vx, vy = (vxa + vxb) / 2.0, (vya + vyb) / 2.0
            wx += vx * dt
            wy += vy * dt
            yaw = math.atan2((math.sin(yawa) + math.sin(yawb)) / 2.0,
                             (math.cos(yawa) + math.cos(yawb)) / 2.0)
            # v_world = R(yaw) · v_body
            bx += (vx * math.cos(yaw) - vy * math.sin(yaw)) * dt
            by += (vx * math.sin(yaw) + vy * math.cos(yaw)) * dt

        world_err.append(math.hypot(dpx - wx, dpy - wy))
        body_err.append(math.hypot(dpx - bx, dpy - by))
        integrals.append(((dpx, dpy), (wx, wy), (bx, by)))

    out = {"blocks": blocks, "travel_m": travel, "repeats": repeats,
           "world_residual_m": sum(world_err), "body_residual_m": sum(body_err),
           "scale": _fit_scales(integrals)}

    if blocks < MIN_BLOCKS:
        out["verdict"] = "indeterminate"
        out["why"] = (f"only {blocks} usable {BLOCK_S:.1f}s blocks "
                      f"(need {MIN_BLOCKS}) — run it for longer")
        return out
    if travel < MIN_TRAVEL_M:
        out["verdict"] = "indeterminate"
        out["why"] = (f"the robot moved {travel:.2f} m (need {MIN_TRAVEL_M}) — "
                      "a stationary robot satisfies both hypotheses")
        return out

    # Spread of heading, on the circle. Without it the rotation is a constant and
    # the two hypotheses differ by that constant rather than by their shape, so a
    # straight walk from the start pose "confirms" whichever the yaw happens to
    # favour.
    spread = _yaw_spread(yaws)
    out["yaw_spread_rad"] = spread
    if spread < MIN_YAW_SPREAD_RAD:
        out["verdict"] = "indeterminate"
        out["why"] = (f"heading only varied by {math.degrees(spread):.0f}° "
                      f"(need {math.degrees(MIN_YAW_SPREAD_RAD):.0f}°) — "
                      "at constant heading the two frames are the same arithmetic. "
                      "Walk it along two clearly different directions.")
        return out

    world, body = out["world_residual_m"], out["body_residual_m"]
    ratio = (max(world, body) / max(min(world, body), 1e-9))
    out["ratio"] = ratio
    out["residual_fraction"] = min(world, body) / max(travel, 1e-9)

    # Does the better hypothesis actually explain the path? Asked before the
    # ratio, because "one model is 1.5x less wrong than the other" is a verdict
    # only if at least one of them is right, and the interesting failure here is
    # both being wrong — which is what it looks like if `velocity` is a filtered
    # estimate that does not integrate to the reported `position`.
    if out["residual_fraction"] > MAX_RESIDUAL_FRACTION:
        out["verdict"] = "indeterminate"
        out["why"] = (
            f"neither hypothesis explains the path: the better of the two leaves "
            f"{min(world, body):.1f} m of residual against {travel:.1f} m "
            f"travelled ({out['residual_fraction'] * 100:.0f}%, "
            f"need under {MAX_RESIDUAL_FRACTION * 100:.0f}%). `position` is not "
            "the integral of `velocity` in either frame, so the declaration "
            "cannot be settled this way — and that is itself a finding about "
            "what `velocity` is.")
        return out

    if ratio < 1.5:
        out["verdict"] = "indeterminate"
        out["why"] = (f"the two hypotheses fit equally well (ratio {ratio:.2f}) — "
                      "the run carries too little turning to tell them apart. "
                      "Walk it along two clearly different directions.")
        return out

    out["verdict"] = "world" if world < body else "body"
    return out


def _fit_scales(integrals: list) -> dict:
    """Best single scale factor relating each integral to the measured path.

    Exists because "neither hypothesis explains the path" is a dead end as stated
    and a lead as measured. If one scalar `k` makes an integral line up with the
    reported displacement, the frame question is answered and the leftover is a
    *magnitude* disagreement — a unit, a sample-rate assumption, or a velocity
    that is not the derivative of the reported position. If no `k` helps, the
    disagreement is in direction, which is a different investigation.

    Least squares, so `k = Σ(dp·I) / Σ(I·I)`, with the residual reported at that
    `k`. Reported for both hypotheses because which frame wins can change once
    the magnitudes are commensurate — a 4x error swamps a rotation.
    """
    out = {}
    for index, name in ((1, "world"), (2, "body")):
        num = den = 0.0
        for dp, *hypotheses in integrals:
            ix, iy = hypotheses[index - 1]
            num += dp[0] * ix + dp[1] * iy
            den += ix * ix + iy * iy
        if den <= 1e-12:
            out[name] = {"k": None, "residual_m": None}
            continue
        k = num / den
        residual = 0.0
        for dp, *hypotheses in integrals:
            ix, iy = hypotheses[index - 1]
            residual += math.hypot(dp[0] - k * ix, dp[1] - k * iy)
        out[name] = {"k": k, "residual_m": residual}
    return out


def _blocks(rows: list, seconds: float):
    """Split into contiguous runs of roughly `seconds`, breaking on any gap.

    A gap is a block boundary rather than something to integrate across: the
    position moved by an unknown amount while we were not listening, and folding
    that into a block would charge the difference to both hypotheses.
    """
    out = []
    current = []
    for row in rows:
        if current:
            if row[0] - current[-1][0] > 0.5:          # a gap, not an interval
                if row[0] - current[0][0] >= seconds * 0.5 and len(current) > 1:
                    out.append(current)
                current = []
            elif row[0] - current[0][0] >= seconds:
                current.append(row)
                out.append(current)
                current = [row]
                continue
        current.append(row)
    if len(current) > 1 and current[-1][0] - current[0][0] >= seconds * 0.5:
        out.append(current)
    return out


def _dedupe(rows: list):
    """Drop republished readings, keeping the first of each run. Returns `(rows, dropped)`.

    **R1 sends each reading about 3.7 times.** Measured on r1_sz: 4938 messages in
    ten seconds, of which 3602 consecutive pairs carried a bit-identical position —
    the DDS topic runs at ~490 Hz while the state behind it updates at roughly
    130. A repeat is not a measurement, and differencing across one asks the
    arithmetic to explain how the robot moved 0 m in 2 ms while reporting
    0.23 m/s. Every such pair contributes pure residual to *both* hypotheses.

    That is what made the first hardware run indeterminate. The robot was really
    walking — 1.69 m of path, yaw sweeping through the ±π wrap — and the verdict
    was still "neither hypothesis fits", with residuals of 4.15 m and 3.40 m
    against a path of 1.69 m. Residuals larger than the path were the tell: the
    numbers were dominated by 73% of the pairs being repeats rather than by
    anything about a frame.

    Deduplicating on the whole reading rather than on position alone, because a
    robot that is genuinely stationary reports the same position with fresh
    velocity noise, and those pairs are real evidence — thin, but not fabricated.
    """
    out = []
    dropped = 0
    for row in rows:
        if out and row[1:] == out[-1][1:]:
            dropped += 1
            continue
        out.append(row)
    return out, dropped


def _yaw_spread(yaws: list) -> float:
    """How much the heading varied, as an angle. Circular, so 359°→1° is 2°."""
    if len(yaws) < 2:
        return 0.0
    mean = math.atan2(statistics.fmean(math.sin(y) for y in yaws),
                      statistics.fmean(math.cos(y) for y in yaws))
    return max(abs(math.atan2(math.sin(y - mean), math.cos(y - mean))) for y in yaws) * 2.0


def report(result: dict) -> int:
    print()
    print(f"integration blocks    {result['blocks']} x {BLOCK_S:.1f}s")
    print(f"republished samples   {result['repeats']} dropped (a repeat is not a measurement)")
    print(f"distance travelled    {result['travel_m']:.2f} m")
    if "yaw_spread_rad" in result:
        print(f"heading variation     {math.degrees(result['yaw_spread_rad']):.0f}°")
    print(f"residual if world     {result['world_residual_m']:.3f} m")
    print(f"residual if body      {result['body_residual_m']:.3f} m")
    # Only once there was motion to explain. A least-squares scale against a path
    # of two centimetres fits noise to noise, and prints a confident `k=-629.561`
    # that means nothing — worse than silence, because it looks like a measurement.
    if result["travel_m"] >= MIN_TRAVEL_M:
        for name, fit in (result.get("scale") or {}).items():
            if fit.get("k") is None:
                continue
            print(f"  best-fit scale {name:5s} k={fit['k']:.3f} "
                  f"→ residual {fit['residual_m']:.3f} m"
                  + ("   ← a scalar explains it" if fit["residual_m"]
                     < MAX_RESIDUAL_FRACTION * result["travel_m"] else ""))
    print()

    verdict = result["verdict"]
    if verdict == "indeterminate":
        print(f"INDETERMINATE — {result['why']}")
        return 2
    print(f"VERDICT: velocity is in the {verdict.upper()} frame "
          f"({result['ratio']:.1f}x better fit)")
    print()
    if verdict == "body":
        print("The declaration in unitree/r1/device.py is right. Delete the")
        print("`frame_unverified` key from _LocoStateNode.health().")
    else:
        print("The declaration is WRONG. `motus.odom/1` says body frame and this")
        print("is world frame, so vx/vy swap at non-zero heading. Two fixes, and")
        print("the second is the one worth doing:")
        print("  - declare frame='world', which is honest and makes the consumer")
        print("    refuse it (OdomInterface.usable_for_control is False) — R1")
        print("    then has no stuck detection and says so;")
        print("  - rotate it into body frame in the driver with the yaw from the")
        print("    same message (imu.rpy[2]) and keep declaring body, which loses")
        print("    nothing and is a handful of lines.")
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--seconds", type=float, default=40.0)
    ap.add_argument("--interface", default="",
                    help="network interface for the robot's DDS; auto-detected if omitted")
    # **Every run is saved.** Collecting one costs a person walking a robot for a
    # minute, so the analysis must never be the reason to ask for another. Two of
    # the three runs on r1_sz were spent rediscovering a defect in the estimator
    # rather than measuring the robot, and each cost a walk that a saved file
    # would have made free.
    ap.add_argument("--save", default="/tmp/r1_odom_probe.jsonl",
                    help="where to write the raw readings (empty string to skip)")
    ap.add_argument("--load", default="",
                    help="re-analyse a saved run instead of collecting; needs no robot")
    ap.add_argument("--until-decisive", action="store_true",
                    help="stop once the data can decide, rather than after --seconds; "
                         "--seconds then acts as a give-up backstop. Use this when a "
                         "person has to walk the robot — they can start whenever")
    args = ap.parse_args()

    if args.load:
        with open(args.load) as handle:
            rows = [tuple(json.loads(line)) for line in handle if line.strip()]
        print(f"re-analysing {len(rows)} saved readings from {args.load}")
        return report(analyse(rows))

    rows = _collect(args.seconds, args.interface, args.until_decisive)
    if not rows:
        print("no readings at all — the robot is not publishing rt/odommodestate, "
              "or DDS came up on the wrong interface")
        return 1
    if args.save:
        with open(args.save, "w") as handle:
            for row in rows:
                handle.write(json.dumps(list(row)) + "\n")
        print(f"saved {len(rows)} readings to {args.save} — "
              f"re-analyse with --load {args.save}, no robot needed")
    return report(analyse(rows))


if __name__ == "__main__":
    raise SystemExit(main())
