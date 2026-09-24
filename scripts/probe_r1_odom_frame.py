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
driver container on the robot:

    ssh unitree@10.100.130.6
    docker exec -it embodied-unitree-r1 python3 /work/scripts/probe_r1_odom_frame.py --seconds 40

Then walk the robot around — **including at least one substantial turn** — for
those forty seconds.
"""

from __future__ import annotations

import argparse
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
MIN_PAIRS = 50


def _collect(seconds: float, interface: str) -> list:
    """Every reading arriving in the window, as `(t, x, y, vx, vy, yaw)`."""
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

    print(f"listening on rt/odommodestate for {seconds:.0f}s — "
          "move the robot now, and turn it at least once")
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        time.sleep(0.5)
        print(f"  {len(rows)} readings, {deadline - time.monotonic():.0f}s left",
              end="\r", flush=True)
    print()
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
    yaws = []
    pairs = 0

    for (t0, x0, y0, vx0, vy0, yaw0), (t1, x1, y1, vx1, vy1, yaw1) in zip(rows, rows[1:]):
        dt = t1 - t0
        if dt <= 0 or dt > 0.5:
            continue
        dpx, dpy = x1 - x0, y1 - y0
        travel += math.hypot(dpx, dpy)
        yaws.append(yaw0)
        pairs += 1

        # Midpoint velocity, which is the right one to compare against a finite
        # difference of position: a forward difference of position is centred
        # half a step behind the later sample.
        vx, vy = (vx0 + vx1) / 2.0, (vy0 + vy1) / 2.0
        yaw = math.atan2((math.sin(yaw0) + math.sin(yaw1)) / 2.0,
                         (math.cos(yaw0) + math.cos(yaw1)) / 2.0)

        world_err.append(math.hypot(dpx - vx * dt, dpy - vy * dt))
        # v_world = R(yaw) · v_body
        bx = vx * math.cos(yaw) - vy * math.sin(yaw)
        by = vx * math.sin(yaw) + vy * math.cos(yaw)
        body_err.append(math.hypot(dpx - bx * dt, dpy - by * dt))

    out = {"pairs": pairs, "travel_m": travel, "repeats": repeats,
           "world_residual_m": sum(world_err), "body_residual_m": sum(body_err)}

    if pairs < MIN_PAIRS:
        out["verdict"] = "indeterminate"
        out["why"] = f"only {pairs} usable sample pairs (need {MIN_PAIRS})"
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
    if ratio < 1.5:
        out["verdict"] = "indeterminate"
        out["why"] = (f"the two hypotheses fit equally well (ratio {ratio:.2f}) — "
                      "either the run still carries too little turning, or "
                      "`position` is not the integral of `velocity` at all, "
                      "which would mean neither hypothesis is right and the "
                      "declaration cannot be settled this way")
        return out

    out["verdict"] = "world" if world < body else "body"
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
    print(f"usable sample pairs   {result['pairs']}")
    print(f"republished readings  {result['repeats']} (dropped — a repeat is not a measurement)")
    print(f"distance travelled    {result['travel_m']:.2f} m")
    if "yaw_spread_rad" in result:
        print(f"heading variation     {math.degrees(result['yaw_spread_rad']):.0f}°")
    print(f"residual if world     {result['world_residual_m']:.3f} m")
    print(f"residual if body      {result['body_residual_m']:.3f} m")
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
    args = ap.parse_args()

    rows = _collect(args.seconds, args.interface)
    if not rows:
        print("no readings at all — the robot is not publishing rt/odommodestate, "
              "or DDS came up on the wrong interface")
        return 1
    return report(analyse(rows))


if __name__ == "__main__":
    raise SystemExit(main())
