"""What Bumi reports about its own motion — `motus.odom/1`, as pure functions.

Separate from `device.py` for the same reason `camera_specs.py` is: that module
imports `rclpy`, so anything inside it can only be exercised on a robot. What
lives here is the part where a mistake would be silent — which axes this robot
claims to measure, and what an unmeasured axis is reported as.

── the one rule this file is here to hold ───────────────────────────────────

**Bumi reports no translational velocity at all.** `HighController` exposes an
IMU, joint states and a battery; there is no body-frame speed and no odometry
pose anywhere in the SDK. So `vx`, `vy` and `vz` are `None` in every sample and
absent from `provides`.

Reporting `0.0` for them would be a different claim — that the robot measured
itself standing still — and it is the claim that makes a consumer's stuck
detector ("commanded 0.3 m/s, measured nothing, therefore we have hit
something") fire on every step of a robot that simply cannot answer. That is
rule 1 of `common/odom.py`, and this driver is the shape it was describing.

The consequence is worth stating plainly rather than leaving for somebody to
discover: a navigation policy on Bumi has to fall back to predicting its own
motion from the commands it sent, with the process noise widened accordingly.
`wz` — the axis such a policy leans on hardest, because self-rotation is the
main reason a target moves across the image — **is** measured, so the valuable
half of the format is available here.
"""
from __future__ import annotations

# Which of the six axes this robot genuinely measures. Absent rather than
# listed-and-null so a consumer can decide **at start** whether it can do its
# job, instead of discovering it one sample at a time at 10 Hz.
AXES = ("wx", "wy", "wz")

# How often the sample goes out, and how often the IMU behind it is read. Both
# are separate from `motion_state.poll_interval_s`, which governs the 21-joint
# payload on `/{ns}/motion/state`.
#
# **10 Hz, not the 2 Hz of the joint payload.** navi's `max_obs_age_ms` is
# 500 ms and a 2 Hz publisher lands exactly on it, so half the samples read as
# stale and the consumer alternates between using odometry and not — which
# presents as a tracker that intermittently forgets the robot is turning. The
# reading itself is cheap (one IMU struct, one mode int), so it gets its own
# loop rather than dragging the joint payload up with it.
PUBLISH_S = 0.1
SAMPLE_S = 0.02


def reading(angular_vel) -> list:
    """One IMU read as a twist row, in `motus.control/1` axis order.

    Three `None`s and three numbers, and the `None`s are the point — see the
    module docstring. Raises on an unreadable IMU rather than substituting
    zeros: the caller drops the reading, and a window with nothing in it
    publishes all-`None`, which is the honest answer.
    """
    return [None, None, None,
            float(angular_vel[0]), float(angular_vel[1]), float(angular_vel[2])]


def interface() -> dict:
    """The declaration a consumer reads once, at start."""
    from common.odom import build_interface

    return build_interface(
        provides=AXES,
        rate_hz=1.0 / PUBLISH_S,
        # No pose at all, so there is nothing to drift. Not "unbounded": that
        # would say a position is reported and cannot be trusted, and a consumer
        # is entitled to act differently on "none".
        pose_drift="none",
    )


def sample(readings, *, received_ms: int, workmode=None,
           workmode_name: str = "") -> dict:
    """One averaged `motus.odom/1` sample from a window of readings.

    Averaged rather than decimated: the consumer this format was shaped around
    compares a commanded rate against a measured one, so it is reading a
    threshold crossing, and a noisy sample crosses thresholds it should not.
    `mean_twist` carries the `None` rule through the average — an axis stays
    `None` when no reading in the window carried a number for it.

    The SDK's IMU struct has no timestamp of any kind, so there is no robot
    clock to quote. `resolve_stamp_ms` is still the right call rather than a
    bare `time.time()`: it records `stamp_source: "received"` in the sample,
    which is how a consumer tells "when it was measured" from "when it reached
    you" instead of assuming the first.
    """
    from common.odom import build_sample, mean_twist, resolve_stamp_ms

    stamp_ms, provenance = resolve_stamp_ms(vendor_ms=None,
                                            received_ms=received_ms)
    vendor = dict(provenance)
    # Distinguishes "nothing arrived in this window" from "the robot held
    # still" — both produce an all-`None` twist and they are not the same fact.
    vendor["samples"] = len(readings or ())
    if workmode is not None:
        vendor["workmode"] = workmode
        vendor["workmode_name"] = workmode_name
    return build_sample(
        stamp_ms=stamp_ms,
        twist=mean_twist(readings),
        # Body frame: these are the IMU's own axes and the IMU is bolted to the
        # torso. Unlike R1's this needs no measurement to settle — there is no
        # odometry frame anywhere in this SDK for it to have been confused with.
        frame="body",
        # No position of any kind is available, so there is nothing to report.
        pose=None,
        vendor=vendor,
    )


HEALTH_NOTE = (
    "Bumi 的 SDK 只给 IMU 和关节，没有机体速度也没有位置 —— vx/vy/vz 恒为 null，"
    "pose 恒为 null。依赖实测平移速度的功能（卡死检测之类）在这台机器人上用不了，"
    "这是机器人的限制，不是故障。")
