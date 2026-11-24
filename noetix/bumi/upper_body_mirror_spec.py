"""Joint-command construction for the `upper_body_mirror` card.

This module turns one frame of retargeted arm angles into the 21 motor
commands `LowController.set_joint` demands. It is deliberately dependency
free — no `rclpy`, no Noetix SDK, no `numpy` — so every rule that would
otherwise fail silently on a real robot (a sign flip, a joint pushed past its
mechanical stop, a unit mix-up between degrees and radians) is pinned by
tests that run on a laptop.

Why 21 commands and not 6
-------------------------
`lowcontrol_pybind.cpp` throws unless the vector is exactly 21 long:

    if (cmds.size() != 21) throw std::runtime_error("motorcmd size must be 21");

So "move only the arms" does not exist at this interface. Every tick sends all
21: fifteen joints hold the position they are measured at, six arm joints
follow the operator. That also means the card takes over the motors the
vendor's balance controller was driving, which is why it is off by default and
documented as a whole-body takeover rather than an arm feature.

Units
-----
`MotorCmd.pos` is radians. The evidence is `examples_py/test_policy.py`, whose
standing pose is `-0.1495` for a hip joint — that is -8.6 degrees, only
coherent as radians. This module speaks degrees throughout, because the joint
limits and the `upper_body_track` payload are both in degrees, and converts at
the last step inside `build_joint_commands`.
"""

from __future__ import annotations

import math

JOINT_COUNT = 21

#: Index of each observable arm axis in the 21-motor vector. The order matches
#: `device.py`'s `_BUMI_JOINT_NAMES`: 0-3 left arm, 4-9 left leg, 10-13 right
#: arm, 14-19 right leg, 20 waist.
ARM_JOINT_INDEX = {
    "left": {"pitch": 0, "roll": 1, "elbow": 3},
    "right": {"pitch": 10, "roll": 11, "elbow": 13},
}

#: The six indices the card actually writes.
ARM_INDEXES = (0, 1, 3, 10, 11, 13)

#: Shoulder yaw — humeral rotation. A monocular camera cannot see which way the
#: elbow crease faces, so `upper_body_track` reports it as zero rather than
#: inventing a value. Writing that zero here would rotate the arm to a pose
#: nobody observed, so these two joints always hold their measured position.
UNOBSERVABLE_INDEXES = (2, 12)

#: Mechanical hard limits in degrees, from the 21-joint table in the Bumi
#: professional guide §2.2. Identical to `upper_body_track_spec.ARM_LIMITS_DEG`
#: on purpose: the two cards must not disagree about where a joint stops. The
#: roll ranges are asymmetric because the arms are mirror-mounted.
HARD_LIMIT_DEG = {
    "left": {"pitch": (-135.0, 135.0), "roll": (-8.0, 111.0), "elbow": (-129.0, 0.0)},
    "right": {"pitch": (-135.0, 135.0), "roll": (-111.0, 8.0), "elbow": (-129.0, 0.0)},
}

#: How far inside the hard stop the card keeps its targets. The stop is where
#: the joint physically bottoms out; commanding exactly to it means every
#: overshoot is absorbed by the structure instead of the controller. Ten
#: degrees costs almost nothing in range and leaves the elbow slightly bent at
#: the "zero" pose, which is also the pose least likely to hit a singularity.
LIMIT_MARGIN_DEG = 10.0

#: The pose the arms drift back to when the operator leaves the frame. Zero on
#: every observable axis, then clamped — an arm hanging down, elbow barely
#: bent.
ZERO_TARGETS_DEG = {
    "l_pitch": 0.0,
    "l_roll": 0.0,
    "l_elbow": 0.0,
    "r_pitch": 0.0,
    "r_roll": 0.0,
    "r_elbow": 0.0,
}

#: `upper_body_track` uses this sentinel for "never measured yet", which is not
#: the same as zero and must never be commanded.
UNMEASURED_DEG = -999.0


def arm_key(side: str, axis: str) -> str:
    """`("left", "pitch")` → `"l_pitch"`, matching the payload field prefix."""
    return f"{side[0]}_{axis}"


def _effective_range(side: str, axis: str, margin: float) -> tuple[float, float]:
    """Hard limit pulled in by `margin`, kept non-empty.

    On the elbow the upper stop is 0 degrees (straight). Pulling in by 10
    leaves -10, so the arm never fully straightens — the intended trade. If a
    caller passes a margin wide enough to invert a range, the range collapses
    to its midpoint rather than silently swapping bounds.
    """
    low, high = HARD_LIMIT_DEG[side][axis]
    low += margin
    high -= margin
    if low > high:
        middle = (low + high) / 2.0
        return middle, middle
    return low, high


def clamp_arm(side: str, axis: str, deg: float,
              margin: float = LIMIT_MARGIN_DEG) -> float:
    """Clamp one arm angle into its effective range."""
    low, high = _effective_range(side, axis, margin)
    value = float(deg)
    if value < low:
        return low
    if value > high:
        return high
    return value


def deg_to_rad(deg: float) -> float:
    return float(deg) * math.pi / 180.0


def rad_to_deg(rad: float) -> float:
    return float(rad) * 180.0 / math.pi


def measured_arm_deg(measured_rad) -> dict:
    """Current arm pose in degrees, read back from the 21-motor state.

    This is what the card starts from on its first tick, so enabling the card
    never produces a jump: the first command is the pose the arm is already in.
    """
    if len(measured_rad) != JOINT_COUNT:
        raise ValueError(f"expected {JOINT_COUNT} measured positions, got {len(measured_rad)}")
    out = {}
    for side, axes in ARM_JOINT_INDEX.items():
        for axis, index in axes.items():
            out[arm_key(side, axis)] = rad_to_deg(measured_rad[index])
    return out


def build_joint_commands(measured_rad, targets_deg, *, keep_kp, keep_kd,
                         arm_kp, arm_kd,
                         margin: float = LIMIT_MARGIN_DEG) -> list:
    """All 21 motor commands for one tick, as plain dicts.

    Fifteen joints are commanded to the position they are measured at, with the
    holding gains: the card is not controlling balance, it is declining to
    disturb it. The six arm joints get the retargeted targets with the tracking
    gains. `vel` and `tau` are zero everywhere — this is a position target, not
    a velocity or torque command.

    Dicts rather than SDK objects so this can be tested without the robot; the
    card maps each dict onto a `MotorCmd` in one comprehension.
    """
    if len(measured_rad) != JOINT_COUNT:
        raise ValueError(f"expected {JOINT_COUNT} measured positions, got {len(measured_rad)}")

    # A non-finite feedback value is a dead encoder or a torn DDS sample, not a
    # pose. Copying it into the hold commands would put NaN into every joint, so
    # the whole tick is refused and the card stops instead.
    for value in measured_rad:
        if not math.isfinite(float(value)):
            raise ValueError("measured joint positions must be finite")

    commands = [
        {
            "pos": float(measured_rad[i]),
            "vel": 0.0,
            "tau": 0.0,
            "kp": float(keep_kp),
            "kd": float(keep_kd),
            "motor_id": i,
        }
        for i in range(JOINT_COUNT)
    ]

    for side, axes in ARM_JOINT_INDEX.items():
        for axis, index in axes.items():
            key = arm_key(side, axis)
            target = targets_deg.get(key)
            if target is None or float(target) == UNMEASURED_DEG:
                # No observation for this axis: hold what the motor reports.
                continue
            if not math.isfinite(float(target)):
                # NaN compares False against every clamp bound, so it would pass
                # straight through to `MotorCmd.pos`. Hold instead.
                continue
            commands[index]["pos"] = deg_to_rad(clamp_arm(side, axis, target, margin))
            commands[index]["kp"] = float(arm_kp)
            commands[index]["kd"] = float(arm_kd)

    return commands


def step_toward(current_deg: float, target_deg: float, max_step_deg: float) -> float:
    """One rate-limited step from `current` toward `target`.

    The rate limit is the only thing standing between a wrong sign and a slap,
    so it is applied to the *commanded* pose rather than trusting the upstream
    filter to have smoothed enough.
    """
    delta = float(target_deg) - float(current_deg)
    if delta > max_step_deg:
        return float(current_deg) + max_step_deg
    if delta < -max_step_deg:
        return float(current_deg) - max_step_deg
    return float(target_deg)


def rate_limited_targets(current_deg: dict, desired_deg: dict,
                         max_step_deg: float) -> dict:
    """Apply `step_toward` to all six arm axes at once."""
    return {
        key: step_toward(current_deg.get(key, 0.0), desired_deg.get(key, 0.0),
                         max_step_deg)
        for key in ZERO_TARGETS_DEG
    }


def targets_from_payload(payload: dict) -> dict:
    """Extract the six arm angles from a `upper_body_track` angles payload.

    Returns the sentinel for any axis the tracker has never measured, so
    `build_joint_commands` holds that joint instead of driving it to zero.
    Missing keys are treated the same way — an upstream that stopped reporting
    an axis must not be read as "this axis is at zero".
    """
    if not isinstance(payload, dict):
        raise ValueError("angles payload must be an object")

    targets = {}
    for side, axes in ARM_JOINT_INDEX.items():
        for axis in axes:
            field = f"{arm_key(side, axis)}_deg"
            try:
                number = float(payload.get(field))
            except (TypeError, ValueError):
                targets[arm_key(side, axis)] = UNMEASURED_DEG
                continue
            # NaN and infinity compare False against every bound, so a clamp
            # passes them straight through. They are not angles; treat them the
            # same as "never measured" and hold the joint.
            targets[arm_key(side, axis)] = (
                number if math.isfinite(number) else UNMEASURED_DEG)
    return targets


def max_step_for(rate_limit_deg_s: float, control_hz: float) -> float:
    """Degrees to move per tick at a given control rate.

    Raising the control rate lowers the per-tick step, which is what keeps the
    motion smooth: a 10 Hz output loop moving 3 degrees a tick looks like a
    sequence of jerks, the same rate limit spread over 50 Hz does not.
    """
    if rate_limit_deg_s <= 0:
        raise ValueError("rate_limit_deg_s must be positive")
    if control_hz <= 0:
        raise ValueError("control_hz must be positive")
    return float(rate_limit_deg_s) / float(control_hz)
