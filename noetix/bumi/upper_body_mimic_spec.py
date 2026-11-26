"""Joint-command construction for the `upper_body_mimic` card.

This module turns one frame of retargeted arm angles into the 21 motor
commands `LowController.set_joint` demands. It deliberately avoids ROS, the
Noetix SDK and numpy, so every rule that would otherwise fail silently on a
real robot (a sign flip, a joint pushed past its mechanical stop, a unit
mix-up between degrees and radians) is pinned by tests that run on a laptop.

Why 21 commands and not 8
-------------------------
`lowcontrol_pybind.cpp` throws unless the vector is exactly 21 long:

    if (cmds.size() != 21) throw std::runtime_error("motorcmd size must be 21");

So "use only the eight arm joints" is not an option at this interface. All 21
motors are commanded every tick: the two unobservable shoulder-yaw joints and
the thirteen leg/waist joints hold the position they are measured at, and six
arm joints follow the operator. This is a whole-body takeover in the sense
that no other controller may write those motors while this card runs, which
is why it is off by default and gated behind an explicit DDS sign-off.

Limits and dynamics
-------------------
Mechanical limits are read from the shipped URDF (`resource/bumi_model.urdf`),
converted from radians to degrees. The URDF declares velocity 0 (unknown), so
the motion bounds come from PR #485's validated Gazebo preview: 2 rad/s and
6 rad/s², applied per joint to the *commanded* pose.

Units
-----
`MotorCmd.pos` is radians. The evidence is `examples_py/test_policy.py`, whose
standing pose is `-0.1495` for a hip joint — that is -8.6 degrees, only
coherent as radians. This module speaks degrees throughout and converts at the
last step inside `build_joint_commands`.
"""

from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from pathlib import Path

JOINT_COUNT = 21

#: All eight arm joints. Order matches `device.py`'s `_BUMI_JOINT_NAMES`:
#: 0-3 left pitch/roll/yaw/elbow, 10-13 right pitch/roll/yaw/elbow.
ARM_JOINT_INDEX = {
    "left": {"pitch": 0, "roll": 1, "yaw": 2, "elbow": 3},
    "right": {"pitch": 10, "roll": 11, "yaw": 12, "elbow": 13},
}

#: Only three axes per side are observable from a monocular RGB frame. The
#: shoulder yaw (humeral rotation) is not, so those two joints are held at
#: their measured position rather than invented as zero.
OBSERVABLE_AXES = ("pitch", "roll", "elbow")
UNOBSERVABLE_INDEXES = (2, 12)

#: The six indices the card actually follows.
ARM_INDEXES = tuple(
    ARM_JOINT_INDEX[side][axis]
    for side in ("left", "right") for axis in OBSERVABLE_AXES
)

URDF_PATH = Path(__file__).resolve().parent / "resource" / "bumi_model.urdf"

_URDF_JOINT_NAME = {
    0: "l_arm_pitch_joint", 1: "l_arm_roll_joint",
    2: "l_arm_yaw_joint", 3: "l_elbow_pitch_joint",
    10: "r_arm_pitch_joint", 11: "r_arm_roll_joint",
    12: "r_arm_yaw_joint", 13: "r_elbow_pitch_joint",
}
_NAME_TO_INDEX = {name: index for index, name in _URDF_JOINT_NAME.items()}

# Fallback only for a checkout without the resource directory. The card
# refuses to start in that case rather than silently driving to guessed stops.
_FALLBACK_HARD_LIMIT_DEG = {
    "left": {"pitch": (-135.24, 135.24), "roll": (-8.02, 111.12),
             "yaw": (-90.0, 90.0), "elbow": (-129.47, 0.0)},
    "right": {"pitch": (-135.24, 135.24), "roll": (-111.12, 8.02),
              "yaw": (-90.0, 90.0), "elbow": (-129.47, 0.0)},
}


def _read_urdf_limits() -> dict:
    """Lower/upper stops in degrees, keyed by motor index, straight from URDF."""
    root = ET.parse(URDF_PATH).getroot()
    limits = {}
    for joint in root.iter("joint"):
        index = _NAME_TO_INDEX.get(joint.get("name"))
        if index is None:
            continue
        limit = joint.find("limit")
        if limit is None:
            raise ValueError(f"URDF joint {joint.get('name')} has no <limit>")
        limits[index] = (math.degrees(float(limit.get("lower"))),
                         math.degrees(float(limit.get("upper"))))
    if len(limits) != len(_URDF_JOINT_NAME):
        raise ValueError(
            f"URDF declares {len(limits)} of the {len(_URDF_JOINT_NAME)} arm joints")
    return limits


try:
    _LIMITS_BY_INDEX = _read_urdf_limits()
    URDF_LIMITS_LOADED = True
    URDF_LIMITS_ERROR = ""
except (OSError, ET.ParseError, ValueError, TypeError) as exc:
    _LIMITS_BY_INDEX = None
    URDF_LIMITS_LOADED = False
    URDF_LIMITS_ERROR = f"{type(exc).__name__}: {exc}"

HARD_LIMIT_DEG = {
    side: {
        axis: (_LIMITS_BY_INDEX if URDF_LIMITS_LOADED
               else _FALLBACK_HARD_LIMIT_DEG)[index]
        for axis, index in axes.items()
    }
    for side, axes in ARM_JOINT_INDEX.items()
}

#: Keep this far inside the hard stop. Ten degrees leaves the elbow slightly
#: bent at the "zero" pose, which is the pose least likely to hit a singularity.
LIMIT_MARGIN_DEG = 10.0

#: PR #485 preview bounds, converted from rad to deg for the whole module.
MAX_VELOCITY_RAD_S = 2.0
MAX_ACCELERATION_RAD_S2 = 6.0
MAX_VELOCITY_DEG_S = math.degrees(MAX_VELOCITY_RAD_S)
MAX_ACCELERATION_DEG_S2 = math.degrees(MAX_ACCELERATION_RAD_S2)

#: Safe pose: arms down, elbows slightly bent once the margin is applied.
ZERO_TARGETS_DEG = {
    f"{side[0]}_{axis}": 0.0
    for side in ("left", "right") for axis in OBSERVABLE_AXES
}

#: Upstream's "never measured" sentinel. It is retained only so a stale
#: payload can be recognised; it is never commanded.
UNMEASURED_DEG = -999.0


def arm_key(side: str, axis: str) -> str:
    """`("left", "pitch")` → `"l_pitch"`, matching the payload field prefix."""
    return f"{side[0]}_{axis}"


def safe_pose_deg() -> dict:
    """The pose the arms return to on lost or low-confidence tracking."""
    return dict(ZERO_TARGETS_DEG)


def clamp_targets(desired_deg: dict,
                  margin: float = LIMIT_MARGIN_DEG) -> dict:
    """Keep the trajectory target inside the URDF stops before limiting it.

    Clamping only at command time would let the velocity state chase an
    unreachable target and wind up at the mechanical stop.
    """
    out = {}
    for side in ("left", "right"):
        for axis in OBSERVABLE_AXES:
            key = arm_key(side, axis)
            value = desired_deg.get(key)
            out[key] = (ZERO_TARGETS_DEG[key] if value is None
                        else clamp_arm(side, axis, value, margin))
    return out


def _effective_range(side: str, axis: str, margin: float) -> tuple[float, float]:
    """Hard limit pulled in by `margin`, kept non-empty."""
    low, high = HARD_LIMIT_DEG[side][axis]
    low += margin
    high -= margin
    if low > high:
        middle = (low + high) / 2.0
        return middle, middle
    return low, high


def clamp_arm(side: str, axis: str, deg: float,
              margin: float = LIMIT_MARGIN_DEG) -> float:
    """Clamp one arm angle into its effective range.

    Raises on a non-finite angle rather than returning it: every comparison
    against NaN is False, so a silent `return value` would hand NaN back to the
    caller as if it had been clamped. `build_joint_commands` filters these too.
    """
    value = float(deg)
    if not math.isfinite(value):
        raise ValueError(f"angle must be finite, got {deg!r}")
    low, high = _effective_range(side, axis, margin)
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
    """Current observable arm pose in degrees, read from the 21-motor state."""
    if len(measured_rad) != JOINT_COUNT:
        raise ValueError(f"expected {JOINT_COUNT} measured positions, got {len(measured_rad)}")
    out = {}
    for side, axes in ARM_JOINT_INDEX.items():
        for axis in axes:
            if axis not in OBSERVABLE_AXES:
                continue
            value = float(measured_rad[axes[axis]])
            if not math.isfinite(value):
                raise ValueError("measured joint positions must be finite")
            out[arm_key(side, axis)] = rad_to_deg(value)
    return out


def build_joint_commands(measured_rad, targets_deg, *, keep_kp, keep_kd,
                         arm_kp, arm_kd) -> list:
    """All 21 motor commands for one tick, as plain dicts.

    Fifteen joints hold the position they are measured at with the holding
    gains; the six observable arm joints get the retargeted targets with the
    tracking gains. A missing or non-finite target moves that axis toward the
    safe pose instead of preserving the last human command.

    Arm targets are clamped to the **hard** URDF stops, not the trajectory
    margin. The margin belongs to `clamp_targets`, which every moving target
    passes through before the rate limiter; applying it again here would
    rewrite the measured pose the card seeds its first command from. A real
    elbow measured at 0° (straight) would then be emitted as -10° in the first
    tick — a 10° jump the rate limiter never produced.
    """
    if len(measured_rad) != JOINT_COUNT:
        raise ValueError(f"expected {JOINT_COUNT} measured positions, got {len(measured_rad)}")
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

    for side in ("left", "right"):
        for axis in OBSERVABLE_AXES:
            index = ARM_JOINT_INDEX[side][axis]
            key = arm_key(side, axis)
            target = targets_deg.get(key)
            if target is None or not math.isfinite(float(target)) \
                    or float(target) == UNMEASURED_DEG:
                target = ZERO_TARGETS_DEG[key]
            commands[index]["pos"] = deg_to_rad(
                clamp_arm(side, axis, target, margin=0.0))
            commands[index]["kp"] = float(arm_kp)
            commands[index]["kd"] = float(arm_kd)
    return commands


def step_with_accel(pos_deg: float, vel_deg_s: float, target_deg: float,
                    dt_s: float, max_vel_deg_s: float,
                    max_acc_deg_s2: float) -> tuple[float, float]:
    """One trapezoidal step with a real braking phase.

    The brake speed is the velocity from which a full-deceleration stop fits
    inside the remaining distance, minus one tick's worth of acceleration so the
    discrete steps do not cross the target on the last tick. Velocity is then
    approached under the acceleration bound and position advanced by the
    average velocity over the tick. Nothing here snaps velocity to zero on
    arrival: doing that is a single-tick deceleration of the whole speed and
    breaks the 6 rad/s² contract the function advertises.

    Returning both position and velocity matters: a rate-only limiter can
    reverse direction instantly at the top of a swing, which is exactly the
    jerk the bound is meant to remove.
    """
    if dt_s <= 0:
        raise ValueError("dt_s must be positive")
    if max_vel_deg_s <= 0 or max_acc_deg_s2 <= 0:
        raise ValueError("velocity and acceleration bounds must be positive")

    pos = float(pos_deg)
    velocity = float(vel_deg_s)
    remaining = float(target_deg) - pos
    distance = abs(remaining)
    direction = 1.0 if remaining >= 0 else -1.0
    accel_per_tick = max_acc_deg_s2 * dt_s

    # Converged: both position and speed are within one tick of rest. Land
    # exactly, and zeroing a speed this small is itself within the bound.
    converge_distance = 0.5 * max_acc_deg_s2 * dt_s * dt_s
    if distance <= converge_distance and abs(velocity) <= accel_per_tick:
        return float(target_deg), 0.0

    # v² = 2 a d is the continuous stop-from-here speed. Subtract one tick of
    # deceleration so the discrete integration has room to brake: without it
    # an axis at cruise speed crosses a near target on the final tick.
    brake_speed = max(0.0,
                      math.sqrt(2.0 * max_acc_deg_s2 * distance) - accel_per_tick)
    desired_velocity = direction * min(max_vel_deg_s, brake_speed)

    delta_v = max(-accel_per_tick,
                  min(accel_per_tick, desired_velocity - velocity))
    velocity = max(-max_vel_deg_s, min(max_vel_deg_s, velocity + delta_v))
    step = 0.5 * (float(vel_deg_s) + velocity) * dt_s

    if direction * step > distance:
        # No discrete braking curve can land on an arbitrary target in exactly
        # one tick. Prefer the velocity that reaches it if that still respects
        # the acceleration bound; otherwise stop at the target for this tick
        # without inventing a velocity the motor was never commanded to reach.
        landing_velocity = 2.0 * remaining / dt_s - float(vel_deg_s)
        if abs(landing_velocity - float(vel_deg_s)) <= accel_per_tick + 1e-9:
            velocity = max(-max_vel_deg_s, min(max_vel_deg_s, landing_velocity))
            step = remaining
        else:
            step = direction * distance
    return pos + step, velocity


def accel_limited_targets(current_deg: dict, velocity_deg_s: dict,
                          desired_deg: dict, dt_s: float,
                          max_vel_deg_s: float = MAX_VELOCITY_DEG_S,
                          max_acc_deg_s2: float = MAX_ACCELERATION_DEG_S2
                          ) -> tuple[dict, dict]:
    """Advance all six arm axes by one tick under both bounds."""
    positions, velocities = {}, {}
    for key in ZERO_TARGETS_DEG:
        positions[key], velocities[key] = step_with_accel(
            current_deg.get(key, 0.0), velocity_deg_s.get(key, 0.0),
            desired_deg.get(key, 0.0), dt_s, max_vel_deg_s, max_acc_deg_s2)
    return positions, velocities


def targets_from_payload(payload: dict) -> dict:
    """Extract the six arm angles from an `upper_body_mimic` angles payload.

    A missing, non-numeric or non-finite axis becomes the safe pose, not the
    last human command: an occluded wrist must not leave the arm frozen in a
    pose nobody can see any more.
    """
    if not isinstance(payload, dict):
        raise ValueError("angles payload must be an object")
    targets = {}
    for side in ("left", "right"):
        for axis in OBSERVABLE_AXES:
            key = arm_key(side, axis)
            try:
                number = float(payload.get(f"{key}_deg"))
            except (TypeError, ValueError):
                targets[key] = ZERO_TARGETS_DEG[key]
                continue
            targets[key] = number if math.isfinite(number) else ZERO_TARGETS_DEG[key]
    return targets
