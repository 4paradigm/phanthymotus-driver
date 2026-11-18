"""ROS-free validation rules for the Panther motion cards.

Everything that decides whether a command may be sent lives here or in `panther_motion.py`
and imports nothing from ROS, so the rules are tested on a laptop. Ported from the Panther
driver proposed in PR #520 (validation, limits and the two `SwitchController` field
generations); the lifecycle around it (stop, completion) is in `panther_motion.py`.
"""
from __future__ import annotations

import math
import xml.etree.ElementTree as ET

LEFT_ARM_JOINTS = tuple(f"joint_la{i}" for i in range(8))
RIGHT_ARM_JOINTS = tuple(f"joint_ra{i}" for i in range(8))
HEAD_JOINTS = ("joint_head0", "joint_head1")


def finite_number(value, field: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{field} must be a number")
    value = float(value)
    if not math.isfinite(value):
        raise ValueError(f"{field} must be finite")
    return value


def finite_vector(value, field: str, length: int) -> list:
    if not isinstance(value, (list, tuple)) or len(value) != length:
        raise ValueError(f"{field} must contain exactly {length} numbers")
    return [finite_number(item, f"{field}[{index}]") for index, item in enumerate(value)]


def require_confirmation(args: dict) -> None:
    if args.get("confirm_motion") is not True:
        raise ValueError("confirm_motion=true is required before sending a motion command")


def parse_joint_limits(urdf: str) -> dict:
    """`{joint: (lower, upper)}` for every joint of a URDF that declares both bounds."""
    if not isinstance(urdf, str) or not urdf.strip():
        return {}
    root = ET.fromstring(urdf)
    limits = {}
    for joint in root.findall("joint"):
        name, limit = joint.get("name"), joint.find("limit")
        if not name or limit is None:
            continue
        lower, upper = limit.get("lower"), limit.get("upper")
        if lower is None or upper is None:
            continue
        try:
            low, high = float(lower), float(upper)
        except ValueError:
            continue
        if math.isfinite(low) and math.isfinite(high) and low <= high:
            limits[name] = (low, high)
    return limits


def validate_joint_target(joint_names, positions, limits: dict, current: dict,
                          max_delta_rad: float) -> list:
    """A target for `joint_names`: finite, inside the URDF limits, and a bounded step from now."""
    target = finite_vector(positions, "positions", len(joint_names))
    missing = [name for name in joint_names if name not in limits]
    if missing:
        raise ValueError("no joint limits known for: " + ", ".join(missing))
    unseen = [name for name in joint_names if name not in current]
    if unseen:
        raise ValueError("fresh joint state is missing: " + ", ".join(unseen))
    step = finite_number(max_delta_rad, "max_delta_rad")
    if step <= 0:
        raise ValueError("max_delta_rad must be positive")
    for name, value in zip(joint_names, target):
        lower, upper = limits[name]
        if not lower <= value <= upper:
            raise ValueError(f"{name} target {value} is outside the URDF limit [{lower}, {upper}]")
        delta = abs(value - float(current[name]))
        if delta > step:
            raise ValueError(f"{name} target is {delta:.4f} rad from its current position, "
                             f"over the commissioning step limit of {step:.4f} rad")
    return target


def validate_drive(args: dict, config: dict) -> tuple:
    """`(vx, vy, wz, duration_s)`, or ValueError. Caps come from config, never from the caller."""
    require_confirmation(args)
    vx = finite_number(args.get("linear_x", 0.0), "linear_x")
    vy = finite_number(args.get("linear_y", 0.0), "linear_y")
    wz = finite_number(args.get("angular_z", 0.0), "angular_z")
    duration = finite_number(args.get("duration_s", 1.0), "duration_s")
    max_linear = finite_number(config["max_linear_mps"], "max_linear_mps")
    max_angular = finite_number(config["max_angular_radps"], "max_angular_radps")
    max_duration = finite_number(config["max_duration_s"], "max_duration_s")
    if abs(vx) > max_linear or abs(vy) > max_linear:
        raise ValueError(f"linear speed is over the commissioning cap of {max_linear} m/s")
    if abs(wz) > max_angular:
        raise ValueError(f"angular speed is over the commissioning cap of {max_angular} rad/s")
    if not 0.1 <= duration <= max_duration:
        raise ValueError(f"duration_s must be in [0.1, {max_duration}]")
    return vx, vy, wz, duration


def populate_switch_request(request, activate, deactivate) -> None:
    """Fill a `SwitchController` request for either generation of the interface.

    Humble installs exist with `start/stop_controllers` and with the renamed
    `activate/deactivate_controllers`; the installed message says which one this robot has.
    """
    if hasattr(request, "activate_controllers"):
        request.activate_controllers = list(activate)
        request.deactivate_controllers = list(deactivate)
        asap_field = "activate_asap"
    elif hasattr(request, "start_controllers"):
        request.start_controllers = list(activate)
        request.stop_controllers = list(deactivate)
        asap_field = "start_asap"
    else:
        raise RuntimeError("unsupported controller_manager SwitchController interface")
    request.strictness = 2                      # STRICT: all or nothing
    setattr(request, asap_field, True)
    request.timeout.sec = 5
