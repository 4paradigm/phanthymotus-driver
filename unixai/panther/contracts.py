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


def optional_number(value, field: str):
    """A number from a form field, or None when the field was left empty.

    The canvas sends an empty number box as missing, `None` or `""`; a typed number may arrive as
    a number or as its text. Anything else (a boolean, words, NaN, infinity) is an error, never
    silently "empty": a value the operator typed must not be dropped.
    """
    if value is None or (isinstance(value, str) and not value.strip()):
        return None
    if isinstance(value, str):
        try:
            value = float(value.strip())
        except ValueError:
            raise ValueError(f"{field} must be a number") from None
    return finite_number(value, field)


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


def validate_joint_target(joint_names, positions, limits: dict, current: dict) -> list:
    """A finite target inside the URDF limits, with finite current joint readings."""
    target = finite_vector(positions, "positions", len(joint_names))
    missing = [name for name in joint_names if name not in limits]
    if missing:
        raise ValueError("no joint limits known for: " + ", ".join(missing))
    unseen = [name for name in joint_names if name not in current]
    if unseen:
        raise ValueError("fresh joint state is missing: " + ", ".join(unseen))
    for name, value in zip(joint_names, target):
        finite_number(current[name], f"current.{name}")
        lower, upper = limits[name]
        if not lower <= value <= upper:
            raise ValueError(f"{name} target {value} is outside its range {lower} ~ {upper} rad (URDF limit)")
    return target


def validate_drive(args: dict, config: dict) -> tuple:
    """`(vx, vy, wz, duration_s)`, or ValueError. Caps come from config, never from the caller.
    An empty speed box is 0; an empty duration is 1 s."""
    vx = optional_number(args.get("linear_x"), "linear_x") or 0.0
    vy = optional_number(args.get("linear_y"), "linear_y") or 0.0
    wz = optional_number(args.get("angular_z"), "angular_z") or 0.0
    duration = optional_number(args.get("duration_s"), "duration_s")
    duration = 1.0 if duration is None else duration
    max_linear = finite_number(config["max_linear_mps"], "max_linear_mps")
    max_angular = finite_number(config["max_angular_radps"], "max_angular_radps")
    max_duration = finite_number(config["max_duration_s"], "max_duration_s")
    if abs(vx) > max_linear or abs(vy) > max_linear:
        raise ValueError(f"linear speed must be within -{max_linear:g} ~ {max_linear:g} m/s")
    if abs(wz) > max_angular:
        raise ValueError(f"angular speed must be within -{max_angular:g} ~ {max_angular:g} rad/s")
    if not 0.1 <= duration <= max_duration:
        raise ValueError(f"duration_s must be within 0.1 ~ {max_duration:g} s")
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
