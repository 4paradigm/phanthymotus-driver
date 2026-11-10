"""ROS-free validation and data contracts for the UniX AI Panther driver."""

from __future__ import annotations

import math
import time
import xml.etree.ElementTree as ET
from typing import Optional


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


def finite_vector(value, field: str, length: int) -> list[float]:
    if not isinstance(value, (list, tuple)) or len(value) != length:
        raise ValueError(f"{field} must contain exactly {length} numbers")
    return [finite_number(item, f"{field}[{index}]") for index, item in enumerate(value)]


def require_confirmation(args: dict) -> None:
    if args.get("confirm_motion") is not True:
        raise ValueError("confirm_motion=true is required before sending a motion command")


def parse_joint_limits(urdf: str) -> dict[str, tuple[float, float]]:
    """Read authoritative revolute/prismatic limits from the live URDF."""
    if not isinstance(urdf, str) or not urdf.strip():
        return {}
    root = ET.fromstring(urdf)
    limits: dict[str, tuple[float, float]] = {}
    for joint in root.findall("joint"):
        name = joint.get("name")
        limit = joint.find("limit")
        if not name or limit is None:
            continue
        lower, upper = limit.get("lower"), limit.get("upper")
        if lower is None or upper is None:
            continue
        try:
            lo, hi = float(lower), float(upper)
        except ValueError:
            continue
        if math.isfinite(lo) and math.isfinite(hi) and lo <= hi:
            limits[name] = (lo, hi)
    return limits


def parse_joint_indices(urdf: str) -> dict[str, int]:
    """Return stable kinematic-joint indices from the live URDF.

    ``ros2_control`` blocks may repeat joint names without parent/child links;
    those are controller declarations, not members of the visual kinematic
    tree, and must not affect skeleton indices.
    """
    if not isinstance(urdf, str) or not urdf.strip():
        return {}
    root = ET.fromstring(urdf)
    return {
        joint.get("name"): index
        for index, joint in enumerate(
            item for item in root.findall("joint")
            if item.get("name") and item.find("parent") is not None and item.find("child") is not None
        )
    }


def validate_joint_target(
    joint_names: tuple[str, ...],
    positions,
    limits: dict[str, tuple[float, float]],
    current: dict[str, float],
    max_delta_rad: float,
) -> list[float]:
    target = finite_vector(positions, "positions", len(joint_names))
    missing_limits = [name for name in joint_names if name not in limits]
    if missing_limits:
        raise ValueError(
            "live robot_description has no limits for: " + ", ".join(missing_limits)
        )
    missing_state = [name for name in joint_names if name not in current]
    if missing_state:
        raise ValueError("fresh joint state is missing: " + ", ".join(missing_state))
    max_delta = finite_number(max_delta_rad, "max_delta_rad")
    if max_delta <= 0:
        raise ValueError("max_delta_rad must be positive")
    for name, value in zip(joint_names, target):
        lower, upper = limits[name]
        if not lower <= value <= upper:
            raise ValueError(f"{name} target {value} is outside URDF limit [{lower}, {upper}]")
        delta = abs(value - float(current[name]))
        if delta > max_delta:
            raise ValueError(
                f"{name} target delta {delta:.4f} rad exceeds commissioning gate "
                f"{max_delta:.4f} rad"
            )
    return target


def validate_drive(args: dict, config: dict) -> tuple[float, float, float, float]:
    require_confirmation(args)
    vx = finite_number(args.get("linear_x", 0.0), "linear_x")
    vy = finite_number(args.get("linear_y", 0.0), "linear_y")
    wz = finite_number(args.get("angular_z", 0.0), "angular_z")
    duration = finite_number(args.get("duration_s", 1.0), "duration_s")
    max_linear = finite_number(config["max_linear_mps"], "max_linear_mps")
    max_angular = finite_number(config["max_angular_radps"], "max_angular_radps")
    max_duration = finite_number(config["max_duration_s"], "max_duration_s")
    if abs(vx) > max_linear or abs(vy) > max_linear:
        raise ValueError(f"linear velocity exceeds configured commissioning cap {max_linear} m/s")
    if abs(wz) > max_angular:
        raise ValueError(f"angular velocity exceeds configured commissioning cap {max_angular} rad/s")
    if not 0.1 <= duration <= max_duration:
        raise ValueError(f"duration_s must be in [0.1, {max_duration}]")
    return vx, vy, wz, duration


def fresh(received_monotonic: Optional[float], max_age_s: float = 2.0) -> bool:
    return received_monotonic is not None and time.monotonic() - received_monotonic <= max_age_s


def age_ms(received_monotonic: Optional[float]) -> Optional[int]:
    if received_monotonic is None:
        return None
    return max(0, int((time.monotonic() - received_monotonic) * 1000))


def json_float(value):
    try:
        value = float(value)
    except (TypeError, ValueError):
        return None
    return value if math.isfinite(value) else None


def populate_switch_request(request, activate, deactivate) -> None:
    """Support both controller_manager SwitchController field generations.

    Humble installations exist with ``start/stop`` and with the renamed
    ``activate/deactivate`` fields.  We inspect the installed interface rather
    than assuming which controller_manager patch level Panther carries.
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
    request.strictness = 2
    setattr(request, asap_field, True)
    request.timeout.sec = 5

