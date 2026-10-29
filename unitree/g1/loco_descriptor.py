"""Strict offline validation of a caller-supplied G1 base control descriptor.

No limits or rates are supplied here. A passing descriptor only establishes
shape and numerical consistency, not parameter authority or hardware safety.
The standard twist convention is (vx, vy, vz, wx, wy, wz); joint_names and
frame are existing descriptor fields. This G1 interface requires base_link.
Command validation, source ownership, collision prevention and execution are
outside this module. In particular, the sink clamps before checking hard bounds,
so callers must reject nonzero inactive axes in the original command separately.
"""

import copy
import math

from common.control.descriptor import DescriptorError, parse_descriptor

AXES = ("vx", "vy", "vz", "wx", "wy", "wz")
_ACTIVE = (0, 1, 5)
_INACTIVE = (2, 3, 4)


def _finite(value, name, *, positive=False, integer=False):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise DescriptorError(f"descriptor.{name} must be a finite number")
    try:
        finite = math.isfinite(value)
    except OverflowError:
        finite = False
    if not finite or (positive and value <= 0):
        raise DescriptorError(f"descriptor.{name} must be finite" + (" and positive" if positive else ""))
    if integer and (int(value) != value or value < 1):
        raise DescriptorError(f"descriptor.{name} must be a positive whole millisecond")
    return value


def _array(value, name, *, positive=False):
    if not isinstance(value, (list, tuple)) or len(value) != 6:
        raise DescriptorError(f"descriptor.{name} must contain six numbers")
    return tuple(_finite(item, f"{name}[{index}]", positive=positive)
                 for index, item in enumerate(value))


def validate_loco_descriptor(raw):
    """Return the common parsed Descriptor or raise DescriptorError.

    The caller must explicitly supply lower/upper, max_delta_per_step, all
    three rate fields plus max_obs_age_ms, and force_torque (possibly null).
    Optional standard fields are checked when present. G1 requires group units
    to be absent or empty strings: no unambiguous mixed twist-unit syntax is
    defined here, so top-level units are authoritative. The common parser is
    unchanged. No G1 value is inferred.
    """
    if not isinstance(raw, dict):
        raise DescriptorError("descriptor must be an object")
    data = copy.deepcopy(raw)
    if data.get("mode") != "twist" or type(data.get("dof")) is not int or data["dof"] != 6:
        raise DescriptorError("descriptor must declare mode twist and dof 6")
    names = data.get("joint_names")
    if not isinstance(names, (list, tuple)) or tuple(names) != AXES:
        raise DescriptorError("descriptor.joint_names must explicitly declare vx, vy, vz, wx, wy, wz in order")
    if data.get("frame") != "base_link":
        raise DescriptorError("descriptor.frame must explicitly declare body frame base_link")
    units = data.get("units")
    if not isinstance(units, dict) or any(units.get(k) != v for k, v in
                                          (("linear", "m/s"), ("angular", "rad/s"), ("time", "s"))):
        raise DescriptorError("descriptor.units must explicitly declare m/s, rad/s and s")
    limits = data.get("limits")
    if not isinstance(limits, dict):
        raise DescriptorError("descriptor.limits must be an object")
    lower = _array(limits.get("lower"), "limits.lower")
    upper = _array(limits.get("upper"), "limits.upper")
    for i in _ACTIVE:
        if not lower[i] < 0 < upper[i]:
            raise DescriptorError(f"descriptor.limits axis {AXES[i]} must have finite negative and positive bounds")
    for i in _INACTIVE:
        if lower[i] != 0 or upper[i] != 0:
            raise DescriptorError(f"descriptor.limits axis {AXES[i]} must be pinned to zero")
    _array(limits.get("max_delta_per_step"), "limits.max_delta_per_step", positive=True)
    if limits.get("max_velocity") is not None:
        _array(limits["max_velocity"], "limits.max_velocity", positive=True)
    for name in ("min_magnitude", "min_magnitude_moving"):
        if name in limits:
            values = _array(limits[name], f"limits.{name}")
            for i, value in enumerate(values):
                if value < 0 or (i in _INACTIVE and value != 0) or (i in _ACTIVE and value > min(-lower[i], upper[i])):
                    raise DescriptorError(f"descriptor.limits.{name}[{i}] is inconsistent with the bounds")
    rate = data.get("rate")
    if not isinstance(rate, dict):
        raise DescriptorError("descriptor.rate must be an object")
    for name in ("watchdog_ms", "max_obs_age_ms"):
        _finite(rate.get(name), f"rate.{name}", positive=True, integer=True)
    for name in ("max_hz", "expected_hz"):
        _finite(rate.get(name), f"rate.{name}", positive=True)
    if rate["expected_hz"] > rate["max_hz"]:
        raise DescriptorError("descriptor.rate.expected_hz exceeds max_hz")
    if "force_torque" not in data:
        raise DescriptorError("descriptor.force_torque must be explicitly declared, possibly null")
    force = data["force_torque"]
    if force is not None:
        if not isinstance(force, (list, tuple)) or not force:
            raise DescriptorError("descriptor.force_torque must be null or a non-empty list")
        for index, value in enumerate(force):
            _finite(value, f"force_torque[{index}]", positive=True)
    descriptor = parse_descriptor(data)
    if any(group.advisory or group.mode != "twist" for group in descriptor.groups):
        raise DescriptorError("descriptor.groups must use twist without advisory axes")
    for group in data.get("groups") or ():
        if "unit" in group and group["unit"] != "":
            raise DescriptorError("descriptor.groups unit must be absent or empty for G1 twist")
    return descriptor
