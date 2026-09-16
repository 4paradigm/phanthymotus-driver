"""The action space a driver declares it accepts.

A driver's command card returns this from `info()`. It is the single
authoritative description of the action interface — joint order, units, limits,
rate — and everything upstream negotiates against it before a card is allowed
to start.

Validation is strict and names the offending field. A descriptor is written by
hand once per driver and then drives motors forever; a silent typo in it is not
a bug that shows up as an exception somewhere, it is a robot moving to the
wrong place. `parse_descriptor` therefore rejects rather than defaults.
"""

from __future__ import annotations

from dataclasses import dataclass, field

SCHEMA = "motus.control/1"

# Modes a driver may declare. The set is deliberately small: each one is a
# different contract about what `values` means, and a mode nobody implements is
# a mode nobody has tested.
MODES = (
    "joint_position",    # absolute joint positions, descriptor.units["angle"]
    "joint_velocity",    # joint velocities
    "joint_torque",      # joint torques
    "eef_pose",          # end-effector pose in descriptor.frame
    "twist",             # body twist (vx, vy, vz, wx, wy, wz)
)


class DescriptorError(ValueError):
    """A descriptor that cannot be trusted to drive a motor."""


@dataclass(frozen=True)
class Group:
    """A contiguous slice of the action vector with its own unit and resource.

    A single-arm robot needs none of this: one unit, one physical channel, one
    `units` dict covers it. A humanoid does not. Tianyi's action vector is
    fourteen arm joints in radians followed by twelve finger values that are
    normalised closure — one `units` mapping cannot describe both, and the two
    halves are separate physical channels that an ACP barrier should be able to
    tell apart.

    Optional and backward compatible: a descriptor without `groups` is one
    group covering everything, which is what every existing driver means.
    """

    name: str
    offset: int
    count: int
    unit: str = ""
    resource: str = ""

    @property
    def slice(self) -> slice:
        return slice(self.offset, self.offset + self.count)


@dataclass(frozen=True)
class Descriptor:
    """A validated action space declaration.

    Frozen because the sink caches derived values from it; a descriptor that
    changes under a running sink would silently invalidate the limits every
    command has already been checked against.
    """

    mode: str
    dof: int
    joint_names: tuple[str, ...]
    units: dict
    lower: tuple[float, ...]
    upper: tuple[float, ...]
    watchdog_ms: int
    max_hz: float
    expected_hz: float
    max_velocity: tuple[float, ...] | None = None
    max_delta_per_step: tuple[float, ...] | None = None
    max_obs_age_ms: int | None = None
    frame: str = ""
    end_effector: dict | None = None
    # Absolute per-axis force/torque thresholds. `None` means this robot has no
    # force-torque sensing — declared explicitly rather than omitted, so that a
    # missing protection is visible instead of assumed. See sink.ControlSink.
    force_torque: tuple[float, ...] | None = None
    groups: tuple[Group, ...] = ()
    raw: dict = field(default_factory=dict, repr=False)

    @property
    def has_force_torque(self) -> bool:
        return self.force_torque is not None

    @property
    def resources(self) -> tuple[str, ...]:
        """Physical channels this action space occupies, for `x-resource`."""
        seen = []
        for group in self.groups:
            if group.resource and group.resource not in seen:
                seen.append(group.resource)
        return tuple(seen)


def _require(source: dict, key: str, where: str):
    if key not in source:
        raise DescriptorError(f"descriptor.{where}{key} is required")
    return source[key]


def _number_list(raw, *, dof: int, where: str) -> tuple[float, ...]:
    if not isinstance(raw, (list, tuple)):
        raise DescriptorError(f"descriptor.{where} must be a list of {dof} numbers")
    if len(raw) != dof:
        raise DescriptorError(
            f"descriptor.{where} has {len(raw)} entries but dof is {dof}"
        )
    out = []
    for i, value in enumerate(raw):
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise DescriptorError(f"descriptor.{where}[{i}] is not a number: {value!r}")
        out.append(float(value))
    return tuple(out)


def parse_descriptor(raw: dict) -> Descriptor:
    """Validate a driver's declaration, or raise `DescriptorError` naming the field."""
    if not isinstance(raw, dict):
        raise DescriptorError("descriptor must be an object")

    interface = raw.get("control_interface")
    if interface != SCHEMA:
        raise DescriptorError(
            f"descriptor.control_interface must be {SCHEMA!r}, got {interface!r}"
        )

    mode = _require(raw, "mode", "")
    if mode not in MODES:
        raise DescriptorError(
            f"descriptor.mode {mode!r} is not one of {', '.join(MODES)}"
        )

    dof = _require(raw, "dof", "")
    if isinstance(dof, bool) or not isinstance(dof, int) or dof <= 0:
        raise DescriptorError(f"descriptor.dof must be a positive integer, got {dof!r}")

    joint_names = _require(raw, "joint_names", "")
    if not isinstance(joint_names, (list, tuple)):
        raise DescriptorError("descriptor.joint_names must be a list")
    if len(joint_names) != dof:
        raise DescriptorError(
            f"descriptor.joint_names has {len(joint_names)} entries but dof is {dof} — "
            "the order of this list is what gives `values` its meaning, so a "
            "mismatch here cannot be worked around downstream"
        )
    if any(not isinstance(n, str) or not n for n in joint_names):
        raise DescriptorError("descriptor.joint_names must be non-empty strings")

    units = _require(raw, "units", "")
    if not isinstance(units, dict) or not units:
        raise DescriptorError("descriptor.units must be a non-empty object")

    limits = _require(raw, "limits", "")
    if not isinstance(limits, dict):
        raise DescriptorError("descriptor.limits must be an object")
    lower = _number_list(_require(limits, "lower", "limits."), dof=dof, where="limits.lower")
    upper = _number_list(_require(limits, "upper", "limits."), dof=dof, where="limits.upper")
    for i, (lo, hi) in enumerate(zip(lower, upper)):
        if lo > hi:
            raise DescriptorError(
                f"descriptor.limits: lower[{i}]={lo} is above upper[{i}]={hi}"
            )

    max_velocity = limits.get("max_velocity")
    if max_velocity is not None:
        max_velocity = _number_list(max_velocity, dof=dof, where="limits.max_velocity")
        if any(v <= 0 for v in max_velocity):
            raise DescriptorError("descriptor.limits.max_velocity must be positive")

    max_delta = limits.get("max_delta_per_step")
    if max_delta is not None:
        max_delta = _number_list(max_delta, dof=dof, where="limits.max_delta_per_step")
        if any(d <= 0 for d in max_delta):
            raise DescriptorError("descriptor.limits.max_delta_per_step must be positive")

    rate = _require(raw, "rate", "")
    if not isinstance(rate, dict):
        raise DescriptorError("descriptor.rate must be an object")
    watchdog_ms = _require(rate, "watchdog_ms", "rate.")
    if isinstance(watchdog_ms, bool) or not isinstance(watchdog_ms, (int, float)) or watchdog_ms <= 0:
        raise DescriptorError("descriptor.rate.watchdog_ms must be a positive number")
    max_hz = float(rate.get("max_hz", 0) or 0)
    expected_hz = float(rate.get("expected_hz", 0) or 0)
    if max_hz and expected_hz and expected_hz > max_hz:
        raise DescriptorError(
            f"descriptor.rate.expected_hz {expected_hz} exceeds max_hz {max_hz}"
        )
    max_obs_age = rate.get("max_obs_age_ms")
    if max_obs_age is not None:
        if isinstance(max_obs_age, bool) or not isinstance(max_obs_age, (int, float)) or max_obs_age <= 0:
            raise DescriptorError("descriptor.rate.max_obs_age_ms must be a positive number")
        max_obs_age = int(max_obs_age)

    # `force_torque` must be present, even as null. Omitting it is how a robot
    # ends up assumed to have a protection it does not have.
    if "force_torque" not in raw:
        raise DescriptorError(
            "descriptor.force_torque is required — declare the per-axis absolute "
            "thresholds, or null if this robot has no force-torque sensing. "
            "Omitting it would let a missing protection pass as an oversight."
        )
    force_torque = raw["force_torque"]
    if force_torque is not None:
        if not isinstance(force_torque, (list, tuple)) or not force_torque:
            raise DescriptorError(
                "descriptor.force_torque must be null or a non-empty list of "
                "per-axis absolute thresholds"
            )
        cleaned = []
        for i, value in enumerate(force_torque):
            if isinstance(value, bool) or not isinstance(value, (int, float)) or value <= 0:
                raise DescriptorError(
                    f"descriptor.force_torque[{i}] must be a positive number, got {value!r}"
                )
            cleaned.append(float(value))
        force_torque = tuple(cleaned)

    end_effector = raw.get("end_effector")
    if end_effector is not None and not isinstance(end_effector, dict):
        raise DescriptorError("descriptor.end_effector must be an object or absent")

    groups = _parse_groups(raw.get("groups"), dof=dof)

    return Descriptor(
        mode=mode,
        dof=dof,
        joint_names=tuple(joint_names),
        units=dict(units),
        lower=lower,
        upper=upper,
        watchdog_ms=int(watchdog_ms),
        max_hz=max_hz,
        expected_hz=expected_hz,
        max_velocity=max_velocity,
        max_delta_per_step=max_delta,
        max_obs_age_ms=max_obs_age,
        frame=str(raw.get("frame", "") or ""),
        end_effector=end_effector,
        force_torque=force_torque,
        groups=groups,
        raw=dict(raw),
    )


def _parse_groups(raw, *, dof: int) -> tuple:
    """Validate `groups`, or default to one group covering the whole vector.

    Groups must tile `[0, dof)` exactly, in order and without gaps. A gap would
    leave dimensions with no declared unit and no owning channel — and since
    the whole point of a group is to say what a slice *means*, a dimension in
    no group is a dimension nobody has described.
    """
    if raw is None:
        return (Group(name="all", offset=0, count=dof),)
    if not isinstance(raw, (list, tuple)) or not raw:
        raise DescriptorError("descriptor.groups must be a non-empty list or absent")

    groups, expected = [], 0
    for i, entry in enumerate(raw):
        if not isinstance(entry, dict):
            raise DescriptorError(f"descriptor.groups[{i}] must be an object")
        name = entry.get("name")
        if not isinstance(name, str) or not name:
            raise DescriptorError(f"descriptor.groups[{i}].name is required")
        offset, count = entry.get("offset"), entry.get("count")
        for key, value in (("offset", offset), ("count", count)):
            if isinstance(value, bool) or not isinstance(value, int):
                raise DescriptorError(
                    f"descriptor.groups[{i}].{key} must be an integer")
        if count <= 0:
            raise DescriptorError(f"descriptor.groups[{i}].count must be positive")
        if offset != expected:
            raise DescriptorError(
                f"descriptor.groups[{i}] ({name}) starts at {offset}, expected "
                f"{expected} — groups must tile the vector in order with no gaps"
            )
        expected = offset + count
        groups.append(Group(name=name, offset=offset, count=count,
                            unit=str(entry.get("unit") or ""),
                            resource=str(entry.get("resource") or "")))

    if expected != dof:
        raise DescriptorError(
            f"descriptor.groups cover {expected} dimensions but dof is {dof}")
    return tuple(groups)
