"""AS2W SportModeState -> motus.odom/1, without assuming a velocity frame.

Default: no verified axes and no pose. The SDK field names alone do not prove
that the velocity is body-relative, or that its signs/units match our contract.
After verification, configure the state card with e.g.::

    odom:
      frame: body
      verified_axes: [vx, vy, wz]
      verified_on: "unit/date and verification record"
      max_age_ms: 500

This is an operator declaration, not a measurement performed by this adapter.
Only velocity[0:3] and yaw_speed can be opted in; IMU/position are never used to
invent missing twist axes or a pose. In particular, position[2] is not yaw.
"""
from __future__ import annotations

import math

from common.odom import AXES, build_interface, build_sample, resolve_stamp_ms


def finite_number(value):
    """Unknown/malformed readings stay null, including non-finite SDK floats."""
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    return float(value) if math.isfinite(value) else None


def _vector(msg, key, size):
    value = getattr(msg, key, None)
    try:
        values = list(value)
    except TypeError:
        values = []
    return [finite_number(values[i]) if i < len(values) else None
            for i in range(size)]


def _vendor_stamp_ms(msg):
    stamp = getattr(msg, "stamp", None)
    sec = finite_number(getattr(stamp, "sec", None))
    nanosec = finite_number(getattr(stamp, "nanosec", None))
    if sec is None or nanosec is None or not 0 <= nanosec < 1_000_000_000:
        return None
    return sec * 1000 + nanosec / 1_000_000


class OdomAdapter:
    """Immutable-in-use declaration shared by state metadata and samples."""

    def __init__(self, config=None):
        config = {} if config is None else config
        if not isinstance(config, dict):
            raise ValueError("state.odom must be an object")
        raw_frame = config.get("frame", "unknown")
        if raw_frame not in ("unknown", "body", "world"):
            raise ValueError("state.odom.frame must be unknown, body or world")
        axes = config.get("verified_axes", [])
        if not isinstance(axes, (list, tuple)):
            raise ValueError("state.odom.verified_axes must be a list")
        if any(axis not in ("vx", "vy", "vz", "wz") for axis in axes):
            raise ValueError("AS2W verified_axes may only contain vx, vy, vz, wz")
        if len(set(axes)) != len(axes):
            raise ValueError("state.odom.verified_axes contains duplicates")
        if axes and raw_frame == "unknown":
            raise ValueError("verified odom axes require an explicit body/world frame")
        max_age = finite_number(config.get("max_age_ms", 500))
        if max_age is None or max_age <= 0:
            raise ValueError("state.odom.max_age_ms must be finite and positive")
        self.raw_frame = raw_frame
        # motus.odom/1 has no 'unknown' frame. With no opted-in axes, this is an
        # empty body-frame contract, NOT a claim about the vendor velocity.
        self.frame = "body" if raw_frame == "unknown" else raw_frame
        self.axes = tuple(axis for axis in AXES if axis in axes)
        self.max_age_ms = max_age
        self.verified_on = str(config.get("verified_on", ""))

    def _provenance(self):
        return {"source_topic": "rt/lf/sportmodestate",
                "raw_velocity_frame": self.raw_frame,
                "verified_axes": list(self.axes),
                "verification": "operator-configured" if self.axes else "unverified",
                "verified_on": self.verified_on}

    def interface(self, publish_hz=60.0):
        out = build_interface(provides=self.axes, frame=self.frame,
                              rate_hz=publish_hz, pose_drift="none")
        out["vendor"] = {**self._provenance(),
                         "rate_hz_kind": "publish_ceiling",
                         "max_age_ms": self.max_age_ms}
        return out

    def sample(self, msg, *, received_ms):
        velocity = _vector(msg, "velocity", 3)
        yaw_speed = finite_number(getattr(msg, "yaw_speed", None))
        measured = velocity + [None, None, yaw_speed]
        twist = [measured[i] if axis in self.axes else None
                 for i, axis in enumerate(AXES)]
        stamp_ms, stamp_info = resolve_stamp_ms(
            vendor_ms=_vendor_stamp_ms(msg), received_ms=received_ms)
        vendor = {**self._provenance(), **stamp_info,
                  "received_ms": received_ms,
                  "velocity": velocity, "yaw_speed": yaw_speed,
                  "position": _vector(msg, "position", 3),
                  "mode": finite_number(getattr(msg, "mode", None)),
                  "body_height": finite_number(getattr(msg, "body_height", None))}
        return build_sample(stamp_ms=stamp_ms, twist=twist, frame=self.frame,
                            pose=None, vendor=vendor)
