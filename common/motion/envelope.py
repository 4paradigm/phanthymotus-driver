"""Immutable, lightweight execution proof checks; no numerical dependencies.

Only the numerical worker may issue records after checking the whole independent
joint box. This module validates and consumes that record, never creates a proof
by inspecting endpoints or trusting a public command payload.
"""
from dataclasses import dataclass
import math

SCHEMA = "motus.motion.envelope/1"
IDENTITY = ("boot_id", "session_id", "seq", "source_seq", "mapping_epoch",
            "model_version", "calibration_version", "frame", "valid_until_ns")


def _vector(value, dof):
    if not isinstance(value, (list, tuple)) or len(value) != dof:
        raise ValueError("motion_envelope_dimension")
    if any(type(x) not in (int, float) or not math.isfinite(x) for x in value):
        raise ValueError("motion_envelope_nonfinite")
    return tuple(float(x) for x in value)


@dataclass(frozen=True)
class MotionEnvelope:
    identity: tuple
    lower: tuple
    upper: tuple
    measured_ns: int

    @classmethod
    def from_record(cls, record, *, dof):
        if not isinstance(record, dict) or record.get("schema") != SCHEMA:
            raise ValueError("motion_envelope_schema")
        if set(record) != set(IDENTITY) | {"schema", "lower", "upper", "measured_ns"}:
            raise ValueError("motion_envelope_fields")
        for key in ("seq", "source_seq", "mapping_epoch", "valid_until_ns", "measured_ns"):
            if type(record[key]) is not int or record[key] < 0:
                raise ValueError("motion_envelope_identity")
        for key in ("boot_id", "session_id", "model_version", "calibration_version", "frame"):
            if not isinstance(record[key], str) or not record[key]:
                raise ValueError("motion_envelope_identity")
        lower, upper = _vector(record["lower"], dof), _vector(record["upper"], dof)
        if any(lo > hi for lo, hi in zip(lower, upper)):
            raise ValueError("motion_envelope_bounds")
        if record["measured_ns"] >= record["valid_until_ns"]:
            raise ValueError("motion_envelope_expired")
        return cls(tuple(record[k] for k in IDENTITY), lower, upper, record["measured_ns"])

    def matches(self, command, *, now_ns):
        return (type(now_ns) is int and self.measured_ns <= now_ns < self.identity[-1]
                and tuple(command.get(k) for k in IDENTITY) == self.identity)

    def contains(self, value):
        try:
            value = _vector(value, len(self.lower))
        except (ValueError, OverflowError):
            return False
        return all(lo <= q <= hi for lo, q, hi in zip(self.lower, value, self.upper))

    def allows(self, command, measured, previous, next_position, *, now_ns):
        return (self.matches(command, now_ns=now_ns)
                and all(self.contains(q) for q in (measured, previous, next_position)))
