"""Read-only S10 joint model and hardware-to-model angle mapping.

The joint order, directions, and offsets come from the vendor S10 SDK's
S10Interface and s10_policy_runner. The URDF skeleton is generated from
S10_description/s10_mjcf/mjcf/S10.xml; it contains only link origins and axes.
"""

from __future__ import annotations

import math
from pathlib import Path


JOINT_NAMES = tuple(
    f"{leg}_{part}_joint"
    for leg in ("fl", "fr", "hl", "hr")
    for part in ("hipx", "hipy", "knee", "wheel")
)
JOINT_DIRECTIONS = (1, 1, -1, 1, 1, -1, 1, -1,
                    -1, 1, -1, 1, -1, -1, 1, -1)
JOINT_OFFSETS_DEG = (-35, -145, 156, 0, 35, -145, 156, 0,
                     -35, 145, -156, 0, 35, 145, -156, 0)
MODEL_PATH = Path(__file__).with_name("s10_skeleton.urdf")


def model_description():
    return {"urdf": MODEL_PATH.read_text(encoding="utf-8")}


def model_joint_name(index, record):
    """Only label a sample when its hardware number matches its array index."""
    if not 0 <= index < len(JOINT_NAMES) or not isinstance(record, dict):
        return None
    raw = record.get("name")
    if not isinstance(raw, list) or not all(type(v) is int and 0 <= v < 256 for v in raw):
        return None
    try:
        encoded = bytes(raw).split(b"\0", 1)[0].decode("ascii").strip()
    except (UnicodeError, ValueError):
        return None
    return JOINT_NAMES[index] if encoded == str(index) else None


def model_angle(index, angle):
    if type(angle) not in (int, float) or not math.isfinite(angle):
        return None
    return angle * JOINT_DIRECTIONS[index] + math.radians(JOINT_OFFSETS_DEG[index])
