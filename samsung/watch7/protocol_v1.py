"""Stable wire contract shared by simulated and real Galaxy Watch7 clients."""
from __future__ import annotations

import math
import time
from typing import Any

SAMPLES_TOPIC = "/wearable/watch7/samples"
ACKS_TOPIC = "/wearable/watch7/acks"
PROTOCOL_VERSION = 1
MAX_SKEW_MS = 60_000
MAX_AGE_MS = 30_000


def _number(value: Any, field: str, minimum: float, maximum: float) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{field} must be numeric")
    num = float(value)
    if not math.isfinite(num) or not minimum <= num <= maximum:
        raise ValueError(f"{field} must be within {minimum}..{maximum}")
    return num


def validate_frame(raw: Any, *, now_ms: int | None = None) -> dict:
    """Reject unknown, stale or malformed health data before ROS2 publication."""
    if not isinstance(raw, dict):
        raise ValueError("frame must be an object")
    if raw.get("protocol_version") != PROTOCOL_VERSION:
        raise ValueError("unsupported protocol_version")
    if raw.get("device_id") != "watch7":
        raise ValueError("unsupported device_id")
    if raw.get("source") not in ("watch7_real", "simulated"):
        raise ValueError("source must be watch7_real or simulated")
    session = raw.get("session_id")
    if not isinstance(session, str) or not 1 <= len(session) <= 64 or not session.isascii():
        raise ValueError("invalid session_id")
    seq = raw.get("seq")
    if type(seq) is not int or not 0 <= seq < 2**63:
        raise ValueError("seq must be nonnegative integer")
    ts = raw.get("captured_at_ms")
    if type(ts) is not int or ts < 0:
        raise ValueError("captured_at_ms must be Unix milliseconds")
    now = int(time.time() * 1000) if now_ms is None else now_ms
    if ts > now + MAX_SKEW_MS or ts < now - MAX_AGE_MS:
        raise ValueError("frame timestamp outside freshness window")
    bpm = _number(raw.get("bpm"), "bpm", 25, 240)
    motion = raw.get("motion_score")
    if motion is None:
        clean_motion = None
    else:
        clean_motion = _number(motion, "motion_score", 0, 60)
    return {
        "protocol_version": PROTOCOL_VERSION,
        "device_id": "watch7",
        "source": raw["source"],
        "session_id": session,
        "seq": seq,
        "captured_at_ms": ts,
        "bpm": bpm,
        "motion_score": clean_motion,
    }
