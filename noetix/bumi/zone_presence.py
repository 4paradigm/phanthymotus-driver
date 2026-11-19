"""Reusable, SDK-independent occupancy processor for VOP centre coordinates."""
from __future__ import annotations

import math
import time


def number(value, low, high):
    if type(value) not in (int, float) or not math.isfinite(value) or not low <= value <= high:
        raise ValueError(f"Expected a finite number in {low}..{high}")
    return float(value)


class Occupancy:
    def __init__(self, *, region=(-.6, -.8, .6, 1), enter_s=2, leave_s=1,
                 stale_s=2, max_gap_s=.75, confidence=.5):
        if not isinstance(region, (list, tuple)) or len(region) != 4:
            raise ValueError("region requires [left, top, right, bottom]")
        self.region = tuple(number(v, -1, 1) for v in region)
        if self.region[0] >= self.region[2] or self.region[1] >= self.region[3]:
            raise ValueError("Region must have positive width and height")
        self.enter_s = number(enter_s, .1, 60)
        self.leave_s = number(leave_s, .1, 60)
        self.stale_s = number(stale_s, .1, 60)
        self.max_gap_s = number(max_gap_s, .05, self.stale_s)
        self.confidence = number(confidence, 0, 1)
        self.state = "unknown"
        self.last = None
        self.pending = None
        self.since = None
        self.occupied_since = None
        self.count = None
        self.sequence = 0
        self.error = "waiting_for_input"

    def _unknown(self, reason):
        changed = self.state != "unknown"
        self.state, self.pending, self.since = "unknown", None, None
        self.occupied_since, self.count, self.error = None, None, reason
        return self._event("unknown") if changed else None

    def _event(self, name):
        self.sequence += 1
        return {"event": name, "sequence": self.sequence, "state": self.state,
                "priority": 1, "count": self.count, "reason": self.error}

    def update(self, payload, now=None):
        now = time.monotonic() if now is None else number(now, 0, 1e15)
        try:
            if not isinstance(payload, dict) or not isinstance(payload.get("objects"), list):
                raise ValueError("Expected VOP objects array")
            count = 0
            for obj in payload["objects"]:
                if not isinstance(obj, dict):
                    raise ValueError("Invalid object record")
                if obj.get("name") != "person":
                    continue
                score = number(obj.get("confidence"), 0, 1)
                if score < self.confidence:
                    continue
                position = obj.get("position")
                if not isinstance(position, list) or len(position) != 2:
                    raise ValueError("Person requires VOP centre-relative position")
                x, y = (number(v, -1, 1) for v in position)
                left, top, right, bottom = self.region
                count += int(left <= x <= right and top <= y <= bottom)
        except ValueError as exc:
            event = self._unknown(str(exc))
            self.last = None
            return event
        if self.last is not None and now <= self.last:
            return None
        gap_event = None
        if self.last is not None and now-self.last > self.max_gap_s:
            gap_event = self._unknown("input_gap")
        self.last, self.count, self.error = now, count, ""
        target = "occupied" if count else "empty"
        if target == self.state:
            self.pending, self.since = None, None
            return gap_event
        if self.pending != target:
            self.pending, self.since = target, now
        duration = self.enter_s if target == "occupied" else self.leave_s
        if now-self.since < duration:
            return gap_event
        previous = self.state
        self.state, self.pending, self.since = target, None, None
        self.occupied_since = now if target == "occupied" else None
        return self._event("entered" if target == "occupied" else
                           "left" if previous == "occupied" else "empty")

    def read(self, now=None):
        now = time.monotonic() if now is None else now
        event = None
        if self.last is None or now-self.last > self.stale_s:
            event = self._unknown(self.error or "input_stale")
        return {"state": self.state, "count": self.count,
                "dwell_s": max(0, now-self.occupied_since) if self.occupied_since is not None else 0,
                "input_age_s": now-self.last if self.last is not None else None,
                "reason": self.error, "region": list(self.region),
                "coordinate_convention": "vop_center_relative_-1_to_1",
                "occupancy_basis": "person_detection_box_center",
                "identity_tracking": False}, event
