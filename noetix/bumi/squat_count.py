"""Per-person standing -> crouching -> standing repetition counter."""

import math


class SquatCounter:
    def __init__(self, hold_s=0.2, lost_s=1.0, min_confidence=0.5):
        self.hold_s = float(hold_s)
        self.lost_s = float(lost_s)
        self.min_confidence = float(min_confidence)
        self.reset()

    def reset(self):
        self.count = 0
        self.track_id = None
        self.phase = "waiting"
        self.candidate = None
        self.candidate_since = 0.0
        self.last_seen = None

    def _drop_track(self):
        self.track_id = None
        self.phase = "waiting"
        self.candidate = None
        self.last_seen = None

    def update(self, persons, now):
        """Return a completion event or None. `now` is a monotonic frame time."""
        if not isinstance(persons, list) or not math.isfinite(now):
            return None
        if self.track_id is not None and self.last_seen is not None:
            if now - self.last_seen > self.lost_s:
                self._drop_track()

        person = next((p for p in persons if isinstance(p, dict)
                       and p.get("id") == self.track_id), None)
        if self.track_id is None:
            # The first visible standing subject owns this counting session.
            person = next((p for p in persons if isinstance(p, dict)
                           and isinstance(p.get("id"), int)
                           and p.get("posture") == "standing"
                           and self._confident(p)), None)
            if person is None:
                return None
            self.track_id = person["id"]

        if person is None:
            return None
        posture = person.get("posture") if self._confident(person) else None
        if posture not in ("standing", "crouching"):
            self.candidate = None
            if posture == "sitting":
                self.phase = "waiting"
            return None
        self.last_seen = now
        if posture != self.candidate:
            self.candidate = posture
            self.candidate_since = now
            return None
        if now - self.candidate_since < self.hold_s:
            return None

        if self.phase == "waiting" and posture == "standing":
            self.phase = "up"
        elif self.phase == "up" and posture == "crouching":
            self.phase = "down"
        elif self.phase == "down" and posture == "standing":
            self.phase = "up"
            self.count += 1
            return {"type": "squat_completed", "priority": 1,
                    "track_id": self.track_id, "count": self.count}
        return None

    def _confident(self, person):
        confidence = person.get("posture_confidence")
        return (isinstance(confidence, (int, float))
                and math.isfinite(confidence)
                and confidence >= self.min_confidence)
