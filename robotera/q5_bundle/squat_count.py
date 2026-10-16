"""Squat counter based on discrete posture labels (standing, sitting, crouching).

Counts a repetition if both an 'up' posture (standing) and a 'down' posture
(sitting or crouching) are observed for the same tracked person within a
configurable time window. Continuous transitions are NOT required.
"""

import math

# Configurable labels
POSTURES_DOWN = {"sitting", "crouching"}
POSTURES_UP = {"standing"}


class SquatCounter:
    def __init__(self, lost_s=3.0, window_s=3.0, min_confidence=0.5):
        """
        Args:
            lost_s: seconds without tracked person before reset.
            window_s: time window to look for both up and down postures.
            min_confidence: minimum posture_confidence to accept a label.
        """
        self.lost_s = float(lost_s)
        self.window_s = float(window_s)
        self.min_confidence = float(min_confidence)
        self.reset()

    def reset(self):
        self.count = 0
        self.track_id = None
        self.seen_postures = set()
        self.window_start = 0.0
        self.last_seen = None

    def _drop_track(self):
        self.track_id = None
        self.seen_postures = set()
        self.window_start = 0.0
        self.last_seen = None

    def update(self, persons, now):
        """Return a completion event dict or None."""
        if not isinstance(persons, list) or not math.isfinite(now):
            return None

        # Expire track if lost too long
        if self.track_id is not None and self.last_seen is not None:
            if now - self.last_seen > self.lost_s:
                self._drop_track()

        # Find tracked person
        person = next(
            (p for p in persons if isinstance(p, dict) and p.get("id") == self.track_id),
            None,
        )

        # Acquire a new track from the first person with any valid posture
        if self.track_id is None:
            person = next(
                (p for p in persons
                 if isinstance(p, dict)
                 and isinstance(p.get("id"), int)
                 and self._valid_posture(p.get("posture"), p.get("posture_confidence"))),
                None,
            )
            if person is None:
                return None
            self.track_id = person["id"]

        if person is None:
            return None

        posture = person.get("posture")
        if not self._valid_posture(posture, person.get("posture_confidence")):
            return None

        self.last_seen = now

        # Update observation window
        if self.window_start == 0.0:
            self.window_start = now

        self.seen_postures.add(posture)

        # ── Set-based detection ─────────────────────────────────────────
        # If we've seen BOTH an "up" posture and a "down" posture in the
        # current window, count it as a complete squat.
        has_up = bool(self.seen_postures & POSTURES_UP)
        has_down = bool(self.seen_postures & POSTURES_DOWN)

        if has_up and has_down:
            self.count += 1
            self.seen_postures = set()
            self.window_start = 0.0
            return {
                "type": "squat_completed",
                "priority": 1,
                "track_id": self.track_id,
                "count": self.count,
            }

        # If the window expires without seeing both, reset to avoid false
        # positives from just standing still for a long time.
        if now - self.window_start > self.window_s:
            self.seen_postures = set()
            self.window_start = now

        return None

    def _valid_posture(self, posture, confidence):
        if posture not in POSTURES_UP | POSTURES_DOWN:
            return False
        if not isinstance(confidence, (int, float)) or not math.isfinite(confidence):
            return False
        return confidence >= self.min_confidence
