"""Squat counter based on posture state transition detection.

Counts a repetition when the tracked person transitions from a 'down' posture
(sitting or crouching) to an 'up' posture (standing), with a minimum time
interval since the last counted squat to prevent double-counting.
"""

import math


# Configurable posture labels
POSTURES_DOWN = {"sitting", "crouching"}
POSTURES_UP = {"standing"}


class SquatCounter:
    def __init__(self, min_interval_s=0.8, lost_s=4.0, min_confidence=0.3):
        """
        Args:
            min_interval_s: minimum seconds between consecutive squats.
            lost_s: seconds without tracked person before reset.
            min_confidence: minimum posture_confidence to accept a label.
        """
        self.min_interval_s = float(min_interval_s)
        self.lost_s = float(lost_s)
        self.min_confidence = float(min_confidence)
        self.reset()

    def reset(self):
        self.count = 0
        self.track_id = None
        # Track current posture phase to detect transitions
        self.current_phase = None  # "up" or "down"
        self.last_squat_time = 0.0
        self.last_seen = None

    def _drop_track(self):
        self.track_id = None
        self.current_phase = None
        self.last_squat_time = 0.0
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

        # Determine phase
        is_down = posture in POSTURES_DOWN
        new_phase = "down" if is_down else "up"

        # ── State transition detection ──────────────────────────────────
        if self.current_phase is None:
            # First valid frame: initialize phase
            self.current_phase = new_phase
        elif self.current_phase == "down" and new_phase == "up":
            # Transition: down → up (crouching/sitting → standing)
            if now - self.last_squat_time >= self.min_interval_s:
                self.count += 1
                self.last_squat_time = now
                self.current_phase = "up"
                return {
                    "type": "squat_completed",
                    "priority": 1,
                    "track_id": self.track_id,
                    "count": self.count,
                }
            # else: too soon after last squat, ignore
        # else: same phase or up → down (waiting for next down phase)

        self.current_phase = new_phase
        return None

    def _valid_posture(self, posture, confidence):
        if posture not in POSTURES_UP | POSTURES_DOWN:
            return False
        if not isinstance(confidence, (int, float)) or not math.isfinite(confidence):
            return False
        return confidence >= self.min_confidence
