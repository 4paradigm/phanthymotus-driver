"""Squat counter based on posture-label trend detection.

Instead of requiring strict frame-by-frame transitions or a simple set
collection, this counter maintains a sliding window of recent posture
observations for each tracked person.  A squat is counted when:

1. The window shows a sustained "down" phase (down-posture ratio ≥ threshold)
2. This is followed by an "up" phase (standing posture)

This is robust to transient misclassifications during fast transitions.
"""

import math
from collections import deque

# Configurable posture labels
POSTURES_DOWN = {"sitting", "crouching"}
POSTURES_UP = {"standing"}


class SquatCounter:
    def __init__(self, window_s=2.0, down_ratio=0.5, lost_s=3.0,
                 min_confidence=0.5):
        """
        Args:
            window_s: sliding window duration in seconds.
            down_ratio: minimum fraction of window occupied by down postures
                        to consider the person as "in a squat".
            lost_s: seconds without tracked person before reset.
            min_confidence: minimum posture_confidence to accept a label.
        """
        self.window_s = float(window_s)
        self.down_ratio = float(down_ratio)
        self.lost_s = float(lost_s)
        self.min_confidence = float(min_confidence)
        self.reset()

    def reset(self):
        self.count = 0
        self.track_id = None
        # Sliding window: list of (timestamp, is_down) tuples
        self.window = deque()
        self.in_squat = False
        self.since_down = 0.0  # timestamp when down_ratio threshold was first met
        self.last_seen = None

    def _drop_track(self):
        self.track_id = None
        self.window.clear()
        self.in_squat = False
        self.since_down = 0.0
        self.last_seen = None

    def _prune_window(self, now):
        """Remove entries older than window_s."""
        cutoff = now - self.window_s
        while self.window and self.window[0][0] < cutoff:
            self.window.popleft()

    def _down_fraction(self):
        """Return fraction of window occupied by down postures."""
        if not self.window:
            return 0.0
        down_count = sum(1 for _, is_down in self.window if is_down)
        return down_count / len(self.window)

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
        is_down = posture in POSTURES_DOWN

        # Add observation to sliding window
        self._prune_window(now)
        self.window.append((now, is_down))

        # ── Trend-based state machine ───────────────────────────────────
        frac = self._down_fraction()

        if not self.in_squat:
            # Phase 1: waiting for sustained down posture
            if frac >= self.down_ratio and self.since_down == 0.0:
                self.since_down = now
            if self.since_down > 0.0:
                # Require minimum sustained down time (0.3s) to enter squat state
                if now - self.since_down >= 0.3:
                    self.in_squat = True
        else:
            # Phase 2: in squat, waiting for up posture to complete
            if posture in POSTURES_UP:
                self.count += 1
                self.in_squat = False
                self.since_down = 0.0
                self.window.clear()
                return {
                    "type": "squat_completed",
                    "priority": 1,
                    "track_id": self.track_id,
                    "count": self.count,
                }

        return None

    def _valid_posture(self, posture, confidence):
        if posture not in POSTURES_UP | POSTURES_DOWN:
            return False
        if not isinstance(confidence, (int, float)) or not math.isfinite(confidence):
            return False
        return confidence >= self.min_confidence
