"""Squat counter based on knee joint Y-position (vertical height).

Uses COCO-17 keypoints: left_knee=11, right_knee=12.
Counts a repetition when tracked person's average knee height goes:
  down (depth reached) → back up (above threshold).

Advantages over posture classification:
- Continuous signal, less sensitive to discrete classification gaps
- Knee coords are reliably detected even in partial squat transitions
- No need for initial "standing" label to start tracking
"""

import math


# COCO-17 keypoint indices
_LEFT_HIP = 11
_LEFT_KNEE = 13
_RIGHT_HIP = 12
_RIGHT_KNEE = 14


class SquatCounter:
    def __init__(self, depth_cm=15.0, hold_s=0.15, lost_s=2.0,
                 min_confidence=0.3, min_visibility=2):
        """
        Args:
            depth_cm: minimum knee-drop in cm to count as a squat.
            hold_s: minimum time at bottom before rebound is accepted.
            lost_s: seconds without tracked person before reset.
            min_confidence: minimum keypoint confidence to be used.
            min_visibility: minimum number of visible knee keypoints required.
        """
        self.depth_cm = float(depth_cm)
        self.hold_s = float(hold_s)
        self.lost_s = float(lost_s)
        self.min_confidence = float(min_confidence)
        self.min_visibility = int(min_visibility)
        self.reset()

    def reset(self):
        self.count = 0
        self.track_id = None
        # Phase: "up" (near standing) → "down" (at bottom) → "up" (count++)
        self.phase = "up"
        self.bottom_since = 0.0
        self.bottom_knee_y = None
        self.ref_knee_y = None  # standing reference height
        self.last_seen = None

    def _drop_track(self):
        self.track_id = None
        self.phase = "up"
        self.bottom_since = 0.0
        self.bottom_knee_y = None
        self.ref_knee_y = None
        self.last_seen = None

    @staticmethod
    def _knee_y(person):
        """Return average Y of left/right knee, or None if not visible.

        Returns (avg_y, visible_count):
            avg_y: mean Y coordinate (larger = lower in image).
            visible_count: number of knee keypoints that passed confidence check.
        """
        keypoints = person.get("keypoints")
        if not isinstance(keypoints, list) or len(keypoints) < 17 * 3:
            return None, 0

        ys = []
        for idx in (_LEFT_KNEE, _RIGHT_KNEE):
            x = keypoints[idx * 3]
            y = keypoints[idx * 3 + 1]
            conf = keypoints[idx * 3 + 2]
            if (math.isfinite(x) and math.isfinite(y)
                    and isinstance(conf, (int, float))
                    and conf >= SquatCounter._MIN_CONF_DYN
                    and conf >= person.get("min_confidence", 0.3)):
                ys.append(y)

        if len(ys) < 1:
            return None, 0
        return sum(ys) / len(ys), len(ys)

    # Dynamic min confidence for keypoint checks (also used outside class)
    _MIN_CONF_DYN = 0.3

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

        # Acquire a new track from the first person with visible knees
        if self.track_id is None:
            person = next(
                (p for p in persons
                 if isinstance(p, dict)
                 and isinstance(p.get("id"), int)
                 and self._knee_y(p)[0] is not None),
                None,
            )
            if person is None:
                return None
            self.track_id = person["id"]

        if person is None:
            return None

        knee_y, vis = self._knee_y(person)
        if knee_y is None:
            return None

        self.last_seen = now

        # ── Height-based state machine ──────────────────────────────────
        # Y increases downward in image coords.  Smaller Y = higher (standing).
        # Knee drop = current_knee_y - ref_knee_y (positive = knees lower).

        if self.ref_knee_y is None:
            # First frame: establish standing reference.
            self.ref_knee_y = knee_y
            self.phase = "up"

        knee_drop_px = knee_y - self.ref_knee_y  # > 0 means knees went down

        if self.phase == "up":
            if knee_drop_px > 0.5:  # small hysteresis
                self.phase = "down"

        elif self.phase == "down":
            if knee_drop_px >= self.depth_cm:
                # Reached squat depth.
                if self.bottom_knee_y is None or knee_y > self.bottom_knee_y:
                    self.bottom_knee_y = knee_y
                if self.bottom_since == 0.0:
                    self.bottom_since = now
                elif now - self.bottom_since >= self.hold_s:
                    # Hold time met — start looking for rebound.
                    self.phase = "rebound"
            else:
                # Didn't reach depth — reset bottom timer so a prior
                # bottom_since from a previous cycle can't falsely trigger.
                self.bottom_since = 0.0
                self.bottom_knee_y = None

        elif self.phase == "rebound":
            if knee_drop_px < self.depth_cm * 0.3:
                # Knees rose back above 30% of depth → complete squat.
                self.count += 1
                self.ref_knee_y = knee_y  # update reference to new standing height
                self.bottom_since = 0.0
                self.bottom_knee_y = None
                self.phase = "up"
                return {
                    "type": "squat_completed",
                    "priority": 1,
                    "track_id": self.track_id,
                    "count": self.count,
                }
            else:
                # Didn't rise enough — stay in rebound, keep waiting.
                pass

        return None
