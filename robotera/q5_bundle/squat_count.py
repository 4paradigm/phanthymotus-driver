"""Squat counter based on bounding box height detection.

Uses COCO-17 person bbox to detect squat by tracking vertical size changes.
Counts a repetition when tracked person's bbox height goes:
  down (shorter bbox during squat) → back up (taller bbox when standing).

Advantages over posture labels:
- Direct geometric measurement, no classification errors
- Robust to posture misclassification (standing vs upright vs crouching)
- Works even when posture tag is missing or unreliable
"""

import math


class SquatCounter:
    def __init__(self, height_drop_px=60.0, height_rise_px=40.0,
                 hold_s=0.15, lost_s=4.0, min_confidence=0.3):
        """
        Args:
            height_drop_px: bbox height drop in px to count as squat start.
            height_rise_px: bbox height rise in px from bottom to complete squat.
            hold_s: minimum time at bottom before rebound is accepted.
            lost_s: seconds without tracked person before reset.
            min_confidence: minimum bbox_confidence or posture_confidence.
        """
        self.height_drop_px = float(height_drop_px)
        self.height_rise_px = float(height_rise_px)
        self.hold_s = float(hold_s)
        self.lost_s = float(lost_s)
        self.min_confidence = float(min_confidence)
        self.reset()

    def reset(self):
        self.count = 0
        self.track_id = None
        # Phase: "up" (standing) → "down" (squatting) → "up" (count++)
        self.phase = "up"
        self.ref_height = None     # standing reference bbox height
        self.bottom_height = None  # minimum bbox height at squat bottom
        self.bottom_since = 0.0    # timestamp when bottom was reached
        self.last_seen = None

    def _drop_track(self):
        self.track_id = None
        self.phase = "up"
        self.ref_height = None
        self.bottom_height = None
        self.bottom_since = 0.0
        self.last_seen = None

    @staticmethod
    def _bbox_height(person):
        """Return bbox height (y2 - y1) or None if bbox not available."""
        bbox = person.get("bbox")
        if not isinstance(bbox, (list, tuple)) or len(bbox) < 4:
            return None
        y1 = bbox[1]
        y2 = bbox[3]
        if not (math.isfinite(y1) and math.isfinite(y2)):
            return None
        return y2 - y1

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

        # Acquire a new track from the first person with valid bbox
        if self.track_id is None:
            person = next(
                (p for p in persons
                 if isinstance(p, dict)
                 and isinstance(p.get("id"), int)
                 and self._bbox_height(p) is not None),
                None,
            )
            if person is None:
                return None
            self.track_id = person["id"]

        if person is None:
            return None

        bbox_height = self._bbox_height(person)
        if bbox_height is None:
            return None

        self.last_seen = now

        # ── Height-based state machine ──────────────────────────────────
        if self.ref_height is None:
            # First frame: establish standing reference height
            self.ref_height = bbox_height
            self.phase = "up"

        height_change = self.ref_height - bbox_height  # > 0 means bbox got shorter (squatting)

        if self.phase == "up":
            # Waiting for bbox to shorten (person squatting down)
            if height_change >= self.height_drop_px:
                self.phase = "down"

        elif self.phase == "down":
            if self.bottom_height is None or bbox_height <= self.bottom_height:
                self.bottom_height = bbox_height
            if self.bottom_since == 0.0:
                self.bottom_since = now
            elif now - self.bottom_since >= self.hold_s:
                # Hold time met at bottom — look for rebound (bbox growing taller)
                self.phase = "rebound"
            else:
                # Still going down or at bottom, keep tracking
                pass

        elif self.phase == "rebound":
            if height_change <= self.height_drop_px - self.height_rise_px:
                # Bbox has grown taller by rise_px from bottom → complete squat
                self.count += 1
                self.ref_height = bbox_height  # update reference to new standing height
                self.bottom_since = 0.0
                self.bottom_height = None
                self.phase = "up"
                return {
                    "type": "squat_completed",
                    "priority": 1,
                    "track_id": self.track_id,
                    "count": self.count,
                }
            else:
                # Didn't rise enough yet — stay in rebound
                pass

        return None
