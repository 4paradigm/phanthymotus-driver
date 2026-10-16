"""Bicep curl counter based on wrist Y-position (vertical height).

Uses COCO-17 keypoints: left_shoulder=5, left_elbow=7, left_wrist=9,
                               right_shoulder=6, right_elbow=8, right_wrist=10.
Counts a repetition when tracked person's wrist goes:
  down (arm extended) → up (hand near shoulder) → back down.

Advantages:
- Continuous wrist-height signal, robust to partial curls
- No dependency on discrete posture labels
- Automatic left/right arm selection (uses arm with best keypoint visibility)
"""

import math


# COCO-17 keypoint indices
_LEFT_SHOULDER = 5
_LEFT_ELBOW = 7
_LEFT_WRIST = 9
_RIGHT_SHOULDER = 6
_RIGHT_ELBOW = 8
_RIGHT_WRIST = 10


class BicepCurlCounter:
    def __init__(self, lift_px=40.0, hold_s=0.15, lost_s=2.0,
                 min_confidence=0.3, min_visibility=2):
        """
        Args:
            lift_px: minimum wrist-rise in px to count as a curl.
            hold_s: minimum time at top before descent is accepted.
            lost_s: seconds without tracked person before reset.
            min_confidence: minimum keypoint confidence to be used.
            min_visibility: minimum number of visible wrist/shoulder keypoints.
        """
        self.lift_px = float(lift_px)
        self.hold_s = float(hold_s)
        self.lost_s = float(lost_s)
        self.min_confidence = float(min_confidence)
        self.min_visibility = int(min_visibility)
        self.reset()

    def reset(self):
        self.count = 0
        self.track_id = None
        # Phase: "down" (arm extended) → "up" (hand near shoulder) → "down" (count++)
        self.phase = "down"
        self.top_since = 0.0
        self.top_wrist_y = None
        self.ref_shoulder_y = None  # reference shoulder height
        self.ref_wrist_y = None     # wrist height when arm is extended
        self.last_seen = None
        self._best_arm = None       # 'left' or 'right'

    def _drop_track(self):
        self.track_id = None
        self.phase = "down"
        self.top_since = 0.0
        self.top_wrist_y = None
        self.ref_shoulder_y = None
        self.ref_wrist_y = None
        self.last_seen = None
        self._best_arm = None

    @staticmethod
    def _wrist_shoulder(person):
        """Return (wrist_y, shoulder_y, arm_side) or (None, None, None).

        Checks left and right arms; returns whichever has better keypoint
        visibility (both wrist and shoulder present with sufficient confidence).
        """
        keypoints = person.get("keypoints")
        if not isinstance(keypoints, list) or len(keypoints) < 11 * 3:
            return None, None, None

        best_score = -1
        result = (None, None, None)

        for side, sh_idx, wr_idx in [
            ("left", _LEFT_SHOULDER, _LEFT_WRIST),
            ("right", _RIGHT_SHOULDER, _RIGHT_WRIST),
        ]:
            sh_x = keypoints[sh_idx * 3]
            sh_y = keypoints[sh_idx * 3 + 1]
            sh_conf = keypoints[sh_idx * 3 + 2]
            wr_x = keypoints[wr_idx * 3]
            wr_y = keypoints[wr_idx * 3 + 1]
            wr_conf = keypoints[wr_idx * 3 + 2]

            if (math.isfinite(sh_x) and math.isfinite(sh_y)
                    and math.isfinite(wr_x) and math.isfinite(wr_y)
                    and isinstance(sh_conf, (int, float)) and sh_conf >= 0.3
                    and isinstance(wr_conf, (int, float)) and wr_conf >= 0.3):
                score = sh_conf + wr_conf
                if score > best_score:
                    best_score = score
                    result = (wr_y, sh_y, side)

        return result

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

        # Acquire a new track from the first person with visible arm keypoints
        if self.track_id is None:
            person = next(
                (p for p in persons
                 if isinstance(p, dict)
                 and isinstance(p.get("id"), int)
                 and self._wrist_shoulder(p)[0] is not None),
                None,
            )
            if person is None:
                return None
            self.track_id = person["id"]

        if person is None:
            return None

        wrist_y, shoulder_y, arm = self._wrist_shoulder(person)
        if wrist_y is None:
            return None

        self.last_seen = now
        if self._best_arm is None:
            self._best_arm = arm

        # ── Height-based state machine ──────────────────────────────────
        # Y increases downward in image coords.  Smaller Y = higher in scene.

        if self.ref_shoulder_y is None:
            # First frame: record standing/extended reference.
            self.ref_shoulder_y = shoulder_y
            self.ref_wrist_y = wrist_y
            self.phase = "down"

        wrist_rise_px = self.ref_wrist_y - wrist_y  # > 0 means wrist went UP

        if self.phase == "down":
            if wrist_rise_px > 5:  # small hysteresis to avoid noise
                self.phase = "up"

        elif self.phase == "up":
            if wrist_rise_px >= self.lift_px:
                # Reached curl height.
                if self.top_wrist_y is None or wrist_y < self.top_wrist_y:
                    self.top_wrist_y = wrist_y
                if self.top_since == 0.0:
                    self.top_since = now
                elif now - self.top_since >= self.hold_s:
                    # Hold time met — start looking for descent.
                    self.phase = "descent"
            else:
                # Didn't reach height yet — reset.
                self.top_since = 0.0
                self.top_wrist_y = None

        elif self.phase == "descent":
            if wrist_rise_px < self.lift_px * 0.3:
                # Wrist fell back below 70% of lift height → complete curl.
                self.count += 1
                self.ref_wrist_y = wrist_y  # update reference to new extended position
                self.top_since = 0.0
                self.top_wrist_y = None
                self.phase = "down"
                return {
                    "type": "curl_completed",
                    "priority": 1,
                    "track_id": self.track_id,
                    "count": self.count,
                    "arm": self._best_arm,
                }
            else:
                # Didn't descend enough — stay in descent, keep waiting.
                pass

        return None
