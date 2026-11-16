"""Pose-based fall detection — pure, ROS/YOLO-free, and unit-tested.

A fallen person differs from a standing one in three geometric ways, and each
of the three alone is ambiguous:

  1. the bounding box becomes wide and flat (a crouching person is also short);
  2. the torso (hip -> shoulder) tilts far from vertical (a bending person also
     tilts);
  3. the hips drop to the ankle / ground level (a sitting person also drops).

So a fall is declared only when the box is flat **and** at least one
body-geometry signal agrees, and that condition must hold for `confirm_frames`
consecutive frames. The cooldown then keeps a single continuous fall from
re-alerting every frame; while the subject stays down the card re-alerts once
per `cooldown_s`, and a new fall after standing back up announces immediately
(again subject to the cooldown).

Keypoint indices are COCO-17, which is exactly what YOLOv11-pose outputs.
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass

# COCO-17 keypoint indices (YOLOv11-pose output order).
NOSE = 0
LEFT_EYE, RIGHT_EYE = 1, 2
LEFT_EAR, RIGHT_EAR = 3, 4
LEFT_SHOULDER, RIGHT_SHOULDER = 5, 6
LEFT_ELBOW, RIGHT_ELBOW = 7, 8
LEFT_WRIST, RIGHT_WRIST = 9, 10
LEFT_HIP, RIGHT_HIP = 11, 12
LEFT_KNEE, RIGHT_KNEE = 13, 14
LEFT_ANKLE, RIGHT_ANKLE = 15, 16


@dataclass(frozen=True)
class FallEvent:
    confidence: float
    bbox: tuple[float, float, float, float]  # x1, y1, x2, y2
    aspect_ratio: float | None                # bbox height / width
    torso_angle_deg: float | None             # hip->shoulder vs vertical
    hip_elevation_ratio: float | None         # (ankle_y - hip_y) / bbox height
    timestamp_ms: int


def _midpoint(a, b):
    return ((a[0] + b[0]) / 2.0, (a[1] + b[1]) / 2.0)


def _torso_angle_deg(shoulder_mid, hip_mid):
    """Angle of the hip->shoulder vector from the image vertical, in degrees.

    The image y axis points down, so "vertical up" is (0, -1). A standing
    person has a near-vertical torso (angle ~0); a lying person near 90.
    """
    dx = shoulder_mid[0] - hip_mid[0]
    dy = shoulder_mid[1] - hip_mid[1]
    norm = math.hypot(dx, dy)
    if norm < 1e-6:
        return None
    cos = max(-1.0, min(1.0, -dy / norm))  # dot product with (0, -1)
    return math.degrees(math.acos(cos))


def compute_features(keypoints, bbox, min_keypoint_conf=0.5):
    """Compute the fall signals from one person's keypoints and box.

    Args:
        keypoints: 17 x [x, y, conf] (COCO-17).
        bbox: [x1, y1, x2, y2].
        min_keypoint_conf: joints below this are treated as missing.

    A keypoint-derived value is ``None`` when the joints it needs are missing
    or below ``min_keypoint_conf``, so one occluded joint degrades that signal
    instead of silently voting with garbage.
    """
    def kp(i):
        if keypoints is None or i >= len(keypoints):
            return None
        x, y, c = keypoints[i]
        return (float(x), float(y)) if c >= min_keypoint_conf else None

    x1, y1, x2, y2 = (float(v) for v in bbox)
    w = x2 - x1
    h = y2 - y1
    aspect_ratio = (h / w) if w > 0 else None

    ls, rs = kp(LEFT_SHOULDER), kp(RIGHT_SHOULDER)
    lh, rh = kp(LEFT_HIP), kp(RIGHT_HIP)
    la, ra = kp(LEFT_ANKLE), kp(RIGHT_ANKLE)

    torso_angle = None
    if ls and rs and lh and rh:
        torso_angle = _torso_angle_deg(_midpoint(ls, rs), _midpoint(lh, rh))

    hip_elevation = None
    if lh and rh and la and ra and h > 0:
        # How far the hips sit above the ankles, as a fraction of box height.
        # Standing: hips well above ankles -> ~0.5-0.7. Lying: ~0.
        hip_y = (lh[1] + rh[1]) / 2.0
        ankle_y = (la[1] + ra[1]) / 2.0
        hip_elevation = (ankle_y - hip_y) / h

    return {
        "aspect_ratio": aspect_ratio,
        "torso_angle_deg": torso_angle,
        "hip_elevation_ratio": hip_elevation,
    }


class FallDetector:
    """Stateful classifier: turn per-frame keypoints into (debounced) falls.

    The thresholds are the defaults that ship for a roughly eye-level Bumi
    camera; they are plain numbers to tune, not a model to train.
    """

    def __init__(self, *, min_keypoint_conf=0.5, aspect_ratio_max=1.15,
                 torso_angle_min_deg=45.0, hip_elevation_max=0.25,
                 confirm_frames=3, cooldown_s=8.0):
        if confirm_frames < 1:
            raise ValueError("confirm_frames must be at least 1")
        if cooldown_s < 0:
            raise ValueError("cooldown_s must be non-negative")
        self.min_keypoint_conf = min_keypoint_conf
        self.aspect_ratio_max = aspect_ratio_max
        self.torso_angle_min_deg = torso_angle_min_deg
        self.hip_elevation_max = hip_elevation_max
        self.confirm_frames = confirm_frames
        self.cooldown_s = cooldown_s
        self._down_streak = 0
        self._last_event_ms = float("-inf")

    def is_down(self, features) -> bool:
        ar = features["aspect_ratio"]
        angle = features["torso_angle_deg"]
        hip = features["hip_elevation_ratio"]

        flat = ar is not None and ar < self.aspect_ratio_max
        tilted = angle is not None and angle > self.torso_angle_min_deg
        grounded = hip is not None and hip < self.hip_elevation_max

        # A fall needs the box to be flat plus at least one body-geometry
        # agreement. When the body keypoints are all missing, the box alone may
        # still call it, but only if it is very flat.
        if tilted or grounded:
            return flat and (tilted or grounded)
        if angle is None and hip is None:
            return ar is not None and ar < self.aspect_ratio_max * 0.65
        return False

    def update(self, person, now_ms=None) -> FallEvent | None:
        """Feed one detected person. Returns a FallEvent on a confirmed fall.

        ``person`` is a dict with ``keypoints`` (17x[x,y,c]), ``bbox``
        ([x1,y1,x2,y2]) and optional ``confidence``. ``now_ms`` is an injectable
        Unix-ms clock (tests control it fully); defaults to wall time.
        """
        features = compute_features(
            person["keypoints"], person["bbox"], self.min_keypoint_conf)
        now = time.time() * 1000.0 if now_ms is None else float(now_ms)

        if self.is_down(features):
            self._down_streak += 1
        else:
            self._down_streak = 0

        if self._down_streak < self.confirm_frames:
            return None
        if now - self._last_event_ms < self.cooldown_s * 1000.0:
            return None

        self._last_event_ms = now
        return FallEvent(
            confidence=float(person.get("confidence", 0.0)),
            bbox=tuple(float(v) for v in person["bbox"]),
            aspect_ratio=features["aspect_ratio"],
            torso_angle_deg=features["torso_angle_deg"],
            hip_elevation_ratio=features["hip_elevation_ratio"],
            timestamp_ms=int(now),
        )

    def reset(self) -> None:
        """Clear streak and cooldown state for a fresh session."""
        self._down_streak = 0
        self._last_event_ms = float("-inf")
