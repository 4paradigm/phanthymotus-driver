"""Traffic-light color classification — pure, ROS/YOLO-free, and unit-tested.

yolov8n localizes the traffic-light box (COCO class 9); it does not distinguish
red/yellow/green. This module reads the color from the box's pixels with HSV
analysis and turns per-frame labels into stable spoken announcements.

Color model
-----------
A lit lamp is bright and saturated. In OpenCV's HSV (hue 0-179, sat/val 0-255):

  red    : hue near 0° (also near 180° — the hue circle wraps), two ranges
  yellow : hue roughly 18-40
  green  : hue roughly 45-95

The dark lamp housing and the unlit lamps fail the saturation/value gate, so the
winning color is the dominant *lit* hue inside the box. A margin test rejects
mixed/reflective boxes that show no single clear color.

State machine
-------------
Single-frame labels flicker (reflection, occlusion, motion blur). A color must
hold for ``confirm_frames`` consecutive frames and must differ from the last
announced color before it produces a ``LightEvent``. ``cooldown_s`` then prevents
rapid red->green->red flapping from re-announcing faster than a human can hear.
"""

from __future__ import annotations

import time
from dataclasses import dataclass

RED = "red"
YELLOW = "yellow"
GREEN = "green"
UNKNOWN = "unknown"

# OpenCV HSV hue ranges (hue is 0-179). Red wraps the hue circle, hence two.
_RED_HUE_LO = (0, 12)
_RED_HUE_HI = (168, 180)
_YELLOW_HUE = (18, 40)
_GREEN_HUE = (45, 95)


def compute_color_scores(bgr_roi, *, saturation_min=90, value_min=100):
    """Score the lit-pixel fractions of a traffic-light ROI.

    Args:
        bgr_roi: a BGR numpy image (HxWx3, uint8) cropped to the traffic-light
            box. May be ``None`` or empty.

    Returns a plain JSON-friendly dict::

        {"red": 0.0, "yellow": 0.0, "green": 0.0, "lit": 0.0, "pixels": 0}

    where each color fraction is (lit pixels of that hue) / (all ROI pixels) and
    ``lit`` is (all saturated+bright pixels) / (all ROI pixels).
    """
    import cv2
    import numpy as np

    if bgr_roi is None or getattr(bgr_roi, "size", 0) == 0:
        return {"red": 0.0, "yellow": 0.0, "green": 0.0, "lit": 0.0, "pixels": 0}

    hsv = cv2.cvtColor(bgr_roi, cv2.COLOR_BGR2HSV)
    hue = hsv[:, :, 0]
    sat = hsv[:, :, 1]
    val = hsv[:, :, 2]

    lit = (sat >= saturation_min) & (val >= value_min)
    red = lit & (((hue >= _RED_HUE_LO[0]) & (hue <= _RED_HUE_LO[1]))
                 | ((hue >= _RED_HUE_HI[0]) & (hue <= _RED_HUE_HI[1])))
    yellow = lit & (hue >= _YELLOW_HUE[0]) & (hue <= _YELLOW_HUE[1])
    green = lit & (hue >= _GREEN_HUE[0]) & (hue <= _GREEN_HUE[1])

    total = int(hue.size)
    return {
        "red": float(np.count_nonzero(red)) / total,
        "yellow": float(np.count_nonzero(yellow)) / total,
        "green": float(np.count_nonzero(green)) / total,
        "lit": float(np.count_nonzero(lit)) / total,
        "pixels": total,
    }


def classify_from_scores(scores, *, min_lit_fraction=0.005, dominance_ratio=1.5):
    """Pick one color from :func:`compute_color_scores` output, or 'unknown'.

    Returns ``RED``/``YELLOW``/``GREEN``/``UNKNOWN``.
    """
    lit = float(scores.get("lit", 0.0))
    if lit < min_lit_fraction:
        return UNKNOWN

    candidates = [
        (RED, float(scores.get(RED, 0.0))),
        (YELLOW, float(scores.get(YELLOW, 0.0))),
        (GREEN, float(scores.get(GREEN, 0.0))),
    ]
    candidates.sort(key=lambda pair: pair[1], reverse=True)
    winner, winner_frac = candidates[0]
    _, runner_frac = candidates[1]

    if winner_frac <= 0.0:
        return UNKNOWN
    if runner_frac > 0.0 and winner_frac < runner_frac * dominance_ratio:
        return UNKNOWN
    return winner


@dataclass(frozen=True)
class LightEvent:
    color: str                                   # 'red' | 'yellow' | 'green'
    confidence: float                            # detector box confidence
    fraction: float                              # winning color lit-pixel fraction
    bbox: tuple[float, float, float, float]      # x1, y1, x2, y2
    timestamp_ms: int


class LightStateTracker:
    """Turn per-frame colors into debounced, de-duplicated announcements."""

    def __init__(self, *, confirm_frames=2, cooldown_s=3.0):
        if confirm_frames < 1:
            raise ValueError("confirm_frames must be at least 1")
        if cooldown_s < 0:
            raise ValueError("cooldown_s must be non-negative")
        self.confirm_frames = confirm_frames
        self.cooldown_s = cooldown_s
        self._streak_color = UNKNOWN
        self._streak = 0
        self._announced_color = UNKNOWN
        self._last_event_ms = float("-inf")

    @property
    def announced_color(self) -> str:
        """The most recently announced color, or 'unknown' if none yet."""
        return self._announced_color

    def update(self, color, *, confidence=0.0, fraction=0.0, bbox=None,
               now_ms=None) -> LightEvent | None:
        """Feed one classified color. Returns a ``LightEvent`` on a new stable color.

        ``now_ms`` is an injectable Unix-ms clock (tests control it fully);
        defaults to wall time.
        """
        now = time.time() * 1000.0 if now_ms is None else float(now_ms)

        if color == self._streak_color:
            self._streak += 1
        else:
            self._streak_color = color
            self._streak = 1

        if not color or color == UNKNOWN or self._streak < self.confirm_frames:
            return None
        if color == self._announced_color:
            return None
        if now - self._last_event_ms < self.cooldown_s * 1000.0:
            return None

        self._announced_color = color
        self._last_event_ms = now
        box = (tuple(float(v) for v in bbox) if bbox is not None
               else (0.0, 0.0, 0.0, 0.0))
        return LightEvent(
            color=color,
            confidence=float(confidence),
            fraction=float(fraction),
            bbox=box,
            timestamp_ms=int(now),
        )

    def reset(self) -> None:
        """Clear streak and cooldown state for a fresh session."""
        self._streak_color = UNKNOWN
        self._streak = 0
        self._announced_color = UNKNOWN
        self._last_event_ms = float("-inf")
