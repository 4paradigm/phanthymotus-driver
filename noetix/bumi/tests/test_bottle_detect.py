"""Tests for the standalone bottle-contour detection card.

The synthetic scenes exercise the silhouette rules without a camera; the
tracker and depth helpers are pure and run without ROS.
"""

import importlib.util
import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))


def _load():
    spec = importlib.util.spec_from_file_location(
        "bottle_detect_card", ROOT / "BottleDetect.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture(scope="module")
def detect():
    return _load()


def _bottle_scene(distance_mm=1000, offset_x=0):
    cv2 = pytest.importorskip("cv2")
    bgr = np.zeros((480, 640, 3), dtype=np.uint8)
    points = np.array([
        [280 + offset_x, 430], [280 + offset_x, 170],
        [305 + offset_x, 150], [305 + offset_x, 95],
        [300 + offset_x, 85], [340 + offset_x, 85],
        [340 + offset_x, 95], [335 + offset_x, 150],
        [360 + offset_x, 170], [360 + offset_x, 430],
    ], dtype=np.int32)
    cv2.fillPoly(bgr, [points], (200, 200, 200))
    depth = np.full((480, 640), distance_mm, dtype=np.uint16)
    return bgr, depth


def test_detects_a_bottle_silhouette(detect):
    bgr, depth = _bottle_scene()
    bottles = detect.detect_bottles(bgr, depth)
    assert bottles, "synthetic bottle was not detected"
    bottle = bottles[0]
    assert bottle["distance_m"] == pytest.approx(1.0, abs=0.05)
    assert abs(bottle["bearing_deg"]) < 2.0
    assert bottle["confidence"] >= detect._DEFAULT_MIN_CONFIDENCE
    assert 0.25 <= bottle["shape"]["neck_ratio"] <= 0.85


def test_rejects_a_plain_rectangle(detect):
    cv2 = pytest.importorskip("cv2")
    bgr = np.zeros((480, 640, 3), dtype=np.uint8)
    cv2.rectangle(bgr, (280, 100), (360, 430), (200, 200, 200), -1)
    depth = np.full((480, 640), 1000, dtype=np.uint16)
    assert detect.detect_bottles(bgr, depth) == []


def test_bearing_is_negative_right_of_centre(detect):
    bgr, depth = _bottle_scene(offset_x=120)
    bottles = detect.detect_bottles(bgr, depth)
    assert bottles
    assert bottles[0]["bearing_deg"] < -5.0


def test_nearest_bottle_wins(detect):
    cv2 = pytest.importorskip("cv2")
    left = _bottle_scene(offset_x=-180)[0]
    right = _bottle_scene(offset_x=100)[0]
    bgr = cv2.bitwise_or(left, right)
    depth = np.full((480, 640), 2000, dtype=np.uint16)
    depth[:, 320:] = 1000
    bottles = detect.detect_bottles(bgr, depth)
    assert len(bottles) >= 2
    nearest = detect.pick_nearest(bottles)
    assert nearest["distance_m"] == pytest.approx(1.0, abs=0.05)
    assert nearest["bearing_deg"] < 0


def test_no_depth_means_no_target(detect):
    bgr, _ = _bottle_scene()
    assert detect.detect_bottles(bgr, None) == []


def test_confidence_threshold_can_reject_everything(detect):
    bgr, depth = _bottle_scene()
    assert detect.detect_bottles(bgr, depth, min_confidence=1.0) == []


# ── Pure shape / depth helpers ───────────────────────────────────────────────

def test_symmetry_offset_accepts_a_centred_mask(detect):
    mask = np.zeros((480, 640), dtype=np.uint8)
    mask[100:200, 300:340] = 1
    # An even-width box has a half-pixel centroid bias; it is not asymmetry.
    assert detect._symmetry_offset(mask, (300, 100, 40, 100)) == pytest.approx(
        0.0, abs=0.05)


def test_symmetry_offset_rejects_a_lopsided_mask(detect):
    mask = np.zeros((480, 640), dtype=np.uint8)
    mask[100:200, 300:340] = 1
    mask[100:140, 340:380] = 1
    assert detect._symmetry_offset(mask, (300, 100, 80, 100)) > 0.20


def test_depth_mad_flags_a_mixed_background(detect):
    depth = np.full((480, 640), 1000, dtype=np.uint16)
    depth[:, 320:] = 2000
    stats = detect._depth_stats(depth, (280, 100, 80, 300))
    assert stats is not None
    _distance, mad = stats
    assert mad > 0.35


def test_plain_rectangle_mask_fails_the_neck_rule(detect):
    mask = np.zeros((480, 640), dtype=np.uint8)
    mask[80:430, 300:360] = 1
    area = float(mask.sum())
    assert detect._shape_metrics(mask, area, area * 1.02,
                                 (300, 80, 60, 350)) is None


# ── Tracker ──────────────────────────────────────────────────────────────────

def _candidate(bbox, distance=2.0, bearing=0.0):
    return {"bbox": bbox, "distance_m": distance, "bearing_deg": bearing,
            "confidence": 0.8}


def test_tracker_requires_consecutive_frames(detect):
    tracker = detect.BottleTracker(confirm_frames=2)
    candidate = _candidate([300, 100, 60, 300])
    assert tracker.update([candidate]) is None
    confirmed = tracker.update([candidate])
    assert confirmed is not None
    assert confirmed["confirmed_frames"] == 2


def test_tracker_resets_when_the_box_jumps(detect):
    tracker = detect.BottleTracker(confirm_frames=2)
    first = _candidate([300, 100, 60, 300])
    tracker.update([first])
    confirmed = tracker.update([first])
    assert confirmed is not None
    jumped = _candidate([50, 100, 60, 300])
    assert tracker.update([jumped]) is None
    assert tracker.progress()["hits"] == 1


def test_tracker_resets_without_a_target(detect):
    tracker = detect.BottleTracker(confirm_frames=2)
    tracker.update([_candidate([300, 100, 60, 300])])
    tracker.update([])
    assert tracker.hits == 0
    assert tracker.last is None
