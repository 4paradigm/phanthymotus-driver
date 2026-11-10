"""Tests for the standalone human-shape detection card.

HOG full-body detection cannot be asserted on a synthetic silhouette, so these
tests cover the geometry/NMS/tracker and the empty-frame behaviour; real-person
detection is a hardware test.
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
        "person_detect_card", ROOT / "PersonDetect.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture(scope="module")
def detect():
    return _load()


# ── Geometry ─────────────────────────────────────────────────────────────────

def test_valid_box_requires_a_tall_human_aspect(detect):
    assert detect._valid_box((300, 100, 80, 240))
    assert not detect._valid_box((300, 100, 200, 240))   # too wide
    assert not detect._valid_box((300, 100, 80, 400))    # too tall/thin
    assert not detect._valid_box((300, 100, 80, 90))     # too short


def test_scale_rect_maps_downscaled_boxes_back(detect):
    rect = (80, 80, 64, 128)
    assert detect._scale_rect(rect, 4.0 / 3.0, 640, 480) == [60, 60, 48, 96]


def test_nms_keeps_the_highest_weight_of_an_overlap(detect):
    low = {"bbox": [300, 100, 80, 240], "weight": 0.5}
    high = {"bbox": [302, 102, 80, 240], "weight": 0.9}
    other = {"bbox": [50, 100, 80, 240], "weight": 0.7}
    kept = detect.nms([low, high, other])
    assert high in kept and other in kept and low not in kept


def test_nms_keeps_disjoint_boxes(detect):
    first = {"bbox": [50, 100, 80, 240], "weight": 0.5}
    second = {"bbox": [400, 100, 80, 240], "weight": 0.6}
    assert len(detect.nms([first, second])) == 2


def test_depth_mad_flags_a_mixed_background(detect):
    depth = np.full((480, 640), 3000, dtype=np.uint16)
    depth[:, 320:] = 5000
    stats = detect._depth_stats(depth, (280, 100, 80, 300))
    assert stats is not None
    _distance, mad = stats
    assert mad > 0.8


def test_empty_frame_has_no_people(detect):
    cv2 = pytest.importorskip("cv2")
    if not hasattr(cv2, "HOGDescriptor"):
        pytest.skip("this OpenCV build has no HOGDescriptor")
    bgr = np.zeros((480, 640, 3), dtype=np.uint8)
    depth = np.full((480, 640), 3000, dtype=np.uint16)
    assert detect.detect_people(bgr, depth) == []


def test_missing_depth_returns_nothing(detect):
    bgr = np.zeros((480, 640, 3), dtype=np.uint8)
    assert detect.detect_people(bgr, None) == []


def test_pick_nearest(detect):
    assert detect.pick_nearest([
        {"distance_m": 4.0}, {"distance_m": 2.5}])["distance_m"] == 2.5
    assert detect.pick_nearest([]) is None


# ── Tracker ──────────────────────────────────────────────────────────────────

def _person(bbox, distance=3.0, bearing=0.0, weight=1.0):
    return {"bbox": bbox, "distance_m": distance, "bearing_deg": bearing,
            "weight": weight, "confidence": 0.5}


def test_tracker_requires_consecutive_frames(detect):
    tracker = detect.PersonTracker(confirm_frames=2)
    person = _person([300, 100, 80, 240])
    assert tracker.update([person]) is None
    confirmed = tracker.update([person])
    assert confirmed is not None
    assert confirmed["confirmed_frames"] == 2


def test_tracker_resets_when_the_box_jumps(detect):
    tracker = detect.PersonTracker(confirm_frames=2)
    first = _person([300, 100, 80, 240])
    tracker.update([first])
    assert tracker.update([first]) is not None
    assert tracker.update([_person([50, 100, 80, 240])]) is None
    assert tracker.progress()["hits"] == 1


def test_tracker_resets_without_a_person(detect):
    tracker = detect.PersonTracker(confirm_frames=2)
    tracker.update([_person([300, 100, 80, 240])])
    tracker.update([])
    assert tracker.hits == 0
    assert tracker.last is None
