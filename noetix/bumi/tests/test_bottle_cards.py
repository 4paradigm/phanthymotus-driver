"""Tests for the three "grab the stool" cards.

Detection runs on synthetic bottle silhouettes so the shape rules can be checked
without a camera.  The chase helpers are pure functions; the ROS-facing plugin
constructors are not exercised here.
"""

import importlib.util
import math
import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))


def _load(name, filename):
    spec = importlib.util.spec_from_file_location(name, ROOT / filename)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture(scope="module")
def detect():
    return _load("bottle_detect_test", "BottleDetect.py")


@pytest.fixture(scope="module")
def chase():
    return _load("bottle_chase_test", "BottleChase.py")


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
    bgr = np.zeros((480, 640, 3), dtype=np.uint8)
    left = _bottle_scene(offset_x=-180)[0]
    right = _bottle_scene(offset_x=100)[0]
    bgr = cv2.bitwise_or(left, right)
    depth = np.full((480, 640), 2000, dtype=np.uint16)
    # The right-hand bottle is closer.
    depth[:, 320:] = 1000
    bottles = detect.detect_bottles(bgr, depth)
    assert len(bottles) >= 2
    nearest = detect.pick_nearest(bottles)
    assert nearest["distance_m"] == pytest.approx(1.0, abs=0.05)
    assert nearest["bearing_deg"] < 0


def test_no_depth_means_no_target(detect):
    bgr, _ = _bottle_scene()
    assert detect.detect_bottles(bgr, None) == []


# ── Chase helpers ────────────────────────────────────────────────────────────

def test_speed_tapers_inside_the_slow_zone(chase):
    assert chase.speed_for_distance(2.0, 0.55, 1.2, 0.72) == 0.72
    assert chase.speed_for_distance(1.0, 0.55, 1.2, 0.72) == pytest.approx(
        0.498, abs=0.01)
    assert chase.speed_for_distance(0.60, 0.55, 1.2, 0.72) == 0.45
    assert chase.speed_for_distance(0.55, 0.55, 1.2, 0.72) == 0.0


def test_steering_has_a_dead_band_and_a_stiction_floor(chase):
    assert chase.steer_for_bearing(0.0) == 0.0
    assert chase.steer_for_bearing(3.0) == 0.0
    assert chase.steer_for_bearing(10.0) == pytest.approx(0.40)
    assert chase.steer_for_bearing(40.0) == pytest.approx(0.50)
    assert chase.steer_for_bearing(-40.0) == pytest.approx(-0.50)


def test_forward_speed_turns_in_place_for_a_large_bearing(chase):
    assert chase.forward_for_bearing(0.72, 0.0) == 0.72
    assert chase.forward_for_bearing(0.72, 60.0) == 0.0
    assert chase.forward_for_bearing(0.72, 30.0) == pytest.approx(0.623, abs=0.01)


def test_parse_target_accepts_only_fresh_usable_targets(chase):
    good = {"state": "fresh",
            "bottle": {"distance_m": 1.5, "bearing_deg": -4.0}}
    assert chase.parse_target(good)["distance_m"] == 1.5
    assert chase.parse_target({"state": "no_target"}) is None
    assert chase.parse_target({"state": "fresh", "bottle": {"distance_m": 0.0,
                                                           "bearing_deg": 0}}) is None
    assert chase.parse_target({"state": "fresh", "bottle": {"distance_m": 2.0,
                                                           "bearing_deg": 999}}) is None
    assert chase.parse_target({}) is None


def test_claim_text_is_the_expected_prompt():
    module = _load("stool_claim_test", "StoolClaim.py")
    assert module.CLAIM_TEXT == "抢到板凳！"
