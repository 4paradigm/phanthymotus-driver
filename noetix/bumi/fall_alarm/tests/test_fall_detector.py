"""Tests for the pure fall-detection logic — no ROS, no YOLO."""

import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))  # noetix/bumi

from fall_alarm.fall_detector import (  # noqa: E402
    FallDetector,
    LEFT_SHOULDER, RIGHT_SHOULDER,
    LEFT_HIP, RIGHT_HIP,
    LEFT_ANKLE, RIGHT_ANKLE,
    compute_features,
)


def _standing_person():
    """Tall, vertical person: shoulders above hips above ankles on one midline."""
    kp = [[0.0, 0.0, 0.0] for _ in range(17)]
    kp[LEFT_SHOULDER] = [115.0, 80.0, 0.9]
    kp[RIGHT_SHOULDER] = [135.0, 80.0, 0.9]
    kp[LEFT_HIP] = [118.0, 160.0, 0.9]
    kp[RIGHT_HIP] = [132.0, 160.0, 0.9]
    kp[LEFT_ANKLE] = [120.0, 280.0, 0.9]
    kp[RIGHT_ANKLE] = [130.0, 280.0, 0.9]
    return {"keypoints": kp, "bbox": [100.0, 50.0, 150.0, 300.0], "confidence": 0.9}


def _lying_person():
    """Person lying on their side: torso horizontal, hips near ankles."""
    kp = [[0.0, 0.0, 0.0] for _ in range(17)]
    kp[LEFT_SHOULDER] = [90.0, 230.0, 0.9]
    kp[RIGHT_SHOULDER] = [110.0, 230.0, 0.9]
    kp[LEFT_HIP] = [220.0, 235.0, 0.9]
    kp[RIGHT_HIP] = [240.0, 235.0, 0.9]
    kp[LEFT_ANKLE] = [280.0, 240.0, 0.9]
    kp[RIGHT_ANKLE] = [290.0, 240.0, 0.9]
    return {"keypoints": kp, "bbox": [50.0, 200.0, 300.0, 260.0], "confidence": 0.85}


def _flat_no_keypoints():
    """Very flat box with no confident keypoints (fully occluded)."""
    kp = [[0.0, 0.0, 0.0] for _ in range(17)]
    return {"keypoints": kp, "bbox": [50.0, 200.0, 300.0, 240.0], "confidence": 0.8}


def test_standing_is_not_down():
    det = FallDetector()
    feats = compute_features(_standing_person()["keypoints"], _standing_person()["bbox"])
    assert feats["aspect_ratio"] == pytest.approx(5.0)
    assert feats["torso_angle_deg"] == pytest.approx(0.0, abs=1.0)
    assert feats["hip_elevation_ratio"] == pytest.approx(0.48, abs=0.05)
    assert det.is_down(feats) is False


def test_lying_is_down():
    det = FallDetector()
    feats = compute_features(_lying_person()["keypoints"], _lying_person()["bbox"])
    assert feats["aspect_ratio"] == pytest.approx(0.24, abs=0.02)
    assert feats["torso_angle_deg"] == pytest.approx(90.0, abs=5.0)
    assert feats["hip_elevation_ratio"] == pytest.approx(0.08, abs=0.05)
    assert det.is_down(feats) is True


def test_flat_box_without_keypoints_is_down():
    det = FallDetector()
    feats = compute_features(_flat_no_keypoints()["keypoints"], _flat_no_keypoints()["bbox"])
    assert feats["torso_angle_deg"] is None
    assert feats["hip_elevation_ratio"] is None
    assert det.is_down(feats) is True


def test_requires_confirm_frames():
    det = FallDetector(confirm_frames=3)
    assert det.update(_lying_person(), now_ms=0) is None
    assert det.update(_lying_person(), now_ms=100) is None
    event = det.update(_lying_person(), now_ms=200)
    assert event is not None
    assert event.confidence == pytest.approx(0.85)


def test_cooldown_suppresses_then_allows_realert():
    det = FallDetector(confirm_frames=3, cooldown_s=5.0)
    # First fall at t=200.
    for t in (0, 100):
        assert det.update(_lying_person(), now_ms=t) is None
    assert det.update(_lying_person(), now_ms=200) is not None

    # Still lying, still within cooldown -> suppressed.
    assert det.update(_lying_person(), now_ms=1000) is None
    assert det.update(_lying_person(), now_ms=3000) is None

    # Cooldown elapsed while continuously down -> re-alert (persistent reminder).
    assert det.update(_lying_person(), now_ms=5200) is not None


def test_stand_up_then_fall_again_after_cooldown():
    det = FallDetector(confirm_frames=3, cooldown_s=5.0)
    for t in (0, 100, 200):
        det.update(_lying_person(), now_ms=t)
    # Stand up resets the streak.
    for t in (300, 400, 500):
        det.update(_standing_person(), now_ms=t)
    # Fall again within cooldown -> suppressed (first fall was at t=200).
    for t in (600, 700):
        assert det.update(_lying_person(), now_ms=t) is None
    assert det.update(_lying_person(), now_ms=800) is None
    # After cooldown -> announces.
    assert det.update(_lying_person(), now_ms=5200) is not None


def test_missing_keypoints_do_not_vote():
    # Only aspect ratio is available; body geometry is None. A moderately flat
    # box (ar=1.2) must not trigger when the fallback threshold is not met.
    kp = [[0.0, 0.0, 0.0] for _ in range(17)]
    person = {"keypoints": kp, "bbox": [100.0, 100.0, 200.0, 220.0], "confidence": 0.8}
    det = FallDetector(aspect_ratio_max=1.15)
    feats = compute_features(person["keypoints"], person["bbox"])
    assert det.is_down(feats) is False


def test_confirm_frames_must_be_positive():
    with pytest.raises(ValueError):
        FallDetector(confirm_frames=0)
