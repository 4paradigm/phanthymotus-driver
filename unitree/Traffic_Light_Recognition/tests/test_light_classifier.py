"""Tests for the pure traffic-light color logic — no ROS, no YOLO."""

import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))  # unitree

from Traffic_Light_Recognition.light_classifier import (  # noqa: E402
    RED,
    YELLOW,
    GREEN,
    UNKNOWN,
    LightStateTracker,
    classify_from_scores,
    compute_color_scores,
)


# ── classify_from_scores ─────────────────────────────────────────────────────

def test_classify_red_when_red_dominates():
    scores = {"red": 0.30, "yellow": 0.01, "green": 0.01, "lit": 0.32}
    assert classify_from_scores(scores) == RED


def test_classify_yellow_when_yellow_dominates():
    scores = {"red": 0.01, "yellow": 0.20, "green": 0.01, "lit": 0.22}
    assert classify_from_scores(scores) == YELLOW


def test_classify_green_when_green_dominates():
    scores = {"red": 0.01, "yellow": 0.01, "green": 0.25, "lit": 0.27}
    assert classify_from_scores(scores) == GREEN


def test_classify_unknown_when_nothing_lit():
    scores = {"red": 0.0, "yellow": 0.0, "green": 0.0, "lit": 0.0}
    assert classify_from_scores(scores) == UNKNOWN


def test_classify_unknown_when_lit_below_minimum():
    scores = {"red": 0.004, "yellow": 0.0, "green": 0.0, "lit": 0.004}
    assert classify_from_scores(scores, min_lit_fraction=0.005) == UNKNOWN


def test_classify_unknown_when_no_clear_winner():
    # Red barely beats yellow: 0.20 < 0.15 * 1.5 -> rejected as ambiguous.
    scores = {"red": 0.20, "yellow": 0.15, "green": 0.0, "lit": 0.35}
    assert classify_from_scores(scores, dominance_ratio=1.5) == UNKNOWN


# ── compute_color_scores (needs cv2/numpy; skipped when unavailable) ─────────

def _solid_bgr(h, w, bgr):
    import numpy as np
    return np.full((h, w, 3), bgr, dtype=np.uint8)


def test_compute_color_scores_red():
    pytest.importorskip("cv2")
    scores = compute_color_scores(_solid_bgr(20, 20, (0, 0, 255)))
    assert scores["red"] > 0.9
    assert scores["yellow"] < 0.01
    assert scores["green"] < 0.01
    assert scores["lit"] > 0.9


def test_compute_color_scores_green():
    pytest.importorskip("cv2")
    scores = compute_color_scores(_solid_bgr(20, 20, (0, 255, 0)))
    assert scores["green"] > 0.9
    assert scores["red"] < 0.01


def test_compute_color_scores_yellow():
    pytest.importorskip("cv2")
    scores = compute_color_scores(_solid_bgr(20, 20, (0, 255, 255)))
    assert scores["yellow"] > 0.9
    assert scores["red"] < 0.05


def test_compute_color_scores_empty():
    pytest.importorskip("cv2")
    scores = compute_color_scores(None)
    assert scores["pixels"] == 0
    assert scores["lit"] == 0.0
    assert classify_from_scores(scores) == UNKNOWN


# ── LightStateTracker ────────────────────────────────────────────────────────

def test_requires_confirm_frames():
    tracker = LightStateTracker(confirm_frames=2, cooldown_s=3.0)
    assert tracker.update(RED, confidence=0.9, now_ms=0) is None
    event = tracker.update(RED, confidence=0.9, now_ms=100)
    assert event is not None
    assert event.color == RED
    assert event.confidence == pytest.approx(0.9)


def test_no_reannounce_same_color():
    tracker = LightStateTracker(confirm_frames=2, cooldown_s=3.0)
    assert tracker.update(RED, now_ms=0) is None
    assert tracker.update(RED, now_ms=100) is not None
    # Still red -> no new event.
    assert tracker.update(RED, now_ms=200) is None
    assert tracker.update(RED, now_ms=300) is None


def test_announce_on_color_change():
    tracker = LightStateTracker(confirm_frames=2, cooldown_s=3.0)
    tracker.update(RED, now_ms=0)
    assert tracker.update(RED, now_ms=100) is not None  # announces RED at t=100

    # Change to green; cooldown (3s) from the red announcement suppresses it.
    tracker.update(GREEN, now_ms=200)
    assert tracker.update(GREEN, now_ms=300) is None

    # Cooldown elapsed at t=3100 (>= 100 + 3000).
    event = tracker.update(GREEN, now_ms=3100)
    assert event is not None
    assert event.color == GREEN


def test_unknown_resets_streak():
    tracker = LightStateTracker(confirm_frames=2, cooldown_s=3.0)
    tracker.update(RED, now_ms=0)
    tracker.update(UNKNOWN, now_ms=100)  # dropout resets the streak
    assert tracker.update(RED, now_ms=200) is None
    assert tracker.update(RED, now_ms=300) is not None


def test_confirm_frames_must_be_positive():
    with pytest.raises(ValueError):
        LightStateTracker(confirm_frames=0)
