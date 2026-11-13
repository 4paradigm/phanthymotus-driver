"""Association/history ownership regressions using real JPEG/colour/shape paths."""
from copy import deepcopy

import cv2
import numpy as np
import pytest

from common.traffic_light.processor import TrafficLightProcessor
from test_traffic_light import Clock, detection, jpeg, lamp


OLD = [[0, 0, 100, 100], [60, 0, 160, 100]]
DUPLICATE = [[10, 0, 110, 100], [10, 0, 110, 100]]


@pytest.fixture
def tracker():
    clock = Clock()
    processor = TrafficLightProcessor(clock=clock.now, ros_clock=clock.ns, wall_clock=clock.now)
    return processor, clock


def shared_scene():
    # One complete emitting shape fits inside both overlapping candidate ROIs.
    # This deliberately makes even a wrong association look visually plausible.
    image = np.zeros((100, 400, 3), np.uint8)
    cv2.circle(image, (80, 50), 12, (0, 255, 0), -1)
    cv2.circle(image, (270, 50), 18, (0, 0, 255), -1)
    return image


def frame(tracker, boxes, *, image=None, confidences=None):
    processor, clock = tracker
    payload = detection(clock, boxes, seq=clock.ns())
    if confidences is not None:
        assert len(confidences) == len(boxes)
        for obj, confidence in zip(payload["objects"], confidences):
            obj["confidence"] = confidence
    processor.image(jpeg(shared_scene() if image is None else image), payload["source_stamp"])
    result = processor.detection(payload)
    clock.advance()
    assert result["state"] == ("ok" if boxes else "no_signal")
    return result["signals"]


def stable_old_pair(tracker):
    for _ in range(3):
        signals = frame(tracker, OLD)
    assert [s["color"] for s in signals] == ["green", "green"]
    assert all(s["symbol"] == "circle" and s["stability"] == "stable"
               and s["observations"] == 3 for s in signals)
    return {s["signal_id"] for s in signals}


def assert_fresh(tracker, signals, retired):
    processor, _ = tracker
    assert retired.isdisjoint(processor.tracks)
    ids = {s["signal_id"] for s in signals}
    assert len(ids) == len(signals) and None not in ids and ids.isdisjoint(retired)
    for signal in signals:
        assert signal["observations"] == 1
        assert signal["color"] == signal["symbol"] == "unknown"
        assert signal["stability"] == signal["color_stability"] == signal["symbol_stability"] == "unstable"
        history = processor.tracks[signal["signal_id"]].history
        assert len(history) == 1
        assert all(b["stability"] == "unstable" for b in signal["bulbs"])
    histories = [processor.tracks[s["signal_id"]].history for s in signals]
    assert len({id(history) for history in histories}) == len(histories)
    assert len({id(history[0]) for history in histories}) == len(histories)


@pytest.mark.parametrize("count", [2, 3])
def test_clear_multiple_targets_keep_independent_ids_histories_colors_and_symbols(tracker, count):
    labels = [("red", "arrow_left"), ("green", "arrow_straight"), ("yellow", "arrow_right")][:count]
    image = np.hstack([lamp(color, symbol) for color, symbol in labels])
    boxes = [[i*160, 0, (i+1)*160, 160] for i in range(count)]
    first = frame(tracker, boxes, image=image)
    original = {tuple(s["bbox"]): s["signal_id"] for s in first}
    for step in range(1, 6):
        # Detection order changes cannot change ownership or suppress targets.
        current = boxes[::-1] if step % 2 else boxes
        signals = frame(tracker, current, image=image)
        assert len(signals) == count
        for signal in signals:
            assert signal["signal_id"] == original[tuple(signal["bbox"])]
            index = boxes.index(signal["bbox"])
            history = tracker[0].tracks[signal["signal_id"]].history
            assert signal["observations"] == len(history) == min(step+1, 5)
            assert {(h["color"], h["symbol"]) for h in history} == {labels[index]}
            if step >= 2:
                assert (signal["color"], signal["symbol"]) == labels[index]
                assert signal["stability"] == "stable"


@pytest.mark.parametrize("confidence", [.9, .31])
def test_identical_duplicate_boxes_cannot_consume_two_stable_tracks(tracker, confidence):
    retired = stable_old_pair(tracker)
    signals = frame(tracker, DUPLICATE, confidences=[.9, confidence])
    assert len(signals) == 2  # no dedupe or single-target workaround
    assert_fresh(tracker, signals, retired)
    assert [s["detection_confidence"] for s in signals] == [.9, confidence]


@pytest.mark.parametrize("reverse", [False, True])
def test_high_overlap_many_to_many_retires_both_old_histories(tracker, reverse):
    retired = stable_old_pair(tracker)
    boxes = [[10, 0, 110, 100], [15, 0, 115, 100]]
    assert_fresh(tracker, frame(tracker, boxes[::-1] if reverse else boxes), retired)


def test_one_detection_with_two_old_candidates_cannot_choose_the_largest_iou(tracker):
    retired = stable_old_pair(tracker)
    # IoUs .818 and .333: a large winning margin still cannot prove identity.
    assert_fresh(tracker, frame(tracker, DUPLICATE[:1]), retired)


def test_two_detections_claiming_one_track_both_start_without_its_history(tracker):
    for _ in range(3):
        old = frame(tracker, OLD[:1])
    assert old[0]["stability"] == "stable"
    signals = frame(tracker, [OLD[0], [5, 0, 105, 100]])
    assert_fresh(tracker, signals, {old[0]["signal_id"]})


def test_after_ambiguity_clear_frames_need_three_new_observations(tracker):
    retired = stable_old_pair(tracker)
    ambiguous = frame(tracker, DUPLICATE)
    assert_fresh(tracker, ambiguous, retired)
    duplicate_ids = {s["signal_id"] for s in ambiguous}
    recovered = frame(tracker, OLD)
    # Clear ROIs still compete for the duplicate tracks: retire those as well.
    assert_fresh(tracker, recovered, retired | duplicate_ids)
    new_ids = [s["signal_id"] for s in recovered]
    for observations in (2, 3):
        signals = frame(tracker, OLD)
        assert [s["signal_id"] for s in signals] == new_ids
        assert all(s["observations"] == observations for s in signals)
        assert all(s["stability"] == ("stable" if observations == 3 else "unstable") for s in signals)
        assert all(s["color"] == ("green" if observations == 3 else "unknown") for s in signals)
        assert all(s["symbol"] == ("circle" if observations == 3 else "unknown") for s in signals)
    assert retired.isdisjoint(tracker[0].tracks)


def test_repeated_duplicates_never_accumulate_stable_evidence(tracker):
    retired = stable_old_pair(tracker)
    for _ in range(5):
        signals = frame(tracker, DUPLICATE, confidences=[.9, .31])
        assert_fresh(tracker, signals, retired)
        retired |= {s["signal_id"] for s in signals}


def test_weak_duplicate_point29_is_unknown_and_does_not_steal_valid_history(tracker):
    for _ in range(3):
        old = frame(tracker, OLD[:1])
    history_before = deepcopy(list(tracker[0].tracks[old[0]["signal_id"]].history))
    valid, weak = frame(tracker, [OLD[0], OLD[0]], confidences=[.9, .29])
    assert valid["signal_id"] == old[0]["signal_id"] and valid["observations"] == 4
    assert valid["color"] == "green" and valid["symbol"] == "circle" and valid["stability"] == "stable"
    assert weak["signal_id"] is None and weak["observations"] == 0
    assert weak["color"] == weak["symbol"] == "unknown" and weak["stability"] == "unstable"
    assert weak["reason"] == "invalid_or_weak_detection"
    assert list(tracker[0].tracks[valid["signal_id"]].history)[:3] == history_before


def test_only_weak_frame_clears_history_without_counting_invalid_evidence(tracker):
    retired = stable_old_pair(tracker)
    signals = frame(tracker, DUPLICATE, confidences=[.29, .29])
    assert all(s["signal_id"] is None and s["observations"] == 0
               and s["color"] == s["symbol"] == "unknown" for s in signals)
    assert all(not track.history for track in tracker[0].tracks.values())
    recovered = frame(tracker, OLD)
    assert {s["signal_id"] for s in recovered} == retired
    assert all(s["observations"] == 1 and s["stability"] == "unstable" for s in recovered)


def test_ambiguity_retires_its_whole_connected_component_but_not_a_separate_target(tracker):
    # d0 -> A,B and d1 -> B,C. A/C each having one claimant is insufficient.
    boxes = OLD + [[120, 0, 220, 100], [220, 0, 320, 100]]
    image = shared_scene()
    cv2.circle(image, (180, 50), 12, (0, 255, 0), -1)
    for _ in range(3):
        old = frame(tracker, boxes, image=image)
    affected = {s["signal_id"] for s in old[:3]}
    unaffected = old[3]["signal_id"]
    signals = frame(tracker, [[10, 0, 110, 100], [70, 0, 170, 100], boxes[3]], image=image)
    assert_fresh(tracker, signals[:2], affected)
    assert signals[2]["signal_id"] == unaffected and signals[2]["observations"] == 4
    assert signals[2]["color"] == "red" and signals[2]["symbol"] == "circle"
    assert signals[2]["stability"] == "stable"


def test_visible_crossing_resets_before_targets_separate_in_swapped_order(tracker):
    retired = stable_old_pair(tracker)
    # Crossing/occlusion produces a many-to-one observation, then splits again.
    middle = frame(tracker, [[30, 0, 130, 100]])
    assert_fresh(tracker, middle, retired)
    signals = frame(tracker, OLD[::-1])
    assert all(s["signal_id"] not in retired and s["observations"] == 1
               and s["color"] == s["symbol"] == "unknown" for s in signals)
    assert all(s["stability"] == "unstable" for s in signals)


def test_instant_position_swap_with_changed_evidence_does_not_claim_stable_labels(tracker):
    # A teleport with no ambiguous geometry is unobservable to IoU. IDs denote
    # local spatial tracks, not physical identity. Current-label voting must
    # still refuse both changed colour AND shape instead of emitting old labels.
    boxes = [[0, 0, 160, 160], [160, 0, 320, 160]]
    first = np.hstack((lamp("red", "arrow_left"), lamp("green", "arrow_right")))
    for _ in range(3):
        old = frame(tracker, boxes, image=first)
    swapped = np.hstack((lamp("green", "arrow_right"), lamp("red", "arrow_left")))
    signals = frame(tracker, boxes, image=swapped)
    assert [s["signal_id"] for s in signals] == [s["signal_id"] for s in old]
    for signal in signals:
        assert signal["observations"] == 4 and signal["stability"] == "unstable"
        assert signal["color"] == signal["symbol"] == "unknown"
        assert signal["color_stability"] == signal["symbol_stability"] == "unstable"
        history = tracker[0].tracks[signal["signal_id"]].history
        assert history[-1]["color"] != history[-2]["color"]
        assert history[-1]["symbol"] != history[-2]["symbol"]


def test_large_scale_change_starts_new_history_and_does_not_widen_iou_gate(tracker):
    image = lamp("green")
    for _ in range(3):
        old = frame(tracker, [[0, 0, 160, 160]], image=image)
    # IoU=.25, below the unchanged .3 gate. Complete shape still fits the ROI.
    signals = frame(tracker, [[40, 40, 120, 120]], image=image)
    assert_fresh(tracker, signals, set())
    assert signals[0]["signal_id"] != old[0]["signal_id"]
    assert not tracker[0].tracks[old[0]["signal_id"]].history


def test_disappearance_and_ttl_expiry_keep_existing_reconfirmation_behavior(tracker):
    retired = stable_old_pair(tracker)
    frame(tracker, [])
    assert all(not track.history for track in tracker[0].tracks.values())
    reappeared = frame(tracker, OLD)
    assert {s["signal_id"] for s in reappeared} == retired
    assert all(s["observations"] == 1 and s["color"] == s["symbol"] == "unknown"
               and s["stability"] == "unstable" for s in reappeared)
    tracker[1].advance(1.1)
    expired = frame(tracker, OLD)
    assert_fresh(tracker, expired, retired)


def test_ambiguity_retires_before_allocating_under_the_existing_track_capacity(tracker):
    processor, clock = tracker
    processor = TrafficLightProcessor({"max_tracks": 2}, clock=clock.now,
                                     ros_clock=clock.ns, wall_clock=clock.now)
    tracker = processor, clock
    retired = stable_old_pair(tracker)
    signals = frame(tracker, DUPLICATE)
    assert_fresh(tracker, signals, retired)
    assert len(processor.tracks) == 2
