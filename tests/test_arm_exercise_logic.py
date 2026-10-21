"""ROS-free frame sequences for the two upper-body exercise counters."""

from __future__ import annotations

from common.arm_exercise_logic import ArmExerciseCounter
from common.squat_logic import COCO_NAMES


INDEX = {name: index for index, name in enumerate(COCO_NAMES)}


def person(*, person_id=1, x_offset=0, visibility=0.95,
           left_visibility=None, curl=False, swing=False,
           left_raise=False, right_raise=False, mirrored=False):
    """Front-view synthetic COCO joints; anatomical left is image-right."""
    points = [[450 + x_offset, 200, visibility] for _ in COCO_NAMES]
    torso = {
        "left_shoulder": (490, 220), "right_shoulder": (410, 220),
        "left_hip": (475, 350), "right_hip": (425, 350),
    }
    arms = {
        "left_elbow": (492, 280), "left_wrist": (494, 345),
        "right_elbow": (408, 280), "right_wrist": (406, 345),
    }
    if curl:
        arms.update({"left_elbow": (492, 280), "left_wrist": (548, 250)})
    if swing:
        arms.update({"left_elbow": (550, 223), "left_wrist": (610, 226)})
    if left_raise:
        arms.update({"left_elbow": (540, 220), "left_wrist": (595, 220)})
    if right_raise:
        arms.update({"right_elbow": (360, 220), "right_wrist": (305, 220)})
    for name, (x, y) in {**torso, **arms}.items():
        if mirrored:
            x = 900 - x
        side_visibility = (left_visibility if name.startswith("left_") and
                           left_visibility is not None else visibility)
        points[INDEX[name]] = [x + x_offset, y, side_visibility]
    for name, coordinates in {
        "left_knee": (475, 455), "right_knee": (425, 455),
        "left_ankle": (475, 570), "right_ankle": (425, 570),
    }.items():
        x, y = coordinates
        points[INDEX[name]] = [(900 - x if mirrored else x) + x_offset, y,
                               visibility]
    xs = [point[0] for point in points]
    ys = [point[1] for point in points]
    return {"id": person_id, "score": visibility,
            "bbox": [min(xs) - 15, min(ys) - 15,
                     max(xs) + 15, max(ys) + 15],
            "keypoints": points}


class Sequence:
    def __init__(self, exercise, config=None):
        self.counter = ArmExerciseCounter(exercise, config)
        self.time = 0.0
        self.events = []

    def frame(self, *people, dt=0.1):
        self.time = round(self.time + dt, 3)
        events = self.counter.process({
            "timestamp": self.time, "image_size": [960, 640],
            "keypoint_names": COCO_NAMES, "persons": list(people),
        })
        self.events.extend(events)
        return events

    def hold(self, frames=7, **kwargs):
        for _ in range(frames):
            self.frame(person(**kwargs))

    def reps(self):
        return [event for event in self.events
                if event["type"] == "rep_completed"]


def test_curl_counts_complete_selected_arm_cycle_and_keeps_absolute_total():
    sequence = Sequence("dumbbell_curl")
    sequence.hold(left_visibility=0.99)
    assert sequence.counter.snapshot()["side"] == "left"
    assert sequence.counter.snapshot()["state"] == "ready"
    assert [event["type"] for event in sequence.events].count(
        "ready_to_exercise") == 1
    sequence.hold(5, left_visibility=0.99, curl=True)
    assert sequence.counter.snapshot()["state"] == "peak"
    assert not sequence.reps()
    sequence.hold(5, left_visibility=0.99)
    assert sequence.counter.snapshot()["count_total"] == 1
    (event,) = sequence.reps()
    assert event["count_total"] == 1
    assert event["target_id"] == 1
    assert event["session_id"] == sequence.counter.session_id
    assert event["rep_duration_s"] >= 0.55
    sequence.hold(5, left_visibility=0.99, curl=True)
    sequence.hold(5, left_visibility=0.99)
    assert [event["count_total"] for event in sequence.reps()] == [1, 2]


def test_curl_does_not_switch_to_other_arm_or_count_straight_arm_swing():
    sequence = Sequence("dumbbell_curl")
    sequence.hold(left_visibility=0.99)
    for _ in range(7):
        other = person(left_visibility=0.99)
        other["keypoints"][INDEX["right_wrist"]] = [355, 250, 0.95]
        sequence.frame(other)
    sequence.hold(5, left_visibility=0.99)
    sequence.hold(6, left_visibility=0.99, swing=True)
    sequence.hold(5, left_visibility=0.99)
    assert sequence.reps() == []
    assert sequence.counter.snapshot()["side"] == "left"
    assert sequence.counter.snapshot()["count_total"] == 0


def test_curl_frame_gap_cancels_partial_repetition_and_rearms():
    sequence = Sequence("dumbbell_curl")
    sequence.hold(left_visibility=0.99)
    sequence.hold(5, left_visibility=0.99, curl=True)
    sequence.frame(person(left_visibility=0.99), dt=0.8)
    sequence.hold(5, left_visibility=0.99)
    assert sequence.reps() == []
    assert sequence.counter.snapshot()["state"] == "ready"
    sequence.hold(5, left_visibility=0.99, curl=True)
    sequence.hold(5, left_visibility=0.99)
    assert sequence.counter.snapshot()["count_total"] == 1


def test_curl_missing_selected_arm_does_not_reacquire_via_other_arm():
    sequence = Sequence("dumbbell_curl")
    sequence.hold(left_visibility=0.99)
    sequence.hold(15, left_visibility=0.05)
    assert sequence.counter.snapshot()["target_state"] == "missing"
    assert sequence.counter.snapshot()["state"] == "paused"
    assert [event["type"] for event in sequence.events].count("target_lost") == 1
    assert not any(event["type"] == "target_reacquired"
                   for event in sequence.events)
    sequence.hold(6, left_visibility=0.99)
    assert [event["type"] for event in sequence.events].count(
        "target_reacquired") == 1
    assert sequence.counter.snapshot()["state"] == "ready"
    assert sequence.counter.snapshot()["count_total"] == 0


def test_lateral_raise_requires_both_arms_and_counts_after_full_return():
    sequence = Sequence("lateral_raise")
    sequence.hold()
    assert sequence.counter.snapshot()["state"] == "ready"
    sequence.hold(5, left_raise=True)
    sequence.hold(5)
    assert sequence.reps() == []
    sequence.hold(5, left_raise=True, right_raise=True)
    assert sequence.counter.snapshot()["state"] == "peak"
    sequence.hold(5)
    assert [event["count_total"] for event in sequence.reps()] == [1]
    checks = sequence.counter.diagnostics()["checks"]
    assert checks["rest"] is True
    assert checks["peak"] is False


def test_lateral_raise_recognizes_mirrored_camera_orientation():
    sequence = Sequence("lateral_raise")
    sequence.hold(mirrored=True)
    sequence.hold(5, mirrored=True, left_raise=True, right_raise=True)
    sequence.hold(5, mirrored=True)
    assert sequence.counter.snapshot()["count_total"] == 1
    assert len(sequence.reps()) == 1


def test_target_lock_prefers_clearest_person_and_switch_creates_new_session():
    sequence = Sequence("dumbbell_curl")
    for _ in range(7):
        sequence.frame(person(person_id=1, visibility=0.98),
                       person(person_id=2, x_offset=300, visibility=0.60))
    first_session = sequence.counter.session_id
    assert sequence.counter.target_id == 1
    for _ in range(5):
        sequence.frame(person(person_id=1, visibility=0.90),
                       person(person_id=2, x_offset=300, visibility=0.99))
    assert sequence.counter.target_id == 1
    for _ in range(13):
        sequence.frame(person(person_id=2, x_offset=300, visibility=0.99))
    assert sequence.counter.target_id == 2
    assert sequence.counter.session_id != first_session
    assert sequence.counter.count_total == 0
    assert any(event["type"] == "target_lost" for event in sequence.events)
    assert any(event["type"] == "target_changed" for event in sequence.events)


def test_diagnostics_expose_measurements_checks_and_thresholds():
    sequence = Sequence("lateral_raise")
    sequence.hold()
    report = sequence.counter.diagnostics()
    assert report["selected_target"]["id"] == 1
    assert report["measurements"]["left_outward_reach_ratio"] is not None
    assert report["checks"]["rest"] is True
    assert report["thresholds"]["lateral_peak_reach_min_ratio"] > 0
    assert report["reason"] == "waiting_for_movement"


def test_rejects_invalid_exercise_and_thresholds():
    import pytest

    with pytest.raises(ValueError, match="unsupported"):
        ArmExerciseCounter("push_up")
    with pytest.raises(ValueError, match="unknown"):
        ArmExerciseCounter("dumbbell_curl", {"foo": 1})
    with pytest.raises(ValueError, match="nonnegative"):
        ArmExerciseCounter("lateral_raise", {"min_visibility": float("nan")})
