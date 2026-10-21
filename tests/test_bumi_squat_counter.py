"""Behavioral tests for Bumi's squat counter using pose's COCO-17 wire shape.

The clock is simulated: dwell, dropped frames, and target switching are tested
without a camera, ROS, or waiting for real time to pass.
"""

from __future__ import annotations

import importlib.util
import json
import sys
import types
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from common.squat_logic import SquatCounter  # noqa: E402


KEYPOINT_NAMES = [
    "nose", "left_eye", "right_eye", "left_ear", "right_ear",
    "left_shoulder", "right_shoulder", "left_elbow", "right_elbow",
    "left_wrist", "right_wrist", "left_hip", "right_hip",
    "left_knee", "right_knee", "left_ankle", "right_ankle",
]


def skeleton_payload(timestamp, people):
    return {
        "timestamp": round(timestamp, 3),
        "count": len(people),
        "image_size": [960, 640],
        "keypoint_names": KEYPOINT_NAMES,
        "persons": list(people),
    }


def person(track_id=1, depth=0.0, *, anchor_x=450, scale=1.0,
           visibility=0.95, bend=False, missing_legs=False):
    """Side/oblique view: depth=1 bends the knee and lowers the hip.

    The two legs are offset slightly so either visible side has consistent
    motion. A bend tips only the shoulders while the knees stay extended.
    """
    shoulder_x = -140 * depth + (145 if bend else 0)
    shoulder_y = -425 + 115 * depth + (115 if bend else 0)
    hip_x, hip_y = -95 * depth, -265 + 95 * depth
    knee_x, knee_y = 40 * depth, -135 + 30 * depth

    def point(x, y, confidence=visibility):
        return [round(anchor_x + scale * x, 3),
                round(600 + scale * y, 3), confidence]

    joints = [
        point(shoulder_x, shoulder_y - 45),
        point(shoulder_x - 8, shoulder_y - 50),
        point(shoulder_x + 8, shoulder_y - 50),
        point(shoulder_x - 16, shoulder_y - 45),
        point(shoulder_x + 16, shoulder_y - 45),
        point(shoulder_x - 8, shoulder_y),
        point(shoulder_x + 8, shoulder_y),
        point(shoulder_x - 32, shoulder_y + 65),
        point(shoulder_x + 32, shoulder_y + 65),
        point(shoulder_x - 36, shoulder_y + 125),
        point(shoulder_x + 36, shoulder_y + 125),
        point(hip_x - 8, hip_y), point(hip_x + 8, hip_y),
        point(knee_x - 8, knee_y), point(knee_x + 8, knee_y),
        point(-8, 0), point(8, 0),
    ]
    if missing_legs:
        for index in (13, 14, 15, 16):
            joints[index][2] = 0.0
    xs = [joint[0] for joint in joints]
    ys = [joint[1] for joint in joints]
    return {
        "id": track_id,
        "score": visibility,
        "bbox": [min(xs) - 20, min(ys) - 20, max(xs) + 20, max(ys) + 20],
        "posture": "crouching" if depth >= 0.9 else "standing" if depth == 0 else None,
        "activity": None,
        "keypoints": joints,
    }


class Sequence:
    def __init__(self, counter=None):
        self.counter = counter or SquatCounter()
        self.time = 0.0
        self.events = []

    def frame(self, *people, dt=0.1):
        self.time += dt
        payload = skeleton_payload(self.time, people)
        events = self.counter.process(payload)
        self.events.extend(events)
        return events

    def hold(self, depth, frames, **person_kwargs):
        for _ in range(frames):
            self.frame(person(depth=depth, **person_kwargs))

    def rep(self, **person_kwargs):
        for depth in (0.15, 0.3, 0.5, 0.7, 0.9, 1, 1, 1, 1,
                      0.8, 0.6, 0.4, 0.2, 0, 0, 0, 0, 0):
            self.frame(person(depth=depth, **person_kwargs))

    def reps(self):
        return [event for event in self.events if event["type"] == "rep_completed"]


def test_a_full_down_and_up_counts_once_and_reports_absolute_total():
    sequence = Sequence()
    sequence.hold(0, 27)
    sequence.rep()
    sequence.hold(0, 10)

    assert sequence.counter.snapshot()["count_total"] == 1
    (event,) = sequence.reps()
    assert event["count_total"] == 1
    assert event["target_id"] == 1
    assert event["session_id"] == sequence.counter.snapshot()["session_id"]
    assert event["event_time"] > 0

    sequence.rep()
    assert [event["count_total"] for event in sequence.reps()] == [1, 2]
    assert sequence.counter.snapshot()["count_total"] == 2
    assert sequence.reps()[1]["seq"] > sequence.reps()[0]["seq"]


def test_rep_duration_starts_with_descent_and_excludes_standing_wait():
    sequence = Sequence()
    sequence.hold(0, 127)
    descent_started_at = None
    for depth in (0.15, 0.3, 0.5, 0.7, 0.9, 1, 1, 1, 1,
                  0.8, 0.6, 0.4, 0.2, 0, 0, 0, 0, 0):
        sequence.frame(person(depth=depth))
        if descent_started_at is None and sequence.counter.snapshot()["state"] == "descent":
            descent_started_at = round(sequence.time, 3)

    assert descent_started_at is not None
    (rep,) = sequence.reps()
    assert rep["rep_duration_s"] == pytest.approx(
        rep["event_time"] - descent_started_at)
    assert 0.6 <= rep["rep_duration_s"] <= 1.8


def test_standing_calibration_accepts_irregular_pose_intervals():
    sequence = Sequence()
    for _ in range(7):
        for interval in (0.071, 0.093, 0.084, 0.102):
            sequence.frame(person(), dt=interval)

    assert sequence.counter.snapshot()["calibrated"]
    sequence.rep()
    assert sequence.counter.snapshot()["count_total"] == 1


def test_ready_event_follows_stable_calibration_once_per_baseline():
    sequence = Sequence()
    sequence.hold(0, 20)
    assert not [event for event in sequence.events
                if event["type"] == "ready_to_squat"]

    sequence.hold(0, 7)
    ready = [event for event in sequence.events
             if event["type"] == "ready_to_squat"]
    assert len(ready) == 1
    assert ready[0]["state"] == "top"
    assert ready[0]["calibrated"] is True
    assert ready[0]["count_total"] == 0
    assert ready[0]["session_id"] == sequence.counter.snapshot()["session_id"]

    sequence.hold(0, 20)
    assert len([event for event in sequence.events
                if event["type"] == "ready_to_squat"]) == 1

    sequence.counter.reset()
    sequence.hold(0, 27)
    ready = [event for event in sequence.events
             if event["type"] == "ready_to_squat"]
    assert len(ready) == 2
    assert ready[1]["session_id"] != ready[0]["session_id"]


def test_a_rep_is_not_announced_at_the_bottom_or_during_ascent():
    sequence = Sequence()
    sequence.hold(0, 27)
    for depth in (0.2, 0.4, 0.6, 0.8, 1, 1, 1, 1,
                  0.8, 0.6, 0.4, 0.2):
        sequence.frame(person(depth=depth))
    assert sequence.reps() == []
    assert sequence.counter.snapshot()["count_total"] == 0

    sequence.hold(0, 6)
    assert sequence.counter.snapshot()["count_total"] == 1
    assert len(sequence.reps()) == 1


def test_partial_squat_and_straight_leg_bend_do_not_count():
    sequence = Sequence()
    sequence.hold(0, 27)
    for depth in (0.1, 0.2, 0.25, 0.25, 0.2, 0.1, 0, 0, 0, 0):
        sequence.frame(person(depth=depth))
    for _ in range(8):
        sequence.frame(person(bend=True))
    sequence.hold(0, 7)

    assert sequence.counter.snapshot()["count_total"] == 0
    assert sequence.reps() == []


def test_bottom_bounce_and_top_noise_do_not_create_extra_reps():
    sequence = Sequence()
    sequence.hold(0, 27)
    for depth in (0.2, 0.4, 0.6, 0.8, 1, 0.96, 1, 0.93, 1,
                  0.8, 0.6, 0.4, 0.2, 0.03, 0.08, 0, 0.04, 0, 0, 0, 0):
        sequence.frame(person(depth=depth))
    sequence.hold(0, 10)

    assert sequence.counter.snapshot()["count_total"] == 1
    assert len(sequence.reps()) == 1


def test_visible_primary_person_is_locked_while_another_person_squats():
    sequence = Sequence()

    def frame(main_depth, other_depth):
        # Alternate list order: ordering by detector confidence is incidental.
        people = [person(1, main_depth),
                  person(2, other_depth, anchor_x=730, scale=0.65,
                         visibility=0.75)]
        if int(sequence.time * 10) % 2:
            people.reverse()
        sequence.frame(*people)

    for _ in range(27):
        frame(0, 0)
    for depth in (0.2, 0.4, 0.6, 0.8, 1, 1, 1, 1,
                  0.8, 0.6, 0.4, 0.2, 0, 0, 0, 0, 0):
        frame(0, depth)
    assert sequence.counter.snapshot()["count_total"] == 0
    assert sequence.counter.snapshot()["target_id"] == 1

    for depth in (0.2, 0.4, 0.6, 0.8, 1, 1, 1, 1,
                  0.8, 0.6, 0.4, 0.2, 0, 0, 0, 0, 0):
        frame(depth, 0)
    assert sequence.counter.snapshot()["count_total"] == 1
    assert [event["target_id"] for event in sequence.reps()] == [1]


def test_sustained_target_switch_starts_a_new_session_without_mixing_counts():
    sequence = Sequence()
    sequence.hold(0, 27)
    sequence.rep()
    previous_session = sequence.counter.snapshot()["session_id"]

    for _ in range(38):
        sequence.frame(person(2, anchor_x=730, scale=0.9))
    status = sequence.counter.snapshot()
    assert status["target_id"] == 2
    assert status["session_id"] != previous_session
    assert status["count_total"] == 0
    changes = [event for event in sequence.events if event["type"] == "target_changed"]
    assert len(changes) == 1
    assert changes[0]["previous_session_id"] == previous_session
    ready_sessions = [event["session_id"] for event in sequence.events
                      if event["type"] == "ready_to_squat"]
    assert ready_sessions == [previous_session, status["session_id"]]

    sequence.rep(track_id=2, anchor_x=730, scale=0.9)
    assert sequence.counter.snapshot()["count_total"] == 1
    assert [(event["session_id"], event["count_total"])
            for event in sequence.reps()] == [(previous_session, 1),
                                               (status["session_id"], 1)]


@pytest.mark.parametrize("start_x,return_x", [(450, 450), (180, 780)])
def test_same_single_person_returns_after_long_loss_without_reset(start_x, return_x):
    sequence = Sequence()
    sequence.hold(0, 27, anchor_x=start_x)
    sequence.rep(anchor_x=start_x)
    old_session = sequence.counter.snapshot()["session_id"]

    for _ in range(20):
        sequence.frame()
    assert sequence.counter.snapshot()["target_state"] == "missing"

    sequence.hold(0, 27, anchor_x=return_x)
    status = sequence.counter.snapshot()
    assert status["session_id"] == old_session
    assert status["count_total"] == 1
    assert status["target_state"] == "tracking"
    assert [event["type"] for event in sequence.events].count("target_reacquired") == 1
    assert not [event for event in sequence.events
                if event["type"] == "target_changed"]

    sequence.rep(anchor_x=return_x)
    assert sequence.counter.snapshot()["count_total"] == 2


def test_lost_leg_keypoints_or_frame_gap_cannot_finish_an_old_half_rep():
    sequence = Sequence()
    sequence.hold(0, 27)
    for depth in (0.2, 0.4, 0.7, 1, 1, 1, 1):
        sequence.frame(person(depth=depth))
    for _ in range(8):
        sequence.frame(person(depth=1, missing_legs=True))
    sequence.hold(0, 8)
    assert sequence.reps() == []

    sequence.hold(0, 27)
    for depth in (0.2, 0.4, 0.7, 1, 1, 1, 1):
        sequence.frame(person(depth=depth))
    sequence.frame(person(depth=0), dt=0.9)
    gap_debug = sequence.counter.diagnostics()
    assert gap_debug["frame_gap_exceeded"] is True
    assert gap_debug["frame_gap_s"] > gap_debug["thresholds"]["max_frame_gap_s"]
    sequence.hold(0, 8)
    assert sequence.reps() == []

    sequence.hold(0, 27)
    sequence.rep()
    assert sequence.counter.snapshot()["count_total"] == 1
    assert len(sequence.reps()) == 1


def test_one_bad_leg_frame_at_bottom_must_rearm_at_standing():
    sequence = Sequence()
    sequence.hold(0, 27)
    for depth in (0.2, 0.4, 0.7, 1, 1, 1, 1):
        sequence.frame(person(depth=depth))
    sequence.frame(person(depth=1, missing_legs=True))
    for depth in (1, 1, 0.8, 0.6, 0.4, 0.2, 0, 0, 0, 0, 0):
        sequence.frame(person(depth=depth))
    assert sequence.reps() == []

    sequence.rep()
    assert [event["count_total"] for event in sequence.reps()] == [1]


def test_degenerate_joint_frame_recovers_without_reset():
    sequence = Sequence()
    sequence.hold(0, 27)
    session = sequence.counter.snapshot()["session_id"]
    bad = person()
    for hip_index, knee_index in ((11, 13), (12, 14)):
        bad["keypoints"][knee_index][:2] = bad["keypoints"][hip_index][:2]
    sequence.frame(bad)
    assert sequence.counter.diagnostics()["reason"] == "target_joint_unreadable"
    assert sequence.counter.snapshot()["state"] == "rearming"

    sequence.hold(0, 5)
    sequence.rep()
    assert sequence.counter.snapshot()["session_id"] == session
    assert sequence.counter.snapshot()["count_total"] == 1


def test_stable_new_standing_position_reanchors_without_clearing_count():
    sequence = Sequence()
    sequence.hold(0, 27)
    sequence.rep()
    session = sequence.counter.snapshot()["session_id"]

    sequence.hold(0, 16, anchor_x=560)
    assert sequence.counter.snapshot()["session_id"] == session
    assert sequence.counter.snapshot()["count_total"] == 1
    assert sequence.counter.snapshot()["state"] == "top"
    assert [event.get("reanchored") for event in sequence.events
            if event["type"] == "ready_to_squat"] == [None, True]

    sequence.rep(anchor_x=560)
    assert sequence.counter.snapshot()["count_total"] == 2


def test_initial_shallow_stance_is_replaced_after_a_full_stand():
    sequence = Sequence()
    sequence.hold(0.2, 27)
    old_baseline = sequence.counter.diagnostics()["measurements"]["baseline"]
    session = sequence.counter.snapshot()["session_id"]

    sequence.hold(0, 12)
    new_baseline = sequence.counter.diagnostics()["measurements"]["baseline"]
    assert sequence.counter.snapshot()["session_id"] == session
    assert sequence.counter.snapshot()["count_total"] == 0
    assert new_baseline["knee_angle_deg"] > old_baseline["knee_angle_deg"]
    assert new_baseline["hip_y_px"] < old_baseline["hip_y_px"]

    sequence.rep()
    assert sequence.counter.snapshot()["count_total"] == 1


def test_reanchor_waits_for_standing_and_discards_shifted_partial():
    sequence = Sequence()
    sequence.hold(0, 27)
    sequence.rep()

    sequence.hold(1, 12, anchor_x=560)
    assert sequence.counter.snapshot()["count_total"] == 1
    assert len([event for event in sequence.events
                if event["type"] == "ready_to_squat"]) == 1

    sequence.hold(0, 16, anchor_x=560)
    assert sequence.counter.snapshot()["count_total"] == 1
    sequence.rep(anchor_x=560)
    assert sequence.counter.snapshot()["count_total"] == 2


def test_baseline_moves_only_after_a_completed_rep():
    sequence = Sequence()
    sequence.hold(0, 27)
    original = sequence.counter.diagnostics()["measurements"]["baseline"]
    original_ankle_x = original["ankle_x_px"]

    for depth in (0.2, 0.4, 0.7, 1, 1, 1, 1):
        sequence.frame(person(depth=depth, anchor_x=460))
    assert sequence.counter.diagnostics()["measurements"]["baseline"] == original

    for depth in (0.8, 0.6, 0.4, 0.2, 0, 0, 0, 0, 0):
        sequence.frame(person(depth=depth, anchor_x=460))
    assert sequence.counter.snapshot()["count_total"] == 1
    assert (sequence.counter.diagnostics()["measurements"]["baseline"]["hip_y_px"]
            == original["hip_y_px"])
    assert (sequence.counter.diagnostics()["measurements"]["baseline"]["ankle_x_px"]
            > original_ankle_x)


def test_short_occlusion_keeps_session_but_discards_interrupted_half_rep():
    sequence = Sequence()
    sequence.hold(0, 27)
    session = sequence.counter.snapshot()["session_id"]
    for depth in (0.2, 0.4, 0.7, 1, 1, 1, 1):
        sequence.frame(person(depth=depth))
    sequence.frame()
    sequence.frame()
    for depth in (1, 1, 0.8, 0.6, 0.4, 0.2, 0, 0, 0, 0, 0):
        sequence.frame(person(depth=depth))
    assert sequence.reps() == []
    assert sequence.counter.snapshot()["session_id"] == session

    sequence.rep()
    assert sequence.counter.snapshot()["session_id"] == session
    assert [event["count_total"] for event in sequence.reps()] == [1]


def test_reset_starts_fresh_without_replaying_old_repetitions():
    sequence = Sequence()
    sequence.hold(0, 27)
    sequence.rep()
    previous_session = sequence.counter.snapshot()["session_id"]

    status = sequence.counter.reset()
    assert status["count_total"] == 0
    assert status["session_id"] != previous_session
    reset_session = status["session_id"]
    sequence.hold(0, 27)
    started = [event for event in sequence.events
               if event["type"] == "session_started" and event["session_id"] != previous_session]
    assert started[-1]["session_id"] == reset_session
    assert sequence.counter.snapshot()["session_id"] == reset_session
    sequence.rep()
    assert sequence.counter.snapshot()["count_total"] == 1
    assert sequence.reps()[-1]["session_id"] == reset_session


def test_diagnostics_explain_standing_bottom_and_ascent_decisions():
    sequence = Sequence()
    sequence.hold(0, 27)
    standing = sequence.counter.diagnostics()
    assert standing["state"] == "top"
    assert standing["selected_target"]["id"] == 1
    assert standing["selected_target"]["readable"] is True
    assert standing["measurements"]["baseline"] is not None
    assert (standing["measurements"]["baseline_knee_angle_deg"]
            == standing["measurements"]["baseline"]["knee_angle_deg"])
    assert standing["measurements"]["knee_angle_deg"] > 140
    assert standing["checks"]["top"] is True
    assert standing["checks"]["bottom"] is False
    assert standing["thresholds"]["bottom_knee_drop_deg"] == 40.0

    for depth in (0.2, 0.4, 0.6, 0.8, 1, 1, 1, 1):
        sequence.frame(person(depth=depth))
    bottom = sequence.counter.diagnostics()
    assert bottom["state"] == "bottom"
    assert bottom["measurements"]["knee_drop_deg"] >= 40
    assert bottom["measurements"]["hip_drop_deg"] >= 25
    assert bottom["measurements"]["hip_descent_ratio"] >= 0.10
    assert bottom["checks"]["bottom"] is True
    assert bottom["checks"]["top"] is False
    assert sequence.counter.snapshot()["count_total"] == 0

    for depth in (0.7, 0.5, 0.3, 0.2, 0.1):
        sequence.frame(person(depth=depth))
    ascent = sequence.counter.diagnostics()
    assert ascent["state"] == "ascent"
    assert ascent["checks"]["bottom"] is False
    assert ascent["checks"]["top"] is False
    assert sequence.counter.snapshot()["count_total"] == 0

    sequence.hold(0, 7)
    assert sequence.counter.snapshot()["count_total"] == 1
    assert sequence.counter.diagnostics()["checks"]["top"] is True


def test_diagnostics_show_locked_target_and_reason_for_loss():
    sequence = Sequence()
    for _ in range(27):
        sequence.frame(person(1), person(2, anchor_x=730, scale=0.65,
                                         visibility=0.75))
    tracking = sequence.counter.diagnostics()
    assert tracking["candidate_count"] == 2
    assert tracking["selected_target"]["id"] == 1
    assert tracking["selected_target"]["present"] is True
    assert len(tracking["candidates"]) == 2
    assert all("keypoints" not in item for item in tracking["candidates"])

    sequence.frame(person(1, missing_legs=True))
    bad_pose = sequence.counter.diagnostics()
    assert bad_pose["state"] == "paused"
    assert bad_pose["selected_target"]["present"] is True
    assert bad_pose["selected_target"]["readable"] is False
    assert bad_pose["measurements"]["knee_angle_deg"] is None
    assert bad_pose["checks"]["bottom"] is None
    assert bad_pose["reason"] == "target_unreadable"

    for _ in range(6):
        sequence.frame()
    lost = sequence.counter.diagnostics()
    assert lost["candidate_count"] == 0
    assert lost["target_state"] == "missing"
    assert lost["selected_target"]["present"] is False
    assert lost["reason"] == "target_missing"


# The card itself must accept the pose wire format and publish a sparse event;
# these stubs exercise its ROS callback without an installed ROS distribution.
class _String:
    def __init__(self, data=""):
        self.data = data


class _Clock:
    def __init__(self):
        self.now = 100.0

    def advance(self, seconds):
        self.now += seconds


class _Publisher:
    def __init__(self, topic):
        self.topic = topic
        self.payloads = []

    def publish(self, message):
        self.payloads.append(json.loads(message.data))


class _Subscription:
    def __init__(self, topic, callback):
        self.topic = topic
        self.callback = callback
        self.destroyed = False


class _Node:
    def __init__(self, name):
        self.name = name
        self.publishers = {}
        self.subscriptions = []
        self.timers = []

    def create_publisher(self, message_type, topic, qos):
        publisher = _Publisher(topic)
        self.publishers[topic] = publisher
        return publisher

    def create_subscription(self, message_type, topic, callback, qos):
        subscription = _Subscription(topic, callback)
        self.subscriptions.append(subscription)
        return subscription

    def destroy_subscription(self, subscription):
        subscription.destroyed = True
        return True

    def create_timer(self, interval, callback):
        self.timers.append(callback)

    def get_logger(self):
        return types.SimpleNamespace(warn=lambda message: None,
                                     error=lambda message: None)


@pytest.fixture
def plugin_factory(monkeypatch):
    rclpy = types.ModuleType("rclpy")
    node = types.ModuleType("rclpy.node")
    qos = types.ModuleType("rclpy.qos")
    std_msgs = types.ModuleType("std_msgs")
    std_msgs_msg = types.ModuleType("std_msgs.msg")
    node.Node = _Node
    qos.QoSProfile = lambda **kwargs: kwargs
    qos.ReliabilityPolicy = types.SimpleNamespace(BEST_EFFORT=1, RELIABLE=2)
    qos.HistoryPolicy = types.SimpleNamespace(KEEP_LAST=1)
    qos.DurabilityPolicy = types.SimpleNamespace(VOLATILE=1)
    std_msgs_msg.String = _String
    for name, stub in (
        ("rclpy", rclpy), ("rclpy.node", node), ("rclpy.qos", qos),
        ("std_msgs", std_msgs), ("std_msgs.msg", std_msgs_msg),
        ("squat_logic", sys.modules[SquatCounter.__module__]),
    ):
        monkeypatch.setitem(sys.modules, name, stub)

    spec = importlib.util.spec_from_file_location(
        "bumi_squat_counter_plugin_test", ROOT / "noetix/bumi/squat_counter.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    def build(config=None):
        clock = _Clock()
        card_module = sys.modules[module.SquatCounterPlugin.__module__]
        monkeypatch.setattr(card_module, "time", types.SimpleNamespace(
            monotonic=lambda: clock.now))
        executor = types.SimpleNamespace(add_node=lambda node: None)
        card = module.SquatCounterPlugin(config or {}, "test_bumi", executor)
        return card, clock, card._node

    return build


def _send_pose(subscription, clock, *people, dt=0.1):
    clock.advance(dt)
    subscription.callback(_String(json.dumps(skeleton_payload(clock.now, people))))


def _arm_person(*, curl=False, lateral_raise=False):
    joints = [[450, 200, 0.95] for _ in KEYPOINT_NAMES]
    positions = {
        "left_shoulder": (490, 220), "right_shoulder": (410, 220),
        "left_hip": (475, 350), "right_hip": (425, 350),
        "left_elbow": (492, 280), "left_wrist": (494, 345),
        "right_elbow": (408, 280), "right_wrist": (406, 345),
    }
    if curl:
        positions["left_wrist"] = (548, 250)
    if lateral_raise:
        positions.update({
            "left_elbow": (540, 220), "left_wrist": (595, 220),
            "right_elbow": (360, 220), "right_wrist": (305, 220),
        })
    for name, (x, y) in positions.items():
        joints[KEYPOINT_NAMES.index(name)] = [x, y, 0.95]
    return {"id": 1, "score": 0.95, "bbox": [280, 170, 620, 580],
            "keypoints": joints}


def _plugin_standing(subscription, clock, frames=27):
    for _ in range(frames):
        _send_pose(subscription, clock, person())


def _plugin_rep(subscription, clock):
    for depth in (0.15, 0.3, 0.5, 0.7, 0.9, 1, 1, 1, 1,
                  0.8, 0.6, 0.4, 0.2, 0, 0, 0, 0, 0):
        _send_pose(subscription, clock, person(depth=depth))


def _published_reps(node, event_topic):
    return [payload for payload in node.publishers[event_topic].payloads
            if payload["type"] == "rep_completed"]


def test_plugin_consumes_skeleton_and_publishes_priority_absolute_count(plugin_factory):
    card, clock, node = plugin_factory()
    tool = card.get_tool()
    event_topic = tool["topic_out"][0]["topic"]
    assert tool["topic_in"][0]["format"] == "sensor/pose2d"

    started = card.dispatch("start", {"input_topic": "/camera/poses/skeleton"})
    assert started["state"] == "running"
    assert node.subscriptions[-1].topic == "/camera/poses/skeleton"
    subscription = node.subscriptions[-1]
    _plugin_standing(subscription, clock)
    _plugin_rep(subscription, clock)

    (ready,) = [event for event in node.publishers[event_topic].payloads
                if event["type"] == "ready_to_squat"]
    assert ready["priority"] == 1
    assert ready["calibrated"] is True
    assert ready["session_id"] == card.dispatch("status", {})["session_id"]
    (rep,) = _published_reps(node, event_topic)
    assert rep["priority"] == 1
    assert rep["source"] == "squat_counter"
    assert rep["exercise"] == "squat"
    assert rep["count_total"] == 1
    assert rep["session_id"] == card.dispatch("status", {})["session_id"]
    assert "keypoints" not in rep
    assert card.dispatch("status", {})["count_total"] == 1


def test_plugin_coalesces_fast_reps_to_latest_absolute_total(plugin_factory):
    card, clock, node = plugin_factory({"min_rep_event_interval_s": 10.0})
    event_topic = card.get_tool()["topic_out"][0]["topic"]
    card.dispatch("start", {"input_topic": "/camera/poses/skeleton"})
    subscription = node.subscriptions[-1]
    _plugin_standing(subscription, clock)
    for _ in range(3):
        _plugin_rep(subscription, clock)

    assert card.dispatch("status", {})["count_total"] == 3
    assert [event["count_total"] for event in _published_reps(node, event_topic)] == [1]
    clock.advance(10.0)
    node.timers[0]()
    assert [event["count_total"] for event in _published_reps(node, event_topic)] == [1, 3]


def test_plugin_stop_preserves_total_and_reset_clears_it(plugin_factory):
    card, clock, node = plugin_factory()
    card.dispatch("start", {"input_topic": "/camera/poses/skeleton"})
    subscription = node.subscriptions[-1]
    _plugin_standing(subscription, clock)
    _plugin_rep(subscription, clock)
    old_session = card.dispatch("status", {})["session_id"]

    stopped = card.dispatch("stop", {})
    assert stopped["state"] == "idle"
    assert stopped["count_total"] == 1
    assert subscription.destroyed
    _plugin_rep(subscription, clock)
    assert card.dispatch("status", {})["count_total"] == 1

    reset = card.dispatch("reset", {})
    assert reset["count_total"] == 0
    assert reset["session_id"] != old_session
    card.dispatch("start", {"input_topic": "/camera/poses/skeleton"})
    new_subscription = node.subscriptions[-1]
    _plugin_standing(new_subscription, clock)
    _plugin_rep(new_subscription, clock)
    assert card.dispatch("status", {})["count_total"] == 1


def test_plugin_selects_arm_exercise_and_requires_finish_before_switch(plugin_factory):
    card, _clock, node = plugin_factory({
        "require_explicit_workout_start": True, "coach_mode": True,
        "local_milestone_voice": True,
        "arm_exercises": {"dumbbell_curl": {"curl_peak_max_deg": 90.0}},
    })
    card.dispatch("start", {"input_topic": "/camera/poses/skeleton"})
    first = card.dispatch("begin", {
        "exercise": "dumbbell_curl", "sets": 1,
        "reps_per_set": 2, "rest_s": 15,
    })
    assert first["exercise"] == "dumbbell_curl"
    assert first["workout_state"] == "active"
    assert first["count_total"] == 0
    assert card.dispatch("info", {})["config"]["curl_peak_max_deg"] == 90.0
    assert card.dispatch("debug", {})["动作"] == "哑铃弯举"
    repeated = card.dispatch("begin", {"exercise": "dumbbell_curl"})
    assert repeated["session_id"] == first["session_id"]
    rejected = card.dispatch("begin", {"exercise": "lateral_raise"})
    assert rejected["state"] == "error"
    assert card.dispatch("status", {})["exercise"] == "dumbbell_curl"

    finished = card.dispatch("finish", {})
    assert finished["summary"]["exercise"] == "dumbbell_curl"
    second = card.dispatch("begin", {"exercise": "lateral_raise"})
    assert second["exercise"] == "lateral_raise"
    assert second["session_id"] != first["session_id"]
    assert second["count_total"] == 0
    assert card.dispatch("debug", {})["动作"] == "哑铃侧平举"
    assert len(node.subscriptions) == 1


@pytest.mark.parametrize("exercise,motion", [
    ("dumbbell_curl", {"curl": True}),
    ("lateral_raise", {"lateral_raise": True}),
])
def test_arm_pose_reaches_coach_plan_and_voice_milestone(
        plugin_factory, exercise, motion):
    card, clock, node = plugin_factory({
        "require_explicit_workout_start": True, "coach_mode": True,
        "local_milestone_voice": True,
    })
    tool = card.get_tool()
    event_topic = tool["topic_out"][0]["topic"]
    voice_topic = tool["topic_out"][3]["topic"]
    card.dispatch("start", {"input_topic": "/camera/poses/skeleton"})
    card.dispatch("begin", {"exercise": exercise, "sets": 1,
                            "reps_per_set": 1, "rest_s": 15})
    subscription = node.subscriptions[-1]
    for _ in range(11):
        _send_pose(subscription, clock, _arm_person())
    assert card.dispatch("status", {})["set_state"] == "in_set"
    for _ in range(6):
        _send_pose(subscription, clock, _arm_person(**motion))
    for _ in range(6):
        _send_pose(subscription, clock, _arm_person())

    status = card.dispatch("status", {})
    assert status["exercise"] == exercise
    assert status["count_total"] == 1
    assert status["summary"]["exercise"] == exercise
    assert status["summary"]["goal_reached"] is True
    assert [event["type"] for event in node.publishers[event_topic].payloads
            if event["exercise"] == exercise].count("workout_finished") == 1
    assert any(payload["type"] == "set_ready" and payload["exercise"] == exercise
               for payload in node.publishers[voice_topic].payloads)


@pytest.mark.parametrize("exercise,motion", [
    ("dumbbell_curl", {"curl": True}),
    ("lateral_raise", {"lateral_raise": True}),
])
def test_arm_coach_accepts_first_rep_immediately_after_ready(
        plugin_factory, exercise, motion):
    card, clock, node = plugin_factory({
        "require_explicit_workout_start": True, "coach_mode": True,
        "local_milestone_voice": True,
    })
    card.dispatch("start", {"input_topic": "/camera/poses/skeleton"})
    card.dispatch("begin", {"exercise": exercise, "sets": 1,
                            "reps_per_set": 1, "rest_s": 15})
    subscription = node.subscriptions[-1]
    for _ in range(7):
        _send_pose(subscription, clock, _arm_person())
        if card._counter.snapshot()["state"] == "ready":
            break
    else:
        pytest.fail("arm counter did not become ready")
    assert card.dispatch("status", {})["set_state"] == "in_set"
    for _ in range(5):
        _send_pose(subscription, clock, _arm_person(**motion))
    for _ in range(5):
        _send_pose(subscription, clock, _arm_person())
    assert card.dispatch("status", {})["count_total"] == 1


def test_stale_callback_after_topic_switch_cannot_change_new_session(plugin_factory):
    card, clock, node = plugin_factory()
    card.dispatch("start", {"input_topic": "/camera/a/poses/skeleton"})
    old_subscription = node.subscriptions[-1]
    _plugin_standing(old_subscription, clock)
    _plugin_rep(old_subscription, clock)
    old_session = card.dispatch("status", {})["session_id"]

    changed = card.dispatch("start", {"input_topic": "/camera/b/poses/skeleton"})
    new_subscription = node.subscriptions[-1]
    assert old_subscription.destroyed
    assert changed["count_total"] == 0
    assert changed["session_id"] != old_session
    _plugin_rep(old_subscription, clock)
    assert card.dispatch("status", {})["count_total"] == 0

    _plugin_standing(new_subscription, clock)
    _plugin_rep(new_subscription, clock)
    status = card.dispatch("status", {})
    assert status["input_topic"] == "/camera/b/poses/skeleton"
    assert status["count_total"] == 1
    assert status["session_id"] != old_session


def _published_debug(node, card):
    topic = next(port["topic"] for port in card.get_tool()["topic_out"]
                 if port["topic"].endswith("/fitness/squat/debug"))
    return node.publishers[topic].payloads


def _contains_raw_pose(value):
    if isinstance(value, dict):
        if {"keypoints", "keypoint_names", "persons"} & value.keys():
            return True
        return any(_contains_raw_pose(child) for child in value.values())
    if isinstance(value, list):
        return any(_contains_raw_pose(child) for child in value)
    return False


def test_debug_is_chinese_before_a_session_starts(plugin_factory):
    card, _, _ = plugin_factory()
    debug = card.dispatch("debug", {})
    assert debug["运行状态"] == "未运行"
    assert debug["动作阶段"] == "等待识别人体"
    assert debug["判定原因"] == "等待画面中出现关节清晰的人体"
    assert debug["判定项"]["蹲底条件全部达标"] == "暂无数据"
    assert debug["测量值"]["站姿基线"] is None


def test_debug_is_a_separate_dashboard_port_with_bounded_publish_rate(plugin_factory):
    card, clock, node = plugin_factory({"debug_interval_s": 0.5})
    ports = card.get_tool()["topic_out"]
    assert len(ports) == 3
    debug_port = next(port for port in ports
                      if port["topic"].endswith("/fitness/squat/debug"))
    assert debug_port["format"] == "data/json"
    assert debug_port["message_type"] == "std_msgs/msg/String"
    assert debug_port["topic"] not in {ports[0]["topic"], ports[1]["topic"]}

    card.dispatch("start", {"input_topic": "/camera/poses/skeleton"})
    subscription = node.subscriptions[-1]
    initial = len(_published_debug(node, card))
    for _ in range(10):
        _send_pose(subscription, clock, person(), dt=0.02)
    assert len(_published_debug(node, card)) <= initial + 1

    _plugin_standing(subscription, clock)
    debug = _published_debug(node, card)
    assert initial + 3 <= len(debug) <= initial + 8
    assert all(item["卡片"] == "深蹲计数" for item in debug)
    assert all(item["动作"] == "深蹲" for item in debug)
    assert all("priority" not in item for item in debug)
    assert all(not _contains_raw_pose(item) for item in debug)


def test_debug_action_exposes_current_phase_and_counting_checks(plugin_factory):
    card, clock, node = plugin_factory()
    card.dispatch("start", {"input_topic": "/camera/poses/skeleton"})
    subscription = node.subscriptions[-1]
    _plugin_standing(subscription, clock)

    standing = card.dispatch("debug", {})
    assert standing["运行状态"] == "运行中"
    assert standing["动作阶段"] == "站立"
    assert standing["累计次数"] == 0
    assert standing["当前目标"]["人体ID"] == 1
    assert standing["测量值"]["站姿基线"] is not None
    assert (standing["测量值"]["站姿基线膝角（度）"]
            == standing["测量值"]["站姿基线"]["当前膝角（度）"])
    assert standing["判定项"]["站立条件全部达标"] == "通过"
    assert standing["阈值"]["蹲底膝角最小变化（度）"] == 40

    for depth in (0.2, 0.4, 0.6, 0.8, 1, 1, 1, 1):
        _send_pose(subscription, clock, person(depth=depth))
    bottom = card.dispatch("debug", {})
    assert bottom["动作阶段"] == "蹲底"
    assert bottom["累计次数"] == 0
    assert bottom["判定项"]["蹲底条件全部达标"] == "通过"
    assert bottom["测量值"]["膝角相对站姿减小（度）"] >= 40
    assert bottom["当前目标"]["质量分"] > 0
    assert not _contains_raw_pose(bottom)

    for depth in (0.7, 0.5, 0.3, 0.2, 0.1):
        _send_pose(subscription, clock, person(depth=depth))
    ascent = card.dispatch("debug", {})
    assert ascent["动作阶段"] == "起身中"
    assert ascent["判定项"]["蹲底条件全部达标"] == "未通过"
    assert ascent["判定项"]["站立条件全部达标"] == "未通过"

    reasons = []
    for _ in range(7):
        _send_pose(subscription, clock, person())
        reasons.append(card.dispatch("debug", {})["判定原因"])
    completed = card.dispatch("debug", {})
    assert completed["累计次数"] == 1
    assert completed["动作阶段"] == "站立"
    assert "完成一次深蹲，计数加一" in reasons


def test_debug_tracks_bad_input_without_polluting_decision_events(plugin_factory):
    card, clock, node = plugin_factory({"debug_interval_s": 0.5})
    event_topic = card.get_tool()["topic_out"][0]["topic"]
    card.dispatch("start", {"input_topic": "/camera/poses/skeleton"})
    subscription = node.subscriptions[-1]
    _plugin_standing(subscription, clock)
    before_events = len(node.publishers[event_topic].payloads)

    subscription.callback(_String("{bad json"))
    clock.advance(0.6)
    node.timers[0]()
    debug = _published_debug(node, card)[-1]
    assert debug["无效帧数"] == 1
    assert debug["接收帧数"] == debug["解析帧数"] + 1
    assert debug["最近错误"] == "姿态输入不是有效 JSON"
    assert len(node.publishers[event_topic].payloads) == before_events
    assert not _contains_raw_pose(debug)


def test_unexpected_pose_errors_are_sampled_without_losing_diagnostics(
        plugin_factory, monkeypatch):
    card, clock, node = plugin_factory()
    card.dispatch("start", {"input_topic": "/camera/poses/skeleton"})
    subscription = node.subscriptions[-1]
    errors = []
    monkeypatch.setattr(node, "get_logger", lambda: types.SimpleNamespace(
        warn=lambda message: None, error=errors.append))

    def fail(_payload):
        raise RuntimeError("unsupported skeleton")

    monkeypatch.setattr(card._counter, "process", fail)
    for _ in range(101):
        _send_pose(subscription, clock, person())

    assert len(errors) == 2
    assert "1 invalid frame(s)" in errors[0]
    assert "100 invalid frame(s)" in errors[1]
    debug = card.dispatch("debug", {})
    assert debug["无效帧数"] == 101
    assert "unsupported skeleton" in debug["最近错误"]


def test_first_unexpected_error_is_logged_after_malformed_pose(plugin_factory,
                                                               monkeypatch):
    card, clock, node = plugin_factory()
    card.dispatch("start", {"input_topic": "/camera/poses/skeleton"})
    subscription = node.subscriptions[-1]
    warnings, errors = [], []
    monkeypatch.setattr(node, "get_logger", lambda: types.SimpleNamespace(
        warn=warnings.append, error=errors.append))
    subscription.callback(_String("{bad json"))

    def fail(_payload):
        raise RuntimeError("unsupported skeleton")

    monkeypatch.setattr(card._counter, "process", fail)
    _send_pose(subscription, clock, person())
    assert len(warnings) == 1
    assert len(errors) == 1
    assert "unsupported skeleton" in errors[0]


def test_debug_reports_stale_camera_without_new_pose_frames(plugin_factory):
    card, clock, node = plugin_factory({"debug_interval_s": 0.5,
                                        "input_stale_after_s": 1.0})
    card.dispatch("start", {"input_topic": "/camera/poses/skeleton"})
    subscription = node.subscriptions[-1]
    _send_pose(subscription, clock, person())
    assert card.dispatch("debug", {})["输入已过期"] == "否"

    before = len(_published_debug(node, card))
    clock.advance(1.1)
    node.timers[0]()
    latest = _published_debug(node, card)[-1]
    assert len(_published_debug(node, card)) == before + 1
    assert latest["输入已过期"] == "是"
    assert latest["距上一帧（秒）"] >= 1.0
    assert latest["输入话题"] == "/camera/poses/skeleton"


def test_explicit_workout_waits_for_voice_begin_and_begin_is_idempotent(plugin_factory):
    card, clock, node = plugin_factory({"require_explicit_workout_start": True})
    event_topic = card.get_tool()["topic_out"][0]["topic"]
    card.dispatch("start", {"input_topic": "/camera/poses/skeleton"})
    subscription = node.subscriptions[-1]

    _plugin_standing(subscription, clock)
    _plugin_rep(subscription, clock)
    waiting = card.dispatch("status", {})
    assert waiting["workout_state"] == "waiting_start"
    assert waiting["count_total"] == 0
    assert _published_reps(node, event_topic) == []

    started = card.dispatch("begin", {})
    repeated = card.dispatch("begin", {})
    assert started["workout_state"] == repeated["workout_state"] == "active"
    assert started["session_id"] == repeated["session_id"]
    _plugin_standing(subscription, clock)
    _plugin_rep(subscription, clock)
    assert card.dispatch("status", {})["count_total"] == 1


def test_explicit_finish_freezes_count_and_summary_with_rep_timing(plugin_factory):
    card, clock, node = plugin_factory({"require_explicit_workout_start": True})
    event_topic = card.get_tool()["topic_out"][0]["topic"]
    card.dispatch("start", {"input_topic": "/camera/poses/skeleton"})
    subscription = node.subscriptions[-1]
    started = card.dispatch("begin", {})
    _plugin_standing(subscription, clock)
    _plugin_rep(subscription, clock)
    (rep,) = _published_reps(node, event_topic)

    finished = card.dispatch("finish", {})
    summary = finished["summary"]
    assert finished["workout_state"] == "finished"
    assert summary["session_id"] == started["session_id"]
    assert summary["count_total"] == finished["count_total"] == 1
    assert summary["end_reason"] == "voice"
    assert summary["duration_s"] == pytest.approx(clock.now - 100.0, abs=0.1)
    assert summary["mean_rep_duration_s"] == pytest.approx(rep["rep_duration_s"], abs=0.01)
    assert summary["fastest_rep_s"] == pytest.approx(rep["rep_duration_s"], abs=0.01)
    assert summary["slowest_rep_s"] == pytest.approx(rep["rep_duration_s"], abs=0.01)

    assert card.dispatch("finish", {})["summary"] == summary
    _plugin_standing(subscription, clock)
    _plugin_rep(subscription, clock)
    assert card.dispatch("status", {})["summary"] == summary
    assert len(_published_reps(node, event_topic)) == 1


def test_explicit_workout_brief_loss_recovers_but_sustained_absence_finishes(plugin_factory):
    card, clock, node = plugin_factory({
        "require_explicit_workout_start": True,
        "auto_finish_absence_s": 6.0,
    })
    event_topic = card.get_tool()["topic_out"][0]["topic"]
    card.dispatch("start", {"input_topic": "/camera/poses/skeleton"})
    subscription = node.subscriptions[-1]
    card.dispatch("begin", {})
    _plugin_standing(subscription, clock)
    _plugin_rep(subscription, clock)

    for _ in range(10):
        _send_pose(subscription, clock)
    assert card.dispatch("status", {})["workout_state"] == "active"
    _plugin_standing(subscription, clock)
    assert card.dispatch("status", {})["workout_state"] == "active"

    for _ in range(64):
        _send_pose(subscription, clock)
    finished = card.dispatch("status", {})
    assert finished["workout_state"] == "finished"
    assert finished["summary"]["count_total"] == 1
    assert finished["summary"]["end_reason"] == "left_frame"
    events = [e for e in node.publishers[event_topic].payloads
              if e["type"] == "workout_finished"]
    assert len(events) == 1
    assert events[0]["summary"] == finished["summary"]


def test_explicit_workout_camera_stale_is_not_absence(plugin_factory):
    card, clock, node = plugin_factory({
        "require_explicit_workout_start": True,
        "auto_finish_absence_s": 6.0,
        "input_stale_after_s": 1.0,
    })
    event_topic = card.get_tool()["topic_out"][0]["topic"]
    card.dispatch("start", {"input_topic": "/camera/poses/skeleton"})
    subscription = node.subscriptions[-1]
    card.dispatch("begin", {})
    _plugin_standing(subscription, clock)
    for _ in range(8):
        _send_pose(subscription, clock)

    clock.advance(8.0)
    node.timers[0]()
    status = card.dispatch("status", {})
    assert status["input_stale"] is True
    assert status["workout_state"] == "active"
    assert status["summary"] is None


def test_explicit_workout_visible_but_unreadable_person_is_not_departure(plugin_factory):
    card, clock, node = plugin_factory({
        "require_explicit_workout_start": True,
        "auto_finish_absence_s": 6.0,
    })
    event_topic = card.get_tool()["topic_out"][0]["topic"]
    card.dispatch("start", {"input_topic": "/camera/poses/skeleton"})
    subscription = node.subscriptions[-1]
    card.dispatch("begin", {})
    _plugin_standing(subscription, clock)

    for _ in range(70):
        _send_pose(subscription, clock, person(missing_legs=True))

    status = card.dispatch("status", {})
    assert status["target_state"] == "missing"
    assert status["candidate_count"] == 1
    assert status["workout_state"] == "active"
    assert status["summary"] is None
    assert not [e for e in node.publishers[event_topic].payloads
                if e["type"] == "workout_finished"]


def test_explicit_workout_target_switch_finishes_original_subject(plugin_factory):
    card, clock, node = plugin_factory({"require_explicit_workout_start": True})
    event_topic = card.get_tool()["topic_out"][0]["topic"]
    card.dispatch("start", {"input_topic": "/camera/poses/skeleton"})
    subscription = node.subscriptions[-1]
    started = card.dispatch("begin", {})
    _plugin_standing(subscription, clock)
    _plugin_rep(subscription, clock)

    for _ in range(12):
        _send_pose(subscription, clock)
    _send_pose(subscription, clock, person(track_id=2, anchor_x=730))
    finished = card.dispatch("status", {})
    assert finished["workout_state"] == "finished"
    assert finished["summary"]["session_id"] == started["session_id"]
    assert finished["summary"]["target_id"] == 1
    assert finished["summary"]["count_total"] == 1
    assert finished["summary"]["end_reason"] == "target_changed"
    assert [e["summary"]["session_id"] for e in node.publishers[event_topic].payloads
            if e["type"] == "workout_finished"] == [started["session_id"]]

    _plugin_standing(subscription, clock)
    _plugin_rep(subscription, clock)
    assert card.dispatch("status", {})["summary"] == finished["summary"]


def _start_coach_workout(plugin_factory, *, sets=2, reps_per_set=1,
                         rest_s=15, min_rep_event_interval_s=3.0,
                         local_milestone_voice=False):
    card, clock, node = plugin_factory({
        "require_explicit_workout_start": True,
        "coach_mode": True,
        "auto_finish_absence_s": 6.0,
        "min_rep_event_interval_s": min_rep_event_interval_s,
        "local_milestone_voice": local_milestone_voice,
    })
    event_topic = card.get_tool()["topic_out"][0]["topic"]
    card.dispatch("start", {"input_topic": "/camera/poses/skeleton"})
    subscription = node.subscriptions[-1]
    started = card.dispatch("begin", {
        "sets": sets, "reps_per_set": reps_per_set, "rest_s": rest_s,
    })
    return card, clock, node, subscription, event_topic, started


def _events(node, topic, kind):
    return [event for event in node.publishers[topic].payloads
            if event["type"] == kind]


def test_coach_begin_records_plan_and_repeated_begin_keeps_progress(plugin_factory):
    card, clock, node, subscription, topic, started = _start_coach_workout(
        plugin_factory, sets=2, reps_per_set=2, rest_s=15)

    assert started["plan"] == {"sets": 2, "reps_per_set": 2, "rest_s": 15}
    assert started["set_index"] == 1
    assert started["set_count"] == 0
    assert started["set_state"] == "preparing"
    _plugin_standing(subscription, clock)
    assert card.dispatch("status", {})["set_state"] == "in_set"
    _plugin_rep(subscription, clock)

    repeated = card.dispatch("begin", {
        "sets": 3, "reps_per_set": 5, "rest_s": 30,
    })
    assert repeated["session_id"] == started["session_id"]
    assert repeated["plan"] == started["plan"]
    assert repeated["set_index"] == 1
    assert repeated["set_count"] == repeated["count_total"] == 1


def test_coach_target_lost_event_identifies_pose_pause_not_card_pause(plugin_factory):
    card, clock, node, subscription, topic, started = _start_coach_workout(
        plugin_factory, sets=2, reps_per_set=8, rest_s=90)
    assert started["state"] == "running"
    assert started["coach_mode"] is True
    _plugin_standing(subscription, clock)

    for _ in range(10):
        _send_pose(subscription, clock)

    lost = _events(node, topic, "target_lost")[-1]
    assert lost["state"] == "paused"
    assert lost["card_state"] == "running"
    assert lost["workout_state"] == "active"
    assert lost["set_state"] == "in_set"


def test_coach_set_goal_is_immediate_despite_rep_throttle_and_rest_ignores_reps(
        plugin_factory):
    card, clock, node, subscription, topic, _ = _start_coach_workout(
        plugin_factory, sets=2, reps_per_set=2, rest_s=15,
        min_rep_event_interval_s=10.0)
    _plugin_standing(subscription, clock)
    _plugin_rep(subscription, clock)
    assert card.dispatch("status", {})["set_count"] == 1
    _plugin_rep(subscription, clock)

    status = card.dispatch("status", {})
    assert status["set_state"] == "resting"
    assert status["set_count"] == status["count_total"] == 2
    completed = _events(node, topic, "set_completed")
    assert len(completed) == 1
    assert completed[0]["set_index"] == 1
    assert completed[0]["set_count"] == 2

    # Further motions during the break do not become counted repetitions.
    _plugin_rep(subscription, clock)
    status = card.dispatch("status", {})
    assert status["set_state"] == "resting"
    assert status["set_count"] == status["count_total"] == 2
    assert len(_events(node, topic, "set_completed")) == 1


def test_coach_rest_requires_next_set_and_finishes_after_final_goal(plugin_factory):
    card, clock, node, subscription, topic, started = _start_coach_workout(
        plugin_factory, sets=2, reps_per_set=1, rest_s=15)
    _plugin_standing(subscription, clock)
    assert len(_events(node, topic, "set_ready")) == 1
    _plugin_rep(subscription, clock)
    assert card.dispatch("status", {})["set_state"] == "resting"

    # The original begin command is idempotent even while resting.
    assert card.dispatch("begin", {})["set_state"] == "resting"
    clock.advance(15.1)
    node.timers[0]()
    assert card.dispatch("status", {})["set_state"] == "ready_next"
    assert len(_events(node, topic, "rest_finished")) == 1
    node.timers[0]()
    assert len(_events(node, topic, "rest_finished")) == 1

    _plugin_rep(subscription, clock)
    assert card.dispatch("status", {})["count_total"] == 1

    next_set = card.dispatch("next_set", {})
    assert next_set["set_index"] == 2
    assert next_set["set_count"] == 0
    assert next_set["count_total"] == 1
    _plugin_standing(subscription, clock)
    _plugin_rep(subscription, clock)

    finished = card.dispatch("status", {})
    assert finished["workout_state"] == "finished"
    assert finished["set_state"] == "finished"
    assert finished["count_total"] == 2
    assert finished["summary"]["count_total"] == 2
    assert len(_events(node, topic, "workout_finished")) == 1
    assert _events(node, topic, "workout_finished")[0]["summary"] == finished["summary"]
    assert card.dispatch("finish", {})["summary"] == finished["summary"]


def test_coach_voice_finish_during_rest_freezes_partial_plan(plugin_factory):
    card, clock, node, subscription, topic, started = _start_coach_workout(
        plugin_factory, sets=2, reps_per_set=1, rest_s=15)
    _plugin_standing(subscription, clock)
    _plugin_rep(subscription, clock)
    assert card.dispatch("status", {})["set_state"] == "resting"

    finished = card.dispatch("finish", {})
    assert finished["workout_state"] == "finished"
    assert finished["set_state"] == "finished"
    assert finished["summary"]["session_id"] == started["session_id"]
    assert finished["summary"]["count_total"] == 1
    assert finished["summary"]["end_reason"] == "voice"
    clock.advance(20)
    node.timers[0]()
    assert not _events(node, topic, "rest_finished")
    assert card.dispatch("finish", {})["summary"] == finished["summary"]


def test_coach_sustained_absence_during_rest_finishes_workout(plugin_factory):
    card, clock, node, subscription, topic, _ = _start_coach_workout(
        plugin_factory, sets=2, reps_per_set=1, rest_s=15)
    _plugin_standing(subscription, clock)
    _plugin_rep(subscription, clock)
    assert card.dispatch("status", {})["set_state"] == "resting"

    for _ in range(64):
        _send_pose(subscription, clock)
    finished = card.dispatch("status", {})
    assert finished["set_state"] == "finished"
    assert finished["summary"]["count_total"] == 1
    assert finished["summary"]["end_reason"] == "left_frame"
    assert len(_events(node, topic, "workout_finished")) == 1


def test_coach_target_change_during_rest_finishes_original_subject(plugin_factory):
    card, clock, node, subscription, topic, started = _start_coach_workout(
        plugin_factory, sets=2, reps_per_set=1, rest_s=15)
    _plugin_standing(subscription, clock)
    _plugin_rep(subscription, clock)
    assert card.dispatch("status", {})["set_state"] == "resting"

    for _ in range(12):
        _send_pose(subscription, clock)
    _send_pose(subscription, clock, person(track_id=2, anchor_x=730))
    finished = card.dispatch("status", {})
    assert finished["set_state"] == "finished"
    assert finished["summary"]["session_id"] == started["session_id"]
    assert finished["summary"]["count_total"] == 1
    assert finished["summary"]["end_reason"] == "target_changed"
    assert len(_events(node, topic, "workout_finished")) == 1


def test_coach_rejects_invalid_plan_without_starting_or_mutating_session(plugin_factory):
    card, clock, node = plugin_factory({
        "require_explicit_workout_start": True, "coach_mode": True,
    })
    card.dispatch("start", {"input_topic": "/camera/poses/skeleton"})
    original = card.dispatch("status", {})
    for invalid in ({"sets": True}, {"sets": 6}, {"reps_per_set": 0},
                    {"rest_s": 15.5}):
        assert card.dispatch("begin", invalid)["state"] == "error"
        status = card.dispatch("status", {})
        assert status["workout_state"] == "waiting_start"
        assert status["session_id"] == original["session_id"]
        assert status["plan"] is None
    assert card.dispatch("begin", {})["plan"] == {
        "sets": 2, "reps_per_set": 8, "rest_s": 90,
    }


def test_coach_announces_only_milestones_and_goal_without_old_pending_reps(
        plugin_factory):
    card, clock, node, subscription, topic, _ = _start_coach_workout(
        plugin_factory, sets=2, reps_per_set=5, rest_s=15)
    _plugin_standing(subscription, clock)
    for _ in range(5):
        _plugin_rep(subscription, clock)

    assert card.dispatch("status", {})["count_total"] == 5
    assert [event["set_count"] for event in _events(node, topic, "rep_completed")] == [1, 3]
    assert [event["set_count"] for event in _events(node, topic, "set_completed")] == [5]
    clock.advance(1.0)
    node.timers[0]()
    assert [event["set_count"] for event in _events(node, topic, "rep_completed")] == [1, 3]


def test_coach_local_milestones_bypass_model_and_expire(plugin_factory):
    card, clock, node, subscription, event_topic, _ = _start_coach_workout(
        plugin_factory, sets=2, reps_per_set=5, rest_s=15,
        local_milestone_voice=True)
    voice_topic = card.get_tool()["topic_out"][3]["topic"]
    assert voice_topic.endswith("/fitness/squat/voice")
    _plugin_standing(subscription, clock)
    for _ in range(5):
        _plugin_rep(subscription, clock)

    assert _events(node, event_topic, "rep_completed") == []
    assert _events(node, event_topic, "set_ready") == []
    assert len(_events(node, event_topic, "set_completed")) == 1
    prompts = node.publishers[voice_topic].payloads
    assert [(p["type"], p["set_count"]) for p in prompts] == [
        ("set_ready", 0), ("rep_completed", 1),
        ("rep_completed", 3), ("set_completed", 5),
    ]
    assert prompts[-1]["text"] == "5次，本组完成。休息15秒。"
    assert all(p["expires_at"] > 0 for p in prompts)
    status = card.dispatch("status", {})
    assert status["local_milestone_voice"] is True
    assert status["last_voice"]["type"] == "set_completed"

    clock.advance(15.1)
    node.timers[0]()
    assert _events(node, event_topic, "rest_finished") == []
    assert node.publishers[voice_topic].payloads[-1]["type"] == "rest_finished"
    card.dispatch("next_set", {})
    _plugin_standing(subscription, clock)
    for _ in range(5):
        _plugin_rep(subscription, clock)
    assert node.publishers[voice_topic].payloads[-1]["type"] == "workout_finished"
    assert len(_events(node, event_topic, "workout_finished")) == 1


def test_coach_next_set_accepts_expired_rest_without_timer_and_ignores_waiting_reps(
        plugin_factory):
    card, clock, node, subscription, topic, _ = _start_coach_workout(
        plugin_factory, sets=2, reps_per_set=1, rest_s=15)
    _plugin_standing(subscription, clock)
    _plugin_rep(subscription, clock)
    assert card.dispatch("next_set", {})["state"] == "error"
    clock.advance(15.1)
    # The voice tool call may beat the 200 ms rest timer callback.
    next_set = card.dispatch("next_set", {})
    assert next_set["set_index"] == 2
    assert next_set["set_state"] == "preparing"
    assert next_set["count_total"] == 1
    assert not _events(node, topic, "rest_finished")
    node.timers[0]()
    assert not _events(node, topic, "rest_finished")
    _plugin_standing(subscription, clock)
    _plugin_rep(subscription, clock)
    assert card.dispatch("status", {})["summary"]["count_total"] == 2


def test_repeated_pose_timestamp_cannot_make_coach_ready_or_appear_fresh(
        plugin_factory):
    card, clock, node, subscription, topic, _ = _start_coach_workout(
        plugin_factory, sets=1, reps_per_set=1, rest_s=15)
    # Reach the calibrated top pose, then replay a frozen frame.
    _plugin_standing(subscription, clock, frames=22)
    assert card.dispatch("status", {})["set_state"] == "preparing"
    frozen = _String(json.dumps(skeleton_payload(clock.now, [person()])))
    for _ in range(10):
        clock.advance(0.1)
        subscription.callback(frozen)
    assert card.dispatch("status", {})["set_state"] == "preparing"
    assert card.dispatch("status", {})["input_stale"] is False
    clock.advance(1.6)
    subscription.callback(frozen)
    assert card.dispatch("status", {})["input_stale"] is True
    assert not _events(node, topic, "set_ready")
