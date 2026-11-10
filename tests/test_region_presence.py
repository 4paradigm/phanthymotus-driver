"""Stateful acceptance with real VOP-shaped frames and a controlled clock."""
import json
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from common.region_presence import PresenceConfig, PresenceMonitor, count_matches


def frame(stamp, count=1, name="person", x=0, y=0, confidence=.9):
    return {"timestamp": stamp, "count": count, "objects": [
        {"name": name, "position": [x, y], "confidence": confidence}
        for _ in range(count)]}


def ready(monitor, count=1, start=0):
    monitor.ingest(frame(start+100, count), start)
    duration = monitor.config.enter_hold_s if count else monitor.config.leave_hold_s
    return monitor.ingest(frame(start+101, count), start+duration)


def test_initialized_is_not_a_claim_that_someone_just_entered():
    m = PresenceMonitor()
    assert m.snapshot(0)["present"] is None
    assert m.ingest(frame(100), 0) == []
    assert m.snapshot(0)["present"] is None
    assert m.ingest(frame(101), .7)[0]["event"] == "initialized"
    assert m.snapshot(.7)["present"] is True


def test_capacity_alert_and_recovery_require_sustained_new_frames():
    m = PresenceMonitor(PresenceConfig(capacity_limit=1, enter_hold_s=1, leave_hold_s=1))
    def observe(stamp, at, count):
        return [e["event"] for e in m.ingest(frame(stamp, count), at)]

    observe(1, 0, 1)
    observe(2, 1, 1)
    assert m.snapshot(1)["over_capacity"] is False
    assert observe(3, 2, 2) == []
    assert observe(3, 3, 2) == []
    assert observe(4, 3.1, 1) == []
    assert observe(5, 4, 2) == []
    assert observe(6, 5, 3) == ["capacity_exceeded"]
    assert observe(7, 6, 2) == []
    assert observe(8, 7, 1) == []
    assert m.snapshot(7)["over_capacity"] is True
    assert observe(9, 8, 1) == ["capacity_restored"]
    assert observe(10, 9, 1) == []
    m.poll(13)
    assert m.snapshot(13)["over_capacity"] is None
    assert observe(11, 14, 2) == []
    assert observe(12, 15, 2) == ["initialized", "capacity_exceeded"]


def test_dwell_only_on_new_occupied_frame_once_per_confirmed_visit():
    m = PresenceMonitor(PresenceConfig(dwell_alert_s=2, enter_hold_s=0, leave_hold_s=0))
    ready(m)
    assert m.ingest(frame(102), 1) == []
    assert m.snapshot(1.5)["occupied_s"] == 1
    assert m.ingest(frame(102), 2) == []  # replay
    assert m.ingest(frame(103, 0), 2) == []  # pending departure
    assert m.snapshot(2)["occupied_s"] is None
    event, = m.ingest(frame(104), 2.1)
    assert (event["event"], event["occupied_s"]) == ("dwell_reached", 2.1)
    assert m.ingest(frame(105), 2.2) == []
    m.ingest(frame(106, 0), 3)
    m.ingest(frame(107, 0), 4)
    assert m.snapshot(4)["occupied_s"] is None
    m.ingest(frame(108), 5)
    m.ingest(frame(109), 6)
    assert m.ingest(frame(110), 8)[0]["event"] == "dwell_reached"
    m.poll(12)
    assert m.snapshot(12)["occupied_s"] is None
    m.ingest(frame(111), 13)
    assert m.ingest(frame(112), 14)[0]["occupied_s"] == 0


def test_empty_to_occupied_to_empty_one_event_each_not_every_frame():
    m = PresenceMonitor()
    assert ready(m, 0)[0]["present"] is False
    assert m.ingest(frame(102), 2) == []
    assert m.snapshot(2)["present"] is None  # transition is not confirmed empty
    event = m.ingest(frame(103, 2), 2.7)[0]
    assert (event["event"], event["detected_count"]) == ("appeared", 2)
    assert m.ingest(frame(104, 3), 3) == []  # crowd size changes are not arrivals
    assert m.ingest(frame(105, 0), 4) == []
    assert m.ingest(frame(106, 0), 5.5)[0]["event"] == "disappeared"
    assert m.ingest(frame(107, 0), 6) == []


def test_brief_miss_does_not_leave_and_reenter():
    m = PresenceMonitor()
    ready(m)
    assert m.ingest(frame(102, 0), 1) == []
    assert m.snapshot(1)["present"] is None
    assert m.ingest(frame(103), 1.3) == []
    assert m.snapshot(1.3)["present"] is True


def test_transient_detection_does_not_enter():
    m = PresenceMonitor()
    ready(m, 0)
    m.ingest(frame(102), 2)
    assert m.ingest(frame(103, 0), 2.2) == []
    assert m.snapshot(2.2)["present"] is False


def test_timeout_is_unknown_not_departure_and_resume_is_initialization():
    m = PresenceMonitor()
    ready(m)
    result = m.poll(4)
    assert result[0]["event"] == "unavailable"
    assert result[0]["state"] == "stale"
    assert m.snapshot(4)["present"] is None
    assert m.poll(5) == []
    assert m.ingest(frame(102, 0), 5) == []
    assert m.ingest(frame(103, 0), 6.5)[0]["event"] == "initialized"


def test_dwell_does_not_bridge_a_data_gap():
    m = PresenceMonitor()
    m.ingest(frame(100), 0)
    events = m.ingest(frame(101), 4)
    assert [e["event"] for e in events] == ["unavailable"]
    assert m.snapshot(4)["present"] is None
    assert m.ingest(frame(102), 4.7)[0]["event"] == "initialized"


def test_zero_hold_still_needs_two_independent_frames():
    m = PresenceMonitor(PresenceConfig(enter_hold_s=0))
    assert m.ingest(frame(100), 0) == []
    assert m.ingest(frame(101), .01)[0]["present"] is True


def test_replay_and_old_frames_do_not_refresh_freshness_or_confirm_dwell():
    m = PresenceMonitor()
    m.ingest(frame(100), 0)
    assert m.ingest(frame(100), 1) == []
    assert m.ingest(frame(99, 0), 2) == []
    events = m.ingest(frame(100), 4)
    assert events[0]["state"] == "stale"
    assert m.snapshot(4)["present"] is None
    assert m.snapshot(4)["received_age_s"] == 4


def test_roi_uses_vop_centre_coordinates_and_filters_name_confidence():
    cfg = PresenceConfig(x_min=0, x_max=.5, y_min=-.5, y_max=.5)
    objects = [frame(1, x=0)["objects"][0], frame(1, x=.5)["objects"][0],
               frame(1, x=-.1)["objects"][0], frame(1, name="chair")["objects"][0],
               frame(1, confidence=.49)["objects"][0]]
    assert count_matches({"timestamp": 100, "count": 5, "objects": objects}, cfg) == (100, 2)
    assert count_matches(frame(100, name="Person"), PresenceConfig())[1] == 1
    assert count_matches(frame(100, name="chair"), PresenceConfig(target_name="chair"))[1] == 1


@pytest.mark.parametrize("payload", [
    {}, [], {"error": "model unavailable"}, {"soc": 50},
    {"timestamp": 1, "count": 0, "objects": None},
    {"timestamp": 1, "count": True, "objects": []},
    {"timestamp": 1, "count": 0, "objects": [{}]},
    frame(float("nan")), frame(1, x=640), frame(1, confidence=float("inf")),
    frame(1, confidence=True), frame(1, confidence=10**500),
])
def test_bad_or_wrong_card_data_invalidates_a_previous_good_result(payload):
    m = PresenceMonitor()
    ready(m)
    events = m.ingest(payload, 1)
    assert events[0]["state"] == "invalid_data"
    assert m.snapshot(1)["present"] is None
    assert m.snapshot(1)["detected_count"] is None
    assert m.ingest(payload, 1.1) == []
    assert m.ingest(frame(102), 1.2) == []
    assert m.ingest(frame(103), 2)[0]["event"] == "initialized"


@pytest.mark.parametrize("changes", [
    {"x_min": .8, "x_max": .2}, {"y_min": -1, "y_max": -1},
    {"min_confidence": "0.5"}, {"target_name": " "}, {"leave_hold_s": -1},
    {"enter_hold_s": True}, {"stale_after_s": float("nan")}, {"typo": 1},
])
def test_configuration_rejects_bad_values(changes):
    cfg = PresenceConfig()
    with pytest.raises(ValueError):
        cfg.updated(changes)
    assert cfg == PresenceConfig()


def test_events_are_json_serializable_and_do_not_include_source_objects():
    m = PresenceMonitor()
    event = ready(m)[0]
    json.dumps(event, allow_nan=False)
    assert "objects" not in event
    assert event["priority"] == 1
