import importlib.util
from pathlib import Path

import pytest

spec = importlib.util.spec_from_file_location("zone", Path(__file__).resolve().parents[1]/"zone_presence.py")
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
Occupancy = module.Occupancy


def frame(*positions):
    return {"objects": [{"name": "person", "confidence": .9, "position": list(p)} for p in positions]}


def feed(engine, payload, begin, end):
    events = []
    for i in range(round((end-begin)*10)+1):
        event = engine.update(payload, round(begin+i*.1, 4))
        if event:
            events.append(event)
    return events


def test_enter_once_count_and_leave_after_hold():
    engine = Occupancy()
    events = feed(engine, frame((0, 0), (.9, 0)), 0, 5)
    assert [e["event"] for e in events] == ["entered"]
    state, _ = engine.read(5)
    assert state["count"] == 1 and state["dwell_s"] == 3
    events = feed(engine, frame(), 5.1, 6.3)
    assert [e["event"] for e in events] == ["left"]


def test_brief_presence_and_brief_absence_are_debounced():
    engine = Occupancy()
    assert feed(engine, frame((0, 0)), 0, 1) == []
    feed(engine, frame(), 1.1, 2.3)
    events = feed(engine, frame((0, 0)), 2.4, 5)
    assert len(events) == 1
    assert feed(engine, frame(), 5.1, 5.5) == []
    assert feed(engine, frame((0, 0)), 5.6, 6) == []
    assert engine.read(6)[0]["state"] == "occupied"


def test_stale_is_unknown_without_false_departure_and_recovery_requires_hold():
    engine = Occupancy()
    feed(engine, frame((0, 0)), 0, 3)
    state, event = engine.read(6)
    assert state["state"] == "unknown" and state["count"] is None
    assert event["event"] == "unknown"
    assert engine.read(7)[1] is None
    assert engine.update(frame((0, 0)), 7) is None
    assert [e["event"] for e in feed(engine, frame((0, 0)), 7.1, 9.2)] == ["entered"]


def test_two_isolated_frames_cannot_prove_sustained_occupancy():
    engine = Occupancy()
    engine.update(frame((0, 0)), 0)
    assert engine.update(frame((0, 0)), 2.5) is None
    assert engine.read(2.5)[0]["state"] == "unknown"


def test_invalid_input_is_not_reported_as_empty():
    engine = Occupancy()
    feed(engine, frame((0, 0)), 0, 3)
    event = engine.update({"objects": [{"name": "person", "confidence": .9}]}, 3.1)
    assert event["event"] == "unknown"
    assert engine.read(3.1)[0]["count"] is None
    assert engine.update({"objects": []}, 3.2) is None


def test_region_boundary_filter_and_duplicate_clock():
    engine = Occupancy()
    engine.update(frame((-.6, -.8), (.6, 1), (.7, 0)), 1)
    assert engine.read(1)[0]["count"] == 2
    engine.update(frame(), 1)
    assert engine.read(1)[0]["count"] == 2


@pytest.mark.parametrize("options", [dict(region=[0, 0, 0, 1]), dict(region=[0, 0, 2, 1]),
    dict(enter_s=True), dict(confidence=float("nan")), dict(max_gap_s=3)])
def test_invalid_config(options):
    with pytest.raises(ValueError):
        Occupancy(**options)
