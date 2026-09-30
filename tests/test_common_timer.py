import threading
import time

import pytest

from common.timer import TimerEngine, TimerPlugin


class Clock:
    def __init__(self):
        self.monotonic = 100.0
        self.wall = 1_800_000_000.0

    def advance(self, seconds):
        self.monotonic += seconds
        self.wall += seconds


def countdown(engine, clock, **overrides):
    args = {
        "timer_id": "green-light",
        "mode": "countdown",
        "duration_sec": 30,
        "alarms": [
            {"alarm_id": "ten-left", "trigger_type": "remaining",
             "trigger_sec": 10, "event": "green-ending"},
            {"alarm_id": "zero", "trigger_type": "remaining",
             "trigger_sec": 0, "event": "green-finished"},
        ],
        "payload": {"scene": "traffic", "cycle": 1},
    }
    args.update(overrides)
    return engine.start(args, clock.monotonic, clock.wall)


def test_countdown_alarm_and_completion_are_ordered_and_once_only():
    clock, engine = Clock(), TimerEngine()
    started = countdown(engine, clock)
    assert started["remaining_sec"] == 30
    assert started["next_alarm"]["alarm_id"] == "ten-left"

    clock.advance(20)
    events = engine.poll(clock.monotonic, clock.wall)
    assert [(event["type"], event["event"]) for event in events] == [
        ("timer_alarm", "green-ending")]
    assert events[0]["remaining_sec"] == 10
    assert events[0]["payload"] == {"scene": "traffic", "cycle": 1}

    clock.advance(10)
    events = engine.poll(clock.monotonic, clock.wall)
    assert [(event["type"], event["event"]) for event in events] == [
        ("timer_alarm", "green-finished"),
        ("timer_completed", "timer-completed"),
    ]
    assert all(event["remaining_sec"] == 0 for event in events)
    assert engine.poll(clock.monotonic, clock.wall) == []


def test_countup_alarm_pause_resume_and_reset():
    clock, engine = Clock(), TimerEngine()
    engine.start({
        "timer_id": "exercise",
        "mode": "countup",
        "duration_sec": 120,
        "alarms": [{"alarm_id": "minute", "trigger_type": "elapsed",
                    "trigger_sec": 60, "event": "one-minute"}],
    }, clock.monotonic, clock.wall)
    clock.advance(30)
    assert engine.pause("exercise", clock.monotonic)["elapsed_sec"] == 30
    clock.advance(50)
    assert engine.info("exercise", clock.monotonic)["elapsed_sec"] == 30
    engine.resume("exercise", clock.monotonic)
    clock.advance(30)
    assert [event["event"] for event in engine.poll(clock.monotonic, clock.wall)] == [
        "one-minute"]

    old_run = engine.info("exercise", clock.monotonic)["run_id"]
    reset = engine.reset("exercise", clock.monotonic, clock.wall)
    assert reset["run_id"] != old_run
    assert reset["elapsed_sec"] == 0


def test_tick_coalesces_missed_intervals_instead_of_flooding():
    clock, engine = Clock(), TimerEngine()
    countdown(engine, clock, emit_interval_sec=1, alarms=[])
    clock.advance(5.4)
    events = engine.poll(clock.monotonic, clock.wall)
    assert len(events) == 1
    assert events[0]["type"] == "timer_tick"
    clock.advance(0.6)
    assert [event["type"] for event in engine.poll(clock.monotonic, clock.wall)] == [
        "timer_tick"]


@pytest.mark.parametrize("terminal_status", ["completed", "cancelled"])
@pytest.mark.parametrize("restart", ["start", "reset"])
def test_terminal_timer_does_not_bypass_active_timer_limit(terminal_status, restart):
    clock, engine = Clock(), TimerEngine(max_timers=1)
    countdown(engine, clock, timer_id="first", duration_sec=1, alarms=[])
    if terminal_status == "completed":
        clock.advance(1)
        engine.poll(clock.monotonic, clock.wall)
    else:
        engine.cancel("first", clock.monotonic, clock.wall)
    countdown(engine, clock, timer_id="second", duration_sec=30, alarms=[])
    with pytest.raises(ValueError, match="timer_limit_reached"):
        if restart == "reset":
            engine.reset("first", clock.monotonic, clock.wall)
        else:
            countdown(engine, clock, timer_id="first", alarms=[])
    assert engine.info("first", clock.monotonic)["status"] == terminal_status
    assert engine.info("second", clock.monotonic)["status"] == "running"
    replacement = countdown(engine, clock, timer_id="second", replace=True, alarms=[])
    assert replacement["status"] == "running"
    engine.cancel("second", clock.monotonic, clock.wall)
    assert engine.reset("first", clock.monotonic, clock.wall)["status"] == "running"


@pytest.mark.parametrize("once", [True, False])
def test_alarm_once_field_is_rejected(once):
    clock, engine = Clock(), TimerEngine()
    with pytest.raises(ValueError, match="invalid_alarm_fields"):
        countdown(engine, clock, alarms=[{
            "alarm_id": "repeat",
            "trigger_type": "elapsed",
            "trigger_sec": 1,
            "event": "repeat",
            "once": once,
        }])


def test_replace_invalidates_old_run_and_cancel_is_terminal():
    clock, engine = Clock(), TimerEngine()
    first = countdown(engine, clock)
    with pytest.raises(ValueError, match="timer_already_exists"):
        countdown(engine, clock)
    second = countdown(engine, clock, replace=True)
    assert second["run_id"] != first["run_id"]
    result, event = engine.cancel("green-light", clock.monotonic)
    assert result["status"] == "cancelled"
    assert event["run_id"] == second["run_id"]
    clock.advance(100)
    assert engine.poll(clock.monotonic, clock.wall) == []


@pytest.mark.parametrize("args,error", [
    ({"timer_id": "BAD", "mode": "countdown", "duration_sec": 3},
     "invalid_timer_id"),
    ({"timer_id": "timer", "mode": "countdown"},
     "countdown_requires_duration_sec"),
    ({"timer_id": "timer", "mode": "countup"},
     "unbounded_timer_has_no_output"),
    ({"timer_id": "timer", "mode": "countup", "alarms": [
        {"alarm_id": "bad", "trigger_type": "remaining", "trigger_sec": 1,
         "event": "bad"}]}, "remaining_alarm_requires_countdown"),
])
def test_invalid_inputs_are_rejected(args, error):
    clock, engine = Clock(), TimerEngine()
    with pytest.raises(ValueError, match=error):
        engine.start(args, clock.monotonic, clock.wall)


def test_plugin_dispatched_lifecycle_is_idempotent_and_reports_state():
    plugin = TimerPlugin({}, "test", None)
    tool = plugin.get_tool()
    actions = tool["inputSchema"]["properties"]["action"]["enum"]
    action_params = tool["inputSchema"]["x-action-params"]

    assert {"start", "stop", "create"}.issubset(actions)
    assert action_params["start"]["params"] == []
    assert action_params["stop"]["params"] == []
    assert "timer_id" in action_params["create"]["params"]
    assert plugin.dispatch("info", {}) == {
        "state": "idle",
        "topic_out": [{"topic": "/test/timer/events", "format": "data/json"}],
    }

    assert plugin.dispatch("start", {}) == {"state": "running"}
    worker = plugin._thread
    assert worker is not None and worker.is_alive()
    assert plugin.dispatch("start", {}) == {"state": "running"}
    assert plugin._thread is worker
    assert plugin.dispatch("info", {})["state"] == "running"

    assert plugin.dispatch("stop", {}) == {"state": "idle"}
    assert plugin._thread is None
    assert plugin.dispatch("stop", {}) == {"state": "idle"}
    assert plugin.dispatch("info", {})["state"] == "idle"


def test_plugin_schema_and_background_delivery():
    events, delivered = [], threading.Event()

    def sink(event):
        events.append(event)
        delivered.set()

    plugin = TimerPlugin({}, "test", None, event_sink=sink)
    tool = plugin.get_tool()
    assert tool["type"] == "processor"
    alarm_schema = tool["inputSchema"]["properties"]["alarms"]["items"]
    assert "once" not in alarm_schema["properties"]
    assert alarm_schema["additionalProperties"] is False
    assert tool["topic_out"] == [{"topic": "/test/timer/events", "format": "data/json"}]
    assert plugin.dispatch("start", {}) == {"state": "running"}
    try:
        result = plugin.dispatch("create", {
            "timer_id": "short",
            "mode": "countdown",
            "duration_sec": 0.03,
            "alarms": [{"alarm_id": "done", "trigger_type": "remaining",
                        "trigger_sec": 0, "event": "short-done"}],
        })
        assert result["status"] == "running"
        assert delivered.wait(1.0)
        deadline = time.monotonic() + 1.0
        while not any(item["type"] == "timer_completed" for item in events):
            assert time.monotonic() < deadline
            time.sleep(0.005)
        assert [item["type"] for item in events] == [
            "timer_alarm", "timer_completed"]
        info = plugin.dispatch("info", {"timer_id": "short"})
        assert info["status"] == "completed"
        assert info["topic_out"] == [
            {"topic": "/test/timer/events", "format": "data/json"}]
    finally:
        assert plugin.dispatch("stop", {}) == {"state": "idle"}
