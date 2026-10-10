import json
import sys
import threading
import time
import types

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
    assert events[0]["priority"] == 1

    clock.advance(10)
    events = engine.poll(clock.monotonic, clock.wall)
    assert [(event["type"], event["event"]) for event in events] == [
        ("timer_alarm", "green-finished"),
        ("timer_completed", "timer-completed"),
    ]
    assert all(event["remaining_sec"] == 0 for event in events)
    assert [event["priority"] for event in events] == [1, 1]
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
    assert events[0]["priority"] == 0
    clock.advance(0.6)
    assert [event["type"] for event in engine.poll(clock.monotonic, clock.wall)] == [
        "timer_tick"]


@pytest.mark.parametrize("mode", ["countdown", "countup"])
@pytest.mark.parametrize("poll_delay", [0, 2])
def test_due_timer_emits_alarm_and_completion_without_refresh_tick(mode, poll_delay):
    clock, engine = Clock(), TimerEngine()
    engine.start({
        "timer_id": "phase",
        "mode": mode,
        "duration_sec": 3,
        "emit_interval_sec": 1,
        "alarms": [{"alarm_id": "done", "trigger_type": "elapsed",
                    "trigger_sec": 3, "event": "phase-done"}],
    }, clock.monotonic, clock.wall)

    clock.advance(2)
    assert [event["type"] for event in engine.poll(clock.monotonic, clock.wall)] == [
        "timer_tick"]
    clock.advance(1 + poll_delay)
    assert [event["type"] for event in engine.poll(clock.monotonic, clock.wall)] == [
        "timer_alarm", "timer_completed"]
    assert engine.poll(clock.monotonic, clock.wall) == []


@pytest.mark.parametrize("mode", ["countdown", "countup"])
def test_completion_drift_uses_actual_elapsed_time(mode):
    clock, engine = Clock(), TimerEngine()
    engine.start({"timer_id": "delayed", "mode": mode, "duration_sec": 3},
                 clock.monotonic, clock.wall)
    clock.advance(8)
    completed, = engine.poll(clock.monotonic, clock.wall)
    assert completed["elapsed_sec"] == 3
    assert completed["scheduled_elapsed_sec"] == 3
    assert completed["drift_ms"] == 5000


def test_completion_drift_excludes_paused_time():
    clock, engine = Clock(), TimerEngine()
    engine.start({"timer_id": "paused", "mode": "countdown", "duration_sec": 3},
                 clock.monotonic, clock.wall)
    clock.advance(1)
    engine.pause("paused", clock.monotonic)
    clock.advance(10)
    engine.resume("paused", clock.monotonic)
    clock.advance(4)
    completed, = engine.poll(clock.monotonic, clock.wall)
    assert completed["drift_ms"] == 2000


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
    assert event["priority"] == 1
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
    assert set(action_params) == set(actions)
    assert all(isinstance(entry["description"], str) and entry["description"].strip()
               for entry in action_params.values())
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


def test_plugin_accepts_dashboard_serialized_create_fields():
    plugin = TimerPlugin({}, "test", None)
    plugin.start()
    schema = plugin.get_tool()["inputSchema"]["properties"]
    assert schema["alarms"]["type"] == ["array", "string"]
    assert schema["payload"]["type"] == ["object", "string"]
    for field in ("replace", "auto_remove"):
        assert schema[field]["type"] == "string"
        assert schema[field]["enum"] == ["false", "true"]
        assert schema[field]["default"] == "false"

    first = plugin.dispatch("create", {
        "timer_id": "canvas-test",
        "mode": "countdown",
        "duration_sec": 30,
        "alarms": """[{"alarm_id":"done","trigger_type":"remaining",
                       "trigger_sec":0,"event":"canvas-done"}]""",
        "payload": "{\"scene\":\"canvas\"}",
        "replace": "",
        "auto_remove": "false",
    })
    assert first["next_alarm"]["alarm_id"] == "done"
    assert first["payload"] == {"scene": "canvas"}

    replacement = plugin.dispatch("create", {
        "timer_id": "canvas-test",
        "mode": "countdown",
        "duration_sec": 5,
        "alarms": "",
        "payload": "",
        "replace": "TRUE",
        "auto_remove": "",
    })
    assert replacement["run_id"] != first["run_id"]
    assert replacement["next_alarm"] is None
    assert replacement["payload"] == {}
    plugin.stop()


def test_canvas_and_native_json_arguments_match_advertised_schema():
    jsonschema = pytest.importorskip("jsonschema")
    plugin = TimerPlugin({}, "test", None)
    schema = plugin.get_tool()["inputSchema"]
    jsonschema.Draft202012Validator.check_schema(schema)
    validator = jsonschema.Draft202012Validator(schema)
    alarms = [{"alarm_id": "done", "trigger_type": "remaining",
               "trigger_sec": 0, "event": "done"}]
    canvas = {
        "action": "create", "timer_id": "canvas-wire", "mode": "countdown",
        "duration_sec": 5, "alarms": json.dumps(alarms),
        "payload": json.dumps({"scene": "canvas"}),
        "replace": "false", "auto_remove": "false",
    }
    native = {**canvas, "timer_id": "native-wire", "alarms": alarms,
              "payload": {"scene": "native"}}
    validator.validate(canvas)
    validator.validate(native)
    assert not validator.is_valid({**canvas, "replace": True})
    plugin.start()
    try:
        for wire in (canvas, native):
            result = plugin.dispatch("create", {k: v for k, v in wire.items()
                                                 if k != "action"})
            assert result["status"] == "running"
            assert result["next_alarm"]["alarm_id"] == "done"
    finally:
        plugin.stop()


def test_plugin_ignores_agent_dispatch_metadata_but_rejects_unknown_timer_fields():
    plugin = TimerPlugin({}, "test", None)
    plugin.start()
    args = {
        "timer_id": "agent-test", "mode": "countdown", "duration_sec": 5,
        "_tool_name": "timer", "concurrent": False,
        "_trace_id": "trace-123", "instance_id": "canvas-instance",
    }
    created = plugin.dispatch("create", args)
    assert created["status"] == "running"
    assert args["_trace_id"] == "trace-123"
    assert args["instance_id"] == "canvas-instance"
    assert plugin.dispatch("info", {
        "timer_id": "agent-test", "_trace_id": "trace-456",
        "instance_id": "canvas-instance",
    })["status"] == "running"
    with pytest.raises(ValueError, match="invalid_start_fields: surprise"):
        plugin.dispatch("create", {
            "timer_id": "another-test", "mode": "countdown", "duration_sec": 5,
            "surprise": True,
        })
    plugin.stop()


@pytest.mark.parametrize("field,value,error", [
    ("alarms", "[", "alarms_must_be_valid_json"),
    ("alarms", "111", "invalid_alarms"),
    ("payload", "{", "payload_must_be_valid_json"),
    ("payload", "[]", "payload_must_be_object"),
    ("replace", "yes", "boolean_option_required"),
    ("auto_remove", "0", "boolean_option_required"),
])
def test_plugin_rejects_invalid_dashboard_serialized_fields(field, value, error):
    plugin = TimerPlugin({}, "test", None)
    plugin.start()
    args = {
        "timer_id": "invalid-canvas-input",
        "mode": "countdown",
        "duration_sec": 5,
        field: value,
    }
    with pytest.raises(ValueError, match=error):
        plugin.dispatch("create", args)
    plugin.stop()


def test_plugin_rejects_create_resume_and_reset_without_worker():
    plugin = TimerPlugin({}, "test", None)
    with pytest.raises(ValueError, match="timer_not_started"):
        plugin.dispatch("create", {"timer_id": "demo", "mode": "countdown",
                                   "duration_sec": 30})
    assert plugin.dispatch("list", {})["timers"] == []

    plugin.start()
    try:
        plugin.dispatch("create", {"timer_id": "demo", "mode": "countdown",
                                   "duration_sec": 30})
        plugin.dispatch("pause", {"timer_id": "demo"})
    finally:
        plugin.stop()
    for action in ("resume", "reset"):
        with pytest.raises(ValueError, match="timer_not_started"):
            plugin.dispatch(action, {"timer_id": "demo"})
    assert plugin.dispatch("info", {"timer_id": "demo"})["status"] == "paused"
    plugin.start()
    try:
        assert plugin.dispatch("resume", {"timer_id": "demo"})["status"] == "running"
    finally:
        plugin.stop()


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


def test_failing_event_sink_does_not_stop_later_timer_events(capsys):
    received = []
    second_completed = threading.Event()

    def sink(event):
        if event["timer_id"] == "first" and event["type"] == "timer_tick":
            raise RuntimeError("sink unavailable")
        received.append(event)
        if event["timer_id"] == "second" and event["type"] == "timer_completed":
            second_completed.set()

    plugin = TimerPlugin({}, "test", None, event_sink=sink)
    plugin.dispatch("start", {})
    try:
        plugin.dispatch("create", {
            "timer_id": "first", "mode": "countdown",
            "duration_sec": 0.2, "emit_interval_sec": 0.1,
        })
        plugin.dispatch("create", {
            "timer_id": "second", "mode": "countdown", "duration_sec": 0.35,
        })
        assert second_completed.wait(1.5)
        assert plugin.dispatch("info", {"timer_id": "first"})["status"] == "completed"
        assert plugin.dispatch("info", {"timer_id": "second"})["status"] == "completed"
        assert any(event["timer_id"] == "first" and event["type"] == "timer_completed"
                   for event in received)
        assert "event sink failed: sink unavailable" in capsys.readouterr().out
    finally:
        plugin.dispatch("stop", {})


def test_persistent_delivery_failures_are_throttled_and_escaped(capsys, monkeypatch):
    clock = Clock()
    failing = True

    def sink(_event):
        if failing:
            raise RuntimeError("bad\nline " + "X" * 1000)

    class Publisher:
        def publish(self, _message):
            if failing:
                raise RuntimeError("publish\nline " + "Y" * 1000)

    message_module = types.ModuleType("std_msgs.msg")
    message_module.String = type("String", (), {})
    monkeypatch.setitem(sys.modules, "std_msgs", types.ModuleType("std_msgs"))
    monkeypatch.setitem(sys.modules, "std_msgs.msg", message_module)
    plugin = TimerPlugin({}, "test", None, monotonic=lambda: clock.monotonic,
                         event_sink=sink)
    plugin._publisher = Publisher()

    for _ in range(100):
        plugin._publish({"type": "timer_tick"})
    first_logs = capsys.readouterr().out.splitlines()
    assert len(first_logs) == 2
    assert all(len(line) < 300 and "\\nline" in line for line in first_logs)

    clock.advance(60)
    plugin._publish({"type": "timer_tick"})
    sampled_logs = capsys.readouterr().out.splitlines()
    assert len(sampled_logs) == 2
    assert all("suppressed 99 repeats" in line for line in sampled_logs)

    failing = False
    plugin._publish({"type": "timer_tick"})
    assert capsys.readouterr().out == ""
    failing = True
    plugin._publish({"type": "timer_tick"})
    assert len(capsys.readouterr().out.splitlines()) == 2
