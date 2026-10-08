"""Hardware-free checks of G1 LED sequencing, ownership and worker shutdown."""
import importlib.util
import json
from pathlib import Path
import threading
import time

import pytest
from jsonschema import Draft202012Validator

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("g1_led_control_test", ROOT / "unitree/g1/led_control.py")
led = importlib.util.module_from_spec(spec)
spec.loader.exec_module(led)
GREEN = {"r": 0, "g": 255, "b": 0, "duration_sec": 2}
RED = {"r": 255, "g": 0, "b": 0, "duration_sec": 1}


class Client:
    def __init__(self):
        self.calls = []
        self.event = threading.Event()
        self.ret = 0

    def LedControl(self, *rgb):
        self.calls.append((time.monotonic(), rgb))
        self.event.set()
        return self.ret


@pytest.fixture
def model():
    """Deterministic phase/command tests without launching a worker."""
    p = led.LedPlugin({}, "", None, Client())
    clock = [100.0]
    p._clock = lambda: clock[0]
    p.start = lambda: setattr(p, "_enabled", True)
    return p, clock


def cycle(p, **kwargs):
    return p.dispatch("cycle", {"sequence": [GREEN, RED], **kwargs})


def test_multiple_cycles_boundaries_and_delayed_wakeup(model):
    p, clock = model
    cycle(p, repeat_count=2, end_behavior="hold")
    for elapsed, expected in [(0, (1, 1)), (2, (1, 2)), (3, (2, 1)), (5.5, (2, 2))]:
        clock[0] = 100 + elapsed
        p._advance(clock[0])
        assert (p._cycle, p._stage) == expected
    clock[0] = 107  # late worker wakes; no replay of missed colours
    p._advance(clock[0])
    assert p._status == "completed"
    assert p._rgb == (255, 0, 0)
    assert p._elapsed == 6
    assert p._mode == "hold"


def test_pause_resume_freezes_elapsed_and_blocks_hooks(model):
    p, clock = model
    cycle(p)
    clock[0] = 101
    p.dispatch("pause", {})
    assert p._rgb == (0, 255, 0)
    for state in ("speaking", "thinking", "hearing", "idle"):
        assert p.dispatch("state", {"state": state})["ignored"]
    clock[0] = 111
    assert p.dispatch("info", {})["elapsed_sec"] == 1
    p.dispatch("resume", {})
    clock[0] = 112
    p._advance(clock[0])
    assert p._stage == 2
    assert p._elapsed == 2


def test_error_hook_preempts_and_cannot_resume(model):
    p, _ = model
    cycle(p)
    p.dispatch("state", {"state": "error"})
    assert p._status == "interrupted"
    assert p._semantic_rgb(100) == (255, 0, 0)
    with pytest.raises(ValueError, match="no_paused"):
        p.dispatch("resume", {})


@pytest.mark.parametrize("ending,mode,rgb,pending", [
    ("release", "state", (0, 0, 0), True),
    ("off", "hold", (0, 0, 0), False),
    ("hold", "hold", (255, 0, 0), False),
])
def test_completion_options(model, ending, mode, rgb, pending):
    p, _ = model
    cycle(p, end_behavior=ending)
    p._advance(103)
    assert (p._mode, p._rgb, p._release_pending) == (mode, rgb, pending)
    assert p._status == "completed"


def test_infinite_cycles_and_replacement(model):
    p, clock = model
    cycle(p, repeat_count=0)
    clock[0] = 3000
    p._advance(clock[0])
    assert p._status == "running"
    assert p._info()["remaining_sec"] is None
    cycle(p, sequence=[RED], repeat_count=1)
    assert p._origin == 3000
    assert p._stage == 1
    p.dispatch("set", {"r": 4, "g": 5, "b": 6})
    assert p._stages == []
    assert p._rgb == (4, 5, 6)
    assert p.dispatch("state", {"state": "speaking"})["ignored"]


@pytest.mark.parametrize("overrides", [
    {"sequence": []}, {"sequence": "bad-json"},
    {"repeat_count": True}, {"repeat_count": -1},
    {"sequence": [{**GREEN, "r": 256}]},
    {"sequence": [{**GREEN, "duration_sec": float("nan")}]},
    {"sequence": [{**GREEN, "duration_sec": 1e-300}]},
    {"sequence": [{**GREEN, "duration_sec": .1}]},
    {"sequence": [{**GREEN, "typo": 1}]},
])
def test_invalid_input_preserves_current_cycle(model, overrides):
    p, _ = model
    cycle(p)
    old = list(p._stages)
    with pytest.raises(ValueError):
        cycle(p, **overrides)
    assert p._stages == old and p._status == "running"


def test_schema_canvas_json_and_native_arguments(model):
    p, _ = model
    schema = p.get_tool()["inputSchema"]
    Draft202012Validator.check_schema(schema)
    for sequence in ([GREEN, RED], json.dumps([GREEN, RED])):
        args = {"action": "cycle", "sequence": sequence, "repeat_count": 3}
        Draft202012Validator(schema).validate(args)
        assert p.dispatch("cycle", args)["cycle_status"] == "running"
    assert set(schema["properties"]["action"]["enum"]) == set(schema["x-action-params"])
    assert all(v["description"] for v in schema["x-action-params"].values())


def wait_for(predicate, timeout=1.5):
    end = time.monotonic() + timeout
    while not predicate():
        assert time.monotonic() < end, "worker did not make progress"
        time.sleep(.005)


def test_worker_continuous_output_pause_stop_and_restart():
    hz = 5
    c = Client()
    p = led.LedPlugin({}, "", None, c)
    try:
        cycle(p)
        wait_for(lambda: len(c.calls) >= 3)
        intervals = [c.calls[i + 1][0] - c.calls[i][0] for i in range(2)]
        assert all(.8 / hz <= gap < 2 / hz for gap in intervals)
        p.dispatch("pause", {})
        before = len(c.calls)
        wait_for(lambda: len(c.calls) >= before + 2)
        assert all(rgb == (0, 255, 0) for _, rgb in c.calls)
        p.stop()
        assert c.calls[-1][1] == (0, 0, 0)
        count = len(c.calls)
        time.sleep(.25)
        assert len(c.calls) == count
        assert not p._thread.is_alive()
        assert p.dispatch("state", {"state": "speaking"})["ignored"]
        p.start()
        p.dispatch("set", {"r": 1, "g": 2, "b": 3})
        wait_for(lambda: c.calls[-1][1] == (1, 2, 3))
    finally:
        p.stop()


def test_release_allows_semantic_effect_after_completion():
    c = Client()
    p = led.LedPlugin({}, "", None, c)
    try:
        cycle(p, sequence=[{**GREEN, "duration_sec": .2}])
        wait_for(lambda: p._status == "completed")
        wait_for(lambda: c.calls[-1][1] == (0, 0, 0))
        p.dispatch("state", {"state": "speaking"})
        wait_for(lambda: c.calls[-1][1] == (129, 216, 208))
    finally:
        p.stop()


def test_sdk_failure_is_visible_and_does_not_retry_forever():
    c = Client()
    c.ret = 7
    p = led.LedPlugin({}, "", None, c)
    try:
        cycle(p)
        wait_for(lambda: p._status == "failed")
        assert "code=7" in p._info()["last_error"]
        count = len(c.calls)
        time.sleep(.25)
        assert len(c.calls) == count
    finally:
        p.stop()


def test_stop_waits_for_inflight_sdk_and_prevents_stale_writes():
    entered, release, stopped = threading.Event(), threading.Event(), threading.Event()

    class BlockingClient(Client):
        def LedControl(self, *rgb):
            if not self.calls:
                entered.set()
                assert release.wait(2)
            return super().LedControl(*rgb)

    c = BlockingClient()
    p = led.LedPlugin({}, "", None, c)
    stopper = None
    try:
        cycle(p)
        assert entered.wait(1)
        def stop():
            p.stop()
            stopped.set()
        stopper = threading.Thread(target=stop)
        stopper.start()
        assert not stopped.wait(.05)
        release.set()
        assert stopped.wait(1)
        assert c.calls[-1][1] == (0, 0, 0)
        count = len(c.calls)
        time.sleep(.15)
        assert len(c.calls) == count
        assert not p._thread.is_alive()
    finally:
        release.set()
        if stopper:
            stopper.join(2)
        p.stop()


def test_slow_sdk_does_not_starve_stop():
    class SlowClient(Client):
        def LedControl(self, *rgb):
            time.sleep(.25)  # longer than the fixed 5 Hz refresh period
            return super().LedControl(*rgb)

    c = SlowClient()
    p = led.LedPlugin({}, "", None, c)
    stopped = threading.Event()
    stopper = None
    try:
        cycle(p)
        assert c.event.wait(1)
        def stop():
            p.stop()
            stopped.set()
        stopper = threading.Thread(target=stop, daemon=True)
        stopper.start()
        assert stopped.wait(1), "slow output must yield control to stop"
        assert not p._thread.is_alive()
        assert c.calls[-1][1] == (0, 0, 0)
    finally:
        if stopper:
            stopper.join(1)
        if stopped.is_set():
            p.stop()


def test_actuator_lifecycle_contract_and_fresh_restart():
    p = led.LedPlugin({}, "", None, Client())
    try:
        assert p.dispatch("start", {}) == {"state": "ready"}
        cycle(p, repeat_count=3)
        p.dispatch("pause", {})
        assert p.dispatch("stop", {})["state"] == "idle"
        assert p.dispatch("info", {})["cycle_status"] == "stopped"
        assert p.dispatch("start", {}) == {"state": "ready"}
        info = p.dispatch("info", {})
        assert info["state"] == "ready"
        assert info["cycle_status"] == "idle"
        assert info["stage_index"] == info["cycle_index"] == 0
        assert info["elapsed_sec"] == 0
        assert info["remaining_sec"] is None
        assert info["repeat_count"] is None
        assert info["last_error"] is None
        assert p._stages == p._bounds == []
        p.dispatch("set", {"r": 1})
        assert p.dispatch("info", {})["cycle_status"] == "idle"
    finally:
        p.stop()


def test_repeated_start_preserves_running_cycle():
    p = led.LedPlugin({}, "", None, Client())
    try:
        cycle(p, repeat_count=2)
        origin, thread = p._origin, p._thread
        assert p.dispatch("start", {}) == {"state": "ready"}
        assert p._origin == origin
        assert p._thread is thread
        assert p.dispatch("info", {})["cycle_status"] == "running"
        assert p._repeats == 2
    finally:
        p.stop()


@pytest.mark.parametrize("legacy_hz", [5, "5", 10, "10", "", None, 6])
def test_legacy_refresh_input_cannot_change_fixed_frequency(model, legacy_hz):
    p, _ = model
    for action, args in [("cycle", {"sequence": [GREEN]}), ("set", {"b": 255}), ("off", {})]:
        result = p.dispatch(action, {**args, "refresh_hz": legacy_hz})
        assert result["refresh_hz"] == 5
    schema = p.get_tool()["inputSchema"]
    assert "refresh_hz" not in schema["properties"]
    assert all("refresh_hz" not in entry["params"] for entry in schema["x-action-params"].values())


def test_interrupt_action_removed_without_changing_other_actions(model):
    p, _ = model
    schema = p.get_tool()["inputSchema"]
    expected = {"start", "state", "set", "cycle", "pause", "resume", "off", "stop", "info"}
    assert set(schema["properties"]["action"]["enum"]) == expected
    assert set(schema["x-action-params"]) == expected
    cycle(p)
    assert p.dispatch("interrupt", {}) is None
    assert p._status == "running"


@pytest.mark.parametrize("action", ["unknown_action", "interrupt"])
def test_unknown_action_returns_none_without_changing_output(model, action):
    p, _ = model
    assert p.dispatch(action, {}) is None
    assert not p._enabled
    cycle(p)
    before = p.dispatch("info", {})
    origin, due = p._origin, p._next_due
    assert p.dispatch(action, {}) is None
    assert p.dispatch("info", {}) == before
    assert (p._origin, p._next_due) == (origin, due)
    assert p._client.calls == []
