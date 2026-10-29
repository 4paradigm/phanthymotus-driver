"""Offline tests for servo integration; no SDK, ROS, or hardware."""
import importlib.util
import json
import sys
import time
from pathlib import Path
from types import SimpleNamespace

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
spec = importlib.util.spec_from_file_location("g1_loco_servo", ROOT / "unitree/g1/loco_servo.py")
m = importlib.util.module_from_spec(spec)
spec.loader.exec_module(m)

class Transport:
    def __init__(self):
        self.moves = []
        self.stops = 0
        self.ack = "ok"
        self.paused = False
        self.error = None
        self.reject = False
    def status(self):
        return {"stop_status": self.ack, "paused": self.paused, "last_error": self.error}
    def submit_velocity(self, *args, deadline):
        if self.reject or self.paused or deadline <= time.monotonic(): return False
        self.moves.append((args, deadline)); return True
    def pause(self):
        self.stops += 1; self.paused = True; self.ack = "pending"; return True
    def stop(self): return self.pause()
    def wait_stop(self, timeout): return self.ack
    def resume(self):
        if self.ack != "ok": return False
        self.paused = False; return True

def command(seq=1):
    now = int(time.time() * 1000)
    return {"schema": "motus.control/1", "mode": "twist", "dof": 6,
            "values": [.2, .1, 0, 0, 0, -.3], "seq": seq, "source": "test",
            "priority": 50, "stamp_ms": now, "obs_stamp_ms": now, "ttl_ms": 200}

def card(t=None, reader=None, **cfg):
    t = t or Transport()
    cfg.setdefault("dry_run", False)
    obj = m.LocoServoPlugin(cfg, "g1", object(), t,
                             reader or (lambda: {"fsm_id": 500, "ts": time.time()}))
    obj._open = lambda topic: None
    assert obj.dispatch("start", {"input_topic": "/test"})["state"] == "running"
    return obj, t

def send(obj, cmd=None):
    obj._on_message(SimpleNamespace(data=json.dumps(cmd or command())))

def test_submit_deadline_and_mapping():
    obj, t = card(); cmd = command(); before = time.monotonic(); send(obj, cmd)
    args, deadline = t.moves[0]
    assert args == (.2, .1, -.3, 1.0)
    assert before < deadline <= before + .21
    assert obj._info()["physical_stop_confirmed"] is False

def test_optional_observation_timestamp_matches_control_sink_contract():
    obj, t = card()
    cmd = command()
    del cmd["obs_stamp_ms"]
    send(obj, cmd)
    assert len(t.moves) == 1


@pytest.mark.parametrize("field", ["stamp_ms", "obs_stamp_ms"])
def test_far_future_timestamp_is_refused(field):
    obj, t = card()
    cmd = command(); cmd[field] += 10000
    send(obj, cmd)
    assert not t.moves
    assert obj._last_command["verdict"] == "ERROR"


def test_bundle_shutdown_seals_start_and_resume():
    obj, t = card()
    assert obj.stop()["state"] == "stopped"
    assert obj.dispatch("start", {"input_topic": "/test"})["state"] == "error"
    assert obj.dispatch("resume", {})["state"] == "error"


def test_observation_deadline_and_revalidation(monkeypatch):
    obj, t = card(); cmd = command(); cmd["obs_stamp_ms"] -= 450
    before = time.monotonic(); send(obj, cmd)
    assert before < t.moves[0][1] <= before + .06
    obj, t = card()
    def stale():
        time.sleep(.025)
        return {"fsm_id": 500, "ts": time.time()}
    obj._fsm_reader = stale
    cmd = command(); cmd["ttl_ms"] = 10
    send(obj, cmd)
    assert not t.moves and obj._last_command["verdict"] == "ERROR"

@pytest.mark.parametrize("snapshot", [{}, {"fsm_id": 500, "ts": 0},
                                      {"fsm_id": 811, "ts": time.time()},
                                      {"fsm_id": 500, "ts": float("nan")}])
def test_fsm_missing_stale_wrong_mode_refused(snapshot):
    obj, t = card(reader=lambda: snapshot); send(obj)
    assert not t.moves and obj._last_command["verdict"] == "ERROR"

@pytest.mark.parametrize("action", ["pause", "stop", "bundle", "watchdog"])
def test_pending_failed_retry_and_resume(action):
    obj, t = card(); send(obj)
    if action == "bundle":
        with pytest.raises(RuntimeError): obj.stop()
    elif action == "watchdog":
        obj._sink._last_apply_ms -= m.WATCHDOG_MS + 1; obj._tick()
    else:
        assert obj.dispatch(action, {})["state"] == "stop_pending"
    assert t.stops == 1
    assert obj.dispatch("resume", {})["state"] == "error"
    t.ack = "failed"; t.error = "SDK failed"
    assert obj._info()["state"] == "stop_failed"
    assert obj.dispatch("stop", {})["state"] == "stop_pending"
    assert t.stops == 2
    t.ack = "ok"; t.error = None
    assert obj.dispatch("stop", {})["state"] == "stopped"

def test_watchdog_resumes_only_after_stop_ack_and_fresh_command():
    obj, t = card(); send(obj)
    obj._sink._last_apply_ms -= m.WATCHDOG_MS + 1
    obj._tick()
    assert t.ack == "pending"
    send(obj, command(2))
    assert len(t.moves) == 1
    t.ack = "ok"
    stale = command(2); stale["stamp_ms"] -= 1000
    send(obj, stale)
    assert len(t.moves) == 1 and t.paused
    send(obj, command(3))
    assert len(t.moves) == 2 and not t.paused


def test_manual_pause_after_watchdog_does_not_auto_resume():
    obj, t = card(); send(obj)
    obj._sink._last_apply_ms -= m.WATCHDOG_MS + 1
    obj._tick(); t.ack = "ok"
    obj.dispatch("pause", {})
    send(obj, command(2))
    assert len(t.moves) == 1 and t.paused


def test_transport_watchdog_state_and_explicit_resume():
    obj, t = card(); send(obj)
    t.paused = True; t.ack = "pending"
    send(obj, command(2)); assert len(t.moves) == 1
    assert obj._info()["state"] == "stop_pending"
    t.ack = "ok"
    assert obj.dispatch("resume", {})["state"] == "running"
    send(obj, command(2)); assert len(t.moves) == 2

def test_dry_run_never_writes_and_live_toggle_waits():
    obj, t = card(dry_run=True); send(obj)
    assert not t.moves
    assert obj.dispatch("stop", {})["state"] == "stopped" and t.stops == 0
    obj, t = card(); send(obj)
    assert not obj.dispatch("config", {"dry_run": True})["ok"]
    assert not obj._dry_run and t.stops == 1
    t.ack = "ok"
    assert obj.dispatch("config", {"dry_run": True})["ok"]

def test_transport_rejection_is_visible():
    obj, t = card(); t.reject = True; send(obj)
    assert obj._last_command["verdict"] == "ERROR"
    assert "rejected" in obj._info()["last_error"]

@pytest.mark.parametrize("hz", [0, 1, 1000/300, 21, float("nan"), float("inf")])
def test_expected_frequency(hz):
    with pytest.raises(ValueError): m.LocoServoPlugin({"expected_hz": hz}, "g1", None, Transport(), lambda: {})


def test_stop_request_exception_cannot_report_stopped():
    obj, t = card(); send(obj)
    def fail(): raise RuntimeError("queue broken")
    t.pause = fail
    assert obj.dispatch("pause", {})["state"] == "stop_failed"
    assert obj.dispatch("resume", {})["state"] == "error"
    with pytest.raises(RuntimeError, match="queue broken"): obj.stop()



def test_pending_stop_can_be_retried_with_pause():
    obj, t = card(); send(obj)
    assert obj.dispatch("stop", {})["state"] == "stop_pending"
    assert obj.dispatch("pause", {})["state"] == "stop_pending"
    assert t.stops == 2

def test_dry_run_transition_rejects_stop_queue_error():
    obj, t = card(); send(obj)
    def fail(): raise RuntimeError("queue broken")
    t.pause = fail
    assert not obj.dispatch("config", {"dry_run": True})["ok"]
    assert obj._dry_run is False
