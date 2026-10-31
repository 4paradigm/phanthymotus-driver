"""Offline concurrency tests; these do not exercise DDS or a robot."""
import importlib.util
import threading
import time
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("g1_servo_transport", ROOT / "unitree/g1/loco_servo_transport.py")
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def eventually(predicate, timeout=1):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if predicate():
            return
        time.sleep(0.002)
    assert predicate()


class FakeClient:
    def __init__(self):
        self.calls = []
        self.lock = threading.Lock()
        self.write_gate = None

    def _begin(self, kind, args):
        if self.write_gate is not None and kind == "move":
            self.write_gate.wait()
        event = threading.Event()
        record = {"kind": kind, "args": args, "event": event, "code": 0, "consumed": False}
        with self.lock:
            self.calls.append(record)
        def wait():
            event.wait()
            record["consumed"] = True
            return record["code"]
        return wait

    def BeginVelocity(self, *args):
        return self._begin("move", args)

    def BeginStopMove(self):
        return self._begin("stop", ())

    def items(self, kind):
        with self.lock:
            return [r for r in self.calls if r["kind"] == kind]


def send(t, x=1, ttl=2):
    return t.submit_velocity(x, 0, 0, .3, deadline=time.monotonic() + ttl)


def release_all(client):
    with client.lock:
        for record in client.calls:
            record["event"].set()


def test_stop_overtakes_ack_and_stale_ack_cannot_change_new_generation():
    c = FakeClient()
    t = module.LocoServoTransport(c, watchdog_s=10)
    assert send(t)
    eventually(lambda: len(c.items("move")) == 1)
    assert t.pause()
    eventually(lambda: len(c.items("stop")) == 1)
    assert not t.resume() and not send(t)
    c.items("stop")[0]["event"].set()
    eventually(lambda: t.status()["stop_status"] == "ok")
    assert t.resume()
    c.items("move")[0]["code"] = 42
    c.items("move")[0]["event"].set()
    eventually(lambda: c.items("move")[0]["consumed"])
    assert not t.status()["paused"]
    assert send(t, 2)
    eventually(lambda: len(c.items("move")) == 2)
    t.close()
    eventually(lambda: len(c.items("stop")) == 2)
    release_all(c)
    eventually(lambda: all(r["consumed"] for r in c.calls))


def test_latest_only_outstanding_bound_and_deadline_expiry():
    c = FakeClient()
    t = module.LocoServoTransport(c, watchdog_s=10, max_outstanding=2)
    assert send(t, 1)
    eventually(lambda: len(c.items("move")) == 1)
    assert send(t, 2, ttl=.08)
    assert send(t, 3, ttl=.08)
    eventually(lambda: len(c.items("stop")) == 1)
    assert len(c.items("move")) == 1
    assert t.status()["outstanding"] == 2
    release_all(c)
    eventually(lambda: t.status()["stop_status"] == "ok")
    assert t.resume()
    assert not send(t, ttl=-1)
    t.close()
    eventually(lambda: len(c.items("stop")) == 2)
    release_all(c)


def test_latest_pending_replaces_older_and_is_sent_after_capacity_frees():
    c = FakeClient()
    t = module.LocoServoTransport(c, watchdog_s=10, max_outstanding=2)
    assert send(t, 1)
    eventually(lambda: len(c.items("move")) == 1)
    assert send(t, 2) and send(t, 3)
    c.items("move")[0]["event"].set()
    eventually(lambda: len(c.items("move")) == 2)
    assert c.items("move")[1]["args"][0] == 3
    t.close()
    eventually(lambda: len(c.items("stop")) == 1)
    release_all(c)
    eventually(lambda: all(r["consumed"] for r in c.calls))


def test_stop_failure_requires_successful_retry_and_old_stop_ack_is_ignored():
    c = FakeClient()
    t = module.LocoServoTransport(c, watchdog_s=10)
    t.pause()
    eventually(lambda: len(c.items("stop")) == 1)
    t.pause()
    c.items("stop")[0]["event"].set()
    eventually(lambda: len(c.items("stop")) == 2)
    assert t.status()["stop_status"] == "pending" and not t.resume()
    second = c.items("stop")[1]
    second["code"] = 7
    second["event"].set()
    eventually(lambda: t.status()["stop_status"] == "failed")
    assert not t.resume()
    t.stop()
    eventually(lambda: len(c.items("stop")) == 3)
    c.items("stop")[2]["event"].set()
    eventually(lambda: t.status()["stop_status"] == "ok")
    assert t.resume()
    t.close()
    eventually(lambda: len(c.items("stop")) == 4)
    release_all(c)


def test_blocked_sdk_write_does_not_block_pause_or_watchdog():
    c = FakeClient()
    c.write_gate = threading.Event()
    t = module.LocoServoTransport(c, watchdog_s=.04)
    assert send(t)
    eventually(lambda: t.status()["outstanding"] == 1)
    eventually(lambda: t.status()["paused"])
    assert t.status()["stop_status"] == "pending"
    assert t.pause()
    c.write_gate.set()
    eventually(lambda: len(c.items("stop")) == 1)
    release_all(c)
    eventually(lambda: t.status()["stop_status"] == "ok")
    t.close()
    eventually(lambda: len(c.items("stop")) == 2)
    release_all(c)


def test_congested_pending_commands_do_not_extend_sent_velocity_watchdog():
    c = FakeClient()
    t = module.LocoServoTransport(c, watchdog_s=.08, max_outstanding=2)
    assert send(t)
    eventually(lambda: len(c.items("move")) == 1)
    end = time.monotonic() + .25
    while time.monotonic() < end and not t.status()["paused"]:
        send(t)
        time.sleep(.01)
    eventually(lambda: len(c.items("stop")) == 1)
    assert len(c.items("move")) == 1
    release_all(c)
    eventually(lambda: t.status()["stop_status"] == "ok")
    t.close()
    eventually(lambda: len(c.items("stop")) == 2)
    release_all(c)


def test_close_stop_failure_can_be_retried():
    c = FakeClient()
    t = module.LocoServoTransport(c, watchdog_s=10)
    t.close()
    eventually(lambda: len(c.items("stop")) == 1)
    c.items("stop")[0]["code"] = 9
    c.items("stop")[0]["event"].set()
    eventually(lambda: t.status()["stop_status"] == "failed")
    assert not t.status()["closed"]
    t.close()
    eventually(lambda: len(c.items("stop")) == 2)
    c.items("stop")[1]["event"].set()
    eventually(lambda: t.status()["closed"])


def test_move_failure_requests_stop_and_invalid_inputs():
    c = FakeClient()
    t = module.LocoServoTransport(c, watchdog_s=10)
    with pytest.raises(ValueError):
        t.submit_velocity(float("nan"), 0, 0, 1, deadline=time.monotonic()+1)
    assert send(t)
    eventually(lambda: len(c.items("move")) == 1)
    c.items("move")[0]["code"] = 3
    c.items("move")[0]["event"].set()
    eventually(lambda: len(c.items("stop")) == 1)
    assert t.status()["paused"] and not send(t)
    release_all(c)
    eventually(lambda: t.status()["stop_status"] == "ok")
    t.close()
    eventually(lambda: len(c.items("stop")) == 2)
    release_all(c)
