import json
import math
import threading
import time
from types import SimpleNamespace
from unittest.mock import patch

import pytest

from bunker_servo import BunkerServo, build_descriptor
from common.control import parse_descriptor
from device import BunkerMiniDevice


class CAN:
    def __init__(self):
        self.targets = []
        self.stops = 0
        self.modes = 0
    def request_control_mode(self, *args):
        self.modes += 1
    def validate_target(self, vx, wz):
        return vx, wz
    def set_target(self, vx, wz, lease_s):
        self.targets.append((vx, wz, lease_s))
    def stop_motion(self):
        self.stops += 1
        return {"state": "stopped"}


def device():
    removed = []
    d = SimpleNamespace(config={}, _motion_lock=threading.Lock(),
        _motion_enabled=True, _active_move_id=None, _active_move_stop=None,
        _direct_until=0.0, can=CAN(), bridge=SimpleNamespace(
            bind_control=lambda topic, callback: (topic, callback),
            unbind_control=removed.append))
    d.servo = BunkerServo(d)
    d._run_move = lambda *args: None
    d.removed = removed
    return d


def command(values=None, **overrides):
    payload = {"schema": "motus.control/1", "mode": "twist", "dof": 6,
        "source": "navi", "seq": 1, "stamp_ms": int(time.time() * 1000),
        "ttl_ms": 300, "values": values or [0.3, 0, 0, 0, 0, 0.2]}
    payload.update(overrides)
    return SimpleNamespace(data=json.dumps(payload))


def start(d):
    return d.servo.dispatch("start", {"input_topic": "/navi/control"})


def test_descriptor_and_card_match_navi_si_twist():
    descriptor = parse_descriptor(build_descriptor({}))
    assert descriptor.upper == (1, 0, 0, 0, 0, math.radians(60))
    d = device()
    assert d.servo.get_tool()["topic_in"][0]["format"] == "control/velocity"
    assert d.servo.dispatch("info", {})["control_interface"]["mode"] == "twist"


def test_start_consume_pause_resume_stop():
    d = device()
    assert start(d)["state"] == "running"
    d.servo.on_command(command())
    assert d.can.targets == [(0.3, 0.2, 0.3)]
    d.servo.dispatch("pause", {})
    d.servo.on_command(command(seq=2))
    assert len(d.can.targets) == 1
    d.servo.dispatch("resume", {})
    d.servo.on_command(command(seq=3))
    assert len(d.can.targets) == 2
    assert d.servo.dispatch("stop", {}) == {"state": "idle"}
    assert len(d.removed) == 1
    d.servo.on_command(command(seq=4))
    assert len(d.can.targets) == 2


@pytest.mark.parametrize("message", [command(stamp_ms=1), command(dof=3),
    command([0.3, 0.1, 0, 0, 0, 0]), command([1.01, 0, 0, 0, 0, 0]),
    command(obs_stamp_ms=1), command([float("nan"), 0, 0, 0, 0, 0])])
def test_invalid_stale_or_unsupported_commands_never_reach_can(message):
    d = device()
    start(d)
    d.servo.on_command(message)
    assert d.can.targets == []


def test_watchdog_stops_and_can_target_has_independent_lease():
    d = device()
    start(d)
    now = [1000]
    d.servo.sink._clock = lambda: now[0]
    d.servo.on_command(command())
    stops = d.can.stops
    now[0] = 1301
    d.servo.tick()
    assert d.can.stops == stops + 1
    assert d.can.targets[-1][2] == 0.3


def test_duplicate_sequence_is_dropped_and_steps_are_limited():
    d = device()
    start(d)
    d.servo.on_command(command([0.1, 0, 0, 0, 0, 0.1]))
    d.servo.on_command(command([0.5, 0, 0, 0, 0, 0.8]))
    assert len(d.can.targets) == 1
    d.servo.on_command(command([0.5, 0, 0, 0, 0, 0.8], seq=2))
    assert d.can.targets[-1] == pytest.approx((0.2, 0.3, 0.3))


def test_malformed_json_or_can_failure_pauses_and_stops():
    d = device()
    start(d)
    d.servo.on_command(SimpleNamespace(data="invalid JSON"))
    assert d.servo.paused
    d.servo.dispatch("resume", {})
    d.can.set_target = lambda *args, **kwargs: (_ for _ in ()).throw(RuntimeError("fault"))
    d.servo.on_command(command())
    assert d.servo.paused
    assert d.servo.last_error == "RuntimeError"


def test_direct_move_pauses_servo_and_stream_cannot_replace_or_stop_direct_move():
    d = device()
    start(d)
    with patch("device.threading.Thread"):
        BunkerMiniDevice.dispatch(d, "move", {"vx": 0.1, "duration_s": 1})
    assert d.servo.paused
    stops = d.can.stops
    d.servo.on_command(command())
    d.servo.dispatch("pause", {})
    assert d.can.stops == stops
    assert d.can.targets == []
    with pytest.raises(RuntimeError, match="base_drive"):
        d.servo.dispatch("resume", {})
    d.servo.dispatch("stop", {})
    assert d.can.stops == stops


def test_disabled_motion_and_missing_topic_have_no_can_side_effects():
    d = device()
    with pytest.raises(ValueError):
        d.servo.dispatch("start", {})
    assert d.can.modes == 0
    d._motion_enabled = False
    with pytest.raises(RuntimeError):
        start(d)
    assert d.can.modes == 0


def test_binding_failure_never_requests_can_mode_and_can_failure_unbinds():
    d = device()
    with patch.object(d.bridge, "bind_control", side_effect=ValueError("invalid topic")):
        with pytest.raises(ValueError):
            start(d)
    assert d.can.modes == 0
    with patch.object(d.can, "request_control_mode", side_effect=RuntimeError("stale state")):
        with pytest.raises(RuntimeError):
            start(d)
    assert len(d.removed) == 1
    assert not d.servo.running
