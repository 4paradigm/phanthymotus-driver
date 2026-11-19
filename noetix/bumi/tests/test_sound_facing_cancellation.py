"""Cancellation signals remain independent of a blocked source-health query."""

import json
import os
from pathlib import Path
import sys
import threading
import time
import traceback
import types

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from sound_facing import SoundFacing, SourceHealthProbe
from sound_authorization_fixtures import approved_authority, signed
from test_sound_facing import Clock, Motion, feed
from test_sound_facing_card import ros as _ros_fixture
from sound_facing_card import SoundFacingPlugin

ros = _ros_fixture  # Make the shared ROS-free pytest fixture visible here.


def test_completed_health_result_belongs_to_its_waiter(monkeypatch):
    """A second query must not erase a successful first caller's result."""
    first_read_done, release_first = threading.Event(), threading.Event()
    second_entered, release_second = threading.Event(), threading.Event()
    calls = []
    real_wait = threading.Event.wait

    def health():
        calls.append(True)
        if len(calls) == 2:
            second_entered.set()
            real_wait(release_second, 2)
        return True

    def wait(event, timeout=None):
        result = real_wait(event, timeout)
        if threading.current_thread().name == "first_health_reader" and timeout == .05:
            first_read_done.set()
            real_wait(release_first, 2)
        return result

    monkeypatch.setattr(threading.Event, "wait", wait)
    probe, results = SourceHealthProbe(health), {}
    first = threading.Thread(name="first_health_reader",
                             target=lambda: results.update(first=probe.read()))
    second = threading.Thread(target=lambda: results.update(second=probe.read()))
    try:
        first.start()
        assert real_wait(first_read_done, 1)
        probe.worker.join(1)  # first callback completed; its reader is paused
        second.start()
        assert real_wait(second_entered, 1)
        release_first.set()
        first.join(.2)
        second.join(.2)
        assert results == {"first": True, "second": False}
    finally:
        release_first.set(); release_second.set()
        first.join(1)
        if second.ident is not None:second.join(1)
        if probe.worker:probe.worker.join(1)


class RecordingMotion(Motion):
    def __init__(self):
        super().__init__()
        self.cancel_received = threading.Event()
        self.cancel_time = None

    def interrupt_owned_session(self, owner):
        result = super().interrupt_owned_session(owner)
        if self.owner is owner:
            if self.cancel_time is None:
                self.cancel_time = time.perf_counter()
            self.cancel_received.set()
        return result


def record(label, data):
    if os.getenv("SOUND_TEST_EVIDENCE"):
        root = Path(os.environ["SOUND_TEST_EVIDENCE"])
        root.mkdir(parents=True, exist_ok=True)
        (root / (label + ".json")).write_text(json.dumps(data, indent=2))


@pytest.mark.parametrize("action", ["disable", "stop_signal", "session_expiry", "authorization_expiry",
                                  "authorization_revocation", "repeated_cancel", "renew_revoke"])
def test_blocked_source_does_not_block_safety(action):
    clock, motion = Clock(), RecordingMotion()
    block, entered, release = threading.Event(), threading.Event(), threading.Event()
    def health():
        if block.is_set():
            entered.set(); release.wait(5)
        return True
    authority = approved_authority(clock)
    engine = SoundFacing(motion, monotonic=clock.mono, wall_ns=clock.wall,
                         source_health=health, authority=authority)
    engine.start(); session = engine.enable()["session_id"]; feed(clock, engine)
    assert engine.action_id and len(motion.calls) == 1
    block.set()
    caller = threading.Thread(target=lambda: engine.renew(session) if action == "renew_revoke" else engine.tick())
    caller.start(); assert entered.wait(1)
    started = time.perf_counter()
    try:
        if action == "session_expiry":
            clock.advance(31); engine.tick()
        elif action in ("authorization_expiry", "authorization_revocation"):
            if action == "authorization_expiry":clock.advance(301)
            else:authority.invalidate()
            engine.tick()
        elif action == "stop_signal":
            engine.interrupt(); engine.disable()
        else:
            for _ in range(5 if action == "repeated_cancel" else 1):engine.disable()
        assert motion.cancel_received.wait(0.2)
        latency = motion.cancel_time - started
        assert latency < 0.2
        caller.join(0.3)
        if caller.is_alive():
            frame = sys._current_frames().get(caller.ident)
            stack = "".join(traceback.format_stack(frame)) if frame else "no live frame"
            pytest.fail("background caller still active after cancellation; " + stack)
        assert engine.session_id is None
        count = len(motion.calls)
    finally:
        release.set(); caller.join(1)
        if engine._source_probe.worker:engine._source_probe.worker.join(1)
    assert len(motion.calls) == count == 1
    assert engine.renew(session)["error"] == "session_not_active_or_mismatched"
    record(action, {"cancel_signal_latency_s": latency, "controller_ack": True,
                   "final_state": engine.state, "enabled": engine.session_id is not None,
                   "new_turn_after_stop": False, "deadlock": False,
                   "controller": "RecordingMotion double", "physical_stop_verified": False})


def test_late_enable_query_cannot_reactivate_cancelled_intent():
    clock, motion = Clock(), RecordingMotion()
    entered, release = threading.Event(), threading.Event()
    def health():entered.set(); release.wait(3); return True
    authority = approved_authority(clock)
    engine = SoundFacing(motion, monotonic=clock.mono, wall_ns=clock.wall,
                         source_health=health, authority=authority)
    engine.start(); old = signed(authority)
    result = {}
    caller = threading.Thread(target=lambda: result.update(engine.enable()))
    caller.start(); assert entered.wait(1)
    engine.disable(); caller.join(.3)
    release.set(); engine._source_probe.worker.join(1)
    assert not caller.is_alive() and result["error"] == "enable_cancelled"
    assert engine.session_id is None and motion.owner is None and not motion.calls
    assert engine.authorize(old)["error"] == "authorization_intent_mismatch"


def test_one_blocked_probe_does_not_spawn_unbounded_workers():
    entered, release = threading.Event(), threading.Event()
    def health():entered.set(); release.wait(3); return True
    engine = SoundFacing(Motion(), source_health=health)
    try:
        assert engine._source_probe.read() is False
        original = engine._source_probe.worker
        for _ in range(4):assert engine._source_probe.read() is False
        assert engine._source_probe.worker is original
        engine._source_probe.invalidate()
    finally:
        release.set(); original.join(1)
    assert engine._source_probe.result is None


@pytest.mark.parametrize("action", ["stop", "disable_follow", "cancel"])
def test_actual_card_safety_actions_signal_before_source_or_lifecycle_waits(ros, action):
    clock, motion = Clock(), RecordingMotion()
    block, entered, release = threading.Event(), threading.Event(), threading.Event()
    source_lock = threading.RLock()
    def info(*_):
        if block.is_set():
            entered.set()
            with source_lock:pass
        return {"state": "running", "calibrated_directions": ["front", "right"]}
    card = SoundFacingPlugin({}, "sim", ros, motion, source_plugin=types.SimpleNamespace(dispatch=info),
                             authority=approved_authority(clock))
    card._engine = SoundFacing(motion, {"settle_s": .5}, monotonic=clock.mono,
                               wall_ns=clock.wall, source_health=card._source_healthy,
                               authority=approved_authority(clock))
    card.start();card._engine.enable();feed(clock, card._engine)
    holder = threading.Thread(target=lambda: _hold(source_lock, release))
    holder.start()
    source_lock_owned = time.monotonic() + 1
    while source_lock.acquire(blocking=False):
        source_lock.release()
        assert time.monotonic() < source_lock_owned
        time.sleep(.001)
    block.set()
    assert entered.wait(1)
    result = {}
    started = time.perf_counter()
    stopper = threading.Thread(target=lambda: result.update(card.dispatch(action, {})))
    stopper.start()
    try:
        assert motion.cancel_received.wait(.2)
        latency = motion.cancel_time - started
        assert latency < .2
        motion.finish("stopped")  # the fake controller acknowledges its terminal
        deadline = time.monotonic() + 2
        while stopper.is_alive() and time.monotonic() < deadline:
            clock.advance(.1)
            time.sleep(.01)
        stopper.join(.2)
        assert not stopper.is_alive()
        assert card._engine.session_id is None
    finally:
        release.set();holder.join(1)
        if card._engine._source_probe.worker:card._engine._source_probe.worker.join(1)
        for _ in range(8):clock.advance(.1);card._engine.tick()
        card.stop()
    assert len(motion.calls) == 1
    record("card_" + action, {"cancel_signal_latency_s": latency, "controller_ack": True,
                              "enabled": False, "deadlock": False,
                              "new_turn_after_stop": False, "physical_stop_verified": False})


def _hold(lock, release):
    with lock:release.wait(4)


def test_retired_source_result_cannot_validate_a_replacement_authorization():
    clock, motion = Clock(), Motion()
    entered, release = threading.Event(), threading.Event()
    def health():entered.set();release.wait(2);return True
    engine = SoundFacing(motion, monotonic=clock.mono, wall_ns=clock.wall,
                         authority=approved_authority(clock), source_health=health)
    engine.start()
    pending = threading.Thread(target=engine.enable);pending.start();assert entered.wait(1)
    engine.disable()
    engine.prepare_authorization()
    engine.authorize(signed(engine.authority))
    release.set();pending.join(1);engine._source_probe.worker.join(1)
    assert not engine.session_id and not motion.calls
    assert engine._source_probe.result is None
