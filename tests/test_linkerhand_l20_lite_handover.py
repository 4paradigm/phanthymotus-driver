"""Explicit hand commands must stop the streaming owner before taking the bus."""

from pathlib import Path
import sys
import threading
import time

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / "linkerhand/l20_lite")]
from modbus_device import build_plugins


def plugins():
    config = yaml.safe_load((ROOT / "linkerhand/l20_lite/config.yaml").read_text())
    plugins_by_name = {plugin.PREFIX: plugin for plugin in build_plugins(config, "test", None)}
    return [plugins_by_name[name] for name in ("hand", "hand_gesture", "glove_teleop")]


def test_hand_motion_waits_for_teleop_disarm_before_dispatch(monkeypatch):
    hand, gesture, glove = plugins()
    order = []
    hand._teleop_active = True

    def disarm():
        order.append("stream_cleanup_finished")
        hand._teleop_active = False

    def dispatch(action, args):
        assert not hand._teleop_active
        order.append(action)
        return {"state": "accepted"}

    monkeypatch.setattr(glove, "_disarm", disarm)
    monkeypatch.setattr(hand._legacy, "dispatch", dispatch)
    hand.dispatch("set_positions", {"thumb_flex": 200, "confirm_motion": True})
    assert order == ["stream_cleanup_finished", "set_positions"]


def test_hand_and_gesture_stop_release_teleop_without_opening_uart(monkeypatch):
    hand, gesture, glove = plugins()
    cleanups = []

    def disarm():
        cleanups.append(True)
        hand._teleop_active = False

    monkeypatch.setattr(glove, "_disarm", disarm)
    for plugin in (hand, gesture):
        hand._teleop_active = True
        plugin.dispatch("stop", {})
        assert not hand._teleop_active
    assert len(cleanups) == 2
    assert hand._legacy.connection._client is None


class ReleaseClient:
    def __init__(self, fail=True):
        self.fail = fail
        self.attempts = 0
        self.closed = False

    def set_torques(self, values):
        assert values == [0] * 10
        self.attempts += 1
        if self.fail:
            raise TimeoutError('zero torque was not acknowledged')

    def close(self):
        self.closed = True


def test_failed_release_latches_handover_and_explicit_retry_recovers(monkeypatch):
    hand, gesture, glove = plugins()
    monkeypatch.setenv('GLOVE_PREVIEW_ONLY', '0')
    client = ReleaseClient()
    glove._clients['left'] = client
    glove._armed = hand._teleop_active = True
    with pytest.raises(RuntimeError, match='handover is locked'):
        glove._disarm()
    assert not glove._armed and hand._teleop_active
    assert glove._pending_disarm and not glove._cleanup_done.is_set()
    assert not client.closed and glove._clients['left'] is client
    assert 'left' in glove.dispatch('info', {})['release_failures']
    before = client.attempts
    glove._finish_disarm()  # Background polling cannot silently clear the latch.
    assert client.attempts == before
    with pytest.raises(RuntimeError, match='cleanup is still pending'):
        glove._reserve_arm()
    admitted = []
    monkeypatch.setattr(hand._legacy, 'dispatch', lambda *args: admitted.append(args))
    with pytest.raises(RuntimeError, match='handover is locked'):
        hand.dispatch('set_positions', {'thumb_flex': 200, 'confirm_motion': True})
    with pytest.raises(RuntimeError, match='handover is locked'):
        gesture.dispatch('number', {'number': 1, 'confirm_motion': True})
    assert not admitted and not client.closed and hand._teleop_active
    client.fail = False
    before = client.attempts
    with pytest.raises(RuntimeError, match='explicitly retry glove disarm'):
        hand.dispatch('set_positions', {'thumb_flex': 200, 'confirm_motion': True})
    with pytest.raises(RuntimeError, match='explicitly retry glove disarm'):
        gesture.dispatch('number', {'number': 1, 'confirm_motion': True})
    assert client.attempts == before  # Recovery never occurs as a side effect of handover.
    glove.dispatch('disarm', {})
    assert client.closed and not glove._clients and not glove._release_failures
    assert glove._cleanup_done.is_set() and not hand._teleop_active
    hand.dispatch('set_positions', {'thumb_flex': 200, 'confirm_motion': True})
    assert len(admitted) == 1


def test_partial_dual_hand_release_retains_only_failed_side():
    hand, _, glove = plugins()
    left, right = ReleaseClient(False), ReleaseClient(True)
    glove._clients.update(left=left, right=right)
    hand._teleop_active = True
    with pytest.raises(RuntimeError, match='handover is locked'):
        glove._disarm()
    assert left.closed and not right.closed
    assert set(glove._clients) == {'right'}
    right.fail = False
    glove.dispatch('disarm', {})
    assert left.attempts == 1 and right.attempts == 2
    assert glove._cleanup_done.is_set()


def test_background_release_failure_wakes_disarm_caller_without_false_success():
    hand, _, glove = plugins()
    client = ReleaseClient()
    glove._clients['left'] = client
    hand._teleop_active = True
    with glove._lock:
        glove._ensure_control_threads_locked()
    try:
        with pytest.raises(RuntimeError, match='handover is locked'):
            glove._disarm()
        assert glove._release_attempt_done.is_set()
        assert not glove._cleanup_done.is_set()
        assert hand._teleop_active and not client.closed
    finally:
        client.fail = False
        glove.dispatch('stop', {})


@pytest.mark.parametrize('kind', ['hand', 'gesture'])
def test_arm_cannot_overtake_manual_request_before_worker_reservation(monkeypatch, kind):
    hand, gesture, glove = plugins()
    monkeypatch.setenv('GLOVE_PREVIEW_ONLY', '0')
    entered = threading.Event()
    resume = threading.Event()
    arm_attempted = threading.Event()
    arm_done = threading.Event()
    worker_done = threading.Event()
    errors, results = [], []

    class ObservedLock:
        def __init__(self):
            self.lock = threading.RLock()

        def __enter__(self):
            if threading.current_thread().name == 'arm-contender':
                arm_attempted.set()
            self.lock.acquire()
            return self

        def __exit__(self, *args):
            self.lock.release()

    hand._ownership_lock = ObservedLock()
    glove._thread = object()
    glove._last_frame = {'timestampMs': 1}
    glove._last_received = time.monotonic()
    glove.watchdog_s = 10  # This case tests admission, not source expiry.
    if kind == 'hand':
        def pause_dispatch(action, args):
            entered.set()
            assert resume.wait(2)
            hand._legacy._active = {'action_id': 'manual-test'}
            return {'state': 'running'}
        monkeypatch.setattr(hand._legacy, 'dispatch', pause_dispatch)
        manual = lambda: hand.dispatch('set_positions', {'thumb_flex': 200, 'confirm_motion': True})
    else:
        prepare = gesture._prepare
        def pause_prepare(args, action):
            entered.set()
            assert resume.wait(2)
            return prepare(args, action)
        monkeypatch.setattr(gesture, '_prepare', pause_prepare)
        monkeypatch.setattr(hand, '_discover', lambda: {'right': '/dev/fake'})
        monkeypatch.setattr(gesture, '_execute', lambda active: worker_done.wait(3))
        manual = lambda: gesture.dispatch('number', {'number': 1, 'confirm_motion': True})

    def request_manual():
        try:
            results.append(manual())
        except Exception as exc:
            errors.append(exc)

    def request_arm():
        try:
            results.append(glove._reserve_arm())
        except Exception as exc:
            errors.append(exc)
        finally:
            arm_done.set()

    manual_thread = threading.Thread(target=request_manual)
    arm_thread = threading.Thread(target=request_arm, name='arm-contender')
    try:
        manual_thread.start()
        assert entered.wait(2)
        arm_thread.start()
        assert arm_attempted.wait(2)
        assert not arm_done.is_set()
        resume.set()
        manual_thread.join(2)
        arm_thread.join(2)
        assert not manual_thread.is_alive() and not arm_thread.is_alive()
        assert len(results) == 1 and results[0]['state'] == 'running'
        assert len(errors) == 1 and 'is still running' in str(errors[0])
        assert not glove._arming and not hand._teleop_active
    finally:
        resume.set()
        worker_done.set()
        manual_thread.join(2)
        if arm_thread.ident is not None:
            arm_thread.join(2)
        glove._thread = None


def test_old_cleanup_finalizer_does_not_clear_new_arm_reservation():
    hand, _, glove = plugins()
    hand._teleop_active = True
    glove._arming = True
    glove._finish_disarm()
    assert hand._teleop_active and glove._arming


@pytest.mark.parametrize('control_workers', [False, True])
def test_failed_stop_can_retry_disarm_and_restart_without_driver_restart(monkeypatch, control_workers):
    hand, _, glove = plugins()
    client = ReleaseClient()
    publishers = []

    class Publisher:
        def __init__(self):
            self.closed = False
            publishers.append(self)

        def close(self):
            self.closed = True

    monkeypatch.setattr('glove_teleop_device.os.path.exists', lambda _: True)
    monkeypatch.setattr(glove, '_publisher_factory', lambda _: Publisher())
    monkeypatch.setattr(glove, '_direct_loop', lambda: glove._shutdown.wait())
    glove.dispatch('start', {})
    old_receiver = glove._thread
    glove._clients['left'] = client
    glove._armed = hand._teleop_active = True
    if control_workers:
        with glove._lock:
            glove._ensure_control_threads_locked()
    try:
        with pytest.raises(RuntimeError, match='handover is locked'):
            glove.dispatch('stop', {})
        assert not old_receiver.is_alive() and publishers[0].closed
        assert glove._control_thread is None and glove._watchdog_thread is None
        assert not glove._stopping
        assert hand._teleop_active and glove._pending_disarm
        assert not glove._cleanup_done.is_set() and not client.closed
        with pytest.raises(RuntimeError, match='cleanup'):
            glove.dispatch('start', {})
        client.fail = False
        glove.dispatch('disarm', {})
        assert not hand._teleop_active and glove._cleanup_done.is_set()
        assert glove.dispatch('start', {})['state'] == 'running'
        assert glove._thread is not old_receiver and glove._thread.is_alive()
        assert len(publishers) == 2 and not publishers[1].closed
        assert not glove._armed
    finally:
        client.fail = False
        glove.dispatch('stop', {})
