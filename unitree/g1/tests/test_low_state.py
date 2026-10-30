"""Shared source ownership, clock continuity and live safety checks; no hardware."""
from pathlib import Path
from queue import Queue, Empty
import sys
import threading
import time
from types import SimpleNamespace

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from low_state import LowStateSource
from arm_stream import ArmStreamExecutor


def message(tick, q=0.):
    return SimpleNamespace(tick=tick, motor_state=[SimpleNamespace(
        q=q, dq=0., mode=1, motorstate=0, vol=48., temperature=[20, 20]) for _ in range(29)])


@pytest.fixture
def channels(monkeypatch):
    opened = []
    class Subscriber:
        fail = False
        close_fail = None
        def __init__(self, topic, dtype):
            self.topic, self.handler, self.closed = topic, None, 0
            self.messages = Queue()
            self.read_threads = []
            opened.append(self)
        def Init(self, handler=None, queueLen=0, *, qos=None):
            self.handler = handler
            if self.fail is True or self.fail == self.topic: raise RuntimeError('reader_init_failed')
        def Read(self, timeout=None):
            self.read_threads.append(threading.current_thread())
            try: item = self.messages.get(timeout=timeout)
            except Empty: return None
            if isinstance(item, Exception): raise item
            return item
        def Close(self):
            self.closed += 1
            assert all(not thread.is_alive() for thread in self.read_threads)
            if self.topic == 'rt/lowstate' and self.handler: self.handler(message(9999))
            if self.close_fail == self.topic: raise RuntimeError('reader_close_failed')
    import sdk_adapter
    monkeypatch.setattr(sdk_adapter, 'FeedbackSubscriber', Subscriber)
    monkeypatch.setitem(sys.modules, 'unitree_sdk2py.idl.unitree_hg.msg.dds_', SimpleNamespace(LowState_=object, LowCmd_=object))
    monkeypatch.setitem(sys.modules, 'cyclonedds.qos', SimpleNamespace(
        Qos=lambda *args: args, Policy=SimpleNamespace(History=SimpleNamespace(KeepLast=lambda n:n))))
    monkeypatch.setitem(sys.modules, 'cyclonedds.internal', SimpleNamespace(InvalidSample=type('InvalidSample', (), {})))
    monkeypatch.setitem(sys.modules, 'sport_mode_state', SimpleNamespace(SportModeState_=object))
    return opened, Subscriber


def test_dedicated_reader_delivers_and_failure_invalidates_cache(channels):
    opened, _ = channels
    source = LowStateSource(); source.start()
    try:
        assert opened[0].handler is None
        opened[0].messages.put(message(100))
        deadline = time.monotonic()+1
        while source.snapshot() is None and time.monotonic() < deadline: time.sleep(.001)
        assert source.snapshot().source_tick == 100
        assert opened[0].read_threads[0] is source._thread
        opened[0].messages.put(RuntimeError('read_failed'))
        source._thread.join(timeout=1)
        assert not source._thread.is_alive() and source.snapshot() is None
    finally:
        source.stop()


def test_thread_start_failure_closes_reader(channels, monkeypatch):
    opened, _ = channels
    def fail(_): raise RuntimeError('thread_start_failed')
    monkeypatch.setattr(threading.Thread, 'start', fail)
    source = LowStateSource()
    with pytest.raises(RuntimeError, match='thread_start_failed'): source.start()
    assert source.snapshot() is None and source._subscriber is None
    assert opened[0].closed == 1


def test_shared_clock_does_not_renew_on_reads_late_delivery_or_duplicate(channels):
    now = [1_000_000_000]
    source = LowStateSource(clock=lambda:now[0]); source.start()
    source._on_state(message(1000)); first = source.snapshot()
    now[0] += 20_000_000_000
    assert source.snapshot() is first
    source._on_state(message(1020))
    sample = source.snapshot()
    assert sample.sampled_ns == first.sampled_ns+20_000_000
    assert sample.received_ns == now[0]
    source._on_state(message(1020, 1.)); source._on_state(message(1019, 2.))
    assert source.snapshot() is sample
    source.stop(); assert source.snapshot() is None


def test_wrap_and_restart_invalidate_old_cache_and_close_once(channels):
    opened, _ = channels
    now = [1_000_000_000]
    source = LowStateSource(clock=lambda:now[0]); source.start(); source.start()
    assert len(opened) == 1
    source._on_state(message(0xfffffff0)); first = source.snapshot()
    now[0] += 32_000_000; source._on_state(message(16))
    assert source.snapshot().sampled_ns == first.sampled_ns+32_000_000
    source.stop(); source.stop()
    assert opened[0].closed == 1 and source.snapshot() is None
    source.start(); assert source.snapshot() is None
    now[0] += 1_000_000; source._on_state(message(1))
    assert source.snapshot().sampled_ns == now[0] and source.snapshot().source_tick == 1
    source.stop()


def test_reader_failure_and_malformed_tick_never_publish_old_sample(channels):
    opened, Subscriber = channels
    Subscriber.fail = True
    source = LowStateSource()
    with pytest.raises(RuntimeError, match='reader_init_failed'): source.start()
    assert source.snapshot() is None and opened[0].closed == 1
    Subscriber.fail = False; source.start(); source._on_state(message(1))
    source._on_state(SimpleNamespace(tick=None))
    assert source.snapshot() is None
    source.stop()


def test_shared_arm_preserves_fsm_guard_and_source_age(channels):
    now = [1_000_000_000]
    source = LowStateSource(clock=lambda:now[0]); source.start(); source._on_state(message(1000))
    arm = ArmStreamExecutor({}, 'test', None, low_state=source, clock=lambda:now[0])
    arm.profile = {'safety': {'allowed_fsm_ids': [500]}}
    with pytest.raises(ValueError, match='motion_feedback_stale'): arm._fresh()
    arm._fsm = {'arm_ns':now[0], 'fsm_id':801}
    with pytest.raises(ValueError, match='motion_mode_unavailable'): arm._fresh()
    arm._fsm['fsm_id'] = 500
    result = arm._fresh()[0]
    result['q'][0] = 9.; result['motor_telemetry'][0]['temperature'][0] = 99
    assert arm._fresh()[1] == [0.]*10 and source.snapshot().message.motor_state[15].temperature == [20,20]
    now[0] += 201_000_000; arm._fsm['arm_ns'] = now[0]
    with pytest.raises(ValueError, match='arm_feedback_stale'): arm._fresh()
    source.stop()


def test_shared_arm_does_not_own_source_or_skip_other_observers(channels):
    opened, _ = channels
    source = LowStateSource(); source.start()
    arm = ArmStreamExecutor({}, 'test', None, low_state=source)
    arm._run = lambda: None
    arm.start(); arm.start()
    assert [s.topic for s in opened] == ['rt/lowstate', 'rt/sportmodestate', 'rt/arm_sdk']
    assert arm.stop()['authority_released']
    assert opened[0].closed == 0 and all(s.closed == 1 for s in opened[1:])
    source.stop(); assert opened[0].closed == 1
    with pytest.raises(ValueError, match='shared_lowstate_requires_live_safety_checks'):
        ArmStreamExecutor({}, 'test', None, low_state=source, snapshot=lambda:{})


@pytest.mark.parametrize('topic', ['rt/sportmodestate', 'rt/arm_sdk'])
def test_failed_observer_start_retries_all_safety_subscriptions(channels, topic):
    opened, Subscriber = channels
    source = LowStateSource(); source.start()
    arm = ArmStreamExecutor({}, 'test', None, low_state=source); arm._run = lambda:None
    Subscriber.fail = topic
    with pytest.raises(RuntimeError, match='reader_init_failed'): arm.start()
    assert arm._thread is None and arm._fsm_subscriber is None and arm._arm_observer is None
    assert opened[0].closed == 0 and all(sub.closed == 1 for sub in opened[1:])
    Subscriber.fail = False; arm.start()
    assert arm._fsm_subscriber is not None and arm._arm_observer is not None
    assert arm.stop()['authority_released']; source.stop()


def test_partial_close_failure_does_not_mask_init_or_skip_remaining_cleanup(channels, caplog, capfd):
    opened, Subscriber = channels
    source = LowStateSource(); source.start()
    arm = ArmStreamExecutor({}, 'test', None, low_state=source); arm._run = lambda:None
    Subscriber.fail = 'rt/arm_sdk'; Subscriber.close_fail = 'rt/sportmodestate'
    with pytest.raises(RuntimeError, match='reader_init_failed'): arm.start()
    assert arm._fsm_subscriber is None and arm._arm_observer is None
    assert all(sub.closed == 1 for sub in opened[1:])
    assert 'reader_close_failed' in caplog.text+capfd.readouterr().err
    Subscriber.fail = False; Subscriber.close_fail = None
    arm.start(); assert arm._fsm_subscriber is not None and arm._arm_observer is not None
    arm.stop(); source.stop()
    Subscriber.fail = 'rt/lowstate'; Subscriber.close_fail = 'rt/lowstate'
    with pytest.raises(RuntimeError, match='reader_init_failed'): source.start()
    assert source.snapshot() is None and source._subscriber is None


def test_gaps_fence_previews_when_feedback_is_dropped_not_when_it_resumes():
    now = [1_000_000_000]
    source = LowStateSource(clock=lambda: now[0])
    source._active = True
    source._on_state(message(1)); assert source.gaps == 0
    now[0] += 2_000_000; source._on_state(message(3)); assert source.gaps == 0
    now[0] += 150_000_000; source._on_state(message(4)); assert source.gaps == 1
    # Counted at the bad sample itself: no later valid packet is needed.
    source._on_state(SimpleNamespace(tick='bad'))
    assert source.snapshot() is None and source.gaps == 2
    source._on_state(SimpleNamespace(tick='bad')); assert source.gaps == 2
    now[0] += 2_000_000; source._on_state(message(5)); assert source.gaps == 2
    source.stop(); assert source.gaps == 3
    source._active = True
    now[0] += 2_000_000; source._on_state(message(6)); assert source.gaps == 3


def test_reader_failure_fences_previews_immediately(channels):
    opened, _ = channels
    source = LowStateSource(); source.start()
    try:
        opened[0].messages.put(message(1))
        deadline = time.monotonic()+2
        while source.snapshot() is None and time.monotonic() < deadline: time.sleep(.005)
        assert source.snapshot() is not None and source.gaps == 0
        opened[0].messages.put(RuntimeError('reader broke'))
        while source.snapshot() is not None and time.monotonic() < deadline: time.sleep(.005)
        assert source.snapshot() is None and source.gaps == 1
    finally:
        source.stop()


def test_arm_feedback_epoch_includes_shared_source_gaps():
    source = LowStateSource()
    arm = ArmStreamExecutor({}, 'test', None, low_state=source)
    epoch = arm.feedback_epoch
    source.gaps += 1
    assert arm.feedback_epoch == epoch + 1
