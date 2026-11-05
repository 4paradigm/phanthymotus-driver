"""Driver-owned DDS extension boundary; no robot or SDK participant created."""
from pathlib import Path
from queue import Queue, Empty
import sys
import threading
from types import SimpleNamespace

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from sdk_adapter import FeedbackSubscriber


@pytest.fixture
def factory(monkeypatch):
    opened = []
    class InvalidSample: pass
    class Channel:
        fail = False
        close_fail = False
        def __init__(self, topic, dtype):
            self.topic, self.dtype = topic, dtype
            self.messages = Queue()
            self.closed = 0
            self.threads = []
            self.timeouts = []
            opened.append(self)
        def SetReader(self, qos, handler=None, queueLen=0):
            assert qos == ('keep-last', 1)
            assert handler is None and queueLen == 0
            if self.fail: raise RuntimeError('init_failed')
        def Read(self, timeout):
            assert not self.closed
            assert 0 < timeout <= .1
            self.threads.append(threading.current_thread())
            self.timeouts.append(timeout)
            try: value = self.messages.get(timeout=timeout)
            except Empty: return None
            if isinstance(value, Exception): raise value
            return value
        def CloseReader(self):
            self.closed += 1
            assert self.closed == 1, 'SDK close is not idempotent'
            assert all(not t.is_alive() for t in self.threads if t is not threading.current_thread())
            if self.close_fail: raise RuntimeError('close_failed')
    monkeypatch.setitem(sys.modules, 'unitree_sdk2py.core.channel', SimpleNamespace(
        ChannelFactory=lambda: SimpleNamespace(CreateChannel=Channel)))
    monkeypatch.setitem(sys.modules, 'cyclonedds.qos', SimpleNamespace(
        Qos=lambda policy: policy, Policy=SimpleNamespace(History=SimpleNamespace(
            KeepLast=lambda count: ('keep-last', count)))))
    monkeypatch.setitem(sys.modules, 'cyclonedds.internal', SimpleNamespace(InvalidSample=InvalidSample))
    return opened, Channel, InvalidSample


def test_public_channel_qos_empty_invalid_and_restart(factory):
    opened, _, invalid = factory
    sub = FeedbackSubscriber('rt/lowstate', object)
    sub.Init(); sub.Init()
    assert len(opened) == 1 and sub._thread is None
    channel = opened[0]
    assert sub.Read(.001) is None
    channel.messages.put(invalid())
    assert sub.Read(.001) is None
    sample = object(); channel.messages.put(sample)
    assert sub.Read(.001) is sample
    for timeout in (None, 0, -1, .2, float('nan')):
        with pytest.raises(ValueError): sub.Read(timeout)
    sub.Close(); sub.Close()
    assert channel.closed == 1 and sub.Read() is None
    sub.Init()
    assert len(opened) == 2 and sub.Read(.001) is None
    sub.Close()


def test_callback_close_waits_for_inflight_handler_and_drops_pending(factory):
    opened, _, invalid = factory
    entered, release, closed = threading.Event(), threading.Event(), threading.Event()
    values = []
    def handler(sample):
        values.append(sample)
        entered.set()
        assert release.wait(2)
    sub = FeedbackSubscriber('rt/sportmodestate', object)
    sub.Init(handler)
    channel, worker = opened[0], sub._thread
    channel.messages.put(None); channel.messages.put(invalid()); channel.messages.put('first')
    assert entered.wait(1)
    channel.messages.put('pending')
    def close():
        sub.Close(); closed.set()
    closer = threading.Thread(target=close)
    closer.start()
    try:
        assert sub._closed.wait(1)
        assert not closed.is_set() and channel.closed == 0
    finally:
        release.set(); closer.join(2)
    assert closed.is_set() and not worker.is_alive()
    assert values == ['first'] and channel.closed == 1
    assert all(t is worker for t in channel.threads)
    sub.Close()
    sub.Init(lambda sample: values.append(sample))
    sub.Close()
    assert values == ['first']  # The old channel's pending sample is never replayed.


def test_close_interrupts_idle_reader_and_releases_once(factory):
    opened, _, _ = factory
    sub = FeedbackSubscriber('rt/sportmodestate', object)
    sub.Init(lambda _: pytest.fail('no valid sample'))
    worker = sub._thread
    sub.Close(); sub.Close()
    assert not worker.is_alive() and opened[0].closed == 1


@pytest.mark.parametrize('stage', ['reader', 'thread'])
def test_partial_start_failure_preserves_error_and_can_retry(factory, monkeypatch, stage):
    opened, channel_type, _ = factory
    sub = FeedbackSubscriber('rt/sportmodestate', object)
    original_start = threading.Thread.start
    if stage == 'reader': channel_type.fail = True
    else:
        def fail(_): raise RuntimeError('thread_failed')
        monkeypatch.setattr(threading.Thread, 'start', fail)
    channel_type.close_fail = True
    with pytest.raises(RuntimeError, match='init_failed' if stage == 'reader' else 'thread_failed'):
        sub.Init(lambda _: None)
    assert opened[0].closed == 1 and sub._channel is None and sub._thread is None
    sub.Close()
    channel_type.fail = channel_type.close_fail = False
    monkeypatch.setattr(threading.Thread, 'start', original_start)
    sub.Init(lambda _: None); sub.Close()
    assert opened[1].closed == 1


def test_empty_read_logging_is_scoped_not_a_global_error_mute(factory, caplog, request, monkeypatch):
    import logging
    opened, _, _ = factory
    sub = FeedbackSubscriber('rt/lowstate', object); sub.Init()
    logger = logging.getLogger('unitree_sdk2py.core.channel')
    # ROS/other collected modules may disable ancestor propagation. Isolate this
    # test's capture at the logger; restore its propagation setting afterwards.
    monkeypatch.setattr(logger, 'propagate', False)
    logger.addHandler(caplog.handler)
    request.addfinalizer(lambda: logger.removeHandler(caplog.handler))
    def report(error, text):
        try: raise error
        except Exception: logger.warning(text, exc_info=True)
    def read(_):
        report(StopIteration(), 'owned empty read')
        report(RuntimeError('DDS failed'), 'real read failure')
        thread = threading.Thread(target=report, args=(StopIteration(), 'other SDK thread'))
        thread.start(); thread.join()
        return None
    opened[0].Read = read
    assert sub.Read() is None
    report(StopIteration(), 'outside owned read')
    assert [r.message for r in caplog.records] == [
        'real read failure', 'other SDK thread', 'outside owned read']
    def fail(_): raise RuntimeError('escaped read failure')
    opened[0].Read = fail
    with pytest.raises(RuntimeError): sub.Read()
    report(StopIteration(), 'after failed read')
    assert caplog.records[-1].message == 'after failed read'
    sub.Close()


@pytest.mark.parametrize('failure', ['reader', 'handler'])
def test_failed_reader_does_not_emit_or_leave_thread(factory, failure):
    opened, _, _ = factory
    sub = FeedbackSubscriber('rt/sportmodestate', object)
    values = []
    def fail_handler(_): raise RuntimeError('synthetic_handler_failure')
    sub.Init(values.append if failure == 'reader' else fail_handler)
    worker = sub._thread
    opened[0].messages.put(RuntimeError('synthetic_read_failure') if failure == 'reader' else object())
    worker.join(1)
    assert not worker.is_alive() and values == []
    sub.Close(); sub.Close()
    assert opened[0].closed == 1
