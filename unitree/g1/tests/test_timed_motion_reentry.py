"""Offline concurrency checks for stop acknowledgement and action ownership.

These tests describe the mainline semantics: a later action may replace an
in-flight or failed stop; stale replies must remain associated with the old
operation and must not change the new action's software state.  They do not
claim that an acknowledgement proves a physical stop.
"""
import threading

from timed_motion import TimedMotion


class Client:
    def __init__(self):
        self.calls = []
        self.stop_ret = 0
        self.entered = threading.Event()
        self.release = threading.Event()
        self.release.set()

    def BeginMove(self, *args):
        self.calls.append('move')
        return lambda: 0

    def BeginStopMove(self):
        self.calls.append('stop')
        def wait():
            self.entered.set()
            assert self.release.wait(2)
            return self.stop_ret
        return wait


def test_inflight_stop_reply_remains_with_old_action_after_replacement():
    client = Client()
    events = []
    motion = TimedMotion(client, on_stop=events.append)
    assert motion.start(0, 0, 0, 0, 'a')['ret'] == 0
    client.release.clear()
    client.stop_ret = 17
    result = []
    worker = threading.Thread(target=lambda: result.append(motion.stop()))
    worker.start()
    try:
        assert client.entered.wait(2)
        assert not motion.is_idle()
        assert motion.get_result('a')['status'] == 'pending'
        assert motion.start(0, 0, 0, 0, 'b')['ret'] == 0
        old = motion.get_result('a')
        assert old['status'] == 'cancelled'
        assert old['result']['reason'] == 'replaced'
        assert motion.get_result('b')['status'] == 'pending'
        assert client.calls == ['move', 'stop', 'move']
    finally:
        client.release.set()
        worker.join(2)
    assert not worker.is_alive()
    assert result[0]['ret'] == 17
    assert result[0]['action_id'] == 'a'
    assert result[0]['state'] == 'superseded'
    assert motion.get_result('a') == old
    assert motion.active['id'] == 'b'
    assert motion.get_result('b')['status'] == 'pending'
    assert events == []
    calls = list(client.calls)
    assert motion.stop('stop_repeat', expected_id='a', retry=True)['state'] == 'superseded'
    assert client.calls == calls
    client.stop_ret = 0
    assert motion.stop()['ret'] == 0
    assert motion.get_result('b')['status'] == 'cancelled'
    assert motion.get_result('a') == old


def test_stop_ack_and_pending_old_move_reply_allow_replacement():
    class SlowMove(Client):
        def __init__(self):
            super().__init__()
            self.move_entered = threading.Event()
            self.move_release = threading.Event()
        def BeginMove(self, *args):
            self.calls.append('move')
            if len(self.calls) == 1:
                def wait():
                    self.move_entered.set()
                    assert self.move_release.wait(2)
                    return 0
                return wait
            return lambda: 0
    client = SlowMove()
    motion = TimedMotion(client)
    results = []
    worker = threading.Thread(target=lambda: results.append(motion.start(0, 0, 0, 0, 'a')))
    worker.start()
    try:
        assert client.move_entered.wait(2)
        assert motion.stop()['ret'] == 0
        assert not motion.is_idle()
        assert motion.get_result('a')['status'] == 'pending'
        assert motion.start(0, 0, 0, 0, 'b')['ret'] == 0
        old = motion.get_result('a')
        assert old['status'] == 'cancelled'
        assert old['result']['reason'] == 'replaced'
        assert client.calls == ['move', 'stop', 'move']
    finally:
        client.move_release.set()
        worker.join(2)
    assert not worker.is_alive()
    assert results[0]['ret'] == 0
    assert results[0]['state'] == 'superseded'
    assert motion.get_result('a') == old
    assert motion.active['id'] == 'b'
    assert motion.get_result('b')['status'] == 'pending'


def test_idle_stop_inflight_failure_does_not_prevent_start():
    client = Client()
    motion = TimedMotion(client)
    client.release.clear()
    client.stop_ret = None
    results = []
    worker = threading.Thread(target=lambda: results.append(motion.stop()))
    worker.start()
    try:
        assert client.entered.wait(2)
        assert motion.is_idle()
        assert motion.start(0, 0, 0, 0, 'a')['ret'] == 0
        assert client.calls == ['stop', 'move']
    finally:
        client.release.set()
        worker.join(2)
    assert not worker.is_alive()
    assert results[0]['ret'] is None
    assert results[0]['state'] == 'superseded'
    assert motion.active['id'] == 'a'
    assert motion.get_result('a')['status'] == 'pending'


def test_out_of_order_idle_stop_results_do_not_prevent_start():
    class OutOfOrderClient(Client):
        def __init__(self):
            super().__init__()
            self.first_entered = threading.Event()
            self.first_release = threading.Event()
        def BeginStopMove(self):
            self.calls.append('stop')
            if len(self.calls) == 1:
                def wait():
                    self.first_entered.set()
                    assert self.first_release.wait(2)
                    return 0
                return wait
            return lambda: 17
    client = OutOfOrderClient()
    motion = TimedMotion(client)
    results = []
    first = threading.Thread(target=lambda: results.append(motion.stop()))
    first.start()
    try:
        assert client.first_entered.wait(2)
        assert motion.stop()['ret'] == 17
        assert motion.is_idle()
        assert motion.start(0, 0, 0, 0, 'a')['ret'] == 0
    finally:
        client.first_release.set()
        first.join(2)
    assert not first.is_alive()
    assert results[0]['ret'] == 0
    assert results[0]['state'] == 'superseded'
    assert client.calls == ['stop', 'stop', 'move']
    assert motion.active['id'] == 'a'
    assert motion.get_result('a')['status'] == 'pending'
