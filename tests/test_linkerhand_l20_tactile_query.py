"""Fresh tactile queries, cancellation, FIFO admission and CAN command isolation."""

from __future__ import annotations

import queue
import sys
import threading
import time
from pathlib import Path

import can
import pytest

DRIVER = Path(__file__).resolve().parents[1] / "linkerhand" / "l20"
sys.path.insert(0, str(DRIVER))

from can_client import FINGER_NAMES, FT_MATRIX_REQ, FT_TOUCH_B1, L20CanClient, TactileQueryTimeout


class MatrixReplyBus:
    """Deliver protocol rows through the client's actual receive thread."""

    def __init__(self):
        self.can_id = 0x28
        self.requests = []
        self.request_order = []
        self.messages = []
        self.rows = {finger: set(range(12)) for finger in range(5)}
        self.reply_value = 10
        self.send_delay_s = 0
        self.requested = threading.Event()
        self._inbox = queue.Queue()

    def send(self, message):
        data = list(message.data)
        self.messages.append(message)
        if len(data) == 1 and 0x41 <= data[0] <= 0x45:
            self._inbox.put(can.Message(arbitration_id=self.can_id,
                                       data=[data[0], 70, 200, 255, 0, 0, 255],
                                       is_extended_id=False))
            return
        if len(data) == 1 and FT_TOUCH_B1 <= data[0] < FT_TOUCH_B1 + 5:
            finger = data[0] - FT_TOUCH_B1
            self.request_order.append(('scalar', finger))
            self._inbox.put(can.Message(arbitration_id=self.can_id,
                                       data=[data[0], 1, self.reply_value + finger],
                                       is_extended_id=False))
            return
        if len(data) != 2 or not FT_TOUCH_B1 <= data[0] < FT_TOUCH_B1 + 5 or data[1] != FT_MATRIX_REQ:
            return
        finger = data[0] - FT_TOUCH_B1
        self.requests.append(finger)
        self.request_order.append(('matrix', finger))
        self.requested.set()
        if self.send_delay_s:
            time.sleep(self.send_delay_s)
        for row in sorted(self.rows.get(finger, set())):
            self._inbox.put(can.Message(
                arbitration_id=self.can_id,
                data=[data[0], row * 16] + [self.reply_value + finger + row] * 6,
                is_extended_id=False,
            ))

    def recv(self, timeout):
        try:
            return self._inbox.get(timeout=timeout)
        except queue.Empty:
            return None

    def shutdown(self):
        pass


@pytest.fixture
def matrix_client():
    bus = MatrixReplyBus()
    client = L20CanClient(
        side="left", recv_timeout_s=0.01,
        bus_factory=lambda *_: bus,
    )
    try:
        yield client, bus
    finally:
        client.close()


def test_selected_fingers_need_no_other_finger_replies(matrix_client):
    client, bus = matrix_client
    bus.rows = {0: set(range(12)), 1: set(range(12))}
    result = client.get_fresh_matrix_touch(["thumb", "index"], timeout_s=0.2)
    assert list(result) == ["thumb", "index"]
    assert bus.requests == [0, 1]
    assert result["thumb"][11] == [21] * 6
    assert result["index"][0] == [11] * 6
    assert (2, 0) not in client._matrix_updates
    result["thumb"][0][0] = 255
    assert client._matrix_touch[0][0][0] == 10


def test_reordered_rows_after_last_row_are_still_confirmed(matrix_client):
    client, bus = matrix_client
    bus.rows[1] = {11}
    results, errors = [], []
    def query():
        try:
            results.append(client.get_fresh_matrix_touch(['index'], timeout_s=.5))
        except BaseException as exc:
            errors.append(exc)
    reader = threading.Thread(target=query)
    reader.start()
    try:
        assert bus.requested.wait(1)
        time.sleep(.03)
        assert not results
        for row in reversed(range(11)):
            bus._inbox.put(can.Message(arbitration_id=bus.can_id,
                                       data=[FT_TOUCH_B1 + 1, row * 16] + [11 + row] * 6,
                                       is_extended_id=False))
        reader.join(1)
        assert not reader.is_alive() and not errors
        assert results[0]['index'][0] == [11] * 6
        assert results[0]['index'][11] == [22] * 6
        assert bus.requests == [1]
    finally:
        reader.join(1)


def test_selected_finger_requires_each_of_its_twelve_rows(matrix_client):
    client, bus = matrix_client
    bus.rows[1].remove(11)
    with pytest.raises(TactileQueryTimeout, match="本轮完整") as failure:
        client.get_fresh_matrix_touch(["thumb", "index"], timeout_s=0.06)
    assert bus.requests == [0, 1, 1]
    assert failure.value.code == 'tactile_query_timeout'
    assert failure.value.details['query_phase'] == 'receive_rows'
    assert failure.value.details['missing_rows'] == {'index': [11]}
    assert failure.value.details['requests_sent'] == ['thumb', 'index', 'index']
    assert failure.value.details['retry_count'] == 1


def test_cached_matrix_row_cannot_complete_new_partial_reply(matrix_client):
    client, bus = matrix_client
    first = client.get_fresh_matrix_touch(["index"], timeout_s=0.1)
    assert first["index"][11] == [22] * 6
    bus.rows[1].remove(11)
    bus.reply_value = 40
    with pytest.raises(TimeoutError, match="本轮完整"):
        client.get_fresh_matrix_touch(["index"], timeout_s=0.05)
    assert client._matrix_touch[1][0] == [41] * 6
    assert client._matrix_touch[1][11] == [22] * 6


def test_single_retry_recovers_a_dropped_reply_with_all_new_rows(matrix_client, monkeypatch):
    client, bus = matrix_client
    bus.rows[1] = set(range(6))
    send = bus.send
    def drop_first_tail(message):
        send(message)
        if list(message.data) == [FT_TOUCH_B1 + 1, FT_MATRIX_REQ] and bus.requests.count(1) == 1:
            bus.rows[1] = set(range(12))
            bus.reply_value = 30
    monkeypatch.setattr(bus, 'send', drop_first_tail)
    result = client.get_fresh_matrix_touch(['thumb', 'index'], timeout_s=.3)
    assert bus.requests == [0, 1, 1]
    assert result['thumb'][0] == [10] * 6
    assert result['index'][0] == [31] * 6
    assert result['index'][11] == [42] * 6


def test_retry_does_not_merge_two_incomplete_generations_or_extend_deadline(matrix_client, monkeypatch):
    client, bus = matrix_client
    bus.rows[1] = set(range(6))
    send = bus.send
    def complementary_partial_rows(message):
        send(message)
        if list(message.data) == [FT_TOUCH_B1 + 1, FT_MATRIX_REQ]:
            bus.rows[1] = set(range(6, 12))
    monkeypatch.setattr(bus, 'send', complementary_partial_rows)
    started = time.monotonic()
    with pytest.raises(TactileQueryTimeout) as failure:
        client.get_fresh_matrix_touch(['index'], timeout_s=.15)
    assert time.monotonic() - started < .4
    assert bus.requests == [1, 1]
    assert failure.value.details['missing_rows'] == {'index': list(range(6))}
    assert failure.value.details['retry_count'] == 1
    assert failure.value.details['timeout_s'] == .15


def test_default_query_still_requests_and_returns_all_sixty_rows(matrix_client):
    client, bus = matrix_client
    result = client.get_fresh_matrix_touch()
    assert list(result) == FINGER_NAMES
    assert bus.requests == list(range(5))
    assert len(client._matrix_updates) == 60
    for finger, name in enumerate(FINGER_NAMES):
        assert len(result[name]) == 12
        assert result[name][11] == [21 + finger] * 6


def _query_in_thread(client, cancel):
    errors = []
    started = threading.Event()

    def run():
        started.set()
        try:
            client.get_fresh_matrix_touch(["index"], timeout_s=1, cancel=cancel)
        except BaseException as exc:
            errors.append(exc)

    thread = threading.Thread(target=run)
    thread.start()
    assert started.wait(0.5)
    return thread, errors


def test_cancel_while_waiting_for_query_lock_sends_no_can(matrix_client):
    client, bus = matrix_client
    cancel = threading.Event()
    client._tactile_query_lock.acquire()
    try:
        thread, errors = _query_in_thread(client, cancel)
        time.sleep(0.025)
        cancel.set()
        thread.join(0.3)
        assert not thread.is_alive()
        assert len(errors) == 1 and isinstance(errors[0], InterruptedError)
        assert bus.requests == []
    finally:
        client._tactile_query_lock.release()


def test_cancel_while_waiting_for_rows_releases_query_lock(matrix_client):
    client, bus = matrix_client
    bus.rows[1] = set()
    cancel = threading.Event()
    thread, errors = _query_in_thread(client, cancel)
    try:
        assert bus.requested.wait(0.5)
        cancel.set()
        thread.join(0.3)
        assert not thread.is_alive()
        assert len(errors) == 1 and isinstance(errors[0], InterruptedError)
        assert client._tactile_query_lock.acquire(blocking=False)
        client._tactile_query_lock.release()
    finally:
        cancel.set()
        thread.join(1)


def test_timeout_includes_waiting_for_query_lock(matrix_client):
    client, bus = matrix_client
    client._tactile_query_lock.acquire()
    try:
        start = time.monotonic()
        with pytest.raises(TactileQueryTimeout, match="本轮完整") as failure:
            client.get_fresh_matrix_touch(["index"], timeout_s=0.04)
        assert time.monotonic() - start < 0.3
        assert bus.requests == []
        assert failure.value.details['query_phase'] == 'query_lock'
        assert failure.value.details['requests_sent'] == []
        assert failure.value.details['missing_rows'] == {}
        assert failure.value.details['lock_wait_s'] >= 0.04
        assert client._tactile_query_waiters == []
    finally:
        client._tactile_query_lock.release()


def test_timeout_includes_sending_the_request(matrix_client):
    client, bus = matrix_client
    bus.send_delay_s = 0.025
    with pytest.raises(TactileQueryTimeout, match="本轮完整") as failure:
        client.get_fresh_matrix_touch(["thumb", "index"], timeout_s=0.01)
    assert bus.requests == [0]
    assert failure.value.details['query_phase'] == 'send_request'
    assert failure.value.details['requests_sent'] == ['thumb']
    assert failure.value.details['elapsed_s'] >= 0.025
    assert client._tactile_query_lock.acquire(blocking=False)
    client._tactile_query_lock.release()


@pytest.mark.parametrize("fingers", [[], ["unknown"], ["thumb", "unknown"],
                                     ["index", "index"], "index", [None]])
def test_invalid_fingers_fail_before_can(matrix_client, fingers):
    client, bus = matrix_client
    with pytest.raises(ValueError, match="fingers"):
        client.get_fresh_matrix_touch(fingers)
    assert bus.requests == []


@pytest.mark.parametrize("timeout_s", [0, -1, True, None, "1", float("inf"), float("nan")])
def test_invalid_timeout_fails_before_can(matrix_client, timeout_s):
    client, bus = matrix_client
    with pytest.raises(ValueError, match="timeout_s"):
        client.get_fresh_matrix_touch(["index"], timeout_s=timeout_s)
    assert bus.requests == []


def test_already_cancelled_query_sends_no_can(matrix_client):
    client, bus = matrix_client
    cancel = threading.Event()
    cancel.set()
    with pytest.raises(InterruptedError):
        client.get_fresh_matrix_touch(["index"], cancel=cancel)
    assert bus.requests == []


@pytest.mark.parametrize('cancelled', [False, True])
def test_scalar_wait_for_shared_query_lock_is_bounded_and_cancellable(matrix_client, cancelled):
    client, bus = matrix_client
    cancel = threading.Event()
    errors = []
    client._tactile_query_lock.acquire()
    def query():
        try:
            client.get_fresh_tactile(['index'], timeout_s=0.06, cancel=cancel)
        except BaseException as exc:
            errors.append(exc)
    thread = threading.Thread(target=query)
    try:
        thread.start()
        if cancelled:
            time.sleep(0.01)
            cancel.set()
        thread.join(0.5)
        assert not thread.is_alive(), 'scalar query ignored its lock-wait budget/cancellation'
        assert len(errors) == 1
        assert isinstance(errors[0], InterruptedError if cancelled else TactileQueryTimeout)
        if not cancelled:
            assert errors[0].details['query_phase'] == 'query_lock'
            assert errors[0].details['missing_fingers'] == []
        assert bus.request_order == [] and client._tactile_query_waiters == []
    finally:
        cancel.set()
        client._tactile_query_lock.release()
        thread.join(1)


def test_streaming_queries_cannot_reacquire_ahead_of_queued_matrix_or_scalar_reads(matrix_client, monkeypatch):
    client, bus = matrix_client
    entered, resume = threading.Event(), threading.Event()
    send = bus.send
    errors = []
    threads = []
    def blocked_first_send(message):
        if not entered.is_set():
            entered.set()
            assert resume.wait(2)
        send(message)
    monkeypatch.setattr(bus, 'send', blocked_first_send)
    def start(query):
        def run():
            try:
                query()
            except BaseException as exc:
                errors.append(exc)
        thread = threading.Thread(target=run)
        threads.append(thread)
        thread.start()
    def wait_queue(count):
        deadline = time.monotonic() + 1
        while time.monotonic() < deadline:
            with client._tactile_query_condition:
                if len(client._tactile_query_waiters) == count:
                    return
            time.sleep(0.002)
        pytest.fail('query did not join the queue')
    def streaming_reader():
        for _ in range(2):
            client.get_fresh_matrix_touch(['thumb'], timeout_s=2)
    try:
        start(streaming_reader)
        assert entered.wait(1)
        start(lambda: client.get_fresh_tactile(['index'], timeout_s=2))
        wait_queue(1)
        start(lambda: client.get_fresh_matrix_touch(['middle'], timeout_s=2))
        wait_queue(2)
        resume.set()
        for thread in threads:
            thread.join(2)
        assert not any(thread.is_alive() for thread in threads) and not errors
        assert bus.request_order == [('matrix', 0), ('scalar', 1), ('matrix', 2), ('matrix', 0)]
        assert client._tactile_query_waiters == []
    finally:
        resume.set()
        for thread in threads:
            thread.join(2)


@pytest.mark.parametrize('cancelled', [False, True])
def test_expired_or_cancelled_queue_head_does_not_block_next_reader(matrix_client, cancelled):
    client, bus = matrix_client
    cancel = threading.Event()
    client._tactile_query_lock.acquire()
    errors = []
    def first_query():
        try:
            client.get_fresh_matrix_touch(['thumb'], timeout_s=0.06, cancel=cancel)
        except BaseException as exc:
            errors.append(exc)
    first = threading.Thread(target=first_query)
    try:
        first.start()
        if cancelled:
            time.sleep(0.01)
            cancel.set()
        first.join(0.5)
        assert not first.is_alive() and len(errors) == 1
        assert isinstance(errors[0], InterruptedError if cancelled else TactileQueryTimeout)
    finally:
        cancel.set()
        client._tactile_query_lock.release()
        first.join(1)
    assert client.get_fresh_matrix_touch(['index'], timeout_s=0.5)['index'][0] == [11] * 6
    assert bus.requests == [1] and client._tactile_query_waiters == []


@pytest.mark.parametrize('interloper', ['positions', 'device_info'])
def test_incomplete_matrix_reply_excludes_position_queries_and_other_commands(matrix_client, interloper):
    client, bus = matrix_client
    bus.rows[1] = set(range(6))
    bus.messages.clear()
    errors, results = [], []
    def read_matrix():
        try:
            results.append(client.get_fresh_matrix_touch(['index'], timeout_s=1))
        except BaseException as exc:
            errors.append(exc)
    def other_command():
        try:
            if interloper == 'positions':
                client.get_positions_vector()
            else:
                client.send_command(0xC0, [], sleep_s=0)
        except BaseException as exc:
            errors.append(exc)
    matrix_thread = threading.Thread(target=read_matrix)
    other = threading.Thread(target=other_command)
    matrix_thread.start()
    try:
        assert bus.requested.wait(1)
        other.start()
        time.sleep(.03)
        assert [list(message.data) for message in bus.messages] == [[FT_TOUCH_B1 + 1, FT_MATRIX_REQ]]
    finally:
        for row in range(6, 12):
            bus._inbox.put(can.Message(arbitration_id=bus.can_id,
                                       data=[FT_TOUCH_B1 + 1, row * 16] + [11 + row] * 6,
                                       is_extended_id=False))
        matrix_thread.join(2)
        if other.ident is not None:
            other.join(2)
    assert not matrix_thread.is_alive() and not other.is_alive() and not errors
    assert results[0]['index'][11] == [22] * 6
    assert client.get_fresh_matrix_touch(['thumb'], timeout_s=.5)['thumb'][0] == [10] * 6


@pytest.mark.parametrize('method,values,frames', [
    ('set_joint_positions', [255] * 20, list(range(0x41, 0x46))),
    ('set_speeds', [85] * 5, list(range(0x49, 0x4E))),
    ('set_currents', [100] * 5, [*range(0x51, 0x56), 0x82]),
    ('set_joint_torque_limits', [100] * 20, list(range(0x51, 0x56))),
])
def test_tactile_reads_cannot_split_a_motor_command_batch(matrix_client, monkeypatch, method, values, frames):
    client, bus = matrix_client
    entered, resume = threading.Event(), threading.Event()
    send = bus.send
    errors = []
    bus.messages.clear()
    def pause_first_frame(message):
        if not entered.is_set():
            entered.set()
            assert resume.wait(2)
        send(message)
    monkeypatch.setattr(bus, 'send', pause_first_frame)
    def write():
        try:
            getattr(client, method)(values)
        except BaseException as exc:
            errors.append(exc)
    def read():
        try:
            client.get_fresh_matrix_touch(['index'], timeout_s=1)
        except BaseException as exc:
            errors.append(exc)
    writer, reader = threading.Thread(target=write), threading.Thread(target=read)
    writer.start()
    try:
        assert entered.wait(1)
        reader.start()
        deadline = time.monotonic() + 1
        while time.monotonic() < deadline:
            with client._tactile_query_condition:
                if client._tactile_query_waiters:
                    break
            time.sleep(.002)
        else:
            pytest.fail('tactile reader did not wait for the batch')
    finally:
        resume.set()
        writer.join(2)
        if reader.ident is not None:
            reader.join(2)
    assert not writer.is_alive() and not reader.is_alive() and not errors
    assert [message.data[0] for message in bus.messages] == frames + [FT_TOUCH_B1 + 1]


@pytest.mark.parametrize('lock_name,phase', [('_position_query_lock', 'position_lock'),
                                           ('_command_lock', 'command_lock')])
@pytest.mark.parametrize('cancelled', [False, True])
def test_cancel_or_timeout_during_bus_admission_releases_all_acquired_locks(
        matrix_client, lock_name, phase, cancelled):
    client, bus = matrix_client
    lock = getattr(client, lock_name)
    cancel = threading.Event()
    errors = []
    def query():
        try:
            client.get_fresh_matrix_touch(['index'], timeout_s=.06, cancel=cancel)
        except BaseException as exc:
            errors.append(exc)
    thread = threading.Thread(target=query)
    lock.acquire()
    try:
        thread.start()
        if cancelled:
            time.sleep(.01)
            cancel.set()
        thread.join(.5)
        assert not thread.is_alive() and len(errors) == 1
        assert isinstance(errors[0], InterruptedError if cancelled else TactileQueryTimeout)
        if not cancelled:
            assert errors[0].details['query_phase'] == phase
            assert errors[0].details['requests_sent'] == []
        assert not bus.requests and not client._tactile_query_waiters
    finally:
        cancel.set()
        lock.release()
        thread.join(1)
    assert client.get_fresh_matrix_touch(['index'], timeout_s=.5)['index'][0] == [11] * 6
