"""Position timeout evidence without retries, cached success or motion commands."""
import threading
import time

import pytest

from test_linkerhand_l20 import L20CanClient, MockCanMessage
from test_linkerhand_l20_binding import SwitchableCanBus
from can_client import PositionQueryTimeout


SERIAL = (0x41, 0x42, 0x43, 0x44, 0x45)
LEGACY = (0x01, 0x02, 0x03, 0x04)


@pytest.fixture(params=[False, True], ids=['serial', 'legacy'])
def position_client(request):
    bus = SwitchableCanBus(can_id=0x28)
    client = L20CanClient(side='left', can_id=0x28, recv_timeout_s=0.03,
                          legacy_l20_mode=request.param, bus_factory=lambda *_: bus)
    bus.sent_messages.clear()
    try:
        yield client, bus, LEGACY if request.param else SERIAL
    finally:
        client.close()


def frame_names(frames):
    return [f'0x{ft:02X}' for ft in frames]


def assert_single_read_batch(bus, frames):
    assert [list(msg.data) for msg in bus.sent_messages] == [[ft] for ft in frames]


def test_success_preserves_single_request_batch_and_clears_diagnostic_context(position_client):
    client, bus, frames = position_client
    assert len(client.get_positions_vector()) == 20
    assert_single_read_batch(bus, frames)
    assert not hasattr(client._position_query_context, 'current')


@pytest.mark.parametrize('missing_index', range(5))
def test_missing_frame_has_protocol_mapping_and_receive_phase(position_client, missing_index):
    client, bus, frames = position_client
    if missing_index >= len(frames):
        pytest.skip('legacy position feedback has four joint groups, not five fingers')
    bus.missing_frame = frames[missing_index]
    with pytest.raises(PositionQueryTimeout, match='左手.*0x28') as error:
        client.get_positions_vector()
    details = error.value.details
    assert isinstance(error.value, TimeoutError)
    assert error.value.code == 'position_query_timeout'
    assert details['query_type'] == 'position' and details['query_phase'] == 'receive_reply'
    assert details['protocol'] == ('legacy' if client.legacy_l20_mode else 'serial')
    assert details['expected_frames'] == details['sent_frames'] == frame_names(frames)
    assert details['missing_frames'] == [f'0x{bus.missing_frame:02X}']
    assert details['received_frames'] == frame_names([ft for ft in frames if ft != bus.missing_frame])
    if client.legacy_l20_mode:
        assert details['missing_groups'] == [('pitch', 'yaw', 'roll', 'tip')[missing_index]]
        assert 'missing_fingers' not in details
    else:
        assert details['missing_fingers'] == [('thumb', 'index', 'middle', 'ring', 'pinky')[missing_index]]
        assert 'missing_groups' not in details
    assert details['in_flight_frame'] is None and details['not_sent_frames'] == []
    assert details['timeout_s'] == 0.03 and details['timeout_scope'] == 'receive_reply'
    assert details['receive_wait_s'] >= 0.029
    assert details['elapsed_s'] >= details['receive_wait_s'] + details['send_elapsed_s'] - 0.00001
    assert set(details['request_elapsed_s']) == set(frame_names(frames))
    assert_single_read_batch(bus, frames)


def test_concurrent_readers_keep_independent_missing_frame_evidence(position_client, monkeypatch):
    client, bus, frames = position_client
    send = bus.send
    losses = {'position-reader-a': frames[1], 'position-reader-b': frames[-1]}
    def selective_send(msg):
        if msg.data[0] == losses.get(threading.current_thread().name):
            bus.sent_messages.append(msg)
            return
        send(msg)
    monkeypatch.setattr(bus, 'send', selective_send)
    errors = {}
    def query():
        try:
            client.get_positions_vector()
        except Exception as exc:
            errors[threading.current_thread().name] = exc
    workers = [threading.Thread(target=query, name=name, daemon=True) for name in losses]
    for worker in workers:
        worker.start()
    for worker in workers:
        worker.join(3)
        assert not worker.is_alive()
    assert set(errors) == set(losses)
    for name, ft in losses.items():
        assert isinstance(errors[name], PositionQueryTimeout)
        assert errors[name].details['missing_frames'] == [f'0x{ft:02X}']
        assert errors[name].details['sent_frames'] == frame_names(frames)
    assert [list(msg.data) for msg in bus.sent_messages] == [[ft] for ft in frames] * 2
    assert not hasattr(client._position_query_context, 'current')


def test_cached_and_late_positions_do_not_hide_missing_fresh_replies(position_client):
    client, bus, frames = position_client
    client.get_positions_vector()
    bus.sent_messages.clear()
    bus.online = False
    with pytest.raises(PositionQueryTimeout) as error:
        client.get_positions_vector()
    details = error.value.details
    assert details['missing_frames'] == frame_names(frames) and details['received_frames'] == []
    assert_single_read_batch(bus, frames)
    # A late reply changes the cache, never the captured timeout evidence.
    ft = frames[0]
    payload = [ft, 0, 0, 0, 0, 0] if client.legacy_l20_mode else [ft, 0, 0, 0, 0, 0, 0]
    client._process_message(MockCanMessage(client.can_id, payload))
    assert details['received_frames'] == [] and details['missing_frames'] == frame_names(frames)
    assert not client.probe_positions(timeout_s=0.03), 'late/cached feedback is not a fresh full reply'
    bus.online = True
    assert client.probe_positions(timeout_s=0.03)


def test_send_timeout_distinguishes_completed_in_flight_and_unrequested_frames(position_client, monkeypatch):
    client, bus, frames = position_client
    send = client.send_command
    def timeout(frame, data, sleep_s=0):
        if frame == frames[2]:
            raise TimeoutError('simulated send/open timeout')
        return send(frame, data, sleep_s=sleep_s)
    monkeypatch.setattr(client, 'send_command', timeout)
    with pytest.raises(PositionQueryTimeout) as error:
        client.get_positions_vector()
    details = error.value.details
    assert details['query_phase'] == 'send_request' and details['receive_wait_s'] == 0
    assert details['sent_frames'] == frame_names(frames[:2])
    assert details['in_flight_frame'] == f'0x{frames[2]:02X}'
    assert details['not_sent_frames'] == frame_names(frames[3:])
    assert set(details['missing_frames']) <= set(frame_names(frames[:2]))
    assert details['cause_type'] == 'TimeoutError'
    assert details['cause_message'] == 'simulated send/open timeout'
    assert set(details['request_elapsed_s']) == set(frame_names(frames[:3]))
    assert error.value.__cause__ is not None
    assert_single_read_batch(bus, frames[:2])
    assert not hasattr(client._position_query_context, 'current')
    monkeypatch.setattr(client, 'send_command', send)
    bus.sent_messages.clear()
    client.request_positions()  # Direct asynchronous requests retain their original interface.
    assert_single_read_batch(bus, frames)
    assert details['sent_frames'] == frame_names(frames[:2])


@pytest.mark.parametrize('lock_name,timing_name', [
    ('_position_query_lock', 'query_lock_wait_s'),
    ('_command_lock', 'command_lock_wait_s'),
])
def test_lock_wait_is_measured_without_consuming_existing_receive_budget(position_client, monkeypatch,
                                                                       lock_name, timing_name):
    client, bus, frames = position_client
    bus.online = False
    entered = threading.Event()
    lock = getattr(client, lock_name)
    class ObservedLock:
        def __enter__(self):
            entered.set()
            lock.acquire()
            return self
        def __exit__(self, *args):
            lock.release()
    monkeypatch.setattr(client, lock_name, ObservedLock())
    errors = []
    def query():
        try:
            client.get_positions_vector()
        except Exception as exc:
            errors.append(exc)
    lock.acquire()
    worker = threading.Thread(target=query, daemon=True)
    try:
        worker.start()
        assert entered.wait(2)
        time.sleep(0.04)  # Longer than the receive budget: lock waits remain outside it.
    finally:
        lock.release()
        worker.join(3)
    assert not worker.is_alive() and len(errors) == 1
    assert isinstance(errors[0], PositionQueryTimeout)
    details = errors[0].details
    assert details[timing_name] >= 0.039
    assert details['query_phase'] == 'receive_reply' and details['receive_wait_s'] >= 0.029
    assert details['missing_frames'] == frame_names(frames)
    assert_single_read_batch(bus, frames)
