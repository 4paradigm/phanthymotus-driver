"""Native SocketCAN and opt-in QEMU regressions; no real hardware access."""
import errno
from collections import deque
import sys
import threading
import struct
import time
from pathlib import Path
from types import SimpleNamespace
from concurrent.futures import ThreadPoolExecutor

import pytest
from can.interfaces.socketcan.socketcan import capture_message as _NATIVE_CAPTURE_MESSAGE
from test_linkerhand_l20 import L20CanClient, MockCanBus


@pytest.fixture
def socket_environment(monkeypatch):
    import socket
    from can.interfaces.socketcan import socketcan
    calls = []
    owner = threading.get_ident()
    def setsockopt(sock, level, option, value, *args):
        # Some older card tests leave daemon retry loops alive. Their unrelated
        # socket attempts must not alter this test's operation count.
        if threading.get_ident() == owner:
            calls.append((sock, level, option, value, args))
        return 'original-result'
    monkeypatch.setattr(socket.socket, 'setsockopt', setsockopt)
    # Record this original before the compatibility installer changes it.
    monkeypatch.setattr(socketcan, 'capture_message', socketcan.capture_message)
    return socket, socketcan, calls


@pytest.mark.parametrize('flag', [None, '0', '1'])
def test_qemu_hooks_require_explicit_startup_opt_in(socket_environment, monkeypatch, flag):
    import can
    socket, socketcan, _ = socket_environment
    if flag is None:
        monkeypatch.delenv('LINKERHAND_L20_QEMU_COMPAT', raising=False)
    else:
        monkeypatch.setenv('LINKERHAND_L20_QEMU_COMPAT', flag)
    original_setsockopt, original_capture = socket.socket.setsockopt, socketcan.capture_message
    if flag != '1':
        def unexpected_patch():
            pytest.fail('native/default mode must not install QEMU hooks')
        monkeypatch.setattr(L20CanClient, '_patch_socketcan_for_qemu', unexpected_patch)
    monkeypatch.setattr(L20CanClient, '_bring_up_socketcan', lambda *_: None)
    bus = MockCanBus(can_id=0x28)
    monkeypatch.setattr(can.interface, 'Bus', lambda **kwargs: bus)
    client = L20CanClient(side='left', recv_timeout_s=.01)
    try:
        assert client._qemu_compat is (flag == '1')
        for _ in range(3):
            assert client._open_bus('can0', 1_000_000) is bus
        if flag == '1':
            assert socket.socket.setsockopt is not original_setsockopt
            assert socketcan.capture_message is not original_capture
            assert socket.socket.setsockopt.__wrapped__ is original_setsockopt
        else:
            assert socket.socket.setsockopt is original_setsockopt
            assert socketcan.capture_message is original_capture
    finally:
        client.close()


@pytest.mark.parametrize('flag', ['', 'true', 'false', '2', 'yes'])
def test_invalid_qemu_opt_in_fails_before_opening_can(monkeypatch, flag):
    monkeypatch.setenv('LINKERHAND_L20_QEMU_COMPAT', flag)
    opened = []
    with pytest.raises(ValueError, match='LINKERHAND_L20_QEMU_COMPAT'):
        L20CanClient(bus_factory=lambda *args: opened.append(args))
    assert not opened


def test_thousands_of_patch_installs_do_not_build_recursive_socket_wrappers(socket_environment):
    socket, socketcan, calls = socket_environment
    L20CanClient._patch_socketcan_for_qemu()
    installed = socket.socket.setsockopt
    capture = socketcan.capture_message
    for _ in range(2500):
        L20CanClient._patch_socketcan_for_qemu()
    previous_limit = sys.getrecursionlimit()
    try:
        sys.setrecursionlimit(1000)
        assert socket.socket.setsockopt(object(), 1, 2, b'value') == 'original-result'
    finally:
        sys.setrecursionlimit(previous_limit)
    assert len(calls) == 1
    assert socket.socket.setsockopt is installed
    assert socketcan.capture_message is capture


def test_parallel_left_right_reconnects_install_one_wrapper(socket_environment):
    socket, _, calls = socket_environment
    barrier = threading.Barrier(8)
    def reconnect(_):
        barrier.wait()
        for _ in range(300):
            L20CanClient._patch_socketcan_for_qemu()
        return socket.socket.setsockopt
    with ThreadPoolExecutor(max_workers=8) as pool:
        installed = list(pool.map(reconnect, range(8)))
    assert all(fn is installed[0] for fn in installed)
    assert installed[0](object(), 1, 2, b'value') == 'original-result'
    assert len(calls) == 1


@pytest.fixture
def can_sysfs(tmp_path, monkeypatch):
    import can_client
    import hand_device
    import subprocess
    root = tmp_path / 'net'
    root.mkdir()
    monkeypatch.setattr(can_client, 'Path', lambda path: root if path == '/sys/class/net' else Path(path))
    monkeypatch.setattr(hand_device, 'Path', lambda path: root if path == '/sys/class/net' else Path(path))
    forked = []
    def no_child(*args, **kwargs):
        forked.append(args)
        raise AssertionError('CAN paths must not create subprocesses under QEMU')
    monkeypatch.setattr(subprocess, 'Popen', no_child)
    yield root
    assert not forked


def can_entry(root, name='can0', *, flags='0x1', kind=280):
    entry = root / name
    entry.mkdir()
    (entry / 'type').write_text(str(kind))
    (entry / 'flags').write_text(flags)
    (entry / 'ifindex').write_text('7')
    return entry


def test_up_interface_and_discovery_never_spawn_ip(can_sysfs, monkeypatch):
    import can_client
    from hand_device import _discover_can_channels
    can_entry(can_sysfs, 'can1')
    can_entry(can_sysfs, 'can0')
    can_entry(can_sysfs, 'ens33', kind=1)
    (can_sysfs / 'removed_adapter').mkdir()
    def no_netlink(*args, **kwargs):
        pytest.fail('an enabled interface must not be reconfigured')
    monkeypatch.setattr(can_client.socket, 'socket', no_netlink)
    assert _discover_can_channels() == ['can0', 'can1']
    L20CanClient._bring_up_socketcan('can0', 1000000)


def test_missing_interface_and_non_can_interface_fail_without_subprocess(can_sysfs):
    from hand_device import _discover_can_channels
    assert _discover_can_channels() == ['can0']
    with pytest.raises(FileNotFoundError):
        L20CanClient._bring_up_socketcan('can0', 1000000)
    can_entry(can_sysfs, 'ens33', kind=1)
    with pytest.raises(OSError) as exc:
        L20CanClient._bring_up_socketcan('ens33', 1000000)
    assert exc.value.errno == errno.EPROTOTYPE


def netlink_ack(error=0, sequence=1):
    return struct.pack('=IHHIIi', 20, 2, 0, sequence, 0, error)


@pytest.fixture
def route_reply(can_sysfs, monkeypatch):
    import can_client
    route = SimpleNamespace(sent=[], timeouts=[], packets=[netlink_ack()], bound=None, closed=False)
    class RouteSocket:
        def __enter__(self): return self
        def __exit__(self, *args): route.closed = True
        def settimeout(self, seconds): route.timeouts.append(seconds)
        def bind(self, address): route.bound = address
        def sendto(self, message, address): route.sent.append((message, address))
        def recv(self, size):
            if not route.packets:
                raise TimeoutError('netlink acknowledgement timed out')
            packet = route.packets.pop(0)
            if isinstance(packet, Exception): raise packet
            if callable(packet): return packet()
            return packet
    monkeypatch.setattr(can_client.socket, 'socket', lambda *args: RouteSocket())
    return route


def test_down_interface_sends_bounded_can_bitrate_and_up_request(can_sysfs, route_reply):
    can_entry(can_sysfs, flags='0x0')
    route = route_reply
    # Unrelated sequence must not count as our success acknowledgement.
    route.packets = [netlink_ack(sequence=2), netlink_ack()]
    L20CanClient._bring_up_socketcan('can0', 1000000)
    assert route.closed and route.bound == (0, 0)
    assert all(0 < value <= 1 for value in route.timeouts)
    assert len(route.sent) == 1 and route.sent[0][1] == (0, 0)
    message = route.sent[0][0]
    assert struct.unpack_from('=IHHII', message) == (len(message), 16, 5, 1, 0)
    assert struct.unpack_from('=BxHiII', message, 16) == (0, 0, 7, 1, 1)
    def attributes(data):
        values = {}
        offset = 0
        while offset < len(data):
            size, kind = struct.unpack_from('=HH', data, offset)
            values[kind] = data[offset + 4:offset + size]
            offset += (size + 3) & ~3
        return values
    info = attributes(attributes(message[32:])[18 | 0x8000])
    assert info[1] == b'can\0'
    timing = attributes(info[2 | 0x8000])[1]
    assert struct.unpack('=8I', timing) == (1000000, 0, 0, 0, 0, 0, 0, 0)


@pytest.mark.parametrize('error', [errno.EPERM, errno.EINVAL, errno.EBUSY])
def test_netlink_errors_are_reported_and_socket_closes(can_sysfs, route_reply, error):
    can_entry(can_sysfs, flags='0x0')
    route_reply.packets = [netlink_ack(-error)]
    with pytest.raises(OSError) as exc:
        L20CanClient._bring_up_socketcan('can0', 1000000)
    assert exc.value.errno == error and route_reply.closed


def test_other_hand_can_enable_adapter_first(can_sysfs, route_reply):
    entry = can_entry(can_sysfs, flags='0x0')
    def concurrent_enable():
        (entry / 'flags').write_text('0x1')
        return netlink_ack(-errno.EBUSY)
    route_reply.packets = [concurrent_enable]
    L20CanClient._bring_up_socketcan('can0', 1000000)
    assert route_reply.closed


@pytest.mark.parametrize('reply', [b'', b'bad', struct.pack('=IHHII', 64, 2, 0, 1, 0),
                                  struct.pack('=IHHII', 16, 2, 0, 1, 0),
                                  TimeoutError('missing acknowledgement')])
def test_invalid_or_missing_acknowledgement_does_not_hang(can_sysfs, route_reply, reply):
    can_entry(can_sysfs, flags='0x0')
    route_reply.packets = [reply]
    with pytest.raises((OSError, TimeoutError)):
        L20CanClient._bring_up_socketcan('can0', 1000000)
    assert route_reply.closed and all(0 < value <= 1 for value in route_reply.timeouts)


def test_missing_adapter_does_not_block_grasp_info_stop_or_sensor_stop(can_sysfs):
    from hand_device import _SingleHandController
    from adaptive_grasp_device import AdaptiveGraspPlugin
    from tactile_device import TactilePlugin
    config = {'can_channel': 'missing_adapter', 'side': 'left'}
    ctrl = _SingleHandController(config, 'left', 'missing_adapter', threading.RLock(), can_id=0x28)
    hand = SimpleNamespace(config=config, get_controller_for_side=lambda *args: ctrl)
    grasp = AdaptiveGraspPlugin(hand, {}, config)
    sensor = TactilePlugin(hand, {'auto_start': False}, config)
    try:
        sensor._start_streaming()
        deadline = time.monotonic() + 2
        while not sensor._latest_data and time.monotonic() < deadline:
            time.sleep(.005)
        assert sensor._latest_data['state'] == 'error'
        for action in ('adaptive_grasp', 'release'):
            started = time.monotonic()
            with pytest.raises(RuntimeError, match='未响应'):
                grasp.dispatch(action, {'bound_side': 'left', 'confirm_motion': True})
            assert grasp.dispatch('info', {'bound_side': 'left'})['active_action_id'] is None
            assert grasp.dispatch('stop', {'bound_side': 'left'})['state'] == 'idle'
            assert time.monotonic() - started < 2
        sensor._stop_streaming()
        assert not sensor._running()
    finally:
        sensor.stop()
        grasp.stop()
        ctrl.close()


@pytest.mark.parametrize('side, can_id', [('left', 0x28), ('right', 0x27)])
@pytest.mark.parametrize('qemu_compat', [False, True], ids=['native', 'qemu'])
def test_failed_bus_opens_can_recover_without_poisoning_global_sockets(
        socket_environment, monkeypatch, side, can_id, qemu_compat):
    import can
    socket, _, calls = socket_environment
    client = L20CanClient.__new__(L20CanClient)
    client.side, client.can_id = side, can_id
    client._qemu_compat = qemu_compat
    attempts = []
    recovered = object()
    owner = threading.get_ident()
    def open_bus(**kwargs):
        if threading.get_ident() != owner:
            raise OSError(errno.ENODEV, 'Unrelated background retry')
        attempts.append(kwargs)
        # Socket operations also model unrelated HTTP registration in this process.
        socket.socket.setsockopt(object(), 1, 2, b'value')
        if len(attempts) <= 1200:
            raise OSError(errno.ENODEV, 'CAN adapter unplugged')
        return recovered
    monkeypatch.setattr(can.interface, 'Bus', open_bus)
    monkeypatch.setattr(client, '_bring_up_socketcan', lambda *_: None)
    for _ in range(1200):
        with pytest.raises(RuntimeError, match='CAN adapter unplugged'):
            client._open_bus('can0', 1000000)
    assert client._open_bus('can0', 1000000) is recovered
    assert len(calls) == len(attempts) == 1201
    assert attempts[-1]['can_filters'] == [{'can_id': can_id, 'can_mask': 0x7FF}]


@pytest.mark.parametrize('error_number, ignored', [(errno.ENOPROTOOPT, True), (42, True),
                                                  (errno.EBADF, False), (errno.EACCES, False)])
def test_qemu_compatibility_does_not_hide_other_socket_errors(socket_environment, monkeypatch, error_number, ignored):
    socket, _, _ = socket_environment
    def failing(*_):
        raise OSError(error_number, 'socket option failed')
    monkeypatch.setattr(socket.socket, 'setsockopt', failing)
    L20CanClient._patch_socketcan_for_qemu()
    if ignored:
        assert socket.socket.setsockopt(object(), 1, 2, 3) is None
    else:
        with pytest.raises(OSError) as exc:
            socket.socket.setsockopt(object(), 1, 2, 3)
        assert exc.value.errno == error_number


def test_qemu_capture_still_decodes_can_when_ancillary_timestamp_is_absent(socket_environment):
    import can
    _, socketcan, _ = socket_environment
    L20CanClient._patch_socketcan_for_qemu()
    frame = socketcan.build_can_frame(can.Message(arbitration_id=0x28, data=[0x41, 1, 2, 3, 4, 5, 6], is_extended_id=False))
    class FakeSocket:
        def recvmsg(self, *_):
            return frame, [], 0, ('can0',)
    message = socketcan.capture_message(FakeSocket(), True)
    assert message.arbitration_id == 0x28 and message.channel == 'can0'
    assert list(message.data) == [0x41, 1, 2, 3, 4, 5, 6]
    assert message.is_rx and message.timestamp > 0


def queued_socketcan_receiver(socket_environment, monkeypatch, packets, *, qemu_compat=True):
    """Exercise the real decoder and receive loop with a queued CAN socket."""
    import can
    socket, socketcan, _ = socket_environment
    if qemu_compat:
        L20CanClient._patch_socketcan_for_qemu()
    else:
        monkeypatch.setattr(socketcan, 'capture_message', _NATIVE_CAPTURE_MESSAGE)
    queued = deque(packets)
    flags_seen, waits, received = [], [], []
    class QueuedSocket:
        def recvmsg(self, size, ancillary_size, flags=0):
            flags_seen.append(flags)
            if not queued:
                raise BlockingIOError(errno.EAGAIN, 'no queued frame')
            item = queued.popleft()
            if isinstance(item, Exception):
                raise item
            ancillary = [] if qemu_compat else [
                (socket.SOL_SOCKET, socketcan.constants.SO_TIMESTAMPNS,
                 socketcan.RECEIVED_TIMESTAMP_STRUCT.pack(123, 456))]
            return socketcan.build_can_frame(item), ancillary, 0, ('can0',)
    class QueuedBus:
        channel = 'can0'
        socket = QueuedSocket()
        def recv(self, timeout):
            waits.append(timeout)
            if not queued:
                client._running = False
                return None
            return socketcan.capture_message(self.socket, True)
    monkeypatch.setattr(socketcan, 'SocketcanBus', QueuedBus)
    client = object.__new__(L20CanClient)
    client._bus = QueuedBus()
    client._running = True
    client._recv_timeout_s = .5
    client.can_id = 0x28
    client._process_message = received.append
    return client, queued, flags_seen, waits, received


@pytest.mark.parametrize('qemu_compat', [False, True], ids=['native', 'qemu'])
def test_receive_loop_drains_a_full_matrix_without_waiting_for_each_row(
        socket_environment, monkeypatch, qemu_compat):
    import can
    frames = [can.Message(arbitration_id=0x28, data=[0xB1 + finger, row * 16] + [row] * 6,
                          is_extended_id=False) for finger in range(5) for row in range(12)]
    frames.insert(10, can.Message(arbitration_id=0x27, data=[0xB1, 0] + [255] * 6,
                                  is_extended_id=False))
    client, queued, flags, waits, received = queued_socketcan_receiver(
        socket_environment, monkeypatch, frames, qemu_compat=qemu_compat)
    original_setsockopt = socket_environment[0].socket.setsockopt
    original_capture = socket_environment[1].capture_message
    client._recv_loop()
    assert not queued and len(received) == 60
    assert all(message.arbitration_id == 0x28 for message in received)
    assert list(received[-1].data) == [0xB5, 176] + [11] * 6
    assert len(waits) == 2  # First frame, then the next ordinary wait after draining.
    assert flags[0] == 0 and all(value == socket_environment[0].MSG_DONTWAIT for value in flags[1:])
    assert socket_environment[0].socket.setsockopt is original_setsockopt
    assert socket_environment[1].capture_message is original_capture


@pytest.mark.parametrize('qemu_compat', [False, True], ids=['native', 'qemu'])
def test_receive_burst_is_bounded_and_never_waits_for_an_extra_frame(
        socket_environment, monkeypatch, qemu_compat):
    import can
    frame = can.Message(arbitration_id=0x28, data=[0xB1, 0] + [1] * 6, is_extended_id=False)
    client, queued, flags, waits, received = queued_socketcan_receiver(
        socket_environment, monkeypatch, [frame] * 130, qemu_compat=qemu_compat)
    client._drain_ready_messages(client._bus)
    assert len(received) == 64 and len(queued) == 66 and not waits
    assert all(value == socket_environment[0].MSG_DONTWAIT for value in flags)
    queued.clear()
    client._drain_ready_messages(client._bus)
    assert len(received) == 64 and len(flags) == 65 and not waits


@pytest.mark.parametrize('change', ['stop', 'reconnect'])
def test_receive_burst_stops_on_shutdown_or_connection_change(socket_environment, monkeypatch, change):
    import can
    frame = can.Message(arbitration_id=0x28, data=[0xB1, 0] + [1] * 6, is_extended_id=False)
    client, queued, flags, _, received = queued_socketcan_receiver(socket_environment, monkeypatch, [frame] * 12)
    def process(message):
        received.append(message)
        if change == 'stop':
            client._running = False
        else:
            client._bus = object()
    client._process_message = process
    client._drain_ready_messages(client._bus)
    assert len(received) == 1 and len(queued) == 11 and len(flags) == 1


def test_receive_burst_does_not_treat_unplugged_socket_as_empty_queue(socket_environment, monkeypatch):
    import can
    client, _, flags, _, received = queued_socketcan_receiver(
        socket_environment, monkeypatch, [OSError(errno.ENODEV, 'adapter unplugged')])
    with pytest.raises(can.CanOperationError) as failure:
        client._drain_ready_messages(client._bus)
    assert failure.value.error_code == errno.ENODEV and len(flags) == 1 and not received


def test_other_can_backends_keep_their_normal_receive_path(socket_environment, monkeypatch):
    client, _, flags, _, received = queued_socketcan_receiver(socket_environment, monkeypatch, [])
    client._bus = SimpleNamespace(socket=client._bus.socket, channel='can0')
    client._drain_ready_messages(client._bus)
    assert not flags and not received
