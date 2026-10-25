"""QEMU SocketCAN reconnect regressions; no real socket or hardware access."""
import errno
import sys
import threading
from concurrent.futures import ThreadPoolExecutor

import pytest
from test_linkerhand_l20 import L20CanClient


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


@pytest.mark.parametrize('side, can_id', [('left', 0x28), ('right', 0x27)])
def test_failed_bus_opens_can_recover_without_poisoning_global_sockets(socket_environment, monkeypatch, side, can_id):
    import can
    socket, _, calls = socket_environment
    client = L20CanClient.__new__(L20CanClient)
    client.side, client.can_id = side, can_id
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
