"""Golden App frames and fragmented, malformed and missing TCP responses."""

import importlib.util
import errno
import json
import struct
from pathlib import Path
from types import SimpleNamespace

import pytest


PATH = Path(__file__).resolve().parents[1] / "noetix" / "bumi" / "walking_mode_client.py"
SPEC = importlib.util.spec_from_file_location("bumi_app_client_test", PATH)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class Socket:
    def __init__(self, incoming=b""):
        self.incoming = incoming
        self.outgoing = []
        self.closed = False
        self.timeouts = []

    def settimeout(self, timeout):
        assert timeout > 0
        self.timeouts.append(timeout)

    def sendall(self, data):
        self.outgoing.append(data)

    def recv(self, size):
        # Every frame crosses many recv boundaries, like a TCP stream can.
        size = min(size, 3)
        part, self.incoming = self.incoming[:size], self.incoming[size:]
        return part

    def close(self):
        self.closed = True


def connection(monkeypatch, data):
    sock = Socket(data)
    monkeypatch.setattr(MODULE.socket, "create_connection", lambda *_args: sock)
    return sock, MODULE.WalkingModeClient("127.0.0.1")


@pytest.mark.parametrize(("command", "body"), [
    ("walk", b'{"cmd":"walk"}'),
    ("run", b'{"cmd":"run"}'),
    ("get_status", b'{"cmd":"get_status"}'),
])
def test_frame_matches_the_app_magic_length_type_and_trailer(command, body):
    expected = bytes.fromhex("be ba fe ca ef be ad de")
    expected += struct.pack("<II", len(body), 0) + body + struct.pack("<I", len(body))
    assert MODULE.encode_packet({"cmd": command}) == expected


def test_connect_is_passive_and_status_query_handles_fragmentation(monkeypatch):
    incoming = MODULE.encode_packet({"cmd": "status", "data": {"status": 3}})
    sock, client = connection(monkeypatch, incoming)
    with client:
        assert sock.outgoing == []
        assert client.get_status() == 3
    assert sock.closed
    assert sock.outgoing == [MODULE.encode_packet({"cmd": "get_status"})]


@pytest.mark.parametrize(("model", "command"), [("lower_body", "walk"), ("whole_body", "run")])
def test_switch_uses_only_the_two_app_commands(monkeypatch, model, command):
    sock, client = connection(monkeypatch, b"")
    with client:
        client.set_walking_model(model)
    assert sock.outgoing == [MODULE.encode_packet({"cmd": command})]


@pytest.mark.parametrize("incoming", [
    b"invalid magic!!!" + b"\x00" * 16,
    MODULE.MAGIC + struct.pack("<II", MODULE.MAX_BODY_BYTES + 1, 0),
    MODULE.encode_packet({"cmd": "status", "data": {"status": 3}})[:-1],
    MODULE.encode_packet({"cmd": "status", "data": {"status": 3}})[:-4] + b"\x00" * 4,
    MODULE.encode_packet({"cmd": "status", "data": {"status": True}}),
    MODULE.encode_packet({"cmd": "status", "data": []}),
])
def test_bad_response_fails_and_closes_without_sending_mode_commands(monkeypatch, incoming):
    sock, client = connection(monkeypatch, incoming)
    with pytest.raises(MODULE.AppProtocolError), client:
        client.get_status()
    assert sock.closed
    assert sock.outgoing == [MODULE.encode_packet({"cmd": "get_status"})]


def test_other_text_frames_are_skipped_without_becoming_status(monkeypatch):
    incoming = MODULE.encode_packet({"cmd": "battery", "data": {"status": 99}})
    incoming += MODULE.encode_packet({"cmd": "status", "data": {"status": 2}})
    _, client = connection(monkeypatch, incoming)
    with client:
        assert client.get_status() == 2


def test_unknown_model_sends_nothing(monkeypatch):
    sock, client = connection(monkeypatch, b"")
    with pytest.raises(KeyError), client:
        client.set_walking_model("CHEER")
    assert sock.outgoing == []


def test_command_timeout_is_shorter_than_status_timeout(monkeypatch):
    incoming = MODULE.encode_packet({"cmd": "status", "data": {"status": 2}})
    sock, client = connection(monkeypatch, incoming)
    client.timeout = 5.0
    with client:
        client.set_walking_model("whole_body")
        assert sock.timeouts == [0.1]
        assert client.get_status() == 2
        assert sock.timeouts[1] == 5.0


def test_request_deadline_caps_connect_and_command_write(monkeypatch):
    now = [0.0]
    sock = Socket()
    connect_timeouts = []
    monkeypatch.setattr(MODULE, "time", SimpleNamespace(monotonic=lambda: now[0]))

    def connect(address, timeout):
        connect_timeouts.append(timeout)
        return sock

    monkeypatch.setattr(MODULE.socket, "create_connection", connect)
    with MODULE.WalkingModeClient("127.0.0.1", timeout=15.0, deadline=2.0) as client:
        assert connect_timeouts == [2.0]
        now[0] = 1.95
        client.set_walking_model("whole_body")
        assert sock.timeouts == [pytest.approx(0.05)]
        now[0] = 2.0
        with pytest.raises(TimeoutError):
            client.set_walking_model("lower_body")
        assert sock.outgoing == [MODULE.encode_packet({"cmd": "run"})]
    assert sock.closed


def test_status_send_and_read_share_one_query_budget(monkeypatch):
    now = [0.0]
    sock, client = connection(monkeypatch, b"")
    monkeypatch.setattr(MODULE, "time", SimpleNamespace(monotonic=lambda: now[0]))

    def send(data):
        sock.outgoing.append(data)
        now[0] += 1.0

    monkeypatch.setattr(sock, "sendall", send)
    monkeypatch.setattr(sock, "recv", lambda _size: pytest.fail("query budget exhausted during send"))
    with client, pytest.raises(TimeoutError):
        client.get_status()
    assert sock.closed
    assert sock.timeouts == [1.0]


def test_status_read_is_capped_by_the_request_deadline(monkeypatch):
    now = [0.0]
    sock = Socket()
    monkeypatch.setattr(MODULE, "time", SimpleNamespace(monotonic=lambda: now[0]))
    monkeypatch.setattr(MODULE.socket, "create_connection", lambda *_args: sock)

    def recv(_size):
        now[0] += sock.timeouts[-1]
        raise TimeoutError("status timed out")

    monkeypatch.setattr(sock, "recv", recv)
    with MODULE.WalkingModeClient("127.0.0.1", timeout=15, deadline=2) as client:
        with pytest.raises(TimeoutError):
            client.get_status()
    assert now[0] == 2.0
    assert sock.timeouts == [2.0, 2.0]


def test_discovery_accepts_only_the_configured_motion_board(monkeypatch):
    expected_source = "192.168.55.102"

    class Receiver:
        received = iter([
            (b"invalid JSON", (expected_source, 1234)),
            (json.dumps({"perception_ip": "10.0.0.99"}).encode(), ("10.0.0.3", 1234)),
            (json.dumps({"perception_ip": "invalid"}).encode(), (expected_source, 1234)),
            (json.dumps({"perception_ip": "192.0.2.22"}).encode(), (expected_source, 1234)),
        ])

        def __enter__(self):
            return self

        def __exit__(self, *_args):
            pass

        def setsockopt(self, level, option, value):
            assert (level, option, value) == (MODULE.socket.SOL_SOCKET, MODULE.socket.SO_REUSEADDR, 1)
            self.reuse = True

        def bind(self, address):
            assert self.reuse
            assert address == ("0.0.0.0", 9090)

        def settimeout(self, _timeout):
            pass

        def recvfrom(self, _size):
            return next(self.received)

    monkeypatch.setattr(MODULE.socket, "socket", lambda *_args: Receiver())
    assert MODULE.discover_host(expected_source, 1.0) == "192.0.2.22"


def test_exclusive_discovery_port_reports_the_explicit_host_fallback(monkeypatch):
    class Receiver:
        def __enter__(self):
            return self

        def __exit__(self, *_args):
            pass

        def setsockopt(self, *_args):
            pass

        def bind(self, _address):
            raise OSError(errno.EADDRINUSE, "Address already in use")

    monkeypatch.setattr(MODULE.socket, "socket", lambda *_args: Receiver())
    with pytest.raises(MODULE.AppProtocolError, match="plugins.walking_mode.app_host"):
        MODULE.discover_host("192.168.55.102", 1.0)
