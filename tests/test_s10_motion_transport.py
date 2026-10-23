"""Documented S10 framing and TLS boundary, entirely against fake sockets."""
import hashlib
import json
import socket
import ssl
import struct
import threading
import time

import pytest

from deep_robotics.s10_pro.motion_transport import (
    ALLOWED_REQUESTS, AXIS, HEARTBEAT, HEADER, MAX_ASDU, MOTION_STATE, SYNC, VELOCITY,
    NotConnected, ProtocolError, RequestCancelled, S10TLSClient, StreamDecoder,
    TransportDisabled, axis_items, decode_frame, encode_frame, motion_state_items,
    velocity_items,
)


STAMP = "2026-10-22 16:00:00"
CERTIFICATE = b"offline fixture certificate, not a robot certificate"
PIN = hashlib.sha256(CERTIFICATE).hexdigest()


def encoded(message_id=0, pair=HEARTBEAT, items=None):
    return encode_frame(message_id, *pair, items, timestamp=STAMP)


def ack(message_id=0, pair=HEARTBEAT, error=0):
    return encoded(message_id, pair, {"ErrorCode": error, "ErrorMessage": "Success" if not error else "Rejected"})


def test_s10_header_matches_independent_documented_layout():
    raw = encoded(0x2345, MOTION_STATE, {"MotionParam": 1})
    payload = raw[16:]
    expected = (b"\xeb\x91\xeb\x90" + len(payload).to_bytes(2, "little")
                + b"\x45\x23\x01\x00\x01" + b"\0" * 5)
    assert raw[:16] == expected
    body = json.loads(payload)["PatrolDevice"]
    assert body == {"Type": 0x00100001, "Command": 0x00200002, "Time": STAMP, "Items": {"MotionParam": 1}}
    assert decode_frame(raw).message_id == 0x2345


@pytest.mark.parametrize("offset,value", [(0, 0), (8, 0), (9, 1), (10, 0), (11, 1), (15, 1)])
def test_unsupported_header_and_fragment_are_rejected(offset, value):
    raw = bytearray(encoded())
    raw[offset] = value
    with pytest.raises(ProtocolError):
        decode_frame(raw)


@pytest.mark.parametrize("raw", [b"", b"\xeb" * 15, encoded()[:-1], encoded() + b"extra"])
def test_truncated_and_overlong_frames_are_rejected(raw):
    with pytest.raises(ProtocolError):
        decode_frame(raw)


@pytest.mark.parametrize("length", [0, 1001, 65535])
def test_declared_length_is_bounded_before_payload_read(length):
    raw = HEADER.pack(SYNC, length, 0, 1, 0, 1, b"\0" * 5)
    with pytest.raises(ProtocolError):
        StreamDecoder().feed(raw)


@pytest.mark.parametrize("message_id", [-1, 65536, True, 1.0])
def test_message_id_is_not_truncated_or_coerced(message_id):
    with pytest.raises(ValueError):
        encoded(message_id)


def test_encoding_rejects_nonfinite_and_oversized_data():
    with pytest.raises(ValueError):
        encoded(items={"x": float("nan")})
    with pytest.raises(ProtocolError):
        encoded(items={"x": "a" * MAX_ASDU})


@pytest.mark.parametrize("payload", [
    b'{"PatrolDevice":{"Type":1,"Type":2,"Command":3,"Time":"x","Items":{}}}',
    b'{"PatrolDevice":{"Type":NaN,"Command":3,"Time":"x","Items":{}}}',
    b'{"PatrolDevice":{"Type":true,"Command":3,"Time":"x","Items":{}}}',
    b'{"PatrolDevice":{"Type":1,"Command":3,"Time":"x","Items":[]}}',
    b'{"PatrolDevice":{"Type":1,"Command":3,"Time":"x","Items":{}},"extra":0}',
    b'\xff',
])
def test_untrusted_json_is_strict(payload):
    frame = HEADER.pack(SYNC, len(payload), 0, 1, 0, 1, b"\0" * 5) + payload
    with pytest.raises(ProtocolError):
        decode_frame(frame)


def test_partial_and_coalesced_tls_reads_produce_complete_frames_only():
    first, second = encoded(1), encoded(2)
    decoder = StreamDecoder()
    assert decoder.feed(first[:7]) == []
    assert decoder.feed(first[7:-1]) == []
    frames = decoder.feed(first[-1:] + second)
    assert [frame.message_id for frame in frames] == [1, 2]
    assert decoder.feed(b"") == []


def test_stream_input_and_resynchronization_are_fail_closed():
    with pytest.raises(ProtocolError):
        StreamDecoder().feed(b"x" * 8193)
    with pytest.raises(ProtocolError):
        StreamDecoder().feed(b"garbage-prefix!!" + encoded())


def test_decoder_does_not_retain_malformed_input_after_failure():
    decoder = StreamDecoder()
    for _ in range(10):
        with pytest.raises(ProtocolError):
            decoder.feed(b"x" * 8192)
        assert not decoder._buffer
    assert len(decoder.feed(encoded())) == 1


def test_state_and_axes_helpers_do_not_copy_m20_estop_semantics():
    assert motion_state_items("stand") == {"MotionParam": 1}
    assert motion_state_items("lie") == {"MotionParam": 4}
    for action in ("stop", "soft_estop", "reset", "rl", "calibrate"):
        with pytest.raises(ValueError):
            motion_state_items(action)
    assert axis_items() == dict.fromkeys(("X", "Y", "Z", "Roll", "Pitch", "Yaw"), 0.0)
    assert axis_items(-.1, .2, -.3)["Yaw"] == -.3


@pytest.mark.parametrize("value", [float("inf"), float("nan"), True, "0", 1.01, -1.01])
def test_normalized_axis_validation(value):
    with pytest.raises(ValueError):
        axis_items(value)


def test_physical_velocity_units_have_documented_protocol_limits():
    assert velocity_items(1.67, -.4, 1)["X"] == 1.67
    assert velocity_items()["Yaw"] == 0
    for values in [(1.671, 0, 0), (0, .401, 0), (0, 0, -1.001), (True, 0, 0)]:
        with pytest.raises(ValueError):
            velocity_items(*values)


class FakeSocket:
    def __init__(self, reads=(), certificate=CERTIFICATE):
        self.reads = list(reads)
        self.certificate = certificate
        self.sent = []
        self.timeout = .05
        self.closed = False

    def settimeout(self, value):
        assert 0 < value <= 1
        self.timeout = value

    def getpeercert(self, binary_form=False):
        assert binary_form
        return self.certificate

    def sendall(self, data):
        if self.closed:
            raise OSError("closed")
        self.sent.append(data)

    def recv(self, size):
        assert size == 4096
        if self.closed:
            return b""
        if self.reads:
            value = self.reads.pop(0)
            if isinstance(value, BaseException):
                raise value
            return value
        time.sleep(min(self.timeout, .003))
        raise socket.timeout()

    def shutdown(self, how):
        assert how == socket.SHUT_RDWR
        self.closed = True

    def close(self):
        self.closed = True


class FakeContext:
    verify_mode = ssl.CERT_REQUIRED
    check_hostname = True

    def __init__(self, active):
        self.active = active
        self.server_hostname = None

    def wrap_socket(self, raw, *, server_hostname):
        assert raw is self.active
        self.server_hostname = server_hostname
        return self.active


def client_fixture(reads=(), **overrides):
    active = FakeSocket(reads)
    context = FakeContext(active)
    calls = []

    def create(address, *, timeout):
        calls.append((address, timeout))
        return active

    def context_factory(**options):
        assert options == {"cafile": "offline-ca.pem"}
        return context

    options = dict(enabled=True, ca_file="offline-ca.pem", certificate_sha256=PIN,
                   server_hostname="robot.example.invalid", timeout_s=.05,
                   socket_factory=create, context_factory=context_factory)
    options.update(overrides)
    return S10TLSClient(**options), active, context, calls


def test_transport_is_lazy_and_requires_explicit_connection():
    client, active, context, calls = client_fixture([ack()])
    assert not calls and not client.connected
    with pytest.raises(NotConnected):
        client.request(*HEARTBEAT)
    assert not calls and not active.sent
    client.connect()
    assert len(calls) == 1 and client.connected
    assert context.server_hostname == "robot.example.invalid"
    client.connect()
    assert len(calls) == 1
    client.close()
    assert active.closed and not client.connected


def test_default_disabled_client_cannot_connect_or_send():
    calls = []
    client = S10TLSClient(socket_factory=lambda *a, **k: calls.append(a))
    with pytest.raises(TransportDisabled):
        client.connect()
    with pytest.raises(TransportDisabled):
        client.request(*HEARTBEAT)
    assert calls == []


@pytest.mark.parametrize("options", [
    {"ca_file": None}, {"certificate_sha256": None}, {"certificate_sha256": "bad"},
    {"timeout_s": 0}, {"timeout_s": 1.1}, {"timeout_s": float("nan")},
    {"host": "robot.example.invalid"}, {"enabled": "true"},
])
def test_opt_in_configuration_requires_bounded_verified_tls(options):
    with pytest.raises(ValueError):
        client_fixture(**options)


@pytest.mark.parametrize("verify_mode,check_hostname", [(ssl.CERT_NONE, False), (ssl.CERT_OPTIONAL, True), (ssl.CERT_REQUIRED, False)])
def test_context_cannot_disable_certificate_or_hostname_validation(verify_mode, check_hostname):
    client, active, context, calls = client_fixture()
    context.verify_mode, context.check_hostname = verify_mode, check_hostname
    with pytest.raises(ValueError):
        client.connect()
    assert not calls and not client.connected


def test_pin_mismatch_closes_socket_and_never_sends_application_data():
    client, active, _, _ = client_fixture(certificate_sha256="0" * 64)
    with pytest.raises(ssl.SSLCertVerificationError):
        client.connect()
    assert active.closed and not active.sent and not client.connected


def test_close_during_connect_cannot_leave_a_late_open_socket():
    client, active, context, _ = client_fixture()
    original = context.wrap_socket

    def closing_wrap(raw, *, server_hostname):
        client.close()
        return original(raw, server_hostname=server_hostname)

    context.wrap_socket = closing_wrap
    with pytest.raises(RequestCancelled):
        client.connect()
    assert active.closed and not client.connected


def test_protocol_ack_is_accepted_but_never_physical_confirmation():
    client, active, _, _ = client_fixture([ack()])
    client.connect()
    receipt = client.request(*HEARTBEAT, timestamp=STAMP)
    assert receipt["message_id"] == 0
    assert receipt["sent"] and receipt["acknowledged"] and receipt["accepted"]
    assert receipt["feedback_confirmed"] is False
    assert decode_frame(active.sent[0]).message_id == 0


def test_error_response_is_not_success():
    client, _, _, _ = client_fixture([ack(error=0xE006)])
    client.connect()
    receipt = client.request(*HEARTBEAT)
    assert receipt["accepted"] is False and receipt["error_code"] == 0xE006
    assert receipt["feedback_confirmed"] is False


@pytest.mark.parametrize("pair", [AXIS, VELOCITY])
def test_motion_streams_have_no_ack_and_never_claim_accepted(pair):
    client, active, _, _ = client_fixture()
    client.connect()
    result = client.request(*pair, axis_items(.1, -.1, .05))
    assert result["sent"] is True
    assert result["acknowledged"] is False and result["accepted"] is None
    assert result["feedback_confirmed"] is False
    assert len(active.sent) == 1


def test_ack_must_match_id_type_and_command_and_report_queue_is_bounded():
    reports = [encoded(n + 1, (0x00100064, 0x00F00000), {"BasicStatus": {"MotionState": 1}}) for n in range(36)]
    client, _, _, _ = client_fixture(reports + [ack()])
    client.connect()
    assert client.request(*HEARTBEAT)["accepted"] is True
    saved = client.reports()
    assert len(saved) == 32 and saved[0]["message_id"] == 5
    assert all(isinstance(report["received_monotonic"], float) for report in saved)
    saved[0]["payload"].clear()
    assert client.reports()[0]["payload"]


@pytest.mark.parametrize("reply", [ack(1), ack(0, MOTION_STATE), b"", encoded(items={})])
def test_unmatched_missing_or_malformed_response_cannot_succeed(reply):
    client, active, _, _ = client_fixture([reply])
    client.connect()
    with pytest.raises((TimeoutError, ConnectionError, ValueError)):
        client.request(*HEARTBEAT)
    assert active.closed and not client.connected


def test_cancelled_request_never_sends_and_does_not_implicitly_reconnect():
    client, active, _, calls = client_fixture()
    cancel = threading.Event()
    cancel.set()
    with pytest.raises(RequestCancelled):
        client.request(*HEARTBEAT, cancel_event=cancel)
    assert not calls and not active.sent
    client.connect()
    with pytest.raises(RequestCancelled):
        client.request(*HEARTBEAT, cancel_event=cancel)
    assert not active.sent


def test_cancellation_interrupts_response_wait_and_discards_session():
    client, active, _, _ = client_fixture(timeout_s=.2)
    client.connect()
    cancel = threading.Event()
    failures = []

    def request():
        try:
            client.request(*HEARTBEAT, cancel_event=cancel)
        except Exception as exc:
            failures.append(exc)

    worker = threading.Thread(target=request)
    worker.start()
    end = time.monotonic() + 1
    while not active.sent and time.monotonic() < end:
        time.sleep(.001)
    cancel.set()
    worker.join(timeout=.3)
    assert not worker.is_alive() and isinstance(failures[0], RequestCancelled)
    assert not client.connected and active.closed


def test_request_ids_wrap_without_replaying_an_old_ack():
    client, active, _, _ = client_fixture()
    client.connect()
    client._message_id = 65535
    first = client.request(*AXIS, axis_items())
    second = client.request(*AXIS, axis_items())
    assert (first["message_id"], second["message_id"]) == (65535, 0)
    assert [decode_frame(raw).message_id for raw in active.sent] == [65535, 0]


@pytest.mark.parametrize("pair,items", [
    ((0x00100002, 0x00500002), {"Mode": 1}),  # No implicit mode switching.
    (MOTION_STATE, {"MotionParam": -2}),  # Read-only state, not a stop command.
    (MOTION_STATE, {"MotionParam": 5}),  # No calibration command.
    (MOTION_STATE, {"MotionParam": True}),
    (HEARTBEAT, {"extra": 1}),
    (AXIS, {"X": .1}),
    (AXIS, {**axis_items(), "Z": .1}),
    (AXIS, {**axis_items(), "X": 1.1}),
    (VELOCITY, {**axis_items(), "Yaw": float("inf")}),
    (VELOCITY, {**axis_items(), "X": 1.68}),
    (VELOCITY, {**axis_items(), "Y": -.41}),
    (VELOCITY, {**axis_items(), "Yaw": 1.01}),
])
def test_initial_request_allowlist_excludes_unplanned_controls(pair, items):
    client, active, _, _ = client_fixture()
    client.connect()
    with pytest.raises(ValueError):
        client.request(*pair, items)
    assert not active.sent and client.connected
