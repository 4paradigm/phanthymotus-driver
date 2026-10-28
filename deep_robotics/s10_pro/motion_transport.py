"""Bounded S10 V1.0.2 single-frame JSON codec and opt-in TLS transport.

Protocol source: https://support.deeprobotics.cn/docs/api/pages/259676015
This module never opens a socket at import or construction. It is not a verified
motion backend: a valid frame, TLS connection, or successful protocol response
does not establish control ownership or confirm a physical action.
"""
from __future__ import annotations

import copy
import hashlib
import ipaddress
import json
import math
import re
import socket
import ssl
import struct
import threading
import time
from collections import deque
from dataclasses import dataclass
from datetime import datetime


PROTOCOL_SOURCE = "https://support.deeprobotics.cn/docs/api/pages/259676015"
SYNC = b"\xeb\x91\xeb\x90"
HEADER = struct.Struct("<4sHHBBB5s")
MAX_ASDU = 1000
MAX_FEED = 8192
HEARTBEAT = (0x00100064, 0x00000005)
MOTION_STATE = (0x00100001, 0x00200002)
AXIS = (0x00100001, 0x00100002)
VELOCITY = (0x00100001, 0x00110002)
NO_RESPONSE = frozenset({AXIS, VELOCITY})
ALLOWED_REQUESTS = frozenset({HEARTBEAT, MOTION_STATE, AXIS, VELOCITY})
VELOCITY_LIMITS = {"X": 1.67, "Y": .4, "Yaw": 1.0}


class ProtocolError(ValueError):
    pass


class TransportDisabled(RuntimeError):
    pass


class NotConnected(RuntimeError):
    pass


class RequestCancelled(RuntimeError):
    pass


def _integer(value, name, maximum):
    if isinstance(value, bool) or not isinstance(value, int) or not 0 <= value <= maximum:
        raise ValueError(name + " must be an unsigned integer in range")
    return value


def _unique_object(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ProtocolError("duplicate JSON key")
        result[key] = value
    return result


def _body(payload):
    if not isinstance(payload, dict) or set(payload) != {"PatrolDevice"}:
        raise ProtocolError("expected PatrolDevice envelope")
    body = payload["PatrolDevice"]
    if not isinstance(body, dict) or not {"Type", "Command", "Time", "Items"} <= set(body):
        raise ProtocolError("incomplete PatrolDevice object")
    _integer(body["Type"], "Type", 0xFFFFFFFF)
    _integer(body["Command"], "Command", 0xFFFFFFFF)
    if not isinstance(body["Time"], str) or len(body["Time"]) > 64:
        raise ProtocolError("invalid Time")
    if not isinstance(body["Items"], dict):
        raise ProtocolError("Items must be an object")
    return body


@dataclass(frozen=True)
class Frame:
    message_id: int
    payload: dict


def encode_frame(message_id, message_type, command, items=None, *, timestamp=None):
    """Encode one documented packet; segmentation and XML are unsupported."""
    _integer(message_id, "message_id", 0xFFFF)
    _integer(message_type, "message_type", 0xFFFFFFFF)
    _integer(command, "command", 0xFFFFFFFF)
    if items is None:
        items = {}
    if not isinstance(items, dict):
        raise ValueError("items must be an object")
    if timestamp is None:
        timestamp = datetime.now().astimezone().strftime("%Y-%m-%d %H:%M:%S")
    if not isinstance(timestamp, str) or not re.fullmatch(r"\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2}", timestamp):
        raise ValueError("timestamp must use the documented local-time format")
    payload = {"PatrolDevice": {"Type": message_type, "Command": command,
                                "Time": timestamp, "Items": items}}
    raw = json.dumps(payload, ensure_ascii=False, separators=(",", ":"), allow_nan=False).encode("utf-8")
    if not 0 < len(raw) <= MAX_ASDU:
        raise ProtocolError("single-frame ASDU exceeds documented 1000-byte limit")
    return HEADER.pack(SYNC, len(raw), message_id, 1, 0, 1, b"\0" * 5) + raw


def _header(raw):
    if not isinstance(raw, (bytes, bytearray)) or len(raw) < HEADER.size:
        raise ProtocolError("incomplete header")
    sync, length, message_id, fmt, packet_index, version, reserved = HEADER.unpack_from(raw)
    if sync != SYNC or fmt != 1 or version != 1 or reserved != b"\0" * 5:
        raise ProtocolError("unsupported S10 header")
    if packet_index != 0:
        raise ProtocolError("segmented ASDU is unsupported")
    if not 0 < length <= MAX_ASDU:
        raise ProtocolError("invalid single-frame ASDU length")
    return length, message_id


def decode_frame(raw):
    length, message_id = _header(raw)
    if len(raw) != HEADER.size + length:
        raise ProtocolError("ASDU length mismatch")
    try:
        payload = json.loads(raw[HEADER.size:].decode("utf-8"), object_pairs_hook=_unique_object,
            parse_constant=lambda _: (_ for _ in ()).throw(ProtocolError("nonfinite JSON")))
        _body(payload)
    except (UnicodeDecodeError, json.JSONDecodeError, RecursionError, ValueError) as exc:
        raise ProtocolError("invalid JSON ASDU") from exc
    return Frame(message_id, payload)


class StreamDecoder:
    """Strict bounded TLS-stream framing; malformed data closes the session."""
    def __init__(self):
        self._buffer = bytearray()

    def feed(self, data):
        if not isinstance(data, (bytes, bytearray)) or len(data) > MAX_FEED:
            self._buffer.clear()
            raise ProtocolError("input chunk exceeds bounded receive size")
        self._buffer.extend(data)
        result = []
        try:
            while len(self._buffer) >= HEADER.size:
                length, _ = _header(self._buffer)
                total = HEADER.size + length
                if len(self._buffer) < total:
                    break
                result.append(decode_frame(bytes(self._buffer[:total])))
                del self._buffer[:total]
            if len(self._buffer) > HEADER.size + MAX_ASDU:
                raise ProtocolError("receive buffer exceeds single-frame bound")
        except BaseException:
            self._buffer.clear()
            raise
        return result


def motion_state_items(action):
    if action not in ("stand", "lie"):
        raise ValueError("only documented stand and lie actions are supported")
    return {"MotionParam": {"stand": 1, "lie": 4}[action]}


def axis_items(x=0.0, y=0.0, yaw=0.0):
    """Normalized axes, not a claim of physical velocity units."""
    values = (x, y, yaw)
    if any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) or abs(v) > 1 for v in values):
        raise ValueError("normalized axes must be finite numbers in [-1, 1]")
    return {"X": float(x), "Y": float(y), "Z": 0.0, "Roll": 0.0, "Pitch": 0.0, "Yaw": float(yaw)}


def velocity_items(vx=0.0, vy=0.0, yaw_rate=0.0):
    """Documented navigation velocity in m/s, m/s and rad/s, respectively.

    These protocol maxima do not replace the controller's lower initial-test
    limits, ownership checks, duration bounds, or confirmed navigation mode.
    """
    items = {"X": vx, "Y": vy, "Z": 0.0, "Roll": 0.0, "Pitch": 0.0, "Yaw": yaw_rate}
    return _request_items(VELOCITY, items)


def _request_items(pair, items):
    items = {} if items is None else items
    if not isinstance(items, dict):
        raise ValueError("items must be an object")
    if pair == HEARTBEAT:
        if items:
            raise ValueError("heartbeat takes no items")
    elif pair == MOTION_STATE:
        if set(items) != {"MotionParam"} or type(items["MotionParam"]) is not int or items["MotionParam"] not in (1, 4):
            raise ValueError("only documented stand/lie state commands are permitted")
    else:
        if set(items) != {"X", "Y", "Z", "Roll", "Pitch", "Yaw"}:
            raise ValueError("the six documented motion fields are required")
        if any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) for v in items.values()):
            raise ValueError("motion values must be finite numbers")
        if any(items[key] != 0 for key in ("Z", "Roll", "Pitch")):
            raise ValueError("initial transport excludes height and body attitude commands")
        if pair == AXIS and any(abs(v) > 1 for v in items.values()):
            raise ValueError("normalized axes must be in [-1, 1]")
        if pair == VELOCITY and any(abs(items[key]) > bound for key, bound in VELOCITY_LIMITS.items()):
            raise ValueError("navigation velocity exceeds documented S10 protocol maximum")
    return copy.deepcopy(items)


class S10TLSClient:
    """Explicit opt-in TLS I/O; no automatic heartbeat, reconnect or motion.

    CA/hostname validation and an independently supplied leaf-certificate pin
    are all required. Request timeout closes the connection to prevent late
    responses from satisfying a later request. Velocity frames have no protocol
    acknowledgement and therefore return accepted=None, never success.
    """
    def __init__(self, *, host="10.21.33.103", port=30003, enabled=False,
                 ca_file=None, certificate_sha256=None, server_hostname=None,
                 timeout_s=0.5, socket_factory=socket.create_connection,
                 context_factory=ssl.create_default_context, clock=time.monotonic):
        if not isinstance(enabled, bool):
            raise ValueError("enabled must be boolean")
        ipaddress.ip_address(host)  # Avoid unbounded DNS resolution in connect.
        _integer(port, "port", 0xFFFF)
        if not port:
            raise ValueError("port must be nonzero")
        if isinstance(timeout_s, bool) or not isinstance(timeout_s, (float, int)) or not math.isfinite(timeout_s) or not .05 <= timeout_s <= 1.0:
            raise ValueError("timeout_s must be finite and in [0.05, 1.0]")
        if certificate_sha256 is not None and (not isinstance(certificate_sha256, str) or not re.fullmatch(r"[0-9a-fA-F]{64}", certificate_sha256)):
            raise ValueError("certificate_sha256 must be a SHA256 hex digest")
        if enabled and (not ca_file or not certificate_sha256):
            raise ValueError("enabled TLS transport requires a CA file and independent certificate pin")
        if server_hostname is not None and (not isinstance(server_hostname, str) or not server_hostname or len(server_hostname) > 253):
            raise ValueError("invalid server_hostname")
        self.host, self.port = host, port
        self.enabled, self.ca_file = enabled, ca_file
        self.certificate_sha256 = certificate_sha256.lower() if certificate_sha256 else None
        self.server_hostname = server_hostname or host
        self.timeout_s, self._clock = float(timeout_s), clock
        self._socket_factory, self._context_factory = socket_factory, context_factory
        self._socket = None
        self._io_lock, self._state_lock = threading.Lock(), threading.Lock()
        self._decoder, self._reports = StreamDecoder(), deque(maxlen=32)
        self._message_id = 0
        self._generation = 0

    @property
    def connected(self):
        with self._state_lock:
            return self._socket is not None

    def connect(self):
        if not self.enabled:
            raise TransportDisabled("TLS transport is disabled")
        if not self._io_lock.acquire(timeout=self.timeout_s):
            raise TimeoutError("TLS I/O is busy")
        raw = wrapped = None
        try:
            if self.connected:
                return
            with self._state_lock:
                generation = self._generation
            context = self._context_factory(cafile=self.ca_file)
            if context.verify_mode != ssl.CERT_REQUIRED or not context.check_hostname:
                raise ValueError("TLS certificate and hostname verification are mandatory")
            raw = self._socket_factory((self.host, self.port), timeout=self.timeout_s)
            raw.settimeout(self.timeout_s)
            wrapped = context.wrap_socket(raw, server_hostname=self.server_hostname)
            wrapped.settimeout(self.timeout_s)
            certificate = wrapped.getpeercert(binary_form=True)
            if not certificate or hashlib.sha256(certificate).hexdigest() != self.certificate_sha256:
                raise ssl.SSLCertVerificationError("robot certificate pin mismatch")
            with self._state_lock:
                if generation != self._generation:
                    raise RequestCancelled("connection was cancelled by close")
                self._socket = wrapped
                self._decoder = StreamDecoder()
        except BaseException:
            for item in (wrapped, raw):
                if item is not None:
                    try:
                        item.close()
                    except OSError:
                        pass
            raise
        finally:
            self._io_lock.release()

    def _drop(self, active):
        with self._state_lock:
            if self._socket is active:
                self._socket = None
        if active is not None:
            try:
                active.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            try:
                active.close()
            except OSError:
                pass

    def close(self):
        # Closing the socket also interrupts a request waiting for its response.
        with self._state_lock:
            self._generation += 1
            active, self._socket = self._socket, None
        self._drop(active)

    def reports(self):
        with self._state_lock:
            return copy.deepcopy(list(self._reports))

    def request(self, message_type, command, items=None, *, cancel_event=None, timestamp=None):
        if not self.enabled:
            raise TransportDisabled("TLS transport is disabled")
        _integer(message_type, "message_type", 0xFFFFFFFF)
        _integer(command, "command", 0xFFFFFFFF)
        pair = (message_type, command)
        if pair not in ALLOWED_REQUESTS:
            raise ValueError("command is outside the initial S10 transport allowlist")
        items = _request_items(pair, items)
        if cancel_event is not None and cancel_event.is_set():
            raise RequestCancelled("request cancelled before send")
        deadline = self._clock() + self.timeout_s
        while not self._io_lock.acquire(timeout=min(.05, self.timeout_s)):
            if cancel_event is not None and cancel_event.is_set():
                raise RequestCancelled("request cancelled while waiting for I/O")
            if self._clock() >= deadline:
                raise TimeoutError("TLS I/O is busy")
        active = None
        try:
            with self._state_lock:
                active = self._socket
            if active is None:
                raise NotConnected("explicit connect is required")
            if cancel_event is not None and cancel_event.is_set():
                raise RequestCancelled("request cancelled before send")
            message_id = self._message_id
            self._message_id = (message_id + 1) & 0xFFFF
            frame = encode_frame(message_id, message_type, command, items, timestamp=timestamp)
            remaining = deadline - self._clock()
            if remaining <= 0:
                raise TimeoutError("request expired before send")
            active.settimeout(remaining)
            if cancel_event is not None and cancel_event.is_set():
                raise RequestCancelled("request cancelled immediately before send")
            active.sendall(frame)
            receipt = {"message_id": message_id, "message_type": message_type, "command": command,
                       "sent": True, "acknowledged": False, "accepted": None, "feedback_confirmed": False}
            if pair in NO_RESPONSE:
                return receipt
            while self._clock() < deadline:
                if cancel_event is not None and cancel_event.is_set():
                    raise RequestCancelled("request cancelled while waiting for response")
                active.settimeout(min(.05, max(.001, deadline - self._clock())))
                try:
                    raw = active.recv(4096)
                except socket.timeout:
                    continue
                if not raw:
                    raise ConnectionError("TLS peer closed before response")
                matching = None
                for received in self._decoder.feed(raw):
                    body = _body(received.payload)
                    if received.message_id == message_id and (body["Type"], body["Command"]) == pair:
                        if matching is not None:
                            raise ProtocolError("duplicate matching protocol response")
                        matching = body
                    else:
                        with self._state_lock:
                            self._reports.append({"message_id": received.message_id,
                                "received_monotonic": self._clock(), "payload": received.payload})
                if matching is not None:
                    error = matching["Items"].get("ErrorCode")
                    _integer(error, "ErrorCode", 0xFFFFFFFF)
                    message = matching["Items"].get("ErrorMessage", "")
                    if not isinstance(message, str) or len(message) > 512:
                        raise ProtocolError("invalid protocol response message")
                    receipt.update(acknowledged=True, accepted=error == 0, error_code=error, error_message=message)
                    return receipt
            raise TimeoutError("protocol response deadline exceeded")
        except BaseException:
            # A failed/cancelled transaction cannot leave a session or late ACK
            # available for a subsequent request. No implicit retry is issued.
            self._drop(active)
            raise
        finally:
            self._io_lock.release()
