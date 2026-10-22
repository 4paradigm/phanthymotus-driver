#!/usr/bin/env python3
"""Lynx S10 documented TLS/TCP basic_server framing and client."""

from __future__ import annotations

import json
import socket
import ssl
import struct
import threading
import time
from datetime import datetime


SYNC = b"\xeb\x91\xeb\x90"
HEADER = struct.Struct("<4sHHBBB5x")
MAX_ASDU = 1000


def encode_frame(message_id: int, message_type: int, command: int, items: dict | None = None) -> bytes:
    payload = json.dumps({"PatrolDevice": {
        "Type": int(message_type), "Command": int(command),
        "Time": datetime.now().astimezone().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3],
        "Items": items or {},
    }}, ensure_ascii=False, separators=(",", ":")).encode("utf-8")
    if len(payload) > MAX_ASDU:
        raise ValueError(f"ASDU exceeds documented {MAX_ASDU}-byte limit")
    return HEADER.pack(SYNC, len(payload), message_id & 0xFFFF, 1, 0, 1) + payload


def decode_frame(frame: bytes) -> tuple[int, dict]:
    if len(frame) < HEADER.size:
        raise ValueError("incomplete S10 frame header")
    sync, length, message_id, data_format, packet_number, version = HEADER.unpack_from(frame)
    if sync != SYNC or data_format != 1 or packet_number != 0 or version != 1:
        raise ValueError("invalid S10 sync or unsupported non-JSON ASDU")
    if length > MAX_ASDU or len(frame) != HEADER.size + length:
        raise ValueError("invalid S10 ASDU length")
    return message_id, json.loads(frame[HEADER.size:].decode("utf-8"))


class StreamDecoder:
    def __init__(self):
        self.buffer = bytearray()
        self.fragments = {}

    def feed(self, data: bytes) -> list[tuple[int, dict]]:
        self.buffer.extend(data)
        frames = []
        while True:
            offset = self.buffer.find(SYNC)
            if offset < 0:
                self.buffer[:] = self.buffer[-3:]
                break
            if offset:
                del self.buffer[:offset]
            if len(self.buffer) < HEADER.size:
                break
            sync, length, message_id, data_format, packet_number, version = HEADER.unpack_from(self.buffer)
            if sync != SYNC or data_format != 1 or version != 1:
                del self.buffer[0]
                continue
            if length > MAX_ASDU:
                del self.buffer[0]
                continue
            total = HEADER.size + length
            if len(self.buffer) < total:
                break
            chunk = bytes(self.buffer[HEADER.size:total])
            del self.buffer[:total]
            if packet_number == 0:
                try:
                    payload = json.loads(chunk.decode("utf-8"))
                except (UnicodeDecodeError, json.JSONDecodeError):
                    payload = None
                if payload is not None:
                    frames.append((message_id, payload))
                    self.fragments.pop(message_id, None)
                    continue
                self.fragments[message_id] = {"next": 1, "payload": bytearray(chunk)}
                continue
            fragment = self.fragments.get(message_id)
            if fragment is None or packet_number != fragment["next"]:
                self.fragments.pop(message_id, None)
                continue
            if len(fragment["payload"]) + length > 65535:
                self.fragments.pop(message_id, None)
                continue
            fragment["payload"].extend(chunk)
            fragment["next"] += 1
            if length < MAX_ASDU:
                frames.append((message_id, json.loads(fragment["payload"].decode("utf-8"))))
                self.fragments.pop(message_id, None)
        return frames


class BasicServerClient:
    """TLS/TCP native client. Certificate verification stays enabled by default."""

    def __init__(self, config: dict):
        native = config.get("basic_server", {})
        self.host = str(native.get("host", "10.21.33.103"))
        self.port = int(native.get("tls_tcp_port", 30003))
        self.timeout = float(native.get("timeout", 2.0))
        self.config = config
        self.enabled = bool(native.get("enabled", True))
        self.context = ssl.create_default_context(cafile=native.get("ca_cert") or None)
        self._socket = None
        self._decoder = StreamDecoder()
        self._lock = threading.RLock()
        self._id_lock = threading.Lock()
        self._message_id = -1
        self._stop = threading.Event()
        self._thread = None
        self._reader_thread = None
        self._state_lock = threading.Lock()
        self._response_condition = threading.Condition()
        self._responses = {}
        self._pending = {}
        self.reports = {}
        self.last_error = None

    def _next_id(self):
        with self._id_lock:
            self._message_id = (self._message_id + 1) & 0xFFFF
            return self._message_id

    def _connect(self):
        if self._socket is None:
            raw = socket.create_connection((self.host, self.port), timeout=self.timeout)
            try:
                self._socket = self.context.wrap_socket(raw, server_hostname=self.host)
                self._socket.settimeout(None)
            except Exception:
                raw.close()
                raise

    def _remember(self, payload):
        body = payload.get("PatrolDevice", {})
        key = f"{body.get('Type')}:{body.get('Command')}"
        with self._state_lock:
            self.reports[key] = payload

    def request(self, message_type: int, command: int, items: dict | None = None, *, response_timeout: float | None = None):
        if not self.enabled:
            return {"state": "disabled"}
        with self._lock:
            self._connect()
            message_id = self._next_id()
            self._ensure_reader()
            with self._response_condition:
                self._pending[message_id] = (int(message_type), int(command))
            self._socket.sendall(encode_frame(message_id, message_type, command, items))
        deadline = time.monotonic() + (self.timeout if response_timeout is None else float(response_timeout))
        with self._response_condition:
            while message_id not in self._responses and time.monotonic() < deadline:
                self._response_condition.wait(deadline - time.monotonic())
            payload = self._responses.pop(message_id, None)
            self._pending.pop(message_id, None)
        if payload is None:
            raise TimeoutError("S10 TLS/TCP response timed out")
        return payload

    def send(self, message_type: int, command: int, items: dict | None = None):
        """Send a documented fire-and-forget command (axis/velocity have no reply)."""
        if not self.enabled:
            return {"state": "disabled"}
        with self._lock:
            self._connect()
            self._ensure_reader()
            message_id = self._next_id()
            self._socket.sendall(encode_frame(message_id, message_type, command, items))
            return {"state": "sent", "transport": "TLS/TCP", "message_id": message_id,
                    "type": int(message_type), "command": int(command)}

    def _ensure_reader(self):
        if self._reader_thread and self._reader_thread.is_alive():
            return
        self._reader_thread = threading.Thread(target=self._read_loop, daemon=True, name="s10-basic-server-reader")
        self._reader_thread.start()

    def _read_loop(self):
        while not self._stop.is_set():
            with self._lock:
                connection = self._socket
            if connection is None:
                self._stop.wait(0.1)
                continue
            try:
                data = connection.recv(65536)
                if not data:
                    raise ConnectionError("S10 TLS/TCP connection closed")
                for message_id, payload in self._decoder.feed(data):
                    self._remember(payload)
                    body = payload.get("PatrolDevice", {})
                    with self._response_condition:
                        if self._pending.get(message_id) == (body.get("Type"), body.get("Command")):
                            self._responses[message_id] = payload
                            self._response_condition.notify_all()
            except (OSError, ssl.SSLError, ConnectionError) as exc:
                self.last_error = str(exc)
                self.close_socket()
                with self._response_condition:
                    self._response_condition.notify_all()

    def start(self):
        if not self.enabled or self._thread:
            return
        def heartbeat():
            while not self._stop.wait(1.0):
                try:
                    self.request(0x00100064, 0x00000005)
                    self.last_error = None
                except Exception as exc:
                    self.last_error = str(exc)
                    self.close_socket()
        self._thread = threading.Thread(target=heartbeat, daemon=True, name="s10-basic-server")
        self._thread.start()

    def snapshot(self):
        with self._state_lock:
            reports = dict(self.reports)
        return {"enabled": self.enabled, "transport": "TLS/TCP", "host": self.host,
                "port": self.port, "connected": self._socket is not None,
                "last_error": self.last_error, "reports": reports}

    def close_socket(self):
        with self._lock:
            if self._socket:
                try:
                    self._socket.close()
                except OSError:
                    pass
                self._socket = None
            self._decoder = StreamDecoder()

    def close(self):
        self._stop.set()
        self.close_socket()
        for thread in (self._thread, self._reader_thread):
            if thread and thread is not threading.current_thread():
                thread.join(timeout=1.0)
