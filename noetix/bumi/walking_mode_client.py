"""Walking-mode commands and status queries over the robot's TCP service.

Connecting does not send a command. Mode changes are explicit, sent once, and
never retried after a transport error. No SDK controller is created here.
"""

from __future__ import annotations

import errno
import ipaddress
import json
import socket
import struct
import time


MAGIC = bytes.fromhex("be ba fe ca ef be ad de")
MODEL_COMMANDS = {"lower_body": "walk", "whole_body": "run"}
MODEL_CODES = {"lower_body": 2, "whole_body": 3}
MAX_BODY_BYTES = 1024 * 1024
COMMAND_TIMEOUT_S = 0.1


class AppProtocolError(ValueError):
    pass


def encode_packet(data: dict) -> bytes:
    body = json.dumps(data, separators=(",", ":"), ensure_ascii=False).encode("utf-8")
    return MAGIC + struct.pack("<II", len(body), 0) + body + struct.pack("<I", len(body))


def _address(value: str) -> str:
    address = ipaddress.IPv4Address(value)
    if address.is_unspecified or address.is_multicast or str(address) == "255.255.255.255":
        raise AppProtocolError("App host must be a unicast IPv4 address")
    return str(address)


def discover_host(source_ip: str, timeout: float) -> str:
    """Receive the known motion board's advertisement; send no scan packets."""
    source_ip = _address(source_ip)
    deadline = time.monotonic() + timeout
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as receiver:
        receiver.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            receiver.bind(("0.0.0.0", 9090))
        except OSError as exc:
            if exc.errno == errno.EADDRINUSE:
                raise AppProtocolError(
                    "UDP 9090 is in use. Configure plugins.walking_mode.app_host "
                    "with the robot's App service IPv4 address to bypass discovery.") from exc
            raise
        while time.monotonic() < deadline:
            receiver.settimeout(max(0.001, deadline - time.monotonic()))
            raw, peer = receiver.recvfrom(16384)
            if peer[0] != source_ip:
                continue
            try:
                data = json.loads(raw)
                if isinstance(data, dict) and isinstance(data.get("perception_ip"), str):
                    return _address(data["perception_ip"])
            except (ValueError, UnicodeDecodeError):
                continue
    raise TimeoutError("No App advertisement from the configured motion board")


class WalkingModeClient:
    def __init__(self, host: str, port: int = 8888, timeout: float = 1.0,
                 deadline: float | None = None):
        self.host = _address(host)
        self.port = int(port)
        self.timeout = float(timeout)
        self._deadline = deadline
        self._socket = None

    def __enter__(self):
        self._socket = socket.create_connection((self.host, self.port), self._io_timeout(self.timeout))
        return self

    def __exit__(self, *_args):
        if self._socket is not None:
            self._socket.close()
            self._socket = None

    def _io_timeout(self, timeout: float) -> float:
        if self._deadline is not None:
            timeout = min(timeout, self._deadline - time.monotonic())
        if timeout <= 0:
            raise TimeoutError("Walking-mode request timed out")
        return timeout

    def _send(self, data: dict, *, timeout: float | None = None) -> None:
        if self._socket is None:
            raise AppProtocolError("App connection is not open")
        self._socket.settimeout(self._io_timeout(self.timeout if timeout is None else timeout))
        self._socket.sendall(encode_packet(data))

    def set_walking_model(self, model: str) -> None:
        # Validate before sending anything; these are the two App commands,
        # not DDS action numbers or arbitrary caller-provided JSON.
        self._send({"cmd": MODEL_COMMANDS[model]}, timeout=COMMAND_TIMEOUT_S)

    def _read_exact(self, size: int, deadline: float) -> bytes:
        data = bytearray()
        while len(data) < size:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise TimeoutError("App status query timed out")
            self._socket.settimeout(remaining)
            part = self._socket.recv(size - len(data))
            if not part:
                raise AppProtocolError("App closed the connection before a complete frame")
            data.extend(part)
        return bytes(data)

    def get_status(self, *, deadline: float | None = None) -> int:
        query_deadline = time.monotonic() + self.timeout
        deadline = min(query_deadline, deadline) if deadline is not None else query_deadline
        if self._deadline is not None:
            deadline = min(deadline, self._deadline)
        self._send({"cmd": "get_status"}, timeout=deadline - time.monotonic())
        while time.monotonic() < deadline:
            header = self._read_exact(16, deadline)
            if header[:8] != MAGIC:
                raise AppProtocolError("Invalid App frame magic")
            size, kind = struct.unpack_from("<II", header, 8)
            if size > MAX_BODY_BYTES:
                raise AppProtocolError("App frame exceeds the supported size")
            body = self._read_exact(size, deadline)
            trailer = self._read_exact(4, deadline)
            if struct.unpack("<I", trailer)[0] != size:
                raise AppProtocolError("App frame length and trailer disagree")
            if kind != 0:
                continue
            data = json.loads(body)
            if not isinstance(data, dict) or data.get("cmd") != "status":
                continue
            details = data.get("data")
            status = details.get("status") if isinstance(details, dict) else None
            if type(status) is not int:
                raise AppProtocolError("App status is missing an integer mode code")
            if time.monotonic() >= deadline:
                raise TimeoutError("App status query timed out")
            return status
        raise TimeoutError("No App status response within the query deadline")
