"""One receive-only SDK connection shared by every CRA sensor card.

The SDK's DobotRobot.Connect opens Dashboard as well as feedback. Sensors use
DobotConnection directly instead: no Dashboard socket, RequestControl or writes.
TCP framing, freshness and per-joint brake bits are owned by this adapter.
"""

from __future__ import annotations

import copy
import dataclasses
import ipaddress
import math
import os
import threading
import time

import numpy as np
from dobot_sdk.core.connection import DobotConnection
from dobot_sdk.protocol.data_types import MyType, TEST_VALUE_MAGIC
from dobot_sdk.protocol.feedback import FeedbackParser
from dobot_sdk.version import __version__ as SDK_VERSION


PACKET_SIZE = 1440
FEEDBACK_PERIOD_S = {30004: 0.008, 30005: 0.2, 30006: 1.0}
SDK_COMMIT = "55ec1ec82201aaf2e6d54aa47b6272bbaa14c815"
ROBOT_MODES = {
    0: "unknown", 1: "initializing", 2: "brake_open", 3: "power_off",
    4: "disabled", 5: "enabled_idle", 6: "backdrive", 7: "running",
    8: "single_move", 9: "error", 10: "paused", 11: "collision",
}


def positive_number(value, name):
    number = float(value)
    if not math.isfinite(number) or number <= 0:
        raise ValueError(f"{name} must be positive and finite")
    return number


def endpoint(address, port):
    address = str(address).strip()
    if address:
        address = str(ipaddress.IPv4Address(address))
    if isinstance(port, bool) or str(port) not in ("30004", "30005", "30006"):
        raise ValueError("feedback_port must be 30004, 30005 or 30006")
    return address, int(port)


def disconnect(connection):
    # Upstream disconnect() puts shutdown and close in the same try block.
    # Failed connects can make shutdown raise before close, leaking a socket on
    # every reconnect. Capture the socket and always close it as well.
    sock = getattr(connection, "_socket", None)
    try:
        connection.disconnect()
    finally:
        if sock is not None:
            sock.close()


class PacketStream:
    """Reassemble fragmented/coalesced TCP records and resync after bad bytes."""

    def __init__(self):
        self.buffer = bytearray()
        self.discarded_bytes = 0

    def feed(self, data):
        self.buffer.extend(data)
        packets = []
        header = PACKET_SIZE.to_bytes(2, "little")
        magic = TEST_VALUE_MAGIC.to_bytes(8, "little")
        while len(self.buffer) >= 2:
            offset = self.buffer.find(header)
            if offset < 0:
                # Preserve a possible first header byte across recv boundaries.
                drop = len(self.buffer) - 1
                self.discarded_bytes += drop
                del self.buffer[:drop]
                break
            if offset:
                self.discarded_bytes += offset
                del self.buffer[:offset]
            if len(self.buffer) < 56:
                break
            if self.buffer[48:56] != magic:
                del self.buffer[:2]
                self.discarded_bytes += 2
                continue
            if len(self.buffer) < PACKET_SIZE:
                break
            packets.append(bytes(self.buffer[:PACKET_SIZE]))
            del self.buffer[:PACKET_SIZE]
        return packets


def decode_packet(packet):
    if len(packet) != PACKET_SIZE:
        raise ValueError("feedback record must contain 1440 bytes")
    raw = np.frombuffer(packet, dtype=MyType.newbyteorder("<"), count=1)[0]
    if int(raw["len"]) != PACKET_SIZE or int(raw["TestValue"]) != TEST_VALUE_MAGIC:
        raise ValueError("invalid feedback header or magic")
    # Use the upstream parser, retaining its complete data model.
    status = FeedbackParser().parse(packet)
    if status is None:
        raise ValueError("official SDK could not decode feedback")
    state = dataclasses.asdict(status)

    def check_finite(value):
        if isinstance(value, dict):
            for item in value.values():
                check_finite(item)
        elif isinstance(value, list):
            for item in value:
                check_finite(item)
        elif isinstance(value, float) and not math.isfinite(value):
            raise ValueError("non-finite number in feedback")

    check_finite(state)
    # SDK 2.0.0 collapses BrakeStatus to bool and labels mode 11 as JOG.
    # Keep its code unmodified; correct semantics here against V4.6.6 docs.
    state["robot_mode"] = int(raw["RobotMode"])
    state["robot_type"] = int(raw["CRRobotType"]) & 0xFF
    state["robot_mode_name"] = ROBOT_MODES.get(state["robot_mode"], "unknown")
    state["brake_status"] = int(raw["BrakeStatus"]) & 0xFF
    # V4.6.6: bit 5 is J1, bit 0 is J6; expose the array in J1..J6 order.
    state["brake_released"] = [bool(state["brake_status"] & (1 << (5 - i))) for i in range(6)]
    # SafeState is assigned dynamically by the SDK parser and is absent from
    # its dataclass fields. Preserve both bytes, including their high bits.
    state["safety_state"] = int(raw["SafetyState"]) & 0xFF
    state["safe_state"] = int(raw["SafeState"]) & 0xFF
    return state


class FeedbackClient:
    def __init__(self, config, connection_factory=DobotConnection, clock=time.monotonic):
        self.address, self.port = endpoint(
            os.environ.get("DOBOT_CONTROLLER_IP", config.get("controller_ip", "")),
            os.environ.get("DOBOT_FEEDBACK_PORT", config.get("feedback_port", 30004)),
        )
        self.model_name = os.environ.get("DOBOT_MODEL_NAME", config.get("model_name", ""))
        self._configured_stale_timeout = positive_number(config.get("stale_timeout_s", 2), "stale_timeout_s")
        self.connect_timeout = positive_number(config.get("connect_timeout_s", 2), "connect_timeout_s")
        self._configured_receive_timeout = positive_number(config.get("receive_timeout_s", 1), "receive_timeout_s")
        self.reconnect_delay = positive_number(config.get("reconnect_delay_s", 1), "reconnect_delay_s")
        self._factory = connection_factory
        self._clock = clock
        self._lock = threading.Lock()
        self._lifecycle = threading.RLock()
        self._stop = threading.Event()
        self._thread = None
        self._connection = None
        self._latest = None
        self._received_at = None
        self._received_wall_ms = None
        self._transport_connected = False
        self._error = None
        self._cleanup_error = None
        self._packets = 0
        self._invalid_packets = 0
        self._discarded_bytes = 0

    @property
    def receive_timeout(self):
        # Config values are lower bounds. Allow three expected periods so a
        # 1 Hz stream is not disconnected at its next nominal packet deadline.
        return max(self._configured_receive_timeout, 3 * FEEDBACK_PERIOD_S[self.port])

    @property
    def stale_timeout(self):
        return max(self._configured_stale_timeout, 3 * FEEDBACK_PERIOD_S[self.port])

    def _disconnect(self, connection):
        try:
            disconnect(connection)
        except Exception as exc:
            # Cleanup must not kill the receive worker or skip stop/join. Keep
            # the primary transport error and expose cleanup failure separately.
            with self._lock:
                self._cleanup_error = str(exc)

    def configure(self, address, port):
        address, port = endpoint(address, port)
        with self._lifecycle:
            if (address, port) == (self.address, self.port):
                return
            self.stop()
            with self._lock:
                self.address, self.port = address, port
                self._latest = self._received_at = self._received_wall_ms = None
                self._error = None
                self._cleanup_error = None
                self._packets = self._invalid_packets = self._discarded_bytes = 0
            self.start()

    def start(self):
        with self._lifecycle:
            if self._thread and self._thread.is_alive():
                return
            if not self.address:
                return
            self._stop.clear()
            self._thread = threading.Thread(target=self._run, daemon=True, name="dobot-feedback")
            self._thread.start()

    def stop(self):
        with self._lifecycle:
            self._stop.set()
            with self._lock:
                connection = self._connection
            if connection is not None:
                self._disconnect(connection)
            thread = self._thread
            if thread is not None:
                thread.join(self.connect_timeout + self.receive_timeout + 2)
                if thread.is_alive():
                    raise RuntimeError("feedback worker did not stop; refusing a second connection")
            self._thread = None
            with self._lock:
                self._transport_connected = False

    def _run(self):
        while not self._stop.is_set():
            connection = None
            try:
                connection = self._factory(self.address, self.port)
                stream = PacketStream()
                connection.set_timeout(self.connect_timeout, self.receive_timeout)
                with self._lock:
                    self._connection = connection
                    # A new transport must deliver its own valid record before it
                    # becomes healthy; do not revive the previous session's cache.
                    self._received_at = None
                connection.connect()
                with self._lock:
                    self._transport_connected = True
                last_valid = self._clock()
                while not self._stop.is_set():
                    data = connection.receive_bytes(144000)
                    if not data:
                        raise ConnectionError("feedback stream closed")
                    packets = stream.feed(data)
                    for packet in packets:
                        try:
                            state = decode_packet(packet)
                        except ValueError as exc:
                            with self._lock:
                                self._invalid_packets += 1
                                self._error = str(exc)
                            continue
                        last_valid = self._clock()
                        with self._lock:
                            self._latest = state
                            self._received_at = last_valid
                            self._received_wall_ms = int(time.time() * 1000)
                            self._packets += 1
                            self._error = None
                            self._cleanup_error = None
                    with self._lock:
                        self._discarded_bytes += stream.discarded_bytes
                    stream.discarded_bytes = 0
                    if self._clock() - last_valid > self.stale_timeout:
                        raise TimeoutError("no valid feedback within stale_timeout_s")
            except Exception as exc:
                with self._lock:
                    if not self._stop.is_set():
                        self._error = str(exc)
            finally:
                try:
                    if connection is not None:
                        self._disconnect(connection)
                finally:
                    with self._lock:
                        self._connection = None
                        self._transport_connected = False
            self._stop.wait(self.reconnect_delay)

    def snapshot(self):
        with self._lock:
            age = None if self._received_at is None else max(0, self._clock() - self._received_at)
            fresh = self._transport_connected and age is not None and age <= self.stale_timeout
            return {
                "configured": bool(self.address),
                "connected": bool(fresh),
                "transport_connected": self._transport_connected,
                "stale": not fresh,
                "age_s": age,
                "received_at_ms": self._received_wall_ms,
                "controller_ip": self.address,
                "feedback_port": self.port,
                "receive_timeout_s": self.receive_timeout,
                "stale_timeout_s": self.stale_timeout,
                # Geometry identification may survive a feedback outage; joint
                # positions below remain withheld whenever the stream is stale.
                "robot_type_code": None if self._latest is None else self._latest["robot_type"],
                "packets_received": self._packets,
                "invalid_packets": self._invalid_packets,
                "discarded_bytes": self._discarded_bytes,
                "error": self._error,
                "cleanup_error": self._cleanup_error,
                # Never display cached positions as current data after an outage.
                "data": copy.deepcopy(self._latest) if fresh else None,
            }
