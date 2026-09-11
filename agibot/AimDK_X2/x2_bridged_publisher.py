"""Transparent Unix-socket publisher for X2 Agent Core output."""

from __future__ import annotations

import json
import socket
import struct
import threading
from typing import Any, Type

SOCKET_PATH = "/tmp/agibot_x2_bridge/bridge_main.sock"
MAX_MESSAGE_BYTES = 64 * 1024 * 1024


def _type_name(msg_type: Type) -> str:
    parts = msg_type.__module__.split(".")
    if len(parts) >= 2:
        return f"{parts[0]}/{parts[1]}/{msg_type.__name__}"
    return f"{msg_type.__module__}/{msg_type.__name__}"


def _policy_name(value: Any, default: str) -> str:
    name = getattr(value, "name", None)
    if isinstance(name, str):
        return name.lower()
    text = str(value).rsplit(".", 1)[-1].lower()
    return text if text in {"reliable", "best_effort", "volatile", "transient_local", "keep_last"} else default


def _qos_metadata(qos: Any) -> dict[str, Any]:
    """Serialize the rclpy publisher QoS used before the domain bridge."""
    if isinstance(qos, int):
        return {
            "reliability": "reliable",
            "durability": "volatile",
            "history": "keep_last",
            "depth": qos,
        }
    return {
        "reliability": _policy_name(getattr(qos, "reliability", None), "reliable"),
        "durability": _policy_name(getattr(qos, "durability", None), "volatile"),
        "history": _policy_name(getattr(qos, "history", None), "keep_last"),
        "depth": int(getattr(qos, "depth", 10)),
    }


class BridgedPublisher:
    def __init__(self, msg_type: Type, topic: str, qos: Any = 10):
        self.msg_type = msg_type
        self.topic = topic
        self.qos = _qos_metadata(qos)
        self._socket = None
        self._connected = False
        self._connect_lock = threading.Lock()
        self._send_lock = threading.Lock()

    def _connect(self) -> bool:
        if self._connected:
            return True
        with self._connect_lock:
            if self._connected:
                return True
            sock = None
            try:
                sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
                sock.settimeout(1.0)
                sock.connect(SOCKET_PATH)
                metadata = json.dumps({
                    "topic": self.topic,
                    "msg_type": _type_name(self.msg_type),
                    "qos": self.qos,
                }).encode()
                sock.sendall(struct.pack("<I", len(metadata)) + metadata)
                sock.settimeout(0.1)
                self._socket = sock
                self._connected = True
                print(f"[x2-bridge-pub] connected {self.topic}", flush=True)
                return True
            except OSError:
                if sock is not None:
                    sock.close()
                return False

    def publish(self, msg: Any) -> None:
        from rclpy.serialization import serialize_message

        payload = serialize_message(msg)
        if len(payload) > MAX_MESSAGE_BYTES:
            raise ValueError(
                f"bridge frame for {self.topic} is {len(payload)} bytes; "
                f"maximum is {MAX_MESSAGE_BYTES}"
            )
        if not self._connect():
            return
        try:
            with self._send_lock:
                self._socket.sendall(struct.pack("<I", len(payload)) + payload)
        except (BrokenPipeError, ConnectionResetError, OSError):
            self.destroy()

    def destroy(self) -> None:
        sock, self._socket = self._socket, None
        self._connected = False
        if sock is not None:
            try:
                sock.close()
            except OSError:
                pass


def create_bridged_publisher(msg_type: Type, topic: str, qos: Any = 10) -> BridgedPublisher:
    return BridgedPublisher(msg_type, topic, qos)
