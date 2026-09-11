"""Unix-socket publisher for X2 Agent Core output."""

from __future__ import annotations

import json
import os
import socket
import struct
import threading
from typing import Any, Type


DEFAULT_SOCKET_PATH = "/tmp/agibot_x2_bridge/bridge_main.sock"


def _type_name(msg_type: Type) -> str:
    parts = msg_type.__module__.split(".")
    if len(parts) >= 2:
        return f"{parts[0]}/{parts[1]}/{msg_type.__name__}"
    return f"{msg_type.__module__}/{msg_type.__name__}"


class BridgedPublisher:
    """Serialize ROS messages into a bounded, reconnecting Unix stream."""

    def __init__(self, msg_type: Type, topic: str, socket_path: str | None = None):
        self.msg_type = msg_type
        self.topic = topic
        self.socket_path = socket_path or os.environ.get("X2_BRIDGE_SOCKET", DEFAULT_SOCKET_PATH)
        self._socket: socket.socket | None = None
        self._connected = False
        self._connect_lock = threading.Lock()
        self._send_lock = threading.Lock()
        self._count = 0
        self._dropped = 0
        self._last_error = ""

    def _connect(self) -> bool:
        if self._connected:
            return True
        with self._connect_lock:
            if self._connected:
                return True
            sock = None
            try:
                sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
                sock.settimeout(0.25)
                sock.connect(self.socket_path)
                metadata = json.dumps({
                    "topic": self.topic,
                    "msg_type": _type_name(self.msg_type),
                }).encode("utf-8")
                sock.sendall(struct.pack("<I", len(metadata)) + metadata)
                # Sensor frames are lossy. Never let a congested Core side
                # block the robot-domain executor for an unbounded duration.
                sock.settimeout(0.1)
                self._socket = sock
                self._connected = True
                self._last_error = ""
                print(
                    f"[x2-bridge-pub] connected {self.topic} ({_type_name(self.msg_type)})",
                    flush=True,
                )
                return True
            except OSError as exc:
                self._last_error = str(exc)[:240]
                if sock is not None:
                    try:
                        sock.close()
                    except OSError:
                        pass
                return False

    def publish(self, msg: Any) -> bool:
        from rclpy.serialization import serialize_message

        payload = serialize_message(msg)
        if not self._connect():
            self._dropped += 1
            return False
        try:
            frame = struct.pack("<I", len(payload)) + payload
            with self._send_lock:
                assert self._socket is not None
                self._socket.sendall(frame)
            self._count += 1
            return True
        except (BrokenPipeError, ConnectionResetError, OSError) as exc:
            self._last_error = str(exc)[:240]
            self._dropped += 1
            self.destroy(clear_error=False)
            return False

    def snapshot(self) -> dict[str, Any]:
        return {
            "bridge_connected": self._connected,
            "bridge_sent_messages": self._count,
            "bridge_dropped_messages": self._dropped,
            "bridge_last_error": self._last_error,
        }

    def destroy(self, *, clear_error: bool = True) -> None:
        sock, self._socket = self._socket, None
        self._connected = False
        if clear_error:
            self._last_error = ""
        if sock is not None:
            try:
                sock.close()
            except OSError:
                pass


def create_bridged_publisher(msg_type: Type, topic: str) -> BridgedPublisher:
    return BridgedPublisher(msg_type, topic)
