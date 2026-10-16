"""Transparent Unix-socket publisher for X2 domain-42 output."""

from __future__ import annotations

import json
import os
import socket
import struct
import threading
from typing import Any, Type

SOCKET_PATH = "/tmp/agibot_x2_bridge/bridge_main.sock"


def _type_name(msg_type: Type) -> str:
    parts = msg_type.__module__.split(".")
    if len(parts) >= 2:
        return f"{parts[0]}/{parts[1]}/{msg_type.__name__}"
    return f"{msg_type.__module__}/{msg_type.__name__}"


class BridgedPublisher:
    def __init__(self, msg_type: Type, topic: str):
        self.msg_type = msg_type
        self.topic = topic
        self._socket = None
        self._connected = False
        self._connect_lock = threading.Lock()
        self._send_lock = threading.Lock()
        self._count = 0

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
                }).encode()
                sock.sendall(struct.pack("<I", len(metadata)) + metadata)
                # A stalled domain-42 consumer must not block the robot-domain
                # executor indefinitely. Sensor frames are lossy by design, so
                # drop a congested frame and reconnect on the next callback.
                sock.settimeout(0.1)
                self._socket = sock
                self._connected = True
                print(f"[x2-bridge-pub] connected {self.topic} ({_type_name(self.msg_type)})", flush=True)
                return True
            except OSError:
                if sock is not None:
                    try:
                        sock.close()
                    except OSError:
                        pass
                return False

    def publish(self, msg: Any) -> None:
        from rclpy.serialization import serialize_message

        payload = serialize_message(msg)
        if not self._connect():
            return
        try:
            with self._send_lock:
                self._socket.sendall(struct.pack("<I", len(payload)) + payload)
            self._count += 1
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


def create_bridged_publisher(msg_type: Type, topic: str) -> BridgedPublisher:
    return BridgedPublisher(msg_type, topic)
