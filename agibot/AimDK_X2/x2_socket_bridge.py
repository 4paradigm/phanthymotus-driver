#!/usr/bin/env python3
"""Publish X2 Unix-socket frames into Agent Core's isolated ROS domain."""

from __future__ import annotations

import json
import os
import signal
import socket
import struct
import threading
import time
from typing import Any

from common import logsafe

logsafe.install(check_fd=False)

import rclpy
from rclpy.context import Context
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

from x2_bridged_publisher import DEFAULT_SOCKET_PATH


MAX_METADATA_BYTES = 16 * 1024
MAX_MESSAGE_BYTES = 64 * 1024 * 1024
QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=200,
    durability=DurabilityPolicy.VOLATILE,
)


def _read_exact(conn: socket.socket, size: int) -> bytes | None:
    chunks = []
    remaining = size
    while remaining:
        chunk = conn.recv(min(remaining, 65536))
        if not chunk:
            return None
        chunks.append(chunk)
        remaining -= len(chunk)
    return b"".join(chunks)


def _read_frame(conn: socket.socket, limit: int) -> bytes | None:
    raw_length = _read_exact(conn, 4)
    if raw_length is None:
        return None
    length = struct.unpack("<I", raw_length)[0]
    if length > limit:
        raise ValueError(f"socket frame length {length} exceeds limit {limit}")
    return _read_exact(conn, length)


class TopicHandler:
    def __init__(self, topic: str, type_name: str, ctx: Context, executor: Any):
        self.topic = topic
        self.type_name = type_name
        self.msg_class = get_message(type_name)
        self.node = Node(f"x2_bridge_{topic.strip('/').replace('/', '_')}", context=ctx)
        self.publisher = self.node.create_publisher(self.msg_class, topic, QOS)
        executor.add_node(self.node)
        self.count = 0
        print(f"[x2-socket-bridge] handler {topic} {type_name}", flush=True)

    def publish(self, payload: bytes) -> None:
        self.publisher.publish(deserialize_message(payload, self.msg_class))
        self.count += 1
        if self.count == 1:
            print(f"[x2-socket-bridge] first frame {self.topic}", flush=True)

    def destroy(self) -> None:
        self.node.destroy_node()


class Server:
    def __init__(self):
        self.domain_id = int(os.environ.get("CORE_DOMAIN_ID", os.environ.get("ROS_DOMAIN_ID", "42")))
        self.socket_path = os.environ.get("X2_BRIDGE_SOCKET", DEFAULT_SOCKET_PATH)
        self.context = Context()
        rclpy.init(context=self.context, domain_id=self.domain_id)
        self.executor = rclpy.executors.MultiThreadedExecutor(context=self.context)
        self.handlers: dict[str, TopicHandler] = {}
        self.handlers_lock = threading.Lock()
        self.stop_event = threading.Event()
        self.sock: socket.socket | None = None

    def _handler(self, topic: str, type_name: str) -> TopicHandler:
        with self.handlers_lock:
            handler = self.handlers.get(topic)
            if handler is None:
                handler = TopicHandler(topic, type_name, self.context, self.executor)
                self.handlers[topic] = handler
            elif handler.type_name != type_name:
                raise ValueError(f"topic {topic} reconnected with a different ROS type")
            return handler

    def client(self, conn: socket.socket) -> None:
        try:
            metadata_raw = _read_frame(conn, MAX_METADATA_BYTES)
            if metadata_raw is None:
                return
            metadata = json.loads(metadata_raw)
            topic = metadata["topic"]
            type_name = metadata["msg_type"]
            if not isinstance(topic, str) or not topic.startswith("/"):
                raise ValueError("bridge topic must be an absolute ROS topic")
            if not isinstance(type_name, str):
                raise ValueError("bridge message type must be a string")
            handler = self._handler(topic, type_name)
            while not self.stop_event.is_set():
                payload = _read_frame(conn, MAX_MESSAGE_BYTES)
                if payload is None:
                    break
                handler.publish(payload)
        except Exception as exc:
            print(f"[x2-socket-bridge] client error: {str(exc)[:240]}", flush=True)
        finally:
            conn.close()

    def start(self) -> None:
        os.makedirs(os.path.dirname(self.socket_path), exist_ok=True)
        try:
            os.unlink(self.socket_path)
        except FileNotFoundError:
            pass
        self.sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        self.sock.bind(self.socket_path)
        self.sock.listen(64)
        threading.Thread(target=self._accept, daemon=True, name="x2-bridge-accept").start()
        threading.Thread(target=self._spin, daemon=True, name="x2-bridge-spin").start()
        print(
            f"[x2-socket-bridge] ready domain={self.domain_id} socket={self.socket_path} "
            f"profile={os.environ.get('FASTRTPS_DEFAULT_PROFILES_FILE', 'default')}",
            flush=True,
        )

    def _accept(self) -> None:
        assert self.sock is not None
        while not self.stop_event.is_set():
            try:
                self.sock.settimeout(1.0)
                conn, _ = self.sock.accept()
                threading.Thread(target=self.client, args=(conn,), daemon=True).start()
            except socket.timeout:
                continue
            except OSError:
                break

    def _spin(self) -> None:
        while not self.stop_event.is_set() and rclpy.ok(context=self.context):
            self.executor.spin_once(timeout_sec=0.1)

    def stop(self) -> None:
        if self.stop_event.is_set():
            return
        self.stop_event.set()
        if self.sock is not None:
            try:
                self.sock.close()
            except OSError:
                pass
        self.executor.shutdown()
        with self.handlers_lock:
            for handler in self.handlers.values():
                handler.destroy()
            self.handlers.clear()
        if rclpy.ok(context=self.context):
            rclpy.shutdown(context=self.context)
        try:
            os.unlink(self.socket_path)
        except FileNotFoundError:
            pass


def main() -> None:
    server = Server()
    signal.signal(signal.SIGTERM, lambda *_: server.stop())
    signal.signal(signal.SIGINT, lambda *_: server.stop())
    server.start()
    try:
        while not server.stop_event.is_set():
            time.sleep(1)
    finally:
        server.stop()


if __name__ == "__main__":
    main()
