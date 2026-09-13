#!/usr/bin/env python3
"""Publish X2 socket frames into Agent Core's isolated ROS domain 42."""

from __future__ import annotations

from common import logsafe

logsafe.install()

import json
import os
import signal
import socket
import struct
import threading
import time
from typing import Any

import rclpy
from rclpy.context import Context
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

SOCKET_PATH = "/tmp/agibot_x2_bridge/bridge_main.sock"
MAX_METADATA_BYTES = 64 * 1024
MAX_MESSAGE_BYTES = 64 * 1024 * 1024


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


def _read_frame(conn: socket.socket, maximum: int, label: str) -> bytes | None:
    raw_len = _read_exact(conn, 4)
    if raw_len is None:
        return None
    size = struct.unpack("<I", raw_len)[0]
    if size > maximum:
        raise ValueError(f"{label} frame is {size} bytes; maximum is {maximum}")
    return _read_exact(conn, size)


def _qos_profile(metadata: Any) -> tuple[dict[str, Any], QoSProfile]:
    if not isinstance(metadata, dict):
        raise ValueError("qos metadata must be an object")
    reliability = metadata.get("reliability")
    durability = metadata.get("durability")
    history = metadata.get("history")
    depth = metadata.get("depth")
    if reliability not in {"reliable", "best_effort"}:
        raise ValueError(f"unsupported reliability: {reliability!r}")
    if durability not in {"volatile", "transient_local"}:
        raise ValueError(f"unsupported durability: {durability!r}")
    if history != "keep_last":
        raise ValueError(f"unsupported history: {history!r}")
    if not isinstance(depth, int) or isinstance(depth, bool) or not 1 <= depth <= 10000:
        raise ValueError(f"invalid qos depth: {depth!r}")
    normalized = {
        "reliability": reliability,
        "durability": durability,
        "history": history,
        "depth": depth,
    }
    profile = QoSProfile(
        reliability={
            "reliable": ReliabilityPolicy.RELIABLE,
            "best_effort": ReliabilityPolicy.BEST_EFFORT,
        }[reliability],
        durability={
            "volatile": DurabilityPolicy.VOLATILE,
            "transient_local": DurabilityPolicy.TRANSIENT_LOCAL,
        }[durability],
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
    )
    return normalized, profile


class TopicHandler:
    def __init__(self, topic: str, type_name: str, qos: dict[str, Any], context: Context, executor: Any):
        self.topic = topic
        self.type_name = type_name
        self.qos, qos_profile = _qos_profile(qos)
        self.msg_class = get_message(type_name)
        self.node = Node(f"x2_bridge_{topic.strip('/').replace('/', '_')}", context=context)
        self.publisher = self.node.create_publisher(self.msg_class, topic, qos_profile)
        executor.add_node(self.node)
        self.count = 0

    def publish(self, payload: bytes) -> None:
        self.publisher.publish(deserialize_message(payload, self.msg_class))
        self.count += 1
        if self.count == 1:
            print(f"[x2-socket-bridge] first frame {self.topic}", flush=True)


class Server:
    def __init__(self):
        self.context = Context()
        rclpy.init(context=self.context, domain_id=42)
        self.executor = rclpy.executors.MultiThreadedExecutor(context=self.context)
        self.handlers = {}
        self.handlers_lock = threading.Lock()
        self.stop_event = threading.Event()
        os.makedirs(os.path.dirname(SOCKET_PATH), exist_ok=True)
        try:
            os.unlink(SOCKET_PATH)
        except FileNotFoundError:
            pass

    def client(self, conn: socket.socket) -> None:
        try:
            metadata_raw = _read_frame(conn, MAX_METADATA_BYTES, "metadata")
            if metadata_raw is None:
                return
            metadata = json.loads(metadata_raw)
            topic, type_name = metadata["topic"], metadata["msg_type"]
            if not isinstance(topic, str) or not topic.startswith("/"):
                raise ValueError("topic must be an absolute ROS topic")
            if not isinstance(type_name, str) or not type_name:
                raise ValueError("msg_type must be a non-empty string")
            qos, _ = _qos_profile(metadata.get("qos"))
            with self.handlers_lock:
                handler = self.handlers.get(topic)
                if handler is None:
                    handler = self.handlers[topic] = TopicHandler(
                        topic, type_name, qos, self.context, self.executor
                    )
                elif handler.type_name != type_name or handler.qos != qos:
                    raise ValueError(
                        f"conflicting registration for {topic}: "
                        f"existing type={handler.type_name} qos={handler.qos}, "
                        f"requested type={type_name} qos={qos}"
                    )
            while not self.stop_event.is_set():
                payload = _read_frame(conn, MAX_MESSAGE_BYTES, "message")
                if payload is None:
                    break
                handler.publish(payload)
        except Exception as exc:
            print(f"[x2-socket-bridge] client error: {str(exc)[:240]}", flush=True)
        finally:
            conn.close()

    def start(self) -> None:
        self.sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        self.sock.bind(SOCKET_PATH)
        self.sock.listen(64)
        threading.Thread(target=self._accept, daemon=True).start()
        threading.Thread(target=self._spin, daemon=True).start()
        print(f"[x2-socket-bridge] ready domain=42 profile={os.environ.get('FASTRTPS_DEFAULT_PROFILES_FILE', 'default')}", flush=True)

    def _accept(self) -> None:
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
        self.stop_event.set()
        try:
            self.sock.close()
        except OSError:
            pass
        self.executor.shutdown()
        if rclpy.ok(context=self.context):
            rclpy.shutdown(context=self.context)
        try:
            os.unlink(SOCKET_PATH)
        except FileNotFoundError:
            pass


def main() -> None:
    server = Server()
    signal.signal(signal.SIGTERM, lambda *_: server.stop())
    signal.signal(signal.SIGINT, lambda *_: server.stop())
    server.start()
    while not server.stop_event.is_set():
        time.sleep(1)


if __name__ == "__main__":
    main()
