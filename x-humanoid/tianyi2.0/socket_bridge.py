#!/usr/bin/env python3
"""
Socket Bridge Server - Receives messages via Unix socket and publishes to domain 42.

This process runs independently, allowing it to communicate with agent-core
while the main process uses domain 0 for body controller.

Architecture:
    Main process (domain 0, dds_profile.xml)
        → Plugins use BridgedPublisher
        → Send via Unix socket
        → This bridge process
        → Publish to domain 42 (inherits DDS config from environment)
        → Agent Core receives

Usage:
    python3 socket_bridge.py [--config config.yaml]
"""

import os
import sys
import json
import signal
import socket
import struct
import threading
import time
from pathlib import Path
from typing import Dict
import yaml

import rclpy
from rclpy.context import Context
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


# Default QoS profiles
RELIABLE_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    durability=DurabilityPolicy.VOLATILE,
)

BEST_EFFORT_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=200,
    durability=DurabilityPolicy.VOLATILE,
)


class TopicHandler:
    """Handles a single topic: receives from socket, publishes to domain 42."""

    def __init__(self, topic: str, msg_type_name: str, ctx: Context, executor):
        self.topic = topic
        self.msg_type_name = msg_type_name
        self.msg_class = get_message(msg_type_name)
        self.msg_count = 0
        self.context_invalid = False

        # Create publisher on domain 42
        self.node = Node(
            f"bridge_{topic.strip('/').replace('/', '_')}",
            context=ctx,
        )

        # Use BEST_EFFORT for all topics to match Agent Core's expectations
        # Agent Core's phanthy_bus_bridge subscribes with BEST_EFFORT
        qos = BEST_EFFORT_QOS

        self.pub = self.node.create_publisher(self.msg_class, topic, qos)
        executor.add_node(self.node)

        print(f"[socket-bridge] topic handler created: {topic} ({msg_type_name})", flush=True)

    def publish(self, serialized_msg: bytes):
        """Deserialize and publish message."""
        try:
            msg = deserialize_message(serialized_msg, self.msg_class)
            self.pub.publish(msg)
            self.msg_count += 1

            # Debug first few messages for camera and imu
            if ("camera" in self.topic or "imu" in self.topic) and self.msg_count <= 5:
                print(f"[socket-bridge] {self.topic}: msg #{self.msg_count}, serialized={len(serialized_msg)} bytes, type={type(msg)}", flush=True)

            if self.msg_count % 100 == 0:
                print(
                    f"[socket-bridge] {self.topic}: published {self.msg_count} messages",
                    flush=True,
                )
        except Exception as e:
            error_msg = str(e)
            print(f"[socket-bridge] ERROR publishing to {self.topic}: {e}", flush=True)

            # If context is invalid, mark this handler as broken so it can be recreated
            if "context is invalid" in error_msg or "publisher's context is invalid" in error_msg:
                print(f"[socket-bridge] Context invalid for {self.topic}, handler needs recreation", flush=True)
                self.context_invalid = True
                raise  # Re-raise to let caller know this handler is broken

            print(f"[socket-bridge]   serialized length: {len(serialized_msg)} bytes", flush=True)
            print(f"[socket-bridge]   first 100 bytes: {serialized_msg[:100]}", flush=True)
            import traceback
            traceback.print_exc()


class SocketBridgeServer:
    """Manages Unix socket server and topic handlers."""

    SOCKET_DIR = "/tmp/tianyi_bridge"

    def __init__(self):
        # Setup domain 42 with default DDS config (same as agent-core)
        # Agent-core and socket bridge must use the same DDS configuration
        # to communicate. They inherit from main process environment.
        self.ctx = Context()
        rclpy.init(context=self.ctx, domain_id=42)
        self.executor = rclpy.executors.MultiThreadedExecutor(context=self.ctx)

        dds_config = os.environ.get("FASTRTPS_DEFAULT_PROFILES_FILE", "default")
        print(
            f"[socket-bridge] domain 42 initialized (DDS config: {dds_config})",
            flush=True,
        )

        self.handlers: Dict[str, TopicHandler] = {}
        self._stop_flag = threading.Event()

        # Create socket directory
        os.makedirs(self.SOCKET_DIR, exist_ok=True)

    def handle_client(self, conn: socket.socket, addr):
        """Handle a client connection (one per topic)."""
        try:
            # First message: topic metadata (JSON)
            length_bytes = conn.recv(4)
            if len(length_bytes) < 4:
                return

            metadata_len = struct.unpack("<I", length_bytes)[0]

            # Read metadata in chunks (like message data)
            metadata_bytes = b""
            while len(metadata_bytes) < metadata_len:
                chunk = conn.recv(min(metadata_len - len(metadata_bytes), 65536))
                if not chunk:
                    return
                metadata_bytes += chunk

            metadata = json.loads(metadata_bytes.decode("utf-8"))

            topic = metadata["topic"]
            msg_type = metadata["msg_type"]

            print(f"[socket-bridge] new client: {topic} ({msg_type})", flush=True)

            # Create handler if not exists or if previous handler has invalid context
            if topic not in self.handlers or getattr(self.handlers.get(topic), 'context_invalid', False):
                if topic in self.handlers:
                    print(f"[socket-bridge] recreating handler for {topic} (previous context was invalid)", flush=True)
                    # Clean up old handler
                    old_handler = self.handlers[topic]
                    try:
                        self.executor.remove_node(old_handler.node)
                        old_handler.node.destroy_node()
                    except Exception as e:
                        print(f"[socket-bridge] cleanup error for {topic}: {e}", flush=True)

                self.handlers[topic] = TopicHandler(topic, msg_type, self.ctx, self.executor)

            handler = self.handlers[topic]

            # Receive and publish messages
            while not self._stop_flag.is_set():
                # Read message length
                length_bytes = conn.recv(4)
                if len(length_bytes) < 4:
                    break

                msg_len = struct.unpack("<I", length_bytes)[0]

                # Read message data
                msg_data = b""
                while len(msg_data) < msg_len:
                    chunk = conn.recv(min(msg_len - len(msg_data), 65536))
                    if not chunk:
                        break
                    msg_data += chunk

                if len(msg_data) < msg_len:
                    break

                # Publish
                handler.publish(msg_data)

        except Exception as e:
            print(f"[socket-bridge] client handler error: {e}", flush=True)
        finally:
            conn.close()

    def start(self):
        """Start bridge server with single dynamic socket."""
        # Create single main socket that accepts all topics dynamically
        main_socket_path = os.path.join(self.SOCKET_DIR, "bridge_main.sock")

        if os.path.exists(main_socket_path):
            os.remove(main_socket_path)

        self.main_sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        self.main_sock.bind(main_socket_path)
        self.main_sock.listen(100)  # High backlog for multiple publishers

        print(f"[socket-bridge] listening on {main_socket_path} (dynamic topics)", flush=True)

        # Accept all connections
        def accept_all():
            while not self._stop_flag.is_set():
                try:
                    self.main_sock.settimeout(1.0)
                    conn, addr = self.main_sock.accept()
                    threading.Thread(
                        target=self.handle_client,
                        args=(conn, addr),
                        daemon=True
                    ).start()
                except socket.timeout:
                    continue
                except Exception as e:
                    if not self._stop_flag.is_set():
                        print(f"[socket-bridge] accept error: {e}", flush=True)
                    break

        threading.Thread(target=accept_all, daemon=True).start()

        # Start executor
        def spin():
            while not self._stop_flag.is_set() and rclpy.ok(context=self.ctx):
                self.executor.spin_once(timeout_sec=0.1)

        threading.Thread(target=spin, daemon=True).start()

        print("[socket-bridge] server started", flush=True)

    def stop(self):
        """Stop bridge server."""
        print("[socket-bridge] stopping...", flush=True)
        self._stop_flag.set()
        self.executor.shutdown()
        rclpy.shutdown(context=self.ctx)

        # Cleanup sockets
        import glob
        for sock_file in glob.glob(f"{self.SOCKET_DIR}/*.sock"):
            try:
                os.remove(sock_file)
            except:
                pass


def main():
    print("[socket-bridge] starting socket bridge server...", flush=True)

    bridge = SocketBridgeServer()

    def stop_handler(*_):
        bridge.stop()
        sys.exit(0)

    signal.signal(signal.SIGTERM, stop_handler)
    signal.signal(signal.SIGINT, stop_handler)

    bridge.start()

    # Keep alive
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        stop_handler()


if __name__ == "__main__":
    main()
