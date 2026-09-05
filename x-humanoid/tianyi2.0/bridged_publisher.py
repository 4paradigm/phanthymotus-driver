#!/usr/bin/env python3
"""
Bridged Publisher - Transparent cross-domain publisher using Unix socket.

Provides a drop-in replacement for rclpy.create_publisher that internally
forwards messages through a bridge process for cross-domain communication.

Usage in plugins (UNCHANGED from normal ROS2):
    from bridged_publisher import create_bridged_publisher

    # Instead of: node.create_publisher(MsgType, topic, qos)
    pub = create_bridged_publisher(node, MsgType, topic, qos)
    pub.publish(msg)  # Same interface!

The bridge process handles:
- Receiving serialized messages via Unix socket
- Publishing to domain 42 with dds-local.xml for agent-core communication
"""

import os
import socket
import struct
import threading
from typing import Type, Any
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.qos import QoSProfile
from rclpy.serialization import serialize_message


class BridgedPublisher:
    """Publisher that forwards messages to bridge process via Unix socket.

    Drop-in replacement for rclpy.Publisher with same interface.
    """

    SOCKET_DIR = "/tmp/tianyi_bridge"
    MAIN_SOCKET = "bridge_main.sock"

    def __init__(self, node: Node, msg_type: Type, topic: str, qos: QoSProfile):
        self.node = node
        self.msg_type = msg_type
        self.topic = topic
        self.qos = qos
        self._socket = None
        self._connected = False
        self._msg_count = 0
        self._send_lock = threading.Lock()  # Protect socket send operations

        # All publishers connect to the same main socket
        self.socket_path = os.path.join(self.SOCKET_DIR, self.MAIN_SOCKET)

        # Try to connect (non-blocking, will retry on publish if fails)
        self._try_connect()

    def _try_connect(self) -> bool:
        """Attempt to connect to bridge process socket."""
        if self._connected:
            return True

        try:
            if not os.path.exists(self.socket_path):
                return False

            self._socket = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
            self._socket.connect(self.socket_path)
            self._connected = True

            # Send topic metadata on first connect
            # Convert msg_type to ROS2 format: sensor_msgs/msg/JointState
            # from sensor_msgs.msg.JointState class
            module_parts = self.msg_type.__module__.split('.')
            if len(module_parts) >= 2:
                # e.g., "sensor_msgs.msg" -> "sensor_msgs/msg"
                package = module_parts[0]
                msg_module = module_parts[1] if len(module_parts) > 1 else "msg"
                msg_type_str = f"{package}/{msg_module}/{self.msg_type.__name__}"
            else:
                # Fallback
                msg_type_str = f"{self.msg_type.__module__}/{self.msg_type.__name__}"

            print(f"[bridged_pub] {self.topic}: msg_type={self.msg_type}, module={self.msg_type.__module__}, formatted={msg_type_str}", flush=True)

            metadata = {
                "topic": self.topic,
                "msg_type": msg_type_str,
            }
            import json
            metadata_bytes = json.dumps(metadata).encode("utf-8")
            self._socket.sendall(struct.pack("<I", len(metadata_bytes)))
            self._socket.sendall(metadata_bytes)

            return True
        except Exception as e:
            if self._socket:
                self._socket.close()
                self._socket = None
            self._connected = False
            return False

    def publish(self, msg: Any) -> None:
        """Publish message (same interface as rclpy.Publisher)."""
        # Try to connect if not connected
        if not self._connected:
            if not self._try_connect():
                # Bridge not available, silently drop (or log once)
                if self._msg_count == 0:
                    print(f"[bridged_pub] WARNING: bridge not available for {self.topic}, "
                          f"messages will be dropped", flush=True)
                self._msg_count += 1
                return

        try:
            # Serialize message
            serialized = serialize_message(msg)

            # Send: [4-byte length][serialized message]
            # Use lock to prevent interleaving when multiple threads publish to same topic
            with self._send_lock:
                self._socket.sendall(struct.pack("<I", len(serialized)))
                self._socket.sendall(serialized)

            self._msg_count += 1

            # Debug: log first few IMU publishes
            if "imu" in self.topic.lower() and self._msg_count <= 5:
                print(f"[bridged_pub] {self.topic}: published msg #{self._msg_count}, size={len(serialized)} bytes", flush=True)

        except (BrokenPipeError, ConnectionResetError, OSError) as e:
            # Connection lost, mark as disconnected for retry
            self._connected = False
            if self._socket:
                self._socket.close()
                self._socket = None
            print(f"[bridged_pub] connection lost for {self.topic}, will retry", flush=True)

    def destroy(self) -> None:
        """Cleanup (same interface as rclpy.Publisher)."""
        if self._socket:
            try:
                self._socket.close()
            except:
                pass
            self._socket = None
        self._connected = False

    # For compatibility with code that checks publisher properties
    @property
    def topic_name(self) -> str:
        return self.topic


def create_bridged_publisher(
    node: Node,
    msg_type: Type,
    topic: str,
    qos: QoSProfile,
) -> BridgedPublisher:
    """Create a bridged publisher (drop-in replacement for node.create_publisher).

    Args:
        node: ROS2 node (for compatibility, not actually used)
        msg_type: Message class
        topic: Topic name
        qos: QoS profile (stored but not used, bridge uses its own)

    Returns:
        BridgedPublisher that looks like rclpy.Publisher
    """
    return BridgedPublisher(node, msg_type, topic, qos)


def should_use_bridge(topic: str) -> bool:
    """Check if a topic should use bridged publisher.

    Use bridge for topics that need to reach agent-core on domain 42.
    Don't use for domain-0-only topics (body controller commands).

    Args:
        topic: Topic name

    Returns:
        True if should use bridge, False for direct publish
    """
    # Bridge all state/sensor topics that go to agent-core
    bridge_patterns = [
        "/state/",
        "/camera/",
        "/asr/",
        "/nav/",
        "/controlled_spatial/",
        "/ext_mic/",
    ]

    return any(pattern in topic for pattern in bridge_patterns)


def create_smart_publisher(
    node: Node,
    msg_type: Type,
    topic: str,
    qos: QoSProfile,
    context = None,
) -> Publisher:
    """Smart publisher: uses bridge for cross-domain topics, direct for others.

    Drop-in replacement for node.create_publisher that automatically chooses
    between bridged (for agent-core) and direct (for body controller).

    Args:
        node: ROS2 node
        msg_type: Message class
        topic: Topic name
        qos: QoS profile
        context: ROS2 context (for direct publisher)

    Returns:
        BridgedPublisher or regular Publisher
    """
    if should_use_bridge(topic):
        return create_bridged_publisher(node, msg_type, topic, qos)
    else:
        # Direct publisher for domain 0 (body controller)
        return node.create_publisher(msg_type, topic, qos)
