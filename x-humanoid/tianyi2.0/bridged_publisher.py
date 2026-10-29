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

import json
import os
import socket
import struct
import threading
from typing import Type, Any
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.serialization import deserialize_message, serialize_message


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
        self._connect_lock = threading.Lock()  # Protect connection establishment

        # All publishers connect to the same main socket
        self.socket_path = os.path.join(self.SOCKET_DIR, self.MAIN_SOCKET)

        # Try to connect (non-blocking, will retry on publish if fails)
        self._try_connect()

    def _try_connect(self) -> bool:
        """Attempt to connect to bridge process socket.

        Thread-safe: only one thread can attempt connection at a time.
        """
        # Quick check without lock (optimization for already-connected case)
        if self._connected:
            return True

        # Serialize connection attempts to prevent duplicate connections
        with self._connect_lock:
            # Double-check after acquiring lock (another thread may have connected)
            if self._connected:
                return True

            try:
                if not os.path.exists(self.socket_path):
                    return False

                self._socket = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
                self._socket.connect(self.socket_path)

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

                # Only mark connected after metadata is successfully sent
                self._connected = True
                print(f"[bridged_pub] {self.topic}: connected and metadata sent", flush=True)

                return True
            except Exception as e:
                print(f"[bridged_pub] {self.topic}: connection failed: {e}", flush=True)
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


class BridgedSubscription:
    """Subscription that receives from the bridge process via Unix socket.

    The mirror of BridgedPublisher, and the half that did not exist. A card on
    the main process cannot subscribe on domain 42: that participant runs under
    the vendor DDS profile so the body link on 192.168.41.x survives, and it is
    invisible to agent-core. The bridge process, which does hold the loopback
    profile, subscribes on its behalf and ships each message back down the same
    socket the publishers use.

    Two consequences worth knowing at the call site:

      **The callback runs on this reader thread, not on the rclpy executor.**
      Anything it touches must be safe to touch from another thread. Nothing is
      serialised against the node's timers.

      **A dropped connection is silent by design.** The bridge may be starting,
      restarting, or gone; the reader retries rather than raising, because the
      alternative is a card that dies because a helper process blinked.
    """

    SOCKET_DIR = "/tmp/tianyi_bridge"
    MAIN_SOCKET = "bridge_main.sock"
    RETRY_S = 1.0

    def __init__(self, node: Node, msg_type: Type, topic: str, callback,
                 qos: QoSProfile):
        self.node = node
        self.msg_type = msg_type
        self.topic = topic
        self.callback = callback
        self.qos = qos
        self._socket = None
        self._stop = threading.Event()
        self._msg_count = 0
        self.socket_path = os.path.join(self.SOCKET_DIR, self.MAIN_SOCKET)

        self._thread = threading.Thread(
            target=self._run, name=f"bridged-sub{topic.replace('/', '-')}",
            daemon=True)
        self._thread.start()

    # ── the reader ───────────────────────────────────────────────────────────

    def _run(self):
        while not self._stop.is_set():
            if not self._connect():
                self._stop.wait(self.RETRY_S)
                continue
            try:
                self._pump()
            except Exception as e:      # noqa: BLE001 — reconnect, do not die
                if not self._stop.is_set():
                    print(f"[bridged_sub] {self.topic}: link lost ({e}), retrying",
                          flush=True)
            finally:
                self._close_socket()
            self._stop.wait(self.RETRY_S)

    def _connect(self) -> bool:
        try:
            if not os.path.exists(self.socket_path):
                return False
            sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
            sock.connect(self.socket_path)

            module_parts = self.msg_type.__module__.split(".")
            msg_type_str = (f"{module_parts[0]}/{module_parts[1]}/{self.msg_type.__name__}"
                            if len(module_parts) >= 2
                            else f"{self.msg_type.__module__}/{self.msg_type.__name__}")

            metadata = {
                "topic": self.topic,
                "msg_type": msg_type_str,
                "direction": "in",
                # Passed rather than assumed: the bridge has to match whatever is
                # publishing on domain 42, and a QoS mismatch moves no data while
                # both ends look healthy.
                "qos": {
                    "reliability": ("reliable"
                                    if self.qos.reliability == ReliabilityPolicy.RELIABLE
                                    else "best_effort"),
                    "depth": int(getattr(self.qos, "depth", 1) or 1),
                },
            }
            payload = json.dumps(metadata).encode("utf-8")
            sock.sendall(struct.pack("<I", len(payload)))
            sock.sendall(payload)

            self._socket = sock
            print(f"[bridged_sub] {self.topic}: subscribed via bridge", flush=True)
            return True
        except Exception as e:      # noqa: BLE001
            print(f"[bridged_sub] {self.topic}: connect failed: {e}", flush=True)
            self._close_socket()
            return False

    def _recv_exactly(self, count: int):
        buffer = b""
        while len(buffer) < count:
            chunk = self._socket.recv(min(count - len(buffer), 65536))
            if not chunk:
                return None
            buffer += chunk
        return buffer

    def _pump(self):
        while not self._stop.is_set():
            header = self._recv_exactly(4)
            if header is None:
                return                                  # bridge closed
            payload = self._recv_exactly(struct.unpack("<I", header)[0])
            if payload is None:
                return
            message = deserialize_message(payload, self.msg_type)
            self._msg_count += 1
            try:
                self.callback(message)
            except Exception as e:      # noqa: BLE001
                # A caller's bug must not kill the link, or one bad frame ends
                # the subscription for good.
                print(f"[bridged_sub] {self.topic}: callback raised: {e}",
                      flush=True)

    def _close_socket(self):
        if self._socket is not None:
            try:
                self._socket.close()
            except Exception:       # noqa: BLE001
                pass
            self._socket = None

    # ── rclpy.Subscription-shaped surface ────────────────────────────────────

    def destroy(self) -> None:
        self._stop.set()
        self._close_socket()

    @property
    def topic_name(self) -> str:
        return self.topic


def create_bridged_subscription(
    node: Node,
    msg_type: Type,
    topic: str,
    callback,
    qos: QoSProfile,
) -> BridgedSubscription:
    """Create a bridged subscription (drop-in for node.create_subscription)."""
    subscription = BridgedSubscription(node, msg_type, topic, callback, qos)
    # Tracked on the node so destroy_node can close it: rclpy knows nothing
    # about this object, so without the registry a start/stop cycle leaves the
    # reader thread and its socket alive — the orphaned-subscription failure
    # this repo has already paid for once elsewhere.
    node.__dict__.setdefault("_bridged_subscriptions", []).append(subscription)
    return subscription


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
    # Bridge all state/sensor topics that go to agent-core.
    #
    # `/servo/` is here because the servo card's state output is
    # `/nvidia_desktop/servo/state`, which does NOT contain "/state/" — the
    # segment is last, so there is no trailing slash. It therefore published
    # straight onto the invisible domain-42 participant and agent-core never saw
    # it, which broke the vla -> servo -> (state) -> vla feedback loop in the
    # direction nobody was looking at. Substring matching on a path is what made
    # that possible; the list stays, but this is the trap it sets.
    bridge_patterns = [
        "/state/",
        "/servo/",
        "/camera/",
        "/asr/",
        "/nav/",
        "/controlled_spatial/",
        "/ext_mic/",
    ]

    return any(pattern in topic for pattern in bridge_patterns)


def should_bridge_subscription(topic: str) -> bool:
    """Every domain-42 subscription must be bridged. There is no other path.

    Deliberately not a whitelist, unlike the publisher side. A publisher on the
    main process's domain-42 participant at least *works* for anything already
    on that participant, so choosing per topic is meaningful. A subscription
    there can never receive anything at all — the participant is invisible to
    agent-core — so a topic left off a list would not fall back to a slower
    path, it would fall back to silence.

    The `topic` argument is kept so the decision has somewhere to live if a
    genuine exception ever appears.
    """
    del topic
    return True


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
