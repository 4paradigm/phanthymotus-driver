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
from rclpy.serialization import serialize_message
from rosidl_runtime_py.utilities import get_message


# How often a bridged topic reports how many messages it has carried. Time-based
# rather than count-based so a 16 kHz audio topic costs the same number of log
# lines as a 10 Hz one; see the comment in TopicHandler.publish.
PROGRESS_INTERVAL_S = 300.0
MAX_METADATA_BYTES = 64 * 1024
MAX_MESSAGE_BYTES = 64 * 1024 * 1024
# This is only the bridge-side budget.  camera_depth has already spent part of
# Navi's 500 ms lease transforming, compressing, serializing and writing the
# Unix frame, so a depth sample must not wait here for another full lease.
DEPTH_MAX_PENDING_AGE_S = 0.05
DEPTH_WORKER_JOIN_TIMEOUT_S = 1.0
DEPTH_STALE_LOG_INTERVAL_S = 30.0
SLOW_LOG_INTERVAL_S = 30.0

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


def _is_hardware_depth(topic: str) -> bool:
    return topic.endswith("/camera/head/depth/visual_depth")


def _outbound_qos(topic: str, requested=None) -> QoSProfile:
    """Keep only the newest hardware-depth frame on the domain-42 side.

    Existing bridged topics retain the historical depth=200 behaviour.  The
    hardware depth stream is different: every frame replaces the previous one,
    and replaying an old FIFO after a short DDS stall is actively harmful to
    navigation.  New clients carry their requested queue depth in metadata;
    old clients for this one topic still get the safe depth=1 default.
    """
    if _is_hardware_depth(topic):
        requested = requested or {}
        depth = max(1, min(int(requested.get("depth", 1) or 1), 1))
        return QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=depth,
            durability=DurabilityPolicy.VOLATILE,
        )
    return BEST_EFFORT_QOS


def _recv_exact(conn: socket.socket, size: int):
    """Read one framed field without assuming a Unix-socket recv is complete."""
    if size < 0:
        raise ValueError("negative frame size")
    data = bytearray(size)
    view = memoryview(data)
    offset = 0
    while offset < size:
        received = conn.recv_into(view[offset:], min(size - offset, 65536))
        if received == 0:
            return None
        offset += received
    return bytes(data)


def _configure_fastdds_publication() -> None:
    """Keep large depth writes off the bridge's Python/client threads.

    This process also forwards inbound navigation commands.  FastDDS defaults
    to synchronous publication, so a large image write can otherwise hold the
    process GIL and delay those commands.  The setting must be applied before
    rclpy.init creates the participant.
    """
    os.environ["RMW_FASTRTPS_PUBLICATION_MODE"] = "ASYNCHRONOUS"
    # When this is 1, rmw_fastrtps ignores the publication-mode environment
    # variable and takes publishMode from XML.  The loopback transport profile
    # remains loaded through FASTRTPS_DEFAULT_PROFILES_FILE either way.
    os.environ.pop("RMW_FASTRTPS_USE_QOS_FROM_XML", None)


class TopicHandler:
    """Handles a single topic: receives from socket, publishes to domain 42."""

    def __init__(self, topic: str, msg_type_name: str, ctx: Context, executor,
                 qos_spec=None):
        self.topic = topic
        self.msg_type_name = msg_type_name
        self.msg_class = get_message(msg_type_name)
        self.msg_count = 0
        self.context_invalid = False
        self.is_hardware_depth = _is_hardware_depth(topic)
        self.last_receive_gap_ms = None
        self.max_receive_gap_ms = 0.0
        self.last_publish_ms = 0.0
        self.max_publish_ms = 0.0
        self._last_receive_ts = None
        self.received_count = 0
        self.coalesced_count = 0
        self.stale_pending_count = 0
        self._state = threading.Condition()
        self._pending_depth = None
        self._closing = False
        self._worker_error = None
        self._worker = None
        self._active_inline_publishes = 0
        self._close_timeout_reported = False
        self._last_stale_log_ts = 0.0
        self._last_slow_log_ts = 0.0
        self._slow_event_count = 0
        # Starts at 0 so the very first message always prints; after that the
        # interval governs.
        self._last_progress_ts = 0.0

        # Create publisher on domain 42
        self.node = Node(
            f"bridge_{topic.strip('/').replace('/', '_')}",
            context=ctx,
        )

        # Keep BEST_EFFORT to match Agent Core.  Hardware depth additionally
        # preserves its newest-only queue depth instead of inheriting the
        # generic bridge's depth=200 FIFO.
        qos = _outbound_qos(topic, qos_spec)

        self.pub = self.node.create_publisher(self.msg_class, topic, qos)
        executor.add_node(self.node)

        if self.is_hardware_depth:
            self._worker = threading.Thread(
                target=self._depth_publish_loop,
                name="tianyi2-depth-dds",
                daemon=True,
            )
            self._worker.start()

        print(f"[socket-bridge] topic handler created: {topic} ({msg_type_name})", flush=True)

    def publish(self, serialized_msg: bytes):
        """Accept one original-CDR frame for publication.

        Non-depth topics retain the historical inline behaviour.  Hardware
        depth is a latest-value stream: the socket reader only replaces one
        pending slot, while a dedicated worker performs the DDS write.  Thus a
        temporary DDS slowdown cannot create a Python FIFO of stale images.
        """
        received_at = time.monotonic()
        with self._state:
            if self._closing:
                reason = self._worker_error or "topic handler is closed"
                raise RuntimeError(str(reason))
            if self.context_invalid:
                raise RuntimeError("publisher's context is invalid")
            if self.is_hardware_depth and (
                    self._worker is None or not self._worker.is_alive()):
                reason = self._worker_error or "depth DDS worker is not running"
                raise RuntimeError(str(reason))

            if self._last_receive_ts is not None:
                gap_ms = (received_at - self._last_receive_ts) * 1000
                self.last_receive_gap_ms = gap_ms
                self.max_receive_gap_ms = max(self.max_receive_gap_ms, gap_ms)
            self._last_receive_ts = received_at
            self.received_count += 1

            if self.is_hardware_depth:
                if self._pending_depth is not None:
                    self.coalesced_count += 1
                self._pending_depth = (serialized_msg, received_at)
                self._state.notify()
                return True
            self._active_inline_publishes += 1

        # Preserve the legacy non-depth call contract (inline side effect, no
        # return value).  Only depth uses the asynchronous acceptance result.
        try:
            self._publish_now(serialized_msg)
        finally:
            with self._state:
                self._active_inline_publishes -= 1
                self._state.notify_all()
        return None

    def _depth_publish_loop(self):
        """Publish only the newest pending hardware-depth frame."""
        while True:
            with self._state:
                while not self._closing and self._pending_depth is None:
                    self._state.wait()
                # Closing wins over pending: shutdown never drains stale data.
                if self._closing:
                    self._pending_depth = None
                    return
                serialized_msg, received_at = self._pending_depth
                self._pending_depth = None

            age_s = time.monotonic() - received_at
            if age_s >= DEPTH_MAX_PENDING_AGE_S:
                with self._state:
                    self.stale_pending_count += 1
                    now = time.monotonic()
                    report_stale = (
                        self.stale_pending_count == 1 or
                        now - self._last_stale_log_ts >=
                        DEPTH_STALE_LOG_INTERVAL_S)
                    if report_stale:
                        self._last_stale_log_ts = now
                        stale_pending_count = self.stale_pending_count
                if report_stale:
                    print(
                        f"[socket-bridge] DROP stale {self.topic}: "
                        f"queue_age_ms={age_s * 1000:.1f} "
                        f"bytes={len(serialized_msg)} "
                        f"total={stale_pending_count}",
                        flush=True,
                    )
                continue

            try:
                self._publish_now(serialized_msg)
            except Exception as exc:
                # Context-invalid is the fatal path re-raised by _publish_now.
                # Save it so the socket reader closes the connection on the
                # next frame and BridgedPublisher can reconnect to a recreated
                # handler.  Do not let a dead worker silently accept frames.
                with self._state:
                    self._worker_error = exc
                    self._closing = True
                    self._pending_depth = None
                    self._state.notify_all()
                return

    def _publish_now(self, serialized_msg: bytes):
        """Publish original CDR bytes without a Python decode/re-encode."""
        try:
            publish_started = time.monotonic()
            # ROS 2 Humble Publisher.publish(bytes) uses publish_raw /
            # rcl_publish_serialized_message.  The CDR produced in the driver
            # process is therefore forwarded byte-for-byte to domain 42.
            self.pub.publish(serialized_msg)
            publish_ms = (time.monotonic() - publish_started) * 1000
            with self._state:
                self.last_publish_ms = publish_ms
                self.max_publish_ms = max(self.max_publish_ms, publish_ms)
                self.msg_count += 1
                msg_count = self.msg_count
                receive_gap_ms = self.last_receive_gap_ms

            # Debug first few messages for camera and imu
            if ("camera" in self.topic or "imu" in self.topic) and msg_count <= 5:
                print(f"[socket-bridge] {self.topic}: msg #{msg_count}, serialized={len(serialized_msg)} bytes, raw_cdr=true", flush=True)

            now = time.monotonic()
            slow_event = publish_ms >= 100.0 or (
                    self.is_hardware_depth and
                    receive_gap_ms is not None
                    and receive_gap_ms >= 400.0)
            if slow_event:
                with self._state:
                    self._slow_event_count += 1
                    report_slow = (
                        self._slow_event_count == 1 or
                        now - self._last_slow_log_ts >= SLOW_LOG_INTERVAL_S)
                    if report_slow:
                        self._last_slow_log_ts = now
                        slow_event_count = self._slow_event_count
                if report_slow:
                    print(
                        f"[socket-bridge] SLOW {self.topic}: "
                        f"rx_gap_ms={receive_gap_ms!r} "
                        f"dds_publish_ms={publish_ms:.1f} "
                        f"bytes={len(serialized_msg)} "
                        f"total={slow_event_count}",
                        flush=True,
                    )

            # Progress on a clock, not on a message count.
            #
            # Every 100 messages sounds modest until you notice the rate: a
            # 16 kHz audio topic and the joint stream each produced ~750 lines,
            # and all the bridged topics together were 78% of this container's
            # log (2200 of 2809). That crowds out the plugin errors, which are
            # the only reason anyone opens it.
            #
            # What the line is for is "is data still flowing", and a 5-minute
            # tick answers that just as well as a rate-proportional one — while
            # costing the same few lines whether the topic carries 10 Hz of
            # joint state or 16 kHz of audio. The first message is still logged
            # immediately, because "did this bridge ever work" is a different
            # question and it wants an instant answer.
            with self._state:
                report_progress = (
                    msg_count == 1 or
                    now - self._last_progress_ts >= PROGRESS_INTERVAL_S)
                if report_progress:
                    self._last_progress_ts = now
                    received_count = self.received_count
                    coalesced_count = self.coalesced_count
                    stale_pending_count = self.stale_pending_count
            if report_progress:
                depth_stats = ""
                if self.is_hardware_depth:
                    depth_stats = (
                        f", received={received_count}, "
                        f"coalesced={coalesced_count}, "
                        f"stale_pending={stale_pending_count}")
                print(
                    f"[socket-bridge] {self.topic}: published {msg_count} "
                    f"messages{depth_stats}",
                    flush=True,
                )
            return True
        except Exception as e:
            error_msg = str(e)
            print(f"[socket-bridge] ERROR publishing to {self.topic}: {e}", flush=True)

            # If context is invalid, mark this handler as broken so it can be recreated
            if "context is invalid" in error_msg or "publisher's context is invalid" in error_msg:
                print(f"[socket-bridge] Context invalid for {self.topic}, handler needs recreation", flush=True)
                self.context_invalid = True
                raise  # Re-raise to let caller know this handler is broken

            print(f"[socket-bridge]   serialized length: {len(serialized_msg)} bytes", flush=True)
            import traceback
            traceback.print_exc()
            return False

    def close(self, timeout=DEPTH_WORKER_JOIN_TIMEOUT_S) -> bool:
        """Stop the optional depth worker without draining its pending slot.

        Returns whether no worker remains alive.  The result matters during a
        handler recreation: destroying a publisher still in use by a worker is
        unsafe, so the server refuses that replacement if a bounded join fails.
        """
        worker = self._worker
        deadline = time.monotonic() + max(0.0, float(timeout))
        with self._state:
            self._closing = True
            self._pending_depth = None
            self._state.notify_all()
            while self._active_inline_publishes:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    break
                self._state.wait(remaining)

        if worker is not None and worker is not threading.current_thread():
            worker.join(timeout=max(0.0, deadline - time.monotonic()))
        with self._state:
            stopped = (
                self._active_inline_publishes == 0 and
                (worker is None or not worker.is_alive()))
        if not stopped and not self._close_timeout_reported:
            self._close_timeout_reported = True
            print(
                f"[socket-bridge] WARNING: worker for {self.topic} did not "
                f"stop within {float(timeout):.1f}s",
                flush=True,
            )
        return stopped


def qos_from(spec: dict) -> QoSProfile:
    """Rebuild the caller's QoS on this side of the socket.

    Outbound deliberately forces BEST_EFFORT (see TopicHandler) because every
    reader in agent-core is BEST_EFFORT. Inbound has no such luxury: the profile
    has to match whatever is *publishing* on domain 42, and a mismatch there is
    silent — the subscription exists, the publisher exists, and no data moves.
    """
    spec = spec or {}
    reliability = (ReliabilityPolicy.RELIABLE
                   if str(spec.get("reliability", "best_effort")).lower() == "reliable"
                   else ReliabilityPolicy.BEST_EFFORT)
    return QoSProfile(
        reliability=reliability,
        history=HistoryPolicy.KEEP_LAST,
        depth=int(spec.get("depth", 1)),
        durability=DurabilityPolicy.VOLATILE,
    )


class SubscriptionHandler:
    """One inbound topic: subscribes on domain 42, writes frames back to the client.

    The mirror image of TopicHandler, and it exists because the bridge used to
    have no inbound direction at all. The main process cannot subscribe on domain
    42 itself — its participant runs under the vendor DDS profile so that the
    body link on 192.168.41.x stays up, and that participant is invisible to
    agent-core. Publishers had a way across; subscriptions did not, so a card
    that needed to *receive* started cleanly, reported running, and never heard
    anything. `servo` was the first such card.

    One handler per connection rather than per topic: each carries the socket it
    forwards to, and two cards subscribing to one topic are two readers.
    """

    def __init__(self, topic: str, msg_type_name: str, qos: QoSProfile,
                 ctx: Context, executor, conn: socket.socket):
        self.topic = topic
        self.msg_class = get_message(msg_type_name)
        self.conn = conn
        self.msg_count = 0
        self.failed = False
        self._send_lock = threading.Lock()
        # Starts at 0 so the first forwarded message always prints; after that
        # PROGRESS_INTERVAL_S governs. Same contract as TopicHandler.
        self._last_progress_ts = 0.0

        name = f"bridge_in_{topic.strip('/').replace('/', '_')}_{id(conn) & 0xffff:x}"
        self.node = Node(name, context=ctx)
        self.sub = self.node.create_subscription(
            self.msg_class, topic, self._forward, qos)
        executor.add_node(self.node)
        print(f"[socket-bridge] inbound handler created: {topic} ({msg_type_name})",
              flush=True)

    def _forward(self, msg):
        """Same framing as the outbound direction, in the other direction."""
        if self.failed:
            return
        try:
            data = serialize_message(msg)
            with self._send_lock:
                self.conn.sendall(struct.pack("<I", len(data)))
                self.conn.sendall(data)
            self.msg_count += 1
            # On a clock, not a message count — the same reasoning as the
            # outbound direction, which was changed first and left this one
            # behind. A count-based trigger is loudest on the fastest topics,
            # which are exactly the ones needing the least commentary, and
            # nearly silent on slow ones where a stall would matter most.
            #
            # Inbound topics happen to be low-rate today, so this is not what is
            # flooding the log. It is the same latent bug all the same: the day
            # something high-rate is wired inbound, it floods.
            now = time.monotonic()
            if self.msg_count == 1 or now - self._last_progress_ts >= PROGRESS_INTERVAL_S:
                self._last_progress_ts = now
                print(f"[socket-bridge] {self.topic}: forwarded {self.msg_count} "
                      f"messages inbound", flush=True)
        except (BrokenPipeError, ConnectionResetError, OSError) as e:
            # Marked rather than raised: this runs on the executor thread, and a
            # raise here would take down every other topic sharing it.
            self.failed = True
            print(f"[socket-bridge] inbound {self.topic}: client gone ({e})",
                  flush=True)
        except Exception as e:      # noqa: BLE001
            # Anything else is our bug, not the client's departure, and saying
            # "client gone" about a NameError sends the next person to look at
            # the socket. Caught for the same reason as above, reported honestly.
            self.failed = True
            print(f"[socket-bridge] inbound {self.topic}: forwarding failed "
                  f"({type(e).__name__}: {e})", flush=True)
            import traceback
            traceback.print_exc()

    def close(self, executor):
        try:
            executor.remove_node(self.node)
        finally:
            # destroy_node, not only remove_node: otherwise the subscription and
            # the ROS node name leak, and the next start collides with itself.
            self.node.destroy_node()


class SocketBridgeServer:
    """Manages Unix socket server and topic handlers."""

    SOCKET_DIR = "/tmp/tianyi_bridge"

    # Loopback-only profile every other container on domain 42 loads. Set here
    # rather than inherited from the environment, for two reasons:
    #
    #  * Inheriting is what broke. This process only ever *read* the variable
    #    and printed it; the container never set it, because setting it in
    #    compose would apply to main.py too and confine the body context to
    #    127.0.0.1, cutting the vendor link on 192.168.41.x. So the bridge fell
    #    back to FastDDS's default transport, bound every interface, and stopped
    #    matching the loopback-only readers in agent-core and perception.
    #  * Setting it per process is the documented way out of "one profile per
    #    process": this is a separate process from main.py, so its profile is
    #    its own. joints_bridge_v2.run_publisher already does exactly this.
    #
    # The symptom was size-dependent and therefore misleading: small topics
    # (joints, ext_mic) still got through often enough to look healthy, while
    # 47 KB camera frames were dropped almost entirely — so the camera looked
    # broken and everything else looked fine.
    DDS_LOCAL_PROFILE = "/opt/phanthy-motus/dds-local.xml"

    def __init__(self):
        # Must precede rclpy.init: FastDDS reads the variable when the
        # participant is created and caches the parsed profile process-wide.
        if os.path.exists(self.DDS_LOCAL_PROFILE):
            os.environ["FASTRTPS_DEFAULT_PROFILES_FILE"] = self.DDS_LOCAL_PROFILE
            print(f"[socket-bridge] using DDS profile {self.DDS_LOCAL_PROFILE}", flush=True)
        else:
            # Loud, because the failure it precedes is not: the bridge would
            # publish happily onto a transport nothing else is listening on.
            print(
                f"[socket-bridge] WARNING: {self.DDS_LOCAL_PROFILE} not found — "
                "falling back to FastDDS defaults, which bind every interface "
                "and will NOT match agent-core's loopback-only readers",
                flush=True,
            )

        _configure_fastdds_publication()

        self.ctx = Context()
        rclpy.init(context=self.ctx, domain_id=42)
        self.executor = rclpy.executors.MultiThreadedExecutor(context=self.ctx)

        dds_config = os.environ.get("FASTRTPS_DEFAULT_PROFILES_FILE", "default")
        print(
            f"[socket-bridge] domain 42 initialized (DDS config: {dds_config})",
            flush=True,
        )

        self.handlers: Dict[str, TopicHandler] = {}
        self._handlers_lock = threading.Lock()
        self._stop_flag = threading.Event()
        self._stop_lock = threading.Lock()
        self._stopped = False

        # Create socket directory
        os.makedirs(self.SOCKET_DIR, exist_ok=True)

    def handle_client(self, conn: socket.socket, addr):
        """Handle a client connection (one per topic)."""
        try:
            # First message: topic metadata (JSON)
            length_bytes = _recv_exact(conn, 4)
            if length_bytes is None:
                return

            metadata_len = struct.unpack("<I", length_bytes)[0]
            if metadata_len > MAX_METADATA_BYTES:
                raise ValueError(
                    f"bridge metadata too large: {metadata_len} bytes")

            # Read metadata in chunks (like message data)
            metadata_bytes = _recv_exact(conn, metadata_len)
            if metadata_bytes is None:
                return

            metadata = json.loads(metadata_bytes.decode("utf-8"))

            topic = metadata["topic"]
            msg_type = metadata["msg_type"]
            # Absent means "out": every client written before the inbound
            # direction existed sends no direction field, and must keep working.
            direction = metadata.get("direction", "out")

            print(f"[socket-bridge] new client: {topic} ({msg_type}) [{direction}]",
                  flush=True)

            if direction == "in":
                self.handle_inbound(conn, topic, msg_type, metadata.get("qos"))
                return

            # Creation and context-invalid replacement are serialized per
            # server.  Two reconnecting clients must never create two workers
            # for the same depth topic.
            with self._handlers_lock:
                current = self.handlers.get(topic)
                if current is None or getattr(current, "context_invalid", False):
                    if current is not None:
                        print(
                            f"[socket-bridge] recreating handler for {topic} "
                            "(previous context was invalid)",
                            flush=True,
                        )
                        if not self._close_handler(current):
                            raise RuntimeError(
                                f"worker for {topic} did not stop; refusing "
                                "unsafe handler replacement")
                        self.handlers.pop(topic, None)
                    self.handlers[topic] = TopicHandler(
                        topic, msg_type, self.ctx, self.executor,
                        metadata.get("qos"))

                handler = self.handlers[topic]

            # Receive and publish messages
            while not self._stop_flag.is_set():
                # Read message length
                length_bytes = _recv_exact(conn, 4)
                if length_bytes is None:
                    break

                msg_len = struct.unpack("<I", length_bytes)[0]
                if msg_len > MAX_MESSAGE_BYTES:
                    raise ValueError(
                        f"bridge message too large: {msg_len} bytes")

                # Read message data
                msg_data = _recv_exact(conn, msg_len)
                if msg_data is None:
                    break

                # Publish
                handler.publish(msg_data)

        except Exception as e:
            print(f"[socket-bridge] client handler error: {e}", flush=True)
        finally:
            conn.close()

    def _close_handler(self, handler: TopicHandler) -> bool:
        """Stop a handler before destroying the ROS entities it owns."""
        if not handler.close():
            return False
        try:
            self.executor.remove_node(handler.node)
        finally:
            handler.node.destroy_node()
        return True

    def handle_inbound(self, conn: socket.socket, topic: str, msg_type: str,
                       qos_spec):
        """Subscribe on domain 42 for this client until it disconnects.

        Nothing is read from the socket afterwards — the client never sends
        again — so `recv` here is purely how a disconnect is noticed. It returns
        b"" on a clean close and raises on a reset; either ends the handler and
        tears the subscription down, which is what keeps a start/stop cycle from
        leaving a live reader behind.
        """
        handler = None
        try:
            handler = SubscriptionHandler(topic, msg_type, qos_from(qos_spec),
                                          self.ctx, self.executor, conn)
            while not self._stop_flag.is_set():
                conn.settimeout(1.0)
                try:
                    if not conn.recv(1):
                        break                      # client closed
                except socket.timeout:
                    if handler.failed:
                        break                      # forwarding side gave up
                    continue
        except Exception as e:      # noqa: BLE001
            print(f"[socket-bridge] inbound {topic} error: {e}", flush=True)
        finally:
            if handler is not None:
                handler.close(self.executor)
            print(f"[socket-bridge] inbound handler closed: {topic}", flush=True)

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
        with self._stop_lock:
            if self._stopped:
                return
            self._stopped = True

        print("[socket-bridge] stopping...", flush=True)
        self._stop_flag.set()

        main_sock = getattr(self, "main_sock", None)
        if main_sock is not None:
            try:
                main_sock.close()
            except OSError:
                pass

        # Stop every depth worker and discard its pending slot while the ROS
        # context is still valid.  Only then is it safe to destroy publishers.
        with self._handlers_lock:
            handlers = list(self.handlers.values())
            self.handlers.clear()
        stopped_handlers = []
        for handler in handlers:
            if handler.close():
                stopped_handlers.append(handler)
            else:
                print(
                    f"[socket-bridge] WARNING: leaving live handler node "
                    f"intact during shutdown: {handler.topic}",
                    flush=True,
                )
        for handler in stopped_handlers:
            try:
                self.executor.remove_node(handler.node)
                handler.node.destroy_node()
            except Exception as exc:
                print(
                    f"[socket-bridge] cleanup error for {handler.topic}: {exc}",
                    flush=True,
                )

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
