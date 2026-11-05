"""The Tianyi socket bridge — specifically the inbound direction.

The bridge exists because the main driver process cannot use domain 42 directly:
its participant runs under the vendor DDS profile so the body link on
192.168.41.x stays up, and that participant is invisible to agent-core. For a
long time the bridge only carried *publishers*, which was enough while every
card only reported outward. `servo` was the first card that needed to receive a
command stream, and it started cleanly, reported running, and never heard
anything.

What these pin down:

  routing     which topics cross the bridge, in each direction — the publisher
              side is a whitelist and got `/nvidia_desktop/servo/state` wrong,
              the subscription side cannot be one
  protocol    the metadata frame the bridge dispatches on, including the QoS
              that has to match whatever is publishing on domain 42
  framing     length-prefixed messages arriving in the callback, including the
              case that matters most — a message split across recv boundaries
  teardown    a destroyed subscription stops its thread and closes its socket,
              because the canvas does start/stop/start within seconds

No robot, no ROS: `rclpy` is stubbed. The socket, however, is a real Unix
socket — the framing is the thing under test and a faked transport would be
testing the fake.

Run: PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest tests/test_tianyi_bridge.py -q
"""

from __future__ import annotations

import json
import os
import shutil
import socket
import struct
import sys
import tempfile
import threading
import time
import types
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
DRIVER = ROOT / "x-humanoid" / "tianyi2.0"


def _stub_rclpy():
    """Only what bridged_publisher.py imports at module level."""
    rclpy = types.ModuleType("rclpy")

    node_mod = types.ModuleType("rclpy.node")
    node_mod.Node = type("Node", (), {})

    publisher_mod = types.ModuleType("rclpy.publisher")
    publisher_mod.Publisher = type("Publisher", (), {})

    qos_mod = types.ModuleType("rclpy.qos")

    class ReliabilityPolicy:
        RELIABLE = "reliable"
        BEST_EFFORT = "best_effort"

    class QoSProfile:
        def __init__(self, reliability=ReliabilityPolicy.BEST_EFFORT, depth=1,
                     **kwargs):
            self.reliability = reliability
            self.depth = depth

    qos_mod.QoSProfile = QoSProfile
    qos_mod.ReliabilityPolicy = ReliabilityPolicy

    # Identity (de)serialization: the framing is under test, not CDR.
    ser_mod = types.ModuleType("rclpy.serialization")
    ser_mod.serialize_message = lambda msg: msg
    ser_mod.deserialize_message = lambda payload, msg_type: payload

    for name, module in (("rclpy", rclpy), ("rclpy.node", node_mod),
                         ("rclpy.publisher", publisher_mod),
                         ("rclpy.qos", qos_mod),
                         ("rclpy.serialization", ser_mod)):
        sys.modules.setdefault(name, module)
    return qos_mod


QOS = _stub_rclpy()
sys.path.insert(0, str(DRIVER))

import bridged_publisher as bp  # noqa: E402


class FakeMsg:
    """Stands in for a ROS message class; only __module__/__name__ are read."""


FakeMsg.__module__ = "std_msgs.msg"


# ── routing ──────────────────────────────────────────────────────────────────

def test_the_servo_state_topic_is_bridged():
    """It was not, and the reason is a trap worth keeping a test on.

    `/nvidia_desktop/servo/state` does not contain `"/state/"` — the segment is
    last, so there is no trailing slash — and the whitelist matches substrings.
    It published onto the invisible participant and agent-core never saw it.
    """
    assert bp.should_use_bridge("/nvidia_desktop/servo/state")


def test_body_controller_topics_are_never_bridged():
    """Domain-0 commands must go straight to the robot. Routing one of these
    through the bridge would put arm commands on the wrong domain entirely."""
    for topic in ("/arm/cmd_pos", "/inspire_hand/ctrl/left_hand", "/arm/status"):
        assert not bp.should_use_bridge(topic), topic


def test_subscriptions_are_bridged_unconditionally():
    """Not a whitelist, unlike publishers: a domain-42 subscription on the main
    process receives nothing at all, so a topic left off a list would fall back
    to silence rather than to a slower path."""
    for topic in ("/actucore/vla/cmd", "/anything/at/all", "/x"):
        assert bp.should_bridge_subscription(topic)


def test_outbound_publish_reports_when_bridge_is_unavailable(sock_dir, monkeypatch):
    monkeypatch.setattr(bp.BridgedPublisher, "SOCKET_DIR", str(sock_dir))
    publisher = bp.BridgedPublisher(
        node=types.SimpleNamespace(), msg_type=FakeMsg, topic="/depth",
        qos=QOS.QoSProfile())
    assert publisher.publish(b"frame") is False


def test_outbound_publish_reports_an_accepted_frame():
    writes = []
    publisher = object.__new__(bp.BridgedPublisher)
    publisher._connected = True
    publisher._socket = types.SimpleNamespace(sendall=writes.append)
    publisher._send_lock = threading.Lock()
    publisher._connect_lock = threading.Lock()
    publisher._msg_count = 0
    publisher.topic = "/depth"
    assert publisher.publish(b"frame") is True
    assert b"frame" in writes


def test_integer_qos_shorthand_is_normalized_for_bridge_metadata():
    """Several legacy cards pass create_publisher(..., 10), not QoSProfile."""
    assert bp._qos_metadata(10) == {
        "reliability": "best_effort",
        "depth": 10,
    }


# ── the protocol ─────────────────────────────────────────────────────────────

@pytest.fixture
def sock_dir():
    """A short temp dir. pytest's tmp_path is far past the ~104 character limit
    macOS puts on an AF_UNIX path, and the failure is an unhelpful OSError."""
    directory = tempfile.mkdtemp(prefix="brg", dir="/tmp")
    yield Path(directory)
    shutil.rmtree(directory, ignore_errors=True)


@pytest.fixture
def bridge(sock_dir, monkeypatch):
    """A real Unix socket standing in for socket_bridge.py."""
    monkeypatch.setattr(bp.BridgedPublisher, "SOCKET_DIR", str(sock_dir))
    monkeypatch.setattr(bp.BridgedSubscription, "SOCKET_DIR", str(sock_dir))
    monkeypatch.setattr(bp.BridgedSubscription, "RETRY_S", 0.01)
    path = sock_dir / bp.BridgedSubscription.MAIN_SOCKET

    server = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    server.bind(str(path))
    server.listen(4)
    server.settimeout(5.0)

    state = {}

    def accept():
        conn, _ = server.accept()
        length = struct.unpack("<I", conn.recv(4))[0]
        state["metadata"] = json.loads(conn.recv(length).decode())
        state["conn"] = conn

    thread = threading.Thread(target=accept, daemon=True)
    thread.start()
    yield state, thread
    try:
        if "conn" in state:
            state["conn"].close()
        server.close()
    except OSError:
        pass


def _subscribe(callback, qos=None):
    return bp.BridgedSubscription(
        node=types.SimpleNamespace(__dict__={}), msg_type=FakeMsg,
        topic="/actucore/vla/cmd", callback=callback,
        qos=qos or QOS.QoSProfile(reliability=QOS.ReliabilityPolicy.BEST_EFFORT,
                                  depth=1))


def test_the_metadata_frame_says_inbound_and_carries_the_qos(bridge):
    """Direction is what the bridge dispatches on, and its absence means "out"
    so that every publisher written before this existed keeps working. QoS is
    sent rather than assumed: a mismatch against the domain-42 publisher moves
    no data while both ends look healthy."""
    state, thread = bridge
    subscription = _subscribe(lambda msg: None)
    thread.join(timeout=5)

    metadata = state["metadata"]
    assert metadata["direction"] == "in"
    assert metadata["topic"] == "/actucore/vla/cmd"
    assert metadata["msg_type"] == "std_msgs/msg/FakeMsg"
    assert metadata["qos"] == {"reliability": "best_effort", "depth": 1}
    subscription.destroy()


def test_outbound_metadata_preserves_the_requested_queue_depth(bridge):
    """camera_depth's depth=1 must survive the cross-domain hop."""
    state, thread = bridge
    publisher = bp.BridgedPublisher(
        node=types.SimpleNamespace(), msg_type=FakeMsg,
        topic="/nvidia_desktop/camera/head/depth/visual_depth",
        qos=QOS.QoSProfile(
            reliability=QOS.ReliabilityPolicy.BEST_EFFORT, depth=1),
    )
    thread.join(timeout=5)

    assert state["metadata"]["qos"] == {
        "reliability": "best_effort",
        "depth": 1,
    }
    publisher.destroy()


def test_a_forwarded_message_reaches_the_callback(bridge):
    state, thread = bridge
    got = []
    subscription = _subscribe(got.append)
    thread.join(timeout=5)

    payload = b"one command"
    state["conn"].sendall(struct.pack("<I", len(payload)) + payload)

    deadline = time.time() + 5
    while not got and time.time() < deadline:
        time.sleep(0.01)
    assert got == [payload]
    subscription.destroy()


def test_a_message_split_across_reads_is_reassembled(bridge):
    """The failure this prevents is not a dropped message but a corrupt one:
    a short read that was treated as complete would desynchronise the stream
    and every subsequent frame would be garbage."""
    state, thread = bridge
    got = []
    subscription = _subscribe(got.append)
    thread.join(timeout=5)

    payload = bytes(range(256)) * 40          # comfortably over one chunk
    conn = state["conn"]
    conn.sendall(struct.pack("<I", len(payload)))
    for offset in range(0, len(payload), 700):
        conn.sendall(payload[offset:offset + 700])
        time.sleep(0.002)

    deadline = time.time() + 5
    while not got and time.time() < deadline:
        time.sleep(0.01)
    assert got == [payload]
    subscription.destroy()


def test_a_raising_callback_does_not_kill_the_link(bridge):
    """One bad frame must not end the subscription for good — the card would go
    deaf with nothing in its own logs to say so."""
    state, thread = bridge
    seen = []

    def callback(msg):
        seen.append(msg)
        if len(seen) == 1:
            raise ValueError("boom")

    subscription = _subscribe(callback)
    thread.join(timeout=5)

    for payload in (b"first", b"second"):
        state["conn"].sendall(struct.pack("<I", len(payload)) + payload)

    deadline = time.time() + 5
    while len(seen) < 2 and time.time() < deadline:
        time.sleep(0.01)
    assert seen == [b"first", b"second"]
    subscription.destroy()


# ── the server side ──────────────────────────────────────────────────────────
#
# These exist because the first version of the inbound path shipped with
# `serialize_message` used but never imported, and the tests above did not catch
# it: they only ever exercised the client. The NameError surfaced on hardware,
# reported as "client gone", which is the wrong place to send the next person.

def _server_module():
    """socket_bridge.py with its ROS surface stubbed, importable on a laptop."""
    import importlib

    context_mod = types.ModuleType("rclpy.context")
    context_mod.Context = type("Context", (), {})
    executors_mod = types.ModuleType("rclpy.executors")
    executors_mod.MultiThreadedExecutor = type("MultiThreadedExecutor", (), {})
    utilities_mod = types.ModuleType("rosidl_runtime_py.utilities")
    utilities_mod.get_message = lambda name: FakeMsg
    runtime_pkg = types.ModuleType("rosidl_runtime_py")
    yaml_mod = types.ModuleType("yaml")
    yaml_mod.safe_load = lambda *a, **k: {}

    qos = sys.modules["rclpy.qos"]
    qos.HistoryPolicy = type("HistoryPolicy", (), {"KEEP_LAST": "keep_last"})
    qos.DurabilityPolicy = type("DurabilityPolicy", (), {"VOLATILE": "volatile"})

    for name, module in (("rclpy.context", context_mod),
                         ("rclpy.executors", executors_mod),
                         ("rosidl_runtime_py", runtime_pkg),
                         ("rosidl_runtime_py.utilities", utilities_mod),
                         ("yaml", yaml_mod)):
        sys.modules.setdefault(name, module)
    return importlib.import_module("socket_bridge")


class _ServerNode:
    """Small Node surface for exercising the outbound bridge handler."""

    published = None

    def __init__(self, *args, **kwargs):
        self.created_qos = None
        self.destroyed = False

    def create_publisher(self, msg_class, topic, qos):
        self.created_qos = qos
        messages = []
        self.published = messages
        return types.SimpleNamespace(publish=messages.append)

    def destroy_node(self):
        self.destroyed = True


class _Executor:
    def __init__(self):
        self.removed = []

    def add_node(self, node):
        self.node = node

    def remove_node(self, node):
        self.removed.append(node)


def _wait_until(predicate, timeout=1.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.002)
    return bool(predicate())


def test_hardware_depth_is_raw_relayed_with_depth_one_qos(monkeypatch):
    server = _server_module()
    monkeypatch.setattr(server, "Node", _ServerNode)
    monkeypatch.setattr(
        server, "QoSProfile", lambda **kwargs: types.SimpleNamespace(**kwargs))
    payload = bytes(range(256)) * 1200
    handler = server.TopicHandler(
        "/nvidia_desktop/camera/head/depth/visual_depth",
        "std_msgs/msg/FakeMsg", object(), _Executor(),
        {"reliability": "best_effort", "depth": 1},
    )

    try:
        handler.publish(payload)

        assert _wait_until(lambda: handler.node.published == [payload])
        assert handler.node.published[0] is payload
        assert handler.node.created_qos.depth == 1
        assert handler.msg_count == 1
        assert handler.max_publish_ms >= 0
    finally:
        assert handler.close()


class _BlockingPublisher:
    def __init__(self, fail=None):
        self.entered = threading.Event()
        self.release = threading.Event()
        self.published = []
        self.fail = fail

    def publish(self, payload):
        self.published.append(payload)
        self.entered.set()
        if len(self.published) == 1:
            assert self.release.wait(2)
        if self.fail is not None:
            raise self.fail


def _depth_handler(monkeypatch):
    server = _server_module()
    monkeypatch.setattr(server, "Node", _ServerNode)
    monkeypatch.setattr(
        server, "QoSProfile", lambda **kwargs: types.SimpleNamespace(**kwargs))
    handler = server.TopicHandler(
        "/nvidia_desktop/camera/head/depth/visual_depth",
        "std_msgs/msg/FakeMsg", object(), _Executor(),
        {"reliability": "best_effort", "depth": 1},
    )
    return server, handler


def test_depth_worker_keeps_one_latest_pending_frame(monkeypatch):
    server, handler = _depth_handler(monkeypatch)
    publisher = _BlockingPublisher()
    handler.pub = publisher
    try:
        assert handler.publish(b"A")
        assert publisher.entered.wait(1)
        assert handler.publish(b"B")
        assert handler.publish(b"C")
        with handler._state:
            assert handler._pending_depth == (b"C", handler._pending_depth[1])
            assert handler.coalesced_count == 1
        publisher.release.set()
        assert _wait_until(lambda: publisher.published == [b"A", b"C"])
        assert handler.received_count == 3
        assert handler.msg_count == 2
    finally:
        publisher.release.set()
        assert handler.close()


def test_depth_worker_drops_a_pending_frame_at_the_delivery_deadline(monkeypatch):
    server, handler = _depth_handler(monkeypatch)
    assert server.DEPTH_MAX_PENDING_AGE_S == pytest.approx(0.05)
    publisher = _BlockingPublisher()
    handler.pub = publisher
    try:
        assert handler.publish(b"in-flight")
        assert publisher.entered.wait(1)
        assert handler.publish(b"stale")
        # Meet the configured local deadline without making the unit test wait.
        # It is intentionally below Navi's 500 ms lease so DDS has time left.
        with handler._state:
            payload, received_at = handler._pending_depth
            handler._pending_depth = (
                payload, received_at - server.DEPTH_MAX_PENDING_AGE_S)
        publisher.release.set()
        assert _wait_until(lambda: handler.stale_pending_count == 1)
        assert publisher.published == [b"in-flight"]
        assert handler.msg_count == 1
    finally:
        publisher.release.set()
        assert handler.close()


def test_repeated_stale_depth_drops_are_log_rate_limited(monkeypatch, capsys):
    server, handler = _depth_handler(monkeypatch)
    try:
        for payload in (b"stale-1", b"stale-2"):
            with handler._state:
                handler._pending_depth = (
                    payload,
                    time.monotonic() - server.DEPTH_MAX_PENDING_AGE_S,
                )
                handler._state.notify()
            assert _wait_until(
                lambda: handler.stale_pending_count ==
                (1 if payload == b"stale-1" else 2))

        output = capsys.readouterr().out
        assert output.count("DROP stale") == 1
        assert "total=1" in output
    finally:
        assert handler.close()


def test_depth_close_discards_pending_and_is_idempotent(monkeypatch):
    _, handler = _depth_handler(monkeypatch)
    publisher = _BlockingPublisher()
    handler.pub = publisher
    close_result = []
    try:
        handler.publish(b"in-flight")
        assert publisher.entered.wait(1)
        handler.publish(b"must-not-drain")
        closer = threading.Thread(
            target=lambda: close_result.append(handler.close()), daemon=True)
        closer.start()
        assert _wait_until(lambda: handler._closing)
        with handler._state:
            assert handler._pending_depth is None
        publisher.release.set()
        closer.join(1)
        assert close_result == [True]
        assert publisher.published == [b"in-flight"]
        assert handler.close()
        with pytest.raises(RuntimeError, match="closed"):
            handler.publish(b"after-close")
    finally:
        publisher.release.set()
        handler.close()


def test_depth_worker_failure_is_reported_to_the_next_submit(monkeypatch):
    _, handler = _depth_handler(monkeypatch)
    publisher = _BlockingPublisher(
        RuntimeError("publisher's context is invalid"))
    publisher.release.set()
    handler.pub = publisher
    try:
        handler.publish(b"broken")
        assert _wait_until(lambda: handler.context_invalid)
        assert _wait_until(lambda: handler._worker_error is not None)
        with pytest.raises(RuntimeError, match="context is invalid"):
            handler.publish(b"next")
    finally:
        publisher.release.set()
        assert handler.close()


def test_other_outbound_topics_keep_the_historical_queue_depth(monkeypatch):
    server = _server_module()
    monkeypatch.setattr(server, "Node", _ServerNode)
    handler = server.TopicHandler(
        "/nvidia_desktop/state/odom", "std_msgs/msg/FakeMsg",
        object(), _Executor(), {"depth": 1},
    )
    try:
        assert handler.node.created_qos is server.BEST_EFFORT_QOS
        # Non-depth topics keep the historical inline path: publication has
        # completed when publish() returns, rather than entering the worker.
        assert handler.publish(b"odom") is None
        assert handler.node.published == [b"odom"]
        assert handler._worker is None
    finally:
        assert handler.close()


def test_publish_error_does_not_log_raw_cdr_bytes(monkeypatch, capsys):
    server = _server_module()
    monkeypatch.setattr(server, "Node", _ServerNode)
    handler = server.TopicHandler(
        "/nvidia_desktop/state/odom", "std_msgs/msg/FakeMsg",
        object(), _Executor(), {"depth": 1},
    )
    secret = b"raw-cdr-secret-that-must-not-reach-logs"
    handler.pub = types.SimpleNamespace(
        publish=lambda _: (_ for _ in ()).throw(ValueError("publish failed")))
    try:
        assert handler.publish(secret) is None
        output = capsys.readouterr().out
        assert "serialized length" in output
        assert secret.decode() not in output
        assert "first 100 bytes" not in output
    finally:
        assert handler.close()


def test_slow_warning_does_not_flag_intentionally_low_rate_topics(
        monkeypatch, capsys):
    server = _server_module()
    monkeypatch.setattr(server, "Node", _ServerNode)
    handler = server.TopicHandler(
        "/nvidia_desktop/state/battery", "std_msgs/msg/FakeMsg",
        object(), _Executor(), {"depth": 1},
    )
    handler._last_receive_ts = server.time.monotonic() - 1.0

    try:
        handler.publish(b"battery")
        assert "SLOW" not in capsys.readouterr().out
    finally:
        assert handler.close()


def test_exact_reader_reassembles_split_header_and_large_payload():
    server = _server_module()
    left, right = socket.socketpair()
    payload = bytes(range(256)) * 1200

    def write_in_fragments():
        framed = struct.pack("<I", len(payload)) + payload
        for offset in range(0, len(framed), 997):
            left.sendall(framed[offset:offset + 997])
        left.close()

    writer = threading.Thread(target=write_in_fragments, daemon=True)
    writer.start()
    header = server._recv_exact(right, 4)
    assert struct.unpack("<I", header)[0] == len(payload)
    assert server._recv_exact(right, len(payload)) == payload
    writer.join(timeout=5)
    right.close()


def test_fastdds_async_mode_is_selected_before_participant_creation(
        monkeypatch):
    server = _server_module()
    monkeypatch.setenv("RMW_FASTRTPS_USE_QOS_FROM_XML", "1")
    monkeypatch.setenv("RMW_FASTRTPS_PUBLICATION_MODE", "SYNCHRONOUS")

    server._configure_fastdds_publication()

    assert os.environ["RMW_FASTRTPS_PUBLICATION_MODE"] == "ASYNCHRONOUS"
    assert "RMW_FASTRTPS_USE_QOS_FROM_XML" not in os.environ


def test_server_stop_closes_handlers_before_ros_shutdown(monkeypatch, tmp_path):
    server = _server_module()
    events = []

    class Handler:
        topic = "/depth"
        node = types.SimpleNamespace(
            destroy_node=lambda: events.append("destroy"))

        def close(self):
            events.append("close")
            return True

    class Executor:
        def remove_node(self, node):
            events.append("remove")

        def shutdown(self):
            events.append("executor-shutdown")

    instance = server.SocketBridgeServer.__new__(server.SocketBridgeServer)
    instance.SOCKET_DIR = str(tmp_path)
    instance.handlers = {"/depth": Handler()}
    instance._handlers_lock = threading.Lock()
    instance._stop_flag = threading.Event()
    instance._stop_lock = threading.Lock()
    instance._stopped = False
    instance.executor = Executor()
    instance.ctx = object()
    instance.main_sock = types.SimpleNamespace(
        close=lambda: events.append("socket-close"))
    monkeypatch.setattr(
        server.rclpy, "shutdown",
        lambda context: events.append("rclpy-shutdown"), raising=False)

    instance.stop()
    instance.stop()  # idempotent

    assert events.count("close") == 1
    assert events.index("close") < events.index("remove")
    assert events.index("remove") < events.index("destroy")
    assert events.index("destroy") < events.index("executor-shutdown")
    assert events.index("executor-shutdown") < events.index("rclpy-shutdown")


def test_handler_replacement_never_destroys_a_live_worker():
    server = _server_module()
    events = []
    instance = server.SocketBridgeServer.__new__(server.SocketBridgeServer)
    instance.executor = types.SimpleNamespace(
        remove_node=lambda node: events.append("remove"))
    handler = types.SimpleNamespace(
        close=lambda: events.append("close") or False,
        node=types.SimpleNamespace(
            destroy_node=lambda: events.append("destroy")),
    )

    assert instance._close_handler(handler) is False
    assert events == ["close"]


def test_the_server_can_serialize_what_it_forwards():
    """The import that was missing. `_forward` is the only place the bridge
    serializes rather than deserializes, so nothing else would have caught it."""
    server = _server_module()

    assert hasattr(server, "serialize_message"), (
        "socket_bridge must import serialize_message — the inbound direction "
        "serializes on the way out, and its absence fails only at runtime")


def test_the_server_frames_a_forwarded_message_the_way_the_client_reads_it():
    """One test across both halves of the protocol: if the two ever disagree on
    framing, every inbound message is garbage and nothing says why."""
    server = _server_module()
    left, right = socket.socketpair()

    handler = server.SubscriptionHandler.__new__(server.SubscriptionHandler)
    handler.topic = "/actucore/vla/cmd"
    handler.conn = left
    handler.msg_count = 0
    handler.failed = False
    handler._send_lock = threading.Lock()

    handler._forward(b"a command")

    header = right.recv(4)
    assert struct.unpack("<I", header)[0] == len(b"a command")
    assert right.recv(len(b"a command")) == b"a command"
    assert not handler.failed
    left.close(); right.close()


def test_a_dead_client_is_reported_as_such_and_our_own_bugs_are_not():
    """"client gone" about a NameError is a lie that costs an afternoon."""
    server = _server_module()
    left, right = socket.socketpair()
    right.close()

    handler = server.SubscriptionHandler.__new__(server.SubscriptionHandler)
    handler.topic = "/actucore/vla/cmd"
    handler.conn = left
    handler.msg_count = 0
    handler.failed = False
    handler._send_lock = threading.Lock()

    handler._forward(b"x")          # peer is gone: an OSError path
    assert handler.failed
    left.close()


# ── teardown ─────────────────────────────────────────────────────────────────

def test_destroy_stops_the_reader_and_closes_the_socket(bridge):
    """Without this a stop leaves the thread alive, still delivering into a
    callback whose plugin believes it was torn down."""
    state, thread = bridge
    subscription = _subscribe(lambda msg: None)
    thread.join(timeout=5)

    subscription.destroy()

    deadline = time.time() + 5
    while subscription._thread.is_alive() and time.time() < deadline:
        time.sleep(0.01)
    assert not subscription._thread.is_alive()
    assert subscription._socket is None


def test_the_subscription_is_registered_for_node_teardown():
    """`create_bridged_subscription` records it on the node so the patched
    destroy_node can find it; rclpy knows nothing about these objects."""
    node = types.SimpleNamespace()
    node.__dict__["_bridged_subscriptions"] = []
    subscription = bp.create_bridged_subscription(
        node, FakeMsg, "/actucore/vla/cmd", lambda m: None,
        QOS.QoSProfile(depth=1))

    assert node.__dict__["_bridged_subscriptions"] == [subscription]
    subscription.destroy()


def test_a_missing_bridge_is_survivable(sock_dir, monkeypatch):
    """The bridge may be starting, restarting or gone. A card that died because
    a helper process blinked would be worse than one that waits for it."""
    monkeypatch.setattr(bp.BridgedSubscription, "SOCKET_DIR", str(sock_dir))
    monkeypatch.setattr(bp.BridgedSubscription, "RETRY_S", 0.01)
    assert not os.path.exists(sock_dir / bp.BridgedSubscription.MAIN_SOCKET)

    subscription = _subscribe(lambda msg: None)
    time.sleep(0.1)

    assert subscription._thread.is_alive()      # retrying, not dead
    subscription.destroy()
