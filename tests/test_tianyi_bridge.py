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
