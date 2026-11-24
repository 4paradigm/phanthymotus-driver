"""Behaviour tests for the tf_tree card's plugin layer.

These run without ROS installed: the module is imported with lightweight
stand-ins for rclpy, and only the decision logic (what gets published, with
which stamp, and when it is dropped) is exercised here.
"""

from __future__ import annotations

import json
import sys
import types
from pathlib import Path

import pytest

BUMI = str(Path(__file__).resolve().parents[1])
sys.path.insert(0, BUMI)


# ── ROS stand-ins ────────────────────────────────────────────────────────────


class _Stamp:
    def __init__(self, ns=0):
        self.nanoseconds = ns

    def to_msg(self):
        return self.nanoseconds


class _Clock:
    def __init__(self):
        self.now_ns = 1_700_000_000_000_000_000

    def now(self):
        return _Stamp(self.now_ns)


class _Time:
    def __init__(self, *, nanoseconds):
        self.nanoseconds = nanoseconds

    def to_msg(self):
        return self.nanoseconds


class _Transform:
    def __init__(self):
        self.header = types.SimpleNamespace(stamp=None, frame_id="")
        self.child_frame_id = ""
        self.transform = types.SimpleNamespace(
            translation=types.SimpleNamespace(x=0.0, y=0.0, z=0.0),
            rotation=types.SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0))


class _Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg)


class _Broadcaster:
    def __init__(self, *args, **kwargs):
        self.sent = []

    def sendTransform(self, items):
        self.sent.append(items)


class _Node:
    def __init__(self, *args, **kwargs):
        self._clock = _Clock()
        self.clock = self._clock

    def get_clock(self):
        return self._clock

    def create_publisher(self, *args, **kwargs):
        return _Publisher()

    def create_subscription(self, *args, **kwargs):
        return object()

    def destroy_subscription(self, *args):
        pass


@pytest.fixture
def card_module(monkeypatch):
    rclpy = types.ModuleType("rclpy")
    rclpy.Node = _Node
    node_mod = types.ModuleType("rclpy.node")
    node_mod.Node = _Node
    time_mod = types.ModuleType("rclpy.time")
    time_mod.Time = _Time
    qos_mod = types.ModuleType("rclpy.qos")
    qos_mod.QoSProfile = lambda **kwargs: kwargs
    for name, values in {
        "ReliabilityPolicy": ("BEST_EFFORT", "RELIABLE"),
        "HistoryPolicy": ("KEEP_LAST", "KEEP_ALL"),
        "DurabilityPolicy": ("VOLATILE", "TRANSIENT_LOCAL"),
    }.items():
        enum = type(name, (), {})
        for value in values:
            setattr(enum, value, value)
        setattr(qos_mod, name, enum)
    std_msgs = types.ModuleType("std_msgs")
    msg_mod = types.ModuleType("std_msgs.msg")
    msg_mod.String = type("String", (), {})
    std_msgs.msg = msg_mod
    geo = types.ModuleType("geometry_msgs")
    geo_msg = types.ModuleType("geometry_msgs.msg")
    geo_msg.TransformStamped = _Transform
    geo.msg = geo_msg
    tf2 = types.ModuleType("tf2_ros")
    tf2.StaticTransformBroadcaster = _Broadcaster
    tf2.TransformBroadcaster = _Broadcaster

    for name, mod in {
        "rclpy": rclpy, "rclpy.node": node_mod, "rclpy.time": time_mod,
        "rclpy.qos": qos_mod, "std_msgs": std_msgs, "std_msgs.msg": msg_mod,
        "geometry_msgs": geo, "geometry_msgs.msg": geo_msg, "tf2_ros": tf2,
    }.items():
        monkeypatch.setitem(sys.modules, name, mod)

    sys.modules.pop("tf_tree_card", None)
    import tf_tree_card
    return tf_tree_card


def _make_card(card_module, *, urdf_path=None, ttl_ms=500):
    cfg = {
        "urdf_path": urdf_path or str(Path(BUMI) / "resource" / "bumi_model.urdf"),
        "publish_rate_hz": 10.0,
        "max_joint_age_s": 0.5,
        "vision_ttl_ms": ttl_ms,
    }
    executor = types.SimpleNamespace(add_node=lambda node: None)
    return card_module.TfTreePlugin(cfg, "bumi_test", executor)


# ── the local-transform bug ──────────────────────────────────────────────────


def test_static_transforms_use_local_origin(card_module, tmp_path):
    # Synthetic URDF: a fixed child of base_link with a non-zero mount offset,
    # plus a revolute grandchild. This card must not depend on which fixed
    # frames happen to exist in the shipped asset (currently none).
    urdf = tmp_path / "two_links.urdf"
    urdf.write_text("""<?xml version="1.0"?>
    <robot name="test">
      <link name="base_link"/><link name="mount"/><link name="arm"/>
      <joint name="mount_joint" type="fixed">
        <parent link="base_link"/><child link="mount"/>
        <origin xyz="0.1 0.2 0.3" rpy="0 0 0"/>
      </joint>
      <joint name="arm_joint" type="revolute">
        <parent link="mount"/><child link="arm"/>
        <origin xyz="0.4 0 0" rpy="0 0 0"/><axis xyz="0 0 1"/>
        <limit lower="-3.14" upper="3.14" effort="1" velocity="1"/>
      </joint>
    </robot>
    """, encoding="utf-8")
    plugin = _make_card(card_module, urdf_path=str(urdf))
    node = plugin._node
    node._publish_static()
    sent = node._static_bc.sent[-1]

    by_child = {t.child_frame_id: t for t in sent}
    assert set(by_child) == {"mount"}
    mount = by_child["mount"]
    assert mount.header.frame_id == "base_link"
    assert (mount.transform.translation.x,
            mount.transform.translation.y,
            mount.transform.translation.z) == pytest.approx((0.1, 0.2, 0.3))


def test_dynamic_transforms_are_local_to_their_parent(card_module):
    plugin = _make_card(card_module)
    node = plugin._node
    sample = json.dumps({"joints": [
        {"name": "l_arm_pitch_joint", "q": 0.0},
        {"name": "l_arm_roll_joint", "q": 0.0},
    ]})
    node._on_joints(types.SimpleNamespace(data=sample))
    sent = node._dynamic_bc.sent[-1]

    by_child = {t.child_frame_id: t for t in sent}
    roll = by_child["l_arm_roll_link"]
    # Its origin xyz is (-0.0010007, 0.044, 0) in the *pitch* link frame —
    # the old code sent (0.0001, 0.126, 0.313), the root-relative pose.
    assert roll.header.frame_id == "l_arm_pitch_link"
    assert roll.transform.translation.x == pytest.approx(-0.0010007, abs=1e-9)
    assert roll.transform.translation.y == pytest.approx(0.044, abs=1e-9)
    assert roll.transform.translation.z == pytest.approx(0.0, abs=1e-9)
    assert len(sent) == 21  # all revolute joints, not just the two with values


# ── the timestamp bug ────────────────────────────────────────────────────────


def test_dynamic_stamp_is_the_joint_sample_time(card_module):
    plugin = _make_card(card_module)
    node = plugin._node
    node.get_clock().now_ns = 1_700_000_999_000_000_000  # pretend publish time
    node._on_joints(types.SimpleNamespace(data='{"joints":[{"name":"l_arm_pitch_joint","q":0.2}]}'))
    # Overwrite the clock AFTER the sample arrived so publish time differs.
    node.get_clock().now_ns = 1_700_001_000_000_000_000
    stamp = node._dynamic_bc.sent[-1][0].header.stamp
    assert stamp == 1_700_000_999_000_000_000
    assert stamp != node.get_clock().now().nanoseconds


def test_stale_joint_sample_stops_dynamic_stream(card_module):
    plugin = _make_card(card_module)
    node = plugin._node
    node._now = None
    # Freeze monotonic so the sample ages deterministically.
    import tf_tree_card as module
    original = module.time.monotonic
    clock = [1000.0]
    module.time.monotonic = lambda: clock[0]
    try:
        node._on_joints(types.SimpleNamespace(data='{"joints":[{"name":"l_arm_pitch_joint","q":0.2}]}'))
        assert node._degraded() == ""
        clock[0] += 1.0
        assert node._degraded() == "joints_stale"
    finally:
        module.time.monotonic = original


def test_zero_camera_mount_is_flagged_as_assumed(card_module):
    """The shipped URDF uses a nominal zero camera translation; the node must
    surface that so downstream never treats the static transform as measured."""
    plugin = _make_card(card_module)
    node = plugin._node
    assert node._camera_extrinsic_assumed is True
    assert plugin.dispatch("info", {})["camera_extrinsic_assumed"] is True


def test_already_stale_joint_sample_is_rejected(card_module):
    """P1 regression: a joint sample whose producer stamp is already older than
    max_joint_age_s when read must not be published as a fresh pose."""
    plugin = _make_card(card_module)  # max_joint_age_s = 0.5
    node = plugin._node
    now_ns = node.get_clock().now().nanoseconds
    stale_ns = now_ns - int(2.0 * 1e9)  # 2 s old at receipt
    msg = types.SimpleNamespace(data=json.dumps({
        "stamp_ns": stale_ns, "timestamp_source": "read_completion",
        "joints": [{"name": "l_arm_pitch_joint", "q": 0.2}],
    }))
    node._dynamic_bc.sent.clear()
    node._on_joints(msg)
    assert node._dynamic_bc.sent == []  # nothing published from a stale sample
    assert node._stale_joint_samples == 1
    assert node._last_joint_at is None  # never accepted as a fresh sample


# ── vision lifecycle ─────────────────────────────────────────────────────────


def _vision_msg(entries):
    return types.SimpleNamespace(data=json.dumps({"transforms": entries}))


def test_vision_transform_uses_its_own_stamp(card_module):
    plugin = _make_card(card_module)
    node = plugin._node
    import time as real_time
    now = int(real_time.time() * 1000)
    node._on_vision(_vision_msg([{
        "parent": "camera_optical_frame", "child": "person_head",
        "stamp_ms": now, "translation": [0, 0, 1], "rotation": [0, 0, 0, 1],
    }]))
    assert node._dynamic_bc.sent[-1][0].header.stamp == pytest.approx(
        now * 1_000_000, abs=1000)  # float ms → ns rounding


def test_vision_entries_expire_when_upstream_goes_silent(card_module):
    plugin = _make_card(card_module, ttl_ms=100)
    node = plugin._node
    import time as real_time
    stale = int(real_time.time() * 1000) - 5000
    node._vision["person_head"] = {
        "parent": "camera_optical_frame", "child": "person_head",
        "stamp_ms": stale, "translation": [0, 0, 1], "rotation": [0, 0, 0, 1],
    }
    node._vision_parent["person_head"] = "camera_optical_frame"
    node._prune_vision(int(real_time.time() * 1000))
    assert node._vision == {}
    assert node._vision_parent == {}


def test_conflict_degraded_recovers(card_module):
    plugin = _make_card(card_module)
    node = plugin._node
    import time as real_time
    now = int(real_time.time() * 1000)
    first = {"parent": "camera_optical_frame", "child": "head",
             "stamp_ms": now, "translation": [0, 0, 1], "rotation": [0, 0, 0, 1]}
    second = dict(first, parent="base_link")

    node._on_joints(types.SimpleNamespace(data='{"joints":[{"name":"l_arm_pitch_joint","q":0.0}]}'))
    node._on_vision(_vision_msg([first]))
    assert node._degraded() == ""  # joints fresh, one unambiguous parent
    # A parent changing between batches is legitimate, not a conflict.
    node._on_vision(_vision_msg([second]))
    assert node._degraded() == ""
    # Two parents for the SAME child in ONE batch is the real conflict.
    node._on_vision(_vision_msg([first, second]))
    assert node._degraded() == "conflicting_parent"
    # Next unambiguous batch clears the transient degradation.
    node._on_vision(_vision_msg([dict(first, stamp_ms=now + 100)]))
    assert node._degraded() == ""
    # The cumulative counter still records that a conflict happened.
    assert node._conflicting_parent == 1


def test_vision_callback_and_ttl_sweep_are_concurrent_safe(card_module):
    """P1 regression: the ROS callback writes the vision maps while the status
    thread prunes them. Without a shared lock this raises
    'dictionary changed size during iteration' and kills the thread."""
    import threading
    import time as real_time

    plugin = _make_card(card_module, ttl_ms=100)
    node = plugin._node
    errors = []

    now = int(real_time.time() * 1000)
    def entry(i):
        return {"parent": "camera_optical_frame", "child": f"obj_{i}",
                "stamp_ms": now, "translation": [0, 0, 1],
                "rotation": [0, 0, 0, 1]}

    def writer():
        try:
            for i in range(200):
                node._on_vision(_vision_msg([entry(i % 10)]))
        except Exception as exc:  # pragma: no cover - fails only without the lock
            errors.append(exc)

    def sweeper():
        try:
            for _ in range(200):
                with node._vision_lock:
                    node._prune_vision(int(real_time.time() * 1000))
                    _ = len(node._vision)
                _ = node.snapshot()
        except Exception as exc:  # pragma: no cover
            errors.append(exc)

    threads = [threading.Thread(target=writer), threading.Thread(target=sweeper)]
    for t in threads:
        t.start()
    for t in threads:
        t.join(timeout=5)
    assert all(not t.is_alive() for t in threads)
    assert errors == []


# ── registration surface ─────────────────────────────────────────────────────


def test_tool_declares_no_pose_output(card_module):
    plugin = _make_card(card_module)
    tool = plugin.get_tool()
    assert tool["name"] == "tf_tree"
    assert tool["type"] == "processor"
    assert tool["topic_in"] == [{"format": "data/json"}]
    assert tool["topic_out"][0]["format"] == "data/json"
    assert "status" in tool["topic_out"][0]["topic"]
    assert "info" in tool["inputSchema"]["properties"]["action"]["enum"]


def test_start_builds_tree_without_vision_topic(card_module):
    plugin = _make_card(card_module)
    plugin.start()  # what start_all() calls
    assert plugin._node.running
    assert plugin._node._vision == {}


def test_dispatched_start_attaches_vision_topic_after_boot(card_module):
    """P1 regression: tf_tree is on by default, so boot start() runs first.
    A later canvas dispatch("start", {input_topic}) must still subscribe to the
    vision topic; an early `if self._running: return` silently drops the wiring.
    """
    plugin = _make_card(card_module)
    plugin.start()  # start_all(): no vision topic yet
    assert plugin._node.running

    subscribed = []
    original = plugin._node._subscribe_vision
    plugin._node._subscribe_vision = lambda topic: (
        subscribed.append(topic), original(topic))[1]

    result = plugin.dispatch("start", {"input_topic": "/bumi_test/vision_tf/ar/transforms"})
    assert result["state"] == "running"
    assert subscribed == ["/bumi_test/vision_tf/ar/transforms"]
    plugin.stop()
