"""Embedded tf capability of the visual-odometry card.

The two standalone tf cards (vision_tf, tf_tree) were retired per CTO direction:
"full localisation tf is an implicit capability of a functional card, not
scattered cards". These tests pin the new contract:

  * the embedded tree is OFF unless plugins.visual_odometry.tf.enabled is true;
  * when on, an accepted camera odom pose is published as a dynamic tf edge
    from the estimator's own stamp (not publish time);
  * stop pauses the tree; close destroys it; reset can reuse it;
  * the standalone card plugins are no longer registered by the bundle.

Only ROS transport and the tf-tree node are faked — the lifecycle and wiring
decisions are the real VisualOdometryNode code.
"""
from __future__ import annotations

import sys
import types
from types import SimpleNamespace as NS

import pytest

from noetix.bumi import visual_odometry as vo


class FakeNode:
    """Minimal rclpy Node surface used by VisualOdometryNode's constructor."""

    def __init__(self, *args, **kwargs):
        self.published = {}
        self.subscriptions = []
        self.timers = []
        self.destroyed = False

    def get_clock(self):
        return NS(now=lambda: NS(nanoseconds=10**12, to_msg=lambda: "stamp"))

    def create_publisher(self, cls, topic, qos):
        messages = self.published.setdefault(topic, [])
        return NS(publish=messages.append)

    def create_subscription(self, cls, topic, cb, qos):
        sub = NS(topic=topic, cb=cb)
        self.subscriptions.append(sub)
        return sub

    def create_timer(self, *args, **kwargs):
        self.timers.append(args)
        return NS()

    def destroy_subscription(self, sub):
        if sub in self.subscriptions:
            self.subscriptions.remove(sub)

    def destroy_publisher(self, pub):
        pass

    def destroy_node(self):
        self.destroyed = True

    def get_logger(self):
        return NS(warning=lambda *_: None, error=lambda *_: None)


class FakeExecutor:
    def __init__(self):
        self.nodes = []

    def add_node(self, node):
        if node not in self.nodes:
            self.nodes.append(node)

    def remove_node(self, node):
        if node in self.nodes:
            self.nodes.remove(node)


@pytest.fixture
def ros(monkeypatch):
    """Install the minimal fake rclpy/std_msgs modules."""
    class _QoS:
        def __init__(self, **kwargs):
            self.__dict__.update(kwargs)
            self.reliability = self.durability = self.history = None

    class _Policies:
        RELIABLE = BEST_EFFORT = TRANSIENT_LOCAL = VOLATILE = KEEP_LAST = "policy"

    class _String:
        def __init__(self, data=""):
            self.data = data

    class _Bool:
        def __init__(self, data=False):
            self.data = data

    for name, attrs in (
        ("rclpy.node", {"Node": FakeNode}),
        ("rclpy.qos", {"qos_profile_sensor_data": object(), "QoSProfile": _QoS,
                       "ReliabilityPolicy": _Policies, "DurabilityPolicy": _Policies,
                       "HistoryPolicy": _Policies}),
        ("std_msgs.msg", {"String": _String, "Bool": _Bool}),
    ):
        mod = types.ModuleType(name)
        mod.__dict__.update(attrs)
        monkeypatch.setitem(sys.modules, name, mod)
    return FakeExecutor()


class FakeTfTree:
    """Stands in for tf_tree_card._TfTreeNode, recording published edges."""

    instances = []

    def __init__(self, namespace, urdf_path, rate, max_age, ttl,
                 node_name=None, status_topic=None):
        self.namespace = namespace
        self._status_topic = status_topic
        self.running = False
        self.parse_error = None
        self.edges = []
        self.started_with = None
        self.destroyed = False
        FakeTfTree.instances.append(self)

    @property
    def status_topic(self):
        return self._status_topic

    def start(self, input_topic):
        self.running = True
        self.started_with = input_topic

    def stop(self):
        self.running = False

    def publish_pose(self, parent, child, position, quaternion, stamp_ns):
        self.edges.append(dict(parent=parent, child=child, position=position,
                               quaternion=quaternion, stamp_ns=stamp_ns))
        return True

    def snapshot(self):
        return dict(state="running" if self.running else "idle", link_count=24)

    def destroy_node(self):
        self.destroyed = True


@pytest.fixture
def fake_tf(monkeypatch):
    """Replace the lazily-imported tf-tree node class with a fake."""
    FakeTfTree.instances = []
    fake_mod = types.ModuleType("tf_tree_card")
    fake_mod._TfTreeNode = FakeTfTree
    monkeypatch.setitem(sys.modules, "tf_tree_card", fake_mod)
    return fake_mod


def _node(config, ros):
    return vo.VisualOdometryNode("robot", config, executor=ros)


def _estimate(stamp_ns=5_000_000_000):
    return dict(stamp_ns=stamp_ns, pose=dict(
        position=[0.1, 0.2, 0.3], quaternion=[0.0, 0.0, 0.0, 1.0],
        frame_id="bumi_visual_deadbeef_odom",
        child_frame_id="bumi_visual_deadbeef_base",
        covariance=[0.0] * 36))


def test_tf_is_disabled_by_default(ros, fake_tf):
    node = _node({}, ros)
    assert node.tf_enabled is False
    node.start_tf()
    assert node.tf_node is None
    assert node.tf_states["state"] == "disabled"
    node.close()
    assert not FakeTfTree.instances


def test_enabling_tf_starts_an_internal_tree_with_no_vision_topic(ros, fake_tf):
    node = _node({"tf": {"enabled": True}}, ros)
    node.start_tf()
    assert node.tf_node is not None
    assert node.tf_node.started_with == ""          # no left-link vision topic
    assert node.tf_node.running is True
    assert node.tf_node.status_topic == f"/robot{vo.TF_STATUS_TOPIC}"
    assert node.tf_node in ros.nodes                # joined to this card's executor


def test_accepted_pose_is_published_with_estimator_stamp(ros, fake_tf):
    node = _node({"tf": {"enabled": True}}, ros)
    node.start_tf()
    node.publish_tf_pose(_estimate(stamp_ns=7_000_000_123))
    edges = node.tf_node.edges
    assert len(edges) == 1
    edge = edges[0]
    assert edge["parent"] == "bumi_visual_deadbeef_odom"
    assert edge["child"] == "bumi_visual_deadbeef_base"
    assert edge["stamp_ns"] == 7_000_000_123        # estimator time, not now()
    assert edge["position"] == [0.1, 0.2, 0.3]


def test_no_tf_edge_when_disabled_or_stopped(ros, fake_tf):
    off = _node({}, ros)
    off.publish_tf_pose(_estimate())
    assert not FakeTfTree.instances

    on = _node({"tf": {"enabled": True}}, ros)
    on.start_tf()
    tf_node = on.tf_node
    on.stop_tf()
    on.publish_tf_pose(_estimate())
    assert tf_node.edges == []


def test_stop_pauses_and_close_destroys_the_tree(ros, fake_tf):
    node = _node({"tf": {"enabled": True}}, ros)
    node.start_tf()
    tf_node = node.tf_node
    # stop() pauses but keeps the node so reset() can reuse it.
    node.stop()
    assert tf_node.running is False
    # close() from a running/stopped node releases it from the executor.
    node = _node({"tf": {"enabled": True}}, ros)
    node.start_tf()
    tf_node = node.tf_node
    node.close()
    assert tf_node.destroyed is True


def test_reset_reuses_the_same_node_without_leaking_executor_entries(ros, fake_tf, monkeypatch):
    """P1 regression: stop/start cycles must not add same-named nodes."""
    # start() would bind camera topics; only the tf lifecycle is under test.
    monkeypatch.setattr(vo.VisualOdometryNode, "bind_inputs", lambda self, ns: None)
    node = _node({"tf": {"enabled": True}}, ros)
    node.start_tf()
    first = node.tf_node
    assert len(ros.nodes) == 1

    for _ in range(3):
        node.stop()
        node.start()          # start() calls start_tf() -> reuse
        assert node.tf_node is first
        assert first.running is True
        assert len(ros.nodes) == 1        # no leak

    node.stop_tf(destroy=True)
    assert first.destroyed is True
    assert ros.nodes == []                # removed from executor on destroy


def test_info_exposes_embedded_tf_health(ros, fake_tf, monkeypatch):
    monkeypatch.setattr(vo.VisualOdometryNode, "publish", lambda self: dict(
        vendor={}, pose=None))
    fake_common_odom = types.ModuleType("common.odom")
    fake_common_odom.build_interface = lambda **k: {}
    monkeypatch.setitem(sys.modules, "common.odom", fake_common_odom)
    node = _node({"tf": {"enabled": True}}, ros)
    node.start_tf()
    info = node.info()
    assert info["tf"]["enabled"] is True
    assert info["tf"]["state"] == "running"
    assert info["tf"]["link_count"] == 24
