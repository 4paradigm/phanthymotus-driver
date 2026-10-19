"""Tianyi velocity path, lifecycle and live settings; no ROS or robot required."""

import ast
import importlib.util
import json
from pathlib import Path
import sys
import types

import pytest

ROOT = Path(__file__).resolve().parents[1]
DRIVER = ROOT / "x-humanoid/tianyi2.0"
sys.path.insert(0, str(ROOT))


class Clock:
    def __init__(self):
        self.now = 50.0

    def time(self):
        return 1000 + self.now

    def monotonic(self):
        return self.now

    def advance(self, seconds=0.1):
        self.now += seconds


class Twist:
    def __init__(self):
        self.linear = types.SimpleNamespace(x=0.0, y=0.0, z=0.0)
        self.angular = types.SimpleNamespace(x=0.0, y=0.0, z=0.0)


class Publisher:
    def __init__(self):
        self.messages = []
        self.fail = False

    def publish(self, msg):
        if self.fail:
            raise RuntimeError("transport failed")
        self.messages.append([msg.linear.x, msg.linear.y, msg.linear.z,
                              msg.angular.x, msg.angular.y, msg.angular.z])


class Node:
    def __init__(self, name, context=None):
        self.name, self.context = name, context
        self.subscriptions, self.publishers, self.timers = [], [], []
        self.destroyed = False

    def create_subscription(self, kind, topic, callback, qos):
        self.subscriptions.append((kind, topic, callback, qos))
        return object()

    def create_publisher(self, kind, topic, qos):
        pub = Publisher()
        self.publishers.append((kind, topic, pub, qos))
        return pub

    def create_timer(self, period, callback):
        self.timers.append((period, callback))
        return object()

    def destroy_node(self):
        self.destroyed = True


class Executor:
    def __init__(self):
        self.nodes = []

    def add_node(self, node):
        self.nodes.append(node)

    def remove_node(self, node):
        if node in self.nodes:
            self.nodes.remove(node)


@pytest.fixture
def setup(monkeypatch):
    modules = {name: types.ModuleType(name) for name in
               ("rclpy", "rclpy.node", "rclpy.qos", "std_msgs", "std_msgs.msg",
                "geometry_msgs", "geometry_msgs.msg")}
    modules["rclpy.node"].Node = Node
    modules["rclpy.qos"].QoSProfile = lambda **kw: types.SimpleNamespace(**kw)
    enums = types.SimpleNamespace(BEST_EFFORT=1, RELIABLE=2, VOLATILE=3)
    modules["rclpy.qos"].ReliabilityPolicy = enums
    modules["rclpy.qos"].DurabilityPolicy = enums
    modules["std_msgs.msg"].String = types.SimpleNamespace
    modules["std_msgs.msg"].Bool = types.SimpleNamespace
    modules["geometry_msgs.msg"].Twist = Twist
    for name, mod in modules.items():
        monkeypatch.setitem(sys.modules, name, mod)
    spec = importlib.util.spec_from_file_location("tianyi_loco_servo_test", DRIVER / "loco_servo.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    clock = Clock()
    module.time = clock
    original_sink = module.ControlSink
    module.ControlSink = lambda *a, **kw: original_sink(
        *a, **kw, clock=lambda: int(clock.now * 1000),
        wall_clock=lambda: int(clock.time() * 1000))
    ros = types.SimpleNamespace(ctx_core=object(), ctx_tianyi=object(),
                                executor_core=Executor(), executor_tianyi=Executor())
    plugins = []

    def create(**config):
        plugin = module.LocoServoPlugin(config, "ubuntu", ros)
        plugins.append(plugin)
        return plugin

    yield types.SimpleNamespace(module=module, clock=clock, ros=ros, create=create)
    for plugin in plugins:
        plugin.stop()


def start(setup, **config):
    plugin = setup.create(**config)
    assert plugin.dispatch("start", {})["state"] == "running"
    setup.clock.advance()
    return plugin


def command(setup, values=None, seq=1, **extra):
    stamp = int(setup.clock.time() * 1000)
    return {"schema": "motus.control/1", "mode": "twist", "dof": 6,
            "values": values or [0.2, 0, 0, 0, 0, 0.4],
            "seq": seq, "source": "navi", "priority": 0,
            "stamp_ms": stamp, "obs_stamp_ms": stamp, "ttl_ms": 200, **extra}


def send(plugin, cmd):
    plugin._on_message(types.SimpleNamespace(data=json.dumps(cmd)))


def test_startup_is_inert_and_domain_routing_is_correct(setup):
    plugin = setup.create()
    plugin.start()
    assert not setup.ros.executor_core.nodes
    assert not setup.ros.executor_tianyi.nodes
    assert plugin.dispatch("info", {})["state"] == "idle"
    assert plugin.dispatch("start", {})["state"] == "running"
    core, body = setup.ros.executor_core.nodes[0], setup.ros.executor_tianyi.nodes[0]
    assert core.context is setup.ros.ctx_core
    assert core.subscriptions[0][1] == "/actucore/navi/cmd"
    assert core.subscriptions[0][3].depth == 1
    assert body.context is setup.ros.ctx_tianyi
    assert body.publishers[0][1] == "/cmd_vel"
    assert not plugin._pub.messages
    assert "LocoServoPlugin" not in (DRIVER / "main.py").read_text().split("_ALWAYS_START =", 1)[1].split("}", 1)[0]


def test_command_maps_body_axes_and_ramps_from_zero(setup):
    plugin = start(setup)
    send(plugin, command(setup))
    output = plugin._pub.messages[-1]
    assert output == pytest.approx([0.03, 0, 0, 0, 0, 0.06])
    setup.clock.advance()
    send(plugin, command(setup, seq=2))
    assert plugin._pub.messages[-1] == pytest.approx([0.06, 0, 0, 0, 0, 0.12])
    # A burst cannot accelerate faster just because callbacks run faster.
    send(plugin, command(setup, seq=3))
    assert plugin._pub.messages[-1] == pytest.approx([0.06, 0, 0, 0, 0, 0.12])
    setup.clock.advance()
    send(plugin, command(setup, [0] * 6, seq=4))
    assert plugin._pub.messages[-1] == [0] * 6


@pytest.mark.parametrize("extra", [
    {"schema": "bad"}, {"mode": "joint_position"}, {"dof": 3},
    {"frame": "world"},
    {"source": ""}, {"seq": True}, {"stamp_ms": 1},
    {"ttl_ms": -1}, {"priority": float("inf")},
    {"obs_stamp_ms": 1}, {"stamp_ms": float("nan")},
    {"stamp_ms": 999999999}, {"obs_stamp_ms": 999999999},
    {"values": [float("nan"), 0, 0, 0, 0, 0]},
    {"values": [0, 0.1, 0, 0, 0, 0]},
    {"values": [0, 0, 0.01, 0, 0, 0]},
    {"values": [0, 0, 0, 0.1, 0, 0]},
    {"values": [0.31, 0, 0, 0, 0, 0]},
    {"values": [True, 0, 0, 0, 0, 0]},
])
def test_invalid_commands_never_reach_body(setup, extra):
    plugin = start(setup)
    send(plugin, command(setup, **extra))
    assert not plugin._pub.messages
    assert plugin._counts["accepted"] == 0


def test_replayed_stale_and_invalid_frames_do_not_feed_watchdog(setup):
    plugin = start(setup)
    original = command(setup)
    send(plugin, original)
    count = len(plugin._pub.messages)
    send(plugin, original)
    assert len(plugin._pub.messages) == count
    setup.clock.advance(0.31)
    send(plugin, command(setup, seq=2, stamp_ms=int(setup.clock.time() * 1000) - 250))
    plugin._on_message(types.SimpleNamespace(data="bad json"))
    plugin._tick()
    assert plugin._pub.messages[-1] == [0] * 6
    assert plugin._counts["accepted"] == 1
    setup.clock.advance()
    send(plugin, command(setup, seq=3))
    assert plugin._pub.messages[-1] == pytest.approx([0.03, 0, 0, 0, 0, 0.06])


def test_dry_run_never_publishes_but_checks_and_counts(setup):
    plugin = start(setup, dry_run=True)
    pub = plugin._pub
    send(plugin, command(setup))
    assert plugin._counts["accepted"] == 1
    assert plugin._counts["dry_run_commands"] == 1
    assert plugin._last["output"][0] > 0
    assert plugin._info()["control_interface"]["dry_run"] is True
    send(plugin, command(setup, seq=2, values=[1, 0, 0, 0, 0, 0]))
    setup.clock.advance(1)
    plugin._tick()
    plugin.dispatch("pause", {})
    plugin.stop()
    assert pub.messages == []


def test_runtime_toggles_stop_old_motion_then_apply_to_fresh_commands(setup):
    plugin = start(setup)
    send(plugin, command(setup))
    result = plugin.dispatch("config", {"dry_run": True})
    assert result["dry_run"] is True
    assert plugin._pub.messages[-1] == [0] * 6
    count = len(plugin._pub.messages)
    setup.clock.advance()
    send(plugin, command(setup, seq=2))
    assert len(plugin._pub.messages) == count
    plugin.dispatch("config", {"dry_run": False, "rotate_only": True})
    setup.clock.advance()
    send(plugin, command(setup, seq=3))
    assert plugin._pub.messages[-1] == pytest.approx([0, 0, 0, 0, 0, 0.06])
    result = plugin.dispatch("config", {"rotate_only": "false", "dry_run": True})
    assert result["state"] == "error"
    assert plugin._settings == {"dry_run": False, "rotate_only": True, "require_standing": False}


def test_pause_resume_and_restart_drop_queued_callbacks_and_preserve_replay_protection(setup):
    plugin = start(setup)
    send(plugin, command(setup))
    queued = command(setup, seq=2)
    plugin.dispatch("pause", {})
    count = len(plugin._pub.messages)
    setup.clock.advance()
    send(plugin, command(setup, seq=3))
    assert len(plugin._pub.messages) == count
    plugin.dispatch("resume", {})
    send(plugin, queued)
    assert plugin._pub.messages[-1] == [0] * 6
    setup.clock.advance()
    send(plugin, command(setup, seq=4))
    assert plugin._pub.messages[-1][0] > 0
    old_node = setup.ros.executor_core.nodes[0]
    old_callback = old_node.subscriptions[0][2]
    plugin.dispatch("stop", {})
    assert old_node.destroyed
    assert not setup.ros.executor_core.nodes
    assert not setup.ros.executor_tianyi.nodes
    plugin.dispatch("start", {})
    setup.clock.advance()
    old_callback(types.SimpleNamespace(data=json.dumps(command(setup, seq=5))))
    assert not plugin._pub.messages
    send(plugin, command(setup, seq=1))
    assert plugin._pub.messages[-1][0] > 0
    assert plugin.dispatch("unsupported", {}) is None


def test_standing_gate_needs_fresh_true_feedback_and_stops_on_loss(setup):
    plugin = start(setup, require_standing=True, standing_topic="/verified/standing")
    send(plugin, command(setup))
    assert plugin._pub.messages[-1] == [0] * 6
    assert plugin._counts["accepted"] == 0
    plugin._on_standing(types.SimpleNamespace(data=False), plugin._generation)
    send(plugin, command(setup, seq=2))
    assert plugin._counts["accepted"] == 0
    plugin._on_standing(types.SimpleNamespace(data=True), plugin._generation)
    send(plugin, command(setup, seq=3))
    assert plugin._counts["accepted"] == 1
    setup.clock.advance(1.1)
    plugin._tick()
    assert plugin._pub.messages[-1] == [0] * 6
    assert "stale" in plugin._info()["posture_problem"]
    plugin._on_standing(types.SimpleNamespace(data=True), plugin._generation)
    setup.clock.advance()
    send(plugin, command(setup, seq=4))
    assert plugin._counts["accepted"] == 2


def test_standing_option_without_source_reports_reason_not_a_fake_r1_fsm(setup):
    plugin = start(setup, require_standing=True)
    send(plugin, command(setup))
    assert plugin._counts["accepted"] == 0
    assert "standing_topic" in plugin._info()["posture_problem"]


def test_publish_error_is_visible_and_pauses_instead_of_counting_success(setup):
    plugin = start(setup)
    plugin._pub.fail = True
    send(plugin, command(setup))
    info = plugin._info()
    assert info["state"] == "paused"
    assert info["counts"]["accepted"] == 0
    assert info["counts"]["publish_errors"] > 0
    plugin._pub.fail = False
    plugin._tick()
    assert plugin._pub.messages[-1] == [0] * 6


def test_partial_open_failure_cleans_nodes(setup, monkeypatch):
    plugin = setup.create()
    def fail(*args):
        raise RuntimeError("publisher unavailable")
    monkeypatch.setattr(Node, "create_publisher", fail)
    assert plugin.dispatch("start", {})["state"] == "error"
    assert not plugin._nodes
    assert not setup.ros.executor_core.nodes
    assert not setup.ros.executor_tianyi.nodes


def test_input_subscription_really_passes_through_bridge_patch(setup, monkeypatch):
    bridged = []
    transport = types.ModuleType("bridged_publisher")
    transport.should_bridge_subscription = lambda topic: True
    transport.should_use_bridge = lambda topic: "/state/" in topic
    transport.create_bridged_publisher = lambda *a: pytest.fail("/cmd_vel must stay on domain 0")
    transport.create_bridged_subscription = lambda *a: bridged.append(a)
    monkeypatch.setitem(sys.modules, "bridged_publisher", transport)
    spec = importlib.util.spec_from_file_location("tianyi_loco_bridge_test", DRIVER / "bridge_integration.py")
    bridge = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(bridge)
    bridge.enable(setup.ros.ctx_core)
    try:
        plugin = start(setup)
        assert len(bridged) == 1
        assert bridged[0][0].context is setup.ros.ctx_core
        assert bridged[0][2] == "/actucore/navi/cmd"
        bridged[0][3](types.SimpleNamespace(data=json.dumps(command(setup))))
        assert plugin._pub.messages[-1][0] > 0
        plugin.stop()
    finally:
        bridge.disable()


def load_bundle():
    from common import lifecycle
    tree = ast.parse((DRIVER / "main.py").read_text())
    node = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == "TianyiDeviceBundle")
    scope = {"DualDomainROS2": object, "_lifecycle": lifecycle}
    exec(compile(ast.Module(body=[node], type_ignores=[]), "main.py", "exec"), scope)
    return scope["TianyiDeviceBundle"]


def test_bundle_hands_control_to_explicit_card_and_requires_cancel_before_resume(setup, monkeypatch):
    monkeypatch.setitem(sys.modules, "loco_servo", setup.module)
    bundle = load_bundle()({"plugins": {"loco_servo": {"enabled": True}}}, "ubuntu", setup.ros, None)
    class Nav:
        def start(self):
            pass
        def get_tool(self):
            return {"name": "nav", "type": "actuator"}
        def dispatch(self, action, args):
            if action == "move_to":
                assert bundle._loco_servo._paused
                assert bundle._loco_servo._pub.messages[-1] == [0] * 6
                return {"state": "navigating"}
            return {"state": "stopped", "api_result": {}}
    bundle._plugins.append(Nav())
    bundle.start_all()
    servo = bundle._loco_servo
    try:
        assert bundle.dispatch("loco_servo", {"action": "start"})["state"] == "running"
        setup.clock.advance()
        send(servo, command(setup))
        bundle.dispatch("nav", {"action": "move_to"})
        assert bundle.dispatch("loco_servo", {"action": "resume"})["state"] == "error"
        bundle.dispatch("nav", {"action": "cancel"})
        assert bundle.dispatch("loco_servo", {"action": "resume"})["state"] == "running"
        assert any(tool["name"] == "loco_servo" for tool in bundle.get_all_tools())
    finally:
        servo.stop()


def test_yielded_stream_writes_nothing_including_zeros_during_other_motion(setup):
    plugin = start(setup)
    send(plugin, command(setup))
    pub = plugin._pub
    plugin.pause_for_explicit_command("nav.move_to")
    assert pub.messages[-1] == [0] * 6
    count = len(pub.messages)
    plugin._tick()
    plugin.dispatch("config", {"rotate_only": True})
    plugin.dispatch("pause", {})
    plugin.pause_for_explicit_command("home.go_home")
    plugin.stop()
    assert len(pub.messages) == count


def test_refuses_start_when_explicit_base_owner_exists(setup):
    plugin = setup.create()
    plugin._conflict = lambda: "nav owns the chassis"
    assert plugin.dispatch("start", {})["state"] == "error"
    assert not setup.ros.executor_core.nodes


def test_zero_bypasses_rate_limit_and_rate_drops_do_not_extend_watchdog(setup):
    plugin = start(setup)
    send(plugin, command(setup))
    setup.clock.advance(0.01)
    send(plugin, command(setup, seq=2))
    assert plugin._last["reason"] == "command rate exceeds 20 Hz"
    assert plugin._counts["accepted"] == 1
    send(plugin, command(setup, [0] * 6, seq=3))
    assert plugin._pub.messages[-1] == [0] * 6


def test_custom_input_and_idempotent_start_keep_one_subscription(setup):
    plugin = setup.create()
    assert plugin.dispatch("start", {"input_topics": ["/custom/cmd"]})["input"] == "/custom/cmd"
    assert plugin.dispatch("start", {"input_topic": "/custom/cmd"})["state"] == "running"
    assert len(setup.ros.executor_core.nodes) == 1
    assert plugin.dispatch("start", {"input_topic": "/different/cmd"})["state"] == "error"


def test_unknown_tool_cannot_steal_base_ownership(setup, monkeypatch):
    monkeypatch.setitem(sys.modules, "loco_servo", setup.module)
    bundle = load_bundle()({"plugins": {"loco_servo": {"enabled": True}}}, "ubuntu", setup.ros, None)
    bundle.start_all()
    try:
        assert bundle.dispatch("nav", {"action": "move_to"}) is None
        assert bundle._base_owner == ""
        assert bundle.dispatch("loco_servo", {"action": "start"})["state"] == "running"
    finally:
        bundle.stop_all()


@pytest.mark.parametrize("config", [{"expected_hz": 21}, {"max_vx": float("nan")},
                                     {"max_wz": -1}, {"dry_run": "false"}])
def test_invalid_config_is_rejected(setup, config):
    with pytest.raises(ValueError):
        setup.create(**config)
