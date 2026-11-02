"""Tianyi velocity path, lifecycle and live settings; no ROS or robot required."""

import ast
import importlib.util
import json
from pathlib import Path
import sys
import threading
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


class CommandLeaseExpired(ValueError):
    pass


class StaleVelocityCommand(CommandLeaseExpired):
    def __init__(self, message, *, zero_velocity_confirmed=False):
        super().__init__(message)
        self.zero_velocity_confirmed = zero_velocity_confirmed


class FakeBackend:
    def __init__(self, config=None):
        self.messages = []
        self.fail = False
        self.connect_fail = False
        self.connected = False
        self.deadlines = []
        self.supports_atomic_expiry_update = (config or {}).get("atomic_expiry_update", False)
        self.expiry_updates = []
        self.supports_staged_stop = (config or {}).get("staged_stop", False)
        self.cancel_deadlines = []
        self.soft_stops = 0
        self.hard_stops = 0

    def connect(self):
        if self.connect_fail:
            raise RuntimeError("connection failed")
        self.connected = True

    def write(self, values, deadline, *, replace_expired=False, cancel_deadline_ms=None):
        if self.fail:
            raise RuntimeError("transport failed")
        self.deadlines.append(deadline)
        self.expiry_updates.append(replace_expired)
        self.cancel_deadlines.append(cancel_deadline_ms)
        self.messages.append(list(values))

    def stop(self):
        if self.fail:
            raise RuntimeError("stop failed")
        if self.connected:
            self.hard_stops += 1
            self.messages.append([0] * 6)

    def soft_stop(self):
        if self.fail:
            raise RuntimeError("soft stop failed")
        self.soft_stops += 1
        self.messages.append([0] * 6)

    def close(self):
        self.connected = False

    def info(self):
        return {"type": "slamware_sdk", "connected": self.connected}


class Node:
    def __init__(self, name, context=None):
        self.name, self.context = name, context
        self.subscriptions, self.publishers, self.timers = [], [], []
        self.destroyed = False
        self.guards = []

    def create_subscription(self, kind, topic, callback, qos):
        self.subscriptions.append((kind, topic, callback, qos))
        return object()

    def create_publisher(self, kind, topic, qos):
        raise AssertionError("loco_servo must not publish to /cmd_vel")

    def create_timer(self, period, callback):
        timer = (period, callback)
        self.timers.append(timer)
        return timer

    def create_guard_condition(self, callback):
        guard = types.SimpleNamespace(callback=callback, triggered=0)
        def trigger():
            if self.destroyed:
                raise RuntimeError("node destroyed")
            guard.triggered += 1
        guard.trigger = trigger
        self.guards.append(guard)
        return guard

    def destroy_timer(self, timer):
        self.timers.remove(timer)

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
    backend_mod = types.ModuleType("slamtec_velocity")
    backend_mod.SlamtecVelocityBackend = FakeBackend
    backend_mod.CommandLeaseExpired = CommandLeaseExpired
    backend_mod.StaleVelocityCommand = StaleVelocityCommand
    monkeypatch.setitem(sys.modules, "slamtec_velocity", backend_mod)
    spec = importlib.util.spec_from_file_location("tianyi_loco_servo_test", DRIVER / "loco_servo.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    clock = Clock()
    module.time = clock
    original_sink = module.ControlSink
    module.ControlSink = lambda *a, **kw: original_sink(
        *a, **kw, clock=lambda: int(clock.now * 1000),
        wall_clock=lambda: int(clock.time() * 1000))
    # Keep fake-clock tests deterministic; timer lifecycle tests explicitly
    # install the production watchdog on a fake ROS node.
    launch_watchdog = module.LocoServoPlugin._start_watchdog
    monkeypatch.setattr(module.LocoServoPlugin, "_start_watchdog", lambda *args: None)
    ros = types.SimpleNamespace(ctx_core=object(), ctx_tianyi=object(),
                                executor_core=Executor(), executor_tianyi=Executor())
    plugins = []

    def create(**config):
        config.setdefault("dry_run", False)
        plugin = module.LocoServoPlugin(config, "ubuntu", ros)
        plugins.append(plugin)
        return plugin

    yield types.SimpleNamespace(module=module, clock=clock, ros=ros, create=create,
                                launch_watchdog=lambda plugin: launch_watchdog(plugin, plugin._generation))
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
    plugin._tick()  # One deterministic control iteration, without a live thread.


def test_startup_is_inert_and_domain_routing_is_correct(setup):
    plugin = setup.create()
    plugin.start()
    assert not setup.ros.executor_core.nodes
    assert not setup.ros.executor_tianyi.nodes
    assert plugin.dispatch("info", {})["state"] == "idle"
    assert plugin.dispatch("start", {})["state"] == "running"
    core = setup.ros.executor_core.nodes[0]
    assert core.context is setup.ros.ctx_core
    assert core.subscriptions[0][1] == "/actucore/navi/cmd"
    assert core.subscriptions[0][3].depth == 1
    assert not setup.ros.executor_tianyi.nodes
    assert plugin._backend.connected
    assert plugin._info()["output_topic"] is None
    assert not plugin._backend.messages
    assert "LocoServoPlugin" not in (DRIVER / "main.py").read_text().split("_ALWAYS_START =", 1)[1].split("}", 1)[0]


@pytest.mark.parametrize("yaw", [-1.0, 1.0])
def test_default_search_yaw_limit_reaches_sdk_with_ramp_and_rejects_excess(setup, yaw):
    plugin = start(setup)
    limits = plugin._info()["control_interface"]["limits"]
    assert limits["lower"] == [-0.3, 0, 0, 0, 0, -1.0]
    assert limits["upper"] == [0.3, 0, 0, 0, 0, 1.0]
    for seq in range(1, 21):
        send(plugin, command(setup, [0, 0, 0, 0, 0, yaw], seq=seq))
        if seq == 1:
            assert plugin._backend.messages[-1][5] == pytest.approx(yaw * 0.06)
        setup.clock.advance()
    assert plugin._counts["accepted"] == 20
    assert plugin._backend.messages[-1] == pytest.approx([0, 0, 0, 0, 0, yaw])
    writes = len(plugin._backend.messages)
    send(plugin, command(setup, [0, 0, 0, 0, 0, yaw * 1.001], seq=21))
    assert plugin._last["verdict"] == "REJECTED"
    assert "axis 5" in plugin._last["reason"]
    assert len(plugin._backend.messages) == writes


def test_explicit_lower_yaw_cap_is_preserved(setup):
    plugin = start(setup, max_wz=0.6)
    send(plugin, command(setup, [0, 0, 0, 0, 0, -1.0]))
    assert plugin._last["verdict"] == "REJECTED"
    assert not plugin._backend.messages


def test_command_maps_body_axes_and_ramps_from_zero(setup):
    plugin = start(setup)
    send(plugin, command(setup))
    output = plugin._backend.messages[-1]
    assert output == pytest.approx([0.03, 0, 0, 0, 0, 0.06])
    setup.clock.advance()
    send(plugin, command(setup, seq=2))
    assert plugin._backend.messages[-1] == pytest.approx([0.06, 0, 0, 0, 0, 0.12])
    # A burst cannot accelerate faster just because callbacks run faster.
    send(plugin, command(setup, seq=3))
    assert plugin._backend.messages[-1] == pytest.approx([0.06, 0, 0, 0, 0, 0.12])
    setup.clock.advance()
    send(plugin, command(setup, [0] * 6, seq=4))
    assert plugin._backend.messages[-1] == [0] * 6


@pytest.mark.parametrize("sdk_delay", [0.0, 0.04, 0.07, 0.09])
def test_regular_10hz_stream_is_not_throttled_by_sdk_reply_latency(setup, monkeypatch, sdk_delay):
    plugin = start(setup)
    backend = plugin._backend
    original_write = backend.write

    def delayed_write(values, deadline):
        setup.clock.advance(sdk_delay)
        original_write(values, deadline)

    monkeypatch.setattr(backend, "write", delayed_write)
    stream_started = setup.clock.monotonic()
    for sequence in range(1, 13):
        # The producer runs at 10Hz even if a prior callback was dropped and
        # therefore did not call the (delayed) SDK at all.
        setup.clock.advance(stream_started + (sequence - 1) * 0.1 - setup.clock.monotonic())
        cmd = command(setup, seq=sequence)
        send(plugin, cmd)
        plugin._tick()

    assert plugin._counts["accepted"] == 12
    assert plugin._counts["dropped"] == 0
    assert plugin._counts["holds"] == 0
    assert plugin._last_output == pytest.approx([0.2, 0, 0, 0, 0, 0.4])
    assert len(backend.messages) == 12


def test_direct_callbacks_serialize_sdk_calls_without_a_dispatch_queue(setup, monkeypatch):
    plugin = start(setup, staged_stop=True)
    entered, release, returned = threading.Event(), threading.Event(), threading.Event()
    original_write = plugin._backend.write
    def blocked_write(*args, **kwargs):
        if not entered.is_set():
            entered.set()
            assert release.wait(2)
        return original_write(*args, **kwargs)
    monkeypatch.setattr(plugin._backend, "write", blocked_write)
    first = threading.Thread(target=lambda: plugin._on_message(
        types.SimpleNamespace(data=json.dumps(command(setup)))))
    def next_callback():
        plugin._on_message(types.SimpleNamespace(data=json.dumps(command(setup,seq=2))))
        returned.set()
    second = threading.Thread(target=next_callback)
    try:
        first.start()
        assert entered.wait(1)
        second.start()
        assert not returned.wait(0.02)
        setup.clock.advance(0.1)
        release.set()
        first.join(1)
        second.join(1)
        assert returned.is_set() and not first.is_alive() and not second.is_alive()
        assert len(plugin._backend.messages) == 2
        assert plugin._counts["accepted"] == 2
    finally:
        release.set()
        first.join(1)
        if second.ident is not None:
            second.join(1)


def test_direct_callback_has_no_local_queue_wait_or_ttl_extension(setup):
    plugin = start(setup)
    original = command(setup, stamp_ms=int(setup.clock.time() * 1000) - 90)
    plugin._on_message(types.SimpleNamespace(data=json.dumps(original)))
    setup.clock.advance(0.03)
    plugin._tick()
    timing = plugin._info()["last_command_timing"]
    assert timing["receive_age_ms"] == 90
    assert timing["local_wait_ms"] == pytest.approx(0)
    assert timing["command_age_ms"] == 90
    assert timing["remaining_before_apply_ms"] == 110
    assert plugin._backend.deadlines[-1] == original["stamp_ms"] + original["ttl_ms"]


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
    assert not plugin._backend.messages
    assert plugin._counts["accepted"] == 0


def test_replayed_stale_and_invalid_frames_do_not_feed_watchdog(setup):
    plugin = start(setup)
    original = command(setup)
    send(plugin, original)
    count = len(plugin._backend.messages)
    send(plugin, original)
    assert len(plugin._backend.messages) == count
    setup.clock.advance(0.31)
    send(plugin, command(setup, seq=2, stamp_ms=int(setup.clock.time() * 1000) - 250))
    plugin._on_message(types.SimpleNamespace(data="bad json"))
    plugin._tick()
    assert plugin._backend.messages[-1] == [0] * 6
    assert plugin._counts["accepted"] == 1
    setup.clock.advance()
    send(plugin, command(setup, seq=3))
    assert plugin._backend.messages[-1] == pytest.approx([0.03, 0, 0, 0, 0, 0.06])


def test_dry_run_never_publishes_but_checks_and_counts(setup):
    plugin = start(setup, dry_run=True)
    pub = plugin._backend
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
    assert plugin._backend.messages[-1] == [0] * 6
    count = len(plugin._backend.messages)
    setup.clock.advance()
    send(plugin, command(setup, seq=2))
    assert len(plugin._backend.messages) == count
    plugin.dispatch("config", {"dry_run": False, "rotate_only": True})
    setup.clock.advance()
    send(plugin, command(setup, seq=3))
    assert plugin._backend.messages[-1] == pytest.approx([0, 0, 0, 0, 0, 0.06])
    result = plugin.dispatch("config", {"rotate_only": "false", "dry_run": True})
    assert result["state"] == "error"
    assert plugin._settings == {"dry_run": False, "rotate_only": True, "require_standing": False}


def test_pause_resume_and_restart_drop_queued_callbacks_and_preserve_replay_protection(setup):
    plugin = start(setup)
    send(plugin, command(setup))
    queued = command(setup, seq=2)
    plugin.dispatch("pause", {})
    count = len(plugin._backend.messages)
    setup.clock.advance()
    send(plugin, command(setup, seq=3))
    assert len(plugin._backend.messages) == count
    plugin.dispatch("resume", {})
    send(plugin, queued)
    assert plugin._backend.messages[-1] == [0] * 6
    setup.clock.advance()
    send(plugin, command(setup, seq=4))
    assert plugin._backend.messages[-1][0] > 0
    old_node = setup.ros.executor_core.nodes[0]
    old_callback = old_node.subscriptions[0][2]
    plugin.dispatch("stop", {})
    assert old_node.destroyed
    assert not setup.ros.executor_core.nodes
    assert not setup.ros.executor_tianyi.nodes
    plugin.dispatch("start", {})
    setup.clock.advance()
    old_callback(types.SimpleNamespace(data=json.dumps(command(setup, seq=5))))
    assert not plugin._backend.messages
    send(plugin, command(setup, seq=1))
    assert plugin._backend.messages[-1][0] > 0
    assert plugin.dispatch("unsupported", {}) is None


def test_standing_gate_needs_fresh_true_feedback_and_stops_on_loss(setup):
    plugin = start(setup, require_standing=True, standing_topic="/verified/standing")
    send(plugin, command(setup))
    assert plugin._backend.messages[-1] == [0] * 6
    assert plugin._counts["accepted"] == 0
    plugin._on_standing(types.SimpleNamespace(data=False), plugin._generation)
    send(plugin, command(setup, seq=2))
    assert plugin._counts["accepted"] == 0
    plugin._on_standing(types.SimpleNamespace(data=True), plugin._generation)
    send(plugin, command(setup, seq=3))
    assert plugin._counts["accepted"] == 1
    setup.clock.advance(1.1)
    plugin._tick()
    assert plugin._backend.messages[-1] == [0] * 6
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
    plugin._backend.fail = True
    send(plugin, command(setup))
    info = plugin._info()
    assert info["state"] == "paused"
    assert info["counts"]["accepted"] == 0
    assert info["counts"]["publish_errors"] > 0
    plugin._backend.fail = False
    plugin._tick()
    assert plugin._backend.messages[-1] == [0] * 6


@pytest.mark.parametrize("atomic", [False, True])
def test_atomic_expiry_update_avoids_the_extra_stop_round_trip(setup, monkeypatch, atomic):
    plugin = start(setup, atomic_expiry_update=atomic)
    send(plugin, command(setup))
    setup.clock.advance(0.21)
    backend = plugin._backend
    original_stop = backend.stop
    def slow_stop():
        setup.clock.advance(0.08)
        original_stop()
    monkeypatch.setattr(backend, "stop", slow_stop)
    fresh = command(setup, seq=2, stamp_ms=int(setup.clock.time() * 1000) - 150)
    send(plugin, fresh)
    if atomic:
        assert plugin._counts["accepted"] == 2
        assert plugin._counts["holds"] == 0
        assert backend.expiry_updates == [False, True]
        assert backend.deadlines[-1] == fresh["stamp_ms"] + fresh["ttl_ms"]
        assert plugin._info()["last_command_timing"]["remaining_after_apply_ms"] == 50
    else:
        assert plugin._counts["accepted"] == 1
        assert plugin._counts["holds"] == 1
        assert backend.expiry_updates == [False]
    assert plugin._info()["dispatch"]["expiry_update"] == (
        "native_atomic" if atomic else "stop_then_set")


def test_atomic_expiry_update_restarts_both_ramps_from_zero(setup):
    plugin = start(setup, atomic_expiry_update=True)
    for seq in range(1, 9):
        send(plugin, command(setup, seq=seq))
        setup.clock.advance(0.1)
    assert plugin._last_output[0] > 0.15 and plugin._last_output[5] > 0.3
    setup.clock.advance(0.21)
    send(plugin, command(setup, seq=9))
    assert plugin._backend.expiry_updates[-1] is True
    assert plugin._last_output == pytest.approx([0.03, 0, 0, 0, 0, 0.06])


@pytest.mark.parametrize("extra", [
    {"schema": "wrong"}, {"seq": 1}, {"obs_stamp_ms": 0},
    {"ttl_ms": 0}, {"values": [0, 1, 0, 0, 0, 0]},
    {"source": "other", "priority": -1},
])
def test_invalid_fresh_frame_cannot_use_atomic_expiry_update(setup, extra):
    plugin = start(setup, atomic_expiry_update=True)
    send(plugin, command(setup))
    setup.clock.advance(0.21)
    send(plugin, command(setup, seq=2) | extra)
    assert plugin._counts["accepted"] == 1
    assert plugin._backend.expiry_updates == [False]
    assert plugin._backend.messages[-1] == [0] * 6
    assert plugin._last_output == [0] * 6


def test_atomic_expiry_update_does_not_replace_zero_pause_or_handover_stops(setup):
    plugin = start(setup, atomic_expiry_update=True)
    send(plugin, command(setup))
    setup.clock.advance(0.21)
    send(plugin, command(setup, [0] * 6, seq=2))
    assert plugin._backend.expiry_updates == [False]
    assert plugin._backend.messages[-1] == [0] * 6
    assert plugin.dispatch("pause", {})["state"] == "paused"
    setup.clock.advance(0.1)
    assert plugin.dispatch("resume", {})["state"] == "running"
    setup.clock.advance(0.1)
    send(plugin, command(setup, seq=3))
    assert plugin.pause_for_explicit_command("nav.move_to") is True
    assert plugin._backend.expiry_updates == [False, False]
    assert plugin._backend.messages[-1] == [0] * 6


def test_failed_atomic_expiry_update_still_pauses_and_requires_confirmed_stop(setup):
    plugin = start(setup, atomic_expiry_update=True)
    send(plugin, command(setup))
    setup.clock.advance(0.21)
    plugin._backend.fail = True
    send(plugin, command(setup, seq=2))
    assert plugin._paused and plugin._zero_pending
    assert plugin._counts["accepted"] == 1
    assert plugin.pause_for_explicit_command("nav.move_to") is False
    plugin._backend.fail = False


def test_native_expiry_stops_without_latching_and_accepts_only_a_fresh_command(setup, monkeypatch):
    plugin = start(setup)
    send(plugin, command(setup))
    backend = plugin._backend
    original_write = backend.write
    setup.clock.advance()
    expired = command(setup, seq=2)
    with monkeypatch.context() as patch:
        def expire(*args):
            raise CommandLeaseExpired("command lease expired at worker: remaining_ms=-12")
        patch.setattr(backend, "write", expire)
        send(plugin, expired)
    info = plugin._info()
    assert info["state"] == "running"
    assert info["counts"]["dropped"] == 1
    assert info["counts"]["accepted"] == 1
    assert info["counts"]["publish_errors"] == 0
    assert info["last"]["verdict"] == "DROPPED"
    assert "remaining_ms=-12" in info["last"]["reason"]
    assert backend.messages[-1] == [0] * 6
    assert info["output"] == [0] * 6
    assert plugin._last_valid_at is None and plugin._lease_expiry_ms == 0
    # Retrying the same sequence cannot replay the rejected motion.
    setup.clock.advance()
    before = len(backend.messages)
    send(plugin, expired)
    assert len(backend.messages) == before
    fresh = command(setup, seq=3, ttl_ms=120)
    send(plugin, fresh)
    assert backend.write == original_write
    assert backend.messages[-1] == pytest.approx([0.03, 0, 0, 0, 0, 0.06])
    assert backend.deadlines[-1] == fresh["stamp_ms"] + 120
    assert plugin._counts["accepted"] == 2


def test_expiry_with_failed_stop_still_pauses_and_refuses_handover(setup, monkeypatch):
    plugin = start(setup)
    def expire(*args):
        raise CommandLeaseExpired("command lease expired")
    monkeypatch.setattr(plugin._backend, "write", expire)
    plugin._backend.fail = True
    send(plugin, command(setup))
    assert plugin._info()["state"] == "paused"
    assert plugin._zero_pending
    assert plugin._counts["publish_errors"] > 0
    assert plugin.pause_for_explicit_command("home.go_home") is False
    assert not plugin._yielded
    plugin._backend.fail = False


def test_partial_open_failure_cleans_nodes(setup, monkeypatch):
    plugin = setup.create()
    def fail(*args):
        raise RuntimeError("publisher unavailable")
    monkeypatch.setattr(FakeBackend, "connect", fail)
    assert plugin.dispatch("start", {})["state"] == "error"
    assert not plugin._nodes
    assert not setup.ros.executor_core.nodes
    assert not setup.ros.executor_tianyi.nodes


def test_input_subscription_really_passes_through_bridge_patch(setup, monkeypatch):
    bridged = []
    transport = types.ModuleType("bridged_publisher")
    transport.should_bridge_subscription = lambda topic: True
    transport.should_use_bridge = lambda topic: "/state/" in topic
    transport.create_bridged_publisher = lambda *a: pytest.fail("SDK backend must not create a ROS publisher")
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
        plugin._tick()
        assert plugin._backend.messages[-1][0] > 0
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
    bundle = load_bundle()({"plugins": {"loco_servo": {"enabled": True, "dry_run": False}}}, "ubuntu", setup.ros, None)
    class Nav:
        def start(self):
            pass
        def get_tool(self):
            return {"name": "nav", "type": "actuator"}
        def dispatch(self, action, args):
            if action == "move_to":
                assert bundle._loco_servo._paused
                assert bundle._loco_servo._backend.messages[-1] == [0] * 6
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
    pub = plugin._backend
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


def test_custom_input_and_idempotent_start_keep_one_subscription(setup):
    plugin = setup.create()
    assert plugin.dispatch("start", {"input_topics": ["/custom/cmd"]})["input"] == "/custom/cmd"
    assert plugin.dispatch("start", {"input_topic": "/custom/cmd"})["state"] == "running"
    assert len(setup.ros.executor_core.nodes) == 1
    assert plugin.dispatch("start", {"input_topic": "/different/cmd"})["state"] == "error"


def test_unknown_tool_cannot_steal_base_ownership(setup, monkeypatch):
    monkeypatch.setitem(sys.modules, "loco_servo", setup.module)
    bundle = load_bundle()({"plugins": {"loco_servo": {"enabled": True, "dry_run": False}}}, "ubuntu", setup.ros, None)
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


def test_new_card_defaults_to_dry_run_without_connecting_to_sdk(setup):
    plugin = setup.module.LocoServoPlugin({}, "ubuntu", setup.ros)
    try:
        assert plugin.dispatch("start", {})["dry_run"] is True
        assert not plugin._backend.connected
        setup.clock.advance()
        send(plugin, command(setup))
        assert plugin._info()["counts"]["dry_run_commands"] == 1
        assert plugin._backend.messages == []
    finally:
        plugin.stop()


def test_disabling_dry_run_requires_successful_readonly_sdk_connect(setup):
    plugin = start(setup, dry_run=True)
    plugin._backend.connect_fail = True
    result = plugin.dispatch("config", {"dry_run": False})
    assert result["state"] == "error"
    assert plugin._info()["dry_run"] is True
    assert not plugin._backend.messages


def test_sdk_lease_preserves_input_ttl_and_observation_deadline(setup):
    plugin = start(setup)
    cmd = command(setup, ttl_ms=150)
    send(plugin, cmd)
    assert plugin._backend.deadlines[-1] == cmd["stamp_ms"] + 150
    setup.clock.advance()
    cmd = command(setup, seq=2, obs_stamp_ms=int(setup.clock.time() * 1000) - 450)
    send(plugin, cmd)
    assert plugin._backend.deadlines[-1] == cmd["obs_stamp_ms"] + 500


def test_unconfirmed_stop_refuses_stop_resume_config_and_handover(setup):
    plugin = start(setup)
    send(plugin, command(setup))
    plugin._backend.fail = True
    assert plugin.pause_for_explicit_command("home.go_home") is False
    assert not plugin._yielded
    assert plugin.dispatch("stop", {})["state"] == "error"
    assert plugin.dispatch("resume", {})["state"] == "error"
    assert plugin.dispatch("config", {"dry_run": True})["state"] == "error"
    assert not plugin._settings["dry_run"]
    plugin._backend.fail = False
    assert plugin.dispatch("stop", {})["state"] == "idle"


def test_bundle_does_not_start_other_motion_when_sdk_stop_fails(setup, monkeypatch):
    monkeypatch.setitem(sys.modules, "loco_servo", setup.module)
    bundle = load_bundle()({"plugins": {"loco_servo": {"enabled": True, "dry_run": False}}}, "ubuntu", setup.ros, None)
    calls = []
    class Home:
        def get_tool(self):
            return {"name": "home", "type": "actuator"}
        def start(self):
            pass
        def dispatch(self, action, args):
            calls.append(action)
            return {"state": "going_home"}
    bundle._plugins.append(Home())
    bundle.start_all()
    bundle.dispatch("loco_servo", {"action": "start"})
    servo = bundle._loco_servo
    setup.clock.advance()
    send(servo, command(setup))
    servo._backend.fail = True
    assert bundle.dispatch("home", {"action": "go_home"})["state"] == "error"
    assert calls == []
    assert bundle._base_owner == ""
    servo._backend.fail = False
    servo.stop()


def test_direct_callback_checks_sink_then_calls_sdk_in_same_thread(setup, monkeypatch):
    plugin = start(setup, staged_stop=True)
    events = []
    submit, write = plugin._sink.submit, plugin._backend.write
    caller = threading.get_ident()
    def checked(cmd):
        events.append(("sink", threading.get_ident()))
        return submit(cmd)
    def applied(*args, **kwargs):
        events.append(("sdk", threading.get_ident()))
        return write(*args, **kwargs)
    monkeypatch.setattr(plugin._sink, "submit", checked)
    monkeypatch.setattr(plugin._backend, "write", applied)
    plugin._on_message(types.SimpleNamespace(data=json.dumps(command(setup))))
    assert events == [("sink", caller), ("sdk", caller)]
    assert plugin._counts["accepted"] == 1
    assert plugin._info()["dispatch"]["mode"] == "direct_callback"
    assert not hasattr(plugin, "_inbox") and not hasattr(plugin, "_controller_thread")


def test_only_ros_watchdog_timer_is_created_and_destroyed(setup, monkeypatch):
    plugin = start(setup, staged_stop=True)
    def no_thread(*args, **kwargs):
        raise AssertionError("servo must not create a dispatch thread")
    monkeypatch.setattr(threading, "Thread", no_thread)
    setup.launch_watchdog(plugin)
    node = plugin._nodes[0][0]
    assert len(node.timers) == 1 and node.timers[0][0] == 0.02
    send(plugin, command(setup))
    setup.clock.advance(0.31)
    node.timers[0][1]()
    assert plugin._backend.soft_stops == 1
    assert plugin.dispatch("stop", {})["state"] == "idle"
    assert not node.timers and node.destroyed


def test_direct_staged_10hz_updates_never_insert_zero_or_cancel(setup):
    plugin = start(setup, staged_stop=True)
    for seq in range(1, 31):
        plugin._on_message(types.SimpleNamespace(data=json.dumps(command(setup, seq=seq))))
        setup.clock.advance(0.1)
        plugin._tick()
    assert plugin._counts["accepted"] == 30
    assert plugin._backend.soft_stops == plugin._backend.hard_stops == 0
    assert all(values[0] > 0 and values[5] > 0 for values in plugin._backend.messages)


@pytest.mark.parametrize("bad", ["ttl", "observation", "sequence", "arbitration", "axis"])
def test_stale_or_rejected_packets_do_not_stop_or_renew_old_motion(setup, bad):
    plugin = start(setup, staged_stop=True)
    send(plugin, command(setup))
    valid_at, cancel = plugin._last_valid_at, plugin._cancel_deadline_ms
    setup.clock.advance(0.12)
    cmd = command(setup, seq=2)
    if bad == "ttl":
        cmd.update(stamp_ms=int(setup.clock.time()*1000)-110, ttl_ms=50)
    elif bad == "observation":
        cmd["obs_stamp_ms"] -= 501
    elif bad == "sequence":
        cmd["seq"] = 1
    elif bad == "arbitration":
        cmd.update(source="other", priority=-1)
    else:
        cmd["values"][1] = 0.1
    send(plugin, cmd)
    assert plugin._backend.soft_stops == plugin._backend.hard_stops == 0
    assert len(plugin._backend.messages) == 1
    assert plugin._last_valid_at == valid_at and plugin._cancel_deadline_ms == cancel
    setup.clock.advance(0.19)
    plugin._tick()
    assert plugin._backend.soft_stops == 1 and plugin._backend.hard_stops == 0
    setup.clock.advance(0.70)
    plugin._tick()
    assert plugin._backend.hard_stops == 1


def test_soft_zero_is_idempotent_and_fresh_command_resumes_without_cancel(setup):
    plugin = start(setup, staged_stop=True)
    send(plugin, command(setup))
    cancel = plugin._cancel_deadline_ms
    setup.clock.advance(0.31)
    for _ in range(10):
        plugin._tick()
    assert plugin._backend.soft_stops == 1
    assert plugin._backend.hard_stops == 0 and plugin._cancel_deadline_ms == cancel
    setup.clock.advance(0.2)
    send(plugin, command(setup, seq=2))
    assert plugin._backend.hard_stops == 0
    assert plugin._backend.messages[-1] == pytest.approx([0.03,0,0,0,0,0.06])
    assert plugin._cancel_deadline_ms > cancel


def test_descriptor_and_execution_use_same_500ms_observation_age(setup):
    plugin = start(setup, staged_stop=True)
    assert plugin._descriptor["rate"] == {
        "max_hz":20,"expected_hz":10.0,"watchdog_ms":300,
        "max_obs_age_ms":500,"cancel_after_ms":1000}
    obs = int(setup.clock.time()*1000)
    for seq in range(1, 5):
        send(plugin, command(setup, seq=seq, obs_stamp_ms=obs))
        setup.clock.advance(0.1)
    # Fresh packet based on the same old observation cannot prolong motion.
    send(plugin, command(setup, seq=5, obs_stamp_ms=obs))
    assert plugin._backend.deadlines[-1] == obs+500
    setup.clock.advance(0.11)
    plugin._tick()
    assert plugin._backend.soft_stops == 1 and plugin._backend.hard_stops == 0
    cancel = plugin._cancel_deadline_ms
    send(plugin, command(setup, seq=6, obs_stamp_ms=obs))
    assert plugin._counts["accepted"] == 5 and plugin._cancel_deadline_ms == cancel


def test_callback_wait_and_sdk_reply_do_not_restart_original_receive_budget(setup, monkeypatch):
    plugin = start(setup, staged_stop=True)
    original = plugin._process_message
    received = setup.clock.now
    def delayed(*args):
        setup.clock.advance(0.18)
        return original(*args)
    monkeypatch.setattr(plugin, "_process_message", delayed)
    plugin._on_message(types.SimpleNamespace(data=json.dumps(command(setup))))
    assert plugin._last_valid_at == received
    assert plugin._backend.deadlines[-1] == int((1000+received)*1000)+300
    assert plugin._backend.cancel_deadlines[-1] == int((1000+received)*1000)+1000
    setup.clock.advance(0.13)
    plugin._tick()
    assert plugin._backend.soft_stops == 1


@pytest.mark.parametrize("action", ["zero","pause","stop","yield"])
def test_explicit_stop_actions_do_not_wait_for_staged_timeouts(setup, action):
    plugin = start(setup, staged_stop=True)
    send(plugin, command(setup))
    backend = plugin._backend
    if action == "zero":
        # Zero bypasses the nonzero 20Hz pacing gate.
        plugin._on_message(types.SimpleNamespace(data=json.dumps(command(setup,[0]*6,seq=2))))
    elif action == "yield":
        assert plugin.pause_for_explicit_command("home")
    else:
        plugin.dispatch(action,{})
    assert backend.hard_stops >= 1 and backend.soft_stops == 0
    assert plugin._last_output == [0]*6


@pytest.mark.parametrize("confirmed_zero", [False, True])
def test_native_stale_reply_never_renews_accepted_cancel_deadline(setup, monkeypatch, confirmed_zero):
    plugin = start(setup, staged_stop=True)
    send(plugin, command(setup))
    old_output, cancel, valid_at = list(plugin._last_output), plugin._cancel_deadline_ms, plugin._last_valid_at
    setup.clock.advance(0.1)
    with monkeypatch.context() as patch:
        def stale(*args, **kwargs):
            raise StaleVelocityCommand("stale SDK packet", zero_velocity_confirmed=confirmed_zero)
        patch.setattr(plugin._backend,"write",stale)
        send(plugin, command(setup,seq=2))
    assert plugin._backend.hard_stops == plugin._backend.soft_stops == 0
    assert plugin._last_valid_at == valid_at and plugin._cancel_deadline_ms == cancel
    assert plugin._last_output == ([0]*6 if confirmed_zero else old_output)
    setup.clock.advance(0.1)
    send(plugin, command(setup,seq=3))
    expected = 0.03 if confirmed_zero else 0.06
    assert plugin._last_output[0] == pytest.approx(expected)


def test_soft_zero_failure_pauses_and_uses_confirmed_hard_stop(setup, monkeypatch):
    plugin = start(setup, staged_stop=True)
    send(plugin, command(setup))
    def fail():
        raise RuntimeError("soft zero not acknowledged")
    monkeypatch.setattr(plugin._backend,"soft_stop",fail)
    setup.clock.advance(0.31)
    plugin._tick()
    assert plugin._paused and plugin._backend.hard_stops == 1
    assert not plugin._zero_pending
