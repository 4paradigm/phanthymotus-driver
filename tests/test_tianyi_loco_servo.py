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
        self.zero_velocity_confirmed = zero_velocity_confirmed is True


class FakeBackend:
    def __init__(self, config=None):
        self.messages = []
        self.fail = False
        self.connect_fail = False
        self.connected = False
        self.deadlines = []
        self.supports_atomic_expiry_update = (config or {}).get("atomic_expiry_update", False)
        self.supports_staged_stop = (config or {}).get("staged_stop", True)
        self.expiry_updates = []
        self.preserved_actions = []
        self.stop_calls = 0
        self.cancel_deadlines = []
        self.soft_stop_calls = []

    def connect(self):
        if self.connect_fail:
            raise RuntimeError("connection failed")
        self.connected = True

    def write(self, values, deadline, *, replace_expired=False, preserve_action=False,
              cancel_deadline_ms=None):
        if self.fail:
            raise RuntimeError("transport failed")
        self.deadlines.append(deadline)
        self.expiry_updates.append(replace_expired)
        self.preserved_actions.append(preserve_action)
        self.cancel_deadlines.append(cancel_deadline_ms)
        self.messages.append(list(values))

    def soft_stop(self, cancel_deadline_ms=None):
        if self.fail:
            raise RuntimeError("soft stop failed")
        self.soft_stop_calls.append(cancel_deadline_ms)
        if not self.supports_staged_stop:
            return self.stop()
        self.messages.append([0] * 6)

    def stop(self):
        self.stop_calls += 1
        if self.fail:
            raise RuntimeError("stop failed")
        if self.connected:
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
        self.timers.append((period, callback))
        return object()

    def create_guard_condition(self, callback):
        guard = types.SimpleNamespace(callback=callback, triggered=0)
        def trigger():
            if self.destroyed:
                raise RuntimeError("node destroyed")
            guard.triggered += 1
        guard.trigger = trigger
        self.guards.append(guard)
        return guard

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
    # Keep fake-clock unit tests deterministic; concurrency tests below launch
    # the real production worker explicitly and never spin a ROS executor.
    launch_controller = module.LocoServoPlugin._start_controller
    monkeypatch.setattr(module.LocoServoPlugin, "_start_controller", lambda *args: None)
    ros = types.SimpleNamespace(ctx_core=object(), ctx_tianyi=object(),
                                executor_core=Executor(), executor_tianyi=Executor())
    plugins = []

    def create(**config):
        config.setdefault("dry_run", False)
        plugin = module.LocoServoPlugin(config, "ubuntu", ros)
        plugins.append(plugin)
        return plugin

    yield types.SimpleNamespace(module=module, clock=clock, ros=ros, create=create,
                                launch_controller=lambda plugin: launch_controller(plugin, plugin._generation))
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


def send_received_earlier(setup, plugin, cmd, seconds):
    """Deliver cmd as if its callback ran `seconds` ago (shorter receipt lease)."""
    received = (setup.clock.monotonic() - seconds, int(setup.clock.time() * 1000 - seconds * 1000))
    plugin._on_message(types.SimpleNamespace(data=json.dumps(cmd)), None, received)
    plugin._tick()


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
            assert plugin._backend.messages[-1][5] == pytest.approx(yaw * 0.2)
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
    assert output == pytest.approx([0.03, 0, 0, 0, 0, 0.2])
    setup.clock.advance()
    send(plugin, command(setup, seq=2))
    assert plugin._backend.messages[-1] == pytest.approx([0.06, 0, 0, 0, 0, 0.4])
    # A burst cannot accelerate faster just because callbacks run faster.
    send(plugin, command(setup, seq=3))
    assert plugin._backend.messages[-1] == pytest.approx([0.06, 0, 0, 0, 0, 0.4])
    setup.clock.advance()
    send(plugin, command(setup, [0] * 6, seq=4))
    assert plugin._backend.messages[-1] == [0] * 6


@pytest.mark.parametrize("sdk_delay", [0.0, 0.04, 0.07, 0.09])
def test_regular_10hz_stream_is_not_throttled_by_sdk_reply_latency(setup, monkeypatch, sdk_delay):
    plugin = start(setup)
    backend = plugin._backend
    original_write = backend.write

    def delayed_write(values, deadline, **options):
        setup.clock.advance(sdk_delay)
        original_write(values, deadline, **options)

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


def test_sdk_latency_does_not_allow_apply_starts_above_20hz(setup, monkeypatch):
    plugin = start(setup)
    original_write = plugin._backend.write

    def delayed_write(values, deadline, **options):
        setup.clock.advance(0.03)
        original_write(values, deadline, **options)

    monkeypatch.setattr(plugin._backend, "write", delayed_write)
    send(plugin, command(setup))
    setup.clock.advance(0.005)  # 35ms between starts: still above the 20Hz cap.
    send(plugin, command(setup, seq=2))
    assert plugin._counts["accepted"] == 1
    assert plugin._counts["dropped"] == 0
    assert plugin._info()["input_queue"]["pending"] == 1
    send(plugin, command(setup, [0] * 6, seq=3))
    assert plugin._backend.messages[-1] == [0] * 6


def test_worker_dispatches_continuous_stream_without_spinning_ros(setup, monkeypatch):
    plugin = start(setup)
    node = setup.ros.executor_core.nodes[0]
    assert not node.guards and not node.timers
    applied = threading.Event()
    original_write = plugin._backend.write
    def slow_sdk(values, deadline, **options):
        setup.clock.advance(0.07)
        original_write(values, deadline, **options)
        applied.set()
    monkeypatch.setattr(plugin._backend, "write", slow_sdk)
    setup.launch_controller(plugin)
    began = setup.clock.monotonic()
    for seq in range(1, 21):
        setup.clock.advance(began + (seq - 1) * 0.1 - setup.clock.monotonic())
        applied.clear()
        node.subscriptions[0][2](types.SimpleNamespace(data=json.dumps(command(setup, seq=seq))))
        assert applied.wait(1), "controller waited for a ROS executor callback"
        assert plugin.dispatch("info", {})["counts"]["accepted"] == seq
    assert plugin._counts["accepted"] == 20
    assert plugin._counts["holds"] == plugin._counts["dropped"] == 0
    assert len(plugin._backend.messages) == 20
    timing = plugin._info()["last_command_timing"]
    assert timing["command_age_ms"] == timing["observation_age_ms"] == 0
    assert timing["apply_round_trip_ms"] == pytest.approx(70)
    assert 229 <= timing["remaining_after_apply_ms"] <= 231


def test_worker_receives_latest_while_sdk_waits_and_keeps_receipt_deadline(setup, monkeypatch):
    plugin = start(setup)
    backend = plugin._backend
    entered, release, latest_applied = threading.Event(), threading.Event(), threading.Event()
    original_write = backend.write
    def blocked_first(values, deadline, **options):
        if not backend.messages:
            entered.set()
            assert release.wait(2)
        original_write(values, deadline, **options)
        if len(backend.messages) == 2:
            latest_applied.set()
    monkeypatch.setattr(backend, "write", blocked_first)
    setup.launch_controller(plugin)
    try:
        plugin._on_message(types.SimpleNamespace(data=json.dumps(command(setup))))
        assert entered.wait(1)
        for seq in range(2, 42):
            setup.clock.advance(0.001)
            newest = command(setup, [0.15, 0, 0, 0, 0, -0.2], seq=seq)
            plugin._on_message(types.SimpleNamespace(data=json.dumps(newest)))
        assert plugin._inbox.info()["pending"] == 1
        assert plugin._inbox.info()["coalesced"] == 39
        setup.clock.advance(0.03)
        release.set()
        assert latest_applied.wait(1)
        info = plugin.dispatch("info", {})
        assert info["counts"]["accepted"] == 2
        assert info["last_command_timing"]["seq"] == 41
        assert backend.deadlines[-1] == pytest.approx(newest["stamp_ms"] + 300, abs=1)
        assert info["counts"]["holds"] == 0
    finally:
        release.set()


def test_worker_observation_deadline_stops_without_ros_callbacks_and_never_replays(setup, monkeypatch):
    plugin = start(setup)
    backend = plugin._backend
    applied, stopped = threading.Event(), threading.Event()
    original_write, original_stop = backend.write, backend.stop
    def write(values, deadline, **options):
        original_write(values, deadline, **options)
        applied.set()
    def stop():
        original_stop()
        stopped.set()
    monkeypatch.setattr(backend, "write", write)
    monkeypatch.setattr(backend, "stop", stop)
    setup.launch_controller(plugin)
    cmd = command(setup)
    plugin._on_message(types.SimpleNamespace(data=json.dumps(cmd)))
    assert applied.wait(1)
    plugin.dispatch("info", {})  # Wait until the accepted-state update completes.
    setup.clock.advance(1.01)
    plugin._controller_wake.set()
    assert stopped.wait(1)
    info = plugin.dispatch("info", {})
    assert info["output"] == [0] * 6
    assert info["last"]["verdict"] == "HOLD"
    assert len(backend.deadlines) == 1
    assert backend.deadlines[0] == pytest.approx(cmd["stamp_ms"] + 300, abs=1)


def test_worker_zero_command_bypasses_motion_pacing(setup, monkeypatch):
    plugin = start(setup)
    backend = plugin._backend
    applied, stopped = threading.Event(), threading.Event()
    original_write, original_stop = backend.write, backend.stop
    def write(values, deadline, **options):
        original_write(values, deadline, **options)
        applied.set()
    def stop():
        original_stop()
        stopped.set()
    monkeypatch.setattr(backend, "write", write)
    monkeypatch.setattr(backend, "stop", stop)
    setup.launch_controller(plugin)
    plugin._on_message(types.SimpleNamespace(data=json.dumps(command(setup))))
    assert applied.wait(1)
    plugin.dispatch("info", {})
    setup.clock.advance(0.01)
    plugin._on_message(types.SimpleNamespace(data=json.dumps(command(setup, [0] * 6, seq=2))))
    assert stopped.wait(1)
    info = plugin.dispatch("info", {})
    assert info["counts"]["accepted"] == 2
    assert info["output"] == [0] * 6


def test_stop_joins_outside_state_lock_and_blocks_restart_until_old_worker_exits(setup, monkeypatch):
    plugin = start(setup)
    entered, release, stopping = threading.Event(), threading.Event(), threading.Event()
    original_tick, original_stop = plugin._tick, plugin._backend.stop
    def blocked_tick(generation=None):
        entered.set()
        assert release.wait(2)
        original_tick(generation)
    def confirmed_stop():
        original_stop()
        stopping.set()
    monkeypatch.setattr(plugin, "_tick", blocked_tick)
    monkeypatch.setattr(plugin._backend, "stop", confirmed_stop)
    setup.launch_controller(plugin)
    old_worker = plugin._controller_thread
    result = {}
    stopper = threading.Thread(target=lambda: result.update(plugin.dispatch("stop", {})))
    try:
        assert entered.wait(1)
        stopper.start()
        assert stopping.wait(1)
        assert plugin.dispatch("info", {})["state"] == "stopping"
        assert plugin.dispatch("start", {})["state"] == "error"
        release.set()
        stopper.join(2)
        assert not stopper.is_alive()
        assert result["state"] == "idle", "joining under _lock deadlocks the old worker"
        assert not old_worker.is_alive()
        assert plugin._controller_thread is None
        assert not plugin._nodes
    finally:
        release.set()
        if stopper.ident is not None:
            stopper.join(2)


def test_paused_worker_resumes_only_on_fresh_input_and_restarts_without_thread_leaks(setup, monkeypatch):
    plugin = start(setup)
    setup.launch_controller(plugin)
    old_callback = setup.ros.executor_core.nodes[0].subscriptions[0][2]
    old_worker = plugin._controller_thread
    assert plugin.dispatch("pause", {})["state"] == "paused"
    old_callback(types.SimpleNamespace(data=json.dumps(command(setup))))
    assert plugin._inbox.info()["pending"] == 0
    setup.clock.advance()
    assert plugin.dispatch("resume", {})["state"] == "running"
    setup.clock.advance()
    applied = threading.Event()
    original_write = plugin._backend.write
    def write(values, deadline, **options):
        original_write(values, deadline, **options)
        applied.set()
    monkeypatch.setattr(plugin._backend, "write", write)
    old_callback(types.SimpleNamespace(data=json.dumps(command(setup, seq=2))))
    assert applied.wait(1)
    assert plugin.dispatch("stop", {})["state"] == "idle"
    assert not old_worker.is_alive()
    for _ in range(3):
        assert plugin.dispatch("start", {})["state"] == "running"
        setup.launch_controller(plugin)
        current = plugin._controller_thread
        old_callback(types.SimpleNamespace(data=json.dumps(command(setup, seq=3))))
        assert plugin._inbox.info()["pending"] == 0
        assert plugin.dispatch("stop", {})["state"] == "idle"
        assert not current.is_alive()


def test_join_timeout_preserves_resources_and_blocks_new_worker(setup, monkeypatch):
    plugin = start(setup)
    entered, release = threading.Event(), threading.Event()
    original_tick = plugin._tick
    def blocked_tick(generation=None):
        entered.set()
        assert release.wait(3)
        original_tick(generation)
    monkeypatch.setattr(plugin, "_tick", blocked_tick)
    setup.launch_controller(plugin)
    worker, backend = plugin._controller_thread, plugin._backend
    try:
        assert entered.wait(1)
        assert plugin.dispatch("stop", {})["state"] == "error"
        assert plugin.dispatch("info", {})["state"] == "stopping"
        assert plugin.dispatch("start", {})["state"] == "error"
        assert plugin._backend is backend and backend.connected
        assert plugin._controller_thread is worker and worker.is_alive()
        release.set()
        worker.join(1)
        assert plugin.dispatch("stop", {})["state"] == "idle"
        assert plugin._backend is None
    finally:
        release.set()
        worker.join(1)


def test_unexpected_worker_failure_stops_motion_and_requires_explicit_restart(setup, monkeypatch):
    plugin = start(setup)
    stopped = threading.Event()
    original_stop = plugin._backend.stop
    def stop():
        original_stop()
        stopped.set()
    def broken_tick(generation=None):
        raise RuntimeError("injected control-loop failure")
    monkeypatch.setattr(plugin._backend, "stop", stop)
    monkeypatch.setattr(plugin, "_tick", broken_tick)
    setup.launch_controller(plugin)
    assert stopped.wait(1)
    plugin._controller_thread.join(1)
    info = plugin.dispatch("info", {})
    assert info["state"] == "paused"
    assert info["last"]["verdict"] == "ERROR"
    assert "control-loop failure" in info["last"]["reason"]
    assert not info["dispatch"]["worker_alive"]
    assert plugin.dispatch("resume", {})["state"] == "error"


def test_resume_refuses_failed_worker_even_before_thread_has_exited(setup, monkeypatch):
    plugin = start(setup)
    failed, release = threading.Event(), threading.Event()
    original_loop = plugin._control_loop
    def broken_tick(generation=None):
        raise RuntimeError("injected failure")
    def delayed_exit(*args):
        original_loop(*args)
        failed.set()
        assert release.wait(2)
    monkeypatch.setattr(plugin, "_tick", broken_tick)
    monkeypatch.setattr(plugin, "_control_loop", delayed_exit)
    setup.launch_controller(plugin)
    try:
        assert failed.wait(1)
        assert plugin._controller_thread.is_alive()
        assert plugin.dispatch("info", {})["dispatch"]["worker_failed"]
        assert plugin.dispatch("resume", {})["state"] == "error"
    finally:
        release.set()
        plugin._controller_thread.join(1)


def test_worker_start_failure_cleans_ros_and_sdk_resources(setup, monkeypatch):
    plugin = setup.create()
    def cannot_start(generation):
        raise RuntimeError("thread creation denied")
    monkeypatch.setattr(plugin, "_start_controller", cannot_start)
    result = plugin.dispatch("start", {})
    assert result["state"] == "error"
    assert "thread creation denied" in result["error"]
    assert not plugin._running and not plugin._nodes
    assert plugin._backend is None and plugin._controller_thread is None
    assert not plugin._inbox.info()["active"]


def test_bunched_arrivals_keep_latest_until_next_apply_slot(setup):
    plugin = start(setup)
    send(plugin, command(setup))
    setup.clock.advance(0.02)
    send(plugin, command(setup, [0.1, 0, 0, 0, 0, 0.2], seq=2))
    setup.clock.advance(0.01)
    latest = command(setup, [0.15, 0, 0, 0, 0, -0.2], seq=3)
    send(plugin, latest)
    assert plugin._counts["accepted"] == 1
    assert plugin._info()["input_queue"]["pending"] == 1
    setup.clock.advance(0.03)
    plugin._tick()
    assert plugin._counts["accepted"] == 2
    assert plugin._counts["dropped"] == 0
    assert plugin._last["requested"] == latest["values"]
    assert plugin._backend.deadlines[-1] == pytest.approx(latest["stamp_ms"] + 300, abs=1)
    assert plugin._counts["holds"] == 0


def test_inbound_callback_remains_responsive_while_sdk_is_waiting(setup, monkeypatch):
    plugin = start(setup)
    entered, release, received = threading.Event(), threading.Event(), threading.Event()
    original_write = plugin._backend.write

    def blocked_write(values, deadline, **options):
        entered.set()
        assert release.wait(2), "test did not release fake SDK"
        original_write(values, deadline, **options)

    monkeypatch.setattr(plugin._backend, "write", blocked_write)
    first = threading.Thread(target=lambda: plugin._on_message(
        types.SimpleNamespace(data=json.dumps(command(setup)))))
    controller = threading.Thread(target=plugin._tick)
    def receive_next():
        plugin._on_message(types.SimpleNamespace(data=json.dumps(command(setup, seq=2))))
        received.set()
    next_frame = threading.Thread(target=receive_next)
    try:
        first.start()
        first.join(0.05)
        controller.start()
        assert entered.wait(1)
        next_frame.start()
        assert received.wait(0.2), "SDK wait blocked the bridge reader from receiving the next frame"
    finally:
        release.set()
        for thread in (first, controller, next_frame):
            if thread.ident is not None:
                thread.join(2)
                assert not thread.is_alive()


def test_control_timer_takes_only_latest_motion_with_original_receipt_deadline(setup):
    plugin = start(setup)
    for sequence in range(1, 26):
        setup.clock.advance(0.01)
        latest = command(setup, [sequence / 100, 0, 0, 0, 0, 0.1], seq=sequence)
        plugin._on_message(types.SimpleNamespace(data=json.dumps(latest)))
    assert not plugin._backend.messages
    before = plugin._info()["input_queue"]
    assert before["received"] == 25 and before["coalesced"] == 24
    assert before["pending"] == 1
    plugin._tick()
    assert plugin._counts["received"] == plugin._counts["accepted"] == 1
    assert plugin._last["requested"][0] == 0.25
    assert plugin._backend.deadlines == pytest.approx([latest["stamp_ms"] + 300], abs=1)
    assert plugin._info()["input_queue"]["pending"] == 0
    assert plugin._info()["input_queue"]["processed"] == 1


def test_queue_wait_does_not_retimestamp_or_execute_expired_input(setup):
    plugin = start(setup)
    plugin._on_message(types.SimpleNamespace(data=json.dumps(command(setup))))
    setup.clock.advance(0.21)
    plugin._tick()
    assert not plugin._backend.messages
    assert plugin._counts["accepted"] == 0
    assert plugin._counts["dropped"] == 1
    assert plugin._info()["input_queue"]["last_wait_ms"] == pytest.approx(210)


def test_timing_separates_input_ttl_age_from_receipt_watchdog_budget(setup):
    plugin = start(setup)
    original = command(setup, stamp_ms=int(setup.clock.time() * 1000) - 90)
    plugin._on_message(types.SimpleNamespace(data=json.dumps(original)))
    setup.clock.advance(0.03)
    plugin._tick()
    timing = plugin._info()["last_command_timing"]
    assert timing["receive_age_ms"] == 90
    assert timing["local_wait_ms"] == pytest.approx(30)
    assert timing["command_age_ms"] == 120
    assert timing["remaining_before_apply_ms"] == pytest.approx(270, abs=1)
    assert timing["execution_deadline_basis"] == "receive_watchdog"
    assert timing["receive_watchdog_ms"] == 300
    assert plugin._backend.deadlines[-1] == pytest.approx(original["stamp_ms"] + 90 + 300, abs=1)


def test_zero_is_prioritized_and_cannot_be_overwritten_by_a_later_motion(setup):
    plugin = start(setup)
    for cmd in (command(setup), command(setup, [0] * 6, seq=2), command(setup, seq=3)):
        plugin._on_message(types.SimpleNamespace(data=json.dumps(cmd)))
    assert plugin._info()["input_queue"]["pending"] == 2
    assert plugin._info()["input_queue"]["coalesced"] == 1
    plugin._tick()
    assert plugin._backend.messages == [[0] * 6]
    assert plugin._last["requested"] == [0] * 6
    setup.clock.advance()
    plugin._tick()
    assert plugin._backend.messages[-1] == pytest.approx([0.03, 0, 0, 0, 0, 0.2])
    assert plugin._info()["input_queue"]["processed"] == 2


def test_pause_config_and_restart_discard_pending_input_and_old_generation(setup):
    plugin = start(setup)
    old_node = setup.ros.executor_core.nodes[0]
    old_callback = old_node.subscriptions[0][2]
    old_generation = plugin._generation
    old_tick = lambda: plugin._tick(old_generation)
    plugin._on_message(types.SimpleNamespace(data=json.dumps(command(setup))))
    plugin.dispatch("pause", {})
    assert plugin._info()["input_queue"]["pending"] == 0
    assert not plugin._info()["input_queue"]["active"]
    plugin._on_message(types.SimpleNamespace(data=json.dumps(command(setup, seq=2))))
    assert plugin._info()["input_queue"]["pending"] == 0
    setup.clock.advance()
    plugin.dispatch("resume", {})
    setup.clock.advance()
    plugin._on_message(types.SimpleNamespace(data=json.dumps(command(setup, seq=3))))
    plugin.dispatch("config", {"rotate_only": True})
    assert plugin._info()["input_queue"]["pending"] == 0
    assert plugin._info()["input_queue"]["active"]
    plugin.stop()
    plugin.dispatch("start", {})
    setup.clock.advance()
    old_callback(types.SimpleNamespace(data=json.dumps(command(setup, seq=4))))
    assert plugin._info()["input_queue"]["pending"] == 0
    plugin._on_message(types.SimpleNamespace(data=json.dumps(command(setup))))
    old_tick()
    assert not plugin._backend.messages
    assert plugin._info()["input_queue"]["pending"] == 1
    plugin._tick()
    assert plugin._backend.messages[-1][5] > 0


def test_timer_does_not_replay_last_motion_or_refresh_receipt_watchdog_without_new_input(setup):
    plugin = start(setup)
    send(plugin, command(setup))
    before = list(plugin._backend.messages)
    setup.clock.advance(0.1)
    plugin._tick()
    assert plugin._backend.messages == before
    setup.clock.advance(0.91)
    plugin._tick()
    assert plugin._backend.messages[-1] == [0] * 6
    assert len(plugin._backend.deadlines) == 1
    assert plugin._info()["input_queue"]["processed"] == 1


def test_pending_motion_cannot_run_while_a_stop_is_unconfirmed(setup, monkeypatch):
    plugin = start(setup)
    plugin._zero_pending = True
    plugin._backend.fail = True
    monkeypatch.setattr(plugin._backend, "write", lambda *args: pytest.fail("motion before confirmed stop"))
    plugin._on_message(types.SimpleNamespace(data=json.dumps(command(setup))))
    plugin._tick()
    assert plugin._info()["state"] == "paused"
    assert plugin._info()["input_queue"]["pending"] == 0
    assert not plugin._info()["input_queue"]["active"]
    plugin._backend.fail = False


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


def test_replayed_stale_and_invalid_frames_do_not_extend_execution_deadline(setup):
    plugin = start(setup)
    original = command(setup)
    send(plugin, original)
    count = len(plugin._backend.messages)
    send(plugin, original)
    assert len(plugin._backend.messages) == count
    setup.clock.advance(1.01)
    send(plugin, command(setup, seq=2, stamp_ms=int(setup.clock.time() * 1000) - 250))
    plugin._on_message(types.SimpleNamespace(data="bad json"))
    plugin._tick()
    assert plugin._backend.messages[-1] == [0] * 6
    assert plugin._counts["accepted"] == 1
    setup.clock.advance()
    send(plugin, command(setup, seq=3))
    assert plugin._backend.messages[-1] == pytest.approx([0.03, 0, 0, 0, 0, 0.2])


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
    assert plugin._backend.messages[-1] == pytest.approx([0, 0, 0, 0, 0, 0.2])
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
    plugin = start(setup, staged_stop=False, atomic_expiry_update=atomic)
    send(plugin, command(setup))
    setup.clock.advance(1.01)
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
        assert backend.deadlines[-1] == pytest.approx(fresh["stamp_ms"] + 150 + 300, abs=1)
        assert plugin._info()["last_command_timing"]["remaining_after_apply_ms"] == pytest.approx(300, abs=1)
    else:
        # This frame passed admission before the old worker's STOP. The
        # receive/observation deadline, rather than TTL, bounds execution.
        assert plugin._counts["accepted"] == 2
        assert plugin._counts["holds"] >= 1
        assert any(backend.messages[-1])
        assert backend.expiry_updates == [False, False]
    assert plugin._info()["dispatch"]["expiry_update"] == (
        "native_atomic" if atomic else "stop_then_set")


def test_atomic_expiry_update_restarts_both_ramps_from_zero(setup):
    plugin = start(setup, staged_stop=False, atomic_expiry_update=True)
    for seq in range(1, 9):
        send(plugin, command(setup, seq=seq))
        setup.clock.advance(0.1)
    assert plugin._last_output[0] > 0.15 and plugin._last_output[5] > 0.3
    setup.clock.advance(1.01)
    send(plugin, command(setup, seq=9))
    assert plugin._backend.expiry_updates[-1] is True
    assert plugin._last_output == pytest.approx([0.03, 0, 0, 0, 0, 0.2])


@pytest.mark.parametrize("extra", [
    {"schema": "wrong"}, {"seq": 1}, {"obs_stamp_ms": 0},
    {"ttl_ms": 0}, {"values": [0, 1, 0, 0, 0, 0]},
    {"source": "other", "priority": -1},
])
def test_invalid_fresh_frame_cannot_use_atomic_expiry_update(setup, extra):
    plugin = start(setup, atomic_expiry_update=True)
    # Make the receipt lease expire before the source ownership timeout.
    # Lower-priority input must still fail arbitration against the live owner.
    send_received_earlier(setup, plugin, command(setup), 0.15)
    setup.clock.advance(0.16)
    send(plugin, command(setup, seq=2) | extra)
    assert plugin._counts["accepted"] == 1
    assert plugin._backend.expiry_updates == [False]
    assert plugin._backend.messages[-1] == [0] * 6
    assert plugin._last_output == [0] * 6


def test_atomic_expiry_update_does_not_replace_zero_pause_or_handover_stops(setup):
    plugin = start(setup, staged_stop=False, atomic_expiry_update=True)
    send(plugin, command(setup))
    setup.clock.advance(1.01)
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
    plugin = start(setup, staged_stop=False, atomic_expiry_update=True)
    send(plugin, command(setup))
    setup.clock.advance(1.01)
    plugin._backend.fail = True
    send(plugin, command(setup, seq=2))
    assert plugin._paused and plugin._zero_pending
    assert plugin._counts["accepted"] == 1
    assert plugin.pause_for_explicit_command("nav.move_to") is False
    plugin._backend.fail = False


def queue_during_stop(setup, plugin, monkeypatch, make_command):
    original_stop = plugin._backend.stop
    latest = {}
    def stop_with_new_input():
        original_stop()
        if latest:  # Inject once, including when the replacement is zero.
            return
        setup.clock.advance(0.08)
        latest.update(make_command())
        plugin._on_message(types.SimpleNamespace(data=json.dumps(latest)))
    monkeypatch.setattr(plugin._backend, "stop", stop_with_new_input)
    return latest


def test_newer_motion_arriving_during_legacy_stop_runs_next_iteration(setup, monkeypatch):
    plugin = start(setup, staged_stop=False)
    send(plugin, command(setup))
    setup.clock.advance(1.01)
    now = int(setup.clock.time() * 1000)
    aging = command(setup, seq=2, stamp_ms=now - 150)
    latest = queue_during_stop(setup, plugin, monkeypatch, lambda: command(
        setup, [0.1, 0, 0, 0, 0, 0.2], seq=3))
    send(plugin, aging)
    assert plugin._inbox.info()["pending"] == 1
    setup.clock.advance(0.06)
    plugin._tick()
    assert plugin._counts["accepted"] == 3
    assert plugin._last["requested"] == latest["values"]
    assert plugin._backend.deadlines[-1] == pytest.approx(latest["stamp_ms"] + 300, abs=1)
    assert plugin._counts["received"] == 3
    assert plugin._counts["dropped"] == plugin._counts["refreshed_after_stop"] == 0
    assert plugin._inbox.info()["processed"] == 3
    assert plugin._inbox.info()["pending"] == 0


def test_zero_arriving_during_stop_prevents_selected_motion_from_resuming(setup, monkeypatch):
    plugin = start(setup, staged_stop=False)
    send(plugin, command(setup))
    setup.clock.advance(1.01)
    latest = queue_during_stop(setup, plugin, monkeypatch, lambda: command(setup, [0] * 6, seq=3))
    send(plugin, command(setup, seq=2))
    plugin._tick()  # The priority zero is handled after the admitted frame yields.
    assert plugin._last["requested"] == latest["values"] == [0] * 6
    assert plugin._counts["accepted"] == 2
    assert len(plugin._backend.deadlines) == 1  # No selected motion was written.
    assert plugin._inbox.info()["pending"] == 0


@pytest.mark.parametrize("extra, verdict", [
    ({"ttl_ms": 0}, "REJECTED"),
    ({"stamp_age_ms": 201}, "DROPPED"),
    ({"obs_stamp_ms": 0}, "DROPPED"),
    ({"values": [0.1, 1, 0, 0, 0, 0]}, "REJECTED"),
    ({"schema": "wrong"}, "REJECTED"),
])
def test_refreshed_motion_still_requires_the_full_contract(setup, monkeypatch, extra, verdict):
    plugin = start(setup, staged_stop=False)
    send(plugin, command(setup))
    setup.clock.advance(1.01)
    def refreshed():
        kwargs = dict(extra)
        age = kwargs.pop("stamp_age_ms", 0)
        return command(setup, seq=3, stamp_ms=int(setup.clock.time() * 1000) - age, **kwargs)
    queue_during_stop(setup, plugin, monkeypatch, refreshed)
    send(plugin, command(setup, seq=2))
    setup.clock.advance(0.06)
    plugin._tick()
    assert plugin._last["verdict"] == verdict
    assert plugin._counts["accepted"] == 2
    assert plugin._counts["refreshed_after_stop"] == 0
    assert len(plugin._backend.deadlines) == 2
    assert any(plugin._last_output)  # Invalid later input cannot cancel this valid frame.


@pytest.mark.parametrize("extra", [
    {"source": "other"}, {"source": ""}, {"priority": 1}, {"priority": False},
    {"seq": 2}, {"seq": 1}, {"seq": True}, {"seq": 3.0},
])
def test_unrelated_or_unordered_motion_stays_queued_after_stop(setup, monkeypatch, extra):
    plugin = start(setup, staged_stop=False)
    send(plugin, command(setup))
    setup.clock.advance(1.01)
    queue_during_stop(setup, plugin, monkeypatch, lambda: command(setup, seq=3) | extra)
    send(plugin, command(setup, seq=2))
    assert plugin._last["requested"] == [0.2, 0, 0, 0, 0, 0.4]
    assert plugin._counts["accepted"] == 2
    assert plugin._counts["refreshed_after_stop"] == 0
    assert plugin._inbox.info()["pending"] == 1
    assert plugin._inbox.info()["processed"] == 2


def test_refresh_cannot_bypass_source_arbitration_for_a_pending_zero(setup, monkeypatch):
    plugin = start(setup, staged_stop=False)
    # The receipt lease expires in 150ms, leaving the source ownership
    # valid through the 80ms confirmed-stop round trip below.
    send_received_earlier(setup, plugin, command(setup, source="owner", priority=10), 0.15)
    setup.clock.advance(0.16)
    queue_during_stop(setup, plugin, monkeypatch, lambda: command(
        setup, [0] * 6, seq=3, source="other", priority=0))
    send(plugin, command(setup, seq=2, source="owner", priority=10))
    plugin._tick()
    assert plugin._last["verdict"] == "DROPPED"
    assert "outranked" in plugin._last["reason"]
    assert plugin._counts["accepted"] == 1
    assert plugin._last_output == [0] * 6
    assert len(plugin._backend.deadlines) == 1


def test_unconfirmed_stop_does_not_consume_a_new_motion_for_refresh(setup, monkeypatch):
    plugin = start(setup, staged_stop=False)
    send(plugin, command(setup))
    setup.clock.advance(1.01)
    queued = []
    def failed_stop():
        if not queued:
            latest = command(setup, seq=3)
            plugin._on_message(types.SimpleNamespace(data=json.dumps(latest)))
            queued.append(latest)
        raise RuntimeError("stop unconfirmed")
    with monkeypatch.context() as patch:
        patch.setattr(plugin._backend, "stop", failed_stop)
        send(plugin, command(setup, seq=2))
    assert plugin._paused and plugin._zero_pending
    assert plugin._counts["refreshed_after_stop"] == 0
    assert plugin._inbox.info()["processed"] == 2
    assert len(plugin._backend.deadlines) == 1


def test_native_stale_packet_does_not_cancel_and_accepts_only_a_new_sequence(setup, monkeypatch):
    plugin = start(setup)
    send(plugin, command(setup))
    backend = plugin._backend
    original_write = backend.write
    setup.clock.advance()
    expired = command(setup, seq=2)
    with monkeypatch.context() as patch:
        def expire(*args, **kwargs):
            raise StaleVelocityCommand("command lease expired at worker: remaining_ms=-12")
        patch.setattr(backend, "write", expire)
        send(plugin, expired)
    info = plugin._info()
    assert info["state"] == "running"
    assert info["counts"]["dropped"] == 1
    assert info["counts"]["accepted"] == 1
    assert info["counts"]["publish_errors"] == 0
    assert info["last"]["verdict"] == "DROPPED"
    assert "remaining_ms=-12" in info["last"]["reason"]
    assert any(backend.messages[-1])
    assert any(info["output"])
    assert plugin._last_valid_at is not None and plugin._lease_expiry_ms > 0
    assert not backend.stop_calls and not backend.soft_stop_calls
    # Retrying the same sequence cannot replay the rejected motion.
    setup.clock.advance()
    before = len(backend.messages)
    send(plugin, expired)
    assert len(backend.messages) == before
    fresh = command(setup, seq=3, ttl_ms=120)
    send(plugin, fresh)
    assert backend.write == original_write
    assert backend.messages[-1] == pytest.approx([0.06, 0, 0, 0, 0, 0.4])
    assert backend.deadlines[-1] == pytest.approx(fresh["stamp_ms"] + 300, abs=1)
    assert plugin._counts["accepted"] == 2


def test_soft_stop_failure_after_stale_input_pauses_and_refuses_handover(setup, monkeypatch):
    plugin = start(setup)
    send(plugin, command(setup))
    setup.clock.advance(0.1)
    def expire(*args, **kwargs):
        raise StaleVelocityCommand("command lease expired")
    monkeypatch.setattr(plugin._backend, "write", expire)
    plugin._backend.fail = True
    send(plugin, command(setup, seq=2))
    assert plugin._info()["state"] == "running"
    setup.clock.advance(0.21)
    plugin._tick()
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
    monkeypatch.setitem(sys.modules, "loco_servo_process", setup.module)
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


def test_zero_bypasses_apply_pacing_and_discards_pending_motion(setup):
    plugin = start(setup)
    send(plugin, command(setup))
    setup.clock.advance(0.01)
    send(plugin, command(setup, seq=2))
    assert plugin._info()["input_queue"]["pending"] == 1
    assert plugin._counts["accepted"] == 1
    send(plugin, command(setup, [0] * 6, seq=3))
    assert plugin._backend.messages[-1] == [0] * 6


def test_custom_input_and_idempotent_start_keep_one_subscription(setup):
    plugin = setup.create()
    assert plugin.dispatch("start", {"input_topics": ["/custom/cmd"]})["input"] == "/custom/cmd"
    assert plugin.dispatch("start", {"input_topic": "/custom/cmd"})["state"] == "running"
    assert len(setup.ros.executor_core.nodes) == 1
    assert plugin.dispatch("start", {"input_topic": "/different/cmd"})["state"] == "error"


def test_unknown_tool_cannot_steal_base_ownership(setup, monkeypatch):
    monkeypatch.setitem(sys.modules, "loco_servo_process", setup.module)
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


@pytest.mark.parametrize("min_vx", [0.0, 0.025, 0.3])
def test_min_vx_declares_only_the_measured_forward_floor(setup, min_vx):
    plugin = setup.create(min_vx=min_vx)
    info = plugin.dispatch("info", {})
    assert info["min_vx"] == min_vx
    assert info["control_interface"]["limits"]["min_magnitude"] == [min_vx, 0, 0, 0, 0, 0]
    assert info["control_interface"]["limits"]["upper"] == [0.3, 0, 0, 0, 0, 1.0]
    assert not setup.ros.executor_core.nodes
    assert plugin._backend is None


def test_min_vx_defaults_to_no_declared_floor_and_schema_uses_actual_max_vx(setup):
    plugin = setup.create(max_vx=0.2)
    assert plugin._info()["min_vx"] == 0
    assert plugin._info()["control_interface"]["limits"]["min_magnitude"] == [0] * 6
    schema = plugin.get_tool()["configSchema"]["properties"]["min_vx"]
    assert schema["type"] == "number" and schema["default"] == schema["minimum"] == 0
    assert schema["maximum"] == 0.2
    assert "实测" in schema["description"] and "仅向规划器声明" in schema["description"]
    assert "不在执行端强行抬高速度" in schema["description"]


@pytest.mark.parametrize("min_vx", [True, False, None, "0.02", float("nan"), float("inf"),
                                    -float("inf"), -0.001, 0.301, 10 ** 1000])
def test_invalid_min_vx_is_rejected_before_start_or_live_change(setup, min_vx):
    with pytest.raises(ValueError, match="min_vx"):
        setup.create(min_vx=min_vx)
    plugin = start(setup)
    plugin._on_message(types.SimpleNamespace(data=json.dumps(command(setup))))
    before = plugin._info()
    descriptor, parsed, sink = plugin._descriptor, plugin._parsed_descriptor, plugin._sink
    result = plugin.dispatch("config", {"min_vx": min_vx, "dry_run": True})
    assert result["state"] == "error" and "min_vx" in result["error"]
    assert plugin._min_vx == 0 and not plugin._settings["dry_run"]
    assert plugin._descriptor is descriptor and plugin._parsed_descriptor is parsed and plugin._sink is sink
    assert plugin._info()["input_queue"]["pending"] == before["input_queue"]["pending"] == 1
    assert plugin._backend.messages == []


def test_min_vx_metadata_does_not_raise_small_input_velocity_or_prevent_zero(setup):
    plugin = start(setup, min_vx=0.05)
    send(plugin, command(setup, values=[0.000005, 0, 0, 0, 0, 0]))
    assert plugin._last["verdict"] == "APPLIED"
    assert plugin._backend.messages[-1] == pytest.approx([0.000005, 0, 0, 0, 0, 0])
    send(plugin, command(setup, seq=2, values=[0] * 6))
    assert plugin._last["verdict"] == "APPLIED"
    assert plugin._backend.messages[-1] == [0] * 6


def test_live_min_vx_change_confirms_stop_clears_input_and_preserves_sink_arbitration(setup):
    plugin = start(setup)
    send(plugin, command(setup, seq=5, priority=10))
    setup.clock.advance()
    plugin._on_message(types.SimpleNamespace(data=json.dumps(command(setup, seq=6, priority=10))))
    sink = plugin._sink
    result = plugin.dispatch("config", {"min_vx": 0.025})
    assert result["ok"] and result["min_vx"] == 0.025
    assert result["control_interface"]["limits"]["min_magnitude"] == [0.025, 0, 0, 0, 0, 0]
    assert plugin._backend.messages[-1] == [0] * 6
    assert plugin._sink is sink
    assert plugin._inbox.info()["pending"] == 0 and plugin._inbox.info()["active"]
    setup.clock.advance()
    send(plugin, command(setup, seq=5, priority=10))
    assert plugin._last["verdict"] == "DROPPED" and "not newer" in plugin._last["reason"]
    send(plugin, command(setup, seq=1, source="another", priority=0))
    assert plugin._last["verdict"] == "DROPPED" and "outranked" in plugin._last["reason"]
    send(plugin, command(setup, seq=6, priority=10, values=[0.005, 0, 0, 0, 0, 0]))
    assert plugin._last["verdict"] == "APPLIED" and plugin._backend.messages[-1][0] == 0.005


def test_live_min_vx_unchanged_does_not_stop_or_clear_pending_input(setup):
    plugin = start(setup, min_vx=0.025)
    send(plugin, command(setup))
    setup.clock.advance()
    plugin._on_message(types.SimpleNamespace(data=json.dumps(command(setup, seq=2))))
    writes = list(plugin._backend.messages)
    result = plugin.dispatch("config", {"min_vx": 0.025})
    assert result["ok"] and plugin._backend.messages == writes
    assert plugin._inbox.info()["pending"] == 1


def test_min_vx_change_keeps_previous_configuration_when_stop_is_unconfirmed(setup):
    plugin = start(setup)
    send(plugin, command(setup))
    descriptor, parsed, sink = plugin._descriptor, plugin._parsed_descriptor, plugin._sink
    plugin._backend.fail = True
    result = plugin.dispatch("config", {"min_vx": 0.025, "dry_run": True})
    assert result["state"] == "error" and "stop unconfirmed" in result["error"]
    assert plugin._min_vx == 0 and not plugin._settings["dry_run"]
    assert plugin._descriptor is descriptor and plugin._parsed_descriptor is parsed and plugin._sink is sink
    assert plugin._paused and not plugin._inbox.info()["active"]
    plugin._backend.fail = False


def test_min_vx_change_keeps_previous_configuration_when_readonly_connect_fails(setup):
    plugin = start(setup, dry_run=True)
    descriptor, parsed, sink = plugin._descriptor, plugin._parsed_descriptor, plugin._sink
    plugin._backend.connect_fail = True
    result = plugin.dispatch("config", {"min_vx": 0.025, "dry_run": False})
    assert result["state"] == "error" and "connection failed" in result["error"]
    assert plugin._min_vx == 0 and plugin._settings["dry_run"]
    assert plugin._descriptor is descriptor and plugin._parsed_descriptor is parsed and plugin._sink is sink


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


def test_sdk_lease_uses_receipt_watchdog_only(setup):
    plugin = start(setup)
    cmd = command(setup, ttl_ms=150)
    send(plugin, cmd)
    assert plugin._backend.deadlines[-1] == pytest.approx(cmd["stamp_ms"] + 300, abs=1)
    setup.clock.advance()
    # Observation age only gates admission: a 450ms-old detection still gets
    # the full receipt lease instead of 50ms.
    cmd = command(setup, seq=2, obs_stamp_ms=int(setup.clock.time() * 1000) - 450)
    send(plugin, cmd)
    assert plugin._backend.deadlines[-1] == pytest.approx(cmd["stamp_ms"] + 300, abs=1)


def test_old_but_admissible_observations_do_not_cap_output_at_one_step(setup):
    # Regression: with obs_stamp_ms + 500 as an execution deadline, a steady
    # 10Hz stream of ~420ms-old detections soft-zeroed every frame and the
    # shared clamp restarted from zero, holding output at 0.03 m/s / 0.2 rad/s.
    plugin = start(setup)
    for seq in range(1, 31):
        now = int(setup.clock.time() * 1000)
        send(plugin, command(setup, [0.2, 0, 0, 0, 0, 0.8], seq=seq, obs_stamp_ms=now - 420))
        for _ in range(5):
            setup.clock.advance(0.02)
            plugin._tick()
    assert plugin._counts["soft_holds"] == 0 and plugin._counts["holds"] == 0
    assert plugin._last_output == pytest.approx([0.2, 0, 0, 0, 0, 0.8])


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
    monkeypatch.setitem(sys.modules, "loco_servo_process", setup.module)
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


@pytest.mark.parametrize("intervals", [[0.1] * 20, [0.051, 0.149] * 10])
def test_jittered_10hz_input_uses_the_same_advertised_step_per_valid_frame(setup, intervals):
    # The shared contract is per submitted frame, rather than elapsed time.
    # Equal numbers of valid frames must give equal progress under jitter.
    plugin = start(setup, max_wz=2.0, angular_acceleration=0.6)
    values = [0.2, 0, 0, 0, 0, 2.0]
    send(plugin, command(setup, values, seq=1))
    for seq, interval in enumerate(intervals, start=2):
        setup.clock.advance(interval)
        send(plugin, command(setup, values, seq=seq))
    assert plugin._last_output[5] == pytest.approx(0.6 / 10 * 21)
    assert plugin._counts["accepted"] == 21
    assert plugin._counts["holds"] == plugin._counts["dropped"] == 0


def test_default_turn_ramp_reaches_search_limit_within_half_second(setup):
    plugin = start(setup)
    assert plugin._info()["angular_acceleration"] == 2.0
    assert plugin._info()["linear_acceleration"] == 0.3
    for seq in range(1, 6):
        send(plugin, command(setup, [0.2, 0, 0, 0, 0, 1.0], seq=seq))
        assert plugin._last_output[5] == pytest.approx(0.2 * seq)
        setup.clock.advance()
    assert plugin._last_output[0] == pytest.approx(0.15)
    assert plugin._backend.stop_calls == 0


@pytest.mark.parametrize("translating", [False, True])
def test_nonzero_target_crossing_zero_does_not_cancel_sdk_action(setup, translating):
    # 125ms and 2 rad/s² give exactly representable quarter-unit steps, so the
    # reversal lands precisely on zero rather than a tiny nonzero float.
    plugin = start(setup, expected_hz=8, linear_acceleration=2.0)
    forward = [0.25 if translating else 0, 0, 0, 0, 0, 0.25]
    reverse = [-value for value in forward]
    send(plugin, command(setup, forward, seq=1))
    assert plugin._last_output == pytest.approx(forward)
    setup.clock.advance(0.125)
    send(plugin, command(setup, reverse, seq=2))
    assert plugin._last_output == [0] * 6
    assert plugin._backend.preserved_actions == [False, True]
    assert plugin._backend.stop_calls == 0
    assert plugin._backend.deadlines[-1] > setup.clock.time() * 1000
    setup.clock.advance(0.125)
    send(plugin, command(setup, reverse, seq=3))
    assert plugin._last_output == pytest.approx(reverse)
    assert plugin._backend.stop_calls == 0
    setup.clock.advance(0.125)
    send(plugin, command(setup, [0] * 6, seq=4))
    assert plugin._backend.stop_calls == 1
    assert plugin._last_output == [0] * 6
    assert plugin._backend.preserved_actions == [False, True, False]


def test_acceleration_configuration_is_declared_and_idle_update_does_no_io(setup):
    plugin = setup.create()
    properties = plugin.get_tool()["configSchema"]["properties"]
    assert properties["linear_acceleration"]["default"] == 0.3
    assert properties["angular_acceleration"]["default"] == 2.0
    assert properties["linear_acceleration"]["exclusiveMinimum"] == 0
    result = plugin.dispatch("config", {"linear_acceleration": 0.45,
                                        "angular_acceleration": 1.2})
    assert result["ok"] and result["state"] == "idle"
    assert result["linear_acceleration"] == 0.45
    assert result["angular_acceleration"] == 1.2
    assert result["acceleration_limits"] == {"linear_m_s2": 0.45, "angular_rad_s2": 1.2}
    assert result["control_interface"]["limits"]["upper"] == [0.3, 0, 0, 0, 0, 1.0]
    assert plugin._backend is None and plugin._sink is None
    assert not setup.ros.executor_core.nodes


@pytest.mark.parametrize("field", ["linear_acceleration", "angular_acceleration"])
@pytest.mark.parametrize("value", [True, False, None, "1", 0, -0.1,
                                  float("nan"), float("inf"), -float("inf"), 10 ** 400])
def test_invalid_acceleration_changes_are_atomic_and_do_not_stop_motion(setup, field, value):
    with pytest.raises(ValueError, match=field):
        setup.create(**{field: value})
    plugin = start(setup)
    send(plugin, command(setup))
    before = plugin._info()
    descriptor, sink = plugin._descriptor, plugin._sink
    result = plugin.dispatch("config", {field: value, "dry_run": True, "min_vx": 0.025})
    assert result["state"] == "error" and field in result["error"]
    after = plugin._info()
    for key in ("linear_acceleration", "angular_acceleration", "min_vx", "dry_run", "output"):
        assert after[key] == before[key]
    assert plugin._descriptor is descriptor and plugin._sink is sink
    assert plugin._backend.stop_calls == 0
    assert plugin._counts["holds"] == 0


def test_live_acceleration_config_stops_clears_old_input_and_keeps_arbitration(setup):
    plugin = start(setup)
    send(plugin, command(setup))
    sink = plugin._sink
    plugin._on_message(types.SimpleNamespace(data=json.dumps(command(setup, seq=2))))
    result = plugin.dispatch("config", {"linear_acceleration": 0.6,
                                        "angular_acceleration": 1.0})
    assert result["ok"] and result["state"] == "running"
    assert plugin._backend.stop_calls == 1
    assert plugin._sink is sink
    assert plugin._info()["input_queue"]["pending"] == 0
    plugin._tick()
    assert plugin._counts["accepted"] == 1
    setup.clock.advance()
    send(plugin, command(setup, seq=3))
    assert plugin._last_output == pytest.approx([0.06, 0, 0, 0, 0, 0.1])


def test_unchanged_acceleration_config_does_not_stop_or_discard_input(setup):
    plugin = start(setup)
    send(plugin, command(setup))
    plugin._on_message(types.SimpleNamespace(data=json.dumps(command(setup, seq=2))))
    result = plugin.dispatch("config", {"linear_acceleration": 0.3,
                                        "angular_acceleration": 2.0})
    assert result["ok"]
    assert plugin._backend.stop_calls == 0
    assert plugin._info()["input_queue"]["pending"] == 1


def test_acceleration_update_does_not_commit_after_unconfirmed_stop(setup):
    plugin = start(setup)
    send(plugin, command(setup))
    plugin._backend.fail = True
    result = plugin.dispatch("config", {"angular_acceleration": 1.0, "dry_run": True})
    assert result["state"] == "error"
    assert plugin._accel == (0.3, 2.0)
    assert not plugin._settings["dry_run"]
    assert plugin._paused
    plugin._backend.fail = False


def test_acceleration_update_does_not_commit_after_readonly_connect_failure(setup):
    plugin = start(setup, dry_run=True)
    plugin._backend.connect_fail = True
    result = plugin.dispatch("config", {"angular_acceleration": 1.0, "dry_run": False})
    assert result["state"] == "error"
    assert plugin._accel == (0.3, 2.0)
    assert plugin._settings["dry_run"]
    assert not plugin._backend.messages


def test_descriptor_declares_the_only_step_limiter_and_zero_exception(setup):
    plugin = setup.create()
    interface = plugin._info()["control_interface"]
    assert interface["limits"]["max_delta_per_step"] == [0.03, 1e-6, 1e-6, 1e-6, 1e-6, 0.2]
    assert interface["limits"]["zero_immediate"] is True
    assert "max_velocity" not in interface["limits"]  # Delta/dt here would be jerk.
    varied = setup.create(expected_hz=20, linear_acceleration=0.6, angular_acceleration=1.0)
    assert varied._info()["control_interface"]["limits"]["max_delta_per_step"] == [
        0.03, 1e-6, 1e-6, 1e-6, 1e-6, 0.05]


def test_sdk_receives_shared_clamped_values_without_a_second_ramp(setup, monkeypatch):
    plugin = start(setup)
    seen = []
    clamp = plugin._sink._clamp_step
    def capture(values):
        result = clamp(values)
        seen.append(result[0])
        return result
    monkeypatch.setattr(plugin._sink, "_clamp_step", capture)
    for sequence in range(1, 7):
        send(plugin, command(setup, [0.3, 0, 0, 0, 0, 1], seq=sequence))
        assert plugin._backend.messages[-1] == list(seen[-1])
        assert plugin._last_output == list(seen[-1])
        assert plugin._last["verdict"] == "CLAMPED"
        setup.clock.advance(0.06)  # A second private dt ramp would disagree here.
    assert plugin._backend.messages[0] == [0.03, 0, 0, 0, 0, 0.2]
    assert plugin._last_output[0] == pytest.approx(0.18)


def test_single_axis_zero_immediately_stops_translation_while_turning(setup):
    plugin = start(setup)
    for sequence in range(1, 12):
        send(plugin, command(setup, [0.3, 0, 0, 0, 0, 0.6], seq=sequence))
        setup.clock.advance()
    send(plugin, command(setup, [0, 0, 0, 0, 0, -0.6], seq=12))
    assert plugin._last_output[0] == 0
    assert plugin._last_output[5] == pytest.approx(0.4)
    assert plugin._backend.stop_calls == 0
    assert plugin._sink._last_values == tuple(plugin._last_output)
    setup.clock.advance()
    send(plugin, command(setup, [0] * 6, seq=13))
    assert plugin._last_output == [0] * 6
    assert plugin._backend.stop_calls == 1
    assert plugin._sink._last_values == (0,) * 6


def test_soft_hold_rebases_actual_zero_without_forgetting_source_or_sequence(setup):
    plugin = start(setup)
    for sequence in range(1, 12):
        send(plugin, command(setup, [0.3, 0, 0, 0, 0, 1], seq=sequence,
                             source="owner", priority=10))
        setup.clock.advance()
    setup.clock.advance(0.21)
    plugin._tick()
    assert plugin._soft_held and plugin._sink._last_values == (0,) * 6
    assert plugin._sink.stats()["holder"] == "owner"
    assert plugin._sink._seq_by_source["owner"] == 11
    send(plugin, command(setup, seq=11, source="owner", priority=10))
    assert plugin._last["verdict"] == "DROPPED"
    send(plugin, command(setup, [0.3, 0, 0, 0, 0, 1], seq=12, source="owner", priority=10))
    assert plugin._last_output == [0.03, 0, 0, 0, 0, 0.2]
    assert plugin._backend.stop_calls == 0


def test_live_step_config_reconfigures_the_same_sink_and_preserves_arbitration(setup):
    plugin = start(setup)
    send(plugin, command(setup, source="owner", priority=10))
    sink = plugin._sink
    result = plugin.dispatch("config", {"linear_acceleration": 0.6, "angular_acceleration": 1.0})
    assert result["ok"] and plugin._sink is sink
    assert sink.descriptor is plugin._parsed_descriptor
    assert sink.descriptor.max_delta_per_step == (0.06, 1e-6, 1e-6, 1e-6, 1e-6, 0.1)
    assert sink.stats()["holder"] == "owner" and sink._seq_by_source["owner"] == 1
    setup.clock.advance()
    send(plugin, command(setup, seq=1, source="owner", priority=10))
    assert plugin._last["verdict"] == "DROPPED"
    send(plugin, command(setup, seq=2, source="other", priority=0))
    assert plugin._last["verdict"] == "DROPPED" and "outranked" in plugin._last["reason"]
    send(plugin, command(setup, [0.3, 0, 0, 0, 0, 1], seq=2, source="owner", priority=10))
    assert plugin._last_output == [0.06, 0, 0, 0, 0, 0.1]


def test_rotate_only_rebases_to_the_translation_actually_executed(setup):
    plugin = start(setup, rotate_only=True)
    for sequence in range(1, 6):
        send(plugin, command(setup, [0.3, 0, 0, 0, 0, 1], seq=sequence))
        assert plugin._last_output[0] == 0
        assert plugin._sink._last_values[0] == 0
        setup.clock.advance()
    assert plugin.dispatch("config", {"rotate_only": False})["ok"]
    setup.clock.advance()
    send(plugin, command(setup, [0.3, 0, 0, 0, 0, 1], seq=6))
    assert plugin._last_output == [0.03, 0, 0, 0, 0, 0.2]


def test_fresh_input_after_native_motion_deadline_clamps_from_zero_before_python_tick(setup):
    plugin = start(setup)
    for sequence in range(1, 12):
        send(plugin, command(setup, [0.3, 0, 0, 0, 0, 1], seq=sequence))
        setup.clock.advance()
    setup.clock.advance(0.21)
    # Let a new valid frame be selected before Python's post-input deadline
    # check. Native FOLLOW will have zeroed the old motion independently.
    send(plugin, command(setup, [0.3, 0, 0, 0, 0, 1], seq=12))
    assert plugin._last_output == [0.03, 0, 0, 0, 0, 0.2]
    assert plugin._backend.stop_calls == 0
    assert not plugin._soft_held


def test_expired_zero_input_does_not_change_the_shared_execution_baseline(setup):
    plugin = start(setup)
    send(plugin, command(setup, [0.3, 0, 0, 0, 0, 1]))
    previous = plugin._sink._last_values
    setup.clock.advance(0.1)
    send(plugin, command(setup, [0] * 6, seq=2, stamp_ms=int(setup.clock.time() * 1000) - 201))
    assert plugin._last["verdict"] == "DROPPED"
    assert plugin._sink._last_values == previous
    assert plugin._backend.stop_calls == 0
    assert tuple(plugin._last_output) == previous


@pytest.mark.parametrize("field", ["linear_acceleration", "angular_acceleration"])
def test_configuration_rejects_underflowed_step_before_stopping_current_motion(setup, field):
    plugin = start(setup)
    send(plugin, command(setup))
    previous = plugin._descriptor
    assert plugin.dispatch("config", {field: 5e-324})["state"] == "error"
    assert plugin._descriptor is previous
    assert plugin._backend.stop_calls == 0
    assert plugin._accel == (0.3, 2.0)


@pytest.mark.parametrize("confirmed_zero", [False, True])
def test_native_stale_ack_rebases_only_a_confirmed_real_zero(setup, monkeypatch, confirmed_zero):
    plugin = start(setup)
    for sequence in range(1, 11):
        send(plugin, command(setup, [0.3, 0, 0, 0, 0, 1], seq=sequence))
        setup.clock.advance()
    backend = plugin._backend
    old_output = list(plugin._last_output)
    old_baseline = plugin._sink._last_values
    old_receipt = plugin._last_valid_at
    old_motion = plugin._lease_expiry_ms
    old_cancel = plugin._cancel_deadline_ms
    old_output_time = plugin._last_output_at

    def expire_during_sdk(values, deadline, **options):
        assert deadline < old_motion  # Late receipt leaves only 10ms.
        setup.clock.advance(0.02)
        assert old_motion > setup.clock.time() * 1000
        if confirmed_zero:
            # Native's own deadline loop, rather than Python STOP/soft_stop.
            backend.messages.append([0] * 6)
        raise StaleVelocityCommand("motion deadline spent in SDK", zero_velocity_confirmed=confirmed_zero)

    with monkeypatch.context() as patch:
        patch.setattr(backend, "write", expire_during_sdk)
        send_received_earlier(setup, plugin, command(setup, [0.3, 0, 0, 0, 0, 1], seq=11), 0.29)
    assert plugin._counts["accepted"] == 10 and plugin._counts["dropped"] == 1
    assert plugin._last["verdict"] == "DROPPED"
    assert plugin._last_valid_at == old_receipt
    assert plugin._cancel_deadline_ms == old_cancel
    assert backend.stop_calls == 0 and not backend.soft_stop_calls
    if confirmed_zero:
        assert plugin._last_output == [0] * 6
        assert plugin._sink._last_values == (0,) * 6
        assert plugin._last_output_at is None
        assert plugin._lease_expiry_ms == 0 and plugin._soft_held
    else:
        assert plugin._last_output == old_output
        assert plugin._sink._last_values == old_baseline
        assert plugin._last_output_at == old_output_time
        assert plugin._lease_expiry_ms == old_motion and not plugin._soft_held

    send(plugin, command(setup, [0.3, 0, 0, 0, 0, 1], seq=12))
    expected = [0.03, 0, 0, 0, 0, 0.2] if confirmed_zero else old_output
    assert plugin._last_output == pytest.approx(expected)
    assert backend.messages[-1] == pytest.approx(expected)
    assert plugin._sink._last_values == tuple(plugin._last_output)
    assert plugin._counts["accepted"] == 11
    assert backend.stop_calls == 0


def _follow_loop(setup, plugin, *, nav_gain=1.5, latency_s=0.4, target=0.6, seconds=6.0):
    """Closed loop: navi turns at nav_gain x a latency_s-old bearing; the base
    integrates the executed wz. Returns (bearing history, executed wz history)."""
    yaw_log = [(setup.clock.monotonic(), 0.0)]
    yaw, bearings, executed, seq = 0.0, [], [], 0

    def yaw_at(t):
        for when, value in reversed(yaw_log):
            if when <= t:
                return value
        return 0.0

    for _ in range(int(seconds * 10)):
        seq += 1
        now = setup.clock.monotonic()
        bearing_obs = target - yaw_at(now - latency_s)
        wz = max(-1.0, min(1.0, nav_gain * bearing_obs))
        stamp = int(setup.clock.time() * 1000)
        send(plugin, command(setup, [0, 0, 0, 0, 0, wz], seq=seq,
                             obs_stamp_ms=stamp - int(latency_s * 1000)))
        for _ in range(5):
            yaw += plugin._last_output[5] * 0.02
            setup.clock.advance(0.02)
            yaw_log.append((setup.clock.monotonic(), yaw))
            plugin._tick()
        bearings.append(target - yaw)
        executed.append(plugin._last_output[5])
    return bearings, executed


def _sign_changes(values, floor=0.02):
    signs = [1 if v > floor else -1 for v in values if abs(v) > floor]
    return sum(a != b for a, b in zip(signs, signs[1:]))


def test_latency_compensation_damps_the_closed_follow_loop(setup):
    plain, plain_wz = _follow_loop(setup, start(setup, dry_run=True))
    compensated, comp_wz = _follow_loop(setup, start(setup, dry_run=True, yaw_latency_gain=1.5))
    # Without compensation a 400ms-old bearing makes the base swing past the
    # target and back; with gain equal to navi's it settles without reversing.
    assert min(plain) < -0.05 and _sign_changes(plain_wz) >= 2
    assert min(compensated) > -0.05 and _sign_changes(comp_wz) == 0
    assert abs(compensated[-1]) < 0.03


def test_latency_compensation_subtracts_rotation_since_observation(setup):
    plugin = start(setup, yaw_latency_gain=1.0)
    for seq in range(1, 11):  # Turn at a steady 0.5 rad/s for one second.
        send(plugin, command(setup, [0, 0, 0, 0, 0, 0.5], seq=seq))
        setup.clock.advance(0.1)
    now = int(setup.clock.time() * 1000)
    send(plugin, command(setup, [0, 0, 0, 0, 0, 0.5], seq=11, obs_stamp_ms=now - 400))
    turn = plugin._info()["last_command_timing"]["turn"]
    # Since the frame was captured the base turned about 0.5 x 0.4 = 0.2 rad.
    assert turn["rotated_since_observation_rad"] == pytest.approx(0.2, abs=0.02)
    assert turn["shaped_wz"] == pytest.approx(0.3, abs=0.02)
    assert plugin._last_output[5] == pytest.approx(0.3, abs=0.02)


def test_compensation_is_off_by_default_and_leaves_navi_output_unchanged(setup):
    plugin = start(setup)
    for seq in range(1, 11):
        send(plugin, command(setup, [0, 0, 0, 0, 0, 0.5], seq=seq))
        setup.clock.advance(0.1)
    now = int(setup.clock.time() * 1000)
    send(plugin, command(setup, [0, 0, 0, 0, 0, 0.5], seq=11, obs_stamp_ms=now - 400))
    assert plugin._info()["yaw_latency_gain"] == 0.0
    assert plugin._last_output[5] == pytest.approx(0.5)


def test_wz_limit_clamps_instead_of_rejecting(setup):
    plugin = start(setup, wz_limit=0.4)
    for seq in range(1, 8):
        send(plugin, command(setup, [0.1, 0, 0, 0, 0, -0.9], seq=seq))
        setup.clock.advance(0.1)
    assert plugin._counts["rejected"] == 0
    assert plugin._last_output[5] == pytest.approx(-0.4)
    assert plugin._info()["last_command_timing"]["turn"]["requested_wz"] == -0.9
    # The descriptor navi reads is unchanged.
    assert plugin._info()["control_interface"]["limits"]["upper"][5] == 1.0


def test_explicit_zero_is_never_shaped_into_motion(setup):
    plugin = start(setup, yaw_latency_gain=2.0)
    for seq in range(1, 6):
        send(plugin, command(setup, [0, 0, 0, 0, 0, 0.8], seq=seq))
        setup.clock.advance(0.1)
    now = int(setup.clock.time() * 1000)
    send(plugin, command(setup, [0] * 6, seq=6, obs_stamp_ms=now - 400))
    assert plugin._last_output == [0] * 6
    assert plugin._backend.messages[-1] == [0] * 6


def test_compensated_turn_to_zero_keeps_the_action(setup):
    # navi still asks to turn, compensation cancels it exactly: velocity goes
    # to zero but this is not an explicit stop, so the action is retained.
    plugin = start(setup, yaw_latency_gain=1.0)
    for seq in range(1, 11):
        send(plugin, command(setup, [0, 0, 0, 0, 0, 0.5], seq=seq))
        setup.clock.advance(0.1)
    now = int(setup.clock.time() * 1000)
    stops = plugin._backend.stop_calls
    send(plugin, command(setup, [0, 0, 0, 0, 0, 0.2], seq=11, obs_stamp_ms=now - 400))
    assert plugin._last_output[5] == pytest.approx(0.0, abs=1e-9)
    assert plugin._backend.stop_calls == stops
    assert plugin._last_preserved_action is True


@pytest.mark.parametrize("args", [{"wz_limit": 0}, {"wz_limit": 1.5}, {"wz_limit": True},
                                  {"yaw_latency_gain": -0.1}, {"yaw_latency_gain": 6},
                                  {"yaw_latency_gain": float("nan")}])
def test_follow_shaping_config_is_validated(setup, args):
    plugin = start(setup)
    assert plugin.dispatch("config", args)["state"] == "error"
    assert plugin._wz_limit == 1.0 and plugin._yaw_gain == 0.0


def test_follow_shaping_config_applies_without_stopping(setup):
    plugin = start(setup)
    send(plugin, command(setup))
    stops = plugin._backend.stop_calls
    result = plugin.dispatch("config", {"wz_limit": 0.6, "yaw_latency_gain": 1.2})
    assert result["ok"] and result["wz_limit"] == 0.6 and result["yaw_latency_gain"] == 1.2
    assert plugin._backend.stop_calls == stops
    schema = plugin.get_tool()["inputSchema"]
    assert {"wz_limit", "yaw_latency_gain"} <= set(schema["properties"])
    assert {"wz_limit", "yaw_latency_gain"} <= set(schema["x-action-params"]["config"]["params"])
