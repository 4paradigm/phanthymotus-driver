"""Run the real ActuCore producer against the AS2W receiver without ROS.

The other repository is intentionally not copied into this one. Check out
4paradigm/phanthymotus at PLATFORM_REVISION, then run:

  PHANTHYMOTUS_CHECKOUT=/path/to/phanthymotus \
    python -m pytest tests/test_as2w_control_stream_compat.py -q

Only the ROS endpoints/clock/SDK are fakes: publisher construction, navigation
policy, message builder/JSON serialization, receiver callback and ControlSink
all execute their real implementations. Type matching is modeled, not DDS
discovery, serialization/type support, scheduling or real robot behavior.
"""
from __future__ import annotations

import importlib
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import sys
import types

import pytest


ROOT = Path(__file__).resolve().parents[1]
PLATFORM_REVISION = "b42effb65b395d0860e36f85ba40f6e10129e55a"
PACKAGE = "_as2w_platform_compat_plugins"


class Clock:
    seconds = 1000.0

    def now(self):
        return self.seconds

    def advance(self, seconds):
        self.seconds += seconds


class String:
    def __init__(self):
        self.data = ""


class FakeSDK:
    """Any SDK call would violate these dry-run tests."""
    max_call_seconds = 0.35

    def __init__(self):
        self.calls = []

    def __getattr__(self, name):
        if name in {"Move", "StopMove", "GetState", "acquire_control", "release_control"}:
            def forbidden(*args):
                self.calls.append((name, args))
                raise AssertionError("dry_run called SDK: " + name)
            return forbidden
        raise AttributeError(name)


class Bus:
    """Match type and topic before delivering the actual published object."""
    def __init__(self):
        self.subscriptions = []
        self.publishers = []
        self.pending = []
        self.messages = []
        self.delivered = 0
        self.hold = False

    def deliver(self, publisher, message):
        assert type(message) is publisher.message_type
        for sub in self.subscriptions:
            if sub.topic == publisher.topic and sub.message_type is publisher.message_type:
                sub.callback(message)
                self.delivered += 1

    def flush(self):
        pending, self.pending = self.pending, []
        for publisher, message in pending:
            self.deliver(publisher, message)


def _load(monkeypatch, name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    monkeypatch.setitem(sys.modules, name, module)
    spec.loader.exec_module(module)
    return module


@pytest.fixture
def link(monkeypatch):
    checkout = os.environ.get("PHANTHYMOTUS_CHECKOUT")
    if not checkout:
        pytest.skip("set PHANTHYMOTUS_CHECKOUT to the pinned platform checkout for cross-repository validation")
    checkout = Path(checkout).resolve()
    revision = subprocess.check_output(
        ["git", "-C", str(checkout), "rev-parse", "HEAD"], text=True).strip()
    assert revision == PLATFORM_REVISION, "update/audit the pinned producer before changing its revision"
    bus, clock, sdk = Bus(), Clock(), FakeSDK()

    class Node:
        def __init__(self, name):
            self.name = name

        def create_publisher(self, message_type, topic, qos):
            publisher = types.SimpleNamespace(message_type=message_type, topic=topic, qos=qos)

            def publish(message):
                bus.messages.append(message)
                if bus.hold:
                    bus.pending.append((publisher, message))
                else:
                    bus.deliver(publisher, message)

            publisher.publish = publish
            bus.publishers.append(publisher)
            return publisher

        def create_subscription(self, message_type, topic, callback, qos):
            sub = types.SimpleNamespace(message_type=message_type, topic=topic,
                                        callback=callback, qos=qos)
            bus.subscriptions.append(sub)
            return sub

        def create_timer(self, period, callback):
            return types.SimpleNamespace(period=period, callback=callback)

        def destroy_timer(self, timer):
            pass

        def destroy_node(self):
            pass

    def install(name, **attributes):
        module = types.ModuleType(name)
        module.__dict__.update(attributes)
        monkeypatch.setitem(sys.modules, name, module)
        return module

    install("rclpy")
    install("rclpy.node", Node=Node)
    install("rclpy.qos", QoSProfile=lambda **kw: types.SimpleNamespace(**kw),
            ReliabilityPolicy=types.SimpleNamespace(BEST_EFFORT="best_effort", RELIABLE="reliable"),
            HistoryPolicy=types.SimpleNamespace(KEEP_LAST="keep_last"),
            DurabilityPolicy=types.SimpleNamespace(VOLATILE="volatile"))
    install("std_msgs")
    install("std_msgs.msg", String=String)
    install("sensor_msgs")
    install("sensor_msgs.msg", CompressedImage=type("CompressedImage", (), {}))

    # A unique package prevents collisions with another component's `plugins`.
    package = install(PACKAGE)
    package.__path__ = [str(checkout / "actucore" / "plugins")]
    navi = importlib.import_module(PACKAGE + ".navi.plugin")
    monkeypatch.syspath_prepend(str(ROOT))
    _load(monkeypatch, "as2w_control", ROOT / "unitree/as2w/as2w_control.py")
    servo = _load(monkeypatch, "_as2w_stream_compat_servo", ROOT / "unitree/as2w/loco_servo.py")
    monkeypatch.setattr(navi.time, "time", clock.now)
    executor = types.SimpleNamespace(add_node=lambda node: None, remove_node=lambda node: None)
    receiver = servo.LocoServoPlugin({}, "", executor, sdk)
    controller = receiver._controller
    controller._threaded = False
    controller._clock = controller._wall = clock.now
    producer = navi.NaviPlugin({"view_hz": 0}, executor)

    def start(descriptor=None):
        result = receiver.dispatch("start", {"input_topic": producer._topic})
        assert result["ok"]
        return producer.dispatch("navi", {
            "action": "start",
            "control_interface": descriptor if descriptor is not None else receiver._info()["control_interface"],
            "input_topics": ["/cam/objects", "/cam/visual_depth_summary"],
        })

    def publish_first(observation_age_ms=0):
        # Synthetic sensor measurements feed the actual navigation policy.
        producer._state = navi.policy_mod.State(target="chair")
        producer._objects = {"objects": [{"name": "chair", "confidence": .9,
                                           "position": [.5, 0.0]}]}
        producer._objects_ms = int(clock.now() * 1000) - observation_age_ms
        producer._depth_bands = {"left": 5.0, "center": 5.0, "right": 5.0}
        producer._depth_ms = int(clock.now() * 1000)
        for _ in range(producer._config.confirm_hits + 1):
            producer._tick()
            if bus.messages:
                return json.loads(bus.messages[-1].data)
        pytest.fail("real navigation policy emitted no command")

    yield types.SimpleNamespace(bus=bus, clock=clock, sdk=sdk, receiver=receiver,
                                controller=controller, producer=producer, start=start,
                                publish_first=publish_first)
    producer._stop()
    receiver.stop()
    for name in list(sys.modules):
        if name.startswith(PACKAGE + "."):
            sys.modules.pop(name, None)


def test_actual_navi_string_publisher_reaches_as2w_shared_sink(link):
    assert link.start()["state"] == "running"
    message = link.publish_first()
    publisher = link.bus.publishers[0]
    subscription = next(s for s in link.bus.subscriptions if s.topic == publisher.topic)
    assert publisher.message_type is subscription.message_type is String
    assert publisher.qos == 10 and subscription.qos == 1  # both use default RELIABLE QoS
    assert message["schema"] == "motus.control/1"
    assert message["mode"] == "twist" and message["dof"] == 6
    assert message["control_interface"]["joint_names"] == ["vx", "vy", "vz", "wx", "wy", "wz"]
    assert any(message["values"]), "exercise a nonzero real policy command"
    assert link.bus.delivered == 1
    link.controller.pump()
    info = link.controller.info()
    assert info["validated"] == info["simulated"] == 1
    assert info["last_outcome"]["verdict"] in {"applied", "clamped"}
    assert link.sdk.calls == []


def test_actual_published_command_is_rejected_after_transport_delay(link):
    assert link.start()["state"] == "running"
    link.bus.hold = True
    message = link.publish_first()
    link.clock.advance((message["ttl_ms"] + 1) / 1000)
    link.bus.flush()
    link.controller.pump()
    info = link.controller.info()
    assert link.bus.delivered == 1
    assert info["validated"] == info["simulated"] == 0
    assert info["rejected"] == 1
    assert "expired" in info["last_outcome"]["reason"].lower()
    assert link.sdk.calls == []


def test_actual_producer_observation_stamp_is_checked_by_receiver(link):
    assert link.start()["state"] == "running"
    # Producer policy permits the observation, but receiver's negotiated
    # 500 ms ceiling must still reject it at the motor-side safety boundary.
    link.producer._config.max_obs_age_ms = 1000
    message = link.publish_first(observation_age_ms=600)
    assert message["stamp_ms"] - message["obs_stamp_ms"] == 600
    link.controller.pump()
    info = link.controller.info()
    assert info["validated"] == info["simulated"] == 0
    assert info["rejected"] == 1
    assert "observation" in info["last_outcome"]["reason"].lower()
    assert link.sdk.calls == []


def test_actual_producer_refuses_a_mismatched_receiver_descriptor(link):
    descriptor = link.receiver._info()["control_interface"]
    descriptor["mode"] = "joint_position"
    result = link.start(descriptor)
    assert result["state"] == "error"
    assert "twist" in result["message"]
    assert link.bus.publishers == []
    assert link.sdk.calls == []
