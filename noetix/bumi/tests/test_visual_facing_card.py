"""Plugin-level behaviour for visual_facing, without ROS or the vendor SDK."""

from __future__ import annotations

import json
import math
import sys
import time
import types
from pathlib import Path

import pytest

BUMI = str(Path(__file__).resolve().parents[1])
sys.path.insert(0, BUMI)


class _String:
    def __init__(self, data=""):
        self.data = data


class _Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg.data)


class _Node:
    def __init__(self, name):
        self.name = name
        self.publishers = {}
        self.subscriptions = {}

    def create_publisher(self, _type, topic, _qos):
        pub = _Publisher()
        self.publishers[topic] = pub
        return pub

    def create_subscription(self, _type, topic, callback, _qos):
        self.subscriptions[topic] = callback
        return types.SimpleNamespace(topic=topic)

    def destroy_subscription(self, _sub):
        pass


class _HighCtrl:
    def __init__(self):
        self.mode = 2
        self.commands = []

    def get_mode(self):
        return self.mode

    def publish_cmd(self, x, y, z, action, index):
        self.commands.append((x, y, z))


@pytest.fixture
def card_module(monkeypatch):
    rclpy = types.ModuleType("rclpy")
    node_mod = types.ModuleType("rclpy.node")
    node_mod.Node = _Node
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
    msg_mod.String = _String
    std_msgs.msg = msg_mod
    sdk = types.ModuleType("highcontrol_py")
    sdk.ControlCmd = types.SimpleNamespace(DEFAULT="DEFAULT")

    for name, mod in {
        "rclpy": rclpy, "rclpy.node": node_mod, "rclpy.qos": qos_mod,
        "std_msgs": std_msgs, "std_msgs.msg": msg_mod,
        "highcontrol_py": sdk,
    }.items():
        monkeypatch.setitem(sys.modules, name, mod)

    sys.modules.pop("visual_facing_card", None)
    import visual_facing_card
    # Keep the worker deterministic: tests drive _tick themselves.
    monkeypatch.setattr(
        visual_facing_card.threading, "Thread",
        lambda *a, **kw: types.SimpleNamespace(start=lambda: None,
                                               is_alive=lambda: False,
                                               join=lambda **k: None))
    return visual_facing_card


def _make_plugin(card_module, *, ctrl=None, cfg=None):
    ctrl = ctrl or _HighCtrl()
    plugin = card_module.VisualFacingPlugin(
        cfg or {}, "bumi_test", types.SimpleNamespace(add_node=lambda node: None), ctrl)
    return plugin, ctrl


def _start(plugin, topic="/vision/ar/poses"):
    out = plugin.dispatch("start", {"input_topic": topic})
    assert out["state"] == "running"
    return plugin._run_token


def _target(x=0.0, z=1.0, ident=17, age_ms=0):
    return json.dumps({
        "marker_id": ident,
        "tvec": [x, 0.0, z],
        "stamp_ms": time.time() * 1000.0 - age_ms,
    })


def _feed(plugin, payload, topic="/vision/ar/poses"):
    plugin._node.subscriptions[topic](_String(payload))


def _status_messages(plugin):
    return plugin._node.publishers[plugin._status_topic].messages


def test_start_without_a_topic_never_claims_to_be_running(card_module):
    plugin, ctrl = _make_plugin(card_module)
    out = plugin.dispatch("start", {})
    assert out["state"] == "error"
    assert ctrl.commands == []


def test_start_and_enable_are_two_separate_deliberate_actions(card_module):
    plugin, ctrl = _make_plugin(card_module)
    token = _start(plugin)
    assert ctrl.commands == []  # start alone never moves the robot
    assert plugin.dispatch("enable", {"confirm": False})["state"] == "error"
    assert plugin.dispatch("enable", {"confirm": True})["state"] == "armed"
    assert plugin._deadline_ms is not None
    plugin._tick(token)


def test_a_visible_target_produces_a_zero_translation_yaw_command(card_module):
    plugin, ctrl = _make_plugin(card_module)
    token = _start(plugin)
    plugin.dispatch("enable", {"confirm": True})
    _feed(plugin, _target(x=0.6))  # target on the right -> negative z
    plugin._tick(token)
    assert len(ctrl.commands) == 1
    x, y, z = ctrl.commands[0]
    assert x == 0.0 and y == 0.0
    assert z < 0.0
    status = json.loads(_status_messages(plugin)[-1])
    assert status["state"] == "turning"
    assert status["yaw_error_deg"] == pytest.approx(-math.degrees(math.atan2(0.6, 1.0)),
                                                    abs=0.01)


def test_an_occluded_target_publishes_exactly_one_zero(card_module):
    plugin, ctrl = _make_plugin(card_module)
    token = _start(plugin)
    plugin.dispatch("enable", {"confirm": True})
    _feed(plugin, _target(x=0.6))
    plugin._tick(token)
    assert ctrl.commands[-1][2] != 0.0
    plugin._last_target = None
    plugin._tick(token)
    plugin._tick(token)
    # The original turn command plus exactly one zero edge; a dead source
    # must not generate a zero on every silent tick.
    assert len(ctrl.commands) == 2
    assert ctrl.commands[-1][2] == 0.0
    assert json.loads(_status_messages(plugin)[-1])["stop_reason"] == "no_fresh_target"


def test_an_expired_authorisation_stops_the_turn(card_module):
    plugin, ctrl = _make_plugin(card_module)
    token = _start(plugin)
    plugin.dispatch("enable", {"confirm": True})
    _feed(plugin, _target(x=0.6))
    plugin._tick(token)
    plugin._deadline_ms = time.time() * 1000.0 - 1.0
    plugin._tick(token)
    assert ctrl.commands[-1][2] == 0.0
    assert json.loads(_status_messages(plugin)[-1])["stop_reason"] == \
        "authorization_expired"


def test_protection_mode_and_estop_each_stop_immediately(card_module):
    plugin, ctrl = _make_plugin(card_module)
    token = _start(plugin)
    plugin.dispatch("enable", {"confirm": True})
    _feed(plugin, _target(x=0.6))
    plugin._tick(token)
    ctrl.mode = 26
    plugin._tick(token)
    assert ctrl.commands[-1][2] == 0.0
    assert json.loads(_status_messages(plugin)[-1])["stop_reason"] == "protection_mode"

    ctrl.mode = 2
    plugin._tick(token)
    assert ctrl.commands[-1][2] != 0.0
    plugin._on_estop(_String('{"latched": true}'))
    plugin._tick(token)
    assert ctrl.commands[-1][2] == 0.0
    assert json.loads(_status_messages(plugin)[-1])["stop_reason"] == "estop"


def test_hysteresis_keeps_a_centred_robot_centred(card_module):
    plugin, ctrl = _make_plugin(card_module)
    token = _start(plugin)
    plugin.dispatch("enable", {"confirm": True})
    _feed(plugin, _target(x=math.tan(math.radians(8.0))))
    plugin._tick(token)
    assert ctrl.commands == []  # inside the dead band while stopped

    _feed(plugin, _target(x=math.tan(math.radians(30.0))))
    plugin._tick(token)
    assert ctrl.commands[-1][2] != 0.0

    _feed(plugin, _target(x=0.0))
    plugin._tick(token)
    plugin._tick(token)
    assert ctrl.commands[-1][2] == 0.0
    assert plugin._state == "centered"
    _feed(plugin, _target(x=math.tan(math.radians(8.0))))
    plugin._tick(token)
    assert ctrl.commands[-1][2] == 0.0  # no restart inside the band


def test_a_superseded_worker_publishes_nothing(card_module):
    plugin, ctrl = _make_plugin(card_module)
    token = _start(plugin)
    plugin.dispatch("enable", {"confirm": True})
    _feed(plugin, _target(x=0.6))
    plugin._teardown()
    plugin._tick(token)
    assert ctrl.commands == []
    assert _status_messages(plugin) == []


def test_the_spec_module_reaches_the_image():
    dockerfile = (Path(BUMI) / "Dockerfile").read_text(encoding="utf-8")
    assert "COPY visual_facing_spec.py /work/visual_facing_spec.py" in dockerfile
    assert "COPY visual_facing_card.py /work/visual_facing_card.py" in dockerfile
