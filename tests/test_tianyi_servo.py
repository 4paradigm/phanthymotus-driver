"""The Tianyi servo card — bimanual continuous control, 26 dimensions.

`ControlSink` is tested on its own in test_control_sink.py. This file covers
what is Tianyi-shaped, which is where a Tianyi-shaped mistake would be:

  - the two ways an operator can authorise motion, and that both are needed
    because the card is started two ways
  - the descriptor's own shape, since a wrong width is caught once here rather
    than once per command at 30 Hz

No robot, no ROS: `rclpy` and the message packages are stubbed, which is enough
because device.py only imports four of them at module level.

Run: PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest tests/test_tianyi_servo.py -q
"""

from __future__ import annotations

import importlib.util
import sys
import types
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
DRIVER = ROOT / "x-humanoid" / "tianyi2.0"
sys.path.insert(0, str(ROOT))


def _stub_ros():
    """The four module-level ROS imports in device.py, and nothing more.

    Deliberately minimal: a stub that grows to cover every message type is a
    second implementation of ROS that can drift from the real one, and these
    tests do not publish anything.
    """
    modules = {}

    rclpy = types.ModuleType("rclpy")
    node = types.ModuleType("rclpy.node")
    node.Node = type("Node", (), {"__init__": lambda self, *a, **k: None})
    qos = types.ModuleType("rclpy.qos")
    for name in ("QoSProfile", "ReliabilityPolicy", "HistoryPolicy",
                 "DurabilityPolicy"):
        setattr(qos, name, type(name, (), {
            "__init__": lambda self, *a, **k: None,
            "RELIABLE": 1, "BEST_EFFORT": 2, "KEEP_LAST": 1, "VOLATILE": 1,
            "TRANSIENT_LOCAL": 2,
        }))
    rclpy.node, rclpy.qos = node, qos
    std = types.ModuleType("std_msgs")
    std_msg = types.ModuleType("std_msgs.msg")
    for name in ("String", "Bool", "UInt32MultiArray"):
        setattr(std_msg, name, type(name, (), {}))
    std.msg = std_msg

    modules.update({"rclpy": rclpy, "rclpy.node": node, "rclpy.qos": qos,
                    "std_msgs": std, "std_msgs.msg": std_msg})
    return modules


@pytest.fixture(scope="module")
def servo():
    """Import the card with ROS stubbed, and put every module back afterwards.

    The snapshot/restore matters: several drivers in this repo have a file
    called `device.py`, so a leaked `sys.modules['device']` makes a *different*
    test file import this robot's driver and fail somewhere unrelated.
    """
    saved = dict(sys.modules)
    sys.modules.update(_stub_ros())
    try:
        def load(name, filename):
            spec = importlib.util.spec_from_file_location(name, DRIVER / filename)
            module = importlib.util.module_from_spec(spec)
            sys.modules[name] = module
            spec.loader.exec_module(module)
            return module

        device = load("tianyi_device", "device.py")
        sys.modules["device"] = device          # servo.py imports `device`
        yield load("tianyi_servo", "servo.py")
    finally:
        sys.modules.clear()
        sys.modules.update(saved)


def make_plugin(servo, **config):
    return servo.TianyiServoPlugin(config, namespace="nvidia_desktop", ros2=None)


# ── pause is the model's lever, not a start gate ─────────────────────────────

def test_pause_stops_applying_without_dropping_the_subscription(servo):
    """`stop` is not an answer to "wait": only agent-core knows the input topic
    and the downstream descriptor, so a model that stopped this card could not
    start it again."""
    plugin = make_plugin(servo)
    plugin._running = True
    plugin._input_topic = "/x"

    assert plugin.dispatch("pause", {})["state"] == "paused"
    assert plugin._input_topic == "/x"
    assert plugin.dispatch("info", {})["state"] == "paused"

    assert plugin.dispatch("resume", {})["state"] == "running"
    assert plugin.dispatch("info", {})["state"] == "running"


def test_a_command_arriving_during_a_pause_is_dropped_not_queued(servo):
    """Applying it at resume would be a jump computed from a stale world."""
    plugin = make_plugin(servo)
    plugin._running = True
    plugin._sink = _RefusingSink()
    plugin.dispatch("pause", {})

    plugin._on_message(_Message('{"values": [0] * 26}'))

    assert plugin._sink.submitted == []


def test_pausing_a_card_that_is_not_running_is_not_an_error(servo):
    assert make_plugin(servo).dispatch("pause", {})["state"] == "idle"


def test_start_does_not_ask_for_a_confirmation_the_canvas_cannot_give(servo):
    """`start-project` builds the start arguments itself (agent-core
    src/api/config.py `_start_and_resolve`) and sends only action, instance_id,
    input_topic and control_interface. A card gating on anything else can never
    be started the one way it is meant to be — and since a card answering
    `error` rolls the whole project back, wiring it up stopped this robot's
    ASR, camera and TTS cards too. That is what happened on the real Tianyi.
    """
    definition = make_plugin(servo).get_tool()
    properties = definition["inputSchema"]["properties"]

    assert set(properties) <= {"action", "input_topic"}
    for spec in definition["inputSchema"]["x-action-params"].values():
        assert set(spec["params"]) <= {"input_topic"}


class _Message:
    def __init__(self, data):
        self.data = data


class _RefusingSink:
    def __init__(self):
        self.submitted = []

    def submit(self, payload):
        self.submitted.append(payload)
        raise AssertionError("a paused card must not reach the sink")


# ── the action space ─────────────────────────────────────────────────────────

def test_the_descriptor_is_26_wide_and_says_which_dimension_is_which(servo):
    descriptor = servo.build_descriptor()

    assert descriptor["dof"] == 26
    assert len(descriptor["joint_names"]) == 26
    assert len(descriptor["limits"]["lower"]) == 26
    assert len(descriptor["limits"]["upper"]) == 26
    # One `units` mapping cannot say "radians here, normalised closure there".
    assert [g["name"] for g in descriptor["groups"]] == [
        "arm_l", "arm_r", "hand_l", "hand_r"]
    assert [(g["offset"], g["count"]) for g in descriptor["groups"]] == [
        (0, 7), (7, 7), (14, 6), (20, 6)]


def test_the_arms_do_not_share_limits(servo):
    """Shoulder roll is (-15, 150) left and (-150, 15) right.

    A descriptor built from one side and mirrored would authorise the wrong
    half of each range — which looks like a working robot until the arm goes
    the wrong way.
    """
    limits = servo.build_descriptor()["limits"]
    left_roll = (limits["lower"][1], limits["upper"][1])
    right_roll = (limits["lower"][8], limits["upper"][8])

    assert left_roll != right_roll
    assert left_roll[1] > 0 and right_roll[0] < 0


def test_the_hands_are_normalised_zero_to_one(servo):
    limits = servo.build_descriptor()["limits"]
    assert limits["lower"][14:] == [0.0] * 12
    assert limits["upper"][14:] == [1.0] * 12


def test_missing_force_torque_is_declared_not_omitted(servo):
    """Declared null so the absent protection is visible, not forgotten."""
    descriptor = servo.build_descriptor()
    assert "force_torque" in descriptor
    assert descriptor["force_torque"] is None
