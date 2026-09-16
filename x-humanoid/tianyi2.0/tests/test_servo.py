"""Tianyi's streaming control card — arms and hands as one action vector.

`ControlSink` is tested on its own in tests/test_control_sink.py. This file
covers only what is Tianyi-specific, which is where a Tianyi-shaped mistake
would live rather than a protocol-shaped one:

  - the 26-dimension layout, and that its limits come from the *same* vendor
    constants the call-shaped `arm`/`hand` tools enforce
  - left and right arms genuinely differ, so a mirrored descriptor would
    authorise the wrong half of each range
  - the hand's polarity is inverted at the wire — 0 open here, 1.0 open there,
    and backwards means "open" becomes "clench"
  - commands arrive on domain 42 and the robot listens on domain 0
  - silence holds; it does not release whatever is being carried

No ROS, no robot: rclpy and the vendor messages are stubbed, publishers record.

Run: python3 -m pytest x-humanoid/tianyi2.0/tests/test_servo.py -q
"""

from __future__ import annotations

import math
import sys
import types
from pathlib import Path

import pytest

DRIVER = Path(__file__).resolve().parents[1]
ROOT = DRIVER.parents[1]
sys.path.insert(0, str(ROOT))

# Two things this file must not do to the rest of the suite, both learned the
# hard way by breaking it:
#
#  1. **Not put the driver directory on sys.path, and not leave `device` bound.**
#     Two bundles here have a `device.py` and a `servo.py` and import their
#     siblings by bare name, so a path entry or a lingering `sys.modules['device']`
#     makes one driver's test load the other driver's module — which surfaced as
#     `cannot import name '_RATED_MOTOR_CURRENT_A' from 'realman_rm75_device'`,
#     reading as a broken module rather than as two tests colliding.
#
#  2. **Not leave the ROS stubs installed.** `device.py` cannot be imported
#     without an rclpy, but a fake one left in sys.modules is inherited by every
#     test file that runs afterwards — it took out tianyi's own camera-lifecycle
#     suite and realman's servo suite, neither of which has anything to do with
#     this card.
#
# So: stub, load under unique names, restore immediately (the `from … import`
# bindings are already made by then), and put the stubs back only for the
# duration of each test, since servo.py imports rclpy and the vendor messages
# lazily inside the methods that use them.

_STUBBED = ("rclpy", "rclpy.node", "rclpy.qos", "std_msgs", "std_msgs.msg",
            "sensor_msgs", "sensor_msgs.msg", "bodyctrl_msgs", "bodyctrl_msgs.msg",
            "device")


def _snapshot(names=_STUBBED):
    return {name: sys.modules.get(name) for name in names}


def _restore(saved):
    for name, module in saved.items():
        if module is None:
            sys.modules.pop(name, None)
        else:
            sys.modules[name] = module


def _exec_module(path: Path, name: str):
    module = types.ModuleType(name)
    module.__file__ = str(path)
    sys.modules[name] = module
    exec(compile(path.read_text(encoding="utf-8"), str(path), "exec"),
         module.__dict__)
    return module


def _stub_ros():
    """Enough of rclpy and the vendor messages to import device.py and servo.py."""
    rclpy = types.ModuleType("rclpy")
    rclpy.node = types.ModuleType("rclpy.node")
    rclpy.qos = types.ModuleType("rclpy.qos")

    class Node:
        def __init__(self, *args, **kwargs):
            self.subscriptions_made = []
            self.timers = []

        def create_publisher(self, *args, **kwargs):
            return _Publisher(args[1] if len(args) > 1 else "")

        def create_subscription(self, msg_type, topic, callback, qos):
            self.subscriptions_made.append(topic)

        def create_timer(self, period, callback):
            self.timers.append(period)

        def destroy_node(self):
            pass

    class QoSProfile:
        def __init__(self, **kwargs):
            pass

    class Enum:
        BEST_EFFORT = RELIABLE = KEEP_LAST = VOLATILE = 0

    rclpy.node.Node = Node
    rclpy.qos.QoSProfile = QoSProfile
    rclpy.qos.ReliabilityPolicy = Enum
    rclpy.qos.HistoryPolicy = Enum
    rclpy.qos.DurabilityPolicy = Enum
    sys.modules.update({"rclpy": rclpy, "rclpy.node": rclpy.node,
                        "rclpy.qos": rclpy.qos})

    std_msgs = types.ModuleType("std_msgs")
    std_msgs.msg = types.ModuleType("std_msgs.msg")
    for name in ("String", "Bool", "UInt32MultiArray"):
        setattr(std_msgs.msg, name, type(name, (), {}))
    sys.modules.update({"std_msgs": std_msgs, "std_msgs.msg": std_msgs.msg})

    sensor = types.ModuleType("sensor_msgs")
    sensor.msg = types.ModuleType("sensor_msgs.msg")

    class JointState:
        def __init__(self):
            self.name = []
            self.position = []

    sensor.msg.JointState = JointState
    sys.modules.update({"sensor_msgs": sensor, "sensor_msgs.msg": sensor.msg})

    body = types.ModuleType("bodyctrl_msgs")
    body.msg = types.ModuleType("bodyctrl_msgs.msg")

    class SetMotorPosition:
        def __init__(self):
            self.name = 0
            self.pos = 0.0
            self.spd = 0.0
            self.cur = 0.0

    class CmdSetMotorPosition:
        def __init__(self):
            self.cmds = []

    body.msg.SetMotorPosition = SetMotorPosition
    body.msg.CmdSetMotorPosition = CmdSetMotorPosition
    sys.modules.update({"bodyctrl_msgs": body, "bodyctrl_msgs.msg": body.msg})
    return Node


class _Publisher:
    def __init__(self, topic=""):
        self.topic = topic
        self.published = []

    def publish(self, message):
        self.published.append(message)


_SAVED = _snapshot()
try:
    _stub_ros()
    device_mod = _exec_module(DRIVER / "device.py", "tianyi_device_for_servo_test")
    sys.modules["device"] = device_mod          # servo.py: from device import …
    servo_mod = _exec_module(DRIVER / "servo.py", "tianyi_servo_under_test")
finally:
    _restore(_SAVED)

ArmPlugin = device_mod.ArmPlugin
HandPlugin = device_mod.HandPlugin
_RATED_MOTOR_CURRENT_A = device_mod._RATED_MOTOR_CURRENT_A

from common.control import ControlSink, Verdict, parse_descriptor  # noqa: E402


@pytest.fixture(autouse=True)
def ros_stubs():
    """Stubs for the duration of one test only.

    servo.py imports rclpy and the vendor messages inside the methods that use
    them, so they have to be present while a test runs — and absent the rest of
    the time, or every test file that follows inherits a fake ROS.
    """
    saved = _snapshot()
    _stub_ros()
    sys.modules["device"] = device_mod
    try:
        yield
    finally:
        _restore(saved)


class FakeROS2:
    """The driver's two contexts, so a card can get them the wrong way round."""

    def __init__(self):
        self.ctx_core = "domain42"
        self.ctx_tianyi = "domain0"
        self.core_nodes = []
        self.tianyi_nodes = []
        self.executor_core = types.SimpleNamespace(
            add_node=self.core_nodes.append,
            remove_node=lambda node: None)
        self.executor_tianyi = types.SimpleNamespace(
            add_node=self.tianyi_nodes.append,
            remove_node=lambda node: None)


def make_plugin(**config):
    return servo_mod.TianyiServoPlugin(config, "tianyi", FakeROS2())


def wired_plugin(**config):
    """A plugin with its publishers in place, without going through ROS."""
    plugin = make_plugin(**config)
    plugin._arm_pub = _Publisher("/arm/cmd_pos")
    plugin._left_hand_pub = _Publisher("/inspire_hand/ctrl/left_hand")
    plugin._right_hand_pub = _Publisher("/inspire_hand/ctrl/right_hand")
    return plugin


# ── the action space ─────────────────────────────────────────────────────────

def test_the_vector_is_arms_then_hands():
    d = servo_mod.build_descriptor()
    assert d["dof"] == 26
    assert d["joint_names"][0] == "left_shoulder_pitch"
    assert d["joint_names"][7] == "right_shoulder_pitch"
    assert d["joint_names"][14] == "left_little"
    assert d["joint_names"][20] == "right_little"


def test_groups_tile_the_vector_and_name_their_units():
    """One `units` mapping cannot say radians here and normalised there."""
    parsed = parse_descriptor(servo_mod.build_descriptor())
    assert [g.name for g in parsed.groups] == ["arm_l", "arm_r", "hand_l", "hand_r"]
    assert [g.unit for g in parsed.groups] == ["rad", "rad", "normalized", "normalized"]
    assert sum(g.count for g in parsed.groups) == 26
    assert parsed.resources == ("arm_l", "arm_r", "hand_l", "hand_r")


def test_arm_limits_come_from_the_same_constants_the_arm_tool_enforces():
    """Two paths to the same motors must not disagree about their limits."""
    d = servo_mod.build_descriptor()
    for i, (low, high) in enumerate(ArmPlugin._LEFT_POSE_LIMITS):
        assert d["limits"]["lower"][i] == pytest.approx(math.radians(low))
        assert d["limits"]["upper"][i] == pytest.approx(math.radians(high))
    for i, (low, high) in enumerate(ArmPlugin._RIGHT_POSE_LIMITS):
        assert d["limits"]["lower"][7 + i] == pytest.approx(math.radians(low))
        assert d["limits"]["upper"][7 + i] == pytest.approx(math.radians(high))


def test_the_two_arms_are_not_mirrored():
    """Shoulder roll is (-15,150) left and (-150,15) right — asymmetric on purpose."""
    d = servo_mod.build_descriptor()
    assert d["limits"]["lower"][1] != d["limits"]["lower"][8]
    assert d["limits"]["upper"][1] != d["limits"]["upper"][8]


def test_hands_are_normalised_closure():
    d = servo_mod.build_descriptor()
    assert d["limits"]["lower"][14:] == [0.0] * 12
    assert d["limits"]["upper"][14:] == [1.0] * 12


def test_finger_names_follow_the_hand_tools_order():
    d = servo_mod.build_descriptor()
    assert d["joint_names"][14:20] == [f"left_{n}" for n in HandPlugin._FINGER_NAMES]


def test_force_torque_is_declared_null_not_omitted():
    d = servo_mod.build_descriptor()
    assert "force_torque" in d and d["force_torque"] is None


def test_the_descriptor_parses():
    parsed = parse_descriptor(servo_mod.build_descriptor())
    assert parsed.mode == "joint_position"
    assert parsed.dof == 26


def test_a_rate_above_the_ceiling_is_refused():
    with pytest.raises(ValueError):
        servo_mod.TianyiServoPlugin({"expected_hz": 500}, "tianyi", FakeROS2())


# ── the tool ─────────────────────────────────────────────────────────────────

def test_the_tool_holds_all_four_channels():
    tool = make_plugin().get_tool()
    assert tool["type"] == "actuator"
    assert tool["topic_in"][0]["format"] == "control/joint"
    assert tool["inputSchema"]["x-resource"] == ["arm_l", "arm_r", "hand_l", "hand_r"]


def test_no_completion_is_declared():
    """A stream has no end; an open pending would block every other actuator."""
    assert "x-completion" not in make_plugin().get_tool()["inputSchema"]


def test_both_interrupt_hooks_stop_it():
    hooks = make_plugin().get_tool()["inputSchema"]["x-hooks"]
    assert hooks["on_interrupt_all"]["action"] == "stop"
    assert hooks["on_interrupt_motion"]["action"] == "stop"


# ── refusals ─────────────────────────────────────────────────────────────────

def test_start_requires_explicit_confirmation():
    result = make_plugin().dispatch("start", {"input_topic": "/x",
                                              "confirm_motion": False})
    assert result["state"] == "error"
    assert "confirm_motion" in result["message"]


def test_start_refuses_without_an_input_topic():
    result = make_plugin().dispatch("start", {"confirm_motion": True})
    assert result["state"] == "error"
    assert "input_topic" in result["message"]


def test_bundle_start_does_not_begin_streaming():
    """A restarted container must not come up moving a humanoid."""
    plugin = make_plugin()
    plugin.start()
    assert plugin.dispatch("info", {})["state"] == "idle"


# ── the two domains ──────────────────────────────────────────────────────────

def test_commands_are_taken_on_core_and_published_on_tianyi():
    """Subscribing on the wrong domain gives a card that never hears anything."""
    plugin = make_plugin()
    plugin._open("/actucore/vla/cmd")

    ros2 = plugin._ros2
    assert len(ros2.core_nodes) == 1 and len(ros2.tianyi_nodes) == 1
    assert ros2.core_nodes[0].subscriptions_made == ["/actucore/vla/cmd"]
    # And the watchdog has a heartbeat of its own, since silence produces no
    # callbacks and silence is what it exists to notice.
    assert ros2.core_nodes[0].timers


# ── radians in, radians out; closure inverted ────────────────────────────────

def test_arm_commands_carry_radians_motor_ids_and_rated_current():
    plugin = wired_plugin()
    left = [0.1] * 7
    right = [-0.2] * 7

    plugin._publish_arms(left, right)

    message = plugin._arm_pub.published[0]
    assert len(message.cmds) == 14
    assert [c.name for c in message.cmds] == list(range(11, 18)) + list(range(21, 28))
    assert message.cmds[0].pos == pytest.approx(0.1)       # no unit conversion
    assert message.cmds[7].pos == pytest.approx(-0.2)
    assert message.cmds[0].cur == _RATED_MOTOR_CURRENT_A[11]


def test_hand_polarity_is_inverted_at_the_wire():
    """0 is open here and 1.0 is open there; backwards turns open into clench."""
    plugin = wired_plugin()

    plugin._publish_hand(plugin._left_hand_pub, [0.0] * 6)      # fully open
    assert plugin._left_hand_pub.published[0].position == [1.0] * 6

    plugin._publish_hand(plugin._left_hand_pub, [1.0] * 6)      # fully closed
    assert plugin._left_hand_pub.published[1].position == [0.0] * 6


def test_hand_message_names_the_six_fingers_by_index():
    plugin = wired_plugin()
    plugin._publish_hand(plugin._left_hand_pub, [0.5] * 6)
    assert plugin._left_hand_pub.published[0].name == ["1", "2", "3", "4", "5", "6"]


def test_one_command_drives_both_arms_and_both_hands():
    """The model emits one instant; splitting it across cards would not."""
    plugin = wired_plugin()
    plugin._apply([0.0] * 14 + [0.5] * 12, None)

    assert len(plugin._arm_pub.published) == 1
    assert len(plugin._left_hand_pub.published) == 1
    assert len(plugin._right_hand_pub.published) == 1


# ── end to end through the sink ──────────────────────────────────────────────

def _message(values, clock, seq=1):
    return {"schema": "motus.control/1", "seq": seq, "stamp_ms": clock,
            "obs_stamp_ms": clock, "ttl_ms": 100, "source": "test",
            "priority": 50, "mode": "joint_position", "dof": 26,
            "values": list(values)}


def test_an_out_of_range_shoulder_never_reaches_the_robot():
    plugin = wired_plugin()
    clock = [1_000_000]
    sink = ControlSink(servo_mod.build_descriptor(), plugin._apply,
                       clock=lambda: clock[0])

    beyond = math.radians(ArmPlugin._LEFT_POSE_LIMITS[1][1] + 10)
    values = [0.0] * 26
    values[1] = beyond

    outcome = sink.submit(_message(values, clock[0]))

    assert outcome.verdict is Verdict.REJECTED
    assert plugin._arm_pub.published == []          # not one joint got through


def test_a_finger_beyond_full_closure_is_rejected():
    plugin = wired_plugin()
    clock = [1_000_000]
    sink = ControlSink(servo_mod.build_descriptor(), plugin._apply,
                       clock=lambda: clock[0])

    values = [0.0] * 26
    values[14] = 1.5                                 # closure only runs 0..1

    assert sink.submit(_message(values, clock[0])).verdict is Verdict.REJECTED
    assert plugin._left_hand_pub.published == []


def test_a_valid_command_reaches_every_publisher():
    plugin = wired_plugin()
    clock = [1_000_000]
    sink = ControlSink(servo_mod.build_descriptor(), plugin._apply,
                       clock=lambda: clock[0])

    outcome = sink.submit(_message([0.0] * 14 + [0.2] * 12, clock[0]))

    assert outcome.applied
    assert plugin._arm_pub.published
    assert plugin._left_hand_pub.published[0].position == pytest.approx([0.8] * 6)


# ── silence ──────────────────────────────────────────────────────────────────

def test_silence_holds_rather_than_dropping_what_is_being_carried():
    """Releasing would drop the payload onto whatever is underneath."""
    plugin = wired_plugin()
    plugin._hold()
    assert plugin._left_hand_pub.published == []
    assert plugin._right_hand_pub.published == []


def test_releasing_on_silence_is_available_but_opt_in():
    plugin = wired_plugin(release_hands_on_watchdog=True)
    plugin._hold()
    assert plugin._left_hand_pub.published[0].position == [1.0] * 6   # open
    assert plugin._right_hand_pub.published[0].position == [1.0] * 6
