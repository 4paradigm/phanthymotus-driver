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
    properties = make_plugin(servo).get_tool()["inputSchema"]["properties"]

    assert set(properties) <= {"action", "input_topic"}


def test_the_model_is_given_pause_and_resume_and_not_stop(servo):
    """agent-core splits a tool into one LLM-callable function per
    `x-action-params` entry (mcp_client.py `_to_openai_schema`), so this list
    *is* the model's reach. `stop` belongs to the project lifecycle: a model
    calling it would take the card out of a running project without the project
    knowing, and could not put it back — `start` needs the input topic and the
    downstream descriptor that only agent-core has.
    """
    schema = make_plugin(servo).get_tool()["inputSchema"]

    assert set(schema["x-action-params"]) == {"pause", "resume"}
    # Still dispatchable by the canvas, just not offered to the model.
    assert {"start", "stop"} <= set(schema["properties"]["action"]["enum"])


def test_the_interrupt_hooks_pause_rather_than_tear_the_card_down(servo):
    """A framework interrupt should stop the arms, not unwire them."""
    hooks = make_plugin(servo).get_tool()["inputSchema"]["x-hooks"]

    assert hooks["on_interrupt_motion"]["action"] == "pause"
    assert hooks["on_interrupt_all"]["action"] == "pause"


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


# ── the state this card reports ──────────────────────────────────────────────

def _feed(plugin, servo, *, arms=0.0, left=None, right=None):
    """Push one round of body feedback in, the way the robot's own topics do."""
    class _Motor:
        def __init__(self, name, pos):
            self.name, self.pos = name, pos

    class _Status:
        def __init__(self, status):
            self.status = status

    motors = []
    for side, base in servo.ARM_MOTOR_BASE.items():
        for offset in range(7):
            motors.append(_Motor(base + offset, arms))
    plugin._on_arm_status(_Status(motors))

    class _JointState:
        def __init__(self, position):
            self.name = [str(i + 1) for i in range(6)]
            self.position = position

    plugin._on_hand_state("left", _JointState(left if left is not None else [1.0] * 6))
    plugin._on_hand_state("right", _JointState(right if right is not None else [1.0] * 6))


def test_the_state_is_the_same_26_dimensions_as_the_commands(servo):
    """One list defines both, which is the point of putting state on this card.

    A separate card would have to be kept in step by hand; here the order, the
    units and the polarity cannot drift because they are the same object.
    """
    plugin = make_plugin(servo)
    _feed(plugin, servo)

    values = plugin.state_vector()

    assert len(values) == servo.build_descriptor()["dof"] == 26
    assert plugin.get_tool()["topic_out"][0]["format"] == "state/joint"


def test_the_hands_report_closure_the_way_the_descriptor_defines_it(servo):
    """Inspire feeds an **open ratio** (1.0 open); the descriptor wants closure
    (0 open). Getting this backwards tells a policy the hand is shut when it is
    open — and around an object, closing is the direction that breaks things."""
    plugin = make_plugin(servo)
    _feed(plugin, servo, left=[1.0] * 6, right=[0.0] * 6)

    values = plugin.state_vector()

    assert values[servo.LEFT_HAND] == [0.0] * 6      # open ratio 1.0 → closure 0
    assert values[servo.RIGHT_HAND] == [1.0] * 6     # open ratio 0.0 → closure 1


def test_state_and_command_are_inverses_of_each_other(servo):
    """The two conversions live in one file so they can be checked together.

    `_publish_hand` writes `1 - closure` to the wire; the feedback path reads
    `1 - position` back. A round trip must be the identity, or the robot's idea
    of where it is and where it was told to go drift apart silently.
    """
    plugin = make_plugin(servo)
    for closure in (0.0, 0.25, 0.5, 1.0):
        on_the_wire = 1.0 - closure          # what _publish_hand sends
        _feed(plugin, servo, left=[on_the_wire] * 6, right=[on_the_wire] * 6)
        assert plugin.state_vector()[servo.LEFT_HAND] == pytest.approx([closure] * 6)


def test_a_missing_channel_reports_nothing_rather_than_zeros(servo):
    """Zero is a plausible reading in both units — arms straight, hands open —
    so padding would hand a policy an invented world it cannot question."""
    plugin = make_plugin(servo)

    assert plugin.state_vector() is None        # nothing at all yet

    class _JointState:
        name = [str(i + 1) for i in range(6)]
        position = [1.0] * 6

    plugin._on_hand_state("left", _JointState())
    assert plugin.state_vector() is None        # arms and right hand still absent


def test_the_arms_are_read_from_the_motor_ids_the_commands_use(servo):
    """11..17 and 21..27 — the same base ids `_publish_arms` writes to."""
    plugin = make_plugin(servo)
    assert servo.ARM_MOTOR_BASE == {"left": 11, "right": 21}

    _feed(plugin, servo, arms=0.5)
    values = plugin.state_vector()

    assert values[servo.LEFT_ARM] == pytest.approx([0.5] * 7)
    assert values[servo.RIGHT_ARM] == pytest.approx([0.5] * 7)


def test_the_state_topic_is_declared_so_a_feedback_loop_can_resolve(servo):
    """In vla → servo →(state)→ vla neither card can learn its input from a
    source that has already started, so the declaration is all that is left."""
    port = make_plugin(servo).get_tool()["topic_out"][0]
    assert port["topic"] == "/nvidia_desktop/servo/state"
