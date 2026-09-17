"""Which way round Inspire's finger feedback reads.

The hand publishes an **open ratio**: 1.0 is open, 0.0 is closed. Three places
in the Tianyi driver depend on knowing that, and for a while one of them had it
backwards, so the `hand_state` card told the LLM the opposite of the truth —
asking "is the hand open" got "fully_closed" from a hand that was open.

Measured on a Tianyi sitting idle with both hands visibly open:

    left  [1.0, 1.0, 1.0, 0.998, 1.0, 0.977]
    right [0.993, 0.993, 1.0, 0.993, 0.996, 0.979]

These tests pin the convention at each place that encodes it, so the next
person to touch one of them finds out from a test rather than from a hand
closing around something.

Run: PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest tests/test_tianyi_hand_polarity.py -q
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

# Measured on the robot, hands open. Kept as data so the numbers in the
# docstring above and the assertions below cannot drift apart.
IDLE_OPEN_HANDS = {
    "left": [1.0, 1.0, 1.0, 0.998, 1.0, 0.977],
    "right": [0.993, 0.993, 1.0, 0.993, 0.996, 0.979],
}


@pytest.fixture(scope="module")
def device():
    saved = dict(sys.modules)
    rclpy = types.ModuleType("rclpy")
    node = types.ModuleType("rclpy.node")
    node.Node = type("Node", (), {"__init__": lambda self, *a, **k: None})
    qos = types.ModuleType("rclpy.qos")
    for name in ("QoSProfile", "ReliabilityPolicy", "HistoryPolicy",
                 "DurabilityPolicy"):
        setattr(qos, name, type(name, (), {
            "__init__": lambda self, *a, **k: None,
            "RELIABLE": 1, "BEST_EFFORT": 2, "KEEP_LAST": 1,
            "VOLATILE": 1, "TRANSIENT_LOCAL": 2,
        }))
    rclpy.node, rclpy.qos = node, qos
    std = types.ModuleType("std_msgs")
    std_msg = types.ModuleType("std_msgs.msg")
    for name in ("String", "Bool", "UInt32MultiArray"):
        setattr(std_msg, name, type(name, (), {}))
    std.msg = std_msg
    sys.modules.update({"rclpy": rclpy, "rclpy.node": node, "rclpy.qos": qos,
                        "std_msgs": std, "std_msgs.msg": std_msg})
    try:
        spec = importlib.util.spec_from_file_location("tianyi_device_polarity",
                                                      DRIVER / "device.py")
        module = importlib.util.module_from_spec(spec)
        sys.modules["tianyi_device_polarity"] = module
        spec.loader.exec_module(module)
        yield module
    finally:
        sys.modules.clear()
        sys.modules.update(saved)


def test_the_robots_own_idle_reading_is_called_open(device):
    """The reading that started this: an idle robot with open hands."""
    for side, positions in IDLE_OPEN_HANDS.items():
        for finger, value in enumerate(positions, start=1):
            label = device._hand_position_label(value)
            assert label == "fully_open", f"{side} finger {finger} = {value}"


def test_zero_is_closed_and_one_is_open(device):
    assert device._hand_position_label(1.0) == "fully_open"
    assert device._hand_position_label(0.0) == "fully_closed"


def test_the_labels_run_in_one_direction(device):
    """Monotonic, so no threshold can be edited into an island."""
    order = ["fully_closed", "almost_closed", "half_closed",
             "almost_open", "fully_open"]
    seen = [device._hand_position_label(v / 100) for v in range(0, 101)]
    ranks = [order.index(label) for label in seen]
    assert ranks == sorted(ranks)


def test_the_tool_description_states_the_same_convention(device):
    """The description is what the LLM reads; a correct number under a wrong
    sentence is still a wrong answer."""
    plugin = device.HandStatePlugin.__new__(device.HandStatePlugin)
    plugin._topic = "/x/state/hand"
    description = device.HandStatePlugin.get_tool(plugin)["description"]

    assert "1=open 0=closed" in description


def test_the_command_path_still_inverts_on_the_way_out(device):
    """The other half of the convention, and the dangerous half.

    `hand` takes 0-100 where 100 is closed; the wire takes 1.0 for open. If
    this ever stops inverting, "open the hand" becomes "clench", which around
    an object is the direction that breaks something.
    """
    for angle, expected in ((0, 1.0), (100, 0.0), (50, 0.5)):
        assert (100 - angle) / 100.0 == expected


def test_the_skeleton_reads_the_same_feedback_as_an_open_ratio(device):
    """Anchors the correction: this one was always right, which is why the
    dashboard's hands have rendered correctly all along."""
    wide_open = device._skeleton_hand_bend_rad(1.0, "left", "index")
    clenched = device._skeleton_hand_bend_rad(0.0, "left", "index")

    assert wide_open == 0.0            # open ratio 1.0 → no bend
    assert clenched > wide_open        # open ratio 0.0 → fully bent
