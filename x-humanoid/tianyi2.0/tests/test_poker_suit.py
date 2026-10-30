"""PokerSuitPlugin tests — no ROS, no camera, no robot.

Covers the plugin's tool declaration, lifecycle states, and the suit classifier's
behaviour on synthetic contours. The frame-grabbing path is not tested here
because it requires a live camera topic.

Run: python3 -m pytest x-humanoid/tianyi2.0/tests/test_poker_suit.py -q
"""

from __future__ import annotations

import sys
import types
from pathlib import Path

import pytest

DRIVER = Path(__file__).resolve().parents[1]
ROOT = DRIVER.parents[1]
sys.path.insert(0, str(ROOT))

# device.py cannot be imported without rclpy, so stub it like test_servo.py does.
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
    rclpy = types.ModuleType("rclpy")
    rclpy.node = types.ModuleType("rclpy.node")
    rclpy.qos = types.ModuleType("rclpy.qos")

    class Node:
        def __init__(self, *args, **kwargs):
            pass

        def create_publisher(self, *args, **kwargs):
            return None

        def create_subscription(self, *args, **kwargs):
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
    sensor.msg.JointState = type("JointState", (), {})
    sys.modules.update({"sensor_msgs": sensor, "sensor_msgs.msg": sensor.msg})

    body = types.ModuleType("bodyctrl_msgs")
    body.msg = types.ModuleType("bodyctrl_msgs.msg")
    sys.modules.update({"bodyctrl_msgs": body, "bodyctrl_msgs.msg": body.msg})


_SAVED = _snapshot()
try:
    _stub_ros()
    device_mod = _exec_module(DRIVER / "device.py", "tianyi_device_for_poker_test")
finally:
    _restore(_SAVED)

PokerSuitPlugin = device_mod.PokerSuitPlugin


class FakeROS2:
    def __init__(self):
        self.ctx_tianyi = "domain0"
        self.ctx_core = "domain42"
        self.tianyi_nodes = []
        self.core_nodes = []
        self.executor_tianyi = types.SimpleNamespace(add_node=self.tianyi_nodes.append)
        self.executor_core = types.SimpleNamespace(add_node=self.core_nodes.append)


def make_plugin(**config):
    return PokerSuitPlugin(config, "tianyi", FakeROS2())


# ── tool declaration ─────────────────────────────────────────────────────────

def test_tool_name_and_type():
    tool = make_plugin().get_tool()
    assert tool["name"] == "poker_suit"
    assert tool["type"] == "sensor"


def test_tool_declares_all_actions():
    schema = make_plugin().get_tool()["inputSchema"]
    actions = schema["properties"]["action"]["enum"]
    assert set(actions) == {"recognize", "info", "start", "stop"}


def test_tool_has_no_required_params_beyond_action():
    schema = make_plugin().get_tool()["inputSchema"]
    assert schema["required"] == ["action"]


# ── lifecycle ────────────────────────────────────────────────────────────────

def test_info_before_start_is_idle():
    result = make_plugin().dispatch("info", {})
    assert result["state"] == "idle"
    assert result["frame_available"] is False


def test_stop_before_start_is_safe():
    result = make_plugin().dispatch("stop", {})
    assert result["state"] == "idle"


def test_unknown_action_is_an_error():
    result = make_plugin().dispatch("dance", {})
    assert "error" in result


# ── suit classifier ──────────────────────────────────────────────────────────

def _contour(points):
    import numpy as np
    return np.array(points, dtype=np.int32).reshape(-1, 1, 2)


def test_diamond_shape_is_classified_as_diamond():
    # A clean four-point diamond
    diamond = _contour([[50, 0], [100, 50], [50, 100], [0, 50]])
    plugin = make_plugin()
    plugin._cv2 = __import__("cv2")
    plugin._np = __import__("numpy")
    assert plugin._classify_suit(diamond) == "diamond"


def test_heart_shape_is_classified_as_heart():
    # Rough heart outline: top notch + bottom point
    heart = _contour([
        [50, 100], [20, 80], [10, 50], [20, 20], [40, 10],
        [50, 30], [60, 10], [80, 20], [90, 50], [80, 80],
    ])
    plugin = make_plugin()
    plugin._cv2 = __import__("cv2")
    plugin._np = __import__("numpy")
    assert plugin._classify_suit(heart) == "heart"


def test_ambiguous_contour_is_rejected():
    triangle = _contour([[0, 0], [50, 100], [100, 0]])
    plugin = make_plugin()
    plugin._cv2 = __import__("cv2")
    plugin._np = __import__("numpy")
    assert plugin._classify_suit(triangle) == "unknown"


def test_suit_names_are_singular_and_localized():
    assert PokerSuitPlugin._SUITS == ("heart", "diamond", "club", "spade")
    assert PokerSuitPlugin._SUIT_LABELS["heart"] == "红桃"
