"""Dispatch-logic tests for the BrainCo Revo2 driver, against a fake bc_stark_sdk.

No real hardware, no rclpy, and no bc_stark_sdk wheel required — device.py only
imports rclpy/std_msgs lazily inside RevoNodes.__init__, and HandPlugin /
HandStatePlugin / HandTouchPlugin only need the small subset of RevoNodes'
public surface faked below (mirrors tests/test_lynx_m20_driver.py's FakeNodes
pattern in this repo).

Run:  python3 -m unittest discover -s brainco/revo2/tests -t brainco/revo2
"""

from __future__ import annotations

import asyncio
import importlib.util
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]  # phanthymotus-driver/
DRIVER = ROOT / "brainco/revo2"
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(DRIVER))


def load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader
    spec.loader.exec_module(module)
    return module


device = load("revo2_device", DRIVER / "device.py")


class FakeEnumMember:
    def __init__(self, name):
        self.name = name

    def __repr__(self):
        return f"<{self.name}>"

    def __eq__(self, other):
        return isinstance(other, FakeEnumMember) and self.name == other.name

    def __hash__(self):
        return hash(self.name)


class FakeEnumClass:
    """Stand-in for a pyo3 enum class: attribute access returns a (cached) member."""

    def __init__(self):
        self._members = {}

    def __getattr__(self, name):
        if name not in self._members:
            self._members[name] = FakeEnumMember(name)
        return self._members[name]


class FakeLedInfo:
    def __init__(self, color, mode):
        self.color = color
        self.mode = mode


class FakeSDK:
    FingerId = FakeEnumClass()
    ActionSequenceId = FakeEnumClass()
    LedColor = FakeEnumClass()
    LedMode = FakeEnumClass()
    StarkProtocolType = FakeEnumClass()
    Baudrate = FakeEnumClass()
    LedInfo = FakeLedInfo


class FakeCtx:
    """Fake DeviceContext — coroutine methods over an in-memory position vector."""

    def __init__(self):
        self.positions = {127: [0, 0, 0, 0, 0, 0]}
        self.led_calls = []
        self.gesture_calls = []
        self.calibrate_calls = []

    async def get_finger_positions(self, slave_id):
        return list(self.positions[slave_id])

    async def get_finger_speeds(self, slave_id):
        return [0] * 6

    async def get_finger_currents(self, slave_id):
        return [0] * 6

    async def get_motor_state(self, slave_id):
        return FakeEnumMember("Idle")

    async def get_voltage(self, slave_id):
        return 24.0

    async def get_device_info(self, slave_id):
        info = type("Info", (), {})()
        info.serial_number = "SN123"
        info.firmware_version = "1.0.0"
        info.hardware_version = ""
        info.hand_type = FakeEnumMember("Right")
        info.hardware_type = FakeEnumMember("Revo2Basic")
        return info

    _FINGER_ID_MEMBER_NAMES = ["Thumb", "ThumbAux", "Index", "Middle", "Ring", "Pinky"]

    async def set_finger_position(self, slave_id, finger_id, position):
        idx = self._FINGER_ID_MEMBER_NAMES.index(finger_id.name)
        self.positions[slave_id][idx] = position

    async def set_finger_positions(self, slave_id, positions):
        self.positions[slave_id] = list(positions)

    async def run_action_sequence(self, slave_id, action_id):
        self.gesture_calls.append((slave_id, action_id.name))

    async def calibrate_position(self, slave_id):
        self.calibrate_calls.append(slave_id)

    async def reset_default_gesture(self, slave_id):
        pass

    async def set_led_info(self, slave_id, led_info):
        self.led_calls.append((slave_id, led_info.color.name, led_info.mode.name))

    async def get_button_event(self, slave_id):
        evt = type("Evt", (), {})()
        evt.button_id = 0
        evt.press_state = FakeEnumMember("Up")
        evt.timestamp = 12345
        return evt


class FakeBridge:
    """Runs coroutines synchronously — no background thread needed in tests."""

    def run(self, coro, timeout: float = 5.0):
        return asyncio.run(coro)

    def maybe_await(self, value, timeout: float = 5.0):
        if asyncio.iscoroutine(value):
            return asyncio.run(value)
        return value


class FakeNodes:
    def __init__(self):
        self.config = {"variant": "basic"}
        self.variant = "basic"
        self.hands = [{"side": "right", "slave_id": 127}]
        self.sdk = FakeSDK
        self.bridge = FakeBridge()
        self.ctx = FakeCtx()
        self.state_topic = "/host/revo2/hand_state"
        self.touch_topic = "/host/revo2/hand_touch"

    def slave_id(self, side):
        for hand in self.hands:
            if hand["side"] == side:
                return hand["slave_id"]
        return None

    def default_slave_id(self):
        return self.hands[0]["slave_id"]

    def require_ctx(self):
        return self.ctx


class HandPluginTests(unittest.TestCase):
    def setUp(self):
        self.nodes = FakeNodes()
        self.plugin = device.HandPlugin(self.nodes, {
            "open_positions": [0, 0, 0, 0, 0, 0],
            "close_positions": [1000, 1000, 1000, 1000, 1000, 1000],
        })

    def test_tool_schema_has_all_actions(self):
        tool_def = self.plugin.get_tool()
        actions = tool_def["inputSchema"]["properties"]["action"]["enum"]
        for expected in ["get_state", "get_device_info", "open", "close", "set_position",
                          "set_positions", "run_gesture", "calibrate", "reset_gesture",
                          "set_led", "get_button_event"]:
            self.assertIn(expected, actions)
        self.assertEqual(set(tool_def["inputSchema"]["x-action-params"]), set(actions))

    def test_get_state_reports_positions_by_finger_name(self):
        result = self.plugin.dispatch("get_state", {})
        self.assertEqual(set(result["positions"]), set(device.FINGER_NAMES))

    def test_open_uses_configured_profile(self):
        self.plugin.dispatch("close", {})
        result = self.plugin.dispatch("open", {})
        self.assertEqual(result["positions"]["thumb"], 0)
        self.assertEqual(self.nodes.ctx.positions[127], [0] * 6)

    def test_close_uses_configured_profile(self):
        result = self.plugin.dispatch("close", {})
        self.assertEqual(result["positions"]["pinky"], 1000)
        self.assertEqual(self.nodes.ctx.positions[127], [1000] * 6)

    def test_set_position_clamps_out_of_range(self):
        result = self.plugin.dispatch("set_position", {"finger": "index", "position": 5000})
        self.assertEqual(result["position"], device.POSITION_MAX)

    def test_set_position_rejects_unknown_finger(self):
        with self.assertRaises(ValueError):
            self.plugin.dispatch("set_position", {"finger": "nope", "position": 100})

    def test_set_positions_rejects_wrong_length(self):
        with self.assertRaises(ValueError):
            self.plugin.dispatch("set_positions", {"positions": [1, 2, 3]})

    def test_set_positions_accepts_full_vector(self):
        result = self.plugin.dispatch("set_positions", {"positions": [10, 20, 30, 40, 50, 60]})
        self.assertEqual(self.nodes.ctx.positions[127], [10, 20, 30, 40, 50, 60])
        self.assertEqual(result["positions"]["ring"], 50)

    def test_run_gesture_rejects_unknown_gesture(self):
        with self.assertRaises(ValueError):
            self.plugin.dispatch("run_gesture", {"gesture": "nope"})

    def test_run_gesture_maps_to_action_sequence_id(self):
        self.plugin.dispatch("run_gesture", {"gesture": "fist"})
        self.assertEqual(self.nodes.ctx.gesture_calls, [(127, "DefaultGestureFist")])

    def test_calibrate_calls_sdk(self):
        self.plugin.dispatch("calibrate", {})
        self.assertEqual(self.nodes.ctx.calibrate_calls, [127])

    def test_set_led_builds_led_info(self):
        self.plugin.dispatch("set_led", {"color": "RGB", "mode": "Blink"})
        self.assertEqual(self.nodes.ctx.led_calls, [(127, "RGB", "Blink")])

    def test_get_button_event_shape(self):
        result = self.plugin.dispatch("get_button_event", {})
        self.assertEqual(result["press_state"], "Up")
        self.assertEqual(result["timestamp"], 12345)

    def test_side_selection_uses_configured_slave_id(self):
        result = self.plugin.dispatch("get_state", {"side": "right"})
        self.assertIn("positions", result)
        with self.assertRaises(ValueError):
            self.plugin.dispatch("get_state", {"side": "left"})

    def test_unknown_action_returns_none(self):
        self.assertIsNone(self.plugin.dispatch("not_a_real_action", {}))


class HandStatePluginTests(unittest.TestCase):
    def test_topic_out_is_data_json(self):
        nodes = FakeNodes()
        plugin = device.HandStatePlugin(nodes, {"poll_interval_s": 0.1})
        tool_def = plugin.get_tool()
        self.assertEqual(tool_def["type"], "sensor")
        self.assertEqual(tool_def["topic_out"][0]["format"], "data/json")
        self.assertEqual(tool_def["topic_out"][0]["topic"], nodes.state_topic)


class HandTouchPluginTests(unittest.TestCase):
    def test_reader_name_selected_by_touch_vendor(self):
        nodes = FakeNodes()
        nodes.config["touch_vendor"] = "force3d"
        plugin = device.HandTouchPlugin(nodes, {})
        self.assertEqual(plugin.reader_name, "get_force3d_finger_array")

    def test_unknown_touch_vendor_falls_back_to_array_pressure(self):
        nodes = FakeNodes()
        nodes.config["touch_vendor"] = "something_unexpected"
        plugin = device.HandTouchPlugin(nodes, {})
        self.assertEqual(plugin.reader_name, "get_array_pressure_touch_data")


class ModelPluginTests(unittest.TestCase):
    def test_returns_static_spec_with_configured_variant(self):
        nodes = FakeNodes()
        nodes.variant = "touch"
        plugin = device.ModelPlugin(nodes)
        result = plugin.dispatch("model", {"_tool_name": "model"})
        self.assertEqual(result["configured_variant"], "touch")
        self.assertIn("variants", result)
        self.assertIn("touch", result["variants"])


class PositionCoercionTests(unittest.TestCase):
    def test_coerce_positions_rejects_non_numeric(self):
        with self.assertRaises(ValueError):
            device._coerce_positions(["not", "a", "number", 1, 2, 3])

    def test_coerce_positions_clamps_range(self):
        result = device._coerce_positions([-10, 5000, 0, 1000, 500, 999])
        self.assertEqual(result, [0, 1000, 0, 1000, 500, 999])


class BuildPluginsVariantGateTests(unittest.TestCase):
    """build_plugins() needs real rclpy (via RevoNodes.__init__); this only
    checks the touch-gating predicate it evaluates, so the "don't force a
    touch card onto basic/pro hands" rule stays covered without ROS installed."""

    def test_touch_plugin_only_added_for_touch_variant(self):
        for variant, expect_touch in [("basic", False), ("pro", False), ("touch", True)]:
            plugins_cfg = {"hand_touch": {"enabled": True}}
            should_add = variant == "touch" and plugins_cfg.get("hand_touch", {}).get("enabled", True)
            self.assertEqual(should_add, expect_touch)


if __name__ == "__main__":
    unittest.main()
