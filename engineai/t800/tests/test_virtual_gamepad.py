import struct
import sys
import time
import types
import unittest
from unittest import mock
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from virtual_gamepad import MOTION_MACROS, VirtualGamepadPlugin, encode_gamepad  # noqa: E402


class FakeLcm:
    def __init__(self):
        self.messages = []

    def publish(self, channel, payload):
        self.messages.append((channel, payload))


class VirtualGamepadTests(unittest.TestCase):
    def test_encoder_matches_engineai_wire_layout(self):
        payload = encode_gamepad([1] + [0] * 11, [0.1] * 6, timestamp_us=123)
        self.assertEqual(112, len(payload))
        values = struct.unpack(">Qq12i6d", payload)
        self.assertEqual(0xAD95CC1F0C874EE5, values[0])
        self.assertEqual(123, values[1])
        self.assertEqual(1, values[2])
        self.assertAlmostEqual(0.1, values[-1])

    def test_macro_repeats_and_releases(self):
        plugin = VirtualGamepadPlugin({"rate_hz": 100.0}, "robot", None)
        plugin._lcm = FakeLcm()
        result = plugin.dispatch("macro", {"macro": "dance", "hold_seconds": 0.05})
        self.assertEqual(MOTION_MACROS["dance"], result["buttons"])
        time.sleep(0.1)
        self.assertGreaterEqual(len(plugin._lcm.messages), 2)
        released = struct.unpack(">Qq12i6d", plugin._lcm.messages[-1][1])
        self.assertEqual((0,) * 12, released[2:14])

    def test_rejects_unknown_button_and_clamps_analogs(self):
        plugin = VirtualGamepadPlugin({}, "robot", None)
        plugin._lcm = FakeLcm()
        self.assertIn("error", plugin.dispatch("press", {"buttons": ["NOPE"]}))
        result = plugin.dispatch("command", {"buttons": [], "analogs": [2, -2, 0, 0, 0, 0], "duration": -1})
        self.assertEqual([1.0, -1.0, 0.0, 0.0, 0.0, 0.0], result["analogs"])
        plugin.dispatch("release", {})

    def test_start_reports_lcm_initialization_failure_to_bundle(self):
        plugin = VirtualGamepadPlugin({}, "robot", None)
        fake_lcm = types.SimpleNamespace(LCM=mock.Mock(side_effect=OSError("no multicast route")))
        with mock.patch.dict(sys.modules, {"lcm": fake_lcm}):
            with self.assertRaisesRegex(RuntimeError, "no multicast route"):
                plugin.start()
        self.assertEqual("unavailable", plugin.dispatch("status", {})["state"])


if __name__ == "__main__":
    unittest.main()
