from __future__ import annotations

import importlib.util
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
DRIVER = ROOT / "realman/gripper"
sys.path.insert(0, str(ROOT))

spec = importlib.util.spec_from_file_location("realman_gripper_device", DRIVER / "device.py")
device = importlib.util.module_from_spec(spec)
assert spec.loader
spec.loader.exec_module(device)


class FakeBridge:
    def __init__(self):
        self.calls = []

    def publish(self, command, params):
        self.calls.append((command, params))
        return {"command": command, **params}


class FakeNodes:
    def __init__(self):
        self.bridge = FakeBridge()

    def close(self):
        pass


class GripperPluginTests(unittest.TestCase):
    def setUp(self):
        self.nodes = FakeNodes()
        self.plugin = device.GripperPlugin(self.nodes)

    def test_tool_schema_exposes_position_control(self):
        schema = self.plugin.get_tool()
        position = schema["inputSchema"]["properties"]["position"]

        self.assertEqual(schema["name"], "gripper")
        self.assertEqual(schema["type"], "actuator")
        self.assertEqual(position["minimum"], 0)
        self.assertEqual(position["maximum"], 1000)
        self.assertEqual(
            schema["inputSchema"]["x-action-params"]["set_position"]["params"],
            ["position"],
        )

    def test_set_position_publishes_realman_payload(self):
        result = self.plugin.dispatch("set_position", {"position": 800})

        self.assertEqual(
            self.nodes.bridge.calls,
            [("hand_follow_pos", {"hand_pos": [800]})],
        )
        self.assertEqual(result, {"command": "hand_follow_pos", "hand_pos": [800]})

    def test_position_is_clamped_to_driver_range(self):
        self.plugin.dispatch("set_position", {"position": -1})
        self.plugin.dispatch("set_position", {"position": 1001})

        self.assertEqual(self.nodes.bridge.calls[0][1]["hand_pos"], [0])
        self.assertEqual(self.nodes.bridge.calls[1][1]["hand_pos"], [1000])

    def test_invalid_position_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "position must be a number"):
            self.plugin.dispatch("set_position", {"position": "bad"})


if __name__ == "__main__":
    unittest.main()
