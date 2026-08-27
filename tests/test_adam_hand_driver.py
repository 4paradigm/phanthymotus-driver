from __future__ import annotations

import importlib.util
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
DEVICE = ROOT / "pnpbotics/adam/device.py"


def load_device_module():
    spec = importlib.util.spec_from_file_location("adam_hand_contract", DEVICE)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader
    spec.loader.exec_module(module)
    return module


adam = load_device_module()


class AdamHandRangeTests(unittest.TestCase):
    def test_hand_commands_are_limited_to_the_effective_hardware_range(self):
        plugin = adam.HandPlugin(
            {"hand_type": "adam_client", "position_max": 1800},
            "",
            None,
        )

        tool = plugin.get_tool()
        value_schema = tool["inputSchema"]["properties"]["value"]

        self.assertEqual(adam.HAND_POSITION_MAX, 1000)
        self.assertEqual(value_schema["minimum"], 0)
        self.assertEqual(value_schema["maximum"], 1000)
        self.assertEqual(
            plugin._open_positions,
            [1000, 1000, 1000, 1000, 1000, 0,
             1000, 1000, 1000, 1000, 1000, 0],
        )

    def test_hand_state_reports_dead_band_values_at_1000(self):
        payload = adam._hand_state_payload(
            [0, 250, 1000, 1200, 1800, -1,
             1001, 750, 1800, 1000, 1, 999],
            0,
            fresh=True,
        )

        expected = [0, 250, 1000, 1000, 1000, 0,
                    1000, 750, 1000, 1000, 1, 999]
        self.assertEqual(payload["position_max"], 1000)
        self.assertEqual(payload["position"], expected)
        self.assertEqual(payload["left"]["position"], expected[:6])
        self.assertEqual(payload["right"]["position"], expected[6:])

    def test_hand_state_card_documents_the_normalized_range(self):
        card = adam.HandStatePlugin(
            {},
            "",
            None,
            state_cache=adam.HandStateCache(),
            ros2_enabled=False,
        ).get_tool()

        self.assertIn("0-1000", card["description"])
        self.assertIn("dead band", card["description"])


if __name__ == "__main__":
    unittest.main()
