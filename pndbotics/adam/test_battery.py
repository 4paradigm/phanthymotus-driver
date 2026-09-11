"""ROS-free tests for the Adam battery-card payload."""

from __future__ import annotations

import sys
import types
import unittest

sys.modules.setdefault("numpy", types.ModuleType("numpy"))

from device import _battery_payload


class _Battery:
    voltage = 46.2
    current = -3.5
    power = -161.7
    wh_accumulated = 87.25
    status = "discharging"


class BatteryPayloadTests(unittest.TestCase):
    def test_emits_the_bms_fields_and_source(self):
        data = _battery_payload(_Battery(), 1234)
        self.assertEqual(data, {
            "timestamp_ms": 1234,
            "voltage": 46.2,
            "current": -3.5,
            "power": -161.7,
            "wh_accumulated": 87.25,
            "status": "discharging",
            "source_topic": "rt/lowstate",
        })

    def test_partial_or_invalid_bms_sample_still_has_a_valid_payload(self):
        data = _battery_payload(object(), 5678)
        self.assertEqual(data["timestamp_ms"], 5678)
        self.assertEqual(data["status"], "unknown")
        self.assertEqual(data["source_topic"], "rt/lowstate")
        self.assertIsNone(data["voltage"])
        self.assertIsNone(data["current"])


if __name__ == "__main__":
    unittest.main()
