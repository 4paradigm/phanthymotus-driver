"""ROS-free tests for the Adam battery-card payload."""

from __future__ import annotations

import sys
import types
import unittest

sys.modules.setdefault("numpy", types.ModuleType("numpy"))

from device import _battery_payload, _StatePublisherNode, StatePlugin
from battery_status import BatteryStatusReceiver
from unittest import mock
import json
import threading


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
            "percentage": None,
            "percentage_available": False,
            "percentage_message": "The Adam DDS BMS message does not provide state of charge",
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
        self.assertIsNone(data["percentage"])
        self.assertFalse(data["percentage_available"])


class PacBatteryIntegrationTests(unittest.TestCase):
    def test_dds_fields_unchanged_even_with_conflicting_pac_electrical_values(self):
        receiver = BatteryStatusReceiver("http://localhost:8626")
        receiver._connected = True
        receiver._accept(json.dumps({"capacity":91,"percentage":100,
                                     "voltage":9999,"current":9999,"pstatus":"Normal"}))
        before = _battery_payload(_Battery(), 1234)
        after = _battery_payload(_Battery(), 1234, receiver.snapshot())
        for key in ("timestamp_ms", "voltage", "current", "power", "wh_accumulated", "status", "source_topic"):
            self.assertEqual(before[key], after[key])
        self.assertEqual(after["percentage"], 91)
        self.assertTrue(after["percentage_available"])
        receiver._connected = False
        offline = _battery_payload(_Battery(), 1234, receiver.snapshot())
        self.assertEqual(offline["voltage"], 46.2)
        self.assertIsNone(offline["percentage"])
        self.assertFalse(offline["percentage_available"])

    def test_pac_only_card_query_preserves_topic_and_does_not_reconnect(self):
        node = object.__new__(_StatePublisherNode)
        node._lock = threading.Lock()
        node._latest_state = None
        node._latest_state_at_ms = None
        node._topic_battery = "/adam/state/battery"
        node._battery_receiver = BatteryStatusReceiver("http://localhost:8626")
        node._battery_receiver._connected = True
        node._battery_receiver._accept('{"capacity":91}')
        plugin = object.__new__(StatePlugin)
        plugin._node = node
        plugin._running = True
        for action in ("get", "info", "battery"):
            result = plugin.dispatch(action, {"_tool_name":"battery"})
            self.assertEqual(result["data"]["percentage"],91)
            self.assertIsNone(result["data"]["voltage"])
            self.assertEqual(result["topic_out"][0]["topic"],"/adam/state/battery")
        self.assertIsNone(node._battery_receiver._thread)


if __name__ == "__main__":
    unittest.main()
