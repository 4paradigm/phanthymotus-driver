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


class StateCleanupTests(unittest.TestCase):
    def make_plugin(self):
        plugin = object.__new__(StatePlugin)
        plugin._running = True
        plugin._node = mock.Mock()
        plugin._executor = mock.Mock()
        plugin._battery_receiver = mock.Mock()
        plugin._battery_receiver.stop.side_effect = RuntimeError('PAC failure')
        plugin._poll_lifecycle_lock = threading.Lock()
        plugin._poll_stop_event = threading.Event()
        plugin._poll_thread = threading.Thread(target=plugin._poll_stop_event.wait)
        plugin._poll_thread.start()
        self.addCleanup(plugin._poll_thread.join, 2)
        self.addCleanup(plugin._poll_stop_event.set)
        return plugin

    def test_pac_failure_does_not_skip_dds_stop(self):
        plugin = self.make_plugin()
        worker = plugin._poll_thread
        with self.assertLogs('device', level='ERROR'):
            with self.assertRaisesRegex(RuntimeError, 'PAC failure'):
                plugin.stop()
        self.assertFalse(worker.is_alive())
        self.assertIsNone(plugin._poll_thread)
        self.assertFalse(plugin._running)
        plugin._node.set_active.assert_called_once_with(False)

    def test_close_attempts_all_resources_and_reports_all_failures(self):
        plugin = self.make_plugin()
        worker = plugin._poll_thread
        plugin._node.set_active.side_effect = ValueError('publisher failure')
        plugin._executor.remove_node.side_effect = ValueError('detach failure')
        plugin._node.destroy_node.side_effect = ValueError('destroy failure')
        with self.assertLogs('device', level='ERROR') as logs:
            with self.assertRaises(RuntimeError) as raised:
                plugin.close()
        self.assertFalse(worker.is_alive())
        plugin._executor.remove_node.assert_called_once_with(plugin._node)
        plugin._node.destroy_node.assert_called_once_with()
        for message in ('PAC failure', 'publisher failure', 'detach failure', 'destroy failure'):
            self.assertIn(message, str(raised.exception))
            self.assertTrue(any(message in line for line in logs.output))


if __name__ == "__main__":
    unittest.main()
