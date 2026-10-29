"""Hardware-free checks for the L20 card's known-good SocketCAN contract."""

from __future__ import annotations

import importlib.util
import sys
import types
import unittest
from pathlib import Path
from unittest.mock import patch

DRIVER = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(DRIVER))


class FakeMessage:
    def __init__(self, arbitration_id, is_extended_id, data):
        self.arbitration_id = arbitration_id
        self.is_extended_id = is_extended_id
        self.data = data


fake_can = types.ModuleType("can")
fake_can.interface = types.SimpleNamespace(Bus=None)
fake_can.Message = FakeMessage
with patch.dict(sys.modules, {"can": fake_can}):
    spec = importlib.util.spec_from_file_location("linkerhand_l20_device", DRIVER / "device.py")
    device = importlib.util.module_from_spec(spec)
    assert spec.loader
    spec.loader.exec_module(device)


class FakeBus:
    def __init__(self):
        self.sent = []
        self.closed = False

    def send(self, message, timeout):
        self.sent.append((message, timeout))

    def shutdown(self):
        self.closed = True


def valid_config():
    return {
        "connection": {"can_iface": "can1"},
        "hands": [{"side": "right", "can_id": 39}],
        "plugins": {"hand": {"enabled": True}},
    }


class LinkerHandL20DeviceTests(unittest.TestCase):
    def setUp(self):
        self.bus = FakeBus()
        self.bus_factory = patch.object(device.can.interface, "Bus", return_value=self.bus)
        self.bus_factory.start()
        self.addCleanup(self.bus_factory.stop)
        self.plugin = device.build_plugins(valid_config(), "linkerhand_l20")[0]

    def test_start_does_not_move_the_hand(self):
        self.plugin.start()
        self.assertEqual([], self.bus.sent)

    def test_open_matches_the_verified_can_frame(self):
        self.plugin.start()
        result = self.plugin.dispatch("open", {})
        message, timeout = self.bus.sent[-1]
        self.assertEqual((39, False, [2, 255, 255, 255, 255, 255]),
                         (message.arbitration_id, message.is_extended_id, message.data))
        self.assertEqual(1.0, timeout)
        self.assertEqual("command_sent", result["state"])

    def test_close_matches_the_verified_can_frame(self):
        self.plugin.start()
        result = self.plugin.dispatch("close", {})
        message, _timeout = self.bus.sent[-1]
        self.assertEqual((39, False, [2, 0, 0, 0, 0, 0]),
                         (message.arbitration_id, message.is_extended_id, message.data))
        self.assertEqual("command_sent", result["state"])

    def test_stop_closes_bus_and_returns_idle(self):
        self.plugin.start()
        result = self.plugin.dispatch("stop", {})
        self.assertTrue(self.bus.closed)
        self.assertEqual("idle", result["state"])

    def test_disabled_hand_is_not_exposed(self):
        config = valid_config()
        config["plugins"]["hand"]["enabled"] = False
        self.assertEqual([], device.build_plugins(config, "linkerhand_l20"))

    def test_unexpected_hardware_config_is_rejected(self):
        config = valid_config()
        config["connection"]["can_iface"] = "can0"
        with self.assertRaisesRegex(ValueError, "Unexpected hand hardware configuration"):
            device.build_plugins(config, "linkerhand_l20")


if __name__ == "__main__":
    unittest.main()
