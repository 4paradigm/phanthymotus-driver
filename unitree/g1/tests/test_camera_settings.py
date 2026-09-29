"""Offline tests for G1 RealSense settings; no camera or DDS is opened."""

import unittest
from types import SimpleNamespace

from unitree.g1.camera_settings import CameraSettingsPlugin, RealSenseSettingsOps


class FakeSensor:
    def __init__(self):
        self.values = {
            "enable_auto_exposure": 1, "exposure": 100,
            "enable_auto_white_balance": 1, "white_balance": 4000,
            "brightness": 0,
        }
        self.writes = []

    def supports(self, option):
        return option in self.values

    def is_option_read_only(self, option):
        return False

    def get_option(self, option):
        return self.values[option]

    def get_option_range(self, option):
        return SimpleNamespace(min=0, max=10000, step=1, default=0)

    def set_option(self, option, value):
        self.writes.append((option, value))
        self.values[option] = value


class CameraSettingsTests(unittest.TestCase):
    def setUp(self):
        self.sensor = FakeSensor()
        rs = SimpleNamespace(option=SimpleNamespace(**{name: name for name in self.sensor.values}))
        self.ops = RealSenseSettingsOps(self.sensor, rs)

    def test_initialization_reads_without_writing(self):
        self.assertEqual(self.sensor.writes, [])
        self.assertTrue(self.ops.initial["values"]["auto_exposure"])

    def test_manual_exposure_disables_auto_and_reset_restores_it(self):
        result = self.ops.set({"exposure": 300})
        self.assertTrue(result["success"])
        self.assertEqual(self.sensor.writes[:2],
                         [("enable_auto_exposure", 0.0), ("exposure", 300.0)])
        self.ops.reset()
        self.assertEqual(self.sensor.values["enable_auto_exposure"], 1)

    def test_validation_rejects_out_of_range_and_wrong_boolean(self):
        with self.assertRaises(ValueError):
            self.ops.set({"brightness": 10001})
        with self.assertRaises(ValueError):
            self.ops.set({"auto_exposure": 1})
        self.assertEqual(self.sensor.writes, [])

    def test_white_balance_can_return_to_initial_auto_mode(self):
        self.ops.set({"white_balance": 5000})
        self.assertFalse(self.ops.read()["values"]["auto_white_balance"])
        self.ops.reset()
        self.assertTrue(self.ops.read()["values"]["auto_white_balance"])

    def test_card_sends_one_validated_request(self):
        class Camera:
            def request_settings(self, action, values=None):
                return {"action": action, "values": values}

        card = CameraSettingsPlugin(Camera())
        self.assertEqual(card.dispatch("set", {"exposure": 200}),
                         {"action": "set", "values": {"exposure": 200}})
        self.assertFalse(card.dispatch("set", {"exposure": 200, "brightness": 1})["success"])


if __name__ == "__main__":
    unittest.main()
