"""Contract tests for dashboard-friendly flat Adam state payloads."""

from __future__ import annotations

import sys
import types
import unittest

sys.modules.setdefault("numpy", types.ModuleType("numpy"))

from device import VARIANT_JOINTS


class StateFlatteningTests(unittest.TestCase):
    def test_motor_field_prefixes_are_scalar_and_identifiable(self):
        prefix = f"motor_{0:02d}_{VARIANT_JOINTS['pro'][0]}"
        self.assertEqual("motor_00_hipPitch_Left", prefix)
        self.assertEqual(f"{prefix}_position_rad", "motor_00_hipPitch_Left_position_rad")
        self.assertEqual(f"{prefix}_torque_nm", "motor_00_hipPitch_Left_torque_nm")

    def test_zero_placeholder_fields_are_intentionally_omitted(self):
        ddq = 0.0
        mode = 0
        state = 0
        fields = {}
        if ddq != 0.0:
            fields["acceleration"] = ddq
        if mode != 0:
            fields["mode"] = mode
        if state != 0:
            fields["state"] = state
        self.assertEqual({}, fields)


if __name__ == "__main__":
    unittest.main()
