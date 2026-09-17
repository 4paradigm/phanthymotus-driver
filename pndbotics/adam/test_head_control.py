"""Contract tests for Adam's dedicated head control card."""

from __future__ import annotations

import sys
import types
import unittest

sys.modules.setdefault("numpy", types.ModuleType("numpy"))

from device import HeadControlPlugin


class _Control:
    def __init__(self):
        self.targets = None
        self._hold_q = [0.0] * 31

    def start(self):
        pass

    def stop(self):
        return {"state": "idle"}

    def dispatch(self, action, args):
        return {"state": "idle" if action == "stop" else "active"}

    def _ready_error(self):
        return None

    def _set_targets(self, targets):
        self.targets = targets

    @staticmethod
    def _joint_index(name):
        return {"neckYaw": 15, "neckPitch": 16}[name]


class HeadControlTests(unittest.TestCase):
    def test_head_schema_exposes_documented_limits(self):
        tool = HeadControlPlugin(_Control()).get_tool()
        props = tool["inputSchema"]["properties"]
        self.assertEqual(-60.0, props["yaw_deg"]["minimum"])
        self.assertEqual(60.0, props["pitch_deg"]["maximum"])

    def test_yaw_targets_lowcmd_neck_joint(self):
        control = _Control()
        result = HeadControlPlugin(control).dispatch("set_yaw", {"yaw_deg": 30})
        self.assertTrue(result["success"])
        self.assertAlmostEqual(0.5235987756, control.targets["neckYaw"])


if __name__ == "__main__":
    unittest.main()
