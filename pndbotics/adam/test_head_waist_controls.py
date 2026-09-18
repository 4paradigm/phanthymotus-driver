"""ROS-free contract tests for Adam head and waist control cards."""

from __future__ import annotations

import math
import sys
import types
import unittest

sys.modules.setdefault("numpy", types.ModuleType("numpy"))

from device import (
    AdamDeviceBundle,
    HeadControlPlugin,
    UpperBodyLowcmdController,
    WaistControlPlugin,
)


class _FakeControl:
    def __init__(self):
        self.calls = []

    def start(self):
        return {"state": "ready"}

    def stop(self):
        return {"state": "idle"}

    def info(self):
        return {"state": "idle"}

    def set_target(self, joint, radians, duration):
        self.calls.append((joint, radians, duration))
        return None

    def set_targets(self, targets, duration):
        self.calls.append((dict(targets), duration))
        return None

    def reset(self, joints, duration):
        self.calls.append((tuple(joints), duration))
        return None


class HeadWaistSchemaTests(unittest.TestCase):
    def test_head_angles_are_all_visible_with_real_ranges(self):
        tool = HeadControlPlugin(_FakeControl()).get_tool()
        schema = tool["inputSchema"]
        self.assertEqual(schema["properties"]["action"]["enum"], ["reset"])
        self.assertNotIn("x-action-params", schema)
        for field in ("yaw_deg", "pitch_deg"):
            self.assertIn(field, schema["properties"])
            self.assertEqual(schema["properties"][field]["minimum"], -60.0)
            self.assertEqual(schema["properties"][field]["maximum"], 60.0)

    def test_waist_angles_are_all_visible_with_real_ranges(self):
        tool = WaistControlPlugin(_FakeControl()).get_tool()
        schema = tool["inputSchema"]
        self.assertEqual(schema["properties"]["action"]["enum"], ["reset"])
        self.assertNotIn("x-action-params", schema)
        expected = {
            "roll_deg": (-16.0, 16.0),
            "pitch_deg": (-48.0, 78.0),
            "yaw_deg": (-47.0, 47.0),
        }
        for field, limits in expected.items():
            self.assertEqual(schema["properties"][field]["minimum"], limits[0])
            self.assertEqual(schema["properties"][field]["maximum"], limits[1])

    def test_reset_applies_visible_angles_or_returns_to_start(self):
        control = _FakeControl()
        plugin = HeadControlPlugin(control)
        result = plugin.dispatch(
            "reset", {"yaw_deg": 30, "pitch_deg": -10, "duration_s": 2})
        self.assertTrue(result["success"])
        targets, duration = control.calls[-1]
        self.assertAlmostEqual(targets["neckYaw"], math.radians(30))
        self.assertAlmostEqual(targets["neckPitch"], math.radians(-10))
        self.assertEqual(duration, 2.0)

        result = plugin.dispatch("reset", {"pitch_deg": 61})
        self.assertFalse(result["success"])
        self.assertEqual(result["code"], "INVALID_ARGUMENT")

        result = plugin.dispatch("reset", {})
        self.assertTrue(result["success"])
        self.assertEqual(control.calls[-1], (("neckYaw", "neckPitch"), None))


class _RecordingUpperBodyController(UpperBodyLowcmdController):
    def __init__(self):
        super().__init__({}, dds_lowcmd_pub=object())
        self._hold_q = [float(index) for index in range(self._DOF)]
        self._segment_q = list(self._hold_q)
        self._state_ready.set()
        self.target_batches = []

    def set_targets(self, targets_by_name, duration_s=None):
        self.target_batches.append((dict(targets_by_name), duration_s))
        return None


class UpperBodyControllerTests(unittest.TestCase):
    def test_reset_submits_all_axes_as_one_target_batch(self):
        control = _RecordingUpperBodyController()
        control.reset(["neckYaw", "neckPitch"], 1.5)
        self.assertEqual(len(control.target_batches), 1)
        targets, duration = control.target_batches[0]
        self.assertEqual(duration, 1.5)
        self.assertEqual(
            targets,
            {
                "neckYaw": control._hold_q[control._joint_index("neckYaw")],
                "neckPitch": control._hold_q[control._joint_index("neckPitch")],
            },
        )


class BundleRegistrationTests(unittest.TestCase):
    def test_bundle_registers_only_basic_head_and_waist_cards(self):
        config = {
            "variant": "pro",
            "plugins": {
                "state": {"enabled": False},
                "estop": {"enabled": False},
                "loco": {"enabled": False},
                "camera": {"enabled": False},
                "vision_capture": {"enabled": False},
                "arm": {"enabled": False},
                "hand": {"enabled": False},
                "hand_state": {"enabled": False},
                "model": {"enabled": False},
                "head": {"enabled": True},
                "waist": {"enabled": True},
            },
        }
        bundle = AdamDeviceBundle(
            config, "", None, None, ros2_enabled=False,
            dds_lowcmd_pub=object(), dds_upper_body_lowstate_sub=object(),
        )
        names = [tool["name"] for tool in bundle.get_all_tools()]
        self.assertEqual(names, ["waist_control", "head_control"])
        owners = {
            id(plugin._control)
            for plugin in bundle._plugins
            if isinstance(plugin, (HeadControlPlugin, WaistControlPlugin))
        }
        self.assertEqual(len(owners), 1)
        self.assertEqual(
            sum(isinstance(plugin, UpperBodyLowcmdController)
                for plugin in bundle._plugins),
            0,
        )
        self.assertNotIn("head_gesture", names)
        self.assertNotIn("waist_gesture", names)


if __name__ == "__main__":
    unittest.main()
