"""ROS-free contract tests for Adam head and waist control cards."""

from __future__ import annotations

import math
import sys
import threading
import types
import unittest
from unittest.mock import patch

sys.modules.setdefault("numpy", types.ModuleType("numpy"))

from device import (
    ADAM_PRO_JOINTS,
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
        self.assertEqual(
            schema["properties"]["action"]["enum"], ["set_angles", "reset"])
        self.assertEqual(
            schema["x-action-params"],
            {
                "set_angles": {
                    "params": ["duration_s", "yaw_deg", "pitch_deg"],
                },
                "reset": {"params": []},
            },
        )
        for field in ("yaw_deg", "pitch_deg"):
            self.assertIn(field, schema["properties"])
            self.assertEqual(schema["properties"][field]["minimum"], -60.0)
            self.assertEqual(schema["properties"][field]["maximum"], 60.0)

    def test_waist_angles_are_all_visible_with_real_ranges(self):
        tool = WaistControlPlugin(_FakeControl()).get_tool()
        schema = tool["inputSchema"]
        self.assertEqual(
            schema["properties"]["action"]["enum"], ["set_angles", "reset"])
        self.assertEqual(
            schema["x-action-params"],
            {
                "set_angles": {
                    "params": [
                        "duration_s", "roll_deg", "pitch_deg", "yaw_deg",
                    ],
                },
                "reset": {"params": []},
            },
        )
        expected = {
            "roll_deg": (-16.0, 16.0),
            "pitch_deg": (-48.0, 78.0),
            "yaw_deg": (-47.0, 47.0),
        }
        for field, limits in expected.items():
            self.assertEqual(schema["properties"][field]["minimum"], limits[0])
            self.assertEqual(schema["properties"][field]["maximum"], limits[1])

    def test_set_angles_applies_visible_angles_and_reset_returns_to_start(self):
        control = _FakeControl()
        plugin = HeadControlPlugin(control)
        result = plugin.dispatch(
            "set_angles", {"yaw_deg": 30, "pitch_deg": -10, "duration_s": 2})
        self.assertTrue(result["success"])
        targets, duration = control.calls[-1]
        self.assertAlmostEqual(targets["neckYaw"], math.radians(30))
        self.assertAlmostEqual(targets["neckPitch"], math.radians(-10))
        self.assertEqual(duration, 2.0)

        result = plugin.dispatch("set_angles", {"pitch_deg": 61})
        self.assertFalse(result["success"])
        self.assertEqual(result["code"], "INVALID_ARGUMENT")

        result = plugin.dispatch("set_angles", {})
        self.assertFalse(result["success"])
        self.assertEqual(result["code"], "INVALID_ARGUMENT")

        result = plugin.dispatch("reset", {})
        self.assertTrue(result["success"])
        self.assertEqual(control.calls[-1], (("neckYaw", "neckPitch"), None))

    def test_lifecycle_stop_reports_idle(self):
        plugin = HeadControlPlugin(_FakeControl())
        self.assertEqual(plugin.stop(), {"state": "idle"})


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


class _MotorState:
    def __init__(self, q):
        self.q = q


class _LowState:
    def __init__(self, positions):
        self.motor_state = [_MotorState(q) for q in positions]


class _SequenceSubscriber:
    def __init__(self, messages):
        self.messages = list(messages)

    def Read(self, timeout=None):
        if self.messages:
            return self.messages.pop(0)
        return None


class _RecordingPublisher:
    def __init__(self, result=True):
        self.result = result
        self.calls = []

    def Write(self, command, timeout=None):
        self.calls.append((command, timeout))
        return self.result


class _MotorCommand:
    pass


class _LowCommand:
    def __init__(self, dof):
        self.mode_pr = 0
        self.motor_cmd = [_MotorCommand() for _ in range(dof)]


def _fake_lowcmd(dof):
    return _LowCommand(dof)


class UpperBodyControllerTests(unittest.TestCase):
    def test_pro_joint_indices_match_vendor_low_level_layout(self):
        self.assertEqual(ADAM_PRO_JOINTS.index("waistYaw"), 12)
        self.assertEqual(ADAM_PRO_JOINTS.index("waistRoll"), 13)
        self.assertEqual(ADAM_PRO_JOINTS.index("waistPitch"), 14)
        self.assertEqual(ADAM_PRO_JOINTS.index("neckYaw"), 29)
        self.assertEqual(ADAM_PRO_JOINTS.index("neckPitch"), 30)

    def test_initial_state_accepts_complete_frame_after_timeout(self):
        positions = [float(index) for index in range(31)]
        control = UpperBodyLowcmdController(
            {}, dds_lowcmd_pub=object(),
            dds_lowstate_sub=_SequenceSubscriber([None, _LowState(positions)]),
        )
        control._read_initial_state()
        self.assertTrue(control._state_ready.is_set())
        self.assertEqual(control._hold_q, positions)
        self.assertIsNone(control._last_error)

    def test_short_initial_state_reports_actual_motor_count(self):
        control = UpperBodyLowcmdController(
            {"state_wait_timeout_s": 0}, dds_lowcmd_pub=object(),
            dds_lowstate_sub=object(),
        )
        self.assertFalse(control._load_initial_state(_LowState([0.0] * 30)))
        result = control.set_targets({"neckYaw": 0.0})
        self.assertEqual(result["code"], "LOWSTATE_UNAVAILABLE")
        self.assertEqual(
            result["message"],
            "rt/lowstate has 30 motors; expected at least 31",
        )

    def test_non_finite_initial_state_is_rejected(self):
        positions = [0.0] * 31
        positions[29] = math.nan
        control = UpperBodyLowcmdController(
            {"state_wait_timeout_s": 0}, dds_lowcmd_pub=object(),
            dds_lowstate_sub=object(),
        )
        self.assertFalse(control._load_initial_state(_LowState(positions)))
        self.assertEqual(
            control._last_error,
            "rt/lowstate contains a non-finite motor position",
        )

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

    def test_write_command_counts_only_successful_dds_writes(self):
        publisher = _RecordingPublisher()
        control = UpperBodyLowcmdController({}, dds_lowcmd_pub=publisher)
        control._hold_q = [0.0] * control._DOF
        control._segment_q = list(control._hold_q)
        control._active = True
        with patch(
                "device.pnd_adam_msg_dds__LowCmd_", _fake_lowcmd,
                create=True):
            control._write_command()
        self.assertEqual(control._writes, 1)
        self.assertIsNone(control._last_error)
        self.assertEqual(publisher.calls[0][1], 0.2)

    def test_write_command_records_false_dds_result_as_failure(self):
        publisher = _RecordingPublisher(result=False)
        control = UpperBodyLowcmdController({}, dds_lowcmd_pub=publisher)
        control._hold_q = [0.0] * control._DOF
        control._segment_q = list(control._hold_q)
        control._active = True
        with patch(
                "device.pnd_adam_msg_dds__LowCmd_", _fake_lowcmd,
                create=True):
            control._write_command()
        self.assertEqual(control._writes, 0)
        self.assertIn("not matched", control._last_error)

    def test_set_targets_waits_for_successful_dds_write(self):
        publisher = _RecordingPublisher()
        control = UpperBodyLowcmdController({}, dds_lowcmd_pub=publisher)
        control._hold_q = [0.0] * control._DOF
        control._segment_q = list(control._hold_q)
        control._state_ready.set()
        timer = threading.Timer(0.01, control._write_command)
        with patch(
                "device.pnd_adam_msg_dds__LowCmd_", _fake_lowcmd,
                create=True):
            timer.start()
            result = control.set_targets({"neckYaw": 0.1})
            timer.join()
        self.assertIsNone(result)
        self.assertEqual(control._writes, 1)

    def test_set_targets_reports_failed_dds_write(self):
        publisher = _RecordingPublisher(result=False)
        control = UpperBodyLowcmdController({}, dds_lowcmd_pub=publisher)
        control._hold_q = [0.0] * control._DOF
        control._segment_q = list(control._hold_q)
        control._state_ready.set()
        timer = threading.Timer(0.01, control._write_command)
        with patch(
                "device.pnd_adam_msg_dds__LowCmd_", _fake_lowcmd,
                create=True):
            timer.start()
            result = control.set_targets({"neckYaw": 0.1})
            timer.join()
        self.assertEqual(result["code"], "DDS_WRITE_FAILED")
        self.assertIn("not matched", result["message"])
        self.assertEqual(control._writes, 0)


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

    def test_bundle_disables_pro_only_controls_for_other_variants(self):
        config = {
            "variant": "sp",
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
        self.assertNotIn("head_control", names)
        self.assertNotIn("waist_control", names)
        self.assertIsNone(bundle._upper_body_control)


if __name__ == "__main__":
    unittest.main()
