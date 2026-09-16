"""ROS-free validation tests for the Adam human-facing arm interface."""

from __future__ import annotations

import math
import sys
import types
import unittest

sys.modules.setdefault("numpy", types.ModuleType("numpy"))

import device
from device import (ADAM_PRO_JOINTS, ARM_ACTIONS, ARM_JOINT_CONTROLS,
                    ARM_POSES, ArmControlPlugin, _arm_target_radians)


class _FakePublisher:
    def __init__(self):
        self.commands = []

    def Write(self, command):
        self.commands.append(command)


def _fake_lowcmd(dof):
    return types.SimpleNamespace(
        mode_pr=0,
        motor_cmd=[types.SimpleNamespace(mode=0, q=0.0, dq=0.0, tau=0.0,
                                         kp=0.0, kd=0.0, ki=0.0)
                   for _ in range(dof)],
    )


class ArmControlTests(unittest.TestCase):
    def test_control_ids_are_human_facing_and_cover_each_upper_body_joint(self):
        self.assertIn("left_shoulder_pitch", ARM_JOINT_CONTROLS)
        self.assertIn("right_wrist_roll", ARM_JOINT_CONTROLS)
        self.assertNotIn("shoulderPitch_Left", ARM_JOINT_CONTROLS)
        self.assertIn("neutral", ARM_POSES)

    def test_arm_lowcmd_rejects_non_pro_layouts_explicitly(self):
        with self.assertRaisesRegex(ValueError, "only Adam Pro"):
            ArmControlPlugin({}, "", None, variant="sp")

    def test_each_joint_has_a_distinct_action_and_angle_field(self):
        self.assertEqual("left_elbow", ARM_ACTIONS["set_left_elbow"])
        self.assertEqual("right_wrist_roll", ARM_ACTIONS["set_right_wrist_roll"])
        self.assertEqual(len(ARM_JOINT_CONTROLS), len(ARM_ACTIONS))

    def test_degrees_convert_to_the_ros_joint_target(self):
        name, target = _arm_target_radians("left_shoulder_pitch", -90)
        self.assertEqual("shoulderPitch_Left", name)
        self.assertAlmostEqual(-math.pi / 2, target)

    def test_each_joint_rejects_its_own_limit_violation(self):
        with self.assertRaisesRegex(ValueError, "left_shoulder_roll"):
            _arm_target_radians("left_shoulder_roll", -40)
        with self.assertRaisesRegex(ValueError, "right_shoulder_roll"):
            _arm_target_radians("right_shoulder_roll", 40)

    def test_unknown_or_nonfinite_inputs_are_rejected(self):
        with self.assertRaisesRegex(ValueError, "advertised"):
            _arm_target_radians("dof_pos/shoulderPitch_Left", 0)
        with self.assertRaisesRegex(ValueError, "finite"):
            _arm_target_radians("left_elbow", float("nan"))

    def test_lowcmd_holds_non_arm_joints_and_uses_official_arm_pd(self):
        publisher = _FakePublisher()
        plugin = ArmControlPlugin({}, "", None, dds_lowcmd_pub=publisher)
        plugin._hold_q = [index / 100.0 for index in range(31)]
        plugin._current_q = plugin._hold_q.copy()
        plugin._state_ready.set()
        plugin._active = True
        elbow = ADAM_PRO_JOINTS.index("elbow_Left")
        plugin._target_q[elbow] = -0.5

        original_factory = getattr(device, "pnd_adam_msg_dds__LowCmd_", None)
        device.pnd_adam_msg_dds__LowCmd_ = _fake_lowcmd
        try:
            plugin._write_command(0.02)
        finally:
            if original_factory is None:
                del device.pnd_adam_msg_dds__LowCmd_
            else:
                device.pnd_adam_msg_dds__LowCmd_ = original_factory

        command = publisher.commands[-1]
        hip = ADAM_PRO_JOINTS.index("hipPitch_Left")
        self.assertEqual(command.motor_cmd[hip].q, plugin._hold_q[hip])
        self.assertEqual(command.motor_cmd[hip].kp, 400.0)
        self.assertEqual(command.motor_cmd[hip].kd, 6.1)
        self.assertLess(command.motor_cmd[elbow].q, plugin._hold_q[elbow])
        self.assertEqual(command.motor_cmd[elbow].kp, 100.0)
        self.assertEqual(command.motor_cmd[elbow].kd, 2.0)

    def test_release_writes_zero_arm_gain_before_deactivating(self):
        publisher = _FakePublisher()
        plugin = ArmControlPlugin({}, "", None, dds_lowcmd_pub=publisher)
        plugin._hold_q = [0.0] * 31
        plugin._current_q = [0.0] * 31
        plugin._state_ready.set()
        plugin._active = True
        elbow = ADAM_PRO_JOINTS.index("elbow_Left")
        plugin._target_q[elbow] = -0.5
        plugin._release_started_at = __import__("time").monotonic() - 2.0

        original_factory = getattr(device, "pnd_adam_msg_dds__LowCmd_", None)
        device.pnd_adam_msg_dds__LowCmd_ = _fake_lowcmd
        try:
            plugin._write_command(0.02)
        finally:
            if original_factory is None:
                del device.pnd_adam_msg_dds__LowCmd_
            else:
                device.pnd_adam_msg_dds__LowCmd_ = original_factory

        self.assertEqual(publisher.commands[-1].motor_cmd[elbow].kp, 0.0)
        self.assertFalse(plugin._active)

    def test_stop_waits_for_release_and_includes_waist(self):
        publisher = _FakePublisher()
        plugin = ArmControlPlugin({}, "", None, dds_lowcmd_pub=publisher)
        plugin._hold_q = [0.0] * 31
        plugin._current_q = [0.0] * 31
        plugin._state_ready.set()
        plugin._active = True
        plugin._streaming = True
        waist = ADAM_PRO_JOINTS.index("waistYaw")
        plugin._target_q[waist] = 0.2
        original_factory = getattr(device, "pnd_adam_msg_dds__LowCmd_", None)
        device.pnd_adam_msg_dds__LowCmd_ = _fake_lowcmd
        try:
            plugin._release_started_at = __import__("time").monotonic() - 2.0
            plugin._write_command(0.02)
        finally:
            if original_factory is None:
                del device.pnd_adam_msg_dds__LowCmd_
            else:
                device.pnd_adam_msg_dds__LowCmd_ = original_factory
        self.assertEqual(0.0, publisher.commands[-1].motor_cmd[waist].kp)


if __name__ == "__main__":
    unittest.main()
