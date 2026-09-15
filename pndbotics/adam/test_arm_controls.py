"""ROS-free validation tests for the Adam human-facing arm interface."""

from __future__ import annotations

import math
import sys
import types
import unittest

sys.modules.setdefault("numpy", types.ModuleType("numpy"))

from device import ARM_JOINT_CONTROLS, ARM_POSES, _arm_target_radians


class ArmControlTests(unittest.TestCase):
    def test_control_ids_are_human_facing_and_cover_each_upper_body_joint(self):
        self.assertIn("left_shoulder_pitch", ARM_JOINT_CONTROLS)
        self.assertIn("right_wrist_roll", ARM_JOINT_CONTROLS)
        self.assertNotIn("shoulderPitch_Left", ARM_JOINT_CONTROLS)
        self.assertIn("neutral", ARM_POSES)

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


if __name__ == "__main__":
    unittest.main()
