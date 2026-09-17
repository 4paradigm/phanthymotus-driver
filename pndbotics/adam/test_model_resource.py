"""Regression tests for the Adam dashboard model resource."""

from __future__ import annotations

import sys
import types
import unittest
import xml.etree.ElementTree as ET

sys.modules.setdefault("numpy", types.ModuleType("numpy"))

from device import HAND_SKELETON_JOINTS, ModelPlugin, _hand_skeleton_positions


class ModelResourceTests(unittest.TestCase):
    def test_pro_uses_official_hand_kinematic_tree(self):
        plugin = ModelPlugin({}, "", None, variant="pro")
        result = plugin.dispatch("model", {})

        self.assertEqual("adam_inspire.urdf", result["urdf_file"])
        self.assertTrue(result["hand_visuals"])
        self.assertEqual("visual_linear", result["hand_feedback_mapping"])
        self.assertEqual("visual_approximation", result["neck_kinematics"])
        self.assertFalse(result["mesh_assets_included"])
        joints = {joint.get("name"): joint.get("type")
                  for joint in ET.fromstring(result["urdf"]).findall("joint")}
        for name in ("neckYaw", "neckPitch", "L_index_MCP_joint",
                     "R_index_MCP_joint", "wristRoll_Left", "wristRoll_Right"):
            self.assertEqual("revolute", joints[name])

    def test_standard_keeps_standard_model(self):
        plugin = ModelPlugin({}, "", None, variant="standard")
        result = plugin.dispatch("model", {})
        self.assertEqual("adam_pro.urdf", result["urdf_file"])

    def test_hand_feedback_maps_to_the_named_revolute_joints(self):
        joints = _hand_skeleton_positions([0] * 6 + [1000] * 6)
        self.assertEqual([item[0] for item in HAND_SKELETON_JOINTS],
                         [item["name"] for item in joints])
        self.assertEqual(0.0, joints[0]["q"])
        self.assertEqual(1.5533, joints[6]["q"])
        self.assertTrue(all(item["visual_mapping"] for item in joints))


if __name__ == "__main__":
    unittest.main()
