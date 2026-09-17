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
        model_joints = {
            joint.get("name"): joint
            for joint in ET.fromstring(result["urdf"]).findall("joint")
        }
        joints = {name: joint.get("type") for name, joint in model_joints.items()}
        for name in ("neckYaw", "neckPitch", "L_index_MCP_joint",
                     "L_index_DIP_joint", "R_index_MCP_joint", "R_index_DIP_joint",
                     "wristRoll_Left", "wristRoll_Right"):
            self.assertEqual("revolute", joints[name])
        # The official axes are mirrored across the hands. A generic axis
        # splays a finger when the visualization receives a close command.
        self.assertEqual("-1 0 0", model_joints["L_index_MCP_joint"].find("axis").get("xyz"))
        self.assertEqual("1 0 0", model_joints["R_index_MCP_joint"].find("axis").get("xyz"))

    def test_standard_keeps_standard_model(self):
        plugin = ModelPlugin({}, "", None, variant="standard")
        result = plugin.dispatch("model", {})
        self.assertEqual("adam_pro.urdf", result["urdf_file"])

    def test_hand_feedback_maps_to_the_named_revolute_joints(self):
        joints = _hand_skeleton_positions([0] * 6 + [1000] * 6)
        self.assertEqual([item[0] for item in HAND_SKELETON_JOINTS],
                         [item["name"] for item in joints[::2]])
        self.assertEqual([item[1] for item in HAND_SKELETON_JOINTS],
                         [item["name"] for item in joints[1::2]])
        self.assertEqual(0.0, joints[0]["q"])
        self.assertEqual(1.5533, joints[12]["q"])
        self.assertAlmostEqual(joints[0]["q"] * 0.6, joints[1]["q"])
        self.assertEqual(0.4538, joints[9]["q"])
        self.assertTrue(all(item["visual_mapping"] for item in joints))


if __name__ == "__main__":
    unittest.main()
