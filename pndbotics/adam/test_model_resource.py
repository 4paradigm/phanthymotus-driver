"""Regression tests for the Adam dashboard model resource."""

from __future__ import annotations

import sys
import types
import unittest
import xml.etree.ElementTree as ET

sys.modules.setdefault("numpy", types.ModuleType("numpy"))

from device import ModelPlugin


class ModelResourceTests(unittest.TestCase):
    def test_pro_uses_official_hand_kinematic_tree(self):
        plugin = ModelPlugin({}, "", None, variant="pro")
        result = plugin.dispatch("model", {})

        self.assertEqual("adam_inspire.urdf", result["urdf_file"])
        self.assertTrue(result["hand_visuals"])
        self.assertFalse(result["hand_feedback_mapping"])
        self.assertFalse(result["neck_kinematics"])
        self.assertFalse(result["mesh_assets_included"])
        joints = {joint.get("name") for joint in ET.fromstring(result["urdf"]).findall("joint")}
        self.assertIn("L_index_MCP_joint", joints)
        self.assertIn("R_index_MCP_joint", joints)
        self.assertIn("wristRoll_Left", joints)
        self.assertIn("wristRoll_Right", joints)

    def test_standard_keeps_standard_model(self):
        plugin = ModelPlugin({}, "", None, variant="standard")
        result = plugin.dispatch("model", {})
        self.assertEqual("adam_pro.urdf", result["urdf_file"])


if __name__ == "__main__":
    unittest.main()
