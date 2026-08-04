from __future__ import annotations

import importlib.util
import sys
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))


def load_device():
    spec = importlib.util.spec_from_file_location("adam_pro_contract", ROOT / "pndbotics/adam_pro/device.py")
    module = importlib.util.module_from_spec(spec)
    assert spec.loader
    spec.loader.exec_module(module)
    return module


adam = load_device()


class FakeRobot:
    def get_topic_names_and_types(self): return []
    def get_service_names_and_types(self): return []
    def get_node_names_and_namespaces(self): return []


class FakeNodes:
    robot = FakeRobot()
    camera_topic = "/adam/camera"
    low_state = None
    def snapshot(self): return {"state": "waiting", "joints": []}
    def positions(self): return None
    def close(self): pass


def tools(plugin):
    return plugin.get_tools() if hasattr(plugin, "get_tools") else [plugin.get_tool()]


class AdamProContractTests(unittest.TestCase):
    def test_official_31_motor_layout_and_gains(self):
        self.assertEqual(31, len(adam.JOINT_NAMES))
        self.assertEqual(31, len(set(adam.JOINT_NAMES)))
        self.assertEqual(31, len(adam.KP))
        self.assertEqual(31, len(adam.KD))
        self.assertEqual(tuple(range(31)), adam.GROUPS["all"])

    def test_tools_are_unique_and_action_schemas_match(self):
        nodes = FakeNodes()
        low = adam.AdamLowLevelPlugin(nodes, {"control": {}})
        plugins = [adam.AdamStatePlugin(nodes, "robot"), low, adam.AdamHandPlugin(nodes), adam.AdamChoreographyPlugin(low)]
        definitions = [item for plugin in plugins for item in tools(plugin)]
        names = [item["name"] for item in definitions]
        self.assertEqual(len(names), len(set(names)))
        self.assertEqual({"state", "camera", "joint_groups", "capabilities", "ros_graph", "joints", "hand", "choreography"}, set(names))
        for item in definitions:
            if item["type"] == "actuator":
                schema = item["inputSchema"]
                self.assertEqual(set(schema["properties"]["action"]["enum"]), set(schema["x-action-params"]))

    def test_vendor_revision_topics_and_metadata_are_pinned(self):
        dockerfile = (ROOT / "pndbotics/adam_pro/Dockerfile").read_text()
        config = (ROOT / "pndbotics/adam_pro/config.yaml").read_text()
        metadata = (ROOT / "pndbotics/adam_pro/driver.yaml").read_text()
        self.assertIn("29d92afbf417b32afc26c9450f27d211abf1b259", dockerfile)
        for topic in ("/lowstate", "/lowcmd", "/handcmd"):
            self.assertIn(topic, config)
        self.assertIn("mcp_port: 15712", config)
        self.assertIn("port: 15712", metadata)
        self.assertIn("../../common", metadata)


if __name__ == "__main__":
    unittest.main()
