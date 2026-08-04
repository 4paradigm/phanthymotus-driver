from __future__ import annotations

import importlib.util
import sys
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))


def load_device():
    spec = importlib.util.spec_from_file_location("lynx_s10_contract", ROOT / "deep_robotics/lynx_s10/device.py")
    module = importlib.util.module_from_spec(spec)
    assert spec.loader
    spec.loader.exec_module(module)
    return module


s10 = load_device()


class FakeBridge:
    def publish(self, command, params=None): return {"state": "awaiting_supplier_ack", "command": command, "params": params or {}}


class LynxS10ContractTests(unittest.TestCase):
    def test_supplier_motion_and_choreography_schemas_are_complete(self):
        bridge = FakeBridge()
        definitions = [s10.S10MotionPlugin(bridge).get_tool(), s10.S10DancePlugin(bridge).get_tool()]
        self.assertEqual({"motion", "choreography"}, {item["name"] for item in definitions})
        for item in definitions:
            schema = item["inputSchema"]
            self.assertEqual(set(schema["properties"]["action"]["enum"]), set(schema["x-action-params"]))
        self.assertIn("recover", definitions[0]["inputSchema"]["properties"]["action"]["enum"])

    def test_supplier_commands_require_acknowledgement(self):
        result = s10.S10MotionPlugin(FakeBridge()).dispatch("stand", {})
        self.assertEqual("awaiting_supplier_ack", result["state"])
        self.assertEqual("motion.stand", result["command"])

    def test_standard_ros2_topics_and_metadata_are_explicit(self):
        config = (ROOT / "deep_robotics/lynx_s10/config.yaml").read_text()
        source = (ROOT / "deep_robotics/lynx_s10/device.py").read_text()
        metadata = (ROOT / "deep_robotics/lynx_s10/driver.yaml").read_text()
        for topic in ("/cmd_vel", "/goal_pose", "/joint_states", "/imu/data", "/odom", "/battery_state", "/lidar/points"):
            self.assertIn(topic, config + source)
        self.assertIn("mcp_port: 15716", config)
        self.assertIn("port: 15716", metadata)
        self.assertIn("../../common", metadata)

    def test_official_drdds_revision_is_pinned_without_claiming_s10_topic_mapping(self):
        dockerfile = (ROOT / "deep_robotics/lynx_s10/Dockerfile").read_text()
        readme = (ROOT / "deep_robotics/lynx_s10/README.md").read_text()
        self.assertIn("a0d1a29eec5c4db5a9107595bb51e3be8122b86c", dockerfile)
        self.assertIn("deep-robotics-msg", dockerfile)
        self.assertIn("does not currently document", readme)


if __name__ == "__main__":
    unittest.main()
