from __future__ import annotations

import importlib.util
import sys
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))


def load_device():
    spec = importlib.util.spec_from_file_location("q5_device_contract", ROOT / "robotera/q5/device.py")
    module = importlib.util.module_from_spec(spec)
    assert spec.loader
    spec.loader.exec_module(module)
    return module


q5 = load_device()


class FakeRobot:
    def get_topic_names_and_types(self): return []
    def get_service_names_and_types(self): return []
    def get_node_names_and_namespaces(self): return []


class FakeNodes:
    robot = FakeRobot()
    joint_state = None
    def joint_snapshot(self): return []
    def close(self): pass


def definitions(plugin):
    return plugin.get_tools() if hasattr(plugin, "get_tools") else [plugin.get_tool()]


class Q5ToolContractTests(unittest.TestCase):
    def setUp(self):
        self.nodes = FakeNodes()
        self.body = q5.Q5BodyPlugin(self.nodes)
        self.plugins = [
            q5.Q5StatePlugin(self.nodes, "robot"),
            q5.Q5LifecyclePlugin(self.nodes),
            self.body,
            q5.Q5BasePlugin(self.nodes, {"control": {}}),
            q5.Q5HandPlugin(self.nodes),
            q5.Q5MpcPlugin(self.nodes),
            q5.Q5ChoreographyPlugin(self.nodes, self.body),
        ]

    def test_tool_names_are_unique_and_action_schemas_are_complete(self):
        tools = [item for plugin in self.plugins for item in definitions(plugin)]
        names = [item["name"] for item in tools]
        self.assertEqual(len(names), len(set(names)))
        self.assertEqual({"joints", "capabilities", "ros_graph", "lifecycle", "body", "base", "hand", "mpc", "choreography"}, set(names))
        for item in tools:
            if item["type"] != "actuator":
                continue
            schema = item["inputSchema"]
            self.assertEqual(set(schema["properties"]["action"]["enum"]), set(schema["x-action-params"]), item["name"])

    def test_state_info_returns_authoritative_topic(self):
        result = self.plugins[0].dispatch("info", {"_tool_name": "joints"})
        self.assertEqual("/robot/q5/state", result["topic_out"][0]["topic"])

    def test_hand_and_choreography_contracts_are_exposed(self):
        self.assertEqual(12, len(q5.Q5HandPlugin.LITE))
        actions = q5.Q5ChoreographyPlugin(self.nodes, self.body).get_tool()["inputSchema"]["properties"]["action"]["enum"]
        self.assertIn("custom", actions)
        self.assertIn("vendor_action", actions)


class Q5VendorContractTests(unittest.TestCase):
    def test_metadata_port_and_shared_build_context_match(self):
        config = (ROOT / "robotera/q5/config.yaml").read_text()
        metadata = (ROOT / "robotera/q5/driver.yaml").read_text()
        dockerfile = (ROOT / "robotera/q5/Dockerfile").read_text()
        self.assertIn("mcp_port: 15711", config)
        self.assertIn("port: 15711", metadata)
        self.assertIn("../../common", metadata)
        self.assertIn("COPY common/ /work/common/", dockerfile)
        self.assertIn("EXPOSE 15711", dockerfile)

    def test_public_vendor_revisions_and_interfaces_are_pinned(self):
        dockerfile = (ROOT / "robotera/q5/Dockerfile").read_text()
        source = (ROOT / "robotera/q5/device.py").read_text()
        config = (ROOT / "robotera/q5/config.yaml").read_text()
        self.assertIn("578353e11a9b46a87356fb994982f726051ba6ce", dockerfile)
        self.assertIn("86a415493d34af309e1c8c007fdea467d33d51e4", dockerfile)
        for contract in ("/dynamic_launch", "/simple_actions", "/servo_poses", "/slam/start_map", "/era_nav/nav_act"):
            self.assertIn(contract, source + config)


class SharedRuntimeTests(unittest.TestCase):
    def test_bundle_routes_tool_and_preserves_tool_name(self):
        from common.vendor_runtime import DriverBundle, tool

        class Plugin:
            def get_tool(self): return tool("probe", "sensor", "probe")
            def dispatch(self, action, args): return {"action": action, "tool": args["_tool_name"]}

        self.assertEqual({"action": "probe", "tool": "probe"}, DriverBundle([Plugin()]).dispatch("probe", {}))


if __name__ == "__main__":
    unittest.main()
