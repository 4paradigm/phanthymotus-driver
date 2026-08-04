from __future__ import annotations

import importlib.util
import sys
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))


def load_device():
    spec = importlib.util.spec_from_file_location("u1_contract", ROOT / "ubtech/u1_pro/device.py")
    module = importlib.util.module_from_spec(spec)
    assert spec.loader
    spec.loader.exec_module(module)
    return module


u1 = load_device()


class FakeBridge:
    state_topic = "/u1/state"
    def publish(self, command, params=None): return {"state": "awaiting_supplier_ack", "command": command, "params": params or {}}
    def snapshot(self): return {"supplier": {"state": "waiting"}}
    def acknowledgement(self, value): return {"state": "pending", "id": value}
    def graph(self): return {"topics": [], "services": [], "nodes": []}
    def close(self): pass


def tools(plugin):
    return plugin.get_tools() if hasattr(plugin, "get_tools") else [plugin.get_tool()]


class U1ContractTests(unittest.TestCase):
    def setUp(self):
        bridge = FakeBridge()
        self.plugins = [
            u1.U1StatePlugin(bridge), u1.U1SystemPlugin(bridge), u1.U1LocomotionPlugin(bridge),
            u1.U1BodyPlugin(bridge), u1.U1HeadPlugin(bridge), u1.U1HandsPlugin(bridge),
            u1.U1SpeechPlugin(bridge), u1.U1InteractionPlugin(bridge),
            u1.U1ChoreographyPlugin(bridge), u1.U1VendorPlugin(bridge),
        ]

    def test_broad_tool_surface_is_unique_and_schema_complete(self):
        definitions = [item for plugin in self.plugins for item in tools(plugin)]
        names = [item["name"] for item in definitions]
        self.assertGreaterEqual(len(names), 13)
        self.assertEqual(len(names), len(set(names)))
        for item in definitions:
            if item["type"] == "actuator":
                schema = item["inputSchema"]
                self.assertEqual(set(schema["properties"]["action"]["enum"]), set(schema["x-action-params"]))

    def test_commands_never_claim_unacknowledged_physical_success(self):
        result = self.plugins[6].dispatch("say", {"text": "hello"})
        self.assertEqual("awaiting_supplier_ack", result["state"])
        capabilities = self.plugins[0].dispatch("capabilities", {"_tool_name": "capabilities"})
        self.assertFalse(capabilities["supplier_idl_connected"])

    def test_metadata_and_bridge_topics_are_explicit(self):
        config = (ROOT / "ubtech/u1_pro/config.yaml").read_text()
        metadata = (ROOT / "ubtech/u1_pro/driver.yaml").read_text()
        for topic in ("/u1_pro/driver/command", "/u1_pro/driver/state", "/u1_pro/driver/ack"):
            self.assertIn(topic, config)
        self.assertIn("mcp_port: 15714", config)
        self.assertIn("port: 15714", metadata)
        self.assertIn("../../common", metadata)


if __name__ == "__main__":
    unittest.main()
