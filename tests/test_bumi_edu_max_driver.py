from __future__ import annotations

import importlib.util
import sys
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))


def load_device():
    spec = importlib.util.spec_from_file_location("bumi_contract", ROOT / "noetix/bumi_edu_max/device.py")
    module = importlib.util.module_from_spec(spec)
    assert spec.loader
    spec.loader.exec_module(module)
    return module


bumi = load_device()


class FakeSdk:
    low = None
    def command(self, *args, **kwargs): pass


def tools(plugin):
    return plugin.get_tools() if hasattr(plugin, "get_tools") else [plugin.get_tool()]


class BumiContractTests(unittest.TestCase):
    def test_all_public_control_commands_are_exposed(self):
        self.assertEqual(18, len(bumi.COMMANDS))
        for name in ("WALK", "RUN", "DANCE", "DANCE1", "DANCE2", "FALLTOSTAND", "STARTTEACH", "SAVETEACH", "PLAYTEACH"):
            self.assertIn(name, bumi.COMMANDS)

    def test_tool_contracts_are_unique_and_complete(self):
        sdk = FakeSdk()
        plugins = [
            bumi.BumiMotionPlugin(sdk, {"control": {}}), bumi.BumiDancePlugin(sdk),
            bumi.BumiTeachingPlugin(sdk), bumi.BumiLowLevelPlugin(sdk, {"control": {}}),
        ]
        definitions = [item for plugin in plugins for item in tools(plugin)]
        self.assertEqual({"motion", "dance", "teaching", "joints"}, {item["name"] for item in definitions})
        for item in definitions:
            schema = item["inputSchema"]
            self.assertEqual(set(schema["properties"]["action"]["enum"]), set(schema["x-action-params"]))

    def test_native_sdk_revision_and_metadata_are_pinned(self):
        dockerfile = (ROOT / "noetix/bumi_edu_max/Dockerfile").read_text()
        config = (ROOT / "noetix/bumi_edu_max/config.yaml").read_text()
        metadata = (ROOT / "noetix/bumi_edu_max/driver.yaml").read_text()
        self.assertIn("052ea95bf2d503cc2174b68a015b374591d3d54e", dockerfile)
        self.assertIn("noetix_sdk_bumi", dockerfile)
        self.assertIn("joint_count: 21", config)
        self.assertIn("mcp_port: 15713", config)
        self.assertIn("port: 15713", metadata)
        self.assertIn("../../common", metadata)


if __name__ == "__main__":
    unittest.main()
