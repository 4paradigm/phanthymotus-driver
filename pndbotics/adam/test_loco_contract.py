"""Schema/adapter contract tests for Adam RL locomotion."""

from __future__ import annotations

import sys
import types
import unittest
from pathlib import Path

sys.modules.setdefault("numpy", types.ModuleType("numpy"))
sys.path.insert(0, str(Path(__file__).parent))

from device import RlLocoPlugin


class _Grpc:
    def __init__(self):
        self.mode = None

    def set_mode(self, mode):
        self.mode = mode
        return {"success": True, "current_state": mode}

    def set_motion(self, command, motion_file):
        self.motion = (command, motion_file)
        return {"success": True, "current_motion": motion_file}

    def set_control_mode(self, domain_id):
        self.domain_id = domain_id
        return {"success": True, "domain_id": domain_id}


class LocoContractTests(unittest.TestCase):
    def test_set_mode_schema_and_dispatch_use_rl_state_names(self):
        grpc = _Grpc()
        plugin = RlLocoPlugin({}, "adam", None, grpc)

        mode_schema = plugin.get_tool()["inputSchema"]["properties"]["target_state"]
        self.assertEqual(mode_schema["type"], "string")

        result = plugin.dispatch("set_mode", {"target_state": "STAND_WALK"})
        self.assertEqual(grpc.mode, "STAND_WALK")
        self.assertEqual(result["current_state"], "STAND_WALK")

    def test_full_rl_actions_are_exposed(self):
        grpc = _Grpc()
        plugin = RlLocoPlugin({}, "adam", None, grpc)

        actions = plugin.get_tool()["inputSchema"]["properties"]["action"]["enum"]
        self.assertTrue({"motion", "tracking_motion", "set_control_mode",
                         "get_control_state", "shutdown"}.issubset(actions))
        plugin.dispatch("motion", {"command": "PLAY", "motion_file": "Sources/motion/Greeting.txt"})
        plugin.dispatch("set_control_mode", {"domain_id": 1})
        self.assertEqual(("PLAY", "Sources/motion/Greeting.txt"), grpc.motion)
        self.assertEqual(1, grpc.domain_id)


if __name__ == "__main__":
    unittest.main()
