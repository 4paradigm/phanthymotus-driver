"""Schema/adapter contract tests for Adam RL locomotion."""

from __future__ import annotations

import sys
import types
import unittest

sys.modules.setdefault("numpy", types.ModuleType("numpy"))

from device import LocoPlugin


class _Grpc:
    def __init__(self):
        self.mode = None

    def set_mode(self, mode):
        self.mode = mode
        return {"success": True, "current_state": mode}


class LocoContractTests(unittest.TestCase):
    def test_set_mode_schema_and_dispatch_use_rl_state_names(self):
        grpc = _Grpc()
        plugin = LocoPlugin({}, "adam", None, grpc)

        mode_schema = plugin.get_tool()["inputSchema"]["properties"]["mode"]
        self.assertEqual(mode_schema["type"], "string")

        result = plugin.dispatch("set_mode", {"mode": "walk"})
        self.assertEqual(grpc.mode, "walk")
        self.assertEqual(result["current_state"], "walk")


if __name__ == "__main__":
    unittest.main()
