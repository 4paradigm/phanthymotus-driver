"""Regression tests for Adam's side-specific hand commands."""

from __future__ import annotations

import sys
import types
import unittest

sys.modules.setdefault("numpy", types.ModuleType("numpy"))

from device import HandPlugin


class _StateCache:
    def __init__(self, positions=None):
        self.positions = positions

    def fresh_positions(self, timeout):
        return self.positions

    def status(self, timeout):
        return {"reader_available": True}


class HandControlTests(unittest.TestCase):
    def _plugin(self, positions=None):
        plugin = HandPlugin({}, "", None, state_cache=_StateCache(positions))
        plugin._activate = lambda target, action: {"state": "active", "target": list(target)}
        return plugin

    def test_one_sided_open_preserves_the_other_hand_from_feedback(self):
        plugin = self._plugin([10, 11, 12, 13, 14, 15, 20, 21, 22, 23, 24, 25])
        result = plugin.dispatch("open", {"side": "left"})
        self.assertEqual(plugin._open_positions[:6], result["target"][:6])
        self.assertEqual([20, 21, 22, 23, 24, 25], result["target"][6:])

    def test_one_sided_command_requires_known_other_hand_state(self):
        plugin = self._plugin()
        result = plugin.dispatch("open", {"side": "left"})
        self.assertEqual("HANDSTATE_UNAVAILABLE", result["error"])


if __name__ == "__main__":
    unittest.main()
