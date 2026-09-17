"""Regression tests for Adam's side-specific hand commands."""

from __future__ import annotations

import sys
import types
import unittest

sys.modules.setdefault("numpy", types.ModuleType("numpy"))

from device import HandGesturePlugin, HandPlugin, HandStateCache


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

    def test_thumbs_up_and_fist_have_distinct_thumb_targets(self):
        self.assertNotEqual(
            HandGesturePlugin._GESTURES["thumbs_up"],
            HandGesturePlugin._GESTURES["fist"],
        )
        self.assertGreater(HandGesturePlugin._GESTURES["thumbs_up"][4],
                           HandGesturePlugin._GESTURES["fist"][4])

    def test_gesture_catalog_includes_common_semantic_poses(self):
        gestures = HandGesturePlugin._GESTURES
        for name in ("light_grip", "pinch", "ok_sign", "handshake_grip", "three", "rock"):
            self.assertIn(name, gestures)
            self.assertEqual(6, len(gestures[name]))
        self.assertNotEqual(gestures["pinch"], gestures["ok_sign"])

    def test_skeleton_uses_commanded_target_when_hand_feedback_is_absent(self):
        cache = HandStateCache()
        cache.set_commanded_positions([1000] * 12)
        positions, source = cache.skeleton_positions(1.0)
        self.assertEqual([1000] * 12, positions)
        self.assertEqual("rt/handcmd_target", source)

    def test_skeleton_prefers_fresh_hand_feedback_over_command_target(self):
        cache = HandStateCache()
        cache.set_commanded_positions([1000] * 12)
        with cache._lock:
            cache._latest_position = [0] * 12
            cache._received_monotonic = __import__("time").monotonic()
        positions, source = cache.skeleton_positions(1.0)
        self.assertEqual([0] * 12, positions)
        self.assertEqual("rt/handstate", source)


if __name__ == "__main__":
    unittest.main()
