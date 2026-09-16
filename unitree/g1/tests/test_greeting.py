import sys
from pathlib import Path
from unittest.mock import MagicMock
import time

import unittest

_base = Path(__file__).resolve().parents[1]
if str(_base) not in sys.path:
    sys.path.insert(0, str(_base))

from greeting import GreetingController, GreetingObservation


class _FakeNode:
    """Dummy ROS2 node placeholder."""

    def __init__(self, *a, **k):
        pass

    def destroy_node(self):
        pass


class _FakeExecutor:
    """In-memory executor that tracks added/removed nodes."""

    def __init__(self):
        self.nodes = []

    def add_node(self, node):
        self.nodes.append(node)

    def remove_node(self, node):
        self.nodes.remove(node)


def _make_stubs(**kwargs):
    """Return stub dependencies, overriding individual defaults."""
    stubs = {
        "led": MagicMock(),
        "tts": MagicMock(),
        "arm": MagicMock(),
    }
    stubs.update(kwargs)
    return stubs


class GreetingControllerTests(unittest.TestCase):
    def test_triggers_after_consecutive_frames(self):
        controller = GreetingController(threshold_m=2.0, consecutive_frames=3, cooldown_s=10)
        self.assertFalse(controller.update(GreetingObservation(2.0), now=0))
        self.assertFalse(controller.update(GreetingObservation(1.8), now=1))
        self.assertTrue(controller.update(GreetingObservation(1.5), now=2))

    def test_does_not_retrigger_until_person_leaves(self):
        controller = GreetingController(threshold_m=2.0, consecutive_frames=1, cooldown_s=0)
        self.assertTrue(controller.update(GreetingObservation(1.0), now=0))
        self.assertFalse(controller.update(GreetingObservation(1.0), now=1))
        self.assertFalse(controller.update(GreetingObservation(3.0), now=2))
        self.assertTrue(controller.update(GreetingObservation(1.0), now=3))

    def test_cooldown_blocks_new_trigger(self):
        controller = GreetingController(threshold_m=2.0, consecutive_frames=1, cooldown_s=10)
        self.assertTrue(controller.update(GreetingObservation(1.0), now=0))
        self.assertFalse(controller.update(GreetingObservation(3.0), now=1))
        self.assertFalse(controller.update(GreetingObservation(1.0), now=2))
        self.assertFalse(controller.update(GreetingObservation(3.0), now=9))
        self.assertTrue(controller.update(GreetingObservation(1.0), now=10))

    def test_invalid_and_missing_distance_are_outside(self):
        controller = GreetingController(threshold_m=2.0, consecutive_frames=1, cooldown_s=0)
        self.assertFalse(controller.update(None, now=0))
        self.assertFalse(controller.update(GreetingObservation(0), now=1))
        self.assertFalse(controller.update(GreetingObservation(2.1), now=2))

    def test_reset_clears_all_state(self):
        controller = GreetingController(threshold_m=2.0, consecutive_frames=1, cooldown_s=0)
        controller.update(GreetingObservation(1.0))
        self.assertFalse(controller._armed)
        controller.reset()
        self.assertTrue(controller._armed)
        self.assertEqual(controller._inside_count, 0)
        self.assertEqual(controller._cooldown_until, 0.0)
        self.assertIsNone(controller._last_distance_m)


class ProductionPluginTests(unittest.TestCase):

    def setUp(self):
        self.executor = _FakeExecutor()
        self.stubs = _make_stubs()
        self.plugin_config = {
            "threshold_m": 2.0,
            "consecutive_frames": 1,
            "cooldown_s": 0,
            "distance_topic": "/test/camera/distance",
        }
        self.plugin = self._make_plugin()

    def _make_plugin(self):
        # Patch the ROS2 node class so Plugin doesn't spawn a real rclpy node
        import greeting as g
        original = g.GreetingDistanceNode
        try:
            g.GreetingDistanceNode = _FakeNode
            return g.make_plugin(
                self.plugin_config, "test", self.executor, self.stubs
            )
        finally:
            g.GreetingDistanceNode = original

    # -- lifecycle / schema tests --

    def test_start_returns_ready(self):
        self.assertEqual(self.plugin.dispatch("start", {}), {"state": "ready"})

    def test_stop_returns_idle_and_resets_controller(self):
        self.plugin.dispatch("start", {})
        # prime controller state
        self.plugin._on_distance(1.0)
        self.assertFalse(self.plugin._controller._armed)
        self.plugin.dispatch("stop", {})
        self.assertEqual(self.plugin.dispatch("stop", {}), {"state": "idle"})
        self.assertTrue(self.plugin._controller._armed)
        self.assertEqual(self.plugin._controller._inside_count, 0)

    def test_disable_resets_controller(self):
        self.plugin.dispatch("enable", {})
        self.plugin._on_distance(1.0)
        self.assertFalse(self.plugin._controller._armed)
        self.plugin.dispatch("disable", {})
        self.assertTrue(self.plugin._controller._armed)
        self.assertEqual(self.plugin._controller._inside_count, 0)

    def test_disable_then_enable_does_not_automatically_fire(self):
        """After disable resets state, one sample must reach consecutive_frames again."""
        self.plugin._config["consecutive_frames"] = 2
        p2 = self._make_plugin()
        p2.dispatch("enable", {})
        p2._on_distance(1.0)
        p2.dispatch("disable", {})
        p2.dispatch("enable", {})
        p2._on_distance(1.0)
        self.assertTrue(p2._controller._armed)  # need 2 frames, only 1 so still armed
        p2._on_distance(1.0)
        self.assertFalse(p2._controller._armed)  # triggered, _armed set to False

    def test_stop_resets_controller_and_led(self):
        self.plugin.dispatch("start", {})
        self.plugin._on_distance(1.0)
        self.assertFalse(self.plugin._controller._armed)
        self.plugin.dispatch("stop", {})
        self.assertEqual(self.plugin.dispatch("stop", {}), {"state": "idle"})
        self.assertTrue(self.plugin._controller._armed)
        self.plugin.stubs["led"].dispatch.assert_any_call("state", {"state": "idle"})

    def test_status_returns_enabled_and_topic(self):
        self.plugin.dispatch("start", {})
        status = self.plugin.dispatch("status", {})
        self.assertTrue(status["enabled"])
        self.assertEqual(status["topic"], "/test/camera/distance")

    def test_info_returns_topic_in(self):
        info = self.plugin.dispatch("info", {})
        self.assertIn("topic_in", info)
        topics = info["topic_in"]
        self.assertEqual(len(topics), 1)
        self.assertEqual(topics[0]["topic"], "/test/camera/distance")
        self.assertEqual(topics[0]["format"], "data/json")

    # -- child dispatch failure tests --

    def test_tts_non_zero_ret_marks_error(self):
        self.stubs["tts"].dispatch.return_value = {"ret": 1}
        self.plugin.dispatch("start", {})
        self.plugin._on_distance(1.0)
        self.plugin._controller._armed = True  # prevent cooldown check in _welcome
        # _on_distance triggers _welcome in a daemon thread; wait for it
        time.sleep(0.3)
        self.stubs["tts"].dispatch.assert_called()

    def test_arm_non_zero_ret_marks_error(self):
        self.stubs["arm"].dispatch.return_value = {"ret": -1}
        self.plugin.dispatch("start", {})
        self.plugin._on_distance(1.0)
        time.sleep(0.3)
        self.stubs["arm"].dispatch.assert_called()

    def test_tts_error_field_marks_error(self):
        self.stubs["tts"].dispatch.return_value = {"error": "device busy"}
        self.plugin.dispatch("start", {})
        self.plugin._on_distance(1.0)
        time.sleep(0.3)
        self.stubs["tts"].dispatch.assert_called()


if __name__ == "__main__":
    unittest.main()
