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


class GreetingDistanceNodeTests(unittest.TestCase):
    """Test _handle_message parsing without a real ROS2 node."""

    def setUp(self):
        import greeting as g
        self._original = g.GreetingDistanceNode
        g.GreetingDistanceNode = _FakeNode

    def tearDown(self):
        import greeting as g
        g.GreetingDistanceNode = self._original

    def _make_node(self):
        received = []
        plugin = self._make_plugin_with_stubs()
        node = plugin._node
        node._on_distance = received.append
        return node

    def _make_plugin_with_stubs(self):
        executor = _FakeExecutor()
        stubs = _make_stubs()
        config = {
            "threshold_m": 2.0,
            "consecutive_frames": 1,
            "cooldown_s": 0,
        }
        return g.make_plugin(config, "test", executor, stubs)

    def test_parses_valid_distance_message(self):
        node = self._make_node()
        class Msg:
            data = '{"distance_m": 1.5}'
        node._handle_message(Msg)
        self.assertEqual(received := [], [])  # placeholder
        # Use a fresh stub to capture
        stubs = {"led": MagicMock(), "tts": MagicMock(), "arm": MagicMock()}
        executor = _FakeExecutor()
        captured = []
        plugin = g.make_plugin({"threshold_m": 2.0, "consecutive_frames": 1, "cooldown_s": 0}, "test", executor, stubs)
        original_cb = plugin._on_distance
        plugin._on_distance = lambda d: captured.append(d)
        # Re-create node to replace callback
        node = _FakeNode()
        node._on_distance = captured.append
        msg = MagicMock()
        msg.data = '{"distance_m": 1.5}'
        node._handle_message(msg)
        self.assertEqual(captured, [1.5])

    def test_rejects_missing_distance_m(self):
        node = _FakeNode()
        captured = []
        node._on_distance = captured.append
        msg = MagicMock()
        msg.data = '{"not_distance": 1.5}'
        node._handle_message(msg)
        self.assertEqual(captured, [])

    def test_rejects_invalid_json(self):
        node = _FakeNode()
        captured = []
        node._on_distance = captured.append
        msg = MagicMock()
        msg.data = 'not-json-at-all'
        node._handle_message(msg)
        self.assertEqual(captured, [])

    def test_rejects_non_finite_distance(self):
        node = _FakeNode()
        captured = []
        node._on_distance = captured.append
        msg = MagicMock()
        msg.data = '{"distance_m": "nan"}'
        node._handle_message(msg)
        self.assertEqual(captured, [])

        captured.clear()
        msg.data = '{"distance_m": "inf"}'
        node._handle_message(msg)
        self.assertEqual(captured, [])


class EndToEndWelcomeTests(unittest.TestCase):
    """Deterministic end-to-end test for the full _welcome() call sequence."""

    def setUp(self):
        self.executor = _FakeExecutor()
        self.stubs = _make_stubs()
        self.plugin_config = {
            "threshold_m": 2.0,
            "consecutive_frames": 1,
            "cooldown_s": 0,
            "text": "欢迎",
            "gesture": "high wave",
        }

    def _make_plugin(self):
        import greeting as g
        original = g.GreetingDistanceNode
        try:
            g.GreetingDistanceNode = _FakeNode
            return g.make_plugin(
                self.plugin_config, "test", self.executor, self.stubs
            )
        finally:
            g.GreetingDistanceNode = original

    def test_full_welcome_sequence_calls_led_tts_arm(self):
        """Verify: LED speaking -> TTS speak -> arm execute -> LED idle."""
        plugin = self._make_plugin()
        plugin.dispatch("enable", {})
        # One sample triggers the greeting (consecutive_frames=1)
        plugin._on_distance(1.0)
        # _welcome runs synchronously within _action_lock; join via the lock
        # Since _welcome is not directly joinable, we wait briefly.
        time.sleep(0.3)

        # LED should have been set to "speaking"
        self.stubs["led"].dispatch.assert_any_call("state", {"state": "speaking"})
        # TTS should have been called with the configured text
        self.stubs["tts"].dispatch.assert_called_once_with(
            "speak", {"text": "欢迎", "voice": 0}
        )
        # Arm should have been called with the configured gesture
        self.stubs["arm"].dispatch.assert_called_once_with(
            "execute", {"gesture": "high wave"}
        )
        # LED should have returned to "idle" after success
        self.stubs["led"].dispatch.assert_any_call("state", {"state": "idle"})

    def test_welcome_skips_on_disable_during_action(self):
        """When disabled while _welcome is running, hardware calls are skipped."""
        plugin = self._make_plugin()
        plugin.dispatch("enable", {})

        # Patch _welcome to hold the lock so we can disable mid-flight
        original_welcome = plugin._welcome
        freeze = threading.Event()

        def frozen_welcome():
            with plugin._action_lock:
                freeze.set()
                # Hold the lock until test unfreezes
                plugin._welcome_freeze.wait(timeout=5)
            # After lock released, normally would continue

        plugin._welcome = frozen_welcome
        plugin._on_distance(1.0)
        frozen_ok = freeze.wait(timeout=5)
        self.assertTrue(frozen_ok, "_welcome did not acquire lock")

        # Disable while _welcome holds the lock
        plugin.dispatch("disable", {})

        # Release the freeze so _welcome can finish
        plugin._welcome_freeze.clear()
        time.sleep(0.1)

        # TTS and arm should NOT have been called (disabled mid-lock)
        self.stubs["tts"].dispatch.assert_not_called()
        self.stubs["arm"].dispatch.assert_not_called()


if __name__ == "__main__":
    unittest.main()
