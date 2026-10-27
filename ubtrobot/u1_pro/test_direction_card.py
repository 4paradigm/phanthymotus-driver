"""Offline SDK -> public observation contracts; no robot access."""
import json
import sys
import types
import unittest
from pathlib import Path
from unittest import mock

sys.path.insert(0, str(Path(__file__).parent))
import direction_card as direction


class SoundDirectionTests(unittest.TestCase):
    def setUp(self):
        self.nodes = mock.Mock()
        self.nodes.namespace = "test"
        self.nodes.String = type("String", (), {})
        self.plugin = direction.SoundDirectionPlugin(self.nodes)
        self.clock = mock.patch.object(direction.time, "monotonic", return_value=100).start()
        self.addCleanup(mock.patch.stopall)

    def observation(self):
        return json.loads(self.nodes.core.create_publisher.return_value.publish.call_args.args[0].data)

    def test_transport_and_actions(self):
        tool = self.plugin.get_tool()
        self.assertEqual(tool["topic_out"], [{"topic": "/test/sound_direction", "format": "sensor/sound-direction"}])
        self.assertEqual(tool["inputSchema"]["properties"]["action"]["enum"], ["start", "stop", "info"])
        self.plugin.dispatch("start", {})
        args = self.nodes.robot.create_subscription.call_args.args
        self.assertEqual(args[:2], (self.nodes.String, direction.DOA_TOPIC))
        self.assertIs(args[3], self.nodes._sensor_qos)
        args[2](types.SimpleNamespace(data='{"azimuth":-45}'))
        self.assertEqual(self.observation()["angle_deg"], 45)
        self.nodes.set_system_enabled.assert_not_called()
        self.nodes.call_sdk.assert_not_called()

    def test_envelopes_and_confidence(self):
        for event in ({"azimuth": 12, "confidence": .8},
                      {"data": {"azimuth": 12, "confidence": .8}},
                      {"data": '{"azimuth":12,"confidence":0.8}'}):
            self.assertEqual(direction.parse_doa(types.SimpleNamespace(data=json.dumps(event))), (12, .8, True))
        self.assertEqual(direction.parse_doa('{"azimuth":-10}'), (-10, None, True))

    def test_invalid_messages(self):
        for event in ('null', '[]', '{}', 'bad', {"azimuth": True},
                      {"azimuth": float("nan")}, {"azimuth": float("inf")}, {"azimuth": 10**1000},
                      {"azimuth": 180}, {"azimuth": -181},
                      {"azimuth": 0, "confidence": 2}, {"azimuth": 0, "valid": "false"},
                      ' ' * 16385, {"data": {"data": {}}}):
            with self.subTest(event=str(event)[:80]), self.assertRaises((ValueError, TypeError)):
                direction.parse_doa(event)

    def test_front_left_right_rear(self):
        for source, expected in ((0, 0), (-45, 45), (90, -90), (-180, 180), (179, -179)):
            with self.subTest(source=source):
                self.assertEqual(direction.platform_angle(source), expected)
        for source in (True, None, float("nan"), 180):
            with self.assertRaises(ValueError):
                direction.platform_angle(source)

    def test_no_event_invalid_stale_and_recovery(self):
        self.assertEqual(self.plugin.dispatch("info", {})["state"], "idle")
        self.plugin.start()
        self.assertEqual(self.observation()["state"], "no_event")
        self.assertIsNone(self.observation()["angle_deg"])
        self.plugin._on_event('{"azimuth":0,"confidence":0}')
        self.assertEqual(self.observation()["state"], "fresh")
        self.assertEqual(self.observation()["angle_deg"], 0)
        self.assertEqual(self.observation()["confidence"], 0)
        self.clock.return_value = 102
        self.plugin._tick()
        self.assertEqual(self.observation()["state"], "stale")
        self.assertIsNone(self.observation()["angle_deg"])
        self.assertNotIn("confidence", self.observation())
        self.plugin._on_event('{"azimuth":-15}')
        self.assertEqual(self.observation()["angle_deg"], 15)
        self.assertNotIn("confidence", self.observation())
        self.plugin._on_event('bad')
        self.assertEqual(self.observation()["state"], "no_event")
        self.assertTrue(self.observation()["error"])
        self.assertIsNone(self.observation()["angle_deg"])
        self.plugin._on_event('{"azimuth":15,"valid":false}')
        self.assertEqual(self.observation()["state"], "no_event")
        self.assertFalse(self.observation()["error"])

    def test_no_cross_speaker_smoothing_or_fabricated_capture_clock(self):
        self.plugin.start()
        self.plugin._on_event('{"azimuth":-60,"timestamp":1700000000000000}')
        stamp = self.observation()["timestamp_ms"]
        self.plugin._tick()
        self.assertEqual(self.observation()["timestamp_ms"], stamp)
        self.plugin._on_event('{"azimuth":70}')
        data = self.observation()
        self.assertEqual(data["angle_deg"], -70)
        self.assertEqual(data["audio_window_start_us"], 0)
        self.assertEqual(data["audio_window_end_us"], 0)
        self.assertEqual(data["association_state"], "unknown")
        self.assertEqual(data["timestamp_basis"], "receipt_time_only")

    def test_idempotent_lifecycle_and_queued_old_callbacks(self):
        self.plugin.start()
        callback = self.nodes.robot.create_subscription.call_args.args[2]
        self.plugin.start()
        self.nodes.robot.create_subscription.assert_called_once()
        self.plugin._on_event('{"azimuth":30}')
        self.plugin.stop()
        self.assertEqual(self.observation()["state"], "no_event")
        self.plugin.stop()
        self.nodes.robot.destroy_subscription.assert_called_once()
        self.nodes.robot.destroy_timer.assert_called_once()
        self.nodes.core.destroy_publisher.assert_called_once()
        self.plugin.start()
        callback(types.SimpleNamespace(data='{"azimuth":-90}'))
        self.assertEqual(self.plugin.dispatch("info", {})["sdk_events_received"], 0)
        self.assertEqual(self.observation()["state"], "no_event")
        self.assertIsNone(self.plugin.dispatch("other", {}))

    def test_partial_start_cleanup(self):
        for step in ("create_subscription", "create_timer"):
            with self.subTest(step=step):
                nodes = mock.Mock(namespace="test", String=type("String", (), {}))
                plugin = direction.SoundDirectionPlugin(nodes)
                getattr(nodes.robot, step).side_effect = RuntimeError("unavailable")
                with self.assertRaisesRegex(RuntimeError, "unavailable"):
                    plugin.start()
                self.assertFalse(plugin.running)
                self.assertIsNone(plugin._publisher)
                self.assertIsNone(plugin._subscription)
                self.assertIsNone(plugin._timer)
                nodes.core.destroy_publisher.assert_called_once()

    def test_publication_failure_still_releases_all_resources(self):
        self.plugin.start()
        self.nodes.core.create_publisher.return_value.publish.side_effect = RuntimeError("publisher failed")
        with self.assertRaisesRegex(RuntimeError, "publisher failed"):
            self.plugin.stop()
        self.nodes.robot.destroy_subscription.assert_called_once()
        self.nodes.robot.destroy_timer.assert_called_once()
        self.nodes.core.destroy_publisher.assert_called_once()
        self.assertFalse(self.plugin.running)
        self.assertIsNone(self.plugin._publisher)
        self.plugin.stop()

    def test_cleanup_failure_does_not_skip_other_resources_and_can_retry(self):
        self.plugin.start()
        self.nodes.robot.destroy_timer.side_effect = RuntimeError("timer failed")
        with self.assertRaisesRegex(RuntimeError, "timer failed"):
            self.plugin.stop()
        self.nodes.robot.destroy_subscription.assert_called_once()
        self.nodes.core.destroy_publisher.assert_called_once()
        with self.assertRaisesRegex(RuntimeError, "timer failed"):
            self.plugin.start()
        self.nodes.robot.create_subscription.assert_called_once()
        self.nodes.robot.destroy_timer.side_effect = None
        self.plugin.stop()
        self.assertIsNone(self.plugin._timer)

    def test_registration_and_image_packaging(self):
        import test_device  # ROS-free test stubs
        import device
        self.assertIs(device.SoundDirectionPlugin, direction.SoundDirectionPlugin)
        root = Path(__file__).parent
        self.assertIn("direction_card.py", (root / "Dockerfile").read_text())
        self.assertIn("name: sound_direction, type: sensor", (root / "driver.yaml").read_text())
        self.assertNotIn("head_orientation", (root / "driver.yaml").read_text())


if __name__ == "__main__":
    unittest.main()
