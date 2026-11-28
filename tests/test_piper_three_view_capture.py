"""Hardware-free contracts for the PiPER three-view live sensor."""

from pathlib import Path
import sys
import tempfile
from types import ModuleType, SimpleNamespace
import unittest
from unittest.mock import Mock, call, patch

import yaml

from agilex.piper.three_view_card import LIVE_NAMES, TOOL_NAME, ThreeViewLive
from common.vendor_runtime import DriverBundle


NOW_MS = 1_700_000_000_000
JPEG = b"\xff\xd8\xff\xd9"
PIPER_MANIFEST = Path(__file__).resolve().parents[1] / "agilex/piper/driver.yaml"


class ThreeViewLiveTests(unittest.TestCase):
    def setUp(self):
        self.node = Mock()
        self.ros2 = SimpleNamespace(ctx_core=object(), executor_core=Mock())
        ros_modules = {
            name: ModuleType(name)
            for name in ("rclpy", "rclpy.node", "rclpy.qos", "sensor_msgs", "sensor_msgs.msg")
        }
        ros_modules["rclpy.node"].Node = Mock(return_value=self.node)
        ros_modules["rclpy.qos"].qos_profile_sensor_data = object()
        ros_modules["sensor_msgs.msg"].CompressedImage = object()
        modules = patch.dict(sys.modules, ros_modules)
        modules.start()
        self.addCleanup(modules.stop)
        clock = patch("agilex.piper.three_view_card.time_ns", return_value=NOW_MS * 1_000_000)
        clock.start()
        self.addCleanup(clock.stop)
        self.publishers = {view: Mock() for view in LIVE_NAMES}
        self.node.create_publisher.side_effect = list(self.publishers.values())
        self.plugin = ThreeViewLive("piper_test", self.ros2)
        self.bundle = DriverBundle([self.plugin])
        self.addCleanup(self.bundle.stop_all)

    def call(self, action):
        return self.bundle.dispatch(TOOL_NAME, {"action": action})

    def feed(self, view, delta_ms, data=JPEG):
        topic = f"/aloha_lab/camera/{view}/jpeg"
        callback = next(
            subscription.args[2] for subscription in self.node.create_subscription.call_args_list
            if subscription.args[1] == topic
        )
        sec, ms = divmod(NOW_MS + delta_ms, 1000)
        message = SimpleNamespace(
            header=SimpleNamespace(stamp=SimpleNamespace(sec=sec, nanosec=ms * 1_000_000)),
            data=data,
        )
        callback(message)
        return message

    def test_declares_three_live_outputs_without_actions(self):
        [definition] = self.bundle.get_all_tools()
        self.assertEqual(definition["title"], "Three-View Live")
        self.assertEqual(definition["type"], "sensor")
        self.assertNotIn("action", definition["inputSchema"]["properties"])
        self.assertEqual([port["topic"] for port in definition["topic_out"]], [
            "/piper_test/piper/three_view/front_live",
            "/piper_test/piper/three_view/left_wrist_live",
            "/piper_test/piper/three_view/right_wrist_live",
        ])
        self.assertTrue(all(port["format"] == "image/jpeg" for port in definition["topic_out"]))
        manifest = yaml.safe_load(PIPER_MANIFEST.read_text(encoding="utf-8"))
        self.assertIn({"name": TOOL_NAME, "type": "sensor"}, manifest["cards"])
        self.assertIsNone(self.call("capture"))

    def test_streams_on_driver_start_and_uses_canvas_lifecycle(self):
        self.bundle.start_all()
        self.assertEqual(self.call("start"), {"state": "running"})
        self.assertEqual(self.node.create_subscription.call_count, 3)
        first = {view: self.feed(view, -100) for view in self.publishers}
        self.assertEqual(self.call("stop"), {"state": "idle"})
        second = {view: self.feed(view, -40) for view in self.publishers}
        for view, publisher in self.publishers.items():
            self.assertEqual(publisher.publish.call_args_list, [call(first[view]), call(second[view])])

        self.bundle.stop_all()
        self.ros2.executor_core.remove_node.assert_called_once_with(self.node)
        self.node.destroy_node.assert_called_once()
        self.assertEqual(self.call("info"), {"state": "idle"})

    def test_rejects_stale_or_invalid_frames(self):
        self.bundle.start_all()
        self.feed("front", -2001)
        self.feed("left", -50, b"\xff\xd8missing-end-marker")
        self.feed("right", 101)
        for publisher in self.publishers.values():
            publisher.publish.assert_not_called()

    def test_bundle_registers_only_when_enabled(self):
        from agilex.piper.device import build_plugins

        with tempfile.TemporaryDirectory() as data_dir:
            config = {"data_dir": data_dir}
            default = DriverBundle(build_plugins(config, "piper_test", None))
            enabled = DriverBundle(build_plugins(
                {**config, "three_view_capture": {"enabled": True}}, "piper_test", None
            ))
        default_tools = {tool["name"] for tool in default.get_all_tools()}
        enabled_tools = {tool["name"] for tool in enabled.get_all_tools()}
        self.assertNotIn(TOOL_NAME, default_tools)
        self.assertEqual(enabled_tools - default_tools, {TOOL_NAME})
