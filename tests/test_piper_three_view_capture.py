"""Hardware-free contracts for the PiPER three-view capture card."""

from pathlib import Path
import sys
import tempfile
from types import ModuleType, SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import yaml

from agilex.piper.three_view_card import TOOL_NAME, ThreeViewCapture
from common.vendor_runtime import DriverBundle


NOW_MS = 1_700_000_000_000
JPEG = b"\xff\xd8\xff\xd9"
PIPER_MANIFEST = Path(__file__).resolve().parents[1] / "agilex/piper/driver.yaml"


class ThreeViewCaptureTests(unittest.TestCase):
    def setUp(self):
        self.node = Mock()
        self.ros2 = SimpleNamespace(ctx_core=object(), executor_core=Mock())
        rclpy = ModuleType("rclpy")
        rclpy.__path__ = []
        rclpy_node = ModuleType("rclpy.node")
        rclpy_node.Node = Mock(return_value=self.node)
        qos = ModuleType("rclpy.qos")
        qos.DurabilityPolicy = SimpleNamespace(TRANSIENT_LOCAL=object())
        qos.ReliabilityPolicy = SimpleNamespace(RELIABLE=object())
        qos.QoSProfile = Mock()
        qos.qos_profile_sensor_data = object()
        sensor_msgs = ModuleType("sensor_msgs")
        sensor_msgs.__path__ = []
        sensor_msgs_msg = ModuleType("sensor_msgs.msg")
        sensor_msgs_msg.CompressedImage = object()
        ros_modules = patch.dict(sys.modules, {
            "rclpy": rclpy, "rclpy.node": rclpy_node, "rclpy.qos": qos,
            "sensor_msgs": sensor_msgs, "sensor_msgs.msg": sensor_msgs_msg,
        })
        ros_modules.start()
        self.addCleanup(ros_modules.stop)
        clock = patch("agilex.piper.three_view_card.time_ns", return_value=NOW_MS * 1_000_000)
        clock.start()
        self.addCleanup(clock.stop)
        self.publishers = {view: Mock() for view in ("front", "left", "right")}
        self.node.create_publisher.side_effect = list(self.publishers.values())
        self.plugin = ThreeViewCapture("piper_test", self.ros2)
        self.bundle = DriverBundle([self.plugin])
        self.addCleanup(self.bundle.stop_all)

    def call(self, action):
        return self.bundle.dispatch(TOOL_NAME, {"action": action})

    def feed(self, view, delta_ms, data=JPEG):
        topic = self.plugin.sources[view]
        callback = next(
            call.args[2] for call in self.node.create_subscription.call_args_list
            if call.args[1] == topic
        )
        sec, ms = divmod(NOW_MS + delta_ms, 1000)
        message = SimpleNamespace(
            header=SimpleNamespace(stamp=SimpleNamespace(sec=sec, nanosec=ms * 1_000_000)),
            data=data,
        )
        callback(message)
        return message

    def test_tool_manifest_and_lifecycle(self):
        [definition] = self.bundle.get_all_tools()
        self.assertEqual(definition["name"], TOOL_NAME)
        self.assertEqual(definition["type"], "actuator")
        self.assertFalse(definition["inputSchema"]["x-is-dangerous"])
        self.assertIn("capture", definition["inputSchema"]["properties"]["action"]["enum"])
        self.assertEqual(len(definition["topic_out"]), 3)
        self.assertTrue(all(port["format"] == "image/jpeg" for port in definition["topic_out"]))
        manifest = yaml.safe_load(PIPER_MANIFEST.read_text(encoding="utf-8"))
        self.assertIn({"name": TOOL_NAME, "type": "actuator"}, manifest["cards"])

        self.assertEqual(self.call("info")["state"], "idle")
        self.assertEqual(self.call("capture")["status"], "inactive")
        self.assertEqual(self.call("start")["state"], "ready")
        self.assertEqual(self.call("start")["state"], "ready")
        self.assertEqual(self.call("info")["state"], "running")
        self.ros2.executor_core.add_node.assert_called_once_with(self.node)
        self.assertEqual(self.node.create_subscription.call_count, 3)
        self.assertEqual(self.call("stop")["state"], "idle")
        self.assertEqual(self.call("stop")["state"], "idle")
        self.assertEqual(self.call("capture")["status"], "inactive")
        self.ros2.executor_core.remove_node.assert_called_once_with(self.node)
        self.node.destroy_node.assert_called_once()

    def test_unsupported_action_returns_none(self):
        self.assertIsNone(self.call("unsupported_action"))
        self.assertEqual(self.call("info")["state"], "idle")

    def test_missing_views(self):
        self.call("start")
        self.feed("front", -40)
        self.assertEqual(self.call("capture"), {
            "status": "missing_frames", "missing_views": ["left", "right"], "skew_ms": None,
        })
        for publisher in self.publishers.values():
            publisher.publish.assert_not_called()

    def test_publishes_closest_triplet_and_replays_preview(self):
        self.call("start")
        front = self.feed("front", -90)
        left = self.feed("left", -80)
        right = self.feed("right", -70)
        self.feed("front", -15)

        result = self.call("capture")
        self.assertEqual(result["status"], "ready")
        self.assertEqual(result["skew_ms"], 20)
        self.assertEqual(result["timestamps_ms"], {
            "front": NOW_MS - 90, "left": NOW_MS - 80, "right": NOW_MS - 70,
        })
        for view, message in zip(("front", "left", "right"), (front, left, right)):
            self.publishers[view].publish.assert_called_once_with(message)

        preview = self.node.create_timer.call_args.args[1]
        preview()
        self.call("stop")
        preview()
        for publisher in self.publishers.values():
            self.assertEqual(publisher.publish.call_count, 2)

    def test_out_of_sync_group_is_not_published(self):
        self.call("start")
        self.feed("front", -200)
        self.feed("left", -100)
        self.feed("right", -50)
        result = self.call("capture")
        self.assertEqual(result["status"], "out_of_sync")
        self.assertEqual(result["skew_ms"], 150)
        for publisher in self.publishers.values():
            publisher.publish.assert_not_called()

    def test_stale_or_invalid_input_does_not_fill_view(self):
        self.call("start")
        self.feed("front", -2001)
        self.feed("left", -50, b"\xff\xd8missing-end-marker")
        self.feed("left", -40, b"not a JPEG")
        self.feed("right", -25)
        result = self.call("capture")
        self.assertEqual(result["status"], "missing_frames")
        self.assertEqual(result["missing_views"], ["front", "left"])


class ThreeViewBundleRegistrationTests(unittest.TestCase):
    def test_bundle_registers_the_actuator_only_when_enabled(self):
        from agilex.piper.device import build_plugins

        with tempfile.TemporaryDirectory() as data_dir:
            config = {"data_dir": data_dir}
            default_bundle = DriverBundle(build_plugins(config, "piper_test", None))
            enabled_bundle = DriverBundle(build_plugins(
                {**config, "three_view_capture": {"enabled": True}}, "piper_test", None
            ))
        default_tools = {tool["name"]: tool for tool in default_bundle.get_all_tools()}
        enabled_tools = {tool["name"]: tool for tool in enabled_bundle.get_all_tools()}
        self.assertNotIn(TOOL_NAME, default_tools)
        self.assertEqual(set(enabled_tools) - set(default_tools), {TOOL_NAME})
        self.assertEqual(enabled_tools[TOOL_NAME]["type"], "actuator")
