"""Optional calibration sideband preserves shared camera capture and image contracts."""

import builtins
import json
import types
import unittest
from unittest import mock

import numpy as np
import test_realman_realsense as fixtures
import realsense_metadata as metadata


class MetadataTests(unittest.TestCase):
    def setUp(self):
        self.sdk = types.SimpleNamespace(timestamp_domain=types.SimpleNamespace(
            system_time="system_time", global_time="global_time"))
        self.sideband = metadata.RGBDMetadata("robot", "serial", self.sdk)
        self.node = fixtures.FakeNode()
        self.status = {}
        self.routes = {"rgb-card": "rgb", "depth-card": "depth"}
        self.depth_topic = "/robot/ext_camera/depth_card/depth/metadata"
        self.clock = self.enterContext(mock.patch.object(metadata.time, "monotonic", return_value=100.0))
        self.enterContext(mock.patch.dict("sys.modules", {
            "rclpy.qos": types.SimpleNamespace(qos_profile_sensor_data=None),
            "std_msgs.msg": types.SimpleNamespace(String=types.SimpleNamespace),
        }))
        self.frames = {
            "rgb": fixtures.FakeFrame(np.zeros((720, 1280, 3), np.uint8), 99.98),
            "depth": fixtures.FakeFrame(np.zeros((480, 640), np.uint16), 99.981),
        }
        self.headers = {name: types.SimpleNamespace(
            stamp=types.SimpleNamespace(sec=100, nanosec=index), frame_id="robot_" + name + "_optical")
            for index, name in enumerate(self.frames)}

    def send(self):
        self.sideband.publish(self.node, self.routes, self.frames, self.headers, self.status)

    def test_acquisition_time_is_not_replaced_or_filtered_by_consumer_age_policy(self):
        for stamp in (98, 99.9, 101):
            with self.subTest(stamp=stamp):
                self.frames["rgb"].stamp = stamp
                self.assertEqual(metadata.frame_time_ns(self.frames["rgb"], self.sdk), round(stamp * 1e9))
        for stamp in (0, -1, float("nan"), float("inf")):
            self.frames["rgb"].stamp = stamp
            with self.assertRaises(ValueError):
                metadata.frame_time_ns(self.frames["rgb"], self.sdk)
        self.frames["rgb"].get_frame_timestamp_domain = lambda: "hardware"
        with self.assertRaisesRegex(ValueError, "host-synchronized"):
            metadata.frame_time_ns(self.frames["rgb"], self.sdk)

    def test_wire_metadata_keeps_version_units_profiles_identity_and_exact_headers(self):
        self.send()
        result = json.loads(self.node.publishers[self.depth_topic].messages[-1].data)
        self.assertEqual(result["version"], 2)
        self.assertEqual(result["rgb_intrinsics"]["width"], 1280)
        self.assertEqual(result["depth_intrinsics"]["width"], 640)
        self.assertEqual(result["depth_scale_m"], 0.001)
        self.assertEqual(result["depth_aligned_to"], "depth")
        self.assertEqual(result["depth_to_color"]["translation"], [0.02, 0, 0])
        self.assertEqual(result["rgb_topics"], ["/robot/ext_camera/rgb_card/rgb"])
        self.assertEqual(result["serial_number"], "serial")
        self.assertEqual(result["session_id"], self.sideband.session_id)
        self.assertEqual(result["rgb_stamp_ns"], 99_980_000_000)
        self.assertEqual(result["depth_stamp_ns"], 99_981_000_000)
        for index, name in enumerate(("rgb", "depth")):
            self.assertEqual(result[name + "_header_stamp_ns"], 100_000_000_000 + index)
            self.assertEqual(result[name + "_frame_id"], self.headers[name].frame_id)

    def test_cache_is_per_profile_and_session_but_timestamps_are_per_frame(self):
        with mock.patch.object(metadata, "calibration", wraps=metadata.calibration) as read:
            self.send()
            self.frames["rgb"] = fixtures.FakeFrame(np.zeros((720, 1280, 3), np.uint8), 100.5)
            self.send()
            self.assertEqual(read.call_count, 1)
            result = json.loads(self.node.publishers[self.depth_topic].messages[-1].data)
            self.assertEqual(result["rgb_stamp_ns"], 100_500_000_000)
            self.frames["rgb"] = fixtures.FakeFrame(np.zeros((480, 640, 3), np.uint8), 101)
            self.send()
            self.assertEqual(read.call_count, 2)
            result = json.loads(self.node.publishers[self.depth_topic].messages[-1].data)
            self.assertEqual(result["rgb_intrinsics"]["width"], 640)
            session = self.sideband.session_id
            self.sideband = metadata.RGBDMetadata("robot", "serial", self.sdk)
            self.send()
            self.assertEqual(read.call_count, 3)
            self.assertNotEqual(session, self.sideband.session_id)

    def test_no_listener_skips_sdk_and_serialization_then_join_leave_resume(self):
        with mock.patch.object(fixtures.FakePublisher, "get_subscription_count", return_value=0) as count, \
                mock.patch.object(metadata, "calibration", wraps=metadata.calibration) as read, \
                mock.patch.object(metadata.json, "dumps", wraps=json.dumps) as encode:
            self.send()
            read.assert_not_called()
            encode.assert_not_called()
            publisher = self.node.publishers[self.depth_topic]
            count.return_value = 1
            self.send()
            self.assertEqual(len(publisher.messages), 1)
            count.return_value = 0
            self.send()
            self.assertEqual(len(publisher.messages), 1)
            count.return_value = 1
            self.send()
            self.assertEqual(len(publisher.messages), 2)
            self.assertEqual(read.call_count, 1)

    def test_missing_rgb_or_depth_route_does_not_query_calibration(self):
        with mock.patch.object(metadata, "calibration", wraps=metadata.calibration) as read:
            for channel in ("rgb", "depth", "infrared"):
                self.routes = {"only": channel}
                self.headers = {channel: types.SimpleNamespace()}
                self.send()
            read.assert_not_called()

    def test_failed_publisher_creation_retries_without_stopping_existing_sideband(self):
        self.routes["other-depth"] = "depth"
        create = self.node.create_publisher
        def failing(kind, topic, qos):
            if "other_depth" in topic:
                raise RuntimeError("creation failed")
            return create(kind, topic, qos)
        with mock.patch.object(self.node, "create_publisher", side_effect=failing) as attempt:
            self.send()
            self.assertEqual(self.status["rgbd_error"], "creation failed")
            self.send()
            self.assertEqual(attempt.call_count, 2)
            self.assertEqual(self.status["rgbd_error"], "creation failed")
        self.clock.return_value += 1.1
        self.send()
        self.assertEqual(len(self.node.publishers[self.depth_topic].messages), 3)
        self.assertEqual(len(self.node.publishers["/robot/ext_camera/other_depth/depth/metadata"].messages), 1)
        self.assertNotIn("rgbd_error", self.status)

    def test_route_removal_does_not_publish_stale_messages_even_if_cleanup_fails(self):
        self.send()
        publisher = self.node.publishers[self.depth_topic]
        self.routes = {"rgb-card": "rgb"}
        with mock.patch.object(self.node, "destroy_publisher", side_effect=RuntimeError("cleanup failed")):
            self.send()
        self.assertEqual(len(publisher.messages), 1)
        self.assertEqual(self.status["rgbd_error"], "cleanup failed")
        self.clock.return_value += 1.1
        self.send()
        self.assertNotIn(self.depth_topic, self.node.publishers)
        self.routes["depth-card"] = "depth"
        self.send()
        self.assertEqual(len(self.node.publishers[self.depth_topic].messages), 1)

    def test_one_broken_publisher_does_not_block_other_sideband_consumers(self):
        self.routes["other-depth"] = "depth"
        self.send()
        broken = self.node.publishers[self.depth_topic]
        other = self.node.publishers["/robot/ext_camera/other_depth/depth/metadata"]
        for method in ("publish", "get_subscription_count"):
            with self.subTest(method=method), mock.patch.object(broken, method, side_effect=RuntimeError(method)):
                before = len(other.messages)
                self.send()
                self.assertEqual(len(other.messages), before + 1)
                self.assertEqual(self.status["rgbd_error"], method)

    def test_clock_setup_failure_is_isolated_per_sensor_and_enumeration(self):
        self.sdk.option = types.SimpleNamespace(global_time_enabled="global")
        broken, healthy = mock.Mock(), mock.Mock()
        broken.supports.side_effect = RuntimeError("unsupported")
        self.sideband.configure_clock(types.SimpleNamespace(query_sensors=lambda: [broken, healthy]))
        healthy.set_option.assert_called_once_with("global", 1)
        self.sideband.configure_clock(mock.Mock(query_sensors=mock.Mock(side_effect=RuntimeError("no sensors"))))
        self.assertEqual(self.sideband.error, "no sensors")


class CaptureIsolationTests(unittest.TestCase):
    def capture(self):
        return fixtures.RGBDCaptureTests().capture(
            [(.1, fixtures.rs.STREAMS), (.2, fixtures.rs.STREAMS), (.3, fixtures.rs.STREAMS)],
            routes={"rgb": "rgb", "depth": "depth", "ir": "infrared"})

    def test_sideband_failures_preserve_every_original_image_and_do_not_reconnect(self):
        create, publish = fixtures.FakeNode.create_publisher, fixtures.FakePublisher.publish
        def broken_create(node, kind, topic, qos):
            if topic.endswith("/metadata"):
                raise RuntimeError("create metadata failed")
            return create(node, kind, topic, qos)
        def broken_publish(publisher, message):
            if publisher.topic.endswith("/metadata"):
                raise RuntimeError("publish metadata failed")
            return publish(publisher, message)
        original_import = builtins.__import__
        def broken_import(name, *args, **kwargs):
            if name == "std_msgs.msg":
                raise ImportError("optional message unavailable")
            return original_import(name, *args, **kwargs)
        for failure in (
            mock.patch.object(fixtures.FakeNode, "create_publisher", broken_create),
            mock.patch.object(fixtures.FakePublisher, "publish", broken_publish),
            mock.patch.object(fixtures.FakePublisher, "get_subscription_count", side_effect=RuntimeError("discovery")),
            mock.patch.object(metadata, "calibration", side_effect=RuntimeError("calibration")),
            mock.patch.object(metadata, "frame_time_ns", side_effect=ValueError("timestamp")),
            mock.patch.object(metadata.json, "dumps", side_effect=TypeError("serialization")),
            mock.patch("builtins.__import__", side_effect=broken_import),
        ):
            with self.subTest(failure=failure), failure:
                errors, _, pipeline, _, node = self.capture()
                self.assertEqual(errors, [])
                self.assertEqual(pipeline.start.call_count, 1)
                self.assertEqual(pipeline.stop.call_count, 1)
                sideband = node.publishers.get("/robot_a/ext_camera/depth/depth/metadata")
                self.assertTrue(sideband is None or not sideband.messages)
                for key, channel in (("rgb", "rgb"), ("depth", "depth"), ("ir", "infrared")):
                    messages = node.publishers[f"/robot_a/ext_camera/{key}/{channel}"].messages
                    self.assertEqual(len(messages), 3)
                    for message in messages:
                        self.assertEqual(message.header.frame_id, f"robot_a_{channel}_optical")
                        self.assertEqual(message.header.stamp.sec, 100)
                        self.assertEqual(message.header.stamp.nanosec, 100_000_000)
                        if channel == "depth":
                            self.assertEqual(message.format, "16UC1; compressedDepth zlib")
                            self.assertEqual(message.data, fixtures.rs.encode_depth(np.ones((480, 640), np.uint16), .001))
                        else:
                            self.assertEqual(message.format, "jpeg")
                            self.assertEqual(message.data, bytes([1, 2, 3]))


if __name__ == "__main__":
    unittest.main()
