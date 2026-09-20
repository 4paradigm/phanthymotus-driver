"""External input synchronization without ROS, physical cameras or arm movement."""

import copy
import json
import threading
import types
import unittest
from unittest import mock
import zlib

import cv2
import numpy as np
import test_realman_pick_place as fixtures
from pick_place.inputs import ObservationInputs, resolve_topics
from pick_place.alignment import decode_depth, validate_calibration


TOPICS = [
    "/robot/ext_camera/color/rgb",
    "/robot/ext_camera/range/depth",
    "/robot/ext_camera/color/rgb/objects",
]


def calibration(stamp=1000.0):
    def intr(w, h):
        return dict(
            width=w, height=h, fx=w, fy=h, ppx=w / 2, ppy=h / 2, model="distortion.none", coeffs=[0] * 5
        )

    return dict(
        version=2,
        serial_number="camera-a",
        session_id="session-a",
        rgb_topics=[TOPICS[0]],
        rgb_header_stamp_ns=round(stamp * 1e9),
        depth_header_stamp_ns=round(stamp * 1e9) + 1,
        rgb_frame_id="robot_rgb_optical",
        depth_frame_id="robot_depth_optical",
        rgb_stamp_ns=round((stamp - 0.02) * 1e9),
        depth_stamp_ns=round((stamp - 0.019) * 1e9),
        rgb_intrinsics=intr(80, 60),
        depth_intrinsics=intr(40, 30),
        depth_to_color=dict(rotation=[1, 0, 0, 0, 1, 0, 0, 0, 1], translation=[0.02, 0, 0]),
        depth_scale_m=0.001,
        depth_aligned_to="depth",
    )


class InputTests(unittest.TestCase):
    def setUp(self):
        self.inputs = ObservationInputs()
        self.inputs._topics = TOPICS[:]
        self.inputs._source_key = (("/", "robot_realsense_rgbd_camera", (1,)),) * 3
        self.inputs._calibration = calibration()
        self.inputs._session_id = "session-a"
        self.now = 1000.0
        self.enterContext(mock.patch("pick_place.inputs.time.time", side_effect=lambda: self.now))
        self.enterContext(mock.patch("pick_place.inputs.time.monotonic", side_effect=lambda: self.now))
        self.align = self.enterContext(
            mock.patch("pick_place.inputs.align_depth", return_value=np.full((60, 80), 400, dtype="<u2"))
        )
        self.cancel = threading.Event()
        self.cancel.wait = self.advance
        self.on_wait = lambda: None
        self.feed()

    def advance(self, seconds):
        self.now += seconds
        self.feed()
        self.on_wait()

    def feed(self, value=120, objects=None, metadata=None):
        m = calibration(self.now) if metadata is None else metadata
        jpeg = cv2.imencode(".jpg", np.full((60, 80, 3), value, np.uint8))[1].tobytes()
        for name, data, fmt in (
            ("rgb", jpeg, "jpeg"),
            (
                "depth",
                zlib.compress(np.full((30, 40), 400, dtype="<u2").tobytes()),
                "16UC1; compressedDepth zlib",
            ),
        ):
            sec, nano = divmod(m[name + "_header_stamp_ns"], 1_000_000_000)
            msg = types.SimpleNamespace(
                data=data,
                format=fmt,
                header=types.SimpleNamespace(
                    stamp=types.SimpleNamespace(sec=sec, nanosec=nano),
                    frame_id="robot_" + name + "_optical",
                ),
            )
            self.inputs.receive(name, msg, message_info={"source_timestamp": m[name + "_stamp_ns"]})
        if objects is None:
            objects = [dict(name="banana", position=[0.2, -0.1], confidence=0.8)]
        payload = dict(timestamp=self.now, count=len(objects), objects=objects, latency_ms=10)
        self.inputs.receive("objects", types.SimpleNamespace(data=json.dumps(payload)))

    def snapshot(self, **kwargs):
        return self.inputs.snapshot(1000.0, self.cancel, lambda: None, **kwargs)

    def test_connects_by_stream_semantics_not_connection_order(self):
        self.assertEqual(resolve_topics({"input_topics": TOPICS[::-1], "input_topic": TOPICS[2]}), TOPICS)
        self.assertEqual([x["topic"] for x in self.inputs.topics()], [TOPICS[1], TOPICS[0], TOPICS[2]])
        self.assertTrue(self.inputs.info()["fresh"])
        for topics in (
            TOPICS[:2],
            TOPICS + [TOPICS[0]],
            [TOPICS[0], TOPICS[1], "/other/rgb/objects"],
            [TOPICS[0]] * 3,
        ):
            with self.subTest(topics=topics), self.assertRaises(ValueError):
                resolve_topics({"input_topics": topics})

    def test_ros_binding_is_idempotent_and_only_uses_three_private_subscriptions(self):
        import sys
        node = mock.Mock()
        ros = types.SimpleNamespace(ctx_core=object(), executor_core=mock.Mock())
        inputs = ObservationInputs(ros)
        modules = {"rclpy.node": types.SimpleNamespace(Node=mock.Mock(return_value=node)),
                   "rclpy.qos": types.SimpleNamespace(qos_profile_sensor_data=object()),
                   "sensor_msgs.msg": types.SimpleNamespace(CompressedImage=object),
                   "std_msgs.msg": types.SimpleNamespace(String=object)}
        finished = threading.Event()
        def reader(node, subscriptions, generation, stopped):
            stopped.wait(2)
            node.destroy_node()
            finished.set()
        with mock.patch.dict(sys.modules, modules), mock.patch.object(inputs, "_read", side_effect=reader):
            inputs.start({"input_topics": TOPICS[::-1]})
            inputs.start({"input_topics": TOPICS})
            calls = node.create_subscription.call_args_list
            self.assertEqual([c.args[1] for c in calls], TOPICS)
            node.create_publisher.assert_not_called()
            ros.executor_core.add_node.assert_not_called()
            inputs.stop()
            self.assertTrue(finished.wait(1))
        node.destroy_node.assert_called_once()
        ros.executor_core.remove_node.assert_not_called()
        self.assertEqual(inputs._errors, {})

    def test_stop_cancels_blocked_ros_setup_without_resurrecting_node(self):
        import sys
        for stage in ("construct", "subscribe"):
            with self.subTest(stage=stage):
                entered, release = threading.Event(), threading.Event()
                node, executor = mock.Mock(), mock.Mock()
                ros = types.SimpleNamespace(ctx_core=object(), executor_core=executor)
                inputs = ObservationInputs(ros)
                def block(*args, **kwargs):
                    entered.set()
                    if not release.wait(3):
                        raise RuntimeError("test setup release timed out")
                    return node
                constructor = mock.Mock(return_value=node)
                {"construct": constructor, "subscribe": node.create_subscription}[stage].side_effect = block
                modules = {"rclpy.node": types.SimpleNamespace(Node=constructor),
                           "rclpy.qos": types.SimpleNamespace(qos_profile_sensor_data=object()),
                           "sensor_msgs.msg": types.SimpleNamespace(CompressedImage=object),
                           "std_msgs.msg": types.SimpleNamespace(String=object)}
                with mock.patch.dict(sys.modules, modules):
                    worker = threading.Thread(target=inputs.start, args=({"input_topics": TOPICS},))
                    worker.start()
                    try:
                        self.assertTrue(entered.wait(1))
                        generation = inputs._generation
                        inputs.stop()
                        self.assertIsNone(inputs._node)
                        self.assertEqual(inputs.identity()["topics"], [])
                        inputs.receive("objects", types.SimpleNamespace(data="{}"), generation)
                        self.assertEqual(inputs._errors, {})
                    finally:
                        release.set()
                        worker.join(2)
                    self.assertFalse(worker.is_alive())
                    self.assertIsNone(inputs._node)
                    self.assertEqual(inputs.identity()["topics"], [])
                    node.destroy_node.assert_called_once()
                    executor.add_node.assert_not_called()
                    executor.remove_node.assert_not_called()

    def test_dds_source_timestamp_is_used_instead_of_image_header(self):
        result = self.snapshot()
        metadata = result["source_calibration"]
        self.assertEqual(result["captured_at"], metadata["rgb_stamp_ns"] / 1e9)
        self.assertNotEqual(result["captured_at"], metadata["rgb_header_stamp_ns"] / 1e9)
        self.assertEqual(result["synchronization"]["timestamp_source"], "dds_source_timestamp")
        self.assertTrue(result["synchronization"]["capture_time_approximate"])

    def test_recent_dds_publication_is_an_explicit_capture_time_approximation(self):
        # Header/capture age cannot prove when a republished image was acquired.
        # The accepted contract uses DDS publication time, not delivery time.
        def old_header():
            metadata = calibration(self.now)
            metadata.update(rgb_header_stamp_ns=1, depth_header_stamp_ns=2)
            self.inputs._buffers["rgb"].clear()
            self.inputs._buffers["depth"].clear()
            self.feed(metadata=metadata)
        self.on_wait = old_header
        result = self.snapshot()
        self.assertGreater(result["captured_at"], 1000.58)
        self.assertEqual(result["source_calibration"]["rgb_header_stamp_ns"], 1)
        self.assertTrue(result["synchronization"]["capture_time_approximate"])

    def test_pre_settle_dds_samples_are_not_usable(self):
        def delayed():
            self.inputs._buffers["rgb"].clear()
            self.inputs._buffers["depth"].clear()
            metadata = calibration(self.now)
            metadata.update(rgb_stamp_ns=999_990_000_000, depth_stamp_ns=999_991_000_000)
            self.feed(metadata=metadata)
        self.on_wait = delayed
        with self.assertRaisesRegex(RuntimeError, "timed out"):
            self.snapshot(timeout=.9)
        self.align.assert_not_called()

    def test_rgb_depth_pairing_requires_bounded_source_time_skew(self):
        def skewed():
            self.inputs._buffers["depth"].clear()
            self.inputs._buffers["depth"].append(dict(self.inputs._buffers["rgb"][-1],
                                                    stamp_ns=round((self.now - .8) * 1e9)))
        self.on_wait = skewed
        with self.assertRaisesRegex(RuntimeError, "timed out"):
            self.snapshot(timeout=1)
        self.align.assert_not_called()

    def test_snapshot_waits_for_post_settle_window_and_new_vop_results(self):
        result = self.snapshot()
        self.assertGreaterEqual(result["captured_at"], 1000.58)
        self.assertEqual(result["objects"][0]["name"], "banana")
        self.assertEqual(result["objects"][0]["position"], [0.2, -0.1])
        self.assertEqual(result["synchronization"]["mode"], "stationary_window")
        self.assertEqual(result["synchronization"]["rgb_depth_skew_ms"], 1)
        self.assertEqual(result["depth_scale_m"], 0.001)
        self.assertEqual(result["intrinsics"], calibration()["rgb_intrinsics"])
        self.assertEqual(decode_depth(result["depth_zlib"], 80, 60).shape, (60, 80))
        self.align.assert_called_once()

    def test_empty_detection_is_a_valid_observation(self):
        self.on_wait = lambda: self.feed(objects=[])
        self.assertEqual(self.snapshot()["objects"], [])

    def test_scene_change_restarts_window_and_uses_new_detections(self):
        def change_scene():
            if self.now > 1000.25:
                self.inputs._buffers["rgb"].pop()
                self.feed(value=220)

        self.on_wait = change_scene
        result = self.snapshot()
        self.assertGreater(result["synchronization"]["settled_after"], 1000.2)
        self.assertGreater(result["synchronization"]["window_restarts"], 0)
        self.assertGreater(result["captured_at"], 1000.8)
        self.align.assert_called_once()

    def test_continuous_scene_change_times_out_without_returning_stale_detection(self):
        def change():
            self.inputs._buffers["rgb"].pop()
            self.feed(value=220 if int(self.now * 20) % 2 else 120)
        self.on_wait = change
        with self.assertRaisesRegex(RuntimeError, "timed out"):
            self.snapshot(timeout=2)
        self.align.assert_not_called()

    def test_pose_recovery_discards_entire_observation_window(self):
        checks = []
        def check():
            if self.now > 1000.3 and not checks:
                checks.append(self.now)
                return False
            return True
        result = self.inputs.snapshot(1000, self.cancel, check)
        self.assertEqual(len(checks), 1)
        self.assertGreater(result["synchronization"]["settled_after"], checks[0])
        self.assertGreater(result["captured_at"], checks[0] + .7)
        self.assertGreater(result["objects_timestamp"], checks[0] + .7)

    def test_disturbance_during_alignment_discards_candidate_before_return(self):
        disturbed = []
        def check():
            if self.align.call_count == 1 and not disturbed:
                disturbed.append(self.now)
                return False
            return True
        result = self.inputs.snapshot(1000, self.cancel, check)
        self.assertEqual(len(disturbed), 1)
        self.assertEqual(self.align.call_count, 2)
        self.assertGreater(result["captured_at"], disturbed[0] + .7)

    def test_old_detection_does_not_become_fresh_from_delivery_time(self):
        def stale():
            self.inputs._buffers["objects"].clear()
            self.inputs.receive(
                "objects",
                types.SimpleNamespace(
                    data=json.dumps(dict(timestamp=999, count=0, objects=[], latency_ms=100))
                ),
            )

        self.on_wait = stale
        with self.assertRaisesRegex(RuntimeError, "timed out"):
            self.snapshot(timeout=1)
        self.align.assert_not_called()

    def test_inference_started_before_settling_is_not_selected(self):
        def old_inference():
            self.inputs._buffers["objects"].clear()
            self.inputs.receive(
                "objects",
                types.SimpleNamespace(
                    data=json.dumps(dict(timestamp=self.now, count=0, objects=[], latency_ms=2000))
                ),
            )

        self.on_wait = old_inference
        with self.assertRaisesRegex(RuntimeError, "timed out"):
            self.snapshot(timeout=1)

    def test_consumer_rejects_old_future_or_missing_dds_timestamp(self):
        for stamp in (998_000_000_000, 1001_000_000_000, None, 0, True):
            with self.subTest(stamp=stamp):
                self.inputs.receive("rgb", types.SimpleNamespace(), message_info={"source_timestamp": stamp})
                self.assertEqual(self.inputs.info()["state"], "error")
                self.assertIn("timestamp", self.inputs.info()["error"])
                self.now += .001
                self.feed()

    def test_input_gap_invalidates_saved_observation_identity(self):
        before = self.inputs.identity()
        self.now += 1.1
        self.assertFalse(self.inputs.info()["fresh"])
        self.assertNotEqual(before, self.inputs.identity())
        self.feed()
        self.assertTrue(self.inputs.info()["fresh"])
        self.assertNotEqual(before, self.inputs.identity())

    def test_calibration_is_queried_once_per_source_not_per_observation(self):
        self.inputs._calibration = None
        with mock.patch("pick_place.inputs.read_calibration", return_value=calibration()) as read:
            self.inputs._initialize_calibration(self.inputs._generation)
            self.snapshot()
            self.inputs._initialize_calibration(self.inputs._generation)
            self.snapshot()
        read.assert_called_once()

    def test_stop_during_calibration_discards_late_result(self):
        entered, release = threading.Event(), threading.Event()
        self.inputs._calibration = None
        def blocked(*args):
            entered.set()
            release.wait(2)
            return calibration()
        with mock.patch("pick_place.inputs.read_calibration", side_effect=blocked):
            worker = threading.Thread(target=self.inputs._initialize_calibration, args=(self.inputs._generation,))
            worker.start()
            try:
                self.assertTrue(entered.wait(1))
                self.inputs.stop()
                self.assertIsNone(self.inputs._calibration)
            finally:
                release.set()
                worker.join(1)
        self.assertIsNone(self.inputs._calibration)
        self.assertEqual(self.inputs.identity()["topics"], [])

    def test_publisher_replacement_invalidates_calibration_and_observation(self):
        node = mock.Mock()
        def endpoint(name="camera", gid=1):
            return types.SimpleNamespace(node_namespace="/", node_name=name, endpoint_gid=[gid])
        node.get_publishers_info_by_topic.side_effect = lambda t: [endpoint()]
        self.inputs._refresh_source(node, self.inputs._generation)
        self.inputs._calibration = calibration()
        self.feed()
        before = self.inputs.identity()
        node.get_publishers_info_by_topic.side_effect = lambda t: [endpoint(gid=2)]
        self.inputs._refresh_source(node, self.inputs._generation)
        self.assertNotEqual(before, self.inputs.identity())
        self.assertIsNone(self.inputs._calibration)
        self.assertTrue(all(not b for b in self.inputs._buffers.values()))
        for publishers in ([endpoint(), endpoint()], [endpoint("other")] ):
            node.get_publishers_info_by_topic.side_effect = lambda t: publishers if t == TOPICS[1] else [endpoint()]
            self.inputs._refresh_source(node, self.inputs._generation)
            self.assertEqual(self.inputs.info()["state"], "error")

    def test_calibration_failure_is_not_queried_every_frame(self):
        self.inputs._calibration = None
        with mock.patch("pick_place.inputs.read_calibration", side_effect=RuntimeError("camera unavailable")) as read:
            self.inputs._initialize_calibration(self.inputs._generation)
            self.feed()
            self.inputs._initialize_calibration(self.inputs._generation)
        read.assert_called_once()
        self.assertIn("calibration unavailable", self.inputs.info()["error"])

    def test_resolution_change_invalidates_saved_observation(self):
        before = self.inputs.identity()
        jpeg = cv2.imencode(".jpg", np.zeros((48, 64, 3), np.uint8))[1].tobytes()
        msg = types.SimpleNamespace(data=jpeg, format="jpeg", header=types.SimpleNamespace(
            frame_id="robot_rgb_optical", stamp=types.SimpleNamespace(sec=1000, nanosec=0)))
        self.inputs.receive("rgb", msg, message_info=types.SimpleNamespace(source_timestamp=1_000_000_000_000))
        self.assertNotEqual(before, self.inputs.identity())
        self.assertEqual(self.inputs.info()["state"], "error")
        self.assertIn("dimensions", self.inputs.info()["error"])

    def test_reader_preserves_dds_info_and_owns_node_destruction(self):
        node, sub = mock.Mock(), mock.MagicMock()
        stop = threading.Event()
        info = {"source_timestamp": 1_000_000_000_000}
        msg = object()
        sub.handle.take_message.side_effect = [(msg, info), None]
        def received(*args):
            stop.set()
        with mock.patch.object(self.inputs, "_refresh_source"), mock.patch.object(
                self.inputs, "_initialize_calibration"), mock.patch.object(
                self.inputs, "receive", side_effect=received) as receive:
            self.inputs._read(node, [("rgb", object, sub)], self.inputs._generation, stop)
        receive.assert_called_once_with("rgb", msg, self.inputs._generation, info)
        node.destroy_node.assert_called_once()

    def test_stop_during_take_does_not_destroy_a_live_subscription(self):
        node, sub = mock.Mock(), mock.MagicMock()
        entered, release, stopped = threading.Event(), threading.Event(), threading.Event()
        self.inputs._reader_stop = stopped
        self.inputs._node = node
        generation = self.inputs._generation
        def take(*args):
            entered.set()
            release.wait(2)
            return types.SimpleNamespace(), {"source_timestamp": 1_000_000_000_000}
        sub.handle.take_message.side_effect = take
        with mock.patch.object(self.inputs, "_refresh_source"), mock.patch.object(self.inputs, "_initialize_calibration"):
            worker = threading.Thread(target=self.inputs._read,
                                      args=(node, [("rgb", object, sub)], generation, stopped))
            worker.start()
            try:
                self.assertTrue(entered.wait(1))
                self.inputs.stop()
                node.destroy_node.assert_not_called()
            finally:
                release.set()
                worker.join(1)
        self.assertFalse(worker.is_alive())
        node.destroy_node.assert_called_once()
        self.assertEqual(self.inputs._errors, {})
        self.assertIsNone(self.inputs._node)

    def test_source_change_during_snapshot_is_rejected(self):
        def changed():
            with self.inputs._condition:
                self.inputs._reset_frames()
        self.on_wait = changed
        with self.assertRaisesRegex(RuntimeError, "source changed"):
            self.snapshot()

    def test_cancel_and_stop_do_not_acquire_or_replay(self):
        self.cancel.set()
        with self.assertRaisesRegex(RuntimeError, "cancelled"):
            self.snapshot()
        generation = self.inputs._generation
        self.inputs.stop()
        self.inputs.receive("objects", types.SimpleNamespace(data="{}"), generation)
        self.assertTrue(all(not b for b in self.inputs._buffers.values()))
        self.align.assert_not_called()

    def test_depth_decoding_has_exact_size_and_rejects_trailing_payload(self):
        raw = np.arange(12, dtype="<u2").reshape(3, 4)
        encoded = zlib.compress(raw.tobytes())
        np.testing.assert_array_equal(decode_depth(encoded, 4, 3), raw)
        for invalid in (encoded[:-2], encoded + b"extra", zlib.compress(b"x" * 10000)):
            with self.assertRaises(ValueError):
                decode_depth(invalid, 4, 3)

    def test_invalid_projection_metadata_is_rejected_before_sdk(self):
        for edit in (
            lambda m: m["depth_to_color"].update(rotation=[0] * 9),
            lambda m: m["depth_to_color"].update(translation=[0, float("nan"), 0]),
            lambda m: m["depth_intrinsics"].update(width=0),
            lambda m: m["rgb_intrinsics"].update(coeffs=[0] * 4),
        ):
            with self.subTest(edit=edit), self.assertRaises(ValueError):
                metadata = calibration()
                edit(metadata)
                validate_calibration(metadata)


if __name__ == "__main__":
    unittest.main()
