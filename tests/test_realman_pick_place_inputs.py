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

    def feed(self, value=120, objects=None, metadata=None, metadata_first=False):
        m = calibration(self.now) if metadata is None else metadata
        if metadata_first:
            self.inputs.receive("metadata", types.SimpleNamespace(data=json.dumps(m)))
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
            self.inputs.receive(name, msg)
        if not metadata_first:
            self.inputs.receive("metadata", types.SimpleNamespace(data=json.dumps(m)))
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

    def test_ros_binding_is_idempotent_and_stop_removes_only_own_subscriptions(self):
        import sys
        node = mock.Mock()
        ros = types.SimpleNamespace(ctx_core=object(), executor_core=mock.Mock())
        inputs = ObservationInputs(ros)
        modules = {"rclpy.node": types.SimpleNamespace(Node=mock.Mock(return_value=node)),
                   "rclpy.qos": types.SimpleNamespace(qos_profile_sensor_data=object()),
                   "sensor_msgs.msg": types.SimpleNamespace(CompressedImage=object),
                   "std_msgs.msg": types.SimpleNamespace(String=object)}
        with mock.patch.dict(sys.modules, modules):
            inputs.start({"input_topics": TOPICS[::-1]})
            inputs.start({"input_topics": TOPICS})
        calls = node.create_subscription.call_args_list
        self.assertEqual([c.args[1] for c in calls], [*TOPICS, TOPICS[1] + "/metadata"])
        node.create_publisher.assert_not_called()
        ros.executor_core.add_node.assert_called_once_with(node)
        callbacks = [c.args[2] for c in calls]
        inputs.stop()
        ros.executor_core.remove_node.assert_called_once_with(node)
        node.destroy_node.assert_called_once()
        for callback in callbacks:
            callback(types.SimpleNamespace())
        self.assertTrue(all(not b for b in inputs._buffers.values()))
        self.assertEqual(inputs._errors, {})

    def test_stop_cancels_blocked_ros_setup_without_resurrecting_node(self):
        import sys
        for stage in ("construct", "subscribe", "register"):
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
                {"construct": constructor, "subscribe": node.create_subscription,
                 "register": executor.add_node}[stage].side_effect = block
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
                    if stage == "register":
                        executor.remove_node.assert_called_once_with(node)
                    else:
                        executor.remove_node.assert_not_called()
                    constructor.side_effect = node.create_subscription.side_effect = None
                    executor.add_node.side_effect = None
                    inputs.start({"input_topics": TOPICS})
                    self.assertIs(inputs._node, node)
                    inputs.stop()

    def test_metadata_can_arrive_before_images_without_changing_capture_time(self):
        def reordered_feed():
            self.now += .05
            self.feed(metadata_first=True)
        self.cancel.wait = lambda _: reordered_feed()
        result = self.snapshot()
        metadata = result["source_calibration"]
        self.assertEqual(result["captured_at"], metadata["rgb_stamp_ns"] / 1e9)
        self.assertNotEqual(result["captured_at"], metadata["rgb_header_stamp_ns"] / 1e9)

    def test_recent_publication_cannot_make_pre_settle_capture_usable(self):
        def delayed():
            self.inputs._buffers["metadata"].clear()
            metadata = calibration(self.now)
            metadata.update(rgb_stamp_ns=999_990_000_000, depth_stamp_ns=999_991_000_000)
            self.feed(metadata=metadata)
        self.on_wait = delayed
        with self.assertRaisesRegex(RuntimeError, "timed out"):
            self.snapshot(timeout=.9)
        self.align.assert_not_called()

    def test_images_must_match_metadata_headers_exactly(self):
        def mismatch():
            self.inputs._buffers["rgb"][-1]["stamp_ns"] += 1
        self.on_wait = mismatch
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

    def test_mismatched_camera_skew_and_calibration_are_rejected(self):
        for edit in (
            lambda m: m.update(rgb_topics=["/other/rgb"]),
            lambda m: m.update(depth_stamp_ns=m["depth_stamp_ns"] - 300_000_000),
            lambda m: m.update(depth_scale_m=0.0001),
            lambda m: m["rgb_intrinsics"].update(fx=0),
        ):
            with self.subTest(edit=edit):
                metadata = calibration(self.now)
                edit(metadata)
                self.inputs.receive("metadata", types.SimpleNamespace(data=json.dumps(metadata)))
                self.assertEqual(self.inputs.info()["state"], "error")
                self.feed()

    def test_consumer_rejects_old_or_future_acquisition_with_fresh_publication(self):
        for name in ("rgb_stamp_ns", "depth_stamp_ns"):
            for seconds in (998, 1001):
                with self.subTest(name=name, seconds=seconds):
                    metadata = calibration(self.now)
                    metadata[name] = seconds * 1_000_000_000
                    self.inputs.receive("metadata", types.SimpleNamespace(data=json.dumps(metadata)))
                    self.assertEqual(self.inputs.info()["state"], "error")
                    self.assertIn("timestamp", self.inputs.info()["error"])
                    self.feed()

    def test_calibration_cannot_be_applied_to_another_session(self):
        self.on_wait = lambda: self.feed(metadata={**calibration(self.now), "session_id": "other-session"})
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
