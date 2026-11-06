"""Actual rclpy/DDS tests with synthetic JPEG publishers, never a robot camera.

Run via run_image_quality_ros_tests.sh. The test domain is isolated and uses
loopback only. Without rclpy these tests are explicitly skipped, never PASS.
"""
from __future__ import annotations

import importlib.util
import json
import math
from pathlib import Path
import sys
import time
import unittest
import uuid

import cv2
import numpy as np

BUMI = Path(__file__).resolve().parents[1]
if str(BUMI) not in sys.path:
    sys.path.insert(0, str(BUMI))
from image_quality import ImageQualityAnalyzer
from image_quality_runtime import ImageQualityRosRuntime

HAS_ROS = all(importlib.util.find_spec(name) is not None
              for name in ("rclpy", "sensor_msgs", "std_msgs"))


def texture(low=80, high=176):
    return np.where(np.indices((96, 128)).sum(axis=0) % 2, high, low).astype(np.uint8)


@unittest.skipUnless(HAS_ROS, "real ROS2 Python packages are unavailable")
class RosNodeTests(unittest.TestCase):
    def test_single_owned_node_is_added_and_removed_without_context_shutdown(self):
        import rclpy
        from rclpy.context import Context
        from rclpy.executors import MultiThreadedExecutor
        context = Context()
        rclpy.init(context=context)
        executor = MultiThreadedExecutor(num_threads=2, context=context)
        runtime = ImageQualityRosRuntime(namespace="iqg_test", executor=executor)
        try:
            runtime.start(input_topic="/iqg_test/input", output_topic="/iqg_test/output",
                          run_id=str(uuid.uuid4()) + "." + str(uuid.uuid4()),
                          analysis_hz=2.0, run_if_current=lambda operation: False)
            self.assertEqual(len(executor.get_nodes()), 1)
            runtime.close()
            runtime.close()
            self.assertEqual(executor.get_nodes(), [])
            self.assertTrue(context.ok())
        finally:
            runtime.close()
            executor.shutdown(timeout_sec=2)
            context.shutdown()


@unittest.skipUnless(HAS_ROS, "real ROS2 Python packages are unavailable")
class RosPubSubTests(unittest.TestCase):
    def setUp(self):
        import rclpy
        from rclpy.context import Context
        from rclpy.executors import MultiThreadedExecutor
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
        from sensor_msgs.msg import CompressedImage
        from std_msgs.msg import String
        self.CompressedImage = CompressedImage
        self.context = Context()
        rclpy.init(context=self.context)
        self.executor = MultiThreadedExecutor(num_threads=3, context=self.context)
        self.source = rclpy.create_node("iqg_test_" + uuid.uuid4().hex[:10], context=self.context)
        self.executor.add_node(self.source)
        self.input_topic = "/iqg_test_" + uuid.uuid4().hex + "/camera/color"
        self.output_topic = self.input_topic + "/image_quality_guard"
        self.received = []
        self.publisher = self.source.create_publisher(CompressedImage, self.input_topic,
            QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                       history=HistoryPolicy.KEEP_LAST, depth=1,
                       durability=DurabilityPolicy.VOLATILE))
        self.subscription = self.source.create_subscription(
            String, self.output_topic,
            lambda message: self.received.append((time.monotonic(), json.loads(message.data))), 10)
        self.analyzer = ImageQualityAnalyzer({"confirm_samples": 1, "recover_samples": 1})
        self.current = True
        self.runtime = ImageQualityRosRuntime(namespace="iqg_test", executor=self.executor)
        self.addCleanup(self.cleanup_ros)
        self.run_id = str(uuid.uuid4()) + "." + str(uuid.uuid4())
        self.runtime.start(input_topic=self.input_topic, output_topic=self.output_topic,
                           run_id=self.run_id, analysis_hz=2.0, run_if_current=self.gate)
        self.spin_until(lambda: self.publisher.get_subscription_count() > 0, 5)

    def gate(self, operation):
        if not self.current:
            return False
        operation(self.analyzer)
        return True

    def cleanup_ros(self):
        self.current = False
        self.runtime.close()
        self.executor.remove_node(self.source)
        self.source.destroy_node()
        self.executor.shutdown(timeout_sec=2)
        self.context.shutdown()

    def spin_until(self, predicate, timeout=3):
        deadline = time.monotonic() + timeout
        while not predicate() and time.monotonic() < deadline:
            self.executor.spin_once(timeout_sec=.02)
        self.assertTrue(predicate(), "ROS2 condition did not settle before timeout")

    def spin_for(self, seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            self.executor.spin_once(timeout_sec=.02)

    def send(self, image=None, *, payload=None, image_format="jpeg"):
        if image is not None:
            ok, encoded = cv2.imencode(".jpg", image)
            self.assertTrue(ok)
            payload = encoded.tobytes()
        message = self.CompressedImage()
        message.format = image_format
        message.data = payload or b""
        self.publisher.publish(message)

    def next_result(self, image=None, **kwargs):
        before = len(self.received)
        self.send(image, **kwargs)
        self.spin_until(lambda: len(self.received) > before)
        return self.received[-1][1]

    def test_real_dds_jpeg_quality_and_json_message_contract(self):
        samples = ((texture(), []), (texture(5, 35), ["LOW_LIGHT"]),
                   (texture(241, 255), ["OVEREXPOSED"]),
                   (cv2.GaussianBlur(texture(), (15, 15), 3), ["BLURRED"]),
                   (np.zeros((96, 128), dtype=np.uint8), ["LOW_LIGHT", "BLURRED"]))
        for image, reasons in samples:
            with self.subTest(reasons=reasons):
                result = self.next_result(image)
                self.assertEqual(result["reasons"], reasons)
                self.assertEqual(result["state"], "DEGRADED" if reasons else "HEALTHY")
                self.assertEqual(result["run_id"], self.run_id)
                self.assertEqual(result["input_topic"], self.input_topic)
                self.assertEqual(result["timestamp_kind"], "analysis_time")
                json.dumps(result, allow_nan=False)
        result = self.next_result(payload=b"invalid JPEG")
        self.assertEqual(result["state"], "UNKNOWN")
        self.assertIsNone(result["metrics"])
        self.assertEqual(result["error"]["code"], "INVALID_JPEG")

    def test_no_new_frame_means_no_repeated_dds_output(self):
        self.next_result(texture())
        count = len(self.received)
        self.spin_for(1.1)
        self.assertEqual(len(self.received), count)

    def test_actual_best_effort_match_and_latest_frame(self):
        from rclpy.qos import ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
        qos = self.runtime._subscription.qos_profile
        self.assertEqual(qos.reliability, ReliabilityPolicy.BEST_EFFORT)
        self.assertEqual(qos.history, HistoryPolicy.KEEP_LAST)
        self.assertEqual(qos.depth, 1)
        self.assertEqual(qos.durability, DurabilityPolicy.VOLATILE)
        # Both inputs arrive before the next timer; keep only the latest one.
        self.send(texture(5, 35))
        self.send(texture(241, 255))
        self.spin_until(lambda: len(self.received) > 0)
        self.assertEqual(self.received[-1][1]["candidate_reasons"], ["OVEREXPOSED"])

    def test_high_input_rate_is_limited_to_two_analysis_starts_per_second(self):
        start = time.monotonic()
        while time.monotonic() - start < 1.6:
            self.send(texture())
            self.executor.spin_once(timeout_sec=.01)
        self.spin_for(.15)
        count = len(self.received)
        self.assertGreaterEqual(count, 1)
        self.assertLessEqual(count, math.ceil((time.monotonic() - start) * 2) + 1)
        self.assertLessEqual(self.runtime.info()["effective_analysis_hz"], 2.01)

    def test_close_removes_endpoints_and_saved_callback_cannot_publish(self):
        self.next_result(texture())
        old_callback = self.runtime._tick
        self.runtime.close()
        count = len(self.received)
        self.send(texture(5, 35))
        old_callback()
        self.spin_for(.65)
        self.assertEqual(len(self.received), count)
        self.assertNotIn(self.runtime._node, self.executor.get_nodes())

    def test_real_guard_restart_rejects_original_stop_and_old_callbacks(self):
        from image_quality_guard import ImageQualityGuardPlugin
        self.runtime.close()
        card = ImageQualityGuardPlugin({"confirm_samples": 1, "recover_samples": 1},
                                       "iqg_test", self.executor)
        self.addCleanup(card.shutdown)
        status = card.lifecycle_status()
        epoch = str(uuid.uuid4())
        self.assertEqual(card.bind_epoch(driver_boot_id=status["driver_boot_id"],
                                         expected_epoch=None, proposed_epoch=epoch)["admission_epoch"], epoch)
        first = epoch + "." + str(uuid.uuid4())
        second = epoch + "." + str(uuid.uuid4())
        binding = {"input_topic": self.input_topic, "instance_id": "same-canvas-card"}
        self.assertEqual(card.dispatch("start", {**binding, "run_id": first})["state"], "running")
        old_runtime = card._runtime
        self.spin_until(lambda: self.publisher.get_subscription_count() > 0)
        before = len(self.received)
        self.send(texture(5, 35))
        self.spin_until(lambda: len(self.received) > before)
        self.assertEqual(self.received[-1][1]["run_id"], first)
        self.assertEqual(card.dispatch("stop", {"run_id": first})["state"], "idle")
        self.assertEqual(card.dispatch("start", {**binding, "run_id": second})["state"], "running")
        self.assertEqual(card.dispatch("stop", {"run_id": first})["code"], "STALE_RUN")
        self.assertEqual(card.dispatch("stop", {})["code"], "MISSING_RUN_ID")
        old_runtime._on_image(self.CompressedImage())
        old_runtime._tick()
        self.assertEqual(card.info()["run_id"], second)
        self.spin_until(lambda: self.publisher.get_subscription_count() > 0)
        before = len(self.received)
        self.send(texture(241, 255))
        self.spin_until(lambda: any(value[1]["run_id"] == second for value in self.received[before:]))
        self.assertEqual(self.received[-1][1]["reasons"], ["OVEREXPOSED"])
        self.assertEqual(card.dispatch("stop", {"run_id": second})["state"], "idle")
        self.assertEqual(len(self.executor.get_nodes()), 1)


if __name__ == "__main__":
    unittest.main()
