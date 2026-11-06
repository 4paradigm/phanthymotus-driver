"""Real OpenCV JPEG analysis with isolated ROS handles and deterministic clocks.

These are transport unit tests, not real ROS2 pub/sub or Bumi experiments.
test_image_quality_ros.py exercises the actual rclpy transport separately.
"""
from __future__ import annotations

import json
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest

import cv2
import numpy as np

BUMI = Path(__file__).resolve().parents[1]
if str(BUMI) not in sys.path:
    sys.path.insert(0, str(BUMI))
from image_quality import ImageQualityAnalyzer
from image_quality_runtime import ImageQualityRosRuntime


def texture(low=80, high=176):
    return np.where(np.indices((96, 128)).sum(axis=0) % 2, high, low).astype(np.uint8)


def jpeg(image):
    ok, data = cv2.imencode(".jpg", image)
    if not ok:
        raise AssertionError("OpenCV JPEG encoding failed")
    return data.tobytes()


class Clock:
    def __init__(self):
        self.now = 0.0

    def __call__(self):
        return self.now


class Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class Node:
    instances = []

    def __init__(self, name, **kwargs):
        self.name, self.kwargs = name, kwargs
        self.publishers, self.subscriptions, self.timers = [], [], []
        self.destroyed = False
        self.destroy_sub_failures = 0
        Node.instances.append(self)

    def create_publisher(self, kind, topic, qos):
        publisher = Publisher()
        self.publishers.append((kind, topic, qos, publisher))
        return publisher

    def create_subscription(self, kind, topic, callback, qos, **kwargs):
        subscription = SimpleNamespace(callback=callback)
        self.subscriptions.append((kind, topic, qos, subscription, kwargs))
        return subscription

    def create_timer(self, period, callback, **kwargs):
        timer = SimpleNamespace(callback=callback, cancelled=False)
        def cancel():
            timer.cancelled = True
        timer.cancel = cancel
        self.timers.append((period, timer, kwargs))
        return timer

    def destroy_subscription(self, subscription):
        if self.destroy_sub_failures:
            self.destroy_sub_failures -= 1
            raise RuntimeError("subscription close failure")
        self.subscriptions = [s for s in self.subscriptions if s[3] is not subscription]
        return True

    def destroy_publisher(self, publisher):
        self.publishers = [p for p in self.publishers if p[3] is not publisher]
        return True

    def destroy_timer(self, timer):
        self.timers = [t for t in self.timers if t[1] is not timer]
        return True

    def destroy_node(self):
        self.destroyed = True


class Executor:
    context = None
    def __init__(self):
        self.nodes = []
        self.fail_add = False

    def add_node(self, node):
        self.nodes.append(node)
        if self.fail_add:
            raise RuntimeError("add_node failed after acquiring ownership")

    def remove_node(self, node):
        if node in self.nodes:
            self.nodes.remove(node)


def bindings():
    return SimpleNamespace(
        ok=lambda **kwargs: True, Node=Node,
        Clock=lambda **kwargs: SimpleNamespace(**kwargs),
        ClockType=SimpleNamespace(STEADY_TIME="steady"),
        TimerGroup=object, ReceiveGroup=object,
        QoSProfile=lambda **kwargs: SimpleNamespace(**kwargs),
        ReliabilityPolicy=SimpleNamespace(BEST_EFFORT="best_effort", RELIABLE="reliable"),
        HistoryPolicy=SimpleNamespace(KEEP_LAST="keep_last"),
        DurabilityPolicy=SimpleNamespace(VOLATILE="volatile"),
        CompressedImage=type("CompressedImage", (), {}),
        String=lambda **kwargs: SimpleNamespace(**kwargs))


class RuntimeTests(unittest.TestCase):
    def setUp(self):
        self.clock = Clock()
        self.executor = Executor()
        self.analyzer = ImageQualityAnalyzer()
        self.accept = True
        self.ros = bindings()
        self.runtime = ImageQualityRosRuntime(
            namespace="robot", executor=self.executor, ros_bindings=self.ros,
            monotonic=self.clock, wall_time=lambda: 1760000000.125)
        self.addCleanup(self.runtime.close)

    def start(self):
        self.runtime.start(input_topic="/robot/camera/color",
                           output_topic="/robot/camera/color/image_quality_guard",
                           run_id="epoch.run", analysis_hz=2.0,
                           run_if_current=self.gate)
        self.publisher = self.runtime._publisher

    def gate(self, operation):
        if not self.accept:
            return False
        operation(self.analyzer)
        return True

    def sample(self, image=None, *, data=None, image_format="jpeg"):
        self.runtime._on_image(SimpleNamespace(
            data=jpeg(image) if image is not None else data, format=image_format))
        self.runtime._tick()
        self.clock.now += .5
        return json.loads(self.publisher.messages[-1].data) if self.publisher.messages else None

    def test_constructor_is_inert(self):
        self.assertEqual(self.executor.nodes, [])
        self.assertIsNone(self.runtime._node)

    def test_actual_qos_topics_and_steady_timer_contract(self):
        self.start()
        node = self.executor.nodes[0]
        kind, topic, qos, _, _ = node.subscriptions[0]
        self.assertIs(kind, self.ros.CompressedImage)
        self.assertEqual(topic, "/robot/camera/color")
        self.assertEqual(vars(qos), dict(reliability="best_effort", history="keep_last",
                                         depth=1, durability="volatile"))
        self.assertEqual(node.publishers[0][1], "/robot/camera/color/image_quality_guard")
        self.assertEqual(node.publishers[0][2].reliability, "reliable")
        self.assertEqual(node.timers[0][0], .5)
        self.assertEqual(node.timers[0][2]["clock"].clock_type, "steady")

    def test_clear_normal_jpeg_requires_real_five_sample_warmup(self):
        self.start()
        for _ in range(4):
            self.assertEqual(self.sample(texture())["state"], "UNKNOWN")
        result = self.sample(texture())
        self.assertEqual(result["state"], "HEALTHY")
        self.assertEqual(result["reasons"], [])
        self.assertEqual(result["schema"], "motus.image_quality/1")
        self.assertEqual(result["run_id"], "epoch.run")
        self.assertEqual(result["timestamp_ms"], 1760000000125)
        self.assertEqual(result["timestamp_kind"], "analysis_time")

    def test_real_low_light_jpeg(self):
        self.start()
        for _ in range(3):
            result = self.sample(texture(5, 35))
        self.assertEqual(result["reasons"], ["LOW_LIGHT"])

    def test_real_overexposed_jpeg(self):
        self.start()
        for _ in range(3):
            result = self.sample(texture(241, 255))
        self.assertEqual(result["reasons"], ["OVEREXPOSED"])

    def test_real_blurred_jpeg(self):
        self.start()
        blurred = cv2.GaussianBlur(texture(), (15, 15), 3)
        for _ in range(3):
            result = self.sample(blurred)
        self.assertEqual(result["reasons"], ["BLURRED"])

    def test_real_black_jpeg_has_multiple_reasons(self):
        self.start()
        for _ in range(3):
            result = self.sample(np.zeros((96, 128), dtype=np.uint8))
        self.assertEqual(set(result["reasons"]), {"LOW_LIGHT", "BLURRED"})

    def test_invalid_jpeg_keeps_confirmed_reason_and_null_metrics(self):
        self.start()
        for _ in range(3):
            self.sample(texture(5, 35))
        result = self.sample(data=b"broken")
        self.assertEqual(result["state"], "UNKNOWN")
        self.assertEqual(result["reasons"], ["LOW_LIGHT"])
        self.assertIsNone(result["metrics"])
        self.assertEqual(result["error"]["code"], "INVALID_JPEG")

    def test_bad_format_decode_failure_and_empty_input(self):
        self.start()
        for data, fmt, code in ((jpeg(texture()), "png", "UNSUPPORTED_FORMAT"),
                                (b"\xff\xd8bad\xff\xd9", "jpeg", "DECODE_FAILED"),
                                (b"", "jpeg", "INVALID_JPEG")):
            with self.subTest(code=code):
                result = self.sample(data=data, image_format=fmt)
                self.assertEqual(result["state"], "UNKNOWN")
                self.assertIsNone(result["metrics"])
                self.assertEqual(result["error"]["code"], code)

    def test_monotonic_limit_ignores_rapid_callbacks(self):
        self.start()
        self.sample(texture())
        self.clock.now = .1
        for _ in range(20):
            self.runtime._on_image(SimpleNamespace(data=jpeg(texture()), format="jpeg"))
            self.runtime._tick()
        self.assertEqual(len(self.publisher.messages), 1)
        self.clock.now = .499
        self.runtime._tick()
        self.assertEqual(len(self.publisher.messages), 1)
        self.clock.now = .5
        self.runtime._tick()
        self.assertEqual(len(self.publisher.messages), 2)
        self.assertEqual(self.runtime.info()["effective_analysis_hz"], 2.0)

    def test_latest_frame_replaces_old_without_processing_backlog(self):
        self.start()
        self.runtime._on_image(SimpleNamespace(data=jpeg(texture(5, 35)), format="jpeg"))
        self.runtime._on_image(SimpleNamespace(data=jpeg(texture(241, 255)), format="jpeg"))
        self.runtime._tick()
        result = json.loads(self.publisher.messages[-1].data)
        self.assertEqual(result["candidate_reasons"], ["OVEREXPOSED"])
        self.assertEqual(self.runtime.info()["frames_overwritten"], 1)

    def test_no_new_frame_does_not_repeat_analysis_or_publication(self):
        self.start()
        self.sample(texture())
        for _ in range(12):
            self.runtime._tick()
            self.clock.now += .5
        self.assertEqual(len(self.publisher.messages), 1)
        self.assertEqual(self.runtime.info()["analyses_published"], 1)

    def test_rejected_old_generation_neither_analyzes_nor_publishes(self):
        self.start()
        self.accept = False
        self.sample(texture(5, 35))
        self.assertEqual(self.publisher.messages, [])
        self.assertIsNone(self.analyzer.snapshot()["metrics"])

    def test_close_invalidates_saved_callbacks_and_releases_owned_resources(self):
        self.start()
        node = self.executor.nodes[0]
        callback = node.subscriptions[0][3].callback
        timer_callback = node.timers[0][1].callback
        self.runtime.close()
        callback(SimpleNamespace(data=jpeg(texture()), format="jpeg"))
        timer_callback()
        self.assertEqual(self.publisher.messages, [])
        self.assertEqual(self.executor.nodes, [])
        self.assertTrue(node.destroyed)
        self.assertEqual((node.subscriptions, node.publishers, node.timers), ([], [], []))
        self.runtime.close()

    def test_cleanup_failure_retains_only_failed_resource_and_allows_retry(self):
        self.start()
        node = self.executor.nodes[0]
        node.destroy_sub_failures = 1
        with self.assertRaisesRegex(RuntimeError, "subscription close failure"):
            self.runtime.close()
        self.assertFalse(node.destroyed)
        self.assertIsNotNone(self.runtime._subscription)
        self.assertIsNone(self.runtime._publisher)
        self.assertIsNone(self.runtime._timer)
        self.assertEqual(self.executor.nodes, [])
        self.runtime.close()
        self.assertTrue(node.destroyed)

    def test_partial_add_failure_can_be_cleaned(self):
        self.executor.fail_add = True
        with self.assertRaisesRegex(RuntimeError, "add_node failed"):
            self.start()
        self.runtime.close()
        self.assertEqual(self.executor.nodes, [])
        self.assertIsNone(self.runtime._node)

    def test_json_is_finite_and_contains_no_original_image(self):
        self.start()
        for _ in range(3):
            result = self.sample(texture(5, 35))
        encoded = self.publisher.messages[-1].data
        self.assertEqual(json.loads(json.dumps(result, allow_nan=False)), result)
        self.assertLess(len(encoded), 4096)
        self.assertNotIn("base64", encoded)
        self.assertNotIn("data", result)

    def test_runtime_never_initializes_or_shuts_down_shared_executor(self):
        self.start()
        sentinel = object()
        self.executor.nodes.append(sentinel)
        self.runtime.close()
        self.assertEqual(self.executor.nodes, [sentinel])


if __name__ == "__main__":
    unittest.main()
