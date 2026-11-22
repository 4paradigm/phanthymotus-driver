"""Real ROS 2 DDS integration test for the Food Segmentation Card.

Run only in a sourced ROS 2 Humble environment::

    source /opt/ros/humble/setup.bash
    python3 -m pytest tests/bumi_food_segmentation/test_ros2_integration.py -q

It uses an in-process fake segmentation runtime, but real rclpy nodes,
publishers, subscriptions, executor, CompressedImage and String messages.
"""

from __future__ import annotations

import json
import os
import threading
import time
import uuid

import numpy as np
import pytest


try:
    import cv2
    import rclpy
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import (
        DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy,
    )
    from sensor_msgs.msg import CompressedImage
    from std_msgs.msg import String
except ImportError as exc:
    ROS_IMPORT_ERROR = str(exc)
else:
    ROS_IMPORT_ERROR = None

from noetix.bumi.food_segmentation import FoodSegmentationPlugin
from noetix.bumi.food_segmentation_contract import COCO_FOOD_CLASSES, RawInstance


class FakeRuntime:
    def __init__(self):
        self.names = {0: "person", **COCO_FOOD_CLASSES}
        self.load_count = 0

    def load(self, _model_path, **_options):
        self.load_count += 1

    def predict(self, image):
        mask = np.zeros(image.shape[:2], dtype=np.uint8)
        mask[2:6, 2] = 1
        mask[5, 2:7] = 1  # non-rectangular L mask
        return [RawInstance(47, 0.9, [2, 2, 7, 6], mask.tolist())]


def _wait(predicate, timeout=5.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.02)
    raise AssertionError("ROS 2 condition was not reached before timeout")


@pytest.mark.skipif(
    ROS_IMPORT_ERROR is not None,
    reason=f"requires sourced ROS 2 Humble and OpenCV: {ROS_IMPORT_ERROR}",
)
def test_real_ros2_dds_lifecycle_header_and_outputs():
    suffix = uuid.uuid4().hex[:10]
    input_topic = f"/food_segmentation_test_{suffix}/input"
    metadata_topic = f"/food_segmentation_test_{suffix}/metadata"
    overlay_topic = f"/food_segmentation_test_{suffix}/overlay"
    runtime = FakeRuntime()
    executor = None
    test_node = None
    plugin = None
    spin_thread = None

    rclpy.init()
    try:
        executor = MultiThreadedExecutor(num_threads=4)
        plugin = FoodSegmentationPlugin({
            "model_path": "/local-test/fake-segmentation.pt",
            "input_topic": input_topic,
            "metadata_topic": metadata_topic,
            "overlay_topic": overlay_topic,
            "max_rate_hz": 30,
        }, suffix, executor, runtime_factory=lambda: runtime)
        test_node = Node(f"food_segmentation_test_client_{suffix}")
        executor.add_node(test_node)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.VOLATILE,
        )
        metadata_messages = []
        overlay_messages = []
        output_event = threading.Event()

        def on_metadata(message):
            metadata_messages.append(message)
            if overlay_messages:
                output_event.set()

        def on_overlay(message):
            overlay_messages.append(message)
            if metadata_messages:
                output_event.set()

        test_node.create_subscription(String, metadata_topic, on_metadata, qos)
        test_node.create_subscription(
            CompressedImage, overlay_topic, on_overlay, qos)
        input_publisher = test_node.create_publisher(
            CompressedImage, input_topic, qos)

        spin_thread = threading.Thread(target=executor.spin, daemon=True)
        spin_thread.start()
        assert plugin.start(input_topic)
        assert plugin.start(input_topic)  # idempotent
        _wait(lambda: test_node.count_subscribers(input_topic) >= 1)

        source = np.zeros((12, 16, 3), dtype=np.uint8)
        source[:, :, 1] = 140
        ok, encoded = cv2.imencode(".jpg", source)
        assert ok
        message = CompressedImage()
        message.header.stamp.sec = 1234
        message.header.stamp.nanosec = 5678
        message.header.frame_id = "integration_camera"
        message.format = "jpeg"
        message.data = encoded.tobytes()

        deadline = time.monotonic() + 5
        while not output_event.is_set() and time.monotonic() < deadline:
            input_publisher.publish(message)
            output_event.wait(0.1)
        assert output_event.is_set()
        metadata = json.loads(metadata_messages[-1].data)
        overlay = overlay_messages[-1]
        assert metadata["frame_timestamp"] == {"sec": 1234, "nanosec": 5678}
        assert metadata["processed_instance_count"] == 1
        assert metadata["instances"][0]["class_name"] == "apple"
        assert metadata["instances"][0]["mask_area_px"] == 8
        assert overlay.header.frame_id == "integration_camera"
        assert overlay.header.stamp.sec == 1234
        assert overlay.header.stamp.nanosec == 5678
        assert overlay.format == "jpeg" and bytes(overlay.data).startswith(b"\xff\xd8")

        observe_seconds = float(os.environ.get(
            "FOOD_SEGMENTATION_ROS2_OBSERVE_SECONDS", "0"))
        if observe_seconds > 0:
            observe_seconds = min(observe_seconds, 300.0)
            print(
                "ROS2 observation window topics:",
                input_topic, metadata_topic, overlay_topic,
                f"({observe_seconds:g}s)",
                flush=True,
            )
            time.sleep(observe_seconds)

        assert plugin.stop()
        stopped_counts = (len(metadata_messages), len(overlay_messages))
        for _ in range(3):
            input_publisher.publish(message)
            time.sleep(0.05)
        assert (len(metadata_messages), len(overlay_messages)) == stopped_counts

        output_event.clear()
        assert plugin.start(input_topic)
        _wait(lambda: test_node.count_subscribers(input_topic) >= 1)
        deadline = time.monotonic() + 5
        while not output_event.is_set() and time.monotonic() < deadline:
            input_publisher.publish(message)
            output_event.wait(0.1)
        assert output_event.is_set()
        assert runtime.load_count == 2
        assert plugin.stop()
        assert plugin.stop()
    finally:
        if plugin is not None:
            plugin.stop()
        if executor is not None and test_node is not None:
            executor.remove_node(test_node)
            test_node.destroy_node()
        if executor is not None:
            executor.shutdown(timeout_sec=2)
        if rclpy.ok():
            rclpy.shutdown()
        if spin_thread is not None:
            spin_thread.join(2)
