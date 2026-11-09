"""Local ROS 2/DDS smoke check with synthetic pixels; no camera or robot SDK.

Run after sourcing ROS Humble, with ROS_LOCALHOST_ONLY=1 and an isolated domain.
"""

from __future__ import annotations

import json
import os
import sys
import threading
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "noetix" / "bumi"))
from planar_rectify import PlanarRectifyPlugin


def main():
    if os.environ.get("ROS_LOCALHOST_ONLY") != "1" or os.environ.get("ROS_DOMAIN_ID") in (None, "0", "42"):
        raise RuntimeError("use localhost-only ROS with an isolated non-production domain")
    rclpy.init()
    executor = MultiThreadedExecutor(num_threads=2)
    node = Node("planar_rectify_synthetic_test")
    namespace = f"rectify_local_{os.getpid()}"
    source = f"/{namespace}/camera/color"
    outputs, reports = [], []
    node.create_subscription(CompressedImage, f"/{namespace}/planar_rectify/image",
                             outputs.append, qos_profile_sensor_data)
    node.create_subscription(String, f"/{namespace}/planar_rectify/state",
                             lambda msg: reports.append(json.loads(msg.data)), qos_profile_sensor_data)
    publisher = node.create_publisher(CompressedImage, source, qos_profile_sensor_data)
    executor.add_node(node)
    config = {"mode": "manual", "corners": [[0.1,0.1],[0.9,0.1],[0.9,0.9],[0.1,0.9]],
              "output_width": 320, "output_height": 240, "max_fps": 5}
    plugin = PlanarRectifyPlugin(config, namespace, executor)
    spin = threading.Thread(target=executor.spin, daemon=True)
    stopped = False
    try:
        plugin.start()
        spin.start()
        image = np.full((480, 640, 3), 240, np.uint8)
        cv2.putText(image, "SYNTHETIC", (100, 230), cv2.FONT_HERSHEY_SIMPLEX, 1, (20,20,20), 2)
        ok, data = cv2.imencode(".jpg", image)
        assert ok
        deadline, sent = time.monotonic() + 4, 0
        while time.monotonic() < deadline and not (outputs and any(r["valid"] for r in reports)):
            sent += 1
            msg = CompressedImage()
            msg.format = "jpeg"
            msg.header.stamp.sec = 1000 + sent
            msg.data = data.tobytes()
            publisher.publish(msg)
            time.sleep(0.1)
        assert outputs and any(r["valid"] for r in reports), "no real ROS output"
        received = outputs[-1]
        decoded = cv2.imdecode(np.frombuffer(received.data, np.uint8), cv2.IMREAD_COLOR)
        assert decoded.shape[:2] == (240, 320)
        assert 1001 <= received.header.stamp.sec <= 1000 + sent
        assert received.header.frame_id == "planar_rectified_pixels"
        deadline = time.monotonic() + 4
        while time.monotonic() < deadline and not any(r["status"] == "stale_input" for r in reports):
            time.sleep(0.05)
        assert reports[-1]["status"] == "stale_input" and not reports[-1]["valid"]
        plugin.stop()
        stopped = True
        deadline = time.monotonic() + 1
        while time.monotonic() < deadline and reports[-1]["status"] != "idle":
            time.sleep(0.02)
        assert reports[-1]["status"] == "idle" and not reports[-1]["valid"]
        assert plugin._thread is None and plugin._node is None
        assert executor.get_nodes() == [node]
        print(json.dumps({"result": "passed", "input_messages": sent,
                          "rectified_images": len(outputs), "status_messages": len(reports),
                          "stale_observed": True, "idle_observed": True,
                          "worker_and_node_released": True,
                          "source": "synthetic JPEG over local ROS 2/DDS"}))
    finally:
        if not stopped:
            plugin.stop()
        executor.shutdown()
        if spin.is_alive():
            spin.join(timeout=2)
        executor.remove_node(node)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
