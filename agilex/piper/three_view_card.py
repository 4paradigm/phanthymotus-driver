"""Publish live views from the three existing ALOHA JPEG streams."""

from threading import RLock
from time import time_ns

from common.vendor_runtime import tool


TOOL_NAME = "piper_three_view_capture"
LIVE_NAMES = {"front": "front_live", "left": "left_wrist_live", "right": "right_wrist_live"}
MAX_AGE_MS = 2000
MAX_CLOCK_LEAD_MS = 100
MAX_JPEG_BYTES = 4 * 1024 * 1024


class ThreeViewLive:
    def __init__(self, namespace, ros2):
        self.ros2 = ros2
        self.outputs = {
            view: f"/{namespace}/piper/three_view/{name}" for view, name in LIVE_NAMES.items()
        }
        self.node_name = f"piper_three_view_{namespace}"
        self._lock = RLock()
        self._node = None

    def get_tool(self):
        definition = tool(
            TOOL_NAME, "sensor", "Live video from the front and both wrist cameras.",
            topic_out=[
                {"topic": topic, "format": "image/jpeg", "description": f"{view} live"}
                for view, topic in self.outputs.items()
            ],
        )
        definition["title"] = "Three-View Live"
        return definition

    def start(self):
        with self._lock:
            if self._node is not None or self.ros2 is None or self.ros2.ctx_core is None:
                return

            from rclpy.node import Node
            from rclpy.qos import qos_profile_sensor_data
            from sensor_msgs.msg import CompressedImage

            node = Node(self.node_name, context=self.ros2.ctx_core)
            try:
                for view, output in self.outputs.items():
                    publisher = node.create_publisher(CompressedImage, output, qos_profile_sensor_data)
                    node.create_subscription(
                        CompressedImage, f"/aloha_lab/camera/{view}/jpeg",
                        lambda message, publisher=publisher: self._publish(node, publisher, message),
                        qos_profile_sensor_data,
                    )
                self.ros2.executor_core.add_node(node)
            except BaseException:
                node.destroy_node()
                raise
            self._node = node

    def stop(self):
        with self._lock:
            node, self._node = self._node, None
        if node is not None:
            try:
                self.ros2.executor_core.remove_node(node)
            finally:
                node.destroy_node()

    def _publish(self, node, publisher, message):
        stamp = message.header.stamp
        timestamp_ms = stamp.sec * 1000 + stamp.nanosec // 1_000_000
        age_ms = time_ns() // 1_000_000 - timestamp_ms
        data = message.data
        if timestamp_ms <= 0 or not -MAX_CLOCK_LEAD_MS <= age_ms <= MAX_AGE_MS:
            return
        if not 4 <= len(data) <= MAX_JPEG_BYTES:
            return
        if bytes(data[:2]) != b"\xff\xd8" or bytes(data[-2:]) != b"\xff\xd9":
            return

        with self._lock:
            if self._node is node:
                publisher.publish(message)

    def dispatch(self, action, args):
        if action == "start":
            self.start()
        elif action == "stop":
            # The live stream belongs to the driver, independent of the canvas project.
            return {"state": "idle"}
        elif action != "info":
            return None
        with self._lock:
            return {"state": "running" if self._node is not None else "idle"}
