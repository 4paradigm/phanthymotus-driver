"""Capture a time-matched JPEG from each of the three existing ALOHA streams."""

from collections import deque
from itertools import product
from threading import RLock
from time import time_ns

from common.vendor_runtime import action_schema, tool


TOOL_NAME = "piper_three_view_capture"
OUTPUT_NAMES = {"front": "front_jpeg", "left": "left_wrist_jpeg", "right": "right_wrist_jpeg"}
MAX_SKEW_MS = 100
MAX_AGE_MS = 2000
MAX_JPEG_BYTES = 4 * 1024 * 1024


class ThreeViewCapture:
    def __init__(self, namespace, ros2):
        self.ros2 = ros2
        self.sources = {view: f"/aloha_lab/camera/{view}/jpeg" for view in OUTPUT_NAMES}
        self.outputs = {
            view: f"/{namespace}/piper/three_view/{name}" for view, name in OUTPUT_NAMES.items()
        }
        self.node_name = f"piper_three_view_{namespace}"
        self._lock = RLock()
        self._node = None
        self._publishers = {}
        self._frames = {view: deque(maxlen=12) for view in self.sources}
        self._last_capture = {}

    def _ports(self):
        return [{"topic": topic, "format": "image/jpeg", "description": view}
                for view, topic in self.outputs.items()]

    def get_tool(self):
        schema = action_schema({
            "capture": ([], "Capture time-matched JPEGs from the front and both wrist cameras."),
            "info": ([], "Show capture status."),
            "start": ([], "Subscribe to the three camera streams."),
            "stop": ([], "Unsubscribe and clear the capture preview."),
        }, {})
        schema["properties"]["action"]["default"] = "capture"
        schema["properties"]["action"]["oneOf"] = [
            {"const": name, "title": label} for name, label in (
                ("capture", "Capture"), ("info", "Status"), ("start", "Start"), ("stop", "Stop")
            )
        ]
        schema["x-is-dangerous"] = False
        definition = tool(TOOL_NAME, "actuator", "Capture synchronized JPEGs from the three ALOHA cameras.",
                          schema, topic_out=self._ports())
        definition["title"] = "Three-View Capture"
        return definition

    def start(self):
        with self._lock:
            if self._node is not None or self.ros2 is None or self.ros2.ctx_core is None:
                return
            from rclpy.node import Node
            from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
            from sensor_msgs.msg import CompressedImage

            node = Node(self.node_name, context=self.ros2.ctx_core)
            try:
                preview_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
                publishers = {}
                for view, source in self.sources.items():
                    node.create_subscription(
                        CompressedImage, source,
                        lambda message, view=view: self._receive(node, view, message),
                        qos_profile_sensor_data,
                    )
                    publishers[view] = node.create_publisher(CompressedImage, self.outputs[view], preview_qos)
                node.create_timer(1.0, lambda: self._preview(node))
                self.ros2.executor_core.add_node(node)
            except BaseException:
                node.destroy_node()
                raise
            self._node = node
            self._publishers = publishers

    def stop(self):
        with self._lock:
            node, self._node = self._node, None
            self._publishers = {}
            self._last_capture = {}
            for frames in self._frames.values():
                frames.clear()
        if node is not None:
            try:
                self.ros2.executor_core.remove_node(node)
            finally:
                node.destroy_node()

    def _receive(self, node, view, message):
        stamp = message.header.stamp
        timestamp_ms = stamp.sec * 1000 + stamp.nanosec // 1_000_000
        data = message.data
        if timestamp_ms <= 0 or not 4 <= len(data) <= MAX_JPEG_BYTES:
            return
        if bytes(data[:2]) != b"\xff\xd8" or bytes(data[-2:]) != b"\xff\xd9":
            return
        with self._lock:
            if self._node is node:
                self._frames[view].append((timestamp_ms, message))

    def _preview(self, node):
        with self._lock:
            if self._node is node:
                self._publish_capture()

    def _publish_capture(self):
        for view, message in self._last_capture.items():
            self._publishers[view].publish(message)

    def _fresh_frames(self, now_ms):
        return {
            view: [frame for frame in received if -MAX_SKEW_MS <= now_ms - frame[0] <= MAX_AGE_MS]
            for view, received in self._frames.items()
        }

    @staticmethod
    def _group_order(group):
        stamps = [frame[0] for frame in group]
        first = min(stamps)
        return max(stamps) - first, -first

    def _capture(self):
        with self._lock:
            if self._node is None:
                return {"status": "inactive", "message": "Start the camera subscriptions first."}

            fresh = self._fresh_frames(time_ns() // 1_000_000)
            missing = [view for view, frames in fresh.items() if not frames]
            if missing:
                return {"status": "missing_frames", "missing_views": missing, "skew_ms": None}

            matches = dict(zip(fresh, min(product(*fresh.values()), key=self._group_order)))
            timestamps = {view: frame[0] for view, frame in matches.items()}
            skew_ms = max(timestamps.values()) - min(timestamps.values())
            result = {
                "status": "out_of_sync", "missing_views": [], "skew_ms": skew_ms, "timestamps_ms": timestamps,
            }
            if skew_ms <= MAX_SKEW_MS:
                self._last_capture = {view: frame[1] for view, frame in matches.items()}
                self._publish_capture()
                result["status"] = "ready"
                result["topic_out"] = self._ports()
            return result

    def dispatch(self, action, args):
        if action == "capture":
            return self._capture()
        if action == "start":
            self.start()
        elif action == "stop":
            self.stop()
        elif action != "info":
            return None
        active_state = "ready" if action == "start" else "running"
        with self._lock:
            return {
                "state": active_state if self._node is not None else "idle",
                "sources": self.sources,
                "topic_out": self._ports(),
            }
