"""Capture a time-matched JPEG from each of the three existing ALOHA streams."""

from collections import deque
from itertools import product
from threading import RLock
from time import time_ns

from agilex.piper.three_view_capture import VIEWS, check_frame_group
from common.vendor_runtime import action_schema, tool


TOOL_NAME = "piper_three_view_capture"
MAX_SKEW_MS = 100
MAX_AGE_MS = 2000
MAX_JPEG_BYTES = 4 * 1024 * 1024


class ThreeViewCapture:
    def __init__(self, namespace, ros2):
        self.ros2 = ros2
        self.sources = {view: f"/aloha_lab/camera/{view}/jpeg" for view in VIEWS}
        self.outputs = {view: f"/{namespace}/piper/three_view/{view}/jpeg" for view in VIEWS}
        self.node_name = f"piper_three_view_{namespace}"
        self._lock = RLock()
        self._node = None
        self._publishers = {}
        self._frames = {view: deque(maxlen=12) for view in VIEWS}
        self._last_capture = {}

    def _ports(self):
        return [{"topic": self.outputs[view], "format": "image/jpeg", "description": view}
                for view in VIEWS]

    def get_tool(self):
        schema = action_schema({
            "capture": ([], "采集一组时间接近的前方、左腕、右腕照片"),
            "info": ([], "查看取证状态"),
            "start": ([], "订阅三路相机"),
            "stop": ([], "停止订阅并清除取证预览"),
        }, {})
        schema["properties"]["action"]["default"] = "capture"
        schema["properties"]["action"]["oneOf"] = [
            {"const": name, "title": label} for name, label in (
                ("capture", "取证"), ("info", "查看状态"), ("start", "开始订阅"), ("stop", "停止订阅")
            )
        ]
        schema["x-is-dangerous"] = False
        definition = tool(TOOL_NAME, "actuator", "复用 ALOHA 三路 JPEG 图像，按时间戳完成工位取证。",
                          schema, topic_out=self._ports())
        definition["title"] = "三视角工位取证"
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
                for view in VIEWS:
                    node.create_subscription(
                        CompressedImage, self.sources[view],
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
        return max(stamps) - min(stamps), -min(stamps)

    def _capture(self):
        with self._lock:
            if self._node is None:
                return {"status": "inactive", "message": "请先开始订阅相机。"}

            fresh = self._fresh_frames(time_ns() // 1_000_000)
            if not all(fresh.values()):
                return check_frame_group({view: frames[-1][0] for view, frames in fresh.items() if frames})

            matches = min(product(*(fresh[view] for view in VIEWS)), key=self._group_order)
            timestamps = {view: frame[0] for view, frame in zip(VIEWS, matches)}
            result = check_frame_group(timestamps, MAX_SKEW_MS)
            result["timestamps_ms"] = timestamps
            if result["status"] == "ready":
                capture = {view: frame[1] for view, frame in zip(VIEWS, matches)}
                for view, message in capture.items():
                    self._publishers[view].publish(message)
                self._last_capture = capture
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
            raise ValueError(f"不支持的三视角取证操作：{action}")
        with self._lock:
            return {
                "state": "running" if self._node is not None else "idle",
                "sources": self.sources,
                "topic_out": self._ports(),
            }
