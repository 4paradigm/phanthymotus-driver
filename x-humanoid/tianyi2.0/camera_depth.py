#!/usr/bin/env python3
"""天轶2.0 Orbbec 深度相机原子卡片。

机器人在 ROS domain 0 发布深度帧。本插件仅保留最新帧，
Agent Core domain 上发布紧密排列、小端的 ``sensor_msgs/Image``。
``image/depth-z16`` 消费者可将 data 直接解读为以毫米为单位的
uint16 数组。
"""

import threading
import time

from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import Image


_DEPTH_SOURCE_TOPIC = "/ob_camera_head/depth/image_raw"
_DEPTH_ENCODINGS = {"16uc1", "mono16"}

_DEPTH_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    durability=DurabilityPolicy.VOLATILE,
)


class CameraDepthPlugin:
    """将天轶 Orbbec 最新 Z16 深度帧转发给 Agent Core。"""

    def __init__(self, plugin_config: dict, namespace: str, ros2):
        del plugin_config
        self._topic = f"/{namespace}/camera/depth"
        self._sub_node = Node(
            "tianyi2_camera_depth_sub", context=ros2.ctx_tianyi
        )
        ros2.executor_tianyi.add_node(self._sub_node)
        self._pub_node = Node(
            "tianyi2_camera_depth_pub", context=ros2.ctx_core
        )
        ros2.executor_core.add_node(self._pub_node)

        self._running = False
        self._subscription = None
        self._publisher = None
        self._worker = None
        self._worker_stop = None

        self._lifecycle_lock = threading.Lock()
        self._frame_lock = threading.Lock()
        self._frame_ready = threading.Event()
        self._latest_frame = None
        self._last_warning_at = 0.0

    def get_tool(self) -> dict:
        return {
            "name": "camera_depth",
            "type": "sensor",
            "multiInstance": False,
            "description": (
                "天轶2.0 头部 Orbbec 深度相机 — 16位深度图，"
                "每个像素表示毫米距离"
            ),
            "inputSchema": {"type": "object", "properties": {}},
            "topic_out": [
                {"topic": self._topic, "format": "image/depth-z16"}
            ],
        }

    def start(self) -> None:
        """启动深度桥接；重复调用不会创建重复资源。"""
        with self._lifecycle_lock:
            if self._running:
                return

            self._publisher = self._pub_node.create_publisher(
                Image, self._topic, _DEPTH_QOS
            )
            self._subscription = self._sub_node.create_subscription(
                Image,
                _DEPTH_SOURCE_TOPIC,
                self._on_depth_frame,
                _DEPTH_QOS,
            )
            self._running = True
            self._worker_stop = threading.Event()
            self._worker = threading.Thread(
                target=self._publish_loop,
                args=(self._worker_stop,),
                name="tianyi2_camera_depth",
                daemon=True,
            )
            self._worker.start()

        print(
            f"[CameraDepthPlugin] {_DEPTH_SOURCE_TOPIC} -> {self._topic}",
            flush=True,
        )

    def stop(self) -> None:
        """停止发布，并释放本卡片创建的订阅和发布器。"""
        with self._lifecycle_lock:
            if not self._running:
                return
            self._running = False
            self._worker_stop.set()
            self._frame_ready.set()
            if self._worker is not None:
                self._worker.join()
            if self._subscription is not None:
                self._sub_node.destroy_subscription(self._subscription)
                self._subscription = None
            if self._publisher is not None:
                self._pub_node.destroy_publisher(self._publisher)
                self._publisher = None
            self._worker = None
            self._worker_stop = None

        with self._frame_lock:
            self._latest_frame = None
        self._frame_ready.clear()

    def _on_depth_frame(self, msg: Image) -> None:
        """回调只替换最新帧，格式转换留给后台线程。"""
        with self._frame_lock:
            if not self._running:
                return
            self._latest_frame = msg
        self._frame_ready.set()

    def _publish_loop(self, worker_stop: threading.Event) -> None:
        while not worker_stop.is_set():
            if not self._frame_ready.wait(timeout=0.25):
                continue
            self._frame_ready.clear()
            if worker_stop.is_set():
                return

            with self._frame_lock:
                frame = self._latest_frame
                self._latest_frame = None
            if frame is None:
                continue

            try:
                output = self._to_depth_z16(frame)
                if worker_stop.is_set():
                    return
                publisher = self._publisher
                if output is not None and publisher is not None:
                    publisher.publish(output)
            except Exception as exc:
                self._warn(f"depth frame conversion failed: {exc}")

    def _to_depth_z16(self, source: Image):
        """返回紧密小端 16UC1 图像；无效时返回 None。"""
        encoding = source.encoding.strip().lower()
        if encoding not in _DEPTH_ENCODINGS:
            self._warn(
                f"unsupported depth encoding {source.encoding!r}; "
                "expected 16UC1 or mono16"
            )
            return None

        width = int(source.width)
        height = int(source.height)
        row_bytes = width * 2
        source_step = int(source.step)
        if width <= 0 or height <= 0 or source_step < row_bytes:
            self._warn(
                f"invalid depth layout: {width}x{height}, step={source_step}"
            )
            return None

        raw = bytes(source.data)
        required_bytes = source_step * height
        if len(raw) < required_bytes:
            self._warn(
                f"short depth frame: got {len(raw)} bytes, "
                f"need {required_bytes}"
            )
            return None

        if source_step == row_bytes:
            packed = raw[:required_bytes]
        else:
            packed_buffer = bytearray(row_bytes * height)
            for row in range(height):
                src_start = row * source_step
                dst_start = row * row_bytes
                packed_buffer[dst_start : dst_start + row_bytes] = raw[
                    src_start : src_start + row_bytes
                ]
            packed = bytes(packed_buffer)

        if bool(source.is_bigendian):
            little_endian = bytearray(packed)
            little_endian[0::2], little_endian[1::2] = (
                little_endian[1::2],
                little_endian[0::2],
            )
            packed = bytes(little_endian)

        output = Image()
        output.header.stamp = source.header.stamp
        output.header.frame_id = source.header.frame_id
        output.height = height
        output.width = width
        output.encoding = "16UC1"
        output.is_bigendian = 0
        output.step = row_bytes
        output.data = packed
        return output

    def _warn(self, message: str) -> None:
        """限制重复相机错误的日志频率。"""
        now = time.monotonic()
        if now - self._last_warning_at < 5.0:
            return
        self._last_warning_at = now
        self._sub_node.get_logger().warning(f"[CameraDepthPlugin] {message}")

    def dispatch(self, action: str, args: dict) -> dict:
        del args
        if action == "start":
            self.start()
            return self._status()
        if action == "stop":
            self.stop()
            return {"state": "idle"}
        if action in {"info", "camera_depth"}:
            return self._status()
        return {"error": f"unknown action: {action}"}

    def _status(self) -> dict:
        return {
            "state": "running" if self._running else "idle",
            "topic_out": [
                {"topic": self._topic, "format": "image/depth-z16"}
            ],
        }
