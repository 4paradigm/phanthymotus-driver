"""Live SLAM map publisher for the As2W ``slam_mapping`` sensor card.

The vendor SLAM service publishes map-frame point clouds independently from
the ``slam_operate`` RPC service used by :mod:`controlled_spatial`.  This card
bridges those clouds to the mapping packet already consumed by Agent Core for
G1 and Go2.
"""

from __future__ import annotations

import array
import itertools
import json
import math
import queue
import struct
import threading
import time

from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import UInt8MultiArray
from unitree_sdk2py.core.channel import ChannelSubscriber
from unitree_sdk2py.idl.sensor_msgs.msg.dds_ import PointCloud2_
from unitree_sdk2py.idl.std_msgs.msg.dds_ import String_


_MAP_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    durability=DurabilityPolicy.VOLATILE,
)


class _SlamMappingNode:
    """Accumulate vendor SLAM clouds and publish bounded map snapshots."""

    _SOURCE_TOPICS = (
        "rt/unitree/slam_mapping/points",
        "rt/unitree/slam_relocation/points",
    )
    _PUBLISH_INTERVAL = 1.0

    def __init__(self, topic, executor, config):
        from rclpy.node import Node

        self.node = Node("as2w_slam_mapping")
        self.pub = self.node.create_publisher(UInt8MultiArray, topic, _MAP_QOS)
        self.topic = topic
        self._executor = executor
        self._voxel_size = max(0.02, float(config.get("voxel_size", 0.06)))
        self._max_input_points = max(1000, int(config.get("max_input_points", 20000)))
        self._max_map_points = max(1000, int(config.get("max_map_points", 50000)))
        self._max_buffer_points = max(
            self._max_map_points, int(config.get("max_buffer_points", 200000)))

        self._lock = threading.RLock()
        self._publish_lock = threading.Lock()
        self._voxels = {}
        self._pose = None
        self._map_status = "idle"
        self._frames = {source: 0 for source in self._SOURCE_TOPICS}
        self._published = 0
        self._last_cloud_time = None
        self._last_publish_time = 0.0
        self._last_error = ""
        self._closing = threading.Event()
        self._cloud_queue = queue.Queue(maxsize=1)
        self._worker = threading.Thread(
            target=self._process_loop, daemon=True, name="as2w_slam_mapping")
        self._worker.start()

        self._subs = []
        try:
            info_sub = ChannelSubscriber("rt/slam_info", String_)
            info_sub.Init(self._on_slam_info, 10)
            self._subs.append(info_sub)
        except Exception as exc:
            self._last_error = f"rt/slam_info subscription failed: {exc}"
            self.node.get_logger().warning(self._last_error)

        for source in self._SOURCE_TOPICS:
            try:
                sub = ChannelSubscriber(source, PointCloud2_)
                sub.Init(lambda message, source=source: self._on_cloud(source, message), 1)
                self._subs.append(sub)
                self.node.get_logger().info(f"As2W SLAM mapping listening on {source}")
            except Exception as exc:
                self._last_error = f"{source} subscription failed: {exc}"
                self.node.get_logger().warning(self._last_error)

        self._timer = self.node.create_timer(1.0, self._heartbeat)
        executor.add_node(self.node)

    def close(self):
        if self._closing.is_set():
            return
        self._closing.set()
        try:
            self._timer.cancel()
        except Exception:
            pass
        for sub in self._subs:
            try:
                sub.Close()
            except Exception:
                pass
        self._worker.join(timeout=2.0)
        try:
            self._executor.remove_node(self.node)
        except Exception:
            pass
        self.node.destroy_node()

    def status(self):
        with self._lock:
            age_ms = (-1 if self._last_cloud_time is None else
                      round((time.monotonic() - self._last_cloud_time) * 1000))
            return {
                "state": "running" if not self._closing.is_set() else "idle",
                "map_status": self._map_status,
                "pose_available": self._pose is not None,
                "voxel_points": len(self._voxels),
                "source_frames": dict(self._frames),
                "published": self._published,
                "last_cloud_ago_ms": age_ms,
                "last_error": self._last_error,
            }

    def _on_slam_info(self, message):
        try:
            payload = json.loads(message.data)
            message_type = payload.get("type", "")
            if message_type not in ("mapping_info", "pos_info"):
                return
            pose = payload.get("data", {}).get("currentPose")
            if not pose:
                return
            qx = float(pose.get("q_x", 0.0))
            qy = float(pose.get("q_y", 0.0))
            qz = float(pose.get("q_z", 0.0))
            qw = float(pose.get("q_w", 1.0))
            yaw = math.atan2(
                2.0 * (qw * qz + qx * qy),
                1.0 - 2.0 * (qy * qy + qz * qz),
            )
            with self._lock:
                previous = self._map_status
                self._pose = {
                    "x": float(pose.get("x", 0.0)),
                    "y": float(pose.get("y", 0.0)),
                    "yaw": yaw,
                }
                if message_type == "mapping_info" or previous == "mapping":
                    self._map_status = "mapping"
                else:
                    self._map_status = "localized"
                # A localized -> mapping transition denotes a fresh mapping
                # session. Do not mix the preceding relocation cloud into it.
                if previous == "localized" and self._map_status == "mapping":
                    self._voxels.clear()
            self._publish(force=False)
        except (AttributeError, TypeError, ValueError) as exc:
            with self._lock:
                self._last_error = f"invalid slam_info: {exc}"

    def _on_cloud(self, source, message):
        if self._closing.is_set():
            return
        try:
            data = bytes(message.data)
            point_step = int(message.point_step)
            point_count = int(message.width) * int(message.height)
            if point_step < 12 or point_count <= 0 or len(data) < point_step:
                return
            offsets = {field.name: int(field.offset) for field in message.fields}
            item = (source, data, point_step, point_count, offsets,
                    bool(getattr(message, "is_bigendian", False)))
            try:
                self._cloud_queue.put_nowait(item)
            except queue.Full:
                try:
                    self._cloud_queue.get_nowait()
                except queue.Empty:
                    pass
                self._cloud_queue.put_nowait(item)
        except Exception as exc:
            with self._lock:
                self._last_error = f"cloud enqueue failed: {exc}"

    def _process_loop(self):
        while not self._closing.is_set():
            try:
                item = self._cloud_queue.get(timeout=0.5)
            except queue.Empty:
                continue
            source, data, point_step, point_count, offsets, bigendian = item
            try:
                points = self._decode_points(
                    data, point_step, point_count, offsets, bigendian,
                    self._max_input_points)
                voxel_size = self._voxel_size
                with self._lock:
                    for x, y, z in points:
                        key = (round(x / voxel_size), round(y / voxel_size), round(z / voxel_size))
                        self._voxels[key] = (x, y, z)
                    if len(self._voxels) > self._max_buffer_points:
                        retained = list(self._voxels.items())[-self._max_buffer_points:]
                        self._voxels = dict(retained)
                    self._frames[source] += 1
                    self._last_cloud_time = time.monotonic()
                    self._last_error = ""
                self._publish(force=False)
            except Exception as exc:
                with self._lock:
                    self._last_error = f"cloud processing failed: {exc}"

    @staticmethod
    def _decode_points(data, point_step, point_count, offsets, bigendian, max_points):
        """Extract a stable, bounded XYZ sample from a PointCloud2 payload."""
        if point_step <= 0 or point_count <= 0:
            return []
        if not all(axis in offsets for axis in ("x", "y", "z")):
            offsets = {"x": 0, "y": 4, "z": 8}
        usable = min(point_count, len(data) // point_step)
        if usable <= 0:
            return []
        stride = max(1, math.ceil(usable / max_points))
        fmt = ">f" if bigendian else "<f"
        raw = memoryview(data)
        points = []
        for index in range(0, usable, stride):
            base = index * point_step
            try:
                x = struct.unpack_from(fmt, raw, base + offsets["x"])[0]
                y = struct.unpack_from(fmt, raw, base + offsets["y"])[0]
                z = struct.unpack_from(fmt, raw, base + offsets["z"])[0]
            except (IndexError, struct.error):
                break
            if (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)
                    and abs(x) < 300.0 and abs(y) < 300.0 and abs(z) < 50.0):
                points.append((x, y, z))
        return points

    @staticmethod
    def _build_payload(points, pose):
        robot_x = float(pose["x"]) if pose else 0.0
        robot_y = float(pose["y"]) if pose else 0.0
        # The shared mapping renderer applies a negative display rotation.
        robot_yaw = -float(pose["yaw"]) if pose else 0.0
        body = bytearray(len(points) * 12)
        for index, point in enumerate(points):
            struct.pack_into("<fff", body, index * 12, *point)
        return struct.pack("<fffBI", robot_x, robot_y, robot_yaw, 0x03, len(points)) + body

    def _publish(self, force):
        now = time.monotonic()
        with self._lock:
            if not force and now - self._last_publish_time < self._PUBLISH_INTERVAL:
                return
            point_count = len(self._voxels)
            stride = max(1, math.ceil(point_count / self._max_map_points))
            values = list(itertools.islice(self._voxels.values(), 0, None, stride))
            values = values[:self._max_map_points]
            pose = dict(self._pose) if self._pose else None
            self._last_publish_time = now
        payload = self._build_payload(values, pose)
        message = UInt8MultiArray()
        message.data = array.array("B", payload)
        with self._publish_lock:
            if self._closing.is_set():
                return
            self.pub.publish(message)
        with self._lock:
            self._published += 1

    def _heartbeat(self):
        # Publish an empty/unchanged snapshot too: the dashboard can distinguish
        # a live card waiting for vendor SLAM from a disconnected card.
        self._publish(force=True)


class SlamMappingPlugin:
    """Sensor-card lifecycle wrapper around :class:`_SlamMappingNode`."""

    PREFIX = "slam_mapping"

    def __init__(self, config, namespace, executor, *_, **__):
        self._config = dict(config)
        self._namespace = namespace
        self._executor = executor
        self._topic = f"/{namespace}/spatial/mapping" if namespace else "/spatial/mapping"
        self._node = _SlamMappingNode(self._topic, executor, self._config)

    def get_tool(self):
        return {
            "name": self.PREFIX,
            "type": "sensor",
            "multiInstance": False,
            "description": (
                "As2W live SLAM 3D map with robot pose. Requires the vendor "
                "unitree_slam service and a running mapping or relocation session."
            ),
            "inputSchema": {"type": "object", "properties": {}},
            "topic_out": [{"topic": self._topic, "format": "sensor/mapping"}],
        }

    def start(self):
        if self._node is None:
            self._node = _SlamMappingNode(self._topic, self._executor, self._config)

    def stop(self):
        if self._node is not None:
            self._node.close()
            self._node = None

    def dispatch(self, action, _args):
        if action in (self.PREFIX, "start"):
            self.start()
            return self._info("running")
        if action == "stop":
            self.stop()
            return self._info("idle")
        if action == "info":
            return self._info("running" if self._node else "idle")
        return None

    def _info(self, state):
        info = self._node.status() if self._node is not None else {"state": state}
        info["topic_out"] = [{"topic": self._topic, "format": "sensor/mapping"}]
        return info
