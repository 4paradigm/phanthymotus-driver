"""
LidarPlugin — Livox Mid-360 point cloud sensor for Go2.

Subscribes to DDS PointCloud2 topic, applies gravity alignment using co-located
Livox IMU, and republishes as UInt8MultiArray for the dashboard 3D renderer.

Binary output format: [uint32 point_step][uint32 total_points][raw PointCloud2 bytes]
"""

import queue
import struct
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from pointcloud_utils import gravity_align_inplace

_LOW_LAT_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=200,
    durability=DurabilityPolicy.VOLATILE,
)

LIDAR_CLOUD_INTERVAL = 0.05  # 20 Hz max (source is ~15Hz, allow headroom)


class _LidarNode(Node):
    """Subscribes to DDS utlidar PointCloud2 and republishes with gravity alignment."""

    def __init__(self, cloud_topic: str):
        super().__init__("go2_lidar")
        from std_msgs.msg import UInt8MultiArray
        self._cloud_pub = self.create_publisher(UInt8MultiArray, cloud_topic, _LOW_LAT_QOS)
        self._last_cloud_time: float = 0.0
        self._imu_roll: float = 0.0
        self._imu_pitch: float = 0.0

        # Diagnostics
        self._cb_count: int = 0
        self._cb_accepted: int = 0
        self._cb_dropped: int = 0
        self._cb_first_time: float = 0.0
        self._worker_count: int = 0
        self._worker_total_ms: float = 0.0

        # Worker thread for point cloud processing
        self._cloud_queue: queue.Queue = queue.Queue(maxsize=10)
        self._worker = threading.Thread(target=self._process_loop, daemon=True, name="lidar_worker")
        self._worker.start()

        # Subscribe DDS PointCloud2
        try:
            from unitree_sdk2py.core.channel import ChannelSubscriber
            from unitree_sdk2py.idl.sensor_msgs.msg.dds_ import PointCloud2_
            self._cloud_sub = ChannelSubscriber("rt/utlidar/cloud", PointCloud2_)
            self._cloud_sub.Init(self._on_cloud, 1)
            self.get_logger().info(f"LidarNode subscribed rt/utlidar/cloud → {cloud_topic}")
        except Exception as e:
            self.get_logger().warn(f"LidarNode: failed to subscribe cloud: {e}")

        # Subscribe DDS Livox IMU for gravity alignment
        try:
            from unitree_sdk2py.core.channel import ChannelSubscriber
            from unitree_sdk2py.idl.sensor_msgs.msg.dds_ import Imu_
            self._livox_imu_sub = ChannelSubscriber("rt/utlidar/imu", Imu_)
            self._livox_imu_sub.Init(self._on_livox_imu, 10)
            self.get_logger().info("LidarNode subscribed rt/utlidar/imu for gravity alignment")
        except Exception as e:
            self.get_logger().warn(f"LidarNode: failed to subscribe Livox IMU: {e}")

    def _on_cloud(self, msg) -> None:
        """DDS callback — throttle and enqueue for worker thread."""
        self._cb_count += 1
        now = time.monotonic()
        if now - self._last_cloud_time < LIDAR_CLOUD_INTERVAL:
            return
        self._last_cloud_time = now
        self._cb_accepted += 1

        if self._cb_first_time == 0.0:
            self._cb_first_time = now

        point_step = msg.point_step
        total_points = msg.width * msg.height
        data = msg.data if isinstance(msg.data, (bytes, bytearray)) else bytes(msg.data)

        try:
            self._cloud_queue.put_nowait((point_step, total_points, data,
                                         self._imu_roll, self._imu_pitch))
        except queue.Full:
            self._cb_dropped += 1

        # Print stats every 2000 accepted frames
        if self._cb_accepted % 2000 == 0:
            elapsed = now - self._cb_first_time
            avg_hz = self._cb_accepted / elapsed if elapsed > 0 else 0
            print(
                f"[lidar:stats] received={self._cb_count} accepted={self._cb_accepted} "
                f"dropped={self._cb_dropped} avg_hz={avg_hz:.1f} "
                f"worker_avg={self._worker_total_ms / max(self._worker_count, 1):.1f}ms",
                flush=True,
            )

    def _process_loop(self) -> None:
        """Worker thread: gravity alignment + publish."""
        import array as _array
        from std_msgs.msg import UInt8MultiArray
        while True:
            item = self._cloud_queue.get()
            if item is None:
                break
            point_step, total_points, data, roll, pitch = item
            t0 = time.monotonic()

            data = gravity_align_inplace(data, point_step, total_points, roll, pitch)

            header = struct.pack('<II', point_step, total_points)
            buf = bytearray(8 + len(data))
            buf[:8] = header
            buf[8:] = data
            ros_msg = UInt8MultiArray()
            ros_msg.data = _array.array('B', buf)
            self._cloud_pub.publish(ros_msg)

            elapsed_ms = (time.monotonic() - t0) * 1000
            self._worker_count += 1
            self._worker_total_ms += elapsed_ms

    def _on_livox_imu(self, msg) -> None:
        """Compute roll/pitch from Livox IMU accelerometer."""
        import math
        try:
            acc = msg.linear_acceleration
            ax, ay, az = float(acc.x), float(acc.y), float(acc.z)
            # Livox mounted inverted: flip y,z to get upright-equivalent frame
            self._imu_roll = math.atan2(-ay, -az)
            self._imu_pitch = math.atan2(-ax, math.sqrt(ay * ay + az * az))
        except Exception:
            pass


class LidarPlugin:
    PREFIX = "lidar"

    def __init__(self, plugin_config: dict, namespace: str, executor):
        self._cloud_topic = f"/{namespace}/lidar/cloud"
        self._node = _LidarNode(self._cloud_topic)
        executor.add_node(self._node)

    def get_tools(self) -> list:
        return [self._cloud_tool()]

    def _cloud_tool(self) -> dict:
        return {
            "name": "lidar_cloud",
            "type": "sensor",
            "multiInstance": False,
            "description": f"Livox Mid-360 full point cloud passthrough at ~15Hz. Binary format: [uint32 point_step][uint32 total_points][raw PointCloud2 bytes]. Publishes to {self._cloud_topic}",
            "inputSchema": {"type": "object", "properties": {}},
            "topic_out": [{"topic": self._cloud_topic, "format": "sensor/pointcloud"}],
            "configSchema": {
                "type": "object",
                "properties": {},
            },
        }

    def start(self) -> None:
        pass

    def stop(self) -> None:
        pass

    def dispatch(self, action: str, args: dict) -> dict | None:
        if action == "start":
            return {"state": "running"}
        if action == "stop":
            return {"state": "idle"}
        if action == "info":
            return {"state": "running", "topic_out": [{"topic": self._cloud_topic, "format": "sensor/pointcloud"}]}
        return None
