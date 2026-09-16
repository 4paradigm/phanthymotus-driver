"""AS2W lidar bridge from Unitree DDS PointCloud2 to sensor/pointcloud."""
import struct
import threading
import time

from std_msgs.msg import UInt8MultiArray
from unitree_sdk2py.core.channel import ChannelSubscriber
from unitree_sdk2py.idl.sensor_msgs.msg.dds_ import PointCloud2_


# AS2W firmware revisions have used different names for the direct lidar
# stream. Map/relocation clouds are conditional SLAM products, not live lidar.
_DEFAULT_SOURCE_TOPICS = (
    "rt/utlidar/cloud_deskewed",
    "rt/utlidar/cloud",
    "rt/utlidar/cloud_livox_mid360",
    "rt/utlidar/cloud_jt128",
)
_SOURCE_TIMEOUT_SECONDS = 3.0


class _LidarNode:
    def __init__(self, topic, executor, source_topics=None):
        from rclpy.node import Node
        self.node = Node("as2w_lidar")
        self.pub = self.node.create_publisher(UInt8MultiArray, topic, 10)
        configured = source_topics or _DEFAULT_SOURCE_TOPICS
        self.source_topics = tuple(dict.fromkeys(configured))
        self.subs = []
        self._lock = threading.Lock()
        self._active_source = None
        self._last_seen = {source: 0.0 for source in self.source_topics}
        self._frames = {source: 0 for source in self.source_topics}
        self._bytes = {source: 0 for source in self.source_topics}
        for source_topic in self.source_topics:
            try:
                sub = ChannelSubscriber(source_topic, PointCloud2_)
                sub.Init(lambda msg, source=source_topic: self._on_cloud(source, msg), 1)
                self.subs.append(sub)
                self.node.get_logger().info(f"AS2W lidar listening on {source_topic}")
            except Exception as exc:
                self.node.get_logger().warning(f"AS2W lidar could not subscribe {source_topic}: {exc}")
        self.node.create_timer(15.0, self._report)
        executor.add_node(self.node)

    def _report(self):
        now = time.monotonic()
        with self._lock:
            if (self._active_source and
                    now - self._last_seen[self._active_source] > _SOURCE_TIMEOUT_SECONDS):
                self.node.get_logger().warning(
                    f"AS2W lidar source {self._active_source} timed out; waiting for another source")
                self._active_source = None
            frames, sizes, active = dict(self._frames), dict(self._bytes), self._active_source
        summary = ", ".join(f"{source}={frames[source]} frames/{sizes[source]} B" for source in self.source_topics)
        if active:
            self.node.get_logger().info(f"AS2W lidar active source {active}; {summary}")
        else:
            self.node.get_logger().warning(
                "AS2W lidar has received no PointCloud2 frames. "
                f"Candidates: {summary}. Set plugins.lidar.source_topics for this firmware.")

    def _on_cloud(self, source, msg):
        data = msg.data if isinstance(msg.data, (bytes, bytearray)) else bytes(msg.data)
        point_step = int(msg.point_step)
        point_count = int(msg.width) * int(msg.height)
        if point_step <= 0 or point_count <= 0 or not data:
            return
        with self._lock:
            self._frames[source] += 1
            self._bytes[source] += len(data)
            now = time.monotonic()
            self._last_seen[source] = now
            if (self._active_source is None or
                    now - self._last_seen[self._active_source] > _SOURCE_TIMEOUT_SECONDS):
                previous = self._active_source
                self._active_source = source
                self.node.get_logger().info(
                    f"AS2W lidar selected live source {source}"
                    + (f" (replacing {previous})" if previous else ""))
            if source != self._active_source:
                return
        payload = struct.pack("<II", point_step, point_count) + data
        out = UInt8MultiArray()
        out.data = list(payload)
        self.pub.publish(out)


class LidarPlugin:
    PREFIX = "lidar"

    def __init__(self, config, namespace, executor):
        self.topic = f"/{namespace}/lidar/cloud"
        self.node = _LidarNode(self.topic, executor, config.get("source_topics"))

    def get_tools(self):
        return [self._cloud_tool()]

    def _cloud_tool(self):
        return {"name": "lidar_cloud", "type": "sensor", "multiInstance": False,
                "description": f"AS2W live lidar PointCloud2 passthrough. Binary format [uint32 point_step][uint32 point_count][raw data], published to {self.topic}",
                "inputSchema": {"type": "object", "properties": {}},
                "topic_out": [{"topic": self.topic, "format": "sensor/pointcloud"}]}

    def start(self):
        pass

    def stop(self):
        for sub in getattr(self.node, "subs", []):
            try:
                sub.Close()
            except Exception:
                pass
        self.node.node.destroy_node()

    def dispatch(self, action, args):
        if action in ("start", "info", "lidar_cloud"):
            return {"state": "running", "topic_out": [{"topic": self.topic, "format": "sensor/pointcloud"}]}
        if action == "stop":
            return {"state": "idle"}
        return None
