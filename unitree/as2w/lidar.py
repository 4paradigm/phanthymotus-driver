"""As2W lidar bridge from Unitree DDS PointCloud2 to sensor/pointcloud."""
import array
import queue
import struct
import threading
import time

from std_msgs.msg import UInt8MultiArray
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from unitree_sdk2py.core.channel import ChannelSubscriber
from unitree_sdk2py.idl.sensor_msgs.msg.dds_ import PointCloud2_


# As2W firmware revisions have used different names for the direct lidar
# stream. Map/relocation clouds are conditional SLAM products, not live lidar.
_DEFAULT_SOURCE_TOPICS = (
    "rt/utlidar/cloud_livox_mid360",
    "rt/utlidar/cloud_deskewed",
    "rt/utlidar/cloud",
    "rt/utlidar/cloud_jt128",
)
_SOURCE_TIMEOUT_SECONDS = 1.5
# A large Livox frame is expensive to decode in Python.  The dashboard has no
# useful visual benefit from all source points, while a smaller uniform sample
# makes it substantially more likely that the displayed frame is the latest.
_MAX_RENDER_POINTS = 12000
# Official As2W URDF JT128 fixed joint: rpy=(-pi, 1.4661, -pi).
# This maps points from the lidar frame into the As2W base frame.  Keeping the
# values explicit avoids pulling numpy into the latency-sensitive bridge.
_JT128_R = (
    (-0.1045051633, 0.0, 0.9945243440),
    (0.0, 1.0, 0.0),
    (-0.9945243440, 0.0, -0.1045051633),
)
# The As2W visual alignment verified on hardware is the base-frame ordering.
# Agent-core maps wire coordinates as display=(wire_y, -wire_z, -wire_x), so
# use this preimage to produce display=(base_x, base_y, base_z).
_LIDAR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    durability=DurabilityPolicy.VOLATILE,
)


class _LidarNode:
    def __init__(self, topic, executor, source_topics=None):
        from rclpy.node import Node
        self.node = Node("as2w_lidar")
        self.pub = self.node.create_publisher(UInt8MultiArray, topic, _LIDAR_QOS)
        configured = source_topics or _DEFAULT_SOURCE_TOPICS
        self.source_topics = tuple(dict.fromkeys(configured))
        self.subs = []
        self._lock = threading.Lock()
        self._active_source = None
        self._last_seen = {source: 0.0 for source in self.source_topics}
        self._frames = {source: 0 for source in self.source_topics}
        self._bytes = {source: 0 for source in self.source_topics}
        self._published = 0
        self._dropped = 0
        self._processing_seconds = 0.0
        # A live Livox frame can exceed 1 MiB. Do not serialize and publish it
        # from the CycloneDDS callback; that starves the DDS reader and causes
        # the intermittent one-frame behaviour observed on As2W.
        self._cloud_queue = queue.Queue(maxsize=1)
        self._stopped = threading.Event()
        self._worker = threading.Thread(target=self._publish_loop, daemon=True,
                                        name="as2w_lidar_publish")
        self._worker.start()
        for source_topic in self.source_topics:
            try:
                sub = ChannelSubscriber(source_topic, PointCloud2_)
                sub.Init(lambda msg, source=source_topic: self._on_cloud(source, msg), 1)
                self.subs.append(sub)
                self.node.get_logger().info(f"As2W lidar listening on {source_topic}")
            except Exception as exc:
                self.node.get_logger().warning(f"As2W lidar could not subscribe {source_topic}: {exc}")
        self.node.create_timer(15.0, self._report)
        executor.add_node(self.node)

    def _report(self):
        now = time.monotonic()
        with self._lock:
            if (self._active_source and
                    now - self._last_seen[self._active_source] > _SOURCE_TIMEOUT_SECONDS):
                self.node.get_logger().warning(
                    f"As2W lidar source {self._active_source} timed out; waiting for another source")
                self._active_source = None
            frames, sizes, active = dict(self._frames), dict(self._bytes), self._active_source
            published, dropped = self._published, self._dropped
            processing = self._processing_seconds
        summary = ", ".join(f"{source}={frames[source]} frames/{sizes[source]} B" for source in self.source_topics)
        timing = f"published={published}, dropped={dropped}, avg_convert={processing / published * 1000:.1f}ms" if published else "published=0"
        if active:
            self.node.get_logger().info(f"As2W lidar active source {active}; {summary}; {timing}")
        else:
            self.node.get_logger().warning(
                "As2W lidar has received no PointCloud2 frames. "
                f"Candidates: {summary}. Set plugins.lidar.source_topics for this firmware.")

    def _publish_loop(self):
        while not self._stopped.is_set():
            try:
                point_step, point_count, data, offsets, endian = self._cloud_queue.get(timeout=0.5)
            except queue.Empty:
                continue
            try:
                start = time.monotonic()
                data = self._to_xyz(data, point_step, point_count, offsets, endian)
                if not data:
                    continue
                # The dashboard renderer intentionally consumes compact XYZ points
                # (it reads float32 x/y/z at offsets 0/4/8).  Do not forward the
                # vendor's intensity/line/timestamp fields or their offsets.
                payload = struct.pack("<II", 12, len(data) // 12) + data
                out = UInt8MultiArray()
                # This avoids constructing millions of boxed Python ints per frame.
                out.data = array.array("B", payload)
                self.pub.publish(out)
                with self._lock:
                    self._published += 1
                    self._processing_seconds += time.monotonic() - start
            except Exception as exc:
                self.node.get_logger().warning(f"As2W lidar publish failed; continuing: {exc}")

    @staticmethod
    def _to_xyz(data, point_step, point_count, offsets, endian):
        """Extract little-endian compact XYZ for the dashboard renderer.

        Unitree's direct cloud uses the standard lidar frame (x forward, y
        left, z up).  The renderer applies its own x/y/z axis map, so changing
        signs here would rotate the cloud twice.  ``PointCloud2`` fields are
        not guaranteed to start at byte zero; normalize them here instead.
        """
        if not all(key in offsets for key in ("x", "y", "z")):
            if point_step < 12:
                return b""
            offsets = {"x": 0, "y": 4, "z": 8}
        raw = memoryview(data)
        fmt = ">f" if endian else "<f"
        count = min(point_count, _MAX_RENDER_POINTS)
        stride = max(1, point_count // count)
        selected = min(count, (point_count + stride - 1) // stride)
        out = bytearray(selected * 12)
        try:
            for target_index in range(selected):
                index = target_index * stride
                base = index * point_step
                target = target_index * 12
                x = struct.unpack_from(fmt, raw, base + offsets["x"])[0]
                y = struct.unpack_from(fmt, raw, base + offsets["y"])[0]
                z = struct.unpack_from(fmt, raw, base + offsets["z"])[0]
                # Convert JT128 lidar coordinates to As2W base coordinates.
                bx = _JT128_R[0][0] * x + _JT128_R[0][1] * y + _JT128_R[0][2] * z
                by = _JT128_R[1][0] * x + _JT128_R[1][1] * y + _JT128_R[1][2] * z
                bz = _JT128_R[2][0] * x + _JT128_R[2][1] * y + _JT128_R[2][2] * z
                rx, ry, rz = -bz, bx, -by
                # Output is always little-endian, independent of DDS input.
                struct.pack_into("<fff", out, target, rx, ry, rz)
        except (IndexError, struct.error, ValueError):
            return b""
        return bytes(out)

    def _on_cloud(self, source, msg):
        try:
            data = msg.data if isinstance(msg.data, (bytes, bytearray)) else bytes(msg.data)
            point_step = int(msg.point_step)
            point_count = int(msg.width) * int(msg.height)
            if point_step <= 0 or point_count <= 0 or not data:
                return
            point_count = min(point_count, len(data) // point_step)
            if point_count <= 0:
                return
        except Exception as exc:
            self.node.get_logger().warning(f"As2W lidar dropped malformed frame: {exc}")
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
                    f"As2W lidar selected live source {source}"
                    + (f" (replacing {previous})" if previous else ""))
            if source != self._active_source:
                return
        offsets = {}
        for field in getattr(msg, "fields", []) or []:
            name = str(getattr(field, "name", "")).lower()
            if name in ("x", "y", "z"):
                offsets[name] = int(getattr(field, "offset", 0))
        endian = bool(getattr(msg, "is_bigendian", False))
        try:
            item = (point_step, point_count, data, offsets, endian)
            self._cloud_queue.put_nowait(item)
        except queue.Full:
            with self._lock:
                self._dropped += 1
            # A dashboard should receive the latest scan, never a stale backlog.
            try:
                self._cloud_queue.get_nowait()
            except queue.Empty:
                pass
            try:
                self._cloud_queue.put_nowait(item)
            except queue.Full:
                pass

    def close(self):
        self._stopped.set()
        for sub in self.subs:
            try:
                sub.Close()
            except Exception:
                pass
        self._worker.join(timeout=1)
        self.node.destroy_node()


class LidarPlugin:
    PREFIX = "lidar"

    def __init__(self, config, namespace, executor):
        self.topic = f"/{namespace}/lidar/cloud"
        self._config = config
        self._executor = executor
        self.node = _LidarNode(self.topic, executor, config.get("source_topics"))

    def get_tools(self):
        return [self._cloud_tool()]

    def _cloud_tool(self):
        return {"name": "lidar_cloud", "type": "sensor", "multiInstance": False,
                "description": f"As2W live lidar PointCloud2 passthrough. Binary format [uint32 point_step][uint32 point_count][raw data], published to {self.topic}",
                "inputSchema": {"type": "object", "properties": {}},
                "topic_out": [{"topic": self.topic, "format": "sensor/pointcloud"}]}

    def start(self):
        if self.node is None:
            self.node = _LidarNode(self.topic, self._executor, self._config.get("source_topics"))

    def stop(self):
        if self.node is not None:
            self.node.close()
            self.node = None

    def dispatch(self, action, args):
        if action == "start":
            self.start()
            return {"state": "running", "topic_out": [{"topic": self.topic, "format": "sensor/pointcloud"}]}
        if action == "stop":
            self.stop()
            return {"state": "idle"}
        if action in ("start", "info", "lidar_cloud"):
            return {"state": "running", "topic_out": [{"topic": self.topic, "format": "sensor/pointcloud"}]}
        return None
