"""Expose Helios VMR lidar as native ROS 2 clouds and a Motus preview."""

import asyncio
import os
import struct
import threading
import time
from array import array
from dataclasses import dataclass
from pathlib import Path

from common.vendor_runtime import tool
from .sdk_lidar_worker import HEADER, MAGIC, MAX_POINTS, MAX_STAMP_NS, SOURCES, XYZ

STALE_S = 2
SDK_SUBSCRIPTION_WINDOW_S = 1.48
PREVIEW_POINTS = 6000
PREVIEW_MAX_SKEW_NS = 250_000_000
XYZI = struct.Struct("<ffff")


@dataclass(frozen=True)
class Cloud:
    stamp_ns: int
    count: int
    xyz: bytes
    received_s: float


async def read_cloud(stream):
    magic, location, stamp, count = HEADER.unpack(await stream.readexactly(HEADER.size))
    if (magic != MAGIC or location not in SOURCES or not 0 < stamp <= MAX_STAMP_NS
            or not 0 < count <= MAX_POINTS):
        raise ValueError("invalid Helios lidar frame header")
    xyz = await stream.readexactly(count * XYZ.size)
    return location, Cloud(stamp, count, xyz, time.monotonic())


def preview_frames(clouds):
    if not clouds:
        return {}
    newest = max(cloud.stamp_ns for cloud in clouds.values())
    return {source: clouds[source] for source in SOURCES
            if source in clouds and newest - clouds[source].stamp_ns <= PREVIEW_MAX_SKEW_NS}


def pack_preview(clouds):
    budget = PREVIEW_POINTS // len(clouds)
    counts = [min(cloud.count, budget) for cloud in clouds]
    point_count = sum(counts)
    data = bytearray(8 + point_count * XYZI.size)
    struct.pack_into("<II", data, 0, XYZI.size, point_count)
    offset = 8
    for cloud, count in zip(clouds, counts):
        for index in range(count):
            point = index * cloud.count // count
            x, y, z = XYZ.unpack_from(cloud.xyz, point * XYZ.size)
            XYZI.pack_into(data, offset, x, y, z, 0.0)
            offset += XYZI.size
    return data


class SdkLidarClient:
    def __init__(self, config):
        self._python = Path(os.getenv("HELIOS_LIDAR_SDK_PYTHON", "/opt/sdk-python/bin/python3.12"))
        self._lib = Path(os.getenv("HELIOS_LIDAR_SDK_LIB", "/opt/vmr-sdk/lib"))
        self._host = config.get("lidar_sdk_host", "192.168.0.100")
        self._port = str(config.get("lidar_sdk_port", 9003))
        self._latest = {}
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = None

    @property
    def installed(self):
        return self._python.is_file() and self._lib.is_dir()

    def start(self):
        if self.installed and (self._thread is None or not self._thread.is_alive()):
            self._stop.clear()
            self._thread = threading.Thread(target=lambda: asyncio.run(self._run()),
                                            name="helios-lidar-sdk", daemon=True)
            self._thread.start()

    def stop(self):
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=7)

    def snapshot(self):
        if self._stop.is_set():
            return {}
        with self._lock:
            now = time.monotonic()
            return {source: cloud for source, cloud in self._latest.items()
                    if now - cloud.received_s <= STALE_S}

    async def _run(self):
        worker = Path(__file__).with_name("sdk_lidar_worker.py")
        env = dict(os.environ, PYTHONPATH=str(self._lib), PYTHONUNBUFFERED="1")
        env["LD_LIBRARY_PATH"] = os.pathsep.join(
            part for part in (str(self._lib), env.get("LD_LIBRARY_PATH")) if part)
        loop = asyncio.get_running_loop()
        while not self._stop.is_set():
            process = None
            failed = updated = False
            deadline = loop.time() + SDK_SUBSCRIPTION_WINDOW_S
            try:
                process = await asyncio.create_subprocess_exec(
                    str(self._python), "-P", str(worker), self._host, self._port,
                    env=env, stdout=asyncio.subprocess.PIPE, stderr=asyncio.subprocess.DEVNULL,
                )
                while not self._stop.is_set():
                    # A fresh SDK subscription starts near the newest published frame.
                    remaining = max(0.0, deadline - loop.time())
                    source, cloud = await asyncio.wait_for(read_cloud(process.stdout), remaining)
                    with self._lock:
                        previous = self._latest.get(source)
                        if (previous is None or cloud.stamp_ns > previous.stamp_ns
                                or (cloud.stamp_ns != previous.stamp_ns
                                    and cloud.received_s - previous.received_s > STALE_S)):
                            self._latest[source] = cloud
                            updated = True
            except asyncio.TimeoutError:
                # Keep recent clouds across normal rotations for the dual preview.
                pass
            except (OSError, EOFError, ValueError):
                failed = True
            finally:
                if failed or self._stop.is_set():
                    with self._lock:
                        self._latest.clear()
                if process is not None:
                    try:
                        process.kill()
                    except ProcessLookupError:
                        pass
                    # Drain the pipe so asyncio can close its subprocess transport.
                    await process.communicate()
            if not self._stop.is_set() and (failed or not updated):
                await asyncio.sleep(1)


class HeliosLidar:
    NAME = "lidar_pointcloud"

    def __init__(self, config, namespace, ros2, client=None):
        self._ros2 = ros2
        self._node = None
        self._client = client if client is not None else SdkLidarClient(config)
        self._namespace = namespace
        self._frame_id = f"{namespace}/chassis_cloud"
        self._preview = {"topic": f"/{namespace}/lidar/preview", "format": "sensor/pointcloud"}
        self._native_topics = {
            source: f"/{namespace}/lidar/{name}/points"
            for source, name in SOURCES.items()
        }
        self._last_sent = {}

    def get_tool(self):
        return tool(
            self.NAME, "sensor",
            "Helios 左前与右后雷达三维点云。两路叠加抽样预览，两路原生 XYZ 用于下游处理，单位为米。",
            topic_out=[self._preview],
        )

    def start(self):
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data, QoSProfile
        from sensor_msgs.msg import PointCloud2, PointField
        from std_msgs.msg import UInt8MultiArray

        if self._node is not None:
            return
        qos = QoSProfile(depth=1, reliability=qos_profile_sensor_data.reliability)
        fields = [PointField(name=axis, offset=offset, datatype=PointField.FLOAT32, count=1)
                  for axis, offset in zip("xyz", (0, 4, 8))]
        node = Node("helios_lidar", namespace=self._namespace, context=self._ros2.ctx_core)
        try:
            preview = node.create_publisher(UInt8MultiArray, self._preview["topic"], qos)
            native = {source: node.create_publisher(PointCloud2, topic, qos)
                      for source, topic in self._native_topics.items()}
            displayed = ()

            def publish():
                nonlocal displayed
                clouds = self._client.snapshot()
                for source, cloud in clouds.items():
                    if self._last_sent.get(source) == cloud.stamp_ns:
                        continue
                    msg = PointCloud2(height=1, width=cloud.count, fields=fields, is_dense=True,
                                      point_step=XYZ.size, row_step=len(cloud.xyz))
                    msg.header.frame_id = self._frame_id
                    msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(cloud.stamp_ns, 1_000_000_000)
                    msg.data = array("B", cloud.xyz)
                    native[source].publish(msg)
                    self._last_sent[source] = cloud.stamp_ns
                frames = preview_frames(clouds) if preview.get_subscription_count() else {}
                frame_ids = tuple((source, cloud.received_s) for source, cloud in frames.items())
                if frames and frame_ids != displayed:
                    data = pack_preview(tuple(frames.values()))
                    preview.publish(UInt8MultiArray(data=array("B", data)))
                displayed = frame_ids

            node.create_timer(0.05, publish)
            self._ros2.executor_core.add_node(node)
        except Exception:
            node.destroy_node()
            raise
        self._node = node
        self._client.start()

    def stop(self):
        self._client.stop()
        if self._node is not None:
            self._ros2.executor_core.remove_node(self._node)
            self._node.destroy_node()
            self._node = None
        self._last_sent.clear()

    def dispatch(self, action, args):
        ports = self.get_tool()["topic_out"]
        if action == "info":
            return {
                **self.get_tool(),
                "frame_id": self._frame_id,
                "position_unit": "m",
                "native_topics": [
                    {"topic": topic, "message_type": "sensor_msgs/msg/PointCloud2"}
                    for topic in self._native_topics.values()
                ],
            }
        if action in (self.NAME, "read", "status", "start", "config"):
            clouds = self._client.snapshot()
            now = time.monotonic()
            return {"sdk_installed": self._client.installed, "topic_out": ports,
                    "preview_sources": [SOURCES[source] for source in preview_frames(clouds)], "sources": {
                name: {"location": source, "available": source in clouds,
                       **({"points": clouds[source].count, "age_ms": round(
                           1000 * (now - clouds[source].received_s))} if source in clouds else {})}
                for source, name in SOURCES.items()
            }}
        if action == "stop":
            return {"state": "idle", "topic_out": ports}
        raise ValueError(f"unsupported {self.NAME} action: {action}")
