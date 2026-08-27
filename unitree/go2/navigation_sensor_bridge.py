"""Raw MID360 DDS to standard ROS2 sensor topics for traditional navigation.

The bridge is intentionally independent from the dashboard-oriented LidarPlugin.
It preserves raw PointCloud2 fields, normalizes LiDAR/IMU timestamps into the
Jetson ROS clock domain, and applies one fixed mounting rotation to both the
estimator point cloud and IMU.  It never applies dynamic gravity alignment.
"""

from __future__ import annotations

from array import array
import json
import os
from pathlib import Path
import queue
import subprocess
import sys
import threading
import time

from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu, PointCloud2, PointField
from std_msgs.msg import String

from navigation_pointcloud import (
    NAVIGATION_FIELDS,
    NAVIGATION_POINT_STEP,
    rotate_covariance9,
    rotate_orientation_xyzw,
    rotate_vector3,
    unitree_mid360_to_navigation_cloud,
    validated_rotation_matrix,
)
from navigation_time import ClockOffsetEstimator, split_ns, stamp_to_ns
from unitree_sdk2py.core.channel import ChannelSubscriber
from unitree_sdk2py.idl.sensor_msgs.msg.dds_ import Imu_, PointCloud2_


_NAVIGATION_CLOUD_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=2,
    durability=DurabilityPolicy.VOLATILE,
)
_IMU_QOS = QoSProfile(
    # Navigation estimators generally use a reliable IMU subscription.  Match
    # that contract so the endpoint cannot silently disconnect on QoS.
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=200,
    durability=DurabilityPolicy.VOLATILE,
)


def _absolute_topic(value: str | None, fallback: str) -> str:
    topic = (value or fallback).strip()
    return topic if topic.startswith("/") else f"/{topic}"


class _NavigationSensorNode(Node):
    def __init__(self, config: dict, namespace: str):
        super().__init__("go2_navigation_sensor_bridge")

        prefix = f"/{namespace}/navigation"
        self.cloud_topic = _absolute_topic(config.get("cloud_topic"), f"{prefix}/lidar")
        self.imu_topic = _absolute_topic(config.get("imu_topic"), f"{prefix}/imu")
        self._status_topic = f"{prefix}/_bridge_status"
        self._raw_cloud_topic = config.get(
            "raw_cloud_topic", "rt/utlidar/cloud"
        )
        self._raw_imu_topic = config.get(
            "raw_imu_topic", "rt/utlidar/imu"
        )
        self._lidar_frame = config.get("lidar_frame", "utlidar_lidar")
        self._imu_frame = config.get("imu_frame", self._lidar_frame)
        self._sensor_rotation = validated_rotation_matrix(
            config.get("sensor_rotation_matrix")
        )

        # Do not use ``self._clock``: rclpy.node.Node owns that attribute and
        # get_clock() returns it internally.
        self._clock_offset = ClockOffsetEstimator(
            warmup_samples=int(config.get("clock_warmup_samples", 32)),
            window_samples=int(config.get("clock_window_samples", 400)),
            reset_threshold_ns=int(
                float(config.get("clock_reset_threshold_ms", 1000.0)) * 1_000_000
            ),
            reset_confirm_samples=int(config.get("clock_reset_confirm_samples", 8)),
        )

        self._cloud_pub = self.create_publisher(
            PointCloud2,
            self.cloud_topic,
            _NAVIGATION_CLOUD_QOS,
        )
        self._imu_pub = self.create_publisher(Imu, self.imu_topic, _IMU_QOS)
        self._status_pub = self.create_publisher(String, self._status_topic, 10)

        self._cloud_queue: queue.Queue = queue.Queue(maxsize=2)
        self._imu_queue: queue.Queue = queue.Queue(maxsize=4)
        self._stop = threading.Event()
        self._cloud_worker = threading.Thread(
            target=self._cloud_loop, name="navigation_cloud", daemon=True
        )
        self._imu_worker = threading.Thread(
            target=self._imu_loop, name="navigation_imu", daemon=True
        )
        self._cloud_worker.start()
        self._imu_worker.start()

        self._last_stamp_ns = {"cloud": 0, "imu": 0}
        self._last_status_time = 0.0
        self._counters = {
            "cloud_received": 0,
            "cloud_published": 0,
            "cloud_dropped": 0,
            "imu_received": 0,
            "imu_published": 0,
            "imu_dropped": 0,
            "cloud_invalid_timestamps": 0,
            "imu_invalid_timestamps": 0,
            "stamp_clamped": 0,
        }
        self._last_receive_monotonic = {"cloud": 0.0, "imu": 0.0}

        self._cloud_sub = ChannelSubscriber(self._raw_cloud_topic, PointCloud2_)
        self._cloud_sub.Init(self._on_cloud, 1)
        self._imu_sub = ChannelSubscriber(self._raw_imu_topic, Imu_)
        # Direct callback avoids the SDK BQueue dropping new samples when its
        # single slot is occupied.  This callback only timestamps and enqueues.
        self._imu_sub.Init(self._on_imu)
        self.get_logger().info(
            f"Navigation sensors: {self._raw_cloud_topic} -> "
            f"{self.cloud_topic}; "
            f"{self._raw_imu_topic} -> {self.imu_topic}; "
            f"frame={self._lidar_frame}, "
            f"sensor_rotation={self._sensor_rotation.reshape(9).tolist()}"
        )

    def _correct_stamp(self, source_stamp, stream: str) -> int | None:
        try:
            source_ns = stamp_to_ns(source_stamp.sec, source_stamp.nanosec)
            host_ns = self.get_clock().now().nanoseconds
            corrected_ns = self._clock_offset.correct_observation(source_ns, host_ns)
        except (AttributeError, TypeError, ValueError) as exc:
            counter = f"{stream}_invalid_timestamps"
            self._counters[counter] += 1
            count = self._counters[counter]
            if count == 1 or count % 100 == 0:
                self.get_logger().warning(
                    f"invalid {stream} timestamp count={count}: {exc}"
                )
            return None

        if corrected_ns is None:
            return None
        if corrected_ns <= self._last_stamp_ns[stream]:
            corrected_ns = self._last_stamp_ns[stream] + 1
            self._counters["stamp_clamped"] += 1
        self._last_stamp_ns[stream] = corrected_ns
        return corrected_ns

    @staticmethod
    def _set_stamp(header, timestamp_ns: int, frame_id: str) -> None:
        sec, nanosec = split_ns(timestamp_ns)
        header.stamp.sec = sec
        header.stamp.nanosec = nanosec
        header.frame_id = frame_id

    def _on_cloud(self, msg) -> None:
        self._counters["cloud_received"] += 1
        self._last_receive_monotonic["cloud"] = time.monotonic()
        corrected_ns = self._correct_stamp(msg.header.stamp, "cloud")
        if corrected_ns is None:
            self._counters["cloud_dropped"] += 1
            self._maybe_publish_status()
            return

        fields = [
            (str(field.name), int(field.offset), int(field.datatype), int(field.count))
            for field in msg.fields
        ]
        item = (
            corrected_ns,
            int(msg.height),
            int(msg.width),
            fields,
            bool(msg.is_bigendian),
            int(msg.point_step),
            int(msg.row_step),
            bytes(msg.data),
            bool(msg.is_dense),
        )
        try:
            self._cloud_queue.put_nowait(item)
        except queue.Full:
            try:
                self._cloud_queue.get_nowait()
            except queue.Empty:
                pass
            self._counters["cloud_dropped"] += 1
            self._cloud_queue.put_nowait(item)
        self._maybe_publish_status()

    def _cloud_loop(self) -> None:
        while not self._stop.is_set():
            try:
                item = self._cloud_queue.get(timeout=0.5)
            except queue.Empty:
                continue
            if item is None:
                break

            (
                corrected_ns,
                height,
                width,
                fields,
                is_bigendian,
                point_step,
                row_step,
                data,
                is_dense,
            ) = item
            try:
                converted = unitree_mid360_to_navigation_cloud(
                    data=data,
                    height=height,
                    width=width,
                    point_step=point_step,
                    row_step=row_step,
                    fields=fields,
                    header_stamp_ns=corrected_ns,
                    rotation_matrix=self._sensor_rotation,
                )
                out = PointCloud2()
                self._set_stamp(out.header, corrected_ns, self._lidar_frame)
                out.height = 1
                out.width = height * width
                out.fields = [
                    PointField(
                        name=field.name,
                        offset=field.offset,
                        datatype=field.datatype,
                        count=field.count,
                    )
                    for field in NAVIGATION_FIELDS
                ]
                out.is_bigendian = False
                out.point_step = NAVIGATION_POINT_STEP
                out.row_step = NAVIGATION_POINT_STEP * out.width
                # Humble's generated setter validates a bytes object one element
                # at a time in debug mode. array('B') uses its constant-time path.
                out.data = array("B", converted)
                out.is_dense = is_dense
                self._cloud_pub.publish(out)
                self._counters["cloud_published"] += 1
            except (TypeError, ValueError) as exc:
                self._counters["cloud_dropped"] += 1
                if self._counters["cloud_dropped"] <= 3:
                    self.get_logger().warning(
                        f"navigation cloud conversion failed: {exc}"
                    )

    def _on_imu(self, msg) -> None:
        self._counters["imu_received"] += 1
        self._last_receive_monotonic["imu"] = time.monotonic()
        corrected_ns = self._correct_stamp(msg.header.stamp, "imu")
        if corrected_ns is None:
            self._counters["imu_dropped"] += 1
            self._maybe_publish_status()
            return

        try:
            self._imu_queue.put_nowait((corrected_ns, msg))
        except queue.Full:
            # Prefer fresh inertial data over completing a stale backlog.
            try:
                self._imu_queue.get_nowait()
            except queue.Empty:
                pass
            self._counters["imu_dropped"] += 1
            self._imu_queue.put_nowait((corrected_ns, msg))
        self._maybe_publish_status()

    def _imu_loop(self) -> None:
        while not self._stop.is_set():
            try:
                item = self._imu_queue.get(timeout=0.5)
            except queue.Empty:
                continue
            if item is None:
                break
            corrected_ns, msg = item

            out = Imu()
            self._set_stamp(out.header, corrected_ns, self._imu_frame)

            q = msg.orientation
            q_norm_sq = (
                float(q.x) ** 2
                + float(q.y) ** 2
                + float(q.z) ** 2
                + float(q.w) ** 2
            )
            if q_norm_sq > 0.25:
                qx, qy, qz, qw = rotate_orientation_xyzw(
                    (float(q.x), float(q.y), float(q.z), float(q.w)),
                    self._sensor_rotation,
                )
                out.orientation.x = qx
                out.orientation.y = qy
                out.orientation.z = qz
                out.orientation.w = qw
                out.orientation_covariance = rotate_covariance9(
                    msg.orientation_covariance, self._sensor_rotation
                )
            else:
                # The live MID360 stream reports an all-zero quaternion.  Publish
                # identity and mark orientation unavailable per REP-145.
                out.orientation.w = 1.0
                out.orientation_covariance[0] = -1.0

            wx, wy, wz = rotate_vector3(
                (
                    float(msg.angular_velocity.x),
                    float(msg.angular_velocity.y),
                    float(msg.angular_velocity.z),
                ),
                self._sensor_rotation,
            )
            out.angular_velocity.x = wx
            out.angular_velocity.y = wy
            out.angular_velocity.z = wz
            out.angular_velocity_covariance = rotate_covariance9(
                msg.angular_velocity_covariance, self._sensor_rotation
            )
            ax, ay, az = rotate_vector3(
                (
                    float(msg.linear_acceleration.x),
                    float(msg.linear_acceleration.y),
                    float(msg.linear_acceleration.z),
                ),
                self._sensor_rotation,
            )
            out.linear_acceleration.x = ax
            out.linear_acceleration.y = ay
            out.linear_acceleration.z = az
            out.linear_acceleration_covariance = rotate_covariance9(
                msg.linear_acceleration_covariance, self._sensor_rotation
            )
            self._imu_pub.publish(out)
            self._counters["imu_published"] += 1

    def _maybe_publish_status(self) -> None:
        now = time.monotonic()
        if now - self._last_status_time < 1.0:
            return
        self._last_status_time = now
        status = self.status()
        clock = status["clock"]
        if clock["offset_ns"] is not None:
            clock["offset_sec"] = round(clock["offset_ns"] / 1_000_000_000, 6)
        if clock["residual_ns"] is not None:
            clock["residual_ms"] = round(clock["residual_ns"] / 1_000_000, 3)
        out = String()
        out.data = json.dumps(
            {
                **status,
                "raw_topics": {
                    "cloud": self._raw_cloud_topic,
                    "imu": self._raw_imu_topic,
                },
                "frames": {
                    "lidar": self._lidar_frame,
                    "imu": self._imu_frame,
                },
                "sensor_rotation_matrix": [
                    float(value) for value in self._sensor_rotation.reshape(9)
                ],
            },
            separators=(",", ":"),
        )
        self._status_pub.publish(out)

    def status(self) -> dict:
        now = time.monotonic()
        ages = {
            stream: (
                round((now - received_at) * 1000.0, 1)
                if received_at > 0.0
                else None
            )
            for stream, received_at in self._last_receive_monotonic.items()
        }
        clock = self._clock_offset.snapshot().to_dict()
        blockers = []
        if not clock["ready"]:
            blockers.append("clock_not_ready")
        if ages["cloud"] is None or ages["cloud"] > 500.0:
            blockers.append("cloud_stale")
        if ages["imu"] is None or ages["imu"] > 100.0:
            blockers.append("imu_stale")
        if not self._counters["cloud_published"]:
            blockers.append("cloud_not_published")
        if not self._counters["imu_published"]:
            blockers.append("imu_not_published")
        return {
            "ready": not blockers,
            "blockers": blockers,
            "receive_age_ms": ages,
            "clock": clock,
            "counters": dict(self._counters),
        }

    def close(self) -> None:
        for subscriber in (self._cloud_sub, self._imu_sub):
            try:
                subscriber.Close()
            except Exception:
                pass
        self._stop.set()
        for work_queue in (self._cloud_queue, self._imu_queue):
            try:
                work_queue.put_nowait(None)
            except queue.Full:
                try:
                    work_queue.get_nowait()
                except queue.Empty:
                    pass
                work_queue.put_nowait(None)
        self._cloud_worker.join(timeout=2.0)
        self._imu_worker.join(timeout=2.0)


class _NavigationSensorMonitorNode(Node):
    """Lightweight main-process monitor for the isolated bridge worker."""

    def __init__(self, config: dict, namespace: str):
        super().__init__("go2_navigation_sensor_status")
        prefix = f"/{namespace}/navigation"
        self.cloud_topic = _absolute_topic(config.get("cloud_topic"), f"{prefix}/lidar")
        self.imu_topic = _absolute_topic(config.get("imu_topic"), f"{prefix}/imu")
        self.lidar_frame = config.get("lidar_frame", "utlidar_lidar")
        self.imu_frame = config.get("imu_frame", self.lidar_frame)
        self._status_topic = f"{prefix}/_bridge_status"
        self._lock = threading.RLock()
        self._last_status = None
        self._last_status_monotonic = 0.0
        self._status_sub = self.create_subscription(
            String, self._status_topic, self._on_status, 10
        )

    def _on_status(self, msg) -> None:
        try:
            payload = json.loads(msg.data)
        except (AttributeError, TypeError, ValueError, json.JSONDecodeError):
            return
        if not isinstance(payload, dict):
            return
        with self._lock:
            self._last_status = payload
            self._last_status_monotonic = time.monotonic()

    def status(self, worker_running: bool) -> dict:
        with self._lock:
            payload = dict(self._last_status or {})
            received_at = self._last_status_monotonic

        status_age_ms = (
            round((time.monotonic() - received_at) * 1000.0, 1)
            if received_at > 0.0
            else None
        )
        blockers = list(payload.get("blockers") or [])
        if not worker_running:
            blockers.append("worker_not_running")
        if status_age_ms is None:
            blockers.append("status_not_received")
        elif status_age_ms > 2000.0:
            blockers.append("status_stale")

        blockers = list(dict.fromkeys(blockers))
        return {
            **payload,
            "ready": not blockers,
            "blockers": blockers,
            "worker_running": bool(worker_running),
            "status_age_ms": status_age_ms,
        }


class NavigationSensorPlugin:
    PREFIX = "navigation_sensors"

    def __init__(
        self,
        plugin_config: dict,
        namespace: str,
        executor,
        network_iface: str = "eth0",
    ):
        self._executor = executor
        self._namespace = namespace
        self._network_iface = network_iface
        self._status_node = _NavigationSensorMonitorNode(plugin_config, namespace)
        executor.add_node(self._status_node)
        self._worker_path = Path(__file__).with_name("navigation_sensor_bridge_main.py")
        self._proc = None

    def get_tools(self) -> list[dict]:
        return [
            self._tool(
                "navigation_lidar",
                "Normalized Go2 MID360 PointCloud2 for navigation consumers",
                self._status_node.cloud_topic,
                "sensor/pointcloud",
                "sensor_msgs/msg/PointCloud2",
                "RELIABLE + KEEP_LAST(depth=2) + VOLATILE",
                self._status_node.lidar_frame,
            ),
            self._tool(
                "navigation_imu",
                "Normalized MID360 IMU in the same clock domain as LiDAR",
                self._status_node.imu_topic,
                "sensor/imu",
                "sensor_msgs/msg/Imu",
                "RELIABLE + KEEP_LAST(depth=200) + VOLATILE",
                self._status_node.imu_frame,
            ),
        ]

    @staticmethod
    def _tool(
        name: str,
        description: str,
        topic: str,
        message_format: str,
        ros_type: str,
        qos: str,
        frame_id: str,
    ) -> dict:
        descriptor = {
            "topic": topic,
            "format": message_format,
            "ros_type": ros_type,
            "qos": qos,
            "timestamp": "MID360 source clock normalized to ROS system time",
        }
        if frame_id:
            descriptor["frame_id"] = frame_id
        return {
            "name": name,
            "type": "sensor",
            "multiInstance": False,
            "description": f"{description}. Publishes to {topic}",
            "inputSchema": {"type": "object", "properties": {}},
            "topic_out": [descriptor],
        }

    def start(self) -> None:
        if self._worker_running():
            return
        if not self._worker_path.is_file():
            raise FileNotFoundError(f"navigation sensor worker missing: {self._worker_path}")
        env = os.environ.copy()
        env["ROS_NAMESPACE"] = self._namespace
        self._proc = subprocess.Popen(
            [sys.executable, str(self._worker_path), self._network_iface],
            cwd=str(self._worker_path.parent),
            env=env,
            stdout=sys.stdout,
            stderr=sys.stderr,
        )
        print(
            f"[navigation-sensors] isolated worker started pid={self._proc.pid}",
            flush=True,
        )

    def _worker_running(self) -> bool:
        return self._proc is not None and self._proc.poll() is None

    def _stop_worker(self) -> None:
        proc = self._proc
        if proc is None:
            return
        if proc.poll() is None:
            proc.terminate()
            try:
                proc.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                proc.kill()
                proc.wait(timeout=2.0)
        self._proc = None

    def stop(self) -> None:
        self._stop_worker()
        try:
            self._executor.remove_node(self._status_node)
            self._status_node.destroy_node()
        except Exception:
            pass

    def dispatch(self, action: str, args: dict) -> dict | None:
        if action in {"start", "info"}:
            if action == "start" and not self._worker_running():
                self.start()
            tool_name = args.get("_tool_name", "")
            tools = {tool["name"]: tool for tool in self.get_tools()}
            selected = tools.get(tool_name, tools["navigation_lidar"])
            worker_running = self._worker_running()
            status = self._status_node.status(worker_running)
            if action == "start":
                state = "running" if worker_running else "error"
            else:
                state = "ready" if status["ready"] else "not_ready"
            return {
                "state": state,
                "topic_out": selected["topic_out"],
                "worker_pid": self._proc.pid if worker_running else None,
                **status,
            }
        if action == "stop":
            self._stop_worker()
            return {"state": "idle", "worker_running": False, "worker_pid": None}
        return None
