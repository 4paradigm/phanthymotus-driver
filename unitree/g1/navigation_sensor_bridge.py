"""Raw MID360 DDS to standard ROS2 sensor topics for traditional navigation.

The bridge is intentionally independent from the dashboard-oriented LidarPlugin.
It preserves raw PointCloud2 fields, normalizes LiDAR/IMU timestamps into the
Jetson ROS clock domain, and applies one fixed mounting rotation to both the
estimator point cloud and IMU.  It never applies dynamic gravity alignment.
"""

from __future__ import annotations

from array import array
import json
import queue
import threading
import time

from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu, PointCloud2, PointField
from std_msgs.msg import String

from navigation_pointcloud import (
    FAST_LIVO_FIELDS,
    FAST_LIVO_POINT_STEP,
    rotate_covariance9,
    rotate_orientation_xyzw,
    rotate_vector3,
    unitree_mid360_to_fast_livo,
    validated_rotation_matrix,
)
from navigation_time import ClockOffsetEstimator, split_ns, stamp_to_ns
from unitree_sdk2py.core.channel import ChannelSubscriber
from unitree_sdk2py.idl.sensor_msgs.msg.dds_ import Imu_, PointCloud2_


_RAW_CLOUD_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=2,
    durability=DurabilityPolicy.VOLATILE,
)
_FAST_LIVO_CLOUD_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=2,
    durability=DurabilityPolicy.VOLATILE,
)
_IMU_QOS = QoSProfile(
    # The pinned FAST-LIVO2 port uses the ROS default reliable subscription.
    # A reliable publisher remains compatible with best-effort diagnostics,
    # while the inverse pairing silently disconnects the estimator input.
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
        super().__init__("g1_navigation_sensor_bridge")

        prefix = f"/{namespace}/navigation"
        self.cloud_topic = _absolute_topic(config.get("cloud_topic"), f"{prefix}/lidar")
        self.fast_livo_cloud_topic = _absolute_topic(
            config.get("fast_livo_cloud_topic"), f"{prefix}/lidar_fast_livo"
        )
        self._publish_raw_cloud = bool(config.get("publish_raw_cloud", True))
        self._publish_fast_livo_cloud = bool(
            config.get("publish_fast_livo_cloud", True)
        )
        self.imu_topic = _absolute_topic(config.get("imu_topic"), f"{prefix}/imu")
        self.diagnostics_topic = _absolute_topic(
            config.get("diagnostics_topic"), f"{prefix}/sensor_diagnostics"
        )
        self._raw_cloud_topic = config.get(
            "raw_cloud_topic", "rt/utlidar/cloud_livox_mid360"
        )
        self._raw_imu_topic = config.get(
            "raw_imu_topic", "rt/utlidar/imu_livox_mid360"
        )
        self._raw_lidar_frame = config.get("raw_lidar_frame", "livox_raw_frame")
        self._lidar_frame = config.get("lidar_frame", "livox_frame")
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

        self._cloud_pub = (
            self.create_publisher(PointCloud2, self.cloud_topic, _RAW_CLOUD_QOS)
            if self._publish_raw_cloud
            else None
        )
        self._fast_livo_cloud_pub = (
            self.create_publisher(
                PointCloud2,
                self.fast_livo_cloud_topic,
                _FAST_LIVO_CLOUD_QOS,
            )
            if self._publish_fast_livo_cloud
            else None
        )
        self._imu_pub = self.create_publisher(Imu, self.imu_topic, _IMU_QOS)
        self._diagnostics_pub = self.create_publisher(String, self.diagnostics_topic, 10)

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
        self._last_diagnostics_time = 0.0
        self._counters = {
            "cloud_received": 0,
            "cloud_published": 0,
            "cloud_dropped": 0,
            "fast_livo_cloud_published": 0,
            "fast_livo_cloud_dropped": 0,
            "imu_received": 0,
            "imu_published": 0,
            "imu_dropped": 0,
            "stamp_clamped": 0,
        }
        self._last_receive_monotonic = {"cloud": 0.0, "imu": 0.0}

        self._cloud_sub = ChannelSubscriber(self._raw_cloud_topic, PointCloud2_)
        self._cloud_sub.Init(self._on_cloud, 1)
        self._imu_sub = ChannelSubscriber(self._raw_imu_topic, Imu_)
        # Direct callback avoids the SDK BQueue dropping new samples when its
        # single slot is occupied.  This callback only timestamps and enqueues.
        self._imu_sub.Init(self._on_imu)
        cloud_outputs = []
        if self._publish_fast_livo_cloud:
            cloud_outputs.append(self.fast_livo_cloud_topic)
        if self._publish_raw_cloud:
            cloud_outputs.append(self.cloud_topic)
        self.get_logger().info(
            f"Navigation sensors: {self._raw_cloud_topic} -> "
            f"{','.join(cloud_outputs) or '(disabled)'}; "
            f"{self._raw_imu_topic} -> {self.imu_topic}; "
            f"raw_frame={self._raw_lidar_frame}, corrected_frame={self._lidar_frame}, "
            f"sensor_rotation={self._sensor_rotation.reshape(9).tolist()}"
        )

    def _correct_stamp(self, source_stamp, stream: str) -> int | None:
        try:
            source_ns = stamp_to_ns(source_stamp.sec, source_stamp.nanosec)
            host_ns = self.get_clock().now().nanoseconds
            corrected_ns = self._clock_offset.correct_observation(source_ns, host_ns)
        except (AttributeError, TypeError, ValueError) as exc:
            self.get_logger().warning(f"invalid {stream} timestamp: {exc}")
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
            self._maybe_publish_diagnostics()
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
        self._maybe_publish_diagnostics()

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
            expected_size = row_step * height
            if point_step <= 0 or height <= 0 or width <= 0 or len(data) < expected_size:
                self._counters["cloud_dropped"] += 1
                continue

            # The estimator input is the priority path.  Publishing the raw and
            # adapted 0.4/0.6 MB clouds together can saturate Python ROS2
            # serialization on the live Jetson, so the raw diagnostic output is
            # independently switchable.
            if self._fast_livo_cloud_pub is not None:
                try:
                    converted = unitree_mid360_to_fast_livo(
                        data=data,
                        point_count=height * width,
                        point_step=point_step,
                        fields=fields,
                        header_stamp_ns=corrected_ns,
                        rotation_matrix=self._sensor_rotation,
                    )
                    fast_livo = PointCloud2()
                    self._set_stamp(
                        fast_livo.header, corrected_ns, self._lidar_frame
                    )
                    fast_livo.height = 1
                    fast_livo.width = height * width
                    fast_livo.fields = [
                        PointField(
                            name=field.name,
                            offset=field.offset,
                            datatype=field.datatype,
                            count=field.count,
                        )
                        for field in FAST_LIVO_FIELDS
                    ]
                    fast_livo.is_bigendian = False
                    fast_livo.point_step = FAST_LIVO_POINT_STEP
                    fast_livo.row_step = FAST_LIVO_POINT_STEP * fast_livo.width
                    # Humble's generated setter validates a bytes object one
                    # element at a time in debug mode.  array('B') takes its
                    # constant-time fast path for large PointCloud2 payloads.
                    fast_livo.data = array("B", converted)
                    fast_livo.is_dense = is_dense
                    self._fast_livo_cloud_pub.publish(fast_livo)
                    self._counters["fast_livo_cloud_published"] += 1
                except (TypeError, ValueError) as exc:
                    self._counters["fast_livo_cloud_dropped"] += 1
                    if self._counters["fast_livo_cloud_dropped"] <= 3:
                        self.get_logger().warning(
                            f"FAST-LIVO cloud conversion failed: {exc}"
                        )

            if self._cloud_pub is not None:
                out = PointCloud2()
                self._set_stamp(out.header, corrected_ns, self._raw_lidar_frame)
                out.height = height
                out.width = width
                out.fields = [
                    PointField(
                        name=name,
                        offset=offset,
                        datatype=datatype,
                        count=count,
                    )
                    for name, offset, datatype, count in fields
                ]
                out.is_bigendian = is_bigendian
                out.point_step = point_step
                out.row_step = row_step
                out.data = array("B", data)
                out.is_dense = is_dense
                self._cloud_pub.publish(out)
                self._counters["cloud_published"] += 1

    def _on_imu(self, msg) -> None:
        self._counters["imu_received"] += 1
        self._last_receive_monotonic["imu"] = time.monotonic()
        corrected_ns = self._correct_stamp(msg.header.stamp, "imu")
        if corrected_ns is None:
            self._counters["imu_dropped"] += 1
            self._maybe_publish_diagnostics()
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
        self._maybe_publish_diagnostics()

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

    def _maybe_publish_diagnostics(self) -> None:
        now = time.monotonic()
        if now - self._last_diagnostics_time < 1.0:
            return
        self._last_diagnostics_time = now
        clock = self._clock_offset.snapshot().to_dict()
        if clock["offset_ns"] is not None:
            clock["offset_sec"] = round(clock["offset_ns"] / 1_000_000_000, 6)
        if clock["residual_ns"] is not None:
            clock["residual_ms"] = round(clock["residual_ns"] / 1_000_000, 3)
        out = String()
        out.data = json.dumps(
            {
                "clock": clock,
                "counters": dict(self._counters),
                "raw_topics": {
                    "cloud": self._raw_cloud_topic,
                    "imu": self._raw_imu_topic,
                },
                "frames": {
                    "raw_lidar": self._raw_lidar_frame,
                    "lidar": self._lidar_frame,
                    "imu": self._imu_frame,
                },
                "sensor_rotation_matrix": [
                    float(value) for value in self._sensor_rotation.reshape(9)
                ],
            },
            separators=(",", ":"),
        )
        self._diagnostics_pub.publish(out)

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
        if self._publish_fast_livo_cloud and not self._counters[
            "fast_livo_cloud_published"
        ]:
            blockers.append("fast_livo_cloud_not_published")
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


class NavigationSensorPlugin:
    PREFIX = "navigation_sensors"

    def __init__(self, plugin_config: dict, namespace: str, executor):
        self._executor = executor
        self._node = _NavigationSensorNode(plugin_config, namespace)
        executor.add_node(self._node)

    def get_tools(self) -> list[dict]:
        tools = []
        if self._node._publish_fast_livo_cloud:
            tools.append(
                self._tool(
                    "navigation_lidar_fast_livo",
                    "MID360 PointCloud2 adapted for the pinned FAST-LIVO2 ROS2 port",
                    self._node.fast_livo_cloud_topic,
                    "sensor/pointcloud",
                    "sensor_msgs/msg/PointCloud2",
                    "RELIABLE + KEEP_LAST(depth=2) + VOLATILE",
                    "livox_frame",
                )
            )
        if self._node._publish_raw_cloud:
            tools.append(
                self._tool(
                    "navigation_lidar",
                    "Standard PointCloud2 with raw MID360 fields and per-point time preserved",
                    self._node.cloud_topic,
                    "sensor/pointcloud",
                    "sensor_msgs/msg/PointCloud2",
                    "BEST_EFFORT + KEEP_LAST(depth=2) + VOLATILE",
                    "livox_raw_frame",
                )
            )
        tools.extend(
            [
                self._tool(
                    "navigation_imu",
                    "Standard IMU for FAST-LIVO2 in the same normalized clock domain as LiDAR",
                    self._node.imu_topic,
                    "sensor/imu",
                    "sensor_msgs/msg/Imu",
                    "RELIABLE + KEEP_LAST(depth=200) + VOLATILE",
                    "livox_frame",
                ),
                self._tool(
                    "navigation_sensor_diagnostics",
                    "Navigation sensor clock and drop diagnostics",
                    self._node.diagnostics_topic,
                    "data/json",
                    "std_msgs/msg/String",
                    "RELIABLE + KEEP_LAST(depth=10) + VOLATILE",
                    "",
                ),
            ]
        )
        return tools

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
        pass

    def stop(self) -> None:
        self._node.close()
        try:
            self._executor.remove_node(self._node)
            self._node.destroy_node()
        except Exception:
            pass

    def dispatch(self, action: str, args: dict) -> dict | None:
        if action in {"start", "info"}:
            tool_name = args.get("_tool_name", "")
            tools = {tool["name"]: tool for tool in self.get_tools()}
            selected = tools.get(tool_name, tools["navigation_sensor_diagnostics"])
            status = self._node.status()
            return {
                "state": "ready" if status["ready"] else "not_ready",
                "topic_out": selected["topic_out"],
                **status,
            }
        if action == "stop":
            return {"state": "idle"}
        return None
