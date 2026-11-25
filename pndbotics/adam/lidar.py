"""Livox MID-360 runtime and ``lidar_cloud`` card for PNDbotics Adam.

The physical lidar sends the Livox UDP protocol directly to the Jetson. A
supervised ``livox_ros_driver2`` child converts that stream to PointCloud2;
this module publishes the compact XYZ wire format consumed by the dashboard.
"""

from __future__ import annotations

import array
import json
import math
import os
import queue
import signal
import struct
import subprocess
import tempfile
import threading
import time
from pathlib import Path

import numpy as np

try:
    from rclpy.node import Node
    from rclpy.qos import (
        DurabilityPolicy,
        HistoryPolicy,
        QoSProfile,
        ReliabilityPolicy,
    )
    from sensor_msgs.msg import PointCloud2
    from std_msgs.msg import UInt8MultiArray

    HAS_ROS2 = True
except Exception:  # Host-side unit tests do not require ROS 2.
    HAS_ROS2 = False
    Node = object
    PointCloud2 = None
    UInt8MultiArray = None


_OUTPUT_POINT_STEP = 12
_FLOAT32 = 7
_DEFAULT_RESTART_DELAY_S = 3.0


def _rotation_matrix(roll_deg: float, pitch_deg: float, yaw_deg: float) -> np.ndarray:
    """Return the XYZ fixed-axis rotation used for the lidar mount."""
    roll, pitch, yaw = (math.radians(float(value))
                        for value in (roll_deg, pitch_deg, yaw_deg))
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.asarray((
        (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
        (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
        (-sp, cp * sr, cp * cr),
    ), dtype=np.float32)


def _mount_transform(config: dict) -> tuple[np.ndarray, np.ndarray]:
    rotation = config.get("mount_rotation_deg", {})
    translation = config.get("mount_translation_m", {})
    if not isinstance(rotation, dict) or not isinstance(translation, dict):
        raise ValueError("lidar mount rotation and translation must be mappings")
    matrix = _rotation_matrix(
        rotation.get("roll", rotation.get("x", 0.0)),
        rotation.get("pitch", rotation.get("y", 0.0)),
        rotation.get("yaw", rotation.get("z", 0.0)),
    )
    offset = np.asarray([
        float(translation.get("x", 0.0)),
        float(translation.get("y", 0.0)),
        float(translation.get("z", 0.0)),
    ], dtype=np.float32)
    if not np.isfinite(matrix).all() or not np.isfinite(offset).all():
        raise ValueError("lidar mount transform must contain finite values")
    return matrix, offset


def encode_pointcloud(message, *, max_points: int, max_distance_m: float | None,
                      rotation: np.ndarray, translation: np.ndarray) -> bytes:
    """Validate PointCloud2 and encode uniformly sampled little-endian XYZ.

    Source points are transformed into Adam's conventional base frame
    (x-forward, y-left, z-up). The final permutation is the inverse of the
    renderer's ``display=(wire_y,-wire_z,-wire_x)`` mapping.
    """
    width, height = int(message.width), int(message.height)
    point_step, row_step = int(message.point_step), int(message.row_step)
    if width < 0 or height < 0 or point_step <= 0:
        raise ValueError("invalid PointCloud2 dimensions")
    if row_step < width * point_step:
        raise ValueError("PointCloud2 row_step is shorter than its packed row")
    raw = memoryview(bytes(message.data))
    if len(raw) < row_step * height:
        raise ValueError("PointCloud2 data is shorter than row_step * height")

    fields = {}
    for field in message.fields:
        name = str(field.name).lower()
        if name not in ("x", "y", "z"):
            continue
        if name in fields:
            raise ValueError(f"duplicate PointCloud2 field: {name}")
        if int(getattr(field, "datatype", _FLOAT32)) != _FLOAT32:
            raise ValueError(f"PointCloud2 {name} must be FLOAT32")
        if int(getattr(field, "count", 1)) != 1:
            raise ValueError(f"PointCloud2 {name} must have count=1")
        offset = int(field.offset)
        if offset < 0 or offset + 4 > point_step:
            raise ValueError(f"PointCloud2 {name} extends beyond point_step")
        fields[name] = offset
    if set(fields) != {"x", "y", "z"}:
        raise ValueError("PointCloud2 is missing x/y/z fields")

    byte_order = ">" if bool(message.is_bigendian) else "<"
    dtype = np.dtype({
        "names": ["x", "y", "z"],
        "formats": [byte_order + "f4"] * 3,
        "offsets": [fields[name] for name in ("x", "y", "z")],
        "itemsize": point_step,
    })
    rows = []
    for row in range(height):
        rows.append(np.ndarray(
            (width,), dtype=dtype, buffer=raw, offset=row * row_step))
    if rows:
        view = rows[0] if len(rows) == 1 else np.concatenate(rows)
        points = np.column_stack((view["x"], view["y"], view["z"]))
    else:
        points = np.empty((0, 3), dtype=np.float32)
    points = np.asarray(points, dtype=np.float32)
    valid = np.isfinite(points).all(axis=1)
    if max_distance_m is not None and max_distance_m > 0:
        valid &= np.einsum("ij,ij->i", points, points) <= float(max_distance_m) ** 2
    points = points[valid]

    limit = max(1, min(int(max_points), 40000))
    if len(points) > limit:
        indices = np.linspace(0, len(points) - 1, limit, dtype=np.int64)
        points = points[indices]
    if len(points):
        base = points @ np.asarray(rotation, dtype=np.float32).T
        base += np.asarray(translation, dtype=np.float32)
        packed = np.empty_like(base, dtype="<f4")
        packed[:, 0] = -base[:, 2]
        packed[:, 1] = base[:, 0]
        packed[:, 2] = -base[:, 1]
    else:
        packed = np.empty((0, 3), dtype="<f4")
    return struct.pack("<II", _OUTPUT_POINT_STEP, len(packed)) + packed.tobytes()


def build_livox_config(config: dict) -> dict:
    """Build the SDK2 MID-360 network configuration from Adam YAML values."""
    host_ip = str(config.get("host_ip", "192.168.1.50"))
    lidar_ip = str(config.get("lidar_ip", "192.168.1.173"))
    return {
        # SDK2 firmware logging defaults to enabled and can consume hundreds of
        # MiB. Container logs already capture driver diagnostics.
        "lidar_log_enable": False,
        "lidar_summary_info": {"lidar_type": 8},
        "MID360": {
            "lidar_net_info": {
                "cmd_data_port": 56100,
                "push_msg_port": 56200,
                "point_data_port": 56300,
                "imu_data_port": 56400,
                "log_data_port": 56500,
            },
            # Match the pinned livox_ros_driver2 MID360_config.json shape.
            "host_net_info": {
                "cmd_data_ip": host_ip,
                "cmd_data_port": 56101,
                "push_msg_ip": host_ip,
                "push_msg_port": 56201,
                "point_data_ip": host_ip,
                "point_data_port": 56301,
                "imu_data_ip": host_ip,
                "imu_data_port": 56401,
                "log_data_ip": "",
                "log_data_port": 56501,
            },
        },
        "lidar_configs": [{
            "ip": lidar_ip,
            "pcl_data_type": 1,
            "pattern_mode": int(config.get("pattern_mode", 0)),
            "extrinsic_parameter": {
                "roll": 0.0, "pitch": 0.0, "yaw": 0.0,
                "x": 0, "y": 0, "z": 0,
            },
        }],
    }


class LivoxDriverRuntime:
    """Supervise the ROS driver child for the Adam bundle lifetime."""

    def __init__(self, config: dict, *, process_factory=subprocess.Popen):
        self._config = dict(config)
        self._factory = process_factory
        self._process = None
        self._lock = threading.RLock()
        self._stop = threading.Event()
        self._thread = None
        self._restart_count = 0
        self._last_exit_code = None
        self._last_error = None
        configured_path = self._config.get("driver_config_path")
        self._config_path = (
            Path(str(configured_path)) if configured_path else
            Path(tempfile.gettempdir()) / "adam-mid360-config.json")

    def _write_config(self):
        self._config_path.parent.mkdir(parents=True, exist_ok=True)
        temporary = self._config_path.with_suffix(self._config_path.suffix + ".tmp")
        temporary.write_text(json.dumps(build_livox_config(self._config), indent=2) + "\n")
        temporary.replace(self._config_path)

    def command(self) -> list[str]:
        return [
            "ros2", "run", "livox_ros_driver2", "livox_ros_driver2_node",
            "--ros-args",
            "-p", "xfer_format:=0",
            "-p", "multi_topic:=0",
            "-p", "data_src:=0",
            "-p", f"publish_freq:={float(self._config.get('publish_freq_hz', 10.0))}",
            "-p", "output_data_type:=0",
            "-p", f"frame_id:={self._config.get('frame_id', 'livox_frame')}",
            "-p", f"user_config_path:={self._config_path}",
            "-p", "cmdline_input_bd_code:=livox0000000001",
        ]

    def start(self):
        with self._lock:
            if self._thread is not None and self._thread.is_alive():
                return
            self._write_config()
            self._stop.clear()
            self._thread = threading.Thread(
                target=self._supervise, daemon=True, name="adam-livox-supervisor")
            self._thread.start()

    def _supervise(self):
        while not self._stop.is_set():
            try:
                process = self._factory(self.command(), start_new_session=True)
                with self._lock:
                    self._process = process
                    self._last_error = None
                exit_code = process.wait()
                with self._lock:
                    self._last_exit_code = exit_code
                    self._process = None
                    if not self._stop.is_set():
                        self._restart_count += 1
            except Exception as exc:
                with self._lock:
                    self._process = None
                    self._last_error = f"{type(exc).__name__}: {exc}"
                    if not self._stop.is_set():
                        self._restart_count += 1
            if not self._stop.wait(float(self._config.get(
                    "restart_delay_s", _DEFAULT_RESTART_DELAY_S))):
                print("[adam:livox] driver stopped; restarting", flush=True)

    def stop(self):
        self._stop.set()
        with self._lock:
            process = self._process
            thread = self._thread
        if process is not None and process.poll() is None:
            try:
                os.killpg(process.pid, signal.SIGTERM)
            except (ProcessLookupError, PermissionError):
                process.terminate()
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(process.pid, signal.SIGKILL)
                except (ProcessLookupError, PermissionError):
                    process.kill()
                process.wait(timeout=2)
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=7)
        with self._lock:
            self._process = None
            self._thread = None

    def status(self) -> dict:
        with self._lock:
            process = self._process
            running = process is not None and process.poll() is None
            supervising = self._thread is not None and self._thread.is_alive()
            return {
                "driver_state": (
                    "running" if running else "starting" if supervising else "stopped"),
                "driver_pid": process.pid if running else None,
                "driver_exit_code": self._last_exit_code,
                "driver_restarts": self._restart_count,
                "driver_error": self._last_error,
                "driver_config_path": str(self._config_path),
            }


class _LidarNode(Node):
    def __init__(self, config: dict, namespace: str):
        super().__init__("adam_livox_lidar")
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.output_topic = f"/{namespace}/lidar/cloud"
        self.source_topic = str(config.get("source_topic", "/livox/lidar"))
        self._publisher = self.create_publisher(UInt8MultiArray, self.output_topic, qos)
        self._subscription = self.create_subscription(
            PointCloud2, self.source_topic, self._on_cloud, qos)
        self._queue = queue.Queue(maxsize=1)
        self._stop = threading.Event()
        self._active = True
        self._lock = threading.Lock()
        self._received = self._published = self._dropped = self._invalid = 0
        self._last_received = None
        self._last_published = None
        self._max_points = max(1, min(int(config.get("max_points", 10000)), 40000))
        distance = config.get("max_distance_m")
        self._max_distance_m = None if distance in (None, 0, 0.0) else float(distance)
        self._rotation, self._translation = _mount_transform(config)
        self._worker = threading.Thread(
            target=self._work, daemon=True, name="adam-livox-pointcloud")
        self._worker.start()

    def set_active(self, active: bool):
        with self._lock:
            self._active = bool(active)

    def _on_cloud(self, message):
        now = time.monotonic()
        with self._lock:
            self._received += 1
            self._last_received = now
            active = self._active
        if not active:
            return
        try:
            self._queue.put_nowait(message)
        except queue.Full:
            try:
                self._queue.get_nowait()
            except queue.Empty:
                pass
            with self._lock:
                self._dropped += 1
            try:
                self._queue.put_nowait(message)
            except queue.Full:
                pass

    def _work(self):
        while not self._stop.is_set():
            try:
                message = self._queue.get(timeout=0.2)
            except queue.Empty:
                continue
            try:
                payload = encode_pointcloud(
                    message, max_points=self._max_points,
                    max_distance_m=self._max_distance_m,
                    rotation=self._rotation, translation=self._translation)
                output = UInt8MultiArray()
                output.data = array.array("B", payload)
                self._publisher.publish(output)
                with self._lock:
                    self._published += 1
                    self._last_published = time.monotonic()
            except Exception as exc:
                with self._lock:
                    self._invalid += 1
                    invalid = self._invalid
                if invalid == 1 or invalid % 100 == 0:
                    self.get_logger().warning(f"Adam lidar dropped invalid cloud: {exc}")

    def status(self) -> dict:
        now = time.monotonic()
        with self._lock:
            return {
                "source_topic": self.source_topic,
                "received": self._received,
                "published": self._published,
                "dropped": self._dropped,
                "invalid": self._invalid,
                "last_frame_age_s": (
                    None if self._last_received is None else round(now - self._last_received, 3)),
                "last_publish_age_s": (
                    None if self._last_published is None else round(now - self._last_published, 3)),
                "max_points": self._max_points,
            }

    def close(self):
        self._stop.set()
        self._worker.join(timeout=2)
        self.destroy_node()


class LivoxLidarPlugin:
    PREFIX = "lidar"

    def __init__(self, plugin_config: dict, namespace: str, executor,
                 *, node_factory=_LidarNode, runtime_factory=LivoxDriverRuntime):
        if not HAS_ROS2 and node_factory is _LidarNode:
            raise RuntimeError("ROS2 is unavailable")
        self._executor = executor
        self._node = node_factory(plugin_config, namespace)
        self._runtime = runtime_factory(plugin_config)
        self._active = True
        executor.add_node(self._node)

    def get_tool(self) -> dict:
        return {
            "name": "lidar_cloud",
            "type": "sensor",
            "multiInstance": False,
            "description": (
                "Adam Livox MID-360 real-time point cloud; compact little-endian "
                f"XYZ stream produced from {self._node.source_topic}."),
            "inputSchema": {"type": "object", "properties": {}},
            "topic_out": [{"topic": self._node.output_topic,
                           "format": "sensor/pointcloud"}],
        }

    def start(self):
        self._runtime.start()
        self._active = True
        self._node.set_active(True)

    def stop(self):
        self._active = False
        self._node.set_active(False)
        self._runtime.stop()

    def close(self):
        self._runtime.stop()
        try:
            self._executor.remove_node(self._node)
        except Exception:
            pass
        self._node.close()

    def dispatch(self, action: str, args: dict) -> dict | None:
        if action == "start":
            self._runtime.start()
            self._active = True
            self._node.set_active(True)
            return self._info("running")
        if action == "stop":
            # Keep the Livox runtime warm so reopening a card does not rebind
            # UDP ports or wait for the lidar to reconnect.
            self._active = False
            self._node.set_active(False)
            return self._info("idle")
        if action in ("info", "status", "lidar_cloud"):
            return self._info("running" if self._active else "idle")
        return None

    def _info(self, state: str) -> dict:
        return {
            "state": state,
            "topic_out": [{"topic": self._node.output_topic,
                           "format": "sensor/pointcloud"}],
            **self._runtime.status(),
            **self._node.status(),
        }
