"""Read the AS2W display cloud and publish a directional obstacle summary.

Distances are horizontal ranges from the configured base-link origin.  They
are observations from a sampled cloud, not verified free-space measurements.
"""
import json
import math
import struct
import threading
import time


_SECTORS = ("front", "front_left", "front_right", "left", "right", "rear")
_UNKNOWN = "unknown"


def _number(config, name, default):
    value = config.get(name, default)
    if isinstance(value, bool):
        raise ValueError(f"{name} must be a finite number")
    try:
        value = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name} must be a finite number") from exc
    if not math.isfinite(value):
        raise ValueError(f"{name} must be a finite number")
    return value


def _positive_integer(config, name, default):
    value = _number(config, name, default)
    if value < 1 or not value.is_integer():
        raise ValueError(f"{name} must be a positive integer")
    return int(value)


def _sector(x, y):
    """Six non-overlapping sectors in the base frame, +X forward, +Y left."""
    angle = math.degrees(math.atan2(y, x))
    if -30 <= angle <= 30:
        return "front"
    if 30 < angle <= 60:
        return "front_left"
    if 60 < angle <= 120:
        return "left"
    if -60 <= angle < -30:
        return "front_right"
    if -120 <= angle < -60:
        return "right"
    return "rear"


def _unknown_obstacles():
    return {name: {"distance_m": None, "status": _UNKNOWN, "point_count": 0}
            for name in _SECTORS}


class ObstacleMonitorCore:
    """Point-cloud parsing and status logic, independent of ROS and hardware."""

    def __init__(self, config, monotonic=time.monotonic, wall_clock=time.time):
        self._critical = _number(config, "critical_distance_m", 0.4)
        self._warning = _number(config, "warning_distance_m", 1.5)
        self._max_range = _number(config, "max_range_m", 5.0)
        self._z_min = _number(config, "min_height_m", -0.25)
        self._z_max = _number(config, "max_height_m", 1.0)
        self._timeout = _number(config, "timeout_s", 1.5)
        self._confirm_frames = _positive_integer(config, "confirm_frames", 2)
        self._min_clear_points = _positive_integer(config, "min_clear_points", 3)
        # These offsets are from the JT128 joint in resource/as2w.urdf.  The
        # actual active cloud source and mounting must be verified on the robot.
        self._offset_x = _number(config, "lidar_offset_x_m", 0.27666)
        self._offset_y = _number(config, "lidar_offset_y_m", 0.0)
        self._offset_z = _number(config, "lidar_offset_z_m", 0.078863)
        if not 0 < self._critical < self._warning < self._max_range:
            raise ValueError("require 0 < critical_distance_m < warning_distance_m < max_range_m")
        if self._z_min >= self._z_max:
            raise ValueError("min_height_m must be less than max_height_m")
        if self._timeout <= 0:
            raise ValueError("timeout_s must be positive")
        self._monotonic = monotonic
        self._wall_clock = wall_clock
        self._lock = threading.Lock()
        self.reset()

    def _reset_locked(self):
        self._last_received = None
        self._received_at = None
        self._valid_frame = False
        self._candidates = {name: (None, 0) for name in _SECTORS}
        self._obstacles = _unknown_obstacles()

    def reset(self):
        with self._lock:
            self._reset_locked()

    def _parse(self, payload):
        """Decode [little-endian step, count, XYZ points] from lidar_cloud."""
        try:
            data = bytes(payload)
        except (TypeError, ValueError):
            return None
        if len(data) < 8:
            return None
        step, count = struct.unpack_from("<II", data)
        if step < 12 or count == 0 or count > (len(data) - 8) // step:
            return None
        nearest = {name: None for name in _SECTORS}
        counts = {name: 0 for name in _SECTORS}
        finite_count = 0
        for index in range(count):
            wire_x, wire_y, wire_z = struct.unpack_from("<fff", data, 8 + index * step)
            if not all(math.isfinite(value) for value in (wire_x, wire_y, wire_z)):
                continue
            # Zero triples are commonly placeholder returns, not observations.
            if wire_x == wire_y == wire_z == 0.0:
                continue
            finite_count += 1
            # lidar_cloud's wire order is (-base_z, base_x, -base_y), with
            # axes rotated but without the URDF mount translation.
            x = wire_y + self._offset_x
            y = -wire_z + self._offset_y
            z = -wire_x + self._offset_z
            distance = math.hypot(x, y)
            if not self._z_min <= z <= self._z_max or not 0 < distance <= self._max_range:
                continue
            sector = _sector(x, y)
            counts[sector] += 1
            if nearest[sector] is None or distance < nearest[sector]:
                nearest[sector] = distance
        if finite_count == 0:
            return None
        return nearest, counts

    def consume(self, payload):
        received = self._monotonic()
        received_at = self._wall_clock()
        parsed = self._parse(payload)
        with self._lock:
            if (self._last_received is not None and
                    received - self._last_received > self._timeout):
                self._candidates = {name: (None, 0) for name in _SECTORS}
            self._last_received = received
            self._received_at = received_at
            if parsed is None:
                self._valid_frame = False
                self._candidates = {name: (None, 0) for name in _SECTORS}
                self._obstacles = _unknown_obstacles()
                return
            self._valid_frame = True
            nearest, counts = parsed
            obstacles = {}
            for name in _SECTORS:
                distance, count = nearest[name], counts[name]
                if distance is None:
                    candidate = _UNKNOWN
                elif distance <= self._critical:
                    candidate = "critical"
                elif distance <= self._warning:
                    candidate = "warning"
                elif count >= self._min_clear_points:
                    candidate = "clear"
                else:
                    candidate = _UNKNOWN
                previous, frames = self._candidates[name]
                if candidate == _UNKNOWN:
                    self._candidates[name] = (None, 0)
                    status = _UNKNOWN
                else:
                    frames = frames + 1 if previous == candidate else 1
                    self._candidates[name] = (candidate, frames)
                    # An unconfirmed observation is unknown, never stale clear.
                    status = candidate if frames >= self._confirm_frames else _UNKNOWN
                obstacles[name] = {
                    "distance_m": round(distance, 3) if distance is not None else None,
                    "status": status,
                    "point_count": count,
                }
            self._obstacles = obstacles

    def snapshot(self, running=True):
        now = self._monotonic()
        timestamp = self._wall_clock()
        with self._lock:
            age = None if self._last_received is None else max(0.0, now - self._last_received)
            if not running:
                sensor_status = "not_running"
            elif age is None:
                sensor_status = "no_data"
            elif age > self._timeout:
                sensor_status = "stale"
            elif not self._valid_frame:
                sensor_status = "invalid"
            else:
                sensor_status = "healthy"
            obstacles = ({name: dict(value) for name, value in self._obstacles.items()}
                         if sensor_status == "healthy" else _unknown_obstacles())
            states = {value["status"] for value in obstacles.values()}
            overall = next(status for status in ("critical", "warning", _UNKNOWN, "clear")
                           if status in states)
            return {
                "timestamp": timestamp,
                "received_at": self._received_at,
                "receive_age_ms": None if age is None else round(age * 1000),
                # The source's UInt8MultiArray does not contain acquisition time.
                "source_timestamp": None,
                "frame_id": "base_link",
                "geometry_status": "unverified",
                "distance_reference": "base_link_origin_horizontal",
                "sensor_status": sensor_status,
                "overall_status": overall,
                "obstacles": obstacles,
            }


class _ObstacleMonitorNode:
    def __init__(self, input_topic, output_topic, executor, core, publish_hz):
        from rclpy.node import Node
        from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
        from std_msgs.msg import String, UInt8MultiArray

        self._executor = executor
        self._core = core
        self._String = String
        self._lock = threading.Lock()
        self._closed = False
        self.node = Node("as2w_obstacle_monitor")
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=1,
                         durability=DurabilityPolicy.VOLATILE)
        try:
            self._pub = self.node.create_publisher(String, output_topic, qos)
            self._sub = self.node.create_subscription(UInt8MultiArray, input_topic,
                                                      self._on_cloud, qos)
            self._timer = self.node.create_timer(1.0 / publish_hz, self._publish)
            executor.add_node(self.node)
        except Exception:
            try:
                executor.remove_node(self.node)
            except Exception:
                pass
            self.node.destroy_node()
            raise

    def _on_cloud(self, message):
        with self._lock:
            if not self._closed:
                self._core.consume(message.data)

    def _publish(self):
        with self._lock:
            if not self._closed:
                message = self._String()
                message.data = json.dumps(self._core.snapshot(), separators=(",", ":"))
                self._pub.publish(message)

    def close(self):
        with self._lock:
            if self._closed:
                return
            self._closed = True
            try:
                self._executor.remove_node(self.node)
            except Exception:
                pass
            self.node.destroy_node()


class ObstacleMonitorPlugin:
    PREFIX = "obstacle_monitor"

    def __init__(self, config, namespace, executor):
        self._input_topic = f"/{namespace}/lidar/cloud" if namespace else "/lidar/cloud"
        self._output_topic = (f"/{namespace}/obstacle/monitor" if namespace
                              else "/obstacle/monitor")
        self._publish_hz = _number(config, "publish_hz", 5.0)
        if self._publish_hz <= 0:
            raise ValueError("publish_hz must be positive")
        self._core = ObstacleMonitorCore(config)
        self._executor = executor
        self._node = None
        self._lock = threading.RLock()

    def get_tool(self):
        return {
            "name": self.PREFIX,
            "type": "sensor",
            "multiInstance": False,
            "description": ("Approximate directional obstacle distances from the AS2W "
                            "lidar_cloud stream. Requires calibrated geometry and is not "
                            "a collision-prevention system."),
            "inputSchema": {"type": "object", "properties": {
                "action": {"type": "string", "enum": ["start", "stop", "info", "read"]}}},
            "topic_out": [{"topic": self._output_topic, "format": "data/json"}],
        }

    def start(self):
        with self._lock:
            if self._node is None:
                self._core.reset()
                self._node = _ObstacleMonitorNode(self._input_topic, self._output_topic,
                                                  self._executor, self._core,
                                                  self._publish_hz)

    def stop(self):
        with self._lock:
            if self._node is not None:
                self._node.close()
                self._node = None
            self._core.reset()

    def dispatch(self, action, _args):
        with self._lock:
            if action in (self.PREFIX, "start"):
                self.start()
            elif action == "stop":
                self.stop()
            elif action not in ("info", "read"):
                return None
            result = self._core.snapshot(running=self._node is not None)
            result["state"] = "running" if self._node is not None else "idle"
            result["input_topic"] = self._input_topic
            result["topic_out"] = [{"topic": self._output_topic, "format": "data/json"}]
            return result
