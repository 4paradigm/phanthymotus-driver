"""Read-only Tianyi telemetry and source-timestamped chassis odometry.

Sources follow the supplied Slamware OpenAPI 1.1.0 document. Odometry uses
localization/odopose for telemetry; the SLAM port integrates diagnostic increments.
"""

import copy
import http.client
import json
import math
import posixpath
import threading
import time
from urllib.parse import urlsplit

from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import String

from common.odom import build_interface, build_sample, mean_twist, resolve_stamp_ms
from loco_odometry import OdometrySource, CLOCK_ID, UNCERTAINTY_NS


class _LocoStateClient:
    """Thread-local persistent HTTP transport for the read-only state card.

    ``HTTPConnection`` is not safe to share between the fast and slow polling
    lanes, so each worker thread owns one keep-alive connection.  Failed or
    server-closed connections are discarded and recreated by the next request.
    """

    def __init__(self, base_url):
        self._base = base_url.rstrip("/")
        parsed = urlsplit(self._base)
        if parsed.scheme != "http" or not parsed.hostname:
            raise ValueError("loco_state base_url must be an http URL")
        self._host = parsed.hostname
        self._port = parsed.port or 80
        self._prefix = parsed.path.rstrip("/")
        self._local = threading.local()
        self._connections = set()
        self._connections_lock = threading.Lock()

    def _connection(self, timeout):
        connection = getattr(self._local, "connection", None)
        if connection is None:
            connection = http.client.HTTPConnection(
                self._host, self._port, timeout=timeout)
            self._local.connection = connection
            with self._connections_lock:
                self._connections.add(connection)
        else:
            try:
                connection.timeout = timeout
                if connection.sock is not None:
                    connection.sock.settimeout(timeout)
            except Exception:
                # A locally closed/broken keep-alive socket may fail before
                # request() gets a chance to reconnect.  Do not pin this
                # worker to that object forever: discard it and let the same
                # read proceed on a fresh connection.
                self._discard(connection)
                connection = http.client.HTTPConnection(
                    self._host, self._port, timeout=timeout)
                self._local.connection = connection
                with self._connections_lock:
                    self._connections.add(connection)
        return connection

    def _discard(self, connection):
        try:
            connection.close()
        except Exception:
            pass
        if getattr(self._local, "connection", None) is connection:
            self._local.connection = None
        with self._connections_lock:
            self._connections.discard(connection)

    def close(self):
        with self._connections_lock:
            connections = list(self._connections)
            self._connections.clear()
        for connection in connections:
            try:
                connection.close()
            except Exception:
                pass
        self._local = threading.local()

    def _get(self, path, *, timeout, scalar_key=None):
        connection = self._connection(timeout)
        full_path = posixpath.join(self._prefix or "/", path.lstrip("/"))
        if not full_path.startswith("/"):
            full_path = "/" + full_path
        try:
            connection.request("GET", full_path, headers={"Accept": "application/json"})
            response = connection.getresponse()
            body = response.read()
            if not 200 <= response.status < 300:
                raise RuntimeError(f"HTTP {response.status}")
            result = json.loads(body)
            if response.will_close:
                self._discard(connection)
            if scalar_key is not None:
                value = _number(result)
                if value is None or value < 0:
                    return {"error": "invalid_total_odometry"}
                return {scalar_key: value}
            if not isinstance(result, dict):
                return {"error": "invalid_response_type"}
            return result
        except Exception as exc:
            self._discard(connection)
            return {"error": str(exc)}

    def get_odometry_pose(self, *, timeout):
        return self._get("/api/core/slam/v1/localization/odopose", timeout=timeout)

    def get_speed(self, *, timeout):
        return self._get("/api/core/motion/v1/speed", timeout=timeout)

    def get_imu(self, *, timeout):
        return self._get("/api/core/slam/v1/imu", timeout=timeout)

    def get_total_odometry(self, *, timeout):
        return self._get("/api/core/statistics/v1/odometry", timeout=timeout,
                         scalar_key="length")


def _number(value):
    if isinstance(value, (int, float)) and not isinstance(value, bool):
        if math.isfinite(value):
            return value
    return None


def _vector(data, keys):
    if not isinstance(data, dict):
        return None
    values = [_number(data.get(key)) for key in keys]
    return values if all(value is not None for value in values) else None


def _imu_state(data):
    """Discard fields whose vendor validity bits are unset; retain native units."""
    if not isinstance(data, dict) or "error" in data:
        return {"available": False}
    bitmap = data.get("availibilityBitMap")  # Spelling is part of the vendor API.
    if not isinstance(bitmap, int) or isinstance(bitmap, bool) or bitmap < 0:
        return {"available": False}
    result = {"available": False, "availibilityBitMap": bitmap}
    for bit, source, target, keys in (
        (1, "quaternion", "quaternion", "wxyz"),
        (2, "acc", "accelerometer", "xyz"),
        (4, "gyro", "gyroscope", "xyz"),
        (8, "compass", "compass", "xyz"),
        (16, "raw_acc", "raw_accelerometer", "xyz"),
        (32, "raw_gyro", "raw_gyroscope", "xyz"),
        (64, "raw_compass", "raw_compass", "xyz"),
        (512, "euler_angle", "euler_angle", "xyz"),
    ):
        value = _vector(data.get(source), keys)
        if bitmap & bit and value is not None:
            result[target] = value
            result["available"] = True
    timestamp = _number(data.get("timestamp"))
    if timestamp is not None:
        result["uptime_ms"] = timestamp  # Not a Unix timestamp.
    return result


def locomotion_snapshot(readings, timestamps):
    """Build one poll's state, without substituting old or invented readings."""
    pose = readings.get("pose", {})
    speed = readings.get("speed", {})
    position = _vector(pose, "xyz") if "error" not in pose else None
    rpy = _vector(pose, ("roll", "pitch", "yaw")) if "error" not in pose else None
    motion = ([_number(speed.get(key)) for key in ("vx", "vy", "omega")]
              if "error" not in speed else [None, None, None])
    speed_available = all(value is not None for value in motion)
    errors = {key: value["error"] for key, value in readings.items() if "error" in value}
    if position is None or rpy is None:
        errors.setdefault("pose", "missing_or_invalid_odometry_pose")
    if not speed_available:
        errors.setdefault("speed", "missing_or_invalid_measured_speed")
    imu = _imu_state(readings.get("imu"))
    if not imu["available"]:
        errors.setdefault("imu", "no_valid_chassis_imu_fields")
    length = _number(readings.get("length", {}).get("length"))
    if length is None or length < 0:
        length = None
        errors.setdefault("length", "missing_or_invalid_total_odometry")
    return {
        "timestamp_ms": max(timestamps.values()) if timestamps else int(time.time() * 1000),
        "sample_timestamps_ms": dict(timestamps),
        "available": position is not None and rpy is not None and speed_available,
        "source": "slamtec",
        "pose_source": "/api/core/slam/v1/localization/odopose",
        "frame_id": "odom",
        "child_frame_id": "base_link",
        "units": {"linear": "m/s", "angular": "rad/s", "length": "m"},
        "position": position,
        "rpy": rpy,
        "velocity": [motion[0], motion[1], None],  # No vertical speed sensor.
        "yaw_speed": motion[2],
        "v": motion[0],
        "w": motion[2],
        "length": length,
        "displacement": None,
        "motion_status": ("moving" if any(abs(v) > 0.001 for v in motion) else "stationary")
                         if speed_available else "unknown",
        "mode": None,
        "body_height": None,
        "unavailable_fields": ["mode", "body_height"],
        "imu": imu,
        "errors": errors,
    }


def _pose(snapshot):
    """Measured odometry pose; twist's body frame does not relabel its position."""
    position = snapshot["position"]
    rpy = snapshot["rpy"]
    if position is None or rpy is None:
        return None
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return {
        "stamp_ms": snapshot["sample_timestamps_ms"].get("pose", snapshot["timestamp_ms"]),
        "frame_id": "odom", "child_frame_id": "base_link",
        "position": dict(zip("xyz", position)),
        "orientation": {"x": sr * cp * cy - cr * sp * sy,
                        "y": cr * sp * cy + sr * cp * sy,
                        "z": cr * cp * sy - sr * sp * cy,
                        "w": cr * cp * cy + sr * sp * sy},
    }


class _DisplacementTracker:
    """Translation from the first valid pose, not integrated speed or path length."""

    def __init__(self):
        self._origin = None

    def update(self, snapshot):
        snapshot["displacement"] = None
        pose = _pose(snapshot)
        if pose is None:
            return snapshot
        if self._origin is None:
            self._origin = {"stamp_ms": pose["stamp_ms"],
                            "position": dict(pose["position"]),
                            "yaw": snapshot["rpy"][2]}
        delta = {axis: pose["position"][axis] - self._origin["position"][axis]
                 for axis in "xyz"}
        yaw_delta = snapshot["rpy"][2] - self._origin["yaw"]
        snapshot["displacement"] = {
            "stamp_ms": pose["stamp_ms"], "frame_id": "odom",
            "origin": copy.deepcopy(self._origin),
            "translation": delta,
            "distance": math.hypot(*delta.values()),
            "yaw": math.atan2(math.sin(yaw_delta), math.cos(yaw_delta)),
        }
        return snapshot


def odometry_sample(snapshot):
    """The same poll in motus.odom/1, including measured pose in the odom frame.

    The REST speed endpoint has no measurement timestamp. Stamp the reply's
    arrival, without confusing the IMU uptime with a wall-clock observation.
    There is one REST speed reading per window, so its mean is the reading.
    """
    velocity = snapshot["velocity"]
    stamp, provenance = resolve_stamp_ms(
        vendor_ms=None,
        received_ms=snapshot["sample_timestamps_ms"].get("speed", snapshot["timestamp_ms"]))
    return build_sample(
        stamp_ms=stamp,
        twist=mean_twist([[velocity[0], velocity[1], None, None, None,
                           snapshot["yaw_speed"]]]),
        pose=_pose(snapshot),
        vendor={**copy.deepcopy(snapshot), **provenance,
                "samples": int(any(value is not None for value in
                                   (velocity[0], velocity[1], snapshot["yaw_speed"])))},
    )


class LocoStatePlugin:
    """Always-on observer; MCP stop leaves polling active, shutdown stops it."""

    def __init__(self, plugin_config, namespace, ros2, base_url):
        self._client = _LocoStateClient(base_url)
        self._hz = float(plugin_config.get("hz", 20))
        self._imu_hz = float(plugin_config.get("imu_hz", 5))
        self._length_hz = float(plugin_config.get("length_hz", 1))
        self._timeout = float(plugin_config.get("request_timeout", 0.2))
        if not math.isfinite(self._hz) or not 0 < self._hz <= 50:
            raise ValueError("loco_state.hz must be in (0, 50]")
        for name, value in (("imu_hz", self._imu_hz),
                            ("length_hz", self._length_hz)):
            if not math.isfinite(value) or not 0 < value <= 50:
                raise ValueError(f"loco_state.{name} must be in (0, 50]")
        if not math.isfinite(self._timeout) or not 0 < self._timeout <= 5:
            raise ValueError("loco_state.request_timeout must be in (0, 5]")
        self._topic = f"/{namespace}/loco/state"
        self._odom_topic = f"/{namespace}/state/odom"
        self._odom_interface = build_interface(
            provides=["vx", "vy", "wz"], rate_hz=self._hz,
            pose_drift="unbounded", timing={"stamp_source": "robot"})
        self._odom_interface["slam_pose"] = {"available": True, "profile": "motus_slam/1"}
        self._node = Node("tianyi2_loco_state_pub", context=ros2.ctx_core)
        ros2.executor_core.add_node(self._node)
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=1,
                         durability=DurabilityPolicy.VOLATILE)
        self._pub = self._node.create_publisher(String, self._topic, qos)
        self._odom_pub = self._node.create_publisher(String, self._odom_topic, qos)
        self._lock = threading.Lock()
        self._lifecycle_lock = threading.Lock()
        self._stop_event = threading.Event()
        self._thread = None
        self._slow_thread = None
        self._slow_lock = threading.Lock()
        self._slow_readings = {
            "imu": {"error": "not_sampled_yet"},
            "length": {"error": "not_sampled_yet"},
        }
        self._slow_timestamps = {}
        self._latest = locomotion_snapshot({}, {})
        self._displacement = _DisplacementTracker()
        self._samples = 0
        self._odometry = OdometrySource(base_url, self._hz, self._publish_odometry)

    def get_tool(self):
        return {
            "name": "loco_state", "type": "sensor", "multiInstance": False,
            "description": (
                "天轶2.0 locomotion state (always active) — odometry position, "
                "measured velocity, displacement, total mileage and chassis IMU; "
                "mode/body_height unavailable. "
                f"Persistent HTTP: pose/speed {self._hz:g}Hz, IMU {self._imu_hz:g}Hz, "
                f"mileage {self._length_hz:g}Hz to {self._topic}, "
                f"and source-timestamped planar SLAM odometry to {self._odom_topic}"),
            "inputSchema": {"type": "object", "properties": {}},
            "topic_out": [{"topic": self._topic, "format": "data/json"},
                          {"topic": self._odom_topic, "format": "state/odom"}],
        }

    def start(self):
        with self._lifecycle_lock:
            if self._thread is not None and self._thread.is_alive():
                return
            self._stop_event.clear()
            self._slow_thread = threading.Thread(
                target=self._slow_poll_loop,
                name="tianyi-loco-state-slow", daemon=True)
            self._thread = threading.Thread(target=self._poll_loop,
                                            name="tianyi-loco-state", daemon=True)
            self._slow_thread.start()
            self._thread.start()
            self._odometry.start()

    def stop(self):
        # Shutdown only. Never clear this event while the old worker is alive.
        with self._lifecycle_lock:
            self._stop_event.set()
            self._odometry.stop()
            if self._thread is not None:
                self._thread.join(timeout=self._timeout + 1)
            if self._slow_thread is not None:
                self._slow_thread.join(timeout=self._timeout + 1)
            close = getattr(self._client, "close", None)
            if close is not None:
                close()

    def _read(self, method):
        try:
            result = method(timeout=self._timeout)
            if not isinstance(result, dict):
                result = {"error": "invalid_response_type"}
        except Exception as exc:
            result = {"error": str(exc)}
        return result, int(time.time() * 1000)

    def _poll_once(self, _unused_pool=None):
        # Pose and speed are the control feedback path.  Read them sequentially
        # over one keep-alive connection; neither waits for telemetry that is
        # useful to humans but irrelevant to the control loop.
        readings, timestamps = {}, {}
        for key, method in (("pose", self._client.get_odometry_pose),
                            ("speed", self._client.get_speed)):
            readings[key], timestamps[key] = self._read(method)
        with self._slow_lock:
            readings.update(copy.deepcopy(self._slow_readings))
            timestamps.update(self._slow_timestamps)
        return locomotion_snapshot(readings, timestamps)

    def _sample_slow(self, key, method):
        reading, stamp = self._read(method)
        with self._slow_lock:
            self._slow_readings[key] = reading
            self._slow_timestamps[key] = stamp

    def _slow_poll_loop(self):
        schedules = {
            "imu": [self._client.get_imu, 1.0 / self._imu_hz, 0.0],
            "length": [self._client.get_total_odometry, 1.0 / self._length_hz, 0.0],
        }
        while not self._stop_event.is_set():
            now = time.monotonic()
            for key, schedule in schedules.items():
                method, period, due = schedule
                if now >= due:
                    self._sample_slow(key, method)
                    schedule[2] = time.monotonic() + period
            wait = min(schedule[2] for schedule in schedules.values()) - time.monotonic()
            self._stop_event.wait(max(0, wait))

    def _poll_loop(self):
        while not self._stop_event.is_set():
            started = time.monotonic()
            snapshot = self._poll_once()
            if self._stop_event.is_set():
                break
            snapshot = self._displacement.update(snapshot)
            with self._lock:
                self._latest = snapshot
                self._samples += 1
            out = String()
            out.data = json.dumps(snapshot, allow_nan=False)
            self._pub.publish(out)
            self._stop_event.wait(max(0, 1 / self._hz - (time.monotonic() - started)))

    def _publish_odometry(self, sample):
        # Preserve REST telemetry under vendor, never substitute it for a lost
        # diagnostic pose or stamp the pose with HTTP arrival time.
        with self._lock:
            latest = copy.deepcopy(self._latest)
        sample = {**sample, "vendor": {**latest, **sample["vendor"]}}
        message = String()
        message.data = json.dumps(sample, allow_nan=False)
        self._odom_pub.publish(message)

    def dispatch(self, action, args):
        if action == "stop":
            # Canvas lifecycle acknowledgement. The always-active observer keeps
            # polling until driver shutdown calls stop().
            return {"state": "idle"}
        if action == "start":
            self.start()
        if action in ("read", "get"):
            with self._lock:
                return copy.deepcopy(self._latest)
        if action in ("start", "info"):
            running = (self._thread is not None and self._thread.is_alive()
                       and self._slow_thread is not None and self._slow_thread.is_alive()
                       and not self._stop_event.is_set())
            with self._lock:
                latest = copy.deepcopy(self._latest)
                samples = self._samples
            state = "running" if running else "idle"
            if running and samples and not latest["available"]:
                state = "error"
            stamp = latest["sample_timestamps_ms"].get("speed")
            return {"state": state,
                    "topic_out": self.get_tool()["topic_out"],
                    "odom_interface": copy.deepcopy(self._odom_interface),
                    "slam_odometry": {**self._odometry.info(), "clock_id": CLOCK_ID,
                                      "synchronization": "bounded_software",
                                      "clock_mapping_budget_ms": UNCERTAINTY_NS // 1_000_000},
                    "available": latest["available"], "samples": samples,
                    "sampling": {"pose_speed_hz": self._hz,
                                 "imu_hz": self._imu_hz,
                                 "length_hz": self._length_hz,
                                 "transport": "persistent_http",
                                 "slam_odometry_transport": "diagnostic_tcp",
                                 "slam_odometry_publish_hz": self._hz},
                    "last_sample_ago_ms": int(time.time() * 1000) - stamp if stamp else None,
                    "errors": latest["errors"]}
        return None
