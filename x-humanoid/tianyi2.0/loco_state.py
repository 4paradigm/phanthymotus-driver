"""Read-only Tianyi locomotion state from the Slamtec chassis REST API.

Sources follow the supplied Slamware OpenAPI 1.1.0 document. Odometry uses
localization/odopose, never the map-localized localization/pose endpoint.
"""

from concurrent.futures import ThreadPoolExecutor
import copy
import json
import math
import threading
import time

from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import String

from common.odom import build_interface, build_sample, mean_twist, resolve_stamp_ms


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
    return {
        "timestamp_ms": max(timestamps.values()) if timestamps else int(time.time() * 1000),
        "sample_timestamps_ms": dict(timestamps),
        "available": position is not None and rpy is not None and speed_available,
        "source": "slamtec",
        "pose_source": "/api/core/slam/v1/localization/odopose",
        "frame_id": "odom",
        "child_frame_id": "base_link",
        "position": position,
        "rpy": rpy,
        "velocity": [motion[0], motion[1], None],  # No vertical speed sensor.
        "yaw_speed": motion[2],
        "motion_status": ("moving" if any(abs(v) > 0.001 for v in motion) else "stationary")
                         if speed_available else "unknown",
        "mode": None,
        "body_height": None,
        "unavailable_fields": ["mode", "body_height"],
        "imu": imu,
        "errors": errors,
    }


def odometry_sample(snapshot):
    """The same poll in motus.odom/1; pose remains in the vendor block.

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
        vendor={**copy.deepcopy(snapshot), **provenance,
                "samples": int(any(value is not None for value in
                                   (velocity[0], velocity[1], snapshot["yaw_speed"])))},
    )


class LocoStatePlugin:
    """Always-on observer; MCP stop leaves polling active, shutdown stops it."""

    def __init__(self, plugin_config, namespace, ros2, slamtec_client):
        self._client = slamtec_client
        self._hz = float(plugin_config.get("hz", 10))
        self._timeout = float(plugin_config.get("request_timeout", 0.5))
        if not math.isfinite(self._hz) or not 0 < self._hz <= 10:
            raise ValueError("loco_state.hz must be in (0, 10]")
        if not math.isfinite(self._timeout) or not 0 < self._timeout <= 5:
            raise ValueError("loco_state.request_timeout must be in (0, 5]")
        self._topic = f"/{namespace}/loco/state"
        self._odom_topic = f"/{namespace}/state/odom"
        self._odom_interface = build_interface(
            provides=["vx", "vy", "wz"], rate_hz=self._hz, pose_drift="none")
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
        self._latest = locomotion_snapshot({}, {})
        self._samples = 0

    def get_tool(self):
        return {
            "name": "loco_state", "type": "sensor", "multiInstance": False,
            "description": (
                "天轶2.0 locomotion state (always active) — odometry position, "
                "measured velocity and chassis IMU; mode/body_height unavailable. "
                f"Target {self._hz:g}Hz (HTTP latency dependent) to {self._topic}, "
                f"and the same reading as motus.odom/1 to {self._odom_topic}"),
            "inputSchema": {"type": "object", "properties": {}},
            "topic_out": [{"topic": self._topic, "format": "data/json"},
                          {"topic": self._odom_topic, "format": "state/odom"}],
        }

    def start(self):
        with self._lifecycle_lock:
            if self._thread is not None and self._thread.is_alive():
                return
            self._stop_event.clear()
            self._thread = threading.Thread(target=self._poll_loop,
                                            name="tianyi-loco-state", daemon=True)
            self._thread.start()

    def stop(self):
        # Shutdown only. Never clear this event while the old worker is alive.
        with self._lifecycle_lock:
            self._stop_event.set()
            if self._thread is not None:
                self._thread.join(timeout=self._timeout + 1)

    def _read(self, method):
        try:
            result = method(timeout=self._timeout)
            if not isinstance(result, dict):
                result = {"error": "invalid_response_type"}
        except Exception as exc:
            result = {"error": str(exc)}
        return result, int(time.time() * 1000)

    def _poll_once(self, pool):
        futures = {key: pool.submit(self._read, method) for key, method in (
            ("pose", self._client.get_odometry_pose),
            ("speed", self._client.get_speed),
            ("imu", self._client.get_imu),
        )}
        readings, timestamps = {}, {}
        for key, future in futures.items():
            readings[key], timestamps[key] = future.result()
        return locomotion_snapshot(readings, timestamps)

    def _poll_loop(self):
        # One bounded group in flight, even when the chassis cannot sustain 10Hz.
        with ThreadPoolExecutor(max_workers=3) as pool:
            while not self._stop_event.is_set():
                started = time.monotonic()
                snapshot = self._poll_once(pool)
                if self._stop_event.is_set():
                    break
                with self._lock:
                    self._latest = snapshot
                    self._samples += 1
                out = String()
                out.data = json.dumps(snapshot, allow_nan=False)
                self._pub.publish(out)
                odom = String()
                odom.data = json.dumps(odometry_sample(snapshot), allow_nan=False)
                self._odom_pub.publish(odom)
                self._stop_event.wait(max(0, 1 / self._hz - (time.monotonic() - started)))

    def dispatch(self, action, args):
        if action == "start":
            self.start()
        if action in ("read", "get"):
            with self._lock:
                return copy.deepcopy(self._latest)
        if action in ("start", "stop", "info"):
            running = (self._thread is not None and self._thread.is_alive()
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
                    "available": latest["available"], "samples": samples,
                    "last_sample_ago_ms": int(time.time() * 1000) - stamp if stamp else None,
                    "errors": latest["errors"]}
        return {"error": f"Unknown action: {action}"}
