"""Read-only, bounded polling of the EDU upper-level state interfaces."""
import copy
import json
import math
import threading
import time


def finite_vector(value, length=None):
    if (not isinstance(value, list) or (length is not None and len(value) != length)
            or any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) for v in value)):
        raise ValueError("invalid telemetry vector")
    return list(value)


def validate_sample(channel, data):
    data = copy.deepcopy(data)
    data.pop("accid", None)  # Private identity is not part of a recording topic.
    if channel == "joint_states":
        names = data.get("names")
        if (not isinstance(names, list) or not 1 <= len(names) <= 64
                or any(not isinstance(n, str) or not n for n in names)
                or len(set(names)) != len(names)):
            raise ValueError("invalid joint names")
        for key in ("q", "dq", "tau"):
            finite_vector(data.get(key), len(names))
    elif channel == "eef_pose":
        for side in ("left", "right"):
            finite_vector(data.get(f"{side}_position"), 3)
            quat = finite_vector(data.get(f"{side}_quat"), 4)
            if abs(sum(v * v for v in quat) - 1) > .01:
                raise ValueError("invalid WXYZ quaternion")
    # Validate optional vendor records without inventing absent fields.
    json.dumps(data, allow_nan=False)
    return data


class TronTelemetry:
    def __init__(self, client, config, profile, *, ros2=None, namespace="tron2"):
        self.client, self.profile, self._ros2 = client, profile, ros2
        self.hz = config.get("telemetry_hz", 5)
        if type(self.hz) not in (int, float) or not math.isfinite(self.hz) or not 1 <= self.hz <= 10:
            raise ValueError("telemetry_hz must be in [1, 10]")
        self.topic = f"/{namespace.strip('/') or 'tron2'}/tron2/state"
        self.queries = {}
        if profile in ("fixed_arms", "mobile_arms"):
            self.queries.update(joint_states="request_get_joint_state", eef_pose="request_get_move_pose")
            if config.get("gripper_state_enabled") is True:
                self.queries["gripper"] = "request_get_limx_2fclaw_state"
        if profile == "mobile_arms" and config.get("mobile_state_enabled") is True:
            self.queries.update(lifter="request_lifter_state", chassis="request_chassis_state")
        self._lock = threading.RLock()
        self._lifecycle = threading.Lock()
        self._stop = threading.Event()
        self._thread = None
        self._samples = {}
        self._errors = {}
        self._node = None
        self._publish_error = None

    def topic_out(self):
        return [{"topic": self.topic, "format": "data/json"}]

    def start(self):
        with self._lifecycle:
            if self._thread and self._thread.is_alive():
                return
            if self._ros2 is not None and self._node is None:
                from rclpy.node import Node
                from rclpy.qos import qos_profile_sensor_data
                from std_msgs.msg import String
                self._message_type = String
                self._node = Node("tron2_telemetry", context=self._ros2.ctx_core)
                self._publisher = self._node.create_publisher(String, self.topic, qos_profile_sensor_data)
                self._node.create_timer(1 / self.hz, self.publish)
                self._ros2.executor_core.add_node(self._node)
            self._stop.clear()
            self._thread = threading.Thread(target=self._loop, name="tron2-telemetry", daemon=True)
            self._thread.start()

    def stop(self):
        with self._lifecycle:
            self._stop.set()
            if self._thread:
                self._thread.join(timeout=self.client.timeout + 1)
                if self._thread.is_alive():
                    raise RuntimeError("telemetry query is still stopping")
            with self._lock:
                if self._node is not None:
                    self._ros2.executor_core.remove_node(self._node)
                    self._node.destroy_node()
                    self._node = None
                self._samples.clear()
                self._errors.clear()

    def _loop(self):
        while not self._stop.is_set():
            started = time.monotonic()
            self.poll_once()
            self._stop.wait(max(0, 1 / self.hz - (time.monotonic() - started)))

    def poll_once(self):
        if not self.client.connected:
            return
        # A single worker bounds in-flight requests to one; slow responses never
        # queue a growing backlog. No motion lock is held during a query.
        for channel, title in self.queries.items():
            if self._stop.is_set():
                break
            try:
                sample = self.client.request_sample(title)
                sample["data"] = validate_sample(channel, sample["data"])
                with self._lock:
                    self._samples[channel] = sample
                    self._errors.pop(channel, None)
            except Exception:
                with self._lock:
                    self._errors[channel] = "query failed or invalid source sample"

    def snapshot(self):
        with self._lock:
            samples, errors = copy.deepcopy(self._samples), dict(self._errors)
        for key, title in (("robot_info", "notify_robot_info"), ("imu", "notify_imu")):
            try:
                sample = self.client.notification_sample(title)
                sample["data"] = validate_sample(key, sample["data"])
                samples[key] = sample
            except Exception:
                samples[key] = None
        now = time.monotonic_ns()
        channels = {}
        for key in ["robot_info", "imu", *self.queries]:
            sample = samples.get(key)
            age = (now - sample["received_monotonic_ns"]) / 1e6 if sample else None
            ttl = 2500 if key == "robot_info" else 1500
            fresh = bool(self.client.connected and sample and sample["session_id"] == self.client.session_id
                         and age <= ttl and key not in errors)
            channels[key] = {"fresh": fresh, "age_ms": age, "error": errors.get(key),
                             "sample": sample if fresh else None}
        return {"format": "data/json", "timestamp_ms": time.time_ns() // 1_000_000,
                "profile": self.profile, "connected": self.client.connected,
                "channels": channels,
                "last_velocity_command": self.client.command_sample(),
                "units": {"q": "rad", "dq": "rad/s", "tau": "Nm", "eef_position": "m",
                          "eef_quaternion": "wxyz"},
                "poll_hz_limit": self.hz}

    def publish(self):
        with self._lock:
            if self._node is None or self._stop.is_set():
                return
            try:
                message = self._message_type()
                message.data = json.dumps(self.snapshot(), allow_nan=False)
                self._publisher.publish(message)
                self._publish_error = None
            except Exception:
                self._publish_error = "telemetry publication failed"

    def info(self):
        with self._lock:
            return {"polling": bool(self._thread and self._thread.is_alive() and not self._stop.is_set()),
                    "publishing": self._node is not None and not self._stop.is_set(),
                    "topic_out": self.topic_out(), "poll_hz_limit": self.hz,
                    "channels": ["robot_info", "imu", *self.queries],
                    "publish_error": self._publish_error}
