"""Panther ROS 2 device adapter.

All robot-facing endpoints live on CycloneDDS/domain 0.  Sensor payloads are
normalized and handed to ``core_bridge.BridgeWorker``; no robot command topic
is forwarded into Agent Core's DDS domain.
"""

from __future__ import annotations

from datetime import datetime
import hashlib
import json
import math
import os
import ssl
import threading
import time
import urllib.request
from pathlib import Path
from uuid import uuid4

from contracts import (
    ARM_JOINT_FIELDS,
    ARM_MIRROR_SIGNS,
    HEAD_JOINTS,
    LEFT_ARM_JOINTS,
    RIGHT_ARM_JOINTS,
    age_ms,
    byte_value,
    degrees_vector_to_radians,
    finite_number,
    fresh,
    head_target_from_degrees,
    intersect_joint_limits,
    json_float,
    parse_joint_indices,
    parse_joint_limits,
    populate_switch_request,
    validate_drive,
    validate_joint_target,
)
from common.odom import build_interface, build_sample, mean_twist, resolve_stamp_ms
from core_bridge import BridgeWorker, output_topics
from joint_display import dedicated_row, display_gripper_row, joint_row, render_model
from laser_cloud import current_points, point_packet


MODEL_PATH = Path(__file__).with_name("resource") / "panther.urdf"


def _notify_completion(action_id, status: str, result: dict, tool: str) -> None:
    """Report a long-running card result without blocking the motion worker."""
    if not action_id:
        return
    payload = json.dumps(
        {"action_id": action_id, "status": status, "result": result,
         "tool": tool, "ts": time.time()},
        ensure_ascii=False, allow_nan=False,
    ).encode()
    endpoint = os.environ.get("AGENT_CORE_URL", "https://localhost:15678").rstrip("/")

    def deliver():
        try:
            request = urllib.request.Request(
                f"{endpoint}/api/acp/complete",
                data=payload,
                headers={"Content-Type": "application/json"},
                method="POST",
            )
            ca_file = os.environ.get("AGENT_CORE_CA_CERT") or None
            context = ssl.create_default_context(cafile=ca_file)
            context.check_hostname = False
            if ca_file is None:
                context.verify_mode = ssl.CERT_NONE
            error = None
            for attempt in range(1, 4):
                try:
                    urllib.request.urlopen(request, timeout=3, context=context).close()
                    return
                except Exception as exc:
                    error = exc
                    if attempt < 3:
                        time.sleep(0.2 * attempt)
            raise RuntimeError(f"callback failed after 3 attempts: {error}")
        except Exception as exc:
            print(f"[completion] {tool} action_id={action_id} failed: {exc}", flush=True)

    threading.Thread(target=deliver, daemon=True, name=f"panther_acp_{tool}").start()


def _message_dict(value):
    if value is None or isinstance(value, (str, int, bool)):
        return value
    if isinstance(value, float):
        return json_float(value)
    if isinstance(value, (bytes, bytearray)):
        return list(value)
    if isinstance(value, (list, tuple)):
        return [_message_dict(item) for item in value]
    fields = getattr(value, "get_fields_and_field_types", None)
    if callable(fields):
        return {name: _message_dict(getattr(value, name)) for name in fields()}
    return str(value)


class PantherDevice:
    def __init__(self, config: dict, namespace: str, executor):
        import rclpy
        from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
        from geometry_msgs.msg import Twist
        from nav_msgs.msg import Odometry
        from rclpy.action import ActionClient
        from rclpy.node import Node
        from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
        from sensor_msgs.msg import BatteryState, Imu, JointState, LaserScan
        from std_msgs.msg import Float64MultiArray, String
        from control_msgs.action import FollowJointTrajectory
        from controller_manager_msgs.srv import ListControllers, SwitchController
        from unix_msgs.msg import GripperCommand
        from camera import PantherCameraRuntime

        self._rclpy = rclpy
        self._cfg = config
        self.namespace = namespace
        self._lock = threading.RLock()
        self._cache: dict[str, tuple[object, float]] = {}
        self._last_card_publish: dict[str, float] = {}
        self._callback_errors: dict[str, dict] = {}
        self._laser_lock = threading.Lock()
        laser = config.get("plugins", {}).get("laser_scan", {})
        self._laser_topics = tuple(laser.get("topics", ("/scan", "/lidar_front/scan")))
        self._laser_max_points = int(laser.get("max_points", 20000))
        self._laser_max_range_m = float(laser.get("max_range_m", 12.0))
        if not (1 <= self._laser_max_points <= 40000 and 0 < self._laser_max_range_m <= 100):
            raise ValueError("invalid laser_scan cloud limits")
        self._laser_source = None
        self._laser_seen = {}
        self._laser_cloud_info = {}
        self._model_urdf = MODEL_PATH.read_text(encoding="utf-8")
        self._model_joint_limits = parse_joint_limits(self._model_urdf)
        self._urdf = ""
        self._joint_limits: dict[str, tuple[float, float]] = {}
        # The live control URDF can have fixed lift and finger joints. Keep the
        # articulated display model separate from live motion validation.
        self._render_urdf, self._render_mimics = render_model(self._model_urdf)
        self._render_joint_indices = parse_joint_indices(self._render_urdf)
        self._render_source = "bundled articulated display model"
        self._render_model_sha256 = hashlib.sha256(self._render_urdf.encode()).hexdigest()
        self._odom_window: list[list] = []
        self._odom_vendor_ms = None
        self._odom_vendor_meta: dict = {}
        self._drive_lock = threading.Lock()
        self._drive_generation = 0
        self._drive_action_id = None
        self._drive_topic = None
        self._drive_pub = None
        self._drive_stop_request = None
        self._drive_stop_thread = None
        self._controller_query_lock = threading.Lock()
        self._arm_lock = threading.RLock()
        self._arm_active = None
        self._head_lock = threading.RLock()
        self._head_active = None
        self._closed = False
        self._node = Node("panther_motus_driver")
        executor.add_node(self._node)
        self._executor = executor
        self._bridge = BridgeWorker(namespace)
        self._topics = output_topics(namespace)

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        latched_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        def guarded(card, callback):
            return lambda *args: self._safe_callback(card, callback, *args)

        self._node.create_subscription(
            JointState, "/joint_states", guarded("joints", self._on_joints), sensor_qos
        )
        self._node.create_subscription(
            BatteryState,
            "/battery_state_broadcaster/battery_state",
            guarded("battery", self._on_battery),
            sensor_qos,
        )
        self._node.create_subscription(
            Imu,
            "/imu_sensor_broadcaster/imu",
            guarded("imu", lambda msg: self._store_and_publish("imu", msg, self._imu_payload)),
            sensor_qos,
        )
        self._node.create_subscription(Odometry, "/odom", guarded("odom", self._on_odom), sensor_qos)
        self._node.create_timer(0.1, guarded("odom", self._publish_odom_window))
        self._node.create_subscription(
            DiagnosticArray, "/diagnostics", guarded("robot_state", self._on_diagnostics), sensor_qos
        )
        self._node.create_subscription(
            DiagnosticArray, "/hardware_states", guarded("robot_state", self._on_hardware_states), sensor_qos
        )
        self._node.create_subscription(
            DiagnosticStatus, "/hardware_events", guarded("robot_state", self._on_hardware_event), reliable_qos
        )
        self._node.create_subscription(
            JointState, "/left_gripper/joint_states",
            guarded("robot_state", lambda msg: self._on_gripper("left", msg)), sensor_qos
        )
        self._node.create_subscription(
            JointState, "/right_gripper/joint_states",
            guarded("robot_state", lambda msg: self._on_gripper("right", msg)), sensor_qos
        )
        self._node.create_subscription(
            JointState,
            "/lifting_controller/joint_states",
            guarded("robot_state", self._on_lifting),
            sensor_qos,
        )
        self._node.create_subscription(
            String, "/robot_description", guarded("model", self._on_robot_description), latched_qos
        )
        if laser.get("enabled", False):
            for topic in self._laser_topics:
                self._node.create_subscription(
                    LaserScan, topic,
                    guarded("laser_scan", lambda msg, topic=topic: self._on_laser(msg, topic)),
                    sensor_qos,
                )

        self._twist_type = Twist
        self._drive_qos = reliable_qos
        self._head_pub = self._node.create_publisher(
            Float64MultiArray, "/head_forward_position_controller/commands", reliable_qos
        )
        self._lifting_pub = self._node.create_publisher(
            JointState, "/lifting_controller/commands", reliable_qos
        )
        self._left_gripper_pub = self._node.create_publisher(
            GripperCommand, "/left_gripper/gripper_command", reliable_qos
        )
        self._right_gripper_pub = self._node.create_publisher(
            GripperCommand, "/right_gripper/gripper_command", reliable_qos
        )

        self._list_controllers = self._node.create_client(
            ListControllers, "/controller_manager/list_controllers"
        )
        self._switch_controllers = self._node.create_client(
            SwitchController, "/controller_manager/switch_controller"
        )
        self._arm_actions = {
            "left": ActionClient(
                self._node,
                FollowJointTrajectory,
                "/left_arm_joint_trajectory_position_controller/follow_joint_trajectory",
            ),
            "right": ActionClient(
                self._node,
                FollowJointTrajectory,
                "/right_arm_joint_trajectory_position_controller/follow_joint_trajectory",
            ),
        }
        self._controller_poll_future = None
        self._node.create_timer(1.0, guarded("robot_state", self._poll_controllers))
        self._camera = PantherCameraRuntime(
            self._node, self._bridge, config.get("plugins", {}).get("camera", {}), sensor_qos
        )

    @property
    def topics(self):
        return self._topics

    def schema_joint_limits_deg(self) -> dict[str, tuple[float, float]]:
        """Expose bundled limits for registration-time UI hints only."""
        return {
            name: (math.degrees(lower), math.degrees(upper))
            for name, (lower, upper) in self._model_joint_limits.items()
        }

    def start(self) -> None:
        self._bridge.start()

    def close(self) -> None:
        if self._closed:
            return
        base_cfg = self._cfg.get("plugins", {}).get("base_drive", {})
        stop_wait = (float(base_cfg.get("stop_match_timeout_s", 3.0))
                     + float(base_cfg.get("stop_publish_duration_s", 1.0)))
        self.cancel_drive("driver_shutdown", force=True, wait_s=stop_wait)
        self._closed = True
        self._stop_arm("driver_shutdown", wait_s=20.0)
        self._stop_head("driver_shutdown", wait_s=5.0)
        self._camera.close()
        self._bridge.stop()
        pending_stop = self._wait_for_pending_drive_stop(0.0)
        if pending_stop is not None and not pending_stop.get("stop_confirmed"):
            with self._drive_lock:
                request = self._drive_stop_request
                self._drive_stop_request = None
                if request is not None:
                    request["event"].set()
            if request is not None:
                print(
                    "[base_drive] shutdown stop is unconfirmed; no matched "
                    "subscriber received the zero-velocity sequence",
                    flush=True,
                )
        try:
            self._executor.remove_node(self._node)
        except Exception:
            pass
        self._node.destroy_node()

    def _put(self, key: str, value) -> float:
        received = time.monotonic()
        with self._lock:
            self._cache[key] = (value, received)
        return received

    def _get(self, key: str):
        with self._lock:
            return self._cache.get(key, (None, None))

    def _safe_callback(self, card: str, callback, *args) -> None:
        """Keep one malformed ROS message from stopping every Panther card."""
        try:
            callback(*args)
        except Exception as exc:
            now = int(time.time() * 1000)
            with self._lock:
                record = self._callback_errors.setdefault(
                    card, {"count": 0, "last_error": None, "last_error_at_ms": None}
                )
                record["count"] += 1
                record["last_error"] = f"{type(exc).__name__}: {exc}"
                record["last_error_at_ms"] = now
                count = record["count"]
            if count == 1 or count % 100 == 0:
                print(
                    f"[{card}] ROS callback failed #{count}: {type(exc).__name__}: {exc}",
                    flush=True,
                )

    def _callback_health(self, card: str) -> dict:
        with self._lock:
            record = dict(self._callback_errors.get(card, {}))
        return {
            "callback_errors": int(record.get("count", 0)),
            "last_callback_error": record.get("last_error"),
            "last_callback_error_at_ms": record.get("last_error_at_ms"),
        }

    def _store_and_publish(self, card: str, msg, builder) -> None:
        received = self._put(card, msg)
        rates = {"imu": 10.0, "laser_scan": 5.0}
        self._publish_limited(card, builder(msg, received), rates.get(card, 10.0))

    def _publish_limited(self, card: str, payload: dict, rate_hz: float, force: bool = False) -> bool:
        now = time.monotonic()
        with self._lock:
            previous = self._last_card_publish.get(card, 0.0)
            if not force and now - previous < 1.0 / rate_hz:
                return False
            self._last_card_publish[card] = now
        self._bridge.publish(card, payload)
        return True

    def _on_robot_description(self, msg) -> None:
        try:
            limits = parse_joint_limits(msg.data)
            indices = parse_joint_indices(msg.data)
        except Exception as exc:
            print(f"[model] rejected robot_description: {exc}", flush=True)
            return
        with self._lock:
            self._urdf = msg.data
            self._joint_limits = limits
        changed = sorted(
            name for name in set(self._model_joint_limits) | set(limits)
            if self._model_joint_limits.get(name) != limits.get(name)
        )
        print(
            f"[model] loaded live URDF with {len(indices)} kinematic joints "
            f"and {len(limits)} joint limits"
            + (f"; limits differ from bundled model: {', '.join(changed)}" if changed else ""),
            flush=True,
        )

    def _on_joints(self, msg) -> None:
        received = self._put("joints", msg)
        self._publish_limited("joints", self._joints_payload(msg, received), 10.0)

    def _on_diagnostics(self, msg) -> None:
        self._put("diagnostics", msg)
        self._publish_robot_state(force=True)

    def _on_hardware_states(self, msg) -> None:
        self._put("hardware_states", msg)
        self._publish_robot_state()

    def _on_hardware_event(self, msg) -> None:
        self._put("hardware_event", msg)
        self._publish_robot_state(force=True)

    def _on_battery(self, msg) -> None:
        received = self._put("battery", msg)
        self._publish_limited(
            "battery",
            {"available": True, **self._battery_payload(msg, received)},
            2.0,
        )

    def _on_gripper(self, side: str, msg) -> None:
        self._put(f"gripper_{side}", msg)
        self._publish_cached_joints()
        self._publish_robot_state()

    def _on_lifting(self, msg) -> None:
        self._put("lifting", msg)
        self._publish_cached_joints()
        self._publish_robot_state()

    def _publish_cached_joints(self):
        msg, received = self._get("joints")
        if msg is not None and fresh(received):
            self._publish_limited("joints", self._joints_payload(msg, received), 10.0)

    def _on_odom(self, msg) -> None:
        self._put("odom", msg)
        twist = msg.twist.twist
        header = msg.header.stamp
        vendor_ms = int(header.sec) * 1000 + int(header.nanosec) // 1_000_000
        with self._lock:
            self._odom_window.append([
                json_float(twist.linear.x), json_float(twist.linear.y), None,
                None, None, json_float(twist.angular.z),
            ])
            self._odom_vendor_ms = vendor_ms
            self._odom_vendor_meta = {
                "frame_id": msg.header.frame_id,
                "child_frame_id": msg.child_frame_id,
            }

    def _on_laser(self, msg, topic) -> None:
        with self._laser_lock:
            now = time.monotonic()
            self._laser_seen[topic] = now
            preferred = next(t for t in self._laser_topics if now - self._laser_seen.get(t, -math.inf) < 1.0)
            if topic != preferred:
                return
            changed_source = topic != self._laser_source
            self._laser_source = topic
            received = self._put("laser_scan", msg)
            self._publish_limited("laser_scan", self._laser_payload(msg, received, topic), 5.0,
                                  force=changed_source)
            if not changed_source and now - self._last_card_publish.get("laser_cloud", -math.inf) < 1 / 15:
                return
            points = current_points(msg, self._laser_max_range_m, self._laser_max_points)
            self._laser_cloud_info = {
                "cloud_frame": msg.header.frame_id, "cloud_points": len(points),
            }
            self._publish_limited("laser_cloud", point_packet(points), 15.0, force=True)

    def _publish_odom_window(self) -> None:
        with self._lock:
            readings = self._odom_window
            self._odom_window = []
            vendor_ms = self._odom_vendor_ms
            vendor = dict(self._odom_vendor_meta)
        if not readings:
            return
        received_ms = int(time.time() * 1000)
        stamp, stamp_meta = resolve_stamp_ms(vendor_ms=vendor_ms, received_ms=received_ms)
        sample = build_sample(
            stamp_ms=stamp,
            twist=mean_twist(readings),
            frame="body",
            vendor={**vendor, **stamp_meta, "source_topic": "/odom"},
        )
        self._put("odom_sample", sample)
        self._bridge.publish("odom", sample)

    def _publish_robot_state(self, force: bool = False) -> None:
        self._publish_limited("robot_state", self.robot_state(), 2.0, force=force)

    def _joints_payload(self, msg, received) -> dict:
        with self._lock:
            joint_indices = dict(self._render_joint_indices)
            model_source = self._render_source
            model_sha256 = self._render_model_sha256
            dedicated = {key: self._cache.get(key, (None, None))
                         for key in ("lifting", "gripper_left", "gripper_right")}
        rows = {}
        sources = {}
        dropped = 0
        for index, name in enumerate(msg.name):
            if name not in joint_indices:
                continue
            row = joint_row(msg, index)
            if row is None:
                dropped += 1
            else:
                rows[name] = row
        for key, name in (("lifting", "body_joint"), ("gripper_left", "joint_lf"),
                          ("gripper_right", "joint_rf")):
            feedback, at = dedicated[key]
            if feedback is not None and fresh(at):
                row = dedicated_row(feedback, name, gripper=key.startswith("gripper_"))
                if row is not None:
                    rows[name] = row
                    sources[name] = key
        # Canvas consumes one value per animated joint; resolve the bundled
        # gripper linkage even when only its master channel is published.
        for name in ("joint_lf", "joint_rf"):
            if name in rows:
                rows[name] = display_gripper_row(rows[name])
        for name, (master, multiplier, offset) in self._render_mimics.items():
            if master in rows:
                rows[name] = {"q": rows[master]["q"] * multiplier + offset}
        joints = [{"idx": joint_indices[name], "name": name, **row}
                  for name, row in rows.items() if name in joint_indices]
        payload = {
            "timestamp_ms": int(time.time() * 1000),
            "fresh": fresh(received),
            "available": True,
            "age_ms": age_ms(received),
            "format": "sensor/skeleton",
            "position_unit": "rad",
            "position_unit_by_joint": {"body_joint": "m"},
            "source_topic": "/joint_states",
            "feedback_overrides": sources,
            "missing_display_joints": [name for name in (*HEAD_JOINTS, "body_joint", "joint_lf", "joint_rf")
                                       if name not in rows],
            "model_source": model_source,
            "model_sha256": model_sha256,
            "joint_count": len(joints),
            "dropped": dropped,
            "joints": joints,
        }
        return payload

    @staticmethod
    def _battery_payload(msg, received) -> dict:
        percentage = json_float(msg.percentage)
        if percentage is not None and 0.0 <= percentage <= 1.0:
            percentage *= 100.0
        return {
            "timestamp_ms": int(time.time() * 1000),
            "fresh": fresh(received),
            "age_ms": age_ms(received),
            "source_topic": "/battery_state_broadcaster/battery_state",
            "percentage": percentage,
            "voltage_v": json_float(msg.voltage),
            "current_a": json_float(msg.current),
            "temperature_c": json_float(msg.temperature),
            "power_supply_status": byte_value(msg.power_supply_status, "power_supply_status"),
            "present": bool(msg.present),
        }

    @staticmethod
    def _imu_payload(msg, received) -> dict:
        def known(covariance) -> bool:
            try:
                if covariance is None or len(covariance) == 0:
                    return True
                first = float(covariance[0])
                return math.isfinite(first) and first != -1.0
            except (TypeError, ValueError, IndexError):
                return False

        orientation = None
        euler = None
        if known(msg.orientation_covariance):
            raw = [json_float(msg.orientation.x), json_float(msg.orientation.y),
                   json_float(msg.orientation.z), json_float(msg.orientation.w)]
            if all(value is not None for value in raw):
                norm = math.sqrt(sum(value * value for value in raw))
                if norm > 0.5:
                    x, y, z, w = [value / norm for value in raw]
                    orientation = [x, y, z, w]
                    euler = [
                        math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y)),
                        math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x)))),
                        math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)),
                    ]
        angular = ([json_float(msg.angular_velocity.x), json_float(msg.angular_velocity.y),
                    json_float(msg.angular_velocity.z)]
                   if known(msg.angular_velocity_covariance) else None)
        acceleration = ([json_float(msg.linear_acceleration.x), json_float(msg.linear_acceleration.y),
                         json_float(msg.linear_acceleration.z)]
                        if known(msg.linear_acceleration_covariance) else None)
        return {
            "timestamp_ms": int(time.time() * 1000),
            "fresh": fresh(received),
            "age_ms": age_ms(received),
            "source_topic": "/imu_sensor_broadcaster/imu",
            "frame_id": msg.header.frame_id,
            "orientation_xyzw": orientation,
            "euler_rpy_rad": euler,
            "angular_velocity_rad_s": angular,
            "linear_acceleration_m_s2": acceleration,
        }

    @staticmethod
    def _laser_payload(msg, received, topic="/scan") -> dict:
        raw = list(msg.ranges)
        bins = min(360, max(1, len(raw)))
        stride = max(1, math.ceil(len(raw) / bins))
        ranges = []
        for start in range(0, len(raw), stride):
            valid = [float(value) for value in raw[start:start + stride]
                     if json_float(value) is not None and msg.range_min <= float(value) <= msg.range_max]
            ranges.append(min(valid) if valid else None)
        finite = [value for value in ranges if value is not None]
        return {
            "timestamp_ms": int(time.time() * 1000),
            "fresh": fresh(received),
            "age_ms": age_ms(received),
            "source_topic": topic,
            "merged": topic == "/scan",
            "frame_id": msg.header.frame_id,
            "angle_min": json_float(msg.angle_min),
            "angle_max": json_float(msg.angle_max),
            "angle_increment": json_float(msg.angle_increment * stride),
            "range_min": json_float(msg.range_min),
            "range_max": json_float(msg.range_max),
            "source_beams": len(raw),
            "bins": len(ranges),
            "valid_ratio": round(len(finite) / len(ranges), 3) if ranges else 0.0,
            "nearest_m": min(finite) if finite else None,
            "ranges": ranges,
        }

    @staticmethod
    def _lifting_payload(msg, received) -> dict:
        return {
            "timestamp_ms": int(time.time() * 1000),
            "fresh": fresh(received),
            "age_ms": age_ms(received),
            "name": list(msg.name),
            "position": [json_float(v) for v in msg.position],
            "velocity": [json_float(v) for v in msg.velocity],
            "effort": [json_float(v) for v in msg.effort],
            "source_topic": "/lifting_controller/joint_states",
        }

    def _lifting_limits(self):
        """Use configured SDK limits; the live URDF may encode lift as fixed."""
        configured = self._cfg["plugins"]["lifting"]
        configured_min = float(configured.get("min_position_m", 0.0))
        configured_max = float(configured["max_position_m"])
        feedback, _ = self._get("lifting")
        joint_name = next((name for name in getattr(feedback, "name", []) if name), None)
        return configured_min, configured_max, "Panther SDK configured limit", joint_name

    def sensor_info(self, card: str) -> dict:
        if card == "robot_state":
            return self.robot_state()
        if card == "model":
            with self._lock:
                render_urdf = self._render_urdf
                render_source = self._render_source
                render_sha256 = self._render_model_sha256
                render_joint_count = len(self._render_joint_indices)
                live_limits = dict(self._joint_limits)
                live_available = bool(self._urdf)
            different = sorted(
                name for name in set(self._model_joint_limits) | set(live_limits)
                if self._model_joint_limits.get(name) != live_limits.get(name)
            ) if live_limits else []
            return {"model": "UniX AI Panther", "urdf": render_urdf,
                    "available": True, "source": render_source,
                    "model_sha256": render_sha256,
                    "render_joint_count": render_joint_count,
                    "live_robot_description_available": live_available,
                    "live_limit_differences": different,
                    **self._callback_health("model")}
        if card == "odom":
            sample, received = self._get("odom_sample")
            return {
                "available": sample is not None,
                "fresh": fresh(received),
                "age_ms": age_ms(received),
                "data": sample,
                "odom_interface": build_interface(
                    provides=["vx", "vy", "wz"], frame="body", rate_hz=10.0,
                    pose_drift="none",
                ),
                **self._callback_health("odom"),
            }
        if card == "laser_scan":
            with self._laser_lock:
                msg, received = self._get(card)
                data = self._laser_payload(msg, received, self._laser_source) if msg is not None else {
                    "available": False, "fresh": False,
                }
                return {"available": msg is not None, **data, **self._laser_cloud_info,
                        **self._callback_health(card)}
        msg, received = self._get(card)
        if msg is None:
            return {"available": False, "fresh": False, "source": card,
                    **self._callback_health(card)}
        builders = {
            "joints": self._joints_payload,
            "battery": self._battery_payload,
            "imu": self._imu_payload,
        }
        return {"available": True, **builders[card](msg, received),
                **self._callback_health(card)}

    def robot_state(self) -> dict:
        diagnostics, diag_at = self._get("diagnostics")
        hardware, hardware_at = self._get("hardware_states")
        event, event_at = self._get("hardware_event")
        joints, joints_at = self._get("joints")
        controllers, controllers_at = self._get("controllers")
        reasons = []
        if not fresh(joints_at):
            reasons.append("no fresh /joint_states")
        if not fresh(hardware_at, 5.0):
            reasons.append("no fresh /hardware_states")
        if not fresh(controllers_at, 3.0):
            reasons.append("no fresh controller_manager response")
        diagnostic_summary = None
        if diagnostics is not None:
            statuses = list(getattr(diagnostics, "status", ()))
            levels = [(item, byte_value(item.level, f"diagnostics[{index}].level"))
                      for index, item in enumerate(statuses)]
            problems = [f"{item.name}: {item.message}" for item, level in levels if level != 0]
            worst = max((level for _item, level in levels), default=0)
            diagnostic_summary = {
                "count": len(statuses), "worst_level": worst,
                "problems": sorted(problems),
            }
        state_available = any(item is not None for item in (diagnostics, hardware, joints))
        newest = max((stamp for stamp in (diag_at, hardware_at, joints_at)
                      if stamp is not None), default=None)
        available = state_available and controllers is not None
        state_fresh = fresh(newest, 5.0)
        return {
            "timestamp_ms": int(time.time() * 1000),
            "available": available,
            "fresh": state_fresh and fresh(controllers_at, 3.0),
            "age_ms": age_ms(newest),
            "ready_for_motion": not reasons,
            "not_ready_reasons": reasons,
            "joint_state": {"available": joints is not None, "fresh": fresh(joints_at), "age_ms": age_ms(joints_at)},
            "controllers": {
                "available": controllers is not None,
                "fresh": fresh(controllers_at, 3.0),
                "age_ms": age_ms(controllers_at),
                "states": controllers or {},
                "last_error": self._get("controller_error")[0],
            },
            "diagnostics": {"available": diagnostics is not None, "fresh": fresh(diag_at, 5.0),
                            "age_ms": age_ms(diag_at), "summary": diagnostic_summary},
            "hardware_states": {"available": hardware is not None, "fresh": fresh(hardware_at, 5.0),
                                "age_ms": age_ms(hardware_at)},
            "last_hardware_event": {"available": event is not None, "age_ms": age_ms(event_at),
                                    "name": getattr(event, "name", None),
                                    "level": (byte_value(event.level, "hardware_event.level")
                                              if event is not None else None),
                                    "message": getattr(event, "message", None)},
            "estop": {"reported": False, "message": "No ROS emergency-stop state was identified"},
            **self._callback_health("robot_state"),
        }

    def lifting_state(self) -> dict:
        message, received = self._get("lifting")
        if message is None:
            return {"available": False, "fresh": False, "age_ms": None,
                    "source_topic": "/lifting_controller/joint_states"}
        return {"available": True, **self._lifting_payload(message, received)}

    def gripper_state(self) -> dict:
        out = {"timestamp_ms": int(time.time() * 1000), "source_topics": {}}
        for side in ("left", "right"):
            msg, received = self._get(f"gripper_{side}")
            out[side] = {
                "available": msg is not None,
                "fresh": fresh(received),
                "age_ms": age_ms(received),
                "data": _message_dict(msg),
            }
            out["source_topics"][side] = f"/{side}_gripper/joint_states"
        return out

    def _ensure_motion_ready(self) -> None:
        status = self.robot_state()
        if not status["ready_for_motion"]:
            raise RuntimeError("motion rejected: fresh joint and hardware state are required")

    def _wait_future(self, future, timeout_s: float):
        event = threading.Event()
        future.add_done_callback(lambda _: event.set())
        if not event.wait(timeout_s):
            raise TimeoutError(f"ROS operation timed out after {timeout_s:g}s")
        exception = future.exception()
        if exception is not None:
            raise exception
        return future.result()

    def _poll_controllers(self) -> None:
        from controller_manager_msgs.srv import ListControllers

        if self._controller_poll_future is not None or not self._list_controllers.service_is_ready():
            return
        future = self._list_controllers.call_async(ListControllers.Request())
        self._controller_poll_future = future

        def completed(done):
            self._controller_poll_future = None
            try:
                result = done.result()
                states = {entry.name: entry.state for entry in result.controller}
                self._put("controllers", states)
                self._put("controller_error", None)
                self._publish_robot_state()
            except Exception as exc:
                self._put("controller_error", f"{type(exc).__name__}: {exc}")
                print(f"[robot_state] controller poll failed: {exc}", flush=True)

        future.add_done_callback(completed)

    def controller_states(self, max_cache_age_s: float = 2.0) -> dict[str, str]:
        from controller_manager_msgs.srv import ListControllers
        cached, received = self._get("controllers")
        if cached is not None and fresh(received, max_cache_age_s):
            return dict(cached)
        # Canvas starts cards concurrently. Serialize the fallback query so one
        # controller-manager response populates the cache for every card.
        with self._controller_query_lock:
            cached, received = self._get("controllers")
            if cached is not None and fresh(received, max_cache_age_s):
                return dict(cached)
            if not self._list_controllers.wait_for_service(timeout_sec=2.0):
                raise RuntimeError("/controller_manager/list_controllers is unavailable")
            result = self._wait_future(
                self._list_controllers.call_async(ListControllers.Request()), 3.0
            )
            states = {entry.name: entry.state for entry in result.controller}
            self._put("controllers", states)
            self._put("controller_error", None)
            return states

    @staticmethod
    def _require_states(states: dict[str, str], expected: dict[str, str]) -> None:
        failures = [f"{name}={states.get(name)!r}, expected {state!r}"
                    for name, state in expected.items() if states.get(name) != state]
        if failures:
            raise RuntimeError("controller readiness failed: " + "; ".join(failures))

    @staticmethod
    def _require_controllers(states: dict[str, str], names) -> None:
        missing = [name for name in names if name not in states]
        if missing:
            raise RuntimeError("controllers not reported by controller_manager: " + ", ".join(missing))

    def _require_controller(self, name: str, state: str = "active") -> None:
        actual = self.controller_states().get(name)
        if actual != state:
            raise RuntimeError(f"controller {name} is {actual!r}, expected {state!r}")

    def _drive_candidates(self) -> list[str]:
        config = self._cfg["plugins"]["base_drive"]
        candidates = config.get("cmd_topics") or [config.get("topic", "/cmd_vel")]
        return [str(topic) for topic in candidates if topic]

    def _drive_topic_status(self) -> dict[str, int]:
        return {topic: int(self._node.count_subscribers(topic))
                for topic in self._drive_candidates()}

    def _select_drive_topic(self, timeout_s: float = 3.0):
        deadline = time.monotonic() + timeout_s
        counts = self._drive_topic_status()
        selected = next((topic for topic in self._drive_candidates() if counts.get(topic, 0) > 0), None)
        while selected is None and time.monotonic() < deadline:
            time.sleep(0.1)
            counts = self._drive_topic_status()
            selected = next(
                (topic for topic in self._drive_candidates() if counts.get(topic, 0) > 0),
                None,
            )
        if selected is None:
            raise RuntimeError(f"no Panther base command topic has subscribers: {counts}")
        with self._drive_lock:
            if self._drive_pub is None or self._drive_topic != selected:
                self._drive_pub = self._node.create_publisher(
                    self._twist_type, selected, self._drive_qos
                )
                self._drive_topic = selected
            publisher = self._drive_pub
        match_deadline = time.monotonic() + timeout_s
        while not self._drive_publisher_matched(publisher) and time.monotonic() < match_deadline:
            time.sleep(0.05)
        if not self._drive_publisher_matched(publisher):
            raise RuntimeError(f"Panther base publisher did not match a subscriber on {selected}")
        return selected, self._drive_topic_status()

    @staticmethod
    def _drive_publisher_matched(publisher) -> bool:
        try:
            return publisher is not None and publisher.get_subscription_count() > 0
        except Exception:
            return False

    def _wait_for_pending_drive_stop(self, timeout_s: float) -> dict | None:
        with self._drive_lock:
            request = self._drive_stop_request
        if request is None:
            return None
        request["event"].wait(max(0.0, timeout_s))
        if request["result"] is not None:
            return dict(request["result"])
        return {"state": "stop_unconfirmed", "stop_confirmed": False,
                "topic": self._drive_topic, "reason": request["reason"],
                "action_id": request["action_id"],
                "last_error": request.get("last_error"),
                "message": "waiting for a matched subscriber before sending zero velocity"}

    def _drive_stop_worker(self, request: dict) -> None:
        from geometry_msgs.msg import Twist
        cfg = self._cfg["plugins"]["base_drive"]
        rate = max(1.0, float(cfg.get("publish_rate_hz", 20.0)))
        duration = max(0.1, float(cfg.get("stop_publish_duration_s", 1.0)))
        repetitions = max(1, int(cfg.get("stop_repetitions", 5)))
        period = 1.0 / rate
        sent = 0
        # Shutdown marks the device closed while other actuator workers are
        # still winding down. Keep the zero-velocity path alive until it is
        # confirmed or close() explicitly surfaces and retires it.
        while True:
            with self._drive_lock:
                if self._drive_stop_request is not request:
                    return
                publisher = self._drive_pub
                topic = self._drive_topic
            if not self._drive_publisher_matched(publisher):
                if request.get("publish_while_unmatched") and publisher is not None:
                    try:
                        publisher.publish(Twist())
                        request["unmatched_zero_attempts"] += 1
                    except Exception as exc:
                        request["last_error"] = f"{type(exc).__name__}: {exc}"
                time.sleep(0.05)
                continue
            deadline = time.monotonic() + duration
            sent = 0
            while time.monotonic() < deadline or sent < repetitions:
                with self._drive_lock:
                    if self._drive_stop_request is not request:
                        return
                if not self._drive_publisher_matched(publisher):
                    sent = 0
                    break
                try:
                    publisher.publish(Twist())
                except Exception as exc:
                    request["last_error"] = f"{type(exc).__name__}: {exc}"
                    sent = 0
                    break
                sent += 1
                time.sleep(period)
            if sent < repetitions:
                continue
            result = {"state": "idle", "stop_confirmed": True, "topic": topic,
                      "zero_commands_sent": sent, "reason": request["reason"],
                      "action_id": request["action_id"]}
            with self._drive_lock:
                if self._drive_stop_request is not request:
                    return
                request["result"] = result
                self._drive_stop_request = None
                request["event"].set()
            action_id = request["action_id"]
            if action_id:
                status = request["completion_status"]
                completion = dict(request.get("completion_result") or {})
                completion.update({"reason": request["reason"], "stop_confirmed": True,
                                   "zero_commands_sent": sent})
                _notify_completion(action_id, status, completion, "base_drive")
            return

    def drive(self, args: dict) -> dict:
        action = args.get("action", "move")
        if action in ("cancel", "stop"):
            return self.cancel_drive("cancelled_by_request", force=True)
        if action in ("start", "info"):
            states = self.controller_states()
            self._require_states(states, {"movebase3_drive_controller": "active"})
            topic, subscribers = self._select_drive_topic()
            return {"state": "ready", "controllers": states, "topic": topic,
                    "cmd_topics": subscribers}
        if action != "move":
            raise ValueError(f"unsupported base_drive action: {action}")
        self._ensure_motion_ready()
        from geometry_msgs.msg import Twist
        self._require_controller("movebase3_drive_controller")
        topic, subscribers = self._select_drive_topic()
        cfg = self._cfg["plugins"]["base_drive"]
        vx, vy, wz, duration = validate_drive(args, cfg)
        wz_deg_s = math.degrees(wz)
        rate = float(cfg["publish_rate_hz"])
        replaced = self.cancel_drive("replaced_by_new_command")
        if replaced.get("state") not in ("idle", "no_active_motion"):
            raise RuntimeError(
                "previous base stop is not confirmed; refusing a new velocity command"
            )
        action_id = str(args.get("action_id") or f"panther_base_{uuid4().hex[:12]}")
        with self._drive_lock:
            self._drive_generation += 1
            generation = self._drive_generation
            self._drive_action_id = action_id

        def run():
            try:
                period = 1.0 / rate
                deadline = time.monotonic() + duration
                while time.monotonic() < deadline:
                    with self._drive_lock:
                        if generation != self._drive_generation:
                            return
                        publisher = self._drive_pub
                        if not self._drive_publisher_matched(publisher):
                            subscriber_lost = True
                        else:
                            subscriber_lost = False
                        msg = Twist(); msg.linear.x = vx; msg.linear.y = vy; msg.angular.z = wz
                        if not subscriber_lost:
                            publisher.publish(msg)
                    if subscriber_lost:
                        self.cancel_drive(
                            "subscriber_lost", completion_status="error",
                            completion_result={"error": "base command subscriber disappeared"},
                        )
                        return
                    time.sleep(period)
                self.cancel_drive("duration_expired", completion_status="completed",
                                  completion_result={
                    "linear_x": vx, "linear_y": vy, "angular_z_deg_s": wz_deg_s,
                    "duration_s": duration, "stopped_automatically": True,
                })
            except Exception as exc:
                self.cancel_drive(
                    "motion_error", completion_status="error",
                    completion_result={"error": f"{type(exc).__name__}: {exc}"},
                )

        threading.Thread(target=run, daemon=True, name="panther_drive").start()
        return {"state": "queued", "action_id": action_id,
                "linear_x": vx, "linear_y": vy, "angular_z_deg_s": wz_deg_s,
                "duration_s": duration, "topic": topic, "cmd_topics": subscribers,
                "auto_stop": True}

    def cancel_drive(self, reason: str = "cancelled", *, force: bool = False,
                     completion_status: str = "cancelled",
                     completion_result: dict | None = None,
                     wait_s: float | None = None) -> dict:
        cfg = self._cfg["plugins"]["base_drive"]
        default_wait = (float(cfg.get("stop_match_timeout_s", 3.0))
                        + float(cfg.get("stop_publish_duration_s", 1.0)))
        pending = self._wait_for_pending_drive_stop(default_wait if wait_s is None else wait_s)
        if pending is not None:
            return pending
        with self._drive_lock:
            action_id = self._drive_action_id
            publisher = self._drive_pub
            if action_id is None and not force:
                return {"state": "no_active_motion", "stop_confirmed": True,
                        "topic": self._drive_topic, "action_id": None}
            if publisher is None:
                return {"state": "idle" if action_id is None else "stop_unconfirmed",
                        "stop_confirmed": action_id is None, "topic": self._drive_topic,
                        "reason": reason, "action_id": action_id}
            self._drive_action_id = None
            self._drive_generation += 1
            previous = self._drive_stop_request
            if previous is not None:
                previous["event"].set()
            request = {
                "action_id": action_id,
                "reason": reason,
                "completion_status": completion_status,
                "completion_result": completion_result or {},
                "event": threading.Event(),
                "result": None,
                "last_error": None,
                "publish_while_unmatched": reason == "driver_shutdown",
                "unmatched_zero_attempts": 0,
            }
            self._drive_stop_request = request
            thread = threading.Thread(
                target=self._drive_stop_worker, args=(request,), daemon=True,
                name="panther_drive_stop",
            )
            self._drive_stop_thread = thread
            thread.start()
        timeout = default_wait if wait_s is None else max(0.0, wait_s)
        request["event"].wait(timeout)
        if request["result"] is not None:
            return dict(request["result"])
        return {"state": "stop_unconfirmed", "stop_confirmed": False,
                "topic": self._drive_topic, "reason": reason,
                "action_id": action_id,
                "last_error": request.get("last_error"),
                "message": "waiting for a matched subscriber before sending zero velocity"}

    def _switch_controllers_exact(self, activate: list[str], deactivate: list[str]) -> None:
        from controller_manager_msgs.srv import SwitchController
        if not activate and not deactivate:
            return
        if not self._switch_controllers.wait_for_service(timeout_sec=2.0):
            raise RuntimeError("/controller_manager/switch_controller is unavailable")
        request = SwitchController.Request()
        populate_switch_request(request, activate, deactivate)
        response = self._wait_future(self._switch_controllers.call_async(request), 7.0)
        if not response.ok:
            raise RuntimeError("controller manager rejected arm mode switch")

    def arm(self, args: dict) -> dict:
        action = args.get("action", "move")
        if action in ("start", "info"):
            with self._arm_lock:
                active = ({key: self._arm_active[key] for key in
                           ("action_id", "action", "side", "duration_s")}
                          if self._arm_active else None)
            states = self.controller_states()
            self._require_controllers(states, (
                "dual_arm_forward_position_controller",
                "left_arm_joint_trajectory_position_controller",
                "right_arm_joint_trajectory_position_controller",
            ))
            if not fresh(self._get("joints")[1]):
                raise RuntimeError("fresh /joint_states is required for arm_control")
            if not self._joint_limits:
                raise RuntimeError("live /robot_description limits are required for arm_control")
            return {"state": "moving" if active else "ready", "active": active,
                    "controllers": states,
                    "joint_state_fresh": fresh(self._get("joints")[1]),
                    "limits_loaded": bool(self._joint_limits)}
        if action in ("stop", "cancel"):
            return self._stop_arm("cancelled_by_request", wait_s=10.0)
        if action not in ("move", "move_separate", "home"):
            raise ValueError(f"unsupported arm_control action: {action}")
        self._ensure_motion_ready()
        cfg = self._cfg["plugins"]["arm_control"]
        max_duration = float(cfg["max_duration_s"])
        side = "both" if action == "move_separate" else args.get(
            "side", "both" if action == "home" else "left"
        )
        if side not in ("left", "right", "both"):
            raise ValueError("side must be left, right, or both")
        joint_msg, received = self._get("joints")
        if not fresh(received):
            raise RuntimeError("fresh /joint_states is required")
        current = {name: value for name, value in zip(joint_msg.name, joint_msg.position)}
        max_delta = math.radians(float(cfg["max_target_delta_deg"]))
        hold_limit_tolerance = math.radians(float(cfg.get("hold_limit_tolerance_deg", 0.5)))
        targets: dict[str, list[float]] = {}

        def read_joint_value(primary: str, legacy: str):
            primary_value = args.get(primary)
            legacy_value = args.get(legacy)
            if primary_value is not None and legacy_value is not None:
                raise ValueError(f"use {primary} or legacy {legacy}, not both")
            if primary_value is not None:
                return finite_number(primary_value, primary)
            if legacy_value is not None:
                return finite_number(legacy_value, legacy)
            return None

        shared_updates = {}
        for index, field in enumerate(ARM_JOINT_FIELDS):
            value = read_joint_value(field, f"j{index}_deg")
            if value is not None:
                shared_updates[index] = value

        def target_for(arm_side: str, names: tuple[str, ...]) -> list[float]:
            legacy_field = "positions_deg" if side == arm_side else f"{arm_side}_positions_deg"
            legacy = args.get(legacy_field)
            if legacy is not None:
                values = degrees_vector_to_radians(legacy, legacy_field, len(names))
                commanded_names = set(names)
            elif action == "home":
                home = cfg.get("home_pose_deg")
                if not isinstance(home, dict) or not home:
                    raise RuntimeError("arm_control home_pose_deg is not configured; refusing unverified home")
                values = [current.get(name) if home.get(name) is None
                          else math.radians(finite_number(home[name], f"home_pose_deg.{name}"))
                          for name in names]
                commanded_names = {name for name in names if home.get(name) is not None}
            elif action == "move" and shared_updates:
                updates = {
                    name: math.radians(
                        shared_updates[index]
                        * (ARM_MIRROR_SIGNS[index]
                           if side == "both" and arm_side == "right" else 1.0)
                    )
                    for index, name in enumerate(names)
                    if index in shared_updates
                }
                values = [updates.get(name, current.get(name)) for name in names]
                commanded_names = set(updates)
            else:
                prefix = "la" if arm_side == "left" else "ra"
                side_prefix = "l" if arm_side == "left" else "r"
                updates = {}
                for index, (name, field) in enumerate(zip(names, ARM_JOINT_FIELDS)):
                    value = read_joint_value(f"{side_prefix}_{field}", f"{prefix}{index}_deg")
                    if value is not None:
                        updates[name] = math.radians(value)
                if not updates:
                    if action == "move":
                        raise ValueError("at least one arm joint target is required")
                    raise ValueError(f"at least one {arm_side} arm joint target is required")
                values = [updates.get(name, current.get(name)) for name in names]
                commanded_names = set(updates)
            # Home is subject to the same per-command delta gate as every
            # other arm target until the candidate pose is independently
            # commissioned on hardware.
            effective_limits = intersect_joint_limits(
                names, self._model_joint_limits, self._joint_limits
            )
            return validate_joint_target(
                names, values, effective_limits, current, max_delta,
                commanded_joint_names=commanded_names,
                hold_limit_tolerance_rad=hold_limit_tolerance,
            )

        if side in ("left", "both"):
            targets["left"] = target_for("left", LEFT_ARM_JOINTS)
        if side in ("right", "both"):
            targets["right"] = target_for("right", RIGHT_ARM_JOINTS)

        if args.get("duration_s") is None:
            speed = math.radians(float(cfg.get("speed_deg_s", 20.0)))
            distance = max(abs(value - float(current[name]))
                           for arm_side, values in targets.items()
                           for name, value in zip(LEFT_ARM_JOINTS if arm_side == "left" else RIGHT_ARM_JOINTS,
                                                  values))
            duration = min(max_duration, max(float(cfg.get("min_duration_s", 1.0)), distance / speed))
        else:
            duration = finite_number(args.get("duration_s"), "duration_s")
        if not 1.0 <= duration <= max_duration:
            raise ValueError(f"duration_s must be in [1.0, {max_duration}]")

        with self._arm_lock:
            if self._arm_active:
                raise RuntimeError("an arm move is already active; call stop first")
            action_id = str(args.get("action_id") or f"panther_arm_{uuid4().hex[:12]}")
            active = {"action_id": action_id, "action": action, "side": side,
                      "duration_s": duration, "stop": threading.Event(), "handles": {},
                      "thread": None}
            self._arm_active = active
            thread = threading.Thread(
                target=self._run_arm,
                args=(active, targets),
                daemon=True,
                name="panther_arm_motion",
            )
            active["thread"] = thread
            thread.start()
        return {"state": "queued", "action_id": action_id, "action": action,
                "side": side, "duration_s": duration}

    def _run_arm(self, active: dict, targets: dict[str, list[float]]) -> None:
        from builtin_interfaces.msg import Duration
        from control_msgs.action import FollowJointTrajectory
        from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

        action_id = active["action_id"]
        restore_activate: list[str] = []
        restore_deactivate: list[str] = []
        switched = False
        result: dict = {"side": active["side"], "duration_s": active["duration_s"]}
        status = "error"
        try:
            states = self.controller_states()
            forward = "dual_arm_forward_position_controller"
            requested = [f"{side}_arm_joint_trajectory_position_controller" for side in targets]
            activate = [name for name in requested if states.get(name) != "active"]
            deactivate = [forward] if states.get(forward) == "active" else []
            restore_activate = list(deactivate)
            restore_deactivate = list(activate)
            if active["stop"].is_set():
                status = "cancelled"
                return
            self._switch_controllers_exact(activate, deactivate)
            switched = bool(activate or deactivate)
            for arm_side, positions in targets.items():
                if active["stop"].is_set():
                    status = "cancelled"
                    return
                client = self._arm_actions[arm_side]
                if not client.wait_for_server(timeout_sec=3.0):
                    raise RuntimeError(f"{arm_side} FollowJointTrajectory action is unavailable")
                trajectory = JointTrajectory()
                trajectory.joint_names = list(LEFT_ARM_JOINTS if arm_side == "left" else RIGHT_ARM_JOINTS)
                point = JointTrajectoryPoint(); point.positions = positions
                duration = active["duration_s"]
                point.time_from_start = Duration(sec=int(duration),
                                                   nanosec=int((duration % 1.0) * 1_000_000_000))
                trajectory.points = [point]
                goal = FollowJointTrajectory.Goal(); goal.trajectory = trajectory
                handle = self._wait_future(client.send_goal_async(goal), 5.0)
                if not handle.accepted:
                    raise RuntimeError(f"{arm_side} trajectory goal was rejected")
                with self._arm_lock:
                    active["handles"][arm_side] = handle
            results = {}
            for arm_side, handle in list(active["handles"].items()):
                wrapped = self._wait_future(handle.get_result_async(), active["duration_s"] + 8.0)
                results[arm_side] = {"status": int(wrapped.status),
                                     "error_code": int(wrapped.result.error_code),
                                     "error_string": wrapped.result.error_string}
                if wrapped.result.error_code != 0 and not active["stop"].is_set():
                    raise RuntimeError(f"{arm_side} trajectory failed: {wrapped.result.error_string}")
            result["result"] = results
            status = "cancelled" if active["stop"].is_set() else "completed"
        except Exception as exc:
            result["error"] = f"{type(exc).__name__}: {exc}"
            status = "cancelled" if active["stop"].is_set() else "error"
        finally:
            try:
                if switched:
                    self._switch_controllers_exact(restore_activate, restore_deactivate)
                result["controllers_restored"] = True
            except Exception as exc:
                result["controllers_restored"] = False
                result["restore_error"] = f"{type(exc).__name__}: {exc}"
                status = "error"
            with self._arm_lock:
                if self._arm_active is active:
                    self._arm_active = None
            _notify_completion(action_id, status, result, "arm_control")

    def _stop_arm(self, reason: str, wait_s: float) -> dict:
        with self._arm_lock:
            active = self._arm_active
            if not active:
                return {"state": "idle", "stopped": False}
            active["stop"].set()
            handles = list(active["handles"].values())
            thread = active.get("thread")
            action_id = active["action_id"]
        for handle in handles:
            try:
                handle.cancel_goal_async()
            except Exception as exc:
                print(f"[arm_control] cancel failed: {exc}", flush=True)
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=wait_s)
        return {"state": "idle" if not thread or not thread.is_alive() else "stopping",
                "stopped": True, "reason": reason, "action_id": action_id}

    def gripper(self, args: dict) -> dict:
        action = args.get("action", "set")
        if action == "stop":
            return {
                "state": "idle",
                "note": "single position command: nothing is in flight in this driver, and no further command will be sent",
            }
        if action in ("start", "info"):
            states = self.controller_states()
            self._require_states(states, {
                "left_unix_gripper3_controller": "active",
                "right_unix_gripper3_controller": "active",
            })
            feedback = self.gripper_state()
            stale = [side for side in ("left", "right") if not feedback[side]["fresh"]]
            if stale:
                raise RuntimeError("fresh gripper feedback is required for: " + ", ".join(stale))
            return {"state": "ready", "feedback": feedback, "controllers": states}
        if action not in ("set", "open", "close"):
            raise ValueError(f"unsupported gripper action: {action}")
        self._ensure_motion_ready()
        from sensor_msgs.msg import JointState
        from unix_msgs.msg import GripperCommand
        side = args.get("side", "both")
        if side not in ("left", "right", "both"):
            raise ValueError("side must be left, right, or both")
        sides = ("left", "right") if side == "both" else (side,)
        states = self.controller_states()
        unavailable = [name for name in (f"{item}_unix_gripper3_controller" for item in sides)
                       if states.get(name) != "active"]
        if unavailable:
            raise RuntimeError("inactive gripper controllers: " + ", ".join(unavailable))
        cfg = self._cfg["plugins"].get("gripper", {})
        position = (float(cfg.get("open_position", 0.0)) if action == "open" else
                    float(cfg.get("closed_position", 1.0)) if action == "close" else
                    finite_number(args.get("position"), "position"))
        if not 0.0 <= position <= 1.0:
            raise ValueError("position must be in [0, 1]")
        mode = "parallel"
        for item in sides:
            message = GripperCommand(); message.mode = mode
            command = JointState(); command.name = [""]; command.position = [position]
            message.command = command
            (self._left_gripper_pub if item == "left" else self._right_gripper_pub).publish(message)
        return {"state": "commanded", "side": side, "mode": mode, "position": position}

    def lifting(self, args: dict) -> dict:
        action = args.get("action", "set_position")
        if action == "stop":
            return {
                "state": "idle",
                "note": "single position command: nothing is in flight in this driver, and no further command will be sent",
            }
        if action in ("start", "info"):
            states = self.controller_states()
            self._require_states(states, {"lifting_controller": "active"})
            feedback = self.lifting_state()
            if not feedback["fresh"]:
                raise RuntimeError("fresh /lifting_controller/joint_states is required")
            minimum, maximum, limit_source, joint_name = self._lifting_limits()
            return {"state": "ready", "feedback": feedback, "controllers": states,
                    "limits_m": [minimum, maximum], "limit_source": limit_source,
                    "joint_name": joint_name}
        if action not in ("set_position", "home"):
            raise ValueError(f"unsupported lifting action: {action}")
        self._ensure_motion_ready()
        from sensor_msgs.msg import JointState
        self._require_controller("lifting_controller")
        configured = self._cfg["plugins"].get("lifting", {})
        position = (finite_number(configured.get("home_position_m", 0.6), "home_position_m")
                    if action == "home" else
                    finite_number(args.get("position_m"), "position_m"))
        minimum, maximum, limit_source, joint_name = self._lifting_limits()
        if maximum < minimum:
            raise RuntimeError("lifting limits are invalid; refusing motion")
        if not minimum <= position <= maximum:
            raise ValueError(f"position_m must be in [{minimum}, {maximum}]")
        message = JointState()
        message.header.stamp = self._node.get_clock().now().to_msg()
        message.name = [""]; message.position = [position]
        message.velocity = []; message.effort = []
        self._lifting_pub.publish(message)
        return {"state": "commanded", "action": action, "position_m": position,
                "limits_m": [minimum, maximum], "limit_source": limit_source,
                "joint_name": joint_name}

    def head(self, args: dict) -> dict:
        action = args.get("action", "look")
        if action in ("start", "info"):
            with self._head_lock:
                active = ({key: self._head_active[key] for key in
                           ("action_id", "action", "speed", "speed_deg_s")}
                          if self._head_active else None)
            states = self.controller_states()
            self._require_states(states, {"head_forward_position_controller": "active"})
            joint_msg, received = self._get("joints")
            if joint_msg is None or not fresh(received):
                raise RuntimeError("fresh /joint_states is required for head")
            missing = [name for name in HEAD_JOINTS if name not in self._joint_limits]
            if missing:
                raise RuntimeError("live URDF limits are required for: " + ", ".join(missing))
            return {"state": "moving" if active else "ready", "active": active,
                    "limits_loaded": True, "controllers": states}
        if action in ("stop", "cancel"):
            return self._stop_head("cancelled_by_request", wait_s=5.0)
        if action not in ("look", "home"):
            raise ValueError(f"unsupported head action: {action}")
        self._ensure_motion_ready()
        self._require_controller("head_forward_position_controller")
        joint_msg, received = self._get("joints")
        if not fresh(received):
            raise RuntimeError("fresh /joint_states is required")
        current = {name: value for name, value in zip(joint_msg.name, joint_msg.position)}
        cfg = self._cfg["plugins"].get("head", {})
        if action == "home":
            home = cfg.get("home_pose_deg")
            if not isinstance(home, dict) or not {"yaw_deg", "pitch_deg"} <= set(home):
                raise RuntimeError(
                    "head home_pose_deg must define yaw_deg and pitch_deg; refusing unverified home"
                )
            target = [math.radians(finite_number(home["yaw_deg"], "home_pose_deg.yaw_deg")),
                      math.radians(finite_number(home["pitch_deg"], "home_pose_deg.pitch_deg"))]
        else:
            target = head_target_from_degrees(args, current)
        yaw_min = float(cfg.get("yaw_min_deg", -90.0))
        yaw_max = float(cfg.get("yaw_max_deg", 90.0))
        pitch_min = float(cfg.get("pitch_min_deg", -86.0))
        pitch_max = float(cfg.get("pitch_max_deg", 20.0))
        target_deg = [math.degrees(value) if value is not None else None for value in target]
        if target_deg[0] is not None and not yaw_min <= target_deg[0] <= yaw_max:
            raise ValueError(f"yaw_deg must be in [{yaw_min}, {yaw_max}]")
        if target_deg[1] is not None and not pitch_min <= target_deg[1] <= pitch_max:
            raise ValueError(f"pitch_deg must be in [{pitch_min}, {pitch_max}]")
        missing = [name for name in HEAD_JOINTS if name not in self._joint_limits]
        if missing:
            raise RuntimeError("live URDF limits are required for: " + ", ".join(missing))
        for name, value in zip(HEAD_JOINTS, target):
            if value is None:
                raise RuntimeError(f"fresh joint state is missing {name}")
            lower, upper = self._joint_limits[name]
            if not lower <= value <= upper:
                raise ValueError(
                    f"{name} target {math.degrees(value):.2f} deg is outside URDF limit "
                    f"[{math.degrees(lower):.2f}, {math.degrees(upper):.2f}] deg"
                )
        speed_name = "low" if action == "home" else str(args.get("speed", "high"))
        if speed_name not in ("low", "high"):
            raise ValueError("speed must be low or high")
        speed_deg_s = finite_number(cfg.get(f"{speed_name}_speed_deg_s"),
                                    f"{speed_name}_speed_deg_s")
        if speed_deg_s <= 0:
            raise ValueError(f"{speed_name}_speed_deg_s must be positive")
        publish_rate_hz = finite_number(cfg.get("publish_rate_hz", 20.0),
                                        "publish_rate_hz")
        if publish_rate_hz <= 0:
            raise ValueError("publish_rate_hz must be positive")

        self._stop_head("replaced_by_new_command", wait_s=5.0)

        action_id = str(args.get("action_id") or f"panther_head_{uuid4().hex[:12]}")
        active = {"action_id": action_id, "action": action, "speed": speed_name,
                  "speed_deg_s": speed_deg_s, "publish_rate_hz": publish_rate_hz,
                  "stop": threading.Event(), "thread": None}
        with self._head_lock:
            self._head_active = active
            thread = threading.Thread(target=self._run_head_motion,
                                      args=(active, [float(current[name]) for name in HEAD_JOINTS], target),
                                      daemon=True, name=f"panther_head_{action}")
            active["thread"] = thread
            thread.start()
        return {"state": "queued", "action_id": action_id,
                "action": action, "speed": speed_name, "speed_deg_s": speed_deg_s,
                "yaw_deg": math.degrees(target[0]), "pitch_deg": math.degrees(target[1])}

    def _run_head_motion(self, active: dict, start: list[float], target: list[float]) -> None:
        from std_msgs.msg import Float64MultiArray
        rate = active["publish_rate_hz"]
        speed = math.radians(active["speed_deg_s"])
        distance = max(abs(b - a) for a, b in zip(start, target))
        steps = max(1, math.ceil(distance / speed * rate))
        status = "completed"
        result = {"action": active["action"], "speed": active["speed"],
                  "speed_deg_s": active["speed_deg_s"],
                  "yaw_deg": math.degrees(target[0]), "pitch_deg": math.degrees(target[1])}
        try:
            for index in range(1, steps + 1):
                if active["stop"].wait(1.0 / rate):
                    status = "cancelled"
                    break
                message = Float64MultiArray()
                message.data = [a + (b - a) * index / steps for a, b in zip(start, target)]
                self._head_pub.publish(message)
        except Exception as exc:
            status = "error"
            result["error"] = f"{type(exc).__name__}: {exc}"
        finally:
            with self._head_lock:
                if self._head_active is active:
                    self._head_active = None
            _notify_completion(active["action_id"], status, result, "head")

    def _stop_head(self, reason: str, wait_s: float) -> dict:
        with self._head_lock:
            active = self._head_active
            if not active:
                return {"state": "idle", "stopped": False}
            active["stop"].set()
            thread = active.get("thread")
            action_id = active["action_id"]
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=wait_s)
        return {"state": "idle" if not thread or not thread.is_alive() else "stopping",
                "stopped": True, "reason": reason, "action_id": action_id}

    def camera(self, kind: str, args: dict) -> dict:
        action = args.get("action", "info")
        if action == "start":
            return self._camera.start(kind)
        if action == "stop":
            return self._camera.stop(kind)
        if action == "info":
            return self._camera.info(kind)
        raise ValueError(f"unsupported camera action: {action}")

    def vision_capture(self, args: dict) -> dict:
        action = args.get("action", "info")
        config = self._cfg.get("plugins", {}).get("vision_capture", {})
        directory = Path(config.get("output_dir", "/opt/phanthy-motus/data/vision_capture/panther")) / "photos"
        if action in ("start", "info"):
            ready = self._node.count_publishers(self._camera.COLOR_TOPIC) > 0
            return {"state": "ready" if ready else "waiting_for_camera",
                    "source_topic": self._camera.COLOR_TOPIC, "photos_dir": str(directory)}
        if action == "stop":
            return {"state": "idle"}
        if action != "capture_photo":
            raise ValueError(f"unsupported vision_capture action: {action}")

        try:
            payload = self._camera.capture_jpeg(timeout_s=3.0)
            captured_at = datetime.now().astimezone()
            directory.mkdir(parents=True, exist_ok=True)
            path = directory / f"IMG_{captured_at:%Y%m%d_%H%M%S}_{uuid4().hex}.jpg"
            partial = path.with_suffix(".part")
            output = partial.open("xb")
            try:
                with output:
                    output.write(payload["data"])
                partial.replace(path)
            finally:
                partial.unlink(missing_ok=True)
        except Exception as exc:
            return {"ok": False, "state": "error", "code": "CAPTURE_FAILED",
                    "message": self._camera._bounded_error(exc)}
        return {"ok": True, "state": "completed", "media_type": "photo",
                "file_path": str(path), "captured_at": captured_at.isoformat(timespec="seconds"),
                "frame_id": payload["frame_id"], "stamp": payload["stamp"]}

