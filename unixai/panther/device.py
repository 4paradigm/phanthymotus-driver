"""Panther ROS 2 device adapter.

All robot-facing endpoints live on CycloneDDS/domain 0.  Sensor payloads are
normalized and handed to ``core_bridge.BridgeWorker``; no robot command topic
is forwarded into Agent Core's DDS domain.
"""

from __future__ import annotations

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
    HEAD_JOINTS,
    LEFT_ARM_JOINTS,
    RIGHT_ARM_JOINTS,
    age_ms,
    degrees_vector_to_radians,
    finite_number,
    fresh,
    head_target_from_degrees,
    json_float,
    parse_joint_indices,
    parse_joint_limits,
    populate_switch_request,
    require_confirmation,
    validate_drive,
    validate_joint_target,
)
from common.odom import build_interface, build_sample, mean_twist, resolve_stamp_ms
from core_bridge import BridgeWorker, output_topics


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
        self._model_urdf = MODEL_PATH.read_text(encoding="utf-8")
        self._model_joint_indices = parse_joint_indices(self._model_urdf)
        self._model_joint_limits = parse_joint_limits(self._model_urdf)
        self._urdf = ""
        self._joint_limits: dict[str, tuple[float, float]] = {}
        self._joint_indices: dict[str, int] = {}
        self._odom_window: list[list] = []
        self._odom_vendor_ms = None
        self._odom_vendor_meta: dict = {}
        self._drive_lock = threading.Lock()
        self._drive_generation = 0
        self._drive_action_id = None
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

        self._node.create_subscription(JointState, "/joint_states", self._on_joints, sensor_qos)
        self._node.create_subscription(
            BatteryState,
            "/battery_state_broadcaster/battery_state",
            self._on_battery,
            sensor_qos,
        )
        self._node.create_subscription(
            Imu,
            "/imu_sensor_broadcaster/imu",
            lambda msg: self._store_and_publish("imu", msg, self._imu_payload),
            sensor_qos,
        )
        self._node.create_subscription(Odometry, "/odom", self._on_odom, sensor_qos)
        self._node.create_timer(0.1, self._publish_odom_window)
        self._node.create_subscription(
            DiagnosticArray, "/diagnostics", self._on_diagnostics, sensor_qos
        )
        self._node.create_subscription(
            DiagnosticArray, "/hardware_states", self._on_hardware_states, sensor_qos
        )
        self._node.create_subscription(
            DiagnosticStatus, "/hardware_events", self._on_hardware_event, reliable_qos
        )
        self._node.create_subscription(
            JointState, "/left_gripper/joint_states", lambda msg: self._on_gripper("left", msg), sensor_qos
        )
        self._node.create_subscription(
            JointState, "/right_gripper/joint_states", lambda msg: self._on_gripper("right", msg), sensor_qos
        )
        self._node.create_subscription(
            JointState,
            "/lifting_controller/joint_states",
            self._on_lifting,
            sensor_qos,
        )
        self._node.create_subscription(String, "/robot_description", self._on_robot_description, latched_qos)
        self._node.create_subscription(
            LaserScan, "/scan", self._on_laser, sensor_qos,
        )

        self._drive_pub = self._node.create_publisher(
            Twist, str(config["plugins"]["base_drive"]["topic"]), reliable_qos
        )
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
        self._node.create_timer(1.0, self._poll_controllers)
        self._camera = PantherCameraRuntime(
            self._node, self._bridge, config.get("plugins", {}).get("camera", {}), sensor_qos
        )

    @property
    def topics(self):
        return self._topics

    def start(self) -> None:
        self._bridge.start()

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        self.cancel_drive()
        self._stop_arm("driver_shutdown", wait_s=20.0)
        self._stop_head("driver_shutdown", wait_s=5.0)
        self._camera.close()
        self._bridge.stop()
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
            self._joint_indices = indices
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
        self._put("battery", msg)
        self._publish_robot_state()

    def _on_gripper(self, side: str, msg) -> None:
        self._put(f"gripper_{side}", msg)
        self._publish_robot_state()

    def _on_lifting(self, msg) -> None:
        self._put("lifting", msg)
        self._publish_robot_state()

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

    def _on_laser(self, msg) -> None:
        received = self._put("laser_scan", msg)
        payload = self._laser_payload(msg, received)
        self._publish_limited("laser_scan", payload, 5.0)
        now = time.monotonic()
        with self._lock:
            previous = self._last_card_publish.get("laser_view", 0.0)
            due = now - previous >= 1.0
        if not due:
            return
        try:
            import cv2
            import numpy as np
            size = 400
            image = np.zeros((size, size, 3), dtype=np.uint8)
            center = size // 2
            maximum = min(6.0, float(payload.get("range_max") or 6.0))
            for index, distance in enumerate(payload["ranges"]):
                if distance is None or distance > maximum:
                    continue
                angle = payload["angle_min"] + index * payload["angle_increment"]
                radius = distance / maximum * (center - 8)
                x = int(round(center + radius * math.cos(angle)))
                y = int(round(center - radius * math.sin(angle)))
                if 0 <= x < size and 0 <= y < size:
                    image[y, x] = (0, 220, 0)
            cv2.circle(image, (center, center), 4, (0, 80, 255), -1)
            ok, encoded = cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, 75])
            if ok:
                self._publish_limited("laser_view", {
                    "message_format": "jpeg", "data": encoded.tobytes(),
                    "frame_id": payload.get("frame_id", ""),
                }, 1.0)
        except Exception as exc:
            print(f"[laser_scan] top-down view failed: {exc}", flush=True)

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
        joint_indices = self._model_joint_indices
        joints = []
        dropped = 0
        for index, name in enumerate(msg.name):
            if name not in joint_indices:
                continue
            if index >= len(msg.position):
                dropped += 1
                continue
            try:
                position = float(msg.position[index])
            except (TypeError, ValueError):
                dropped += 1
                continue
            if not math.isfinite(position):
                dropped += 1
                continue
            item = {"idx": joint_indices[name], "name": name}
            item["q"] = position
            if index < len(msg.velocity):
                item["dq"] = json_float(msg.velocity[index])
            if index < len(msg.effort):
                item["tau"] = json_float(msg.effort[index])
            joints.append(item)
        payload = {
            "timestamp_ms": int(time.time() * 1000),
            "fresh": fresh(received),
            "available": True,
            "age_ms": age_ms(received),
            "format": "sensor/skeleton",
            "source_topic": "/joint_states",
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
            "power_supply_status": int(msg.power_supply_status),
            "present": bool(msg.present),
        }

    @staticmethod
    def _imu_payload(msg, received) -> dict:
        def known(covariance) -> bool:
            try:
                return not covariance or float(covariance[0]) != -1.0
            except (TypeError, ValueError):
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
    def _laser_payload(msg, received) -> dict:
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
            "source_topic": "/scan",
            "source_node": "dual_lidar_merger",
            "merged": True,
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
        """Prefer the live limit for the joint reported by the lift controller."""
        configured_max = float(self._cfg["plugins"]["lifting"]["max_position_m"])
        feedback, _ = self._get("lifting")
        joint_name = next((name for name in getattr(feedback, "name", []) if name), None)
        live_limit = self._joint_limits.get(joint_name) if joint_name else None
        if live_limit is None:
            return 0.0, configured_max, "configured safety limit", joint_name
        lower, upper = live_limit
        return max(0.0, lower), min(configured_max, upper), "live URDF capped by config", joint_name

    def sensor_info(self, card: str) -> dict:
        if card == "robot_state":
            return self.robot_state()
        if card == "model":
            with self._lock:
                live_limits = dict(self._joint_limits)
            different = sorted(
                name for name in set(self._model_joint_limits) | set(live_limits)
                if self._model_joint_limits.get(name) != live_limits.get(name)
            ) if live_limits else []
            return {"model": "UniX AI Panther", "urdf": self._model_urdf,
                    "available": True, "source": "bundled verified model",
                    "live_robot_description_available": bool(live_limits),
                    "live_limit_differences": different}
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
            }
        msg, received = self._get(card)
        if msg is None:
            return {"available": False, "fresh": False, "source": card}
        builders = {
            "joints": self._joints_payload,
            "imu": self._imu_payload,
            "laser_scan": self._laser_payload,
        }
        return {"available": True, **builders[card](msg, received)}

    def robot_state(self) -> dict:
        diagnostics, diag_at = self._get("diagnostics")
        hardware, hardware_at = self._get("hardware_states")
        event, event_at = self._get("hardware_event")
        joints, joints_at = self._get("joints")
        battery, battery_at = self._get("battery")
        lifting, lifting_at = self._get("lifting")
        controllers, controllers_at = self._get("controllers")
        reasons = []
        if self._cfg.get("safety", {}).get("commissioning_motion_enabled") is not True:
            reasons.append("commissioning motion is disabled")
        if not fresh(joints_at):
            reasons.append("no fresh /joint_states")
        if not fresh(hardware_at, 5.0):
            reasons.append("no fresh /hardware_states")
        diagnostic_summary = None
        if diagnostics is not None:
            statuses = list(getattr(diagnostics, "status", ()))
            problems = [f"{item.name}: {item.message}" for item in statuses if int(item.level) != 0]
            worst = max((int(item.level) for item in statuses), default=0)
            diagnostic_summary = {
                "count": len(statuses), "worst_level": worst,
                "problems": sorted(problems),
            }
        return {
            "timestamp_ms": int(time.time() * 1000),
            "ready_for_motion": not reasons,
            "not_ready_reasons": reasons,
            "joint_state": {"available": joints is not None, "fresh": fresh(joints_at), "age_ms": age_ms(joints_at)},
            "battery": ({"available": False, "fresh": False}
                        if battery is None else {"available": True, **self._battery_payload(battery, battery_at)}),
            "grippers": self.gripper_state(),
            "lifting": ({"available": False, "fresh": False}
                        if lifting is None else {"available": True, **self._lifting_payload(lifting, lifting_at)}),
            "controllers": {
                "available": controllers is not None,
                "fresh": fresh(controllers_at, 3.0),
                "age_ms": age_ms(controllers_at),
                "states": controllers or {},
            },
            "diagnostics": {"available": diagnostics is not None, "fresh": fresh(diag_at, 5.0),
                            "age_ms": age_ms(diag_at), "summary": diagnostic_summary},
            "hardware_states": {"available": hardware is not None, "fresh": fresh(hardware_at, 5.0),
                                "age_ms": age_ms(hardware_at), "data": _message_dict(hardware)},
            "last_hardware_event": {"available": event is not None, "age_ms": age_ms(event_at),
                                    "data": _message_dict(event)},
            "estop": {"reported": False, "message": "No ROS emergency-stop state was identified"},
        }

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
        if self._cfg.get("safety", {}).get("commissioning_motion_enabled") is not True:
            raise RuntimeError(
                "motion is disabled by safety.commissioning_motion_enabled; "
                "confirm commissioning limits before enabling it"
            )
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
                self._publish_robot_state()
            except Exception as exc:
                print(f"[robot_state] controller poll failed: {exc}", flush=True)

        future.add_done_callback(completed)

    def controller_states(self) -> dict[str, str]:
        from controller_manager_msgs.srv import ListControllers
        if not self._list_controllers.wait_for_service(timeout_sec=2.0):
            raise RuntimeError("/controller_manager/list_controllers is unavailable")
        result = self._wait_future(self._list_controllers.call_async(ListControllers.Request()), 3.0)
        states = {entry.name: entry.state for entry in result.controller}
        self._put("controllers", states)
        return states

    def _require_controller(self, name: str, state: str = "active") -> None:
        actual = self.controller_states().get(name)
        if actual != state:
            raise RuntimeError(f"controller {name} is {actual!r}, expected {state!r}")

    def drive(self, args: dict) -> dict:
        from geometry_msgs.msg import Twist
        action = args.get("action", "move")
        if action in ("cancel", "stop"):
            stopped = self.cancel_drive("cancelled_by_request")
            return {"state": "idle", "topic": self._cfg["plugins"]["base_drive"]["topic"],
                    "action_id": stopped}
        if action in ("start", "info"):
            states = self.controller_states()
            return {"state": "ready" if states.get("movebase3_drive_controller") == "active" else "unavailable",
                    "controllers": states, "topic": self._cfg["plugins"]["base_drive"]["topic"],
                    "commissioning_motion_enabled": self._cfg.get("safety", {}).get("commissioning_motion_enabled") is True}
        if action != "move":
            raise ValueError(f"unsupported base_drive action: {action}")
        self._ensure_motion_ready()
        self._require_controller("movebase3_drive_controller")
        cfg = self._cfg["plugins"]["base_drive"]
        vx, vy, wz, duration = validate_drive(args, cfg)
        wz_deg_s = math.degrees(wz)
        rate = float(cfg["publish_rate_hz"])
        repetitions = int(cfg["stop_repetitions"])
        self.cancel_drive("replaced_by_new_command")
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
                        msg = Twist(); msg.linear.x = vx; msg.linear.y = vy; msg.angular.z = wz
                        self._drive_pub.publish(msg)
                    time.sleep(period)
                for _ in range(repetitions):
                    with self._drive_lock:
                        if generation != self._drive_generation:
                            return
                        self._drive_pub.publish(Twist())
                    time.sleep(period)
                with self._drive_lock:
                    if generation != self._drive_generation or self._drive_action_id != action_id:
                        return
                    self._drive_action_id = None
                _notify_completion(action_id, "completed", {
                    "linear_x": vx, "linear_y": vy, "angular_z_deg_s": wz_deg_s,
                    "duration_s": duration, "stopped_automatically": True,
                }, "base_drive")
            except Exception as exc:
                with self._drive_lock:
                    if self._drive_action_id == action_id:
                        self._drive_action_id = None
                _notify_completion(action_id, "error",
                                   {"error": f"{type(exc).__name__}: {exc}"}, "base_drive")

        threading.Thread(target=run, daemon=True, name="panther_drive").start()
        return {"state": "queued", "action_id": action_id,
                "linear_x": vx, "linear_y": vy, "angular_z_deg_s": wz_deg_s,
                "duration_s": duration, "topic": cfg["topic"], "auto_stop": True}

    def cancel_drive(self, reason: str = "cancelled"):
        from geometry_msgs.msg import Twist
        with self._drive_lock:
            action_id = self._drive_action_id
            self._drive_action_id = None
            self._drive_generation += 1
            repetitions = int(self._cfg["plugins"]["base_drive"].get("stop_repetitions", 3))
            for _ in range(max(1, repetitions)):
                self._drive_pub.publish(Twist())
        if action_id:
            _notify_completion(action_id, "cancelled", {"reason": reason}, "base_drive")
        return action_id

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
            return {"state": "moving" if active else "ready", "active": active,
                    "controllers": self.controller_states(),
                    "joint_state_fresh": fresh(self._get("joints")[1]),
                    "limits_loaded": bool(self._joint_limits),
                    "commissioning_motion_enabled": self._cfg.get("safety", {}).get("commissioning_motion_enabled") is True}
        if action in ("stop", "cancel"):
            return self._stop_arm("cancelled_by_request", wait_s=10.0)
        if action not in ("move", "home"):
            raise ValueError(f"unsupported arm_control action: {action}")
        require_confirmation(args)
        self._ensure_motion_ready()
        cfg = self._cfg["plugins"]["arm_control"]
        max_duration = float(cfg["max_duration_s"])
        side = args.get("side", "both" if action == "home" else "left")
        if side not in ("left", "right", "both"):
            raise ValueError("side must be left, right, or both")
        joint_msg, received = self._get("joints")
        if not fresh(received):
            raise RuntimeError("fresh /joint_states is required")
        current = {name: value for name, value in zip(joint_msg.name, joint_msg.position)}
        max_delta = math.radians(float(cfg["max_target_delta_deg"]))
        targets: dict[str, list[float]] = {}

        def target_for(arm_side: str, names: tuple[str, ...]) -> list[float]:
            legacy_field = "positions_deg" if side == arm_side else f"{arm_side}_positions_deg"
            legacy = args.get(legacy_field)
            if legacy is not None:
                values = degrees_vector_to_radians(legacy, legacy_field, len(names))
                return validate_joint_target(names, values, self._joint_limits, current, max_delta)
            if action == "home":
                home = cfg.get("home_pose_deg")
                if not isinstance(home, dict) or not home:
                    raise RuntimeError("arm_control home_pose_deg is not configured; refusing unverified home")
                values = [current.get(name) if home.get(name) is None
                          else math.radians(finite_number(home[name], f"home_pose_deg.{name}"))
                          for name in names]
            else:
                prefix = "la" if arm_side == "left" else "ra"
                updates = {
                    name: math.radians(finite_number(args[f"{prefix}{index}_deg"],
                                                     f"{prefix}{index}_deg"))
                    for index, name in enumerate(names)
                    if args.get(f"{prefix}{index}_deg") is not None
                }
                if not updates:
                    raise ValueError(f"at least one {prefix}0_deg..{prefix}7_deg target is required")
                values = [updates.get(name, current.get(name)) for name in names]
            return validate_joint_target(names, values, self._joint_limits, current, max_delta)

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
        from sensor_msgs.msg import JointState
        from unix_msgs.msg import GripperCommand
        action = args.get("action", "set")
        if action in ("start", "info"):
            return {"state": "ready", "feedback": self.gripper_state()}
        if action not in ("set", "open", "close"):
            raise ValueError(f"unsupported gripper action: {action}")
        require_confirmation(args)
        self._ensure_motion_ready()
        side = args.get("side", "both")
        if side not in ("left", "right", "both"):
            raise ValueError("side must be left, right, or both")
        sides = ("left", "right") if side == "both" else (side,)
        states = self.controller_states()
        unavailable = [name for name in (f"{item}_unix_gripper3_controller" for item in sides)
                       if states.get(name) != "active"]
        if unavailable:
            raise RuntimeError("inactive gripper controllers: " + ", ".join(unavailable))
        position = 0.0 if action == "open" else 1.0 if action == "close" else finite_number(
            args.get("position"), "position")
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
        from sensor_msgs.msg import JointState
        action = args.get("action", "set_position")
        if action in ("start", "info"):
            minimum, maximum, limit_source, joint_name = self._lifting_limits()
            return {"state": "ready", "feedback": self.robot_state()["lifting"],
                    "limits_m": [minimum, maximum], "limit_source": limit_source,
                    "joint_name": joint_name}
        if action != "set_position":
            raise ValueError(f"unsupported lifting action: {action}")
        require_confirmation(args)
        self._ensure_motion_ready()
        self._require_controller("lifting_controller")
        position = finite_number(args.get("position_m"), "position_m")
        minimum, maximum, limit_source, joint_name = self._lifting_limits()
        if maximum < minimum:
            raise RuntimeError("lifting limits are invalid; refusing motion")
        if not minimum <= position <= maximum:
            raise ValueError(f"position_m must be in [{minimum}, {maximum}]")
        message = JointState(); message.name = [""]; message.position = [position]
        self._lifting_pub.publish(message)
        return {"state": "commanded", "position_m": position,
                "limits_m": [minimum, maximum], "limit_source": limit_source,
                "joint_name": joint_name}

    def head(self, args: dict) -> dict:
        from std_msgs.msg import Float64MultiArray
        action = args.get("action", "look")
        if action in ("start", "info"):
            with self._head_lock:
                active = ({"action_id": self._head_active["action_id"]}
                          if self._head_active else None)
            return {"state": "moving" if active else "ready", "active": active,
                    "limits_loaded": all(name in self._joint_limits for name in HEAD_JOINTS)}
        if action in ("stop", "cancel"):
            return self._stop_head("cancelled_by_request", wait_s=5.0)
        if action not in ("look", "home"):
            raise ValueError(f"unsupported head action: {action}")
        require_confirmation(args)
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
        self._stop_head("replaced_by_new_command", wait_s=5.0)
        if action == "look":
            message = Float64MultiArray(); message.data = target
            self._head_pub.publish(message)
            return {"state": "commanded", "yaw_deg": math.degrees(target[0]),
                    "pitch_deg": math.degrees(target[1])}

        action_id = str(args.get("action_id") or f"panther_head_{uuid4().hex[:12]}")
        active = {"action_id": action_id, "stop": threading.Event(), "thread": None}
        with self._head_lock:
            self._head_active = active
            thread = threading.Thread(target=self._run_head_home,
                                      args=(active, [float(current[name]) for name in HEAD_JOINTS], target),
                                      daemon=True, name="panther_head_home")
            active["thread"] = thread
            thread.start()
        return {"state": "queued", "action_id": action_id,
                "yaw_deg": math.degrees(target[0]), "pitch_deg": math.degrees(target[1])}

    def _run_head_home(self, active: dict, start: list[float], target: list[float]) -> None:
        from std_msgs.msg import Float64MultiArray
        cfg = self._cfg["plugins"].get("head", {})
        rate = float(cfg.get("home_publish_rate_hz", 20.0))
        speed = math.radians(float(cfg.get("home_speed_deg_s", 20.0)))
        steps = max(1, int(max(abs(b - a) for a, b in zip(start, target)) / speed * rate))
        status = "completed"
        result = {"yaw_deg": math.degrees(target[0]), "pitch_deg": math.degrees(target[1])}
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

