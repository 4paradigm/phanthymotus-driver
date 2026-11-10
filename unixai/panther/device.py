"""Panther ROS 2 device adapter.

All robot-facing endpoints live on CycloneDDS/domain 0.  Sensor payloads are
normalized and handed to ``core_bridge.BridgeWorker``; no robot command topic
is forwarded into Agent Core's DDS domain.
"""

from __future__ import annotations

import threading
import time

from contracts import (
    HEAD_JOINTS,
    LEFT_ARM_JOINTS,
    RIGHT_ARM_JOINTS,
    age_ms,
    finite_number,
    fresh,
    json_float,
    parse_joint_indices,
    parse_joint_limits,
    populate_switch_request,
    require_confirmation,
    validate_drive,
    validate_joint_target,
)
from core_bridge import BridgeWorker, output_topics


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
        self._urdf = ""
        self._joint_limits: dict[str, tuple[float, float]] = {}
        self._joint_indices: dict[str, int] = {}
        self._drive_lock = threading.Lock()
        self._drive_generation = 0
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
        self._node.create_subscription(
            Odometry, "/odom", lambda msg: self._store_and_publish("odom", msg, self._odom_payload), sensor_qos
        )
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
            LaserScan,
            "/scan",
            lambda msg: self._store_and_publish("laser_scan", msg, self._laser_payload),
            sensor_qos,
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
        self._bridge.publish(card, builder(msg, received))

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
        print(
            f"[model] loaded live URDF with {len(indices)} kinematic joints "
            f"and {len(limits)} joint limits",
            flush=True,
        )

    def _on_joints(self, msg) -> None:
        received = self._put("joints", msg)
        self._bridge.publish("joints", self._joints_payload(msg, received))

    def _on_diagnostics(self, msg) -> None:
        self._put("diagnostics", msg)
        self._publish_robot_state()

    def _on_hardware_states(self, msg) -> None:
        self._put("hardware_states", msg)
        self._publish_robot_state()

    def _on_hardware_event(self, msg) -> None:
        self._put("hardware_event", msg)
        self._publish_robot_state()

    def _on_battery(self, msg) -> None:
        self._put("battery", msg)
        self._publish_robot_state()

    def _on_gripper(self, side: str, msg) -> None:
        self._put(f"gripper_{side}", msg)
        self._publish_robot_state()

    def _on_lifting(self, msg) -> None:
        self._put("lifting", msg)
        self._publish_robot_state()

    def _publish_robot_state(self) -> None:
        self._bridge.publish("robot_state", self.robot_state())

    def _joints_payload(self, msg, received) -> dict:
        with self._lock:
            urdf = self._urdf
            joint_indices = dict(self._joint_indices)
        if not urdf or not joint_indices:
            return {
                "timestamp_ms": int(time.time() * 1000),
                "fresh": fresh(received),
                "available": False,
                "age_ms": age_ms(received),
                "format": "sensor/skeleton",
                "source_topic": "/joint_states",
                "joints": [],
                "joint_count": 0,
                "message": "Waiting for authoritative /robot_description before mapping joints",
            }
        joints = []
        for index, name in enumerate(msg.name):
            if name not in joint_indices:
                continue
            item = {"idx": joint_indices[name], "name": name}
            if index < len(msg.position):
                item["q"] = json_float(msg.position[index])
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
            "joints": joints,
        }
        payload["urdf"] = urdf
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
        return {
            "timestamp_ms": int(time.time() * 1000),
            "fresh": fresh(received),
            "age_ms": age_ms(received),
            "source_topic": "/imu_sensor_broadcaster/imu",
            "frame_id": msg.header.frame_id,
            "orientation": _message_dict(msg.orientation),
            "angular_velocity": _message_dict(msg.angular_velocity),
            "linear_acceleration": _message_dict(msg.linear_acceleration),
        }

    @staticmethod
    def _odom_payload(msg, received) -> dict:
        twist = msg.twist.twist
        pose = msg.pose.pose
        return {
            "schema": "motus.odom/1",
            "stamp_ms": int(time.time() * 1000),
            "fresh": fresh(received),
            "age_ms": age_ms(received),
            "frame": "body",
            "source_frame_id": msg.header.frame_id,
            "child_frame_id": msg.child_frame_id,
            "units": {"linear": "m/s", "angular": "rad/s", "length": "m"},
            "twist": [
                json_float(twist.linear.x), json_float(twist.linear.y), json_float(twist.linear.z),
                json_float(twist.angular.x), json_float(twist.angular.y), json_float(twist.angular.z),
            ],
            "pose": {
                "position": _message_dict(pose.position),
                "orientation": _message_dict(pose.orientation),
            },
            "source_topic": "/odom",
        }

    @staticmethod
    def _laser_payload(msg, received) -> dict:
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
            "angle_increment": json_float(msg.angle_increment),
            "range_min": json_float(msg.range_min),
            "range_max": json_float(msg.range_max),
            "ranges": [json_float(value) for value in msg.ranges],
            "intensities": [json_float(value) for value in msg.intensities],
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

    def sensor_info(self, card: str) -> dict:
        if card == "robot_state":
            return self.robot_state()
        if card == "model":
            with self._lock:
                return {"model": "UniX AI Panther", "urdf": self._urdf,
                        "available": bool(self._urdf), "source_topic": "/robot_description"}
        msg, received = self._get(card)
        if msg is None:
            return {"available": False, "fresh": False, "source": card}
        builders = {
            "joints": self._joints_payload,
            "imu": self._imu_payload,
            "odom": self._odom_payload,
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
        return {
            "timestamp_ms": int(time.time() * 1000),
            "ready_for_motion": fresh(joints_at) and fresh(hardware_at, 5.0),
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
                            "age_ms": age_ms(diag_at), "data": _message_dict(diagnostics)},
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
            self.cancel_drive()
            return {"state": "idle", "topic": self._cfg["plugins"]["base_drive"]["topic"]}
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
        rate = float(cfg["publish_rate_hz"])
        repetitions = int(cfg["stop_repetitions"])
        with self._drive_lock:
            self._drive_generation += 1
            generation = self._drive_generation

        def run():
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

        threading.Thread(target=run, daemon=True, name="panther_drive").start()
        return {"state": "running", "linear_x": vx, "linear_y": vy, "angular_z": wz,
                "duration_s": duration, "topic": cfg["topic"], "auto_stop": True}

    def cancel_drive(self) -> None:
        from geometry_msgs.msg import Twist
        with self._drive_lock:
            self._drive_generation += 1
        repetitions = int(self._cfg["plugins"]["base_drive"].get("stop_repetitions", 3))
        for _ in range(max(1, repetitions)):
            self._drive_pub.publish(Twist())

    def _switch_arm_mode(self, p2p: bool) -> None:
        from controller_manager_msgs.srv import SwitchController
        if not self._switch_controllers.wait_for_service(timeout_sec=2.0):
            raise RuntimeError("/controller_manager/switch_controller is unavailable")
        request = SwitchController.Request()
        p2p_names = [
            "left_arm_joint_trajectory_position_controller",
            "right_arm_joint_trajectory_position_controller",
        ]
        populate_switch_request(
            request,
            p2p_names if p2p else ["dual_arm_forward_position_controller"],
            ["dual_arm_forward_position_controller"] if p2p else p2p_names,
        )
        response = self._wait_future(self._switch_controllers.call_async(request), 7.0)
        if not response.ok:
            raise RuntimeError("controller manager rejected arm mode switch")

    def arm(self, args: dict) -> dict:
        from builtin_interfaces.msg import Duration
        from control_msgs.action import FollowJointTrajectory
        from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
        action = args.get("action", "move")
        if action in ("start", "info"):
            return {"state": "ready", "controllers": self.controller_states(),
                    "joint_state_fresh": fresh(self._get("joints")[1]),
                    "limits_loaded": bool(self._joint_limits),
                    "commissioning_motion_enabled": self._cfg.get("safety", {}).get("commissioning_motion_enabled") is True}
        if action not in ("move",):
            raise ValueError(f"unsupported arm_control action: {action}")
        require_confirmation(args)
        self._ensure_motion_ready()
        duration = finite_number(args.get("duration_s", 6.0), "duration_s")
        max_duration = float(self._cfg["plugins"]["arm_control"]["max_duration_s"])
        if not 1.0 <= duration <= max_duration:
            raise ValueError(f"duration_s must be in [1.0, {max_duration}]")
        side = args.get("side", "left")
        if side not in ("left", "right", "both"):
            raise ValueError("side must be left, right, or both")
        joint_msg, received = self._get("joints")
        if not fresh(received):
            raise RuntimeError("fresh /joint_states is required")
        current = {name: value for name, value in zip(joint_msg.name, joint_msg.position)}
        max_delta = float(self._cfg["plugins"]["arm_control"]["max_target_delta_rad"])
        targets = {}
        if side in ("left", "both"):
            field = "positions" if side == "left" else "left_positions"
            targets["left"] = validate_joint_target(
                LEFT_ARM_JOINTS, args.get(field), self._joint_limits, current, max_delta
            )
        if side in ("right", "both"):
            field = "positions" if side == "right" else "right_positions"
            targets["right"] = validate_joint_target(
                RIGHT_ARM_JOINTS, args.get(field), self._joint_limits, current, max_delta
            )
        self._switch_arm_mode(True)
        results = {}
        try:
            goal_handles = {}
            for arm_side, positions in targets.items():
                client = self._arm_actions[arm_side]
                if not client.wait_for_server(timeout_sec=3.0):
                    raise RuntimeError(f"{arm_side} FollowJointTrajectory action is unavailable")
                trajectory = JointTrajectory()
                trajectory.joint_names = list(LEFT_ARM_JOINTS if arm_side == "left" else RIGHT_ARM_JOINTS)
                point = JointTrajectoryPoint()
                point.positions = positions
                point.time_from_start = Duration(
                    sec=int(duration), nanosec=int((duration % 1.0) * 1_000_000_000)
                )
                trajectory.points = [point]
                goal = FollowJointTrajectory.Goal(); goal.trajectory = trajectory
                handle = self._wait_future(client.send_goal_async(goal), 5.0)
                if not handle.accepted:
                    raise RuntimeError(f"{arm_side} trajectory goal was rejected")
                goal_handles[arm_side] = handle
            for arm_side, handle in goal_handles.items():
                wrapped = self._wait_future(handle.get_result_async(), duration + 5.0)
                results[arm_side] = {"status": int(wrapped.status),
                                     "error_code": int(wrapped.result.error_code),
                                     "error_string": wrapped.result.error_string}
                if wrapped.result.error_code != 0:
                    raise RuntimeError(f"{arm_side} trajectory failed: {wrapped.result.error_string}")
        finally:
            self._switch_arm_mode(False)
        return {"state": "completed", "side": side, "duration_s": duration,
                "result": results, "restored_controller": "dual_arm_forward_position_controller"}

    def gripper(self, args: dict) -> dict:
        from sensor_msgs.msg import JointState
        from unix_msgs.msg import GripperCommand
        action = args.get("action", "set")
        if action in ("start", "info"):
            return {"state": "ready", "feedback": self.gripper_state()}
        if action != "set":
            raise ValueError(f"unsupported gripper action: {action}")
        require_confirmation(args)
        self._ensure_motion_ready()
        side = args.get("side")
        if side not in ("left", "right"):
            raise ValueError("side must be left or right")
        self._require_controller(f"{side}_unix_gripper3_controller")
        position = finite_number(args.get("position"), "position")
        if not 0.0 <= position <= 1.0:
            raise ValueError("position must be in [0, 1]")
        mode = args.get("mode", "parallel")
        if mode not in ("2-finger", "parallel", "centric"):
            raise ValueError("mode must be 2-finger, parallel, or centric")
        message = GripperCommand(); message.mode = mode
        command = JointState(); command.name = [""]; command.position = [position]
        message.command = command
        (self._left_gripper_pub if side == "left" else self._right_gripper_pub).publish(message)
        return {"state": "commanded", "side": side, "mode": mode, "position": position}

    def lifting(self, args: dict) -> dict:
        from sensor_msgs.msg import JointState
        action = args.get("action", "set_position")
        if action in ("start", "info"):
            return {"state": "ready", "feedback": self.robot_state()["lifting"]}
        if action != "set_position":
            raise ValueError(f"unsupported lifting action: {action}")
        require_confirmation(args)
        self._ensure_motion_ready()
        self._require_controller("lifting_controller")
        position = finite_number(args.get("position_m"), "position_m")
        maximum = float(self._cfg["plugins"]["lifting"]["max_position_m"])
        if not 0.0 <= position <= maximum:
            raise ValueError(f"position_m must be in [0, {maximum}]")
        message = JointState(); message.name = [""]; message.position = [position]
        self._lifting_pub.publish(message)
        return {"state": "commanded", "position_m": position}

    def head(self, args: dict) -> dict:
        from std_msgs.msg import Float64MultiArray
        action = args.get("action", "look")
        if action in ("start", "info"):
            return {"state": "ready", "limits_loaded": all(name in self._joint_limits for name in HEAD_JOINTS)}
        if action != "look":
            raise ValueError(f"unsupported head action: {action}")
        require_confirmation(args)
        self._ensure_motion_ready()
        self._require_controller("head_forward_position_controller")
        pan = finite_number(args.get("pan_rad"), "pan_rad")
        tilt = finite_number(args.get("tilt_rad"), "tilt_rad")
        missing = [name for name in HEAD_JOINTS if name not in self._joint_limits]
        if missing:
            raise RuntimeError("live URDF limits are required for: " + ", ".join(missing))
        for name, value in zip(HEAD_JOINTS, (pan, tilt)):
            lower, upper = self._joint_limits[name]
            if not lower <= value <= upper:
                raise ValueError(f"{name} target is outside URDF limit [{lower}, {upper}]")
        message = Float64MultiArray(); message.data = [pan, tilt]
        self._head_pub.publish(message)
        return {"state": "commanded", "pan_rad": pan, "tilt_rad": tilt}

    def camera(self, kind: str, args: dict) -> dict:
        action = args.get("action", "info")
        if action == "start":
            return self._camera.start(kind)
        if action == "stop":
            return self._camera.stop(kind)
        if action == "info":
            return self._camera.info(kind)
        raise ValueError(f"unsupported camera action: {action}")

