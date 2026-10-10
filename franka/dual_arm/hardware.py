"""ROS 2 adapter to official franka_ros2 controllers, with no direct FCI loop."""
import copy
import math
import threading
import time


class RosArm:
    def __init__(self, ros2, config, side):
        from rclpy.action import ActionClient
        from rclpy.callback_groups import ReentrantCallbackGroup
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import JointState
        from geometry_msgs.msg import PoseStamped, WrenchStamped
        from control_msgs.action import FollowJointTrajectory, GripperCommand
        self.config, self.joints = config, config["joints"]
        self.node = Node(f"phanthy_franka_{side}", context=ros2.ctx_robot)
        self.ros2 = ros2
        self._lock = threading.Lock()
        self._sample = None
        self._received = 0.0
        self._samples = {}
        self._sample_errors = {}
        self._sequence = 0
        group = ReentrantCallbackGroup()
        self._subscription = self.node.create_subscription(
            JointState, config["joint_state_topic"], self._state,
            qos_profile_sensor_data, callback_group=group)
        self._extra_subscriptions = []
        for key, message_type, callback in (
                ("desired_joint_state_topic", JointState, self._desired_state),
                ("eef_pose_topic", PoseStamped, self._pose),
                ("external_wrench_topic", WrenchStamped, self._wrench)):
            if config.get(key):
                self._extra_subscriptions.append(self.node.create_subscription(
                    message_type, config[key], callback, qos_profile_sensor_data,
                    callback_group=group))
        self.arm = ActionClient(self.node, FollowJointTrajectory, config["trajectory_action"],
                                callback_group=group)
        self.gripper = None
        if config.get("gripper_action"):
            self.gripper = ActionClient(self.node, GripperCommand, config["gripper_action"],
                                        callback_group=group)
        ros2.executor_robot.add_node(self.node)

    def _state(self, message):
        # Never combine two arms by unqualified joint names. Each subscription
        # belongs to one explicitly configured robot namespace.
        try:
            sample = self._joint_sample(message)
        except (ValueError, IndexError, TypeError):
            self._invalid("measured")
            return
        with self._lock:
            self._sample = self._store("measured", message, sample)
            self._received = time.monotonic()

    def _joint_sample(self, message):
        if len(set(message.name)) != len(message.name):
            raise ValueError("duplicate joint names")
        index = {name: i for i, name in enumerate(message.name)}
        if not all(name in index for name in self.joints):
            raise ValueError("missing joints")
        sample = {"names": list(self.joints)}
        for field in ("position", "velocity", "effort"):
            values = getattr(message, field, [])
            if field == "effort" and not values:
                sample[field] = None  # Missing effort is not zero torque.
            else:
                sample[field] = self._numbers([values[index[n]] for n in self.joints])
        return sample

    @staticmethod
    def _numbers(values):
        if any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) for v in values):
            raise ValueError("non-finite measurement")
        return list(values)

    def _store(self, key, message, data):
        # Caller holds _lock. ROS source time is preserved separately from host
        # receipt time; neither is silently substituted for the other.
        header = getattr(message, "header", None)
        stamp = getattr(header, "stamp", None)
        source_ns = stamp.sec * 1_000_000_000 + stamp.nanosec if stamp else None
        self._sequence += 1
        sample = {**data, "sequence": self._sequence, "source_stamp_ns": source_ns,
                  "frame_id": getattr(header, "frame_id", ""),
                  "received_at_ns": time.time_ns(), "received_monotonic_ns": time.monotonic_ns()}
        self._samples[key] = sample
        self._sample_errors.pop(key, None)
        return sample

    def _invalid(self, key):
        with self._lock:
            self._sample_errors[key] = "invalid source sample"
            if key == "measured":
                self._sample = None

    def _desired_state(self, message):
        try:
            sample = self._joint_sample(message)
        except (ValueError, IndexError, TypeError):
            self._invalid("desired")
            return
        with self._lock:
            self._store("desired", message, sample)

    def _pose(self, message):
        try:
            p, q = message.pose.position, message.pose.orientation
            xyz = self._numbers([p.x, p.y, p.z])
            xyzw = self._numbers([q.x, q.y, q.z, q.w])
            if abs(sum(v * v for v in xyzw) - 1.0) > .01:
                raise ValueError("non-unit quaternion")
        except (ValueError, TypeError, AttributeError):
            self._invalid("eef_pose")
            return
        with self._lock:
            self._store("eef_pose", message, {"position": xyz, "quaternion_xyzw": xyzw})

    def _wrench(self, message):
        try:
            f, t = message.wrench.force, message.wrench.torque
            force, torque = self._numbers([f.x, f.y, f.z]), self._numbers([t.x, t.y, t.z])
        except (ValueError, TypeError, AttributeError):
            self._invalid("external_wrench")
            return
        with self._lock:
            self._store("external_wrench", message, {"force_n": force, "torque_nm": torque})

    def telemetry(self):
        now = time.monotonic_ns()
        with self._lock:
            output = {}
            for key in ("measured", "desired", "eef_pose", "external_wrench"):
                sample = self._samples.get(key)
                age = (now - sample["received_monotonic_ns"]) / 1e6 if sample else None
                error = self._sample_errors.get(key)
                fresh = sample is not None and age <= 500 and error is None
                output[key] = {"fresh": fresh, "age_ms": age,
                               "error": error, "sample": copy.deepcopy(sample) if fresh else None}
            return output

    def state(self):
        with self._lock:
            if self._sample is None or time.monotonic() - self._received > .5:
                raise RuntimeError("fresh joint position/velocity feedback unavailable")
            return copy.deepcopy(self._sample)

    def ready(self, kind):
        client = self.arm if kind == "arm" else self.gripper
        return client is not None and client.server_is_ready()

    def send(self, kind, command):
        if kind == "arm":
            from control_msgs.action import FollowJointTrajectory
            from trajectory_msgs.msg import JointTrajectoryPoint
            from builtin_interfaces.msg import Duration
            goal = FollowJointTrajectory.Goal()
            goal.trajectory.joint_names = self.joints
            for timestamp, positions in ((0.0, command["start"]),
                                         (command["duration"], command["target"])):
                point = JointTrajectoryPoint()
                point.positions = list(positions)
                point.velocities = [0.0] * 7
                point.accelerations = [0.0] * 7
                ns = round(timestamp * 1_000_000_000)
                point.time_from_start = Duration(sec=ns // 1_000_000_000, nanosec=ns % 1_000_000_000)
                goal.trajectory.points.append(point)
            goal.goal_time_tolerance = Duration(sec=2)
            return self.arm.send_goal_async(goal)
        from control_msgs.action import GripperCommand
        goal = GripperCommand.Goal()
        # Official franka_gripper multiplies command.position by two. Public
        # driver width is the FULL opening in metres, not one finger's travel.
        goal.command.position = command["width"] / 2.0
        goal.command.max_effort = command["force"]
        return self.gripper.send_goal_async(goal)

    def close(self):
        self.arm.destroy()
        if self.gripper:
            self.gripper.destroy()
        self.ros2.executor_robot.remove_node(self.node)
        self.node.destroy_node()
