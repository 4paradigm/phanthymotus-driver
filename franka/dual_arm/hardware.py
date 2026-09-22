"""ROS 2 adapter to official franka_ros2 controllers, with no direct FCI loop."""
import threading
import time


class RosArm:
    def __init__(self, ros2, config, side):
        from rclpy.action import ActionClient
        from rclpy.callback_groups import ReentrantCallbackGroup
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import JointState
        from control_msgs.action import FollowJointTrajectory, GripperCommand
        self.config, self.joints = config, config["joints"]
        self.node = Node(f"phanthy_franka_{side}", context=ros2.ctx_robot)
        self.ros2 = ros2
        self._lock = threading.Lock()
        self._sample = None
        self._received = 0.0
        group = ReentrantCallbackGroup()
        self._subscription = self.node.create_subscription(
            JointState, config["joint_state_topic"], self._state,
            qos_profile_sensor_data, callback_group=group)
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
        if len(set(message.name)) != len(message.name):
            return
        index = {name: i for i, name in enumerate(message.name)}
        if not all(name in index for name in self.joints):
            return
        try:
            sample = {"names": list(self.joints),
                      "position": [message.position[index[n]] for n in self.joints],
                      "velocity": [message.velocity[index[n]] for n in self.joints]}
        except IndexError:
            return
        with self._lock:
            self._sample, self._received = sample, time.monotonic()

    def state(self):
        import copy
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
