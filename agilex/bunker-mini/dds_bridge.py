"""Explicit JSON topic bridge from the CAN-private DDS domain to Agent Core."""

from __future__ import annotations

import json


class BunkerDDSBridge:
    def __init__(self, ros2, namespace: str):
        from rclpy.node import Node
        from std_msgs.msg import String

        self._String = String
        self._ros2 = ros2
        self._robot = Node("bunker_mini_robot_bridge", context=ros2.ctx_robot)
        self._core = Node("bunker_mini_core_bridge", context=ros2.ctx_core)
        ros2.executor_robot.add_node(self._robot)
        ros2.executor_core.add_node(self._core)
        prefix = f"/{namespace}" if namespace else ""
        self._state_topic = f"{prefix}/state/motor_state"
        self._battery_topic = f"{prefix}/state/battery"
        self._remote_controller_topic = f"{prefix}/state/remote_controller"
        self._odom_topic = f"{prefix}/state/odom"
        self._robot_state = self._robot.create_publisher(String, self._state_topic, 10)
        self._robot_battery = self._robot.create_publisher(String, self._battery_topic, 10)
        self._robot_remote_controller = self._robot.create_publisher(
            String, self._remote_controller_topic, 10)
        self._robot_odom = self._robot.create_publisher(String, self._odom_topic, 10)
        self._core_state = self._core.create_publisher(String, self._state_topic, 10)
        self._core_battery = self._core.create_publisher(String, self._battery_topic, 10)
        self._core_remote_controller = self._core.create_publisher(
            String, self._remote_controller_topic, 10)
        self._core_odom = self._core.create_publisher(String, self._odom_topic, 10)
        self._robot_state_sub = self._robot.create_subscription(
            String, self._state_topic, self._forward(self._core_state), 10)
        self._robot_battery_sub = self._robot.create_subscription(
            String, self._battery_topic, self._forward(self._core_battery), 10)
        self._robot_remote_controller_sub = self._robot.create_subscription(
            String, self._remote_controller_topic,
            self._forward(self._core_remote_controller), 10)
        self._robot_odom_sub = self._robot.create_subscription(
            String, self._odom_topic, self._forward(self._core_odom), 10)

    def _forward(self, publisher):
        def callback(message):
            forwarded = self._String()
            forwarded.data = message.data
            publisher.publish(forwarded)
        return callback

    def _publish(self, publisher, value) -> None:
        message = self._String()
        message.data = json.dumps(value, separators=(",", ":"))
        publisher.publish(message)

    def publish_state(self, value: dict) -> None:
        self._publish(self._robot_state, value)

    def publish_battery(self, value: dict) -> None:
        self._publish(self._robot_battery, value)

    def publish_remote_controller(self, value: dict) -> None:
        self._publish(self._robot_remote_controller, value)

    def publish_odom(self, sample: dict) -> None:
        self._publish(self._robot_odom, sample)

    def close(self) -> None:
        self._ros2.executor_core.remove_node(self._core)
        self._ros2.executor_robot.remove_node(self._robot)
        self._core.destroy_node()
        self._robot.destroy_node()
