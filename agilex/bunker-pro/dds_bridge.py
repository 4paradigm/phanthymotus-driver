"""Explicit JSON topic bridge between the Agent Core and CAN-private DDS domains."""

from __future__ import annotations

import json


class BunkerDDSBridge:
    def __init__(self, ros2, namespace: str):
        from rclpy.node import Node
        from std_msgs.msg import String

        self._String = String
        self._ros2 = ros2
        self._robot = Node("bunker_pro_robot_bridge", context=ros2.ctx_robot)
        self._core = Node("bunker_pro_core_bridge", context=ros2.ctx_core)
        ros2.executor_robot.add_node(self._robot)
        ros2.executor_core.add_node(self._core)
        prefix = f"/{namespace}" if namespace else ""
        self._command_topic = f"{prefix}/control/velocity"
        self._state_topic = f"{prefix}/state/base"
        self._odom_topic = f"{prefix}/state/odom"
        self._robot_cmd = self._robot.create_publisher(String, self._command_topic, 10)
        self._robot_state = self._robot.create_publisher(String, self._state_topic, 10)
        self._robot_odom = self._robot.create_publisher(String, self._odom_topic, 10)
        self._core_state = self._core.create_publisher(String, self._state_topic, 10)
        self._core_odom = self._core.create_publisher(String, self._odom_topic, 10)
        self._robot_state_sub = self._robot.create_subscription(
            String, self._state_topic, self._on_state, 10)
        self._robot_odom_sub = self._robot.create_subscription(
            String, self._odom_topic, self._on_odom, 10)
        self._core_cmd = None

    def bind_control_topic(self, topic: str | None) -> None:
        if self._core_cmd is not None:
            self._core.destroy_subscription(self._core_cmd)
            self._core_cmd = None
        if topic:
            self._core_cmd = self._core.create_subscription(
                self._String, topic, self._on_command, 10)

    def _on_command(self, message):
        # Republish by value. A domain bridge must never share ROS entities.
        forwarded = self._String()
        forwarded.data = message.data
        self._robot_cmd.publish(forwarded)

    def _on_state(self, message):
        forwarded = self._String()
        forwarded.data = message.data
        self._core_state.publish(forwarded)

    def _on_odom(self, message):
        forwarded = self._String()
        forwarded.data = message.data
        self._core_odom.publish(forwarded)

    def publish_state(self, value: dict) -> None:
        message = self._String()
        message.data = json.dumps(value, separators=(",", ":"))
        self._robot_state.publish(message)

    def publish_odom(self, value: dict) -> None:
        message = self._String()
        message.data = json.dumps(value, separators=(",", ":"))
        self._robot_odom.publish(message)

    def close(self) -> None:
        self._ros2.executor_core.remove_node(self._core)
        self._ros2.executor_robot.remove_node(self._robot)
        self._core.destroy_node()
        self._robot.destroy_node()
