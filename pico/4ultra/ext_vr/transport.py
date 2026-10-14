"""Local DDS input publisher and read-only robot feedback subscription."""

import json
import copy
import threading
from common.teleop_contract import COMMAND_TOPIC, STATE_TOPIC, MAX_WIRE_BYTES


class BoundedWriter:
    """One potentially blocking DDS write and one latest input frame."""

    def __init__(self, send, cleanup=lambda: None):
        self.send, self.cleanup = send, cleanup
        self.condition = threading.Condition()
        self.latest = None
        self.closed = False
        self.last_error = None
        self.thread = threading.Thread(
            target=self._run, name="pico-dds-writer", daemon=True
        )
        self.thread.start()

    def publish(self, value):
        with self.condition:
            if self.closed:
                raise ValueError("device_transport_closed")
            self.latest = copy.deepcopy(value)
            self.condition.notify()

    def _run(self):
        try:
            while True:
                with self.condition:
                    self.condition.wait_for(
                        lambda: self.closed or self.latest is not None
                    )
                    if self.closed:
                        return
                    value, self.latest = self.latest, None
                try:
                    self.send(value)
                except Exception as exc:
                    self.last_error = type(exc).__name__
        finally:
            self.cleanup()

    def close(self):
        with self.condition:
            self.closed = True
            self.latest = None
            self.condition.notify()
        # A stalled middleware cannot delay lifecycle/stop indefinitely. Its
        # isolated writer owns eventual destruction, avoiding publish/free races.
        self.thread.join(timeout=0.25)


class RosTransport:
    def __init__(self, ros2, namespace, instance_id, feedback_callback):
        from rclpy.node import Node
        from rclpy.qos import (
            QoSProfile,
            ReliabilityPolicy,
            HistoryPolicy,
            DurabilityPolicy,
        )
        from std_msgs.msg import String

        self.String = String
        self.executor = getattr(ros2, "executor_core", ros2)
        context = getattr(ros2, "ctx_core", None) or getattr(
            self.executor, "context", None
        )
        if context is None or context.get_domain_id() != 42:
            raise ValueError("teleop_device_requires_local_domain_42")
        self.node = Node(
            "teleop_device_" + instance_id.replace("-", "_"), context=context
        )
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=16,
            durability=DurabilityPolicy.VOLATILE,
        )
        command = COMMAND_TOPIC
        self.pub = self.node.create_publisher(String, command, qos)

        def receive(message):
            try:
                if len(message.data.encode("utf-8")) > MAX_WIRE_BYTES:
                    raise ValueError("feedback_too_large")
                value = json.loads(message.data)
            except (ValueError, TypeError, RecursionError):
                value = {}
            if feedback_callback is not None:
                feedback_callback(value)

        self.sub = self.node.create_subscription(String, STATE_TOPIC, receive, qos)
        self.executor.add_node(self.node)
        self.writer = BoundedWriter(self._send, self._cleanup)

    def publish(self, value):
        self.writer.publish(value)

    def _send(self, value):
        msg = self.String()
        msg.data = json.dumps(value, allow_nan=False, separators=(",", ":"))
        self.pub.publish(msg)

    def close(self):
        self.writer.close()

    def _cleanup(self):
        self.executor.remove_node(self.node)
        self.node.destroy_node()
