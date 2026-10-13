"""A single ROS owner thread receives VR input and publishes Canvas status."""
import html
import json
import threading
import time
from uuid import uuid4

from agilex.piper.trajectory import unique
from common.teleop_contract import STATE_TOPIC, FEEDBACK_SCHEMA, MAX_WIRE_BYTES


class TeleopBus:
    def __init__(self, ros2, motion, input_topic):
        if ros2.ctx_core.get_domain_id() != 42:
            raise ValueError("teleop_requires_domain_42")
        self.context, self.motion = ros2.ctx_core, motion
        self.input_topic = input_topic
        self.closed, self.ready = threading.Event(), threading.Event()
        self.error = None
        self.last_received_ns = self.last_accepted_ns = None
        self.sequence = 0
        self.epoch = uuid4().hex
        self.thread = threading.Thread(target=self._run, name="piper-teleop-dds", daemon=True)
        self.thread.start()
        if not self.ready.wait(2) or self.error:
            self.closed.set()
            raise RuntimeError(self.error or "teleop_dds_start_timeout")

    def receive(self, message):
        try:
            if len(message.data.encode("utf-8")) > MAX_WIRE_BYTES:
                raise ValueError("teleop_packet_too_large")
            value = json.loads(message.data, object_pairs_hook=unique)
            if not isinstance(value, dict):
                raise ValueError("invalid_teleop_packet")
            self.last_received_ns = time.monotonic_ns()
            if self.motion.receive(value):
                self.last_accepted_ns = self.last_received_ns
                self.error = None
        except (ValueError, TypeError, RecursionError) as exc:
            self.error = str(exc)

    def feedback(self):
        self.sequence += 1
        state = self.motion.status()
        source = state["source"]
        return {"schema": FEEDBACK_SCHEMA, "instance_id": source[0] if source else "piper",
                "control_instance_id": "piper", "server_epoch": self.epoch, "sequence": self.sequence,
                "clock_id": self.motion.clock_id, "emitted_monotonic_ns": time.monotonic_ns(),
                "source_sequence": state["source_sequence"], "connection_epoch": source[2] if source else 0,
                "space_epoch": source[3] if source else 0, "operator_session_id": state["session_id"],
                "mapping_epoch": 0, "state": state["state"], "reason": state["reason"],
                "capabilities": ["dual_arm", "grippers", "trajectory_record", "trajectory_replay"],
                "execution": {**state, "output_active": state["output"], "hold_confirmed": False,
                              "stop_confirmed": False}, "receipts": [],
                "text": html.escape(f"PiPER 双臂：{state['state']}；{state['reason'] or '无'}"),
                "transport": self.status()}

    def status(self):
        return {"running": self.thread.is_alive() and not self.closed.is_set(), "error": self.error,
                "last_received_ns": self.last_received_ns, "last_accepted_ns": self.last_accepted_ns}

    def _run(self):
        node = executor = None
        try:
            from rclpy.node import Node
            from rclpy.executors import SingleThreadedExecutor
            from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
            from std_msgs.msg import String
            node = Node("piper_teleop", context=self.context)
            executor = SingleThreadedExecutor(context=self.context)
            qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE)
            publisher = node.create_publisher(String, STATE_TOPIC, qos)
            node.create_subscription(String, self.input_topic, self.receive, qos)
            executor.add_node(node)
            self.ready.set()
            next_publish = 0
            while not self.closed.is_set() and self.context.ok():
                executor.spin_once(timeout_sec=.02)
                if time.monotonic() >= next_publish:
                    message = String()
                    message.data = json.dumps(self.feedback(), allow_nan=False, separators=(",", ":"))
                    publisher.publish(message)
                    next_publish = time.monotonic() + .2
        except Exception as exc:
            self.error = str(exc)
            self.motion.request_stop("teleop_dds_failed")
        finally:
            self.ready.set()
            if executor is not None:
                executor.shutdown()
            if node is not None:
                node.destroy_node()

    def close(self):
        self.closed.set()
        self.thread.join(1)
        if self.thread.is_alive():
            raise RuntimeError("teleop_dds_still_stopping")
