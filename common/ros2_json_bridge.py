"""Configurable ROS2 JSON command/ack bridge used before private IDLs arrive."""

from __future__ import annotations

import json
import threading
import time
import uuid

from .vendor_runtime import jsonable


class JsonCommandBridge:
    """Publish correlated commands and retain supplier state/acknowledgements.

    The wire format is deliberately simple and documented: suppliers can bridge
    it immediately, while a later typed adapter can preserve the public MCP API.
    """

    def __init__(self, config, namespace, ros2, model_key):
        from rclpy.node import Node
        from std_msgs.msg import String

        self.config = config
        self.model_key = model_key
        self.robot = Node(f"{model_key}_driver_robot", context=ros2.ctx_robot)
        self.core = Node(f"{model_key}_driver_core", context=ros2.ctx_core)
        ros2.executor_robot.add_node(self.robot)
        ros2.executor_core.add_node(self.core)
        topics = config.get("topics", {})
        self.command_pub = self.robot.create_publisher(String, topics.get("command", f"/{model_key}/command"), 10)
        self.state_topic = f"/{namespace}/{model_key}/state"
        self.state_pub = self.core.create_publisher(String, self.state_topic, 10)
        self.lock = threading.Lock()
        self.state = {"state": "waiting_for_supplier_state"}
        self.acks = {}
        self.last_command = None
        self.closed = False
        self.robot.create_subscription(String, topics.get("state", f"/{model_key}/state"), self._on_state, 10)
        self.robot.create_subscription(String, topics.get("ack", f"/{model_key}/ack"), self._on_ack, 10)

    @staticmethod
    def _decode(msg):
        try:
            return json.loads(msg.data)
        except Exception:
            return {"raw": msg.data}

    def _on_state(self, msg):
        from std_msgs.msg import String
        data = self._decode(msg)
        with self.lock:
            self.state = data
        out = String(); out.data = json.dumps(data, ensure_ascii=False)
        self.state_pub.publish(out)

    def _on_ack(self, msg):
        data = self._decode(msg)
        correlation_id = str(data.get("id", ""))
        if correlation_id:
            with self.lock:
                self.acks[correlation_id] = data

    def publish(self, command, params=None):
        from std_msgs.msg import String
        envelope = {
            "id": str(uuid.uuid4()), "command": str(command),
            "params": jsonable(params or {}), "timestamp": time.time(),
        }
        msg = String(); msg.data = json.dumps(envelope, ensure_ascii=False)
        self.command_pub.publish(msg)
        with self.lock:
            self.last_command = envelope
        return {"state": "awaiting_supplier_ack", **envelope}

    def snapshot(self):
        with self.lock:
            return {"supplier": jsonable(self.state), "last_command": jsonable(self.last_command)}

    def acknowledgement(self, correlation_id):
        with self.lock:
            return jsonable(self.acks.get(str(correlation_id), {"state": "pending", "id": correlation_id}))

    def graph(self):
        return {
            "topics": self.robot.get_topic_names_and_types(),
            "services": self.robot.get_service_names_and_types(),
            "actions": self.robot.get_action_names_and_types(),
            "nodes": self.robot.get_node_names_and_namespaces(),
        }

    def close(self):
        if self.closed: return
        self.closed = True; self.robot.destroy_node(); self.core.destroy_node()
