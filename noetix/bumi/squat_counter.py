"""Bumi processor card consuming Perception's lean pose JSON stream."""

import json
import threading
import time

from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from squat_count import SquatCounter


class SquatCounterPlugin:
    PREFIX = "squat_counter"

    def __init__(self, cfg, namespace, executor):
        self._lock = threading.RLock()
        self._node = Node(f"{namespace}_squat_counter")
        executor.add_node(self._node)
        self._output_topic = f"/{namespace}/squat/count"
        self._publisher = self._node.create_publisher(
            String, self._output_topic,
            QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE))
        self._subscription = None
        self._input_topic = ""
        self._last_warn = 0.0
        self._counter = SquatCounter(cfg.get("hold_s", 0.2),
                                     cfg.get("lost_s", 1.0),
                                     cfg.get("min_confidence", 0.5))

    def get_tool(self):
        return {
            "name": self.PREFIX, "type": "processor", "multiInstance": False,
            "description": "Count a person's complete squats using pose posture events",
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {"type": "string", "enum": ["start", "stop", "info", "reset"]},
                    "input_topic": {"type": "string", "description": "Connected pose /poses topic"},
                },
                "required": ["action"],
                "x-action-params": {
                    "start": {"params": ["input_topic"], "description": "Start a counting session"},
                    "stop": {"params": [], "description": "Stop and clear the session"},
                    "info": {"params": ["input_topic"], "description": "Report count and topics"},
                    "reset": {"params": [], "description": "Reset the count"},
                },
            },
            "topic_in": [{"format": "data/json", "desc": "pose postures"}],
            "topic_out": [{"topic": self._output_topic, "format": "data/json",
                           "desc": "one event per completed squat"}],
        }

    def _info(self, topic=""):
        topic = self._input_topic or topic
        return {
            "state": "running" if self._subscription else "idle",
            "count": self._counter.count, "track_id": self._counter.track_id,
            "phase": self._counter.phase,
            "topic_in": ([{"topic": topic, "format": "data/json"}] if topic else []),
            "topic_out": [{"topic": self._output_topic, "format": "data/json"}],
        }

    def start(self, input_topic=None):
        # Bundle.start_all() runs before the canvas has resolved its input.
        if input_topic is None:
            return self._info()
        if not isinstance(input_topic, str) or not input_topic.startswith("/") or not input_topic.endswith("/poses"):
            return {"state": "error", "error": "Connect the pose data/json output before starting"}
        with self._lock:
            if self._subscription is not None and self._input_topic != input_topic:
                self.stop()
            if self._subscription is None:
                self._counter.reset()
                self._subscription = self._node.create_subscription(
                    String, input_topic, self._on_pose,
                    QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
                self._input_topic = input_topic
            return self._info()

    def stop(self):
        with self._lock:
            if self._subscription is not None:
                self._node.destroy_subscription(self._subscription)
                self._subscription = None
            self._input_topic = ""
            self._counter.reset()
            return self._info()

    def dispatch(self, action, args):
        if action == "start":
            topic = args.get("input_topic") or (args.get("input_topics") or [None])[0]
            return self.start(topic or "")
        if action == "stop":
            return self.stop()
        with self._lock:
            if action == "info":
                return self._info(args.get("input_topic", ""))
            if action == "reset":
                self._counter.reset()
                return self._info()
        return None

    def _on_pose(self, msg):
        try:
            payload = json.loads(msg.data)
            if not isinstance(payload, dict):
                return
            with self._lock:
                if self._subscription is None:
                    return
                event = self._counter.update(payload.get("persons"), time.monotonic())
                if event is not None:
                    event["timestamp"] = time.time()
                    output = String()
                    output.data = json.dumps(event, ensure_ascii=False)
                    self._publisher.publish(output)
        except (ValueError, TypeError, KeyError, IndexError) as exc:
            now = time.monotonic()
            if now - self._last_warn >= 1.0:
                self._last_warn = now
                self._node.get_logger().warning(f"invalid pose frame (rate-limited): {exc}")
