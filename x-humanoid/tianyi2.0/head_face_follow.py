"""Tianyi head actuator for the perception face_tracking stream."""

from __future__ import annotations

import json
import math
import threading
import time

from rclpy.node import Node
from rclpy.exceptions import InvalidTopicNameException
from rclpy.validate_full_topic_name import validate_full_topic_name
from std_msgs.msg import String

FRAME_MAX_AGE = 0.4
COMMAND_PERIOD = 0.2
DEADBAND = 0.12
INPUT = {"format": "data/json", "desc": "face_tracking 输出"}


def _offset(value):
    if not isinstance(value, list) or len(value) != 2:
        return None
    if not all(type(v) in (int, float) and -1 <= v <= 1 for v in value):
        return None
    return value


class HeadFaceFollowPlugin:
    def __init__(self, ros2, head):
        self._ros2 = ros2
        self._head = head
        self._lock = threading.Lock()
        self._node = None
        self._topic = ""
        self._armed = False
        self._armed_at = 0.0
        self._last_frame = None
        self._last_command = -math.inf
        self._track = None
        self._reason = "idle"

    def get_tool(self):
        return {
            "name": "head_face_follow",
            "type": "actuator",
            "description": "天轶头部随动。连接 face_tracking，显式开启后跟随目标转头。",
            "topic_in": [INPUT],
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {"type": "string", "enum": ["start", "arm", "stop", "info"]},
                    "input_topic": {"type": "string", "description": "face_tracking ROS 话题"},
                },
                "required": ["action"],
                "x-resource": "head",
                "x-action-params": {
                    "start": {"params": ["input_topic"], "description": "接入跟踪结果，等待开启"},
                    "arm": {"params": [], "description": "开启头部随动"},
                    "stop": {"params": [], "description": "停止发送随动指令并断开订阅"},
                    "info": {"params": [], "description": "查看是否已开启及等待原因"},
                },
            },
        }

    def start(self):
        # Bundle lazy startup delegates subscription setup to dispatch.
        pass

    def owns_head(self):
        with self._lock:
            return self._armed

    def stop(self):
        with self._lock:
            node = self._node
            self._node = None
            self._topic = ""
            self._armed = False
            self._track = None
            self._last_frame = None
            self._reason = "idle"
            if node is not None:
                self._ros2.executor_core.remove_node(node)
                node.destroy_node()

    def dispatch(self, action, args):
        if action == "stop":
            self.stop()
            return {"state": "idle"}
        with self._lock:
            if action == "start":
                topic = args.get("input_topic")
                if not isinstance(topic, str) or not topic.startswith("/"):
                    return {"state": "error", "error": "Connect face_tracking first"}
                try:
                    validate_full_topic_name(topic)
                except (InvalidTopicNameException, ValueError) as error:
                    return {"state": "error", "error": str(error)}
                if self._node is not None and self._topic != topic:
                    return {"state": "error", "error": "Stop before changing the input topic"}
                if self._node is None:
                    node = Node("tianyi2_head_face_follow", context=self._ros2.ctx_core)
                    try:
                        node.create_subscription(String, topic, lambda msg: self._on_frame(node, msg), 1)
                        if not self._ros2.executor_core.add_node(node):
                            raise RuntimeError("Could not register face follow node")
                    except Exception as error:
                        node.destroy_node()
                        return {"state": "error", "error": str(error)}
                    self._node = node
                    self._topic = topic
                    self._reason = "awaiting_arm"
            elif action == "arm":
                if self._node is None:
                    return {"state": "error", "error": "Start the card first"}
                if not self._armed:
                    check = self._head.face_follow_ready()
                    if check:
                        return check
                    self._armed = True
                    self._armed_at = time.time()
                    self._reason = "waiting_for_face"
                    self._track = None
                    self._last_frame = None
            elif action != "info":
                return None
            reason = self._reason
            if self._armed and self._last_frame is not None:
                if time.time() - self._last_frame > FRAME_MAX_AGE:
                    reason = "tracking_timeout"
            return {
                "state": "running" if self._node else "idle",
                "armed": self._armed,
                "reason": reason,
                "topic_in": [{**INPUT, "topic": self._topic}] if self._topic else [],
            }

    def _on_frame(self, node, message):
        with self._lock:
            if self._node is not node or not self._armed:
                return
            try:
                packet = json.loads(message.data)
            except (ValueError, RecursionError):
                packet = None
            if not isinstance(packet, dict):
                self._track = None
                self._reason = "invalid_tracking_data"
                return
            frame_ts = packet.get("frame_ts")
            target = packet.get("target")
            if packet.get("state") != "tracking" or not isinstance(target, dict):
                self._track = None
                self._reason = "waiting_for_face"
                return

            age = time.time() - frame_ts if type(frame_ts) is float else math.inf
            track = target.get("track_id")
            offset = _offset(target.get("offset"))
            if (not 0 <= age <= FRAME_MAX_AGE or frame_ts < self._armed_at
                    or type(track) is not int or track <= 0 or offset is None):
                self._track = None
                self._reason = "invalid_or_stale_tracking"
                return
            last = self._last_frame
            if last is not None and frame_ts <= last:
                return
            confirmed = self._track == track and last is not None and frame_ts - last <= FRAME_MAX_AGE
            self._last_frame = frame_ts
            self._track = track
            if not confirmed:
                self._reason = "confirming_target"
                return

            dx, dy = (0.0 if abs(v) <= DEADBAND else v for v in offset)
            if not (dx or dy):
                self._reason = "centered"
                return
            now = time.monotonic()
            if now - self._last_command < COMMAND_PERIOD:
                return
            self._last_command = now
            result = self._head.follow_face(dx, dy)
            if result.get("state") == "error" or result.get("error"):
                self._armed = False
                self._reason = result.get("code", "head_command_failed")
            else:
                self._reason = result["state"]
