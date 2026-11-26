"""Experimental G1 in-place follower decision layer for pose-like messages.

Dry-run by default. This module does not connect to the robot or send commands.
The input shape and coordinate sign are hypotheses until checked against the
live perception card. A future adapter must implement the commands returned
by ``ingest`` and arbitrate ownership of the base before issuing motion.
"""
import math
import time
import json
import threading


def pose2d_to_follow_frame(payload, received_at):
    """Convert the documented sensor/pose2d pixel coordinates to [-1, 1]."""
    size = payload.get("image_size")
    persons = payload.get("persons")
    if (not isinstance(size, list) or len(size) != 2 or
            not isinstance(size[0], (int, float)) or
            not math.isfinite(size[0]) or size[0] <= 0 or
            not isinstance(persons, list)):
        raise ValueError("invalid_pose2d")
    converted = []
    for person in persons:
        if not isinstance(person, dict):
            raise ValueError("invalid_person")
        box = person.get("bbox")
        if isinstance(box, list) and len(box) == 4 and all(
                isinstance(v, (int, float)) and math.isfinite(v) for v in box):
            center = (box[0] + box[2]) / 2
        else:
            points = person.get("keypoints")
            if not isinstance(points, list):
                raise ValueError("missing_person_position")
            xs = [p[0] for p in points if isinstance(p, list) and len(p) >= 2
                  and isinstance(p[0], (int, float)) and math.isfinite(p[0])
                  and (len(p) < 3 or isinstance(p[2], (int, float)) and p[2] > 0)]
            if not xs:
                raise ValueError("missing_person_position")
            center = (min(xs) + max(xs)) / 2
        x = 2 * center / size[0] - 1
        if not -1 <= x <= 1:
            raise ValueError("position_out_of_range")
        converted.append({"id": person.get("id", 0), "position": [x, 0]})
    return {"timestamp": received_at, "persons": converted}


class FollowCard:
    def __init__(self, clock=time.monotonic):
        self.clock = clock
        self.enabled = False
        self.target_id = None
        self.last_frame = None
        self.last_command = 0.0
        self.last_motion = False
        self.reason = "idle"

    def start(self):
        self.enabled = True
        self.target_id = None
        self.last_frame = None
        self.reason = "waiting_for_single_person"
        return {"command": "none", "reason": self.reason}

    def stop(self, reason="manual_stop"):
        self.enabled = False
        self.target_id = None
        self.reason = reason
        self.last_motion = False
        self.last_command = 0.0
        return {"command": "stop_move", "reason": reason}

    def ingest(self, payload):
        now = self.clock()
        if not self.enabled:
            return {"command": "none", "reason": self.reason}
        stamp = payload.get("timestamp")
        persons = payload.get("persons")
        if not isinstance(stamp, (float, int)) or not math.isfinite(stamp) or not isinstance(persons, list):
            return self.stop("invalid_pose")
        if self.last_frame is not None and stamp <= self.last_frame:
            return self.stop("stale_pose")
        self.last_frame = stamp
        if self.target_id is None:
            if len(persons) != 1:
                return {"command": "none", "reason": "waiting_for_single_person"}
            self.target_id = persons[0].get("id")
            if self.target_id is None:
                return self.stop("invalid_target")
        matches = [p for p in persons if p.get("id") == self.target_id]
        if len(matches) != 1:
            return self.stop("target_lost")
        position = matches[0].get("position")
        if not isinstance(position, list) or len(position) < 1 or not isinstance(position[0], (float, int)) or not math.isfinite(position[0]):
            return self.stop("invalid_position")
        x = float(position[0])  # Motus pose normalized horizontal position, +right.
        if abs(x) > 1:
            return self.stop("position_out_of_range")
        # Hysteresis: start correction beyond 0.18, stop inside 0.08.
        if abs(x) > .18:
            self.last_motion = True
        elif abs(x) < .08:
            self.last_motion = False
        if not self.last_motion:
            self.last_command = 0.0
            return {"command": "stop_move", "reason": "centered"}
        # Camera x is right-positive, G1 vyaw is left-positive.
        vyaw = -max(-.20, min(.20, x * .4))
        self.last_command = vyaw
        return {"command": "move", "vx": 0.0, "vy": 0.0,
                "vyaw": vyaw, "duration": .2, "reason": "following",
                "target_id": self.target_id}

    def tick(self, age_s):
        if self.enabled and age_s > .3:
            return self.stop("pose_timeout")
        return {"command": "none", "reason": self.reason}


class StationaryFollowPlugin:
    """Canvas card for pose2d wiring and decision preview; never sends motion."""

    PREFIX = "g1_stationary_follow"

    def __init__(self, config, namespace, executor, clock=time.monotonic):
        self._executor = executor
        self._clock = clock
        self._lock = threading.RLock()
        self._controller = FollowCard(clock=clock)
        self._node = None
        self._topic = ""
        self._last_received = None
        self._last_decision = {"command": "none", "reason": "idle"}

    def get_tool(self):
        return {
            "name": self.PREFIX, "type": "actuator", "multiInstance": False,
            "description": "G1 原地小跟班预览：连接人物姿态流，显示转向/停止判断；当前版本不向机器人发送运动指令。",
            "inputSchema": {
                "type": "object", "required": ["action"],
                "properties": {
                    "action": {"type": "string", "enum": ["start", "stop", "info"]},
                    "input_topic": {"type": "string", "description": "sensor/pose2d 人物姿态流话题"},
                },
                "x-action-params": {"info": {"params": []}},
                "x-resource": "base",
            },
            "topic_in": [{"format": "sensor/pose2d", "desc": "COCO-17 人物姿态流"}],
        }

    def start(self):
        # Bundle start does not arm a Canvas-controlled card.
        pass

    def stop(self):
        self._stop()

    def dispatch(self, action, args):
        if action == "start":
            topic = (args.get("input_topic") or "").strip()
            if not topic:
                topics = args.get("input_topics") or []
                topic = (topics[0] or "").strip() if topics else ""
            if not topic:
                return {"state": "error", "error": "请在画布连接 sensor/pose2d 输入"}
            with self._lock:
                if self._node is not None:
                    return {"state": "error", "error": "卡片已经启动"}
                try:
                    from rclpy.node import Node
                    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
                    from std_msgs.msg import String
                    node = Node("g1_stationary_follow")
                    qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                                     history=HistoryPolicy.KEEP_LAST, depth=1)
                    node.create_subscription(String, topic, self._on_message, qos)
                    self._executor.add_node(node)
                except Exception as exc:
                    if "node" in locals():
                        node.destroy_node()
                    return {"state": "error", "error": f"无法订阅人物姿态流: {exc}"}
                self._node = node
                self._topic = topic
                self._last_received = None
                self._last_decision = self._controller.start()
            return {"state": "running", "mode": "preview", "input": topic}
        if action == "stop":
            return self._stop()
        if action == "info":
            with self._lock:
                if self._last_received is not None:
                    watchdog = self._controller.tick(self._clock() - self._last_received)
                    if watchdog["command"] != "none":
                        self._last_decision = watchdog
                return {"state": "running" if self._node is not None else "idle",
                        "mode": "preview", "input": self._topic,
                        "last_decision": dict(self._last_decision)}
        return None

    def _on_message(self, message):
        with self._lock:
            if self._node is None:
                return
            received_at = self._clock()
            try:
                frame = pose2d_to_follow_frame(json.loads(message.data), received_at)
                self._last_decision = self._controller.ingest(frame)
            except (ValueError, TypeError, KeyError, json.JSONDecodeError):
                self._last_decision = self._controller.stop("invalid_pose2d")
            self._last_received = received_at

    def _stop(self):
        with self._lock:
            node, self._node = self._node, None
            self._topic = ""
            self._last_decision = self._controller.stop()
        if node is not None:
            self._executor.remove_node(node)
            node.destroy_node()
        return {"state": "idle", "mode": "preview"}
