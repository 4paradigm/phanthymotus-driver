"""Bumi ROS/MCP card for local squat counting from pose skeleton frames.

The counter runs at camera speed in this driver. Decision Core receives only
sparse progress events and can read the authoritative total with ``status``.
"""

from __future__ import annotations

import json
import math
import threading
import time

from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from squat_logic import DEFAULT_CONFIG, SquatCounter


_POSE_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    durability=DurabilityPolicy.VOLATILE,
)
_OUTPUT_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=20,
    durability=DurabilityPolicy.VOLATILE,
)
_DEBUG_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=2,
    durability=DurabilityPolicy.VOLATILE,
)


def _nonnegative_interval(config: dict, name: str, default: float) -> float:
    value = config.get(name, default)
    if (isinstance(value, bool) or not isinstance(value, (int, float))
            or not math.isfinite(value) or value < 0):
        raise ValueError(f"{name} must be a nonnegative finite number")
    return float(value)


class SquatCounterPlugin:
    """One local squat session, independent of the model's response speed."""

    def __init__(self, plugin_config: dict, namespace: str, executor):
        self._namespace = namespace
        self._counter = SquatCounter({
            key: value for key, value in plugin_config.items() if key in DEFAULT_CONFIG
        })
        self._status_interval_s = _nonnegative_interval(
            plugin_config, "status_interval_s", 0.5)
        self._debug_interval_s = _nonnegative_interval(
            plugin_config, "debug_interval_s", 0.5)
        self._min_rep_event_interval_s = _nonnegative_interval(
            plugin_config, "min_rep_event_interval_s", 3.0)
        self._input_stale_after_s = _nonnegative_interval(
            plugin_config, "input_stale_after_s", 1.5)
        self._event_topic = f"/{namespace}/fitness/squat/events"
        self._status_topic = f"/{namespace}/fitness/squat/status"
        self._debug_topic = f"/{namespace}/fitness/squat/debug"
        self._node = Node("bumi_squat_counter")
        executor.add_node(self._node)
        self._event_pub = self._node.create_publisher(
            String, self._event_topic, _OUTPUT_QOS)
        self._status_pub = self._node.create_publisher(
            String, self._status_topic, _OUTPUT_QOS)
        self._debug_pub = self._node.create_publisher(
            String, self._debug_topic, _DEBUG_QOS)
        self._subscription = None
        self._input_topic = ""
        self._lock = threading.RLock()
        self._pending_rep: dict | None = None
        self._last_rep_published_at: float | None = None
        self._last_status_published_at = 0.0
        self._last_debug_published_at = 0.0
        self._last_input_at: float | None = None
        self._frames_received = 0
        self._frames_parsed = 0
        self._invalid_frames = 0
        self._last_error: str | None = None
        self._node.create_timer(0.2, self._tick)

    def get_tool(self) -> dict:
        return {
            "name": "squat_counter",
            "type": "processor",
            "multiInstance": False,
            "description": (
                "Count one selected person's complete squats from the full pose "
                "skeleton stream. Use status for the current total; connect only "
                "the sparse events output to Decision Core. Status and debug "
                "outputs are for the dashboard."
            ),
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {"type": "string", "enum": [
                        "start", "stop", "info", "status", "debug", "reset"]},
                    "input_topic": {
                        "type": "string",
                        "description": "Full pose /poses/skeleton ROS topic from the canvas connection",
                    },
                },
                "required": ["action"],
                "x-action-params": {
                    "start": {"params": ["input_topic"],
                              "description": "Subscribe to skeleton frames and start a new count"},
                    "stop": {"params": [], "description": "Stop subscribing; retain final total"},
                    "info": {"params": [], "description": "Show card ports and status"},
                    "status": {"params": [], "description": "Read the authoritative current total"},
                    "debug": {"params": [], "description": "Read latest pose measurements and count checks"},
                    "reset": {"params": [], "description": "Clear the count and start a new session"},
                },
            },
            "topic_in": [{
                "format": "sensor/pose2d",
                "message_type": "std_msgs/msg/String",
                "desc": "Full pose /poses/skeleton JSON with COCO keypoints",
            }],
            "topic_out": self._topic_out(),
        }

    def _topic_out(self) -> list[dict]:
        return [
            {"topic": self._event_topic, "format": "data/json",
             "message_type": "std_msgs/msg/String",
             "desc": "Sparse session, target, and rep events for Decision Core"},
            {"topic": self._status_topic, "format": "data/json",
             "message_type": "std_msgs/msg/String",
             "desc": "Current count for a dashboard; do not connect to Decision Core"},
            {"topic": self._debug_topic, "format": "data/json",
             "message_type": "std_msgs/msg/String",
             "desc": "Pose measurements and threshold checks for dashboard debugging"},
        ]

    def start(self) -> None:
        # Bundle startup registers the card; the canvas supplies input_topic
        # later through dispatch("start", ...).
        pass

    def stop(self) -> None:
        with self._lock:
            self._stop_locked()

    def dispatch(self, action: str, args: dict) -> dict | None:
        args.pop("_tool_name", None)
        with self._lock:
            if action == "start":
                return self._start_locked(args.get("input_topic"))
            if action == "stop":
                self._stop_locked()
                return self._status_locked()
            if action == "reset":
                self._pending_rep = None
                self._last_rep_published_at = None
                self._counter.reset()
                self._publish_status_locked(force=True)
                self._publish_debug_locked(force=True)
                return self._status_locked()
            if action == "status":
                return self._status_locked()
            if action == "debug":
                return self._debug_locked()
            if action == "info":
                info = self._status_locked()
                info["topic_in"] = [{
                    "format": "sensor/pose2d",
                    "message_type": "std_msgs/msg/String",
                    **({"topic": self._input_topic} if self._input_topic else {}),
                }]
                info["topic_out"] = self._topic_out()
                info["config"] = dict(self._counter.config)
                return info
        return None

    def _start_locked(self, topic: object) -> dict:
        if not isinstance(topic, str) or not topic.strip():
            return {"state": "error", "error": "input_topic is required"}
        topic = topic.strip()
        if self._subscription is not None and topic == self._input_topic:
            return self._status_locked()
        try:
            new_sub = self._node.create_subscription(
                String, topic,
                lambda message, source=topic: self._on_pose(message, source),
                _POSE_QOS,
            )
        except Exception as exc:
            self._last_error = str(exc)
            return {"state": "error", "error": self._last_error}
        if self._subscription is not None:
            self._node.destroy_subscription(self._subscription)
        self._subscription = new_sub
        self._input_topic = topic
        self._counter.reset()
        self._pending_rep = None
        self._last_rep_published_at = None
        self._last_input_at = time.monotonic()
        self._frames_received = 0
        self._frames_parsed = 0
        self._invalid_frames = 0
        self._last_error = None
        self._publish_status_locked(force=True)
        self._publish_debug_locked(force=True)
        return self._status_locked()

    def _stop_locked(self) -> None:
        if self._subscription is not None:
            self._node.destroy_subscription(self._subscription)
            self._subscription = None
        self._input_topic = ""
        # If the model was busy, leave it one latest absolute count before
        # stopping; the final count is also available through status.
        if self._pending_rep is not None:
            self._publish_event_locked(self._pending_rep)
            self._pending_rep = None
        self._publish_status_locked(force=True)
        self._publish_debug_locked(force=True)

    def _on_pose(self, message: String, source: str) -> None:
        with self._lock:
            # A queued callback from a previous subscription must not mutate
            # the new session after switching inputs or stopping.
            if self._subscription is None or source != self._input_topic:
                return
            self._frames_received += 1
            try:
                payload = json.loads(message.data)
                if not isinstance(payload, dict):
                    raise ValueError("skeleton payload must be a JSON object")
                events = self._counter.process(payload)
                self._frames_parsed += 1
            except (ValueError, TypeError, KeyError) as exc:
                self._invalid_frames += 1
                self._last_error = str(exc)
                if self._invalid_frames == 1 or self._invalid_frames % 100 == 0:
                    self._node.get_logger().warn(
                        f"Squat counter ignored {self._invalid_frames} invalid frame(s): {exc}")
                return
            except Exception as exc:
                self._invalid_frames += 1
                self._last_error = str(exc)
                self._node.get_logger().error(f"Squat counter frame error: {exc}")
                return
            now = time.monotonic()
            self._last_input_at = now
            self._last_error = None
            for event in events:
                self._handle_event_locked(event, now)
            self._publish_status_locked(now)
            self._publish_debug_locked(now)

    def _handle_event_locked(self, event: dict, now: float) -> None:
        kind = event.get("type")
        if kind == "rep_completed":
            if (self._last_rep_published_at is None
                    or now - self._last_rep_published_at
                    >= self._min_rep_event_interval_s):
                self._publish_event_locked(event)
                self._last_rep_published_at = now
                self._pending_rep = None
            else:
                # Keep only the newest absolute count during a busy model turn.
                self._pending_rep = event
            return
        if kind in ("session_started", "target_changed", "target_lost"):
            self._pending_rep = None
        if kind in ("session_started", "target_changed"):
            self._last_rep_published_at = None
        self._publish_event_locked(event)

    def _publish_event_locked(self, event: dict) -> None:
        payload = {"source": "squat_counter", "exercise": "squat",
                   "priority": 1, **event}
        message = String()
        message.data = json.dumps(payload, ensure_ascii=False, allow_nan=False)
        self._event_pub.publish(message)

    def _status_locked(self) -> dict:
        counter = self._counter.snapshot()
        age = (None if self._last_input_at is None
               else max(0.0, time.monotonic() - self._last_input_at))
        return {
            "source": "squat_counter",
            "exercise": "squat",
            "state": "running" if self._subscription is not None else "idle",
            "phase": counter.pop("state"),
            **counter,
            "input_topic": self._input_topic or None,
            "last_frame_age_s": None if age is None else round(age, 3),
            "input_stale": (self._subscription is not None and age is not None
                            and age >= self._input_stale_after_s),
            "frames_received": self._frames_received,
            "frames_parsed": self._frames_parsed,
            "invalid_frames": self._invalid_frames,
            "last_error": self._last_error,
        }

    def _debug_locked(self) -> dict:
        # One-level groups make each metric visible as its own cell in the
        # dashboard's Latest JSON view. Do not include raw pose keypoints.
        diagnostics = dict(self._counter.diagnostics())
        diagnostics.pop("state", None)
        return {**self._status_locked(), **diagnostics}

    def _publish_status_locked(self, now: float | None = None,
                               *, force: bool = False) -> None:
        now = time.monotonic() if now is None else now
        if (not force and now - self._last_status_published_at
                < self._status_interval_s):
            return
        message = String()
        message.data = json.dumps(self._status_locked(), ensure_ascii=False,
                                  allow_nan=False)
        self._status_pub.publish(message)
        self._last_status_published_at = now

    def _publish_debug_locked(self, now: float | None = None,
                              *, force: bool = False) -> None:
        now = time.monotonic() if now is None else now
        if (not force and now - self._last_debug_published_at
                < self._debug_interval_s):
            return
        message = String()
        message.data = json.dumps(self._debug_locked(), ensure_ascii=False,
                                  allow_nan=False)
        self._debug_pub.publish(message)
        self._last_debug_published_at = now

    def _tick(self) -> None:
        with self._lock:
            now = time.monotonic()
            if (self._pending_rep is not None
                    and (self._last_rep_published_at is None
                         or now - self._last_rep_published_at
                         >= self._min_rep_event_interval_s)):
                self._publish_event_locked(self._pending_rep)
                self._pending_rep = None
                self._last_rep_published_at = now
            if self._subscription is not None:
                self._publish_status_locked(now)
                self._publish_debug_locked(now)
