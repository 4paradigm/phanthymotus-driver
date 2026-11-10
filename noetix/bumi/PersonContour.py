#!/usr/bin/env python3
"""Contour-display card for the Bumi person detector.

Subscribes to the camera's colour/depth pair and to ``person_detect``'s target
JSON.  For every detected person it segments the aligned depth image around
that person's distance and draws the resulting silhouette in green on the live
camera frame.  The annotated frame is published as JPEG for the dashboard.

It is a display-only sensor card: no chassis commands, no extra detector.  The
person boxes come from ``person_detect``; depth is only used to turn a box into
an actual outline, so the green line follows the body instead of a rectangle.

The depth segmentation, parsing and drawing helpers are pure and testable
without rclpy; ROS is imported in the plugin constructor.
"""

from __future__ import annotations

import json
import threading
import time
import zlib
from array import array

import numpy as np

try:
    import cv2
except ImportError:  # tests can still import the pure helpers
    cv2 = None

MIN_PERSON_DISTANCE_M = 0.30
MAX_PERSON_DISTANCE_M = 10.0
_DEFAULT_TOLERANCE_M = 0.40
_DEFAULT_LINE_WIDTH = 2
_DEFAULT_DISPLAY_HZ = 10.0
_MAX_TARGET_AGE_S = 1.0


def parse_people(payload) -> list:
    """Return [{bbox, distance_m}] from a person_detect JSON payload."""
    if not isinstance(payload, dict):
        return []
    raw = payload.get("people")
    if not isinstance(raw, list):
        single = payload.get("person")
        raw = [single] if isinstance(single, dict) else []
    people = []
    for item in raw:
        if not isinstance(item, dict):
            continue
        bbox = item.get("bbox")
        distance = item.get("distance_m")
        if (not isinstance(bbox, (list, tuple)) or len(bbox) != 4
                or not all(isinstance(value, (int, float)) for value in bbox)):
            continue
        try:
            x, y, w, h = (int(value) for value in bbox)
            distance = float(distance)
        except (TypeError, ValueError):
            continue
        if w <= 0 or h <= 0 or not np.isfinite(distance):
            continue
        if not MIN_PERSON_DISTANCE_M <= distance <= MAX_PERSON_DISTANCE_M:
            continue
        people.append({"bbox": [x, y, w, h], "distance_m": distance})
    return people


def person_mask(depth_mm, bbox, distance_m: float,
                tolerance_m: float = _DEFAULT_TOLERANCE_M):
    """Binary mask of the depth band around one person inside its box."""
    if cv2 is None or depth_mm is None:
        return None
    height, width = depth_mm.shape[:2]
    x, y, w, h = (int(value) for value in bbox)
    x = max(0, min(x, width - 1))
    y = max(0, min(y, height - 1))
    w = max(1, min(w, width - x))
    h = max(1, min(h, height - y))
    target = float(distance_m) * 1000.0
    tolerance = max(0.05, float(tolerance_m)) * 1000.0
    crop = depth_mm[y:y + h, x:x + w].astype(np.int32)
    mask = ((crop >= target - tolerance) & (crop <= target + tolerance)
            & (crop > 0)).astype(np.uint8) * 255
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,
                            np.ones((3, 3), np.uint8), iterations=1)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE,
                            np.ones((5, 5), np.uint8), iterations=2)
    return mask


def contours_from_mask(mask, bbox, min_area_ratio: float = 0.02) -> list:
    """External contours of a local mask, shifted into full-frame coordinates."""
    if cv2 is None or mask is None:
        return []
    x, y, w, h = (int(value) for value in bbox)
    min_area = max(100.0, min_area_ratio * w * h)
    found = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                             cv2.CHAIN_APPROX_SIMPLE)[0]
    contours = []
    for contour in found:
        if cv2.contourArea(contour) < min_area:
            continue
        contours.append(contour + np.array([[x, y]], dtype=contour.dtype))
    return contours


def draw_people(frame, people: list, depth_mm,
                tolerance_m: float = _DEFAULT_TOLERANCE_M,
                line_width: int = _DEFAULT_LINE_WIDTH):
    """Draw green depth contours for each person on the BGR frame in place."""
    if cv2 is None:
        return frame
    for person in people:
        bbox = person["bbox"]
        distance = person["distance_m"]
        mask = person_mask(depth_mm, bbox, distance, tolerance_m)
        contours = contours_from_mask(mask, bbox)
        if contours:
            cv2.drawContours(frame, contours, -1, (0, 255, 0),
                             line_width, cv2.LINE_AA)
        else:
            # If depth is too noisy, fall back to a green box so the operator
            # still sees which person the detector is tracking.
            x, y, w, h = bbox
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0),
                          line_width)
        x, y, _w, _h = bbox
        label = f"person {distance:.1f} m"
        cv2.putText(frame, label, (x, max(16, y - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1,
                    cv2.LINE_AA)
    return frame


def encode_jpeg(frame, quality: int = 80):
    if cv2 is None:
        return None
    ok, encoded = cv2.imencode(
        ".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, int(quality)])
    return encoded.tobytes() if ok else None


class PersonContourPlugin:
    """MCP sensor card: ``person_contour`` — camera view with green outlines."""

    PREFIX = "person_contour"

    def __init__(self, plugin_config: dict, namespace: str, executor):
        plugin_config = plugin_config or {}
        self._namespace = namespace
        self._target_topic = f"/{namespace}/person/target"
        self._color_topic = f"/{namespace}/camera/color"
        self._depth_topic = f"/{namespace}/camera/depth"
        self._output_topic = f"/{namespace}/person/contour"
        try:
            self._display_hz = min(15.0, max(2.0, float(
                plugin_config.get("display_hz", _DEFAULT_DISPLAY_HZ))))
        except (TypeError, ValueError):
            self._display_hz = _DEFAULT_DISPLAY_HZ
        try:
            self._tolerance_m = max(0.05, float(
                plugin_config.get("depth_tolerance_m", _DEFAULT_TOLERANCE_M)))
        except (TypeError, ValueError):
            self._tolerance_m = _DEFAULT_TOLERANCE_M
        try:
            self._line_width = max(1, min(6, int(
                plugin_config.get("line_width", _DEFAULT_LINE_WIDTH))))
        except (TypeError, ValueError):
            self._line_width = _DEFAULT_LINE_WIDTH
        try:
            self._jpeg_quality = max(30, min(95, int(
                plugin_config.get("jpeg_quality", 80))))
        except (TypeError, ValueError):
            self._jpeg_quality = 80
        self._people = []
        self._people_at = 0.0
        self._color = None
        self._depth = None
        self._color_seq = 0
        self._last_seq = -1
        self._lock = threading.RLock()
        self._sub = None
        self._subscription_topic = self._target_topic
        self._frames = 0

        if cv2 is None:
            raise RuntimeError("OpenCV is required by person_contour")

        import rclpy
        from rclpy.node import Node
        from rclpy.qos import (QoSProfile, ReliabilityPolicy, HistoryPolicy,
                               DurabilityPolicy)
        from sensor_msgs.msg import CompressedImage
        from std_msgs.msg import String

        self._String = String
        self._CompressedImage = CompressedImage
        self._qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                               history=HistoryPolicy.KEEP_LAST, depth=5,
                               durability=DurabilityPolicy.VOLATILE)
        self._node = Node("bumi_person_contour")
        executor.add_node(self._node)
        self._pub = self._node.create_publisher(
            CompressedImage, self._output_topic, self._qos)
        self._node.create_subscription(
            CompressedImage, self._color_topic, self._on_color, self._qos)
        self._node.create_subscription(
            CompressedImage, self._depth_topic, self._on_depth, self._qos)
        self._subscribe(self._target_topic)
        self._node.create_timer(1.0 / self._display_hz, self._tick)
        print(f"[person_contour] target←{self._target_topic} "
              f"image→{self._output_topic}", flush=True)

    def get_tool(self) -> dict:
        return {
            "name": "person_contour",
            "type": "sensor",
            "multiInstance": False,
            "description": (
                "显示机器人摄像头画面，并把 person_detect 检测到的人形用绿色轮廓圈出来。"
                "轮廓来自对齐深度图在人物距离附近的分割；深度不可靠时退回绿色矩形。"
                "只显示，不移动机器人。连接到 person_detect 的目标输出后使用。"
            ),
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {
                        "type": "string",
                        "enum": ["info", "start", "stop"],
                        "description": "info=查看画面状态；start/stop=状态应答。",
                    },
                },
            },
            "topic_in": [{"format": "data/json",
                          "message_type": "std_msgs/msg/String"}],
            "topic_out": [{"topic": self._output_topic,
                           "format": "image/jpeg"}],
        }

    def get_tools(self) -> list:
        return [self.get_tool()]

    def start(self, args: dict | None = None):
        args = args or {}
        topic = str(args.get("input_topic") or "").strip()
        self._subscribe(topic or self._target_topic)
        return {"state": "running", "topic_in": [{
            "topic": self._subscription_topic,
            "format": "data/json",
            "message_type": "std_msgs/msg/String",
        }]}

    def stop(self):
        # Display cards keep the camera window alive while control stops.
        return {"state": "idle"}

    def dispatch(self, action: str, args: dict) -> dict | None:
        if action == "start":
            return self.start(args)
        if action == "stop":
            return self.stop()
        if action == "info":
            with self._lock:
                people = len(self._people)
                age = (time.monotonic() - self._people_at
                       if self._people_at else None)
                color_age = (time.monotonic() - self._color[1]
                             if self._color else None)
                depth_age = (time.monotonic() - self._depth[1]
                             if self._depth else None)
                frames = self._frames
            return {
                "state": "running",
                "target_topic": self._subscription_topic,
                "output_topic": self._output_topic,
                "target_age_s": round(age, 2) if age is not None else None,
                "color_age_s": round(color_age, 2) if color_age is not None else None,
                "depth_age_s": round(depth_age, 2) if depth_age is not None else None,
                "people": people,
                "frames": frames,
                "display_hz": self._display_hz,
            }
        return None

    # ── Inputs ───────────────────────────────────────────────────────────────

    def _subscribe(self, topic: str) -> None:
        if not topic:
            return
        if self._sub is not None and topic == self._subscription_topic:
            return
        if self._sub is not None:
            try:
                self._node.destroy_subscription(self._sub)
            except Exception:
                pass
            self._sub = None
        self._sub = self._node.create_subscription(
            self._String, topic, self._on_target, self._qos)
        self._subscription_topic = topic

    def _on_target(self, msg) -> None:
        try:
            payload = json.loads(msg.data)
        except Exception:
            return
        people = parse_people(payload)
        with self._lock:
            self._people = people
            self._people_at = time.monotonic()

    def _on_color(self, msg) -> None:
        if not getattr(msg, "format", "").startswith("jpeg"):
            return
        with self._lock:
            self._color_seq += 1
            self._color = (bytes(msg.data), time.monotonic(), self._color_seq)

    def _on_depth(self, msg) -> None:
        if "16UC1" not in getattr(msg, "format", ""):
            return
        with self._lock:
            self._depth = (bytes(msg.data), time.monotonic())

    # ── Render ───────────────────────────────────────────────────────────────

    def _tick(self) -> None:
        now = time.monotonic()
        with self._lock:
            color = self._color
            depth = self._depth
            people = list(self._people) if (
                self._people_at and now - self._people_at <= _MAX_TARGET_AGE_S
            ) else []
        if color is None or now - color[1] > 1.5:
            return
        with self._lock:
            if color[2] == self._last_seq:
                return
            self._last_seq = color[2]
        frame = cv2.imdecode(np.frombuffer(color[0], np.uint8),
                             cv2.IMREAD_COLOR)
        if frame is None:
            return
        depth_mm = None
        if depth is not None and now - depth[1] <= 1.5:
            try:
                raw = zlib.decompress(depth[0])
                expected = frame.shape[0] * frame.shape[1]
                values = np.frombuffer(raw, dtype=np.uint16)
                if values.size == expected:
                    depth_mm = values.reshape(frame.shape[0], frame.shape[1])
            except Exception:
                depth_mm = None

        draw_people(frame, people, depth_mm, self._tolerance_m,
                    self._line_width)
        if not people:
            cv2.putText(frame, "no person", (12, 24),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 1,
                        cv2.LINE_AA)
        encoded = encode_jpeg(frame, self._jpeg_quality)
        if encoded is None:
            return
        msg = self._CompressedImage()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.format = "jpeg"
        msg.data = array("B", encoded)
        self._pub.publish(msg)
        with self._lock:
            self._frames += 1
