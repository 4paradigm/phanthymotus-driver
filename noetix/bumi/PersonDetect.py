#!/usr/bin/env python3
"""Human-shape detector card for the Noetix Bumi bundle.

A sensor card: it never commands the chassis.  It consumes the aligned
colour/depth pair published by the camera card, runs OpenCV's HOG + linear SVM
full-body person detector, checks the depth and box geometry, confirms the
nearest person over several frames and publishes their distance and bearing for
a downstream follow/approach card.

HOG is used instead of colour or contour heuristics: a person is not a simple
silhouette, and the default OpenCV people detector ships with the OpenCV package
already in the image, so no model download or runtime network is required.
The detector is downscaled for speed on the Bumi ARM board and boxes are mapped
back to the full-resolution frame before depth sampling.

ROS is imported in the plugin constructor; the geometry, NMS and tracker helpers
are pure and testable without rclpy.
"""

from __future__ import annotations

import json
import math
import threading
import time
import zlib
from array import array

import numpy as np

try:
    import cv2
except ImportError:  # tests can still import the pure helpers
    cv2 = None

from camera_specs import SPECS

MAX_DISTANCE_M = 8.0
MIN_DISTANCE_M = 0.50
_DETECT_WIDTH = 480
_MIN_HEIGHT_PX = 110
_ASPECT_RANGE = (1.4, 4.5)
_DEFAULT_MIN_WEIGHT = 0.50
_DEFAULT_DEPTH_MAD_M = 0.80
_DEFAULT_CONFIRM_FRAMES = 2
_NMS_IOU = 0.45

_HOG = None
_HOG_LOCK = threading.Lock()


def _half_fov_rad() -> float:
    try:
        return float(SPECS["camera"]["half_fov_rad"])
    except Exception:
        return math.radians(35.0)


def _camera_fx(width_px: int) -> float:
    return (width_px / 2.0) / math.tan(_half_fov_rad())


def _hog():
    """Build (once) OpenCV's default full-body HOG people detector."""
    global _HOG
    if cv2 is None:
        raise RuntimeError("OpenCV is unavailable")
    if not hasattr(cv2, "HOGDescriptor"):
        raise RuntimeError(
            "this OpenCV build has no HOGDescriptor; "
            "install opencv-python-headless<5")
    with _HOG_LOCK:
        if _HOG is None:
            hog = cv2.HOGDescriptor()
            hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())
            _HOG = hog
        return _HOG


def _iou(first, second) -> float:
    ax, ay, aw, ah = first
    bx, by, bw, bh = second
    left, top = max(ax, bx), max(ay, by)
    right, bottom = min(ax + aw, bx + bw), min(ay + ah, by + bh)
    if right <= left or bottom <= top:
        return 0.0
    overlap = (right - left) * (bottom - top)
    union = aw * ah + bw * bh - overlap
    return overlap / float(union) if union > 0 else 0.0


def nms(detections: list, iou_threshold: float = _NMS_IOU) -> list:
    """Keep the highest-weight box of each overlapping group."""
    ordered = sorted(detections, key=lambda item: item["weight"], reverse=True)
    kept = []
    for candidate in ordered:
        if any(_iou(candidate["bbox"], other["bbox"]) > iou_threshold
               for other in kept):
            continue
        kept.append(candidate)
    return kept


def _valid_box(bbox, min_height_px: int = _MIN_HEIGHT_PX) -> bool:
    _x, _y, width, height = bbox
    if width < 20 or height < min_height_px:
        return False
    aspect = height / float(width)
    return _ASPECT_RANGE[0] <= aspect <= _ASPECT_RANGE[1]


def _scale_rect(rect, factor: float, width: int, height: int) -> list:
    x, y, w, h = (int(round(value / factor)) for value in rect)
    x = max(0, min(x, width - 1))
    y = max(0, min(y, height - 1))
    w = max(1, min(w, width - x))
    h = max(1, min(h, height - y))
    return [x, y, w, h]


def _depth_stats(depth_mm, bbox):
    """Median distance and median absolute deviation inside the box."""
    x, y, w, h = bbox
    region = depth_mm[y + int(0.20 * h):y + h, x:x + w]
    if region.size == 0:
        return None
    values = region.reshape(-1).astype(np.float32)
    values = values[(values > MIN_DISTANCE_M * 1000.0)
                    & (values < MAX_DISTANCE_M * 1000.0)]
    if values.size < max(20, int(0.05 * region.size)):
        return None
    median = float(np.median(values))
    mad = float(np.median(np.abs(values - median)))
    return median / 1000.0, mad / 1000.0


def detect_people(bgr, depth_mm, fx: float | None = None,
                  min_weight: float = _DEFAULT_MIN_WEIGHT,
                  depth_mad_m: float = _DEFAULT_DEPTH_MAD_M,
                  min_height_px: int = _MIN_HEIGHT_PX) -> list:
    """Run HOG person detection and return valid people nearest-first.

    Each result carries ``distance_m``, ``bearing_deg`` (robot frame: front 0,
    left positive), the full-resolution bounding box, the HOG weight and a
    normalised confidence.  Detections without a plausible aligned depth sample
    are dropped: a distance is required downstream.
    """
    if cv2 is None:
        raise RuntimeError("OpenCV is unavailable")
    if bgr is None or bgr.size == 0:
        return []
    height, width = bgr.shape[:2]
    if depth_mm is None or depth_mm.shape[:2] != (height, width):
        return []
    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    factor = 1.0
    if width > _DETECT_WIDTH:
        factor = width / float(_DETECT_WIDTH)
        gray = cv2.resize(gray, (_DETECT_WIDTH, int(round(height / factor))),
                          interpolation=cv2.INTER_AREA)
    rects, weights = _hog().detectMultiScale(
        gray, winStride=(8, 8), padding=(8, 8), scale=1.05)
    fx = fx or _camera_fx(width)
    cx = width / 2.0

    candidates = []
    for rect, weight in zip(rects, weights):
        score = float(weight)
        if score < min_weight:
            continue
        x, y, w, h = _scale_rect(rect, factor, width, height)
        if not _valid_box((x, y, w, h), min_height_px):
            continue
        stats = _depth_stats(depth_mm, (x, y, w, h))
        if stats is None:
            continue
        distance, mad = stats
        if mad > depth_mad_m:
            continue
        bearing = -math.degrees(math.atan2((x + w / 2.0) - cx, fx))
        candidates.append({
            "distance_m": round(distance, 2),
            "bearing_deg": round(bearing, 1),
            "bbox": [int(x), int(y), int(w), int(h)],
            "weight": round(score, 3),
            "confidence": round(min(1.0, max(0.0, score / 2.0)), 2),
        })

    kept = nms(candidates)
    kept.sort(key=lambda item: item["distance_m"])
    return kept


def pick_nearest(people: list) -> dict | None:
    valid = [item for item in people
             if isinstance(item.get("distance_m"), (int, float))
             and math.isfinite(item["distance_m"])]
    return min(valid, key=lambda item: item["distance_m"]) if valid else None


class PersonTracker:
    """Confirm the nearest person over consecutive frames and smooth it."""

    def __init__(self, confirm_frames: int = _DEFAULT_CONFIRM_FRAMES,
                 iou_threshold: float = 0.30, alpha: float = 0.55):
        self.confirm_frames = max(1, int(confirm_frames))
        self.iou_threshold = iou_threshold
        self.alpha = alpha
        self.hits = 0
        self.last = None

    def reset(self) -> None:
        self.hits = 0
        self.last = None

    def update(self, people: list) -> dict | None:
        nearest = pick_nearest(people)
        if nearest is None:
            self.reset()
            return None
        candidate = dict(nearest)
        if (self.last is not None
                and _iou(self.last["bbox"], candidate["bbox"]) >= self.iou_threshold):
            self.hits += 1
            candidate["distance_m"] = round(
                self.alpha * self.last["distance_m"]
                + (1.0 - self.alpha) * candidate["distance_m"], 2)
            candidate["bearing_deg"] = round(
                self.alpha * self.last["bearing_deg"]
                + (1.0 - self.alpha) * candidate["bearing_deg"], 1)
        else:
            self.hits = 1
        self.last = candidate
        if self.hits < self.confirm_frames:
            return None
        confirmed = dict(candidate)
        confirmed["confirmed_frames"] = self.hits
        return confirmed

    def progress(self) -> dict:
        return {
            "hits": self.hits,
            "required": self.confirm_frames,
            "candidate": dict(self.last) if self.last else None,
        }


class PersonDetectPlugin:
    """MCP sensor card: ``person_detect`` — full-body human target tracker."""

    PREFIX = "person_detect"

    def __init__(self, plugin_config: dict, namespace: str, executor):
        plugin_config = plugin_config or {}
        self._namespace = namespace
        self._target_topic = f"/{namespace}/person/target"
        self._debug_topic = f"/{namespace}/person/debug"
        self._color_topic = f"/{namespace}/camera/color"
        self._depth_topic = f"/{namespace}/camera/depth"
        try:
            rate = float(plugin_config.get("detect_hz", 3.0))
        except (TypeError, ValueError):
            rate = 3.0
        self._detect_hz = min(5.0, max(1.0, rate))
        try:
            self._min_weight = max(0.0, float(
                plugin_config.get("min_weight", _DEFAULT_MIN_WEIGHT)))
        except (TypeError, ValueError):
            self._min_weight = _DEFAULT_MIN_WEIGHT
        try:
            self._depth_mad_m = max(0.05, float(
                plugin_config.get("depth_mad_m", _DEFAULT_DEPTH_MAD_M)))
        except (TypeError, ValueError):
            self._depth_mad_m = _DEFAULT_DEPTH_MAD_M
        try:
            confirm = int(plugin_config.get(
                "confirm_frames", _DEFAULT_CONFIRM_FRAMES))
        except (TypeError, ValueError):
            confirm = _DEFAULT_CONFIRM_FRAMES
        self._tracker = PersonTracker(confirm_frames=confirm)
        self._publish_debug = bool(plugin_config.get("publish_debug", True))
        self._target_out = [{"topic": self._target_topic, "format": "data/json"}]
        if self._publish_debug:
            self._target_out.append({"topic": self._debug_topic,
                                     "format": "image/jpeg"})

        self._lock = threading.RLock()
        self._color = None
        self._depth = None
        self._color_seq = 0
        self._last_seq = -1
        self._last = {"state": "waiting"}
        self._last_at = 0.0

        if cv2 is None:
            raise RuntimeError("OpenCV is required by person_detect")

        import rclpy
        from rclpy.node import Node
        from rclpy.qos import (QoSProfile, ReliabilityPolicy, HistoryPolicy,
                               DurabilityPolicy)
        from sensor_msgs.msg import CompressedImage
        from std_msgs.msg import String

        self._String = String
        self._CompressedImage = CompressedImage
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=5,
                         durability=DurabilityPolicy.VOLATILE)
        self._node = Node("bumi_person_detect")
        executor.add_node(self._node)
        self._pub = self._node.create_publisher(String, self._target_topic, qos)
        self._debug_pub = (self._node.create_publisher(
            CompressedImage, self._debug_topic, qos) if self._publish_debug
            else None)
        self._node.create_subscription(
            CompressedImage, self._color_topic, self._on_color, qos)
        self._node.create_subscription(
            CompressedImage, self._depth_topic, self._on_depth, qos)
        self._node.create_timer(1.0 / self._detect_hz, self._tick)
        print(f"[person_detect] watching {self._color_topic} + "
              f"{self._depth_topic} at {self._detect_hz:g} Hz", flush=True)

    def get_tool(self) -> dict:
        return {
            "name": "person_detect",
            "type": "sensor",
            "multiInstance": False,
            "description": (
                "用 HOG 全身行人检测器识别人形，按深度选择最近的人，发布距离（米）、"
                "方位（度，前方 0、左侧为正）和包围框；可选发布带框的 JPEG 调试图。"
                "只检测，不移动机器人。人应完整站立在画面中，建议 2.5–6 m。"
            ),
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {
                        "type": "string",
                        "enum": ["info", "start", "stop"],
                        "description": "info=查看检测结果；start/stop=状态应答。",
                    },
                },
            },
            "topic_out": self._target_out,
        }

    def get_tools(self) -> list:
        return [self.get_tool()]

    def start(self):
        return {"state": "running", "topic_out": self._target_out}

    def stop(self):
        # Sensor cards keep publishing while intelligent control stops.
        return {"state": "idle"}

    def dispatch(self, action: str, args: dict) -> dict | None:
        if action == "start":
            return self.start()
        if action == "stop":
            return self.stop()
        if action == "info":
            with self._lock:
                last = dict(self._last)
                color_age = (time.monotonic() - self._color[1]
                             if self._color else None)
                depth_age = (time.monotonic() - self._depth[1]
                             if self._depth else None)
                progress = self._tracker.progress()
            return {
                "state": "running",
                "topic_out": self._target_out,
                "color_age_s": round(color_age, 2) if color_age is not None else None,
                "depth_age_s": round(depth_age, 2) if depth_age is not None else None,
                "detect_hz": self._detect_hz,
                "min_weight": self._min_weight,
                "tracker": progress,
                "last": last,
            }
        return None

    # ── ROS callbacks ────────────────────────────────────────────────────────

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

    def _tick(self) -> None:
        now = time.monotonic()
        with self._lock:
            color = self._color
            depth = self._depth
        if color is None or now - color[1] > 1.5:
            self._publish({"state": "stale", "reason": "no_color"})
            return
        with self._lock:
            if color[2] == self._last_seq:
                return
            self._last_seq = color[2]
        frame = cv2.imdecode(np.frombuffer(color[0], np.uint8),
                             cv2.IMREAD_COLOR)
        if frame is None:
            self._publish({"state": "stale", "reason": "bad_color"})
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
        if depth_mm is None:
            self._publish({"state": "stale", "reason": "no_depth"})
            return

        try:
            people = detect_people(
                frame, depth_mm, min_weight=self._min_weight,
                depth_mad_m=self._depth_mad_m)
        except Exception as exc:
            self._publish({"state": "error", "reason": str(exc)})
            return
        confirmed = self._tracker.update(people)
        if confirmed is not None:
            payload = {
                "state": "fresh",
                "trigger": "person_hog",
                "person": confirmed,
                "people": people,
                "count": len(people),
                "timestamp_ms": int(time.time() * 1000),
            }
        elif people:
            payload = {
                "state": "candidate",
                "people": people,
                "count": len(people),
                "tracker": self._tracker.progress(),
                "timestamp_ms": int(time.time() * 1000),
            }
        else:
            payload = {"state": "no_target", "count": 0,
                       "timestamp_ms": int(time.time() * 1000)}
        self._publish(payload)
        self._publish_debug(frame, people, confirmed)

    def _publish(self, payload: dict) -> None:
        with self._lock:
            self._last = payload
            self._last_at = time.monotonic()
        msg = self._String()
        msg.data = json.dumps(payload)
        self._pub.publish(msg)

    def _publish_debug(self, frame, people: list, confirmed) -> None:
        if not self._publish_debug or self._debug_pub is None:
            return
        debug = frame.copy()
        for person in people:
            x, y, w, h = person["bbox"]
            is_target = (confirmed is not None
                         and person["bbox"] == confirmed["bbox"])
            colour = (0, 200, 0) if is_target else (0, 180, 255)
            cv2.rectangle(debug, (x, y), (x + w, y + h), colour, 2)
            label = (f"d={person['distance_m']:.2f} "
                     f"b={person['bearing_deg']:.0f} "
                     f"w={person['weight']:.2f}")
            cv2.putText(debug, label, (x, max(12, y - 6)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, colour, 1)
        ok, encoded = cv2.imencode(
            ".jpg", debug, [cv2.IMWRITE_JPEG_QUALITY, 70])
        if not ok:
            return
        msg = self._CompressedImage()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.format = "jpeg"
        msg.data = array("B", encoded.tobytes())
        self._debug_pub.publish(msg)
