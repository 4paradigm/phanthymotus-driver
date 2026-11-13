#!/usr/bin/env python3
"""Bottle-shape detector card for the Noetix Bumi bundle.

A sensor card: it never commands the chassis.  It consumes the aligned
colour/depth pair published by the camera card, keeps contours whose silhouette
looks like a mineral-water bottle, confirms the nearest candidate over several
frames and publishes its distance and bearing for a downstream approach card.

Detection is shape-based, not colour-based.  A silhouette must be tall and
narrow (aspect 2-6), have a narrower neck than body (0.25-0.85), fill most of
its bounding box, be convex enough, be left/right symmetric, and sit at a
consistent depth.  Three edge sources (grey, saturation, adaptive threshold)
are merged so a labelled or slightly transparent bottle is not lost just
because its grey edges are weak.

An optional JPEG debug stream draws every candidate and the confirmed target;
it is the fastest way to tune thresholds on the real robot.

ROS is imported in the plugin constructor; the geometry, tracker and image
helpers are pure and testable without rclpy.
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

MAX_DISTANCE_M = 6.0
MIN_DISTANCE_M = 0.20
_MIN_HEIGHT_PX = 45
_MIN_AREA_PX = 500
_ASPECT_RANGE = (2.0, 6.0)
_NECK_RATIO_RANGE = (0.25, 0.85)
_RECT_FILL_RANGE = (0.40, 0.95)
_MIN_SOLIDITY = 0.80
_MAX_SYMMETRY_OFFSET = 0.20
_DEFAULT_MIN_CONFIDENCE = 0.45
_DEFAULT_DEPTH_MAD_M = 0.35
_DEFAULT_CONFIRM_FRAMES = 2


def _half_fov_rad() -> float:
    try:
        return float(SPECS["camera"]["half_fov_rad"])
    except Exception:
        return math.radians(35.0)


def _camera_fx(width_px: int) -> float:
    return (width_px / 2.0) / math.tan(_half_fov_rad())


def _row_widths(mask, x: int, y: int, w: int, h: int) -> np.ndarray:
    """Width of the silhouette on each row of the bounding box."""
    widths = np.zeros(h, dtype=np.float32)
    rows = mask[y:y + h, x:x + w]
    for index in range(h):
        columns = np.flatnonzero(rows[index])
        if columns.size:
            widths[index] = float(columns[-1] - columns[0] + 1)
    return widths


def _symmetry_offset(mask, bbox) -> float:
    """Horizontal centroid offset from the box centre, 0 (centred) to 1."""
    x, y, w, h = bbox
    moments = cv2.moments(mask[y:y + h, x:x + w], binaryImage=True)
    if moments["m00"] <= 0:
        return 1.0
    centre = moments["m10"] / moments["m00"]
    return abs(centre - w / 2.0) / max(w / 2.0, 1.0)


def _shape_metrics(mask, contour_area: float, hull_area: float,
                   bbox) -> dict | None:
    """Return bottle-shape measurements, or None when the silhouette fails."""
    x, y, w, h = bbox
    if h < _MIN_HEIGHT_PX or w < 8 or contour_area < _MIN_AREA_PX:
        return None
    aspect = h / float(w)
    if not _ASPECT_RANGE[0] <= aspect <= _ASPECT_RANGE[1]:
        return None
    rect_fill = contour_area / float(w * h)
    if not _RECT_FILL_RANGE[0] <= rect_fill <= _RECT_FILL_RANGE[1]:
        return None
    solidity = contour_area / float(hull_area) if hull_area > 0 else 0.0
    if solidity < _MIN_SOLIDITY:
        return None
    symmetry = _symmetry_offset(mask, bbox)
    if symmetry > _MAX_SYMMETRY_OFFSET:
        return None

    widths = _row_widths(mask, x, y, w, h)
    neck_slice = widths[int(0.08 * h):max(int(0.08 * h) + 1, int(0.30 * h))]
    body_slice = widths[int(0.45 * h):max(int(0.45 * h) + 1, int(0.90 * h))]
    neck_values = neck_slice[neck_slice > 0]
    body_values = body_slice[body_slice > 0]
    if neck_values.size == 0 or body_values.size == 0:
        return None
    neck = float(np.median(neck_values))
    body = float(np.median(body_values))
    if body < 8.0:
        return None
    neck_ratio = neck / body
    if not _NECK_RATIO_RANGE[0] <= neck_ratio <= _NECK_RATIO_RANGE[1]:
        return None

    aspect_score = max(0.0, 1.0 - abs(aspect - 4.0) / 3.0)
    neck_score = max(0.0, 1.0 - abs(neck_ratio - 0.55) / 0.35)
    symmetry_score = max(0.0, 1.0 - symmetry / 0.30)
    confidence = min(1.0, 0.35 * neck_score + 0.25 * aspect_score
                     + 0.20 * solidity + 0.10 * symmetry_score
                     + 0.10 * rect_fill)
    return {
        "aspect": round(aspect, 2),
        "neck_ratio": round(neck_ratio, 2),
        "rect_fill": round(rect_fill, 2),
        "solidity": round(solidity, 2),
        "symmetry": round(symmetry, 3),
        "confidence": round(confidence, 2),
    }


def _contours(gray, saturation) -> list:
    """Merge grey, saturation and adaptive-threshold outer contours."""
    found = []
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)

    grey_edges = cv2.Canny(blurred, 40, 120)
    grey_edges = cv2.morphologyEx(
        cv2.dilate(grey_edges, np.ones((3, 3), np.uint8), iterations=2),
        cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8), iterations=2)
    found.extend(cv2.findContours(grey_edges, cv2.RETR_EXTERNAL,
                                  cv2.CHAIN_APPROX_SIMPLE)[0])

    # A coloured label or cap can outline a bottle whose grey edges vanish.
    sat_edges = cv2.Canny(cv2.GaussianBlur(saturation, (5, 5), 0), 30, 90)
    sat_edges = cv2.morphologyEx(
        cv2.dilate(sat_edges, np.ones((3, 3), np.uint8), iterations=2),
        cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8), iterations=2)
    found.extend(cv2.findContours(sat_edges, cv2.RETR_EXTERNAL,
                                  cv2.CHAIN_APPROX_SIMPLE)[0])

    threshold = cv2.adaptiveThreshold(
        blurred, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
        cv2.THRESH_BINARY_INV, 31, 5)
    threshold = cv2.morphologyEx(
        threshold, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8), iterations=2)
    found.extend(cv2.findContours(threshold, cv2.RETR_EXTERNAL,
                                  cv2.CHAIN_APPROX_SIMPLE)[0])
    return found


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


def detect_bottles(bgr, depth_mm, fx: float | None = None,
                   min_confidence: float = _DEFAULT_MIN_CONFIDENCE,
                   depth_mad_m: float = _DEFAULT_DEPTH_MAD_M) -> list:
    """Find bottle-shaped objects and return them nearest-first.

    Each result carries ``distance_m``, ``bearing_deg`` (robot frame: front 0,
    left positive), the bounding box and shape confidence.  Objects without a
    valid aligned depth sample are skipped: a distance is required downstream,
    and inventing one from the bounding box would be a guess.
    """
    if cv2 is None:
        raise RuntimeError("OpenCV is unavailable")
    if bgr is None or bgr.size == 0:
        return []
    height, width = bgr.shape[:2]
    if depth_mm is None or depth_mm.shape[:2] != (height, width):
        return []
    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    fx = fx or _camera_fx(width)
    cx = width / 2.0

    candidates = []
    for contour in _contours(gray, hsv[:, :, 1]):
        area = cv2.contourArea(contour)
        if area < _MIN_AREA_PX:
            continue
        x, y, w, h = cv2.boundingRect(contour)
        hull = cv2.convexHull(contour)
        hull_area = cv2.contourArea(hull)
        mask = np.zeros((height, width), dtype=np.uint8)
        cv2.drawContours(mask, [contour], -1, 255, thickness=-1)
        metrics = _shape_metrics(mask, area, hull_area, (x, y, w, h))
        if metrics is None or metrics["confidence"] < min_confidence:
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
            "confidence": metrics["confidence"],
            "shape": metrics,
        })

    # Several sources can describe the same bottle; keep the best of each
    # overlapping group.
    candidates.sort(key=lambda item: item["confidence"], reverse=True)
    kept = []
    for candidate in candidates:
        if any(_iou(candidate["bbox"], other["bbox"]) > 0.45 for other in kept):
            continue
        kept.append(candidate)
    kept.sort(key=lambda item: item["distance_m"])
    return kept


def pick_nearest(bottles: list) -> dict | None:
    valid = [item for item in bottles
             if isinstance(item.get("distance_m"), (int, float))
             and math.isfinite(item["distance_m"])]
    return min(valid, key=lambda item: item["distance_m"]) if valid else None


class BottleTracker:
    """Confirm the nearest candidate over consecutive frames and smooth it."""

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

    def update(self, bottles: list) -> dict | None:
        nearest = pick_nearest(bottles)
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


class BottleDetectPlugin:
    """MCP sensor card: ``bottle_detect`` — bottle-shaped target tracker."""

    PREFIX = "bottle_detect"

    def __init__(self, plugin_config: dict, namespace: str, executor):
        plugin_config = plugin_config or {}
        self._namespace = namespace
        self._target_topic = f"/{namespace}/bottle/target"
        self._debug_topic = f"/{namespace}/bottle/debug"
        self._color_topic = f"/{namespace}/camera/color"
        self._depth_topic = f"/{namespace}/camera/depth"
        try:
            rate = float(plugin_config.get("detect_hz", 5.0))
        except (TypeError, ValueError):
            rate = 5.0
        self._detect_hz = min(10.0, max(1.0, rate))
        try:
            self._min_confidence = min(1.0, max(0.0, float(
                plugin_config.get("min_confidence", _DEFAULT_MIN_CONFIDENCE))))
        except (TypeError, ValueError):
            self._min_confidence = _DEFAULT_MIN_CONFIDENCE
        try:
            self._depth_mad_m = max(0.05, float(
                plugin_config.get("depth_mad_m", _DEFAULT_DEPTH_MAD_M)))
        except (TypeError, ValueError):
            self._depth_mad_m = _DEFAULT_DEPTH_MAD_M
        try:
            confirm = int(plugin_config.get("confirm_frames", _DEFAULT_CONFIRM_FRAMES))
        except (TypeError, ValueError):
            confirm = _DEFAULT_CONFIRM_FRAMES
        self._tracker = BottleTracker(confirm_frames=confirm)
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
            raise RuntimeError("OpenCV is required by bottle_detect")

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
        self._node = Node("bumi_bottle_detect")
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
        print(f"[bottle_detect] watching {self._color_topic} + "
              f"{self._depth_topic} at {self._detect_hz:g} Hz", flush=True)

    def get_tool(self) -> dict:
        return {
            "name": "bottle_detect",
            "type": "sensor",
            "multiInstance": False,
            "description": (
                "检测 Bumi 视野中符合矿泉水瓶轮廓的目标（细长、带较窄瓶颈、左右对称），"
                "按深度选择最近的一瓶，发布距离（米）、方位（度，前方 0、左侧为正）"
                "和包围框；可选发布带框的 JPEG 调试图。只检测，不移动机器人。"
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
                "min_confidence": self._min_confidence,
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
            bottles = detect_bottles(
                frame, depth_mm, min_confidence=self._min_confidence,
                depth_mad_m=self._depth_mad_m)
        except Exception as exc:
            self._publish({"state": "error", "reason": str(exc)})
            return
        confirmed = self._tracker.update(bottles)
        if confirmed is not None:
            payload = {
                "state": "fresh",
                "trigger": "bottle_shape",
                "bottle": confirmed,
                "count": len(bottles),
                "timestamp_ms": int(time.time() * 1000),
            }
        elif bottles:
            payload = {
                "state": "candidate",
                "count": len(bottles),
                "tracker": self._tracker.progress(),
                "timestamp_ms": int(time.time() * 1000),
            }
        else:
            payload = {"state": "no_target", "count": 0,
                       "timestamp_ms": int(time.time() * 1000)}
        self._publish(payload)
        self._publish_debug(frame, bottles, confirmed)

    def _publish(self, payload: dict) -> None:
        with self._lock:
            self._last = payload
            self._last_at = time.monotonic()
        msg = self._String()
        msg.data = json.dumps(payload)
        self._pub.publish(msg)

    def _publish_debug(self, frame, bottles: list, confirmed) -> None:
        if not self._publish_debug or self._debug_pub is None:
            return
        debug = frame.copy()
        for bottle in bottles:
            x, y, w, h = bottle["bbox"]
            is_target = (confirmed is not None
                         and bottle["bbox"] == confirmed["bbox"])
            colour = (0, 200, 0) if is_target else (0, 180, 255)
            cv2.rectangle(debug, (x, y), (x + w, y + h), colour, 2)
            label = (f"d={bottle['distance_m']:.2f} "
                     f"b={bottle['bearing_deg']:.0f} "
                     f"c={bottle['confidence']:.2f}")
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
