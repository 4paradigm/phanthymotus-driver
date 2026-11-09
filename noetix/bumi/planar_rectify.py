"""Planar region rectification over the existing Bumi JPEG stream.

Only image geometry is produced: no OCR, camera acquisition, inferred metric
dimensions, motion, storage, or new model/runtime dependency.
"""

from __future__ import annotations

import copy
import json
import math
import re
import threading
import time

import cv2
import numpy as np


DEFAULTS = {
    "mode": "auto", "corners": [], "output_width": 640,
    "output_height": 480, "max_fps": 2.0,
}
MAX_JPEG_BYTES = 2 * 1024 * 1024
MAX_PIXELS = 1920 * 1080
MAX_AGE_S = 2.0
MIN_AREA_RATIO = 0.15
_TOPIC = re.compile(r"/(?:[A-Za-z_][A-Za-z0-9_]*)(?:/[A-Za-z_][A-Za-z0-9_]*)*")


def settings(config: dict) -> dict:
    """Validate fully before replacing an active configuration."""
    cfg = {key: copy.deepcopy(config.get(key, default)) for key, default in DEFAULTS.items()}
    if cfg["mode"] not in ("auto", "manual"):
        raise ValueError("mode must be auto or manual")
    for key in ("output_width", "output_height"):
        if type(cfg[key]) is not int or not 64 <= cfg[key] <= 1280:
            raise ValueError(f"{key} must be an integer in 64..1280")
    if (type(cfg["max_fps"]) not in (int, float)
            or not math.isfinite(cfg["max_fps"]) or not 1 <= cfg["max_fps"] <= 5):
        raise ValueError("max_fps must be finite and in 1..5")
    corners = cfg["corners"]
    if not isinstance(corners, list):
        raise ValueError("corners must be a list")
    if corners:
        if (len(corners) != 4 or any(
                not isinstance(p, list) or len(p) != 2
                or any(type(v) not in (int, float) or not math.isfinite(v)
                       or not 0 <= v <= 1 for v in p) for p in corners)):
            raise ValueError("corners must contain four normalized [x,y] pairs")
        _validate_quad(np.asarray(corners, np.float32), min_area=0.001, min_edge=0.01)
    if cfg["mode"] == "manual" and not corners:
        raise ValueError("manual mode requires four corners")
    return cfg


def _validate_quad(quad, min_area, min_edge):
    if (quad.shape != (4, 2) or not np.isfinite(quad).all()
            or not cv2.isContourConvex(quad)
            or cv2.contourArea(quad, oriented=True) < min_area
            or np.min(np.linalg.norm(quad - np.roll(quad, 1, axis=0), axis=1)) < min_edge):
        raise ValueError("corners must form a nondegenerate clockwise polygon: TL,TR,BR,BL")


def jpeg_size(data: bytes) -> tuple[int, int]:
    """Bound decoded allocation by reading JPEG SOF dimensions before imdecode."""
    if not 4 <= len(data) <= MAX_JPEG_BYTES or data[:2] != b"\xff\xd8":
        raise ValueError("invalid JPEG or encoded size exceeds 2 MiB")
    offset = 2
    sof = {0xC0, 0xC1, 0xC2}  # baseline / extended sequential / progressive
    while offset < len(data):
        if data[offset] != 0xFF:
            raise ValueError("invalid JPEG marker")
        while offset < len(data) and data[offset] == 0xFF:
            offset += 1
        if offset >= len(data):
            break
        marker = data[offset]
        offset += 1
        if marker in (0xD9, 0xDA):
            break
        if marker in (0x01, *range(0xD0, 0xD8)):
            continue
        if offset + 2 > len(data):
            break
        length = int.from_bytes(data[offset:offset + 2], "big")
        if length < 2 or offset + length > len(data):
            break
        if marker in sof and length >= 8:
            height = int.from_bytes(data[offset + 3:offset + 5], "big")
            width = int.from_bytes(data[offset + 5:offset + 7], "big")
            if min(width, height) < 32 or width * height > MAX_PIXELS:
                raise ValueError("JPEG dimensions outside supported bounds")
            return width, height
        offset += length
    raise ValueError("unsupported or incomplete JPEG")


def _ordered(points):
    center = points.mean(axis=0)
    angle = np.arctan2(points[:, 1] - center[1], points[:, 0] - center[0])
    quad = points[np.argsort(angle)]
    return np.roll(quad, -int(np.argmin(quad.sum(axis=1))), axis=0).astype(np.float32)


def find_quad(image):
    """Conservative geometric proposals; a quadrilateral is not a document label."""
    height, width = image.shape[:2]
    scale = min(1.0, 640 / max(width, height))
    small = cv2.resize(image, (round(width * scale), round(height * scale)))
    gray = cv2.cvtColor(small, cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(cv2.GaussianBlur(gray, (5, 5), 0), 50, 150)
    edges = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
    contours, _ = cv2.findContours(edges, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    candidates = []
    small_height, small_width = gray.shape
    for contour in sorted(contours, key=cv2.contourArea, reverse=True)[:40]:
        area = cv2.contourArea(contour)
        if not MIN_AREA_RATIO <= area / gray.size <= 0.95:
            continue
        approx = cv2.approxPolyDP(contour, 0.02 * cv2.arcLength(contour, True), True)
        if len(approx) != 4:
            continue
        quad = _ordered(approx[:, 0, :].astype(np.float32))
        try:
            _validate_quad(quad, gray.size * MIN_AREA_RATIO, 16)
        except ValueError:
            continue
        if (np.any(quad < 2) or np.any(quad[:, 0] > small_width - 3)
                or np.any(quad[:, 1] > small_height - 3)):
            continue
        # Canny's inner/outer boundary must not become two separate candidates.
        if any(np.max(np.linalg.norm(quad - old, axis=1)) < 8 for old in candidates):
            continue
        candidates.append(quad)
    if not candidates:
        return None, "no_boundary", 0
    candidates.sort(key=cv2.contourArea, reverse=True)
    if len(candidates) > 1:
        first, second = candidates[:2]
        intersection, _ = cv2.intersectConvexConvex(first, second)
        smaller = cv2.contourArea(second)
        # A substantial, distinct second rectangle needs operator selection.
        if smaller >= 0.65 * cv2.contourArea(first) and intersection < 0.85 * smaller:
            return None, "ambiguous_boundary", len(candidates)
    # Account for rounded resize dimensions on each axis, not nominal scale.
    return (candidates[0] * [width / small_width, height / small_height]).astype(np.float32), "ok", len(candidates)


def rectify(data: bytes, config: dict) -> tuple[dict, bytes | None]:
    width, height = jpeg_size(data)
    image = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
    if image is None or image.shape[:2] != (height, width):
        raise ValueError("JPEG decoding failed or dimensions changed")
    report = {"valid": False, "mode": config["mode"], "source_size": [width, height],
              "output_size": [config["output_width"], config["output_height"]],
              "corners_px": None, "homography": None,
              "geometry": "planar_projective_pixels", "metric_geometry": False}
    if config["mode"] == "manual":
        quad = (np.asarray(config["corners"], np.float32) * [width - 1, height - 1]).astype(np.float32)
        _validate_quad(quad, width * height * 0.001, 2)
        report.update(status="ok", candidate_count=None, selection="configured_corners")
    else:
        quad, reason, count = find_quad(image)
        report.update(status=reason, candidate_count=count, selection="geometric_candidate")
        if quad is None:
            return report, None
    out_width, out_height = report["output_size"]
    destination = np.float32([[0, 0], [out_width - 1, 0],
                             [out_width - 1, out_height - 1], [0, out_height - 1]])
    matrix = cv2.getPerspectiveTransform(quad, destination)
    if not np.isfinite(matrix).all() or abs(np.linalg.det(matrix)) < 1e-12:
        raise ValueError("singular perspective transform")
    output = cv2.warpPerspective(image, matrix, (out_width, out_height))
    encoded, jpeg = cv2.imencode(".jpg", output, [cv2.IMWRITE_JPEG_QUALITY, 90])
    if not encoded:
        raise ValueError("output JPEG encoding failed")
    report.update(valid=True, corners_px=quad.tolist(), homography=matrix.tolist())
    return report, jpeg.tobytes()


class PlanarRectifyPlugin:
    PREFIX = "planar_rectify"

    def __init__(self, plugin_config: dict, namespace: str, executor):
        self._config = settings(plugin_config)
        self._default_topic = f"/{namespace}/camera/color"
        self._input_topic = self._default_topic
        self._image_topic = f"/{namespace}/planar_rectify/image"
        self._state_topic = f"/{namespace}/planar_rectify/state"
        self._executor = executor
        self._lock = threading.Lock()
        self._lifecycle = threading.RLock()
        self._publish_lock = threading.Lock()
        self._cancel = threading.Event()
        self._cancel.set()
        self._node = self._thread = self._latest = None
        self._sequence = 0
        self._last_stamp = None
        self._result = {"valid": False, "status": "idle"}

    def get_tool(self):
        config_schema = {
            "type": "object", "properties": {
                "mode": {"type": "string", "enum": ["auto", "manual"], "default": "auto"},
                "corners": {"type": "array", "default": [], "maxItems": 4,
                            "items": {"type": "array", "minItems": 2, "maxItems": 2,
                                      "items": {"type": "number", "minimum": 0, "maximum": 1}},
                            "description": "Manual TL,TR,BR,BL normalized [x,y] corners"},
                "output_width": {"type": "integer", "minimum": 64, "maximum": 1280, "default": 640},
                "output_height": {"type": "integer", "minimum": 64, "maximum": 1280, "default": 480},
                "max_fps": {"type": "number", "minimum": 1, "maximum": 5, "default": 2},
            },
        }
        return {
            "name": self.PREFIX, "type": "processor", "multiInstance": False,
            "x-resource": ["image_processing"],
            "description": "Flatten a planar quadrilateral from Bumi JPEG images for OCR or reading. "
                           "Automatic boundaries are geometric candidates, not document recognition. "
                           "Configure output aspect ratio; no OCR, motion or metric measurements.",
            "inputSchema": {
                "type": "object", "properties": {
                    "action": {"type": "string", "enum": ["start", "stop", "info", "config"]},
                    "input_topic": {"type": "string"}, **config_schema["properties"]},
                "required": ["action"],
                "x-action-params": {
                    "start": {"params": ["input_topic"]}, "stop": {"params": []},
                    "info": {"params": []}, "config": {"params": list(DEFAULTS)}},
            },
            "configSchema": config_schema,
            "topic_in": [{"format": "image/jpeg", "desc": "Planar scene"}],
            "topic_out": [{"topic": self._image_topic, "format": "image/jpeg", "desc": "Rectified region"},
                          {"topic": self._state_topic, "format": "data/json", "desc": "Validity and transform"}],
        }

    def _on_frame(self, msg):
        with self._lock:
            if self._cancel.is_set():
                return
            self._sequence += 1
            if len(msg.data) > MAX_JPEG_BYTES:
                self._latest = None
                self._result = {"valid": False, "status": "invalid_image", "reason": "encoded_size_limit"}
                return
            stamp = (int(msg.header.stamp.sec), int(msg.header.stamp.nanosec))
            if stamp != (0, 0):
                if self._last_stamp is not None and stamp <= self._last_stamp:
                    self._latest = None
                    self._result = {"valid": False, "status": "replayed_input"}
                    return
                self._last_stamp = stamp
            self._latest = (self._sequence, bytes(msg.data), time.monotonic(), stamp)

    def _make_node(self):
        from rclpy.node import Node
        from rclpy.qos import (QoSProfile, ReliabilityPolicy, HistoryPolicy,
                              DurabilityPolicy)
        from sensor_msgs.msg import CompressedImage
        from std_msgs.msg import String

        node = Node("bumi_planar_rectify")
        try:
            qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                             history=HistoryPolicy.KEEP_LAST, depth=1,
                             durability=DurabilityPolicy.VOLATILE)
            node.image_pub = node.create_publisher(CompressedImage, self._image_topic, qos)
            node.state_pub = node.create_publisher(String, self._state_topic, qos)
            node.create_subscription(CompressedImage, self._input_topic, self._on_frame, qos)

            def status():
                with self._publish_lock:
                    if not self._cancel.is_set():
                        node.send_status(self.info())

            node.send_status = lambda report: node.state_pub.publish(
                String(data=json.dumps(report, allow_nan=False)))

            def image(data, stamp):
                msg = CompressedImage()
                msg.format = "jpeg"
                msg.header.stamp.sec, msg.header.stamp.nanosec = stamp
                msg.header.frame_id = "planar_rectified_pixels"
                msg.data = data
                node.image_pub.publish(msg)

            node.send_image = image
            node.create_timer(0.5, status)
            return node
        except BaseException:
            node.destroy_node()
            raise

    def start(self):
        with self._lifecycle:
            if self._thread is not None:
                return
            node = self._make_node()
            registered = False
            try:
                self._executor.add_node(node)
                registered = True
                with self._lock:
                    self._latest = None
                    self._last_stamp = None
                    self._result = {"valid": False, "status": "no_input"}
                self._cancel.clear()
                thread = threading.Thread(target=self._run_guarded, name="bumi_planar_rectify", daemon=True)
                self._node, self._thread = node, thread
                thread.start()
            except BaseException:
                self._cancel.set()
                self._node = self._thread = None
                try:
                    if registered:
                        self._executor.remove_node(node)
                finally:
                    node.destroy_node()
                raise

    def stop(self):
        with self._lifecycle:
            self._cancel.set()
            if self._thread is not None:
                self._thread.join(timeout=5)
                if self._thread.is_alive():
                    raise RuntimeError("planar rectification worker did not stop")
                self._thread = None
            with self._lock:
                self._latest = None
                self._result = {"valid": False, "status": "idle"}
            with self._publish_lock:
                if self._node is not None:
                    try:
                        self._node.send_status(self.info())
                    except Exception:
                        # Terminal DDS notification is best effort; a failed
                        # publisher must not prevent the rest of bundle shutdown.
                        pass
                    finally:
                        self._executor.remove_node(self._node)
                        self._node.destroy_node()
                        self._node = None

    def _run(self):
        last_sequence = -1
        while not self._cancel.wait(1 / self._config["max_fps"]):
            with self._lock:
                frame = self._latest
            if frame is None or frame[0] == last_sequence:
                continue
            sequence, data, received, stamp = frame
            last_sequence = sequence
            if time.monotonic() - received > MAX_AGE_S:
                continue
            try:
                result, image = rectify(data, self._config)
            except (ValueError, cv2.error):
                result, image = {"valid": False, "status": "invalid_image"}, None
            except Exception:
                result, image = {"valid": False, "status": "processing_error"}, None
            result.update(frame_sequence=sequence, input_received_monotonic_s=received,
                          input_ros_stamp={"sec": stamp[0], "nanosec": stamp[1]},
                          timestamp_source="host_receipt", capture_time_verified=False)
            with self._publish_lock:
                if self._cancel.is_set():
                    return
                if time.monotonic() - received > MAX_AGE_S:
                    result.update(valid=False, status="stale_input")
                    image = None
                with self._lock:
                    # A rejected newer input must not be overwritten by old work.
                    if self._latest is None:
                        continue
                    self._result = result
                if image is not None:
                    self._node.send_image(image, stamp)

    def _run_guarded(self):
        try:
            self._run()
        except Exception:
            with self._lock:
                self._latest = None
                self._result = {"valid": False, "status": "worker_error"}

    def info(self):
        with self._lock:
            result = copy.deepcopy(self._result)
            latest = self._latest
        now = time.monotonic()
        received = result.get("input_received_monotonic_s")
        result["input_age_s"] = None if received is None else max(0.0, now - received)
        if not self._cancel.is_set():
            if latest is not None and now - latest[2] > MAX_AGE_S:
                result.update(valid=False, status="stale_input", homography=None, corners_px=None)
            elif received is not None and now - received > MAX_AGE_S:
                result.update(valid=False, status="stale_result", homography=None, corners_px=None)
        state = "idle" if self._cancel.is_set() else (
            "error" if result["status"] == "worker_error" else "running")
        result.update(state=state,
                      input_topic=self._input_topic, config=copy.deepcopy(self._config),
                      topic_out=self.get_tool()["topic_out"], camera_info=[],
                      metric_geometry=False, output_geometry="planar_projective_pixels")
        return result

    def dispatch(self, action: str, args: dict) -> dict | None:
        if action in ("info", "robot_info"):
            return self.info()
        if action == "stop":
            self.stop()
            return {"state": "idle"}
        if action in ("start", "config"):
            with self._lifecycle:
                try:
                    cfg = settings({**self._config, **{k: args[k] for k in DEFAULTS if k in args}})
                    topics = args.get("input_topics")
                    if topics is not None and (not isinstance(topics, list) or len(topics) != 1):
                        raise ValueError("exactly one image input topic is required")
                    topic = args.get("input_topic") or (topics[0] if topics else self._input_topic)
                    if (not isinstance(topic, str) or not _TOPIC.fullmatch(topic)
                            or topic in (self._image_topic, self._state_topic)):
                        raise ValueError("invalid or feedback-loop input topic")
                except (ValueError, TypeError) as exc:
                    return {"state": "error", "error": "invalid_parameters", "detail": str(exc)}
                running = not self._cancel.is_set()
                self.stop()
                self._config, self._input_topic = cfg, topic
                if action == "start" or running:
                    self.start()
                return self.info()
        return None
