"""Tianyi head actuator with built-in anonymous face tracking."""

from __future__ import annotations

import json
import math
import threading
import time

from rclpy.node import Node
from rclpy.exceptions import InvalidTopicNameException
from rclpy.validate_full_topic_name import validate_full_topic_name
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import String
from urllib.request import Request, urlopen

FRAME_MAX_AGE = 0.4
DEPTH_MAX_SKEW = 0.085
MODES = ("face", "nearest_face", "object")


def _stamp(message):
    header = getattr(message, "header", None)
    if header is None:
        return "", 0
    return header.frame_id, header.stamp.sec + header.stamp.nanosec / 1e9


def _nearest_face(boxes, size, rgb, depth, np):
    rgb_frame, rgb_time = _stamp(rgb)
    depth_frame, depth_time = _stamp(depth)
    if (not rgb_frame or rgb_frame != depth_frame or not rgb_time or not depth_time
            or abs(rgb_time - depth_time) > DEPTH_MAX_SKEW
            or size != (depth.width, depth.height)):
        return None
    formats = {"16UC1": ("u2", 0.001), "32FC1": ("f4", 1.0)}
    if depth.encoding not in formats:
        return None
    code, scale = formats[depth.encoding]
    dtype = np.dtype((">" if depth.is_bigendian else "<") + code)
    if depth.step < depth.width * dtype.itemsize or len(depth.data) != depth.height * depth.step:
        return None
    pixels = np.ndarray((depth.height, depth.width), dtype=dtype,
                        buffer=bytes(depth.data), strides=(depth.step, dtype.itemsize))
    candidates = []
    for box in boxes:
        x, y, w, h = box
        # Inner face patch excludes most background and avoids single-pixel noise.
        patch = pixels[max(0, y+h//3):y+2*h//3, max(0, x+w//3):x+2*w//3]
        metres = patch * scale
        valid = metres[np.isfinite(metres) & (metres >= 0.2) & (metres <= 8)]
        if patch.size and valid.size >= max(5, patch.size // 3):
            candidates.append((float(np.median(valid)), box))
    return min(candidates)[1] if candidates else None


def _start_vop(topic):
    request = Request("http://127.0.0.1:15720/mcp", data=json.dumps({
        "jsonrpc": "2.0", "id": 1, "method": "tools/call", "params": {
            "name": "vop", "arguments": {"action": "start", "input_topic": topic},
        },
    }).encode(), headers={"Content-Type": "application/json"})
    with urlopen(request, timeout=3) as response:
        payload = json.load(response)
    result = json.loads(payload["result"]["content"][0]["text"])
    if result.get("state") not in ("running", "loading"):
        raise ValueError(result.get("message", "VOP did not start"))

COMMAND_PERIOD = 0.2
DEADBAND = 0.12
INPUT = {"format": "image/jpeg", "desc": "可选 RGB 相机，默认使用天轶头部相机"}


def _iou(a, b):
    ax, ay, aw, ah = a
    bx, by, bw, bh = b
    width = max(0, min(ax + aw, bx + bw) - max(ax, bx))
    height = max(0, min(ay + ah, by + bh) - max(ay, by))
    intersection = width * height
    return intersection / (aw * ah + bw * bh - intersection)


class _Tracker:
    def __init__(self):
        self.bbox = None
        self.point = None
        self.track_id = 0
        self.last_seen = 0.0
        self.size = None

    def update(self, boxes, size, now):
        if size != self.size or now - self.last_seen >= FRAME_MAX_AGE:
            self.bbox = None
        self.size = size
        if not boxes:
            return None
        if self.bbox is None:
            self.bbox = max(boxes, key=lambda box: box[2] * box[3])
            self.track_id += 1
        else:
            ranked = sorted(((_iou(self.bbox, box), box) for box in boxes), reverse=True)
            score, candidate = ranked[0]
            if score < 0.2 or (len(ranked) > 1 and score - ranked[1][0] < 0.1):
                return None
            self.bbox = candidate
        self.last_seen = now
        x, y, w, h = self.bbox
        width, height = size
        return {
            "track_id": self.track_id,
            "bbox": list(self.bbox),
            "offset": [round((2 * x + w) / width - 1, 4),
                       round((2 * y + h) / height - 1, 4)],
        }


    def update_points(self, points, now):
        if now - self.last_seen >= FRAME_MAX_AGE:
            self.point = None
        if not points:
            return None
        anchor = self.point or (0, 0)
        ranked = sorted((math.dist(anchor, point), point) for point in points)
        distance, candidate = ranked[0]
        if (self.point is not None and distance > 0.25
                or len(ranked) > 1 and ranked[1][0] - distance < 0.05):
            return None
        if self.point is None:
            self.track_id += 1
        self.point = candidate
        self.last_seen = now
        return {"track_id": self.track_id, "offset": candidate}


class _Detector:
    def __init__(self):
        import cv2
        import numpy as np

        self.np = np
        self.cv2 = cv2
        path = cv2.data.haarcascades + "haarcascade_frontalface_default.xml"
        self.cascade = self.cv2.CascadeClassifier(str(path))
        if self.cascade.empty():
            raise RuntimeError(f"OpenCV could not load face cascade: {path}")

    def detect(self, data):
        gray = self.cv2.imdecode(self.np.frombuffer(data, dtype=self.np.uint8),
                                 self.cv2.IMREAD_GRAYSCALE)
        if gray is None:
            raise ValueError("undecodable frame")
        height, width = gray.shape
        small_w, small_h = width, height
        if max(width, height) > 640:
            scale = 640 / max(width, height)
            small_w, small_h = max(1, round(width * scale)), max(1, round(height * scale))
            gray = self.cv2.resize(gray, (small_w, small_h), interpolation=self.cv2.INTER_AREA)
        boxes = []
        for x, y, w, h in self.cascade.detectMultiScale(
            gray, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30)
        ):
            left, right = round(x * width / small_w), round((x + w) * width / small_w)
            top, bottom = round(y * height / small_h), round((y + h) * height / small_h)
            boxes.append((left, top, right - left, bottom - top))
        return boxes, (width, height)


class HeadFaceFollowPlugin:
    def __init__(self, ros2, head, namespace, camera=None):
        self._ros2 = ros2
        self._head = head
        self._camera = camera
        self._default_topic = f"/{namespace}/camera/head"
        self._detector = None
        self._tracker = None
        self._last_sample = -math.inf
        self._lock = threading.Lock()
        self._node = None
        self._topic = ""
        self._armed = False
        self._armed_at = 0.0
        self._last_frame = None
        self._last_command = -math.inf
        self._track = None
        self._mode = "face"
        self._depth_node = None
        self._depth_frames = []
        self._object_name = ""
        self._object_packet = None
        self._object_stamp = 0
        self._reason = "idle"

    def get_tool(self):
        return {
            "name": "head_face_follow",
            "type": "actuator",
            "description": "天轶头部随动。内置人脸检测和跟踪，开启后使用头部相机跟随人脸。",
            "topic_in": [INPUT],
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {"type": "string", "enum": ["start", "arm", "stop", "info", "set_target", "set_mode"]},
                    "mode": {"type": "string", "enum": list(MODES)},
                    "object_name": {"type": "string", "description": "VOP 类别名。空字符串切回人脸"},
                    "input_topic": {"type": "string", "description": "可选 JPEG 相机话题，留空使用内置头部相机"},
                },
                "required": ["action"],
                "x-resource": "head",
                "x-action-params": {
                    "start": {"params": ["input_topic"], "description": "启动相机订阅，等待开启"},
                    "set_mode": {"params": ["mode", "object_name"], "description": "切换跟随模式"},
                    "set_target": {"params": ["object_name"], "description": "由大脑选择要跟踪的物体"},
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
            self._detector = None
            self._reset_tracking()
            depth_node = self._depth_node
            self._depth_node = None
            self._depth_frames.clear()
            self._tracker = None
            self._topic = ""
            self._mode = "face"
            self._object_name = ""
            self._armed = False
            self._track = None
            self._last_frame = None
            self._reason = "idle"
            if node is not None:
                self._ros2.executor_core.remove_node(node)
                node.destroy_node()
            if depth_node is not None:
                self._ros2.executor_tianyi.remove_node(depth_node)
                depth_node.destroy_node()

    def _reset_tracking(self):
        self._tracker = _Tracker()
        self._track = self._last_frame = None
        self._object_packet = None

    def dispatch(self, action, args):
        if action == "stop":
            self.stop()
            return {"state": "idle"}
        with self._lock:
            if action == "start":
                topic = args.get("input_topic") or self._default_topic
                if not isinstance(topic, str) or not topic.startswith("/"):
                    return {"state": "error", "error": "Expected an absolute JPEG camera topic"}
                try:
                    validate_full_topic_name(topic)
                except (InvalidTopicNameException, ValueError) as error:
                    return {"state": "error", "error": str(error)}
                if self._node is not None and self._topic != topic:
                    return {"state": "error", "error": "Stop before changing the input topic"}
                if self._node is None:
                    node = Node("tianyi2_head_face_follow", context=self._ros2.ctx_core)
                    try:
                        detector = _Detector()
                        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
                        node.create_subscription(
                            CompressedImage, topic, lambda msg: self._on_frame(node, msg), qos)
                        node.create_subscription(
                            String, topic + "/objects", lambda msg: self._objects(node, msg), 1)
                        if topic == self._default_topic and self._camera is not None:
                            self._camera.start()
                        if not self._ros2.executor_core.add_node(node):
                            raise RuntimeError("Could not register face follow node")
                    except Exception as error:
                        node.destroy_node()
                        return {"state": "error", "error": str(error)}
                    self._node = node
                    self._detector = detector
                    self._topic = topic
                    self._reason = "awaiting_arm"
            elif action in ("set_mode", "set_target"):
                name = args.get("object_name", self._object_name if action == "set_mode" else None)
                if not isinstance(name, str):
                    return {"state": "error", "error": "Provide a VOP class name"}
                name = name.strip().casefold()
                mode = ("object" if name else "face") if action == "set_target" else args.get("mode")
                if mode not in MODES or (mode == "object" and not name):
                    return {"state": "error", "error": "Invalid mode or missing object name"}
                if self._node is None:
                    return {"state": "error", "error": "Start the card before selecting a mode"}
                if mode == "nearest_face" and self._topic != self._default_topic:
                    return {"state": "error", "error": "Start the card with its built-in head camera"}
                if mode == "nearest_face" and self._depth_node is None:
                    try:
                        self._start_geometry()
                    except Exception as error:
                        return {"state": "error", "error": str(error)}
                if mode == "object":
                    if self._node.count_publishers(self._topic + "/objects") == 0:
                        try:
                            _start_vop(self._topic)
                        except (OSError, ValueError, KeyError, IndexError, TypeError) as error:
                            return {"state": "error", "error": f"Cannot start VOP: {error}"}
                self._mode = mode
                self._object_name = name if mode == "object" else ""
                self._reset_tracking()
                self._reason = f"waiting_for_{mode}"
            elif action == "arm":
                if self._node is None:
                    return {"state": "error", "error": "Start the card first"}
                if not self._armed:
                    check = self._head.face_follow_ready()
                    if check:
                        return check
                    self._armed = True
                    self._armed_at = time.monotonic()
                    self._reset_tracking()
                    self._last_sample = -math.inf
                    self._reason = "waiting_for_face"
                    self._track = None
                    self._last_frame = None
            elif action != "info":
                return None
            reason = self._reason
            if self._armed and time.monotonic() - max(
                self._armed_at, self._last_sample
            ) > FRAME_MAX_AGE:
                reason = "camera_timeout"
            state = "idle"
            if self._node is not None:
                state = "running" if self._armed else "ready"
            return {
                "state": state,
                "armed": self._armed,
                "mode": self._mode,
                "object_name": self._object_name,
                "reason": reason,
                "topic_in": [{**INPUT, "topic": self._topic}] if self._topic else [],
            }

    def _start_geometry(self):
        node = Node("tianyi2_follow_geometry", context=self._ros2.ctx_tianyi)
        qos = QoSProfile(depth=2, reliability=ReliabilityPolicy.BEST_EFFORT)
        try:
            node.create_subscription(Image, "/ob_camera_head/depth/image_raw",
                                     lambda msg: self._depth_frame(node, msg), qos)
            if not self._ros2.executor_tianyi.add_node(node):
                raise RuntimeError("Could not register follow geometry node")
        except Exception:
            node.destroy_node()
            raise
        self._depth_node = node

    def _depth_frame(self, node, msg):
        with self._lock:
            if node is self._depth_node and self._mode == "nearest_face" and self._armed:
                self._depth_frames = (self._depth_frames + [msg])[-4:]

    def _objects(self, node, msg):
        with self._lock:
            if node is not self._node or not self._armed or not self._object_name:
                return
            self._object_packet = None
            try:
                payload = json.loads(msg.data)
                ts = float(payload["timestamp"])
                latency = float(payload.get("latency_ms", 0)) / 1000
                age = time.time() - ts
                if (ts <= self._object_stamp
                        or not (0 <= age and 0 <= latency and age + latency <= FRAME_MAX_AGE)):
                    return
                if payload.get("error") or payload.get("available") is False:
                    return
                points = []
                for obj in payload["objects"]:
                    if obj["name"].casefold() != self._object_name:
                        continue
                    x, y = obj["position"]
                    confidence = obj["confidence"]
                    values = (x, y, confidence)
                    if not all(type(value) in (int, float) for value in values):
                        continue
                    if -1 <= x <= 1 and -1 <= y <= 1 and 0.5 <= confidence <= 1:
                        points.append((x, y))
                self._object_stamp = ts
                self._object_packet = (points, time.monotonic() + FRAME_MAX_AGE - age - latency)
            except (ValueError, KeyError, TypeError, AttributeError, OverflowError):
                return

    def _on_frame(self, node, message):
        received = time.monotonic()
        with self._lock:
            if self._node is not node or not self._armed or received < self._armed_at:
                return
            if received - self._last_sample < 0.12:
                return
            self._last_sample = received
            detector, tracker = self._detector, self._tracker
            name = self._object_name
            mode = self._mode
            depth_frames = self._depth_frames
            if name:
                packet = self._object_packet
                self._object_packet = None
                if packet is None or received > packet[1]:
                    self._reason = "waiting_for_object_detection"
                    return
        try:
            if name:
                boxes, size = (), None
            else:
                boxes, size = detector.detect(message.data)
                if mode == "nearest_face" and boxes:
                    _, stamp = _stamp(message)
                    depth = min(depth_frames, key=lambda frame: abs(_stamp(frame)[1] - stamp),
                                default=None)
                    nearest = (_nearest_face(boxes, size, message, depth, detector.np)
                               if depth is not None else None)
                    boxes = [nearest] if nearest is not None else []
        except (ValueError, detector.cv2.error):
            boxes, size = (), None
        with self._lock:
            if self._node is not node or not self._armed or self._tracker is not tracker:
                return
            if ((not name and size is None) or time.monotonic() - received > FRAME_MAX_AGE
                    or (name and time.monotonic() > packet[1])):
                self._track = None
                self._reason = "invalid_or_stale_frame"
                return
            if mode == "nearest_face" and boxes and tracker.bbox and _iou(tracker.bbox, boxes[0]) < 0.2:
                tracker.bbox = None
            target = (tracker.update_points(packet[0], received) if name
                      else tracker.update(boxes, size, received))
            if target is None:
                self._track = None
                self._reason = {"object": "waiting_for_object",
                                "nearest_face": "waiting_for_registered_face_depth"}.get(
                                    mode, "waiting_for_face")
                return
            track = target["track_id"]
            offset = target["offset"]
            last = self._last_frame
            confirmed = self._track == track and last is not None and received - last <= FRAME_MAX_AGE
            self._last_frame = received
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
