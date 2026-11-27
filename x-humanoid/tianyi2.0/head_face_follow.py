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
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String

FRAME_MAX_AGE = 0.4
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
        self._object_name = ""
        self._object_packet = None
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
                    "action": {"type": "string", "enum": ["start", "arm", "stop", "info", "set_target"]},
                    "object_name": {"type": "string", "description": "VOP 类别名。空字符串切回人脸"},
                    "input_topic": {"type": "string", "description": "可选 JPEG 相机话题，留空使用内置头部相机"},
                },
                "required": ["action"],
                "x-resource": "head",
                "x-action-params": {
                    "start": {"params": ["input_topic"], "description": "启动相机订阅，等待开启"},
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
            self._tracker = None
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
            elif action == "set_target":
                name = args.get("object_name")
                if not isinstance(name, str):
                    return {"state": "error", "error": "Provide a VOP class name"}
                self._object_name = name.strip().casefold()
                self._object_packet = None
                self._tracker = _Tracker()
                self._track = self._last_frame = None
            elif action == "arm":
                if self._node is None:
                    return {"state": "error", "error": "Start the card first"}
                if not self._armed:
                    check = self._head.face_follow_ready()
                    if check:
                        return check
                    self._armed = True
                    self._armed_at = time.monotonic()
                    self._tracker = _Tracker()
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
            return {
                "state": "running" if self._node else "idle",
                "armed": self._armed,
                "object_name": self._object_name,
                "reason": reason,
                "topic_in": [{**INPUT, "topic": self._topic}] if self._topic else [],
            }

    def _objects(self, node, msg):
        with self._lock:
            if node is not self._node or not self._armed or not self._object_name:
                return
            self._object_packet = None
            try:
                payload = json.loads(msg.data)
                ts = float(payload["timestamp"])
                latency = float(payload["latency_ms"]) / 1000
                age = time.time() - ts
                if not (0 <= age and 0 <= latency and age + latency <= FRAME_MAX_AGE):
                    return
                boxes = []
                for obj in payload["objects"]:
                    if obj["name"].casefold() == self._object_name:
                        x, y, right, bottom = map(float, obj["bbox"])
                        if all(map(math.isfinite, (x, y, right, bottom))) and right > x and bottom > y:
                            boxes.append((x, y, right - x, bottom - y))
                self._object_packet = (ts, boxes, time.monotonic() + FRAME_MAX_AGE - age - latency)
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
            if name:
                packet = self._object_packet
                self._object_packet = None
                if packet is None or received > packet[2]:
                    self._reason = "waiting_for_object_detection"
                    return
        try:
            if name:
                image = detector.cv2.imdecode(detector.np.frombuffer(message.data, dtype=detector.np.uint8),
                                              detector.cv2.IMREAD_GRAYSCALE)
                if image is None:
                    raise ValueError("Invalid camera frame")
                size = (image.shape[1], image.shape[0])
                boxes = [b for b in packet[1] if b[0] >= 0 and b[1] >= 0
                         and b[0] + b[2] <= size[0] and b[1] + b[3] <= size[1]]
            else:
                boxes, size = detector.detect(message.data)
        except (ValueError, detector.cv2.error):
            boxes, size = (), None
        with self._lock:
            if self._node is not node or not self._armed or self._tracker is not tracker:
                return
            if size is None or time.monotonic() - received > FRAME_MAX_AGE or (name and time.monotonic() > packet[2]):
                self._track = None
                self._reason = "invalid_or_stale_frame"
                return
            target = tracker.update(boxes, size, received)
            if target is None:
                self._track = None
                self._reason = "waiting_for_face"
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
