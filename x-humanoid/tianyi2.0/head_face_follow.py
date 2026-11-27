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
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from std_msgs.msg import String
from urllib.request import Request, urlopen

FRAME_MAX_AGE = 0.4
DEPTH_MAX_SKEW = 0.085
SPEECH_TIMEOUT = 0.55
MODES = ("face", "nearest_face", "speaker", "object")


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


def _face_at_bearing(boxes, intrinsics, bearing):
    _, fx, cx = intrinsics
    ranked = sorted((abs(math.degrees(math.atan2(cx - (x + w / 2), fx)) - bearing),
                     (x, y, w, h)) for x, y, w, h in boxes)
    if not ranked or ranked[0][0] > 12:
        return None
    if len(ranked) > 1 and ranked[1][0] - ranked[0][0] < 5:
        return None
    return ranked[0][1]


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
        self._mode = "face"
        self._depth_node = None
        self._depth_frames = []
        self._intrinsics = None
        self._speech = None
        self._speech_stamp = 0
        self._speaker_locked = False
        self._speaker_topic = f"/{namespace}/head_face_follow/speech"
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
            self._intrinsics = None
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
        self._speech = None
        self._speaker_locked = False
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
                        node.create_subscription(
                            String, self._speaker_topic, lambda msg: self._speech_event(node, msg), qos)
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
                if mode in ("nearest_face", "speaker") and self._topic != self._default_topic:
                    return {"state": "error", "error": "Start the card with its built-in head camera"}
                if mode in ("nearest_face", "speaker") and self._depth_node is None:
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
            return {
                "state": "running" if self._node else "idle",
                "armed": self._armed,
                "mode": self._mode,
                "speech_topic": self._speaker_topic,
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
            node.create_subscription(CameraInfo, "/ob_camera_head/color/camera_info",
                                     lambda msg: self._camera_info(node, msg), qos)
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

    def _camera_info(self, node, msg):
        with self._lock:
            if (node is self._depth_node and math.isfinite(msg.k[0]) and msg.k[0] > 0
                    and math.isfinite(msg.k[2])):
                self._intrinsics = ((msg.width, msg.height), msg.k[0], msg.k[2])

    def _speech_event(self, node, msg):
        # A calibrated provider emits current camera-relative VAD and azimuth.
        try:
            data = json.loads(msg.data)
            ts = float(data["ts"])
            active = data["active"]
            if type(active) is not bool or not 0 <= time.time() - ts <= SPEECH_TIMEOUT:
                return
            bearing = float(data["camera_bearing_deg"]) if active else None
            if active and not -90 <= bearing <= 90:
                return
        except (ValueError, KeyError, TypeError, OverflowError):
            return
        with self._lock:
            if node is not self._node or not self._armed or self._mode != "speaker":
                return
            if ts <= self._speech_stamp:
                return
            self._speech_stamp = ts
            if not active or (self._speech and ts - self._speech[0] > SPEECH_TIMEOUT):
                self._reset_tracking()
            self._speech = (ts, bearing) if active else None

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
                if (ts <= self._object_stamp
                        or not (0 <= age and 0 <= latency and age + latency <= FRAME_MAX_AGE)):
                    return
                boxes = []
                for obj in payload["objects"]:
                    if obj["name"].casefold() == self._object_name:
                        x, y, right, bottom = map(float, obj["bbox"])
                        if all(map(math.isfinite, (x, y, right, bottom))) and right > x and bottom > y:
                            boxes.append((x, y, right - x, bottom - y))
                self._object_stamp = ts
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
            mode = self._mode
            depth_frames = self._depth_frames
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
            if size is None or time.monotonic() - received > FRAME_MAX_AGE or (name and time.monotonic() > packet[2]):
                self._track = None
                self._reason = "invalid_or_stale_frame"
                return
            if mode == "speaker":
                if not self._speech or time.time() - self._speech[0] > SPEECH_TIMEOUT:
                    self._reset_tracking()
                    self._reason = "waiting_for_speech"
                    return
                if not self._speaker_locked:
                    candidate = (_face_at_bearing(boxes, self._intrinsics, self._speech[1])
                                 if self._intrinsics and self._intrinsics[0] == size else None)
                    boxes = [candidate] if candidate is not None else []
                    self._speaker_locked = bool(boxes)
                elif size != tracker.size or received - tracker.last_seen >= FRAME_MAX_AGE:
                    # Hold identity through the utterance, even when vision drops it.
                    self._reason = "speaker_face_lost"
                    return
            if mode == "nearest_face" and boxes and tracker.bbox and _iou(tracker.bbox, boxes[0]) < 0.2:
                tracker.bbox = None
            target = tracker.update(boxes, size, received)
            if target is None:
                self._track = None
                self._reason = {"object": "waiting_for_object", "speaker": "waiting_for_speaker_face",
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
