"""Select anonymous visual targets independently of motor transport."""

import math
import zlib

MAX_AGE = 0.5
WIDTH, HEIGHT = 640, 480


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
        if size != self.size or now - self.last_seen >= MAX_AGE:
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
        if now - self.last_seen >= MAX_AGE:
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


def image_stamp(message):
    stamp = message.header.stamp
    return stamp.sec + stamp.nanosec / 1e9


def nearest_face(boxes, size, rgb, depth, np):
    if depth is None or size != (WIDTH, HEIGHT) or not rgb.header.frame_id:
        return None
    # Require a registered RGB-D producer. Native depth optical frames are
    # deliberately rejected even when dimensions happen to agree.
    if (rgb.header.frame_id != depth.header.frame_id or depth.format != "image/depth-zlib"
            or not image_stamp(rgb) or not image_stamp(depth)
            or abs(image_stamp(rgb) - image_stamp(depth)) > 0.085):
        return None
    expected = WIDTH * HEIGHT * 2
    decoder = zlib.decompressobj()
    try:
        raw = decoder.decompress(bytes(depth.data), expected + 1)
    except zlib.error:
        return None
    if len(raw) != expected or not decoder.eof or decoder.unused_data:
        return None
    pixels = np.frombuffer(raw, dtype="<u2").reshape(HEIGHT, WIDTH)
    candidates = []
    for box in boxes:
        x, y, w, h = box
        patch = pixels[max(0, y + h//3):y + 2*h//3, max(0, x + w//3):x + 2*w//3]
        valid = patch[(patch >= 200) & (patch <= 8000)]
        if valid.size >= max(5, patch.size // 3):
            candidates.append((float(np.median(valid)), box))
    return min(candidates) if candidates else None


def finite(value):
    try:
        return type(value) in (float, int) and math.isfinite(value)
    except OverflowError:
        return False


class FollowPolicy:
    def __init__(self, mode, label=""):
        self.mode = mode
        self.label = label
        self.tracker = _Tracker()
        self.speech = None
        self.utterance = None
        self.speech_stamp = -math.inf
        self.locked = False
        self.object_stamp = -math.inf

    def faces(self, boxes, size, rgb, depth, np, now, wall):
        tracker = self.tracker
        distance_mm = None
        if self.mode == "nearest_face":
            result = nearest_face(boxes, size, rgb, depth, np)
            if result is None:
                return None, "waiting_for_registered_face_depth"
            distance_mm, candidate = result
            if tracker.bbox and _iou(tracker.bbox, candidate) < 0.2:
                tracker.bbox = None
            boxes = [candidate]
        else:
            if self.speech is None or not 0 <= wall - self.speech_stamp <= MAX_AGE:
                return None, "waiting_for_speech"
            if not self.locked:
                ranked = sorted((abs((2*x+w)/size[0]-1 - self.speech), (x,y,w,h))
                                for x, y, w, h in boxes)
                if not ranked or ranked[0][0] > 0.2:
                    return None, "waiting_for_speaker_face"
                if len(ranked) > 1 and ranked[1][0] - ranked[0][0] < 0.08:
                    return None, "ambiguous_speaker"
                boxes = [ranked[0][1]]
                self.locked = True
            elif tracker.size != size or now - tracker.last_seen >= MAX_AGE:
                return None, "speaker_face_lost"
        target = tracker.update(boxes, size, now)
        if target is not None and distance_mm is not None:
            target["distance_mm"] = distance_mm
        return target, "tracking" if target else "target_lost"

    def speech_event(self, payload, wall):
        stamp, active = payload.get("timestamp"), payload.get("active")
        utterance = payload.get("utterance_id")
        if (not finite(stamp) or not 0 <= wall - stamp <= MAX_AGE or stamp <= self.speech_stamp
                or type(active) is not bool or not isinstance(utterance, str) or not utterance):
            return False
        if self.utterance is not None and utterance != self.utterance:
            return False
        if not active:
            self.utterance = self.speech = None
            self.speech_stamp = stamp
            self.locked = False
            self.tracker = _Tracker()
            return True
        offset = payload.get("image_x")
        if not finite(offset) or not -1 <= offset <= 1:
            return False
        self.speech_stamp = stamp
        self.speech = offset
        self.utterance = utterance
        return True

    def objects(self, payload, wall, now):
        stamp = payload.get("timestamp")
        latency = payload.get("latency_ms", 0)
        if (not finite(stamp) or not finite(latency) or latency < 0
                or stamp <= self.object_stamp or not 0 <= wall - stamp
                or wall - stamp + latency / 1000 > MAX_AGE):
            return None, "stale_object_frame"
        values = payload.get("objects")
        if not isinstance(values, list) or payload.get("error") or payload.get("available") is False:
            return None, "unavailable_object_frame"
        self.object_stamp = stamp
        points = []
        for value in values:
            if not isinstance(value, dict) or str(value.get("name", "")).casefold() != self.label:
                continue
            point, confidence = value.get("position"), value.get("confidence")
            if (isinstance(point, list) and len(point) == 2
                    and all(finite(x) and -1 <= x <= 1 for x in point)
                    and finite(confidence) and 0.5 <= confidence <= 1):
                points.append(tuple(point))
        target = self.tracker.update_points(points, now)
        return target, "tracking" if target else "waiting_for_object"
