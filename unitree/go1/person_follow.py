"""Go1 follow target selection and conservative image-based steering."""

from __future__ import annotations

from dataclasses import dataclass, replace
from io import BytesIO
from pathlib import Path
import select
import socket
import struct
import threading
import time

try:
    from . import camera
except ImportError:  # Go1 容器以 /work 为模块根目录运行。
    import camera


@dataclass(frozen=True)
class Detection:
    kind: str
    x1: float
    y1: float
    x2: float
    y2: float
    score: float
    appearance: tuple[int, ...] | None = None

    @property
    def center_x(self):
        return (self.x1 + self.x2) / 2


def _candidates(detections):
    people = [d for d in detections if d.kind == "person" and d.score >= .5]
    shoes = sorted((d for d in detections if d.kind == "shoe" and d.score >= .5),
                   key=lambda d: d.x1)
    result = list(people)
    used = set()
    for i, shoe in enumerate(shoes):
        if i in used or any(p.x1 <= shoe.center_x <= p.x2 and shoe.y2 <= p.y2 + .08
                            for p in people):
            continue
        partner = next((j for j in range(i + 1, len(shoes)) if j not in used
                        and abs(shoes[j].y2 - shoe.y2) <= .06
                        and 0 < shoes[j].x1 - shoe.x2 <= .12), None)
        if partner is None:
            result.append(shoe)
            continue
        other = shoes[partner]
        used.add(partner)
        result.append(Detection("shoe", shoe.x1, min(shoe.y1, other.y1),
                                other.x2, max(shoe.y2, other.y2),
                                min(shoe.score, other.score), shoe.appearance))
    return result


def _appearance_matches(previous, current):
    if previous.appearance is None or current.appearance is None:
        return True
    if len(previous.appearance) != len(current.appearance):
        return False
    difference = sum(abs(a - b) for a, b in zip(previous.appearance, current.appearance))
    return difference <= .2 * 255 * len(previous.appearance)


class TargetTracker:
    """Acquire the apparent nearest target once; never silently change identity."""

    def __init__(self):
        self.target = None
        self.lost = False

    def update(self, detections):
        candidates = _candidates(detections)
        if self.lost:
            return None
        if self.target is None:
            candidates.sort(key=lambda d: d.y2, reverse=True)
            if not candidates or (len(candidates) > 1 and
                                  candidates[0].y2 - candidates[1].y2 < .05):
                return None
            self.target = candidates[0]
            return self.target
        # 中文说明：锁定后只接受位置连续的目标；旁人接近时按位移比较，难以区分就停车。
        matches = sorted(((abs(d.center_x - self.target.center_x) +
                           abs(d.y2 - self.target.y2), d) for d in candidates
                          if abs(d.center_x - self.target.center_x) <= .15
                          and abs(d.y2 - self.target.y2) <= .15
                          and (d.kind != self.target.kind or
                               _appearance_matches(self.target, d))), key=lambda item: item[0])
        if not matches or (len(matches) > 1 and matches[1][0] - matches[0][0] < .08):
            self.target = None
            self.lost = True
            return None
        self.target = matches[0][1]
        return self.target


def follow_command(target):
    """Return (forward m/s, yaw rad/s); image geometry is only a distance proxy."""
    # 远景样本的脚点在 0.53–0.56；过远或过近都保持停车，且不原地追转。
    if target is None or target.y2 < .58 or target.y2 >= .68:
        return 0.0, 0.0
    vx = min(.15, max(0.0, (.68 - target.y2) * .6))
    error_x = target.center_x - .5
    yaw = 0.0 if abs(error_x) <= .08 else max(-.3, min(.3, -error_x * .8))
    return vx, yaw


def decode_yolox(output, image_width, image_height, resize_ratio):
    """Decode a raw 416px YOLOX head with classes [person, shoe]."""
    import numpy as np

    raw = np.asarray(output)
    if raw.shape != (1, 3549, 7):
        raise ValueError("YOLOX model must have two classes [person, shoe] and a raw 416px head")
    grids = []
    strides = []
    for stride in (8, 16, 32):
        side = 416 // stride
        y, x = np.meshgrid(np.arange(side), np.arange(side), indexing="ij")
        grids.append(np.stack((x, y), axis=-1).reshape(-1, 2))
        strides.append(np.full((side * side, 1), stride))
    grid = np.concatenate(grids)
    stride = np.concatenate(strides)
    rows = raw[0]
    scores = rows[:, 4:5] * rows[:, 5:7]
    selected = np.argwhere(scores >= .5)
    if len(selected) == 0:
        return []
    selected = sorted(selected, key=lambda pair: scores[pair[0], pair[1]], reverse=True)[:100]
    proposals = []
    for row_index, class_index in selected:
        center = (rows[row_index, :2] + grid[row_index]) * stride[row_index, 0]
        size = np.exp(np.clip(rows[row_index, 2:4], -10, 10)) * stride[row_index, 0]
        x1, y1 = (center - size / 2) / resize_ratio
        x2, y2 = (center + size / 2) / resize_ratio
        box = Detection(("person", "shoe")[class_index],
                        max(0.0, min(1.0, float(x1 / image_width))),
                        max(0.0, min(1.0, float(y1 / image_height))),
                        max(0.0, min(1.0, float(x2 / image_width))),
                        max(0.0, min(1.0, float(y2 / image_height))),
                        float(scores[row_index, class_index]))
        if box.x2 > box.x1 and box.y2 > box.y1:
            proposals.append(box)
    kept = []
    for box in proposals:
        if any(box.kind == prior.kind and _overlap(box, prior) > .45 for prior in kept):
            continue
        kept.append(box)
    return kept


def _overlap(a, b):
    width = max(0.0, min(a.x2, b.x2) - max(a.x1, b.x1))
    height = max(0.0, min(a.y2, b.y2) - max(a.y1, b.y1))
    intersection = width * height
    area_a = (a.x2 - a.x1) * (a.y2 - a.y1)
    area_b = (b.x2 - b.x1) * (b.y2 - b.y1)
    return intersection / (area_a + area_b - intersection) if intersection else 0.0


class YoloXOnnxDetector:
    """Run a two-class YOLOX-Nano ONNX model with a single CPU inference thread."""

    def __init__(self, model_path):
        import onnxruntime as ort

        options = ort.SessionOptions()
        options.intra_op_num_threads = 1
        options.inter_op_num_threads = 1
        self._session = ort.InferenceSession(str(model_path), sess_options=options,
                                             providers=["CPUExecutionProvider"])
        self._input = self._session.get_inputs()[0]
        if self._input.shape != [1, 3, 416, 416]:
            raise ValueError("YOLOX model input must be [1, 3, 416, 416]")

    def detect(self, jpeg):
        from PIL import Image
        import numpy as np

        image = Image.open(BytesIO(jpeg)).convert("RGB")
        width, height = image.size
        ratio = min(416 / width, 416 / height)
        resized = image.resize((int(width * ratio), int(height * ratio)))
        canvas = np.full((416, 416, 3), 114, dtype=np.uint8)
        rgb = np.asarray(resized)
        canvas[:rgb.shape[0], :rgb.shape[1]] = rgb[:, :, ::-1]
        tensor = np.ascontiguousarray(canvas.transpose(2, 0, 1), dtype=np.float32)[None]
        output = self._session.run(None, {self._input.name: tensor})[0]
        detections = decode_yolox(output, width, height, ratio)
        # 中文说明：只保留每个框的 4×4 色彩摘要，供锁定后的身份连续性检查。
        result = []
        for box in detections:
            x1, y1 = int(box.x1 * width), int(box.y1 * height)
            x2, y2 = max(x1 + 1, int(box.x2 * width)), max(y1 + 1, int(box.y2 * height))
            pixels = image.crop((x1, y1, x2, y2)).resize((4, 4)).getdata()
            result.append(replace(box, appearance=tuple(channel for pixel in pixels
                                                        for channel in pixel)))
        return result


def extract_latest_frame(pending):
    """Discard old complete JPEGs while retaining an incomplete TCP tail."""
    latest = None
    while len(pending) >= 4:
        length = struct.unpack(">I", pending[:4])[0]
        if length <= 0 or length > 5_000_000:
            raise ValueError("invalid camera frame length")
        if len(pending) < 4 + length:
            break
        latest = bytes(pending[4:4 + length])
        del pending[:4 + length]
    return latest


class PersonFollowPlugin:
    """One camera, one tracked target, one cancellable motion loop."""

    def __init__(self, plugin_config, namespace=None, executor=None, client=None):
        del namespace, executor
        self._client = client
        self._model_path = Path(plugin_config.get("model_path") or "")
        self._has_model_path = bool(plugin_config.get("model_path"))
        endpoint = camera._resolve_positions_raw(plugin_config)["front"]
        self._endpoint = endpoint["board_ip"], int(endpoint["image_port"])
        self._cancel = threading.Event()
        self._lock = threading.Lock()
        self._motion_lock = threading.Lock()
        self._owns_motion = False
        self._worker = None
        self._state = "ready"
        self._reason = None

    def get_tool(self):
        # 中文说明：follow 是持续状态的启动动作；不占 ACP 异步屏障，stop 和 loco.stop 才能随时接管。
        return {"name": "person_follow", "type": "actuator", "multiInstance": False,
                "description": "Follow the apparent nearest person, including a shoe-only view; stop on uncertainty.",
                "inputSchema": {
                    "type": "object", "required": ["action"], "additionalProperties": False,
                    "x-is-dangerous": True,
                    "properties": {
                        "action": {"type": "string", "enum": ["start", "follow", "stop", "info"]},
                        "confirm": {"type": "boolean", "description": "跟随运动前需明确确认。"},
                    },
                    "x-action-params": {
                        "start": {"params": [], "description": "准备跟随卡；不会让机器人运动。"},
                        "follow": {"params": ["confirm"], "description": "启动持续跟随；stop 结束，info 查询状态。"},
                        "stop": {"params": [], "description": "立即停车并停止跟随。"},
                        "info": {"params": [], "description": "查看跟随状态。"},
                    },
                }}

    def start(self):
        return {"state": "ready"}

    def stop(self):
        self._cancel.set()
        with self._motion_lock:
            if self._owns_motion:
                self._client.stop_move()
                self._owns_motion = False
        with self._lock:
            worker = self._worker
        if worker is not None and worker is not threading.current_thread():
            worker.join(timeout=2)
        with self._lock:
            self._state = "idle"
        return {"ok": True, "state": "idle"}

    def preempt(self):
        with self._lock:
            active = self._worker is not None and self._worker.is_alive()
        if active:
            self.stop()

    def dispatch(self, action, args):
        args = args or {}
        if action == "start":
            return self.start()
        if action == "stop":
            return self.stop()
        if action == "info":
            with self._lock:
                return {"ok": True, "state": self._state, "reason": self._reason,
                        "model_ready": self._has_model_path and self._model_path.is_file()}
        if action != "follow":
            return None
        if args.get("confirm") is not True:
            return {"ok": False, "code": "PRECONDITION_FAILED", "message": "confirm=true is required"}
        if not self._has_model_path or not self._model_path.is_file():
            return {"ok": False, "code": "MODEL_UNAVAILABLE", "message": "shoe/person ONNX model is missing"}
        if self._client is None or not self._client.available:
            return {"ok": False, "code": "ROBOT_UNAVAILABLE", "message": "Go1 SDK is unavailable"}
        # 中文说明：与拍照和推流共用机位锁；跟随占用 front 时不能再打开同机位相机。
        with camera._CAMERA_LOCK:
            if "front" in camera._SNAPSHOT_POSITIONS or camera.running_stream("front") is not None:
                return {"ok": False, "code": "RESOURCE_BUSY", "message": "front camera is occupied"}
            with self._lock:
                if self._worker is not None and self._worker.is_alive():
                    return {"ok": False, "code": "RESOURCE_BUSY", "message": "already following"}
                camera._SNAPSHOT_POSITIONS.add("front")
                self._cancel.clear()
                self._state = "starting"
                self._reason = None
                with self._motion_lock:
                    self._owns_motion = True
                self._worker = threading.Thread(target=self._run_guarded, daemon=True,
                                                name="go1_person_follow")
                self._worker.start()
        return {"ok": True, "state": "starting"}

    def _run_guarded(self):
        try:
            self._run()
        except Exception as exc:
            if not self._cancel.is_set():
                with self._lock:
                    self._state = "error"
                    self._reason = str(exc)
        finally:
            self._cancel.set()
            with self._motion_lock:
                if self._owns_motion:
                    self._client.stop_move()
                    self._owns_motion = False
            with camera._CAMERA_LOCK:
                camera._SNAPSHOT_POSITIONS.discard("front")
            with self._lock:
                if self._state not in ("error", "idle"):
                    self._state = "idle"

    def _run(self):
        detector = YoloXOnnxDetector(self._model_path)
        if self._cancel.is_set():
            return
        tracker = TargetTracker()
        host, port = self._endpoint
        with socket.create_connection((host, port), timeout=8) as connection:
            connection.setblocking(False)
            if self._cancel.is_set():
                return
            pending = bytearray()
            last_frame = time.monotonic()
            first_frame = False
            previous_recv = None
            with self._lock:
                self._state = "following"
            while not self._cancel.is_set():
                readable, _, _ = select.select([connection], [], [], .1)
                if readable:
                    # 中文说明：推理慢于相机时读空接收队列，只处理最新完整帧。
                    received = 0
                    while received < 2_097_152:
                        try:
                            chunk = connection.recv(65536)
                        except BlockingIOError:
                            break
                        if not chunk:
                            raise ConnectionError("front camera disconnected")
                        pending.extend(chunk)
                        received += len(chunk)
                    if received >= 2_097_152:
                        raise RuntimeError("camera frames are backing up")
                latest = extract_latest_frame(pending)
                if latest is None:
                    if time.monotonic() - last_frame > .5:
                        self._stop_owned_motion()
                        if first_frame:
                            raise TimeoutError("camera frame is stale")
                    if not first_frame and time.monotonic() - last_frame > 10:
                        raise TimeoutError("front camera produced no frame")
                    continue
                first_frame = True
                last_frame = time.monotonic()
                detections = detector.detect(latest)
                if self._cancel.is_set():
                    break
                if time.monotonic() - last_frame > .4:
                    self._stop_owned_motion()
                    raise TimeoutError("inference is too slow")
                target = tracker.update(detections)
                if tracker.lost:
                    self._stop_owned_motion()
                    raise RuntimeError("target lost or ambiguous")
                vx, yaw = follow_command(target)
                if vx == 0 and yaw == 0:
                    self._stop_owned_motion()
                    continue
                diag = self._client.diagnostics()
                recv = diag.get("recv_count", 0)
                if (not diag.get("accessible") or
                        (previous_recv is not None and recv <= previous_recv) or
                        time.monotonic() - last_frame > .4):
                    self._stop_owned_motion()
                    raise RuntimeError("Go1 feedback or camera frame is stale")
                previous_recv = recv
                with self._motion_lock:
                    if self._cancel.is_set() or not self._owns_motion:
                        break
                    result = self._client.move(vx, 0.0, yaw, gait=1)
                if result is None:
                    raise RuntimeError("Go1 rejected the follow command")

    def _stop_owned_motion(self):
        with self._motion_lock:
            if self._owns_motion:
                self._client.stop_move()


def make_person_follow(plugin_config, namespace, executor, client):
    return PersonFollowPlugin(plugin_config, namespace, executor, client)
