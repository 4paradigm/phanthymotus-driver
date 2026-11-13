"""R1 photos and videos captured from the camera card's existing streams.

Contract follows `robotera/q5_bundle` / `unitree/go2` `vision_capture`: the same
four action names, the same slim result fields, and the same asynchronous
completion for `record_video`. Only the source side is R1-specific.

R1's cameras are published by `device.CameraPlugin`'s gstreamer subprocess on
`/{namespace}/camera/{main,left,right,depth}`. This card subscribes to those
ports — it never opens a camera device, so adding it cannot disturb the four
streams that the canvas is already showing.

`camera_depth` is deliberately not a source. That port carries a depth
*visualisation* encoded as JPEG for the dashboard, not an RGB frame; saving it
as a photo or encoding it into an MP4 would produce a file nobody can use and
no error to explain why.

Storage is a host mount, not the container's own filesystem: a container
recreation — which every deploy does — would otherwise delete the photos the
operator just took. `deploy/service.yml` mounts `/opt/phanthy-motus/data/r1`.
"""

from __future__ import annotations

import json
import logging
import os
from datetime import datetime
from pathlib import Path
import select
import shutil
import ssl
import subprocess
import tempfile
import threading
import time
import urllib.request
from uuid import uuid4

from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage


# Ports offered by the R1 camera card. `depth` is excluded on purpose; see the
# module docstring.
SOURCES = {
    "main": "camera_main",
    "left": "camera_left",
    "right": "camera_right",
}

FIRST_FRAME_TIMEOUT_S = 5.0
MAX_FRAME_AGE_S = 3.0

log = logging.getLogger(__name__)


def _timestamp() -> str:
    return datetime.now().astimezone().isoformat(timespec="seconds")


def jpeg_size(data: bytes):
    """(width, height) from a JPEG's SOF marker, or None.

    Kept local instead of pulling Pillow into the driver image: the only thing
    this card needs from the frame is its dimensions, and a wrong size is
    cosmetic, so a parser that returns None on anything unexpected is enough.
    """
    if len(data) < 4 or data[0:2] != b"\xff\xd8":
        return None
    index = 2
    size = len(data)
    while index + 3 < size:
        if data[index] != 0xFF:
            index += 1
            continue
        marker = data[index + 1]
        if marker in (0xD8, 0xD9, 0x01) or 0xD0 <= marker <= 0xD7:
            index += 2
            continue
        if index + 4 > size:
            return None
        length = int.from_bytes(data[index + 2:index + 4], "big")
        if marker in (0xC0, 0xC1, 0xC2, 0xC3, 0xC5, 0xC6, 0xC7,
                      0xC9, 0xCA, 0xCB, 0xCD, 0xCE, 0xCF):
            if index + 9 > size:
                return None
            height = int.from_bytes(data[index + 5:index + 7], "big")
            width = int.from_bytes(data[index + 7:index + 9], "big")
            return (width, height)
        if length < 2:
            return None
        index += 2 + length
    return None


# Admittedly blunt, but this is not a semantic verdict: it only catches the two
# failures that make the *next* step fail — `vop.recognize_by_photo` returns
# nothing useful from a black or smeared frame, and without this the skill has no
# way to tell "no cup on the table" from "the photo was unusable".
DARK_BELOW = 35.0
BLUR_BELOW = 30.0


def quality_verdict(brightness: float, sharpness: float) -> dict:
    """Advisory capture quality. Never used to reject a photo by itself."""
    if brightness < DARK_BELOW:
        hint = "画面偏暗，识别会漏；建议补光后重拍"
    elif sharpness < BLUR_BELOW:
        hint = "画面模糊，识别会漏；建议保持稳定后重拍"
    else:
        hint = ""
    return {"brightness": round(float(brightness), 1),
            "sharpness": round(float(sharpness), 1),
            "ok": not hint, "hint": hint}


def image_quality(data: bytes):
    """`quality_verdict` for a JPEG, or None when OpenCV is unavailable.

    OpenCV is in the R1 image already (the camera path uses it elsewhere); the
    import stays lazy so this module still imports in a laptop test env.
    """
    try:
        import cv2
        import numpy
    except Exception:
        return None
    buffer = numpy.frombuffer(data, dtype=numpy.uint8)
    gray = cv2.imdecode(buffer, cv2.IMREAD_GRAYSCALE)
    if gray is None:
        return None
    brightness = float(gray.mean())
    sharpness = float(cv2.Laplacian(gray, cv2.CV_64F).var())
    return quality_verdict(brightness, sharpness)


def _atomic_write(path: Path, data: bytes) -> None:
    """Write beside the target, then rename: a reader never sees half a JPEG."""
    partial = path.with_name(path.name + ".part")
    try:
        partial.write_bytes(data)
        os.replace(partial, path)
    finally:
        if partial.exists():
            try:
                partial.unlink()
            except OSError:
                pass


class FrameStore:
    """Newest JPEG per camera port, with a wait for a frame newer than a mark.

    A card that returns a cached frame turns "take a photo" into "give me
    whatever the camera last saw", which after a canvas restart is a picture of
    an empty room. `wait` therefore takes the sequence observed *before* the
    request and only returns when a later frame arrives.
    """

    def __init__(self):
        self._condition = threading.Condition()
        self._frames = {}

    def _entry(self, topic: str) -> dict:
        return self._frames.setdefault(
            topic, {"sequence": 0, "data": b"", "received": 0.0, "count": 0})

    def put(self, topic: str, data: bytes) -> None:
        with self._condition:
            entry = self._entry(topic)
            entry["sequence"] += 1
            entry["data"] = bytes(data)
            entry["received"] = time.monotonic()
            entry["count"] += 1
            self._condition.notify_all()

    def mark(self, topic: str) -> int:
        with self._condition:
            return self._entry(topic)["sequence"]

    def wait(self, topic: str, after: int, timeout: float, cancel=None):
        """(data, received) for the first frame with sequence > `after`."""
        deadline = time.monotonic() + timeout
        with self._condition:
            entry = self._entry(topic)
            while entry["sequence"] <= after:
                if cancel is not None and cancel.is_set():
                    raise RuntimeError("Video recording was cancelled")
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise TimeoutError(
                        f"No fresh frame from {topic} within {timeout:.1f}s "
                        f"(last frame {self._age(entry):.1f}s ago, {entry['count']} received)")
                self._condition.wait(min(0.1, remaining))
            return entry["data"], entry["received"]

    def snapshot(self, topic: str) -> dict:
        with self._condition:
            entry = self._entry(topic)
            return {"sequence": entry["sequence"], "count": entry["count"],
                    "age_s": self._age(entry) if entry["count"] else None}

    def clear(self) -> None:
        with self._condition:
            self._frames.clear()

    @staticmethod
    def _age(entry: dict) -> float:
        return time.monotonic() - entry["received"] if entry["received"] else float("inf")


class _CaptureNode(Node):
    def __init__(self, topics: dict, on_frame):
        super().__init__("r1_vision_capture")
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._subscriptions = [
            self.create_subscription(
                CompressedImage, topic,
                (lambda message, camera=camera: on_frame(camera, message)),
                qos,
            )
            for camera, topic in topics.items()
        ]


class VisionCapturePlugin:
    PREFIX = "vision_capture"

    def __init__(self, plugin_config, namespace, executor, completion_sender=None):
        self._namespace = namespace
        self._topics = {camera: f"/{namespace}/camera/{camera}" for camera in SOURCES}
        self._camera = str(plugin_config.get("camera", "main"))
        if self._camera not in SOURCES:
            raise ValueError(f"camera must be one of {sorted(SOURCES)}")
        self._output_dir = Path(plugin_config.get(
            "output_dir", "/opt/phanthy-motus/data/r1/captures")).expanduser()
        self._fps = max(1, min(15, int(plugin_config.get("fps", 10))))
        self._max_duration_s = max(1, min(30, int(plugin_config.get("max_duration_s", 30))))
        # 0 disables pruning. Seven days matches common/file_intake.py.
        self._retention_days = max(0, int(plugin_config.get("retention_days", 7)))

        self._store = FrameStore()
        self._executor = executor
        self._node = None

        self._lock = threading.Lock()
        self._active = None
        self._last = None
        self._last_photo = None
        self._encoder = self._encode_video
        self._probe = self._probe_video
        self._notify = completion_sender or self._notify_complete

    # ── MCP surface ──────────────────────────────────────────────────────────

    def get_tool(self) -> dict:
        camera = {
            "type": "string",
            "enum": list(SOURCES),
            "description": "摄像头：main 主相机 / left 左目 / right 右目；省略时使用卡片配置。",
        }
        return {
            "name": self.PREFIX,
            "type": "actuator",
            "multiInstance": False,
            "description": (
                "把 R1 相机的画面保存到本机：拍照存 JPG，录像存无声 H.264 MP4，"
                "返回文件绝对路径。需要先启动 camera 卡片；深度相机不作为来源。"
            ),
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {"type": "string", "enum": [
                        "start", "stop", "info", "list_cameras", "capture_photo", "record_video"]},
                    "camera": camera,
                    "duration_s": {
                        "type": "integer", "minimum": 1,
                        "maximum": self._max_duration_s,
                        "default": min(5, self._max_duration_s),
                        "description": f"录像时长，默认 5 秒，最大 {self._max_duration_s} 秒。",
                    },
                },
                "required": ["action"],
                "additionalProperties": False,
                "x-action-params": {
                    "start": {"params": [], "description": "订阅相机画面并返回就绪状态。"},
                    "stop": {"params": [], "description": "取消正在进行的录像并删除未完成的文件。"},
                    "info": {"params": [], "description": "查看各相机帧的新鲜度、保存目录、最近一次拍照与录像结果。"},
                    "list_cameras": {"params": [], "description": "列出可用相机及其最近一帧的时间。"},
                    "capture_photo": {"params": ["camera"],
                                      "description": "保存所选相机的一张新照片（JPG），并返回画质评估；偏暗或模糊会提示重拍。"},
                    "record_video": {"params": ["duration_s", "camera"],
                                     "description": "录制所选相机的视频（无声），默认 5 秒。"},
                },
                "x-completion": {"actions": ["record_video"],
                                 "timeout": self._max_duration_s + 15},
            },
            "configSchema": {
                "type": "object",
                "properties": {
                    "camera": {**camera, "default": "main"},
                },
            },
        }

    def start(self) -> dict:
        # Subscribe on start, not in __init__: a stopped card must not keep
        # copying 30 JPEG/s per port, and "stop" on the canvas has to mean the
        # card is no longer reading the bus.
        with self._lock:
            if self._node is None:
                self._node = _CaptureNode(self._topics, self._on_frame)
                self._executor.add_node(self._node)
        threading.Thread(target=self._prune, name="r1_capture_prune", daemon=True).start()
        return {"state": "ready"}

    def stop(self) -> dict:
        cancelled = self._cancel_recording()
        with self._lock:
            node, self._node = self._node, None
        if node is not None:
            try:
                self._executor.remove_node(node)
            finally:
                node.destroy_node()
        self._store.clear()
        return cancelled if cancelled.get("state") == "stopping" else {"state": "idle"}

    def dispatch(self, action: str, args: dict):
        args = {key: value for key, value in (args or {}).items() if key != "_tool_name"}
        if action == "start":
            return self.start()
        if action == "stop":
            return self.stop()
        if action == "info":
            return self._info()
        if action == "list_cameras":
            return self._list_cameras()
        if action == "capture_photo":
            return self._capture_photo(args)
        if action == "record_video":
            return self._record_video(args)
        return None

    # ── frames ───────────────────────────────────────────────────────────────

    def _on_frame(self, camera: str, message) -> None:
        self._store.put(self._topics[camera], message.data)

    def _resolve_camera(self, args: dict) -> str:
        camera = args.get("camera") or self._camera
        if camera not in SOURCES:
            raise ValueError(f"camera must be one of {sorted(SOURCES)}")
        return camera

    # ── photo ────────────────────────────────────────────────────────────────

    def _capture_photo(self, args: dict) -> dict:
        path = None
        try:
            camera = self._resolve_camera(args)
            topic = self._topics[camera]
            mark = self._store.mark(topic)
            data, received = self._store.wait(topic, mark, FIRST_FRAME_TIMEOUT_S)
            path = self._new_path("photos", "IMG", ".jpg")
            _atomic_write(path, data)
            width, height = jpeg_size(data) or (None, None)
            result = {
                "ok": True,
                "media_type": "photo",
                "file_path": str(path),
                "camera": camera,
                "width": width,
                "height": height,
                "bytes": len(data),
                "captured_at": _timestamp(),
                "frame_age_s": round(time.monotonic() - received, 3),
                "quality": image_quality(data),
            }
            with self._lock:
                self._last_photo = result
            return result
        except Exception as exc:
            if path is not None:
                self._remove_partial(path)
            return {"ok": False, "code": "CAPTURE_FAILED", "message": str(exc)}

    # ── video ────────────────────────────────────────────────────────────────

    def _record_video(self, args: dict) -> dict:
        if shutil.which("ffmpeg") is None or shutil.which("ffprobe") is None:
            return {"ok": False, "code": "RECORD_FAILED",
                    "message": "ffmpeg and ffprobe are required"}
        try:
            camera = self._resolve_camera(args)
        except ValueError as exc:
            return {"ok": False, "code": "RECORD_FAILED", "message": str(exc)}
        requested = args.get("duration_s", min(5, self._max_duration_s))
        try:
            requested = int(requested)
        except (TypeError, ValueError):
            return {"ok": False, "code": "RECORD_FAILED",
                    "message": "duration_s must be an integer"}
        requested = max(1, min(self._max_duration_s, requested))

        with self._lock:
            if self._active is not None:
                return {"ok": False, "code": "RECORD_BUSY",
                        "message": "Another recording is in progress",
                        "action_id": self._active["action_id"]}
            active = {
                "action_id": f"vision_capture_record_video_{uuid4().hex}",
                "camera": camera,
                "duration_s": requested,
                "path": self._new_path("videos", "VID", ".mp4"),
                "cancel": threading.Event(),
                "started_at": _timestamp(),
                "thread": None,
            }
            self._active = active

        thread = threading.Thread(target=self._run_recording, args=(active,),
                                  name="r1_vision_capture", daemon=True)
        active["thread"] = thread
        thread.start()
        return {
            "ok": True,
            "state": "recording",
            "action_id": active["action_id"],
            "file_path": str(active["path"]),
            "duration_s": requested,
            "camera": camera,
            "started_at": active["started_at"],
        }

    def _cancel_recording(self) -> dict:
        with self._lock:
            active = self._active
        if active is None:
            return {"state": "idle"}
        active["cancel"].set()
        thread = active["thread"]
        if thread is not None:
            thread.join(timeout=5.0)
        # The worker owns cleanup and the terminal completion; this only reports
        # that the cancel was delivered, so the canvas can leave "recording".
        return {"ok": True, "state": "stopping", "action_id": active["action_id"]}

    def _run_recording(self, active: dict) -> None:
        cancel = active["cancel"]
        try:
            result = self._encoder(active, cancel)
        except Exception as exc:
            result = {"ok": False,
                      "code": "RECORD_CANCELLED" if cancel.is_set() else "RECORD_FAILED",
                      "message": str(exc)}

        with self._lock:
            # Cancellation wins over a successful encode that finished at the
            # same moment: the operator asked for the file to be gone.
            if cancel.is_set() and result.get("ok"):
                self._remove_partial(Path(result["file_path"]))
                result = {"ok": False, "code": "RECORD_CANCELLED",
                          "message": "Video recording was cancelled"}
            if not result.get("ok"):
                self._remove_partial(active["path"])
            status = "completed" if result.get("ok") else (
                "cancelled" if result.get("code") == "RECORD_CANCELLED" else "error")
            self._last = {
                "action_id": active["action_id"],
                "status": status,
                "camera": active["camera"],
                "duration_s": active["duration_s"],
                "started_at": active["started_at"],
                "finished_at": _timestamp(),
                "result": result,
            }
            self._active = None

        try:
            self._notify(active["action_id"], status, result)
        except Exception as exc:
            log.warning("[vision_capture] completion delivery failed: %s", exc)

    def _encode_video(self, active: dict, cancel) -> dict:
        camera = active["camera"]
        topic = self._topics[camera]
        path = active["path"]
        duration = active["duration_s"]
        process = None
        completed = False
        try:
            mark = self._store.mark(topic)
            frame, received = self._store.wait(topic, mark, FIRST_FRAME_TIMEOUT_S, cancel=cancel)
            with tempfile.TemporaryFile() as errors:
                process = subprocess.Popen([
                    "ffmpeg", "-nostdin", "-y", "-loglevel", "error",
                    "-f", "mjpeg", "-framerate", str(self._fps),
                    "-probesize", "32", "-analyzeduration", "0", "-i", "pipe:0", "-an",
                    "-c:v", "libx264", "-preset", "ultrafast", "-threads", "2",
                    "-vf", ("pad=ceil(iw/2)*2:ceil(ih/2)*2,"
                            f"setpts=PTS-STARTPTS,fps={self._fps}"),
                    "-frames:v", str(self._fps * duration),
                    "-pix_fmt", "yuv420p", "-movflags", "+faststart", str(path),
                ], stdin=subprocess.PIPE, stdout=subprocess.DEVNULL, stderr=errors, bufsize=0)
                os.set_blocking(process.stdin.fileno(), False)

                deadline = time.monotonic() + duration
                frames = 0
                while True:
                    self._write_frame(process, frame, cancel)
                    frames += 1
                    remaining = deadline - time.monotonic()
                    if remaining <= 0:
                        break
                    cancel.wait(min(1.0 / self._fps, remaining))
                    try:
                        mark = self._store.mark(topic)
                        frame, received = self._store.wait(
                            topic, mark, min(MAX_FRAME_AGE_S, max(remaining, 0.05)),
                            cancel=cancel)
                    except TimeoutError:
                        # A healthy low-rate source may have no frame exactly at
                        # the deadline; a stalled one must still fail.
                        if (not cancel.is_set() and time.monotonic() >= deadline
                                and time.monotonic() - received < MAX_FRAME_AGE_S):
                            break
                        raise

                if frames < min(2, self._fps * duration):
                    raise RuntimeError("Not enough fresh frames to record video")
                # The mjpeg demuxer only emits a JPEG once the next one starts,
                # so a short recording closed stdin with nothing decoded and
                # produced a 261-byte MP4 with no video stream. One repeated
                # frame terminates the stream; -frames:v caps the output.
                self._write_frame(process, frame, cancel)
                process.stdin.close()
                finalize_deadline = time.monotonic() + 10
                while process.poll() is None:
                    if cancel.wait(0.1):
                        raise RuntimeError("Video recording was cancelled")
                    if time.monotonic() >= finalize_deadline:
                        raise RuntimeError("ffmpeg finalization timed out")
                if process.returncode != 0 or not path.exists() or path.stat().st_size == 0:
                    errors.seek(0)
                    raise RuntimeError(errors.read(4096).decode("utf-8", "replace")
                                       or "ffmpeg failed to create MP4")

            media = self._probe(path, duration, self._fps)
            if cancel.is_set():
                raise RuntimeError("Video recording was cancelled")
            completed = True
            return {"ok": True, "media_type": "video", "file_path": str(path),
                    "recorded_duration_s": media["duration_s"], "frames": frames,
                    "captured_at": _timestamp()}
        finally:
            if process is not None:
                if process.poll() is None:
                    process.kill()
                    try:
                        process.wait(timeout=2)
                    except subprocess.TimeoutExpired:
                        pass
                if process.stdin is not None and not process.stdin.closed:
                    process.stdin.close()
            if not completed:
                self._remove_partial(path)

    @staticmethod
    def _write_frame(process, data: bytes, cancel) -> None:
        # Never block on a wedged encoder: stop() must be able to cancel even
        # while ffmpeg's stdin is full.
        pending = memoryview(data)
        deadline = time.monotonic() + 3
        while pending:
            if cancel.is_set():
                raise RuntimeError("Video recording was cancelled")
            if process.poll() is not None:
                raise RuntimeError("ffmpeg exited while recording")
            if time.monotonic() >= deadline:
                raise RuntimeError("ffmpeg input timed out")
            if select.select([], [process.stdin], [], 0.1)[1]:
                try:
                    pending = pending[os.write(process.stdin.fileno(), pending):]
                except BlockingIOError:
                    pass

    @staticmethod
    def _probe_video(path: Path, requested: int, fps: int) -> dict:
        """Only acknowledge completion once the muxed file has the right shape."""
        metadata = json.loads(subprocess.check_output([
            "ffprobe", "-v", "error", "-select_streams", "v:0",
            "-show_entries", "format=duration:stream=duration,nb_frames",
            "-of", "json", str(path),
        ], stderr=subprocess.PIPE, timeout=5))
        stream = metadata["streams"][0]
        duration = float(metadata["format"]["duration"])
        frames = int(stream.get("nb_frames") or 0)
        # `-frames:v` is exact, so both checks are exact: a short file here
        # means the encoder was cut off, not that the source was slow.
        if abs(duration - requested) > 0.05:
            raise RuntimeError(
                f"Invalid completed video: {duration:.2f}s for a {requested}s request")
        if frames != requested * fps:
            raise RuntimeError(
                f"Invalid completed video: {frames} frames, expected {requested * fps}")
        return {"duration_s": duration, "frames": frames}

    # ── status ───────────────────────────────────────────────────────────────

    def _list_cameras(self) -> dict:
        cameras = []
        for camera, topic in self._topics.items():
            snapshot = self._store.snapshot(topic)
            cameras.append({
                "camera": camera,
                "topic": topic,
                "available": bool(snapshot["count"])
                               and snapshot["age_s"] is not None
                               and snapshot["age_s"] <= MAX_FRAME_AGE_S,
                "last_frame_age_s": (round(snapshot["age_s"], 3)
                                     if snapshot["age_s"] is not None else None),
                "frames": snapshot["count"],
            })
        return {"state": "ready", "cameras": cameras}

    def _info(self) -> dict:
        with self._lock:
            active = self._active
            last = self._last
            last_photo = self._last_photo
            public = ({key: active[key] for key in
                       ("action_id", "camera", "duration_s", "started_at")}
                      | {"file_path": str(active["path"])}) if active else None
        return {
            "state": "recording" if active else "ready",
            "output_dir": str(self._output_dir),
            "default_camera": self._camera,
            "fps": self._fps,
            "max_duration_s": self._max_duration_s,
            "encoder_available": all(
                shutil.which(name) is not None for name in ("ffmpeg", "ffprobe")),
            "retention_days": self._retention_days,
            "cameras": self._list_cameras()["cameras"],
            "active_recording": public,
            "last_recording": last,
            "last_capture": last_photo,
        }

    # ── files ────────────────────────────────────────────────────────────────

    def _new_path(self, folder: str, prefix: str, suffix: str) -> Path:
        directory = self._output_dir / folder
        directory.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        return directory / f"{prefix}_{stamp}_{uuid4().hex[:8]}{suffix}"

    def _prune(self) -> None:
        """Drop evidence older than `retention_days`, off the start path.

        A capture card is called every time a skill wants to look at something,
        so without this the host's disk fills with JPEGs nobody will open. The
        directory itself is kept: it is a mount point, and removing it under a
        running container is how the next capture fails with ENOENT.
        """
        if self._retention_days <= 0:
            return
        cutoff = time.time() - self._retention_days * 86400
        removed = 0
        for folder in ("photos", "videos"):
            directory = self._output_dir / folder
            if not directory.is_dir():
                continue
            for path in directory.iterdir():
                try:
                    if path.is_file() and path.stat().st_mtime < cutoff:
                        path.unlink()
                        removed += 1
                except OSError as exc:
                    log.warning("[vision_capture] could not prune %s: %s", path, exc)
        if removed:
            print(f"[vision_capture] pruned {removed} capture(s) older than "
                  f"{self._retention_days}d", flush=True)

    @staticmethod
    def _remove_partial(path: Path) -> None:
        try:
            Path(path).unlink(missing_ok=True)
        except OSError as exc:
            log.error("[vision_capture] could not remove incomplete file %s: %s", path, exc)

    def _notify_complete(self, action_id: str, status: str, result: dict) -> None:
        payload = json.dumps({"action_id": action_id, "status": status,
                              "result": result, "tool": self.PREFIX,
                              "ts": time.time()}).encode()
        url = os.environ.get("AGENT_CORE_URL", "https://localhost:15678").rstrip("/")
        context = ssl.create_default_context()
        if url.startswith(("https://localhost:", "https://127.0.0.1:")):
            context.check_hostname = False
            context.verify_mode = ssl.CERT_NONE
        request = urllib.request.Request(
            url + "/api/acp/complete", data=payload,
            headers={"Content-Type": "application/json"})
        with urllib.request.urlopen(request, timeout=3, context=context):
            pass
