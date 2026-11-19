"""Save media from the shared Panther RGB stream."""

from datetime import datetime
from pathlib import Path
import re
import shutil
import subprocess
import tempfile
import threading
import time
from uuid import uuid4


_FILENAME = re.compile(r"(?:IMG_\d{8}_\d{6}_[0-9a-f]{32}\.jpg|VID_\d{8}_\d{6}_[0-9a-f]{32}\.mp4)")
MAX_DURATION_S = 30
VIDEO_FPS = 10
ENCODE_TIMEOUT_S = 20


def _error(code, message):
    return {"ok": False, "state": "error", "code": code, "message": message}


class VisionCapture:
    def __init__(self, camera, config, notify):
        self._camera = camera
        self._notify = notify
        self._directory = Path(config.get("output_dir", "/opt/phanthy-motus/data/vision_capture/panther"))
        self._lock = threading.Lock()
        self._closed = False
        self._photo_active = False
        self._recording = None

    def _path(self, name):
        return self._directory / ("photos" if name.endswith(".jpg") else "videos") / name

    def _list(self):
        files = []
        for directory in (self._directory / "photos", self._directory / "videos"):
            if not directory.is_dir() or directory.is_symlink():
                continue
            for path in directory.iterdir():
                if not _FILENAME.fullmatch(path.name) or path != self._path(path.name):
                    continue
                try:
                    if not path.is_file() or path.is_symlink():
                        continue
                    stat = path.stat()
                except FileNotFoundError:
                    continue
                files.append((stat.st_mtime, {"filename": path.name, "file_path": str(path),
                                             "size_bytes": stat.st_size}))
        files.sort(key=lambda item: item[0], reverse=True)
        return {"ok": True, "state": "listed", "files": [item[1] for item in files[:100]],
                "total": len(files)}

    def _delete(self, name):
        if not isinstance(name, str) or not _FILENAME.fullmatch(name):
            return _error("INVALID_ARGUMENT", "use a photo or video filename returned by list")
        path = self._path(name)
        if path.parent.is_symlink() or path.is_symlink() or not path.is_file():
            return _error("NOT_FOUND", f"media file not found: {name}")
        try:
            path.unlink()
        except FileNotFoundError:
            return _error("NOT_FOUND", f"media file not found: {name}")
        return {"ok": True, "state": "deleted", "filename": name}

    def _photo(self):
        with self._lock:
            if self._closed:
                return _error("SHUTTING_DOWN", "media capture is shutting down")
            if self._photo_active or self._recording:
                return _error("RESOURCE_BUSY", "another media capture is in progress")
            self._photo_active = True
        try:
            payload = self._camera.capture_jpeg(timeout_s=3.0)
            when = datetime.now().astimezone()
            path = self._path(f"IMG_{when:%Y%m%d_%H%M%S}_{uuid4().hex}.jpg")
            path.parent.mkdir(parents=True, exist_ok=True)
            with tempfile.TemporaryDirectory(prefix=".capture-", dir=path.parent) as work:
                partial = Path(work) / "frame.jpg"
                partial.write_bytes(payload["data"])
                partial.replace(path)
            return {"ok": True, "state": "completed", "media_type": "photo",
                    "file_path": str(path), "filename": path.name,
                    "captured_at": when.isoformat(timespec="seconds"),
                    "frame_id": payload["frame_id"], "stamp": payload["stamp"]}
        except Exception as exc:
            return _error("CAPTURE_FAILED", self._camera._bounded_error(exc))
        finally:
            with self._lock:
                self._photo_active = False

    def _start_video(self, duration):
        if isinstance(duration, bool) or not isinstance(duration, int) or not 1 <= duration <= MAX_DURATION_S:
            return _error("INVALID_ARGUMENT", f"duration_s must be an integer from 1 to {MAX_DURATION_S}")
        encoder = shutil.which("ffmpeg")
        if encoder is None:
            return _error("ENCODER_UNAVAILABLE", "ffmpeg is not installed")
        if not self._camera.rgb_available():
            return _error("CAPTURE_FAILED", "no RGB camera publisher; start camera_color first")
        with self._lock:
            if self._closed:
                return _error("SHUTTING_DOWN", "media capture is shutting down")
            if self._photo_active or self._recording:
                return _error("RESOURCE_BUSY", "another media capture is in progress")
            action_id = f"panther_video_{uuid4().hex}"
            cancel = threading.Event()
            worker = threading.Thread(target=self._run_video, args=(encoder, duration, action_id, cancel),
                                      name="panther_video", daemon=True)
            self._recording = (action_id, cancel, worker)
            try:
                worker.start()
            except Exception:
                self._recording = None
                raise
        return {"ok": True, "state": "recording", "action_id": action_id, "duration_s": duration}

    @staticmethod
    def _encode(encoder, source, target, cancel):
        command = [encoder, "-nostdin", "-loglevel", "error", "-f", "mjpeg", "-framerate", str(VIDEO_FPS),
                   "-i", str(source), "-an", "-vf", "pad=ceil(iw/2)*2:ceil(ih/2)*2",
                   "-c:v", "libx264", "-preset", "ultrafast", "-threads", "2", "-pix_fmt", "yuv420p",
                   "-movflags", "+faststart", str(target)]
        with subprocess.Popen(command, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE) as process:
            deadline = time.monotonic() + ENCODE_TIMEOUT_S
            try:
                while not cancel.is_set():
                    try:
                        _, err = process.communicate(timeout=0.2)
                    except subprocess.TimeoutExpired:
                        if time.monotonic() >= deadline:
                            raise TimeoutError("video encoding timed out")
                        continue
                    if process.returncode:
                        raise RuntimeError(err.decode(errors="replace")[-240:])
                    return
            finally:
                if process.poll() is None:
                    process.kill()
                    process.communicate()

    def _run_video(self, encoder, duration, action_id, cancel):
        status, result = "cancelled", {"ok": False, "state": "cancelled"}
        try:
            path = self._path(f"VID_{datetime.now():%Y%m%d_%H%M%S}_{uuid4().hex}.mp4")
            path.parent.mkdir(parents=True, exist_ok=True)
            with tempfile.TemporaryDirectory(prefix=".record-", dir=path.parent) as work:
                source, target = Path(work) / "frames.mjpeg", Path(work) / "video.mp4"
                with source.open("wb") as output:
                    first = self._camera.capture_jpeg(timeout_s=3.0)
                    output.write(first["data"])
                    start = time.monotonic()
                    for index in range(1, duration * VIDEO_FPS):
                        if cancel.wait(max(0.0, start + index / VIDEO_FPS - time.monotonic())):
                            break
                        if time.monotonic() > start + duration + 2.0:
                            raise TimeoutError("RGB frames arrived too slowly to record")
                        output.write(self._camera.capture_jpeg(timeout_s=0.75)["data"])
                if not cancel.is_set():
                    self._encode(encoder, source, target, cancel)
                with self._lock:
                    if not cancel.is_set():
                        if not target.is_file() or not target.stat().st_size:
                            raise RuntimeError("encoder did not produce a video")
                        target.replace(path)
                        status = "completed"
                        result = {"ok": True, "state": status, "media_type": "video", "duration_s": duration,
                                  "file_path": str(path), "filename": path.name}
                        self._recording = None
        except Exception as exc:
            if status != "completed" and not cancel.is_set():
                status, result = "error", _error("RECORD_FAILED", self._camera._bounded_error(exc))
        finally:
            with self._lock:
                if self._recording and self._recording[0] == action_id:
                    self._recording = None
            self._notify(action_id, status, result, "vision_capture")

    def stop(self):
        with self._lock:
            if self._recording:
                self._recording[1].set()
                return {"state": "stopping", "action_id": self._recording[0]}
        return {"state": "idle"}

    def close(self):
        with self._lock:
            self._closed = True
            recording = self._recording
            if recording:
                recording[1].set()
        if recording:
            recording[2].join(timeout=4.0)

    def dispatch(self, args):
        action = args.get("action", "info")
        try:
            if action in ("start", "info"):
                ready = self._camera.rgb_available()
                with self._lock:
                    recording = self._recording[0] if self._recording else None
                return {"state": "recording" if recording else "ready" if ready else "waiting_for_camera",
                        "action_id": recording, "source_topic": self._camera.COLOR_TOPIC,
                        "photos_dir": str(self._directory / "photos"), "videos_dir": str(self._directory / "videos")}
            if action == "capture_photo":
                return self._photo()
            if action == "record_video":
                return self._start_video(args.get("duration_s", 5))
            if action == "stop":
                return self.stop()
            if action == "list":
                return self._list()
            if action == "delete":
                return self._delete(args.get("name"))
            return _error("INVALID_ARGUMENT", f"unsupported vision_capture action: {action}")
        except OSError as exc:
            return _error("MEDIA_IO_FAILED", self._camera._bounded_error(exc))
