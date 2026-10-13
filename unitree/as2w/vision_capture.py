"""Persistent photos and asynchronous silent video from the AS2W camera topic."""

from datetime import datetime
import json
import logging
import os
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
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage


_FIRST_FRAME_TIMEOUT_S = 5.0
_MAX_FRAME_AGE_S = 3.0
log = logging.getLogger(__name__)


def _timestamp():
    return datetime.now().astimezone().isoformat(timespec="milliseconds")


class VisionCapturePlugin:
    """Capture media without opening a second Unitree VideoClient."""

    PREFIX = "vision_capture"

    def __init__(self, plugin_config, namespace, executor):
        self._topic = f"/{namespace}/camera/front"
        self._output_dir = Path(plugin_config.get(
            "output_dir", "/opt/phanthy-motus/data/vision_capture")).expanduser()
        self._fps = max(1, min(15, int(plugin_config.get("fps", 8))))
        self._max_duration_s = max(
            1, min(30, int(plugin_config.get("max_duration_s", 30))))
        self._condition = threading.Condition()
        self._latest = None
        self._sequence = 0
        self._recording_lock = threading.Lock()
        self._active_recording = None
        self._last_recording = None
        self._node = Node("as2w_vision_capture")
        self._subscription = self._node.create_subscription(
            CompressedImage, self._topic, self._on_frame,
            qos_profile_sensor_data)
        executor.add_node(self._node)

    def get_tool(self):
        return {
            "name": self.PREFIX,
            "type": "actuator",
            "multiInstance": False,
            "description": (
                "Save an AS2W front-camera JPEG or asynchronously record a "
                "silent H.264 MP4 to persistent storage."),
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {"type": "string", "enum": [
                        "start", "capture_photo", "record_video",
                        "info", "stop"]},
                    "duration_s": {
                        "type": "integer", "minimum": 1,
                        "maximum": self._max_duration_s,
                        "default": min(5, self._max_duration_s),
                    },
                },
                "required": ["action"],
                "additionalProperties": False,
                "x-action-params": {
                    "start": {"params": []},
                    "capture_photo": {"params": []},
                    "record_video": {"params": ["duration_s"]},
                    "info": {"params": []},
                    "stop": {"params": []},
                },
                "x-completion": {
                    "actions": ["record_video"],
                    # First-frame acquisition, encoder finalization and probe
                    # can add 20 seconds beyond the requested recording.
                    "timeout": self._max_duration_s + 25,
                },
            },
        }

    def _source(self):
        return {"camera": "front", "name": "AS2W built-in front camera",
                "topic": self._topic}

    def _on_frame(self, msg):
        data = bytes(getattr(msg, "data", b""))
        image_format = str(getattr(msg, "format", "")).lower()
        if "jpeg" not in image_format and "jpg" not in image_format:
            return
        if not data.startswith(b"\xff\xd8") or not data.endswith(b"\xff\xd9"):
            return
        stamp = getattr(getattr(msg, "header", None), "stamp", None)
        timestamp = (getattr(stamp, "sec", 0)
                     + getattr(stamp, "nanosec", 0) / 1e9) if stamp else 0.0
        now = self._node.get_clock().now().nanoseconds / 1e9
        age = now - timestamp if timestamp else 0.0
        if age > _MAX_FRAME_AGE_S or age < -_MAX_FRAME_AGE_S:
            return
        with self._condition:
            self._sequence += 1
            self._latest = (
                data, time.monotonic() - max(0.0, age), self._sequence)
            self._condition.notify_all()

    def _frame(self, after_sequence, timeout_s=_FIRST_FRAME_TIMEOUT_S,
               cancel=None):
        deadline = time.monotonic() + timeout_s
        with self._condition:
            while True:
                if cancel is not None and cancel.is_set():
                    raise RuntimeError("Video recording was cancelled")
                frame = self._latest
                if (frame is not None
                        and time.monotonic() - frame[1] <= _MAX_FRAME_AGE_S
                        and frame[2] > after_sequence):
                    return frame
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise RuntimeError(
                        f"No fresh JPEG frame on {self._topic}; start the "
                        "camera card and check ROS connectivity")
                self._condition.wait(min(0.1, remaining))

    def _info(self):
        with self._recording_lock:
            active = self._active_recording
            public = ({key: active[key] for key in (
                "action_id", "state", "duration_s", "started_at", "source")}
                if active else None)
            last = self._last_recording
        with self._condition:
            frame = self._latest
            age = time.monotonic() - frame[1] if frame else None
        ready = age is not None and age <= _MAX_FRAME_AGE_S
        return {
            "ok": ready,
            "state": "ready" if ready else "waiting_for_camera",
            "source": self._source(),
            "output_dir": str(self._output_dir),
            "photos_dir": str(self._output_dir / "photos"),
            "videos_dir": str(self._output_dir / "videos"),
            "fps": self._fps,
            "max_duration_s": self._max_duration_s,
            "latest_frame_age_s": round(age, 3) if age is not None else None,
            "encoder_available": all(
                shutil.which(name) is not None
                for name in ("ffmpeg", "ffprobe")),
            "active_recording": public,
            "last_recording": last,
        }

    def start(self):
        # Bundle startup happens before the executor spins, so freshness lives
        # on info rather than making startup block for a frame.
        return {"state": "ready"}

    def _new_path(self, directory, prefix, suffix):
        target = self._output_dir / directory
        target.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        return target / f"{prefix}_{stamp}_{uuid4().hex[:8]}{suffix}"

    def _capture_photo(self):
        path = None
        try:
            # Snapshot the sequence at request admission: cached frames can
            # never satisfy a photo request.
            with self._condition:
                sequence = self._sequence
            data, received, _ = self._frame(sequence)
            path = self._new_path("photos", "IMG", ".jpg")
            path.write_bytes(data)
            return {
                "ok": True,
                "media_type": "photo",
                "file_path": str(path),
                "captured_at": _timestamp(),
                "frame_age_s": round(time.monotonic() - received, 3),
            }
        except Exception as exc:
            if path is not None:
                self._remove_partial(path)
            return {"ok": False, "code": "CAPTURE_FAILED",
                    "message": str(exc)}

    @staticmethod
    def _remove_partial(path):
        try:
            path.unlink(missing_ok=True)
        except OSError as exc:
            log.error("[vision_capture] Could not remove %s: %s", path, exc)
            return str(exc)
        return None

    @staticmethod
    def _write_frame(process, data, cancel):
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
                    pending = pending[
                        os.write(process.stdin.fileno(), pending):]
                except BlockingIOError:
                    pass

    def _record_video(self, active):
        cancel = active["cancel"]
        process = None
        path = active["path"]
        completed = False
        try:
            with self._condition:
                sequence = self._sequence
            frame = self._frame(sequence, cancel=cancel)
            with tempfile.TemporaryFile() as errors:
                process = subprocess.Popen([
                    "ffmpeg", "-nostdin", "-y", "-loglevel", "error",
                    "-use_wallclock_as_timestamps", "1", "-f", "mjpeg",
                    "-probesize", "32", "-analyzeduration", "0",
                    "-framerate", str(self._fps), "-i", "pipe:0", "-an",
                    "-c:v", "libx264", "-preset", "ultrafast",
                    "-threads", "2",
                    "-vf", ("scale=out_range=tv,"
                            "pad=ceil(iw/2)*2:ceil(ih/2)*2,"
                            f"setpts=PTS-STARTPTS,fps={self._fps},"
                            "tpad=stop_mode=clone:stop=-1"),
                    "-frames:v", str(self._fps * active["duration_s"]),
                    "-pix_fmt", "yuv420p", "-color_range", "tv",
                    "-vsync", "vfr", "-movflags", "+faststart", str(path),
                ], stdin=subprocess.PIPE, stdout=subprocess.DEVNULL,
                    stderr=errors, bufsize=0)
                os.set_blocking(process.stdin.fileno(), False)
                with self._recording_lock:
                    active["process"] = process
                deadline = time.monotonic() + active["duration_s"]
                frames = 0
                while True:
                    tick = time.monotonic()
                    self._write_frame(process, frame[0], cancel)
                    frames += 1
                    cancel.wait(min(
                        max(0, 1 / self._fps - (time.monotonic() - tick)),
                        max(0, deadline - time.monotonic())))
                    remaining = deadline - time.monotonic()
                    if remaining <= 0:
                        break
                    try:
                        frame = self._frame(
                            frame[2], min(remaining, _MAX_FRAME_AGE_S), cancel)
                    except RuntimeError:
                        if (not cancel.is_set() and time.monotonic() >= deadline
                                and time.monotonic() - frame[1]
                                < _MAX_FRAME_AGE_S):
                            break
                        raise
                if frames < min(2, self._fps * active["duration_s"]):
                    raise RuntimeError("Not enough fresh frames to record video")
                process.stdin.close()
                encode_deadline = time.monotonic() + 10
                while process.poll() is None:
                    if cancel.wait(0.1):
                        raise RuntimeError("Video recording was cancelled")
                    if time.monotonic() >= encode_deadline:
                        raise RuntimeError("ffmpeg finalization timed out")
                if (process.returncode != 0 or not path.exists()
                        or path.stat().st_size == 0):
                    errors.seek(0)
                    raise RuntimeError(
                        errors.read(4096).decode("utf-8", "replace")
                        or "ffmpeg failed to create MP4")
            media = self._probe_video(
                path, active["duration_s"], self._fps)
            if cancel.is_set():
                raise RuntimeError("Video recording was cancelled")
            completed = True
            return {
                "ok": True,
                "media_type": "video",
                "file_path": str(path),
                "recorded_duration_s": media["duration_s"],
                "frames": frames,
                "captured_at": _timestamp(),
            }
        except Exception as exc:
            return {
                "ok": False,
                "code": ("RECORD_CANCELLED" if cancel.is_set()
                         else "RECORD_FAILED"),
                "message": str(exc),
            }
        finally:
            if process is not None:
                if process.poll() is None:
                    process.kill()
                    process.wait(timeout=2)
                if process.stdin is not None and not process.stdin.closed:
                    process.stdin.close()
            with self._recording_lock:
                active["process"] = None
            if not completed:
                self._remove_partial(path)

    @staticmethod
    def _probe_video(path, requested, fps):
        metadata = json.loads(subprocess.check_output([
            "ffprobe", "-v", "error", "-show_entries",
            "format=duration:stream=codec_type,codec_name,duration,nb_frames",
            "-of", "json", str(path),
        ], stderr=subprocess.PIPE, timeout=5))
        video = [stream for stream in metadata.get("streams", [])
                 if stream.get("codec_type") == "video"]
        audio = [stream for stream in metadata.get("streams", [])
                 if stream.get("codec_type") == "audio"]
        if len(video) != 1 or audio or video[0].get("codec_name") != "h264":
            raise RuntimeError("Invalid completed video: expected one H.264 video track and no audio")
        duration = float(metadata["format"]["duration"])
        stream_duration = float(video[0]["duration"])
        frames = int(video[0]["nb_frames"])
        if (abs(duration - requested) > 0.001
                or abs(stream_duration - requested) > 0.001
                or frames != requested * fps):
            raise RuntimeError(
                "Invalid completed video: "
                f"duration={duration}s, stream={stream_duration}s, "
                f"frames={frames}")
        return {"duration_s": duration, "frames": frames}

    def _notify_complete(self, action_id, status, result):
        payload = json.dumps({
            "action_id": action_id, "status": status, "result": result,
            "tool": self.PREFIX, "ts": time.time(),
        }).encode()
        url = os.environ.get(
            "AGENT_CORE_URL", "https://localhost:15678").rstrip("/")
        context = ssl.create_default_context()
        if url.startswith(("https://localhost:", "https://127.0.0.1:")):
            context.check_hostname = False
            context.verify_mode = ssl.CERT_NONE
        try:
            request = urllib.request.Request(
                url + "/api/acp/complete", data=payload,
                headers={"Content-Type": "application/json"})
            with urllib.request.urlopen(
                    request, timeout=3, context=context):
                pass
        except Exception as exc:
            log.warning("[vision_capture] ACP completion failed: %s", exc)

    def _record_video_async(self, active):
        try:
            result = self._record_video(active)
        except Exception as exc:
            result = {"ok": False, "code": "RECORD_FAILED",
                      "message": str(exc)}
        with self._recording_lock:
            if active["cancel"].is_set():
                cleanup_error = None
                if result.get("ok"):
                    cleanup_error = self._remove_partial(
                        Path(result["file_path"]))
                result = {"ok": False, "code": "RECORD_CANCELLED",
                          "message": "Video recording was cancelled"}
                if cleanup_error:
                    result["cleanup_error"] = cleanup_error
            status = ("completed" if result.get("ok") else
                      "cancelled" if result.get("code") == "RECORD_CANCELLED"
                      else "error")
            self._last_recording = {
                "action_id": active["action_id"],
                "status": status,
                "result": result,
            }
            active["state"] = status
            active["finished"] = True
        try:
            self._notify_complete(active["action_id"], status, result)
        finally:
            with self._recording_lock:
                self._active_recording = None

    def _start_video_recording(self, args):
        requested = args.get("duration_s", min(5, self._max_duration_s))
        if type(requested) is not int or not 1 <= requested <= self._max_duration_s:
            return {
                "ok": False, "code": "INVALID_DURATION",
                "message": ("duration_s must be an integer between 1 and "
                            f"{self._max_duration_s}"),
            }
        missing = [name for name in ("ffmpeg", "ffprobe")
                   if shutil.which(name) is None]
        if missing:
            return {
                "ok": False, "code": "ENCODER_UNAVAILABLE",
                "message": f"Required executable(s) not found: {', '.join(missing)}",
            }
        with self._recording_lock:
            if self._active_recording is not None:
                return {"ok": False, "code": "RECORD_IN_PROGRESS",
                        "message": "A video recording is already in progress"}
            action_id = f"vision_capture_record_video_{uuid4().hex}"
            queued_at = _timestamp()
            path = self._new_path("videos", "video", ".mp4")
            active = {
                "action_id": action_id, "state": "recording",
                "duration_s": requested, "started_at": queued_at,
                "cancel": threading.Event(), "process": None,
                "source": self._source(), "path": path,
            }
            thread = threading.Thread(
                target=self._record_video_async, args=(active,), daemon=True,
                name="as2w_vision_capture_record_video")
            active["thread"] = thread
            self._active_recording = active
            try:
                thread.start()
            except Exception:
                self._active_recording = None
                self._remove_partial(path)
                raise
        return {
            "ok": True, "state": "recording", "action_id": action_id,
            "media_type": "video", "requested_duration_s": requested,
            "source": self._source(), "queued_at": queued_at,
            "file_path": str(path),
        }

    def stop(self):
        with self._recording_lock:
            active = self._active_recording
            if active is None:
                return {"ok": True, "state": "idle"}
            if not active.get("finished"):
                active["cancel"].set()
                active["state"] = "stopping"
            process = active["process"]
        with self._condition:
            self._condition.notify_all()
        if process is not None and process.poll() is None:
            try:
                process.terminate()
            except ProcessLookupError:
                pass
        active["thread"].join(timeout=6)
        with self._recording_lock:
            stopping = self._active_recording is active
        return {"ok": True, "state": "stopping" if stopping else "idle",
                "action_id": active["action_id"]}

    def dispatch(self, action, args):
        if action == "start":
            return self.start()
        if action == "capture_photo":
            return self._capture_photo()
        if action == "record_video":
            return self._start_video_recording(args)
        if action == "info":
            return self._info()
        if action == "stop":
            return self.stop()
        return None
