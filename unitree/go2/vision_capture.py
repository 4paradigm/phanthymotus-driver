"""Persistent JPEG photos and asynchronous MP4 recording from Go2 ROS images.

Keeps the Q5 vision_capture action/ACP contract while sharing the existing
camera publisher instead of opening the physical camera a second time.
"""

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
    PREFIX = "vision_capture"

    def __init__(self, plugin_config, namespace, executor, external_camera=None):
        self._front_topic = f"/{namespace}/camera/front"
        self._external_camera = external_camera
        self._camera = plugin_config.get("camera", "front")
        self._external_instance_id = plugin_config.get("external_instance_id", "")
        if self._camera not in ("front", "external"):
            raise ValueError("camera must be front or external")
        self._output_dir = Path(plugin_config.get(
            "output_dir", "/opt/phanthy-motus/data/vision_capture")).expanduser()
        self._fps = max(1, min(15, int(plugin_config.get("fps", 15))))
        self._max_duration_s = max(1, min(30, int(plugin_config.get("max_duration_s", 30))))
        self._condition = threading.Condition()
        self._streams = {}
        self._recording_lock = threading.Lock()
        self._active_recording = None
        self._last_recording = None
        self._node = Node("go2_vision_capture")
        executor.add_node(self._node)

    def get_tool(self):
        camera_property = {"type": "string", "enum": ["front", "external"],
                           "description": "摄像头：front 内置前置，external 外接（only rgb）；省略时使用卡片配置。"}
        return {
            "name": self.PREFIX, "type": "actuator", "multiInstance": False,
            "description": "Save a Go2 RGB JPEG photo or record a silent H.264 MP4 video to persistent storage.",
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {"type": "string", "enum": [
                        "start", "capture_photo", "record_video", "list_cameras", "info", "stop"]},
                    "camera": camera_property,
                    "duration_s": {"type": "integer", "minimum": 1,
                                   "maximum": self._max_duration_s,
                                   "default": 5},
                },
                "required": ["action"], "additionalProperties": False,
                "x-action-params": {
                    "start": {"params": [], "description": "订阅配置的相机 RGB 图像。"},
                    "capture_photo": {"params": ["camera"], "description": "保存所选相机的 RGB 照片为 JPG。"},
                    "record_video": {"params": ["duration_s", "camera"], "description": "录制所选相机的 RGB 视频，默认 5 秒。"},
                    "list_cameras": {"params": [], "description": "列出内置相机和已启动的外接相机实例。"},
                    "info": {"params": [], "description": "查看图像来源、保存目录和录像结果。"},
                    "stop": {"params": [], "description": "取消当前录像并删除未完成文件。"},
                },
                "x-completion": {"actions": ["record_video"],
                                 "timeout": self._max_duration_s + 15},
            },
            "configSchema": {"type": "object", "properties": {
                "camera": {**camera_property, "default": "front"},
            }},
        }

    def _sources(self):
        sources = [{"camera": "front", "name": "Go2 内置前置相机", "topic": self._front_topic}]
        if self._external_camera is not None:
            info = self._external_camera.dispatch("info", {})
            for instance_id in info.get("active_instances", []):
                status = self._external_camera.dispatch("info", {"instance_id": instance_id})
                if status.get("state") != "running" or status.get("channel", "rgb") != "rgb":
                    continue
                for output in status.get("topic_out", []):
                    if output.get("format") == "image/jpeg" and output.get("topic"):
                        sources.append({"camera": "external", "external_instance_id": instance_id,
                                        "name": status.get("device_name") or instance_id,
                                        "device_path": status.get("device_path"), "topic": output["topic"]})
        return sources

    def _resolve_source(self, args):
        camera = args.get("camera", self._camera)
        if camera == "front":
            source = {"camera": "front", "name": "Go2 内置前置相机", "topic": self._front_topic}
        elif camera == "external":
            instance_id = args.get("external_instance_id", self._external_instance_id)
            if instance_id and self._external_camera is not None:
                status = self._external_camera.dispatch("info", {"instance_id": instance_id})
                channel = status.get("channel", "rgb")
                if channel != "rgb":
                    raise ValueError(f"External camera instance '{instance_id}' uses channel '{channel}'; "
                                     "vision_capture currently supports only RGB. Select a channel=rgb instance.")
            candidates = [source for source in self._sources() if source["camera"] == "external"
                          and (not instance_id or source["external_instance_id"] == instance_id)]
            if not candidates:
                raise ValueError("No running RGB external camera matches the selection; "
                                 "start an ext_camera instance with channel=rgb and use its instance_id")
            if len(candidates) != 1:
                raise ValueError("Multiple RGB external cameras are running; keep only the intended RGB instance running before capture")
            source = candidates[0]
        else:
            raise ValueError("camera must be front or external")
        topic = source["topic"]
        with self._condition:
            if topic not in self._streams:
                self._streams[topic] = {"latest": None, "sequence": 0}
                try:
                    self._streams[topic]["subscription"] = self._node.create_subscription(
                        CompressedImage, topic, lambda msg: self._on_frame(topic, msg), qos_profile_sensor_data)
                except Exception:
                    del self._streams[topic]
                    raise
        return source

    def _on_frame(self, topic, msg):
        data = bytes(msg.data)
        if "jpeg" not in msg.format.lower() and "jpg" not in msg.format.lower():
            return
        if not data.startswith(b"\xff\xd8") or not data.endswith(b"\xff\xd9"):
            return
        stamp = msg.header.stamp
        timestamp = stamp.sec + stamp.nanosec / 1e9
        now = self._node.get_clock().now().nanoseconds / 1e9
        # Both built-in and external Go2 publishers stamp their frames. Reject
        # delayed DDS data as well as a locally stale cache; allow unstamped JPEG.
        age = now - timestamp if timestamp else 0.0
        if age > _MAX_FRAME_AGE_S or age < -_MAX_FRAME_AGE_S:
            return
        with self._condition:
            stream = self._streams[topic]
            stream["sequence"] += 1
            stream["latest"] = (data, time.monotonic() - max(0.0, age), stream["sequence"])
            self._condition.notify_all()

    def _frame(self, source, after_sequence=None, timeout_s=_FIRST_FRAME_TIMEOUT_S, cancel=None):
        deadline = time.monotonic() + timeout_s
        with self._condition:
            while True:
                if cancel is not None and cancel.is_set():
                    raise RuntimeError("Video recording was cancelled")
                frame = self._streams[source["topic"]]["latest"]
                if (frame is not None and time.monotonic() - frame[1] <= _MAX_FRAME_AGE_S
                        and (after_sequence is None or frame[2] > after_sequence)):
                    return frame
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise RuntimeError(f"No fresh JPEG frame on {source['topic']}; start the source camera and check ROS connectivity")
                self._condition.wait(min(0.1, remaining))

    def _info(self):
        with self._recording_lock:
            try:
                source = self._resolve_source({})
                message = ""
            except ValueError as exc:
                source, message = None, str(exc)
            active = self._active_recording
            public = ({key: active[key] for key in (
                "action_id", "state", "duration_s", "started_at", "source")} if active else None)
            last = self._last_recording
            camera, instance_id = self._camera, self._external_instance_id
        with self._condition:
            frame = self._streams[source["topic"]]["latest"] if source else None
            age = time.monotonic() - frame[1] if frame else None
        ready = age is not None and age <= _MAX_FRAME_AGE_S
        return {"ok": ready, "state": "ready" if ready else "waiting_for_camera",
                "source": source, "camera": camera, "external_instance_id": instance_id,
                "message": message, "output_dir": str(self._output_dir),
                "photos_dir": str(self._output_dir / "photos"),
                "videos_dir": str(self._output_dir / "videos"),
                "fps": self._fps, "max_duration_s": self._max_duration_s,
                "latest_frame_age_s": round(age, 3) if age is not None else None,
                "encoder_available": all(shutil.which(name) is not None for name in ("ffmpeg", "ffprobe")),
                "active_recording": public, "last_recording": last}

    def start(self):
        # Bundle startup precedes executor spinning: do not wait here for DDS.
        return self._info()

    def _new_path(self, directory, prefix, suffix):
        target = self._output_dir / directory
        target.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        return target / f"{prefix}_{stamp}_{uuid4().hex[:8]}{suffix}"

    def _capture_photo(self, args):
        path = None
        try:
            with self._recording_lock:
                source = self._resolve_source(args)
            # Every photo is a frame received after this request, including after
            # switching cameras. Cached data from the other camera cannot leak.
            with self._condition:
                sequence = self._streams[source["topic"]]["sequence"]
            data, received, _ = self._frame(source, sequence)
            path = self._new_path("photos", "IMG", ".jpg")
            path.write_bytes(data)
            return {"ok": True, "media_type": "photo", "file_path": str(path), "source": source,
                    "captured_at": datetime.now().isoformat(timespec="seconds"),
                    "frame_age_s": round(time.monotonic() - received, 3)}
        except Exception as exc:
            if path is not None:
                self._remove_partial(path)
            return {"ok": False, "code": "CAPTURE_FAILED", "message": str(exc)}

    @staticmethod
    def _remove_partial(path):
        try:
            path.unlink(missing_ok=True)
        except OSError as exc:
            log.error("[vision_capture] Could not remove incomplete file %s: %s", path, exc)
            return str(exc)
        return None

    @staticmethod
    def _write_frame(process, data, cancel):
        # Never block indefinitely on a wedged encoder or hold a Python buffered
        # stdin lock that would prevent stop() from cancelling the recording.
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

    def _record_video(self, active):
        cancel = active["cancel"]
        source = active["source"]
        process = None
        path = None
        completed = False
        try:
            # Start from a frame arriving after the request, not a cached image.
            with self._condition:
                sequence = self._streams[source["topic"]]["sequence"]
            frame = self._frame(source, sequence, cancel=cancel)
            path = self._new_path("videos", "video", ".mp4")
            with tempfile.TemporaryFile() as errors:
                process = subprocess.Popen([
                    "ffmpeg", "-nostdin", "-y", "-loglevel", "error",
                    "-use_wallclock_as_timestamps", "1", "-f", "mjpeg",
                    "-probesize", "32", "-analyzeduration", "0",
                    "-framerate", str(self._fps), "-i", "pipe:0", "-an",
                    "-c:v", "libx264", "-preset", "ultrafast", "-threads", "2",
                    # Normalize the timeline, retain real-time playback at a
                    # constant output rate, and extend the last frame to the
                    # requested endpoint. VFR alone can end one tick early
                    # (e.g. 4.934s for a 5s request).
                    "-vf", ("scale=out_range=tv,pad=ceil(iw/2)*2:ceil(ih/2)*2,"
                            f"setpts=PTS-STARTPTS,fps={self._fps},tpad=stop_mode=clone:stop=-1"),
                    "-frames:v", str(self._fps * active["duration_s"]),
                    "-pix_fmt", "yuv420p", "-color_range", "tv", "-vsync", "vfr",
                    "-movflags", "+faststart", str(path),
                ], stdin=subprocess.PIPE, stdout=subprocess.DEVNULL, stderr=errors, bufsize=0)
                os.set_blocking(process.stdin.fileno(), False)
                with self._recording_lock:
                    active["process"] = process
                started = time.monotonic()
                deadline = started + active["duration_s"]
                frames = 0
                while True:
                    tick = time.monotonic()
                    self._write_frame(process, frame[0], cancel)
                    frames += 1
                    cancel.wait(min(max(0, 1 / self._fps - (time.monotonic() - tick)),
                                    max(0, deadline - time.monotonic())))
                    remaining = deadline - time.monotonic()
                    if remaining <= 0:
                        break
                    try:
                        frame = self._frame(source, frame[2], min(remaining, _MAX_FRAME_AGE_S), cancel)
                    except RuntimeError:
                        # A healthy low-fps source may have no final frame exactly
                        # at the deadline. A stalled source must still fail.
                        if (not cancel.is_set() and time.monotonic() >= deadline
                                and time.monotonic() - frame[1] < _MAX_FRAME_AGE_S):
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
                if process.returncode != 0 or not path.exists() or path.stat().st_size == 0:
                    errors.seek(0)
                    raise RuntimeError(errors.read(4096).decode("utf-8", "replace") or "ffmpeg failed to create MP4")
            media = self._probe_video(path, active["duration_s"], self._fps)
            if cancel.is_set():
                raise RuntimeError("Video recording was cancelled")
            completed = True
            return {"ok": True, "media_type": "video", "file_path": str(path),
                    "recorded_duration_s": media["duration_s"], "frames": frames,
                    "captured_at": _timestamp()}
        except Exception as exc:
            return {"ok": False, "code": "RECORD_CANCELLED" if cancel.is_set() else "RECORD_FAILED",
                    "message": str(exc)}
        finally:
            if process is not None:
                if process.poll() is None:
                    process.kill()
                    process.wait(timeout=2)
                if not process.stdin.closed:
                    process.stdin.close()
            with self._recording_lock:
                active["process"] = None
            if not completed and path is not None:
                self._remove_partial(path)

    @staticmethod
    def _probe_video(path, requested, fps):
        """Only acknowledge completion after the muxed file has the right duration."""
        metadata = json.loads(subprocess.check_output([
            "ffprobe", "-v", "error", "-select_streams", "v:0",
            "-show_entries", "format=duration:stream=duration,nb_frames", "-of", "json", str(path),
        ], stderr=subprocess.PIPE, timeout=5))
        stream = metadata["streams"][0]
        duration = float(metadata["format"]["duration"])
        stream_duration = float(stream["duration"])
        frames = int(stream["nb_frames"])
        if (not abs(duration - requested) <= 0.001
                or not abs(stream_duration - requested) <= 0.001 or frames != requested * fps):
            raise RuntimeError(f"Invalid completed video: duration={duration}s, stream={stream_duration}s, frames={frames}")
        return {"duration_s": duration, "frames": frames}

    def _notify_complete(self, action_id, status, result):
        payload = json.dumps({"action_id": action_id, "status": status,
                              "result": result, "tool": self.PREFIX, "ts": time.time()}).encode()
        url = os.environ.get("AGENT_CORE_URL", "https://localhost:15678").rstrip("/")
        ctx = ssl.create_default_context()
        if url.startswith(("https://localhost:", "https://127.0.0.1:")):
            ctx.check_hostname = False
            ctx.verify_mode = ssl.CERT_NONE
        try:
            request = urllib.request.Request(url + "/api/acp/complete", data=payload,
                                             headers={"Content-Type": "application/json"})
            with urllib.request.urlopen(request, timeout=3, context=ctx):
                pass
        except Exception as exc:
            log.warning("[vision_capture] ACP completion delivery failed: %s", exc)

    def _record_video_async(self, active):
        try:
            result = self._record_video(active)
        except Exception as exc:
            result = {"ok": False, "code": "RECORD_FAILED", "message": str(exc)}
        with self._recording_lock:
            # stop() may race with successful encoding; cancellation wins until
            # the terminal result is committed under this same lock.
            if active["cancel"].is_set():
                cleanup_error = None
                if result.get("ok"):
                    cleanup_error = self._remove_partial(Path(result["file_path"]))
                result = {"ok": False, "code": "RECORD_CANCELLED", "message": "Video recording was cancelled"}
                if cleanup_error:
                    result["cleanup_error"] = cleanup_error
            status = "completed" if result.get("ok") else (
                "cancelled" if result.get("code") == "RECORD_CANCELLED" else "error")
            self._last_recording = {"action_id": active["action_id"], "status": status, "result": result}
            active["state"] = status
            active["finished"] = True
        # The worker owns the only terminal notification, including shutdown.
        try:
            self._notify_complete(active["action_id"], status, result)
        finally:
            with self._recording_lock:
                self._active_recording = None

    def _start_video_recording(self, args):
        requested = args.get("duration_s", 5)
        if type(requested) is not int or not 1 <= requested <= self._max_duration_s:
            return {"ok": False, "code": "INVALID_DURATION",
                    "message": f"duration_s must be an integer between 1 and {self._max_duration_s}"}
        if shutil.which("ffmpeg") is None or shutil.which("ffprobe") is None:
            return {"ok": False, "code": "RECORD_FAILED", "message": "ffmpeg and ffprobe are required"}
        with self._recording_lock:
            if self._active_recording is not None:
                return {"ok": False, "code": "RECORD_IN_PROGRESS", "message": "A video recording is already in progress"}
            try:
                source = self._resolve_source(args)
            except ValueError as exc:
                return {"ok": False, "code": "RECORD_FAILED", "message": str(exc)}
            action_id = f"vision_capture_record_video_{uuid4().hex}"
            queued_at = _timestamp()
            active = {"action_id": action_id, "state": "recording", "duration_s": requested,
                      "started_at": queued_at, "queued_at": queued_at, "queued_mono": time.monotonic(),
                      "cancel": threading.Event(), "process": None, "source": source}
            thread = threading.Thread(target=self._record_video_async, args=(active,),
                                      daemon=True, name="go2_vision_capture_record_video")
            active["thread"] = thread
            self._active_recording = active
            try:
                thread.start()
            except Exception:
                self._active_recording = None
                raise
        return {"ok": True, "state": "queued", "action_id": action_id,
                "media_type": "video", "requested_duration_s": requested, "source": source,
                "queued_at": queued_at,
                "message": "Recording queued; ACP completion follows only after the MP4 is finalized and verified."}

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
        return {"ok": True, "state": "stopping" if stopping else "idle", "action_id": active["action_id"]}

    def dispatch(self, action, args):
        if action == "config":
            with self._recording_lock:
                camera = args.get("camera", self._camera)
                instance_id = args.get("external_instance_id", self._external_instance_id)
                if camera not in ("front", "external") or not isinstance(instance_id, str):
                    return {"ok": False, "code": "INVALID_CAMERA", "message": "Invalid camera configuration"}
                if (self._active_recording is not None
                        and (camera, instance_id) != (self._camera, self._external_instance_id)):
                    return {"ok": False, "code": "RECORD_IN_PROGRESS", "message": "Stop recording before changing camera configuration"}
                self._camera, self._external_instance_id = camera, instance_id
            return {"ok": True, "camera": camera, "external_instance_id": instance_id}
        if action == "start":
            return self.start()
        if action == "info":
            return self._info()
        if action == "capture_photo":
            return self._capture_photo(args)
        if action == "record_video":
            return self._start_video_recording(args)
        if action == "stop":
            return self.stop()
        if action == "list_cameras":
            return {"ok": True, "cameras": self._sources()}
        return None
