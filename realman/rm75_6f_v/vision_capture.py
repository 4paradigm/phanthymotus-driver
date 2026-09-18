#!/usr/bin/env python3
"""Persistent RGB photos and asynchronous MP4 recording from RealSense ext_camera.

Subscribes to the JPEG topic published by the RealSense RGB stream (ext_camera
card, channel=rgb) and provides:
  - capture_photo : save a single JPEG photo
  - record_video  : record an H.264 MP4 video via ffmpeg
  - stop          : cancel an active recording
"""

from __future__ import annotations

import json
import logging
import os
from pathlib import Path
import select
import shutil
import ssl
import subprocess
import threading
import time
import urllib.request
from uuid import uuid4

from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage

log = logging.getLogger(__name__)

_FIRST_FRAME_TIMEOUT_S = 5.0
_MAX_FRAME_AGE_S = 3.0


class VisionCapturePlugin:
    PREFIX = "vision_capture"

    def __init__(self, plugin_config: dict, namespace: str, executor,
                 ext_camera=None, context=None):
        self._namespace = namespace
        self._ext_camera = ext_camera
        self._output_dir = Path(
            plugin_config.get(
                "output_dir",
                "/opt/phanthy-motus/data/vision_capture",
            )
        ).expanduser()
        self._fps = max(1, min(15, int(plugin_config.get("fps", 15))))
        self._max_duration_s = max(
            1, min(30, int(plugin_config.get("max_duration_s", 30)))
        )
        self._condition = threading.Condition()
        self._streams: dict = {}
        self._recording_lock = threading.Lock()
        self._active_recording: dict | None = None
        self._last_recording: dict | None = None
        self._node = Node("realman_vision_capture", context=context)
        executor.add_node(self._node)

    # ── Tool definition ────────────────────────────────────────────────────

    def get_tools(self) -> list:
        return [
            {
                "name": self.PREFIX,
                "type": "actuator",
                "multiInstance": False,
                "description": (
                    "Save a RealSense RGB JPEG photo or record a silent "
                    "H.264 MP4 video to persistent storage."
                ),
                "inputSchema": {
                    "type": "object",
                    "properties": {
                        "action": {
                            "type": "string",
                            "enum": [
                                "start",
                                "capture_photo",
                                "record_video",
                                "list_cameras",
                                "info",
                                "stop",
                            ],
                        },
                        "instance_id": {
                            "type": "string",
                            "description": (
                                "RealSense ext_camera instance id to use; "
                                "omitted to use the configured default."
                            ),
                        },
                        "duration_s": {
                            "type": "integer",
                            "minimum": 1,
                            "maximum": self._max_duration_s,
                            "default": min(
                                5, self._max_duration_s
                            ),
                        },
                    },
                    "required": ["action"],
                    "additionalProperties": False,
                    "x-action-params": {
                        "start": {
                            "params": [],
                            "description": "Subscribe to the RealSense RGB topic.",
                        },
                        "capture_photo": {
                            "params": ["instance_id"],
                            "description": "Save the latest RGB frame as JPG.",
                        },
                        "record_video": {
                            "params": ["duration_s", "instance_id"],
                            "description": (
                                "Record RGB video to MP4 (default 5 s)."
                            ),
                        },
                        "list_cameras": {
                            "params": [],
                            "description": "List available RGB sources.",
                        },
                        "info": {
                            "params": [],
                            "description": (
                                "View source, output dir, and recording status."
                            ),
                        },
                        "stop": {
                            "params": [],
                            "description": (
                                "Cancel the active recording and discard "
                                "incomplete file."
                            ),
                        },
                    },
                    "x-completion": {
                        "actions": ["record_video"],
                        "timeout": self._max_duration_s + 20,
                    },
                },
                "configSchema": {"type": "object", "properties": {}},
            },
        ]

    # ── Source discovery ───────────────────────────────────────────────────

    def _sources(self) -> list[dict]:
        sources: list[dict] = []
        if self._ext_camera is not None:
            info = self._ext_camera.dispatch("info", {})
            for instance_id in info.get("active_instances", []):
                status = self._ext_camera.dispatch(
                    "info", {"instance_id": instance_id}
                )
                if status.get("state") != "running":
                    continue
                if status.get("channel", "rgb") != "rgb":
                    continue
                for output in status.get("topic_out", []):
                    if (
                        output.get("format") == "image/jpeg"
                        and output.get("topic")
                    ):
                        sources.append(
                            {
                                "camera": "external",
                                "external_instance_id": instance_id,
                                "name": status.get(
                                    "device_name"
                                )
                                or instance_id,
                                "device_path": status.get(
                                    "device_path", ""
                                ),
                                "topic": output["topic"],
                            }
                        )
        return sources

    def _resolve_source(self, args: dict) -> dict:
        instance_id = args.get(
            "instance_id", ""
        )
        if not instance_id or self._ext_camera is None:
            raise ValueError("instance_id is required for vision_capture")

        status = self._ext_camera.dispatch(
            "info", {"instance_id": instance_id}
        )
        if status.get("state") != "running":
            raise ValueError(
                f"ext_camera instance '{instance_id}' is not running"
            )
        if status.get("channel", "rgb") != "rgb":
            raise ValueError(
                f"instance '{instance_id}' uses channel "
                f"'{status.get('channel')}', expected rgb"
            )

        topic: str | None = None
        for output in status.get("topic_out", []):
            if output.get("format") == "image/jpeg" and output.get("topic"):
                topic = output["topic"]
                break
        if not topic:
            raise ValueError(
                f"No RGB jpeg topic found for instance '{instance_id}'"
            )

        source: dict = {
            "camera": "external",
            "external_instance_id": instance_id,
            "name": status.get("device_name") or instance_id,
            "device_path": status.get("device_path", ""),
            "topic": topic,
        }

        with self._condition:
            if topic not in self._streams:
                self._streams[topic] = {
                    "latest": None,
                    "sequence": 0,
                }
                try:
                    self._streams[topic]["subscription"] = (
                        self._node.create_subscription(
                            CompressedImage,
                            topic,
                            lambda msg, s=source: self._on_frame(s, msg),
                            qos_profile_sensor_data,
                        )
                    )
                except Exception:
                    del self._streams[topic]
                    raise
        return source

    # ── Frame callback ─────────────────────────────────────────────────────

    def _on_frame(self, source: dict, msg: CompressedImage) -> None:
        data = bytes(msg.data)
        fmt = msg.format.lower()
        if "jpeg" not in fmt and "jpg" not in fmt:
            return
        if not data.startswith(b"\xff\xd8") or not data.endswith(b"\xff\xd9"):
            return
        stamp = msg.header.stamp
        timestamp = stamp.sec + stamp.nanosec / 1e9
        now = self._node.get_clock().now().nanoseconds / 1e9
        age = now - timestamp if timestamp else 0.0
        if age > _MAX_FRAME_AGE_S or age < -_MAX_FRAME_AGE_S:
            return
        with self._condition:
            stream = self._streams[source["topic"]]
            stream["sequence"] += 1
            stream["latest"] = (
                data,
                time.monotonic() - max(0.0, age),
                stream["sequence"],
            )
            self._condition.notify_all()

    # ── Frame fetching ─────────────────────────────────────────────────────

    def _frame(
        self,
        source: dict,
        after_sequence: int | None = None,
        timeout_s: float = _FIRST_FRAME_TIMEOUT_S,
        cancel: threading.Event | None = None,
    ) -> tuple[bytes, float, int]:
        deadline = time.monotonic() + timeout_s
        with self._condition:
            while True:
                if cancel is not None and cancel.is_set():
                    raise RuntimeError("Video recording was cancelled")
                frame = self._streams[source["topic"]]["latest"]
                if (
                    frame is not None
                    and time.monotonic() - frame[1] <= _MAX_FRAME_AGE_S
                    and (
                        after_sequence is None
                        or frame[2] > after_sequence
                    )
                ):
                    return frame
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise RuntimeError(
                        f"No fresh JPEG frame on {source['topic']}; "
                        "start the ext_camera instance and check ROS "
                        "connectivity"
                    )
                self._condition.wait(min(0.1, remaining))

    # ── File helpers ───────────────────────────────────────────────────────

    def _new_path(self, directory: str, prefix: str, suffix: str) -> Path:
        target = self._output_dir / directory
        target.mkdir(parents=True, exist_ok=True)
        stamp = time.strftime("%Y%m%d_%H%M%S_%f")
        return target / f"{prefix}_{stamp}_{uuid4().hex[:8]}{suffix}"

    @staticmethod
    def _remove_partial(path: Path) -> str | None:
        try:
            path.unlink(missing_ok=True)
        except OSError as exc:
            log.error(
                "[vision_capture] Could not remove partial file %s: %s",
                path,
                exc,
            )
            return str(exc)
        return None

    # ── Photo capture ──────────────────────────────────────────────────────

    def _capture_photo(self, args: dict) -> dict:
        path: Path | None = None
        try:
            with self._recording_lock:
                source = self._resolve_source(args)
            with self._condition:
                sequence = self._streams[source["topic"]]["sequence"]
            data, received, _ = self._frame(source, sequence)
            path = self._new_path("photos", "IMG", ".jpg")
            path.write_bytes(data)
            return {
                "ok": True,
                "media_type": "photo",
                "file_path": str(path),
                "source": source,
                "captured_at": time.strftime("%Y-%m-%dT%H:%M:%S"),
                "frame_age_s": round(
                    time.monotonic() - received, 3
                ),
            }
        except Exception as exc:
            if path is not None:
                self._remove_partial(path)
            return {
                "ok": False,
                "code": "CAPTURE_FAILED",
                "message": str(exc),
            }

    # ── Video recording ────────────────────────────────────────────────────

    @staticmethod
    def _write_frame(
        process: subprocess.Popen, data: bytes, cancel: threading.Event
    ) -> None:
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
                        os.write(
                            process.stdin.fileno(), pending
                        )
                    ]
                except BlockingIOError:
                    pass

    def _record_video(self, active: dict) -> dict:
        cancel = active["cancel"]
        source = active["source"]
        process: subprocess.Popen | None = None
        path: Path | None = None
        completed = False
        try:
            with self._condition:
                sequence = self._streams[source["topic"]]["sequence"]
            frame = self._frame(source, sequence, cancel=cancel)
            path = self._new_path("videos", "video", ".mp4")
            with subprocess.Popen(
                [
                    "ffmpeg",
                    "-nostdin",
                    "-y",
                    "-loglevel",
                    "error",
                    "-use_wallclock_as_timestamps",
                    "1",
                    "-f",
                    "mjpeg",
                    "-probesize",
                    "32",
                    "-analyzeduration",
                    "0",
                    "-framerate",
                    str(self._fps),
                    "-i",
                    "pipe:0",
                    "-an",
                    "-c:v",
                    "libx264",
                    "-preset",
                    "ultrafast",
                    "-threads",
                    "2",
                    "-vf",
                    (
                        "scale=out_range=tv,pad=ceil(iw/2)*2:"
                        "ceil(ih/2)*2,setpts=PTS-STARTPTS,fps="
                        f"{self._fps},tpad=stop_mode=clone:"
                        "stop=-1"
                    ),
                    "-frames:v",
                    str(self._fps * active["duration_s"]),
                    "-pix_fmt",
                    "yuv420p",
                    "-color_range",
                    "tv",
                    "-vsync",
                    "vfr",
                    "-movflags",
                    "+faststart",
                    str(path),
                ],
                stdin=subprocess.PIPE,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.PIPE,
                bufsize=0,
            ) as process:
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
                    cancel.wait(
                        min(
                            max(
                                0,
                                1 / self._fps
                                - (time.monotonic() - tick),
                            ),
                            max(0, deadline - time.monotonic()),
                        )
                    )
                    remaining = deadline - time.monotonic()
                    if remaining <= 0:
                        break
                    try:
                        frame = self._frame(
                            source,
                            frame[2],
                            min(remaining, _MAX_FRAME_AGE_S),
                            cancel,
                        )
                    except RuntimeError:
                        if (
                            not cancel.is_set()
                            and time.monotonic() >= deadline
                            and time.monotonic()
                            - frame[1]
                            < _MAX_FRAME_AGE_S
                        ):
                            break
                        raise
                if (
                    frames
                    < min(2, self._fps * active["duration_s"])
                ):
                    raise RuntimeError(
                        "Not enough fresh frames to record video"
                    )
                process.stdin.close()
                encode_deadline = time.monotonic() + 10
                while process.poll() is None:
                    if cancel.wait(0.1):
                        raise RuntimeError(
                            "Video recording was cancelled"
                        )
                    if time.monotonic() >= encode_deadline:
                        raise RuntimeError(
                            "ffmpeg finalization timed out"
                        )
                if (
                    process.returncode != 0
                    or not path.exists()
                    or path.stat().st_size == 0
                ):
                    stderr = process.stderr.read(
                        4096
                    ).decode("utf-8", "replace")
                    raise RuntimeError(
                        stderr
                        or "ffmpeg failed to create MP4"
                    )
            if cancel.is_set():
                raise RuntimeError(
                    "Video recording was cancelled"
                )
            completed = True
            return {
                "ok": True,
                "media_type": "video",
                "file_path": str(path),
                "source": source,
                "recorded_duration_s": active["duration_s"],
                "frames": frames,
                "encoded_frames": self._fps
                * active["duration_s"],
                "captured_at": time.strftime(
                    "%Y-%m-%dT%H:%M:%S"
                ),
            }
        except Exception as exc:
            return {
                "ok": False,
                "code": (
                    "RECORD_CANCELLED"
                    if cancel.is_set()
                    else "RECORD_FAILED"
                ),
                "message": str(exc),
            }
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

    # ── ACP completion callback ────────────────────────────────────────────

    def _notify_complete(
        self, action_id: str, status: str, result: dict
    ) -> None:
        payload = json.dumps(
            {
                "action_id": action_id,
                "status": status,
                "result": result,
                "tool": self.PREFIX,
                "ts": time.time(),
            }
        ).encode()
        url = (
            os.environ.get("AGENT_CORE_URL", "https://localhost:15678")
            .rstrip("/")
        )
        ctx = ssl.create_default_context()
        if url.startswith(
            ("https://localhost:", "https://127.0.0.1:")
        ):
            ctx.check_hostname = False
            ctx.verify_mode = ssl.CERT_NONE
        try:
            request = urllib.request.Request(
                url + "/api/acp/complete",
                data=payload,
                headers={"Content-Type": "application/json"},
            )
            with urllib.request.urlopen(
                request, timeout=3, context=ctx
            ):
                pass
        except Exception as exc:
            log.warning(
                "[vision_capture] ACP completion delivery failed: %s",
                exc,
            )

    # ── Async recording wrapper ────────────────────────────────────────────

    def _record_video_async(self, active: dict) -> None:
        try:
            result = self._record_video(active)
        except Exception as exc:
            result = {
                "ok": False,
                "code": "RECORD_FAILED",
                "message": str(exc),
            }
        with self._recording_lock:
            if active["cancel"].is_set():
                cleanup_error = None
                if result.get("ok"):
                    cleanup_error = self._remove_partial(
                        Path(result["file_path"])
                    )
                result = {
                    "ok": False,
                    "code": "RECORD_CANCELLED",
                    "message": "Video recording was cancelled",
                }
                if cleanup_error:
                    result["cleanup_error"] = cleanup_error
            terminal_status = (
                "completed"
                if result.get("ok")
                else (
                    "cancelled"
                    if result.get("code") == "RECORD_CANCELLED"
                    else "error"
                )
            )
            self._last_recording = {
                "action_id": active["action_id"],
                "status": terminal_status,
                "result": result,
            }
            active["state"] = terminal_status
            active["finished"] = True
        try:
            self._notify_complete(
                active["action_id"], terminal_status, result
            )
        finally:
            with self._recording_lock:
                self._active_recording = None

    def _start_video_recording(self, args: dict) -> dict:
        requested = args.get(
            "duration_s", min(5, self._max_duration_s)
        )
        if (
            type(requested) is not int
            or not 1 <= requested <= self._max_duration_s
        ):
            return {
                "ok": False,
                "code": "INVALID_DURATION",
                "message": (
                    f"duration_s must be an integer between 1 and "
                    f"{self._max_duration_s}"
                ),
            }
        if shutil.which("ffmpeg") is None:
            return {
                "ok": False,
                "code": "RECORD_FAILED",
                "message": "ffmpeg is not installed",
            }
        with self._recording_lock:
            if self._active_recording is not None:
                return {
                    "ok": False,
                    "code": "RECORD_IN_PROGRESS",
                    "message": (
                        "A video recording is already in progress"
                    ),
                }
            try:
                source = self._resolve_source(args)
            except ValueError as exc:
                return {
                    "ok": False,
                    "code": "RECORD_FAILED",
                    "message": str(exc),
                }
            action_id = (
                f"vision_capture_record_video_"
                f"{uuid4().hex}"
            )
            active = {
                "action_id": action_id,
                "state": "recording",
                "duration_s": requested,
                "started_at": time.strftime(
                    "%Y-%m-%dT%H:%M:%S"
                ),
                "cancel": threading.Event(),
                "process": None,
                "source": source,
            }
            thread = threading.Thread(
                target=self._record_video_async,
                args=(active,),
                daemon=True,
                name="realman_vision_capture_record_video",
            )
            active["thread"] = thread
            self._active_recording = active
            try:
                thread.start()
            except Exception:
                self._active_recording = None
                raise
        return {
            "ok": True,
            "state": "queued",
            "action_id": action_id,
            "media_type": "video",
            "requested_duration_s": requested,
            "source": source,
        }

    def stop(self) -> dict:
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
        return {
            "ok": True,
            "state": "stopping" if stopping else "idle",
            "action_id": active["action_id"],
        }

    # ── Status ─────────────────────────────────────────────────────────────

    def _info(self) -> dict:
        with self._recording_lock:
            active = self._active_recording
            public = (
                {
                    key: active[key]
                    for key in (
                        "action_id",
                        "state",
                        "duration_s",
                        "started_at",
                        "source",
                    )
                }
                if active
                else None
            )
            last = self._last_recording

        with self._condition:
            frame_age: float | None = None
            for stream in self._streams.values():
                frame = stream["latest"]
                if frame is not None:
                    frame_age = round(
                        time.monotonic() - frame[1], 3
                    )
                    break
        ready = (
            frame_age is not None and frame_age <= _MAX_FRAME_AGE_S
        )
        return {
            "ok": ready,
            "state": (
                "ready" if ready else "waiting_for_camera"
            ),
            "output_dir": str(self._output_dir),
            "photos_dir": str(self._output_dir / "photos"),
            "videos_dir": str(self._output_dir / "videos"),
            "fps": self._fps,
            "max_duration_s": self._max_duration_s,
            "latest_frame_age_s": frame_age,
            "encoder_available": shutil.which(
                "ffmpeg"
            ) is not None,
            "active_recording": public,
            "last_recording": last,
            "cameras": self._sources(),
        }

    def start(self) -> dict:
        return self._info()

    # ── Dispatch ───────────────────────────────────────────────────────────

    def dispatch(self, action: str, args: dict) -> dict | None:
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
