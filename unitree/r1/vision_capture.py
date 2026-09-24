"""R1 main-camera photos and short videos with microphone audio."""

from datetime import datetime
import json
import math
import os
from pathlib import Path
import shutil
import ssl
import struct
import subprocess
import tempfile
import threading
import time
import urllib.request
from uuid import uuid4
import wave


SAMPLE_RATE = 16000
MAX_FRAME_AGE_S = 3.0


def require_encoder():
    """Return ffmpeg's path only when both encoding and verification are available."""
    ffmpeg = shutil.which("ffmpeg")
    if not ffmpeg or not shutil.which("ffprobe"):
        raise RuntimeError("ffmpeg and ffprobe are required for video recording")
    return ffmpeg


def write_audio_timeline(path, chunks, started, duration_s):
    """Place received 16 kHz PCM chunks on the recording clock, filling gaps."""
    samples = bytearray(round(duration_s * SAMPLE_RATE) * 2)
    minimum = None
    maximum = None
    for received, pcm in chunks:
        if not pcm or len(pcm) % 2:
            continue
        start_sample = round((received - started) * SAMPLE_RATE) - len(pcm) // 2
        source_start = max(0, -start_sample)
        destination = max(0, start_sample)
        count = min(len(pcm) // 2 - source_start, len(samples) // 2 - destination)
        if count <= 0:
            continue
        portion = pcm[source_start * 2:(source_start + count) * 2]
        samples[destination * 2:(destination + count) * 2] = portion
        for (value,) in struct.iter_unpack("<h", portion):
            minimum = value if minimum is None else min(minimum, value)
            maximum = value if maximum is None else max(maximum, value)
    if minimum is None or minimum == maximum:
        raise ValueError("No usable audio samples arrived during recording")
    with wave.open(str(path), "wb") as output:
        output.setnchannels(1)
        output.setsampwidth(2)
        output.setframerate(SAMPLE_RATE)
        output.writeframes(samples)


def encode_video(frames_path, audio_path, output_path, fps, duration_s):
    """Encode a fixed-rate JPEG stream and PCM audio, then verify both tracks."""
    ffmpeg = require_encoder()
    command = [
        ffmpeg, "-nostdin", "-y", "-v", "error",
        "-f", "mjpeg", "-r", str(fps), "-i", str(frames_path),
        "-i", str(audio_path), "-t", str(duration_s),
        "-c:v", "libx264", "-pix_fmt", "yuv420p",
        "-c:a", "aac", "-b:a", "96k", "-movflags", "+faststart",
        str(output_path),
    ]
    subprocess.run(command, check=True, capture_output=True, timeout=duration_s + 30)
    probe = subprocess.run([
        "ffprobe", "-v", "error", "-show_entries",
        "stream=codec_type:format=duration", "-of", "json", str(output_path),
    ], check=True, capture_output=True, timeout=5)
    details = json.loads(probe.stdout)
    tracks = {stream["codec_type"] for stream in details.get("streams", [])}
    actual_duration = float(details.get("format", {}).get("duration", 0))
    if not {"video", "audio"}.issubset(tracks) or not math.isclose(
            actual_duration, duration_s, abs_tol=0.25):
        raise RuntimeError("Recorded MP4 lacks audio/video or has an invalid duration")
    return actual_duration


class VisionCapturePlugin:
    PREFIX = "vision_capture"

    def __init__(self, plugin_config, namespace, executor):
        # 中文说明：只订阅现有 ROS2 流，不重复打开相机或麦克风硬件。
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import CompressedImage
        from audio_msgs.msg import AudioChunk

        self._camera_topic = f"/{namespace}/camera/main"
        self._audio_topic = f"/{namespace}/mic/audio"
        self._output_dir = Path(plugin_config.get(
            "output_dir", "/opt/phanthy-motus/data/vision_capture"))
        self._fps = max(1, min(15, int(plugin_config.get("fps", 10))))
        self._max_duration_s = max(1, min(30, int(plugin_config.get("max_duration_s", 30))))
        self._condition = threading.Condition()
        self._frame = None
        self._frame_sequence = 0
        self._last_audio_at = 0.0
        self._active = None
        self._last_result = None
        self._node = Node("r1_vision_capture")
        self._node.create_subscription(
            CompressedImage, self._camera_topic, self._on_frame, qos_profile_sensor_data)
        self._node.create_subscription(
            AudioChunk, self._audio_topic, self._on_audio, qos_profile_sensor_data)
        executor.add_node(self._node)

    def get_tool(self):
        return {
            "name": self.PREFIX, "type": "actuator", "multiInstance": False,
            "description": "Save an R1 main-camera JPEG photo or a short MP4 with R1 microphone audio.",
            "inputSchema": {
                "type": "object", "properties": {
                    "action": {"type": "string", "enum": [
                        "start", "capture_photo", "record_video", "info", "stop"]},
                    "duration_s": {"type": "integer", "minimum": 1,
                                   "maximum": self._max_duration_s, "default": 5},
                }, "required": ["action"], "additionalProperties": False,
                "x-action-params": {
                    "start": {"params": [], "description": "检查音画输入状态。"},
                    "capture_photo": {"params": [], "description": "拍摄并保存主相机 JPEG。"},
                    "record_video": {"params": ["duration_s"],
                                     "description": "录制主相机画面和麦克风声音。"},
                    "info": {"params": [], "description": "查看输入与最近一次录制结果。"},
                    "stop": {"params": [], "description": "取消录像并删除未完成文件。"},
                },
                "x-completion": {"actions": ["record_video"],
                                 "timeout": self._max_duration_s + 45},
            },
        }

    def _on_frame(self, message):
        data = bytes(message.data)
        if "jpeg" not in message.format.lower() or not data.startswith(b"\xff\xd8") \
                or not data.endswith(b"\xff\xd9"):
            return
        with self._condition:
            self._frame_sequence += 1
            self._frame = (data, time.monotonic(), self._frame_sequence)
            self._condition.notify_all()

    def _on_audio(self, message):
        if message.format not in ("pcm_16k_16bit_mono", "audio/pcm-16k"):
            return
        pcm = bytes(message.data)
        if not pcm or len(pcm) % 2:
            return
        now = time.monotonic()
        with self._condition:
            self._last_audio_at = now
            if self._active is not None and self._active.get("started_mono") is not None:
                self._active["audio"].append((now, pcm))
            self._condition.notify_all()

    def _wait_frame(self, after_sequence=None, timeout_s=5.0, max_age_s=MAX_FRAME_AGE_S,
                    cancel=None):
        deadline = time.monotonic() + timeout_s
        with self._condition:
            while True:
                if cancel is not None and cancel.is_set():
                    raise RuntimeError("Recording cancelled")
                frame = self._frame
                if (frame is not None and time.monotonic() - frame[1] <= max_age_s
                        and (after_sequence is None or frame[2] > after_sequence)):
                    return frame
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise RuntimeError("No fresh R1 main-camera JPEG frame arrived")
                self._condition.wait(min(0.1, remaining))

    def _path(self, folder, prefix, suffix):
        directory = self._output_dir / folder
        directory.mkdir(parents=True, exist_ok=True)
        return directory / (f"{prefix}_{datetime.now():%Y%m%d_%H%M%S_%f}_"
                            f"{uuid4().hex[:8]}{suffix}")

    def _photo(self):
        path = None
        try:
            with self._condition:
                sequence = self._frame_sequence
            frame = self._wait_frame(after_sequence=sequence)
            path = self._path("photos", "IMG", ".jpg")
            with path.open("xb") as output:
                output.write(frame[0])
            return {"ok": True, "media_type": "photo", "file_path": str(path),
                    "captured_at": datetime.now().astimezone().isoformat(timespec="seconds")}
        except Exception as exc:
            if path is not None:
                path.unlink(missing_ok=True)
            return {"ok": False, "code": "CAPTURE_FAILED", "message": str(exc)}

    def _notify(self, action_id, status, result):
        payload = json.dumps({"action_id": action_id, "status": status,
                              "result": result, "tool": self.PREFIX,
                              "ts": time.time()}).encode()
        url = os.environ.get("AGENT_CORE_URL", "https://localhost:15678").rstrip("/")
        context = None
        if url.startswith(("https://localhost:", "https://127.0.0.1:")):
            context = ssl._create_unverified_context()
        request = urllib.request.Request(
            url + "/api/acp/complete", data=payload,
            headers={"Content-Type": "application/json"})
        try:
            with urllib.request.urlopen(request, timeout=3, context=context):
                pass
        except Exception as exc:
            print(f"[r1 vision_capture] completion callback failed: {exc}", flush=True)

    def _record(self, active):
        output = active["path"]
        cancel = active["cancel"]
        try:
            with self._condition:
                sequence = self._frame_sequence
            first = self._wait_frame(after_sequence=sequence, cancel=cancel)
            with tempfile.TemporaryDirectory(dir=self._output_dir) as temporary:
                frames_path = Path(temporary) / "frames.mjpeg"
                audio_path = Path(temporary) / "audio.wav"
                started = time.monotonic()
                with self._condition:
                    active["started_mono"] = started
                with frames_path.open("wb") as frames:
                    for index in range(active["duration_s"] * self._fps):
                        target = started + index / self._fps
                        if cancel.wait(max(0.0, target - time.monotonic())):
                            raise RuntimeError("Recording cancelled")
                        frame = first if index == 0 else self._wait_frame(
                            timeout_s=1.0, max_age_s=1.0, cancel=cancel)
                        frames.write(frame[0])
                if cancel.wait(max(0.0, started + active["duration_s"] - time.monotonic())):
                    raise RuntimeError("Recording cancelled")
                with self._condition:
                    audio = list(active["audio"])
                write_audio_timeline(audio_path, audio, started, active["duration_s"])
                if cancel.is_set():
                    raise RuntimeError("Recording cancelled")
                duration = encode_video(
                    frames_path, audio_path, output, self._fps, active["duration_s"])
            if cancel.is_set():
                raise RuntimeError("Recording cancelled")
            result = {"ok": True, "media_type": "video", "file_path": str(output),
                      "recorded_duration_s": duration,
                      "captured_at": datetime.now().astimezone().isoformat(timespec="seconds")}
        except Exception as exc:
            output.unlink(missing_ok=True)
            result = {"ok": False,
                      "code": "RECORD_CANCELLED" if cancel.is_set() else "RECORD_FAILED",
                      "message": str(exc)}
        with self._condition:
            status = "completed" if result["ok"] else (
                "cancelled" if cancel.is_set() else "error")
            self._last_result = {"action_id": active["action_id"],
                                 "status": status, "result": result}
        self._notify(active["action_id"], status, result)
        with self._condition:
            self._active = None

    def _start_recording(self, args):
        duration = args.get("duration_s", 5)
        if type(duration) is not int or not 1 <= duration <= self._max_duration_s:
            return {"ok": False, "code": "INVALID_DURATION",
                    "message": f"duration_s must be 1..{self._max_duration_s}"}
        try:
            require_encoder()
        except RuntimeError as exc:
            return {"ok": False, "code": "RECORD_FAILED", "message": str(exc)}
        with self._condition:
            if self._active is not None:
                return {"ok": False, "code": "RECORD_IN_PROGRESS"}
            if time.monotonic() - self._last_audio_at > MAX_FRAME_AGE_S:
                return {"ok": False, "code": "MIC_UNAVAILABLE",
                        "message": "No recent R1 microphone data"}
            self._output_dir.mkdir(parents=True, exist_ok=True)
            active = {"action_id": f"r1_vision_capture_{uuid4().hex}",
                      "duration_s": duration, "cancel": threading.Event(),
                      "audio": [], "started_mono": None,
                      "path": self._path("videos", "video", ".mp4")}
            thread = threading.Thread(target=self._record, args=(active,), daemon=True)
            active["thread"] = thread
            self._active = active
            thread.start()
        return {"ok": True, "state": "queued", "action_id": active["action_id"],
                "media_type": "video", "file_path": str(active["path"]),
                "requested_duration_s": duration}

    def start(self):
        return {"state": "ready"}

    def stop(self):
        with self._condition:
            active = self._active
            if active is None:
                return {"state": "idle"}
            active["cancel"].set()
            self._condition.notify_all()
        active["thread"].join(timeout=5)
        return {"state": "stopping" if active["thread"].is_alive() else "idle",
                "action_id": active["action_id"]}

    def dispatch(self, action, args):
        if action == "start":
            return self.start()
        if action == "stop":
            return self.stop()
        if action == "capture_photo":
            return self._photo()
        if action == "record_video":
            return self._start_recording(args)
        if action == "info":
            with self._condition:
                frame_age = time.monotonic() - self._frame[1] if self._frame else None
                audio_age = time.monotonic() - self._last_audio_at if self._last_audio_at else None
                active = self._active
                return {"state": "recording" if active else "ready",
                        "camera_topic": self._camera_topic, "audio_topic": self._audio_topic,
                        "frame_age_s": round(frame_age, 3) if frame_age is not None else None,
                        "audio_age_s": round(audio_age, 3) if audio_age is not None else None,
                        "photos_dir": str(self._output_dir / "photos"),
                        "videos_dir": str(self._output_dir / "videos"),
                        "active_action_id": active["action_id"] if active else None,
                        "last_recording": self._last_result}
        return None
