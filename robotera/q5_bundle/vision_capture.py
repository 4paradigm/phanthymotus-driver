"""Persistent Q5 photo/video capture using the established camera worker."""

from __future__ import annotations

from datetime import datetime
from pathlib import Path
import subprocess
import time


CARD = "vision_capture"
_FIRST_FRAME_TIMEOUT_S = 5.0


class Plugin:
    def __init__(self, plugin_config, namespace, executor, client):
        del namespace, executor
        self._worker = getattr(client, "camera_worker", None)
        self._output_dir = Path(str(plugin_config.get(
            "output_dir", "/opt/phanthy-motus/data/vision_capture"))).expanduser()
        self._fps = max(1, min(15, int(plugin_config.get("fps", 10))))
        self._max_duration_s = max(1, min(30, int(plugin_config.get("max_duration_s", 30))))

    def get_tool(self):
        return {
            "name": CARD, "type": "actuator", "multiInstance": False,
            "description": "Capture a Q5 RGB photo or record a video (1–30 seconds) to persistent storage.",
            "inputSchema": {"type": "object", "properties": {
                "action": {"type": "string", "enum": ["start", "capture_photo", "record_video", "info", "stop"]},
                "duration_s": {"type": "integer", "minimum": 1, "maximum": 30, "default": 5,
                               "description": "默认值为5秒（可填写1–30秒）"},
            }, "required": ["action"], "additionalProperties": False,
                "x-action-params": {
                    "start": {"params": [], "description": "检查相机 worker 是否就绪。"},
                    "capture_photo": {"params": [], "description": "拍摄并保存一张当前 RGB 照片。"},
                    "record_video": {"params": ["duration_s"], "description": "录制并保存 1–30 秒 RGB 视频，默认 5 秒。"},
                    "info": {"params": [], "description": "查看保存目录与相机状态。"},
                    "stop": {"params": [], "description": "停止卡片。"},
                }},
        }

    def get_tools(self):
        return [self.get_tool()]

    def start(self):
        return {"state": "ready" if self._camera_ready() else "error"}

    def stop(self):
        pass

    def _camera_ready(self):
        return self._worker is not None and bool(getattr(self._worker, "_running", False))

    def _info(self):
        frame = None
        if self._camera_ready():
            try:
                frame, _ = self._frame(timeout_s=0)
            except RuntimeError:
                pass
        timestamp_ms = (frame or {}).get("timestamp_ms", 0)
        age = round(max(0.0, time.time() - timestamp_ms / 1000), 2) if timestamp_ms else None
        return {"ok": self._camera_ready(), "output_dir": str(self._output_dir),
                "photos_dir": str(self._output_dir / "photos"),
                "videos_dir": str(self._output_dir / "videos"), "fps": self._fps,
                "max_duration_s": self._max_duration_s, "latest_frame_age_s": age,
                "source": "q5_camera_worker"}

    def _frame(self, after_sequence=None, timeout_s=_FIRST_FRAME_TIMEOUT_S):
        if not self._camera_ready():
            raise RuntimeError("Q5 camera worker is unavailable")
        frame, sequence = self._worker.wait_for_frame("rgb", after_sequence, timeout_s)
        if not isinstance(frame, dict) or not frame.get("data"):
            raise RuntimeError("No RGB frame has arrived yet")
        timestamp_ms = frame.get("timestamp_ms", 0)
        if timestamp_ms and time.time() - timestamp_ms / 1000 > 3.0:
            frame, sequence = self._worker.wait_for_frame("rgb", sequence, timeout_s)
            if not isinstance(frame, dict) or not frame.get("data"):
                raise RuntimeError("No fresh RGB frame has arrived yet")
        return frame, sequence

    def _capture_photo(self, args):
        try:
            frame, _ = self._frame()
            directory = self._output_dir / "photos"
            directory.mkdir(parents=True, exist_ok=True)
            stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
            path = directory / f"IMG_{stamp}.jpg"
            path.write_bytes(frame["data"])
            return {"ok": True, "media_type": "photo", "file_path": str(path),
                    "captured_at": datetime.now().isoformat(timespec="seconds"),
                    "frame_age_s": round(max(0.0, time.time() - frame.get("timestamp_ms", 0) / 1000), 2)}
        except Exception as exc:
            return {"ok": False, "code": "CAPTURE_FAILED", "message": str(exc)}

    def _record_video(self, args):
        process = None
        try:
            requested = int(args.get("duration_s", 5))
            if not 1 <= requested <= self._max_duration_s:
                return {"ok": False, "code": "INVALID_DURATION", "message": f"duration_s must be between 1 and {self._max_duration_s}"}
            frame, _ = self._frame()
            directory = self._output_dir / "videos"
            directory.mkdir(parents=True, exist_ok=True)
            stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
            path = directory / f"video_{stamp}.mp4"
            process = subprocess.Popen([
                "ffmpeg", "-y", "-loglevel", "error", "-f", "mjpeg", "-r", str(self._fps), "-i", "-",
                "-an", "-c:v", "libx264", "-pix_fmt", "yuv420p", str(path),
            ], stdin=subprocess.PIPE, stderr=subprocess.PIPE)
            frames, deadline = 0, time.monotonic() + requested
            while time.monotonic() < deadline:
                # The worker continuously refreshes its newest JPEG cache.
                # Sample that cache at the requested output FPS rather than
                # failing the whole recording when one inter-process notify is
                # missed between two otherwise healthy camera frames.
                tick = time.monotonic()
                frame, _ = self._frame(timeout_s=0)
                process.stdin.write(frame["data"])
                frames += 1
                time.sleep(min(max(0.0, 1.0 / self._fps - (time.monotonic() - tick)),
                               max(0.0, deadline - time.monotonic())))
            process.stdin.close()
            stderr = process.stderr.read().decode("utf-8", "replace")
            if process.wait(timeout=10) != 0 or not path.exists():
                raise RuntimeError(stderr.strip() or "ffmpeg failed to create MP4")
            return {"ok": True, "media_type": "video", "file_path": str(path),
                    "recorded_duration_s": requested, "frames": frames,
                    "captured_at": datetime.now().isoformat(timespec="seconds")}
        except Exception as exc:
            if process is not None:
                try:
                    process.kill()
                except Exception:
                    pass
            return {"ok": False, "code": "RECORD_FAILED", "message": str(exc)}

    def dispatch(self, action, args):
        if action == "start":
            return {"state": "ready" if self._camera_ready() else "error",
                    "message": "" if self._camera_ready() else "Q5 camera worker is unavailable"}
        if action == "info":
            return self._info()
        if action == "capture_photo":
            return self._capture_photo(args)
        if action == "record_video":
            return self._record_video(args)
        if action == "stop":
            return {"state": "idle"}
        return None


def make_plugin(plugin_config, namespace, executor, client):
    return Plugin(plugin_config, namespace, executor, client)
