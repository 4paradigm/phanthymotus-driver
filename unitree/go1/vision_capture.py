"""Save Go1 Nano RGB photos and videos to persistent storage."""

import json
import logging
import os
import shutil
import socket
import ssl
import struct
import subprocess
import threading
import time
import urllib.request
from datetime import datetime
from pathlib import Path
from uuid import uuid4


if __package__:
    from . import camera
else:
    import camera


POSITIONS = tuple(camera._VALID_POSITIONS)
log = logging.getLogger(__name__)


class VisionCapturePlugin:
    def __init__(self, plugin_config, namespace=None, executor=None, client=None):
        del namespace, executor, client
        self._endpoints = {
            position: (endpoint["board_ip"], int(endpoint["image_port"]))
            for position, endpoint in camera._resolve_positions_raw(plugin_config).items()
        }
        self._output_dir = Path(plugin_config.get(
            "output_dir", "/opt/phanthy-motus/data/vision_capture"))
        self._active = set()
        self._last_capture = None
        self._last_recording = None
        self._recording = None
        self._recording_thread = None
        self._encoder = None

    def get_tool(self):
        return {
            "name": "vision_capture", "type": "actuator", "multiInstance": False,
            "description": "Capture a Go1 camera photo or record a 1-30 second MP4 video.",
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {"type": "string", "enum": ["start", "capture_photo", "record_video", "info", "stop"]},
                    "position": {"type": "string", "enum": list(POSITIONS), "default": "front"},
                    "duration_s": {"type": "integer", "minimum": 1, "maximum": 30, "default": 5},
                },
                "required": ["action"], "additionalProperties": False,
                "x-completion": {"actions": ["capture_photo", "record_video"], "timeout": 120},
                "x-resource": "camera",
                "x-action-params": {
                    "start": {"params": [], "description": "准备拍照录像卡，无需启动 camera_rgb。"},
                    "capture_photo": {"params": ["position"], "description": "保存指定机位的新 JPEG。"},
                    "record_video": {"params": ["position", "duration_s"], "description": "录制指定机位的 MP4，默认 5 秒，最长 30 秒。"},
                    "info": {"params": [], "description": "查看照片和视频目录。"},
                    "stop": {"params": [], "description": "停用卡片并取消正在录制的视频。"},
                },
            },
        }

    def start(self):
        return {"state": "ready"}

    def stop(self):
        # 已受理的抓拍继续完成；录像收到取消信号后由 worker 释放机位并上报 ACP。
        with camera._CAMERA_LOCK:
            if self._recording:
                self._recording.set()
            worker = self._recording_thread
            encoder = self._encoder
        if encoder is not None and encoder.poll() is None:
            try:
                encoder.terminate()
            except OSError:
                pass
        if worker is not None and worker is not threading.current_thread():
            worker.join(timeout=15)
        with camera._CAMERA_LOCK:
            return {"state": "idle", "capture_active": bool(self._active)}

    @staticmethod
    def _receive_exact(connection, size, deadline, cancel=None):
        chunks = bytearray()
        while len(chunks) < size:
            remaining = deadline - time.monotonic()
            if cancel is not None and cancel.is_set():
                raise InterruptedError("video recording cancelled")
            if remaining <= 0:
                raise TimeoutError("camera frame deadline exceeded")
            connection.settimeout(min(remaining, 0.5) if cancel is not None else remaining)
            try:
                chunk = connection.recv(size - len(chunks))
            except socket.timeout:
                if cancel is None:
                    raise
                continue
            if not chunk:
                raise OSError("camera stream closed before a complete JPEG arrived")
            chunks.extend(chunk)
        return bytes(chunks)

    def _capture_jpeg(self, position):
        host, port = self._endpoints[position]
        # 中文说明：连接会让 Nano 按需开启相机；读完一帧即断开并释放机位。
        with socket.create_connection((host, port), timeout=8) as connection:
            # 整帧共用 20 秒期限，避免慢速分片反复重置超时导致 ACP 永不完成。
            deadline = time.monotonic() + 20
            length = struct.unpack(">I", self._receive_exact(connection, 4, deadline))[0]
            if not 0 < length <= 5_000_000:
                raise OSError(f"invalid JPEG frame length: {length}")
            return self._receive_exact(connection, length, deadline)

    def dispatch(self, action, args):
        if action == "start":
            return self.start()
        if action == "stop":
            return self.stop()
        if action == "info":
            with camera._CAMERA_LOCK:
                return {"state": "recording" if self._recording else "capturing" if self._active else "ready",
                        "output_dir": str(self._output_dir),
                        "photos_dir": str(self._output_dir / "photos"),
                        "videos_dir": str(self._output_dir / "videos"),
                        "positions": list(POSITIONS), "last_capture": self._last_capture,
                        "last_recording": self._last_recording}
        if action not in ("capture_photo", "record_video"):
            return None

        position = args.get("position", "front")
        if position not in POSITIONS:
            return {"ok": False, "code": "INVALID_ARGUMENT", "message": "unknown camera position"}
        duration = args.get("duration_s", 5)
        if action == "record_video":
            if isinstance(duration, bool) or not isinstance(duration, int) or not 1 <= duration <= 30:
                return {"ok": False, "code": "INVALID_ARGUMENT", "message": "duration_s must be 1-30 seconds"}
            if not shutil.which("ffmpeg"):
                return {"ok": False, "code": "ENCODER_UNAVAILABLE", "message": "ffmpeg is required for MP4 recording"}
        with camera._CAMERA_LOCK:
            if action == "record_video" and self._recording is not None:
                return {"ok": False, "code": "RESOURCE_BUSY", "message": "a video recording is already active"}
            if position in camera._SNAPSHOT_POSITIONS or camera.running_stream(position) is not None:
                return {"ok": False, "code": "RESOURCE_BUSY",
                        "message": f"camera position {position!r} is occupied; stop its stream first"}
            camera._SNAPSHOT_POSITIONS.add(position)
            self._active.add(position)
            cancel = threading.Event() if action == "record_video" else None
            if cancel is not None:
                self._recording = cancel
        action_id = f"vision_capture_{uuid4().hex}"
        # 先确定文件名，画布收到受理结果时即可显示目标路径；完成回调才确认文件存在。
        directory = self._output_dir / ("videos" if cancel else "photos")
        suffix = ".mp4" if cancel else ".jpg"
        path = directory / f"{position}_{datetime.now():%Y%m%d_%H%M%S_%f}_{uuid4().hex[:8]}{suffix}"
        try:
            target = self._record_async if cancel else self._capture_async
            worker_args = (position, action_id, path, duration, cancel) if cancel else (position, action_id, path)
            worker = threading.Thread(target=target, args=worker_args, daemon=True)
            if cancel is not None:
                with camera._CAMERA_LOCK:
                    self._recording_thread = worker
            worker.start()
        except Exception as exc:
            with camera._CAMERA_LOCK:
                camera._SNAPSHOT_POSITIONS.discard(position)
                self._active.discard(position)
                if cancel is not None:
                    self._recording = None
                    self._recording_thread = None
            return {"ok": False, "code": "RECORD_FAILED" if cancel else "CAPTURE_FAILED", "message": str(exc)}
        return {"ok": True, "state": "recording" if cancel else "capturing", "action_id": action_id,
                "position": position, "file_path": str(path)}

    def _notify_complete(self, action_id, status, result):
        # ACP/TLS 通知仅用标准库；视频编码依赖 Dockerfile 安装的 ffmpeg。
        payload = json.dumps({"action_id": action_id, "status": status, "result": result,
                              "tool": "vision_capture", "ts": time.time()}).encode()
        url = os.environ.get("AGENT_CORE_URL", "https://localhost:15678").rstrip("/")
        # 最坏 3×3 秒请求 + 0.5/1 秒退避，留在 120 秒 ACP 超时预算内。
        for attempt in range(3):
            try:
                ctx = ssl.create_default_context()
                if url.startswith(("https://localhost:", "https://127.0.0.1:")):
                    ctx.check_hostname = False
                    ctx.verify_mode = ssl.CERT_NONE
                request = urllib.request.Request(url + "/api/acp/complete", data=payload,
                                                 headers={"Content-Type": "application/json"}, method="POST")
                with urllib.request.urlopen(request, timeout=3, context=ctx):
                    pass
                return
            except Exception as exc:
                if attempt == 2:
                    log.warning("[vision_capture] ACP completion delivery failed for %s: %s", action_id, exc)
                else:
                    time.sleep(0.5 * 2 ** attempt)

    def _capture_async(self, position, action_id, path):
        try:
            result = self._capture_and_save(position, path)
        except Exception as exc:
            result = {"ok": False, "code": "CAPTURE_FAILED", "message": str(exc)}
        finally:
            with camera._CAMERA_LOCK:
                camera._SNAPSHOT_POSITIONS.discard(position)
                self._active.discard(position)
        status = "completed" if result.get("ok") else "error"
        with camera._CAMERA_LOCK:
            self._last_capture = {"action_id": action_id, "status": status, "result": result}
        self._notify_complete(action_id, status, result)

    def _capture_and_save(self, position, path):
        try:
            jpeg = self._capture_jpeg(position)
        except OSError as exc:
            return {"ok": False, "code": "CAMERA_UNAVAILABLE",
                    "message": f"cannot capture from {position}: {exc}"}
        if not jpeg or not jpeg.startswith(b"\xff\xd8") or not jpeg.endswith(b"\xff\xd9"):
            return {"ok": False, "code": "CAMERA_UNAVAILABLE",
                    "message": f"invalid JPEG from {position}"}

        temporary_path = None
        try:
            path.parent.mkdir(parents=True, exist_ok=True)
            temporary_path = path.with_name(f".{path.name}.tmp")
            # 中文说明：先写入并同步临时文件，再公开 JPEG 路径，避免重启后留下空照片。
            with temporary_path.open("xb") as output:
                output.write(jpeg)
                output.flush()
                os.fsync(output.fileno())
            temporary_path.replace(path)
            return {"ok": True, "position": position, "media_type": "photo",
                    "file_path": str(path),
                    "captured_at": datetime.now().astimezone().isoformat(timespec="seconds")}
        except OSError as exc:
            if temporary_path is not None:
                temporary_path.unlink(missing_ok=True)
            return {"ok": False, "code": "SAVE_FAILED", "message": str(exc)}

    def _record_async(self, position, action_id, path, duration, cancel):
        try:
            result = self._record_and_save(position, path, duration, cancel)
        except Exception as exc:
            result = {"ok": False, "code": "RECORD_FAILED", "message": str(exc)}
        finally:
            with camera._CAMERA_LOCK:
                camera._SNAPSHOT_POSITIONS.discard(position)
                self._active.discard(position)
                if self._recording is cancel:
                    self._recording = None
        status = "completed" if result.get("ok") else "error"
        with camera._CAMERA_LOCK:
            self._last_recording = {"action_id": action_id, "status": status, "result": result}
        try:
            self._notify_complete(action_id, status, result)
        finally:
            with camera._CAMERA_LOCK:
                if self._recording_thread is threading.current_thread():
                    self._recording_thread = None

    def _record_and_save(self, position, path, duration, cancel):
        temporary_path = path.with_name(f".{path.stem}.tmp.mp4")
        process = None
        stderr_thread = None
        stderr_tail = bytearray()
        published = False
        try:
            host, port = self._endpoints[position]
            with socket.create_connection((host, port), timeout=8) as connection:
                path.parent.mkdir(parents=True, exist_ok=True)
                process = subprocess.Popen([
                    "ffmpeg", "-y", "-loglevel", "error", "-f", "mjpeg", "-r", "15", "-i", "pipe:0",
                    "-an", "-c:v", "libx264", "-pix_fmt", "yuv420p", "-f", "mp4", str(temporary_path),
                ], stdin=subprocess.PIPE, stderr=subprocess.PIPE)
                with camera._CAMERA_LOCK:
                    self._encoder = process

                def drain_stderr():
                    while chunk := process.stderr.read(4096):
                        stderr_tail.extend(chunk)
                        del stderr_tail[:-16384]

                stderr_thread = threading.Thread(target=drain_stderr, daemon=True)
                stderr_thread.start()
                frames = 0
                deadline = None
                next_frame_at = 0
                while not cancel.is_set():
                    frame_deadline = min(deadline, time.monotonic() + 2) if deadline else time.monotonic() + 20
                    if deadline and time.monotonic() >= deadline:
                        break
                    try:
                        length = struct.unpack(">I", self._receive_exact(connection, 4, frame_deadline, cancel))[0]
                    except TimeoutError:
                        if deadline and time.monotonic() >= deadline:
                            break
                        raise
                    if not 0 < length <= 5_000_000:
                        raise OSError(f"invalid JPEG frame length: {length}")
                    try:
                        jpeg = self._receive_exact(connection, length, frame_deadline, cancel)
                    except TimeoutError:
                        if deadline and time.monotonic() >= deadline:
                            break
                        raise
                    if not jpeg.startswith(b"\xff\xd8") or not jpeg.endswith(b"\xff\xd9"):
                        raise OSError("invalid JPEG frame")
                    now = time.monotonic()
                    if deadline is None:
                        deadline = now + duration
                    # Nano RGB 可达 30-60 fps；限到编码器的 15 fps，避免视频播放时长膨胀。
                    if now >= next_frame_at:
                        process.stdin.write(jpeg)
                        frames += 1
                        next_frame_at = now + 1 / 15
                if cancel.is_set():
                    raise InterruptedError("video recording cancelled")
                process.stdin.close()
                process.wait(timeout=30)
                stderr_thread.join(timeout=2)
                if stderr_thread.is_alive():
                    raise RuntimeError("ffmpeg error output did not close")
                if process.returncode != 0 or not temporary_path.is_file() or temporary_path.stat().st_size == 0:
                    raise RuntimeError(stderr_tail.decode("utf-8", "replace") or "ffmpeg failed")
                if cancel.is_set():
                    raise InterruptedError("video recording cancelled")
                # 录像结束后才公开 MP4，避免画布看到尚未写好索引的文件。
                with temporary_path.open("rb+") as output:
                    os.fsync(output.fileno())
                temporary_path.replace(path)
                published = True
                return {"ok": True, "position": position, "media_type": "video",
                        "file_path": str(path), "recorded_duration_s": duration, "frames": frames,
                        "captured_at": datetime.now().astimezone().isoformat(timespec="seconds")}
        except InterruptedError:
            return {"ok": False, "code": "RECORD_CANCELLED", "message": "video recording was cancelled"}
        except Exception as exc:
            if cancel.is_set():
                return {"ok": False, "code": "RECORD_CANCELLED", "message": "video recording was cancelled"}
            return {"ok": False, "code": "RECORD_FAILED", "message": str(exc)}
        finally:
            with camera._CAMERA_LOCK:
                if self._encoder is process:
                    self._encoder = None
            if process is not None:
                if process.stdin and not process.stdin.closed:
                    try:
                        process.stdin.close()
                    except OSError:
                        pass
                if process.poll() is None:
                    process.terminate()
                    try:
                        process.wait(timeout=2)
                    except subprocess.TimeoutExpired:
                        process.kill()
                        process.wait(timeout=2)
                if stderr_thread is not None:
                    stderr_thread.join(timeout=2)
                process.stderr.close()
            if not published:
                temporary_path.unlink(missing_ok=True)


def make_vision_capture(plugin_config, namespace, executor, client):
    return VisionCapturePlugin(plugin_config, namespace, executor, client)
