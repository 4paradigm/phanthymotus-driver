"""Save Go1 Nano RGB photos and videos to persistent storage."""

import json
import logging
import os
import re
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
VIDEO_FPS = 15


class VisionCapturePlugin:
    PREFIX = "vision_capture"

    def __init__(self, plugin_config, namespace=None, executor=None, client=None):
        del namespace, executor, client
        self._endpoints = {
            position: (endpoint["board_ip"], int(endpoint["image_port"]))
            for position, endpoint in camera._resolve_positions_raw(plugin_config).items()
        }
        self._output_dir = Path(plugin_config.get(
            "output_dir", "/opt/phanthy-motus/data/vision_capture"))
        self._active = set()
        self._active_paths = set()
        self._last_capture = None
        self._last_recording = None
        self._recording = None
        self._recording_thread = None
        self._encoder = None
        self._workers = set()
        self._shutting_down = False

    def get_tool(self):
        return {
            "name": self.PREFIX, "type": "actuator", "multiInstance": False,
            "description": "Capture a Go1 camera photo or record a 1-30 second MP4 video.",
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {"type": "string", "enum": ["start", "capture_photo", "record_video", "list", "delete", "info", "stop"]},
                    "position": {"type": "string", "enum": list(POSITIONS), "default": "front"},
                    "duration_s": {"type": "integer", "minimum": 1, "maximum": 30, "default": 5},
                    "image_name": {"type": "string", "description": "照片文件名，不含 .jpg；留空则自动命名。"},
                    "video_name": {"type": "string", "description": "视频文件名，不含 .mp4；留空则自动命名。"},
                    "name": {"type": "string", "description": "删除时填写完整的 .jpg 或 .mp4 文件名。"},
                },
                "required": ["action"], "additionalProperties": False,
                "x-completion": {"actions": ["capture_photo", "record_video"], "timeout": 120},
                "x-action-params": {
                    "start": {"params": [], "description": "准备拍照录像卡，无需启动 camera_rgb。"},
                    "capture_photo": {"params": ["position", "image_name"], "description": "保存指定机位的新 JPEG。"},
                    "record_video": {"params": ["position", "duration_s", "video_name"], "description": "录制指定机位的 MP4，默认 5 秒，最长 30 秒。"},
                    "list": {"params": [], "description": "列出已保存的照片和视频。"},
                    "delete": {"params": ["name"], "description": "按完整文件名删除照片或视频。"},
                    "info": {"params": [], "description": "查看照片和视频目录。"},
                    "stop": {"params": [], "description": "停用卡片并取消正在录制的视频。"},
                },
            },
        }

    def start(self):
        return {"state": "ready"}

    def stop(self):
        # 已开始的拍照继续完成；录像收到取消信号后由 worker 释放机位。
        with camera._CAMERA_LOCK:
            if self._recording:
                self._recording.set()
            worker = self._recording_thread
            encoder = self._encoder
            # 终止与编码器安装/卸载共用锁，避免终止下一次录像的进程。
            if encoder is not None and encoder.poll() is None:
                try:
                    encoder.terminate()
                except OSError:
                    pass
        if worker is not None and worker is not threading.current_thread():
            worker.join(timeout=15)
        with camera._CAMERA_LOCK:
            return {"state": "idle", "capture_active": bool(self._active)}

    def shutdown(self):
        # 关闭等待所有已受理 worker（包括已释放机位、仍在重试 ACP 的录像）。
        with camera._CAMERA_LOCK:
            self._shutting_down = True
            workers = tuple(self._workers)
        self.stop()
        for worker in workers:
            if worker is not threading.current_thread():
                worker.join()

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
            # 整帧共用 20 秒期限，避免慢速分片反复重置超时导致 MCP 调用一直等待。
            deadline = time.monotonic() + 20
            length = struct.unpack(">I", self._receive_exact(connection, 4, deadline))[0]
            if not 0 < length <= 5_000_000:
                raise OSError(f"invalid JPEG frame length: {length}")
            return self._receive_exact(connection, length, deadline)

    def _file_path(self, position, media_type, name):
        suffix = ".mp4" if media_type == "video" else ".jpg"
        directory = self._output_dir / ("videos" if media_type == "video" else "photos")
        if name is not None and name != "":
            if not isinstance(name, str) or not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_-]{0,99}", name):
                raise ValueError("name must use 1-100 letters, digits, '_' or '-', without an extension")
            return directory / f"{name}{suffix}"
        return directory / f"{position}_{datetime.now():%Y%m%d_%H%M%S_%f}_{uuid4().hex[:8]}{suffix}"

    def _channel_path(self, path):
        channel_root = Path(os.environ.get("PHANTHY_CHANNEL_OUTPUT_DIR", "/work/resource/vision_capture"))
        try:
            return str(channel_root / path.relative_to(self._output_dir))
        except ValueError:
            return str(path)

    def _list_files(self):
        files = []
        for directory, suffix, mime in ((self._output_dir / "photos", ".jpg", "image/jpeg"),
                                        (self._output_dir / "videos", ".mp4", "video/mp4")):
            if directory.exists():
                for path in directory.iterdir():
                    # 点号开头的临时文件尚未发布，不应显示为可用媒体。
                    if path.is_file() and not path.name.startswith(".") and path.suffix.lower() == suffix:
                        files.append({"filename": path.name, "path": str(path),
                                      "channel_reply_path": self._channel_path(path),
                                      "size": path.stat().st_size, "mime": mime})
        files.sort(key=lambda item: Path(item["path"]).stat().st_mtime, reverse=True)
        return {"state": "listed", "files": files}

    def _delete_file(self, name):
        if not isinstance(name, str) or not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_-]{0,99}\.(?:jpg|mp4)", name, re.I):
            return {"ok": False, "code": "INVALID_ARGUMENT", "message": "name must be a complete .jpg or .mp4 filename"}
        directory = "photos" if name.lower().endswith(".jpg") else "videos"
        path = self._output_dir / directory / name
        with camera._CAMERA_LOCK:
            if path in self._active_paths:
                return {"ok": False, "code": "RESOURCE_BUSY", "message": "media file is being written"}
        if not path.is_file():
            return {"ok": False, "code": "NOT_FOUND", "message": f"file not found: {name}"}
        path.unlink()
        return {"ok": True, "state": "deleted", "filename": name}

    def dispatch(self, action, args):
        if action == "start":
            return self.start()
        if action == "stop":
            return self.stop()
        if action == "list":
            return self._list_files()
        if action == "delete":
            return self._delete_file(args.get("name"))
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
        try:
            path = self._file_path(position, "video" if action != "capture_photo" else "photo",
                                   args.get("image_name" if action == "capture_photo" else "video_name"))
        except ValueError as exc:
            return {"ok": False, "code": "INVALID_ARGUMENT", "message": str(exc)}
        with camera._CAMERA_LOCK:
            if self._shutting_down:
                return {"ok": False, "code": "SHUTTING_DOWN", "message": "vision_capture is shutting down"}
            if action != "capture_photo" and self._recording is not None:
                return {"ok": False, "code": "RESOURCE_BUSY", "message": "a video recording is already active"}
            if position in camera._SNAPSHOT_POSITIONS or camera.running_stream(position) is not None:
                return {"ok": False, "code": "RESOURCE_BUSY",
                        "message": f"camera position {position!r} is occupied; stop its stream first"}
            if path.exists() or path in self._active_paths:
                return {"ok": False, "code": "FILE_EXISTS", "message": f"file already exists: {path.name}"}
            camera._SNAPSHOT_POSITIONS.add(position)
            self._active.add(position)
            self._active_paths.add(path)
            cancel = threading.Event() if action != "capture_photo" else None
            if cancel is not None:
                self._recording = cancel
        action_id = f"vision_capture_{uuid4().hex}"
        # 初始 MCP result 仅表示受理；文件可用性以 ACP 终态为准。
        worker = None
        try:
            target = self._record_async if cancel else self._capture_async
            worker_args = (position, action_id, path, duration, cancel) if cancel else (position, action_id, path)
            worker = threading.Thread(target=target, args=worker_args, daemon=True)
            with camera._CAMERA_LOCK:
                if self._shutting_down:
                    raise RuntimeError("vision_capture is shutting down")
                if cancel is not None:
                    self._recording_thread = worker
                self._workers.add(worker)
                worker.start()
        except Exception as exc:
            with camera._CAMERA_LOCK:
                camera._SNAPSHOT_POSITIONS.discard(position)
                self._active.discard(position)
                self._active_paths.discard(path)
                if cancel is not None:
                    self._recording = None
                    self._recording_thread = None
                if worker is not None:
                    self._workers.discard(worker)
            return {"ok": False, "code": "RECORD_FAILED" if cancel else "CAPTURE_FAILED", "message": str(exc)}
        return {"ok": True, "state": "recording" if cancel else "capturing",
                "action_id": action_id, "position": position,
                "file_path": str(path), "filename": path.name,
                "channel_reply_path": self._channel_path(path), "file_ready": False}

    def _capture_async(self, position, action_id, path):
        try:
            result = self._capture_and_save(position, path)
        except Exception as exc:
            result = {"ok": False, "code": "CAPTURE_FAILED", "message": str(exc)}
        finally:
            with camera._CAMERA_LOCK:
                camera._SNAPSHOT_POSITIONS.discard(position)
                self._active.discard(position)
                self._active_paths.discard(path)
        status = "completed" if result.get("ok") else "error"
        with camera._CAMERA_LOCK:
            self._last_capture = {"action_id": action_id, "status": status, "result": result}
        try:
            self._notify_complete(action_id, status, result)
        finally:
            with camera._CAMERA_LOCK:
                self._workers.discard(threading.current_thread())

    def _notify_complete(self, action_id, status, result):
        # 同一终态重试同一 payload；不发送 /api/event，也不读取 ACCESS_TOKEN。
        payload = json.dumps({"action_id": action_id, "status": status, "result": result,
                              "tool": self.PREFIX, "ts": time.time()}).encode()
        url = os.environ.get("AGENT_CORE_URL", "https://localhost:15678").rstrip("/")
        ctx = ssl.create_default_context()
        if url.startswith(("https://localhost:", "https://127.0.0.1:")):
            ctx.check_hostname = False
            ctx.verify_mode = ssl.CERT_NONE
        # 最坏 3×3 秒请求 + 0.5/1 秒退避；无持久化重试队列。
        for attempt in range(3):
            try:
                request = urllib.request.Request(url + "/api/acp/complete", data=payload,
                                                 headers={"Content-Type": "application/json"}, method="POST")
                with urllib.request.urlopen(request, timeout=3, context=ctx):
                    pass
                return
            except Exception as exc:
                if attempt == 2:
                    logging.getLogger(__name__).warning(
                        "ACP completion delivery failed for %s: %s", action_id, exc)
                else:
                    time.sleep(0.5 * 2 ** attempt)

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
                    "file_path": str(path), "filename": path.name,
                    "channel_reply_path": self._channel_path(path), "mime": "image/jpeg", "size": len(jpeg),
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
                self._active_paths.discard(path)
                if self._recording is cancel:
                    self._recording = None
        status = ("completed" if result.get("ok") else
                  "cancelled" if result.get("code") == "RECORD_CANCELLED" else "error")
        with camera._CAMERA_LOCK:
            self._last_recording = {"action_id": action_id, "status": status, "result": result}
        try:
            self._notify_complete(action_id, status, result)
        finally:
            with camera._CAMERA_LOCK:
                self._workers.discard(threading.current_thread())
                if self._recording_thread is threading.current_thread():
                    self._recording_thread = None

    def _record_and_save(self, position, path, duration, cancel):
        temporary_path = path.with_name(f".{path.stem}.tmp.mp4")
        frames_path = path.with_name(f".{path.stem}.tmp.mjpeg")
        frames_output = None
        process = None
        stderr_thread = None
        stderr_tail = bytearray()
        published = False
        try:
            host, port = self._endpoints[position]
            with socket.create_connection((host, port), timeout=8) as connection:
                path.parent.mkdir(parents=True, exist_ok=True)
                # 先把选中的 JPEG 落到临时流，避免慢编码器阻塞 Nano 收帧。
                frames_output = frames_path.open("wb")
                frames = 0
                source_frames = 0
                total_frames = duration * VIDEO_FPS
                deadline = None
                started_at = None
                recording_started_at = None
                last_jpeg = None
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
                        started_at = now
                        recording_started_at = datetime.now().astimezone()
                        deadline = now + duration
                    source_frames += 1
                    slot = int((now - started_at) * VIDEO_FPS)
                    slot = min(total_frames - 1, slot)
                    # 源帧率低于 15 fps 时填补缺口，保持 MP4 播放时长与实际录制时长一致。
                    while frames < slot:
                        frames_output.write(last_jpeg)
                        frames += 1
                    if frames <= slot:
                        frames_output.write(jpeg)
                        frames += 1
                    last_jpeg = jpeg
                if cancel.is_set():
                    raise InterruptedError("video recording cancelled")
                if last_jpeg is None:
                    raise RuntimeError("no camera frames received")
                recording_ended_at = datetime.now().astimezone()
                while frames < total_frames:
                    frames_output.write(last_jpeg)
                    frames += 1
                frames_output.close()
                frames_output = None
                connection.close()
                with frames_path.open("rb") as frames_input:
                    process = subprocess.Popen([
                        "ffmpeg", "-y", "-loglevel", "error", "-f", "mjpeg", "-r", str(VIDEO_FPS), "-i", "pipe:0",
                        "-an", "-c:v", "libx264", "-preset", "ultrafast", "-pix_fmt", "yuv420p",
                        "-f", "mp4", str(temporary_path),
                    ], stdin=frames_input, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
                    with camera._CAMERA_LOCK:
                        self._encoder = process
                        # stop 可能在 Popen 返回前已取消；此时直接进入 finally 清理子进程。
                        if cancel.is_set():
                            raise InterruptedError("video recording cancelled")

                    def drain_stderr():
                        while chunk := process.stderr.read(4096):
                            stderr_tail.extend(chunk)
                            del stderr_tail[:-16384]

                    stderr_thread = threading.Thread(target=drain_stderr, daemon=True)
                    stderr_thread.start()
                    process.wait(timeout=30)
                stderr_thread.join(timeout=2)
                if stderr_thread.is_alive():
                    raise RuntimeError("ffmpeg error output did not close")
                if process.returncode != 0 or not temporary_path.is_file() or temporary_path.stat().st_size == 0:
                    raise RuntimeError(stderr_tail.decode("utf-8", "replace") or "ffmpeg failed")
                # 录像结束后才公开 MP4，避免画布看到尚未写好索引的文件。
                with temporary_path.open("rb+") as output:
                    os.fsync(output.fileno())
                with camera._CAMERA_LOCK:
                    # 与 stop 的取消信号互斥，避免检查后仍发布已取消的录像。
                    if cancel.is_set():
                        raise InterruptedError("video recording cancelled")
                    temporary_path.replace(path)
                    published = True
                    # 发布是成功的提交点；之后 stop 不再把已保存录像标为取消。
                    if self._recording is cancel:
                        self._recording = None
                file_ready_at = datetime.now().astimezone()
                return {"ok": True, "position": position, "media_type": "video",
                        "file_path": str(path), "filename": path.name,
                        "channel_reply_path": self._channel_path(path), "mime": "video/mp4",
                        "size": path.stat().st_size, "recorded_duration_s": frames / VIDEO_FPS, "frames": frames,
                        "source_frames": source_frames,
                        "recording_started_at": recording_started_at.isoformat(timespec="milliseconds"),
                        "recording_ended_at": recording_ended_at.isoformat(timespec="milliseconds"),
                        "file_ready_at": file_ready_at.isoformat(timespec="milliseconds"),
                        "captured_at": file_ready_at.isoformat(timespec="seconds")}
        except InterruptedError:
            return {"ok": False, "code": "RECORD_CANCELLED", "message": "video recording was cancelled"}
        except Exception as exc:
            if cancel.is_set():
                return {"ok": False, "code": "RECORD_CANCELLED", "message": "video recording was cancelled"}
            return {"ok": False, "code": "RECORD_FAILED", "message": str(exc)}
        finally:
            if frames_output is not None:
                frames_output.close()
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
            frames_path.unlink(missing_ok=True)


def make_vision_capture(plugin_config, namespace, executor, client):
    return VisionCapturePlugin(plugin_config, namespace, executor, client)
