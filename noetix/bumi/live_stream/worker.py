"""Feed Bumi's existing JPEG and PCM sources to a single Linux FFmpeg process.

The worker never opens a second RealSense or MediaController capture.
"""
from __future__ import annotations

import os
import queue
import select
import subprocess
import threading
import time

from .transport import FLVFramer, RTMPPublisher, RTMPTransportError


class AudioBuffer:
    FORMATS = frozenset(("pcm_16k_16bit_mono", "audio/pcm-16k"))

    def __init__(self, capacity: int = 96):
        self.queue: queue.Queue[bytes] = queue.Queue(maxsize=capacity)
        self.last_received = 0.0
        self.dropped = 0
        self._lock = threading.Lock()

    def push(self, data: bytes, fmt: str) -> bool:
        if fmt not in self.FORMATS or not data or len(data) % 2 or len(data) > 131072:
            return False
        with self._lock:
            self.last_received = time.monotonic()
            if self.queue.full():
                try:
                    self.queue.get_nowait()
                except queue.Empty:
                    pass
                self.dropped += 1
            self.queue.put_nowait(bytes(data))
        return True

    def fresh(self, max_age: float = 4.0) -> bool:
        return self.last_received > 0 and time.monotonic() - self.last_received < max_age

    def clear(self) -> None:
        while True:
            try:
                self.queue.get_nowait()
            except queue.Empty:
                break


ERROR_MESSAGES = {
    "ENCODER_START_FAILED": "直播编码器启动失败，请检查机器人 FFmpeg 环境",
    "RTMP_PROCESS_EXITED": "推流异常结束，请刷新虎牙地址并检查机器人网络",
    "VIDEO_PIPE_FAILED": "实时画面中断，请检查 Bumi 摄像头",
    "AUDIO_PIPE_FAILED": "实时声音中断，请检查 Bumi 麦克风",
    "LIBRTMP_UNAVAILABLE": "机器人缺少 librtmp1 运行库",
    "RTMP_CONNECT_FAILED": "虎牙服务器连接失败，请检查网络和推流地址",
    "RTMP_PUBLISH_REJECTED": "虎牙拒绝推流，请更新推流地址",
    "RTMP_WRITE_FAILED": "向虎牙上传直播数据失败",
}


class LiveStreamWorker:
    """Thread-safe explicit start/stop; never auto-live on driver startup."""

    def __init__(self, camera, audio: AudioBuffer, fps: int = 15,
                 bitrate_kbps: int = 1200, popen=subprocess.Popen,
                 publisher_factory=RTMPPublisher):
        self.camera = camera
        self.audio = audio
        self.fps = max(1, min(15, int(fps)))
        self.bitrate_kbps = max(300, min(2400, int(bitrate_kbps)))
        self._popen = popen
        self._publisher_factory = publisher_factory
        self._lock = threading.RLock()
        self._cancel = threading.Event()
        self._thread = None
        self._process = None
        self._status = "idle"
        self._error = ""
        self._start_time = 0.0
        self._frames = 0
        self._chunks = 0
        self._published_records = 0

    def info(self) -> dict:
        with self._lock:
            return {
                "state": self._status,
                "error_code": self._error or None,
                "message": ERROR_MESSAGES.get(self._error, ""),
                "duration_s": round(time.monotonic() - self._start_time, 1)
                if self._start_time and self._status in ("starting", "streaming") else 0,
                "video_frames": self._frames,
                "audio_chunks": self._chunks,
                "audio_dropped": self.audio.dropped,
                "published_records": self._published_records,
            }

    def start(self, url: str) -> dict:
        with self._lock:
            if self._thread is not None and self._thread.is_alive():
                return self.info()
            if self.camera is None or not self.camera.is_running():
                return {"state": "error", "error_code": "CAMERA_UNAVAILABLE",
                        "message": "摄像头未就绪，请检查 Camera Card"}
            if not self.audio.fresh():
                return {"state": "error", "error_code": "MIC_UNAVAILABLE",
                        "message": "麦克风没有实时音频，请检查 Mic Card"}
            self.audio.clear()
            self._cancel = threading.Event()
            self._error = ""
            self._frames = self._chunks = 0
            self._published_records = 0
            self._start_time = time.monotonic()
            self._status = "starting"
            self._thread = threading.Thread(
                target=self._run, args=(url, self._cancel),
                name="bumi-huya-live", daemon=True)
            self._thread.start()
            return {"state": "starting", "message": "正在建立虎牙推流连接"}

    def stop(self) -> dict:
        with self._lock:
            self._cancel.set()
            proc, thread = self._process, self._thread
        if proc is not None and proc.poll() is None:
            proc.terminate()
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=7)
        with self._lock:
            if thread is not None and thread.is_alive():
                self._status = "stopping"
                return {"state": "stopping", "message": "正在结束编码进程"}
            self._process = self._thread = None
            self._status = "idle"
            self._start_time = 0
            self._error = ""
            return {"state": "idle", "message": "直播已停止"}

    @staticmethod
    def _write(fd: int, payload: bytes, cancel: threading.Event) -> None:
        pending = memoryview(payload)
        deadline = time.monotonic() + 3
        while pending:
            if cancel.is_set():
                return
            if time.monotonic() > deadline:
                raise TimeoutError("ffmpeg pipe timed out")
            if not select.select([], [fd], [], 0.15)[1]:
                continue
            try:
                pending = pending[os.write(fd, pending):]
            except BlockingIOError:
                continue

    def _run(self, url: str, cancel: threading.Event) -> None:
        proc = None
        audio_fd = None
        writers = []
        failure: queue.Queue[str] = queue.Queue()
        error_code = ""
        publisher = None
        try:
            # RTMP credentials are only passed to the in-process library.
            publisher = self._publisher_factory(url)
            publisher.connect()
            audio_r, audio_fd = os.pipe()
            cmd = [
                "ffmpeg", "-hide_banner", "-nostdin", "-loglevel", "error",
                "-thread_queue_size", "64", "-f", "mjpeg",
                "-framerate", str(self.fps), "-i", "pipe:0",
                "-thread_queue_size", "128", "-f", "s16le",
                "-ar", "16000", "-ac", "1", "-i", f"pipe:{audio_r}",
                "-map", "0:v:0", "-map", "1:a:0",
                "-c:v", "libx264", "-preset", "veryfast", "-tune", "zerolatency",
                "-pix_fmt", "yuv420p", "-b:v", f"{self.bitrate_kbps}k",
                "-g", str(self.fps * 2),
                "-c:a", "aac", "-ar", "44100", "-b:a", "96k",
                "-f", "flv", "pipe:1",
            ]
            try:
                proc = self._popen(cmd, stdin=subprocess.PIPE,
                                   stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
                                   pass_fds=(audio_r,), bufsize=0)
            finally:
                os.close(audio_r)
            os.set_blocking(proc.stdin.fileno(), False)
            os.set_blocking(audio_fd, False)
            os.set_blocking(proc.stdout.fileno(), False)
            with self._lock:
                self._process = proc

            def video() -> None:
                sequence = None
                next_tick = time.monotonic()
                try:
                    while not cancel.is_set():
                        if cancel.wait(max(0, next_tick - time.monotonic())):
                            return
                        frame, sequence = self.camera.wait_for_color_frame(
                            after_sequence=sequence, timeout_s=1.5)
                        if not frame or not frame.get("data"):
                            raise TimeoutError
                        self._write(proc.stdin.fileno(), frame["data"], cancel)
                        with self._lock:
                            self._frames += 1
                        next_tick = max(next_tick + 1 / self.fps, time.monotonic())
                except (OSError, TimeoutError, RuntimeError, ValueError):
                    failure.put("VIDEO_PIPE_FAILED")
                    cancel.set()

            def audio() -> None:
                try:
                    while not cancel.is_set():
                        if not self.audio.fresh(max_age=8):
                            raise TimeoutError
                        try:
                            data = self.audio.queue.get(timeout=0.25)
                        except queue.Empty:
                            continue
                        self._write(audio_fd, data, cancel)
                        with self._lock:
                            self._chunks += 1
                except (OSError, TimeoutError, RuntimeError, ValueError):
                    failure.put("AUDIO_PIPE_FAILED")
                    cancel.set()

            for name, target in (("huya-video", video), ("huya-audio", audio)):
                thread = threading.Thread(target=target, name=name, daemon=True)
                writers.append(thread)
                thread.start()

            framer = FLVFramer()
            while not cancel.is_set():
                if select.select([proc.stdout.fileno()], [], [], 0.2)[0]:
                    payload = os.read(proc.stdout.fileno(), 65536)
                    if payload:
                        for record in framer.feed(payload):
                            publisher.send(record)
                            with self._lock:
                                self._published_records += 1
                    elif proc.poll() is not None:
                        error_code = "RTMP_PROCESS_EXITED"
                        break
                elif proc.poll() is not None:
                    error_code = "RTMP_PROCESS_EXITED"
                    break
                with self._lock:
                    if (self._status == "starting" and self._frames
                            and self._chunks and self._published_records):
                        self._status = "streaming"
            if not failure.empty():
                error_code = failure.get_nowait()
        except RTMPTransportError as exc:
            error_code = str(exc)
        except (OSError, RuntimeError, ValueError):
            error_code = "ENCODER_START_FAILED"
        finally:
            cancel.set()
            if proc is not None:
                if proc.poll() is None:
                    proc.terminate()
                    try:
                        proc.wait(timeout=3)
                    except subprocess.TimeoutExpired:
                        proc.kill()
                        proc.wait(timeout=2)
                if proc.stdin:
                    proc.stdin.close()
                if proc.stdout:
                    proc.stdout.close()
            if audio_fd is not None:
                os.close(audio_fd)
            for writer in writers:
                if writer.is_alive():
                    writer.join(timeout=2)
            if not failure.empty():
                error_code = failure.get_nowait()
            if publisher is not None:
                try:
                    publisher.close()
                except OSError:
                    pass
            with self._lock:
                self._process = None
                if self._status != "idle":
                    self._status = "error" if error_code else "idle"
                    self._error = error_code
