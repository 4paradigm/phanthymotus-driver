"""Bounded, on-demand V4L2 -> JPEG capture using standard GStreamer plugins.

No vendor control script, GPIO trigger, camera reset, or robot command is run.
The five colour devices are identified by their kernel names, not video indices.
"""

from __future__ import annotations

import os
import selectors
import shutil
import struct
import subprocess
import threading
import time
from pathlib import Path

MAX_JPEG_BYTES = 2 * 1024 * 1024

SOURCES = {
    "chest_rgb": {"label": "胸部 RGB（G300）", "device_name": "vi-output, G300 color 9-006f",
                "format": "YUY2", "width": 1280, "height": 800, "fps": 30},
    "head_rgb": {"label": "头部 RGB（G300）", "device_name": "vi-output, G300 color 9-0067",
                "format": "YUY2", "width": 1280, "height": 800, "fps": 30},
    "head_front_fisheye": {"label": "头前鱼眼（SHW3H）", "device_name": "vi-output, shw3h 10-0020",
                "format": "UYVY", "width": 1920, "height": 1536, "fps": 60},
    "head_rear_left_fisheye": {"label": "头后左鱼眼（SHW3H）", "device_name": "vi-output, shw3h 10-0021",
                "format": "UYVY", "width": 1920, "height": 1536, "fps": 60},
    "head_rear_right_fisheye": {"label": "头后右鱼眼（SHW3H）", "device_name": "vi-output, shw3h 10-0022",
                "format": "UYVY", "width": 1920, "height": 1536, "fps": 60},
}


def resolve_device(source, sysfs=Path("/sys/class/video4linux"), dev=Path("/dev")):
    """Never accidentally open a depth, infrared or metadata node after reboot."""
    expected = source["device_name"]
    matches = []
    for entry in sysfs.glob("video[0-9]*"):
        try:
            if entry.joinpath("name").read_text().strip() == expected and dev.joinpath(entry.name).exists():
                matches.append(str(dev / entry.name))
        except OSError:
            continue
    if len(matches) != 1:
        raise ValueError(f"Expected exactly one colour device named {expected!r}; found {len(matches)}")
    return matches[0]


CONVERT_THREADS = 4


OUTPUT_WIDTH = 640


def output_size(source):
    """Published JPEG size: fixed width, native aspect ratio (known before any frame)."""
    return OUTPUT_WIDTH, OUTPUT_WIDTH * source["height"] // source["width"]


def capture_command(source, device, fps):
    """Drop frames before scaling/conversion; preserve the native aspect ratio."""
    width, height = output_size(source)
    return [
        "gst-launch-1.0", "-q", "v4l2src", f"device={device}", "io-mode=mmap", "!",
        f"video/x-raw,format={source['format']},width={source['width']},"
        f"height={source['height']},framerate={source['fps']}/1", "!",
        "queue", "max-size-buffers=1", "max-size-bytes=0", "max-size-time=0", "leaky=downstream", "!",
        "videorate", "drop-only=true", "!", f"video/x-raw,framerate={fps}/1", "!",
        # Single-threaded scaling of the 1920x1536 fisheye frames topped out near 9 fps on
        # the Orin; GStreamer 1.20's n-threads reaches the 15 fps target (measured on site).
        "videoscale", f"n-threads={CONVERT_THREADS}", "!", f"video/x-raw,width={width},height={height}", "!",
        "videoconvert", f"n-threads={CONVERT_THREADS}", "!", "jpegenc", "quality=80", "!",
        "fdsink", "fd=1", "sync=false",
    ]


def jpeg_dimensions(data):
    """Read JPEG SOF; reject non-colour, truncated, oversized and bogus images."""
    if len(data) > MAX_JPEG_BYTES or not data.startswith(b"\xff\xd8") or not data.endswith(b"\xff\xd9"):
        raise ValueError("invalid JPEG frame")
    offset = 2
    while offset + 4 <= len(data):
        if data[offset] != 255:
            raise ValueError("invalid JPEG marker")
        while offset < len(data) and data[offset] == 255:
            offset += 1
        if offset >= len(data):
            break
        marker = data[offset]
        offset += 1
        if marker in (0xD8, 0xD9, 0xDA):
            break
        length = int.from_bytes(data[offset:offset + 2], "big")
        if length < 2 or offset + length > len(data):
            raise ValueError("truncated JPEG segment")
        if marker in (0xC0, 0xC1, 0xC2):
            if length < 8:
                break
            height, width, channels = struct.unpack(">HHB", data[offset + 3:offset + 8])
            if not (0 < width <= 4096 and 0 < height <= 4096 and channels == 3):
                raise ValueError("expected a three-channel colour JPEG")
            return width, height
        offset += length
    raise ValueError("JPEG has no supported colour SOF")


class JpegFramer:
    """Split jpegenc's image2pipe output without an unbounded backlog."""

    def __init__(self):
        self.buffer = bytearray()

    def feed(self, chunk):
        self.buffer.extend(chunk)
        frames = []
        while self.buffer:
            start = self.buffer.find(b"\xff\xd8")
            if start < 0:
                self.buffer[:] = self.buffer[-1:]  # SOI may span reads.
                break
            if start:
                del self.buffer[:start]
            end = self.buffer.find(b"\xff\xd9", 2)
            if end < 0:
                if len(self.buffer) > MAX_JPEG_BYTES:
                    self.buffer.clear()
                    raise ValueError("JPEG capture exceeded frame limit")
                break
            frame = bytes(self.buffer[:end + 2])
            del self.buffer[:end + 2]
            jpeg_dimensions(frame)
            frames.append(frame)
        return frames


class CameraCapture:
    """One camera owner, reconnecting capture, bounded first-frame wait and stop."""

    def __init__(self, source, callback, fps=15, *, device_resolver=resolve_device,
                 command_builder=capture_command, stall_timeout_s=4):
        self.source = source
        self.callback = callback
        self.fps = fps
        self._resolve = device_resolver
        self._command = command_builder
        self._stall_timeout = stall_timeout_s
        self._lock = threading.RLock()
        self._stopping = threading.Event()
        self._ready = threading.Event()
        self._thread = None
        self._process = None
        self._state = "idle"
        self._error = None
        self._received = None
        self._received_wall = None
        self._sequence = 0
        self._dimensions = None

    def start(self, timeout=7):
        with self._lock:
            if self._thread and self._thread.is_alive():
                return
            if self._command is capture_command and not shutil.which("gst-launch-1.0"):
                raise ValueError("GStreamer capture tools are not installed in the driver image")
            self._device = self._resolve(self.source)
            self._stopping.clear()
            self._ready.clear()
            self._state = "waiting"
            self._thread = threading.Thread(target=self._run, daemon=True, name="helios-rgb")
            self._thread.start()
        if not self._ready.wait(timeout):
            error = self.snapshot()["error"] or "No colour frame received before startup timeout"
            self.stop()
            raise ValueError(error)

    def stop(self):
        with self._lock:
            self._stopping.set()
            process = self._process
            thread = self._thread
        if process is not None:
            self._terminate(process)
        if thread and thread is not threading.current_thread():
            thread.join(timeout=3)
            if thread.is_alive():
                raise RuntimeError("camera capture worker did not stop")
        with self._lock:
            self._thread = None
            self._state = "idle"
            self._received = self._received_wall = None

    @staticmethod
    def _terminate(process):
        if process.poll() is None:
            process.terminate()
            try:
                process.wait(timeout=1)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=1)

    def snapshot(self):
        with self._lock:
            age = None if self._received is None else time.monotonic() - self._received
            state = "stale" if self._state == "online" and age is not None and age > self._stall_timeout else self._state
            return {"state": state, "available": state == "online", "frame_count": self._sequence,
                    "received_at_ms": self._received_wall, "age_ms": None if age is None else int(age * 1000),
                    "width": self._dimensions[0] if self._dimensions else None,
                    "height": self._dimensions[1] if self._dimensions else None,
                    "error": self._error}

    def _run(self):
        while not self._stopping.is_set():
            process = None
            try:
                # Serialise process creation against stop(), including before the first frame.
                with self._lock:
                    if self._stopping.is_set():
                        break
                    process = subprocess.Popen(self._command(self.source, self._device, self.fps),
                                               stdout=subprocess.PIPE, stderr=subprocess.PIPE, bufsize=0)
                    self._process = process
                self._read(process)
            except Exception as exc:
                with self._lock:
                    self._state = "reconnecting"
                    self._error = str(exc)[:500]
            finally:
                if process is not None:
                    self._terminate(process)
                    process.stdout.close()
                    process.stderr.close()
                with self._lock:
                    self._process = None
            self._stopping.wait(1)

    def _read(self, process):
        framer = JpegFramer()
        last_frame = time.monotonic()
        errors = bytearray()
        with selectors.DefaultSelector() as selector:
            for pipe in (process.stdout, process.stderr):
                os.set_blocking(pipe.fileno(), False)
                selector.register(pipe, selectors.EVENT_READ)
            while not self._stopping.is_set():
                for key, _ in selector.select(timeout=0.1):
                    chunk = os.read(key.fileobj.fileno(), 65536)
                    if not chunk:
                        selector.unregister(key.fileobj)
                        continue
                    if key.fileobj is process.stderr:
                        errors.extend(chunk)
                        del errors[:-1024]
                        continue
                    for frame in framer.feed(chunk):
                        if self._stopping.is_set():
                            return
                        dimensions = jpeg_dimensions(frame)
                        self.callback(frame, int(time.time_ns()))
                        last_frame = time.monotonic()
                        with self._lock:
                            self._state = "online"
                            self._error = None
                            self._received = last_frame
                            self._received_wall = time.time_ns() // 1_000_000
                            self._dimensions = dimensions
                            self._sequence += 1
                        self._ready.set()
                if process.poll() is not None or time.monotonic() - last_frame > self._stall_timeout:
                    detail = errors.decode("utf-8", errors="replace").strip()[-500:]
                    raise ValueError(detail or "Colour stream ended or stopped producing frames")
