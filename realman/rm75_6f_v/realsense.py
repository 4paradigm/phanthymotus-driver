"""Shared RealSense RGB/depth/infrared capture for RM75 ext_camera cards."""
from __future__ import annotations

import copy
import hashlib
import multiprocessing as mp
import queue
import threading
import time
import zlib

import numpy as np

DEPTH_WIDTH, DEPTH_HEIGHT = 640, 480
RGB_USB3_WIDTH, RGB_USB3_HEIGHT = 1280, 720
RGB_USB2_WIDTH, RGB_USB2_HEIGHT = 640, 480
STREAMS = ("rgb", "depth", "infrared")
FORMATS = {
    "rgb": "image/jpeg",
    "depth": "image/depth-zlib",
    "infrared": "image/jpeg",
}
ENCODINGS = {"rgb": "bgr8", "depth": "16UC1", "infrared": "mono8"}
UNITS = {"rgb": "color", "depth": "mm", "infrared": "intensity"}
STREAM_INDEX = {"rgb": 0, "depth": 0, "infrared": 1}
STALE_SECONDS = 3.0
STARTUP_SECONDS = 10.0


def _device_info(device, field, default=""):
    try:
        if device.supports(field):
            return device.get_info(field)
    except Exception:
        pass
    return default


def find_device_by_serial(rs, context, serial_number):
    """Return exactly the configured RealSense, never an arbitrary first device."""
    matches = [
        device for device in context.query_devices()
        if str(_device_info(
            device, rs.camera_info.serial_number)).strip() == serial_number
    ]
    if len(matches) != 1:
        raise RuntimeError("Selected RealSense camera is unavailable")
    return matches[0]


def encode_depth(raw: np.ndarray, scale: float) -> bytes:
    """Renderer contract: zlib of 640x480 little-endian uint16 millimetres."""
    if raw.shape != (DEPTH_HEIGHT, DEPTH_WIDTH) or raw.dtype != np.uint16:
        raise ValueError("Expected a 640x480 Z16 depth frame")
    if not np.isfinite(scale) or scale <= 0:
        raise ValueError("Invalid RealSense depth scale")
    mm = np.rint(raw.astype(np.float64) * scale * 1000.0)
    # Zero means invalid; never wrap out-of-range measurements into near objects.
    mm[(mm < 1) | (mm > 65535)] = 0
    return zlib.compress(mm.astype("<u2").tobytes(), 1)


def _report(status_queue, status):
    status = copy.deepcopy(status)
    try:
        status_queue.put_nowait(status)
    except queue.Full:
        try:
            status_queue.get_nowait()
        except queue.Empty:
            pass
        try:
            status_queue.put_nowait(status)
        except queue.Full:
            pass


def _capture(namespace, serial_number, routes, commands, quit_event, status_queue):
    """Own one SDK pipeline and fan all three channels out to card instances."""
    from common import logsafe

    logsafe.install(check_fd=False)
    node = pipeline = None
    streaming = False
    status = {
        "frames": {},
        "last_frame": {},
        "channels": {},
        "profiles": {},
        "error": None,
    }
    try:
        import cv2
        import pyrealsense2 as rs
        import rclpy
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import CompressedImage

        context = rs.context()
        device = find_device_by_serial(rs, context, serial_number)
        usb_type = str(_device_info(
            device, rs.camera_info.usb_type_descriptor, "unknown"))
        usb3 = usb_type.startswith("3")
        fps = 15 if usb3 else 6
        rgb_width = RGB_USB3_WIDTH if usb3 else RGB_USB2_WIDTH
        rgb_height = RGB_USB3_HEIGHT if usb3 else RGB_USB2_HEIGHT

        scale = device.first_depth_sensor().get_depth_scale()
        if not np.isfinite(scale) or scale <= 0:
            raise RuntimeError("RealSense reported an invalid depth scale")

        status.update({
            "device_name": _device_info(
                device, rs.camera_info.name, "Intel RealSense"),
            "serial_number": serial_number,
            "usb_type": usb_type,
            "depth_scale_m": scale,
            "profiles": {
                "rgb": {"width": rgb_width, "height": rgb_height, "fps": fps},
                "depth": {"width": DEPTH_WIDTH, "height": DEPTH_HEIGHT, "fps": fps},
                "infrared": {"width": DEPTH_WIDTH, "height": DEPTH_HEIGHT, "fps": fps},
            },
        })

        config = rs.config()
        config.enable_device(serial_number)
        config.enable_stream(
            rs.stream.color, rgb_width, rgb_height, rs.format.bgr8, fps)
        config.enable_stream(
            rs.stream.depth, DEPTH_WIDTH, DEPTH_HEIGHT, rs.format.z16, fps)
        config.enable_stream(
            rs.stream.infrared, 1, DEPTH_WIDTH, DEPTH_HEIGHT, rs.format.y8, fps)

        rclpy.init()
        suffix = hashlib.sha256(serial_number.encode()).hexdigest()[:12]
        node = rclpy.create_node(f"{namespace}_realsense_rgbd_{suffix}")
        publishers = {}
        pipeline = rs.pipeline(context)
        pipeline.start(config)
        streaming = True
        started_at = time.monotonic()
        last_received = {stream: None for stream in STREAMS}
        last_report = 0.0

        while not quit_event.is_set():
            while True:
                try:
                    routes = commands.get_nowait()
                except queue.Empty:
                    break

            for key in list(publishers):
                if key not in routes or publishers[key][0] != routes[key]:
                    node.destroy_publisher(publishers.pop(key)[1])
            for key, channel in routes.items():
                if key not in publishers:
                    topic = (
                        f"/{namespace}/ext_camera/"
                        f"{key.replace('-', '_')}/{channel}"
                    )
                    publishers[key] = (
                        channel,
                        node.create_publisher(
                            CompressedImage, topic, qos_profile_sensor_data),
                    )

            try:
                frameset = pipeline.wait_for_frames(200)
            except RuntimeError:
                frameset = None
            now = time.monotonic()
            frames = {}
            if frameset:
                frames = {
                    "rgb": frameset.get_color_frame(),
                    "depth": frameset.get_depth_frame(),
                    "infrared": frameset.get_infrared_frame(1),
                }
                for stream, frame in frames.items():
                    if frame:
                        last_received[stream] = now

            if any(timestamp is None for timestamp in last_received.values()):
                if now - started_at >= STARTUP_SECONDS:
                    raise RuntimeError(
                        "RealSense RGB/depth/infrared startup timed out")
            elif any(
                now - timestamp >= STALE_SECONDS
                for timestamp in last_received.values()
            ):
                raise RuntimeError(
                    "RealSense RGB/depth/infrared frames stopped arriving")

            for stream, frame in frames.items():
                if not frame or stream not in routes.values():
                    continue
                raw = np.asanyarray(frame.get_data())
                msg = CompressedImage()
                msg.header.stamp = node.get_clock().now().to_msg()
                msg.header.frame_id = f"{namespace}_{stream}_optical"
                if stream == "depth":
                    msg.format = "16UC1; compressedDepth zlib"
                    msg.data = encode_depth(raw, scale)
                else:
                    profile = status["profiles"][stream]
                    expected_shape = (
                        (profile["height"], profile["width"], 3)
                        if stream == "rgb" else
                        (profile["height"], profile["width"])
                    )
                    if raw.shape != expected_shape or raw.dtype != np.uint8:
                        raise RuntimeError(
                            f"Expected a valid {ENCODINGS[stream]} {stream} frame")
                    quality = 80 if stream == "rgb" else 85
                    success, jpeg = cv2.imencode(
                        ".jpg", raw, [cv2.IMWRITE_JPEG_QUALITY, quality])
                    if not success:
                        raise RuntimeError(f"{stream} JPEG encoding failed")
                    msg.format = "jpeg"
                    msg.data = jpeg.tobytes()

                for key, (channel, publisher) in publishers.items():
                    if channel == stream:
                        publisher.publish(msg)
                        status["frames"][key] = status["frames"].get(key, 0) + 1
                        status["last_frame"][key] = now
                        status["channels"][key] = channel

            if now - last_report >= 0.2:
                _report(status_queue, status)
                last_report = now
    except Exception as exc:
        status["error"] = str(exc)
        _report(status_queue, status)
    finally:
        if streaming:
            try:
                pipeline.stop()
            except Exception:
                pass
        if node is not None:
            node.destroy_node()
            rclpy.shutdown()


class RealSenseSession:
    """One physical camera shared by all RGB/depth/infrared card instances."""

    def __init__(self, namespace, serial_number):
        self.namespace = namespace
        self.serial_number = serial_number
        self._lock = threading.RLock()
        self._ctx = mp.get_context("spawn")
        self._proc = self._queue = self._quit = None
        self._commands = None
        self._wanted = {}
        self._requested_at = {}
        self._status = {}

    def _drain(self):
        if self._queue is not None:
            while True:
                try:
                    self._status = self._queue.get_nowait()
                except queue.Empty:
                    break

    def _close(self):
        if self._proc is not None:
            self._quit.set()
            self._proc.join(timeout=1.5)
            if self._proc.is_alive():
                self._proc.terminate()
                self._proc.join(timeout=0.5)
            if self._proc.is_alive():
                self._proc.kill()
                self._proc.join(timeout=0.5)
            self._proc.close()
            self._proc = None
        for pipe_queue in (self._queue, self._commands):
            if pipe_queue is not None:
                pipe_queue.cancel_join_thread()
                pipe_queue.close()
        self._queue = self._commands = None

    def start(self, instance_id, channel):
        with self._lock:
            self._drain()
            current = self.info(instance_id, channel)
            if (
                self._wanted.get(instance_id) == channel
                and current["state"] in ("running", "starting")
            ):
                return current
            self._wanted[instance_id] = channel
            self._requested_at[instance_id] = time.monotonic()
            if (
                self._proc is None
                or not self._proc.is_alive()
                or self._status.get("error")
                or current["state"] == "error"
            ):
                self._close()
                self._status = {}
                self._queue = self._ctx.Queue(maxsize=4)
                self._quit = self._ctx.Event()
                self._commands = self._ctx.Queue()
                for key in self._wanted:
                    self._requested_at[key] = time.monotonic()
                self._proc = self._ctx.Process(
                    target=_capture,
                    args=(
                        self.namespace,
                        self.serial_number,
                        dict(self._wanted),
                        self._commands,
                        self._quit,
                        self._queue,
                    ),
                    name="realman_realsense_rgbd",
                    daemon=True,
                )
                self._proc.start()
            else:
                self._commands.put(dict(self._wanted))
            return self.info(instance_id, channel)

    def stop(self, instance_id):
        with self._lock:
            channel = self._wanted.pop(instance_id, "rgb")
            self._requested_at.pop(instance_id, None)
            if not self._wanted:
                self._close()
                self._status = {}
            elif self._commands is not None:
                self._commands.put(dict(self._wanted))
            return self.info(instance_id, channel)

    def info(self, instance_id, channel):
        with self._lock:
            self._drain()
            now = time.monotonic()
            last = self._status.get("last_frame", {}).get(instance_id)
            requested = self._requested_at.get(instance_id, now)
            fresh = (
                last is not None
                and last >= requested
                and now - last < STALE_SECONDS
                and self._status.get("channels", {}).get(instance_id) == channel
            )
            error = None
            state = "idle"
            if instance_id in self._wanted:
                error = self._status.get("error")
                if not error and (self._proc is None or not self._proc.is_alive()):
                    error = "RealSense capture process exited"
                stalled = (
                    last is not None
                    and last >= requested
                    and now - last >= STALE_SECONDS
                )
                if not error and (
                    stalled or (not fresh and now - requested >= STARTUP_SECONDS)
                ):
                    error = "No fresh RealSense frames"
                state = "error" if error else ("running" if fresh else "starting")

            default_width = (
                RGB_USB3_WIDTH if channel == "rgb" else DEPTH_WIDTH)
            default_height = (
                RGB_USB3_HEIGHT if channel == "rgb" else DEPTH_HEIGHT)
            profile = self._status.get("profiles", {}).get(channel, {})
            return {
                "channel": channel,
                "state": state,
                "fresh": fresh and state == "running",
                "error": error,
                "device_name": self._status.get("device_name"),
                "serial_number": self.serial_number,
                "width": profile.get("width", default_width),
                "height": profile.get("height", default_height),
                "fps": profile.get("fps"),
                "usb_type": self._status.get("usb_type"),
                "encoding": ENCODINGS[channel],
                "source_stream": channel,
                "stream_index": STREAM_INDEX[channel],
                "unit": UNITS[channel],
                "depth_scale_m": (
                    self._status.get("depth_scale_m")
                    if channel == "depth" else None
                ),
                "frames_published": self._status.get(
                    "frames", {}).get(instance_id, 0),
                "last_frame_age_s": (
                    None if last is None else max(0, now - last)),
                "topic_in": [],
                "topic_out": [{
                    "topic": (
                        f"/{self.namespace}/ext_camera/"
                        f"{instance_id.replace('-', '_')}/{channel}"
                    ),
                    "format": FORMATS[channel],
                }],
            }
