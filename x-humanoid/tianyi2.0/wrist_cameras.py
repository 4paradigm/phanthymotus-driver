"""Tianyi D405 wrist RGB and depth cards (one isolated SDK owner per wrist).

Only the two worker processes access the USB cameras. Each worker owns one
RealSense RGB-D pipeline and publishes directly to the loopback-only domain 42,
so a native SDK crash cannot bring down Tianyi's body-controller driver.

The four cards have independent lifecycle and output topics. Each wrist's
RGB/Depth cards share a session; stopping one channel leaves the other alive.
"""
from __future__ import annotations

import multiprocessing as mp
import os
import queue
import re
import threading
import time
import zlib

WIDTH = 640
HEIGHT = 480
FPS = 15
STARTUP_SECONDS = 12.0
STALE_SECONDS = 5.0
CHANNELS = ("rgb", "depth")
FORMATS = {"rgb": "image/jpeg", "depth": "image/depth-zlib"}
DDS_PROFILE = "/opt/phanthy-motus/dds-local.xml"


def camera_topic(namespace: str, side: str, channel: str) -> str:
    if side not in ("left", "right") or channel not in CHANNELS:
        raise ValueError("invalid wrist side or channel")
    return f"/{namespace.strip('/')}/camera/wrist/{side}/{channel}"


def encode_depth_mm(depth, scale_m: float) -> bytes:
    """Convert SDK Z16 depth units to renderer's 640x480 uint16 millimetres.

    Neither assume SDK units are millimetres nor wrap out-of-range values.
    The current Core depth renderer expects headerless little-endian Z16 zlib.
    """
    import numpy as np

    if (depth.shape != (HEIGHT, WIDTH) or depth.dtype != np.uint16
            or not np.isfinite(scale_m) or scale_m <= 0):
        raise ValueError("invalid D405 Z16 shape, dtype or SDK depth scale")
    mm = np.rint(depth.astype(np.float64) * float(scale_m) * 1000.0)
    mm[(mm < 1) | (mm > 65535)] = 0
    return zlib.compress(mm.astype("<u2").tobytes(), 1)


def _frame_intrinsics(frame):
    """Read actual calibrated optics; never fill unknown values from a guess."""
    intr = frame.profile.as_video_stream_profile().get_intrinsics()
    return {
        "width": int(intr.width),
        "height": int(intr.height),
        "K": [float(intr.fx), 0.0, float(intr.ppx),
              0.0, float(intr.fy), float(intr.ppy),
              0.0, 0.0, 1.0],
        "D": [float(v) for v in intr.coeffs],
        "sdk_distortion": str(intr.model),
    }


def _report(status_queue, status):
    snapshot = {
        "error": status.get("error"),
        "last_frame": dict(status["last_frame"]),
        "frames": dict(status["frames"]),
        "intrinsics": dict(status["intrinsics"]),
        "sdk_timestamp_ms": dict(status["sdk_timestamp_ms"]),
        "depth_scale_m": status.get("depth_scale_m"),
        "device_name": status.get("device_name"),
    }
    try:
        status_queue.put_nowait(snapshot)
    except queue.Full:
        # Latest-only status: never block image acquisition on a slow info().
        try:
            status_queue.get_nowait()
        except queue.Empty:
            pass
        try:
            status_queue.put_nowait(snapshot)
        except queue.Full:
            pass


def _capture_worker(namespace, side, serial, quality, initial_channels,
                    commands, stop_event, status_queue):
    """Run on its own process: only this process creates an rs.pipeline."""
    # Set before rclpy creates a participant; never expose camera data on LAN.
    os.environ["ROS_DOMAIN_ID"] = "42"
    os.environ["RMW_IMPLEMENTATION"] = "rmw_fastrtps_cpp"
    os.environ["FASTRTPS_DEFAULT_PROFILES_FILE"] = DDS_PROFILE
    pipe = node = None
    started = ros_started = False
    status = {
        "error": None, "last_frame": {}, "frames": {}, "intrinsics": {},
        "sdk_timestamp_ms": {}, "depth_scale_m": None, "device_name": None,
    }
    try:
        if not os.path.isfile(DDS_PROFILE):
            raise RuntimeError("Missing loopback-only DDS profile; refusing camera publication")
        import cv2
        import numpy as np
        import pyrealsense2 as rs
        import rclpy
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import CompressedImage

        matched = [d for d in rs.context().query_devices()
                   if d.get_info(rs.camera_info.serial_number) == serial]
        if len(matched) != 1:
            raise RuntimeError(f"D405 serial {serial} not found or ambiguous")
        device_name = matched[0].get_info(rs.camera_info.name)
        if "D405" not in device_name:
            raise RuntimeError(f"serial {serial} is not a D405: {device_name}")
        status["device_name"] = device_name

        config = rs.config()
        config.enable_device(serial)
        config.enable_stream(rs.stream.color, WIDTH, HEIGHT, rs.format.rgb8, FPS)
        config.enable_stream(rs.stream.depth, WIDTH, HEIGHT, rs.format.z16, FPS)
        pipe = rs.pipeline()
        active_profile = pipe.start(config)
        started = True
        scale = float(active_profile.get_device().first_depth_sensor().get_depth_scale())
        if not np.isfinite(scale) or scale <= 0:
            raise RuntimeError("D405 returned invalid depth scale")
        status["depth_scale_m"] = scale

        rclpy.init(domain_id=42)
        ros_started = True
        node = rclpy.create_node(f"tianyi_wrist_{side}_capture")
        pubs = {channel: node.create_publisher(
            CompressedImage, camera_topic(namespace, side, channel),
            qos_profile_sensor_data) for channel in CHANNELS}

        wanted = set(initial_channels)
        last_report = 0.0
        last_input = time.monotonic()
        while not stop_event.is_set():
            while True:
                try:
                    wanted = set(commands.get_nowait())
                except queue.Empty:
                    break
            try:
                frameset = pipe.wait_for_frames(timeout_ms=1000)
            except RuntimeError:
                if time.monotonic() - last_input >= STARTUP_SECONDS:
                    raise RuntimeError("No D405 RGB-D frames received within timeout")
                continue
            last_input = time.monotonic()
            for channel, frame in (("rgb", frameset.get_color_frame()),
                                   ("depth", frameset.get_depth_frame())):
                if channel not in wanted or not frame:
                    continue
                if channel not in status["intrinsics"]:
                    status["intrinsics"][channel] = _frame_intrinsics(frame)
                raw = np.asanyarray(frame.get_data())
                msg = CompressedImage()
                # Host receive/publish time, not a hardware-clock timestamp
                # misrepresented as a ROS epoch. Raw SDK timestamp in info().
                msg.header.stamp = node.get_clock().now().to_msg()
                msg.header.frame_id = f"{namespace}_wrist_{side}_{channel}_optical_frame"
                if channel == "rgb":
                    if raw.shape != (HEIGHT, WIDTH, 3) or raw.dtype != np.uint8:
                        raise RuntimeError("Unexpected D405 RGB8 frame layout")
                    bgr = cv2.cvtColor(raw, cv2.COLOR_RGB2BGR)
                    ok, encoded = cv2.imencode(
                        ".jpg", bgr, [cv2.IMWRITE_JPEG_QUALITY, quality])
                    if not ok:
                        raise RuntimeError("D405 JPEG encoding failed")
                    msg.format = "jpeg"
                    msg.data = encoded.tobytes()
                else:
                    msg.format = "16UC1; compressedDepth zlib"
                    msg.data = encode_depth_mm(raw, scale)
                pubs[channel].publish(msg)
                now = time.monotonic()
                status["last_frame"][channel] = now
                status["frames"][channel] = status["frames"].get(channel, 0) + 1
                status["sdk_timestamp_ms"][channel] = float(frame.get_timestamp())
            now = time.monotonic()
            if now - last_report >= 0.5:
                _report(status_queue, status)
                last_report = now
    except Exception as exc:
        status["error"] = str(exc)
        _report(status_queue, status)
    finally:
        if started:
            try:
                pipe.stop()
            except Exception:
                pass
        if node is not None:
            try:
                node.destroy_node()
            except Exception:
                pass
        if ros_started:
            try:
                rclpy.shutdown()
            except Exception:
                pass


class WristCameraSession:
    """One physical D405, shared by its independently controlled RGB/Depth cards."""

    def __init__(self, namespace, side, serial, jpeg_quality=85):
        if side not in ("left", "right"):
            raise ValueError("invalid wrist side")
        self.namespace = namespace
        self.side = side
        self.serial = str(serial or "").strip()
        self.jpeg_quality = max(1, min(100, int(jpeg_quality)))
        self.blocked_reason = None
        self._ctx = mp.get_context("spawn")
        self._lock = threading.RLock()
        self._proc = self._commands = self._queue = self._quit = None
        self._wanted = set()
        self._requested_at = {}
        self._status = {}

    def _drain(self):
        if self._queue is None:
            return
        while True:
            try:
                self._status = self._queue.get_nowait()
            except queue.Empty:
                break

    def _close(self):
        if self._proc is not None:
            self._quit.set()
            self._proc.join(timeout=2.0)
            if self._proc.is_alive():
                self._proc.terminate()
                self._proc.join(timeout=1.0)
            if self._proc.is_alive():
                self._proc.kill()
                self._proc.join(timeout=0.5)
            self._proc.close()
            self._proc = None
        for q in (self._queue, self._commands):
            if q is not None:
                q.cancel_join_thread()
                q.close()
        self._queue = self._commands = self._quit = None

    def _send_wanted(self):
        """Send latest channel selection without blocking the MCP HTTP thread."""
        if self._commands is None:
            return
        state = tuple(sorted(self._wanted))
        try:
            self._commands.put_nowait(state)
        except queue.Full:
            try:
                self._commands.get_nowait()
            except queue.Empty:
                pass
            try:
                self._commands.put_nowait(state)
            except queue.Full:
                pass

    def start(self, channel):
        with self._lock:
            if channel not in CHANNELS:
                raise ValueError("invalid wrist channel")
            if self.blocked_reason:
                raise RuntimeError(self.blocked_reason)
            if not re.fullmatch(r"\d{12}", self.serial):
                raise RuntimeError(
                    f"Bind {self.side} wrist to verified 12-digit RealSense SDK "
                    "Serial Number in plugins.wrist_cameras before starting")
            self._drain()
            if channel in self._wanted and self._proc is not None and self._proc.is_alive():
                return self.info(channel)
            self._wanted.add(channel)
            self._requested_at[channel] = time.monotonic()
            if (self._proc is None or not self._proc.is_alive()
                    or self._status.get("error")):
                self._close()
                self._status = {}
                for name in self._wanted:
                    self._requested_at[name] = time.monotonic()
                self._commands = self._ctx.Queue(maxsize=8)
                self._queue = self._ctx.Queue(maxsize=8)
                self._quit = self._ctx.Event()
                self._proc = self._ctx.Process(
                    target=_capture_worker,
                    args=(self.namespace, self.side, self.serial, self.jpeg_quality,
                          tuple(self._wanted), self._commands, self._quit, self._queue),
                    name=f"tianyi_wrist_{self.side}", daemon=True)
                self._proc.start()
            else:
                self._send_wanted()
            return self.info(channel)

    def stop(self, channel):
        with self._lock:
            self._wanted.discard(channel)
            self._requested_at.pop(channel, None)
            if not self._wanted:
                self._close()
                self._status = {}
            elif self._commands is not None:
                self._send_wanted()
            return self.info(channel)

    def info(self, channel):
        from common.camera_info import build as camera_info_build

        with self._lock:
            self._drain()
            topic = camera_topic(self.namespace, self.side, channel)
            now = time.monotonic()
            last = self._status.get("last_frame", {}).get(channel)
            requested = self._requested_at.get(channel, now)
            fresh = (channel in self._wanted and last is not None
                     and last >= requested and now-last < STALE_SECONDS)
            error = self.blocked_reason or self._status.get("error")
            if channel in self._wanted:
                if not error and (self._proc is None or not self._proc.is_alive()):
                    error = "D405 capture worker exited"
                if not error and now-requested >= STARTUP_SECONDS and not fresh:
                    error = "D405 not supplying fresh frames"
                state = "error" if error else ("running" if fresh else "starting")
            else:
                state = "unconfigured" if not self.serial else "idle"
                error = None
            intr = self._status.get("intrinsics", {}).get(channel)
            camera_info = camera_info_build(
                topic=topic, id=f"x-humanoid/tianyi2.0/wrist-{self.side}-{channel}",
                format=FORMATS[channel], width=WIDTH, height=HEIGHT,
                K=intr["K"] if intr else None,
                D=intr["D"] if intr else None,
                distortion_model="unknown",  # SDK raw distortion model retained below
                source="vendor-spec" if intr else "unknown",
                pipeline=["realsense-sdk", "jpeg" if channel == "rgb" else "mm-zlib"],
                vendor={"sdk_distortion": intr["sdk_distortion"]} if intr else {})
            return {
                "state": state, "error": error, "side": self.side, "serial": self.serial or None,
                "channel": channel, "fresh": fresh, "width": WIDTH, "height": HEIGHT, "fps": FPS,
                "format": FORMATS[channel], "depth_scale_m": self._status.get("depth_scale_m")
                if channel == "depth" else None,
                "frames_published": self._status.get("frames", {}).get(channel, 0),
                "last_frame_age_s": None if last is None else max(0.0, now-last),
                "sdk_timestamp_ms": self._status.get("sdk_timestamp_ms", {}).get(channel),
                "stamp_source": "host_ros_clock",
                "depth_aligned_to_rgb": False,
                "camera_info": [camera_info],
                "topic_out": [{"topic": topic, "format": FORMATS[channel]}],
            }


class WristCameraPlugin:
    """One fixed wrist/modality sensor card; no implicit camera motion."""

    def __init__(self, session: WristCameraSession, channel: str):
        if channel not in CHANNELS:
            raise ValueError("invalid wrist channel")
        self._session = session
        self._channel = channel
        self._name = f"camera_wrist_{session.side}_{channel}"

    def get_tool(self):
        return {
            "name": self._name, "type": "sensor",
            "description": f"Tianyi {self._session.side} wrist D405 {self._channel} stream",
            "inputSchema": {"type": "object", "properties": {}},
            "topic_out": [{
                "topic": camera_topic(self._session.namespace, self._session.side, self._channel),
                "format": FORMATS[self._channel],
            }],
        }

    def start(self):
        return self._session.start(self._channel)

    def stop(self):
        return self._session.stop(self._channel)

    def dispatch(self, action, args):
        if action == "start":
            return self.start()
        if action == "stop":
            return self.stop()
        if action == "info":
            return self._session.info(self._channel)
        return {"state": "error", "error": f"Unknown action: {action}"}
