"""Pick-place-owned RGB-D acquisition; no camera card or ROS stream dependency."""

import multiprocessing as mp
import queue
import threading
import time
from uuid import uuid4
import zlib

import numpy as np

from hardware import CameraLease


def snapshot_rgbd(rs, cv2, frameset, scale, serial_number, after):
    """Freeze one aligned RGB-D pair acquired after the requested wall time."""
    color = frameset.get_color_frame()
    depth = frameset.get_depth_frame()
    if not color or not depth:
        return None
    timestamps = []
    for frame in (color, depth):
        domain = frame.get_frame_timestamp_domain()
        if domain not in (rs.timestamp_domain.system_time, rs.timestamp_domain.global_time):
            raise RuntimeError("RealSense snapshot requires host-synchronized timestamps")
        timestamps.append(frame.get_timestamp() / 1000.0)
    now = time.time()
    if min(timestamps) <= after:
        return None
    if any(not np.isfinite(t) or t > now + 0.1 or now - t > 1.0 for t in timestamps):
        raise RuntimeError("RealSense snapshot timestamps are stale or invalid")
    aligned = rs.align(rs.stream.color).process(frameset)
    color, depth = aligned.get_color_frame(), aligned.get_depth_frame()
    if not color or not depth:
        raise RuntimeError("RealSense RGB-D alignment failed")
    rgb = np.asanyarray(color.get_data())
    raw = np.asanyarray(depth.get_data())
    if raw.shape != rgb.shape[:2] or raw.dtype != np.uint16:
        raise RuntimeError("Invalid aligned depth frame")
    success, jpeg = cv2.imencode(".jpg", rgb, [cv2.IMWRITE_JPEG_QUALITY, 90])
    if not success:
        raise RuntimeError("Snapshot JPEG encoding failed")
    intr = color.profile.as_video_stream_profile().get_intrinsics()
    return {
        "jpeg": jpeg.tobytes(), "depth_zlib": zlib.compress(raw.astype("<u2").tobytes()),
        "depth_scale_m": scale, "serial_number": serial_number,
        "captured_at": timestamps[0], "depth_captured_at": timestamps[1],
        "width": intr.width, "height": intr.height,
        "intrinsics": {"fx": intr.fx, "fy": intr.fy, "ppx": intr.ppx, "ppy": intr.ppy,
                       "model": str(intr.model), "coeffs": list(intr.coeffs)},
    }


def _report(messages, result):
    try:
        messages.put_nowait(result)
    except queue.Full:
        try:
            messages.get_nowait()
        except queue.Empty:
            pass
        try:
            messages.put_nowait(result)
        except queue.Full:
            pass


def _capture(serial, requests, results, statuses, stopped):
    pipeline = None
    started = False
    try:
        import cv2
        import pyrealsense2 as rs

        with CameraLease(serial):
            context = rs.context()
            devices = [device for device in context.query_devices()
                       if device.get_info(rs.camera_info.serial_number) == serial]
            if len(devices) != 1:
                raise RuntimeError("Selected RealSense is unavailable")
            device = devices[0]
            usb3 = (device.supports(rs.camera_info.usb_type_descriptor)
                    and device.get_info(rs.camera_info.usb_type_descriptor).startswith("3"))
            width, height, fps = (1280, 720, 15) if usb3 else (640, 480, 6)
            scale = device.first_depth_sensor().get_depth_scale()
            if not np.isfinite(scale) or scale <= 0:
                raise RuntimeError("Invalid RealSense depth scale")
            for sensor in device.query_sensors():
                if sensor.supports(rs.option.global_time_enabled):
                    sensor.set_option(rs.option.global_time_enabled, 1)
            config = rs.config()
            config.enable_device(serial)
            config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
            config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, fps)
            pipeline = rs.pipeline(context)
            pipeline.start(config)
            started = True
            pending = None
            first_frame_deadline = time.monotonic() + 10
            last_frame = None
            last_report = 0
            try:
                while not stopped.is_set():
                    try:
                        pending = requests.get_nowait()
                    except queue.Empty:
                        pass
                    try:
                        frames = pipeline.wait_for_frames(200)
                    except RuntimeError:
                        frames = None
                    now = time.monotonic()
                    pair = (frames.get_color_frame(), frames.get_depth_frame()) if frames else ()
                    fresh = (len(pair) == 2 and all(pair)
                             and all(frame.get_frame_timestamp_domain() in (
                                 rs.timestamp_domain.system_time, rs.timestamp_domain.global_time) for frame in pair)
                             and all(0 <= time.time() - frame.get_timestamp() / 1000.0 < 1 for frame in pair))
                    if fresh:
                        last_frame = now
                        if pending is not None:
                            request_id, after = pending
                            photo = snapshot_rgbd(rs, cv2, frames, scale, serial, after)
                            if photo is not None:
                                _report(results, {"request_id": request_id, **photo})
                                pending = None
                    if ((last_frame is None and now >= first_frame_deadline)
                            or (last_frame is not None and now - last_frame >= 1)):
                        raise RuntimeError("Fresh synchronized RealSense RGB-D feedback unavailable")
                    if now - last_report >= 0.2:
                        _report(statuses, {"last_frame": last_frame})
                        last_report = now
            finally:
                if started:
                    pipeline.stop()
                    started = False
    except Exception as exc:
        _report(statuses, {"error": str(exc)})


class SnapshotCamera:
    def __init__(self, serial):
        self.serial = serial
        self._ctx = mp.get_context("spawn")
        self._process = None
        self._requests = self._results = self._statuses = self._stopped = None
        self._status = {}
        self._snapshot_lock = threading.Lock()

    def start(self):
        if self._process is not None:
            raise RuntimeError("Snapshot camera is already started")
        self._requests = self._ctx.Queue(maxsize=1)
        self._results = self._ctx.Queue(maxsize=1)
        self._statuses = self._ctx.Queue(maxsize=4)
        self._stopped = self._ctx.Event()
        self._status = {}
        self._process = self._ctx.Process(target=_capture, args=(
            self.serial, self._requests, self._results, self._statuses, self._stopped),
            daemon=True, name="pick_place_rgbd")
        self._process.start()

    def info(self):
        if self._statuses is not None:
            while True:
                try:
                    self._status = self._statuses.get_nowait()
                except queue.Empty:
                    break
        error = self._status.get("error")
        if not error and (self._process is None or not self._process.is_alive()):
            error = "Snapshot camera is stopped"
        last = self._status.get("last_frame")
        fresh = last is not None and time.monotonic() - last < 1
        return {"state": "error" if error else "running" if fresh else "starting",
                "fresh": fresh and not error, "error": error}

    def snapshot(self, after, cancel, check, timeout=5.0):
        with self._snapshot_lock:
            request_id = uuid4().hex
            self._requests.put_nowait((request_id, after))
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                if cancel.is_set():
                    raise RuntimeError("Snapshot cancelled")
                check()
                state = self.info()
                if state["state"] != "running":
                    raise RuntimeError(state["error"] or "Snapshot camera feedback lost")
                try:
                    result = self._results.get_nowait()
                except queue.Empty:
                    result = None
                if result is not None and result["request_id"] == request_id:
                    return result
                cancel.wait(0.05)
            raise RuntimeError("Fresh RGB-D snapshot timed out")

    def stop(self):
        if self._process is not None:
            self._stopped.set()
            if self._process.pid is not None:
                self._process.join(timeout=1.5)
                if self._process.is_alive():
                    self._process.terminate()
                    self._process.join(timeout=0.5)
                if self._process.is_alive():
                    self._process.kill()
                    self._process.join(timeout=0.5)
            self._process.close()
            self._process = None
        for messages in (self._requests, self._results, self._statuses):
            if messages is not None:
                messages.cancel_join_thread()
                messages.close()
        self._requests = self._results = self._statuses = None


class SnapshotCameras:
    def select(self):
        import pyrealsense2 as rs

        devices = list(rs.context().query_devices())
        if len(devices) != 1:
            raise RuntimeError("observe requires exactly one connected RealSense camera")
        serial = str(devices[0].get_info(rs.camera_info.serial_number)).strip()
        if not serial:
            raise RuntimeError("RealSense serial number is missing")
        return SnapshotCamera(serial)
