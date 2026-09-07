"""Go2 external RealSense depth and left-infrared sensor cards.

The stereo sensor has one owner. Card enable flags control publication without
reopening the sensor when its sibling card starts/stops. The color sensor is
never opened here, so ext_camera can continue using its V4L2 interface.
"""
from __future__ import annotations

import multiprocessing as mp
import copy
import queue
import threading
import time
import zlib

import numpy as np

WIDTH, HEIGHT = 640, 480
STREAMS = ("ext_depth", "ext_infrared")
FORMATS = {"ext_depth": "image/depth-zlib", "ext_infrared": "image/jpeg"}
STALE_SECONDS = 3.0
STARTUP_SECONDS = 10.0


def encode_depth(raw: np.ndarray, scale: float) -> bytes:
    """Renderer contract: zlib of 640x480 little-endian uint16 millimetres."""
    if raw.shape != (HEIGHT, WIDTH) or raw.dtype != np.uint16:
        raise ValueError("Expected a 640x480 Z16 depth frame")
    if not np.isfinite(scale) or scale <= 0:
        raise ValueError("Invalid RealSense depth scale")
    mm = np.rint(raw.astype(np.float64) * scale * 1000.0)
    # Zero means invalid; never wrap out-of-range measurements into near objects.
    mm[(mm < 1) | (mm > 65535)] = 0
    return zlib.compress(mm.astype('<u2').tobytes(), 1)


def _report(status_queue, status):
    status = copy.deepcopy(status)  # Queue serializes on its feeder thread.
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


def _capture(namespace, enabled, quit_event, status_queue):
    """Process boundary bounds SDK failures and isolates capture from robot RPC."""
    sensor = node = None
    opened = streaming = False
    status = {"frames": {s: 0 for s in STREAMS}, "last_frame": {}, "error": None}
    try:
        import cv2
        import pyrealsense2 as rs
        import rclpy
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import CompressedImage

        context = rs.context()
        devices = list(context.query_devices())
        if len(devices) != 1:
            raise RuntimeError(f"Expected one external RealSense camera, found {len(devices)}")
        device = devices[0]
        usb_type = (device.get_info(rs.camera_info.usb_type_descriptor)
                    if device.supports(rs.camera_info.usb_type_descriptor) else 'unknown')
        # Leave room for the existing 720p/15 RGB stream on a USB 2 connection.
        # Unknown transport uses the conservative profile too.
        fps = 15 if usb_type.startswith('3') else 6
        status['usb_type'] = usb_type
        status['fps'] = fps
        sensor = device.first_depth_sensor()
        scale = sensor.get_depth_scale()
        if not np.isfinite(scale) or scale <= 0:
            raise RuntimeError("RealSense reported an invalid depth scale")
        status["device_name"] = device.get_info(rs.camera_info.name)
        status["depth_scale_m"] = scale
        profiles = []
        for kind, fmt, index in ((rs.stream.depth, rs.format.z16, 0),
                                 (rs.stream.infrared, rs.format.y8, 1)):
            matches = [p for p in sensor.get_stream_profiles()
                       if p.stream_type() == kind and p.format() == fmt
                       and p.stream_index() == index and p.fps() == fps
                       and p.as_video_stream_profile().width() == WIDTH
                       and p.as_video_stream_profile().height() == HEIGHT]
            if not matches:
                raise RuntimeError(f"{kind} 640x480@{fps} {fmt} is not supported")
            profiles.append(matches[0])

        rclpy.init()
        node = rclpy.create_node(f"{namespace}_realsense_stereo")
        publishers = {s: node.create_publisher(
            CompressedImage, f"/{namespace}/{s}/image", qos_profile_sensor_data)
            for s in STREAMS}
        frames = rs.frame_queue(4)
        sensor.open(profiles)
        opened = True
        sensor.start(frames)
        streaming = True
        last_received = {s: time.monotonic() for s in STREAMS}
        last_report = 0.0
        while not quit_event.is_set():
            ok, frame = frames.try_wait_for_frame(200)
            now = time.monotonic()
            if any(now - t > STALE_SECONDS for t in last_received.values()):
                raise RuntimeError("RealSense depth/infrared frames stopped arriving")
            if not ok:
                continue
            kind = frame.profile.stream_type()
            if kind == rs.stream.depth:
                stream = "ext_depth"
            elif kind == rs.stream.infrared and frame.profile.stream_index() == 1:
                stream = "ext_infrared"
            else:
                continue
            last_received[stream] = now
            if not enabled[stream].is_set():
                continue
            raw = np.asanyarray(frame.get_data())
            msg = CompressedImage()
            msg.header.stamp = node.get_clock().now().to_msg()
            msg.header.frame_id = f"{namespace}_{stream}_optical"
            if stream == "ext_depth":
                msg.format = "16UC1; compressedDepth zlib"
                msg.data = encode_depth(raw, scale)
            else:
                if raw.shape != (HEIGHT, WIDTH) or raw.dtype != np.uint8:
                    raise RuntimeError("Expected a 640x480 Y8 infrared frame")
                success, jpeg = cv2.imencode('.jpg', raw, [cv2.IMWRITE_JPEG_QUALITY, 85])
                if not success:
                    raise RuntimeError("Infrared JPEG encoding failed")
                msg.format = "jpeg"
                msg.data = jpeg.tobytes()
            # Stop can race encoding; recheck immediately before publication.
            if not enabled[stream].is_set():
                continue
            publishers[stream].publish(msg)
            status["frames"][stream] += 1
            status["last_frame"][stream] = now
            if now - last_report >= 0.2:
                _report(status_queue, status)
                last_report = now
    except Exception as exc:
        status["error"] = str(exc)
        _report(status_queue, status)
    finally:
        # Do not let a failed stop prevent close/node cleanup. The parent also
        # enforces a process deadline if native SDK shutdown is stuck.
        if streaming:
            try:
                sensor.stop()
            except Exception:
                pass
        if opened:
            try:
                sensor.close()
            except Exception:
                pass
        if node is not None:
            node.destroy_node()
            rclpy.shutdown()


class RealSenseSession:
    """One stereo device shared by two single-instance cards."""

    def __init__(self, namespace):
        self.namespace = namespace
        self._lock = threading.RLock()
        self._ctx = mp.get_context('spawn')
        self._proc = self._queue = self._quit = None
        self._enabled = {}
        self._wanted = set()
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
        if self._queue is not None:
            self._queue.close()
            self._queue = None

    def start(self, stream):
        with self._lock:
            self._drain()
            current = self.info(stream)
            if current['state'] in ('running', 'starting'):
                return current
            self._wanted.add(stream)
            self._requested_at[stream] = time.monotonic()
            if (self._proc is None or not self._proc.is_alive()
                    or self._status.get('error') or current['state'] == 'error'):
                self._close()
                self._status = {}
                self._queue = self._ctx.Queue(maxsize=4)
                self._quit = self._ctx.Event()
                self._enabled = {s: self._ctx.Event() for s in STREAMS}
                for s in self._wanted:
                    self._enabled[s].set()
                    self._requested_at[s] = time.monotonic()
                self._proc = self._ctx.Process(target=_capture, args=(
                    self.namespace, self._enabled, self._quit, self._queue),
                    name='go2_realsense_stereo', daemon=True)
                self._proc.start()
            else:
                self._enabled[stream].set()
            # A start request schedules capture; running requires a published
            # frame. info exposes readiness/errors without blocking HTTP on USB.
            return self.info(stream)

    def stop(self, stream):
        with self._lock:
            self._wanted.discard(stream)
            if stream in self._enabled:
                self._enabled[stream].clear()
            if not self._wanted:
                self._close()
                self._status = {}
            return self.info(stream)

    def info(self, stream):
        with self._lock:
            self._drain()
            now = time.monotonic()
            last = self._status.get('last_frame', {}).get(stream)
            requested = self._requested_at.get(stream, now)
            fresh = last is not None and last >= requested and now - last < STALE_SECONDS
            error = None
            state = 'idle'
            if stream in self._wanted:
                error = self._status.get('error')
                if not error and (self._proc is None or not self._proc.is_alive()):
                    error = 'RealSense capture process exited'
                stalled = last is not None and last >= requested and now-last >= STALE_SECONDS
                if not error and (stalled or (not fresh and now-requested >= STARTUP_SECONDS)):
                    error = 'No fresh RealSense frames'
                state = 'error' if error else ('running' if fresh else 'starting')
            return {
                'state': state, 'fresh': fresh and state == 'running', 'error': error,
                'device_name': self._status.get('device_name'),
                'width': WIDTH, 'height': HEIGHT, 'fps': self._status.get('fps'),
                'usb_type': self._status.get('usb_type'),
                'encoding': '16UC1' if stream == 'ext_depth' else 'mono8',
                'source_stream': 'depth' if stream == 'ext_depth' else 'infrared',
                'stream_index': 0 if stream == 'ext_depth' else 1,
                'unit': 'mm' if stream == 'ext_depth' else 'intensity',
                'depth_scale_m': self._status.get('depth_scale_m') if stream == 'ext_depth' else None,
                'frames_published': self._status.get('frames', {}).get(stream, 0),
                'last_frame_age_s': None if last is None else max(0, now-last),
                'topic_in': [],
                'topic_out': [{'topic': f'/{self.namespace}/{stream}/image',
                               'format': FORMATS[stream]}],
            }


class _StereoPlugin:
    def __init__(self, plugin_config, namespace, executor, session):
        self._session = session

    def get_tool(self):
        return {
            'name': self.PREFIX, 'type': 'sensor', 'multiInstance': False,
            'description': self.DESCRIPTION,
            'inputSchema': {'type': 'object', 'properties': {}},
            'topic_in': [],
            'topic_out': [{'topic': f'/{self._session.namespace}/{self.PREFIX}/image',
                           'format': FORMATS[self.PREFIX]}],
        }

    def start(self):
        pass  # Start only when the canvas enables this sensor.

    def stop(self):
        self._session.stop(self.PREFIX)

    def dispatch(self, action, args):
        if action == 'start':
            return self._session.start(self.PREFIX)
        if action == 'stop':
            return self._session.stop(self.PREFIX)
        if action in ('info', self.PREFIX):
            return self._session.info(self.PREFIX)
        return None


class ExtDepthPlugin(_StereoPlugin):
    PREFIX = 'ext_depth'
    DESCRIPTION = 'External RealSense depth — 640x480, zlib uint16 millimetres (0=invalid); USB2: 6fps, USB3: 15fps'


class ExtInfraredPlugin(_StereoPlugin):
    PREFIX = 'ext_infrared'
    DESCRIPTION = 'RealSense 左近红外图像（反射强度，非热成像）— 640x480 灰度 JPEG；USB2: 6fps, USB3: 15fps'
