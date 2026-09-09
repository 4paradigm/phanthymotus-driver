"""Shared RealSense stereo capture for ext_camera depth/infrared instances.

RGB is owned independently by the existing V4L2 capture path. Each instance
receives only its selected channel on a modality-specific topic.
"""
from __future__ import annotations

import multiprocessing as mp
import copy
import hashlib
from pathlib import Path
import queue
import re
import threading
import time
import zlib

import numpy as np

WIDTH, HEIGHT = 640, 480
STREAMS = ("depth", "infrared")
FORMATS = {"depth": "image/depth-zlib", "infrared": "image/jpeg"}
STALE_SECONDS = 3.0
STARTUP_SECONDS = 10.0


def matches_usb_device(physical_port: str, usb_path: str) -> bool:
    """Match the SDK device to the physical USB parent of the selected RGB node."""
    if not physical_port or not usb_path:
        return False
    # Linux V4L2 reports the depth video node's absolute sysfs path. Require
    # the directory boundary so port 2-1 cannot accidentally match port 2-10.
    if physical_port.startswith('/'):
        return physical_port.startswith(usb_path.rstrip('/') + '/')
    # The Linux RSUSB backend instead reports bus-port.chain-device_address
    # (librealsense v2.56.5 src/libusb/enumerator-libusb.cpp:get_device_path).
    # Derive that identifier from the SAME selected USB ancestor. Include the
    # device address to reject stale identifiers after a disconnect/reconnect.
    if not re.fullmatch(r'\d+-\d+(?:\.\d+)*-\d+', physical_port):
        return False
    try:
        root = Path(usb_path)
        bus = int((root / 'busnum').read_text().strip())
        ports = (root / 'devpath').read_text().strip()
        address = int((root / 'devnum').read_text().strip())
    except (OSError, ValueError):
        return False
    return physical_port == f'{bus}-{ports}-{address}'


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


def _capture(namespace, usb_path, routes, commands, quit_event, status_queue):
    """Process boundary bounds SDK failures and isolates capture from robot RPC."""
    from common import logsafe
    logsafe.install(check_fd=False)

    sensor = node = None
    opened = streaming = False
    status = {"frames": {}, "last_frame": {}, "channels": {}, "error": None}
    try:
        import cv2
        import pyrealsense2 as rs
        import rclpy
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import CompressedImage

        context = rs.context()
        devices = [d for d in context.query_devices()
                   if d.supports(rs.camera_info.physical_port)
                   and matches_usb_device(d.get_info(rs.camera_info.physical_port), usb_path)]
        if len(devices) != 1:
            raise RuntimeError("Selected RealSense camera is unavailable")
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
        usb_suffix = hashlib.sha256(usb_path.encode()).hexdigest()[:12]
        node = rclpy.create_node(f"{namespace}_realsense_stereo_{usb_suffix}")
        publishers = {}
        frames = rs.frame_queue(4)
        sensor.open(profiles)
        opened = True
        sensor.start(frames)
        streaming = True
        started_at = time.monotonic()
        last_received = {s: None for s in STREAMS}
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
                    topic = f"/{namespace}/ext_camera/{key.replace('-', '_')}/{channel}"
                    publishers[key] = (channel, node.create_publisher(
                        CompressedImage, topic, qos_profile_sensor_data))
            ok, frame = frames.try_wait_for_frame(200)
            now = time.monotonic()
            if any(t is None for t in last_received.values()):
                if now - started_at >= STARTUP_SECONDS:
                    raise RuntimeError("RealSense depth/infrared startup timed out")
            elif any(now - t >= STALE_SECONDS for t in last_received.values()):
                raise RuntimeError("RealSense depth/infrared frames stopped arriving")
            if not ok:
                continue
            kind = frame.profile.stream_type()
            if kind == rs.stream.depth:
                stream = "depth"
            elif kind == rs.stream.infrared and frame.profile.stream_index() == 1:
                stream = "infrared"
            else:
                continue
            last_received[stream] = now
            if stream not in routes.values():
                continue
            raw = np.asanyarray(frame.get_data())
            msg = CompressedImage()
            msg.header.stamp = node.get_clock().now().to_msg()
            msg.header.frame_id = f"{namespace}_{stream}_optical"
            if stream == "depth":
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
            for key, (channel, pub) in publishers.items():
                if channel == stream:
                    pub.publish(msg)
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
    """One stereo device shared by ext_camera instances bound to its physical USB path."""

    def __init__(self, namespace, usb_path):
        self.namespace, self.usb_path = namespace, usb_path
        self._lock = threading.RLock()
        self._ctx = mp.get_context('spawn')
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
        for q in (self._queue, self._commands):
            if q is not None:
                q.cancel_join_thread()
                q.close()
        self._queue = self._commands = None

    def start(self, instance_id, channel):
        with self._lock:
            self._drain()
            current = self.info(instance_id, channel)
            if self._wanted.get(instance_id) == channel and current['state'] in ('running', 'starting'):
                return current
            self._wanted[instance_id] = channel
            self._requested_at[instance_id] = time.monotonic()
            if (self._proc is None or not self._proc.is_alive()
                    or self._status.get('error') or current['state'] == 'error'):
                self._close()
                self._status = {}
                self._queue = self._ctx.Queue(maxsize=4)
                self._quit = self._ctx.Event()
                self._commands = self._ctx.Queue()
                for s in self._wanted:
                    self._requested_at[s] = time.monotonic()
                self._proc = self._ctx.Process(target=_capture, args=(
                    self.namespace, self.usb_path, dict(self._wanted), self._commands,
                    self._quit, self._queue),
                    name='realman_realsense_stereo', daemon=True)
                self._proc.start()
            else:
                self._commands.put(dict(self._wanted))
            # A start request schedules capture; running requires a published
            # frame. info exposes readiness/errors without blocking HTTP on USB.
            return self.info(instance_id, channel)

    def stop(self, instance_id):
        with self._lock:
            channel = self._wanted.pop(instance_id, 'depth')
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
            last = self._status.get('last_frame', {}).get(instance_id)
            requested = self._requested_at.get(instance_id, now)
            fresh = (last is not None and last >= requested and now - last < STALE_SECONDS
                     and self._status.get('channels', {}).get(instance_id) == channel)
            error = None
            state = 'idle'
            if instance_id in self._wanted:
                error = self._status.get('error')
                if not error and (self._proc is None or not self._proc.is_alive()):
                    error = 'RealSense capture process exited'
                stalled = last is not None and last >= requested and now-last >= STALE_SECONDS
                if not error and (stalled or (not fresh and now-requested >= STARTUP_SECONDS)):
                    error = 'No fresh RealSense frames'
                state = 'error' if error else ('running' if fresh else 'starting')
            return {
                'channel': channel, 'state': state, 'fresh': fresh and state == 'running', 'error': error,
                'device_name': self._status.get('device_name'),
                'width': WIDTH, 'height': HEIGHT, 'fps': self._status.get('fps'),
                'usb_type': self._status.get('usb_type'),
                'encoding': '16UC1' if channel == 'depth' else 'mono8',
                'source_stream': channel,
                'stream_index': 0 if channel == 'depth' else 1,
                'unit': 'mm' if channel == 'depth' else 'intensity',
                'depth_scale_m': self._status.get('depth_scale_m') if channel == 'depth' else None,
                'frames_published': self._status.get('frames', {}).get(instance_id, 0),
                'last_frame_age_s': None if last is None else max(0, now-last),
                'topic_in': [],
                'topic_out': [{'topic': f"/{self.namespace}/ext_camera/{instance_id.replace('-', '_')}/{channel}",
                               'format': FORMATS[channel]}],
            }
