"""PiPER's native RealSense depth stream, in the depth optical frame."""
from contextlib import ExitStack
from functools import partial
import math
import os
from pathlib import Path
import re
import zlib

from common.camera_info import build
from agilex.piper.camera_live import ImageTransport, PiperCameraLive, camera_settings
from agilex.piper.camera_worker import CameraSession, capture_frames


WIDTH, HEIGHT = 640, 480


def depth_settings(config):
    settings = camera_settings(config)
    serial = os.environ.get('PIPER_DEPTH_SERIAL', '').strip()
    device = os.environ.get('PIPER_DEPTH_VIDEO_DEVICE', '').strip()
    source = os.environ.get('PIPER_DEPTH_ROS_SOURCE', '').strip()
    frame_id = os.environ.get('PIPER_DEPTH_ROS_FRAME_ID', '').strip()
    domain = None
    if source:
        if device or not frame_id or not re.fullmatch(r'(/[A-Za-z_][A-Za-z_0-9]*)+', source):
            raise ValueError('Configure one native ROS depth source, its optical frame, and no V4L2 device')
        try:
            domain = int(os.environ.get('PIPER_DEPTH_ROS_DOMAIN_ID', ''))
        except ValueError as exc:
            raise ValueError('Set PIPER_DEPTH_ROS_DOMAIN_ID to the depth source domain') from exc
        if not 0 <= domain <= 232:
            raise ValueError('ROS depth domain must be between 0 and 232')
    return dict(device=device, serial=serial, enabled=bool(serial or device or source),
                width=WIDTH, height=HEIGHT, fps=settings['fps'], warmup_seconds=settings['warmup_seconds'],
                ros_source=source, ros_domain=domain, ros_frame_id=frame_id,
                ros_profile=os.environ.get('PIPER_DEPTH_ROS_DDS_PROFILE', '').strip())


def encode_depth(frame, scale):
    import numpy as np

    if frame.shape != (HEIGHT, WIDTH) or frame.dtype != np.uint16:
        raise ValueError('RealSense depth requires a 640x480 Z16 frame')
    if not math.isfinite(scale) or scale <= 0:
        raise ValueError('RealSense reported an invalid depth scale')
    mm = np.rint(frame.astype(np.float64) * scale * 1000)
    mm[(mm < 1) | (mm > 65535)] = 0  # Zero is invalid depth in the renderer contract.
    return zlib.compress(mm.astype('<u2').tobytes(), 1)


def _intrinsics(profile):
    intr = profile.get_intrinsics()
    fx, fy, cx, cy = intr.fx, intr.fy, intr.ppx, intr.ppy
    coefficients = list(intr.coeffs)
    if fx <= 0 or fy <= 0 or not all(map(math.isfinite, (fx, fy, cx, cy, *coefficients))):
        raise ValueError('RealSense reported invalid depth intrinsics')
    model = str(intr.model).rsplit('.', 1)[-1]
    return dict(K=[fx, 0, cx, 0, fy, cy, 0, 0, 1], D=coefficients,
                source='derived-from-K', distortion_model='pinhole' if model == 'none' else 'unknown',
                vendor={'realsense_distortion_model': model})


def depth_worker(settings, mailbox, stop):
    from common import logsafe
    logsafe.install(check_fd=False)
    try:
        if settings.get('ros_source'):
            from agilex.piper.camera_depth_ros import capture_ros_depth

            capture_ros_depth(settings, mailbox, stop, encode_depth)
            return

        import numpy as np
        import pyrealsense2 as rs

        context = rs.context()
        devices = [dev for dev in context.query_devices()
                   if dev.supports(rs.camera_info.serial_number)
                   and dev.get_info(rs.camera_info.serial_number) == settings['serial']]
        if len(devices) != 1:
            raise RuntimeError('Configured RealSense serial is unavailable. Check the mapped depth devices.')
        device = devices[0]
        usb = device.get_info(rs.camera_info.usb_type_descriptor) if device.supports(rs.camera_info.usb_type_descriptor) else ''
        fps_limit = 15 if usb.startswith('3') else 6  # Leave USB bandwidth for the color stream.
        sensor = device.first_depth_sensor()
        profiles = [profile.as_video_stream_profile() for profile in sensor.get_stream_profiles()
                    if profile.stream_type() == rs.stream.depth and profile.format() == rs.format.z16
                    and 0 < profile.fps() <= fps_limit]
        matching = [p for p in profiles if (p.width(), p.height()) == (WIDTH, HEIGHT)]
        if not matching:
            raise RuntimeError(f'RealSense has no 640x480 Z16 profile at or below {fps_limit} FPS')
        profile = max(matching, key=lambda p: p.fps())
        frames = rs.frame_queue(1)
        with ExitStack() as cleanup:
            sensor.open(profile)
            cleanup.callback(sensor.close)
            scale = sensor.get_depth_scale()
            mailbox.set_metadata(_intrinsics(profile))
            sensor.start(frames)
            cleanup.callback(sensor.stop)

            def read():
                ok, frame = frames.try_wait_for_frame(200)
                return np.asanyarray(frame.get_data()).copy() if ok else None

            capture_frames(settings, mailbox, stop, read, lambda frame, _: encode_depth(frame, scale))
    except Exception as exc:
        mailbox.fail(exc)


class PiperCameraDepth(PiperCameraLive):
    NAME = 'piper_camera_depth'
    TITLE = 'Depth camera'
    DESCRIPTION = 'Live RealSense depth in millimetres on its native depth grid.'
    TOPIC_SUFFIX = '/camera/depth'
    FORMAT = 'image/depth-zlib'
    ENCODING = '16UC1; compressedDepth zlib'
    FRAME_ID = 'piper_camera_depth_optical_frame'
    sensor_card = True
    settings_from_config = staticmethod(depth_settings)
    offline_frame = None

    def __init__(self, runtime, ros2=None, *,
                 session_factory=partial(CameraSession, worker_target=depth_worker, name='piper-depth-camera'),
                 **kwargs):
        super().__init__(runtime, ros2, session_factory=session_factory, **kwargs)

        if self.settings['ros_source'] and kwargs.get('publisher_factory') is None:
            self.publisher_factory = partial(
                ImageTransport, encoding=self.ENCODING, frame_id=self.settings['ros_frame_id'])

    def _enabled(self):
        return self.settings['enabled']

    def _acquire_source(self):
        if self.settings['ros_source']:
            return None
        return super()._acquire_source()

    def _check_device(self):
        device = self.settings['device']
        if not self.settings['serial'] or not re.fullmatch(r'/dev/video[0-9]+', device):
            raise RuntimeError('Configure the RealSense serial and its exact /dev/videoN depth node')
        if not Path(device).is_char_device():
            raise RuntimeError('Mapped RealSense depth device is unavailable')

    def info(self):
        with self._lock:
            result = super().info()
            serial = self.settings['serial']
            camera_id = f'realsense/{serial}/depth' if serial else self.topic
            result['camera_info'] = [build(
                topic=self.topic, id=camera_id,
                format=self.FORMAT, width=WIDTH, height=HEIGHT, **(self._metadata or {}))]
            return result
