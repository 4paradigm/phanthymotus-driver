"""RealSense backend. Native SDK loading is confined to discover/open."""
from common.cameras.interface import ActiveCameraInfo, CameraFrame, physical_usb_id


def realsense_devices(rs):
    result = []
    for dev in rs.context().query_devices():
        profiles = {'rgb': [], 'depth': []}
        for sensor in dev.query_sensors():
            for p in sensor.get_stream_profiles():
                channel = ('rgb' if p.stream_type() == rs.stream.color else
                           'depth' if p.stream_type() == rs.stream.depth else None)
                allowed = (rs.format.rgb8, rs.format.bgr8) if channel == 'rgb' else (rs.format.z16,)
                if channel is None or p.format() not in allowed:
                    continue
                video = p.as_video_stream_profile()
                profile = {'width': video.width(), 'height': video.height(),
                           'fps': p.fps(), 'format': str(p.format()).split('.')[-1]}
                if profile not in profiles[channel]:
                    profiles[channel].append(profile)
        serial = dev.get_info(rs.camera_info.serial_number)
        try:
            physical = physical_usb_id(dev.get_info(rs.camera_info.physical_port))
        except (RuntimeError, AttributeError, KeyError):
            physical = None
        result.append({'physical_id': physical, 'id': f'realsense:{serial}', 'serial': serial,
                       'name': dev.get_info(rs.camera_info.name), 'backend': 'realsense',
                       'capabilities': {c: bool(v) for c, v in profiles.items()},
                       'profiles': profiles})
    return result


def _intrinsics(frame):
    intr = frame.profile.as_video_stream_profile().get_intrinsics()
    return {'width': int(intr.width), 'height': int(intr.height),
            'K': [float(intr.fx), 0., float(intr.ppx), 0., float(intr.fy), float(intr.ppy), 0., 0., 1.],
            'D': [float(v) for v in intr.coeffs], 'sdk_distortion': str(intr.model)}


class Capture:
    def __init__(self, rs, device, mode):
        self._rs, self._intrinsics = rs, {}
        self._pipe, self._started = rs.pipeline(), False
        config = rs.config()
        config.enable_device(device['serial'])
        for channel, p in mode.items():
            config.enable_stream(rs.stream.color if channel == 'rgb' else rs.stream.depth,
                                 p['width'], p['height'], getattr(rs.format, p['format']), p['fps'])
        try:
            active = self._pipe.start(config)
            self._started = True
            actual = {}
            for channel, requested in mode.items():
                stream = active.get_stream(rs.stream.color if channel == 'rgb' else rs.stream.depth)
                video = stream.as_video_stream_profile()
                actual[channel] = {'width': video.width(), 'height': video.height(),
                                   'fps': video.fps(), 'format': str(video.format()).split('.')[-1]}
                if actual[channel] != requested:
                    raise RuntimeError(f'RealSense did not accept native {channel} profile')
            depth_scale = float(active.get_device().first_depth_sensor().get_depth_scale()) if 'depth' in mode else None
            if depth_scale is not None:
                import math
                if not math.isfinite(depth_scale) or depth_scale <= 0:
                    raise RuntimeError('Camera returned invalid depth scale')
            self._info = ActiveCameraInfo(actual, active.get_device().get_info(rs.camera_info.name), depth_scale)
        except Exception:
            self.close()
            raise

    def info(self):
        return self._info

    def read(self, timeout_ms):
        import numpy as np
        try:
            frameset = self._pipe.wait_for_frames(timeout_ms=timeout_ms)
        except RuntimeError:
            # Common worker/session watchdog bounds repeated SDK read timeouts.
            return None
        frames = {}
        for channel, profile in self._info.profiles.items():
            frame = frameset.get_color_frame() if channel == 'rgb' else frameset.get_depth_frame()
            if not frame:
                continue
            if channel not in self._intrinsics:
                self._intrinsics[channel] = _intrinsics(frame)
            raw = np.asanyarray(frame.get_data())
            # All SDK-backed views must be owned before the frameset releases.
            data = raw[:, :, ::-1].copy() if channel == 'rgb' and profile['format'] == 'rgb8' else raw.copy()
            domain = str(frame.get_frame_timestamp_domain()) if hasattr(frame, 'get_frame_timestamp_domain') else 'unknown'
            frames[channel] = CameraFrame(data, self._intrinsics[channel],
                self._info.depth_scale_m if channel == 'depth' else None, float(frame.get_timestamp()), domain)
        return frames or None

    def close(self):
        if self._started:
            self._started = False
            self._pipe.stop()


class Backend:
    id = 'realsense'

    def discover(self):
        import pyrealsense2 as rs
        return realsense_devices(rs)

    def open(self, device, requested_mode):
        import pyrealsense2 as rs
        return Capture(rs, device, requested_mode)


BACKEND = Backend()
