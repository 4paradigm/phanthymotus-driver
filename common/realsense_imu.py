"""Optional motion streams on an existing RealSense pipeline (no ROS imports)."""
import math
import queue
import threading
import time
import uuid


def motion_profiles(rs, device):
    selected = {}
    for sensor in device.query_sensors():
        for profile in sensor.get_stream_profiles():
            kind = profile.stream_type()
            if kind in (rs.stream.gyro, rs.stream.accel) and profile.format() == rs.format.motion_xyz32f:
                if kind not in selected or profile.fps() > selected[kind].fps():
                    selected[kind] = profile
    return selected if len(selected) == 2 else {}


def start_optional_motion(rs, pipeline, config, device, callback):
    """Negotiate discovered XYZ profiles, retrying the same owner with video only."""
    selected = {}
    reason = 'camera motion streams unsupported'
    try:
        selected = motion_profiles(rs, device)
        if selected:
            for kind, profile in selected.items():
                config.enable_stream(kind, profile.stream_index(), 0, 0, profile.format(), profile.fps())
            reason = None
    except Exception as exc:
        reason = 'motion profile negotiation failed: ' + str(exc)
    if reason is not None:
        # Enable failure may have enabled only one stream.
        if selected:
            for kind in (rs.stream.gyro, rs.stream.accel):
                config.disable_stream(kind)
        return pipeline.start(config, callback), reason
    try:
        return pipeline.start(config, callback), None
    except Exception as exc:
        reason = 'motion pipeline start failed: ' + str(exc)
        try:
            pipeline.stop()
        except Exception:
            pass
        for kind in (rs.stream.gyro, rs.stream.accel):
            config.disable_stream(kind)
        return pipeline.start(config, callback), reason


def factory_rotation(profile, depth_profile, *, extrinsics=None):
    if extrinsics is None:
        extrinsics = profile.get_extrinsics_to(depth_profile)
    values = list(extrinsics.rotation)
    if len(values) != 9 or not all(math.isfinite(v) for v in values):
        raise ValueError('invalid factory motion rotation')
    # rs2_extrinsics.rotation is column-major, R_DEPTH_MOTION.
    r = [[float(values[3*j+i]) for j in range(3)] for i in range(3)]
    if any(abs(sum(r[k][i]*r[k][j] for k in range(3)) - (i == j)) > 1e-4
           for i in range(3) for j in range(3)):
        raise ValueError('nonorthonormal factory motion rotation')
    det = sum(r[0][i]*(r[1][(i+1)%3]*r[2][(i+2)%3]-r[1][(i+2)%3]*r[2][(i+1)%3]) for i in range(3))
    if abs(det-1.) > 1e-4:
        raise ValueError('improper factory motion rotation')
    return r


class MotionSource:
    """Publish raw gyro/accel with actual factory axes and the RGB-D time map."""
    def __init__(self, topic, camera_frame, rs, map_time, publish, *, clock_id):
        self.topic, self.camera_frame, self.rs = topic, camera_frame, rs
        self.map_time, self.publish, self.clock_id = map_time, publish, clock_id
        self.lock = threading.RLock()
        self.configured = False
        self.reason = 'waiting for camera motion profiles'
        self.last_capture_ns = 0
        self.clock_mapping = None
        self.translations = {}
        self.translation_reason = 'waiting for factory motion translation'

    def configure(self, active):
        with self.lock:
            self.configured = False
            self.translations = {}
            self.translation_reason = 'waiting for factory motion translation'
            self.accel = None
            self.previous = {}
            self.last_capture_ns = 0
            self.reason = 'waiting for paired motion captures'
            self.epoch = uuid.uuid4().hex
            profiles = {kind: active.get_stream(kind) for kind in (self.rs.stream.gyro, self.rs.stream.accel)}
            depth = active.get_stream(self.rs.stream.depth)
            extrinsics = {kind: p.get_extrinsics_to(depth) for kind, p in profiles.items()}
            self.rotations = {kind: factory_rotation(p, depth, extrinsics=extrinsics[kind])
                              for kind, p in profiles.items()}
            try:
                translations = {kind: list(value.translation) for kind, value in extrinsics.items()}
                if any(len(values) != 3 or not all(math.isfinite(v) for v in values)
                       for values in translations.values()):
                    raise ValueError('invalid factory motion translation')
                self.translations = {kind: [float(v) for v in values] for kind, values in translations.items()}
                self.translation_reason = None
            except (AttributeError, TypeError, ValueError):
                self.translation_reason = 'factory motion translation missing or invalid'
            serial = active.get_device().get_info(self.rs.camera_info.serial_number)
            self.source_id = 'realsense/' + serial + '/motion'
            self.frames = {kind: f'{self.source_id}/{kind}/{p.stream_index()}/{p.unique_id()}'
                           for kind, p in profiles.items()}
            self.configured = True

    def unavailable(self, reason):
        with self.lock:
            self.reason = str(reason)
            self.accel = None

    def accept(self, frame):
        with self.lock:
            if not self.configured:
                return None
            motion = frame.as_motion_frame()
            kind = motion.profile.stream_type()
            if kind not in self.rotations:
                return None
            domain = str(motion.get_frame_timestamp_domain()).rsplit('.', 1)[-1]
            stamp_ms = float(motion.get_timestamp())
            if not math.isfinite(stamp_ms) or stamp_ms <= 0:
                raise ValueError('invalid motion capture time')
            if stamp_ms <= self.previous.get(kind, 0.):
                raise ValueError('motion capture clock reset or repeated')
            stamp = self.map_time(stamp_ms, domain)
            if not isinstance(stamp, int) or stamp <= 0:
                raise ValueError('RGB-D capture clock mapping unavailable')
            data = motion.get_motion_data()
            values = [float(data.x), float(data.y), float(data.z)]
            if not all(math.isfinite(v) for v in values):
                raise ValueError('nonfinite motion vector')
            self.previous[kind] = stamp_ms
            if kind == self.rs.stream.accel:
                self.accel = (stamp, values)
                return None
            if self.accel is None or abs(stamp-self.accel[0]) > 20_000_000:
                raise ValueError('waiting for accel within 20 ms of gyro')
            clock = self.clock_id()
            if not clock:
                raise ValueError('RGB-D clock identity unavailable')
            packet = dict(schema='motus.imu/1', source_id=self.source_id, epoch=self.epoch,
                          stamp_ns=stamp, accel_stamp_ns=self.accel[0], stamp_source='measurement',
                          clock_id=clock, camera_frame_id=self.camera_frame,
                          gyro_frame_id=self.frames[self.rs.stream.gyro], accel_frame_id=self.frames[self.rs.stream.accel],
                          camera_from_gyro=self.rotations[self.rs.stream.gyro],
                          camera_from_accel=self.rotations[self.rs.stream.accel],
                          rotation_source='sdk_factory_extrinsics', angular_velocity=values,
                          linear_acceleration=self.accel[1],
                          units=dict(angular_velocity='rad/s', linear_acceleration='m/s^2'))
            if self.translations:
                packet.update(camera_from_gyro_translation=self.translations[self.rs.stream.gyro],
                              camera_from_accel_translation=self.translations[self.rs.stream.accel],
                              translation_source='sdk_factory_extrinsics')
            if self.clock_mapping is not None:
                packet['clock_mapping'] = dict(self.clock_mapping)
            self.last_capture_ns, self.reason = stamp, None
            return packet

    def on_frame(self, frame):
        """A pipeline callback may deliver motion alone or inside a composite."""
        try:
            if frame.is_motion_frame():
                packet = self.accept(frame)
                if packet is not None:
                    self.publish(packet)
            elif frame.is_frameset():
                for child in frame.as_frameset():
                    if child.is_motion_frame():
                        self.on_frame(child)
        except Exception as exc:
            self.unavailable(exc)

    def status(self):
        with self.lock:
            live = self.reason is None and -50_000_000 <= time.time_ns()-self.last_capture_ns <= 500_000_000
            return dict(state='ready' if live else 'unavailable',
                        reason=self.reason if self.reason else (None if live else 'motion capture stale'),
                        last_capture_ns=self.last_capture_ns,
                        fusion=dict(state='unavailable' if self.translation_reason else 'available',
                                    **({'reason': self.translation_reason} if self.translation_reason else {})))


class CaptureQueue:
    """Bound only the video framesets; motion is handled at SDK callback rate."""
    def __init__(self, motion):
        self.motion = motion
        self.frames = queue.Queue(maxsize=1)

    def __call__(self, frame):
        self.motion.on_frame(frame)
        if frame.is_frameset():
            fs = frame.as_frameset()
            if fs.get_color_frame() and fs.get_depth_frame():
                try:
                    self.frames.get_nowait()
                except queue.Empty:
                    pass
                try:
                    self.frames.put_nowait(fs)
                except queue.Full:
                    pass

    def wait_for_frames(self, timeout_ms=1000):
        return self.frames.get(timeout=timeout_ms/1000.)
