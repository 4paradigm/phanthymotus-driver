"""D435i motion pairing for RTAB-Map rotation assistance, not inertial odometry."""
from collections import deque
import math
import uuid

try:
    from .visual_odometry_contract import finite
    from .realsense_imu_probe import transform
except ImportError:
    from visual_odometry_contract import finite
    from realsense_imu_probe import transform


class SharedClock:
    """Validate the shared SDK clock, or map a raw device clock for probes.

    Production uses SDK global time (zero extra offset), including its shared
    drift fit. Ordering is checked independently for each physical stream.
    """
    def __init__(self):
        self.reset()

    def reset(self):
        self.offset = self.domain = None
        self.previous = {}

    def convert(self, stream, source_ms, domain, received_ns):
        source = int(finite([source_ms], 1)[0]*1e6)
        if domain not in ('hardware_clock', 'global_time', 'system_time'):
            raise ValueError('unknown camera/IMU clock domain')
        if self.domain is not None and self.domain != domain:
            raise ValueError('camera/IMU clock domain changed')
        if stream in self.previous and source <= self.previous[stream]:
            raise ValueError('camera/IMU timestamp repeated or reset')
        if self.offset is None:
            self.offset = received_ns-source if domain == 'hardware_clock' else 0
        stamp = source+self.offset
        if not -50_000_000 <= received_ns-stamp <= 250_000_000:
            raise ValueError(f'camera/IMU clock drift or transport stale: age {(received_ns-stamp)/1e6:.3f} ms')
        self.domain = domain
        self.previous[stream] = source
        return stamp


class MotionPairing:
    """Interpolate measured accel brackets onto gyro times, never extrapolate."""
    def __init__(self):
        self.accel = deque(maxlen=32)
        self.gyro = deque(maxlen=32)
        self.previous = {}

    def add(self, kind, stamp, xyz):
        xyz = finite(list(xyz), 3)
        if kind not in ('gyro', 'accel'):
            raise ValueError('unknown motion stream')
        if kind in self.previous and stamp <= self.previous[kind]:
            raise ValueError('motion timestamp repeated or reset')
        self.previous[kind] = stamp
        getattr(self, kind).append((stamp, xyz))
        output = []
        while self.gyro and len(self.accel) >= 2:
            t, w = self.gyro[0]
            if t < self.accel[0][0]:
                self.gyro.popleft()  # no earlier bracket at startup
                continue
            while len(self.accel) > 2 and self.accel[1][0] < t:
                self.accel.popleft()
            a, b = self.accel[0], self.accel[1]
            if t > b[0]:
                break
            if b[0]-a[0] > 25_000_000:
                raise ValueError('camera IMU acceleration gap exceeds 25 ms')
            f = (t-a[0])/(b[0]-a[0])
            output.append((t, w, [x+(y-x)*f for x, y in zip(a[1], b[1])]))
            self.gyro.popleft()
        return output


def matrix_quaternion(matrix):
    """Row-major active rotation to ROS xyzw, including rotations near pi."""
    import numpy as np
    r = np.asarray(matrix, dtype=float)
    if (r.shape != (3, 3) or not np.isfinite(r).all()
            or not np.allclose(r @ r.T, np.eye(3), atol=1e-4)
            or abs(np.linalg.det(r)-1) > 1e-4):
        raise ValueError('invalid IMU rotation')
    a, b, c = r[0]
    d, e, f = r[1]
    g, h, i = r[2]
    k = np.array([[a-e-i, b+d, c+g, h-f], [b+d, e-a-i, f+h, c-g],
                  [c+g, f+h, i-a-e, d-b], [h-f, c-g, d-b, a+e+i]])
    _, vectors = np.linalg.eigh(k)
    q = vectors[:, -1]
    return (q if q[3] >= 0 else -q).tolist()


class CameraImuPublisher:
    """Raw IMU in the gyro physical frame; the SDK supplies the actual mount."""
    def __init__(self, node, namespace, pipeline, qos):
        import pyrealsense2 as rs
        from sensor_msgs.msg import Imu
        self.Imu = Imu
        self.rs = rs
        self.publisher = node.create_publisher(Imu, f'/{namespace}/odometry_input/imu_raw', qos)
        self.clock, self.pairing = SharedClock(), MotionPairing()
        self.frame_id = namespace+'_camera_imu'
        self.enabled = False
        profile = pipeline.get_active_profile()
        gyro, accel = profile.get_stream(rs.stream.gyro), profile.get_stream(rs.stream.accel)
        self.mount = transform(gyro.get_extrinsics_to(profile.get_stream(rs.stream.color)))
        accel_mount = transform(accel.get_extrinsics_to(gyro))
        if math.sqrt(sum(v*v for v in accel_mount['translation_m'])) > .001:
            raise ValueError('gyro/accel origins differ by more than 1 mm')
        self.accel_rotation = accel_mount['rotation_row_major']
        self.restart()

    def restart(self, reason=None):
        self.clock.reset()
        self.pairing = MotionPairing()
        self.epoch = uuid.uuid4().hex
        self.last_stamp, self.count, self.max_gap_ns = None, 0, 0
        self.reset_reason = reason

    def observe(self, frame, received_ns):
        p = frame.get_profile()
        kind = 'gyro' if p.stream_type() == self.rs.stream.gyro else 'accel'
        domain = str(frame.get_frame_timestamp_domain()).rsplit('.', 1)[-1]
        stamp = self.clock.convert(kind, frame.get_timestamp(), domain, received_ns)
        v = frame.as_motion_frame().get_motion_data()
        xyz = [v.x, v.y, v.z]
        if kind == 'accel':
            xyz = [sum(a*b for a, b in zip(row, xyz)) for row in self.accel_rotation]
        for t, w, a in self.pairing.add(kind, stamp, xyz):
            if self.last_stamp is not None:
                gap = t-self.last_stamp
                self.max_gap_ns = max(self.max_gap_ns, gap)
                if gap > 25_000_000:
                    raise ValueError('camera IMU gyro gap exceeds 25 ms')
            self.last_stamp = t
            self.count += 1
            if not self.enabled:
                continue
            msg = self.Imu()
            msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(t, 10**9)
            msg.header.frame_id = self.frame_id
            msg.orientation_covariance[0] = -1.  # orientation comes from Madgwick
            msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = w
            msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z = a
            self.publisher.publish(msg)

    def metadata(self):
        return dict(epoch=self.epoch, frame_id=self.frame_id, stamp_ns=self.last_stamp, samples=self.count,
                    max_gap_ms=self.max_gap_ns/1e6, reset_reason=self.reset_reason, **self.mount)
