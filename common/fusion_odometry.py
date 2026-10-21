"""ROS-free contracts for the Bumi RGB-D/body-IMU sidecar.

motus.odom/1 stays unchanged. Fusion-specific quality and clock provenance live
in vendor. A host SDK read is never represented as a source sampling timestamp.
"""
from __future__ import annotations
import math
from collections import deque
from copy import deepcopy
from common.odom import build_sample


def finite(values, size):
    if not isinstance(values, (list, tuple)) or len(values) != size:
        raise ValueError('wrong vector size')
    if any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) for v in values):
        raise ValueError('vector must contain finite numbers')
    return [float(v) for v in values]


def quat_matrix(q):
    x, y, z, w = finite(q, 4)
    n = math.hypot(x, y, z, w)
    if not .99 <= n <= 1.01:
        raise ValueError('quaternion must be normalized')
    x, y, z, w = x/n, y/n, z/n, w/n
    return [[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
            [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
            [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]]


def rotate(r, v):
    return [sum(r[i][j]*v[j] for j in range(3)) for i in range(3)]


def camera_to_body(position, quaternion, mount):
    """T_O_B = T_O_C inverse(T_B_C), including the rotating lever arm."""
    rc = quat_matrix(quaternion)
    rm = quat_matrix(mount['quaternion'])
    rb = [[sum(rc[i][k]*rm[j][k] for k in range(3)) for j in range(3)] for i in range(3)]
    lever = rotate(rb, finite(mount['translation'], 3))
    return [p-l for p, l in zip(finite(position, 3), lever)], rb


def body_velocity(camera_velocity_body, angular_body, lever_body):
    """v_B = v_C - omega cross r_BC; all vectors expressed in B."""
    x, y, z = angular_body; a, b, c = lever_body
    cross = [y*c-z*b, z*a-x*c, x*b-y*a]
    return [v-w for v, w in zip(camera_velocity_body, cross)]


class Calibration:
    def __init__(self, data):
        self.data = deepcopy(data)
        try:
            self.id = data['calibration_id']
            if not isinstance(self.id, str) or not self.id or data['fixed_mount'] is not True:
                raise ValueError('measured fixed installation required')
            for name in ('camera_to_body', 'imu_to_body'):
                finite(data[name]['translation'], 3)
                quat_matrix(data[name]['quaternion'])
            if (data['imu_units'] != 'm/s2,rad/s' or
                    data['imu_quaternion'] != 'xyzw_sensor_to_enu' or data['gravity_included'] is not True):
                raise ValueError('IMU SI, sensor-to-ENU xyzw and gravity convention must be verified')
            finite(data['gyro_bias'], 3)
            if min(finite(data['gyro_variance'], 3)) <= 0:
                raise ValueError('measured gyro variance required')
            timing = data['timing']
            if timing['mode'] != 'host_read_qualified' or not timing['qualification_id']:
                raise ValueError('host-read timing qualification required')
            for name in ('offset_s', 'uncertainty_s', 'max_age_s'):
                finite([timing[name]], 1)
            if not 0 <= timing['uncertainty_s'] <= .05 or not 0 < timing['max_age_s'] <= .5:
                raise ValueError('unqualified timing uncertainty or age')
            if abs(timing['offset_s']) > .5:
                raise ValueError('IMU read delay outside supported history')
        except (KeyError, TypeError) as exc:
            raise ValueError('incomplete measured calibration: '+str(exc)) from exc


class ClockMap:
    """Map a device clock to ROS wall time; detect domain/clock discontinuity.

    An arrival-based hardware-clock offset is an approximation, exposed in
    metadata. It is not exposure synchronization or proof of device freshness.
    """
    def __init__(self):
        self.domain = self.offset = self.previous = None

    def convert(self, source_ms, domain, received_ns):
        finite([source_ms], 1)
        if domain not in ('hardware_clock', 'system_time', 'global_time'):
            raise ValueError('unrecognized camera timestamp domain')
        source = int(source_ms*1e6)
        if self.domain is not None and (domain != self.domain or source <= self.previous):
            raise ValueError('camera clock reset or domain changed')
        if self.offset is None:
            self.offset = received_ns-source if domain == 'hardware_clock' else 0
        result = source+self.offset
        if not -50_000_000 <= received_ns-result <= 250_000_000:
            raise ValueError('camera clock mapping drift or transport delay')
        self.domain, self.previous = domain, source
        return result


class GravityWindow:
    def __init__(self):
        self.rows = deque()

    def observe(self, stamp_ns, acceleration, angular):
        finite(acceleration, 3); finite(angular, 3)
        if (not 8 <= math.hypot(*acceleration) <= 11 or math.hypot(*angular) > .08 or
                (self.rows and stamp_ns <= self.rows[-1][0])):
            self.rows.clear()
            return None
        self.rows.append((stamp_ns, acceleration))
        while len(self.rows) > 1 and stamp_ns-self.rows[1][0] >= 2_000_000_000:
            self.rows.popleft()
        if len(self.rows) < 30 or stamp_ns-self.rows[0][0] < 2_000_000_000:
            return None
        if any(max(row[1][i] for row in self.rows)-min(row[1][i] for row in self.rows) > .25 for i in range(3)):
            return None
        x, y, z = [sum(row[1][i] for row in self.rows)/len(self.rows) for i in range(3)]
        return math.atan2(y, z), math.atan2(-x, math.hypot(y, z))


class Health:
    kinds = ('camera', 'imu', 'visual', 'diagnostic', 'filtered')

    def __init__(self, segment, calibration_id, started_ns):
        self.segment, self.calibration_id, self.started_ns = segment, calibration_id, started_ns
        self.samples, self.counts = {}, {}
        self.pairs = {'visual': deque(maxlen=20), 'diagnostic': deque(maxlen=20)}
        self.reason = None
        self.last_clock = None
        self.details = {}

    def fault(self, reason):
        self.reason = self.reason or str(reason)

    def observe(self, kind, stamp_ns, now_ns, valid, reason=None):
        if kind not in self.kinds:
            raise ValueError('unknown health input')
        if not valid:
            self.fault(reason or ('invalid '+kind))
            return
        if stamp_ns < self.started_ns:
            return  # prior segment queued DDS traffic
        previous = self.samples.get(kind)
        if (not -50_000_000 <= now_ns-stamp_ns <= 500_000_000 or
                (previous is not None and stamp_ns <= previous)):
            self.fault('stale, repeated or reset '+kind)
        self.samples[kind] = stamp_ns
        self.counts[kind] = self.counts.get(kind, 0)+1
        if kind in self.pairs:
            self.pairs[kind].append(stamp_ns)

    def status(self, now_ns):
        if self.last_clock is not None and now_ns < self.last_clock:
            self.fault('ROS clock moved backwards')
        self.last_clock = now_ns
        if any(now_ns-stamp > 500_000_000 for stamp in self.samples.values()):
            self.fault('input stale')
        ready = all(self.counts.get(k, 0) >= 3 for k in self.kinds)
        paired = set(self.pairs['visual']) & set(self.pairs['diagnostic'])
        coherent = bool(paired and now_ns-max(paired) <= 500_000_000 and
                        self.samples.get('filtered', 0) >= max(paired))
        state = 'lost' if self.reason else ('tracking' if ready and coherent else 'initializing')
        return dict(status=state, reason=self.reason or (None if state == 'tracking' else 'waiting for synchronized inputs'),
                    segment_id=self.segment, calibration_id=self.calibration_id,
                    input_age_ms={k: (now_ns-v)/1e6 for k, v in self.samples.items()},
                    visual_stamp_ns=max(paired) if paired else None)


def make_sample(health, now_ns, estimate):
    quality = health.status(now_ns)
    valid = quality['status'] == 'tracking' and estimate is not None
    return build_sample(stamp_ms=(estimate['stamp_ns'] if valid else now_ns)//1_000_000,
                        twist=estimate['twist'] if valid else [None]*6,
                        pose=deepcopy(estimate['pose']) if valid else None,
                        vendor={**quality, 'feedback_source': 'rgbd_imu_ekf', 'accuracy_validated': False,
                                'source_timestamp_available': False,
                                'imu_stamp_source': 'host_read_qualified',
                                'published_ns': now_ns, **deepcopy(health.details)})
