"""ROS-free contracts for the Bumi bundle-local RGB-D visual odometry.

This is the visual-only half that PR #425 shipped, kept without the body-IMU
fusion: there is no IMU input, no robot_localization EKF, no gyro calibration and
no gravity window. RTAB-Map ``rgbd_odometry`` is the only estimator, so the
quality contract is "the camera tracker produced a pose and its own diagnostics
agree with it" and nothing more.

``motus.odom/1`` stays unchanged. Everything visual-specific lives in ``vendor``:
segment, diagnostic counters, input ages and the frame the pose is expressed in.
The pose is in the camera optical frame by default because no measured
camera-to-body mount exists yet; a measured ``camera_to_body`` may be configured
to report the same pose at the body reference point.
"""
from __future__ import annotations
import math
from collections import deque
from copy import deepcopy

from common.odom import build_sample

# Bounds used to refuse input, not to certify accuracy. They are the same
# acceptance tests PR #425 used for the shared raw streams.
MAX_SKEW_NS = 20_000_000            # RGB/depth capture-time difference
MAX_TRANSPORT_NS = 250_000_000      # publisher -> subscriber age
MAX_INPUT_AGE_NS = 500_000_000      # a stream older than this is stale
MIN_INLIERS = 10                    # rtabmap OdomInfo inlier floor
MAX_POSITION_SIGMA_M = 0.2          # refuse obviously broken visual covariance


def finite(values, size):
    """A fixed-length list of finite numbers, or ``ValueError``."""
    if not isinstance(values, (list, tuple)) or len(values) != size:
        raise ValueError('wrong vector size')
    if any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v)
           for v in values):
        raise ValueError('vector must contain finite numbers')
    return [float(v) for v in values]


def quat_matrix(q):
    """Rotation matrix for a normalized xyzw quaternion."""
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
    """Express an optical-frame pose at the body reference point.

    ``mount`` carries the measured ``translation`` (camera origin in body
    coordinates) and ``quaternion`` (optical C to body B, normalized xyzw).
    Returns ``(position, quaternion)``. This is a rigid change of reference
    point, not a fusion step: the same visual trajectory is being described from
    a different point, with the lever arm included while the body turns.
    """
    rc = quat_matrix(quaternion)
    rm = quat_matrix(mount['quaternion'])
    # R_OB = R_OC * R_CB, with mount = R_BC.
    rb = [[sum(rc[i][k]*rm[j][k] for k in range(3)) for j in range(3)] for i in range(3)]
    lever = rotate(rb, finite(mount['translation'], 3))
    body_position = [p-l for p, l in zip(finite(position, 3), lever)]
    # q_OB = q_OC * inverse(q_BC); quaternion product keeps xyzw order.
    x1, y1, z1, w1 = finite(quaternion, 4)
    x2, y2, z2, w2 = finite(mount['quaternion'], 4)
    inverse = [-x2, -y2, -z2, w2]
    out = [w1*inverse[0] + x1*inverse[3] + y1*inverse[2] - z1*inverse[1],
           w1*inverse[1] - x1*inverse[2] + y1*inverse[3] + z1*inverse[0],
           w1*inverse[2] + x1*inverse[1] - y1*inverse[0] + z1*inverse[3],
           w1*inverse[3] - x1*inverse[0] - y1*inverse[1] - z1*inverse[2]]
    return body_position, out


class CameraCalibration:
    """The optional measured camera mount; there is no IMU contract here."""

    def __init__(self, data):
        self.data = deepcopy(data)
        try:
            self.id = data['calibration_id']
            if not isinstance(self.id, str) or not self.id or data['fixed_mount'] is not True:
                raise ValueError('measured fixed installation required')
            finite(data['camera_to_body']['translation'], 3)
            quat_matrix(data['camera_to_body']['quaternion'])
        except (KeyError, TypeError) as exc:
            raise ValueError('incomplete measured camera mount: '+str(exc)) from exc


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
        if not -50_000_000 <= received_ns-result <= MAX_TRANSPORT_NS:
            raise ValueError('camera clock mapping drift or transport delay')
        self.domain, self.previous = domain, source
        return result


class Health:
    """initializing -> tracking, with lost latched for the rest of the segment.

    No filtered/EKF input exists in this contract: a pose that keeps predicting
    while the camera stopped is exactly the failure this state machine refuses.
    """

    kinds = ('camera', 'visual', 'diagnostic')

    def __init__(self, segment, calibration_id, started_ns):
        self.segment, self.calibration_id, self.started_ns = segment, calibration_id, started_ns
        self.samples, self.counts = {}, {}
        self.pairs = {'visual': deque(maxlen=20), 'diagnostic': deque(maxlen=20)}
        self.reason = None
        self.last_clock = None
        self.last_diagnostic = None
        self.details = {}

    def fault(self, reason):
        self.reason = self.reason or str(reason)

    def observe(self, kind, stamp_ns, now_ns, valid, reason=None):
        if kind not in self.kinds:
            raise ValueError('unknown visual-odometry health input')
        if not valid:
            self.fault(reason or ('invalid '+kind))
            return
        if stamp_ns < self.started_ns:
            return  # prior segment queued DDS traffic
        previous = self.samples.get(kind)
        if (not -50_000_000 <= now_ns-stamp_ns <= MAX_INPUT_AGE_NS or
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
        if any(now_ns-stamp > MAX_INPUT_AGE_NS for stamp in self.samples.values()):
            self.fault('input stale')
        ready = (self.counts.get('camera', 0) >= 3 and self.counts.get('visual', 0) >= 3)
        paired = set(self.pairs['visual']) & set(self.pairs['diagnostic'])
        coherent = bool(paired) and now_ns-max(paired) <= MAX_INPUT_AGE_NS
        state = 'lost' if self.reason else ('tracking' if ready and coherent else 'initializing')
        return dict(status=state,
                    reason=self.reason or (None if state == 'tracking'
                                           else 'waiting for paired RGB-D visual odometry'),
                    segment_id=self.segment, calibration_id=self.calibration_id,
                    input_age_ms={k: (now_ns-v)/1e6 for k, v in self.samples.items()},
                    visual_stamp_ns=max(paired) if paired else None)


def make_sample(health, now_ns, estimate, pose_reference='rgb_optical'):
    """One ``motus.odom/1`` sample for the visual odometry sensor.

    The visual pose is reported only in a tracking segment. ``twist`` is all
    ``None`` on purpose: RTAB-Map's odometry message carries a trajectory-frame
    delta, not a measured body velocity, and inventing one would be worse than
    saying it is not measured.
    """
    quality = health.status(now_ns)
    valid = quality['status'] == 'tracking' and estimate is not None
    return build_sample(
        stamp_ms=(estimate['stamp_ns'] if valid else now_ns)//1_000_000,
        twist=[None]*6,
        # ``frame`` names the twist frame. There is no twist; the pose below
        # states its own odom/child frames and pose_reference says which point.
        frame='body',
        pose=deepcopy(estimate['pose']) if valid else None,
        vendor={**quality, 'source': 'rtabmap_rgbd_odometry',
                'pose_reference': pose_reference,
                'accuracy_validated': False,
                'twist_source': None,
                'published_ns': now_ns, **deepcopy(health.details)})
