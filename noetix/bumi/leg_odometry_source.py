"""Standalone Bumi leg kinematics for a generic odometry consumer.

Joint positions provide translation; the SDK orientation rotates the foot
geometry and supplies attitude. There is no acceleration integration, visual
subscription, learned multiplier or motion command in this source.
"""
from __future__ import annotations

from copy import deepcopy
import math
import uuid

from common.odom import build_sample

try:
    from .leg_odometry import LegOdometry, joint_angles, rotation_from_quaternion
except ImportError:  # The Bumi image copies these modules directly into /work.
    from leg_odometry import LegOdometry, joint_angles, rotation_from_quaternion


MAX_SAMPLE_AGE_NS = 250_000_000
MAX_INTERVAL_S = 0.1


def _vector(values, length, name):
    try:
        if isinstance(values, (str, bytes)) or len(values) != length:
            raise ValueError
        if any(isinstance(value, bool) for value in values):
            raise ValueError
        result = [float(value) for value in values]
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError(f'{name} must contain {length} finite numbers') from exc
    if not all(math.isfinite(value) for value in result):
        raise ValueError(f'{name} must contain {length} finite numbers')
    return result


class LegOdometrySource:
    """ROS-free state; the caller serializes access from its own callbacks.

    ``observe`` returns true only when the new read yields a valid pose and
    velocity. A baseline read returns false. Invalid sensor values invalidate
    the current output and raise ValueError. SDK exceptions can use
    ``invalidate`` directly. Publication never changes a sample's read time.
    """

    def __init__(self, namespace, child_frame_id='base_link'):
        self.namespace = str(namespace).strip('/').replace('/', '_') or 'bumi'
        self.child_frame_id = str(child_frame_id)
        if not self.child_frame_id:
            raise ValueError('leg odometry child_frame_id must be nonempty')
        self.reset()

    def reset(self):
        """New independent reference frame; never retain a previous foot step."""
        self.session_id = uuid.uuid4().hex
        self.frame_id = f'{self.namespace}_leg_odom_{self.session_id}'
        # Existing locomotion retains its own configured slip corrections.
        # Visual and scale correction belong to the generic consumer.
        self.leg = LegOdometry(slip_scale=1.0, max_step_velocity_m_s=1.0)
        self._last_stamp_ns = None
        self._last_monotonic_s = None
        self._needs_baseline = False
        self._pose = None
        self._twist = None
        self._status = 'initializing'
        self._reason = 'waiting for leg samples'
        self.sample_count = 0
        self.accepted_count = 0
        # Diagonal process approximation in m^2 and rad^2. The positive initial
        # variances cover a finite foot/contact and SDK attitude uncertainty.
        self._variance = [0.03**2, 0.03**2, 0.05**2,
                          math.radians(3.0)**2, math.radians(3.0)**2,
                          math.radians(5.0)**2]

    def invalidate(self, reason):
        """Immediately withdraw the last estimate without refreshing its time."""
        self._pose = None
        self._twist = None
        self._status = 'lost'
        self._reason = str(reason)
        self._needs_baseline = True

    def observe(self, joints, quaternion, angular_velocity, *, stamp_ns, monotonic_s):
        """Read-time pose in SDK world axes and twist in the current body axes."""
        try:
            if isinstance(stamp_ns, bool) or not isinstance(stamp_ns, int) or stamp_ns <= 0:
                raise ValueError('leg odometry stamp_ns must be a positive integer')
            if isinstance(monotonic_s, bool):
                raise ValueError('leg odometry monotonic_s must be finite')
            monotonic_s = float(monotonic_s)
            if not math.isfinite(monotonic_s):
                raise ValueError('leg odometry monotonic_s must be finite')
            orientation = _vector(quaternion, 4, 'quaternion')
            norm = math.hypot(*orientation)
            if not math.isfinite(norm) or norm < 1e-12:
                raise ValueError('leg odometry quaternion must have nonzero norm')
            orientation = [value / norm for value in orientation]
            angular = _vector(angular_velocity, 3, 'angular_velocity')
            joint_angles(joints)
        except (TypeError, ValueError, OverflowError, AttributeError) as exc:
            self.invalidate(str(exc))
            raise ValueError(str(exc)) from exc

        previous_mono = self._last_monotonic_s
        previous_stamp = self._last_stamp_ns
        discontinuity = previous_mono is not None and (
            not 0.0 < monotonic_s - previous_mono <= MAX_INTERVAL_S
            or not 0 < stamp_ns - previous_stamp <= int(MAX_INTERVAL_S * 1e9)
        )
        if self._needs_baseline or discontinuity:
            self.reset()
            previous_mono = None

        previous_position = list(self.leg.position)
        previous_stance = self.leg.stance_samples
        try:
            self.leg.observe(joints, orientation, monotonic_s)
        except (TypeError, ValueError, OverflowError, AttributeError) as exc:
            self.invalidate(str(exc))
            raise ValueError(str(exc)) from exc
        self._last_stamp_ns = stamp_ns
        self._last_monotonic_s = monotonic_s
        self.sample_count += 1

        if previous_mono is None:
            self._reason = 'establishing leg baseline'
            return False
        if self.leg.stance_samples == previous_stance:
            # LegOdometry uses zero velocity internally for rejected contact
            # steps. Such a step is not a measured standstill. Rebase before
            # resuming so its missing displacement cannot silently bridge frames.
            self.invalidate('leg support step unavailable')
            return False

        dt = monotonic_s - previous_mono
        distance = math.dist(self.leg.position, previous_position)
        angular_distance = math.hypot(*angular) * dt
        # Translational random walk grows with elapsed time and travelled
        # distance (variance per second / metre); attitude grows with time and
        # angular travel. These remain nonzero while stationary and never shrink
        # without an external correction, which belongs to the consumer.
        for axis in range(3):
            self._variance[axis] += (0.01 if axis < 2 else 0.02)**2 * dt
            self._variance[axis] += (0.20 if axis < 2 else 0.30)**2 * distance
        for axis in range(3, 6):
            self._variance[axis] += math.radians(0.5)**2 * dt
            self._variance[axis] += math.radians(1.0)**2 * angular_distance
        covariance = [0.0] * 36
        for axis, variance in enumerate(self._variance):
            covariance[axis * 7] = variance

        rotation = rotation_from_quaternion(orientation)
        body_velocity = [sum(rotation[row][axis] * self.leg.velocity[row]
                             for row in range(3)) for axis in range(3)]
        self._pose = dict(position=list(self.leg.position), quaternion=orientation,
                          frame_id=self.frame_id, child_frame_id=self.child_frame_id,
                          covariance=covariance)
        self._twist = [*body_velocity, *angular]
        self._status = 'tracking'
        self._reason = None
        self.accepted_count += 1
        return True

    def sample(self, now_ns):
        """Return a fresh packet, retaining the last actual SDK read timestamp."""
        status, reason = self._status, self._reason
        valid = self._pose is not None
        if self._last_stamp_ns is not None:
            age_ns = now_ns - self._last_stamp_ns
            if age_ns < 0 or age_ns > MAX_SAMPLE_AGE_NS:
                valid = False
                status = 'lost'
                reason = 'leg sample is in the future' if age_ns < 0 else 'leg sample stale'
        return build_sample(
            stamp_ms=(self._last_stamp_ns or 0) // 1_000_000,
            twist=list(self._twist) if valid else [None] * 6,
            frame='body', pose=deepcopy(self._pose) if valid else None,
            vendor=dict(source='bumi_leg_kinematics', status=status, reason=reason,
                        segment_id=self.session_id),
        )

    def info(self):
        """Acquisition state; use sample(now_ns) for publication-time freshness."""
        return dict(status=self._status, reason=self._reason,
                    session_id=self.session_id, frame_id=self.frame_id,
                    child_frame_id=self.child_frame_id,
                    sample_count=self.sample_count, accepted_count=self.accepted_count,
                    last_stamp_ns=self._last_stamp_ns)
