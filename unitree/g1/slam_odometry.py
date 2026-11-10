"""G1 mapping Odometry conversion; unproven SLAM evidence stays unavailable.

Source contract: https://support.unitree.com/home/en/G1_developer/slam_navigation_services_interface
The source is ROS Odometry (meters, named xyzw quaternion components). Its
header is preserved, but the public G1 contract does not establish measurement
clock semantics or per-pose tracking/reset identity. No configuration flag can
promote these samples to tracking. Raw covariance is evidence, not a claim of
estimator uncertainty; it is retained under vendor rather than pose.
"""
import math
import time
from uuid import uuid4

from common.odom import DEFAULT_UNITS, build_interface
from common.slam_input import input_declaration


SOURCE_TOPIC = 'rt/unitree/slam_mapping/odom'
MISSING_FACTS = (
    'measurement_stamp', 'clock_domain_and_epoch', 'clock_relation_to_lidar',
    'tracking_quality', 'segment_identity', 'tracking_reset_and_relocalization',
    'sensor_extrinsic',
)


def _numbers(values):
    if any(isinstance(v, bool) or not isinstance(v, (int, float)) or
           not math.isfinite(v) for v in values):
        raise ValueError('non_finite_or_non_numeric_pose')
    return [float(v) for v in values]


class MappingOdometry:
    def __init__(self):
        # This is an adapter-process generation, NEVER a robot clock epoch
        # or a verified odometry segment identifier.
        self.source_generation = uuid4().hex
        self._last_stamp_ns = None
        self._frames = None
        self._last_received_monotonic_ns = None
        self._accepted_samples = 0
        self._rejected_samples = 0
        self._possible_discontinuities = 0
        self._continuity_latched = False
        self._last_rejection = None
        self._observation_state = 'waiting'

    def observation_status(self, *, now_monotonic_ns=None, stale_after_s=2.0):
        """Diagnostic receipt health, not a SLAM tracking or clock assertion.

        A receipt gap alone does not prove the robot reset or relocalized.
        Once a timestamp rollback or source-frame change is observed, the
        candidate stream is quarantined until a new adapter generation.
        """
        if (isinstance(stale_after_s, bool) or not isinstance(stale_after_s, (int, float))
                or not math.isfinite(stale_after_s) or stale_after_s <= 0):
            raise ValueError('stale_after_s must be positive and finite')
        if now_monotonic_ns is None:
            now_monotonic_ns = time.monotonic_ns()
        if type(now_monotonic_ns) is not int or now_monotonic_ns < 0:
            raise ValueError('invalid monotonic observation time')
        last = self._last_received_monotonic_ns
        age_s = max(0, now_monotonic_ns - last) / 1e9 if last is not None else None
        stale = age_s > stale_after_s if age_s is not None else None
        state = ('waiting' if last is None else
                 'stale_observation' if stale else self._observation_state)
        return dict(available=False, state=state, receipt_age_s=age_s,
                    receipt_stale=stale, last_source_stamp_ns=self._last_stamp_ns,
                    last_accepted_frames=list(self._frames) if self._frames else None,
                    accepted_samples=self._accepted_samples,
                    rejected_samples=self._rejected_samples,
                    possible_discontinuities=self._possible_discontinuities,
                    continuity_quarantined=self._continuity_latched,
                    last_rejection=self._last_rejection,
                    source_generation=self.source_generation,
                    clock_epoch=None, segment_id=None)

    def convert(self, msg):
        received_monotonic_ns = time.monotonic_ns()
        self._last_received_monotonic_ns = received_monotonic_ns
        quality = dict(status='unavailable', stamp_source=None, clock_id=None,
                       clock_epoch=None, segment_id=None, source_stamp_ns=None,
                       source_generation=self.source_generation,
                       missing_facts=list(MISSING_FACTS), rejection=None)
        result = dict(schema='motus.odom/1', stamp_ms=None, units=dict(DEFAULT_UNITS),
                      frame='world', twist=[None]*6,
                      pose=None, vendor=dict(source_topic=SOURCE_TOPIC, motus_slam=quality))
        try:
            sec, nsec = msg.header.stamp.sec, msg.header.stamp.nanosec
            if (type(sec) is not int or sec < 0 or type(nsec) is not int or
                    not 0 <= nsec < 1_000_000_000 or sec * 1_000_000_000 + nsec < 1_000_000):
                raise ValueError('invalid_source_stamp')
            stamp_ns = sec * 1_000_000_000 + nsec
            quality['source_stamp_ns'] = stamp_ns
            result['stamp_ms'] = stamp_ns // 1_000_000
            frames = (msg.header.frame_id, msg.child_frame_id)
            if any(not isinstance(f, str) or not f.strip() for f in frames):
                raise ValueError('missing_source_frames')
            p, q = msg.pose.pose.position, msg.pose.pose.orientation
            position = _numbers([p.x, p.y, p.z])
            quaternion = _numbers([q.x, q.y, q.z, q.w])
            if abs(sum(v*v for v in quaternion) - 1.) > .002:
                raise ValueError('non_unit_quaternion')
            covariance = list(msg.pose.covariance)
            if len(covariance) != 36:
                raise ValueError('invalid_source_covariance_length')
            result['vendor']['source_covariance'] = _numbers(covariance)
            if self._last_stamp_ns is not None:
                if stamp_ns == self._last_stamp_ns:
                    raise ValueError('duplicate_source_stamp')
                if stamp_ns < self._last_stamp_ns:
                    raise ValueError('out_of_order_source_stamp')
                if stamp_ns // 1_000_000 == self._last_stamp_ns // 1_000_000:
                    raise ValueError('duplicate_profile_stamp')
            if self._frames is not None and frames != self._frames:
                raise ValueError('source_frames_changed')
            if self._continuity_latched:
                raise ValueError('unverified_continuity_after_discontinuity')
            # Commit the new reference only after all checks pass. Invalid
            # packets cannot silently establish a new frame or time baseline.
            self._last_stamp_ns = stamp_ns
            self._frames = frames
            self._accepted_samples += 1
            self._last_rejection = None
            self._observation_state = 'observed_unverified'
            result['pose'] = dict(frame_id=frames[0], child_frame_id=frames[1],
                                  position=position, quaternion=quaternion)
        except (AttributeError, TypeError, ValueError, OverflowError) as error:
            reason = str(error)
            self._rejected_samples += 1
            self._last_rejection = reason
            if reason in ('out_of_order_source_stamp', 'source_frames_changed'):
                if not self._continuity_latched:
                    self._possible_discontinuities += 1
                self._continuity_latched = True
            self._observation_state = ('possible_discontinuity' if self._continuity_latched
                                       else 'invalid_sample')
            quality['rejection'] = reason
        quality['observation'] = dict(state=self._observation_state,
                                      accepted_samples=self._accepted_samples,
                                      rejected_samples=self._rejected_samples,
                                      possible_discontinuities=self._possible_discontinuities,
                                      continuity_quarantined=self._continuity_latched,
                                      received_monotonic_ns=received_monotonic_ns)
        return result

    def interface(self):
        """Describe a pose-only G1 source without claiming SLAM readiness.

        This report is compatible with the generic motus.odom/1 interface,
        but an unknown vendor tracking/clock/segment is not an available
        motus_slam/1 pose. No twist axes or update rate are invented.
        """
        interface = build_interface(provides=[], frame='world',
                                    rate_hz=0., pose_drift='unbounded')
        interface['slam_pose'] = {
            'available': False, 'profile': 'motus_slam/1',
            'missing_facts': list(MISSING_FACTS),
        }
        return interface

    def declaration(self, topic):
        return input_declaration(
            topic, 'state/odom', 'pose', SOURCE_TOPIC,
            source_topic=SOURCE_TOPIC, source_type='nav_msgs.msg.dds_.Odometry_',
            length_unit='m', reference_point='mid360_imu_origin',
            frame_id=self._frames[0] if self._frames else None,
            child_frame_id=self._frames[1] if self._frames else None,
            missing_facts=list(MISSING_FACTS), available=False)
