"""G1 mapping Odometry conversion; unproven SLAM evidence stays unavailable.

Source contract: https://support.unitree.com/home/en/G1_developer/slam_navigation_services_interface
The source is ROS Odometry (meters, named xyzw quaternion components). Its
header is preserved, but the public G1 contract does not establish measurement
clock semantics or per-pose tracking/reset identity. No configuration flag can
promote these samples to tracking. Raw covariance is evidence, not a claim of
estimator uncertainty; it is retained under vendor rather than pose.
"""
import math
from uuid import uuid4

from common.odom import DEFAULT_UNITS
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
        self.source_generation = uuid4().hex
        self._last_stamp_ns = None
        self._frames = None

    def convert(self, msg):
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
            previous_frames = self._frames
            self._frames = frames
            if self._last_stamp_ns is not None:
                if stamp_ns == self._last_stamp_ns:
                    raise ValueError('duplicate_source_stamp')
                if stamp_ns < self._last_stamp_ns:
                    raise ValueError('out_of_order_source_stamp')
                if stamp_ns // 1_000_000 == self._last_stamp_ns // 1_000_000:
                    raise ValueError('duplicate_profile_stamp')
            self._last_stamp_ns = stamp_ns
            if previous_frames is not None and frames != previous_frames:
                raise ValueError('source_frames_changed')
            result['pose'] = dict(frame_id=frames[0], child_frame_id=frames[1],
                                  position=position, quaternion=quaternion)
        except (AttributeError, TypeError, ValueError, OverflowError) as error:
            quality['rejection'] = str(error)
        return result

    def declaration(self, topic):
        return input_declaration(
            topic, 'state/odom', 'pose', SOURCE_TOPIC,
            source_topic=SOURCE_TOPIC, source_type='nav_msgs.msg.dds_.Odometry_',
            length_unit='m', reference_point='mid360_imu_origin',
            frame_id=self._frames[0] if self._frames else None,
            child_frame_id=self._frames[1] if self._frames else None,
            missing_facts=list(MISSING_FACTS), available=False)
