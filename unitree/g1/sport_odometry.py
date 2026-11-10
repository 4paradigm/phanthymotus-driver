"""G1 sport-mode odometry evidence for the generic SLAM input port.

This replaces the native *input source* for the G1 RTAB-Map candidate.
rt/odommodestate is independent of the vendor mapping/relocation service.
The vendor unitree_go SportModeState_ has source stamp, position, velocity and
IMU orientation [w, x, y, z] but NO explicit ROS pose frame or clock/epoch
proof.  Never claim tracking, a calibrated time relation, or a SLAM segment
from an apparently valid position, orientation or receipt timestamp.
"""
import math
import time
from uuid import uuid4

from common.odom import DEFAULT_UNITS, build_interface
from common.slam_input import input_declaration

SOURCE_TOPIC = 'rt/odommodestate'
SOURCE_TYPE = 'unitree_go.msg.dds_.SportModeState_'
MISSING_FACTS = (
    'measurement_stamp_semantics', 'clock_domain_and_epoch',
    'clock_relation_to_depth_or_lidar', 'pose_reference_frame',
    'position_imu_orientation_frame_alignment', 'tracking_quality',
    'segment_identity', 'tracking_reset_and_relocalization',
    'sensor_extrinsic',
)


def _numbers(values, length):
    # CycloneDDS array fields may be lists, tuples or generated array types.
    # Never interpret strings/dicts as numeric sensor vectors.
    if isinstance(values, (str, bytes, dict)) or values is None:
        raise ValueError('invalid_source_vector_length')
    try:
        vector = list(values)
    except (TypeError, ValueError):
        raise ValueError('invalid_source_vector_length') from None
    if len(vector) != length:
        raise ValueError('invalid_source_vector_length')
    if any(isinstance(x, bool) or not isinstance(x, (int, float))
           or not math.isfinite(x) for x in vector):
        raise ValueError('non_finite_or_non_numeric_pose')
    return [float(x) for x in vector]


class SportOdometry:
    """Convert G1 leg/sport odometry to an explicitly unverified SLAM candidate."""

    def __init__(self, *, frame_id=None, child_frame_id=None):
        if (frame_id is None) != (child_frame_id is None):
            raise ValueError('sport odometry requires both frame names or neither')
        if frame_id is not None:
            if any(not isinstance(v, str) or not v.strip()
                   for v in (frame_id, child_frame_id)):
                raise ValueError('sport odometry frame names must be nonempty strings')
        # These are only operator-provided labels; no clock/pose validation is
        # inferred from the fact that they are syntactically valid ROS names.
        self.frame_id = frame_id.strip() if frame_id is not None else None
        self.child_frame_id = child_frame_id.strip() if child_frame_id is not None else None
        self.source_generation = uuid4().hex
        self._last_stamp_ns = None
        self._last_received_monotonic_ns = None
        self._accepted = 0
        self._rejected = 0
        self._continuity_quarantined = False
        self._suspected_resets = 0
        self._last_rejection = None
        self._last_observation = 'waiting'

    def convert(self, msg):
        # Keep callback receipt times as diagnostics only. Neither timestamp
        # can substitute for the missing *measurement* time in G1 SportModeState.
        received = time.monotonic_ns()
        received_wall_ns = time.time_ns()
        self._last_received_monotonic_ns = received
        quality = dict(
            status='unavailable', stamp_source=None, clock_id=None,
            clock_epoch=None, segment_id=None, source_stamp_ns=None,
            source_stamp_valid=False,
            source_generation=self.source_generation, rejection=None,
            missing_facts=list(MISSING_FACTS),
        )
        vendor = dict(
            source_topic=SOURCE_TOPIC, source_type=SOURCE_TYPE,
            received_ns=received_wall_ns,
            received_monotonic_ns=received,
            received_time_kind='driver_callback_receipt_not_measurement',
            motus_slam=quality,
        )
        result = dict(
            schema='motus.odom/1', stamp_ms=None, frame='world',
            units=dict(DEFAULT_UNITS), twist=[None] * 6, pose=None,
            vendor=vendor,
        )
        try:
            # Real G1 rt/odommodestate and rt/lf/odommodestate currently report
            # stamp=(0, 0). Validate/copy finite raw motion evidence FIRST, so
            # a missing sensor stamp never hides the position and orientation.
            position = _numbers(msg.position, 3)
            raw_quat = _numbers(msg.imu_state.quaternion, 4)  # WXYZ in Unitree SDK.
            q = [raw_quat[1], raw_quat[2], raw_quat[3], raw_quat[0]]
            velocity = _numbers(msg.velocity, 3)
            yaw_speed = _numbers([msg.yaw_speed], 1)[0]
            error_code = msg.error_code
            if type(error_code) is not int or error_code < 0:
                raise ValueError('invalid_source_error_code')
            vendor.update(
                raw_position=position,
                raw_imu_quaternion_wxyz=raw_quat,
                raw_imu_quaternion_xyzw=q,
                raw_velocity=velocity, raw_yaw_speed=yaw_speed,
                source_error_code=error_code,
                position_reference='unknown',
                imu_orientation_reference='unknown',
            )
            if abs(sum(v*v for v in raw_quat) - 1.) > .002:
                raise ValueError('non_unit_imu_quaternion')
            if error_code:
                raise ValueError('nonzero_source_error_code')

            sec, nsec = msg.stamp.sec, msg.stamp.nanosec
            # Exact native values (including 0,0) are evidence; never relabel
            # receipt time as source time or promote it to stamp_ms.
            vendor['raw_source_stamp'] = {
                'sec': sec if type(sec) is int else None,
                'nanosec': nsec if type(nsec) is int else None,
            }
            if (type(sec) is not int or sec < 0 or type(nsec) is not int or
                    not 0 <= nsec < 1_000_000_000):
                raise ValueError('invalid_source_stamp')
            stamp_ns = sec * 1_000_000_000 + nsec
            if stamp_ns < 1_000_000:
                raise ValueError('invalid_source_stamp')
            quality['source_stamp_ns'] = stamp_ns
            quality['source_stamp_valid'] = True
            result['stamp_ms'] = stamp_ns // 1_000_000
            if self._last_stamp_ns is not None:
                if stamp_ns == self._last_stamp_ns:
                    raise ValueError('duplicate_source_stamp')
                if stamp_ns < self._last_stamp_ns:
                    raise ValueError('out_of_order_source_stamp')
                if stamp_ns // 1_000_000 == self._last_stamp_ns // 1_000_000:
                    raise ValueError('duplicate_profile_stamp')
            if self._continuity_quarantined:
                raise ValueError('unverified_continuity_after_discontinuity')
            self._last_stamp_ns = stamp_ns
            self._accepted += 1
            self._last_rejection = None
            self._last_observation = 'observed_unverified'
            # Even when the operator supplies frame names, producer metadata
            # remains unavailable until the pose reference, source clock and
            # tracking/restart guarantees are independently verified.
            if self.frame_id is not None:
                result['pose'] = dict(frame_id=self.frame_id,
                                      child_frame_id=self.child_frame_id,
                                      position=position, quaternion=q)
        except (AttributeError, TypeError, ValueError, OverflowError) as error:
            reason = str(error)
            self._rejected += 1
            self._last_rejection = reason
            if reason == 'out_of_order_source_stamp' and not self._continuity_quarantined:
                self._suspected_resets += 1
                self._continuity_quarantined = True
            self._last_observation = ('possible_reset' if self._continuity_quarantined
                                      else 'invalid_sample')
            quality['rejection'] = reason
        quality['observation'] = dict(
            state=self._last_observation,
            accepted_samples=self._accepted, rejected_samples=self._rejected,
            suspected_resets=self._suspected_resets,
            continuity_quarantined=self._continuity_quarantined,
            received_monotonic_ns=received,
            received_wall_ns=received_wall_ns,
        )
        return result

    def observation_status(self, *, now_monotonic_ns=None, stale_after_s=2.):
        if isinstance(stale_after_s, bool) or not isinstance(stale_after_s, (int, float)) or not math.isfinite(stale_after_s) or stale_after_s <= 0:
            raise ValueError('stale_after_s must be positive')
        now = time.monotonic_ns() if now_monotonic_ns is None else now_monotonic_ns
        if type(now) is not int or now < 0:
            raise ValueError('invalid observation time')
        age = max(0, now-self._last_received_monotonic_ns) / 1e9 if self._last_received_monotonic_ns is not None else None
        stale = age > stale_after_s if age is not None else None
        return dict(
            available=False,
            state=('waiting' if age is None else 'stale_observation' if stale
                   else self._last_observation),
            receipt_age_s=age, receipt_stale=stale,
            source_topic=SOURCE_TOPIC, source_generation=self.source_generation,
            last_source_stamp_ns=self._last_stamp_ns,
            frame_id=self.frame_id, child_frame_id=self.child_frame_id,
            accepted_samples=self._accepted, rejected_samples=self._rejected,
            suspected_resets=self._suspected_resets,
            continuity_quarantined=self._continuity_quarantined,
            last_rejection=self._last_rejection, clock_id=None,
            clock_epoch=None, segment_id=None,
        )

    def interface(self):
        interface = build_interface(provides=[], frame='world', rate_hz=0.,
                                    pose_drift='unbounded')
        interface['slam_pose'] = {
            'available': False, 'profile': 'motus_slam/1',
            'missing_facts': list(MISSING_FACTS),
        }
        return interface

    def declaration(self, topic):
        return input_declaration(
            topic, 'state/odom', 'pose', SOURCE_TOPIC,
            source_topic=SOURCE_TOPIC, source_type=SOURCE_TYPE,
            length_unit='m', reference_point=None, frame_id=self.frame_id,
            child_frame_id=self.child_frame_id, available=False,
            missing_facts=list(MISSING_FACTS),
        )
