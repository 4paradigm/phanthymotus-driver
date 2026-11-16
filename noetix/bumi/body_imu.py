"""HighController body IMU adapter for the fixed RGB camera installation."""
import uuid

try:
    from .visual_odometry_contract import CameraCalibration, finite, quat_matrix
except ImportError:
    from visual_odometry_contract import CameraCalibration, finite, quat_matrix


class BodyImuSource:
    def __init__(self, namespace, mount, reference):
        self.source_id = f'noetix/{namespace}/body_imu'
        self.frame = f'{namespace}_body_imu'
        self.reference = reference
        self.setup_reason = None
        try:
            calibration = CameraCalibration(mount)
            if calibration.assumed:
                raise ValueError('verified fixed camera/body rotation required')
            self.calibration_id = calibration.id
            body_from_camera = quat_matrix(calibration.data['camera_to_body']['quaternion'])
            self.rotation = [list(row) for row in zip(*body_from_camera)]
        except (ValueError, TypeError, KeyError) as error:
            self.setup_reason = 'body IMU mount unavailable: ' + str(error)
        self.signature = None
        self.reset()

    def reset(self, *, after_ns=0):
        self.epoch = uuid.uuid4().hex
        self.last_capture_ns = 0
        self.last_read_ns = after_ns
        self.reason = self.setup_reason or 'waiting for body IMU reading'

    def unavailable(self, reason):
        self.reason = str(reason)

    def accept(self, data, now_ns):
        try:
            if self.setup_reason:
                raise ValueError(self.setup_reason)
            stamp = data.get('stamp_ns')
            if (data.get('timestamp_source') != 'read_completion' or type(stamp) is not int or
                    stamp <= self.last_read_ns or not -50_000_000 <= now_ns-stamp <= 500_000_000):
                raise ValueError('body IMU read time is stale, repeated or invalid')
            quaternion = finite(data.get('quaternion'), 4)
            quat_matrix(quaternion)
            angular = finite(data.get('angular_vel'), 3)
            acceleration = finite(data.get('linear_acc'), 3)
            signature = tuple(quaternion + angular + acceleration)
            self.last_read_ns = stamp
            if signature == self.signature:
                return None  # A cached SDK value is not a new measurement.
            camera_frame, clock = self.reference()
            if not camera_frame or not clock:
                raise ValueError('waiting for RGB-D wall-clock mapping')
            packet = dict(schema='motus.imu/1', source_id=self.source_id, epoch=self.epoch,
                stamp_ns=stamp, accel_stamp_ns=stamp, stamp_source='read_completion',
                clock_id=clock, camera_frame_id=camera_frame,
                gyro_frame_id=self.frame, accel_frame_id=self.frame,
                camera_from_gyro=self.rotation, camera_from_accel=self.rotation,
                rotation_source='configured_fixed_mount', calibration_id=self.calibration_id,
                angular_velocity=angular, linear_acceleration=acceleration,
                units=dict(angular_velocity='rad/s', linear_acceleration='m/s^2'),
                clock_mapping=dict(method='host_wall_read_completion', source_clock='ros_wall',
                                   offset_ns=0, verified=False))
            # The existing mount sets the body reference at the camera origin;
            # it does not measure the accelerometer lever arm. Publish no translation.
            self.signature, self.last_capture_ns, self.reason = signature, stamp, None
            return packet
        except (ValueError, TypeError, KeyError) as error:
            self.unavailable(error)
            raise ValueError(str(error)) from error

    def status(self, now_ns):
        live = self.reason is None and -50_000_000 <= now_ns-self.last_capture_ns <= 500_000_000
        return dict(state='ready' if live else 'unavailable', source='body_imu',
                    reason=self.reason if self.reason else (None if live else 'body IMU reading stale'),
                    last_capture_ns=self.last_capture_ns, stamp_source='read_completion',
                    fusion=dict(state='unavailable', reason='full IMU translation vectors are not supplied'))
