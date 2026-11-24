"""Numeric spatial ports; preserve source time without certifying synchronization."""
import copy
import math


def finite_vector(value, size):
    if (not isinstance(value, list) or len(value) != size
            or any(isinstance(x, bool) or not isinstance(x, (int, float))
                   or not math.isfinite(x) for x in value)):
        raise ValueError('invalid numeric vector')
    return list(value)


def stamp_ns(header):
    t = header.get('stamp', {})
    s, n = t.get('secs'), t.get('nsecs')
    if type(s) is not int or type(n) is not int or s < 0 or not 0 <= n < 10**9 or s+n == 0:
        raise ValueError('positive source ROS timestamp required')
    return s*10**9+n


def odometry_pose(message, epoch):
    header = message['header']
    ns = stamp_ns(header)
    raw = message['pose']['pose']
    p = finite_vector([raw['position'][k] for k in 'xyz'], 3)
    q = finite_vector([raw['orientation'][k] for k in 'xyzw'], 4)
    if abs(sum(x*x for x in q)-1.) > 1e-3:
        raise ValueError('invalid odometry pose quaternion')
    if not header.get('frame_id') or not message.get('child_frame_id'):
        raise ValueError('odometry pose frames required')
    cov = message['pose'].get('covariance')
    try:
        cov = finite_vector(cov, 36)
    except ValueError:
        cov = None
    pose = dict(stamp_ms=ns/1e6, frame_id=header['frame_id'],
                child_frame_id=message['child_frame_id'], position=p, quaternion=q, covariance=cov)
    quality = dict(profile='motus_slam/1', status='tracking', stamp_ns=ns,
                   stamp_source='measurement', clock_id='ros1_odom_header', epoch=epoch,
                   pose_reference='body', pose_source='/odom',
                   timestamp_semantics='vendor_ros_header', synchronization='unverified',
                   measurement=dict(schema='motus.slam_frame/1', stamp_ns=ns,
                                    frame_id=header['frame_id'], clock_id='ros1_odom_header',
                                    source_generation=epoch, mapping=dict(verified=False, uncertainty_ns=None)))
    return pose, quality


def imu_packet(value):
    if not isinstance(value, dict) or value.get('schema') != 'motus.imu/1':
        raise ValueError('motus.imu/1 required')
    out = copy.deepcopy(value)
    for key in ('stamp_ns', 'accel_stamp_ns'):
        if type(out.get(key)) is not int or out[key] <= 0:
            raise ValueError('integer source IMU timestamps required')
    for key in ('angular_velocity', 'linear_acceleration'):
        finite_vector(out.get(key), 3)
    for key in ('camera_from_gyro', 'camera_from_accel'):
        r = out.get(key)
        if not isinstance(r, list) or len(r) != 3:
            raise ValueError('camera IMU rotations required')
        for row in r:
            finite_vector(row, 3)
        for i in range(3):
            for j in range(3):
                if abs(sum(r[i][k]*r[j][k] for k in range(3))-(1 if i == j else 0)) > 1e-4:
                    raise ValueError('IMU transform is not a rotation')
        det = sum(r[0][i]*(r[1][(i+1)%3]*r[2][(i+2)%3]-r[1][(i+2)%3]*r[2][(i+1)%3]) for i in range(3))
        if abs(det-1) > 1e-4:
            raise ValueError('IMU transform must preserve handedness')
    for key in ('camera_from_gyro_translation', 'camera_from_accel_translation'):
        if key in out:
            finite_vector(out[key], 3)
    return out
