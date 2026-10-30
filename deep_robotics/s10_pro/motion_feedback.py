"""Read-only validation of motion and odometry observations."""
import math

def odometry_observation(record, now):
    """Parse ODOM for relative movement and optional stationary corroboration."""
    result = {'fresh': False, 'sequence': 0, 'x': None, 'y': None, 'yaw': None,
              'received_monotonic': None, 'age_s': None, 'reason': 'fresh_odometry_required'}
    if not isinstance(record, dict) or not fresh(record, now, .35):
        return result
    raw = record.get('data')
    pose = raw.get('pose') if isinstance(raw, dict) else None
    pose = pose.get('pose') if isinstance(pose, dict) else None
    position = pose.get('position') if isinstance(pose, dict) else None
    orientation = pose.get('orientation') if isinstance(pose, dict) else None
    if not isinstance(position, dict) or not isinstance(orientation, dict):
        return result
    twist = raw.get('twist', {})
    twist = twist.get('twist', {}) if isinstance(twist, dict) else {}
    linear = twist.get('linear', {}) if isinstance(twist, dict) else {}
    speeds = [linear.get(key) for key in ('x', 'y', 'z')] if isinstance(linear, dict) else []
    if len(speeds) != 3 or any(type(v) not in (int, float) or not math.isfinite(v) for v in speeds):
        return result
    if math.sqrt(sum(v * v for v in speeds)) > .30:
        return {**result, 'reason': 'odometry_velocity_inconsistent'}
    values = [position.get('x'), position.get('y'), orientation.get('x'),
              orientation.get('y'), orientation.get('z'), orientation.get('w')]
    if any(type(value) not in (int, float) or not math.isfinite(value) for value in values):
        return result
    norm = math.sqrt(sum(value * value for value in values[2:]))
    if not .5 <= norm <= 1.5:
        return result
    qx, qy, qz, qw = (value / norm for value in values[2:])
    yaw = math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
    sequence = record.get('sequence')
    if type(sequence) is not int or sequence <= 0:
        return result
    angular = twist.get('angular', {}) if isinstance(twist, dict) else {}
    yaw_rate = angular.get('z') if isinstance(angular, dict) else None
    if type(yaw_rate) not in (int, float) or not math.isfinite(yaw_rate):
        return result
    header = raw.get('header', {})
    stamp = header.get('stamp', {}) if isinstance(header, dict) else {}
    sec, nanosec = (stamp.get(k) for k in ('sec', 'nanosec')) if isinstance(stamp, dict) else (None, None)
    source_stamp = (sec + nanosec / 1e9 if type(sec) is int and sec >= 0
                    and type(nanosec) is int and 0 <= nanosec < 1_000_000_000 else None)
    return {'fresh': True, 'sequence': sequence, 'x': float(values[0]),
            'y': float(values[1]), 'yaw': yaw, 'vx': float(speeds[0]),
            'vy': float(speeds[1]), 'yaw_rate': float(yaw_rate),
            'received_monotonic': float(record['received_monotonic']),
            'age_s': float(record['age_s']), 'source_stamp_s': source_stamp}


def fresh(record, now, limit):
    age, received = record.get('age_s'), record.get('received_monotonic')
    return (record.get('fresh') is True and not record.get('error')
            and type(age) in (float, int) and math.isfinite(age) and 0 <= age <= limit
            and type(received) in (float, int) and math.isfinite(received)
            and 0 <= now - received <= limit and abs(now - received - age) <= .1)



def motion_observation(record, now):
    result = {'fresh': False, 'motion_state': None, 'velocity': {},
              'sequence': record.get('sequence'), 'received_monotonic': record.get('received_monotonic')}
    if not fresh(record, now, .5):
        return result
    raw = record.get('data')
    data = raw.get('data', {}) if isinstance(raw, dict) else {}
    state = data.get('motion_state', {})
    speed = [data.get(k) for k in ('vel_x', 'vel_y', 'vel_yaw')]
    if not isinstance(state, dict) or type(state.get('state')) is not int:
        return result
    if any(type(v) not in (int, float) or not math.isfinite(v) for v in speed):
        return result
    return {**result, 'fresh': True, 'motion_state': state['state'],
            'velocity': dict(zip(('vx', 'vy', 'yaw_rate'), speed))}
