"""PiPER user motion: rigid TCP transforms and SDK-planned trajectories.

Pose convention matches piper_sdk@8d4dc29/kinematics/piper_fk.py:
R = Rz(yaw) Ry(pitch) Rx(roll), mm and degrees. No inverse kinematics.
"""
from dataclasses import dataclass
import json
import math
import time

from agilex.piper.intent_motion import EndPose, checked_state, end_pose, fresh


def vector(value, size, name, *, allow_null=False):
    """Accept native API arrays and Core's JSON-text fields; never coerce items."""
    if isinstance(value, str):
        try:
            value = json.loads(value)
        except (ValueError, TypeError) as exc:
            raise ValueError(f'{name} must be a JSON array of {size} numbers.') from exc
    if not isinstance(value, (list, tuple)) or len(value) != size:
        raise ValueError(f'{name} must contain exactly {size} numbers.')
    try:
        valid = all((v is None and allow_null) or
                    (not isinstance(v, bool) and isinstance(v, (int, float)) and math.isfinite(v)) for v in value)
    except OverflowError:
        valid = False
    if not valid:
        raise ValueError(f'{name} must contain finite numbers; strings and booleans are invalid.')
    if allow_null and all(v is None for v in value):
        raise ValueError(f'{name} must specify at least one numeric target.')
    return tuple(None if v is None else float(v) for v in value)


def multiply(a, b):
    return tuple(tuple(sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)) for i in range(3))


def apply(rotation, position):
    return tuple(sum(row[j] * position[j] for j in range(3)) for row in rotation)


def transpose(a):
    return tuple(zip(*a))


def rotation(rpy):
    roll, pitch, yaw = map(math.radians, rpy)
    cr, sr, cp, sp, cy, sy = (math.cos(roll), math.sin(roll), math.cos(pitch),
                             math.sin(pitch), math.cos(yaw), math.sin(yaw))
    return ((cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr),
            (sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr), (-sp, cp*sr, cp*cr))


def euler(matrix):
    pitch = math.atan2(-matrix[2][0], math.hypot(matrix[0][0], matrix[1][0]))
    if abs(math.cos(pitch)) > 1e-8:
        roll = math.atan2(matrix[2][1], matrix[2][2])
        yaw = math.atan2(matrix[1][0], matrix[0][0])
    else:
        yaw = 0.
        roll = math.atan2(-matrix[1][2], matrix[1][1])
    return tuple(math.degrees(v) for v in (roll, pitch, yaw))


def orientation_error(a, b):
    difference = multiply(transpose(rotation(a)), rotation(b))
    cosine = (sum(difference[i][i] for i in range(3)) - 1) / 2
    return math.degrees(math.acos(max(-1., min(1., cosine))))


@dataclass(frozen=True)
class Transform:
    position: tuple
    rotation: tuple

    @classmethod
    def from_pose(cls, values):
        return cls(tuple(values[:3]), rotation(values[3:]))

    def compose(self, other):
        offset = apply(self.rotation, other.position)
        return Transform(tuple(a+b for a, b in zip(self.position, offset)),
                         multiply(self.rotation, other.rotation))

    def inverse(self):
        inverted = transpose(self.rotation)
        return Transform(apply(inverted, tuple(-v for v in self.position)), inverted)

    def values(self):
        return (*self.position, *euler(self.rotation))


def tcp_transform(config):
    cfg = config.get('intent_control', {})
    if cfg.get('tcp_verified') is not True:
        raise RuntimeError('The gripper-tip reference is not calibrated. Configure and verify tcp_offset_mm_deg first.')
    values = vector(cfg.get('tcp_offset_mm_deg'), 6, 'tcp_offset_mm_deg')
    if math.dist(values[:3], (0, 0, 0)) > 300 or any(abs(v) > 180 for v in values[3:]):
        raise ValueError('The configured TCP offset exceeds 300 mm or 180 degrees.')
    return Transform.from_pose(values)


def target_pose(driver, action, args, config):
    measured = end_pose(driver)
    sdk_pose = Transform.from_pose((*measured.position_mm, *measured.orientation_deg))
    if action == 'move':
        delta = vector(args['translation_mm'], 3, 'translation_mm')
        angles = vector(args['rotation_deg'], 3, 'rotation_deg')
        if math.dist(delta, (0, 0, 0)) > 10 or any(abs(v) > 5 for v in angles):
            raise ValueError('Move allows at most 10 mm translation and 5 degrees per rotation axis.')
        if config.get('intent_control', {}).get('tcp_verified') is not True:
            if any(angles):
                raise RuntimeError('Gripper-tip rotation requires TCP calibration. Use rotation_deg=[0,0,0] for uncalibrated translation.')
            # With constant orientation every fixed point on the rigid tool has
            # the same displacement. Define axes parallel to the SDK J6 frame;
            # no tool length or absolute tip coordinate is invented here.
            displacement = apply(sdk_pose.rotation, delta)
            position = tuple(q+d for q, d in zip(measured.position_mm, displacement))
            validate_numeric_pose((*position, *measured.orientation_deg))
            return EndPose(position, measured.orientation_deg, measured.feedback_time), None
        offset = tcp_transform(config)
        tip = sdk_pose.compose(offset)
        goal_tip = tip.compose(Transform.from_pose((*delta, *angles)))
    else:
        offset = tcp_transform(config)
        requested = vector(args['pose_mm_deg'], 6, 'pose_mm_deg', allow_null=True)
        actual_tip = sdk_pose.compose(offset).values()
        pose = tuple(q if v is None else v for q, v in zip(actual_tip, requested))
        if any(abs(v) > 180 for v in pose[3:]):
            raise ValueError('Pose orientation must be within [-180, 180] degrees.')
        goal_tip = Transform.from_pose(pose)
    goal = goal_tip.compose(offset.inverse()).values()
    if action == 'move' and not any(angles):
        # Avoid changing equivalent Euler representations (especially near a
        # singularity) when the user requested translation with no rotation.
        goal = (*goal[:3], *measured.orientation_deg)
    validate_numeric_pose(goal)
    return EndPose(tuple(goal[:3]), tuple(goal[3:]), measured.feedback_time), goal_tip.values()


def validate_numeric_pose(values):
    if not all(math.isfinite(v) and math.isfinite(v*1000)
               and -(2**31) <= round(v*1000) < 2**31 for v in values):
        raise ValueError('Pose exceeds the controller numeric range.')


def joint_target(driver, action, args, config, *, current=None):
    current = current or checked_state(driver, allow_teaching=True)
    if action == 'reset':
        value = config.get('intent_control', {}).get('preset_joints_deg')
        if value is None:
            raise RuntimeError('No return posture is configured. An operator must verify and save preset_joints_deg.')
        target = vector(value, 6, 'preset_joints_deg')
    else:
        requested = vector(args['joints_deg'], 6, 'joints_deg', allow_null=args['joint_mode'] == 'absolute')
        target = (tuple(q+d for q, d in zip(current.joints_deg, requested))
                  if args['joint_mode'] == 'relative' else
                  tuple(q if v is None else v for q, v in zip(current.joints_deg, requested)))
    bounds = driver.joint_bounds_deg()
    if len(bounds) != 6:
        raise RuntimeError('All six effective joint limits are required.')
    for i, (q, (low, high)) in enumerate(zip(target, bounds), 1):
        if not math.isfinite(q) or not low <= q <= high:
            raise ValueError(f'Joint {i} target must be within [{low:g}, {high:g}] degrees.')
    raw = tuple(round(q * 1000) for q in target)
    if driver._bounded_target(raw) != raw:
        raise ValueError('Joint target exceeds effective limits after SDK rounding.')
    return tuple(v / 1000 for v in raw)


def _status(driver):
    status = driver.arm.GetArmStatus()
    if not fresh(status.time_stamp):
        raise RuntimeError('Controller feedback is stale.')
    return status


def send_target(driver, target, speed, kind, require_permission, *, hold=False,
                motion_mode='linear', require_path_permission=None, on_submission=lambda: None):
    """Stop and the final cancellation check share the same submission lock."""
    if motion_mode not in {'linear', 'point_to_point'} or kind != 'pose' and motion_mode != 'linear':
        raise ValueError('Invalid motion path.')
    with driver._send_lock:
        if not hold:
            driver._check_cancelled()
        require_permission()
        checked_state(driver, ready=True)
        _status(driver)
        if not hold and motion_mode == 'point_to_point':
            if not callable(require_path_permission):
                raise RuntimeError('Point-to-point requires supervised hardware acceptance.')
            require_path_permission()
        if kind == 'pose':
            values = (*target.position_mm, *target.orientation_deg)
            validate_numeric_pose(values)
            raw = tuple(round(v * 1000) for v in values)
            end_pose(driver)
        else:
            raw = tuple(round(v * 1000) for v in target)
            if driver._bounded_target(raw) != raw:
                raise ValueError('Joint target exceeds effective limits.')
        sent_at = time.time()
        mode = (0x00 if motion_mode == 'point_to_point' and not hold else 0x02) if kind == 'pose' else 0x01
        on_submission()
        driver.arm.MotionCtrl_2(0x01, mode, speed, 0x00)
        # Permission can be revoked during the mode submission.
        require_permission()
        if not hold:
            driver._check_cancelled()
        checked_state(driver, ready=True)
        if not hold and motion_mode == 'point_to_point':
            require_path_permission()
        if kind == 'pose':
            driver.arm.EndPoseCtrl(*raw)
        else:
            driver.arm.JointCtrl(*raw)
        return sent_at


def execute_target(driver, target, speed, kind, *, require_permission, report=lambda data: None,
                   tcp_offset=None, tip_goal=None, clock=time.monotonic, sleep=time.sleep,
                   motion_mode='linear', require_path_permission=None):
    """Submit one SDK trajectory; require fresh post-submission stable arrival."""
    checked_state(driver, ready=True)
    if kind == 'pose':
        end_pose(driver)
    values = (*target.position_mm, *target.orientation_deg) if kind == 'pose' else target
    path = motion_mode if kind == 'pose' else 'joint'
    report({'stage': 'target', 'target': values, 'target_kind': kind, 'speed_percent': speed,
            'motion_mode': path})
    attempted = False
    held = False
    def submitted():
        nonlocal attempted
        attempted = True
    try:
        driver._check_cancelled()
        require_permission()
        sent_at = send_target(driver, target, speed, kind, require_permission,
                             motion_mode=motion_mode, require_path_permission=require_path_permission,
                             on_submission=submitted)
        deadline, stable_since = clock()+30, None
        while clock() < deadline:
            driver._check_cancelled()
            require_permission()
            if motion_mode == 'point_to_point':
                require_path_permission()
            state, status = checked_state(driver, ready=True), _status(driver)
            if kind == 'pose':
                measured = end_pose(driver)
                actual = (*measured.position_mm, *measured.orientation_deg)
                actual_tip = (Transform.from_pose(actual).compose(tcp_offset).values()
                              if tcp_offset is not None else actual)
                arrival_goal = tip_goal if tip_goal is not None else values
                near = (math.dist(actual_tip[:3], arrival_goal[:3]) <= .5 and
                        orientation_error(actual_tip[3:], arrival_goal[3:]) <= 1)
                feedback_time = min(measured.feedback_time, state.feedback_time)
            else:
                actual, feedback_time = state.joints_deg, state.feedback_time
                near = max(abs(q-t) for q, t in zip(actual, target)) <= .5
            arrived = (near and feedback_time > sent_at and status.time_stamp > sent_at
                       and int(status.arm_status.motion_status) == 0)
            if arrived:
                stable_since = clock() if stable_since is None else stable_since
                if clock()-stable_since >= .5:
                    driver._check_cancelled()
                    require_permission()
                    result = {'executed': True, 'target_kind': kind, 'target': values, 'actual': actual,
                              'motion_mode': path,
                              'message': 'Target reached and stable.'}
                    if kind == 'pose' and tcp_offset is not None:
                        result['actual_tip_mm_deg'] = actual_tip
                    return result
            else:
                stable_since = None
            sleep(.05)
        raise RuntimeError('Movement timed out after 30 seconds; arrival was not confirmed.')
    except BaseException as exc:
        if attempted:
            try:
                measured = end_pose(driver) if kind == 'pose' else checked_state(driver, ready=True).joints_deg
                send_target(driver, measured, speed, kind, require_permission, hold=True)
                held = True
            except Exception:
                pass
        try:
            report({'stage': 'interrupted', 'error': str(exc), 'measured_hold_sent': held})
        except Exception:
            pass
        raise
