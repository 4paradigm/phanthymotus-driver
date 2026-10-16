"""Small base-frame translations using the pinned manufacturer's MOVE_L."""
from dataclasses import dataclass
import math
import time


DIRECTIONS = {'forward': (1, 0, 0), 'backward': (-1, 0, 0),
              'left': (0, 1, 0), 'right': (0, -1, 0),
              'up': (0, 0, 1), 'down': (0, 0, -1)}
POSE_FIELDS = ('X_axis', 'Y_axis', 'Z_axis', 'RX_axis', 'RY_axis', 'RZ_axis')
SMALL_STEP_MM = 1
JOG_SPEED_PERCENT = 1
# Conservative software start exclusion, not the firmware's singularity
# threshold. In the pinned official FK, J4/J6 align at J5=0 (zero J5 offset,
# zero wrist link lengths and opposite pi/2 twists).
WRIST_ALIGNMENT_GUARD_DEG = 1.0


class DirectionUnavailable(RuntimeError):
    """A controller target rejection, distinct from a motor/device failure."""
    def __init__(self, message):
        super().__init__(message)
        self.feedback = None


class JointLimitViolation(RuntimeError):
    """Strict measured-limit failure with the original, unclamped feedback."""
    def __init__(self, message, feedback=None):
        super().__init__(message)
        self.feedback = feedback
        self.failure_code = 'measured_joint_limit'


def controller_error(status):
    if status.startswith('TARGET_POS_EXCEEDS_LIMIT'):
        return DirectionUnavailable('The controller rejected a movement target as outside its limits. '
            'A previous target-limit report is still active; no new target was sent. '
            'Recover the controller before retrying. Reset returns joint angles and does not clear faults.')
    if status.startswith(('NO_SOLUTION', 'SINGULARITY_POINT')):
        reason = 'no kinematic solution' if status.startswith('NO_SOLUTION') else 'a singularity'
        return DirectionUnavailable(f'The controller reports {status} ({reason}). '
            'Control requests are blocked while this report remains active. '
            'Controller recovery requires operator supervision because SDK reset releases motor torque. '
            'The card Reset only returns joint angles. No fault clear was sent.')
    return RuntimeError('The arm reports a fault: ' + status)


@dataclass(frozen=True)
class EndPose:
    position_mm: tuple
    orientation_deg: tuple
    feedback_time: float


def fresh(value, now=None):
    age = (time.time() if now is None else now) - value
    return math.isfinite(value) and 0 <= age <= .5


def end_pose(driver):
    msg = driver.arm.GetArmEndPoseMsgs()
    stamp = msg.time_stamp
    if hasattr(driver.arm, 'frame_times'):
        with driver.arm.frame_lock:
            stamps = [driver.arm.frame_times.get(frame, 0.) for frame in (0x2A2, 0x2A3, 0x2A4)]
        if not all(fresh(t) for t in stamps):
            raise RuntimeError('End-effector feedback is stale; check the CAN connection.')
        stamp = min(stamps)
    if not fresh(stamp):
        raise RuntimeError('End-effector feedback is stale; check the CAN connection.')
    raw = tuple(getattr(msg.end_pose, key) for key in POSE_FIELDS)
    if any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) for v in raw):
        raise RuntimeError('End-effector feedback is invalid.')
    return EndPose(tuple(v / 1000 for v in raw[:3]), tuple(v / 1000 for v in raw[3:]), stamp)


def checked_state(driver, *, ready=False, allow_teaching=False):
    state = driver.get_state()
    if not fresh(state.feedback_time):
        raise RuntimeError('Joint feedback is stale; check the CAN connection.')
    if len(state.joints_deg) != 6 or not all(math.isfinite(v) for v in state.joints_deg):
        raise RuntimeError('Joint feedback is invalid.')
    if any(state.enabled) and not all(state.enabled):
        raise RuntimeError('Some arm motors are disabled; check the arm before retrying.')
    if state.error_code or state.motor_faults or not state.arm_status.startswith('NORMAL'):
        if state.error_code or state.motor_faults:
            raise RuntimeError('The arm reports a device fault: ' + state.arm_status)
        error = controller_error(state.arm_status)
        if isinstance(error, DirectionUnavailable):
            error.feedback = {'controller_status': state.arm_status, 'error_code': state.error_code,
                              'joints_deg': state.joints_deg, 'feedback_time': state.feedback_time}
        raise error
    if not state.teach_status.startswith(('DISABLED', 'STOP_RECORDING', 'TERMINATE_EXECUTION')):
        raise RuntimeError('Teaching or trajectory playback is active; stop it on the robot first.')
    modes = ('STANDBY', 'CAN_CTRL', 'TEACHING_MODE') if allow_teaching else ('STANDBY', 'CAN_CTRL')
    if not state.control_mode.startswith(modes):
        raise RuntimeError('The arm is in an unsupported mode: ' + state.control_mode)
    if ready and (not state.control_mode.startswith('CAN_CTRL') or not all(state.enabled)):
        raise RuntimeError('The arm lost its ready state; the action was cancelled.')
    bounds = driver.joint_bounds_deg()
    if len(bounds) != 6:
        raise RuntimeError('All six effective joint limits are required.')
    violations = [dict(joint=i, actual_deg=q, limits_deg=list(limit))
                  for i, (q, limit) in enumerate(zip(state.joints_deg, bounds), 1)
                  if not limit[0] <= q <= limit[1]]
    if violations:
        first = violations[0]
        raise JointLimitViolation(
            f'Joint {first["joint"]} is outside its effective limits: '
            f'{first["actual_deg"]:g} deg, allowed [{first["limits_deg"][0]:g}, '
            f'{first["limits_deg"][1]:g}] deg. No out-of-range hold target will be sent.',
            {'controller_status': state.arm_status, 'error_code': state.error_code,
             'joints_deg': list(state.joints_deg), 'joint_limits_deg': [list(b) for b in bounds],
             'feedback_time': state.feedback_time, 'violations': violations})
    return state


def cartesian_start_check(driver, state=None):
    """Conservative start guard, not an IK/reachability claim or an arm move."""
    state = state or checked_state(driver, allow_teaching=True)
    bounds = driver.joint_bounds_deg()
    endpoints = [dict(joint=i, actual_deg=q, limits_deg=list(limit))
                 for i, (q, limit) in enumerate(zip(state.joints_deg, bounds), 1)
                 if q <= limit[0] or q >= limit[1]]
    if endpoints:
        names = ', '.join('J' + str(item['joint']) for item in endpoints)
        error = RuntimeError('Cartesian movement is unavailable at a joint limit (' + names + '). '
            'Use Set joints to select an in-range working posture first. No Cartesian target was sent. '
            'Reset is a zero-angle return, not a working posture or fault recovery.')
        error.failure_code = 'cartesian_start_at_joint_limit'
        error.feedback = {'joints_deg': list(state.joints_deg),
            'joint_limits_deg': [list(b) for b in bounds], 'boundary_joints': endpoints,
            'controller_status': state.arm_status, 'feedback_time': state.feedback_time}
        raise error
    if abs(state.joints_deg[4]) <= WRIST_ALIGNMENT_GUARD_DEG:
        error = RuntimeError(
            f'Cartesian movement is blocked near wrist alignment: J5={state.joints_deg[4]:g} deg '
            f'is inside the software +/-{WRIST_ALIGNMENT_GUARD_DEG:g} deg start guard. '
            'Use Set joints to choose a non-aligned working posture while the controller is healthy. '
            'This guard does not guarantee path reachability. No Cartesian target was sent.')
        error.failure_code = 'cartesian_start_near_wrist_alignment'
        error.feedback = {'joints_deg': list(state.joints_deg),
            'controller_status': state.arm_status, 'feedback_time': state.feedback_time,
            'wrist_alignment_guard_deg': WRIST_ALIGNMENT_GUARD_DEG}
        raise error
    return state


def _send(driver, pose, speed, *, cancelled_hold=False):
    raw = tuple(round(v * 1000) for v in (*pose.position_mm, *pose.orientation_deg))
    if any(not -(2 ** 31) <= v < 2 ** 31 for v in raw):
        raise ValueError('The requested position is outside the controller numeric range.')
    with driver._send_lock:
        if not cancelled_hold:
            driver._check_cancelled()
        checked_state(driver, ready=True)
        driver.arm.MotionCtrl_2(0x01, 0x02, speed, 0x00)
        driver.arm.EndPoseCtrl(*raw)


def hold_cartesian(driver, speed):
    try:
        checked_state(driver, ready=True)
        _send(driver, end_pose(driver), speed, cancelled_hold=True)
        return True
    except Exception:
        return False


def move_cartesian(driver, direction, distance, speed, *, clock=time.monotonic, sleep=time.sleep,
                   report=lambda details: None):
    """Confirm each <=1 mm target before advancing; one original reference.

    Public direction actions always request 1 mm at 1%. Longer distances are
    internal only until physical acceptance. Subdivision does not guarantee IK
    feasibility, and never clears a controller error or retries a rejected goal.
    """
    if direction not in DIRECTIONS:
        raise ValueError('Unknown movement direction.')
    if (isinstance(distance, bool) or not isinstance(distance, (int, float))
            or not math.isfinite(distance) or not 1 <= distance <= 10
            or type(speed) is not int or not 1 <= speed <= 10):
        raise ValueError('Movement requires 1–10 mm and integer speed 1–10%.')
    checked_state(driver, ready=True)
    start = end_pose(driver)
    vector = DIRECTIONS[direction]
    final = tuple(p + distance * d for p, d in zip(start.position_mm, vector))
    details = {'direction': direction, 'requested_distance_mm': distance, 'speed_percent': speed,
               'start_mm': start.position_mm, 'target_mm': final, 'orientation_deg': start.orientation_deg,
               'start_joints_deg': driver.get_state().joints_deg, 'completed_steps': 0, 'commands_sent': 0,
               'command_attempted': False}
    report(dict(details))  # Persist the original target before any control frame.
    deadline = clock() + 30
    actual = start
    try:
        travelled = 0.
        while travelled < distance:
            driver._check_cancelled()
            checked_state(driver, ready=True)
            travelled = min(distance, travelled + SMALL_STEP_MM)
            target = EndPose(tuple(p + travelled * d for p, d in zip(start.position_mm, vector)),
                             start.orientation_deg, start.feedback_time)
            details.update(step_target_mm=target.position_mm)
            report(dict(details))
            stable_since = sent_at = None
            while clock() < deadline:
                driver._check_cancelled()
                checked_state(driver, ready=True)
                actual = end_pose(driver)
                status = driver.arm.GetArmStatus()
                if not fresh(status.time_stamp):
                    raise RuntimeError('Controller status is stale.')
                reached = int(status.arm_status.motion_status) == 0
                near = (math.dist(actual.position_mm, target.position_mm) <= .5
                        and all(abs((a - b + 180) % 360 - 180) <= 1
                                for a, b in zip(actual.orientation_deg, target.orientation_deg)))
                if sent_at is not None and actual.feedback_time > sent_at and status.time_stamp > sent_at and near and reached:
                    stable_since = clock() if stable_since is None else stable_since
                    if clock() - stable_since >= .5:
                        break
                else:
                    stable_since = None
                if sent_at is None:
                    # MOVE_L owns the trajectory after submission. Resending
                    # the same target can restart controller planning; poll
                    # feedback without submitting again while it is running.
                    # Timestamp before transmission excludes cached arrival.
                    stamp = time.time()
                    details['command_attempted'] = True
                    _send(driver, target, speed)
                    details['commands_sent'] += 1
                    sent_at = stamp
                sleep(.05)
            else:
                raise RuntimeError('Move timed out after 30 seconds; target completion was not confirmed.')
            details.update(completed_steps=details['completed_steps'] + 1,
                           position_mm=actual.position_mm, actual_displacement_mm=math.dist(start.position_mm, actual.position_mm))
            report(dict(details))
        displacement = math.dist(start.position_mm, actual.position_mm)
        return {'executed': True, 'message': f'Move {direction} completed; requested {distance:g} mm, '
                    f'measured displacement {displacement:.2f} mm.',
                'position_mm': actual.position_mm, 'orientation_deg': actual.orientation_deg,
                'target_mm': final, 'actual_displacement_mm': math.dist(start.position_mm, actual.position_mm),
                'completed_steps': details['completed_steps']}
    except BaseException as exc:
        if isinstance(exc, DirectionUnavailable):
            details['first_error_feedback'] = exc.feedback
        # Best effort only when fresh, healthy feedback still allows control.
        # A rejected target must never be followed by another target/reset.
        held = hold_cartesian(driver, speed) if details['command_attempted'] else False
        details.update(error=str(exc), measured_hold_sent=held)
        try:
            state = driver.get_state()
            details.update(controller_status=state.arm_status, error_code=state.error_code,
                           joints_deg=state.joints_deg, joint_feedback_time=state.feedback_time)
            observed = end_pose(driver)
            details.update(position_mm=observed.position_mm,
                           actual_displacement_mm=math.dist(start.position_mm, observed.position_mm))
        except Exception:
            details['position_available'] = False
        try:
            report(dict(details))
        except Exception:
            pass  # Diagnostic storage must not hide the original failure.
        if isinstance(exc, DirectionUnavailable):
            raise DirectionUnavailable(f'Cannot continue {direction} from this pose while keeping the gripper orientation. '
                'The controller rejected the target; movement may be partial. '
                'Check the robot state before retrying. No reset was sent.') from exc
        raise
