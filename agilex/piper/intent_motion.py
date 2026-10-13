"""Small base-frame translations using the pinned manufacturer's MOVE_L."""
from dataclasses import dataclass
import math
import time


DIRECTIONS = {'forward': (1, 0, 0), 'backward': (-1, 0, 0),
              'left': (0, 1, 0), 'right': (0, -1, 0),
              'up': (0, 0, 1), 'down': (0, 0, -1)}
POSE_FIELDS = ('X_axis', 'Y_axis', 'Z_axis', 'RX_axis', 'RY_axis', 'RZ_axis')


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


def checked_state(driver, *, ready=False):
    state = driver.get_state()
    if not fresh(state.feedback_time):
        raise RuntimeError('Joint feedback is stale; check the CAN connection.')
    if len(state.joints_deg) != 6 or not all(math.isfinite(v) for v in state.joints_deg):
        raise RuntimeError('Joint feedback is invalid.')
    if any(state.enabled) and not all(state.enabled):
        raise RuntimeError('Some arm motors are disabled; check the arm before retrying.')
    if state.error_code or state.motor_faults or not state.arm_status.startswith('NORMAL'):
        raise RuntimeError('The arm reports a fault: ' + state.arm_status)
    if not state.teach_status.startswith(('DISABLED', 'STOP_RECORDING', 'TERMINATE_EXECUTION')):
        raise RuntimeError('Teaching or trajectory playback is active; stop it on the robot first.')
    if not state.control_mode.startswith(('STANDBY', 'CAN_CTRL')):
        raise RuntimeError('The arm is in an unsupported mode: ' + state.control_mode)
    for i, (q, bounds) in enumerate(zip(state.joints_deg, driver.joint_bounds_deg()), 1):
        if not bounds[0] <= q <= bounds[1]:
            raise RuntimeError(f'Joint {i} is outside its effective limits; reposition the arm first.')
    if ready and (not state.control_mode.startswith('CAN_CTRL') or not all(state.enabled)):
        raise RuntimeError('The arm lost its ready state; the action was cancelled.')
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


def move_cartesian(driver, direction, distance, speed, *, clock=time.monotonic, sleep=time.sleep):
    checked_state(driver, ready=True)
    start = end_pose(driver)
    vector = DIRECTIONS[direction]
    target = EndPose(tuple(p + distance * d for p, d in zip(start.position_mm, vector)),
                     start.orientation_deg, start.feedback_time)
    deadline = clock() + 30
    stable_since = None
    sent_at = None
    try:
        while clock() < deadline:
            driver._check_cancelled()
            checked_state(driver, ready=True)
            actual = end_pose(driver)
            # Joint and end-position frames are checked independently. Status
            # must also be received AFTER our first command, never a cached
            # 'reached' flag from an earlier action.
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
                    return {'executed': True, 'position_mm': actual.position_mm,
                            'orientation_deg': actual.orientation_deg, 'target_mm': target.position_mm}
            else:
                stable_since = None
            _send(driver, target, speed)
            if sent_at is None:
                sent_at = time.time()
            sleep(.05)
        raise RuntimeError('Move timed out after 30 seconds; target completion was not confirmed.')
    except BaseException:
        hold_cartesian(driver, speed)
        raise
