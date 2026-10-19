"""Explicit seamless teaching exit using the existing pinned PiPER SDK.

Reference: agilexrobotics/agx_arm_ros@bc485ab8e3d25abbcd3a1b58c105be12bad5dadc
supports seamless switching on Piper >= 1.8.5 and uses move_j(measured_q)
for position holding. Never fall back to ResetPiper or a zero-pose target.
"""
import re
import time

from agilex.piper.intent_motion import checked_state, fresh


def supports_seamless_exit(version):
    match = re.fullmatch(r'S-V(\d+)\.(\d+)-(\d+)', version or '')
    if not match:
        return False
    numbers = tuple(map(int, match.groups()))
    return numbers[0] == 1 and numbers >= (1, 8, 5)


def _state(driver, *, ready=False):
    state = checked_state(driver, ready=ready, allow_teaching=not ready)
    if len(state.enabled) != 6 or not all(state.enabled):
        raise RuntimeError('Exit teaching requires all six arm motors to be enabled; no motors were enabled automatically.')
    if not state.control_mode.startswith(('TEACHING_MODE', 'CAN_CTRL')):
        raise RuntimeError('Exit teaching requires teaching mode or CAN control.')
    return state


def _hold_current_if_ready(driver, require_permission):
    try:
        with driver._send_lock:
            actual = _state(driver, ready=True)
            if not fresh(driver.arm.GetArmStatus().time_stamp):
                return
            target = tuple(round(q * 1000) for q in actual.joints_deg)
            if target != driver._bounded_target(target):
                return
            require_permission()
            driver.arm.MotionCtrl_2(0x01, 0x01, 1, 0x00)
            require_permission()
            driver.arm.JointCtrl(*target)
    except Exception:
        pass  # Lost feedback/ownership means a hold cannot be guaranteed.


def exit_teaching_and_hold(driver, *, on_switch=lambda: None,
                           require_permission=lambda: None,
                           clock=time.monotonic, sleep=time.sleep):
    """Hold measured joints at 1% speed; verify post-command feedback.

    Firmware lookup is an information query. Control starts only after all
    preconditions pass. Stop shares the driver's send lock and cancellation.
    """
    driver._check_cancelled()
    initial = _state(driver)
    if initial.control_mode.startswith('CAN_CTRL'):
        return {'executed': False, 'message': 'Already in CAN control; no mode switch performed.',
                'control_mode': initial.control_mode, 'motor_torque_released': False}
    # Query on this connection, never trust an old arm's cached version.
    version = driver.firmware(query=True)
    driver._check_cancelled()
    if not supports_seamless_exit(version):
        raise RuntimeError('Seamless teaching exit requires a recognized PiPER 1.x firmware version >= 1.8.5. '
                           'No reset was sent; use the robot controls for this firmware.')

    baseline = _state(driver)
    baseline_mode = baseline.control_mode
    if not baseline_mode.startswith('TEACHING_MODE'):
        raise RuntimeError('The control mode changed during the firmware check; retry after checking the arm.')
    deadline = clock() + .3
    while True:
        driver._check_cancelled()
        measured = _state(driver)
        if measured.control_mode != baseline_mode:
            raise RuntimeError('The control mode changed before teaching exit.')
        if max(abs(a-b) for a,b in zip(measured.joints_deg, baseline.joints_deg)) > .2:
            raise RuntimeError('The arm is moving; keep it stationary before exiting teaching.')
        if clock() >= deadline:
            break
        sleep(.02)

    # One target only, at SDK precision (0.001 degree); never clamp to limits.
    target = tuple(round(q * 1000) for q in measured.joints_deg)
    target_deg = tuple(q / 1000 for q in target)
    sent = False
    try:
        on_switch()
        with driver._send_lock:
            driver._check_cancelled()
            actual = _state(driver)
            if actual.control_mode != baseline_mode:
                raise RuntimeError('The control mode changed before teaching exit.')
            if max(abs(a-b) for a,b in zip(actual.joints_deg, target_deg)) > .2:
                raise RuntimeError('The arm moved before teaching exit; retry while stationary.')
            if target != driver._bounded_target(target):
                raise RuntimeError('The measured pose exceeds effective joint limits.')
            sent_at = time.time()
            require_permission()
            sent = True  # A partially transmitted mode change also needs recovery.
            driver.arm.MotionCtrl_2(0x01, 0x01, 1, 0x00)
            require_permission()
            driver.arm.JointCtrl(*target)

        deadline = clock() + 5
        stable_since = None
        can_seen = False
        max_drift = 0.
        while clock() < deadline:
            driver._check_cancelled()
            require_permission()
            actual = _state(driver, ready=can_seen)
            drift = max(abs(a-b) for a,b in zip(actual.joints_deg, target_deg))
            max_drift = max(max_drift, drift)
            if drift > 1:
                raise RuntimeError('The arm moved more than 1 degree during teaching exit; the hold was not confirmed.')
            in_can = actual.control_mode.startswith('CAN_CTRL')
            can_seen = can_seen or in_can
            status = driver.arm.GetArmStatus()
            if not fresh(status.time_stamp):
                raise RuntimeError('Controller feedback is stale during teaching exit.')
            confirmed = (in_can and actual.feedback_time > sent_at and status.time_stamp > sent_at
                         and int(status.arm_status.motion_status) == 0 and drift <= .5)
            if confirmed:
                if stable_since is None:
                    stable_since = clock()
                elif clock() - stable_since >= .5:
                    driver._check_cancelled()
                    return {'executed': True, 'message': 'Teaching mode exited; current position held.',
                            'firmware': version, 'control_mode': actual.control_mode,
                            'target_joints_deg': target_deg, 'joints_deg': actual.joints_deg,
                            'max_drift_deg': max_drift, 'motor_torque_released': False}
            else:
                stable_since = None
            # The first command requests the switch once. While its feedback
            # is pending, do not retry a takeover or send further targets.
            if in_can:
                with driver._send_lock:
                    driver._check_cancelled()
                    current = _state(driver, ready=True)
                    if max(abs(a-b) for a,b in zip(current.joints_deg, target_deg)) > 1:
                        raise RuntimeError('The arm moved unexpectedly during teaching exit.')
                    require_permission()
                    driver.arm.JointCtrl(*target)
            sleep(.05)
        raise RuntimeError('Teaching exit timed out; CAN mode and a stable position were not confirmed.')
    except BaseException:
        if sent:
            # At most one measured-pose hold, and only if feedback says CAN
            # control is still healthy and enabled. Never re-enter from teaching.
            _hold_current_if_ready(driver, require_permission)
        raise
