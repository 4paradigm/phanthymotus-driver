"""Experimental body-IMU distance estimation, independent of camera and ROS.

SDK reads have no source timestamp. Poll timing and unchanged-data checks are
only fault heuristics, not proof of fresh measurements or physical accuracy.
"""
from collections import deque
import json
import math
from pathlib import Path
import tempfile
import time

from leg_odometry import LEG_SLIP_SCALE, LegOdometry, joint_angles


def decode_imu(sample):
    """SDK xyzw attitude, body specific force in m/s², gyro in rad/s."""
    q = tuple(float(sample.ori[i]) for i in range(4))
    acc = tuple(float(sample.linear_acc[i]) for i in range(3))
    gyro = tuple(float(sample.angular_vel[i]) for i in range(3))
    if not all(math.isfinite(v) for v in (*q, *acc, *gyro)):
        raise ValueError('IMU contains non-finite data')
    norm = math.hypot(*q)
    if not 0.8 <= norm <= 1.2:
        raise ValueError(f'IMU quaternion norm {norm:.3f} outside experimental range [0.8, 1.2]')
    acceleration_norm = math.hypot(*acc)
    if acceleration_norm > 50:
        raise ValueError(f'IMU acceleration norm {acceleration_norm:.3f} exceeds experimental limit 50')
    x, y, z, w = (v/norm for v in q)
    ax, ay, az = acc
    tx, ty, tz = 2*(y*az-z*ay), 2*(z*ax-x*az), 2*(x*ay-y*ax)
    world = (ax+w*tx+y*tz-z*ty, ay+w*ty+z*tx-x*tz, az+w*tz+x*ty-y*tx)
    yaw = math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))
    if 1-2*(x*x+y*y) < math.cos(math.radians(20)):
        raise ValueError('IMU tilt exceeds the experimental 20 degree limit')
    return world, gyro, yaw, (*q, *acc, *gyro)


class ImuOdometry:
    def __init__(self):
        self.ready = False
        self.window = deque()
        self.started = self.previous_time = None
        self.baseline = self.previous_acc = (0.0, 0.0, 0.0)
        self.position = self.velocity = (0.0, 0.0, 0.0)
        self.acceleration = (0.0, 0.0, 0.0)
        self.gyro = (0.0, 0.0, 0.0)
        self.yaw = 0.0
        self.signature = None

    def observe(self, sample, now):
        world, self.gyro, self.yaw, self.signature = decode_imu(sample)
        dt = None if self.previous_time is None else now-self.previous_time
        if not math.isfinite(now) or (dt is not None and not 0 < dt <= 0.1):
            raise ValueError('IMU sampling interval invalid or longer than 0.1 s')
        self.previous_time = now
        if self.started is None:
            self.started = now
        if not self.ready:
            if now-self.started > 6:
                raise ValueError('IMU calibration timed out: keep the robot stationary')
            # Accept only gravity-inclusive SI readings. Do not silently guess
            # between g, m/s² and an already gravity-removed acceleration field.
            if not 8 <= math.hypot(*world) <= 11 or math.hypot(*world[:2]) > 0.6:
                raise ValueError('IMU calibration requires gravity-inclusive m/s² and matching attitude')
            if math.hypot(*self.gyro) > 0.08:
                self.window.clear()
                return
            self.window.append((now, world, self.yaw))
            while len(self.window) > 1 and now-self.window[1][0] >= 2:
                self.window.popleft()
            if len(self.window) < 20 or now-self.window[0][0] < 2-1e-9:
                return
            spread = [max(row[1][i] for row in self.window)-min(row[1][i] for row in self.window)
                      for i in range(3)]
            yaw0 = self.window[0][2]
            if (max(spread) > 0.25 or any(abs(angle_delta(row[2], yaw0)) > math.radians(2)
                                         for row in self.window)):
                return
            self.baseline = tuple(sum(row[1][i] for row in self.window)/len(self.window)
                                  for i in range(3))
            self.ready = True
            self.window.clear()
            return
        self.acceleration = tuple(world[i]-self.baseline[i] for i in range(3))
        velocity = tuple(self.velocity[i]+(self.previous_acc[i]+self.acceleration[i])*dt/2
                         for i in range(3))
        self.position = tuple(self.position[i]+(self.velocity[i]+velocity[i])*dt/2 for i in range(3))
        self.velocity = velocity
        self.previous_acc = self.acceleration


def angle_delta(a, b):
    return math.atan2(math.sin(a-b), math.cos(a-b))


def distance_timeout(distance_m):
    # Calibration <=6 s + motion budget (leg path: 5+10*d+5, capped 30 s) +
    # endpoint observation <=4 s. The global deadline is only a coarse bound;
    # the worker itself enforces the calibration, motion and endpoint deadlines
    # separately so a long calibration cannot eat the motion budget.
    return 10.0+min(30.0, 5.0+10.0*distance_m+5.0)


def movement_direction(value=''):
    """Positive requested distance along a starting body axis."""
    if not isinstance(value, str):
        raise ValueError('direction must be 前进/后退/左移/右移 or forward/backward/left/right')
    name = value.strip().lower() or 'forward'
    name = {'前进': 'forward', '后退': 'backward', '左移': 'left', '右移': 'right'}.get(name, name)
    axes = {'forward': (1, 0), 'backward': (-1, 0), 'left': (0, 1), 'right': (0, -1)}
    if name not in axes:
        raise ValueError('direction must be 前进/后退/左移/右移 or forward/backward/left/right')
    return name, *axes[name]


# Normalised `publish_cmd()` argument -> chassis speed in m/s, measured on this
# Bumi on 2026-10-09 (README.md, "标定 loco_servo"): vx=1.0 -> 0.55 m/s and
# |vy|=1.0 -> 0.20 m/s. The distance worker commands normalised values, so this
# table turns its own command history into a travel estimate that is
# independent of the IMU integration.
#
# The integration below remains the measured feedback path. This table is the
# feed-forward cross-check: it cannot observe slip or a surface slower than the
# calibration, but it also cannot inherit the SDK's timestamp-free sampling
# error. `run_distance` compares the two; when they contradict, the worker
# stops trusting the IMU displacement and continues on this estimate.
FULL_SCALE_MPS = {'forward': 0.55, 'backward': 0.55, 'left': 0.20, 'right': 0.20}

# A legged chassis does not creep: below these normalised thresholds the legs
# do nothing (README.md, same calibration). Between the deadband and 1.0 the
# speed is measured to rise roughly linearly to the full scale, so the
# feed-forward estimate must not use full_scale*command - that linear model
# over-estimated the 2026-10-21 0.6-command forward move by about 1.7x.
DEADBAND_NORMALISED = {'forward': 0.45, 'backward': 0.45, 'left': 0.60, 'right': 0.60}


def travel_scale(direction, full_scale=None):
    """Full-scale m/s for normalised command 1.0 along one travel direction."""
    table = FULL_SCALE_MPS if full_scale is None else full_scale
    try:
        value = float(table[direction])
    except (KeyError, TypeError, ValueError):
        raise ValueError(f'no calibrated full-scale speed for direction {direction!r}')
    if not math.isfinite(value) or value <= 0:
        raise ValueError(f'calibrated full-scale speed for {direction!r} must be positive and finite')
    return value


def travel_speed(command, direction, full_scale=None):
    """Feed-forward speed in m/s for one normalised command, deadband-aware."""
    full_scale = travel_scale(direction, full_scale)
    deadband = DEADBAND_NORMALISED[direction]
    command = abs(float(command))
    if command <= deadband:
        return 0.0
    return full_scale*(command-deadband)/(1.0-deadband)


def lateral_limit(distance_m):
    """Path-deviation limit before a distance move is stopped.

    The pure IMU lateral position inherits the same sampling/attitude error as
    the forward estimate; the 2026-10-21 0.8 m forward run estimated 0.155 m of
    lateral offset where the operator measured almost none. The limit therefore
    scales with the requested move instead of aborting every long run.
    """
    return max(0.15, 0.5*distance_m)


# Gait-phase finish. The walking controller stops mid-gait: one foot can end
# in front of the other. Once the distance target is reached, keep commanding
# the same velocity until the two contacts are fore-aft aligned in double
# support, then request zero. The controller finishes the current step after
# zero, so the exact final stance still needs a lead time measured on hardware.
# The walking controller ignores yaw commands below about 0.30 normalised: a
# constant -0.25 command turned the robot by only ~3 deg over 2.5 s (gait sway),
# while -0.5 turned it ~29 deg in 1.5 s. A proportional correction of a few
# degrees therefore does nothing; hold heading with strong pulses once the
# error leaves a small band.
HEADING_BAND_DEG = 1.5
HEADING_PULSE = 0.5
# The sideways gait couples strongly into yaw: the right 1 m run kept turning
# right even with a continuous +0.5 left command, so sideways moves get more
# authority. Measured response to a 0.5 pulse is ~19 deg/s while walking.
HEADING_PULSE_LATERAL = 0.8

PARALLEL_FINISH_TOLERANCE_M = 0.04
PARALLEL_FINISH_TIMEOUT_S = 0.4
# The finish walks at the walking speed (~0.35 m/s); a 0.5 s window would add
# ~0.17 m, so the window and the allowed extra must stay consistent or the
# overshoot guard would fire during the finish itself.
PARALLEL_FINISH_ALLOWANCE_M = 0.15
# The finish may walk past the target only this far; the gait phase arrives
# once per step, so waiting for it can cost ~0.1-0.15 m. Distance accuracy wins
# over feet alignment beyond this cap.
PARALLEL_FINISH_MAX_EXTRA_M = 0.05


def travel_tolerance(distance_m):
    """Allowance before the command integration counts as a fault."""
    return max(0.25, 0.20*distance_m+0.10)


def _write_trace(samples, commands, result):
    # Write only after the final brake, never do disk I/O in the control loop.
    folder = Path(tempfile.gettempdir()) / 'bumi-imu-distance'
    folder.mkdir(exist_ok=True)
    with tempfile.NamedTemporaryFile(mode='w', dir=folder, prefix='imu-', suffix='.json', delete=False) as stream:
        result['imu_trace_path'] = stream.name
        json.dump(dict(samples=samples, command_requests=commands, result=result), stream)
    for old in sorted(folder.glob('imu-*.json'), key=lambda p: p.stat().st_mtime, reverse=True)[5:]:
        old.unlink()


def run_distance(owner, distance_m, direction='', full_scale=None, slip_scale=None):
    direction, body_x, body_y = movement_direction(direction)
    estimator = ImuOdometry()
    # The production LocoPlugin exposes the 21 SDK joint positions; those drive
    # the distance estimate. Owners without joint access (unit-test plants,
    # older bundles) keep the integrated-IMU path.
    leg = (LegOdometry(slip_scale=(LEG_SLIP_SCALE[direction] if slip_scale is None
                                   else float(slip_scale)))
           if hasattr(owner, '_read_distance_joints') else None)
    started = time.monotonic()
    motion_started = brake_started = quiet_since = None
    last_signature = None
    changed_at = started
    progress_at, last_progress = started, 0.0
    moved = 0.0
    source_position, source_velocity = estimator.position, estimator.velocity
    dead_reckoned_m = 0.0
    last_joints = None
    joint_updates = 0
    last_command, last_command_at = 0.0, None
    finishing = False
    finish_started = None
    finish_phase = 'not_reached'
    finish_overshoot = 0.0
    samples, commands = [], []
    def publish(vx, vyaw=0, *, vy=0):
        if vy:
            owner._distance_publish(vx, vyaw, vy=vy)
        else:
            owner._distance_publish(vx, vyaw)
        commands.append(dict(received_mono_s=time.monotonic(), velocity=[vx, vy, vyaw]))

    result = dict(state='error', requested_distance_m=distance_m, requested_direction=direction,
                  feedback_source='leg_odometry_experimental', accuracy_validated=False,
                  source_timestamp_available=False, stationary_assumed=False)
    try:
        publish(0)
        while True:
            if owner._rotate_stop_event.is_set():
                result['state'] = 'stopped'
                break
            problem = owner._rotate_mode_error()
            if problem:
                raise ValueError(problem['error'])
            read_started = time.monotonic()
            raw = owner._read_distance_imu()
            joints = owner._read_distance_joints() if leg is not None else None
            now = time.monotonic()
            samples.append(dict(received_mono_s=now, read_duration_s=now-read_started,
                                orientation_xyzw=[float(raw.ori[i]) for i in range(4)],
                                linear_acceleration_mps2=[float(raw.linear_acc[i]) for i in range(3)],
                                angular_velocity_rad_s=[float(raw.angular_vel[i]) for i in range(3)]))
            if not estimator.ready and now-started > 6.0:
                raise ValueError('IMU calibration timed out')
            estimator.observe(raw, now)
            if leg is not None:
                # Once braking starts the stance foot rolls/slips; keep the
                # estimate that requested the brake instead of integrating noise.
                # Keep integrating through the stop: the coast after the
                # brake is real travel and must be in the reported endpoint.
                leg.observe(joints, raw.ori, now)
                values = joint_angles(joints)
                if values != last_joints:
                    last_joints, joint_updates = values, joint_updates+1
                samples[-1]['leg_joint_positions'] = [round(v, 6) for v in values]
                feet = getattr(leg, 'feet', None) or []
                samples[-1]['leg_feet'] = [[round(v, 6) for v in foot] for foot in feet]
            source_position = leg.position if leg is not None else estimator.position
            source_velocity = leg.velocity if leg is not None else estimator.velocity
            samples[-1].update(phase='calibration' if not estimator.ready else
                              ('braking' if brake_started is not None else 'driving'),
                              position_m=source_position, velocity_mps=source_velocity,
                              acceleration_mps2=estimator.acceleration,
                              imu_position_m=estimator.position)
            if not estimator.ready:
                publish(0)
                time.sleep(0.02)
                continue
            if motion_started is None:
                motion_started = progress_at = changed_at = now
                last_command_at = now
                if leg is not None:
                    # Calibration-phase leg samples must not become the motion
                    # baseline: the first motion sample starts a fresh window.
                    leg.reset_baseline()
                origin_yaw = estimator.yaw
                cy, sy = math.cos(origin_yaw), math.sin(origin_yaw)
                c, s = cy*body_x-sy*body_y, sy*body_x+cy*body_y
                result['start_yaw_deg'] = math.degrees(origin_yaw)
                result['stationary_world_acceleration_mps2'] = estimator.baseline
                print('[loco_target] IMU experimental calibration complete; distance is an estimate', flush=True)
            if estimator.signature != last_signature:
                changed_at, last_signature = now, estimator.signature
            if brake_started is None and now-changed_at > 0.5:
                raise ValueError('IMU data unchanged during motion; source freshness cannot be verified')
            x, y, _ = source_position
            moved = x*c+y*s
            speed = source_velocity[0]*c+source_velocity[1]*s
            yaw_error = angle_delta(origin_yaw, estimator.yaw)
            lateral = abs(-x*s+y*c)
            if brake_started is None:
                # Independent feed-forward travel estimate from the worker's
                # own normalised commands through the measured command->speed
                # curve. It cannot observe slip or the actual displacement, so
                # it is a sanity bound only: a disagreement stops the move with
                # an error instead of being used to finish it.
                dead_reckoned_m += travel_speed(last_command, direction, full_scale)*(now-last_command_at)
                last_command_at = now
                samples[-1]['dead_reckoned_m'] = dead_reckoned_m
                tolerance = travel_tolerance(distance_m)
                if dead_reckoned_m-moved > tolerance:
                    raise ValueError(
                        f'dead-reckoned travel {dead_reckoned_m:.3f} m leads the IMU '
                        f'estimate {moved:.3f} m by more than {tolerance:.3f} m for a '
                        f'{distance_m:.3f} m request')
            remaining = distance_m-moved
            if moved < -0.05:
                raise ValueError('IMU estimated progress is opposite to requested direction by more than 0.05 m')
            if (lateral > lateral_limit(distance_m) or abs(yaw_error) > math.radians(20)
                    or math.hypot(*source_velocity) > 1.2):
                raise ValueError('IMU estimated motion exceeds direction or speed limits')
            phase_active = leg is not None and (finishing or finish_phase in ('parallel', 'timeout'))
            overshoot_limit = PARALLEL_FINISH_ALLOWANCE_M if phase_active else 0.05
            if remaining < -overshoot_limit:
                raise ValueError('IMU estimated overshoot exceeds 0.05 m')
            if brake_started is None:
                time_limit = (min(30.0, 5.0+10.0*distance_m+5.0) if leg is not None
                              else min(20.0, 5.0+10.0*distance_m))
                if now-motion_started > time_limit:
                    raise ValueError('IMU motion time limit reached')
                if moved-last_progress >= 0.01:
                    last_progress, progress_at = moved, now
                else:
                    # The leg estimate can legitimately stall while the feet
                    # slide backwards; the robot keeps walking. Give the leg
                    # path a longer stall window than the IMU path.
                    stall_limit = 8.0 if leg is not None else 3.0
                    if now-progress_at > stall_limit:
                        raise ValueError('IMU estimated distance made no progress')
                brake_gap = max(0.025, min(0.10, max(0, speed)*0.15))
                if finishing or remaining <= brake_gap:
                    if not finishing:
                        finishing, finish_started = True, now
                    parallel = (leg is not None and getattr(leg, 'double_support', False)
                                and abs(getattr(leg, 'foot_gap', 1.0)) <= PARALLEL_FINISH_TOLERANCE_M)
                    capped = moved - distance_m >= PARALLEL_FINISH_MAX_EXTRA_M
                    if (leg is None or parallel or capped
                            or now-finish_started >= PARALLEL_FINISH_TIMEOUT_S):
                        finish_phase = ('immediate' if leg is None else
                                        ('parallel' if parallel else
                                         ('capped' if capped else 'timeout')))
                        finish_overshoot = max(0.0, moved-distance_m)
                        brake_started, finishing = now, False
                if brake_started is None:
                    # Heading hold with pulses above the walking deadband. The
                    # gait drifts left while walking forward; small proportional
                    # corrections never reach the controller, so a pulse is sent
                    # whenever the error leaves the band and zero inside it.
                    error_deg = math.degrees(yaw_error)
                    pulse = HEADING_PULSE_LATERAL if body_y else HEADING_PULSE
                    if error_deg < -HEADING_BAND_DEG:
                        turn = -pulse
                    elif error_deg > HEADING_BAND_DEG:
                        turn = pulse
                    else:
                        turn = 0.0
                    # The lateral deadband is 0.60 normalised, so 0.9 is needed
                    # to actually side-step; 0.65 barely moved.
                    command = 0.9 if body_y else (0.6 if remaining > 0.2 else 0.5)
                    publish(command*body_x, turn, vy=command*body_y)
                    last_command, last_command_at = command, now
            if brake_started is not None:
                publish(0)
                # A quiet accelerometer alone cannot distinguish rest from
                # uniform motion. Require zero command dwell AND low estimated
                # speed; even this is an assumption, not a measured stop.
                quiet = (now-brake_started >= 1.0 and math.hypot(*source_velocity) <= 0.10
                         and max(abs(v) for v in estimator.acceleration) <= 0.2
                         and math.hypot(*estimator.gyro) <= 0.08)
                if not quiet:
                    quiet_since = None
                elif quiet_since is None:
                    quiet_since = now
                elif now-quiet_since >= 0.8:
                    result['pre_zero_velocity_mps'] = source_velocity
                    estimator.velocity = (0.0, 0.0, 0.0)
                    result['stationary_assumed'] = True
                    band = 0.10 if leg is not None else 0.05
                    result['endpoint_band_m'] = band
                    if leg is not None:
                        if remaining > band or remaining < -PARALLEL_FINISH_ALLOWANCE_M:
                            raise ValueError('stopped outside the estimated target band')
                    elif abs(remaining) > band:
                        raise ValueError('stopped outside the estimated target band')
                    result['state'] = 'done'
                    break
                if now-brake_started > 4:
                    raise ValueError('IMU endpoint did not meet assumed-stationary conditions')
            time.sleep(0.02)
    except Exception as exc:
        result['error'] = f'{type(exc).__name__}: {exc}'
    finally:
        publish(0)
    result.update(estimated_distance_m=moved, estimated_error_m=distance_m-moved,
                  estimated_position_m=source_position, estimated_velocity_mps=source_velocity,
                  imu_position_m=estimator.position, imu_velocity_mps=estimator.velocity,
                  distance_source='leg_odometry' if leg is not None else 'imu_integrated',
                  dead_reckoned_m=dead_reckoned_m,
                  leg_samples=(getattr(leg, 'samples', None) if leg is not None else None),
                  leg_stance_samples=(getattr(leg, 'stance_samples', None) if leg is not None else None),
                  leg_joint_updates=(joint_updates if leg is not None else None),
                  leg_slip_scale=(getattr(leg, 'slip_scale', None) if leg is not None else None),
                  finish_phase=finish_phase,
                  finish_overshoot_m=finish_overshoot,
                  final_foot_gap_m=(getattr(leg, 'foot_gap', None) if leg is not None else None),
                  elapsed_s=time.monotonic()-started)
    try:
        _write_trace(samples, commands, result)
    except OSError as exc:
        result['imu_trace_error'] = str(exc)
    return result

def run_target(owner, x_m, y_m, full_scale=None, slip_scale=None):
    """Drive to a point (x forward, y left, metres) in the start body frame.

    Uses forward motion plus heading control only: turn in place toward the
    point when the bearing error is large, otherwise walk forward while the
    pulse law keeps the heading on the bearing. Slip correction is the forward
    factor (the path is mostly along the body x axis). Arrival is a position
    tolerance, not a heading: the final heading is reported, not requested.
    """
    target_x, target_y = float(x_m), float(y_m)
    target_distance = math.hypot(target_x, target_y)
    if (not all(math.isfinite(v) for v in (target_x, target_y))
            or not 0.1 <= target_distance <= 3.0):
        raise ValueError('target must be a finite point 0.1-3 m from the start')
    estimator = ImuOdometry()
    leg_slip = LEG_SLIP_SCALE['forward'] if slip_scale is None else float(slip_scale)
    leg = (LegOdometry(slip_scale=leg_slip)
           if hasattr(owner, '_read_distance_joints') else None)
    started = time.monotonic()
    motion_started = brake_started = quiet_since = None
    last_signature = None
    changed_at = started
    closest, progress_at = target_distance, started
    last_joints = None
    joint_updates = 0
    samples, commands = [], []

    def publish(vx, vyaw=0):
        owner._distance_publish(vx, vyaw)
        commands.append(dict(received_mono_s=time.monotonic(), velocity=[vx, 0.0, vyaw]))

    result = dict(state='error', action='target',
                  requested_target_x_m=target_x, requested_target_y_m=target_y,
                  requested_distance_m=target_distance,
                  feedback_source='leg_odometry_experimental',
                  accuracy_validated=False, source_timestamp_available=False,
                  stationary_assumed=False)
    source_position, source_velocity = estimator.position, estimator.velocity
    try:
        publish(0)
        while True:
            if owner._rotate_stop_event.is_set():
                result['state'] = 'stopped'
                break
            problem = owner._rotate_mode_error()
            if problem:
                raise ValueError(problem['error'])
            read_started = time.monotonic()
            raw = owner._read_distance_imu()
            joints = owner._read_distance_joints() if leg is not None else None
            now = time.monotonic()
            samples.append(dict(received_mono_s=now, read_duration_s=now-read_started,
                                orientation_xyzw=[float(raw.ori[i]) for i in range(4)],
                                linear_acceleration_mps2=[float(raw.linear_acc[i]) for i in range(3)],
                                angular_velocity_rad_s=[float(raw.angular_vel[i]) for i in range(3)]))
            time_limit = min(60.0, 12.0+20.0*target_distance)
            if not estimator.ready and now-started > 6.0:
                raise ValueError('target calibration timed out')
            estimator.observe(raw, now)
            if leg is not None:
                # Keep integrating through the stop: the coast after the
                # brake is real travel and must be in the reported endpoint.
                leg.observe(joints, raw.ori, now)
                values = joint_angles(joints)
                if values != last_joints:
                    last_joints, joint_updates = values, joint_updates+1
                samples[-1]['leg_joint_positions'] = [round(v, 6) for v in values]
                feet = getattr(leg, 'feet', None) or []
                samples[-1]['leg_feet'] = [[round(v, 6) for v in foot] for foot in feet]
            source_position = leg.position if leg is not None else estimator.position
            source_velocity = leg.velocity if leg is not None else estimator.velocity
            samples[-1].update(phase='calibration' if not estimator.ready else
                              ('braking' if brake_started is not None else 'driving'),
                              position_m=source_position, velocity_mps=source_velocity,
                              acceleration_mps2=estimator.acceleration,
                              imu_position_m=estimator.position)
            if not estimator.ready:
                publish(0)
                time.sleep(0.02)
                continue
            if motion_started is None:
                motion_started = progress_at = changed_at = now
                if leg is not None:
                    leg.reset_baseline()
                origin_yaw = estimator.yaw
                result['start_yaw_deg'] = math.degrees(origin_yaw)
                result['stationary_world_acceleration_mps2'] = estimator.baseline
                print('[loco_target] target calibration complete; position is an estimate', flush=True)
            if estimator.signature != last_signature:
                changed_at, last_signature = now, estimator.signature
            if brake_started is None and now-changed_at > 0.5:
                raise ValueError('IMU data unchanged during motion; source freshness cannot be verified')
            x, y, _ = source_position
            cy, sy = math.cos(origin_yaw), math.sin(origin_yaw)
            body_x = x*cy + y*sy
            body_y = -x*sy + y*cy
            to_x, to_y = target_x-body_x, target_y-body_y
            distance = math.hypot(to_x, to_y)
            bearing = math.atan2(to_y, to_x)
            heading = angle_delta(estimator.yaw, origin_yaw)
            heading_error = angle_delta(bearing, heading)
            if brake_started is None:
                if now-motion_started > time_limit:
                    raise ValueError('target move time limit reached')
                if distance < closest-0.01:
                    closest, progress_at = distance, now
                elif now-progress_at > 8.0:
                    raise ValueError('target move made no progress toward the point')
                if math.hypot(*source_velocity) > 1.2:
                    raise ValueError('target motion exceeds speed limits')
                if distance <= 0.10:
                    brake_started = now
                else:
                    err_deg = math.degrees(heading_error)
                    if err_deg < -20.0:
                        turn = -0.8
                    elif err_deg < -1.5:
                        turn = -0.5
                    elif err_deg > 20.0:
                        turn = 0.8
                    elif err_deg > 1.5:
                        turn = 0.5
                    else:
                        turn = 0.0
                    if abs(err_deg) > 60.0:
                        publish(0, turn)
                    else:
                        publish(0.6 if distance > 0.3 else 0.5, turn)
            if brake_started is not None:
                publish(0)
                quiet = (now-brake_started >= 1.0
                         and math.hypot(*source_velocity) <= 0.10
                         and max(abs(v) for v in estimator.acceleration) <= 0.2
                         and math.hypot(*estimator.gyro) <= 0.08)
                if not quiet:
                    quiet_since = None
                elif quiet_since is None:
                    quiet_since = now
                elif now-quiet_since >= 0.8:
                    result['pre_zero_velocity_mps'] = source_velocity
                    estimator.velocity = (0.0, 0.0, 0.0)
                    result['stationary_assumed'] = True
                    final_error = math.hypot(target_x-body_x, target_y-body_y)
                    result['final_position_error_m'] = final_error
                    result['final_heading_deg'] = math.degrees(heading)
                    result['estimated_distance_m'] = final_error
                    result['estimated_error_m'] = final_error
                    if final_error > 0.12:
                        raise ValueError('stopped outside the target point')
                    result['state'] = 'done'
                    break
                if now-brake_started > 4:
                    raise ValueError('target endpoint did not meet assumed-stationary conditions')
            time.sleep(0.02)
    except Exception as exc:
        result['error'] = f'{type(exc).__name__}: {exc}'
    finally:
        publish(0)
    result.update(
        final_position_error_m=result.get('final_position_error_m'),
        final_heading_deg=result.get('final_heading_deg'),
        estimated_position_m=source_position,
        estimated_velocity_mps=source_velocity,
        distance_source='leg_odometry' if leg is not None else 'imu_integrated',
        leg_samples=(getattr(leg, 'samples', None) if leg is not None else None),
        leg_stance_samples=(getattr(leg, 'stance_samples', None) if leg is not None else None),
        leg_joint_updates=(joint_updates if leg is not None else None),
        leg_slip_scale=(getattr(leg, 'slip_scale', None) if leg is not None else None),
        elapsed_s=time.monotonic()-started)
    try:
        _write_trace(samples, commands, result)
    except OSError as exc:
        result['imu_trace_error'] = str(exc)
    return result

