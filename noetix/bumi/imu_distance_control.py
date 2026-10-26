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


# Target walking heading control: the walking controller ignores yaw commands
# below ~0.30, so hold the heading with strong pulses once outside the band.
HEADING_BAND_DEG = 1.5
HEADING_PULSE = 0.5
# Turn-first threshold: above it, turn in place to <=2 deg (leg pose frozen
# during the turn) and then walk; walk-and-turn up to the threshold.
TARGET_TURN_FIRST_DEG = 45.0
TARGET_TURN_DONE_DEG = 2.0
TARGET_REALIGN_DEG = 12.0
# After the position arrives, turn in place to face the target direction. This
# is a comfort alignment: best effort, never a reason to fail a valid position.
TARGET_FINAL_ALIGN_DEG = 2.0
TARGET_FINAL_ALIGN_TIMEOUT_S = 4.0
# Walk this far past the estimated target point: the leg estimate reads ~7%
# short of the physical travel, so stopping exactly at the estimate leaves the
# robot visibly short.
TARGET_ARRIVAL_CREEP_M = 0.05
# The leg estimate is the arrival feedback and reads ~0.10-0.16 m at a stopped
# 1 m target; accept 0.20 m and report the actual value.
TARGET_ARRIVAL_MAX_ERROR_M = 0.20


def travel_tolerance(distance_m):
    """Allowance before the command integration counts as a fault."""
    return max(0.45, 0.25*distance_m+0.20)


def _write_trace(samples, commands, result):
    # Write only after the final brake, never do disk I/O in the control loop.
    folder = Path(tempfile.gettempdir()) / 'bumi-imu-distance'
    folder.mkdir(exist_ok=True)
    with tempfile.NamedTemporaryFile(mode='w', dir=folder, prefix='imu-', suffix='.json', delete=False) as stream:
        result['imu_trace_path'] = stream.name
        json.dump(dict(samples=samples, command_requests=commands, result=result), stream)
    for old in sorted(folder.glob('imu-*.json'), key=lambda p: p.stat().st_mtime, reverse=True)[5:]:
        old.unlink()


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
    walking = None
    walk_aligned = False
    last_command = 0.0
    last_command_at = None
    dead_reckoned_m = 0.0
    final_align_deadline = None
    start_bearing = math.atan2(target_y, target_x)
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
                # Turning in place is not travel: freeze the leg position during
                # turn-first/realign phases and let the first walking sample
                # re-establish the baseline. Braking still integrates (the stop
                # slide is real travel and must be in the reported endpoint).
                turning = ((motion_started is not None and walking is not True
                            and brake_started is None)
                           or final_align_deadline is not None)
                leg.observe(joints, raw.ori, now, frozen=turning)
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
            along = body_x*math.cos(start_bearing) + body_y*math.sin(start_bearing)
            # Once close, the bearing to the point flips as soon as the robot
            # steps past it; keep the approach direction for the final creep.
            near_target = distance < 0.20
            reference_bearing = start_bearing if near_target else bearing
            heading_error = angle_delta(reference_bearing, heading)
            if walking is None:
                walking = abs(math.degrees(bearing)) <= TARGET_TURN_FIRST_DEG
            if brake_started is None and motion_started is not None:
                # Independent feed-forward bound from the worker's own normalised
                # forward commands through the measured command->speed curve. It
                # is a sanity bound only: if the leg/IMU estimate (which decides
                # the stop) lags the commanded travel by more than the tolerance,
                # fail closed instead of walking on a frozen/under-reporting pose.
                if last_command_at is None:
                    last_command_at = now
                dead_reckoned_m += travel_speed(last_command, 'forward',
                                                full_scale)*(now-last_command_at)
                last_command_at = now
                samples[-1]['dead_reckoned_m'] = dead_reckoned_m
                progress = along
                tolerance = travel_tolerance(target_distance)
                if dead_reckoned_m - progress > tolerance:
                    raise ValueError(
                        f'dead-reckoned travel {dead_reckoned_m:.3f} m leads the '
                        f'target estimate {progress:.3f} m by more than '
                        f'{tolerance:.3f} m for a {target_distance:.3f} m target')
            if brake_started is None:
                if now-motion_started > time_limit:
                    raise ValueError('target move time limit reached')
                if distance < closest-0.01:
                    closest, progress_at = distance, now
                elif now-progress_at > 8.0:
                    raise ValueError('target move made no progress toward the point')
                if math.hypot(*source_velocity) > 1.2:
                    raise ValueError('target motion exceeds speed limits')
                if along >= target_distance + TARGET_ARRIVAL_CREEP_M:
                    brake_started = now
                else:
                    err_deg = math.degrees(heading_error)
                    if (walking and walk_aligned and not near_target
                            and abs(err_deg) > TARGET_REALIGN_DEG):
                        # Heading drifted after the initial alignment: stop and
                        # turn in place again. At the walk start the error may
                        # still be within the turn-first band, so it must not
                        # flip a fresh walk back into an in-place turn.
                        walking = False
                        walk_aligned = False
                    elif not walking and abs(err_deg) <= TARGET_TURN_DONE_DEG:
                        walking = True
                        walk_aligned = True
                    if walking:
                        if err_deg < -1.5:
                            turn = -HEADING_PULSE
                        elif err_deg > 1.5:
                            turn = HEADING_PULSE
                        else:
                            turn = 0.0
                        if abs(err_deg) <= TARGET_TURN_DONE_DEG:
                            walk_aligned = True
                        last_command = 0.6 if distance > 0.3 else 0.5
                        publish(last_command, turn)
                    else:
                        # Turn first (or realign): no forward motion.
                        turn = math.copysign(0.8 if abs(err_deg) > 20.0 else 0.5,
                                             heading_error)
                        last_command = 0.0
                        publish(0, turn)
            if brake_started is not None:
                if final_align_deadline is None:
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
                        # Recompute from the latest sample: the braking phase
                        # keeps integrating the coast.
                        fx, fy, _ = source_position
                        final_body_x = fx*cy + fy*sy
                        final_body_y = -fx*sy + fy*cy
                        final_error = math.hypot(target_x-final_body_x,
                                                 target_y-final_body_y)
                        result['final_position_error_m'] = final_error
                        result['estimated_distance_m'] = final_error
                        result['estimated_error_m'] = final_error
                        if final_error > TARGET_ARRIVAL_MAX_ERROR_M:
                            raise ValueError('stopped outside the target point')
                        final_align_deadline = now + TARGET_FINAL_ALIGN_TIMEOUT_S
                else:
                    # Comfort alignment: face the target direction, best effort.
                    heading_now = angle_delta(estimator.yaw, origin_yaw)
                    align_error = angle_delta(start_bearing, heading_now)
                    align_deg = math.degrees(align_error)
                    result['final_heading_deg'] = math.degrees(heading_now)
                    result['final_heading_error_deg'] = align_deg
                    if abs(align_deg) <= TARGET_FINAL_ALIGN_DEG or now > final_align_deadline:
                        # The alignment turn can translate/slide the robot: the
                        # reported endpoint must be the pose after the last
                        # command, not the pose accepted before the turn.
                        fx, fy, _ = source_position
                        final_body_x = fx*cy + fy*sy
                        final_body_y = -fx*sy + fy*cy
                        final_error = math.hypot(target_x-final_body_x,
                                                 target_y-final_body_y)
                        result['final_position_error_m'] = final_error
                        result['estimated_distance_m'] = final_error
                        result['estimated_error_m'] = final_error
                        if final_error > TARGET_ARRIVAL_MAX_ERROR_M:
                            raise ValueError(
                                'heading alignment moved the robot outside the target point')
                        result['final_heading_aligned'] = abs(align_deg) <= TARGET_FINAL_ALIGN_DEG
                        result['state'] = 'done'
                        break
                    turn = math.copysign(0.5 if abs(align_deg) < 20.0 else 0.8, align_error)
                    publish(0, turn)
                if now-brake_started > 10:
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

