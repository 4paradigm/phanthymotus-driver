"""Experimental body-IMU distance estimation, independent of camera and ROS.

SDK reads have no source timestamp. Poll timing and unchanged-data checks are
only fault heuristics, not proof of fresh measurements or physical accuracy.
"""
from collections import deque
import math
import time


def decode_imu(sample):
    """SDK xyzw attitude, body specific force in m/s², gyro in rad/s."""
    q = tuple(float(sample.ori[i]) for i in range(4))
    acc = tuple(float(sample.linear_acc[i]) for i in range(3))
    gyro = tuple(float(sample.angular_vel[i]) for i in range(3))
    if not all(math.isfinite(v) for v in (*q, *acc, *gyro)):
        raise ValueError('IMU contains non-finite data')
    norm = math.hypot(*q)
    if not 0.8 <= norm <= 1.2 or math.hypot(*acc) > 30:
        raise ValueError('invalid IMU quaternion or acceleration')
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
    # Calibration <=6 s, motion <=20 s, endpoint observation <=4 s.
    return 10.0+min(20.0, 5.0+10.0*distance_m)


def run_distance(owner, distance_m):
    estimator = ImuOdometry()
    started = time.monotonic()
    motion_started = brake_started = quiet_since = None
    last_signature = None
    changed_at = started
    progress_at, last_progress = started, 0.0
    moved = 0.0
    result = dict(state='error', requested_distance_m=distance_m,
                  feedback_source='imu_integrated_experimental', accuracy_validated=False,
                  source_timestamp_available=False, stationary_assumed=False)
    try:
        owner._distance_publish(0)
        while True:
            if owner._rotate_stop_event.is_set():
                result['state'] = 'stopped'
                break
            problem = owner._rotate_mode_error()
            if problem:
                raise ValueError(problem['error'])
            raw = owner._read_distance_imu()
            now = time.monotonic()
            if now-started >= distance_timeout(distance_m):
                raise ValueError('IMU distance movement timed out')
            estimator.observe(raw, now)
            if not estimator.ready:
                owner._distance_publish(0)
                time.sleep(0.02)
                continue
            if motion_started is None:
                motion_started = progress_at = changed_at = now
                origin_yaw = estimator.yaw
                c, s = math.cos(origin_yaw), math.sin(origin_yaw)
                result['stationary_world_acceleration_mps2'] = estimator.baseline
                print('[loco_destination] IMU experimental calibration complete; distance is an estimate', flush=True)
            if estimator.signature != last_signature:
                changed_at, last_signature = now, estimator.signature
            if brake_started is None and now-changed_at > 0.5:
                raise ValueError('IMU data unchanged during motion; source freshness cannot be verified')
            x, y, _ = estimator.position
            moved = x*c+y*s
            speed = estimator.velocity[0]*c+estimator.velocity[1]*s
            yaw_error = angle_delta(origin_yaw, estimator.yaw)
            if (abs(-x*s+y*c) > 0.15 or abs(yaw_error) > math.radians(20)
                    or math.hypot(*estimator.velocity) > 1.2):
                raise ValueError('IMU estimated motion exceeds direction or speed limits')
            remaining = distance_m-moved
            if remaining < -0.05:
                raise ValueError('IMU estimated overshoot exceeds 0.05 m')
            if brake_started is None:
                if now-motion_started > min(20.0, 5.0+10.0*distance_m):
                    raise ValueError('IMU motion time limit reached')
                if moved-last_progress >= 0.01:
                    last_progress, progress_at = moved, now
                elif now-progress_at > 3:
                    raise ValueError('IMU estimated distance made no progress')
                if remaining <= max(0.025, min(0.10, max(0, speed)*0.15)):
                    brake_started = now
                else:
                    turn = max(-0.4, min(0.4, 1.5*yaw_error-0.1*estimator.gyro[2]))
                    owner._distance_publish(0.6 if remaining > 0.2 else 0.5, turn)
            if brake_started is not None:
                owner._distance_publish(0)
                # A quiet accelerometer alone cannot distinguish rest from
                # uniform motion. Require zero command dwell AND low estimated
                # speed; even this is an assumption, not a measured stop.
                quiet = (now-brake_started >= 1.0 and math.hypot(*estimator.velocity) <= 0.10
                         and max(abs(v) for v in estimator.acceleration) <= 0.2
                         and math.hypot(*estimator.gyro) <= 0.08)
                if not quiet:
                    quiet_since = None
                elif quiet_since is None:
                    quiet_since = now
                elif now-quiet_since >= 0.8:
                    result['pre_zero_velocity_mps'] = estimator.velocity
                    estimator.velocity = (0.0, 0.0, 0.0)
                    result['stationary_assumed'] = True
                    if abs(remaining) > 0.05:
                        raise ValueError('stopped outside the IMU estimated target band')
                    result['state'] = 'done'
                    break
                if now-brake_started > 4:
                    raise ValueError('IMU endpoint did not meet assumed-stationary conditions')
            time.sleep(0.02)
    except Exception as exc:
        result['error'] = f'{type(exc).__name__}: {exc}'
    finally:
        owner._distance_publish(0)
    result.update(estimated_distance_m=moved, estimated_error_m=distance_m-moved,
                  estimated_position_m=estimator.position, estimated_velocity_mps=estimator.velocity,
                  elapsed_s=time.monotonic()-started)
    return result
