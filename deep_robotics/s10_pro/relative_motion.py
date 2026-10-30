"""Pure feedback controller for bounded S10 relative motion.

The controller never publishes.  It consumes validated odometry samples and
returns one body-frame velocity command at a time, so the canvas adapter can
retain sole ownership of authorization, DDS publication and emergency stop.
"""
from __future__ import annotations

import math


POSITION_TOLERANCE_M = .08
ANGLE_TOLERANCE_RAD = math.radians(5.)
SETTLE_SECONDS = .4
STALL_SECONDS = 5.


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class RelativeMotionError(RuntimeError):
    def __init__(self, reason, message):
        super().__init__(message)
        self.reason = reason


class RelativeController:
    """One-axis relative translation or rotation in the starting body frame."""

    def __init__(self, initial, *, direction, target, max_speed, now):
        if direction not in ('forward', 'backward', 'left', 'right',
                             'turn_left', 'turn_right'):
            raise ValueError('unsupported relative direction')
        if not all(type(value) in (int, float) and math.isfinite(value)
                   for value in (target, max_speed, now)):
            raise ValueError('relative target values must be finite')
        self.turning = direction.startswith('turn_')
        if target <= 0:
            raise ValueError('relative target must be positive')
        self.direction, self.target, self.max_speed = direction, target, max_speed
        self.initial = {**initial, 'yaw_unwrapped': initial['yaw']}
        self.target_yaw = initial['yaw'] + target * (1 if direction == 'turn_left' else -1)
        self.settle_since = None
        self.best_error = float('inf')
        self.progress_time = now
        self.last = dict(self.initial)
        self.last_sequence = initial['sequence']
        self.last_progress = 0.

    def _validate(self, pose, now):
        if not 0 <= now - pose['received_monotonic'] <= .35:
            raise RelativeMotionError('odometry_stale', 'Odometry is stale or from the future')
        if pose['sequence'] == self.last_sequence:
            raise RelativeMotionError('odometry_not_advancing', 'Odometry sequence did not advance')
        dt = pose['received_monotonic'] - self.last['received_monotonic']
        if not 0 < dt <= .5:
            raise RelativeMotionError('odometry_timing_invalid', 'Odometry timing is stale or discontinuous')
        jump = math.hypot(pose['x'] - self.last['x'], pose['y'] - self.last['y'])
        if jump > .10 + .35 * dt:
            raise RelativeMotionError('odometry_position_jump', 'Odometry position changed beyond the safe bound')
        yaw_delta = wrap(pose['yaw'] - self.last['yaw'])
        yaw_jump = abs(yaw_delta)
        if yaw_jump > math.radians(8.) + .35 * dt:
            raise RelativeMotionError('odometry_heading_jump', 'Odometry heading changed beyond the safe bound')
        pose = {**pose, 'yaw_unwrapped': self.last['yaw_unwrapped'] + yaw_delta}
        self.last, self.last_sequence = dict(pose), pose['sequence']
        return pose

    def _progress(self, pose):
        if self.turning:
            requested_sign = 1 if self.direction == 'turn_left' else -1
            return requested_sign * (pose['yaw_unwrapped'] - self.initial['yaw_unwrapped'])
        dx, dy = pose['x'] - self.initial['x'], pose['y'] - self.initial['y']
        c, s = math.cos(self.initial['yaw']), math.sin(self.initial['yaw'])
        forward, left = c * dx + s * dy, -s * dx + c * dy
        return {'forward': forward, 'backward': -forward,
                'left': left, 'right': -left}[self.direction]

    def command(self, pose, now):
        pose = self._validate(pose, now)
        progress = self._progress(pose)
        remaining = self.target - progress
        if progress < self.last_progress - (.04 if self.turning else .03):
            raise RelativeMotionError('motion_reversed', 'Observed motion reversed away from the target')
        self.last_progress = max(self.last_progress, progress)
        tolerance = ANGLE_TOLERANCE_RAD if self.turning else POSITION_TOLERANCE_M
        if remaining < -tolerance:
            raise RelativeMotionError('target_overshot', 'Target was exceeded; stop before another request')
        dx, dy = pose['x'] - self.initial['x'], pose['y'] - self.initial['y']
        if self.turning:
            if math.hypot(dx, dy) > .12:
                raise RelativeMotionError('turn_position_drift', 'Translation exceeded the in-place turn limit')
        else:
            c, s = math.cos(self.initial['yaw']), math.sin(self.initial['yaw'])
            cross_track = (-s * dx + c * dy if self.direction in ('forward', 'backward')
                           else c * dx + s * dy)
            if abs(cross_track) > .12:
                raise RelativeMotionError('cross_track_exceeded', 'Motion deviated from the requested straight line')
            if abs(wrap(pose['yaw'] - self.initial['yaw'])) > math.radians(10):
                raise RelativeMotionError('heading_drift_exceeded', 'Heading changed during straight translation')
        error = abs(remaining)
        if error + tolerance * .25 < self.best_error:
            self.best_error, self.progress_time = error, now
        elif now - self.progress_time > STALL_SECONDS:
            raise RelativeMotionError('motion_stalled', 'No useful closed-loop progress was observed')
        stopped = (abs(pose['yaw_rate']) <= .06 if self.turning else
                   math.hypot(pose['vx'], pose['vy']) <= .05)
        if error <= tolerance:
            if not stopped:
                self.settle_since = None
                return (0., 0., 0.), self.status(progress, remaining, 'settling')
            if self.settle_since is None:
                self.settle_since = pose['received_monotonic']
            done = pose['received_monotonic'] - self.settle_since >= SETTLE_SECONDS
            return (0., 0., 0.), self.status(progress, remaining, 'completed' if done else 'settling')
        self.settle_since = None
        # Slow proportionally near the target while retaining enough command to
        # overcome the experimentally observed S10 deadband.
        minimum = .08 if self.turning else .06
        gain = .8 if self.turning else .7
        speed = min(self.max_speed, max(minimum, gain * error))
        sign = 1. if self.direction in ('forward', 'left', 'turn_left') else -1.
        if self.direction in ('forward', 'backward'):
            command = (sign * speed, 0., 0.)
        elif self.direction in ('left', 'right'):
            command = (0., sign * speed, 0.)
        else:
            command = (0., 0., sign * speed)
        return command, self.status(progress, remaining, 'moving')

    def status(self, progress, remaining, phase):
        unit = 'rad' if self.turning else 'm'
        return {'direction': self.direction, 'phase': phase, 'target': self.target,
                'progress': progress, 'remaining': remaining, 'unit': unit,
                'position_tolerance_m': POSITION_TOLERANCE_M,
                'angle_tolerance_deg': math.degrees(ANGLE_TOLERANCE_RAD)}
