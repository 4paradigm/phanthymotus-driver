"""Feedback cache and planar pose controller for direct relative locomotion."""
import json
import math
import threading
import time


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class SafetyViolation(RuntimeError):
    def __init__(self, reason, message, details=None):
        super().__init__(message)
        self.reason = reason
        self.details = details or {}


class MotionSafety:
    """Latched per-job checks; persistence uses new sensor samples, not loop ticks."""
    DEFAULTS = dict(feedback_timeout_s=.5,
                    tilt_threshold_deg=25., tilt_stop_deg=35., tilt_hold_s=.15,
                    speed_factor=1.5, speed_margin_m_s=.1, yaw_margin_rad_s=.2,
                    overspeed_hold_s=.2, position_jump_slack_m=.1, angle_jump_slack_deg=10.)

    def __init__(self, speed, yaw_speed, config=None):
        self.config = {**self.DEFAULTS, **(config or {})}
        self.config.pop('feedback_skew_s', None)  # Legacy dual-stream setting; now one stream.
        for key, value in self.config.items():
            if key not in self.DEFAULTS or isinstance(value, bool) or not math.isfinite(float(value)) or float(value) <= 0:
                raise ValueError(f'Invalid motion safety setting: {key}')
            self.config[key] = float(value)
        if self.config['tilt_stop_deg'] < self.config['tilt_threshold_deg']:
            raise ValueError('Severe tilt threshold must not be below sustained threshold')
        self.speed_limit = speed * self.config['speed_factor'] + self.config['speed_margin_m_s']
        self.angular_limit = yaw_speed * self.config['speed_factor'] + self.config['yaw_margin_rad_s']
        self.previous = None
        self.since = {}
        self.last_stamp = {}
        self.violation = None

    def _fail(self, reason, message, details=None):
        self.violation = SafetyViolation(reason, message, details)
        raise self.violation

    def _sustained(self, name, exceeded, stamp, duration):
        if self.last_stamp.get(name) == stamp:
            return
        self.last_stamp[name] = stamp
        if not exceeded:
            self.since.pop(name, None)
        else:
            start = self.since.setdefault(name, stamp)
            if stamp - start >= duration - 1e-9:
                self._fail(name, f'{name} persisted for {stamp - start:.3f}s')

    def check(self, pose, now):
        if self.violation:
            raise self.violation
        c = self.config
        keys = ('x', 'y', 'yaw', 'roll', 'pitch', 'speed', 'yaw_rate', 'angular_speed', 'odom_stamp')
        if any(key not in pose or not math.isfinite(pose[key]) for key in keys):
            self._fail('feedback_invalid', 'Missing or nonfinite safety feedback')
        if now - pose['odom_stamp'] < -1e-6:
            self._fail('feedback_invalid', 'Feedback timestamp is in the future')
        if now - pose['odom_stamp'] > c['feedback_timeout_s']:
            self._fail('feedback_stale', 'Odometry feedback is stale')
        tilt = math.degrees(max(abs(pose['roll']), abs(pose['pitch'])))
        if tilt >= c['tilt_stop_deg']:
            self._fail('tilt', f'Severe body tilt: {tilt:.1f} degrees')
        previous = self.previous
        if previous:
            if pose.get('odom_generation', 0) != previous.get('odom_generation', 0):
                self._fail('odom_reset', 'Odometry reference was reinitialized during the active task')
            dt = pose['odom_stamp'] - previous['odom_stamp']
            if dt < 0:
                self._fail('feedback_invalid', 'Feedback timestamps moved backwards')
            if math.hypot(pose['x'] - previous['x'], pose['y'] - previous['y']) > self.speed_limit * dt + c['position_jump_slack_m']:
                self._fail('position_jump', 'Odometry position changed beyond the sample-time limit')
            if abs(pose['yaw'] - previous['yaw']) > self.angular_limit * dt + math.radians(c['angle_jump_slack_deg']):
                self._fail('attitude_jump', 'Odometry heading changed beyond the sample-time limit', {
                    'odom_heading_jump': {
                        'previous_raw_yaw_deg': math.degrees(previous['raw_yaw']) if 'raw_yaw' in previous else None,
                        'current_raw_yaw_deg': math.degrees(pose['raw_yaw']) if 'raw_yaw' in pose else None,
                        'previous_unwrapped_yaw_deg': math.degrees(previous['yaw']),
                        'current_unwrapped_yaw_deg': math.degrees(pose['yaw']),
                        'previous_receive_monotonic_s': previous['odom_stamp'],
                        'current_receive_monotonic_s': pose['odom_stamp'],
                        'sample_interval_s': dt,
                        'actual_change_deg': math.degrees(abs(pose['yaw'] - previous['yaw'])),
                        'allowed_change_deg': math.degrees(self.angular_limit * dt) + c['angle_jump_slack_deg'],
                        'yaw_speed_rad_s': pose['yaw_rate'],
                        'angular_speed_rad_s': pose['angular_speed'],
                        'time_basis': 'local_callback_receive',
                    }})
            if max(abs(pose['roll'] - previous['roll']), abs(pose['pitch'] - previous['pitch'])) > self.angular_limit * dt + math.radians(c['angle_jump_slack_deg']):
                self._fail('attitude_jump', 'Odometry roll/pitch changed beyond the sample-time limit')
        self.previous = dict(pose)
        self._sustained('tilt', tilt >= c['tilt_threshold_deg'], pose['odom_stamp'], c['tilt_hold_s'])
        self._sustained('linear_overspeed', pose['speed'] > self.speed_limit,
                        pose['odom_stamp'], c['overspeed_hold_s'])
        self._sustained('angular_overspeed', pose['angular_speed'] > self.angular_limit,
                        pose['odom_stamp'], c['overspeed_hold_s'])


class PoseFeedback:
    """All control feedback comes from a single loco/state odometry sample."""
    def __init__(self, clock=time.monotonic, timeout=.5):
        self.clock, self.timeout = clock, float(timeout)
        self.lock = threading.Lock()
        self.odom = None
        self.last_yaw = None
        self.unwrapped = 0.
        self.odom_generation = 0

    def _reset(self):
        self.odom, self.last_yaw, self.unwrapped = None, None, 0.
        self.odom_generation += 1

    def update_odom(self, data):
        try:
            x, y = map(float, data['position'][:2])
            vx, vy = map(float, data['velocity'][:2])
            roll, pitch, yaw = map(float, data['imu']['rpy'][:3])
            rate = float(data['yaw_speed'])
            if not all(math.isfinite(v) for v in (x, y, vx, vy, roll, pitch, yaw, rate)):
                raise ValueError('Nonfinite odometry')
        except (ValueError, TypeError, KeyError, IndexError):
            with self.lock:
                self._reset()
            return
        with self.lock:
            now = self.clock()
            if self.odom and now - self.odom['odom_stamp'] > self.timeout:
                self._reset()
            self.unwrapped += wrap(yaw - self.last_yaw) if self.last_yaw is not None else yaw
            self.last_yaw = yaw
            self.odom = dict(x=x, y=y, vx=vx, vy=vy, speed=math.hypot(vx, vy),
                odom_yaw=yaw, yaw=self.unwrapped, raw_yaw=yaw, roll=roll, pitch=pitch,
                yaw_rate=rate, angular_speed=abs(rate), odom_stamp=now,
                odom_generation=self.odom_generation, feedback_source='loco/state',
                gyroscope=data['imu'].get('gyroscope'), quaternion=data['imu'].get('quaternion'),
                accelerometer=data['imu'].get('accelerometer'))

    def snapshot(self):
        with self.lock:
            if self.odom is None:
                raise SafetyViolation('feedback_missing', 'Odometry feedback is required')
            result = dict(self.odom)
        if self.clock() - result['odom_stamp'] > self.timeout:
            raise SafetyViolation('feedback_stale', 'Odometry feedback is stale')
        return result


class RelativeController:
    POSITION_TOLERANCE = .05
    ANGLE_TOLERANCE = math.radians(3)
    SETTLE_SECONDS = .5
    STOP_SPEED = .03
    STOP_YAW_RATE = .03
    STALL_SECONDS = 10.

    def __init__(self, initial, target, speed, yaw_speed, now, min_speed=.2, min_yaw_speed=.3):
        if not math.isfinite(min_speed) or not 0 < min_speed <= speed:
            raise ValueError('Minimum translation speed must be positive and no higher than the speed cap')
        if not math.isfinite(min_yaw_speed) or not 0 < min_yaw_speed <= yaw_speed:
            raise ValueError('Minimum rotation speed must be positive and no higher than the angular speed cap')
        self.initial = initial
        c, s = math.cos(initial['odom_yaw']), math.sin(initial['odom_yaw'])
        self.x = initial['x'] + c * target['x'] - s * target['y']
        self.y = initial['y'] + s * target['x'] + c * target['y']
        self.yaw = initial['yaw'] + target['yaw']
        self.requested_turn = target['yaw']
        self.speed, self.yaw_speed = speed, yaw_speed
        self.min_speed = min_speed
        self.min_yaw_speed = min_yaw_speed
        has_translation = target['x'] != 0 or target['y'] != 0
        self.final_turn_started = not has_translation
        self.phase = 'rotate' if not has_translation and target['yaw'] != 0 else 'translate'
        self.settle_since = None
        self.best = float('inf')
        self.progress_time = now
        self.error = {}

    def update_error(self, pose):
        """Observe errors without advancing phases or generating commands."""
        ex, ey = self.x - pose['x'], self.y - pose['y']
        heading_target = self.yaw if self.final_turn_started else self.initial['yaw']
        ea = wrap(heading_target - pose['yaw'])
        # At a half-turn both directions describe the same heading. Preserve the
        # requested direction around that tie, including small startup noise.
        if (abs(abs(self.requested_turn) - math.pi) < 1e-9
                and abs(abs(ea) - math.pi) <= self.ANGLE_TOLERANCE
                and ea * self.requested_turn < 0):
            ea = math.copysign(2 * math.pi - abs(ea), self.requested_turn)
        distance = math.hypot(ex, ey)
        self.error = dict(position_error_m=distance,
                          angle_error_deg=math.degrees(wrap(self.yaw - pose['yaw'])),
                          heading_error_deg=math.degrees(ea), phase=self.phase)
        return ex, ey, ea, distance

    def step(self, pose, now):
        ex, ey, ea, distance = self.update_error(pose)
        if self.phase == 'rotate':
            self._progress(.2 * abs(ea), now)
            if abs(ea) <= self.ANGLE_TOLERANCE:
                if self._settled(pose, now):
                    self._transition('translate', now)
                return (0., 0., 0.), False
            self.settle_since = None
            rotation = math.copysign(min(self.yaw_speed, max(self.min_yaw_speed, 1.2 * abs(ea))), ea)
            return (0., 0., rotation), False
        self._progress(distance, now)
        if distance <= self.POSITION_TOLERANCE:
            settled = self._settled(pose, now)
            # Correct heading only after reaching and settling at the position.
            if settled and (not self.final_turn_started or abs(ea) > self.ANGLE_TOLERANCE):
                self.final_turn_started = True
                self._transition('rotate', now)
                return (0., 0., 0.), False
            return (0., 0., 0.), settled
        self.settle_since = None
        heading = self.initial['odom_yaw'] + pose['yaw'] - self.initial['yaw']
        c, s = math.cos(heading), math.sin(heading)
        errors = [c * ex + s * ey, -s * ex + c * ey]
        velocity = [0., 0.]
        if distance > self.POSITION_TOLERANCE:
            # Per-axis deadband keeps the combined position error within the
            # circular tolerance, without commanding ineffective small components.
            active = [i for i in range(2) if abs(errors[i]) > self.POSITION_TOLERANCE / math.sqrt(2)]
            if len(active) == 2 and self.speed < math.sqrt(2) * self.min_speed:
                active = [max(active, key=lambda i: abs(errors[i]))]
            magnitudes = [max(self.min_speed, .8 * abs(errors[i])) for i in active]
            base = len(active) * self.min_speed ** 2
            extra = sum(value ** 2 - self.min_speed ** 2 for value in magnitudes)
            scale = min(1., max(0., self.speed ** 2 - base) / extra) if extra > 0 else 1.
            for i, value in zip(active, magnitudes):
                bounded = math.sqrt(self.min_speed ** 2 + scale * (value ** 2 - self.min_speed ** 2))
                velocity[i] = math.copysign(bounded, errors[i])
        vx, vy = velocity
        return (vx, vy, 0.), False

    def _transition(self, phase, now):
        self.phase = phase
        self.error['phase'] = phase
        self.settle_since = None
        self.best, self.progress_time = float('inf'), now

    def _progress(self, score, now):
        if score < self.best - .005:
            self.best, self.progress_time = score, now
        if now - self.progress_time > self.STALL_SECONDS:
            raise RuntimeError(f'No progress in {self.phase} phase')

    def _settled(self, pose, now):
        if pose['speed'] > self.STOP_SPEED or pose['angular_speed'] > self.STOP_YAW_RATE:
            self.settle_since = None
            return False
        if self.settle_since is None:
            self.settle_since = now
        return pose['odom_stamp'] - self.settle_since >= self.SETTLE_SECONDS


def create_feedback_node(namespace, feedback):
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    from std_msgs.msg import String
    node = Node('g1_relative_move_feedback')
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST)
    def receive(update, message):
        try:
            data = json.loads(message.data)
        except (ValueError, TypeError):
            data = {}
        update(data)
    node.create_subscription(String, f'/{namespace}/loco/state', lambda msg: receive(feedback.update_odom, msg), qos)
    return node
