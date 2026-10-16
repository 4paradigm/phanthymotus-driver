"""Feedback cache and planar pose controller for direct relative locomotion."""
import json
import math
import threading
import time


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class SafetyViolation(RuntimeError):
    def __init__(self, reason, message):
        super().__init__(message)
        self.reason = reason


class MotionSafety:
    """Latched per-job checks; persistence uses new sensor samples, not loop ticks."""
    DEFAULTS = dict(feedback_timeout_s=.5, feedback_skew_s=.25,
                    tilt_threshold_deg=25., tilt_stop_deg=35., tilt_hold_s=.15,
                    speed_factor=1.5, speed_margin_m_s=.1, yaw_margin_rad_s=.2,
                    overspeed_hold_s=.2, position_jump_slack_m=.1, angle_jump_slack_deg=10.)

    def __init__(self, speed, yaw_speed, config=None):
        self.config = {**self.DEFAULTS, **(config or {})}
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

    def _fail(self, reason, message):
        self.violation = SafetyViolation(reason, message)
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
        keys = ('x', 'y', 'yaw', 'roll', 'pitch', 'speed', 'yaw_rate', 'angular_speed', 'odom_stamp', 'imu_stamp')
        if any(key not in pose or not math.isfinite(pose[key]) for key in keys):
            self._fail('feedback_invalid', 'Missing or nonfinite safety feedback')
        if min(now - pose['odom_stamp'], now - pose['imu_stamp']) < -1e-6:
            self._fail('feedback_invalid', 'Feedback timestamp is in the future')
        if now - min(pose['odom_stamp'], pose['imu_stamp']) > c['feedback_timeout_s']:
            self._fail('feedback_stale', 'IMU or odometry feedback is stale')
        if abs(pose['odom_stamp'] - pose['imu_stamp']) > c['feedback_skew_s']:
            self._fail('feedback_skew', 'IMU and odometry feedback are not synchronized')
        tilt = math.degrees(max(abs(pose['roll']), abs(pose['pitch'])))
        if tilt >= c['tilt_stop_deg']:
            self._fail('tilt', f'Severe body tilt: {tilt:.1f} degrees')
        previous = self.previous
        if previous:
            if pose.get('imu_generation', 0) != previous.get('imu_generation', 0):
                self._fail('imu_reset', 'IMU reference was reinitialized during the active task')
            dt = pose['odom_stamp'] - previous['odom_stamp']
            da = pose['imu_stamp'] - previous['imu_stamp']
            if dt < 0 or da < 0:
                self._fail('feedback_invalid', 'Feedback timestamps moved backwards')
            if math.hypot(pose['x'] - previous['x'], pose['y'] - previous['y']) > self.speed_limit * dt + c['position_jump_slack_m']:
                self._fail('position_jump', 'Odometry position changed beyond the sample-time limit')
            if abs(pose['yaw'] - previous['yaw']) > self.angular_limit * da + math.radians(c['angle_jump_slack_deg']):
                self._fail('attitude_jump', 'IMU heading changed beyond the sample-time limit')
            if max(abs(pose['roll'] - previous['roll']), abs(pose['pitch'] - previous['pitch'])) > self.angular_limit * da + math.radians(c['angle_jump_slack_deg']):
                self._fail('attitude_jump', 'IMU roll/pitch changed beyond the sample-time limit')
        self.previous = dict(pose)
        self._sustained('tilt', tilt >= c['tilt_threshold_deg'], pose['imu_stamp'], c['tilt_hold_s'])
        self._sustained('linear_overspeed', pose['speed'] > self.speed_limit,
                        pose['odom_stamp'], c['overspeed_hold_s'])
        self._sustained('angular_overspeed', pose['angular_speed'] > self.angular_limit,
                        pose['imu_stamp'], c['overspeed_hold_s'])


class PoseFeedback:
    def __init__(self, clock=time.monotonic, timeout=.5, skew=.25):
        self.clock = clock
        self.timeout, self.skew = float(timeout), float(skew)
        self.lock = threading.Lock()
        self.odom = self.imu = None
        self.last_yaw = None
        self.unwrapped = 0.
        self.imu_generation = 0

    def _reset_imu(self):
        # Caller holds lock. A new generation invalidates every active target
        # even if invalid and recovered samples arrive between control ticks.
        self.imu = None
        self.last_yaw = None
        self.unwrapped = 0.
        self.imu_generation += 1

    def update_odom(self, data):
        try:
            x, y = map(float, data['position'][:2])
            vx, vy = map(float, data['velocity'][:2])
            yaw = float(data['imu']['rpy'][2])
            if not all(math.isfinite(v) for v in (x, y, vx, vy, yaw)):
                raise ValueError('Nonfinite odometry')
            value = dict(x=x, y=y, speed=math.hypot(vx, vy), odom_yaw=yaw, odom_stamp=self.clock())
        except (ValueError, TypeError, KeyError, IndexError):
            value = None
        with self.lock:
            self.odom = value

    def update_imu(self, data):
        try:
            roll, pitch, yaw = map(float, data['rpy'][:3])
            gx, gy, rate = map(float, data['gyroscope'][:3])
            if not all(math.isfinite(v) for v in (roll, pitch, yaw, gx, gy, rate)):
                raise ValueError('Nonfinite IMU')
        except (ValueError, TypeError, KeyError, IndexError):
            with self.lock:
                self._reset_imu()
            return
        with self.lock:
            now = self.clock()
            if self.imu and now - self.imu['imu_stamp'] > self.timeout:
                self._reset_imu()
            self.unwrapped += wrap(yaw - self.last_yaw) if self.last_yaw is not None else yaw
            self.last_yaw = yaw
            self.imu = dict(yaw=self.unwrapped, yaw_rate=rate, roll=roll, pitch=pitch,
                            angular_speed=math.sqrt(gx*gx + gy*gy + rate*rate), imu_stamp=now,
                            imu_generation=self.imu_generation)

    def snapshot(self):
        with self.lock:
            if self.odom is None or self.imu is None:
                raise SafetyViolation('feedback_missing', 'IMU and odometry feedback are required')
            result = {**self.odom, **self.imu}
        now = self.clock()
        if now - min(result['odom_stamp'], result['imu_stamp']) > self.timeout:
            raise SafetyViolation('feedback_stale', 'IMU or odometry feedback is stale')
        if abs(result['odom_stamp'] - result['imu_stamp']) > self.skew:
            raise SafetyViolation('feedback_skew', 'IMU and odometry feedback are not synchronized')
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
        self.final_turn_started = not has_translation or target['yaw'] == 0
        self.phase = 'rotate' if not has_translation and target['yaw'] != 0 else 'translate'
        self.settle_since = None
        self.best = float('inf')
        self.progress_time = now
        self.error = {}

    def step(self, pose, now):
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
        if self.phase == 'brake':
            if self._settled(pose, now):
                self._transition('rotate', now)
            elif now - self.progress_time > self.STALL_SECONDS:
                raise RuntimeError('Could not stop before heading correction')
            return (0., 0., 0.), False
        if self.phase == 'rotate':
            self._progress(.2 * abs(ea), now)
            if abs(ea) <= self.ANGLE_TOLERANCE:
                if self._settled(pose, now):
                    self._transition('translate', now)
                return (0., 0., 0.), False
            self.settle_since = None
            rotation = math.copysign(min(self.yaw_speed, max(self.min_yaw_speed, 1.2 * abs(ea))), ea)
            return (0., 0., rotation), False
        if abs(ea) > self.ANGLE_TOLERANCE:
            self._transition('brake', now)
            return (0., 0., 0.), False
        self._progress(distance, now)
        if distance <= self.POSITION_TOLERANCE:
            settled = self._settled(pose, now)
            if settled and not self.final_turn_started:
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
        return min(pose['odom_stamp'], pose['imu_stamp']) - self.settle_since >= self.SETTLE_SECONDS


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
    node.create_subscription(String, f'/{namespace}/state/imu', lambda msg: receive(feedback.update_imu, msg), qos)
    return node
