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
                self.imu = None
            return
        with self.lock:
            self.unwrapped += wrap(yaw - self.last_yaw) if self.last_yaw is not None else yaw
            self.last_yaw = yaw
            self.imu = dict(yaw=self.unwrapped, yaw_rate=rate, roll=roll, pitch=pitch,
                            angular_speed=math.sqrt(gx*gx + gy*gy + rate*rate), imu_stamp=self.clock())

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

    def __init__(self, initial, target, speed, yaw_speed, now):
        self.initial = initial
        c, s = math.cos(initial['odom_yaw']), math.sin(initial['odom_yaw'])
        self.x = initial['x'] + c * target['x'] - s * target['y']
        self.y = initial['y'] + s * target['x'] + c * target['y']
        self.yaw = initial['yaw'] + target['yaw']
        self.speed, self.yaw_speed = speed, yaw_speed
        self.settle_since = None
        self.best = float('inf')
        self.progress_time = now
        self.error = {}

    def step(self, pose, now):
        ex, ey, ea = self.x - pose['x'], self.y - pose['y'], self.yaw - pose['yaw']
        distance = math.hypot(ex, ey)
        self.error = dict(position_error_m=distance, angle_error_deg=math.degrees(ea))
        inside = distance <= self.POSITION_TOLERANCE and abs(ea) <= self.ANGLE_TOLERANCE
        if inside:
            if self.settle_since is None:
                self.settle_since = now
            stationary = pose['speed'] <= self.STOP_SPEED and pose['angular_speed'] <= self.STOP_YAW_RATE
            fresh_until = min(pose['odom_stamp'], pose['imu_stamp'])
            done = stationary and fresh_until - self.settle_since >= self.SETTLE_SECONDS
            if not stationary:
                self.settle_since = now
            if now - self.progress_time > self.STALL_SECONDS:
                raise RuntimeError('Could not settle at target')
            return (0., 0., 0.), done
        self.settle_since = None
        score = distance + .2 * abs(ea)
        if score < self.best - .005:
            self.best, self.progress_time = score, now
        if now - self.progress_time > self.STALL_SECONDS:
            raise RuntimeError('No progress toward target')
        heading = self.initial['odom_yaw'] + pose['yaw'] - self.initial['yaw']
        c, s = math.cos(heading), math.sin(heading)
        vx, vy = .8 * (c * ex + s * ey), .8 * (-s * ex + c * ey)
        norm = math.hypot(vx, vy)
        if norm > self.speed:
            vx, vy = vx * self.speed / norm, vy * self.speed / norm
        vyaw = max(-self.yaw_speed, min(self.yaw_speed, 1.2 * ea))
        return (vx, vy, vyaw), False


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
