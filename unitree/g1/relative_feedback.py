"""Feedback cache and planar pose controller for direct relative locomotion."""
import json
import math
import threading
import time


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class PoseFeedback:
    def __init__(self, clock=time.monotonic):
        self.clock = clock
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
            yaw, rate = float(data['rpy'][2]), float(data['gyroscope'][2])
            if not all(math.isfinite(v) for v in (yaw, rate)):
                raise ValueError('Nonfinite IMU')
        except (ValueError, TypeError, KeyError, IndexError):
            with self.lock:
                self.imu = None
            return
        with self.lock:
            self.unwrapped += wrap(yaw - self.last_yaw) if self.last_yaw is not None else yaw
            self.last_yaw = yaw
            self.imu = dict(yaw=self.unwrapped, yaw_rate=rate, imu_stamp=self.clock())

    def snapshot(self):
        with self.lock:
            if self.odom is None or self.imu is None:
                raise RuntimeError('IMU and odometry feedback are required')
            result = {**self.odom, **self.imu}
        now = self.clock()
        if now - min(result['odom_stamp'], result['imu_stamp']) > .5:
            raise RuntimeError('IMU or odometry feedback is stale')
        if abs(result['odom_stamp'] - result['imu_stamp']) > .25:
            raise RuntimeError('IMU and odometry feedback are not synchronized')
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
        self.last_pose = initial
        self.error = {}

    def step(self, pose, now):
        # Reject discontinuities rather than interpreting an odometry reset as motion.
        if math.hypot(pose['x'] - self.last_pose['x'], pose['y'] - self.last_pose['y']) > .5:
            raise RuntimeError('Odometry discontinuity')
        if abs(pose['yaw'] - self.last_pose['yaw']) > .5:
            raise RuntimeError('IMU discontinuity')
        self.last_pose = pose
        ex, ey, ea = self.x - pose['x'], self.y - pose['y'], self.yaw - pose['yaw']
        distance = math.hypot(ex, ey)
        self.error = dict(position_error_m=distance, angle_error_deg=math.degrees(ea))
        inside = distance <= self.POSITION_TOLERANCE and abs(ea) <= self.ANGLE_TOLERANCE
        if inside:
            if self.settle_since is None:
                self.settle_since = now
            stationary = pose['speed'] <= self.STOP_SPEED and abs(pose['yaw_rate']) <= self.STOP_YAW_RATE
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
