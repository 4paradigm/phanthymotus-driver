"""Read the independent RGB-D/IMU pipeline and drive a straight distance.

No camera ownership, metric SDK velocity assumptions or loco_servo execution.
ROS imports are delayed so rotation remains usable without fusion packages.
"""
from __future__ import annotations

from collections import deque
import json
import math
import threading
import time


def distance_timeout(distance_m):
    return 15.0 + 30.0 * distance_m


def angle_delta(a, b):
    return math.atan2(math.sin(a-b), math.cos(a-b))


class DistanceFeedback:
    def __init__(self, executor, *, now_ns=None, monotonic=None):
        self.executor = executor
        self.now_ns = now_ns or time.time_ns
        self.monotonic = monotonic or time.monotonic
        self.lock = threading.Lock()
        self.lifecycle_lock = threading.Lock()
        self.node = None
        self.pose = None
        self.quality = None
        self.reason = None

    def start(self):
        with self.lifecycle_lock:
            if self.node is not None:
                return
            if self.executor is None:
                raise ValueError('distance feedback requires a ROS executor')
            from rclpy.node import Node
            from rclpy.qos import qos_profile_sensor_data
            from nav_msgs.msg import Odometry
            from std_msgs.msg import String
            node = Node('bumi_destination_feedback')
            try:
                node.create_subscription(Odometry, '/bumi_odom/filtered', self.update_pose,
                                         qos_profile_sensor_data)
                node.create_subscription(String, '/bumi_odom/status', self.update_status,
                                         qos_profile_sensor_data)
                self.now_ns = lambda: node.get_clock().now().nanoseconds
                self.executor.add_node(node)
                self.node = node
            except Exception:
                node.destroy_node()
                raise

    def stop(self):
        with self.lifecycle_lock:
            if self.node is not None:
                self.executor.remove_node(self.node)
                self.node.destroy_node()
                self.node = None
            with self.lock:
                self.pose = self.quality = None
                self.reason = None

    def _stamp_valid(self, stamp):
        return (isinstance(stamp, int) and not isinstance(stamp, bool) and stamp > 0
                and -50_000_000 <= self.now_ns() - stamp <= 500_000_000)

    def update_pose(self, msg):
        with self.lock:
            try:
                stamp = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
                p, q = msg.pose.pose.position, msg.pose.pose.orientation
                position = (float(p.x), float(p.y), float(p.z))
                quat = (float(q.x), float(q.y), float(q.z), float(q.w))
                norm = math.hypot(*quat)
                if (msg.header.frame_id != 'bumi_odom_odom'
                        or msg.child_frame_id != 'bumi_odom_base_link'
                        or not self._stamp_valid(stamp) or not math.isfinite(norm) or norm < 1e-6
                        or not all(math.isfinite(v) for v in position)
                        or not all(math.isfinite(msg.pose.covariance[i]) and msg.pose.covariance[i] > 0
                                   for i in (0, 7, 14, 21, 28, 35))):
                    raise ValueError('invalid odometry frame, pose, covariance or source time')
                x, y, z, w = (v/norm for v in quat)
                yaw = math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))
                if self.pose is not None:
                    dt = (stamp-self.pose['stamp_ns']) / 1e9
                    shift = math.dist(position, self.pose['position'])
                    if dt <= 0 or shift > 0.05 + 1.5*dt:
                        raise ValueError('odometry timestamp reset or position jump')
                self.pose = dict(position=position, yaw=yaw, stamp_ns=stamp,
                                 received=self.monotonic())
            except (ValueError, TypeError, AttributeError, IndexError) as exc:
                self.reason = self.reason or str(exc)

    def update_status(self, msg):
        with self.lock:
            try:
                data = json.loads(msg.data) if hasattr(msg, 'data') else msg
                segment, stamp = data['segment_id'], data['stamp_ns']
                if not isinstance(segment, str) or not segment or not self._stamp_valid(stamp):
                    raise ValueError('invalid quality segment or source time')
                if self.quality is not None:
                    if segment != self.quality['segment_id']:
                        self.pose = None
                        self.reason = None
                    elif stamp <= self.quality['stamp_ns']:
                        raise ValueError('quality timestamp reset')
                self.quality = dict(segment_id=segment, stamp_ns=stamp,
                                    status=data['status'], received=self.monotonic())
                if data['status'] != 'tracking':
                    self.reason = self.reason or 'odometry quality is not tracking'
            except (ValueError, TypeError, KeyError, AttributeError) as exc:
                self.reason = self.reason or str(exc)

    def snapshot(self):
        with self.lock:
            if self.reason:
                raise ValueError(self.reason)
            if self.pose is None or self.quality is None:
                raise ValueError('distance feedback unavailable: start RGB-D/IMU fusion in the driver ROS domain')
            for sample in (self.pose, self.quality):
                age = self.monotonic() - sample['received']
                if age < 0 or age > 0.5 or not self._stamp_valid(sample['stamp_ns']):
                    raise ValueError('distance feedback stale or clock reset')
            return dict(self.pose, segment_id=self.quality['segment_id'])

    def revalidate(self):
        """Clear a latched fault only when the latest samples are healthy.

        Called from move preflight, never mid-motion: a fault during an
        active move must keep latching (the loop brakes on the raise), while
        a new request revalidates so one transient bad frame does not brick
        the card until the fusion segment restarts.
        """
        with self.lock:
            if (self.reason is not None and self.pose is not None
                    and self.quality is not None
                    and self.quality['status'] == 'tracking'
                    and self.monotonic() - self.pose['received'] <= 0.5
                    and self.monotonic() - self.quality['received'] <= 0.5):
                self.reason = None


def run_distance(owner, distance_m):
    """50 Hz direct SDK loop. Only fresh source samples count toward settling."""
    start_time = time.monotonic()
    moved = 0.0
    result = dict(state='error', requested_distance_m=distance_m, tolerance_m=0.05)
    try:
        origin = owner._read_distance_feedback()
        previous = origin
        c, s = math.cos(origin['yaw']), math.sin(origin['yaw'])
        last_new = start_time
        last_progress, progress_at = 0.0, start_time
        brake_started = None
        window = deque()
        while True:
            now = time.monotonic()
            if owner._rotate_stop_event.is_set():
                result['state'] = 'stopped'
                break
            problem = owner._rotate_mode_error()
            if problem:
                raise ValueError(problem['error'])
            sample = owner._read_distance_feedback()
            if sample['segment_id'] != origin['segment_id']:
                raise ValueError('odometry segment changed during movement')
            dt = (sample['stamp_ns'] - previous['stamp_ns']) / 1e9
            if dt < 0:
                raise ValueError('odometry timestamp reset')
            speed = 0.0
            if dt > 0:
                if math.dist(sample['position'], previous['position']) > 0.05 + 1.5*dt:
                    raise ValueError('odometry position jump')
                dx, dy = (sample['position'][i]-origin['position'][i] for i in (0, 1))
                projected = dx*c + dy*s
                speed = (projected-moved) / dt
                moved = projected
                if abs(-dx*s+dy*c) > 0.15 or abs(angle_delta(sample['yaw'], origin['yaw'])) > math.radians(20):
                    raise ValueError('movement deviated from starting direction')
                last_new = now
                previous = sample
                if brake_started is not None:
                    window.append((sample['stamp_ns'], moved))
                    while len(window) > 1 and sample['stamp_ns']-window[1][0] >= 500_000_000:
                        window.popleft()
            elif sample['position'] != previous['position']:
                raise ValueError('odometry changed without a new source timestamp')
            if now-last_new > 0.5:
                raise ValueError('filtered odometry stopped updating')
            if now-start_time >= distance_timeout(distance_m):
                raise ValueError('distance movement timed out')
            remaining = distance_m-moved
            if remaining < -0.05:
                raise ValueError('distance overshoot exceeds tolerance')
            if brake_started is None:
                if moved-last_progress >= 0.01:
                    last_progress, progress_at = moved, now
                elif now-progress_at > 3:
                    raise ValueError('distance movement made no progress')
                if remaining <= max(0.025, min(0.10, max(0, speed)*0.15)):
                    brake_started = now
                else:
                    # Normalised SDK values, deliberately above the gait deadband.
                    owner._distance_publish(0.65 if remaining > 0.2 else 0.5)
            if brake_started is not None:
                owner._distance_publish(0)
                if len(window) >= 5 and window[-1][0]-window[0][0] >= 500_000_000:
                    values = [row[1] for row in window]
                    elapsed = (window[-1][0]-window[0][0])/1e9
                    if max(values)-min(values) <= 0.015 and abs(values[-1]-values[0])/elapsed <= 0.01:
                        if abs(remaining) > 0.05:
                            raise ValueError('stopped outside distance tolerance')
                        result['state'] = 'done'
                        break
                if now-brake_started > 3:
                    raise ValueError('distance endpoint did not settle')
            time.sleep(0.02)
    except (ValueError, RuntimeError) as exc:
        result['error'] = str(exc)
    finally:
        owner._distance_publish(0)
    result.update(moved_m=moved, final_error_m=distance_m-moved,
                  elapsed_s=time.monotonic()-start_time)
    return result
