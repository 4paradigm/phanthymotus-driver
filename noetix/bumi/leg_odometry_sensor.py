"""Independent Bumi leg-kinematic odometry publisher."""
from __future__ import annotations

import json
import math
import threading
import time

try:
    from .leg_odometry_source import LegOdometrySource
except ImportError:  # The bundle image copies modules into /work.
    from leg_odometry_source import LegOdometrySource


SAMPLE_INTERVAL_S = .025
PUBLISH_INTERVAL_S = .05
STOP_TIMEOUT_S = 1.


class LegOdometryPlugin:
    """Read joint positions and SDK attitude independently of camera consumers."""

    def __init__(self, config, namespace, executor, high_ctrl):
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from std_msgs.msg import String

        if high_ctrl is None:
            raise ValueError('leg odometry needs HighController')
        self.high_ctrl = high_ctrl
        self.node = Node('bumi_leg_odometry')
        self.String = String
        self.topic = f'/{namespace}/bumi_leg_odometry/state/odom'
        self.status_topic = f'/{namespace}/bumi_leg_odometry/status'
        self.pub = self.node.create_publisher(String, self.topic, qos_profile_sensor_data)
        self.status_pub = self.node.create_publisher(String, self.status_topic, 10)
        self.lock = threading.RLock()
        self.source = LegOdometrySource(namespace)
        self.active = False
        self.thread = None
        self.stop_event = None
        self.errors = 0
        self.last_error = None
        self._last_signature = None
        self._published_epoch = None
        self._published_read_ns = None
        self._published_reset = False
        self.timer = self.node.create_timer(PUBLISH_INTERVAL_S, self._publish)
        executor.add_node(self.node)

    def get_tools(self):
        return [{
            'name': 'leg_odometry', 'type': 'sensor', 'multiInstance': False,
            'description': '腿式里程计：关节运动学与机身姿态估计位姿和速度，输出标准里程计数据。',
            'inputSchema': {'type': 'object', 'properties': {
                'action': {'type': 'string', 'enum': ['start', 'stop', 'info']}}},
            'topic_out': [{'topic': self.topic, 'format': 'state/odom'},
                          {'topic': self.status_topic, 'format': 'data/json'}],
        }]

    def start(self):
        with self.lock:
            if self.active:
                return True
            # An SDK read cannot be cancelled. Its worker must retire before a
            # new reader starts, even though stop already disabled publication.
            if self.thread is not None and self.thread.is_alive():
                self.last_error = 'waiting for the previous SDK read to finish'
                return False
            self.source.reset()
            self.last_error = None
            self._last_signature = None
            self.stop_event = threading.Event()
            self.active = True
            self.thread = threading.Thread(target=self._sample_loop,
                args=(self.stop_event,), name='bumi_leg_odometry', daemon=True)
            self.thread.start()
            return True

    def stop(self):
        with self.lock:
            self.active = False
            if self.stop_event is not None:
                self.stop_event.set()
            thread = self.thread
            self.source.invalidate('stopped')
            self._publish(force=True)
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=STOP_TIMEOUT_S)
        return {'state': 'idle'}

    def _current_run(self, event):
        return self.active and (event is None or
            (event is self.stop_event and not event.is_set()))

    def _sample_once(self, event=None):
        with self.lock:
            if not self._current_run(event):
                return
        try:
            raw = self.high_ctrl.get_imu_data()
            quaternion = [float(raw.ori[i]) for i in range(4)]
            angular = [float(raw.angular_vel[i]) for i in range(3)]
            motors = self.high_ctrl.get_joint_state()
            joints = [float(getattr(motor, 'pos', motor)) for motor in motors]
            # Getters expose cached state, not a packet sequence. Include the
            # raw IMU noise and joint velocity/effort so a stationary robot's
            # new readings are distinct; do not round away those changes.
            extra = [*getattr(raw, 'linear_acc', ())]
            for motor in motors:
                extra.extend(getattr(motor, key, None) for key in ('vel', 'tau'))
            signature = tuple([*quaternion, *angular, *joints] + [
                float(value) if value is not None and math.isfinite(float(value)) else None
                for value in extra])
            monotonic_s = time.monotonic()
            stamp_ns = self.node.get_clock().now().nanoseconds
            with self.lock:
                if not self._current_run(event):
                    return
                if signature == self._last_signature:
                    return  # Cached reads must not advance the observation time.
                self.source.observe(joints, quaternion, angular,
                                    stamp_ns=stamp_ns, monotonic_s=monotonic_s)
                self._last_signature = signature
                self.last_error = None
        except Exception as error:
            with self.lock:
                if not self._current_run(event):
                    return
                self.errors += 1
                self.last_error = str(error)
                self.source.invalidate(self.last_error)
                if self.errors == 1 or self.errors % 100 == 0:
                    print(f'[leg_odometry] SDK sample failed ({self.errors}): {error}', flush=True)

    def _sample_loop(self, event):
        try:
            while not event.wait(SAMPLE_INTERVAL_S):
                self._sample_once(event)
        finally:
            with self.lock:
                if self.thread is threading.current_thread():
                    self.thread = None

    def _publish(self, *, force=False):
        with self.lock:
            if not self.active and not force:
                return
            sample = self.source.sample(self.node.get_clock().now().nanoseconds)
            epoch = self.source.acquisition_epoch
            read_ns = self.source.last_stamp_ns
            measured = any(value is not None for value in sample['twist'])
            if measured:
                repeat = epoch == self._published_epoch and read_ns == self._published_read_ns
                if (not repeat and self._published_read_ns is not None and
                        sample['stamp_ms'] <= self._published_read_ns // 1_000_000):
                    # Wire stamps are milliseconds. A rolled-back or colliding
                    # read cannot replace a measurement at the published stamp.
                    sample['pose'] = None
                    sample['twist'] = [None] * 6
                    sample['vendor'].update(status='lost', reason='waiting for a newer read timestamp')
                    measured = False
                elif epoch != self._published_epoch or (repeat and self._published_reset):
                    sample['reset'] = True
            self.pub.publish(self.String(data=json.dumps(sample, allow_nan=False)))
            if measured:
                self._published_epoch = epoch
                self._published_read_ns = read_ns
                self._published_reset = sample.get('reset') is True
            status = dict(sample['vendor'],
                          position_m=(sample.get('pose') or {}).get('position'),
                          last_error=self.last_error, error_count=self.errors)
            self.status_pub.publish(self.String(data=json.dumps(status, allow_nan=False)))

    def info(self):
        with self.lock:
            sample = self.source.sample(self.node.get_clock().now().nanoseconds)
            return dict(
                state='running' if self.active else 'idle',
                topic_out=self.get_tools()[0]['topic_out'],
                input_metadata=[{'schema': 'motus.slam_input/1',
                                 'topic': self.topic, 'format': 'state/odom'}],
                estimate=sample, last_error=self.last_error, error_count=self.errors,
                worker_stopping=bool(not self.active and self.thread is not None
                                     and self.thread.is_alive()))

    def dispatch(self, action, args):
        if action == 'start':
            if self.start():
                return {'state': 'running'}
            return {'state': 'error', 'message': self.last_error}
        if action == 'stop':
            return self.stop()
        if action in ('info', 'leg_odometry'):
            return self.info()
        return None
