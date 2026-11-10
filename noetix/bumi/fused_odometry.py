"""Opt-in, read-only Bumi leg/IMU/RTAB-Map planar odometry sensor.

It neither replaces rgbd_visual_odometry nor writes movement commands. The
visual input is a *measurement*; failures never cause the fused frame to reset.
No claim of full 6-DoF VIO or verified accuracy is made.
"""
from __future__ import annotations
import json
import math
import threading
import time

try:
    from .leg_odometry import LegOdometry
    from .odometry_fusion import PlanarFusion, rotation, yaw_from_quat
except ImportError:  # image copies these modules into /work
    from leg_odometry import LegOdometry
    from odometry_fusion import PlanarFusion, rotation, yaw_from_quat


class FusedOdometryPlugin:
    """Isolated sensor card: 40 Hz robot reads and 20 Hz fused output."""

    def __init__(self, config, namespace, executor, high_ctrl):
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from std_msgs.msg import String
        if high_ctrl is None:
            raise ValueError('fused odometry needs HighController')
        self.high_ctrl = high_ctrl
        self.namespace = namespace
        self.node = Node('bumi_fused_odometry')
        self.String = String
        self.topic = f'/{namespace}/bumi_fused_odometry/state/odom'
        self.status_topic = f'/{namespace}/bumi_fused_odometry/status'
        self.visual_topic = f'/{namespace}/bumi_visual_odometry/state/odom'
        self.pub = self.node.create_publisher(String, self.topic, 10)
        self.status_pub = self.node.create_publisher(String, self.status_topic, 10)
        self.sub = self.node.create_subscription(String, self.visual_topic,
                                                 self._on_visual, qos_profile_sensor_data)
        self.timer = self.node.create_timer(.05, self._publish)
        executor.add_node(self.node)
        self.lock = threading.RLock()
        self.stop_event = threading.Event()
        self.thread = None
        self.active = False
        self.filter = PlanarFusion()
        self.leg = LegOdometry(slip_scale=1.0)
        self.last_leg_position = None
        self.last_monotonic = None
        self.velocity = None
        self.gyro_z = None
        self.height = 0.0
        self.errors = 0
        self.last_status_at = 0.0
        self.last_status = None
        self.last_visual_result = None
        self.last_error = None

    def get_tools(self):
        return [{
            'name': 'fused_odometry', 'type': 'sensor', 'multiInstance': False,
            'description': ('Bumi experimental planar leg+IMU prediction and '
                            'quality-gated RGB-D visual correction; read-only, '
                            'unvalidated accuracy, no motion control.'),
            'inputSchema': {'type': 'object', 'properties': {
                'action': {'type': 'string', 'enum': ['start', 'stop', 'info']}}},
            'topic_out': [{'topic': self.status_topic, 'format': 'data/json'},
                          {'topic': self.topic, 'format': 'state/odom'}],
        }]

    def start(self):
        with self.lock:
            if self.active:
                return True
            self.filter = PlanarFusion()
            self.leg = LegOdometry(slip_scale=1.0)
            self.last_leg_position = None
            self.last_monotonic = None
            self.velocity = None
            self.gyro_z = None
            self.last_visual_result = None
            self.last_error = None
            self.stop_event.clear()
            self.active = True
            self.thread = threading.Thread(target=self._sample_loop, daemon=True,
                                           name='bumi_fused_leg_imu')
            self.thread.start()
            return True

    def stop(self):
        with self.lock:
            self.active = False
            self.stop_event.set()
            thread = self.thread
            self.thread = None
        if thread is not None and thread.is_alive():
            thread.join(timeout=1.0)
        return {'state': 'stopped'}

    def _sample_loop(self):
        while not self.stop_event.wait(.025):
            try:
                # Read close together; the SDK exposes no acquisition timestamps.
                raw = self.high_ctrl.get_imu_data()
                joints = self.high_ctrl.get_joint_state()
                mono = time.monotonic()
                stamp_ns = self.node.get_clock().now().nanoseconds
                quaternion = [float(raw.ori[i]) for i in range(4)]
                imu_yaw = yaw_from_quat(quaternion)
                prev = self.leg.position[:]
                prev_stance = self.leg.stance_samples
                try:
                    self.leg.observe(joints, quaternion, mono)
                except ValueError:
                    # A long sample gap destroys the stance-foot baseline.
                    self.leg.reset_baseline()
                    self.leg.observe(joints, quaternion, mono)
                    prev = self.leg.position[:]
                    prev_stance = self.leg.stance_samples
                valid = self.leg.stance_samples > prev_stance
                delta_world = [a-b for a, b in zip(self.leg.position, prev)]
                delta_body = rotation(-imu_yaw) @ delta_world[:2]
                dt = mono-self.last_monotonic if self.last_monotonic else 0.0
                with self.lock:
                    if not self.active:
                        break
                    self.filter.predict(stamp_ns, imu_yaw, delta_body, valid)
                    if valid and 0.0 < dt <= .12:
                        self.velocity = self.filter._scale(delta_body).tolist()
                        self.velocity = [v/dt for v in self.velocity]
                    else:
                        self.velocity = None
                    self.height = float(self.leg.position[2])
                    self.gyro_z = float(raw.angular_vel[2])
                    self.last_monotonic = mono
            except Exception as exc:
                with self.lock:
                    self.errors += 1
                    self.last_error = str(exc)
                    self.velocity = None
                if self.errors == 1 or self.errors % 100 == 0:
                    print(f'[fused_odometry] sample failure ({self.errors}): {exc}', flush=True)

    def _on_visual(self, msg):
        try:
            raw = json.loads(msg.data)
            if raw.get('schema') != 'motus.odom/1':
                return
            vendor = raw.get('vendor') or {}
            pose = raw.get('pose')
            # Do not use a frozen pose or uncalibrated optical frame as body.
            if vendor.get('status') != 'tracking' or not pose or vendor.get('pose_reference') != 'body':
                return
            diag = vendor.get('visual_diagnostic') or {}
            stamp_ns = vendor.get('visual_stamp_ns')
            if not isinstance(stamp_ns, int) or isinstance(stamp_ns, bool):
                return
            now_ns = self.node.get_clock().now().nanoseconds
            if not 0 <= now_ns-stamp_ns <= 2_500_000_000:
                return
            segment = vendor.get('segment_id')
            with self.lock:
                if not self.active:
                    return
                self.last_visual_result = self.filter.visual(
                    stamp_ns=stamp_ns, segment=segment,
                    position=pose['position'], quaternion=pose['quaternion'],
                    covariance=pose['covariance'], inliers=int(diag.get('inliers', 0)))
        except (ValueError, TypeError, KeyError, OverflowError) as exc:
            with self.lock:
                self.last_visual_result = 'invalid_visual_sample'
                self.last_error = str(exc)

    def _build(self):
        from common.odom import build_sample
        now_ns = self.node.get_clock().now().nanoseconds
        current = self.filter.result(now_ns) if self.active else None
        if current is None:
            status = 'stopped' if not self.active else 'lost'
            vendor = {'status': status, 'reason': 'SDK sample unavailable',
                      'source': 'bumi_leg_imu_rgbd_planar_ekf',
                      'accuracy_validated': False, 'pose_reference': 'body',
                      'visual_result': self.last_visual_result,
                      'last_error': self.last_error, 'error_count': self.errors}
            return build_sample(stamp_ms=now_ns//1_000_000,
                                twist=[None]*6, pose=None, vendor=vendor)
        yaw = current['yaw']
        covariance = [0.0]*36
        covariance[0], covariance[7] = current['covariance'][0][0], current['covariance'][1][1]
        covariance[14] = 1.0  # Vertical translation is not fused here.
        covariance[21] = covariance[28] = 0.5**2  # Roll/pitch not fused.
        covariance[35] = current['covariance'][2][2]
        # x/y/yaw cross-covariances are retained in the ROS 6x6 convention.
        for row, old_row in enumerate((0, 1, 5)):
            for col, old_col in enumerate((0, 1, 5)):
                covariance[old_row*6+old_col] = current['covariance'][row][col]
        pose = dict(position=[*current['position'], self.height],
                    quaternion=[0.0, 0.0, math.sin(yaw/2), math.cos(yaw/2)],
                    frame_id='bumi_fused_odom', child_frame_id='base_link',
                    covariance=covariance)
        twist = [None]*6
        if self.velocity is not None and current['good_leg']:
            twist[0], twist[1] = self.velocity[:2]
        if self.gyro_z is not None and math.isfinite(self.gyro_z):
            twist[5] = self.gyro_z
        vendor = dict(current, source='bumi_leg_imu_rgbd_planar_ekf',
                      accuracy_validated=False, pose_reference='body',
                      planar_only=True, drift='unbounded',
                      timestamp_source='sdk_read_completion',
                      visual_result=self.last_visual_result,
                      last_error=self.last_error, error_count=self.errors)
        # No motus_slam/1 capability is advertised: planar dead reckoning is
        # not a validated replacement for a full 6-DoF mapping odometry source.
        return build_sample(stamp_ms=now_ns//1_000_000,
                            twist=twist, pose=pose, vendor=vendor)

    def _publish(self):
        with self.lock:
            sample = self._build()
            self.pub.publish(self.String(data=json.dumps(sample, allow_nan=False)))
            status = sample['vendor']['status']
            now = time.monotonic()
            if status != self.last_status or now-self.last_status_at >= .2:
                detail = sample['vendor']
                summary = dict(status=status,
                               position_m=(sample['pose'] or {}).get('position'),
                               visual_age_s=detail.get('visual_age_s'),
                               slip_suspected=detail.get('slip_suspected', False),
                               slip_residual_m=detail.get('slip_residual_m'),
                               reason=detail.get('last_error'),
                               accuracy_validated=False)
                self.status_pub.publish(self.String(data=json.dumps(summary, allow_nan=False)))
                self.last_status = status
                self.last_status_at = now

    def info(self):
        with self.lock:
            return dict(state='running' if self.active else 'idle',
                        status_topic=self.status_topic, odom_topic=self.topic,
                        odom_interface={
                            'schema': 'motus.odom/1', 'frame': 'body',
                            'provides': ['vx', 'vy', 'wz'], 'rate_hz': 20,
                            'pose_drift': 'unbounded',
                            'units': {'linear': 'm/s', 'angular': 'rad/s', 'length': 'm'}},
                        estimate=self._build(),
                        description=('Experimental read-only 2-D leg/IMU/visual EKF. '
                                     'Covariance grows without visual correction; '
                                     'not calibrated for full 6-DoF SLAM.'))

    def dispatch(self, action, args):
        if action == 'start':
            return {'state': 'running'} if self.start() else {'state': 'error'}
        if action == 'stop':
            return self.stop()
        if action in ('info', 'fused_odometry'):
            return self.info()
        return None
