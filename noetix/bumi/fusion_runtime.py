"""Managed RGB-D pose + gyro EKF; never sends commands or opens a camera."""
from __future__ import annotations

from collections import deque
import importlib
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import uuid

from camera_runtime import CAMERA

# SDK body IMU bridged by the driver (device.py _BODY_IMU_TOPIC). The host
# kernel lacks CONFIG_HID_SENSOR_HUB, so librealsense cannot see the camera
# IMU on this generation of robots — keep in sync with device.py.
BODY_IMU = '/bumi_odom/body_imu'


def check_dependencies(prefix_lookup=None):
    if prefix_lookup is None:
        from ament_index_python.packages import get_package_prefix
        prefix_lookup = get_package_prefix
    entries = {'realsense2_camera': 'lib/realsense2_camera/realsense2_camera_node',
               'rtabmap_odom': 'lib/rtabmap_odom/rgbd_odometry',
               'robot_localization': 'lib/robot_localization/ekf_node'}
    for package, entry in entries.items():
        try:
            root = Path(prefix_lookup(package))
            if not (root/entry).is_file():
                raise FileNotFoundError(entry)
        except Exception as exc:
            raise RuntimeError(f'missing required fusion dependency {package}: {exc}') from exc
    for module in ('rclpy', 'tf2_ros', 'sensor_msgs.msg', 'nav_msgs.msg',
                   'rtabmap_msgs.msg', 'launch_ros', 'cv2', 'numpy', 'yaml'):
        importlib.import_module(module)


def fusion_parameters(roll, pitch):
    visual = dict(frame_id='bumi_odom_base_link', odom_frame_id='bumi_odom_odom',
                  publish_tf=False, wait_imu_to_init=False, approx_sync=True,
                  approx_sync_max_interval=0.02, qos=2, qos_camera_info=2,
                  topic_queue_size=5, sync_queue_size=5, max_update_rate=10.0,
                  initial_pose=f'0 0 0 {roll} {pitch} 0', **{'Odom/ResetCountdown': '0'})
    ekf = dict(frequency=30.0, sensor_timeout=0.5, two_d_mode=False, publish_tf=True,
               map_frame='bumi_odom_map', odom_frame='bumi_odom_odom',
               base_link_frame='bumi_odom_base_link', world_frame='bumi_odom_odom',
               odom0='/bumi_odom/visual', odom0_config=[True]*6+[False]*9,
               odom0_differential=False, odom0_relative=False, odom0_queue_size=10,
               imu0=BODY_IMU, imu0_config=[False]*9+[True]*3+[False]*3,
               imu0_differential=False, imu0_remove_gravitational_acceleration=False,
               imu0_queue_size=50)
    return visual, ekf


class GravityWindow:
    def __init__(self):
        self.samples = deque()

    def observe(self, stamp_s, acceleration, angular):
        if (not all(math.isfinite(v) for v in (*acceleration, *angular))
                or not 8 <= math.hypot(*acceleration) <= 11
                or math.hypot(*angular) > 0.1
                or (self.samples and stamp_s <= self.samples[-1][0])):
            self.samples.clear()
            return None
        self.samples.append((stamp_s, acceleration))
        while len(self.samples) > 1 and stamp_s-self.samples[1][0] >= 1:
            self.samples.popleft()
        if len(self.samples) < 20 or stamp_s-self.samples[0][0] < 1:
            return None
        mean = [sum(row[1][i] for row in self.samples)/len(self.samples) for i in range(3)]
        if any(max(row[1][i] for row in self.samples)-min(row[1][i] for row in self.samples) > 0.3
               for i in range(3)):
            return None
        x, y, z = mean
        return math.atan2(y, z), math.atan2(-x, math.hypot(y, z))


class FusionHealth:
    kinds = ('imu', 'visual', 'diagnostic', 'filtered')

    def __init__(self, started_ns=0):
        self.segment = uuid.uuid4().hex
        self.started_ns = started_ns
        self.samples = {}
        self.reason = None
        self.last_clock = None

    def observe(self, kind, stamp, now, valid):
        if stamp < self.started_ns:
            return  # DDS may still deliver queued samples from the old EKF.
        previous = self.samples.get(kind)
        if previous is None and not valid:
            return  # RTAB/EKF startup may publish an uninitialized estimate.
        if (not valid or stamp <= 0 or not -50_000_000 <= now-stamp <= 500_000_000
                or (previous is not None and stamp <= previous)):
            self.reason = self.reason or f'invalid or reset {kind} feedback'
        self.samples[kind] = stamp

    def status(self, now):
        if self.last_clock is not None and now < self.last_clock:
            self.reason = self.reason or 'ROS clock reset'
        self.last_clock = now
        stale = [kind for kind, stamp in self.samples.items() if now-stamp > 500_000_000]
        if stale:
            self.reason = self.reason or 'stale '+', '.join(stale)
        missing = [kind for kind in self.kinds if kind not in self.samples]
        if self.reason:
            state = 'lost'
        elif missing:
            state = 'initializing'
        else:
            state = 'tracking'
        return dict(status=state, reason=self.reason or ('waiting for '+', '.join(missing) if missing else None),
                    segment_id=self.segment, stamp_ns=now,
                    reference='body origin; gravity-level initial forward axis')


def pose_valid(msg):
    p, q = msg.pose.pose.position, msg.pose.pose.orientation
    return (msg.header.frame_id == 'bumi_odom_odom' and msg.child_frame_id == 'bumi_odom_base_link'
            and all(math.isfinite(v) for v in (p.x, p.y, p.z, q.x, q.y, q.z, q.w))
            and math.hypot(q.x, q.y, q.z, q.w) > 1e-6
            and all(0 < msg.pose.covariance[i] < 1000 for i in (0, 7, 14, 21, 28, 35)))


def run():
    import yaml
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from tf2_ros import StaticTransformBroadcaster
    from geometry_msgs.msg import TransformStamped
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Imu
    from std_msgs.msg import String
    from rtabmap_msgs.msg import OdomInfo

    # Limit only fusion: the compatibility camera/voice processes keep priority.
    os.nice(10)
    if hasattr(os, 'sched_getaffinity'):
        allowed = sorted(os.sched_getaffinity(0))
        os.sched_setaffinity(0, allowed[-2:])
    rclpy.init()
    node = Node('bumi_managed_fusion', namespace='bumi_odom')
    health, gravity = FusionHealth(node.get_clock().now().nanoseconds), GravityWindow()
    mount = StaticTransformBroadcaster(node)
    transform = TransformStamped()
    transform.header.stamp = node.get_clock().now().to_msg()
    transform.header.frame_id, transform.child_frame_id = 'bumi_odom_base_link', 'bumi_odom_camera_link'
    transform.transform.rotation.w = 1.0
    mount.sendTransform(transform)
    publisher = node.create_publisher(String, '/bumi_odom/status', 10)
    children = []
    initial = None
    previous_distance = None
    lost_since = None
    now = lambda: node.get_clock().now().nanoseconds
    stamp = lambda msg: msg.header.stamp.sec*10**9+msg.header.stamp.nanosec

    def imu(msg):
        nonlocal initial
        a, w = msg.linear_acceleration, msg.angular_velocity
        valid = (all(math.isfinite(v) for v in (a.x, a.y, a.z, w.x, w.y, w.z))
                 and all(math.isfinite(msg.angular_velocity_covariance[i])
                         and msg.angular_velocity_covariance[i] > 0 for i in (0, 4, 8)))
        health.observe('imu', stamp(msg), now(), valid)
        if initial is None and valid:
            # Body-frame samples: no camera-mount lookup, gravity window
            # settles roll/pitch directly.
            initial = gravity.observe(stamp(msg)/1e9, (a.x, a.y, a.z),
                                      (w.x, w.y, w.z))

    def visual(msg):
        if children:
            health.observe('visual', stamp(msg), now(), pose_valid(msg))

    def filtered(msg):
        if children:
            health.observe('filtered', stamp(msg), now(), pose_valid(msg))

    def diagnostic(msg):
        nonlocal previous_distance
        if not children:
            return
        value = msg.distance_travelled
        valid = (not msg.lost and math.isfinite(value)
                 and (previous_distance is None or value >= previous_distance-1e-6))
        previous_distance = value
        health.observe('diagnostic', stamp(msg), now(), valid)

    for cls, topic, callback in ((Imu, BODY_IMU, imu), (Odometry, '/bumi_odom/visual', visual),
                                  (Odometry, '/bumi_odom/filtered', filtered),
                                  (OdomInfo, '/bumi_odom/visual_info', diagnostic)):
        node.create_subscription(cls, topic, callback, qos_profile_sensor_data)

    def stop_children():
        for proc in reversed(children):
            if proc.poll() is None:
                proc.terminate()
                try:
                    proc.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    proc.kill()
            proc.wait(timeout=2)
        children.clear()

    with tempfile.TemporaryDirectory(prefix='bumi-fusion-') as directory:
        def start_children():
            visual, ekf = fusion_parameters(*initial)
            for name, package, executable, parameters, remaps in (
                    ('visual', 'rtabmap_odom', 'rgbd_odometry', visual,
                     [('rgb/image', CAMERA+'/color/image_raw'),
                      ('depth/image', CAMERA+'/aligned_depth_to_color/image_raw'),
                      ('rgb/camera_info', CAMERA+'/color/camera_info'),
                      ('imu', '/bumi_odom/unused_visual_imu'),
                      ('odom', '/bumi_odom/visual'), ('odom_info', '/bumi_odom/visual_info')]),
                    ('fusion', 'robot_localization', 'ekf_node', ekf,
                     [('odometry/filtered', '/bumi_odom/filtered')])):
                path = Path(directory)/(name+'.yaml')
                path.write_text(yaml.safe_dump({'/**': {'ros__parameters': parameters}}))
                from ament_index_python.packages import get_package_prefix
                binary = Path(get_package_prefix(package))/'lib'/package/executable
                args = [str(binary), '--ros-args', '-r', '__node:='+name,
                        '-r', '__ns:=/bumi_odom', '--params-file', str(path)]
                for source, destination in remaps:
                    args += ['-r', source+':='+destination]
                # Inherit the manager's process group: even a killed manager
                # cannot leave an old EKF publishing into its replacement.
                children.append(subprocess.Popen(args))

        def tick():
            nonlocal health, gravity, initial, previous_distance, lost_since
            current = now()
            if not children and initial is not None:
                start_children()
            if any(proc.poll() is not None for proc in children):
                health.reason = 'fusion process exited'
            status = health.status(current)
            if initial is None and status['status'] == 'initializing':
                status['reason'] = 'waiting for one stationary second of camera IMU and factory TF'
            publisher.publish(String(data=json.dumps(status)))
            if status['status'] == 'lost':
                lost_since = current if lost_since is None else lost_since
                # Keep lost visible before restarting into a new segment.
                if current-lost_since >= 1_000_000_000:
                    stop_children()
                    health, gravity = FusionHealth(now()), GravityWindow()
                    initial = previous_distance = lost_since = None
        node.create_timer(0.1, tick)
        try:
            rclpy.spin(node)
        except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
            pass
        finally:
            stop_children()
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == '__main__':
    from common import logsafe
    logsafe.install(check_fd=False)
    check_dependencies()
    if '--check-dependencies' not in sys.argv:
        run()
