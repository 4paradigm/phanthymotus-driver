"""Additional read-only inputs; no camera opener and no chassis commands."""
from __future__ import annotations
import json
import sys
import threading
import time
import uuid
from common.fusion_odometry import ClockMap, finite


def camera_metadata(color, depth, *, epoch, depth_scale, received_ns):
    def source(frame):
        stamp = float(frame.get_timestamp())
        finite([stamp], 1)
        return dict(source_ms=stamp, frame_number=int(frame.get_frame_number()),
                    domain=str(frame.get_frame_timestamp_domain()).rsplit('.', 1)[-1])
    if not 0 < depth_scale < .1:
        raise ValueError('invalid SDK depth scale')
    return dict(epoch=epoch, rgb=source(color), depth=source(depth),
                depth_scale=depth_scale, received_ns=received_ns,
                received_monotonic_ns=time.monotonic_ns())


def imu_reading(imu, begin, end, sequence, epoch):
    if end < begin:
        raise ValueError('IMU read clock reset')
    def covariance(name):
        raw = getattr(imu, name, None)
        return finite(list(raw), 9) if raw is not None else None
    return dict(epoch=epoch, sequence=sequence, source_stamp_ns=None, stamp_source='host_read',
                read_begin_ns=begin, read_end_ns=end,
                quaternion=finite(list(imu.ori), 4), gyro=finite(list(imu.angular_vel), 3),
                acceleration=finite(list(imu.linear_acc), 3),
                orientation_covariance=covariance('ori_cov'), gyro_covariance=covariance('angular_vel_cov'),
                acceleration_covariance=covariance('linear_acc_cov'))


class RawCameraPublisher:
    def __init__(self, node, namespace, pipeline, qos):
        from sensor_msgs.msg import Image, CameraInfo
        from std_msgs.msg import String
        self.Image, self.Info, self.String = Image, CameraInfo, String
        self.node, self.namespace = node, namespace
        root = f'/{namespace}/odometry_input'
        self.pubs = {name: node.create_publisher(cls, root+'/'+name, qos) for name, cls in
                     (('rgb', Image), ('depth', Image), ('info', CameraInfo), ('camera_meta', String))}
        self.depth_scale = pipeline.get_active_profile().get_device().first_depth_sensor().get_depth_scale()
        self.epoch = uuid.uuid4().hex
        self.clocks = [ClockMap(), ClockMap()]
        self.error_at = 0

    def publish(self, source_frames, aligned_frames):
        """Faults in the optional branch never take down the media producer."""
        import numpy as np
        try:
            source_color, source_depth = source_frames.get_color_frame(), source_frames.get_depth_frame()
            color, depth = aligned_frames.get_color_frame(), aligned_frames.get_depth_frame()
            if not all((source_color, source_depth, color, depth)):
                return
            wall = time.time_ns()
            meta = camera_metadata(source_color, source_depth, epoch=self.epoch,
                                   depth_scale=self.depth_scale, received_ns=wall)
            if meta['rgb']['domain'] != meta['depth']['domain']:
                raise ValueError('RGB/depth timestamp domains differ')
            stamps = []
            for index, key in enumerate(('rgb', 'depth')):
                src = meta[key]
                if index == 1 and self.clocks[1].offset is None:
                    self.clocks[1].offset = self.clocks[0].offset
                stamps.append(self.clocks[index].convert(src['source_ms'], src['domain'], wall))
            if abs(stamps[0]-stamps[1]) > 20_000_000:
                raise ValueError('RGB/depth capture skew exceeds 20 ms')
            optical = f'{self.namespace}_rgb_optical'
            for key, frame, stamp, encoding, channels in (
                    ('rgb', color, stamps[0], 'bgr8', 3), ('depth', depth, stamps[1], '32FC1', 4)):
                pixels = np.asanyarray(frame.get_data())
                if key == 'depth':
                    pixels = pixels.astype('<f4')*self.depth_scale
                msg = self.Image()
                msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(stamp, 10**9)
                msg.header.frame_id = optical
                msg.height, msg.width = pixels.shape[:2]
                msg.encoding, msg.is_bigendian, msg.step = encoding, False, msg.width*channels
                msg.data = pixels.tobytes()
                self.pubs[key].publish(msg)
                if key == 'rgb':
                    intr = color.profile.as_video_stream_profile().intrinsics
                    model = str(intr.model).rsplit('.', 1)[-1]
                    coefficients = finite(list(intr.coeffs), 5)
                    if model == 'inverse_brown_conrady' and not any(coefficients):
                        model = 'none'  # zero coefficients are exactly the pinhole model
                    if model not in ('none', 'brown_conrady'):
                        raise ValueError('unsupported color distortion model: '+model)
                    info = self.Info(); info.header = msg.header
                    info.width, info.height = msg.width, msg.height
                    info.distortion_model = 'plumb_bob'; info.d = coefficients
                    info.k = [intr.fx,0.,intr.ppx,0.,intr.fy,intr.ppy,0.,0.,1.]
                    info.r = [1.,0.,0.,0.,1.,0.,0.,0.,1.]
                    info.p = [intr.fx,0.,intr.ppx,0.,0.,intr.fy,intr.ppy,0.,0.,0.,1.,0.]
                    self.pubs['info'].publish(info)
            meta.update(rgb_stamp_ns=stamps[0], depth_stamp_ns=stamps[1],
                        clock_mapping='device_to_host' if meta['rgb']['domain'] == 'hardware_clock' else 'device_wall')
            self.pubs['camera_meta'].publish(self.String(data=json.dumps(meta, allow_nan=False)))
        except Exception as exc:
            if 'camera clock reset' in str(exc) or 'domain changed' in str(exc):
                self.clocks=[ClockMap(),ClockMap()]
                self.epoch=uuid.uuid4().hex
            if time.monotonic()-self.error_at >= 5:
                print('[odometry_input] raw camera unavailable: '+str(exc), flush=True)
                self.error_at = time.monotonic()


class BodyImuBridge:
    def __init__(self, namespace, executor, high_ctrl):
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from std_msgs.msg import String
        self.node = Node('bumi_odometry_imu_input')
        self.pub = self.node.create_publisher(String, f'/{namespace}/odometry_input/imu', qos_profile_sensor_data)
        self.String, self.ctrl, self.executor = String, high_ctrl, executor
        self.epoch, self.sequence = uuid.uuid4().hex, 0
        self.stopped = threading.Event(); self.thread = None
        executor.add_node(self.node)

    def start(self):
        def loop():
            error_at = 0
            while not self.stopped.is_set():
                try:
                    begin = time.time_ns(); imu = self.ctrl.get_imu_data(); end = time.time_ns()
                    self.sequence += 1
                    data = imu_reading(imu, begin, end, self.sequence, self.epoch)
                    self.pub.publish(self.String(data=json.dumps(data, allow_nan=False)))
                except Exception as exc:
                    if time.monotonic()-error_at > 5:
                        print('[odometry_input] IMU: '+str(exc), flush=True); error_at = time.monotonic()
                self.stopped.wait(.02)
        self.thread = threading.Thread(target=loop, daemon=True, name='bumi_odometry_imu')
        self.thread.start()

    def stop(self):
        self.stopped.set()
        if self.thread: self.thread.join(timeout=1)
        self.executor.remove_node(self.node); self.node.destroy_node()
