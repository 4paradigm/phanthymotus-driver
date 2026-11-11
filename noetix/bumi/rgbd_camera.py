"""Generic Perception inputs from the existing Bumi RealSense capture owner.

rs.align samples color into the native depth lens, preserving depth axial Z.
Both images are then sampled on common rectified rays; source capture times are mapped once, never replaced by
publication time. This is a camera optical frame, with no assumed body mount.
"""
from array import array
from copy import deepcopy
import math
import time
import uuid
import zlib

import numpy as np

try:
    from .visual_odometry_contract import ClockMap
except ImportError:
    from visual_odometry_contract import ClockMap

STATUS_MARKER = '[camera_subprocess] RGBD '


def make_geometry(intr):
    import cv2
    if (intr.width, intr.height) != (640, 480):
        raise ValueError('RGB-D requires 640x480 color intrinsics')
    k = np.array([[intr.fx, 0., intr.ppx], [0., intr.fy, intr.ppy], [0., 0., 1.]])
    if not np.isfinite(k).all() or intr.fx <= 0 or intr.fy <= 0:
        raise ValueError('invalid RGB-D intrinsics')
    d = np.asarray(intr.coeffs, dtype=float)
    model = str(intr.model).rsplit('.', 1)[-1]
    if d.shape != (5,) or not np.isfinite(d).all():
        raise ValueError('invalid color distortion coefficients')
    # OpenCV's forward Brown model cannot represent RealSense inverse Brown.
    # The D435i's zero-coefficient inverse model is exactly a pinhole.
    if (model not in ('none', 'brown_conrady', 'modified_brown_conrady', 'inverse_brown_conrady') or
            (np.any(d != 0) and model != 'brown_conrady')):
        raise ValueError('unsupported source distortion: ' + model)
    x, y = cv2.initUndistortRectifyMap(k, d, np.eye(3), k, (640, 480), cv2.CV_32FC1)
    return dict(K=k.ravel().tolist(), map_x=x, map_y=y)


def camera_declarations(root, geometry, frame_id, clock_id, source_clock, offset_ns):
    common = dict(schema='motus.camera/1', id=root + '/rectified', width=640, height=480,
                  K=geometry['K'], D=[0.] * 5, distortion_model='pinhole',
                  source='derived-from-K', vendor=dict(
                      frame_id=frame_id, clock_id=clock_id,
                      registration='registered_rectified',
                      registration_source='realsense_align_color_to_depth_then_rectify',
                      timestamp_semantics='source_capture_mapped',
                      source_clock=source_clock, mapping_offset_ns=offset_ns,
                      clock_mapping='fixed_arrival_offset' if source_clock == 'hardware_clock' else 'device_wall',
                      clock_mapping_verified=False))
    color = dict(deepcopy(common), topic=root + '/color', format='image/jpeg')
    depth = dict(deepcopy(common), topic=root + '/depth', format='image/depth-zlib')
    depth['vendor'].update(depth_unit='mm', depth_source='measured')
    return [color, depth]


class RgbdCameraPublisher:
    """One optional computation branch; never opens a camera or commands motion."""

    def __init__(self, node, namespace, pipeline, qos, *, report=lambda status: None,
                 min_interval_s=0.):
        from sensor_msgs.msg import CompressedImage
        self.Message = CompressedImage
        self.root = f'/{namespace}/camera/rgbd'
        self.frame_id = f'{namespace}_depth_optical'
        self.clock_id = 'realsense_capture/' + uuid.uuid4().hex
        self.scale = pipeline.get_active_profile().get_device().first_depth_sensor().get_depth_scale()
        if not isinstance(self.scale, (int, float)) or not math.isfinite(self.scale) or not 0 < self.scale < .1:
            raise ValueError('invalid SDK depth scale')
        import pyrealsense2 as rs
        self.align_to_depth = rs.align(rs.stream.depth)
        self.pubs = {kind: node.create_publisher(CompressedImage, self.root + '/' + kind, qos)
                     for kind in ('color', 'depth')}
        self.clocks = [ClockMap(), ClockMap()]
        self.geometry = self.signature = None
        self.report = report
        self.ready = False
        self.last_error = None
        self.last_emit = None
        self.min_interval_s = min_interval_s

    def unavailable(self, error):
        if self.ready or self.last_error is None:
            self.report(dict(ready=False, camera_info=[], error=str(error)[:300]))
        self.ready = False
        self.last_error = str(error)

    def publish(self, source_frames):
        import cv2
        now = time.monotonic()
        if self.last_emit is not None and now - self.last_emit < self.min_interval_s:
            return
        sources = [source_frames.get_color_frame(), source_frames.get_depth_frame()]
        aligned_frames = self.align_to_depth.process(source_frames)
        color, depth = aligned_frames.get_color_frame(), aligned_frames.get_depth_frame()
        if not all((*sources, color, depth)):
            raise ValueError('incomplete source RGB-D frameset')
        intr = depth.profile.as_video_stream_profile().intrinsics
        signature = (intr.width, intr.height, intr.fx, intr.fy, intr.ppx, intr.ppy,
                     str(intr.model), tuple(intr.coeffs))
        if self.signature != signature:
            if self.signature is not None:
                raise ValueError('RGB-D calibration changed; restart camera')
            self.geometry = make_geometry(intr)
            self.signature = signature
        rgb, z16 = np.asanyarray(color.get_data()), np.asanyarray(depth.get_data())
        if rgb.shape != (480, 640, 3) or z16.shape != (480, 640) or z16.dtype.kind != 'u' or z16.dtype.itemsize != 2:
            raise ValueError('RGB-D source pixel geometry does not match calibration')
        domains = [str(f.get_frame_timestamp_domain()).rsplit('.', 1)[-1] for f in sources]
        if domains[0] != domains[1]:
            raise ValueError('RGB/depth capture clock domains differ')
        received = time.time_ns()
        stamps = []
        for index, frame in enumerate(sources):
            if index == 1 and self.clocks[1].offset is None:
                self.clocks[1].offset = self.clocks[0].offset
            stamps.append(self.clocks[index].convert(float(frame.get_timestamp()), domains[index], received))
        if abs(stamps[0] - stamps[1]) > 20_000_000:
            raise ValueError('RGB/depth capture skew exceeds 20 ms')
        g = self.geometry
        rectified = cv2.remap(rgb, g['map_x'], g['map_y'], cv2.INTER_LINEAR)
        measured = cv2.remap(z16, g['map_x'], g['map_y'], cv2.INTER_NEAREST)
        metric = np.rint(measured.astype(np.float64) * (self.scale * 1000.))
        metric[(metric < 1) | (metric > 65535)] = 0
        ok, jpeg = cv2.imencode('.jpg', rectified, [cv2.IMWRITE_JPEG_QUALITY, 90])
        if not ok:
            raise ValueError('RGB-D JPEG encoding failed')
        encoded = (jpeg.tobytes(), zlib.compress(metric.astype('<u2').tobytes(), 1))
        messages = []
        for kind, stamp, payload, fmt in zip(('color', 'depth'), stamps, encoded,
                                            ('jpeg', '16UC1; compressedDepth zlib')):
            msg = self.Message()
            msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(stamp, 10**9)
            msg.header.frame_id = self.frame_id
            msg.format, msg.data = fmt, array('B', payload)
            messages.append((kind, msg))
        for kind, msg in messages:
            self.pubs[kind].publish(msg)
        self.last_emit = now
        self.last_error = None
        if not self.ready:
            self.ready = True
            self.report(dict(ready=True, error=None, camera_info=camera_declarations(
                self.root, g, self.frame_id, self.clock_id + '/' + domains[0],
                domains[0], self.clocks[0].offset)))
