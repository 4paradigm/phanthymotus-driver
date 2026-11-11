"""G1 computation-only RGB-D from the existing RealSense capture owner.

SDK alignment supplies color in depth optical coordinates. The same inverse
rectification map then samples both streams; display topics are untouched.
Only SDK global-time timestamps are accepted, never callback receipt times.
"""
from copy import deepcopy
import math
import time
import zlib

import cv2
import numpy as np


class RgbdSource:
    def __init__(self, root, rs):
        self.root, self.rs = root, rs
        self.geometry = None
        self.previous = [0, 0]
        self.last_capture_ns = 0
        self.reason = 'waiting for SDK calibration and global capture time'

    def configure(self, profile):
        self.geometry = None
        self.reason = "waiting for SDK calibration and global capture time"
        intr = profile.get_stream(self.rs.stream.depth).as_video_stream_profile().get_intrinsics()
        values = [intr.width, intr.height, intr.fx, intr.fy, intr.ppx, intr.ppy]
        if not all(math.isfinite(v) for v in values) or min(values[:4]) <= 0:
            raise ValueError('invalid RealSense depth intrinsics')
        coeffs = np.asarray(intr.coeffs, float)
        if coeffs.shape != (5,) or not np.isfinite(coeffs).all():
            raise ValueError('invalid RealSense distortion coefficients')
        if intr.model == self.rs.distortion.none:
            if np.any(coeffs):
                raise ValueError('nonzero coefficients for distortion-free profile')
        elif intr.model != self.rs.distortion.brown_conrady and not (
                intr.model in (self.rs.distortion.inverse_brown_conrady,
                               self.rs.distortion.modified_brown_conrady) and not np.any(coeffs)):
            # Modified and inverse Brown have different equations from OpenCV.
            raise ValueError('unsupported RealSense depth distortion model')
        self.scale = profile.get_device().first_depth_sensor().get_depth_scale()
        if not math.isfinite(self.scale) or self.scale <= 0:
            raise ValueError('invalid measured RealSense depth scale')
        raw_k = np.array([[intr.fx, 0., intr.ppx], [0., intr.fy, intr.ppy], [0., 0., 1.]])
        self.k = raw_k.copy()
        self.k[0] *= 640 / intr.width
        self.k[1] *= 480 / intr.height
        self.maps = cv2.initUndistortRectifyMap(raw_k, coeffs, None, self.k, (640, 480), cv2.CV_32FC1)
        self.shape = (intr.height, intr.width)
        self.aligner = self.rs.align(self.rs.stream.depth)
        self.geometry = True
        self.previous = [0, 0]
        self.last_capture_ns = 0

    def motion_stamp(self, source_ms, domain):
        if domain != 'global_time':
            raise ValueError('SDK global capture clock unavailable for motion')
        stamp = int(source_ms // 1000) * 1_000_000_000 + round((source_ms % 1000) * 1_000_000)
        if not -50_000_000 <= time.time_ns() - stamp <= 500_000_000:
            raise ValueError('stale or future motion capture time')
        return stamp

    def encode(self, frames, now_ns):
        self.reason = 'waiting for valid capture'
        if self.geometry is None:
            raise ValueError('SDK calibration unavailable')
        raw = [frames.get_color_frame(), frames.get_depth_frame()]
        if not all(raw):
            raise ValueError('missing color or measured depth capture')
        if any(f.get_frame_timestamp_domain() != self.rs.timestamp_domain.global_time for f in raw):
            raise ValueError('SDK global capture clock unavailable')
        timestamps = [f.get_timestamp() for f in raw]
        if not all(math.isfinite(t) and t > 0 for t in timestamps):
            raise ValueError('invalid capture time')
        # Split epoch seconds before multiplying to avoid losing nanoseconds.
        stamps = [int(t // 1000) * 1_000_000_000 + round((t % 1000) * 1_000_000)
                  for t in timestamps]
        if (any(s <= old or not -50_000_000 <= now_ns - s <= 1_000_000_000
                for s, old in zip(stamps, self.previous)) or abs(stamps[0] - stamps[1]) > 20_000_000):
            raise ValueError('stale, repeated, future or skewed capture time')
        aligned = self.aligner.process(frames)
        color_frame, depth_frame = aligned.get_color_frame(), aligned.get_depth_frame()
        if not color_frame or not depth_frame:
            raise ValueError('SDK alignment did not produce RGB-D')
        color = np.asarray(color_frame.get_data())
        depth = np.asarray(depth_frame.get_data())
        if color.shape != self.shape + (3,) or depth.shape != self.shape or depth.dtype != np.uint16:
            raise ValueError('unexpected aligned RGB-D shape or measured depth encoding')
        color = cv2.remap(color, *self.maps, cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
        depth = cv2.remap(depth, *self.maps, cv2.INTER_NEAREST, borderMode=cv2.BORDER_CONSTANT)
        mm = np.rint(depth.astype(float) * (self.scale * 1000))
        mm[(mm <= 0) | (mm > 65535)] = 0
        ok, jpeg = cv2.imencode('.jpg', color, [cv2.IMWRITE_JPEG_QUALITY, 80])
        if not ok:
            raise ValueError('RGB-D JPEG encoding failed')
        self.previous = stamps
        self.last_capture_ns = min(stamps)
        self.reason = None
        frame = 'camera_depth_optical_frame'
        return (dict(data=jpeg.tobytes(), stamp_ns=stamps[0], frame_id=frame, format='jpeg'),
                dict(data=zlib.compress(mm.astype('<u2').tobytes(), 1), stamp_ns=stamps[1],
                     frame_id=frame, format='16UC1; compressedDepth zlib'))

    def info(self):
        outputs = [dict(topic=self.root + '/color', format='image/jpeg'),
                   dict(topic=self.root + '/depth', format='image/depth-zlib')]
        declarations = []
        if self.reason is None:
            common = dict(schema='motus.camera/1', id=self.root + '/rectified', width=640, height=480,
                          K=self.k.ravel().tolist(), D=[0.] * 5, distortion_model='pinhole',
                          source='derived-from-K', vendor=dict(frame_id='camera_depth_optical_frame',
                              clock_id='realsense_sdk_global_time_unix', registration='registered_rectified',
                              registration_source='librealsense_align_color_to_depth_then_opencv_rectify',
                              timestamp_semantics='source_capture_mapped'))
            declarations = [dict(deepcopy(common), **port) for port in outputs]
            declarations[1]['vendor'].update(depth_source='measured', depth_unit='mm')
        return dict(state='ready' if declarations else 'unavailable', reason=self.reason,
                    topic_out=outputs, camera_info=declarations, last_capture_ns=self.last_capture_ns)
