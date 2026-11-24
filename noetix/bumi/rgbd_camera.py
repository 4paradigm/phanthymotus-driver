"""Generic Perception inputs from the existing Bumi RealSense capture owner.

rs.align samples depth into the native color lens. Source depth-camera axial Z
is converted to color-camera Z on the sampled color rays using SDK extrinsics.
Both images are sampled on common rectified rays. Source capture times are
mapped once, never replaced by publication time. Body geometry is published
separately using the configured numeric mount or the capture-time TF tree.
"""
from array import array
from copy import deepcopy
import math
import os
import time
import threading
import uuid
import zlib

import numpy as np

try:
    from .visual_odometry_contract import ClockMap
except ImportError:
    from visual_odometry_contract import ClockMap


def fixed_body_mount(config):
    """Convert a fixed body-origin mount when its translation is supplied."""
    if not isinstance(config, dict) or config.get('fixed_mount') is False:
        return None
    mount = config.get('camera_to_body')
    if mount is None or (isinstance(mount, dict) and 'translation' not in mount):
        return None
    translation = np.asarray(mount['translation'], dtype=float)
    q = np.asarray(mount['quaternion'], dtype=float)
    if (translation.shape != (3,) or q.shape != (4,) or
            not np.isfinite(translation).all() or not np.isfinite(q).all() or
            not np.any(q)):
        raise ValueError('camera_to_body requires finite translation and nonzero xyzw quaternion')
    q = q / np.max(np.abs(q))
    x, y, z, w = q / np.linalg.norm(q)
    matrix = np.eye(4)
    matrix[:3, :3] = [[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                      [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                      [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]]
    matrix[:3, 3] = translation
    return matrix.tolist()


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
    # Optimized SDK alignment does not implement every nonzero Brown model.
    # The D435i's measured zero-coefficient profiles are exact pinholes.
    if (model not in ('none', 'brown_conrady', 'modified_brown_conrady', 'inverse_brown_conrady') or
            np.any(d != 0)):
        raise ValueError('unsupported source distortion: ' + model)
    x, y = cv2.initUndistortRectifyMap(k, d, np.eye(3), k, (640, 480), cv2.CV_32FC1)
    return dict(K=k.ravel().tolist(), map_x=np.clip(x, 0, 639), map_y=np.clip(y, 0, 479))


def color_depth_geometry(intr, extrinsics):
    """Recover color Z from the original depth Z retained by SDK alignment."""
    rotation = np.asarray(extrinsics.rotation, dtype=float)
    translation = np.asarray(extrinsics.translation, dtype=float)
    if rotation.size != 9 or translation.shape != (3,):
        raise ValueError('invalid RGB-D depth-to-color calibration shape')
    rotation = rotation.reshape(3, 3, order='F')  # SDK column-major R_COLOR_DEPTH
    if (not np.isfinite(rotation).all() or not np.isfinite(translation).all() or
            not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-6, rtol=0) or
            not np.isclose(np.linalg.det(rotation), 1., atol=1e-6, rtol=0)):
        raise ValueError('invalid RGB-D depth-to-color calibration')
    geometry = make_geometry(intr)
    # Match nearest-neighbour depth resampling: use its source color-pixel ray.
    pixels = np.stack((np.rint(geometry['map_x']), np.rint(geometry['map_y'])), axis=-1)
    rays = pixels.astype(np.float64)
    rays[:, :, 0] = (rays[:, :, 0]-intr.ppx)/intr.fx
    rays[:, :, 1] = (rays[:, :, 1]-intr.ppy)/intr.fy
    row = rotation.T[2]
    denominator = row[0]*rays[:, :, 0] + row[1]*rays[:, :, 1] + row[2]
    if not np.isfinite(denominator).all() or np.any(denominator <= 0):
        raise ValueError('RGB-D depth-to-color calibration has invalid optical rays')
    # Z_DEPTH = (R_DEPTH_COLOR * ray_COLOR).z * Z_COLOR + t_DEPTH_COLOR.z.
    geometry.update(depth_z_scale=1./denominator,
                    depth_z_offset=(rotation.T @ translation)[2]/denominator)
    return geometry


def camera_declarations(root, geometry, frame_id, clock_id, source_clock, offset_ns, *, source_epoch=None):
    common = dict(schema='motus.camera/1', id=root + '/rectified', width=640, height=480,
                  K=geometry['K'], D=[0.] * 5, distortion_model='pinhole',
                  source='derived-from-K', vendor=dict(frame_id=frame_id))
    color = dict(deepcopy(common), topic=root.removesuffix('/rgbd') + '/color', format='image/jpeg')
    depth = dict(deepcopy(common), topic=root.removesuffix('/rgbd') + '/depth', format='image/depth-zlib')
    depth['vendor'].update(depth_unit='mm')
    return [color, depth]


class RgbdCameraPublisher:
    """Sole normalized RGB-D producer for display, capture and computation."""

    def __init__(self, node, namespace, pipeline, qos, *, report=lambda status: None,
                 min_interval_s=0., mount_config=None):
        from sensor_msgs.msg import CompressedImage
        self.Message = CompressedImage
        self.root = f'/{namespace}/camera/rgbd'
        self.frame_id = f'{namespace}_rgb_optical'
        self.clock_id = 'realsense_capture/' + uuid.uuid4().hex
        self.scale = pipeline.get_active_profile().get_device().first_depth_sensor().get_depth_scale()
        if type(self.scale) not in (int, float) or not math.isfinite(self.scale) or not 0 < self.scale < .1:
            raise ValueError('invalid SDK depth scale')
        import pyrealsense2 as rs
        self.align_to_color = rs.align(rs.stream.color)
        self.pubs = {kind: node.create_publisher(CompressedImage, topic, qos)
                     for kind, topic in (('color', f'/{namespace}/camera/color'),
                                         ('depth', f'/{namespace}/camera/depth'))}
        self.capture_pub = node.create_publisher(
            CompressedImage, f'/{namespace}/camera/capture_color', qos)
        self.enabled = {'camera', 'depth', 'imu'}
        self.clocks = [ClockMap(), ClockMap()]
        self._clock_lock = threading.Lock()
        self._clock_error = None
        self.geometry = self.signature = None
        self.report = report
        self.ready = False
        self.last_error = None
        self.last_emit = None
        self.min_interval_s = min_interval_s
        from common.camera_mount import BodyMountSource
        try:
            fixed_matrix = fixed_body_mount(mount_config)
        except (ValueError, TypeError, KeyError):
            # An invalid optional mount must not stop the RGB-D image source.
            # The mount producer reports this invalid matrix as unavailable.
            fixed_matrix = []
        self.body_mount = BodyMountSource(node, self.root,
            body_frame=os.environ.get('CAMERA_BODY_FRAME', 'base_link'),
            camera_frame=lambda: self.frame_id,
            clock_id=lambda: self.clock_id + '/' + str(self.clocks[0].domain),
            fixed_matrix=fixed_matrix, numeric_only=True)
        self.mount_error = None
        self._reported_mount = None
        self.report(dict(ready=False, camera_info=[], error='awaiting calibrated capture',
                         body_mount_status=self.body_mount.status()))

    def set_enabled(self, cards, *, restarted=False):
        cards = set(cards)
        if cards != self.enabled or restarted:
            self.enabled = cards
            self.last_emit = None
            self.ready = False

    def _capture_stamps(self, sources, domains, received):
        with self._clock_lock:
            if self._clock_error:
                raise ValueError(self._clock_error)
            raw = [f.get_timestamp() for f in sources]
            if any(isinstance(v, bool) or not isinstance(v, (int, float)) or
                   not math.isfinite(v) or v <= 0 for v in raw):
                raise ValueError('invalid RGB-D source capture timestamp')
            if any(c.domain is not None and (domain != c.domain or int(ms*1e6) <= c.previous)
                   for c, domain, ms in zip(self.clocks, domains, raw)):
                self._clock_error = 'camera clock reset or domain changed; restart camera source'
                raise ValueError(self._clock_error)
            if domains[0] != domains[1]:
                raise ValueError('RGB/depth capture clock domains differ')
            # Commit both clocks only after the whole pair passes; an SDK IMU
            # callback cannot see a half-initialized or rejected image mapping.
            clocks = deepcopy(self.clocks)
            stamps = []
            for i, ms in enumerate(raw):
                if i == 1 and clocks[1].offset is None:
                    clocks[1].offset = clocks[0].offset
                stamps.append(clocks[i].convert(ms, domains[i], received))
            if abs(stamps[0]-stamps[1]) > 20_000_000:
                raise ValueError('RGB/depth capture skew exceeds 20 ms')
            self.clocks = clocks
            return stamps

    def close(self):
        self.body_mount.close()

    def unavailable(self, error):
        if self.ready or str(error) != self.last_error:
            # These failures cannot be repaired by another frame from the
            # negotiated source. Do not hold a startup request for ten seconds.
            terminal = ('intrinsics', 'distortion', '640x480', 'calibration',
                        'pixel geometry', 'clock reset or domain changed')
            self.report(dict(ready=False, camera_info=[], error=str(error)[:300],
                             retryable=not any(reason in str(error) for reason in terminal),
                             body_mount_status=self.body_mount.status()))
        self.ready = False
        self.last_error = str(error)

    def publish(self, source_frames):
        import cv2
        now = time.monotonic()
        if self.last_emit is not None and now - self.last_emit < self.min_interval_s:
            return
        sources = [source_frames.get_color_frame(), source_frames.get_depth_frame()]
        if not all(sources):
            raise ValueError('incomplete source RGB-D frameset')
        intr = sources[0].profile.as_video_stream_profile().intrinsics
        depth_intr = sources[1].profile.as_video_stream_profile().intrinsics
        # SSE/NEON SDK paths can silently ignore nonzero Brown coefficients.
        if any(np.any(np.asarray(i.coeffs) != 0) for i in (intr, depth_intr)):
            raise ValueError('unsupported source distortion for SDK depth-to-color alignment')
        aligned_frames = self.align_to_color.process(source_frames)
        color, depth = aligned_frames.get_color_frame(), aligned_frames.get_depth_frame()
        if not all((*sources, color, depth)):
            raise ValueError('incomplete source RGB-D frameset')
        try:
            extrinsics = sources[1].profile.get_extrinsics_to(sources[0].profile)
        except Exception as error:
            raise ValueError('RGB-D depth-to-color calibration unavailable') from error
        signature = tuple((i.width, i.height, i.fx, i.fy, i.ppx, i.ppy,
                           str(i.model), tuple(i.coeffs)) for i in (intr, depth_intr)) + (
                               tuple(extrinsics.rotation), tuple(extrinsics.translation))
        if self.signature != signature:
            if self.signature is not None:
                raise ValueError('RGB-D calibration changed; restart camera')
            self.geometry = color_depth_geometry(intr, extrinsics)
            self.signature = signature
        rgb, z16 = np.asanyarray(color.get_data()), np.asanyarray(depth.get_data())
        if rgb.shape != (480, 640, 3) or z16.shape != (480, 640) or z16.dtype.kind != 'u' or z16.dtype.itemsize != 2:
            raise ValueError('RGB-D source pixel geometry does not match calibration')
        domains = [str(f.get_frame_timestamp_domain()).rsplit('.', 1)[-1] for f in sources]
        received = time.time_ns()
        stamps = self._capture_stamps(sources, domains, received)
        g = self.geometry
        rectified = cv2.remap(rgb, g['map_x'], g['map_y'], cv2.INTER_LINEAR)
        measured = cv2.remap(z16, g['map_x'], g['map_y'], cv2.INTER_NEAREST)
        metric = np.rint((measured.astype(np.float64) * self.scale * g['depth_z_scale'] +
                         g['depth_z_offset']) * 1000.)
        metric[(measured == 0) | (metric < 1) | (metric > 65535)] = 0
        ok, jpeg = cv2.imencode('.jpg', rectified, [cv2.IMWRITE_JPEG_QUALITY, 90])
        if not ok:
            raise ValueError('RGB-D JPEG encoding failed')
        encoded = (jpeg.tobytes(), zlib.compress(metric.astype('<u2').tobytes(), 1))
        messages = []
        mount_stamps = set()
        for kind, stamp, payload, fmt in zip(('color', 'depth'), stamps, encoded,
                                            ('jpeg', '16UC1; compressedDepth zlib')):
            if ('camera' if kind == 'color' else 'depth') in self.enabled:
                publisher = self.pubs[kind]
            elif kind == 'color' and 'vision_capture' in self.enabled:
                publisher = self.capture_pub
            else:
                continue
            msg = self.Message()
            msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(stamp, 10**9)
            msg.header.frame_id = self.frame_id
            msg.format, msg.data = fmt, array('B', payload)
            messages.append((publisher, msg))
            mount_stamps.add(stamp)
        # Vision can use either RGB-D header, and point clouds keep the depth header.
        self.mount_error = None
        for stamp in sorted(mount_stamps):
            try:
                self.body_mount.publish(stamp)
            except Exception as error:
                self.mount_error = str(error)
        for publisher, msg in messages:
            publisher.publish(msg)
        self.last_emit = now
        self.last_error = None
        mount_status = self.body_mount.status()
        mount = self.body_mount.declaration()
        if self.mount_error:
            mount_status = dict(mount_status, status='unavailable', reason=self.mount_error)
            mount = dict(mount, status='unavailable', reason=self.mount_error)
        if not self.ready or mount != self._reported_mount:
            self.ready = True
            declarations = camera_declarations(
                self.root, g, self.frame_id, self.clock_id + '/' + domains[0],
                domains[0], self.clocks[0].offset, source_epoch=self.clock_id)
            for declaration in declarations:
                declaration['vendor']['body_mount'] = deepcopy(mount)
            self.report(dict(ready=True, error=None, camera_info=declarations,
                             body_mount_status=mount_status))
            self._reported_mount = deepcopy(mount)
