"""Raw RGB-D inputs for the bundle-local visual odometry card.

Read-only additions to the direct capture process: no camera opener and no
chassis commands. The capture process owns the USB device and hands complete
framesets here; nothing in this optional branch may take JPEG/Z16 media
production down with it.
"""
from __future__ import annotations

import json
import time
import uuid

try:  # imported as part of the noetix.bumi namespace in tests
    from .visual_odometry_contract import ClockMap, finite
except ImportError:  # imported as a top-level /work module by the camera subprocess
    from visual_odometry_contract import ClockMap, finite


def camera_metadata(color, depth, *, epoch, depth_scale, received_ns):
    """Provenance for the source and aligned frames, kept ROS-free for tests."""
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


class RawCameraPublisher:
    """Publish device-timed raw RGB/depth plus CameraInfo and provenance JSON."""

    def __init__(self, node, namespace, pipeline, qos, min_interval_s=0.0):
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
        # The card is off by default; when enabled, sample the raw branch at a
        # bounded rate instead of duplicating every 30 Hz capture frame over DDS.
        self.min_interval_s = max(0.0, float(min_interval_s))
        self.last_publish = 0.0

    def publish(self, source_frames, aligned_frames):
        """Faults in the optional branch never take down the media producer."""
        import numpy as np
        try:
            source_color, source_depth = source_frames.get_color_frame(), source_frames.get_depth_frame()
            color, depth = aligned_frames.get_color_frame(), aligned_frames.get_depth_frame()
            if not all((source_color, source_depth, color, depth)):
                return
            now = time.monotonic()
            if self.min_interval_s > 0 and now - self.last_publish < self.min_interval_s:
                return
            self.last_publish = now
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
            # The field D435i reports inverse_brown_conrady with all-zero
            # coefficients; that is exactly the pinhole model, not a distortion
            # we can advertise as plumb_bob. Validate before publishing so a
            # rejected model never emits a half frameset.
            intr = color.profile.as_video_stream_profile().intrinsics
            model = str(intr.model).rsplit('.', 1)[-1]
            coefficients = finite(list(intr.coeffs), 5)
            if model == 'inverse_brown_conrady' and not any(coefficients):
                model = 'none'  # zero coefficients are exactly the pinhole model
            if model not in ('none', 'brown_conrady'):
                raise ValueError('unsupported color distortion model: '+model)
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
                    rgb_msg = msg
            info = self.Info()
            info.header = rgb_msg.header
            info.width, info.height = rgb_msg.width, rgb_msg.height
            info.distortion_model = 'plumb_bob'
            info.d = coefficients
            info.k = [intr.fx, 0., intr.ppx, 0., intr.fy, intr.ppy, 0., 0., 1.]
            info.r = [1., 0., 0., 0., 1., 0., 0., 0., 1.]
            info.p = [intr.fx, 0., intr.ppx, 0., 0., intr.fy, intr.ppy, 0., 0., 0., 1., 0.]
            self.pubs['info'].publish(info)
            meta.update(rgb_stamp_ns=stamps[0], depth_stamp_ns=stamps[1],
                        clock_mapping='device_to_host' if meta['rgb']['domain'] == 'hardware_clock' else 'device_wall')
            self.pubs['camera_meta'].publish(self.String(data=json.dumps(meta, allow_nan=False)))
        except Exception as exc:
            if 'camera clock reset' in str(exc) or 'domain changed' in str(exc):
                self.clocks = [ClockMap(), ClockMap()]
                self.epoch = uuid.uuid4().hex
            if time.monotonic()-self.error_at >= 5:
                print('[odometry_input] raw camera unavailable: '+str(exc), flush=True)
                self.error_at = time.monotonic()
