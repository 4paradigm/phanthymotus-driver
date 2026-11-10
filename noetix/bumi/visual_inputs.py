"""Raw RGB-D inputs for the bundle-local visual odometry card.

Read-only additions to the direct capture process: no camera opener and no
chassis commands. The capture process owns the USB device and hands complete
framesets here; nothing in this optional branch may take JPEG/Z16 media
production down with it.
"""
from __future__ import annotations

from array import array
import hashlib
import json
import time
import uuid
import zlib

try:  # imported as part of the noetix.bumi namespace in tests
    from .visual_odometry_contract import ClockMap, MAX_TRANSPORT_NS, finite
except ImportError:  # imported as a top-level /work module by the camera subprocess
    from visual_odometry_contract import ClockMap, MAX_TRANSPORT_NS, finite


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


class CaptureTimeMapper:
    """One mapping per camera process, shared by VO and the depth output."""

    def __init__(self, depth_scale):
        self.depth_scale = depth_scale
        self.epoch = uuid.uuid4().hex
        # Stable for this capture subprocess; epoch changes on clock discontinuity.
        self.source_generation = uuid.uuid4().hex
        self.clocks = [ClockMap(), ClockMap()]
        self.cached_key = self.cached_value = None

    def sample(self, color, depth):
        wall = time.time_ns()
        meta = camera_metadata(color, depth, epoch=self.epoch,
                               depth_scale=self.depth_scale, received_ns=wall)
        key = tuple((meta[k]['source_ms'], meta[k]['frame_number'], meta[k]['domain'])
                    for k in ('rgb', 'depth'))
        if key == self.cached_key:
            if any(not -50_000_000 <= wall - stamp <= MAX_TRANSPORT_NS
                   for stamp in self.cached_value[1]):
                raise ValueError('camera clock mapping drift or transport delay')
            return self.cached_value
        try:
            if meta['rgb']['domain'] != meta['depth']['domain']:
                raise ValueError('RGB/depth timestamp domains differ')
            stamps = []
            for index, kind in enumerate(('rgb', 'depth')):
                src = meta[kind]
                if index == 1 and self.clocks[1].offset is None:
                    self.clocks[1].offset = self.clocks[0].offset
                stamps.append(self.clocks[index].convert(src['source_ms'], src['domain'], wall))
            if abs(stamps[0] - stamps[1]) > 20_000_000:
                raise ValueError('RGB/depth capture skew exceeds 20 ms')
            meta.update(rgb_stamp_ns=stamps[0], depth_stamp_ns=stamps[1],
                        clock_mapping='device_to_host' if meta['rgb']['domain'] == 'hardware_clock' else 'device_wall')
            self.cached_key, self.cached_value = key, (meta, stamps)
            return self.cached_value
        except Exception as exc:
            if 'camera clock reset' in str(exc) or 'domain changed' in str(exc):
                self.clocks = [ClockMap(), ClockMap()]
                self.epoch = uuid.uuid4().hex
                self.cached_key = self.cached_value = None
            raise


def depth_frame_metadata(capture, stamp_ns, frame_id, payload, source_generation):
    """Correlate one capture-mapped depth frame with its exact zlib data bytes.

    An arrival-offset clock is never certified by a plausible-looking offset.
    Fallback frames have empty frame_id and intentionally get no sidecar.
    """
    depth = capture['depth']
    source_ms = float(depth['source_ms'])
    source_ns = int(source_ms * 1_000_000)
    if type(stamp_ns) is not int or stamp_ns <= 0 or not frame_id:
        raise ValueError('only capture-mapped depth frames may have provenance')
    return {
        'schema': 'motus.slam_frame/1',
        'stamp_ns': stamp_ns, 'frame_id': frame_id, 'clock_id': 'ros',
        'clock_epoch': capture['epoch'],
        'source_generation': source_generation,
        'source_clock': str(depth['domain']),
        'source_stamp': source_ms, 'source_unit': 'ms',
        'mapping': {'method': capture['clock_mapping'],
                    'version': 'shared_camera_mapper/1',
                    'offset_ns': stamp_ns-source_ns,
                    'verified': False, 'uncertainty_ns': None},
        'data_sha256': hashlib.sha256(bytes(payload)).hexdigest(),
    }


class CaptureTimedDepthPublisher:
    """The depth card's single output: capture-timed 640x480 uint16 mm zlib.

    The canvas, navi and the point-cloud computation all read this one topic.
    It is published on every aligned frame (30 Hz), not gated on subscribers
    and not throttled: it is the card's media output, and a viewer opening late
    must not change what the rest of the robot receives.

    Frames whose capture time cannot be mapped are still published, with the
    arrival stamp and an empty ``frame_id``. Display/navi decode them normally,
    while a consumer that requires proven capture time (the point-cloud
    computation) skips them instead of mistaking publication time for
    measurement time. The SDK ``depth_scale`` is applied here so the payload is
    the declared uint16 millimetre contract on every camera, not only on the
    usual 1 mm scale.
    """

    def __init__(self, publisher, namespace, time_mapper, *, metadata_publisher=None):
        from sensor_msgs.msg import CompressedImage
        self.CompressedImage = CompressedImage
        self.namespace = namespace
        self.time_mapper = time_mapper
        self.publisher = publisher
        self.metadata_publisher = metadata_publisher
        self._metadata_failures = 0
        if metadata_publisher is not None:
            from std_msgs.msg import String
            self.String = String

    def publish(self, source_frames, aligned_frames, *, fallback_stamp_ns=None):
        """Publish one aligned depth frame; True when it carried capture time."""
        import numpy as np
        depth = aligned_frames.get_depth_frame()
        if depth is None:
            return False
        pixels = np.asanyarray(depth.get_data())
        if pixels.shape != (480, 640):
            raise ValueError('compressed depth requires a 640x480 pixel grid')
        mapped, frame_id = False, ''
        stamp_ns = fallback_stamp_ns
        source_color = source_frames.get_color_frame()
        source_depth = source_frames.get_depth_frame()
        if source_color is not None and source_depth is not None:
            try:
                _, stamps = self.time_mapper.sample(source_color, source_depth)
                stamp_ns, frame_id, mapped = stamps[1], f'{self.namespace}_rgb_optical', True
            except Exception:                                  # noqa: BLE001
                # Media first: a broken clock mapping must not drop a depth
                # frame the display/navi path can still use.
                mapped = False
        if stamp_ns is None:
            raise ValueError('arrival stamp required for the depth fallback')
        millimetres = np.rint(pixels.astype(np.float64) * (self.time_mapper.depth_scale * 1000.))
        millimetres[(millimetres < 1) | (millimetres > 65535) | ~np.isfinite(millimetres)] = 0
        msg = self.CompressedImage()
        msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(int(stamp_ns), 10**9)
        msg.header.frame_id = frame_id
        msg.format = '16UC1; compressedDepth zlib'
        msg.data = array('B', zlib.compress(millimetres.astype('<u2').tobytes(), 1))
        self.publisher.publish(msg)
        # Evidence is about the exact compressed frame transmitted above, not
        # its raw Z16 predecessor. No sidecar is emitted for arrival-time fallback.
        if mapped and self.metadata_publisher is not None:
            try:
                packet = depth_frame_metadata(
                    meta, int(stamp_ns), frame_id, msg.data,
                    self.time_mapper.source_generation)
                self.metadata_publisher.publish(
                    self.String(data=json.dumps(packet, allow_nan=False)))
                self._metadata_failures = 0
            except Exception as error:
                self._metadata_failures += 1
                if self._metadata_failures in (1, 100):
                    print('[camera] depth provenance publish failed: ' +
                          str(error)[:200], flush=True)
        return mapped


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
        self.time_mapper = CaptureTimeMapper(self.depth_scale)
        # Sample the enabled raw branch at a bounded rate instead of duplicating
        # every 30 Hz capture frame over DDS.
        self.min_interval_s = max(0.0, float(min_interval_s))
        self.last_publish = 0.0
        # Controlled by the visual-odometry card lifecycle: stop publishes a
        # latched Bool=False on the enable topic so this shared camera process
        # stops converting/serializing raw frames until start re-enables it.
        # Start disabled: the camera process may come up before the card's
        # retained enable sample is delivered.
        self.enabled = False

    def publish(self, source_frames, aligned_frames):
        """Faults in the optional branch never take down the media producer."""
        import numpy as np
        try:
            if not self.enabled:
                return
            source_color, source_depth = source_frames.get_color_frame(), source_frames.get_depth_frame()
            color, depth = aligned_frames.get_color_frame(), aligned_frames.get_depth_frame()
            if not all((source_color, source_depth, color, depth)):
                return
            now = time.monotonic()
            if self.min_interval_s > 0 and now - self.last_publish < self.min_interval_s:
                return
            self.last_publish = now
            meta, stamps = self.time_mapper.sample(source_color, source_depth)
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
                msg.data = array('B', pixels.tobytes())
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
            self.pubs['camera_meta'].publish(self.String(data=json.dumps(meta, allow_nan=False)))
        except Exception as exc:
            # Let the camera loop own failure counting/rotation. It samples the
            # first failure and every 100th so a 30 Hz fault cannot flood the
            # log; swallowing here made the outer throttle unreachable.
            raise
