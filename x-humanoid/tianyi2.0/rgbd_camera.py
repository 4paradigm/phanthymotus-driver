"""One calibrated depth stream shared by original sensor cards and consumers.

The vendor must already express depth in the color optical reference frame.
Different intrinsics still require reprojection: sampling color along the depth
rays also removes its distortion. The resulting two 640x480 images share K.
"""
from collections import deque
from array import array
from copy import deepcopy
import json
import os
import queue
import threading
import time
import uuid
import zlib

import numpy as np

from common.camera_mount import BodyMountSource


def make_geometry(color, depth, *, registered):
    if not registered or not color.header.frame_id or color.header.frame_id != depth.header.frame_id:
        raise ValueError('vendor-registered common optical reference is required')
    for info in (color, depth):
        k = np.asarray(info.k, float).reshape(3, 3)
        if (info.width <= 0 or info.height <= 0 or not np.isfinite(k).all() or
                k[0, 0] <= 0 or k[1, 1] <= 0 or not np.allclose(k[2], [0, 0, 1]) or
                k[0, 1] != 0 or k[1, 0] != 0):
            raise ValueError('invalid source pinhole intrinsics')
    if np.any(np.asarray(depth.d) != 0):
        raise ValueError('registered depth must have rectified rays')
    if color.distortion_model not in ('plumb_bob', 'rational_polynomial') or len(color.d) not in (5, 8):
        raise ValueError('unsupported source color distortion')
    distortion = np.zeros(8)
    distortion[:len(color.d)] = color.d
    if not np.isfinite(distortion).all():
        raise ValueError('invalid color distortion')
    width, height = depth.width, depth.height
    crop_w, crop_h = min(width, height * 4 // 3), min(height, width * 3 // 4)
    left, top = (width - crop_w) // 2, (height - crop_h) // 2
    k = np.asarray(depth.k, float).reshape(3, 3).copy()
    k[0] *= 640 / crop_w
    k[1] *= 480 / crop_h
    k[0, 2] -= left * 640 / crop_w
    k[1, 2] -= top * 480 / crop_h
    u, v = np.meshgrid(np.arange(640), np.arange(480))
    def project(matrix):
        x, y = (u - matrix[0, 2]) / matrix[0, 0], (v - matrix[1, 2]) / matrix[1, 1]
        r2 = x*x + y*y
        k1, k2, p1, p2, k3, k4, k5, k6 = distortion
        radial = (1 + k1*r2 + k2*r2**2 + k3*r2**3) / (1 + k4*r2 + k5*r2**2 + k6*r2**3)
        xd = x*radial + 2*p1*x*y + p2*(r2 + 2*x*x)
        yd = y*radial + p1*(r2 + 2*y*y) + 2*p2*x*y
        maps = dict(color_x=(color.k[0]*xd + color.k[2]).astype(np.float32),
                    color_y=(color.k[4]*yd + color.k[5]).astype(np.float32),
                    depth_x=(depth.k[0]*x + depth.k[2]).astype(np.float32),
                    depth_y=(depth.k[4]*y + depth.k[5]).astype(np.float32))
        valid = np.ones((480, 640), dtype=bool)
        for kind, camera in (('color', color), ('depth', depth)):
            mx, my = maps[kind+'_x'], maps[kind+'_y']
            valid &= (np.isfinite(mx) & np.isfinite(my) & (mx >= 0) & (mx <= camera.width-1)
                      & (my >= 0) & (my <= camera.height-1))
        return maps, valid

    maps, valid = project(k)
    if not valid.all():
        # Keep a centered 4:3 window fully inside both measured image fields.
        # A one-pixel margin protects interpolation between the sampled rays.
        radius = np.maximum(np.abs(u-320.)/320., np.abs(v-240.)/240.)
        extent = float(radius[~valid].min()) - 1./240.
        if extent <= 0:
            raise ValueError('RGB-D has no usable common central field of view')
        zoom = 1./extent
        k[:2] *= zoom
        k[0, 2] += 320.*(1.-zoom)
        k[1, 2] += 240.*(1.-zoom)
        maps, valid = project(k)
        if not valid.all():
            raise ValueError('RGB-D common field cannot be rectified without missing image rays')
    return dict(K=k, frame_id=depth.header.frame_id, **maps)


def camera_declarations(root, geometry, clock, *, color_topic=None, depth_topic=None):
    common = dict(schema='motus.camera/1', id=root + '/rectified',
                  width=640, height=480, K=geometry['K'].ravel().tolist(), D=[0.] * 5,
                  distortion_model='pinhole', source='derived-from-K',
                  vendor=dict(frame_id=geometry['frame_id'], clock_id=clock,
                              registration='registered_rectified',
                              registration_source='orbbec_hw_registered_common_optical_rays',
                              timestamp_semantics='source_capture_mapped'))
    color = dict(deepcopy(common), topic=color_topic or root + '/color', format='image/jpeg')
    depth = dict(deepcopy(common), topic=depth_topic or root + '/depth', format='image/depth-zlib')
    depth['vendor'].update(depth_unit='mm', depth_source='measured')
    return [color, depth]


def _stamp(msg):
    return msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec


class _MountNode:
    """TF stays on robot domain 0; mount packets follow images to domain 42."""
    def __init__(self, source, output):
        self.source, self.output = source, output

    def __getattr__(self, name):
        return getattr(self.source, name)

    def create_publisher(self, *args, **kwargs):
        return self.output.create_publisher(*args, **kwargs)

    def destroy_publisher(self, publisher):
        return self.output.destroy_publisher(publisher)


class RgbdCameraPlugin:
    def __init__(self, config, namespace, ros2, *, managed=False):
        import rclpy
        from rclpy.context import Context
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.node import Node
        self.root = '/' + namespace + '/camera/rgbd'
        self.managed, self.clients = managed, {}
        self.client_epochs = {}
        self.lifecycle_lock = threading.RLock()
        self.frame_interval_ns = round(1_000_000_000 / max(1., min(float(config.get('hz', 10)), 15.)))
        self.clock = 'orbbec_head_global_ros_header'
        # A separate participant prevents body traffic from starving camera
        # receive queues. It uses the same vendor profile and robot domain 0.
        self._body_context = ros2.ctx_tianyi
        self._source_context = Context()
        rclpy.init(context=self._source_context, domain_id=0)
        self.sub_node = Node('tianyi_rgbd_source', context=self._source_context)
        self.pub_node = Node('tianyi_rgbd_output', context=ros2.ctx_core)
        ros2.executor_core.add_node(self.pub_node)
        self.lock = threading.RLock()
        self.imu_lock = threading.RLock()
        self.running = self.registered = self.attached = False
        self.geometry = None
        self.infos, self.signature = {}, None
        self.frames = {'color': deque(maxlen=3), 'depth': deque(maxlen=3)}
        self.pending = queue.Queue(maxsize=1)
        self.worker = None
        self.stop_event = threading.Event()
        self.last_emit = 0
        self.count = 0
        self.image_streams = {kind: dict(received=0, last_capture_ns=None, last_arrival_ns=None)
                              for kind in ('color', 'depth')}
        self.last_publish_ns = None
        self.error = None
        self.future = None
        self.parameter_stage = 'service'
        self.parameter_values = {}
        self.imu_enabled = config.get('imu_enabled', True)
        self.imu_attached = False
        self.imu_reason = 'waiting for camera motion samples and TF'
        self.imu_samples = {kind: deque(maxlen=512) for kind in ('gyro', 'accel')}
        self.imu_epoch = None
        self.imu_last_stamp = 0
        self.imu_publication_attempts = self.imu_max_capture_gap_ns = 0
        self.imu_translation_reason = 'waiting for camera motion TF translation'
        self.body_mount = None
        self._camera_executor = None
        self.imu_node = None
        self._source_executor = SingleThreadedExecutor(context=self._source_context)
        self._source_shutdown = threading.Event()
        self._source_executor.add_node(self.sub_node)
        self._source_context.on_shutdown(self._shutdown_source)
        self._body_context.on_shutdown(self._source_context.try_shutdown)
        self._source_spin = threading.Thread(target=self._spin_source, name='tianyi_rgbd_images', daemon=True)
        self._source_spin.start()
        if self.imu_enabled:
            # Large image fragments must not queue ahead of camera motion.
            self._camera_context = Context()
            rclpy.init(context=self._camera_context, domain_id=0)
            self._source_context.on_shutdown(self._camera_context.try_shutdown)
            self.imu_node = Node('tianyi_rgbd_imu_source', context=self._camera_context)
            self._camera_executor = SingleThreadedExecutor(context=self._camera_context)
            self._camera_shutdown = threading.Event()
            self._camera_executor.add_node(self.imu_node)
            self._camera_spin = threading.Thread(target=self._spin_camera, name='tianyi_rgbd_ros', daemon=True)
            self._camera_spin.start()
            self._camera_context.on_shutdown(self._shutdown_camera)

    def _spin_source(self):
        from rclpy.executors import ExternalShutdownException, ShutdownException
        try:
            while not self._source_shutdown.is_set():
                self._source_executor.spin_once(timeout_sec=.05)
        except (ExternalShutdownException, ShutdownException):
            pass
        except Exception as error:
            if not self._source_shutdown.is_set():
                with self.lock, self.imu_lock:
                    self._source_shutdown.set()
                    self.error = 'camera source callbacks: ' + str(error)
                    self.running = False
                    self.stop_event.set()
        finally:
            self._source_shutdown.set()
            self._source_executor.shutdown(timeout_sec=5)

    def _shutdown_source(self):
        self._source_shutdown.set()
        self._source_executor.wake()

    def _spin_camera(self):
        from rclpy.executors import ExternalShutdownException, ShutdownException
        try:
            while not self._camera_shutdown.is_set():
                self._camera_executor.spin_once(timeout_sec=.05)
        except (ExternalShutdownException, ShutdownException):
            pass
        except Exception as error:
            if not self._camera_shutdown.is_set():
                with self.lock, self.imu_lock:
                    self._camera_shutdown.set()
                    self.error = 'camera callbacks: ' + str(error)
                    self.running = False
                    self.stop_event.set()
        finally:
            self._camera_shutdown.set()
            self._camera_executor.shutdown(timeout_sec=5)

    def _shutdown_camera(self):
        # Context shutdown may hold its lock; let the spin thread clean up.
        self._camera_shutdown.set()
        self._camera_executor.wake()

    @property
    def color_topic(self):
        return self.root.rsplit('/camera/rgbd', 1)[0] + '/camera/head'

    @property
    def depth_topic(self):
        return self.root.rsplit('/camera/rgbd', 1)[0] + '/camera/head/depth'

    def acquire(self, kind):
        with self.lifecycle_lock:
            with self.lock, self.imu_lock:
                if kind in self.clients and self.running:
                    return
                self.clients[kind] = self.sub_node.get_clock().now().nanoseconds
                self.client_epochs[kind] = uuid.uuid4().hex
                if kind == 'imu':
                    self.imu_epoch = uuid.uuid4().hex
                    self.imu_last_stamp = 0
                    for samples in self.imu_samples.values():
                        samples.clear()
            try:
                self.start()
            except Exception:
                with self.lock:
                    self.clients.pop(kind, None)
                raise

    def release(self, kind):
        with self.lifecycle_lock:
            with self.lock, self.imu_lock:
                self.clients.pop(kind, None)
                last = not self.clients
            if last:
                self.stop()

    def _publishes(self, kind, stamp, epochs=None):
        return not self.managed or (kind in self.clients and stamp >= self.clients[kind] and
            (epochs is None or epochs.get(kind) == self.client_epochs[kind]))

    def get_tool(self):
        return {'name': 'camera_rgbd', 'type': 'sensor',
                'description': '天轶配准 RGB-D 计算输入（原始采集时间、共同光学平面）',
                'inputSchema': {'type': 'object', 'properties': {}},
                'topic_out': self._outputs(), 'camera_info': self._declarations(),
                'source_status': self._source_status()}

    def _outputs(self):
        return [{'topic': self.color_topic, 'format': 'image/jpeg'},
                {'topic': self.depth_topic, 'format': 'image/depth-zlib'},
                {'topic': self.root + '/imu', 'format': 'sensor/imu'},
                {'topic': self.root + '/body_mount', 'format': 'state/extrinsic'}]

    def _declarations(self):
        declarations = camera_declarations(self.root, self.geometry, self.clock,
            color_topic=self.color_topic, depth_topic=self.depth_topic) if self.geometry is not None else []
        if getattr(self, 'body_mount', None) is not None:
            for entry in declarations:
                entry['vendor']['body_mount'] = self.body_mount.declaration()
        return declarations

    def _body_mount_status(self):
        if getattr(self, 'body_mount', None) is not None:
            return self.body_mount.status()
        return dict(status='unavailable', reason='body mount source not started', accuracy_validated=False)

    def _source_status(self):
        """Bounded current-start evidence; no image payloads or source messages."""
        with self.lock:
            parameters = dict(stage='validated' if self.registered else
                              getattr(self, 'parameter_stage', 'service'),
                              **getattr(self, 'parameter_values', {}))
            infos = {kind: kind in getattr(self, 'infos', {}) for kind in ('color', 'depth')}
            if not self.running:
                stage, reason = 'stopped', self.error or 'RGB-D source stopped'
            elif not self.registered:
                stage = 'parameter_' + parameters['stage']
                reason = self.error or {
                    'service': 'waiting for /ob_camera_head/ob_camera_head/get_parameters service',
                    'request': 'waiting to request source registration and global capture clock parameters',
                    'response': 'waiting for source registration and global capture clock parameter response',
                    'validation': 'waiting for valid registered depth and source global capture clock parameters',
                }[parameters['stage']]
            elif self.geometry is None:
                missing = [kind for kind, received in infos.items() if not received]
                stage = 'camera_info' if missing else 'rectified_geometry'
                reason = self.error or ('waiting for ' + ' and '.join(missing) + ' CameraInfo' if missing else
                                       'waiting for source images to validate rectified RGB-D geometry')
            elif not self.count or self.error:
                stage, reason = 'first_calibrated_output', self.error or 'waiting for first calibrated RGB-D output'
            else:
                stage, reason = 'ready', None
            return dict(state='available' if stage == 'ready' else 'unavailable', stage=stage,
                        **({'reason': reason[:256]} if reason else {}), parameters=parameters,
                        camera_info=infos, rectified_geometry_ready=self.geometry is not None,
                        first_calibrated_output=self.count > 0,
                        streams=dict({kind: dict(value) for kind, value in getattr(self, 'image_streams', {}).items()},
                                     paired_capture_ns=getattr(self, 'last_emit', 0), forwarded=self.count,
                                     last_publish_ns=getattr(self, 'last_publish_ns', None)))

    def start(self):
        from sensor_msgs.msg import Image, CameraInfo, CompressedImage
        from rclpy.qos import qos_profile_sensor_data
        from rcl_interfaces.srv import GetParameters
        with self.lock, self.imu_lock:
            if self._source_shutdown.is_set():
                raise RuntimeError(self.error or 'camera source callback executor stopped; restart the driver')
            if self._camera_executor is not None and self._camera_shutdown.is_set():
                raise RuntimeError(self.error or 'camera callback executor stopped; restart the driver')
            if self.running:
                return
            if not self.attached:
                self.color_pub = self.pub_node.create_publisher(CompressedImage, self.color_topic, qos_profile_sensor_data)
                self.depth_pub = self.pub_node.create_publisher(CompressedImage, self.depth_topic, qos_profile_sensor_data)
                for kind in ('color', 'depth'):
                    self.sub_node.create_subscription(CameraInfo, '/ob_camera_head/' + kind + '/camera_info',
                        lambda m, k=kind: self._info(k, m), qos_profile_sensor_data)
                    self.sub_node.create_subscription(Image, '/ob_camera_head/' + kind + '/image_raw',
                        lambda m, k=kind: self._image(k, m), qos_profile_sensor_data)
                self.client = self.sub_node.create_client(GetParameters, '/ob_camera_head/ob_camera_head/get_parameters')
                self.sub_node.create_timer(.5, self._parameters)
                self.attached = True
            self.running = True
            self.registered = False
            self.geometry = None
            self.signature = None
            self.infos.clear()
            self.last_emit = 0
            self.count = 0
            self.image_streams = {kind: dict(received=0, last_capture_ns=None, last_arrival_ns=None)
                                  for kind in ('color', 'depth')}
            self.last_publish_ns = None
            self.future = None
            self.parameter_stage = 'service'
            self.parameter_values = {}
            self.error = None
            self.imu_epoch = uuid.uuid4().hex
            self.imu_last_stamp = 0
            self.imu_publication_attempts = self.imu_max_capture_gap_ns = 0
            self.imu_translation_reason = 'waiting for camera motion TF translation'
            for samples in self.imu_samples.values():
                samples.clear()
            self.imu_reason = 'waiting for camera motion samples and TF'
            self._attach_imu()
            if self.body_mount is not None:
                self.body_mount.close()
            self.body_mount = BodyMountSource(_MountNode(self.sub_node, self.pub_node), self.root,
                body_frame=os.environ.get('CAMERA_BODY_FRAME', 'base_link'),
                camera_frame=lambda: self.geometry['frame_id'] if self.geometry is not None else None,
                clock_id=lambda: self.clock)
            self.stop_event.clear()
            self.worker = threading.Thread(target=self._work, name='tianyi_rgbd_source', daemon=True)
            self.worker.start()

    def _attach_imu(self):
        if not self.imu_enabled or self.imu_attached:
            return
        try:
            from sensor_msgs.msg import Imu
            from std_msgs.msg import String
            from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
            from tf2_ros import Buffer, TransformListener
            self.imu_buffer = Buffer()
            self.imu_listener = TransformListener(self.imu_buffer, self.imu_node)
            self.imu_pub = self.pub_node.create_publisher(String, self.root + '/imu', qos_profile_sensor_data)
            # Keep 200 ms of 100 Hz history across short callback stalls. Deeper
            # queues accumulate stale captures under mixed sensor load instead
            # of catching up. Admission still rejects captures over 500 ms old.
            motion_qos = QoSProfile(depth=20, reliability=ReliabilityPolicy.BEST_EFFORT)
            for kind in ('gyro', 'accel'):
                self.imu_node.create_subscription(Imu, '/ob_camera_head/' + kind + '/sample',
                    lambda msg, k=kind: self._imu(k, msg), motion_qos)
            self.imu_attached = True
        except Exception as error:
            self.imu_reason = 'camera motion unavailable: ' + str(error)

    def _imu_extrinsics(self, msg):
        from rclpy.time import Time
        frame = msg.header.frame_id
        if not frame:
            raise ValueError('camera motion frame is missing')
        # Each motion stream has its own real frame and capture timestamp.
        transform = self.imu_buffer.lookup_transform(
            self.geometry['frame_id'], frame, Time.from_msg(msg.header.stamp))
        q = transform.transform.rotation
        values = np.array([q.x, q.y, q.z, q.w], dtype=float)
        if not np.isfinite(values).all() or not np.isclose(values @ values, 1., atol=1e-5):
            raise ValueError('camera motion TF quaternion is not a unit rotation')
        values /= np.linalg.norm(values)
        x, y, z, w = values
        rotation = [[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]]
        try:
            t = transform.transform.translation
            translation = [float(t.x), float(t.y), float(t.z)]
            if not np.isfinite(translation).all():
                raise ValueError('nonfinite camera motion TF translation')
        except (AttributeError, TypeError, ValueError):
            translation = None
        return rotation, translation

    def _imu(self, kind, msg):
        # Image callbacks/publication can stall; motion has its own executor
        # and lock. Lifecycle changes acquire source lock then this lock.
        with self.imu_lock:
            if not self.running or not self.imu_enabled:
                return
            try:
                if not self.registered:
                    raise ValueError('waiting for source global capture clock confirmation')
                if self.geometry is None:
                    raise ValueError('waiting for RGB-D camera optical calibration')
                stamp = _stamp(msg)
                if not self._publishes('imu', stamp):
                    return
                now = self.sub_node.get_clock().now().nanoseconds
                if stamp <= 0 or not -50_000_000 <= now-stamp <= 500_000_000:
                    raise ValueError('stale, future or invalid camera motion capture stamp')
                if not msg.header.frame_id:
                    raise ValueError('camera motion frame is missing')
                vector = getattr(msg, 'angular_velocity' if kind == 'gyro' else 'linear_acceleration')
                if not np.isfinite([vector.x, vector.y, vector.z]).all():
                    raise ValueError('nonfinite camera motion measurement')
                if kind == 'gyro' and stamp <= self.imu_last_stamp:
                    raise ValueError('nonmonotonic camera gyro capture stamp')
                self.imu_samples[kind].append(msg)
                gyro_samples, accel_samples = self.imu_samples['gyro'], self.imu_samples['accel']
                while gyro_samples and accel_samples:
                    gyro = gyro_samples[0]
                    gyro_stamp = _stamp(gyro)
                    while len(accel_samples) > 1 and _stamp(accel_samples[1]) <= gyro_stamp:
                        accel_samples.popleft()
                    accel = min(accel_samples, key=lambda sample: abs(_stamp(sample)-gyro_stamp))
                    accel_stamp = _stamp(accel)
                    if abs(gyro_stamp-accel_stamp) > 20_000_000:
                        self.imu_reason = 'camera gyro/accel capture skew exceeds 20ms'
                        if _stamp(accel_samples[-1]) > gyro_stamp:
                            gyro_samples.popleft()
                            continue
                        break
                    gyro_samples.popleft()
                    if (gyro_stamp <= self.imu_last_stamp or
                            any(not -50_000_000 <= now-s <= 500_000_000 for s in (gyro_stamp, accel_stamp))):
                        raise ValueError('stale or nonmonotonic camera motion capture stamp')
                    if self.imu_last_stamp and gyro_stamp-self.imu_last_stamp > 500_000_000:
                        self.imu_epoch = uuid.uuid4().hex
                    gyro_rotation, gyro_translation = self._imu_extrinsics(gyro)
                    accel_rotation, accel_translation = self._imu_extrinsics(accel)
                    packet = dict(schema='motus.imu/1', source_id='/ob_camera_head/gyro+accel',
                        epoch=self.imu_epoch, stamp_ns=gyro_stamp, accel_stamp_ns=accel_stamp,
                        stamp_source='measurement', clock_id=self.clock,
                        camera_frame_id=self.geometry['frame_id'], gyro_frame_id=gyro.header.frame_id,
                        accel_frame_id=accel.header.frame_id,
                        camera_from_gyro=gyro_rotation, camera_from_accel=accel_rotation,
                        rotation_source='tf', units=dict(angular_velocity='rad/s', linear_acceleration='m/s^2'),
                        angular_velocity=[gyro.angular_velocity.x, gyro.angular_velocity.y, gyro.angular_velocity.z],
                        linear_acceleration=[accel.linear_acceleration.x, accel.linear_acceleration.y, accel.linear_acceleration.z])
                    self.imu_translation_reason = 'camera motion TF translation missing or invalid'
                    if gyro_translation is not None and accel_translation is not None:
                        packet.update(camera_from_gyro_translation=gyro_translation,
                                      camera_from_accel_translation=accel_translation, translation_source='tf')
                        self.imu_translation_reason = None
                    now = self.sub_node.get_clock().now().nanoseconds
                    if any(not -50_000_000 <= now-s <= 500_000_000 for s in (gyro_stamp, accel_stamp)):
                        raise ValueError('camera motion capture became stale during TF processing')
                    from std_msgs.msg import String
                    if self._publishes('imu', gyro_stamp):
                        self.imu_pub.publish(String(data=json.dumps(packet, allow_nan=False)))
                        self.imu_publication_attempts += 1
                        if self.imu_last_stamp:
                            self.imu_max_capture_gap_ns = max(self.imu_max_capture_gap_ns, gyro_stamp-self.imu_last_stamp)
                    self.imu_last_stamp = gyro_stamp
                    self.imu_reason = None
            except Exception as error:
                self.imu_reason = 'camera motion unavailable: ' + str(error)

    def _imu_status(self):
        with self.imu_lock:
            if not getattr(self, 'imu_enabled', True):
                return dict(state='disabled', reason='camera motion source disabled',
                            fusion=dict(state='unavailable', reason='camera motion source disabled'))
            reason = getattr(self, 'imu_reason', 'waiting for camera motion samples and TF')
            if not self.running:
                reason = 'camera motion source stopped'
            elif getattr(self, 'imu_last_stamp', 0) and self.sub_node.get_clock().now().nanoseconds-self.imu_last_stamp > 500_000_000:
                reason = 'camera motion source stale'
            translation_reason = getattr(self, 'imu_translation_reason', 'waiting for camera motion TF translation')
            return dict(state='unavailable' if reason else 'available', **({'reason': reason} if reason else {}),
                        publication_attempts=getattr(self, 'imu_publication_attempts', 0),
                        max_capture_gap_ns=getattr(self, 'imu_max_capture_gap_ns', 0),
                        fusion=dict(state='unavailable' if translation_reason else 'available',
                                    **({'reason': translation_reason} if translation_reason else {})))

    def _parameters(self):
        from rcl_interfaces.srv import GetParameters
        with self.lock:
            if not self.running or self.registered:
                return
            if self.future is None:
                if self.client.service_is_ready():
                    self.parameter_stage = 'request'
                    request = GetParameters.Request()
                    request.names = ['depth_registration', 'time_domain', 'enable_sync_host_time']
                    try:
                        self.future = self.client.call_async(request)
                        self.parameter_stage = 'response'
                        self.error = None
                    except Exception as error:
                        self.error = ('source parameter request failed: ' + str(error))[:256]
                else:
                    self.parameter_stage = 'service'
                    self.error = None
            elif self.future.done():
                try:
                    values = self.future.result().values
                    self.parameter_values = dict(depth_registration=values[0].bool_value,
                        time_domain=values[1].string_value[:64], enable_sync_host_time=values[2].bool_value)
                    self.parameter_stage = 'validation'
                    self.registered = (values[0].bool_value and values[1].string_value == 'global' and values[2].bool_value)
                    self.error = (None if self.registered else
                                  'Orbbec requires registered depth and source global capture time')
                except Exception as error:
                    self.error = ('source parameter response failed: ' + str(error))[:256]
                self.future = None

    def _info(self, kind, msg):
        with self.lock:
            if self.running:
                self.infos[kind] = msg

    def _image(self, kind, msg):
        with self.lock:
            if not self.running:
                return
            stream = self.image_streams[kind]
            stream.update(received=stream['received']+1, last_capture_ns=_stamp(msg), last_arrival_ns=time.time_ns())
            if not self.registered or set(self.infos) != {'color', 'depth'}:
                return
            try:
                signature = tuple((tuple(self.infos[k].k), tuple(self.infos[k].d), self.infos[k].width,
                                   self.infos[k].height, self.infos[k].header.frame_id) for k in ('color', 'depth'))
                if signature != self.signature:
                    if self.signature is not None:
                        raise ValueError('source calibration changed; restart RGB-D source')
                    self.geometry = make_geometry(self.infos['color'], self.infos['depth'], registered=self.registered)
                    self.signature = signature
                if (msg.header.frame_id != self.geometry['frame_id'] or
                        msg.width != self.infos[kind].width or msg.height != self.infos[kind].height):
                    raise ValueError('source image geometry differs from calibration')
                if self.managed and not self.clients.keys() & {'color', 'depth'}:
                    return
                self.frames[kind].append(msg)
                color, depth = self.frames['color'], self.frames['depth']
                while color and depth:
                    delta = _stamp(color[0]) - _stamp(depth[0])
                    if abs(delta) <= 20_000_000:
                        pair = color.popleft(), depth.popleft()
                        if _stamp(pair[1]) - self.last_emit >= self.frame_interval_ns:
                            if self.pending.full():
                                self.pending.get_nowait()
                            self.pending.put_nowait(pair + (dict(self.client_epochs),) if self.managed else pair)
                            self.last_emit = _stamp(pair[1])
                        break
                    (color if delta < 0 else depth).popleft()
            except (ValueError, TypeError) as error:
                with self.imu_lock:
                    self.error = str(error)
                    self.running = False
                    self.stop_event.set()

    def _work(self):
        import cv2
        from sensor_msgs.msg import CompressedImage
        while not self.stop_event.is_set():
            try:
                item = self.pending.get(timeout=.1)
                color, depth = item[:2]
                epochs = item[2] if len(item) == 3 else None
            except queue.Empty:
                continue
            try:
                if color.encoding not in ('rgb8', 'bgr8') or depth.encoding != '16UC1' or depth.is_bigendian:
                    raise ValueError('unsupported source RGB-D encoding')
                rgb = np.frombuffer(color.data, np.uint8).reshape(color.height, color.step)[:, :color.width*3].reshape(color.height, color.width, 3)
                if color.encoding == 'rgb8':
                    rgb = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
                z16 = np.frombuffer(depth.data, np.uint8).reshape(depth.height, depth.step)[:, :depth.width*2].copy().view('<u2').reshape(depth.height, depth.width)
                g = self.geometry
                rectified = cv2.remap(rgb, g['color_x'], g['color_y'], cv2.INTER_LINEAR)
                metric = cv2.remap(z16, g['depth_x'], g['depth_y'], cv2.INTER_NEAREST)
                ok, jpg = cv2.imencode('.jpg', rectified, [cv2.IMWRITE_JPEG_QUALITY, 90])
                if not ok:
                    raise ValueError('RGB-D color encoding failed')
                out_color, out_depth = CompressedImage(), CompressedImage()
                out_color.header, out_depth.header = deepcopy(color.header), deepcopy(depth.header)
                out_color.format, out_color.data = 'jpeg', array('B', jpg.tobytes())
                out_depth.format = '16UC1; compressedDepth zlib'
                out_depth.data = array('B', zlib.compress(metric.astype('<u2', copy=False).tobytes(), 1))
                with self.lock:
                    if not self.running or self.stop_event.is_set():
                        continue
                    publish_color = self._publishes('color', _stamp(color), epochs)
                    publish_depth = self._publishes('depth', _stamp(depth), epochs)
                    if not (publish_color or publish_depth):
                        continue
                    self.body_mount.publish(_stamp(depth))
                    if publish_color:
                        self.color_pub.publish(out_color)
                    if publish_depth:
                        self.depth_pub.publish(out_depth)
                    self.count += 1
                    self.last_publish_ns = time.time_ns()
            except Exception as error:
                with self.lock, self.imu_lock:
                    self.error = str(error)
                    self.running = False
                    self.stop_event.set()

    def stop(self):
        with self.lock, self.imu_lock:
            self.running = False
            self.stop_event.set()
        if self.worker is not None:
            self.worker.join(timeout=5)
            if self.worker.is_alive():
                raise RuntimeError('RGB-D source worker did not stop')
        with self.lock, self.imu_lock:
            if self.body_mount is not None:
                self.body_mount.close()
            for samples in self.imu_samples.values():
                samples.clear()
            for frames in self.frames.values():
                frames.clear()
            while not self.pending.empty():
                self.pending.get_nowait()

    def dispatch(self, action, args):
        if action == 'start':
            self.start()
            deadline = time.monotonic() + 10.
            while time.monotonic() < deadline:
                with self.lock:
                    if not self.running or (self.geometry is not None and self.count > 0 and not self.error):
                        break
                time.sleep(.05)
        elif action == 'stop':
            self.stop()
        with self.lock:
            status = self._source_status()
            startup_error = ('RGB-D startup incomplete: ' + status['reason']
                             if action == 'start' and status['state'] != 'available' else None)
            state = ('running' if status['state'] == 'available' else 'unavailable') if self.running else 'idle'
            return {'state': 'error' if startup_error else state,
                    'source_status': status, **({'reason': status['reason']} if 'reason' in status else {}),
                    'topic_out': self._outputs(),
                    'camera_info': self._declarations(), 'frames_forwarded': self.count,
                    'registered_source': self.registered,
                    'imu_status': self._imu_status(),
                    'body_mount_status': self._body_mount_status(),
                    **({'error': startup_error or self.error} if startup_error or self.error else {})}


class RgbdInputCard:
    """Original sensor card, with one shared calibrated camera owner."""
    def __init__(self, source, kind, display=None):
        self.source, self.kind, self.display = source, kind, display
        self.running = False
        self.lock = threading.RLock()

    def get_tool(self):
        if self.display is not None:
            tool = deepcopy(self.display.get_tool())
        else:
            tool = dict(name='camera_depth', type='sensor', inputSchema={'type': 'object', 'properties': {}},
                        description='天轶配准毫米深度，显示、里程计和 Point Cloud 共用', topic_out=[])
        port = next(p for p in self.source._outputs() if p['format'] ==
                    {'color': 'image/jpeg', 'depth': 'image/depth-zlib', 'imu': 'sensor/imu'}[self.kind])
        port = dict(port, description='相机 IMU（里程计输入）' if self.kind == 'imu' else '配准相机数据')
        if self.kind == 'imu':
            port['name'] = 'camera_imu'
        if self.kind == 'color':
            tool['topic_out'] = [port]
        else:
            tool['topic_out'].append(port)
        with self.source.lock:
            tool['camera_info'] = [d for d in self.source._declarations() if d['topic'] == port['topic']]
            tool['imu_status'] = self.source._imu_status()
            tool['body_mount_status'] = self.source._body_mount_status()
            tool['source_status'] = self.source._source_status()
        return tool

    def start(self):
        with self.lock:
            if self.running:
                return
            if self.display is not None:
                self.display.start()
            try:
                self.source.acquire(self.kind)
            except Exception:
                if self.display is not None:
                    self.display.stop()
                raise
            self.running = True

    def stop(self):
        with self.lock:
            if not self.running:
                return
            self.source.release(self.kind)
            if self.display is not None:
                self.display.stop()
            self.running = False

    def dispatch(self, action, args):
        if action == 'start':
            self.start()
            deadline = time.monotonic() + 10.
            while time.monotonic() < deadline:
                with self.source.lock:
                    ready = (self.source._imu_status()['state'] == 'available' if self.kind == 'imu' else
                             self.source.registered and self.source.geometry is not None
                             and self.source.count > 0 and not self.source.error)
                    if ready or not self.source.running:
                        break
                time.sleep(.05)
        elif action == 'stop':
            self.stop()
        elif action != 'info' and self.display is not None:
            return self.display.dispatch(action, args)
        with self.source.lock:
            tool = self.get_tool()
            ready = (self.source.running and self.source.registered and self.source.geometry is not None
                     and self.source.count > 0 and not self.source.error)
            state = 'running' if self.running and self.source.running else 'idle'
            startup_error = None
            if self.kind in ('color', 'depth') and self.running and not ready:
                state = 'error' if action == 'start' else 'unavailable'
                if action == 'start':
                    startup_error = 'RGB-D startup incomplete: ' + tool['source_status']['reason']
            return dict(state=state,
                        topic_out=tool['topic_out'], camera_info=tool['camera_info'],
                        source_status=tool['source_status'],
                        imu_status=tool['imu_status'], body_mount_status=tool['body_mount_status'],
                        **({'reason': tool['source_status']['reason']}
                           if self.kind in ('color', 'depth') and self.running and not ready else {}),
                        **({'error': startup_error or self.source.error}
                           if startup_error or self.source.error else {}))
