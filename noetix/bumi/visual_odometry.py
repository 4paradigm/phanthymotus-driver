"""Single-container RGB-D visual odometry for the Bumi bundle.

RTAB-Map ``rgbd_odometry`` runs as one supervised child of the bundle process.
The bundle's camera subprocess remains the only RealSense owner and publishes an
optional raw RGB-D branch on ``/{namespace}/odometry_input/*``; this module
consumes it, validates/undistorts frame pairs, launches the estimator and
publishes a read-only ``motus.odom/1`` sample on ``/bumi_visual_odometry/state/odom``.

There is no IMU input, no robot_localization EKF, no visual-inertial fusion and
no consumer in the motion path: stopping this card only reaps its own child and
private topics, never the camera or a motion card.
"""
from __future__ import annotations
from collections import OrderedDict
from copy import deepcopy
import json
import math
import os
from pathlib import Path
import re
import signal
import subprocess
import tempfile
import threading
import time
import uuid

try:  # imported as part of the noetix.bumi namespace in tests
    from .visual_odometry_contract import (CameraCalibration, Health, MAX_INPUT_AGE_NS,
                                           MAX_POSITION_SIGMA_M, MAX_SKEW_NS,
                                           MAX_TRANSPORT_NS, finite, make_sample,
                                           quat_matrix)
except ImportError:  # imported as a top-level /work module inside the bundle image
    from visual_odometry_contract import (CameraCalibration, Health, MAX_INPUT_AGE_NS,
                                          MAX_POSITION_SIGMA_M, MAX_SKEW_NS,
                                          MAX_TRANSPORT_NS, finite, make_sample,
                                          quat_matrix)

PACKAGE = 'rtabmap_odom'
EXECUTABLE = 'rgbd_odometry'
OUTPUT_TOPIC = '/bumi_visual_odometry/state/odom'


# ── Estimator launch contract ────────────────────────────────────────────────

def parameter_file(config):
    """Render the single-node ROS 2 params file without a YAML dependency.

    The bundle image ships PyYAML for its own config, so this is not strictly a
    dependency-saving measure here; keeping the renderer local means the
    visual-odometry path cannot be broken by a YAML API change and makes the
    written file explicit. Values that YAML would read as numbers/bools (e.g.
    the string "0" for ``Odom/ResetCountdown``) are quoted so the parameter type
    stays String.
    """
    plain = re.compile(r'[A-Za-z_][A-Za-z0-9_./-]*')
    numeric = re.compile(r'[-+]?(?:\d+\.?\d*|\.\d+)(?:[eE][-+]?\d+)?')
    reserved = {'true', 'false', 'null', 'yes', 'no', 'on', 'off', '~'}

    def scalar(value):
        if isinstance(value, bool):
            return 'true' if value else 'false'
        if isinstance(value, (int, float)):
            return repr(value)
        text = str(value)
        if (plain.fullmatch(text) and text.lower() not in reserved
                and not numeric.fullmatch(text)):
            return text
        return "'" + text.replace("'", "''") + "'"

    lines = ["'/**':", '  ros__parameters:']
    for key, value in config.items():
        lines.append('    '+scalar(key)+': '+scalar(value))
    return '\n'.join(lines)+'\n'


def parameters(segment, frame_id):
    """Parameters for the bundle-local visual estimator.

    ``frame_id`` is the segment's base frame, which coincides with the optical
    frame when no camera mount is measured. A static base->optical transform is
    published before the estimator starts; RTAB-Map looks that up per frame.
    """
    return dict(frame_id=frame_id, odom_frame_id=segment+'_odom', publish_tf=False,
                wait_imu_to_init=False, approx_sync=True, approx_sync_max_interval=.02,
                qos=2, qos_camera_info=2, topic_queue_size=5, sync_queue_size=5,
                **{'Odom/ResetCountdown': '0'})


def stop_group(proc):
    for sig, timeout in ((signal.SIGINT, 3), (signal.SIGTERM, 2), (signal.SIGKILL, 2)):
        try:
            os.killpg(proc.pid, sig)
        except ProcessLookupError:
            proc.wait(timeout=2)
            return
        try:
            proc.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            continue
        # Reap remaining descendants even if the ROS executable leader died.
        try:
            os.killpg(proc.pid, 0)
        except ProcessLookupError:
            return
    raise RuntimeError('visual odometry process group did not exit')


class ChildProcesses:
    """The one RTAB-Map child; stopping it never touches the camera process."""

    def __init__(self, directory, *, popen=None, stop=None, prefix=None):
        self.directory = Path(directory)
        self.popen, self.reap = popen or subprocess.Popen, stop or stop_group
        self.prefix = prefix
        self.children = []

    def start(self, segment, frame_id):
        if self.children:
            raise RuntimeError('old visual odometry process still present')
        if self.prefix is None:
            from ament_index_python.packages import get_package_prefix
            self.prefix = get_package_prefix
        config = parameters(segment, frame_id)
        root = '/'+segment
        path = self.directory/'odometry.yaml'
        try:
            path.write_text(parameter_file(config))
            args = [str(Path(self.prefix(PACKAGE))/'lib'/PACKAGE/EXECUTABLE), '--ros-args',
                    '-r', '__node:=rgbd_odometry', '-r', '__ns:='+root,
                    '--params-file', str(path)]
            for source, target in (('rgb/image', root+'/rgb'),
                                   ('depth/image', root+'/depth'),
                                   ('rgb/camera_info', root+'/info'),
                                   ('odom', root+'/visual'),
                                   ('odom_info', root+'/diagnostic'),
                                   # No IMU is fused; keep an unused remap so an
                                   # implicit subscription cannot pick a live topic.
                                   ('imu', root+'/unused_imu')):
                args.extend(['-r', source+':='+target])
            env = dict(os.environ, OMP_NUM_THREADS='2', OPENBLAS_NUM_THREADS='1')
            self.children.append(self.popen(args, env=env, start_new_session=True))
        except Exception:
            self.stop()
            raise

    def stop(self):
        failures = []
        for proc in reversed(self.children):
            try:
                self.reap(proc)
            except Exception as exc:
                failures.append(exc)
        if failures:
            raise RuntimeError('cannot reap visual odometry children: '+str(failures))
        self.children.clear()

    def failed(self):
        return any(p.poll() is not None for p in self.children)


# ── Pose contract ────────────────────────────────────────────────────────────

def mount_transform(calibration):
    """Static base->optical TF values.

    No measured mount is an explicit identity, not a zero-extrinsic guess: the
    base frame then coincides with the optical frame, so the pose stays a camera
    pose and ``pose_reference`` says ``rgb_optical``. A measured mount only
    changes this TF; the estimator path is unchanged.
    """
    if calibration is None:
        return [0., 0., 0.], [0., 0., 0., 1.]
    mount = calibration.data['camera_to_body']
    return [float(v) for v in mount['translation']], [float(v) for v in mount['quaternion']]


def estimate(msg, segment, frame_id):
    """Parse one RTAB-Map odometry message; refuse broken poses and covariance.

    RTAB-Map may publish an all-zero/NaN pose before it has registered a frame.
    The caller ignores those until the first valid estimate exists.
    """
    import numpy as np
    if msg.header.frame_id != segment+'_odom':
        raise ValueError('wrong visual odometry odom frame')
    if msg.child_frame_id not in ('', frame_id):
        raise ValueError('wrong visual odometry child frame')
    p, q = msg.pose.pose.position, msg.pose.pose.orientation
    position = finite([p.x, p.y, p.z], 3)
    quaternion = finite([q.x, q.y, q.z, q.w], 4)
    quat_matrix(quaternion)
    covariance = np.asarray(finite(list(msg.pose.covariance), 36)).reshape(6, 6)
    if (not np.allclose(covariance, covariance.T, atol=1e-8)
            or np.min(np.diag(covariance)) < -1e-9):
        raise ValueError('invalid visual pose covariance')
    if max(covariance[i, i] for i in range(3)) > MAX_POSITION_SIGMA_M**2:
        raise ValueError('visual position uncertainty exceeds 0.2 m standard deviation')
    return dict(stamp_ns=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec,
                pose=dict(position=position, quaternion=quaternion,
                          frame_id=segment+'_odom', child_frame_id=frame_id,
                          covariance=list(msg.pose.covariance)))


# ── Runtime ──────────────────────────────────────────────────────────────────

class VisualOdometryNode:
    """Quality state machine and process supervisor for one visual estimator."""

    def __init__(self, namespace, config=None):
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from std_msgs.msg import String
        self.namespace = namespace
        self.node = Node('bumi_visual_odometry')
        self.qos = qos_profile_sensor_data
        self.String = String
        self.lock = threading.RLock()
        self.config = dict(config or {})
        self.active = False
        self.closed = False
        self.output = OUTPUT_TOPIC
        self.publisher = self.node.create_publisher(String, self.output, self.qos)
        self.directory = tempfile.TemporaryDirectory(prefix='bumi-visual-odometry-')
        self.children = ChildProcesses(self.directory.name)
        self.input_subs = []
        self.result_subs = []
        self.segment_pubs = []
        self.calibration = None
        raw = self.config.get('calibration') or None
        self.calibration = CameraCalibration(raw) if raw else None
        self._new_segment()
        self.node.create_timer(.05, self._guard(self.tick))

    # -- lifecycle helpers ---------------------------------------------------
    def _guard(self, callback):
        def guarded(*args):
            with self.lock:
                try:
                    return callback(*args)
                except Exception as exc:
                    self.health.fault(str(exc))
                    self.publish()
        return guarded

    def now(self):
        return self.node.get_clock().now().nanoseconds

    def _new_segment(self):
        self.segment = 'bumi_visual_'+uuid.uuid4().hex
        self.health = Health(self.segment,
                             self.calibration.id if self.calibration else None, self.now())
        self.estimate = None
        self.epochs = {}
        self.cache = OrderedDict()
        self.images = OrderedDict()
        self.maps = None
        self.optics = None
        self.frame_id = None
        self.broadcaster = None
        self.visual_pending = OrderedDict()
        self.diagnostic_pending = OrderedDict()
        self.diagnostic_started = False
        self.last_visual_monotonic = None

    def base_frame(self):
        return self.segment+'_base'

    def pose_reference(self):
        return 'body' if self.calibration is not None else 'rgb_optical'

    def publish(self):
        sample = make_sample(self.health, self.now(), self.estimate, self.pose_reference())
        if not self.active:
            sample['vendor'].update(status='initializing', reason='visual odometry stopped')
            sample['pose'] = None
        self.publisher.publish(self.String(data=json.dumps(sample, allow_nan=False)))
        return sample

    def info(self):
        from common.odom import build_interface
        sample = self.publish()
        return dict(state='running' if self.active else 'idle',
                    topic_out=[{'topic': self.output, 'format': 'state/odom',
                                'message_type': 'std_msgs/msg/String'}],
                    odom_interface=build_interface(provides=[], rate_hz=20,
                                                   pose_drift='unbounded'),
                    quality=sample['vendor'],
                    namespace=self.namespace,
                    pose_reference=self.pose_reference(),
                    segment_id=self.segment,
                    calibration_id=self.calibration.id if self.calibration else None)

    def start(self):
        if not self.active:
            self._new_segment()
            self.active = True
            self.bind_inputs(self.namespace)
        return self.info()

    def stop(self, reason='visual odometry stopped'):
        if self.active:
            self.health.fault(reason)
            self.publish()
        self.active = False
        self.children.stop()
        for sub in self.input_subs+self.result_subs:
            self.node.destroy_subscription(sub)
        for pub in self.segment_pubs:
            self.node.destroy_publisher(pub)
        self.input_subs = []
        self.result_subs = []
        self.segment_pubs = []
        self.frame_id = None
        self.broadcaster = None
        self.maps = None
        self.optics = None
        self.estimate = None
        return self.info()

    def reset(self):
        self.stop('coordinate reset requested')
        return self.start()

    def configure(self, args):
        proposed = {k: v for k, v in args.items() if k == 'calibration'}
        if proposed.get('calibration'):
            CameraCalibration(proposed['calibration'])  # validate before interrupting
        if all(self.config.get(k) == v for k, v in proposed.items()):
            return self.info()
        active = self.active
        self.stop('calibration changed')
        self.config.update(deepcopy(proposed))
        raw = self.config.get('calibration') or None
        self.calibration = CameraCalibration(raw) if raw else None
        return self.start() if active else self.info()

    # -- inputs --------------------------------------------------------------
    def _subscribe(self, cls, topic, callback, destination):
        destination.append(self.node.create_subscription(
            cls, topic, self._guard(callback), self.qos))

    def bind_inputs(self, namespace):
        from sensor_msgs.msg import Image, CameraInfo
        from std_msgs.msg import String
        import message_filters
        root = '/'+namespace+'/odometry_input/'
        self._subscribe(String, root+'camera_meta', self.metadata, self.input_subs)
        # Keep wrappers alive. ROS subscriptions are also explicitly removed on stop.
        self.filters = [message_filters.Subscriber(self.node, cls, root+name, qos_profile=self.qos)
                        for cls, name in ((Image, 'rgb'), (Image, 'depth'), (CameraInfo, 'info'))]
        self.input_subs.extend(f.sub for f in self.filters)
        self.sync = message_filters.ApproximateTimeSynchronizer(self.filters, 5, .02)
        self.sync.registerCallback(self._guard(self.rgbd))

    def _epoch(self, name, value):
        if not isinstance(value, str) or not value:
            raise ValueError('missing input epoch')
        if name in self.epochs and self.epochs[name] != value:
            raise ValueError(name+' input restarted; reset required')
        self.epochs[name] = value

    def metadata(self, msg):
        data = json.loads(msg.data)
        self._epoch('camera', data['epoch'])
        t = data['rgb_stamp_ns']
        now = self.now()
        if abs(t-data['depth_stamp_ns']) > MAX_SKEW_NS:
            raise ValueError('RGB/depth skew')
        if not 0 <= now-data['received_ns'] <= MAX_TRANSPORT_NS:
            raise ValueError('camera transport stale')
        for key in ('rgb', 'depth'):
            if data[key]['domain'] not in ('hardware_clock', 'system_time', 'global_time'):
                raise ValueError('unrecognized camera timestamp domain')
        self.cache[t] = data
        while len(self.cache) > 5:
            self.cache.popitem(last=False)
        self._prepare(t)

    def rgbd(self, rgb, depth, info):
        t = rgb.header.stamp.sec*10**9+rgb.header.stamp.nanosec
        if info.header.stamp.sec*10**9+info.header.stamp.nanosec != t:
            raise ValueError('RGB-D/calibration time mismatch')
        if abs(t-(depth.header.stamp.sec*10**9+depth.header.stamp.nanosec)) > MAX_SKEW_NS:
            raise ValueError('RGB-D/calibration time mismatch')
        if len({rgb.header.frame_id, depth.header.frame_id, info.header.frame_id}) != 1:
            raise ValueError('aligned depth frame mismatch')
        self.images[t] = (rgb, depth, info)
        while len(self.images) > 5:
            self.images.popitem(last=False)
        self._prepare(t)

    def _prepare(self, t):
        if t not in self.cache or t not in self.images:
            return
        meta = self.cache.pop(t)
        rgb, depth, info = self.images.pop(t)
        if (depth.header.stamp.sec*10**9+depth.header.stamp.nanosec != meta['depth_stamp_ns']
                or t != meta['rgb_stamp_ns']):
            raise ValueError('metadata does not describe paired images')
        self.health.observe('camera', t, self.now(), True)
        self.health.details['camera_time_provenance'] = dict(
            epoch=meta.get('epoch'), clock_mapping=meta.get('clock_mapping'),
            depth_scale=meta.get('depth_scale'))
        frame_id = rgb.header.frame_id
        if not frame_id:
            raise ValueError('image frame_id is empty')
        if self.frame_id is not None and frame_id != self.frame_id:
            raise ValueError('optical frame changed; reset required')
        self.frame_id = frame_id
        import numpy as np
        if rgb.encoding != 'bgr8' or depth.encoding != '32FC1' or depth.is_bigendian:
            raise ValueError('unsupported shared image encoding')
        if (rgb.width, rgb.height) != (depth.width, depth.height) or \
                (rgb.width, rgb.height) != (info.width, info.height):
            raise ValueError('camera intrinsics dimensions mismatch')
        key = (info.width, info.height, tuple(info.k), tuple(info.d), info.distortion_model)
        if self.optics is not None and key != self.optics:
            raise ValueError('camera intrinsics changed; reset required')
        self.optics = key
        if info.distortion_model != 'plumb_bob':
            raise ValueError('unsupported distortion')
        k = np.asarray(finite(list(info.k), 9)).reshape(3, 3)
        distortion = finite(list(info.d), len(info.d))
        if k[0, 0] <= 0 or k[1, 1] <= 0:
            raise ValueError('invalid camera focal length')
        identity = not any(abs(v) > 1e-12 for v in distortion)
        if not identity:
            import cv2
            if self.maps is None:
                self.maps = cv2.initUndistortRectifyMap(
                    k, np.asarray(distortion), None, k, (info.width, info.height),
                    cv2.CV_32FC1)
        if not self.children.children:
            self.launch()
        def pixels(msg, dtype, channels):
            if msg.step < msg.width*channels*np.dtype(dtype).itemsize or \
                    len(msg.data) != msg.height*msg.step:
                raise ValueError('invalid shared image stride or size')
            rows = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(msg.height, msg.step)
            return rows[:, :msg.width*channels*np.dtype(dtype).itemsize].copy() \
                       .view(dtype).reshape(msg.height, msg.width, channels).squeeze()
        color = pixels(rgb, 'u1', 3)
        metres = pixels(depth, '<f4', 1)
        if not identity:
            color = cv2.remap(color, *self.maps, cv2.INTER_LINEAR)
            metres = cv2.remap(metres, *self.maps, cv2.INTER_NEAREST)
        if np.count_nonzero(np.isfinite(metres) & (metres > .1) & (metres < 10)) < metres.size*.1:
            raise ValueError('insufficient usable depth')
        from sensor_msgs.msg import Image
        for image, data, encoding, bpp, pub in ((rgb, color, 'bgr8', 3, self.rgb_pub),
                                                (depth, metres, '32FC1', 4, self.depth_pub)):
            out = Image()
            out.header = deepcopy(image.header)
            out.header.frame_id = self.frame_id
            out.width, out.height, out.encoding, out.step = rgb.width, rgb.height, encoding, rgb.width*bpp
            out.data = data.tobytes()
            pub.publish(out)
        info = deepcopy(info)
        info.header.frame_id = self.frame_id
        info.d = [0.]*5
        self.info_pub.publish(info)

    def launch(self):
        from sensor_msgs.msg import Image, CameraInfo
        from nav_msgs.msg import Odometry
        from rtabmap_msgs.msg import OdomInfo
        from geometry_msgs.msg import TransformStamped
        from tf2_ros import StaticTransformBroadcaster
        root = '/'+self.segment
        self.rgb_pub = self.node.create_publisher(Image, root+'/rgb', self.qos)
        self.depth_pub = self.node.create_publisher(Image, root+'/depth', self.qos)
        self.info_pub = self.node.create_publisher(CameraInfo, root+'/info', self.qos)
        self.segment_pubs = [self.rgb_pub, self.depth_pub, self.info_pub]
        # RTAB-Map wants a base frame distinct from the image frame and a TF to
        # it. With no measured mount the transform is identity, so the base frame
        # coincides with the optical frame rather than pretending to be a body point.
        transform = TransformStamped()
        transform.header.stamp = self.node.get_clock().now().to_msg()
        transform.header.frame_id = self.base_frame()
        transform.child_frame_id = self.frame_id
        translation, quaternion = mount_transform(self.calibration)
        transform.transform.translation.x = translation[0]
        transform.transform.translation.y = translation[1]
        transform.transform.translation.z = translation[2]
        transform.transform.rotation.x = quaternion[0]
        transform.transform.rotation.y = quaternion[1]
        transform.transform.rotation.z = quaternion[2]
        transform.transform.rotation.w = quaternion[3]
        self.broadcaster = StaticTransformBroadcaster(self.node)
        self.broadcaster.sendTransform([transform])
        self._subscribe(Odometry, root+'/visual',
                        lambda msg: self.odometry(msg), self.result_subs)
        self._subscribe(OdomInfo, root+'/diagnostic', self.diagnostic, self.result_subs)
        self.children.start(self.segment, self.base_frame())

    def odometry(self, msg):
        try:
            data = estimate(msg, self.segment, self.base_frame())
        except ValueError:
            if 'visual' not in self.health.samples:
                return  # uninitialized RTAB-Map estimate
            raise
        self.visual_pending[data['stamp_ns']] = data
        while len(self.visual_pending) > 20:
            self.visual_pending.popitem(last=False)
        self._release_visual(data['stamp_ns'])

    def _release_visual(self, t):
        if t not in self.visual_pending or t not in self.diagnostic_pending:
            return
        data = self.visual_pending.pop(t)
        self.diagnostic_pending.pop(t)
        self.health.observe('visual', t, self.now(), True)
        self.health.observe('diagnostic', t, self.now(), True)
        if self.health.reason:
            return
        if self.estimate is not None:
            dt = (t-self.estimate['stamp_ns'])/1e9
            shift = math.dist(data['pose']['position'], self.estimate['pose']['position'])
            if dt <= 0 or shift > .03+1.5*dt:
                raise ValueError('visual pose jumped or reset')
        self.estimate = data
        self.last_visual_monotonic = time.monotonic()

    def diagnostic(self, msg):
        try:
            from .visual_odometry_contract import MIN_INLIERS
        except ImportError:
            from visual_odometry_contract import MIN_INLIERS
        valid = not msg.lost and msg.inliers >= MIN_INLIERS and msg.matches >= msg.inliers
        if not valid and not self.diagnostic_started:
            return  # first registration has no motion estimate yet
        if not valid:
            self.health.details['fault_stamp_ns'] = (
                msg.header.stamp.sec*10**9+msg.header.stamp.nanosec)
            self.health.fault('visual tracking lost or insufficient inliers')
            return
        self.diagnostic_started = True
        self.health.details['visual_diagnostic'] = dict(
            inliers=msg.inliers, matches=msg.matches, lost=msg.lost)
        t = msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
        self.diagnostic_pending[t] = msg
        while len(self.diagnostic_pending) > 20:
            self.diagnostic_pending.popitem(last=False)
        self._release_visual(t)

    def tick(self):
        if self.closed:
            return
        if self.active:
            if self.children.failed():
                self.health.fault('RTAB-Map visual odometry process exited')
            if self.last_visual_monotonic is not None and \
                    time.monotonic()-self.last_visual_monotonic > MAX_INPUT_AGE_NS/1e9:
                self.health.fault('visual odometry receipt watchdog expired')
        self.publish()

    def close(self):
        with self.lock:
            if self.closed:
                return
            self.stop('bundle shutting down')
            self.closed = True
            self.node.destroy_node()
            self.directory.cleanup()


# ── Bundle plugin ────────────────────────────────────────────────────────────

class VisualOdometryPlugin:
    """Bundle-facing sensor card for the read-only visual odometry stream."""

    PREFIX = 'visual_odometry'

    def __init__(self, plugin_config, namespace, executor):
        self._namespace = namespace
        self._node = VisualOdometryNode(namespace, plugin_config)
        executor.add_node(self._node.node)

    def get_tools(self) -> list:
        return [{
            'name': 'rgbd_visual_odometry',
            'type': 'sensor',
            'multiInstance': False,
            'description': ('Bumi RGB-D visual odometry (RTAB-Map rgbd_odometry), read-only — '
                            f'no IMU/EKF/fusion, not in the motion path. Publishes to {OUTPUT_TOPIC}'),
            'inputSchema': {'type': 'object', 'properties': {
                'action': {'type': 'string',
                           'enum': ['start', 'stop', 'info', 'reset', 'config']}}},
            'configSchema': {'type': 'object', 'properties': {
                'calibration': {'type': 'object',
                                'description': 'Optional measured camera_to_body mount; '
                                               'without it the pose stays in the optical frame'}}},
            'topic_out': [{'topic': OUTPUT_TOPIC, 'format': 'state/odom'}],
        }]

    def start(self) -> None:
        self._node.start()

    def stop(self) -> None:
        self._node.stop()

    def dispatch(self, action, args) -> dict | None:
        node = self._node
        with node.lock:
            if action == 'start':
                return node.start()
            if action == 'stop':
                return node.stop()
            if action == 'reset':
                return node.reset()
            if action == 'config':
                return node.configure(args)
            if action in ('info', 'rgbd_visual_odometry'):
                # The bundle dispatcher passes the bare tool name when a client
                # calls the card without an explicit action; keep that as info.
                return node.info()
            return None  # unknown actions must be rejected, not reported as success
