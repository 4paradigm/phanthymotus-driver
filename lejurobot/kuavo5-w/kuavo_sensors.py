"""Measured ROS1 telemetry and read-only robot discovery for Kuavo 5-W.

See SDK_ANALYSIS.md for the pinned vendor sources. No SDK default state, MPC
targets, or synthetic joint names are used as measurements.
"""
from __future__ import annotations

from collections import deque
import copy
import hashlib
import json
import math
from pathlib import Path
import threading
import time
import uuid
import xml.etree.ElementTree as ET

from std_msgs.msg import String
from common.odom import build_interface, build_sample, mean_twist, resolve_stamp_ms
from spatial_telemetry import odometry_pose, imu_packet


LINK_ORDER = (['knee_link', 'leg_link', 'waist_link', 'waist_yaw_link'] +
              [f'zarm_{side}{i}_link' for side in ('l', 'r') for i in range(1, 8)] +
              ['zhead_1_link', 'zhead_2_link'])
SENSOR_CARDS = ('joints', 'imu', 'loco_state', 'model')
IDENTITY_REFRESH_S = 1.0
IDENTITY_LEASE_S = 3.0
JOINT_MAX_AGE_S = 0.5
JOINT_FUTURE_TOLERANCE_S = 0.05


def finite(value):
    return (not isinstance(value, bool) and isinstance(value, (int, float))
            and math.isfinite(value))


def ros_time_ns(value, name):
    if not isinstance(value, dict):
        raise ValueError(f'{name} is missing')
    secs, nsecs = value.get('secs'), value.get('nsecs')
    if (type(secs) is not int or type(nsecs) is not int or secs < 0
            or not 0 <= nsecs < 1_000_000_000 or secs + nsecs == 0):
        raise ValueError(f'{name} is invalid')
    return secs * 1_000_000_000 + nsecs


def vector(value, size):
    if not isinstance(value, list) or len(value) != size or not all(map(finite, value)):
        raise ValueError(f'expected {size} finite sensor values')
    return list(value)


def xyz(value, keys=('x', 'y', 'z')):
    return vector([value.get(k) for k in keys], len(keys))


def stamp(message):
    raw = message.get('sensor_time') or message.get('header', {}).get('stamp', {})
    if not isinstance(raw, dict):
        raw = {}
    secs, nsecs = raw.get('secs'), raw.get('nsecs')
    milliseconds = secs * 1000 + nsecs / 1e6 if finite(secs) and finite(nsecs) else None
    return resolve_stamp_ms(vendor_ms=milliseconds, received_ms=int(time.time() * 1000))


def json_safe(value):
    """Keep unmeasured vendor fields diagnostic, without emitting NaN JSON."""
    if isinstance(value, dict):
        return {k: json_safe(v) for k, v in value.items()}
    if isinstance(value, list):
        return [json_safe(v) for v in value]
    return None if isinstance(value, float) and not math.isfinite(value) else value


def model_names(urdf):
    if not isinstance(urdf, str) or len(urdf) > 5_000_000:
        raise ValueError('missing or oversized humanoid_description')
    # External entities/DTDs are not needed in an expanded robot URDF.
    if '<!DOCTYPE' in urdf.upper() or '<!ENTITY' in urdf.upper():
        raise ValueError('URDF must not contain DTDs or entities')
    root = ET.fromstring(urdf)
    if root.tag != 'robot':
        raise ValueError('humanoid_description is not a robot URDF')
    by_child = {}
    for joint in root.findall('joint'):
        child = joint.find('child')
        if child is not None and joint.get('type') in ('revolute', 'continuous'):
            link = child.get('link')
            if link in by_child:
                raise ValueError('ambiguous URDF child link')
            by_child[link] = joint.get('name')
    names = [by_child.get(link) for link in LINK_ORDER]
    if not all(names) or len(set(names)) != 20:
        raise ValueError('URDF does not match verified 5-W lower4 + arms14 + head2 layout')
    return names


class KuavoSensors:
    def __init__(self, config, namespace, ros1, node):
        self.ros1 = ros1
        self._lock = threading.RLock()
        self._closed = threading.Event()
        self._generation = -1
        self._profile = None
        self._urdf, self._names = None, None
        self._model_sha256 = None
        self._identity_checked_at = None
        self._joint_capture_ns = None
        self._joint_captured_at = None
        self._joint_header_ns = None
        self._joint_timebase = config.get('joint_timebase', 'auto')
        if self._joint_timebase not in ('auto', 'wall', 'kuavo_v63_cycle'):
            raise ValueError('joint_timebase must be auto, wall or kuavo_v63_cycle')
        self._joint_limit_config = copy.deepcopy(config.get('joint_limits'))
        self._arm_limit_source = config.get('arm_limit_source', 'commissioned')
        if self._arm_limit_source not in ('commissioned', 'vendor_runtime'):
            raise ValueError('arm_limit_source must be commissioned or vendor_runtime')
        self._leg_limit_source = config.get('leg_limit_source', 'commissioned')
        if self._leg_limit_source not in ('commissioned', 'vendor_runtime'):
            raise ValueError('leg_limit_source must be commissioned or vendor_runtime')
        self._vendor_leg_bounds = None
        self._vendor_leg_error = 'waiting_for_discovery'
        self._vendor_arm_bounds = None
        self._vendor_arm_error = 'waiting_for_discovery'
        self._model_error = 'waiting_for_discovery'
        self._model_path = Path(__file__).resolve().parent / 'resource' / 'kuavo5w_model.urdf'
        self._discovery_error = 'waiting_for_discovery'
        self._discovering = False
        self._retry_at = 0
        self._thread = None
        self._raw_topic = None
        self._raw_generation = None
        self._data = {}
        self._seen = {}
        self._errors = {}
        self._odom_window = deque(maxlen=2048)
        self._odom_epoch = uuid.uuid4().hex
        self._camera_imu, self._camera_imu_at = None, None
        self._camera_imu_status = {'state': 'unavailable', 'reason': 'waiting for native camera IMU relay'}
        self._display_pose = {}
        self._pose_seen = None
        self._running = set()
        self._model_running = False  # Resource lifecycle, never a telemetry stream.
        self._body_frames = set(config.get('odom_body_frames', ['base_link', 'base_footprint']))
        self._torque_unit = config.get('joint_torque_unit')
        self._imu_topic = config.get('imu_topic', '/imu')
        self._standard_imu_at = None
        self._static_rotations = {}
        self._body_gravity = None
        if self._torque_unit not in (None, 'N*m'):
            raise ValueError('joint_torque_unit must be null or hardware-verified N*m')
        self._topics = {'joints': f'/{namespace}/state/joints',
                        'imu': f'/{namespace}/state/imu',
                        'loco_state': f'/{namespace}/state/odom'}
        self._pubs = {key: node.create_publisher(String, topic, 10)
                      for key, topic in self._topics.items()}
        self._joint_state_topic = f'/{namespace}/state/joint_state'
        self._joint_state_pub = node.create_publisher(String, self._joint_state_topic, 1)
        self._joint_raw = None
        self._joint_raw_seen = None
        self._camera_imu_topic = f'/{namespace}/state/camera_imu'
        self._camera_imu_pub = node.create_publisher(String, self._camera_imu_topic, 20)
        self._loco_topic = f'/{namespace}/state/loco_state'
        self._loco_pub = node.create_publisher(String, self._loco_topic, 1)
        # Average all received odometry within each 10 Hz output window. Do not
        # decimate at rosbridge: it both aliases data and creates empty windows.
        ros1.subscribe('/odom', 'nav_msgs/Odometry', self._on_odom)
        ros1.subscribe('/move_base/amcl_pose', 'geometry_msgs/PoseWithCovarianceStamped', self._on_localization)
        # Read-only diagnostics do not depend on the full controller being up.
        for topic in ('/sensors_data_raw', '/sensors_data_raw_shm'):
            ros1.subscribe(topic, 'kuavo_msgs/sensorsData',
                           lambda msg, source=topic: self._on_joint_raw(msg, source), throttle_ms=100)
        ros1.subscribe('/tf_static', 'tf2_msgs/TFMessage', self._on_static_tf)
        ros1.subscribe(self._imu_topic, 'sensor_msgs/Imu', self._on_standard_imu)
        ros1.subscribe('/motus/kuavo5w/camera_imu', 'std_msgs/String', self._on_camera_imu)
        ros1.subscribe('/motus/kuavo5w/camera_imu/status', 'std_msgs/String', self._on_camera_imu_status)
        self._timer = node.create_timer(0.1, self.tick)

    def _session(self):
        connected, generation = self.ros1.connection
        with self._lock:
            if not connected or generation != self._generation:
                self._profile = self._urdf = self._names = None
                self._model_sha256 = None
                self._vendor_arm_bounds = None
                self._identity_checked_at = None
                self._standard_imu_at = self._body_gravity = None
                self._static_rotations.clear()
                self._joint_capture_ns = self._joint_captured_at = None
                self._joint_header_ns = None
                self._data.clear()
                self._seen.clear()
                self._errors.clear()
                self._odom_window.clear()
                self._odom_epoch = uuid.uuid4().hex
                self._camera_imu, self._camera_imu_at = None, None
                self._display_pose.clear()
                self._pose_seen = self._joint_raw_seen = None
                self._joint_raw = None
                self._generation = generation
                self._discovery_error = 'disconnected' if not connected else 'waiting_for_discovery'
                self._model_error = self._discovery_error
        return connected, generation

    def discover(self):
        """Run off the WebSocket receiver; rosapi replies need that receiver."""
        connected, generation = self._session()
        if not connected:
            return
        checked_at = time.monotonic()
        refresh_ok = False
        try:
            defaults = {'/' + name: None for name in (
                'robot_type', 'robot_version', 'armRealDof', 'headRealDof',
                'legRealDof', 'waistRealDof')}
            defaults.update({'/use_shm_communication': False, '/humanoid_description': None})
            if 'vendor_runtime' in (self._arm_limit_source, self._leg_limit_source):
                defaults['/kuavo_configuration'] = None
            values = self.ros1.get_params(defaults)
            params = {name: values['/' + name] for name in (
                'robot_type', 'robot_version', 'armRealDof', 'headRealDof',
                'legRealDof', 'waistRealDof', 'use_shm_communication')}
            missing = [key for key, value in params.items() if value is None]
            if missing:
                raise ValueError('controller_not_ready: missing ROS parameters ' + ', '.join(missing))
            counts = [params[k] for k in ('armRealDof', 'headRealDof', 'legRealDof', 'waistRealDof')]
            if (type(params['robot_type']) is not int or params['robot_type'] != 1 or
                    any(type(v) is not int or v < 0 for v in counts) or
                    counts != [14, 2, 4, 0]):
                raise ValueError('robot is not the supported wheel-arm 4+14+2 configuration')
            if type(params['use_shm_communication']) is not bool:
                raise ValueError('use_shm_communication must be a ROS boolean')
            topic = '/sensors_data_raw_shm' if params['use_shm_communication'] else '/sensors_data_raw'
            try:
                urdf = values['/humanoid_description']
                names = model_names(urdf)
                if names != model_names(self._model_path.read_text(encoding='utf-8')):
                    raise ValueError('runtime joint names differ from resource/kuavo5w_model.urdf; update the reference model')
                model_error = None
            except Exception as exc:
                urdf, names, model_error = None, None, str(exc)
            runtime_bounds, runtime_errors = {}, {}
            for card, offset, size, source in (('arm', 4, 14, self._arm_limit_source),
                                                ('leg', 0, 4, self._leg_limit_source)):
                vendor_bounds, vendor_error = None, None
                if source == 'vendor_runtime':
                    try:
                        if model_error:
                            raise ValueError(model_error)
                        # Match the vendor SDK's live-model source, never the local
                        # display resource. Require the controller's bounds too.
                        settings = values['/kuavo_configuration']
                        if isinstance(settings, str):
                            settings = json.loads(settings)
                        if not isinstance(settings, dict):
                            raise ValueError('missing runtime kuavo_configuration')
                        if any(type(settings.get(key)) is not int or settings[key] != count
                               for key, count in (('NUM_JOINT', 20), ('NUM_ARM_JOINT', 14),
                                                  ('NUM_HEAD_JOINT', 2))):
                            raise ValueError('runtime limit configuration has incompatible joint counts')
                        lower = vector(settings.get('min_joint_position_limits'), 20)
                        upper = vector(settings.get('max_joint_position_limits'), 20)
                        if any(a >= b for a, b in zip(lower, upper)):
                            raise ValueError('invalid controller position limits')
                        elements = ET.fromstring(urdf).findall('joint')
                        joints = {j.get('name'): j for j in elements}
                        if len(joints) != len(elements):
                            raise ValueError('duplicate runtime joint names')
                        vendor_bounds = []
                        for i, name in enumerate(names[offset:offset + size], offset):
                            joint = joints[name]
                            if joint.get('type') != 'revolute':
                                raise ValueError('arm joint must have finite revolute limits')
                            limit = joint.find('limit')
                            lo, hi = float(limit.attrib['lower']), float(limit.attrib['upper'])
                            if not finite(lo) or not finite(hi) or lo >= hi:
                                raise ValueError('invalid runtime URDF arm limits')
                            low = max(lo, math.radians(lower[i]))
                            high = min(hi, math.radians(upper[i]))
                            if low >= high:
                                raise ValueError('invalid or empty runtime arm limit intersection')
                            vendor_bounds.append((low, high))
                    except Exception as exc:
                        vendor_bounds, vendor_error = None, str(exc)
                runtime_bounds[card], runtime_errors[card] = vendor_bounds, vendor_error
            with self._lock:
                if self._closed.is_set() or self.ros1.connection != (True, generation):
                    return
                if self._raw_topic and self._raw_topic != topic:
                    self.ros1.unsubscribe(self._raw_topic)
                    for card in ('joints', 'imu'):
                        if card == 'imu' and self._standard_imu_at is not None:
                            continue
                        self._data.pop(card, None)
                        self._seen.pop(card, None)
                        self._errors.pop(card, None)
                model_sha256 = hashlib.sha256(urdf.encode('utf-8')).hexdigest() if urdf else None
                if self._names != names or self._model_sha256 != model_sha256 or self._profile != params:
                    self._data.pop('joints', None)
                    self._seen.pop('joints', None)
                if self._raw_topic != topic or self._raw_generation != generation:
                    self.ros1.subscribe(topic, 'kuavo_msgs/sensorsData',
                        lambda msg, source=topic, session=generation: self._on_raw(msg, source, session))
                self._raw_topic = topic
                self._raw_generation = generation
                self._profile = params
                self._urdf, self._names, self._model_error = urdf, names, model_error
                self._model_sha256 = model_sha256
                self._vendor_arm_bounds, self._vendor_arm_error = runtime_bounds['arm'], runtime_errors['arm']
                self._vendor_leg_bounds, self._vendor_leg_error = runtime_bounds['leg'], runtime_errors['leg']
                # Lease starts before parameter reads, not when slow reads finish.
                self._identity_checked_at = checked_at
                self._discovery_error = None
                refresh_ok = True
        except Exception as exc:
            with self._lock:
                if self.ros1.connection == (True, generation):
                    self._profile = self._urdf = self._names = None
                    self._model_sha256 = None
                    self._vendor_arm_bounds = None
                    self._identity_checked_at = None
                    for card in ('joints', 'imu'):
                        if card == 'imu' and self._standard_imu_at is not None:
                            continue
                        self._data.pop(card, None)
                        self._seen.pop(card, None)
                        self._errors.pop(card, None)
                    self._discovery_error = str(exc)
                    self._model_error = str(exc)
        finally:
            with self._lock:
                self._discovering = False
                # Keep a start-to-start cadence. Waiting another full interval
                # after a slow read needlessly consumes the previous lease.
                self._retry_at = (checked_at if refresh_ok else time.monotonic()) + IDENTITY_REFRESH_S

    def _identity_fresh(self):
        return (self._identity_checked_at is not None
                and 0 <= time.monotonic() - self._identity_checked_at <= IDENTITY_LEASE_S)

    def require_profile(self):
        connected, _ = self._session()
        with self._lock:
            if not connected or self._profile is None:
                raise RuntimeError(f'robot discovery is not ready: {self._discovery_error}')
            if not self._identity_fresh():
                raise RuntimeError('runtime robot identity verification is stale')

    def health(self):
        self._session()
        with self._lock:
            limits = {}
            for card in ('arm', 'leg'):
                try:
                    self._joint_bounds(card)
                    vendor = (getattr(self, '_' + card + '_limit_source') == 'vendor_runtime'
                              and self._joint_limit_config is None)
                    limits[card] = ({'available': True, 'source': 'vendor_runtime',
                                     'hardware_verified': False,
                                     'runtime_urdf_sha256': self._model_sha256}
                                    if vendor else {'available': True,
                                        'verification': self._joint_limit_config['verification']})
                except (RuntimeError, ValueError, TypeError):
                    limits[card] = {'available': False}
            return {'profile': copy.deepcopy(self._profile), 'sensor_topic': self._raw_topic,
                    'identity_verification': {'valid': self._identity_fresh(),
                        'age_s': None if self._identity_checked_at is None else
                                 max(0, time.monotonic() - self._identity_checked_at)},
                    'joint_limits': limits,
                    'arm_limit_source': self._arm_limit_source,
                    'vendor_arm_limit_error': self._vendor_arm_error,
                    'joint_timing': {'configured': self._joint_timebase,
                                     'source': self._joint_timing_mode(),
                                     'max_age_s': JOINT_MAX_AGE_S},
                    'discovery_error': self._discovery_error, 'model_error': self._model_error,
                    'skeleton_model': {'source': 'resource/kuavo5w_model.urdf',
                                       'reference_version': '63',
                                       'reference_only': True, 'hardware_geometry_verified': False,
                                       'configured_robot_serial': getattr(self, 'robot_serial', None),
                                       'serial_source': 'operator_config_not_hardware_verification',
                                       'runtime_joint_names_verified': self._names is not None}}

    def _joint_bounds(self, card):
        """Caller holds _lock. Use an explicit commissioning record or the
        selected vendor live-model/controller intersection, never display limits.
        """
        policy = self._joint_limit_config
        if policy is None and getattr(self, '_' + card + '_limit_source') == 'vendor_runtime':
            if (self._profile is None or not self._names or self._model_error is not None
                    or not self._identity_fresh()):
                raise RuntimeError('runtime arm model/limit identity is missing or stale')
            bounds = getattr(self, '_vendor_' + card + '_bounds')
            if bounds is None:
                raise RuntimeError(f'vendor runtime {card} limits unavailable: ' + str(getattr(self, '_vendor_' + card + '_error')))
            return list(bounds)
        if not isinstance(policy, dict) or not isinstance(policy.get(card), dict):
            raise RuntimeError(f'hardware-verified joint limits are not configured for {card}')
        if self._profile is None or not self._names or self._model_error is not None:
            raise RuntimeError('runtime joint model is not validated')
        if not self._identity_fresh():
            raise RuntimeError('runtime robot identity verification is stale')
        record = policy.get('verification')
        if not isinstance(record, str) or not record.strip() or len(record) > 512:
            raise ValueError('joint limits require a commissioning verification record')
        version = policy.get('robot_version')
        if (type(version) not in (int, str) or type(version) is not type(self._profile['robot_version'])
                or version != self._profile['robot_version'] or not version
                or (isinstance(version, str) and not version.strip())):
            raise ValueError('joint limits robot_version does not match the runtime robot')
        if policy.get('runtime_urdf_sha256') != self._model_sha256:
            raise ValueError('joint limits runtime URDF fingerprint does not match')
        offset, size = (4, 14) if card == 'arm' else (0, 4)
        entry = policy[card]
        if entry.get('joint_names') != self._names[offset:offset + size]:
            raise ValueError(f'{card} joint limit names/order do not match the runtime model')
        low, high = vector(entry.get('lower_deg'), size), vector(entry.get('upper_deg'), size)
        if any(a >= b for a, b in zip(low, high)):
            raise ValueError('joint limits require finite lower_deg < upper_deg')
        return [(math.radians(a), math.radians(b)) for a, b in zip(low, high)]

    def joint_bounds(self, card):
        with self._lock:
            return self._joint_bounds(card)

    def height_target(self, height, with_pose=False):
        target, pose, _ = self._height_plan(height)
        return (target, pose) if with_pose else target

    def height_state(self):
        return self._height_plan(None)[2]

    def _head_top_state(self, root, torso_z, torso_pitch, waist_yaw):
        """Highest head collision sphere, not a measured scalp/ground height.

        The vendor's visual head is an external STL; do not substitute its
        frame origin or a fixed nominal body height for physical geometry.
        """
        result = {'head_top_height_m': None, 'head_top_frame_id': 'base_link',
                  'head_top_basis': 'live_urdf_head_collision_envelope_estimate',
                  'head_top_is_estimate': True}
        try:
            joints = {j.get('name'): j for j in root.findall('joint')}
            link = root.find("link[@name='zhead_1_link']")
            head = joints['zhead_1_joint']
            if (head.find('parent').get('link') != 'waist_yaw_link'
                    or head.find('child').get('link') != 'zhead_1_link'
                    or vector([float(v) for v in head.find('axis').get('xyz').split()], 3) != [0., 0., 1.]):
                raise ValueError('unsupported head chain')
            def xyz(element):
                origin = element.find('origin')
                if origin is None:
                    return [0., 0., 0.]
                if any(abs(v) > 1e-8 for v in vector([float(v) for v in origin.get('rpy', '0 0 0').split()], 3)):
                    raise ValueError('unsupported rotated head origin')
                return vector([float(v) for v in origin.get('xyz', '0 0 0').split()], 3)
            offset = xyz(head)
            yaw = waist_yaw + self.joint_positions()[0][18]
            tops = []
            for collision in link.findall('collision'):
                sphere = collision.find('geometry/sphere')
                if sphere is None:
                    raise ValueError('unsupported head collision')
                radius = float(sphere.get('radius'))
                if not math.isfinite(radius) or radius <= 0:
                    raise ValueError('invalid head radius')
                cx, cy, cz = xyz(collision)
                # torso R_y(pitch) R_z(waist), then head R_z(yaw).
                local_x = (math.cos(waist_yaw)*offset[0]-math.sin(waist_yaw)*offset[1]
                           + math.cos(yaw)*cx-math.sin(yaw)*cy)
                tops.append(torso_z-math.sin(torso_pitch)*local_x
                            + math.cos(torso_pitch)*(offset[2]+cz)+radius)
            if not tops:
                raise ValueError('missing head collision geometry')
            result['head_top_height_m'] = max(tops)
            result['head_top_geometry_frame'] = 'zhead_1_link'
        except (KeyError, AttributeError, ValueError, TypeError, IndexError):
            result['head_top_reason'] = 'live_head_collision_geometry_unavailable_or_unsupported'
        return result

    def _height_plan(self, height):
        """Planar 3-joint IK from the validated live wheel-arm chain.

        Preserve torso x, pitch and waist yaw; reject unknown geometry or
        unreachable targets instead of applying a guessed joint ratio.
        """
        with self._lock:
            bounds = self._joint_bounds('leg')
            q = self.joint_positions()[0][:4]
            joints = {j.get('name'): j for j in ET.fromstring(self._urdf).findall('joint')}
            origins = []
            for i, name in enumerate(self._names[:4]):
                joint = joints[name]
                origin = joint.find('origin')
                xyz = vector([float(v) for v in origin.get('xyz', '0 0 0').split()], 3)
                rpy = vector([float(v) for v in origin.get('rpy', '0 0 0').split()], 3)
                axis = [float(v) for v in joint.find('axis').get('xyz').split()]
                if any(abs(v) > 1e-8 for v in rpy) or abs(xyz[1]) > 1e-8 or axis != ([0., 1., 0.] if i < 3 else [0., 0., 1.]):
                    raise RuntimeError('unsupported_live_leg_geometry_for_height')
                if i and joint.find('parent').get('link') != joints[self._names[i-1]].find('child').get('link'):
                    raise RuntimeError('unsupported_live_leg_chain_for_height')
                origins.append((xyz[0], xyz[2]))
            parent = joints[self._names[0]].find('parent').get('link')
            visited = set()
            while parent != 'base_link':
                if parent in visited:
                    raise RuntimeError('invalid_live_leg_fixed_chain')
                visited.add(parent)
                parents = [j for j in joints.values() if j.find('child').get('link') == parent]
                if len(parents) != 1 or parents[0].get('type') != 'fixed':
                    raise RuntimeError('unsupported_live_leg_root_for_height')
                fixed = parents[0]
                origin = fixed.find('origin')
                xyz = vector([float(v) for v in origin.get('xyz', '0 0 0').split()], 3)
                rpy = vector([float(v) for v in origin.get('rpy', '0 0 0').split()], 3)
                if abs(xyz[1]) > 1e-8 or any(abs(v) > 1e-8 for v in rpy):
                    raise RuntimeError('unsupported_live_leg_fixed_transform')
                origins[0] = (origins[0][0]+xyz[0], origins[0][1]+xyz[2])
                parent = fixed.find('parent').get('link')
            def rotate(v, a):
                return (math.cos(a)*v[0]+math.sin(a)*v[1], -math.sin(a)*v[0]+math.cos(a)*v[1])
            pitch = sum(q[:3])
            parts = [origins[0], rotate(origins[1], q[0]), rotate(origins[2], q[0]+q[1]), rotate(origins[3], pitch)]
            x = sum(v[0] for v in parts)
            end = rotate(origins[3], pitch)
            dx = x-origins[0][0]-end[0]
            z0 = origins[0][1]+end[1]
            l1, l2 = [math.hypot(*v) for v in origins[1:3]]
            if min(l1, l2) <= 1e-6:
                raise RuntimeError('invalid_live_leg_link_lengths')
            a1, a2 = [math.atan2(v[1], v[0]) for v in origins[1:3]]

            def solutions(z):
                dz = z-z0
                cosine = (dx*dx+dz*dz-l1*l1-l2*l2)/(2*l1*l2)
                if not -1-1e-10 <= cosine <= 1+1e-10:
                    return []
                candidates = []
                for delta in (math.acos(max(-1., min(1., cosine))), -math.acos(max(-1., min(1., cosine)))):
                    knee = a1-math.atan2(dz, dx)+math.atan2(l2*math.sin(delta), l1+l2*math.cos(delta))
                    leg = a2-a1-delta
                    # Both revolute angles have equivalent branches. Wrapping
                    # only the first joint can incorrectly reject a valid IK.
                    for kt in (-2*math.pi, 0., 2*math.pi):
                        for lt in (-2*math.pi, 0., 2*math.pi):
                            k, l = knee+kt, leg+lt
                            target = [k, l, pitch-k-l, q[3]]
                            if all(lo-1e-9 <= v <= hi+1e-9 for v, (lo, hi) in zip(target, bounds)):
                                candidates.append([min(hi, max(lo, v)) for v, (lo, hi) in zip(target, bounds)])
                return candidates

            # Feasibility changes only at a joint bound or a two-link tangent.
            # Intersect each boundary circle with the fixed-x vertical line;
            # test the resulting intervals with the same exact IK as commands.
            boundaries = []
            def circle(cx, cz, radius):
                remaining = radius*radius-(dx-cx)**2
                if remaining >= -1e-10:
                    dz = math.sqrt(max(0., remaining))
                    boundaries.extend((z0+cz-dz, z0+cz+dz))
            circle(0., 0., l1+l2)
            circle(0., 0., abs(l1-l2))
            for k in bounds[0]:
                v = rotate(origins[1], k)
                circle(*v, l2)
            for l in bounds[1]:
                v = rotate(origins[2], l)
                circle(0., 0., math.hypot(origins[1][0]+v[0], origins[1][1]+v[1]))
            for c in bounds[2]:
                v = rotate(origins[2], pitch-c)
                circle(*v, l1)
            boundaries = sorted(set(boundaries))
            intervals = []
            for low, high in zip(boundaries, boundaries[1:]):
                if high-low > 1e-9 and solutions((low+high)/2):
                    if intervals and abs(intervals[-1][1]-low) < 1e-8:
                        intervals[-1][1] = high
                    else:
                        intervals.append([low, high])
            for z in boundaries:
                if solutions(z) and not any(lo-1e-8 <= z <= hi+1e-8 for lo, hi in intervals):
                    intervals.append([z, z])
            intervals.sort()
            current = sum(v[1] for v in parts)
            status = {'height_m': current, 'frame_id': 'base_link', 'target_frame': 'waist_yaw_link',
                      'x_m': x, 'pitch_rad': pitch, 'yaw_rad': q[3],
                      'reachable_height_intervals_m': intervals,
                      'basis': 'live_urdf_and_joint_limits_at_current_x_pitch_yaw',
                      'meaning': 'torso_frame_origin_z_relative_to_base_link_not_total_robot_height'}
            status.update(self._head_top_state(ET.fromstring(self._urdf), current, pitch, q[3]))
            if height is None:
                return None, None, status
            candidates = solutions(height)
            if not candidates:
                ranges = ','.join(f'[{lo:.6f},{hi:.6f}]' for lo,hi in intervals) or 'none'
                raise ValueError(f'height_unreachable_at_current_pose: requested={height:.6f}m; '
                                 f'current={current:.6f}m; reachable_intervals_m={ranges}; '
                                 'preserving_torso_x_pitch_yaw_and_live_joint_limits')
            target = min(candidates, key=lambda t: sum((a-b)**2 for a,b in zip(t,q)))
            return target, {'x': x, 'z': height, 'pitch': pitch, 'yaw': q[3]}, status

    def odom_pose(self):
        with self._lock:
            raw = self._data.get('loco_state', {}).get('vendor', {})
            seen = self._seen.get('loco_state', -1e9)
            stamp_data = raw.get('header', {}).get('stamp', {})
            captured = stamp_data.get('secs', 0) + stamp_data.get('nsecs', 0)*1e-9
            if time.monotonic()-seen > .5 or not -.1 <= time.time()-captured <= .5:
                raise RuntimeError('fresh_stamped_odometry_required_for_pose_motion')
            fields = self._pose_fields(raw['pose']['pose'], raw['header']['frame_id'], '/odom')
            return (fields['x_m'], fields['y_m'], fields['yaw_rad']), captured, fields['frame_id']

    def resolve_joint_target(self, card, updates):
        with self._lock:
            offset, size = (4, 14) if card == 'arm' else (0, 4)
            q, _, _ = self.joint_positions()
            target = q[offset:offset + size]
            for index, value in updates.items():
                target[index] = value
            measured = self.validate_joint_target(card, target)
            return target, measured

    def validate_joint_target(self, card, target):
        with self._lock:
            # Read measurement and model/limit binding under one coherent lock.
            bounds = self._joint_bounds(card)
            measured = self.joint_positions()
            if len(target) != len(bounds):
                raise ValueError('joint target size does not match limits')
            for i, (value, (low, high)) in enumerate(zip(target, bounds)):
                if not finite(value) or not low <= value <= high:
                    raise ValueError(f'{card} joint target {i} is outside hardware-verified limits')
            return measured

    def _joint_timing_mode(self):
        version = self._profile.get('robot_version') if self._profile else None
        if self._joint_timebase == 'auto':
            return 'kuavo_v63_cycle' if version == 63 else 'wall'
        return self._joint_timebase

    def _store_joints(self, message, meta):
        # v63 HardwareNode::sensorThreadFunc reads sensors, stamps the header
        # with ros::Time::now(), and advances sensor_time by a fixed dt. That
        # cycle clock drifts against wall time; it is a sample identity, not a
        # UTC capture clock. Other profiles keep the strict wall-clock policy.
        capture = message.get('sensor_time')
        capture_ns = ros_time_ns(capture, 'joint sensor_time')
        mode = self._joint_timing_mode()
        header_ns = None
        if mode == 'kuavo_v63_cycle':
            if self._profile.get('robot_version') != 63:
                raise ValueError('kuavo_v63_cycle requires runtime robot_version 63')
            header_ns = ros_time_ns(message.get('header', {}).get('stamp'), 'joint header.stamp')
        now, wall = time.monotonic(), time.time()
        age = wall - (header_ns if header_ns is not None else capture_ns) / 1e9
        if not -JOINT_FUTURE_TOLERANCE_S <= age <= JOINT_MAX_AGE_S:
            raise ValueError('joint capture is stale or clocks are not synchronized: '
                             f'capture_age_s={age:.3f}, allowed_s='
                             f'[-{JOINT_FUTURE_TOLERANCE_S}, {JOINT_MAX_AGE_S}]')
        if self._joint_capture_ns is not None and capture_ns <= self._joint_capture_ns:
            return  # No data/freshness update for repeated or reordered captures.
        if header_ns is not None and self._joint_header_ns is not None:
            if header_ns <= self._joint_header_ns:
                return
            # A cycle jump is not evidence of a sustained real-time interval.
            # Do not learn an arbitrary offset from a delayed/replayed frame.
            if abs((capture_ns - self._joint_capture_ns) -
                   (header_ns - self._joint_header_ns)) > 50_000_000:
                raise ValueError('joint cycle/header progression is inconsistent')
        raw = message['joint_data']
        q = vector(raw.get('joint_q'), 20)
        arrays = {}
        for key, source in (('dq', 'joint_v'), ('ddq', 'joint_vd'), ('tau', 'joint_torque')):
            try:
                arrays[key] = vector(raw.get(source), 20)
            except ValueError:
                arrays[key] = [None] * 20
        # SDK effort may be N*m OR Amperes; retain raw values without guessing.
        raw_effort = arrays['tau']
        if self._torque_unit is None:
            arrays['tau'] = [None] * 20
        joints = [{'idx': i, 'name': self._names[i] if self._names else None, 'q': v,
                   **{key: values[i] for key, values in arrays.items()}} for i, v in enumerate(q)]
        if header_ns is not None:
            stamp_ms, provenance = resolve_stamp_ms(vendor_ms=header_ns / 1e6,
                                                    received_ms=int(wall * 1000))
            meta = {'stamp_ms': stamp_ms, 'vendor': {**meta['vendor'], **provenance}}
        self._data['joints'] = {**meta, 'joints': joints,
            'vendor': {**meta['vendor'], 'sensor_time': dict(capture), 'joint_torque': raw_effort,
                       'joint_timebase': mode, 'header': copy.deepcopy(message.get('header')),
                       'joint_torque_unit': self._torque_unit},
            'units': {'q': 'rad', 'dq': 'rad/s', 'ddq': 'rad/s^2', 'tau': self._torque_unit}}
        self._joint_capture_ns = capture_ns
        self._joint_header_ns = header_ns
        # Map capture into the monotonic clock used by command issue/deadlines.
        # Never put a slightly future-dated capture after its own receipt.
        self._joint_captured_at = now - max(0, age)
        self._seen['joints'] = now
        self._errors.pop('joints', None)

    def _on_raw(self, message, source, generation):
        if not self._session()[0]:
            return
        if generation != self._generation:
            return
        self._on_joint_raw(message, source)
        with self._lock:
            if (self._profile is None or source != self._raw_topic or
                    generation != self._generation):
                return
            stamp_ms, provenance = stamp(message)
            meta = {'stamp_ms': stamp_ms, 'vendor': {'topic': self._raw_topic, **provenance}}
            try:
                self._store_joints(message, meta)
            except (KeyError, TypeError, ValueError, AttributeError) as exc:
                self._data.pop('joints', None)
                self._seen.pop('joints', None)
                self._errors['joints'] = str(exc)
            try:
                if self._standard_imu_at is not None and time.monotonic() - self._standard_imu_at <= .5:
                    return  # Independent IMU takes precedence over controller raw IMU.
                imu = message['imu_data']
                quat = xyz(imu['quat'], ('w', 'x', 'y', 'z'))
                if abs(math.sqrt(sum(v*v for v in quat)) - 1) > 0.05:
                    raise ValueError('invalid IMU quaternion')
                self._data['imu'] = {**meta, 'quaternion': quat, 'quaternion_order': 'wxyz',
                    'gyroscope': xyz(imu['gyro']), 'accelerometer': xyz(imu['acc']),
                    'free_acceleration': xyz(imu['free_acc']),
                    'frame_id': message.get('header', {}).get('frame_id') or None,
                    'units': {'gyroscope': 'rad/s', 'accelerometer': 'm/s^2', 'free_acceleration': 'm/s^2'}}
                self._seen['imu'] = time.monotonic()
                self._errors.pop('imu', None)
            except (KeyError, TypeError, ValueError, AttributeError) as exc:
                self._data.pop('imu', None)
                self._seen.pop('imu', None)
                self._errors['imu'] = str(exc)

    def _on_joint_raw(self, message, source):
        """Expose actual arrays even before model verification; never for control."""
        if not self._session()[0]:
            return
        with self._lock:
            try:
                raw = message['joint_data']
                if not isinstance(raw, dict):
                    raise ValueError('missing joint_data')
                values = {}
                for key in ('joint_q', 'joint_v', 'joint_vd', 'joint_torque'):
                    array = raw.get(key)
                    if array is not None:
                        if not isinstance(array, list) or not 1 <= len(array) <= 256 or not all(map(finite, array)):
                            raise ValueError('invalid raw joint array')
                        values[key] = list(array)
                if 'joint_q' not in values:
                    raise ValueError('missing joint_q')
                self._joint_raw = {'source_topic': source, 'sensor_time': json_safe(message.get('sensor_time')),
                    **values, 'joint_count': len(values['joint_q']), 'q_unit': 'rad', 'v_unit': 'rad/s',
                    'vd_unit': 'rad/s^2', 'torque_unit': self._torque_unit,
                    'model_mapping_verified': self._names is not None and self._identity_fresh()}
                self._joint_raw_seen = time.monotonic()
            except (KeyError, TypeError, ValueError, AttributeError):
                self._joint_raw = self._joint_raw_seen = None

    def joint_raw_snapshot(self):
        self._session()
        with self._lock:
            valid = (self.ros1.connection[0] and self._joint_raw_seen is not None
                     and time.monotonic() - self._joint_raw_seen <= .5)
            empty = dict.fromkeys(('joint_q', 'joint_v', 'joint_vd', 'joint_torque', 'sensor_time'))
            return {**empty, **(copy.deepcopy(self._joint_raw) if valid else {'joint_count': 0}), 'valid': bool(valid),
                    'reason': None if valid else 'missing_or_stale_full_body_joint_data'}

    @staticmethod
    def _pose_fields(pose, frame, source):
        position = xyz(pose['position'])
        w, x, y, z = xyz(pose['orientation'], ('w', 'x', 'y', 'z'))
        if not isinstance(frame, str) or not frame or len(frame) > 256 or abs(w*w+x*x+y*y+z*z-1) > .1:
            raise ValueError('invalid pose/frame')
        roll = math.atan2(2*(w*x+y*z), 1-2*(x*x+y*y))
        pitch = math.asin(max(-1., min(1., 2*(w*y-z*x))))
        yaw = math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))
        return dict(zip(('x_m', 'y_m', 'z_m'), position), frame_id=frame, pose_source=source,
                    roll_rad=roll, pitch_rad=pitch, yaw_rad=yaw, yaw_deg=math.degrees(yaw))

    def _on_localization(self, message):
        if not self._session()[0]:
            return
        with self._lock:
            try:
                self._display_pose = self._pose_fields(message['pose']['pose'], message['header']['frame_id'],
                                                       '/move_base/amcl_pose')
                self._pose_seen = time.monotonic()
            except (KeyError, TypeError, ValueError, AttributeError):
                self._display_pose, self._pose_seen = {}, None

    def loco_display(self):
        """Display map localization separately from body-frame navigation odom."""
        self._session()
        with self._lock:
            now = time.monotonic()
            localized = self._pose_seen is not None and now - self._pose_seen <= 3
            result = dict(self._display_pose) if localized else {}
            raw = self._data.get('loco_state', {})
            odom_valid = now - self._seen.get('loco_state', -1e9) <= .5
            if not localized and odom_valid:
                try:
                    vendor = raw['vendor']
                    result = self._pose_fields(vendor['pose']['pose'], vendor['header']['frame_id'], '/odom')
                except (KeyError, TypeError, ValueError, AttributeError):
                    pass
            values = [None] * 6
            if odom_valid:
                twist = raw['vendor']['twist']['twist']
                values[0], values[1], values[5] = twist['linear']['x'], twist['linear']['y'], twist['angular']['z']
            empty = dict.fromkeys(('x_m', 'y_m', 'z_m', 'roll_rad', 'pitch_rad', 'yaw_rad', 'yaw_deg',
                                  'frame_id', 'pose_source'))
            return {**empty, **result, 'pose_valid': bool(result), 'localization_valid': localized,
                    'pose_age_ms': round((now-self._pose_seen)*1000) if localized else
                                   round((now-self._seen['loco_state'])*1000) if result else None,
                    'velocity_frame': raw.get('vendor', {}).get('child_frame_id') if odom_valid else None,
                    'velocity_valid': odom_valid, 'vx_m_s': values[0], 'vy_m_s': values[1], 'wz_rad_s': values[5],
                    'valid': bool(result) or odom_valid,
                    'reason': None if result or odom_valid else 'missing_or_stale_pose_and_odometry'}

    def _on_static_tf(self, message):
        if not self._session()[0]:
            return
        with self._lock:
            for transform in message.get('transforms', []):
                child = transform.get('child_frame_id') if isinstance(transform, dict) else None
                if not isinstance(child, str) or not child.lstrip('/'):
                    continue
                child = child.lstrip('/')
                try:
                    parent = transform['header']['frame_id'].lstrip('/')
                    rotation = xyz(transform['transform']['rotation'], ('w', 'x', 'y', 'z'))
                    norm = math.sqrt(sum(v*v for v in rotation))
                    if not parent or not child or abs(norm-1.) > .05:
                        raise ValueError('invalid static transform')
                    edge = parent, [v/norm for v in rotation]
                    if self._static_rotations.get(child) != edge:
                        self._static_rotations[child] = edge
                        self._body_gravity = None
                except (KeyError, TypeError, ValueError, AttributeError):
                    self._static_rotations.pop(child, None)
                    self._body_gravity = None

    def _gravity_in_body(self, message, orientation):
        header = message.get('header', {})
        try:
            ns = ros_time_ns(header.get('stamp'), 'IMU capture time')
        except ValueError:
            return None
        norm = math.sqrt(sum(v*v for v in orientation))
        w, x, y, z = (v/norm for v in orientation)
        up = [2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y)]
        # Resolve rigid branches through their common ancestor. A moving joint
        # never appears on /tf_static and cannot be used as an IMU mount.
        frame = header.get('frame_id')
        if not isinstance(frame, str) or not frame:
            return None
        frame = frame.lstrip('/')
        body_path = {'base_link': []}
        body = 'base_link'
        while body in self._static_rotations:
            parent, rotation = self._static_rotations[body]
            if parent in body_path:
                return None
            w, x, y, z = rotation
            body_path[parent] = [(w, -x, -y, -z)] + body_path[body]
            body = parent
        rotations = []
        seen = set()
        while frame not in body_path:
            if frame in seen or frame not in self._static_rotations:
                return None
            seen.add(frame)
            frame, rotation = self._static_rotations[frame]
            rotations.append(rotation)
        for w, x, y, z in rotations + body_path[frame]:
            a, b, c = up
            # Unit quaternion vector rotation: v + 2w(q x v) + 2q x (q x v).
            u, v, h = 2*(y*c-z*b), 2*(z*a-x*c), 2*(x*b-y*a)
            up = [a+w*u+y*h-z*v, b+w*v+z*u-x*h, c+w*h+x*v-y*u]
        gravity = dict(frame_id='base_link', stamp_ns=ns, up=up)
        covariance = message.get('orientation_covariance')
        if (isinstance(covariance, list) and len(covariance) == 9 and all(map(finite, covariance))
                and all(covariance[i] >= 0. for i in (0, 4, 8)) and any(covariance)):
            # An upper bound invariant under mounting rotation. ROS all-zero means unknown.
            gravity['variance_bound_rad2'] = max(sum(abs(v) for v in covariance[i:i+3])
                                                 for i in (0, 3, 6))
        return gravity

    def _on_standard_imu(self, message):
        if not self._session()[0]:
            return
        with self._lock:
            try:
                quat = xyz(message['orientation'], ('w', 'x', 'y', 'z'))
                if abs(math.sqrt(sum(v*v for v in quat)) - 1) > .05:
                    raise ValueError('invalid independent IMU quaternion')
                covariance = message.get('orientation_covariance', [])
                orientation = None if covariance and covariance[0] == -1 else quat
                stamp_ms, provenance = stamp(message)
                w, x, y, z = quat
                rpy = [math.atan2(2*(w*x+y*z), 1-2*(x*x+y*y)),
                       math.asin(max(-1., min(1., 2*(w*y-z*x)))),
                       math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))] if orientation else None
                self._data['imu'] = {'stamp_ms': stamp_ms, 'quaternion': orientation,
                    'quaternion_order': 'wxyz', 'rpy': rpy,
                    'gyroscope': xyz(message['angular_velocity']),
                    'accelerometer': xyz(message['linear_acceleration']),
                    'free_acceleration': None, 'frame_id': message.get('header', {}).get('frame_id'),
                    'units': {'gyroscope': 'rad/s', 'accelerometer': 'm/s^2', 'rpy': 'rad'},
                    'vendor': {'topic': self._imu_topic, **provenance}}
                self._body_gravity = self._gravity_in_body(message, quat) if orientation else None
                self._standard_imu_at = self._seen['imu'] = time.monotonic()
                self._errors.pop('imu', None)
            except (KeyError, TypeError, ValueError, AttributeError) as exc:
                self._data.pop('imu', None)
                self._seen.pop('imu', None)
                self._body_gravity = None
                self._errors['imu'] = str(exc)

    def _on_camera_imu_status(self, message):
        try:
            if not self._session()[0] or len(message.get('data', '')) > 8192:
                return
            status = json.loads(message['data'])
            if isinstance(status, dict):
                with self._lock:
                    self._camera_imu_status = status
        except (KeyError, ValueError, TypeError):
            pass

    def _on_camera_imu(self, message):
        if not self._session()[0]:
            return
        try:
            if len(message.get('data', '')) > 65536:
                raise ValueError('oversized IMU packet')
            packet = imu_packet(json.loads(message['data']))
            with self._lock:
                # Repeated bridge deliveries are not new captures.
                if (self._camera_imu and packet.get('epoch') == self._camera_imu.get('epoch')
                        and packet['stamp_ns'] <= self._camera_imu['stamp_ns']):
                    return
                self._camera_imu, self._camera_imu_at = packet, time.monotonic()
                if 'imu' in self._running:
                    out = String()
                    out.data = json.dumps(packet, allow_nan=False, separators=(',', ':'))
                    self._camera_imu_pub.publish(out)
        except (KeyError, ValueError, TypeError) as error:
            with self._lock:
                self._camera_imu, self._camera_imu_at = None, None
                self._camera_imu_status = {'state': 'unavailable', 'reason': str(error)}

    def camera_imu_status(self):
        with self._lock:
            fresh = self._camera_imu_at is not None and time.monotonic()-self._camera_imu_at <= .5
            packet = self._camera_imu if fresh else None
            fusion = packet is not None and all(k in packet for k in
                ('camera_from_gyro_translation', 'camera_from_accel_translation'))
            return {'state': 'available' if packet else 'unavailable',
                    'source': 'orbbec_camera_imu', 'body_display_source': self._imu_topic,
                    'reason': None if packet else self._camera_imu_status.get('reason') or 'missing_or_stale_camera_imu',
                    'fusion': {'state': 'available' if fusion else 'unavailable',
                               'reason': None if fusion else 'fresh raw vectors, rotations and sensor origins required'},
                    'relay': copy.deepcopy(self._camera_imu_status)}

    def _on_odom(self, message):
        if not self._session()[0]:
            return
        with self._lock:
            # Chassis odometry remains available while the whole-body controller
            # is unlaunched. Validate its own frame and data, independently of
            # the profile guard used for issuing motion commands.
            try:
                if message.get('child_frame_id') not in self._body_frames:
                    raise ValueError(f"unverified odometry child frame: {message.get('child_frame_id')!r}")
                twist = message['twist']['twist']
                values = [twist['linear']['x'], twist['linear']['y'], twist['angular']['z']]
                vector(values, 3)
                source_stamp_ms, provenance = stamp(message)
                self._data['loco_state'] = {'stamp_ms': int(time.time()*1000), 'vendor': {
                    'stamp_source': 'received',
                    'source_timing': {'stamp_ms': source_stamp_ms, **provenance},
                    'topic': '/odom', 'header': copy.deepcopy(message.get('header', {})),
                    'child_frame_id': message['child_frame_id'],
                    'pose': json_safe(message.get('pose')), 'twist': json_safe(message['twist'])}}
                try:
                    pose, quality = odometry_pose(message, self._odom_epoch)
                except (KeyError, TypeError, ValueError) as error:
                    pose, quality = None, {'profile': 'motus_slam/1', 'status': 'unavailable',
                                           'reason': str(error), 'pose_source': '/odom'}
                self._data['loco_state']['pose'] = pose
                self._data['loco_state']['vendor']['motus_slam'] = quality
                self._seen['loco_state'] = time.monotonic()
                self._errors.pop('loco_state', None)
                self._odom_window.append((time.monotonic(), [values[0], values[1], None, None, None, values[2]]))
            except (KeyError, TypeError, ValueError, AttributeError) as exc:
                self._data.pop('loco_state', None)
                self._seen.pop('loco_state', None)
                self._odom_window.clear()
                self._errors['loco_state'] = str(exc)

    def joint_positions(self):
        self.require_profile()
        with self._lock:
            if not self._names or self._model_error is not None:
                raise RuntimeError('runtime joint model is not validated')
            seen = self._seen.get('joints', 0)
            if (time.monotonic() - seen > JOINT_MAX_AGE_S or 'joints' not in self._data
                    or self._joint_captured_at is None
                    or time.monotonic() - self._joint_captured_at > JOINT_MAX_AGE_S):
                reason = self._errors.get('joints')
                raise RuntimeError('measured joint feedback is missing or stale' +
                                   (f': {reason}' if reason else ''))
            return ([j['q'] for j in self._data['joints']['joints']],
                    self._joint_captured_at, self._joint_capture_ns)

    def snapshot(self, card):
        connected, generation = self._session()
        with self._lock:
            now = time.monotonic()
            valid = (connected and card in self._data and now - self._seen.get(card, 0) <= 0.5)
            reason = None if valid else (self._errors.get(card) or
                ('missing_or_stale_measurement' if card == 'loco_state' else
                 self._discovery_error or 'missing_or_stale_measurement'))
            if card == 'joints' and not self._names:
                valid, reason = False, self._model_error
            if card == 'joints' and (not self._identity_fresh() or self._joint_captured_at is None
                                    or now - self._joint_captured_at > JOINT_MAX_AGE_S):
                valid, reason = False, reason or 'stale_capture_or_runtime_identity'
            payload = copy.deepcopy(self._data[card]) if valid else {}
            if card == 'loco_state':
                window = [v for at, v in self._odom_window if now - at <= 0.5] if valid else []
                if valid and not window:
                    valid, reason = False, 'no_new_odometry_samples'
                payload = build_sample(stamp_ms=payload.get('stamp_ms', int(time.time()*1000)),
                    twist=mean_twist(window), pose=payload.get('pose'),
                    vendor={**payload.get('vendor', {}), 'samples': len(window)})
                payload['motion_model'] = 'planar'
                capture = payload['vendor'].get('motus_slam', {}).get('stamp_ns')
                gravity = self._body_gravity
                if (valid and gravity and type(capture) is int and
                        abs(capture-gravity['stamp_ns']) <= 50_000_000):
                    payload['vendor']['body_gravity'] = dict(gravity)
            elif card == 'joints' and not valid:
                payload['joints'] = []
            return {**payload, 'valid': bool(valid), 'reason': reason,
                    'connected': connected, 'connection_generation': generation}

    def tick(self):
        connected, _ = self._session()
        with self._lock:
            if self._closed.is_set():
                return
            if connected and not self._discovering and time.monotonic() >= self._retry_at:
                self._discovering = True
                self._thread = threading.Thread(target=self.discover, daemon=True, name='kuavo-discovery')
                self._thread.start()
            for card in self._running:
                payload = self.snapshot(card)
                if card == 'loco_state':
                    self._odom_window.clear()
                message = String()
                message.data = json.dumps(payload, separators=(',', ':'), allow_nan=False)
                self._pubs[card].publish(message)
                if card == 'joints':
                    raw = String()
                    raw.data = json.dumps(self.joint_raw_snapshot(), allow_nan=False, separators=(',', ':'))
                    self._joint_state_pub.publish(raw)
                elif card == 'loco_state':
                    display = String()
                    display.data = json.dumps(self.loco_display(), allow_nan=False, separators=(',', ':'))
                    self._loco_pub.publish(display)

    def tools(self):
        descriptions = {'joints': '全身控制器实测20关节；控制器未启动时 invalid，参考骨架不冒充实际姿态',
                        'imu': '首路独立 /imu 展示；camera_imu计算口输出相机原始gyro/accel、采集纳秒时间和SDK内部旋转/传感器原点，供RGB-D辅助。缺少真实数据时明确unavailable',
                        'loco_state': '位置/姿态/速度展示 + 独立 actucore navi motus.odom/1 里程计；区分 map 定位与 odom'}
        tools = [{'name': name, 'type': 'sensor', 'multiInstance': False,
                  'description': description, 'inputSchema': {'type': 'object', 'properties': {}},
                  'topic_out': [{'topic': self._topics[name],
                                 'format': {'joints': 'sensor/skeleton', 'loco_state': 'state/odom'}.get(name, 'data/json')}]}
                 for name, description in descriptions.items()]
        for tool in tools:
            if tool['name'] == 'joints':
                tool['topic_out'].append({'topic': self._joint_state_topic, 'format': 'data/json',
                                          'desc': 'joint_state 原始关节数组，不依赖显示模型校验'})
            elif tool['name'] == 'imu':
                tool['topic_out'].append({'name': 'camera_imu', 'topic': self._camera_imu_topic,
                    'format': 'sensor/imu', 'desc': 'motus.imu/1 原始相机陀螺仪/加速度及SDK内部外参；用于RGB-D里程计，非首路底盘IMU展示'})
            elif tool['name'] == 'loco_state':
                tool['topic_out'].insert(0, {'topic': self._loco_topic, 'format': 'data/json',
                                            'desc': '位置、角度和速度；frame与来源明确标注'})
        tools.append({'name': 'model', 'type': 'resource',
                      'description': 'Kuavo 5-W 本地参考 URDF 骨架模型；无需机器人在线，实测姿态由 joints 单独提供',
                      'inputSchema': {'type': 'object', 'properties': {}}})
        return tools

    def dispatch(self, card, action):
        self._session()
        with self._lock:
            if card == 'model':
                if action in ('start', 'stop', 'info'):
                    if action != 'info':
                        self._model_running = action == 'start'
                    return {'state': 'running' if self._model_running else 'idle'}
                try:
                    urdf = self._model_path.read_text(encoding='utf-8')
                    model_names(urdf)
                    return {'urdf': urdf}
                except (OSError, ValueError, ET.ParseError) as exc:
                    return {'state': 'error', 'error': f'local skeleton URDF unavailable: {exc}'}
            if action == 'start':
                self._running.add(card)
            elif action == 'stop':
                self._running.discard(card)
            elif action != 'info':
                return {'state': 'error', 'error': f'unknown action: {action}'}
            result = {'state': 'running' if card in self._running else 'idle',
                      'feedback': self.loco_display() if card == 'loco_state' else self.snapshot(card)}
            if card == 'joints':
                result['skeleton_model'] = self.health()['skeleton_model']
            if card == 'imu':
                result['imu_status'] = self.camera_imu_status()
            if card in self._running or card in ('imu', 'loco_state'):
                result['topic_out'] = next(t['topic_out'] for t in self.tools() if t['name'] == card)
            if card == 'loco_state':
                result['odom_interface'] = build_interface(provides=['vx', 'vy', 'wz'], rate_hz=10, pose_drift='unbounded')
                result['odom_interface']['topic'] = self._topics['loco_state']
                result['odom_interface']['timing'] = {
                    'stamp_source': 'received', 'clock_id': 'driver_host_wall',
                    'timestamp_semantics': 'rosbridge_callback_receipt',
                    'measurement_delay_bound': 'unverified'}
                result['odom_interface']['slam_pose'] = {'available': True, 'profile': 'motus_slam/1',
                    'source': '/odom', 'clock_id': 'ros1_odom_header', 'synchronization': 'unverified'}
            return result

    def close(self):
        self._closed.set()
        with self._lock:
            self._running.clear()
            self._model_running = False
