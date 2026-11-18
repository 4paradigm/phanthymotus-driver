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
import xml.etree.ElementTree as ET

from std_msgs.msg import String
from common.odom import build_interface, build_sample, mean_twist, resolve_stamp_ms


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
        self._joint_limit_config = copy.deepcopy(config.get('joint_limits'))
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
        self._display_pose = {}
        self._pose_seen = None
        self._running = set()
        self._model_running = False  # Resource lifecycle, never a telemetry stream.
        self._body_frames = set(config.get('odom_body_frames', ['base_link', 'base_footprint']))
        self._torque_unit = config.get('joint_torque_unit')
        self._imu_topic = config.get('imu_topic', '/imu')
        self._standard_imu_at = None
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
        ros1.subscribe(self._imu_topic, 'sensor_msgs/Imu', self._on_standard_imu)
        self._timer = node.create_timer(0.1, self.tick)

    def _session(self):
        connected, generation = self.ros1.connection
        with self._lock:
            if not connected or generation != self._generation:
                self._profile = self._urdf = self._names = None
                self._model_sha256 = None
                self._identity_checked_at = None
                self._standard_imu_at = None
                self._joint_capture_ns = self._joint_captured_at = None
                self._data.clear()
                self._seen.clear()
                self._errors.clear()
                self._odom_window.clear()
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
        try:
            params = {name: self.ros1.get_param('/' + name) for name in (
                'robot_type', 'robot_version', 'armRealDof', 'headRealDof',
                'legRealDof', 'waistRealDof')}
            params['use_shm_communication'] = self.ros1.get_param('/use_shm_communication', False)
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
                urdf = self.ros1.get_param('/humanoid_description')
                names = model_names(urdf)
                if names != model_names(self._model_path.read_text(encoding='utf-8')):
                    raise ValueError('runtime joint names differ from resource/kuavo5w_model.urdf; update the reference model')
                model_error = None
            except Exception as exc:
                urdf, names, model_error = None, None, str(exc)
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
                # Lease starts before parameter reads, not when slow reads finish.
                self._identity_checked_at = checked_at
                self._discovery_error = None
        except Exception as exc:
            with self._lock:
                if self.ros1.connection == (True, generation):
                    self._profile = self._urdf = self._names = None
                    self._model_sha256 = None
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
                self._retry_at = time.monotonic() + IDENTITY_REFRESH_S

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
                    limits[card] = {'available': True, 'verification': self._joint_limit_config['verification']}
                except (RuntimeError, ValueError, TypeError):
                    limits[card] = {'available': False}
            return {'profile': copy.deepcopy(self._profile), 'sensor_topic': self._raw_topic,
                    'identity_verification': {'valid': self._identity_fresh(),
                        'age_s': None if self._identity_checked_at is None else
                                 max(0, time.monotonic() - self._identity_checked_at)},
                    'joint_limits': limits,
                    'discovery_error': self._discovery_error, 'model_error': self._model_error,
                    'skeleton_model': {'source': 'resource/kuavo5w_model.urdf',
                                       'reference_version': '63',
                                       'reference_only': True, 'hardware_geometry_verified': False,
                                       'configured_robot_serial': getattr(self, 'robot_serial', None),
                                       'serial_source': 'operator_config_not_hardware_verification',
                                       'runtime_joint_names_verified': self._names is not None}}

    def _joint_bounds(self, card):
        """Caller holds _lock. Configuration is an operator commissioning record,
        never inferred from the display URDF or from joint-name agreement alone.
        """
        policy = self._joint_limit_config
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

    def _store_joints(self, message, meta):
        # sensor_time is the vendor capture time. Receipt time/header sequence
        # cannot turn a retransmitted capture into independent evidence.
        capture = message.get('sensor_time')
        if not isinstance(capture, dict):
            raise ValueError('joint sensor_time is missing')
        secs, nsecs = capture.get('secs'), capture.get('nsecs')
        if (type(secs) is not int or type(nsecs) is not int or secs < 0
                or not 0 <= nsecs < 1_000_000_000 or secs + nsecs == 0):
            raise ValueError('joint sensor_time is invalid')
        capture_ns = secs * 1_000_000_000 + nsecs
        now, wall = time.monotonic(), time.time()
        age = wall - capture_ns / 1e9
        if not -JOINT_FUTURE_TOLERANCE_S <= age <= JOINT_MAX_AGE_S:
            raise ValueError('joint capture is stale or clocks are not synchronized: '
                             f'capture_age_s={age:.3f}, allowed_s='
                             f'[-{JOINT_FUTURE_TOLERANCE_S}, {JOINT_MAX_AGE_S}]')
        if self._joint_capture_ns is not None and capture_ns <= self._joint_capture_ns:
            return  # No data/freshness update for repeated or reordered captures.
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
        self._data['joints'] = {**meta, 'joints': joints,
            'vendor': {**meta['vendor'], 'sensor_time': dict(capture), 'joint_torque': raw_effort,
                       'joint_torque_unit': self._torque_unit},
            'units': {'q': 'rad', 'dq': 'rad/s', 'ddq': 'rad/s^2', 'tau': self._torque_unit}}
        self._joint_capture_ns = capture_ns
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
                self._standard_imu_at = self._seen['imu'] = time.monotonic()
                self._errors.pop('imu', None)
            except (KeyError, TypeError, ValueError, AttributeError) as exc:
                self._data.pop('imu', None)
                self._seen.pop('imu', None)
                self._errors['imu'] = str(exc)

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
                stamp_ms, provenance = stamp(message)
                self._data['loco_state'] = {'stamp_ms': stamp_ms, 'vendor': {
                    **provenance, 'topic': '/odom', 'header': copy.deepcopy(message.get('header', {})),
                    'child_frame_id': message['child_frame_id'],
                    'pose': json_safe(message.get('pose')), 'twist': json_safe(message['twist'])}}
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
                raise RuntimeError('measured joint feedback is missing or stale')
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
                    twist=mean_twist(window), vendor={**payload.get('vendor', {}), 'samples': len(window)})
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
                        'imu': '独立 /imu 实测姿态/角速度/加速度，不依赖全身控制器；不将其套到骨架根节点',
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
            if card in self._running:
                result['topic_out'] = next(t['topic_out'] for t in self.tools() if t['name'] == card)
            if card == 'loco_state':
                result['odom_interface'] = build_interface(provides=['vx', 'vy', 'wz'], rate_hz=10)
            return result

    def close(self):
        self._closed.set()
        with self._lock:
            self._running.clear()
            self._model_running = False
