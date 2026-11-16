"""Measured ROS1 telemetry and read-only robot discovery for Kuavo 5-W.

See SDK_ANALYSIS.md for the pinned vendor sources. No SDK default state, MPC
targets, or synthetic joint names are used as measurements.
"""
from __future__ import annotations

from collections import deque
import copy
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
        self._model_error = 'waiting_for_discovery'
        self._model_path = Path(__file__).resolve().parent / 'resource' / 'kuavo5w_model.urdf'
        self._discovery_error = 'waiting_for_discovery'
        self._discovering = False
        self._retry_at = 0
        self._thread = None
        self._raw_topic = None
        self._data = {}
        self._seen = {}
        self._errors = {}
        self._odom_window = deque(maxlen=2048)
        self._running = set()
        self._body_frames = set(config.get('odom_body_frames', ['base_link', 'base_footprint']))
        self._torque_unit = config.get('joint_torque_unit')
        if self._torque_unit not in (None, 'N*m'):
            raise ValueError('joint_torque_unit must be null or hardware-verified N*m')
        self._topics = {'joints': f'/{namespace}/state/joints',
                        'imu': f'/{namespace}/state/imu',
                        'loco_state': f'/{namespace}/state/odom'}
        self._pubs = {key: node.create_publisher(String, topic, 10)
                      for key, topic in self._topics.items()}
        ros1.subscribe('/odom', 'nav_msgs/Odometry', self._on_odom)
        self._timer = node.create_timer(0.1, self.tick)

    def _session(self):
        connected, generation = self.ros1.connection
        with self._lock:
            if not connected or generation != self._generation:
                self._profile = self._urdf = self._names = None
                self._data.clear()
                self._seen.clear()
                self._errors.clear()
                self._odom_window.clear()
                self._generation = generation
                self._discovery_error = 'disconnected' if not connected else 'waiting_for_discovery'
                self._model_error = self._discovery_error
        return connected, generation

    def discover(self):
        """Run off the WebSocket receiver; rosapi replies need that receiver."""
        connected, generation = self._session()
        if not connected:
            return
        try:
            params = {name: self.ros1.get_param('/' + name) for name in (
                'robot_type', 'robot_version', 'armRealDof', 'headRealDof',
                'legRealDof', 'waistRealDof')}
            params['use_shm_communication'] = self.ros1.get_param('/use_shm_communication', False)
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
                        self._data.pop(card, None)
                        self._seen.pop(card, None)
                        self._errors.pop(card, None)
                if self._names != names:
                    self._data.pop('joints', None)
                    self._seen.pop('joints', None)
                self.ros1.subscribe(topic, 'kuavo_msgs/sensorsData',
                    lambda msg, source=topic, session=generation: self._on_raw(msg, source, session))
                self._raw_topic = topic
                self._profile = params
                self._urdf, self._names, self._model_error = urdf, names, model_error
                self._discovery_error = None
        except Exception as exc:
            with self._lock:
                if self.ros1.connection == (True, generation):
                    self._profile = self._urdf = self._names = None
                    for card in ('joints', 'imu'):
                        self._data.pop(card, None)
                        self._seen.pop(card, None)
                        self._errors.pop(card, None)
                    self._discovery_error = str(exc)
                    self._model_error = str(exc)
        finally:
            with self._lock:
                self._discovering = False
                self._retry_at = time.monotonic() + 5

    def require_profile(self):
        connected, _ = self._session()
        with self._lock:
            if not connected or self._profile is None:
                raise RuntimeError(f'robot discovery is not ready: {self._discovery_error}')

    def health(self):
        self._session()
        with self._lock:
            return {'profile': copy.deepcopy(self._profile), 'sensor_topic': self._raw_topic,
                    'discovery_error': self._discovery_error, 'model_error': self._model_error,
                    'skeleton_model': {'source': 'resource/kuavo5w_model.urdf',
                                       'reference_version': '400062',
                                       'runtime_joint_names_verified': self._names is not None}}

    def _on_raw(self, message, source, generation):
        if not self._session()[0]:
            return
        with self._lock:
            if (self._profile is None or source != self._raw_topic or
                    generation != self._generation):
                return
            stamp_ms, provenance = stamp(message)
            meta = {'stamp_ms': stamp_ms, 'vendor': {'topic': self._raw_topic, **provenance}}
            try:
                raw = message['joint_data']
                q = vector(raw.get('joint_q'), 20)
                # Missing optional measurements stay null, never fabricated zeros.
                arrays = {}
                for key, source in (('dq', 'joint_v'), ('ddq', 'joint_vd'), ('tau', 'joint_torque')):
                    try:
                        arrays[key] = vector(raw.get(source), 20)
                    except ValueError:
                        arrays[key] = [None] * 20
                # The SDK data class permits N*m OR Amperes. Preserve raw effort,
                # but do not label a possible current as physical joint torque.
                raw_effort = arrays['tau']
                if self._torque_unit is None:
                    arrays['tau'] = [None] * 20
                joints = [{'idx': i, 'name': self._names[i] if self._names else None, 'q': v,
                           **{key: values[i] for key, values in arrays.items()}} for i, v in enumerate(q)]
                self._data['joints'] = {**meta, 'joints': joints,
                    'vendor': {**meta['vendor'], 'joint_torque': raw_effort,
                               'joint_torque_unit': self._torque_unit},
                    'units': {'q': 'rad', 'dq': 'rad/s', 'ddq': 'rad/s^2', 'tau': self._torque_unit}}
                self._seen['joints'] = time.monotonic()
                self._errors.pop('joints', None)
            except (KeyError, TypeError, ValueError, AttributeError) as exc:
                self._data.pop('joints', None)
                self._seen.pop('joints', None)
                self._errors['joints'] = str(exc)
            try:
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
            if time.monotonic() - seen > 0.5 or 'joints' not in self._data:
                raise RuntimeError('measured joint feedback is missing or stale')
            return [j['q'] for j in self._data['joints']['joints']], seen

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
            if connected and (self._profile is None or self._names is None) and not self._discovering and time.monotonic() >= self._retry_at:
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

    def tools(self):
        descriptions = {'joints': '实测下肢4+双臂14+头部2关节，URDF 精确名称；无轮速/手指伪造值',
                        'imu': '实测 IMU 四元数、角速度、加速度、去重力加速度',
                        'loco_state': '实测底盘本体系 vx/vy/wz，motus.odom/1；未测轴为 null'}
        tools = [{'name': name, 'type': 'sensor', 'multiInstance': False,
                  'description': description, 'inputSchema': {'type': 'object', 'properties': {}},
                  'topic_out': [{'topic': self._topics[name],
                                 'format': {'joints': 'sensor/skeleton', 'loco_state': 'state/odom'}.get(name, 'data/json')}]}
                 for name, description in descriptions.items()]
        tools.append({'name': 'model', 'type': 'resource',
                      'description': 'Kuavo 5-W 本地参考 URDF 骨架模型；无需机器人在线，实测姿态由 joints 单独提供',
                      'inputSchema': {'type': 'object', 'properties': {}}})
        return tools

    def dispatch(self, card, action):
        self._session()
        with self._lock:
            if card == 'model':
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
                      'feedback': self.snapshot(card)}
            if card in self._running:
                result['topic_out'] = next(t['topic_out'] for t in self.tools() if t['name'] == card)
            if card == 'loco_state':
                result['odom_interface'] = build_interface(provides=['vx', 'vy', 'wz'], rate_hz=10)
            return result

    def close(self):
        self._closed.set()
        with self._lock:
            self._running.clear()
