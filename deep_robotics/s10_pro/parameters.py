"""Shared S10 Pro parameter contracts for schemas and runtime validation.

Limits here are application constraints, not manufacturer capability claims.
Omitted action values use declared defaults; omitted configuration updates keep
current values. Explicit null, empty strings and numeric booleans are invalid.
"""
from __future__ import annotations

import copy
import math
import time
import uuid

SENSOR_TITLES = {'state': 'Robot State', 'joints': 'Joint Feedback', 'imu': 'IMU',
                 'odometry': 'Odometry', 'lidar': 'LiDAR Point Cloud',
                 'motion_info': 'Motion Status', 'device_status': 'Device Status',
                 'hard_estop': 'Hardware E-Stop', 'errors': 'Errors',
                 'motion_events': 'Motion Events', 'capabilities': 'Capabilities',
                 'ros_graph': 'ROS Graph', 'device': 'Device Controls'}
SENSOR_REFRESH_HZ = {'state': 1., 'joints': 5., 'imu': 5., 'odometry': 5., 'lidar': 2.,
                     'motion_info': 5., 'device_status': 1., 'hard_estop': 5.,
                     'errors': 1., 'motion_events': 2., 'capabilities': 1.,
                     'ros_graph': .5, 'device': 1.}
META_FIELDS = {'instance_id', '_tool_name'}
MOTION_ACTIONS = ('stand', 'lie', 'velocity', 'stopmotion')
LIFECYCLE_ACTIONS = ('info', 'start', 'stop', 'status', 'config')

_SENSOR_FIELDS = {
    'enabled': {'type': 'boolean', 'default': True, 'title': 'Enabled',
                'description': 'Enable this read-only card. Stopping the card releases only its data stream.'},
    'refresh_hz': {'type': 'number', 'minimum': .1, 'maximum': 30., 'title': 'Publish limit (Hz)',
                   'description': 'Canvas publication frequency limit in Hz; not a raw sensor sampling-rate guarantee.'},
    'stale_after_s': {'type': 'number', 'minimum': .1, 'maximum': 300., 'default': 5.,
                      'title': 'Stale after (s)', 'description': 'Maximum sensor data age in seconds. Motion feedback retains its separate 0.5 s limit.'},
    'max_points': {'type': 'integer', 'minimum': 100, 'maximum': 40000, 'default': 6000,
                   'title': 'Maximum displayed points', 'description': 'Maximum number of points in each published point cloud.'},
}
_MOTION_FIELDS = {
    'enabled': {'type': 'boolean', 'default': False, 'title': 'Enable card status',
                'description': 'Enable card status only. This setting cannot authorize physical execution.'},
    'refresh_hz': {'type': 'number', 'minimum': .5, 'maximum': 10., 'default': 2.,
                   'title': 'Status publish limit (Hz)', 'description': 'Motion status publication frequency limit in Hz.'},
    'max_duration_s': {'type': 'number', 'minimum': .1, 'maximum': 5., 'default': 2.,
                       'title': 'Maximum movement duration (s)', 'description': 'Application limit for a finite velocity request, in seconds.'},
    'action_timeout_s': {'type': 'number', 'minimum': .2, 'maximum': 15., 'default': 8.,
                         'title': 'Posture confirmation timeout (s)', 'description': 'Maximum wait for fresh posture confirmation, in seconds.'},
    'max_vx': {'type': 'number', 'minimum': .01, 'maximum': .3, 'default': .2,
               'title': 'Forward speed limit (m/s)', 'description': 'Symmetric application limit on forward/backward velocity in m/s.'},
    'max_vy': {'type': 'number', 'minimum': .01, 'maximum': .15, 'default': .1,
               'title': 'Lateral speed limit (m/s)', 'description': 'Symmetric application limit on lateral velocity in m/s.'},
    'max_yaw_rate': {'type': 'number', 'minimum': .01, 'maximum': .3, 'default': .2,
                     'title': 'Yaw rate limit (rad/s)', 'description': 'Symmetric application limit on turning rate in rad/s.'},
}
_CONFIG_DESCRIPTION = ('Omitted values preserve the current configuration. Explicit null, empty strings, '
                       'invalid types and unknown fields are rejected; values are never clamped.')


def _object(value, name):
    if not isinstance(value, dict):
        raise ValueError(f'{name} must be an object')
    if any(not isinstance(key, str) for key in value):
        raise ValueError(f'{name} field names must be strings')
    return dict(value)


def _unknown(values, allowed, name='parameters'):
    unknown = sorted(set(values) - set(allowed))
    if unknown:
        raise ValueError(f'Unknown {name}: {", ".join(unknown)}')


def _validate_value(name, value, definition):
    kind = definition['type']
    if kind == 'boolean':
        if not isinstance(value, bool):
            raise ValueError(f'{name} must be a boolean')
        return value
    try:
        finite = isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)
    except OverflowError:
        finite = False
    if not finite:
        raise ValueError(f'{name} must be a finite {"integer" if kind == "integer" else "number"}')
    if kind == 'integer' and not isinstance(value, int):
        raise ValueError(f'{name} must be an integer')
    low, high = definition['minimum'], definition['maximum']
    if not low <= value <= high:
        raise ValueError(f'{name} must be between {low} and {high}')
    return value if kind == 'integer' else float(value)


def _validated(settings, definitions, defaults):
    supplied = _object(settings, 'Settings')
    _unknown(supplied, definitions, 'settings')
    combined = {**defaults, **supplied}
    return {name: _validate_value(name, combined[name], definition) for name, definition in definitions.items()}


def _schema(definitions, settings):
    properties = copy.deepcopy(definitions)
    for name, value in settings.items():
        properties[name]['default'] = value
    return {'type': 'object', 'description': _CONFIG_DESCRIPTION, 'properties': properties,
            'additionalProperties': False}


def sensor_defaults(name):
    if name not in SENSOR_REFRESH_HZ:
        raise ValueError('Unknown sensor card')
    result = {'enabled': True, 'refresh_hz': SENSOR_REFRESH_HZ[name], 'stale_after_s': 5.}
    if name == 'lidar':
        result['max_points'] = 6000
    return result


def _sensor_definitions(name):
    defaults = sensor_defaults(name)
    return {key: copy.deepcopy(_SENSOR_FIELDS[key]) for key in defaults}


def validate_sensor_settings(name, settings):
    return _validated(settings, _sensor_definitions(name), sensor_defaults(name))


def sensor_config_schema(name, settings):
    return _schema(_sensor_definitions(name), validate_sensor_settings(name, settings))


def _strip_metadata(values):
    for name in META_FIELDS:
        if name in values:
            value = values.pop(name)
            if not isinstance(value, str) or not value.strip():
                raise ValueError(f'{name} must be a nonblank string')
    return values


def config_overlap_constraints(fields):
    """Reject direct/nested duplicates in both UI/LLM schema and runtime."""
    return [{'if': {'properties': {'config': {'type': 'object', 'required': [name]}},
                    'required': ['config']}, 'then': {'not': {'required': [name]}}}
            for name in fields]


def normalize_config_updates(options, allowed):
    """Flatten a configuration request while rejecting ambiguity and unknowns."""
    values = _object(options, 'Configuration updates')
    _strip_metadata(values)
    nested = values.pop('config', {})
    nested = _object(nested, 'config')
    overlap = sorted(set(values) & set(nested))
    if overlap:
        raise ValueError(f'Configuration fields cannot appear both directly and in config: {", ".join(overlap)}')
    values = {**nested, **values}
    _unknown(values, allowed, 'configuration fields')
    return values


def validate_empty_arguments(arguments):
    values = _object(arguments, 'Arguments')
    _unknown(values, META_FIELDS)
    _strip_metadata(values)
    return {}


def motion_defaults():
    return {name: item['default'] for name, item in _MOTION_FIELDS.items()}


def validate_motion_settings(settings):
    return _validated(settings, _MOTION_FIELDS, motion_defaults())


def motion_config_schema(settings):
    return _schema(_MOTION_FIELDS, validate_motion_settings(settings))


def motion_action_schema(settings):
    current = validate_motion_settings(settings)
    config = motion_config_schema(current)
    properties = {
        'action': {'type': 'string', 'enum': [*LIFECYCLE_ACTIONS, *MOTION_ACTIONS],
                   'description': 'Required operation. No physical motion is selected by default.'},
        'vx': {'type': 'number', 'minimum': -current['max_vx'], 'maximum': current['max_vx'],
               'default': 0., 'title': 'Forward velocity (m/s)', 'description': 'Forward/backward velocity in m/s, within the current application limit.'},
        'vy': {'type': 'number', 'minimum': -current['max_vy'], 'maximum': current['max_vy'],
               'default': 0., 'title': 'Lateral velocity (m/s)', 'description': 'Lateral velocity in m/s, within the current application limit.'},
        'yaw_rate': {'type': 'number', 'minimum': -current['max_yaw_rate'], 'maximum': current['max_yaw_rate'],
                     'default': 0., 'title': 'Yaw rate (rad/s)', 'description': 'Turning rate in rad/s, within the current application limit.'},
        'duration': {'type': 'number', 'minimum': .1, 'maximum': current['max_duration_s'],
                     'default': min(1., current['max_duration_s']), 'title': 'Movement duration (s)',
                     'description': 'Finite movement duration in seconds, followed by a stop request and fresh stop confirmation.'},
        'request_id': {'type': 'string', 'minLength': 1, 'maxLength': 128, 'pattern': r'\S',
                       'description': 'Optional nonblank idempotency key, at most 128 characters. A UUID is generated when omitted. Reuse the same ID to retry the same request.'},
        'issued_at_ms': {'type': 'integer', 'minimum': 0, 'maximum': 1000000000000000,
                         'description': 'Request creation time in Unix milliseconds. Generated as an integer at receipt when omitted; future skew over 100 ms or age beyond TTL is rejected.'},
        'ttl_ms': {'type': 'integer', 'minimum': 100, 'maximum': 2000, 'default': 1000,
                   'title': 'Request validity (ms)', 'description': 'Request acceptance lifetime in integer milliseconds; not the motion duration.'},
        **copy.deepcopy(config['properties']), 'config': config,
        '_tool_name': {'type': 'string', 'minLength': 1, 'pattern': r'\S', 'description': 'Internal tool routing metadata.'},
        'instance_id': {'type': 'string', 'minLength': 1, 'pattern': r'\S', 'description': 'Internal card instance metadata.'},
    }
    # Conditional field restrictions mirror dispatch validation while preserving
    # the flat properties and action metadata expected by the canvas editor.
    metadata = set(META_FIELDS) | {'action'}
    motion_meta = {'request_id', 'issued_at_ms', 'ttl_ms'}
    velocity = {'vx', 'vy', 'yaw_rate', 'duration'}
    configs = set(config['properties']) | {'config'}
    groups = [(['velocity'], metadata | motion_meta | velocity),
              (['stand', 'lie', 'stopmotion'], metadata | motion_meta),
              (['config', 'start'], metadata | configs),
              (['info', 'status', 'stop'], metadata)]
    conditions = []
    for actions, allowed in groups:
        forbidden = sorted(set(properties) - allowed)
        conditions.append({'if': {'properties': {'action': {'enum': actions}}, 'required': ['action']},
                           'then': {'not': {'anyOf': [{'required': [name]} for name in forbidden]}}})
    return {'type': 'object', 'properties': properties, 'required': ['action'], 'additionalProperties': False,
            'allOf': conditions + config_overlap_constraints(config['properties']), 'x-action-params': {
                'stand': {'params': [], 'description': 'Stand and wait for fresh posture confirmation.'},
                'lie': {'params': [], 'description': 'Lie down and wait for fresh posture confirmation.'},
                'velocity': {'params': ['vx', 'vy', 'yaw_rate', 'duration'],
                             'description': 'Move for a finite duration, then request zero velocity and confirm the stop.'},
                'stopmotion': {'params': [], 'description': 'Cancel the active application task and request a stop. This is not a hardware emergency stop.'}}}


def validate_motion_request(action, arguments, settings, wall_time=time.time, *, check_freshness=True,
                            allowed_actions=MOTION_ACTIONS):
    """Validate an action without emitting output or authorizing its execution."""
    if action not in allowed_actions:
        raise ValueError('Unsupported motion action')
    current = validate_motion_settings(settings)
    values = _object(arguments, 'Motion arguments')
    _strip_metadata(values)
    allowed = {'request_id', 'issued_at_ms', 'ttl_ms'}
    if action == 'velocity':
        allowed |= {'vx', 'vy', 'yaw_rate', 'duration'}
    if action == 'move_2s':
        allowed |= {'direction'}
    _unknown(values, allowed, 'motion parameters')
    properties = motion_action_schema(current)['properties']
    now_seconds = wall_time() if callable(wall_time) else wall_time
    if isinstance(now_seconds, bool) or not isinstance(now_seconds, (int, float)) or not math.isfinite(now_seconds):
        raise ValueError('Current time must be a finite number of seconds')
    now_ms = now_seconds * 1000
    issued = _validate_value('issued_at_ms', values.get('issued_at_ms', int(now_ms)), properties['issued_at_ms'])
    ttl = _validate_value('ttl_ms', values.get('ttl_ms', 1000), properties['ttl_ms'])
    if check_freshness and (now_ms - issued > ttl or issued > now_ms + 100):
        raise ValueError('Motion request is stale or too far in the future')
    rid = values.get('request_id', uuid.uuid4().hex)
    if not isinstance(rid, str) or not rid.strip() or len(rid) > 128:
        raise ValueError('request_id must be a nonblank string of at most 128 characters')
    request = {'action': action, 'request_id': rid, 'issued_at_ms': issued, 'ttl_ms': ttl, 'duration': 0.}
    if action == 'velocity':
        for name in ('vx', 'vy', 'yaw_rate', 'duration'):
            request[name] = _validate_value(name, values.get(name, properties[name]['default']), properties[name])
    if action == 'move_2s':
        direction = values.get('direction')
        if direction not in ('forward', 'backward'):
            raise ValueError('direction must be forward or backward')
        request.update(direction=direction)
    return request
