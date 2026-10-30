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

META_FIELDS = {'instance_id', '_tool_name'}
MOTION_ACTIONS = ('stand_up', 'lie', 'move_2s')

_MOTION_FIELDS = {
    'enabled': {'type': 'boolean', 'default': False, 'title': 'Enable card status',
                'description': 'Enable card status only. This setting cannot authorize physical execution.'},
    'refresh_hz': {'type': 'number', 'minimum': .5, 'maximum': 10., 'default': 2.,
                   'title': 'Status publish limit (Hz)', 'description': 'Motion status publication frequency limit in Hz.'},
    'max_duration_s': {'type': 'number', 'minimum': .1, 'maximum': 5., 'default': 2.,
                       'title': 'Maximum movement duration (s)', 'description': 'Application limit for a finite velocity request, in seconds.'},
    'action_timeout_s': {'type': 'number', 'minimum': .2, 'maximum': 15., 'default': 8.,
                         'title': 'Posture confirmation timeout (s)', 'description': 'Maximum wait for fresh posture confirmation, in seconds.'},
    'max_vx': {'type': 'number', 'minimum': .01, 'maximum': 1., 'default': .2,
               'title': 'Forward speed limit (m/s)', 'description': 'Symmetric application limit on forward/backward velocity in m/s.'},
    'max_vy': {'type': 'number', 'minimum': .01, 'maximum': .15, 'default': .1,
               'title': 'Lateral speed limit (m/s)', 'description': 'Symmetric application limit on lateral velocity in m/s.'},
    'max_yaw_rate': {'type': 'number', 'minimum': .01, 'maximum': 1., 'default': .2,
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


def _strip_metadata(values):
    for name in META_FIELDS:
        if name in values:
            value = values.pop(name)
            if not isinstance(value, str) or not value.strip():
                raise ValueError(f'{name} must be a nonblank string')
    return values


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


def validate_motion_request(action, arguments, settings, wall_time=time.time, *, check_freshness=True,
                            allowed_actions=MOTION_ACTIONS):
    """Validate an action without emitting output or authorizing its execution."""
    if action not in allowed_actions:
        raise ValueError('Unsupported motion action')
    current = validate_motion_settings(settings)
    values = _object(arguments, 'Motion arguments')
    _strip_metadata(values)
    allowed = {'request_id', 'issued_at_ms', 'ttl_ms'}
    if action == 'move_2s':
        allowed |= {'direction', 'speed', 'duration'}
    _unknown(values, allowed, 'motion parameters')
    properties = {'request_id': {'type': 'string', 'minLength': 1, 'maxLength': 128, 'pattern': r'\S'},
                  'issued_at_ms': {'type': 'integer', 'minimum': 0, 'maximum': 1e15},
                  'ttl_ms': {'type': 'integer', 'minimum': 100, 'maximum': 10000}}
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
    if action == 'move_2s':
        direction = values.get('direction')
        if direction not in ('forward', 'backward', 'turn_left', 'turn_right'):
            raise ValueError('invalid finite-motion direction')
        speed = values.get('speed', .15 if direction.startswith('turn_') else .08)
        duration = values.get('duration', 2.)
        if (type(speed) not in (int, float) or not math.isfinite(speed)
                or not ((.08 <= speed <= min(1., current['max_yaw_rate'])) if direction.startswith('turn_')
                        else (.06 <= speed <= min(1., current['max_vx'])))):
            raise ValueError('speed outside finite-motion direction range')
        if type(duration) not in (int, float) or not math.isfinite(duration) or not .5 <= duration <= 5.:
            raise ValueError('duration outside finite-motion range')
        request.update(direction=direction, speed=float(speed), duration=float(duration))
    return request
