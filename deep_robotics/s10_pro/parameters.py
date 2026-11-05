"""Shared S10 Pro parameter contracts for schemas and runtime validation.

Limits here are application constraints, not manufacturer capability claims.
Omitted action values use declared defaults; omitted configuration updates keep
current values. Explicit null, empty strings and numeric booleans are invalid.
"""
from __future__ import annotations

import copy
import math

SENSOR_REFRESH_HZ = {'battery': 1., 'joints': 10., 'imu': 20., 'lidar_cloud': 10.}
META_FIELDS = {'instance_id', '_tool_name'}

_SENSOR_FIELDS = {
    'enabled': {'type': 'boolean', 'default': True, 'title': 'Enabled',
                'description': 'Enable this read-only card. Stopping the card releases only its data stream.'},
    'refresh_hz': {'type': 'number', 'minimum': .1, 'maximum': 30., 'title': 'Publish limit (Hz)',
                   'description': 'Canvas publication frequency limit in Hz; not a raw sensor sampling-rate guarantee.'},
    'stale_after_s': {'type': 'number', 'minimum': .1, 'maximum': 300., 'default': 5.,
                      'title': 'Stale after (s)', 'description': 'Maximum sensor data age in seconds.'},
    'max_points': {'type': 'integer', 'minimum': 100, 'maximum': 40000, 'default': 40000,
                   'title': 'Maximum displayed points', 'description': 'Maximum number of points in each published point cloud.'},
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
    if name in ('lidar', 'lidar_cloud'):
        result['max_points'] = 40000
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
