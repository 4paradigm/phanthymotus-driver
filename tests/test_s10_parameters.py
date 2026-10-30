"""Shared schema/runtime boundaries and side-effect-free request rejection."""
import json
import re

import pytest

from deep_robotics.s10_pro.device import ReadOnlyCard
from deep_robotics.s10_pro import parameters as p


class Bridge:
    def __init__(self):
        self.commands = []

    def acquire(self, *args):
        self.commands.append(('acquire', args))

    def release(self, *args):
        self.commands.append(('release', args))

    def snapshot(self, source):
        return {}

    def status(self):
        return {}


def card(name='lidar'):
    bridge, published = Bridge(), []
    value = ReadOnlyCard(name, {}, 'same_namespace', bridge, lambda *args: published.append(args))
    return value, bridge, published


@pytest.mark.parametrize('name,hz,title', [('state', 1, 'Robot State'), ('joints', 10, 'Joint Feedback'),
    ('imu', 20, 'IMU'), ('odometry', 5, 'Odometry'), ('lidar', 2, 'LiDAR Point Cloud')])
def test_sensor_schema_defaults_and_visible_names_match_runtime(name, hz, title):
    value, bridge, _ = card(name)
    tool = value.get_tool()
    assert tool['name'] == name and tool['title'] == f'S10 Pro · {title}'
    if name in ('joints', 'imu'):
        # These public monitors intentionally use the standard G1 renderer and
        # empty action schema; their backend lifecycle remains independently tested.
        assert tool['inputSchema'] == {'type': 'object', 'properties': {}}
        assert tool['topic_in'] == []
        assert tool['topic_out'] == value._ports()
        assert value.status()['settings']['refresh_hz'] == hz
        assert bridge.commands == []
        return
    assert tool['topic_out'][0]['topic'] == f'/same_namespace/s10_pro/{name}'
    assert value.status()['settings']['refresh_hz'] == hz
    for key, schema in tool['configSchema']['properties'].items():
        assert schema['default'] == value.status()['settings'][key]
        assert schema['description']
    assert 'default' not in tool['inputSchema']['properties']['action']
    assert not re.search(r'[\u4e00-\u9fff]', json.dumps(tool, ensure_ascii=False))
    assert bridge.commands == []


@pytest.mark.parametrize('key,boundaries', [('refresh_hz', (.1, 30.)), ('stale_after_s', (.1, 300.)),
                                           ('max_points', (100, 40000))])
def test_sensor_schema_ranges_accept_both_exact_endpoints(key, boundaries):
    value, bridge, _ = card()
    schema = value.get_tool()['configSchema']['properties'][key]
    assert (schema['minimum'], schema['maximum']) == boundaries
    for boundary in boundaries:
        assert value.configure({key: boundary})['settings'][key] == boundary
        assert value.get_tool()['inputSchema']['properties'][key]['default'] == boundary
    assert bridge.commands == []


@pytest.mark.parametrize('updates', [
    {'unknown': 2}, {'enabled': None}, {'enabled': ''}, {'enabled': 1}, {'refresh_hz': None},
    {'refresh_hz': ''}, {'refresh_hz': True}, {'refresh_hz': float('nan')},
    {'refresh_hz': float('inf')}, {'refresh_hz': 10 ** 400}, {'refresh_hz': .099}, {'refresh_hz': 30.001},
    {'stale_after_s': 0}, {'stale_after_s': 301}, {'max_points': 99}, {'max_points': 40001},
    {'max_points': 6000.0}, {'max_points': False}, {'config': None}, {'config': ''}, {'config': []},
    {'refresh_hz': 2, 'config': {'refresh_hz': 3}}, {'config': {'unknown': 1}},
    {'instance_id': ''}, {'_tool_name': None},
])
@pytest.mark.parametrize('operation', ['configure', 'start'])
def test_invalid_sensor_requests_reject_before_mutation_or_reader_lease(updates, operation):
    value, bridge, published = card()
    before = value.status()['settings']
    with pytest.raises(ValueError) as caught:
        getattr(value, operation)(updates)
    assert not re.search(r'[\u4e00-\u9fff]', str(caught.value))
    assert value.status()['settings'] == before
    assert value.status()['state'] == 'idle'
    assert bridge.commands == [] and published == []


def test_sensor_omission_preserves_current_configuration_and_is_reflected_in_both_schemas():
    value, bridge, _ = card()
    value.configure({'refresh_hz': 4, 'max_points': 1500})
    configured = value.configure({'config': {'stale_after_s': 7}, 'instance_id': 'card-one'})
    assert configured['settings'] == {'enabled': True, 'refresh_hz': 4., 'stale_after_s': 7., 'max_points': 1500}
    tool = value.get_tool()
    for schema in (tool['configSchema'], tool['inputSchema'], tool['inputSchema']['properties']['config']):
        assert schema['properties']['max_points']['default'] == 1500
        assert schema['properties']['refresh_hz']['default'] == 4
    assert bridge.commands == []


@pytest.mark.parametrize('action', ['info', 'status', 'stop'])
@pytest.mark.parametrize('arguments', [{'refresh_hz': 2}, {'unknown': 'x'}, {'config': {}}, {'instance_id': None}])
def test_sensor_read_operations_reject_unused_business_arguments(action, arguments):
    value, bridge, _ = card()
    with pytest.raises(ValueError):
        value.dispatch(action, arguments)
    assert bridge.commands == []


def test_sensor_error_is_stable_english_with_original_diagnostics_preserved():
    value, bridge, _ = card()
    def fail(*args):
        raise RuntimeError('设备暂不可用')
    bridge.acquire = fail
    result = value.start()
    assert result['last_error'] == 'Sensor reader unavailable.'
    assert result['diagnostic_error'] == '设备暂不可用'
    assert result['state'] == 'error'


def test_motion_defaults_and_current_limits_are_shared_by_request_validation_and_schemas():
    settings = p.validate_motion_settings({})
    assert settings == {'enabled': False, 'refresh_hz': 2., 'max_duration_s': 2.,
        'action_timeout_s': 8., 'max_vx': .2, 'max_vy': .1, 'max_yaw_rate': .2}
    schema = p.motion_action_schema(settings)
    assert schema['properties']['duration']['maximum'] == 2
    request = p.validate_motion_request('velocity', {}, settings, 1000)
    assert request['duration'] == 1 and request['vx'] == request['vy'] == request['yaw_rate'] == 0
    assert isinstance(request['issued_at_ms'], int) and request['issued_at_ms'] == 1000000
    assert request['ttl_ms'] == 1000 and len(request['request_id']) == 32
    for name, limit in [('vx', .2), ('vy', .1), ('yaw_rate', .2)]:
        assert schema['properties'][name]['minimum'] == -limit
        assert schema['properties'][name]['maximum'] == limit
        for boundary in [-limit, limit]:
            assert p.validate_motion_request('velocity', {name: boundary}, settings, 1000)[name] == boundary
        with pytest.raises(ValueError):
            p.validate_motion_request('velocity', {name: limit + .0001}, settings, 1000)
    assert 'default' not in schema['properties']['action']


def test_motion_configuration_changes_update_velocity_defaults_and_limits_immediately():
    settings = p.validate_motion_settings({'max_vx': .12, 'max_vy': .04, 'max_yaw_rate': .1, 'max_duration_s': .3})
    schema = p.motion_action_schema(settings)['properties']
    request = p.validate_motion_request('velocity', {}, settings, 1000)
    assert request['duration'] == schema['duration']['default'] == schema['duration']['maximum'] == .3
    assert schema['vx']['maximum'] == .12 and schema['vy']['maximum'] == .04 and schema['yaw_rate']['maximum'] == .1
    config = p.motion_config_schema(settings)
    assert config['properties']['max_vx']['default'] == .12
    with pytest.raises(ValueError):
        p.validate_motion_request('velocity', {'vx': .13}, settings, 1000)


@pytest.mark.parametrize('settings', [{'enabled': None}, {'enabled': 1}, {'unknown': 2},
    {'refresh_hz': False}, {'max_duration_s': ''}, {'max_duration_s': None},
    {'max_vx': float('nan')}, {'max_vy': float('inf')}, {'max_yaw_rate': 1.0001}, {'max_duration_s': 5.1}])
def test_invalid_motion_settings_are_rejected(settings):
    with pytest.raises(ValueError):
        p.validate_motion_settings(settings)


@pytest.mark.parametrize('arguments', [{'vx': None}, {'vx': ''}, {'vx': False}, {'vx': float('nan')},
    {'vy': float('inf')}, {'yaw_rate': -.201}, {'duration': 2.001}, {'duration': .099},
    {'issued_at_ms': 999999.5}, {'issued_at_ms': 1000000.0}, {'issued_at_ms': True},
    {'issued_at_ms': None}, {'issued_at_ms': 10 ** 400}, {'ttl_ms': 100.5}, {'ttl_ms': 1000.0}, {'ttl_ms': False},
    {'ttl_ms': 99}, {'ttl_ms': 2001}, {'ttl_ms': ''}, {'request_id': ''}, {'request_id': ' '},
    {'request_id': 'x' * 129}, {'request_id': None}, {'unknown': 0}, {'instance_id': ''}])
def test_invalid_motion_arguments_are_rejected(arguments):
    with pytest.raises(ValueError):
        p.validate_motion_request('velocity', arguments, {}, 1000)


@pytest.mark.parametrize('action', ['stand', 'lie', 'stopmotion'])
def test_posture_and_stop_have_no_default_executable_target_or_velocity_parameters(action):
    request = p.validate_motion_request(action, {}, {}, 1000)
    assert request['action'] == action and request['duration'] == 0
    for key in ('vx', 'vy', 'yaw_rate', 'duration', 'config', 'enabled'):
        with pytest.raises(ValueError):
            p.validate_motion_request(action, {key: 0}, {}, 1000)


def test_motion_request_freshness_exact_boundaries_and_replay_still_validate_format():
    args = {'request_id': 'one', 'issued_at_ms': 999000, 'ttl_ms': 1000}
    assert p.validate_motion_request('stand', args, {}, 1000)['request_id'] == 'one'
    with pytest.raises(ValueError):
        p.validate_motion_request('stand', args, {}, 1000.001)
    assert p.validate_motion_request('stand', args, {}, 1002, check_freshness=False)['request_id'] == 'one'
    with pytest.raises(ValueError):
        p.validate_motion_request('stand', {**args, 'ttl_ms': 1000.5}, {}, 1002, check_freshness=False)
    assert p.validate_motion_request('stand', {'issued_at_ms': 1000100}, {}, 1000)
    with pytest.raises(ValueError):
        p.validate_motion_request('stand', {'issued_at_ms': 1000101}, {}, 1000)


def test_nested_configuration_overlap_and_unknowns_have_no_silent_precedence():
    assert p.normalize_config_updates({'config': {'max_vx': .1}, 'max_vy': .05}, p.motion_defaults()) == {'max_vx': .1, 'max_vy': .05}
    with pytest.raises(ValueError, match='both directly and in config'):
        p.normalize_config_updates({'config': {'max_vx': .1}, 'max_vx': .1}, p.motion_defaults())
    with pytest.raises(ValueError, match='Unknown'):
        p.normalize_config_updates({'config': {'not_supported': 0}}, p.motion_defaults())
