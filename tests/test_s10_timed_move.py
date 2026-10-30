"""Offline validated two-second movement tests. No ROS, SSH, or robot I/O."""
import copy
import math
import time

import pytest

from deep_robotics.s10_pro.canvas_posture import CanvasPostureBackend, CanvasPosturePlugin


class Bridge:
    def __init__(self):
        self.ids = 100
        self.status_word = 1
        self.faults = []
        self.hes = 0
        self.state = 17
        self.vx = 0.
        self.yaw_rate = 0.
        self.vy = 0.
        self.yaw = 0.
        self.x = self.y = 0.
        self.seq = 0
        self.owners = {}

    def acquire(self, owner, sources, options):
        self.owners[owner] = set(sources)

    def release(self, owner):
        self.owners.pop(owner, None)

    def snapshot(self, source):
        self.seq += 1
        result = {'fresh': True, 'error': None, 'age_s': 0.,
                  'received_monotonic': time.monotonic(), 'sequence': self.seq}
        if source == 'joints':
            self.ids += 1
            result['data'] = {'data': {'joints_data': [{'name': [i], 'data_id': self.ids,
                'status_word': self.status_word} for i in range(16)]}}
        elif source == 'fault_aggregator':
            result['data'] = {'active_faults': self.faults}
        elif source == 'hard_estop':
            result['data'] = {'value': self.hes}
        elif source == 'odometry':
            qz, qw = math.sin(self.yaw / 2), math.cos(self.yaw / 2)
            result['data'] = {'pose': {'pose': {
                'position': {'x': self.x, 'y': self.y, 'z': 0.},
                'orientation': {'x': 0., 'y': 0., 'z': qz, 'w': qw}}},
                'twist': {'twist': {'linear': {'x': self.vx, 'y': self.vy, 'z': 0.},
                                    'angular': {'x': 0., 'y': 0., 'z': self.yaw_rate}}}}
        else:
            result['data'] = {'data': {'motion_state': {'state': self.state},
                'gait_state': {'gait': 0}, 'vel_x': self.vx, 'vel_y': 0.,
                'vel_yaw': self.yaw_rate}}
        return result


class Authorization:
    def __init__(self):
        self.valid = True
        self.allowed = ['stand', 'lie', 'move_mode', 'forward_30cm', 'backward_30cm',
                        'turn_left', 'turn_right', 'relative_forward', 'relative_backward',
                        'relative_left', 'relative_right', 'relative_turn_left',
                        'relative_turn_right']

    def authorized_now(self):
        return self.valid

    def public_status(self):
        return {'authorized': self.valid, 'allowed_actions': list(self.allowed)}

    def permit(self, rid, action, deadline):
        assert self.valid and action in self.allowed
        return {'schema': 's10_pro.operator_permit/1', 'request_id': rid, 'action': action}


class Transport:
    def __init__(self, bridge):
        self.bridge = bridge
        self.running = self.closed = False
        self.states = []
        self.velocities = []
        self.velocity_commands = []
        self.matches = 2
        self.suppress_feedback = False

    def status(self):
        return {'worker_running': self.running, 'closed': self.closed}

    def start(self, **kwargs):
        self.running = True

    def graph_snapshot(self):
        value = {'publisher_prepared': self.running, 'matched_subscribers': self.matches}
        return {'state': dict(value), 'velocity': dict(value)}

    def close(self):
        self.closed, self.running = True, False

    def publish_state(self, state, *, authority, cancel_event):
        self.states.append((state, copy.deepcopy(authority)))
        self.bridge.state = 0 if state == 4 else state
        return {'sent': True, 'accepted': None}

    def publish_velocity(self, vx=0., vy=0., yaw_rate=0., *, authority, cancel_event):
        assert self.running and not self.closed
        self.velocities.append((vx, copy.deepcopy(authority)))
        self.velocity_commands.append((vx, vy, yaw_rate))
        if not self.suppress_feedback or (vx == 0 and yaw_rate == 0):
            self.bridge.vx = vx
            self.bridge.vy = vy
            self.bridge.yaw_rate = yaw_rate
            self.bridge.x += vx * .1
            self.bridge.y += vy * .1
            self.bridge.yaw += yaw_rate * .1
        return {'sent': True, 'accepted': None}


class Notifier:
    def submit(self, payload):
        pass

    def status(self, action_id):
        return {}


@pytest.fixture
def runtime():
    bridge, authorization = Bridge(), Authorization()
    transports = []

    def factory():
        transport = Transport(bridge)
        transports.append(transport)
        return transport

    backend = CanvasPostureBackend(bridge, factory(), authorization, transport_factory=factory)
    plugin = CanvasPosturePlugin({'enabled': True, 'action_timeout_s': 12.}, 'test', backend,
                                 notifier=Notifier())
    plugin.start()
    yield plugin, backend, bridge, authorization, transports
    plugin.close()


def wait_result(plugin, request_id):
    deadline = time.monotonic() + 5
    while time.monotonic() < deadline:
        record = plugin._tasks.get(request_id)
        if record and record['result'] is not None:
            return record['result']
        time.sleep(.01)
    raise AssertionError('timed movement task did not finish')


@pytest.mark.parametrize('direction,sign', [('forward', 1), ('backward', -1)])
def test_validated_two_second_move_repeats_zero_and_confirms_response_and_stop(
        runtime, direction, sign):
    plugin, _, bridge, _, transports = runtime
    rid = 'move-' + direction
    reply = plugin.dispatch('move_2s', {'request_id': rid, 'direction': direction})
    assert reply['accepted']
    result = wait_result(plugin, rid)
    assert result['completed'] and result['stop_confirmed']
    assert result['direction'] == direction
    assert result['commanded_speed'] == .08
    assert result['commanded_speed_unit'] == 'm/s'
    assert result['commanded_duration_s'] == 2.
    assert result['reported_peak_signed_vx_mps'] >= .05
    commands = [value for value, _ in transports[0].velocities]
    assert commands[0] * sign > 0
    assert max(abs(value) for value in commands) == .08
    assert 18 <= result['command_frames'] <= 22
    assert commands[-2:] == [0., 0.]
    assert bridge.state == 17


def test_timed_move_is_rejected_outside_move_mode_without_nav_command(runtime):
    plugin, _, bridge, _, transports = runtime
    bridge.state = 1
    reply = plugin.dispatch('move_2s', {'request_id': 'not-mode', 'direction': 'forward'})
    assert reply.get('accepted') is not True
    assert 'move_mode_required' in reply['blocked_by']
    assert transports[0].velocities == []


def test_missing_velocity_response_stops_and_fails_closed(runtime):
    plugin, backend, _, _, transports = runtime
    transports[0].suppress_feedback = True
    reply = plugin.dispatch('move_2s', {'request_id': 'no-response', 'direction': 'forward'})
    assert reply['accepted']
    result = wait_result(plugin, 'no-response')
    assert not result['completed'] and result['reason'] == 'motion_response_not_observed'
    assert result['stop_confirmed']
    assert transports[0].velocities[-2:] == [(0., transports[0].velocities[-2][1]),
                                             (0., transports[0].velocities[-1][1])]
    assert backend.safety_status()['failed_closed']


def test_new_card_has_one_move_action_with_four_direction_options(runtime):
    import jsonschema
    tools = {tool['name']: tool for tool in runtime[0].get_tools()}
    assert set(tools) == {'motion', 'timed_move', 'relative_move'}
    assert 'move_2s' not in tools['motion']['inputSchema']['properties']['action']['enum']
    schema = tools['timed_move']['inputSchema']
    jsonschema.Draft7Validator.check_schema(schema)
    assert schema['x-resource'] == 'base'
    assert set(schema['properties']) == {'direction', 'speed', 'duration'}
    assert schema['properties']['direction']['enum'] == [
        'forward', 'backward', 'turn_left', 'turn_right']
    jsonschema.validate({'direction': 'backward', 'speed': .08, 'duration': 2.}, schema)
    jsonschema.validate({'direction': 'turn_left', 'speed': .15, 'duration': 5.}, schema)
    for invalid in (
        {},
        {'direction': 'left', 'speed': .08, 'duration': 2.},
        {'direction': 'forward', 'speed': .08, 'duration': 2., 'distance_m': .3},
        {'direction': 'forward', 'speed': .08, 'duration': 2., 'vx': .1},
        {'action': 'stand', 'direction': 'forward', 'speed': .08, 'duration': 2.},
    ):
        with pytest.raises(jsonschema.ValidationError):
            jsonschema.validate(invalid, schema)
    relative = tools['relative_move']['inputSchema']
    jsonschema.Draft7Validator.check_schema(relative)
    assert relative['x-resource'] == 'base'
    assert relative['properties']['direction']['enum'] == [
        'forward', 'backward', 'left', 'right', 'turn_left', 'turn_right']
    jsonschema.validate({'direction': 'left', 'distance': .4, 'speed': .08}, relative)
    jsonschema.validate({'direction': 'turn_right', 'angle': 60, 'speed': .15}, relative)
    for invalid in (
        {'direction': 'left', 'angle': 30, 'speed': .08},
        {'direction': 'turn_left', 'distance': .4, 'speed': .15},
        {'direction': 'forward', 'distance': .8, 'speed': .08},
        {'direction': 'turn_left', 'angle': 90, 'speed': .15},
    ):
        with pytest.raises(jsonschema.ValidationError):
            jsonschema.validate(invalid, relative)


@pytest.mark.parametrize('direction', ['forward', 'backward', 'left', 'right',
                                       'turn_left', 'turn_right'])
def test_relative_schema_default_passes_backend_validation_without_motion(runtime, direction):
    plugin, _, _, _, transports = runtime
    schema = plugin._relative_move_tool()['inputSchema']
    arguments = {'direction': direction, 'speed': schema['properties']['speed']['default']}
    arguments.update(angle=30) if direction.startswith('turn_') else arguments.update(distance=.3)
    # A disabled card still validates parameters before returning unavailable.
    # This exercises the real validator without granting a task or publishing.
    plugin._started = False
    before = list(transports[0].velocities)
    reply = plugin.dispatch('relative_move', {'_tool_name': 'relative_move', **arguments})
    assert not reply['accepted']
    assert transports[0].velocities == before


def test_relative_card_reaches_odometry_target_and_stops(runtime):
    plugin, _, _, _, transports = runtime
    reply = plugin.dispatch('relative_move', {'_tool_name': 'relative_move',
        'request_id': 'relative-forward', 'direction': 'forward',
        'distance': .3, 'speed': .08})
    assert reply['accepted']
    result = wait_result(plugin, 'relative-forward')
    assert result['completed'] and result['stop_confirmed']
    assert result['completion_level'] == 'odometry_relative_target_and_reported_stop'
    assert result['closed_loop_status']['progress'] >= .22
    assert transports[0].velocity_commands[-2:] == [(0., 0., 0.), (0., 0., 0.)]


def test_new_card_routes_move_to_shared_protected_controller(runtime):
    plugin, _, _, _, _ = runtime
    reply = plugin.dispatch('timed_move', {'_tool_name': 'timed_move',
                                     'request_id': 'facade-forward',
                                     'direction': 'forward'})
    assert reply['accepted']
    result = wait_result(plugin, 'facade-forward')
    assert result['completed'] and result['action'] == 'move_2s'
    status = plugin.dispatch('info', {'_tool_name': 'timed_move'})
    assert status['card'] == 'timed_move'
    assert status['supported_actions'] == ['move']


def test_outputs_and_events_are_separate_but_motion_mutex_is_shared(runtime):
    plugin, _, _, _, _ = runtime
    published = []
    plugin._publish = lambda topic, fmt, value: published.append((topic, copy.deepcopy(value)))
    tools = {t['name']: t for t in plugin.get_tools()}
    posture_ports = {p['topic'] for p in tools['motion']['topic_out']}
    timed_ports = {p['topic'] for p in tools['timed_move']['topic_out']}
    assert posture_ports.isdisjoint(timed_ports)
    assert all('/motion_status' not in p and '/motion_events' not in p
               for p in posture_ports | timed_ports)
    plugin._event('requested', action='stand', request_id='posture-only')
    plugin._event('requested', action='move_2s', request_id='move-only')
    posture_event = next(v for t, v in published if t == plugin.events_topic)
    timed_event = next(v for t, v in published if t == plugin.timed_events_topic)
    assert posture_event['action'] == 'stand' and posture_event['card'] == 'motion'
    assert timed_event['action'] == 'move_2s' and timed_event['card'] == 'timed_move'
    plugin._publish_status(plugin.status())
    ps = next(v for t, v in reversed(published) if t == plugin.status_topic)
    ts = next(v for t, v in reversed(published) if t == plugin.timed_status_topic)
    assert set(ps['readiness']['actions']).isdisjoint(ts['readiness']['actions'])
    assert all(e['card'] == 'motion' for e in ps['events'])
    assert all(e['card'] == 'timed_move' for e in ts['events'])
    reply = plugin.dispatch('timed_move', {'_tool_name': 'timed_move',
                            'request_id': 'mutex', 'direction': 'forward'})
    assert reply['accepted']
    assert plugin.dispatch('lie', {'request_id': 'posture-during-move'})['state'] == 'busy'
    assert wait_result(plugin, 'mutex')['completed']


def test_new_card_config_can_be_saved_without_motion(runtime):
    plugin, _, _, _, transports = runtime
    result = plugin.dispatch('config', {'_tool_name': 'timed_move', 'enabled': True})
    assert result['card'] == 'timed_move'
    assert result['settings']['enabled'] is True
    assert not transports[0].states and not transports[0].velocities


def test_request_identity_includes_direction(runtime):
    plugin, _, _, _, _ = runtime
    assert plugin.dispatch('move_2s', {'request_id': 'same', 'direction': 'forward'})['accepted']
    wait_result(plugin, 'same')
    with pytest.raises(ValueError, match='different motion target'):
        plugin.dispatch('move_2s', {'request_id': 'same', 'direction': 'backward'})


@pytest.mark.parametrize('direction,sign', [('turn_left', 1), ('turn_right', -1)])
def test_timed_turn_uses_yaw_axis_and_confirms_stop(runtime, direction, sign):
    plugin, _, _, _, transports = runtime
    rid = 'turn-' + direction
    reply = plugin.dispatch('move_2s', {'request_id': rid, 'direction': direction,
                                        'speed': .15, 'duration': .5})
    assert reply['accepted']
    result = wait_result(plugin, rid)
    assert result['completed'] and result['stop_confirmed']
    assert result['commanded_speed'] == .15
    assert result['commanded_speed_unit'] == 'rad/s'
    assert result['reported_peak_signed_yaw_rad_s'] >= .04
    nonzero = [command for command in transports[0].velocity_commands if command != (0., 0., 0.)]
    assert nonzero and all(vx == 0. and vy == 0. and yaw * sign > 0
                           for vx, vy, yaw in nonzero)
    assert transports[0].velocity_commands[-2:] == [(0., 0., 0.), (0., 0., 0.)]


@pytest.mark.parametrize('arguments', [
    {'direction': 'forward', 'speed': .13, 'duration': 2.},
    {'direction': 'turn_left', 'speed': .07, 'duration': 2.},
    {'direction': 'turn_right', 'speed': .15, 'duration': 5.1},
])
def test_direction_specific_limits_reject_before_publication(runtime, arguments):
    plugin, _, _, _, transports = runtime
    with pytest.raises(ValueError):
        plugin.dispatch('move_2s', {'request_id': 'outside-range', **arguments})
    assert transports[0].velocity_commands == []


@pytest.mark.parametrize('arguments', [
    {'direction': 'sideways'},
    {'direction': True},
    {'direction': 'forward', 'distance_m': .30},
])
def test_invalid_timed_move_is_rejected_before_publication(runtime, arguments):
    plugin, _, _, _, transports = runtime
    with pytest.raises(ValueError):
        plugin.dispatch('move_2s', {'request_id': 'invalid', **arguments})
    assert transports[0].velocities == []
