"""Offline validated two-second movement tests. No ROS, SSH, or robot I/O."""
import copy
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
            result['data'] = {'pose': {'pose': {
                'position': {'x': self.x, 'y': self.y, 'z': 0.},
                'orientation': {'x': 0., 'y': 0., 'z': 0., 'w': 1.}}},
                'twist': {'twist': {'linear': {'x': self.vx, 'y': 0., 'z': 0.}}}}
        else:
            result['data'] = {'data': {'motion_state': {'state': self.state},
                'gait_state': {'gait': 0}, 'vel_x': self.vx, 'vel_y': 0., 'vel_yaw': 0.}}
        return result


class Authorization:
    def __init__(self):
        self.valid = True
        self.allowed = ['stand', 'lie', 'move_mode', 'forward_30cm', 'backward_30cm']

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
        assert self.running and not self.closed and vy == 0 and yaw_rate == 0
        self.velocities.append((vx, copy.deepcopy(authority)))
        if not self.suppress_feedback or vx == 0:
            self.bridge.vx = vx
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
    assert result['commanded_speed_mps'] == .08
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


def test_new_card_has_one_move_action_with_two_direction_options(runtime):
    import jsonschema
    tools = {tool['name']: tool for tool in runtime[0].get_tools()}
    assert set(tools) == {'motion', 'timed_move'}
    assert 'move_2s' not in tools['motion']['inputSchema']['properties']['action']['enum']
    schema = tools['timed_move']['inputSchema']
    jsonschema.Draft7Validator.check_schema(schema)
    assert schema['x-resource'] == 'base'
    assert set(schema['properties']) == {'direction'}
    assert schema['properties']['direction']['enum'] == ['forward', 'backward']
    jsonschema.validate({'direction': 'backward'}, schema)
    for invalid in (
        {},
        {'direction': 'left'},
        {'direction': 'forward', 'distance_m': .3},
        {'direction': 'forward', 'vx': .1},
        {'action': 'stand', 'direction': 'forward'},
    ):
        with pytest.raises(jsonschema.ValidationError):
            jsonschema.validate(invalid, schema)


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
