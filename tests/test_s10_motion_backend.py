"""High-level DDS sequencing with an explicitly fictional offline lease."""
import copy
import threading
import time
from types import SimpleNamespace

import pytest

from deep_robotics.s10_pro.motion_backend import DDSMotionBackend
from deep_robotics.s10_pro.motion import MotionPlugin


def policy():
    return {'verified': True, 'evidence': {key: 'offline fixture only' for key in
        ['identity', 'firmware', 'authority', 'feedback', 'stop', 'watchdog']},
        'robot_identity_matched': True, 'firmware_matched': True, 'robot_deadman_verified': True,
        'robot_deadman_s': .5, 'send_timeout_s': .2, 'velocity_units': {'vx': 'm/s', 'vy': 'm/s', 'yaw_rate': 'rad/s'},
        'actions': ['stand', 'lie', 'velocity', 'stopmotion'], 'stop_semantics': 'zero_velocity_until_feedback',
        'stop_on_authority_loss': False}


class Bridge:
    def __init__(self):
        self.active = {}
        self.value = {}
        self.sequence = 0

    def acquire(self, owner, sources, options):
        self.active[owner] = (sources, options)

    def release(self, owner):
        self.active.pop(owner, None)

    def snapshot(self, source):
        result = copy.deepcopy(self.value)
        if result:
            result['age_s'] = time.monotonic() - result['received_monotonic']
            result['fresh'] = result['age_s'] < .5
        return result

    def put(self, state, vx=0., vy=0., yaw_rate=0.):
        self.sequence += 1
        self.value = {'data': {'data': {'motion_state': {'state': state}, 'gait_state': {'gait': 0x3002},
            'vel_x': vx, 'vel_y': vy, 'vel_yaw': yaw_rate}}, 'age_s': 0., 'fresh': True,
            'received_monotonic': time.monotonic(), 'sequence': self.sequence}


class Transport:
    def __init__(self):
        self.states, self.velocities = [], []
        self.running = True
        self.closed = False

    def status(self):
        return {'worker_running': self.running}

    def publish_state(self, state, *, authority, cancel_event):
        assert not cancel_event.is_set()
        self.states.append((state, authority['request_id']))
        return {'sent': True, 'accepted': None}

    def publish_velocity(self, vx, vy, yaw_rate, *, authority, cancel_event):
        assert not cancel_event.is_set()
        self.velocities.append((vx, vy, yaw_rate, authority['request_id']))
        return {'sent': True, 'accepted': None}

    def close(self):
        self.running = False; self.closed = True


def lease():
    now = time.monotonic()
    return {'owned': True, 'fresh': True, 'age_s': 0., 'received_monotonic': now, 'navigation_mode_confirmed': True}


@pytest.fixture
def backend():
    bridge, transport = Bridge(), Transport()
    value = DDSMotionBackend(bridge, transport, contract=policy(), authority_provider=lease, execution_authorized=True)
    value.start_observation()
    yield value, bridge, transport
    value.close()


def test_default_is_passive_and_does_not_require_tls_or_infer_control():
    bridge, transport = Bridge(), Transport()
    value = DDSMotionBackend(bridge, transport)
    assert not bridge.active
    value.start_observation(); value.start_observation()
    assert list(bridge.active) == ['motion_observer']
    bridge.put(0)
    diagnostics = value.diagnostics()
    assert diagnostics['tls_required'] is False and diagnostics['fresh'] is True
    assert not value.control_status()['owned']
    assert diagnostics['control_usage_mode'] is diagnostics['hard_emergency_stop'] is None
    with pytest.raises(PermissionError):
        value.send('stand', {'request_id': 'x', 'action': 'stand', 'expires_monotonic': time.monotonic() + 3}, threading.Event())
    assert not transport.states and not transport.velocities
    value.stop_observation(); assert not bridge.active
    value.close()


def test_stand_requires_new_distinct_state_one_then_separate_seventeen(backend):
    value, bridge, transport = backend
    request = {'request_id': 'stand-1', 'action': 'stand', 'expires_monotonic': time.monotonic() + 3}
    bridge.put(1)
    result = value.send('stand', request, threading.Event())
    assert result['accepted'] is None
    assert transport.states == [(1, 'stand-1')]
    value.progress('stand', request, threading.Event())
    assert len(transport.states) == 1  # Cached state one is not post-request feedback.
    for _ in range(2):
        bridge.put(1); value.progress('stand', request, threading.Event())
    assert transport.states == [(1, 'stand-1'), (17, 'stand-1')]
    assert not value.feedback('stand', 'stand-1')['confirmed']
    for _ in range(2):
        bridge.put(17); result = value.feedback('stand', 'stand-1')
    assert result['confirmed'] and result['vendor_request_id'] is None
    assert result['correlation'] == 'local_request_marker_only'
    value.progress('stand', request, threading.Event()); assert len(transport.states) == 2


def test_cancel_between_posture_stages_never_sends_rl(backend):
    value, bridge, transport = backend; event = threading.Event()
    request = {'request_id': 'one', 'action': 'stand', 'expires_monotonic': time.monotonic() + 3}
    value.send('stand', request, event)
    bridge.put(1); value.progress('stand', request, event)
    event.set(); bridge.put(1); value.progress('stand', request, event)
    assert transport.states == [(1, 'one')]


def test_lie_is_only_state_four_with_actual_new_feedback(backend):
    value, bridge, transport = backend
    value.send('lie', {'request_id': 'one', 'action': 'lie', 'expires_monotonic': time.monotonic() + 3}, threading.Event())
    assert transport.states == [(4, 'one')]
    for _ in range(2):
        bridge.put(0); assert not value.feedback('lie', 'one')['confirmed']
    for _ in range(2):
        bridge.put(4); result = value.feedback('lie', 'one')
    assert result['confirmed']


def test_velocity_and_zero_require_navigation_attestation(backend):
    value, bridge, transport = backend
    value._authority = lambda: {**lease(), 'navigation_mode_confirmed': False}
    with pytest.raises(PermissionError):
        value.send('velocity', {'request_id': 'one', 'action': 'velocity', 'expires_monotonic': time.monotonic() + 3, 'vx': .1}, threading.Event())
    assert not transport.velocities
    value._authority = lease
    value.send('velocity', {'request_id': 'one', 'action': 'velocity', 'expires_monotonic': time.monotonic() + 3, 'vx': .1}, threading.Event())
    value._authority = lambda: {**lease(), 'navigation_mode_confirmed': False}
    with pytest.raises(PermissionError):
        value.stop('cancel', threading.Event())
    assert len(transport.velocities) == 1
    value._authority = lease
    value.stop('cancel', threading.Event())
    assert transport.velocities[-1][:3] == (0., 0., 0.)


def test_velocity_refresh_does_not_reset_feedback_marker_and_stop_does(backend):
    value, bridge, transport = backend
    request = {'request_id': 'one', 'action': 'velocity', 'expires_monotonic': time.monotonic() + 3, 'vx': .1}
    value.send('velocity', request, threading.Event())
    bridge.put(17, .1); assert not value.feedback('velocity', 'one')['confirmed']
    value.send('velocity', request, threading.Event())
    bridge.put(17, .1); assert value.feedback('velocity', 'one')['confirmed']
    value.stop('duration_elapsed', threading.Event())
    assert not value.feedback('velocity', 'one')['confirmed']
    for _ in range(2):
        bridge.put(17); result = value.feedback('stopmotion', 'one')
    assert result['confirmed']


@pytest.mark.parametrize('change', [{'owned': False}, {'fresh': False}, {'age_s': 5}, {'received_monotonic': 0}])
def test_invalid_authority_never_sends(backend, change):
    value, bridge, transport = backend
    value._authority = lambda: {**lease(), **change}
    with pytest.raises(PermissionError):
        value.send('stand', {'request_id': 'one', 'action': 'stand', 'expires_monotonic': time.monotonic() + 3}, threading.Event())
    assert not transport.states


def test_no_implicit_worker_start_and_no_commands_after_close(backend):
    value, bridge, transport = backend
    transport.running = False
    with pytest.raises(RuntimeError):
        value.send('stand', {'request_id': 'one', 'action': 'stand', 'expires_monotonic': time.monotonic() + 3}, threading.Event())
    transport.running = True
    value.close()
    with pytest.raises(PermissionError):
        value.send('stand', {'request_id': 'two', 'action': 'stand', 'expires_monotonic': time.monotonic() + 3}, threading.Event())
    assert not transport.states


def test_old_finish_cannot_clear_a_new_request(backend):
    value, bridge, transport = backend
    value.send('lie', {'request_id': 'one', 'action': 'lie', 'expires_monotonic': time.monotonic() + 3}, threading.Event())
    value.finish('one')
    value.send('lie', {'request_id': 'two', 'action': 'lie', 'expires_monotonic': time.monotonic() + 3}, threading.Event())
    value.finish('one')
    with pytest.raises(RuntimeError):
        value.send('stand', {'request_id': 'three', 'action': 'stand', 'expires_monotonic': time.monotonic() + 3}, threading.Event())


def test_motion_card_only_observes_in_production_and_releases_its_own_reader_lease():
    bridge = Bridge(); bridge.acquire('state', {'motion_info'}, {})
    backend = DDSMotionBackend(bridge, Transport())
    plugin = MotionPlugin({'enabled': True}, 'test', backend, feedback=backend.feedback, execution_authorized=False)
    try:
        plugin.start()
        bridge.put(0)
        info = plugin.status()
        assert info['availability_detail']['tls_required'] is False
        assert info['availability_detail']['fresh'] is True
        assert info['availability_detail']['command_worker_running'] is True  # injected fake only
        assert not info['execution_available']
        plugin.stop()
        assert list(bridge.active) == ['state']
    finally:
        plugin.close()


def test_stand_stage_feedback_delay_cannot_send_seventeen_after_expiry(backend):
    value, bridge, transport = backend
    request = {'request_id': 'one', 'action': 'stand', 'expires_monotonic': time.monotonic() + .08}
    value.send('stand', request, threading.Event())
    def delayed_poll(action, request_id):
        time.sleep(.1)
        return {'confirmed': True}
    value.feedback.poll = delayed_poll
    with pytest.raises(TimeoutError):
        value.progress('stand', request, threading.Event())
    assert transport.states == [(1, 'one')]


def test_missing_action_deadline_is_never_synthesized_by_backend(backend):
    value, bridge, transport = backend
    with pytest.raises(ValueError):
        value.send('stand', {'request_id': 'one', 'action': 'stand'}, threading.Event())
    assert not transport.states


@pytest.mark.parametrize('expiry', [None, True, False, float('nan'), float('inf'), -1.])
def test_invalid_or_expired_upstream_authority_deadline_never_sends(backend, expiry):
    value, bridge, transport = backend
    value._authority = lambda: {**lease(), 'expires_monotonic': expiry}
    with pytest.raises(PermissionError):
        value.send('stand', {'request_id': 'one', 'action': 'stand',
            'expires_monotonic': time.monotonic() + 3}, threading.Event())
    assert not transport.states


def test_authority_deadline_is_never_extended_and_is_rechecked_before_rl(backend):
    value, bridge, transport = backend
    source_expiry = time.monotonic() + .03
    value._authority = lambda: {**lease(), 'expires_monotonic': source_expiry}
    request = {'request_id': 'one', 'action': 'stand', 'expires_monotonic': time.monotonic() + 3}
    assert value._permit('one', action_deadline=request['expires_monotonic'])['expires_monotonic'] == source_expiry
    value.send('stand', request, threading.Event())
    bridge.put(1); value.progress('stand', request, threading.Event())
    time.sleep(.04)
    bridge.put(1)
    with pytest.raises(PermissionError):
        value.progress('stand', request, threading.Event())
    assert transport.states == [(1, 'one')]


def test_action_deadline_can_shorten_upstream_authority_deadline(backend):
    value, bridge, transport = backend
    value._authority = lambda: {**lease(), 'expires_monotonic': time.monotonic() + .4}
    action_deadline = time.monotonic() + .1
    assert value._permit('one', action_deadline=action_deadline)['expires_monotonic'] == action_deadline
