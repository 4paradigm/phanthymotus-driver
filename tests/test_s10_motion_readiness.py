"""Read-only readiness diagnostics with isolated fictional control evidence.

No ROS, sockets, command worker, publisher or physical robot is started here.
"""
import copy

import pytest

from deep_robotics.s10_pro.motion import MotionPlugin, contract_readiness
from deep_robotics.s10_pro.motion_backend import DDSMotionBackend


def fictional_contract():
    return {'verified': True, 'evidence': {key: 'ISOLATED TEST FICTION: ' + key for key in
        ('identity', 'firmware', 'authority', 'feedback', 'stop', 'watchdog')},
        'robot_identity_matched': True, 'firmware_matched': True, 'robot_deadman_verified': True,
        'robot_deadman_s': .3, 'send_timeout_s': .05,
        'velocity_units': {'vx': 'm/s', 'vy': 'm/s', 'yaw_rate': 'rad/s'},
        'actions': ['stand', 'lie', 'velocity', 'stopmotion'],
        'stop_semantics': 'zero_velocity_until_feedback', 'stop_on_authority_loss': False}


class Bridge:
    def __init__(self):
        self.active = {}
        self.value = {'fresh': True, 'age_s': .1, 'received_monotonic': 99.9, 'sequence': 1,
            'data': {'data': {'motion_state': {'state': 0}, 'gait_state': {'gait': 0x3002},
                'vel_x': 0., 'vel_y': 0., 'vel_yaw': 0.}}}

    def snapshot(self, source):
        assert source == 'motion_info'
        return copy.deepcopy(self.value)

    def acquire(self, owner, sources, options):
        self.active[owner] = (sources, options)

    def release(self, owner):
        self.active.pop(owner, None)


class Transport:
    def __init__(self):
        self.running = True
        self.enabled = True
        self.calls = []

    def status(self):
        return {'worker_running': self.running, 'execution_enabled': self.enabled}

    def start(self, **kwargs):
        raise AssertionError('Readiness must never start a command worker')

    def publish_state(self, *args, **kwargs):
        raise AssertionError('Readiness must never publish a state command')

    def publish_velocity(self, *args, **kwargs):
        raise AssertionError('Readiness must never publish a velocity command')

    def close(self):
        self.running = False


class Notifier:
    def status(self, action_id):
        raise AssertionError('There is no accepted action to query')

    def submit(self, payload):
        raise AssertionError('Readiness must never create an ACP completion')


@pytest.fixture
def subject():
    bridge, transport = Bridge(), Transport()
    lease = {'owned': True, 'fresh': True, 'age_s': .1,
        'received_monotonic': 99.9, 'expires_monotonic': 100.2,
        'navigation_mode_confirmed': True}
    backend = DDSMotionBackend(bridge, transport, contract=fictional_contract(),
        authority_provider=lambda: copy.deepcopy(lease), execution_authorized=True,
        clock=lambda: 100.)
    plugin = MotionPlugin({'enabled': True}, 'isolated', backend, feedback=backend.feedback,
        notifier=Notifier(), execution_authorized=True, clock=lambda: 100., autostart_worker=False)
    yield plugin, backend, bridge, transport, lease
    plugin.close()


def test_started_and_enabled_are_distinct_and_legacy_rejection_is_preserved(subject):
    plugin, _, bridge, transport, _ = subject
    status = plugin.status()
    assert not status['execution_available']
    assert status['readiness']['card_enabled'] is True
    assert status['readiness']['card_started'] is False
    assert status['readiness']['blocked_by'] == ['card_not_started']
    rejected = plugin.dispatch('stand', {})
    assert rejected['blocked_by'] == ['card_not_enabled']
    assert rejected['readiness']['blocked_by'] == ['card_not_started']
    assert rejected['next_steps'][0]['code'] == 'card_not_started'
    assert not bridge.active and not transport.calls
    plugin.start()
    assert plugin.status()['execution_available']
    assert list(bridge.active) == ['motion_observer']
    plugin.configure({'enabled': False})
    status = plugin.status()
    assert not status['execution_available']
    assert status['readiness']['card_started'] is True
    assert status['readiness']['blocked_by'] == ['card_disabled']
    assert plugin.dispatch('stand', {})['blocked_by'] == ['card_not_enabled']


def test_real_backend_requires_explicitly_started_worker_without_starting_it(subject):
    plugin, _, _, transport, _ = subject
    transport.running = False
    plugin.start()
    status = plugin.status()
    assert not status['execution_available']
    assert status['readiness']['command_worker'] == {'required': True, 'running': False}
    assert status['readiness']['blocked_by'] == ['command_worker_not_running']
    assert status['next_steps'][0]['code'] == 'command_worker_not_running'
    assert not transport.calls


@pytest.mark.parametrize('change', [
    {'owned': False}, {'fresh': False}, {'age_s': .500001}, {'age_s': float('nan')},
    {'age_s': float('inf')}, {'age_s': True}, {'age_s': -.1},
    {'received_monotonic': 100.1}, {'received_monotonic': 0.},
    {'expires_monotonic': 100.}, {'expires_monotonic': float('nan')},
])
def test_owned_or_fresh_flags_cannot_hide_invalid_real_authority(subject, change):
    plugin, _, _, _, lease = subject
    lease.update(change)
    plugin.start()
    status = plugin.status()
    assert not status['execution_available']
    authority = status['readiness']['authority']
    assert authority['provider_bound'] is True
    assert authority['fresh'] is False
    assert authority['max_age_s'] == .5
    assert 'fresh_control_authority' in status['readiness']['blocked_by']


def test_visible_fresh_feedback_never_grants_ownership(subject):
    plugin, backend, _, _, _ = subject
    backend._authority = None
    plugin.start()
    status = plugin.status()
    assert status['readiness']['feedback']['fresh']
    assert not status['execution_available']
    assert status['readiness']['authority']['provider_bound'] is False
    assert 'authority_provider_unbound' in status['readiness']['blocked_by']
    rejected = plugin.dispatch('stand', {})
    assert rejected['blocked_by'] == ['fresh_control_authority']
    assert rejected['readiness']['authority']['provider_bound'] is False
    assert 'action_id' not in rejected
    assert plugin.status()['active_request'] is None


@pytest.mark.parametrize('age', [.500001, 8., None, float('nan'), float('inf'), True, -.1])
def test_actual_feedback_age_is_reported_without_weakening_half_second_limit(subject, age):
    plugin, _, bridge, _, _ = subject
    bridge.value.update(age_s=age)
    plugin.start()
    status = plugin.status()
    feedback = status['readiness']['feedback']
    assert not status['execution_available'] and not feedback['fresh']
    assert feedback['max_age_s'] == .5
    expected_age = age if type(age) in (int, float) and 0 <= age < float('inf') else None
    assert feedback['age_s'] == expected_age
    assert 'fresh_motion_feedback' in status['readiness']['blocked_by']


def test_default_production_boundary_explains_evidence_and_process_lock(subject):
    plugin, backend, _, transport, _ = subject
    plugin._authorized = False
    backend._authorized = False
    backend._policy = {}
    backend._authority = None
    transport.running = False
    plugin.configure({'enabled': False})
    result = plugin.dispatch('stand', {})
    assert result['blocked_by'] == ['execution_authorization', 'robot_control_contract', 'card_not_enabled']
    readiness = result['readiness']
    assert readiness['process_authorization'] == {'granted': False, 'plugin_authorized': False,
        'backend_authorized': False, 'transport_enabled': True, 'canvas_configurable': False}
    assert readiness['contract']['machine_evidence']['missing'] == [
        'identity', 'firmware', 'authority', 'feedback', 'stop', 'watchdog']
    assert readiness['contract']['machine_evidence']['unmatched'] == [
        'robot_identity_matched', 'firmware_matched', 'robot_deadman_verified']
    assert readiness['contract']['application_requirements']['robot_deadman_range_s'] == [.1, 1.]
    assert 'complete_action_coverage' in readiness['contract']['application_requirements']['unmet']
    assert result['next_steps'] == readiness['next_steps']
    assert {item['code'] for item in result['next_steps']} == set(readiness['blocked_by'])
    assert not result['accepted'] and not result['confirmed'] and not result['completed']
    assert 'action_id' not in result


def test_application_policy_failures_are_not_reported_as_missing_machine_evidence():
    policy = fictional_contract()
    policy.update(robot_deadman_s=.1, send_timeout_s=.1, actions=['stand'])
    readiness = contract_readiness(policy)
    assert not readiness['accepted_by_application']
    assert readiness['machine_evidence']['missing'] == []
    assert readiness['machine_evidence']['unmatched'] == []
    assert readiness['application_requirements']['unmet'] == [
        'complete_action_coverage', 'send_timeout_below_half_deadman']


@pytest.mark.parametrize('layer,reason', [
    ('backend', 'backend_execution_authorization'), ('transport', 'transport_execution_authorization')])
def test_process_authorization_includes_real_backend_and_transport(subject, layer, reason):
    plugin, backend, _, transport, _ = subject
    if layer == 'backend':
        backend._authorized = False
    else:
        transport.enabled = False
    plugin.start()
    status = plugin.status()
    assert not status['execution_available']
    assert not status['readiness']['process_authorization']['granted']
    assert status['readiness']['process_authorization']['plugin_authorized'] is True
    assert status['readiness']['blocked_by'] == [reason]


def test_feedback_half_second_boundary_remains_unchanged(subject):
    plugin, _, bridge, _, _ = subject
    bridge.value.update(age_s=.5, received_monotonic=99.5)
    plugin.start()
    status = plugin.status()
    assert status['readiness']['feedback']['age_s'] == .5
    assert status['readiness']['feedback']['fresh'] is True
    assert status['execution_available'] is True


def test_closed_controller_rejection_retains_diagnostics_without_new_task(subject):
    plugin, _, _, _, _ = subject
    plugin.close()
    result = plugin.dispatch('stand', {})
    assert result['blocked_by'] == ['controller_closed']
    assert not result['readiness']['ready']
    assert 'controller_closed' in result['readiness']['blocked_by']
    assert not result['accepted'] and 'action_id' not in result


def test_unknown_adapter_is_unavailable_without_probing_action_side_authority():
    class ActionOnlyBackend:
        def contract(self):
            return fictional_contract()

        def control_status(self):
            raise AssertionError('Rendering status must not call an action-side adapter')

    plugin = MotionPlugin({'enabled': True}, 'isolated', ActionOnlyBackend(),
        feedback=lambda *args: {}, notifier=Notifier(), execution_authorized=True, autostart_worker=False)
    try:
        plugin.start()
        status = plugin.status()
        assert not status['execution_available']
        assert status['readiness']['authority']['provider_bound'] is None
        assert not status['readiness']['authority']['fresh']
        assert status['readiness']['feedback']['age_s'] is None
    finally:
        plugin.close()
