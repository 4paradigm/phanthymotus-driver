"""Isolated action-scoped policies. Every grant and physical result is fiction."""

import copy
import time

import pytest

from deep_robotics.s10_pro.motion import (
    CONTROL_CONTRACT_V2, MotionPlugin, action_contract, contract_readiness, validate_contract,
)


def fixture_contract(*actions):
    policies = {}
    for action in actions:
        common = {'verified': True,
            'evidence': {'stop': 'ISOLATED TEST: stop', 'watchdog': 'ISOLATED TEST: loss behavior'},
            'send_timeout_s': .02, 'stop_on_authority_loss': False}
        if action == 'velocity':
            common.update(mode='navigation', robot_deadman_verified=True, robot_deadman_s=.3,
                units={'vx': 'm/s', 'vy': 'm/s', 'yaw_rate': 'rad/s'},
                stop_semantics='zero_velocity_until_feedback')
        else:
            common.update(mode='regular', interruption_verified=True, max_uncontrolled_s=.2,
                          stop_semantics='verified_posture_adapter')
        policies[action] = common
    return {'schema': CONTROL_CONTRACT_V2, 'verified': True,
        'robot_identity_matched': True, 'firmware_matched': True,
        'evidence': {key: 'ISOLATED TEST: ' + key for key in ('identity', 'firmware', 'authority', 'feedback')},
        'actions': policies}


def test_posture_policy_can_be_verified_without_navigation_units_deadman_or_velocity():
    contract = fixture_contract('stand')
    assert validate_contract(contract) and validate_contract(contract, 'stand')
    assert validate_contract(contract, 'stopmotion')
    assert not validate_contract(contract, 'lie') and not validate_contract(contract, 'velocity')
    flat = action_contract(contract, 'stand')
    assert flat['mode'] == 'regular' and flat['action'] == 'stand'
    assert 'robot_deadman_s' not in flat and 'units' not in flat
    assert flat['evidence']['identity'].startswith('ISOLATED TEST')
    assert flat['evidence']['stop'] == 'ISOLATED TEST: stop'
    flat['evidence']['identity'] = 'changed'
    assert contract['evidence']['identity'] != 'changed'
    readiness = contract_readiness(contract)
    assert readiness['accepted_by_application']
    assert readiness['application_requirements']['actions']['stand']['accepted']
    assert not readiness['application_requirements']['actions']['velocity']['accepted']


@pytest.mark.parametrize('field,value', [('verified', False), ('mode', 'navigation'),
    ('interruption_verified', False), ('max_uncontrolled_s', None), ('max_uncontrolled_s', float('nan')),
    ('max_uncontrolled_s', True), ('max_uncontrolled_s', 2), ('send_timeout_s', .2),
    ('stop_semantics', 'zero_velocity_until_feedback'), ('stop_on_authority_loss', True)])
def test_posture_requires_its_own_strict_interruption_and_loss_policy(field, value):
    contract = fixture_contract('stand')
    contract['actions']['stand'][field] = value
    assert not validate_contract(contract, 'stand')
    assert action_contract(contract, 'stand') == {}


@pytest.mark.parametrize('section,field', [('evidence', 'identity'), ('evidence', 'firmware'),
    ('evidence', 'authority'), ('evidence', 'feedback'), ('stand_evidence', 'stop'),
    ('stand_evidence', 'watchdog')])
def test_v2_cannot_accept_missing_global_or_action_evidence(section, field):
    contract = fixture_contract('stand')
    evidence = contract['evidence'] if section == 'evidence' else contract['actions']['stand']['evidence']
    evidence.pop(field)
    assert not validate_contract(contract, 'stand')


def test_bad_velocity_contract_does_not_contaminate_verified_posture():
    contract = fixture_contract('stand', 'velocity')
    contract['actions']['velocity']['robot_deadman_verified'] = False
    assert validate_contract(contract, 'stand') and validate_contract(contract)
    assert not validate_contract(contract, 'velocity')
    assert action_contract(contract, 'velocity') == {}
    contract['actions']['stand']['verified'] = False
    assert not validate_contract(contract)


def test_global_identity_firmware_must_match_for_every_action():
    for field in ('verified', 'robot_identity_matched', 'firmware_matched'):
        contract = fixture_contract('stand', 'velocity')
        contract[field] = False
        assert not validate_contract(contract, 'stand')
        assert not validate_contract(contract, 'velocity')
    assert not validate_contract(fixture_contract())
    assert not validate_contract(fixture_contract('stand'), 'unknown')


class Notifier:
    def __init__(self):
        self.values = []

    def submit(self, payload):
        self.values.append(copy.deepcopy(payload))

    def status(self, _action_id):
        return {'delivery_state': 'acknowledged'}


class FictionalBackend:
    def __init__(self, *, fail_prepare=False):
        self.policy = fixture_contract('stand')
        self.commands = []
        self.prepares = []
        self.fail_prepare = fail_prepare
        self.valid = True
        self.mode = True
        self.allowed = ['stand']
        self.stop_at = None

    def contract(self):
        return copy.deepcopy(self.policy)

    def control_status(self):
        return {'owned': True, 'fresh': True, 'age_s': 0., 'authority_valid': self.valid,
            'feedback_fresh': True, 'regular_mode_confirmed': self.mode,
            'navigation_mode_confirmed': False, 'allowed_actions': self.allowed}

    def prepare_execution(self, action):
        self.prepares.append(action)
        if self.fail_prepare:
            raise RuntimeError('isolated startup rejection')

    def send(self, action, request, _cancel):
        self.commands.append((action, request['request_id'], time.monotonic()))
        return {'accepted': None}

    def stop(self, _reason, _cancel):
        self.stop_at = time.monotonic()
        return {'accepted': None}

    def feedback(self, action, rid):
        now = time.monotonic()
        confirmed = bool(self.commands) if action != 'stopmotion' else self.stop_at is not None
        return {'request_id': rid, 'confirmed': confirmed, 'fresh': True, 'age_s': 0.,
                'received_monotonic': now}


def plugin_for(backend, **kwargs):
    return MotionPlugin({'enabled': True, 'action_timeout_s': .2}, 'fiction', backend,
        feedback=backend.feedback, execution_authorized=True, notifier=Notifier(), **kwargs)


def test_status_start_are_read_only_and_prepare_occurs_before_task_acceptance_only_for_execution():
    backend = FictionalBackend()
    plugin = plugin_for(backend)
    try:
        plugin.status()
        plugin.start()
        assert backend.prepares == backend.commands == []
        result = plugin.execute('stand', {'request_id': 'fiction-stand'})
        assert result['state'] == 'accepted' and backend.prepares == ['stand']
        deadline = time.monotonic() + 2
        while plugin.status()['last_result'] is None and time.monotonic() < deadline:
            time.sleep(.01)
        terminal = plugin.status()['last_result']
        assert terminal['state'] == 'completed' and terminal['confirmed'] is True
        assert len(backend.commands) == 1
        assert plugin.execute('stand', {'request_id': 'fiction-stand'})['terminal'] is True
        assert backend.prepares == ['stand']  # Persisted replay never prepares again.
        rejected = plugin.execute('velocity', {'request_id': 'fiction-velocity'})
        assert rejected['state'] == 'unavailable' and backend.prepares == ['stand']
        assert plugin._tasks.get('fiction-velocity') is None
    finally:
        plugin.close()


def test_failed_explicit_worker_start_has_no_task_record_and_does_not_hold_busy_lock():
    backend = FictionalBackend(fail_prepare=True)
    plugin = plugin_for(backend)
    try:
        plugin.start()
        result = plugin.execute('stand', {'request_id': 'fiction-start-failure'})
        assert result['state'] == 'unavailable'
        assert result['blocked_by'] == ['command_worker_not_running']
        assert plugin._tasks.get('fiction-start-failure') is None and not backend.commands
        assert plugin._busy.acquire(blocking=False)
        plugin._busy.release()
    finally:
        plugin.close()


@pytest.mark.parametrize('failure', ['mode', 'allowed', 'valid'])
def test_mode_scope_or_invalid_authority_prevents_worker_start_and_task_acceptance(failure):
    backend = FictionalBackend()
    setattr(backend, failure, [] if failure == 'allowed' else False)
    plugin = plugin_for(backend)
    try:
        plugin.start()
        result = plugin.execute('stand', {'request_id': 'fiction-authority'})
        assert result['state'] == 'unavailable' and not backend.prepares and not backend.commands
        assert plugin._tasks.get('fiction-authority') is None
        action = result['readiness']['actions']['stand']
        assert not action['ready']
    finally:
        plugin.close()


def test_dynamic_deployment_authorization_revocation_blocks_before_preparation():
    backend = FictionalBackend()
    authorized = [True]
    plugin = plugin_for(backend, authorization_provider=lambda: authorized[0])
    try:
        plugin.start()
        assert plugin.status()['execution_authorized'] is True
        authorized[0] = False
        status = plugin.status()
        assert status['execution_authorized'] is False
        assert status['readiness']['process_authorization']['plugin_authorized'] is False
        result = plugin.execute('stand', {'request_id': 'fiction-expired'})
        assert result['state'] == 'unavailable' and 'execution_authorization' in result['blocked_by']
        assert backend.prepares == backend.commands == []
    finally:
        plugin.close()


def test_slow_preparation_does_not_accept_a_request_after_its_ttl():
    backend=FictionalBackend()
    wall=[1000.]
    def prepare(action):
        backend.prepares.append(action)
        wall[0]+=2.
    backend.prepare_execution=prepare
    plugin=plugin_for(backend,wall_time=lambda:wall[0])
    try:
        plugin.start()
        result=plugin.execute('stand',{'request_id':'expired-before-accept'})
        assert result['state']=='unavailable' and result['blocked_by']==['stale_request']
        assert plugin._tasks.get('expired-before-accept') is None and not backend.commands
    finally:plugin.close()
