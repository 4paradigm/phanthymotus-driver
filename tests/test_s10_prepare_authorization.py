"""Prepare renewal and failure boundaries, using synthetic robot feedback."""
import copy
import json
import time

import pytest

from deep_robotics.s10_pro import operator_authorization as auth
from deep_robotics.s10_pro.canvas_posture import CanvasPostureBackend, CanvasPosturePlugin
from test_s10_timed_move import Bridge, Transport, Notifier


@pytest.fixture
def policy(monkeypatch):
    manifest = {'schema': auth.PREPARE_POLICY_SCHEMA, 'enabled': True,
        'policy_id': 'test-prepare', 'robot_identity': 'machine',
        'allowed_actions': ['stand', 'lie', 'move_mode'], 'session_duration_s': 3600,
        'activation': 'explicit_prepare', 'provenance': 'owner_enabled_canvas_prepare'}
    state = {'raw': json.dumps(manifest).encode(), 'boot': 'boot', 'machine': 'machine',
             'clock': 100., 'wall': 1000.}
    monkeypatch.setattr(auth, '_read', lambda path: state['raw'])
    monkeypatch.setattr(auth, 'prepare_machine_identity', lambda: state['machine'])
    monkeypatch.setattr(auth, 'robot_session_identity', lambda: state['boot'])
    def load(real_clock=False):
        return auth.PrepareAuthorization('/policy', clock=time.monotonic if real_clock else lambda: state['clock'],
            wall_time=time.time if real_clock else lambda: state['wall'])
    return manifest, state, load


def test_loading_and_status_do_not_activate_policy(policy):
    _, _, load = policy
    grant = load()
    assert grant.supports_prepare
    for _ in range(3):
        assert not grant.authorized_now()
        assert grant.public_status()['reason'] == 'prepare_required'
    with pytest.raises(PermissionError):
        grant.permit('no-prepare', 'stand', 110.)


def test_expiry_can_be_renewed_only_by_explicit_prepare(policy):
    _, state, load = policy
    grant = load()
    first = grant.prepare_session()
    assert first['authorized'] and first['expires_at_ms'] == 4600000
    original = state['raw']
    state['clock'] += 3601
    state['wall'] += 3601
    assert not grant.authorized_now()
    assert grant.public_status()['reason'] == 'prepare_required_session_expired'
    second = grant.prepare_session()
    assert second['authorized'] and first['session_id'] != second['session_id']
    assert state['raw'] == original
    with pytest.raises(PermissionError):
        grant.permit('distance-denied', 'forward_30cm', state['clock'] + 10)


@pytest.mark.parametrize('changed', ['boot', 'machine', 'policy'])
def test_revoke_or_identity_change_cannot_be_reset_by_prepare(policy, changed):
    _, state, load = policy
    grant = load()
    grant.prepare_session()
    original = dict(state)
    if changed == 'policy':
        state['raw'] += b' '
    else:
        state[changed] = 'different'
    assert not grant.authorized_now()
    state.update(original)
    with pytest.raises(PermissionError):
        grant.prepare_session()


def test_process_restart_requires_new_prepare(policy):
    _, _, load = policy
    load().prepare_session()
    assert not load().authorized_now()


@pytest.fixture
def runtime(policy):
    _, _, load = policy
    grant, bridge, transports = load(real_clock=True), Bridge(), []
    bridge.state = 0
    def factory():
        transport = Transport(bridge)
        transports.append(transport)
        return transport
    backend = CanvasPostureBackend(bridge, factory(), grant, transport_factory=factory)
    plugin = CanvasPosturePlugin({'enabled': True}, 'test', backend,
        execution_authorized=True, notifier=Notifier())
    plugin.start()
    yield plugin, backend, grant, bridge, transports
    plugin.close()


def test_prepare_renews_and_replaces_channel_without_commands(runtime):
    plugin, backend, grant, _, transports = runtime
    assert not grant.authorized_now()
    result = plugin.dispatch('prepare', {})
    assert result['prepared'] and result['authorization_renewed']
    first = grant.public_status()['session_id']
    assert len(transports) == 2 and transports[0].closed and transports[-1].running
    result = plugin.dispatch('prepare', {})
    assert result['prepared'] and grant.public_status()['session_id'] != first
    assert all(not transport.velocities and not transport.states for transport in transports)
    assert not backend.safety_status()['failed_closed']


def test_bad_feedback_prepare_never_opens_session(runtime):
    plugin, _, grant, bridge, transports = runtime
    bridge.hes = 1
    result = plugin.dispatch('prepare', {})
    assert not result['accepted'] and not grant.authorized_now()
    assert not result['physical_command_sent']
    assert len(transports) == 1


def test_busy_prepare_does_not_renew_or_replace_active_session(runtime):
    plugin, _, grant, _, transports = runtime
    plugin.dispatch('prepare', {})
    identity = grant.public_status()['session_id']
    with plugin._lock:
        plugin._active = {'request_id': 'existing', 'action': 'stand', 'action_id': 'existing'}
        try:
            assert plugin.dispatch('prepare', {})['state'] == 'busy'
            assert grant.public_status()['session_id'] == identity and len(transports) == 2
        finally:
            plugin._active = None


def test_failed_channel_prepare_clears_new_session(runtime):
    plugin, backend, grant, bridge, transports = runtime
    def bad_factory():
        transport = Transport(bridge)
        transport.start = lambda **kwargs: (_ for _ in ()).throw(RuntimeError('test startup failure'))
        transports.append(transport)
        return transport
    backend._transport_factory = bad_factory
    reply = plugin.dispatch('prepare', {})
    assert reply.get('accepted') is not True
    assert not grant.authorized_now()
    assert all(not transport.velocities and not transport.states for transport in transports)


def test_action_click_cannot_renew_expired_session(runtime):
    plugin, _, grant, _, transports = runtime
    # Synthetic expiry, without changing or weakening protected policy.
    grant.prepare_session()
    grant._session['expires_monotonic'] = time.monotonic() - 1.
    reply = plugin.dispatch('stand', {'request_id': 'expired-action'})
    assert reply.get('accepted') is not True
    assert not grant.authorized_now()
    assert len(transports) == 1 and not transports[0].states


def automatic_manifest(manifest, state, *, startup=True, renewal=True):
    manifest.update(schema=auth.AUTOMATIC_PREPARE_POLICY_SCHEMA,
                    activation='automatic_prepare', provenance='owner_approved_automatic_prepare',
                    allow_automatic_startup=startup, allow_automatic_renewal=renewal)
    state['raw'] = json.dumps(manifest).encode()


def test_explicit_policy_cannot_activate_or_renew_automatically(policy):
    _, _, load = policy
    grant = load()
    assert not grant.automatic_policy_enabled()
    for _ in range(2):
        with pytest.raises(PermissionError):
            grant.prepare_session(automatic=True)
        assert not grant.authorized_now()
    grant.prepare_session()
    grant.clear_session('test-expired')
    with pytest.raises(PermissionError):
        grant.prepare_session(automatic=True)
    assert not grant.authorized_now()


@pytest.mark.parametrize('startup,renewal', [(True, True), (True, False), (False, True), (False, False)])
def test_automatic_startup_and_renewal_are_independent(policy, startup, renewal):
    manifest, state, load = policy
    automatic_manifest(manifest, state, startup=startup, renewal=renewal)
    grant = load()
    assert grant.supports_prepare and grant.automatic_policy_enabled()
    assert grant.automatic_prepare_allowed() == startup
    if startup:
        grant.prepare_session(automatic=True)
    else:
        with pytest.raises(PermissionError):
            grant.prepare_session(automatic=True)
        grant.prepare_session()  # Explicit trusted preparation remains distinct.
    state['clock'] += 3601
    state['wall'] += 3601
    assert not grant.authorized_now()
    assert grant.automatic_prepare_allowed() == renewal
    if renewal:
        assert grant.prepare_session(automatic=True)['authorized']
    else:
        with pytest.raises(PermissionError):
            grant.prepare_session(automatic=True)
    # Restarted process must re-evaluate the separately granted startup permission.
    restarted = load()
    assert not restarted.authorized_now()
    assert restarted.automatic_prepare_allowed() == startup


@pytest.mark.parametrize('change', [
    {'allow_automatic_startup': 1}, {'allow_automatic_renewal': 'true'},
    {'allow_automatic_startup': None}, {'activation': 'explicit_prepare'},
    {'provenance': 'owner_enabled_canvas_prepare'}, {'schema': auth.PREPARE_POLICY_SCHEMA}])
def test_automatic_policy_rejects_ambiguous_or_legacy_approvals(policy, change):
    manifest, state, load = policy
    automatic_manifest(manifest, state)
    manifest.update(change)
    state['raw'] = json.dumps(manifest).encode()
    grant = load()
    assert not grant.supports_prepare
    assert not grant.automatic_prepare_allowed()


def test_automatic_policy_cannot_bypass_revoke_or_replacement(policy):
    manifest, state, load = policy
    automatic_manifest(manifest, state)
    grant = load()
    grant.prepare_session(automatic=True)
    state['raw'] += b' '
    assert not grant.automatic_prepare_allowed()
    state['raw'] = json.dumps(manifest).encode()
    assert not grant.automatic_prepare_allowed()
    with pytest.raises(PermissionError):
        grant.prepare_session(automatic=True)
