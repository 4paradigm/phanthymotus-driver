"""Fictional operator sessions only; no ROS, network or robot command I/O."""
import copy
import hashlib
import json
import math
import os

import pytest

from deep_robotics.s10_pro import operator_authorization as authorization
from deep_robotics.s10_pro.motion_dds import AuthorityUnavailable, validate_operator_permit


SCHEMA = 's10_pro.operator_authorization/1'
PERSISTENT_SCHEMA = 's10_pro.operator_authorization/2'
MACHINE_IDENTITY = 'a' * 64
ROBOT_SESSION = 'b' * 64
PERMIT_KEYS = {'schema', 'session_id', 'request_id', 'action', 'robot_session',
               'issued_monotonic', 'expires_monotonic', 'action_deadline_monotonic'}


class Clock:
    def __init__(self):
        self.wall = 1000.
        self.monotonic = 10.

    def time(self):
        return self.wall

    def mono(self):
        return self.monotonic


@pytest.fixture
def operator_session(tmp_path, monkeypatch):
    """Simulate root ownership without adding a product owner-UID override."""
    real_fstat = os.fstat
    ownership = {'uid': 0}

    class OwnershipView:
        def __init__(self, value):
            self.value = value

        def __getattr__(self, name):
            return ownership['uid'] if name == 'st_uid' else getattr(self.value, name)

    monkeypatch.setattr(authorization.os, 'fstat', lambda fd: OwnershipView(real_fstat(fd)))
    monkeypatch.setattr(authorization, 'robot_session_identity', lambda: ROBOT_SESSION)
    monkeypatch.delenv('S10_OPERATOR_AUTHORIZATION_FILE', raising=False)
    clock = Clock()
    manifest = {'schema': SCHEMA, 'execution_authorized': True,
                'session_id': 'fictional_operator_session', 'robot_session': ROBOT_SESSION,
                'allowed_actions': ['stand', 'lie'], 'issued_at_ms': 999900,
                'expires_at_ms': 1060000, 'operator_present': True,
                'handheld_estop_verified': True, 'no_competing_commands': True,
                'provenance': 'onsite_operator_confirmation'}
    path = tmp_path / 'operator-approval.json'

    def write(value=None):
        path.write_text(json.dumps(manifest if value is None else value))
        path.chmod(0o600)

    def load():
        return authorization.OperatorAuthorization(str(path), clock=clock.mono,
            wall_time=clock.time, robot_session=ROBOT_SESSION)

    write()
    return clock, manifest, path, write, load, ownership


def test_no_file_defaults_to_denied_and_canvas_config_cannot_grant(monkeypatch):
    monkeypatch.delenv('S10_OPERATOR_AUTHORIZATION_FILE', raising=False)
    policy = authorization.OperatorAuthorization()
    assert policy.configured is False and not policy.authorized_now()
    with pytest.raises(PermissionError):
        policy.permit('no-grant', 'stand', 100.)
    configured = authorization.load_operator_authorization({
        'execution_authorized': True, 'operator_present': True,
        'operator_authorization': {'execution_authorized': True},
        'S10_OPERATOR_AUTHORIZATION_FILE': '/a/canvas/cannot/select/a/grant'})
    assert configured.configured is False and not configured.authorized_now()


def test_valid_session_permit_is_action_scoped_and_has_no_vendor_lease_claims(operator_session):
    clock, _, _, _, load, _ = operator_session
    policy = load()
    assert policy.configured is True and policy.authorized_now()
    permit = policy.permit('posture-1', 'stand', clock.monotonic + 2.)
    assert set(permit) == PERMIT_KEYS
    assert permit['request_id'] == 'posture-1' and permit['action'] == 'stand'
    assert permit['robot_session'] == ROBOT_SESSION
    assert permit['issued_monotonic'] <= clock.monotonic
    assert clock.monotonic < min(permit['expires_monotonic'], permit['action_deadline_monotonic'])
    assert permit['action_deadline_monotonic'] == clock.monotonic + 2.
    assert not ({'owned', 'fresh', 'vendor_grant', 'firmware_matched'} & permit.keys())
    assert policy.permit('posture-2', 'lie', clock.monotonic + 1.)['action'] == 'lie'


@pytest.mark.parametrize('action', ['velocity', 'stopmotion', 'charging', '17', '', None])
def test_session_cannot_grant_other_actions(operator_session, action):
    clock, _, _, _, load, _ = operator_session
    with pytest.raises((PermissionError, ValueError)):
        load().permit('posture-other', action, clock.monotonic + 1.)


def test_manifest_action_scope_cannot_be_expanded_by_caller(operator_session):
    clock, manifest, _, write, load, _ = operator_session
    manifest['allowed_actions'] = ['lie']
    write()
    policy = load()
    assert policy.authorized_now()
    with pytest.raises(PermissionError):
        policy.permit('stand-out-of-scope', 'stand', clock.monotonic + 1.)
    assert policy.permit('lie-in-scope', 'lie', clock.monotonic + 1.)['action'] == 'lie'


@pytest.mark.parametrize(('field', 'value'), [
    ('schema', 's10_pro.execution_authorization/1'),
    ('execution_authorized', False), ('execution_authorized', 1),
    ('session_id', ''), ('session_id', '   '),
    ('robot_session', 'c' * 64), ('robot_session', 'not-a-boot-session-digest'),
    ('allowed_actions', []), ('allowed_actions', ['stand', 'stand']),
    ('allowed_actions', ['stand', 'velocity']), ('allowed_actions', 'stand'),
    ('issued_at_ms', 1000101), ('issued_at_ms', True),
    ('expires_at_ms', 1000000), ('expires_at_ms', 999900 + 3600001),
    ('expires_at_ms', '1060000'), ('expires_at_ms', float('inf')),
    ('operator_present', False), ('operator_present', 'true'),
    ('handheld_estop_verified', False), ('handheld_estop_verified', 1),
    ('no_competing_commands', False), ('no_competing_commands', 1),
    ('provenance', 'vendor_owned'), ('provenance', 'model_inferred'),
])
def test_invalid_or_unconfirmed_session_never_authorizes(operator_session, field, value):
    _, manifest, _, write, load, _ = operator_session
    manifest[field] = value
    write()
    policy = load()
    assert not policy.authorized_now()
    with pytest.raises(PermissionError):
        policy.permit('invalid-session', 'stand', 11.)


@pytest.mark.parametrize('missing', [
    'schema', 'execution_authorized', 'session_id', 'robot_session', 'allowed_actions',
    'issued_at_ms', 'expires_at_ms', 'operator_present', 'handheld_estop_verified',
    'no_competing_commands', 'provenance',
])
def test_every_session_field_is_required(operator_session, missing):
    _, manifest, _, write, load, _ = operator_session
    del manifest[missing]
    write()
    assert not load().authorized_now()


def test_unknown_duplicate_or_malformed_fields_are_denied_without_secret_diagnostics(operator_session):
    _, manifest, path, write, load, _ = operator_session
    secret = 'PRIVATE_OPERATOR_MATERIAL_MUST_NOT_BE_PUBLIC'
    invalid = copy.deepcopy(manifest)
    invalid[secret] = secret
    raw_invalid = [json.dumps(invalid),
        json.dumps(manifest)[:-1] + ',"operator_present":false}',
        '{"schema":', secret]
    for raw in raw_invalid:
        path.write_text(raw)
        policy = load()
        assert not policy.authorized_now()
        public = json.dumps(policy.public_status())
        assert secret not in public and str(path) not in public
    write()


@pytest.mark.parametrize('mode', [0o620, 0o602, 0o660, 0o666])
def test_group_or_world_writable_file_is_rejected(operator_session, mode):
    _, _, path, _, load, _ = operator_session
    path.chmod(mode)
    assert not load().authorized_now()


def test_non_root_file_is_rejected_even_when_permission_bits_are_private(operator_session):
    _, _, _, _, load, ownership = operator_session
    ownership['uid'] = 1000
    assert not load().authorized_now()


def test_symlink_and_non_regular_file_never_authorize(operator_session):
    _, _, path, _, load, _ = operator_session
    target = path.with_name('original-approval.json')
    path.rename(target)
    path.symlink_to(target)
    assert not load().authorized_now()
    path.unlink()
    path.mkdir()
    assert not load().authorized_now()
    path.rmdir()
    os.mkfifo(path, 0o600)
    assert not load().authorized_now()


@pytest.mark.parametrize('change', ['content', 'permissions', 'ownership', 'missing'])
def test_changed_or_removed_approval_revokes_stickily(operator_session, change):
    _, manifest, path, write, load, ownership = operator_session
    policy = load()
    original = path.read_bytes()
    assert policy.authorized_now()
    if change == 'content':
        # Even semantically equivalent content cannot silently renew a bound file.
        path.write_text(json.dumps(manifest, indent=2))
    elif change == 'permissions':
        path.chmod(0o660)
    elif change == 'ownership':
        ownership['uid'] = 1000
    else:
        path.unlink()
    assert not policy.authorized_now()
    ownership['uid'] = 0
    path.write_bytes(original)
    path.chmod(0o600)
    assert not policy.authorized_now()
    # Restoration needs an explicit fresh object; the revoked runtime stays denied.
    assert load().authorized_now()


def test_monotonic_expiry_cannot_be_extended_by_wall_clock_rollback(operator_session):
    clock, _, _, _, load, _ = operator_session
    policy = load()
    assert policy.authorized_now()
    clock.wall = 900.
    clock.monotonic += 60.01
    assert not policy.authorized_now()
    clock.wall = 1000.
    clock.monotonic = 10.
    assert not policy.authorized_now()


def test_changed_host_boot_revokes_without_claiming_vendor_identity(operator_session, monkeypatch):
    _, _, _, _, load, _ = operator_session
    policy = load()
    assert policy.authorized_now()
    public = policy.public_status()
    assert public['vendor_ownership_claimed'] is False
    assert public['automatic_posture_stop_verified'] is False
    monkeypatch.setattr(authorization, 'robot_session_identity', lambda: 'c' * 64)
    assert not policy.authorized_now()
    monkeypatch.setattr(authorization, 'robot_session_identity', lambda: ROBOT_SESSION)
    assert not policy.authorized_now()


def test_permit_expiry_is_bounded_by_original_session_expiry(operator_session):
    clock, _, _, _, load, _ = operator_session
    policy = load()
    original_session_deadline = clock.monotonic + 60.
    clock.monotonic += 59.
    clock.wall += 59.
    with pytest.raises(PermissionError):
        policy.permit('outlives-session', 'stand', clock.monotonic + 20.)
    permit = policy.permit('near-expiry', 'stand', original_session_deadline - .1)
    assert clock.monotonic < permit['expires_monotonic'] <= original_session_deadline
    clock.monotonic = original_session_deadline
    clock.wall = 1060.
    with pytest.raises(PermissionError):
        policy.permit('expired', 'stand', clock.monotonic + 1.)


def test_one_hour_wall_to_monotonic_rounding_never_exceeds_original_session_bound(operator_session):
    # Deterministic values from the parent-side rejection: separate wall-clock
    # conversions round the expiry a float step beyond issued + 3600 seconds.
    # Keep the transport limit strict; bound the producer's mapping instead.
    clock, manifest, _, write, load, _ = operator_session
    issued_ms = 1792741567000
    now_ms = 1792741651104.4966
    clock.wall = now_ms / 1000
    clock.monotonic = 844.0673228748906
    manifest.update(issued_at_ms=issued_ms, expires_at_ms=issued_ms + 3600000)
    write()
    old_mapped_expiry = clock.monotonic + (manifest['expires_at_ms'] - now_ms) / 1000
    policy = load()
    assert policy.authorized_now()
    permit = policy.permit('rounding-stand', 'stand', clock.monotonic + 1.)
    assert old_mapped_expiry > permit['issued_monotonic'] + 3600
    assert permit['expires_monotonic'] <= permit['issued_monotonic'] + 3600
    assert permit['expires_monotonic'] <= old_mapped_expiry
    assert validate_operator_permit(permit, now=clock.monotonic) == permit

    # Repeated observation and the next unique action preserve both original
    # session timestamps; a query must not renew the operator's approval.
    clock.monotonic += .125
    clock.wall += .125
    assert policy.public_status()['authorized'] is True
    later = policy.permit('rounding-lie', 'lie', clock.monotonic + 2.)
    assert later['issued_monotonic'] == permit['issued_monotonic']
    assert later['expires_monotonic'] == permit['expires_monotonic']
    assert validate_operator_permit(later, now=clock.monotonic) == later

    # One representable step beyond the limit remains invalid. This prevents
    # fixing the valid producer by allowing an unbounded validation epsilon.
    forged = {**later, 'expires_monotonic': math.nextafter(later['issued_monotonic'] + 3600, math.inf)}
    with pytest.raises(AuthorityUnavailable):
        validate_operator_permit(forged, now=clock.monotonic)


@pytest.mark.parametrize('deadline', [0., 10., float('nan'), float('inf'), True, '11'])
def test_invalid_or_elapsed_action_deadline_cannot_create_permit(operator_session, deadline):
    _, _, _, _, load, _ = operator_session
    with pytest.raises((PermissionError, ValueError)):
        load().permit('bad-deadline', 'stand', deadline)


@pytest.mark.parametrize('request_id', ['', '   ', None, 'x' * 129])
def test_permit_requires_valid_bounded_request_identity(operator_session, request_id):
    clock, _, _, _, load, _ = operator_session
    with pytest.raises((PermissionError, ValueError)):
        load().permit(request_id, 'stand', clock.monotonic + 1.)


def test_load_env_bad_file_keeps_read_only_startup_available(tmp_path, monkeypatch):
    path = tmp_path / 'private-malformed.json'
    path.write_text('{not-json-or-a-valid-approval')
    path.chmod(0o600)
    monkeypatch.setenv('S10_OPERATOR_AUTHORIZATION_FILE', str(path))
    policy = authorization.load_operator_authorization({'motion': {'enabled': True}})
    assert policy.configured is True and not policy.authorized_now()
    assert str(path) not in json.dumps(policy.public_status())


@pytest.fixture
def persistent_operator_session(operator_session, monkeypatch):
    """An explicitly non-expiring grant still belongs to one stable machine."""
    clock, manifest, path, write, _, ownership = operator_session
    machine = {'identity': MACHINE_IDENTITY}
    monkeypatch.setattr(authorization, 'machine_identity', lambda: machine['identity'])
    manifest.pop('robot_session')
    manifest.update(schema=PERSISTENT_SCHEMA, robot_identity=MACHINE_IDENTITY,
                    lifetime='until_revoked', expires_at_ms=None)

    def load():
        return authorization.OperatorAuthorization(str(path), clock=clock.mono,
            wall_time=clock.time, robot_identity=MACHINE_IDENTITY)

    write()
    return clock, manifest, path, write, load, ownership, machine


def test_explicit_persistent_grant_has_no_session_time_limit_or_vendor_ownership_claim(persistent_operator_session):
    clock, _, _, _, load, _, _ = persistent_operator_session
    policy = load()
    assert policy.authorized_now()
    public = policy.public_status()
    assert public['lifetime'] == 'until_revoked'
    assert public['non_expiring'] is True and public['expires_at_ms'] is None
    assert public['vendor_ownership_claimed'] is False
    assert public['automatic_posture_stop_verified'] is False
    assert public['canvas_configurable'] is False
    permit = policy.permit('persistent-first', 'stand', clock.monotonic + 10.)
    assert set(permit) == PERMIT_KEYS | {'lifetime'}
    assert permit['schema'] == 's10_pro.operator_permit/2'
    assert permit['lifetime'] == 'until_revoked' and permit['expires_monotonic'] is None
    assert permit['issued_monotonic'] <= clock.monotonic
    assert permit['robot_session'] == ROBOT_SESSION
    assert not ({'owned', 'fresh', 'vendor_grant', 'firmware_matched'} & permit.keys())
    assert validate_operator_permit(permit, now=clock.monotonic) == permit

    # Persisting permission changes its lifetime, not the finite action window.
    # A new query/action after a month must not rewrite the process issue marker.
    month = 30 * 24 * 60 * 60
    clock.monotonic += month
    clock.wall += month
    assert policy.authorized_now() and policy.public_status()['non_expiring'] is True
    later = policy.permit('persistent-later', 'lie', clock.monotonic + 15.)
    assert later['issued_monotonic'] == permit['issued_monotonic']
    assert later['expires_monotonic'] is None
    assert later['action_deadline_monotonic'] == clock.monotonic + 15.
    assert validate_operator_permit(later, now=clock.monotonic) == later


def test_persistent_machine_grant_survives_restart_but_private_permit_uses_current_boot(
        persistent_operator_session, monkeypatch):
    clock, _, _, _, load, _, _ = persistent_operator_session
    original = load()
    old = original.permit('before-restart', 'stand', clock.monotonic + 1.)
    assert old['robot_session'] == ROBOT_SESSION
    new_boot = 'c' * 64
    monkeypatch.setattr(authorization, 'robot_session_identity', lambda: new_boot)
    clock.wall += 86400
    clock.monotonic = 5.
    restarted = load()
    assert restarted.authorized_now()
    new = restarted.permit('after-restart', 'lie', clock.monotonic + 1.)
    assert new['robot_session'] == new_boot
    assert 0 <= new['issued_monotonic'] <= clock.monotonic
    assert new['expires_monotonic'] is None
    assert validate_operator_permit(new, now=clock.monotonic) == new


@pytest.mark.parametrize('wall_time', [0., 999.8])
def test_persistent_issue_timestamp_is_audit_metadata_even_when_rtc_is_behind(
        persistent_operator_session, wall_time):
    clock, manifest, _, _, load, _, _ = persistent_operator_session
    issued_at_ms = manifest['issued_at_ms']
    clock.wall = wall_time
    assert clock.wall * 1000 < issued_at_ms
    policy = load()
    assert policy.authorized_now()
    permit = policy.permit('rtc-behind-issue', 'stand', clock.monotonic + 1.)
    assert permit['issued_monotonic'] == clock.monotonic
    assert permit['expires_monotonic'] is None
    assert manifest['issued_at_ms'] == issued_at_ms
    assert validate_operator_permit(permit, now=clock.monotonic) == permit


def test_persistent_restart_with_reset_rtc_uses_new_boot_and_does_not_renew_audit_time(
        persistent_operator_session, monkeypatch):
    clock, manifest, _, _, load, _, _ = persistent_operator_session
    issued_at_ms = manifest['issued_at_ms']
    first = load().permit('original-boot-request', 'stand', clock.monotonic + 1.)
    clock.wall, clock.monotonic = 0., 2.
    new_boot = 'c' * 64
    monkeypatch.setattr(authorization, 'robot_session_identity', lambda: new_boot)
    restarted = load()
    assert restarted.authorized_now()
    second = restarted.permit('new-boot-request', 'lie', clock.monotonic + 1.)
    assert first['robot_session'] == ROBOT_SESSION and second['robot_session'] == new_boot
    assert second['request_id'] != first['request_id']
    assert second['issued_monotonic'] == clock.monotonic
    assert manifest['issued_at_ms'] == issued_at_ms
    assert validate_operator_permit(second, now=clock.monotonic) == second


def test_persistent_running_process_ignores_wall_clock_rollback_without_renewing_receipt(
        persistent_operator_session):
    clock, _, _, _, load, _, _ = persistent_operator_session
    policy = load()
    first = policy.permit('before-rtc-rollback', 'stand', clock.monotonic + 1.)
    clock.wall, clock.monotonic = 0., clock.monotonic + 2.
    assert policy.authorized_now()
    second = policy.permit('after-rtc-rollback', 'lie', clock.monotonic + 1.)
    assert second['issued_monotonic'] == first['issued_monotonic']
    assert second['expires_monotonic'] is None
    assert validate_operator_permit(second, now=clock.monotonic) == second


def test_persistent_grant_machine_identity_change_revokes_stickily(persistent_operator_session):
    clock, _, _, _, load, _, machine = persistent_operator_session
    policy = load()
    assert policy.authorized_now()
    machine['identity'] = 'd' * 64
    assert not policy.authorized_now()
    machine['identity'] = MACHINE_IDENTITY
    assert not policy.authorized_now()
    with pytest.raises(PermissionError):
        policy.permit('revoked-machine', 'stand', clock.monotonic + 1.)
    assert load().authorized_now()


@pytest.mark.parametrize('change', ['content', 'permissions', 'ownership', 'missing'])
def test_persistent_permission_still_revokes_on_file_or_trust_change(persistent_operator_session, change):
    clock, manifest, path, _, load, ownership, _ = persistent_operator_session
    policy = load()
    original = path.read_bytes()
    assert policy.authorized_now()
    if change == 'content':
        path.write_text(json.dumps(manifest, indent=2))
    elif change == 'permissions':
        path.chmod(0o666)
    elif change == 'ownership':
        ownership['uid'] = 1000
    else:
        path.unlink()
    assert not policy.authorized_now()
    with pytest.raises(PermissionError):
        policy.permit('revoked-persistent', 'stand', clock.monotonic + 1.)
    ownership['uid'] = 0
    path.write_bytes(original)
    path.chmod(0o600)
    assert not policy.authorized_now()
    assert load().authorized_now()


def test_persistent_permission_can_be_explicitly_revoked_without_time_or_file_change(persistent_operator_session):
    clock, _, _, _, load, _, _ = persistent_operator_session
    policy = load()
    assert policy.authorized_now()
    policy.revoke()
    assert not policy.authorized_now()
    clock.monotonic += 86400
    clock.wall += 86400
    with pytest.raises(PermissionError):
        policy.permit('explicitly-revoked', 'lie', clock.monotonic + 1.)


@pytest.mark.parametrize(('field', 'value'), [
    ('schema', SCHEMA), ('execution_authorized', False), ('execution_authorized', 1),
    ('lifetime', 'forever'), ('lifetime', None), ('lifetime', True),
    ('expires_at_ms', 1060000), ('expires_at_ms', 'null'), ('expires_at_ms', 0),
    ('robot_identity', 'd' * 64), ('robot_identity', ''), ('robot_identity', None),
    ('allowed_actions', ['stand', 'velocity']), ('allowed_actions', []),
    ('allowed_actions', ['stand', 'stand']), ('allowed_actions', 'stand'),
    ('issued_at_ms', True), ('issued_at_ms', None),
    ('operator_present', False), ('handheld_estop_verified', False),
    ('no_competing_commands', False), ('provenance', 'vendor_owned'),
])
def test_persistent_schema_is_explicit_and_preserves_operator_and_action_boundaries(
        persistent_operator_session, field, value):
    _, manifest, _, write, load, _, _ = persistent_operator_session
    manifest[field] = value
    write()
    assert not load().authorized_now()


@pytest.mark.parametrize('field', ['schema', 'execution_authorized', 'session_id', 'robot_identity',
    'allowed_actions', 'issued_at_ms', 'expires_at_ms', 'lifetime', 'operator_present',
    'handheld_estop_verified', 'no_competing_commands', 'provenance'])
def test_persistent_manifest_requires_every_field(persistent_operator_session, field):
    _, manifest, _, write, load, _, _ = persistent_operator_session
    manifest.pop(field)
    write()
    assert not load().authorized_now()


def test_persistent_manifest_cannot_mix_in_legacy_boot_or_unknown_grants(persistent_operator_session):
    _, manifest, _, write, load, _, _ = persistent_operator_session
    for extra in ({'robot_session': ROBOT_SESSION}, {'owned': True}, {'extra': 'PRIVATE_NOT_PUBLIC'}):
        write({**manifest, **extra})
        policy = load()
        assert not policy.authorized_now()
        assert 'PRIVATE_NOT_PUBLIC' not in json.dumps(policy.public_status())


def test_persistent_manifest_cannot_be_selected_or_granted_by_canvas_config(persistent_operator_session):
    _, manifest, path, _, _, _, _ = persistent_operator_session
    configured = authorization.load_operator_authorization({
        'operator_authorization': manifest, 'lifetime': 'until_revoked',
        'S10_OPERATOR_AUTHORIZATION_FILE': str(path), 'execution_authorized': True})
    assert configured.configured is False and not configured.authorized_now()


def test_persistent_manifest_duplicate_fields_are_not_accepted(persistent_operator_session):
    _, manifest, path, _, load, _, _ = persistent_operator_session
    path.write_text(json.dumps(manifest)[:-1] + ',"lifetime":"until_revoked"}')
    policy = load()
    assert not policy.authorized_now()


def test_persistent_permission_does_not_expand_action_scope_or_accept_invalid_deadlines(persistent_operator_session):
    clock, manifest, _, write, load, _, _ = persistent_operator_session
    manifest['allowed_actions'] = ['lie']
    write()
    policy = load()
    assert policy.authorized_now()
    for action in ('stand', 'velocity', 'stopmotion', 'charging', '17'):
        with pytest.raises((PermissionError, ValueError)):
            policy.permit('persistent-out-of-scope', action, clock.monotonic + 1.)
    for deadline in (clock.monotonic, float('inf'), float('nan'), True, None):
        with pytest.raises((PermissionError, ValueError)):
            policy.permit('persistent-invalid-deadline', 'lie', deadline)
    assert policy.permit('persistent-scoped', 'lie', clock.monotonic + 1.)['action'] == 'lie'


def test_persistent_permit_producer_enforces_the_same_fifteen_second_action_bound(
        persistent_operator_session):
    clock, _, _, _, load, _, _ = persistent_operator_session
    policy = load()
    at_limit = policy.permit('bounded-fifteen-seconds', 'stand', clock.monotonic + 15.)
    assert at_limit['action_deadline_monotonic'] == clock.monotonic + 15.
    assert validate_operator_permit(at_limit, now=clock.monotonic) == at_limit
    for extra in (.001, 285.):
        with pytest.raises((PermissionError, ValueError)):
            policy.permit('over-fifteen-seconds', 'stand', clock.monotonic + 15. + extra)
    assert policy.authorized_now(), 'an invalid action deadline must not revoke the persistent grant'


def test_legacy_issue_timestamp_and_original_session_deadline_remain_enforced(operator_session):
    clock, manifest, _, write, load, _ = operator_session
    policy = load()
    # V1 remains governed by its original finite session rather than the new
    # V2 producer rule; transport and existing caller limits still apply.
    within_original = policy.permit('legacy-session-window', 'stand', clock.monotonic + 16.)
    assert validate_operator_permit(within_original, now=clock.monotonic) == within_original
    with pytest.raises(PermissionError):
        policy.permit('legacy-past-session', 'stand', clock.monotonic + 61.)
    manifest['issued_at_ms'] = int(clock.wall * 1000) + 1
    write()
    assert not load().authorized_now(), 'a future-issued V1 grant must still be rejected'


def test_legacy_manifest_cannot_become_non_expiring_by_adding_persistent_fields(operator_session):
    _, manifest, _, write, load, _ = operator_session
    for extra in ({'lifetime': 'until_revoked'}, {'robot_identity': MACHINE_IDENTITY}, {'expires_at_ms': None}):
        write({**manifest, **extra})
        assert not load().authorized_now()


def test_machine_identity_is_hash_of_protected_local_id_and_not_boot_session(operator_session, monkeypatch):
    _, _, grant_path, _, _, ownership = operator_session
    identity_path = grant_path.with_name('machine-id')
    value = '0123456789abcdef0123456789abcdef'
    identity_path.write_text(value + '\n')
    identity_path.chmod(0o400)
    monkeypatch.setenv('S10_OPERATOR_MACHINE_ID_FILE', str(identity_path))
    expected = hashlib.sha256(('s10-nos-machine:' + value).encode()).hexdigest()
    assert authorization.machine_identity() == expected
    monkeypatch.setattr(authorization, 'robot_session_identity', lambda: 'c' * 64)
    assert authorization.machine_identity() == expected
    identity_path.chmod(0o666)
    with pytest.raises((OSError, ValueError)):
        authorization.machine_identity()
    identity_path.chmod(0o400)
    ownership['uid'] = 1000
    with pytest.raises((OSError, ValueError)):
        authorization.machine_identity()


@pytest.mark.parametrize('value', ['', 'short', 'g' * 32, '0' * 31, '0' * 33, '0' * 32 + '\nEXTRA'])
def test_machine_identity_rejects_missing_or_malformed_local_id(operator_session, monkeypatch, value):
    _, _, grant_path, _, _, _ = operator_session
    identity_path = grant_path.with_name('machine-id')
    identity_path.write_text(value)
    identity_path.chmod(0o400)
    monkeypatch.setenv('S10_OPERATOR_MACHINE_ID_FILE', str(identity_path))
    with pytest.raises((OSError, ValueError)):
        authorization.machine_identity()


def test_persistent_grant_detects_actual_machine_file_change_and_does_not_resume_after_restore(
        operator_session, monkeypatch):
    clock, manifest, grant_path, write, _, _ = operator_session
    identity_path = grant_path.with_name('machine-id')
    original = '0123456789abcdef0123456789abcdef'
    identity_path.write_text(original + '\n')
    identity_path.chmod(0o400)
    monkeypatch.setenv('S10_OPERATOR_MACHINE_ID_FILE', str(identity_path))
    manifest.pop('robot_session')
    manifest.update(schema=PERSISTENT_SCHEMA, robot_identity=authorization.machine_identity(),
                    lifetime='until_revoked', expires_at_ms=None)
    write()
    policy = authorization.OperatorAuthorization(str(grant_path), clock=clock.mono, wall_time=clock.time)
    assert policy.authorized_now()
    identity_path.chmod(0o600)
    identity_path.write_text('fedcba9876543210fedcba9876543210\n')
    identity_path.chmod(0o400)
    assert not policy.authorized_now()
    identity_path.chmod(0o600)
    identity_path.write_text(original + '\n')
    identity_path.chmod(0o400)
    assert not policy.authorized_now()
    with pytest.raises(PermissionError):
        policy.permit('machine-file-restored', 'stand', clock.monotonic + 1.)
