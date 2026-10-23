"""Fictional operator sessions only; no ROS, network or robot command I/O."""
import copy
import json
import math
import os

import pytest

from deep_robotics.s10_pro import operator_authorization as authorization
from deep_robotics.s10_pro.motion_dds import AuthorityUnavailable, validate_operator_permit


SCHEMA = 's10_pro.operator_authorization/1'
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
