"""Protected operator authorization, including explicit until-revoked grants."""
from __future__ import annotations

import hashlib
import json
import math
import os
import stat
import threading
import time
import uuid
from pathlib import Path

SCHEMA = 's10_pro.operator_authorization/1'
PERMIT_SCHEMA = 's10_pro.operator_permit/1'
PERSISTENT_SCHEMA = 's10_pro.operator_authorization/2'
PERSISTENT_PERMIT_SCHEMA = 's10_pro.operator_permit/2'
MAX_PERSISTENT_ACTION_S = 15.
KEYS = {'schema', 'execution_authorized', 'session_id', 'robot_session',
        'allowed_actions', 'issued_at_ms', 'expires_at_ms', 'operator_present',
        'handheld_estop_verified', 'no_competing_commands', 'provenance'}
PERSISTENT_KEYS = (KEYS - {'robot_session'}) | {'robot_identity', 'lifetime'}

PREPARE_POLICY_SCHEMA = 's10_pro.prepare_authorization_policy/1'
PREPARE_POLICY_KEYS = {'schema', 'enabled', 'policy_id', 'robot_identity',
    'allowed_actions', 'session_duration_s', 'activation', 'provenance'}
PREPARE_IDENTITY_PATH = '/run/s10/prepare-machine-id'


def prepare_machine_identity():
    value = _read(Path(PREPARE_IDENTITY_PATH)).decode('ascii').strip()
    if len(value) != 32 or any(c not in '0123456789abcdef' for c in value):
        raise ValueError('machine identity unavailable')
    return hashlib.sha256(('s10-prepare-machine:' + value).encode()).hexdigest()


def robot_session_identity():
    """Identify this NOS host boot, without claiming motion firmware identity."""
    value = Path('/proc/sys/kernel/random/boot_id').read_text().strip()
    if len(value) != 36 or any(c not in '0123456789abcdef-' for c in value):
        raise ValueError('host boot identity unavailable')
    return hashlib.sha256(('s10-nos-boot:' + value).encode()).hexdigest()


def _object(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError('duplicate authorization key')
        result[key] = value
    return result


def _read(path):
    if not path.is_absolute():
        raise ValueError('absolute authorization path required')
    fd = os.open(path, os.O_RDONLY | os.O_NOFOLLOW | os.O_CLOEXEC | os.O_NONBLOCK)
    try:
        info = os.fstat(fd)
        if not stat.S_ISREG(info.st_mode) or info.st_uid != 0 or info.st_mode & 0o022:
            raise ValueError('protected root-owned regular authorization required')
        raw = os.read(fd, 16385)
        if len(raw) > 16384:
            raise ValueError('authorization exceeds size bound')
        return raw
    finally:
        os.close(fd)


def machine_identity(path=None):
    """Hash the read-only mounted NOS machine identity, without exposing it."""
    selected = Path(path or os.environ.get('S10_OPERATOR_MACHINE_ID_FILE')
                    or '/run/s10/robot-machine-id')
    value = _read(selected).decode('ascii').strip()
    if len(value) != 32 or any(c not in '0123456789abcdef' for c in value):
        raise ValueError('valid machine identity required')
    return hashlib.sha256(('s10-nos-machine:' + value).encode()).hexdigest()


class OperatorAuthorization:
    def __init__(self, path=None, *, clock=time.monotonic, wall_time=time.time,
                 robot_session=None, robot_identity=None):
        self.configured = path is not None
        self._path = Path(path) if path is not None else None
        self._clock, self._wall = clock, wall_time
        self._manifest = {}
        self._digest = None
        self._revoked = False
        self._valid = False
        self._reason = 'operator_authorization_not_configured'
        self._issued = self._expires = 0.
        self._persistent = False
        if not self.configured:
            return
        try:
            self._robot_session = robot_session if robot_session is not None else robot_session_identity()
            raw = _read(self._path)
            value = json.loads(raw, object_pairs_hook=_object,
                parse_constant=lambda _: (_ for _ in ()).throw(ValueError('nonfinite JSON')))
            self._persistent = isinstance(value, dict) and value.get('schema') == PERSISTENT_SCHEMA
            if self._persistent:
                self._robot_identity = robot_identity if robot_identity is not None else machine_identity()
            self._validate(value)
            now, mono = self._wall() * 1000, self._clock()
            if not self._persistent and not value['issued_at_ms'] <= now < value['expires_at_ms']:
                raise ValueError('operator authorization is not current')
            if self._persistent:
                # A process-local receipt for the private command channel, not
                # a renewed grant or a manufacturer ownership lease. Wall time
                # is audit data: an RTC reset cannot expire a persistent grant.
                self._issued, self._expires = mono, None
            else:
                self._issued = mono - (now - value['issued_at_ms']) / 1000
                duration = (value['expires_at_ms'] - value['issued_at_ms']) / 1000
                self._expires = min(mono + (value['expires_at_ms'] - now) / 1000,
                                    self._issued + duration)
            self._manifest, self._digest = value, hashlib.sha256(raw).digest()
            self._valid, self._reason = True, 'operator_supervision_authorized'
        except (OSError, ValueError, TypeError):
            self._reason = 'invalid_operator_authorization'

    def _validate(self, value):
        required = PERSISTENT_KEYS if self._persistent else KEYS
        schema = PERSISTENT_SCHEMA if self._persistent else SCHEMA
        if not isinstance(value, dict) or set(value) != required or value['schema'] != schema:
            raise ValueError('invalid operator authorization schema')
        for key in ('execution_authorized', 'operator_present', 'handheld_estop_verified', 'no_competing_commands'):
            if value[key] is not True:
                raise ValueError('explicit operator facts required')
        identity_key = 'robot_identity' if self._persistent else 'robot_session'
        for key in ('session_id', identity_key, 'provenance'):
            if not isinstance(value[key], str) or not value[key].strip() or len(value[key]) > 512:
                raise ValueError('invalid operator identity/provenance')
        identity = self._robot_identity if self._persistent else self._robot_session
        if len(value['session_id']) > 128 or value[identity_key] != identity:
            raise ValueError('operator authorization belongs to another machine/session')
        if value['provenance'] != 'onsite_operator_confirmation':
            raise ValueError('operator supervision provenance required')
        actions = value['allowed_actions']
        if (not isinstance(actions, list) or not actions or any(type(a) is not str for a in actions)
                or len(set(actions)) != len(actions)
                or not set(actions) <= ({'stand', 'lie'} if self._persistent else
                    {'stand', 'lie', 'move_mode', 'forward_30cm', 'backward_30cm',
                     'turn_left', 'turn_right', 'relative_forward', 'relative_backward',
                     'relative_left', 'relative_right', 'relative_turn_left',
                     'relative_turn_right'})):
            raise ValueError('only explicit finite posture actions are permitted')
        for key in (('issued_at_ms',) if self._persistent else ('issued_at_ms', 'expires_at_ms')):
            if type(value[key]) is not int or value[key] < 0:
                raise ValueError('integer authorization timestamps required')
        if self._persistent:
            if value['lifetime'] != 'until_revoked' or value['expires_at_ms'] is not None:
                raise ValueError('persistent authorization must explicitly have no expiry')
        elif not 0 < value['expires_at_ms'] - value['issued_at_ms'] <= 3600000:
            raise ValueError('operator session must expire within one hour')

    def authorized_now(self):
        if not self._valid or self._revoked:
            return False
        try:
            raw = _read(self._path)
            if self._persistent:
                identity_valid = machine_identity() == self._robot_identity
                time_valid = self._clock() >= self._issued
            else:
                identity_valid = robot_session_identity() == self._robot_session
                time_valid = (self._issued <= self._clock() < self._expires
                    and self._manifest['issued_at_ms'] <= self._wall() * 1000 < self._manifest['expires_at_ms'])
            valid = hashlib.sha256(raw).digest() == self._digest and identity_valid and time_valid
        except (OSError, ValueError, TypeError):
            valid = False
        if not valid:
            self._revoked, self._reason = True, 'operator_authorization_revoked_or_expired'
        return valid

    def public_status(self):
        authorized = self.authorized_now()
        return {'configured': self.configured, 'authorized': authorized,
            'reason': self._reason, 'mode': 'operator_supervised_posture_test',
            'session_id': self._manifest.get('session_id'),
            'allowed_actions': list(self._manifest.get('allowed_actions', [])) if authorized else [],
            'expires_at_ms': self._manifest.get('expires_at_ms'),
            'non_expiring': self._persistent and self._valid,
            'lifetime': 'until_revoked' if self._persistent else 'bounded_session',
            'authorization_survives_reboot': self._persistent,
            'vendor_ownership_claimed': False, 'automatic_posture_stop_verified': False,
            'physical_stop': 'onsite_operator_handheld_estop', 'canvas_configurable': False}

    def revoke(self, reason='operator_authorization_revoked'):
        self._revoked, self._reason = True, reason

    def permit(self, request_id, action, deadline):
        if (not self.authorized_now() or action not in self._manifest['allowed_actions']
                or not isinstance(request_id, str) or not request_id.strip() or len(request_id) > 128
                or type(deadline) not in (int, float) or not math.isfinite(deadline)
                or not self._clock() < deadline
                or (self._persistent and deadline > self._clock() + MAX_PERSISTENT_ACTION_S)
                or (self._expires is not None and deadline > self._expires)):
            raise PermissionError('no current scoped operator authorization')
        result = {'schema': PERSISTENT_PERMIT_SCHEMA if self._persistent else PERMIT_SCHEMA,
            'session_id': self._manifest['session_id'],
            'robot_session': self._robot_session, 'request_id': request_id, 'action': action,
            'issued_monotonic': self._issued, 'expires_monotonic': self._expires,
            'action_deadline_monotonic': float(deadline)}
        if self._persistent:
            result['lifetime'] = 'until_revoked'
        return result


def load_operator_authorization(config=None):
    # The canvas/config dictionary intentionally cannot grant this authorization.
    path = os.environ.get('S10_OPERATOR_AUTHORIZATION_FILE') or None
    if path is not None:
        try:
            value = json.loads(_read(Path(path)), object_pairs_hook=_object)
            if isinstance(value, dict) and value.get('schema') == PREPARE_POLICY_SCHEMA:
                return PrepareAuthorization(path)
        except (OSError, ValueError, TypeError):
            pass
    return OperatorAuthorization(path)


class PrepareAuthorization:
    """Root policy permits explicit prepare to open one bounded local session.

    Loading the policy, rendering status, and executing an action never renew
    authorization. Only prepare_session activates it; the canvas backend must
    first verify stationary telemetry, enabled joints, HES and faults. Sessions
    are memory-only, bound to this machine/boot, and reset on process restart.
    """
    def __init__(self, path, *, clock=time.monotonic, wall_time=time.time):
        self.configured = True
        self.supports_prepare = False
        self._path, self._clock, self._wall = Path(path), clock, wall_time
        self._lock = threading.RLock()
        self._manifest, self._session = {}, None
        self._digest = None
        self._revoked = False
        self._reason = 'invalid_prepare_authorization_policy'
        try:
            raw = _read(self._path)
            value = json.loads(raw, object_pairs_hook=_object,
                parse_constant=lambda _: (_ for _ in ()).throw(ValueError('nonfinite JSON')))
            if (not isinstance(value, dict) or set(value) != PREPARE_POLICY_KEYS
                    or value['schema'] != PREPARE_POLICY_SCHEMA
                    or value['enabled'] is not True
                    or value['activation'] != 'explicit_prepare'
                    or value['provenance'] != 'owner_enabled_canvas_prepare'
                    or not isinstance(value['policy_id'], str) or not value['policy_id'].strip()
                    or len(value['policy_id']) > 96
                    or type(value['session_duration_s']) is not int
                    or not 60 <= value['session_duration_s'] <= 3600):
                raise ValueError('invalid prepare policy')
            actions = value['allowed_actions']
            if (not isinstance(actions, list) or not actions
                    or any(type(a) is not str for a in actions)
                    or len(set(actions)) != len(actions)
                    or not set(actions) <= {'stand', 'lie', 'move_mode', 'forward_30cm', 'backward_30cm',
                                            'turn_left', 'turn_right', 'relative_forward',
                                            'relative_backward', 'relative_left', 'relative_right',
                                            'relative_turn_left', 'relative_turn_right'}):
                raise ValueError('invalid prepare action scope')
            self._identity = prepare_machine_identity()
            if value['robot_identity'] != self._identity:
                raise ValueError('prepare policy belongs to another machine')
            self._boot = robot_session_identity()
            self._manifest, self._digest = value, hashlib.sha256(raw).digest()
            self.supports_prepare = True
            self._reason = 'prepare_required'
        except (OSError, ValueError, TypeError):
            pass

    def _policy_current(self):
        if not self.supports_prepare or self._revoked:
            return False
        try:
            valid = (hashlib.sha256(_read(self._path)).digest() == self._digest
                and prepare_machine_identity() == self._identity
                and robot_session_identity() == self._boot)
        except (OSError, ValueError, TypeError):
            valid = False
        if not valid:
            self.revoke('prepare_policy_revoked_or_changed')
        return valid

    def authorized_now(self):
        with self._lock:
            if not self._policy_current() or self._session is None:
                return False
            session = self._session
            valid = (session['issued_monotonic'] <= self._clock() < session['expires_monotonic']
                and session['issued_at_ms'] <= self._wall() * 1000 < session['expires_at_ms'])
            if not valid:
                self._session = None
                self._reason = 'prepare_required_session_expired'
            return valid

    def prepare_session(self):
        """Called by the idle prepare path after its live telemetry checks."""
        with self._lock:
            if not self._policy_current():
                raise PermissionError('prepare policy unavailable')
            now, mono = int(self._wall() * 1000), self._clock()
            duration = self._manifest['session_duration_s']
            self._session = {'session_id': 'canvas-prepare-' + uuid.uuid4().hex,
                'issued_at_ms': now, 'expires_at_ms': now + duration * 1000,
                'issued_monotonic': mono, 'expires_monotonic': mono + duration}
            self._reason = 'operator_session_prepared'
            return self.public_status()

    def clear_session(self, reason='prepare_required'):
        with self._lock:
            self._session, self._reason = None, reason

    def revoke(self, reason='operator_authorization_revoked'):
        with self._lock:
            self._revoked = True
            self.clear_session(reason)

    def public_status(self):
        with self._lock:
            authorized = self.authorized_now()
            session = self._session or {}
            return {'configured': True, 'authorized': authorized, 'reason': self._reason,
                'mode': 'operator_supervised_posture_test',
                'session_id': session.get('session_id'),
                'allowed_actions': list(self._manifest.get('allowed_actions', [])) if authorized else [],
                'policy_allowed_actions': list(self._manifest.get('allowed_actions', [])),
                'expires_at_ms': session.get('expires_at_ms'),
                'session_duration_s': self._manifest.get('session_duration_s'),
                'renewal_action': 'prepare', 'requires_explicit_prepare': True,
                'vendor_ownership_claimed': False, 'automatic_posture_stop_verified': False,
                'physical_stop': 'onsite_operator_handheld_estop', 'canvas_configurable': False}

    def permit(self, request_id, action, deadline):
        with self._lock:
            if (not self.authorized_now() or action not in self._manifest['allowed_actions']
                    or not isinstance(request_id, str) or not request_id.strip() or len(request_id) > 128
                    or type(deadline) not in (int, float) or not math.isfinite(deadline)
                    or not self._clock() < deadline <= self._session['expires_monotonic']):
                raise PermissionError('no current scoped prepare session')
            return {'schema': PERMIT_SCHEMA, 'session_id': self._session['session_id'],
                'robot_session': self._boot, 'request_id': request_id, 'action': action,
                'issued_monotonic': self._session['issued_monotonic'],
                'expires_monotonic': self._session['expires_monotonic'],
                'action_deadline_monotonic': float(deadline)}
