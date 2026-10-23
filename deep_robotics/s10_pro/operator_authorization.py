"""Protected, expiring operator supervision; never a vendor ownership grant."""
from __future__ import annotations

import hashlib
import json
import math
import os
import stat
import time
from pathlib import Path

SCHEMA = 's10_pro.operator_authorization/1'
PERMIT_SCHEMA = 's10_pro.operator_permit/1'
KEYS = {'schema', 'execution_authorized', 'session_id', 'robot_session',
        'allowed_actions', 'issued_at_ms', 'expires_at_ms', 'operator_present',
        'handheld_estop_verified', 'no_competing_commands', 'provenance'}


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


class OperatorAuthorization:
    def __init__(self, path=None, *, clock=time.monotonic, wall_time=time.time,
                 robot_session=None):
        self.configured = path is not None
        self._path = Path(path) if path is not None else None
        self._clock, self._wall = clock, wall_time
        self._manifest = {}
        self._digest = None
        self._revoked = False
        self._valid = False
        self._reason = 'operator_authorization_not_configured'
        self._issued = self._expires = 0.
        if not self.configured:
            return
        try:
            self._robot_session = robot_session if robot_session is not None else robot_session_identity()
            raw = _read(self._path)
            value = json.loads(raw, object_pairs_hook=_object,
                parse_constant=lambda _: (_ for _ in ()).throw(ValueError('nonfinite JSON')))
            self._validate(value)
            now, mono = self._wall() * 1000, self._clock()
            if not value['issued_at_ms'] <= now < value['expires_at_ms']:
                raise ValueError('operator authorization is not current')
            self._issued = mono - (now - value['issued_at_ms']) / 1000
            # Independent wall-to-monotonic arithmetic can round a one-hour
            # window just above 3600 seconds. Round inward to the original
            # duration as well; never extend approval or relax its validator.
            duration = (value['expires_at_ms'] - value['issued_at_ms']) / 1000
            self._expires = min(mono + (value['expires_at_ms'] - now) / 1000,
                                self._issued + duration)
            self._manifest, self._digest = value, hashlib.sha256(raw).digest()
            self._valid, self._reason = True, 'operator_supervision_authorized'
        except (OSError, ValueError, TypeError):
            self._reason = 'invalid_operator_authorization'

    def _validate(self, value):
        if not isinstance(value, dict) or set(value) != KEYS or value['schema'] != SCHEMA:
            raise ValueError('invalid operator authorization schema')
        for key in ('execution_authorized', 'operator_present', 'handheld_estop_verified', 'no_competing_commands'):
            if value[key] is not True:
                raise ValueError('explicit operator facts required')
        for key in ('session_id', 'robot_session', 'provenance'):
            if not isinstance(value[key], str) or not value[key].strip() or len(value[key]) > 512:
                raise ValueError('invalid operator identity/provenance')
        if len(value['session_id']) > 128 or value['robot_session'] != self._robot_session:
            raise ValueError('operator authorization belongs to another boot')
        if value['provenance'] != 'onsite_operator_confirmation':
            raise ValueError('operator supervision provenance required')
        actions = value['allowed_actions']
        if (not isinstance(actions, list) or not actions or any(type(a) is not str for a in actions)
                or len(set(actions)) != len(actions) or not set(actions) <= {'stand', 'lie'}):
            raise ValueError('only explicit finite posture actions are permitted')
        for key in ('issued_at_ms', 'expires_at_ms'):
            if type(value[key]) is not int or value[key] < 0:
                raise ValueError('integer authorization timestamps required')
        if not 0 < value['expires_at_ms'] - value['issued_at_ms'] <= 3600000:
            raise ValueError('operator session must expire within one hour')

    def authorized_now(self):
        if not self._valid or self._revoked:
            return False
        try:
            raw = _read(self._path)
            valid = (hashlib.sha256(raw).digest() == self._digest
                and robot_session_identity() == self._robot_session
                and self._issued <= self._clock() < self._expires
                and self._manifest['issued_at_ms'] <= self._wall() * 1000 < self._manifest['expires_at_ms'])
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
            'vendor_ownership_claimed': False, 'automatic_posture_stop_verified': False,
            'physical_stop': 'onsite_operator_handheld_estop', 'canvas_configurable': False}

    def revoke(self, reason='operator_authorization_revoked'):
        self._revoked, self._reason = True, reason

    def permit(self, request_id, action, deadline):
        if (not self.authorized_now() or action not in self._manifest['allowed_actions']
                or not isinstance(request_id, str) or not request_id.strip() or len(request_id) > 128
                or type(deadline) not in (int, float) or not math.isfinite(deadline)
                or not self._clock() < deadline <= self._expires):
            raise PermissionError('no current scoped operator authorization')
        return {'schema': PERMIT_SCHEMA, 'session_id': self._manifest['session_id'],
            'robot_session': self._robot_session, 'request_id': request_id, 'action': action,
            'issued_monotonic': self._issued, 'expires_monotonic': self._expires,
            'action_deadline_monotonic': float(deadline)}


def load_operator_authorization(config=None):
    # The canvas/config dictionary intentionally cannot grant this authorization.
    return OperatorAuthorization(os.environ.get('S10_OPERATOR_AUTHORIZATION_FILE') or None)
