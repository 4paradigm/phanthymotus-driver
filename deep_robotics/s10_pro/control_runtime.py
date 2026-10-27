"""Explicit deployment authorization and machine-adapter binding.

The canvas cannot load an adapter or grant authorization. Construction does not
open ROS or start a command worker. A deployment must provide a protected,
time-limited manifest matching the exact adapter contract and machine identity.
A vendor adapter supplies already received authority observations; this module
never infers ownership, changes mode, or manufactures a receipt/lease timestamp.
"""
from __future__ import annotations

import copy
import hashlib
import importlib
import json
import math
import os
from pathlib import Path
import re
import stat
import time

SCHEMA = 's10_pro.execution_authorization/1'
MAX_MANIFEST_BYTES = 65536
MAX_AUTHORIZATION_MS = 4 * 60 * 60 * 1000
EXECUTABLE_ACTIONS = frozenset(('stand', 'lie', 'velocity'))
_MANIFEST_KEYS = {'schema', 'execution_authorized', 'adapter', 'robot_identity',
                  'firmware_identity', 'contract_sha256', 'allowed_actions',
                  'issued_at_ms', 'expires_at_ms'}
_ADAPTER = re.compile(r'[A-Za-z_][A-Za-z0-9_]*(?:\.[A-Za-z_][A-Za-z0-9_]*)*:[A-Za-z_][A-Za-z0-9_]*\Z')


class BindingError(ValueError):
    """Only the bounded code is safe for public diagnostics."""
    def __init__(self, code):
        self.code = code
        super().__init__(code)


def _strict_json(raw):
    def pairs(items):
        result = {}
        for key, value in items:
            if key in result:
                raise BindingError('authorization_duplicate_field')
            result[key] = value
        return result
    return json.loads(raw, object_pairs_hook=pairs,
                      parse_constant=lambda value: (_ for _ in ()).throw(BindingError('authorization_nonfinite')))


def contract_fingerprint(contract):
    """Bind deployment approval to the full, unmasked machine contract."""
    encoded = json.dumps(contract, sort_keys=True, ensure_ascii=True,
                         separators=(',', ':'), allow_nan=False).encode('ascii')
    return hashlib.sha256(encoded).hexdigest()


def read_authorization(path, *, wall_time=time.time, owner_uid=0):
    """Read a regular, protected local file without following a symlink."""
    try:
        fd = os.open(path, os.O_RDONLY | os.O_NOFOLLOW | os.O_NONBLOCK)
    except OSError as exc:
        raise BindingError('authorization_file_unavailable') from exc
    try:
        info = os.fstat(fd)
        if not stat.S_ISREG(info.st_mode) or info.st_size > MAX_MANIFEST_BYTES:
            raise BindingError('authorization_file_invalid')
        if info.st_uid != owner_uid or info.st_mode & (stat.S_IWGRP | stat.S_IWOTH):
            raise BindingError('authorization_file_permissions')
        raw = os.read(fd, MAX_MANIFEST_BYTES + 1)
        if len(raw) > MAX_MANIFEST_BYTES:
            raise BindingError('authorization_file_invalid')
    finally:
        os.close(fd)
    try:
        manifest = _strict_json(raw)
    except BindingError:
        raise
    except (ValueError, UnicodeError) as exc:
        raise BindingError('authorization_json_invalid') from exc
    if not isinstance(manifest, dict) or set(manifest) - _MANIFEST_KEYS:
        raise BindingError('authorization_shape_invalid')
    if manifest.get('schema') != SCHEMA or type(manifest.get('execution_authorized')) is not bool:
        raise BindingError('authorization_schema_invalid')
    if not manifest['execution_authorized']:
        if set(manifest) != {'schema', 'execution_authorized'}:
            raise BindingError('disabled_authorization_fields_invalid')
        return manifest, hashlib.sha256(raw).hexdigest()
    if set(manifest) != _MANIFEST_KEYS:
        raise BindingError('authorization_fields_missing')
    for key in ('robot_identity', 'firmware_identity'):
        value = manifest[key]
        if not isinstance(value, str) or not value.strip() or len(value) > 256:
            raise BindingError('authorization_identity_invalid')
    if not isinstance(manifest['adapter'], str) or not _ADAPTER.fullmatch(manifest['adapter']):
        raise BindingError('authorization_adapter_invalid')
    digest = manifest['contract_sha256']
    if not isinstance(digest, str) or re.fullmatch(r'[0-9a-f]{64}', digest) is None:
        raise BindingError('authorization_contract_digest_invalid')
    actions = manifest['allowed_actions']
    if (not isinstance(actions, list) or not actions or any(type(a) is not str for a in actions)
            or len(actions) != len(set(actions)) or not set(actions) <= EXECUTABLE_ACTIONS):
        raise BindingError('authorization_action_scope_invalid')
    issued, expires = manifest['issued_at_ms'], manifest['expires_at_ms']
    if type(issued) is not int or type(expires) is not int or not 0 <= issued < expires:
        raise BindingError('authorization_time_invalid')
    now = wall_time() * 1000
    if not math.isfinite(now) or issued > now + 100 or expires <= now or expires - issued > MAX_AUTHORIZATION_MS:
        raise BindingError('authorization_expired_or_invalid')
    return manifest, hashlib.sha256(raw).hexdigest()


def _load_adapter(reference, config):
    module_name, factory_name = reference.split(':', 1)
    factory = getattr(importlib.import_module(module_name), factory_name)
    if not callable(factory):
        raise BindingError('adapter_factory_invalid')
    # A deployment selects trusted, locally installed code. The adapter is
    # responsible for documented vendor I/O; none is implemented or guessed here.
    return factory(copy.deepcopy(config))


class RuntimeBinding:
    def __init__(self, *, reason='authorization_not_configured', path=None, manifest=None,
                 manifest_digest=None, contract=None, authority_provider=None,
                 posture_stop=None, wall_time=time.time, clock=time.monotonic, owner_uid=0):
        self._reason = reason
        self._path, self._manifest = path, manifest
        self._manifest_digest = manifest_digest
        self._wall, self._clock, self._owner = wall_time, clock, owner_uid
        self._authority = authority_provider
        self.posture_stop = posture_stop
        self.contract = copy.deepcopy(contract or {})
        self.authorized = reason == 'bound'
        self._closed = False
        self._revoked = False
        remaining = ((manifest["expires_at_ms"] / 1000 - wall_time()) if reason == "bound" else 0)
        self._monotonic_deadline = clock() + max(0., remaining)
        self._last_authority_reason = 'not_observed'
        self._adapter_close = None

    def authorized_now(self):
        if not self.authorized or self._closed or self._revoked:
            return False
        if self._clock() >= self._monotonic_deadline:
            self._reason = 'authorization_expired_or_invalid'
            self._revoked = True
            return False
        try:
            manifest, digest = read_authorization(self._path, wall_time=self._wall, owner_uid=self._owner)
            if digest != self._manifest_digest or manifest != self._manifest:
                self._reason = 'authorization_changed_restart_required'
                self._revoked = True
                return False
            return manifest['execution_authorized'] is True
        except BindingError as exc:
            self._reason = exc.code
            self._revoked = True
            return False

    def authority_provider(self):
        if not self.authorized_now() or not callable(self._authority):
            self._last_authority_reason = 'deployment_not_authorized'
            return {}
        try:
            raw = self._authority()
            if not self.authorized_now():
                raise BindingError('deployment_not_authorized')
            if not isinstance(raw, dict):
                raise BindingError('authority_snapshot_invalid')
            if (raw.get('robot_identity') != self._manifest['robot_identity']
                    or raw.get('firmware_identity') != self._manifest['firmware_identity']):
                raise BindingError('authority_machine_identity_mismatch')
            received, expires = raw.get('received_monotonic'), raw.get('expires_monotonic')
            numeric = lambda x: type(x) in (int, float) and math.isfinite(x)
            now = self._clock()
            if (not numeric(received) or not numeric(expires) or not 0 <= now - received <= .5
                    or not now < expires <= received + .5):
                raise BindingError('authority_observation_expired')
            # Age increases from the ORIGINAL receipt. Do not renew receipt or
            # expiry during a query, on a heartbeat, or from MOTION_INFO.
            age = now - received
            original_age = raw.get('age_s')
            if not numeric(original_age) or not 0 <= original_age <= .5 or abs(age - original_age) > .1:
                raise BindingError('authority_observation_clock_invalid')
            actions = raw.get('allowed_actions')
            if not isinstance(actions, list) or any(type(a) is not str for a in actions):
                raise BindingError('authority_action_scope_invalid')
            permitted = sorted(set(actions) & set(self._manifest['allowed_actions']))
            if raw.get('owned') is not True or raw.get('fresh') is not True:
                raise BindingError('authority_not_owned')
            self._last_authority_reason = 'valid_received_snapshot'
            return {**copy.deepcopy(raw), 'age_s': age, 'allowed_actions': permitted}
        except BindingError as exc:
            self._last_authority_reason = exc.code
            return {}
        except Exception:
            self._last_authority_reason = 'authority_adapter_unavailable'
            return {}

    def public_status(self):
        active = self.authorized_now()
        return {'configured': self._path is not None, 'authorized': active,
                'reason': self._reason, 'adapter_bound': callable(self._authority),
                'contract_bound': bool(self.contract),
                'allowed_actions': list(self._manifest.get('allowed_actions', [])) if self._manifest else [],
                'expires_at_ms': self._manifest.get('expires_at_ms') if self._manifest else None,
                'authority_observation': self._last_authority_reason,
                'canvas_configurable': False, 'command_worker_started_by_binding': False}

    def close(self):
        self._closed = True
        if callable(self._adapter_close):
            self._adapter_close()


def load_runtime_binding(config, *, environ=None, adapter_loader=None, wall_time=time.time,
                         clock=time.monotonic, owner_uid=0):
    """Resolve an explicit deployment; unavailable integrations stay read-only.

    An invalid/missing authorization leaves all sensor cards operational. A
    trusted adapter error is reduced to a bounded diagnostic, never a traceback
    containing machine private paths or credentials.
    """
    env = os.environ if environ is None else environ
    path = env.get('S10_EXECUTION_AUTHORIZATION_FILE')
    if not path:
        return RuntimeBinding(wall_time=wall_time, clock=clock, owner_uid=owner_uid)
    manifest = None
    binding = None
    adapter = None
    try:
        manifest, digest = read_authorization(path, wall_time=wall_time, owner_uid=owner_uid)
        if not manifest['execution_authorized']:
            return RuntimeBinding(reason='authorization_disabled', path=path, manifest=manifest,
                                  wall_time=wall_time, clock=clock, owner_uid=owner_uid)
        adapter = (adapter_loader or _load_adapter)(manifest['adapter'], config)
        if not isinstance(adapter, dict) or set(adapter) - {'contract', 'authority_provider', 'posture_stop_factory', 'close'}:
            raise BindingError('adapter_binding_shape_invalid')
        contract = adapter.get('contract')
        if not isinstance(contract, dict) or contract.get('schema') != 's10_pro.control_contract/2':
            raise BindingError('action_contract_v2_required')
        if (contract.get('robot_identity') != manifest['robot_identity']
                or contract.get('firmware_identity') != manifest['firmware_identity']):
            raise BindingError('contract_machine_identity_mismatch')
        if contract_fingerprint(contract) != manifest['contract_sha256']:
            raise BindingError('contract_digest_mismatch')
        if not callable(adapter.get('authority_provider')):
            raise BindingError('authority_adapter_missing')
        try:
            from .motion import validate_contract
        except ImportError:
            from motion import validate_contract
        restricted = copy.deepcopy(contract)
        actions = restricted.get('actions')
        if not isinstance(actions, dict):
            raise BindingError('action_contract_v2_required')
        restricted['actions'] = {key: value for key, value in actions.items()
                                 if key in manifest['allowed_actions']}
        if not validate_contract(restricted):
            raise BindingError('verified_action_contract_missing')
        posture = None
        reference = adapter.get('posture_stop_factory')
        if reference is not None:
            if not isinstance(reference, str) or not _ADAPTER.fullmatch(reference):
                raise BindingError('posture_factory_invalid')
            try:
                from .posture_process import BoundedPostureProcess
            except ImportError:
                from posture_process import BoundedPostureProcess
            budgets = [value['send_timeout_s'] for key, value in restricted['actions'].items()
                       if key in ('stand', 'lie') and validate_contract(restricted, key)]
            if budgets:
                posture = BoundedPostureProcess(reference, config, send_timeout_s=min(budgets))
        binding = RuntimeBinding(reason='bound', path=path, manifest=manifest,
            manifest_digest=digest, contract=restricted, authority_provider=adapter['authority_provider'],
            posture_stop=posture, wall_time=wall_time, clock=clock, owner_uid=owner_uid)
        binding._adapter_close = adapter.get('close')
        # Re-read the protected manifest after adapter construction. A changed
        # approval cannot be consumed by a slow or delayed adapter startup.
        if not binding.authorized_now():
            raise BindingError('authorization_changed_during_binding')
        return binding
    except BindingError as exc:
        reason = exc.code
    except Exception:
        reason = 'adapter_binding_unavailable'
    if isinstance(adapter, dict) and callable(adapter.get('close')):
        try:
            adapter['close']()
        except Exception:
            pass
    return RuntimeBinding(reason=reason, path=path, manifest=None,
                          wall_time=wall_time, clock=clock, owner_uid=owner_uid)
