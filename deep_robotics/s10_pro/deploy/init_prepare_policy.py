#!/usr/bin/env python3
"""Provision a host-local prepare policy; never contacts or moves the robot."""
from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import stat
import uuid


def make_policy(machine_id, *, duration_s=3600):
    if (len(machine_id) != 32
            or any(c not in '0123456789abcdef' for c in machine_id)):
        raise ValueError('Expected the NOS host machine ID as 32 lowercase hexadecimal characters')
    if type(duration_s) is not int or not 60 <= duration_s <= 3600:
        raise ValueError('Session duration must be an integer from 60 to 3600 seconds')
    return {
        'schema': 's10_pro.prepare_authorization_policy/1',
        'enabled': True,
        'policy_id': 's10-prepare-' + uuid.uuid4().hex,
        'robot_identity': hashlib.sha256(('s10-prepare-machine:' + machine_id).encode()).hexdigest(),
        # Historical internal scopes retain direction-specific transport limits.
        # The public card executes a fixed 2-second pulse, not a 30cm target.
        'allowed_actions': ['stand', 'lie', 'move_mode', 'forward_30cm', 'backward_30cm',
                            'turn_left', 'turn_right', 'relative_forward', 'relative_backward',
                            'relative_left', 'relative_right', 'relative_turn_left',
                            'relative_turn_right'],
        'session_duration_s': duration_s,
        'activation': 'explicit_prepare',
        'provenance': 'owner_enabled_canvas_prepare',
    }


def provision(directory, machine_id, *, duration_s=3600):
    if os.geteuid() != 0:
        raise PermissionError('Run provisioning as root on the NOS host')
    directory = Path(directory)
    if not directory.is_absolute() or directory == Path('/'):
        raise ValueError('A dedicated absolute deployment directory is required')
    policy = make_policy(machine_id, duration_s=duration_s)
    directory.mkdir(parents=True, exist_ok=True, mode=0o700)
    info = directory.lstat()
    if (not stat.S_ISDIR(info.st_mode) or info.st_uid != 0 or info.st_mode & 0o022):
        raise PermissionError('Deployment directory must be root-owned and not group/world writable')
    targets = [directory / 'prepare-machine-id', directory / 'prepare-policy.json']
    if any(p.exists() or p.is_symlink() for p in targets):
        raise FileExistsError('Existing policy or machine identity is preserved; stop and review it before replacement')
    created = []
    try:
        for path, payload in zip(targets, [machine_id + '\n', json.dumps(policy, indent=2) + '\n']):
            fd = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW, 0o600)
            created.append(path)
            with os.fdopen(fd, 'w') as handle:
                handle.write(payload)
                handle.flush()
                os.fsync(handle.fileno())
    except BaseException:
        for path in created:
            path.unlink()
        raise
    return targets


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', default='/opt/phanthy-motus/s10-pro')
    parser.add_argument('--session-duration-s', type=int, default=3600)
    parser.add_argument('--apply', action='store_true')
    parser.add_argument('--operator-present', action='store_true')
    parser.add_argument('--handheld-estop-verified', action='store_true')
    parser.add_argument('--no-competing-commands', action='store_true')
    args = parser.parse_args(argv)
    if not args.apply:
        print('Dry run: no files written. --apply with the three on-site confirmations provisions prepare files.')
        return
    if not all([args.operator_present, args.handheld_estop_verified, args.no_competing_commands]):
        parser.error('--apply requires all three on-site confirmations')
    machine_id = Path('/etc/machine-id').read_text(encoding='ascii').strip()
    paths = provision(args.output_dir, machine_id, duration_s=args.session_duration_s)
    print(json.dumps({'created': [str(p) for p in paths], 'session_active': False,
                      'next_step': 'mount files, start driver, then explicitly select prepare',
                      'physical_command_sent': False}))


if __name__ == '__main__':
    main()
