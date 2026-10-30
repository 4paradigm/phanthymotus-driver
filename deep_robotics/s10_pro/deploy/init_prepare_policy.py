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


def make_policy(machine_id, *, duration_s=3600, automatic_startup=False, automatic_renewal=False):
    if (len(machine_id) != 32
            or any(c not in '0123456789abcdef' for c in machine_id)):
        raise ValueError('Expected the NOS host machine ID as 32 lowercase hexadecimal characters')
    if type(duration_s) is not int or not 60 <= duration_s <= 3600:
        raise ValueError('Session duration must be an integer from 60 to 3600 seconds')
    if type(automatic_startup) is not bool or type(automatic_renewal) is not bool:
        raise ValueError('Automatic permissions require explicit boolean approvals')
    automatic = automatic_startup or automatic_renewal
    policy = {
        'schema': 's10_pro.prepare_authorization_policy/2' if automatic else 's10_pro.prepare_authorization_policy/1',
        'enabled': True,
        'policy_id': 's10-prepare-' + uuid.uuid4().hex,
        'robot_identity': hashlib.sha256(('s10-prepare-machine:' + machine_id).encode()).hexdigest(),
        # Historical internal scopes retain direction-specific transport limits.
        # Public finite movements use speed/duration, not a 30cm target.
        'allowed_actions': ['stand', 'lie', 'move_mode', 'forward_30cm', 'backward_30cm',
                            'turn_left', 'turn_right', 'relative_forward', 'relative_backward',
                            'relative_left', 'relative_right', 'relative_turn_left',
                            'relative_turn_right'],
        'session_duration_s': duration_s,
        'activation': 'automatic_prepare' if automatic else 'explicit_prepare',
        'provenance': 'owner_approved_automatic_prepare' if automatic else 'owner_enabled_canvas_prepare',
    }
    if automatic:
        policy.update(allow_automatic_startup=automatic_startup,
                      allow_automatic_renewal=automatic_renewal)
    return policy


def provision(directory, machine_id, *, duration_s=3600,
              automatic_startup=False, automatic_renewal=False):
    if os.geteuid() != 0:
        raise PermissionError('Run provisioning as root on the NOS host')
    directory = Path(directory)
    if not directory.is_absolute() or directory == Path('/'):
        raise ValueError('A dedicated absolute deployment directory is required')
    policy = make_policy(machine_id, duration_s=duration_s,
                         automatic_startup=automatic_startup, automatic_renewal=automatic_renewal)
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
    parser.add_argument('--approve-automatic-startup', action='store_true',
                        help='Owner explicitly approves automatic session activation on Driver startup')
    parser.add_argument('--approve-automatic-renewal', action='store_true',
                        help='Owner explicitly approves automatic renewal after a session expires')
    args = parser.parse_args(argv)
    if not args.apply:
        policy_mode = 'automatic_prepare' if (args.approve_automatic_startup or args.approve_automatic_renewal) else 'explicit_prepare'
        print(f'Dry run: no files written. activation={policy_mode}; '
              f'automatic_startup={args.approve_automatic_startup}; automatic_renewal={args.approve_automatic_renewal}. '
              '--apply still requires the three on-site confirmations.')
        return
    if not all([args.operator_present, args.handheld_estop_verified, args.no_competing_commands]):
        parser.error('--apply requires all three on-site confirmations')
    machine_id = Path('/etc/machine-id').read_text(encoding='ascii').strip()
    paths = provision(args.output_dir, machine_id, duration_s=args.session_duration_s,
                      automatic_startup=args.approve_automatic_startup,
                      automatic_renewal=args.approve_automatic_renewal)
    print(json.dumps({'created': [str(p) for p in paths], 'session_active': False,
                      'automatic_startup': args.approve_automatic_startup,
                      'automatic_renewal': args.approve_automatic_renewal,
                      'next_step': ('mount files and start Driver; approved preparation remains health-gated'
                                    if args.approve_automatic_startup else
                                    'mount files; explicit trusted preparation is required before first control'),
                      'physical_command_sent': False}))


if __name__ == '__main__':
    main()
