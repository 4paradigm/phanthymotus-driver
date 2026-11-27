"""Operator-authorized, single-use field trial, isolated from verified poses."""
import argparse
from contextlib import closing
import hashlib
import hmac
import json
import math
import os
from pathlib import Path
import re
import secrets
import sqlite3
import stat
import time


TRIAL_ANGLES_HOLD = (4.4, 12.4, 1.0)
PERMIT_LIFETIME_NS = 180_000_000_000


def _boot():
    return Path('/proc/sys/kernel/random/boot_id').read_text().strip()


def _now():
    return time.clock_gettime_ns(time.CLOCK_BOOTTIME)


def _digest(value):
    return hashlib.sha256(value).hexdigest()


def _matches_request(values):
    return (all(type(x) in (int, float) for x in values)
            and tuple(values) == TRIAL_ANGLES_HOLD)


class CommissioningTrial:
    """A pinned site record defines the only target; arguments cannot supply joints."""

    def __init__(self, directory, record_sha256):
        directory = Path(directory)
        if (not directory.is_absolute() or directory.resolve(strict=True) != directory
                or not directory.is_dir() or (os.name == 'posix' and directory.stat().st_mode & 0o077)):
            raise ValueError('directional_trial_private_directory_required')
        self.directory = directory
        self.record_sha256 = record_sha256
        record_path = directory / 'trial.json'
        if record_path.is_symlink() or not record_path.is_file() or record_path.stat().st_size > 8192:
            raise ValueError('directional_trial_record_invalid')
        raw = record_path.read_bytes()
        if not isinstance(record_sha256, str) or _digest(raw) != record_sha256:
            raise ValueError('directional_trial_record_mismatch')
        record = json.loads(raw)
        fields = {'schema', 'trial_id', 'model_sha256', 'start_q_rad', 'target_q_rad',
                  'yaw_deg', 'pitch_deg', 'hold_s'}
        if (not isinstance(record, dict) or set(record) != fields
                or record['schema'] != 'g1.directional-trial.v1'
                or not isinstance(record['trial_id'], str)
                or not re.fullmatch(r'[a-zA-Z0-9_.-]{1,100}', record['trial_id'])
                or not _matches_request([record[k] for k in ('yaw_deg', 'pitch_deg', 'hold_s')])):
            raise ValueError('directional_trial_record_invalid')
        for field in ('start_q_rad', 'target_q_rad'):
            q = record[field]
            if (not isinstance(q, list) or len(q) != 5
                    or any(type(x) not in (int, float) or not math.isfinite(x) for x in q)):
                raise ValueError('directional_trial_record_invalid')
        self.record = record

    @classmethod
    def from_backend(cls, backend):
        config = getattr(backend, 'config', {})
        if 'commissioning_trial' not in config:
            return None
        binding = config['commissioning_trial']
        if (not isinstance(binding, dict) or set(binding) != {'directory', 'record_sha256'}
                or config.get('verified_poses', []) != []):
            raise ValueError('directional_trial_config_invalid')
        trial = cls(binding['directory'], binding['record_sha256'])
        model = backend.kinematics
        if (trial.record['model_sha256'] != model.model_sha256
                or trial.record['start_q_rad'] != config.get('verified_start_q_rad')):
            raise ValueError('directional_trial_binding_mismatch')
        for key, maximum in (('start_tolerance_rad', .05), ('joint_speed_rad_s', .15),
                             ('target_tolerance_rad', .04), ('angular_tolerance_deg', 3.0)):
            value = config.get(key, maximum)
            if type(value) not in (int, float) or not 0 < value <= maximum:
                raise ValueError('directional_trial_guard_limit:' + key)
        model.direction(trial.record['start_q_rad'])
        yaw, pitch = map(math.radians, TRIAL_ANGLES_HOLD[:2])
        target = [math.cos(pitch)*math.cos(yaw), math.cos(pitch)*math.sin(yaw), math.sin(pitch)]
        error = model.evaluate(trial.record['target_q_rad'], target)['angular_error_deg']
        if not 0 <= error <= min(model.angular_tolerance_deg, 0.1):
            raise ValueError('directional_trial_target_mismatch')
        return trial

    def target(self, args):
        if not _matches_request([args.get(k) for k in ('yaw_deg', 'pitch_deg', 'hold_s')]):
            raise ValueError('directional_trial_request_mismatch')
        return list(self.record['target_q_rad'])

    def _database(self, create=False):
        path = self.directory / 'permit.sqlite3'
        if path.is_symlink() or (path.exists() and not stat.S_ISREG(path.stat().st_mode)):
            raise ValueError('directional_trial_permit_invalid')
        db = sqlite3.connect(path.as_uri() + ('?mode=rwc' if create else '?mode=rw'),
                             uri=True, timeout=2)
        db.execute('PRAGMA synchronous=FULL')
        return db

    def arm(self):
        """Robot-side CLI only. The durable receipt permanently consumes this trial."""
        proof = secrets.token_hex(32)
        now, boot = _now(), _boot()
        with closing(self._database(create=True)) as db, db:
            db.execute('BEGIN IMMEDIATE')
            db.execute('CREATE TABLE IF NOT EXISTS permit '
                       '(id INTEGER PRIMARY KEY CHECK (id=1), record TEXT NOT NULL, '
                       'boot TEXT NOT NULL, until_ns INTEGER NOT NULL, proof TEXT NOT NULL, spent TEXT)')
            prior = db.execute('SELECT record, boot, until_ns, spent FROM permit WHERE id=1').fetchone()
            if prior:
                if prior[0] != self.record_sha256 or prior[3] is not None:
                    raise ValueError('directional_trial_spent_or_rebound')
                if prior[1] == boot and now < prior[2]:
                    raise ValueError('directional_trial_already_armed')
            db.execute('INSERT OR REPLACE INTO permit VALUES (1, ?, ?, ?, ?, NULL)',
                       (self.record_sha256, boot, now + PERMIT_LIFETIME_NS, _digest(proof.encode('ascii'))))
        if os.name == 'posix':
            fd = os.open(self.directory, os.O_DIRECTORY | os.O_RDONLY)
            try:
                os.fsync(fd)
            finally:
                os.close(fd)
        return proof

    def authorize(self, proof, *, consume_for=None):
        """Re-check under SQLite's write lock and commit before starting the motor worker."""
        if not isinstance(proof, str) or not re.fullmatch(r'[0-9a-f]{64}', proof):
            raise ValueError('directional_trial_operator_code_required')
        try:
            with closing(self._database()) as db, db:
                if consume_for is not None:
                    db.execute('BEGIN IMMEDIATE')
                row = db.execute('SELECT record, boot, until_ns, proof, spent FROM permit WHERE id=1').fetchone()
                if (not row or row[0] != self.record_sha256 or row[4] is not None
                        or row[1] != _boot() or not 0 < row[2] - _now() <= PERMIT_LIFETIME_NS
                        or not hmac.compare_digest(row[3], _digest(proof.encode('ascii')))):
                    raise ValueError('directional_trial_permit_unavailable')
                if consume_for is not None:
                    db.execute("UPDATE permit SET spent=?, proof='' WHERE id=1", (consume_for,))
        except (sqlite3.Error, OSError) as exc:
            raise ValueError('directional_trial_permit_unavailable') from exc

    def record_terminal(self, action_id, summary):
        """Commit the worker's terminal observation to the same DB as the spent permit."""
        if summary['state'] not in ('completed', 'cancelled', 'error'):
            raise ValueError('directional_trial_result_not_terminal')
        evidence = {key: summary[key] for key in ('state', 'frame', 'actual_direction',
                    'angular_error_deg', 'sample_phase', 'reason', 'external_accuracy')}
        evidence.update(source='driver_worker', action_id=action_id, trial_id=self.record['trial_id'])
        with closing(self._database()) as db, db:
            db.execute('BEGIN IMMEDIATE')
            db.execute('CREATE TABLE IF NOT EXISTS trial_terminal '
                       '(action_id TEXT PRIMARY KEY NOT NULL, evidence TEXT NOT NULL)')
            inserted = db.execute('INSERT INTO trial_terminal (action_id, evidence) '
                                  'SELECT spent, ? FROM permit WHERE id=1 AND spent=? AND record=?',
                                  (json.dumps(evidence, allow_nan=False), action_id, self.record_sha256))
            if inserted.rowcount != 1:
                raise ValueError('directional_trial_result_permit_mismatch')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=['arm'])
    parser.add_argument('directory')
    parser.add_argument('record_sha256')
    args = parser.parse_args()
    try:
        print(CommissioningTrial(args.directory, args.record_sha256).arm())
    except (ValueError, OSError, sqlite3.Error) as exc:
        parser.exit(1, str(exc) + '\n')
