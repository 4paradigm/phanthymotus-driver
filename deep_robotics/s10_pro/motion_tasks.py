"""Durable request identities and exactly one local terminal per S10 task.

Restart never resumes a physical action. Unfinished records become errors with
unconfirmed stopping, and their terminal callback remains eligible for enqueue.
"""
from __future__ import annotations

import json
import os
from pathlib import Path
import sqlite3
import threading
import time
import uuid


class MotionTasks:
    def __init__(self, path=':memory:', *, capacity=4096):
        if type(capacity) is not int or capacity < 1:
            raise ValueError('Task history capacity must be a positive integer')
        self._lock = threading.RLock()
        self.capacity = capacity
        if str(path) != ':memory:':
            path = Path(path)
            path.parent.mkdir(parents=True, exist_ok=True, mode=0o700)
            if path.is_symlink() or (path.exists() and not path.is_file()):
                raise ValueError('Task history must be a regular private file')
            try:
                descriptor = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
            except FileExistsError:
                pass
            else:
                os.close(descriptor)
            os.chmod(path, 0o600)
        self._db = sqlite3.connect(str(path), check_same_thread=False)
        self._db.execute('PRAGMA synchronous=FULL')
        self._db.execute('CREATE TABLE IF NOT EXISTS tasks ('
            'request_id TEXT PRIMARY KEY, action_id TEXT UNIQUE NOT NULL, '
            'fingerprint TEXT NOT NULL, request TEXT NOT NULL, '
            'result TEXT, enqueued INTEGER NOT NULL DEFAULT 0)')
        self._db.commit()
        if str(path) != ':memory:':
            os.chmod(path, 0o600)

    @staticmethod
    def fingerprint(request):
        # Transport retry metadata does not change the requested physical target.
        value = {key: request[key] for key in (
            'action', 'duration', 'vx', 'vy', 'yaw_rate', 'direction') if key in request}
        return json.dumps(value, sort_keys=True, allow_nan=False, separators=(',', ':'))

    @staticmethod
    def _record(row):
        if row is None:
            return None
        return {'request_id': row[0], 'action_id': row[1], 'fingerprint': row[2],
                'request': json.loads(row[3]), 'result': json.loads(row[4]) if row[4] else None,
                'notification_enqueued': bool(row[5])}

    def get(self, request_id):
        with self._lock:
            return self._record(self._db.execute('SELECT * FROM tasks WHERE request_id=?', (request_id,)).fetchone())

    def accept(self, request):
        with self._lock:
            previous = self.get(request['request_id'])
            fingerprint = self.fingerprint(request)
            if previous:
                if previous['fingerprint'] != fingerprint:
                    raise ValueError('request_id already belongs to a different motion target')
                return previous, True
            if self._db.execute('SELECT COUNT(*) FROM tasks').fetchone()[0] >= self.capacity:
                raise RuntimeError('Task history capacity reached; no motion was accepted')
            action_id = 's10_motion_' + uuid.uuid4().hex
            encoded = json.dumps(request, sort_keys=True, allow_nan=False)
            with self._db:
                self._db.execute('INSERT INTO tasks (request_id,action_id,fingerprint,request) VALUES (?,?,?,?)',
                    (request['request_id'], action_id, fingerprint, encoded))
            return self.get(request['request_id']), False

    def finish(self, request_id, result):
        if result.get('status') not in ('completed', 'error', 'cancelled'):
            raise ValueError('A terminal task status is required')
        with self._lock, self._db:
            record = self.get(request_id)
            if not record or result.get('action_id') != record['action_id']:
                raise ValueError('Terminal task identity does not match')
            encoded = json.dumps(result, sort_keys=True, allow_nan=False)
            changed = self._db.execute('UPDATE tasks SET result=? WHERE request_id=? AND result IS NULL',
                (encoded, request_id)).rowcount
            return bool(changed)

    def pending_notifications(self):
        with self._lock:
            return [self._record(row) for row in self._db.execute('SELECT * FROM tasks WHERE result IS NOT NULL AND enqueued=0')]

    def mark_enqueued(self, request_id):
        with self._lock, self._db:
            self._db.execute('UPDATE tasks SET enqueued=1 WHERE request_id=? AND result IS NOT NULL', (request_id,))

    def recover_interrupted(self, wall_time=time.time):
        with self._lock:
            interrupted = [self._record(row) for row in self._db.execute('SELECT * FROM tasks WHERE result IS NULL')]
            for record in interrupted:
                self.finish(record['request_id'], {
                    'request_id': record['request_id'], 'action_id': record['action_id'],
                    'action': record['request']['action'], 'status': 'error', 'state': 'error',
                    'accepted': True, 'confirmed': False, 'stop_confirmed': False, 'completed': False,
                    'reason': 'driver_restarted_stop_unconfirmed', 'finished_at': wall_time()})
            return len(interrupted)

    def close(self):
        with self._lock:
            self._db.close()
