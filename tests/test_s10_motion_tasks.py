"""Durability and single-terminal invariants, without a robot or transport."""
from concurrent.futures import ThreadPoolExecutor
import os

import pytest

from deep_robotics.s10_pro.motion_tasks import MotionTasks


def request(request_id='request-one'):
    return {'request_id': request_id, 'action': 'stand', 'duration': 1.,
            'issued_at_ms': 1, 'ttl_ms': 1000}


def terminal(record, status):
    return {'action_id': record['action_id'], 'request_id': record['request_id'],
            'action': 'stand', 'status': status, 'finished_at': 123.}


def test_concurrent_finish_keeps_exactly_one_immutable_terminal():
    tasks = MotionTasks()
    try:
        record, _ = tasks.accept(request())
        values = [terminal(record, status) for status in ('completed', 'cancelled', 'error')]
        with ThreadPoolExecutor(max_workers=3) as pool:
            changed = list(pool.map(lambda result: tasks.finish(record['request_id'], result), values))
        assert sum(changed) == 1
        assert tasks.get(record['request_id'])['result'] == values[changed.index(True)]
        assert len(tasks.pending_notifications()) == 1
    finally:
        tasks.close()


def test_identity_terminal_and_enqueue_survive_reopen(tmp_path):
    path = tmp_path / 'state' / 'motion.sqlite3'
    tasks = MotionTasks(path)
    record, _ = tasks.accept(request())
    result = terminal(record, 'error')
    assert tasks.finish(record['request_id'], result)
    tasks.close()
    tasks = MotionTasks(path)
    try:
        replay, replayed = tasks.accept({**request(), 'issued_at_ms': 999, 'ttl_ms': 500})
        assert replayed and replay['action_id'] == record['action_id'] and replay['result'] == result
        assert len(tasks.pending_notifications()) == 1
        tasks.mark_enqueued(record['request_id'])
    finally:
        tasks.close()
    tasks = MotionTasks(path)
    try:
        assert tasks.get(record['request_id'])['notification_enqueued']
        assert tasks.pending_notifications() == []
        assert os.stat(path).st_mode & 0o777 == 0o600
    finally:
        tasks.close()


def test_full_history_rejects_new_identity_and_preserves_replay():
    tasks = MotionTasks(capacity=1)
    try:
        original, _ = tasks.accept(request())
        with pytest.raises(RuntimeError, match='capacity'):
            tasks.accept(request('second'))
        replay, replayed = tasks.accept(request())
        assert replayed and replay['action_id'] == original['action_id']
        with pytest.raises(ValueError, match='different motion target'):
            tasks.accept({**request(), 'action': 'lie'})
        assert tasks.get('second') is None
    finally:
        tasks.close()


def test_task_history_rejects_symlink_and_directory(tmp_path):
    target = tmp_path / 'target'
    target.write_text('unchanged')
    link = tmp_path / 'link'
    link.symlink_to(target)
    for path in (link, tmp_path):
        with pytest.raises(ValueError, match='regular private file'):
            MotionTasks(path)
    assert target.read_text() == 'unchanged'
