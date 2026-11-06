"""ACP completion tests: fake feedback and HTTP, no robot or ROS needed."""
import json
import threading
import time

import pytest

from dobot.cra.completion import MotionCompletion
from dobot.cra.device import DobotSensors
from test_dobot_joint_move import ready, set_mode, TARGET


def completion_fixture():
    feedback, dashboard, control, calls = ready()
    events = []
    completion = MotionCompletion(control, {'joint_move': {'motion_timeout_s': 0.08}}, notify=lambda *event: events.append(event))
    completion.poll_interval = 0.001
    return feedback, dashboard, control, completion, events


def wait_result(completion):
    deadline = time.monotonic() + 1
    while (completion.last_completion is None or completion.last_completion["callback"] == "sending") and time.monotonic() < deadline:
        time.sleep(0.001)
    assert completion.last_completion is not None
    return completion.last_completion


def test_prefix_and_completion_contract():
    plugin = DobotSensors({})
    assert plugin.PREFIX == 'dobot_cra'
    schema = next(t for t in plugin.get_tools() if t['name'] == 'joint_move')['inputSchema']
    assert schema['x-completion']['actions'] == ['move_j', 'rel_joint_mov_j', 'resume', 'pause', 'stop_motion']
    assert schema['x-completion']['timeout'] > 300


def test_missing_ca_prevents_motion(monkeypatch):
    monkeypatch.delenv('AGENT_CORE_CA_CERT', raising=False)
    feedback, dashboard, control, calls = ready()
    completion = MotionCompletion(control, {})
    with pytest.raises(RuntimeError, match='AGENT_CORE_CA_CERT'):
        completion.start('move_j', {**TARGET, 'confirm_move': True})
    assert not calls and not dashboard.commands


@pytest.mark.parametrize('failure', ['stale', 'protection', 'timeout', 'wrong_target'])
def test_monitor_never_reports_false_completion(failure):
    feedback, dashboard, control, completion, events = completion_fixture()
    state = feedback.sample['data']
    state['current_command_id'] = 3
    feedback.sample['packets_received'] = 2
    if failure == 'stale':
        feedback.sample['stale'] = True
    elif failure == 'protection':
        state['collision_state'] = 1
    elif failure == 'timeout':
        feedback.sample['packets_received'] = 1
    with pytest.raises((RuntimeError, TimeoutError)):
        completion._monitor(threading.Event(), list(TARGET.values()), 3, 1)


def test_cached_target_idle_does_not_complete_without_new_feedback():
    feedback, _, _, completion, _ = completion_fixture()
    feedback.sample['data']['joint_state']['q_actual'] = list(TARGET.values())
    feedback.sample['data']['current_command_id'] = 3
    feedback.sample['packets_received'] = 1
    with pytest.raises(TimeoutError):
        completion._monitor(threading.Event(), list(TARGET.values()), 3, 1)


def test_completion_needs_correlated_queue_or_observed_running():
    feedback, _, _, completion, _ = completion_fixture()
    feedback.sample['data']['joint_state']['q_actual'] = list(TARGET.values())
    feedback.sample['packets_received'] = 2
    with pytest.raises(TimeoutError):
        completion._monitor(threading.Event(), list(TARGET.values()), 999, 1)
    feedback.sample['data']['current_command_id'] = 999
    assert completion._monitor(threading.Event(), list(TARGET.values()), 999, 1)['reason'] == 'target_reached'


def test_async_preparation_failure_is_correlated_to_action_id():
    feedback, dashboard, _, completion, events = completion_fixture()
    feedback.sample['stale'] = True
    result = completion.start('move_j', {**TARGET, 'confirm_move': True})
    record = wait_result(completion)
    assert record['status'] == 'error' and record['action_id'] == result['action_id']
    assert events[0][0] == result['action_id'] and not dashboard.commands


def test_active_motion_rejects_overlap_and_can_be_cancelled():
    _, _, control, completion, events = completion_fixture()
    entered = threading.Event()
    def execute(action, args, *, cancel=None):
        entered.set()
        assert cancel.wait(1)
        raise RuntimeError('interrupted during preparation')
    control.execute_motion = execute
    result = completion.start('move_j', {**TARGET, 'confirm_move': True})
    assert entered.wait(1)
    with pytest.raises(RuntimeError, match='active'):
        completion.start('move_j', {**TARGET, 'confirm_move': True})
    completion.interrupt()
    completion.shutdown()
    assert wait_result(completion)['status'] == 'cancelled'
    assert events[0][0] == result['action_id']


def test_completion_posts_verified_https_with_matching_action_id(monkeypatch):
    _, _, _, completion, _ = completion_fixture()
    completion._notify = None
    requests = []
    class Response:
        def __enter__(self): return self
        def __exit__(self, *args): pass
        def read(self): return b'{"ok":true,"action_id":"test-id"}'
    def open_request(request, **kwargs):
        requests.append((request, kwargs))
        return Response()
    monkeypatch.setenv('AGENT_CORE_URL', 'https://core:15678')
    monkeypatch.setattr('dobot.cra.completion.urlopen', open_request)
    context = object()
    completion._complete('test-id', 'completed', {'reason': 'target_reached'}, context)
    request, options = requests[0]
    assert request.full_url == 'https://core:15678/api/acp/complete'
    assert options['context'] is context
    assert json.loads(request.data)['tool'] == 'joint_move'
    assert json.loads(request.data)['action_id'] == 'test-id'
    assert completion.last_completion['callback'] == 'accepted'


def test_callback_rejects_unmatched_ack(monkeypatch):
    _, _, _, completion, _ = completion_fixture()
    completion._notify = None
    class Response:
        def __enter__(self): return self
        def __exit__(self, *args): pass
        def read(self): return b'{"ok":true,"action_id":"wrong"}'
    monkeypatch.setattr('dobot.cra.completion.urlopen', lambda *a, **k: Response())
    completion._complete('test-id', 'completed', {}, object())
    assert completion.last_completion['callback'] == 'failed'


@pytest.mark.parametrize('action', ['pause', 'stop_motion'])
def test_pause_and_stop_need_new_stationary_feedback(action):
    feedback, _, _, completion, _ = completion_fixture()
    set_mode(feedback, 10 if action == 'pause' else 5)
    feedback.sample['packets_received'] = 1
    # Old/cached state, even already stopped, cannot complete a new request.
    with pytest.raises(TimeoutError):
        completion._monitor_control(action, threading.Event(), 1)
    original = feedback.snapshot
    count = [1]
    def snapshot():
        count[0] += 1
        return {**original(), 'packets_received': count[0]}
    feedback.snapshot = snapshot
    assert completion._monitor_control(action, threading.Event(), 1)['reason'] == ('paused' if action == 'pause' else 'stopped')


@pytest.mark.parametrize('action', ['pause', 'stop_motion'])
def test_pause_stop_never_complete_while_joints_still_moving(action):
    feedback, _, _, completion, _ = completion_fixture()
    set_mode(feedback, 10 if action == 'pause' else 5)
    feedback.sample['data']['joint_state']['qd_actual'] = [0, 0, 1, 0, 0, 0]
    original = feedback.snapshot
    count = [1]
    def snapshot():
        count[0] += 1
        return {**original(), 'packets_received': count[0]}
    feedback.snapshot = snapshot
    with pytest.raises(TimeoutError):
        completion._monitor_control(action, threading.Event(), 1)


@pytest.mark.parametrize('action,command', [('pause', 'Pause()'), ('stop_motion', 'Stop()')])
def test_pause_and_stop_receive_their_own_completion_ids(action, command):
    feedback, dashboard, _, completion, events = completion_fixture()
    set_mode(feedback, 7)
    original_send, original_snapshot = dashboard.send_receive_text, feedback.snapshot
    count = [0]
    def send(text):
        reply = original_send(text)
        if text == command:
            set_mode(feedback, 10 if action == 'pause' else 5)
        return reply
    def snapshot():
        count[0] += 1
        return {**original_snapshot(), 'packets_received': count[0]}
    feedback.snapshot, dashboard.send_receive_text = snapshot, send
    result = completion.start(action, {'confirm_move': True})
    record = wait_result(completion)
    assert record['action_id'] == result['action_id'] and record['status'] == 'completed'
    assert record['result']['reason'] == ('paused' if action == 'pause' else 'stopped')
    assert dashboard.commands == [command]


def test_stop_is_sent_but_not_claimed_complete_when_feedback_is_lost():
    feedback, dashboard, _, completion, events = completion_fixture()
    feedback.sample.update(connected=False, stale=True, data=None)
    result = completion.start('stop_motion', {'confirm_move': True})
    record = wait_result(completion)
    assert record['action_id'] == result['action_id'] and record['status'] == 'error'
    assert dashboard.commands == ['Stop()']
    assert 'cannot confirm' in record['result']['reason']


def test_stop_preempts_motion_and_both_actions_get_terminal_events():
    feedback, dashboard, control, completion, events = completion_fixture()
    started = threading.Event()
    original_snapshot = feedback.snapshot
    count = [0]
    def snapshot():
        count[0] += 1
        return {**original_snapshot(), 'packets_received': count[0]}
    feedback.snapshot = snapshot
    original = control.execute_motion
    def execute(action, args, *, cancel=None):
        if action == 'move_j':
            started.set()
            assert cancel.wait(1)
            raise RuntimeError('cancelled preparation')
        result = original(action, args, cancel=cancel)
        set_mode(feedback, 5)
        return result
    control.execute_motion = execute
    move = completion.start('move_j', {**TARGET, 'confirm_move': True})
    assert started.wait(1)
    stop = completion.start('stop_motion', {'confirm_move': True})
    deadline = time.monotonic() + 1
    while len(events) < 2 and time.monotonic() < deadline:
        time.sleep(0.001)
    assert {(e[0], e[1]) for e in events} == {(move['action_id'], 'cancelled'), (stop['action_id'], 'completed')}
    assert dashboard.commands == ['Stop()']


def test_stop_remains_available_with_missing_callback_certificate(monkeypatch):
    monkeypatch.delenv('AGENT_CORE_CA_CERT', raising=False)
    feedback, dashboard, control, calls = ready()
    completion = MotionCompletion(control, {'joint_move': {'motion_timeout_s': 0.02}})
    feedback.sample.update(connected=False, stale=True, data=None)
    result = completion.start('stop_motion', {'confirm_move': True})
    record = wait_result(completion)
    assert record['action_id'] == result['action_id']
    assert record['status'] == 'error' and record['callback'] == 'failed'
    assert dashboard.commands == ['Stop()']


def test_resume_completion_observes_running_then_queue_end():
    feedback, _, _, completion, _ = completion_fixture()
    sequence = [7, 5]
    def snapshot():
        mode = sequence.pop(0) if sequence else 5
        set_mode(feedback, mode)
        return {**feedback.sample, 'packets_received': 2}
    feedback.snapshot = snapshot
    assert completion._monitor(threading.Event(), None, None, 1)['reason'] == 'queue_completed'


def test_motion_timeout_reports_error_and_attempts_one_stop():
    feedback, dashboard, _, completion, events = completion_fixture()
    result = completion.start('move_j', {**TARGET, 'confirm_move': True})
    record = wait_result(completion)
    assert record['action_id'] == result['action_id'] and record['status'] == 'error'
    assert 'deadline' in record['result']['reason']
    assert dashboard.commands[-1] == 'Stop()' and dashboard.commands.count('Stop()') == 1


def test_pause_rejected_while_preempting_has_stop_fallback():
    feedback, dashboard, control, completion, events = completion_fixture()
    entered = threading.Event()
    original = control.execute_motion
    def execute(action, args, *, cancel=None):
        if action == 'move_j':
            entered.set()
            assert cancel.wait(1)
            raise RuntimeError('cancelled preparation')
        return original(action, args, cancel=cancel)
    control.execute_motion = execute
    move = completion.start('move_j', {**TARGET, 'confirm_move': True})
    assert entered.wait(1)
    pause = completion.start('pause', {'confirm_move': True})  # idle mode rejects Pause
    deadline = time.monotonic() + 1
    while len(events) < 2 and time.monotonic() < deadline:
        time.sleep(0.001)
    assert {(e[0], e[1]) for e in events} == {(move['action_id'], 'cancelled'), (pause['action_id'], 'error')}
    assert dashboard.commands == ['Stop()']


def test_transient_callback_failure_retries_same_id_without_replaying_motion():
    _, dashboard, _, completion, _ = completion_fixture()
    attempts = []
    def notify(action_id, status, result):
        attempts.append(action_id)
        if len(attempts) == 1:
            raise ConnectionError('temporary callback outage')
    completion._notify = notify
    completion._complete('same-id', 'completed', {}, None)
    assert attempts == ['same-id', 'same-id']
    assert completion.last_completion['callback'] == 'accepted'
    assert not dashboard.commands
