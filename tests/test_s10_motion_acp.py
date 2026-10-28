"""Public asynchronous S10 action contract using an entirely fictional robot.

No ROS, sockets, physical robot or Core modifications are used. Acceptance and
completion are independently observed through dispatch/status/notification APIs.
"""
from __future__ import annotations

import copy
import threading
import time
from contextlib import contextmanager

import pytest

from deep_robotics.s10_pro.motion import MotionPlugin
from deep_robotics.s10_pro.motion_tasks import MotionTasks


def fictional_contract():
    return {'verified': True, 'evidence': {key: 'ISOLATED TEST FICTION: ' + key for key in
        ('identity', 'firmware', 'authority', 'feedback', 'stop', 'watchdog')},
        'robot_identity_matched': True, 'firmware_matched': True, 'robot_deadman_verified': True,
        'robot_deadman_s': .3, 'send_timeout_s': .05,
        'velocity_units': {'vx': 'm/s', 'vy': 'm/s', 'yaw_rate': 'rad/s'},
        'actions': ['stand', 'lie', 'velocity', 'stopmotion'],
        'stop_semantics': 'zero_velocity_until_feedback', 'stop_on_authority_loss': False}


class Backend:
    def __init__(self):
        self.commands, self.stops = [], []
        self.owned, self.fresh, self.feedback_fresh = True, True, True
        self.rejected, self.fail = False, False
        self.send_entered = threading.Event()
        self.send_release = None

    def contract(self):
        return fictional_contract()

    def control_status(self):
        return {'owned': self.owned, 'fresh': self.fresh, 'feedback_fresh': self.feedback_fresh, 'age_s': .005}

    def send(self, action, request, cancel):
        self.send_entered.set()
        if self.send_release is not None:
            assert self.send_release.wait(1), 'isolated test did not release the fake transport'
        if self.fail:
            raise OSError('isolated transport failure')
        if cancel.is_set():
            return {'accepted': False}
        self.commands.append((action, copy.deepcopy(request)))
        return {'accepted': False if self.rejected else None}

    def stop(self, reason, cancel):
        self.stops.append(reason)
        return {'accepted': None, 'confirmed': False}


class Feedback:
    def __init__(self, backend):
        self.backend = backend
        self.posture, self.moving, self.stopping = threading.Event(), threading.Event(), threading.Event()
        self.stale = False
        self.on_posture = None

    def __call__(self, action, request_id):
        if action == 'stopmotion':
            confirmed = bool(self.backend.stops) and self.stopping.is_set()
        elif action == 'velocity':
            confirmed = self.moving.is_set()
        else:
            confirmed = self.posture.is_set()
            if confirmed and self.on_posture is not None:
                self.on_posture()
        age = .6 if self.stale else .005
        return {'request_id': request_id, 'confirmed': confirmed, 'fresh': not self.stale,
                'age_s': age, 'received_monotonic': time.monotonic() - age}


class Recorder:
    def __init__(self, *, fail_count=0):
        self._lock = threading.Lock()
        self.fail_count, self.calls, self.delivered = fail_count, [], []
        self.entered = threading.Event()
        self.release = None

    def submit(self, payload):
        with self._lock:
            self.calls.append(copy.deepcopy(payload))
            fails = self.fail_count > 0
            if fails:
                self.fail_count -= 1
        self.entered.set()
        if self.release is not None:
            assert self.release.wait(1), 'isolated test did not release notification enqueue'
        if fails:
            raise OSError('isolated durable enqueue failure')
        with self._lock:
            self.delivered.append(copy.deepcopy(payload))

    def status(self, action_id):
        with self._lock:
            acknowledged = any(item['action_id'] == action_id for item in self.delivered)
        return {'delivery_state': 'acknowledged' if acknowledged else 'pending',
                'acknowledged': acknowledged, 'action_id': action_id}


@contextmanager
def setup_plugin(*, authorized=True, notifier=None, tasks=None, monitor=True, timeout=.3):
    backend = Backend()
    feedback = Feedback(backend)
    notifier = notifier if notifier is not None else Recorder()
    tasks = tasks if tasks is not None else MotionTasks()
    plugin = MotionPlugin({'enabled': True, 'action_timeout_s': timeout, 'max_duration_s': .3},
        'isolated', backend, feedback=feedback, notifier=notifier, tasks=tasks,
        execution_authorized=authorized, autostart_worker=monitor)
    plugin.start()
    try:
        yield plugin, backend, feedback, notifier, tasks
    finally:
        feedback.stopping.set()
        if backend.send_release is not None:
            backend.send_release.set()
        if notifier.release is not None:
            notifier.release.set()
        plugin.close()
        tasks.close()


def wait_for(predicate, timeout=2):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        value = predicate()
        if value:
            return value
        time.sleep(.005)
    raise AssertionError('isolated asynchronous result did not arrive before deadline')


def terminal(plugin, request_id, timeout=2):
    def read():
        result = plugin.status()['last_result']
        return result if result and result['request_id'] == request_id else None
    return wait_for(read, timeout)


def assert_one_terminal(plugin, recorder, action_id):
    wait_for(lambda: any(item['action_id'] == action_id for item in recorder.delivered))
    events = [item for item in plugin.status()['events'] if item['event'] == 'finished' and item['action_id'] == action_id]
    calls = [item for item in recorder.delivered if item['action_id'] == action_id]
    assert len(events) == len(calls) == 1
    assert calls[0]['status'] == events[0]['status']


def test_completion_metadata_only_registers_three_asynchronous_actions():
    with setup_plugin() as (plugin, _, _, _, _):
        schema = plugin.get_tool()['inputSchema']
        assert schema['x-completion']['actions'] == ['stand', 'lie', 'velocity']
        assert schema['x-completion']['silent_completion'] is False
        assert 'stopmotion' not in schema['x-completion']['actions']
        assert schema['x-resource'] == 'base'


@pytest.mark.parametrize('action', ['stand', 'lie'])
def test_acceptance_is_immediate_and_waits_for_fresh_actual_posture_feedback(action):
    with setup_plugin(timeout=.5) as (plugin, backend, feedback, recorder, tasks):
        started = time.monotonic()
        accepted = plugin.dispatch(action, {'request_id': action + '-one'})
        assert time.monotonic() - started < .15
        assert accepted['accepted'] and not accepted['completed'] and not accepted['confirmed']
        assert accepted['status'] == 'running' and accepted['action_id'].startswith('s10_motion_')
        wait_for(lambda: backend.commands)
        time.sleep(.03)
        assert plugin.status()['last_result'] is None and recorder.delivered == []
        assert tasks.get(action + '-one')['result'] is None
        feedback.posture.set()
        result = terminal(plugin, action + '-one')
        assert result['action_id'] == accepted['action_id']
        assert result['status'] == 'completed' and result['confirmed'] and result['completed']
        assert backend.stops == []
        assert_one_terminal(plugin, recorder, accepted['action_id'])


def test_velocity_must_reach_duration_and_confirm_stop_before_completion():
    with setup_plugin() as (plugin, backend, feedback, recorder, _):
        feedback.moving.set()
        started = time.monotonic()
        accepted = plugin.dispatch('velocity', {'request_id': 'velocity-one', 'vx': .1, 'duration': .15})
        assert time.monotonic() - started < .1
        wait_for(lambda: backend.commands)
        time.sleep(.04)
        assert plugin.status()['last_result'] is None
        wait_for(lambda: backend.stops)
        assert time.monotonic() - started >= .14
        assert plugin.status()['last_result'] is None and recorder.delivered == []
        feedback.stopping.set()
        result = terminal(plugin, 'velocity-one')
        assert result['status'] == 'completed' and result['completed'] and result['stop_confirmed']
        assert_one_terminal(plugin, recorder, accepted['action_id'])


@pytest.mark.parametrize('action', ['stand', 'velocity'])
def test_stale_feedback_cannot_complete_an_accepted_action(action):
    with setup_plugin(timeout=.2) as (plugin, _, feedback, recorder, _):
        feedback.stale = True
        feedback.posture.set()
        feedback.moving.set()
        feedback.stopping.set()
        args = {'request_id': 'stale-' + action}
        if action == 'velocity':
            args['duration'] = .1
        accepted = plugin.dispatch(action, args)
        result = terminal(plugin, args['request_id'])
        assert result['status'] == 'error' and not result['completed']
        assert result['confirmed'] is False and result['stop_confirmed'] is False
        assert_one_terminal(plugin, recorder, accepted['action_id'])


@pytest.mark.parametrize('failure', ['rejected', 'exception'])
def test_failed_send_reports_error_even_when_stop_is_confirmed(failure):
    with setup_plugin() as (plugin, backend, feedback, recorder, _):
        backend.rejected = failure == 'rejected'
        backend.fail = failure == 'exception'
        feedback.stopping.set()
        accepted = plugin.dispatch('stand', {'request_id': failure})
        result = terminal(plugin, failure)
        assert result['status'] == 'error' and not result['completed'] and result['stop_confirmed']
        assert_one_terminal(plugin, recorder, accepted['action_id'])


@pytest.mark.parametrize('stop_confirmed,expected', [(True, 'cancelled'), (False, 'error')])
def test_stopmotion_cancels_original_without_registering_a_second_task(stop_confirmed, expected):
    with setup_plugin(timeout=.5) as (plugin, backend, feedback, recorder, tasks):
        accepted = plugin.dispatch('stand', {'request_id': 'cancel-one'})
        wait_for(lambda: backend.commands)
        if stop_confirmed:
            feedback.stopping.set()
        started = time.monotonic()
        stopped = plugin.dispatch('stopmotion', {'request_id': 'stop-request-only'})
        assert time.monotonic() - started < .1
        assert stopped['accepted'] and not stopped['completed']
        assert stopped['original_action_id'] == accepted['action_id'] and 'action_id' not in stopped
        assert stopped['request_id'] == 'cancel-one'
        assert tasks.get('stop-request-only') is None
        result = terminal(plugin, 'cancel-one')
        assert result['status'] == expected and result['stop_confirmed'] is stop_confirmed
        assert not result['completed'] and not result['confirmed']
        assert_one_terminal(plugin, recorder, accepted['action_id'])


def test_stopmotion_is_immediate_while_a_transport_operation_is_in_flight():
    with setup_plugin(timeout=.5) as (plugin, backend, feedback, recorder, _):
        backend.send_release = threading.Event()
        feedback.stopping.set()
        accepted = plugin.dispatch('stand', {'request_id': 'blocked-send'})
        assert backend.send_entered.wait(.2)
        started = time.monotonic()
        result = plugin.dispatch('stopmotion', {})
        assert time.monotonic() - started < .1
        assert result['original_action_id'] == accepted['action_id']
        backend.send_release.set()
        assert terminal(plugin, 'blocked-send')['status'] == 'cancelled'
        assert backend.commands == []
        assert_one_terminal(plugin, recorder, accepted['action_id'])


def test_no_active_stop_creates_no_task_or_action_identity():
    with setup_plugin() as (plugin, backend, _, recorder, tasks):
        result = plugin.dispatch('stopmotion', {'request_id': 'unused'})
        assert not result['accepted'] and not result['completed']
        assert 'action_id' not in result and 'original_action_id' not in result
        assert tasks.get('unused') is None and recorder.calls == [] and backend.stops == []


def test_inflight_retry_and_terminal_replay_keep_one_task_and_never_resend():
    with setup_plugin(timeout=.5) as (plugin, backend, feedback, recorder, tasks):
        accepted = plugin.dispatch('stand', {'request_id': 'retry-one'})
        wait_for(lambda: backend.commands)
        retry = plugin.dispatch('stand', {'request_id': 'retry-one', 'issued_at_ms': 1, 'ttl_ms': 100})
        assert retry['action_id'] == accepted['action_id'] and retry['replayed']
        assert len(backend.commands) == 1
        with pytest.raises(ValueError, match='different motion target'):
            plugin.dispatch('lie', {'request_id': 'retry-one'})
        feedback.posture.set()
        terminal(plugin, 'retry-one')
        replay = plugin.dispatch('stand', {'request_id': 'retry-one', 'issued_at_ms': 1})
        assert 'action_id' not in replay and replay['original_action_id'] == accepted['action_id']
        assert replay['terminal'] and replay['replayed'] and replay['completed']
        for _ in range(4):
            plugin.dispatch('status', {})
            plugin.dispatch('info', {})
            plugin.dispatch('stand', {'request_id': 'retry-one'})
        assert len(backend.commands) == 1
        assert tasks.get('retry-one')['action_id'] == accepted['action_id']
        assert_one_terminal(plugin, recorder, accepted['action_id'])


def test_busy_locked_and_illegal_requests_are_synchronous_rejections_without_tasks():
    with setup_plugin(authorized=False) as (plugin, backend, _, recorder, tasks):
        result = plugin.dispatch('stand', {'request_id': 'locked'})
        assert not result['accepted'] and 'action_id' not in result
        assert tasks.get('locked') is None and backend.commands == [] and recorder.calls == []
    with setup_plugin(timeout=.5) as (plugin, backend, feedback, recorder, tasks):
        for args in [{'request_id': 'illegal', 'vx': True}, {'request_id': 'illegal', 'duration': 9},
                     {'request_id': 'illegal', 'issued_at_ms': 10.5}, {'request_id': 'illegal', 'unknown': 1}]:
            with pytest.raises(ValueError):
                plugin.dispatch('velocity', args)
        assert tasks.get('illegal') is None and backend.commands == [] and recorder.calls == []
        plugin.dispatch('stand', {'request_id': 'busy-one'})
        wait_for(lambda: backend.commands)
        result = plugin.dispatch('lie', {'request_id': 'busy-two'})
        assert result['state'] == 'busy' and not result['accepted'] and 'action_id' not in result
        assert tasks.get('busy-two') is None
        feedback.posture.set()
        terminal(plugin, 'busy-one')


def test_fast_completion_still_has_one_real_identity_and_one_terminal_notification():
    with setup_plugin() as (plugin, backend, feedback, recorder, _):
        feedback.posture.set()
        accepted = plugin.dispatch('lie', {'request_id': 'fast'})
        result = terminal(plugin, 'fast')
        assert result['action_id'] == accepted['action_id'] and result['completed']
        assert len(backend.commands) == 1
        assert_one_terminal(plugin, recorder, accepted['action_id'])


def test_completion_cancellation_race_has_one_terminal_and_never_false_success():
    with setup_plugin(timeout=.5) as (plugin, backend, feedback, recorder, _):
        feedback.stopping.set()
        entered, release = threading.Event(), threading.Event()
        def hold_fresh_confirmation():
            entered.set()
            assert release.wait(.5)
        feedback.on_posture = hold_fresh_confirmation
        accepted = plugin.dispatch('stand', {'request_id': 'race'})
        wait_for(lambda: backend.commands)
        feedback.posture.set()
        assert entered.wait(.3)
        stopped = plugin.dispatch('stopmotion', {})
        release.set()
        assert stopped['accepted'] and stopped['original_action_id'] == accepted['action_id']
        result = terminal(plugin, 'race')
        assert result['status'] == 'cancelled' and not result['completed'] and result['stop_confirmed']
        assert_one_terminal(plugin, recorder, accepted['action_id'])


def test_terminal_enqueue_failure_retains_identical_payload_for_retry():
    recorder = Recorder(fail_count=1)
    with setup_plugin(notifier=recorder) as (plugin, backend, feedback, _, tasks):
        feedback.posture.set()
        accepted = plugin.dispatch('stand', {'request_id': 'enqueue-retry'})
        terminal(plugin, 'enqueue-retry')
        wait_for(lambda: recorder.delivered)
        assert len(recorder.calls) == 2
        assert recorder.calls[0] == recorder.calls[1]
        assert recorder.calls[0]['action_id'] == accepted['action_id']
        assert tasks.get('enqueue-retry')['notification_enqueued']
        assert len(backend.commands) == 1
        assert_one_terminal(plugin, recorder, accepted['action_id'])


def test_concurrent_status_and_monitor_do_not_enqueue_terminal_more_than_once():
    recorder = Recorder()
    recorder.release = threading.Event()
    with setup_plugin(notifier=recorder) as (plugin, _, feedback, _, _):
        # Hold enqueue after the terminal is durable, to force the monitor and
        # action worker to contend for the same pending-notification record.
        accepted = plugin.dispatch('stand', {'request_id': 'enqueue-race'})
        feedback.posture.set()
        assert recorder.entered.wait(.3)
        for _ in range(5):
            plugin.status()
            time.sleep(.02)
        recorder.release.set()
        terminal(plugin, 'enqueue-race')
        wait_for(lambda: recorder.delivered)
        assert len(recorder.calls) == 1
        assert_one_terminal(plugin, recorder, accepted['action_id'])


def test_restart_marks_interrupted_task_error_without_replaying_motion(tmp_path):
    path = tmp_path / 'tasks.sqlite3'
    tasks = MotionTasks(path)
    original, _ = tasks.accept({'request_id': 'interrupted', 'action': 'velocity', 'duration': .2,
                               'vx': .1, 'vy': 0., 'yaw_rate': 0., 'issued_at_ms': 1, 'ttl_ms': 1000})
    tasks.close()
    tasks = MotionTasks(path)
    with setup_plugin(tasks=tasks) as (plugin, backend, _, recorder, _):
        wait_for(lambda: recorder.delivered)
        recovered = tasks.get('interrupted')['result']
        assert recovered['action_id'] == original['action_id']
        assert recovered['status'] == 'error' and recovered['stop_confirmed'] is False
        assert not recovered['completed'] and not recovered['confirmed']
        assert recovered['reason'] == 'driver_restarted_stop_unconfirmed'
        assert backend.commands == [] and backend.stops == []
        replay = plugin.dispatch('velocity', {'request_id': 'interrupted', 'duration': .2, 'vx': .1,
                                            'issued_at_ms': 1})
        assert replay['terminal'] and replay['status'] == 'error'
        assert 'action_id' not in replay and replay['original_action_id'] == original['action_id']
        assert len(recorder.delivered) == 1
