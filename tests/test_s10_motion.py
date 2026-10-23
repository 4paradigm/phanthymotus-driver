"""Motion lifecycle and safe gating, with no sockets, DDS or physical robot."""
import copy
import threading
import time

import pytest

from deep_robotics.s10_pro.motion import MotionPlugin, UnsupportedBackend, validate_contract


class Recorder:
    def __init__(self):
        self.records = {}
    def submit(self, payload):
        self.records[payload['action_id']] = copy.deepcopy(payload)
    def status(self, action_id):
        return {'delivery_state': 'acknowledged' if action_id in self.records else 'pending'}


def settled(plugin, action, arguments):
    result = plugin.dispatch(action, arguments)
    if result.get('state') != 'accepted':
        return result
    deadline = time.monotonic() + 4
    while time.monotonic() < deadline:
        last = plugin.status()['last_result']
        if last and last.get('action_id') == result['action_id'] and result['action_id'] in plugin._notifier.records:
            return last
        time.sleep(.005)
    raise AssertionError('No terminal task result before the test deadline')


def contract():
    # Explicit offline fiction, never deployed as a real robot contract.
    return {'verified': True, 'evidence': {key: 'offline fixture only: ' + key for key in
        ['identity', 'firmware', 'authority', 'feedback', 'stop', 'watchdog']},
        'robot_identity_matched': True, 'firmware_matched': True, 'robot_deadman_verified': True,
        'robot_deadman_s': .3, 'send_timeout_s': .05, 'velocity_units': {'vx': 'm/s', 'vy': 'm/s', 'yaw_rate': 'rad/s'},
        'actions': ['stand', 'lie', 'velocity', 'stopmotion'], 'stop_semantics': 'zero_velocity_until_feedback',
        'stop_on_authority_loss': False}


class Backend:
    def __init__(self):
        self.commands = []
        self.stops = []
        self.owned = True
        self.fresh = True
        self.feedback_fresh = True
        self.accepted = True
        self.stop_at = None
        self.fail = False
        self.entered = threading.Event()
        self.release = None

    def contract(self):
        return contract()

    def control_status(self):
        return {'owned': self.owned, 'fresh': self.fresh, 'feedback_fresh': self.feedback_fresh, 'age_s': .01}

    def send(self, action, request, cancel):
        self.entered.set()
        if self.release is not None:
            self.release.wait(.15)
        if self.fail:
            raise OSError('offline failure')
        if cancel.is_set():
            return {'accepted': False}
        self.commands.append((action, copy.deepcopy(request), time.monotonic()))
        return {'accepted': self.accepted}

    def stop(self, reason, cancel):
        self.stop_at = time.monotonic()
        self.stops.append(reason)
        return {'accepted': None}


def feedback_for(backend, *, posture=True, moving=True, stopping=True, stale=False, cached=False):
    def feedback(action, request_id):
        okay = stopping and backend.stop_at is not None if action == 'stopmotion' else (moving if action == 'velocity' else posture)
        return {'request_id': request_id, 'confirmed': okay, 'fresh': not stale, 'age_s': 8 if stale else .01,
                'received_monotonic': 0 if cached else time.monotonic()}
    return feedback


@pytest.fixture
def plugin():
    backend = Backend()
    plugin = MotionPlugin({'enabled': True, 'max_duration_s': .3, 'action_timeout_s': .2}, 'test', backend,
        feedback=feedback_for(backend), execution_authorized=True, notifier=Recorder())
    plugin.start()
    yield plugin, backend
    plugin.close()


def test_default_gate_never_calls_network_or_makes_unsupported_stop_claim():
    backend = Backend()
    output = []
    p = MotionPlugin({'enabled': True}, 'test', backend, feedback=feedback_for(backend), publish=lambda *a: output.append(a))
    try:
        p.start()
        for action in ['stand', 'lie', 'velocity']:
            result = settled(p, action, {})
            assert result['state'] == 'unavailable'
            assert not result['accepted'] and not result['confirmed'] and not result['completed']
        assert not settled(p, 'stopmotion', {})['completed']
        p.stop()
        assert backend.commands == [] and backend.stops == []
    finally:
        p.close()


def test_default_backend_has_no_send_method_and_verified_boolean_is_insufficient():
    assert not hasattr(UnsupportedBackend(), 'send')
    assert not validate_contract({'verified': True})
    assert validate_contract(contract())


@pytest.mark.parametrize('key', ['identity', 'firmware', 'authority', 'feedback', 'stop', 'watchdog'])
def test_each_missing_evidence_blocks_execution(key):
    value = contract(); value['evidence'].pop(key)
    assert not validate_contract(value)


@pytest.mark.parametrize('key', ['robot_identity_matched', 'firmware_matched', 'robot_deadman_verified'])
def test_actual_robot_matching_and_robot_deadman_are_required(key):
    value = contract(); value[key] = False
    assert not validate_contract(value)


def test_ui_cannot_grant_process_execution_authorization():
    p = MotionPlugin({}, 'test')
    try:
        for key in ['execution_authorized', 'verified', 'robot_deadman_verified', 'backend']:
            with pytest.raises(ValueError):
                p.configure({key: True})
        p.configure({'enabled': True})
        assert not p.start()['execution_available']
    finally:
        p.close()


def test_fixed_actuator_ports_and_lifecycle_do_not_execute(plugin):
    p, b = plugin
    definition = p.get_tool()
    assert definition['type'] == 'actuator' and definition['topic_in'] == []
    assert definition['topic_out'] == p.status()['topic_out']
    assert [v['topic'] for v in definition['topic_out']] == ['/test/s10_pro/motion_status', '/test/s10_pro/motion_events']
    p.stop(); p.start(); p.start()
    assert not b.commands and not b.stops


@pytest.mark.parametrize('action', ['stand', 'lie'])
def test_posture_requires_fresh_observed_feedback(plugin, action):
    p, b = plugin
    result = settled(p, action, {})
    assert result['accepted'] is True and result['confirmed'] and result['completed']
    assert len(b.commands) == 1 and not b.stops


@pytest.mark.parametrize('options', [{'posture': False}, {'cached': True}, {'stale': True}])
def test_ack_alone_stale_or_cached_feedback_cannot_complete(plugin, options):
    p, b = plugin; p._feedback = feedback_for(b, **options)
    result = settled(p, 'stand', {})
    assert result['accepted'] is True and not result['completed'] and not result['confirmed']
    assert b.stops


def test_velocity_is_finite_refreshed_stopped_and_confirmed(plugin):
    p, b = plugin; b.accepted = None
    begin = time.monotonic()
    result = settled(p, 'velocity', {'vx': .1, 'duration': .25})
    assert .2 <= time.monotonic() - begin < 1
    assert result['accepted'] is True and result['command_accepted'] is None  # Application acceptance is separate from the missing velocity ACK.
    assert result['completed'] and result['confirmed'] and result['stop_confirmed']
    assert len(b.commands) >= 2 and b.stops


def test_zero_feedback_only_cannot_confirm_the_requested_velocity(plugin):
    p, b = plugin; p._feedback = feedback_for(b, moving=False)
    result = settled(p, 'velocity', {'duration': .1})
    assert not result['completed'] and not result['confirmed'] and result['stop_confirmed']


@pytest.mark.parametrize('values', [{'duration': 0}, {'duration': 30}, {'vx': .4}, {'vy': -.2}, {'yaw_rate': 1}, {'vx': float('nan')}, {'duration': True}])
def test_duration_and_velocity_limits_are_enforced_before_send(plugin, values):
    p, b = plugin
    with pytest.raises(ValueError):
        settled(p, 'velocity', values)
    assert not b.commands


@pytest.mark.parametrize('delta', [-10000, 10000])
def test_stale_or_future_requests_are_rejected(plugin, delta):
    p, b = plugin
    with pytest.raises(ValueError):
        settled(p, 'stand', {'issued_at_ms': int(time.time() * 1000) + delta})
    assert not b.commands


def test_duplicate_request_reuses_the_finished_task_without_motion(plugin):
    p, b = plugin
    assert settled(p, 'stand', {'request_id': 'one'})['completed']
    result = settled(p, 'stand', {'request_id': 'one'})
    assert result['completed'] and result['replayed'] and 'action_id' not in result and len(b.commands) == 1


@pytest.mark.parametrize('condition', ['owned', 'fresh', 'feedback_fresh'])
def test_invalid_authority_or_feedback_prevents_first_send(plugin, condition):
    p, b = plugin; setattr(b, condition, False)
    result = settled(p, 'velocity', {'duration': .1})
    assert not result['completed'] and not b.commands


def test_cancel_preempts_movement_and_second_request_is_busy(plugin):
    p, b = plugin; p._feedback = feedback_for(b, posture=False)
    output = []
    thread = threading.Thread(target=lambda: output.append(settled(p, 'stand', {})))
    thread.start(); assert b.entered.wait(.5)
    assert settled(p, 'lie', {})['state'] == 'busy'
    settled(p, 'stopmotion', {})
    thread.join(2)
    assert not thread.is_alive() and output[0]['state'] == 'cancelled'
    assert not output[0]['completed'] and b.stops


def test_lost_control_does_not_send_stop_over_manual_takeover(plugin):
    p, b = plugin
    thread = threading.Thread(target=lambda: settled(p, 'velocity', {'duration': .3}))
    thread.start(); assert b.entered.wait(.5)
    b.owned = False
    thread.join(2)
    assert not thread.is_alive() and not b.stops
    count = len(b.commands); time.sleep(.15)
    assert len(b.commands) == count


def test_lifecycle_stop_closes_status_output_and_cancels_active_action(plugin):
    p, b = plugin; output = []
    p._publish = lambda *args: output.append(args)
    thread = threading.Thread(target=lambda: settled(p, 'velocity', {'duration': .3}))
    thread.start(); assert b.entered.wait(.5)
    p.stop(); thread.join(2)
    count = len(output); time.sleep(.6)
    assert len(output) == count and p.status()['state'] == 'idle'
    assert not p.status()['last_result']['completed']


def test_backend_failure_stops_and_does_not_leak_busy_lock(plugin):
    p, b = plugin; b.fail = True
    assert not settled(p, 'stand', {})['completed']
    assert b.stops
    b.fail = False
    assert settled(p, 'lie', {})['completed']


def test_stop_during_parameter_validation_cannot_be_undone_by_starting_request(plugin):
    p, b = plugin
    def stop_during_validation():
        p.stop()
        return time.time()
    p._wall = stop_during_validation
    result = settled(p, 'stand', {})
    assert result['state'] == 'unavailable' and not result['completed']
    assert not b.commands and not b.stops


def test_zero_velocity_is_repeated_until_fresh_stop_feedback(plugin):
    p, b = plugin
    base = feedback_for(b)
    def delayed_stop(action, request_id):
        value = base(action, request_id)
        if action == 'stopmotion':
            value['confirmed'] = len(b.stops) >= 3
        return value
    p._feedback = delayed_stop
    result = settled(p, 'velocity', {'duration': .1})
    assert result['completed'] and len(b.stops) >= 3


def test_verified_flag_cannot_bypass_fresh_runtime_feedback(plugin):
    p, b = plugin
    p._feedback = feedback_for(b, moving=False)
    output = []
    thread = threading.Thread(target=lambda: output.append(settled(p, 'velocity', {'duration': .3})))
    thread.start(); assert b.entered.wait(.5)
    b.feedback_fresh = False
    thread.join(2)
    assert not thread.is_alive() and not output[0]['completed']
    assert b.stops  # Feedback loss is different from relinquishing authority.


def test_expired_request_before_first_send_is_not_completed(plugin):
    p, b = plugin
    p._wall = lambda: time.time() + (10 if p._active else 0)
    result = settled(p, 'velocity', {'duration': .1})
    assert not result['completed'] and not b.commands


def test_safety_configuration_change_during_validation_invalidates_request(plugin):
    p, b = plugin
    underlying = p._busy
    class ChangeBeforeAcquire:
        def acquire(self, **kwargs):
            p.configure({'max_vx': .01})
            return underlying.acquire(**kwargs)
        def release(self):
            underlying.release()
    p._busy = ChangeBeforeAcquire()
    result = settled(p, 'velocity', {'vx': .15, 'duration': .1})
    assert result['state'] == 'cancelled' and not b.commands


def test_closed_controller_cannot_restart_without_a_watchdog(plugin):
    p, b = plugin
    p.close()
    assert p.start()['state'] == 'idle'
    result = settled(p, 'stand', {})
    assert result['state'] == 'unavailable' and not b.commands


def test_future_monotonic_feedback_is_not_fresh_confirmation(plugin):
    p, b = plugin
    base = feedback_for(b)
    def future(action, request_id):
        value = base(action, request_id)
        value['received_monotonic'] += 5
        return value
    p._feedback = future
    assert not settled(p, 'stand', {})['completed']
