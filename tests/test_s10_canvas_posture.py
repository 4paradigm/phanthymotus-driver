"""Synthetic canvas preflight/recovery tests. No ROS or robot I/O."""
import copy
import threading
import time

import pytest

from deep_robotics.s10_pro.canvas_posture import CanvasPostureBackend, CanvasPosturePlugin


class Bridge:
    def __init__(self):
        self.ids = 100
        self.frozen = False
        self.status_word = 1
        self.stale = False
        self.faults = []
        self.hes = 0
        self.state = 0
        self.speed = 0.
        self.owners = {}
        self.seq = 0

    def acquire(self, owner, sources, options): self.owners[owner] = set(sources)
    def release(self, owner): self.owners.pop(owner, None)
    def snapshot(self, source):
        self.seq += 1
        result = {'fresh': not self.stale, 'error': None, 'age_s': 0.,
                  'received_monotonic': time.monotonic(), 'sequence': self.seq}
        if source == 'joints':
            if not self.frozen: self.ids += 1
            result['data'] = {'data': {'joints_data': [{'name': [i], 'data_id': self.ids,
                'status_word': self.status_word} for i in range(16)]}}
        elif source == 'fault_aggregator': result['data'] = {'active_faults': self.faults}
        elif source == 'hard_estop': result['data'] = {'value': self.hes}
        else: result['data'] = {'data': {'motion_state': {'state': self.state}, 'gait_state': {'gait': 0},
                                        'vel_x': self.speed, 'vel_y': 0., 'vel_yaw': 0.}}
        return result


class Authorization:
    def __init__(self):
        self.valid = True
        self.allowed = ['stand', 'lie', 'move_mode']
        self.issued = time.monotonic() - 1
    def authorized_now(self): return self.valid
    def public_status(self): return {'authorized': self.valid, 'allowed_actions': self.allowed}
    def permit(self, rid, action, deadline):
        assert self.valid and action in self.allowed
        return {'schema': 's10_pro.operator_permit/1', 'request_id': rid, 'action': action}


class Transport:
    def __init__(self, bridge):
        self.bridge = bridge
        self.running = self.closed = False
        self.calls = []
        self.starts = 0
        self.matches = 2
        self.auto_state = True
    def status(self): return {'worker_running': self.running, 'closed': self.closed}
    def start(self, **kwargs):
        assert not self.closed
        self.running = True
        self.starts += 1
    def graph_snapshot(self):
        return {'state': {'publisher_prepared': self.running, 'matched_subscribers': self.matches}}
    def close(self): self.closed, self.running = True, False
    def publish_state(self, state, *, authority, cancel_event):
        assert self.running and not self.closed and not cancel_event.is_set()
        self.calls.append((state, copy.deepcopy(authority)))
        if self.auto_state: self.bridge.state = 0 if state == 4 else state
        return {'sent': True, 'accepted': None}


class Notifier:
    def submit(self, payload): pass
    def status(self, action_id): return {}


@pytest.fixture
def runtime():
    bridge, auth = Bridge(), Authorization()
    transports = []
    def factory():
        t = Transport(bridge)
        transports.append(t)
        return t
    backend = CanvasPostureBackend(bridge, factory(), auth, transport_factory=factory)
    plugin = CanvasPosturePlugin({'enabled': True, 'action_timeout_s': .5}, 'test', backend,
                                 notifier=Notifier())
    plugin.start()
    yield plugin, backend, bridge, auth, transports
    plugin.close()


def wait(predicate):
    end = time.monotonic() + 2
    while time.monotonic() < end:
        if predicate(): return
        time.sleep(.01)
    assert False, 'test task did not finish'


def test_click_auto_prepares_once_and_sends_one_state_without_manual_prepare(runtime):
    p, _, _, _, ts = runtime
    assert p.dispatch('stand', {'request_id': 'one'})['accepted']
    wait(lambda: p._tasks.get('one')['result'] is not None)
    assert p._tasks.get('one')['result']['completed']
    assert [s for s, _ in ts[0].calls] == [1]
    assert ts[0].starts == 1
    assert p.dispatch('stand', {'request_id': 'one'})['replayed']
    assert len(ts[0].calls) == 1


@pytest.mark.parametrize('condition,reason', [
    ('frozen', 'joint_data_not_advancing'), ('sleep', 'joints_not_enabled'),
    ('stale', 'fresh_joint_feedback_required'), ('fault', 'fresh_clear_fault_snapshot_required'),
    ('expired', 'operator_session_unavailable'), ('hes', 'fresh_clear_hard_estop_required')])
def test_unhealthy_click_does_not_publish_or_create_task(runtime, condition, reason):
    p, _, b, auth, ts = runtime
    if condition == 'frozen': b.frozen = True
    if condition == 'sleep': b.status_word = 3
    if condition == 'stale': b.stale = True
    if condition == 'fault': b.faults = [{'code': 33025}]
    if condition == 'expired': auth.valid = False
    if condition == 'hes': b.hes = 1
    reply = p.dispatch('stand', {'request_id': 'blocked'})
    assert not reply['accepted'] and reason in reply['blocked_by']
    assert not ts[0].calls and p._tasks.get('blocked') is None


def test_next_new_action_safely_recovers_failed_worker_without_old_replay(runtime):
    p, backend, b, _, ts = runtime
    backend.abort()
    b.state = 17
    reply = p.dispatch('lie', {'request_id': 'new-lie'})
    assert reply['accepted'] and reply['channel_auto_recovered']
    assert reply['old_action_retried'] is False
    wait(lambda: p._tasks.get('new-lie')['result'] is not None)
    assert len(ts) == 2 and ts[0].closed and not ts[0].calls
    assert [state for state, _ in ts[1].calls] == [4]


def test_consolidated_prepare_recovers_worker_without_replaying_old_action(runtime):
    p, backend, _, _, ts = runtime
    ts[0].auto_state = False
    assert p.dispatch('stand', {'request_id': 'timeout'})['accepted']
    wait(lambda: p._tasks.get('timeout')['result'] is not None and p._active is None)
    assert not p._tasks.get('timeout')['result']['completed'] and backend._failed
    old = copy.deepcopy(p._tasks.get('timeout'))
    result = p.dispatch('prepare', {})
    assert result['recovered'] and result['physical_command_sent'] is False
    assert result['old_action_retried'] is False
    assert len(ts) == 2 and ts[1].running and not ts[1].calls
    assert p._tasks.get('timeout') == old
    assert p.dispatch('stand', {'request_id': 'timeout'})['replayed']
    assert not ts[1].calls


@pytest.mark.parametrize('state,speed', [(4, 0), (0, .1)])
def test_recovery_refuses_nonstationary_or_transitional_state(runtime, state, speed):
    p, backend, b, _, ts = runtime
    backend.abort()
    b.state, b.speed = state, speed
    result = p.dispatch('prepare', {})
    assert result['accepted'] is False
    assert 'stationary_idle_or_standing_required' in result['blocked_by']
    assert len(ts) == 1


def test_recovery_accepts_stationary_move_mode_without_sending(runtime):
    p, backend, b, _, ts = runtime
    backend.abort()
    b.state = 17
    result = p.dispatch('prepare', {})
    assert result['recovered'] and result['physical_command_sent'] is False
    assert len(ts) == 2 and not ts[1].calls


def test_move_mode_requires_stationary_stand_and_sends_only_state17(runtime):
    p, _, b, _, ts = runtime
    b.state = 1
    result = p.dispatch('move_mode', {'request_id': 'mode-once'})
    assert result['accepted']
    wait(lambda: p._tasks.get('mode-once')['result'] is not None)
    done = p._tasks.get('mode-once')['result']
    assert done['completed'] and done['reported_target_state'] == 17
    assert [state for state, _ in ts[0].calls] == [17]
    assert p.dispatch('move_mode', {'request_id': 'mode-once'})['replayed']
    assert len(ts[0].calls) == 1


def test_move_transition_velocity_after_send_does_not_trigger_authority_loss(runtime):
    p, _, b, _, ts = runtime
    b.state = 1
    original = ts[0].publish_state
    def transition(state, **kwargs):
        result = original(state, **kwargs)
        b.speed = .1
        return result
    ts[0].publish_state = transition
    result = p.dispatch('move_mode', {'request_id': 'mode-transition'})
    assert result['accepted']
    wait(lambda: p._tasks.get('mode-transition')['result'] is not None)
    done = p._tasks.get('mode-transition')['result']
    assert done['completed'] and done['reason'] == 'target_state_observed'
    assert [state for state, _ in ts[0].calls] == [17]


@pytest.mark.parametrize('state,speed,reason', [
    (0, 0., 'stand_first_required'),
    (1, .1, 'fresh_stationary_feedback_required'),
])
def test_move_mode_preconditions_do_not_publish(runtime, state, speed, reason):
    p, _, b, _, ts = runtime
    b.state, b.speed = state, speed
    reply = p.dispatch('move_mode', {'request_id': 'mode-denied'})
    assert reply.get('accepted') is not True
    assert reason in reply['blocked_by']
    assert not ts[0].calls and p._tasks.get('mode-denied') is None


def test_native_rl_mode_rejects_stand_and_requires_lie_without_publish(runtime):
    p, _, b, _, ts = runtime
    assert p.dispatch('prepare', {})['prepared']
    b.state = 17
    reply = p.dispatch('stand', {'request_id': 'invalid-17-to-1'})
    assert reply.get('accepted') is not True
    assert 'exit_move_with_lie_required' in reply['blocked_by']
    assert not ts[0].calls
    actions = p.status()['readiness']['actions']
    assert not actions['stand']['ready'] and actions['lie']['ready']


def test_lie_from_move_waits_for_final_idle_feedback(runtime):
    p, _, b, _, ts = runtime
    b.state = 17
    reply = p.dispatch('lie', {'request_id': 'move-to-idle'})
    assert reply['accepted']
    wait(lambda: p._tasks.get('move-to-idle')['result'] is not None)
    done = p._tasks.get('move-to-idle')['result']
    assert done['completed'] and done['reported_command_state'] == 4
    assert done['reported_target_state'] == 0
    assert [state for state, _ in ts[0].calls] == [4]


def test_transition_state4_rejects_new_stand_until_idle(runtime):
    p, _, b, _, ts = runtime
    b.state = 4
    reply = p.dispatch('stand', {'request_id': 'too-early-stand'})
    assert reply.get('accepted') is not True
    assert 'wait_for_idle_required' in reply['blocked_by']
    assert not ts[0].calls


def test_busy_request_cannot_refresh_or_recover(runtime):
    p, _, _, _, ts = runtime
    ts[0].auto_state = False
    assert p.dispatch('stand', {'request_id': 'pending'})['accepted']
    assert p.dispatch('prepare', {})['state'] == 'busy'
    assert p.dispatch('lie', {'request_id': 'other'})['state'] == 'busy'
    assert len(ts) == 1


def test_prepare_sends_no_commands_when_checks_fail_and_exposes_instructions(runtime):
    p, _, b, _, ts = runtime
    b.status_word = 3
    result = p.dispatch('prepare', {})
    assert not result['accepted'] and not result['physical_command_sent']
    assert 'Wake' in next(s['instruction'] for s in result['next_steps'] if s['code'] == 'joints_not_enabled')
    assert not ts[0].calls


def test_prepare_checks_and_prepares_channel_without_sending(runtime):
    p, _, _, _, ts = runtime
    result = p.dispatch('prepare', {})
    assert result['ready'] and result['checked'] and result['prepared']
    assert result['physical_command_sent'] is False
    assert not ts[0].calls and ts[0].running


def test_schema_exposes_one_consolidated_maintenance_action(runtime):
    import jsonschema
    schema = runtime[0].get_tool()['inputSchema']
    jsonschema.Draft7Validator.check_schema(schema)
    actions = schema['properties']['action']['enum']
    assert 'prepare' in actions
    assert 'prepare_test' not in actions
    assert 'check_test' not in actions
    assert 'recover_test' not in actions
    assert 'stopmotion' not in actions
    assert 'stopmotion' not in runtime[0].status()['supported_actions']
    jsonschema.validate({'action': 'prepare'}, schema)
    with pytest.raises(jsonschema.ValidationError):
        jsonschema.validate({'action': 'prepare', 'vx': 1}, schema)
    assert 'move_mode' in schema['properties']['action']['enum']
    jsonschema.validate({'action': 'move_mode', 'request_id': 'mode'}, schema)
    with pytest.raises(jsonschema.ValidationError):
        jsonschema.validate({'action': 'move_mode', 'vx': .1}, schema)
