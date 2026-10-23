"""Synthetic finite posture/state tests only; no DDS or network I/O."""
import copy
import importlib.util
import sys
import threading
import time
from pathlib import Path

import pytest

from deep_robotics.s10_pro.supervised_motion import SupervisedMotionPlugin, SupervisedPostureBackend
from deep_robotics.s10_pro.motion_tasks import MotionTasks


def wait_for(predicate, timeout=2):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if predicate():
            return
        time.sleep(.005)
    raise AssertionError('synthetic condition timed out')


class Authorization:
    configured = True
    def __init__(self):
        self.valid = True
        self.allowed = ['stand', 'lie']
        self.issued = time.monotonic() - 1
        self.expires = self.issued + 60
    def authorized_now(self):
        return self.valid and time.monotonic() < self.expires
    def public_status(self):
        return {'authorized': self.authorized_now(), 'allowed_actions': self.allowed,
                'vendor_ownership_claimed': False}
    def permit(self, rid, action, deadline):
        if not self.authorized_now() or action not in self.allowed or deadline > self.expires:
            raise PermissionError('not authorized')
        return {'schema': 's10_pro.operator_permit/1', 'session_id': 'fictional',
                'robot_session': 'fictional-boot', 'request_id': rid, 'action': action,
                'issued_monotonic': self.issued, 'expires_monotonic': self.expires,
                'action_deadline_monotonic': deadline}


class Bridge:
    def __init__(self):
        self.state = 0
        self.seq = 1
        self.hes = 0
        self.stale = False
        self.freeze_sequence = False
        self.freeze_receipt = False
        self.received = time.monotonic()
        self.active = {}
        self._lock = threading.Lock()
    def acquire(self, owner, sources, options):
        self.active[owner] = (sources, options)
    def release(self, owner):
        self.active.pop(owner, None)
    def snapshot(self, source):
        with self._lock:
            if not self.freeze_sequence:
                self.seq += 1
            now = time.monotonic()
            if not self.freeze_receipt:
                self.received = now
            result = {'fresh': not self.stale, 'received_monotonic': self.received,
                      'age_s': now - self.received, 'sequence': self.seq, 'error': None}
            if source == 'hard_estop':
                result['data'] = {'value': self.hes}
            else:
                result['data'] = {'data': {'motion_state': {'state': self.state},
                    'gait_state': {'gait': 0}, 'vel_x': 0., 'vel_y': 0., 'vel_yaw': 0.}}
            return result


class Transport:
    def __init__(self, bridge):
        self.bridge = bridge
        self.running = self.closed = False
        self.calls = []
        self.auto_state = True
        self.starts = 0
        self.on_start = self.on_send = None
    def status(self):
        return {'worker_running': self.running, 'closed': self.closed}
    def start(self, **kwargs):
        assert not self.closed
        self.running = True
        self.starts += 1
        if self.on_start:
            self.on_start()
    def publish_state(self, state, *, authority, cancel_event):
        assert self.running and not self.closed and not cancel_event.is_set()
        self.calls.append((state, copy.deepcopy(authority)))
        if self.auto_state:
            self.bridge.state = state
        if self.on_send:
            self.on_send()
        return {'sent': True, 'accepted': None}
    def close(self):
        self.closed, self.running = True, False


class Notifier:
    def __init__(self):
        self.sent = []
    def submit(self, payload):
        self.sent.append(copy.deepcopy(payload))
    def status(self, action_id):
        return {'action_id': action_id, 'delivery_state': 'synthetic'}


@pytest.fixture
def runtime():
    bridge, authorization = Bridge(), Authorization()
    transport = Transport(bridge)
    backend = SupervisedPostureBackend(bridge, transport, authorization)
    notifier = Notifier()
    plugin = SupervisedMotionPlugin({'enabled': True, 'action_timeout_s': 1.},
        'test', backend, notifier=notifier)
    plugin.start()
    yield plugin, backend, bridge, authorization, transport, notifier
    plugin.close()


def begin(runtime, action='stand', rid='synthetic-1'):
    plugin, _, _, _, transport, _ = runtime
    receipt = plugin.execute(action, {'request_id': rid})
    assert receipt['accepted'] is True
    wait_for(lambda: any(permit['request_id'] == rid for _, permit in transport.calls))
    return receipt


def finish(runtime, action='stand', rid='synthetic-1'):
    plugin, _, _, _, _, _ = runtime
    wait_for(lambda: plugin.status()['last_result'] is not None
             and plugin.status()['last_result']['request_id'] == rid
             and plugin.status()['active_request'] is None)
    return plugin.status()['last_result']


def test_read_only_lifecycle_never_starts_worker_or_claims_vendor_ownership(runtime):
    plugin, _, _, _, transport, _ = runtime
    for _ in range(3):
        status = plugin.status()
        assert status['execution_available'] is True
        assert status['vendor_control_owned'] is False
        assert status['contract_verified'] is False
        plugin.stop();plugin.start()
    assert not transport.calls and not transport.running


@pytest.mark.parametrize(('action', 'state'), [('stand', 1), ('lie', 4)])
def test_one_state_automatically_completes_from_fresh_feedback_and_preserves_one_terminal(runtime, action, state):
    plugin, _, _, _, transport, notifier = runtime
    receipt = begin(runtime, action)
    assert [x[0] for x in transport.calls] == [state]
    result = finish(runtime, action)
    assert result['completed'] and result['confirmed'] and not result['stop_confirmed']
    assert result['completion_basis'] == 'post_send_target_state_feedback'
    assert result['completion_level'] == 'reported_motion_state'
    assert result['reported_state_confirmed'] is True
    assert result['reason'] == 'target_state_observed'
    assert result['physical_pose_completion_verified'] is False
    assert result['physical_state_unknown'] is True and result['vendor_control_owned'] is False
    assert 'confirmation_source' not in result
    assert [x[0] for x in transport.calls] == [state]
    wait_for(lambda: len(notifier.sent) == 1)
    replay = plugin.execute(action, {'request_id': 'synthetic-1'})
    assert replay['replayed'] and 'action_id' not in replay
    assert replay['original_action_id'] == receipt['action_id']
    assert len(transport.calls) == 1 and len(notifier.sent) == 1


def test_success_worker_can_serve_next_unique_finite_posture_without_replay(runtime):
    plugin, _, _, _, transport, _ = runtime
    begin(runtime);finish(runtime)
    assert not transport.closed
    begin(runtime, 'lie', 'synthetic-2');finish(runtime, 'lie', 'synthetic-2')
    assert [c[0] for c in transport.calls] == [1, 4]


def test_nonmatching_target_state_never_completes_and_does_not_send_rl_transition(runtime):
    plugin, _, bridge, _, transport, _ = runtime
    transport.auto_state = False
    begin(runtime)
    time.sleep(.08)
    assert plugin.status()['active_request'] is not None and plugin.status()['last_result'] is None
    assert bridge.state == 0 and [call[0] for call in transport.calls] == [1]
    result = finish(runtime)
    assert result['status'] == 'error' and result['physical_state_unknown'] is True
    assert not result['stop_confirmed'] and [call[0] for call in transport.calls] == [1]


def test_pre_send_cached_target_and_repeated_sequence_do_not_complete(runtime):
    plugin, _, bridge, _, transport, _ = runtime
    bridge.state = 1
    bridge.freeze_sequence = True
    begin(runtime)
    time.sleep(.08)
    assert plugin.status()['last_result'] is None
    assert plugin.status()['active_request'] is not None
    # A matching cached state must advance to distinct post-send frames.
    bridge.freeze_sequence = False
    assert finish(runtime)['completed'] is True
    assert [call[0] for call in transport.calls] == [1]


def test_exactly_two_distinct_post_send_target_frames_are_required(runtime):
    plugin, backend, bridge, _, transport, _ = runtime
    transport.auto_state = False
    bridge.freeze_sequence = True
    begin(runtime)
    wait_for(lambda: backend.sent_at('synthetic-1') is not None)
    bridge.state = 1
    with bridge._lock:
        bridge.seq += 1
    wait_for(lambda: backend.target_feedback('stand', 'synthetic-1')['matching_samples'] == 1)
    time.sleep(.05)
    assert plugin.status()['active_request'] is not None
    assert plugin.status()['last_result'] is None
    with bridge._lock:
        bridge.seq += 1
    assert finish(runtime)['reported_state_confirmed'] is True
    assert [call[0] for call in transport.calls] == [1]


def test_matching_target_with_stale_receipt_cannot_complete(runtime):
    plugin, _, bridge, _, transport, _ = runtime
    transport.auto_state = False
    begin(runtime)
    with bridge._lock:
        bridge.state = 1
        bridge.freeze_receipt = True
        bridge.received = time.monotonic() - .6
    result = finish(runtime)
    assert result['status'] == 'error' and result['physical_state_unknown']
    assert not result['completed'] and not result['stop_confirmed']
    assert transport.closed and [call[0] for call in transport.calls] == [1]


@pytest.mark.parametrize('loss', ['cancel', 'timeout', 'hes', 'stale', 'authorization', 'card_stop'])
def test_every_failure_is_error_unknown_never_fake_stopped_and_never_restarts(runtime, loss):
    plugin, _, bridge, authorization, transport, notifier = runtime
    transport.auto_state = False
    begin(runtime)
    if loss == 'cancel':plugin.cancel()
    elif loss == 'hes':bridge.hes = 1
    elif loss == 'stale':bridge.stale = True
    elif loss == 'authorization':authorization.valid = False
    elif loss == 'card_stop':plugin.stop()
    wait_for(lambda: plugin.status()['active_request'] is None)
    result = plugin.status()['last_result']
    assert result['status'] == 'error' and not result['completed'] and not result['stop_confirmed']
    assert result['physical_state_unknown'] and result['operator_intervention_required']
    assert transport.closed and [c[0] for c in transport.calls] == [1]
    wait_for(lambda: len(notifier.sent) == 1)
    authorization.valid = True;bridge.stale = False;bridge.hes = 0
    plugin.start()
    assert plugin.execute('lie', {'request_id':'after-error'})['accepted'] is False
    assert len(transport.calls) == 1


@pytest.mark.parametrize('condition', ['hes', 'stale', 'authorization', 'scope'])
def test_unsafe_before_acceptance_creates_no_task_or_worker(runtime, condition):
    plugin, _, bridge, authorization, transport, notifier = runtime
    if condition == 'hes':bridge.hes = 1
    elif condition == 'stale':bridge.stale = True
    elif condition == 'authorization':authorization.valid = False
    else:authorization.allowed = ['lie']
    assert not plugin.execute('stand', {'request_id':'not-accepted'})['accepted']
    assert plugin._tasks.get('not-accepted') is None
    assert not transport.running and not transport.calls and not notifier.sent


def test_velocity_and_removed_confirmation_api_are_rejected_without_task_or_command(runtime):
    plugin, _, _, _, transport, _ = runtime
    assert not plugin.execute('velocity', {'request_id':'velocity'})['accepted']
    with pytest.raises(ValueError):plugin.dispatch('confirm_pose', {'request_id':'x','observed_action':'stand'})
    with pytest.raises(ValueError):plugin.dispatch('stand', {'request_id':'x','observed_action':'stand'})
    tool = plugin.get_tool()
    assert tool['title'] == 'Posture Requests'
    assert 'reported state observed' in tool['inputSchema']['x-action-params']['stand']['description']
    assert 'not independently verified physical pose' in tool['inputSchema']['x-action-params']['stand']['description']
    assert 'velocity' not in tool['inputSchema']['properties']['action']['enum']
    assert 'confirm_pose' not in tool['inputSchema']['properties']['action']['enum']
    assert 'confirm_pose' not in tool['inputSchema']['x-action-params']
    assert 'observed_action' not in tool['inputSchema']['properties']
    assert 'operator_confirmation' not in plugin.status()
    assert not hasattr(plugin, 'confirm_pose')
    assert tool['inputSchema']['x-completion']['actions'] == ['stand','lie']
    assert not transport.calls
    assert plugin._tasks.get('x') is None and plugin._tasks.get('velocity') is None


def test_preparation_loss_rechecks_before_durable_acceptance(runtime):
    plugin, _, _, authorization, transport, _ = runtime
    transport.on_start = lambda: setattr(authorization,'valid',False)
    assert not plugin.execute('stand', {'request_id':'startup-race'})['accepted']
    assert plugin._tasks.get('startup-race') is None
    assert transport.closed and not transport.calls


def test_restart_error_deduplication_does_not_republish(tmp_path):
    path=tmp_path/'tasks.db'
    tasks=MotionTasks(path)
    request={'action':'stand','request_id':'interrupted','duration':0.,'issued_at_ms':int(time.time()*1000),'ttl_ms':1000}
    record,_=tasks.accept(request);tasks.close()
    tasks=MotionTasks(path);bridge=Bridge();transport=Transport(bridge)
    backend=SupervisedPostureBackend(bridge,transport,Authorization())
    plugin=SupervisedMotionPlugin({'enabled':True},'test',backend,tasks=tasks,notifier=Notifier())
    try:
        plugin.start()
        result=plugin.execute('stand',{'request_id':'interrupted'})
        assert result['status']=='error' and result['original_action_id']==record['action_id']
        assert not transport.calls and not transport.running
    finally:plugin.close();tasks.close()


def test_runtime_build_selects_explicit_supervised_mode_without_starting_a_command_worker(monkeypatch):
    root=Path(__file__).resolve().parents[1]
    monkeypatch.syspath_prepend(str(root/'deep_robotics/s10_pro'))
    spec=importlib.util.spec_from_file_location('supervised_runtime_integration',root/'deep_robotics/s10_pro/main.py')
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    operator=Authorization();sensors=Bridge();motion=Bridge();notifier=Notifier()
    plugins=module.build_runtime_plugins({'motion':{'enabled':True}},'test',sensors,
        lambda *args:None,motion_bridge=motion,operator_authorization=operator,notifier=notifier)
    cards={p.name:p for p in plugins}
    try:
        assert len(cards)==14
        card=cards['motion'];card.start()
        assert card.status()['mode']=='operator_supervised_posture_test'
        assert card.status()['execution_available'] is True
        assert card._settings['action_timeout_s']==15.
        assert card._backend.transport.status()['supervised_posture'] is True
        assert card._backend.transport.status()['worker_running'] is False
        assert 'supervised_posture_observer' in motion.active
        assert not sensors.active
    finally:
        for p in plugins:
            p.stop()
            if hasattr(p,'close'):p.close()


def test_automatic_feedback_schema_is_valid_and_has_no_confirmation_api(runtime):
    jsonschema=pytest.importorskip('jsonschema')
    plugin,*_=runtime
    schema=plugin.get_tool()['inputSchema']
    jsonschema.Draft7Validator.check_schema(schema)
    jsonschema.validate({'action':'stand','request_id':'one'},schema)
    jsonschema.validate({'action':'lie'},schema)
    jsonschema.validate({'action':'prepare_test'},schema)
    for payload in ({'action':'confirm_pose','request_id':'one','observed_action':'stand'},
                    {'action':'stand','observed_action':'stand'},
                    {'action':'confirm_pose','request_id':'one','observed_action':'stand','vx':0},
                    {'action':'velocity','duration':1}):
        with pytest.raises(jsonschema.ValidationError):jsonschema.validate(payload,schema)


def test_explicit_prepare_worker_sends_no_command_and_creates_no_task(runtime):
    plugin,_,_,_,transport,notifier=runtime
    result=plugin.dispatch('prepare_test',{})
    assert result=={'prepared':True,'physical_command_sent':False,'command_worker_running':True}
    assert transport.running and not transport.calls and not notifier.sent
    assert plugin.status()['active_request'] is None
    with pytest.raises(ValueError):plugin.dispatch('prepare_test',{'request_id':'must-not-be-accepted'})


def test_identical_canvas_save_during_posture_does_not_cancel_or_close_worker(runtime):
    plugin,_,bridge,_,transport,_=runtime
    transport.auto_state=False
    begin(runtime)
    generation=plugin._generation
    for updates in ({},dict(plugin._settings),{'config':dict(plugin._settings)}):
        plugin.configure(updates)
        assert plugin._active is not None and not plugin._cancel.is_set()
        assert plugin._generation==generation and not transport.closed
    bridge.state=1
    assert finish(runtime)['completed'] is True
    assert [c[0] for c in transport.calls]==[1]


def test_real_setting_change_still_cancels_the_active_supervised_task(runtime):
    plugin,_,_,_,transport,_=runtime
    transport.auto_state=False
    begin(runtime)
    plugin.configure({'refresh_hz':3.})
    wait_for(lambda: plugin.status()['active_request'] is None)
    result=plugin.status()['last_result']
    assert result['reason']=='configuration_changed' and result['status']=='error'
    assert result['physical_state_unknown'] and transport.closed
    assert [c[0] for c in transport.calls]==[1]


def test_core_reapplying_saved_enabled_before_target_state_preserves_active_request(runtime):
    plugin,_,bridge,_,transport,_=runtime
    transport.auto_state=False
    begin(runtime)
    # Real Core /api/mcp/.../call replays saved config before the action call.
    plugin.dispatch('config',{'enabled':True})
    assert plugin.status()['active_request'] is not None
    bridge.state=1
    wait_for(lambda: plugin.status()['active_request'] is None)
    assert plugin.status()['last_result']['completed']
    assert not transport.closed and [x[0] for x in transport.calls]==[1]


def test_core_reapplying_config_before_lie_cannot_cancel_or_replace_pending_stand(runtime):
    plugin,_,_,_,transport,_=runtime
    transport.auto_state=False
    begin(runtime)
    plugin.dispatch('config',{'enabled':True})
    busy=plugin.dispatch('lie',{'request_id':'second-while-pending'})
    assert busy['state']=='busy'
    assert busy['reason']=='active_posture_task_in_progress'
    assert 'confirm_pose' not in str(busy)
    assert busy['active_request_id']=='synthetic-1' and busy['active_action']=='stand'
    assert not transport.closed and not plugin._cancel.is_set()
    assert plugin._tasks.get('second-while-pending') is None
    assert [x[0] for x in transport.calls]==[1]
