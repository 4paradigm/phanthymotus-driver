"""Synthetic posture/state/operator events only; no DDS or network I/O."""
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
        self.active = {}
        self._lock = threading.Lock()
    def acquire(self, owner, sources, options):
        self.active[owner] = (sources, options)
    def release(self, owner):
        self.active.pop(owner, None)
    def snapshot(self, source):
        with self._lock:
            self.seq += 1
            now = time.monotonic()
            result = {'fresh': not self.stale, 'received_monotonic': now,
                      'age_s': 0., 'sequence': self.seq, 'error': None}
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
        self.on_start = self.on_send = None
    def status(self):
        return {'worker_running': self.running, 'closed': self.closed}
    def start(self, **kwargs):
        assert not self.closed
        self.running = True
        if self.on_start:
            self.on_start()
    def publish_state(self, state, *, authority, cancel_event):
        assert self.running and not self.closed and not cancel_event.is_set()
        self.calls.append((state, copy.deepcopy(authority)))
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
    plugin, _, _, _, _, _ = runtime
    receipt = plugin.execute(action, {'request_id': rid})
    assert receipt['accepted'] is True
    wait_for(lambda: plugin.status()['phase'] == 'awaiting_operator_pose_confirmation')
    return receipt


def finish(runtime, action='stand', rid='synthetic-1'):
    plugin, backend, _, _, _, _ = runtime
    wait_for(lambda: backend.target_feedback(action, rid)['confirmed'])
    reply = plugin.confirm_pose({'request_id': rid, 'observed_action': action})
    assert reply['accepted'] is True
    wait_for(lambda: plugin.status()['active_request'] is None)
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
def test_one_state_only_requires_visual_confirmation_and_preserves_exactly_one_terminal(runtime, action, state):
    plugin, _, _, _, transport, notifier = runtime
    receipt = begin(runtime, action)
    time.sleep(.08)
    assert plugin.status()['active_request'] is not None
    assert notifier.sent == []
    assert [x[0] for x in transport.calls] == [state]
    result = finish(runtime, action)
    assert result['completed'] and result['confirmed'] and not result['stop_confirmed']
    assert result['confirmation_source'] == 'operator_visual_and_feedback'
    assert not result['physical_state_unknown'] and result['vendor_control_owned'] is False
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


@pytest.mark.parametrize('bad', [{'request_id':'other','observed_action':'stand'},
                                {'request_id':'synthetic-1','observed_action':'lie'}])
def test_wrong_request_or_pose_cannot_confirm(runtime, bad):
    plugin, _, _, _, _, _ = runtime
    begin(runtime)
    assert plugin.confirm_pose(bad)['accepted'] is False
    assert plugin.status()['active_request'] is not None


def test_confirmation_before_publish_or_before_target_feedback_is_rejected(runtime):
    plugin, _, bridge, _, _, _ = runtime
    assert not plugin.confirm_pose({'request_id':'synthetic-1','observed_action':'stand'})['accepted']
    begin(runtime)
    bridge.state = 0
    assert not plugin.confirm_pose({'request_id':'synthetic-1','observed_action':'stand'})['accepted']


@pytest.mark.parametrize('loss', ['cancel', 'timeout', 'hes', 'stale', 'authorization', 'card_stop'])
def test_every_failure_is_error_unknown_never_fake_stopped_and_never_restarts(runtime, loss):
    plugin, _, bridge, authorization, transport, notifier = runtime
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


def test_velocity_and_unknown_confirmation_fields_are_rejected(runtime):
    plugin, _, _, _, transport, _ = runtime
    assert not plugin.execute('velocity', {'request_id':'velocity'})['accepted']
    with pytest.raises(ValueError):plugin.confirm_pose({'request_id':'x','observed_action':'stand','owned':True})
    tool = plugin.get_tool()
    assert 'velocity' not in tool['inputSchema']['properties']['action']['enum']
    assert 'confirm_pose' in tool['inputSchema']['properties']['action']['enum']
    assert tool['inputSchema']['x-completion']['actions'] == ['stand','lie']
    assert not transport.calls


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


def test_operator_confirmation_schema_is_valid_and_forbids_telemetry_as_visual(runtime):
    jsonschema=pytest.importorskip('jsonschema')
    plugin,*_=runtime
    schema=plugin.get_tool()['inputSchema']
    jsonschema.Draft7Validator.check_schema(schema)
    jsonschema.validate({'action':'confirm_pose','request_id':'one','observed_action':'stand'},schema)
    for payload in ({'action':'confirm_pose','observed_action':'stand'},
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
