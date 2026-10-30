"""Synthetic end-to-end merged locomotion card checks; no ROS or robot I/O."""
import time
import threading
import jsonschema
import pytest
from deep_robotics.s10_pro.loco_move import LocoMoveBackend, LocoMovePlugin
from common.vendor_runtime import DriverBundle
from test_s10_timed_move import Bridge, Transport, Authorization, Notifier
from test_s10_prepare_authorization import policy as prepare_policy

@pytest.fixture
def loco():
    bridge, auth, transports = Bridge(), Authorization(), []
    def factory():
        t = Transport(bridge); transports.append(t); return t
    backend = LocoMoveBackend(bridge, factory(), auth, transport_factory=factory)
    p = LocoMovePlugin({'enabled': True, 'action_timeout_s': 8.}, 'test', backend, notifier=Notifier())
    p.start()
    yield p, backend, bridge, auth, transports
    p.close()

def terminal(p, rid):
    deadline = time.monotonic() + 10
    while time.monotonic() < deadline:
        r = p._tasks.get(rid)
        if r and r['result'] is not None:
            return r['result']
        time.sleep(.01)
    raise AssertionError('Missing terminal result')

def call(p, action, **args):
    return DriverBundle([p]).dispatch('loco_move', {'action': action, **args})

def test_public_schema_merges_cards_and_tracks_real_actions(loco):
    p, _, _, _, _ = loco
    assert [t['name'] for t in p.get_tools()] == ['loco_move', 'relative_move']
    schema = p.get_tool()['inputSchema']
    assert schema['x-completion']['actions'] == ['stand_up', 'sitdown', 'move', 'stand_down']
    assert not {'stand', 'move_mode', 'prepare'} & set(schema['properties']['action']['enum'])
    jsonschema.validate({'action': 'stand_up'}, schema)
    jsonschema.validate({'action': 'move', 'direction': 'forward', 'speed': .08, 'duration': 1}, schema)
    jsonschema.validate({'action': 'stand_up', 'direction': 'forward'}, schema)
    with pytest.raises(jsonschema.ValidationError):
        jsonschema.validate({'action': 'stand_up', 'vx': .1}, schema)
    assert p.get_tool()['topic_out'] == []
    assert call(p, 'info')['topic_out'] == []
    assert p._relative_move_tool()['topic_out'][0]['topic'].endswith('/relative_move_status')

@pytest.mark.parametrize('state,expected', [(0, [1, 17]), (1, [17]), (17, [])])
def test_stand_up_finishes_at_move_mode_in_one_durable_task(loco, state, expected):
    p, _, bridge, _, transports = loco; bridge.state = state
    reply = call(p, 'stand_up', request_id='up')
    assert reply['accepted'], reply
    result = terminal(p, 'up')
    assert result['completed'], result
    assert result['reported_target_state'] == 17
    assert [s for t in transports for s, _ in t.states] == expected
    assert not any(t.velocities for t in transports)
    assert p._tasks._db.execute('SELECT COUNT(*) FROM tasks').fetchone()[0] == 1
    replay = call(p, 'stand_up', request_id='up')
    assert replay['replayed'] and 'action_id' not in replay
    assert [s for t in transports for s, _ in t.states] == expected

@pytest.mark.parametrize('scope', ['stand', 'move_mode'])
def test_both_original_posture_permissions_are_required(loco, scope):
    p, _, bridge, auth, transports = loco; bridge.state = 0
    auth.allowed.remove(scope)
    assert not call(p, 'stand_up')['accepted']
    assert not any(t.states for t in transports)

def test_stale_faults_and_changing_posture_do_not_start_sequence(loco):
    p, _, bridge, _, transports = loco
    for state in (4, 3, 99):
        bridge.state = state
        assert not call(p, 'stand_up')['accepted']
    bridge.state = 0; bridge.faults = [{'code': 33025}]
    assert not call(p, 'stand_up')['accepted']
    assert not any(t.states for t in transports)

def test_first_stage_failure_never_sends_move_mode(loco, monkeypatch):
    p, backend, bridge, _, transports = loco; bridge.state = 0
    original = backend.send
    def send(stage, req, cancel):
        receipt = original(stage, req, cancel)
        bridge.hes = 1
        return receipt
    monkeypatch.setattr(backend, 'send', send)
    reply = call(p, 'stand_up', request_id='fault'); assert reply['accepted']
    result = terminal(p, 'fault')
    assert not result['completed']
    assert [s for t in transports for s, _ in t.states] == [1]

def test_cancel_between_stages_never_enters_move_mode(loco, monkeypatch):
    p, backend, bridge, _, transports = loco; bridge.state = 0
    original = backend.target_feedback
    def feedback(stage, rid):
        p.cancel('test_cancel')
        return original(stage, rid)
    monkeypatch.setattr(backend, 'target_feedback', feedback)
    assert call(p, 'stand_up', request_id='cancel')['accepted']
    assert not terminal(p, 'cancel')['completed']
    assert [s for t in transports for s, _ in t.states] == [1]

def test_move_and_stand_down_reuse_bounded_controller(loco):
    p, _, bridge, _, transports = loco; bridge.state = 17
    assert call(p, 'move', request_id='move', direction='forward', speed=.08, duration=.5)['accepted']
    assert terminal(p, 'move')['stop_confirmed']
    assert call(p, 'stand_down', request_id='down')['accepted']
    assert terminal(p, 'down')['completed']
    assert bridge.state == 0

def test_private_and_retired_actions_are_not_public(loco):
    p, _, _, _, transports = loco
    for action in ('stand', 'move_mode', 'prepare', 'velocity'):
        with pytest.raises(ValueError): call(p, action)
    assert DriverBundle([p]).dispatch('timed_move', {}) is None
    assert DriverBundle([p]).dispatch('motion', {}) is None
    assert not any(t.states or t.velocities for t in transports)

def test_sequence_owns_mutex_across_both_stages(loco, monkeypatch):
    p, backend, bridge, _, transports = loco; bridge.state = 0
    entered, release = threading.Event(), threading.Event()
    original = backend.target_feedback
    def feedback(stage, rid):
        if stage == 'stand':
            entered.set(); release.wait(2)
        return original(stage, rid)
    monkeypatch.setattr(backend, 'target_feedback', feedback)
    try:
        assert call(p, 'stand_up', request_id='exclusive')['accepted']
        assert entered.wait(2)
        assert call(p, 'stand_down', request_id='conflict')['state'] == 'busy'
    finally: release.set()
    terminal(p, 'exclusive')

def test_revoke_after_first_state_does_not_send_second_state(loco, monkeypatch):
    p, backend, bridge, auth, transports = loco; bridge.state = 0
    original = backend.send
    def send(stage, req, cancel):
        r = original(stage, req, cancel); auth.valid = False; return r
    monkeypatch.setattr(backend, 'send', send)
    assert call(p, 'stand_up', request_id='revoke')['accepted']
    assert not terminal(p, 'revoke')['completed']
    assert [s for t in transports for s, _ in t.states] == [1]

def test_completion_is_one_outer_action_and_uses_public_tool_name(loco):
    p, _, bridge, _, _ = loco; bridge.state = 0
    payloads = []
    p._notifier.submit = payloads.append
    reply = call(p, 'stand_up', request_id='notify')
    terminal(p, 'notify')
    p._worker.join(1)
    deadline = time.monotonic() + 1
    while not payloads and time.monotonic() < deadline:
        p._flush_notifications()
        time.sleep(.01)
    assert len(payloads) == 1
    assert payloads[0]['action_id'] == reply['action_id']
    assert payloads[0]['tool'] == 'loco_move'
    assert payloads[0]['result']['action'] == 'stand_up'

def test_shared_status_keeps_relative_events_separate(loco):
    p, _, _, _, _ = loco
    p._event('synthetic', action='stand_up')
    p._event('synthetic', action='move_2s')
    p._event('synthetic', action='relative_move')
    status = call(p, 'info')
    assert [e['action'] for e in status['events']] == ['stand_up', 'move']
    assert set(status['readiness']['actions']) == {'stand_up', 'sitdown', 'move'}
    assert all(e['card'] == 'loco_move' for e in status['events'])

def test_deadline_and_missing_post_send_feedback_never_advance(loco, monkeypatch):
    p, backend, bridge, _, transports = loco; bridge.state = 0
    original = backend.target_feedback
    def expired(stage, rid):
        # Expire the finite outer deadline during state-1 observation.
        p._clock = lambda: time.monotonic() + 30
        return original(stage, rid)
    monkeypatch.setattr(backend, 'target_feedback', expired)
    assert call(p, 'stand_up', request_id='expires')['accepted']
    assert not terminal(p, 'expires')['completed']
    assert [s for t in transports for s, _ in t.states] == [1]

def test_restart_marks_sequence_interrupted_without_resuming(tmp_path):
    from deep_robotics.s10_pro.motion_tasks import MotionTasks
    path = tmp_path / 'tasks.sqlite3'
    tasks = MotionTasks(path)
    tasks.accept({'action': 'stand_up', 'request_id': 'interrupted', 'duration': 0.})
    tasks.close()
    bridge, auth, transport = Bridge(), Authorization(), None
    transport = Transport(bridge)
    backend = LocoMoveBackend(bridge, transport, auth)
    tasks = MotionTasks(path)
    p = LocoMovePlugin({'enabled': True}, 'test', backend, tasks=tasks, notifier=Notifier())
    try:
        record = tasks.get('interrupted')
        assert record['result']['status'] == 'error'
        assert not record['result']['completed']
        assert not transport.states and not transport.velocities
    finally:
        p.close(); tasks.close()

def test_default_runtime_advertises_same_merged_schema_without_grant(monkeypatch):
    import importlib.util
    from pathlib import Path
    monkeypatch.delenv('S10_OPERATOR_AUTHORIZATION_FILE', raising=False)
    monkeypatch.delenv('S10_EXECUTION_AUTHORIZATION_FILE', raising=False)
    driver = Path(__file__).resolve().parents[1] / 'deep_robotics/s10_pro'
    monkeypatch.syspath_prepend(str(driver))
    spec = importlib.util.spec_from_file_location('s10_loco_main', driver / 'main.py')
    module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
    plugins = module.build_runtime_plugins({}, 'test', Bridge(), lambda *a: None)
    try:
        b = DriverBundle(plugins)
        assert {'loco_move', 'relative_move'} <= {t['name'] for t in b.get_all_tools()}
        assert b.dispatch('loco_move', {'action': 'stand_up'})['accepted'] is False
    finally:
        for plugin in plugins:
            plugin.close() if hasattr(plugin, 'close') else plugin.stop()

@pytest.mark.parametrize('direction,sign', [('forward', 1), ('backward', -1)])
def test_expanded_linear_limit_runs_and_still_confirms_zero(loco, direction, sign):
    p, _, bridge, _, transports = loco; bridge.state = 17
    assert call(p, 'move', request_id='faster', direction=direction, speed=1., duration=.5)['accepted']
    result = terminal(p, 'faster')
    assert result['completed'] and result['stop_confirmed']
    assert result['commanded_speed'] == 1.
    frames = [v for t in transports for v in t.velocity_commands]
    assert any(vx == sign * 1. and vy == yaw == 0. for vx, vy, yaw in frames)
    assert frames[-2:] == [(0., 0., 0.), (0., 0., 0.)]

@pytest.mark.parametrize('speed', [1.0001, float('nan'), float('inf'), True])
def test_invalid_expanded_speed_is_rejected_before_sending(loco, speed):
    p, _, _, _, transports = loco
    with pytest.raises(ValueError): call(p, 'move', direction='forward', speed=speed, duration=.5)
    assert not any(t.velocity_commands for t in transports)

def test_smaller_configured_linear_cap_is_respected(loco):
    p, _, _, _, transports = loco
    call(p, 'config', max_vx=.1)
    with pytest.raises(ValueError): call(p, 'move', direction='forward', speed=.15, duration=.5)
    assert not any(t.velocity_commands for t in transports)


@pytest.mark.parametrize('direction,sign', [('turn_left', 1), ('turn_right', -1)])
def test_expanded_turning_cap_uses_rad_per_second_and_confirms_stop(loco, direction, sign):
    p, _, bridge, _, transports = loco
    bridge.state = 17
    assert call(p, 'move', request_id='faster-turn', direction=direction, speed=1., duration=.5)['accepted']
    result = terminal(p, 'faster-turn')
    assert result['completed'] and result['stop_confirmed']
    assert result['commanded_speed_unit'] == 'rad/s'
    frames = [v for t in transports for v in t.velocity_commands]
    assert any(yaw == sign * 1. and vx == vy == 0. for vx, vy, yaw in frames)
    assert frames[-2:] == [(0., 0., 0.), (0., 0., 0.)]


def test_lower_turning_cap_still_overrides_expanded_range(loco):
    p, _, _, _, transports = loco
    call(p, 'config', max_yaw_rate=.2)
    with pytest.raises(ValueError):
        call(p, 'move', direction='turn_left', speed=1., duration=.5)
    assert not any(t.velocity_commands for t in transports)

@pytest.mark.parametrize('hidden', [{'direction': 'forward'}, {'direction': 'forward', 'speed': .08, 'duration': 2.}])
@pytest.mark.parametrize('action,state,states', [('stand_up', 0, [1, 17]), ('sitdown', 17, [4])])
def test_canvas_hidden_move_fields_do_not_block_posture(loco, action, state, states, hidden):
    p, _, bridge, _, transports = loco
    bridge.state = state
    # Official Core's canvas collects every nonempty input, including fields
    # hidden by x-action-params. The direction select is never blank.
    jsonschema.validate({'action': action, **hidden}, p.get_tool()['inputSchema'])
    reply = call(p, action, request_id='canvas-posture', **hidden)
    assert reply['accepted'], reply
    result = terminal(p, 'canvas-posture')
    assert result['completed'], result
    assert [s for t in transports for s, _ in t.states] == states
    assert not any(t.velocity_commands for t in transports)


def test_canvas_hidden_fields_do_not_block_status_or_stop(loco):
    p, _, _, _, transports = loco
    hidden = {'direction': 'turn_left', 'speed': .2, 'duration': 1.}
    assert call(p, 'status', **hidden)['card'] == 'loco_move'
    assert call(p, 'stop_move', **hidden)['reason'] == 'no_application_motion'
    assert not any(t.states or t.velocity_commands for t in transports)


def test_ui_default_remains_read_only_status(loco):
    p, _, _, _, _ = loco
    actions = p.get_tool()['inputSchema']['properties']['action']['enum']
    visible = [a for a in actions if a not in {'start', 'stop', 'info', 'config'}]
    assert visible[0] == 'status'


def test_canvas_compatibility_keeps_unknown_fields_strict(loco):
    p, _, _, _, transports = loco
    for action in ('stand_up', 'stand_down', 'stop_move', 'status'):
        with pytest.raises(ValueError):
            call(p, action, direction='forward', speed=.1, duration=1., vx=.1)
    assert not any(t.states or t.velocity_commands for t in transports)


def test_canvas_hidden_fields_do_not_block_config_or_start(loco):
    p, _, _, _, transports = loco
    hidden = {'direction': 'forward', 'speed': 1., 'duration': 2.}
    call(p, 'config', max_vx=.2, **hidden)
    call(p, 'stop', **hidden)
    call(p, 'start', **hidden)
    assert p._settings['max_vx'] == .2
    assert not any(t.states or t.velocity_commands for t in transports)


def test_canvas_stop_with_hidden_fields_cancels_active_move(loco):
    p, _, _, _, transports = loco
    fields = {'direction': 'forward', 'speed': .1, 'duration': 2.}
    assert call(p, 'move', request_id='canvas-stop', **fields)['accepted']
    deadline = time.monotonic() + 1.
    while not any(t.velocity_commands for t in transports) and time.monotonic() < deadline:
        time.sleep(.01)
    call(p, 'stop_move', **fields)
    result = terminal(p, 'canvas-stop')
    assert not result['completed'] and result['stop_confirmed']
    assert next(t for t in transports if t.velocity_commands).velocity_commands[-1] == (0., 0., 0.)


@pytest.mark.parametrize('initial,expected', [(0, [1, 17]), (1, [17]), (17, [])])
def test_merged_stand_up_uses_real_prepare_permits(prepare_policy, initial, expected):
    from deep_robotics.s10_pro.motion_dds import validate_operator_permit
    from deep_robotics.s10_pro.automatic_prepare import AutomaticPreparation
    from deep_robotics.s10_pro.operator_authorization import AUTOMATIC_PREPARE_POLICY_SCHEMA
    import json
    manifest, policy_state, load = prepare_policy
    manifest.update(schema=AUTOMATIC_PREPARE_POLICY_SCHEMA, activation='automatic_prepare',
                    provenance='owner_approved_automatic_prepare',
                    allow_automatic_startup=True, allow_automatic_renewal=True)
    policy_state['raw'] = json.dumps(manifest).encode()
    grant, bridge = load(real_clock=True), Bridge()
    bridge.state = initial

    class CheckingTransport(Transport):
        def publish_state(self, state, *, authority, cancel_event):
            checked = validate_operator_permit(authority, now=time.monotonic())
            assert checked['action'] == {1: 'stand', 17: 'move_mode'}[state]
            return super().publish_state(state, authority=authority, cancel_event=cancel_event)

    transports = []
    def factory():
        transport = CheckingTransport(bridge)
        transports.append(transport)
        return transport
    backend = LocoMoveBackend(bridge, factory(), grant, transport_factory=factory)
    p = LocoMovePlugin({'enabled': True}, 'test', backend, notifier=Notifier())
    supervisor = AutomaticPreparation(p)
    try:
        supervisor.tick()
        assert supervisor.status()['state'] == 'session_active'
        assert grant.authorized_now()
        assert all(not t.states and not t.velocity_commands for t in transports)
        assert call(p, 'stand_up', request_id='real-grant', direction='forward')['accepted']
        assert terminal(p, 'real-grant')['completed']
        assert [state for t in transports for state, _ in t.states] == expected
        assert all(not t.velocity_commands for t in transports)
    finally:
        supervisor.close()
        p.close()


def test_stand_up_reports_second_stage_preflight_blocker(loco, monkeypatch):
    p, backend, bridge, _, transports = loco
    bridge.state = 0
    original = backend.preflight
    def preflight(**kwargs):
        if kwargs.get('stationary_states') == (1,):
            return {'ready': False, 'blocked_by': ['fresh_stationary_feedback_required']}
        return original(**kwargs)
    monkeypatch.setattr(backend, 'preflight', preflight)
    assert call(p, 'stand_up', request_id='stage-blocked')['accepted']
    result = terminal(p, 'stage-blocked')
    assert result['reason'] == 'stage_preflight_unavailable'
    assert result['failed_stage'] == 'move_mode'
    assert result['failure_phase'] == 'preflight'
    assert result['blocked_by'] == ['fresh_stationary_feedback_required']
    assert [state for t in transports for state, _ in t.states] == [1]


def test_legacy_execution_mode_is_explicitly_rejected(monkeypatch):
    import importlib.util
    from pathlib import Path
    from types import SimpleNamespace
    driver = Path(__file__).resolve().parents[1] / 'deep_robotics/s10_pro'
    monkeypatch.syspath_prepend(str(driver))
    spec = importlib.util.spec_from_file_location('s10_loco_legacy', driver / 'main.py')
    module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
    monkeypatch.setenv('S10_EXECUTION_AUTHORIZATION_FILE', '/nonexistent-test-only')
    with pytest.raises(ValueError, match='legacy execution-contract mode'):
        module.build_runtime_plugins({}, 'test', Bridge(), lambda *a: None,
                                     runtime_binding=SimpleNamespace(authorized=False))


def test_sitdown_is_visible_and_old_alias_remains_callable(loco):
    p, _, bridge, _, transports = loco
    schema = p.get_tool()['inputSchema']
    actions = schema['properties']['action']['enum']
    assert 'sitdown' in actions and 'stand_down' not in actions
    assert {'sitdown', 'stand_down'} <= set(schema['x-completion']['actions'])
    bridge.state = 17
    result = call(p, 'sitdown', request_id='new-sitdown')
    assert result['action'] == 'sitdown' and result['action_id']
    assert terminal(p, 'new-sitdown')['completed']
    bridge.state = 17
    alias = call(p, 'stand_down', request_id='legacy-sitdown')
    assert alias['action'] == 'sitdown'
    assert terminal(p, 'legacy-sitdown')['completed']
    assert [state for t in transports for state, _ in t.states] == [4, 4]
    assert not any(t.velocity_commands for t in transports)


def test_async_acceptance_returns_before_feedback_and_one_acknowledged_acp(loco, tmp_path, monkeypatch):
    from deep_robotics.s10_pro.acp import ACPNotifier, ACPResponse
    p, backend, bridge, _, _ = loco
    bridge.state = 0
    entered, release = threading.Event(), threading.Event()
    original = backend.target_feedback
    posts = []
    def core(payload):
        posts.append(payload)
        return ACPResponse(200, {'ok': True, 'action_id': payload['action_id']})
    notifier = ACPNotifier(tmp_path / 'outbox', transport=core)
    p._notifier = notifier
    def feedback(stage, rid):
        if stage == 'stand':
            entered.set()
            assert release.wait(3)
        return original(stage, rid)
    monkeypatch.setattr(backend, 'target_feedback', feedback)
    try:
        receipt = call(p, 'stand_up', request_id='async-public')
        assert entered.wait(1)
        assert receipt['action_id'] and receipt['status'] == 'running'
        assert not receipt['completed']
        assert 'ACP' in receipt['message'] and len(str(receipt)) < 700
        assert p._tasks.get('async-public')['result'] is None
        assert not posts
        release.set()
        assert terminal(p, 'async-public')['completed']
        p._worker.join(1)
        p._flush_notifications()
        assert notifier.process_pending() == 1
        assert len(posts) == 1
        callback = posts[0]
        assert callback['action_id'] == receipt['action_id']
        assert callback['tool'] == 'loco_move'
        assert callback['result']['completed']
        assert 'stages' not in callback['result']
        assert notifier.status(receipt['action_id'])['acknowledged']
        assert notifier.process_pending() == 0
        assert len(posts) == 1
        # Full evidence remains in the durable task/info result.
        assert len(p._tasks.get('async-public')['result']['stages']) == 2
    finally:
        release.set()
        p._worker.join(4)
        p._notifier = Notifier()
        notifier.close()


def test_compact_status_and_rejections_keep_diagnostics_in_info(loco):
    import json
    p, _, bridge, _, _ = loco
    status = call(p, 'status')
    detailed = call(p, 'info')
    assert 'message' in status and 'readiness' not in status
    assert 'events' not in status and 'operator_session' not in status
    assert 'readiness' in detailed and 'events' in detailed
    bridge.faults = [{'code': 33025}]
    rejected = call(p, 'stand_up')
    assert not rejected['accepted']
    assert 'fresh_clear_fault_snapshot_required' in rejected['blocked_by']
    assert 'message' in rejected and 'readiness' not in rejected
    assert len(json.dumps(rejected)) < 1200
