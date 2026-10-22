"""Explicit buttons and Canvas-compatible handback, real executor with a synthetic SDK/plant."""
import copy
import json
from types import SimpleNamespace
import threading
import time

import pytest
from test_arm_stream import rig, packet, JOINT_NAMES
from test_teleop_settings import input_card
from teleop_control import TeleopControl


def reset_rig(tmp_path, client=None):
    arm, sample, channel, advance, _, sdk = rig(tmp_path, client=client)
    motion = SimpleNamespace(executor=arm, gate=arm, clock_id='offline', _lock=threading.RLock(),
        _preview=None, _preview_state='idle', cancel_pending=lambda: None, _decision={})
    card = TeleopControl({'mode': 'live'}, motion, clock=arm.clock)
    card.binding = {'command_topic': '/teleop/command', 'feedback_topic': '/teleop/state'}
    _, frame = input_card(tmp_path)
    def feed(buttons='', held=False, ms=100):
        advance(ms)
        frame['sequence'] += 1
        frame['received_monotonic_ns'] = arm.clock()
        for side, names in (('left', 'xy'), ('right', 'ab')):
            frame[side]['grip'] = float(held)
            frame[side]['controls'] = {'buttons': {name: {'available': True, 'pressed': name in buttons} for name in names}}
        card.receive(copy.deepcopy(frame)); card.step()
        return frame
    return card, arm, sample, advance, sdk, feed


def test_explicit_confirmation_no_first_frame_or_held_grip_calibration(tmp_path):
    card, arm, sample, advance, sdk, feed = reset_rig(tmp_path)
    begun = []
    card._begin = lambda generation, action: begun.append(action)
    for _ in range(15): feed()
    assert not begun and card.mapping.anchor is None
    for _ in range(15): feed('xa', held=True)
    assert not begun
    for _ in range(11): feed('xa')
    assert begun == ['calibrate']
    for _ in range(15): feed('xa')
    assert begun == ['calibrate']  # One event per press even when setup fails.
    assert not sdk


def test_interrupted_or_conflicting_confirmation_does_not_count(tmp_path):
    card, arm, sample, advance, sdk, feed = reset_rig(tmp_path)
    begun = []
    card._begin = lambda *args: begun.append(True)
    for _ in range(8): feed('xa')
    feed('xa', ms=400)
    for _ in range(4): feed('xa')
    assert not begun
    for _ in range(15): feed('xyab')
    assert not begun and not sdk
    feed()
    for _ in range(11): feed('xa')
    assert begun == [True]


def test_reset_hands_back_once_then_requires_new_baseline(tmp_path):
    card, arm, sample, advance, sdk, feed = reset_rig(tmp_path)
    card._operator = 'old'; card._enabled = True
    card._hold = lambda *a, **kw: setattr(card, '_enabled', False)
    for _ in range(11): feed('yb')
    assert card._reason == 'resetting' and card._operator is None
    assert card.mapping.anchor is None and card._needs_calibration
    for _ in range(60):
        feed('yb'); arm.tick(); time.sleep(.001)
    assert sdk == []
    assert card._reset_operation is None and card._reason == 'waiting_baseline_confirmation'
    assert not card._enabled and card._operator is None
    for _ in range(15): feed('yb')
    assert sdk == []  # Holding Reset after completion does not trigger again.
    begun = []
    card._begin = lambda generation, action: begun.append(action)
    feed()
    for _ in range(11): feed('xa')
    assert begun == ['calibrate']


def test_reset_write_failure_never_claims_completion(tmp_path):
    card, arm, sample, advance, sdk, feed = reset_rig(tmp_path)
    arm._channel.success = False
    for _ in range(11): feed('yb')
    for _ in range(60):
        feed('yb'); arm.tick(); time.sleep(.001)
        if arm._operation['state'] == 'failed':
            feed('yb'); break
    assert card._state == 'hold' and card._reason == 'arm_sdk_write_failed'
    assert not card._enabled and not card._operator
    assert arm._operation['physical_confirmed'] is False
    feed()
    advance(400); card.step()
    feed()
    assert card._state == 'hold' and card._reason == 'arm_sdk_write_failed'
    assert not card._operator and not card._enabled


def test_stop_keeps_reset_on_handback_path(tmp_path):
    card, arm, sample, advance, sdk, feed = reset_rig(tmp_path)
    for _ in range(11): feed('yb')
    card.cancel_for_arm_release()
    result = arm.dispatch('stop', {})
    assert result['action_id'] is None
    for _ in range(60): advance(100); arm.tick()
    assert not sdk and not card._enabled


@pytest.mark.parametrize('changed', ['none', 'buttons', 'moving', 'source'])
def test_baseline_rechecks_confirmation_after_model_preparation(tmp_path, changed):
    card, arm, sample, advance, sdk, feed = reset_rig(tmp_path)
    motion = card.motion
    motion.solver = SimpleNamespace(profile={})
    motion._snapshot = {'poses': [[0, 0, 0, 0, 0, 0, 1]]*2, 'monotonic_ns': arm.clock()}
    motion.dispatch = lambda *a: {}
    def refresh():
        if changed == 'buttons': card._latest['left']['controls']['buttons']['x']['pressed'] = False
        if changed == 'moving': sample['dq'][0] = .1
        if changed == 'source': card._input_changed = True
        motion._snapshot['monotonic_ns'] = arm.clock()
    motion._refresh_snapshot = refresh
    card._stop_motion = lambda **kwargs: None
    for _ in range(11): feed('xa')
    assert bool(card._operator) == (changed in ('none', 'moving'))
    assert bool(card.mapping.anchor) == (changed in ('none', 'moving'))
    assert not card._enabled and not sdk


def test_one_stale_frame_cannot_complete_button_hold(tmp_path):
    card, arm, sample, advance, sdk, feed = reset_rig(tmp_path)
    begun = []
    card._begin = lambda *args: begun.append(True)
    feed('xa')
    for _ in range(12): advance(100); card.step()
    assert not begun and not sdk


@pytest.mark.parametrize('failure', ['input_missing', 'input_stale', 'tracking_lost'])
def test_uninitialized_input_recovery_restores_confirmation_only(tmp_path, failure):
    card, arm, sample, advance, sdk, feed = reset_rig(tmp_path)
    begun = []
    card._begin = lambda *args: begun.append(True)
    if failure != 'input_missing':
        feed()
        if failure == 'input_stale': advance(400)
        else: card._latest['left'].update(tracked=False, position=None, orientation_xyzw=None)
    card.step()
    assert card._state == 'hold' and card._reason == failure
    feed()
    assert card._state == 'ready' and card._reason == 'waiting_baseline_confirmation'
    assert not begun and not sdk and not card._enabled and card.mapping.anchor is None


def test_reset_interrupted_by_input_gap_stays_failed_after_recovery(tmp_path):
    card, arm, sample, advance, sdk, feed = reset_rig(tmp_path)
    for _ in range(11): feed('yb')
    advance(400); card.step()
    assert card._state == 'hold' and card._reason == 'reset_interrupted'
    feed()
    assert card._state == 'hold' and card._reason == 'reset_interrupted'
    assert not sdk and not card._operator and not card._enabled


def test_operator_reset_confirms_handback_and_allows_next_explicit_session(tmp_path):
    card, arm, sample, advance, sdk, feed = reset_rig(tmp_path)
    old_session = arm.session_id
    for _ in range(11): feed('yb')
    for _ in range(40):
        feed('yb'); arm.tick(); time.sleep(.001)
    op = arm._operation
    assert sdk == [] and not op['request_sent'] and op['state'] == 'completed'
    assert op['physical_confirmed'] and op['authority_released']
    assert op['action_id'] is None
    assert not arm.status()['ownership_held']
    assert arm.latest is None and not arm._proofs and arm._channel.closed
    assert card._reason == 'waiting_baseline_confirmation' and not card._enabled

    motion = card.motion
    motion.solver = SimpleNamespace(profile={})
    motion._snapshot = {'poses': [[0, 0, 0, 0, 0, 0, 1]]*2}
    def dispatch(action, args):
        if action == 'prepare_operator_session': return arm.dispatch(action, args)
        assert action in ('calibrate', 'prepare_preview')
        return {}
    motion.dispatch = dispatch
    motion._refresh_snapshot = lambda: motion._snapshot.update(monotonic_ns=arm.clock())
    feed()
    for _ in range(11): feed('xa')
    assert card._operator and card.mapping.epoch == 1 and not card._enabled
    assert not arm.status()['ownership_held'] and sdk == []
    # A subsequent explicit claim uses the ordinary execution checks and a new lease.
    sample['dq'] = [0.]*10
    arm._channel.open = lambda: setattr(arm._channel, 'closed', False)
    claimed = arm.dispatch('claim', {})
    assert claimed['state'] == 'ready' and claimed['session_id'] != old_session
    arm.accept_control(packet(arm)); advance(10); arm.tick()
    assert arm.status()['output_active'] and sdk == []


@pytest.mark.parametrize('failure', ['none', 'unmatched', 'write'])
def test_reset_sdk_request_uses_no_reply_and_propagates_dds_write_failure(failure, monkeypatch):
    import subprocess, sys
    from pathlib import Path
    result = subprocess.run([sys.executable, '-c',
        'from test_teleop_reset import _sdk_reset_check; import sys; _sdk_reset_check(sys.argv[1])', failure],
        cwd=Path(__file__).parent, capture_output=True, text=True, timeout=10)
    assert result.returncode == 0, result.stdout + result.stderr


def _sdk_reset_check(failure):
    import json
    from unitree_sdk2py.core.channel import Channel
    from unitree_sdk2py.rpc.client_stub import ClientStub
    from unitree_sdk2py.g1.arm.g1_arm_action_client import G1ArmActionClient
    writes = []
    def write(request):
        if failure == 'write': raise RuntimeError('synthetic DDS failure')
        writes.append(request)
    writer = Channel._Channel__Writer()
    writer._Writer__publication_matched_count = 0 if failure == 'unmatched' else 1
    writer._Writer__writer = SimpleNamespace(write=write)
    stub = ClientStub.__new__(ClientStub)
    stub._ClientStub__sendChannel = writer
    stub._ClientStub__fail_warns = 0
    client = G1ArmActionClient.__new__(G1ArmActionClient)
    client._Client__apiMapping = {}
    client._Client__enableLease = False
    client._ClientBase__stub = stub
    client._ClientBase__timeout = .001
    client.Init()
    assert client.RequestReset() == (0 if failure == 'none' else 3102)
    if failure == 'none':
        assert len(writes) == 1 and writes[0].header.policy.noreply is True
        assert json.loads(writes[0].parameter) == {'data': 99}
    else: assert not writes


@pytest.mark.parametrize('state', ['ready', 'active', 'hold', 'fault'])
def test_confirmation_rebuilds_existing_baseline_in_every_state(tmp_path, state):
    card, arm, sample, advance, sdk, feed = reset_rig(tmp_path)
    card._operator = 'existing'
    card._state = state
    card._hold = lambda *a, **kw: setattr(card, '_enabled', False)
    begun = []
    card._begin = lambda generation, action: begun.append(action)
    for _ in range(11): feed('xa')
    assert begun == ['calibrate']
    for _ in range(15): feed('xa')
    assert begun == ['calibrate']


@pytest.mark.parametrize('outcome', ['failed', 'unknown'])
def test_operator_can_retry_reset_or_confirm_after_error(tmp_path, outcome):
    card, arm, sample, advance, sdk, feed = reset_rig(tmp_path)
    for _ in range(11): feed('yb')
    op = arm._operation
    op.update(state=outcome, handback_confirmed=True)
    sample['dq'] = [.03]*10
    feed('yb'); feed()
    for _ in range(11): feed('yb')
    if outcome == 'unknown':
        assert arm._operation is op and card._reason == 'robot_not_stopped'
        sample['dq'] = [0.]*10
        feed()
        for _ in range(11): feed('yb')
    assert arm._operation is not op and arm._operation['state'] == 'holding'
    arm._operation.update(state=outcome, handback_confirmed=True)
    assert arm.confirm_operator_reset()
    assert arm._operation['state'] == 'completed'
    assert not arm.status()['ownership_held'] and not arm.release_requested
    assert not arm._operation['physical_confirmed'] and not sdk


def test_confirmation_during_reset_cancels_unsent_action(tmp_path):
    card, arm, sample, advance, sdk, feed = reset_rig(tmp_path)
    for _ in range(11): feed('yb')
    begun = []
    def begin(generation, action):
        begun.append(action)
        assert arm.confirm_operator_reset()
    card._begin = begin
    feed()
    for _ in range(11): feed('xa')
    assert begun == ['calibrate']
    for _ in range(40): advance(100); arm.tick()
    assert not sdk and not arm.status()['ownership_held']


def test_active_confirmation_hands_back_without_reset_or_stationarity(tmp_path):
    card, arm, sample, advance, sdk, feed = reset_rig(tmp_path)
    sample['dq'] = [.03]*10
    arm._stationary = lambda *args: pytest.fail('explicit baseline must not require rest')
    assert arm.confirm_operator_reset()
    assert arm._operation['action_id'] is None
    for _ in range(40): advance(100); arm.tick()
    assert arm._operation['state'] == 'completed'
    assert not arm.status()['ownership_held'] and not arm.release_requested
    assert not sdk and not arm._operation['physical_confirmed']


@pytest.mark.parametrize('buttons', ['xa', 'yb'])
def test_batched_button_release_restarts_long_press(tmp_path, buttons):
    card, arm, sample, advance, sdk, feed = reset_rig(tmp_path)
    events = []
    card._begin = lambda *a: events.append('confirm')
    card._start_reset = lambda: events.append('reset')
    for _ in range(8): frame = feed(buttons)
    released = copy.deepcopy(frame)
    advance(10)
    released.update(sequence=frame['sequence']+1, received_monotonic_ns=arm.clock())
    for side in ('left', 'right'):
        for button in released[side]['controls']['buttons'].values(): button['pressed'] = False
    assert card.receive(released)
    frame['sequence'] += 1
    for _ in range(4): feed(buttons)
    assert not events and not sdk
    for _ in range(7): feed(buttons)
    assert events == ['confirm' if buttons == 'xa' else 'reset']


@pytest.mark.parametrize('failure', ['none', 'stale', 'moving', 'write', 'idle'])
def test_reset_matches_canvas_stop_without_pose_or_vendor_action(tmp_path, failure):
    outcomes = []
    for action in ('stop', 'operator_reset'):
        path = tmp_path/action
        path.mkdir()
        _, arm, sample, advance, sdk, _ = reset_rig(path)
        arm.cfg['reset_pose_path'] = str(path/'does-not-exist.json')
        arm.arm_client.RequestReset = lambda: pytest.fail('Reset must not send 99')
        sample['q'] = [.17]*10
        if failure == 'idle':
            arm.session_id = arm.secret = None
            arm._channel_open = False
        if failure == 'write': arm._channel.success = False
        if failure == 'moving': sample['dq'] = [.03]*10
        result = arm.dispatch(action, {})
        if failure == 'stale':
            advance(400, fresh=False); arm.tick()
        elif failure != 'idle':
            for _ in range(110): advance(100, follow=False); arm.tick()
        op = arm._operation or result
        outcomes.append((op['state'], op.get('reason'), op.get('authority_released'),
                         arm.status()['ownership_held'], arm._channel.writes))
        assert not sdk and op.get('action_id') is None and not op.get('request_sent')
        assert all(q == [.17]*10 for q, _, _ in arm._channel.writes)
        if failure in ('none', 'idle'):
            assert op['state'] == 'completed' and op['authority_released']
            assert not arm.status()['ownership_held']
        else:
            assert op['state'] in ('failed', 'unknown') and not op['authority_released']
    assert outcomes[0] == outcomes[1]


def test_reset_without_ownership_keeps_input_running(tmp_path):
    card, arm, sample, advance, sdk, feed = reset_rig(tmp_path)
    arm.session_id = arm.secret = None
    arm._channel_open = False
    card.mapping.anchor = {'old': True}
    for _ in range(11): feed('yb')
    assert card._reason == 'waiting_baseline_confirmation'
    assert card._reset_operation is None and not card._closed.is_set()
    assert card.binding and card.mapping.anchor is None and card._needs_calibration
    assert not sdk and not arm._channel.writes


def test_reset_waits_for_confirmed_handback_even_after_input_recovers(tmp_path):
    card, arm, sample, advance, sdk, feed = reset_rig(tmp_path)
    sample['dq'] = [.03]*10
    for _ in range(11): feed('yb')
    for _ in range(50): feed('yb'); arm.tick()
    assert card._reason == 'resetting'
    assert arm.status()['ownership_held'] and not arm._operation['authority_released']
    sample['dq'] = [0.]*10
    for _ in range(8): feed('yb'); arm.tick()
    assert card._reason == 'waiting_baseline_confirmation'
    assert not arm.status()['ownership_held'] and not sdk
