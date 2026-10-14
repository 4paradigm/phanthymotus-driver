"""Explicit buttons and native reset, real executor with a synthetic SDK/plant."""
import copy
from types import SimpleNamespace
import threading
import time

import pytest
from test_arm_stream import rig
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
    assert begun == ['begin']
    for _ in range(15): feed('xa')
    assert begun == ['begin']  # One event per press even when setup fails.
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


def test_reset_calls_native_99_once_then_requires_new_baseline(tmp_path):
    card, arm, sample, advance, sdk, feed = reset_rig(tmp_path)
    card._operator = 'old'; card._enabled = True
    card._hold = lambda *a, **kw: setattr(card, '_enabled', False)
    for _ in range(11): feed('yb')
    assert card._reason == 'resetting' and card._operator is None
    assert card.mapping.anchor is None and card._needs_calibration
    for _ in range(60):
        feed('yb'); arm.tick(); time.sleep(.001)
    assert sdk == [99]
    assert card._reset_operation is None and card._reason == 'waiting_baseline_confirmation'
    assert not card._enabled and card._operator is None
    for _ in range(15): feed('yb')
    assert sdk == [99]  # Holding Reset after completion does not trigger again.
    begun = []
    card._begin = lambda generation, action: begun.append(action)
    feed()
    for _ in range(11): feed('xa')
    assert begun == ['calibrate']


def test_reset_sdk_failure_never_claims_completion(tmp_path):
    card, arm, sample, advance, sdk, feed = reset_rig(tmp_path, SimpleNamespace(ExecuteAction=lambda _: 7404))
    for _ in range(11): feed('yb')
    for _ in range(60):
        feed('yb'); arm.tick(); time.sleep(.001)
        if arm._operation['state'] == 'failed':
            feed('yb'); break
    assert card._state == 'hold' and card._reason == 'release_sdk_error'
    assert not card._enabled and not card._operator
    assert arm._operation['physical_confirmed'] is False


def test_stop_fences_reset_before_native_action(tmp_path):
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
    card._stop_motion = lambda: None
    for _ in range(11): feed('xa')
    assert bool(card._operator) == (changed == 'none')
    assert bool(card.mapping.anchor) == (changed == 'none')
    assert not card._enabled and not sdk


def test_one_stale_frame_cannot_complete_button_hold(tmp_path):
    card, arm, sample, advance, sdk, feed = reset_rig(tmp_path)
    begun = []
    card._begin = lambda *args: begun.append(True)
    feed('xa')
    for _ in range(12): advance(100); card.step()
    assert not begun and not sdk
