"""G1 admission uses real coordinator; numerical/hardware output is substituted."""
import copy
import importlib.util
import os
from pathlib import Path
import time

import pytest
from test_motion_control_contract import controller
from test_teleop_settings import input_card
from teleop_control import TeleopControl


def aligned_display():
    arm = {'position_error_m': .001, 'orientation_error_rad': .01,
           'position_tolerance_m': .03, 'orientation_tolerance_rad': .2617993877991494}
    return {'available': True, 'feedback_fresh': True, 'intent_age_ms': 0.,
            'measured': [[[0, 0, 0], [0, 0, .1]]]*2, 'ik': [[[0, 0, 0], [0, 0, .1]]]*2,
            'alignment': {'state': 'ready', 'ready': True, 'left': arm, 'right': arm}}


def rig(tmp_path):
    motion, outputs, _ = controller()
    motion.executor.cfg = {}
    motion.start = lambda: None
    motion.gate.status_original = motion.gate.status
    motion.gate.status = lambda: dict(motion.gate.status_original(), reason=None, stop_confirmed=False, applied_sequence=-1,
                                      resume_ready=True, continuation_ready=False)
    def dispatch(action, args):
        if action in ('pause', 'recoverable_hold'):
            motion.gate.hold('operator_pause'); return {}
        if action == 'resume':
            motion.gate.state = 'ready'
            motion.gate.session_id += '-resumed'
            return {'session_id': motion.gate.session_id, 'secret': motion.gate.secret, 'boot_id': motion.gate.boot_id}
        raise AssertionError(action)
    motion.executor.dispatch = dispatch
    card = TeleopControl({'mode': 'live'}, motion)
    _, frame = input_card(tmp_path)
    card.clock_id = 'offline'
    card.binding = {'command_topic': '/teleop/command', 'feedback_topic': '/teleop/state'}
    card._operator = 'test-operator'
    card.mapping.calibrate(frame, [[0, 0, 0, 0, 0, 0, 1]]*2)
    card._mapping_identity = (frame['device_id'], frame['space_epoch'])
    motion.solver.render = lambda *args: {'poses': [[0, 0, 0, 0, 0, 0, 1]]*2, 'visualization': aligned_display()}
    def feed(seq, held=False, **changes):
        value = copy.deepcopy(frame)
        value.update(sequence=seq, received_monotonic_ns=time.monotonic_ns(), **changes)
        value['left']['grip'] = value['right']['grip'] = float(held)
        assert card.receive(value)
        card.step()
        motion.process_latest()
        card.step()
        motion.process_latest()
        return value
    return card, motion, outputs, feed


def test_preview_then_new_grip_and_regrip_required_after_pause(tmp_path):
    card, motion, outputs, feed = rig(tmp_path)
    feed(1)
    feed(2)
    assert card.display_feedback()['alignment']['ready'] and not outputs
    feed(3, True)
    assert card._enabled and len(outputs) == 1
    feed(4, False)
    assert not card._enabled and len(outputs) == 1
    anchor = card.mapping.anchor
    feed(5, False); feed(6, True)
    assert len(outputs) == 2 and card.mapping.anchor is anchor


def test_premature_grip_never_enables_when_alignment_recovers(tmp_path):
    card, motion, outputs, feed = rig(tmp_path)
    feed(1, True); feed(2, True); feed(3, True)
    assert not outputs and not card._enabled
    feed(4, False); feed(5, True)
    assert len(outputs) == 1


def test_failure_revokes_ready_and_held_grips_cannot_auto_resume(tmp_path):
    card, motion, outputs, feed = rig(tmp_path)
    feed(1); feed(2); feed(3, True)
    solve = motion.solver.solve_frame
    motion.solver.solve_frame = lambda *a: (_ for _ in ()).throw(ValueError('ik_unreachable'))
    feed(4, True)
    motion.solver.solve_frame = solve
    feed(5, True); feed(6, True)
    assert len(outputs) == 1 and not card._enabled
    feed(7, False); feed(8, True)
    assert len(outputs) == 2


def test_stale_and_connection_reset_cannot_reuse_ready(tmp_path):
    card, motion, outputs, feed = rig(tmp_path)
    feed(1); feed(2)
    motion._visualization['valid_until_ns'] = card.clock()-1
    assert card.display_feedback()['alignment']['state'] == 'unavailable'
    feed(3, True, connection_epoch=2)
    assert not outputs and not card._enabled
    with pytest.raises(ValueError, match='old_input_generation'): feed(4, True, connection_epoch=1)


def test_expired_display_keeps_geometry_and_timestamps_without_outputs(tmp_path):
    card, motion, outputs, feed = rig(tmp_path)
    feed(1); feed(2)
    motion._visualization['valid_until_ns'] = card.clock()-1
    before = copy.deepcopy(motion._visualization)
    display = card.display_feedback()
    visual = display['visualization']
    assert visual['available'] and visual['stale'] and not visual['feedback_fresh']
    assert display['alignment'] == {'state':'unavailable', 'ready':False}
    for field in ('ik', 'measured', 'sampled_ns', 'valid_until_ns', 'intent_received_ns', 'source_sequence'):
        assert visual[field] == before[field]
    assert not outputs and motion._visualization == before


def test_refresh_expired_preview_freezes_existing_display_only(tmp_path):
    card, motion, outputs, feed = rig(tmp_path)
    feed(1); feed(2)
    deadline = card.clock()-1
    motion._intent['valid_until_ns'] = motion._visualization['valid_until_ns'] = deadline
    before = copy.deepcopy(motion._visualization)
    motion._refresh_snapshot()
    display = card.display_feedback()['visualization']
    for field in ('ik', 'measured', 'sampled_ns', 'valid_until_ns', 'intent_received_ns', 'source_sequence'):
        assert display[field] == before[field]
    assert display['stale'] and not display['feedback_fresh'] and not outputs


@pytest.mark.parametrize('boundary', ['closed', 'missing', 'input_stale', 'tracking', 'identity', 'calibration', 'mapping', 'cancel'])
def test_display_does_not_retain_across_non_expiry_boundaries(tmp_path, boundary):
    card, motion, outputs, feed = rig(tmp_path)
    feed(1); feed(2)
    motion._visualization['valid_until_ns'] = card.clock()-1
    if boundary == 'closed': card._closed.set()
    elif boundary == 'missing': card._latest = None
    elif boundary == 'input_stale': card._latest['received_monotonic_ns'] -= 301_000_000
    elif boundary == 'tracking': card._latest['left']['tracked'] = False
    elif boundary == 'identity': card._input_changed = True
    elif boundary == 'calibration': card._needs_calibration = True
    elif boundary == 'mapping': card.mapping.epoch += 1
    else: motion.cancel_pending()
    display = card.display_feedback()
    assert not display['visualization']['available']
    assert all(not display['visualization'][key] for key in ('ik','measured','command','targets','torso'))
    assert display['alignment'] == {'state':'unavailable', 'ready':False} and not outputs


def test_unreachable_result_never_borrows_expired_ik(tmp_path):
    card, motion, outputs, feed = rig(tmp_path)
    feed(1); feed(2)
    motion._intent['valid_until_ns'] = card.clock()-1
    motion._decision = {'state':'error', 'reason':'ik_unreachable'}
    unreachable = aligned_display()
    unreachable['ik'] = []
    unreachable['alignment'].update(state='unreachable', ready=False)
    motion.solver.render = lambda *args: {'poses':[[0,0,0,0,0,0,1]]*2, 'visualization':unreachable}
    motion._refresh_snapshot()
    assert card.display_feedback()['visualization']['ik'] == [] and not outputs


def test_g1_feedback_passes_current_pico_consumer(tmp_path):
    root = os.environ.get('PICO_DRIVER_ROOT')
    if not root: pytest.skip('set PICO_DRIVER_ROOT to the paired PR329 checkout')
    spec = importlib.util.spec_from_file_location('paired_pico_feedback', Path(root)/'pico/4ultra/ext_vr/feedback.py')
    module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
    card, motion, outputs, feed = rig(tmp_path)
    feed(1); feed(2)
    module.validate_display(card.feedback())
    feed(3, True)
    module.validate_display(card.feedback())
    motion._visualization['valid_until_ns'] = card.clock()-1
    module.validate_display(card.feedback())
    assert card.feedback()['alignment']['state'] == 'unavailable'


def test_alignment_error_does_not_block_successful_ik(tmp_path):
    card, motion, outputs, feed = rig(tmp_path)
    feed(1); feed(2)
    good_render = motion.solver.render
    def moved(*args):
        result = good_render(*args)
        result['visualization']['alignment'].update(state='aligning', ready=False)
        result['visualization']['alignment']['left']['position_error_m'] = .1
        return result
    motion.solver.render = moved
    feed(3, True)
    assert card._enabled and len(outputs) == 1
    assert card.display_feedback()['alignment']['left']['position_error_m'] == .1


@pytest.mark.parametrize('event', ['release', 'tracking', 'button', 'gap'])
def test_batched_input_cannot_hide_pause_or_auto_resume(tmp_path, event):
    card, motion, outputs, feed = rig(tmp_path)
    feed(1); feed(2); last = feed(3, True)
    assert card._enabled
    before = len(outputs)
    lost = copy.deepcopy(last)
    lost.update(sequence=4, received_monotonic_ns=time.monotonic_ns())
    if event == 'release': lost['left']['grip'] = 0.
    if event == 'tracking': lost['left'].update(tracked=False, position=None, orientation_xyzw=None)
    if event == 'button':
        for side, name in [('left', 'x'), ('right', 'a')]:
            lost[side]['controls'] = {'buttons': {name: {'available': True, 'pressed': True}}}
    if event == 'gap':
        # Source timestamps show a real input outage, not just a skipped sequence.
        card._latest['received_monotonic_ns'] -= 301_000_000
    assert card.receive(lost)
    recovered = copy.deepcopy(last)
    recovered.update(sequence=5, received_monotonic_ns=time.monotonic_ns())
    assert card.receive(recovered)  # No control step between the two callbacks.
    card.step(); motion.process_latest(); card.step()
    assert not card._enabled and len(outputs) == before
    feed(6, True)
    assert not card._enabled and len(outputs) == before
    feed(7, False); feed(8, True)
    assert card._enabled and len(outputs) > before


def test_normal_source_coalescing_does_not_pause_active_control(tmp_path):
    card, motion, outputs, feed = rig(tmp_path)
    feed(1); feed(2); feed(3, True)
    # Observed producer sequence deltas include 2 and 3 during latest-value publish.
    for seq in (5, 8, 9, 11):
        feed(seq, True)
        assert card._enabled and card._state == 'active'
    assert len(outputs) == 5


def test_release_while_resuming_cannot_submit_the_prior_target(tmp_path):
    card, motion, outputs, feed = rig(tmp_path)
    feed(1); feed(2); last = feed(3, True)
    before = len(outputs)
    motion.gate.state = 'hold'
    original = card.executor.dispatch
    def resume(action, args):
        result = original(action, args)
        if action == 'resume':
            released = copy.deepcopy(last)
            released.update(sequence=5, received_monotonic_ns=time.monotonic_ns())
            released['left']['grip'] = 0.
            card.receive(released)
            recovered = copy.deepcopy(last)
            recovered.update(sequence=6, received_monotonic_ns=time.monotonic_ns())
            card.receive(recovered)
        return result
    card.executor.dispatch = resume
    feed(4, True)
    assert not card._enabled and len(outputs) == before
    feed(7, True)
    assert not card._enabled and len(outputs) == before


@pytest.mark.parametrize('reason', ['command_expired', 'arm_feedback_stale', 'motion_envelope_stale_start'])
def test_hold_reason_survives_preview_until_explicit_regrip(tmp_path, reason):
    card, motion, outputs, feed = rig(tmp_path)
    feed(1); feed(2); feed(3, True)
    before = len(outputs)
    motion._decision = {'state':'hold', 'reason':reason}
    feed(4, True)
    assert (card._state, card._reason) == ('hold', reason)
    assert card.feedback()['reason'] == reason
    feed(5, True); feed(6, True)
    assert (card._state, card._reason) == ('hold', reason)
    assert not card._enabled and len(outputs) == before
    feed(7, False); feed(8, True)
    assert card._enabled and card._state == 'active' and card._reason is None
    assert len(outputs) == before+1



def test_rejected_candidate_keeps_grip_enabled_for_next_valid_target(tmp_path):
    card, motion, outputs, feed = rig(tmp_path)
    feed(1); feed(2); feed(3, True)
    before = len(outputs)
    solve = motion.solver.solve_frame
    def stale_candidate(*args):
        result = solve(*args)
        motion.solver.last_envelope.update(lower=[1.]*10, upper=[2.]*10)
        return result
    motion.solver.solve_frame = stale_candidate
    feed(4, True)
    assert card._enabled and card._state == 'active' and card._reason is None
    assert len(outputs) == before and not motion.gate.holds
    assert motion._decision['state'] == 'rejected'
    motion.solver.solve_frame = solve
    feed(5, True)
    assert card._enabled and card._state == 'active'
    assert len(outputs) == before+1 and outputs[-1]['source_seq'] == 5
