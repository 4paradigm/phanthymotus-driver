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
