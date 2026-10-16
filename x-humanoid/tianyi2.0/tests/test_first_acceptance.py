"""Explicit, expiring onsite preparation; all feedback/writes are synthetic."""
from types import SimpleNamespace

import pytest
import sys

from teleop_executor import TeleopExecutor, accepted
from test_motion_stream import rig

@pytest.fixture(autouse=True)
def fixed_joint_maps(monkeypatch):
    monkeypatch.setitem(sys.modules,'device',SimpleNamespace(
        _HEAD_JOINTS={1:'head'},_WAIST_JOINTS={31:'waist'},_LEG_JOINTS={51:'leg'}))


def setup_trial():
    gate, state, writes, advance, _ = rig()
    gate.session_id = None
    gate.state = 'idle'
    item = TeleopExecutor({'live_enabled': True, 'first_acceptance_enabled': True},
                         'test', None, None, None, [])
    item.profile = {'hands_enabled': False, 'first_acceptance': {
        **{k: True for k in ('model_verified', 'workspace_verified', 'pico_verified',
                            'external_control_excluded')},
        'power_feedback_verified': False,
        'operator': 'test operator', 'date': '2026-09-21', 'evidence_sha256': 'a'*64}}
    item.gate = gate
    state['fixed_motor_positions_rad']={'1':-.36,'31':.1,'51':-.2}
    item._streams={part:(gate.clock(),{mid:(state['fixed_motor_positions_rad'][str(mid)],0.,0)})
                   for part,mid in [('head',1),('waist',31),('leg',51)]}
    gate.acceptance_check = item._execution_accepted
    item.start = lambda: None
    item.foreign_publishers = lambda: []
    item.arm = SimpleNamespace(_pos_publisher=None)
    item.arm.start = lambda: setattr(item.arm, '_pos_publisher', object())
    # Any accidental hand access in the arms-only claim path must fail.
    item.hand = None
    return item, state, writes, advance


def test_preparation_is_explicit_and_never_promotes_normal_acceptance():
    item, _, writes, _ = setup_trial()
    assert not item._execution_accepted()
    assert item.dispatch('claim', {})['code'] == 'live_acceptance_missing'
    result = item.dispatch('prepare_first_acceptance', {})
    assert result['first_acceptance_prepared']
    assert not result['ownership_held'] and not result['publisher_present']
    assert not writes and not accepted(item.profile)
    assert item.dispatch('claim', {})['state'] == 'ready'
    assert item.arm._pos_publisher is not None and item.hand is None


@pytest.mark.parametrize('flag', ['model_verified', 'workspace_verified', 'pico_verified',
                                 'external_control_excluded'])
def test_first_acceptance_does_not_bypass_other_evidence(flag):
    item, _, writes, _ = setup_trial()
    item.profile['first_acceptance'][flag] = False
    assert item.dispatch('prepare_first_acceptance', {})['code'] == 'first_acceptance_prerequisites_missing'
    assert item.gate.first_acceptance_deadline_ns is None and not writes


@pytest.mark.parametrize('condition', ['disabled', 'shadow', 'hands', 'busy', 'competitor', 'stale', 'estop', 'power_off', 'fault'])
def test_preparation_keeps_runtime_gates(condition):
    item, state, writes, advance = setup_trial()
    if condition == 'disabled': item.cfg['first_acceptance_enabled'] = False
    elif condition == 'shadow': item.gate.live_enabled = False
    elif condition == 'hands': item.profile['hands_enabled'] = True
    elif condition == 'busy': item.legacy_busy = lambda: True
    elif condition == 'competitor': item.foreign_publishers = lambda: ['other']
    elif condition == 'estop': state['estop'] = True
    elif condition == 'power_off': state['power_on'] = False
    elif condition == 'fault': state['fault'] = True
    else: advance(101, fresh=False)
    assert item.dispatch('prepare_first_acceptance', {})['state'] == 'error'
    assert item.gate.first_acceptance_deadline_ns is None and not writes


def test_deadline_holds_and_cannot_be_extended_while_owned():
    item, state, writes, advance = setup_trial()
    item.dispatch('prepare_first_acceptance', {})
    deadline = item.gate.first_acceptance_deadline_ns
    item.dispatch('claim', {})
    assert item.dispatch('prepare_first_acceptance', {})['code'] == 'first_acceptance_requires_idle'
    item.gate.hold('operator_pause')
    advance(20); item.tick()
    advance(20); item.tick()
    item.gate.resume()
    assert item.gate.first_acceptance_deadline_ns == deadline
    advance(60_000); item.tick()
    assert item.gate.state == 'hold' and item.gate.reason == 'first_acceptance_expired'
    assert writes[-1] == (state['q'], None)
    advance(20); item.tick()
    assert item.gate.status()['hold_confirmed']
    with pytest.raises(ValueError): item.gate.resume()
    assert not item._execution_accepted()


def test_restart_forgets_preparation():
    item, _, _, _ = setup_trial()
    item.dispatch('prepare_first_acceptance', {})
    replacement, _, _, _ = setup_trial()
    assert not replacement._execution_accepted()
    assert replacement.gate.first_acceptance_deadline_ns is None
    assert replacement._fixed_baseline is None

def test_preparation_captures_robot_pose_not_old_profile_or_headset():
    item,state,writes,_=setup_trial()
    item.profile['fixed_motor_positions_rad']={'1':0.,'31':0.,'51':0.}
    item.profile['headset_pose']=[99.,99.,99.]
    state['fixed_body']=False  # Old session reference may differ.
    assert item.dispatch('prepare_first_acceptance',{})['first_acceptance_prepared']
    assert item._fixed_baseline==state['fixed_motor_positions_rad']
    assert item.profile['fixed_motor_positions_rad']['1']==0. and not writes
    reference=dict(item._fixed_baseline)
    item.gate.session_id='owned'
    state['fixed_motor_positions_rad']['1']=.3
    assert item.dispatch('prepare_first_acceptance',{})['code']=='first_acceptance_requires_idle'
    assert item._fixed_baseline==reference

@pytest.mark.parametrize('bad',['moving','missing','nonfinite'])
def test_bad_fixed_feedback_never_becomes_reference(bad):
    item,state,writes,_=setup_trial()
    if bad=='moving':item._streams['head'][1][1]=(-.36,.1,0)
    elif bad=='missing':del state['fixed_motor_positions_rad']['1']
    else:state['fixed_motor_positions_rad']['1']=float('nan')
    assert item.dispatch('prepare_first_acceptance',{})['state']=='error'
    assert item._fixed_baseline is None and item.gate.first_acceptance_deadline_ns is None and not writes


def test_operator_session_is_explicit_continuous_and_release_revokes():
    item,state,writes,advance=setup_trial()
    assert item.dispatch('prepare_operator_session',{})['code']=='operator_session_disabled'
    item.cfg['operator_session_enabled']=True
    assert item.dispatch('prepare_operator_session',{})['operator_session_prepared']
    assert not writes and item.gate.first_acceptance_deadline_ns is None
    assert not accepted(item.profile)
    lease=item.dispatch('claim',{})
    assert lease['state']=='ready'
    advance(120_000)
    assert item._execution_accepted()
    args={k:lease[k] for k in ('session_id','secret')}
    assert not item.dispatch('release',args).get('error')
    assert not item._execution_accepted()


def test_operator_prepare_failure_does_not_keep_old_permission():
    item,state,_,_=setup_trial();item.cfg['operator_session_enabled']=True
    assert item.dispatch('prepare_operator_session',{})['operator_session_prepared']
    item._capture_fixed_baseline=lambda:(_ for _ in ()).throw(ValueError('fixed_ns_stale'))
    assert item.dispatch('prepare_operator_session',{})['code']=='fixed_ns_stale'
    assert not item._execution_accepted()
