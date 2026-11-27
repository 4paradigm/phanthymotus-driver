import _point_test_helpers
"""Mode-gated point schema and synthetic executor integration."""
import json
import threading
from types import SimpleNamespace

import pytest

from directional_pointing import DirectionalPointing
from directional_stationary import DirectionalStationaryGuard
from _point_test_helpers import setup
from _point_test_helpers import Backend


def test_walking_schema_reserves_both_arms_and_preserves_point_stop_contract():
    backend = SimpleNamespace(config={}, kinematics=SimpleNamespace(
        solver_mode='bounded_ik', angular_tolerance_deg=3.,
        yaw_limits_deg=(-20., 20.), pitch_limits_deg=(-5., 25.)))
    card = DirectionalPointing(backend)
    tool = card.get_tool()
    schema = tool['inputSchema']
    assert schema['x-resource'] == ['arm_l', 'arm_r']
    assert '常规运控模式' in tool['description']
    assert 'torso_link' in tool['description']
    assert '左臂保持' in tool['description']
    assert '静止' not in tool['description']
    assert schema['properties']['action']['enum'] == ['point', 'stop']
    assert schema['x-action-params']['point']['params'] == ['yaw_deg', 'pitch_deg', 'hold_s']
    assert schema['x-action-params']['stop']['required'] == ['action_id']
    assert schema['x-completion'] == {'actions': ['point'], 'timeout': 120}
    assert schema['x-hooks']['on_interrupt_all']['params'] == {'_interrupt_active': True}


def test_walking_card_repeated_calls_have_independent_ids_starts_and_receipts(tmp_path):
    backend = Backend(str(tmp_path))
    notices = []
    done = threading.Event()

    def notify(ident, state, data, **kwargs):
        notices.append((ident, state, data))
        done.set()

    card = DirectionalPointing(backend, notify)
    try:
        for _ in range(2):
            done.clear()
            ticket = card.dispatch('point', {'yaw_deg': 20, 'pitch_deg': 20, 'hold_s': 1})
            assert ticket.get('state') == 'accepted', ticket
            assert done.wait(2)
            ident, state, result = notices[-1]
            assert (ident, state) == (ticket['action_id'], 'completed')
            assert result['left_max_error_rad'] == .0004
            assert len(json.dumps(result)) < 600
            stored = json.loads((tmp_path/(ident+'.json')).read_text())
            assert stored['result']['left_hold']['target_q_rad'] == [.1] * 5
        assert backend.calls == [[0.] * 5, [.1] * 5]
        assert notices[0][0] != notices[1][0]
    finally:
        card.stop()


def mode_plant(tmp_path, monkeypatch, submode):
    backend, arm, sample, channel, stop, _, _, writes, _ = setup(tmp_path, monkeypatch)
    guard = DirectionalStationaryGuard(None, clock=lambda: arm.clock()/1e9,
                                      wall_clock=lambda: 1000.+arm.clock()/1e9)
    mode = [500, submode]
    last = [None]
    base_frames = []

    def mode_sample():
        now = arm.clock()
        if now != last[0]:
            guard.update_fsm({'fsm_id': mode[0], 'fsm_mode': mode[1], 'ts': 1000.+now/1e9})
            frame = {'velocity': [.4, 0., 0.], 'yaw_speed': .2,
                     'position': [now/1e9*.4, 0., 0.], 'yaw': now/1e9*.2}
            guard.update_odom(frame)
            base_frames.append(frame)
            last[0] = now
        return guard.check()

    backend.stationary = mode_sample
    return backend, arm, channel, stop, writes, mode, base_frames


@pytest.mark.parametrize('submode', [0, 1])
def test_mode_allows_synthetic_point_with_walking_base_feedback(tmp_path, monkeypatch, submode):
    backend, arm, channel, stop, writes, _, base_frames = mode_plant(tmp_path, monkeypatch, submode)
    phases = []
    backend.execute([.1, 0., 0., .1, 0.], .2, stop, lambda phase, q: phases.append(phase))
    assert 'moving' in phases and 'holding' in phases
    assert len(base_frames) > 100 and base_frames[-1]['position'][0] > base_frames[0]['position'][0]
    assert any(weight > 0. and q[5] > .05 for _, q, weight in writes)
    assert channel.closed and arm.status()['release']['authority_released']


def test_operational_mode_loss_during_point_releases_without_new_trajectory(tmp_path, monkeypatch):
    backend, arm, channel, stop, writes, mode, _ = mode_plant(tmp_path, monkeypatch, 0)
    transition = []

    def observe(phase, q):
        if phase == 'moving' and not transition:
            mode[:] = [0, 0]
            transition.append(len(writes))

    with pytest.raises(RuntimeError, match='fsm_not_operational'):
        backend.execute([.1, 0., 0., .1, 0.], .2, stop, observe)
    assert transition
    last_q = writes[transition[0]-1][1]
    assert all(q == last_q for _, q, _ in writes[transition[0]:])
    assert channel.closed and arm.status()['release']['authority_released']


@pytest.mark.parametrize('phase', ['planning', 'claim', 'first_hold'])
def test_moving_arms_can_enter_point_with_gradual_authority(tmp_path, monkeypatch, phase):
    backend, arm, sample, channel, stop, _, _, writes, _ = setup(tmp_path, monkeypatch)
    original_publish = channel.publish_point_position

    def publish(q, *, weight):
        # Synthetic motor feedback settles when the test controller has authority.
        # This tests sequencing, not the physical settling dynamics of walking.
        if weight > 0.:
            sample['dq'][:] = [0.] * 10
        return original_publish(q, weight=weight)

    monkeypatch.setattr(channel, 'publish_point_position', publish)
    if phase == 'planning':
        sample['dq'][1] = .1
        began = arm.clock()
        monkeypatch.setattr(backend.kinematics, 'solve',
                            lambda direction, *, start_q5, left_q5=None: list(start_q5))
        backend.plan([1., 0., 0.], .2)
        assert arm.clock() == began and not writes
    elif phase == 'claim':
        original = arm._claim

        def claim(**kwargs):
            lease = original(**kwargs)
            sample['dq'][1] = .1
            return lease

        monkeypatch.setattr(arm, '_claim', claim)
    else:
        original = backend.motion.solver.joint_envelope

        def proof(*args):
            bounds = original(*args)
            sample['dq'][1] = .1
            return bounds

        monkeypatch.setattr(backend.motion.solver, 'joint_envelope', proof)
    backend.execute([.1, 0., 0., .1, 0.], .2, stop, lambda *_: None)
    first_positive = next((q, weight) for _, q, weight in writes if weight > 0.)
    assert first_positive[0] == [0.] * 10
    assert 0. < first_positive[1] < .01
    assert max(weight for _, _, weight in writes) == 1.
    assert channel.closed and arm.status()['release']['authority_released']


def test_moving_arm_plan_still_requires_fresh_joint_feedback(tmp_path, monkeypatch):
    backend, arm, sample, channel, _, _, _, writes, _ = setup(tmp_path, monkeypatch)
    sample['dq'][1] = .1
    sample['arm_ns'] = arm.clock()-201_000_000
    with pytest.raises(ValueError, match='arm_feedback_stale'):
        backend.plan([1., 0., 0.], .2)
    assert arm.session_id is None and not writes and not channel.closed


def test_stop_before_moving_arm_claim_has_no_output(tmp_path, monkeypatch):
    backend, arm, sample, channel, stop, _, _, writes, _ = setup(tmp_path, monkeypatch)
    sample['dq'][1] = .1
    stop.set()
    with pytest.raises(RuntimeError, match='^cancelled$'):
        backend.execute([.1, 0., 0., .1, 0.], .2, stop, lambda *_: None)
    assert arm.session_id is None and not writes and not channel.closed
