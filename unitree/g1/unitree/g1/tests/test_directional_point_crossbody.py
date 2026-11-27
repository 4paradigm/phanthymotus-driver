import _point_test_helpers
"""Cross-body point uses one real-shaped dual-arm sample; no robot transport."""
import math

import pytest

from _point_test_helpers import direct_setup, FIELD_START, TARGET
from _point_test_helpers import assert_released


LEFT = [.28758546710014343,.23094800114631653,-.018096180632710457,
        .9727954864501953,.09320131689310074]


def setup(tmp_path, monkeypatch, **kwargs):
    values = direct_setup(tmp_path,monkeypatch,angular_tolerance_deg=5.,
                          joint_speed_rad_s=1.,**kwargs)
    values[2]['q'][:] = LEFT+FIELD_START
    return values


def mirror(q):
    return [q[0],-q[1],-q[2],q[3],-q[4]]


def test_positive90_real_dual_arm_geometry_roundtrip_keeps_left_target(tmp_path,monkeypatch):
    backend,arm,sample,channel,stop,_,_,writes,_ = setup(tmp_path,monkeypatch,isolate_geometry=False)
    wanted = [0.,1.,0.]
    plan = backend.plan(wanted,.4)
    phases = []
    result = backend.execute(plan['target_q_rad'],.4,stop,
        lambda phase,q: phases.append(phase),planned_start_q5=plan['start_q_rad'],target_direction=wanted)
    arrival = result['arrival_diagnostics']
    assert arrival['hold_completed'] and arrival['angular_error_deg'] <= 5.
    assert arrival['hold_elapsed_s'] >= .4
    assert 'holding' in phases and 'returning' in phases
    assert all(q[:5] == LEFT for _,q,_ in writes)
    assert max(abs(a-b) for a,b in zip(sample['q'][5:],FIELD_START)) < .04
    assert any(max(box[4] for box in backend.kinematics._collision_bounds(q[5:])) > .1
               for _,q,w in writes if w == 1.)
    assert_released(arm,channel)


def test_plan_and_execution_recheck_changed_left_before_output(tmp_path,monkeypatch):
    backend,arm,sample,channel,stop,_,_,writes,_ = setup(tmp_path,monkeypatch,isolate_geometry=False)
    wanted = [0.,1.,0.]
    plan = backend.plan(wanted,.2)
    changed = mirror([(a+b)/2 for a,b in zip(FIELD_START,plan['target_q_rad'])])
    assert backend.kinematics.check_measured_pose(FIELD_START,left_q5=changed) >= .003
    assert backend.kinematics.check_measured_pose(plan['target_q_rad'],left_q5=changed) >= .003
    sample['q'][:5] = changed
    checked = []
    path = backend.kinematics.check_path

    def check(start,target,**kwargs):
        checked.append(list(kwargs['left_q5']))
        return path(start,target,**kwargs)

    monkeypatch.setattr(backend.kinematics,'check_path',check)
    with pytest.raises(RuntimeError,match='directional_body_clearance'):
        backend.execute(plan['target_q_rad'],.2,stop,lambda *_:None,
                        planned_start_q5=plan['start_q_rad'],target_direction=wanted)
    assert checked and all(left == changed for left in checked)
    assert not any(w > 0 for _,_,w in writes)
    assert_released(arm,channel)


def test_search_result_is_rechecked_against_new_left_feedback(tmp_path,monkeypatch):
    backend,arm,sample,_,_,_,_,writes,_ = setup(tmp_path,monkeypatch,isolate_geometry=False)
    solve = backend.kinematics.solve
    checked = []

    def solve_then_left_changes(direction,*,start_q5,left_q5):
        assert list(start_q5) == FIELD_START and list(left_q5) == LEFT
        target = solve(direction,start_q5=start_q5,left_q5=left_q5)
        changed = mirror([(a+b)/2 for a,b in zip(start_q5,target)])
        sample['q'][:5] = changed
        checked.append(changed)
        return target

    monkeypatch.setattr(backend.kinematics,'solve',solve_then_left_changes)
    with pytest.raises(ValueError,match='directional_body_clearance'):
        backend.plan([0.,1.,0.],.2)
    assert checked and not writes and arm.session_id is None


def test_actual_left_only_collision_is_detected_while_right_is_at_target(tmp_path,monkeypatch):
    backend,arm,sample,channel,stop,_,_,_,_ = setup(tmp_path,monkeypatch,isolate_geometry=False)
    wanted = [0.,1.,0.]
    plan = backend.plan(wanted,.8)
    snapshot = arm._snapshot_provider
    changed = []

    def provider():
        frame = dict(snapshot())
        frame['q'] = list(frame['q'])
        if changed and not arm.release_requested:
            frame['q'][:5] = changed[0]
        return frame

    def observe(phase,q):
        if phase == 'holding' and not changed:
            collision = mirror(q)
            with pytest.raises(ValueError,match='directional_measured_body_clearance'):
                backend.kinematics.check_measured_pose(q,left_q5=collision)
            changed.append(collision)

    monkeypatch.setattr(arm,'_snapshot_provider',provider)
    with pytest.raises(RuntimeError,match='directional_measured_body_clearance'):
        backend.execute(plan['target_q_rad'],.8,stop,observe,
                        planned_start_q5=plan['start_q_rad'],target_direction=wanted)
    assert changed
    assert_released(arm,channel)


def test_same_stamp_right_pose_cache_must_notice_left_only_change(tmp_path,monkeypatch):
    backend,arm,sample,channel,stop,_,_,writes,_ = setup(tmp_path,monkeypatch)
    checked = []
    exercised = []

    def check(right,*,left_q5):
        checked.append((tuple(right),tuple(left_q5)))
        if left_q5[1] > LEFT[1]+.01:
            raise ValueError('directional_measured_body_clearance')
        return .01

    def observe(phase,q):
        if phase == 'moving' and writes and not exercised:
            guard = arm._directional_guard
            guard()
            count = len(checked)
            guard()
            assert len(checked) == count
            stamp,right = sample['arm_ns'],list(sample['q'][5:])
            sample['q'][1] += .02
            with pytest.raises(ValueError,match='directional_measured_body_clearance'):
                guard()
            assert len(checked) == count+1
            assert sample['arm_ns'] == stamp and sample['q'][5:] == right
            assert checked[-1][0] == checked[-2][0]
            sample['q'][1] -= .02
            exercised.append(True)
            stop.set()

    monkeypatch.setattr(backend.kinematics,'check_measured_pose',check)
    with pytest.raises(RuntimeError,match='^cancelled$'):
        backend.execute(TARGET,.4,stop,observe)
    assert exercised
    assert_released(arm,channel)


def test_missing_left_feedback_is_rejected_before_claim(tmp_path,monkeypatch):
    backend,arm,sample,_,_,_,_,writes,_ = setup(tmp_path,monkeypatch,isolate_geometry=False)
    sample['q'] = list(FIELD_START)
    with pytest.raises(ValueError,match='feedback_q'):
        backend.plan([0.,1.,0.],.2)
    assert not writes and arm.session_id is None
