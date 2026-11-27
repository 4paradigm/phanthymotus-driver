"""Point lease isolation and startup/release causality using a synthetic plant."""
import math
import pytest
from _point_test_helpers import setup, direct_setup, TARGET, assert_released
from common.motion.protocol import envelope

def inject_before_first_reference(backend, monkeypatch, inject):
    """The first unstarted check verifies the claim seed, the next gates the first reference."""
    original = backend._require_lease
    unstarted_checks = 0

    def check(lease, **kwargs):
        nonlocal unstarted_checks
        if kwargs.get('unstarted'):
            unstarted_checks += 1
            if unstarted_checks == 2:
                assert backend.arm.session_id == lease['session_id']
                assert backend.arm.seq == backend.arm.applied_seq == -1
                inject()
        return original(lease, **kwargs)

    monkeypatch.setattr(backend, '_require_lease', check)


@pytest.mark.parametrize('guard_fault', [False, True])
def test_same_lease_hold_before_first_sample_preserves_real_cause(tmp_path, monkeypatch, guard_fault):
    backend, arm, sample, _, stop, _, _, writes, _ = setup(tmp_path, monkeypatch)
    phases = []

    def changed_state():
        if guard_fault:
            arm._directional_fault = 'directional_guard:directional_not_stationary:base_moving'
            arm._release({}, action99=False, cancel_pending=True)
        else:
            arm.hold('arm_feedback_stale', recoverable=True)
        sample['dq'][0] = .025

    inject_before_first_reference(backend, monkeypatch, changed_state)
    expected = 'directional_guard:directional_not_stationary:base_moving' if guard_fault else 'arm_feedback_stale'
    with pytest.raises(RuntimeError, match='release_timeout') as caught:
        backend.execute([.1, 0., 0., .1, 0.], 1., stop, lambda phase, q: phases.append(phase))
    error = caught.value
    assert 'cause=' + expected in str(error) and 'ownership_changed' not in str(error)
    assert phases == ['releasing']
    assert error.startup_failure['phase'] == 'first_reference'
    assert error.startup_failure['same_session'] is True
    assert error.startup_failure['sequence'] == error.startup_failure['applied_sequence'] == -1
    assert error.release_failure['operation_match'] is True
    assert error.release_failure['timed_out_state'] == 'confirming_handback'
    assert error.release_failure['max_dq_motor_id'] == 15
    assert error.release_failure['max_abs_dq_rad_s'] == .025
    assert all(weight == 0. for _, _, weight in writes)


def test_first_reference_is_queued_atomically_and_numerical_work_is_outside_lock(tmp_path, monkeypatch):
    backend, arm, _, _, stop, _, _, _, _ = setup(tmp_path, monkeypatch)
    original_envelope = backend.motion.solver.joint_envelope
    original_submit = backend._submit
    submitted = []

    def solve_envelope(*args):
        assert not arm.lock._is_owned()
        assert arm.seq == -1 and arm.latest is None
        return original_envelope(*args)

    def submit(lease, seq, reference, sample, bounds):
        assert arm.lock._is_owned()
        if not submitted:
            assert seq == 1 and arm.seq == -1
            assert reference == arm.last_q
            assert arm._directional_guard()['stationary'] is True
        result = original_submit(lease, seq, reference, sample, bounds)
        submitted.append(seq)
        return result

    def observe(phase, q):
        if phase == 'moving':
            assert submitted and arm.latest is not None and arm.seq >= 1

    monkeypatch.setattr(backend.motion.solver, 'joint_envelope', solve_envelope)
    monkeypatch.setattr(backend, '_submit', submit)
    backend.execute([.1, 0., 0., .1, 0.], .1, stop, observe)
    assert submitted[0] == 1


def test_other_owner_operation_is_never_attached_or_stopped(tmp_path, monkeypatch):
    backend, arm, _, _, stop, _, _, writes, _ = setup(tmp_path, monkeypatch)

    def another_owner():
        arm.session_id = 'another-owner'
        foreign = {'operation_id': 'foreign', 'session_id': arm.session_id,
                   'state': 'unknown', 'reason': 'foreign_reason'}
        arm._operation = foreign
        arm._operations['foreign'] = foreign

    inject_before_first_reference(backend, monkeypatch, another_owner)
    with pytest.raises(RuntimeError, match='ownership_changed') as caught:
        backend.execute([.1, 0., 0., .1, 0.], .1, stop, lambda *_: None)
    assert caught.value.startup_failure['same_session'] is False
    assert caught.value.release_failure['operation_match'] is False
    assert caught.value.release_failure['state'] == 'no_matching_operation'
    assert 'foreign_reason' not in str(caught.value.release_failure)
    assert arm.session_id == 'another-owner' and not writes


def test_point_then_ordinary_lease_restores_writer_filter_and_default_speed(tmp_path,monkeypatch):
    backend,arm,sample,channel,stop,_,_,_,advance = direct_setup(tmp_path,monkeypatch,joint_speed_rad_s=.15)
    backend.execute(TARGET,.1,stop,lambda *_:None)
    assert_released(arm,channel)
    assert arm._directional_guard is None and arm._lease_velocity is None
    assert arm.replay_deadline_ns is None
    ordinary_writes = []
    def ordinary(q,*,weight):
        ordinary_writes.append((list(q),weight))
        sample['q'][:] = q
        return True
    monkeypatch.setattr(channel,'publish_servo_position',ordinary)
    monkeypatch.setattr(channel,'publish_point_position',lambda *a,**k:pytest.fail('point writer leaked into ordinary lease'))
    assert 'error' not in arm.dispatch('prepare_operator_session',{})
    lease = arm.dispatch('claim',{})
    assert lease['state'] == 'ready', lease
    start = list(sample['q'])
    now = arm.clock()
    target = [q+delta for q,delta in zip(start,[.2,.04]+[0.]*8)]
    command = envelope(lease,seq=1,source_seq=1,mapping_epoch=0,generated_ns=now,
                       valid_until_ns=now+150_000_000,mode='joint_position',dof=10,
                       values=target,**arm.versions)
    proof = {key:command[key] for key in ('boot_id','session_id','seq','source_seq','mapping_epoch',
                                       'model_version','calibration_version','frame','valid_until_ns')}
    proof.update(schema='motus.motion.envelope/1',measured_ns=sample['arm_ns'],
                 lower=[lo for lo,hi in arm.limits],upper=[hi for lo,hi in arm.limits])
    arm.install_motion_envelope(proof)
    assert arm.accept_control(command)
    advance(.004)
    first = ordinary_writes[0][0]
    # Existing ordinary servo behavior: 1 rad/s cap per joint and 120 ms
    # smoothing. A point lease uses one shared scale instead of these ratios.
    assert first[0]-start[0] == pytest.approx(.004)
    assert first[1]-start[1] == pytest.approx(.04*(1-math.exp(-.004/.12)))
    assert 'directional_fault' not in arm.status()
    result = arm.dispatch('stop',lease)
    for _ in range(500):
        if result.get('state') == 'completed': break
        advance(.02)
        result = arm.status()['release']
    assert result['state'] == 'completed' and result['physical_confirmed']
    assert 'session_id' not in result  # Ordinary stop keeps its existing schema.
