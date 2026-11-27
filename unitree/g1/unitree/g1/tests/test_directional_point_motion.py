"""Direct point execution with the production executor and synthetic feedback."""
import math
import pytest
from _point_test_helpers import (direct_setup, TARGET, FIELD_START, MOTOR_IDS,
                                 assert_released, delayed_bias_plant, optional_left)
import arm_stream

def test_current_seed_moves_during_weight_ramp_without_initial_hold(tmp_path, monkeypatch):
    backend, arm, sample, channel, stop, _, _, writes, _ = direct_setup(tmp_path, monkeypatch)
    start = [.01,.02,.03,.04,.05, .15,-.12,.1,.6,-.1]
    sample['q'][:] = start
    phases = []
    records = []
    install = arm.install_motion_envelope

    def envelope(record):
        records.append(record.copy())
        return install(record)

    monkeypatch.setattr(arm, 'install_motion_envelope', envelope)
    backend.execute(TARGET, .2, stop, lambda name,q: phases.append(name))
    assert writes[0][1] == start
    moved = next(row for row in writes if row[1][5:] != start[5:] and row[2] > 0.)
    assert 0 < moved[0]-writes[0][0] < 100_000_000 and moved[2] < 1.
    assert 'taking_over' not in phases and phases[0] == 'moving'
    assert 'holding' in phases and 'returning' in phases
    assert all(q[:5] == start[:5] for _,q,_ in writes)
    for before, after in zip(writes, writes[1:]):
        dt = (after[0]-before[0])/1e9
        assert max(abs(a-b) for a,b in zip(after[1], before[1])) <= .15*dt+1e-9
    assert records
    assert all(record['lower'] == [low for low,_ in arm.limits]
               and record['upper'] == [high for _,high in arm.limits] for record in records)
    assert_released(arm, channel)


def test_large_tracking_error_and_lag_are_diagnostic_not_an_abort(tmp_path, monkeypatch):
    backend, arm, sample, channel, stop, _, _, _, _ = direct_setup(tmp_path, monkeypatch)
    bias = [.025,-.025,.03,-.025,.03, .025,.025,-.03,-.025,.03]
    phase, reads, writes = delayed_bias_plant(arm, sample, channel, monkeypatch,
                                              lag_ns=20_000_000, bias=bias)
    result = backend.execute(TARGET, .2, stop, lambda name,q: phase.__setitem__(0,name))
    for name in ('moving', 'holding', 'returning'):
        assert any(p == name and max(abs(q-r) for q,r in zip(measured, reference)) > .025
                   for p,_,measured,reference,_ in reads)
    diagnostic = result['tracking_diagnostics']
    assert diagnostic['policy'] == 'diagnostic_only'
    assert len(diagnostic['max_abs_error_rad']) == 10
    assert min(diagnostic['max_abs_error_rad']) >= .02
    assert any(weight < 1. and q[5] > 0. for _,q,weight,p in writes if p != 'releasing')
    assert_released(arm, channel)


@pytest.mark.parametrize('index', range(10))
def test_every_measured_joint_hard_limit_still_fails(tmp_path, monkeypatch, index):
    backend, arm, sample, channel, stop, _, _, writes, _ = direct_setup(tmp_path, monkeypatch)
    original = arm._snapshot_provider
    injected = [False]

    def snapshot():
        state = dict(original())
        state['q'] = list(state['q'])
        # One invalid feedback interval ends when stop is requested; fresh
        # subsequent frames let the unchanged handback protocol finish.
        if injected[0] and not arm.release_requested:
            state['q'][index] = arm.limits[index][1]+.0001
        return state

    def observe(phase, q):
        if phase == 'moving' and writes:
            injected[0] = True

    monkeypatch.setattr(arm, '_snapshot_provider', snapshot)
    with pytest.raises(RuntimeError, match='motion_envelope_joint_limit') as error:
        backend.execute(TARGET, .2, stop, observe)
    evidence = error.value.envelope_failure
    assert evidence['predicate'] == 'measured_joint_limit' and evidence['motor_id'] == MOTOR_IDS[index]
    assert all(low <= v <= high for _,q,_ in writes for v,(low,high) in zip(q,arm.limits))
    assert_released(arm, channel)


def test_real_measured_body_collision_fails_even_with_valid_nominal_path(tmp_path, monkeypatch):
    backend, arm, sample, channel, stop, _, _, writes, _ = direct_setup(
        tmp_path, monkeypatch, isolate_geometry=False)
    sample['q'][5:] = FIELD_START
    angle = math.radians(20)
    direction = [math.cos(angle)**2, math.sin(angle)*math.cos(angle), math.sin(angle)]
    plan = backend.plan(direction, .2)
    with pytest.raises(ValueError, match='directional_measured_body_clearance'):
        backend.kinematics.check_measured_pose([0.]*5,
            **optional_left(backend.kinematics.check_measured_pose,sample['q'][:5]))
    original = arm._snapshot_provider
    injected = [False]

    def snapshot():
        state = dict(original())
        state['q'] = list(state['q'])
        if injected[0] and not arm.release_requested:
            state['q'][5:] = [0.]*5
        return state

    def observe(phase, q):
        if phase == 'moving' and writes:
            injected[0] = True

    monkeypatch.setattr(arm, '_snapshot_provider', snapshot)
    with pytest.raises(RuntimeError, match='directional_measured_body_clearance'):
        backend.execute(plan['target_q_rad'], .2, stop, observe, planned_start_q5=FIELD_START)
    assert_released(arm, channel)


def test_stop_during_direct_motion_preserves_handback(tmp_path, monkeypatch):
    backend, arm, _, channel, stop, _, _, writes, _ = direct_setup(tmp_path, monkeypatch)

    def observe(phase, q):
        if phase == 'moving' and writes:
            stop.set()

    with pytest.raises(RuntimeError, match='^cancelled$'):
        backend.execute(TARGET, .2, stop, observe)
    assert max((weight for _,_,weight in writes), default=0.) < 1.
    assert_released(arm, channel)


def test_stationary_actual_arm_cannot_report_target_success(tmp_path, monkeypatch):
    backend, arm, sample, channel, stop, _, _, _, _ = direct_setup(tmp_path, monkeypatch)
    phase = ['moving']
    original = channel.publish_point_position

    def publish(q, *, weight):
        result = original(q, weight=weight)
        if phase[0] != 'releasing':
            sample['q'][:] = [0.]*10
        return result

    monkeypatch.setattr(channel, 'publish_point_position', publish)
    with pytest.raises(RuntimeError, match='directional_target_not_reached') as error:
        backend.execute(TARGET, .2, stop, lambda name,q: phase.__setitem__(0,name))
    assert max(error.value.tracking_diagnostics['max_abs_error_rad']) > .1
    assert_released(arm, channel)


def test_measured_pose_cache_does_not_cache_freshness_or_changed_positions(tmp_path, monkeypatch):
    backend, arm, sample, channel, stop, _, _, writes, _ = direct_setup(tmp_path, monkeypatch)
    checked = []
    monkeypatch.setattr(backend.kinematics, 'check_measured_pose', lambda q, **kwargs: checked.append(tuple(q)) or .01)
    exercised = []

    def observe(phase, q):
        if phase != 'moving' or not writes or exercised:
            return
        guard = arm._directional_guard
        guard()
        count = len(checked)
        guard()
        assert len(checked) == count
        sample['q'][7] += .03
        guard()
        assert len(checked) == count+1
        clock = arm.clock
        now = clock()
        try:
            arm.clock = lambda: now+300_000_000
            with pytest.raises(ValueError, match='arm_feedback_stale'):
                guard()
        finally:
            arm.clock = clock
        exercised.append(True)
        stop.set()

    with pytest.raises(RuntimeError, match='^cancelled$'):
        backend.execute(TARGET, .2, stop, observe)
    assert exercised
    assert_released(arm, channel)


def test_command_must_stay_on_the_model_checked_line(tmp_path, monkeypatch):
    backend, arm, _, channel, stop, _, _, writes, _ = direct_setup(tmp_path, monkeypatch)
    injected = False

    def observe(phase, q):
        nonlocal injected
        if phase == 'moving' and writes and not injected:
            arm.last_q[0] += .001
            injected = True

    with pytest.raises(RuntimeError, match='directional_command_left_validated_path'):
        backend.execute(TARGET, .2, stop, observe)
    assert injected
    assert_released(arm, channel)


def test_new_terminal_sample_is_checked_before_return_can_complete(tmp_path, monkeypatch):
    backend, arm, _, channel, stop, _, _, _, _ = direct_setup(tmp_path, monkeypatch)
    original_read = backend._read
    phase = ['moving']
    settled = [None]
    injected = []
    rejected_pose = (.001, 0., 0., 0., 0.)

    def check_pose(q5, **kwargs):
        if tuple(q5) == rejected_pose:
            raise ValueError('directional_measured_body_clearance')
        return .01

    def read(*args, **kwargs):
        sample, q, dq = original_read(*args, **kwargs)
        if (phase[0] == 'returning' and not injected and max(map(abs,q[5:])) <= .04
                and max(map(abs,dq)) <= .02):
            if settled[0] is None:
                settled[0] = sample['arm_ns']
            elif sample['arm_ns']-settled[0] >= 200_000_000:
                # This consumed frame differs from the preceding guard's frame.
                # It is within the return tolerance, so only its pose check can
                # stop an otherwise-ready completion from being reported.
                q = list(q[:5])+list(rejected_pose)
                sample = {**sample, 'q': list(q)}
                injected.append(sample['arm_ns'])
        return sample, q, dq

    monkeypatch.setattr(backend.kinematics, 'check_measured_pose', check_pose)
    monkeypatch.setattr(backend, '_read', read)
    with pytest.raises(RuntimeError, match='directional_measured_body_clearance'):
        backend.execute(TARGET, .2, stop, lambda name,q: phase.__setitem__(0,name))
    assert injected
    assert_released(arm, channel)


def test_two_current_real_starts_use_nominal_path_without_fixed_start_gate(tmp_path, monkeypatch):
    backend, arm, sample, channel, stop, _, _, _, _ = direct_setup(
        tmp_path, monkeypatch, isolate_geometry=False)
    angle = math.radians(20)
    direction = [math.cos(angle)**2, math.sin(angle)*math.cos(angle), math.sin(angle)]
    for start in (FIELD_START, [.29,-.22,.02,.976,-.034]):
        sample['q'][5:] = start
        plan = backend.plan(direction, .2)
        assert plan['start_q_rad'] == start
        backend.check_ready(plan['target_q_rad'], .2, planned_start_q5=start)
        backend.execute(plan['target_q_rad'], .2, stop, lambda *_: None, planned_start_q5=start)
        assert_released(arm, channel)
    assert backend.config['verified_start_q_rad'] == [0.]*5

