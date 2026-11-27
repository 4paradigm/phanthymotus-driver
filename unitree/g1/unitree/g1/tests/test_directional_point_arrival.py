import _point_test_helpers
"""Direction-based point arrival; synthetic plant, no robot transport."""
import math

import pytest

from _point_test_helpers import direct_setup, FIELD_START, TARGET, optional_left
from _point_test_helpers import assert_released


def direction(yaw, pitch):
    yaw, pitch = math.radians(yaw), math.radians(pitch)
    return [math.cos(pitch)*math.cos(yaw), math.cos(pitch)*math.sin(yaw), math.sin(pitch)]


def arrival_setup(tmp_path, monkeypatch, **kwargs):
    return direct_setup(tmp_path, monkeypatch, angular_tolerance_deg=5., **kwargs)


def test_recorded_direction_error_within_five_degrees_completes_continuous_hold(tmp_path, monkeypatch):
    backend, arm, sample, channel, stop, _, _, _, _ = arrival_setup(
        tmp_path, monkeypatch, isolate_geometry=False)
    sample['q'][5:] = FIELD_START
    requested = direction(20,20)
    plan = backend.plan(requested, .6)
    # Reproduce the logged direction using a constructed IK pose; these are
    # not the physical robot's unknown measured joint angles for that action.
    actual_q = backend.kinematics.solve(direction(18.5922,15.9083), start_q5=FIELD_START,
        **optional_left(backend.kinematics.solve,sample['q'][:5]))
    assert 4.30 < backend.kinematics.evaluate(actual_q, requested)['angular_error_deg'] < 4.31
    phase = ['moving']
    observations = []
    original = channel.publish_point_position

    def publish(q, *, weight):
        result = original(q, weight=weight)
        if phase[0] not in ('returning','releasing'):
            sample['q'][5:] = actual_q
        return result

    def observe(name, q):
        phase[0] = name
        observations.append((arm.clock(),name))

    monkeypatch.setattr(channel, 'publish_point_position', publish)
    result = backend.execute(plan['target_q_rad'], .6, stop, observe,
                             planned_start_q5=FIELD_START, target_direction=requested)
    arrival = result['arrival_diagnostics']
    assert arrival['target_direction'] == requested and arrival['tolerance_deg'] == 5.
    assert arrival['hold_completed'] and arrival['hold_elapsed_s'] >= .6
    assert arrival['max_abs_joint_error_rad'] > .04
    assert 4.30 < arrival['angular_error_deg'] < 4.31
    held = next(t for t,p in observations if p == 'holding')
    returned = next(t for t,p in observations if p == 'returning')
    assert returned-held >= 600_000_000
    assert_released(arm, channel)


def test_redundant_wrist_position_and_velocity_do_not_block_direction_hold(tmp_path, monkeypatch):
    backend, arm, sample, channel, stop, _, _, _, _ = arrival_setup(tmp_path, monkeypatch)
    original = channel.publish_point_position
    phase = ['moving']

    def publish(q, *, weight):
        result = original(q, weight=weight)
        if phase[0] not in ('returning','releasing'):
            t = arm.clock()/1e9
            sample['q'][9] += .2+.05*math.sin(2*t)
            sample['dq'][9] = .1*math.cos(2*t)
        else:
            sample['dq'][9] = 0.
        return result

    monkeypatch.setattr(channel, 'publish_point_position', publish)
    result = backend.execute(TARGET, .6, stop, lambda name,q: phase.__setitem__(0,name))
    arrival = result['arrival_diagnostics']
    assert arrival['hold_completed'] and abs(arrival['joint_error_rad'][4]) > .04
    assert arrival['angular_error_deg'] <= 5.
    assert_released(arm, channel)


@pytest.mark.parametrize('error_deg,completed', [(5., True), (5.001, False)])
def test_direction_threshold_is_inclusive_and_excess_times_out(tmp_path, monkeypatch, error_deg, completed):
    backend, arm, _, channel, stop, _, _, _, _ = arrival_setup(tmp_path, monkeypatch)
    monkeypatch.setattr(backend.kinematics, 'evaluate', lambda q,goal:
                        {'direction': list(goal), 'angular_error_deg': error_deg})
    phases = []
    if completed:
        result = backend.execute(TARGET, .4, stop, lambda name,q: phases.append(name))
        arrival = result['arrival_diagnostics']
        assert arrival['hold_completed'] and arrival['hold_elapsed_s'] >= .4
    else:
        with pytest.raises(RuntimeError, match='directional_target_not_reached') as failure:
            backend.execute(TARGET, .4, stop, lambda name,q: phases.append(name))
        arrival = failure.value.arrival_diagnostics
        assert not arrival['hold_completed'] and arrival['longest_hold_s'] == 0.
        assert 'holding' not in phases
    assert arrival['angular_error_deg'] == error_deg and arrival['tolerance_deg'] == 5.
    assert_released(arm, channel)


def test_out_of_tolerance_frame_resets_hold_without_immediate_abort(tmp_path, monkeypatch):
    backend, arm, _, channel, stop, _, _, _, _ = arrival_setup(tmp_path, monkeypatch)
    first_hold = []
    observations = []

    def evaluate(q, goal):
        age = (arm.clock()-first_hold[0])/1e9 if first_hold else -1
        return {'direction': list(goal), 'angular_error_deg': 5.2 if .3 <= age < .5 else 4.3}

    def observe(phase, q):
        observations.append((arm.clock(),phase))
        if phase == 'holding' and not first_hold:
            first_hold.append(arm.clock())

    monkeypatch.setattr(backend.kinematics, 'evaluate', evaluate)
    result = backend.execute(TARGET, .8, stop, observe)
    arrival = result['arrival_diagnostics']
    assert arrival['hold_completed'] and arrival['hold_elapsed_s'] >= .8
    assert arrival['hold_start_feedback_ns'] >= first_hold[0]+500_000_000
    assert any(first_hold[0]+300_000_000 <= t < first_hold[0]+500_000_000 and p == 'moving'
               for t,p in observations)
    returned = next(t for t,p in observations if p == 'returning')
    assert returned >= first_hold[0]+1_300_000_000
    assert_released(arm, channel)


def test_backend_uses_original_requested_direction_not_the_ik_pose_axis(tmp_path, monkeypatch):
    backend, arm, _, channel, stop, _, _, _, _ = arrival_setup(tmp_path, monkeypatch)
    requested = [.7,.5,.5]
    goals = []

    def evaluate(q, goal):
        goals.append(list(goal))
        return {'direction': list(goal), 'angular_error_deg': 4.3}

    monkeypatch.setattr(backend.kinematics, 'evaluate', evaluate)
    result = backend.execute(TARGET, .2, stop, lambda *_: None, target_direction=requested)
    assert goals and all(goal == requested for goal in goals)
    assert result['arrival_diagnostics']['target_direction'] == requested
    assert_released(arm, channel)


def test_full_authority_still_required_but_no_extra_direction_settling_wait(tmp_path, monkeypatch):
    backend, arm, _, channel, stop, _, _, _, _ = arrival_setup(tmp_path, monkeypatch)
    full_at = arm.clock()+2_000_000_000
    write = arm._write
    observations = []

    def limited(q, tau, weight):
        return write(q,tau,min(weight,.5) if arm.clock() < full_at else weight)

    monkeypatch.setattr(arm, '_write', limited)
    backend.execute([0.]*5, .4, stop,
                     lambda phase,q: observations.append((arm.clock(),phase,arm._weight)))
    held = next(t for t,p,w in observations if p == 'holding')
    returned = next(t for t,p,w in observations if p == 'returning')
    assert full_at <= held < full_at+50_000_000
    assert returned-held >= 400_000_000
    assert all(w >= 1 for _,p,w in observations if p == 'holding')
    assert_released(arm, channel)


def test_configured_arm_speed_is_faster_without_shortening_hold(tmp_path, monkeypatch):
    durations = []
    for speed in (.15, 1.0):
        local = tmp_path/str(speed)
        local.mkdir()
        backend, arm, _, channel, stop, _, _, writes, _ = arrival_setup(
            local, monkeypatch, joint_speed_rad_s=speed)
        start = arm.clock()
        observations = []
        backend.execute(TARGET, .6, stop,
                         lambda phase,q: observations.append((arm.clock(),phase)))
        held = next(t for t,p in observations if p == 'holding')
        returned = next(t for t,p in observations if p == 'returning')
        assert returned-held >= 600_000_000
        for previous,current in zip(writes,writes[1:]):
            assert max(abs(a-b) for a,b in zip(previous[1],current[1])) <= speed*(current[0]-previous[0])/1e9+1e-9
        durations.append(held-start)
        assert_released(arm, channel)
    assert durations[1] < durations[0]


def test_holding_waits_for_endpoint_output_and_its_feedback(tmp_path, monkeypatch):
    backend, arm, sample, channel, stop, _, _, _, _ = arrival_setup(tmp_path, monkeypatch)
    calls = []
    submit = backend._submit
    endpoint = []

    def record(lease, seq, reference, frame, bounds):
        result = submit(lease, seq, reference, frame, bounds)
        if reference[5:] == TARGET and not endpoint:
            endpoint.append((seq, arm.clock()))
        return result

    def observe(phase, q):
        if phase == 'holding':
            assert endpoint and arm.applied_seq >= endpoint[0][0]
            assert sample['arm_ns'] > endpoint[0][1]
            calls.append(arm.clock())

    monkeypatch.setattr(backend, '_submit', record)
    result = backend.execute(TARGET, .4, stop, observe)
    arrival = result['arrival_diagnostics']
    assert calls and arrival['endpoint_sequence'] >= endpoint[0][0]
    assert arrival['hold_start_feedback_ns'] >= arrival['endpoint_applied_ns']
    assert_released(arm, channel)


def test_delayed_first_full_weight_feedback_does_not_backdate_hold(tmp_path, monkeypatch):
    backend, arm, sample, channel, stop, _, _, _, _ = arrival_setup(tmp_path, monkeypatch)
    read = backend._read
    full_at = []

    def lagged(*args, **kwargs):
        frame,q,dq = read(*args, **kwargs)
        if arm._weight >= 1 and not full_at:
            full_at.append(arm.clock())
        if full_at and arm.clock()-full_at[0] < 160_000_000:
            frame = dict(frame, arm_ns=frame['arm_ns']-180_000_000)
        return frame,q,dq

    monkeypatch.setattr(backend, '_read', lagged)
    result = backend.execute([0.]*5, .4, stop, lambda *_: None)
    arrival = result['arrival_diagnostics']
    assert arrival['hold_start_feedback_ns'] >= arrival['endpoint_applied_ns']
    assert arrival['hold_start_feedback_ns'] >= full_at[0]+160_000_000
    assert arrival['feedback_ns'] >= full_at[0]+560_000_000
    assert_released(arm, channel)


def test_frozen_holding_sample_cannot_accumulate_wall_clock_hold(tmp_path, monkeypatch):
    backend, arm, sample, channel, stop, _, fresh, _, _ = arrival_setup(tmp_path, monkeypatch)
    began = []

    def observe(phase, q):
        if phase == 'holding' and not began:
            began.append(arm.clock())
            fresh[0] = False
        if phase == 'releasing':
            fresh[0] = True
            sample['arm_ns'] = arm.clock()

    with pytest.raises(RuntimeError) as failure:
        backend.execute(TARGET, 1., stop, observe)
    arrival = failure.value.arrival_diagnostics
    assert began and not arrival['hold_completed']
    assert arrival['hold_elapsed_s'] == arrival['longest_hold_s'] == 0.
    assert 'stale' in str(failure.value) or 'expired' in str(failure.value)
    assert_released(arm, channel)


def test_authority_loss_on_repeated_feedback_resets_continuous_hold(tmp_path, monkeypatch):
    backend, arm, _, channel, stop, _, _, _, _ = arrival_setup(tmp_path, monkeypatch)
    read = backend._read
    previous_stamp = [None]
    pending = [False]
    first_hold, lost_at = [], []

    def transient_loss(*args, **kwargs):
        frame,q,dq = read(*args, **kwargs)
        if pending[0] and not lost_at:
            lost_at.append(arm.clock())
            frame = dict(frame, arm_ns=previous_stamp[0])
            arm._weight = .5
        previous_stamp[0] = frame['arm_ns']
        return frame,q,dq

    def observe(phase, q):
        if phase == 'holding':
            if not first_hold:
                first_hold.append(arm.clock())
            elif arm.clock()-first_hold[0] >= 200_000_000:
                pending[0] = True

    monkeypatch.setattr(backend, '_read', transient_loss)
    result = backend.execute(TARGET, .6, stop, observe)
    arrival = result['arrival_diagnostics']
    assert lost_at and arrival['hold_completed']
    assert arrival['hold_start_feedback_ns'] > lost_at[0]
    assert arrival['feedback_ns']-arrival['hold_start_feedback_ns'] >= 600_000_000
    assert arrival['feedback_ns'] >= lost_at[0]+600_000_000
    assert_released(arm, channel)
