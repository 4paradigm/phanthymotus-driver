"""Pure observations only; no ROS, sockets, publishers or robot requests."""
import copy
import math

import pytest

from deep_robotics.s10_pro.motion_feedback import MotionFeedback


class Clock:
    def __init__(self):
        self.now = 100.
    def __call__(self):
        return self.now
    def advance(self, value=.1):
        self.now += value


class Source:
    def __init__(self, clock):
        self.clock = clock
        self.sequence = 1
        self.received = clock()
        self.state = 0
        self.speed = (0., 0., 0.)
        self.error = None
        self.explicit_receive = True
        self.override = {}
    def put(self, state=None, speed=None):
        self.clock.advance()
        self.sequence += 1
        self.received = self.clock()
        if state is not None:
            self.state = state
        if speed is not None:
            self.speed = speed
    def __call__(self):
        raw = {'sequence': self.sequence, 'age_s': self.clock() - self.received,
            'fresh': self.clock() - self.received <= .5, 'error': self.error,
            'data': {'header': {'stamp': {'sec': -999}}, 'data': {
                'motion_state': {'state': self.state}, 'gait_state': {'gait': 0x3002},
                'vel_x': self.speed[0], 'vel_y': self.speed[1], 'vel_yaw': self.speed[2]}}}
        if self.explicit_receive:
            raw['received_monotonic'] = self.received
        raw.update(copy.deepcopy(self.override))
        return raw


@pytest.fixture
def observer():
    clock = Clock(); source = Source(clock)
    return MotionFeedback(source, clock=clock), source, clock


def fresh_two(observer, action, state=None, speed=None, rid='one'):
    feedback, source, _ = observer
    source.put(state, speed)
    first = feedback(action, rid)
    source.put(state, speed)
    return first, feedback(action, rid)


def test_no_request_context_cannot_confirm_even_observed_target(observer):
    feedback, source, _ = observer
    source.put(17)
    value = feedback('stand', 'one')
    assert value['fresh'] and not value['confirmed']
    assert value['reason'] == 'no_request_context'
    assert value['vendor_request_id'] is None


@pytest.mark.parametrize('action,state', [('stand_pose', 1), ('stand', 17), ('lie', 4)])
def test_posture_requires_two_new_distinct_matching_frames(observer, action, state):
    feedback, source, _ = observer
    feedback.begin(action, 'one')
    source.put(state)
    assert not feedback(action, 'one')['confirmed']
    for _ in range(8):
        assert not feedback(action, 'one')['confirmed']
    source.put(state)
    result = feedback(action, 'one')
    assert result['confirmed'] and result['matching_samples'] == 2
    assert result['correlation'] == 'local_request_marker_only'


@pytest.mark.parametrize('action,state', [('stand', 1), ('lie', 0), ('stand_pose', 17)])
def test_do_not_guess_alternative_posture_terminal_states(observer, action, state):
    feedback, _, _ = observer
    feedback.begin(action, 'one')
    assert not fresh_two(observer, action, state)[1]['confirmed']


def test_pre_request_cached_goal_is_not_evidence(observer):
    feedback, source, clock = observer
    source.state = 17
    clock.advance(.01)
    feedback.begin('stand', 'one')
    assert feedback('stand', 'one')['reason'] == 'not_after_request'
    assert not feedback('stand', 'one')['confirmed']


def test_request_id_is_local_and_mismatched_owner_never_confirms(observer):
    feedback, _, _ = observer
    feedback.begin('stand', 'one')
    fresh_two(observer, 'stand', 17)
    result = feedback('stand', 'unrelated')
    assert not result['confirmed'] and result['reason'] == 'request_context_mismatch'
    assert result['vendor_request_id'] is None


def test_begin_is_idempotent_during_repeated_velocity_refresh(observer):
    feedback, source, _ = observer
    first = feedback.begin('velocity', 'one', {'vx': .1}, generation=7)
    source.put(17, (.1, 0, 0))
    assert not feedback('velocity', 'one')['confirmed']
    repeat = feedback.begin('velocity', 'one', {'vx': .1}, generation=7)
    assert repeat == first
    source.put(17, (.1, 0, 0))
    assert feedback('velocity', 'one')['confirmed']


def test_changed_target_requires_new_context_instead_of_silent_retarget(observer):
    feedback, _, _ = observer
    feedback.begin('velocity', 'one', {'vx': .1})
    with pytest.raises(ValueError):
        feedback.begin('velocity', 'one', {'vx': .2})


def test_stand_dds_stages_require_new_observations_per_stage(observer):
    feedback, source, _ = observer
    stage1 = feedback.begin('stand_pose', 'one', generation=1)
    assert fresh_two(observer, 'stand_pose', 1)[1]['confirmed']
    stage2 = feedback.begin('stand', 'one', generation=1)
    assert stage2['generation'] > stage1['generation']
    assert not feedback('stand_pose', 'one')['confirmed']
    assert not feedback('stand', 'one')['confirmed']
    assert fresh_two(observer, 'stand', 17)[1]['confirmed']


def test_switch_to_stop_requires_new_zero_feedback(observer):
    feedback, source, _ = observer
    feedback.begin('velocity', 'one', {'vx': .1})
    assert fresh_two(observer, 'velocity', 17, (.1, 0, 0))[1]['confirmed']
    stop = feedback.begin('stopmotion', 'one')
    assert not feedback('stopmotion', 'one')['confirmed']
    assert not feedback('velocity', 'one')['confirmed']
    assert fresh_two(observer, 'stopmotion', 17, (0, 0, 0))[1]['confirmed']
    assert not feedback.invalidate('one', generation=stop['generation'] - 1)
    assert feedback('stopmotion', 'one')['confirmed']


def test_invalidation_does_not_let_old_cancellation_clear_new_request(observer):
    feedback, _, _ = observer
    old = feedback.begin('stand', 'old')
    current = feedback.begin('lie', 'new')
    assert not feedback.invalidate('old', generation=old['generation'])
    assert feedback('lie', 'new')['generation'] == current['generation']
    assert feedback.invalidate('new', generation=current['generation'])
    assert not feedback('lie', 'new')['confirmed']


def test_regressed_source_sequence_invalidates_current_context(observer):
    feedback, source, _ = observer
    feedback.begin('stand', 'one')
    assert fresh_two(observer, 'stand', 17)[1]['confirmed']
    source.sequence -= 1
    result = feedback('stand', 'one')
    assert not result['confirmed'] and result['reason'] == 'source_sequence_regressed'
    source.sequence += 10
    assert not feedback('stand', 'one')['confirmed']


def test_stability_requires_consecutive_matching_observed_frames(observer):
    feedback, source, _ = observer
    feedback.begin('stand', 'one')
    for state, expected in [(17, False), (0, False), (17, False), (17, True)]:
        source.put(state)
        assert feedback('stand', 'one')['confirmed'] is expected


def test_stale_error_and_missing_data_clear_stability(observer):
    feedback, source, clock = observer
    feedback.begin('stand', 'one')
    assert fresh_two(observer, 'stand', 17)[1]['confirmed']
    clock.advance(.6)
    assert not feedback('stand', 'one')['confirmed']
    source.put(17)
    assert not feedback('stand', 'one')['confirmed']
    source.error = 'reader_restart'
    assert feedback('stand', 'one')['reason'] == 'source_error'
    source.error = None
    source.put(17)
    assert not feedback('stand', 'one')['confirmed']


@pytest.mark.parametrize('key,value', [('age_s', float('nan')), ('age_s', -.1),
    ('age_s', True), ('age_s', .6), ('sequence', True), ('sequence', 0),
    ('received_monotonic', True), ('received_monotonic', float('inf')),
    ('received_monotonic', 200), ('received_monotonic', 1), ('data', None)])
def test_invalid_snapshot_never_confirms(observer, key, value):
    feedback, source, _ = observer
    feedback.begin('stand', 'one')
    source.override[key] = value
    source.put(17)
    result = feedback('stand', 'one')
    assert not result['fresh'] and not result['confirmed']


@pytest.mark.parametrize('state', [None, '17', True, float('nan')])
def test_invalid_state_fields_never_confirm(observer, state):
    feedback, source, _ = observer
    feedback.begin('stand', 'one')
    source.override['data'] = {'data': {'motion_state': {'state': state}, 'vel_x': 0, 'vel_y': 0, 'vel_yaw': 0}}
    source.put(17)
    assert not feedback('stand', 'one')['fresh']


@pytest.mark.parametrize('speed', [(float('inf'), 0, 0), (0, True, 0), (0, 0, '0')])
def test_invalid_velocity_fields_never_confirm(observer, speed):
    feedback, source, _ = observer
    feedback.begin('stand', 'one')
    source.put(17, speed)
    assert not feedback('stand', 'one')['confirmed']


def test_vendor_wall_clock_stamp_does_not_affect_receive_freshness(observer):
    feedback, source, _ = observer
    feedback.begin('stand', 'one')
    assert fresh_two(observer, 'stand', 17)[1]['confirmed']


def test_fallback_for_older_bridge_age_has_same_new_sequence_guard(observer):
    feedback, source, _ = observer
    source.explicit_receive = False
    feedback.begin('stand', 'one')
    assert not feedback('stand', 'one')['confirmed']
    first, second = fresh_two(observer, 'stand', 17)
    assert not first['confirmed'] and second['confirmed']
    assert second['received_monotonic'] <= source.clock()


def test_context_expires_and_clock_regression_invalidates(observer):
    feedback, source, clock = observer
    feedback.begin('stand', 'one')
    clock.advance(16)
    source.put(17)
    assert feedback('stand', 'one')['reason'] == 'context_expired'
    feedback.begin('stand', 'two')
    clock.now -= 1
    assert feedback('stand', 'two')['reason'] == 'context_expired'


def test_retired_owner_generation_cannot_replace_a_newer_stage(observer):
    feedback, _, _ = observer
    feedback.begin('stand_pose', 'one', generation=3)
    latest = feedback.begin('stand', 'one', generation=4)
    with pytest.raises(ValueError):
        feedback.begin('stand_pose', 'one', generation=3)
    assert feedback('stand', 'one')['generation'] == latest['generation']


def test_long_observation_gap_is_not_a_stable_two_frame_window(observer):
    feedback, source, clock = observer
    feedback.begin('stand', 'one')
    source.put(17)
    assert not feedback('stand', 'one')['confirmed']
    clock.advance(2)
    source.put(17)
    assert not feedback('stand', 'one')['confirmed']
    source.put(17)
    assert feedback('stand', 'one')['confirmed']


@pytest.mark.parametrize('target,actual', [({'vx': .1}, (.1, 0, 0)),
    ({'vy': -.1}, (0, -.1, 0)), ({'yaw_rate': .2}, (0, 0, .2)),
    ({'vx': .01}, (.008, 0, 0)), ({'vx': -.01}, (-.008, 0, 0))])
def test_velocity_confirmation_uses_documented_measured_physical_units(observer, target, actual):
    feedback, _, _ = observer
    feedback.begin('velocity', 'one', target)
    result = fresh_two(observer, 'velocity', 17, actual)[1]
    assert result['confirmed'] and not result['zero_target']


@pytest.mark.parametrize('target,actual', [({'vx': .01}, (0, 0, 0)),
    ({'vx': .1}, (-.1, 0, 0)), ({'vy': -.1}, (0, .1, 0)),
    ({'vx': .1}, (.03, 0, 0)), ({'yaw_rate': .2}, (0, 0, 0))])
def test_nonzero_target_requires_direction_and_meaningful_amplitude(observer, target, actual):
    feedback, _, _ = observer
    feedback.begin('velocity', 'one', target)
    assert not fresh_two(observer, 'velocity', 17, actual)[1]['confirmed']


def test_velocity_match_outside_rl_state_does_not_confirm(observer):
    feedback, _, _ = observer
    feedback.begin('velocity', 'one', {'vx': .1})
    assert not fresh_two(observer, 'velocity', 1, (.1, 0, 0))[1]['confirmed']


def test_all_zero_velocity_target_is_explicitly_stop_observation_not_proof_of_motion(observer):
    feedback, _, _ = observer
    feedback.begin('velocity', 'one', {})
    result = fresh_two(observer, 'velocity', 17, (0, 0, 0))[1]
    assert result['confirmed'] and result['zero_target']


def test_stop_accepts_near_zero_velocities_without_inventing_posture(observer):
    feedback, _, _ = observer
    feedback.begin('stopmotion', 'one')
    assert fresh_two(observer, 'stopmotion', 0, (.005, -.005, .01))[1]['confirmed']


def test_stop_does_not_confirm_while_any_axis_still_moves(observer):
    feedback, _, _ = observer
    feedback.begin('stopmotion', 'one')
    assert not fresh_two(observer, 'stopmotion', 17, (0, 0, .1))[1]['confirmed']


def test_preflight_reports_observation_without_inventing_safety_authority(observer):
    feedback, source, _ = observer
    source.put(17, (.1, 0, 0))
    result = feedback.preflight()
    assert result['fresh'] and result['motion_state'] == 17
    assert result['units']['vx'] == 'm/s'
    assert result['control_usage_mode'] is None and result['hard_emergency_stop'] is None
    assert result['control_ownership'] is None and not result['robot_deadman_verified']
    assert not result['execution_ready'] and result['read_only']


@pytest.mark.parametrize('options', [{'stable_samples': 1}, {'stable_samples': True},
    {'max_age_s': 4}, {'max_age_s': float('nan')}, {'context_ttl_s': 0},
    {'velocity_linear_tolerance': True}, {'stop_yaw_tolerance': 2}])
def test_observer_configuration_is_bounded(options):
    with pytest.raises(ValueError):
        MotionFeedback(lambda: {}, **options)


@pytest.mark.parametrize('action,target', [('unknown', {}), ('velocity', {'vx': True}),
    ('velocity', {'vy': .5}), ('velocity', {'yaw_rate': float('inf')}), ('velocity', [])])
def test_invalid_targets_do_not_create_context(observer, action, target):
    feedback, _, _ = observer
    with pytest.raises(ValueError):
        feedback.begin(action, 'one', target)
    assert feedback('stand', 'one')['reason'] == 'no_request_context'


def test_snapshot_exception_is_an_explicit_unfresh_observation():
    def broken():
        raise OSError('offline fixture error')
    feedback = MotionFeedback(broken)
    feedback.begin('stand', 'one')
    assert not feedback('stand', 'one')['confirmed']
    assert not feedback.preflight()['fresh']
