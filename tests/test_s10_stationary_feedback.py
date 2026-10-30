"""Stationary tolerance uses speed magnitude and distinct recent samples."""
import time

import pytest

from deep_robotics.s10_pro.stationary_feedback import stationary_velocity, StationaryWindow
from deep_robotics.s10_pro.canvas_posture import odometry_observation
from test_s10_prepare_authorization import policy, runtime


def observed(sequence=1, received=1., vx=-.011, vy=-.039, yaw=.012, state=17):
    return {'fresh': True, 'sequence': sequence, 'received_monotonic': received,
        'motion_state': state, 'velocity': {'vx': vx, 'vy': vy, 'yaw_rate': yaw}}


def test_measured_bias_is_within_new_tolerance():
    assert stationary_velocity(observed())


@pytest.mark.parametrize('vx,vy,yaw', [(.05,0.,0.),(0.,-.05,0.),(.04,.04,0.),
    (0.,0.,.06),(.12,0.,0.),(float('nan'),0.,0.),(True,0.,0.)])
def test_motion_or_invalid_values_still_fail(vx, vy, yaw):
    assert not stationary_velocity(observed(vx=vx,vy=vy,yaw=yaw))


def test_three_distinct_frames_spanning_point_three_seconds_are_required():
    window = StationaryWindow(clock=lambda: 1.3)
    assert not window.observe(observed(1,1.), (17,))['ready']
    assert not window.observe(observed(2,1.15), (17,))['ready']
    assert window.observe(observed(3,1.3), (17,))['ready']


def test_cached_frames_cannot_supply_stable_window():
    window = StationaryWindow(clock=lambda: 1.3)
    for _ in range(10):
        value = window.observe(observed(1,1.2), (17,))
        assert not value['ready'] and value['matching_samples'] == 1


def test_spike_needs_a_new_stable_window():
    now = [1.]
    window = StationaryWindow(clock=lambda: now[0])
    for seq, stamp in enumerate([1.,1.15,1.3], 1):
        now[0] = stamp
        result = window.observe(observed(seq,stamp), (17,))
    assert result['ready']
    now[0] = 1.4
    assert not window.observe(observed(4,1.4,vx=.10), (17,))['ready']
    now[0] = 1.5
    assert not window.observe(observed(5,1.5), (17,))['ready']
    now[0] = 1.65
    assert not window.observe(observed(6,1.65), (17,))['ready']
    now[0] = 1.8
    assert window.observe(observed(7,1.8), (17,))['ready']


def test_reset_or_stale_feedback_retires_old_evidence():
    window = StationaryWindow(clock=lambda: 1.3)
    for seq, stamp in enumerate([1.,1.15,1.3],1):
        result = window.observe(observed(seq,stamp), (17,))
    assert result['ready']
    window.reset()
    assert not window.observe(observed(4,1.3), (17,))['ready']
    assert not window.observe({**observed(), 'fresh':False}, (17,))['ready']


def test_prepare_accepts_small_reported_velocity_without_sending(runtime):
    plugin, _, grant, bridge, transports = runtime
    bridge.vx = .04
    reply = plugin.dispatch('prepare', {})
    assert reply['prepared'] and grant.authorized_now()
    assert reply['stationary_check']['linear_tolerance_mps'] == .05
    assert all(not transport.states and not transport.velocities for transport in transports)


def test_implausible_odometry_still_cannot_be_used_for_distance(runtime):
    _, backend, _, bridge, _ = runtime
    raw = bridge.snapshot('odometry')
    raw['data']['twist']['twist']['linear']['x'] = 4.
    observed_odom = odometry_observation(raw,time.monotonic())
    assert not observed_odom['fresh'] and observed_odom['reason'] == 'odometry_velocity_inconsistent'


def odom(sequence, received, **updates):
    return {'fresh': True, 'sequence': sequence, 'received_monotonic': received,
            'source_stamp_s': received,
            'x': 0., 'y': 0., 'yaw': 0., 'vx': .003, 'vy': -.007,
            'yaw_rate': .002, **updates}


def bias_window(mutate=None):
    now = [1.]
    window = StationaryWindow(clock=lambda: now[0])
    result = None
    for i in range(7):
        now[0] = 1. + i * .1
        motion = observed(i + 1, now[0], vx=-.157, vy=-.043)
        position = odom(i + 1, now[0], x=i * .0001)
        if mutate:
            mutate(i, motion, position)
        result = window.observe(motion, (17,), position)
    return window, result, now


def test_measured_motion_bias_needs_independent_advancing_stationary_odometry():
    window, result, now = bias_window()
    assert result['ready']
    assert result['confirmation_basis'] == 'motion_and_odometry'
    assert result['reported_velocity']['vx'] == -.157
    assert result['linear_tolerance_mps'] == .05
    assert result['odometry_corroboration']['position_span_m'] < .001
    window.reset()
    assert not window.observe(observed(10, now[0], vx=-.157), (17,), odom(10, now[0]))['ready']


@pytest.mark.parametrize('case', ['absent', 'stale', 'frozen_odom', 'frozen_motion', 'fast_odom',
                                  'moving_position', 'out_and_back', 'turning', 'bias_too_large',
                                  'unstable_bias', 'invalid_odom', 'invalid_motion', 'wrong_state', 'frozen_stamp'])
def test_bias_exception_never_accepts_missing_evidence_or_real_motion(case):
    def mutate(i, motion, position):
        if case == 'absent': position.clear()
        if case == 'stale': position['fresh'] = False
        if case == 'frozen_odom': position.update(sequence=1, received_monotonic=1.)
        if case == 'frozen_motion': motion.update(sequence=1, received_monotonic=1.)
        if case == 'fast_odom': position['vx'] = -.157
        if case == 'moving_position': position['x'] = i * .015
        if case == 'out_and_back': position['x'] = .02 if i == 3 else 0.
        if case == 'turning': position['yaw'] = i * .01
        if case == 'bias_too_large': motion['velocity']['vx'] = -.21
        if case == 'unstable_bias': motion['velocity']['vx'] = -.08 if i % 2 else -.16
        if case == 'invalid_odom': position['vx'] = float('nan')
        if case == 'invalid_motion': motion['velocity'] = [0., 0., 0.]
        if case == 'wrong_state': motion['motion_state'] = 4
        if case == 'frozen_stamp': position['source_stamp_s'] = 1.
    _, result, _ = bias_window(mutate)
    assert not result['ready']


def test_both_sources_need_at_least_half_a_second_and_separate_frames():
    now = [1.]
    window = StationaryWindow(clock=lambda: now[0])
    for i in range(6):
        now[0] = 1. + i * .1
        result = window.observe(observed(i + 1, now[0], vx=-.157), (17,), odom(i + 1, now[0]))
        assert result['ready'] == (i == 5)
    now[0] += .4
    stale = window.observe(observed(6, 1.5, vx=-.157), (17,), odom(6, 1.5))
    assert not stale['ready']


def test_prepare_can_corroborate_bias_without_posture_or_velocity_commands(runtime, monkeypatch):
    plugin, backend, grant, bridge, transports = runtime
    original = bridge.snapshot
    def snapshot(source):
        value = original(source)
        if source == 'motion_info':
            value['data']['data'].update(vel_x=-.157, vel_y=-.043)
        if source == 'odometry':
            stamp = time.time_ns()
            value['data']['header'] = {'stamp': {'sec': stamp // 1_000_000_000,
                                               'nanosec': stamp % 1_000_000_000}}
        return value
    monkeypatch.setattr(bridge, 'snapshot', snapshot)
    reply = plugin.dispatch('prepare', {})
    assert reply['prepared'] and grant.authorized_now()
    assert reply['stationary_check']['confirmation_basis'] == 'motion_and_odometry'
    assert all(not t.states and not t.velocity_commands for t in transports)
