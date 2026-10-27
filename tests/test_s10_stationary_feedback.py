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
