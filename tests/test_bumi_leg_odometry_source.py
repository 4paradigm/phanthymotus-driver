"""Standalone leg odometry: real kinematics and the shared odometry format."""
import json
import math
from types import SimpleNamespace

import pytest

from noetix.bumi.leg_odometry import DISTANCE_SLIP_SCALE, JOINT_INDEX, JOINT_ORDER, LegOdometry
from noetix.bumi.leg_odometry_source import LegOdometrySource


IDENTITY = [0.0, 0.0, 0.0, 1.0]
ZERO_GYRO = [0.0, 0.0, 0.0]
IDENTITY_ROTATION = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]


def joints(pitch=0.0):
    values = [0.0] * len(JOINT_ORDER)
    values[JOINT_INDEX['l_leg_pitch_joint']] = pitch
    values[JOINT_INDEX['r_leg_pitch_joint']] = pitch
    return values


def read(source, pitch=0.0, *, stamp_ns=1_000_000_000,
         monotonic_s=1.0, quaternion=IDENTITY, angular_velocity=ZERO_GYRO):
    return source.observe(joints(pitch), quaternion, angular_velocity,
                          stamp_ns=stamp_ns, monotonic_s=monotonic_s)


def walking_source(quaternion=IDENTITY):
    source = LegOdometrySource('robot')
    assert not read(source, quaternion=quaternion)
    assert read(source, 0.01, quaternion=quaternion,
                stamp_ns=1_020_000_000, monotonic_s=1.02)
    return source


def hip_pitch_displacement(angle):
    # Both hip-pitch joints rotate the straight leg and sole centre around
    # the hip. These lever arms are metres from the vendor geometry.
    lever_x, lever_z = 0.02911801, -0.51897
    return [lever_x * (1.0 - math.cos(angle)) - lever_z * math.sin(angle),
            0.0,
            lever_z * (1.0 - math.cos(angle)) + lever_x * math.sin(angle)]


def test_first_read_only_sets_baseline_and_identical_stationary_read_is_valid():
    source = LegOdometrySource('robot')
    assert not read(source)
    assert source.sample(1_000_000_000)['pose'] is None
    assert read(source, stamp_ns=1_020_000_000, monotonic_s=1.02)
    packet = source.sample(1_030_000_000)
    assert packet['schema'] == 'motus.odom/1'
    assert packet['frame'] == 'body'
    assert packet['units'] == {'linear': 'm/s', 'angular': 'rad/s', 'length': 'm'}
    assert packet['stamp_ms'] == 1020
    assert packet['pose']['position'] == pytest.approx([0.0, 0.0, 0.0])
    assert packet['pose']['quaternion'] == IDENTITY
    assert packet['twist'] == pytest.approx([0.0] * 6)
    assert packet['vendor']['status'] == 'tracking'
    json.dumps(packet, allow_nan=False)


def test_raw_leg_displacement_has_no_empirical_slip_multiplier():
    source = walking_source()
    packet = source.sample(1_020_000_000)
    expected = hip_pitch_displacement(0.01)
    assert packet['pose']['position'] == pytest.approx(expected, abs=1e-10)
    assert packet['twist'][:3] == pytest.approx([value / 0.02 for value in expected])
    assert LegOdometry().slip_scale == DISTANCE_SLIP_SCALE == 1.7


@pytest.mark.parametrize('interval_ms', [25, 50, 75])
def test_same_hip_trajectory_retains_displacement_across_sample_intervals(interval_ms):
    source = LegOdometrySource('robot')
    assert not read(source)
    previous_position = [0.0, 0.0, 0.0]
    duration_ms = 150
    for elapsed_ms in range(interval_ms, duration_ms + 1, interval_ms):
        angle = 0.6 * elapsed_ms / 1000
        stamp_ns = 1_000_000_000 + elapsed_ms * 1_000_000
        assert read(source, angle, stamp_ns=stamp_ns, monotonic_s=1.0 + elapsed_ms / 1000)
        packet = source.sample(stamp_ns)
        expected = hip_pitch_displacement(angle)
        assert packet['pose']['position'] == pytest.approx(expected, abs=1e-10)
        assert packet['twist'][:3] == pytest.approx([
            (current - previous) / (interval_ms / 1000)
            for current, previous in zip(expected, previous_position)
        ], abs=1e-10)
        previous_position = expected


@pytest.mark.parametrize(('quaternion', 'world_axes'), [
    ([0.0, 0.0, math.sqrt(0.5), math.sqrt(0.5)], (1, 0)),
    ([math.sqrt(0.5), 0.0, 0.0, math.sqrt(0.5)], (2, 1)),
])
def test_pose_is_world_frame_but_linear_twist_uses_full_body_rotation(quaternion, world_axes):
    reference = walking_source().sample(1_020_000_000)
    rotated = walking_source(quaternion).sample(1_020_000_000)
    x, y, z = reference['pose']['position']
    expected = [-y, x, z] if world_axes == (1, 0) else [x, -z, y]
    assert rotated['pose']['position'] == pytest.approx(expected, abs=1e-10)
    assert rotated['twist'][:3] == pytest.approx(reference['twist'][:3], abs=1e-10)
    assert rotated['pose']['quaternion'] == pytest.approx(quaternion)


def test_normalizes_orientation_and_accepts_sdk_motor_objects():
    source = LegOdometrySource('robot', child_frame_id='body')
    motors = [SimpleNamespace(pos=0.0) for _ in range(21)]
    source.observe(motors, [0.0, 0.0, 0.0, 2.0], [0.1, -0.2, 0.3],
                   stamp_ns=1_000_000_000, monotonic_s=1.0)
    assert source.observe(motors, [0.0, 0.0, 0.0, 2.0], [0.1, -0.2, 0.3],
                          stamp_ns=1_020_000_000, monotonic_s=1.02)
    packet = source.sample(1_020_000_000)
    assert packet['pose']['quaternion'] == IDENTITY
    assert packet['pose']['child_frame_id'] == 'body'
    assert packet['twist'][3:] == [0.1, -0.2, 0.3]


def test_republication_preserves_read_timestamp_and_stale_pose_is_unavailable():
    source = walking_source()
    first = source.sample(1_030_000_000)
    repeated = source.sample(1_200_000_000)
    assert repeated['stamp_ms'] == first['stamp_ms'] == 1020
    assert repeated['pose'] == first['pose']
    assert source.sample(1_270_000_000)['pose'] is not None
    stale = source.sample(1_270_000_001)
    assert stale['stamp_ms'] == 1020
    assert stale['pose'] is None
    assert stale['twist'] == [None] * 6
    assert stale['vendor']['status'] == 'lost'


def test_future_sample_is_not_reported_as_current_after_clock_rollback():
    source = walking_source()
    packet = source.sample(1_019_000_000)
    assert packet['pose'] is None
    assert packet['twist'] == [None] * 6


def test_covariance_is_positive_and_increases_without_visual_feedback():
    source = walking_source()
    first = source.sample(1_020_000_000)['pose']['covariance']
    assert len(first) == 36
    assert all(math.isfinite(value) for value in first)
    assert all(first[index * 7] > 0.0 for index in range(6))
    read(source, 0.02, stamp_ns=1_040_000_000, monotonic_s=1.04)
    second = source.sample(1_040_000_000)['pose']['covariance']
    assert all(second[index * 7] > first[index * 7] for index in range(6))


@pytest.mark.parametrize(('stamp_ns', 'monotonic_s'), [
    (1_200_000_000, 1.2),
    (1_040_000_000, 1.01),
    (1_020_000_000, 1.04),
    (900_000_000, 1.04),
])
def test_time_discontinuity_starts_new_frame_and_does_not_integrate_across_gap(stamp_ns, monotonic_s):
    source = walking_source()
    previous = source.sample(1_020_000_000)['pose']['frame_id']
    assert not read(source, 0.4, stamp_ns=stamp_ns, monotonic_s=monotonic_s)
    assert source.sample(stamp_ns)['pose'] is None
    assert read(source, 0.4, stamp_ns=stamp_ns + 20_000_000, monotonic_s=monotonic_s + 0.02)
    resumed = source.sample(stamp_ns + 20_000_000)
    assert resumed['pose']['frame_id'] != previous
    assert resumed['pose']['position'] == pytest.approx([0.0, 0.0, 0.0])
    assert resumed['twist'] == pytest.approx([0.0] * 6)


def test_reset_makes_a_new_frame_without_reusing_previous_feet():
    source = walking_source()
    previous = source.sample(1_020_000_000)['pose']['frame_id']
    source.reset()
    assert source.sample(2_000_000_000)['pose'] is None
    assert not read(source, 0.4, stamp_ns=2_000_000_000, monotonic_s=2.0)
    assert read(source, 0.4, stamp_ns=2_020_000_000, monotonic_s=2.02)
    resumed = source.sample(2_020_000_000)
    assert resumed['pose']['frame_id'] != previous
    assert resumed['pose']['position'] == pytest.approx([0.0, 0.0, 0.0])


@pytest.mark.parametrize(('pitch', 'elapsed_ms'), [(0.6, 20), (0.04, 10), (0.19, 75)])
def test_invalid_stance_does_not_publish_a_measured_zero_speed(pitch, elapsed_ms):
    source = walking_source()
    # The 10 ms case moves only 1.56 cm, but implies more than 1 m/s.
    stamp_ns = 1_020_000_000 + elapsed_ms * 1_000_000
    assert not read(source, pitch, stamp_ns=stamp_ns, monotonic_s=1.02 + elapsed_ms / 1000)
    packet = source.sample(stamp_ns)
    assert packet['pose'] is None
    assert packet['twist'] == [None, None, None, *ZERO_GYRO]
    assert packet['vendor']['status'] != 'tracking'


@pytest.mark.parametrize('overrides', [
    {'joints': [0.0] * 4},
    {'joints': [float('nan')] * 21},
    {'quaternion': [0.0, 0.0, 1.0]},
    {'quaternion': [0.0] * 4},
    {'quaternion': [0.0, 0.0, float('inf'), 1.0]},
    {'angular_velocity': [0.0, 0.0]},
    {'angular_velocity': [0.0, float('nan'), 0.0]},
    {'stamp_ns': True},
    {'stamp_ns': 0},
    {'monotonic_s': float('nan')},
])
def test_bad_sensor_sample_invalidates_without_refreshing_timestamp(overrides):
    source = walking_source()
    kwargs = dict(joints=joints(0.02), quaternion=IDENTITY,
                  angular_velocity=ZERO_GYRO, stamp_ns=1_040_000_000, monotonic_s=1.04)
    kwargs.update(overrides)
    with pytest.raises(ValueError):
        source.observe(**kwargs)
    packet = source.sample(1_040_000_000)
    assert packet['stamp_ms'] == 1020
    assert packet['pose'] is None
    assert packet['twist'] == [None] * 6
    assert packet['vendor']['status'] == 'lost'


def test_sdk_failure_invalidates_immediately_and_recovery_rebuilds_baseline():
    source = walking_source()
    previous = source.sample(1_020_000_000)['pose']['frame_id']
    source.invalidate('SDK read failed')
    packet = source.sample(1_025_000_000)
    assert packet['pose'] is None
    assert packet['stamp_ms'] == 1020
    assert packet['vendor']['reason'] == 'SDK read failed'
    assert not read(source, 0.2, stamp_ns=1_040_000_000, monotonic_s=1.04)
    assert read(source, 0.2, stamp_ns=1_060_000_000, monotonic_s=1.06)
    resumed = source.sample(1_060_000_000)
    assert resumed['pose']['frame_id'] != previous
    assert resumed['pose']['position'] == pytest.approx([0.0, 0.0, 0.0])


def test_published_packet_cannot_mutate_the_integrator_state():
    source = walking_source()
    first = source.sample(1_020_000_000)
    expected = first['pose']['position'][:]
    first['pose']['position'][0] = 99.0
    first['pose']['quaternion'][0] = 99.0
    first['twist'][0] = 99.0
    fresh = source.sample(1_020_000_000)
    assert fresh['pose']['position'] == expected
    assert fresh['pose']['quaternion'] == IDENTITY
    assert fresh['twist'][0] != 99.0


@pytest.mark.parametrize('roll', [0.0, math.pi / 2])
def test_lower_world_foot_stays_in_support_through_a_direction_reversal(roll):
    source = LegOdometrySource('robot')
    c, s = math.cos(roll), math.sin(roll)
    rotation = [[1, 0, 0], [0, c, -s], [0, s, c]]
    for step in range(16):
        progress = step if step <= 10 else 20 - step
        world = [[.03 - .005 * progress, .1, -.48], [.03 + .002 * progress, -.1, -.43]]
        # Inverse roll creates the identical physical trajectory in tilted body axes.
        feet = [[x, c*y + s*z, -s*y + c*z] for x, y, z in world]
        source.leg.observe_feet(feet, rotation, step * .025)
        assert source.leg.position == pytest.approx([.005 * progress, 0, 0])


@pytest.mark.parametrize('interval_ms', [10, 25, 50, 75])
def test_equal_height_disagreement_selects_one_support_at_every_sample_rate(interval_ms):
    source = LegOdometrySource('robot')
    for elapsed_ms in range(0, 301, interval_ms):
        t = elapsed_ms / 1000
        feet = [[.03 - .2*t, .1, -.48], [.03 - .5*t, -.1, -.48]]
        source.leg.observe_feet(feet, IDENTITY_ROTATION, t)
    assert source.leg.position == pytest.approx([.06, 0, 0])
    assert not source.leg.double_support


def test_equal_length_steps_separated_by_45_degrees_do_not_indicate_double_support():
    leg = LegOdometrySource('robot').leg
    leg.observe_feet([[0, .1, -.48], [0, -.1, -.48]], IDENTITY_ROTATION, 0)
    d = .005 / math.sqrt(2)
    leg.observe_feet([[-.005, .1, -.48], [-d, -.1-d, -.48]], IDENTITY_ROTATION, .025)
    assert not leg.double_support


def test_support_persistence_resolves_small_speed_ambiguities():
    leg = LegOdometrySource('robot').leg
    for step, (left_x, right_x) in enumerate([(0., 0.), (-.005, .01), (0., .0051)]):
        leg.observe_feet([[left_x, .1, -.48], [right_x, -.1, -.48]],
                         IDENTITY_ROTATION, step * .025)
    # Both candidates reversed. Left remains planted despite 0.1 mm more travel.
    assert leg.position == pytest.approx([0, 0, 0])


def test_rejected_spike_withdraws_translation_then_recovers_clean_endpoint_and_covariance():
    clean, noisy = LegOdometrySource('robot'), LegOdometrySource('robot')
    original_frame = noisy.frame_id
    proc_before = None
    for i, (good, bad) in enumerate(zip([0, .01, .02, .03, .04], [0, .01, .60, .03, .04])):
        stamp = 1_000_000_000 + 25_000_000 * i
        times = dict(stamp_ns=stamp, monotonic_s=1 + i * .025, angular_velocity=[.1, -.2, .3])
        read(clean, good, **times)
        read(noisy, bad, **times)
        packet = noisy.sample(stamp)
        if i == 1:
            proc_before = packet['pose'].get('process_covariance')
        if i == 2:
            assert packet['pose'] is None
            assert packet['twist'] == [None, None, None, .1, -.2, .3]
            assert noisy.sample(stamp + 100_000_000)['stamp_ms'] == 1050
            assert noisy.sample(stamp + 250_000_001)['twist'] == [None] * 6
        if i >= 3:
            ref = clean.sample(stamp)['pose']
            pose = packet['pose']
            assert pose['frame_id'] == original_frame
            assert pose['position'] == pytest.approx(ref['position'])
            assert len(proc_before) == len(pose['process_covariance']) == 36
            for j in range(0, 36, 7):
                assert pose['process_covariance'][j] > proc_before[j] > 0
                assert pose['covariance'][j] > pose['process_covariance'][j]
            assert pose['process_covariance'][0] > ref['process_covariance'][0]


def test_failed_endpoint_followed_by_long_gap_starts_a_new_origin():
    source = walking_source()
    original_frame = source.frame_id
    for tick in range(1, 5):
        assert not read(source, .6, stamp_ns=1_020_000_000 + tick * 20_000_000,
                        monotonic_s=1.02 + tick * .02)
    # 110 ms from the accepted endpoint, but only 30 ms from the last rejected read.
    assert not read(source, .04, stamp_ns=1_130_000_000, monotonic_s=1.13)
    assert read(source, .04, stamp_ns=1_150_000_000, monotonic_s=1.15)
    recovered = source.sample(1_150_000_000)['pose']
    assert recovered['frame_id'] != original_frame
    assert recovered['position'] == pytest.approx([0, 0, 0])


def test_process_noise_increases_with_disagreement_at_the_same_elapsed_time_and_distance():
    def pose_after(right_pitch):
        source = LegOdometrySource('robot')
        read(source)
        q = joints(.01)
        q[JOINT_INDEX['r_leg_pitch_joint']] = right_pitch
        source.observe(q, IDENTITY, ZERO_GYRO, stamp_ns=1_025_000_000, monotonic_s=1.025)
        return source.sample(1_025_000_000)['pose']

    agree, disagree = pose_after(.01), pose_after(.03)
    assert disagree['position'] == pytest.approx(agree['position'])
    for pose in (agree, disagree):
        process = pose['process_covariance']
        assert len(process) == 36 and all(math.isfinite(x) for x in process)
        assert all(process[i] == 0 for i in range(36) if i % 7)
        assert all(process[i] > 0 for i in range(0, 36, 7))
        assert pose['covariance'][0] - process[0] == pytest.approx(.03**2)
    assert disagree['process_covariance'][0] > agree['process_covariance'][0]


def test_nearly_stationary_orthogonal_steps_do_not_claim_double_support():
    leg = LegOdometrySource('robot').leg
    leg.observe_feet([[0, .1, -.48], [0, -.1, -.48]], IDENTITY_ROTATION, 0)
    leg.observe_feet([[-.0002, .1, -.48], [0, -.1002, -.48]], IDENTITY_ROTATION, .025)
    assert not leg.double_support


@pytest.mark.parametrize('endpoint_ms', [1025, 570])
@pytest.mark.parametrize(('extra_stamp_ns', 'extra_monotonic_ns', 'accepted'), [
    (0, 0, True),
    (1, 0, False),
    (0, 1, False),
])
def test_recovery_100_ms_boundary_for_both_clocks(endpoint_ms, extra_stamp_ns,
                                                   extra_monotonic_ns, accepted):
    source = LegOdometrySource('robot')
    for offset_ms, angle in [(-25, 0), (0, .01), (25, .60)]:
        ms = endpoint_ms + offset_ms
        read(source, angle, stamp_ns=ms * 1_000_000, monotonic_s=ms / 1000)
    frame = source.frame_id
    deadline_ns = (endpoint_ms + 100) * 1_000_000
    result = read(source, .04, stamp_ns=deadline_ns + extra_stamp_ns,
                  monotonic_s=(deadline_ns + extra_monotonic_ns) / 1_000_000_000)
    assert result is accepted
    assert (source.frame_id == frame) is accepted


def test_elevated_only_velocity_plausible_foot_cannot_certify_a_step():
    leg = LegOdometrySource('robot').leg
    leg.observe_feet([[0, .1, -.48], [0, -.1, -.43]], IDENTITY_ROTATION, 0)
    leg.observe_feet([[-.10, .1, -.48], [.002, -.1, -.43]], IDENTITY_ROTATION, .025)
    assert leg.stance_samples == 0
    leg.observe_feet([[-.01, .1, -.48], [.004, -.1, -.43]], IDENTITY_ROTATION, .050)
    assert leg.position == pytest.approx([.01, 0, 0])


def test_recovery_with_opposite_feet_lifted_at_each_endpoint_cannot_bridge_translation():
    leg = LegOdometrySource('robot').leg
    leg.observe_feet([[0, .1, -.48], [0, -.1, -.44]], IDENTITY_ROTATION, 0)
    leg.observe_feet([[.20, .1, -.48], [.20, -.1, -.44]], IDENTITY_ROTATION, .025)
    leg.observe_feet([[.003, .1, -.44], [-.008, -.1, -.48]], IDENTITY_ROTATION, .075)
    assert leg.stance_samples == 0


def test_clear_stance_height_reduces_ambiguity_when_velocity_evidence_is_equal():
    def ambiguity(right_z):
        leg = LegOdometrySource('robot').leg
        leg.observe_feet([[0, .1, -.48], [0, -.1, right_z]], IDENTITY_ROTATION, 0)
        leg.observe_feet([[-.005, .1, -.48], [.005, -.1, right_z]], IDENTITY_ROTATION, .025)
        assert not leg.double_support
        return leg.support_ambiguity

    assert ambiguity(-.43) < ambiguity(-.48)


@pytest.mark.parametrize(('stamp_ms', 'monotonic_s', 'fault'), [
    (1000, 1., False),   # Cold start
    (1531, 1.531, False),  # 506 ms since the previous read
    (1050, 1.050, True),
    (1015, 1.050, False),  # ROS clock reversal
    (1050, 1.015, False),  # Monotonic clock reversal
])
def test_fresh_gyro_survives_acquisition_baseline(stamp_ms, monotonic_s, fault):
    source = LegOdometrySource('robot')
    if stamp_ms != 1000:
        read(source)
        read(source, stamp_ns=1_025_000_000, monotonic_s=1.025)
    if fault:
        source.invalidate('SDK fault')
        assert source.sample(1_030_000_000)['twist'] == [None] * 6
    stamp_ns = stamp_ms * 1_000_000
    assert not read(source, .4, stamp_ns=stamp_ns, monotonic_s=monotonic_s,
                    angular_velocity=[.1, -.2, .75])
    packet = source.sample(stamp_ns)
    assert packet['twist'] == [None, None, None, .1, -.2, .75]
    assert packet['pose'] is None
    assert packet['stamp_ms'] == stamp_ms
    assert source.sample(stamp_ns - 1)['twist'] == [None] * 6
    assert source.sample(stamp_ns + 250_000_001)['twist'] == [None] * 6


def test_sparse_506_ms_reads_report_rates_without_accumulating_distance():
    source = LegOdometrySource('robot')
    for i, rate in enumerate((.25, -.75, .5, 0.)):
        ms = 1000 + 506 * i
        assert not read(source, pitch=i * .01, stamp_ns=ms * 1_000_000,
                        monotonic_s=ms / 1000, angular_velocity=[0., 0., rate])
        sample = source.sample(ms * 1_000_000)
        assert sample['pose'] is None
        assert sample['twist'] == [None, None, None, 0., 0., rate]
        assert sample['stamp_ms'] == ms
