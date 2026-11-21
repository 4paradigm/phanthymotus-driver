"""Standalone leg odometry: real kinematics and the shared odometry format."""
import json
import math
from types import SimpleNamespace

import pytest

from noetix.bumi.leg_odometry import DISTANCE_SLIP_SCALE, LegOdometry
from noetix.bumi.leg_odometry_source import LegOdometrySource


IDENTITY = [0.0, 0.0, 0.0, 1.0]
ZERO_GYRO = [0.0, 0.0, 0.0]


def joints(pitch=0.0):
    values = [0.0] * 21
    values[4] = values[14] = pitch
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
    # Both hip-pitch joints rotate the straight leg and sole centre. These
    # lever arms are metres from the vendor geometry, independent of an EKF.
    lever_x, lever_z, angle = 0.02911801, -0.51897, 0.01
    expected = [lever_x * (1.0 - math.cos(angle)) - lever_z * math.sin(angle),
                0.0,
                lever_z * (1.0 - math.cos(angle)) + lever_x * math.sin(angle)]
    assert packet['pose']['position'] == pytest.approx(expected, abs=1e-10)
    assert packet['twist'][:3] == pytest.approx([value / 0.02 for value in expected])
    assert LegOdometry().slip_scale == DISTANCE_SLIP_SCALE == 1.7


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


def test_invalid_stance_does_not_publish_a_measured_zero_speed():
    source = walking_source()
    assert not read(source, 0.6, stamp_ns=1_040_000_000, monotonic_s=1.04)
    packet = source.sample(1_040_000_000)
    assert packet['pose'] is None
    assert packet['twist'] == [None] * 6
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
