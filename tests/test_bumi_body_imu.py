"""Body sensor provenance, fixed mounting and honest read-time semantics."""
import json
import importlib.util
from pathlib import Path
from types import SimpleNamespace as NS

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
MOUNT = dict(calibration_id='level-forward', fixed_mount=True, assumed=False,
             camera_to_body=dict(translation=[0., 0., 0.],
                                 quaternion=[-.5, .5, -.5, .5]))


def source(mount=MOUNT):
    from noetix.bumi.body_imu import BodyImuSource
    return BodyImuSource('robot', mount, lambda: ('robot_rgb_optical', 'capture/global_time'))


def reading(stamp=1_000_000_000, **changes):
    data = dict(stamp_ns=stamp, timestamp_source='read_completion',
                quaternion=[0., 0., 0., 1.], angular_vel=[1., 2., 3.],
                linear_acc=[0., 0., 9.81])
    data.update(changes)
    return data


def test_body_packet_keeps_raw_axes_and_read_time_and_inverts_fixed_camera_mount():
    imu = source()
    packet = imu.accept(reading(), 1_010_000_000)
    assert packet['source_id'] == 'noetix/robot/body_imu'
    assert packet['gyro_frame_id'] == packet['accel_frame_id'] == 'robot_body_imu'
    assert packet['camera_frame_id'] == 'robot_rgb_optical'
    assert packet['clock_id'] == 'capture/global_time'
    assert packet['stamp_ns'] == packet['accel_stamp_ns'] == 1_000_000_000
    assert packet['stamp_source'] == 'read_completion'
    assert packet['clock_mapping'] == dict(method='host_wall_read_completion',
                                           source_clock='ros_wall', offset_ns=0, verified=False)
    assert packet['rotation_source'] == 'configured_fixed_mount'
    assert packet['calibration_id'] == 'level-forward'
    assert packet['angular_velocity'] == [1., 2., 3.]
    assert packet['linear_acceleration'] == [0., 0., 9.81]
    assert packet['camera_from_gyro'] == packet['camera_from_accel'] == [
        [0., -1., 0.], [0., 0., -1.], [1., 0., 0.]]
    assert 'camera_from_accel_translation' not in packet
    assert imu.status(1_010_000_000)['state'] == 'ready'
    assert imu.status(1_010_000_000)['fusion']['state'] == 'unavailable'
    assert imu.status(1_500_000_001)['state'] == 'unavailable'


@pytest.mark.parametrize('change', [dict(timestamp_source='measurement'),
    dict(stamp_ns=True), dict(stamp_ns=400_000_000), dict(stamp_ns=1_100_000_000),
    dict(angular_vel=[float('nan'), 0., 0.]), dict(quaternion=[0., 0., 0., 0.])])
def test_invalid_body_reading_does_not_become_a_fresh_measurement(change):
    imu = source()
    with pytest.raises(ValueError):
        imu.accept(reading(**change), 1_010_000_000)
    assert imu.status(1_010_000_000)['state'] == 'unavailable'


@pytest.mark.parametrize('mount', [None, dict(MOUNT, fixed_mount=False)])
def test_missing_or_nonfixed_mount_never_publishes_identity(mount):
    imu = source(mount)
    with pytest.raises(ValueError):
        imu.accept(reading(), 1_010_000_000)
    assert imu.status(1_010_000_000)['state'] == 'unavailable'


def test_assumed_mount_keeps_numeric_body_rotation_without_measured_translation():
    imu = source(dict(MOUNT, assumed=True))
    packet = imu.accept(reading(), 1_010_000_000)
    assert packet['camera_from_gyro'] == packet['camera_from_accel'] == [
        [0., -1., 0.], [0., 0., -1.], [1., 0., 0.]]
    assert packet['rotation_source'] == 'configured_fixed_mount'
    assert 'camera_from_accel_translation' not in packet
    assert imu.status(1_010_000_000)['state'] == 'ready'


def test_cached_sdk_values_do_not_refresh_freshness_and_restart_has_new_epoch():
    imu = source()
    first = imu.accept(reading(), 1_010_000_000)
    assert imu.accept(reading(1_050_000_000), 1_060_000_000) is None
    assert imu.status(1_510_000_000)['state'] == 'unavailable'
    with pytest.raises(ValueError):
        imu.accept(reading(900_000_000, angular_vel=[0., 0., 0.]), 1_010_000_000)
    imu.reset()
    assert imu.accept(reading(1_600_000_000), 1_610_000_000) is None
    assert imu.status(1_610_000_000)['state'] == 'unavailable'
    second = imu.accept(reading(1_650_000_000, angular_vel=[1., 2., 3.1]), 1_660_000_000)
    assert second['epoch'] != first['epoch']


def test_real_generic_consumer_uses_body_axes_once_and_discloses_time_basis():
    path = ROOT.parent/'pr371-current-review/perception/plugins/rgbd_imu.py'
    if not path.exists():
        pytest.skip('independent Perception checkout is not present')
    spec = importlib.util.spec_from_file_location('generic_body_imu_consumer', path)
    consumer = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(consumer)
    packet = json.loads(json.dumps(source().accept(reading(), 1_010_000_000)))
    attitude = consumer.ImuAttitude('robot_rgb_optical', 'capture/global_time')
    sample = attitude.observe(packet, 1_010_000_000)
    np.testing.assert_allclose(sample['angular_velocity'], [-2., -3., 1.])
    np.testing.assert_allclose(sample['linear_acceleration'], [0., -9.81, 0.])
    np.testing.assert_allclose(consumer.rotation_from_quaternion(sample['quaternion']) @ [0., -1., 0.],
                               [0., 0., 1.], atol=1e-9)
    assert attitude.info(1_010_000_000)['stamp_source'] == 'read_completion'
