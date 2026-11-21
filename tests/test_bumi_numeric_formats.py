"""Bumi exports numeric sensor contracts without qualification declarations."""
import json

import numpy as np

from noetix.bumi.body_imu import BodyImuSource
from noetix.bumi.rgbd_camera import camera_declarations, fixed_body_mount
from noetix.bumi.visual_odometry_contract import CameraCalibration


MOUNT = dict(camera_to_body=dict(translation=[.2, .1, .3],
                               quaternion=[-.5, .5, -.5, .5]),
             body_imu_translation=dict(gyro=[.1, 0., 0.], accel=[0., .2, 0.]))


def test_camera_mount_uses_numbers_without_calibration_certificates():
    mount = CameraCalibration(MOUNT)
    assert mount.data['camera_to_body'] == MOUNT['camera_to_body']
    labels = dict(MOUNT, calibration_id=None, assumed='unused', calibrated=False)
    assert CameraCalibration(labels).data['camera_to_body'] == MOUNT['camera_to_body']


def test_numeric_camera_mount_preserves_rotation_for_large_finite_quaternion():
    config = dict(camera_to_body=dict(translation=[.2, .1, .3],
                                     quaternion=[-1e308, 1e308, -1e308, 1e308]))
    np.testing.assert_allclose(fixed_body_mount(config),
                               [[0., 0., 1., .2], [-1., 0., 0., .1],
                                [0., -1., 0., .3], [0., 0., 0., 1.]])


def test_rgbd_public_metadata_contains_numeric_geometry_without_clock_or_source_claims():
    geometry = dict(K=[500., 0., 320., 0., 500., 240., 0., 0., 1.])
    entries = camera_declarations('/robot/camera/rgbd', geometry, 'robot_rgb_optical',
                                  'internal-clock', 'hardware_clock', 123, source_epoch='internal-run')
    for entry in entries:
        assert set(entry['vendor']) == ({'frame_id', 'depth_unit'}
                                       if entry['format'] == 'image/depth-zlib' else {'frame_id'})
        assert entry['K'] == geometry['K']
        assert entry['D'] == [0.] * 5 and entry['distortion_model'] == 'pinhole'
    assert entries[1]['vendor']['depth_unit'] == 'mm'


def test_body_imu_public_packet_keeps_computable_numbers_only():
    source = BodyImuSource('robot', MOUNT, lambda: ('robot_rgb_optical', None))
    packet = source.accept(dict(stamp_ns=1_000_000_000,
                                quaternion=[0., 0., 0., 1.], angular_vel=[1., 2., 3.],
                                linear_acc=[0., 0., 9.81]), 1_010_000_000)
    assert set(packet) == {'schema', 'stamp_ns', 'accel_stamp_ns', 'camera_frame_id',
                           'gyro_frame_id', 'accel_frame_id', 'camera_from_gyro',
                           'camera_from_accel', 'angular_velocity', 'linear_acceleration',
                           'units', 'camera_from_gyro_translation', 'camera_from_accel_translation'}
    assert packet['stamp_ns'] == packet['accel_stamp_ns'] == 1_000_000_000
    np.testing.assert_allclose(np.array(packet['camera_from_gyro']) @ [1., 2., 3.], [-2., -3., 1.])
    json.dumps(packet, allow_nan=False)
