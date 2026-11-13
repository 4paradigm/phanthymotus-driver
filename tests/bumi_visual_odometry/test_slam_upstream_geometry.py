"""Bumi geometry evidence consumed by generic, robot-agnostic SLAM."""
import numpy as np
import yaml
from pathlib import Path

from noetix.bumi.visual_odometry import VisualOdometryNode
from noetix.bumi.visual_odometry_contract import CameraCalibration
from noetix.bumi.camera_specs import declare


ASSUMED = {
    'calibration_id': 'assumed-level-forward-optical-to-body',
    'fixed_mount': True, 'assumed': True,
    'camera_to_body': {
        'translation': [0, 0, 0],
        'quaternion': [-0.5, 0.5, -0.5, 0.5],
    },
}


def _node(calibration):
    node = object.__new__(VisualOdometryNode)
    node.namespace = 'ubuntu'
    node.segment = 'segment'
    node.frame_id = 'ubuntu_rgb_optical'
    node.calibration = CameraCalibration(calibration) if calibration else None
    return node


def test_assumed_bumi_mount_is_described_not_declared_calibrated():
    mount = _node(ASSUMED).slam_extrinsic()
    assert mount['parent_frame'] == 'segment_base'
    assert mount['child_frame'] == 'ubuntu_rgb_optical'
    assert mount['calibrated'] is False
    assert mount['pose_reference'] == 'body_axes'
    assert mount['source'] == 'bumi_axis_assumption'
    assert mount['version'] == ASSUMED['calibration_id']
    np.testing.assert_allclose(mount['matrix'], [
        [0, 0, 1, 0], [-1, 0, 0, 0],
        [0, -1, 0, 0], [0, 0, 0, 1]])


def test_shipped_bumi_mount_is_accepted_as_calibrated_for_slam():
    config_path = Path(__file__).resolve().parents[2] / 'noetix/bumi/config.yaml'
    calibration = yaml.safe_load(config_path.read_text())['plugins']['visual_odometry']['calibration']
    mount = _node(calibration).slam_extrinsic()
    assert mount['calibrated'] is True
    assert mount['source'] == 'measured_mount'
    assert mount['pose_reference'] == 'body'
    assert mount['version'] == 'verified-bumi-level-forward-optical-to-body-v1'
    np.testing.assert_allclose(mount['matrix'], [
        [0, 0, 1, 0], [-1, 0, 0, 0],
        [0, -1, 0, 0], [0, 0, 0, 1]])


def test_measured_mount_is_distinguished_from_assumed_and_unknown():
    measured = {**ASSUMED, 'assumed': False, 'calibration_id': 'survey-2026'}
    mount = _node(measured).slam_extrinsic()
    assert mount['calibrated'] is True
    assert mount['source'] == 'measured_mount'
    assert mount['pose_reference'] == 'body'
    assert _node(None).slam_extrinsic() is None
    no_frame = _node(measured)
    no_frame.frame_id = None
    assert no_frame.slam_extrinsic() is None


def test_depth_camera_info_advertises_frame_without_clock_proof():
    values = declare('depth', '/ubuntu/camera/depth', 'image/depth-zlib')
    assert len(values) == 1
    hint = values[0]['vendor']['slam_geometry']
    assert hint['frame_id'] == 'ubuntu_rgb_optical'
    assert hint['clock_id'] == 'ros'
    assert hint['clock_verified'] is False
    assert hint['clock_mapping'] == 'shared_camera_mapper'
    assert hint['observation_model'] == 'pinhole'


def test_depth_geometry_hint_survives_runtime_intrinsics_adoption():
    runtime = {'depth': {
        'width': 640, 'height': 480, 'fx': 505., 'fy': 506.,
        'ppx': 320., 'ppy': 240., 'coeffs': [0.] * 5,
        'model': 'none',
    }}
    declaration = declare('depth', '/ubuntu/camera/depth',
                          'image/depth-zlib', runtime)[0]
    assert declaration['source'] == 'derived-from-K'
    hint = declaration['vendor']['slam_geometry']
    assert hint['frame_id'] == 'ubuntu_rgb_optical'
    assert hint['clock_verified'] is False
    assert declaration['vendor']['realsense_distortion_model'] == 'none'
