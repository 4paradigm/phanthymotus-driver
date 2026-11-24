"""Bumi geometry evidence consumed by generic, robot-agnostic SLAM."""
import numpy as np
import yaml
from pathlib import Path

from noetix.bumi.visual_odometry import VisualOdometryNode
from noetix.bumi.visual_odometry_contract import CameraCalibration


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


def test_legacy_labels_do_not_change_numeric_bumi_mount():
    mount = _node(ASSUMED).slam_extrinsic()
    assert mount['parent_frame'] == 'segment_base'
    assert mount['child_frame'] == 'ubuntu_rgb_optical'
    assert set(mount) == {'parent_frame', 'child_frame', 'mode', 'matrix'}
    np.testing.assert_allclose(mount['matrix'], [
        [0, 0, 1, 0], [-1, 0, 0, 0],
        [0, -1, 0, 0], [0, 0, 0, 1]])


def test_shipped_bumi_mount_reports_the_measured_numeric_transform():
    config_path = Path(__file__).resolve().parents[2] / 'noetix/bumi/config.yaml'
    calibration = yaml.safe_load(config_path.read_text())['plugins']['visual_odometry']['calibration']
    node = _node(calibration)
    assert node.pose_reference() == 'body'
    mount = node.slam_extrinsic()
    assert set(mount) == {'parent_frame', 'child_frame', 'mode', 'matrix'}
    np.testing.assert_allclose(mount['matrix'], [
        [0, 0, 1, 0], [-1, 0, 0, 0],
        [0, -1, 0, 0], [0, 0, 0, 1]])


def test_measured_and_assumed_labels_produce_identical_numeric_geometry():
    measured = {**ASSUMED, 'assumed': False, 'calibration_id': 'survey-2026'}
    mount = _node(measured).slam_extrinsic()
    assert mount == _node(ASSUMED).slam_extrinsic()
    assert _node(None).slam_extrinsic() is None
    no_frame = _node(measured)
    no_frame.frame_id = None
    assert no_frame.slam_extrinsic() is None
