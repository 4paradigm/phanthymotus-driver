"""Visual odometry contract: geometry, time, quality and output format.

These exercise the production code with independent literals. There is no IMU
input, no EKF and no fused-output in this contract, so nothing here may require
one.
"""
import math
from types import SimpleNamespace as NS

import numpy as np
import pytest

from noetix.bumi.visual_odometry_contract import (CameraCalibration, ClockMap, Health,
                                    camera_to_body, finite, make_sample, quat_matrix)
from noetix.bumi.visual_odometry import estimate, mount_transform, parameter_file


def calibration():
    return dict(calibration_id='fixture', fixed_mount=True,
                camera_to_body=dict(translation=[0.4, 0, 0.2], quaternion=[0, 0, 0, 1]))


def test_finite_rejects_nan_and_wrong_size():
    assert finite([1, 2, 3], 3) == [1., 2., 3.]
    for bad in ([1, 2], [1, 2, float('nan')], 'abc'):
        with pytest.raises(ValueError):
            finite(bad, 3)


def test_calibration_requires_a_measured_fixed_mount_only():
    with pytest.raises(ValueError):
        CameraCalibration({})
    c = CameraCalibration(calibration())
    assert c.id == 'fixture'
    for key in ('camera_to_body', 'fixed_mount'):
        data = calibration()
        data.pop(key)
        with pytest.raises(ValueError):
            CameraCalibration(data)
    bad = calibration()
    bad['fixed_mount'] = False
    with pytest.raises(ValueError):
        CameraCalibration(bad)
    bad = calibration()
    bad['camera_to_body']['quaternion'] = [0, 0, 0, 0]
    with pytest.raises(ValueError):
        CameraCalibration(bad)
    # No IMU contract may sneak back in.
    data = calibration()
    CameraCalibration(data)
    assert 'imu_to_body' not in CameraCalibration(data).data


def test_turn_with_camera_lever_arm_reports_stationary_body():
    # Body rotates +90 degrees at the origin; the camera at (0.4,0,0.2)
    # rides to (0,0.4,0.2), so the body point is still the origin.
    p, q = camera_to_body([0, 0.4, 0.2], [0, 0, math.sqrt(.5), math.sqrt(.5)],
                          calibration()['camera_to_body'])
    assert p == pytest.approx([0, 0, 0], abs=1e-12)
    expected = [v for row in quat_matrix([0, 0, math.sqrt(.5), math.sqrt(.5)]) for v in row]
    assert [v for row in quat_matrix(q) for v in row] == pytest.approx(expected, abs=1e-12)


def test_identity_mount_keeps_the_pose_unchanged():
    p, q = camera_to_body([1, 2, 3], [0, 0, 0, 1],
                          dict(translation=[0, 0, 0], quaternion=[0, 0, 0, 1]))
    assert p == pytest.approx([1, 2, 3]) and q == pytest.approx([0, 0, 0, 1])


def test_bad_quaternion_and_time_domains_refuse():
    with pytest.raises(ValueError):
        quat_matrix([0, 0, 0, 0])
    clock = ClockMap()
    assert clock.convert(1000, 'hardware_clock', 10_000_000_000) == 10_000_000_000
    assert clock.convert(1033, 'hardware_clock', 10_033_000_000) == 10_033_000_000
    with pytest.raises(ValueError):
        clock.convert(2, 'hardware_clock', 10_050_000_000)
    with pytest.raises(ValueError):
        ClockMap().convert(2, 'unknown', 10_050_000_000)
    assert ClockMap().convert(2000, 'global_time', 2_000_000_000) == 2_000_000_000


def healthy():
    health = Health('s', 'c', started_ns=1_000_000_000)
    for i in range(3):
        t = 1_000_000_000+i*50_000_000
        for kind in health.kinds:
            health.observe(kind, t, t, True)
    return health


def test_tracking_needs_camera_and_paired_visual_diagnostic():
    health = healthy()
    assert health.status(1_100_000_000)['status'] == 'tracking'
    # A visual pose with no matching diagnostic is not tracking evidence.
    only_camera = Health('s', None, started_ns=0)
    for i in range(5):
        only_camera.observe('camera', i*10_000_000, i*10_000_000, True)
    assert only_camera.status(60_000_000)['status'] == 'initializing'


def test_lost_is_latched_and_never_hidden_by_a_later_pose():
    health = healthy()
    assert health.status(1_100_000_000)['status'] == 'tracking'
    health.observe('diagnostic', 1_020_000_000, 1_110_000_000, False, 'visual lost')
    assert health.status(1_110_000_000)['status'] == 'lost'
    health.observe('visual', 1_710_000_000, 1_710_000_000, True)
    assert health.status(1_710_000_000)['status'] == 'lost'


def test_stale_input_is_lost_not_predicted_forward():
    health = healthy()
    assert health.status(1_100_000_000)['status'] == 'tracking'
    assert health.status(900_000_000_000)['status'] == 'lost'
    assert 'stale' in health.status(900_000_000_000)['reason']


def test_sample_never_invents_a_twist_or_a_pose_without_tracking():
    health = Health('s', None, started_ns=0)
    sample = make_sample(health, 100, None)
    assert sample['schema'] == 'motus.odom/1'
    assert sample['twist'] == [None]*6
    assert sample['pose'] is None
    assert sample['vendor']['source'] == 'rtabmap_rgbd_odometry'
    assert sample['vendor']['pose_reference'] == 'rgb_optical'
    assert sample['vendor']['accuracy_validated'] is False
    assert sample['vendor']['status'] == 'initializing'


def test_tracking_sample_carries_the_pose_and_diagnostic_details():
    health = healthy()
    health.details['visual_diagnostic'] = dict(inliers=20, matches=30, lost=False)
    pose = dict(position=[1, 2, 3], quaternion=[0, 0, 0, 1],
                frame_id='s_odom', child_frame_id='opt', covariance=[0.]*36)
    sample = make_sample(health, 1_100_000_000, dict(stamp_ns=1_090_000_000, pose=pose),
                         pose_reference='body')
    assert sample['pose'] == pose
    assert sample['twist'] == [None]*6
    assert sample['vendor']['status'] == 'tracking'
    assert sample['vendor']['pose_reference'] == 'body'
    assert sample['vendor']['visual_diagnostic'] == dict(inliers=20, matches=30, lost=False)


def odometry_msg(*, position=(0, 0, 0), quaternion=(0, 0, 0, 1), covariance=None,
                 frame='seg_odom', child='opt', sec=10, nanosec=0):
    return NS(header=NS(stamp=NS(sec=sec, nanosec=nanosec), frame_id=frame),
              child_frame_id=child,
              pose=NS(pose=NS(position=NS(x=position[0], y=position[1], z=position[2]),
                              orientation=NS(x=quaternion[0], y=quaternion[1],
                                            z=quaternion[2], w=quaternion[3])),
                      covariance=list(covariance if covariance is not None else [0.]*36)))


def test_estimate_parses_rtabmap_pose_and_frames():
    data = estimate(odometry_msg(position=(1, 2, 3)), 'seg', 'opt')
    assert data['stamp_ns'] == 10*10**9
    assert data['pose']['position'] == [1., 2., 3.]
    assert data['pose']['frame_id'] == 'seg_odom'
    assert data['pose']['child_frame_id'] == 'opt'
    assert data['pose']['covariance'] == [0.]*36


def test_estimate_refuses_wrong_frames_and_broken_covariance():
    with pytest.raises(ValueError):
        estimate(odometry_msg(frame='other'), 'seg', 'opt')
    with pytest.raises(ValueError):
        estimate(odometry_msg(child='other'), 'seg', 'opt')
    with pytest.raises(ValueError):
        estimate(odometry_msg(quaternion=(0, 0, 0, 0)), 'seg', 'opt')
    negative = [0.]*36
    negative[0] = -1.
    with pytest.raises(ValueError):
        estimate(odometry_msg(covariance=negative), 'seg', 'opt')
    too_uncertain = [0.]*36
    too_uncertain[0] = too_uncertain[7] = too_uncertain[14] = .09
    with pytest.raises(ValueError):
        estimate(odometry_msg(covariance=too_uncertain), 'seg', 'opt')


def test_mount_transform_is_identity_or_the_measured_mount():
    assert mount_transform(None) == ([0., 0., 0.], [0., 0., 0., 1.])
    mount = calibration()['camera_to_body']
    assert mount_transform(CameraCalibration(calibration())) == (
        mount['translation'], mount['quaternion'])


def test_parameter_file_preserves_ros_parameter_types():
    text = parameter_file(dict(frame_id='robot_rgb_optical', publish_tf=False,
                               **{'Odom/ResetCountdown': '0'},
                               approx_sync_max_interval=.02))
    import yaml
    parsed = yaml.safe_load(text)['/**']['ros__parameters']
    assert parsed['frame_id'] == 'robot_rgb_optical'
    assert parsed['publish_tf'] is False
    assert parsed['Odom/ResetCountdown'] == '0'
    assert parsed['approx_sync_max_interval'] == .02


def test_visual_results_get_a_longer_grace_than_the_camera_feed():
    health = healthy()
    # Camera stays fresh while RTAB's visual result is ~500 ms old: walking
    # processing latency must not end the segment on the old 500 ms edge.
    health.observe('camera', 1_200_000_000, 1_200_000_000, True)
    quality = health.status(1_600_000_000)
    assert quality['status'] == 'tracking'
    # A genuinely stalled estimator still ends the segment.
    assert health.status(2_800_000_000)['status'] == 'lost'
    assert 'stale' in health.status(2_800_000_000)['reason']


def test_level_forward_mount_maps_optical_axes_to_robot_body_axes():
    from noetix.bumi.visual_odometry_contract import quat_matrix
    R = quat_matrix([-.5, .5, -.5, .5])
    def apply(vector):
        return [sum(R[i][j]*vector[j] for j in range(3)) for i in range(3)]
    # optical x=right -> body y=-left; optical y=down -> body z=-up;
    # optical z=forward -> body x=forward.
    assert apply([1., 0., 0.]) == [0., -1., 0.]
    assert apply([0., 1., 0.]) == [0., 0., -1.]
    assert apply([0., 0., 1.]) == [1., 0., 0.]


def test_one_point_eight_hz_capture_does_not_trip_the_camera_watchdog():
    # The older shared-capture cadence was ~556 ms between frames. The camera
    # watchdog is 1 s so that cadence still produces tracking observations.
    health = Health('s', None, started_ns=0)
    for i in range(4):
        t = i*556_000_000
        for kind in health.kinds:
            health.observe(kind, t, t, True)
    quality = health.status(4*556_000_000)
    assert quality['status'] == 'tracking'
    assert quality['input_age_ms']['camera'] < 1000
