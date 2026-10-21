"""Independent literals exercise the production geometry and fault contract."""
import math
import pytest
from common.fusion_odometry import (Calibration, Health, camera_to_body, body_velocity,
                                    quat_matrix, make_sample, ClockMap)


def calibration():
    return dict(calibration_id='fixture', fixed_mount=True,
                camera_to_body=dict(translation=[0.4, 0, 0.2], quaternion=[0, 0, 0, 1]),
                imu_to_body=dict(translation=[0, 0, 0], quaternion=[0, 0, 0, 1]),
                imu_units='m/s2,rad/s', imu_quaternion='xyzw_sensor_to_enu',
                gravity_included=True, gyro_bias=[0, 0, 0], gyro_variance=[0.001]*3,
                timing=dict(mode='host_read_qualified', offset_s=0.02, uncertainty_s=0.01,
                            qualification_id='fixture', max_age_s=0.25))


def test_requires_measured_contract():
    with pytest.raises(ValueError): Calibration({})
    c = Calibration(calibration())
    assert c.id == 'fixture'
    for key in ('imu_units', 'imu_quaternion', 'timing', 'fixed_mount'):
        data = calibration(); data.pop(key)
        with pytest.raises(ValueError): Calibration(data)


def test_turn_with_camera_lever_arm_reports_stationary_body():
    # Body rotates +90 degrees at origin; camera at (0.4,0,0.2) -> (0,0.4,0.2).
    p, r = camera_to_body([0, 0.4, 0.2], [0, 0, math.sqrt(.5), math.sqrt(.5)],
                          calibration()['camera_to_body'])
    assert p == pytest.approx([0, 0, 0], abs=1e-12)
    assert r[1][0] == pytest.approx(1)
    assert body_velocity([0, .4, 0], [0, 0, 1], [0.4, 0, .2]) == pytest.approx([0, 0, 0])


def test_bad_quaternion_and_time_domains_refuse():
    with pytest.raises(ValueError): quat_matrix([0, 0, 0, 0])
    clock = ClockMap()
    assert clock.convert(1000, 'hardware_clock', 10_000_000_000) == 10_000_000_000
    assert clock.convert(1033, 'hardware_clock', 10_033_000_000) == 10_033_000_000
    with pytest.raises(ValueError): clock.convert(2, 'hardware_clock', 10_050_000_000)
    with pytest.raises(ValueError): ClockMap().convert(2, 'unknown', 10_050_000_000)


def healthy():
    health = Health('s', 'c', started_ns=1_000_000_000)
    for i in range(3):
        t = 1_000_000_000+i*50_000_000
        for kind in health.kinds: health.observe(kind, t, t, True)
    return health


def test_predicting_filter_does_not_hide_visual_loss():
    health = healthy()
    assert health.status(1_100_000_000)['status'] == 'tracking'
    health.observe('filtered', 1_700_000_000, 1_700_000_000, True)
    assert health.status(1_700_000_000)['status'] == 'lost'
    health.observe('visual', 1_710_000_000, 1_710_000_000, True)
    assert health.status(1_710_000_000)['status'] == 'lost'


def test_late_diagnostic_fault_is_latched_even_if_stamp_old():
    h = healthy()
    h.observe('diagnostic', 1_020_000_000, 1_110_000_000, False, 'visual lost')
    assert h.status(1_110_000_000)['reason'] == 'visual lost'
    sample = make_sample(h, 1_110_000_000, None)
    assert sample['pose'] is None and sample['twist'] == [None]*6
    assert sample['vendor']['segment_id'] == 's'
    assert sample['vendor']['accuracy_validated'] is False
