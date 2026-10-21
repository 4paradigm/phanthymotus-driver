import math
import pytest
from noetix.bumi_odometry.timing import estimate_offset


def signal(t): return [0.,0.,.2*math.sin(2.1*t)+.13*math.sin(4.7*t)]


def test_estimates_read_delay_from_excited_rotation_not_static_data():
    # True IMU sample t is read at t+40ms. Visual timestamps retain t.
    visual=[(i*.02,signal(i*.02)) for i in range(501)]
    imu=[(i*.02+.04,signal(i*.02)) for i in range(501)]
    result=estimate_offset(visual,imu)
    assert result['offset_s']==pytest.approx(.04,abs=.005)
    assert result['uncertainty_s']<=.015
    with pytest.raises(ValueError,match='excitation'):
        estimate_offset([(i*.02,[0,0,0]) for i in range(501)],[(i*.02,[0,0,0]) for i in range(501)])


def test_nonmonotonic_or_short_timing_records_refuse_qualification():
    with pytest.raises(ValueError): estimate_offset([(0,[0,0,1])]*200,[(0,[0,0,1])]*200)


def test_visual_rate_uses_body_orientation_delta_and_not_ekf_velocity():
    from noetix.bumi_odometry.bag_export import angular_rate
    assert angular_rate([0,0,0,1],[0,0,math.sin(.05),math.cos(.05)],.1)==pytest.approx([0,0,1])
    assert angular_rate([0,0,0,1],[0,0,-math.sin(.05),-math.cos(.05)],.1)==pytest.approx([0,0,1])
