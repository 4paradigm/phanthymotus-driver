"""ROS-free checks for Bumi's planar fusion math and safety guards."""
import math
import numpy as np
from noetix.bumi.odometry_fusion import PlanarFusion, yaw_from_quat


def q(yaw):
    return [0,0,math.sin(yaw/2),math.cos(yaw/2)]


def cov(x=.01,y=.01,yaw=.03):
    result = [0.0]*36
    result[0], result[7], result[35] = x*x, y*y, yaw*yaw
    return result


def make_filter():
    ekf = PlanarFusion()
    for i in range(20):
        assert ekf.predict(1_000_000_000+i*25_000_000,0,(.002,0),True)
    return ekf


def test_first_visual_segment_preserves_pose():
    ekf=make_filter()
    x=ekf.steps[-1].state.copy()
    assert ekf.visual(stamp_ns=1_400_000_000,segment='seg1',position=[55,20,0],
                      quaternion=q(.3),covariance=cov(),inliers=50) == 'anchored'
    assert np.allclose(x,ekf.steps[-1].state)
    assert ekf.visual(stamp_ns=1_425_000_000,segment='seg1',position=[55.012,20,0],
                      quaternion=q(.3),covariance=cov(),inliers=50) == 'corrected'
    assert ekf.result(1_475_000_000)['status']=='tracking'


def test_rewind_corrects_delayed_pose_and_replays_future_motion():
    ekf=make_filter()
    x=ekf.steps[-1].state[0]
    assert ekf.visual(stamp_ns=1_250_000_000,segment='a',position=[0,0,0],
                      quaternion=q(0),covariance=cov(),inliers=40) == 'anchored'
    assert ekf.visual(stamp_ns=1_350_000_000,segment='a',position=[.07,0,0],
                      quaternion=q(0),covariance=cov(),inliers=40) == 'corrected'
    assert ekf.steps[-1].state[0] > x
    assert len(ekf.steps) == 20


def test_tracking_loss_grows_covariance_but_keeps_position():
    ekf=make_filter()
    x=ekf.steps[-1].state[0]
    p=ekf.steps[-1].cov[0,0]
    for i in range(20,40):
        ekf.predict(1_000_000_000+i*25_000_000,0,(.002,0),True)
    out=ekf.result(2_000_000_000)
    assert out['status'] == 'degraded'
    assert out['position'][0] > x
    assert out['covariance'][0][0] > p


def test_new_odom_segment_does_not_jump():
    ekf=make_filter()
    assert ekf.visual(stamp_ns=1_400_000_000,segment='a',position=[9,1,0],
                      quaternion=q(.1),covariance=cov(),inliers=40) == 'anchored'
    x=ekf.steps[-1].state.copy()
    assert ekf.visual(stamp_ns=1_425_000_000,segment='b',position=[0,0,0],
                      quaternion=q(0),covariance=cov(),inliers=40) == 'anchored'
    assert np.allclose(x,ekf.steps[-1].state)


def test_visual_outlier_and_replay_rejected():
    ekf=make_filter()
    assert ekf.visual(stamp_ns=1_300_000_000,segment='a',position=[0,0,0],quaternion=q(0),covariance=cov(),inliers=30)=='anchored'
    assert ekf.visual(stamp_ns=1_325_000_000,segment='a',position=[20,0,0],quaternion=q(0),covariance=cov(),inliers=30)=='implausible_visual'
    assert ekf.visual(stamp_ns=1_325_000_000,segment='a',position=[.005,0,0],quaternion=q(0),covariance=cov(),inliers=5)=='invalid_visual'
    assert ekf.visual(stamp_ns=1_325_000_000,segment='a',position=[.005,0,0],quaternion=q(0),covariance=cov(),inliers=30)=='corrected'
    assert ekf.visual(stamp_ns=1_325_000_000,segment='a',position=[.005,0,0],quaternion=q(0),covariance=cov(),inliers=30)=='replayed_visual'


def test_stale_sdk_no_pose():
    ekf=make_filter()
    assert ekf.result(3_000_000_000) is None
    assert not ekf.predict(1_475_000_000, 0, (.1,0),True)


def test_imu_yaw_tracks_increment_and_wrap():
    ekf=PlanarFusion()
    ekf.predict(1_000_000_000,math.pi-.05)
    ekf.predict(1_025_000_000,-math.pi+.05, (.02,0),True)
    assert abs(ekf.steps[-1].state[2]-.10)<1e-6
    assert math.isclose(yaw_from_quat(q(.3)),.3)


def test_plausible_high_inlier_visual_delta_flags_leg_slip():
    ekf = PlanarFusion()
    for i in range(36):
        ekf.predict(1_000_000_000+i*25_000_000, 0, (.01, 0), True)
    assert ekf.visual(stamp_ns=1_250_000_000, segment='s', position=[0,0,0],
                      quaternion=q(0), covariance=cov(), inliers=50) == 'anchored'
    assert ekf.visual(stamp_ns=1_750_000_000, segment='s', position=[.01,0,0],
                      quaternion=q(0), covariance=cov(), inliers=50) == 'corrected'
    assert ekf.slip_suspected
    assert ekf.slip_residual_m > .07


def test_clean_motion_slowly_calibrates_directional_leg_scale():
    ekf = PlanarFusion()
    for i in range(48):
        ekf.predict(1_000_000_000+i*25_000_000, 0, (.004, 0), True)
    assert ekf.visual(stamp_ns=1_250_000_000, segment='s', position=[0,0,0],
                      quaternion=q(0), covariance=cov(), inliers=50) == 'anchored'
    assert ekf.visual(stamp_ns=1_275_000_000, segment='s', position=[.008,0,0],
                      quaternion=q(0), covariance=cov(), inliers=50) == 'corrected'
    initial_scale = ekf.scales['forward']
    assert ekf.visual(stamp_ns=1_800_000_000, segment='s', position=[.176,0,0],
                      quaternion=q(0), covariance=cov(), inliers=50) == 'corrected'
    assert ekf.scales['forward'] > initial_scale
