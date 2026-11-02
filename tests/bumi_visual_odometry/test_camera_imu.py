import pytest

from noetix.bumi.camera_imu import SharedClock, MotionPairing, matrix_quaternion
from noetix.bumi.visual_odometry_contract import quat_matrix


def test_images_and_imu_share_one_offset_with_independent_monotonic_sequences():
    clock = SharedClock()
    assert clock.convert('gyro', 1000., 'hardware_clock', 10_000_000_000) == 10_000_000_000
    assert clock.convert('rgb', 998., 'hardware_clock', 10_003_000_000) == 9_998_000_000
    assert clock.convert('accel', 1001., 'hardware_clock', 10_005_000_000) == 10_001_000_000
    with pytest.raises(ValueError):
        clock.convert('gyro', 999., 'hardware_clock', 10_006_000_000)
    with pytest.raises(ValueError):
        clock.convert('rgb', 1002., 'global_time', 10_006_000_000)


def test_accel_is_interpolated_at_actual_gyro_time_without_extrapolation():
    pairing = MotionPairing()
    assert pairing.add('gyro', 5_000_000, [1, 2, 3]) == []
    assert pairing.add('accel', 0, [0, 0, 10]) == []
    assert pairing.add('accel', 10_000_000, [2, 4, 10]) == [(5_000_000, [1., 2., 3.], [1., 2., 10.])]
    assert pairing.add('gyro', 15_000_000, [4, 5, 6]) == []
    assert pairing.add('accel', 20_000_000, [4, 8, 10]) == [(15_000_000, [4., 5., 6.], [3., 6., 10.])]


def test_pairing_refuses_reversed_timestamps_and_large_accel_gaps():
    pairing = MotionPairing()
    pairing.add('accel', 0, [0, 0, 10])
    with pytest.raises(ValueError):
        pairing.add('accel', 0, [0, 0, 10])
    pairing.add('gyro', 5_000_000, [0, 0, 0])
    with pytest.raises(ValueError):
        pairing.add('accel', 100_000_000, [0, 0, 10])


@pytest.mark.parametrize('matrix', [
    [[0., -1., 0.], [1., 0., 0.], [0., 0., 1.]],
    [[1., 0., 0.], [0., -1., 0.], [0., 0., -1.]],
])
def test_rotation_to_tf_preserves_direction_including_half_turns(matrix):
    import numpy as np
    assert np.asarray(quat_matrix(matrix_quaternion(matrix))) == pytest.approx(np.asarray(matrix))


def test_imu_assistance_requires_orientation_and_refuses_it_after_500ms():
    from noetix.bumi.visual_odometry_contract import Health
    now = 10_000_000_000
    health = Health('segment', None, now-200_000_000, imu_required=True)
    for t in (now-100_000_000, now-50_000_000, now):
        for kind in ('camera', 'visual', 'diagnostic'):
            health.observe(kind, t, now, True)
    assert health.status(now)['status'] == 'initializing'
    for t in (now-10_000_000, now-5_000_000, now):
        health.observe('imu', t, now, True)
    assert health.status(now)['status'] == 'tracking'
    assert health.status(now+500_000_001)['status'] == 'lost'


def test_assisted_estimator_launches_filter_and_uses_private_orientation(tmp_path):
    from pathlib import Path
    from types import SimpleNamespace as NS
    from noetix.bumi.visual_odometry import ChildProcesses
    calls, stopped = [], []
    def popen(args, **kwargs):
        calls.append(args)
        return NS(pid=len(calls), poll=lambda: None)
    manager = ChildProcesses(tmp_path, popen=popen, stop=stopped.append,
                             prefix=lambda package: Path('/opt/ros/humble'))
    manager.start('segment', 'base', imu_topic='/robot/odometry_input/imu_raw')
    assert len(calls) == 2
    assert 'imu_filter_madgwick_node' in calls[0][0]
    assert 'imu/data_raw:=/robot/odometry_input/imu_raw' in calls[0]
    assert 'imu/data:=/segment/imu' in calls[0]
    assert 'imu:=/segment/imu' in calls[1]
    assert 'wait_imu_to_init: true' in (tmp_path/'odometry.yaml').read_text()
    filter_parameters = (tmp_path/'madgwick.yaml').read_text()
    assert 'use_mag: false' in filter_parameters and 'publish_tf: false' in filter_parameters
    manager.stop()
    assert [p.pid for p in stopped] == [2, 1]


def test_raw_publisher_rotates_accel_to_gyro_axes_and_keeps_factory_mount(monkeypatch):
    import sys
    from types import SimpleNamespace as NS
    from noetix.bumi.camera_imu import CameraImuPublisher
    identity = [1.,0.,0.,0.,1.,0.,0.,0.,1.]
    quarter_turn = [0.,1.,0.,-1.,0.,0.,0.,0.,1.]
    profiles = {'g': NS(get_extrinsics_to=lambda _: NS(rotation=identity, translation=[.02,-.004,-.01])),
                'a': NS(get_extrinsics_to=lambda _: NS(rotation=quarter_turn, translation=[0.,0.,0.])),
                'c': object()}
    monkeypatch.setitem(sys.modules,'pyrealsense2',NS(stream=NS(gyro='g',accel='a',color='c')))
    class Imu:
        def __init__(self):
            self.header=NS(stamp=NS(sec=0,nanosec=0),frame_id='')
            self.orientation_covariance=[0.]*9
            self.angular_velocity=NS(x=0.,y=0.,z=0.)
            self.linear_acceleration=NS(x=0.,y=0.,z=0.)
    monkeypatch.setitem(sys.modules,'sensor_msgs.msg',NS(Imu=Imu))
    published=[]
    node=NS(create_publisher=lambda *args:NS(publish=published.append))
    pipeline=NS(get_active_profile=lambda:NS(get_stream=lambda kind:profiles[kind]))
    publisher=CameraImuPublisher(node,'robot',pipeline,object())
    publisher.enabled=True
    def frame(kind,t,xyz):
        return NS(get_profile=lambda:NS(stream_type=lambda:kind),get_timestamp=lambda:t,
                  get_frame_timestamp_domain=lambda:'hardware_clock',
                  as_motion_frame=lambda:NS(get_motion_data=lambda:NS(x=xyz[0],y=xyz[1],z=xyz[2])))
    publisher.observe(frame('g',1005,[.1,.2,.3]),10_000_000_000)
    publisher.observe(frame('a',1000,[1.,0.,10.]),10_005_000_000)
    assert not published
    publisher.observe(frame('a',1010,[3.,0.,10.]),10_010_000_000)
    msg=published[0]
    assert (msg.linear_acceleration.x,msg.linear_acceleration.y,msg.linear_acceleration.z)==pytest.approx([0.,2.,10.])
    assert msg.orientation_covariance[0]==-1 and msg.header.frame_id=='robot_camera_imu'
    assert msg.header.stamp.sec==10 and msg.header.stamp.nanosec==0
    assert publisher.metadata()['translation_m']==[.02,-.004,-.01]
    epoch=publisher.metadata()['epoch']
    publisher.restart()
    assert publisher.metadata()['epoch']!=epoch and publisher.metadata()['stamp_ns'] is None


def test_assistance_explicitly_disables_acceleration_motion_prediction():
    from noetix.bumi.visual_odometry import parameters
    assert parameters('segment','base',imu_assisted=True)['Odom/GuessSmoothingDelay']=='0'


def test_sdk_global_clock_is_not_reanchored_to_packet_arrival():
    clock=SharedClock()
    for seconds in (0,60,120,300):
        source_ns=10_000_000_000+seconds*10**9
        for kind,delay in (('gyro',5_000_000),('rgb',40_000_000),('accel',10_000_000)):
            assert clock.convert(kind,source_ns/1e6,'global_time',source_ns+delay)==source_ns
    assert clock.offset==0
    with pytest.raises(ValueError,match='repeated or reset'):
        clock.convert('gyro',10_000.,'global_time',310_000_000_000)
