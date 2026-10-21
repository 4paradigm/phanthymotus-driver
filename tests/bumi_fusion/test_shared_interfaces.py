"""Exercise the two PRs' real producer/consumer contracts without ROS or USB."""
import json
from types import SimpleNamespace as NS

import numpy as np
import pytest
from test_runtime import runtime
from test_contract import calibration, healthy
from test_inputs import frame
from test_feedback import binding
from noetix.bumi.odometry_inputs import RawCameraPublisher, imu_reading
from noetix.bumi.odometry_feedback import DistanceFeedback
from common.fusion_odometry import make_sample


def test_shared_raw_camera_and_imu_are_accepted_by_independent_sensor(runtime,monkeypatch):
    import sys
    from noetix.bumi import odometry_inputs
    class Image:
        def __init__(self): self.header=NS(stamp=NS(sec=0,nanosec=0),frame_id='')
    class String:
        def __init__(self,data): self.data=data
    monkeypatch.setitem(sys.modules,'sensor_msgs.msg',NS(Image=Image,CameraInfo=Image,Imu=Image))
    monkeypatch.setitem(sys.modules,'std_msgs.msg',NS(String=String))
    r=runtime; r.config['calibration']=calibration(); r.start()
    monkeypatch.setattr(odometry_inputs.time,'time_ns',r.now)
    outputs={}
    node=NS(create_publisher=lambda cls,topic,qos:NS(publish=outputs.setdefault(topic,[]).append))
    pipeline=NS(get_active_profile=lambda:NS(get_device=lambda:NS(first_depth_sensor=lambda:NS(get_depth_scale=lambda:.002))))
    color=frame(1000);color.get_data=lambda:np.zeros((2,3,3),dtype='u1')
    intr=NS(fx=100.,fy=100.,ppx=1.,ppy=1.,model='distortion.none',coeffs=[0.]*5)
    color.profile=NS(as_video_stream_profile=lambda:NS(intrinsics=intr))
    depth=frame(1010);depth.get_data=lambda:np.full((2,3),500,dtype='<u2')
    frames=NS(get_color_frame=lambda:color,get_depth_frame=lambda:depth)
    RawCameraPublisher(node,'robot',pipeline,object()).publish(frames,frames)
    root='/robot/odometry_input/'
    r.metadata(outputs[root+'camera_meta'][0])
    r.rgbd(*(outputs[root+key][0] for key in ('rgb','depth','info')))
    assert r.health.samples['camera']==10**10
    assert np.frombuffer(outputs[root+'depth'][0].data,dtype='<f4').tolist()==pytest.approx([1.]*6)
    sdk=NS(ori=[0,0,0,1],angular_vel=[0,0,0],linear_acc=[0,0,9.81])
    for index in range(102):
        r.node.time=10**10+index*20_000_000
        data=imu_reading(sdk,r.now(),r.now(),index+1,'imu-session')
        r.imu(String(json.dumps(data)))
    assert r.initial==pytest.approx((0,0))
    assert r.health.calibration_id=='fixture'
    assert not r.children.children  # no RTAB-Map or camera process launched by the test
    assert data['source_stamp_ns'] is None


def test_sensor_output_builder_matches_destination_input_without_translation(monkeypatch):
    h=healthy(); now=1_100_000_000
    estimate=dict(stamp_ns=now,twist=[0.]*6,
                  pose=dict(position=[2.,3.,0.],quaternion=[0,0,0,1],frame_id='odom',child_frame_id='base_link',
                            covariance=[.001 if i%7==0 else 0 for i in range(36)]))
    f=DistanceFeedback(None,now_ns=lambda:now,monotonic=lambda:1.1)
    monkeypatch.setattr(f,'_subscribe',lambda topic:None)
    f.apply(binding(topic='/bumi_rgbd_imu_odometry/state/odom',issued=now))
    f.update(make_sample(h,now,estimate))
    context=f.freeze()
    assert context['source']=='rgbd_imu_ekf' and context['segment']=='s'
    assert context['origin']['pose']['position']==[2.,3.,0.]
