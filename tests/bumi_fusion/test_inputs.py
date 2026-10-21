import json
from types import SimpleNamespace as NS
import pytest
from noetix.bumi.odometry_inputs import camera_metadata, imu_reading


def frame(ms, number=1):
    return NS(get_timestamp=lambda: ms, get_frame_number=lambda: number,
              get_frame_timestamp_domain=lambda: 'timestamp_domain.hardware_clock')


def test_camera_keeps_distinct_capture_times_and_depth_scale():
    meta = camera_metadata(frame(1000), frame(1010), epoch='e', depth_scale=.002,
                           received_ns=10_000_000_000)
    assert meta['rgb']['source_ms'] == 1000
    assert meta['depth']['source_ms'] == 1010
    assert meta['depth_scale'] == .002
    assert meta['epoch'] == 'e' and meta['received_ns'] == 10_000_000_000
    json.dumps(meta, allow_nan=False)


def test_imu_does_not_invent_source_timestamp_or_confidence():
    sdk = NS(ori=[0,0,0,1], angular_vel=[0,0,.2], linear_acc=[0,0,9.81])
    data = imu_reading(sdk, 100, 200, 7, 'session')
    assert data['source_stamp_ns'] is None and data['stamp_source'] == 'host_read'
    assert data['read_begin_ns'] == 100 and data['read_end_ns'] == 200
    assert data['gyro_covariance'] is None
    assert data['sequence'] == 7
    sdk.angular_vel[0] = float('nan')
    with pytest.raises(ValueError): imu_reading(sdk,100,200,8,'session')


def test_shared_raw_publisher_uses_sdk_scale_and_separate_source_times(monkeypatch):
    import sys
    import numpy as np
    from noetix.bumi.odometry_inputs import RawCameraPublisher
    class Image:
        def __init__(self):self.header=NS(stamp=NS(sec=0,nanosec=0),frame_id='')
    class String:
        def __init__(self,data):self.data=data
    messages={}
    node=NS(create_publisher=lambda cls,topic,qos:NS(publish=messages.setdefault(topic,[]).append))
    monkeypatch.setitem(sys.modules,'sensor_msgs.msg',NS(Image=Image,CameraInfo=Image))
    monkeypatch.setitem(sys.modules,'std_msgs.msg',NS(String=String))
    intr=NS(fx=100.,fy=101.,ppx=1.,ppy=1.,model='distortion.brown_conrady',coeffs=[0.]*5)
    color=frame(1000);color.get_data=lambda:np.zeros((2,3,3),dtype='u1')
    color.profile=NS(as_video_stream_profile=lambda:NS(intrinsics=intr))
    depth=frame(1010);depth.get_data=lambda:np.full((2,3),500,dtype='<u2')
    frames=NS(get_color_frame=lambda:color,get_depth_frame=lambda:depth)
    pipeline=NS(get_active_profile=lambda:NS(get_device=lambda:NS(first_depth_sensor=lambda:NS(get_depth_scale=lambda:.002))))
    publisher=RawCameraPublisher(node,'robot',pipeline,object())
    publisher.publish(frames,frames)
    raw=messages['/robot/odometry_input/depth'][0]
    assert raw.encoding=='32FC1' and raw.step==12
    assert np.frombuffer(raw.data,dtype='<f4').tolist()==pytest.approx([1.]*6)
    rgb=messages['/robot/odometry_input/rgb'][0]
    def stamp(msg):return msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
    assert stamp(raw)-stamp(rgb)==10_000_000
    meta=json.loads(messages['/robot/odometry_input/camera_meta'][0].data)
    assert meta['depth_scale']==.002 and meta['rgb']['source_ms']==1000
