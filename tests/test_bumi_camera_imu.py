"""Exercise Bumi's real capture-owner callback boundary without ROS/hardware."""
import importlib.util
import json
import sys
import time
from pathlib import Path
from types import SimpleNamespace as NS

ROOT=Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('bumi_imu_capture_fixture',ROOT/'tests/bumi_visual_odometry/test_bundle_hook.py')
fixture=importlib.util.module_from_spec(spec)
spec.loader.exec_module(fixture)
capture=fixture.capture


def test_shared_bumi_callback_publishes_motion_while_rgbd_failure_keeps_video(capture, monkeypatch):
    device,state=capture
    sdk=sys.modules['pyrealsense2']
    sdk.stream.gyro,sdk.stream.accel='gyro','accel'
    sdk.format.motion_xyz32f='xyz'
    sdk.camera_info=NS(serial_number='serial')
    profiles={kind:NS(stream_type=lambda kind=kind:kind,format=lambda:'xyz',fps=lambda:100,
                       stream_index=lambda:0,unique_id=lambda kind=kind:kind,
                       get_extrinsics_to=lambda depth:NS(rotation=[1,0,0,0,1,0,0,0,1]))
              for kind in ('gyro','accel','depth')}
    camera=NS(query_sensors=lambda:[NS(get_stream_profiles=lambda:list(profiles.values()))],
              get_info=lambda kind:'camera123')
    active=NS(get_stream=profiles.__getitem__,get_device=lambda:camera)
    sdk.pipeline_wrapper=lambda pipeline:pipeline
    sdk.config=lambda:NS(enable_stream=lambda *args:state.streams.append(args),
                         disable_stream=lambda kind:None,resolve=lambda pipeline:active)
    pipeline_type=sdk.pipeline
    original_wait=pipeline_type.wait_for_frames
    def start(pipeline,config,callback):
        state.pipeline,state.callback=pipeline,callback
        return active
    monkeypatch.setattr(pipeline_type,'start',start)
    offset=time.time_ns()-1_000_000_000
    class Rgbd:
        def __init__(self,*args,**kwargs):
            self.clocks=[NS(offset=offset,domain='hardware_clock')]
            self.clock_id='shared_rgbd'
        def motion_stamp(self,ms,domain):
            assert domain=='hardware_clock'
            return int(ms*1e6)+self.clocks[0].offset
        def publish(self,frames):raise ValueError('RGB-D test failure')
        def unavailable(self,error):state.rgbd_error=str(error)
    monkeypatch.setitem(sys.modules,'rgbd_camera',NS(RgbdCameraPublisher=Rgbd))
    def motion(kind):
        return NS(is_motion_frame=lambda:True,as_motion_frame=lambda:NS(
            profile=profiles[kind],get_timestamp=lambda:1000.,
            get_frame_timestamp_domain=lambda:'hardware_clock',
            get_motion_data=lambda:NS(x=0.,y=9.81 if kind=='accel' else 0.,z=0.)),
            is_frameset=lambda:False)
    class Composite:
        def __init__(self,frames):self.frames=frames
        def is_motion_frame(self):return False
        def is_frameset(self):return True
        def as_frameset(self):return self
        def __iter__(self):return iter(())
        def get_color_frame(self):return self.frames.get_color_frame()
        def get_depth_frame(self):return self.frames.get_depth_frame()
    def spin(node):
        state.aligned=True
        frames=original_wait(state.pipeline,1000)
        state.callback(motion('accel'))
        state.callback(motion('gyro'))
        state.callback(Composite(frames))
    state.on_spin=spin
    monkeypatch.delenv('BUMI_VISUAL_ODOMETRY',raising=False)
    device._camera_subprocess('robot')
    packet=json.loads(state.published['/robot/camera/rgbd/imu'][0].data)
    assert packet['clock_id']=='shared_rgbd/hardware_clock'
    assert packet['camera_frame_id']=='robot_depth_optical'
    assert packet['stamp_ns']==1_000_000_000+offset
    assert packet['clock_mapping']['offset_ns']==offset
    assert packet['clock_mapping']['verified'] is False
    assert len(state.published['/robot/camera/color'])==1
    assert len(state.published['/robot/camera/depth'])==1
    assert state.rgbd_error=='RGB-D test failure' and state.stopped
