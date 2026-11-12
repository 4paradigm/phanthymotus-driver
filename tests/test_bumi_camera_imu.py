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
                       get_extrinsics_to=lambda depth, kind=kind:NS(rotation=[1,0,0,0,1,0,0,0,1],
                           translation=[.012, -.034, .056] if kind=='gyro' else [-.078, .09, -.011]))
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
        def set_enabled(self,cards):pass
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
    assert packet['camera_from_gyro_translation']==[.012, -.034, .056]
    assert packet['camera_from_accel_translation']==[-.078, .09, -.011]
    assert packet['translation_source']=='sdk_factory_extrinsics'
    assert packet['stamp_ns']==1_000_000_000+offset
    assert packet['clock_mapping']['offset_ns']==offset
    assert packet['clock_mapping']['verified'] is False
    assert len(state.published['/robot/camera/color'])==1
    assert not state.published.get('/robot/camera/depth')
    assert state.rgbd_error=='RGB-D test failure' and state.stopped


def test_imu_card_keeps_body_json_and_reports_camera_disabled(monkeypatch):
    device=fixture._load_device(monkeypatch)
    state=device.StatePlugin.__new__(device.StatePlugin)
    state._namespace='robot'
    state._camera_plugin=None
    state._node=NS(_imu_enabled=True)
    state._node.set_imu_enabled=lambda enabled:setattr(state._node,'_imu_enabled',enabled)
    ports=next(t['topic_out'] for t in state.get_tools() if t['name']=='imu')
    assert ports==[
        {'topic':'/robot/state/imu','format':'data/json'},
        {'name':'camera_imu','topic':'/robot/camera/rgbd/imu','format':'sensor/imu','optional':True}]
    result=state.dispatch('info',{'_tool_name':'imu'})
    assert result['imu_status']['state']=='unavailable'
    assert 'disabled' in result['imu_status']['reason']
    assert result['state']=='running'
    assert state.dispatch('stop',{'_tool_name':'imu'})['state']=='idle'
    assert state._node._imu_enabled is False
    assert state.dispatch('start',{'_tool_name':'imu'})['state']=='running'


def test_imu_card_only_controls_camera_motion_lease(monkeypatch):
    device=fixture._load_device(monkeypatch)
    state=device.StatePlugin.__new__(device.StatePlugin)
    state._namespace='robot'
    state._node=NS(_imu_enabled=True)
    state._node.set_imu_enabled=lambda enabled:setattr(state._node,'_imu_enabled',enabled)
    calls=[]
    state._camera_plugin=NS(start_card=lambda name:calls.append(('start',name)),
        stop_card=lambda name:calls.append(('stop',name)),
        camera_imu_info=lambda:dict(imu_status=dict(state='unavailable',reason='motion unsupported')))
    for action in ('stop','start','info'):
        response=state.dispatch(action,{'_tool_name':'imu'})
        assert response['imu_status']['reason']=='motion unsupported'
    assert calls==[('stop','imu'),('start','imu')]
