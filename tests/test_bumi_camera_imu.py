"""Exercise Bumi's real capture-owner callback boundary without ROS/hardware."""
import importlib.util
import json
import sys
import threading
import time
from pathlib import Path
from types import SimpleNamespace as NS

import yaml

ROOT=Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('bumi_imu_capture_fixture',ROOT/'tests/bumi_visual_odometry/test_bundle_hook.py')
fixture=importlib.util.module_from_spec(spec)
spec.loader.exec_module(fixture)
capture=fixture.capture


def test_capture_uses_body_imu_without_discovering_or_enabling_camera_motion(capture, monkeypatch):
    from test_bumi_body_imu import reading
    config=yaml.safe_load((ROOT/'noetix/bumi/config.yaml').read_text())
    mount=config['plugins']['visual_odometry']['calibration']
    device,state=capture
    sdk=sys.modules['pyrealsense2']
    probes=[]
    sdk.pipeline_wrapper=lambda pipeline:probes.append(pipeline)
    monkeypatch.setenv('BUMI_BODY_IMU_MOUNT',json.dumps(mount))
    state.frame_limit=2
    imu_threads = []
    def published(topic, _):
        if topic == '/robot/state/imu_calibrated':
            imu_threads.append(threading.current_thread())

    state.on_publish = published

    def frame_boundary():
        if state.iterations == 2:
            state.send_imu(NS(data=json.dumps(reading(10_030_000_000))))

    state.before_frame = frame_boundary
    device._camera_subprocess('robot')
    packet=json.loads(state.published['/robot/state/imu_calibrated'][0].data)
    assert not {'source_id','epoch','clock_id','translation_source'} & packet.keys()
    assert packet['stamp_source'] == 'read_completion'
    assert packet['clock_mapping']['verified'] is False
    assert packet['camera_frame_id']=='robot_rgb_optical'
    assert '/robot/state/imu' not in state.node.subscriptions
    assert len(imu_threads) == 1 and imu_threads[0] is not threading.current_thread()
    assert state.imu_node.destroyed and not imu_threads[0].is_alive()
    assert packet['angular_velocity']==[1.,2.,3.]
    assert packet['camera_from_gyro_translation'] == mount['body_imu_translation']['gyro']
    assert packet['camera_from_accel_translation'] == mount['body_imu_translation']['accel']
    assert probes==[]
    assert {args[0] for args in state.streams}=={'color','depth'}
    assert '/robot/camera/rgbd/imu' not in state.publisher_topics
    assert state.published['/robot/camera/color'] and state.published['/robot/camera/depth']


def test_imu_card_keeps_body_json_and_reports_adapter_disabled(monkeypatch):
    device=fixture._load_device(monkeypatch)
    state=device.StatePlugin.__new__(device.StatePlugin)
    state._namespace='robot'
    state._camera_plugin=None
    state._node=NS(_imu_enabled=True)
    state._node.set_imu_enabled=lambda enabled:setattr(state._node,'_imu_enabled',enabled)
    ports=next(t['topic_out'] for t in state.get_tools() if t['name']=='imu')
    assert ports==[
        {'topic':'/robot/state/imu','format':'data/json'},
        {'name':'body_imu','topic':'/robot/state/imu_calibrated','format':'sensor/imu','optional':True}]
    result=state.dispatch('info',{'_tool_name':'imu'})
    assert result['imu_status']['state']=='unavailable'
    assert 'disabled' in result['imu_status']['reason']
    assert result['state']=='running'
    assert state.dispatch('stop',{'_tool_name':'imu'})['state']=='idle'
    assert state._node._imu_enabled is False
    assert state.dispatch('start',{'_tool_name':'imu'})['state']=='running'


def test_imu_card_only_controls_body_adapter_lease(monkeypatch):
    device=fixture._load_device(monkeypatch)
    state=device.StatePlugin.__new__(device.StatePlugin)
    state._namespace='robot'
    state._node=NS(_imu_enabled=True)
    state._node.set_imu_enabled=lambda enabled:setattr(state._node,'_imu_enabled',enabled)
    calls=[]
    state._camera_plugin=NS(start_card=lambda name:calls.append(('start',name)),
        stop_card=lambda name:calls.append(('stop',name)),
        body_imu_info=lambda:dict(imu_status=dict(state='unavailable',reason='body adapter unavailable')))
    for action in ('stop','start','info'):
        response=state.dispatch(action,{'_tool_name':'imu'})
        assert response['imu_status']['reason']=='body adapter unavailable'
    assert calls==[('stop','imu'),('start','imu')]
