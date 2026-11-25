"""Offline RealSense motion contract for the G1 capture owner."""
import ast
import importlib
import sys
from pathlib import Path
from types import SimpleNamespace as NS

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT))


def helper():
    assert (ROOT/'common/realsense_imu.py').exists(), 'shared camera motion adapter missing'
    return importlib.import_module('common.realsense_imu')


def sdk():
    return NS(stream=NS(gyro='gyro', accel='accel', depth='depth'),
              format=NS(motion_xyz32f='xyz'), camera_info=NS(serial_number='serial'))


MISSING = object()


def profile(kind, fps=100, rotation=(0, 1, 0, -1, 0, 0, 0, 0, 1), translation=MISSING):
    extrinsics = NS(rotation=rotation)
    if translation is not MISSING:
        extrinsics.translation = translation
    return NS(stream_type=lambda:kind, format=lambda:'xyz', fps=lambda:fps,
              stream_index=lambda:0, unique_id=lambda:kind+'-17',
              get_extrinsics_to=lambda target:extrinsics)


def device(profiles):
    return NS(query_sensors=lambda:[NS(get_stream_profiles=lambda:profiles)],
              get_info=lambda field:'camera-123')


def test_factory_rotation_converts_sdk_column_major_and_preserves_raw_vectors():
    h = helper()
    p = [profile('gyro'), profile('accel')]
    active = NS(get_stream=lambda kind: next((x for x in p if x.stream_type()==kind), NS()),
                get_device=lambda:device(p))
    source = h.MotionSource('/cam/imu', 'depth_optical', sdk(), lambda ms, domain: int(ms*1e6), lambda packet:None, clock_id=lambda:'rgbd-clock')
    source.configure(active)
    packet = source.accept(motion('accel', 1000, [0, 9.81, 0]))
    assert packet is None
    packet = source.accept(motion('gyro', 1005, [1, 2, 3]))
    r = packet['camera_from_gyro']
    assert np.allclose(np.asarray(r)@[1,0,0], [0,1,0])
    assert packet['angular_velocity'] == [1,2,3]
    assert packet['linear_acceleration'] == [0,9.81,0]
    assert packet['stamp_ns']==1005000000 and packet['accel_stamp_ns']==1000000000
    assert packet['clock_id']=='rgbd-clock' and packet['camera_frame_id']=='depth_optical'
    assert packet['rotation_source']=='sdk_factory_extrinsics'
    assert packet['schema']=='motus.imu/1' and packet['stamp_source']=='measurement'
    assert packet['units']=={'angular_velocity':'rad/s','linear_acceleration':'m/s^2'}
    assert packet['gyro_frame_id'] != packet['accel_frame_id']
    assert 'orientation' not in packet
    epoch=packet['epoch']
    source.configure(active)
    source.accept(motion('accel',2000,[0,9.81,0]))
    assert source.accept(motion('gyro',2005,[0,0,0]))['epoch'] != epoch


def motion(kind, stamp, xyz, domain='hardware_clock'):
    return NS(is_motion_frame=lambda:True, as_motion_frame=lambda:NS(
        profile=profile(kind), get_timestamp=lambda:stamp,
        get_frame_timestamp_domain=lambda:domain,
        get_motion_data=lambda:NS(x=xyz[0],y=xyz[1],z=xyz[2])))


def test_unsupported_d435_does_not_enable_motion_and_only_starts_video():
    h=helper(); calls=[]
    config=NS(enable_stream=lambda *a:calls.append(a))
    pipe=NS(start=lambda *a:calls.append('start') or 'active')
    active, reason=h.start_optional_motion(sdk(),pipe,config,device([]),lambda f:None)
    assert active=='active' and 'unsupported' in reason and calls==['start']


def test_motion_negotiation_failure_retries_same_owner_video_only():
    h=helper(); calls=[]; attempts=[]
    config=NS(enable_stream=lambda *a:calls.append(('enable',a)), disable_stream=lambda s:calls.append(('disable',s)))
    def start(*args):
        attempts.append(args)
        if len(attempts)==1: raise RuntimeError('motion USB bandwidth')
        return 'video active'
    active, reason=h.start_optional_motion(sdk(),NS(start=start,stop=lambda:None),config,
                                            device([profile('gyro'),profile('accel')]),lambda f:None)
    assert active=='video active' and 'motion USB bandwidth' in reason
    assert len(attempts)==2
    assert {x[1] for x in calls if x[0]=='disable'}=={'gyro','accel'}


def test_invalid_factory_axes_and_bad_capture_time_are_unavailable():
    h=helper()
    p=[profile('gyro',rotation=(1,0,0,0,1,0,0,0,-1)),profile('accel')]
    active=NS(get_stream=lambda kind: next((x for x in p if x.stream_type()==kind), NS()),get_device=lambda:device(p))
    source=h.MotionSource('/imu','optical',sdk(),lambda ms,domain:None,lambda packet:None, clock_id=lambda:'rgbd-clock')
    with pytest.raises(ValueError,match='rotation'):source.configure(active)
    assert source.status()['state']=='unavailable'


def test_motion_callback_handles_composites_without_eating_video_or_raising():
    h=helper(); seen=[]
    source=h.MotionSource('/imu','optical',sdk(),lambda ms,domain:int(ms*1e6),seen.append, clock_id=lambda:'rgbd-clock')
    p=[profile('gyro'),profile('accel')]
    source.configure(NS(get_stream=lambda kind:next((x for x in p if x.stream_type()==kind), NS()),get_device=lambda:device(p)))
    fs=NS(is_motion_frame=lambda:False,is_frameset=lambda:True,
          as_frameset=lambda:[motion('accel',1000,[0,9.81,0]),motion('gyro',1005,[1,2,3])])
    source.on_frame(fs)
    assert len(seen)==1
    source.on_frame(motion('gyro',1006,[float('nan'),0,0]))
    assert len(seen)==1 and source.status()['state']=='unavailable'


def test_g1_card_declares_optional_json_imu_port_and_status():
    h=helper()
    for rel, cls in [('unitree/g1/device.py','RealSensePlugin')]:
        tree=ast.parse((ROOT/rel).read_text())
        node=next(x for x in tree.body if isinstance(x,ast.ClassDef) and x.name==cls)
        text=ast.unparse(node)
        assert 'imu_status' in text and 'sensor/imu' in text
        assert 'common.realsense_imu' in (ROOT/rel).read_text()


def load_camera_class(relative, name):
    tree=ast.parse((ROOT/relative).read_text())
    node=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name==name)
    import math, time
    scope={'math':math,'time':time}
    exec(compile(ast.Module(body=[node],type_ignores=[]),relative,'exec'),scope)
    return scope[name]


def test_g1_motion_uses_same_global_capture_conversion_and_refuses_hardware_clock():
    cls=load_camera_class('unitree/g1/rgbd_camera.py','RgbdSource')
    source=cls('/g1/camera/rgbd',None)
    import time
    now=time.time_ns()
    stamp=now/1e6
    assert abs(source.motion_stamp(stamp,'global_time')-now)<500
    with pytest.raises(ValueError,match='global'):source.motion_stamp(stamp,'hardware_clock')


def test_video_callback_queue_stays_bounded_and_dispatches_motion_separately():
    h=helper(); seen=[]
    capture=h.CaptureQueue(NS(on_frame=seen.append))
    frames=[]
    for number in range(4):
        fs=NS(number=number,get_color_frame=lambda:True,get_depth_frame=lambda:True)
        frames.append(NS(is_frameset=lambda:True,as_frameset=lambda fs=fs:fs))
        capture(frames[-1])
    assert len(seen)==4
    assert capture.frames.qsize()==1
    assert capture.wait_for_frames(10).number==3
    capture(NS(is_frameset=lambda:False))
    assert capture.frames.empty()


def test_missing_accel_profile_never_requests_a_partial_motion_capture():
    h=helper(); calls=[]
    config=NS(enable_stream=lambda *a:calls.append(a))
    result,reason=h.start_optional_motion(sdk(),NS(start=lambda *a:'video'),config,
                                         device([profile('gyro')]),lambda frame:None)
    assert result=='video' and 'unsupported' in reason and not calls


def test_bad_motion_timestamp_and_clock_reset_do_not_publish():
    h=helper(); packets=[]
    source=h.MotionSource('/imu','optical',sdk(),lambda ms,domain:int(ms*1e6),packets.append,
                          clock_id=lambda:'rgbd-clock')
    p=[profile('gyro'),profile('accel')]
    source.configure(NS(get_stream=lambda kind:next((x for x in p if x.stream_type()==kind),NS()),get_device=lambda:device(p)))
    source.on_frame(motion('accel',1000,[0,9.81,0]))
    source.on_frame(motion('gyro',1005,[0,0,0]))
    source.on_frame(motion('gyro',999,[0,0,0]))
    source.on_frame(motion('accel',float('nan'),[0,9.81,0]))
    assert len(packets)==1 and source.status()['state']=='unavailable'


def test_g1_motion_callback_preserves_partial_video_display_frames():
    import queue
    tree=ast.parse((ROOT/'unitree/g1/device.py').read_text())
    owner=next(n for n in ast.walk(tree) if isinstance(n,ast.ClassDef) and n.name=='_RealSenseNode')
    method=next(n for n in owner.body if isinstance(n,ast.FunctionDef) and n.name=='_on_frame')
    import time
    scope={'queue':queue,'time':time,'np':np,'RS_DIST_INTERVAL':.1,'card_epochs':None}
    exec(compile(ast.Module(body=[method],type_ignores=[]),'g1_callback','exec'),scope)
    errors=[]
    node=NS(_imu=NS(on_frame=lambda frame:None),_rgbd_configured=False,
            _color_q=queue.Queue(maxsize=1),
            _last_dist_time=time.monotonic(),_last_ts=0.,
            get_clock=lambda:NS(now=lambda:NS(to_msg=lambda:None)),
            get_logger=lambda:NS(error=errors.append))
    pixels=np.ones((2,2,3),np.uint8)
    fs=NS(get_color_frame=lambda:NS(get_data=lambda:pixels),get_depth_frame=lambda:None)
    scope['_on_frame'](node,NS(is_frameset=lambda:True,as_frameset=lambda:fs))
    assert not errors and node._color_q.qsize()==1



def configured_motion(profiles):
    h = helper()
    source = h.MotionSource('/cam/imu', 'depth_optical', sdk(), lambda ms, domain:int(ms*1e6),
                            lambda packet:None, clock_id=lambda:'rgbd-clock')
    active = NS(get_stream=lambda kind:next((p for p in profiles if p.stream_type()==kind), NS()),
                get_device=lambda:device(profiles))
    source.configure(active)
    return source, active


def test_factory_translation_is_sensor_origin_in_depth_axes_without_inversion():
    source, _ = configured_motion([profile('gyro', translation=[.012, -.034, .056]),
                                   profile('accel', translation=[-.078, .09, -.011])])
    source.accept(motion('accel', 1000, [0, 9.81, 0]))
    packet = source.accept(motion('gyro', 1005, [1, 2, 3]))
    assert packet['camera_from_gyro_translation'] == [.012, -.034, .056]
    assert packet['camera_from_accel_translation'] == [-.078, .09, -.011]
    assert packet['translation_source'] == 'sdk_factory_extrinsics'
    assert source.status()['fusion']['state'] == 'available'


@pytest.mark.parametrize('translation', [MISSING, None, [1., 2.], [float('nan'), 0., 0.],
                                        [0., float('inf'), 0.]])
def test_missing_or_invalid_factory_translation_keeps_attitude_without_calibrated_translation(translation):
    source, _ = configured_motion([profile('gyro', translation=[.01, .02, .03]),
                                   profile('accel', translation=translation)])
    source.accept(motion('accel', 1000, [0, 9.81, 0]))
    packet = source.accept(motion('gyro', 1005, [1, 2, 3]))
    assert packet['angular_velocity'] == [1., 2., 3.]
    assert packet['rotation_source'] == 'sdk_factory_extrinsics'
    assert 'camera_from_gyro_translation' not in packet
    assert 'camera_from_accel_translation' not in packet
    assert 'translation_source' not in packet
    assert source.status()['fusion']['state'] == 'unavailable'
    assert 'translation' in source.status()['fusion']['reason']


def test_reconfigure_removes_old_factory_translation_and_pairs_only_new_epoch_samples():
    profiles = [profile('gyro', translation=[.01, .02, .03]), profile('accel', translation=[.04, .05, .06])]
    source, active = configured_motion(profiles)
    source.accept(motion('accel', 1000, [0, 9.81, 0]))
    first = source.accept(motion('gyro', 1005, [1, 2, 3]))
    profiles[1] = profile('accel')
    source.configure(active)
    with pytest.raises(ValueError, match='accel'):
        source.accept(motion('gyro', 2000, [0, 0, 0]))
    source.accept(motion('accel', 2001, [0, 9.81, 0]))
    resumed = source.accept(motion('gyro', 2005, [0, 0, 0]))
    assert resumed['epoch'] != first['epoch']
    assert 'camera_from_accel_translation' not in resumed
    assert source.status()['fusion']['state'] == 'unavailable'



def test_factory_rotation_helper_keeps_profile_to_depth_call_contract():
    depth = object()
    def extrinsics_to(target):
        assert target is depth
        return NS(rotation=[0, 1, 0, -1, 0, 0, 0, 0, 1])
    result = helper().factory_rotation(NS(get_extrinsics_to=extrinsics_to), depth)
    assert result == [[0., -1., 0.], [1., 0., 0.], [0., 0., 1.]]
