"""Offline checks of the G1 computational RGB-D source (no ROS or hardware)."""
import ast
import importlib.util
from pathlib import Path
from types import SimpleNamespace as NS
import zlib

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]


def source_module():
    path = ROOT / 'rgbd_camera.py'
    assert path.exists(), 'G1 must expose a gated computational RGB-D source'
    spec = importlib.util.spec_from_file_location('g1_rgbd_camera', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class Frame:
    def __init__(self, data, stamp=1700000000123., domain='global'):
        self.data, self.stamp, self.domain = data, stamp, domain
    def get_data(self): return self.data
    def get_timestamp(self): return self.stamp
    def get_frame_timestamp_domain(self): return self.domain


class Frames:
    def __init__(self, color=None, depth=None):
        self.color = color or Frame(np.full((480, 640, 3), 125, np.uint8))
        self.depth = depth or Frame(np.full((480, 640), 4000, np.uint16), 1700000000128.)
    def get_color_frame(self): return self.color
    def get_depth_frame(self): return self.depth


def configured(model='none', coeffs=None):
    module = source_module()
    intr = NS(width=640, height=480, fx=400., fy=410., ppx=320., ppy=240.,
              model=model, coeffs=coeffs or [0.] * 5)
    aligned = []
    aligner = NS(process=lambda fs: aligned.append(fs) or fs)
    rs = NS(stream=NS(color='color', depth='depth'), timestamp_domain=NS(global_time='global'),
            distortion=NS(none='none', brown_conrady='brown', inverse_brown_conrady='inverse',
                          modified_brown_conrady='modified'),
            align=lambda target: aligner if target == 'depth' else pytest.fail('align color to measured depth optical plane'))
    profile = NS(get_stream=lambda _: NS(as_video_stream_profile=lambda: NS(get_intrinsics=lambda: intr)),
                 get_device=lambda: NS(first_depth_sensor=lambda: NS(get_depth_scale=lambda: .00025)))
    source = module.RgbdSource('/g1/camera/rgbd', rs)
    source.configure(profile)
    return source, aligned, intr


def test_g1_source_declares_only_verified_capture_geometry_and_encodes_measured_mm():
    source, aligned, _ = configured()
    assert source.info()['camera_info'] == []
    frames = Frames()
    color, depth = source.encode(frames, 1700000000130000000)
    assert aligned == [frames]  # SDK alignment is required, equal frame names are insufficient.
    assert color['stamp_ns'] == 1700000000123000000
    assert depth['stamp_ns'] == 1700000000128000000
    assert color['data'].startswith(b'\xff\xd8')
    measured = np.frombuffer(zlib.decompress(depth['data']), dtype='<u2').reshape(480, 640)
    assert np.all(measured == 1000)  # SDK scale is 0.25 mm, not an assumed millimetre.
    declarations = source.info()['camera_info']
    assert {d['format'] for d in declarations} == {'image/jpeg', 'image/depth-zlib'}
    a, b = declarations
    assert a['id'] == b['id'] and a['K'] == b['K']
    assert a['width'] == 640 and a['height'] == 480
    assert a['D'] == [0.] * 5 and a['distortion_model'] == 'pinhole'
    assert a['vendor']['registration'] == 'registered_rectified'
    assert a['vendor']['timestamp_semantics'] == 'source_capture_mapped'
    assert a['vendor']['clock_id'] == b['vendor']['clock_id']
    assert b['vendor']['depth_source'] == 'measured' and b['vendor']['depth_unit'] == 'mm'
    assert color['frame_id'] == depth['frame_id'] == a['vendor']['frame_id']


@pytest.mark.parametrize('domain,stamp,reason', [('hardware',1700000000123.,'global'),
                                              ('global',0.,'capture'),
                                              ('global',1699999998000.,'capture')])
def test_g1_never_replaces_unmapped_or_stale_capture_with_receipt_time(domain, stamp, reason):
    source, _, _ = configured()
    frames = Frames(color=Frame(np.zeros((480,640,3),np.uint8),stamp,domain))
    with pytest.raises(ValueError, match=reason):
        source.encode(frames, 1700000000130000000)
    assert source.info()['camera_info'] == []


def test_g1_rejects_repeated_frames_and_withdraws_ready_declaration():
    source, _, _ = configured()
    source.encode(Frames(), 1700000000130000000)
    with pytest.raises(ValueError, match='capture'):
        source.encode(Frames(), 1700000000130000000)
    assert source.info()['state'] == 'unavailable'
    assert source.info()['camera_info'] == []


def test_g1_nonzero_brown_rectifies_both_streams_with_the_same_map():
    source, _, intr = configured('brown', [.1, 0, 0, 0, 0])
    # At x=500, the source sample shifts right; nearest-neighbour depth must follow.
    frame = Frames(depth=Frame(np.tile(np.arange(640,dtype=np.uint16),(480,1)),1700000000128.))
    _, depth = source.encode(frame, 1700000000130000000)
    values = np.frombuffer(zlib.decompress(depth['data']),dtype='<u2').reshape(480,640)
    assert values[240,500] > 125


def test_g1_unsupported_distortion_and_invalid_calibration_are_unavailable():
    with pytest.raises(ValueError, match='distortion'):
        configured('inverse', [.1,0,0,0,0])
    source, _, intr = configured()
    intr.fx = float('nan')
    # A changed SDK profile must be revalidated.
    profile = NS(get_stream=lambda _: NS(as_video_stream_profile=lambda: NS(get_intrinsics=lambda: intr)))
    with pytest.raises(ValueError, match='intrinsics'):
        source.configure(profile)


def test_g1_source_declares_the_original_shared_depth_topic():
    source, _, _ = configured()
    source.encode(Frames(), 1700000000130000000)
    assert source.info()['topic_out'] == [
        {'topic': '/g1/camera/rgbd/color', 'format': 'image/jpeg'},
        {'topic': '/g1/camera/depth', 'format': 'image/depth-zlib'}]
    assert source.info()['camera_info'][1]['topic'] == '/g1/camera/depth'


def test_g1_parent_reports_sdk_failure_and_withdraws_stale_geometry():
    import queue
    import time
    tree = ast.parse((ROOT/'device.py').read_text())
    node = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'RealSensePlugin')
    namespace = {'queue':queue, 'time':time}
    exec(compile(ast.Module(body=[node],type_ignores=[]),str(ROOT/'device.py'),'exec'),namespace)
    plugin = namespace['RealSensePlugin'].__new__(namespace['RealSensePlugin'])
    plugin._namespace = 'g1'
    plugin._proc = NS(is_alive=lambda: True)
    plugin._rgbd_status_queue = queue.Queue()
    plugin._rgbd_status = None
    plugin._rgbd_status_queue.put(dict(state='unavailable',reason='unsupported RealSense depth distortion model',
                                       camera_info=[],last_capture_ns=0))
    assert plugin._rgbd_info()['reason'] == 'unsupported RealSense depth distortion model'
    source, _, _ = configured()
    source.encode(Frames(), 1700000000130000000)
    plugin._rgbd_status_queue.put(source.info())
    assert plugin._rgbd_info()['camera_info'] == []  # Historical captures cannot leave calibration ready.


@pytest.mark.parametrize('model', ['inverse', 'modified'])
def test_g1_zero_coefficient_depth_models_are_pinhole(model):
    source, _, _ = configured(model)
    source.encode(Frames(), 1700000000130000000)
    assert source.info()['state'] == 'ready'


def test_single_capture_profile_feeds_display_calibration_and_computational_source(monkeypatch):
    import sys
    tree = ast.parse((ROOT/'device.py').read_text())
    owner = next(n for n in ast.walk(tree) if isinstance(n, ast.ClassDef) and n.name == '_RealSenseNode')
    method = next(n for n in owner.body if isinstance(n, ast.FunctionDef) and n.name == 'start_capture')
    profile, calls = object(), []
    device = NS(get_info=lambda _: 'camera-123', query_sensors=lambda: [])
    pipeline = NS()
    config = NS(enable_device=lambda _: None, enable_stream=lambda *args: None)
    rs = NS(context=lambda: NS(query_devices=lambda: [device]),
            camera_info=NS(serial_number='serial'), pipeline=lambda: pipeline,
            config=lambda: config, stream=NS(depth='depth', color='color'),
            format=NS(z16='z16', bgr8='bgr8'))

    def start_motion(sdk, pipe, settings, actual_device, callback):
        assert (sdk, pipe, settings, actual_device) == (rs, pipeline, config, device)
        calls.append(('capture', callback))
        return profile, None

    def stream_profiles(active, sdk):
        assert active is profile and sdk is rs
        return {'depth': {'fx': 420.1}, 'color': {'fx': 1020.1}}

    monkeypatch.setitem(sys.modules, 'camera_specs', NS(active_stream_profiles=stream_profiles))
    threads = NS(Thread=lambda **kwargs: NS(start=lambda: None))
    scope = dict(rs=rs, threading=threads, start_optional_motion=start_motion,
                 calibration_sender=NS(send=lambda value: calls.append(('calibration', value))),
                 RS_DEPTH_W=640, RS_DEPTH_H=480, RS_DEPTH_FPS=15,
                 RS_COLOR_W=1920, RS_COLOR_H=1080, RS_COLOR_FPS=15)
    exec(compile(ast.Module(body=[method], type_ignores=[]), str(ROOT/'device.py'), 'exec'), scope)
    node = NS(_pipeline=None, _worker_stop=NS(clear=lambda: None),
              _color_loop=lambda: None, _rgbd_loop=lambda: None,
              _on_frame=lambda frame: None, _rgbd_configured=False,
              _imu=NS(configure=lambda active: calls.append(('imu', active))),
              _rgbd=NS(configure=lambda active: calls.append(('rgbd', active))),
              get_logger=lambda: NS(info=lambda _: None, warning=lambda _: None))
    scope['start_capture'](node)
    assert calls == [('capture', node._on_frame), ('imu', profile), ('rgbd', profile),
                     ('calibration', {'depth': {'fx': 420.1}, 'color': {'fx': 1020.1}})]
    assert node._pipeline is pipeline and node._rgbd_configured


class Mount:
    def __init__(self):
        self.calls = []
        self.error = False
    def publish(self, stamp):
        self.calls.append(('publish', stamp))
        if self.error:
            raise RuntimeError('TF lookup failed')
        return True
    def declaration(self):
        return dict(schema='motus.camera_mount/1', topic='/robot/camera/rgbd/body_mount',
                    parent_frame='pelvis', child_frame='camera_depth_optical_frame', status='available',
                    source='tf', version='capture_tf_v1', epoch='test-epoch', mode='dynamic',
                    length_unit='m', transform_direction='body_from_camera',
                    reason=None, accuracy_validated=False)
    def status(self):
        return dict(status='available', parent_frame='pelvis')
    def reset(self): self.calls.append(('reset',))
    def close(self): self.calls.append(('close',))


def test_g1_mount_uses_depth_capture_and_declarations_independently_of_rgbd_ready():
    source, _, _ = configured()
    mount = Mount()
    source.body_mount = mount
    assert source.info()['body_mount_status'] == mount.status()
    source.encode(Frames(), 1700000000130000000)
    assert mount.calls == [('publish', 1700000000128000000)]
    a, b = source.info()['camera_info']
    assert a['vendor']['body_mount'] == b['vendor']['body_mount'] == mount.declaration()


def test_g1_mount_provider_failure_preserves_rgbd_and_reports_failure():
    source, _, _ = configured()
    source.body_mount = Mount()
    source.body_mount.error = True
    pair = source.encode(Frames(), 1700000000130000000)
    assert pair[1]['stamp_ns'] == 1700000000128000000
    assert source.info()['state'] == 'ready'
    assert source.info()['body_mount_status']['status'] == 'unavailable'
    assert 'TF lookup failed' in source.info()['body_mount_status']['reason']
    assert source.info()['camera_info'][0]['vendor']['body_mount']['status'] == 'unavailable'


def test_g1_mount_resets_with_capture_configuration_and_closes():
    source, _, intr = configured()
    source.body_mount = Mount()
    profile = NS(get_stream=lambda _: NS(as_video_stream_profile=lambda: NS(get_intrinsics=lambda: intr)),
                 get_device=lambda: NS(first_depth_sensor=lambda: NS(get_depth_scale=lambda: .001)))
    source.configure(profile)
    source.close()
    assert source.body_mount.calls == [('reset',), ('close',)]


@pytest.mark.parametrize('override,expected', [(None, 'pelvis'), ('torso_link', 'torso_link')])
def test_g1_existing_ros_capture_owner_constructs_mount_with_optical_clock(monkeypatch, override, expected):
    import os
    import queue
    import threading
    if override is None: monkeypatch.delenv('CAMERA_BODY_FRAME', raising=False)
    else: monkeypatch.setenv('CAMERA_BODY_FRAME', override)
    tree = ast.parse((ROOT/'device.py').read_text())
    owner = next(n for n in ast.walk(tree) if isinstance(n, ast.ClassDef) and n.name == '_RealSenseNode')
    init = next(n for n in owner.body if isinstance(n, ast.FunctionDef) and n.name == '__init__')
    init.name = 'initialize'
    calls = []
    topics = []
    class Node:
        def __init__(self, *args): pass
        def create_publisher(self, cls, topic, qos):
            topics.append(topic)
            return NS()
        def get_logger(self): return NS(info=lambda _: None)
    class Owner(Node):
        def _publish_imu(self, packet): pass
        def _publish_rgbd_status(self): pass
    def mount(node, root, **kwargs):
        calls.append((node, root, kwargs))
        return Mount()
    scope = dict(super=lambda: NS(__init__=lambda *args: None), queue=queue, threading=threading,
                 namespace='g1', os=os, BodyMountSource=mount, RgbdSource=source_module().RgbdSource,
                 rs=object(), MotionSource=lambda *args, **kwargs: NS(),
                 CompressedImage=object, _String=object, _QOS=object())
    import sys
    monkeypatch.setitem(sys.modules, 'sensor_msgs.msg', NS(CompressedImage=object))
    exec(compile(ast.Module(body=[init], type_ignores=[]), str(ROOT/'device.py'), 'exec'), scope)
    node = Owner()
    scope['initialize'](node, '/color', '/depth', '/dist')
    assert calls == [(node, '/g1/camera/rgbd', dict(body_frame=expected,
                      camera_frame='camera_depth_optical_frame', clock_id='realsense_sdk_global_time_unix'))]
    assert node._rgbd.body_mount.declaration()['parent_frame'] == 'pelvis'
    assert topics.count('/depth') == 1
    assert '/g1/camera/rgbd/depth' not in topics
    assert node._rgbd.depth_topic == '/depth'


def actual_mount(monkeypatch, path, parent, child, clock, events):
    """Load the shipped provider, substituting only unavailable ROS transport."""
    import json
    import sys
    spec = importlib.util.spec_from_file_location('tested_camera_mount', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    class Buffer:
        def __init__(self): self.missing = False
        def lookup_transform(self, target, source, capture):
            events.append(('lookup', target, source, capture.nanoseconds))
            if self.missing: raise LookupError(f'{target} <- {source} missing at capture')
            return NS(header=NS(frame_id=target, stamp=NS(
                          sec=capture.nanoseconds // 10**9, nanosec=capture.nanoseconds % 10**9)), child_frame_id=source,
                      transform=NS(translation=NS(x=.2, y=-.1, z=.3),
                                   rotation=NS(x=0., y=0., z=0., w=1.)))
    class Listener:
        def __init__(self, *args): pass
        def unregister(self): events.append(('unregister',))
    monkeypatch.setitem(sys.modules, 'std_msgs.msg', NS(String=lambda **kwargs: NS(**kwargs)))
    monkeypatch.setitem(sys.modules, 'rclpy.qos', NS(qos_profile_sensor_data=object()))
    monkeypatch.setitem(sys.modules, 'rclpy.time', NS(Time=lambda **kwargs: NS(**kwargs)))
    monkeypatch.setitem(sys.modules, 'rclpy.clock', NS(ClockType=NS(ROS_TIME='ros')))
    monkeypatch.setitem(sys.modules, 'tf2_ros', NS(Buffer=Buffer, TransformListener=Listener))
    node = NS(create_publisher=lambda cls, topic, qos: NS(
                  publish=lambda message: events.append(('packet', json.loads(message.data)))),
              destroy_publisher=lambda _: events.append(('destroy_publisher',)))
    return module.BodyMountSource(node, '/robot/camera/rgbd', body_frame=parent,
                                  camera_frame=child, clock_id=clock)


def test_g1_real_provider_receives_exact_depth_ns_and_tf_failure_keeps_images(monkeypatch):
    source, _, _ = configured()
    events = []
    mount = actual_mount(monkeypatch, ROOT.parents[1] / 'common/camera_mount.py',
                         'pelvis', 'camera_depth_optical_frame', 'realsense_sdk_global_time_unix', events)
    source.body_mount = mount
    pair = source.encode(Frames(), 1700000000130000000)
    assert events[0] == ('lookup', 'pelvis', 'camera_depth_optical_frame', pair[1]['stamp_ns'])
    assert events[1][1]['status'] == 'available'
    assert events[1][1]['matrix'][0][3] == .2
    assert events[1][1]['clock_id'] == 'realsense_sdk_global_time_unix'
    assert events[1][1]['stamp_ns'] == pair[1]['stamp_ns']
    assert source.info()['camera_info'][0]['vendor']['body_mount'] == mount.declaration()
    mount.buffer.missing = True
    later = Frames(Frame(np.zeros((480,640,3),np.uint8),1700000000153.),
                   Frame(np.full((480,640),4000,np.uint16),1700000000158.))
    assert source.encode(later, 1700000000160000000)[1]['data']
    assert events[-1][1]['status'] == 'unavailable'
    assert source.info()['state'] == 'ready'
    assert source.info()['body_mount_status']['status'] == 'unavailable'
    source.close()
    assert events[-2:] == [('unregister',), ('destroy_publisher',)]
