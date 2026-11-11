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


def test_g1_rgbd_card_is_registered_in_existing_single_capture_owner():
    tree = ast.parse((ROOT/'device.py').read_text())
    plugin = next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='RealSensePlugin')
    method = next(n for n in plugin.body if isinstance(n,ast.FunctionDef) and n.name=='get_tools')
    assert '_rgbd_tool' in ast.unparse(method)
    assert 'camera_rgbd' in (ROOT/'driver.yaml').read_text()
    assert 'COPY rgbd_camera.py' in (ROOT/'Dockerfile').read_text()


def test_g1_rgbd_info_dispatch_uses_live_status_and_preserves_display_ports():
    tree = ast.parse((ROOT/'device.py').read_text())
    plugin_ast = next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='RealSensePlugin')
    namespace = {}
    exec(compile(ast.Module(body=[plugin_ast],type_ignores=[]),str(ROOT/'device.py'),'exec'),namespace)
    plugin = namespace['RealSensePlugin'].__new__(namespace['RealSensePlugin'])
    plugin._rgbd_info = lambda: {'state':'unavailable','reason':'SDK global capture clock unavailable'}
    assert plugin.dispatch('info', {'_tool_name':'camera_rgbd'})['reason'] == 'SDK global capture clock unavailable'


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


def test_first_rgbd_start_waits_for_source_metadata():
    import time
    tree=ast.parse((ROOT/'device.py').read_text())
    node=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='RealSensePlugin')
    namespace={'time':time}
    exec(compile(ast.Module(body=[node],type_ignores=[]),str(ROOT/'device.py'),'exec'),namespace)
    plugin=namespace['RealSensePlugin'].__new__(namespace['RealSensePlugin'])
    statuses=iter([{'state':'unavailable','camera_info':[]}, {'state':'ready','camera_info':[{'schema':'motus.camera/1'}]}])
    plugin._rgbd_info=lambda:next(statuses)
    plugin.is_running=lambda:True
    assert plugin.dispatch('start',{'_tool_name':'camera_rgbd'})['camera_info']
