"""Bumi generic RGB-D source: calibrated pixels and capture time, no hardware."""
import importlib.util
from pathlib import Path
import sys
from types import SimpleNamespace as NS
import zlib

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from noetix.bumi import rgbd_camera as rgbd


def intrinsics(**changes):
    data = dict(width=640, height=480, fx=600., fy=601., ppx=320., ppy=240.,
                model='inverse_brown_conrady', coeffs=[0.] * 5)
    data.update(changes)
    return NS(**data)


def frame(ms, pixels, intr=None, domain='hardware_clock'):
    return NS(get_timestamp=lambda: ms, get_frame_number=lambda: int(ms),
              get_frame_timestamp_domain=lambda: 'timestamp_domain.' + domain,
              get_data=lambda: pixels,
              profile=NS(as_video_stream_profile=lambda: NS(intrinsics=intr or intrinsics())))


def frames(ms=1000., intr=None, domain='hardware_clock'):
    color = frame(ms, np.full((480, 640, 3), 40, np.uint8), intr, domain)
    depth = frame(ms + 10., np.full((480, 640), 500, '<u2'), intr, domain)
    return NS(get_color_frame=lambda: color, get_depth_frame=lambda: depth)


@pytest.fixture
def source(monkeypatch):
    class Message:
        def __init__(self):
            self.header = NS(stamp=NS(sec=0, nanosec=0), frame_id='')
    monkeypatch.setitem(sys.modules, 'sensor_msgs.msg', NS(CompressedImage=Message))
    messages = {}
    node = NS(create_publisher=lambda cls, topic, qos: NS(publish=messages.setdefault(topic, []).append))
    pipeline = NS(get_active_profile=lambda: NS(get_device=lambda: NS(
        first_depth_sensor=lambda: NS(get_depth_scale=lambda: .002))))
    notices = []
    monkeypatch.setitem(sys.modules, 'pyrealsense2', NS(
        stream=NS(depth='depth'), align=lambda target: NS(process=lambda frames: frames)))
    publisher = rgbd.RgbdCameraPublisher(node, 'robot', pipeline, object(), report=notices.append)
    monkeypatch.setattr(rgbd.time, 'time_ns', lambda: 10_000_000_000)
    return publisher, messages, notices


def stamp(message):
    return message.header.stamp.sec * 10**9 + message.header.stamp.nanosec


def test_source_scales_measured_depth_and_keeps_distinct_capture_headers(source):
    publisher, messages, notices = source
    pair = frames()
    publisher.publish(pair)
    color = messages['/robot/camera/rgbd/color'][0]
    depth = messages['/robot/camera/rgbd/depth'][0]
    assert stamp(color) == 10_000_000_000
    assert stamp(depth) == 10_010_000_000
    assert color.header.frame_id == depth.header.frame_id == 'robot_depth_optical'
    decoded = np.frombuffer(zlib.decompress(depth.data), '<u2').reshape(480, 640)
    assert decoded[240, 320] == 1000
    assert notices[-1]['ready'] is True
    vendor = notices[-1]['camera_info'][0]['vendor']
    assert vendor['source_clock'] == 'hardware_clock'
    assert vendor['clock_mapping'] == 'fixed_arrival_offset'
    assert vendor['mapping_offset_ns'] == 9_000_000_000
    assert vendor['clock_mapping_verified'] is False
    # Subsequent captures advance by source time, not variable processing delay.
    later = frames(ms=1030.)
    publisher.publish(later)
    assert stamp(messages['/robot/camera/rgbd/color'][1]) == 10_030_000_000


def test_source_declarations_are_accepted_by_generic_perception_contract(source):
    publisher, _, notices = source
    pair = frames()
    publisher.publish(pair)
    declarations = notices[-1]['camera_info']
    assert len(declarations) == 2
    # Load the real independent generic consumer rather than duplicating its checks.
    consumer_root = ROOT.parent / 'pr371-current-review'
    if not (consumer_root / 'perception/plugins/rgbd_odometry_contract.py').exists():
        pytest.skip('independent Perception checkout is not present')
    spec = importlib.util.spec_from_file_location('perception.plugins.bumi_generic_contract',
        consumer_root / 'perception/plugins/rgbd_odometry_contract.py')
    module = importlib.util.module_from_spec(spec)
    old_path = list(sys.path)
    sys.path.insert(0, str(consumer_root))
    try:
        spec.loader.exec_module(module)
        accepted = module.validate_inputs({d['topic']: d for d in declarations},
                                          [d['topic'] for d in declarations])
    finally:
        sys.path[:] = old_path
    assert accepted['frame_id'] == 'robot_depth_optical'
    assert accepted['K'].tolist() == [[600., 0., 320.], [0., 601., 240.], [0., 0., 1.]]
    assert all(d['vendor']['clock_mapping_verified'] is False for d in declarations)


def test_distorted_color_and_registered_depth_use_the_same_rectified_rays():
    geometry = rgbd.make_geometry(intrinsics(model='brown_conrady', coeffs=[.2, 0., 0., 0., 0.]))
    # At x=.5, y=0 the original distorted image has x=.525.
    assert geometry['map_x'][240, 620] == pytest.approx(635., abs=.001)
    assert geometry['map_y'][240, 620] == pytest.approx(240.)
    assert geometry['K'] == [600., 0., 320., 0., 601., 240., 0., 0., 1.]


@pytest.mark.parametrize('change,reason', [
    ({'coeffs': [.1, 0., 0., 0., 0.]}, 'distortion'),
    ({'fx': float('nan')}, 'intrinsics'),
    ({'width': 1280}, '640x480'),
    ({'model': 'ftheta'}, 'distortion'),
])
def test_unusable_geometry_never_publishes_a_partial_pair(source, change, reason):
    publisher, messages, _ = source
    pair = frames(intr=intrinsics(**change))
    with pytest.raises(ValueError, match=reason):
        publisher.publish(pair)
    assert not any(messages.values())


def test_repeated_capture_is_rejected_without_receipt_time_substitution(source):
    publisher, messages, _ = source
    pair = frames()
    publisher.publish(pair)
    with pytest.raises(ValueError, match='clock reset'):
        publisher.publish(pair)
    assert all(len(v) == 1 for v in messages.values())


def test_bad_depth_scale_is_rejected_before_any_pair_is_published(monkeypatch):
    monkeypatch.setitem(sys.modules, 'sensor_msgs.msg', NS(CompressedImage=NS))
    node = NS(create_publisher=lambda *args: NS())
    pipeline = NS(get_active_profile=lambda: NS(get_device=lambda: NS(
        first_depth_sensor=lambda: NS(get_depth_scale=lambda: float('nan')))))
    with pytest.raises(ValueError, match='depth scale'):
        rgbd.RgbdCameraPublisher(node, 'robot', pipeline, object())


_spec = importlib.util.spec_from_file_location('bumi_rgbd_capture_fixture',
    ROOT / 'tests/bumi_visual_odometry/test_bundle_hook.py')
_capture_fixture = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_capture_fixture)
capture = _capture_fixture.capture


def test_capture_owner_feeds_generic_source_and_keeps_display_on_source_failure(capture, monkeypatch):
    device, state = capture
    calls = []
    class BrokenSource:
        def __init__(self, node, namespace, pipeline, qos, **kwargs):
            calls.append(('init', namespace, pipeline))
        def publish(self, source):
            calls.append(('pair', source))
            raise ValueError('test unsupported calibration')
        def unavailable(self, error):
            calls.append(('unavailable', str(error)))
    monkeypatch.setitem(sys.modules, 'rgbd_camera', NS(
        RgbdCameraPublisher=BrokenSource))
    monkeypatch.delenv('BUMI_VISUAL_ODOMETRY', raising=False)
    device._camera_subprocess('robot')
    assert [c[0] for c in calls] == ['init', 'pair', 'unavailable']
    assert len(state.published['/robot/camera/color']) == 1
    assert len(state.published['/robot/camera/depth']) == 1
    assert len(state.streams) == 2  # one existing pipeline, two physical streams
    assert state.stopped


def test_camera_rgbd_card_returns_both_declarations_on_start_and_info(monkeypatch):
    device = _capture_fixture._load_device(monkeypatch)
    monkeypatch.syspath_prepend(str(ROOT / 'noetix/bumi'))
    plugin = device.CameraPlugin.__new__(device.CameraPlugin)
    import threading
    plugin._namespace = 'robot'
    plugin._color_topic = '/robot/camera/color'
    plugin._depth_topic = '/robot/camera/depth'
    plugin._intrinsics_lock = threading.Lock()
    plugin._intrinsics = {}
    plugin._proc = NS(poll=lambda: None)
    geometry = dict(K=[600., 0., 320., 0., 601., 240., 0., 0., 1.])
    declarations = rgbd.camera_declarations('/robot/camera/rgbd', geometry,
                                           'robot_depth_optical', 'test_clock', 'global_time', 0)
    plugin._rgbd_status = dict(ready=True, camera_info=declarations, error=None)
    for action in ('start', 'info'):
        result = plugin.dispatch(action, {'_tool_name': 'camera_rgbd'})
        assert result['sourceReady'] is True
        assert 'error' not in result  # MCP treats even error=None as a failed tool call.
        assert result['camera_info'] == declarations
        assert result['topic_out'] == [{'topic': '/robot/camera/rgbd/color', 'format': 'image/jpeg'},
                                       {'topic': '/robot/camera/rgbd/depth', 'format': 'image/depth-zlib'},
                                       {'topic': '/robot/camera/rgbd/imu', 'format': 'sensor/imu', 'optional': True}]
    plugin._proc = NS(poll=lambda: 1)
    assert plugin.dispatch('info', {'_tool_name': 'camera_rgbd'})['sourceReady'] is False
    plugin._proc = NS(poll=lambda: None)
    plugin._rgbd_status = dict(ready=False, camera_info=[], error='unsupported calibration')
    result = plugin.dispatch('info', {'_tool_name': 'camera_rgbd'})
    assert result['sourceReady'] is False and result['camera_info'] == []
    assert result['error'] == 'unsupported calibration'


def test_rectified_pair_samples_registered_color_and_depth_together(source):
    import cv2
    publisher, messages, _ = source
    pair = frames(intr=intrinsics(model='brown_conrady', coeffs=[.2, 0., 0., 0., 0.]))
    color_pixels = pair.get_color_frame().get_data()
    color_pixels[:] = 0
    color_pixels[:, 630:] = 200
    depth_pixels = pair.get_depth_frame().get_data()
    depth_pixels[:] = np.arange(640, dtype='<u2')[None, :]
    publisher.publish(pair)
    rectified = cv2.imdecode(np.frombuffer(messages['/robot/camera/rgbd/color'][0].data, np.uint8), cv2.IMREAD_COLOR)
    depth = np.frombuffer(zlib.decompress(messages['/robot/camera/rgbd/depth'][0].data), '<u2').reshape(480, 640)
    assert int(rectified[240, 620, 0]) == pytest.approx(200, abs=3)
    assert depth[240, 620] == 1270  # nearest source column 635, SDK scale 2 mm
    assert depth[240, 639] == 0     # source ray falls beyond the sensor image


def test_capture_failure_after_recovery_clears_source_readiness(capture, monkeypatch):
    device, state = capture
    state.frame_limit = 4
    calls = []
    class IntermittentSource:
        def __init__(self, *args, **kwargs):
            self.count = 0
        def publish(self, *args):
            self.count += 1
            if self.count in (1, 3):
                raise ValueError('invalid capture')
        def unavailable(self, error):
            calls.append(str(error))
    monkeypatch.setitem(sys.modules, 'rgbd_camera', NS(RgbdCameraPublisher=IntermittentSource))
    device._camera_subprocess('robot')
    assert calls == ['invalid capture', 'invalid capture']


def test_native_depth_geometry_controls_the_computation_pair(source):
    publisher, messages, notices = source
    pair = frames()
    pair.get_depth_frame().profile = NS(as_video_stream_profile=lambda: NS(
        intrinsics=intrinsics(fx=610., fy=611.)))
    publisher.publish(pair)
    assert notices[-1]['camera_info'][0]['K'] == [610., 0., 320., 0., 611., 240., 0., 0., 1.]
    assert messages['/robot/camera/rgbd/depth'][0].header.frame_id == 'robot_depth_optical'


def test_first_source_start_waits_for_core_camera_metadata(monkeypatch):
    device=_capture_fixture._load_device(monkeypatch)
    plugin=device.CameraPlugin.__new__(device.CameraPlugin)
    statuses=iter([dict(sourceReady=False,camera_info=[]),dict(sourceReady=True,camera_info=[{'schema':'motus.camera/1'}])])
    plugin._rgbd_info=lambda:next(statuses)
    plugin.is_running=lambda:True
    assert plugin.dispatch('start',{'_tool_name':'camera_rgbd'})['camera_info']
