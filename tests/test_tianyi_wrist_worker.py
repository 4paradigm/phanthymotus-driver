"""Run the capture/session boundary with controlled SDK and ROS dependencies."""
import importlib
import multiprocessing as mp
from pathlib import Path
import queue
import sys
import threading
import time
from types import ModuleType, SimpleNamespace as NS
import zlib

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / 'x-humanoid/tianyi2.0')]
wrist = importlib.import_module('wrist_cameras')


def descriptor(depth=True):
    return {'id': 'realsense:non-numeric', 'serial': 'non-numeric', 'backend': 'realsense' if depth else 'uvc',
            'path': '/dev/webcam', 'name': 'Another Camera', 'capabilities': {'rgb': True, 'depth': depth}}


def profiles(depth=True):
    p = {'rgb': {'width': 640, 'height': 480, 'fps': 15, 'format': 'rgb8'}}
    if depth: p['depth'] = {'width': 640, 'height': 480, 'fps': 15, 'format': 'z16'}
    return p


@pytest.fixture
def capture(monkeypatch, tmp_path):
    from common import logsafe
    monkeypatch.setattr(logsafe, '_installed', False)
    monkeypatch.setattr(sys, 'stdout', sys.stdout)
    monkeypatch.setattr(sys, 'stderr', sys.stderr)
    for key in ('ROS_DOMAIN_ID', 'RMW_IMPLEMENTATION', 'FASTRTPS_DEFAULT_PROFILES_FILE', 'RCUTILS_COLORIZED_OUTPUT'):
        monkeypatch.setenv(key, wrist.os.environ.get(key, ''))
    dds = tmp_path / 'dds.xml'
    dds.write_text('<dds/>')
    monkeypatch.setattr(wrist, 'DDS_PROFILE', str(dds))
    control, child_control = mp.get_context('spawn').Pipe()
    stop, ready = threading.Event(), threading.Event()
    statuses, published, hooks = queue.Queue(), [], {}
    depth = np.full((480, 640), 1000, np.uint16)
    rgb = np.zeros((480, 640, 3), np.uint8)
    intr = NS(width=640, height=480, fx=600, fy=601, ppx=319, ppy=239, coeffs=[0.] * 5, model='none')
    def frame(channel):
        def data():
            if 'encode' in hooks: hooks['encode'](channel)
            return rgb if channel == 'rgb' else depth
        return NS(get_data=data, get_timestamp=lambda: 123.,
                  profile=NS(as_video_stream_profile=lambda: NS(get_intrinsics=lambda: intr)))
    frames = NS(get_color_frame=lambda: frame('rgb'), get_depth_frame=lambda: frame('depth'))
    dev = NS(first_depth_sensor=lambda: NS(get_depth_scale=lambda: .0001))
    class Pipeline:
        calls = 0
        def start(self, cfg):
            if 'startup' in hooks: hooks['startup']()
            return NS(get_device=lambda: dev)
        def wait_for_frames(self, timeout_ms):
            self.calls += 1
            if self.calls > 1:
                stop.set()
                return NS(get_color_frame=lambda: None, get_depth_frame=lambda: None)
            if 'wait' in hooks: hooks['wait']()
            return frames
        def stop(self): pass
    rs = ModuleType('pyrealsense2')
    rs.config = lambda: NS(enable_device=lambda *a: None, enable_stream=lambda *a: None)
    rs.pipeline = Pipeline
    rs.stream = NS(color='color', depth='depth')
    rs.format = NS(rgb8='rgb8', bgr8='bgr8', z16='z16')
    monkeypatch.setitem(sys.modules, 'pyrealsense2', rs)
    class Publisher:
        def __init__(self, topic): self.topic = topic
        def publish(self, msg):
            if 'publish' in hooks: hooks['publish'](self.topic)
            published.append((self.topic, msg))
    node = NS(create_publisher=lambda cls, topic, qos: Publisher(topic), destroy_publisher=lambda p: None,
              get_clock=lambda: NS(now=lambda: NS(to_msg=lambda: NS())), destroy_node=lambda: None)
    ros = ModuleType('rclpy')
    ros.init = lambda **kw: None
    ros.shutdown = lambda: None
    ros.create_node = lambda name: node
    qos = ModuleType('rclpy.qos')
    qos.qos_profile_sensor_data = object()
    msg = ModuleType('sensor_msgs.msg')
    msg.CompressedImage = lambda: NS(header=NS())
    for name, value in [('rclpy', ros), ('rclpy.qos', qos), ('sensor_msgs', ModuleType('sensor_msgs')), ('sensor_msgs.msg', msg)]:
        monkeypatch.setitem(sys.modules, name, value)
    def update(routes):
        control.send((1, routes))
        assert control.poll(2), 'route change did not acknowledge'
        assert control.recv() == 1
    def run(depth=True):
        wrist._capture_worker('robot', descriptor(depth), profiles(depth), 85,
                              {'card-a': 'left', 'card-b': 'right'}, child_control, stop, statuses)
    yield NS(run=run, hooks=hooks, published=published, stop=stop, update=update,
             statuses=statuses, control=control, child_control=child_control)
    stop.set()
    control.close()
    child_control.close()


def test_rgbd_worker_fans_out_real_depth_units_without_model_check(capture):
    capture.run()
    assert [t for t, m in capture.published] == ['/robot/wrist_camera/card_a/rgb', '/robot/wrist_camera/card_b/rgb',
                                               '/robot/wrist_camera/card_a/depth', '/robot/wrist_camera/card_b/depth']
    for topic, msg in capture.published:
        if topic.endswith('/depth'):
            raw = np.frombuffer(zlib.decompress(msg.data), '<u2')
            assert len(raw) == 640 * 480 and set(raw) == {100}
    status = capture.statuses.get_nowait()
    assert status['intrinsics']['rgb']['K'] == [600., 0., 319., 0., 601., 239., 0., 0., 1.]
    assert status['sdk_timestamp_ms']['depth'] == 123.


@pytest.mark.parametrize('phase', ['wait', 'encode', 'startup'])
def test_route_change_during_capture_prevents_removed_instance_publication(capture, phase):
    if phase == 'encode':
        capture.hooks[phase] = lambda channel: capture.update({'card-b': 'right'}) if channel == 'rgb' else None
    else:
        capture.hooks[phase] = lambda: capture.update({'card-b': 'right'})
    capture.run()
    assert [t for t, m in capture.published] == ['/robot/wrist_camera/card_b/rgb', '/robot/wrist_camera/card_b/depth']


def test_route_ack_waits_for_inflight_publish(capture):
    entered, release, done = threading.Event(), threading.Event(), threading.Event()
    def publish(topic):
        if topic.endswith('card_a/rgb'):
            entered.set()
            assert release.wait(3)
    capture.hooks['publish'] = publish
    worker = threading.Thread(target=capture.run)
    worker.start()
    assert entered.wait(3)
    capture.control.send((1, {'card-b': 'right'}))
    try:
        assert not capture.control.poll(.1), 'stop acknowledged before publication completed'
    finally:
        release.set()
        assert capture.control.poll(3)
        assert capture.control.recv() == 1
        worker.join(3)
    assert not worker.is_alive()


def test_uvc_worker_publishes_jpeg_only_and_releases_device(capture, monkeypatch):
    import cv2
    released = []
    class Camera:
        calls = 0
        def set(self, *a): return True
        def get(self, key): return {cv2.CAP_PROP_FRAME_WIDTH: 640, cv2.CAP_PROP_FRAME_HEIGHT: 480, cv2.CAP_PROP_FPS: 15}[key]
        def isOpened(self): return True
        def read(self):
            self.calls += 1
            if self.calls > 1:
                capture.stop.set()
                return False, None
            return True, np.zeros((480, 640, 3), np.uint8)
        def release(self): released.append(True)
    monkeypatch.setattr(cv2, 'VideoCapture', lambda *a: Camera())
    capture.run(depth=False)
    assert len(capture.published) == 2
    assert all(t.endswith('/rgb') for t, m in capture.published)
    assert released == [True]


def test_depth_encoding_rejects_invalid_layout_and_scale_and_never_wraps():
    data = np.full((480, 640), 1000, np.uint16)
    for scale in [0, -1, float('nan'), float('inf')]:
        with pytest.raises(ValueError): wrist.encode_depth_mm(data, scale)
    with pytest.raises(ValueError): wrist.encode_depth_mm(np.zeros((240, 320), np.uint16), .001)
    assert not np.frombuffer(zlib.decompress(wrist.encode_depth_mm(data, 1)), '<u2').any()


def test_session_reports_missing_channel_even_with_fresh_sibling(monkeypatch):
    s = wrist.WristCameraSession('robot', descriptor(), profiles())
    s._proc = NS(is_alive=lambda: True)
    s.routes = {'card-a': 'left'}
    s._requested_at = {'card-a': 100.}
    s._status = {'last_frame': {'rgb': 112.}, 'frames': {'rgb': 12}, 'intrinsics': {}}
    monkeypatch.setattr(wrist.time, 'monotonic', lambda: 112.)
    info = s.info('card-a', 'left')
    assert info['state'] == 'error'
    assert info['streams']['rgb']['state'] == 'running'
    assert info['streams']['depth']['state'] == 'error'
    assert info['camera_info'][0]['K'] is None


def _unresponsive_worker(namespace, device, selected, quality, routes, control, stop, statuses):
    stop.wait(10)


def test_route_timeout_closes_unresponsive_child_and_allows_restart(monkeypatch):
    monkeypatch.setattr(wrist, '_capture_worker', _unresponsive_worker)
    monkeypatch.setattr(wrist, 'ROUTE_TIMEOUT_SECONDS', .15)
    s = wrist.WristCameraSession('robot', descriptor(), profiles())
    try:
        s.add('card-a', 'left')
        s.add('card-b', 'right')
        assert not s.healthy()
        assert s.info('card-a', 'left')['state'] == 'error'
        s.remove('card-b')
        s.add('card-a', 'left')
        assert s._proc.is_alive()
    finally:
        s.close()


def _routing_worker(namespace, device, selected, quality, routes, control, stop, statuses):
    while not stop.is_set():
        if control.poll(.05):
            revision, routes = control.recv()
            control.send(revision)


def test_real_session_add_remove_routes_keeps_shared_process(monkeypatch):
    monkeypatch.setattr(wrist, '_capture_worker', _routing_worker)
    s = wrist.WristCameraSession('robot', descriptor(), profiles())
    try:
        s.add('card-a', 'left')
        process = s._proc
        s.add('card-b', 'right')
        s.remove('card-a')
        assert s._proc is process and process.is_alive()
        assert s.routes == {'card-b': 'right'}
        s.remove('card-b')
        assert s._proc is None
    finally:
        s.close()


@pytest.mark.parametrize('elapsed,last,expected', [(1., None, 'loading'), (2., 101.5, 'running'), (12., None, 'error')])
def test_session_first_frame_readiness(monkeypatch, elapsed, last, expected):
    s = wrist.WristCameraSession('robot', descriptor(), profiles())
    s.routes, s._requested_at = {'card-a': 'left'}, {'card-a': 100.}
    s._proc = NS(is_alive=lambda: True)
    s._status = {'last_frame': {} if last is None else {'rgb': last, 'depth': last}}
    monkeypatch.setattr(wrist.time, 'monotonic', lambda: 100. + elapsed)
    assert s.info('card-a', 'left')['state'] == expected


def test_missing_dds_profile_never_opens_usb_and_reports_error(capture, monkeypatch):
    monkeypatch.setattr(wrist, 'DDS_PROFILE', '/nonexistent/dds-profile.xml')
    capture.run()
    assert 'Missing loopback-only DDS profile' in capture.statuses.get_nowait()['error']
    assert not capture.published


@pytest.mark.parametrize('last_frames,expected', [({}, False), ({'rgb': 101., 'depth': 101.}, False),
                                                 ({'rgb': 112.}, True)])
def test_health_recovers_a_stalled_pipeline_without_interrupting_a_fresh_sibling(monkeypatch, last_frames, expected):
    s = wrist.WristCameraSession('robot', descriptor(), profiles())
    s.routes, s._requested_at = {'card-a': 'left'}, {'card-a': 100.}
    s._proc = NS(is_alive=lambda: True)
    s._status = {'last_frame': last_frames}
    monkeypatch.setattr(wrist.time, 'monotonic', lambda: 112.)
    assert s.healthy() is expected
