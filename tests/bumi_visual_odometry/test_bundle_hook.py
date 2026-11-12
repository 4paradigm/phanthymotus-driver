"""The bundle camera subprocess owns the one RealSense pipeline.

The optional raw RGB-D branch for the visual odometry card has to share that
pipeline and must never be able to stop JPEG/Z16 media production. Only the
hardware, codec, ROS and child-process boundaries are replaced here.
"""
from array import array
import importlib.util
import json
import sys
import threading
import zlib
import time
import numpy as np
import cv2
from pathlib import Path
from types import SimpleNamespace as NS

import pytest

BUMI = Path(__file__).resolve().parents[2]/'noetix/bumi'


def _load_device(monkeypatch):
    spec = importlib.util.spec_from_file_location(
        'bumi_vo_bundle_test_loader', BUMI/'tests/test_loco_destination.py')
    loader = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(loader)
    device, _ = loader._load_device(monkeypatch)
    return device


@pytest.fixture
def capture(monkeypatch):
    device = _load_device(monkeypatch)
    state = NS(iterations=0, frame_limit=1, published={}, stopped=False,
               aligned=False, streams=[], alignment=[], spin_calls=0,
               depth_scale=.001, scale_error=None, depth_dims=(640, 480))
    raw = NS(color=b'BGR', depth=b'\x01\x00\x02\x00' + bytes(640 * 480 * 2 - 4))

    class Pixels:
        def __init__(self, data):
            self._data = data

        def tobytes(self):
            return self._data

    class Frame:
        def __init__(self, data):
            self.data = data
            self.profile = NS(as_video_stream_profile=lambda: NS(intrinsics=NS(
                width=state.depth_dims[0], height=state.depth_dims[1], fx=600., fy=601.,
                ppx=320., ppy=240., model='none', coeffs=[0.] * 5)))

        def get_timestamp(self):
            return 1000. + state.iterations * 30.

        def get_frame_timestamp_domain(self):
            return 'hardware_clock'

        def get_data(self):
            return self.data

        def get_width(self):
            return state.depth_dims[0]

        def get_height(self):
            return state.depth_dims[1]

    class Frames:
        def __init__(self):
            self.source_ms = 1000. + state.iterations * 30.
        def get_color_frame(self):
            frame = Frame(np.full((480, 640, 3), 20, np.uint8))
            frame.get_timestamp = lambda: self.source_ms
            return frame

        def get_depth_frame(self):
            payload = getattr(state, 'raw_depth_override', raw.depth)
            pixels = np.frombuffer(payload, '<u2')
            if pixels.size == 640 * 480:
                pixels = pixels.reshape(state.depth_dims[1], state.depth_dims[0])
            frame = Frame(pixels)
            frame.get_timestamp = lambda: self.source_ms
            return frame

    class Pipeline:
        def start(self, config):
            pass

        def get_active_profile(self):
            intr = NS(width=640, height=480, fx=600., fy=601., ppx=320., ppy=240.,
                      model='distortion.brown_conrady', coeffs=[0.]*5)

            def get_depth_scale():
                if state.scale_error is not None:
                    raise state.scale_error
                return state.depth_scale

            return NS(get_stream=lambda stream: NS(
                as_video_stream_profile=lambda: NS(intrinsics=intr)),
                get_device=lambda: NS(first_depth_sensor=lambda: NS(
                    get_depth_scale=get_depth_scale)))

        def wait_for_frames(self, timeout_ms):
            state.iterations += 1
            if getattr(state, 'before_frame', None):
                state.before_frame()
            if state.iterations > state.frame_limit:
                raise RuntimeError('test loop finished')
            return Frames()

        def poll_for_frames(self):
            return None

        def stop(self):
            state.stopped = True

    def align(target):
        state.alignment.append(target)

        def process(frames):
            state.aligned = True
            return frames
        return NS(process=process)

    monkeypatch.setitem(sys.modules, 'pyrealsense2', NS(
        pipeline=Pipeline,
        config=lambda: NS(enable_stream=lambda *args: state.streams.append(args)),
        stream=NS(color='color', depth='depth'),
        format=NS(bgr8='bgr8', z16='z16'), align=align))
    monkeypatch.setitem(sys.modules, 'numpy', np)
    monkeypatch.setitem(sys.modules, 'turbojpeg', None)
    monkeypatch.setitem(sys.modules, 'cv2', NS(
        IMWRITE_JPEG_QUALITY=1, CV_32FC1=cv2.CV_32FC1,
        INTER_LINEAR=cv2.INTER_LINEAR, INTER_NEAREST=cv2.INTER_NEAREST,
        initUndistortRectifyMap=cv2.initUndistortRectifyMap, remap=cv2.remap,
        imencode=lambda *args: (True, NS(tobytes=lambda: b'JPEG'))))

    import common.logsafe
    monkeypatch.setattr(common.logsafe, 'install', lambda **kwargs: None)

    class Message:
        def __init__(self):
            self.header = NS(stamp=NS(sec=0, nanosec=0), frame_id='')
            self.data = b''
            self.format = ''

    class Bool:
        def __init__(self, data=False):
            self.data = data
            self.profile = NS(as_video_stream_profile=lambda: NS(intrinsics=NS(
                width=state.depth_dims[0], height=state.depth_dims[1], fx=600., fy=601.,
                ppx=320., ppy=240., model='none', coeffs=[0.] * 5)))

        def get_timestamp(self):
            return 1000. + state.iterations * 30.

        def get_frame_timestamp_domain(self):
            return 'hardware_clock'

    messages = sys.modules['sensor_msgs.msg']
    monkeypatch.setattr(messages, 'CompressedImage', Message)
    monkeypatch.setattr(messages, 'Image', Message)
    sys.modules['std_msgs.msg'].Bool = Bool
    qos = sys.modules['rclpy.qos']
    qos.ReliabilityPolicy.RELIABLE = 'reliable'
    qos.DurabilityPolicy.TRANSIENT_LOCAL = 'transient_local'
    qos.QoSProfile = lambda **kwargs: NS(**kwargs)

    class Node:
        def __init__(self, name):
            state.node = self
            self.subscriptions = {}

        def create_publisher(self, cls, topic, qos):
            state.published[topic] = []
            def publish(message):
                hook = getattr(state, 'on_publish', None)
                if hook:
                    hook(topic, message)
                state.published[topic].append(message)
            return NS(publish=publish)

        def create_subscription(self, cls, topic, callback, qos):
            self.subscriptions[topic] = callback
            self.subscription_qos = getattr(self, 'subscription_qos', {})
            self.subscription_qos[topic] = qos
            return NS()

        def get_clock(self):
            return NS(now=lambda: NS(to_msg=lambda: NS(sec=1, nanosec=2),
                                     nanoseconds=1_000_000_002))

    monkeypatch.setattr(sys.modules['rclpy.node'], 'Node', Node)
    rclpy = sys.modules['rclpy']
    rclpy.init = lambda: None

    def spin_once(node, timeout_sec=0):
        state.spin_calls += 1
        hook = getattr(state, 'on_spin', None)
        if hook:
            hook(node)

    rclpy.spin_once = spin_once
    from noetix.bumi import rgbd_camera
    monkeypatch.setitem(sys.modules, 'rgbd_camera', rgbd_camera)
    monkeypatch.setattr(time, 'monotonic', lambda: 100. + state.iterations)
    monkeypatch.setattr(time, 'time_ns', lambda: 10_000_000_000 + state.iterations * 30_000_000)
    return device, state


def test_original_depth_is_normalized_capture_mapped_and_owned_once(capture, monkeypatch):
    device, state = capture
    monkeypatch.delenv('BUMI_VISUAL_ODOMETRY', raising=False)
    state.depth_scale = .002
    device._camera_subprocess('robot')
    depth = state.published['/robot/camera/depth'][0]
    assert depth.header.frame_id == 'robot_depth_optical'
    assert depth.header.stamp.sec * 10**9 + depth.header.stamp.nanosec == 10_030_000_000
    assert isinstance(depth.data, array) and depth.data.typecode == 'B'
    assert zlib.decompress(depth.data) == b'\x02\x00\x04\x00' + bytes(640 * 480 * 2 - 4)
    assert '/robot/camera/rgbd/depth' not in state.published
    assert state.alignment == ['depth']
    assert state.stopped


def test_invalid_capture_never_falls_back_to_legacy_depth(capture, monkeypatch):
    device, state = capture
    state.frame_limit = 2
    class Broken:
        def __init__(self, *a, **kw): pass
        def set_enabled(self, cards): pass
        def publish(self, frames): raise ValueError('invalid capture timestamp')
        def unavailable(self, error): pass
    monkeypatch.setitem(sys.modules, 'rgbd_camera', NS(RgbdCameraPublisher=Broken))
    device._camera_subprocess('robot')
    assert len(state.published['/robot/camera/color']) == 2
    assert not state.published.get('/robot/camera/depth')
    assert '/robot/camera/depth/metadata' not in state.published


def test_depth_card_waits_for_real_normalized_camera_info(capture, monkeypatch):
    device, _ = capture
    card = device.CameraPlugin.__new__(device.CameraPlugin)
    card._activations = dict(camera=0, depth=0, imu=0)
    card._stop_generations = dict(camera=0, depth=0, imu=0)
    card._namespace = 'robot'
    card._color_topic, card._depth_topic = '/robot/camera/color', '/robot/camera/depth'
    card._intrinsics_lock = threading.Lock()
    card._rgbd_status = dict(ready=False, camera_info=[], error='awaiting calibrated capture')
    card._proc = None
    card._cards = set()
    card._lifecycle_lock = threading.RLock()
    card._intrinsics = {}
    tool = next(x for x in card.get_tools() if x['name'] == 'depth')
    result = card.dispatch('info', {'_tool_name': 'depth'})
    assert tool['topic_out'] == result['topic_out'] == [
        {'topic': '/robot/camera/depth', 'format': 'image/depth-zlib'}]
    assert result['camera_info'] == [] and result['sourceReady'] is False



def test_raw_branch_reuses_the_same_frameset_as_the_media_publishers(
        capture, monkeypatch):
    device, state = capture
    monkeypatch.setenv('BUMI_VISUAL_ODOMETRY', '1')
    seen = []

    class RawCameraPublisher:
        def __init__(self, node, namespace, pipeline, qos, **kwargs):
            seen.append(('init', namespace, kwargs.get('min_interval_s')))

        def publish(self, source_frames, aligned_frames):
            assert state.aligned
            seen.append(('publish', source_frames, aligned_frames))

    monkeypatch.setitem(sys.modules, 'visual_inputs',
                        NS(RawCameraPublisher=RawCameraPublisher))
    device._camera_subprocess('robot')
    assert seen[0] == ('init', 'robot', 0.1)
    assert [step[0] for step in seen] == ['init', 'publish']
    assert state.published['/robot/camera/color']
    assert state.published['/robot/camera/depth']
    assert state.stopped


def test_raw_rate_cap_comes_from_the_environment(capture, monkeypatch):
    device, state = capture
    monkeypatch.setenv('BUMI_VISUAL_ODOMETRY', '1')
    monkeypatch.setenv('BUMI_VISUAL_ODOMETRY_MAX_FPS', '5')
    seen = []

    class RawCameraPublisher:
        def __init__(self, *args, **kwargs):
            seen.append(kwargs)

        def publish(self, source_frames, aligned_frames):
            pass

    monkeypatch.setitem(sys.modules, 'visual_inputs',
                        NS(RawCameraPublisher=RawCameraPublisher))
    device._camera_subprocess('robot')
    assert seen == [{'min_interval_s': .2}]


def test_raw_publish_failures_never_stop_media_production(capture, monkeypatch, capsys):
    device, state = capture
    monkeypatch.setenv('BUMI_VISUAL_ODOMETRY', '1')
    state.frame_limit = 2
    failures = []

    class RawCameraPublisher:
        def __init__(self, *args, **kwargs):
            pass

        def publish(self, source_frames, aligned_frames):
            failures.append(source_frames)
            raise RuntimeError('raw branch exploded')

    monkeypatch.setitem(sys.modules, 'visual_inputs',
                        NS(RawCameraPublisher=RawCameraPublisher))
    device._camera_subprocess('robot')
    assert len(failures) == 2
    assert len(state.published['/robot/camera/color']) == 2
    assert len(state.published['/robot/camera/depth']) == 2
    # First failure is reported, the second one is not: the 30 Hz loop must not
    # flood a rotated Docker log with one record per frame.
    output = capsys.readouterr().out
    assert output.count('raw RGB-D publish failed') == 1
    assert 'failure #1): raw branch exploded' in output
    assert state.stopped


def test_persistent_raw_failures_are_log_sampled(capture, monkeypatch, capsys):
    device, state = capture
    monkeypatch.setenv('BUMI_VISUAL_ODOMETRY', '1')
    state.frame_limit = 250
    failures = []

    class RawCameraPublisher:
        def __init__(self, *args, **kwargs):
            pass

        def publish(self, source_frames, aligned_frames):
            failures.append(source_frames)
            raise RuntimeError('raw branch stayed broken')

    monkeypatch.setitem(sys.modules, 'visual_inputs',
                        NS(RawCameraPublisher=RawCameraPublisher))
    device._camera_subprocess('robot')
    assert len(failures) == 250
    assert len(state.published['/robot/camera/color']) == 250
    output = capsys.readouterr().out
    # 1st, 100th and 200th failure only.
    assert output.count('raw RGB-D publish failed') == 3
    assert 'failure #1)' in output and 'failure #100)' in output and 'failure #200)' in output
    assert state.stopped


def test_raw_branch_construction_failure_keeps_media_production(capture, monkeypatch, capsys):
    device, state = capture
    monkeypatch.setenv('BUMI_VISUAL_ODOMETRY', '1')

    class RawCameraPublisher:
        def __init__(self, *args, **kwargs):
            raise RuntimeError('no ROS camera messages')

    monkeypatch.setitem(sys.modules, 'visual_inputs',
                        NS(RawCameraPublisher=RawCameraPublisher))
    device._camera_subprocess('robot')
    assert state.published['/robot/camera/color']
    assert 'raw RGB-D branch unavailable: no ROS camera messages' in capsys.readouterr().out
    assert state.stopped


def test_raw_branch_is_off_when_the_visual_odometry_card_is_disabled(capture, monkeypatch):
    device, state = capture
    monkeypatch.delenv('BUMI_VISUAL_ODOMETRY', raising=False)
    seen = []

    class RawCameraPublisher:
        def __init__(self, *args, **kwargs):
            seen.append(args)

    monkeypatch.setitem(sys.modules, 'visual_inputs',
                        NS(RawCameraPublisher=RawCameraPublisher))
    device._camera_subprocess('robot')
    assert seen == []
    assert state.published['/robot/camera/color']
    assert state.stopped


def test_missing_raw_branch_module_is_optional(capture, monkeypatch, capsys):
    device, state = capture
    monkeypatch.setenv('BUMI_VISUAL_ODOMETRY', '1')
    monkeypatch.setitem(sys.modules, 'visual_inputs', None)
    device._camera_subprocess('robot')
    assert state.published['/robot/camera/color']
    assert 'raw RGB-D branch unavailable' in capsys.readouterr().out


def test_raw_control_stops_publications_but_not_media(capture, monkeypatch):
    device, state = capture
    monkeypatch.setenv('BUMI_VISUAL_ODOMETRY', '1')
    state.frame_limit = 3
    published = []

    class RawCameraPublisher:
        enabled = True

        def __init__(self, *args, **kwargs):
            pass

        def publish(self, source_frames, aligned_frames):
            published.append(source_frames)

    def on_spin(node):
        if state.iterations == 1:
            callback = node.subscriptions['/robot/odometry_input/raw_enabled']
            callback(NS(data=False))

    state.on_spin = on_spin
    monkeypatch.setitem(sys.modules, 'visual_inputs',
                        NS(RawCameraPublisher=RawCameraPublisher))
    device._camera_subprocess('robot')
    # The camera loop serviced its own ROS callbacks (spin_once), so the card
    # stop signal reached it. First frame published raw; the shared branch was
    # disabled before frames two and three, while JPEG/Z16 kept running.
    assert state.spin_calls == 4
    assert len(published) == 1
    assert len(state.published['/robot/camera/color']) == 3
    assert len(state.published['/robot/camera/depth']) == 3


def test_camera_starts_raw_disabled_until_the_latched_enable_arrives(capture, monkeypatch):
    device, state = capture
    monkeypatch.setenv('BUMI_VISUAL_ODOMETRY', '1')
    state.frame_limit = 3
    published = []

    class RawCameraPublisher:
        enabled = False

        def __init__(self, *args, **kwargs):
            pass

        def publish(self, source_frames, aligned_frames):
            published.append(source_frames)

    def on_spin(node):
        if state.iterations == 1:
            callback = node.subscriptions['/robot/odometry_input/raw_enabled']
            callback(NS(data=True))

    state.on_spin = on_spin
    monkeypatch.setitem(sys.modules, 'visual_inputs',
                        NS(RawCameraPublisher=RawCameraPublisher))
    device._camera_subprocess('robot')
    # Frame one: no retained enable yet, so no raw publication. The latched
    # enable arrives on the next tick; frames two and three publish raw.
    assert len(published) == 2
    assert len(state.published['/robot/camera/color']) == 3
    assert state.node.subscription_qos[
        '/robot/odometry_input/raw_enabled'].durability == 'transient_local'


@pytest.mark.parametrize('value', ['0', '-5', 'nan', 'inf', 'not-a-number', ''])
def test_invalid_raw_rate_falls_back_to_the_documented_default(capture, monkeypatch, value):
    device, state = capture
    monkeypatch.setenv('BUMI_VISUAL_ODOMETRY', '1')
    monkeypatch.setenv('BUMI_VISUAL_ODOMETRY_MAX_FPS', value)
    seen = []

    class RawCameraPublisher:
        def __init__(self, *args, **kwargs):
            seen.append(kwargs.get('min_interval_s'))

        def publish(self, source_frames, aligned_frames):
            pass

    monkeypatch.setitem(sys.modules, 'visual_inputs',
                        NS(RawCameraPublisher=RawCameraPublisher))
    device._camera_subprocess('robot')
    assert len(seen) == 1 and abs(seen[0]-0.1) < 1e-9


@pytest.mark.parametrize('scale', [0.0, -0.001, 0.1, 1.0,
                                   float('nan'), float('inf'),
                                   -float('inf'), True, None])
def test_invalid_sdk_depth_scale_disables_depth_but_keeps_color(
        capture, monkeypatch, capsys, scale):
    device, state = capture
    monkeypatch.delenv('BUMI_VISUAL_ODOMETRY', raising=False)
    state.depth_scale = scale
    monkeypatch.setitem(sys.modules, 'visual_inputs', None)
    device._camera_subprocess('robot')
    assert state.published['/robot/camera/color']
    assert not state.published.get('/robot/camera/depth')
    assert 'invalid SDK depth scale' in capsys.readouterr().out


@pytest.mark.parametrize(('dimensions', 'payload'), [
    ((320, 960), b'\x01\x00' + bytes(640 * 480 * 2 - 2)),
    ((640, 479), b'\x01\x00' + bytes(640 * 480 * 2 - 2)),
    ((640, 480), b'\x01\x00'),
    ((640, 480), b'\x01'),
    ((640, 480), b'\x01\x00' + bytes(640 * 480 * 2)),
])
def test_normalized_source_rejects_wrong_dimensions_and_raw_buffer_sizes(
        capture, monkeypatch, dimensions, payload, capsys):
    device, state = capture
    monkeypatch.delenv('BUMI_VISUAL_ODOMETRY', raising=False)
    monkeypatch.setitem(sys.modules, 'visual_inputs', None)
    state.depth_dims = dimensions
    state.raw_depth_override = payload
    state.frame_limit = 2
    device._camera_subprocess('robot')
    assert len(state.published['/robot/camera/color']) == 2
    assert not state.published.get('/robot/camera/depth')
    output = capsys.readouterr().out
    assert not state.published['/robot/camera/depth']



def test_card_controls_stop_depth_without_stopping_color_or_owner(capture, monkeypatch):
    device, state = capture
    monkeypatch.delenv('BUMI_VISUAL_ODOMETRY', raising=False)
    state.frame_limit = 4
    def on_spin(node):
        callback = node.subscriptions['/robot/camera/cards_enabled']
        if state.iterations == 1:
            callback(NS(data=json.dumps(dict(generation=1, enabled=['camera'], activations=dict(camera=0, depth=0, imu=0)))))
        elif state.iterations == 2:
            callback(NS(data=json.dumps(dict(generation=2, enabled=['camera','depth'], activations=dict(camera=0, depth=1, imu=0)))))
        elif state.iterations == 3:
            callback(NS(data=json.dumps(dict(generation=3, enabled=['depth'], activations=dict(camera=0, depth=1, imu=0)))))
    state.on_spin = on_spin
    device._camera_subprocess('robot')
    assert len(state.published['/robot/camera/color']) == 3
    assert len(state.published['/robot/camera/rgbd/color']) == 3
    depth = state.published['/robot/camera/depth']
    assert len(depth) == 3
    assert [m.header.stamp.nanosec for m in depth] == [30_000_000, 90_000_000, 120_000_000]
    assert '/robot/camera/rgbd/depth' not in state.published
    assert state.stopped
