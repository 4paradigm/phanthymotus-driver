"""The bundle camera subprocess owns the one RealSense pipeline.

The optional raw RGB-D branch for the visual odometry card has to share that
pipeline and must never be able to stop JPEG/Z16 media production. Only the
hardware, codec, ROS and child-process boundaries are replaced here.
"""
from array import array
import importlib.util
import sys
import zlib
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
               aligned=False, streams=[], alignment=[], spin_calls=0)
    raw = NS(color=b'BGR', depth=b'\x01\x00\x02\x00')

    class Pixels:
        def __init__(self, data):
            self._data = data

        def tobytes(self):
            return self._data

    class Frame:
        def __init__(self, data):
            self.data = data

        def get_data(self):
            return self.data

    class Frames:
        def get_color_frame(self):
            assert state.aligned
            return Frame(raw.color)

        def get_depth_frame(self):
            assert state.aligned
            return Frame(raw.depth)

    class Pipeline:
        def start(self, config):
            pass

        def get_active_profile(self):
            intr = NS(width=640, height=480, fx=600., fy=601., ppx=320., ppy=240.,
                      model='distortion.brown_conrady', coeffs=[0.]*5)
            return NS(get_stream=lambda stream: NS(
                as_video_stream_profile=lambda: NS(intrinsics=intr)))

        def wait_for_frames(self, timeout_ms):
            state.iterations += 1
            if getattr(state, 'before_frame', None):
                state.before_frame()
            if state.iterations > state.frame_limit:
                raise RuntimeError('test loop finished')
            return Frames()

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
    monkeypatch.setitem(sys.modules, 'numpy', NS(asanyarray=lambda data: Pixels(data)))
    monkeypatch.setitem(sys.modules, 'turbojpeg', None)
    monkeypatch.setitem(sys.modules, 'cv2', NS(
        IMWRITE_JPEG_QUALITY=1,
        imencode=lambda *args: (True, NS(tobytes=lambda: b'JPEG'))))

    import common.logsafe
    monkeypatch.setattr(common.logsafe, 'install', lambda **kwargs: None)

    class Message:
        def __init__(self):
            self.header = NS(stamp=None)
            self.data = b''
            self.format = ''

    class Bool:
        def __init__(self, data=False):
            self.data = data

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
            return NS(publish=state.published[topic].append)

        def create_subscription(self, cls, topic, callback, qos):
            self.subscriptions[topic] = callback
            self.subscription_qos = getattr(self, 'subscription_qos', {})
            self.subscription_qos[topic] = qos
            return NS()

        def get_clock(self):
            return NS(now=lambda: NS(to_msg=lambda: NS(sec=1, nanosec=2)))

    monkeypatch.setattr(sys.modules['rclpy.node'], 'Node', Node)
    rclpy = sys.modules['rclpy']
    rclpy.init = lambda: None

    def spin_once(node, timeout_sec=0):
        state.spin_calls += 1
        hook = getattr(state, 'on_spin', None)
        if hook:
            hook(node)

    rclpy.spin_once = spin_once
    return device, state


def test_media_uses_byte_buffers_without_changing_encoded_payloads(capture, monkeypatch):
    device, state = capture
    monkeypatch.delenv('BUMI_VISUAL_ODOMETRY', raising=False)
    device._camera_subprocess('robot')

    color = state.published['/robot/camera/color'][0]
    depth = state.published['/robot/camera/depth'][0]
    assert isinstance(color.data, array) and color.data.typecode == 'B'
    assert isinstance(depth.data, array) and depth.data.typecode == 'B'
    assert bytes(color.data) == b'JPEG'
    assert color.format == 'jpeg'
    assert zlib.decompress(depth.data) == b'\x01\x00\x02\x00'
    assert depth.format == '16UC1; compressedDepth zlib'


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
    assert seen[0] == ('init', 'robot', 0.0)
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
        def __init__(self, *args):
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
