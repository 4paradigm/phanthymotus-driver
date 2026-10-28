"""The bundle camera subprocess owns the one RealSense pipeline.

The optional raw RGB-D branch for the visual-odometry driver has to share that
pipeline and must never be able to stop JPEG/Z16 media production. Only the
hardware, codec, ROS and child-process boundaries are replaced here.
"""
import importlib.util
import sys
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
               aligned=False, streams=[], alignment=[])
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

    messages = sys.modules['sensor_msgs.msg']
    monkeypatch.setattr(messages, 'CompressedImage', Message)
    monkeypatch.setattr(messages, 'Image', Message)

    class Node:
        def __init__(self, name):
            pass

        def create_publisher(self, cls, topic, qos):
            state.published[topic] = []
            return NS(publish=state.published[topic].append)

        def get_clock(self):
            return NS(now=lambda: NS(to_msg=lambda: NS(sec=1, nanosec=2)))

    monkeypatch.setattr(sys.modules['rclpy.node'], 'Node', Node)
    rclpy = sys.modules['rclpy']
    rclpy.init = lambda: None
    return device, state


def test_raw_branch_reuses_the_same_frameset_as_the_media_publishers(capture, monkeypatch):
    device, state = capture
    seen = []

    class RawCameraPublisher:
        def __init__(self, node, namespace, pipeline, qos):
            seen.append(('init', namespace))

        def publish(self, source_frames, aligned_frames):
            assert state.aligned
            seen.append(('publish', source_frames, aligned_frames))

    monkeypatch.setitem(sys.modules, 'visual_inputs',
                        NS(RawCameraPublisher=RawCameraPublisher))
    device._camera_subprocess('robot')
    assert seen[0] == ('init', 'robot')
    assert [step[0] for step in seen] == ['init', 'publish']
    assert state.published['/robot/camera/color']
    assert state.published['/robot/camera/depth']
    assert state.stopped


def test_raw_publish_failures_never_stop_media_production(capture, monkeypatch, capsys):
    device, state = capture
    state.frame_limit = 2
    failures = []

    class RawCameraPublisher:
        def __init__(self, *args):
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
    assert capsys.readouterr().out.count(
        'raw RGB-D publish failed: raw branch exploded') == 2
    assert state.stopped


def test_raw_branch_construction_failure_keeps_media_production(capture, monkeypatch, capsys):
    device, state = capture

    class RawCameraPublisher:
        def __init__(self, *args):
            raise RuntimeError('no ROS camera messages')

    monkeypatch.setitem(sys.modules, 'visual_inputs',
                        NS(RawCameraPublisher=RawCameraPublisher))
    device._camera_subprocess('robot')
    assert state.published['/robot/camera/color']
    assert 'raw RGB-D branch unavailable: no ROS camera messages' in capsys.readouterr().out
    assert state.stopped


def test_missing_raw_branch_module_is_optional(capture, monkeypatch, capsys):
    device, state = capture
    monkeypatch.setitem(sys.modules, 'visual_inputs', None)
    device._camera_subprocess('robot')
    assert state.published['/robot/camera/color']
    assert 'raw RGB-D branch unavailable' in capsys.readouterr().out
