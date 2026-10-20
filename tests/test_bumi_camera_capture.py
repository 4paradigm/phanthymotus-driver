"""Direct RealSense capture contracts, with only hardware/codec/ROS boundaries replaced."""
import importlib.util
import json
from pathlib import Path
import signal
import sys
from types import SimpleNamespace as NS
import zlib

import pytest

BUMI = Path(__file__).resolve().parents[1]/'noetix/bumi'
sys.path.insert(0, str(BUMI))
import camera_runtime as module


def test_runtime_launches_only_direct_capture_and_restarts_without_duplicate_owner(monkeypatch):
    opened = []
    def popen(args, **kwargs):
        proc = NS(pid=100+len(opened), code=None, stdout=[])
        proc.poll = lambda: proc.code
        proc.wait = lambda timeout: proc.code
        opened.append((args, kwargs, proc))
        return proc
    monkeypatch.setattr(module.subprocess, 'Popen', popen)
    reaped = []
    monkeypatch.setattr(module, 'stop_process', reaped.append)
    runtime = module.CameraRuntime('robot', print)
    runtime.tick(0)
    assert len(opened) == 1
    assert opened[0][0] == [sys.executable, str(BUMI/'camera_runtime.py'), 'capture', 'robot']
    assert opened[0][1]['start_new_session'] is True
    runtime.tick(1)
    assert len(opened) == 1 and runtime.is_running()
    opened[0][2].code = 1
    runtime.tick(6)
    assert reaped == [opened[0][2]]
    assert len(opened) == 2 and runtime.is_running()
    runtime.stop()
    runtime.tick(20)
    assert len(opened) == 2 and not runtime.is_running()


def test_unreaped_capture_is_not_replaced_and_supervision_survives(monkeypatch):
    opened, lines = [], []
    def popen(*args, **kwargs):
        proc = NS(pid=123, stdout=[], code=None)
        proc.poll = lambda: proc.code
        opened.append(proc)
        return proc
    monkeypatch.setattr(module.subprocess, 'Popen', popen)
    runtime = module.CameraRuntime('robot', lines.append)
    runtime.tick(0)
    opened[0].code = 1
    def blocked(proc):
        raise RuntimeError('capture group is still alive')
    monkeypatch.setattr(module, 'stop_process', blocked)
    runtime.tick(6)
    assert len(opened) == 1 and any('still alive' in line for line in lines)
    runtime.tick(7)
    assert len(opened) == 1
    monkeypatch.setattr(module, 'stop_process', lambda proc: None)
    runtime.tick(12)
    assert len(opened) == 2


@pytest.fixture
def capture(monkeypatch):
    fixture_spec = importlib.util.spec_from_file_location('bumi_capture_ros_stubs',
                                                         BUMI/'tests/test_loco_destination.py')
    fixture = importlib.util.module_from_spec(fixture_spec)
    fixture_spec.loader.exec_module(fixture)
    device, _ = fixture._load_device(monkeypatch)
    state = NS(streams=[], alignment=[], published={}, stopped=False, destroyed=False,
               initialized=False, start_error=False, frame_error=False, encoded=True,
               iterations=0, aligned=False)
    raw = NS(color=b'BGR', depth=b'\x01\x00\x02\x00')
    class Frame:
        def __init__(self, data): self.data = data
        def get_data(self): return NS(tobytes=lambda: self.data)
    class Frames:
        def get_color_frame(self):
            assert state.aligned
            return Frame(raw.color)
        def get_depth_frame(self):
            assert state.aligned
            return Frame(raw.depth)
    class Pipeline:
        def start(self, config):
            if state.start_error: raise RuntimeError('USB busy')
        def get_active_profile(self):
            intr = NS(width=640, height=480, fx=600, fy=601, ppx=320, ppy=240,
                      model='distortion.brown_conrady', coeffs=[0.0]*5)
            return NS(get_stream=lambda stream: NS(as_video_stream_profile=lambda: NS(intrinsics=intr)))
        def wait_for_frames(self, timeout_ms):
            state.iterations += 1
            if state.frame_error: raise RuntimeError('camera disconnected')
            if getattr(state, 'terminate', False):
                signal.getsignal(signal.SIGTERM)(signal.SIGTERM, None)
            if state.iterations > 1: raise KeyboardInterrupt
            return Frames()
        def stop(self): state.stopped = True
    def align(target):
        state.alignment.append(target)
        def process(frames):
            state.aligned = True
            return frames
        return NS(process=process)
    monkeypatch.setitem(sys.modules, 'pyrealsense2', NS(pipeline=Pipeline,
        config=lambda: NS(enable_stream=lambda *args: state.streams.append(args)),
        stream=NS(color='color', depth='depth'), format=NS(bgr8='bgr8', z16='z16'), align=align))
    monkeypatch.setitem(sys.modules, 'numpy', NS(asanyarray=lambda data: data))
    monkeypatch.setitem(sys.modules, 'turbojpeg', None)  # Exercise the installed cv2 fallback.
    monkeypatch.setitem(sys.modules, 'cv2', NS(IMWRITE_JPEG_QUALITY=1,
        imencode=lambda *args: (state.encoded, NS(tobytes=lambda: b'JPEG'))))
    import common.logsafe
    monkeypatch.setattr(common.logsafe, 'install', lambda **kwargs: None)
    class Message:
        def __init__(self): self.header = NS(stamp=None)
    monkeypatch.setattr(sys.modules['sensor_msgs.msg'], 'CompressedImage', Message)
    class Node:
        def __init__(self, name): pass
        def create_publisher(self, cls, topic, qos):
            state.published[topic] = []
            return NS(publish=state.published[topic].append)
        def get_clock(self): return NS(now=lambda: NS(to_msg=lambda: NS(sec=123, nanosec=456)))
        def destroy_node(self): state.destroyed = True
    monkeypatch.setattr(sys.modules['rclpy.node'], 'Node', Node)
    rclpy = sys.modules['rclpy']
    rclpy.init = lambda: setattr(state, 'initialized', True)
    rclpy.ok = lambda: state.initialized
    rclpy.shutdown = lambda: setattr(state, 'initialized', False)
    return device, state, raw


def test_direct_capture_keeps_streams_alignment_formats_and_intrinsics(capture, capsys):
    _, state, raw = capture
    module.capture('robot')
    assert state.streams == [('color', 640, 480, 'bgr8', 30), ('depth', 640, 480, 'z16', 30)]
    assert state.alignment == ['color']
    color, = state.published['/robot/camera/color']
    depth, = state.published['/robot/camera/depth']
    assert color.format == 'jpeg' and color.data == b'JPEG'
    assert depth.format == '16UC1; compressedDepth zlib'
    assert zlib.decompress(depth.data) == raw.depth
    assert (color.header.stamp.sec, color.header.stamp.nanosec) == (123, 456)
    assert (depth.header.stamp.sec, depth.header.stamp.nanosec) == (123, 456)
    line = next(line for line in capsys.readouterr().out.splitlines() if 'INTRINSICS ' in line)
    optics = json.loads(line.split('INTRINSICS ', 1)[1])
    assert optics['color'] == optics['depth']
    assert optics['depth']['fx'] == 600 and optics['depth']['height'] == 480
    assert state.stopped and state.destroyed and not state.initialized


@pytest.mark.parametrize('fault', ['start_error', 'frame_error', 'encoded'])
def test_capture_failures_release_usb_and_ros(capture, fault):
    _, state, _ = capture
    setattr(state, fault, fault != 'encoded')
    module.capture('robot')
    assert state.stopped and state.destroyed and not state.initialized
    assert not any(state.published.values())


def test_sigterm_cleans_up_and_restores_previous_handler(capture, capsys):
    _, state, _ = capture
    previous = signal.getsignal(signal.SIGTERM)
    state.terminate = True
    module.capture('robot')
    assert state.stopped and state.destroyed and not state.initialized
    assert signal.getsignal(signal.SIGTERM) == previous
    assert '[camera_subprocess] error:' not in capsys.readouterr().out


def test_camera_plugin_ignores_old_fusion_flag_and_keeps_photo_frame_cache(capture, monkeypatch, tmp_path):
    device, _, _ = capture
    # Exercise the real runtime manager; replace only process creation/watch thread.
    opened = []
    def popen(args, **kwargs):
        proc = NS(pid=123, stdout=[])
        proc.poll = lambda: None
        opened.append(args)
        return proc
    monkeypatch.setattr(module.subprocess, 'Popen', popen)
    monkeypatch.setattr(module.CameraRuntime, 'start', lambda self: self.tick(0))
    camera = device.CameraPlugin({'fusion_enabled': True}, 'robot', NS(add_node=lambda node: None))
    cache = camera._frame_node
    camera.start()
    camera.start()
    assert len(opened) == 1 and opened[0][-2:] == ['capture', 'robot']
    msg = NS(header=NS(stamp=NS(sec=123, nanosec=456)), format='jpeg', data=b'JPEG')
    cache._on_frame(msg)
    frame, sequence = camera.wait_for_color_frame(timeout_s=0)
    assert frame['data'] == b'JPEG' and sequence == 1
    assert camera._frame_node is cache
    assert [tool['topic_out'][0]['topic'] for tool in camera.get_tools()] == [
        '/robot/camera/color', '/robot/camera/depth']
    vision = device.VisionCapturePlugin({'output_dir': str(tmp_path)}, camera)
    result = vision.dispatch('capture_photo', {})
    assert result['ok'] and Path(result['file_path']).read_bytes() == b'JPEG'
    assert len(opened) == 1  # Photo capture consumes the same cache/USB owner.
