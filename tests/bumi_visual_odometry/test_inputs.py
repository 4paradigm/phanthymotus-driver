"""Raw RGB-D inputs for the bundle-local visual odometry card.

Camera-only port of PR #425's `tests/bumi_fusion/test_inputs.py`: the restored
bundle publishes raw RGB/depth/CameraInfo/provenance and nothing with an IMU.

Run: python3 -m pytest -q tests/bumi_visual_odometry/test_inputs.py
"""
from array import array
import json
from pathlib import Path
import sys
from types import SimpleNamespace as NS

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

import noetix.bumi.visual_inputs as visual_inputs  # noqa: E402
from noetix.bumi.visual_inputs import (  # noqa: E402
    RawCameraPublisher, camera_metadata)

HARDWARE = 'timestamp_domain.hardware_clock'


def frame(ms, number=1, domain=HARDWARE):
    return NS(get_timestamp=lambda: ms, get_frame_number=lambda: number,
              get_frame_timestamp_domain=lambda: domain)


def test_camera_keeps_distinct_capture_times_and_depth_scale():
    meta = camera_metadata(frame(1000), frame(1010), epoch='e', depth_scale=.002,
                           received_ns=10_000_000_000)
    assert meta['rgb']['source_ms'] == 1000
    assert meta['depth']['source_ms'] == 1010
    assert meta['rgb']['frame_number'] == 1
    assert meta['rgb']['domain'] == 'hardware_clock'
    assert meta['depth_scale'] == .002
    assert meta['epoch'] == 'e' and meta['received_ns'] == 10_000_000_000
    json.dumps(meta, allow_nan=False)


def test_camera_rejects_sdk_depth_scale_outside_the_physical_range():
    with pytest.raises(ValueError):
        camera_metadata(frame(1000), frame(1000), epoch='e', depth_scale=0,
                        received_ns=1)
    with pytest.raises(ValueError):
        camera_metadata(frame(1000), frame(1000), epoch='e', depth_scale=.5,
                        received_ns=1)


@pytest.fixture
def raw_ros(monkeypatch):
    """ROS message classes and a node whose publishers just record messages."""
    class Image:
        def __init__(self):
            self.header = NS(stamp=NS(sec=0, nanosec=0), frame_id='')
    class String:
        def __init__(self, data=''):
            self.data = data
    messages = {}
    node = NS(create_publisher=lambda cls, topic, qos: NS(
        publish=messages.setdefault(topic, []).append))
    monkeypatch.setitem(sys.modules, 'sensor_msgs.msg', NS(Image=Image, CameraInfo=Image))
    monkeypatch.setitem(sys.modules, 'std_msgs.msg', NS(String=String))
    return messages, node


def publisher(raw_ros, depth_scale=.002):
    messages, node = raw_ros
    pipeline = NS(get_active_profile=lambda: NS(
        get_device=lambda: NS(first_depth_sensor=lambda: NS(
            get_depth_scale=lambda: depth_scale))))
    return RawCameraPublisher(node, 'robot', pipeline, object()), messages


def frames(model='distortion.brown_conrady', coeffs=None, ms=1000):
    intr = NS(fx=100., fy=101., ppx=1., ppy=1., model=model,
              coeffs=[0.]*5 if coeffs is None else coeffs)
    color = frame(ms)
    color.get_data = lambda: np.zeros((2, 3, 3), dtype='u1')
    color.profile = NS(as_video_stream_profile=lambda: NS(intrinsics=intr))
    depth = frame(ms+10)
    depth.get_data = lambda: np.full((2, 3), 500, dtype='<u2')
    return NS(get_color_frame=lambda: color, get_depth_frame=lambda: depth)


def _stamp(msg):
    return msg.header.stamp.sec*10**9+msg.header.stamp.nanosec


def test_shared_raw_publisher_uses_sdk_scale_and_separate_source_times(raw_ros):
    raw_publisher, messages = publisher(raw_ros)
    aligned = frames()
    raw_publisher.publish(aligned, aligned)

    raw = messages['/robot/odometry_input/depth'][0]
    # Humble's uint8[] setter scans bytes in Python; array('B') takes its
    # buffer fast path (about 402 ms versus 0.22 ms per RGB-D pair on Bumi).
    assert isinstance(raw.data, array) and raw.data.typecode == 'B'
    assert raw.encoding == '32FC1' and raw.step == 12
    assert np.frombuffer(raw.data, dtype='<f4').tolist() == pytest.approx([1.]*6)
    rgb = messages['/robot/odometry_input/rgb'][0]
    assert isinstance(rgb.data, array) and rgb.data.typecode == 'B'
    assert rgb.encoding == 'bgr8' and rgb.step == 9
    assert _stamp(raw)-_stamp(rgb) == 10_000_000
    meta = json.loads(messages['/robot/odometry_input/camera_meta'][0].data)
    assert meta['depth_scale'] == .002 and meta['rgb']['source_ms'] == 1000
    assert meta['depth']['source_ms'] == 1010
    assert meta['rgb_stamp_ns'] == _stamp(rgb)
    assert meta['depth_stamp_ns'] == _stamp(raw)
    assert meta['clock_mapping'] == 'device_to_host'


def test_estimator_forwarding_keeps_the_byte_array_fast_path(raw_ros):
    from noetix.bumi.visual_odometry import VisualOdometryNode
    from noetix.bumi.visual_odometry_contract import Health

    raw_publisher, messages = publisher(raw_ros)
    aligned = frames()
    raw_publisher.publish(aligned, aligned)
    rgb = messages['/robot/odometry_input/rgb'][0]
    depth = messages['/robot/odometry_input/depth'][0]
    info = messages['/robot/odometry_input/info'][0]
    meta = json.loads(messages['/robot/odometry_input/camera_meta'][0].data)
    stamp = _stamp(rgb)
    # Exercise real validation/forwarding without ROS or an estimator child.
    node = VisualOdometryNode.__new__(VisualOdometryNode)
    node.cache = {stamp: meta}
    node.images = {stamp: (rgb, depth, info)}
    node.health = Health('seg', None, 0)
    node.now = lambda: stamp+20_000_000
    node.frame_id = node.optics = node.maps = None
    node.children = NS(children=[object()])
    forwarded = {'rgb': [], 'depth': [], 'info': []}
    for name in forwarded:
        setattr(node, name+'_pub', NS(publish=forwarded[name].append))

    node._prepare(stamp)

    for name, source in (('rgb', rgb), ('depth', depth)):
        output = forwarded[name][0]
        assert isinstance(output.data, array) and output.data.typecode == 'B'
        assert bytes(output.data) == bytes(source.data)
        assert output.encoding == source.encoding
        assert output.step == source.step
        assert _stamp(output) == _stamp(source)


def test_zero_coefficient_inverse_brown_conrady_is_the_pinhole_model(raw_ros):
    """The field D435i reports inverse_brown_conrady with all-zero coefficients."""
    raw_publisher, messages = publisher(raw_ros)
    aligned = frames('distortion.inverse_brown_conrady', [0.]*5)
    raw_publisher.publish(aligned, aligned)

    assert set(messages) == {'/robot/odometry_input/rgb', '/robot/odometry_input/depth',
                             '/robot/odometry_input/info', '/robot/odometry_input/camera_meta'}
    info = messages['/robot/odometry_input/info'][0]
    assert info.distortion_model == 'plumb_bob'
    assert info.d == [0.]*5
    assert info.k == [100., 0., 1., 0., 101., 1., 0., 0., 1.]


def test_nonzero_inverse_brown_conrady_is_rejected_before_publishing(raw_ros, capsys):
    raw_publisher, messages = publisher(raw_ros)
    raw_publisher.error_at = -10.0  # monotonic may be process-relative; force the log
    aligned = frames('distortion.inverse_brown_conrady', [0., .1, 0., 0., 0.])
    raw_publisher.publish(aligned, aligned)

    assert not any(messages.values())
    assert 'unsupported color distortion model' in capsys.readouterr().out


def test_non_finite_coefficients_are_rejected_before_publishing(raw_ros):
    raw_publisher, messages = publisher(raw_ros)
    aligned = frames('distortion.brown_conrady', [0., float('nan'), 0., 0., 0.])
    raw_publisher.publish(aligned, aligned)
    assert not any(messages.values())


def test_raw_publisher_samples_at_the_configured_max_rate(raw_ros, monkeypatch):
    raw_publisher, messages = publisher(raw_ros)
    raw_publisher.min_interval_s = .1
    raw_publisher.last_publish = 0.

    class FakeTime:
        ticks = iter([100., 100.05, 100.2])
        monotonic_ns = staticmethod(lambda: 1_000_000_000)
        time_ns = staticmethod(lambda: 1_000_000_000)

        @staticmethod
        def monotonic():
            return next(FakeTime.ticks)

    monkeypatch.setattr(visual_inputs, 'time', FakeTime)
    first, second, third = frames(), frames(), frames(ms=1030)
    raw_publisher.publish(first, first)     # t=100.00 -> publish
    raw_publisher.publish(second, second)   # t=100.05 -> skipped
    raw_publisher.publish(third, third)     # t=100.20 -> publish
    assert len(messages['/robot/odometry_input/rgb']) == 2
    assert len(messages['/robot/odometry_input/depth']) == 2
    assert len(messages['/robot/odometry_input/info']) == 2
    assert len(messages['/robot/odometry_input/camera_meta']) == 2
