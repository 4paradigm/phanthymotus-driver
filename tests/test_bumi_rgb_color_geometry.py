"""Native RGB coverage and measured color-camera Z, without camera hardware."""
import sys
from types import SimpleNamespace as NS
import zlib

import cv2
import numpy as np
import pytest

from noetix.bumi import rgbd_camera


@pytest.fixture
def capture(monkeypatch):
    class Message:
        def __init__(self):
            self.header = NS(stamp=NS(sec=0, nanosec=0), frame_id='')
    monkeypatch.setitem(sys.modules, 'sensor_msgs.msg', NS(CompressedImage=Message))
    pixels = np.full((480, 640, 3), 80, np.uint8)
    depth = np.full((480, 640), 500, '<u2')
    extrinsic = NS(rotation=[1., 0., 0., 0., 1., 0., 0., 0., 1.], translation=[0., 0., 0.])
    def profile(focal):
        intr = NS(width=640, height=480, fx=focal, fy=focal, ppx=320., ppy=240.,
                  model='none', coeffs=[0.] * 5)
        return NS(as_video_stream_profile=lambda: NS(intrinsics=intr),
                  get_extrinsics_to=lambda target: extrinsic)
    color_profile, depth_profile = profile(600.), profile(390.)
    def frame(data, calibration):
        return NS(get_data=lambda: data, profile=calibration, get_timestamp=lambda:1000.,
                  get_frame_timestamp_domain=lambda:'hardware_clock')
    sources = NS(get_color_frame=lambda:frame(pixels, color_profile),
                 get_depth_frame=lambda:frame(depth, depth_profile))
    targets = []
    def align(target):
        targets.append(target)
        def process(frames):
            if target == 'color':
                # SDK rasterization retains source-camera axial Z, including zeros.
                return NS(get_color_frame=frames.get_color_frame,
                          get_depth_frame=lambda:frame(depth, color_profile))
            # Color-to-depth cannot fill the wider depth field or missing depth.
            registered = pixels.copy()
            registered[:80] = registered[400:] = 0
            registered[:, :110] = registered[:, 530:] = 0
            registered[depth == 0] = 0
            return NS(get_color_frame=lambda:frame(registered, depth_profile),
                      get_depth_frame=frames.get_depth_frame)
        return NS(process=process)
    monkeypatch.setitem(sys.modules, 'pyrealsense2', NS(stream=NS(color='color', depth='depth'), align=align))
    mount = NS(status=lambda:dict(status='unavailable'), declaration=lambda:None,
               publish=lambda stamp:None, close=lambda:None)
    monkeypatch.setitem(sys.modules, 'common.camera_mount', NS(BodyMountSource=lambda *a, **kw:mount))
    messages, notices = {}, []
    node = NS(create_publisher=lambda cls, topic, qos:NS(publish=messages.setdefault(topic, []).append))
    pipeline = NS(get_active_profile=lambda:NS(get_device=lambda:NS(
        first_depth_sensor=lambda:NS(get_depth_scale=lambda:.002))))
    monkeypatch.setattr(rgbd_camera.time, 'time_ns', lambda:10_000_000_000)
    publisher = rgbd_camera.RgbdCameraPublisher(node, 'robot', pipeline, object(), report=notices.append)
    yield publisher, sources, messages, notices, extrinsic, depth
    publisher.close()


def decode(messages):
    rgb = cv2.imdecode(np.frombuffer(messages['/robot/camera/color'][0].data, np.uint8), cv2.IMREAD_COLOR)
    depth = np.frombuffer(zlib.decompress(messages['/robot/camera/depth'][0].data), '<u2').reshape(480,640)
    return rgb, depth


def test_native_rgb_field_is_visible_even_where_depth_is_missing(capture):
    publisher, sources, messages, notices, _, depth = capture
    depth[180:300, 250:390] = 0
    publisher.publish(sources)
    rgb, registered = decode(messages)
    assert rgb.shape == (480,640,3)
    assert np.all(rgb >= 75)  # includes all four edges and the depth hole
    assert registered[240,320] == 0
    assert registered[100,100] == 1000
    declarations = notices[-1]['camera_info']
    assert all(d['vendor']['frame_id'] == 'robot_rgb_optical' for d in declarations)
    assert all(d['K'] == [600.,0.,320.,0.,600.,240.,0.,0.,1.] for d in declarations)


def test_aligned_source_depth_is_converted_to_color_axial_z(capture):
    publisher, sources, messages, _, extrinsic, depth = capture
    extrinsic.translation = [0.,0.,.1]
    depth[240,320] = 0
    publisher.publish(sources)
    _, registered = decode(messages)
    assert registered[100,100] == 1100
    assert registered[240,320] == 0  # translation must not fill invalid depth


def test_color_axial_z_uses_real_rotation_and_each_color_ray(capture):
    publisher, sources, messages, _, extrinsic, _ = capture
    # Depth-to-color rotation: 15 degrees about Y; SDK stores column-major.
    c, s = .9659258262890683, .25881904510252074
    extrinsic.rotation = [c,0.,-s,0.,1.,0.,s,0.,c]
    publisher.publish(sources)
    _, registered = decode(messages)
    assert registered[240,20] == 1195
    assert registered[240,320] == 1035
    assert registered[240,620] == 913


@pytest.mark.parametrize('change', [
    dict(rotation=[1.]*9), dict(translation=[float('nan'),0.,0.]),
])
def test_invalid_depth_to_color_calibration_never_publishes_images(capture, change):
    publisher, sources, messages, _, extrinsic, _ = capture
    for key, value in change.items():setattr(extrinsic,key,value)
    with pytest.raises(ValueError, match='calibration'):
        publisher.publish(sources)
    assert not any(messages.values())


@pytest.mark.parametrize('kind', ['color', 'depth'])
def test_nonzero_source_distortion_cannot_be_silently_ignored_by_sdk_align(capture, kind):
    publisher, sources, messages, _, _, _ = capture
    source = sources.get_color_frame() if kind == 'color' else sources.get_depth_frame()
    intr = source.profile.as_video_stream_profile().intrinsics
    intr.model, intr.coeffs = 'brown_conrady', [.2,0.,0.,0.,0.]
    with pytest.raises(ValueError, match='distortion'):
        publisher.publish(sources)
    assert not any(messages.values())
