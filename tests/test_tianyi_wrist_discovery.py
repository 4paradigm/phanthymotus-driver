"""Camera capability negotiation and non-invasive USB discovery."""
import importlib
from pathlib import Path
import sys
from types import SimpleNamespace as NS

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'x-humanoid/tianyi2.0'))


def module():
    return importlib.import_module('wrist_camera_devices')


def profile(w=640, h=480, fps=15, format='rgb8'):
    return {'width': w, 'height': h, 'fps': fps, 'format': format}


def test_select_profiles_uses_capabilities_not_model_or_serial_length():
    d = {'profiles': {'rgb': [profile(fps=30), profile()],
                      'depth': [profile(format='z16'), profile(fps=30, format='z16')]}}
    selected = module().select_profiles(d, 'auto', 'auto')
    assert selected == {'rgb': profile(), 'depth': profile(format='z16')}
    assert module().select_profiles(d, '30', '640x480')['depth']['fps'] == 30


def test_only_rgb_device_keeps_native_supported_resolution():
    d = {'profiles': {'rgb': [profile(1280, 720, 30, 'MJPG')], 'depth': []}}
    assert module().select_profiles(d, 'auto', 'auto') == {'rgb': profile(1280, 720, 30, 'MJPG')}


def test_depth_requires_native_640_by_480_and_common_fps():
    d = {'profiles': {'rgb': [profile()], 'depth': [profile(848, 480, format='z16')]}}
    with pytest.raises(ValueError, match='640x480'):
        module().select_profiles(d, 'auto', 'auto')
    d['profiles']['depth'] = [profile(fps=30, format='z16')]
    with pytest.raises(ValueError, match='frame rate'):
        module().select_profiles(d, 'auto', 'auto')


def test_requested_unsupported_profile_is_not_silently_replaced():
    d = {'profiles': {'rgb': [profile()], 'depth': []}}
    for fps, resolution in [('30', 'auto'), ('auto', '1280x720')]:
        with pytest.raises(ValueError):
            module().select_profiles(d, fps, resolution)


def test_realsense_enumerates_rgbd_profiles_without_d405_whitelist():
    m = module()
    class Profile:
        def __init__(self, stream, format):
            self.stream, self.fmt = stream, format
        def stream_type(self): return self.stream
        def format(self): return self.fmt
        def fps(self): return 15
        def as_video_stream_profile(self): return NS(width=lambda: 640, height=lambda: 480)
    dev = NS(get_info=lambda key: {'serial': 'any-serial', 'name': 'Another RealSense', 'port': '/usb/2'}[key],
             query_sensors=lambda: [NS(get_stream_profiles=lambda: [Profile('color', 'rgb8'), Profile('depth', 'z16')])])
    rs = NS(context=lambda: NS(query_devices=lambda: [dev]), camera_info=NS(serial_number='serial', name='name', physical_port='port'),
            stream=NS(color='color', depth='depth'), format=NS(rgb8='rgb8', bgr8='bgr8', z16='z16'))
    entries = m.realsense_devices(rs)
    assert entries[0]['id'] == 'realsense:any-serial'
    assert entries[0]['capabilities'] == {'rgb': True, 'depth': True}
    assert entries[0]['profiles']['depth'] == [profile(format='z16')]


def test_v4l2_groups_interfaces_skips_metadata_and_vendor_owned_cameras(monkeypatch, tmp_path):
    m = module()
    nodes = [str(tmp_path / f'video{i}') for i in range(4)]
    for p in nodes: Path(p).touch()
    linkdir = tmp_path / 'by-id'
    linkdir.mkdir()
    (linkdir / 'usb-webcam-video-index0').symlink_to(nodes[0])
    infos = {
        nodes[0]: 'Card type : Generic USB Webcam\nBus info : usb-webcam\nDevice Caps : 0x1\n Video Capture\n',
        nodes[1]: 'Card type : Generic USB Webcam\nBus info : usb-webcam\nDevice Caps : 0x1\n Video Capture\n',
        nodes[2]: 'Card type : Metadata\nCapabilities : Video Capture\nDevice Caps : 0x1\n Metadata Capture\n',
        nodes[3]: 'Card type : Intel RealSense\nBus info : usb-depth\nDevice Caps : 0x1\n Video Capture\n',
    }
    formats = "[0]: 'MJPG' (Motion-JPEG)\n Size: Discrete 640x480\n  Interval: Discrete 0.066s (15.000 fps)\n"
    def run(args, **kwargs):
        return infos[args[2]] if args[-1] == '--info' else formats
    monkeypatch.setattr(m.subprocess, 'check_output', run)
    entries = m.uvc_devices(nodes, str(linkdir))
    assert len(entries) == 1
    assert entries[0]['path'] == str(linkdir / 'usb-webcam-video-index0')
    assert entries[0]['capabilities'] == {'rgb': True, 'depth': False}
    assert entries[0]['profiles']['rgb'] == [profile(format='MJPG')]


def test_v4l2_does_not_claim_head_orbbec_camera(monkeypatch, tmp_path):
    m = module()
    p = tmp_path / 'video0'
    p.touch()
    monkeypatch.setattr(m.subprocess, 'check_output', lambda *a, **k: 'Card type : Orbbec Gemini\nDevice Caps : 0x1\nVideo Capture\n')
    assert m.uvc_devices([str(p)], str(tmp_path / 'none')) == []


def test_uvc_identity_and_interface_survive_video_node_renumbering(monkeypatch, tmp_path):
    m = module()
    linkdir = tmp_path / 'by-id'
    linkdir.mkdir()
    first, second = linkdir / 'usb-camera-video-index0', linkdir / 'usb-camera-video-index1'
    nodes = [tmp_path / 'video2', tmp_path / 'video10', tmp_path / 'video0']
    for p in nodes: p.touch()
    info = 'Card type : USB Camera\nBus info : usb-camera-port\nDevice Caps : 0x1\n Video Capture\n'
    formats = "[0]: 'MJPG' (Motion-JPEG)\n Size: Discrete 640x480\n Interval: Discrete 0.066s (15.000 fps)\n"
    monkeypatch.setattr(m.subprocess, 'check_output', lambda args, **kw: info if args[-1] == '--info' else formats)
    first.symlink_to(nodes[0])
    second.symlink_to(nodes[1])
    before, = m.uvc_devices([str(nodes[0]), str(nodes[1])], str(linkdir))
    first.unlink()
    second.unlink()
    first.symlink_to(nodes[2])
    second.symlink_to(nodes[0])
    after, = m.uvc_devices([str(nodes[2]), str(nodes[0])], str(linkdir))
    assert before['id'] == after['id'] == 'uvc:usb-camera'
    assert before['path'] == after['path'] == str(first)


def test_rgb_resolution_can_change_while_native_depth_remains_640_by_480():
    d = {'profiles': {'rgb': [profile(), profile(1280, 720)], 'depth': [profile(format='z16')]}}
    selected = module().select_profiles(d, 'auto', '1280x720')
    assert selected == {'rgb': profile(1280, 720), 'depth': profile(format='z16')}
