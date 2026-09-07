"""Camera discovery regression tests using V4L2 output from the Go2 host."""
import sys
import types
import unittest
import numpy as np  # Load before patching sys.modules so NumPy is not reimported.
from pathlib import Path
from unittest import mock


def load_ext_devices():
    stubs = {name: types.ModuleType(name) for name in (
        'cv2', 'rclpy', 'rclpy.node', 'rclpy.qos', 'std_msgs.msg',
        'sensor_msgs.msg', 'audio_msgs.msg',
    )}
    stubs['rclpy.node'].Node = object
    qos = stubs['rclpy.qos']
    qos.QoSProfile = lambda **kwargs: None
    qos.ReliabilityPolicy = types.SimpleNamespace(BEST_EFFORT=1)
    qos.HistoryPolicy = types.SimpleNamespace(KEEP_LAST=1)
    qos.DurabilityPolicy = types.SimpleNamespace(VOLATILE=1)
    stubs['std_msgs.msg'].Header = object
    stubs['sensor_msgs.msg'].CompressedImage = object
    stubs['audio_msgs.msg'].AudioChunk = object
    path = Path(__file__).resolve().parents[1] / 'unitree/go2/ext_devices.py'
    module = types.ModuleType('go2_ext_camera_test')
    with mock.patch.dict(sys.modules, stubs):
        exec(compile('from __future__ import annotations\n' + path.read_text(),
                     str(path), 'exec'), module.__dict__)
    return module


ext = load_ext_devices()


class CameraDiscoveryTest(unittest.TestCase):
    def enumerate(self, devices):
        def output(args, **kwargs):
            name, caps, formats = devices[args[2]]
            if args[3] == '--info':
                return (f'Card type : {name}\nCapabilities : 0x84a00001\n'
                        f'\tVideo Capture\n\tMetadata Capture\n'
                        f'Device Caps : 0x04200001\n\t{caps}\n\tStreaming\n')
            if formats is None:
                raise ext.subprocess.CalledProcessError(1, args)
            return '\n'.join(f"[{i}]: '{fmt}'\n\tSize: Discrete 1280x720"
                             for i, fmt in enumerate(formats))
        with mock.patch.object(ext.glob, 'glob', return_value=list(devices)), \
             mock.patch.object(ext.subprocess, 'check_output', side_effect=output):
            return ext._enumerate_ext_cameras()

    def test_realsense_exposes_only_color_among_six_nodes(self):
        name = 'Intel(R) RealSense(TM) Depth Ca'
        formats = [('Z16 ',), (), ('GREY', 'UYVY', 'Y8I '), (), ('YUYV',), ()]
        devices = {f'/dev/video{i}': (name, 'Metadata Capture' if i % 2 else 'Video Capture', f)
                   for i, f in enumerate(formats)}
        result = self.enumerate(devices)
        self.assertEqual([d['path'] for d in result], ['/dev/video4'])
        self.assertEqual(result[0]['formats'], ['YUYV'])
        self.assertEqual(result[0]['resolutions'], ['1280x720'])

    def test_metadata_is_not_capture_even_when_format_probe_fails(self):
        self.assertEqual(self.enumerate({
            '/dev/video1': ('USB Webcam', 'Metadata Capture', None),
        }), [])

    def test_unprobed_realsense_is_not_assumed_to_be_color(self):
        self.assertEqual(self.enumerate({
            '/dev/video0': ('Intel RealSense', 'Video Capture', None),
        }), [])

    def test_regular_usb_webcam_is_preserved(self):
        result = self.enumerate({
            '/dev/video6': ('USB Webcam', 'Video Capture', ('MJPG', 'YUYV')),
            '/dev/video7': ('USB Webcam', 'Metadata Capture', ()),
        })
        self.assertEqual([d['path'] for d in result], ['/dev/video6'])
        self.assertEqual(result[0]['formats'], ['MJPG', 'YUYV'])


if __name__ == '__main__':
    unittest.main()
