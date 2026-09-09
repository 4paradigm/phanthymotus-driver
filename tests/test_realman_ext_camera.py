"""RealMan upper-computer camera discovery and card contract regressions."""
import sys
from contextlib import nullcontext
import types
import unittest
import numpy as np  # Load before patching sys.modules so NumPy is not reimported.
from pathlib import Path
from unittest import mock


def load_camera():
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
    path = Path(__file__).resolve().parents[1] / 'realman/rm75_6f_v/camera.py'
    module = types.ModuleType('realman_ext_camera_test')
    with mock.patch.dict(sys.modules, stubs):
        exec(compile('from __future__ import annotations\n' + path.read_text(),
                     str(path), 'exec'), module.__dict__)
    return module


ext = load_camera()


class CameraDiscoveryTest(unittest.TestCase):
    def enumerate(self, devices, probe_usb=False):
        def query(path):
            name, caps, formats = devices[path]
            if caps != 'Video Capture' or formats is None:
                return {}
            return {'name': name, 'formats': list(formats),
                    'resolutions': ['1280x720'] if formats else []}
        usb = (nullcontext() if probe_usb else mock.patch.object(
            ext, '_realsense_usb_path', return_value='/sys/devices/test-usb'))
        with usb, mock.patch.object(ext.glob, 'glob', return_value=list(devices)), \
             mock.patch.object(ext, '_query_v4l2_device', side_effect=query):
            return ext._enumerate_ext_cameras()

    def test_realsense_exposes_only_color_among_six_nodes(self):
        name = 'Intel(R) RealSense(TM) Depth Ca'
        formats = [['Z16 '], [], ['GREY', 'UYVY', 'Y8I '], [], ['YUYV'], []]
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

    def test_real_v4l2_device_caps_layout(self):
        def ioctl(_fd, request, buffer):
            if request == ext._VIDIOC_QUERYCAP:
                buffer[16:16 + len(b'Intel RealSense D435')] = b'Intel RealSense D435'
                ext.struct.pack_into('=I', buffer, 84, ext._V4L2_CAP_DEVICE_CAPS)
                ext.struct.pack_into('=I', buffer, 88, ext._V4L2_CAP_VIDEO_CAPTURE)
            elif request == ext._VIDIOC_ENUM_FMT:
                if ext.struct.unpack_from('=I', buffer, 0)[0] > 0:
                    raise OSError(ext.errno.EINVAL, 'done')
                ext.struct.pack_into('=I', buffer, 44, int.from_bytes(b'YUYV', 'little'))
            elif request == ext._VIDIOC_ENUM_FRAMESIZES:
                if ext.struct.unpack_from('=I', buffer, 0)[0] > 0:
                    raise OSError(ext.errno.EINVAL, 'done')
                ext.struct.pack_into('=III', buffer, 8, 1, 1280, 720)
            else:
                self.fail(f'unexpected ioctl {request:#x}')

        with mock.patch.object(ext.os, 'open', return_value=7) as opened, \
             mock.patch.object(ext.os, 'close') as closed, \
             mock.patch.object(ext, '_ioctl', side_effect=ioctl):
            result = ext._query_v4l2_device('/dev/video4')
        self.assertEqual(result, {'name': 'Intel RealSense D435',
                                  'formats': ['YUYV'], 'resolutions': ['1280x720']})
        opened.assert_called_once_with('/dev/video4', ext.os.O_RDONLY | ext.os.O_NONBLOCK)
        closed.assert_called_once_with(7)

    def test_metadata_capability_is_rejected_before_format_probe(self):
        calls = []

        def ioctl(_fd, request, buffer):
            calls.append(request)
            ext.struct.pack_into('=I', buffer, 84, ext._V4L2_CAP_DEVICE_CAPS)
            ext.struct.pack_into('=I', buffer, 88, 0x00800000)

        with mock.patch.object(ext.os, 'open', return_value=8), \
             mock.patch.object(ext.os, 'close'), \
             mock.patch.object(ext, '_ioctl', side_effect=ioctl):
            self.assertEqual(ext._query_v4l2_device('/dev/video1'), {})
        self.assertEqual(calls, [ext._VIDIOC_QUERYCAP])

    def test_realsense_unknown_format_is_not_exposed_as_rgb(self):
        self.assertEqual(self.enumerate({
            '/dev/video4': ('Intel RealSense', 'Video Capture', ['ABCD']),
        }), [])

    def test_sysfs_failure_does_not_hide_an_unrelated_webcam(self):
        devices = {
            '/dev/video4': ('Intel RealSense', 'Video Capture', ['YUYV']),
            '/dev/video6': ('USB Webcam', 'Video Capture', ['MJPG']),
        }
        for error in (FileNotFoundError('unplugged'), OSError('sysfs unavailable')):
            with self.subTest(error=error), \
                 mock.patch.object(ext.Path, 'resolve', side_effect=error):
                result = self.enumerate(devices, probe_usb=True)
                self.assertEqual([d['path'] for d in result], ['/dev/video6'])

    def test_disappearing_usb_ancestor_does_not_abort_enumeration(self):
        with mock.patch.object(ext.Path, 'resolve', return_value=Path('/sys/devices/usb1/1-1/video4')), \
             mock.patch.object(ext.Path, 'is_file', side_effect=OSError('unplugged')):
            self.assertEqual(ext._realsense_usb_path('/dev/video4'), '')

    def test_usb_identity_comes_from_the_physical_ancestor(self):
        with mock.patch.object(ext.Path, 'resolve', return_value=Path('/sys/devices/usb1/1-1/video4')), \
             mock.patch.object(ext.Path, 'is_file', autospec=True,
                               side_effect=lambda p: p == Path('/sys/devices/usb1/1-1/idVendor')):
            self.assertEqual(ext._realsense_usb_path('/dev/video4'), '/sys/devices/usb1/1-1')


class FakeRGB:
    def __init__(self, *args, **kwargs):
        self.state = 'idle'
        self.starts = 0

    def start(self):
        self.starts += 1
        self.state = 'running'
        return self._status_dict()

    def stop(self):
        self.state = 'idle'
        return self._status_dict()

    def _status_dict(self):
        return {'state': self.state}


class FakeStereo:
    def __init__(self, namespace, usb_path):
        self.usb_path = usb_path
        self.routes = {}

    def start(self, instance_id, channel):
        self.routes[instance_id] = channel
        return self.info(instance_id, channel)

    def stop(self, instance_id):
        self.routes.pop(instance_id, None)

    def info(self, instance_id, channel):
        return {'state': 'running' if instance_id in self.routes else 'idle', 'channel': channel}


class CameraChannelTests(unittest.TestCase):
    def setUp(self):
        self.devices = [dict(path='/dev/video4', name='D435', formats=['YUYV'],
                             resolutions=['1280x720', '640x480'], realsense=True, usb_path='test-a'),
                        dict(path='/dev/video8', name='Webcam', formats=['MJPG'],
                             resolutions=['1280x720'], realsense=False, usb_path='')]
        self.enumeration = mock.patch.object(ext, '_enumerate_ext_cameras', return_value=self.devices)
        self.enumeration.start()
        self.addCleanup(self.enumeration.stop)
        self.rgb = mock.patch.object(ext, '_ExtCameraNode', FakeRGB)
        self.rgb.start()
        self.addCleanup(self.rgb.stop)
        self.sdk = mock.patch.dict(sys.modules, {'realsense': types.SimpleNamespace(RealSenseSession=FakeStereo)})
        self.sdk.start()
        self.addCleanup(self.sdk.stop)
        self.plugin = ext.ExtCameraPlugin({}, 'robot_a', None)

    def test_one_pure_sensor_with_channel_configuration(self):
        tools = self.plugin.get_tools()
        self.assertEqual([t['name'] for t in tools], ['ext_camera'])
        self.assertTrue(tools[0]['multiInstance'])
        self.assertEqual(tools[0]['inputSchema'], {'type': 'object', 'properties': {}})
        channel = tools[0]['configSchema']['properties']['channel']
        self.assertEqual(channel['enum'], ['rgb', 'depth', 'infrared'])
        self.assertEqual(channel['default'], 'rgb')
        self.assertEqual(channel['scope'], 'instance')

    def test_running_card_switches_all_modalities_and_topics(self):
        args = {'instance_id': 'card-a'}
        self.assertEqual(self.plugin.dispatch('start', args)['channel'], 'rgb')
        original = self.plugin._nodes['card-a']
        for channel in ('depth', 'infrared', 'rgb'):
            result = self.plugin.dispatch('config', {**args, 'channel': channel})
            self.assertEqual(result['state'], 'running')
            expected_format = 'image/depth-zlib' if channel == 'depth' else 'image/jpeg'
            self.assertEqual(result['topic_out'], [{'topic': f'/robot_a/ext_camera/card_a/{channel}', 'format': expected_format}])
        self.assertEqual(original.state, 'idle')

    def test_idle_config_infers_selected_topic_without_starting(self):
        result = self.plugin.dispatch('config', {'instance_id':'card-a', 'channel':'depth'})
        self.assertEqual(result['state'], 'idle')
        self.assertEqual(result['topic_out'][0]['format'], 'image/depth-zlib')
        self.assertEqual(self.plugin._nodes, {})

    def test_regular_camera_cannot_switch_to_depth_or_lose_working_rgb(self):
        args = {'instance_id':'card-a', 'device_path':'/dev/video8'}
        self.plugin.dispatch('start', args)
        node = self.plugin._nodes['card-a']
        with self.assertRaisesRegex(ValueError, 'RealSense'):
            self.plugin.dispatch('config', {**args, 'channel':'depth'})
        self.assertEqual(node.state, 'running')
        self.assertEqual(self.plugin.dispatch('info', args)['channel'], 'rgb')

    def test_instances_share_stereo_owner_and_stopping_one_preserves_other(self):
        self.plugin.dispatch('start', {'instance_id':'card-a', 'channel':'depth'})
        self.plugin.dispatch('start', {'instance_id':'card-b', 'channel':'infrared'})
        session = self.plugin._sessions['test-a']
        self.assertEqual(session.routes, {'card-a':'depth', 'card-b':'infrared'})
        self.plugin.dispatch('stop', {'instance_id':'card-a'})
        self.assertEqual(session.routes, {'card-b':'infrared'})

    def test_saved_channel_configs_are_isolated_when_project_starts_each_card(self):
        channels = {'card-a': 'rgb', 'card-b': 'depth', 'card-c': 'infrared'}
        for ident, channel in channels.items():
            self.plugin.dispatch('config', {'instance_id': ident, 'channel': channel})
        outputs = []
        for ident, channel in channels.items():
            result = self.plugin.dispatch('start', {'instance_id': ident})
            self.assertEqual(result['state'], 'running')
            self.assertEqual(result['channel'], channel)
            outputs.append(result['topic_out'][0]['topic'])
        self.assertEqual(len(set(outputs)), 3)
        self.assertEqual(set(self.plugin._nodes), set(channels))

    def test_start_reports_activation_but_info_keeps_pending_frame_readiness(self):
        with mock.patch.object(FakeStereo, 'info', return_value={'state': 'starting', 'fresh': False}):
            result = self.plugin.dispatch('start', {'instance_id': 'card-a', 'channel': 'depth'})
            self.assertEqual(result['state'], 'running')
            self.assertEqual(result['readiness'], 'starting')
            self.assertFalse(result['fresh'])
            self.assertEqual(self.plugin.dispatch('info', {'instance_id': 'card-a'})['state'], 'starting')

    def test_start_does_not_mask_a_known_capture_error(self):
        with mock.patch.object(FakeStereo, 'info', return_value={
                'state': 'error', 'fresh': False, 'error': 'Device disconnected'}):
            result = self.plugin.dispatch('start', {'instance_id': 'card-a', 'channel': 'depth'})
            self.assertEqual(result['state'], 'error')
            self.assertEqual(result['error'], 'Device disconnected')

    def test_selected_device_usb_path_is_used(self):
        self.devices.append({**self.devices[0], 'path':'/dev/video10', 'usb_path':'test-b'})
        self.plugin.dispatch('start', {'instance_id':'card-a', 'channel':'infrared', 'device_path':'/dev/video10'})
        self.assertEqual(list(self.plugin._sessions), ['test-b'])

    def test_repeated_config_does_not_restart_and_ids_do_not_collide(self):
        args = {'instance_id':'card-a'}
        self.plugin.dispatch('start', args)
        node = self.plugin._nodes['card-a']
        self.plugin.dispatch('config', {**args, 'channel':'rgb'})
        self.assertIs(node, self.plugin._nodes['card-a'])
        self.assertEqual(node.starts, 1)
        with self.assertRaisesRegex(ValueError, 'collides'):
            self.plugin.dispatch('start', {'instance_id':'card_a'})


if __name__ == '__main__':
    unittest.main()
