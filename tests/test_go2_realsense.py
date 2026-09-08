"""Wire format and shared stereo ownership; USB acquisition is tested on Go2."""
import importlib.util
from pathlib import Path
import queue
import sys
import threading
import types
import unittest
from unittest import mock
import zlib

import numpy as np

SOURCE = Path(__file__).resolve().parents[1] / 'unitree/go2/realsense.py'
spec = importlib.util.spec_from_file_location('go2_realsense', SOURCE)
rs = importlib.util.module_from_spec(spec)
spec.loader.exec_module(rs)


class DeviceBindingTests(unittest.TestCase):
    USB = '/sys/devices/platform/3610000.xhci/usb2/2-1'
    # D435i + pinned SDK on Go2: physical_port is the depth V4L2 sysfs path,
    # whereas the selected RGB interface is 2-1:1.3 under the same USB parent.
    V4L2_PORT = USB + '/2-1:1.0/video4linux/video0'

    def test_real_linux_physical_port_matches_the_selected_rgb_usb_parent(self):
        self.assertTrue(rs.matches_usb_device(self.V4L2_PORT, self.USB))
        self.assertFalse(rs.matches_usb_device(
            self.USB + '0/2-10:1.0/video4linux/video0', self.USB))
        self.assertFalse(rs.matches_usb_device(self.V4L2_PORT, self.USB + '0'))

    def test_rsusb_identifier_matches_bus_port_chain_and_device_address(self):
        # librealsense v2.56.5 get_device_path(): bus-port.chain-device_address.
        fields = {'busnum': '2\n', 'devpath': '1.3\n', 'devnum': '7\n'}
        with mock.patch.object(rs.Path, 'read_text', autospec=True,
                               side_effect=lambda p: fields[p.name]):
            self.assertTrue(rs.matches_usb_device('2-1.3-7', self.USB))
            for unrelated in ('3-1.3-7', '2-1.30-7', '2-1.3-8', '2-1-7'):
                with self.subTest(port=unrelated):
                    self.assertFalse(rs.matches_usb_device(unrelated, self.USB))

    def test_unknown_or_unavailable_usb_identity_does_not_pick_a_device(self):
        for port in ('', 'unknown', '2-1', '/dev/video0'):
            with self.subTest(port=port):
                self.assertFalse(rs.matches_usb_device(port, self.USB))
        with mock.patch.object(rs.Path, 'read_text', side_effect=FileNotFoundError('unplugged')):
            self.assertFalse(rs.matches_usb_device('2-1-7', self.USB))
        with mock.patch.object(rs.Path, 'read_text', return_value='invalid'):
            self.assertFalse(rs.matches_usb_device('2-1-7', self.USB))


class DepthEncodingTests(unittest.TestCase):
    def test_converts_device_units_to_millimetres_and_preserves_invalid(self):
        raw = np.zeros((480, 640), dtype=np.uint16)
        raw[0, :4] = [0, 4000, 8000, 65535]
        decoded = np.frombuffer(zlib.decompress(rs.encode_depth(raw, .00025)), dtype='<u2')
        self.assertEqual(decoded.size, 640*480)
        np.testing.assert_array_equal(decoded[:4], [0, 1000, 2000, 16384])

    def test_depth_overflow_does_not_wrap_to_a_near_object(self):
        raw = np.full((480, 640), 40000, dtype=np.uint16)
        decoded = np.frombuffer(zlib.decompress(rs.encode_depth(raw, .002)), dtype='<u2')
        self.assertFalse(np.any(decoded))

    def test_bad_depth_shape_dtype_and_scale_are_rejected(self):
        raw = np.zeros((480, 640), dtype=np.uint16)
        for scale in (0, -1, float('nan'), float('inf')):
            with self.subTest(scale=scale), self.assertRaises(ValueError):
                rs.encode_depth(raw, scale)
        for bad in (raw[:240], raw.astype(np.uint8)):
            with self.assertRaises(ValueError):
                rs.encode_depth(bad, .001)


class StatusQueue(queue.Queue):
    def cancel_join_thread(self):
        pass

    def close(self):
        pass


class FakeProcess:
    def __init__(self, *, target, args, **kwargs):
        self.quit = args[4]
        self.alive = False
        self.closed = False

    def start(self):
        self.alive = True

    def is_alive(self):
        return self.alive

    def join(self, timeout):
        if self.quit.is_set():
            self.alive = False

    def terminate(self):
        self.alive = False

    kill = terminate

    def close(self):
        self.closed = True


class StereoLifecycleTests(unittest.TestCase):
    def setUp(self):
        self.clock = mock.patch.object(rs.time, 'monotonic', return_value=100.0)
        self.now = self.clock.start()
        self.addCleanup(self.clock.stop)
        self.session = rs.RealSenseSession('robot_a', '/sys/devices/test-usb')
        self.session._ctx = types.SimpleNamespace(
            Event=threading.Event, Queue=StatusQueue, Process=FakeProcess)
        self.addCleanup(self.session.stop, 'card-a')
        self.addCleanup(self.session.stop, 'card-b')

    def report(self, routes, error=None):
        self.session._queue.put({'frames': {key: 1 for key in routes},
                                 'last_frame': {key: self.now.return_value for key in routes},
                                 'channels': dict(routes), 'error': error, 'depth_scale_m': .001})

    def test_start_and_stale_status_require_real_frames_for_the_channel(self):
        self.assertEqual(self.session.start('card-a', 'depth')['state'], 'starting')
        self.report({'card-a': 'depth'})
        self.assertTrue(self.session.info('card-a', 'depth')['fresh'])
        self.now.return_value += 3.1
        info = self.session.info('card-a', 'depth')
        self.assertEqual(info['state'], 'error')
        self.assertFalse(info['fresh'])

    def test_shared_owner_fanout_and_last_stop_release(self):
        self.session.start('card-a', 'depth')
        process = self.session._proc
        self.session.start('card-b', 'infrared')
        self.report({'card-a': 'depth', 'card-b': 'infrared'})
        self.assertIs(self.session._proc, process)
        self.assertEqual(self.session.stop('card-a')['state'], 'idle')
        self.assertTrue(process.alive)
        self.assertEqual(self.session.info('card-b', 'infrared')['state'], 'running')
        self.session.stop('card-b')
        self.assertTrue(process.closed)
        self.assertIsNone(self.session._proc)

    def test_channel_switch_rejects_inflight_frames_from_old_channel(self):
        self.session.start('card-a', 'depth')
        self.report({'card-a': 'depth'})
        process = self.session._proc
        self.now.return_value += 1
        self.assertEqual(self.session.start('card-a', 'infrared')['state'], 'starting')
        self.now.return_value += .1
        self.report({'card-a': 'depth'})  # A previous frame was in flight during config.
        self.assertFalse(self.session.info('card-a', 'infrared')['fresh'])
        self.report({'card-a': 'infrared'})
        info = self.session.info('card-a', 'infrared')
        self.assertTrue(info['fresh'])
        self.assertIs(self.session._proc, process)
        self.assertEqual(info['topic_out'][0]['topic'], '/robot_a/ext_camera/card_a/infrared')
        self.assertEqual(info['topic_out'][0]['format'], 'image/jpeg')

    def test_retry_restores_all_instances_after_worker_failure(self):
        self.session.start('card-a', 'depth')
        self.session.start('card-b', 'infrared')
        self.report({}, error='Device unavailable')
        self.assertEqual(self.session.info('card-a', 'depth')['state'], 'error')
        old = self.session._proc
        self.session.start('card-a', 'depth')
        self.assertTrue(old.closed)
        self.assertEqual(self.session._wanted, {'card-a':'depth', 'card-b':'infrared'})
        self.session._proc.alive = False
        self.assertEqual(self.session.info('card-b', 'infrared')['state'], 'error')

    def test_startup_timeout_is_not_running(self):
        self.session.start('card-a', 'depth')
        self.now.return_value += 10.1
        self.assertEqual(self.session.info('card-a', 'depth')['state'], 'error')


class StereoCaptureTests(unittest.TestCase):
    """Exercise the real worker loop with timed SDK frames and no ROS hardware."""

    def capture(self, samples, usb_path='/sys/devices/usb1/1-1', physical_port=None):
        clock = [100.0]
        quit_event = threading.Event()
        statuses = queue.Queue()
        samples = iter(samples)

        def profile(kind, fmt, index):
            return types.SimpleNamespace(
                stream_type=lambda: kind, format=lambda: fmt,
                stream_index=lambda: index, fps=lambda: 6,
                as_video_stream_profile=lambda: types.SimpleNamespace(
                    width=lambda: 640, height=lambda: 480))

        profiles = [profile('depth', 'z16', 0), profile('infrared', 'y8', 1)]

        def wait_frame(timeout):
            try:
                elapsed, kind = next(samples)
            except StopIteration:
                quit_event.set()
                return False, None
            clock[0] = 100.0 + elapsed
            p = next((p for p in profiles if p.stream_type() == kind), None)
            return (True, types.SimpleNamespace(profile=p)) if p else (False, None)

        sensor = mock.Mock()
        sensor.get_depth_scale.return_value = .001
        sensor.get_stream_profiles.return_value = profiles
        device = types.SimpleNamespace(
            supports=lambda key: True,
            get_info=lambda key: {'physical_port': physical_port or usb_path + '/1-1:1.0/video4linux/video0',
                                  'usb_type_descriptor': '2.1', 'name': 'D435i'}[key],
            first_depth_sensor=lambda: sensor)
        sdk = types.SimpleNamespace(
            context=lambda: types.SimpleNamespace(query_devices=lambda: [device]),
            camera_info=types.SimpleNamespace(physical_port='physical_port',
                usb_type_descriptor='usb_type_descriptor', name='name'),
            stream=types.SimpleNamespace(depth='depth', infrared='infrared'),
            format=types.SimpleNamespace(z16='z16', y8='y8'),
            frame_queue=lambda size: types.SimpleNamespace(try_wait_for_frame=wait_frame))
        ros = types.SimpleNamespace(init=mock.Mock(), create_node=mock.Mock(return_value=mock.Mock()),
                                    shutdown=mock.Mock())
        logsafe = types.SimpleNamespace(install=mock.Mock())
        modules = {'common': types.SimpleNamespace(logsafe=logsafe), 'cv2': types.SimpleNamespace(),
                   'pyrealsense2': sdk, 'rclpy': ros,
                   'rclpy.qos': types.SimpleNamespace(qos_profile_sensor_data=None),
                   'sensor_msgs.msg': types.SimpleNamespace(CompressedImage=object)}
        with mock.patch.dict(sys.modules, modules), \
             mock.patch.object(rs.time, 'monotonic', side_effect=lambda: clock[0]):
            # Empty routes still require both SDK streams to remain healthy.
            rs._capture('robot_a', usb_path, {}, queue.Queue(), quit_event, statuses)
        logsafe.install.assert_called_once_with(check_fd=False)
        sensor.stop.assert_called_once()
        sensor.close.assert_called_once()
        errors = [s['error'] for s in list(statuses.queue) if s.get('error')]
        return errors, ros.create_node.call_args.args[0]

    def test_worker_can_bind_the_rsusb_backend_identifier(self):
        fields = {'busnum': '1', 'devpath': '1', 'devnum': '4'}
        with mock.patch.object(rs.Path, 'read_text', autospec=True,
                               side_effect=lambda p: fields[p.name]):
            errors, _ = self.capture([(.1, 'depth'), (.2, 'infrared')], physical_port='1-1-4')
        self.assertEqual(errors, [])

    def test_slow_usb_first_frames_get_the_startup_window(self):
        errors, _ = self.capture([(4, None), (4.5, 'depth'), (5, 'infrared')])
        self.assertEqual(errors, [])

    def test_startup_grace_applies_until_both_streams_have_arrived(self):
        errors, _ = self.capture([(.1, 'depth'), (4, None), (4.5, 'depth'), (5, 'infrared')])
        self.assertEqual(errors, [])

    def test_missing_first_stream_eventually_times_out(self):
        for samples in ([(10.1, None)], [(.1, 'depth'), (10.1, 'depth')]):
            with self.subTest(samples=samples):
                errors, _ = self.capture(samples)
                self.assertEqual(errors, ['RealSense depth/infrared startup timed out'])

    def test_stream_stall_after_startup_uses_shorter_deadline(self):
        errors, _ = self.capture([(.1, 'depth'), (.2, 'infrared'), (3.3, 'depth')])
        self.assertEqual(errors, ['RealSense depth/infrared frames stopped arriving'])

    def test_two_physical_cameras_have_distinct_stable_ros_node_names(self):
        _, first = self.capture([], '/sys/devices/usb1/1-1')
        _, second = self.capture([], '/sys/devices/usb1/1-2')
        _, again = self.capture([], '/sys/devices/usb1/1-1')
        self.assertNotEqual(first, second)
        self.assertEqual(first, again)
        self.assertRegex(first, r'^robot_a_realsense_stereo_[a-z0-9]+$')


if __name__ == '__main__':
    unittest.main()
