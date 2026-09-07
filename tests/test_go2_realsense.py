"""Wire format and shared stereo ownership; USB acquisition is tested on Go2."""
import importlib.util
from pathlib import Path
import queue
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
    def close(self):
        pass


class FakeProcess:
    def __init__(self, *, target, args, **kwargs):
        self.quit = args[2]
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
        self.session = rs.RealSenseSession('robot_a')
        self.session._ctx = types.SimpleNamespace(
            Event=threading.Event, Queue=StatusQueue, Process=FakeProcess)
        self.depth = rs.ExtDepthPlugin({}, 'robot_a', None, self.session)
        self.ir = rs.ExtInfraredPlugin({}, 'robot_a', None, self.session)
        self.addCleanup(self.session.stop, 'ext_depth')
        self.addCleanup(self.session.stop, 'ext_infrared')

    def report(self, streams, error=None):
        self.session._queue.put({'frames': {s: 1 for s in streams},
                                 'last_frame': {s: self.now.return_value for s in streams},
                                 'error': error, 'depth_scale_m': .001})

    def test_schema_and_prestart_info_agree_without_opening_usb(self):
        for plugin, fmt in ((self.depth, 'image/depth-zlib'), (self.ir, 'image/jpeg')):
            plugin.start()
            tool = plugin.get_tool()
            self.assertEqual(tool['type'], 'sensor')
            self.assertFalse(tool['multiInstance'])
            info = plugin.dispatch('info', {})
            self.assertEqual(info['state'], 'idle')
            self.assertEqual(info['topic_out'], tool['topic_out'])
            self.assertEqual(info['topic_out'][0]['format'], fmt)
        self.assertIsNone(self.session._proc)

    def test_running_requires_a_published_frame(self):
        self.assertEqual(self.depth.dispatch('start', {})['state'], 'starting')
        self.report(['ext_depth'])
        self.assertTrue(self.depth.dispatch('info', {})['fresh'])
        self.now.return_value += 3.1
        info = self.depth.dispatch('info', {})
        self.assertEqual(info['state'], 'error')
        self.assertFalse(info['fresh'])

    def test_sibling_stop_does_not_close_sensor_and_last_stop_releases_it(self):
        self.depth.dispatch('start', {})
        process = self.session._proc
        self.ir.dispatch('start', {})
        self.report(rs.STREAMS)
        self.assertIs(self.session._proc, process)
        self.assertEqual(self.depth.dispatch('start', {})['state'], 'running')
        self.assertEqual(self.depth.dispatch('stop', {})['state'], 'idle')
        self.assertTrue(process.alive)
        self.assertFalse(self.session._enabled['ext_depth'].is_set())
        self.assertEqual(self.ir.dispatch('info', {})['state'], 'running')
        self.ir.dispatch('stop', {})
        self.assertTrue(process.closed)
        self.assertIsNone(self.session._proc)

    def test_reenabled_stream_cannot_reuse_an_old_frame(self):
        self.depth.dispatch('start', {})
        self.ir.dispatch('start', {})
        self.report(rs.STREAMS)
        self.depth.dispatch('stop', {})
        self.now.return_value += 1
        self.assertEqual(self.depth.dispatch('start', {})['state'], 'starting')
        self.report(rs.STREAMS)
        self.assertEqual(self.depth.dispatch('info', {})['state'], 'running')

    def test_failed_worker_is_reported_and_start_retries(self):
        self.depth.dispatch('start', {})
        self.report([], error='Device unavailable')
        self.assertEqual(self.depth.dispatch('info', {})['error'], 'Device unavailable')
        old = self.session._proc
        self.assertEqual(self.depth.dispatch('start', {})['state'], 'starting')
        self.assertTrue(old.closed)
        self.assertIsNot(self.session._proc, old)
        self.session._proc.alive = False
        self.assertEqual(self.depth.dispatch('info', {})['state'], 'error')

    def test_startup_timeout_is_not_running(self):
        self.depth.dispatch('start', {})
        self.now.return_value += 10.1
        self.assertEqual(self.depth.dispatch('info', {})['state'], 'error')


if __name__ == '__main__':
    unittest.main()
