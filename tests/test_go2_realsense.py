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


if __name__ == '__main__':
    unittest.main()
