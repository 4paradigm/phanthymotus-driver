"""Live-color tests use fake captures/ROS or an isolated non-camera process."""
import multiprocessing as mp
from pathlib import Path
import tempfile
import struct
import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import cv2
import numpy as np

from agilex.piper.camera_live import PiperCameraLive, add_camera_live, camera_settings, offline_image
from agilex.piper.camera_owner import CameraOwner
from agilex.piper.camera_worker import CameraSession, Mailbox, encode_frame
from common.vendor_runtime import DriverBundle


def blocked_reader(settings, mailbox, stop):
    # Simulate a driver blocked in read() while holding its mailbox lock.
    mailbox.values[0] = -1
    mailbox.lock.acquire()
    time.sleep(60)


class Transport:
    def __init__(self, *args):
        self.messages = []
        self.closed = 0
    def publish(self, jpeg, stamp):
        self.messages.append((jpeg, stamp))
    def close(self):
        self.closed += 1


class Session:
    def __init__(self, settings, *, frozen=False, error=None):
        self.sequence = 0
        self.closed = 0
        self.disposed = 0
        self.frozen = frozen
        self.error = error
        self.last = None
        self.next_frame = 0
    def read(self):
        if self.error:
            return None, self.error
        now = time.monotonic()
        if now >= self.next_frame and (not self.frozen or self.last is None):
            self.sequence += 1
            self.last = dict(sequence=self.sequence, timestamp=time.time(), width=640, height=480,
                             capture_fps=30., jpeg=b'live-' + str(self.sequence).encode())
            self.next_frame = now + .05
        return self.last, None
    def close(self):
        self.closed += 1
        return True
    def dispose(self):
        self.disposed += 1


class LiveCameraTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.runtime = SimpleNamespace(config={'name': '机械臂 1', 'camera': {
            'device': '/dev/piper-camera-1', 'warmup_seconds': 0, 'live': {'enabled': True}}},
            topic='/agilex/piper/rgb', data_dir=Path(self.tmp.name), camera_enabled=True)
        self.transports = []
        self.sessions = []
        self.path_patch = patch('agilex.piper.camera_live.Path.exists', return_value=True)
        self.path_patch.start()
        self.addCleanup(self.path_patch.stop)

    def make(self, factory=None):
        def publisher(*args):
            transport = Transport();self.transports.append(transport);return transport
        def session(settings):
            value = (factory or Session)(settings);self.sessions.append(value);return value
        plugin = PiperCameraLive(self.runtime, object(), publisher_factory=publisher, session_factory=session)
        plugin.RETRY_SECONDS = .03
        self.addCleanup(plugin.stop)
        return plugin

    def wait(self, condition, timeout=2):
        deadline = time.monotonic() + timeout
        while not condition():
            if time.monotonic() > deadline:
                self.fail('timed out waiting for camera test condition')
            time.sleep(.01)

    def test_resource_card_and_auto_stream_has_new_frames_and_independent_topic(self):
        plugin = self.make()
        definition = plugin.get_tools()[0]
        self.assertEqual(definition['type'], 'resource')
        self.assertEqual(set(definition['inputSchema']['properties']['action']['enum']), {'start', 'stop', 'info'})
        self.assertEqual(definition['topic_out'], [{'topic': '/agilex/piper/camera/live', 'format': 'image/jpeg'}])
        self.assertEqual(plugin.start()['state'], 'starting')
        self.wait(lambda: plugin.info()['有效帧数'] >= 3)
        plugin.start()
        self.assertEqual(len(self.sessions), 1)
        result = DriverBundle([plugin]).dispatch('piper_camera_live', {'action': 'info'})
        self.assertEqual(result['状态'], '实时画面正常')
        self.assertEqual(result['实际分辨率'], [640, 480])
        self.assertGreater(self.transports[0].messages[-1][1], self.transports[0].messages[0][1])
        self.assertEqual(plugin.stop()['state'], 'idle')
        previous = len(self.transports[0].messages)
        time.sleep(.04)
        self.assertEqual(len(self.transports[0].messages), previous)
        self.assertEqual(self.sessions[0].closed, 1)
        self.assertEqual(self.sessions[0].disposed, 1)
        self.assertEqual(self.transports[0].closed, 1)

    def test_sensor_card_like_joints_has_no_action_menu_and_accepts_core_lifecycle(self):
        self.runtime.interface_language = 'en'
        plugin = self.make()
        plugin.sensor_card = True
        definition = plugin.get_tools()[0]
        self.assertEqual((definition['title'], definition['type']), ('On camera', 'sensor'))
        self.assertEqual(definition['inputSchema']['properties'], {})
        bundle = DriverBundle([plugin])
        bundle.dispatch('piper_camera_live', {'action': 'start'})
        self.wait(lambda: plugin.info()['frames'] >= 2)
        info = bundle.dispatch('piper_camera_live', {'action': 'info', 'input_topic': ''})
        self.assertEqual(info['state'], 'running')
        self.assertGreaterEqual(info['frames'], 2)
        self.assertEqual(bundle.dispatch('piper_camera_live', {'action': 'stop'})['state'], 'idle')
        self.assertEqual(len(self.sessions), 1)
        for args in ({'action': 'capture'}, {'action': 'info', 'input_topic': '/another/camera'},
                     {'action': 'info', 'width_mm': 20}):
            with self.assertRaises(ValueError): bundle.dispatch('piper_camera_live', args)

    def test_stale_frame_is_not_replayed_offline_is_jpeg_and_recovery_is_new_session(self):
        plugin = self.make(lambda cfg: Session(cfg, frozen=not self.sessions))
        plugin.STALE_SECONDS = .12
        plugin.start()
        self.wait(lambda: len(self.sessions) >= 2 and plugin.info()['有效帧数'] >= 3)
        plugin.stop()
        messages = self.transports[0].messages
        self.assertEqual(sum(jpeg == b'live-1' for jpeg, _ in messages), 2)  # One frame per session, never stale replay.
        offline = next(jpeg for jpeg, _ in messages if jpeg.startswith(b'\xff\xd8'))
        self.assertIsNotNone(cv2.imdecode(np.frombuffer(offline, np.uint8), cv2.IMREAD_COLOR))
        self.assertEqual(self.sessions[0].closed, 1)
        self.assertIsNone(plugin.info()['错误原因'])

    def test_missing_camera_never_opens_session_and_stop_cancels_retries(self):
        plugin = self.make()
        with patch('agilex.piper.camera_live.Path.exists', return_value=False):
            plugin.start()
            self.wait(lambda: plugin.info()['state'] == 'offline')
            self.assertIn('不存在', plugin.info()['错误原因'])
            plugin.stop()
        self.assertEqual(self.sessions, [])
        self.assertFalse(plugin._thread.is_alive())
        with CameraOwner(self.tmp.name, '/dev/piper-camera-1'):
            pass

    def test_camera_busy_is_visible_and_releases_lease_after_stop(self):
        plugin = self.make()
        with CameraOwner(self.tmp.name, '/dev/piper-camera-1'):
            plugin.start()
            self.wait(lambda: plugin.info()['state'] == 'offline')
            self.assertIn('占用', plugin.info()['错误原因'])
            self.assertEqual(self.sessions, [])
        self.wait(lambda: plugin.info()['state'] == 'running')
        with self.assertRaisesRegex(RuntimeError, '占用'):
            CameraOwner(self.tmp.name, '/dev/piper-camera-1')
        plugin.stop()
        with CameraOwner(self.tmp.name, '/dev/piper-camera-1'):
            pass

    def test_repeated_lifecycle_and_disabled_or_rosless_does_not_capture(self):
        plugin = self.make()
        self.runtime.camera_enabled = False
        self.assertEqual(plugin.start()['state'], 'disabled')
        self.assertEqual(self.sessions, [])
        self.runtime.camera_enabled = True
        for _ in range(2):
            plugin.start();self.wait(lambda: plugin.info()['state'] == 'running')
            plugin.stop();plugin.stop()
        self.assertEqual(len(self.sessions), 2)
        rosless = PiperCameraLive(self.runtime)
        rosless.start();self.wait(lambda: rosless.info()['state'] == 'offline');rosless.stop()
        self.assertIn('ROS', rosless.info()['错误原因'])

    def test_stream_does_not_use_arm_gate_or_any_control_method(self):
        self.runtime.gate = threading.Lock();self.runtime.gate.acquire()
        self.addCleanup(self.runtime.gate.release)
        self.runtime.connected = lambda: self.fail('CAN connection accessed')
        self.runtime.require_motion_permission = lambda: self.fail('motion permission accessed')
        plugin = self.make();plugin.start()
        self.wait(lambda: plugin.info()['有效帧数'] >= 2);plugin.stop()
        self.assertTrue(self.runtime.gate.locked())
        for action in ('capture', 'move_joint', 'enable'):
            with self.assertRaises(ValueError):plugin.dispatch(action, {'action': action})

    def test_shared_photo_capture_lock_preserves_existing_capture_authorization(self):
        calls = []
        self.runtime.capture = lambda args: calls.append(args) or {'photo': True}
        self.runtime.get_tools = lambda: []
        plugins = add_camera_live([self.runtime], self.runtime.config, None)
        with CameraOwner(self.tmp.name, '/dev/piper-camera-1'):
            with self.assertRaisesRegex(RuntimeError, '占用'):
                self.runtime.capture({'capture_confirmed': True})
        self.assertEqual(calls, [])
        self.assertTrue(self.runtime.capture({'capture_confirmed': True})['photo'])
        self.assertEqual(calls, [{'capture_confirmed': True}])
        self.assertEqual(len(plugins), 2)

    def test_two_device_topic_and_camera_ownership_are_separate(self):
        other = SimpleNamespace(**vars(self.runtime))
        other.config = {'camera': {'device': '/dev/piper-camera-2'}}
        other.topic = '/piper2/piper/rgb'
        plugin = PiperCameraLive(other)
        self.assertEqual(plugin.topic, '/piper2/piper/camera/live')
        with CameraOwner(self.tmp.name, '/dev/piper-camera-1'), CameraOwner(self.tmp.name, '/dev/piper-camera-2'):
            pass

    def test_mailbox_only_keeps_latest_frame_and_nonblocking_read_survives_writer_lock(self):
        mailbox = Mailbox.create(mp.get_context('spawn'))
        for i in range(20):mailbox.write(str(i).encode(), time.time(), 640, 480, 30)
        frame, error = mailbox.read()
        self.assertEqual(frame['sequence'], 20)
        self.assertEqual(frame['jpeg'], b'19')
        self.assertIsNone(error)
        with mailbox.lock:
            self.assertEqual(mailbox.read(), (None, None))

    def test_stalled_process_is_terminated_and_resource_is_released(self):
        session = CameraSession(camera_settings(self.runtime.config), worker_target=blocked_reader)
        try:
            self.wait(lambda: session.mailbox.values[0] == -1, timeout=5)
            self.assertEqual(session.mailbox.read(), (None, None))
            began = time.monotonic()
            self.assertTrue(session.close())
            self.assertLess(time.monotonic() - began, 3)
            self.assertFalse(session.process.is_alive())
        finally:
            session.dispose()

    def test_stop_during_session_start_prevents_late_frames_and_second_generation(self):
        entered, release = threading.Event(), threading.Event()
        def blocked(settings):
            entered.set();release.wait(3)
            return Session(settings)
        plugin = self.make(blocked)
        plugin.start()
        stopper = None
        try:
            self.assertTrue(entered.wait(1))
            stopper = threading.Thread(target=plugin.stop);stopper.start()
            self.assertTrue(plugin._stop.wait(1))
            with self.assertRaises(RuntimeError):plugin.start()
        finally:
            release.set()
            if stopper is not None:stopper.join(2)
        self.assertFalse(stopper.is_alive())
        self.assertEqual(plugin.info()['state'], 'idle')
        self.assertEqual(self.transports[0].messages, [])
        self.assertEqual(self.sessions[0].closed, 1)
        with CameraOwner(self.tmp.name, '/dev/piper-camera-1'):
            pass

    def test_color_frame_encoding_is_jpeg_with_stamp_and_invalid_settings_are_rejected(self):
        frame = np.zeros((480, 640, 3), np.uint8)
        stamp = time.time()
        jpeg = encode_frame(frame, stamp, 80)
        self.assertEqual(jpeg[2:4], b'\xff\xef')
        offset = 6 + len(b'PIPER_LIVE_V1 ')
        self.assertEqual(struct.unpack('>d', jpeg[offset:offset+8])[0], stamp)
        decoded = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
        self.assertEqual(decoded.shape, frame.shape)
        self.assertGreater(decoded[:29].sum(), 0)
        self.assertGreater(len(offline_image(640, 480)), 0)
        for live in ({'fps': True}, {'fps': 31}, {'width': 641}, {'jpeg_quality': 96}, {'enabled': 'true'}):
            with self.subTest(live=live), self.assertRaises(ValueError):
                camera_settings({'camera': {'live': live}})
        with self.assertRaises(ValueError):camera_settings({'camera': {'device': '/dev/video0'}})


if __name__ == '__main__':unittest.main()
