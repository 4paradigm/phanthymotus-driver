"""Test the production two-card registration with a simulated SDK only."""
import tempfile
import threading
import unittest
from pathlib import Path
from unittest.mock import patch

from agilex.piper.device import build_plugins
from common.vendor_runtime import DriverBundle
from test_piper_controls import Driver


class PiperEntrypointTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.plugins = build_plugins({'name': '机械臂 1', 'data_dir': self.tmp.name,
                                      'async_actions': False}, 'agilex', None)
        self.runtime, self.status = self.plugins
        self.runtime.factory = Driver
        self.status.auto_connect = False
        self.bundle = DriverBundle(self.plugins)
        self.addCleanup(self.runtime.stop)
        self.addCleanup(self.status.stop)

    def call(self, action, **args):
        return self.bundle.dispatch('arm_control', {'action': action, **args})

    def test_production_registration_metadata_and_passive_lifecycle(self):
        tools = {t['name']: t for t in self.bundle.get_all_tools()}
        self.assertEqual(set(tools), {'arm_control', 'piper_status', 'model', 'piper_photo'})
        schema = tools['arm_control']['inputSchema']
        self.assertNotIn('x-confirmation', schema)
        self.assertFalse({'read', 'allow_motion', 'block_motion', 'enter_teaching', 'exit_teaching'}
                         & set(schema['properties']['action']['enum']))
        self.assertEqual(tools['piper_status']['type'], 'resource')
        self.assertEqual(tools['piper_status']['topic_out'][0]['topic'], '/agilex/piper/state/joints')
        self.assertIn('<robot', self.bundle.dispatch('model', {})['urdf'])
        self.call('start')
        self.bundle.dispatch('piper_status', {'action': 'start'})
        self.bundle.dispatch('piper_status', {'action': 'stop'})
        self.assertIsNone(self.runtime.driver)

    def test_direct_jog_and_explicit_one_use_preview(self):
        sdk = self.runtime.connected().arm
        preview = self.call('jog_positive', execute=False, joint=1, step_deg='0.5')
        self.assertEqual(sdk.calls, [])
        self.assertTrue(self.call('jog_positive', execute=True, preview_id=preview['preview_id'],
                                  joint=1, step_deg='0.5', workspace_clear=True, take_control=True)['executed'])
        with self.assertRaises(RuntimeError):
            self.call('jog_positive', execute=True, preview_id=preview['preview_id'],
                      joint=1, step_deg='0.5', workspace_clear=True, take_control=True)
        self.assertTrue(self.call('jog_negative', joint=1, step_deg=.5)['executed'])
        self.assertFalse(self.runtime.motion_enabled)
        with self.assertRaises(RuntimeError):
            self.runtime.require_motion_permission()

    def test_invalid_real_runtime_feedback_mode_limits_and_gripper_do_not_send(self):
        sdk = self.runtime.connected().arm
        for attribute, bad_value in [('mode', 'TEACHING_MODE(0x2)'), ('frozen', True),
                                     ('error', 1), ('enabled', (False,) * 6)]:
            previous = getattr(sdk, attribute)
            setattr(sdk, attribute, bad_value)
            with self.subTest(attribute=attribute), self.assertRaises(RuntimeError):
                self.call('jog_positive', joint=1, step_deg=.5)
            setattr(sdk, attribute, previous)
        with self.assertRaises(ValueError):
            self.call('move_joint', joint=2, target_deg=-1)
        with self.assertRaises(RuntimeError):
            self.call('gripper_open')
        with self.assertRaises(ValueError):
            self.call('disable', supported=False)
        self.assertEqual(sdk.calls, [])

    def test_real_async_completion_is_scoped_to_arm_control(self):
        self.runtime.config['async_actions'] = True
        sdk = self.runtime.connected().arm
        completed = threading.Event()
        reports = []
        def notify(*args):
            reports.append(args)
            completed.set()
        with patch.object(self.runtime, 'acp_complete', side_effect=notify):
            result = self.call('jog_positive', joint=1, step_deg=.5)
            self.assertEqual(result['state'], 'running')
            self.assertTrue(completed.wait(3))
        self.assertEqual(reports[0][1], 'completed')
        self.assertEqual(reports[0][3], 'arm_control')
        self.assertTrue(self.runtime.last_actions['arm_control']['result']['executed'])
        self.assertFalse(self.runtime.gate.locked())
        self.assertFalse(self.runtime.motion_enabled)
        with self.assertRaises(RuntimeError):
            self.runtime.require_motion_permission()
        self.assertTrue(any(c[0] == 'joints' for c in sdk.calls))

    def test_monitor_snapshot_does_not_invalidate_or_cancel_while_busy(self):
        sdk = self.runtime.connected().arm
        preview = self.call('jog_positive', execute=False)
        previous_signature = self.runtime.last_signature
        with self.runtime.operation('arm_control', 'jog_positive'):
            sdk.raw = (2000, *sdk.raw[1:])
            with patch.object(self.runtime, 'status', side_effect=AssertionError('status side effect')), \
                 patch.object(self.runtime, 'read_state', side_effect=AssertionError('read_state side effect')):
                snapshot = self.runtime.read_monitor_snapshot()
            self.assertTrue(snapshot['busy'])
            self.assertTrue(snapshot['fresh'])
            self.assertIn(preview['preview_id'], self.runtime.previews.entries)
            self.assertEqual(previous_signature, self.runtime.last_signature)
            self.assertFalse(self.runtime.cancel.is_set())
            self.assertFalse(self.runtime.driver.cancelled.is_set())
            self.assertEqual(sdk.calls, [])

    def test_monitor_never_waits_for_connection_lock(self):
        locked, release = threading.Event(), threading.Event()
        def hold_lock():
            with self.runtime.connection_lock:
                locked.set()
                release.wait(2)
        worker = threading.Thread(target=hold_lock)
        worker.start()
        try:
            self.assertTrue(locked.wait(1))
            snapshot = self.runtime.read_monitor_snapshot()
            self.assertFalse(snapshot['fresh'])
            self.assertIn('connection', snapshot['reasons'])
        finally:
            release.set()
            worker.join(1)
        self.assertFalse(worker.is_alive())
        self.assertIsNone(self.runtime.driver)

    def test_two_device_entrypoints_have_separate_runtime_and_topics(self):
        other = build_plugins({'name': '机械臂 2', 'data_dir': str(Path(self.tmp.name) / 'arm2')}, 'piper2', None)
        self.addCleanup(other[0].stop)
        self.addCleanup(other[1].stop)
        self.assertIsNot(self.runtime, other[0])
        self.assertEqual(other[1].get_tools()[0]['topic_out'][0]['topic'], '/piper2/piper/state/joints')
        self.assertIsNone(other[0].driver)


if __name__ == '__main__':
    unittest.main()
