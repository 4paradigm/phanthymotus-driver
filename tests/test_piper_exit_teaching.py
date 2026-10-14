"""Seamless teaching exit acceptance against a recording SDK; no CAN access."""
from dataclasses import replace
import tempfile
import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from agilex.piper.device import build_plugins
from agilex.piper.intent_teaching import exit_teaching_and_hold, supports_seamless_exit
from common.vendor_runtime import DriverBundle
from test_piper_intent import IntentDriver, FastClock


class TeachingExitTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        motion_env = patch.dict('os.environ', {'PIPER_MOTION_ENABLED': '1'})
        motion_env.start()
        self.addCleanup(motion_env.stop)
        self.plugins = build_plugins({'data_dir': self.tmp.name, 'async_actions': False}, 'agilex', None)
        self.runtime, self.background = self.plugins[:2]
        self.assertTrue(self.runtime.motion_enabled)
        self.runtime.factory = IntentDriver
        self.background.auto_connect = False
        self.addCleanup(self.runtime.stop)
        self.addCleanup(self.background.stop)
        self.bundle = DriverBundle(self.plugins)
        self.driver = self.runtime.connected()
        self.sdk = self.driver.arm
        self.sdk.mode = 'TEACHING_MODE(0x2)'
        self.sdk.teach = 'STOP_RECORDING(0x2)'
        self.clock = FastClock()
        self.sleep_hook = lambda seconds: None
        def sleep(seconds):
            self.sleep_hook(seconds)
            self.clock.sleep(seconds)
        self.patch = patch('agilex.piper.intent_control.exit_teaching_and_hold',
                           side_effect=lambda *a, **kw: exit_teaching_and_hold(
                               *a, **kw, clock=self.clock.now, sleep=sleep))
        self.patch.start()
        self.addCleanup(self.patch.stop)

    def call(self, action='exit_teaching_and_hold', **args):
        return self.bundle.dispatch('arm_control', {'action': action, **args})

    def controls(self):
        return [c for c in self.sdk.calls if c[0] != 'firmware_query']

    def test_schema_has_explicit_parameterless_async_action(self):
        schema = self.runtime.get_tools()[0]['inputSchema']
        self.assertEqual(schema['x-action-params']['exit_teaching_and_hold']['params'], [])
        self.assertIn('exit_teaching_and_hold', schema['x-completion']['actions'])
        self.assertIn({'const': 'exit_teaching_and_hold', 'title': 'Exit teaching and hold'},
                      schema['properties']['action']['oneOf'])
        with self.assertRaises(ValueError):
            self.call(width_mm=20)
        self.assertEqual(self.sdk.calls, [])

    def test_current_pose_once_no_home_reset_enable_or_gripper_commands(self):
        target = self.sdk.raw
        gripper_before = self.sdk.gripper_enabled
        result = self.call()
        self.assertEqual(result['state'], 'completed')
        self.assertEqual(result['firmware'], 'S-V1.9-0')
        self.assertEqual(result['target_joints_deg'], tuple(q/1000 for q in target))
        self.assertGreaterEqual(self.clock.now(), .8)
        self.assertEqual(self.sdk.calls[0], ('firmware_query',))
        self.assertEqual(self.controls()[0], ('mode', 1, 1, 1, 0))
        self.assertTrue(all(c == ('joints', target) for c in self.controls() if c[0] == 'joints'))
        self.assertEqual({c[0] for c in self.controls()}, {'mode', 'joints'})
        self.assertEqual(sum(c[0] == 'mode' for c in self.controls()), 1)
        self.assertTrue(all(self.sdk.enabled))
        self.assertEqual(self.sdk.gripper_enabled, gripper_before)
        self.assertFalse(result['motor_torque_released'])
        self.assertEqual(self.call('info')['task']['result'], {k:v for k,v in result.items() if k != 'state'})
        calls = list(self.sdk.calls)
        self.assertFalse(self.call()['executed'])
        self.assertEqual(self.sdk.calls, calls)

    def test_firmware_gate_no_controls_for_old_unknown_or_missing_version(self):
        for version in ('S-V1.8-4', 'S-V1.7-9', 'S-V2.0-0', None, '', 'bogus', 'S-V1.9-0junk'):
            with self.subTest(version=version), patch.object(self.driver, 'firmware', return_value=version), \
                 self.assertRaisesRegex(RuntimeError, 'firmware'):
                self.call()
        self.assertEqual(self.controls(), [])
        for version in ('S-V1.8-5', 'S-V1.8-9', 'S-V1.9-0'):
            self.assertTrue(supports_seamless_exit(version))

    def test_invalid_state_never_sends_controls(self):
        cases = [('frozen', True), ('raw', (float('nan'), 10000, -10000, 0, 20000, 0)),
                 ('raw', (1000, -2000, -10000, 0, 20000, 0)), ('error', 1),
                 ('mode', 'STANDBY(0x0)'), ('mode', 'UNKNOWN(0xff)'),
                 ('enabled', (False,)*6), ('enabled', (True, False, True, True, True, True)),
                 ('teach', 'START_RECORDING(0x1)'), ('teach', 'EXECUTE_TRAJECTORY(0x3)'),
                 ('status', 'SINGULARITY_POINT(0x3)')]
        for field, value in cases:
            old = getattr(self.sdk, field)
            try:
                setattr(self.sdk, field, value)
                with self.subTest(field=field, value=value), self.assertRaises(RuntimeError):
                    self.call()
            finally:
                setattr(self.sdk, field, old)
        self.assertEqual(self.sdk.calls, [])

    def test_motion_or_external_mode_change_during_preflight_rejected(self):
        for change in ('pose', 'mode'):
            self.sdk.mode = 'TEACHING_MODE(0x2)'
            self.sdk.calls.clear()
            def hook(seconds):
                if change == 'pose':
                    self.sdk.raw = (self.sdk.raw[0]+300, *self.sdk.raw[1:])
                else:
                    self.sdk.mode = 'CAN_CTRL(0x1)'
            self.sleep_hook = hook
            with self.subTest(change=change), self.assertRaises(RuntimeError):
                self.call()
            self.assertEqual(self.controls(), [])

    def test_no_confirmation_timeout_does_not_repeat_teaching_takeover(self):
        def ignore_mode(*args): self.sdk.calls.append(('mode', *args))
        self.sdk.MotionCtrl_2 = ignore_mode
        with self.assertRaisesRegex(RuntimeError, 'timed out'):
            self.call()
        self.assertEqual([c[0] for c in self.controls()], ['mode', 'joints'])
        self.assertEqual(self.call('info')['state'], 'failed')

    def test_cached_feedback_and_unreached_status_cannot_report_success(self):
        get_state = self.driver.get_state
        get_status = self.sdk.GetArmStatus
        for case in ('joints', 'status', 'unreached'):
            self.sdk.mode = 'TEACHING_MODE(0x2)'
            self.sdk.calls.clear()
            self.driver.get_state = (lambda: replace(get_state(), feedback_time=time.time()-.2)) if case == 'joints' else get_state
            self.sdk.GetArmStatus = (lambda: SimpleNamespace(time_stamp=time.time()-.2,
                arm_status=SimpleNamespace(motion_status=0))) if case == 'status' else get_status
            self.sdk.reached = case != 'unreached'
            with self.subTest(case=case), self.assertRaisesRegex(RuntimeError, 'timed out'):
                self.call()
            self.assertEqual(self.call('info')['state'], 'failed')

    def test_hold_requires_position_stability(self):
        original = self.sdk.JointCtrl
        def displaced(*target):
            original(*target)
            self.sdk.raw = (target[0]+750, *target[1:])
        self.sdk.JointCtrl = displaced
        with self.assertRaisesRegex(RuntimeError, 'timed out'):
            self.call()
        self.assertNotEqual(self.call('info')['state'], 'completed')

    def test_transition_fault_stale_enable_or_ownership_loss_stops_old_target(self):
        for field, value in [('mode', 'TEACHING_MODE(0x2)'), ('enabled', (False,)*6),
                             ('error', 1), ('frozen', True), ('status_frozen', True),
                             ('teach', 'EXECUTE_TRAJECTORY(0x3)')]:
            saved = vars(self.sdk).copy()
            self.sdk.calls = []
            self.sdk.mode = 'TEACHING_MODE(0x2)'
            before_change = []
            def hook(seconds):
                if self.sdk.mode.startswith('CAN_CTRL'):
                    before_change[:] = self.controls()
                    setattr(self.sdk, field, value)
            self.sleep_hook = hook
            try:
                with self.subTest(field=field), self.assertRaises(RuntimeError): self.call()
                self.assertEqual(self.controls(), before_change)
            finally:
                self.sdk.__dict__.update(saved)

    def test_unexpected_drift_holds_latest_pose_instead_of_old_target(self):
        target = self.sdk.raw
        drifted = (target[0]+1500, *target[1:])
        def hook(seconds):
            if self.sdk.mode.startswith('CAN_CTRL'): self.sdk.raw = drifted
        self.sleep_hook = hook
        with self.assertRaisesRegex(RuntimeError, 'more than 1 degree'):
            self.call()
        self.assertEqual(self.controls()[-1], ('joints', drifted))
        self.assertTrue(all(self.sdk.enabled))

    def test_async_stop_during_firmware_query_never_switches(self):
        self.runtime.config['async_actions'] = True
        entered, release, complete = threading.Event(), threading.Event(), threading.Event()
        def firmware(query=False):
            entered.set(); release.wait(2); return 'S-V1.9-0'
        with patch.object(self.driver, 'firmware', side_effect=firmware), \
             patch.object(self.runtime, 'acp_complete', side_effect=lambda *args: complete.set()):
            self.call()
            try:
                self.assertTrue(entered.wait(1))
                self.assertEqual(self.call('info')['state'], 'preparing')
                with self.assertRaises(RuntimeError): self.call()
                self.assertEqual(self.call('stop_motion')['state'], 'stopping')
            finally:
                release.set()
            self.assertTrue(complete.wait(2))
        self.assertEqual(self.call('info')['state'], 'cancelled')
        self.assertEqual(self.controls(), [])

    def test_async_stop_after_switch_only_holds_current_pose(self):
        self.runtime.config['async_actions'] = True
        entered, release, complete = threading.Event(), threading.Event(), threading.Event()
        def hook(seconds):
            if self.sdk.mode.startswith('CAN_CTRL'):
                entered.set(); release.wait(2)
        self.sleep_hook = hook
        with patch.object(self.runtime, 'acp_complete', side_effect=lambda *args: complete.set()):
            self.call()
            try:
                self.assertTrue(entered.wait(1))
                self.assertEqual(self.call('info')['state'], 'moving')
                self.assertEqual(self.call('stop_motion')['state'], 'stopping')
                self.assertEqual(self.call('stop_motion')['state'], 'stopping')
                self.sdk.raw = (self.sdk.raw[0]+100, *self.sdk.raw[1:])
                stopped_at = self.sdk.raw
                count = len(self.controls())
            finally:
                release.set()
            self.assertTrue(complete.wait(2))
        self.assertEqual(self.call('info')['state'], 'cancelled')
        self.assertEqual(self.controls()[count:], [('mode',1,1,1,0), ('joints',stopped_at)])
        self.assertTrue(all(self.sdk.enabled))
        self.assertEqual(self.call('stop_motion')['state'], 'stopped')


if __name__ == '__main__':
    unittest.main()
