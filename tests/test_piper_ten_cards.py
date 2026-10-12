"""Public ten-card workflows, command isolation, cancellation and completion."""
from dataclasses import replace
import json
import tempfile
import threading
import time
import unittest
from unittest.mock import patch
from http.server import ThreadingHTTPServer
import urllib.request

from agilex.piper.device import PiperPlugin
from agilex.piper.cards import TITLES
from common.vendor_runtime import DriverBundle, make_handler
from test_piper_controls import Driver as BaseDriver, SDK as BaseSDK


class SDK(BaseSDK):
    def __init__(self):
        super().__init__()
        self.target = self.raw

    def JointCtrl(self, *target):
        self.calls.append(('joints', target))
        self.target = tuple(target)
        if self.follow and all(self.enabled):
            self.raw = self.target

    def EnablePiper(self):
        self.calls.append(('enable',))
        self.enabled = (True,) * 6
        if self.follow:
            self.raw = self.target

    def DisablePiper(self):
        self.calls.append(('disable',))
        self.enabled = (False,) * 6


class Driver(BaseDriver):
    def __init__(self, name='can0'):
        super().__init__(name)
        self.arm = SDK()


class TenCardTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        with patch.dict('os.environ', {'PIPER_MOTION_ENABLED': '0', 'PIPER_CAMERA_ENABLED': '0'}):
            self.plugin = PiperPlugin({'data_dir': self.tmp.name, 'async_actions': False}, driver_factory=Driver)
        self.addCleanup(self.plugin.stop)
        self.bundle = DriverBundle([self.plugin])
        self.tick = 0.

    def call(self, name, action, **args):
        return self.bundle.dispatch('piper_' + name, {'action': action, **args})

    def connected(self):
        return self.plugin.connected().arm

    def fast(self, func):
        def clock():
            self.tick += .05
            return self.tick
        with patch('agilex.piper.arm.time.sleep'), patch('agilex.piper.arm.time.monotonic', side_effect=clock):
            return func()

    def test_exactly_ten_public_cards_and_start_never_connects(self):
        self.assertEqual({t['name'] for t in self.plugin.get_tools()}, set(TITLES))
        for name in TITLES:
            self.bundle.dispatch(name, {'action': 'start'})
            self.assertIsNone(self.plugin.driver)
        self.assertIsNone(self.bundle.dispatch('piper_arm', {'action': 'prepare'}))
        self.assertIsNone(self.bundle.dispatch('piper_mode', {'action': 'switch'}))

    def test_wrong_card_and_wrong_bool_reject_before_connection(self):
        for name, action, args in [('joint', 'enable', {}), ('gripper', 'disable', {}),
                                   ('enable', 'enable', {'execute': 'true'}),
                                   ('disable', 'disable', {'supported': 'true'})]:
            with self.subTest(name=name), self.assertRaises(ValueError):
                self.call(name, action, **args)
            self.assertIsNone(self.plugin.driver)

    def test_enable_requires_can_and_already_enabled_is_no_command(self):
        sdk = self.connected()
        result = self.call('enable', 'enable', object='arm', execute=True)
        self.assertTrue(result['confirmed'])
        self.assertFalse(result['executed'])
        self.assertEqual(sdk.calls, [])
        sdk.mode, sdk.enabled = 'STANDBY(0x0)', (False,) * 6
        with self.assertRaisesRegex(RuntimeError, '切换电脑控制'):
            self.call('enable', 'enable', object='arm')
        self.assertEqual(sdk.calls, [])

    def test_mode_preview_switch_and_no_op_never_enable(self):
        sdk = self.connected()
        sdk.mode, sdk.enabled = 'STANDBY(0x0)', (False,) * 6
        self.plugin.motion_enabled = True
        plan = self.call('can', 'switch', other_controllers_stopped=True)
        self.assertTrue(plan['allowed'])
        self.assertEqual(sdk.calls, [])
        result = self.call('can', 'switch', execute=True, other_controllers_stopped=True, preview_id=plan['preview_id'])
        self.assertTrue(result['confirmed'])
        self.assertEqual(sdk.enabled, (False,) * 6)
        self.assertEqual(len(sdk.calls), 1)
        self.call('can', 'switch', execute=True, other_controllers_stopped=True)
        self.assertEqual(len(sdk.calls), 1)
        with self.assertRaises(RuntimeError):
            self.call('standby', 'switch', execute=True, preview_id=plan['preview_id'])

    def test_boundary_correction_disclosed_and_requires_specific_confirmation(self):
        sdk = self.connected()
        sdk.enabled = (False,) * 6
        sdk.raw = (1000, -2000, 1000, 0, 20000, 0)
        self.plugin.motion_enabled = True
        plan = self.call('enable', 'enable', object='arm')
        self.assertEqual(plan['boundary_adjustment_deg'], (0, 2, -1, 0, 0, 0))
        self.assertEqual(sdk.calls, [])
        with self.assertRaisesRegex(RuntimeError, '先预览'):
            self.call('enable', 'enable', object='arm', execute=True, workspace_clear=True)
        self.assertEqual(sdk.calls, [])
        plan = self.call('enable', 'enable', object='arm')
        result = self.fast(lambda: self.call('enable', 'enable', object='arm', execute=True,
            preview_id=plan['preview_id'], workspace_clear=True, allow_limit_adjustment=True))
        self.assertTrue(result['executed'])
        self.assertTrue(all(sdk.enabled))
        self.assertTrue(any(c[0] == 'enable' for c in sdk.calls))

    def test_gripper_enable_is_separate_from_open_and_preserves_width(self):
        sdk = self.connected()
        self.plugin.motion_enabled = True
        with self.assertRaisesRegex(RuntimeError, '电机使能'):
            self.call('gripper', 'open')
        self.assertEqual(sdk.calls, [])
        plan = self.call('enable', 'enable', object='gripper')
        self.assertEqual(plan['target_mm'], 20)
        self.assertEqual(sdk.calls, [])
        self.fast(lambda: self.call('enable', 'enable', object='gripper', execute=True,
            preview_id=plan['preview_id'], workspace_clear=True, take_control=True))
        self.assertTrue(sdk.gripper_enabled)
        self.assertEqual(sdk.width, 20)
        count = len(sdk.calls)
        self.call('enable', 'enable', object='gripper', execute=True)
        self.assertEqual(len(sdk.calls), count)
        plan = self.call('gripper', 'set_width', width_mm=5)
        self.fast(lambda: self.call('gripper', 'set_width', width_mm=5, execute=True,
            preview_id=plan['preview_id'], workspace_clear=True, take_control=True))
        self.assertEqual(sdk.width, 5)

    def test_disable_requires_support_and_no_op_sends_nothing(self):
        sdk = self.connected()
        self.plugin.motion_enabled = True
        with self.assertRaises(ValueError):
            self.call('disable', 'disable', object='arm')
        self.assertEqual(sdk.calls, [])
        self.fast(lambda: self.call('disable', 'disable', object='arm', supported=True))
        self.assertEqual(sdk.enabled, (False,) * 6)
        count = len(sdk.calls)
        self.call('disable', 'disable', object='arm')
        self.assertEqual(len(sdk.calls), count)

    def test_preview_token_cannot_execute_changed_joint_target(self):
        sdk = self.connected()
        self.plugin.motion_enabled = True
        plan = self.call('joint', 'move', joint=1, target_deg=2)
        self.assertEqual(sdk.calls, [])
        with self.assertRaises(RuntimeError):
            self.call('joint', 'move', joint=1, target_deg=3, execute=True,
                preview_id=plan['preview_id'], workspace_clear=True, take_control=True)
        self.assertEqual(sdk.calls, [])

    def test_async_public_identity_scope_stop_and_gate_release(self):
        self.connected()
        self.plugin.motion_enabled = True
        self.plugin.config['async_actions'] = True
        started, release, completed = threading.Event(), threading.Event(), threading.Event()
        reports = []
        def move(*args):
            started.set()
            release.wait(2)
            if self.plugin.cancel.is_set():
                raise RuntimeError('cancelled')
            return self.plugin.driver.get_state()
        def notify(*args):
            reports.append(args)
            self.assertFalse(self.plugin.gate.locked())
            completed.set()
        plan = self.call('joint', 'move', joint=1, target_deg=2)
        with patch.object(self.plugin.driver, '_move_joint_target', side_effect=move), patch.object(self.plugin, 'acp_complete', side_effect=notify):
            try:
                result = self.call('joint', 'move', joint=1, target_deg=2, execute=True,
                    preview_id=plan['preview_id'], workspace_clear=True, take_control=True)
                self.assertTrue(started.wait(1))
                self.assertEqual(self.plugin.active_card, 'piper_joint')
                with self.assertRaises(RuntimeError):
                    self.call('enable', 'enable')
                self.assertFalse(self.call('stop', 'halt', target='gripper')['operation_stopping'])
                self.assertFalse(self.plugin.cancel.is_set())
                self.assertTrue(self.call('stop', 'halt', target='arm')['operation_stopping'])
                self.assertTrue(self.plugin.driver.cancelled.is_set())
                release.set()
                self.assertTrue(completed.wait(1))
                self.assertEqual(reports[0][-1], 'piper_joint')
                self.assertEqual(self.plugin.last_actions['piper_joint']['status'], 'cancelled')
            finally:
                release.set()

    def test_joint_execution_reaches_target_without_changing_other_axes(self):
        sdk = self.connected()
        self.plugin.motion_enabled = True
        original = sdk.raw
        plan = self.call('joint', 'move', joint=1, target_deg=2)
        result = self.fast(lambda: self.call('joint', 'move', joint=1, target_deg=2, execute=True,
            preview_id=plan['preview_id'], workspace_clear=True, take_control=True))
        self.assertTrue(result['executed'])
        self.assertEqual(sdk.raw, (2000, *original[1:]))
        self.assertFalse(any(c[0] in {'enable', 'disable', 'gripper'} for c in sdk.calls))

    def test_gripper_disable_is_separate_and_does_not_disable_arm(self):
        sdk = self.connected()
        sdk.gripper_enabled = True
        self.plugin.motion_enabled = True
        with self.assertRaises(ValueError):
            self.call('disable', 'disable', object='gripper')
        self.assertEqual(sdk.calls, [])
        result = self.call('disable', 'disable', object='gripper', supported=True)
        self.assertTrue(result['confirmed'])
        self.assertFalse(sdk.gripper_enabled)
        self.assertTrue(all(sdk.enabled))
        self.assertTrue(all(c[0] == 'gripper' for c in sdk.calls))

    def test_camera_denied_before_launch_and_start_is_passive(self):
        self.call('photo', 'start')
        with patch('agilex.piper.runtime.subprocess.Popen') as process:
            with self.assertRaisesRegex(RuntimeError, '默认关闭'):
                self.call('photo', 'capture', capture_confirmed=True)
            process.assert_not_called()
        self.assertIsNone(self.plugin.driver)

    def test_stop_scope_tracks_gripper_enable_on_shared_enable_card(self):
        self.connected()
        self.plugin.motion_enabled = True
        self.plugin.worker_context.operation_object = 'gripper'
        with self.plugin.operation('piper_enable', 'enable'):
            self.assertFalse(self.call('stop', 'halt', target='arm')['operation_stopping'])
            self.assertFalse(self.plugin.cancel.is_set())
            self.assertTrue(self.call('stop', 'halt', target='gripper')['operation_stopping'])
            self.assertTrue(self.plugin.cancel.is_set())
        self.plugin.worker_context.operation_object = None

    def test_permission_block_can_cancel_public_joint_while_busy(self):
        self.connected()
        self.plugin.motion_enabled = self.plugin.manual_motion_permission = True
        with self.plugin.operation('piper_joint', 'move'):
            result = self.call('permission', 'block')
            self.assertTrue(result['operation_stopping'])
            self.assertFalse(self.plugin.motion_enabled)
            self.assertTrue(self.plugin.driver.cancelled.is_set())

    def test_http_exposes_ten_and_legacy_tools_cannot_be_called(self):
        server = ThreadingHTTPServer(('127.0.0.1', 0), make_handler(lambda: self.bundle, 'test', 'test'))
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        try:
            def rpc(method, params):
                req = urllib.request.Request(f'http://127.0.0.1:{server.server_port}/mcp',
                    json.dumps({'jsonrpc': '2.0', 'id': 1, 'method': method, 'params': params}).encode(),
                    {'Content-Type': 'application/json'})
                with urllib.request.urlopen(req) as response:return json.load(response)
            self.assertEqual(len(rpc('tools/list', {})['result']['tools']), 10)
            self.assertEqual(rpc('tools/call', {'name': 'piper_arm', 'arguments': {'action': 'prepare'}})['error']['code'], -32601)
            self.assertIsNone(self.plugin.driver)
        finally:
            server.shutdown();server.server_close();thread.join(1)
