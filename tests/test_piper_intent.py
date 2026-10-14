"""Production intent interface against a recording SDK; no hardware access."""
from copy import deepcopy
import json
import math
import tempfile
import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from agilex.piper.device import build_plugins
from agilex.piper.intent_motion import DIRECTIONS, end_pose, move_cartesian
from common.vendor_runtime import DriverBundle
from test_piper_controls import Driver, SDK


class IntentSDK(SDK):
    def __init__(self):
        super().__init__()
        self.pose = (200000, 0, 250000, 180000, 0, 90000)
        self.pose_frozen = False
        self.status_frozen = False
        self.follow_pose = True
        self.reached = True
    def GetArmEndPoseMsgs(self):
        fields = ('X_axis', 'Y_axis', 'Z_axis', 'RX_axis', 'RY_axis', 'RZ_axis')
        return SimpleNamespace(time_stamp=0 if self.pose_frozen else time.time(),
                               end_pose=SimpleNamespace(**dict(zip(fields, self.pose))))
    def GetArmStatus(self):
        return SimpleNamespace(time_stamp=0 if self.status_frozen else time.time(),
                               arm_status=SimpleNamespace(motion_status=0 if self.reached else 1))
    def EndPoseCtrl(self, *target):
        self.calls.append(('end_pose', target))
        if self.follow_pose:
            self.pose = target
    def EnablePiper(self):
        self.calls.append(('enable_arm',))
        self.enabled = (True,) * 6


class IntentDriver(Driver):
    def __init__(self, can_name='can0'):
        super().__init__(can_name)
        self.arm = IntentSDK()
    def prepare_control(self, **kwargs):
        # The actual preparation primitive retains its dedicated tests. This
        # fixture verifies ownership/order around preparation and execution.
        self._check_cancelled()
        self.arm.calls.append(('prepare', kwargs))
        self.arm.mode = 'CAN_CTRL(0x1)'
        self.arm.enabled = (True,) * 6
        return {'executed': True}


class FastClock:
    def __init__(self): self.value = 0.
    def now(self): return self.value
    def sleep(self, seconds): self.value += seconds


def fast_move(*args, **kwargs):
    clock = FastClock()
    return move_cartesian(*args, **kwargs, clock=clock.now, sleep=clock.sleep)


class IntentTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.plugins = build_plugins({'data_dir': self.tmp.name, 'async_actions': False,
                                      'name': '机械臂 1'}, 'agilex', None)
        self.runtime, self.background = self.plugins[:2]
        self.runtime.factory = IntentDriver
        self.background.auto_connect = False
        self.bundle = DriverBundle(self.plugins)
        self.addCleanup(self.runtime.stop)
        self.addCleanup(self.background.stop)
        self.driver = self.runtime.connected()
        self.sdk = self.driver.arm
        self.motion = patch('agilex.piper.intent_control.move_cartesian', side_effect=fast_move)
        self.motion.start()
        self.addCleanup(self.motion.stop)

    def call(self, action, **args):
        return self.bundle.dispatch('arm_control', {'action': action, **args})

    def test_only_user_actions_and_english_tools_with_background_outputs(self):
        definitions = self.bundle.get_all_tools()
        self.assertNotRegex(json.dumps(definitions, ensure_ascii=False), r'[\u3400-\u9fff]')
        tools = {t['name']: t for t in definitions}
        self.assertEqual(set(tools), {'arm_control', 'model', 'piper_photo', 'piper_camera_live'})
        self.assertEqual(tools['model']['type'], 'resource')
        actions = tools['arm_control']['inputSchema']['properties']['action']['enum']
        self.assertEqual(set(actions) - {'start', 'stop', 'info', 'select_action'},
                         {'move', 'stop_motion', 'open_gripper', 'close_gripper', 'set_gripper_opening',
                          'exit_teaching_and_hold'})
        self.assertEqual(tools['arm_control']['topic_out'][0]['topic'], '/agilex/piper/state/joints')
        self.assertFalse(self.call('select_action')['executed'])
        self.assertEqual(self.sdk.calls, [])
        self.assertIn('<robot', self.bundle.dispatch('model', {})['urdf'])
        self.background.start()
        params = self.background.monitor.read()['parameters']
        self.assertEqual(params['Feedback'], 'fresh')
        self.assertEqual(params['End position (mm)'], [200., 0., 250.])
        self.assertNotRegex(json.dumps(params, ensure_ascii=False), r'[\u3400-\u9fff]')

    def test_all_directions_units_orientation_and_target_is_not_accumulated(self):
        for direction, vector in DIRECTIONS.items():
            with self.subTest(direction=direction):
                self.sdk.pose = (200000, 0, 250000, 180000, 0, 90000)
                self.sdk.calls.clear()
                result = self.call('move', direction=direction, distance_mm=1, speed_percent=1)
                expected = tuple(a + int(1000 * d) for a, d in zip((200000, 0, 250000), vector)) + (180000, 0, 90000)
                targets = [c[1] for c in self.sdk.calls if c[0] == 'end_pose']
                self.assertTrue(targets)
                self.assertTrue(all(t == expected for t in targets))
                self.assertEqual(result['state'], 'completed')
                self.assertEqual(result['orientation_deg'], (180., 0., 90.))
                self.assertFalse(any(c[0] in ('prepare', 'enable_arm', 'joints') for c in self.sdk.calls))
                self.assertFalse(self.runtime.motion_enabled)

    def test_empty_and_missing_optional_parameters_use_declared_defaults(self):
        self.call('move', direction='', distance_mm='', speed_percent='')
        self.assertEqual(self.sdk.pose[2], 255000)
        self.assertTrue(all(c[3] == 5 for c in self.sdk.calls if c[0] == 'mode'))

    def test_bad_arguments_and_old_actions_send_nothing(self):
        cases = [{'distance_mm': 0}, {'distance_mm': 11}, {'distance_mm': True},
                 {'distance_mm': float('nan')}, {'speed_percent': 0}, {'speed_percent': 11},
                 {'speed_percent': 1.5}, {'speed_percent': '5'}, {'direction': 'camera_left'},
                 {'execute': True}, {'preview_id': 'old'}]
        for args in cases:
            with self.subTest(args=args), self.assertRaises(ValueError):
                self.call('move', **args)
        for action in ('enable', 'disable', 'allow_motion', 'move_joint', 'jog_positive', 'switch_mode'):
            with self.subTest(action=action), self.assertRaisesRegex(ValueError, 'replaced'):
                self.call(action)
        self.assertEqual(self.sdk.calls, [])

    def test_preparation_once_before_move_then_repeated_move_is_already_ready(self):
        self.sdk.enabled = (False,) * 6
        self.sdk.mode = 'STANDBY(0x0)'
        self.call('move', direction='up', distance_mm=1, speed_percent=1)
        self.assertEqual(self.sdk.calls[0][0], 'prepare')
        self.assertFalse(self.sdk.calls[0][1]['allow_limit_adjustment'])
        self.assertEqual(self.sdk.calls[0][1]['speed_pct'], 1)
        self.call('move', direction='down', distance_mm=1)
        self.assertEqual(sum(c[0] == 'prepare' for c in self.sdk.calls), 1)

    def test_real_preparation_primitive_is_compatible_with_intent_ownership(self):
        from agilex.piper.arm import PiperDriver
        self.driver.prepare_control = PiperDriver.prepare_control.__get__(self.driver)
        self.sdk.enabled = (False,) * 6
        self.sdk.mode = 'STANDBY(0x0)'
        tick = [0.]
        def now():
            tick[0] += .05
            return tick[0]
        with patch('agilex.piper.arm.time.sleep'), patch('agilex.piper.arm.time.monotonic', side_effect=now):
            result = self.call('move', distance_mm=1)
        self.assertTrue(result['executed'])
        kinds = [c[0] for c in self.sdk.calls]
        self.assertIn('enable_arm', kinds)
        self.assertLess(kinds.index('enable_arm'), kinds.index('end_pose'))

    def test_fault_mode_partial_enable_and_stale_pose_never_prepare(self):
        for attr, value in [('mode', 'TEACHING_MODE(0x2)'), ('mode', 'WIFI_CONTROL_MODE(0x4)'),
                            ('enabled', (True, False, True, True, True, True)), ('error', 1),
                            ('frozen', True), ('pose_frozen', True), ('status', 'SINGULARITY_POINT(0x3)')]:
            old = getattr(self.sdk, attr)
            setattr(self.sdk, attr, value)
            with self.subTest(attr=attr, value=value), self.assertRaises(RuntimeError):
                self.call('move')
            setattr(self.sdk, attr, old)
        self.assertEqual(self.sdk.calls, [])

    def test_individual_end_frames_are_required_even_if_aggregate_is_fresh(self):
        self.sdk.frame_lock = threading.Lock()
        self.sdk.frame_times = {0x2A2: time.time(), 0x2A3: 0., 0x2A4: time.time()}
        self.sdk.frame_counts = {}
        self.sdk._arm_can = SimpleNamespace(tx_frames=[])
        with self.assertRaisesRegex(RuntimeError, 'stale'):
            self.call('move')
        self.assertEqual(self.sdk.calls, [])

    def test_unreached_target_times_out_and_does_not_claim_completion(self):
        self.sdk.reached = False
        with self.assertRaisesRegex(RuntimeError, 'timed out'):
            self.call('move', distance_mm=1)
        self.assertEqual(self.call('info')['state'], 'failed')
        self.assertFalse(self.runtime.gate.locked())

    def test_status_or_pose_loss_after_command_cancels_with_no_old_target_retry(self):
        old_send = self.sdk.EndPoseCtrl
        def send(*target):
            old_send(*target)
            self.sdk.pose_frozen = True
        self.sdk.EndPoseCtrl = send
        with self.assertRaisesRegex(RuntimeError, 'stale'):
            self.call('move')
        self.assertEqual(sum(c[0] == 'end_pose' for c in self.sdk.calls), 1)

    def test_gripper_prepares_and_enables_at_measured_width_before_target(self):
        with patch('agilex.piper.arm.time.sleep'), patch('agilex.piper.arm.time.monotonic', side_effect=[i * .1 for i in range(100)]):
            result = self.call('open_gripper')
        widths = [c[1] for c in self.sdk.calls if c[0] == 'gripper']
        self.assertEqual(widths[0], 20000)
        self.assertEqual(widths[-1], 70000)
        self.assertTrue(result['gripper']['enabled'])
        self.assertEqual(result['state'], 'completed')
        self.assertTrue(all(c[4] == 0 for c in self.sdk.calls if c[0] == 'gripper'))

    def test_width_metadata_uses_effective_limits_and_rejects_before_prepare(self):
        self.driver.gripper_limits_mm = (25, 40)
        width = self.runtime.get_tools()[0]['inputSchema']['properties']['width_mm']
        self.assertEqual((width['minimum'], width['maximum'], width['default']), (25, 40, 25))
        self.sdk.enabled = (False,) * 6
        with self.assertRaises(ValueError):
            self.call('set_gripper_opening', width_mm=20)
        self.assertEqual(self.sdk.calls, [])

    def test_stop_during_prepare_and_concurrent_request_never_submit_target(self):
        self.runtime.config['async_actions'] = True
        self.sdk.enabled = (False,) * 6
        entered, release, complete = threading.Event(), threading.Event(), threading.Event()
        original = self.driver.prepare_control
        def prepare(**kwargs):
            entered.set(); release.wait(3)
            return original(**kwargs)
        self.driver.prepare_control = prepare
        with patch.object(self.runtime, 'acp_complete', side_effect=lambda *args: complete.set()):
            accepted = self.call('move', distance_mm=1)
            self.assertIn('action_id', accepted)
            try:
                self.assertTrue(entered.wait(1))
                self.assertEqual(self.call('info')['state'], 'preparing')
                with self.assertRaises(RuntimeError):
                    self.call('move')
                self.assertEqual(self.call('stop_motion')['state'], 'stopping')
                self.assertEqual(self.call('stop_motion')['state'], 'stopping')
            finally:
                release.set()
            self.assertTrue(complete.wait(3))
        self.assertEqual(self.call('info')['state'], 'cancelled')
        self.assertEqual(self.sdk.calls, [])
        self.assertTrue(self.call('stop_motion')['state'] == 'stopped')

    def test_stop_during_move_preserves_torque_and_reports_actual_cancellation(self):
        self.runtime.config['async_actions'] = True
        entered, release, complete = threading.Event(), threading.Event(), threading.Event()
        clock = FastClock()
        def sleep(seconds):
            entered.set(); release.wait(3); clock.sleep(seconds)
        self.motion.stop()
        with patch('agilex.piper.intent_control.move_cartesian', side_effect=lambda *args: move_cartesian(*args, clock=clock.now, sleep=sleep)), \
             patch.object(self.runtime, 'acp_complete', side_effect=lambda *args: complete.set()):
            self.call('move', distance_mm=1)
            try:
                self.assertTrue(entered.wait(1))
                self.assertEqual(self.call('stop_motion')['state'], 'stopping')
            finally:
                release.set()
            self.assertTrue(complete.wait(3))
        self.assertEqual(self.call('info')['state'], 'cancelled')
        self.assertTrue(all(self.sdk.enabled))
        self.assertEqual(sum(c[0] == 'end_pose' for c in self.sdk.calls), 2)  # target, then measured hold

    def test_background_lifecycle_and_two_device_topics_do_not_send_controls(self):
        self.background.start(); self.background.stop()
        other = build_plugins({'data_dir': self.tmp.name + '/arm2', 'can_interface': 'can1'}, 'piper2', None)
        self.addCleanup(other[0].stop)
        self.addCleanup(other[1].stop)
        self.assertEqual(other[0].get_tools()[0]['topic_out'][0]['topic'], '/piper2/piper/state/joints')
        self.assertEqual(self.sdk.calls, [])
        self.assertIsNone(other[0].driver)

    def test_monitor_does_not_wait_for_lifecycle_lock_during_stop(self):
        locked, release = threading.Event(), threading.Event()
        def hold():
            with self.runtime.lifecycle:
                locked.set();release.wait(2)
        thread = threading.Thread(target=hold);thread.start()
        try:
            self.assertTrue(locked.wait(1))
            result = self.runtime.read_monitor_snapshot()
            self.assertEqual(result['task']['state'], 'idle')
            self.assertTrue(result['fresh'])
            self.assertFalse(release.is_set())
        finally:
            release.set();thread.join(1)

    def test_stop_and_completed_publication_are_serialized_without_stuck_stopping(self):
        self.runtime.config['async_actions'] = True
        publishing, release, complete = threading.Event(), threading.Event(), threading.Event()
        original = self.runtime._phase
        def phase(state, **values):
            if state == 'completed':
                publishing.set();release.wait(3)
            return original(state, **values)
        stops = []
        with patch.object(self.runtime, '_phase', side_effect=phase), \
             patch.object(self.runtime, 'acp_complete', side_effect=lambda *args:complete.set()):
            self.call('move', distance_mm=1)
            self.assertTrue(publishing.wait(1))
            stop = threading.Thread(target=lambda:stops.append(self.call('stop_motion')))
            stop.start()
            release.set();stop.join(2)
            self.assertTrue(complete.wait(2))
        self.assertFalse(stop.is_alive())
        self.assertEqual(stops[0]['state'], 'stopped')
        self.assertEqual(self.call('info')['state'], 'completed')

    def test_stop_immediately_after_async_reservation_cancels_before_commands(self):
        self.runtime.config['async_actions'] = True
        entered, release, complete = threading.Event(), threading.Event(), threading.Event()
        original = self.runtime.dispatch_sync
        def sync(action, args):
            if action == 'move':
                entered.set();release.wait(2)
            return original(action, args)
        with patch.object(self.runtime, 'dispatch_sync', side_effect=sync), \
             patch.object(self.runtime, 'acp_complete', side_effect=lambda *args:complete.set()):
            self.call('move')
            self.assertTrue(entered.wait(1))
            self.assertEqual(self.call('stop_motion')['state'], 'stopping')
            release.set()
            self.assertTrue(complete.wait(2))
        self.assertEqual(self.call('info')['state'], 'cancelled')
        self.assertEqual(self.sdk.calls, [])

    def test_cached_reached_flag_and_orientation_error_do_not_report_completion(self):
        self.sdk.GetArmStatus = lambda:SimpleNamespace(time_stamp=time.time() - .1,
            arm_status=SimpleNamespace(motion_status=0))
        with self.assertRaisesRegex(RuntimeError, 'timed out'):
            self.call('move', distance_mm=1)
        self.sdk.GetArmStatus = lambda:SimpleNamespace(time_stamp=time.time(),
            arm_status=SimpleNamespace(motion_status=0))
        original = self.sdk.EndPoseCtrl
        def wrong_orientation(*target):
            original(*target)
            self.sdk.pose = (*self.sdk.pose[:3], self.sdk.pose[3] + 2000, *self.sdk.pose[4:])
        self.sdk.EndPoseCtrl = wrong_orientation
        with self.assertRaisesRegex(RuntimeError, 'timed out'):
            self.call('move', distance_mm=1)

    def test_preparation_failure_does_not_submit_a_move(self):
        self.sdk.enabled = (False,) * 6
        self.driver.prepare_control = lambda **kwargs:(_ for _ in ()).throw(RuntimeError('Preparation failed'))
        with self.assertRaisesRegex(RuntimeError, 'Preparation failed'):
            self.call('move')
        self.assertEqual(self.sdk.calls, [])
        self.assertEqual(self.call('info')['state'], 'failed')


if __name__ == '__main__':
    unittest.main()
