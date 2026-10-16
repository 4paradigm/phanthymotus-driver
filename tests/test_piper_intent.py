"""Retained intent primitives against a recording SDK; no hardware access.

The replacement production bundle is covered by test_piper_unified.py.
"""
from copy import deepcopy
import json
import math
import tempfile
import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from piper_legacy_fixture import build_plugins
from agilex.piper.intent_motion import DIRECTIONS, end_pose, move_cartesian
from agilex.piper.intent_control import TASK_ACTIONS
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
        motion_env = patch.dict('os.environ', {'PIPER_MOTION_ENABLED': '1'})
        motion_env.start()
        self.addCleanup(motion_env.stop)
        self.plugins = build_plugins({'data_dir': self.tmp.name, 'async_actions': False,
                                      'name': '机械臂 1'}, 'agilex', None)
        self.runtime, self.background = self.plugins[:2]
        # Positive motion fixtures model an explicitly authorized deployment.
        self.assertTrue(self.runtime.motion_enabled)
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
                         {*('move_' + d for d in DIRECTIONS), 'stop_motion', 'open_gripper', 'close_gripper', 'set_gripper_opening',
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
                result = self.call('move_' + direction)
                expected = tuple(a + int(1000 * d) for a, d in zip((200000, 0, 250000), vector)) + (180000, 0, 90000)
                targets = [c[1] for c in self.sdk.calls if c[0] == 'end_pose']
                self.assertTrue(targets)
                self.assertTrue(all(t == expected for t in targets))
                self.assertEqual(result['state'], 'completed')
                self.assertEqual(result['orientation_deg'], (180., 0., 90.))
                self.assertFalse(any(c[0] in ('prepare', 'enable_arm', 'joints') for c in self.sdk.calls))
                self.assertTrue(self.runtime.motion_enabled)
                self.assertFalse(self.runtime.manual_motion_permission)

    def test_direction_action_uses_fixed_small_step_and_low_speed(self):
        self.call('move_up')
        self.assertEqual(self.sdk.pose[2], 251000)
        self.assertTrue(all(c[3] == 1 for c in self.sdk.calls if c[0] == 'mode'))

    def test_disabled_deployment_blocks_every_task_before_any_sdk_command(self):
        self.runtime.motion_enabled = False
        for mode, enabled in [('CAN_CTRL(0x1)', (True,) * 6),
                              ('STANDBY(0x0)', (False,) * 6),
                              ('TEACHING_MODE(0x2)', (True,) * 6)]:
            self.sdk.mode, self.sdk.enabled = mode, enabled
            for action in sorted(TASK_ACTIONS):
                with self.subTest(mode=mode, action=action), \
                     self.assertRaisesRegex(RuntimeError, 'PIPER_MOTION_ENABLED=1'):
                    self.call(action)
                self.assertEqual(self.sdk.calls, [])
                self.assertEqual(self.call('info')['state'], 'failed')
                self.assertFalse(self.runtime.motion_enabled)
                self.assertFalse(self.runtime.gate.locked())

    def test_default_disabled_deployment_does_not_connect_and_start_cannot_open_gate(self):
        for value in (None, '0'):
            with self.subTest(value=value), patch.dict('os.environ', {}, clear=True):
                if value is not None:
                    import os
                    os.environ['PIPER_MOTION_ENABLED'] = value
                plugins = build_plugins({'data_dir': self.tmp.name, 'async_actions': False}, 'agilex', None)
                runtime, background = plugins[:2]
                background.auto_connect = False
                runtime.factory = lambda *args: self.fail('Disabled task constructed a CAN driver')
                bundle = DriverBundle(plugins)
                try:
                    bundle.dispatch('arm_control', {'action': 'start'})
                    for action in sorted(TASK_ACTIONS):
                        with self.assertRaisesRegex(RuntimeError, 'Motion is disabled'):
                            bundle.dispatch('arm_control', {'action': action})
                    self.assertFalse(runtime.motion_enabled)
                    self.assertIsNone(runtime.driver)
                    self.assertFalse(bundle.dispatch('arm_control', {'action': 'select_action'})['executed'])
                    self.assertEqual(bundle.dispatch('arm_control', {'action': 'stop_motion'})['state'], 'stopped')
                    self.assertIn('<robot', bundle.dispatch('model', {})['urdf'])
                finally:
                    runtime.stop(); background.stop(); plugins[2].stop()

    def test_gate_is_rechecked_after_feedback_and_preparation(self):
        original = self.driver.get_state
        def revoke_on_feedback():
            self.runtime.motion_enabled = False
            return original()
        self.driver.get_state = revoke_on_feedback
        with self.assertRaisesRegex(RuntimeError, 'Motion is disabled'):
            self.call('move_up')
        self.assertEqual(self.sdk.calls, [])
        self.driver.get_state = original
        self.runtime.motion_enabled = True
        self.sdk.mode = 'STANDBY(0x0)'
        self.sdk.enabled = (False,) * 6
        prepare = self.driver.prepare_control
        def revoke_after_prepare(**kwargs):
            result = prepare(**kwargs)
            self.runtime.motion_enabled = False
            return result
        self.driver.prepare_control = revoke_after_prepare
        with self.assertRaisesRegex(RuntimeError, 'Motion is disabled'):
            self.call('move_up')
        self.assertEqual([c[0] for c in self.sdk.calls], ['prepare'])

    def test_teaching_permission_revoked_during_firmware_or_stability_never_controls(self):
        from agilex.piper.intent_teaching import exit_teaching_and_hold
        firmware = self.driver.firmware
        for stage in ('firmware', 'stability'):
            self.sdk.mode = 'TEACHING_MODE(0x2)'
            self.sdk.teach = 'STOP_RECORDING(0x2)'
            self.sdk.calls.clear()
            self.runtime.motion_enabled = True
            clock = FastClock()
            def query(**kwargs):
                result = firmware(**kwargs)
                if stage == 'firmware':self.runtime.motion_enabled = False
                return result
            def sleep(seconds):
                if stage == 'stability':self.runtime.motion_enabled = False
                clock.sleep(seconds)
            with self.subTest(stage=stage), patch.object(self.driver, 'firmware', side_effect=query), \
                 patch('agilex.piper.intent_control.exit_teaching_and_hold',
                       side_effect=lambda *a, **kw: exit_teaching_and_hold(*a, **kw, clock=clock.now, sleep=sleep)), \
                 self.assertRaisesRegex(RuntimeError, 'Motion is disabled'):
                self.call('exit_teaching_and_hold')
            self.assertEqual([c for c in self.sdk.calls if c[0] != 'firmware_query'], [])
            self.assertEqual(self.call('info')['state'], 'failed')

    def test_async_disabled_deployment_reports_failure_without_sdk_commands(self):
        self.runtime.config['async_actions'] = True
        self.runtime.motion_enabled = False
        for action in sorted(TASK_ACTIONS):
            complete, entered, release = threading.Event(), threading.Event(), threading.Event()
            require = self.runtime.require_motion_permission
            def delayed_gate():
                entered.set()
                release.wait(2)
                require()
            with self.subTest(action=action), patch.object(self.runtime, 'acp_complete',
                    side_effect=lambda *args: complete.set()), \
                 patch.object(self.runtime, 'require_motion_permission', side_effect=delayed_gate):
                try:
                    self.assertEqual(self.call(action)['state'], 'running')
                    self.assertTrue(entered.wait(1))
                finally:
                    release.set()
                self.assertTrue(complete.wait(2))
            task = self.call('info')['task']
            self.assertEqual(task['state'], 'failed')
            self.assertIn('Motion is disabled', task['error'])
            self.assertEqual(self.sdk.calls, [])

    def test_control_card_discovers_background_camera_only_when_configured(self):
        def outputs(runtime):
            return runtime.get_tools()[0]['topic_out']
        self.assertFalse(any(t['format'] == 'image/jpeg' for t in outputs(self.runtime)))
        self.runtime.config['camera'] = {'live': {'enabled': True}}
        self.assertEqual(outputs(self.runtime)[-1],
                         {'topic': '/agilex/piper/camera/live', 'format': 'image/jpeg'})
        other = build_plugins({'camera': {'device': '/dev/piper-camera-2',
                               'live': {'enabled': True}}, 'data_dir': self.tmp.name}, 'piper2', None)
        self.addCleanup(other[0].stop)
        self.addCleanup(other[1].stop)
        self.addCleanup(other[2].stop)
        self.assertEqual(outputs(other[0])[-1],
                         {'topic': '/piper2/piper/camera/live', 'format': 'image/jpeg'})
        self.assertEqual(outputs(other[0])[-1], other[2].get_tools()[0]['topic_out'][0])
        self.assertEqual(self.sdk.calls, [])

    def test_move_waits_for_controller_planning_without_resending_target(self):
        # This controller needs uninterrupted planning time. Every submitted
        # target restarts that delay, even when the coordinates are identical.
        clock = FastClock()
        pending = {}
        self.sdk.follow_pose = False
        original = self.sdk.EndPoseCtrl
        def send(*target):
            original(*target)
            self.sdk.reached = False
            pending.update(target=target, ready_at=clock.now() + .3)
        def sleep(seconds):
            clock.sleep(seconds)
            if pending and clock.now() >= pending['ready_at']:
                self.sdk.pose = pending['target']
                self.sdk.reached = True
        self.sdk.EndPoseCtrl = send
        result = move_cartesian(self.driver, 'forward', 1, 1,
                                clock=clock.now, sleep=sleep)
        self.assertTrue(result['executed'])
        self.assertGreaterEqual(clock.now(), .8)
        self.assertEqual(sum(c[0] == 'end_pose' for c in self.sdk.calls), 1)
        self.assertEqual(sum(c[0] == 'mode' for c in self.sdk.calls), 1)

    def test_unmoving_controller_is_not_flooded_with_repeated_goals(self):
        self.sdk.follow_pose = False
        reports = []
        clock = FastClock()
        with self.assertRaisesRegex(RuntimeError, 'timed out'):
            move_cartesian(self.driver, 'forward', 1, 1, clock=clock.now,
                           sleep=clock.sleep, report=lambda d: reports.append(deepcopy(d)))
        targets = [c[1] for c in self.sdk.calls if c[0] == 'end_pose']
        # One movement submission and the existing best-effort measured hold.
        self.assertEqual(targets, [(201000, 0, 250000, 180000, 0, 90000), self.sdk.pose])
        self.assertEqual(reports[-1]['commands_sent'], 1)
        self.assertEqual(reports[-1]['actual_displacement_mm'], 0)

    def test_bad_arguments_and_old_actions_send_nothing(self):
        cases = [{'distance_mm': 0}, {'distance_mm': 11}, {'distance_mm': True},
                 {'distance_mm': float('nan')}, {'speed_percent': 0}, {'speed_percent': 11},
                 {'speed_percent': 1.5}, {'speed_percent': '5'}, {'direction': 'camera_left'},
                 {'execute': True}, {'preview_id': 'old'}]
        for args in cases:
            with self.subTest(args=args), self.assertRaises(ValueError):
                self.call('move_up', **args)
        for action in ('move', 'enable', 'disable', 'allow_motion', 'move_joint', 'jog_positive', 'switch_mode'):
            with self.subTest(action=action), self.assertRaisesRegex(ValueError, 'replaced'):
                self.call(action)
        self.assertEqual(self.sdk.calls, [])

    def test_preparation_once_before_move_then_repeated_move_is_already_ready(self):
        self.sdk.enabled = (False,) * 6
        self.sdk.mode = 'STANDBY(0x0)'
        self.call('move_up')
        self.assertEqual(self.sdk.calls[0][0], 'prepare')
        self.assertFalse(self.sdk.calls[0][1]['allow_limit_adjustment'])
        self.assertEqual(self.sdk.calls[0][1]['speed_pct'], 1)
        self.call('move_down')
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
            result = self.call('move_up')
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
                self.call('move_up')
            setattr(self.sdk, attr, old)
        self.assertEqual(self.sdk.calls, [])

    def test_individual_end_frames_are_required_even_if_aggregate_is_fresh(self):
        self.sdk.frame_lock = threading.Lock()
        self.sdk.frame_times = {0x2A2: time.time(), 0x2A3: 0., 0x2A4: time.time()}
        self.sdk.frame_counts = {}
        self.sdk._arm_can = SimpleNamespace(tx_frames=[])
        with self.assertRaisesRegex(RuntimeError, 'stale'):
            self.call('move_up')
        self.assertEqual(self.sdk.calls, [])

    def test_unreached_target_times_out_and_does_not_claim_completion(self):
        self.sdk.reached = False
        with self.assertRaisesRegex(RuntimeError, 'timed out'):
            self.call('move_up')
        self.assertEqual(self.call('info')['state'], 'failed')
        self.assertFalse(self.runtime.gate.locked())

    def test_status_or_pose_loss_after_command_cancels_with_no_old_target_retry(self):
        old_send = self.sdk.EndPoseCtrl
        def send(*target):
            old_send(*target)
            self.sdk.pose_frozen = True
        self.sdk.EndPoseCtrl = send
        with self.assertRaisesRegex(RuntimeError, 'stale'):
            self.call('move_up')
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
            accepted = self.call('move_up')
            self.assertIn('action_id', accepted)
            try:
                self.assertTrue(entered.wait(1))
                self.assertEqual(self.call('info')['state'], 'preparing')
                with self.assertRaises(RuntimeError):
                    self.call('move_up')
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
        with patch('agilex.piper.intent_control.move_cartesian', side_effect=lambda *args, **kwargs: move_cartesian(*args, **kwargs, clock=clock.now, sleep=sleep)), \
             patch.object(self.runtime, 'acp_complete', side_effect=lambda *args: complete.set()):
            self.call('move_up')
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
            self.call('move_up')
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
            if action == 'move_up':
                entered.set();release.wait(2)
            return original(action, args)
        with patch.object(self.runtime, 'dispatch_sync', side_effect=sync), \
             patch.object(self.runtime, 'acp_complete', side_effect=lambda *args:complete.set()):
            self.call('move_up')
            self.assertTrue(entered.wait(1))
            self.assertEqual(self.call('stop_motion')['state'], 'stopping')
            release.set()
            self.assertTrue(complete.wait(2))
        self.assertEqual(self.call('info')['state'], 'cancelled')
        self.assertEqual(self.sdk.calls, [])

    def test_cached_reached_flag_and_orientation_error_do_not_report_completion(self):
        cached_stamp = time.time() - .1
        self.sdk.GetArmStatus = lambda:SimpleNamespace(time_stamp=cached_stamp,
            arm_status=SimpleNamespace(motion_status=0))
        with self.assertRaisesRegex(RuntimeError, 'timed out'):
            self.call('move_up')
        self.sdk.GetArmStatus = lambda:SimpleNamespace(time_stamp=time.time(),
            arm_status=SimpleNamespace(motion_status=0))
        original = self.sdk.EndPoseCtrl
        def wrong_orientation(*target):
            original(*target)
            self.sdk.pose = (*self.sdk.pose[:3], self.sdk.pose[3] + 2000, *self.sdk.pose[4:])
        self.sdk.EndPoseCtrl = wrong_orientation
        with self.assertRaisesRegex(RuntimeError, 'timed out'):
            self.call('move_up')

    def test_preparation_failure_does_not_submit_a_move(self):
        self.sdk.enabled = (False,) * 6
        self.driver.prepare_control = lambda **kwargs:(_ for _ in ()).throw(RuntimeError('Preparation failed'))
        with self.assertRaisesRegex(RuntimeError, 'Preparation failed'):
            self.call('move_up')
        self.assertEqual(self.sdk.calls, [])
        self.assertEqual(self.call('info')['state'], 'failed')


    def test_direction_actions_need_no_movement_parameters(self):
        schema = self.runtime.get_tools()[0]['inputSchema']
        self.assertFalse({'direction', 'distance_mm', 'speed_percent'} & schema['properties'].keys())
        for direction in DIRECTIONS:
            self.assertEqual(schema['x-action-params']['move_' + direction]['params'], [])
        self.assertNotIn('move', schema['properties']['action']['enum'])
        self.assertEqual(schema['properties']['action']['default'], 'select_action')
        for supplied in ({'distance_mm': 1}, {'speed_percent': 1}, {'direction': 'up'}):
            with self.assertRaisesRegex(ValueError, 'Unexpected'):
                self.call('move_up', **supplied)
        self.assertEqual(self.sdk.calls, [])

    def test_core_retained_width_does_not_change_direction_target_or_audit(self):
        # Older Core keeps a populated width input when selecting Move.
        result = self.call('move_up', width_mm=50)
        self.assertEqual(result['state'], 'completed')
        self.assertEqual(self.sdk.pose[2], 251000)
        targets = [c for c in self.sdk.calls if c[0] == 'end_pose']
        self.assertEqual(len(targets), 1)
        self.assertNotIn('width_mm', self.call('info')['task']['request'])
        self.assertFalse(any(c[0] == 'gripper' for c in self.sdk.calls))

    def test_core_retained_width_does_not_block_passive_lifecycle_or_stop(self):
        for action in ('select_action', 'info', 'start', 'stop_motion'):
            with self.subTest(action=action):
                self.call(action, width_mm=50)
        self.assertEqual(self.sdk.calls, [])
        for action in TASK_ACTIONS - {'set_gripper_opening'}:
            validated = self.runtime._args('arm_control', action,
                                          {'action': action, 'width_mm': 50})
            self.assertNotIn('width_mm', validated)
        with self.assertRaisesRegex(ValueError, 'Unexpected'):
            self.call('move_up', width_mm=50, width_typo=50)
        self.assertEqual(self.sdk.calls, [])

    def test_retained_width_does_not_bypass_teaching_guard(self):
        self.sdk.mode = 'TEACHING_MODE(0x2)'
        with self.assertRaisesRegex(RuntimeError, 'TEACHING_MODE'):
            self.call('move_up', width_mm=50)
        self.assertEqual(self.sdk.calls, [])

    def test_standby_enabled_arm_prepares_once_at_low_speed(self):
        self.sdk.mode = 'STANDBY(0x0)'
        self.call('move_up')
        self.assertEqual(self.sdk.calls[0][0], 'prepare')
        self.assertEqual(self.sdk.calls[0][1]['speed_pct'], 1)
        self.call('move_down')
        self.assertEqual(sum(c[0] == 'prepare' for c in self.sdk.calls), 1)

    def test_original_target_is_persisted_before_first_transmission(self):
        from pathlib import Path
        path = Path(self.tmp.name) / 'arm-control-events.jsonl'
        original = self.sdk.EndPoseCtrl
        def send(*target):
            entries = [json.loads(line) for line in path.read_text().splitlines()]
            self.assertTrue(any(e.get('details', {}).get('target_mm') == [200., 0., 251.] for e in entries))
            original(*target)
        self.sdk.EndPoseCtrl = send
        self.call('move_up')
        entries = [json.loads(line) for line in path.read_text().splitlines()]
        self.assertEqual(entries[0]['event'], 'request')
        self.assertEqual(entries[-1]['task']['state'], 'completed')
        self.assertEqual(entries[-1]['task']['action'], 'move_up')
        self.assertEqual(entries[-1]['task']['diagnostics']['actual_displacement_mm'], 1.)
        self.assertNotIn('token', path.read_text().lower())

    def test_latched_target_rejection_blocks_every_control_without_reset(self):
        self.sdk.status = 'TARGET_POS_EXCEEDS_LIMIT(0x4)'
        for action in ('move_up', 'move_down', 'open_gripper', 'exit_teaching_and_hold'):
            with self.assertRaisesRegex(RuntimeError, 'still active'):
                self.call(action)
            task = self.call('info')['task']
            self.assertEqual(task['outcome'], 'direction_unavailable')
            self.assertEqual(task['diagnostics']['controller_status'], self.sdk.status)
        self.assertEqual(self.sdk.calls, [])

    def test_journal_failure_before_control_fails_closed(self):
        with patch.object(self.runtime, '_audit', side_effect=OSError('Diagnostic storage unavailable')):
            with self.assertRaisesRegex(RuntimeError, 'storage unavailable'):
                self.call('move_up')
        self.assertEqual(self.sdk.calls, [])
        self.assertFalse(self.runtime.gate.locked())
        self.assertEqual(self.call('info')['state'], 'failed')

    def test_target_rejection_after_motion_records_partial_displacement(self):
        original = self.sdk.EndPoseCtrl
        def send(*target):
            original(*target)
            self.sdk.status = 'TARGET_POS_EXCEEDS_LIMIT(0x4)'
        self.sdk.EndPoseCtrl = send
        with self.assertRaisesRegex(RuntimeError, 'Cannot continue up'):
            self.call('move_up')
        task = self.call('info')['task']
        self.assertEqual(task['outcome'], 'direction_unavailable')
        self.assertEqual(task['diagnostics']['commands_sent'], 1)
        self.assertEqual(task['diagnostics']['actual_displacement_mm'], 1.)
        self.assertFalse(task['diagnostics']['measured_hold_sent'])
        self.assertEqual(task['diagnostics']['first_error_feedback']['controller_status'], self.sdk.status)
        self.assertEqual(sum(c[0] == 'end_pose' for c in self.sdk.calls), 1)
        self.assertFalse(any(c[0] in ('teach', 'joints', 'enable_arm') for c in self.sdk.calls))

    def test_internal_longer_request_uses_original_reference_and_one_mm_steps(self):
        self.motion.stop()
        clock = FastClock()
        original = self.sdk.EndPoseCtrl
        reports = []
        def send(*target):
            original(*target)
            self.sdk.pose = (*self.sdk.pose[:2], self.sdk.pose[2] - 200, *self.sdk.pose[3:])
        self.sdk.EndPoseCtrl = send
        result = move_cartesian(self.driver, 'up', 2.5, 1, clock=clock.now, sleep=clock.sleep,
                                report=lambda d:reports.append(deepcopy(d)))
        targets = [c[1] for c in self.sdk.calls if c[0] == 'end_pose']
        self.assertEqual(sorted(set(t[2] for t in targets)), [251000, 252000, 252500])
        self.assertTrue(all(t[3:] == (180000, 0, 90000) for t in targets))
        self.assertEqual(result['target_mm'], (200., 0., 252.5))
        self.assertEqual(result['completed_steps'], 3)
        self.assertAlmostEqual(result['actual_displacement_mm'], 2.3)
        self.assertEqual(reports[-1]['completed_steps'], 3)

    def test_second_step_rejection_never_sends_third_step_or_recovery(self):
        self.motion.stop()
        clock = FastClock()
        original = self.sdk.EndPoseCtrl
        reports = []
        def send(*target):
            if target[2] == 252000:
                self.sdk.calls.append(('end_pose', target))
                self.sdk.status = 'TARGET_POS_EXCEEDS_LIMIT(0x4)'
            else:
                original(*target)
        self.sdk.EndPoseCtrl = send
        with self.assertRaisesRegex(RuntimeError, 'Cannot continue up'):
            move_cartesian(self.driver, 'up', 3, 1, clock=clock.now, sleep=clock.sleep,
                           report=lambda d:reports.append(deepcopy(d)))
        self.assertEqual(sorted(set(c[1][2] for c in self.sdk.calls if c[0] == 'end_pose')), [251000, 252000])
        self.assertEqual(sum(c[0] == 'end_pose' and c[1][2] == 252000 for c in self.sdk.calls), 1)
        self.assertEqual(reports[-1]['completed_steps'], 1)
        self.assertEqual(reports[-1]['actual_displacement_mm'], 1.)
        self.assertEqual(reports[-1]['controller_status'], self.sdk.status)
        self.assertFalse(reports[-1]['measured_hold_sent'])

    def test_cancel_between_steps_never_advances_old_target(self):
        self.motion.stop()
        clock = FastClock()
        def report(details):
            if details['completed_steps'] == 1:
                self.driver.cancel_motion()
        with self.assertRaises(RuntimeError):
            move_cartesian(self.driver, 'up', 3, 1, clock=clock.now, sleep=clock.sleep, report=report)
        self.assertTrue(all(c[1][2] == 251000 for c in self.sdk.calls if c[0] == 'end_pose'))
        self.assertTrue(all(self.sdk.enabled))

    def test_device_error_bits_are_not_classified_as_direction_only(self):
        self.sdk.status = 'TARGET_POS_EXCEEDS_LIMIT(0x4)'
        self.sdk.error = 1
        with self.assertRaisesRegex(RuntimeError, 'device fault'):
            self.call('move_up')
        self.assertEqual(self.call('info')['task']['outcome'], 'operation_failed')
        self.assertEqual(self.sdk.calls, [])

    def test_core_topic_discovery_is_read_only_and_movement_stays_strict(self):
        result = self.call('info', input_topic='')
        self.assertEqual(result['state'], 'idle')
        for action, args in [('info', {'input_topic': '/unexpected'}),
                             ('move_up', {'input_topic': ''})]:
            with self.assertRaisesRegex(ValueError, 'Unexpected'):
                self.call(action, **args)
        self.assertEqual(self.sdk.calls, [])

    def test_retry_is_available_only_after_external_healthy_feedback(self):
        self.sdk.status = 'TARGET_POS_EXCEEDS_LIMIT(0x4)'
        with self.assertRaises(RuntimeError):
            self.call('move_up')
        self.assertEqual(self.sdk.calls, [])
        self.sdk.status = 'NORMAL(0x0)'  # Robot recovers externally, not via our driver.
        self.assertEqual(self.call('move_down')['state'], 'completed')
        self.assertEqual(self.sdk.pose[2], 249000)
        self.assertFalse(any(c[0] in ('teach', 'enable_arm', 'prepare', 'joints') for c in self.sdk.calls))

    def test_gripper_input_remains_strict_and_empty_width_uses_default(self):
        for value in (-1, 71, True, float('nan'), float('inf'), [], None, '20'):
            with self.subTest(value=value), self.assertRaises(ValueError):
                self.call('set_gripper_opening', width_mm=value)
        self.assertEqual(self.sdk.calls, [])
        with patch('agilex.piper.arm.time.sleep'), patch('agilex.piper.arm.time.monotonic', side_effect=[i * .1 for i in range(100)]):
            result = self.call('set_gripper_opening', width_mm='')
        self.assertEqual(result['gripper']['width_mm'], 20.)

    def test_core_start_before_second_move_reports_busy_without_cancelling(self):
        self.runtime.config['async_actions'] = True
        self.sdk.enabled = (False,) * 6
        entered, release, complete = threading.Event(), threading.Event(), threading.Event()
        original = self.driver.prepare_control
        def prepare(**kwargs):
            entered.set(); release.wait(3)
            return original(**kwargs)
        self.driver.prepare_control = prepare
        with patch.object(self.runtime, 'acp_complete', side_effect=lambda *args:complete.set()):
            self.call('start')
            self.call('move_forward')
            try:
                self.assertTrue(entered.wait(1))
                self.assertEqual(self.call('start')['state'], 'busy')
                self.assertFalse(self.runtime.cancel.is_set())
                with self.assertRaisesRegex(RuntimeError, 'busy'):
                    self.call('move_forward')
                self.assertEqual(self.call('info')['task']['action'], 'move_forward')
                self.assertEqual(self.call('info')['state'], 'preparing')
            finally:
                release.set()
            self.assertTrue(complete.wait(3))
        self.assertEqual(self.call('info')['state'], 'completed')
        self.assertEqual(sum(c[0]=='prepare' for c in self.sdk.calls),1)
        self.assertEqual(self.call('start')['state'], 'ready')
        self.call('move_backward')
        # Wait for the second async request before tearing down the fake driver.
        for _ in range(100):
            if not self.runtime.gate.locked():break
            time.sleep(.01)
        self.assertFalse(self.runtime.gate.locked())

    def test_core_start_before_stop_is_reachable_and_cannot_clear_cancellation(self):
        self.runtime.config['async_actions'] = True
        entered, release, complete = threading.Event(), threading.Event(), threading.Event()
        clock = FastClock()
        def sleep(seconds):
            entered.set();release.wait(3);clock.sleep(seconds)
        self.motion.stop()
        with patch('agilex.piper.intent_control.move_cartesian',
                   side_effect=lambda *args,**kwargs:move_cartesian(*args,**kwargs,clock=clock.now,sleep=sleep)), \
             patch.object(self.runtime,'acp_complete',side_effect=lambda *args:complete.set()):
            self.call('start');self.call('move_forward')
            try:
                self.assertTrue(entered.wait(1))
                self.assertEqual(self.call('start')['state'],'busy')
                self.assertEqual(self.call('stop_motion')['state'],'stopping')
                count=len(self.sdk.calls)
                self.assertEqual(self.call('start')['state'],'stopping')
                self.assertTrue(self.runtime.cancel.is_set())
                self.assertTrue(self.driver.cancelled.is_set())
                self.assertEqual(len(self.sdk.calls),count)
                self.assertEqual(self.call('stop_motion')['state'],'stopping')
                with self.assertRaisesRegex(RuntimeError,'busy'):
                    self.call('move_backward')
            finally:
                release.set()
            self.assertTrue(complete.wait(3))
        self.assertEqual(self.call('info')['state'],'cancelled')
        self.assertEqual(self.call('start')['state'],'ready')
        self.assertTrue(all(self.sdk.enabled))
        targets=[c[1] for c in self.sdk.calls if c[0]=='end_pose']
        self.assertEqual(len(targets),2)  # original goal, then measured hold
        self.assertTrue(all(t[0]==201000 for t in targets))

    def test_start_during_card_shutdown_preserves_stopped_flags_until_worker_finishes(self):
        self.runtime.config['async_actions'] = True
        self.sdk.enabled=(False,)*6
        entered,release,complete=threading.Event(),threading.Event(),threading.Event()
        original=self.driver.prepare_control
        def prepare(**kwargs):
            entered.set();release.wait(3);return original(**kwargs)
        self.driver.prepare_control=prepare
        with patch.object(self.runtime,'acp_complete',side_effect=lambda *args:complete.set()):
            self.call('move_forward')
            try:
                self.assertTrue(entered.wait(1))
                self.call('stop')
                self.assertIn('arm_control',self.runtime.stopped_cards)
                self.assertEqual(self.call('start')['state'],'stopping')
                self.assertIn('arm_control',self.runtime.stopped_cards)
                self.assertTrue(self.runtime.cancel.is_set())
            finally:release.set()
            self.assertTrue(complete.wait(3))
        self.assertEqual(self.sdk.calls,[])
        self.assertEqual(self.call('start')['state'],'ready')
        self.assertNotIn('arm_control',self.runtime.stopped_cards)

if __name__ == '__main__':
    unittest.main()
