"""Waypoint persistence and production dispatch with recorded SDK commands only."""
from dataclasses import replace
import fcntl
import os
from pathlib import Path
import tempfile
import threading
import time
import unittest
from unittest.mock import patch

from agilex.piper.device import build_plugins
from agilex.piper.waypoints import WaypointStore, waypoint_name
from common.vendor_runtime import DriverBundle
import test_piper_unified as unified_tests
from test_piper_unified import fast_capture


def record(**values):
    return dict(joints_deg=[1, 10, -10, 0, 20, 0], device_serial='usb-1', can_interface='can0',
                saved_at=time.time(), feedback_time=time.time(), gripper=None, **values)


class StoreTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.store = WaypointStore(self.tmp.name)

    def test_crud_restart_unicode_names_and_safe_path(self):
        self.assertEqual(self.store.list(), [])
        for name in ['取料位', '放料位', '../name']:
            self.store.capture(name, record())
        restarted = WaypointStore(self.tmp.name)
        self.assertEqual(len(restarted.list()), 3)
        self.assertEqual(restarted.read(' 取料位 ')['device_serial'], 'usb-1')
        self.assertTrue(restarted.delete('取料位')['deleted'])
        with self.assertRaisesRegex(ValueError, 'does not exist'):
            self.store.read('取料位')
        self.assertFalse((Path(self.tmp.name).parent / 'name').exists())

    def test_duplicate_requires_explicit_overwrite(self):
        self.store.capture('位', record())
        before = self.store.path.read_bytes()
        with self.assertRaisesRegex(ValueError, 'already exists'):
            self.store.capture('位', record())
        self.assertEqual(before, self.store.path.read_bytes())
        updated = record(); updated['joints_deg'][0] = 2
        self.store.capture('位', updated, overwrite=True)
        self.assertEqual(self.store.read('位')['joints_deg'][0], 2)

    def test_failed_replace_keeps_old_file_and_removes_temporary(self):
        self.store.capture('old', record())
        before = self.store.path.read_bytes()
        with patch('agilex.piper.waypoints.os.replace', side_effect=OSError('disk failure')):
            with self.assertRaises(OSError):
                self.store.capture('new', record())
        self.assertEqual(before, self.store.path.read_bytes())
        self.assertEqual(list(Path(self.tmp.name).glob('.waypoints-*')), [])

    def test_corrupt_and_unsupported_files_are_never_overwritten(self):
        for content in ['{broken', '{"version":2,"points":{}}', '{"version":1,"points":{"x":{}}}']:
            self.store.path.write_text(content)
            with self.assertRaises(ValueError):
                self.store.capture('new', record())
            self.assertEqual(content, self.store.path.read_text())

    def test_invalid_inputs_and_missing_delete_preserve_library(self):
        for name in ['', '  ', '\n', 'x\x00y', 'x'*81, None]:
            with self.subTest(name=name), self.assertRaises(ValueError):
                waypoint_name(name)
        for joints in [[0]*5, [0]*5+[float('nan')], [True]*6]:
            invalid = record(); invalid['joints_deg'] = joints
            with self.assertRaises(ValueError):
                self.store.capture('bad', invalid)
        with self.assertRaisesRegex(ValueError, 'does not exist'):
            self.store.delete('missing')
        self.assertFalse(self.store.path.exists())

    def test_concurrent_store_instances_do_not_lose_updates(self):
        errors = []
        def save(i):
            try:
                WaypointStore(self.tmp.name).capture(str(i), record())
            except Exception as exc:
                errors.append(exc)
        threads = [threading.Thread(target=save, args=(i,)) for i in range(12)]
        for t in threads: t.start()
        for t in threads: t.join(2)
        self.assertEqual(errors, [])
        self.assertEqual(len(self.store.list()), 12)


class WaypointTests(unittest.TestCase):
    tearDown = unified_tests.UnifiedTests.tearDown

    def setUp(self):
        unified_tests.UnifiedTests.setUp(self)
        for target, kwargs in [('adapter_serial', {'return_value': 'usb-1'}),
                               ('capture_position', {'side_effect': fast_capture})]:
            mock = patch('agilex.piper.waypoint_control.' + target, **kwargs)
            mock.start(); self.addCleanup(mock.stop)

    def call(self, action, **args):
        return self.bundle.dispatch('waypoints', {'action': action, **args})

    def save(self, name='取料位'):
        return self.call('capture', name=name)

    def test_schema_catalog_and_passive_lifecycle(self):
        tools = {t['name']: t for t in self.bundle.get_all_tools()}
        schema = tools['waypoints']['inputSchema']
        self.assertEqual(schema['x-resource'], 'arm')
        self.assertEqual(schema['x-completion']['actions'], ['capture', 'move_to'])
        self.assertEqual(self.call('start')['state'], 'ready')
        self.assertFalse(self.call('select_action')['executed'])
        self.assertEqual(self.call('info', input_topic='')['state'], 'ready')
        self.assertEqual(self.sdk.calls, [])

    def test_decision_control_is_passive_offline_and_keeps_motion_permission_closed(self):
        self.runtime.motion_enabled = False
        self.runtime.disconnect()
        self.call('stop')
        self.runtime.cancel.set()
        previous_task = dict(self.runtime.task)
        with patch.object(self.runtime, 'connected', side_effect=AssertionError('must not connect')):
            with patch('agilex.piper.waypoint_control.adapter_serial', side_effect=AssertionError('must not probe CAN')):
                result = self.call('decision_control')
        self.assertEqual(result['state'], 'ready')
        self.assertEqual(result['control_mode'], 'decision_core')
        self.assertTrue(result['waiting_for_instruction'])
        self.assertTrue(result['manual_actions_available'])
        for flag in ('executed', 'hardware_verified', 'canvas_binding_verified', 'motion_enabled'):
            self.assertFalse(result[flag])
        self.assertTrue(self.runtime.cancel.is_set())
        self.assertFalse(self.runtime.motion_enabled)
        self.assertEqual(self.runtime.task, previous_task)
        self.assertEqual(self.call('info')['state'], 'ready')
        self.assertEqual(self.call('list')['waypoints'], [])
        self.assertEqual(self.sdk.calls, [])

    def test_decision_control_schema_and_busy_operations_preserve_state(self):
        schema = next(t for t in self.bundle.get_all_tools() if t['name'] == 'waypoints')['inputSchema']
        option = next(x for x in schema['properties']['action']['oneOf'] if x['const'] == 'decision_control')
        self.assertEqual(option['title'], '智能托管（decision_core）')
        self.assertEqual(schema['x-action-params']['decision_control']['params'], [])
        self.assertNotIn('decision_control', schema['x-completion']['actions'])
        for card, action in [('arm_control', 'set_joints'), ('waypoints', 'capture'), ('piper_photo', 'capture')]:
            with self.runtime.operation(card, action):
                self.runtime.cancel.set()
                before = (dict(self.runtime.task), set(self.runtime.stopped_cards), self.runtime.closed)
                with self.assertRaisesRegex(RuntimeError, 'busy'):
                    self.call('decision_control')
                self.assertTrue(self.runtime.cancel.is_set())
                self.assertTrue(self.runtime.gate.locked())
                self.assertEqual(before, (self.runtime.task, self.runtime.stopped_cards, self.runtime.closed))
        self.assertEqual(self.sdk.calls, [])

    def test_decision_control_does_not_bypass_move_permission_or_replace_manual_actions(self):
        self.save()
        self.runtime.motion_enabled = False
        self.call('decision_control')
        self.assertEqual(self.call('read', name='取料位')['name'], '取料位')
        with self.assertRaisesRegex(RuntimeError, 'disabled'):
            self.call('move_to', name='取料位')
        self.assertTrue(self.call('delete', name='取料位')['deleted'])
        self.assertEqual(self.sdk.calls, [])

    def test_capture_list_read_delete_with_motion_disabled_and_restart(self):
        self.runtime.motion_enabled = False
        point = self.save()['waypoint']
        self.assertEqual(point['joints_deg'], [1, 10, -10, 0, 20, 0])
        self.assertEqual(point['gripper']['width_mm'], 20)
        self.assertFalse(point['gripper_replayed'])
        self.assertEqual(self.call('read', name='取料位'), point)
        self.assertEqual(len(self.call('list')['waypoints']), 1)
        other = build_plugins(self.config, 'same', None)
        try:
            self.assertEqual(DriverBundle(other).dispatch('waypoints', {'action': 'read', 'name': '取料位'}), point)
        finally:
            for plugin in other: plugin.stop()
        self.assertTrue(self.call('delete', name='取料位')['deleted'])
        self.assertEqual(self.sdk.calls, [])

    def test_list_read_info_do_not_connect_or_require_identity(self):
        self.save()
        self.runtime.disconnect()
        with patch.object(self.runtime, 'connected', side_effect=AssertionError('must remain offline')):
            with patch('agilex.piper.waypoint_control.adapter_serial', return_value=None):
                self.assertEqual(len(self.call('list')['waypoints']), 1)
                self.assertEqual(self.call('read', name='取料位')['name'], '取料位')
                self.call('info')
                self.call('delete', name='取料位')
        self.assertEqual(self.sdk.calls, [])

    def test_capture_refuses_missing_changed_identity_movement_and_frozen_feedback(self):
        with patch('agilex.piper.waypoint_control.adapter_serial', return_value=None):
            with self.assertRaisesRegex(ValueError, 'identity'):
                self.save()
        with patch('agilex.piper.waypoint_control.adapter_serial', side_effect=['usb-1', 'usb-2']):
            with self.assertRaisesRegex(RuntimeError, 'changed'):
                self.save()
        state = self.driver.get_state()
        with patch.object(self.driver, 'get_state', return_value=state):
            with self.assertRaisesRegex(RuntimeError, 'did not advance'):
                self.save()
        original = self.driver.get_state
        count = [0]
        def moving():
            count[0] += 1
            current = original()
            return replace(current, joints_deg=(1+count[0], 10, -10, 0, 20, 0))
        with patch.object(self.driver, 'get_state', side_effect=moving):
            with self.assertRaisesRegex(RuntimeError, 'stationary'):
                self.save()
        self.assertFalse(self.runtime.waypoints.path.exists())
        self.assertEqual(self.sdk.calls, [])

    def test_capture_rejects_fault_partial_enable_stale_and_controller_motion(self):
        for field, value in [('error', 1), ('enabled', (True,)*5+(False,)), ('frozen', True), ('reached', False)]:
            previous = getattr(self.sdk, field)
            setattr(self.sdk, field, value)
            try:
                with self.subTest(field=field), self.assertRaises(RuntimeError):
                    self.save()
            finally:
                setattr(self.sdk, field, previous)
        self.assertEqual(self.sdk.calls, [])
        self.assertFalse(self.runtime.waypoints.path.exists())

    def test_capture_in_stationary_teaching_does_not_take_over(self):
        self.sdk.mode = 'TEACHING_MODE(0x2)'
        self.save()
        self.assertEqual(self.sdk.calls, [])

    def test_unavailable_gripper_observation_does_not_block_joint_capture(self):
        with patch.object(self.driver, 'get_gripper', side_effect=RuntimeError('stale')):
            self.assertIsNone(self.save()['waypoint']['gripper'])
        self.assertEqual(self.sdk.calls, [])

    def test_move_to_uses_saved_joints_and_preserves_gripper_and_reset(self):
        self.save()
        self.sdk.raw = (2000, 12000, -12000, 1000, 21000, 1000)
        self.sdk.width = 30
        result = self.call('move_to', name='取料位', speed_percent=2)
        self.assertEqual(result['state'], 'completed')
        self.assertEqual(result['target'], (1, 10, -10, 0, 20, 0))
        self.assertEqual(self.sdk.width, 30)
        self.assertFalse(any(c[0] in {'gripper', 'reset'} for c in self.sdk.calls))
        self.assertEqual(self.runtime.return_preset()['joints_deg'], [0]*6)
        self.assertFalse((Path(self.tmp.name)/'return-preset.json').exists())

    def test_move_rejects_foreign_connection_missing_point_disabled_permission_and_limits(self):
        self.save()
        for serial in ['usb-2', None]:
            with patch('agilex.piper.waypoint_control.adapter_serial', return_value=serial):
                with self.assertRaises(ValueError):
                    self.call('move_to', name='取料位')
        self.runtime.config['can_interface'] = 'can1'
        with self.assertRaisesRegex(ValueError, 'another arm'):
            self.call('move_to', name='取料位')
        self.runtime.config['can_interface'] = 'can0'
        with self.assertRaisesRegex(ValueError, 'does not exist'):
            self.call('move_to', name='missing')
        self.runtime.motion_enabled = False
        with self.assertRaisesRegex(RuntimeError, 'disabled'):
            self.call('move_to', name='取料位')
        self.runtime.motion_enabled = True
        bad = record(); bad['joints_deg'][0] = 151
        self.runtime.waypoints.capture('out', bad)
        with self.assertRaisesRegex(RuntimeError, 'within'):
            self.call('move_to', name='out')
        self.assertEqual(self.sdk.calls, [])

    def test_move_timeout_is_not_reported_as_arrival(self):
        self.save()
        self.sdk.raw = (2000,12000,-12000,0,20000,0)
        self.sdk.follow = False
        with self.assertRaises(RuntimeError):
            self.call('move_to', name='取料位')
        self.assertEqual(self.runtime.task['state'], 'failed')

    def test_schema_rejects_bad_names_speed_and_unknown_fields_before_commands(self):
        for action, args in [('capture', {'name': ''}), ('capture', {'name': 'x', 'overwrite': 'true'}),
                             ('move_to', {'name': 'x', 'speed_percent': True}),
                             ('move_to', {'name': 'x', 'speed_percent': 11}),
                             ('read', {'name': 'x', 'unexpected': 1})]:
            with self.subTest(action=action, args=args), self.assertRaises(ValueError):
                self.call(action, **args)
        self.assertEqual(self.sdk.calls, [])

    def test_busy_arm_and_waypoint_share_one_operation_gate(self):
        with self.runtime.operation('arm_control', 'set_joints'):
            before = dict(self.runtime.task)
            with self.assertRaises(RuntimeError):
                self.save()
            with self.assertRaises(RuntimeError):
                self.call('move_to', name='missing')
            self.assertEqual(self.runtime.task, before)
        with self.runtime.operation('waypoints', 'capture'):
            with self.assertRaises(RuntimeError):
                self.bundle.dispatch('arm_control', {'action': 'set_joints', 'joints_deg': [1,10,-10,0,20,0]})
        self.assertEqual(self.sdk.calls, [])

    def test_async_move_keeps_waypoint_identity_and_arm_stop_cancels(self):
        self.save()
        self.runtime.config['async_actions'] = True
        entered, release = threading.Event(), threading.Event()
        original = self.runtime._execute_task
        def delayed(action, args):
            entered.set()
            release.wait(2)
            return original(action, args)
        with patch.object(self.runtime, '_execute_task', side_effect=delayed):
            accepted = self.call('move_to', name='取料位')
            self.assertTrue(entered.wait(1))
            self.assertEqual(accepted['state'], 'running')
            self.assertEqual(self.runtime.active_card, 'waypoints')
            self.assertEqual(self.call('start')['state'], 'busy')
            with self.assertRaises(RuntimeError):
                self.call('delete', name='取料位')
            stopped = self.bundle.dispatch('arm_control', {'action': 'stop_motion'})
            self.assertTrue(stopped['operation_stopping'])
            release.set()
            self.runtime.completions.wait(3)
        completed = self.runtime.last_actions['waypoints']
        self.assertEqual(completed['status'], 'cancelled')
        self.assertEqual(self.runtime.task['state'], 'cancelled')
        self.assertEqual(completed['action_id'], accepted['action_id'])
        self.assertFalse(self.runtime.gate.locked())
        self.assertFalse(any(c[0] == 'joints' for c in self.sdk.calls))

    def test_stop_during_motion_holds_measured_joints_and_does_not_report_success(self):
        self.save()
        self.sdk.raw = (2000,12000,-12000,0,20000,0)
        self.sdk.follow = False
        clock = unified_tests.FastClock()
        def interrupt(seconds):
            self.call('stop_motion')
            clock.sleep(seconds)
        with patch('agilex.piper.unified_control.execute_target', side_effect=lambda *a, **kw:
                unified_tests.execute_target(*a, **kw, clock=clock.now, sleep=interrupt)):
            with self.assertRaisesRegex(RuntimeError, 'cancelled'):
                self.call('move_to', name='取料位')
        targets = [call[1] for call in self.sdk.calls if call[0] == 'joints']
        self.assertEqual(len(targets), 2)
        self.assertEqual(targets[-1], self.sdk.raw)
        self.assertEqual(self.runtime.task['state'], 'cancelled')
        self.assertFalse(self.runtime.gate.locked())

    def test_async_success_posts_waypoint_completion_once(self):
        self.save()
        self.runtime.config['async_actions'] = True
        original = self.runtime._execute_task
        def delayed(*args, **kwargs):
            time.sleep(.08)
            return original(*args, **kwargs)
        with patch.object(self.runtime, '_execute_task', side_effect=delayed):
            result = self.call('move_to', name='取料位')
            self.assertEqual(result['state'], 'running')
            self.runtime.completions.wait(3)
        completion = self.runtime.last_actions['waypoints']
        self.assertEqual(completion['status'], 'completed')
        self.assertEqual(completion['result']['waypoint']['name'], '取料位')
        self.runtime.acp_complete.assert_called_once()
        self.assertEqual(self.runtime.acp_complete.call_args.args[-1], 'waypoints')
        self.assertEqual(self.runtime.acp_complete.call_args.args[0], result['action_id'])

    def test_stop_during_slow_storage_does_not_wait_for_disk_or_replace_old_point(self):
        self.save()
        before = self.runtime.waypoints.path.read_bytes()
        self.runtime.config['async_actions'] = True
        flushing, release = threading.Event(), threading.Event()
        original = os.fsync
        def slow_flush(fd):
            original(fd)
            flushing.set()
            release.wait(2)
        with patch('agilex.piper.waypoints.os.fsync', side_effect=slow_flush):
            accepted = self.call('capture', name='取料位', overwrite=True)
            self.assertEqual(accepted['state'], 'running')
            self.assertTrue(flushing.wait(1))
            try:
                # This must finish while the writer is still blocked in fsync.
                self.assertTrue(self.call('stop')['operation_stopping'])
                self.assertFalse(release.is_set())
            finally:
                release.set()
            self.runtime.completions.wait(3)
        self.assertEqual(self.runtime.last_actions['waypoints']['status'], 'cancelled')
        self.assertEqual(self.runtime.waypoints.path.read_bytes(), before)
        self.assertEqual(list(Path(self.tmp.name).glob('.waypoints-*')), [])

    def test_stop_while_waiting_for_another_writer_cancels_before_lock_is_released(self):
        self.runtime.config['async_actions'] = True
        waiting = threading.Event()
        original = fcntl.flock
        def observe_lock(fd, flags):
            try:
                return original(fd, flags)
            except BlockingIOError:
                waiting.set()
                raise
        with (Path(self.tmp.name) / 'waypoints.lock').open('a') as owner:
            original(owner, fcntl.LOCK_EX)
            with patch('agilex.piper.waypoints.fcntl.flock', side_effect=observe_lock):
                result = self.save()
                self.assertEqual(result['state'], 'running')
                self.assertTrue(waiting.wait(1))
                self.assertTrue(self.call('stop')['operation_stopping'])
                self.runtime.completions.wait(3)
                self.assertFalse(self.runtime.gate.locked())
        self.assertEqual(self.runtime.last_actions['waypoints']['status'], 'cancelled')
        self.assertFalse(self.runtime.waypoints.path.exists())

    def test_posture_or_mode_change_during_write_preserves_old_waypoint(self):
        self.save()
        before = self.runtime.waypoints.path.read_bytes()
        original = os.fsync
        for field, value in [('raw', (2000,10000,-10000,0,20000,0)), ('mode', 'STANDBY(0x0)')]:
            prior = getattr(self.sdk, field)
            def change_after_write(fd):
                original(fd)
                setattr(self.sdk, field, value)
            try:
                with patch('agilex.piper.waypoints.os.fsync', side_effect=change_after_write):
                    with self.subTest(field=field), self.assertRaisesRegex(RuntimeError, 'changed before saving'):
                        self.call('capture', name='取料位', overwrite=True)
                self.assertEqual(self.runtime.waypoints.path.read_bytes(), before)
            finally:
                setattr(self.sdk, field, prior)
        self.assertEqual(self.sdk.calls, [])

    def test_stop_capture_cannot_save_and_start_does_not_clear_cancellation(self):
        self.runtime.config['async_actions'] = True
        entered, release = threading.Event(), threading.Event()
        def delayed(driver, **kwargs):
            entered.set(); release.wait(2)
            return fast_capture(driver, **kwargs)
        with patch('agilex.piper.waypoint_control.capture_position', side_effect=delayed):
            accepted = self.save()
            self.assertTrue(entered.wait(1))
            self.assertTrue(self.call('stop')['operation_stopping'])
            self.assertEqual(self.call('start')['state'], 'stopping')
            release.set()
            self.runtime.completions.wait(3)
        self.assertEqual(self.runtime.last_actions['waypoints']['status'], 'cancelled')
        self.assertFalse(self.runtime.waypoints.path.exists())
        self.assertEqual(self.call('start')['state'], 'ready')
        self.assertEqual(self.sdk.calls, [])


if __name__ == '__main__':
    unittest.main()
