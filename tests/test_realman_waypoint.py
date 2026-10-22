"""Offline waypoint tests: no vendor library, ROS or hardware connection."""
import copy
from concurrent.futures import ThreadPoolExecutor
import json
from pathlib import Path
import sys
import tempfile
import threading
import time
import unittest
from unittest.mock import patch

DRIVER = Path(__file__).resolve().parents[1] / 'realman/rm75_6f_v'
sys.path.insert(0, str(DRIVER))
from device import RM75Plugin, CartesianPlugin
from waypoint import WaypointPlugin, WaypointStore


class FakeSDK:
    connected = True
    motion_enabled = True

    def __init__(self):
        self.motion_gate = threading.Lock()
        self.joints = [1.] * 7
        self.commands = []
        self.fail = None
        self.robot = {'arm_model': 'RM_75', 'arm_dof': 7, 'force_type': '6FB'}
        self.frame = {'name': 'tool', 'pose': [0.] * 6, 'payload': 0., 'x': 0., 'y': 0., 'z': 0.}
        self.reach = True

    def call(self, method):
        if method == self.fail:
            raise RuntimeError('模拟读取失败')
        result = {
            'rm_get_robot_info': self.robot,
            'rm_get_current_tool_frame': self.frame,
            'rm_get_current_work_frame': self.frame,
            'rm_get_joint_degree': self.joints,
            'rm_get_current_arm_state': {'pose': [0.] * 6},
            'rm_get_joint_drive_min_pos': [-360.] * 7,
            'rm_get_joint_drive_max_pos': [360.] * 7,
            'rm_get_arm_all_state': {'joint_err_code': [0] * 7, 'joint_en_flag': [1] * 7, 'err': {'err': []}},
        }[method]
        return copy.deepcopy(result)

    def command(self, method, *args):
        self.commands.append((method, args))
        if self.fail == method:
            raise RuntimeError('模拟指令失败')
        if method == 'rm_movej' and self.reach:
            self.joints = list(args[0])


class WaypointTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.sdk = FakeSDK()
        self.arm = RM75Plugin(self.sdk, {})
        self.plugin = WaypointPlugin(self.sdk, {'safety': {'poll_interval_seconds': .005,
                                      'max_motion_seconds': .1}}, self.arm)
        self.plugin.store = WaypointStore(self.tmp.name)
        self.events = []
        self.callback = patch('device._acp_complete', side_effect=lambda *event: (self.events.append(event) or ('accepted', None)))
        self.callback.start()
        self.addCleanup(self.callback.stop)

    def save(self, **kw):
        return self.plugin.dispatch('save', {'name': '取料点', **kw})

    def move(self, **kw):
        return self.plugin.dispatch('move', {'name': '取料点', 'confirm_motion': True, **kw})

    def wait(self):
        deadline = time.monotonic() + 2
        while not self.events and time.monotonic() < deadline:
            time.sleep(.005)
        self.assertTrue(self.events)

    def test_lifecycle_read_operations_and_persistence_never_move(self):
        self.plugin.start()
        self.plugin.dispatch('start', {})
        record = self.save()['waypoint']
        self.assertEqual([1.] * 7, record['joint_degree'])
        self.assertEqual('6FB', record['robot']['force_type'])
        self.assertEqual(record, WaypointStore(self.tmp.name).get('取料点'))
        self.assertEqual(1, self.plugin.dispatch('list', {})['count'])
        self.plugin.dispatch('info', {})
        self.plugin.dispatch('cancel', {})
        self.plugin.stop()
        self.plugin.dispatch('delete', {'name': '取料点'})
        self.assertEqual([], WaypointStore(self.tmp.name).list())
        self.assertEqual([], self.sdk.commands)

    def test_duplicates_and_explicit_overwrite(self):
        self.save()
        self.sdk.joints = [2.] * 7
        for overwrite in (False, 'true', 1):
            with self.assertRaises(ValueError):
                self.save(overwrite=overwrite)
        self.save(overwrite=True)
        self.assertEqual([2.] * 7, self.plugin.store.get('取料点')['joint_degree'])

    def test_confirmation_limits_and_speed(self):
        self.save()
        for args in ({'confirm_motion': False}, {'confirm_motion': 'true'},
                     {'speed_percent': 11}, {'speed_percent': 1.5}):
            with self.assertRaises(ValueError):
                self.move(**args)
        self.sdk.joints = [999.] * 7
        self.save(overwrite=True)
        with self.assertRaises(ValueError):
            self.move()
        self.assertEqual([], self.sdk.commands)

    def test_move_acp_and_schema(self):
        self.save()
        result = self.move()
        self.assertEqual('running', result['state'])
        self.wait()
        self.assertEqual((result['action_id'], 'completed'), self.events[0][:2])
        self.assertEqual('waypoint', self.events[0][3])
        self.assertEqual('completed', self.plugin.dispatch('info', {})['last_completion']['status'])
        schema = self.plugin.get_tools()[0]['inputSchema']
        self.assertEqual(['move'], schema['x-completion']['actions'])
        self.assertEqual('cancel', schema['x-hooks']['on_interrupt_motion']['action'])

    def test_mutex_cancel_and_only_cancel_own_action(self):
        self.save()
        self.sdk.motion_gate.acquire()
        try:
            for action, args in [('move', {'name': '取料点', 'confirm_motion': True}), ('save', {'name': 'x'})]:
                with self.assertRaises(RuntimeError):
                    self.plugin.dispatch(action, args)
            self.plugin.dispatch('cancel', {})
            self.assertEqual([], self.sdk.commands)
        finally:
            self.sdk.motion_gate.release()
        self.sdk.reach = False
        self.sdk.joints = [0.] * 7
        self.move()
        with self.assertRaises(RuntimeError):
            self.arm._start_motion({'confirm_motion': True})
        self.plugin.dispatch('cancel', {})
        self.wait()
        self.assertEqual('cancelled', self.events[0][1])
        self.assertFalse(self.sdk.motion_gate.locked())
        self.assertIn(('rm_set_arm_slow_stop', ()), self.sdk.commands)

    def test_joint_stop_cancels_waypoint_and_cartesian_invalidates(self):
        CartesianPlugin(self.sdk, {}, arm_plugin=self.arm)
        self.save()
        with patch.object(self.arm, '_cartesian_pose_invalidator') as invalidate:
            self.sdk.reach = False
            self.sdk.joints = [0.] * 7
            self.move()
            invalidate.assert_called_once()
            self.arm._stop_motion()
            self.wait()
        self.assertEqual('cancelled', self.events[0][1])

    def test_configuration_and_read_failures_refuse_without_commands(self):
        self.save()
        original = copy.deepcopy(self.sdk.robot)
        for key, value in [('arm_dof', 6), ('arm_model', 'RM_65'), ('force_type', 'B')]:
            self.sdk.robot = {**original, key: value}
            with self.assertRaises(ValueError):
                self.move()
        self.sdk.robot = original
        self.sdk.frame['pose'][0] = .1
        with self.assertRaises(ValueError):
            self.move()
        self.sdk.frame['pose'][0] = 0.
        for method in ('rm_get_robot_info', 'rm_get_current_tool_frame',
                       'rm_get_current_work_frame', 'rm_get_joint_degree', 'rm_get_current_arm_state'):
            self.sdk.fail = method
            with self.assertRaises(RuntimeError):
                self.save(overwrite=True)
        self.assertEqual([], self.sdk.commands)

    def test_failures_in_monitor_have_error_acp(self):
        self.save()
        self.sdk.reach = False
        self.sdk.joints = [0.] * 7
        self.move()
        self.wait()
        self.assertEqual('error', self.events[0][1])
        self.assertFalse(self.sdk.motion_gate.locked())

    def test_invalid_names_missing_and_corrupt_files(self):
        for name in ('../x', '/tmp/a', '', 'a b', 'x\n', 'x' * 65):
            with self.assertRaises(ValueError):
                self.save(name=name)
        for action in ('delete', 'move'):
            with self.assertRaises(ValueError):
                self.plugin.dispatch(action, {'name': 'missing', 'confirm_motion': True})
        self.save()
        path = Path(self.tmp.name) / 'waypoints.json'
        for content in ('{', '{"version":2,"waypoints":{}}', '{"version":1,"waypoints":{"x":{}}}'):
            path.write_text(content)
            for operation in (self.plugin.store.list, lambda: self.save(overwrite=True), lambda: self.plugin.store.delete('x')):
                with self.assertRaisesRegex(ValueError, '损坏'):
                    operation()
            self.assertEqual(content, path.read_text())

    def test_atomic_failure_preserves_previous_file(self):
        self.save()
        with patch('waypoint.os.replace', side_effect=OSError('模拟磁盘错误')):
            with self.assertRaises(OSError):
                self.save(overwrite=True)
        self.assertEqual(1, len(self.plugin.store.list()))
        self.assertEqual([], list(Path(self.tmp.name).glob('.waypoints-*')))

    def test_concurrent_store_instances_serialize_updates(self):
        record = self.save()['waypoint']
        def save(index):
            WaypointStore(self.tmp.name).save({**record, 'name': f'p{index}'})
        with ThreadPoolExecutor(max_workers=8) as executor:
            list(executor.map(save, range(24)))
        self.assertEqual(25, len(self.plugin.store.list()))

    def test_invalid_measured_state_and_controller_limits_refuse(self):
        self.save()
        for joints in ([0.] * 6, [float('nan')] * 7):
            self.sdk.joints = joints
            with self.assertRaises((ValueError, RuntimeError)):
                self.move()
        self.sdk.joints = [0.] * 7
        original = self.sdk.call
        def invalid_limits(method):
            if method == 'rm_get_joint_drive_min_pos':
                return [float('nan')] * 7
            return original(method)
        with patch.object(self.sdk, 'call', side_effect=invalid_limits):
            with self.assertRaises(RuntimeError):
                self.move()
        self.assertEqual([], self.sdk.commands)

    def test_disabled_motion_and_submission_failure_release_gate(self):
        self.save()
        self.sdk.motion_enabled = False
        with self.assertRaises(PermissionError):
            self.move()
        self.sdk.motion_enabled = True
        self.sdk.fail = 'rm_movej'
        with self.assertRaises(RuntimeError):
            self.move()
        self.assertFalse(self.sdk.motion_gate.locked())
        self.assertIsNone(self.plugin.dispatch('info', {})['active_action_id'])

    def test_deployment_mount_is_dedicated_and_schema_has_no_path(self):
        directory = '/opt/phanthy-motus/data/waypoints/realman'
        self.assertIn(f'{directory}:{directory}:rw', (DRIVER / 'deploy/service.yml').read_text())
        self.assertIn(f'install -d -m 0700 {directory}', (DRIVER / 'Dockerfile').read_text())
        self.assertNotIn('path', self.plugin.get_tools()[0]['inputSchema']['properties'])


if __name__ == '__main__':
    unittest.main()
