"""Cartesian planning uses the installed SDK FK; CAN commands stay simulated."""
import math
from types import SimpleNamespace
import threading
import time
import unittest
from unittest.mock import patch

import numpy as np
from piper_sdk.kinematics.piper_fk import C_PiperForwardKinematics

from agilex.piper import cartesian
import test_piper_ten_cards as ten


class CartesianTests(unittest.TestCase):
    call = ten.TenCardTests.call
    connected = ten.TenCardTests.connected
    fast = ten.TenCardTests.fast

    def setUp(self):
        ten.TenCardTests.setUp(self)
        self.sdk = self.connected()
        self.sdk.raw = (20000, 60000, -90000, 0, 45000, 0)
        self.model = C_PiperForwardKinematics(dh_is_offset=0)
        self.pose_stale = False
        self.pose_shift = 0.
        def pose_message():
            values = self.model.CalFK([math.radians(v/1000) for v in self.sdk.raw])[-1]
            values[0] += self.pose_shift
            return SimpleNamespace(time_stamp=0 if self.pose_stale else time.time(),
                                   end_pose=SimpleNamespace(**dict(zip(
                                       ('X_axis','Y_axis','Z_axis','RX_axis','RY_axis','RZ_axis'),
                                       [round(v*1000) for v in values]))))
        self.sdk.GetArmEndPoseMsgs = pose_message

    def move(self, **args):
        return self.call('cartesian', 'move', **args)

    def test_cartesian_read_without_permission_or_commands(self):
        result = self.call('cartesian', 'read')
        self.assertTrue(result['fresh'])
        self.assertEqual(result['frame'], 'base')
        self.assertFalse(self.plugin.motion_enabled)
        self.assertEqual(self.sdk.calls, [])

    def test_all_axes_and_signs_keep_pose_and_limits_without_commands(self):
        initial = cartesian.read_pose(self.plugin.driver)['pose']
        for axis in 'xyz':
            for distance in (-5, 5):
                with self.subTest(axis=axis, distance=distance):
                    preview = self.move(axis=axis, distance_mm=distance)
                    goal = self.model.CalFK(list(np.radians(preview['target_deg'])))[-1]
                    expected = list(initial)
                    expected['xyz'.index(axis)] += distance
                    self.assertLess(np.linalg.norm(np.array(goal[:3])-expected[:3]), .1)
                    self.assertLess(np.linalg.norm(cartesian.rotation_error(goal, expected)), math.radians(.1))
                    self.assertLessEqual(preview['sampled_path_deviation_mm'], 2)
                    self.assertLessEqual(max(map(abs, preview['delta_deg'])), 12)
                    self.assertFalse(preview['executed'])
        self.assertEqual(self.sdk.calls, [])

    def test_execute_requires_permission_and_current_confirmation(self):
        preview = self.move()
        with self.assertRaises(RuntimeError):
            self.move(execute=True, preview_id=preview['preview_id'], workspace_clear=True, take_control=True)
        self.plugin.motion_enabled = True
        with self.assertRaises(ValueError):
            self.move(execute=True, preview_id=preview['preview_id'], workspace_clear=True)
        self.assertEqual(self.sdk.calls, [])

    def test_simulated_execution_reaches_frozen_goal_and_cannot_replay(self):
        self.plugin.motion_enabled = True
        preview = self.move(axis='z', distance_mm=5)
        result = self.fast(lambda: self.move(axis='z', distance_mm=5, execute=True,
                            preview_id=preview['preview_id'], workspace_clear=True, take_control=True))
        self.assertTrue(result['confirmed'])
        self.assertLess(result['position_error_mm'], .1)
        self.assertLess(result['orientation_error_deg'], .1)
        self.assertEqual(self.sdk.raw, tuple(round(q*1000) for q in preview['target_deg']))
        with self.assertRaises(RuntimeError):
            self.move(axis='z', distance_mm=5, execute=True, preview_id=preview['preview_id'],
                      workspace_clear=True, take_control=True)

    def test_changed_parameters_and_start_invalidate_preview(self):
        self.plugin.motion_enabled = True
        for args in ({'axis': 'x'}, {'distance_mm': 6}, {'speed_pct': 6}, {'speed_deg_s': 2}):
            preview = self.move()
            with self.assertRaises(RuntimeError):
                self.move(execute=True, preview_id=preview['preview_id'], workspace_clear=True, take_control=True, **args)
        preview = self.move()
        self.sdk.raw = (self.sdk.raw[0]+1000, *self.sdk.raw[1:])
        with self.assertRaises(RuntimeError):
            self.move(execute=True, preview_id=preview['preview_id'], workspace_clear=True, take_control=True)
        self.assertEqual(self.sdk.calls, [])

    def test_stale_pose_and_partial_frames_rejected(self):
        self.pose_stale = True
        with self.assertRaises(RuntimeError):
            self.move()
        self.pose_stale = False
        self.sdk.frame_lock = threading.Lock()
        self.sdk.frame_counts = {}
        self.sdk._arm_can = SimpleNamespace(tx_frames=[])
        self.sdk.frame_times = {0x2A2: time.time(), 0x2A3: 0., 0x2A4: time.time()}
        with self.assertRaises(RuntimeError):
            self.call('cartesian', 'read')
        self.assertEqual(self.sdk.calls, [])

    def test_model_mismatch_and_unreachable_goal_refused(self):
        self.pose_shift = 20
        with self.assertRaises(RuntimeError):
            self.move()
        self.pose_shift = 0
        self.plugin.driver.joint_limits_deg = [(v/1000-.01, v/1000+.01) for v in self.sdk.raw]
        with self.assertRaises(ValueError):
            self.move()
        self.assertEqual(self.sdk.calls, [])

    def test_invalid_parameters_before_hardware(self):
        for args in ({'distance_mm': 21}, {'distance_mm': 0}, {'distance_mm': float('nan')},
                     {'axis': 'tool'}, {'speed_pct': True}, {'execute': 'true'}):
            with self.subTest(args=args), self.assertRaises(ValueError):
                self.move(**args)
        self.assertEqual(self.sdk.calls, [])

    def test_shared_gate_and_stop_scope(self):
        with self.plugin.operation('piper_cartesian', 'move'):
            with self.assertRaises(RuntimeError):
                self.call('joint', 'move', target_deg=20)
            self.assertFalse(self.call('stop', 'halt', target='gripper')['operation_stopping'])
            self.assertFalse(self.plugin.cancel.is_set())
            self.assertTrue(self.call('stop', 'halt', target='arm')['operation_stopping'])
            self.assertTrue(self.plugin.driver.cancelled.is_set())

    def test_permission_close_cancels_cartesian_and_sensor_keeps_working(self):
        with self.plugin.operation('piper_cartesian', 'move'):
            self.assertTrue(self.call('permission', 'block')['operation_stopping'])
            self.assertTrue(self.plugin.cancel.is_set())
            self.assertTrue(self.plugin.driver.cancelled.is_set())

    def test_stop_lifecycle_revokes_manual_permission(self):
        self.plugin.motion_enabled = self.plugin.manual_motion_permission = True
        self.call('cartesian', 'stop')
        self.assertFalse(self.plugin.motion_enabled)
        self.assertFalse(self.plugin.manual_motion_permission)
        self.assertEqual(self.sdk.calls, [])

    def test_pose_error_monitor_cancels_before_first_command(self):
        self.plugin.motion_enabled = True
        preview = self.move()
        self.pose_shift = 20
        with self.assertRaises(RuntimeError):
            self.move(execute=True, preview_id=preview['preview_id'], workspace_clear=True, take_control=True)
        self.assertEqual(self.sdk.calls, [])

    def test_rotation_wrap_is_continuous(self):
        a, b = [0,0,0,0,0,179.99], [0,0,0,0,0,-179.99]
        self.assertLess(np.linalg.norm(cartesian.rotation_error(a,b)), math.radians(.03))

    def test_expired_preview_and_failed_read_invalidate_target(self):
        self.plugin.motion_enabled = True
        preview = self.move()
        self.plugin.previews.entries[preview['preview_id']].created = time.monotonic()-61
        with self.assertRaises(RuntimeError):
            self.move(execute=True, preview_id=preview['preview_id'], workspace_clear=True, take_control=True)
        self.move()
        self.pose_stale = True
        with self.assertRaises(RuntimeError):
            self.call('cartesian', 'read')
        self.assertFalse(self.plugin.previews.entries)
        self.assertEqual(self.sdk.calls, [])

    def test_pose_failure_during_motion_holds_without_releasing_torque(self):
        self.plugin.motion_enabled = True
        preview = self.move()
        original = self.sdk.JointCtrl
        def send(*target):
            original(*target)
            self.pose_shift = 20
        self.sdk.JointCtrl = send
        with self.assertRaises(RuntimeError):
            self.fast(lambda: self.move(execute=True, preview_id=preview['preview_id'],
                                       workspace_clear=True, take_control=True))
        self.assertTrue(all(self.sdk.enabled))
        self.assertNotEqual(self.sdk.raw, tuple(round(q*1000) for q in preview['target_deg']))
        self.assertTrue(any(call[0] == 'joints' for call in self.sdk.calls))

    def test_rank_deficient_kinematics_rejected(self):
        with self.assertRaisesRegex(ValueError, '奇异'):
            cartesian.solve_target(lambda q: [0.,0.,0.,0.,0.,0.], (0.,)*6,
                                   [1.,0.,0.,0.,0.,0.], [(-20.,20.)]*6)
