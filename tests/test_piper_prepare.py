"""Disconnected initialization tests; no SDK import or CAN connection."""
import math
import unittest
from unittest.mock import patch

from agilex.piper.arm import ArmState, PiperDriver


class FakeArm:
    def __init__(self):
        self.calls = []
        self.raw = (1000, -2000, -1000, 0, 30600, 0)
        self.target = self.raw
        self.mode = 'STANDBY(0x0)'
        self.enabled = (False,) * 6
        self.fail_after_enable = False

    def GetSDKJointLimitParam(self, name):
        limits = [(-150, 150), (0, 180), (-170, 0), (-100, 100), (-70, 70), (-120, 120)]
        return tuple(map(math.radians, limits[int(name[1:])-1]))

    def MotionCtrl_2(self, *args):
        self.calls.append(('mode', args))
        self.mode = 'CAN_CTRL(0x1)'

    def JointCtrl(self, *target):
        self.calls.append(('target', target))
        self.target = target
        if all(self.enabled):
            self.raw = target

    def EnablePiper(self):
        self.calls.append(('enable',))
        self.enabled = (True,) * 6
        self.raw = self.target


class Driver(PiperDriver):
    def __init__(self):
        super().__init__()
        self.arm = FakeArm()

    def _raw_state(self):
        a = self.arm
        error = 5 if a.fail_after_enable and any(a.enabled) else 0
        return a.raw, ArmState(tuple(x/1000 for x in a.raw), a.enabled,
            a.mode, 'NORMAL(0x0)' if not error else 'ERROR', 'DISABLED(0x0)', error), error


class PrepareTests(unittest.TestCase):
    def setUp(self):
        self.d = Driver()
        self.tick = 0

    def clock(self):
        self.tick += .05
        return self.tick

    def execute(self, **extra):
        with patch('agilex.piper.arm.time.sleep'), patch('agilex.piper.arm.time.monotonic', side_effect=self.clock):
            return self.d.prepare_control(execute=True, workspace_clear=True,
                allow_limit_adjustment=True, **extra)

    def test_preview_is_readonly_and_discloses_boundary_adjustment(self):
        plan = self.d.prepare_control()
        self.assertEqual(plan['boundary_adjustment_deg'], (0, 2, 0, 0, 0, 0))
        self.assertFalse(plan['executed'])
        self.assertEqual(self.d.arm.calls, [])

    def test_requires_clear_workspace(self):
        with self.assertRaises(RuntimeError):
            self.d.prepare_control(execute=True, allow_limit_adjustment=True)
        self.assertEqual(self.d.arm.calls, [])

    def test_boundary_adjustment_requires_opt_in(self):
        with self.assertRaises(RuntimeError):
            self.d.prepare_control(execute=True, workspace_clear=True)
        self.assertEqual(self.d.arm.calls, [])

    def test_rejects_teaching_even_with_flags(self):
        self.d.arm.mode = 'TEACHING_MODE(0x2)'
        with self.assertRaises(RuntimeError):
            self.execute()
        self.assertEqual(self.d.arm.calls, [])

    def test_rejects_partial_enable(self):
        self.d.arm.enabled = (True, False, False, False, False, False)
        with self.assertRaises(RuntimeError):
            self.execute()
        self.assertEqual(self.d.arm.calls, [])

    def test_rejects_large_boundary_error(self):
        self.d.arm.raw = (1000, -4000, -1000, 0, 30600, 0)
        with self.assertRaises(RuntimeError):
            self.execute()
        self.assertEqual(self.d.arm.calls, [])

    def test_preloads_pose_before_enabling_and_preserves_j5(self):
        result = self.execute()
        calls = self.d.arm.calls
        first_enable = next(i for i,c in enumerate(calls) if c[0] == 'enable')
        self.assertGreaterEqual(sum(c[0] == 'target' for c in calls[:first_enable]), 1)
        self.assertTrue(result['executed'])
        self.assertTrue(all(result['state']['enabled']))
        self.assertTrue(all(c[1][4] == 30600 for c in calls if c[0] == 'target'))

    def test_fault_after_enable_is_reported_not_success(self):
        self.d.arm.fail_after_enable = True
        with self.assertRaises(RuntimeError):
            self.execute()

    def ready_to_move(self):
        self.d.arm.raw = (1000, 0, -1000, 0, 30600, 0)
        self.d.arm.enabled = (True,) * 6
        self.d.arm.mode = 'CAN_CTRL(0x1)'

    def test_move_five_degrees_reaches_target_without_changing_other_joints(self):
        self.ready_to_move()
        with patch('agilex.piper.arm.time.sleep'), patch('agilex.piper.arm.time.monotonic', side_effect=self.clock):
            result = self.d.move_j1_to(6, 5, True)
        self.assertAlmostEqual(result.joints_deg[0], 6)
        self.assertEqual(result.joints_deg[1:], (0, -1, 0, 30.6, 0))
        self.assertTrue(all(result.enabled))

    def test_cancel_during_ramp_holds_measured_pose_and_does_not_finish_target(self):
        self.ready_to_move()
        def cancel_after_progress(_):
            if self.d.arm.raw[0] > 1200:
                self.d.cancel_motion()
        with patch('agilex.piper.arm.time.sleep', side_effect=cancel_after_progress):
            with self.assertRaisesRegex(RuntimeError, 'cancelled'):
                self.d.move_j1_to(6, 5, True)
        self.assertLess(self.d.arm.raw[0], 2000)
        self.assertTrue(all(self.d.arm.enabled))
        self.assertEqual(self.d.arm.calls[-1], ('target', self.d.arm.raw))

    def test_mode_loss_does_not_reassert_can_control(self):
        self.ready_to_move()
        def switch_to_teach(_):
            self.d.arm.mode = 'TEACHING_MODE(0x2)'
        with patch('agilex.piper.arm.time.sleep', side_effect=switch_to_teach):
            with self.assertRaises(RuntimeError):
                self.d.move_j1_to(6, 5, True)
        self.assertEqual(self.d.arm.mode, 'TEACHING_MODE(0x2)')
        self.assertEqual(sum(call[0] == 'mode' for call in self.d.arm.calls), 1)

    def test_invalid_target_and_large_step_emit_no_commands(self):
        self.ready_to_move()
        for value in (float('nan'), float('inf'), 20):
            with self.subTest(value=value), self.assertRaises(ValueError):
                self.d.move_j1_to(value, 5, True)
        self.assertEqual(self.d.arm.calls, [])


if __name__ == '__main__':
    unittest.main()
