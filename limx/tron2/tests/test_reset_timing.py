"""Shared reset timing, original targets and fixed wave jaw settings; no hardware."""
import math
import unittest
import test_pose as fixture
from reset import duration_for_change
from pose import MOTION_SCALES

HOME=fixture.HOME

class ResetTimingTests(unittest.IsolatedAsyncioTestCase):
    asyncSetUp=fixture.PoseTests.asyncSetUp
    asyncTearDown=fixture.PoseTests.asyncTearDown
    accept_feedback=fixture.PoseTests.accept_feedback
    fake_send=fixture.PoseTests.fake_send
    fast_arrival=fixture.PoseTests.fast_arrival
    fake_gripper_state=fixture.PoseTests.fake_gripper_state
    fake_gripper_send=fixture.PoseTests.fake_gripper_send

    def test_old_reset_cases_are_exactly_twenty_percent_faster(self):
        for delta,old in ((0,8),(39.9,8),(40,8),(41,9),(78.2,16),(120,24),(300,60)):
            self.assertAlmostEqual(old/duration_for_change(delta),1.2)
        self.assertAlmostEqual(duration_for_change(0),6.666666666666667)

    def test_invalid_change_is_rejected_before_duration_planning(self):
        for delta in (-1,math.nan,math.inf,True,None,'60'):
            with self.assertRaises(ValueError):duration_for_change(delta)

    def test_pose_and_arm_reset_share_duration_and_saved_home_without_rewriting(self):
        original=self.file.read_bytes()
        q=HOME.copy();q[2]=math.radians(6.36);q[9]=math.radians(-6.31)
        self.accept_feedback(q+[0,0])
        arm=self.adapter.call('arm_control',{'action':'reset'})
        for preset in ('salute','wave','welcome'):
            plan=self.adapter.pose.plan({'pose':preset});home=plan['frames'][0]
            self.assertEqual(home['kind'],'home');self.assertEqual(home['joint'],arm['target_q'])
            self.assertEqual(home['time'],arm['duration_seconds'])
            self.assertAlmostEqual(home['time'],16/1.2)
        self.assertEqual(original,self.file.read_bytes());self.reader.send_movej.assert_not_awaited()

    def test_explicit_arm_move_duration_and_gesture_speeds_are_not_rescaled(self):
        result=self.adapter.call('arm_control',{'action':'move','joint':HOME,'time':8})
        self.assertEqual(result['time'],8)
        self.assertEqual(MOTION_SCALES,{'salute':1.44,'wave':1.44,'welcome':1.2})

    def test_only_wave_has_one_right_gripper_cycle_at100_speed_and10_force(self):
        for preset in ('salute','wave','welcome'):
            plan=self.adapter.pose.plan({'pose':preset})
            steps=[f['gripper'] for f in plan['frames'] if f.get('kind')=='gripper']
            expected=[{'side':'right','opening_mm':mm,'speed':100,'force':10} for mm in (85,0)] if preset=='wave' else []
            self.assertEqual(steps,expected)
