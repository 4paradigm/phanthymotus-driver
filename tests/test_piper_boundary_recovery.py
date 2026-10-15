"""Boundary feedback regression with a recording SDK; no robot or network access.

Reset keeps the official zero target. A short observation window must not turn
out-of-range measured feedback into an accepted target or a successful arrival.
"""
import tempfile
import unittest
from unittest.mock import patch

from agilex.piper.device import build_plugins
from agilex.piper.intent_motion import checked_state, end_pose
from agilex.piper.user_motion import execute_target, send_target
from common.vendor_runtime import DriverBundle
from test_piper_intent import FastClock, IntentDriver


class BoundaryExecutionTests(unittest.TestCase):
    def setUp(self):
        self.driver = IntentDriver()
        self.sdk = self.driver.arm
        self.clock = FastClock()
        self.reports = []
        self.submitted = False

    def excursion(self, millidegrees=1, until=None, joint=3):
        """Report an excursion only after the real fake-SDK submission."""
        original_submit = self.sdk.JointCtrl
        original_state = self.driver._raw_state

        def submit(*target):
            original_submit(*target)
            self.submitted = True

        def raw_state():
            if self.submitted:
                outside = until is None or self.clock.now() < until
                raw = [0] * 6
                raw[joint - 1] = millidegrees if outside else 0
                self.sdk.raw = tuple(raw)
            return original_state()

        self.sdk.JointCtrl = submit
        self.driver._raw_state = raw_state

    def execute(self, **kwargs):
        return execute_target(
            self.driver, (0.,) * 6, 1, 'joints', require_permission=lambda: None,
            report=self.reports.append, clock=self.clock.now,
            sleep=self.clock.sleep, **kwargs)

    def assert_original_target_only(self):
        self.assertEqual(self.sdk.calls, [
            ('mode', 1, 1, 1, 0), ('joints', (0, 0, 0, 0, 0, 0))])

    def test_checked_state_still_rejects_one_millidegree_outside_limit(self):
        self.sdk.raw = (0, 0, 1, 0, 0, 0)
        with self.assertRaisesRegex(RuntimeError, 'Joint 3') as caught:
            checked_state(self.driver, ready=True)
        self.assertEqual(caught.exception.feedback['joints_deg'][2], .001)
        self.assertEqual(self.sdk.calls, [])

    def test_reset_observes_transient_boundary_feedback_then_stable_arrival(self):
        self.excursion(until=.2)
        result = self.execute(boundary_settling=True)
        self.assertTrue(result['executed'])
        self.assertEqual(result['target'], (0.,) * 6)
        self.assertEqual(result['actual'], (0.,) * 6)
        self.assertGreaterEqual(self.clock.now(), .7)
        self.assertLess(self.clock.now(), 1.5)
        self.assert_original_target_only()

    def test_persistent_small_excursion_never_becomes_success_or_hold_target(self):
        self.excursion()
        with self.assertRaisesRegex(RuntimeError, 'Joint 3|boundary|limit'):
            self.execute(boundary_settling=True)
        self.assertGreaterEqual(self.clock.now(), .5)
        self.assertLess(self.clock.now(), 1.)
        self.assertFalse(self.reports[-1]['measured_hold_sent'])
        self.assert_original_target_only()

    def test_reset_observation_also_handles_lower_boundary_without_clamping(self):
        self.excursion(millidegrees=-1, until=.2, joint=2)
        result = self.execute(boundary_settling=True)
        self.assertEqual(result['actual'], (0.,) * 6)
        self.assertGreaterEqual(self.clock.now(), .7)
        self.assert_original_target_only()

    def test_gross_excursion_fails_without_waiting_or_clamping(self):
        self.excursion(millidegrees=501)
        with self.assertRaisesRegex(RuntimeError, 'Joint 3|boundary|limit'):
            self.execute(boundary_settling=True)
        self.assertLess(self.clock.now(), .1)
        self.assertFalse(self.reports[-1]['measured_hold_sent'])
        self.assert_original_target_only()

    def test_ordinary_joint_action_does_not_inherit_reset_settling(self):
        self.excursion(until=.2)
        with self.assertRaisesRegex(RuntimeError, 'Joint 3'):
            self.execute()
        self.assertLess(self.clock.now(), .1)
        self.assert_original_target_only()

    def test_settling_flag_does_not_relax_an_interior_joint_target(self):
        self.excursion(until=.2)
        interior_target = (0., .1, -.1, 0., 0., 0.)
        with self.assertRaisesRegex(RuntimeError, 'Joint 3|boundary|limit'):
            execute_target(
                self.driver, interior_target, 1, 'joints',
                require_permission=lambda: None, report=self.reports.append,
                clock=self.clock.now, sleep=self.clock.sleep,
                boundary_settling=True)
        self.assertLess(self.clock.now(), .1)
        self.assertEqual(self.sdk.calls, [
            ('mode', 1, 1, 1, 0), ('joints', (0, 100, -100, 0, 0, 0))])

    def test_controller_rejection_during_boundary_observation_is_immediate(self):
        self.excursion()
        original_sleep = self.clock.sleep

        def reject(seconds):
            original_sleep(seconds)
            self.sdk.status = 'TARGET_POS_EXCEEDS_LIMIT(0x4)'

        with patch.object(self.clock, 'sleep', side_effect=reject):
            with self.assertRaisesRegex(RuntimeError, 'outside its limits'):
                self.execute(boundary_settling=True)
        self.assertLess(self.clock.now(), .2)
        self.assertFalse(self.reports[-1]['measured_hold_sent'])
        self.assert_original_target_only()

    def test_stale_feedback_during_boundary_observation_still_fails(self):
        self.excursion()
        original_sleep = self.clock.sleep

        def stale(seconds):
            original_sleep(seconds)
            self.sdk.frozen = True

        with patch.object(self.clock, 'sleep', side_effect=stale):
            with self.assertRaisesRegex(RuntimeError, 'stale'):
                self.execute(boundary_settling=True)
        self.assertLess(self.clock.now(), .2)
        self.assert_original_target_only()

    def test_mode_loss_during_boundary_observation_still_fails(self):
        self.excursion()
        original_sleep = self.clock.sleep

        def lose_mode(seconds):
            original_sleep(seconds)
            self.sdk.mode = 'STANDBY(0x0)'

        with patch.object(self.clock, 'sleep', side_effect=lose_mode):
            with self.assertRaisesRegex(RuntimeError, 'ready state|mode'):
                self.execute(boundary_settling=True)
        self.assertLess(self.clock.now(), .2)
        self.assert_original_target_only()

    def test_cancellation_during_boundary_observation_cannot_submit_hold(self):
        self.excursion()
        original_sleep = self.clock.sleep

        def cancel(seconds):
            original_sleep(seconds)
            self.driver.cancelled.set()

        with patch.object(self.clock, 'sleep', side_effect=cancel):
            with self.assertRaisesRegex(RuntimeError, 'cancelled'):
                self.execute(boundary_settling=True)
        self.assertLess(self.clock.now(), .2)
        self.assert_original_target_only()

    def test_permission_revocation_during_observation_cannot_submit_hold(self):
        self.excursion()

        def permission():
            if self.clock.now() > 0:
                raise RuntimeError('Motion permission revoked')

        with self.assertRaisesRegex(RuntimeError, 'permission revoked'):
            execute_target(
                self.driver, (0.,) * 6, 1, 'joints', require_permission=permission,
                report=self.reports.append, clock=self.clock.now,
                sleep=self.clock.sleep, boundary_settling=True)
        self.assertLess(self.clock.now(), .2)
        self.assert_original_target_only()

    def test_expired_boundary_excursion_never_sends_hold_if_next_sample_recovers(self):
        original_submit = self.sdk.JointCtrl
        original_state = self.driver._raw_state
        error_delivered = False

        def submit(*target):
            original_submit(*target)
            self.submitted = True

        def raw_state():
            nonlocal error_delivered
            if self.submitted:
                self.sdk.raw = (0, 0, 0 if error_delivered else 1, 0, 0, 0)
            snapshot = original_state()
            if self.submitted and self.clock.now() >= .5:
                error_delivered = True
            return snapshot

        self.sdk.JointCtrl = submit
        self.driver._raw_state = raw_state
        with self.assertRaisesRegex(RuntimeError, 'Joint 3|boundary|limit'):
            self.execute(boundary_settling=True)
        self.assertTrue(error_delivered)
        self.assertEqual(self.driver.get_state().joints_deg, (0.,) * 6)
        self.assertFalse(self.reports[-1]['measured_hold_sent'])
        self.assert_original_target_only()

    def test_controller_rejection_never_sends_hold_if_next_sample_is_healthy(self):
        original_submit = self.sdk.JointCtrl
        original_state = self.driver._raw_state

        def submit(*target):
            original_submit(*target)
            self.submitted = True
            self.sdk.status = 'TARGET_POS_EXCEEDS_LIMIT(0x4)'

        def raw_state():
            snapshot = original_state()
            if self.submitted:
                self.sdk.status = 'NORMAL(0x0)'
            return snapshot

        self.sdk.JointCtrl = submit
        self.driver._raw_state = raw_state
        with self.assertRaisesRegex(RuntimeError, 'outside its limits'):
            self.execute(boundary_settling=True)
        self.assertEqual(self.driver.get_state().arm_status, 'NORMAL(0x0)')
        self.assertFalse(self.reports[-1]['measured_hold_sent'])
        self.assertEqual(self.reports[-1]['failure_code'], 'target_rejected_after_submission')
        self.assert_original_target_only()

    def test_pose_submission_rechecks_boundary_under_send_lock(self):
        target = end_pose(self.driver)
        # Model the arm reaching an endpoint after the earlier UI/preflight
        # snapshot, before this primitive takes the submission lock.
        self.sdk.raw = (0,) * 6
        original_state = self.driver._raw_state
        observed_lock_states = []

        def raw_state():
            observed_lock_states.append(self.driver._send_lock._is_owned())
            return original_state()

        self.driver._raw_state = raw_state
        with self.assertRaisesRegex(RuntimeError, r'Cartesian.*joint limit.*Set joints'):
            send_target(self.driver, target, 1, 'pose', lambda: None)
        self.assertTrue(observed_lock_states)
        self.assertTrue(all(observed_lock_states))
        self.assertEqual(self.sdk.calls, [])

    def test_pose_boundary_reached_during_mode_submission_blocks_actual_target(self):
        target = end_pose(self.driver)
        original_mode = self.sdk.MotionCtrl_2

        def reach_boundary(*arguments):
            original_mode(*arguments)
            self.sdk.raw = (0,) * 6

        self.sdk.MotionCtrl_2 = reach_boundary
        with self.assertRaisesRegex(RuntimeError, r'Cartesian.*joint limit.*Set joints') as caught:
            send_target(self.driver, target, 1, 'pose', lambda: None)
        self.assertEqual(self.sdk.calls, [('mode', 1, 2, 1, 0)])
        self.assertIn('No Cartesian target was sent.', str(caught.exception))


class CartesianBoundaryPreflightTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        gate = patch.dict('os.environ', {'PIPER_MOTION_ENABLED': '1'})
        gate.start()
        self.addCleanup(gate.stop)
        self.plugins = build_plugins(
            {'data_dir': temporary.name, 'async_actions': False,
             'intent_control': {'tcp_verified': False}}, 'piper_arm1', None)
        self.runtime, self.background = self.plugins[:2]
        self.background.auto_connect = False
        self.runtime.factory = IntentDriver
        for plugin in self.plugins:
            self.addCleanup(plugin.stop)
        completion = patch.object(self.runtime, 'acp_complete', return_value=None)
        completion.start()
        self.addCleanup(completion.stop)
        self.driver = self.runtime.connected()
        self.sdk = self.driver.arm
        self.bundle = DriverBundle(self.plugins)

    def call(self, action, **arguments):
        return self.bundle.dispatch('arm_control', {'action': action, **arguments})

    def test_zero_pose_move_is_rejected_before_any_control_command(self):
        self.sdk.raw = (0,) * 6
        with self.assertRaisesRegex(RuntimeError, r'Cartesian.*joint limit.*Set joints'):
            self.call('move', translation_mm=[1, 0, 0])
        self.assertEqual(self.sdk.calls, [])

    def test_zero_pose_teaching_move_does_not_exit_teaching(self):
        self.sdk.raw = (0,) * 6
        self.sdk.mode = 'TEACHING_MODE(0x2)'
        with self.assertRaisesRegex(RuntimeError, r'Cartesian.*joint limit.*Set joints'):
            self.call('move', translation_mm=[1, 0, 0])
        self.assertEqual(self.sdk.calls, [])
        self.assertEqual(self.sdk.mode, 'TEACHING_MODE(0x2)')

    def test_zero_pose_disabled_move_does_not_prepare_or_enable(self):
        self.sdk.raw = (0,) * 6
        self.sdk.mode, self.sdk.enabled = 'STANDBY(0x0)', (False,) * 6
        with self.assertRaisesRegex(RuntimeError, r'Cartesian.*joint limit.*Set joints'):
            self.call('move', translation_mm=[1, 0, 0])
        self.assertEqual(self.sdk.calls, [])
        self.assertEqual(self.sdk.enabled, (False,) * 6)

    def test_zero_increment_remains_noop_at_zero_pose(self):
        self.sdk.raw = (0,) * 6
        result = self.call('move', translation_mm=[0, 0, 0])
        self.assertFalse(result['executed'])
        self.assertEqual(result['state'], 'completed')
        self.assertEqual(self.sdk.calls, [])

    def test_interior_move_still_uses_one_original_linear_target(self):
        clock = FastClock()

        def fast_execute(*args, **kwargs):
            return execute_target(*args, **kwargs, clock=clock.now, sleep=clock.sleep)

        with patch('agilex.piper.unified_control.execute_target', side_effect=fast_execute):
            result = self.call('move', translation_mm=[1, 0, 0])
        self.assertEqual(result['state'], 'completed')
        self.assertEqual(result['motion_mode'], 'linear')
        self.assertEqual([call for call in self.sdk.calls if call[0] == 'end_pose'],
                         [('end_pose', (200000, 1000, 250000, 180000, 0, 90000))])
        self.assertEqual(len(self.sdk.calls), 2)

    def test_existing_rejection_is_not_overridden_by_boundary_guidance(self):
        self.sdk.raw = (0,) * 6
        self.sdk.status = 'TARGET_POS_EXCEEDS_LIMIT(0x4)'
        with self.assertRaisesRegex(RuntimeError, 'still active'):
            self.call('move', translation_mm=[1, 0, 0])
        self.assertEqual(self.sdk.calls, [])

    def test_stop_cannot_claim_controller_stopped_while_fault_active(self):
        self.sdk.status = 'TARGET_POS_EXCEEDS_LIMIT(0x4)'
        result = self.call('stop_motion')
        self.assertEqual(result['state'], 'stop_unconfirmed')
        self.assertFalse(result['controller_idle_confirmed'])
        self.assertEqual(self.sdk.calls, [])

    def test_stop_cannot_claim_controller_stopped_while_motion_active(self):
        self.sdk.reached = False
        result = self.call('stop_motion')
        self.assertEqual(result['state'], 'stop_unconfirmed')
        self.assertFalse(result['controller_idle_confirmed'])
        self.assertEqual(self.sdk.calls, [])

    def test_idle_stop_remains_passive_and_confirms_fresh_controller_status(self):
        result = self.call('stop_motion')
        self.assertEqual(result['state'], 'stopped')
        self.assertTrue(result['controller_idle_confirmed'])
        self.assertEqual(self.sdk.calls, [])

    def test_preparation_drift_to_boundary_cancels_cartesian_target(self):
        self.sdk.raw = (1000, 10000, -100, 0, 20000, 0)
        self.sdk.mode, self.sdk.enabled = 'STANDBY(0x0)', (False,) * 6
        original_prepare = self.driver.prepare_control

        def drift_to_boundary(**arguments):
            result = original_prepare(**arguments)
            self.sdk.raw = (*self.sdk.raw[:2], 0, *self.sdk.raw[3:])
            return result

        self.driver.prepare_control = drift_to_boundary
        with self.assertRaisesRegex(RuntimeError, r'Cartesian.*joint limit.*Set joints') as caught:
            self.call('move', translation_mm=[1, 0, 0])
        self.assertTrue(any(call[0] == 'prepare' for call in self.sdk.calls))
        self.assertFalse(any(call[0] == 'end_pose' for call in self.sdk.calls))
        self.assertIn('No Cartesian target was sent.', str(caught.exception))
        self.assertNotIn('No control command was sent.', str(caught.exception))
        diagnostics = self.runtime.task['diagnostics']
        self.assertTrue(diagnostics['control_attempted'])
        self.assertFalse(diagnostics['target_attempted'])
        self.assertFalse(diagnostics['target_submitted'])

    def test_controller_already_moving_rejects_without_new_command(self):
        self.sdk.reached = False
        with self.assertRaisesRegex(RuntimeError, 'still reports motion'):
            self.call('move', translation_mm=[1, 0, 0])
        self.assertEqual(self.sdk.calls, [])
        diagnostics = self.runtime.task['diagnostics']
        self.assertEqual(diagnostics['failure_code'], 'controller_still_moving')
        self.assertFalse(diagnostics['control_attempted'])
        self.assertFalse(diagnostics['target_attempted'])


if __name__ == '__main__':
    unittest.main()
