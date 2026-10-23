"""Idle feedback gaps must recover without weakening active-motion faults."""
import time
import types
import unittest
from unittest import mock

import device
from test_arm_controls import _FakePublisher, _prime_arm_plugin


class IdleFeedbackTests(unittest.TestCase):
    def setUp(self):
        self.control = _prime_arm_plugin(_FakePublisher())
        self.control._lowstate_sub = mock.Mock()
        self.control._lowstate_sub.Read.return_value = None
        self.control._last_feedback_at = time.monotonic() - 2

    def resume_feedback(self):
        positions = list(self.control._current_q)
        positions[17] += 0.05
        sample = types.SimpleNamespace(motor_state=[
            types.SimpleNamespace(q=q, dq=0.0, state=0) for q in positions])
        self.control._lowstate_sub.Read.side_effect = [sample, None]
        self.control._poll_feedback()
        return positions

    def test_idle_gap_blocks_new_motion_and_info_without_latching(self):
        self.control._poll_feedback()
        self.assertIsNone(self.control._feedback_error)
        info = self.control.dispatch('info', {})
        self.assertFalse(info['lowstate_ready'])
        self.assertIn('waiting for fresh feedback', info['last_error'])
        result = self.control.dispatch('set_shoulder', {
            'confirm': True, 'pitch_deg': -30})
        self.assertEqual('LOWSTATE_UNAVAILABLE', result['code'])
        self.assertIsNone(self.control._feedback_error)
        self.assertFalse(self.control._active)
        self.assertFalse(self.control._publisher.commands)

    def test_fresh_idle_feedback_recovers_without_publishing_or_starting_old_move(self):
        self.control._poll_feedback()
        positions = self.resume_feedback()
        info = self.control.dispatch('info', {})
        self.assertTrue(info['lowstate_ready'])
        self.assertIsNone(info['last_error'])
        self.assertEqual(positions, self.control._current_q)
        self.assertFalse(self.control._target_q)
        self.assertFalse(self.control._active)
        self.assertEqual(0, info['writes'])
        # A subsequent explicitly confirmed request takes over the fresh pose.
        self.control._confirm_target_write = lambda generation: None
        result = self.control.dispatch('set_shoulder', {
            'confirm': True, 'pitch_deg': -30})
        self.assertTrue(result['success'], result)
        self.assertEqual(positions, self.control._hold_q)

    def test_active_or_streaming_gap_remains_latched_when_data_returns(self):
        for active, streaming in ((True, False), (False, True)):
            with self.subTest(active=active, streaming=streaming):
                self.setUp()
                self.control._active = active
                self.control._streaming = streaming
                self.control._poll_feedback()
                self.assertIn('feedback timed out', self.control._feedback_error)
                self.resume_feedback()
                self.assertEqual('LOWSTATE_UNAVAILABLE', self.control._ready_error()['code'])
                self.assertFalse(self.control._publisher.commands)

    def test_invalid_idle_motor_status_still_latches(self):
        sample = types.SimpleNamespace(motor_state=[
            types.SimpleNamespace(q=0.0, dq=0.0, state=64 if i == 17 else 0)
            for i in range(31)])
        self.control._lowstate_sub.Read.return_value = sample
        self.control._poll_feedback()
        self.assertIn('64', self.control._feedback_error)
        self.resume_feedback()
        self.assertEqual('LOWSTATE_UNAVAILABLE', self.control._ready_error()['code'])


if __name__ == '__main__':
    unittest.main()
