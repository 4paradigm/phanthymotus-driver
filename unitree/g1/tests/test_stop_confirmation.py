from __future__ import annotations

import threading
import time
import unittest

from safety_harness import (
    OdomStopMonitor,
    issue_stop_and_confirm,
    resolve_proposal_stop_timeouts,
)


class StopMoveConfirmationIntegrationTest(unittest.TestCase):
    def setUp(self):
        self.monitor = OdomStopMonitor()

    def confirm(self, stop_move, timeout=0.2):
        return issue_stop_and_confirm(
            stop_move=stop_move,
            monitor=self.monitor,
            timeout=timeout,
            max_age=0.5,
            linear_epsilon=0.03,
            yaw_epsilon=0.05,
        )

    def test_zero_frame_arriving_during_synchronous_stopmove_is_confirmed(self):
        def stop_move():
            self.monitor.record((0.0, 0.0, 0.0))
            return 0

        result = self.confirm(stop_move)

        self.assertTrue(result["stop_confirmed"])
        diagnostics = result["stop_confirmation"]
        self.assertEqual(diagnostics["stop_move_ret"], 0)
        self.assertIsNone(diagnostics["stop_move_error"])
        self.assertEqual(diagnostics["odometry_callbacks_since_confirmation"], 1)
        self.assertEqual(diagnostics["odometry_callbacks_during_stop_move"], 1)
        self.assertEqual(
            diagnostics["last_odometry_velocity"],
            {"x": 0.0, "y": 0.0, "yaw": 0.0},
        )

    def test_zero_frame_arriving_after_stopmove_waits_on_condition(self):
        callback_finished = threading.Event()

        def publish_delayed_zero():
            try:
                time.sleep(0.03)
                self.monitor.record((0.0, 0.0, 0.0))
            finally:
                callback_finished.set()

        publisher = threading.Thread(target=publish_delayed_zero, daemon=True)
        publisher.start()

        result = self.confirm(lambda: 0)

        self.assertTrue(callback_finished.wait(0.5))
        publisher.join(timeout=0.5)
        self.assertTrue(result["stop_confirmed"])
        self.assertFalse(result["stop_confirmation"]["confirmation_timed_out"])
        self.assertEqual(
            result["stop_confirmation"]["odometry_callbacks_during_stop_move"],
            0,
        )

    def test_nonzero_frame_fails_closed_and_reports_last_velocity(self):
        def publish_nonzero():
            time.sleep(0.02)
            self.monitor.record((0.04, 0.0, 0.0))

        publisher = threading.Thread(target=publish_nonzero, daemon=True)
        publisher.start()

        result = self.confirm(lambda: 0, timeout=0.08)

        publisher.join(timeout=0.5)
        self.assertFalse(result["stop_confirmed"])
        diagnostics = result["stop_confirmation"]
        self.assertTrue(diagnostics["confirmation_timed_out"])
        self.assertEqual(diagnostics["odometry_callbacks_since_confirmation"], 1)
        self.assertEqual(
            diagnostics["last_odometry_velocity"],
            {"x": 0.04, "y": 0.0, "yaw": 0.0},
        )

    def test_no_new_frame_times_out_with_callback_counts(self):
        self.monitor.record((0.0, 0.0, 0.0))

        result = self.confirm(lambda: 0, timeout=0.05)

        self.assertFalse(result["stop_confirmed"])
        diagnostics = result["stop_confirmation"]
        self.assertTrue(diagnostics["confirmation_timed_out"])
        self.assertEqual(diagnostics["odometry_callback_count"], 1)
        self.assertEqual(diagnostics["odometry_callbacks_since_confirmation"], 0)
        self.assertIsNotNone(diagnostics["confirmation_started_monotonic"])
        self.assertIsNotNone(diagnostics["confirmation_started_unix_ms"])
        self.assertIsNotNone(diagnostics["last_odometry_age_ms"])

    def test_stopmove_exception_is_reported_without_waiting(self):
        def stop_move():
            raise RuntimeError("rpc unavailable")

        started = time.monotonic()
        result = self.confirm(stop_move, timeout=0.2)

        self.assertLess(time.monotonic() - started, 0.1)
        self.assertFalse(result["stop_confirmed"])
        self.assertIsNone(result["ret"])
        self.assertEqual(
            result["stop_confirmation"]["stop_move_error"],
            "rpc unavailable",
        )

    def test_stop_rpc_timeout_has_margin_within_confirmation_budget(self):
        rpc_timeout, confirmation_timeout = resolve_proposal_stop_timeouts({
            "velocity_proposal_stop_rpc_timeout": 0.1,
            "velocity_proposal_stop_confirm_timeout": 0.5,
        })

        self.assertEqual(rpc_timeout, 0.2)
        self.assertEqual(confirmation_timeout, 0.5)

    def test_stop_rpc_timeout_never_exceeds_confirmation_budget(self):
        rpc_timeout, confirmation_timeout = resolve_proposal_stop_timeouts({
            "velocity_proposal_stop_rpc_timeout": 10.0,
            "velocity_proposal_stop_confirm_timeout": 0.4,
        })

        self.assertEqual(rpc_timeout, 0.4)
        self.assertEqual(confirmation_timeout, 0.4)


if __name__ == "__main__":
    unittest.main()
