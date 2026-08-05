from __future__ import annotations

import threading
import unittest

from safety_harness import SmartMotionProxy


class FakeProcess:
    def __init__(self):
        self.join_count = 0

    def join(self):
        self.join_count += 1


class SmartMotionExitMonitorTest(unittest.TestCase):
    def test_child_exit_always_issues_parent_side_stop(self):
        process = FakeProcess()
        stop_calls = []
        proxy = SmartMotionProxy.__new__(SmartMotionProxy)
        proxy._proc = process
        proxy._fallback_stop = lambda: stop_calls.append("StopMove")

        proxy._monitor_process_exit()

        self.assertEqual(process.join_count, 1)
        self.assertEqual(stop_calls, ["StopMove"])

    def test_stop_failure_does_not_crash_exit_monitor(self):
        proxy = SmartMotionProxy.__new__(SmartMotionProxy)
        proxy._proc = FakeProcess()

        def fail_stop():
            raise RuntimeError("rpc unavailable")

        proxy._fallback_stop = fail_stop
        proxy._monitor_process_exit()


class SmartMotionParentStopSequenceTest(unittest.TestCase):
    def make_proxy(self, fallback_stop):
        proxy = SmartMotionProxy.__new__(SmartMotionProxy)
        proxy._call_lock = threading.RLock()
        proxy._fallback_stop = fallback_stop
        return proxy

    def test_bind_brackets_parent_stop_with_child_confirmation(self):
        calls = []
        proxy = self.make_proxy(lambda: calls.append(("stop", {})) or 0)

        def fake_call(method, **kwargs):
            calls.append((method, kwargs))
            if method == "begin_velocity_proposal_stop_confirmation":
                return {
                    "confirmation_start": {
                        "monotonic": 12.0,
                        "unix_ms": 34,
                        "odometry_callback_count": 56,
                    }
                }
            return {"connected": True, "external": kwargs["external_stop_attempt"]}

        proxy._call = fake_call
        result = proxy.bind_velocity_proposal("/proposal", "nav-1")

        self.assertEqual(
            [name for name, _ in calls],
            ["begin_velocity_proposal_stop_confirmation", "stop", "bind_velocity_proposal"],
        )
        self.assertTrue(result["connected"])
        self.assertEqual(result["external"]["stop_move_ret"], 0)
        self.assertIsNone(result["external"]["stop_move_error"])
        self.assertEqual(
            result["external"]["confirmation_start"]["odometry_callback_count"],
            56,
        )

    def test_parent_stop_exception_is_forwarded_for_fail_closed_confirmation(self):
        def fail_stop():
            raise RuntimeError("parent rpc unavailable")

        proxy = self.make_proxy(fail_stop)

        def fake_call(method, **kwargs):
            if method == "begin_velocity_proposal_stop_confirmation":
                return {
                    "confirmation_start": {
                        "monotonic": 1.0,
                        "unix_ms": 2,
                        "odometry_callback_count": 3,
                    }
                }
            return kwargs["external_stop_attempt"]

        proxy._call = fake_call
        result = proxy.unbind_velocity_proposal("canvas_stop")

        self.assertIsNone(result["stop_move_ret"])
        self.assertEqual(result["stop_move_error"], "parent rpc unavailable")

    def test_missing_child_boundary_still_issues_parent_stop_and_fails_closed(self):
        stop_calls = []
        proxy = self.make_proxy(lambda: stop_calls.append("stop") or 0)
        proxy._call = lambda method, **kwargs: {"error": "child unavailable"}

        result = proxy.bind_velocity_proposal("/proposal", "nav-1")

        self.assertEqual(stop_calls, ["stop"])
        self.assertFalse(result["stop_confirmed"])
        self.assertEqual(result["stop_move_ret"], 0)
        self.assertEqual(result["error"], "child unavailable")


if __name__ == "__main__":
    unittest.main()
