from __future__ import annotations

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


if __name__ == "__main__":
    unittest.main()
