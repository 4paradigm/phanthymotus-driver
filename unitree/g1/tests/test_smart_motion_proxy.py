from __future__ import annotations

import inspect
from pathlib import Path
import queue
import threading
import time
import unittest

from safety_harness import (
    SmartMotionProxy,
    _run_smart_motion_process,
    request_parent_set_velocity,
)


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


class SmartMotionParentVelocitySequenceTest(unittest.TestCase):
    def start_proxy(self, parent_set_velocity):
        proxy = SmartMotionProxy.__new__(SmartMotionProxy)
        proxy._parent_velocity_queue = queue.Queue()
        proxy._parent_velocity_result_queue = queue.Queue()
        proxy._parent_velocity_shutdown = threading.Event()
        proxy._parent_set_velocity = parent_set_velocity
        proxy._parent_velocity_thread = threading.Thread(
            target=proxy._serve_parent_velocity_requests,
            daemon=True,
        )
        proxy._parent_velocity_thread.start()
        return proxy

    def stop_proxy(self, proxy):
        proxy._signal_parent_velocity_shutdown()
        proxy._parent_velocity_thread.join(timeout=0.5)
        self.assertFalse(proxy._parent_velocity_thread.is_alive())

    def test_child_request_executes_set_velocity_on_parent(self):
        calls = []
        proxy = self.start_proxy(
            lambda vx, vy, vyaw, duration: calls.append(
                (vx, vy, vyaw, duration)
            ) or 0
        )
        try:
            result = request_parent_set_velocity(
                proxy._parent_velocity_queue,
                proxy._parent_velocity_result_queue,
                request_id=7,
                vx=0.1,
                vy=0.02,
                vyaw=0.03,
                duration=0.2,
                timeout=0.2,
            )
        finally:
            self.stop_proxy(proxy)

        self.assertEqual(calls, [(0.1, 0.02, 0.03, 0.2)])
        self.assertEqual(result["request_id"], 7)
        self.assertEqual(result["ret"], 0)
        self.assertIsNone(result["error"])
        self.assertIsNotNone(result["completed_monotonic"])

    def test_parent_nonzero_return_is_correlated(self):
        proxy = self.start_proxy(lambda *_: 3104)
        try:
            result = request_parent_set_velocity(
                proxy._parent_velocity_queue,
                proxy._parent_velocity_result_queue,
                request_id=8,
                vx=0.1,
                vy=0.0,
                vyaw=0.0,
                duration=0.2,
                timeout=0.2,
            )
        finally:
            self.stop_proxy(proxy)

        self.assertEqual(result["ret"], 3104)
        self.assertIsNone(result["error"])

    def test_parent_exception_is_returned_to_child(self):
        def fail(*_):
            raise RuntimeError("parent rpc unavailable")

        proxy = self.start_proxy(fail)
        try:
            result = request_parent_set_velocity(
                proxy._parent_velocity_queue,
                proxy._parent_velocity_result_queue,
                request_id=9,
                vx=0.1,
                vy=0.0,
                vyaw=0.0,
                duration=0.2,
                timeout=0.2,
            )
        finally:
            self.stop_proxy(proxy)

        self.assertIsNone(result["ret"])
        self.assertEqual(result["error"], "parent rpc unavailable")

    def test_child_wait_is_bounded_when_parent_does_not_reply(self):
        request_queue = queue.Queue()
        result_queue = queue.Queue()
        started = time.monotonic()

        result = request_parent_set_velocity(
            request_queue,
            result_queue,
            request_id=10,
            vx=0.1,
            vy=0.0,
            vyaw=0.0,
            duration=0.2,
            timeout=0.03,
        )

        self.assertLess(time.monotonic() - started, 0.15)
        self.assertIsNone(result["ret"])
        self.assertEqual(result["error"], "parent_set_velocity_timeout")

    def test_stale_parent_reply_cannot_satisfy_new_request(self):
        request_queue = queue.Queue()
        result_queue = queue.Queue()
        result_queue.put({"request_id": 10, "ret": 0, "error": None})

        def reply_current():
            time.sleep(0.01)
            result_queue.put({
                "request_id": 11,
                "ret": 3104,
                "error": None,
            })

        responder = threading.Thread(target=reply_current, daemon=True)
        responder.start()
        result = request_parent_set_velocity(
            request_queue,
            result_queue,
            request_id=11,
            vx=0.1,
            vy=0.0,
            vyaw=0.0,
            duration=0.2,
            timeout=0.2,
        )
        responder.join(timeout=0.5)

        self.assertEqual(result["request_id"], 11)
        self.assertEqual(result["ret"], 3104)

    def test_proposal_execution_has_no_child_loco_setvelocity(self):
        source = inspect.getsource(_run_smart_motion_process)

        self.assertNotIn("proposal_loco_client", source)
        self.assertIn("apply_parent_set_velocity", source)

    def test_main_wires_parent_rpc_proxy_set_velocity(self):
        main_source = Path(__file__).resolve().parents[1].joinpath(
            "main.py"
        ).read_text(encoding="utf-8")

        self.assertIn(
            "parent_set_velocity=loco_client.SetVelocity",
            main_source,
        )


if __name__ == "__main__":
    unittest.main()
