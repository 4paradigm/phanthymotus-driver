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
    request_parent_apply_velocity_proposal,
    request_parent_get_fsm_id,
    request_parent_stop_velocity_proposal,
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


class SmartMotionCallRoutingTest(unittest.TestCase):
    class AliveProcess:
        def __init__(self):
            self.alive = True
            self.terminate_count = 0
            self.join_count = 0

        def is_alive(self):
            return self.alive

        def terminate(self):
            self.terminate_count += 1
            self.alive = False

        def join(self, timeout=None):
            self.join_count += 1

    def make_proxy(self, *, dispatch_results: bool):
        proxy = SmartMotionProxy.__new__(SmartMotionProxy)
        proxy._proc = self.AliveProcess()
        proxy._cmd_queue = queue.Queue()
        proxy._result_queue = queue.Queue()
        proxy._pending = {}
        proxy._dispatch_lock = threading.Lock()
        proxy._req_counter = 0
        proxy._req_lock = threading.Lock()
        if dispatch_results:
            proxy._dispatch_thread = threading.Thread(
                target=proxy._dispatch_results,
                daemon=True,
            )
            proxy._dispatch_thread.start()
        return proxy

    def test_concurrent_calls_receive_only_their_correlated_reply(self):
        proxy = self.make_proxy(dispatch_results=True)
        results = {}

        first_thread = threading.Thread(
            target=lambda: results.setdefault(
                "first", proxy._call("first", timeout=0.5)
            ),
            daemon=True,
        )
        second_thread = threading.Thread(
            target=lambda: results.setdefault(
                "second", proxy._call("second", timeout=0.5)
            ),
            daemon=True,
        )
        first_thread.start()
        second_thread.start()
        commands = {
            command["method"]: command
            for command in (
                proxy._cmd_queue.get(timeout=0.2),
                proxy._cmd_queue.get(timeout=0.2),
            )
        }

        proxy._result_queue.put({
            "_req_id": commands["second"]["_req_id"],
            "reply": "second-result",
        })
        proxy._result_queue.put({
            "_req_id": commands["first"]["_req_id"],
            "reply": "first-result",
        })
        first_thread.join(timeout=0.5)
        second_thread.join(timeout=0.5)

        self.assertFalse(first_thread.is_alive())
        self.assertFalse(second_thread.is_alive())
        self.assertEqual(results["first"], {"reply": "first-result"})
        self.assertEqual(results["second"], {"reply": "second-result"})

    def test_timeout_terminates_unresponsive_motion_owner(self):
        proxy = self.make_proxy(dispatch_results=False)

        result = proxy._call("hung", timeout=0.01)

        self.assertEqual(result, {"error": "SmartMotion subprocess timeout (hung)"})
        self.assertEqual(proxy._proc.terminate_count, 1)
        self.assertEqual(proxy._proc.join_count, 1)


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


class SmartMotionParentLocoSequenceTest(unittest.TestCase):
    def start_proxy(
        self,
        parent_apply_velocity,
        parent_get_fsm_id=lambda: (0, 500),
        fallback_stop=lambda: 0,
    ):
        proxy = SmartMotionProxy.__new__(SmartMotionProxy)
        proxy._parent_velocity_queue = queue.Queue()
        proxy._parent_velocity_result_queue = queue.Queue()
        proxy._parent_velocity_shutdown = threading.Event()
        proxy._parent_apply_velocity_proposal = parent_apply_velocity
        proxy._parent_get_fsm_id = parent_get_fsm_id
        proxy._fallback_stop = fallback_stop
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

    def test_child_request_executes_controlled_velocity_on_parent(self):
        calls = []
        def apply(vx, vy, vyaw, deadline, nav_id, sequence, request_id):
            calls.append(
                (vx, vy, vyaw, deadline, nav_id, sequence, request_id)
            )
            return {"ret": 0, "error": None, "applied": True}

        proxy = self.start_proxy(apply)
        deadline = time.monotonic() + 0.2
        try:
            result = request_parent_apply_velocity_proposal(
                proxy._parent_velocity_queue,
                proxy._parent_velocity_result_queue,
                request_id=7,
                vx=0.1,
                vy=0.02,
                vyaw=0.03,
                deadline_monotonic=deadline,
                nav_id="nav-1",
                sequence=3,
                timeout=0.2,
            )
        finally:
            self.stop_proxy(proxy)

        self.assertEqual(
            calls,
            [(0.1, 0.02, 0.03, deadline, "nav-1", 3, 7)],
        )
        self.assertEqual(result["request_id"], 7)
        self.assertEqual(result["ret"], 0)
        self.assertIsNone(result["error"])
        self.assertIsNotNone(result["completed_monotonic"])

    def test_parent_nonzero_return_is_correlated(self):
        proxy = self.start_proxy(
            lambda *_: {"ret": 3104, "error": None, "applied": False}
        )
        try:
            result = request_parent_apply_velocity_proposal(
                proxy._parent_velocity_queue,
                proxy._parent_velocity_result_queue,
                request_id=8,
                vx=0.1,
                vy=0.0,
                vyaw=0.0,
                deadline_monotonic=time.monotonic() + 0.2,
                nav_id="nav-1",
                sequence=4,
                timeout=0.2,
            )
        finally:
            self.stop_proxy(proxy)

        self.assertEqual(result["ret"], 3104)
        self.assertIsNone(result["error"])

    def test_runtime_stop_is_ordered_after_inflight_parent_apply(self):
        calls = []
        apply_started = threading.Event()
        release_apply = threading.Event()

        def apply(*_):
            calls.append("apply_started")
            apply_started.set()
            release_apply.wait(timeout=0.5)
            calls.append("apply_completed")
            return {"ret": 0, "error": None, "applied": True}

        def stop():
            calls.append("stop")
            return 0

        proxy = self.start_proxy(apply, fallback_stop=stop)
        try:
            proxy._parent_velocity_queue.put({
                "method": "apply_velocity_proposal",
                "request_id": 20,
                "vx": 0.1,
                "vy": 0.0,
                "vyaw": 0.0,
                "deadline_monotonic": time.monotonic() + 1.0,
                "nav_id": "nav-1",
                "sequence": 1,
            })
            self.assertTrue(apply_started.wait(timeout=0.2))
            proxy._parent_velocity_queue.put({
                "method": "stop_velocity_proposal",
                "request_id": 21,
            })
            time.sleep(0.01)
            self.assertEqual(calls, ["apply_started"])
            release_apply.set()
            first = proxy._parent_velocity_result_queue.get(timeout=0.5)
            second = proxy._parent_velocity_result_queue.get(timeout=0.5)
        finally:
            release_apply.set()
            self.stop_proxy(proxy)

        self.assertEqual(calls, ["apply_started", "apply_completed", "stop"])
        self.assertEqual(first["request_id"], 20)
        self.assertEqual(first["ret"], 0)
        self.assertEqual(second["request_id"], 21)
        self.assertEqual(second["ret"], 0)

    def test_runtime_parent_stop_failure_is_reported(self):
        proxy = self.start_proxy(
            lambda *_: {"ret": 0, "error": None, "applied": True},
            fallback_stop=lambda: 3104,
        )
        try:
            result = request_parent_stop_velocity_proposal(
                proxy._parent_velocity_queue,
                proxy._parent_velocity_result_queue,
                request_id=22,
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
            result = request_parent_apply_velocity_proposal(
                proxy._parent_velocity_queue,
                proxy._parent_velocity_result_queue,
                request_id=9,
                vx=0.1,
                vy=0.0,
                vyaw=0.0,
                deadline_monotonic=time.monotonic() + 0.2,
                nav_id="nav-1",
                sequence=5,
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

        result = request_parent_apply_velocity_proposal(
            request_queue,
            result_queue,
            request_id=10,
            vx=0.1,
            vy=0.0,
            vyaw=0.0,
            deadline_monotonic=time.monotonic() + 0.2,
            nav_id="nav-1",
            sequence=6,
            timeout=0.03,
        )

        self.assertLess(time.monotonic() - started, 0.15)
        self.assertIsNone(result["ret"])
        self.assertEqual(result["error"], "parent_velocity_proposal_timeout")

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
        result = request_parent_apply_velocity_proposal(
            request_queue,
            result_queue,
            request_id=11,
            vx=0.1,
            vy=0.0,
            vyaw=0.0,
            deadline_monotonic=time.monotonic() + 0.2,
            nav_id="nav-1",
            sequence=7,
            timeout=0.2,
        )
        responder.join(timeout=0.5)

        self.assertEqual(result["request_id"], 11)
        self.assertEqual(result["ret"], 3104)

    def test_child_request_gets_fsm_id_from_parent(self):
        proxy = self.start_proxy(
            lambda *_: {"ret": 0, "error": None, "applied": True},
            lambda: (0, 500),
        )
        try:
            result = request_parent_get_fsm_id(
                proxy._parent_velocity_queue,
                proxy._parent_velocity_result_queue,
                request_id=12,
                timeout=0.2,
            )
        finally:
            self.stop_proxy(proxy)

        self.assertEqual(result["request_id"], 12)
        self.assertEqual(result["ret"], 0)
        self.assertEqual(result["fsm_id"], 500)
        self.assertIsNone(result["error"])

    def test_parent_fsm_exception_is_returned_to_child(self):
        def fail():
            raise RuntimeError("parent FSM RPC unavailable")

        proxy = self.start_proxy(
            lambda *_: {"ret": 0, "error": None, "applied": True},
            fail,
        )
        try:
            result = request_parent_get_fsm_id(
                proxy._parent_velocity_queue,
                proxy._parent_velocity_result_queue,
                request_id=13,
                timeout=0.2,
            )
        finally:
            self.stop_proxy(proxy)

        self.assertIsNone(result["ret"])
        self.assertIsNone(result["fsm_id"])
        self.assertEqual(result["error"], "parent FSM RPC unavailable")

    def test_parent_fsm_wait_is_bounded(self):
        started = time.monotonic()
        result = request_parent_get_fsm_id(
            queue.Queue(),
            queue.Queue(),
            request_id=14,
            timeout=0.03,
        )

        self.assertLess(time.monotonic() - started, 0.15)
        self.assertIsNone(result["ret"])
        self.assertEqual(result["error"], "parent_get_fsm_id_timeout")

    def test_stale_set_velocity_reply_cannot_satisfy_fsm_request(self):
        request_queue = queue.Queue()
        result_queue = queue.Queue()
        result_queue.put({
            "method": "set_velocity",
            "request_id": 14,
            "ret": 0,
            "error": None,
        })

        def reply_current():
            time.sleep(0.01)
            result_queue.put({
                "method": "get_fsm_id",
                "request_id": 15,
                "ret": 0,
                "fsm_id": 801,
                "error": None,
            })

        responder = threading.Thread(target=reply_current, daemon=True)
        responder.start()
        result = request_parent_get_fsm_id(
            request_queue,
            result_queue,
            request_id=15,
            timeout=0.2,
        )
        responder.join(timeout=0.5)

        self.assertEqual(result["request_id"], 15)
        self.assertEqual(result["fsm_id"], 801)

    def test_proposal_execution_has_no_child_loco_setvelocity(self):
        source = inspect.getsource(_run_smart_motion_process)

        self.assertNotIn("proposal_loco_client", source)
        self.assertNotIn("fsm_loco_client", source)
        self.assertIn("apply_parent_velocity_proposal", source)
        self.assertIn("proposal_apply_loop", source)
        self.assertIn("query_parent_fsm_id", source)
        self.assertIn("stop_parent_velocity_proposal", source)
        self.assertEqual(
            source.count("= stop_parent_velocity_proposal()"),
            2,
        )
        self.assertIn("last_proposal_stop_result = dict(result)", source)
        self.assertIn('"last_proposal_stop": (', source)
        self.assertIn("last_proposal_stop_result = None", source)
        self.assertIn("parent_loco_rpc_lock", source)
        self.assertIn("proposal_ros_spin_loop", source)
        callback_source = source[
            source.index("def on_velocity_proposal"):
            source.index("def apply_safety_velocity")
        ]
        self.assertLess(
            callback_source.index("refresh_proposal_execution_deadline("),
            callback_source.index("proposal_apply_diagnostics.record_accepted()"),
        )
        self.assertIn('name="g1_loco_proposal_ros"', source)
        self.assertEqual(source.count("executor.spin_once"), 2)

    def test_main_wires_parent_rpc_proxy_loco_calls(self):
        main_source = Path(__file__).resolve().parents[1].joinpath(
            "main.py"
        ).read_text(encoding="utf-8")

        self.assertIn(
            "parent_apply_velocity_proposal=loco_client.ApplyVelocityProposal",
            main_source,
        )
        self.assertIn(
            "parent_get_fsm_id=loco_client.GetFsmId",
            main_source,
        )


if __name__ == "__main__":
    unittest.main()
