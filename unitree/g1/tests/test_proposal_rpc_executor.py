from __future__ import annotations

import queue
import threading
import unittest

from rpc_proxy import (
    PROPOSAL_CONTINUOUS_DURATION_SECONDS,
    ProposalRpcExecutor,
    RpcProxy,
)


class FakeClock:
    def __init__(self, monotonic=10.0, wall=1_800_000_000.0):
        self.monotonic_value = monotonic
        self.wall_value = wall

    def monotonic(self):
        return self.monotonic_value

    def wall(self):
        return self.wall_value

    def advance(self, seconds):
        self.monotonic_value += seconds
        self.wall_value += seconds


class FakeLoco:
    def __init__(self, clock, set_velocity_ret=0, set_velocity_delay=0.0):
        self.clock = clock
        self.set_velocity_ret = set_velocity_ret
        self.set_velocity_delay = set_velocity_delay
        self.velocity_calls = []
        self.stop_calls = 0

    def SetVelocity(self, vx, vy, vyaw, duration):
        self.velocity_calls.append((vx, vy, vyaw, duration))
        self.clock.advance(self.set_velocity_delay)
        return self.set_velocity_ret

    def StopMove(self):
        self.stop_calls += 1
        return 0


class ProposalRpcExecutorTest(unittest.TestCase):
    def make_executor(self, **loco_kwargs):
        clock = FakeClock()
        loco = FakeLoco(clock, **loco_kwargs)
        executor = ProposalRpcExecutor(
            loco,
            monotonic=clock.monotonic,
            wall_time=clock.wall,
        )
        return clock, loco, executor

    def apply(self, executor, deadline=10.25, sequence=1):
        return executor.apply(
            0.1,
            0.02,
            0.03,
            deadline_monotonic=deadline,
            nav_id="nav-1",
            sequence=sequence,
            request_id=7,
        )

    def test_live_proposal_uses_known_working_continuous_velocity_semantics(self):
        _, loco, executor = self.make_executor()

        result = self.apply(executor)

        self.assertTrue(result["applied"])
        self.assertEqual(result["ret"], 0)
        self.assertEqual(result["rpc_method"], "SetVelocity(continuous)")
        self.assertEqual(
            loco.velocity_calls,
            [(0.1, 0.02, 0.03, PROPOSAL_CONTINUOUS_DURATION_SECONDS)],
        )
        self.assertTrue(executor.active)
        self.assertEqual(executor.nav_id, "nav-1")
        self.assertEqual(executor.sequence, 1)

    def test_deadline_expiry_stops_and_retires_parent_lease(self):
        clock, loco, executor = self.make_executor()
        self.apply(executor)

        clock.advance(0.251)
        stopped = executor.expire_if_due()

        self.assertEqual(stopped["reason"], "proposal_ttl_expired")
        self.assertEqual(stopped["ret"], 0)
        self.assertEqual(loco.stop_calls, 1)
        self.assertFalse(executor.active)

    def test_command_expired_in_queue_is_never_applied(self):
        _, loco, executor = self.make_executor()

        result = self.apply(executor, deadline=9.99)

        self.assertFalse(result["applied"])
        self.assertEqual(result["error"], "proposal_ttl_expired_before_rpc")
        self.assertEqual(loco.velocity_calls, [])
        self.assertEqual(loco.stop_calls, 1)
        self.assertFalse(executor.active)

    def test_rpc_completion_after_deadline_is_stopped_fail_closed(self):
        _, loco, executor = self.make_executor(set_velocity_delay=0.3)

        result = self.apply(executor)

        self.assertEqual(result["ret"], 0)
        self.assertFalse(result["applied"])
        self.assertEqual(result["error"], "proposal_ttl_expired_after_rpc")
        self.assertEqual(loco.stop_calls, 1)
        self.assertFalse(executor.active)

    def test_nonzero_velocity_rpc_return_triggers_stop(self):
        _, loco, executor = self.make_executor(set_velocity_ret=3104)

        result = self.apply(executor)

        self.assertEqual(result["ret"], 3104)
        self.assertFalse(result["applied"])
        self.assertEqual(loco.stop_calls, 1)
        self.assertFalse(executor.active)

    def test_newer_proposal_refreshes_parent_deadline_and_identity(self):
        clock, loco, executor = self.make_executor()
        self.apply(executor, deadline=10.20, sequence=1)
        clock.advance(0.1)

        result = self.apply(executor, deadline=10.35, sequence=2)

        self.assertTrue(result["applied"])
        self.assertEqual(executor.deadline_monotonic, 10.35)
        self.assertEqual(executor.sequence, 2)
        self.assertEqual(len(loco.velocity_calls), 2)
        clock.advance(0.11)
        self.assertIsNone(executor.expire_if_due())


class RpcProxyCorrelationTest(unittest.TestCase):
    def make_proxy(self):
        proxy = RpcProxy.__new__(RpcProxy)
        proxy._lock = threading.Lock()
        proxy._request_id = 0
        proxy._motion_rpc_timeout = 0.01
        proxy._cmd_q = queue.Queue()
        proxy._result_q = queue.Queue()
        return proxy

    def test_proposal_timeout_replaces_worker_and_reports_unknown_execution(self):
        proxy = self.make_proxy()
        replacements = []
        proxy._replace_timed_out_worker = lambda: replacements.append(True)

        result = proxy.ApplyVelocityProposal(
            0.1,
            0.0,
            0.0,
            deadline_monotonic=10.25,
            nav_id="nav-1",
            sequence=1,
            request_id=7,
        )

        self.assertEqual(replacements, [True])
        self.assertEqual(result["error"], "parent_velocity_proposal_timeout")
        self.assertFalse(result["applied"])

    def test_late_timed_out_reply_cannot_satisfy_next_stop(self):
        proxy = self.make_proxy()
        first = proxy._call("SetVelocity", timeout=0.01)
        first_command = proxy._cmd_q.get_nowait()
        proxy._result_q.put({
            "request_id": first_command["request_id"],
            "result": {"stale": True},
        })

        def respond_to_stop():
            command = proxy._cmd_q.get(timeout=0.2)
            proxy._result_q.put({
                "request_id": command["request_id"],
                "result": 0,
            })

        responder = threading.Thread(target=respond_to_stop, daemon=True)
        responder.start()
        second = proxy._call("StopMove", timeout=0.2)
        responder.join(timeout=0.5)

        self.assertIsNone(first)
        self.assertEqual(second, 0)
        self.assertFalse(responder.is_alive())


if __name__ == "__main__":
    unittest.main()
