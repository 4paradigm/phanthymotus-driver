from __future__ import annotations

import unittest

from safety_harness import ProposalApplyDiagnostics
from velocity_proposal import VelocityProposalGate, ProposalLimits


class ProposalApplyDiagnosticsTest(unittest.TestCase):
    def test_counts_and_last_set_velocity_result_are_reported(self):
        diagnostics = ProposalApplyDiagnostics()
        diagnostics.record_received()
        diagnostics.record_accepted()
        diagnostics.record_set_velocity(
            {
                "request_id": 12,
                "ret": 0,
                "error": None,
                "completed_monotonic": 34.5,
                "completed_unix_ms": 56,
            },
            duration=0.2,
        )
        diagnostics.record_applied()

        self.assertEqual(
            diagnostics.snapshot(),
            {
                "received": 1,
                "accepted": 1,
                "rejected": 0,
                "applied": 1,
                "last_rejection_reason": None,
                "last_set_velocity_ret": 0,
                "last_set_velocity_error": None,
                "last_set_velocity_request_id": 12,
                "last_set_velocity_duration_ms": 200,
                "last_set_velocity_completed_monotonic": 34.5,
                "last_set_velocity_completed_unix_ms": 56,
            },
        )

    def test_rejection_and_apply_failure_remain_after_canvas_stop(self):
        diagnostics = ProposalApplyDiagnostics()
        diagnostics.record_received()
        diagnostics.record_accepted()
        diagnostics.record_set_velocity(
            {
                "request_id": 13,
                "ret": 3104,
                "error": None,
                "completed_monotonic": 70.0,
                "completed_unix_ms": 80,
            },
            duration=0.15,
        )
        diagnostics.record_rejected("set_velocity_failed")

        gate = VelocityProposalGate(ProposalLimits())
        gate.bind("/proposal", "nav-1")
        gate.unbind("canvas_stop")
        status = gate.snapshot(100.0)
        status["proposal_execution"] = diagnostics.snapshot()

        self.assertEqual(status["last_reason"], "canvas_stop")
        self.assertEqual(
            status["proposal_execution"]["last_rejection_reason"],
            "set_velocity_failed",
        )
        self.assertEqual(
            status["proposal_execution"]["last_set_velocity_ret"],
            3104,
        )
        self.assertEqual(status["proposal_execution"]["received"], 1)
        self.assertEqual(status["proposal_execution"]["accepted"], 1)
        self.assertEqual(status["proposal_execution"]["rejected"], 1)
        self.assertEqual(status["proposal_execution"]["applied"], 0)


if __name__ == "__main__":
    unittest.main()
