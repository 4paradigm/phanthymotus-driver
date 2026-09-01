from __future__ import annotations

import math
import queue
import unittest

from safety_harness import (
    ProposalApplyDiagnostics,
    ProposalExecutionLease,
    authoritative_proposal_stop_reason,
    obstacle_observation_applies,
    percentile_ms,
    put_latest,
    proposal_decision_requires_physical_stop,
    translation_obstacle_heading,
    velocity_commands_differ,
)
from velocity_proposal import VelocityProposalGate, ProposalLimits


class ProposalApplyDiagnosticsTest(unittest.TestCase):
    def test_counts_and_last_set_velocity_result_are_reported(self):
        diagnostics = ProposalApplyDiagnostics()
        diagnostics.begin_session("nav-1")
        diagnostics.record_received(10.0)
        diagnostics.record_legal_proposal(10.1)
        diagnostics.record_nav2_heartbeat(10.2)
        diagnostics.record_accepted(10.3)
        diagnostics.record_set_velocity(
            {
                "request_id": 12,
                "rpc_method": "SetVelocity(continuous)",
                "request": {
                    "vx": 0.1,
                    "vy": 0.0,
                    "vyaw": 0.0,
                    "nav_id": "nav-1",
                    "sequence": 1,
                },
                "ret": 0,
                "error": None,
                "applied": True,
                "started_monotonic": 34.3,
                "completed_monotonic": 34.5,
                "completed_unix_ms": 56,
                "duration_ms": 200,
            },
            queued_monotonic=34.25,
        )
        diagnostics.record_applied(now=10.4)

        self.assertEqual(
            diagnostics.snapshot(),
            {
                "session_nav_id": "nav-1",
                "received": 1,
                "accepted": 1,
                "rejected": 0,
                "applied": 1,
                "coalesced": 0,
                "first_received_monotonic": 10.0,
                "last_received_monotonic": 10.0,
                "last_callback_received_monotonic": 10.0,
                "last_legal_proposal_monotonic": 10.1,
                "last_accepted_proposal_monotonic": 10.3,
                "last_applied_proposal_monotonic": 10.4,
                "last_nav2_heartbeat_monotonic": 10.2,
                "last_receive_gap_ms": None,
                "max_receive_gap_ms": None,
                "last_proposal_age_ms": None,
                "max_proposal_age_ms": None,
                "expired_on_arrival": 0,
                "watchdog_faults_by_reason": {},
                "first_rejection_reason": None,
                "last_rejection_reason": None,
                "rejections_by_reason": {},
                "last_set_velocity_rpc_method": "SetVelocity(continuous)",
                "last_set_velocity_request": {
                    "vx": 0.1,
                    "vy": 0.0,
                    "vyaw": 0.0,
                    "nav_id": "nav-1",
                    "sequence": 1,
                },
                "last_set_velocity_ret": 0,
                "last_set_velocity_error": None,
                "last_set_velocity_applied": True,
                "last_set_velocity_request_id": 12,
                "last_set_velocity_duration_ms": 200,
                "last_set_velocity_queue_delay_ms": 50,
                "last_set_velocity_end_to_end_ms": 250,
                "set_velocity_rpc_samples": 1,
                "set_velocity_rpc_p50_ms": 200,
                "set_velocity_rpc_p95_ms": 200,
                "set_velocity_rpc_p99_ms": 200,
                "set_velocity_rpc_max_ms": 200,
                "last_set_velocity_completed_monotonic": 34.5,
                "last_set_velocity_completed_unix_ms": 56,
                "last_set_velocity_stop_ret": None,
                "last_set_velocity_stop_error": None,
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
                "started_monotonic": 69.85,
                "completed_monotonic": 70.0,
                "completed_unix_ms": 80,
                "duration_ms": 150,
            },
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

    def test_rpc_percentiles_use_measured_result_duration(self):
        diagnostics = ProposalApplyDiagnostics()
        diagnostics.begin_session("nav-latency")
        for duration_ms in (10, 20, 30, 40, 500):
            diagnostics.record_set_velocity({"duration_ms": duration_ms})

        status = diagnostics.snapshot()

        self.assertEqual(status["set_velocity_rpc_samples"], 5)
        self.assertEqual(status["set_velocity_rpc_p50_ms"], 30)
        self.assertEqual(status["set_velocity_rpc_p95_ms"], 500)
        self.assertEqual(status["set_velocity_rpc_p99_ms"], 500)
        self.assertEqual(status["set_velocity_rpc_max_ms"], 500)
        self.assertEqual(percentile_ms([], 0.99), None)

    def test_latest_only_queue_coalesces_without_backlog(self):
        commands = queue.Queue(maxsize=1)

        self.assertFalse(put_latest(commands, {"sequence": 1}))
        self.assertTrue(put_latest(commands, {"sequence": 2}))
        self.assertTrue(put_latest(commands, {"sequence": 3}))

        self.assertEqual(commands.qsize(), 1)
        self.assertEqual(commands.get_nowait(), {"sequence": 3})

    def test_arrival_timing_and_watchdog_faults_are_reported(self):
        diagnostics = ProposalApplyDiagnostics()
        diagnostics.begin_session("nav-timing")
        diagnostics.record_received(10.0)
        diagnostics.record_received(10.12)
        diagnostics.record_proposal_arrival(
            {"issued_at_unix_ms": 1_000, "ttl_ms": 200},
            received_unix_ms=1_120,
        )
        diagnostics.record_proposal_arrival(
            {"issued_at_unix_ms": 2_000, "ttl_ms": 200},
            received_unix_ms=2_250,
        )
        diagnostics.record_watchdog_fault("proposal_ttl_expired")
        diagnostics.record_watchdog_fault("proposal_ttl_expired")

        status = diagnostics.snapshot()

        self.assertEqual(status["first_received_monotonic"], 10.0)
        self.assertEqual(status["last_received_monotonic"], 10.12)
        self.assertEqual(status["last_receive_gap_ms"], 120)
        self.assertEqual(status["max_receive_gap_ms"], 120)
        self.assertEqual(status["last_proposal_age_ms"], 250)
        self.assertEqual(status["max_proposal_age_ms"], 250)
        self.assertEqual(status["expired_on_arrival"], 1)
        self.assertEqual(
            status["watchdog_faults_by_reason"],
            {"proposal_ttl_expired": 2},
        )

    def test_first_rejection_and_breakdown_survive_cleanup_rejection(self):
        diagnostics = ProposalApplyDiagnostics()
        diagnostics.begin_session("nav-9")
        diagnostics.record_rejected("set_velocity_failed")
        diagnostics.record_rejected("stop_transition")
        diagnostics.record_rejected("stop_transition")

        status = diagnostics.snapshot()

        self.assertEqual(status["first_rejection_reason"], "set_velocity_failed")
        self.assertEqual(status["last_rejection_reason"], "stop_transition")
        self.assertEqual(
            status["rejections_by_reason"],
            {"set_velocity_failed": 1, "stop_transition": 2},
        )

    def test_new_lease_resets_counts_but_unbind_does_not(self):
        diagnostics = ProposalApplyDiagnostics()
        diagnostics.begin_session("nav-1")
        diagnostics.record_received()
        diagnostics.record_rejected("invalid_json")

        retained = diagnostics.snapshot()
        diagnostics.begin_session("nav-2")
        reset = diagnostics.snapshot()

        self.assertEqual(retained["session_nav_id"], "nav-1")
        self.assertEqual(retained["received"], 1)
        self.assertEqual(reset["session_nav_id"], "nav-2")
        self.assertEqual(reset["received"], 0)
        self.assertIsNone(reset["first_received_monotonic"])
        self.assertEqual(reset["watchdog_faults_by_reason"], {})
        self.assertIsNone(reset["first_rejection_reason"])

    def test_implicit_lease_session_starts_with_first_valid_proposal(self):
        diagnostics = ProposalApplyDiagnostics()
        diagnostics.begin_session(None)
        diagnostics.record_received(9.0)
        diagnostics.record_rejected("frame_mismatch")
        diagnostics.begin_session("nav-implicit")
        diagnostics.record_received(10.0)

        status = diagnostics.snapshot()

        self.assertEqual(status["session_nav_id"], "nav-implicit")
        self.assertEqual(status["received"], 1)
        self.assertEqual(status["first_received_monotonic"], 10.0)
        self.assertEqual(status["rejected"], 0)

    def test_applied_count_deduplicates_safety_reapply_by_proposal_identity(self):
        diagnostics = ProposalApplyDiagnostics()
        diagnostics.begin_session("nav-1")

        diagnostics.record_applied("nav-1", 4)
        diagnostics.record_applied("nav-1", 4)
        diagnostics.record_applied("nav-1", 5)

        self.assertEqual(diagnostics.snapshot()["applied"], 2)

    def test_new_session_can_apply_the_same_sequence_again(self):
        diagnostics = ProposalApplyDiagnostics()
        diagnostics.begin_session("nav-1")
        diagnostics.record_applied("nav-1", 1)

        diagnostics.begin_session("nav-2")
        diagnostics.record_applied("nav-2", 1)

        self.assertEqual(diagnostics.snapshot()["applied"], 1)


class ProposalExecutionLeaseTest(unittest.TestCase):
    def test_first_proposal_does_not_activate_before_rpc_arm(self):
        lease = ProposalExecutionLease()

        self.assertTrue(lease.renew_if_active(10.25))
        self.assertIsNone(lease.watchdog_snapshot())

    def test_valid_new_proposal_renews_active_deadline_without_shortening(self):
        lease = ProposalExecutionLease()
        generation = lease.arm(10.25)

        self.assertTrue(lease.renew_if_active(10.40))
        self.assertEqual(lease.watchdog_snapshot(), (generation, 10.40))
        self.assertTrue(lease.renew_if_active(10.30))
        self.assertEqual(lease.watchdog_snapshot(), (generation, 10.40))

    def test_watchdog_rechecks_deadline_after_concurrent_renewal(self):
        lease = ProposalExecutionLease()
        generation = lease.arm(10.25)
        observed = lease.watchdog_snapshot()
        lease.renew_if_active(10.50)

        tripped = lease.trip(
            observed[0],
            "proposal_ttl_expired",
            now=10.30,
        )

        self.assertFalse(tripped)
        self.assertEqual(lease.watchdog_snapshot(), (generation, 10.50))
        self.assertIsNone(lease.current_fault())

    def test_fault_cannot_be_renewed_or_cleared_by_later_apply(self):
        lease = ProposalExecutionLease()
        generation = lease.arm(10.25)

        self.assertTrue(
            lease.trip(generation, "proposal_ttl_expired", now=10.26)
        )
        self.assertFalse(lease.renew_if_active(10.50))
        self.assertIsNone(lease.arm(10.50))
        self.assertEqual(
            lease.current_fault(),
            (generation, "proposal_ttl_expired"),
        )

        lease.clear()
        self.assertTrue(lease.renew_if_active(10.75))
        self.assertIsNotNone(lease.arm(10.75))


class VelocityCommandDifferenceTest(unittest.TestCase):
    def test_identical_safety_velocity_is_a_noop(self):
        self.assertFalse(
            velocity_commands_differ(
                (0.15, 0.0, 0.0),
                (0.15, 0.0, 0.0),
            )
        )

    def test_deceleration_and_resume_are_changes(self):
        self.assertTrue(
            velocity_commands_differ(
                (0.15, 0.12, 0.35),
                (0.15, 0.10, 0.20),
            )
        )


class ObstacleRecoveryPolicyTest(unittest.TestCase):
    def test_pure_rotation_has_no_translation_obstacle_cone(self):
        command = {"vx": 0.0, "vy": 0.0, "vyaw": 0.2}

        self.assertIsNone(translation_obstacle_heading(command))
        self.assertFalse(
            obstacle_observation_applies(command, 0.0, math.radians(30))
        )

    def test_matching_translation_cone_remains_fail_closed(self):
        command = {"vx": 0.1, "vy": 0.0, "vyaw": 0.0}

        self.assertTrue(
            obstacle_observation_applies(command, 0.0, math.radians(30))
        )

    def test_stale_forward_cone_does_not_block_escape_direction(self):
        reverse = {"vx": -0.05, "vy": 0.0, "vyaw": 0.0}
        lateral = {"vx": 0.0, "vy": 0.1, "vyaw": 0.0}

        self.assertFalse(
            obstacle_observation_applies(reverse, 0.0, math.radians(30))
        )
        self.assertFalse(
            obstacle_observation_applies(lateral, 0.0, math.radians(30))
        )

    def test_watchdog_fault_wins_over_concurrent_obstacle_stop(self):
        self.assertEqual(
            authoritative_proposal_stop_reason(
                "obstacle",
                (4, "proposal_ttl_expired"),
            ),
            "proposal_ttl_expired",
        )
        self.assertEqual(
            authoritative_proposal_stop_reason("obstacle", None),
            "obstacle",
        )

    def test_recoverable_hold_drains_stale_samples_without_repeated_stop(self):
        self.assertFalse(
            proposal_decision_requires_physical_stop(
                "proposal_ttl_expired",
                has_proposal=True,
                proposal_motion_active=False,
                newly_disarmed=False,
                recoverable_stop_active=True,
            )
        )
        self.assertTrue(
            proposal_decision_requires_physical_stop(
                "proposal_ttl_expired",
                has_proposal=True,
                proposal_motion_active=True,
                newly_disarmed=True,
                recoverable_stop_active=False,
            )
        )

    def test_idle_bootstrap_rejection_does_not_issue_redundant_stop(self):
        self.assertFalse(
            proposal_decision_requires_physical_stop(
                "retired_nav_id_replay",
                has_proposal=False,
                proposal_motion_active=False,
                newly_disarmed=False,
                recoverable_stop_active=False,
            )
        )


if __name__ == "__main__":
    unittest.main()
