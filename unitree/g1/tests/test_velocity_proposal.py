from __future__ import annotations

import math
import unittest

from velocity_proposal import (
    DEFAULT_VELOCITY_PROPOSAL_TOPIC,
    ProposalLimits,
    VelocityProposalGate,
    VelocityProposalValidationError,
    resolve_expected_nav_id,
    resolve_input_topic,
    resolve_proposal_limits,
    validate_velocity_proposal,
    velocity_proposal_port,
)


EXPECTED_TOPIC = "/ubuntu/navigation/nav2/velocity_proposal"


def proposal(**changes):
    value = {
        "schema": "phanthy.navigation.velocity_proposal.v1",
        "nav_id": "nav-001",
        "sequence": 1,
        "issued_at_unix_ms": 1_800_000_000_000,
        "ttl_ms": 200,
        "frame": "base_link",
        "shadow_only": True,
        "physical_execution": False,
        "nav_status": "navigating",
        "velocity": {"x": 0.10, "y": 0.02, "yaw": 0.15},
    }
    value.update(changes)
    return value


class TopicResolutionTest(unittest.TestCase):
    def test_n5_topic_is_not_namespace_dependent(self):
        self.assertEqual(DEFAULT_VELOCITY_PROPOSAL_TOPIC, EXPECTED_TOPIC)

    def test_port_matches_n5_contract(self):
        self.assertEqual(
            velocity_proposal_port(EXPECTED_TOPIC),
            {
                "port": "velocity_proposal",
                "topic": EXPECTED_TOPIC,
                "format": "data/json",
                "ros_type": "std_msgs/msg/String",
                "qos": "RELIABLE + KEEP_LAST(depth=10) + VOLATILE",
                "schema": "phanthy.navigation.velocity_proposal.v1",
            },
        )

    def test_accepts_single_input_topic(self):
        self.assertEqual(
            resolve_input_topic({"input_topic": EXPECTED_TOPIC}, EXPECTED_TOPIC),
            EXPECTED_TOPIC,
        )

    def test_accepts_single_input_topics_entry(self):
        self.assertEqual(
            resolve_input_topic({"input_topics": [EXPECTED_TOPIC]}, EXPECTED_TOPIC),
            EXPECTED_TOPIC,
        )

    def test_rejects_empty_multiple_or_unexpected_topics(self):
        for args in (
            {},
            {"input_topics": []},
            {"input_topics": [EXPECTED_TOPIC, ""]},
            {"input_topics": [EXPECTED_TOPIC, "/other"]},
            {
                "input_topic": EXPECTED_TOPIC,
                "input_topics": [EXPECTED_TOPIC],
            },
            {"input_topic": "/ubuntu/navigation/nav2/cmd_vel_shadow"},
        ):
            with self.subTest(args=args), self.assertRaises(ValueError):
                resolve_input_topic(args, EXPECTED_TOPIC)

    def test_resolves_trusted_expected_nav_id_from_control_plane(self):
        self.assertEqual(
            resolve_expected_nav_id({"expected_nav_id": " nav-001 "}),
            "nav-001",
        )

    def test_rejects_missing_or_invalid_expected_nav_id(self):
        for args in (
            {},
            {"expected_nav_id": ""},
            {"expected_nav_id": 123},
            {"expected_nav_id": "x" * 129},
        ):
            with self.subTest(args=args), self.assertRaises(ValueError):
                resolve_expected_nav_id(args)


class ProposalValidationTest(unittest.TestCase):
    def setUp(self):
        self.limits = ProposalLimits()

    def test_valid_proposal(self):
        result = validate_velocity_proposal(proposal(), self.limits)
        self.assertEqual(result.nav_id, "nav-001")
        self.assertEqual(result.ttl_ms, 200)
        self.assertFalse(result.is_zero)

    def test_accepts_declared_loco_component_endpoints(self):
        velocities = (
            {"x": 1.0, "y": 0.0, "yaw": 0.0},
            {"x": -1.0, "y": 0.0, "yaw": 0.0},
            {"x": 0.0, "y": 1.0, "yaw": 0.0},
            {"x": 0.0, "y": -1.0, "yaw": 0.0},
            {"x": 1.0, "y": 1.0, "yaw": 0.0},
            {"x": 0.0, "y": 0.0, "yaw": 2.0},
            {"x": 0.0, "y": 0.0, "yaw": -2.0},
        )
        for velocity in velocities:
            with self.subTest(velocity=velocity):
                validate_velocity_proposal(
                    proposal(velocity=velocity),
                    self.limits,
                )

    def test_resolves_config_within_driver_envelope(self):
        limits = resolve_proposal_limits({})
        self.assertEqual(limits.min_x, -1.0)
        self.assertEqual(limits.max_x, 1.0)
        self.assertEqual(limits.max_abs_y, 1.0)
        self.assertEqual(limits.max_abs_yaw, 2.0)
        self.assertAlmostEqual(limits.max_planar_speed, math.sqrt(2.0))

        loosened = resolve_proposal_limits(
            {
                "velocity_proposal_min_x": -2.0,
                "velocity_proposal_max_x": 2.0,
                "velocity_proposal_max_abs_y": 2.0,
                "velocity_proposal_max_abs_yaw": 3.0,
                "velocity_proposal_max_planar_speed": 2.0,
            }
        )
        self.assertEqual(loosened.min_x, -1.0)
        self.assertEqual(loosened.max_x, 1.0)
        self.assertEqual(loosened.max_abs_y, 1.0)
        self.assertEqual(loosened.max_abs_yaw, 2.0)
        self.assertAlmostEqual(loosened.max_planar_speed, math.sqrt(2.0))

    def test_rejects_wrong_schema_frame_or_flags(self):
        cases = (
            {"schema": "other"},
            {"frame": "map"},
            {"shadow_only": False},
            {"physical_execution": True},
        )
        for changes in cases:
            with self.subTest(changes=changes), self.assertRaises(VelocityProposalValidationError):
                validate_velocity_proposal(proposal(**changes), self.limits)

    def test_rejects_invalid_identity_timing_and_status_fields(self):
        cases = (
            {"nav_id": ""},
            {"sequence": True},
            {"issued_at_unix_ms": math.inf},
            {"ttl_ms": True},
            {"nav_status": "unknown"},
            {"nav_status": None, "status": "navigating"},
            {"nav_status": "navigating", "status": "navigating"},
        )
        for changes in cases:
            with self.subTest(changes=changes), self.assertRaises(VelocityProposalValidationError):
                validate_velocity_proposal(proposal(**changes), self.limits)

    def test_rejects_nonfinite_and_each_speed_limit(self):
        velocities = (
            {"x": math.nan, "y": 0.0, "yaw": 0.0},
            {"x": 1.001, "y": 0.0, "yaw": 0.0},
            {"x": -1.001, "y": 0.0, "yaw": 0.0},
            {"x": 0.0, "y": 1.001, "yaw": 0.0},
            {"x": 0.0, "y": 0.0, "yaw": 2.001},
        )
        for velocity in velocities:
            with self.subTest(velocity=velocity), self.assertRaises(VelocityProposalValidationError):
                validate_velocity_proposal(proposal(velocity=velocity), self.limits)

    def test_rejects_tightened_planar_speed(self):
        limits = ProposalLimits(max_planar_speed=1.0)
        with self.assertRaises(VelocityProposalValidationError):
            validate_velocity_proposal(
                proposal(velocity={"x": 0.8, "y": 0.8, "yaw": 0.0}),
                limits,
            )

    def test_rejects_ttl_above_250_ms(self):
        with self.assertRaises(VelocityProposalValidationError):
            validate_velocity_proposal(proposal(ttl_ms=251), self.limits)

    def test_terminal_status_requires_zero_velocity(self):
        with self.assertRaises(VelocityProposalValidationError):
            validate_velocity_proposal(proposal(nav_status="arrived"), self.limits)
        result = validate_velocity_proposal(
            proposal(nav_status="arrived", velocity={"x": 0.0, "y": 0.0, "yaw": 0.0}),
            self.limits,
        )
        self.assertTrue(result.is_terminal)
        self.assertTrue(result.is_zero)


class ProposalGateTest(unittest.TestCase):
    def setUp(self):
        self.gate = VelocityProposalGate(ProposalLimits())
        self.gate.bind(EXPECTED_TOPIC, "nav-001")

    def test_missing_expected_nav_id_waits_without_arming(self):
        gate = VelocityProposalGate(ProposalLimits())
        gate.bind(EXPECTED_TOPIC)

        status = gate.snapshot(10.0)
        self.assertTrue(status["connected"])
        self.assertFalse(status["armed"])
        self.assertTrue(status["waiting_for_nav_id"])
        self.assertEqual(status["binding_mode"], "first_valid_proposal")
        self.assertIsNone(status["expected_nav_id"])
        self.assertIsNone(status["auto_bound_nav_id"])

    def test_first_fresh_valid_proposal_atomically_claims_lease(self):
        gate = VelocityProposalGate(ProposalLimits())
        gate.bind(EXPECTED_TOPIC)

        accepted = gate.accept(
            proposal(issued_at_unix_ms=1_000, ttl_ms=200),
            now=50.0,
            now_unix_ms=1_100,
        )

        self.assertTrue(accepted.execute)
        status = gate.snapshot(50.0)
        self.assertTrue(status["armed"])
        self.assertFalse(status["waiting_for_nav_id"])
        self.assertEqual(status["expected_nav_id"], "nav-001")
        self.assertEqual(status["active_nav_id"], "nav-001")
        self.assertEqual(status["auto_bound_nav_id"], "nav-001")
        self.assertEqual(status["binding_mode"], "first_valid_proposal")

    def test_invalid_or_expired_proposal_cannot_claim_waiting_lease(self):
        gate = VelocityProposalGate(ProposalLimits())
        gate.bind(EXPECTED_TOPIC)

        invalid = gate.accept(proposal(frame="map"), now=50.0)
        expired = gate.accept(
            proposal(issued_at_unix_ms=1_000, ttl_ms=200),
            now=50.1,
            now_unix_ms=1_201,
        )

        self.assertFalse(invalid.execute)
        self.assertFalse(invalid.stop)
        self.assertEqual(invalid.reason, "frame_mismatch")
        self.assertFalse(expired.execute)
        self.assertFalse(expired.stop)
        self.assertEqual(expired.reason, "proposal_ttl_expired")
        self.assertTrue(gate.waiting_for_nav_id)
        self.assertFalse(gate.armed)
        self.assertEqual(gate.expected_nav_id, "")

        fresh = gate.accept(
            proposal(nav_id="nav-002", issued_at_unix_ms=1_300),
            now=50.2,
            now_unix_ms=1_301,
        )
        self.assertTrue(fresh.execute)
        self.assertEqual(gate.expected_nav_id, "nav-002")

    def test_zero_or_terminal_proposal_cannot_claim_waiting_lease(self):
        gate = VelocityProposalGate(ProposalLimits())
        gate.bind(EXPECTED_TOPIC)

        zero = gate.accept(
            proposal(
                nav_id="nav-old",
                sequence=10,
                velocity={"x": 0.0, "y": 0.0, "yaw": 0.0},
            ),
            now=50.0,
        )
        terminal = gate.accept(
            proposal(
                nav_id="nav-old",
                sequence=11,
                nav_status="arrived",
                velocity={"x": 0.0, "y": 0.0, "yaw": 0.0},
            ),
            now=50.1,
        )

        for decision in (zero, terminal):
            self.assertFalse(decision.execute)
            self.assertFalse(decision.stop)
            self.assertEqual(decision.reason, "waiting_for_active_proposal")
        self.assertTrue(gate.waiting_for_nav_id)
        self.assertFalse(gate.armed)
        self.assertEqual(gate.expected_nav_id, "")

        active = gate.accept(
            proposal(nav_id="nav-002", sequence=1),
            now=50.2,
        )
        self.assertTrue(active.execute)
        self.assertEqual(gate.expected_nav_id, "nav-002")

    def test_rotation_only_proposal_can_claim_waiting_lease(self):
        gate = VelocityProposalGate(ProposalLimits())
        gate.bind(EXPECTED_TOPIC)

        rotation = gate.accept(
            proposal(
                nav_id="nav-rotate",
                velocity={"x": 0.0, "y": 0.0, "yaw": 0.25},
            ),
            now=50.0,
        )

        self.assertTrue(rotation.execute)
        self.assertTrue(gate.armed)
        self.assertEqual(gate.expected_nav_id, "nav-rotate")

    def test_retired_nav_id_cannot_reclaim_waiting_lease(self):
        gate = VelocityProposalGate(ProposalLimits())
        gate.bind(EXPECTED_TOPIC, "nav-001")
        gate.unbind("canvas_stop")
        gate.bind(EXPECTED_TOPIC)

        replay = gate.accept(proposal(nav_id="nav-001"), now=50.0)

        self.assertFalse(replay.execute)
        self.assertEqual(replay.reason, "retired_nav_id_replay")
        self.assertTrue(gate.waiting_for_nav_id)
        self.assertFalse(gate.armed)
        self.assertTrue(
            gate.accept(proposal(nav_id="nav-002"), now=50.1).execute
        )

    def test_first_packet_must_match_control_plane_nav_lease(self):
        rejected = self.gate.accept(
            proposal(nav_id="attacker-selected", sequence=1), now=10.0
        )
        self.assertTrue(rejected.stop)
        self.assertEqual(rejected.reason, "nav_id_mismatch")
        self.assertFalse(self.gate.armed)

    def test_sequence_must_strictly_increase_for_active_nav(self):
        first = self.gate.accept(proposal(sequence=10), now=100.0)
        second = self.gate.accept(proposal(sequence=11), now=100.05)
        replay = self.gate.accept(proposal(sequence=11), now=100.10)
        self.assertTrue(first.execute)
        self.assertTrue(second.execute)
        self.assertTrue(replay.stop)
        self.assertFalse(self.gate.armed)
        self.assertEqual(replay.reason, "sequence_not_increasing")

    def test_nav_id_cannot_change_mid_task(self):
        self.assertTrue(self.gate.accept(proposal(), now=10.0).execute)
        rejected = self.gate.accept(proposal(nav_id="nav-002", sequence=2), now=10.1)
        self.assertTrue(rejected.stop)
        self.assertEqual(rejected.reason, "nav_id_mismatch")

    def test_terminal_zero_retires_and_disarms_nav_lease(self):
        self.assertTrue(self.gate.accept(proposal(), now=10.0).execute)
        terminal = self.gate.accept(
            proposal(
                sequence=2,
                nav_status="arrived",
                velocity={"x": 0.0, "y": 0.0, "yaw": 0.0},
            ),
            now=10.1,
        )
        self.assertTrue(terminal.stop)
        self.assertFalse(self.gate.armed)
        next_task = self.gate.accept(proposal(nav_id="nav-002", sequence=1), now=10.2)
        self.assertFalse(next_task.execute)
        self.assertTrue(next_task.stop)

        self.gate.bind(EXPECTED_TOPIC, "nav-002")
        self.assertTrue(
            self.gate.accept(proposal(nav_id="nav-002", sequence=1), now=10.3).execute
        )

    def test_confirmed_terminal_stop_rearms_first_valid_binding_for_next_task(self):
        gate = VelocityProposalGate(ProposalLimits())
        gate.bind(EXPECTED_TOPIC)
        self.assertTrue(gate.accept(proposal(nav_id="nav-001"), now=10.0).execute)
        terminal = gate.accept(
            proposal(
                nav_id="nav-001",
                sequence=2,
                nav_status="arrived",
                velocity={"x": 0.0, "y": 0.0, "yaw": 0.0},
            ),
            now=10.1,
        )

        self.assertTrue(terminal.stop)
        self.assertFalse(gate.armed)
        self.assertTrue(gate.resume_waiting_after_terminal_stop(True))
        self.assertTrue(gate.waiting_for_nav_id)
        self.assertEqual(gate.expected_nav_id, "")

        replay = gate.accept(
            proposal(nav_id="nav-001", sequence=3),
            now=10.2,
        )
        self.assertFalse(replay.execute)
        self.assertEqual(replay.reason, "retired_nav_id_replay")
        self.assertTrue(gate.waiting_for_nav_id)

        next_task = gate.accept(
            proposal(nav_id="nav-002", sequence=1),
            now=10.3,
        )
        self.assertTrue(next_task.execute)
        self.assertEqual(gate.expected_nav_id, "nav-002")

    def test_expired_terminal_zero_can_close_current_first_valid_task(self):
        gate = VelocityProposalGate(ProposalLimits())
        gate.bind(EXPECTED_TOPIC)
        self.assertTrue(gate.accept(proposal(nav_id="nav-001"), now=10.0).execute)

        terminal = gate.accept(
            proposal(
                nav_id="nav-001",
                sequence=2,
                nav_status="stopped",
                velocity={"x": 0.0, "y": 0.0, "yaw": 0.0},
            ),
            now=10.5,
            now_unix_ms=1_800_000_000_500,
        )

        self.assertTrue(terminal.stop)
        self.assertFalse(terminal.execute)
        self.assertEqual(terminal.reason, "proposal_zero")
        self.assertEqual(gate.last_reason, "nav_task_terminal")
        self.assertTrue(gate.resume_waiting_after_terminal_stop(True))
        self.assertTrue(
            gate.accept(proposal(nav_id="nav-002", sequence=1), now=10.6).execute
        )

    def test_unconfirmed_terminal_stop_remains_fail_closed(self):
        gate = VelocityProposalGate(ProposalLimits())
        gate.bind(EXPECTED_TOPIC)
        self.assertTrue(gate.accept(proposal(), now=10.0).execute)
        gate.accept(
            proposal(
                sequence=2,
                nav_status="arrived",
                velocity={"x": 0.0, "y": 0.0, "yaw": 0.0},
            ),
            now=10.1,
        )

        self.assertFalse(gate.resume_waiting_after_terminal_stop(False))
        self.assertFalse(gate.armed)
        self.assertFalse(gate.waiting_for_nav_id)
        self.assertEqual(gate.last_reason, "nav_task_terminal_stop_unconfirmed")

    def test_late_terminal_confirmation_cannot_rearm_after_unbind(self):
        gate = VelocityProposalGate(ProposalLimits())
        gate.bind(EXPECTED_TOPIC)
        self.assertTrue(gate.accept(proposal(), now=10.0).execute)
        gate.accept(
            proposal(
                sequence=2,
                nav_status="arrived",
                velocity={"x": 0.0, "y": 0.0, "yaw": 0.0},
            ),
            now=10.1,
        )
        gate.unbind("canvas_stop")

        self.assertFalse(gate.resume_waiting_after_terminal_stop(True))
        self.assertFalse(gate.connected_topic)
        self.assertFalse(gate.waiting_for_nav_id)
        self.assertEqual(gate.last_reason, "canvas_stop")

    def test_explicit_or_manual_override_lease_never_auto_rearms(self):
        self.assertTrue(self.gate.accept(proposal(), now=10.0).execute)
        self.gate.accept(
            proposal(
                sequence=2,
                nav_status="arrived",
                velocity={"x": 0.0, "y": 0.0, "yaw": 0.0},
            ),
            now=10.1,
        )
        self.assertFalse(self.gate.resume_waiting_after_terminal_stop(True))
        self.assertFalse(self.gate.waiting_for_nav_id)

        gate = VelocityProposalGate(ProposalLimits())
        gate.bind(EXPECTED_TOPIC)
        self.assertTrue(gate.accept(proposal(), now=20.0).execute)
        gate.disarm("manual_override")
        self.assertFalse(gate.resume_waiting_after_terminal_stop(True))
        self.assertFalse(gate.waiting_for_nav_id)
        self.assertEqual(gate.last_reason, "manual_override")

    def test_completed_nav_id_cannot_be_replayed(self):
        self.assertTrue(self.gate.accept(proposal(), now=10.0).execute)
        self.gate.accept(
            proposal(
                sequence=2,
                nav_status="arrived",
                velocity={"x": 0.0, "y": 0.0, "yaw": 0.0},
            ),
            now=10.1,
        )
        with self.assertRaises(ValueError):
            self.gate.bind(EXPECTED_TOPIC, "nav-001")

    def test_watchdog_requests_stop_without_retiring_trusted_lease(self):
        accepted = self.gate.accept(proposal(ttl_ms=200), now=50.0)
        self.assertAlmostEqual(accepted.duration, 0.2)
        self.assertFalse(self.gate.watchdog(now=50.199).stop)
        expired = self.gate.watchdog(now=50.201)
        self.assertTrue(expired.stop)
        self.assertEqual(expired.reason, "proposal_ttl_expired")
        self.assertTrue(self.gate.armed)
        self.assertFalse(self.gate.recoverable_stop_active)

        self.gate.hold_after_confirmed_stop("proposal_ttl_expired")
        recovered = self.gate.accept(proposal(sequence=2), now=50.202)
        self.assertTrue(recovered.execute)
        self.assertTrue(self.gate.armed)
        self.assertFalse(self.gate.recoverable_stop_active)

    def test_end_to_end_ttl_rejects_proposal_already_expired_at_driver(self):
        rejected = self.gate.accept(
            proposal(issued_at_unix_ms=1_000, ttl_ms=200),
            now=50.0,
            now_unix_ms=1_201,
        )

        self.assertTrue(rejected.stop)
        self.assertEqual(rejected.reason, "proposal_ttl_expired")
        self.assertTrue(self.gate.armed)
        self.assertFalse(self.gate.recoverable_stop_active)
        self.assertEqual(self.gate.last_reason, "proposal_ttl_expired")

        self.gate.hold_after_confirmed_stop("proposal_ttl_expired")
        self.assertEqual(
            self.gate.last_reason,
            "proposal_ttl_stop_recoverable",
        )
        self.assertTrue(
            self.gate.accept(proposal(sequence=2), now=50.1).execute
        )

    def test_confirmed_obstacle_stop_keeps_lease_for_fresh_escape_command(self):
        self.assertTrue(self.gate.accept(proposal(sequence=1), now=50.0).execute)
        self.gate.hold_for_obstacle()

        held = self.gate.snapshot(50.1)
        self.assertTrue(held["connected"])
        self.assertTrue(held["armed"])
        self.assertTrue(held["recoverable_stop_active"])
        self.assertEqual(held["active_nav_id"], "nav-001")
        self.assertEqual(held["last_reason"], "obstacle_stop_recoverable")

        stale = self.gate.accept(
            proposal(sequence=2, issued_at_unix_ms=1_000, ttl_ms=200),
            now=50.2,
            now_unix_ms=1_201,
        )
        self.assertTrue(stale.stop)
        self.assertEqual(stale.reason, "proposal_ttl_expired")
        self.assertTrue(self.gate.armed)
        self.assertTrue(self.gate.recoverable_stop_active)
        self.assertEqual(self.gate.last_sequence, 2)

        escape = self.gate.accept(
            proposal(
                sequence=3,
                velocity={"x": 0.0, "y": 0.0, "yaw": 0.2},
            ),
            now=50.3,
        )
        self.assertTrue(escape.execute)
        self.assertTrue(self.gate.armed)
        self.assertFalse(self.gate.recoverable_stop_active)

    def test_stale_samples_drain_during_confirmed_ttl_hold(self):
        self.assertTrue(self.gate.accept(proposal(sequence=1), now=50.0).execute)
        self.gate.watchdog(now=50.3)
        self.gate.hold_after_confirmed_stop("proposal_ttl_expired")

        stale = self.gate.accept(
            proposal(sequence=2, issued_at_unix_ms=1_000, ttl_ms=200),
            now=50.4,
            now_unix_ms=1_201,
        )

        self.assertTrue(stale.stop)
        self.assertEqual(stale.reason, "proposal_ttl_expired")
        self.assertTrue(self.gate.armed)
        self.assertTrue(self.gate.recoverable_stop_active)
        self.assertEqual(
            self.gate.last_reason,
            "proposal_ttl_stop_recoverable",
        )
        self.assertTrue(
            self.gate.accept(proposal(sequence=3), now=50.5).execute
        )

    def test_hard_fault_still_disarms_from_recoverable_obstacle_stop(self):
        self.assertTrue(self.gate.accept(proposal(sequence=1), now=10.0).execute)
        self.gate.hold_for_obstacle()

        rejected = self.gate.accept(
            proposal(sequence=2, frame="map"),
            now=10.1,
        )

        self.assertTrue(rejected.stop)
        self.assertEqual(rejected.reason, "frame_mismatch")
        self.assertFalse(self.gate.armed)
        self.assertFalse(self.gate.recoverable_stop_active)

    def test_terminal_zero_retires_recoverable_obstacle_lease(self):
        self.assertTrue(self.gate.accept(proposal(sequence=1), now=10.0).execute)
        self.gate.hold_for_obstacle()

        terminal = self.gate.accept(
            proposal(
                sequence=2,
                nav_status="arrived",
                velocity={"x": 0.0, "y": 0.0, "yaw": 0.0},
            ),
            now=10.1,
        )

        self.assertTrue(terminal.stop)
        self.assertFalse(self.gate.armed)
        self.assertFalse(self.gate.recoverable_stop_active)

    def test_end_to_end_ttl_uses_only_remaining_producer_lease(self):
        accepted = self.gate.accept(
            proposal(issued_at_unix_ms=1_000, ttl_ms=200),
            now=50.0,
            now_unix_ms=1_150,
        )

        self.assertTrue(accepted.execute)
        self.assertAlmostEqual(accepted.duration, 0.05)
        self.assertAlmostEqual(self.gate.deadline_monotonic, 50.05)

    def test_future_producer_timestamp_cannot_extend_max_ttl(self):
        accepted = self.gate.accept(
            proposal(issued_at_unix_ms=2_000, ttl_ms=200),
            now=50.0,
            now_unix_ms=1_000,
        )

        self.assertAlmostEqual(accepted.duration, 0.2)
        self.assertAlmostEqual(self.gate.deadline_monotonic, 50.2)

    def test_invalid_payload_disarms_until_explicit_bind(self):
        rejected = self.gate.accept(proposal(frame="map"), now=10.0)
        self.assertTrue(rejected.stop)
        self.assertFalse(self.gate.armed)
        still_rejected = self.gate.accept(proposal(sequence=2), now=10.1)
        self.assertFalse(still_rejected.execute)
        with self.assertRaises(ValueError):
            self.gate.bind(EXPECTED_TOPIC, "nav-001")
        self.gate.bind(EXPECTED_TOPIC, "nav-002")
        self.assertTrue(
            self.gate.accept(proposal(nav_id="nav-002"), now=10.2).execute
        )

    def test_canvas_unbind_retires_nav_id_against_delayed_replay(self):
        self.gate.unbind("canvas_stop")
        with self.assertRaises(ValueError):
            self.gate.bind(EXPECTED_TOPIC, "nav-001")

    def test_repeated_identical_bind_is_idempotent_without_sequence_reset(self):
        self.assertTrue(self.gate.accept(proposal(sequence=10), now=10.0).execute)
        self.gate.bind(EXPECTED_TOPIC, "nav-001")
        replay = self.gate.accept(proposal(sequence=10), now=10.1)
        self.assertTrue(replay.stop)
        self.assertEqual(replay.reason, "sequence_not_increasing")

    def test_retired_nav_ids_are_not_dropped_after_a_fixed_window(self):
        self.gate.unbind("canvas_stop")
        for index in range(2, 41):
            self.gate.bind(EXPECTED_TOPIC, f"nav-{index:03d}")
            self.gate.unbind("canvas_stop")
        for index in range(1, 41):
            with self.subTest(index=index), self.assertRaises(ValueError):
                self.gate.bind(EXPECTED_TOPIC, f"nav-{index:03d}")


if __name__ == "__main__":
    unittest.main()
