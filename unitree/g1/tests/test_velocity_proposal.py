from __future__ import annotations

import math
import unittest

from velocity_proposal import (
    DEFAULT_VELOCITY_PROPOSAL_TOPIC,
    ProposalLimits,
    VelocityProposalGate,
    VelocityProposalValidationError,
    resolve_input_topic,
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


class ProposalValidationTest(unittest.TestCase):
    def setUp(self):
        self.limits = ProposalLimits()

    def test_valid_proposal(self):
        result = validate_velocity_proposal(proposal(), self.limits)
        self.assertEqual(result.nav_id, "nav-001")
        self.assertEqual(result.ttl_ms, 200)
        self.assertFalse(result.is_zero)

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
            {"x": 0.151, "y": 0.0, "yaw": 0.0},
            {"x": -0.051, "y": 0.0, "yaw": 0.0},
            {"x": 0.0, "y": 0.121, "yaw": 0.0},
            {"x": 0.0, "y": 0.0, "yaw": 0.351},
            {"x": 0.15, "y": 0.11, "yaw": 0.0},
        )
        for velocity in velocities:
            with self.subTest(velocity=velocity), self.assertRaises(VelocityProposalValidationError):
                validate_velocity_proposal(proposal(velocity=velocity), self.limits)

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
        self.gate.bind(EXPECTED_TOPIC)

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

    def test_terminal_zero_releases_nav_lease(self):
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
        next_task = self.gate.accept(proposal(nav_id="nav-002", sequence=1), now=10.2)
        self.assertTrue(next_task.execute)

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
        replay = self.gate.accept(proposal(sequence=3), now=10.2)
        self.assertTrue(replay.stop)
        self.assertEqual(replay.reason, "retired_nav_id_replay")

    def test_watchdog_uses_local_monotonic_deadline(self):
        accepted = self.gate.accept(proposal(ttl_ms=200), now=50.0)
        self.assertAlmostEqual(accepted.duration, 0.2)
        self.assertFalse(self.gate.watchdog(now=50.199).stop)
        expired = self.gate.watchdog(now=50.201)
        self.assertTrue(expired.stop)
        self.assertEqual(expired.reason, "proposal_ttl_expired")
        self.assertFalse(self.gate.armed)
        self.assertFalse(self.gate.accept(proposal(sequence=2), now=50.202).execute)

    def test_invalid_payload_disarms_until_explicit_bind(self):
        rejected = self.gate.accept(proposal(frame="map"), now=10.0)
        self.assertTrue(rejected.stop)
        self.assertFalse(self.gate.armed)
        still_rejected = self.gate.accept(proposal(sequence=2), now=10.1)
        self.assertFalse(still_rejected.execute)
        self.gate.bind(EXPECTED_TOPIC)
        self.assertTrue(self.gate.accept(proposal(), now=10.2).execute)


if __name__ == "__main__":
    unittest.main()
