"""Pure validation and lease state for the platform navigation velocity proposal gate.

T800 execution path: LocomotionPlugin owns the gate directly (no SmartMotion
subprocess) and maps accepted proposals onto the 100Hz BodyVelCmd stream.
This module deliberately has no ROS 2 or EngineAI dependency.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Mapping, Optional


VELOCITY_PROPOSAL_SCHEMA = "phanthy.navigation.velocity_proposal.v1"
DEFAULT_VELOCITY_PROPOSAL_TOPIC = "/ubuntu/navigation/nav2/velocity_proposal"
TERMINAL_STATUSES = {
    "paused",
    "arrived",
    "cancelled",
    "stopped",
    "error",
    "aborted",
    "rejected",
}
ACTIVE_STATUSES = {"planning", "navigating", "replanning", "running", "active"}
ALLOWED_STATUSES = TERMINAL_STATUSES | ACTIVE_STATUSES
_STATUS_FIELD = "nav_status"
_UNSUPPORTED_STATUS_ALIASES = {"status", "navigation_status", "navigation_state"}


def velocity_proposal_port(topic: str) -> dict:
    """Return the authoritative N5 canvas port declaration."""
    return {
        "port": "velocity_proposal",
        "topic": topic,
        "format": "data/json",
        "ros_type": "std_msgs/msg/String",
        "qos": "RELIABLE + KEEP_LAST(depth=10) + VOLATILE",
        "schema": VELOCITY_PROPOSAL_SCHEMA,
    }


class VelocityProposalValidationError(ValueError):
    """Validation error with a stable fail-closed reason code."""

    def __init__(self, code: str):
        super().__init__(code)
        self.code = code


@dataclass(frozen=True)
class ProposalLimits:
    schema: str = VELOCITY_PROPOSAL_SCHEMA
    frame: str = "base_link"
    max_ttl_ms: int = 250
    min_x: float = -0.05
    max_x: float = 0.15
    max_abs_y: float = 0.12
    max_abs_yaw: float = 0.35
    max_planar_speed: float = 0.18


@dataclass(frozen=True)
class ValidatedVelocityProposal:
    nav_id: str
    sequence: int
    issued_at_unix_ms: float
    ttl_ms: int
    frame: str
    status: str
    x: float
    y: float
    yaw: float

    @property
    def is_zero(self) -> bool:
        return self.x == 0.0 and self.y == 0.0 and self.yaw == 0.0

    @property
    def is_terminal(self) -> bool:
        return self.status in TERMINAL_STATUSES


@dataclass(frozen=True)
class ProposalDecision:
    execute: bool = False
    stop: bool = False
    reason: str = ""
    duration: float = 0.0
    proposal: Optional[ValidatedVelocityProposal] = None


def resolve_input_topic(args: Mapping[str, Any], expected_topic: str) -> str:
    """Resolve the canvas connection and reject every topic except the N5 port."""
    has_input_topic = bool(str(args.get("input_topic") or "").strip())
    has_input_topics = args.get("input_topics") is not None
    if has_input_topic and has_input_topics:
        raise ValueError("input_topic_and_input_topics_are_mutually_exclusive")
    topics = []
    input_topics = args.get("input_topics")
    if input_topics is not None:
        if not isinstance(input_topics, (list, tuple)):
            raise ValueError("input_topics_must_be_a_list")
        if len(input_topics) != 1:
            raise ValueError("exactly_one_velocity_proposal_topic_required")
        topics.append(str(input_topics[0]).strip())
    input_topic = str(args.get("input_topic") or "").strip()
    if input_topic and input_topic not in topics:
        topics.append(input_topic)
    if len(topics) != 1:
        raise ValueError("exactly_one_velocity_proposal_topic_required")
    if topics[0] != expected_topic:
        raise ValueError("unexpected_velocity_proposal_topic")
    return topics[0]


def resolve_expected_nav_id(args: Mapping[str, Any]) -> str:
    """Resolve the task lease supplied by the trusted lifecycle control plane."""
    value = args.get("expected_nav_id")
    if value is None or value == "":
        raise ValueError("expected_nav_id_required")
    if not isinstance(value, str):
        raise ValueError("invalid_expected_nav_id")
    nav_id = value.strip()
    if not nav_id:
        raise ValueError("expected_nav_id_required")
    if len(nav_id) > 128:
        raise ValueError("invalid_expected_nav_id")
    return nav_id


def _finite_number(value: Any, code: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise VelocityProposalValidationError(code)
    result = float(value)
    if not math.isfinite(result):
        raise VelocityProposalValidationError(code)
    return result


def _status(payload: Mapping[str, Any]) -> str:
    if any(field in payload for field in _UNSUPPORTED_STATUS_ALIASES):
        raise VelocityProposalValidationError("unsupported_navigation_status_field")
    status = payload.get(_STATUS_FIELD)
    if not isinstance(status, str) or not status.strip():
        raise VelocityProposalValidationError("invalid_navigation_status")
    normalized = status.strip().lower()
    if normalized not in ALLOWED_STATUSES:
        raise VelocityProposalValidationError("unsupported_navigation_status")
    return normalized


def validate_velocity_proposal(
    payload: Mapping[str, Any],
    limits: ProposalLimits,
) -> ValidatedVelocityProposal:
    if not isinstance(payload, Mapping):
        raise VelocityProposalValidationError("proposal_must_be_object")
    if payload.get("schema") != limits.schema:
        raise VelocityProposalValidationError("schema_mismatch")
    if payload.get("frame") != limits.frame:
        raise VelocityProposalValidationError("frame_mismatch")
    if payload.get("shadow_only") is not True:
        raise VelocityProposalValidationError("shadow_only_flag_required")
    if payload.get("physical_execution") is not False:
        raise VelocityProposalValidationError("physical_execution_flag_must_be_false")

    nav_id = payload.get("nav_id")
    if not isinstance(nav_id, str) or not nav_id.strip() or len(nav_id) > 128:
        raise VelocityProposalValidationError("invalid_nav_id")
    sequence = payload.get("sequence")
    if isinstance(sequence, bool) or not isinstance(sequence, int) or sequence < 0:
        raise VelocityProposalValidationError("invalid_sequence")
    ttl_ms = payload.get("ttl_ms")
    if (
        isinstance(ttl_ms, bool)
        or not isinstance(ttl_ms, int)
        or ttl_ms <= 0
        or ttl_ms > limits.max_ttl_ms
    ):
        raise VelocityProposalValidationError("invalid_ttl_ms")
    issued_at = _finite_number(payload.get("issued_at_unix_ms"), "invalid_issued_at_unix_ms")

    velocity = payload.get("velocity")
    if not isinstance(velocity, Mapping):
        raise VelocityProposalValidationError("velocity_must_be_object")
    x = _finite_number(velocity.get("x"), "invalid_velocity_x")
    y = _finite_number(velocity.get("y"), "invalid_velocity_y")
    yaw = _finite_number(velocity.get("yaw"), "invalid_velocity_yaw")
    if x < limits.min_x or x > limits.max_x:
        raise VelocityProposalValidationError("velocity_x_limit")
    if abs(y) > limits.max_abs_y:
        raise VelocityProposalValidationError("velocity_y_limit")
    if abs(yaw) > limits.max_abs_yaw:
        raise VelocityProposalValidationError("velocity_yaw_limit")
    if math.hypot(x, y) > limits.max_planar_speed:
        raise VelocityProposalValidationError("planar_speed_limit")

    result = ValidatedVelocityProposal(
        nav_id=nav_id.strip(),
        sequence=sequence,
        issued_at_unix_ms=issued_at,
        ttl_ms=ttl_ms,
        frame=limits.frame,
        status=_status(payload),
        x=x,
        y=y,
        yaw=yaw,
    )
    if result.is_terminal and not result.is_zero:
        raise VelocityProposalValidationError("terminal_status_requires_zero_velocity")
    return result


class VelocityProposalGate:
    """Connection, task lease, replay protection, and monotonic TTL state."""

    def __init__(self, limits: ProposalLimits):
        self.limits = limits
        self.connected_topic = ""
        self.armed = False
        self.expected_nav_id = ""
        self.retired_nav_ids: set[str] = set()
        self.last_sequence = -1
        self.last_receive_monotonic = 0.0
        self.deadline_monotonic = 0.0
        self.last_reason = "not_connected"

    def bind(self, topic: str, expected_nav_id: str) -> None:
        nav_id = resolve_expected_nav_id({"expected_nav_id": expected_nav_id})
        if self.is_bound_to(topic, nav_id):
            return
        if nav_id in self.retired_nav_ids:
            raise ValueError("retired_nav_id_replay")
        self.connected_topic = topic
        self.armed = True
        self.expected_nav_id = nav_id
        self.last_sequence = -1
        self.last_receive_monotonic = 0.0
        self.deadline_monotonic = 0.0
        self.last_reason = ""

    def unbind(self, reason: str = "canvas_stop") -> None:
        self._retire_expected_nav_id()
        self.connected_topic = ""
        self.armed = False
        self.expected_nav_id = ""
        self.last_sequence = -1
        self.last_receive_monotonic = 0.0
        self.deadline_monotonic = 0.0
        self.last_reason = reason

    def disarm(self, reason: str) -> None:
        self._retire_expected_nav_id()
        self.armed = False
        self.deadline_monotonic = 0.0
        self.last_reason = reason

    def _retire_expected_nav_id(self) -> None:
        if self.expected_nav_id:
            self.retired_nav_ids.add(self.expected_nav_id)

    def is_bound_to(self, topic: str, expected_nav_id: str) -> bool:
        return bool(
            self.armed
            and self.connected_topic == topic
            and self.expected_nav_id == expected_nav_id
        )

    def accept(self, payload: Mapping[str, Any], now: float) -> ProposalDecision:
        if not self.connected_topic:
            return ProposalDecision(stop=True, reason="proposal_not_connected")
        if not self.armed:
            return ProposalDecision(stop=True, reason=self.last_reason or "proposal_not_armed")
        try:
            proposal = validate_velocity_proposal(payload, self.limits)
        except VelocityProposalValidationError as exc:
            self.disarm(exc.code)
            return ProposalDecision(stop=True, reason=exc.code)

        if proposal.nav_id != self.expected_nav_id:
            self.disarm("nav_id_mismatch")
            return ProposalDecision(stop=True, reason="nav_id_mismatch", proposal=proposal)
        if proposal.sequence <= self.last_sequence:
            self.disarm("sequence_not_increasing")
            return ProposalDecision(stop=True, reason="sequence_not_increasing", proposal=proposal)

        self.last_sequence = proposal.sequence
        self.last_receive_monotonic = now
        if proposal.is_zero:
            self.deadline_monotonic = 0.0
            if proposal.is_terminal:
                self.disarm("nav_task_terminal")
            else:
                self.last_reason = proposal.status
            return ProposalDecision(stop=True, reason="proposal_zero", proposal=proposal)

        duration = proposal.ttl_ms / 1000.0
        self.deadline_monotonic = now + duration
        self.last_reason = ""
        return ProposalDecision(execute=True, duration=duration, proposal=proposal)

    def watchdog(self, now: float) -> ProposalDecision:
        if self.deadline_monotonic <= 0.0 or now < self.deadline_monotonic:
            return ProposalDecision()
        self.disarm("proposal_ttl_expired")
        return ProposalDecision(stop=True, reason=self.last_reason)

    def snapshot(self, now: float) -> dict:
        age_ms = None
        if self.last_receive_monotonic > 0.0:
            age_ms = max(0, round((now - self.last_receive_monotonic) * 1000))
        return {
            "connected": bool(self.connected_topic),
            "armed": self.armed,
            "topic": self.connected_topic or None,
            "expected_nav_id": self.expected_nav_id or None,
            "active_nav_id": self.expected_nav_id if self.armed else None,
            "last_sequence": self.last_sequence if self.last_sequence >= 0 else None,
            "last_message_age_ms": age_ms,
            "last_reason": self.last_reason or None,
        }
