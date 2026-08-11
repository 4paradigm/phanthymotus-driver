"""Pure validation and lease state for the N5 Nav2 velocity proposal gate.

This module deliberately has no ROS 2 or Unitree dependency.  The SmartMotion
subprocess owns validation and authorization; only a ProposalDecision returned
here may be forwarded to the parent Driver RPC worker for physical execution.
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

DRIVER_MIN_X = -1.0
DRIVER_MAX_X = 1.0
DRIVER_MAX_ABS_Y = 1.0
DRIVER_MAX_ABS_YAW = 2.0
DRIVER_MAX_PLANAR_SPEED = math.sqrt(2.0)


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
    min_x: float = DRIVER_MIN_X
    max_x: float = DRIVER_MAX_X
    max_abs_y: float = DRIVER_MAX_ABS_Y
    max_abs_yaw: float = DRIVER_MAX_ABS_YAW
    max_planar_speed: float = DRIVER_MAX_PLANAR_SPEED


def resolve_proposal_limits(config: Mapping[str, Any]) -> ProposalLimits:
    """Resolve deploy-time tightening against the Driver's fixed envelope."""
    return ProposalLimits(
        frame="base_link",
        max_ttl_ms=min(
            250,
            int(config.get("velocity_proposal_max_ttl_ms", 250)),
        ),
        min_x=max(
            DRIVER_MIN_X,
            float(config.get("velocity_proposal_min_x", DRIVER_MIN_X)),
        ),
        max_x=min(
            DRIVER_MAX_X,
            float(config.get("velocity_proposal_max_x", DRIVER_MAX_X)),
        ),
        max_abs_y=min(
            DRIVER_MAX_ABS_Y,
            float(config.get("velocity_proposal_max_abs_y", DRIVER_MAX_ABS_Y)),
        ),
        max_abs_yaw=min(
            DRIVER_MAX_ABS_YAW,
            float(
                config.get(
                    "velocity_proposal_max_abs_yaw",
                    DRIVER_MAX_ABS_YAW,
                )
            ),
        ),
        max_planar_speed=min(
            DRIVER_MAX_PLANAR_SPEED,
            float(
                config.get(
                    "velocity_proposal_max_planar_speed",
                    DRIVER_MAX_PLANAR_SPEED,
                )
            ),
        ),
    )


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
        self.binding_mode = ""
        self.waiting_for_nav_id = False
        self.expected_nav_id = ""
        self.retired_nav_ids: set[str] = set()
        self.last_sequence = -1
        self.last_receive_monotonic = 0.0
        self.deadline_monotonic = 0.0
        self.last_reason = "not_connected"
        self.recoverable_stop_active = False

    def bind(self, topic: str, expected_nav_id: Optional[str] = None) -> None:
        if expected_nav_id is None or expected_nav_id == "":
            if self.is_waiting_on(topic):
                return
            self.connected_topic = topic
            self.armed = False
            self.binding_mode = "first_valid_proposal"
            self.waiting_for_nav_id = True
            self.expected_nav_id = ""
            self.last_sequence = -1
            self.last_receive_monotonic = 0.0
            self.deadline_monotonic = 0.0
            self.last_reason = "waiting_for_nav_id"
            self.recoverable_stop_active = False
            return

        nav_id = resolve_expected_nav_id({"expected_nav_id": expected_nav_id})
        if self.is_bound_to(topic, nav_id):
            return
        if nav_id in self.retired_nav_ids:
            raise ValueError("retired_nav_id_replay")
        self.connected_topic = topic
        self.armed = True
        self.binding_mode = "explicit"
        self.waiting_for_nav_id = False
        self.expected_nav_id = nav_id
        self.last_sequence = -1
        self.last_receive_monotonic = 0.0
        self.deadline_monotonic = 0.0
        self.last_reason = ""
        self.recoverable_stop_active = False

    def unbind(self, reason: str = "canvas_stop") -> None:
        self._retire_expected_nav_id()
        self.connected_topic = ""
        self.armed = False
        self.binding_mode = ""
        self.waiting_for_nav_id = False
        self.expected_nav_id = ""
        self.last_sequence = -1
        self.last_receive_monotonic = 0.0
        self.deadline_monotonic = 0.0
        self.last_reason = reason
        self.recoverable_stop_active = False

    def disarm(self, reason: str) -> None:
        self._retire_expected_nav_id()
        self.armed = False
        self.waiting_for_nav_id = False
        self.deadline_monotonic = 0.0
        self.last_reason = reason
        self.recoverable_stop_active = False

    def resume_waiting_after_terminal(self) -> bool:
        """Release an automatic terminal task and wait for the next nav id.

        Terminal StopMove and odometry results remain diagnostic evidence, but
        they do not latch the first-valid Canvas connection.  Explicit leases
        and non-terminal safety disarms still require a new lifecycle bind.
        """
        if (
            not self.connected_topic
            or self.binding_mode != "first_valid_proposal"
            or self.armed
            or self.waiting_for_nav_id
            or self.last_reason != "nav_task_terminal"
        ):
            return False
        self.waiting_for_nav_id = True
        self.expected_nav_id = ""
        self.last_sequence = -1
        self.last_receive_monotonic = 0.0
        self.deadline_monotonic = 0.0
        self.last_reason = "waiting_for_nav_id"
        self.recoverable_stop_active = False
        return True

    def hold_for_obstacle(self) -> None:
        """Keep the nav lease armed after a confirmed local obstacle stop."""
        self.hold_after_confirmed_stop("obstacle")

    def hold_after_confirmed_stop(self, reason: str) -> None:
        """Keep a trusted nav lease after a confirmed recoverable stop.

        Ordinary obstacle stops and a single proposal TTL lapse are local
        braking events, not proof that the Agent Core lease is invalid.  The
        caller must only enter this state after StopMove has been acknowledged
        and fresh odometry has confirmed zero velocity.
        """
        if not self.armed:
            return
        recoverable_reasons = {
            "obstacle": "obstacle_stop_recoverable",
            "proposal_ttl_expired": "proposal_ttl_stop_recoverable",
        }
        if reason not in recoverable_reasons:
            raise ValueError("unsupported_recoverable_stop_reason")
        self.deadline_monotonic = 0.0
        self.last_reason = recoverable_reasons[reason]
        self.recoverable_stop_active = True

    def request_ttl_stop(
        self,
        now: float,
        proposal: Optional[ValidatedVelocityProposal] = None,
    ) -> ProposalDecision:
        """Request fail-closed braking without retiring the trusted lease.

        The lease remains armed only so the stop path can move it into a
        recoverable hold after measured zero velocity.  No new command can be
        executed while that serialized stop confirmation is in progress.
        """
        if proposal is not None:
            self.last_sequence = proposal.sequence
            self.last_receive_monotonic = now
        self.deadline_monotonic = 0.0
        self.last_reason = "proposal_ttl_expired"
        self.recoverable_stop_active = False
        return ProposalDecision(
            stop=True,
            reason="proposal_ttl_expired",
            proposal=proposal,
        )

    def _retire_expected_nav_id(self) -> None:
        if self.expected_nav_id:
            self.retired_nav_ids.add(self.expected_nav_id)

    def is_bound_to(self, topic: str, expected_nav_id: str) -> bool:
        return bool(
            self.armed
            and self.connected_topic == topic
            and self.expected_nav_id == expected_nav_id
        )

    def is_waiting_on(self, topic: str) -> bool:
        return bool(
            self.connected_topic == topic
            and self.binding_mode == "first_valid_proposal"
            and self.waiting_for_nav_id
            and not self.armed
            and not self.expected_nav_id
        )

    def _claim_first_valid_proposal(self, nav_id: str) -> None:
        if not self.waiting_for_nav_id:
            raise ValueError("not_waiting_for_nav_id")
        resolved = resolve_expected_nav_id({"expected_nav_id": nav_id})
        if resolved in self.retired_nav_ids:
            raise ValueError("retired_nav_id_replay")
        self.expected_nav_id = resolved
        self.armed = True
        self.waiting_for_nav_id = False
        self.last_reason = ""

    @staticmethod
    def _remaining_duration(
        proposal: ValidatedVelocityProposal,
        now_unix_ms: Optional[float],
    ) -> float:
        duration = proposal.ttl_ms / 1000.0
        if now_unix_ms is None:
            return duration
        remaining = (
            proposal.issued_at_unix_ms
            + proposal.ttl_ms
            - float(now_unix_ms)
        ) / 1000.0
        if remaining <= 0.0:
            return 0.0
        # A future producer timestamp may not extend the configured TTL.
        return min(duration, remaining)

    def accept(
        self,
        payload: Mapping[str, Any],
        now: float,
        now_unix_ms: Optional[float] = None,
    ) -> ProposalDecision:
        if not self.connected_topic:
            return ProposalDecision(stop=True, reason="proposal_not_connected")
        if not self.armed and not self.waiting_for_nav_id:
            return ProposalDecision(stop=True, reason=self.last_reason or "proposal_not_armed")
        try:
            proposal = validate_velocity_proposal(payload, self.limits)
        except VelocityProposalValidationError as exc:
            if self.waiting_for_nav_id:
                return ProposalDecision(reason=exc.code)
            self.disarm(exc.code)
            return ProposalDecision(stop=True, reason=exc.code)

        duration = self._remaining_duration(proposal, now_unix_ms)
        if self.waiting_for_nav_id:
            if duration <= 0.0:
                return ProposalDecision(
                    reason="proposal_ttl_expired",
                    proposal=proposal,
                )
            if proposal.nav_id in self.retired_nav_ids:
                return ProposalDecision(
                    reason="retired_nav_id_replay",
                    proposal=proposal,
                )
            if proposal.is_zero:
                # A retained zero/terminal sample from an earlier task must
                # not claim a newly waiting lease.  Only an executable command
                # establishes physical-control intent for a new nav_id.
                return ProposalDecision(
                    reason="waiting_for_active_proposal",
                    proposal=proposal,
                )
            try:
                self._claim_first_valid_proposal(proposal.nav_id)
            except ValueError as exc:
                return ProposalDecision(
                    reason=str(exc),
                    proposal=proposal,
                )

        if proposal.nav_id != self.expected_nav_id:
            self.disarm("nav_id_mismatch")
            return ProposalDecision(stop=True, reason="nav_id_mismatch", proposal=proposal)
        if proposal.sequence <= self.last_sequence:
            self.disarm("sequence_not_increasing")
            return ProposalDecision(stop=True, reason="sequence_not_increasing", proposal=proposal)

        if proposal.is_terminal:
            # A terminal proposal is schema-validated to carry exact zero
            # velocity.  For the currently bound nav_id, accepting it after
            # its motion TTL has elapsed can only stop and retire the lease;
            # it can never authorize stale motion.  This keeps consecutive
            # first-valid tasks recoverable when a loaded ROS executor delays
            # the one-shot terminal sample beyond the 250 ms motion budget.
            self.last_sequence = proposal.sequence
            self.last_receive_monotonic = now
            self.deadline_monotonic = 0.0
            self.disarm("nav_task_terminal")
            return ProposalDecision(
                stop=True,
                reason="proposal_zero",
                proposal=proposal,
            )

        if duration <= 0.0:
            if now_unix_ms is not None:
                if self.recoverable_stop_active:
                    # Stop confirmation can outlive the proposal TTL while
                    # ROS retains newer samples.  The robot is already at a
                    # confirmed stop, so reject this stale sample without
                    # retiring the task lease; only a fresh later sample may
                    # resume execution.
                    self.last_sequence = proposal.sequence
                    self.last_receive_monotonic = now
                    self.deadline_monotonic = 0.0
                    # Preserve whether this hold came from an obstacle or a
                    # prior TTL stop while stale retained samples drain.
                    return ProposalDecision(
                        stop=True,
                        reason="proposal_ttl_expired",
                        proposal=proposal,
                    )
                return self.request_ttl_stop(now, proposal)

        self.last_sequence = proposal.sequence
        self.last_receive_monotonic = now
        if proposal.is_zero:
            self.deadline_monotonic = 0.0
            self.last_reason = proposal.status
            return ProposalDecision(stop=True, reason="proposal_zero", proposal=proposal)

        self.deadline_monotonic = now + duration
        self.last_reason = ""
        self.recoverable_stop_active = False
        return ProposalDecision(execute=True, duration=duration, proposal=proposal)

    def watchdog(self, now: float) -> ProposalDecision:
        if self.deadline_monotonic <= 0.0 or now < self.deadline_monotonic:
            return ProposalDecision()
        return self.request_ttl_stop(now)

    def snapshot(self, now: float) -> dict:
        age_ms = None
        if self.last_receive_monotonic > 0.0:
            age_ms = max(0, round((now - self.last_receive_monotonic) * 1000))
        return {
            "connected": bool(self.connected_topic),
            "armed": self.armed,
            "binding_mode": self.binding_mode or None,
            "waiting_for_nav_id": self.waiting_for_nav_id,
            "auto_bound_nav_id": (
                self.expected_nav_id
                if self.binding_mode == "first_valid_proposal"
                and self.expected_nav_id
                else None
            ),
            "topic": self.connected_topic or None,
            "expected_nav_id": self.expected_nav_id or None,
            "active_nav_id": self.expected_nav_id if self.armed else None,
            "last_sequence": self.last_sequence if self.last_sequence >= 0 else None,
            "last_message_age_ms": age_ms,
            "last_reason": self.last_reason or None,
            "recoverable_stop_active": self.recoverable_stop_active,
        }
