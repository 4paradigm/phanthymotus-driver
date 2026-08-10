"""Deterministic lifecycle gating for arm SDK command publication.

Ported from Unitree ``xr_teleoperate`` under Apache-2.0; see NOTICE.md.

This module deliberately has no robot, DDS, or third-party dependencies.  It
only decides whether a controller may publish and which SDK weight it should
use.  Sending any final zero-weight frames remains the controller's job.
"""

from __future__ import annotations

import math
import threading
from dataclasses import dataclass
from enum import Enum
from numbers import Real
from typing import Optional


class ArmSdkState(str, Enum):
    """States in the arm command publication lifecycle."""

    DISARMED = "disarmed"
    ARMING = "arming"
    ARMED = "armed"
    RELEASING = "releasing"
    HARD_FAULT = "hard_fault"


@dataclass(frozen=True)
class ArmSdkSample:
    """An immutable decision for one controller iteration."""

    state: ArmSdkState
    publish_allowed: bool
    weight: float


class ArmSdkGate:
    """Thread-safe state machine for gradually taking and releasing arm control.

    Time is supplied by the caller so the gate is deterministic and easy to
    test.  All time-bearing calls must use a finite, nondecreasing clock.
    ``hard_fault`` is intentionally latched and there is no reset operation;
    recovery requires constructing a new gate as part of a deliberate restart.
    """

    def __init__(self) -> None:
        self._lock = threading.RLock()
        self._state = ArmSdkState.DISARMED
        self._weight = 0.0
        self._last_now: Optional[float] = None
        self._transition_started_at = 0.0
        self._transition_duration = 0.0
        self._transition_start_weight = 0.0
        self._release_reason: Optional[str] = None
        self._fault_reason: Optional[str] = None

    @property
    def state(self) -> ArmSdkState:
        with self._lock:
            return self._state

    @property
    def release_reason(self) -> Optional[str]:
        with self._lock:
            return self._release_reason

    @property
    def fault_reason(self) -> Optional[str]:
        with self._lock:
            return self._fault_reason

    def arm(self, now: float, ramp_s: float) -> ArmSdkSample:
        """Begin taking control, ramping SDK weight from zero to one.

        Arming is only valid while disarmed.  A zero-second ramp transitions
        directly to ``ARMED``.
        """

        now_value = self._validate_finite_number(now, "now")
        ramp_value = self._validate_ramp(ramp_s)

        with self._lock:
            self._observe(now_value)
            if self._state is ArmSdkState.HARD_FAULT:
                raise RuntimeError("cannot arm a hard-faulted gate")
            if self._state is not ArmSdkState.DISARMED:
                raise RuntimeError(f"cannot arm while gate is {self._state.value}")

            self._release_reason = None
            self._transition_started_at = now_value
            self._transition_duration = ramp_value
            self._transition_start_weight = 0.0
            self._weight = 0.0

            if ramp_value == 0.0:
                self._state = ArmSdkState.ARMED
                self._weight = 1.0
            else:
                self._state = ArmSdkState.ARMING

            return self._snapshot()

    def release(self, now: float, ramp_s: float, reason: str) -> ArmSdkSample:
        """Release control, ramping from the current SDK weight to zero.

        Release may interrupt an in-progress arm ramp.  Calling it while
        already disarmed is a safe no-op.  Repeated calls while releasing are
        idempotent and do not restart or lengthen the existing ramp.
        """

        now_value = self._validate_finite_number(now, "now")
        ramp_value = self._validate_ramp(ramp_s)
        reason_value = self._validate_reason(reason)

        with self._lock:
            self._observe(now_value)
            if self._state is ArmSdkState.HARD_FAULT:
                raise RuntimeError("cannot release a hard-faulted gate")
            if self._state is ArmSdkState.DISARMED:
                self._release_reason = reason_value
                return self._snapshot()
            if self._state is ArmSdkState.RELEASING:
                return self._snapshot()

            self._release_reason = reason_value
            self._transition_started_at = now_value
            self._transition_duration = ramp_value
            self._transition_start_weight = self._weight

            if ramp_value == 0.0 or self._weight == 0.0:
                self._finish_release()
            else:
                self._state = ArmSdkState.RELEASING

            return self._snapshot()

    def hard_fault(self, reason: str) -> ArmSdkSample:
        """Latch an unrecoverable fault and prohibit further publication.

        The first fault reason is retained so a secondary cleanup failure
        cannot hide the initiating fault.
        """

        reason_value = self._validate_reason(reason)

        with self._lock:
            if self._state is not ArmSdkState.HARD_FAULT:
                self._fault_reason = reason_value
                self._state = ArmSdkState.HARD_FAULT
                self._weight = 0.0
                self._transition_duration = 0.0
                self._transition_start_weight = 0.0
            return self._snapshot()

    def sample(self, now: float) -> ArmSdkSample:
        """Advance the state machine to ``now`` and return its decision."""

        now_value = self._validate_finite_number(now, "now")
        with self._lock:
            self._observe(now_value)
            return self._snapshot()

    def _observe(self, now: float) -> None:
        if self._last_now is not None and now < self._last_now:
            raise ValueError(
                f"now must be nondecreasing (received {now}, last {self._last_now})"
            )
        self._last_now = now
        self._advance(now)

    def _advance(self, now: float) -> None:
        if self._state not in (ArmSdkState.ARMING, ArmSdkState.RELEASING):
            return

        elapsed = now - self._transition_started_at
        progress = min(1.0, max(0.0, elapsed / self._transition_duration))

        if self._state is ArmSdkState.ARMING:
            self._weight = progress
            if progress == 1.0:
                self._state = ArmSdkState.ARMED
                self._weight = 1.0
            return

        self._weight = self._transition_start_weight * (1.0 - progress)
        if progress == 1.0:
            self._finish_release()

    def _finish_release(self) -> None:
        self._state = ArmSdkState.DISARMED
        self._weight = 0.0
        self._transition_duration = 0.0
        self._transition_start_weight = 0.0

    def _snapshot(self) -> ArmSdkSample:
        publish_allowed = self._state in (
            ArmSdkState.ARMING,
            ArmSdkState.ARMED,
            ArmSdkState.RELEASING,
        )
        return ArmSdkSample(
            state=self._state,
            publish_allowed=publish_allowed,
            weight=self._weight,
        )

    @staticmethod
    def _validate_finite_number(value: float, name: str) -> float:
        if isinstance(value, bool) or not isinstance(value, Real):
            raise TypeError(f"{name} must be a real number")
        result = float(value)
        if not math.isfinite(result):
            raise ValueError(f"{name} must be finite")
        return result

    @classmethod
    def _validate_ramp(cls, ramp_s: float) -> float:
        result = cls._validate_finite_number(ramp_s, "ramp_s")
        if result < 0.0:
            raise ValueError("ramp_s must be nonnegative")
        return result

    @staticmethod
    def _validate_reason(reason: str) -> str:
        if not isinstance(reason, str):
            raise TypeError("reason must be a string")
        if not reason.strip():
            raise ValueError("reason must not be empty")
        return reason


__all__ = ["ArmSdkGate", "ArmSdkSample", "ArmSdkState"]
