#!/usr/bin/env python3
"""
drivers/unitree/g1/safety_harness.py — SmartMotion 独立进程安全层。

架构：
  - SmartMotionProcess: 独立子进程，拥有自己的 DDS 通道和 ROS2 节点
    - 订阅 LiDAR 点云，10Hz 全量 numpy 处理
    - 在本地执行停止和 SlamClient RPC
    - 将 proposal SetVelocity 和 FSM GetFsmId 有界交给主进程 Driver RPC
    - 发布运动事件到 ROS2 topic
  - SmartMotionProxy: 主进程中的轻量代理，通过 multiprocessing Queue 通信
    - 对外暴露与原 SmartMotion 相同的 API
    - 非阻塞命令发送，同步等待结果

此模块不是 MCP plugin，由驱动生命周期管理，默认自动启动。
"""

from __future__ import annotations

import enum
import json
import math
import multiprocessing as mp
import os
import queue
import struct
import threading
import time
from collections import deque
from dataclasses import dataclass, field
from typing import Optional

from velocity_proposal import (
    DEFAULT_VELOCITY_PROPOSAL_TOPIC,
    VelocityProposalGate,
    resolve_expected_nav_id,
    resolve_proposal_limits,
)


UNITREE_STOP_MOVE_DURATION_SECONDS = 1.0
DEFAULT_STOP_CONFIRM_TIMEOUT_SECONDS = 2.0
MAX_STOP_CONFIRM_TIMEOUT_SECONDS = 3.0
TRANSLATION_OBSTACLE_EPSILON = 0.01


def resolve_stop_confirmation_timeout(configured_timeout):
    """Bound zero-odom confirmation around the G1 StopMove command.

    unitree_sdk2py implements StopMove as SetVelocity(0, 0, 0) with the
    default one-second duration.  The physical confirmation window must not
    expire before that command can finish, while remaining fail-closed and
    bounded against missing odometry.
    """
    return min(
        MAX_STOP_CONFIRM_TIMEOUT_SECONDS,
        max(
            UNITREE_STOP_MOVE_DURATION_SECONDS,
            float(configured_timeout),
        ),
    )


# ── Enums & Data Classes (shared between processes) ──────────────────────────

class MotionState(enum.Enum):
    IDLE = "idle"
    MOVING = "moving"
    NAVIGATING = "navigating"
    NAV_PAUSED = "nav_paused"


class StopReason(enum.Enum):
    COMMAND = "command"
    OBSTACLE = "obstacle"
    DURATION_EXPIRED = "duration_expired"
    NAV_COMPLETE = "nav_complete"
    TILT = "tilt"
    FOOT_AIRBORNE = "foot_airborne"
    COMM_TIMEOUT = "comm_timeout"
    JOINT_OVERHEAT = "joint_overheat"


class SpeedZone(enum.Enum):
    NORMAL = "normal"
    DECELERATED = "decelerated"
    STOPPED = "stopped"


def translation_obstacle_heading(command, epsilon=TRANSLATION_OBSTACLE_EPSILON):
    """Return the translation heading, or ``None`` for an in-place command."""
    if not command:
        return None
    vx = float(command.get("vx", 0.0))
    vy = float(command.get("vy", 0.0))
    if math.hypot(vx, vy) <= float(epsilon):
        return None
    return math.atan2(vy, vx)


def obstacle_observation_applies(
    command,
    observation_heading,
    heading_tolerance,
):
    """Reject stale cones and never treat pure rotation as translation."""
    command_heading = translation_obstacle_heading(command)
    if command_heading is None or observation_heading is None:
        return False
    delta = abs(
        (command_heading - float(observation_heading) + math.pi)
        % (2 * math.pi)
        - math.pi
    )
    return delta <= max(0.0, float(heading_tolerance))


def authoritative_proposal_stop_reason(requested_reason, watchdog_fault):
    """Preserve an execution fault that won before a concurrent local stop."""
    if watchdog_fault:
        return str(watchdog_fault[1])
    return str(requested_reason)


def proposal_decision_requires_physical_stop(
    reason,
    has_proposal,
    proposal_motion_active,
    newly_disarmed,
    recoverable_stop_active,
):
    """Let confirmed obstacle holds drain stale ROS samples without RPCs."""
    if (
        recoverable_stop_active
        and reason == "proposal_ttl_expired"
        and not proposal_motion_active
        and not newly_disarmed
    ):
        return False
    return bool(has_proposal or proposal_motion_active or newly_disarmed)


@dataclass
class SpeedLimits:
    vx_normal: float = 1.0
    vx_max: float = 5.0
    vx_decel: float = 0.5
    vy_normal: float = 0.2
    vy_max: float = 5.0
    vy_decel: float = 0.1
    vyaw_normal: float = 0.4
    vyaw_max: float = 5.0
    vyaw_decel: float = 0.2


@dataclass(frozen=True)
class StopConfirmationStart:
    monotonic: float
    unix_ms: int
    odometry_callback_count: int


class OdomStopMonitor:
    """Wait for measured zero velocity without blocking the odom callback."""

    def __init__(self):
        self._condition = threading.Condition()
        self._last_time = 0.0
        self._last_velocity = (float("inf"), float("inf"), float("inf"))
        self._callback_count = 0
        self._callback_history = deque(maxlen=4096)

    def record(self, velocity, received_monotonic=None):
        received = (
            time.monotonic()
            if received_monotonic is None
            else float(received_monotonic)
        )
        motion = tuple(float(value) for value in velocity)
        if len(motion) != 3:
            raise ValueError("odometry velocity must contain x, y, and yaw")
        with self._condition:
            self._last_time = received
            self._last_velocity = motion
            self._callback_count += 1
            self._callback_history.append((received, self._callback_count))
            self._condition.notify_all()

    def begin_confirmation(self):
        with self._condition:
            return StopConfirmationStart(
                monotonic=time.monotonic(),
                unix_ms=round(time.time() * 1000),
                odometry_callback_count=self._callback_count,
            )

    def latest(self, now=None):
        observed_at = time.monotonic() if now is None else float(now)
        with self._condition:
            age = (
                observed_at - self._last_time
                if self._last_time
                else float("inf")
            )
            return {
                "time": self._last_time,
                "age": age,
                "velocity": self._last_velocity,
                "callback_count": self._callback_count,
            }

    def callback_count_at(self, observed_monotonic):
        """Return the last callback count observed at or before a timestamp."""
        boundary = float(observed_monotonic)
        with self._condition:
            for received, callback_count in reversed(self._callback_history):
                if received <= boundary:
                    return callback_count
            return 0

    def wait_for_stopped(
        self,
        start,
        timeout,
        max_age,
        linear_epsilon,
        yaw_epsilon,
        stop_move_ret,
        stop_move_error=None,
        stop_move_completed_monotonic=None,
        callbacks_at_stop_move_completion=None,
    ):
        deadline = start.monotonic + timeout
        confirmed = False
        timed_out = False
        with self._condition:
            while stop_move_ret == 0:
                now = time.monotonic()
                vx, vy, yaw = self._last_velocity
                is_new = (
                    self._callback_count > start.odometry_callback_count
                    and self._last_time >= start.monotonic
                    and self._last_time <= deadline
                )
                is_fresh = self._last_time and now - self._last_time <= max_age
                is_zero = (
                    abs(vx) <= linear_epsilon
                    and abs(vy) <= linear_epsilon
                    and abs(yaw) <= yaw_epsilon
                )
                if is_new and is_fresh and is_zero:
                    confirmed = True
                    break
                remaining = deadline - now
                if remaining <= 0.0:
                    timed_out = True
                    break
                # Condition.wait releases the monitor lock, so the dedicated
                # Unitree ch_reader thread can record and signal the next odom.
                self._condition.wait(timeout=remaining)

            now = time.monotonic()
            age = now - self._last_time if self._last_time else float("inf")
            vx, vy, yaw = self._last_velocity
            diagnostics = {
                "stop_move_ret": stop_move_ret,
                "stop_move_error": stop_move_error,
                "confirmation_started_monotonic": start.monotonic,
                "confirmation_started_unix_ms": start.unix_ms,
                "stop_move_completed_monotonic": stop_move_completed_monotonic,
                "stop_move_duration_ms": (
                    round(
                        (stop_move_completed_monotonic - start.monotonic) * 1000
                    )
                    if stop_move_completed_monotonic is not None
                    else None
                ),
                "last_odometry_monotonic": self._last_time or None,
                "last_odometry_age_ms": (
                    round(age * 1000) if math.isfinite(age) else None
                ),
                "last_odometry_velocity": {
                    "x": vx if math.isfinite(vx) else None,
                    "y": vy if math.isfinite(vy) else None,
                    "yaw": yaw if math.isfinite(yaw) else None,
                },
                "odometry_callback_count": self._callback_count,
                "odometry_callbacks_since_confirmation": max(
                    0,
                    self._callback_count - start.odometry_callback_count,
                ),
                "odometry_callbacks_during_stop_move": max(
                    0,
                    (
                        callbacks_at_stop_move_completion
                        if callbacks_at_stop_move_completion is not None
                        else start.odometry_callback_count
                    ) - start.odometry_callback_count,
                ),
                "confirmation_timed_out": timed_out,
            }
        return confirmed, diagnostics


def finish_stop_confirmation(
    monitor,
    start,
    stop_move_ret,
    stop_move_error,
    stop_move_completed_monotonic,
    timeout,
    max_age,
    linear_epsilon,
    yaw_epsilon,
    after_stop_attempt=None,
):
    """Confirm an acknowledged StopMove against post-boundary odometry."""
    callbacks_at_stop_move_completion = monitor.callback_count_at(
        stop_move_completed_monotonic
    )
    if after_stop_attempt is not None:
        after_stop_attempt()

    stop_confirmed, diagnostics = monitor.wait_for_stopped(
        start=start,
        timeout=timeout,
        max_age=max_age,
        linear_epsilon=linear_epsilon,
        yaw_epsilon=yaw_epsilon,
        stop_move_ret=stop_move_ret,
        stop_move_error=stop_move_error,
        stop_move_completed_monotonic=stop_move_completed_monotonic,
        callbacks_at_stop_move_completion=callbacks_at_stop_move_completion,
    )
    result = {
        "ret": stop_move_ret,
        "stop_confirmed": stop_confirmed,
        "stop_issued_monotonic": start.monotonic,
        "stop_confirmation": diagnostics,
    }
    if stop_move_error:
        result["error"] = f"StopMove failed: {stop_move_error}"
    elif stop_move_ret != 0:
        result["error"] = f"StopMove failed, ret={stop_move_ret}"
    return result


def issue_stop_and_confirm(
    stop_move,
    monitor,
    timeout,
    max_age,
    linear_epsilon,
    yaw_epsilon,
    after_stop_attempt=None,
):
    """Issue a bounded StopMove and require a subsequent measured zero frame."""
    start = monitor.begin_confirmation()
    stop_move_ret = None
    stop_move_error = None
    try:
        stop_move_ret = stop_move()
    except Exception as exc:
        stop_move_error = str(exc)
    stop_move_completed_monotonic = time.monotonic()
    return finish_stop_confirmation(
        monitor=monitor,
        start=start,
        stop_move_ret=stop_move_ret,
        stop_move_error=stop_move_error,
        stop_move_completed_monotonic=stop_move_completed_monotonic,
        timeout=timeout,
        max_age=max_age,
        linear_epsilon=linear_epsilon,
        yaw_epsilon=yaw_epsilon,
        after_stop_attempt=after_stop_attempt,
    )


def aggregate_stop_attempts(attempts):
    """Return the final stop result without discarding earlier evidence."""
    if not attempts:
        raise ValueError("at least one stop attempt is required")
    result = dict(attempts[-1])
    result["stop_attempt_count"] = len(attempts)
    result["stop_attempts"] = [dict(attempt) for attempt in attempts]
    return result


class ProposalExecutionLease:
    """Thread-safe deadline ownership for an applied velocity proposal.

    A newly validated proposal may renew an already applied velocity while the
    serialized parent RPC is still in flight.  It must not, however, activate
    execution before the first SetVelocity call starts or revive a lease after
    the watchdog has tripped.
    """

    def __init__(self):
        self._lock = threading.Lock()
        self._generation = 0
        self._active = False
        self._deadline = 0.0
        self._fault = None

    def arm(self, deadline: float) -> Optional[int]:
        candidate = float(deadline)
        with self._lock:
            if self._fault is not None:
                return None
            self._generation += 1
            if self._active:
                self._deadline = max(self._deadline, candidate)
            else:
                self._deadline = candidate
            self._active = True
            return self._generation

    def renew_if_active(self, deadline: float) -> bool:
        """Renew an active lease; return false if a fault already won."""
        candidate = float(deadline)
        with self._lock:
            if self._fault is not None:
                return False
            if self._active:
                self._deadline = max(self._deadline, candidate)
            return True

    def clear_and_observe_fault(self):
        """Clear the lease and atomically return any fault that won first."""
        with self._lock:
            fault = self._fault
            self._generation += 1
            self._active = False
            self._deadline = 0.0
            self._fault = None
            return fault

    def clear(self) -> None:
        self.clear_and_observe_fault()

    def fault_for_generation(self, generation: int) -> str:
        with self._lock:
            if self._fault and self._fault[0] == generation:
                return self._fault[1]
            return ""

    def is_current_generation(self, generation: int) -> bool:
        """Return whether an asynchronous apply still owns the active lease."""
        with self._lock:
            return bool(
                self._active
                and self._fault is None
                and generation == self._generation
            )

    def current_fault(self):
        with self._lock:
            return self._fault

    def watchdog_snapshot(self):
        with self._lock:
            if not self._active:
                return None
            return self._generation, self._deadline

    def trip(self, generation: int, reason: str, now: float) -> bool:
        """Atomically trip only if the observed lease is still due/faulted."""
        with self._lock:
            if not self._active or generation != self._generation:
                return False
            if reason == "proposal_ttl_expired" and now < self._deadline:
                return False
            self._active = False
            self._fault = (generation, reason)
            return True


class ProposalWatchdogTransition:
    """Serialize watchdog trip/stop dispatch against execution cleanup."""

    def __init__(self, lease: ProposalExecutionLease):
        self._lease = lease
        self._lock = threading.Lock()

    def clear_and_observe_fault(self):
        """Wait for an already-tripped stop dispatch before clearing."""
        with self._lock:
            return self._lease.clear_and_observe_fault()

    def trip_and_dispatch_stop(
        self,
        generation: int,
        reason: str,
        now: float,
        dispatch_stop,
    ) -> bool:
        """Dispatch the physical stop before cleanup may cross generations."""
        with self._lock:
            if not self._lease.trip(generation, reason, now):
                return False
            dispatch_stop()
            return True


def proposal_apply_result_is_current(
    command,
    proposal_gate,
    proposal_execution_lease,
    generation,
) -> bool:
    """Reject an RPC completion superseded by stop, task, or sequence change."""
    return bool(
        proposal_execution_lease.is_current_generation(generation)
        and proposal_gate.armed
        and command.get("nav_id") == proposal_gate.expected_nav_id
        and command.get("sequence") == proposal_gate.last_sequence
    )


def velocity_commands_differ(current, target, abs_tol: float = 1e-9) -> bool:
    """Return whether a safety update changes the effective velocity."""
    return any(
        not math.isclose(
            float(before),
            float(after),
            rel_tol=0.0,
            abs_tol=abs_tol,
        )
        for before, after in zip(current, target)
    )


def put_latest(command_queue, command) -> bool:
    """Insert one command and replace, rather than backlog, an older one.

    Returns true when a queued command was coalesced away.  The consumer may
    race with replacement, but a capacity-one queue can never retain more than
    the newest command available at the replacement boundary.
    """
    coalesced = False
    while True:
        try:
            command_queue.put_nowait(command)
            return coalesced
        except queue.Full:
            try:
                command_queue.get_nowait()
                coalesced = True
            except queue.Empty:
                pass


def percentile_ms(samples, percentile):
    """Return a nearest-rank latency percentile for diagnostics."""
    if not samples:
        return None
    ordered = sorted(int(value) for value in samples)
    rank = max(1, math.ceil(float(percentile) * len(ordered)))
    return ordered[min(len(ordered) - 1, rank - 1)]


@dataclass
class ProposalApplyDiagnostics:
    """Current or most recent nav-lease execution evidence for ``loco info``."""

    session_nav_id: Optional[str] = None
    received: int = 0
    accepted: int = 0
    rejected: int = 0
    applied: int = 0
    coalesced: int = 0
    first_received_monotonic: Optional[float] = None
    last_received_monotonic: Optional[float] = None
    last_receive_gap_ms: Optional[int] = None
    max_receive_gap_ms: Optional[int] = None
    last_proposal_age_ms: Optional[int] = None
    max_proposal_age_ms: Optional[int] = None
    expired_on_arrival: int = 0
    watchdog_faults_by_reason: dict = field(default_factory=dict)
    first_rejection_reason: Optional[str] = None
    last_rejection_reason: Optional[str] = None
    rejections_by_reason: dict = field(default_factory=dict)
    last_set_velocity_rpc_method: Optional[str] = None
    last_set_velocity_request: Optional[dict] = None
    last_set_velocity_ret: Optional[int] = None
    last_set_velocity_error: Optional[str] = None
    last_set_velocity_applied: Optional[bool] = None
    last_set_velocity_request_id: Optional[int] = None
    last_set_velocity_duration_ms: Optional[int] = None
    last_set_velocity_queue_delay_ms: Optional[int] = None
    last_set_velocity_end_to_end_ms: Optional[int] = None
    last_set_velocity_completed_monotonic: Optional[float] = None
    last_set_velocity_completed_unix_ms: Optional[int] = None
    last_set_velocity_stop_ret: Optional[int] = None
    last_set_velocity_stop_error: Optional[str] = None
    _lock: threading.RLock = field(
        default_factory=threading.RLock,
        init=False,
        repr=False,
    )
    _applied_identities: set = field(
        default_factory=set,
        init=False,
        repr=False,
    )
    _set_velocity_rpc_durations_ms: deque = field(
        default_factory=lambda: deque(maxlen=512),
        init=False,
        repr=False,
    )

    def begin_session(self, nav_id: Optional[str]) -> None:
        """Reset per-lease counters while retaining them after later cleanup."""
        with self._lock:
            self.session_nav_id = nav_id
            self.received = 0
            self.accepted = 0
            self.rejected = 0
            self.applied = 0
            self.coalesced = 0
            self.first_received_monotonic = None
            self.last_received_monotonic = None
            self.last_receive_gap_ms = None
            self.max_receive_gap_ms = None
            self.last_proposal_age_ms = None
            self.max_proposal_age_ms = None
            self.expired_on_arrival = 0
            self.watchdog_faults_by_reason = {}
            self.first_rejection_reason = None
            self.last_rejection_reason = None
            self.rejections_by_reason = {}
            self.last_set_velocity_rpc_method = None
            self.last_set_velocity_request = None
            self.last_set_velocity_ret = None
            self.last_set_velocity_error = None
            self.last_set_velocity_applied = None
            self.last_set_velocity_request_id = None
            self.last_set_velocity_duration_ms = None
            self.last_set_velocity_queue_delay_ms = None
            self.last_set_velocity_end_to_end_ms = None
            self.last_set_velocity_completed_monotonic = None
            self.last_set_velocity_completed_unix_ms = None
            self.last_set_velocity_stop_ret = None
            self.last_set_velocity_stop_error = None
            self._applied_identities = set()
            self._set_velocity_rpc_durations_ms = deque(maxlen=512)

    def bind_session_nav_id(self, nav_id: str) -> None:
        """Attach a first-proposal lease without resetting arrival evidence."""
        with self._lock:
            if self.session_nav_id is None:
                self.session_nav_id = str(nav_id)

    def bind_claimed_session(
        self,
        nav_id: str,
        received_monotonic: float,
        payload,
        received_unix_ms: float,
    ) -> bool:
        """Bind an auto-claimed lease and roll counters on later tasks.

        The first claim preserves invalid/expired packets observed while the
        Canvas connection was initially waiting.  A later claim starts a new
        per-lease diagnostic session and restores the claiming packet as its
        first arrival evidence.
        """
        resolved = str(nav_id)
        with self._lock:
            if self.session_nav_id is None:
                self.session_nav_id = resolved
                return False
            if self.session_nav_id == resolved:
                return False
            # Keep rollover atomic against async apply/watchdog diagnostics.
            self.begin_session(resolved)
            self.record_received(received_monotonic)
            self.record_proposal_arrival(payload, received_unix_ms)
            return True

    def record_received(self, received_monotonic: Optional[float] = None) -> None:
        received = (
            time.monotonic()
            if received_monotonic is None
            else float(received_monotonic)
        )
        with self._lock:
            self.received += 1
            if self.first_received_monotonic is None:
                self.first_received_monotonic = received
            if self.last_received_monotonic is not None:
                gap_ms = max(
                    0,
                    round((received - self.last_received_monotonic) * 1000),
                )
                self.last_receive_gap_ms = gap_ms
                if (
                    self.max_receive_gap_ms is None
                    or gap_ms > self.max_receive_gap_ms
                ):
                    self.max_receive_gap_ms = gap_ms
            self.last_received_monotonic = received

    def record_proposal_arrival(
        self,
        payload,
        received_unix_ms: float,
    ) -> None:
        """Record source-to-callback age without participating in validation."""
        if not isinstance(payload, dict):
            return
        issued_at = payload.get("issued_at_unix_ms")
        ttl_ms = payload.get("ttl_ms")
        if (
            isinstance(issued_at, bool)
            or not isinstance(issued_at, (int, float))
            or not math.isfinite(float(issued_at))
        ):
            return
        age_ms = round(float(received_unix_ms) - float(issued_at))
        with self._lock:
            self.last_proposal_age_ms = age_ms
            if (
                self.max_proposal_age_ms is None
                or age_ms > self.max_proposal_age_ms
            ):
                self.max_proposal_age_ms = age_ms
            if (
                not isinstance(ttl_ms, bool)
                and isinstance(ttl_ms, int)
                and ttl_ms > 0
                and age_ms >= ttl_ms
            ):
                self.expired_on_arrival += 1

    def record_watchdog_fault(self, reason: str) -> None:
        with self._lock:
            self.watchdog_faults_by_reason[reason] = (
                self.watchdog_faults_by_reason.get(reason, 0) + 1
            )

    def record_accepted(self) -> None:
        with self._lock:
            self.accepted += 1

    def record_coalesced(self) -> None:
        with self._lock:
            self.coalesced += 1

    def record_rejected(self, reason: str) -> None:
        with self._lock:
            self.rejected += 1
            if self.first_rejection_reason is None:
                self.first_rejection_reason = reason
            self.last_rejection_reason = reason
            self.rejections_by_reason[reason] = (
                self.rejections_by_reason.get(reason, 0) + 1
            )

    def record_set_velocity(
        self,
        result: dict,
        queued_monotonic: Optional[float] = None,
    ) -> None:
        """Record measured RPC timing, never the proposal TTL budget."""
        started_monotonic = result.get("started_monotonic")
        completed_monotonic = result.get("completed_monotonic")
        completed_unix_ms = result.get("completed_unix_ms")
        duration_ms = result.get("duration_ms")
        if (
            duration_ms is None
            and started_monotonic is not None
            and completed_monotonic is not None
        ):
            duration_ms = max(
                0,
                round(
                    (float(completed_monotonic) - float(started_monotonic))
                    * 1000
                ),
            )
        queue_delay_ms = None
        end_to_end_ms = None
        if queued_monotonic is not None and started_monotonic is not None:
            queue_delay_ms = max(
                0,
                round(
                    (float(started_monotonic) - float(queued_monotonic))
                    * 1000
                ),
            )
        if queued_monotonic is not None and completed_monotonic is not None:
            end_to_end_ms = max(
                0,
                round(
                    (float(completed_monotonic) - float(queued_monotonic))
                    * 1000
                ),
            )
        with self._lock:
            self.last_set_velocity_rpc_method = result.get("rpc_method")
            request = result.get("request")
            self.last_set_velocity_request = (
                dict(request) if isinstance(request, dict) else None
            )
            self.last_set_velocity_ret = result.get("ret")
            self.last_set_velocity_error = result.get("error")
            self.last_set_velocity_applied = bool(result.get("applied"))
            self.last_set_velocity_request_id = result.get("request_id")
            self.last_set_velocity_duration_ms = (
                max(0, int(duration_ms))
                if duration_ms is not None
                else None
            )
            self.last_set_velocity_queue_delay_ms = queue_delay_ms
            self.last_set_velocity_end_to_end_ms = end_to_end_ms
            if duration_ms is not None:
                self._set_velocity_rpc_durations_ms.append(
                    max(0, int(duration_ms))
                )
            self.last_set_velocity_completed_monotonic = (
                float(completed_monotonic)
                if completed_monotonic is not None
                else None
            )
            self.last_set_velocity_completed_unix_ms = (
                int(completed_unix_ms)
                if completed_unix_ms is not None
                else None
            )
            self.last_set_velocity_stop_ret = result.get("stop_ret")
            self.last_set_velocity_stop_error = result.get("stop_error")

    def record_applied(
        self,
        nav_id: Optional[str] = None,
        sequence: Optional[int] = None,
    ) -> None:
        with self._lock:
            if nav_id is not None and sequence is not None:
                identity = (str(nav_id), int(sequence))
                if identity in self._applied_identities:
                    return
                self._applied_identities.add(identity)
            self.applied += 1

    def snapshot(self) -> dict:
        with self._lock:
            return {
                "session_nav_id": self.session_nav_id,
                "received": self.received,
                "accepted": self.accepted,
                "rejected": self.rejected,
                "applied": self.applied,
                "coalesced": self.coalesced,
                "first_received_monotonic": self.first_received_monotonic,
                "last_received_monotonic": self.last_received_monotonic,
                "last_receive_gap_ms": self.last_receive_gap_ms,
                "max_receive_gap_ms": self.max_receive_gap_ms,
                "last_proposal_age_ms": self.last_proposal_age_ms,
                "max_proposal_age_ms": self.max_proposal_age_ms,
                "expired_on_arrival": self.expired_on_arrival,
                "watchdog_faults_by_reason": dict(
                    self.watchdog_faults_by_reason
                ),
                "first_rejection_reason": self.first_rejection_reason,
                "last_rejection_reason": self.last_rejection_reason,
                "rejections_by_reason": dict(self.rejections_by_reason),
                "last_set_velocity_rpc_method": (
                    self.last_set_velocity_rpc_method
                ),
                "last_set_velocity_request": (
                    dict(self.last_set_velocity_request)
                    if self.last_set_velocity_request is not None
                    else None
                ),
                "last_set_velocity_ret": self.last_set_velocity_ret,
                "last_set_velocity_error": self.last_set_velocity_error,
                "last_set_velocity_applied": self.last_set_velocity_applied,
                "last_set_velocity_request_id": (
                    self.last_set_velocity_request_id
                ),
                "last_set_velocity_duration_ms": (
                    self.last_set_velocity_duration_ms
                ),
                "last_set_velocity_queue_delay_ms": (
                    self.last_set_velocity_queue_delay_ms
                ),
                "last_set_velocity_end_to_end_ms": (
                    self.last_set_velocity_end_to_end_ms
                ),
                "set_velocity_rpc_samples": len(
                    self._set_velocity_rpc_durations_ms
                ),
                "set_velocity_rpc_p50_ms": percentile_ms(
                    self._set_velocity_rpc_durations_ms,
                    0.50,
                ),
                "set_velocity_rpc_p95_ms": percentile_ms(
                    self._set_velocity_rpc_durations_ms,
                    0.95,
                ),
                "set_velocity_rpc_p99_ms": percentile_ms(
                    self._set_velocity_rpc_durations_ms,
                    0.99,
                ),
                "set_velocity_rpc_max_ms": (
                    max(self._set_velocity_rpc_durations_ms)
                    if self._set_velocity_rpc_durations_ms
                    else None
                ),
                "last_set_velocity_completed_monotonic": (
                    self.last_set_velocity_completed_monotonic
                ),
                "last_set_velocity_completed_unix_ms": (
                    self.last_set_velocity_completed_unix_ms
                ),
                "last_set_velocity_stop_ret": self.last_set_velocity_stop_ret,
                "last_set_velocity_stop_error": (
                    self.last_set_velocity_stop_error
                ),
            }


def _request_parent_loco_rpc(
    request_queue,
    result_queue,
    request: dict,
    timeout: float,
    timeout_error: str,
    ipc_error_prefix: str,
) -> dict:
    """Send one correlated parent Loco RPC request and wait boundedly."""
    started = time.monotonic()
    request_id = request["request_id"]
    try:
        request_queue.put(request)
    except Exception as exc:
        return {
            "request_id": request_id,
            "ret": None,
            "error": f"{ipc_error_prefix}: {exc}",
            "completed_monotonic": time.monotonic(),
            "completed_unix_ms": round(time.time() * 1000),
        }

    deadline = started + max(0.0, float(timeout))
    while True:
        remaining = deadline - time.monotonic()
        if remaining <= 0.0:
            return {
                "request_id": request_id,
                "ret": None,
                "error": timeout_error,
                "completed_monotonic": time.monotonic(),
                "completed_unix_ms": round(time.time() * 1000),
            }
        try:
            result = result_queue.get(timeout=remaining)
        except queue.Empty:
            return {
                "request_id": request_id,
                "ret": None,
                "error": timeout_error,
                "completed_monotonic": time.monotonic(),
                "completed_unix_ms": round(time.time() * 1000),
            }
        if result.get("request_id") == request_id:
            return result


def request_parent_apply_velocity_proposal(
    request_queue,
    result_queue,
    request_id: int,
    vx: float,
    vy: float,
    vyaw: float,
    deadline_monotonic: float,
    nav_id: str,
    sequence: int,
    timeout: float,
) -> dict:
    """Request one Driver-deadline-controlled velocity from the parent."""
    request = {
        "vx": vx,
        "vy": vy,
        "vyaw": vyaw,
        "deadline_monotonic": deadline_monotonic,
        "nav_id": nav_id,
        "sequence": sequence,
    }
    result = _request_parent_loco_rpc(
        request_queue,
        result_queue,
        request={
            "method": "apply_velocity_proposal",
            "request_id": request_id,
            **request,
        },
        timeout=timeout,
        timeout_error="parent_velocity_proposal_timeout",
        ipc_error_prefix="parent_velocity_proposal_ipc_error",
    )
    result.setdefault("rpc_method", "SetVelocity(continuous)")
    result.setdefault("request", request)
    result.setdefault("applied", False)
    return result


def request_parent_stop_velocity_proposal(
    request_queue,
    result_queue,
    request_id: int,
    timeout: float,
) -> dict:
    """Order StopMove after every earlier parent proposal RPC."""
    return _request_parent_loco_rpc(
        request_queue,
        result_queue,
        request={
            "method": "stop_velocity_proposal",
            "request_id": request_id,
        },
        timeout=timeout,
        timeout_error="parent_velocity_stop_timeout",
        ipc_error_prefix="parent_velocity_stop_ipc_error",
    )


def request_parent_get_fsm_id(
    request_queue,
    result_queue,
    request_id: int,
    timeout: float,
) -> dict:
    """Request one fail-closed GetFsmId sample from the parent RpcProxy."""
    return _request_parent_loco_rpc(
        request_queue,
        result_queue,
        request={
            "method": "get_fsm_id",
            "request_id": request_id,
        },
        timeout=timeout,
        timeout_error="parent_get_fsm_id_timeout",
        ipc_error_prefix="parent_get_fsm_id_ipc_error",
    )


# ── SmartMotionProxy (main process) ─────────────────────────────────────────

class SmartMotionProxy:
    """Main-process proxy that communicates with the SmartMotion subprocess."""

    def __init__(self, namespace: str, config: dict, network_iface: str,
                 proposal_config: Optional[dict] = None,
                 fallback_stop=None, parent_apply_velocity_proposal=None,
                 parent_get_fsm_id=None):
        ctx = mp.get_context("spawn")
        self._cmd_queue = ctx.Queue()
        self._result_queue = ctx.Queue()
        self._parent_velocity_queue = ctx.Queue()
        self._parent_velocity_result_queue = ctx.Queue()
        self._call_lock = threading.RLock()
        self._request_id = 0
        self._fallback_stop = fallback_stop
        self._parent_apply_velocity_proposal = parent_apply_velocity_proposal
        self._parent_get_fsm_id = parent_get_fsm_id
        self._parent_velocity_shutdown = threading.Event()
        self._proc = ctx.Process(
            target=_run_smart_motion_process,
            args=(namespace, config, proposal_config or {}, network_iface,
                  self._cmd_queue, self._result_queue,
                  self._parent_velocity_queue,
                  self._parent_velocity_result_queue),
            name="smart_motion", daemon=True,
        )
        self._proc.start()
        self._parent_velocity_thread = threading.Thread(
            target=self._serve_parent_velocity_requests,
            daemon=True,
            name="smart_motion_parent_velocity",
        )
        self._parent_velocity_thread.start()
        self._exit_monitor = threading.Thread(
            target=self._monitor_process_exit,
            daemon=True,
            name="smart_motion_exit_monitor",
        )
        self._exit_monitor.start()
        print(f"[SmartMotionProxy] subprocess started → pid={self._proc.pid}")

    def _serve_parent_velocity_requests(self) -> None:
        """Execute child-approved Loco calls on the parent RpcProxy."""
        while True:
            request = self._parent_velocity_queue.get()
            if request is None:
                return
            request_id = request.get("request_id")
            method = request.get("method", "apply_velocity_proposal")
            ret = None
            fsm_id = None
            error = None
            response = {}
            try:
                if method == "apply_velocity_proposal":
                    if self._parent_apply_velocity_proposal is None:
                        raise RuntimeError(
                            "parent proposal RPC is unavailable"
                        )
                    result = self._parent_apply_velocity_proposal(
                        request["vx"],
                        request["vy"],
                        request["vyaw"],
                        request["deadline_monotonic"],
                        request["nav_id"],
                        request["sequence"],
                        request_id,
                    )
                    if not isinstance(result, dict):
                        raise RuntimeError(
                            "parent proposal RPC returned an invalid result"
                        )
                    response.update(result)
                    ret = result.get("ret")
                    error = result.get("error")
                elif method == "get_fsm_id":
                    if self._parent_get_fsm_id is None:
                        raise RuntimeError(
                            "parent GetFsmId RPC is unavailable"
                        )
                    result = self._parent_get_fsm_id()
                    if not isinstance(result, (list, tuple)) or len(result) != 2:
                        raise RuntimeError(
                            "parent GetFsmId returned an invalid result"
                        )
                    ret, fsm_id = result
                elif method == "stop_velocity_proposal":
                    if self._fallback_stop is None:
                        raise RuntimeError(
                            "parent StopMove RPC is unavailable"
                        )
                    ret = self._fallback_stop()
                else:
                    raise RuntimeError(
                        f"unsupported parent Loco RPC method: {method}"
                    )
            except Exception as exc:
                error = str(exc)
            response.update({
                "method": method,
                "request_id": request_id,
                "ret": ret,
                "fsm_id": fsm_id,
                "error": error,
            })
            response.setdefault("completed_monotonic", time.monotonic())
            response.setdefault("completed_unix_ms", round(time.time() * 1000))
            self._parent_velocity_result_queue.put(response)

    def _signal_parent_velocity_shutdown(self) -> None:
        shutdown = getattr(self, "_parent_velocity_shutdown", None)
        request_queue = getattr(self, "_parent_velocity_queue", None)
        if shutdown is None or request_queue is None or shutdown.is_set():
            return
        shutdown.set()
        request_queue.put(None)

    def _monitor_process_exit(self) -> None:
        """Issue an independent parent-side stop on every child exit.

        The child normally stops itself in its cleanup path.  This second path
        covers unhandled exceptions, SIGKILL/SIGTERM, and forced termination
        after an IPC timeout, where Python cleanup cannot be relied upon.
        """
        self._proc.join()
        try:
            if self._fallback_stop is not None:
                self._fallback_stop()
        except Exception as exc:
            print(
                f"[SmartMotionProxy] child-exit StopMove failed: {exc}",
                flush=True,
            )
        finally:
            self._signal_parent_velocity_shutdown()

    def _call(self, method: str, timeout: float = 15.0, **kwargs) -> dict:
        """Serialize calls and correlate replies across HTTP/ROS threads."""
        with self._call_lock:
            if not self._proc.is_alive():
                return {"error": "SmartMotion subprocess is not running"}
            self._request_id += 1
            request_id = self._request_id
            self._cmd_queue.put({
                "method": method,
                "request_id": request_id,
                **kwargs,
            })
            deadline = time.monotonic() + timeout
            while True:
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    self._terminate_unresponsive_process()
                    return {"error": f"SmartMotion subprocess timeout ({method})"}
                try:
                    envelope = self._result_queue.get(timeout=remaining)
                except queue.Empty:
                    self._terminate_unresponsive_process()
                    return {"error": f"SmartMotion subprocess timeout ({method})"}
                if envelope.get("request_id") == request_id:
                    return envelope.get("result") or {}

    def _terminate_unresponsive_process(self) -> None:
        """Fail closed: an IPC-hung motion owner may not keep executing."""
        if self._proc.is_alive():
            self._proc.terminate()
            self._proc.join(timeout=1.0)

    def _call_with_parent_stop(self, method: str, **kwargs) -> dict:
        """Bracket a parent-side StopMove with child odometry confirmation."""
        with self._call_lock:
            prepared = self._call("begin_velocity_proposal_stop_confirmation")
            confirmation_start = prepared.get("confirmation_start")
            stop_move_ret = None
            stop_move_error = None
            try:
                if self._fallback_stop is None:
                    raise RuntimeError("parent StopMove RPC is unavailable")
                stop_move_ret = self._fallback_stop()
            except Exception as exc:
                stop_move_error = str(exc)
            stop_move_completed_monotonic = time.monotonic()

            if not confirmation_start:
                return {
                    "error": prepared.get("error")
                    or "SmartMotion stop confirmation did not start",
                    "stop_confirmed": False,
                    "stop_move_ret": stop_move_ret,
                    "stop_move_error": stop_move_error,
                }
            return self._call(
                method,
                external_stop_attempt={
                    "confirmation_start": confirmation_start,
                    "stop_move_ret": stop_move_ret,
                    "stop_move_error": stop_move_error,
                    "stop_move_completed_monotonic": (
                        stop_move_completed_monotonic
                    ),
                },
                **kwargs,
            )

    def move(self, vx: float, vy: float, vyaw: float, duration: float = -1.0) -> dict:
        return self._call("move", vx=vx, vy=vy, vyaw=vyaw, duration=duration)

    def stop(self, reason: str = "command") -> dict:
        return self._call("stop", reason=reason)

    def navigate_to(self, x: float, y: float, yaw: float, target_name: str = "",
                    speed: float = 0.5, mode: int = 1) -> dict:
        return self._call("navigate_to", x=x, y=y, yaw=yaw, target_name=target_name,
                          speed=speed, mode=mode)

    def pause_nav(self, reason: str = "command") -> dict:
        return self._call("pause_nav", reason=reason)

    def resume_nav(self) -> dict:
        return self._call("resume_nav")

    def stop_nav(self) -> dict:
        return self._call("stop_nav")

    def wait_nav_done(self, stall_timeout: float = 60) -> dict:
        return self._call("wait_nav_done", stall_timeout=stall_timeout,
                          timeout=stall_timeout + 30)

    def get_state(self) -> dict:
        return self._call("get_state")

    def bind_velocity_proposal(
        self,
        topic: str,
        expected_nav_id: Optional[str] = None,
    ) -> dict:
        return self._call_with_parent_stop(
            "bind_velocity_proposal",
            topic=topic,
            expected_nav_id=expected_nav_id,
        )

    def unbind_velocity_proposal(self, reason: str = "canvas_stop") -> dict:
        return self._call_with_parent_stop(
            "unbind_velocity_proposal",
            reason=reason,
        )

    def get_velocity_proposal_status(self) -> dict:
        return self._call("get_velocity_proposal_status")

    # ── SLAM RPC passthrough (single SlamClient owns all SLAM operations) ──

    def start_mapping(self) -> dict:
        return self._call("start_mapping", timeout=15.0)

    def stop_mapping(self, pcd_path: str) -> dict:
        return self._call("stop_mapping", pcd_path=pcd_path, timeout=15.0)

    def init_pose(self, x=0.0, y=0.0, z=0.0, q_x=0.0, q_y=0.0, q_z=0.0, q_w=1.0,
                  address="") -> dict:
        return self._call("init_pose", x=x, y=y, z=z, q_x=q_x, q_y=q_y,
                          q_z=q_z, q_w=q_w, address=address, timeout=15.0)

    def shutdown(self) -> None:
        try:
            self._call("shutdown", timeout=3.0)
            self._proc.join(timeout=3.0)
        except Exception:
            pass
        if self._proc.is_alive():
            self._proc.terminate()
            self._proc.join(timeout=2.0)
        # Ensure the parent-side child-exit StopMove completes before the
        # caller tears down the independent RpcProxy used by that callback.
        self._exit_monitor.join(timeout=2.0)
        self._signal_parent_velocity_shutdown()
        self._parent_velocity_thread.join(timeout=2.0)
        print("[SmartMotionProxy] subprocess stopped")


# ── SmartMotion subprocess entry ─────────────────────────────────────────────

def _run_smart_motion_process(namespace: str, config: dict, proposal_config: dict,
                              network_iface: str, cmd_queue: mp.Queue,
                              result_queue: mp.Queue,
                              parent_velocity_queue: mp.Queue,
                              parent_velocity_result_queue: mp.Queue):
    """Entry point for the SmartMotion subprocess.

    Initializes its own DDS channel, RPC clients, ROS2 node, and LiDAR subscription.
    Runs independently from the main driver process — no GIL contention.
    """
    import numpy as np
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
    from rclpy.executors import SingleThreadedExecutor
    from std_msgs.msg import String, UInt8MultiArray

    from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
    from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient
    from unitree_sdk2py.g1.slam.slam_client import SlamClient

    _QOS = QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
        depth=200,
        durability=DurabilityPolicy.VOLATILE,
    )
    _PROPOSAL_QOS = QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
        durability=DurabilityPolicy.VOLATILE,
    )

    # ── Initialize DDS ──
    ChannelFactoryInitialize(0, network_iface)
    print(f"[SmartMotion:pid={os.getpid()}] DDS initialized on {network_iface}")

    # ── Initialize RPC clients ──
    loco_client = LocoClient()
    loco_client.Init()
    loco_client.SetTimeout(10.0)

    # Proposal execution and its FSM authorization are deliberately routed
    # through the parent Driver RpcProxy. Stopping and watchdog stopping keep
    # independent child clients so a delayed parent RPC cannot block the
    # fail-closed stop paths.
    proposal_rpc_timeout = min(
        1.0,
        max(
            0.1,
            float(
                proposal_config.get("velocity_proposal_rpc_timeout", 0.5)
            ),
        ),
    )
    stop_rpc_timeout = min(
        0.2,
        max(
            0.02,
            float(
                proposal_config.get(
                    "velocity_proposal_stop_rpc_timeout",
                    0.1,
                )
            ),
        ),
    )
    fsm_rpc_timeout = min(
        0.5, max(0.05, float(proposal_config.get("velocity_proposal_fsm_rpc_timeout", 0.2)))
    )
    stop_loco_client = LocoClient()
    stop_loco_client.Init()
    stop_loco_client.SetTimeout(stop_rpc_timeout)
    watchdog_loco_client = LocoClient()
    watchdog_loco_client.Init()
    watchdog_loco_client.SetTimeout(stop_rpc_timeout)
    slam_client = SlamClient()
    slam_client.Init()
    slam_client.SetTimeout(10.0)  # must be AFTER Init() — Init() resets timeout to 5.0

    print(
        f"[SmartMotion:pid={os.getpid()}] stop LocoClient + "
        "parent proposal/FSM RPC + SlamClient ready"
    )

    # ── Initialize ROS2 ──
    rclpy.init()
    executor = SingleThreadedExecutor()

    class _EventNode(Node):
        def __init__(self):
            super().__init__("g1_safety_harness")
            self._pub = self.create_publisher(
                String, f"/{namespace}/safety/motion_events", _QOS
            )

        def publish(self, event: dict):
            msg = String()
            msg.data = json.dumps(event)
            self._pub.publish(msg)

    event_node = _EventNode()
    executor.add_node(event_node)
    print(f"[SmartMotion:pid={os.getpid()}] ROS2 event node ready → /{namespace}/safety/motion_events")

    class _VelocityProposalNode(Node):
        """Own the one dynamically connected N5 proposal subscription."""

        def __init__(self):
            super().__init__("g1_loco_velocity_proposal")
            self._subscription = None
            self.topic = ""

        def bind(self, topic: str, callback) -> None:
            self.unbind()
            self._subscription = self.create_subscription(
                String, topic, callback, _PROPOSAL_QOS
            )
            self.topic = topic

        def unbind(self) -> None:
            if self._subscription is not None:
                self.destroy_subscription(self._subscription)
                self._subscription = None
            self.topic = ""

    proposal_node = _VelocityProposalNode()
    executor.add_node(proposal_node)

    # ── Config ──
    decel_threshold = config.get("decel_threshold", 2.0)
    stop_threshold = config.get("stop_threshold", 0.8)
    cone_half_angle = math.radians(config.get("cone_half_angle", 30))
    z_min = config.get("z_min", 0.1)
    z_max = config.get("z_max", 1.8)
    limits = SpeedLimits()
    proposal_enabled = bool(proposal_config.get("velocity_proposal_enabled", True))
    proposal_driver_authorized = bool(
        proposal_config.get("velocity_proposal_driver_authorized", False)
    )
    expected_proposal_topic = DEFAULT_VELOCITY_PROPOSAL_TOPIC
    # Deployment configuration may tighten, but never loosen, Driver limits.
    proposal_limits = resolve_proposal_limits(proposal_config)
    proposal_gate = VelocityProposalGate(proposal_limits)
    proposal_odom_timeout = float(proposal_config.get("velocity_proposal_odom_timeout", 0.5))
    proposal_scan_timeout = float(proposal_config.get("velocity_proposal_scan_timeout", 0.5))
    proposal_fsm_check_interval = float(
        proposal_config.get("velocity_proposal_fsm_check_interval", 0.2)
    )
    proposal_fsm_timeout = max(
        proposal_fsm_check_interval,
        float(proposal_config.get("velocity_proposal_fsm_timeout", 0.5)),
    )
    proposal_nav_status_timeout = min(
        proposal_limits.max_ttl_ms / 1000.0,
        float(proposal_config.get("velocity_proposal_nav_status_timeout", 0.25)),
    )
    proposal_watchdog_hz = max(
        20.0, float(proposal_config.get("velocity_proposal_watchdog_hz", 40.0))
    )
    proposal_stop_confirm_timeout = resolve_stop_confirmation_timeout(
        proposal_config.get(
            "velocity_proposal_stop_confirm_timeout",
            DEFAULT_STOP_CONFIRM_TIMEOUT_SECONDS,
        )
    )
    proposal_stop_linear_epsilon = max(
        0.0, float(proposal_config.get("velocity_proposal_stop_linear_epsilon", 0.03))
    )
    proposal_stop_yaw_epsilon = max(
        0.0, float(proposal_config.get("velocity_proposal_stop_yaw_epsilon", 0.05))
    )
    proposal_allowed_fsm_ids = {
        int(value)
        for value in proposal_config.get("velocity_proposal_allowed_fsm_ids", [500, 801])
    }

    # ── State ──
    state = MotionState.IDLE
    current_cmd = None  # dict: {vx, vy, vyaw, duration, start_time, end_time}
    nav_cmd = None      # dict: {target_name, target_pose, start_time}
    speed_zone = SpeedZone.NORMAL
    move_timer = None   # threading.Timer
    last_cloud_time = 0.0
    last_fsm_check = 0.0
    last_fsm_id = None
    last_fsm_ret = None
    last_fsm_error = None
    main_control_ready = False
    last_nav_status_time = 0.0
    odom_stop_monitor = OdomStopMonitor()
    safety_lock = threading.Lock()
    motion_command_lock = threading.RLock()
    proposal_execution_lease = ProposalExecutionLease()
    proposal_watchdog_transition = ProposalWatchdogTransition(
        proposal_execution_lease
    )
    proposal_connected = threading.Event()
    proposal_stop_transition = threading.Event()
    safety_threads_shutdown = threading.Event()
    proposal_callback_ready = threading.Event()
    proposal_callback_failed = threading.Event()
    proposal_callback_error_lock = threading.Lock()
    proposal_callback_error = None
    proposal_ros_thread = None
    parent_loco_request_id = 0
    parent_loco_rpc_lock = threading.Lock()
    proposal_apply_diagnostics = ProposalApplyDiagnostics()
    proposal_apply_queue = queue.Queue(maxsize=1)
    last_stop_result = None
    last_proposal_stop_result = None

    def motion_synchronized(func):
        def locked(*args, **kwargs):
            with motion_command_lock:
                return func(*args, **kwargs)
        return locked

    def proposal_callback_status():
        with proposal_callback_error_lock:
            error = proposal_callback_error
        return {
            "ready": proposal_callback_ready.is_set(),
            "alive": bool(
                proposal_ros_thread is not None
                and proposal_ros_thread.is_alive()
            ),
            "failed": proposal_callback_failed.is_set(),
            "error": error,
        }

    def arm_proposal_execution(deadline):
        return proposal_execution_lease.arm(deadline)

    def refresh_proposal_execution_deadline(deadline):
        return proposal_execution_lease.renew_if_active(deadline)

    def clear_proposal_execution():
        return proposal_watchdog_transition.clear_and_observe_fault()

    def proposal_fault_for_generation(generation):
        return proposal_execution_lease.fault_for_generation(generation)

    def current_proposal_watchdog_fault():
        return proposal_execution_lease.current_fault()

    def apply_parent_velocity_proposal(command):
        nonlocal parent_loco_request_id
        # The FSM monitor and proposal callback share one request/result pair.
        # Serialize them so one waiter cannot consume the other's reply.
        with parent_loco_rpc_lock:
            parent_loco_request_id += 1
            result = request_parent_apply_velocity_proposal(
                request_queue=parent_velocity_queue,
                result_queue=parent_velocity_result_queue,
                request_id=parent_loco_request_id,
                vx=command["vx"],
                vy=command["vy"],
                vyaw=command["vyaw"],
                deadline_monotonic=command["deadline_monotonic"],
                nav_id=command["nav_id"],
                sequence=command["sequence"],
                timeout=proposal_rpc_timeout,
            )
        return result

    def stop_parent_velocity_proposal():
        """Serialize StopMove behind any in-flight parent proposal apply."""
        nonlocal parent_loco_request_id
        with parent_loco_rpc_lock:
            parent_loco_request_id += 1
            return request_parent_stop_velocity_proposal(
                request_queue=parent_velocity_queue,
                result_queue=parent_velocity_result_queue,
                request_id=parent_loco_request_id,
                timeout=proposal_rpc_timeout,
            )

    def clear_proposal_apply_queue():
        while True:
            try:
                proposal_apply_queue.get_nowait()
            except queue.Empty:
                return

    def enqueue_proposal_apply(command):
        """Keep only the newest validated command while the parent RPC is busy."""
        if put_latest(proposal_apply_queue, command):
            proposal_apply_diagnostics.record_coalesced()

    def query_parent_fsm_id():
        nonlocal parent_loco_request_id
        with parent_loco_rpc_lock:
            parent_loco_request_id += 1
            return request_parent_get_fsm_id(
                request_queue=parent_velocity_queue,
                result_queue=parent_velocity_result_queue,
                request_id=parent_loco_request_id,
                timeout=fsm_rpc_timeout,
            )

    # Obstacle state (written by LiDAR callback, read by main loop)
    obstacle_lock = threading.Lock()
    obstacle_dist = float("inf")
    obstacle_angle = 0.0
    lateral_obstacle = False
    obstacle_scan_heading = None

    # Nav arrival state (updated by rt/slam_info DDS callback in this subprocess)
    slam_info_lock = threading.Lock()
    nav_arrived_flag = False
    nav_arrived_error = None
    nav_current_pose = None

    # ── Helpers ──
    def clamp(vx, vy, vyaw, zone):
        if zone == SpeedZone.DECELERATED:
            vx = max(-limits.vx_decel, min(limits.vx_decel, vx))
            vy = max(-limits.vy_decel, min(limits.vy_decel, vy))
            vyaw = max(-limits.vyaw_decel, min(limits.vyaw_decel, vyaw))
        else:
            vx = max(-limits.vx_max, min(limits.vx_max, vx))
            vy = max(-limits.vy_max, min(limits.vy_max, vy))
            vyaw = max(-limits.vyaw_max, min(limits.vyaw_max, vyaw))
        return vx, vy, vyaw

    def publish_event(event_type, data):
        event = {"type": event_type, "timestamp": time.time(), **data}
        event_node.publish(event)
        print(f"[SmartMotion] event: {event_type} | {json.dumps(data)}", flush=True)

    def issue_stop_move():
        """Call StopMove and return the Unitree RPC acknowledgement code."""
        return stop_loco_client.StopMove()

    @motion_synchronized
    def do_stop(
        reason_str,
        confirm_physical_stop=False,
        external_stop_attempt=None,
    ):
        nonlocal state, current_cmd, speed_zone, move_timer, stop_repeat_count
        nonlocal last_stop_result, last_proposal_stop_result
        was_proposal_motion = bool(
            current_cmd and current_cmd.get("source") == "velocity_proposal"
        )
        if move_timer:
            move_timer.cancel()
            move_timer = None
        clear_proposal_apply_queue()
        # Linearize terminal/local stop against the independent watchdog.  If
        # clear wins, a stale generation can no longer trip.  If trip wins,
        # preserve that fault so this stop cannot authorize a new lease while
        # the watchdog's physical StopMove may still be in flight.
        watchdog_fault = clear_proposal_execution()
        reason_str = authoritative_proposal_stop_reason(
            reason_str,
            watchdog_fault,
        )
        if watchdog_fault and (
            reason_str != "proposal_ttl_expired" or not proposal_gate.armed
        ):
            proposal_gate.disarm(reason_str)
        recoverable_stop_requested = bool(
            proposal_gate.armed
            and reason_str in {"obstacle", "proposal_ttl_expired"}
            and (was_proposal_motion or reason_str == "proposal_ttl_expired")
        )
        proposal_stop_context = bool(
            was_proposal_motion or recoverable_stop_requested
        )
        retry_proposal_stop = (
            proposal_stop_context and external_stop_attempt is None
        )
        was_moving = state == MotionState.MOVING

        if proposal_stop_context and external_stop_attempt is None:
            # The independent client brakes immediately.  The ordered parent
            # StopMove then runs after any in-flight continuous SetVelocity,
            # preventing a late successful apply from restarting motion.
            start = odom_stop_monitor.begin_confirmation()
            try:
                watchdog_loco_client.StopMove()
            except Exception:
                pass
            parent_stop = stop_parent_velocity_proposal()
            external_stop_attempt = {
                "confirmation_start": {
                    "monotonic": start.monotonic,
                    "unix_ms": start.unix_ms,
                    "odometry_callback_count": start.odometry_callback_count,
                },
                "stop_move_ret": parent_stop.get("ret"),
                "stop_move_error": parent_stop.get("error"),
                "stop_move_completed_monotonic": parent_stop.get(
                    "completed_monotonic",
                    time.monotonic(),
                ),
            }
            confirm_physical_stop = True

        def complete_logical_stop():
            nonlocal state, current_cmd, speed_zone, stop_repeat_count
            state = MotionState.IDLE
            current_cmd = None
            speed_zone = SpeedZone.NORMAL
            stop_repeat_count = 3  # repeat StopMove to ensure controller stops

        if confirm_physical_stop:
            if external_stop_attempt is not None:
                start_data = external_stop_attempt["confirmation_start"]
                result = finish_stop_confirmation(
                    monitor=odom_stop_monitor,
                    start=StopConfirmationStart(
                        monotonic=float(start_data["monotonic"]),
                        unix_ms=int(start_data["unix_ms"]),
                        odometry_callback_count=int(
                            start_data["odometry_callback_count"]
                        ),
                    ),
                    stop_move_ret=external_stop_attempt.get("stop_move_ret"),
                    stop_move_error=external_stop_attempt.get(
                        "stop_move_error"
                    ),
                    stop_move_completed_monotonic=float(
                        external_stop_attempt[
                            "stop_move_completed_monotonic"
                        ]
                    ),
                    timeout=(
                        min(0.15, proposal_stop_confirm_timeout)
                        if retry_proposal_stop
                        else proposal_stop_confirm_timeout
                    ),
                    max_age=proposal_odom_timeout,
                    linear_epsilon=proposal_stop_linear_epsilon,
                    yaw_epsilon=proposal_stop_yaw_epsilon,
                    after_stop_attempt=complete_logical_stop,
                )
            else:
                result = issue_stop_and_confirm(
                    issue_stop_move,
                    odom_stop_monitor,
                    proposal_stop_confirm_timeout,
                    proposal_odom_timeout,
                    proposal_stop_linear_epsilon,
                    proposal_stop_yaw_epsilon,
                    after_stop_attempt=complete_logical_stop,
                )
            stop_attempts = [result]
            if retry_proposal_stop and not result.get("stop_confirmed"):
                # A continuous G1 velocity command can acknowledge one
                # StopMove while measured gait velocity remains nonzero.  Use
                # one bounded, ordered retry and retain both confirmations.
                retry_start = odom_stop_monitor.begin_confirmation()
                try:
                    watchdog_loco_client.StopMove()
                except Exception:
                    pass
                retry_parent_stop = stop_parent_velocity_proposal()
                result = finish_stop_confirmation(
                    monitor=odom_stop_monitor,
                    start=retry_start,
                    stop_move_ret=retry_parent_stop.get("ret"),
                    stop_move_error=retry_parent_stop.get("error"),
                    stop_move_completed_monotonic=retry_parent_stop.get(
                        "completed_monotonic",
                        time.monotonic(),
                    ),
                    timeout=proposal_stop_confirm_timeout,
                    max_age=proposal_odom_timeout,
                    linear_epsilon=proposal_stop_linear_epsilon,
                    yaw_epsilon=proposal_stop_yaw_epsilon,
                )
                stop_attempts.append(result)
            result = aggregate_stop_attempts(stop_attempts)
        else:
            stop_ret = None
            stop_error = None
            try:
                stop_ret = issue_stop_move()
            except Exception as exc:
                stop_error = str(exc)
            complete_logical_stop()
            result = {
                "ret": stop_ret,
                "stop_confirmed": stop_ret == 0,
                "stop_issued_monotonic": time.monotonic(),
            }
            if stop_error:
                result["error"] = f"StopMove failed: {stop_error}"
        if recoverable_stop_requested:
            if result.get("stop_confirmed"):
                proposal_gate.hold_after_confirmed_stop(reason_str)
            else:
                unconfirmed_reason = (
                    "obstacle_stop_unconfirmed"
                    if reason_str == "obstacle"
                    else "proposal_ttl_stop_unconfirmed"
                )
                proposal_gate.disarm(unconfirmed_reason)
        if was_moving:
            event_data = {"reason": reason_str}
            if reason_str == "obstacle":
                with obstacle_lock:
                    event_data["obstacle_distance"] = round(obstacle_dist, 2)
                    event_data["obstacle_angle_deg"] = round(math.degrees(obstacle_angle), 1)
            publish_event("motion_stop", event_data)
        result.update({"state": "idle", "reason": reason_str})
        if confirm_physical_stop:
            last_stop_result = dict(result)
            if proposal_stop_context:
                last_proposal_stop_result = dict(result)
        return result

    @motion_synchronized
    def do_stop_nav():
        nonlocal state, nav_cmd, speed_zone
        try:
            slam_client.PauseNav()
        except Exception:
            pass
        was_nav = state in (MotionState.NAVIGATING, MotionState.NAV_PAUSED)
        state = MotionState.IDLE
        nav_cmd = None
        speed_zone = SpeedZone.NORMAL
        if was_nav:
            publish_event("nav_stopped", {"reason": "command"})
        return {"status": "stopped"}

    @motion_synchronized
    def duration_expired():
        nonlocal state, current_cmd, speed_zone, move_timer
        move_timer = None
        if state == MotionState.MOVING:
            do_stop("duration_expired")

    # ── LiDAR DDS Subscription ──
    def on_cloud(msg):
        nonlocal obstacle_dist, obstacle_angle, lateral_obstacle
        nonlocal obstacle_scan_heading, last_cloud_time

        # Get heading
        if state == MotionState.MOVING and current_cmd:
            heading = translation_obstacle_heading(current_cmd)
        else:
            # NAVIGATING/IDLE: LiDAR is in body frame, heading=0 = robot forward
            heading = 0.0

        # Parse UInt8MultiArray format: [uint32 point_step][uint32 total_points][raw bytes]
        raw = bytes(msg.data)
        if len(raw) < 8:
            return
        point_step = struct.unpack_from('<I', raw, 0)[0]
        total_points = struct.unpack_from('<I', raw, 4)[0]
        data = raw[8:]

        n_valid = min(total_points, len(data) // point_step)
        if n_valid == 0:
            return
        with safety_lock:
            last_cloud_time = time.monotonic()
        if heading is None:
            # In-place rotation has no translation corridor.  Keep scan
            # freshness, but do not apply a synthetic forward obstacle cone.
            with obstacle_lock:
                obstacle_dist = float("inf")
                obstacle_angle = 0.0
                lateral_obstacle = False
                obstacle_scan_heading = None
            return

        # Numpy batch extraction (xyz at offsets 0, 4, 8 — already gravity-aligned)
        buf = np.frombuffer(data, dtype=np.uint8, count=n_valid * point_step).reshape(n_valid, point_step)
        px = buf[:, 0:4].copy().view(dtype='<f4').flatten()
        py = buf[:, 4:8].copy().view(dtype='<f4').flatten()
        pz = buf[:, 8:12].copy().view(dtype='<f4').flatten()

        # Vectorized filter
        z_mask = (pz >= z_min) & (pz <= z_max)
        dist_sq = px * px + py * py
        range_mask = (dist_sq >= 0.04) & (dist_sq <= 6.25)
        valid = z_mask & range_mask

        if not np.any(valid):
            with obstacle_lock:
                obstacle_dist = float("inf")
                obstacle_angle = 0.0
                lateral_obstacle = False
                obstacle_scan_heading = heading
            return

        vx_pts = px[valid]
        vy_pts = py[valid]
        vdist = np.sqrt(dist_sq[valid])

        point_angles = np.arctan2(vy_pts, vx_pts)

        # Normal mode: forward cone based on heading
        angle_diffs = np.abs(np.mod(point_angles - heading + math.pi, 2 * math.pi) - math.pi)
        # Forward cone
        forward_mask = angle_diffs <= cone_half_angle
        min_fwd_dist = float("inf")
        min_fwd_angle = 0.0
        if np.any(forward_mask):
            fwd_dists = vdist[forward_mask]
            idx = np.argmin(fwd_dists)
            min_fwd_dist = float(fwd_dists[idx])
            min_fwd_angle = float(point_angles[forward_mask][idx])

            if min_fwd_dist <= decel_threshold and state in (MotionState.MOVING, MotionState.NAVIGATING):
                fwd_x = vx_pts[forward_mask][idx]
                fwd_y = vy_pts[forward_mask][idx]
                fwd_z = pz[valid][forward_mask][idx]
                print(f"[SmartMotion:lidar] closest_fwd: dist={min_fwd_dist:.2f}m "
                      f"xyz=({fwd_x:.2f},{fwd_y:.2f},{fwd_z:.2f}) "
                      f"angle={math.degrees(min_fwd_angle):.1f}° "
                      f"total_fwd_pts={int(forward_mask.sum())} "
                      f"heading={math.degrees(heading):.1f}°", flush=True)

        # Lateral (45°-90°, within stop_threshold)
        lat_mask = (angle_diffs >= math.radians(45)) & \
                   (angle_diffs <= math.radians(90)) & \
                   (vdist < stop_threshold)
        lat_detected = bool(np.any(lat_mask))

        with obstacle_lock:
            obstacle_dist = min_fwd_dist
            obstacle_angle = min_fwd_angle
            lateral_obstacle = lat_detected
            obstacle_scan_heading = heading

    try:
        event_node.create_subscription(
            UInt8MultiArray, f"/{namespace}/lidar/cloud", on_cloud, _QOS)
        print(f"[SmartMotion:pid={os.getpid()}] LiDAR subscribed (ROS2 /{namespace}/lidar/cloud)")
    except Exception as e:
        print(f"[SmartMotion:pid={os.getpid()}] WARNING: LiDAR subscribe failed: {e}")

    # ── Safety monitors config ──
    tilt_threshold = math.radians(config.get("tilt_threshold_deg", 35))
    foot_force_min = config.get("foot_force_min", 10)
    foot_airborne_timeout = config.get("foot_airborne_timeout", 0.2)
    comm_timeout = config.get("comm_timeout", 0.5)
    motor_temp_decel = config.get("motor_temp_decel", 75)
    motor_temp_stop = config.get("motor_temp_stop", 85)

    # Safety state
    last_lowstate_time = 0.0
    tilt_triggered = False
    foot_airborne_start = 0.0
    foot_force_seen_nonzero = False  # only enable airborne detection after seeing real force
    max_motor_temp = 0.0
    last_temp_check = 0.0
    stop_repeat_count = 0  # counter for repeated StopMove after emergency

    @motion_synchronized
    def emergency_stop(reason_str, extra=None):
        """Emergency stop with optional damp mode for tilt."""
        nonlocal state, current_cmd, speed_zone, move_timer, stop_repeat_count
        was_proposal_motion = bool(
            current_cmd and current_cmd.get("source") == "velocity_proposal"
        )
        was_active = state in (
            MotionState.MOVING,
            MotionState.NAVIGATING,
            MotionState.NAV_PAUSED,
        )
        # Preserve the emergency path's immediate physical brake.  Proposal
        # motion additionally performs an ordered parent stop below so a late
        # continuous-velocity reply cannot restart the robot.
        try:
            stop_loco_client.StopMove()
        except Exception:
            pass
        if reason_str == "tilt":
            try:
                loco_client.SetFsmId(1)  # damp mode
            except Exception:
                pass
        if was_proposal_motion:
            proposal_gate.disarm(f"{reason_str}_latched")
            do_stop(reason_str)
        else:
            if move_timer:
                move_timer.cancel()
                move_timer = None
            clear_proposal_execution()
        if state in (MotionState.NAVIGATING, MotionState.NAV_PAUSED):
            try:
                slam_client.PauseNav()
            except Exception:
                pass
        state = MotionState.IDLE
        current_cmd = None
        speed_zone = SpeedZone.NORMAL
        stop_repeat_count = 3  # repeat StopMove in main loop to ensure it takes effect
        if was_active:
            event_data = {"reason": reason_str}
            if extra:
                event_data.update(extra)
            publish_event("safety_stop", event_data)
            print(f"[SmartMotion] emergency_stop({reason_str}): StopMove sent", flush=True)

    # ── LowState DDS Subscription (IMU tilt + joint temp) ──
    def on_lowstate(msg):
        nonlocal last_lowstate_time, tilt_triggered, max_motor_temp, last_temp_check

        now = time.monotonic()
        # Tilt detection (20Hz, every callback)
        imu = msg.imu_state
        roll = abs(float(imu.rpy[0]))
        pitch = abs(float(imu.rpy[1]))
        is_tilted = roll > tilt_threshold or pitch > tilt_threshold
        with safety_lock:
            last_lowstate_time = now
            was_tilted = tilt_triggered
            tilt_triggered = is_tilted

        if (
            is_tilted
            and not was_tilted
            and state in (
                MotionState.MOVING,
                MotionState.NAVIGATING,
                MotionState.NAV_PAUSED,
            )
        ):
            emergency_stop("tilt", {
                "roll_deg": round(math.degrees(roll), 1),
                "pitch_deg": round(math.degrees(pitch), 1),
            })

        # Joint temperature (1Hz check)
        if now - last_temp_check >= 1.0:
            last_temp_check = now
            temp_max = 0.0
            for m in msg.motor_state:
                for t in m.temperature:
                    if float(t) > temp_max:
                        temp_max = float(t)
            with safety_lock:
                max_motor_temp = temp_max

    try:
        from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_
        lowstate_sub = ChannelSubscriber("rt/lowstate", LowState_)
        lowstate_sub.Init(on_lowstate, 10)
        print(f"[SmartMotion:pid={os.getpid()}] LowState subscribed (tilt + temp)")
    except Exception as e:
        print(f"[SmartMotion:pid={os.getpid()}] WARNING: LowState subscribe failed: {e}")

    # ── OdomState DDS Subscription (foot force) ──
    def on_odom(msg):
        nonlocal foot_airborne_start, foot_force_seen_nonzero

        velocity = list(msg.velocity)
        motion = (
            float(velocity[0]) if len(velocity) > 0 else float("inf"),
            float(velocity[1]) if len(velocity) > 1 else float("inf"),
            float(msg.yaw_speed),
        )
        odom_stop_monitor.record(motion)

        if state != MotionState.MOVING:
            foot_airborne_start = 0.0
            return

        forces = list(msg.foot_force)
        if len(forces) < 4:
            return

        all_airborne = all(f < foot_force_min for f in forces[:4])

        # Only enable airborne detection after seeing at least one valid (non-zero) reading
        if not foot_force_seen_nonzero:
            if not all_airborne:
                foot_force_seen_nonzero = True
                print(f"[SmartMotion] foot_force sensor active: {[round(f,1) for f in forces[:4]]}", flush=True)
            return  # skip detection until sensor is confirmed working

        if all_airborne:
            now = time.monotonic()
            if foot_airborne_start == 0.0:
                foot_airborne_start = now
            elif now - foot_airborne_start > foot_airborne_timeout:
                airborne_ms = round((now - foot_airborne_start) * 1000)
                foot_airborne_start = 0.0
                emergency_stop("foot_airborne", {
                    "foot_forces": [round(f, 1) for f in forces[:4]],
                    "airborne_duration_ms": airborne_ms,
                })
        else:
            foot_airborne_start = 0.0

    try:
        from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_
        odom_sub = ChannelSubscriber("rt/odommodestate", SportModeState_)
        odom_sub.Init(on_odom, 10)
        print(f"[SmartMotion:pid={os.getpid()}] OdomState subscribed (foot force)")
    except Exception as e:
        print(f"[SmartMotion:pid={os.getpid()}] WARNING: OdomState subscribe failed: {e}")

    # ── SLAM info DDS subscription (for nav arrival/obstacle callbacks) ──
    slam_info_msg_count = 0
    slam_info_pos_count = 0

    def on_slam_info(msg):
        nonlocal nav_arrived_flag, nav_arrived_error, nav_current_pose
        nonlocal slam_info_msg_count, slam_info_pos_count
        slam_info_msg_count += 1
        try:
            data = json.loads(msg.data)
        except Exception:
            return
        msg_type = data.get("type", "")
        if msg_type in ("pos_info", "mapping_info"):
            slam_info_pos_count += 1
            pose_data = data.get("data", {}).get("currentPose")
            if pose_data:
                q_x = float(pose_data.get("q_x", 0.0))
                q_y = float(pose_data.get("q_y", 0.0))
                q_z = float(pose_data.get("q_z", 0.0))
                q_w = float(pose_data.get("q_w", 1.0))
                yaw = math.atan2(
                    2 * (q_w * q_z + q_x * q_y),
                    1 - 2 * (q_y * q_y + q_z * q_z),
                )
                with slam_info_lock:
                    nav_current_pose = {
                        "x": pose_data["x"], "y": pose_data["y"], "yaw": round(yaw, 3)
                    }
                # Debug: log first few pos_info to verify subprocess receives them
                if slam_info_pos_count <= 3:
                    print(f"[SmartMotion] slam_info pos_info #{slam_info_pos_count}: "
                          f"x={pose_data['x']:.3f} y={pose_data['y']:.3f}", flush=True)
        elif msg_type == "ctrl_info":
            # ctrl_info is never published by SLAM service (verified via test_slam_info_logger)
            # Keep handler for future compatibility but don't rely on it
            ctrl_data = data.get("data", {})
            if ctrl_data.get("is_arrived"):
                with slam_info_lock:
                    nav_arrived_flag = True
            obs = ctrl_data.get("obsInfo", {})
            if obs.get("state") and obs.get("time", 0) > 10:
                with slam_info_lock:
                    nav_arrived_error = "blocked by obstacle for >10s"
                    nav_arrived_flag = True

    try:
        from unitree_sdk2py.idl.std_msgs.msg.dds_ import String_ as StringMsg_
        slam_info_sub = ChannelSubscriber("rt/slam_info", StringMsg_)
        slam_info_sub.Init(on_slam_info, 10)
        print(f"[SmartMotion:pid={os.getpid()}] rt/slam_info subscribed (nav arrival)")
    except Exception as e:
        print(f"[SmartMotion:pid={os.getpid()}] WARNING: rt/slam_info subscribe failed: {e}")

    def refresh_fsm_state():
        nonlocal last_fsm_check, last_fsm_id, last_fsm_ret
        nonlocal last_fsm_error, main_control_ready
        fsm_id = None
        ret = None
        error = None
        try:
            result = query_parent_fsm_id()
            ret = result.get("ret")
            fsm_id = result.get("fsm_id")
            error = result.get("error")
            ready = (
                error is None
                and ret == 0
                and int(fsm_id) in proposal_allowed_fsm_ids
            )
        except Exception as exc:
            error = str(exc)
            ready = False
        with safety_lock:
            last_fsm_check = time.monotonic()
            last_fsm_id = fsm_id
            last_fsm_ret = ret
            last_fsm_error = error
            main_control_ready = ready

    def proposal_runtime_status(force_fsm_check=False):
        """Return current Driver-owned readiness for proposal execution."""
        if force_fsm_check:
            refresh_fsm_state()
        now = time.monotonic()
        odom = odom_stop_monitor.latest(now)
        with safety_lock:
            lowstate_age = now - last_lowstate_time if last_lowstate_time else float("inf")
            scan_age = now - last_cloud_time if last_cloud_time else float("inf")
            nav_status_age = (
                now - last_nav_status_time if last_nav_status_time else float("inf")
            )
            fsm_age = now - last_fsm_check if last_fsm_check else float("inf")
            temperature = max_motor_temp
            fsm_id = last_fsm_id
            fsm_ret = last_fsm_ret
            fsm_error = last_fsm_error
            control_ready = main_control_ready
            tilted = tilt_triggered
        odom_age = odom["age"]

        reason = ""
        if not proposal_driver_authorized:
            reason = "driver_execution_not_authorized"
        elif lowstate_age > comm_timeout:
            reason = "main_control_state_stale"
        elif odom_age > proposal_odom_timeout:
            reason = "odometry_stale"
        elif scan_age > proposal_scan_timeout:
            reason = "scan_stale"
        elif tilted:
            reason = "tilt_latched"
        elif temperature > motor_temp_stop:
            reason = "joint_overheat_latched"
        elif nav_status_age > proposal_nav_status_timeout:
            reason = "nav2_status_stale"
        elif fsm_error is not None or (fsm_ret is not None and fsm_ret != 0):
            reason = "main_control_rpc_failed"
        elif fsm_age > proposal_fsm_timeout:
            reason = "main_control_status_stale"
        elif not control_ready:
            reason = "main_control_not_ready"

        return {
            "ready": not reason,
            "reason": reason or None,
            "fsm_id": fsm_id,
            "fsm_ret": fsm_ret,
            "fsm_error": fsm_error,
            # In this G1 API, a fresh allowed locomotion FSM is the native
            # fail-closed proxy for main-control/e-stop execution permission.
            "emergency_stop_clear": control_ready and fsm_age <= proposal_fsm_timeout,
            "lowstate_age_ms": None if not math.isfinite(lowstate_age) else round(lowstate_age * 1000),
            "odometry_age_ms": None if not math.isfinite(odom_age) else round(odom_age * 1000),
            "odometry_callback_count": odom["callback_count"],
            "scan_age_ms": None if not math.isfinite(scan_age) else round(scan_age * 1000),
            "nav2_status_age_ms": None if not math.isfinite(nav_status_age) else round(nav_status_age * 1000),
            "fsm_age_ms": None if not math.isfinite(fsm_age) else round(fsm_age * 1000),
        }

    # ── Command handlers ──
    @motion_synchronized
    def handle_move(
        vx,
        vy,
        vyaw,
        duration,
        finite_rpc=False,
        source="mcp",
        proposal=None,
        deadline_monotonic=None,
    ):
        nonlocal state, current_cmd, speed_zone, move_timer

        if state in (MotionState.NAVIGATING, MotionState.NAV_PAUSED):
            do_stop_nav()
        if move_timer:
            move_timer.cancel()
            move_timer = None

        previous = None
        if state == MotionState.MOVING and current_cmd:
            previous = {
                "vx": current_cmd["vx"],
                "vy": current_cmd["vy"],
                "vyaw": current_cmd["vyaw"],
                "source": current_cmd.get("source", "mcp"),
            }

        clamped_vx, clamped_vy, clamped_vyaw = clamp(vx, vy, vyaw, SpeedZone.NORMAL)

        now = time.time()
        current_cmd = {
            "vx": vx, "vy": vy, "vyaw": vyaw,
            "duration": duration, "start_time": now,
            "end_time": (now + duration) if duration > 0 else None,
            "source": source,
            "effective_velocity": (
                clamped_vx,
                clamped_vy,
                clamped_vyaw,
            ),
        }
        if proposal is not None:
            current_cmd.update({
                "nav_id": proposal.nav_id,
                "sequence": proposal.sequence,
                "deadline_monotonic": deadline_monotonic,
            })
        state = MotionState.MOVING
        speed_zone = SpeedZone.NORMAL

        if finite_rpc:
            if proposal is None or deadline_monotonic is None:
                raise ValueError("proposal identity and deadline are required")
            enqueue_proposal_apply({
                "vx": clamped_vx,
                "vy": clamped_vy,
                "vyaw": clamped_vyaw,
                "deadline_monotonic": float(deadline_monotonic),
                "queued_monotonic": time.monotonic(),
                "nav_id": proposal.nav_id,
                "sequence": proposal.sequence,
            })
            ret = None
        else:
            clear_proposal_execution()
            ret = loco_client.Move(clamped_vx, clamped_vy, clamped_vyaw, True)

        if duration > 0 and not finite_rpc:
            move_timer = threading.Timer(duration, duration_expired)
            move_timer.start()

        if previous and not (
            source == "velocity_proposal"
            and previous.get("source") == "velocity_proposal"
        ):
            publish_event("new_command", {
                "previous": previous,
                "new": {
                    "vx": clamped_vx,
                    "vy": clamped_vy,
                    "vyaw": clamped_vyaw,
                    "source": source,
                },
            })
        elif not previous:
            publish_event("motion_start", {
                "params": {
                    "vx": clamped_vx,
                    "vy": clamped_vy,
                    "vyaw": clamped_vyaw,
                    "duration": duration,
                    "source": source,
                },
            })

        return {
            "ret": ret,
            "vx": clamped_vx,
            "vy": clamped_vy,
            "vyaw": clamped_vyaw,
            "duration": duration,
            "state": state.value,
            "queued": finite_rpc,
        }

    @motion_synchronized
    def handle_navigate_to(x, y, yaw, target_name, speed=0.5, mode=1, stall_timeout=60):
        nonlocal state, nav_cmd, speed_zone, nav_arrived_flag, nav_arrived_error

        if state == MotionState.MOVING:
            do_stop("command")
        elif state in (MotionState.NAVIGATING, MotionState.NAV_PAUSED):
            do_stop_nav()

        # Reset arrival state for this navigation session
        with slam_info_lock:
            nav_arrived_flag = False
            nav_arrived_error = None

        q_z = math.sin(yaw / 2)
        q_w = math.cos(yaw / 2)
        # Clear any paused state before starting new navigation.
        # If previous nav was paused (by obstacle or user), SLAM service may still
        # be in paused state — NavigateTo would fail with code=3104.
        try:
            slam_client.ResumeNav()
        except Exception:
            pass
        code, resp = slam_client.NavigateTo(x, y, 0, 0, 0, q_z, q_w,
                                              speed=speed, mode=mode)

        if code != 0:
            return {"error": f"NavigateTo failed, code={code}", "response": resp}

        label = target_name or f"({x:.1f}, {y:.1f})"
        nav_cmd = {"target_name": label, "target_pose": {"x": x, "y": y, "yaw": yaw}, "start_time": time.time()}
        state = MotionState.NAVIGATING
        speed_zone = SpeedZone.NORMAL

        publish_event("nav_start", {"target_name": label, "target_pose": {"x": x, "y": y, "yaw": yaw}})
        # Return immediately — non-blocking. wait_navigation_done handles arrival detection.
        return {"status": "navigating", "target": label, "pose": {"x": x, "y": y, "yaw": yaw}}

    def handle_wait_nav_done(stall_timeout=60):
        """Block until navigation completes or robot is stuck.
        Arrival detection: pose-based (distance to target < 0.3m).
        Also checks ctrl_info.is_arrived if SLAM service ever publishes it.
        Runs safety checks and ROS2 spin during the wait."""
        nonlocal state, nav_arrived_flag, nav_arrived_error, nav_cmd, speed_zone, stop_repeat_count
        poll_interval = 0.5
        last_pose = None
        stall_start = time.time()
        wait_start = time.time()

        with slam_info_lock:
            last_pose = dict(nav_current_pose) if nav_current_pose else None

        if last_pose is None:
            print(f"[SmartMotion] wait_nav_done: WARNING no pose data yet "
                  f"(pos_info count={slam_info_pos_count})", flush=True)

        while state in (MotionState.NAVIGATING, MotionState.NAV_PAUSED):
            elapsed = time.time() - wait_start

            with slam_info_lock:
                arrived = nav_arrived_flag
                error = nav_arrived_error
                pose = dict(nav_current_pose) if nav_current_pose else None
                nav_arrived_flag = False
                nav_arrived_error = None

            # ctrl_info-based arrival (secondary — SLAM service may not publish ctrl_info)
            if arrived:
                state = MotionState.IDLE
                nav_cmd = None
                speed_zone = SpeedZone.NORMAL
                if error:
                    return {"status": "error", "error": error}
                print(f"[SmartMotion] wait_nav_done: arrived via ctrl_info "
                      f"after {elapsed:.1f}s, pose={pose}", flush=True)
                return {"status": "arrived", "pose": pose}

            # Pose-based arrival detection (primary mechanism)
            if pose and nav_cmd:
                tx = nav_cmd["target_pose"]["x"]
                ty = nav_cmd["target_pose"]["y"]
                dist = math.sqrt((pose["x"] - tx)**2 + (pose["y"] - ty)**2)
                if dist < 0.3:
                    state = MotionState.IDLE
                    nav_cmd = None
                    speed_zone = SpeedZone.NORMAL
                    print(f"[SmartMotion] wait_nav_done: arrived via pose "
                          f"after {elapsed:.1f}s, dist={dist:.3f}m, pose={pose}", flush=True)
                    return {"status": "arrived", "pose": pose}
                # Debug log every 5s
                if int(elapsed) % 5 == 0 and abs(elapsed - int(elapsed)) < poll_interval:
                    print(f"[SmartMotion] wait_nav_done: {elapsed:.0f}s elapsed, "
                          f"dist={dist:.3f}m, pose={pose}", flush=True)

            # Stall detection — no movement for stall_timeout seconds.
            # Note: NAV_PAUSED (obstacle stop) doesn't count as stall —
            # the robot is intentionally stopped waiting for obstacle to clear.
            if state == MotionState.NAVIGATING and pose and last_pose:
                dx = pose["x"] - last_pose["x"]
                dy = pose["y"] - last_pose["y"]
                moved = math.sqrt(dx * dx + dy * dy)
                if moved > 0.05:
                    stall_start = time.time()
                    last_pose = pose
                else:
                    last_pose = pose

            if state == MotionState.NAVIGATING and time.time() - stall_start > stall_timeout:
                do_stop_nav()
                print(f"[SmartMotion] wait_nav_done: TIMEOUT after {stall_timeout}s "
                      f"no movement, pose={pose}", flush=True)
                return {"status": "timeout",
                        "error": f"No movement for {stall_timeout}s, navigation cancelled",
                        "pose": pose}

            # Run safety checks during wait.  ROS callbacks are serviced by
            # the dedicated proposal/event executor thread.
            process_safety_checks()
            if stop_repeat_count > 0 and state == MotionState.IDLE:
                loco_client.StopMove()
                stop_repeat_count -= 1

            time.sleep(poll_interval)

        # State changed externally (e.g., emergency stop)
        print(f"[SmartMotion] wait_nav_done: state changed to {state.value}", flush=True)
        return {"status": "stopped", "state": state.value}

    @motion_synchronized
    def handle_pause_nav(reason_str):
        nonlocal state
        if state != MotionState.NAVIGATING:
            return {"error": f"Cannot pause nav: state is {state.value}"}
        code, _ = slam_client.PauseNav()
        state = MotionState.NAV_PAUSED
        event_data = {"reason": reason_str}
        if reason_str == "obstacle":
            with obstacle_lock:
                event_data["obstacle_distance"] = round(obstacle_dist, 2)
        publish_event("nav_paused", event_data)
        return {"status": "paused"} if code == 0 else {"error": f"PauseNav failed, code={code}"}

    @motion_synchronized
    def handle_resume_nav():
        nonlocal state
        if state != MotionState.NAV_PAUSED:
            return {"error": f"Cannot resume nav: state is {state.value}"}
        code, _ = slam_client.ResumeNav()
        state = MotionState.NAVIGATING
        publish_event("nav_resumed", {})
        return {"status": "resumed"} if code == 0 else {"error": f"ResumeNav failed, code={code}"}

    @motion_synchronized
    def handle_get_state():
        result = {"state": state.value, "speed_zone": speed_zone.value}
        if current_cmd and state == MotionState.MOVING:
            result["motion"] = current_cmd.copy()
        if nav_cmd and state in (MotionState.NAVIGATING, MotionState.NAV_PAUSED):
            result["navigation"] = nav_cmd.copy()
        with obstacle_lock:
            result["obstacle_distance"] = round(obstacle_dist, 2) if obstacle_dist != float("inf") else None
            result["lateral_obstacle"] = lateral_obstacle
        return result

    @motion_synchronized
    def handle_begin_velocity_proposal_stop_confirmation():
        # Freeze proposal execution before the responsive parent RpcProxy
        # issues StopMove.  Keep the subscription until zero odometry is
        # confirmed so teardown remains fail-closed.
        if state in (MotionState.NAVIGATING, MotionState.NAV_PAUSED):
            do_stop_nav()
        proposal_stop_transition.set()
        proposal_connected.clear()
        clear_proposal_apply_queue()
        clear_proposal_execution()
        start = odom_stop_monitor.begin_confirmation()
        fail_safe_stop_ret = None
        fail_safe_stop_error = None
        try:
            fail_safe_stop_ret = watchdog_loco_client.StopMove()
        except Exception as exc:
            fail_safe_stop_error = str(exc)
        return {
            "confirmation_start": {
                "monotonic": start.monotonic,
                "unix_ms": start.unix_ms,
                "odometry_callback_count": start.odometry_callback_count,
            },
            "fail_safe_stop_ret": fail_safe_stop_ret,
            "fail_safe_stop_error": fail_safe_stop_error,
        }

    @motion_synchronized
    def handle_bind_velocity_proposal(
        topic,
        expected_nav_id=None,
        external_stop_attempt=None,
    ):
        nonlocal last_proposal_stop_result
        if expected_nav_id is not None and expected_nav_id != "":
            try:
                expected_nav_id = resolve_expected_nav_id(
                    {"expected_nav_id": expected_nav_id}
                )
            except ValueError as exc:
                stopped = do_stop(
                    str(exc),
                    confirm_physical_stop=True,
                    external_stop_attempt=external_stop_attempt,
                )
                proposal_gate.disarm(str(exc))
                diagnostics = stopped.get("stop_confirmation") or {}
                return {
                    "error": str(exc),
                    "connected": bool(proposal_node.topic),
                    "armed": False,
                    "stop_confirmed": stopped.get("stop_confirmed", False),
                    "stop_move_ret": diagnostics.get("stop_move_ret"),
                    "stop_move_error": diagnostics.get("stop_move_error"),
                    "stop_confirmation": diagnostics,
                }
        else:
            expected_nav_id = None
        already_bound = (
            proposal_gate.is_bound_to(topic, expected_nav_id)
            if expected_nav_id is not None
            else proposal_gate.is_waiting_on(topic)
        )
        if already_bound:
            stopped = do_stop(
                "proposal_bind_existing",
                confirm_physical_stop=True,
                external_stop_attempt=external_stop_attempt,
            )
            if not stopped.get("stop_confirmed"):
                proposal_gate.disarm("stop_unconfirmed")
                diagnostics = stopped.get("stop_confirmation") or {}
                return {
                    "error": "StopMove/odometry stop was not confirmed before proposal bind",
                    "connected": bool(proposal_node.topic),
                    "armed": False,
                    "stop_confirmed": False,
                    "stop_move_ret": diagnostics.get("stop_move_ret"),
                    "stop_move_error": diagnostics.get("stop_move_error"),
                    "stop_confirmation": diagnostics,
                }
            proposal_connected.set()
            proposal_stop_transition.clear()
            result = handle_get_velocity_proposal_status()
            result["state"] = "connected"
            result["stop_confirmed"] = True
            result["stop_move_ret"] = stopped.get("ret")
            result["stop_move_error"] = (
                (stopped.get("stop_confirmation") or {}).get(
                    "stop_move_error"
                )
            )
            result["stop_confirmation"] = stopped.get("stop_confirmation")
            return result
        if proposal_gate.connected_topic or proposal_node.topic:
            stopped = handle_unbind_velocity_proposal(
                "proposal_rebind",
                external_stop_attempt=external_stop_attempt,
            )
            if not stopped.get("stop_confirmed"):
                return {
                    "error": "StopMove was not confirmed before proposal rebind",
                    "connected": False,
                    "stop_confirmed": False,
                    "stop_move_ret": stopped.get("stop_move_ret"),
                    "stop_move_error": stopped.get("stop_move_error"),
                    "stop_confirmation": stopped.get("stop_confirmation"),
                }
        else:
            stopped = do_stop(
                "proposal_bind",
                confirm_physical_stop=True,
                external_stop_attempt=external_stop_attempt,
            )
            if not stopped.get("stop_confirmed"):
                diagnostics = stopped.get("stop_confirmation") or {}
                return {
                    "error": "StopMove/odometry stop was not confirmed before proposal bind",
                    "connected": False,
                    "stop_confirmed": False,
                    "stop_move_ret": diagnostics.get("stop_move_ret"),
                    "stop_move_error": diagnostics.get("stop_move_error"),
                    "stop_confirmation": diagnostics,
                }
        if not proposal_enabled:
            return {"error": "velocity_proposal_execution_disabled", "connected": False}
        if not proposal_driver_authorized:
            return {"error": "driver_execution_not_authorized", "connected": False}
        if topic != expected_proposal_topic:
            return {
                "error": "unexpected_velocity_proposal_topic",
                "expected_topic": expected_proposal_topic,
                "connected": False,
            }
        callback_status = proposal_callback_status()
        if (
            callback_status["failed"]
            or not callback_status["ready"]
            or not callback_status["alive"]
        ):
            proposal_gate.disarm("proposal_callback_unavailable")
            return {
                "error": "velocity_proposal_callback_unavailable",
                "connected": False,
                "proposal_callback": callback_status,
            }
        try:
            last_proposal_stop_result = None
            proposal_gate.bind(topic, expected_nav_id)
            proposal_apply_diagnostics.begin_session(expected_nav_id)
            proposal_node.bind(topic, on_velocity_proposal)
            proposal_connected.set()
            refresh_fsm_state()
        except Exception as exc:
            proposal_gate.unbind("subscription_create_failed")
            proposal_connected.clear()
            proposal_node.unbind()
            try:
                stop_loco_client.StopMove()
            except Exception:
                pass
            return {"error": f"velocity_proposal_subscribe_failed: {exc}", "connected": False}
        proposal_stop_transition.clear()
        result = handle_get_velocity_proposal_status()
        result["state"] = "connected"
        result["stop_confirmed"] = True
        result["stop_move_ret"] = stopped.get("stop_move_ret", stopped.get("ret"))
        result["stop_move_error"] = stopped.get("stop_move_error")
        result["stop_confirmation"] = stopped.get("stop_confirmation")
        return result

    @motion_synchronized
    def handle_unbind_velocity_proposal(
        reason="canvas_stop",
        external_stop_attempt=None,
    ):
        nonlocal last_stop_result
        # N5 ordering: command and observe physical stop before destroying the
        # only proposal subscription.
        if (
            external_stop_attempt is None
            and state in (MotionState.NAVIGATING, MotionState.NAV_PAUSED)
        ):
            do_stop_nav()
        stopped = do_stop(
            reason,
            confirm_physical_stop=True,
            external_stop_attempt=external_stop_attempt,
        )
        for _ in range(2 if external_stop_attempt is None else 0):
            # Preserve the original retry contract: retry rejected/failed RPC
            # attempts, but do not mask a measured nonzero or odom timeout by
            # issuing more StopMove calls.
            if stopped.get("ret") == 0:
                break
            stopped = issue_stop_and_confirm(
                issue_stop_move,
                odom_stop_monitor,
                proposal_stop_confirm_timeout,
                proposal_odom_timeout,
                proposal_stop_linear_epsilon,
                proposal_stop_yaw_epsilon,
            )
        last_stop_result = dict(stopped)
        if not stopped["stop_confirmed"]:
            proposal_gate.disarm("stop_unconfirmed")
            proposal_connected.clear()
            diagnostics = stopped.get("stop_confirmation") or {}
            return {
                "state": "error",
                "connected": bool(proposal_node.topic),
                "armed": False,
                "stop_confirmed": False,
                "subscriber_retained": bool(proposal_node.topic),
                "reason": reason,
                "error": stopped.get("error") or "fresh zero-odometry stop was not confirmed",
                "stop_move_ret": diagnostics.get("stop_move_ret"),
                "stop_move_error": diagnostics.get("stop_move_error"),
                "stop_confirmation": diagnostics,
            }
        proposal_gate.unbind(reason)
        proposal_connected.clear()
        proposal_node.unbind()
        return {
            "state": "idle",
            "connected": False,
            "stop_confirmed": True,
            "reason": reason,
            "stop_move_ret": stopped.get("ret"),
            "stop_move_error": (
                (stopped.get("stop_confirmation") or {}).get("stop_move_error")
            ),
            "stop_confirmation": stopped.get("stop_confirmation"),
            **({"error": stopped["error"]} if stopped.get("error") else {}),
        }

    @motion_synchronized
    def handle_get_velocity_proposal_status():
        result = proposal_gate.snapshot(time.monotonic())
        latest_stop = last_stop_result or {}
        latest_proposal_stop = last_proposal_stop_result or {}
        result.update({
            "enabled": proposal_enabled,
            "canvas_running": proposal_connected.is_set(),
            "stop_transition_active": proposal_stop_transition.is_set(),
            "driver_authorized": bool(
                proposal_driver_authorized
                and not proposal_stop_transition.is_set()
                and proposal_gate.connected_topic
                and proposal_gate.armed
            ),
            "expected_topic": expected_proposal_topic,
            "schema": proposal_limits.schema,
            "proposal_callback": proposal_callback_status(),
            "safety": proposal_runtime_status(force_fsm_check=False),
            "proposal_execution": proposal_apply_diagnostics.snapshot(),
            "last_stop_confirmed": latest_stop.get("stop_confirmed"),
            "last_stop_move_ret": latest_stop.get("ret"),
            "last_stop_move_error": (
                (latest_stop.get("stop_confirmation") or {}).get(
                    "stop_move_error"
                )
            ),
            "last_stop_confirmation": latest_stop.get("stop_confirmation"),
            "last_proposal_stop": (
                dict(latest_proposal_stop)
                if latest_proposal_stop
                else None
            ),
        })
        return result

    @motion_synchronized
    def on_velocity_proposal(msg):
        nonlocal last_nav_status_time, stop_repeat_count
        now = time.monotonic()
        received_unix_ms = time.time() * 1000
        proposal_apply_diagnostics.record_received(now)
        if proposal_stop_transition.is_set():
            proposal_apply_diagnostics.record_rejected("stop_transition")
            return
        pending_fault = current_proposal_watchdog_fault()
        try:
            payload = json.loads(msg.data)
        except (json.JSONDecodeError, TypeError, AttributeError):
            if pending_fault:
                _, reason = pending_fault
                proposal_apply_diagnostics.record_rejected(reason)
                if reason == "proposal_ttl_expired":
                    proposal_gate.watchdog(now)
                else:
                    proposal_gate.disarm(reason)
                do_stop(reason)
                return
            proposal_apply_diagnostics.record_rejected("invalid_json")
            if proposal_gate.armed:
                proposal_gate.disarm("invalid_json")
                do_stop("invalid_json")
            return
        proposal_apply_diagnostics.record_proposal_arrival(
            payload,
            received_unix_ms,
        )
        if pending_fault:
            _, reason = pending_fault
            proposal_apply_diagnostics.record_rejected(reason)
            if reason == "proposal_ttl_expired":
                proposal_gate.watchdog(now)
            else:
                proposal_gate.disarm(reason)
            do_stop(reason)
            return

        was_armed = proposal_gate.armed
        was_waiting_for_nav_id = proposal_gate.waiting_for_nav_id
        decision = proposal_gate.accept(
            payload,
            now,
            now_unix_ms=received_unix_ms,
        )
        if was_waiting_for_nav_id and proposal_gate.expected_nav_id:
            proposal_apply_diagnostics.bind_claimed_session(
                proposal_gate.expected_nav_id,
                received_monotonic=now,
                payload=payload,
                received_unix_ms=received_unix_ms,
            )
        if decision.stop:
            if decision.reason != "proposal_zero":
                proposal_apply_diagnostics.record_rejected(decision.reason)
            terminal_proposal = bool(
                decision.proposal and decision.proposal.is_terminal
            )
            if terminal_proposal:
                stopped = do_stop(
                    decision.reason,
                    confirm_physical_stop=terminal_proposal,
                )
                terminal_stop_clean = bool(
                    stopped.get("reason") == decision.reason
                )
                resumed_waiting = proposal_gate.resume_waiting_after_terminal_stop(
                    bool(stopped.get("stop_confirmed"))
                    and terminal_stop_clean
                )
                if resumed_waiting:
                    # The confirmed terminal stop already completed the
                    # physical brake.  Do not let its deferred StopMove
                    # repeats cross the lease boundary into the next task.
                    stop_repeat_count = 0
                return
            is_proposal_motion = bool(
                current_cmd and current_cmd.get("source") == "velocity_proposal"
            )
            newly_disarmed = was_armed and not proposal_gate.armed
            if proposal_decision_requires_physical_stop(
                decision.reason,
                decision.proposal is not None,
                is_proposal_motion,
                newly_disarmed,
                proposal_gate.recoverable_stop_active,
            ):
                do_stop(decision.reason)
            return
        if not decision.execute or decision.proposal is None:
            proposal_apply_diagnostics.record_rejected(
                decision.reason or "proposal_not_executable"
            )
            return

        with safety_lock:
            last_nav_status_time = now

        # The independent FSM monitor keeps this sample fresh.  Avoid a
        # synchronous per-frame query that would block ROS proposal callbacks.
        runtime = proposal_runtime_status(force_fsm_check=False)
        if not runtime["ready"]:
            reason = runtime["reason"] or "driver_safety_not_ready"
            proposal_apply_diagnostics.record_rejected(reason)
            proposal_gate.disarm(reason)
            do_stop(reason)
            return

        proposal = decision.proposal
        remaining = proposal_gate.deadline_monotonic - time.monotonic()
        if remaining <= 0.0:
            proposal_apply_diagnostics.record_rejected(
                "proposal_ttl_expired"
            )
            proposal_gate.watchdog(time.monotonic())
            do_stop("proposal_ttl_expired")
            return
        if not refresh_proposal_execution_deadline(
            proposal_gate.deadline_monotonic
        ):
            fault = current_proposal_watchdog_fault()
            reason = fault[1] if fault else "proposal_execution_fault"
            proposal_apply_diagnostics.record_rejected(reason)
            if reason == "proposal_ttl_expired":
                proposal_gate.watchdog(time.monotonic())
            else:
                proposal_gate.disarm(reason)
            do_stop(reason)
            return
        proposal_apply_diagnostics.record_accepted()
        handle_move(
            proposal.x,
            proposal.y,
            proposal.yaw,
            remaining,
            finite_rpc=True,
            source="velocity_proposal",
            proposal=proposal,
            deadline_monotonic=proposal_gate.deadline_monotonic,
        )

    @motion_synchronized
    def apply_safety_velocity(cmd, vx, vy, vyaw):
        """Queue a bounded parent-side proposal update during deceleration."""
        if cmd and cmd.get("source") == "velocity_proposal":
            remaining = proposal_gate.deadline_monotonic - time.monotonic()
            if remaining <= 0.0:
                proposal_gate.watchdog(time.monotonic())
                do_stop("proposal_ttl_expired")
                return
            if not cmd.get("nav_id") or cmd.get("sequence") is None:
                proposal_apply_diagnostics.record_rejected(
                    "proposal_identity_missing"
                )
                proposal_gate.disarm("proposal_identity_missing")
                do_stop("proposal_identity_missing")
                return
            target_velocity = (float(vx), float(vy), float(vyaw))
            effective_velocity = cmd.get("effective_velocity")
            if (
                effective_velocity is not None
                and not velocity_commands_differ(
                    effective_velocity,
                    target_velocity,
                )
            ):
                return False
            cmd["effective_velocity"] = target_velocity
            enqueue_proposal_apply({
                "vx": vx,
                "vy": vy,
                "vyaw": vyaw,
                "deadline_monotonic": proposal_gate.deadline_monotonic,
                "queued_monotonic": time.monotonic(),
                "nav_id": cmd["nav_id"],
                "sequence": cmd["sequence"],
            })
            return True
        loco_client.Move(vx, vy, vyaw, True)
        return True

    # ── Safety checks (inline in main loop) ──
    @motion_synchronized
    def process_safety_checks():
        nonlocal state, speed_zone

        # 1. Communication timeout
        with safety_lock:
            lowstate_age = time.monotonic() - last_lowstate_time
            temp = max_motor_temp

        if lowstate_age > comm_timeout and state in (MotionState.MOVING, MotionState.NAVIGATING, MotionState.NAV_PAUSED):
            emergency_stop("comm_timeout", {"last_msg_age_ms": round(lowstate_age * 1000)})
            return

        # 2. Joint overheat
        if state == MotionState.MOVING:
            if temp > motor_temp_stop:
                emergency_stop("joint_overheat", {"max_temp": round(temp, 1)})
                return
            elif temp > motor_temp_decel:
                if speed_zone != SpeedZone.DECELERATED:
                    speed_zone = SpeedZone.DECELERATED
                    if current_cmd:
                        dvx, dvy, dvyaw = clamp(current_cmd["vx"], current_cmd["vy"], current_cmd["vyaw"], SpeedZone.DECELERATED)
                        apply_safety_velocity(current_cmd, dvx, dvy, dvyaw)
                        publish_event("joint_overheat", {
                            "max_temp": round(temp, 1),
                            "action": "decelerate",
                            "new_speed": {"vx": dvx, "vy": dvy, "vyaw": dvyaw},
                        })
                    return

        # 3. Obstacle detection (original logic)
        with obstacle_lock:
            dist = obstacle_dist
            lateral = lateral_obstacle
            angle = obstacle_angle
            scan_heading = obstacle_scan_heading

        if state == MotionState.MOVING:
            cmd = current_cmd  # snapshot to avoid race condition
            if not obstacle_observation_applies(
                cmd,
                scan_heading,
                cone_half_angle,
            ):
                return
            if dist <= stop_threshold:
                if speed_zone != SpeedZone.STOPPED:
                    speed_zone = SpeedZone.STOPPED
                    print(f"[SmartMotion:obstacle] STOP — dist={dist:.2f}m angle={math.degrees(angle):.1f}° lateral={lateral}", flush=True)
                    do_stop("obstacle")
            elif dist <= decel_threshold or lateral:
                if speed_zone != SpeedZone.DECELERATED:
                    speed_zone = SpeedZone.DECELERATED
                    print(f"[SmartMotion:obstacle] DECEL — dist={dist:.2f}m angle={math.degrees(angle):.1f}° lateral={lateral}", flush=True)
                    if cmd:
                        dvx, dvy, dvyaw = clamp(cmd["vx"], cmd["vy"], cmd["vyaw"], SpeedZone.DECELERATED)
                        apply_safety_velocity(cmd, dvx, dvy, dvyaw)
                        with obstacle_lock:
                            od = obstacle_dist
                            oa = obstacle_angle
                        publish_event("motion_decelerate", {
                            "obstacle_distance": round(od, 2),
                            "obstacle_angle_deg": round(math.degrees(oa), 1),
                            "original_speed": {"vx": cmd["vx"], "vy": cmd["vy"], "vyaw": cmd["vyaw"]},
                            "new_speed": {"vx": dvx, "vy": dvy, "vyaw": dvyaw},
                        })
            else:
                if speed_zone == SpeedZone.DECELERATED:
                    speed_zone = SpeedZone.NORMAL
                    if cmd:
                        cvx, cvy, cvyaw = clamp(cmd["vx"], cmd["vy"], cmd["vyaw"], SpeedZone.NORMAL)
                        apply_safety_velocity(cmd, cvx, cvy, cvyaw)
                        publish_event("motion_resume", {"speed": {"vx": cvx, "vy": cvy, "vyaw": cvyaw}})

        elif state == MotionState.NAVIGATING:
            # In mode=1 (stop-on-obstacle), SLAM service handles obstacle stopping.
            # No local PauseNav/ResumeNav needed — avoids conflicts with SLAM state.
            pass

        elif state == MotionState.NAV_PAUSED:
            # Obstacle cleared — resume navigation.
            # Only resume if we were paused by local obstacle detection.
            if dist > decel_threshold and not lateral:
                try:
                    slam_client.ResumeNav()
                except Exception:
                    pass
                state = MotionState.NAVIGATING
                speed_zone = SpeedZone.NORMAL
                publish_event("nav_resumed", {})
                print(f"[SmartMotion:nav_obstacle] RESUME NAV — "
                      f"dist={dist:.2f}m, obstacle cleared", flush=True)

    def fsm_monitor_loop():
        """Poll main-control authorization independently of proposal callbacks."""
        while not safety_threads_shutdown.is_set():
            if not proposal_connected.wait(timeout=0.05):
                continue
            refresh_fsm_state()
            if safety_threads_shutdown.wait(proposal_fsm_check_interval):
                break

    def proposal_apply_loop():
        """Apply only the newest validated proposal without blocking ROS spin."""
        while not safety_threads_shutdown.is_set():
            try:
                command = proposal_apply_queue.get(timeout=0.05)
            except queue.Empty:
                continue

            with motion_command_lock:
                if (
                    proposal_stop_transition.is_set()
                    or not proposal_connected.is_set()
                    or not proposal_gate.armed
                    or command["nav_id"] != proposal_gate.expected_nav_id
                    or command["sequence"] != proposal_gate.last_sequence
                ):
                    continue
                now = time.monotonic()
                if now >= command["deadline_monotonic"]:
                    proposal_apply_diagnostics.record_rejected(
                        "proposal_ttl_expired"
                    )
                    proposal_gate.watchdog(now)
                    do_stop("proposal_ttl_expired")
                    continue
                runtime = proposal_runtime_status(force_fsm_check=False)
                if not runtime["ready"]:
                    reason = runtime["reason"] or "driver_safety_not_ready"
                    proposal_apply_diagnostics.record_rejected(reason)
                    proposal_gate.disarm(reason)
                    do_stop(reason)
                    continue
                generation = arm_proposal_execution(
                    command["deadline_monotonic"]
                )
                if generation is None:
                    fault = current_proposal_watchdog_fault()
                    reason = fault[1] if fault else "proposal_execution_fault"
                    proposal_apply_diagnostics.record_rejected(reason)
                    if reason == "proposal_ttl_expired":
                        proposal_gate.watchdog(time.monotonic())
                    else:
                        proposal_gate.disarm(reason)
                    do_stop(reason)
                    continue

            result = apply_parent_velocity_proposal(command)

            with motion_command_lock:
                watchdog_reason = proposal_fault_for_generation(generation)
                if watchdog_reason:
                    proposal_apply_diagnostics.record_rejected(
                        watchdog_reason
                    )
                    if watchdog_reason == "proposal_ttl_expired":
                        proposal_gate.watchdog(time.monotonic())
                    else:
                        proposal_gate.disarm(watchdog_reason)
                    do_stop(watchdog_reason)
                    continue

                if not proposal_apply_result_is_current(
                    command,
                    proposal_gate,
                    proposal_execution_lease,
                    generation,
                ):
                    # Stop/terminal/manual override cleared the generation, or
                    # a newer task/sequence superseded this result while the
                    # parent RPC was in flight.  It must not disarm or pollute
                    # diagnostics for the current lease.
                    continue

                proposal_apply_diagnostics.record_set_velocity(
                    result,
                    queued_monotonic=command["queued_monotonic"],
                )

                error = result.get("error")
                ret = result.get("ret")
                if error or ret != 0 or not result.get("applied"):
                    if error in {
                        "proposal_ttl_expired_before_rpc",
                        "proposal_ttl_expired_after_rpc",
                    }:
                        reason = "proposal_ttl_expired"
                    elif error == "parent_velocity_proposal_timeout":
                        reason = "parent_velocity_proposal_timeout"
                    else:
                        reason = "set_velocity_failed"
                    proposal_apply_diagnostics.record_rejected(reason)
                    if reason == "proposal_ttl_expired":
                        proposal_gate.request_ttl_stop(time.monotonic())
                    else:
                        proposal_gate.disarm(reason)
                    do_stop(reason)
                    continue
                proposal_apply_diagnostics.record_applied(
                    command["nav_id"],
                    command["sequence"],
                )

    def proposal_watchdog_loop():
        """At >=20 Hz, physically stop an active proposal on TTL or fault."""
        period = 1.0 / proposal_watchdog_hz
        while not safety_threads_shutdown.wait(period):
            now = time.monotonic()
            lease = proposal_execution_lease.watchdog_snapshot()
            if lease is None:
                continue
            generation, deadline = lease
            reason = "proposal_ttl_expired" if now >= deadline else ""
            if not reason:
                runtime = proposal_runtime_status(force_fsm_check=False)
                if not runtime["ready"]:
                    reason = runtime["reason"] or "driver_safety_not_ready"
            if not reason:
                continue
            def dispatch_watchdog_stop():
                proposal_apply_diagnostics.record_watchdog_fault(reason)
                try:
                    watchdog_loco_client.StopMove()
                except Exception:
                    pass

            if not proposal_watchdog_transition.trip_and_dispatch_stop(
                generation,
                reason,
                now,
                dispatch_watchdog_stop,
            ):
                continue

    def consume_proposal_watchdog_fault():
        fault = current_proposal_watchdog_fault()
        if not fault:
            return
        _, reason = fault
        with motion_command_lock:
            if current_proposal_watchdog_fault() != fault:
                return
            if reason == "proposal_ttl_expired":
                proposal_gate.watchdog(time.monotonic())
            else:
                proposal_gate.disarm(reason)
            do_stop(reason)

    def proposal_ros_spin_loop():
        """Service proposal/event callbacks independently of the busy loop."""
        nonlocal proposal_callback_error
        try:
            # Complete one non-blocking executor cycle before advertising that
            # binds can safely create the proposal subscription.
            executor.spin_once(timeout_sec=0)
            proposal_callback_ready.set()
            while not safety_threads_shutdown.is_set():
                executor.spin_once(timeout_sec=0.02)
        except Exception as exc:
            with proposal_callback_error_lock:
                proposal_callback_error = str(exc)
            proposal_callback_failed.set()
            proposal_callback_ready.clear()
            with motion_command_lock:
                if proposal_gate.connected_topic or proposal_gate.armed:
                    proposal_apply_diagnostics.record_rejected(
                        "proposal_callback_error"
                    )
                proposal_gate.disarm("proposal_callback_error")
                do_stop("proposal_callback_error")
            print(
                f"[SmartMotion] proposal ROS callback failed closed: {exc}",
                flush=True,
            )
        finally:
            proposal_callback_ready.clear()

    # ── Main loop ──
    fsm_monitor_thread = threading.Thread(
        target=fsm_monitor_loop,
        daemon=True,
        name="g1_loco_fsm_monitor",
    )
    proposal_watchdog_thread = threading.Thread(
        target=proposal_watchdog_loop,
        daemon=True,
        name="g1_loco_velocity_watchdog",
    )
    proposal_apply_thread = threading.Thread(
        target=proposal_apply_loop,
        daemon=True,
        name="g1_loco_velocity_apply",
    )
    proposal_ros_thread = threading.Thread(
        target=proposal_ros_spin_loop,
        daemon=True,
        name="g1_loco_proposal_ros",
    )
    proposal_ros_thread.start()
    if not proposal_callback_ready.wait(timeout=1.0):
        with proposal_callback_error_lock:
            if proposal_callback_error is None:
                proposal_callback_error = "proposal_callback_start_timeout"
        proposal_callback_failed.set()
    fsm_monitor_thread.start()
    proposal_watchdog_thread.start()
    proposal_apply_thread.start()
    print(f"[SmartMotion:pid={os.getpid()}] entering main loop")
    running = True
    last_obstacle_check = 0.0
    last_slam_info_log = 0.0

    while running:
        try:
            consume_proposal_watchdog_fault()
        except Exception as exc:
            with motion_command_lock:
                proposal_gate.disarm("watchdog_fault_handler_error")
                try:
                    stop_loco_client.StopMove()
                except Exception:
                    pass
            print(
                f"[SmartMotion] watchdog fault handling failed closed: {exc}",
                flush=True,
            )
        # Process commands (non-blocking)
        cmd = None
        try:
            cmd = cmd_queue.get(timeout=0.05)
            method = cmd.get("method")
            result = None

            if method == "move":
                with motion_command_lock:
                    proposal_gate.disarm("manual_override")
                    result = handle_move(
                        cmd["vx"], cmd["vy"], cmd["vyaw"], cmd["duration"],
                        source="mcp",
                    )
            elif method == "stop":
                with motion_command_lock:
                    proposal_gate.disarm("manual_stop")
                    if state in (MotionState.NAVIGATING, MotionState.NAV_PAUSED):
                        do_stop_nav()
                    result = do_stop(cmd.get("reason", "command"))
            elif method == "navigate_to":
                with motion_command_lock:
                    proposal_gate.disarm("native_navigation_override")
                    result = handle_navigate_to(
                        cmd["x"], cmd["y"], cmd["yaw"],
                        cmd.get("target_name", ""),
                        speed=cmd.get("speed", 0.5),
                        mode=cmd.get("mode", 1),
                        stall_timeout=cmd.get("stall_timeout", 60),
                    )
            elif method == "pause_nav":
                result = handle_pause_nav(cmd.get("reason", "command"))
            elif method == "resume_nav":
                result = handle_resume_nav()
            elif method == "stop_nav":
                with motion_command_lock:
                    proposal_gate.disarm("native_stop_nav")
                    if state == MotionState.MOVING:
                        result = do_stop("native_stop_nav")
                    else:
                        result = do_stop_nav()
            elif method == "wait_nav_done":
                result = handle_wait_nav_done(cmd.get("stall_timeout", 60))
            elif method == "get_state":
                result = handle_get_state()
            elif method == "begin_velocity_proposal_stop_confirmation":
                result = handle_begin_velocity_proposal_stop_confirmation()
            elif method == "bind_velocity_proposal":
                try:
                    result = handle_bind_velocity_proposal(
                        cmd.get("topic", ""),
                        cmd.get("expected_nav_id"),
                        external_stop_attempt=cmd.get(
                            "external_stop_attempt"
                        ),
                    )
                finally:
                    proposal_stop_transition.clear()
            elif method == "unbind_velocity_proposal":
                try:
                    result = handle_unbind_velocity_proposal(
                        cmd.get("reason", "canvas_stop"),
                        external_stop_attempt=cmd.get(
                            "external_stop_attempt"
                        ),
                    )
                finally:
                    proposal_stop_transition.clear()
            elif method == "get_velocity_proposal_status":
                result = handle_get_velocity_proposal_status()
            elif method == "start_mapping":
                code, resp = slam_client.StartMapping()
                result = {"code": code, "response": resp}
            elif method == "stop_mapping":
                pcd_path = cmd["pcd_path"]
                slam_client.SetTimeout(15.0)
                try:
                    code, resp = slam_client.StopMapping(pcd_path)
                finally:
                    slam_client.SetTimeout(10.0)
                result = {"code": code, "response": resp}
            elif method == "init_pose":
                code, resp = slam_client.InitPose(
                    cmd.get("x", 0.0), cmd.get("y", 0.0), cmd.get("z", 0.0),
                    cmd.get("q_x", 0.0), cmd.get("q_y", 0.0),
                    cmd.get("q_z", 0.0), cmd.get("q_w", 1.0),
                    cmd.get("address", "")
                )
                result = {"code": code, "response": resp}
            elif method == "shutdown":
                handle_unbind_velocity_proposal("process_shutdown")
                running = False
                result = {"status": "shutdown"}
            else:
                result = {"error": f"unknown SmartMotion method: {method}"}

            if result is not None:
                result_queue.put({
                    "request_id": cmd.get("request_id"),
                    "result": result,
                })
        except queue.Empty:
            pass
        except Exception as exc:
            with motion_command_lock:
                proposal_gate.disarm("smart_motion_command_error")
                proposal_stop_transition.clear()
                try:
                    stop_loco_client.StopMove()
                except Exception:
                    pass
            if cmd is not None:
                result_queue.put({
                    "request_id": cmd.get("request_id"),
                    "result": {"error": f"SmartMotion command failed: {exc}"},
                })

        # Safety checks at 10Hz
        now = time.monotonic()
        if now - last_obstacle_check >= 0.1:
            last_obstacle_check = now
            try:
                process_safety_checks()
            except Exception as exc:
                with motion_command_lock:
                    proposal_gate.disarm("safety_check_error")
                    try:
                        stop_loco_client.StopMove()
                    except Exception:
                        pass
                print(
                    f"[SmartMotion] safety check failed closed: {exc}",
                    flush=True,
                )

            # Repeat StopMove after emergency_stop to ensure controller receives it
            with motion_command_lock:
                if stop_repeat_count > 0 and state == MotionState.IDLE:
                    try:
                        stop_loco_client.StopMove()
                    except Exception:
                        pass
                    stop_repeat_count -= 1

        # Periodic health log: verify slam_info subscription is working
        if now - last_slam_info_log >= 10.0:
            last_slam_info_log = now
            print(f"[SmartMotion] health: slam_info msgs={slam_info_msg_count} "
                  f"pos_info={slam_info_pos_count} state={state.value}", flush=True)

    # Cleanup
    safety_threads_shutdown.set()
    proposal_connected.clear()
    try:
        executor.wake()
    except Exception:
        pass
    proposal_ros_thread.join(timeout=0.5)
    proposal_watchdog_thread.join(timeout=0.5)
    fsm_monitor_thread.join(timeout=0.75)
    proposal_apply_thread.join(timeout=proposal_rpc_timeout + 0.25)
    if move_timer:
        move_timer.cancel()
    with motion_command_lock:
        do_stop("process_exit", confirm_physical_stop=True)
        proposal_gate.unbind("process_exit")
        proposal_node.unbind()
    executor.shutdown()
    rclpy.shutdown()
    print(f"[SmartMotion:pid={os.getpid()}] shutdown complete")
