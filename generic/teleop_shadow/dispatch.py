"""Final-dispatch arbitration for the robot-free teleop Shadow Driver.

The adapter in this package records what a live adapter *would* receive.  It
never imports a robot SDK or publishes a hardware command.  The arbiter is the
executable reference for the safety boundary that later live adapters must
implement: a one-slot motion mailbox, a non-droppable stop path, generation
fencing, final deadline checks, and acknowledged startup/shutdown safety.
"""

from __future__ import annotations

import copy
import hashlib
import hmac
import json
import math
import re
import threading
import time
from collections import Counter, deque
from collections.abc import Callable, Mapping
from dataclasses import dataclass
from typing import Any, Protocol

from protocol import MAX_SEQUENCE

_TRACKING_KEYS = frozenset({"head", "left_controller", "right_controller"})
_ACK_CODE_RE = re.compile(r"^[a-z][a-z0-9_]{0,63}$")


def _tracking_ready(value: Any) -> bool:
    return (
        isinstance(value, Mapping)
        and set(value) == _TRACKING_KEYS
        and all(item is True for item in value.values())
    )


def _normalized_ack(value: Any, invalid_code: str) -> AdapterAck:
    if not isinstance(value, AdapterAck) or not isinstance(value.ok, bool):
        return AdapterAck(False, invalid_code)
    if not isinstance(value.code, str) or not _ACK_CODE_RE.fullmatch(value.code):
        return AdapterAck(False, invalid_code)
    return value


@dataclass(frozen=True)
class AdapterAck:
    """Bounded adapter acknowledgement with a stable, non-secret code."""

    ok: bool
    code: str = "ok"


@dataclass(frozen=True)
class MotionIntent:
    """Sanitized motion intent delivered to an output adapter.

    ``frame`` deliberately omits the fence.  Session and dispatch generations
    are internal monotonic fences and are safe to expose to a test adapter.
    """

    session_generation: int
    dispatch_generation: int
    sequence: int
    clutch_sequence: int
    admitted_monotonic: float
    expires_monotonic: float
    frame: Mapping[str, Any]


@dataclass(frozen=True)
class StopRequest:
    dispatch_generation: int
    reason: str
    requested_monotonic: float
    deadline_monotonic: float


class OutputAdapter(Protocol):
    """Serial adapter contract used by the final-dispatch owner thread.

    Implementations must enforce their deadline in the same critical section as
    the real SDK write. ``apply`` may not leave an unbounded background output;
    any asynchronous downstream work must carry the supplied generation and a
    hardware-enforced TTL. A successful ``safe_stop`` ack means all older
    generations are cancelled, drained, or made incapable of later output.
    Adapter methods must also impose their own downstream I/O timeout: Python
    can detect a stalled call and revoke authority, but cannot interrupt a
    blocking vendor SDK call safely. Adapter methods must not re-enter the
    arbiter; a defensive self-close rejection exists only to avoid deadlock.
    """

    def startup_safe(self, deadline_monotonic: float) -> AdapterAck: ...

    def apply(self, intent: MotionIntent) -> AdapterAck: ...

    def safe_stop(self, request: StopRequest) -> AdapterAck: ...

    def snapshot(self) -> dict: ...

    def close(self) -> AdapterAck | None: ...


class StopHandle:
    """Waitable result for one non-droppable safety request."""

    def __init__(self, request: StopRequest, *, safety_deadline: float):
        self.request = request
        self.safety_deadline = safety_deadline
        self._event = threading.Event()
        self._ack: AdapterAck | None = None

    def _finish(self, ack: AdapterAck) -> None:
        self._ack = ack
        self._event.set()

    def wait(self, timeout: float) -> AdapterAck:
        if not self._event.wait(timeout):
            return AdapterAck(False, "stop_ack_timeout")
        return self._ack or AdapterAck(False, "stop_ack_missing")

    @property
    def done(self) -> bool:
        return self._event.is_set()


@dataclass(frozen=True)
class _PendingMotion:
    intent: MotionIntent
    authority_digest: str


def _binding_digest(value: Mapping[str, Any]) -> str:
    required = ("boot_id", "session_id", "epoch", "fence")
    if any(key not in value for key in required):
        raise ValueError("authority binding is incomplete")
    encoded = json.dumps(
        {key: value[key] for key in required},
        ensure_ascii=False,
        allow_nan=False,
        separators=(",", ":"),
        sort_keys=True,
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


class RecordingAdapter:
    """Bounded, thread-safe zero-actuation adapter for visible verification."""

    def __init__(
        self,
        *,
        clock: Callable[[], float] = time.monotonic,
        max_records: int = 64,
    ):
        if max_records < 8 or max_records > 4096:
            raise ValueError("max_records must be in [8, 4096]")
        self._clock = clock
        self._lock = threading.Lock()
        self._records: deque[dict] = deque(maxlen=max_records)
        self._current = {"kind": "safe", "reason": "not_started"}
        self._closed = False

    def _record(self, value: dict) -> None:
        record = {"recorded_monotonic": self._clock(), **copy.deepcopy(value)}
        self._records.append(record)
        self._current = copy.deepcopy(value)

    def startup_safe(self, deadline_monotonic: float) -> AdapterAck:
        with self._lock:
            if self._closed:
                return AdapterAck(False, "adapter_closed")
            self._record({
                "kind": "would_stop",
                "reason": "startup_safe",
                "dispatch_generation": 0,
                "deadline_monotonic": deadline_monotonic,
            })
            if self._clock() >= deadline_monotonic:
                return AdapterAck(False, "startup_safe_deadline_missed")
            return AdapterAck(True)

    def apply(self, intent: MotionIntent) -> AdapterAck:
        with self._lock:
            if self._closed:
                return AdapterAck(False, "adapter_closed")
            if self._clock() >= intent.expires_monotonic:
                return AdapterAck(False, "intent_expired_at_adapter")
            # The adapter contract itself refuses unsafe content even if a
            # caller accidentally bypasses the runtime admission layer.
            if intent.frame.get("deadman") is not True:
                return AdapterAck(False, "unsafe_deadman")
            if not _tracking_ready(intent.frame.get("tracking")):
                return AdapterAck(False, "unsafe_tracking")
            self._record({
                "kind": "would_apply",
                "sequence": intent.sequence,
                "clutch_sequence": intent.clutch_sequence,
                "session_generation": intent.session_generation,
                "dispatch_generation": intent.dispatch_generation,
                "base_twist": copy.deepcopy(intent.frame.get("base_twist")),
            })
            return AdapterAck(True)

    def safe_stop(self, request: StopRequest) -> AdapterAck:
        with self._lock:
            if self._closed:
                return AdapterAck(False, "adapter_closed")
            self._record({
                "kind": "would_stop",
                "reason": request.reason,
                "dispatch_generation": request.dispatch_generation,
                "deadline_monotonic": request.deadline_monotonic,
            })
            if self._clock() >= request.deadline_monotonic:
                return AdapterAck(False, "adapter_stop_deadline_missed")
            return AdapterAck(True)

    def snapshot(self) -> dict:
        with self._lock:
            return {
                "kind": "recording",
                "closed": self._closed,
                "current": copy.deepcopy(self._current),
                "records": copy.deepcopy(list(self._records)),
            }

    def close(self) -> None:
        with self._lock:
            self._closed = True


class FinalDispatchArbiter:
    """Own all adapter I/O and enforce the last command authority check."""

    _MOTION_STATES = frozenset({"safe_waiting_frame", "motion_eligible"})

    def __init__(
        self,
        adapter: OutputAdapter,
        *,
        clock: Callable[[], float] = time.monotonic,
        safety_clock: Callable[[], float] = time.monotonic,
        io_timeout_ms: int = 250,
    ):
        if io_timeout_ms < 20 or io_timeout_ms > 5000:
            raise ValueError("io_timeout_ms must be in [20, 5000]")
        self._adapter = adapter
        self._clock = clock
        self._safety_clock = safety_clock
        self._io_timeout = io_timeout_ms / 1000.0
        self._condition = threading.Condition(threading.RLock())
        self._stop_queue: deque[StopHandle] = deque()
        self._mailbox: _PendingMotion | None = None
        self._generation = 0
        self._authority_digest: str | None = None
        self._session_generation: int | None = None
        self._state = "starting"
        self._fault_code: str | None = None
        self._stop_acknowledged = False
        self._last_admitted_sequence: int | None = None
        self._last_applied_sequence: int | None = None
        self._last_decision = "starting"
        self._counters: Counter[str] = Counter()
        self._shutdown = False
        self._closed = False
        self._close_lock = threading.Lock()
        self._adapter_snapshot_lock = threading.Lock()
        self._close_result: AdapterAck | None = None
        self._adapter_close_ack: AdapterAck | None = None
        self._adapter_snapshot: dict = {
            "kind": "unavailable",
            "reason": "startup_in_progress",
        }
        self._io_inflight_kind: str | None = None
        self._io_deadline_safety: float | None = None
        self._startup_done = threading.Event()
        self._startup_decision = threading.Event()
        self._startup_owner_done = threading.Event()
        # Fail-safe default: unless the constructor explicitly hands the
        # adapter to the final-dispatch worker, the startup owner closes it
        # after startup_safe eventually returns.
        self._startup_should_close = True

        started_at = self._clock()
        started_safety = self._safety_clock()
        startup_result: list[AdapterAck] = []

        def run_startup_safe() -> None:
            try:
                startup_result.append(_normalized_ack(
                    adapter.startup_safe(started_at + self._io_timeout),
                    "invalid_startup_ack",
                ))
            except Exception:  # noqa: BLE001 -- normalize adapter faults
                startup_result.append(AdapterAck(False, "startup_safe_exception"))
            finally:
                self._capture_adapter_snapshot_owned()
                self._startup_done.set()
            # The startup call may outlive the constructor timeout.  It keeps
            # exclusive adapter ownership until the constructor chooses either
            # a worker hand-off or cleanup; close/snapshot never race it.
            self._startup_decision.wait()
            try:
                if self._startup_should_close:
                    self._close_adapter_owned()
            finally:
                self._startup_owner_done.set()

        self._startup_thread = threading.Thread(
            target=run_startup_safe,
            daemon=True,
            name="teleop-shadow-startup-safe",
        )
        self._startup_thread.start()
        completed = self._startup_done.wait(self._io_timeout)
        elapsed_safety = self._safety_clock() - started_safety
        ack = (
            startup_result[0]
            if completed and startup_result
            else AdapterAck(False, "startup_safe_timeout")
        )
        if ack.ok and elapsed_safety <= self._io_timeout:
            self._state = "safe_unarmed"
            self._stop_acknowledged = True
            self._last_decision = "startup_safe_ack"
            self._counters["startup_safe_acks"] += 1
        else:
            self._state = "fault_latched"
            self._fault_code = "startup_safe_timeout" if ack.ok else ack.code
            self._last_decision = "startup_safe_failed"
            self._counters["adapter_faults"] += 1

        self._worker: threading.Thread | None = None
        if self._fault_code is None:
            self._startup_should_close = False
        self._startup_decision.set()
        if self._fault_code is None:
            # The startup owner performs no adapter calls after hand-off.  This
            # short wait also prevents successful starts from leaving a
            # transient daemon behind in short-lived tests.
            self._startup_owner_done.wait(self._io_timeout)
            self._worker = threading.Thread(
                target=self._worker_loop,
                daemon=True,
                name="teleop-shadow-final-dispatch",
            )
            self._worker.start()

    @property
    def ready(self) -> bool:
        with self._condition:
            self._refresh_io_stall_locked()
            return self._fault_code is None and not self._closed

    def begin_prepare(self) -> StopHandle:
        return self.trip(
            "prepare_shadow",
            target_state="transitioning",
            retain_authority=False,
        )

    def complete_prepare(
        self,
        handle: StopHandle,
        binding: Mapping[str, Any],
        session_generation: int,
    ) -> AdapterAck:
        digest = _binding_digest(binding)
        with self._condition:
            if self._closed:
                return AdapterAck(False, "dispatch_closed")
            if self._fault_code is not None:
                return AdapterAck(False, self._fault_code)
            if (
                handle.request.dispatch_generation != self._generation
                or not handle.done
                or self._state != "transitioning"
            ):
                return AdapterAck(False, "prepare_superseded")
            ack = handle.wait(0)
            if not ack.ok:
                return ack
            self._authority_digest = digest
            self._session_generation = session_generation
            self._state = "safe_waiting_frame"
            self._stop_acknowledged = True
            self._last_admitted_sequence = None
            self._last_applied_sequence = None
            self._last_decision = "prepared_after_stop_ack"
            self._counters["sessions_armed"] += 1
            return AdapterAck(True)

    def publish_latest(
        self,
        frame: Mapping[str, Any],
        *,
        session_generation: int,
        expires_monotonic: float,
        allow_reclutch: bool = False,
    ) -> AdapterAck:
        if frame.get("deadman") is not True:
            return AdapterAck(False, "unsafe_deadman")
        if not _tracking_ready(frame.get("tracking")):
            return AdapterAck(False, "unsafe_tracking")
        try:
            digest = _binding_digest(frame)
            sequence = frame["sequence"]
            clutch_sequence = frame["clutch_sequence"]
            if (
                isinstance(sequence, bool)
                or not isinstance(sequence, int)
                or sequence < 0
                or sequence > MAX_SEQUENCE
            ):
                raise ValueError("invalid sequence")
            if (
                isinstance(clutch_sequence, bool)
                or not isinstance(clutch_sequence, int)
                or clutch_sequence < 0
                or clutch_sequence > MAX_SEQUENCE
            ):
                raise ValueError("invalid clutch sequence")
            expiry = float(expires_monotonic)
            if not math.isfinite(expiry):
                raise ValueError("invalid expiry")
        except (KeyError, TypeError, ValueError, OverflowError):
            return AdapterAck(False, "invalid_intent")

        admitted = self._clock()
        sanitized = copy.deepcopy(dict(frame))
        sanitized.pop("fence", None)
        with self._condition:
            if self._closed:
                return AdapterAck(False, "dispatch_closed")
            if self._fault_code is not None:
                return AdapterAck(False, self._fault_code)
            if self._session_generation != session_generation:
                return AdapterAck(False, "stale_session_generation")
            if self._authority_digest is None or not hmac.compare_digest(
                digest, self._authority_digest
            ):
                return AdapterAck(False, "authority_mismatch")
            if self._state == "safe_reclutch_required" and allow_reclutch:
                self._state = "motion_eligible"
                self._counters["dispatch_reclutches"] += 1
            elif self._state not in self._MOTION_STATES:
                return AdapterAck(False, "motion_inhibited")
            if expiry <= admitted:
                return AdapterAck(False, "intent_expired")
            if (
                self._last_admitted_sequence is not None
                and sequence <= self._last_admitted_sequence
            ):
                return AdapterAck(False, "sequence_not_increasing")
            if self._mailbox is not None:
                self._counters["mailbox_replacements"] += 1
            intent = MotionIntent(
                session_generation=session_generation,
                dispatch_generation=self._generation,
                sequence=sequence,
                clutch_sequence=clutch_sequence,
                admitted_monotonic=admitted,
                expires_monotonic=expiry,
                frame=sanitized,
            )
            self._mailbox = _PendingMotion(intent, digest)
            self._state = "motion_eligible"
            self._last_admitted_sequence = sequence
            self._last_decision = "admitted"
            self._counters["motion_admitted"] += 1
            self._condition.notify_all()
            return AdapterAck(True)

    def trip(
        self,
        reason: str,
        *,
        target_state: str,
        retain_authority: bool,
    ) -> StopHandle:
        if not isinstance(reason, str) or not reason:
            raise ValueError("stop reason is required")
        with self._condition:
            self._generation += 1
            now = self._clock()
            request = StopRequest(
                dispatch_generation=self._generation,
                reason=reason,
                requested_monotonic=now,
                deadline_monotonic=now + self._io_timeout,
            )
            handle = StopHandle(
                request,
                safety_deadline=self._safety_clock() + self._io_timeout,
            )
            if self._mailbox is not None:
                self._mailbox = None
                self._counters["mailbox_cleared_by_stop"] += 1
            self._state = target_state
            self._stop_acknowledged = False
            self._last_decision = f"stop_requested:{reason}"
            if not retain_authority:
                self._authority_digest = None
                self._session_generation = None
            if self._closed or self._shutdown:
                handle._finish(AdapterAck(False, "dispatch_closed"))
            elif self._worker is None:
                handle._finish(AdapterAck(False, "dispatch_unavailable"))
            else:
                self._stop_queue.append(handle)
                self._counters["stop_requests"] += 1
                self._condition.notify_all()
            return handle

    @staticmethod
    def wait_safe(handle: StopHandle, timeout: float = 0.5) -> AdapterAck:
        return handle.wait(timeout)

    def wait_applied(self, sequence: int, timeout: float = 1.0) -> bool:
        deadline = time.monotonic() + timeout
        with self._condition:
            while self._last_applied_sequence != sequence:
                remaining = deadline - time.monotonic()
                if remaining <= 0 or self._fault_code is not None or self._closed:
                    return False
                self._condition.wait(remaining)
            return True

    def snapshot(self) -> dict:
        with self._condition:
            self._refresh_io_stall_locked()
            state = {
                "kind": "recording",
                "state": self._state,
                "ready": self._fault_code is None and not self._closed,
                "generation": self._generation,
                "mailbox_depth": int(self._mailbox is not None),
                "stop_queue_depth": len(self._stop_queue),
                "last_admitted_sequence": self._last_admitted_sequence,
                "last_would_apply_sequence": self._last_applied_sequence,
                "last_decision": self._last_decision,
                "stop_acknowledged": self._stop_acknowledged,
                "fault_code": self._fault_code,
                "io_inflight": self._io_inflight_kind,
                "counters": dict(sorted(self._counters.items())),
            }
        with self._adapter_snapshot_lock:
            cached_adapter = self._adapter_snapshot
        # Cache entries are replaced, never mutated.  Copying outside both
        # arbiter locks prevents high-rate status reads from starving the I/O
        # owner while still returning an isolated public value.
        state["adapter"] = copy.deepcopy(cached_adapter)
        return state

    def _refresh_io_stall_locked(self) -> None:
        if (
            self._io_inflight_kind is None
            or self._io_deadline_safety is None
            or self._fault_code is not None
            or self._safety_clock() < self._io_deadline_safety
        ):
            return
        self._fault_code = "adapter_io_stalled"
        self._state = "fault_latched"
        self._authority_digest = None
        self._session_generation = None
        self._mailbox = None
        self._stop_acknowledged = False
        self._last_decision = f"io_stalled:{self._io_inflight_kind}"
        self._counters["adapter_faults"] += 1
        self._counters["adapter_io_stalls"] += 1
        # The owner may currently be blocked, but the stop remains queued and
        # will be its first operation if the vendor call ever returns.
        self.trip(
            "adapter_io_stalled",
            target_state="fault_latched",
            retain_authority=False,
        )

    def close(self, timeout: float = 0.5) -> AdapterAck:
        if self._worker is threading.current_thread():
            return AdapterAck(False, "owner_cannot_self_close")
        with self._close_lock:
            unavailable_result: AdapterAck | None = None
            with self._condition:
                if self._close_result is not None:
                    return self._close_result
                worker = self._worker
                if worker is None:
                    code = self._fault_code or "dispatch_unavailable"
                    self._closed = True
                    self._shutdown = True
                    self._state = "fault_latched"
                    self._stop_acknowledged = False
                    self._close_result = AdapterAck(False, code)
                    unavailable_result = self._close_result
            if unavailable_result is not None:
                # A timed-out startup call still owns the adapter.  It will
                # close it serially if it ever returns; wait only within the
                # caller's shutdown budget.
                if self._startup_thread is not threading.current_thread():
                    self._startup_owner_done.wait(timeout)
                return unavailable_result

            handle = self.trip(
                "service_close",
                target_state="closing",
                retain_authority=False,
            )
            stop_ack = self.wait_safe(handle, timeout)
            with self._condition:
                self._closed = True
                self._shutdown = True
                self._mailbox = None
                self._state = "closing" if stop_ack.ok else "fault_latched"
                if not stop_ack.ok:
                    self._fault_code = self._fault_code or stop_ack.code
                self._condition.notify_all()
            worker.join(timeout=max(timeout, self._io_timeout * 2))
            with self._condition:
                if worker.is_alive():
                    result = AdapterAck(False, stop_ack.code if not stop_ack.ok else "owner_close_timeout")
                elif not stop_ack.ok:
                    result = stop_ack
                elif self._adapter_close_ack is None:
                    result = AdapterAck(False, "adapter_close_ack_missing")
                else:
                    result = self._adapter_close_ack
                self._close_result = result
                self._state = "closed" if result.ok else "fault_latched"
                if not result.ok:
                    self._fault_code = self._fault_code or result.code
                    self._stop_acknowledged = False
                return result

    def _worker_loop(self) -> None:
        try:
            while True:
                with self._condition:
                    while not self._stop_queue and self._mailbox is None and not self._shutdown:
                        self._condition.wait()
                    if self._shutdown and not self._stop_queue:
                        break
                    if self._stop_queue:
                        item: StopHandle | _PendingMotion = self._stop_queue.popleft()
                        is_stop = True
                    else:
                        assert self._mailbox is not None
                        item = self._mailbox
                        self._mailbox = None
                        is_stop = False
                if is_stop:
                    assert isinstance(item, StopHandle)
                    self._perform_stop(item)
                else:
                    assert isinstance(item, _PendingMotion)
                    self._perform_motion(item)
        finally:
            self._close_adapter_owned()

    def _close_adapter_owned(self) -> None:
        """Close the adapter from its sole current owner thread."""

        try:
            raw_ack = self._adapter.close()
            close_ack = _normalized_ack(
                AdapterAck(True) if raw_ack is None else raw_ack,
                "invalid_close_ack",
            )
        except Exception:  # noqa: BLE001 -- normalize adapter close faults
            close_ack = AdapterAck(False, "adapter_close_exception")
        self._capture_adapter_snapshot_owned()
        with self._condition:
            self._adapter_close_ack = close_ack
            self._condition.notify_all()

    def _capture_adapter_snapshot_owned(self) -> None:
        """Refresh public adapter evidence without crossing the owner boundary."""

        try:
            snapshot = self._adapter.snapshot()
            if not isinstance(snapshot, dict):
                snapshot = {
                    "kind": "unavailable",
                    "reason": "invalid_adapter_snapshot",
                }
            else:
                snapshot = copy.deepcopy(snapshot)
        except Exception:  # noqa: BLE001 -- status must survive adapter diagnostics
            snapshot = {
                "kind": "unavailable",
                "reason": "adapter_snapshot_exception",
            }
        with self._adapter_snapshot_lock:
            self._adapter_snapshot = snapshot

    def _perform_stop(self, handle: StopHandle) -> None:
        request = handle.request
        started_safety = self._safety_clock()
        with self._condition:
            self._io_inflight_kind = "safe_stop"
            self._io_deadline_safety = handle.safety_deadline
        try:
            ack = _normalized_ack(
                self._adapter.safe_stop(request),
                "invalid_stop_ack",
            )
            self._capture_adapter_snapshot_owned()
        except Exception:  # noqa: BLE001 -- normalize adapter faults
            ack = AdapterAck(False, "adapter_stop_exception")
        finished_safety = self._safety_clock()
        elapsed_safety = finished_safety - started_safety
        if ack.ok:
            if finished_safety >= handle.safety_deadline:
                ack = AdapterAck(False, "adapter_stop_deadline_missed")
            elif elapsed_safety > self._io_timeout:
                ack = AdapterAck(False, "adapter_stop_timeout")
        with self._condition:
            self._io_inflight_kind = None
            self._io_deadline_safety = None
            handle._finish(ack)
            if ack.ok:
                self._counters["stop_acks"] += 1
                if request.dispatch_generation == self._generation:
                    self._stop_acknowledged = True
                    self._last_decision = f"would_stop:{request.reason}"
            else:
                self._fault_code = self._fault_code or ack.code
                self._state = "fault_latched"
                self._authority_digest = None
                self._session_generation = None
                self._mailbox = None
                self._stop_acknowledged = False
                self._last_decision = f"stop_failed:{ack.code}"
                self._counters["adapter_faults"] += 1
            self._condition.notify_all()

    def _perform_motion(self, pending: _PendingMotion) -> None:
        intent = pending.intent
        with self._condition:
            now = self._clock()
            valid = (
                self._fault_code is None
                and not self._closed
                and self._state == "motion_eligible"
                and intent.dispatch_generation == self._generation
                and intent.session_generation == self._session_generation
                and self._authority_digest is not None
                and hmac.compare_digest(pending.authority_digest, self._authority_digest)
            )
            if not valid:
                self._last_decision = "motion_dropped_stale"
                self._counters["motion_dropped_stale"] += 1
                self._condition.notify_all()
                return
            if now >= intent.expires_monotonic:
                self._last_decision = "motion_dropped_expired"
                self._counters["motion_dropped_expired"] += 1
                self.trip(
                    "dispatch_deadline",
                    target_state="safe_reclutch_required",
                    retain_authority=True,
                )
                self._condition.notify_all()
                return
            # This is the linearization point.  A concurrent trip after this
            # commit is queued behind the bounded apply and cannot acknowledge
            # until its safe-stop has executed.
            self._last_decision = "motion_committed"
            remaining = max(0.0, intent.expires_monotonic - now)
            started_safety = self._safety_clock()
            self._io_inflight_kind = "apply"
            self._io_deadline_safety = started_safety + min(self._io_timeout, remaining)

        try:
            ack = _normalized_ack(
                self._adapter.apply(intent),
                "invalid_apply_ack",
            )
            self._capture_adapter_snapshot_owned()
        except Exception:  # noqa: BLE001 -- normalize adapter faults
            ack = AdapterAck(False, "adapter_apply_exception")
        finished = self._clock()
        finished_safety = self._safety_clock()
        elapsed_safety = finished_safety - started_safety
        if ack.ok:
            if finished >= intent.expires_monotonic:
                ack = AdapterAck(False, "adapter_apply_deadline_missed")
            elif elapsed_safety > self._io_timeout:
                ack = AdapterAck(False, "adapter_apply_timeout")
        with self._condition:
            self._io_inflight_kind = None
            self._io_deadline_safety = None
            if (
                ack.ok
                and self._fault_code is None
                and not self._closed
                and intent.dispatch_generation == self._generation
            ):
                self._last_applied_sequence = intent.sequence
                self._last_decision = "would_apply"
                self._counters["motion_applied"] += 1
            elif ack.ok:
                self._counters["late_adapter_returns"] += 1
            else:
                already_faulted = self._fault_code is not None
                self._fault_code = self._fault_code or ack.code
                self._state = "fault_latched"
                self._authority_digest = None
                self._session_generation = None
                self._mailbox = None
                self._stop_acknowledged = False
                self._last_decision = f"apply_failed:{self._fault_code}"
                if not already_faulted:
                    self._counters["adapter_faults"] += 1
                # A failed motion write still schedules the independent safety
                # path.  Its acknowledgement cannot clear the fault latch.
                if not already_faulted:
                    self.trip(
                        "adapter_fault",
                        target_state="fault_latched",
                        retain_authority=False,
                    )
            self._condition.notify_all()
