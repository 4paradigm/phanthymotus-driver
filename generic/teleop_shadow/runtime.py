"""Thread-safe, robot-free teleoperation shadow session runtime."""

from __future__ import annotations

import copy
import re
import secrets
import threading
import time
import uuid
from collections import Counter
from collections.abc import Callable, Mapping
from typing import Any

from dispatch import FinalDispatchArbiter, RecordingAdapter, StopHandle
from protocol import (
    CAPABILITIES,
    CAPABILITY_DIGEST,
    MODE,
    ProtocolError,
    bind_rtc_frame_v1,
    validate_frame_v1,
)

SESSION_STATES = {
    "idle",
    "prepared_shadow",
    "active_shadow",
    "hold",
    "paused",
    "released",
    "fault",
}


class ShadowRuntime:
    """Own fencing, watchdogs and a recording-only final dispatch boundary."""

    def __init__(
        self,
        *,
        lease_timeout_ms: int = 1000,
        pose_timeout_ms: int = 200,
        watchdog_interval_ms: int = 25,
        driver_id: str = "teleop-shadow-driver",
        driver_name: str = "Generic Teleop Shadow Diagnostics",
        robot_id: str | None = None,
        clock: Callable[[], float] = time.monotonic,
        auto_watchdog: bool = True,
        dispatch_io_timeout_ms: int = 100,
        dispatch_ack_timeout_ms: int = 200,
        dispatcher: FinalDispatchArbiter | None = None,
    ):
        if lease_timeout_ms < 100:
            raise ValueError("lease_timeout_ms must be >= 100")
        if pose_timeout_ms < 20:
            raise ValueError("pose_timeout_ms must be >= 20")
        if watchdog_interval_ms < 5:
            raise ValueError("watchdog_interval_ms must be >= 5")
        if dispatch_io_timeout_ms < 20 or dispatch_io_timeout_ms > 150:
            raise ValueError("dispatch_io_timeout_ms must be in [20, 150]")
        if (
            dispatch_ack_timeout_ms < dispatch_io_timeout_ms
            or dispatch_ack_timeout_ms > 200
        ):
            raise ValueError(
                "dispatch_ack_timeout_ms must be >= dispatch_io_timeout_ms and <= 200"
            )
        self.boot_id = str(uuid.uuid4())
        self.driver_id = driver_id
        self.driver_name = driver_name
        self.robot_id = robot_id
        self.capability_digest = CAPABILITY_DIGEST
        self._lease_timeout = lease_timeout_ms / 1000.0
        self._pose_timeout = pose_timeout_ms / 1000.0
        self._watchdog_interval = watchdog_interval_ms / 1000.0
        self._clock = clock
        self._lock = threading.RLock()
        self._transition_lock = threading.Lock()
        self._stop_event = threading.Event()
        self._watchdog_thread: threading.Thread | None = None
        self._dispatch_ack_timeout = dispatch_ack_timeout_ms / 1000.0
        self._dispatcher = dispatcher or FinalDispatchArbiter(
            RecordingAdapter(clock=clock),
            clock=clock,
            io_timeout_ms=dispatch_io_timeout_ms,
        )
        self._closed = False

        self._epoch = 0
        self._generation = 0
        self._state = "idle"
        self._reason: str | None = None
        self._session_id: str | None = None
        self._fence: str | None = None
        self._prepared_at: float | None = None
        self._last_lease_at: float | None = None
        self._lease_armed = False
        self._lease_grace_until: float | None = None
        self._capture_id: str | None = None
        self._authority_valid = False
        self._authority_invalid_reason: str | None = "not_prepared"
        self._last_pose_at: float | None = None
        self._latest_frame: dict | None = None
        self._last_sequence: int | None = None
        self._hold_clutch_sequence = -1
        self._rtc_connected = False
        self._channels = {"teleop-control": False, "teleop-pose": False}
        self._counters: Counter[str] = Counter()
        if auto_watchdog:
            self.start()

    def start(self) -> None:
        """Start one long-lived watchdog thread; calling twice is harmless."""

        with self._lock:
            if self._closed:
                return
            if self._watchdog_thread and self._watchdog_thread.is_alive():
                return
            self._stop_event.clear()
            self._watchdog_thread = threading.Thread(
                target=self._watchdog_loop,
                daemon=True,
                name="teleop-shadow-watchdog",
            )
            self._watchdog_thread.start()

    def close(self) -> None:
        self._stop_event.set()
        thread = self._watchdog_thread
        if thread and thread is not threading.current_thread():
            thread.join(timeout=max(1.0, self._watchdog_interval * 4))
        with self._transition_lock:
            with self._lock:
                if self._closed:
                    return
                self._closed = True
                if self._authority_valid:
                    self._generation += 1
                self._authority_valid = False
                self._authority_invalid_reason = "service_close"
                self._state = "released"
                self._reason = "service_close"
                self._clear_session_locked()
            ack = self._dispatcher.close(self._dispatch_ack_timeout)
            if not ack.ok:
                with self._lock:
                    self._latch_dispatch_fault_locked(ack.code)

    def prepare_shadow(self, args: dict, *, lease_armed: bool = True) -> dict:
        """Install a fenced session.

        ``prepare_shadow`` remains the low-level adapter boundary used by the
        robot-specific Driver.  The public MCP card no longer accepts these
        authority fields from Core; :meth:`prepare_local_session` generates
        them inside the Driver instead.
        """

        session_id = self._canonical_uuid(args.get("session_id"), "session_id")
        epoch = args.get("epoch")
        if isinstance(epoch, bool) or not isinstance(epoch, int) or epoch < 1:
            raise ProtocolError("invalid_epoch", "epoch must be an integer >= 1")
        fence = args.get("fence")
        if not isinstance(fence, str) or not re.fullmatch(r"[A-Za-z0-9_-]{24,128}", fence):
            raise ProtocolError("invalid_fence", "fence must be a URL-safe token of 24-128 characters")
        binding = {
            "boot_id": self.boot_id,
            "session_id": session_id,
            "epoch": epoch,
            "fence": fence,
        }
        with self._transition_lock:
            with self._lock:
                self._require_open_locked()
                if epoch <= self._epoch:
                    raise ProtocolError("stale_epoch", f"epoch must be greater than {self._epoch}")
                # Fence RTC and Frame callbacks before the safe-stop leaves the
                # runtime lock.  The new authority is installed only after the
                # final adapter has acknowledged that stop.
                self._generation += 1
                generation = self._generation
                self._authority_valid = False
                self._authority_invalid_reason = "prepare_shadow"
                self._state = "hold"
                self._reason = "prepare_shadow"
                self._clear_session_locked()
                handle = self._dispatcher.begin_prepare()
            ack = self._dispatcher.wait_safe(handle, self._dispatch_ack_timeout)
            if not ack.ok:
                with self._lock:
                    self._latch_dispatch_fault_locked(ack.code)
                raise ProtocolError(
                    "dispatch_stop_unconfirmed",
                    "final output adapter did not acknowledge prepare safe-stop",
                )
            with self._lock:
                self._require_open_locked()
                dispatch_ack = self._dispatcher.complete_prepare(handle, binding, generation)
                if not dispatch_ack.ok:
                    self._latch_dispatch_fault_locked(dispatch_ack.code)
                    raise ProtocolError(
                        "dispatch_prepare_failed",
                        "final output adapter could not arm the Shadow recording session",
                    )
                now = self._clock()
                self._epoch = epoch
                self._state = "prepared_shadow"
                self._reason = None
                self._session_id = session_id
                self._fence = fence
                self._prepared_at = now
                self._last_lease_at = now if lease_armed else None
                self._lease_armed = lease_armed
                self._lease_grace_until = None
                self._capture_id = None
                self._authority_valid = True
                self._authority_invalid_reason = None
                self._last_pose_at = None
                self._latest_frame = None
                self._last_sequence = None
                self._hold_clutch_sequence = -1
                self._rtc_connected = False
                self._channels = {"teleop-control": False, "teleop-pose": False}
                self._counters["sessions_prepared"] += 1
                return self._public_snapshot_locked(now)

    def prepare_local_session(self) -> dict:
        """Create a Driver-owned exclusive session with private authority."""

        with self._lock:
            self._require_open_locked()
            if self._authority_valid:
                if self._state == "paused":
                    raise ProtocolError(
                        "session_paused",
                        "release the paused session before starting a new one",
                    )
                return self._public_snapshot_locked(self._clock())
            epoch = self._epoch + 1
        return self.prepare_shadow(
            {
                "session_id": str(uuid.uuid4()),
                "epoch": epoch,
                "fence": secrets.token_urlsafe(32),
            },
            # Pairing can legitimately take longer than the motion lease.  The
            # lease begins only after one authenticated Capture is bound.
            lease_armed=False,
        )

    def bind_capture(self, capture_id: str) -> tuple[dict, int]:
        """Bind the sole authenticated Capture and start/renew its lease."""

        capture_id = self._canonical_uuid(capture_id, "capture_id")
        now = self._clock()
        with self._lock:
            self._require_open_locked()
            self._require_session_live_locked()
            if self._state == "paused":
                raise ProtocolError(
                    "session_paused",
                    "release the paused session before starting a new one",
                )
            if self._capture_id is not None and self._capture_id != capture_id:
                raise ProtocolError(
                    "capture_conflict",
                    "another paired Capture already owns the local session",
                )
            if self._lease_armed and self._lease_deadline_passed_locked(now):
                self._invalidate_authority_locked("lease_timeout", target_state="hold")
                raise ProtocolError(
                    "session_expired",
                    "the Capture control lease expired; start a new local session",
                )
            self._capture_id = capture_id
            self._lease_armed = True
            self._last_lease_at = now
            self._counters["capture_bindings"] += 1
            self._counters["lease_heartbeats"] += 1
            return self._ticket_binding_locked(), self._generation

    def renew_capture_lease(self, capture_id: str, generation: int) -> dict:
        """Renew authority only from the paired Capture control connection."""

        capture_id = self._canonical_uuid(capture_id, "capture_id")
        now = self._clock()
        with self._lock:
            self._require_open_locked()
            self._require_session_live_locked()
            if generation != self._generation:
                raise ProtocolError("stale_capture_generation", "Capture belongs to a stale session")
            if self._capture_id != capture_id:
                raise ProtocolError("capture_mismatch", "Capture does not own the local session")
            self._require_authority_valid_locked(now)
            self._last_lease_at = now
            self._lease_grace_until = None
            self._counters["lease_heartbeats"] += 1
            return self._public_snapshot_locked(now)

    def begin_capture_negotiation(
        self,
        capture_id: str,
        generation: int,
        *,
        grace_ms: int = 15_000,
    ) -> dict:
        """Grant ICE/DTLS time anchored to the last real Capture presence.

        Signaling offers are not lease heartbeats. Repeated offers therefore
        cannot move this deadline forward.
        """

        if grace_ms < 1000 or grace_ms > 30_000:
            raise ProtocolError("invalid_negotiation_grace", "negotiation grace must be in [1000, 30000] ms")
        capture_id = self._canonical_uuid(capture_id, "capture_id")
        now = self._clock()
        with self._lock:
            self._require_open_locked()
            self._require_session_live_locked()
            if generation != self._generation or self._capture_id != capture_id:
                raise ProtocolError("capture_mismatch", "Capture does not own the local session")
            self._require_authority_valid_locked(now)
            if self._last_lease_at is None:
                raise ProtocolError(
                    "capture_lease_missing",
                    "Capture negotiation requires a real presence lease",
                )
            grace_until = self._last_lease_at + grace_ms / 1000.0
            if now > grace_until:
                self._invalidate_authority_locked(
                    "lease_timeout",
                    target_state="hold",
                )
                raise ProtocolError(
                    "session_expired",
                    "Capture control lease expired; start a new local session",
                )
            self._lease_grace_until = grace_until
            self._counters["capture_negotiations"] += 1
            return self._public_snapshot_locked(now)

    def mark_capture_disconnected(self, capture_id: str, generation: int) -> dict:
        """Enter HOLD without allowing a stale socket to mutate a new session."""

        capture_id = self._canonical_uuid(capture_id, "capture_id")
        now = self._clock()
        with self._lock:
            if generation != self._generation or self._capture_id != capture_id:
                self._counters["stale_capture_callbacks"] += 1
                return self._public_snapshot_locked(now)
            if not self._authority_valid:
                return self._public_snapshot_locked(now)
            handle = None
            if self._state in ("prepared_shadow", "active_shadow"):
                handle = self._enter_hold_locked("capture_disconnected")
            self._counters["capture_disconnects"] += 1
        self._wait_for_transition_stop(handle, "capture disconnect")
        with self._lock:
            return self._public_snapshot_locked(self._clock())

    def capture_hold(self, capture_id: str, generation: int, reason: str) -> dict:
        """Immediately inhibit motion when the authenticated Capture loses focus."""

        capture_id = self._canonical_uuid(capture_id, "capture_id")
        now = self._clock()
        with self._lock:
            if generation != self._generation or self._capture_id != capture_id:
                self._counters["stale_capture_callbacks"] += 1
                return self._public_snapshot_locked(now)
            if not self._authority_valid:
                return self._public_snapshot_locked(now)
            handle = None
            if self._state in ("prepared_shadow", "active_shadow"):
                handle = self._enter_hold_locked(reason)
                self._counters["capture_focus_losses"] += 1
        self._wait_for_transition_stop(handle, reason)
        with self._lock:
            return self._public_snapshot_locked(self._clock())

    def pause_local(self) -> dict:
        with self._transition_lock:
            now = self._clock()
            with self._lock:
                self._require_open_locked()
                self._require_session_live_locked()
                self._require_authority_valid_locked(now)
                handle = self._enter_hold_locked("operator_pause", target_state="paused")
            self._wait_for_transition_stop(handle, "pause")
            with self._lock:
                return self._public_snapshot_locked(self._clock())

    def release_local(self, *, reason: str = "operator_release") -> dict:
        with self._transition_lock:
            with self._lock:
                self._require_open_locked()
                if not self._authority_valid:
                    return self._public_snapshot_locked(self._clock())
                handle = self._invalidate_authority_locked(reason, target_state="released")
                self._counters["sessions_released"] += 1
                if reason == "emergency_stop":
                    self._counters["emergency_stops"] += 1
            self._wait_for_transition_stop(handle, reason)
            with self._lock:
                return self._public_snapshot_locked(self._clock())

    def heartbeat(self, args: dict) -> dict:
        now = self._clock()
        with self._lock:
            self._require_open_locked()
            self._require_session_live_locked()
            self._require_identity_locked(args)
            self._require_authority_valid_locked(now)
            self._last_lease_at = now
            self._counters["lease_heartbeats"] += 1
            return self._public_snapshot_locked(now)

    def pause(self, args: dict) -> dict:
        with self._transition_lock:
            now = self._clock()
            with self._lock:
                self._require_open_locked()
                self._require_session_live_locked()
                self._require_identity_locked(args)
                self._require_authority_valid_locked(now)
                handle = self._enter_hold_locked("operator_pause", target_state="paused")
            self._wait_for_transition_stop(handle, "pause")
            with self._lock:
                return self._public_snapshot_locked(self._clock())

    def soft_stop(self, args: dict) -> dict:
        with self._transition_lock:
            now = self._clock()
            with self._lock:
                self._require_open_locked()
                self._require_session_live_locked()
                self._require_identity_locked(args)
                self._require_authority_valid_locked(now)
                if self._state == "paused":
                    raise ProtocolError("session_paused", "a paused session requires a new prepare_shadow")
                handle = self._enter_hold_locked("soft_stop")
                self._counters["soft_stops"] += 1
            self._wait_for_transition_stop(handle, "soft_stop")
            with self._lock:
                return self._public_snapshot_locked(self._clock())

    def release(self, args: dict | None = None, *, lifecycle: bool = False) -> dict:
        with self._transition_lock:
            with self._lock:
                self._require_open_locked()
                if not lifecycle:
                    self._require_session_live_locked()
                    self._require_identity_locked(args or {})
                reason = "lifecycle_stop" if lifecycle else "operator_release"
                handle = self._invalidate_authority_locked(reason, target_state="released")
                self._counters["sessions_released"] += 1
            self._wait_for_transition_stop(handle, "release")
            with self._lock:
                return self._public_snapshot_locked(self._clock())

    def submit_shadow_frame(
        self,
        raw_frame: Any,
        *,
        source: str,
        rtc_generation: int | None = None,
    ) -> dict:
        try:
            frame = validate_frame_v1(raw_frame)
        except ProtocolError as exc:
            with self._lock:
                self._counters["frames_rejected"] += 1
                self._counters[f"reject_{exc.code}"] += 1
            raise

        now = self._clock()
        with self._lock:
            try:
                self._require_open_locked()
                self._require_session_live_locked()
                self._require_identity_locked(frame)
                self._require_authority_valid_locked(now)
                if source == "rtc":
                    if rtc_generation != self._generation:
                        raise ProtocolError(
                            "stale_rtc_generation",
                            "RTC Pose belongs to a stale session generation",
                        )
                    if not self._rtc_connected or not all(self._channels.values()):
                        if self._state in ("prepared_shadow", "active_shadow"):
                            self._enter_hold_locked("rtc_not_ready")
                        raise ProtocolError(
                            "rtc_not_ready",
                            "both teleop-control and teleop-pose must be open for RTC Pose",
                        )
                if self._state == "paused":
                    raise ProtocolError("session_inactive", f"cannot accept frames in state {self._state}")
                sequence = frame["sequence"]
                if self._last_sequence is not None and sequence <= self._last_sequence:
                    raise ProtocolError(
                        "sequence_not_increasing",
                        f"sequence must be greater than {self._last_sequence}",
                    )
            except ProtocolError as exc:
                self._counters["frames_rejected"] += 1
                self._counters[f"reject_{exc.code}"] += 1
                raise

            self._latest_frame = frame
            self._last_sequence = sequence
            self._last_pose_at = now
            self._counters["frames_accepted"] += 1
            self._counters[f"frames_from_{source}"] += 1

            tracking_ok = all(frame["tracking"].values())
            dispatch_frame = False
            allow_reclutch = False
            if self._state == "hold" and self._reason == "soft_stop":
                # Core has no resume/rearm action.  An operator soft-stop is a
                # latch: only release followed by a newer prepare can re-arm.
                self._counters["frames_held_after_soft_stop"] += 1
            elif not frame["deadman"]:
                self._enter_hold_locked("deadman_released")
            elif not tracking_ok:
                self._enter_hold_locked("tracking_lost")
            elif self._state == "prepared_shadow":
                self._state = "active_shadow"
                self._reason = None
                dispatch_frame = True
            elif self._state == "hold":
                if frame["clutch_sequence"] > self._hold_clutch_sequence:
                    self._state = "active_shadow"
                    self._reason = None
                    self._counters["explicit_reclutches"] += 1
                    dispatch_frame = True
                    allow_reclutch = True
                else:
                    self._counters["frames_held_without_reclutch"] += 1
            elif self._state == "active_shadow":
                dispatch_frame = True

            if dispatch_frame:
                lease_started = (
                    self._last_lease_at if self._last_lease_at is not None else now
                )
                dispatch_ack = self._dispatcher.publish_latest(
                    frame,
                    session_generation=self._generation,
                    expires_monotonic=min(
                        now + self._pose_timeout,
                        lease_started + self._lease_timeout,
                    ),
                    allow_reclutch=allow_reclutch,
                )
                if not dispatch_ack.ok:
                    self._counters["dispatch_rejections"] += 1
                    self._counters[f"dispatch_reject_{dispatch_ack.code}"] += 1
                    dispatch_state = self._dispatcher.snapshot()["state"]
                    if dispatch_ack.code == "intent_expired" or (
                        dispatch_ack.code == "motion_inhibited"
                        and dispatch_state == "safe_reclutch_required"
                    ):
                        self._enter_hold_locked("pose_timeout")
                    else:
                        self._dispatcher.trip(
                            "dispatch_fault",
                            target_state="fault_latched",
                            retain_authority=False,
                        )
                        self._latch_dispatch_fault_locked(dispatch_ack.code)
                        raise ProtocolError(
                            "dispatch_fault",
                            "final output authority check failed; a new Driver process is required",
                        )

            result = self._public_snapshot_locked(now)
            result["accepted_sequence"] = sequence
            return result

    def mark_channel(self, generation: int, label: str, opened: bool) -> dict:
        now = self._clock()
        with self._lock:
            if generation != self._generation:
                self._counters["stale_rtc_callbacks"] += 1
                return self._public_snapshot_locked(now)
            if self._state in ("idle", "released"):
                self._counters["terminal_rtc_callbacks_ignored"] += 1
                return self._public_snapshot_locked(now)
            if not self._authority_valid or self._lease_deadline_passed_locked(now):
                if self._authority_valid:
                    self._invalidate_authority_locked("lease_timeout", target_state="hold")
                self._counters["expired_rtc_callbacks_ignored"] += 1
                return self._public_snapshot_locked(now)
            if label in self._channels:
                self._channels[label] = bool(opened)
            self._rtc_connected = all(self._channels.values())
            if not opened and self._state in ("prepared_shadow", "active_shadow"):
                self._enter_hold_locked("rtc_disconnected")
            return self._public_snapshot_locked(now)

    def submit_rtc_frame(
        self,
        raw_frame: Any,
        *,
        authority: Mapping[str, Any],
        rtc_generation: int,
    ) -> dict:
        """Bind one public RTC frame before entering the common safety path."""

        try:
            frame = bind_rtc_frame_v1(raw_frame, authority=authority)
        except ProtocolError as exc:
            with self._lock:
                self._counters["frames_rejected"] += 1
                self._counters[f"reject_{exc.code}"] += 1
            raise
        return self.submit_shadow_frame(
            frame,
            source="rtc",
            rtc_generation=rtc_generation,
        )

    def mark_rtc_disconnected(self, generation: int, reason: str = "rtc_disconnected") -> dict:
        now = self._clock()
        with self._lock:
            if generation != self._generation:
                self._counters["stale_rtc_callbacks"] += 1
                return self._public_snapshot_locked(now)
            if self._state in ("idle", "released"):
                self._counters["terminal_rtc_callbacks_ignored"] += 1
                return self._public_snapshot_locked(now)
            if not self._authority_valid or self._lease_deadline_passed_locked(now):
                if self._authority_valid:
                    self._invalidate_authority_locked("lease_timeout", target_state="hold")
                self._counters["expired_rtc_callbacks_ignored"] += 1
                return self._public_snapshot_locked(now)
            self._rtc_connected = False
            self._channels = {"teleop-control": False, "teleop-pose": False}
            if self._state in ("prepared_shadow", "active_shadow"):
                self._enter_hold_locked(reason)
            self._counters["rtc_disconnects"] += 1
            return self._public_snapshot_locked(now)

    def record_protocol_error(self, code: str) -> None:
        with self._lock:
            self._counters["protocol_errors"] += 1
            self._counters[f"protocol_{code}"] += 1

    def status(self) -> dict:
        with self._lock:
            now = self._clock()
            if self._authority_valid and self._lease_deadline_passed_locked(now):
                self._invalidate_authority_locked("lease_timeout", target_state="hold")
            return self._public_snapshot_locked(now)

    def ticket_binding(self) -> dict:
        with self._lock:
            self._require_open_locked()
            self._require_session_live_locked()
            self._require_authority_valid_locked(self._clock())
            return self._ticket_binding_locked()

    def rtc_authority_snapshot(self) -> tuple[dict, int]:
        """Capture one immutable RTC authority tuple under the runtime lock."""

        with self._lock:
            self._require_open_locked()
            self._require_session_live_locked()
            self._require_authority_valid_locked(self._clock())
            return self._ticket_binding_locked(), self._generation

    def session_generation(self) -> int:
        """Return the opaque internal generation used only to fence RTC callbacks."""

        with self._lock:
            return self._generation

    def generation_matches(self, generation: int) -> bool:
        with self._lock:
            self._sync_dispatch_fault_locked()
            if generation != self._generation:
                return False
            if not self._authority_valid:
                return False
            if self._lease_deadline_passed_locked(self._clock()):
                self._invalidate_authority_locked("lease_timeout", target_state="hold")
                return False
            return True

    def watchdog_tick(self) -> dict:
        now = self._clock()
        with self._lock:
            if self._authority_valid and self._state in (
                "prepared_shadow", "active_shadow", "hold", "paused"
            ):
                if self._lease_deadline_passed_locked(now):
                    self._invalidate_authority_locked("lease_timeout", target_state="hold")
                elif (
                    self._state == "active_shadow"
                    and self._last_pose_at is not None
                    and now - self._last_pose_at > self._pose_timeout
                ):
                    self._enter_hold_locked("pose_timeout")
                    self._counters["pose_timeouts"] += 1
            return self._public_snapshot_locked(now)

    def _watchdog_loop(self) -> None:
        while not self._stop_event.wait(self._watchdog_interval):
            self.watchdog_tick()

    def _enter_hold_locked(
        self,
        reason: str,
        *,
        target_state: str = "hold",
    ) -> StopHandle | None:
        changed = self._state != target_state or self._reason != reason
        self._state = target_state
        self._reason = reason
        if self._latest_frame is not None:
            self._hold_clutch_sequence = max(
                self._hold_clutch_sequence,
                int(self._latest_frame["clutch_sequence"]),
            )
        if not changed:
            return None
        dispatch_state = (
            "safe_latched"
            if target_state == "paused" or reason == "soft_stop"
            else "safe_reclutch_required"
        )
        return self._dispatcher.trip(
            reason,
            target_state=dispatch_state,
            retain_authority=True,
        )

    def _lease_deadline_passed_locked(self, now: float) -> bool:
        if self._lease_grace_until is not None and now <= self._lease_grace_until:
            return False
        return (
            self._lease_armed
            and (self._last_lease_at is None or now - self._last_lease_at > self._lease_timeout)
        )

    def _invalidate_authority_locked(
        self,
        reason: str,
        *,
        target_state: str,
    ) -> StopHandle:
        was_valid = self._authority_valid
        self._authority_valid = False
        self._authority_invalid_reason = reason
        if was_valid:
            self._generation += 1
        self._state = target_state
        self._reason = reason
        self._clear_session_locked()
        handle = self._dispatcher.trip(
            reason,
            target_state="safe_revoked",
            retain_authority=False,
        )
        if was_valid and reason == "lease_timeout":
            self._counters["lease_timeouts"] += 1
        return handle

    def _clear_session_locked(self) -> None:
        self._session_id = None
        self._fence = None
        self._prepared_at = None
        self._last_lease_at = None
        self._lease_armed = False
        self._lease_grace_until = None
        self._capture_id = None
        self._last_pose_at = None
        self._latest_frame = None
        self._last_sequence = None
        self._hold_clutch_sequence = -1
        self._rtc_connected = False
        self._channels = {"teleop-control": False, "teleop-pose": False}

    def _wait_for_transition_stop(
        self,
        handle: StopHandle | None,
        operation: str,
    ) -> None:
        if handle is None:
            return
        ack = self._dispatcher.wait_safe(handle, self._dispatch_ack_timeout)
        if ack.ok:
            return
        with self._lock:
            self._latch_dispatch_fault_locked(ack.code)
        raise ProtocolError(
            "dispatch_stop_unconfirmed",
            f"final output adapter did not acknowledge {operation} safe-stop",
        )

    def _latch_dispatch_fault_locked(self, code: str) -> None:
        if self._authority_valid:
            self._generation += 1
        self._authority_valid = False
        self._authority_invalid_reason = "dispatch_fault"
        self._state = "fault"
        self._reason = "dispatch_fault"
        self._clear_session_locked()
        self._counters["dispatch_faults"] += 1
        self._counters[f"dispatch_fault_{code}"] += 1

    def _sync_dispatch_fault_locked(self) -> None:
        dispatch = self._dispatcher.snapshot()
        code = dispatch.get("fault_code")
        if code is not None and self._authority_invalid_reason != "dispatch_fault":
            self._latch_dispatch_fault_locked(str(code))

    def _require_open_locked(self) -> None:
        if self._closed:
            raise ProtocolError("service_closed", "the Driver process is closing")
        self._sync_dispatch_fault_locked()

    def _require_authority_valid_locked(self, now: float) -> None:
        if not self._authority_valid:
            self._raise_invalid_authority_locked()
        if self._lease_deadline_passed_locked(now):
            self._invalidate_authority_locked("lease_timeout", target_state="hold")
            raise ProtocolError(
                "session_expired",
                "Capture control lease expired; a new prepare_shadow is required",
            )

    def _require_session_live_locked(self) -> None:
        if not self._authority_valid:
            self._raise_invalid_authority_locked()

    def _raise_invalid_authority_locked(self) -> None:
        if self._authority_invalid_reason == "lease_timeout":
            raise ProtocolError(
                "session_expired",
                "Capture control lease expired; a new prepare_shadow is required",
            )
        raise ProtocolError("session_inactive", "a new prepare_shadow session is required")

    def _require_identity_locked(self, value: dict) -> None:
        required = ("boot_id", "session_id", "epoch", "fence")
        missing = [key for key in required if key not in value]
        if missing:
            raise ProtocolError("missing_identity", f"missing session identity fields: {missing}")
        if value["boot_id"] != self.boot_id:
            raise ProtocolError("boot_mismatch", "boot_id does not match this Driver process")
        if value["session_id"] != self._session_id:
            raise ProtocolError("session_mismatch", "session_id does not match the active session")
        if isinstance(value["epoch"], bool) or value["epoch"] != self._epoch:
            raise ProtocolError("epoch_mismatch", "epoch does not match the active session")
        if not isinstance(value["fence"], str) or not secrets.compare_digest(value["fence"], self._fence or ""):
            raise ProtocolError("fence_mismatch", "fence does not match the active session")

    def _ticket_binding_locked(self) -> dict:
        assert self._session_id is not None and self._fence is not None
        return {
            "boot_id": self.boot_id,
            "session_id": self._session_id,
            "epoch": self._epoch,
            "fence": self._fence,
            "capability_digest": self.capability_digest,
        }

    @staticmethod
    def _canonical_uuid(value: Any, name: str) -> str:
        if not isinstance(value, str):
            raise ProtocolError("invalid_session_id", f"{name} must be a canonical UUID")
        try:
            parsed = uuid.UUID(value)
        except ValueError as exc:
            raise ProtocolError("invalid_session_id", f"{name} must be a canonical UUID") from exc
        if str(parsed) != value.lower():
            raise ProtocolError("invalid_session_id", f"{name} must be a canonical UUID")
        return str(parsed)

    def _public_snapshot_locked(self, now: float) -> dict:
        self._sync_dispatch_fault_locked()
        lease_age = None if self._last_lease_at is None else max(0.0, now - self._last_lease_at)
        pose_age = None if self._last_pose_at is None else max(0.0, now - self._last_pose_at)
        latest = copy.deepcopy(self._latest_frame)
        if latest is not None:
            latest.pop("fence", None)
        return {
            "driver": self.driver_id,
            "driver_id": self.driver_id,
            "driver_name": self.driver_name,
            "driver_type": "teleop-shadow",
            "robot_id": self.robot_id,
            "mode": MODE,
            "actuation_enabled": False,
            "boot_id": self.boot_id,
            "session_id": self._session_id,
            "epoch": self._epoch,
            "state": self._state,
            "reason": self._reason,
            "authority_valid": self._authority_valid,
            "capability_digest": self.capability_digest,
            "capabilities": copy.deepcopy(CAPABILITIES),
            "lease": {
                "source": "paired-capture-control-only",
                "timeout_ms": round(self._lease_timeout * 1000),
                "age_ms": None if lease_age is None else round(lease_age * 1000, 3),
                "armed": self._lease_armed,
                "fresh": (
                    not self._lease_armed
                    or (lease_age is not None and lease_age <= self._lease_timeout)
                ),
                "authority_valid": self._authority_valid,
                "expired_latched": self._authority_invalid_reason == "lease_timeout",
                "negotiation_grace": (
                    self._lease_grace_until is not None and now <= self._lease_grace_until
                ),
            },
            "capture": {
                "paired": self._capture_id is not None,
                "capture_id": self._capture_id,
                "renews_lease": True,
            },
            "pose": {
                "timeout_ms": round(self._pose_timeout * 1000),
                "age_ms": None if pose_age is None else round(pose_age * 1000, 3),
                "fresh": pose_age is not None and pose_age <= self._pose_timeout,
                "latest_sequence": self._last_sequence,
                "latest": latest,
            },
            "rtc": {
                "connected": self._rtc_connected,
                "channels": dict(self._channels),
                "renews_lease": False,
            },
            "dispatch": self._dispatcher.snapshot(),
            "counters": dict(sorted(self._counters.items())),
        }
