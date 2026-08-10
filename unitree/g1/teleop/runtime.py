"""Thread-safe G1 teleoperation session runtime."""

from __future__ import annotations

import copy
import re
import secrets
import threading
import time
import uuid
from collections import Counter, deque
from collections.abc import Callable, Mapping
from typing import Any

from .descriptor import CAPABILITIES, PROFILE_ID, capability_digest
from .dispatch import FinalDispatchArbiter, OutputAdapter, StopHandle
from .protocol import (
    ProtocolError,
    bind_rtc_frame_v1,
    validate_frame_v1,
)

SESSION_STATES = {
    "idle",
    "prepared_shadow",
    "active_shadow",
    "prepared_live",
    "active_live",
    "hold",
    "paused",
    "released",
    "fault",
}
class G1TeleopRuntime:
    """Own fencing, watchdogs and the sole G1 final-dispatch boundary."""

    def __init__(
        self,
        *,
        lease_timeout_ms: int = 1000,
        pose_timeout_ms: int = 200,
        watchdog_interval_ms: int = 25,
        driver_id: str = "unitree-g1",
        driver_name: str = "Unitree G1 Bundle",
        robot_id: str | None = None,
        clock: Callable[[], float] = time.monotonic,
        auto_watchdog: bool = True,
        dispatch_io_timeout_ms: int = 100,
        dispatch_ack_timeout_ms: int = 200,
        mode: str,
        adapter: OutputAdapter,
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
        if mode not in ("shadow", "live"):
            raise ValueError("mode must be 'shadow' or 'live'")
        expected_hardware = mode == "live"
        if getattr(adapter, "hardware_output", None) is not expected_hardware:
            raise ValueError("adapter hardware_output must match the configured session mode")
        self.boot_id = str(uuid.uuid4())
        self.driver_id = driver_id
        self.driver_name = driver_name
        self.robot_id = robot_id or driver_id
        self.mode = mode
        self.profile_id = PROFILE_ID
        self.capabilities = copy.deepcopy(CAPABILITIES)
        self.capability_digest = capability_digest(mode)
        self.actuation_enabled = expected_hardware
        self._prepared_state = f"prepared_{mode}"
        self._active_state = f"active_{mode}"
        self._prepare_action = f"prepare_{mode}"
        self._lease_timeout = lease_timeout_ms / 1000.0
        self._pose_timeout = pose_timeout_ms / 1000.0
        self._watchdog_interval = watchdog_interval_ms / 1000.0
        self._clock = clock
        self._lock = threading.RLock()
        self._transition_lock = threading.Lock()
        self._stop_event = threading.Event()
        self._watchdog_thread: threading.Thread | None = None
        self._dispatch_ack_timeout = dispatch_ack_timeout_ms / 1000.0
        self._dispatcher = FinalDispatchArbiter(
            adapter,
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
        self._authority_valid = False
        self._authority_invalid_reason: str | None = "not_prepared"
        self._last_pose_at: float | None = None
        self._latest_frame: dict | None = None
        self._last_sequence: int | None = None
        self._hold_clutch_sequence = -1
        self._neutral_required = True
        self._rtc_connected = False
        self._channels = {"teleop-control": False, "teleop-pose": False}
        self._counters: Counter[str] = Counter()
        self._frame_times: deque[float] = deque(maxlen=256)
        self._sequence_gaps = 0
        self._rtc_rtt_ms: float | None = None
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
                name="g1-teleop-watchdog",
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

    def prepare(self, requested_mode: str, args: dict) -> dict:
        if requested_mode != self.mode:
            raise ProtocolError(
                "mode_not_configured",
                f"Driver is configured for {self.mode!r}; {requested_mode!r} prepare is unavailable",
            )
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
                self._authority_invalid_reason = self._prepare_action
                self._state = "hold"
                self._reason = self._prepare_action
                self._clear_session_locked()
                handle = self._dispatcher.begin_prepare(self._prepare_action)
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
                        "final output adapter could not arm the configured teleoperation session",
                    )
                now = self._clock()
                self._epoch = epoch
                self._state = self._prepared_state
                self._reason = None
                self._session_id = session_id
                self._fence = fence
                self._prepared_at = now
                self._last_lease_at = now
                self._authority_valid = True
                self._authority_invalid_reason = None
                self._last_pose_at = None
                self._latest_frame = None
                self._last_sequence = None
                self._hold_clutch_sequence = -1
                self._neutral_required = True
                self._rtc_connected = False
                self._channels = {"teleop-control": False, "teleop-pose": False}
                self._counters["sessions_prepared"] += 1
                return self._public_snapshot_locked(now)

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
                    raise ProtocolError(
                        "session_paused",
                        f"a paused session requires a new {self._prepare_action}",
                    )
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

    def submit_frame(
        self,
        raw_frame: Any,
        *,
        source: str,
        rtc_generation: int | None = None,
    ) -> dict:
        received_at = self._clock()
        with self._lock:
            self._counters["frames_received"] += 1
        try:
            frame = validate_frame_v1(raw_frame, expected_mode=self.mode)
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
                        if self._state in (self._prepared_state, self._active_state):
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

            if self._last_sequence is not None and sequence > self._last_sequence + 1:
                self._sequence_gaps += sequence - self._last_sequence - 1
            self._latest_frame = frame
            self._last_sequence = sequence
            self._last_pose_at = now
            self._counters["frames_accepted"] += 1
            self._counters[f"frames_from_{source}"] += 1
            self._frame_times.append(now)

            tracking_ok = all(frame["tracking"].values())
            dispatch_frame = False
            allow_reclutch = False
            if self._state == "hold" and self._reason == "soft_stop":
                # Core has no resume/rearm action.  An operator soft-stop is a
                # latch: only release followed by a newer prepare can re-arm.
                self._counters["frames_held_after_soft_stop"] += 1
            elif not tracking_ok:
                self._enter_hold_locked("tracking_lost")
            elif self._neutral_required:
                if frame["deadman"]:
                    self._counters["frames_held_neutral_required"] += 1
                    if self._state == self._active_state:
                        self._enter_hold_locked("neutral_required")
                else:
                    self._neutral_required = False
                    self._hold_clutch_sequence = max(
                        self._hold_clutch_sequence,
                        int(frame["clutch_sequence"]),
                    )
                    self._counters["neutral_observations"] += 1
            elif not frame["deadman"]:
                self._enter_hold_locked("deadman_released")
            elif self._state == self._prepared_state:
                if frame["clutch_sequence"] > self._hold_clutch_sequence:
                    self._state = self._active_state
                    self._reason = None
                    dispatch_frame = True
                    allow_reclutch = True
                else:
                    self._counters["frames_held_without_reclutch"] += 1
            elif self._state == "hold":
                if frame["clutch_sequence"] > self._hold_clutch_sequence:
                    self._state = self._active_state
                    self._reason = None
                    self._counters["explicit_reclutches"] += 1
                    dispatch_frame = True
                    allow_reclutch = True
                else:
                    self._counters["frames_held_without_reclutch"] += 1
            elif self._state == self._active_state:
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
                    received_monotonic=received_at,
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
            was_connected = self._rtc_connected
            if label in self._channels:
                self._channels[label] = bool(opened)
            self._rtc_connected = all(self._channels.values())
            if self._rtc_connected and not was_connected:
                self._neutral_required = True
                self._counters["rtc_neutral_resets"] += 1
            if not opened and self._state in (self._prepared_state, self._active_state):
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
            frame = bind_rtc_frame_v1(
                raw_frame,
                authority=authority,
                expected_mode=self.mode,
            )
        except ProtocolError as exc:
            with self._lock:
                self._counters["frames_rejected"] += 1
                self._counters[f"reject_{exc.code}"] += 1
            raise
        return self.submit_frame(
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
            if self._state in (self._prepared_state, self._active_state):
                self._enter_hold_locked(reason)
            self._counters["rtc_disconnects"] += 1
            return self._public_snapshot_locked(now)

    def record_protocol_error(self, code: str) -> None:
        with self._lock:
            self._counters["protocol_errors"] += 1
            self._counters[f"protocol_{code}"] += 1

    def record_rtc_rtt(self, milliseconds: float | None) -> None:
        with self._lock:
            if milliseconds is None:
                self._rtc_rtt_ms = None
                return
            value = float(milliseconds)
            if value < 0.0 or value > 60_000.0:
                raise ValueError("RTC RTT must be in [0, 60000] milliseconds")
            self._rtc_rtt_ms = round(value, 3)

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
            self._sync_dispatch_fault_locked()
            if self._authority_valid and self._state in (
                self._prepared_state, self._active_state, "hold", "paused"
            ):
                if self._lease_deadline_passed_locked(now):
                    self._invalidate_authority_locked("lease_timeout", target_state="hold")
                elif (
                    self._state == self._active_state
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
        return self._last_lease_at is None or now - self._last_lease_at > self._lease_timeout

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
        self._last_pose_at = None
        self._latest_frame = None
        self._last_sequence = None
        self._hold_clutch_sequence = -1
        self._neutral_required = True
        self._frame_times.clear()
        self._sequence_gaps = 0
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
            return
        release_reason = dispatch.get("external_release_reason")
        dispatch_state = dispatch.get("state")
        if (
            (
                release_reason in {"command_timeout", "intent_expired"}
                or dispatch_state == "safe_reclutch_required"
            )
            and self._authority_valid
            and self._state == self._active_state
        ):
            self._state = "hold"
            self._reason = (
                str(release_reason)
                if release_reason in {"command_timeout", "intent_expired"}
                else "pose_timeout"
            )
            self._neutral_required = True
            if self._latest_frame is not None:
                self._hold_clutch_sequence = max(
                    self._hold_clutch_sequence,
                    int(self._latest_frame["clutch_sequence"]),
                )
            self._counters["hardware_ttl_holds"] += 1

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
                f"Core MCP heartbeat lease expired; a new {self._prepare_action} is required",
            )

    def _require_session_live_locked(self) -> None:
        if not self._authority_valid:
            self._raise_invalid_authority_locked()

    def _raise_invalid_authority_locked(self) -> None:
        if self._authority_invalid_reason == "lease_timeout":
            raise ProtocolError(
                "session_expired",
                f"Core MCP heartbeat lease expired; a new {self._prepare_action} is required",
            )
        raise ProtocolError(
            "session_inactive",
            f"a new {self._prepare_action} session is required",
        )

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
        dispatch = self._dispatcher.snapshot()
        adapter_snapshot = dispatch.get("adapter", {})
        adapter_diagnostics = (
            adapter_snapshot.get("diagnostics", {})
            if isinstance(adapter_snapshot, dict)
            else {}
        )
        adapter_latency = adapter_diagnostics.get("latency_ms", {})
        dispatch_latency = dispatch.get("latency_ms", {})
        empty_latency = {"last": None, "p50": None, "p95": None, "p99": None, "count": 0}
        latency = {
            "receive_to_admit": copy.deepcopy(
                dispatch_latency.get("receive_to_admit", empty_latency)
            ),
            "mailbox_wait": copy.deepcopy(
                dispatch_latency.get("mailbox_wait", empty_latency)
            ),
            "ik": copy.deepcopy(adapter_latency.get("ik", empty_latency)),
            "adapter_apply": copy.deepcopy(
                adapter_latency.get("adapter_apply", empty_latency)
            ),
            "robot_follow": copy.deepcopy(
                adapter_latency.get("robot_follow", empty_latency)
            ),
        }
        if len(self._frame_times) >= 2:
            span = self._frame_times[-1] - self._frame_times[0]
            frame_rate_hz = 0.0 if span <= 0.0 else (len(self._frame_times) - 1) / span
        else:
            frame_rate_hz = 0.0
        transport = {
            "rtc_rtt_ms": self._rtc_rtt_ms,
            "pose_age_ms": None if pose_age is None else round(pose_age * 1000, 3),
            "frame_rate_hz": round(min(1000.0, max(0.0, frame_rate_hz)), 3),
            "frames_received": int(self._counters.get("frames_received", 0)),
            "frames_rejected": int(self._counters.get("frames_rejected", 0)),
            "sequence_gaps": int(self._sequence_gaps),
            "mailbox_replacements": int(
                dispatch.get("counters", {}).get("mailbox_replacements", 0)
            ),
        }
        output = copy.deepcopy(adapter_snapshot.get("output", {}))
        return {
            "driver": self.driver_id,
            "driver_id": self.driver_id,
            "driver_name": self.driver_name,
            "driver_type": "teleop",
            "robot_id": self.robot_id,
            "profile_id": self.profile_id,
            "mode": self.mode,
            "actuation_enabled": self.actuation_enabled,
            "boot_id": self.boot_id,
            "session_id": self._session_id,
            "epoch": self._epoch,
            "state": self._state,
            "reason": self._reason,
            "authority_valid": self._authority_valid,
            "capability_digest": self.capability_digest,
            "capabilities": copy.deepcopy(self.capabilities),
            "lease": {
                "source": "agent-core-mcp-heartbeat-only",
                "timeout_ms": round(self._lease_timeout * 1000),
                "age_ms": None if lease_age is None else round(lease_age * 1000, 3),
                "fresh": lease_age is not None and lease_age <= self._lease_timeout,
                "authority_valid": self._authority_valid,
                "expired_latched": self._authority_invalid_reason == "lease_timeout",
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
            "dispatch": dispatch,
            "diagnostics": {
                "transport": transport,
                "latency_ms": latency,
            },
            "output": output,
            "counters": dict(sorted(self._counters.items())),
        }
