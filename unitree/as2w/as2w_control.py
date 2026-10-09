"""AS2W SDK scheduling and ownership boundary around the shared ControlSink.

ROS callbacks replace one raw mailbox entry. The sole writer runs the shared
safety chain immediately before bounded SDK dispatch, so queued or rejected
frames never refresh the watchdog or advance the applied velocity baseline.
"""
from __future__ import annotations

from copy import deepcopy
from dataclasses import dataclass
import math
import threading
import time

from common.control import ControlSink, Outcome, Verdict

AXES = ("vx", "vy", "vz", "wx", "wy", "wz")
STANDING = {"STAND_UP", "BALANCE_STAND", "RECOVERY_STAND", "STANDING",
            "AI_STAND_UP", "AI_BALANCE_STAND", "AI_RECOVERY_STAND"}
MOVING = {"WALK", "WALKING", "RUN", "RUNNING", "MOVE", "MOVING",
          "REGULAR_WALK", "REGULAR_RUN", "AI_FREE_WALK", "AI_WALK", "AI_RUN"}


def finite(value):
    return (not isinstance(value, bool) and isinstance(value, (int, float))
            and math.isfinite(value))


@dataclass(frozen=True)
class Frame:
    message: object
    generation: int


class VelocityController:
    """One SDK writer, with protocol/safety decisions owned by ControlSink."""

    def __init__(self, client, descriptor, *, dry_run=True, rotate_only=False,
                 state_provider=None, conflict=None, odom_provider=None,
                 clock=time.monotonic, wall_clock=time.time, threaded=True):
        self.client, self.descriptor = client, descriptor
        self._clock, self._wall = clock, wall_clock
        self._state_provider, self._conflict = state_provider, conflict
        self._odom_provider = odom_provider
        self._cv = threading.Condition(threading.RLock())
        self._writer = threading.Lock()
        self._transition = threading.Lock()
        self._threaded, self._thread = threaded, None
        self._closing = False
        self._generation = 0
        self._executing_generation = None
        self._latest = None
        self._last_frame = self._last_send = None
        self._state_checked, self._state_problem = float("-inf"), ""
        self._paused, self._owned = True, False
        self._stop_pending = False
        self._completed_stop_generation = None
        self._stop_event = threading.Event()
        self._stop_event.set()
        self._release_after_stop = False
        self._pending_dry_run = False
        self._reason = "not started"
        self._dry_run, self._rotate_only = bool(dry_run), bool(rotate_only)
        self._last_ret = self._last_move_ret = self._last_stop_ret = None
        self._stop_accepted = self._physical_stopped = None
        self._stop_ack_ms = self._stationary_since = self._last_stop_sample = None
        self._stationary_samples = 0
        self._last_outcome = None
        self._stats = dict(received=0, queued=0, validated=0, rejected=0,
                           replaced=0, simulated=0, attempted=0, sdk_accepted=0,
                           sdk_errors=0, stop_attempts=0, stop_accepted=0)
        self._sink = ControlSink(
            descriptor, self._apply, strict_stream=True,
            before_apply=self._before_apply,
            on_watchdog=self._on_sink_stop, on_abort=self._on_sink_abort,
            clock=lambda: self._clock() * 1000,
            wall_clock=lambda: self._wall() * 1000)

    @property
    def period_s(self):
        return 1.0 / self.descriptor["rate"]["expected_hz"]

    @property
    def _fault(self):
        return self._sink.fault_reason

    def _client_problem(self):
        if self.client is None:
            return "dedicated control client is unavailable"
        for name in ("Move", "StopMove", "GetState", "acquire_control", "release_control"):
            if not callable(getattr(self.client, name, None)):
                return "dedicated control client is missing " + name
        bound = getattr(self.client, "max_call_seconds", None)
        if not finite(bound) or not 0 < bound <= 0.5:
            return "control client must declare a bounded call of at most 0.5 seconds"
        ready = getattr(self.client, "control_ready", False)
        if callable(ready):
            ready = ready()
        if ready is not True:
            return "dedicated control channel is not ready"
        return ""

    def _read_posture(self, force=False):
        if not force and self._clock() - self._state_checked < 0.25:
            return self._state_problem
        try:
            result = (self._state_provider() if self._state_provider else
                      self.client.GetState())
            if not isinstance(result, tuple) or len(result) != 2:
                problem = "GetState returned no locomotion state"
            else:
                code, state = result
                name = str(state.get("fsm_name", "")).strip().upper() if isinstance(state, dict) else ""
                problem = ("" if code == 0 and not isinstance(code, bool) and name in STANDING | MOVING else
                           f"posture unavailable or not ready: ret={code}, fsm={name or 'UNKNOWN'}")
        except Exception as exc:
            problem = f"GetState failed: {type(exc).__name__}: {exc}"
        self._state_checked = self._clock()
        self._state_problem = problem
        return problem

    def activate(self):
        """Acquire the SDK writer and establish a fresh sink session from rest."""
        with self._transition:
            with self._cv:
                if self._sink.aborted:
                    return self._result(False, "fault is latched; call reset_fault first")
                if self._stop_pending or self._closing:
                    return self._result(False, "controller is stopping or closed")
                if not self._paused:
                    return self._result(True)
                dry_run = self._dry_run
            if self._conflict and self._conflict():
                return self._result(False, "another chassis action is active")
            if not dry_run:
                problem = self._client_problem()
                if problem:
                    return self._result(False, problem)
                try:
                    claim = self.client.acquire_control()
                except Exception as exc:
                    return self._result(False, f"control acquisition failed: {exc}")
                if not isinstance(claim, dict) or claim.get("ok") is not True:
                    return self._result(False, "chassis is occupied or acquisition failed")
                with self._cv:
                    self._owned = True
            with self._writer:
                with self._cv:
                    self._request_stop_locked("clearing prior velocity before stream start", release=False)
                self._execute_stop()
                if self._sink.aborted:
                    return self._result(False)
                if not dry_run:
                    problem = self._read_posture(force=True)
                    if problem:
                        try:
                            self.client.release_control()
                        except Exception as exc:
                            self._sink.abort(f"control release failed: {exc}")
                            self._finish_sink_operation()
                        else:
                            with self._cv:
                                self._owned = False
                        return self._result(False, problem)
                # This baseline is commanded zero, not a claim of measured rest.
                self._sink.reset(initial_values=(0.0,) * 6,
                                 minimum_stamp_ms=self._wall() * 1000)
                with self._cv:
                    self._generation += 1
                    self._latest = None
                    self._last_frame = self._last_send = None
                    self._last_outcome = None
                    self._paused = False
                    self._reason = ""
                    if self._threaded and self._thread is None:
                        self._thread = threading.Thread(target=self._run, daemon=True,
                                                        name="as2w-control-stream")
                        self._thread.start()
                    self._cv.notify_all()
            return self._result(True)

    def _result(self, ok, error=""):
        with self._cv:
            problem = error or self._fault
            return {"ok": ok, "state": "error" if problem else
                    "stopping" if self._stop_pending else "paused" if self._paused else "running",
                    **({"error": problem} if problem else {})}

    def submit(self, message):
        """Queue only; actual verdict is published after the worker runs the sink."""
        with self._cv:
            self._stats["received"] += 1
            if self._paused or self._sink.aborted or self._closing:
                self._stats["rejected"] += 1
                return {"ok": False, "verdict": "REJECTED", "reason": "controller is paused or faulted"}
            if self._latest is not None:
                self._stats["replaced"] += 1
            self._latest = Frame(deepcopy(message), self._generation)
            self._stats["queued"] += 1
            self._cv.notify_all()
            return {"ok": True, "verdict": "QUEUED"}

    def _request_stop_locked(self, reason, *, release=True):
        self._generation += 1
        self._latest = None
        self._paused = True
        self._reason = reason
        self._release_after_stop = release
        self._stop_pending = True
        self._stop_accepted = None
        self._stop_event.clear()
        self._cv.notify_all()

    def _execute_stop(self):
        try:
            self._sink.stop(self._reason)
        finally:
            self._finish_sink_operation()

    def _finish_sink_operation(self):
        # A callback can throw after StopMove succeeds (e.g. release fails).
        # Publish completion only after ControlSink has latched that failure.
        with self._cv:
            if self._stop_pending and self._completed_stop_generation == self._generation:
                self._stop_pending = False
                self._stop_event.set()

    def pause(self, reason="paused", *, timeout=0.0):
        with self._transition:
            with self._cv:
                if not self._stop_pending:
                    self._request_stop_locked(reason)
            if not self._threaded or self._thread is None:
                self.pump()
        if timeout > 0:
            self._stop_event.wait(timeout)
        with self._cv:
            ok = not self._stop_pending and self._stop_accepted is True and not self._sink.aborted
            return self._result(ok, "stop acknowledgement pending" if self._stop_pending else "")

    def reset_fault(self, timeout=1.5):
        with self._transition:
            with self._cv:
                self._request_stop_locked("resetting fault", release=False)
            if not self._threaded or self._thread is None:
                self.pump()
            self._stop_event.wait(timeout)
            with self._writer:
                with self._cv:
                    if self._stop_pending or self._stop_accepted is not True:
                        return self._result(False, "cannot reset without successful StopMove acknowledgement")
                    if self._owned:
                        try:
                            self.client.release_control()
                        except Exception as exc:
                            self._sink.abort(f"control release failed: {exc}")
                            self._finish_sink_operation()
                            return self._result(False)
                        self._owned = False
                    self._sink.reset(initial_values=(0.0,) * 6,
                                     minimum_stamp_ms=self._wall() * 1000)
                    self._reason = "fault reset; explicitly resume to accept new frames"
                    if self._pending_dry_run:
                        self._dry_run, self._pending_dry_run = True, False
                    return self._result(True)

    def configure(self, *, dry_run=None, rotate_only=None):
        for name, value in (("dry_run", dry_run), ("rotate_only", rotate_only)):
            if value is not None and not isinstance(value, bool):
                return self._result(False, name + " must be boolean")
        with self._transition:
            with self._cv:
                if rotate_only is not None:
                    if not self._paused:
                        return self._result(False, "pause before changing rotate_only")
                    self._rotate_only = rotate_only
                if dry_run is None or dry_run == self._dry_run:
                    return self._result(not self._sink.aborted)
                if not dry_run:
                    if not self._paused or self._stop_pending or self._sink.aborted:
                        return self._result(False, "pause/reset_fault before enabling real control")
                    self._dry_run = False
                    self._generation += 1
                    self._latest = None
                    return self._result(True)
                self._pending_dry_run = True
                self._request_stop_locked("stopping before entering dry_run")
            if not self._threaded or self._thread is None:
                self.pump()
            return self._result(not self._sink.aborted)

    def _on_sink_stop(self):
        with self._cv:
            # Explicit stops set their release policy before entering the sink;
            # expiry/watchdog stops arrive directly from the sink worker.
            if not self._stop_pending:
                self._request_stop_locked(self._sink.last_stop_reason)
            generation = self._generation
        try:
            self._stop_hardware()
        finally:
            with self._cv:
                self._completed_stop_generation = generation

    def _on_sink_abort(self):
        with self._cv:
            self._request_stop_locked(self._sink.fault_reason, release=False)
        self._on_sink_stop()

    def _stop_hardware(self, *, release=None):
        with self._cv:
            owned = self._owned
            self._stats["stop_attempts"] += int(owned)
        try:
            ret = self.client.StopMove() if owned else 0
        except Exception as exc:
            ret = f"{type(exc).__name__}: {exc}"
        with self._cv:
            self._last_ret = self._last_stop_ret = ret
            self._stop_accepted = ret == 0 and not isinstance(ret, bool)
            self._physical_stopped = None
            self._stop_ack_ms = self._wall() * 1000 if owned and self._stop_accepted else None
            self._stationary_since = self._last_stop_sample = None
            self._stationary_samples = 0
            if not self._stop_accepted:
                raise RuntimeError(f"StopMove failed: {ret}; control remains held")
            self._stats["stop_accepted"] += int(owned)
            should_release = self._release_after_stop if release is None else release
            if should_release and self._owned and not self._sink.aborted:
                self.client.release_control()
                self._owned = False
            if self._pending_dry_run and not self._sink.aborted:
                self._dry_run, self._pending_dry_run = True, False

    def _cancelled(self):
        with self._cv:
            return (self._paused or self._stop_pending or
                    self._executing_generation != self._generation)

    def _before_apply(self, values, gripper):
        if self._cancelled():
            return Outcome(Verdict.DROPPED, "control session was cancelled")
        if not self._dry_run and any(values):
            problem = self._read_posture()
            if problem:
                raise RuntimeError(problem)
        if self._cancelled():
            return Outcome(Verdict.DROPPED, "control session was cancelled")
        # ControlSink rechecks freshness after this potentially slow SDK query.

    def _apply(self, values, gripper):
        with self._cv:
            if self._cancelled():
                return Outcome(Verdict.DROPPED, "control session was cancelled")
            self._last_send = self._clock()
            if self._dry_run:
                self._stats["simulated"] += 1
                return
            vx, vy, wz = values[0], values[1], values[5]
            if self._rotate_only:
                vx = vy = 0.0
            zero = not any((vx, vy, wz))
            if not zero:
                self._stats["attempted"] += 1
                self._physical_stopped = self._stop_ack_ms = None
                self._stop_accepted = None
        if zero:
            # A validated zero command holds ownership and keeps the stream
            # armed. Explicit pause/watchdog/abort instead use sink callbacks.
            self._stop_hardware(release=False)
            return
        try:
            ret = self.client.Move(vx, vy, wz)
        except Exception as exc:
            ret = f"{type(exc).__name__}: {exc}"
        with self._cv:
            self._last_ret = self._last_move_ret = ret
            if ret != 0 or isinstance(ret, bool):
                self._stats["sdk_errors"] += 1
                raise RuntimeError(f"Move failed: {ret}")
            self._stats["sdk_accepted"] += 1

    def _verify_stopped(self):
        """Only fresh, measured body vx/vy/wz after the ACK can confirm rest.

        Three distinct stationary samples spanning >=200 ms are required. Null
        axes (the AS2W default until measured) leave verification unknown.
        """
        if self._odom_provider is None:
            return
        try:
            sample = self._odom_provider()
        except Exception:
            sample = None
        with self._cv:
            ack = self._stop_ack_ms
            if ack is None:
                return
            sample = sample if isinstance(sample, dict) else {}
            stamp, twist = sample.get("stamp_ms"), sample.get("twist")
            usable = (sample.get("schema") == "motus.odom/1" and sample.get("frame") == "body"
                      and finite(stamp) and stamp >= ack and 0 <= self._wall() * 1000 - stamp <= 500
                      and isinstance(twist, (list, tuple)) and len(twist) == 6
                      and all(finite(twist[i]) for i in (0, 1, 5)))
            if not usable:
                self._physical_stopped = None
                self._stationary_since = None
                self._stationary_samples = 0
                self._last_stop_sample = None
                return
            if self._last_stop_sample is not None and stamp <= self._last_stop_sample:
                return
            self._last_stop_sample = stamp
            if math.hypot(twist[0], twist[1]) > 0.03 or abs(twist[5]) > 0.05:
                self._physical_stopped = False
                self._stationary_since = None
                self._stationary_samples = 0
                return
            if self._stationary_since is None:
                self._stationary_since = stamp
            self._stationary_samples += 1
            if self._stationary_samples >= 3 and stamp - self._stationary_since >= 200:
                self._physical_stopped = True

    def pump(self):
        """Run the shared check chain and SDK application on the sole writer."""
        with self._writer:
            self._verify_stopped()
            with self._cv:
                stopping = self._stop_pending
                paused = self._paused
            if stopping:
                self._execute_stop()
                return
            if paused:
                return
            # Only ControlSink owns TTL/watchdog state. Mailbox arrivals cannot
            # reset it; neither can rejected frames or an SDK failure.
            self._sink.tick()
            self._finish_sink_operation()
            with self._cv:
                if self._paused or self._latest is None:
                    return
                frame = self._latest
                values = frame.message.get("values") if isinstance(frame.message, dict) else None
                zero = isinstance(values, (list, tuple)) and len(values) == 6 and not any(values)
                if (not zero and self._last_send is not None and
                        self._clock() - self._last_send < self.period_s):
                    return
                self._latest = None
                self._executing_generation = frame.generation
            outcome = self._sink.submit(frame.message)
            self._finish_sink_operation()
            with self._cv:
                self._last_outcome = {"verdict": outcome.verdict.value,
                                      "reason": outcome.reason,
                                      "values": outcome.values,
                                      "warnings": list(outcome.warnings)}
                if outcome.applied:
                    self._last_frame = self._clock()
                    self._stats["validated"] += 1
                else:
                    self._stats["rejected"] += 1
                stopping = self._stop_pending
            if stopping:
                self._execute_stop()

    def _run(self):
        while True:
            try:
                self.pump()
            except Exception as exc:
                with self._writer:
                    self._sink.abort(f"control worker failed: {type(exc).__name__}: {exc}")
                    self._finish_sink_operation()
            with self._cv:
                if self._closing and not self._stop_pending:
                    return
                self._cv.wait(0.02)

    def close(self, timeout=1.5):
        result = self.pause("card shutdown", timeout=timeout)
        with self._cv:
            self._closing = True
            self._cv.notify_all()
        if self._thread:
            self._thread.join(timeout)
        return result

    def info(self):
        with self._cv:
            return {**self._result(not self._sink.aborted), "dry_run": self._dry_run,
                    "rotate_only": self._rotate_only, "require_standing": True,
                    "owner_acquired": self._owned, "generation": self._generation,
                    "reason": self._reason, "fault_latched": self._sink.aborted,
                    "last_ret": self._last_ret, "stop_acknowledged": self._stop_accepted,
                    "last_move_ret": self._last_move_ret, "last_stop_ret": self._last_stop_ret,
                    "physical_stop_verified": self._physical_stopped,
                    "control_call_bound_s": getattr(self.client, "max_call_seconds", None),
                    "stop_dispatch_note": "Stop waits for at most one in-flight bounded call, then a stop RPC; scheduling and physical braking are additional.",
                    "pending_frames": int(self._latest is not None),
                    "last_command_age_ms": None if self._last_frame is None else
                    round((self._clock() - self._last_frame) * 1000),
                    "safety_sink": self._sink.stats(), "last_outcome": self._last_outcome,
                    **self._stats}
