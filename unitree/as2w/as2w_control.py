"""Latest-value AS2W control, independent of ROS and of the vendor SDK.

The injected client must be a dedicated bounded control channel, implementing
Move/StopMove/GetState, acquire_control/release_control, control_ready and
max_call_seconds. A normal RpcProxy with multi-second serialized calls is not
this interface. There is exactly one SDK writer here; a stop invalidates queued
frames and runs immediately after any already in-flight bounded call.

SDK acknowledgement is deliberately separate from measured physical stopping.
No software timeout here establishes the firmware's behaviour after link loss.
"""
from __future__ import annotations

from dataclasses import dataclass
import math
import threading
import time


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
    values: tuple
    deadline: float
    generation: int


class VelocityController:
    """The ROS callback only validates and replaces one pending frame.

    ``pump`` is also the deterministic no-hardware test seam. Production calls
    it from one worker, never from the ROS subscriber or timer.
    """

    def __init__(self, client, descriptor, *, dry_run=True, rotate_only=False,
                 state_provider=None, conflict=None, odom_provider=None,
                 clock=time.monotonic, wall_clock=time.time, threaded=True):
        self.client = client
        self.descriptor = descriptor
        self._clock, self._wall = clock, wall_clock
        self._state_provider = state_provider
        self._conflict = conflict
        self._odom_provider = odom_provider
        self._cv = threading.Condition(threading.RLock())
        self._writer = threading.Lock()
        self._transition = threading.Lock()
        self._threaded = threaded
        self._thread = None
        self._closing = False
        self._generation = 0
        self._epoch_ms = 0
        self._latest = None
        self._seq = {}
        self._holder = None
        self._holder_priority = 0
        self._holder_seen = 0
        self._last_frame = None
        self._last_send = None
        self._active_deadline = None
        self._last_values = (0.0,) * 6
        self._state_checked = float("-inf")
        self._state_problem = ""
        self._paused = True
        self._owned = False
        self._stop_pending = False
        self._stop_event = threading.Event()
        self._stop_event.set()
        self._release_after_stop = False
        self._pending_dry_run = False
        self._fault = ""
        self._reason = "not started"
        self._dry_run = bool(dry_run)
        self._rotate_only = bool(rotate_only)
        self._last_ret = None
        self._last_move_ret = None
        self._last_stop_ret = None
        self._stop_accepted = None
        self._physical_stopped = None
        self._stop_ack_ms = None
        self._stationary_since = None
        self._stationary_samples = 0
        self._last_stop_sample = None
        self._stats = dict(received=0, validated=0, rejected=0, replaced=0,
                           simulated=0, attempted=0, sdk_accepted=0,
                           sdk_errors=0, stop_attempts=0, stop_accepted=0)

    @property
    def watchdog_s(self):
        return self.descriptor["rate"]["watchdog_ms"] / 1000.0

    @property
    def period_s(self):
        return 1.0 / self.descriptor["rate"]["expected_hz"]

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
        """Explicit start/resume. Never consumes pre-resume frames."""
        with self._transition:
            with self._cv:
                if self._fault:
                    return self._result(False, "fault is latched; call reset_fault first")
                if self._stop_pending:
                    return self._result(False, "stop acknowledgement is pending")
                if self._closing:
                    return self._result(False, "controller is closed")
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
                    return self._result(False, (claim or {}).get("error", "chassis is occupied")
                                        if isinstance(claim, dict) else "invalid control acquisition result")
                with self._cv:
                    self._owned = True
                    self._request_stop_locked("clearing prior velocity before stream start", release=False)
                with self._writer:
                    self._stop_hardware()
                with self._cv:
                    if self._fault:
                        return self._result(False)
                problem = self._read_posture(force=True)
                if problem:
                    # No Move has been issued. Release only this unused claim.
                    self.client.release_control()
                    with self._cv:
                        self._owned = False
                    return self._result(False, problem)
            with self._cv:
                self._generation += 1
                self._epoch_ms = self._wall() * 1000
                self._latest = None
                self._seq.clear()
                self._holder = None
                self._last_frame = None
                self._last_send = None
                self._active_deadline = None
                self._last_values = (0.0,) * 6
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
            return {"ok": ok, "state": "error" if error or self._fault else
                    "stopping" if self._stop_pending else "paused" if self._paused else "running",
                    **({"error": error or self._fault} if error or self._fault else {})}

    def _reject(self, reason):
        self._stats["rejected"] += 1
        return {"ok": False, "verdict": "REJECTED", "reason": reason}

    def submit(self, message):
        """No SDK or ROS calls; bounded validation then latest-only replacement."""
        with self._cv:
            self._stats["received"] += 1
            if self._paused or self._fault or self._closing:
                return self._reject("controller is paused or faulted")
            if not isinstance(message, dict):
                return self._reject("command must be an object")
            if (message.get("schema") != "motus.control/1" or message.get("mode") != "twist"
                    or isinstance(message.get("dof"), bool) or message.get("dof") != 6):
                return self._reject("expected motus.control/1 twist with dof=6")
            values = message.get("values")
            if not isinstance(values, (list, tuple)) or len(values) != 6 or not all(map(finite, values)):
                return self._reject("values must contain six finite numbers")
            limits = self.descriptor["limits"]
            if any(v < lo or v > hi for v, lo, hi in zip(values, limits["lower"], limits["upper"])):
                return self._reject("velocity is outside declared limits")
            stamp, obs, ttl = (message.get(k) for k in ("stamp_ms", "obs_stamp_ms", "ttl_ms"))
            if not all(map(finite, (stamp, obs, ttl))):
                return self._reject("finite stamp_ms, obs_stamp_ms and ttl_ms are required")
            now_ms = self._wall() * 1000
            max_obs = self.descriptor["rate"]["max_obs_age_ms"]
            if ttl <= 0 or ttl > self.watchdog_s * 1000:
                return self._reject("ttl_ms must be positive and no greater than watchdog_ms")
            if stamp < self._epoch_ms:
                return self._reject("command predates this start/resume epoch")
            if stamp > now_ms + 50 or obs > now_ms + 50 or obs > stamp + 50:
                return self._reject("command or observation timestamp is in the future")
            remaining_ms = min(ttl - (now_ms - stamp), max_obs - (now_ms - obs))
            if remaining_ms <= 0:
                return self._reject("command or observation has expired")
            source, session, seq = message.get("source"), message.get("session_id", ""), message.get("seq")
            if not isinstance(source, str) or not source or len(source) > 160:
                return self._reject("source must be a non-empty bounded string")
            if not isinstance(session, str) or len(session) > 160:
                return self._reject("session_id must be a bounded string")
            if isinstance(seq, bool) or not isinstance(seq, int) or not 0 <= seq < 2**63:
                return self._reject("seq must be a nonnegative integer")
            previous = self._seq.get(source)
            if previous and (session != previous[0] or seq <= previous[1]):
                return self._reject("source restarted or sequence replayed; pause/resume establishes a new session")
            if source not in self._seq and len(self._seq) >= 32:
                return self._reject("too many control sources in this session")
            priority = message.get("priority", 0)
            if isinstance(priority, bool) or not isinstance(priority, int) or not 0 <= priority <= 100:
                return self._reject("priority must be an integer in [0,100]")
            now = self._clock()
            if (self._holder and source != self._holder and
                    now - self._holder_seen <= self.watchdog_s and priority <= self._holder_priority):
                return self._reject("another stream owns the current control lease")
            self._seq[source] = (session, seq)
            self._holder, self._holder_priority, self._holder_seen = source, priority, now
            self._last_frame = now
            self._stats["validated"] += 1
            if self._latest is not None:
                self._stats["replaced"] += 1
            # Monotonic deadlines remain valid if the wall clock later changes.
            self._latest = Frame(tuple(float(v) for v in values),
                                 now + remaining_ms / 1000, self._generation)
            self._cv.notify_all()
            return {"ok": True, "verdict": "QUEUED"}

    def _request_stop_locked(self, reason, *, release=True, fault=False):
        self._generation += 1
        self._latest = None
        self._paused = True
        self._reason = reason
        if fault:
            self._fault = reason
        self._release_after_stop = bool(release and not self._fault)
        self._stop_accepted = None
        self._stop_pending = True
        self._stop_event.clear()
        self._cv.notify_all()

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
            ok = not self._stop_pending and self._stop_accepted is True and not self._fault
            return self._result(ok, "stop acknowledgement pending" if self._stop_pending else self._fault)

    def reset_fault(self, timeout=1.5):
        """An explicit acknowledgement, only after StopMove has succeeded."""
        with self._transition:
            with self._cv:
                self._request_stop_locked("resetting fault", release=False)
            if not self._threaded or self._thread is None:
                self.pump()
            self._stop_event.wait(timeout)
            with self._cv:
                if self._stop_pending or self._stop_accepted is not True:
                    return self._result(False, "cannot reset without successful StopMove acknowledgement")
                if self._owned:
                    self.client.release_control()
                    self._owned = False
                self._fault = ""
                self._reason = "fault reset; explicitly resume to accept new frames"
                if self._pending_dry_run:
                    self._dry_run = True
                    self._pending_dry_run = False
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
                    return self._result(not bool(self._fault))
                if not dry_run:
                    if not self._paused or self._stop_pending or self._fault:
                        return self._result(False, "pause/reset_fault before enabling real control")
                    self._dry_run = False
                    self._generation += 1
                    self._latest = None
                    return self._result(True)
                # Never let dry_run suppress the stop for a previous real Move.
                self._pending_dry_run = True
                self._request_stop_locked("stopping before entering dry_run")
            if not self._threaded or self._thread is None:
                self.pump()
            return self._result(not bool(self._fault))

    def _stop_hardware(self):
        with self._cv:
            owned = self._owned
            self._stats["stop_attempts"] += int(owned)
        try:
            ret = self.client.StopMove() if owned else 0
        except Exception as exc:
            ret = f"{type(exc).__name__}: {exc}"
        with self._cv:
            self._last_ret = ret
            self._last_stop_ret = ret
            self._stop_accepted = (ret == 0 and not isinstance(ret, bool))
            self._physical_stopped = None
            self._active_deadline = None
            self._stop_ack_ms = self._wall() * 1000 if owned and self._stop_accepted else None
            self._stationary_since = None
            self._stationary_samples = 0
            self._last_stop_sample = None
            if self._stop_accepted:
                self._stats["stop_accepted"] += int(owned)
                self._last_values = (0.0,) * 6
                if self._release_after_stop and self._owned:
                    try:
                        self.client.release_control()
                    except Exception as exc:
                        self._fault = f"control release failed: {exc}"
                    else:
                        self._owned = False
                if self._pending_dry_run and not self._fault:
                    self._dry_run = True
                    self._pending_dry_run = False
            else:
                self._fault = f"StopMove failed: {ret}; control remains latched"
            self._stop_pending = False
            self._stop_event.set()

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
        """One worker turn. Stop work always wins over any queued frame."""
        with self._writer:
            self._verify_stopped()
            with self._cv:
                now = self._clock()
                if (not self._paused and self._active_deadline is not None
                        and now >= self._active_deadline):
                    self._request_stop_locked("executed command TTL expired")
                elif (not self._paused and self._last_frame is not None
                        and now - self._last_frame >= self.watchdog_s):
                    self._request_stop_locked("control stream watchdog expired")
                stopping = self._stop_pending
                if not stopping:
                    if self._paused or self._latest is None:
                        return
                    zero = not any(self._latest.values[2:] if self._rotate_only else self._latest.values)
                    if (not zero and self._last_send is not None
                            and now - self._last_send < self.period_s):
                        return
                    frame, self._latest = self._latest, None
                    dry_run = self._dry_run
            if stopping:
                self._stop_hardware()
                return
            if not dry_run and not zero:
                problem = self._read_posture()
                if problem:
                    with self._cv:
                        self._request_stop_locked(problem, release=False, fault=True)
                    self._stop_hardware()
                    return
            with self._cv:
                now = self._clock()
                if frame.generation != self._generation or self._paused or self._stop_pending:
                    return
                if now >= frame.deadline:
                    self._stats["rejected"] += 1
                    self._request_stop_locked("queued command expired before SDK dispatch")
                    return
                values = list(frame.values)
                if self._rotate_only:
                    values[0] = values[1] = 0.0
                # Slew from rest, based on elapsed monotonic time. A full zero
                # command bypasses acceleration shaping and stops immediately.
                zero = not any(values)
                dt = min(self.period_s if self._last_send is None else now - self._last_send,
                         self.watchdog_s)
                if not zero:
                    for i, acceleration in enumerate(self.descriptor["acceleration_limits"]):
                        delta = acceleration * dt
                        values[i] = max(self._last_values[i] - delta,
                                        min(self._last_values[i] + delta, values[i]))
                self._last_send = now
                if dry_run:
                    self._stats["simulated"] += 1
                    self._last_values = tuple(values)
                    self._active_deadline = frame.deadline if not zero else None
                    return
                self._stats["stop_attempts" if zero else "attempted"] += 1
                if not zero:
                    self._physical_stopped = None
                    self._stop_ack_ms = None
            try:
                ret = (self.client.StopMove() if zero else
                       self.client.Move(values[0], values[1], values[5]))
            except Exception as exc:
                ret = f"{type(exc).__name__}: {exc}"
            with self._cv:
                self._last_ret = ret
                if zero:
                    self._last_stop_ret = ret
                else:
                    self._last_move_ret = ret
                if ret != 0 or isinstance(ret, bool):
                    self._stats["sdk_errors"] += 1
                    self._request_stop_locked(f"{'StopMove' if zero else 'Move'} failed: {ret}",
                                              release=False, fault=True)
                else:
                    self._stats["stop_accepted" if zero else "sdk_accepted"] += 1
                    self._last_values = tuple(values)
                    self._active_deadline = frame.deadline if not zero else None
                    self._stop_accepted = True if zero else None
                    if zero and self._stop_ack_ms is None:
                        self._stop_ack_ms = self._wall() * 1000
                        self._stationary_since = None
                        self._stationary_samples = 0
                        self._last_stop_sample = None
                    if not zero and self._clock() >= frame.deadline:
                        self._request_stop_locked("command TTL expired while awaiting SDK acknowledgement")
                stopping = self._stop_pending
            if stopping:
                self._stop_hardware()

    def _run(self):
        while True:
            try:
                self.pump()
            except Exception as exc:
                with self._cv:
                    self._request_stop_locked(f"control worker failed: {type(exc).__name__}: {exc}",
                                              release=False, fault=True)
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
            return {**self._result(not bool(self._fault)), "dry_run": self._dry_run,
                    "rotate_only": self._rotate_only, "require_standing": True,
                    "owner_acquired": self._owned, "generation": self._generation,
                    "reason": self._reason, "fault_latched": bool(self._fault),
                    "last_ret": self._last_ret, "stop_acknowledged": self._stop_accepted,
                    "last_move_ret": self._last_move_ret, "last_stop_ret": self._last_stop_ret,
                    "physical_stop_verified": self._physical_stopped,
                    "control_call_bound_s": getattr(self.client, "max_call_seconds", None),
                    "stop_dispatch_note": "Stop waits for at most one in-flight bounded call, then a stop RPC; scheduling and physical braking are additional.",
                    "pending_frames": int(self._latest is not None),
                    "last_command_age_ms": None if self._last_frame is None else
                    round((self._clock() - self._last_frame) * 1000), **self._stats}
