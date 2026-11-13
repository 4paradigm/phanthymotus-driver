"""Sound-facing supervision, with no ROS, SDK, acoustic or motor algorithm.

State changes are serialized, but external source queries never own the state
lock. Motion remains in the existing LocoPlugin. Operator signatures grant a
bounded permission; Agent/Canvas renew only the shorter tracking session.
"""

from __future__ import annotations

import math
import threading
import time
from collections import deque
from dataclasses import asdict, dataclass, fields

from sound_facing_authority import OperatorAuthority


CONVENTION = "robot_front_0_left_positive_ccw"
PARAMETER_BOUNDS = {
    "deadzone_deg": (3, 60), "release_deg": (0, 30), "spread_deg": (1, 30),
    "stable_windows": (3, 20), "stable_span_s": (0.2, 5), "max_age_s": (0.1, 2),
    "max_window_s": (0.05, 2), "max_gap_s": (0.1, 2), "stream_timeout_s": (0.3, 5),
    "settle_s": (0.5, 5), "reversal_hold_s": (1, 10),
    "turn_timeout_s": (1, 80), "session_timeout_s": (1, 30),
}


@dataclass(frozen=True)
class FacingConfig:
    deadzone_deg: float = 12.0
    release_deg: float = 6.0
    spread_deg: float = 12.0
    stable_windows: int = 3
    stable_span_s: float = 0.6
    max_age_s: float = 1.0
    max_window_s: float = 1.0
    max_gap_s: float = 0.8
    stream_timeout_s: float = 1.5
    settle_s: float = 0.8
    reversal_hold_s: float = 2.0
    turn_timeout_s: float = 80.0
    session_timeout_s: float = 30.0

    @classmethod
    def parse(cls, changes):
        if not isinstance(changes, dict) or set(changes) - {f.name for f in fields(cls)}:
            raise ValueError("unknown_configuration")
        cfg = cls(**changes)
        for name, value in asdict(cfg).items():
            if type(value) not in (int, float) or not math.isfinite(value):
                raise ValueError(f"invalid_{name}")
        if type(cfg.stable_windows) is not int or not 3 <= cfg.stable_windows <= 20:
            raise ValueError("stable_windows must be an integer from 3 to 20")
        for name, (low, high) in PARAMETER_BOUNDS.items():
            if not low <= getattr(cfg, name) <= high:
                raise ValueError(f"{name} must be within {low}..{high}")
        if cfg.release_deg >= cfg.deadzone_deg or cfg.max_gap_s > cfg.stream_timeout_s:
            raise ValueError("invalid_hysteresis_or_gap")
        return cfg

    @classmethod
    def schema_properties(cls):
        return {name: {"type": "integer" if name == "stable_windows" else "number",
                       "default": value, "minimum": PARAMETER_BOUNDS[name][0],
                       "maximum": PARAMETER_BOUNDS[name][1]}
                for name, value in asdict(cls()).items()}


def delta_deg(a, b):
    return (a - b + 180) % 360 - 180


class DirectionGate:
    """Filter observations, counting only disjoint acoustic capture windows."""

    def __init__(self, config):
        self.cfg = config
        self.samples = deque(maxlen=config.stable_windows)
        self.seen = (0, 0)

    def clear(self):
        self.samples.clear()

    def observe(self, data, wall_us, cutoff_us):
        cfg = self.cfg
        if not isinstance(data, dict) or data.get("state") != "fresh":
            self.clear()
            return None, "no_reliable_observation"
        angle = data.get("angle_deg")
        begin, end = data.get("audio_window_start_us"), data.get("audio_window_end_us")
        published = data.get("timestamp_ms")
        if (data.get("angle_convention") != CONVENTION
                or type(angle) not in (float, int) or not math.isfinite(angle)
                or not -180 <= angle <= 180
                or type(begin) is not int or type(end) is not int
                or type(published) is not int
                or not 1_000_000_000_000_000 <= begin < end
                or end - begin > cfg.max_window_s * 1e6
                or not 0 <= wall_us - end <= cfg.max_age_s * 1e6
                or not 0 <= wall_us - published * 1000 <= cfg.max_age_s * 1e6
                or published * 1000 + 1000 < end):
            self.clear()
            return None, "invalid_or_expired_observation"
        if begin <= cutoff_us:
            self.clear()
            return None, "before_observation_boundary"
        if begin <= self.seen[0] or end <= self.seen[1]:
            self.clear()
            return None, "duplicate_or_out_of_order"
        self.seen = (begin, end)
        # Contradictory sliding windows invalidate evidence too, even when
        # they cannot count as another independent sample.
        if self.samples and any(abs(delta_deg(angle, p[2])) > cfg.spread_deg for p in self.samples):
            self.clear()
        if self.samples and begin < self.samples[-1][1]:
            return None, "overlapping_window"
        if self.samples and begin - self.samples[-1][1] > cfg.max_gap_s * 1e6:
            self.clear()
        self.samples.append((begin, end, angle))
        if (len(self.samples) < cfg.stable_windows
                or end - self.samples[0][0] < cfg.stable_span_s * 1e6):
            return None, "collecting_stable_windows"
        # Circular mean prevents +179/-179 being averaged to the front.
        radians = [math.radians(p[2]) for p in self.samples]
        mean = math.degrees(math.atan2(sum(map(math.sin, radians)), sum(map(math.cos, radians))))
        # Keep the observed side at the exactly ambiguous rear boundary.
        if abs(abs(mean) - 180) < 1e-6:
            mean = math.copysign(180.0, self.samples[-1][2])
        return mean, "stable"


class SourceHealthProbe:
    """One bounded caller wait and at most one outstanding external query.

    A stuck Python callback cannot be killed. It remains a single daemon;
    callers fail closed after 50ms, and invalidation discards its late result.
    No external callback runs under this probe's lock or the engine lock.
    """

    def __init__(self, callback):
        self.callback = callback
        self.lock = threading.Lock()
        self.version = 0
        self.result = None
        self.worker = None
        self._query = None

    def invalidate(self):
        with self.lock:
            self.version += 1
            self.result = None
            # Retire and wake this query immediately. The external callback
            # may remain blocked, but no caller needs to wait for it after
            # revocation; its generation check still refuses late results.
            if self._query is not None:
                self._query["done"].set()

    def read(self):
        if self.callback is None:
            return True
        with self.lock:
            if self.worker is None or not self.worker.is_alive():
                self.result = None
                self._query = {"version": self.version, "done": threading.Event(), "result": None}
                self.worker = threading.Thread(target=self._read, args=(self._query,),
                                               name="bumi_sound_source_health", daemon=True)
                self.worker.start()
            query = self._query
        query["done"].wait(0.05)
        with self.lock:
            # Another caller may already have begun a newer query. Read the
            # result associated with our wait, never its mutable shared slot.
            result = query["result"]
            return bool(query["version"] == self.version and result is not None
                        and 0 <= time.monotonic() - result[0] <= 0.1 and result[1])

    def _read(self, query):
        try:
            healthy = self.callback() is True
        except Exception:
            healthy = False
        with self.lock:
            if self.version == query["version"]:
                self.result = (time.monotonic(), healthy)
                query["result"] = self.result
        query["done"].set()


class SoundFacing:
    """Continuous session; one correlated local turn, then settle and reobserve.

    Sessions have a monotonic deadline, renewed only by an explicit MCP action
    matching the current process-local session ID. Sensor traffic, status reads
    and the supervisor cannot renew it. IDs correlate sessions, not identities.
    """

    def __init__(self, motion, config=None, monotonic=time.monotonic,
                 wall_ns=time.time_ns, source_health=None, authority=None, device="bumi"):
        self.motion = motion
        self.cfg = FacingConfig.parse(config or {})
        self._mono, self._wall = monotonic, wall_ns
        self.authority = authority if authority is not None else OperatorAuthority(device, monotonic=monotonic)
        self._source_probe = SourceHealthProbe(source_health)
        self.lock = threading.RLock()
        self._signal_lock = threading.Lock()
        self._stop_epoch = self._stop_applied = 0
        self._stop_requested = threading.Event()
        self._generation = 0
        self.owner = object()
        self.gate = DirectionGate(self.cfg)
        self.state, self.reason = "idle", "disabled"
        self.running = False
        self.session_id = None
        self._expires = 0.0
        self.reserved = False
        self.action_id = None
        self.last_result = None
        self._stop_unconfirmed = False
        self._draining = False
        self._had_motion = False
        self._cancel_requested_at = None
        self.turns = 0
        self._last_side = 0
        self._aligned = False
        self._last_turn_at = -math.inf
        self._received = self._mono()
        self._clock_pair = self._sample_clock()
        self._clock_diagnostics = None

    def _sample_clock(self):
        before = self._mono()
        wall = self._wall() / 1e9
        after = self._mono()
        return before, wall, after

    def interrupt(self):
        """Independent, owner-scoped signal; no state/source/lifecycle lock."""
        with self._signal_lock:
            self._stop_epoch += 1
            epoch = self._stop_epoch
            self._stop_requested.set()
        interrupt = getattr(self.motion, "interrupt_owned_session", None)
        if interrupt is not None:
            interrupt(self.owner)
        elif self.action_id is not None:
            self.motion.cancel_owned_rotation(self.owner, self.action_id)
        return epoch

    def prepare_authorization(self):
        with self.lock:
            if self.session_id or self.reserved or self.action_id:
                return {**self.snapshot(), "error": "stop_and_release_before_new_authorization"}
            with self._signal_lock:
                if self._stop_epoch != self._stop_applied:
                    return {**self.snapshot(), "error": "cancellation_pending"}
                self._stop_requested.clear()
            self._generation += 1
            self.authority.invalidate()
            self._source_probe.invalidate()
            return self.snapshot()

    def authorize(self, document):
        # Cryptographic parsing/verification is outside the engine lock too.
        try:
            self.authority.authorize(document)
            return self.snapshot()
        except ValueError as exc:
            return {**self.snapshot(), "error": str(exc)}
        except Exception:
            with self.lock:
                self._fault("operator_authorization_unavailable")
                return self.snapshot()

    def snapshot(self):
        with self.lock:
            return {"state": self.state, "reason": self.reason,
                    "running": self.running, "enabled": self.session_id is not None,
                    "session_id": self.session_id,
                    "session_remaining_s": max(0.0, self._expires - self._mono()) if self.session_id else 0.0,
                    "motion_reserved": self.reserved, "turn_action_id": self.action_id,
                    "turns_completed": self.turns, "last_result": self.last_result,
                    "settings": asdict(self.cfg), "authorization": self.authority.snapshot(),
                    "clock_diagnostics": self._clock_diagnostics,
                    "physical_motion_verified": False}

    def start(self):
        with self.lock:
            self.running = True  # registration/start never enables tracking
            return self.snapshot()

    def configure(self, changes):
        with self.lock:
            cfg = FacingConfig.parse({**asdict(self.cfg), **changes})
            if cfg != self.cfg:
                self.disable()
                if self.reserved:
                    raise ValueError("wait_for_owned_motion_to_stop")
                self.cfg, self.gate = cfg, DirectionGate(cfg)
            return self.snapshot()

    def enable(self):
        self.tick()
        # Snapshot the intent, then query the external source without the lock.
        with self.lock:
            if not self.running:
                return {**self.snapshot(), "error": "card_not_started"}
            if self.session_id is not None:
                return self.snapshot()
            if self.reserved or self.action_id:
                return {**self.snapshot(), "error": "previous_motion_not_released"}
            approved = self.authority.snapshot()
            if approved["state"] != "approved":
                return {**self.snapshot(), "error": "operator_authorization_required" if approved["available"]
                        else "operator_authorization_unavailable"}
            generation = self._generation
            stop_epoch = self._stop_epoch
        source_ready = self._source_probe.read()
        with self.lock:
            if (generation != self._generation or stop_epoch != self._stop_epoch
                    or self._stop_requested.is_set() or not self.running):
                return {**self.snapshot(), "error": "enable_cancelled"}
            if self.session_id or self.reserved:
                return self.snapshot()
            try:
                if not source_ready:
                    raise RuntimeError("sound_direction_unavailable")
                session_id = self.authority.claim()
                # Install the provisional lease before the Loco health check;
                # the state lock hides it from callers until preflight commits.
                # Failure below always revokes it and releases the reservation.
                self.session_id = session_id
                self._expires = min(self._mono() + self.cfg.session_timeout_s,
                                    self.authority.deadline(session_id))
                result = self.motion.reserve_relative_motion(
                    self.owner, authorization=lambda: self._motion_authorized(session_id))
                if result.get("state") != "reserved":
                    raise RuntimeError(result.get("error", "reservation_failed"))
                self.reserved = True
                health = self.motion.relative_motion_health(self.owner)
                if health.get("state") != "ready":
                    raise RuntimeError(health.get("error", "health_failed"))
                self._health = health
                if self._stop_requested.is_set():
                    raise RuntimeError("enable_cancelled")
                now = self._mono()
                cutoff_us = self._wall() // 1000
                clock_pair = self._sample_clock()
                authorization_deadline = self.authority.deadline(session_id)
            except Exception as exc:
                self._fault(str(exc))
                return self.snapshot()
            self.session_id = session_id
            self._expires = min(self._expires, authorization_deadline)
            self._received = now
            self._cutoff_us = cutoff_us
            self._clock_pair = clock_pair
            self.gate = DirectionGate(self.cfg)
            self._aligned, self._last_side = False, 0
            self._had_motion = False
            self._cancel_requested_at = None
            self.state, self.reason = "observing", "explicitly_enabled"
            return self.snapshot()

    def _motion_authorized(self, session_id):
        """Memory-only predicate also enforced at the real publisher boundary.

        Never takes the engine lock: delayed SDK preflight cannot extend a
        session or make the worker depend on the supervisor acquiring it.
        """
        if (self._stop_requested.is_set() or self.session_id != session_id
                or self._mono() >= self._expires):
            return False
        try:
            self.authority.deadline(session_id)
            return True
        except Exception:
            return False

    def renew(self, session_id):
        self.tick()
        with self.lock:
            if not self.session_id or not isinstance(session_id, str) or session_id != self.session_id:
                return {**self.snapshot(), "error": "session_not_active_or_mismatched"}
            try:
                self._expires = min(self._mono() + self.cfg.session_timeout_s,
                                    self.authority.deadline(session_id))
            except Exception:
                self._fault("operator_authorization_expired_or_revoked")
            return self.snapshot()

    def _revoke(self):
        self._generation += 1
        self.authority.invalidate()
        self._source_probe.invalidate()
        self.session_id = None
        self._expires = 0.0

    def _release(self):
        if self.reserved and self.action_id is None and not self._stop_unconfirmed:
            if self._had_motion and not self._draining:
                self._draining = True
                self._drain_deadline = self._mono() + max(6.0, self.cfg.settle_s * 3)
                self._quiet_since = None
                self._quiet_anchor = None
            if self._draining:
                stability = self.motion.owned_motion_stability(self.owner)
                if stability.get("state") != "ready":
                    self._quiet_since = None
                elif (stability.get("stationary") is not True or self._quiet_anchor is None
                      or abs(delta_deg(stability["yaw_deg"], self._quiet_anchor)) > 0.5):
                    self._quiet_since = self._mono() if stability.get("stationary") is True else None
                    self._quiet_anchor = stability["yaw_deg"]
                elif self._quiet_since is None:
                    self._quiet_since = self._mono()
                if self._quiet_since is None or self._mono() - self._quiet_since < self.cfg.settle_s:
                    if self._mono() >= self._drain_deadline:
                        self._stop_unconfirmed = True
                        self.state, self.reason = "fault", "owned_stop_unconfirmed"
                    elif self.state != "fault":
                        self.state, self.reason = "stopping", "awaiting_imu_quiet"
                    return
            result = self.motion.release_relative_motion(self.owner)
            if result.get("state") != "released":
                self.state, self.reason = "fault", "ownership_release_failed"
            else:
                self.reserved = False
                self._draining = False
                if self.state != "fault":
                    self.state, self.reason = "idle", "disabled"

    def _cancel(self):
        if self.action_id:
            if self._cancel_requested_at is None:
                self._cancel_requested_at = self._mono()
            result = self.motion.cancel_owned_rotation(self.owner, self.action_id)
            if result.get("state") not in ("cancelling", "unchanged"):
                self.state, self.reason = "fault", "owned_cancel_failed"

    def _fault(self, reason):
        epoch = self.interrupt()
        self.state, self.reason = "fault", reason
        self.gate.clear()
        self._revoke()
        self._stop_applied = max(self._stop_applied, epoch)
        try:
            self._cancel()
            self._release()
        except Exception:
            self.reason += "; downstream_cleanup_unavailable"

    def disable(self):
        epoch = self.interrupt()
        with self.lock:
            self.gate.clear()
            self.state, self.reason = ("stopping", "cancelling") if self.action_id else ("idle", "disabled")
            self._revoke()
            self._stop_applied = max(self._stop_applied, epoch)
            if self._stop_unconfirmed:
                self.state, self.reason = "fault", "owned_stop_unconfirmed"
            try:
                self._cancel()
                self._release()
            except Exception:
                self.state, self.reason = "fault", "downstream_cleanup_unavailable"
            return self.snapshot()

    def observe(self, data):
        self.tick()
        with self.lock:
            if not self.session_id:
                return
            if isinstance(data, dict) and data.get("state") in ("fresh", "no_event", "stale"):
                self._received = self._mono()
            # Movement noise and pre-turn queued samples cannot contribute.
            if self.action_id or self.state == "settling":
                return
            if self._health.get("stationary") is not True:
                self.gate.clear()
                self.state, self.reason = "paused", "robot_not_stationary"
                return
            angle, reason = self.gate.observe(data, self._wall() // 1000, self._cutoff_us)
            self.state, self.reason = "paused", reason
            if angle is None:
                return
            now = self._mono()
            magnitude = abs(angle)
            if magnitude <= self.cfg.release_deg:
                self._aligned = True
            if magnitude < self.cfg.deadzone_deg:
                self.state, self.reason = "observing", "within_deadzone"
                return
            # To cross sides again, require a larger angle until a front
            # observation has crossed the release band, plus a cooldown.
            side = 1 if angle > 0 else -1
            if self._last_side and side != self._last_side:
                if (now - self._last_turn_at < self.cfg.reversal_hold_s
                        or (not self._aligned and magnitude < self.cfg.deadzone_deg + self.cfg.release_deg)):
                    self.reason = "reversal_hysteresis"
                    return
            try:
                self.authority.deadline(self.session_id)
                if self._stop_requested.is_set():
                    raise RuntimeError("session_cancelled")
                health = self.motion.relative_motion_health(self.owner)
                if health.get("state") != "ready":
                    raise RuntimeError(health.get("error", "health_failed"))
                if health.get("stationary") is not True:
                    self.gate.clear()
                    self.state, self.reason = "paused", "robot_not_stationary"
                    return
                # A concurrent stop is sticky at the Loco acceptance boundary.
                self.authority.deadline(self.session_id)
                if self._stop_requested.is_set():
                    raise RuntimeError("session_cancelled")
                result = self.motion.rotate_owned(self.owner, angle)
                if result.get("state") != "running" or not isinstance(result.get("action_id"), str):
                    raise RuntimeError(result.get("error", "rotation_not_accepted"))
                self.action_id = result["action_id"]
                self._had_motion = True
                self._requested_angle = angle
                self._deadline = now + self.cfg.turn_timeout_s
                self._last_side, self._aligned = side, False
                self.gate.clear()
                self.state, self.reason = "turning", "owned_rotation_running"
            except Exception as exc:
                self._fault(str(exc))

    def tick(self):
        with self.lock:
            generation = self._generation
            active = self.session_id is not None
            # Check independent deadlines before waiting even 50ms for a probe.
            if active:
                try:
                    self.authority.deadline(self.session_id)
                except Exception:
                    self._fault("operator_authorization_expired_or_revoked")
                    active = False
                if active and self._mono() >= self._expires:
                    self._fault("session_expired")
                    active = False
        source_ready = self._source_probe.read() if active else True
        with self.lock:
            if generation != self._generation:
                return self.snapshot()  # a retired query cannot validate a new intent
            now = self._mono()
            try:
                if self.session_id is not None:
                    old_before, old_wall, old_after = self._clock_pair
                    before, wall, after = self._sample_clock()
                    delta_wall = wall - old_wall
                    self._clock_diagnostics = {"sample_span_s": after - before,
                        "wall_delta_s": delta_wall, "monotonic_delta_min_s": before - old_after,
                        "monotonic_delta_max_s": after - old_before, "tolerance_s": 0.25}
                    if after - before > 0.25:
                        self._fault("capture_clock_sampling_delayed")
                    elif (before < old_before
                            or delta_wall < before - old_after - 0.25
                            or delta_wall > after - old_before + 0.25):
                        self._fault("capture_clock_discontinuity")
                    elif now >= self._expires:
                        self._fault("session_expired")
                    elif now - self._received > self.cfg.stream_timeout_s:
                        self._fault("sound_stream_lost")
                    elif self._stop_requested.is_set():
                        self._fault("session_cancelled")
                    elif not source_ready:
                        self._fault("sound_direction_unavailable")
                    else:
                        self.authority.deadline(self.session_id)
                        self._clock_pair = (before, wall, after)
                        health = self.motion.relative_motion_health(self.owner)
                        if health.get("state") != "ready":
                            self._fault(health.get("error", "motion_health_failed"))
                        else:
                            self._health = health
                if self.action_id is not None:
                    receipt = self.motion.owned_rotation_status(self.owner, self.action_id)
                    if receipt.get("state") == "terminal" and receipt.get("action_id") == self.action_id:
                        self.last_result = dict(receipt["result"])
                        self.action_id = None
                        if self.last_result.get("stop_command_sent") is not True:
                            self._stop_unconfirmed = True
                            self._fault("owned_stop_unconfirmed")
                            return self.snapshot()
                        if self.session_id is None:
                            self._release()
                        elif self.last_result.get("state") != "done":
                            self._fault("rotation_" + str(self.last_result.get("state", "failed")))
                        else:
                            error = self.last_result.get("final_error_deg")
                            requested = self.last_result.get("requested_angle_deg")
                            if (type(error) not in (float, int) or not math.isfinite(error)
                                    or abs(error) > 2.0 or requested != self._requested_angle):
                                self._fault("invalid_rotation_completion")
                                return self.snapshot()
                            self.turns += 1
                            self._last_turn_at = now
                            self._settle_until = now + self.cfg.settle_s
                            self._settle_deadline = now + max(6.0, self.cfg.settle_s * 3)
                            self._settle_anchor = health["yaw_deg"]
                            self.gate.clear()
                            self.state, self.reason = "settling", "imu_loop_completed"
                    elif receipt.get("state") != "running":
                        self._fault("uncorrelated_or_missing_motion_receipt")
                    elif self.session_id is not None and now >= self._deadline:
                        self._fault("rotation_timeout")
                    elif (self._cancel_requested_at is not None
                          and now - self._cancel_requested_at >= 5.0):
                        self._stop_unconfirmed = True
                        self.state, self.reason = "fault", "owned_stop_unconfirmed"
                elif self._draining:
                    self._release()
                elif self.session_id and self.state == "settling":
                    if now >= self._settle_deadline:
                        self._fault("settling_timeout")
                    elif (not health["stationary"]
                          or abs(delta_deg(health["yaw_deg"], self._settle_anchor)) > 0.5):
                        self._settle_until = now + self.cfg.settle_s
                        self._settle_anchor = health["yaw_deg"]
                        self.reason = "settling_motion_detected"
                    elif now >= self._settle_until:
                        # Require the WHOLE acoustic window to be after settling,
                        # not merely a recent receive or publication timestamp.
                        self._cutoff_us = self._wall() // 1000
                        self.gate.clear()
                        self.state, self.reason = "observing", "awaiting_post_turn_windows"
            except Exception as exc:
                self._fault("downstream_unavailable: " + type(exc).__name__)
            return self.snapshot()
