"""ControlSink — every check a command passes before it reaches a motor.

One shared implementation rather than one per driver. There are fourteen
bundles in this repo; a safety chain copied fourteen times is a safety chain
that diverges fourteen ways, and the copy that drifts is the one on the robot
you are not looking at.

The chain, in order. The distinction between *dropped* and *rejected* runs
through all of it: dropped is the network being a network, rejected is somebody
having wired something up wrong, and only the second is worth waking a person
for.

  1. contract        schema / mode / dof against the descriptor  → REJECTED
  2. freshness       ttl expiry, stale observation, seq regress  → DROPPED
  3. arbitration     priority, then latest seq                   → DROPPED
  4. step clamp      max_delta_per_step                          → CLAMPED
  5. hard limits     position bounds, max_velocity               → REJECTED
  6. collision       *not implemented* — see below
  7. force-torque    per-axis absolute threshold                 → ABORTED
  8. continuity      *caller's job* — see below
  9. watchdog        no valid command within watchdog_ms         → hold
 10. escalation      N consecutive watchdog periods              → abort

Two of those are honest gaps rather than implementations:

**6. Collision re-validation is not here and cannot be faked.** MoveIt Pro
checks every point of a chunk against a planning scene, with padding, and keeps
monitoring the scene so that an object appearing mid-run stops the robot. We
have no planning scene — only a URDF from a driver's `resource` tool, with no
scene representation and no FK/collision runtime. `workspace` (an optional
Cartesian bounding box in the descriptor) catches a policy running away; it
catches nothing at all on a tabletop. Until a scene representation exists, the
real mitigations are procedural: run in simulation until you trust the policy,
then first real runs at reduced speed with a person and a physical e-stop.

**8. Continuity — smoothing, densifying and blending chunks, and sizing the
committed window — belongs to whatever executes a trajectory, which is the
driver's own controller.** It is named in the chain because it is part of
getting this right, not because this class does it. One consequence is worth
stating where the driver author will read it: the committed window must be at
least the p99 inference latency or the robot pauses between chunks, and the
larger it is the longer e-stop takes to actually stop — that trade-off has to
be made deliberately and reported in `info()`, not defaulted.

**A pause is not a safe state.** When commands stop arriving the robot holds,
and it resumes the moment a valid command lands — without warning. The `ttl_ms`
check is what keeps it from resuming on a stale command, so `ttl_ms` has to be
set from measured latency; set it generously and the protection is gone while
still appearing to be there. Whatever wires this class to a topic should
announce the resume on the activity stream.

ROS-free on purpose, like `common/lifecycle.py`: decoding a topic is the
driver's job, a plain dict arrives here, and the whole chain is testable with a
fake clock on a laptop. See tests/test_control_sink.py.
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from enum import Enum

from .descriptor import SCHEMA, Descriptor, parse_descriptor


class Verdict(str, Enum):
    APPLIED = "applied"
    CLAMPED = "clamped"      # applied, but a step was limited on the way
    DROPPED = "dropped"      # stale, out of order, or outranked — routine
    REJECTED = "rejected"    # contract or limit violation — somebody must see this
    ABORTED = "aborted"      # force-torque, or escalation after repeated silence


# Verdicts that mean the command never reached `apply`.
_NOT_APPLIED = (Verdict.DROPPED, Verdict.REJECTED, Verdict.ABORTED)


@dataclass
class Outcome:
    verdict: Verdict
    reason: str = ""
    values: tuple[float, ...] | None = None
    warnings: list[str] = field(default_factory=list)

    @property
    def applied(self) -> bool:
        return self.verdict in (Verdict.APPLIED, Verdict.CLAMPED)


def _monotonic_ms() -> int:
    return int(time.monotonic() * 1000)


class ControlSink:
    """Gate between a command source and a driver's `apply`.

    Args:
        descriptor: the driver's declaration; a dict is parsed, a `Descriptor`
            is used as-is.
        apply: `(values: tuple[float, ...], gripper: float | None) -> None`.
            Called only for commands that pass every check.
        on_watchdog: called once when `watchdog_ms` elapses with no valid
            command. Defaults to a no-op meaning "hold"; a driver that can do
            better — decelerate to a stop, return to a safe pose — should pass
            its own.
        on_abort: called once when the watchdog has fired
            `escalate_after` times in a row, or when a force-torque threshold is
            crossed. Defaults to `on_watchdog`. This is what keeps a robot from
            holding an arm in the air indefinitely while the agent believes the
            action is still running.
        escalate_after: consecutive watchdog periods before escalating.
        clock: `() -> int` milliseconds, monotonic. Injected for tests.
    """

    def __init__(
        self,
        descriptor,
        apply,
        *,
        on_watchdog=None,
        on_abort=None,
        escalate_after: int = 5,
        clock=None,
    ):
        self.descriptor: Descriptor = (
            descriptor if isinstance(descriptor, Descriptor) else parse_descriptor(descriptor)
        )
        if escalate_after < 1:
            raise ValueError("escalate_after must be at least 1")

        self._apply = apply
        self._on_watchdog = on_watchdog
        self._on_abort = on_abort if on_abort is not None else on_watchdog
        self._escalate_after = escalate_after
        self._clock = clock or _monotonic_ms

        # Last command actually applied — the baseline for step clamping and
        # for the velocity check.
        self._last_values: tuple[float, ...] | None = None
        self._last_apply_ms: int | None = None
        # Highest-priority source seen recently, and the last seq per source.
        self._holder: str | None = None
        self._holder_priority: int = 0
        self._holder_seen_ms: int = 0
        self._seq_by_source: dict[str, int] = {}

        self._active = False           # has anything ever been applied
        self._holding = False          # watchdog has fired and not yet cleared
        self._aborted = False
        self._watchdog_strikes = 0
        self._last_watchdog_ms: int | None = None

        self.counters: dict[str, int] = {}

    # ── public API ───────────────────────────────────────────────────────────

    def submit(self, message: dict) -> Outcome:
        """Run one command through the chain. Calls `apply` only if it passes."""
        if self._aborted:
            return self._count(Outcome(Verdict.ABORTED, "sink is aborted; call reset()"))

        now = self._clock()

        outcome = self._check_contract(message)
        if outcome is not None:
            return self._count(outcome)

        outcome = self._check_freshness(message, now)
        if outcome is not None:
            return self._count(outcome)

        outcome = self._check_arbitration(message, now)
        if outcome is not None:
            return self._count(outcome)

        values = tuple(float(v) for v in message["values"])
        warnings: list[str] = []

        values, clamped = self._clamp_step(values)
        if clamped:
            warnings.append(
                f"step clamped on {len(clamped)} joint(s): {', '.join(clamped)}"
            )

        outcome = self._check_hard_limits(values, now)
        if outcome is not None:
            return self._count(outcome)

        gripper = message.get("gripper")
        self._apply(values, float(gripper) if gripper is not None else None)

        self._last_values = values
        self._last_apply_ms = now
        self._active = True
        self._holding = False
        self._watchdog_strikes = 0
        self._last_watchdog_ms = None

        verdict = Verdict.CLAMPED if clamped else Verdict.APPLIED
        return self._count(Outcome(verdict, values=values, warnings=warnings))

    def tick(self) -> Outcome | None:
        """Drive the watchdog. Call periodically — at least once per watchdog_ms.

        Returns an `Outcome` on the tick that fires the watchdog or escalates,
        `None` otherwise. Idle before the first command is not a watchdog
        condition: nothing has been promised yet, and firing here would mean
        every card reports a fault before it is used.
        """
        if self._aborted or not self._active:
            return None

        now = self._clock()
        since = now - (self._last_apply_ms or now)
        if since < self.descriptor.watchdog_ms:
            return None

        # One strike per elapsed watchdog period, not one per tick — otherwise
        # the escalation threshold would depend on how often the caller ticks.
        if self._last_watchdog_ms is not None and (
            now - self._last_watchdog_ms < self.descriptor.watchdog_ms
        ):
            return None

        self._last_watchdog_ms = now
        self._holding = True
        self._watchdog_strikes += 1

        if self._watchdog_strikes >= self._escalate_after:
            return self._count(self._abort(
                f"no valid command for {self._watchdog_strikes} watchdog periods "
                f"({since} ms)"
            ))

        if self._on_watchdog is not None:
            self._on_watchdog()
        return self._count(Outcome(
            Verdict.DROPPED,
            f"watchdog: no valid command for {since} ms (strike "
            f"{self._watchdog_strikes}/{self._escalate_after})",
        ))

    def force_torque(self, readings) -> Outcome | None:
        """Feed a force-torque sample. Aborts immediately if any axis is over.

        Returns an `Outcome` when it aborts, `None` otherwise. A robot whose
        descriptor declares `force_torque: null` has no thresholds, so this is a
        no-op there — deliberately visible in the descriptor rather than in the
        absence of a call.
        """
        thresholds = self.descriptor.force_torque
        if thresholds is None or self._aborted:
            return None
        for i, value in enumerate(readings):
            if i >= len(thresholds):
                break
            if abs(float(value)) > thresholds[i]:
                return self._count(self._abort(
                    f"force-torque axis {i} at {value} exceeds {thresholds[i]}"
                ))
        return None

    def reset(self) -> None:
        """Clear state for a new session. The only way out of `aborted`.

        Deliberately explicit: an abort that cleared itself on the next command
        would turn a fault into a stutter, and the next command after a
        force-torque abort is exactly the one that should not run.
        """
        self._last_values = None
        self._last_apply_ms = None
        self._holder = None
        self._holder_priority = 0
        self._holder_seen_ms = 0
        self._seq_by_source.clear()
        self._active = False
        self._holding = False
        self._aborted = False
        self._watchdog_strikes = 0
        self._last_watchdog_ms = None

    @property
    def aborted(self) -> bool:
        return self._aborted

    @property
    def holding(self) -> bool:
        return self._holding

    def stats(self) -> dict:
        return {
            "active": self._active,
            "holding": self._holding,
            "aborted": self._aborted,
            "holder": self._holder,
            "watchdog_strikes": self._watchdog_strikes,
            "counters": dict(self.counters),
        }

    # ── chain ────────────────────────────────────────────────────────────────

    def _check_contract(self, message: dict) -> Outcome | None:
        """1. Reconcile against the descriptor.

        `mode` and `dof` travel on every message on purpose, redundantly. This
        is the last line against an upstream whose action space changed without
        the driver being told — and the answer to that is to refuse, not to
        apply the first `dof` values and hope.
        """
        if not isinstance(message, dict):
            return Outcome(Verdict.REJECTED, "message is not an object")

        schema = message.get("schema")
        if schema != SCHEMA:
            return Outcome(Verdict.REJECTED, f"schema {schema!r} != {SCHEMA!r}")

        mode = message.get("mode")
        if mode != self.descriptor.mode:
            return Outcome(
                Verdict.REJECTED,
                f"mode {mode!r} but this driver accepts {self.descriptor.mode!r}",
            )

        values = message.get("values")
        if not isinstance(values, (list, tuple)):
            return Outcome(Verdict.REJECTED, "values must be a list")

        dof = message.get("dof")
        if dof != self.descriptor.dof:
            return Outcome(
                Verdict.REJECTED,
                f"dof {dof!r} but this driver has {self.descriptor.dof}",
            )
        if len(values) != self.descriptor.dof:
            return Outcome(
                Verdict.REJECTED,
                f"values has {len(values)} entries but dof is {self.descriptor.dof}",
            )
        for i, value in enumerate(values):
            if isinstance(value, bool) or not isinstance(value, (int, float)):
                return Outcome(Verdict.REJECTED, f"values[{i}] is not a number: {value!r}")

        if not isinstance(message.get("source"), str) or not message["source"]:
            return Outcome(Verdict.REJECTED, "source is required and must be a string")
        return None

    def _check_freshness(self, message: dict, now: int) -> Outcome | None:
        """2. Age and ordering. Routine failures — dropped, not rejected.

        A command can be freshly generated and still act on an 800 ms old
        picture, which is why `obs_stamp_ms` is checked separately from
        `stamp_ms` rather than inferred from it.
        """
        stamp = message.get("stamp_ms")
        ttl = message.get("ttl_ms")
        if not isinstance(stamp, (int, float)) or isinstance(stamp, bool):
            return Outcome(Verdict.REJECTED, "stamp_ms is required")
        if not isinstance(ttl, (int, float)) or isinstance(ttl, bool) or ttl <= 0:
            return Outcome(Verdict.REJECTED, "ttl_ms is required and must be positive")

        age = now - stamp
        if age > ttl:
            return Outcome(Verdict.DROPPED, f"expired: {age} ms old, ttl {int(ttl)} ms")

        max_obs_age = self.descriptor.max_obs_age_ms
        obs_stamp = message.get("obs_stamp_ms")
        if max_obs_age is not None and isinstance(obs_stamp, (int, float)):
            obs_age = now - obs_stamp
            if obs_age > max_obs_age:
                return Outcome(
                    Verdict.DROPPED,
                    f"observation {obs_age} ms old, limit {max_obs_age} ms",
                )

        seq = message.get("seq")
        if not isinstance(seq, int) or isinstance(seq, bool):
            return Outcome(Verdict.REJECTED, "seq is required and must be an integer")
        source = message["source"]
        last = self._seq_by_source.get(source)
        if last is not None and seq <= last:
            return Outcome(Verdict.DROPPED, f"seq {seq} not newer than {last}")
        return None

    def _check_arbitration(self, message: dict, now: int) -> Outcome | None:
        """3. Priority between sources.

        A higher-priority source holds the channel for as long as it keeps
        talking — `watchdog_ms` of silence and it loses the hold. That window is
        the same one the watchdog uses on purpose: the point at which a source
        is considered gone should not depend on which question is being asked.
        """
        source = message["source"]
        priority = message.get("priority", 0)
        if isinstance(priority, bool) or not isinstance(priority, (int, float)):
            return Outcome(Verdict.REJECTED, f"priority is not a number: {priority!r}")
        priority = int(priority)

        holder_stale = (
            self._holder is None
            or now - self._holder_seen_ms > self.descriptor.watchdog_ms
        )
        if not holder_stale and source != self._holder and priority < self._holder_priority:
            return Outcome(
                Verdict.DROPPED,
                f"outranked: {source} at {priority} vs {self._holder} at "
                f"{self._holder_priority}",
            )

        # Record the seq only once the message has won arbitration; an outranked
        # source must not advance its own counter, or the command it sends after
        # winning the channel back would look like a replay.
        self._seq_by_source[source] = message["seq"]
        self._holder = source
        self._holder_priority = priority
        self._holder_seen_ms = now
        return None

    def _clamp_step(self, values: tuple[float, ...]):
        """4. Limit how far one step may move. Clamp, do not reject.

        A clamped point is still going where the policy meant to go, just more
        slowly. Rejecting here would break the motion into stutters for what is
        usually one noisy sample.
        """
        limits = self.descriptor.max_delta_per_step
        if limits is None or self._last_values is None:
            return values, []

        out = list(values)
        clamped = []
        for i, (want, previous, limit) in enumerate(zip(values, self._last_values, limits)):
            delta = want - previous
            if delta > limit:
                out[i] = previous + limit
                clamped.append(self.descriptor.joint_names[i])
            elif delta < -limit:
                out[i] = previous - limit
                clamped.append(self.descriptor.joint_names[i])
        return tuple(out), clamped

    def _check_hard_limits(self, values: tuple[float, ...], now: int) -> Outcome | None:
        """5. Physical limits. Reject the whole command; never clamp to the bound.

        The opposite of step clamping, and deliberately so. Clamping to a joint
        limit produces a trajectory that is neither what the policy asked for
        nor anything anyone has validated: the policy believes the robot reached
        A, it is actually parked at bound B, and every command after that is
        built on a false premise. Refusing hands the situation to the watchdog,
        which holds — a state the policy can at least observe.
        """
        for i, value in enumerate(values):
            lo, hi = self.descriptor.lower[i], self.descriptor.upper[i]
            if value < lo or value > hi:
                return Outcome(
                    Verdict.REJECTED,
                    f"{self.descriptor.joint_names[i]} at {value} outside "
                    f"[{lo}, {hi}]",
                )

        max_velocity = self.descriptor.max_velocity
        if max_velocity is None or self._last_values is None or self._last_apply_ms is None:
            return None
        dt = (now - self._last_apply_ms) / 1000.0
        if dt <= 0:
            return None
        for i, (value, previous) in enumerate(zip(values, self._last_values)):
            speed = abs(value - previous) / dt
            if speed > max_velocity[i]:
                return Outcome(
                    Verdict.REJECTED,
                    f"{self.descriptor.joint_names[i]} would move at {speed:.3f} "
                    f"(limit {max_velocity[i]})",
                )
        return None

    # ── helpers ──────────────────────────────────────────────────────────────

    def _abort(self, reason: str) -> Outcome:
        self._aborted = True
        self._holding = True
        if self._on_abort is not None:
            self._on_abort()
        return Outcome(Verdict.ABORTED, reason)

    def _count(self, outcome: Outcome) -> Outcome:
        key = outcome.verdict.value
        self.counters[key] = self.counters.get(key, 0) + 1
        return outcome
