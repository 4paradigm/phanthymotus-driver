"""The simulated world: navigation jobs, speech, and the event log.

Everything the cards need to agree about lives here, behind one lock. The
backend owns integration and collision; this file owns *what the robot was asked
to do* and *what actually happened* — which is the thing the exhibition-tour
assertions read.

Concurrency follows `perception/README.md` § "Plugin Concurrency" to the letter,
because `dispatch()` runs on `ThreadingHTTPServer` threads and the tick thread
mutates the same state:

1. One ``RLock`` guards every read-modify-write of pose / job / speech / events.
   Reentrant because a nav command legitimately cancels before it submits.
2. The lock is **never** held across a sleep or a callback. The tick loop sleeps
   outside it, and terminal callbacks (which POST over HTTP) fire after release.
3. ``submit_job`` registers the job **before** it starts running, so a concurrent
   cancel can find it. Backwards, you get an orphan job that nothing can stop and
   that keeps POSTing ``completed`` for a navigation the user abandoned.
4. ``cancel_job`` sets ``cancel_event`` **before** taking the lock, so a cancel
   arriving mid-tick is seen at the top of the next step regardless of ordering.
5. Exactly one terminal callback per ``action_id``, enforced by a compare-and-set
   on ``terminal_posted`` under the lock. Both arrival and preemption can reach
   the terminal transition, and a double POST is *silent* on the agent-core side
   (``mark_action_complete`` just overwrites) — it does not fail, it only makes
   the transcript wrong.
"""

from __future__ import annotations

import math
import threading
import uuid
from dataclasses import dataclass, field

from simulator.generic.backend import WorldBackend
from simulator.generic.clock import Clock, RealClock
from simulator.generic.geometry import Pose, normalize_angle

# Job state / result codes mirror the Slamtec vocabulary the real navigation
# cards report, so an assertion written against the simulator keeps its meaning
# when pointed at `x-humanoid/tianyi2.0`.
STATE_NEW = 0
STATE_RUNNING = 1
STATE_PAUSED = 3
STATE_DONE = 4

RESULT_OK = 0
RESULT_CANCELLED = -1
RESULT_FAILED = -2

_STATUS_BY_RESULT = {RESULT_OK: "completed", RESULT_CANCELLED: "cancelled", RESULT_FAILED: "failed"}

POS_TOL = 0.12          # m
YAW_TOL = 0.06          # rad, final heading
STEER_TOL = 0.30        # rad, above this turn in place instead of driving


@dataclass
class NavJob:
    id: str
    kind: str                       # navigate_to | move_to | rotate | rotate_to
    target: Pose
    label: str = ""
    state: int = STATE_NEW
    result: int = RESULT_OK
    reason: str = ""
    started_at: float = 0.0
    ended_at: float | None = None
    dist_total: float = 0.0
    dist_done: float = 0.0
    phase: str = "translate"        # translate | final_yaw
    terminal_posted: bool = False
    cancel_event: threading.Event = field(default_factory=threading.Event)

    @property
    def fraction(self) -> float:
        if self.dist_total <= 1e-6:
            return 1.0 if self.state == STATE_DONE else 0.0
        return max(0.0, min(1.0, self.dist_done / self.dist_total))

    @property
    def status(self) -> str:
        """`running` 直到抵达终态。

        原本只看 `result`，而 `result` 在任务**运行中**就是 RESULT_OK（0）—— 于是
        一个刚走到一半的任务对外报 `completed`。Orin6 上看到的就是
        `job: 三号展区 completed fraction=0.483`：两个字段互相矛盾，而读的人会信
        `status`。ACP 回调只在终态发出，所以线上没有报错这个假状态，但 `nav.read`
        会把它端给 LLM。"""
        if self.state != STATE_DONE:
            return "running"
        return _STATUS_BY_RESULT.get(self.result, "failed")

    def as_dict(self) -> dict:
        return {
            "action_id": self.id, "kind": self.kind, "label": self.label,
            "target": self.target.as_dict(), "state": self.state, "result": self.result,
            "status": self.status, "reason": self.reason,
            "progress": {"dist_done": round(self.dist_done, 3),
                         "dist_total": round(self.dist_total, 3),
                         "fraction": round(self.fraction, 3)},
        }


@dataclass
class Utterance:
    id: str
    text: str
    duration: float
    started_at: float | None = None
    ended_at: float | None = None
    outcome: str = "pending"        # pending | completed | interrupted
    terminal_posted: bool = False

    def as_dict(self) -> dict:
        return {"action_id": self.id, "text": self.text, "duration": round(self.duration, 3),
                "started_at": self.started_at, "ended_at": self.ended_at,
                "status": "completed" if self.outcome == "completed" else
                          "cancelled" if self.outcome == "interrupted" else "running"}


class VirtualWorld:
    """Owns the mutable world. Cards read `snapshot()`; only `step()` advances it."""

    def __init__(self, backend: WorldBackend, clock: Clock | None = None, config: dict | None = None):
        cfg = config or {}
        self._backend = backend
        self._clock = clock or RealClock()
        self._lock = threading.RLock()

        self._tick_hz = float(cfg.get("tick_hz", 20.0))
        self._chars_per_sec = float(cfg.get("chars_per_sec", 5.0))
        self._min_speech = float(cfg.get("min_speech_seconds", 0.5))

        self._job: NavJob | None = None
        self._speech: Utterance | None = None
        self._speech_queue: list[Utterance] = []
        self._events: list[dict] = []
        self._manual = (0.0, 0.0)
        # Tags (Slamtec's word for a named place) live here rather than on the
        # navigator: the map card draws them and the navigator drives to them,
        # and a scenario `tag_place`s new ones at runtime. One owner, two readers.
        self._tags: dict[str, dict] = {}
        self._trail: list[tuple[float, float]] = []
        self._trail_step = float(cfg.get("trail_step_m", 0.15))
        self._trail_max = int(cfg.get("trail_max_points", 4000))

        # Lists, not single slots. Several cards submit jobs (`nav`,
        # `switch_mode`) and several drive the mouth (`tts`, `speaker`); with one
        # slot the last registrant would receive — and mis-attribute — everyone
        # else's completions. Each listener filters on the action_ids it owns.
        self._nav_listeners: list = []
        self._speech_listeners: list = []
        # Called once per step with (t, dt). The scenario card uses it to fire
        # scripted injections on *simulated* time, so a tour replays identically
        # under a fake clock and on a rig.
        self._step_listeners: list = []

        self._thread: threading.Thread | None = None
        self._stopping = threading.Event()

    # ---- wiring -------------------------------------------------------

    def add_nav_listener(self, fn) -> None:
        """Called once per job with its terminal dict. Invoked outside the lock."""
        self._nav_listeners.append(fn)

    def add_speech_listener(self, fn) -> None:
        self._speech_listeners.append(fn)

    def add_step_listener(self, fn) -> None:
        """``fn(t, dt)`` after each integration step, outside the lock."""
        self._step_listeners.append(fn)

    def remove_step_listener(self, fn) -> bool:
        """Cards live for the process, so they never need this — but anything
        transient does. Without it a listener registered per run accumulates, and
        several of them drive the same robot at once: the first run looks fine
        and every later one degrades, which reads as flakiness rather than as a
        leak."""
        try:
            self._step_listeners.remove(fn)
            return True
        except ValueError:
            return False

    def _emit(self, listeners: list, payload: dict) -> None:
        for listener in listeners:
            try:
                listener(payload)
            except Exception as exc:  # one bad listener must not swallow the rest
                print(f"[sim-world] listener failed: {exc}", flush=True)

    # ---- lifecycle ----------------------------------------------------

    def start(self) -> None:
        if self._thread is not None:
            return
        self._stopping.clear()
        self._thread = threading.Thread(target=self._tick_loop, daemon=True, name="sim-world")
        self._thread.start()

    def stop(self) -> None:
        self._stopping.set()
        thread, self._thread = self._thread, None
        if thread is not None:
            thread.join(timeout=2.0)

    def _tick_loop(self) -> None:
        dt = 1.0 / self._tick_hz
        while not self._stopping.is_set():
            began = self._clock.now()
            try:
                self.step(dt)
            except Exception as exc:  # a tick must never kill the world
                print(f"[sim-world] tick failed: {exc}", flush=True)
            # Sleep outside the lock — rule 2. A cancel must never queue behind it.
            self._clock.sleep(max(0.0, dt - (self._clock.now() - began)))

    # ---- the single place time moves ----------------------------------

    def step(self, dt: float) -> None:
        """Advance the world by ``dt``. Safe to call directly with a FakeClock."""
        nav_done: dict | None = None
        speech_done: list[dict] = []

        with self._lock:
            job = self._job
            if job is not None and job.cancel_event.is_set() and job.state != STATE_DONE:
                nav_done = self._finish_job_locked(job, RESULT_CANCELLED, job.reason or "cancelled")
            else:
                self._backend.apply(self._drive_command_locked())
                self._backend.step(dt)
                self._record_trail_locked()
                nav_done = self._update_job_locked()
            speech_done = self._update_speech_locked()

        # Callbacks POST over HTTP. They run with no lock held — rule 2.
        for listener in self._step_listeners:
            try:
                listener(self._clock.now(), dt)
            except Exception as exc:
                print(f"[sim-world] step listener failed: {exc}", flush=True)
        if nav_done is not None:
            self._emit(self._nav_listeners, nav_done)
        for payload in speech_done:
            self._emit(self._speech_listeners, payload)

    # ---- navigation ---------------------------------------------------

    def _drive_command_locked(self) -> dict:
        job = self._job
        if job is None or job.state != STATE_RUNNING:
            lin, ang = self._manual
            return {"lin": lin, "ang": ang}

        state = self._backend.state()
        pose: Pose = state["pose"]
        max_lin = getattr(self._backend, "_max_lin", 0.5)
        max_ang = getattr(self._backend, "_max_ang", 0.6)
        accel = getattr(self._backend, "_accel", 0.4)

        if job.kind in ("rotate", "rotate_to") or job.phase == "final_yaw":
            err = normalize_angle(job.target.yaw - pose.yaw)
            return {"lin": 0.0, "ang": _sign(err) * min(max_ang, abs(err) * 2.0)}

        dist = pose.distance_to(job.target)
        err = normalize_angle(pose.bearing_to(job.target) - pose.yaw)
        if abs(err) > STEER_TOL:
            # Turn in place first; driving while badly misaligned makes the path
            # a spiral and the distance assertions meaningless.
            return {"lin": 0.0, "ang": _sign(err) * min(max_ang, abs(err) * 2.0)}
        # Trapezoid: never enter a state the remaining distance cannot brake from.
        approach = math.sqrt(max(0.0, 2.0 * accel * max(0.0, dist - POS_TOL * 0.5)))
        return {"lin": min(max_lin, approach), "ang": _sign(err) * min(max_ang, abs(err) * 2.0)}

    def _update_job_locked(self) -> dict | None:
        job = self._job
        if job is None or job.state != STATE_RUNNING:
            return None
        state = self._backend.state()
        pose: Pose = state["pose"]

        if state.get("contact"):
            return self._finish_job_locked(job, RESULT_FAILED, f"blocked by geometry at {state['contact']}")

        job.dist_done = max(job.dist_done, job.dist_total - pose.distance_to(job.target))

        if job.kind in ("rotate", "rotate_to"):
            if abs(normalize_angle(job.target.yaw - pose.yaw)) <= YAW_TOL:
                return self._finish_job_locked(job, RESULT_OK, "")
            return None

        if job.phase == "translate":
            if pose.distance_to(job.target) <= POS_TOL:
                job.dist_done = job.dist_total
                job.phase = "final_yaw"
            return None

        if abs(normalize_angle(job.target.yaw - pose.yaw)) <= YAW_TOL:
            return self._finish_job_locked(job, RESULT_OK, "")
        return None

    def _finish_job_locked(self, job: NavJob, result: int, reason: str) -> dict | None:
        """Compare-and-set the terminal transition. Returns the payload only to the winner."""
        if job.terminal_posted:
            return None
        job.terminal_posted = True
        job.state = STATE_DONE
        job.result = result
        job.reason = reason
        job.ended_at = self._clock.now()
        self._backend.apply({"lin": 0.0, "ang": 0.0})
        self._manual = (0.0, 0.0)
        payload = job.as_dict()
        payload["elapsed"] = round(job.ended_at - job.started_at, 3)
        payload["pose"] = self._backend.state()["pose"].as_dict()
        event = {RESULT_OK: "arrive", RESULT_CANCELLED: "nav_cancelled"}.get(result, "nav_failed")
        self._log_locked(event, **payload)
        return payload

    def submit_job(self, kind: str, target: Pose, label: str = "") -> NavJob:
        """Replace any running job. Registers before running — rule 3."""
        previous = self._job
        if previous is not None:
            previous.cancel_event.set()          # rule 4: signal before the lock
        superseded = None
        with self._lock:
            if previous is not None and not previous.terminal_posted:
                superseded = self._finish_job_locked(previous, RESULT_CANCELLED, "superseded by a new command")
            pose: Pose = self._backend.state()["pose"]
            job = NavJob(
                id=f"sim-nav-{uuid.uuid4().hex[:12]}",
                kind=kind, target=target.copy(), label=label,
                started_at=self._clock.now(),
                dist_total=0.0 if kind in ("rotate", "rotate_to") else pose.distance_to(target),
            )
            self._job = job                       # registered...
            job.state = STATE_RUNNING             # ...and only then running
            self._log_locked("nav_start", **job.as_dict())
        if superseded is not None:
            self._emit(self._nav_listeners, superseded)
        return job

    def cancel_job(self, reason: str = "cancelled") -> dict | None:
        """Rule 4: signal cancellation before taking any lock."""
        job = self._job
        if job is None:
            return None
        job.reason = reason
        job.cancel_event.set()
        with self._lock:
            payload = self._finish_job_locked(job, RESULT_CANCELLED, reason)
        if payload is not None:
            self._emit(self._nav_listeners, payload)
        return payload

    def set_velocity(self, lin: float, ang: float) -> None:
        """Direct drive (`loco`). Preempts navigation — one base, one owner."""
        if self._job is not None and not self._job.terminal_posted:
            self.cancel_job("preempted by direct velocity command")
        with self._lock:
            self._manual = (float(lin), float(ang))
            self._log_locked("velocity", lin=float(lin), ang=float(ang))

    # ---- speech -------------------------------------------------------

    def speak(self, text: str) -> Utterance:
        duration = max(self._min_speech, len(text or "") / max(0.1, self._chars_per_sec))
        utterance = Utterance(id=f"sim-tts-{uuid.uuid4().hex[:12]}", text=text or "", duration=duration)
        with self._lock:
            if self._speech is None:
                self._begin_speech_locked(utterance)
            else:
                self._speech_queue.append(utterance)
        return utterance

    def _begin_speech_locked(self, utterance: Utterance) -> None:
        utterance.started_at = self._clock.now()
        self._speech = utterance
        self._log_locked("speak_start", action_id=utterance.id, text=utterance.text,
                         duration=round(utterance.duration, 3))

    def _update_speech_locked(self) -> list[dict]:
        done: list[dict] = []
        current = self._speech
        if current is None or current.started_at is None:
            return done
        if self._clock.now() - current.started_at < current.duration:
            return done
        payload = self._finish_speech_locked(current, "completed")
        if payload is not None:
            done.append(payload)
        if self._speech_queue:
            self._begin_speech_locked(self._speech_queue.pop(0))
        return done

    def _finish_speech_locked(self, utterance: Utterance, outcome: str) -> dict | None:
        if utterance.terminal_posted:
            return None
        utterance.terminal_posted = True
        utterance.outcome = outcome
        utterance.ended_at = self._clock.now()
        if self._speech is utterance:
            self._speech = None
        payload = utterance.as_dict()
        self._log_locked("speak_end", outcome=outcome, **payload)
        return payload

    def interrupt_speech(self, reason: str = "interrupted") -> list[dict]:
        done: list[dict] = []
        with self._lock:
            pending, self._speech_queue = self._speech_queue, []
            if self._speech is not None:
                payload = self._finish_speech_locked(self._speech, "interrupted")
                if payload is not None:
                    done.append(payload)
            for queued in pending:
                payload = self._finish_speech_locked(queued, "interrupted")
                if payload is not None:
                    done.append(payload)
            self._log_locked("speech_interrupt", reason=reason, count=len(done))
        for payload in done:
            self._emit(self._speech_listeners, payload)
        return done

    # ---- events -------------------------------------------------------

    def log(self, event: str, **payload) -> dict:
        with self._lock:
            return self._log_locked(event, **payload)

    def _log_locked(self, event: str, **payload) -> dict:
        # The field is `event`, not `kind`: a job already has a `kind` (its
        # motion type), and one dict cannot carry both meanings under one name.
        record = {"t": round(self._clock.now(), 4), "event": event, **payload}
        self._events.append(record)
        return record

    def _record_trail_locked(self) -> None:
        """Where the robot has actually been — the map card draws it, and
        'did it ever enter an occupied cell' is answered from it."""
        pose: Pose = self._backend.state()["pose"]
        if self._trail and math.hypot(pose.x - self._trail[-1][0], pose.y - self._trail[-1][1]) < self._trail_step:
            return
        self._trail.append((round(pose.x, 3), round(pose.y, 3)))
        if len(self._trail) > self._trail_max:
            del self._trail[:len(self._trail) - self._trail_max]

    # ---- tags ---------------------------------------------------------

    def set_tags(self, tags: list[dict]) -> None:
        with self._lock:
            self._tags = {t["name"]: dict(t) for t in tags if t.get("name")}

    def tag_place(self, name: str, description: str = "") -> dict:
        """Name the robot's current pose, exactly as the real chassis does."""
        with self._lock:
            pose = self._backend.state()["pose"]
            tag = {"name": name, "x": round(pose.x, 3), "y": round(pose.y, 3),
                   "yaw": round(pose.yaw, 3), "description": description}
            self._tags[name] = tag
            self._log_locked("tag_place", **tag)
            return dict(tag)

    def untag_place(self, name: str) -> bool:
        with self._lock:
            removed = self._tags.pop(name, None) is not None
            if removed:
                self._log_locked("untag_place", name=name)
            return removed

    def tags(self) -> list[dict]:
        with self._lock:
            return [dict(t) for t in self._tags.values()]

    def tag(self, name: str) -> dict | None:
        with self._lock:
            found = self._tags.get(name)
            return dict(found) if found else None

    def trail(self) -> list[tuple[float, float]]:
        with self._lock:
            return list(self._trail)

    def events(self, since: float | None = None) -> list[dict]:
        with self._lock:
            if since is None:
                return [dict(record) for record in self._events]
            return [dict(record) for record in self._events if record["t"] >= since]

    # ---- read model ---------------------------------------------------

    def snapshot(self) -> dict:
        """Plain dict, deep enough that callers cannot reach back into the world."""
        with self._lock:
            state = self._backend.state()
            return {
                "t": round(self._clock.now(), 4),
                "pose": state["pose"].as_dict(),
                "joints": list(state["joints"]),
                "lin": round(state["lin"], 4),
                "ang": round(state["ang"], 4),
                "odometer": round(state["odometer"], 3),
                "contact": state.get("contact"),
                "job": self._job.as_dict() if self._job else None,
                "speech": self._speech.as_dict() if self._speech else None,
                "speech_queued": len(self._speech_queue),
                "event_count": len(self._events),
            }

    def reset(self, scene: dict) -> None:
        with self._lock:
            self._backend.reset(scene)
            self._job = None
            self._speech = None
            self._speech_queue = []
            self._events = []
            self._manual = (0.0, 0.0)
            self._trail = []
            self._tags = {}


def _sign(value: float) -> float:
    return 0.0 if value == 0 else (1.0 if value > 0 else -1.0)
