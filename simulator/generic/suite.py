"""Run a set of scenarios back to back and score each one.

## Why the runner lives here and not in the browser

A suite is scenarios x repeats, and a single exhibition tour is minutes of wall
clock. Driven from a web page, closing the tab kills the run. So `run_suite` is
an action on the card: it starts, returns immediately, and progress is polled
through `sim_report` — which is barrier-exempt precisely because it is a
`resource`, so polling works *while* a navigation is pending.

CI drives the same `run_suite`; only the entry point differs (CLI, exit code).
Two entry points, one execution path.

## Why it is a step machine rather than a thread

It advances on the world's step listener, so it behaves identically under a fake
clock in pytest and a real clock on a rig, and there is no second thread to
reason about against the world's lock.

## What actually drives the robot

Nothing here does. The runner loads a scenario and injects its opening prompt on
`/remote_control/message` — the same path a real remote control uses — and then
watches. The LLM on the machine is what decides to navigate and announce. That
separation is the point: the suite measures the agent, so it must not be the
agent.

## Repeats are not optional

An LLM is stochastic, so a single run's score is one sample of a distribution.
`repeats` defaults to 1 for a quick look, but the summary always reports `n`
alongside mean and standard deviation — a score shown without its `n` reads as a
conclusion when it is a coin flip.
"""

from __future__ import annotations

import statistics
from dataclasses import dataclass, field

from simulator.generic import assertions
from simulator.generic.world import STATE_DONE

DEFAULT_STALL_SECONDS = 90.0
# Quiet time after the last waypoint before a case is called finished.
DEFAULT_SETTLE_SECONDS = 5.0


@dataclass
class Case:
    scenario: str
    repeat: int
    seed: int
    started_at: float | None = None
    ended_at: float | None = None
    outcome: str = "pending"          # pending | ok | timeout | stalled | error
    score: dict = field(default_factory=dict)
    checks: list = field(default_factory=list)
    detail: str = ""

    def as_dict(self) -> dict:
        return {"scenario": self.scenario, "repeat": self.repeat, "seed": self.seed,
                "started_at": self.started_at, "ended_at": self.ended_at,
                "elapsed": (round(self.ended_at - self.started_at, 2)
                            if self.started_at is not None and self.ended_at is not None else None),
                "outcome": self.outcome, "score": self.score,
                "failures": [c["name"] for c in self.checks if not c["ok"]],
                "detail": self.detail}


class SuiteRunner:
    """One suite. Advances on `world.step`; owns no thread of its own."""

    def __init__(self, card, world):
        self._card = card
        self._world = world
        self._cases: list[Case] = []
        self._index = 0
        self._state = "idle"           # idle | loading | running | done
        self._case_started = 0.0
        self._last_progress = 0.0
        self._last_event_count = 0
        self._stall_seconds = DEFAULT_STALL_SECONDS
        self._settle_seconds = DEFAULT_SETTLE_SECONDS
        world.add_step_listener(self._on_step)

    # ---- control ------------------------------------------------------

    def start(self, scenarios: list[str], repeats: int = 1, seed: int = 0,
              stall_seconds: float = DEFAULT_STALL_SECONDS,
              settle_seconds: float = DEFAULT_SETTLE_SECONDS) -> dict:
        self._cases = [Case(scenario=slug, repeat=index, seed=seed + index)
                       for slug in scenarios for index in range(max(1, int(repeats)))]
        self._index = 0
        self._stall_seconds = float(stall_seconds)
        self._settle_seconds = float(settle_seconds)
        self._state = "loading" if self._cases else "done"
        self._world.log("suite_start", scenarios=scenarios, repeats=repeats, cases=len(self._cases))
        return self.status()

    def abort(self) -> dict:
        if self._state != "done":
            self._state = "done"
            self._world.log("suite_abort", completed=self._completed())
        return self.status()

    @property
    def running(self) -> bool:
        return self._state != "idle" and self._state != "done"

    # ---- the machine --------------------------------------------------

    def _on_step(self, t: float, _dt: float) -> None:
        if self._state == "loading":
            self._begin_case(t)
        elif self._state == "running":
            self._watch_case(t)

    def _begin_case(self, t: float) -> None:
        case = self._cases[self._index]
        result = self._card.do_load(scenario=case.scenario)
        if "error" in result:
            case.outcome = "error"
            case.detail = result["error"]
            self._advance(t)
            return
        self._card.do_run()
        scenario = self._card.active
        case.started_at = t
        self._case_started = t
        self._last_progress = t
        self._last_event_count = len(self._world.events())
        self._state = "running"
        prompt = (scenario.raw.get("prompt") or "").strip()
        if prompt:
            # The opening instruction. The LLM on this machine is what decides to
            # navigate and announce — the suite measures the agent, so it must
            # not be the agent.
            self._card.do_inject(text=prompt, kind="user_message")

    def _watch_case(self, t: float) -> None:
        case = self._cases[self._index]
        scenario = self._card.active
        events = self._world.events()
        if len(events) != self._last_event_count:
            self._last_event_count = len(events)
            self._last_progress = t

        expected = list((scenario.expect or {}).get("waypoint_order") or [])
        visited = [e["label"] for e in events if e["event"] == "arrive" and e.get("label")]
        budget = float((scenario.expect or {}).get("max_wall_seconds") or 0.0)
        elapsed = t - self._case_started

        if expected and visited[:len(expected)] == expected and self._settled(t):
            self._finish_case(t, "ok")
        elif budget and elapsed > budget:
            self._finish_case(t, "timeout", f"exceeded {budget}s")
        elif t - self._last_progress > self._stall_seconds:
            # Silence is not success. A run that stops producing events has to be
            # recorded as stalled, or a crashed agent scores the same as one that
            # simply has not finished yet.
            self._finish_case(t, "stalled",
                              f"no event for {self._stall_seconds}s at {visited}")

    def _settled(self, t: float) -> bool:
        """Reaching the last waypoint is not finishing the task.

        Two separate mistakes were possible here and the first one was made:
        closing the case on arrival cut off the final announcement, so every run
        failed `announce_after_arrive` on its last stop — a scoring artefact
        indistinguishable from a real orchestration fault.

        Being idle *at that instant* is not enough either, because deciding to
        speak takes the agent a round trip. So a case ends only once the robot has
        stopped moving, stopped talking, and stayed quiet for `settle_seconds`.
        """
        snapshot = self._world.snapshot()
        job = snapshot.get("job")
        if snapshot.get("speech") is not None:
            return False
        if job is not None and job["state"] != STATE_DONE:
            return False
        return (t - self._last_progress) >= self._settle_seconds

    def _finish_case(self, t: float, outcome: str, detail: str = "") -> None:
        case = self._cases[self._index]
        scenario = self._card.active
        grid = self._world._backend.state()["grid"]  # noqa: SLF001
        case.checks = assertions.evaluate(scenario, self._world.events(),
                                          acp_posts=self._card.acp_posts,
                                          trail=self._world.trail(), grid=grid)
        case.score = assertions.score(scenario, case.checks)
        case.outcome = outcome
        case.detail = detail
        case.ended_at = t
        self._world.log("suite_case", **case.as_dict())
        self._card.do_abort()
        self._advance(t)

    def _advance(self, t: float) -> None:
        self._index += 1
        if self._index >= len(self._cases):
            self._state = "done"
            self._world.log("suite_done", **self.summary())
        else:
            self._state = "loading"

    # ---- reporting ----------------------------------------------------

    def _completed(self) -> int:
        return sum(1 for case in self._cases if case.outcome != "pending")

    def status(self) -> dict:
        return {"state": self._state, "case": self._index, "cases": len(self._cases),
                "completed": self._completed(),
                "current": self._cases[self._index].scenario
                if self._state != "done" and self._index < len(self._cases) else None}

    def summary(self) -> dict:
        """Mean, standard deviation and **n**, per scenario.

        `n` is reported even when it is 1. An LLM is stochastic; a single score
        shown without its sample size reads as a conclusion when it is a coin
        flip, and that is the line between a benchmark and a demo.
        """
        by_scenario: dict[str, list[Case]] = {}
        for case in self._cases:
            by_scenario.setdefault(case.scenario, []).append(case)

        scenarios = {}
        for slug, cases in by_scenario.items():
            scored = [c for c in cases if c.score.get("total") is not None]
            totals = [c.score["total"] for c in scored]
            scenarios[slug] = {
                "n": len(cases),
                "scored": len(scored),
                "mean": round(statistics.fmean(totals), 1) if totals else None,
                "stdev": round(statistics.stdev(totals), 1) if len(totals) > 1 else None,
                "pass_rate": (round(100.0 * sum(1 for c in cases if c.outcome == "ok") / len(cases), 1)
                              if cases else None),
                "outcomes": _tally(c.outcome for c in cases),
                "by_dimension": _mean_dimensions(scored),
            }

        all_scored = [c for c in self._cases if c.score.get("total") is not None]
        totals = [c.score["total"] for c in all_scored]
        return {
            "state": self._state,
            "n": len(self._cases),
            "scored": len(all_scored),
            "mean": round(statistics.fmean(totals), 1) if totals else None,
            "stdev": round(statistics.stdev(totals), 1) if len(totals) > 1 else None,
            "outcomes": _tally(c.outcome for c in self._cases),
            "scenarios": scenarios,
            "cases": [case.as_dict() for case in self._cases],
        }


def _tally(values) -> dict:
    counts: dict[str, int] = {}
    for value in values:
        counts[value] = counts.get(value, 0) + 1
    return counts


def _mean_dimensions(cases: list[Case]) -> dict:
    """None where nothing measured that dimension — never 0.

    "not measured" and "measured and failed" are different facts; averaging them
    together is how a benchmark starts lying.
    """
    collected: dict[str, list[float]] = {}
    for case in cases:
        for name, value in (case.score.get("by_dimension") or {}).items():
            if value is not None:
                collected.setdefault(name, []).append(value)
    return {name: round(statistics.fmean(values), 1) for name, values in collected.items()}
