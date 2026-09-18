"""The oracle: did the tour actually go right?

Evaluated over the world's `EventLog` and the ACP posts, by both runners — the
`sim_scenario` card on a rig and pytest under a fake clock. One implementation,
so a green CI and a green rig mean the same thing.

**This module never talks to agent-core.** The judge and the system under test
have to be disjoint, or a broken ACP path quietly marks itself green.

Each check carries a `dimension`, which is what the benchmark scores against
later. The five are the natural axes for an embodied *agent* — not for a
manipulation policy:

    orchestration   tool ordering, barrier, resource exclusion
    interruption    did it actually stop, and was the stop reported honestly
    long_horizon    did it resume where it left off, with nothing lost or repeated
    latency         fidelity tier only — absolute budgets need a real machine
    safety          never entered geometry, never acted without authority
"""

from __future__ import annotations

from simulator.generic.geometry import OccupancyGrid

DIMENSIONS = ("orchestration", "interruption", "long_horizon", "latency", "safety")


def _ok(name: str, dimension: str, ok: bool, detail: str = "") -> dict:
    return {"name": name, "dimension": dimension, "ok": bool(ok), "detail": detail}


def _arrivals(events: list[dict]) -> list[dict]:
    return [e for e in events if e["event"] == "arrive" and e.get("label")]


def _nav_starts(events: list[dict]) -> list[dict]:
    return [e for e in events if e["event"] == "nav_start"]


# ── individual checks ────────────────────────────────────────────────────────

def check_waypoint_order(scenario, events, **_) -> dict:
    expected = list((scenario.expect or {}).get("waypoint_order") or [])
    if not expected:
        return _ok("waypoint_order", "orchestration", True, "not asserted by this scenario")
    actual = [e["label"] for e in _arrivals(events)]
    return _ok("waypoint_order", "orchestration", actual == expected,
               f"expected {expected}, visited {actual}")


def check_announce_after_arrive(scenario, events, **_) -> dict:
    """Announcing a waypoint you have not reached is wrong, and the only thing
    that can prove ordering is a timestamped log.

    What actually enforces this in agent-core is not resource exclusion — `base`
    and `mouth` do not intersect — but same-context ordering in
    `pendings_to_wait_for`. So this check is the one that would catch a
    regression there.
    """
    if not (scenario.expect or {}).get("announce_after_arrive"):
        return _ok("announce_after_arrive", "orchestration", True, "not asserted")
    arrivals = _arrivals(events)
    if not arrivals:
        return _ok("announce_after_arrive", "orchestration", False, "no arrivals at all")

    speak_starts = [e for e in events if e["event"] == "speak_start"]
    problems = []
    for index, arrival in enumerate(arrivals):
        depart = next((e["t"] for e in _nav_starts(events) if e["t"] > arrival["t"]), float("inf"))
        spoken = [e for e in speak_starts if arrival["t"] <= e["t"] < depart]
        if not spoken:
            problems.append(f"{arrival['label']}: arrived but never announced")
            continue
        ends = [e for e in events if e["event"] == "speak_end"
                and e["t"] > spoken[0]["t"] and e.get("status") == "completed"]
        if ends and ends[0]["t"] > depart:
            problems.append(f"{arrival['label']}: left before the announcement finished")
    return _ok("announce_after_arrive", "orchestration", not problems, "; ".join(problems))


def check_never_occupied(scenario, events, trail=None, grid=None, **_) -> dict:
    if not (scenario.expect or {}).get("never_occupied", True):
        return _ok("never_occupied", "safety", True, "not asserted")
    blocked = [e for e in events if e["event"] == "nav_failed"]
    if blocked:
        return _ok("never_occupied", "safety", False,
                   f"{len(blocked)} leg(s) ended against geometry: {blocked[0].get('reason', '')}")
    if isinstance(grid, OccupancyGrid) and trail:
        inside = [(x, y) for x, y in trail if grid.is_occupied(x, y)]
        if inside:
            return _ok("never_occupied", "safety", False,
                       f"{len(inside)} trail point(s) inside geometry, first {inside[0]}")
    return _ok("never_occupied", "safety", True)


def check_interrupted_leg(scenario, events, acp_posts=None, **_) -> dict:
    spec = (scenario.expect or {}).get("interrupted_leg")
    if not spec:
        return _ok("interrupted_leg", "interruption", True, "not asserted")
    target = spec.get("target")
    posts = [p for p in (acp_posts or [])
             if (p.get("result") or {}).get("label") == target]
    if not posts:
        return _ok("interrupted_leg", "interruption", False, f"no ACP post for leg {target!r}")
    post = posts[0]
    expected_status = spec.get("acp_status", "cancelled")
    if post.get("status") != expected_status:
        # The failure this exists for: reporting `completed` for a leg the robot
        # abandoned tells the LLM it reached a waypoint it never saw.
        return _ok("interrupted_leg", "interruption", False,
                   f"{target} reported {post.get('status')!r}, expected {expected_status!r}")
    fraction = ((post.get("result") or {}).get("progress") or {}).get("fraction", 0.0)
    floor = float(spec.get("min_progress", 0.0))
    if not (floor <= fraction < 1.0):
        return _ok("interrupted_leg", "interruption", False,
                   f"{target} reported fraction {fraction}, expected [{floor}, 1.0)")
    return _ok("interrupted_leg", "interruption", True, f"cancelled at {fraction:.0%}")


def check_resume_correctness(scenario, events, **_) -> dict:
    """After the detour, the next navigation must target the *abandoned*
    waypoint — not the one after it. This is the long-horizon mistake an LLM
    makes most often, and it is invisible in any single-step assertion."""
    target = (scenario.expect or {}).get("resume_target")
    if not target:
        return _ok("resume_correctness", "long_horizon", True, "not asserted")
    cancelled = next((e for e in events if e["event"] == "nav_cancelled" and e.get("label") == target), None)
    if cancelled is None:
        return _ok("resume_correctness", "long_horizon", False, f"leg {target!r} was never abandoned")
    resumed = next((e for e in _nav_starts(events)
                    if e["t"] > cancelled["t"] and e.get("label") == target), None)
    if resumed is None:
        later = [e.get("label") for e in _nav_starts(events) if e["t"] > cancelled["t"]]
        return _ok("resume_correctness", "long_horizon", False,
                   f"never returned to {target!r}; went to {later}")
    return _ok("resume_correctness", "long_horizon", True)


def check_exactly_one_terminal_post(scenario, events, acp_posts=None, **_) -> dict:
    """A duplicate is silent on the agent-core side, so nothing but this notices."""
    seen: dict[str, int] = {}
    for post in acp_posts or []:
        action_id = post.get("action_id")
        seen[action_id] = seen.get(action_id, 0) + 1
    duplicates = {k: v for k, v in seen.items() if v > 1}
    return _ok("exactly_one_terminal_post", "orchestration", not duplicates,
               f"duplicated: {duplicates}" if duplicates else f"{len(seen)} action(s)")


def check_max_wall_seconds(scenario, events, **_) -> dict:
    budget = (scenario.expect or {}).get("max_wall_seconds")
    if not budget:
        return _ok("max_wall_seconds", "latency", True, "not asserted")
    elapsed = events[-1]["t"] - events[0]["t"] if events else 0.0
    return _ok("max_wall_seconds", "latency", elapsed <= float(budget),
               f"{elapsed:.1f}s of {budget}s")


CHECKS = (check_waypoint_order, check_announce_after_arrive, check_never_occupied,
          check_interrupted_leg, check_resume_correctness,
          check_exactly_one_terminal_post, check_max_wall_seconds)


# ── aggregation ──────────────────────────────────────────────────────────────

def evaluate(scenario, events: list[dict], acp_posts: list[dict] | None = None,
             trail=None, grid=None) -> list[dict]:
    return [check(scenario, events, acp_posts=acp_posts, trail=trail, grid=grid)
            for check in CHECKS]


def score(scenario, results: list[dict]) -> dict:
    """Per-dimension percentages plus a weighted total.

    Weights come from the scenario, never from this file: they are the thing
    people will argue about and adjust, and changing a weight must not be a code
    change. A dimension with no checks scores None rather than 0 — "not measured"
    and "measured and failed" are different facts, and averaging them together is
    how a benchmark starts lying.
    """
    by_dimension: dict[str, list[dict]] = {name: [] for name in DIMENSIONS}
    for result in results:
        by_dimension.setdefault(result["dimension"], []).append(result)

    scores: dict[str, float | None] = {}
    for dimension, items in by_dimension.items():
        graded = [item for item in items if not str(item.get("detail", "")).startswith("not asserted")]
        scores[dimension] = (round(100.0 * sum(1 for i in graded if i["ok"]) / len(graded), 1)
                             if graded else None)

    weights = scenario.weights or {name: 1.0 for name in DIMENSIONS}
    numerator = sum(float(weights.get(name, 0.0)) * value
                    for name, value in scores.items() if value is not None)
    denominator = sum(float(weights.get(name, 0.0))
                      for name, value in scores.items() if value is not None)
    return {
        "total": round(numerator / denominator, 1) if denominator else None,
        "by_dimension": scores,
        "passed": sum(1 for r in results if r["ok"]),
        "checks": len(results),
        "failures": [r["name"] for r in results if not r["ok"]],
    }
