"""Batch runs and scoring.

The distinction this file exists to enforce: a benchmark reports **n**, a demo
reports a number. An LLM is stochastic, so a single run's score is one sample of
a distribution — and a score shown without its sample size reads as a conclusion
when it is a coin flip.

The other one: silence is not success. A run that stops producing events has to
be recorded as stalled, or a crashed agent scores the same as one that simply
has not finished yet.

Run: PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest tests/test_sim_suite.py -q
"""

from __future__ import annotations

import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from simulator.generic import acp  # noqa: E402
from simulator.generic.backend import LocalBackend  # noqa: E402
from simulator.generic.cards_audio import TtsCard  # noqa: E402
from simulator.generic.cards_motion import ControlledSpatialCard  # noqa: E402
from simulator.generic.cards_scenario import SimReportCard, SimScenarioCard  # noqa: E402
from simulator.generic.clock import FakeClock  # noqa: E402
from simulator.generic.scenario import Scenario  # noqa: E402
from simulator.generic.world import VirtualWorld  # noqa: E402

sys.path.insert(0, str(ROOT / "tests"))
from test_sim_exhibition import ScriptedGuide  # noqa: E402

SCENARIO_DIR = ROOT / "simulator" / "generic" / "scenarios"
TOUR = SCENARIO_DIR / "exhibition_tour.yaml"
DT = 0.05
PLAN = ("入口", "一号展区", "二号展区", "三号展区")


class SuiteGuide:
    """A guide that restarts for each case.

    On a rig the LLM plays this part, woken by the prompt the suite injects. Here
    it watches for that same injection and starts a fresh tour, which is how one
    stand-in covers a multi-case run.
    """

    def __init__(self, nav, tts, world, **kwargs):
        self.nav, self.tts, self.world, self.kwargs = nav, tts, world, kwargs
        self.inner = None
        # Keyed on the prompt's timestamp, not on a count: `do_load` resets the
        # world and clears the event log, so a counter drops back to 1 at each
        # new case and the second one never starts.
        self._last_prompt = None
        world.add_step_listener(self._on_step)

    def _on_step(self, t, dt):
        prompts = [e for e in self.world.events()
                   if e["event"] == "injection" and "参观" in e.get("text", "")]
        if prompts and prompts[-1]["t"] != self._last_prompt:
            self._last_prompt = prompts[-1]["t"]
            self.inner = ScriptedGuide(self.nav, self.tts, Scenario.load(TOUR), PLAN,
                                       register=False, **self.kwargs)
            self.inner.wake_at = t
            return
        if self.inner is not None:
            self.inner.on_step(t, dt)


def build(guide=True, **guide_kwargs):
    acp.clear_observers()
    acp.set_transport(lambda url, payload: None)
    clock = FakeClock()
    world = VirtualWorld(LocalBackend(), clock, {"tick_hz": 1.0 / DT, "chars_per_sec": 5.0})
    config = {"embodiment": {"kind": "wheeled", "dof": 2, "joint_names": ["a", "b"]}}

    scenario = SimScenarioCard(world, config, "sim", scenario_dirs=[SCENARIO_DIR])
    nav = ControlledSpatialCard(world, config, "sim")
    tts = TtsCard(world, config, "sim")
    report = SimReportCard(world, config, "sim", scenario_card=scenario)
    if guide:
        SuiteGuide(nav, tts, world, **guide_kwargs)
    return {"world": world, "clock": clock, "scenario": scenario, "report": report, "nav": nav}


def run(rig, seconds, dt=DT):
    for _ in range(int(round(seconds / dt))):
        rig["clock"].advance(dt)
        rig["world"].step(dt)


@pytest.fixture
def rig():
    built = build()
    yield built
    acp.set_transport(None)
    acp.clear_observers()


# ── running a suite ──────────────────────────────────────────────────────────

def test_run_suite_returns_immediately_and_is_polled(rig):
    """Driven from a web page a suite would die with the tab, so it runs on the
    card and progress is polled through `sim_report` — which works mid-run
    because that card is a `resource` and the barrier exempts those."""
    result = rig["scenario"].dispatch("run_suite", {"scenarios": ["exhibition_tour"], "repeats": 2})

    assert result["state"] == "running"
    assert result["suite"]["cases"] == 2
    assert rig["report"].get_tool()["type"] == "resource"

    run(rig, 20.0)
    progress = rig["report"].dispatch("sim_report", {"what": "suite"})
    assert progress["state"] == "running"


def test_a_two_repeat_suite_completes_both_and_scores_each(rig):
    rig["scenario"].dispatch("run_suite", {"scenarios": ["exhibition_tour"], "repeats": 2})
    run(rig, 900.0)

    summary = rig["report"].dispatch("sim_report", {"what": "suite"})

    assert summary["state"] == "done"
    assert summary["outcomes"] == {"ok": 2}
    assert [case["score"]["total"] for case in summary["cases"]] == [100.0, 100.0]


def test_summary_always_reports_n_even_when_it_is_one(rig):
    """The line between a benchmark and a demo."""
    rig["scenario"].dispatch("run_suite", {"scenarios": ["exhibition_tour"], "repeats": 1})
    run(rig, 500.0)

    summary = rig["report"].dispatch("sim_report", {"what": "suite"})

    assert summary["n"] == 1
    assert summary["scenarios"]["exhibition_tour"]["n"] == 1
    assert summary["stdev"] is None, "one sample has no spread, and must not pretend to"


def test_repeats_produce_a_spread_not_a_single_number(rig):
    rig["scenario"].dispatch("run_suite", {"scenarios": ["exhibition_tour"], "repeats": 3})
    run(rig, 1400.0)

    summary = rig["report"].dispatch("sim_report", {"what": "suite"})

    assert summary["n"] == 3
    assert summary["mean"] is not None
    assert summary["stdev"] is not None
    assert summary["scenarios"]["exhibition_tour"]["pass_rate"] == 100.0


def test_seeds_advance_per_repeat(rig):
    rig["scenario"].dispatch("run_suite", {"scenarios": ["exhibition_tour"],
                                           "repeats": 3, "seed": 100})
    run(rig, 1400.0)

    seeds = [case["seed"] for case in
             rig["report"].dispatch("sim_report", {"what": "suite"})["cases"]]

    assert seeds == [100, 101, 102]


def test_the_suite_injects_the_scenario_prompt_to_wake_the_agent(rig):
    """The runner never drives the robot itself — it measures the agent, so it
    must not be the agent."""
    rig["scenario"].dispatch("run_suite", {"scenarios": ["exhibition_tour"]})
    run(rig, 5.0)

    prompts = [e for e in rig["world"].events()
               if e["event"] == "injection" and e.get("injection_kind") == "user_message"]

    assert prompts and "参观" in prompts[0]["text"]


def test_an_empty_scenario_list_runs_everything_discovered(rig):
    result = rig["scenario"].dispatch("run_suite", {})

    assert result["scenarios"] == ["exhibition_tour"]


def test_an_unknown_scenario_is_reported_not_silently_dropped(rig):
    result = rig["scenario"].dispatch("run_suite",
                                      {"scenarios": ["exhibition_tour", "nope"]})

    assert result["unknown"] == ["nope"]
    assert result["scenarios"] == ["exhibition_tour"]


def test_a_suite_of_only_unknown_scenarios_is_an_error(rig):
    result = rig["scenario"].dispatch("run_suite", {"scenarios": ["nope"]})

    assert "error" in result
    assert result["available"] == ["exhibition_tour"]


# ── silence is not success ───────────────────────────────────────────────────

def test_a_run_that_stops_producing_events_is_recorded_as_stalled():
    """A crashed agent must not score the same as one that has not finished."""
    rig = build(guide=False)          # nothing drives the robot at all
    try:
        rig["scenario"].dispatch("run_suite", {"scenarios": ["exhibition_tour"]})
        run(rig, 400.0)

        summary = rig["report"].dispatch("sim_report", {"what": "suite"})

        assert summary["outcomes"] == {"stalled": 1}
        assert summary["cases"][0]["detail"].startswith("no event for")
        assert summary["cases"][0]["score"]["total"] < 100.0
    finally:
        acp.set_transport(None)
        acp.clear_observers()


def test_a_stalled_case_still_gets_scored_rather_than_skipped():
    rig = build(guide=False)
    try:
        rig["scenario"].dispatch("run_suite", {"scenarios": ["exhibition_tour"]})
        run(rig, 400.0)

        case = rig["report"].dispatch("sim_report", {"what": "suite"})["cases"][0]

        assert "waypoint_order" in case["failures"]
        assert case["score"]["by_dimension"]["orchestration"] is not None
    finally:
        acp.set_transport(None)
        acp.clear_observers()


def test_abort_keeps_the_results_already_collected(rig):
    rig["scenario"].dispatch("run_suite", {"scenarios": ["exhibition_tour"], "repeats": 3})
    run(rig, 150.0)          # long enough for one case, not for three

    result = rig["scenario"].dispatch("abort_suite", {})
    summary = rig["report"].dispatch("sim_report", {"what": "suite"})

    assert result["suite"]["state"] == "done"
    assert summary["outcomes"].get("ok", 0) >= 1
    assert summary["outcomes"].get("pending", 0) >= 1


# ── scoring behaviour ────────────────────────────────────────────────────────

def test_a_guide_that_never_resumes_scores_lower_than_one_that_does():
    good = build()
    bad = build(resume=False)
    try:
        for rig in (good, bad):
            rig["scenario"].dispatch("run_suite", {"scenarios": ["exhibition_tour"]})
            run(rig, 900.0)

        good_summary = good["report"].dispatch("sim_report", {"what": "suite"})
        bad_summary = bad["report"].dispatch("sim_report", {"what": "suite"})

        assert bad_summary["mean"] < good_summary["mean"]
        assert bad_summary["scenarios"]["exhibition_tour"]["by_dimension"]["long_horizon"] == 0.0
        assert good_summary["scenarios"]["exhibition_tour"]["by_dimension"]["long_horizon"] == 100.0
    finally:
        acp.set_transport(None)
        acp.clear_observers()


def test_dimension_means_omit_what_nothing_measured(rig):
    """`not measured` and `measured and failed` are different facts; averaging
    them is how a benchmark starts lying."""
    rig["scenario"].dispatch("run_suite", {"scenarios": ["exhibition_tour"]})
    run(rig, 900.0)

    dimensions = rig["report"].dispatch(
        "sim_report", {"what": "suite"})["scenarios"]["exhibition_tour"]["by_dimension"]

    assert set(dimensions) == {"orchestration", "interruption", "long_horizon", "safety", "latency"}
    assert all(value is not None for value in dimensions.values())


def test_suite_progress_shows_up_on_the_state_topic(rig):
    """The `data/json` topic the KV panel renders — a `resource` card has no
    renderer of its own, so without this the batch is invisible on the canvas."""
    rig["scenario"].dispatch("run_suite", {"scenarios": ["exhibition_tour"], "repeats": 2})
    run(rig, 30.0)

    payload = rig["scenario"].payload()

    assert payload["suite"]["state"] == "running"
    assert payload["suite"]["cases"] == 2
    assert payload["suite"]["current"] == "exhibition_tour"


def test_each_case_records_the_failures_by_name(rig):
    rig["scenario"].dispatch("run_suite", {"scenarios": ["exhibition_tour"]})
    run(rig, 900.0)

    case = rig["report"].dispatch("sim_report", {"what": "suite"})["cases"][0]

    assert case["failures"] == []
    assert case["elapsed"] is not None and case["elapsed"] > 0
