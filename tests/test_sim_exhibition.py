"""The exhibition tour, replayed under a fake clock.

On a rig an LLM drives these cards. Here a deliberately small scripted agent
stands in for it, so that CI can run the same scenario file, against the same
`assertions.py`, without an API key or four minutes of wall clock.

**The agent is not the oracle.** It only calls cards; every verdict comes from
`assertions.evaluate` reading the event log and the ACP posts. Keeping those
disjoint is the whole reason a broken ACP path cannot mark itself green — so this
file also runs deliberately *wrong* agents and checks that the oracle fails them.

Run: PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest tests/test_sim_exhibition.py -q
"""

from __future__ import annotations

import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from simulator.generic import acp, assertions  # noqa: E402
from simulator.generic.backend import LocalBackend  # noqa: E402
from simulator.generic.cards_audio import TtsCard  # noqa: E402
from simulator.generic.cards_motion import ControlledSpatialCard  # noqa: E402
from simulator.generic.cards_scenario import SimReportCard, SimScenarioCard  # noqa: E402
from simulator.generic.cards_sensors import SpatialMapCard  # noqa: E402
from simulator.generic.clock import FakeClock  # noqa: E402
from simulator.generic.scenario import Scenario  # noqa: E402
from simulator.generic.world import VirtualWorld  # noqa: E402

SCENARIO_DIR = ROOT / "simulator" / "generic" / "scenarios"
TOUR = SCENARIO_DIR / "exhibition_tour.yaml"
DT = 0.05


# ── the stand-in agent ───────────────────────────────────────────────────────

class ScriptedGuide:
    """What a competent LLM would do: go, announce, go on — and when interrupted,
    take the detour and then come back to the leg it abandoned.

    Runs off the world's step listener so it reacts on simulated time. It reads
    only what a real agent can see: ACP terminals and the event log.
    """

    LLM_LATENCY = 0.6          # a decision is not instantaneous

    def __init__(self, nav, tts, scenario, plan, resume=True, announce=True, register=True):
        self.nav, self.tts, self.scenario = nav, tts, scenario
        self.queue = list(plan)
        self.resume = resume
        self.announce = announce
        self.state = "idle"
        self.current = None
        self.interrupted_at = None
        self.wake_at = 0.0
        self._terminals: list[dict] = []
        self._seen_injections = 0
        nav.world.add_nav_listener(self._terminals.append)
        if register:
            # A suite constructs one of these per case and steps it itself. Left
            # self-registered, every past guide keeps driving the same robot.
            nav.world.add_step_listener(self.on_step)

    def on_step(self, t, _dt):
        if t < self.wake_at:
            return
        self._handle_injection(t)
        terminal = self._terminals.pop(0) if self._terminals else None
        if terminal is not None:
            self._on_terminal(terminal, t)
            return
        if self.state == "idle":
            self._next_leg(t)
        elif self.state == "announcing" and self.nav.world.snapshot()["speech"] is None:
            self.state = "idle"
            self.wake_at = t + self.LLM_LATENCY

    def _handle_injection(self, t):
        injections = [e for e in self.nav.world.events() if e["event"] == "injection"]
        if len(injections) <= self._seen_injections:
            return
        self._seen_injections = len(injections)
        if self.state != "navigating" or self.current is None:
            return
        # Abandon this leg, take the detour, and — if this guide is the competent
        # one — put the abandoned waypoint back at the front of the queue.
        self.interrupted_at = self.current
        self.nav.dispatch("stop_nav", {})
        head = ["洗手间"] + ([self.current] if self.resume else [])
        self.queue = head + self.queue
        self.state = "idle"
        self.wake_at = t + self.LLM_LATENCY

    def _on_terminal(self, terminal, t):
        if terminal["status"] != "completed":
            self.state = "idle"
            self.wake_at = t + self.LLM_LATENCY
            return
        label = terminal.get("label") or ""
        if self.announce and label:
            poi = self.scenario.waypoint(label) or {}
            self.tts.dispatch("speak", {"text": f"{label}到了。{poi.get('description', '')}"})
            self.state = "announcing"
        else:
            self.state = "idle"
            self.wake_at = t + self.LLM_LATENCY

    def _next_leg(self, t):
        if not self.queue:
            self.state = "done"
            return
        self.current = self.queue.pop(0)
        result = self.nav.dispatch("navigate_to_tag", {"name": self.current})
        if "error" in result:
            self.state = "done"
            return
        self.state = "navigating"


# ── rig ──────────────────────────────────────────────────────────────────────

def build(scenario_slug="exhibition_tour"):
    posts: list[dict] = []
    acp.clear_observers()
    acp.set_transport(lambda url, payload: posts.append(payload))
    clock = FakeClock()
    world = VirtualWorld(LocalBackend(), clock, {"tick_hz": 1.0 / DT, "chars_per_sec": 5.0})
    config = {"embodiment": {"kind": "wheeled", "dof": 2, "joint_names": ["a", "b"]}}

    scenario_card = SimScenarioCard(world, config, "sim", scenario_dirs=[SCENARIO_DIR])
    nav = ControlledSpatialCard(world, config, "sim")
    tts = TtsCard(world, config, "sim")
    report = SimReportCard(world, config, "sim", scenario_card=scenario_card)

    scenario_card.dispatch("load", {"scenario": scenario_slug})
    return {"world": world, "clock": clock, "scenario": scenario_card, "nav": nav,
            "tts": tts, "report": report, "posts": posts, "config": config}


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


def drive(rig, plan=("入口", "一号展区", "二号展区", "三号展区"), seconds=600.0, **kwargs):
    guide = ScriptedGuide(rig["nav"], rig["tts"], Scenario.load(TOUR), plan, **kwargs)
    rig["scenario"].dispatch("run", {})
    run(rig, seconds)
    return guide


# ── the scenario file itself ─────────────────────────────────────────────────

def test_the_shipped_tour_has_no_unreachable_leg():
    """`LocalBackend` drives straight at the target — there is no path planner,
    while the real Slamtec chassis has one. A wall between two waypoints would
    fail the leg for a reason that has nothing to do with orchestration, so
    `validate()` reports it at load rather than leaving it to be discovered."""
    assert Scenario.load(TOUR).validate() == []


def test_load_reports_warnings_rather_than_failing_silently():
    scenario = Scenario.from_dict({
        "name": "blocked", "spawn": {"x": 0.0, "y": 0.0},
        "map": {"resolution": 0.1, "bounds": [-1, -1, 11, 5], "walls": [[4.0, -1.0, 4.4, 5.0]]},
        "pois": [{"name": "far", "x": 9.0, "y": 0.0}],
    }, slug="blocked")

    # validate 只查导览真的会走的那些段 —— 一张 14 个点的真实地图全连通要 91 次
    # 规划，而绝大多数段这趟导览根本不走。
    scenario.expect = {"waypoint_order": ["far"]}
    warnings = scenario.validate()

    assert warnings and "far" in warnings[0]


def test_scenario_discovery_feeds_the_canvas_dropdown(rig):
    """`sidebar.js` renders `oneOf [{const, title}]` as a <select>, so dropping a
    yaml into the directory is enough — no rebuild, no code change."""
    schema = rig["scenario"].config_schema()
    options = schema["properties"]["scenario"]["oneOf"]

    assert {"const": "exhibition_tour", "title": "展厅导览 · 基础巡游"} in options


# ── the happy path ───────────────────────────────────────────────────────────

def test_a_competent_guide_passes_every_assertion(rig):
    drive(rig)

    report = rig["report"].report()
    failures = [check for check in report["assertions"] if not check["ok"]]

    assert not failures, [(f["name"], f["detail"]) for f in failures]
    assert report["score"]["total"] == 100.0


def test_the_tour_visits_the_waypoints_in_the_expected_order(rig):
    drive(rig)

    visited = rig["report"].report()["waypoints"]

    assert visited == ["入口", "一号展区", "洗手间", "二号展区", "三号展区"]


def test_every_waypoint_is_announced_after_arriving_and_before_leaving(rig):
    drive(rig)
    report = rig["report"].report()

    transcript = report["transcript"]
    assert len(transcript) == 5
    assert all(line["status"] == "completed" for line in transcript)
    assert transcript[0]["text"].startswith("入口到了")


def test_the_interrupted_leg_is_reported_cancelled_with_partial_progress(rig):
    drive(rig)
    report = rig["report"].report()

    leg = next(check for check in report["assertions"] if check["name"] == "interrupted_leg")
    cancelled = [p for p in report["acp_posts"] if p.get("status") == "cancelled"]

    assert leg["ok"], leg["detail"]
    assert [p["result"]["label"] for p in cancelled] == ["二号展区"]
    assert 0.0 < cancelled[0]["result"]["progress"]["fraction"] < 1.0


def test_the_injection_fires_relative_to_reaching_a_waypoint(rig):
    drive(rig)

    injections = [e for e in rig["world"].events() if e["event"] == "injection"]

    assert len(injections) == 1
    arrivals = [e["t"] for e in rig["world"].events() if e.get("label") == "一号展区"
                and e["event"] == "arrive"]
    assert injections[0]["t"] == pytest.approx(arrivals[0] + 8.0, abs=0.1)
    assert injections[0]["text"].startswith("先别走了")


def test_each_action_reports_its_terminal_exactly_once(rig):
    drive(rig)
    posts = rig["report"].report()["acp_posts"]

    ids = [p["action_id"] for p in posts]

    assert len(ids) == len(set(ids)), "an action reported twice"


# ── the oracle has to be able to fail ────────────────────────────────────────

def test_a_guide_that_forgets_to_resume_fails_long_horizon(rig):
    """The mistake an LLM makes most often: after the detour it carries on to the
    *next* waypoint instead of the one it abandoned. Invisible to any single-step
    check."""
    drive(rig, resume=False)

    report = rig["report"].report()
    resume = next(c for c in report["assertions"] if c["name"] == "resume_correctness")
    order = next(c for c in report["assertions"] if c["name"] == "waypoint_order")

    assert resume["ok"] is False
    assert "never returned" in resume["detail"]
    assert order["ok"] is False
    assert report["score"]["by_dimension"]["long_horizon"] == 0.0
    assert report["score"]["total"] < 100.0


def test_a_silent_guide_fails_the_announcement_check(rig):
    drive(rig, announce=False)

    check = next(c for c in rig["report"].report()["assertions"]
                 if c["name"] == "announce_after_arrive")

    assert check["ok"] is False
    assert "never announced" in check["detail"]


def test_the_oracle_rejects_a_completed_status_on_an_abandoned_leg(rig):
    """The specific lie ACP must never tell: reporting `completed` for a leg the
    robot walked away from says it reached a waypoint it never saw."""
    drive(rig)
    scenario = rig["scenario"].active
    posts = [dict(p) for p in rig["scenario"].acp_posts]
    for post in posts:
        if (post.get("result") or {}).get("label") == "二号展区" and post["status"] == "cancelled":
            post["status"] = "completed"

    check = assertions.check_interrupted_leg(scenario, rig["world"].events(), acp_posts=posts)

    assert check["ok"] is False
    assert "expected 'cancelled'" in check["detail"]


def test_a_tour_that_drives_through_geometry_fails_safety():
    rig = build()
    try:
        # 目标放在场景边界之外 —— 规划器绕不出去，必须如实报失败。
        rig["scenario"].dispatch("run", {})
        rig["nav"].dispatch("navigate_to_pose", {"x": 40.0, "y": 40.0, "yaw": 0.0})
        run(rig, 60.0)

        report = rig["report"].report()
        safety = next(c for c in report["assertions"] if c["name"] == "never_occupied")

        assert safety["ok"] is False
        assert "against geometry" in safety["detail"]
        assert report["score"]["by_dimension"]["safety"] == 0.0
    finally:
        acp.set_transport(None)


# ── scoring ──────────────────────────────────────────────────────────────────

def test_weights_come_from_the_scenario_not_from_code(rig):
    drive(rig, resume=False)
    scenario = rig["scenario"].active
    results = rig["report"].report()["assertions"]

    default = assertions.score(scenario, results)["total"]
    scenario.weights = {**scenario.weights, "long_horizon": 1}
    reweighted = assertions.score(scenario, results)["total"]

    assert reweighted > default, "changing a weight must change the score without a code change"


def test_an_unmeasured_dimension_scores_none_rather_than_zero():
    """`not measured` and `measured and failed` are different facts; averaging
    them together is how a benchmark starts lying."""
    scenario = Scenario.from_dict({"name": "bare", "expect": {}}, slug="bare")

    result = assertions.score(scenario, assertions.evaluate(scenario, [], acp_posts=[]))

    assert result["by_dimension"]["long_horizon"] is None
    assert result["by_dimension"]["latency"] is None


def test_report_records_what_the_run_was_measured_against(rig, monkeypatch):
    """A score with no configuration attached is noise — the axis anyone cares
    about is whether the number moved when the model or the prompt changed."""
    monkeypatch.setenv("SIM_LLM_MODEL", "claude-opus-5")
    monkeypatch.setenv("IMAGE_TAG", "release.260918.abc1234")
    drive(rig, seconds=5.0)

    environment = rig["report"].report()["environment"]

    assert environment["llm_model"] == "claude-opus-5"
    assert environment["image_tag"] == "release.260918.abc1234"
    assert environment["tier"] == "fidelity"


# ── the cards around it ──────────────────────────────────────────────────────

def test_sim_report_is_a_resource_so_progress_can_be_polled_mid_tour(rig):
    """`_needs_barrier` exempts sensor and resource. As an actuator, every status
    read would queue behind a 90 second navigation and a suite could not be
    polled at all."""
    assert rig["report"].get_tool()["type"] == "resource"

    drive(rig, seconds=30.0)
    mid = rig["report"].report()

    assert mid["state"] == "running"
    assert mid["elapsed"] is not None


def test_sim_report_lists_available_scenarios(rig):
    listing = rig["report"].dispatch("sim_report", {"what": "list"})

    by_slug = {item["slug"]: item for item in listing["scenarios"]}

    assert "exhibition_tour" in by_slug
    assert by_slug["exhibition_tour"]["waypoints"][0] == "入口"
    # 北京 2F 那张不在场景里写 pois —— 点位跟着地图走，所以列表里是空的，
    # 载入时才从 maps/bj-2f.json 解析出那 14 个真实点位。
    assert by_slug["beijing_2f_tour"]["waypoints"] == []


def test_scenario_state_topic_carries_progress_for_the_kv_panel(rig):
    drive(rig, seconds=40.0)

    payload = rig["scenario"].payload()

    assert payload["scenario"] == "exhibition_tour"
    assert payload["visited"]
    assert payload["remaining"]
    assert payload["state"] == "running"


def test_the_map_card_shows_the_scenario_waypoints(rig):
    card = SpatialMapCard(rig["world"], rig["config"], "sim")
    card.set_waypoints_provider(rig["scenario"].waypoints)

    assert [w["name"] for w in card._waypoints()] == [  # noqa: SLF001
        "入口", "一号展区", "洗手间", "二号展区", "三号展区"]


def test_reset_returns_the_world_to_the_start_of_the_scenario(rig):
    drive(rig, seconds=40.0)
    assert rig["world"].events()

    rig["scenario"].dispatch("reset", {})

    snapshot = rig["world"].snapshot()
    assert snapshot["pose"] == {"x": 0.0, "y": 0.0, "yaw": 0.0}
    assert [e["event"] for e in rig["world"].events()] == ["scenario_load"]


def test_note_records_a_barge_in_even_when_nothing_else_reacts(rig):
    """Bound to on_interrupt_all. On a robot whose navigation does not stop, this
    note is the only evidence the interrupt was ever delivered."""
    rig["scenario"].dispatch("note", {"text": "on_interrupt_all"})

    assert [e["event"] for e in rig["world"].events() if e["event"] == "note"] == ["note"]
