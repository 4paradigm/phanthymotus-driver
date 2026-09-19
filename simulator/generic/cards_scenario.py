"""`sim_scenario` and `sim_report` — the run control and the oracle.

This is what makes the bundle usable "the same way as the current canvas": you
drag `sim_scenario` onto the canvas next to `nav` and `tts`, wire it to
`decision_core`, and it is a card like any other.

## Why `sim_scenario` is an actuator and `sim_report` is a resource

`sim_scenario` changes the world, so it should obey the barrier exactly like
anything else — the simulator's own control path is subject to the rules it is
testing.

`sim_report` must be `resource`: `_needs_barrier` (llm.py:601) exempts `sensor`
and `resource`, which is what lets progress be read *while* a 90 second
navigation is pending. As an actuator every status read would queue behind the
tour, and a suite run could not be polled at all.

## Why the report also goes out on a topic

A `resource` card has **no renderer on the canvas** — grepping `'resource'`
across `agent-core/web/js/` returns nothing. So `sim_report` alone is invisible;
the verdict has to be published as `data/json` as well, where `kv-latest.js`
picks it up. `sim_report` stays because CI and scripts need the structured reply.
"""

from __future__ import annotations

import json
import os
from pathlib import Path

from simulator.generic import acp, assertions
from simulator.generic.card_base import Card
from simulator.generic.scenario import Scenario, discover
from simulator.generic.suite import SuiteRunner

BUNDLE_DIR = Path(__file__).resolve().parent
DEFAULT_SCENARIO_DIRS = (BUNDLE_DIR / "scenarios", BUNDLE_DIR / "scenarios" / "user")


class SimScenarioCard(Card):
    NAME = "sim_scenario"
    KIND = "actuator"
    DESCRIPTION = "仿真场景控制 — 加载/启动/注入事件/复位；跑一趟可断言的导览"
    TOPIC = "scenario/state"
    FORMAT = "data/json"
    HZ = 2.0
    HOOKS = {"on_interrupt_all": {"action": "note"}}
    # `run`/`abort`, not `start`/`stop`: the framework sends `start` and `stop` to
    # every card as lifecycle verbs, so a tour named `start` would be kicked off
    # by agent-core merely starting the project — and `Card.dispatch` would
    # intercept it before `do_start` ever ran.
    ACTIONS = {
        "load": (["scenario"], "加载一个场景：重置世界、装载地图与航点"),
        "run": ([], "开始计时并启动脚本化注入"),
        "inject": (["text", "kind"], "立刻注入一个事件（如用户插话）"),
        "abort": ([], "停止计时，保留事件记录"),
        "reset": ([], "回到场景初始状态并清空记录"),
        "note": (["text"], "往事件记录里写一条备注；打断钩子绑在这里"),
        "run_suite": (["scenarios", "repeats", "seed"],
                      "按顺序跑一批场景并逐个评分；立即返回，进度用 sim_report 轮询"),
        "abort_suite": ([], "中止正在跑的批次，保留已完成的结果"),
        "read": ([], "读取当前场景与进度"),
    }
    PROPERTIES = {
        "scenario": {"type": "string", "description": "场景 slug"},
        "text": {"type": "string"},
        "kind": {"type": "string", "description": "事件类型，默认 user_message"},
        "scenarios": {"type": "array", "items": {"type": "string"},
                      "description": "要跑的场景 slug 列表；留空表示全部"},
        "repeats": {"type": "integer", "description": "每个场景重复次数；LLM 是随机的，n=1 的分数没有意义"},
        "seed": {"type": "integer"},
    }
    CONFIG_SCHEMA = {}

    def __init__(self, world, config, namespace, ros2=None, scenario_dirs=None):
        super().__init__(world, config, namespace, ros2)
        self._dirs = [Path(d) for d in (scenario_dirs or DEFAULT_SCENARIO_DIRS)]
        self._scenarios: dict[str, Scenario] = {}
        self._active: Scenario | None = None
        self._t0: float | None = None
        self._fired: set[int] = set()
        self._acp_posts: list[dict] = []
        self._injector = None
        self.refresh()
        world.add_step_listener(self._on_step)
        self.suite = SuiteRunner(self, world)
        # What actually went to /api/acp/complete, not the world's internal job
        # payload — see `acp.add_observer`.
        acp.add_observer(self.record_acp)

    # ---- discovery ----------------------------------------------------

    def refresh(self) -> dict[str, Scenario]:
        self._scenarios = discover(*self._dirs)
        return self._scenarios

    def config_schema(self) -> dict:
        """Built at call time from the scenario directory, so dropping a YAML
        into the bind-mounted folder and refreshing the canvas is enough — no
        rebuild, no code change. `sidebar.js` renders an `enum` as a <select>,
        and `oneOf [{const, title}]` lets the label be the human name while the
        value stays the slug."""
        scenarios = self.refresh()
        options = [{"const": slug, "title": scenario.name}
                   for slug, scenario in sorted(scenarios.items())]
        return {
            "type": "object",
            "properties": {
                "scenario": {
                    "type": "string", "title": "场景",
                    "description": "从 scenarios/ 目录扫描得到；新增 yaml 后刷新画布即可",
                    **({"oneOf": options} if options else {}),
                },
                "speed_scale": {"type": "number", "title": "加速倍率", "default": 1.0,
                                "description": "仅影响仿真时钟，不影响 LLM 往返"},
                "seed": {"type": "integer", "title": "随机种子", "default": 0},
            },
        }

    # ---- injection ----------------------------------------------------

    def set_injector(self, fn) -> None:
        """``fn(text, kind)`` — publishes onto `/remote_control/message`, which is
        the same path a real remote control uses. Left unset, injections are
        recorded in the event log only, which is what the pytest runner wants."""
        self._injector = fn

    def _on_step(self, t: float, _dt: float) -> None:
        if self._active is None or self._t0 is None:
            return
        elapsed = t - self._t0
        arrivals = {e["label"]: e["t"] - self._t0
                    for e in self.world.events()
                    if e["event"] == "arrive" and e.get("label")}
        for index, injection in enumerate(self._active.injections):
            if index in self._fired or not injection.due(elapsed, arrivals):
                continue
            self._fired.add(index)
            self._deliver(injection.text, injection.kind, scheduled=round(elapsed, 2))

    def _deliver(self, text: str, kind: str, scheduled: float | None = None) -> dict:
        event = self.world.log("injection", text=text, injection_kind=kind, scheduled=scheduled)
        if self._injector is not None:
            try:
                self._injector(text, kind)
            except Exception as exc:
                print(f"[sim-scenario] injection transport failed: {exc}", flush=True)
        return event

    # ---- actions ------------------------------------------------------

    def do_load(self, scenario: str = "", **_):
        scenarios = self.refresh()
        chosen = scenarios.get(scenario) or (self._active if not scenario else None)
        if chosen is None:
            return {"error": f"unknown scenario: {scenario}", "available": sorted(scenarios)}
        self._active = chosen
        self._t0 = None
        self._fired = set()
        self._acp_posts = []
        self.world.reset(chosen.scene())
        # 场景里的 POI 载入成地图上的 tag —— 真机上导览前也是先把展区打好点，
        # 之后每一段走的都是 navigate_to_tag。
        self.world.set_tags(chosen.waypoints())
        warnings = chosen.validate()
        for warning in warnings:
            self.world.log("scenario_warning", text=warning)
        self.world.log("scenario_load", scenario=chosen.slug, name=chosen.name)
        return {"state": "idle", "loaded": chosen.slug, "warnings": warnings, **chosen.summary()}

    def do_run(self, **_):
        if self._active is None:
            return {"error": "no scenario loaded"}
        self._t0 = self.world.snapshot()["t"]
        self._fired = set()
        self.world.log("scenario_start", scenario=self._active.slug)
        return {"state": "running", "scenario": self._active.slug,
                "waypoints": [poi.get("name") for poi in self._active.pois]}

    def do_abort(self, **_):
        self._t0 = None
        if self._active is not None:
            self.world.log("scenario_stop", scenario=self._active.slug)
        return {"state": "idle"}

    def do_reset(self, **_):
        if self._active is None:
            return {"state": "idle", "reset": False}
        return self.do_load(scenario=self._active.slug)

    def do_inject(self, text: str = "", kind: str = "user_message", **_):
        if not str(text).strip():
            return {"error": "inject requires text"}
        self._deliver(str(text), kind)
        return {"state": "running", "injected": text, "kind": kind}

    def do_note(self, text: str = "", **_):
        # Bound to on_interrupt_all so every barge-in leaves a trace. On a robot
        # whose navigation does not stop, this note is the only evidence that the
        # interrupt was ever delivered.
        self.world.log("note", text=str(text) or "interrupt")
        return {"state": "running", "noted": text}

    def do_run_suite(self, scenarios=None, repeats: int = 1, seed: int = 0, **_):
        available = sorted(self.refresh())
        chosen = [slug for slug in (scenarios or available) if slug in available]
        unknown = [slug for slug in (scenarios or []) if slug not in available]
        if not chosen:
            return {"error": "no runnable scenario", "available": available, "unknown": unknown}
        # `suite` is nested, not spread: the card has a `state` and so does the
        # batch, and spreading let the batch's overwrite the card's — the third
        # time in this bundle that two different meanings collided under one key.
        return {"state": "running", "scenarios": chosen, "repeats": max(1, int(repeats)),
                "unknown": unknown, "suite": self.suite.start(chosen, repeats=repeats, seed=seed)}

    def do_abort_suite(self, **_):
        return {"state": "idle", "suite": self.suite.abort()}

    def do_read(self, **_):
        return self.payload()

    # ---- published state ----------------------------------------------

    def elapsed(self) -> float | None:
        if self._t0 is None:
            return None
        return round(self.world.snapshot()["t"] - self._t0, 2)

    def waypoints(self) -> list[dict]:
        return self._active.waypoints() if self._active else []

    def record_acp(self, body: dict) -> None:
        """One ACP body as posted. Only while a scenario is loaded — otherwise a
        card constructed for another purpose would accumulate forever."""
        if self._active is not None:
            self._acp_posts.append(dict(body))

    @property
    def active(self) -> Scenario | None:
        return self._active

    @property
    def acp_posts(self) -> list[dict]:
        return list(self._acp_posts)

    def payload(self):
        snapshot = self.world.snapshot()
        events = self.world.events()
        visited = [e["label"] for e in events if e["event"] == "arrive" and e.get("label")]
        return {
            "state": "running" if self._t0 is not None else "idle",
            "scenario": self._active.slug if self._active else None,
            "scenario_name": self._active.name if self._active else None,
            "elapsed": self.elapsed(),
            "waypoints": [poi.get("name") for poi in self.waypoints()],
            "visited": visited,
            "remaining": [name for name in [poi.get("name") for poi in self.waypoints()]
                          if name not in visited],
            "pose": snapshot["pose"],
            "job": snapshot["job"],
            "speech": snapshot["speech"],
            "injections_fired": len(self._fired),
            "acp_posts": len(self._acp_posts),
            "suite": self.suite.status(),
        }


class SimReportCard(Card):
    """Facts and verdicts. Never talks to agent-core — the judge and the system
    under test stay disjoint, or a broken ACP path marks itself green."""

    NAME = "sim_report"
    KIND = "resource"
    DESCRIPTION = "仿真运行结果 — 事件记录、播报记录、断言判定与得分"
    TOPIC = ""

    def __init__(self, world, config, namespace, ros2=None, scenario_card=None):
        super().__init__(world, config, namespace, ros2)
        self._scenario_card = scenario_card

    def dispatch(self, action: str, args: dict) -> dict:
        # A resource tool is invoked by its own name; vendor_runtime does not pop
        # an `action` for this kind, so sub-commands arrive as an explicit arg.
        args = {k: v for k, v in (args or {}).items() if k != "_tool_name"}
        which = args.get("what", "report")
        if which == "list":
            return self.do_list()
        if which == "suite":
            return self.do_suite()
        return self.report()

    def do_list(self) -> dict:
        scenarios = self._scenario_card.refresh() if self._scenario_card else {}
        return {"scenarios": [scenario.summary() for scenario in sorted(
            scenarios.values(), key=lambda s: s.slug)]}

    def do_suite(self) -> dict:
        """Batch progress and scores. Readable *while* the suite runs, because
        this card is a `resource` and `_needs_barrier` exempts those."""
        card = self._scenario_card
        if card is None:
            return {"error": "no scenario card"}
        return card.suite.summary()

    def transcript(self) -> list[dict]:
        """Requested text, start, end, outcome. This is the assertion surface for
        announcement correctness — and on an Orin, which has no real speaker, the
        only way to check announcement ordering at all."""
        events = self.world.events()
        starts = {e.get("action_id"): e for e in events if e["event"] == "speak_start"}
        lines = []
        for event in events:
            if event["event"] != "speak_end":
                continue
            start = starts.get(event.get("action_id"), {})
            lines.append({"text": start.get("text", event.get("text", "")),
                          "started_at": start.get("t"), "ended_at": event["t"],
                          "status": event.get("status")})
        return lines

    def report(self) -> dict:
        card = self._scenario_card
        scenario = card.active if card else None
        events = self.world.events()
        if scenario is None:
            return {"state": "idle", "error": "no scenario loaded",
                    "events": len(events), "transcript": self.transcript()}

        grid = self.world._backend.state()["grid"]  # noqa: SLF001
        results = assertions.evaluate(scenario, events, acp_posts=card.acp_posts,
                                      trail=self.world.trail(), grid=grid)
        return {
            "state": "running" if card.elapsed() is not None else "idle",
            "scenario": scenario.slug,
            "scenario_name": scenario.name,
            "elapsed": card.elapsed(),
            "events": events,
            "transcript": self.transcript(),
            "waypoints": [e["label"] for e in events if e["event"] == "arrive" and e.get("label")],
            "acp_posts": card.acp_posts,
            "assertions": results,
            "score": assertions.score(scenario, results),
            "environment": _environment(),
        }


def _environment() -> dict:
    """What the run was measured against. A score with no configuration attached
    is noise — the axis anyone actually cares about is "did the number move when
    I changed the model or the prompt", and that is unanswerable without this."""
    return {
        "tier": os.environ.get("SIM_TIER", "fidelity"),
        "host": os.environ.get("HOSTNAME", ""),
        "agent_core_url": os.environ.get("AGENT_CORE_URL", ""),
        "image_tag": os.environ.get("IMAGE_TAG", ""),
        "git_sha": os.environ.get("GIT_SHA", ""),
        "llm_model": os.environ.get("SIM_LLM_MODEL", ""),
    }


def scenario_report_json(card: SimReportCard) -> str:
    return json.dumps(card.report(), ensure_ascii=False)
