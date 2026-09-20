"""`sim_scenario` and `sim_report` — world control, and the facts it produced.

This is what makes the bundle usable "the same way as the current canvas": you
drag `sim_scenario` onto the canvas next to `nav` and `tts`, wire it to
`decision_core`, and it is a card like any other.

## These cards do not own a test case

They used to: `run_suite` drove batches, `assertions.py` judged them, and the
score came back out of `sim_report`. Both are gone. A test case is
**solution + execution plan + evaluation plan**, it spans agent-core and the
driver, and a card cannot hold it — a card lives in one driver, so when the
driver is missing the card does not exist either, and there is nowhere to say
"you are missing this driver". The case is a `test` section in a solution
package; agent-core runs it and judges it (`benchmark_case.py`,
`benchmark_runner.py`).

What is left here is the world: load it, reset it, inject into it, and report
what happened. **The driver produces facts; agent-core judges them.**

## Why `sim_scenario` is an actuator and `sim_report` is a resource

`sim_scenario` changes the world, so it should obey the barrier exactly like
anything else — the simulator's own control path is subject to the rules it is
testing.

`sim_report` must be `resource`: `_needs_barrier` (llm.py:601) exempts `sensor`
and `resource`, which is what lets progress be read *while* a 90 second
navigation is pending. As an actuator every status read would queue behind the
tour, and a run in progress could not be polled at all.

## Why the report also goes out on a topic

A `resource` card has **no renderer on the canvas** — grepping `'resource'`
across `agent-core/web/js/` returns nothing. So `sim_report` alone is invisible;
progress has to be published as `data/json` as well, where `kv-latest.js` picks
it up. `sim_report` stays because agent-core, CI and scripts read the structured
reply.
"""

from __future__ import annotations

import json
import os
from pathlib import Path

from simulator.generic import acp
from simulator.generic.card_base import Card
from simulator.generic.geometry import OCCUPIED
from simulator.generic.maps import discover as discover_maps
from simulator.generic.scenario import Scenario, discover

BUNDLE_DIR = Path(__file__).resolve().parent
DEFAULT_SCENARIO_DIRS = (BUNDLE_DIR / "scenarios", BUNDLE_DIR / "scenarios" / "user")
DEFAULT_MAP_DIRS = (BUNDLE_DIR / "maps", BUNDLE_DIR / "maps" / "user")


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
        "reset": (["scenario", "map", "spawn", "seed"],
                  "回到初始状态并清空记录；可指定地图与出生位姿"),
        "note": (["text"], "往事件记录里写一条备注；打断钩子绑在这里"),
        "list_maps": ([], "列出可用地图（扫描目录，新增地图无需重建镜像）"),
        "read": ([], "读取当前场景与进度"),
    }
    PROPERTIES = {
        "scenario": {"type": "string", "description": "场景 slug"},
        "text": {"type": "string"},
        "kind": {"type": "string", "description": "事件类型，默认 user_message"},
        "map": {"type": "string", "description": "reset 时指定地图；留空沿用当前场景"},
        "spawn": {"type": "object", "description": "reset 时指定出生位姿 {x, y, yaw}"},
        "seed": {"type": "integer"},
        "owner": {"type": "string",
                  "description": "基准测试跑动的持有者；被持有时别人改不动世界"},
    }
    CONFIG_SCHEMA = {}

    def __init__(self, world, config, namespace, ros2=None, scenario_dirs=None, map_dirs=None):
        super().__init__(world, config, namespace, ros2)
        self._dirs = [Path(d) for d in (scenario_dirs or DEFAULT_SCENARIO_DIRS)]
        self._map_dirs = [Path(d) for d in (map_dirs or DEFAULT_MAP_DIRS)]
        self._scenarios: dict[str, Scenario] = {}
        self._active: Scenario | None = None
        self._t0: float | None = None
        self._fired: set[int] = set()
        self._acp_posts: list[dict] = []
        self._injector = None
        self._owner = ""
        self.refresh()
        world.add_step_listener(self._on_step)
        # What actually went to /api/acp/complete, not the world's internal job
        # payload — see `acp.add_observer`.
        acp.add_observer(self.record_acp)

    # ---- discovery ----------------------------------------------------

    def refresh(self) -> dict[str, Scenario]:
        self._scenarios = discover(*self._dirs)
        return self._scenarios

    def maps(self) -> dict:
        """每次调用重扫 —— 丢一份地图进 bind-mount 的目录，刷新画布就能选到。"""
        return discover_maps(*self._map_dirs)

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

    # ---- ownership ----------------------------------------------------

    def _held_by_someone_else(self, owner: str) -> dict | None:
        """基准测试跑着的时候，别人改不动世界。

        这张卡是 LLM 可调用的工具，`load` 和 `reset` 就在它的 action 枚举里 ——
        也就是**被测的 agent 能重置正在测它的那次测量**，而且重置之后什么痕迹都不
        剩，那次跑动只会记一个说不清的低分。跑动的持有者带 `owner` 过来，其他调用
        方在此期间被拒。
        """
        if self._owner and owner != self._owner:
            return {"error": f"世界正被 {self._owner} 的基准测试跑动持有",
                    "owner": self._owner}
        return None

    # ---- actions ------------------------------------------------------

    def do_load(self, scenario: str = "", owner: str = "", **_):
        held = self._held_by_someone_else(owner)
        if held:
            return held
        scenarios = self.refresh()
        chosen = scenarios.get(scenario) or (self._active if not scenario else None)
        if chosen is None:
            return {"error": f"unknown scenario: {scenario}", "available": sorted(scenarios)}
        # 场景引用地图名时，在这里解析并绑上 —— 只有这张卡知道去哪些目录找。
        if chosen.map_name:
            asset = self.maps().get(chosen.map_name)
            if asset is None:
                return {"error": f"场景 {chosen.slug} 引用的地图 {chosen.map_name} 找不到",
                        "available_maps": sorted(self.maps())}
            chosen.bind_map(asset)
        self._active = chosen
        return self._load_active()

    def _load_active(self, seed: int = 0):
        chosen = self._active
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

    def do_abort(self, owner: str = "", **_):
        held = self._held_by_someone_else(owner)
        if held:
            return held
        self._t0 = None
        self._owner = ""            # 跑动结束，世界还给画布
        if self._active is not None:
            self.world.log("scenario_stop", scenario=self._active.slug)
        return {"state": "idle"}

    def do_reset(self, scenario: str = "", map: str = "", spawn=None,  # noqa: A002
                 seed: int = 0, owner: str = "", **_):
        """回到起点。

        地图与出生点可以由调用方给 —— 基准测试用例自带世界（`test.run.world`），
        驱动里的 yaml 是另一份来源。两份各存一次，迟早会对不上，所以用例给了就以
        用例为准。
        """
        held = self._held_by_someone_else(owner)
        if held:
            return held
        self._owner = owner or self._owner

        if map:
            asset = self.maps().get(map)
            if asset is None:
                return {"error": f"找不到地图 {map}", "available_maps": sorted(self.maps())}
            # 指定了地图就**从这张地图新建一个场景**，不是把新地图绑到旧场景上。
            #
            # Orin6 上抓到的：绑到旧场景上，栅格换成了 bj-2f，航点却还是上一个场景
            # 的（`Scenario.waypoints()` 自己的 `pois` 优先于地图的）。于是机器人
            # 在北京 2F 的图上找「一号展区」—— 用例说的 P3/P4/P5 一个都不存在，
            # 而日志里看着一切正常：导航发出去了，barrier 也过了。
            target = Scenario.from_dict({"name": asset.name}, slug=map)
            target.bind_map(asset)
            if spawn:
                target.spawn = dict(spawn)
            elif asset.pois:
                # 没给出生点就停在第一个点位上，而不是原点 —— 真实地图的原点通常
                # 在墙里，机器人一上来就判撞。
                first = asset.pois[0]
                target.spawn = {"x": float(first.get("x", 0.0)),
                                "y": float(first.get("y", 0.0)),
                                "yaw": float(first.get("yaw", 0.0))}
            self._active = target
            return self._load_active(seed=seed)

        if self._active is None:
            return {"state": "idle", "reset": False}
        return self.do_load(scenario=self._active.slug, owner=owner)

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

    def do_list_maps(self, **_):
        return {"state": "running" if self._running else "idle",
                "maps": [m.summary() for m in sorted(self.maps().values(), key=lambda a: a.name)]}

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
            "owner": self._owner,
        }


class SimReportCard(Card):
    """Facts, and only facts.

    The verdict used to be computed here, which put the judge inside the system
    under test — a broken ACP path marks itself green. It now lives in
    agent-core's `benchmark_case.py`, and this card produces what the judge reads:
    the event log, the transcript, the ACP bodies as posted, and the few
    measurements that need the world's own geometry to compute.

    `trail_occupied` is the one that has to stay on this side. Judging "never
    drove through a wall" from the integrator's own `nav_failed` events asks the
    integrator to report its own bug; measuring the trail against the grid does
    not. So the count is a fact produced here and judged there.
    """

    NAME = "sim_report"
    KIND = "resource"
    DESCRIPTION = "仿真运行结果 — 事件记录、播报记录、ACP 上报"
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
        return self.report()

    def do_list(self) -> dict:
        scenarios = self._scenario_card.refresh() if self._scenario_card else {}
        return {"scenarios": [scenario.summary() for scenario in sorted(
            scenarios.values(), key=lambda s: s.slug)]}

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

        return {
            "state": "running" if card.elapsed() is not None else "idle",
            "scenario": scenario.slug,
            "scenario_name": scenario.name,
            "elapsed": card.elapsed(),
            "events": events,
            "transcript": self.transcript(),
            "waypoints": [e["label"] for e in events if e["event"] == "arrive" and e.get("label")],
            "acp_posts": card.acp_posts,
            "trail_occupied": self.trail_occupied(),
            "environment": _environment(),
        }

    def trail_occupied(self) -> int:
        """走过的轨迹里落在占用格上的点数。

        必须在这一侧算 —— 它要的是世界自己的栅格。而它值得算：只看积分器自己报的
        `nav_failed`，等于让积分器报告自己的 bug。
        """
        grid = self.world._backend.state()["grid"]  # noqa: SLF001
        if grid is None:
            return 0
        return sum(1 for x, y in self.world.trail()
                   if grid.at(*grid.world_to_cell(x, y)) == OCCUPIED)


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
