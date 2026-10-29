"""Scenario definitions: the YAML is the single source of truth.

The same file is *run* by the `sim_scenario` card on a rig and *replayed* by
pytest under a fake clock. One definition, two runners — otherwise the CI green
and the rig green stop meaning the same thing.

It describes a **world**, not a test case. A case is a solution package with a
`test` section, and agent-core runs and judges it; the driver's job stops at
producing facts.

## 地图有两种来源，场景两种都能引用

    map: {bounds: [...], walls: [[...]]}   # 合成的：几个矩形，能读、能在 PR 里改
    map: bj-2f                              # 抓来的：从 maps/ 目录解析成资产

合成地图写成矩形是对的 —— 一串矩形能读、能改、能争论，committed 的二进制栅格不能。
但抓来的地图不成立：北京 2F 是 1543×2198 的真实栅格，没有任何一组矩形描述得了。
所以两种都支持，各用在该用的地方。

**点位跟着地图走，不跟着场景走。** 真机上 POI 就是按地图存的（`poi` 表以
`(name, map_name)` 为键）。场景不写 `pois` 时用地图自带的那套；写了则以场景为准，
这样同一张地图可以有「完整导览」「只看南区」好几个场景而不必复制点位。
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path

from simulator.generic.geometry import OCCUPIED, OccupancyGrid

SCENARIO_SUFFIXES = (".yaml", ".yml")


@dataclass
class Injection:
    """When a scripted event fires.

    `at` is absolute simulated seconds. `after_arrival` + `delay` is relative to
    reaching a waypoint, and is the one to prefer: on a rig the LLM round trip
    runs anywhere from 3 to 48 seconds, so an absolute time that lands mid-leg
    under a fake clock can land two waypoints later on a real machine — and the
    interruption assertion then fails for a reason that has nothing to do with
    the robot.
    """

    at: float | None = None
    after_arrival: str = ""
    delay: float = 0.0
    kind: str = "user_message"
    text: str = ""

    def due(self, elapsed: float, arrivals: dict[str, float]) -> bool:
        if self.after_arrival:
            reached = arrivals.get(self.after_arrival)
            return reached is not None and elapsed >= reached + self.delay
        return self.at is not None and elapsed >= self.at

    def as_dict(self) -> dict:
        return {"at": self.at, "after_arrival": self.after_arrival, "delay": self.delay,
                "kind": self.kind, "text": self.text}


@dataclass
class Scenario:
    slug: str
    name: str
    path: Path | None = None
    spawn: dict = field(default_factory=dict)
    motion: dict = field(default_factory=dict)
    speech: dict = field(default_factory=dict)
    embodiment: dict = field(default_factory=dict)
    map_spec: dict = field(default_factory=dict)
    pois: list[dict] = field(default_factory=list)
    injections: list[Injection] = field(default_factory=list)
    expect: dict = field(default_factory=dict)
    weights: dict = field(default_factory=dict)
    raw: dict = field(default_factory=dict)

    # ---- construction --------------------------------------------------

    @classmethod
    def from_dict(cls, data: dict, slug: str = "", path: Path | None = None) -> "Scenario":
        return cls(
            slug=slug or data.get("slug") or (path.stem if path else "scenario"),
            name=data.get("name") or slug or "scenario",
            path=path,
            spawn=data.get("spawn") or {},
            motion=data.get("motion") or {},
            speech=data.get("speech") or {},
            embodiment=data.get("embodiment") or {},
            map_spec=data.get("map") or {},
            pois=list(data.get("pois") or []),
            injections=[Injection(at=(float(item["at"]) if item.get("at") is not None else None),
                                  after_arrival=item.get("after_arrival", ""),
                                  delay=float(item.get("delay", 0.0)),
                                  kind=item.get("kind", "user_message"),
                                  text=item.get("text", ""))
                        for item in (data.get("injections") or [])],
            expect=data.get("expect") or {},
            weights=data.get("weights") or {},
            raw=data,
        )

    @classmethod
    def load(cls, path: str | Path) -> "Scenario":
        import yaml

        path = Path(path)
        with path.open(encoding="utf-8") as handle:
            return cls.from_dict(yaml.safe_load(handle) or {}, slug=path.stem, path=path)

    # ---- derived -------------------------------------------------------

    def bind_map(self, asset) -> None:
        """由场景卡在 load 时注入 —— 只有它知道去哪些目录找地图。"""
        self._map_asset = asset

    @property
    def map_name(self) -> str:
        return self.map_spec if isinstance(self.map_spec, str) else ""

    def grid(self) -> OccupancyGrid:
        asset = getattr(self, "_map_asset", None)
        if asset is not None:
            return asset.grid()
        if isinstance(self.map_spec, str):
            # 引用了一张地图但没绑上 —— 与其悄悄退回一张空图让导览莫名其妙地
            # 走得通，不如直接说清楚。
            raise ValueError(f"场景引用了地图 {self.map_spec!r}，但没有加载到")
        spec = self.map_spec or {}
        resolution = float(spec.get("resolution", 0.05))
        x0, y0, x1, y1 = [float(v) for v in spec.get("bounds", [-5.0, -5.0, 5.0, 5.0])]
        grid = OccupancyGrid(resolution, (x0, y0),
                             max(1, int(round((x1 - x0) / resolution))),
                             max(1, int(round((y1 - y0) / resolution))))
        if spec.get("border", True):
            grid.border(OCCUPIED)
        for wall in spec.get("walls") or []:
            grid.fill_rect(*[float(v) for v in wall], OCCUPIED)
        return grid

    def scene(self) -> dict:
        """The dict `WorldBackend.reset` takes."""
        embodiment = self.embodiment or {}
        return {
            "grid": self.grid(),
            "spawn": self.spawn or {"x": 0.0, "y": 0.0, "yaw": 0.0},
            "motion": self.motion or {},
            "dof": int(embodiment.get("dof", 0)),
            "joint_speed": float(embodiment.get("joint_speed", 1.0)),
            "scan": self.raw.get("scan") or {},
        }

    def waypoints(self) -> list[dict]:
        if self.pois:
            return [dict(poi) for poi in self.pois]
        asset = getattr(self, "_map_asset", None)
        return [dict(poi) for poi in (asset.pois if asset else [])]

    def waypoint(self, name: str) -> dict | None:
        return next((poi for poi in self.waypoints() if poi.get("name") == name), None)

    def validate(self) -> list[str]:
        """Warnings, not errors — chiefly: which waypoint pairs are unreachable.

        判据是「**规划得出来吗**」，不是「直线通不通」。早先没有规划器时用的是
        后者，而真地图一上来就否掉了它：北京 2F 的 P3→P15 十二段里有五段直线穿墙，
        展厅里本来就有展墙。现在有了 A*，该报的是真正到不了的点 —— 比如被墙围死、
        或落在未探索区之外的点位。
        """
        from simulator.generic.backend.planner import GridPlanner

        warnings: list[str] = []
        grid = self.grid()
        radius = float((self.motion or {}).get("radius", 0.25))
        planner = GridPlanner(grid, radius)
        spawn = self.spawn or {}
        points = [("起点", float(spawn.get("x", 0.0)), float(spawn.get("y", 0.0)))]
        points += [(poi.get("name", "?"), float(poi["x"]), float(poi["y"]))
                   for poi in self.waypoints()]

        for name, x, y in points[1:]:
            if grid.is_occupied(x, y):
                warnings.append(f"航点 {name} 落在障碍物里")
        # 只查导览真的会走的那些段。全连通检查在一张 14 个点的真实地图上要 91 次
        # 规划，而其中绝大多数段这趟导览根本不走。
        order = list((self.expect or {}).get("waypoint_order") or [])
        legs = list(zip(order, order[1:])) if order else []
        if order:
            legs.insert(0, (points[0][0], order[0]))
        by_name = {n: (x, y) for n, x, y in points}
        for a, b in legs:
            if a not in by_name or b not in by_name:
                continue
            if planner.plan(by_name[a], by_name[b]) is None:
                warnings.append(f"{a} → {b} 规划不出路径；这一段导览走不通")
        return warnings

    def summary(self) -> dict:
        # `waypoints()`，不是 `self.pois` —— 后者只有场景自己声明的那些，而从地图资产
        # 建出来的场景一个都没有（航点在资产里）。两者在同一个类里对「航点」给出不同
        # 答案，而送出去的是错的那个：Orin6 上热加一张图、载入成功，返回里却写着
        # `waypoints: []`，读的人会以为这张图没有点位 —— 其实世界已经拿到了。
        return {
            "slug": self.slug, "name": self.name,
            "waypoints": [poi.get("name") for poi in self.waypoints()],
            "injections": [item.as_dict() for item in self.injections],
            "expect": dict(self.expect),
            "weights": dict(self.weights),
        }


def discover(*directories: str | Path) -> dict[str, Scenario]:
    """Every scenario under the given directories, keyed by slug.

    Later directories win, so a bind-mounted `scenarios/user/` can shadow one
    baked into the image — which is what lets a rig get a new scenario without a
    rebuild. A file that fails to parse is skipped with a log line rather than
    taking the whole card down with it.
    """
    found: dict[str, Scenario] = {}
    for directory in directories:
        base = Path(directory)
        if not base.is_dir():
            continue
        for path in sorted(base.rglob("*")):
            if path.suffix.lower() not in SCENARIO_SUFFIXES:
                continue
            try:
                scenario = Scenario.load(path)
            except Exception as exc:
                print(f"[sim-scenario] skipping {path}: {exc}", flush=True)
                continue
            found[scenario.slug] = scenario
    return found
