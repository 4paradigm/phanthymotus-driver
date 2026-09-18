"""Scenario definitions: the YAML is the single source of truth.

The same file is *run* by the `sim_scenario` card on a rig and *replayed* by
pytest under a fake clock. One definition, two runners, one `assertions.py` —
otherwise the CI green and the rig green stop meaning the same thing.

## The map is rectangles in YAML, not a generated blob

`map.walls` is a list of `[x0, y0, x1, y1]` rectangles rasterised at load time.
A binary occupancy grid committed to the repo would be undiffable, unreviewable
and impossible to tweak without a tool; a list of walls can be read, changed in a
PR, and argued about. `OccupancyGrid.from_dict` still exists for maps captured
off a real robot.
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

    def grid(self) -> OccupancyGrid:
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
        return [dict(poi) for poi in self.pois]

    def waypoint(self, name: str) -> dict | None:
        return next((dict(poi) for poi in self.pois if poi.get("name") == name), None)

    def validate(self) -> list[str]:
        """Warnings, not errors — chiefly: which waypoint pairs are unreachable.

        `LocalBackend` drives straight at the target; there is **no path
        planner**, while the real Slamtec chassis has one. So a map with an
        obstacle between two waypoints produces a leg that fails against
        geometry, and the tour looks broken for a reason that has nothing to do
        with the orchestration being tested.

        Rather than hide that, `do_load` reports it. A scenario that needs a
        route around something is a scenario this backend cannot run yet — which
        is one of the things a Stage 1 collision/planning backend would fix.
        """
        warnings: list[str] = []
        grid = self.grid()
        radius = float((self.motion or {}).get("radius", 0.25))
        spawn = self.spawn or {}
        points = [("起点", float(spawn.get("x", 0.0)), float(spawn.get("y", 0.0)))]
        points += [(poi.get("name", "?"), float(poi["x"]), float(poi["y"])) for poi in self.pois]

        for name, x, y in points[1:]:
            if grid.is_occupied(x, y):
                warnings.append(f"航点 {name} 落在障碍物里")
        for i, (name_a, ax, ay) in enumerate(points):
            for name_b, bx, by in points[i + 1:]:
                if grid.segment_blocked(ax, ay, bx, by, radius):
                    warnings.append(
                        f"{name_a} → {name_b} 直线不通；本后端不做路径规划，若导览用到这一段会撞墙")
        return warnings

    def summary(self) -> dict:
        return {
            "slug": self.slug, "name": self.name,
            "waypoints": [poi.get("name") for poi in self.pois],
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
