"""地图资产：一张占用栅格 + 它自己的点位。

## 为什么地图和场景是两样东西

真机上 POI 是**按地图存的** —— `controlled_spatial.py` 的 `poi` 表以
`(name, map_name)` 为唯一键。地图和点位是一份资产，场景是在它上面跑的一趟导览：
同一张 `bj-2f` 可以有「完整导览」「只看南区」「低电量返航」好几个场景，而换一个
展厅只需要换地图文件，不必动任何场景逻辑。

早先的设计把地图内联在场景里（`bounds` + `walls` 几个矩形）。那对**自己编的**地图
是对的——一串矩形能读、能在 PR 里改、能争论；但对**从真机抓来的**地图不成立：
北京 2F 是 1543×2198 的栅格，没有任何一组矩形能描述它。两种来源都要支持：

    map: {bounds: [...], walls: [[...]]}   # 合成的，写在场景里
    map: bj-2f                              # 抓来的，从 maps/ 目录解析

## 目录是可扩的，不是固定的

`discover()` 扫若干目录，后面的覆盖前面的 —— 打进镜像的一份打底，bind-mount 进来
的一份可以新增或覆盖。丢一个 json 进 `/opt/phanthy-motus/data/sim/maps`，刷新画布
就能选到，不必重建镜像，也不必改代码。

## 文件格式

```json
{"name": "bj-2f", "resolution": 0.06, "origin": [-1.38, -15.375],
 "width": 385, "height": 549, "data": "<base64(zlib(uint8 cells))>",
 "pois": [{"name": "P3", "x": 6.241, "y": 10.804, "yaw": 1.571, "description": "四个范式"}],
 "source": {...}}
```

`source` 记来源（哪台机器、哪个文件、什么时候、怎么转的）。一张不知道从哪来的
地图，过两个月就没人敢动它。
"""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path

from simulator.generic.geometry import OccupancyGrid

MAP_SUFFIXES = (".json",)


@dataclass
class MapAsset:
    name: str
    grid_spec: dict
    pois: list[dict] = field(default_factory=list)
    source: dict = field(default_factory=dict)
    path: Path | None = None

    @classmethod
    def from_dict(cls, data: dict, name: str = "", path: Path | None = None) -> "MapAsset":
        return cls(
            name=name or data.get("name") or (path.stem if path else "map"),
            grid_spec={k: data[k] for k in ("resolution", "origin", "width", "height", "data")
                       if k in data},
            pois=list(data.get("pois") or []),
            source=dict(data.get("source") or {}),
            path=path,
        )

    @classmethod
    def load(cls, path: str | Path) -> "MapAsset":
        path = Path(path)
        with path.open(encoding="utf-8") as handle:
            return cls.from_dict(json.load(handle), name=path.stem, path=path)

    def grid(self) -> OccupancyGrid:
        return OccupancyGrid.from_dict(self.grid_spec)

    def extent(self) -> dict:
        grid = self.grid()
        return {"resolution": grid.resolution, "origin": list(grid.origin),
                "size": [grid.width, grid.height],
                "metres": [round(grid.width * grid.resolution, 2),
                           round(grid.height * grid.resolution, 2)]}

    def summary(self) -> dict:
        return {"name": self.name, "pois": [p.get("name") for p in self.pois],
                "source": self.source, **self.extent()}

    def to_dict(self) -> dict:
        return {"name": self.name, **self.grid_spec, "pois": self.pois, "source": self.source}

    def save(self, path: str | Path) -> Path:
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open("w", encoding="utf-8") as handle:
            json.dump(self.to_dict(), handle, ensure_ascii=False)
        return path


def discover(*directories: str | Path) -> dict[str, MapAsset]:
    """扫目录里的地图，后面的目录覆盖前面的。

    解析失败的文件跳过并打一行日志，而不是把整张卡带崩 —— 一个坏掉的地图文件
    不该让其它地图也用不了。
    """
    found: dict[str, MapAsset] = {}
    for directory in directories:
        base = Path(directory)
        if not base.is_dir():
            continue
        for path in sorted(base.rglob("*")):
            if path.suffix.lower() not in MAP_SUFFIXES:
                continue
            try:
                asset = MapAsset.load(path)
            except Exception as exc:
                print(f"[sim-maps] skipping {path}: {exc}", flush=True)
                continue
            found[asset.name] = asset
    return found
