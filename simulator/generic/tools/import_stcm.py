#!/usr/bin/env python3
"""把 Slamtec 的 `.stcm` 地图 + 驱动的 POI 库转成一份仿真地图资产。

    python3 -m simulator.generic.tools.import_stcm \\
        --stcm controlled_bj-2f.stcm --db controlled_spatial.db \\
        --map-name bj-2f --out simulator/generic/maps/bj-2f.json

在机器人上这两样东西都在（只读即可）：

    /opt/phanthy-motus/data/maps/controlled_<name>.stcm
    /opt/phanthy-motus/data/controlled_spatial.db     # poi 表，按 map_name 分

## 为什么要有这个脚本，而不是转一次就算了

一张不知道从哪来、也没法再生成一遍的地图，过两个月就没人敢动。脚本把来源固定
下来，导出的文件里也记着 `source`。

## 格值语义，以及一个踩过的坑

`controlled_spatial_map.py` 的解码器把「非 0 且非 127」全当 feature 画出来 —— 那是
**给渲染用的**。照搬去做碰撞会把整条走廊判成墙：实测 14 个 POI 里 8 个"落在障碍
里"、12 段导览有 11 段"不通"。看完整直方图才看清楚：

    0            未知（未探索）
    10..120      置信度梯度，每档一两万到四万格，都是「已探索但不如 127 确定」
    127          空地
    >127         真墙，只占约 1%（墙本来就薄）

所以阈值是 **`> 127`**。按这个重算，没有 POI 落在墙里。

## 降采样

原图 0.015 m/格（340 万格）。这个后端的射线投射和占用扫描都是纯 Python，扛不住；
默认降到 0.06 m。降采样里**障碍优先**：一个块里只要有一格是墙，降完就是墙 —— 宁可
让通道窄一点，也不要把墙磨没了让机器人穿过去。
"""

from __future__ import annotations

import argparse
import base64
import json
import sqlite3
import struct
import time
import zlib
from pathlib import Path

FREE = 0
OCCUPIED = 100
WALL_THRESHOLD = 127          # 严格大于此值才是墙，见模块文档


def read_stcm(path: Path) -> dict:
    raw = path.read_bytes()
    if not raw.startswith(b"STCM"):
        raise ValueError(f"{path} 不是 STCM 地图")
    meta = raw[:min(len(raw), 65536)]
    if b"vnd.slamtec.map-layer/vnd.grid-map+binary" not in meta:
        raise ValueError("STCM 里没有 grid-map 层")

    def prop(name: str) -> str:
        offset = meta.find(name.encode("ascii"))
        if offset < 0:
            raise ValueError(f"STCM 缺少属性: {name}")
        at = offset + len(name)
        (length,) = struct.unpack_from("<H", meta, at)
        return meta[at + 2:at + 2 + length].decode("ascii")

    width, height = int(prop("dimension_width")), int(prop("dimension_height"))
    resolution = float(prop("resolution_x"))
    origin = (float(prop("origin_x")), float(prop("origin_y")))
    count = width * height
    if count > len(raw):
        raise ValueError("STCM 栅格长度超出文件大小")
    return {"width": width, "height": height, "resolution": resolution,
            "origin": origin, "cells": raw[len(raw) - count:]}


def downsample(stcm: dict, factor: int) -> dict:
    src, W, H = stcm["cells"], stcm["width"], stcm["height"]
    w, h = W // factor, H // factor
    out = bytearray(w * h)
    for ty in range(h):
        base = ty * factor * W
        row = ty * w
        for tx in range(w):
            left = base + tx * factor
            wall = False
            for dy in range(factor):
                start = left + dy * W
                if any(v > WALL_THRESHOLD for v in src[start:start + factor]):
                    wall = True
                    break
            out[row + tx] = OCCUPIED if wall else FREE
    return {"width": w, "height": h, "resolution": stcm["resolution"] * factor,
            "origin": stcm["origin"], "cells": out}


def read_pois(db_path: Path, map_name: str) -> list[dict]:
    conn = sqlite3.connect(f"file:{db_path}?mode=ro", uri=True)
    conn.row_factory = sqlite3.Row
    rows = conn.execute(
        "SELECT name, description, x, y, yaw FROM poi WHERE map_name = ? ORDER BY name",
        (map_name,)).fetchall()
    conn.close()
    return [{"name": r["name"], "x": round(r["x"], 4), "y": round(r["y"], 4),
             "yaw": round(r["yaw"], 4), "description": r["description"] or ""}
            for r in rows]


def build(stcm_path: Path, db_path: Path | None, map_name: str, factor: int) -> dict:
    grid = downsample(read_stcm(stcm_path), factor)
    pois = read_pois(db_path, map_name) if db_path else []
    occupied = sum(1 for c in grid["cells"] if c == OCCUPIED)
    return {
        "name": map_name,
        "resolution": round(grid["resolution"], 6),
        "origin": [round(grid["origin"][0], 4), round(grid["origin"][1], 4)],
        "width": grid["width"], "height": grid["height"],
        "data": base64.b64encode(zlib.compress(bytes(grid["cells"]), 9)).decode(),
        "pois": pois,
        "source": {
            "stcm": stcm_path.name, "poi_db": db_path.name if db_path else None,
            "downsample": factor, "wall_threshold": f"> {WALL_THRESHOLD}",
            "occupied_cells": occupied, "imported_at": time.strftime("%Y-%m-%d"),
            "note": "格值 >127 才是墙；10..120 是置信度梯度，不是障碍",
        },
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--stcm", required=True, type=Path)
    parser.add_argument("--db", type=Path, help="controlled_spatial.db，用于取 POI")
    parser.add_argument("--map-name", required=True)
    parser.add_argument("--factor", type=int, default=4, help="降采样倍数，默认 4（0.015→0.06m）")
    parser.add_argument("--out", required=True, type=Path)
    args = parser.parse_args()

    asset = build(args.stcm, args.db, args.map_name, args.factor)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(asset, ensure_ascii=False), encoding="utf-8")
    print(f"{args.out}  {asset['width']}x{asset['height']} @{asset['resolution']}m  "
          f"墙格 {asset['source']['occupied_cells']}  POI {len(asset['pois'])} 个  "
          f"({args.out.stat().st_size // 1024} KB)")


if __name__ == "__main__":
    main()
