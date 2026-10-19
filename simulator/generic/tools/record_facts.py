#!/usr/bin/env python3
"""把一趟导览录成事实流，给 agent-core 的裁判当夹具。

裁判在 agent-core（`benchmark_case.py`），事实在这里。两个仓之间不抽共享包 —— 和
`motus.vla/1` 的所有权模型一样，规格是一份文档，两侧各自实现、各自跑契约测试。这个
脚本产出的就是那份契约的样本：一次真实重放的事件流、播报记录和 ACP 上报。

    python3 simulator/generic/tools/record_facts.py > facts.json
    # 落到 phanthymotus/agent-core/tests/fixtures/exhibition_facts.json

事实流的形状变了，就该重跑这个脚本并提交新夹具 —— agent-core 侧的判定测试会因此
失败，而那正是要的：形状变了两边就得一起改。
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
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
DT = 0.05


def record(slug: str = "exhibition_tour", resume: bool = True,
           announce: bool = True, seconds: float = 600.0) -> dict:
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
    scenario_card.dispatch("load", {"scenario": slug})

    ScriptedGuide(nav, tts, Scenario.load(SCENARIO_DIR / f"{slug}.yaml"),
                  ["入口", "一号展区", "二号展区", "三号展区"],
                  resume=resume, announce=announce)
    scenario_card.dispatch("run", {})
    for _ in range(int(round(seconds / DT))):
        clock.advance(DT)
        world.step(DT)

    facts = report.report()
    acp.set_transport(None)
    acp.clear_observers()
    return facts


if __name__ == "__main__":
    print(json.dumps(record(), ensure_ascii=False, indent=1))
