"""
switch_gait.py — Go1 步态切换控制卡（HIGHLEVEL）。

自包含：一张卡 = 一个文件。main.py 按 config.yaml 里的卡名自动 import 并 make_plugin()。
本卡**只设置期望步态**（写共享 client 的常驻 _desired_gait），实际走路仍由移动卡（loco）
触发——移动卡未显式指定步态的 move 会自动采用这里设定的步态；若正在走，切步态会即时生效。

步态（gaitType，对照 comm.h / GAIT_NAMES）：
  idle=0 / trot=1 / trot_run=2 / climb_stair=3 / trot_obstacle=4
其中 trot_run/climb_stair/trot_obstacle 更快或更激进，须显式 confirm=true 才切。

⚠️ 控制卡须上真机验证量程+安全后才能上架（见 CONTRIBUTING.md §4）。依赖 client.set_gait()。
前置：狗须【已站立】；步态只在移动（move）时体现。
"""

from __future__ import annotations

import time

# ── 卡片元数据 ────────────────────────────────────────────────────────────────
CARD = "switch_gait"             # 卡名 = MCP 工具名 = config.yaml key = 本文件名
TYPE = "actuator"
CONTROL_LEVEL = "HIGHLEVEL"
NODE = "go1_switch_gait"         # 预留（本卡不发 topic）
DESC = ("Go1 步态切换（HIGHLEVEL，只设期望步态，实际运动由移动卡 loco 触发）。"
        "action 选步态：idle/trot/trot_run/climb_stair/trot_obstacle；"
        "trot_run/climb_stair/trot_obstacle 须 confirm=true。action=info 查当前期望步态。")

# 步态名 → (gaitType, 是否需 confirm)
GAITS = {
    "idle": (0, False),
    "trot": (1, False),
    "trot_run": (2, True),
    "climb_stair": (3, True),
    "trot_obstacle": (4, True),
}
_TYPE_TO_NAME = {gt: name for name, (gt, _) in GAITS.items()}


def _ms() -> int:
    return int(time.time() * 1000)


def _err(code: str, message: str) -> dict:
    return {"ok": False, "code": code, "message": message}


class Plugin:
    """控制卡：切换共享 client 的常驻期望步态。无异步宏、无 topic。"""

    def __init__(self, plugin_config, namespace, executor, client):
        self._client = client

    # ── 生命周期 ──
    def start(self):
        pass

    def stop(self):
        pass

    # ── 工具声明 ──
    def get_tool(self):
        return {
            "name": CARD, "type": TYPE, "multiInstance": False, "description": DESC,
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {"type": "string", "enum": list(GAITS.keys()),
                               "description": "目标步态"},
                    "confirm": {"type": "boolean",
                                "description": "trot_run/climb_stair/trot_obstacle 须显式 true"},
                },
                "required": ["action"],
                "x-action-params": {
                    name: {"params": (["confirm"] if need else []),
                           "description": ("切到 %s%s" % (name, "（须 confirm=true）" if need else ""))}
                    for name, (gt, need) in GAITS.items()
                },
            },
        }

    # ── 分发（返回 plain dict；未知 action 返回 None）──
    def dispatch(self, action, args):
        if action == "start":
            return {"state": "ready"}
        if action == "stop":
            return {"state": "idle"}
        if action == "info":
            g = self._client.desired_gait()
            return {"ok": True, "card": CARD, "action": "info", "control_level": CONTROL_LEVEL,
                    "applied": {"desired_gait_type": g,
                                "desired_gait": _TYPE_TO_NAME.get(g, "unknown")},
                    "timestamp_ms": _ms()}
        if action in GAITS:
            return self._set(action, args or {})
        return None

    # ── 切步态（confirm 校验 → 写 client 期望步态）──
    def _set(self, action, args):
        gait_type, need_confirm = GAITS[action]
        if need_confirm and not bool(args.get("confirm", False)):
            return _err("PRECONDITION_FAILED",
                        "步态 '%s' 较快/较激进，须显式 confirm=true 才切换" % action)
        applied = self._client.set_gait(gait_type)
        return {"ok": True, "card": CARD, "action": action, "control_level": CONTROL_LEVEL,
                "applied": {"gait": action, "gait_type": applied,
                            "note": "期望步态已设置；实际运动由移动卡(loco)触发"},
                "timestamp_ms": _ms()}


def make_plugin(plugin_config, namespace, executor, client):
    """main.py 装配入口。"""
    return Plugin(plugin_config, namespace, executor, client)
