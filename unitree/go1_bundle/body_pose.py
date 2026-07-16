"""
body_pose.py — Go1 机身姿态 / 高度控制卡（HIGHLEVEL）。

自包含：一张卡 = 一个文件。main.py 按 config.yaml 里的卡名自动 import 并 make_plugin()。
站立姿态下调机身姿态角(roll/pitch/yaw)、机身高度偏移、抬脚高度偏移，或一键复位。
底层走共享 client 的 set_posture(mode=1 force_stand + euler + bodyHeight + footRaiseHeight)，
由唯一的后台线程持续下发；越界一律拒绝，不静默截断。

依据《Go1 动作/运控与感知扩展能力卡片》§4.3：
  set_attitude(roll_rad/pitch_rad/yaw_rad) / set_body_height(offset_m) /
  set_foot_raise_height(offset_m) / reset
高度、抬脚均为【相对默认值的偏移量】，不是绝对高度。

⚠️ 控制卡须上真机验证量程+安全后才能上架（见 CONTRIBUTING.md §4）。
前置：狗须【已站立】；姿态/高度只在站立姿态模式(mode=1)下有效。
"""

from __future__ import annotations

import time

# ── 卡片元数据 ────────────────────────────────────────────────────────────────
CARD = "body_pose"               # 卡名 = MCP 工具名 = config.yaml key = 本文件名
TYPE = "actuator"
CONTROL_LEVEL = "HIGHLEVEL"
NODE = "go1_body_pose"           # 预留（本卡不发 topic）
DESC = ("Go1 机身姿态/高度（HIGHLEVEL）。站立时调机身姿态角(roll/pitch/yaw)、机身高度偏移、"
        "抬脚高度偏移，或一键复位。高度/抬脚均为【相对默认值的偏移量】。"
        "前置:狗须【已站立】。参数越界会被拒绝。")

# 站立姿态模式（Go1 HighCmd mode）——姿态/高度/抬脚只在此模式下有效。
M_FORCE_STAND = 1

# 输入范围（依据能力卡片 §4.3；越界一律拒绝，不静默裁剪）。
ROLL_MIN, ROLL_MAX = -0.75, 0.75            # roll_rad
PITCH_MIN, PITCH_MAX = -0.75, 0.75          # pitch_rad
YAW_MIN, YAW_MAX = -0.6, 0.6                # yaw_rad
BODY_H_MIN, BODY_H_MAX = -0.13, 0.03        # 机身高度偏移 m（相对默认高度）
FOOT_MIN, FOOT_MAX = -0.06, 0.03            # 抬脚高度偏移 m（相对默认抬脚；可负）


def _ms() -> int:
    return int(time.time() * 1000)


def _err(code: str, message: str) -> dict:
    return {"ok": False, "code": code, "message": message}


def _num(v, name):
    try:
        return float(v), None
    except (TypeError, ValueError):
        return None, _err("INVALID_ARGUMENT", "'%s' 必须是数字" % name)


def _rng(v, name, lo, hi):
    f, e = _num(v, name)
    if e:
        return None, e
    if not (lo <= f <= hi):
        return None, _err("INVALID_ARGUMENT", "'%s'=%s 超范围 [%s, %s]" % (name, f, lo, hi))
    return f, None


class Plugin:
    """控制卡：站立姿态下的机身姿态角 / 高度偏移 / 抬脚偏移。

    维护累积姿态(roll/pitch/yaw/body_height/foot_raise)，每次动作只改对应字段后下发完整
    姿态——这样 set_body_height 不会把之前设的姿态角清零；reset 才把全部归零。
    """

    def __init__(self, plugin_config, namespace, executor, client):
        self._client = client
        self._roll = 0.0
        self._pitch = 0.0
        self._yaw = 0.0
        self._body_height = 0.0
        self._foot_raise = 0.0

    # ── 生命周期 ──
    def start(self):
        pass

    def stop(self):
        pass

    # ── 下发当前累积姿态（mode=1 force_stand）──
    def _apply(self):
        self._client.set_posture(
            M_FORCE_STAND,
            euler=(self._roll, self._pitch, self._yaw),
            body_height=self._body_height,
            foot_raise=self._foot_raise,
        )

    # ── 工具声明 ──
    def get_tool(self):
        return {
            "name": CARD, "type": TYPE, "multiInstance": False, "description": DESC,
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {"type": "string",
                               "enum": ["set_attitude", "set_body_height",
                                        "set_foot_raise_height", "reset"],
                               "description": "要执行的动作"},
                    "roll_rad":  {"type": "number",
                                  "description": "横滚角 rad [%.2f,%.2f]" % (ROLL_MIN, ROLL_MAX)},
                    "pitch_rad": {"type": "number",
                                  "description": "俯仰角 rad [%.2f,%.2f]" % (PITCH_MIN, PITCH_MAX)},
                    "yaw_rad":   {"type": "number",
                                  "description": "偏航角 rad [%.2f,%.2f]" % (YAW_MIN, YAW_MAX)},
                    "offset_m":  {"type": "number",
                                  "description": ("相对默认值的偏移量 m。set_body_height:[%.2f,%.2f];"
                                                  "set_foot_raise_height:[%.2f,%.2f]"
                                                  % (BODY_H_MIN, BODY_H_MAX, FOOT_MIN, FOOT_MAX))},
                },
                "required": ["action"],
                "x-action-params": {
                    "set_attitude":          {"params": ["roll_rad", "pitch_rad", "yaw_rad"],
                                              "description": "站立时设机身姿态角(roll/pitch/yaw)"},
                    "set_body_height":       {"params": ["offset_m"],
                                              "description": "机身高度偏移(相对默认高度,负=更低)"},
                    "set_foot_raise_height": {"params": ["offset_m"],
                                              "description": "抬脚高度偏移(相对默认抬脚,可正可负)"},
                    "reset":                 {"params": [],
                                              "description": "姿态角/高度偏移/抬脚偏移全部复位为 0"},
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
            return self._ok("info", self._applied())
        args = args or {}

        if action == "set_attitude":
            roll, e = _rng(args.get("roll_rad", 0.0), "roll_rad", ROLL_MIN, ROLL_MAX)
            if e:
                return e
            pitch, e = _rng(args.get("pitch_rad", 0.0), "pitch_rad", PITCH_MIN, PITCH_MAX)
            if e:
                return e
            yaw, e = _rng(args.get("yaw_rad", 0.0), "yaw_rad", YAW_MIN, YAW_MAX)
            if e:
                return e
            self._roll, self._pitch, self._yaw = roll, pitch, yaw
            self._apply()
            return self._ok(action, {"roll_rad": roll, "pitch_rad": pitch,
                                     "yaw_rad": yaw, "mode": M_FORCE_STAND})

        if action == "set_body_height":
            off, e = _rng(args.get("offset_m", 0.0), "offset_m", BODY_H_MIN, BODY_H_MAX)
            if e:
                return e
            self._body_height = off
            self._apply()
            return self._ok(action, {"body_height_offset_m": off})

        if action == "set_foot_raise_height":
            off, e = _rng(args.get("offset_m", 0.0), "offset_m", FOOT_MIN, FOOT_MAX)
            if e:
                return e
            self._foot_raise = off
            self._apply()
            return self._ok(action, {"foot_raise_height_offset_m": off})

        if action == "reset":
            self._roll = self._pitch = self._yaw = 0.0
            self._body_height = 0.0
            self._foot_raise = 0.0
            self._apply()
            return self._ok(action, self._applied())

        return None

    # ── 规范返回 ──
    def _applied(self) -> dict:
        return {"roll_rad": self._roll, "pitch_rad": self._pitch, "yaw_rad": self._yaw,
                "body_height_offset_m": self._body_height,
                "foot_raise_height_offset_m": self._foot_raise}

    def _ok(self, action, applied) -> dict:
        return {"ok": True, "card": CARD, "action": action, "control_level": CONTROL_LEVEL,
                "applied": applied, "timestamp_ms": _ms()}


def make_plugin(plugin_config, namespace, executor, client):
    """main.py 装配入口。"""
    return Plugin(plugin_config, namespace, executor, client)
