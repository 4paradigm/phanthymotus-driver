"""
loco.py — Go1 基础运动控制卡(三维速度 + 站立/姿态/身高)。

自包含:一张卡 = 一个文件。速度用共享 client 的 move()/stop_move();站立/姿态/身高用
共享 client 的 set_posture()(mode=1 force_stand + euler + bodyHeight)。不改动任何现有逻辑。
核心:move 同时给 vx/vy/vyaw 做合成运动(已真机验证)。

前置:狗须【已站立】(高层无法从地面扶起,需遥控扶起)。
⚠️ 控制卡须上真机验证量程+安全后上架(见 CONTRIBUTING.md)。
"""

from __future__ import annotations

import time

CARD = "loco"
TYPE = "actuator"
CONTROL_LEVEL = "HIGHLEVEL"
DESC = ("Go1 基础运动 — 三维速度(vx 前后/vy 左右平移/vyaw 偏航,可组合)+ 停 + "
        "站起/趴下/平衡站立/恢复/阻尼 + 机身姿态角(roll/pitch/yaw)/身高/抬脚/复位。"
        "move 为持续命令(停发约0.5s自停)。前置:狗须【已站立】。参数越界会被拒绝。")

TROT = 1
VX_MAX, VY_MAX, VYAW_MAX = 1.0, 0.6, 2.0
ROLL_MAX = PITCH_MAX = 0.5
YAW_MAX = 0.5
DZ_MAX = 0.12
FOOT_MAX = 0.12
# mode(Go1 HighCmd)
M_FORCE_STAND, M_STAND_DOWN, M_STAND_UP, M_DAMP, M_RECOVERY = 1, 5, 6, 7, 8


def _ms() -> int:
    return int(time.time() * 1000)


def _err(code, msg) -> dict:
    return {"ok": False, "code": code, "message": msg}


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
    def __init__(self, plugin_config, namespace, executor, client):
        self._client = client

    def start(self):
        pass

    def stop(self):
        try:
            self._client.stop_move()
        except Exception:  # noqa: BLE001
            pass

    def get_tool(self):
        return {
            "name": CARD, "type": TYPE, "multiInstance": False, "description": DESC,
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {"type": "string",
                               "enum": ["move", "stop", "stand_up", "stand_down", "balance_stand",
                                        "recovery_stand", "damp", "set_attitude", "body_height",
                                        "foot_raise", "reset"],
                               "description": "要执行的动作"},
                    "vx":   {"type": "number", "description": "前后速度 m/s [-%.1f,%.1f]" % (VX_MAX, VX_MAX)},
                    "vy":   {"type": "number", "description": "左右平移 m/s [-%.1f,%.1f]" % (VY_MAX, VY_MAX)},
                    "vyaw": {"type": "number", "description": "偏航 rad/s [-%.1f,%.1f]" % (VYAW_MAX, VYAW_MAX)},
                    "roll":  {"type": "number", "description": "横滚角 rad [-%.1f,%.1f]" % (ROLL_MAX, ROLL_MAX)},
                    "pitch": {"type": "number", "description": "俯仰角 rad [-%.1f,%.1f]" % (PITCH_MAX, PITCH_MAX)},
                    "yaw":   {"type": "number", "description": "偏航角 rad [-%.1f,%.1f]" % (YAW_MAX, YAW_MAX)},
                    "dz":    {"type": "number", "description": "身高偏移 m [-%.2f,%.2f]" % (DZ_MAX, DZ_MAX)},
                    "h":     {"type": "number", "description": "抬脚高度 m [0,%.2f]" % FOOT_MAX},
                },
                "required": ["action"],
                "x-action-params": {
                    "move":           {"params": ["vx", "vy", "vyaw"], "description": "三维速度运动(可组合),持续~0.5s自停"},
                    "stop":           {"params": [], "description": "立即停下站稳"},
                    "stand_up":       {"params": [], "description": "站起"},
                    "stand_down":     {"params": [], "description": "趴下"},
                    "balance_stand":  {"params": [], "description": "力控平衡站立"},
                    "recovery_stand": {"params": [], "description": "跌倒后恢复站立"},
                    "damp":           {"params": [], "description": "阻尼/软停"},
                    "set_attitude":   {"params": ["roll", "pitch", "yaw"], "description": "站立时设机身姿态角"},
                    "body_height":    {"params": ["dz"], "description": "微调机身高度"},
                    "foot_raise":     {"params": ["h"], "description": "行走抬脚高度"},
                    "reset":          {"params": [], "description": "姿态/高度/抬脚复位"},
                },
            },
        }

    def dispatch(self, action, args):
        if action == "start":
            return {"state": "ready"}
        if action == "info":
            return {"ok": True, "card": CARD, "action": "info", "control_level": CONTROL_LEVEL,
                    "applied": {}, "timestamp_ms": _ms()}
        args = args or {}
        c = self._client

        def ok(applied):
            return {"ok": True, "card": CARD, "action": action, "control_level": CONTROL_LEVEL,
                    "applied": applied, "timestamp_ms": _ms()}

        if action == "stop":
            c.stop_move()
            return ok({"stopped": True})
        if action == "move":
            vx, e = _rng(args.get("vx", 0.0), "vx", -VX_MAX, VX_MAX)
            if e:
                return e
            vy, e = _rng(args.get("vy", 0.0), "vy", -VY_MAX, VY_MAX)
            if e:
                return e
            vyaw, e = _rng(args.get("vyaw", 0.0), "vyaw", -VYAW_MAX, VYAW_MAX)
            if e:
                return e
            return ok(c.move(vx, vy, vyaw, gait=TROT))
        if action == "stand_up":
            c.set_posture(M_STAND_UP)
            return ok({"mode": M_STAND_UP})
        if action == "stand_down":
            c.set_posture(M_STAND_DOWN)
            return ok({"mode": M_STAND_DOWN})
        if action == "balance_stand":
            c.set_posture(M_FORCE_STAND)
            return ok({"mode": M_FORCE_STAND})
        if action == "recovery_stand":
            c.set_posture(M_RECOVERY)
            return ok({"mode": M_RECOVERY})
        if action == "damp":
            c.set_posture(M_DAMP)
            return ok({"mode": M_DAMP})
        if action == "set_attitude":
            roll, e = _rng(args.get("roll", 0.0), "roll", -ROLL_MAX, ROLL_MAX)
            if e:
                return e
            pitch, e = _rng(args.get("pitch", 0.0), "pitch", -PITCH_MAX, PITCH_MAX)
            if e:
                return e
            yaw, e = _rng(args.get("yaw", 0.0), "yaw", -YAW_MAX, YAW_MAX)
            if e:
                return e
            c.set_posture(M_FORCE_STAND, euler=(roll, pitch, yaw))
            return ok({"roll": roll, "pitch": pitch, "yaw": yaw})
        if action == "body_height":
            dz, e = _rng(args.get("dz", 0.0), "dz", -DZ_MAX, DZ_MAX)
            if e:
                return e
            c.set_posture(M_FORCE_STAND, body_height=dz)
            return ok({"dz": dz})
        if action == "foot_raise":
            h, e = _rng(args.get("h", 0.0), "h", 0.0, FOOT_MAX)
            if e:
                return e
            c.set_posture(M_FORCE_STAND, foot_raise=h)
            return ok({"h": h})
        if action == "reset":
            c.set_posture(M_FORCE_STAND, euler=(0.0, 0.0, 0.0), body_height=0.0, foot_raise=0.0)
            return ok({"reset": True})
        return None


def make_plugin(plugin_config, namespace, executor, client):
    return Plugin(plugin_config, namespace, executor, client)
