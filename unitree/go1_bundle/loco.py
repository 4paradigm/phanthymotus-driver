"""
loco.py — Go1 基础运动控制卡(三维速度 + 站立/姿态/身高)。

自包含:一张卡 = 一个文件。速度用共享 client 的 move()/stop_move();站立/姿态/身高用
共享 client 的 set_posture()(mode=1 force_stand + euler + bodyHeight)。不改动任何现有逻辑。
核心:move 同时给 vx/vy/vyaw 做合成运动(已真机验证)。

前置:狗须【已站立】(高层无法从地面扶起,需遥控扶起)。
⚠️ 控制卡须上真机验证量程+安全后上架(见 CONTRIBUTING.md)。
"""

from __future__ import annotations

import threading
import time

CARD = "loco"
TYPE = "actuator"
CONTROL_LEVEL = "HIGHLEVEL"
DESC = ("Go1 基础运动 — 三维速度(vx 前后/vy 左右平移/vyaw 偏航,可组合)+ 停 + "
        "站起/趴下/平衡站立/恢复/阻尼 + 机身姿态角(roll/pitch/yaw)/身高/抬脚/复位。"
        "move 可选 duration 秒(前端指定执行时间,到点自动停);不填则持续~0.5s自停。"
        "前置:狗须【已站立】。参数越界会被拒绝。")

TROT = 1
# 速度上限 = Go1 EDU Sport Mode 出厂默认设计值(超此值固件会自行钳制到设计上限)。
VX_MAX, VY_MAX, VYAW_MAX = 1.5, 0.8, 1.57
DURATION_MAX = 300.0   # move 单次最长持续秒(前端可调执行时间;防误传超大值)
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
        self._cancel = threading.Event()   # duration 重发线程的中断标志
        self._thread = None
        self._tlock = threading.Lock()

    def start(self):
        pass

    def stop(self):
        self._cancel_timed()
        try:
            self._client.stop_move()
        except Exception:  # noqa: BLE001
            pass

    # ── duration:后台按节奏重发 move 刷 0.5s 看门狗,到点自动停 ──────────────────
    def _cancel_timed(self):
        """停掉正在跑的 duration 重发线程(新命令进来先清旧的,避免旧线程盖新动作)。"""
        with self._tlock:
            th = self._thread
            self._thread = None
        if th and th.is_alive():
            self._cancel.set()
            th.join(timeout=1.0)

    def _timed_move(self, vx, vy, vyaw, duration):
        """持续 duration 秒:每 ~0.1s 重发一次 move 刷看门狗,到点 stop_move。"""
        end = time.monotonic() + duration
        while time.monotonic() < end:
            if self._cancel.is_set():
                return                      # 被新命令/stop 打断;由对方接管状态
            self._client.move(vx, vy, vyaw, gait=TROT)
            time.sleep(0.1)
        self._client.stop_move()

    def _start_timed(self, vx, vy, vyaw, duration):
        self._cancel_timed()
        self._cancel.clear()
        th = threading.Thread(target=self._timed_move, args=(vx, vy, vyaw, duration),
                              daemon=True, name="go1_loco_timed_move")
        with self._tlock:
            self._thread = th
        th.start()

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
                    "vyaw": {"type": "number", "description": "偏航 rad/s [-%.2f,%.2f]" % (VYAW_MAX, VYAW_MAX)},
                    "duration": {"type": "number",
                                 "description": "move 持续秒数(0,%.0f];前端指定执行时间,到点自动停;不填/≤0=持续~0.5s自停" % DURATION_MAX},
                    "roll":  {"type": "number", "description": "横滚角 rad [-%.1f,%.1f]" % (ROLL_MAX, ROLL_MAX)},
                    "pitch": {"type": "number", "description": "俯仰角 rad [-%.1f,%.1f]" % (PITCH_MAX, PITCH_MAX)},
                    "yaw":   {"type": "number", "description": "偏航角 rad [-%.1f,%.1f]" % (YAW_MAX, YAW_MAX)},
                    "dz":    {"type": "number", "description": "身高偏移 m [-%.2f,%.2f]" % (DZ_MAX, DZ_MAX)},
                    "h":     {"type": "number", "description": "抬脚高度 m [0,%.2f]" % FOOT_MAX},
                },
                "required": ["action"],
                "x-action-params": {
                    "move":           {"params": ["vx", "vy", "vyaw", "duration"], "description": "三维速度运动(可组合);给 duration 秒则到点自动停,不给则持续~0.5s自停"},
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
        # 任何新的运动/姿态命令进来,先停掉上一条 move(duration) 的重发线程,
        # 否则旧线程会继续刷 move 把新动作盖掉。move(duration) 分支会自己重开。
        self._cancel_timed()

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
            duration, e = _rng(args.get("duration", 0.0) or 0.0, "duration", 0.0, DURATION_MAX)
            if e:
                return e
            if duration > 0:
                self._start_timed(vx, vy, vyaw, duration)   # 后台重发,到点自动停,不阻塞返回
                return ok({"vx": vx, "vy": vy, "vyaw": vyaw, "gait": TROT, "duration": duration})
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
