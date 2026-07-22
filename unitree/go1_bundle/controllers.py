"""
controllers.py — Go1 运动控制卡合集(actuator)。

合并了以下控制卡（每张卡行为不变）：
  - loco: 基础运动（三维速度 + 站立/姿态/身高/阻尼）
  - body_pose: 机身姿态/高度/抬脚偏移
  - switch_gait: 步态切换
  - gesture: 表演/表情卡
  - special_motion: 官方特殊动作（jump_yaw_left / straight_hand）

所有控制卡通过共享 client 下发 HighCmd，无 ROS2 topic 发布。
"""

from __future__ import annotations

import math
import threading
import time

# ============================================================================
# loco — 基础运动控制（三维速度 + 站立/姿态/身高/阻尼）
# ============================================================================

CARD_LOCO = "loco"
TYPE_LOCO = "actuator"
CONTROL_LEVEL_LOCO = "HIGHLEVEL"
DESC_LOCO = (
    "Go1 基础运动 — 三维速度(vx 前后 m/s / vy 左右平移 m/s / vyaw 偏航 °/s,可组合)+ 停 + "
    "站起/趴下/平衡站立/恢复/阻尼。"
    "move 可选 duration 秒(前端指定执行时间,到点自动停);不填则持续~0.5s自停。"
    "前置:狗须【已站立】。参数越界会被拒绝。(机身姿态角/高度/抬脚见 body_pose 卡)"
)

_TROT = 1
VX_MAX_LOCO, VY_MAX_LOCO = 1.0, 0.8
VYAW_MAX_DEG_LOCO = 90.0
DURATION_MAX_LOCO = 300.0
M_FORCE_STAND_LOCO, M_STAND_DOWN_LOCO, M_STAND_UP_LOCO, M_DAMP_LOCO, M_RECOVERY_LOCO = 1, 5, 6, 7, 8


def _ms_loco():
    return int(time.time() * 1000)


def _err_loco(code, msg):
    return {"ok": False, "code": code, "message": msg}


def _num_loco(v, name):
    try:
        return float(v), None
    except (TypeError, ValueError):
        return None, _err_loco("INVALID_ARGUMENT", "'%s' 必须是数字" % name)


def _rng_loco(v, name, lo, hi):
    f, e = _num_loco(v, name)
    if e:
        return None, e
    if not (lo <= f <= hi):
        return None, _err_loco("INVALID_ARGUMENT", "'%s'=%s 超范围 [%s, %s]" % (name, f, lo, hi))
    return f, None


class LocoPlugin:
    def __init__(self, plugin_config, namespace, executor, client):
        self._client = client
        self._cancel = threading.Event()
        self._thread = None
        self._tlock = threading.Lock()

    def start(self):
        pass

    def stop(self):
        self._cancel_timed()
        try:
            self._client.stop_move()
        except Exception:
            pass

    def _cancel_timed(self):
        with self._tlock:
            th = self._thread
            self._thread = None
        if th and th.is_alive():
            self._cancel.set()
            th.join(timeout=1.0)

    def _timed_move(self, vx, vy, vyaw, duration):
        end = time.monotonic() + duration
        while time.monotonic() < end:
            if self._cancel.is_set():
                return
            self._client.move(vx, vy, vyaw, gait=_TROT)
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
            "name": CARD_LOCO, "type": TYPE_LOCO, "multiInstance": False, "description": DESC_LOCO,
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {"type": "string",
                               "enum": ["move", "stop", "stand_up", "stand_down", "balance_stand",
                                        "recovery_stand", "damp"],
                               "description": "要执行的动作"},
                    "vx": {"type": "number", "description": "前后速度 m/s [-%.1f,%.1f]" % (VX_MAX_LOCO, VX_MAX_LOCO)},
                    "vy": {"type": "number", "description": "左右平移 m/s [-%.1f,%.1f]" % (VY_MAX_LOCO, VY_MAX_LOCO)},
                    "vyaw": {"type": "number", "description": "偏航角速度 °/s [-%.0f,%.0f]" % (VYAW_MAX_DEG_LOCO, VYAW_MAX_DEG_LOCO)},
                    "duration": {"type": "number",
                                 "description": "move 持续秒数(0,%.0f];前端指定执行时间,到点自动停;不填/≤0=持续~0.5s自停" % DURATION_MAX_LOCO},
                },
                "required": ["action"],
                "x-action-params": {
                    "move": {"params": ["vx", "vy", "vyaw", "duration"],
                             "description": "三维速度运动(可组合;vyaw 角度制 °/s);给 duration 秒则到点自动停,不给则持续~0.5s自停"},
                    "stop": {"params": [], "description": "立即停下站稳"},
                    "stand_up": {"params": [], "description": "站起"},
                    "stand_down": {"params": [], "description": "趴下"},
                    "balance_stand": {"params": [], "description": "力控平衡站立"},
                    "recovery_stand": {"params": [], "description": "跌倒后恢复站立"},
                    "damp": {"params": [], "description": "阻尼/软停"},
                },
            },
        }

    def dispatch(self, action, args):
        if action == "start":
            return {"state": "ready"}
        if action == "info":
            return {"ok": True, "card": CARD_LOCO, "action": "info", "control_level": CONTROL_LEVEL_LOCO,
                    "applied": {}, "timestamp_ms": _ms_loco()}
        args = args or {}
        c = self._client
        self._cancel_timed()

        def ok(applied):
            return {"ok": True, "card": CARD_LOCO, "action": action, "control_level": CONTROL_LEVEL_LOCO,
                    "applied": applied, "timestamp_ms": _ms_loco()}

        if action == "stop":
            c.stop_move()
            return ok({"stopped": True})
        if action == "move":
            vx, e = _rng_loco(args.get("vx", 0.0), "vx", -VX_MAX_LOCO, VX_MAX_LOCO)
            if e:
                return e
            vy, e = _rng_loco(args.get("vy", 0.0), "vy", -VY_MAX_LOCO, VY_MAX_LOCO)
            if e:
                return e
            vyaw_deg, e = _rng_loco(args.get("vyaw", 0.0), "vyaw", -VYAW_MAX_DEG_LOCO, VYAW_MAX_DEG_LOCO)
            if e:
                return e
            duration, e = _rng_loco(args.get("duration", 0.0) or 0.0, "duration", 0.0, DURATION_MAX_LOCO)
            if e:
                return e
            vyaw = math.radians(vyaw_deg)
            applied = {"vx": vx, "vy": vy, "vyaw_deg": vyaw_deg, "gait": _TROT}
            if duration > 0:
                self._start_timed(vx, vy, vyaw, duration)
                applied["duration"] = duration
                return ok(applied)
            c.move(vx, vy, vyaw, gait=_TROT)
            return ok(applied)
        if action == "stand_up":
            c.set_posture(M_STAND_UP_LOCO)
            return ok({"mode": M_STAND_UP_LOCO})
        if action == "stand_down":
            c.set_posture(M_STAND_DOWN_LOCO)
            return ok({"mode": M_STAND_DOWN_LOCO})
        if action == "balance_stand":
            c.set_posture(M_FORCE_STAND_LOCO)
            return ok({"mode": M_FORCE_STAND_LOCO})
        if action == "recovery_stand":
            c.set_posture(M_RECOVERY_LOCO)
            return ok({"mode": M_RECOVERY_LOCO})
        if action == "damp":
            c.set_posture(M_DAMP_LOCO)
            return ok({"mode": M_DAMP_LOCO})
        return None


def make_loco(plugin_config, namespace, executor, client):
    return LocoPlugin(plugin_config, namespace, executor, client)


# ============================================================================
# body_pose — 机身姿态 / 高度控制
# ============================================================================

CARD_BODY_POSE = "body_pose"
TYPE_BODY_POSE = "actuator"
CONTROL_LEVEL_BODY_POSE = "HIGHLEVEL"
DESC_BODY_POSE = (
    "Go1 机身姿态/高度（HIGHLEVEL）。站立时调机身姿态角(roll/pitch/yaw)、机身高度偏移、"
    "抬脚高度偏移，或一键复位。高度/抬脚均为【相对默认值的偏移量】。"
    "前置:狗须【已站立】。参数越界会被拒绝。"
)

M_FORCE_STAND_BP = 1
ROLL_MIN_BP, ROLL_MAX_BP = -0.75, 0.75
PITCH_MIN_BP, PITCH_MAX_BP = -0.75, 0.75
YAW_MIN_BP, YAW_MAX_BP = -0.6, 0.6
BODY_H_MIN_BP, BODY_H_MAX_BP = -0.13, 0.03
FOOT_MIN_BP, FOOT_MAX_BP = -0.06, 0.03


def _ms_bp():
    return int(time.time() * 1000)


def _err_bp(code, message):
    return {"ok": False, "code": code, "message": message}


def _num_bp(v, name):
    try:
        return float(v), None
    except (TypeError, ValueError):
        return None, _err_bp("INVALID_ARGUMENT", "'%s' 必须是数字" % name)


def _rng_bp(v, name, lo, hi):
    f, e = _num_bp(v, name)
    if e:
        return None, e
    if not (lo <= f <= hi):
        return None, _err_bp("INVALID_ARGUMENT", "'%s'=%s 超范围 [%s, %s]" % (name, f, lo, hi))
    return f, None


class BodyPosePlugin:
    def __init__(self, plugin_config, namespace, executor, client):
        self._client = client
        self._roll = 0.0
        self._pitch = 0.0
        self._yaw = 0.0
        self._body_height = 0.0
        self._foot_raise = 0.0

    def start(self):
        pass

    def stop(self):
        pass

    def _apply(self):
        self._client.set_posture(
            M_FORCE_STAND_BP,
            euler=(self._roll, self._pitch, self._yaw),
            body_height=self._body_height,
            foot_raise=self._foot_raise,
        )

    def get_tool(self):
        return {
            "name": CARD_BODY_POSE, "type": TYPE_BODY_POSE, "multiInstance": False, "description": DESC_BODY_POSE,
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {"type": "string",
                               "enum": ["set_attitude", "set_body_height",
                                        "set_foot_raise_height", "reset"],
                               "description": "要执行的动作"},
                    "roll_rad": {"type": "number",
                                 "description": "横滚角 rad [%.2f,%.2f]" % (ROLL_MIN_BP, ROLL_MAX_BP)},
                    "pitch_rad": {"type": "number",
                                  "description": "俯仰角 rad [%.2f,%.2f]" % (PITCH_MIN_BP, PITCH_MAX_BP)},
                    "yaw_rad": {"type": "number",
                                "description": "偏航角 rad [%.2f,%.2f]" % (YAW_MIN_BP, YAW_MAX_BP)},
                    "offset_m": {"type": "number",
                                 "description": ("相对默认值的偏移量 m。set_body_height:[%.2f,%.2f];"
                                                 "set_foot_raise_height:[%.2f,%.2f]"
                                                 % (BODY_H_MIN_BP, BODY_H_MAX_BP, FOOT_MIN_BP, FOOT_MAX_BP))},
                },
                "required": ["action"],
                "x-action-params": {
                    "set_attitude": {"params": ["roll_rad", "pitch_rad", "yaw_rad"],
                                     "description": "站立时设机身姿态角(roll/pitch/yaw)"},
                    "set_body_height": {"params": ["offset_m"],
                                        "description": "机身高度偏移(相对默认高度,负=更低)"},
                    "set_foot_raise_height": {"params": ["offset_m"],
                                              "description": "抬脚高度偏移(相对默认抬脚,可正可负)"},
                    "reset": {"params": [], "description": "姿态角/高度偏移/抬脚偏移全部复位为 0"},
                },
            },
        }

    def dispatch(self, action, args):
        if action == "start":
            return {"state": "ready"}
        if action == "stop":
            return {"state": "idle"}
        if action == "info":
            return self._ok("info", self._applied())
        args = args or {}

        if action == "set_attitude":
            roll, e = _rng_bp(args.get("roll_rad", 0.0), "roll_rad", ROLL_MIN_BP, ROLL_MAX_BP)
            if e:
                return e
            pitch, e = _rng_bp(args.get("pitch_rad", 0.0), "pitch_rad", PITCH_MIN_BP, PITCH_MAX_BP)
            if e:
                return e
            yaw, e = _rng_bp(args.get("yaw_rad", 0.0), "yaw_rad", YAW_MIN_BP, YAW_MAX_BP)
            if e:
                return e
            self._roll, self._pitch, self._yaw = roll, pitch, yaw
            self._apply()
            return self._ok(action, {"roll_rad": roll, "pitch_rad": pitch, "yaw_rad": yaw, "mode": M_FORCE_STAND_BP})

        if action == "set_body_height":
            off, e = _rng_bp(args.get("offset_m", 0.0), "offset_m", BODY_H_MIN_BP, BODY_H_MAX_BP)
            if e:
                return e
            self._body_height = off
            self._apply()
            return self._ok(action, {"body_height_offset_m": off})

        if action == "set_foot_raise_height":
            off, e = _rng_bp(args.get("offset_m", 0.0), "offset_m", FOOT_MIN_BP, FOOT_MAX_BP)
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

    def _applied(self):
        return {"roll_rad": self._roll, "pitch_rad": self._pitch, "yaw_rad": self._yaw,
                "body_height_offset_m": self._body_height,
                "foot_raise_height_offset_m": self._foot_raise}

    def _ok(self, action, applied):
        return {"ok": True, "card": CARD_BODY_POSE, "action": action, "control_level": CONTROL_LEVEL_BODY_POSE,
                "applied": applied, "timestamp_ms": _ms_bp()}


def make_body_pose(plugin_config, namespace, executor, client):
    return BodyPosePlugin(plugin_config, namespace, executor, client)


# ============================================================================
# switch_gait — 步态切换
# ============================================================================

CARD_SWITCH_GAIT = "switch_gait"
TYPE_SWITCH_GAIT = "actuator"
CONTROL_LEVEL_SWITCH_GAIT = "HIGHLEVEL"
DESC_SWITCH_GAIT = (
    "Go1 步态切换（HIGHLEVEL，只设期望步态，实际运动由移动卡 loco 触发）。"
    "action 选步态：idle/trot/trot_run/climb_stair/trot_obstacle；"
    "trot_run/climb_stair/trot_obstacle 须 confirm=true。action=info 查当前期望步态。"
)

_GAITS = {
    "idle": (0, False),
    "trot": (1, False),
    "trot_run": (2, True),
    "climb_stair": (3, True),
    "trot_obstacle": (4, True),
}
_TYPE_TO_NAME = {gt: name for name, (gt, _) in _GAITS.items()}


def _ms_sg():
    return int(time.time() * 1000)


def _err_sg(code, message):
    return {"ok": False, "code": code, "message": message}


class SwitchGaitPlugin:
    def __init__(self, plugin_config, namespace, executor, client):
        self._client = client

    def start(self):
        pass

    def stop(self):
        pass

    def get_tool(self):
        return {
            "name": CARD_SWITCH_GAIT, "type": TYPE_SWITCH_GAIT, "multiInstance": False, "description": DESC_SWITCH_GAIT,
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {"type": "string", "enum": list(_GAITS.keys()),
                               "description": "目标步态"},
                    "confirm": {"type": "boolean",
                                "description": "trot_run/climb_stair/trot_obstacle 须显式 true"},
                },
                "required": ["action"],
                "x-action-params": {
                    name: {"params": (["confirm"] if need else []),
                           "description": ("切到 %s%s" % (name, "（须 confirm=true）" if need else ""))}
                    for name, (gt, need) in _GAITS.items()
                },
            },
        }

    def dispatch(self, action, args):
        if action == "start":
            return {"state": "ready"}
        if action == "stop":
            return {"state": "idle"}
        if action == "info":
            g = self._client.desired_gait()
            return {"ok": True, "card": CARD_SWITCH_GAIT, "action": "info", "control_level": CONTROL_LEVEL_SWITCH_GAIT,
                    "applied": {"desired_gait_type": g, "desired_gait": _TYPE_TO_NAME.get(g, "unknown")},
                    "timestamp_ms": _ms_sg()}
        if action in _GAITS:
            return self._set(action, args or {})
        return None

    def _set(self, action, args):
        gait_type, need_confirm = _GAITS[action]
        if need_confirm and not bool(args.get("confirm", False)):
            return _err_sg("PRECONDITION_FAILED",
                           "步态 '%s' 较快/较激进，须显式 confirm=true 才切换" % action)
        applied = self._client.set_gait(gait_type)
        return {"ok": True, "card": CARD_SWITCH_GAIT, "action": action, "control_level": CONTROL_LEVEL_SWITCH_GAIT,
                "applied": {"gait": action, "gait_type": applied,
                            "note": "期望步态已设置；实际运动由移动卡(loco)触发"},
                "timestamp_ms": _ms_sg()}


def make_switch_gait(plugin_config, namespace, executor, client):
    return SwitchGaitPlugin(plugin_config, namespace, executor, client)


# ============================================================================
# gesture — 表演/表情卡
# ============================================================================

CARD_GESTURE = "gesture"
TYPE_GESTURE = "actuator"
CONTROL_LEVEL_GESTURE = "HIGHLEVEL"
DESC_GESTURE = (
    "Go1 表演/表情 — 作揖/点头/摇头/歪头/环视、跳舞/俯卧撑/晃圈、坐下/低姿匍匐/昂首挺立/"
    "回正站立、以及欢迎套餐。全部异步执行、立即返回;stop 可打断。前置:狗须【已站立】。"
)

_FORCE_STAND_G = 1
_TROT_G = 1
_ATT_MAX_G = 0.55
_H_MAX_G = 0.12

_ACTIONS_G = {"greet", "nod", "shake", "head_tilt", "look_around", "dance", "push_up",
              "wobble", "sit", "crouch", "stand_tall", "recover", "welcome"}
_TIMES_ACTIONS_G = {"greet", "nod", "shake", "push_up", "wobble"}


def _ms_g():
    return int(time.time() * 1000)


def _clamp_g(v, lim):
    return max(-lim, min(lim, float(v)))


class GesturePlugin:
    def __init__(self, plugin_config, namespace, executor, client):
        self._client = client
        self._lock = threading.Lock()
        self._cancel = threading.Event()
        self._thread = None
        self._cur = {"roll": 0.0, "pitch": 0.0, "yaw": 0.0, "h": 0.0}

    def start(self):
        pass

    def stop(self):
        self._cancel.set()
        try:
            self._client.stop_move()
            self._client.set_posture(_FORCE_STAND_G)
        except Exception:
            pass

    def _glide(self, pitch=0.0, roll=0.0, yaw=0.0, h=0.0, dur=0.5, hz=50) -> bool:
        tgt = {"roll": _clamp_g(roll, _ATT_MAX_G), "pitch": _clamp_g(pitch, _ATT_MAX_G),
               "yaw": _clamp_g(yaw, _ATT_MAX_G), "h": _clamp_g(h, _H_MAX_G)}
        start = dict(self._cur)
        steps = max(1, int(dur * hz))
        for i in range(1, steps + 1):
            if self._cancel.is_set():
                return False
            t = i / steps
            e = t * t * (3.0 - 2.0 * t)
            r = start["roll"] + (tgt["roll"] - start["roll"]) * e
            p = start["pitch"] + (tgt["pitch"] - start["pitch"]) * e
            y = start["yaw"] + (tgt["yaw"] - start["yaw"]) * e
            hh = start["h"] + (tgt["h"] - start["h"]) * e
            self._client.set_posture(_FORCE_STAND_G, euler=(r, p, y), body_height=hh)
            time.sleep(1.0 / hz)
        self._cur = tgt
        return True

    def _hold(self, seconds: float) -> bool:
        end = time.monotonic() + max(0.0, seconds)
        while time.monotonic() < end:
            if self._cancel.is_set():
                return False
            time.sleep(0.03)
        return True

    def _neutral(self, dur=0.4):
        self._glide(0, 0, 0, 0, dur=dur)

    def _greet(self, times):
        for _ in range(max(1, times)):
            if not self._glide(pitch=0.4, h=-0.05, dur=0.6):
                return
            if not self._hold(0.5):
                return
            if not self._glide(0, 0, 0, 0, dur=0.5):
                return

    def _nod(self, times):
        for _ in range(max(1, times)):
            if not self._glide(h=-0.10, dur=0.28):
                return
            if not self._glide(h=0.06, dur=0.25):
                return

    def _shake(self, times):
        for _ in range(max(1, times)):
            if not self._glide(yaw=0.35, dur=0.22):
                return
            if not self._glide(yaw=-0.35, dur=0.22):
                return

    def _head_tilt(self, side):
        sign = -1.0 if str(side).lower() in ("right", "右") else 1.0
        if self._glide(roll=sign * 0.5, dur=0.6):
            self._hold(1.2)

    def _look_around(self):
        for y in (0.4, -0.4, 0.0):
            if not self._glide(yaw=y, dur=0.8):
                return
            if not self._hold(0.6):
                return

    def _push_up(self, times):
        for _ in range(max(1, times)):
            if not self._glide(h=-0.10, dur=0.6):
                return
            if not self._glide(h=0.10, dur=0.6):
                return

    def _wobble(self, times):
        for _ in range(max(1, times)):
            for p, r in ((0.25, 0.0), (0.0, 0.25), (-0.25, 0.0), (0.0, -0.25)):
                if not self._glide(pitch=p, roll=r, dur=0.4):
                    return

    def _sit(self):
        self._glide(pitch=-0.35, h=-0.12, dur=1.0)
        self._hold(1.0)

    def _crouch(self):
        self._glide(h=-0.12, dur=0.7)
        self._hold(1.5)

    def _stand_tall(self):
        self._glide(pitch=-0.10, h=0.12, dur=0.7)
        self._hold(1.5)

    def _recover(self):
        self._glide(0, 0, 0, 0, dur=0.8)
        self._hold(0.5)

    def _dance(self):
        if not self._glide(pitch=-0.15, h=0.10, dur=0.4):
            return
        for _ in range(2):
            if not self._glide(roll=0.5, yaw=0.25, h=0.05, dur=0.32):
                return
            if not self._glide(roll=-0.5, yaw=-0.25, h=0.05, dur=0.32):
                return
        for _ in range(2):
            if not self._glide(pitch=0.35, h=-0.06, dur=0.22):
                return
            if not self._glide(pitch=-0.15, h=0.08, dur=0.22):
                return
        self._client.move(0.0, 0.0, -0.9, gait=_TROT_G)
        if not self._hold(1.5):
            return
        self._client.stop_move()
        self._cur = {"roll": 0.0, "pitch": 0.0, "yaw": 0.0, "h": 0.0}
        if not self._glide(h=0.12, dur=0.25):
            return
        if not self._glide(h=-0.10, dur=0.25):
            return
        if not self._glide(pitch=0.4, h=-0.05, dur=0.4):
            return
        self._hold(0.3)

    def _welcome(self):
        self._greet(1)
        self._nod(2)

    def _run(self, action, args):
        try:
            if action in _TIMES_ACTIONS_G:
                times = int(args.get("times", 2))
                if action == "greet":
                    self._greet(times)
                elif action == "nod":
                    self._nod(times)
                elif action == "shake":
                    self._shake(times)
                elif action == "push_up":
                    self._push_up(times)
                elif action == "wobble":
                    self._wobble(times)
            elif action == "head_tilt":
                self._head_tilt(args.get("side", "left"))
            elif action == "look_around":
                self._look_around()
            elif action == "sit":
                self._sit()
            elif action == "crouch":
                self._crouch()
            elif action == "stand_tall":
                self._stand_tall()
            elif action == "recover":
                self._recover()
            elif action == "dance":
                self._dance()
            elif action == "welcome":
                self._welcome()
        finally:
            if not self._cancel.is_set() and action not in ("sit", "crouch", "stand_tall", "head_tilt"):
                self._neutral(0.4)
                self._client.set_posture(_FORCE_STAND_G)
                self._cur = {"roll": 0.0, "pitch": 0.0, "yaw": 0.0, "h": 0.0}
            self._lock.release()

    def get_tool(self):
        return {
            "name": CARD_GESTURE, "type": TYPE_GESTURE, "multiInstance": False, "description": DESC_GESTURE,
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {"type": "string",
                               "enum": ["greet", "nod", "shake", "head_tilt", "look_around",
                                        "dance", "push_up", "wobble", "sit", "crouch",
                                        "stand_tall", "recover", "welcome", "stop"],
                               "description": "要执行的动作"},
                    "times": {"type": "integer", "description": "次数(greet/nod/shake/push_up/wobble),默认 2"},
                    "side": {"type": "string", "enum": ["left", "right"], "description": "歪头方向(head_tilt)"},
                },
                "required": ["action"],
                "x-action-params": {
                    "greet": {"params": ["times"], "description": "作揖/打招呼(前倾点头)"},
                    "nod": {"params": ["times"], "description": "点头(上下起伏)"},
                    "shake": {"params": ["times"], "description": "摇头(左右转)"},
                    "head_tilt": {"params": ["side"], "description": "歪头好奇并保持"},
                    "look_around": {"params": [], "description": "环视/四处看"},
                    "dance": {"params": [], "description": "跳舞(摇摆→点头→转圈→蹦跳→谢幕,约6秒)"},
                    "push_up": {"params": ["times"], "description": "俯卧撑(上下起伏)"},
                    "wobble": {"params": ["times"], "description": "晃动画圆"},
                    "sit": {"params": [], "description": "坐下(近似:降低+后仰,保持)"},
                    "crouch": {"params": [], "description": "低姿匍匐(降到最低,保持)"},
                    "stand_tall": {"params": [], "description": "昂首挺立(升高+抬头,保持)"},
                    "recover": {"params": [], "description": "回正/恢复中立站立"},
                    "welcome": {"params": [], "description": "欢迎套餐(作揖+点头)"},
                    "stop": {"params": [], "description": "立即停下、打断进行中的动作、回正站立"},
                },
            },
        }

    def dispatch(self, action, args):
        if action == "start":
            return {"state": "ready"}
        if action == "stop":
            self.stop()
            return {"ok": True, "card": CARD_GESTURE, "action": "stop", "control_level": CONTROL_LEVEL_GESTURE,
                    "applied": {"stopped": True}, "timestamp_ms": _ms_g()}
        if action == "info":
            return {"ok": True, "card": CARD_GESTURE, "action": "info", "control_level": CONTROL_LEVEL_GESTURE,
                    "applied": dict(self._cur), "timestamp_ms": _ms_g()}
        if action not in _ACTIONS_G:
            return {"ok": False, "code": "INVALID_ARGUMENT", "message": "unknown action: %s" % action}
        if not self._lock.acquire(blocking=False):
            return {"ok": False, "code": "PRECONDITION_FAILED",
                    "message": "另一个动作正在执行,请先 stop"}
        self._cancel.clear()
        self._thread = threading.Thread(target=self._run, args=(action, dict(args or {})),
                                        daemon=True, name="gesture-%s" % action)
        self._thread.start()
        return {"ok": True, "card": CARD_GESTURE, "action": action, "control_level": CONTROL_LEVEL_GESTURE,
                "applied": {"async": True}, "timestamp_ms": _ms_g(),
                "note": "已下发,后台执行"}


def make_gesture(plugin_config, namespace, executor, client):
    return GesturePlugin(plugin_config, namespace, executor, client)


# ============================================================================
# special_motion — 官方特殊动作
# ============================================================================

CARD_SPECIAL_MOTION = "special_motion"
TYPE_SPECIAL_MOTION = "actuator"
CONTROL_LEVEL_SPECIAL_MOTION = "HIGHLEVEL"
DESC_SPECIAL_MOTION = (
    "Go1 官方特殊动作 (HIGHLEVEL) — jump_yaw_left / straight_hand。"
    "封装为同步动作序列：先进入 balance_stand(mode=1) 并确认站稳，再 set_posture(mode=10/11)，"
    "保持白名单时长后回到 mode=1。调用会阻塞整个动作过程并返回完整执行结果"
    "（是否站稳/是否进入特殊模式/progress 采样/是否安全回位）。动作时长由 config.yaml 白名单配置，"
    "须在目标 Go1 固件上实机验证。高风险动作须 confirm=true。前置：狗须【已站立】。"
)

M_FORCE_STAND_SM = 1

_SPECIAL_MOTION_MODES = {
    "jump_yaw_left": 10,
    "straight_hand": 11,
}

_DEFAULT_DURATIONS_SM = {
    "jump_yaw_left": 3.0,
    "straight_hand": 4.0,
}


def _ms_sm():
    return int(time.time() * 1000)


def _err_sm(code, message):
    return {"ok": False, "code": code, "message": message}


class SpecialMotionPlugin:
    def __init__(self, plugin_config, namespace, executor, client):
        self._client = client
        self._lock = threading.Lock()
        self._running_seq = None
        self._seq_id = 0

        cfg = plugin_config or {}
        self._durations = {
            "jump_yaw_left": float(cfg.get("jump_yaw_left_duration_s",
                                           _DEFAULT_DURATIONS_SM["jump_yaw_left"])),
            "straight_hand": float(cfg.get("straight_hand_duration_s",
                                           _DEFAULT_DURATIONS_SM["straight_hand"])),
        }
        self._stabilize_timeout = float(cfg.get("stabilize_timeout_s", 3.0))
        self._settle_s = float(cfg.get("stabilize_settle_s", 0.5))

    def start(self):
        pass

    def stop(self):
        try:
            self._client.set_posture(M_FORCE_STAND_SM)
        except Exception:
            pass

    def get_tool(self):
        return {
            "name": CARD_SPECIAL_MOTION, "type": TYPE_SPECIAL_MOTION, "multiInstance": False, "description": DESC_SPECIAL_MOTION,
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {"type": "string",
                               "enum": ["jump_yaw_left", "straight_hand", "stop"],
                               "description": "要执行的动作"},
                    "confirm": {"type": "boolean", "description": "高风险动作二次确认（须 true）"},
                },
                "required": ["action"],
                "x-action-params": {
                    "jump_yaw_left": {"params": ["confirm"],
                                      "description": "左向跳跃偏航，需 confirm=true（阻塞至动作完成）"},
                    "straight_hand": {"params": ["confirm"],
                                      "description": "直立手势，需 confirm=true（阻塞至动作完成）"},
                    "stop": {"params": [],
                             "description": "立即回到平衡站立（打断/兜底）"},
                },
            },
        }

    def dispatch(self, action, args):
        if action == "start":
            return {"state": "ready"}
        if action == "info":
            return {"ok": True, "card": CARD_SPECIAL_MOTION, "action": "info", "control_level": CONTROL_LEVEL_SPECIAL_MOTION,
                    "applied": {"running_seq": self._running_seq}, "timestamp_ms": _ms_sm()}
        if action == "stop":
            self._client.set_posture(M_FORCE_STAND_SM)
            return {"ok": True, "card": CARD_SPECIAL_MOTION, "action": "stop", "control_level": CONTROL_LEVEL_SPECIAL_MOTION,
                    "applied": {"mode": M_FORCE_STAND_SM}, "timestamp_ms": _ms_sm()}

        if action not in _SPECIAL_MOTION_MODES:
            return None

        args = args or {}
        if not args.get("confirm"):
            return _err_sm("PRECONDITION_FAILED", f"{action} requires confirm=true")

        snap = self._client.snapshot()
        if not snap.get("fresh"):
            return _err_sm("NO_FEEDBACK",
                           "no fresh HighState (sport channel not acquired / STUB); refuse special motion")

        with self._lock:
            if self._running_seq is not None:
                return _err_sm("PRECONDITION_FAILED",
                               f"special motion '{self._running_seq}' is already running")
            self._seq_id += 1
            seq_id = self._seq_id
            self._running_seq = action

        try:
            return self._run_sequence(action, _SPECIAL_MOTION_MODES[action],
                                      self._durations[action], seq_id)
        finally:
            with self._lock:
                self._running_seq = None

    def _current_mode(self):
        return self._client.snapshot().get("mode")

    def _current_progress(self) -> float:
        try:
            return round(float(self._client.snapshot().get("progress", 0.0)), 3)
        except Exception:
            return 0.0

    def _wait_mode(self, target: int, timeout: float) -> bool:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self._current_mode() == target:
                return True
            time.sleep(0.05)
        return self._current_mode() == target

    def _hold_and_sample(self, mode_code: int, duration: float) -> list:
        samples = []
        start = time.monotonic()
        deadline = start + duration
        while time.monotonic() < deadline:
            self._client.set_posture(mode_code)
            samples.append([round(time.monotonic() - start, 2), self._current_progress()])
            time.sleep(0.2)
        return samples

    def _run_sequence(self, action: str, mode_code: int,
                      duration: float, seq_id: int) -> dict:
        steps = {}
        client = self._client

        client.set_posture(M_FORCE_STAND_SM)
        stood = self._wait_mode(M_FORCE_STAND_SM, self._stabilize_timeout)
        steps["balance_stand_confirmed"] = stood
        if not stood:
            client.set_posture(M_FORCE_STAND_SM)
            return {
                "ok": False, "code": "PRECONDITION_FAILED",
                "message": (f"robot did not reach balance_stand (mode=1) within "
                            f"{self._stabilize_timeout}s (current mode={self._current_mode()}); "
                            "special motion aborted before firing"),
                "card": CARD_SPECIAL_MOTION, "action": action, "control_level": CONTROL_LEVEL_SPECIAL_MOTION,
                "sequence_id": seq_id, "steps": steps, "timestamp_ms": _ms_sm(),
            }
        time.sleep(self._settle_s)

        client.set_posture(mode_code)
        entered = self._wait_mode(mode_code, min(1.5, self._stabilize_timeout))
        steps["entered_special_mode"] = entered
        samples = self._hold_and_sample(mode_code, duration)
        steps["progress_samples"] = samples
        steps["progress_max"] = max((p for _, p in samples), default=0.0)

        client.set_posture(M_FORCE_STAND_SM)
        returned = self._wait_mode(M_FORCE_STAND_SM, self._stabilize_timeout)
        steps["returned_to_stand"] = returned

        note = None
        if not entered and steps["progress_max"] == 0.0:
            note = ("HighState.mode never showed mode=%d and progress stayed 0 — "
                    "firmware may not have executed this motion (check Legged_sport "
                    "version / robot readiness)." % mode_code)

        return {
            "ok": bool(returned),
            "card": CARD_SPECIAL_MOTION, "action": action, "control_level": CONTROL_LEVEL_SPECIAL_MOTION,
            "sequence_id": seq_id,
            "mode": mode_code,
            "duration_s": duration,
            "state": "completed" if returned else "completed_with_warnings",
            "steps": steps,
            **({"note": note} if note else {}),
            "timestamp_ms": _ms_sm(),
        }


def make_special_motion(plugin_config, namespace, executor, client):
    return SpecialMotionPlugin(plugin_config, namespace, executor, client)
