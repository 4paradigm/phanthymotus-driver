"""
gesture.py — Go1 表演/表情卡(姿态动作,异步执行)。

自包含:一张卡 = 一个文件。用共享 client 的 set_posture()(整机 roll/pitch/yaw + 身高,
mode=1 force_stand)做缓动插值(smoothstep),得到平滑的作揖/点头/坐/伸懒腰等动作;
dance 里的转圈用 client.move()。所有动作在后台线程跑、立即返回 ok(长动作不阻塞平台调用、不超时),
可被 stop 打断。移植自真机验证过的 go1_actions 逻辑。

前置:狗须【已站立】。⚠️ 控制卡须上真机验证幅度+安全后上架。
"""

from __future__ import annotations

import threading
import time

CARD = "gesture"
TYPE = "actuator"
CONTROL_LEVEL = "HIGHLEVEL"
DESC = ("Go1 表演/表情 — 作揖/点头/摇头/歪头/环视、跳舞/俯卧撑/晃圈、坐下/低姿匍匐/昂首挺立/"
        "回正站立、以及欢迎套餐。全部异步执行、立即返回;stop 可打断。前置:狗须【已站立】。")

FORCE_STAND = 1
TROT = 1
_ATT_MAX = 0.55       # roll/pitch/yaw rad 安全上限
_H_MAX = 0.12         # 身高偏移 m 上限

_ACTIONS = {"greet", "nod", "shake", "head_tilt", "look_around", "dance", "push_up",
            "wobble", "sit", "crouch", "stand_tall", "recover", "welcome"}
_TIMES_ACTIONS = {"greet", "nod", "shake", "push_up", "wobble"}


def _ms() -> int:
    return int(time.time() * 1000)


def _clamp(v, lim):
    return max(-lim, min(lim, float(v)))


class Plugin:
    """表演卡:后台单线程跑姿态编排,可被 stop 打断。"""

    def __init__(self, plugin_config, namespace, executor, client):
        self._client = client
        self._lock = threading.Lock()       # 同时只跑一个动作
        self._cancel = threading.Event()
        self._thread = None
        self._cur = {"roll": 0.0, "pitch": 0.0, "yaw": 0.0, "h": 0.0}  # 缓动起点

    def start(self):
        pass

    def stop(self):
        self._cancel.set()
        try:
            self._client.stop_move()
            self._client.set_posture(FORCE_STAND)   # 回正站立
        except Exception:  # noqa: BLE001
            pass

    # ── 缓动 / 保持 ──
    def _glide(self, pitch=0.0, roll=0.0, yaw=0.0, h=0.0, dur=0.5, hz=50) -> bool:
        tgt = {"roll": _clamp(roll, _ATT_MAX), "pitch": _clamp(pitch, _ATT_MAX),
               "yaw": _clamp(yaw, _ATT_MAX), "h": _clamp(h, _H_MAX)}
        start = dict(self._cur)
        steps = max(1, int(dur * hz))
        for i in range(1, steps + 1):
            if self._cancel.is_set():
                return False
            t = i / steps
            e = t * t * (3.0 - 2.0 * t)      # smoothstep
            r = start["roll"] + (tgt["roll"] - start["roll"]) * e
            p = start["pitch"] + (tgt["pitch"] - start["pitch"]) * e
            y = start["yaw"] + (tgt["yaw"] - start["yaw"]) * e
            hh = start["h"] + (tgt["h"] - start["h"]) * e
            self._client.set_posture(FORCE_STAND, euler=(r, p, y), body_height=hh)
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

    # ── 动作序列(移植自 go1_actions)──
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
        self._client.move(0.0, 0.0, -0.9, gait=TROT)   # 转圈
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

    # ── 后台执行 ──
    def _run(self, action, args):
        try:
            if action in _TIMES_ACTIONS:
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
            # sit/crouch/stand_tall/head_tilt 保持姿态;其余回中立站立
            if not self._cancel.is_set() and action not in ("sit", "crouch", "stand_tall", "head_tilt"):
                self._neutral(0.4)
                self._client.set_posture(FORCE_STAND)
                self._cur = {"roll": 0.0, "pitch": 0.0, "yaw": 0.0, "h": 0.0}
            self._lock.release()

    def get_tool(self):
        return {
            "name": CARD, "type": TYPE, "multiInstance": False, "description": DESC,
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {"type": "string",
                               "enum": ["greet", "nod", "shake", "head_tilt", "look_around",
                                        "dance", "push_up", "wobble", "sit", "crouch",
                                        "stand_tall", "recover", "welcome", "stop"],
                               "description": "要执行的动作"},
                    "times": {"type": "integer", "description": "次数(greet/nod/shake/push_up/wobble),默认 2"},
                    "side":  {"type": "string", "enum": ["left", "right"], "description": "歪头方向(head_tilt)"},
                },
                "required": ["action"],
                "x-action-params": {
                    "greet":       {"params": ["times"], "description": "作揖/打招呼(前倾点头)"},
                    "nod":         {"params": ["times"], "description": "点头(上下起伏)"},
                    "shake":       {"params": ["times"], "description": "摇头(左右转)"},
                    "head_tilt":   {"params": ["side"], "description": "歪头好奇并保持"},
                    "look_around": {"params": [], "description": "环视/四处看"},
                    "dance":       {"params": [], "description": "跳舞(摇摆→点头→转圈→蹦跳→谢幕,约6秒)"},
                    "push_up":     {"params": ["times"], "description": "俯卧撑(上下起伏)"},
                    "wobble":      {"params": ["times"], "description": "晃动画圆"},
                    "sit":         {"params": [], "description": "坐下(近似:降低+后仰,保持)"},
                    "crouch":      {"params": [], "description": "低姿匍匐(降到最低,保持)"},
                    "stand_tall":  {"params": [], "description": "昂首挺立(升高+抬头,保持)"},
                    "recover":     {"params": [], "description": "回正/恢复中立站立"},
                    "welcome":     {"params": [], "description": "欢迎套餐(作揖+点头)"},
                    "stop":        {"params": [], "description": "立即停下、打断进行中的动作、回正站立"},
                },
            },
        }

    def dispatch(self, action, args):
        if action == "start":
            return {"state": "ready"}
        if action == "stop":
            self.stop()
            return {"ok": True, "card": CARD, "action": "stop", "control_level": CONTROL_LEVEL,
                    "applied": {"stopped": True}, "timestamp_ms": _ms()}
        if action == "info":
            return {"ok": True, "card": CARD, "action": "info", "control_level": CONTROL_LEVEL,
                    "applied": dict(self._cur), "timestamp_ms": _ms()}
        if action not in _ACTIONS:
            return {"ok": False, "code": "INVALID_ARGUMENT", "message": "unknown action: %s" % action}
        # 抢锁(同时只跑一个);抢不到说明有动作在跑
        if not self._lock.acquire(blocking=False):
            return {"ok": False, "code": "PRECONDITION_FAILED",
                    "message": "另一个动作正在执行,请先 stop"}
        self._cancel.clear()
        self._thread = threading.Thread(target=self._run, args=(action, dict(args or {})),
                                        daemon=True, name="gesture-%s" % action)
        self._thread.start()
        return {"ok": True, "card": CARD, "action": action, "control_level": CONTROL_LEVEL,
                "applied": {"async": True}, "timestamp_ms": _ms(),
                "note": "已下发,后台执行"}


def make_plugin(plugin_config, namespace, executor, client):
    return Plugin(plugin_config, namespace, executor, client)
