"""
spin.py — Go1 原地转圈（闭环）控制卡。

自包含：一张卡 = 一个文件。main.py 按 config.yaml 里的卡名自动 import 并 make_plugin()。
本卡是 go1_bundle 里第一张**控制卡**：读共享 client 的 snapshot()["imu"]["rpy_rad"]（yaw）
做闭环累计，通过 client.move() 下发原地偏航速度，转够目标角自停。异步、可被 stop 打断。

前置：狗必须【已站立】（软件无法从地面扶起，需遥控扶起）。
安全：move 带 0.5s 看门狗（宏线程停发即自停）；宏另设 max_time 兜底防 yaw 读不到时空转。
⚠️ 控制卡须上真机验证量程+安全后才能上架（见 CONTRIBUTING.md §4）。
依赖 go1_sdk_client 的下发原语 move()/stop_move()。
"""

from __future__ import annotations

import math
import threading
import time

# ── 卡片元数据 ────────────────────────────────────────────────────────────────
CARD = "spin"                    # 卡名 = MCP 工具名 = config.yaml key = 本文件名
TYPE = "actuator"
CONTROL_LEVEL = "HIGHLEVEL"
NODE = "go1_spin"                # 预留（本卡不发 topic）
DESC = ("Go1 原地转圈（闭环，读 IMU yaw 转够自停）。action=spin：转 degrees 度"
        "（direction=left 左转 / right 右转，默认 left；360=转一圈；异步、立即返回）；"
        "action=stop：立即停下站稳、打断进行中的转圈。前置：狗须【已站立】。")

TROT = 1                         # gaitType: trot（转圈需行走步态）
DEFAULT_RATE = 0.6               # 默认偏航角速度 rad/s
RATE_MIN, RATE_MAX = 0.1, 1.5
DEG_MIN, DEG_MAX = 0.0, 3600.0


def _yaw(client) -> float:
    """从共享 snapshot 取 IMU yaw（rad）；无数据返回 0.0。"""
    imu = client.snapshot().get("imu") or {}
    return float(imu.get("rpy_rad", [0.0, 0.0, 0.0])[2])


def _ms() -> int:
    return int(time.time() * 1000)


def _err(code: str, message: str) -> dict:
    return {"ok": False, "code": code, "message": message}


class Plugin:
    """控制卡：异步转圈宏（单后台线程，可被 stop 打断）。"""

    def __init__(self, plugin_config, namespace, executor, client):
        self._client = client
        self._cancel = threading.Event()
        self._thread = None
        self._status = {"state": "idle", "progress_rad": 0.0, "target_rad": 0.0}

    # ── 生命周期 ──
    def start(self):
        pass

    def stop(self):
        self._cancel.set()
        try:
            self._client.stop_move()
        except Exception:  # noqa: BLE001
            pass

    # ── 工具声明 ──
    def get_tool(self):
        return {
            "name": CARD, "type": TYPE, "multiInstance": False, "description": DESC,
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {"type": "string", "enum": ["spin", "stop"], "description": "要执行的动作"},
                    "degrees": {"type": "number", "description": "spin：转动角度，(0, 3600]，默认 360"},
                    "direction": {"type": "string", "enum": ["left", "right"],
                                  "description": "spin：方向，left=左转(+) / right=右转(-)，默认 left"},
                    "yaw_rate": {"type": "number", "description": "spin：偏航角速度 rad/s，[0.1, 1.5]，默认 0.6"},
                },
                "required": ["action"],
                "x-action-params": {
                    "spin": {"params": ["degrees", "direction", "yaw_rate"],
                             "description": "原地转 degrees 度后闭环自停。异步，立即返回；stop 或 damp 可打断。"},
                    "stop": {"params": [], "description": "立即停下站稳，打断进行中的转圈。"},
                },
            },
        }

    # ── 分发（返回 plain dict；未知 action 返回 None）──
    def dispatch(self, action, args):
        if action == "start":
            return {"state": "ready"}
        if action == "stop":
            self._cancel.set()
            self._client.stop_move()
            self._status["state"] = "idle"
            return {"ok": True, "card": CARD, "action": "stop", "control_level": CONTROL_LEVEL,
                    "applied": {"stopped": True}, "timestamp_ms": _ms()}
        if action == "info":
            return {"ok": True, "card": CARD, "action": "info", "control_level": CONTROL_LEVEL,
                    "applied": dict(self._status), "timestamp_ms": _ms()}
        if action == "spin":
            return self._spin(args or {})
        return None

    # ── 转圈（参数校验 → 起异步宏）──
    def _spin(self, args):
        deg = args.get("degrees", 360.0)
        try:
            deg = float(deg)
        except (TypeError, ValueError):
            return _err("INVALID_ARGUMENT", "'degrees' 必须是数字")
        if not (DEG_MIN < deg <= DEG_MAX):
            return _err("INVALID_ARGUMENT", "'degrees'=%s 超范围 (0, %s]" % (deg, DEG_MAX))

        direction = str(args.get("direction", "left")).lower()
        if direction not in ("left", "right"):
            return _err("INVALID_ARGUMENT", "'direction'=%s 须为 left/right" % direction)

        rate = args.get("yaw_rate", DEFAULT_RATE)
        try:
            rate = float(rate)
        except (TypeError, ValueError):
            return _err("INVALID_ARGUMENT", "'yaw_rate' 必须是数字")
        if not (RATE_MIN <= rate <= RATE_MAX):
            return _err("INVALID_ARGUMENT", "'yaw_rate'=%s 超范围 [%s, %s]" % (rate, RATE_MIN, RATE_MAX))

        sgn = 1.0 if direction == "left" else -1.0
        target_rad = math.radians(deg)
        signed_rate = rate * sgn

        # 打断上一个未结束的转圈，再起新的
        self._cancel.set()
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=1.0)
        self._cancel = threading.Event()
        self._status = {"state": "spinning", "progress_rad": 0.0, "target_rad": round(target_rad, 3)}
        self._thread = threading.Thread(target=self._run, args=(target_rad, signed_rate),
                                        daemon=True, name="go1_spin_macro")
        self._thread.start()
        return {"ok": True, "card": CARD, "action": "spin", "control_level": CONTROL_LEVEL,
                "applied": {"degrees": deg, "direction": direction, "yaw_rate_rad_s": round(rate, 3),
                            "target_rad": round(target_rad, 3)},
                "timestamp_ms": _ms()}

    def _run(self, target_rad, signed_rate):
        """闭环宏：signed 累计 yaw 增量（噪声正负抵消），转够 target 或被 cancel/超时即停。"""
        client = self._client
        cancel = self._cancel
        # max_time 兜底：理论时长×2 + 3s，防 yaw 读不到（如 STUB/掉线）时无限空转
        max_time = abs(target_rad / signed_rate) * 2.0 + 3.0
        prev = _yaw(client)
        acc = 0.0
        t0 = time.monotonic()
        try:
            while not cancel.is_set() and (time.monotonic() - t0) < max_time:
                if abs(acc) >= target_rad:
                    break
                client.move(0.0, 0.0, signed_rate, gait=TROT)
                time.sleep(0.05)
                cur = _yaw(client)
                d = cur - prev
                if d > math.pi:
                    d -= 2 * math.pi
                elif d < -math.pi:
                    d += 2 * math.pi
                acc += d
                prev = cur
                self._status["progress_rad"] = round(abs(acc), 3)
        finally:
            client.stop_move()
            self._status["state"] = "idle"


def make_plugin(plugin_config, namespace, executor, client):
    """main.py 装配入口。"""
    return Plugin(plugin_config, namespace, executor, client)
