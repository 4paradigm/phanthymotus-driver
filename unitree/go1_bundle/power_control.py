"""
power_control.py — Go1 电源控制卡（关机，高风险不可逆）。

自包含：一张卡 = 一个文件。main.py 按 config.yaml 自动 import 并 make_plugin()。
下发经共享 client 的 request_power_off（HighCmd.bms.off=0xA5）。

动作：power_off —— 需 confirm=true + reason；前置:状态反馈正常(fresh) 且 机器人静止。
⚠️ 一经执行关闭电池，驱动侧【不可逆】、不能远程恢复。禁止模型在无明确用户确认时自动调用。
   控制卡须上真机验证后才能上架（CONTRIBUTING.md §4）——关机验证请在可安全断电场景下做。
"""

from __future__ import annotations

import time

CARD = "power_control"
TYPE = "actuator"
CONTROL_LEVEL = "ANY"
DESC = ("Go1 电源:power_off(BmsCmd.off=0xA5)。驱动侧【不可逆】,不能远程恢复;"
        "前置:机器人必须静止、状态反馈正常;需 confirm=true + reason(关机原因)。")


def _ms() -> int:
    return int(time.time() * 1000)


def _ok(action, applied) -> dict:
    return {"ok": True, "card": CARD, "action": action, "control_level": CONTROL_LEVEL,
            "applied": applied, "timestamp_ms": _ms()}


def _err(code, message) -> dict:
    return {"ok": False, "code": code, "message": message}


class Plugin:
    """控制卡插件：关机（confirm + reason + 静止前置）。"""

    def __init__(self, plugin_config, namespace, executor, client):
        self._client = client

    def get_tool(self):
        return {"name": CARD, "type": TYPE, "multiInstance": False, "description": DESC,
                "inputSchema": {
                    "type": "object",
                    "properties": {
                        "action": {"type": "string", "enum": ["power_off"]},
                        "confirm": {"type": "boolean", "description": "必须 true"},
                        "reason": {"type": "string", "description": "关机原因（必填）"},
                    },
                    "required": ["action"],
                    "x-action-params": {
                        "power_off": {"params": ["confirm", "reason"],
                                      "description": "关闭电池（不可逆）。需静止 + confirm + reason。"},
                    },
                }}

    def start(self):
        pass

    def stop(self):
        pass

    def _is_moving(self, snap) -> bool:
        if int(snap.get("mode", 0)) == 2:      # walk
            return True
        vel = snap.get("velocity") or [0.0, 0.0, 0.0]
        try:
            if any(abs(float(v)) > 0.05 for v in vel):
                return True
        except (TypeError, ValueError):
            pass
        try:
            if abs(float(snap.get("yaw_speed", 0.0))) > 0.05:
                return True
        except (TypeError, ValueError):
            pass
        return False

    def dispatch(self, action, args):
        args = args or {}
        if action in ("start",):
            return {"state": "ready"}
        if action in ("stop", "info"):
            return {"state": "idle" if action == "stop" else "running"}
        if action != "power_off":
            return _err("INVALID_ARGUMENT", "unknown action '%s'" % action)
        if not args.get("confirm"):
            return _err("PRECONDITION_FAILED", "power_off requires confirm=true")
        if not args.get("reason"):
            return _err("INVALID_ARGUMENT", "power_off requires 'reason'")
        snap = self._client.snapshot()
        if not snap.get("fresh"):
            return _err("NO_FEEDBACK", "no fresh state; refuse power_off")
        if self._is_moving(snap):
            return _err("PRECONDITION_FAILED", "robot must be static (stop_move/damp) before power_off")
        self._client.request_power_off()
        return _ok("power_off", {"accepted": True, "command_sent_at_ms": _ms(), "reason": args.get("reason")})


def make_plugin(plugin_config, namespace, executor, client):
    """main.py 装配入口。"""
    return Plugin(plugin_config, namespace, executor, client)
