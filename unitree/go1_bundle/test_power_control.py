"""
test_power_control.py — Go1 电源控制卡（关机，高风险不可逆）。

⚠️ 文件名 test_ 前缀 = **尚未真机验收**（团队约定：未验收卡片加 test 前缀，
   验收通过后去掉前缀 → power_control）。故当前卡名/工具名亦为 test_power_control。

自包含：一张卡 = 一个文件。main.py 按 config.yaml 自动 import 并 make_plugin()。

【关机通道（2026-07-16/17 实测定案）】
  高层 HighCmd.bms.off=0xA5 发给运动服务 Legged_sport(.161:8082) —— **实测收到不执行**
  （同一条高层通道上 loco.move 能驱动狗，说明通道是活的，但 Legged_sport 只管运动、不管电源）。
  故本卡改走**系统关机 poweroff**关掉机器狗主控 Pi（Linux）→ 整机下电。
  ⚠️ 但容器（ros-base 精简镜像）**内部没有 poweroff/systemctl 二进制**，直接 `sudo poweroff`
     会 FileNotFoundError。容器以 --privileged + pid=host 运行，故用 **nsenter 进宿主机
     PID1 的命名空间**执行宿主机的 poweroff（实测容器内 nsenter -t 1 能看到宿主机 /sbin/poweroff）。

动作：power_off —— 需 confirm=true + reason；前置:状态反馈正常(fresh) 且 机器人静止。
⚠️ 一经执行【不可逆】、不能远程恢复（要人到现场按电源键重开）。禁止模型在无明确用户确认时自动调用。
   控制卡须上真机验证后才能上架（CONTRIBUTING.md §4）——关机验证请在可安全断电场景下做。
"""

from __future__ import annotations

import subprocess
import time

CARD = "test_power_control"     # 未验收 → test 前缀；验收通过后改回 power_control（并同步 config/Dockerfile/文件名）
TYPE = "actuator"
CONTROL_LEVEL = "ANY"
DESC = ("Go1 电源:power_off(关主控 Pi→整机下电)。驱动侧【不可逆】,不能远程恢复;"
        "前置:机器人必须静止、状态反馈正常;需 confirm=true + reason(关机原因)。"
        "注:高层 bms.off 已实测关不了机,故走系统关机(nsenter 进宿主机执行 poweroff)。")

# 关机命令：容器内无 poweroff 二进制 → 用 nsenter 进宿主机 PID1 命名空间执行宿主机的 poweroff。
# 依赖：容器 --privileged + pid=host（deploy/service.yml 已配）；nsenter 由 util-linux 提供(镜像自带)。
POWER_OFF_CMD = ["nsenter", "-t", "1", "-m", "-u", "-i", "-n", "-p", "--", "poweroff"]


def _ms() -> int:
    return int(time.time() * 1000)


def _ok(action, applied) -> dict:
    return {"ok": True, "card": CARD, "action": action, "control_level": CONTROL_LEVEL,
            "applied": applied, "timestamp_ms": _ms()}


def _err(code, message) -> dict:
    return {"ok": False, "code": code, "message": message}


class Plugin:
    """控制卡插件：关机（confirm + reason + 静止前置；nsenter 进宿主机执行 poweroff）。"""

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
                                      "description": "关机（poweroff 整机下电，不可逆）。需静止 + confirm + reason。"},
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
        # 触发系统关机；poweroff 会终止本进程，故先把成功包络算好再发命令。
        applied = {"accepted": True, "command": " ".join(POWER_OFF_CMD),
                   "command_sent_at_ms": _ms(), "reason": args.get("reason")}
        try:
            # 不阻塞等待（关机会杀掉自己）；仅捕获"命令本身无法启动"（如 sudo 不可用）。
            subprocess.Popen(POWER_OFF_CMD)
        except Exception as e:  # noqa: BLE001
            return _err("EXEC_FAILED", "poweroff failed to launch: %s" % e)
        return _ok("power_off", applied)


def make_plugin(plugin_config, namespace, executor, client):
    """main.py 装配入口。"""
    return Plugin(plugin_config, namespace, executor, client)
