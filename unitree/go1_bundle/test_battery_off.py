"""
test_battery_off.py — Go1 电池关断卡（低层 BmsCmd.off=0xA5 → 主控板 MCU .10:8007）。

⚠️ 文件名 test_ 前缀 = 关电池极高危+不可逆，虽已真机验收通过一次，仍保守留前缀待复核；
   验收充分后去前缀 → battery_off。

✅ 2026-07-17 真机验收通过：发本卡后狗**灯灭、电机彻底断电**（=按电源键效果），重启恢复正常。
   这是**真·断电池**，区别于 test_power_control 的 nsenter poweroff（只关 Pi、灯还亮、不断电）。
   → 本卡取代 test_power_control 作为 Go1 的关机/断电卡。

【为什么走低层】
  高层 HighCmd.bms.off=0xA5 → Legged_sport(.161:8082) 已实测"收到不执行"（它只管运动）。
  nsenter poweroff → 只关 Pi Linux，电池不断（灯亮、电跑、要现场重启，且站立时会摔）。
  本卡走**低层**：LowCmd.bms.off=0xA5 → 主控板 MCU(192.168.123.10:8007)，SDK comm.h 注释
  明说 "set 0xA5 to turn off the battery"。主控板在狗内部网络（ping 通 RTT 0.19ms，不用插线）。

【实现】
  不动高层常驻 client（不互斥、不抢 8090 端口）。临时建一个 LOWLEVEL(0xFF) UDP
  （本地端口 8091），构造 LowCmd 设 bms.off=0xA5，连发 N 帧（~100ms）确保主控板收到，然后关闭
  临时 UDP。STUB（无 robot_interface）下返回 EXEC_FAILED。

动作：battery_off —— 需 confirm=true + reason；前置:状态反馈正常(fresh) 且 机器人静止。
⚠️ 一经执行【不可逆】、不能远程恢复（断电后要现场按电源键重开）。站立时执行狗会瘫倒。
   禁止模型在无明确用户确认时自动调用。
"""

from __future__ import annotations

import time

CARD = "test_battery_off"       # 未去前缀（极高危待复核）；验收充分后改回 battery_off（同步 config/Dockerfile/文件名）
TYPE = "actuator"
CONTROL_LEVEL = "ANY"
DESC = ("Go1 电池关断:battery_off(低层 BmsCmd.off=0xA5→主控板.10:8007 真断电,已真机验收灯灭断电)。"
        "驱动侧【不可逆】,不能远程恢复;前置:机器人必须静止、状态反馈正常;"
        "需 confirm=true + reason(关机原因)。真·断电池(区别于只关 Pi 的 poweroff)。")

# 低层网络参数（udp.h：UDP_SERVER_IP_BASIC / UDP_SERVER_PORT）
LOWLEVEL = 0xFF                 # comm.h: LOWLEVEL 控制字（高层 0xEE）
LOW_TARGET_IP = "192.168.123.10"  # 主控板 MCU（管电池/电源）
LOW_TARGET_PORT = 8007           # UDP_SERVER_PORT
LOW_LOCAL_PORT = 8091            # 临时本地端口（避开高层 8090）
BMS_OFF_CODE = 0xA5              # comm.h: BmsCmd.off = 0xA5 → turn off the battery
SEND_ROUNDS = 50                 # 连发 50 帧(~100ms @500Hz)确保主控板收到
SEND_PERIOD = 0.002              # 2ms/帧


def _ms() -> int:
    return int(time.time() * 1000)


def _ok(action, applied) -> dict:
    return {"ok": True, "card": CARD, "action": action, "control_level": CONTROL_LEVEL,
            "applied": applied, "timestamp_ms": _ms()}


def _err(code, message) -> dict:
    return {"ok": False, "code": code, "message": message}


def _send_bms_off_once() -> dict:
    """临时建低层 UDP，连发 LowCmd.bms.off=0xA5 给主控板，发完即关。返回发送结果摘要。"""
    import robot_interface as sdk   # 惰性 import（Mac 无 .so 时进 STUB）
    udp = None
    sent = 0
    try:
        udp = sdk.UDP(LOWLEVEL, LOW_LOCAL_PORT, LOW_TARGET_IP, LOW_TARGET_PORT)
        cmd = sdk.LowCmd()
        udp.InitCmdData(cmd)
        cmd.bms.off = BMS_OFF_CODE
        for _ in range(SEND_ROUNDS):
            udp.SetSend(cmd)
            udp.Send()
            sent += 1
            time.sleep(SEND_PERIOD)
        return {"sent_frames": sent, "target": f"{LOW_TARGET_IP}:{LOW_TARGET_PORT}",
                "bms_off": BMS_OFF_CODE}
    finally:
        # UDP 析构会关闭 socket（探测时观测到 "Closing UDP." 日志）
        try:
            del udp
        except Exception:  # noqa: BLE001
            pass


class Plugin:
    """电池关断卡：battery_off（confirm + reason + 静止前置；低层 bms.off）。"""

    def __init__(self, plugin_config, namespace, executor, client):
        self._client = client   # 共享高层 client（仅用于读 snapshot 做静止前置检查）

    def get_tool(self):
        return {"name": CARD, "type": TYPE, "multiInstance": False, "description": DESC,
                "inputSchema": {
                    "type": "object",
                    "properties": {
                        "action": {"type": "string", "enum": ["battery_off"]},
                        "confirm": {"type": "boolean", "description": "必须 true"},
                        "reason": {"type": "string", "description": "关机原因（必填）"},
                    },
                    "required": ["action"],
                    "x-action-params": {
                        "battery_off": {"params": ["confirm", "reason"],
                                        "description": "断电池（低层 bms.off=0xA5，不可逆）。需静止 + confirm + reason。"},
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
        if action != "battery_off":
            return _err("INVALID_ARGUMENT", "unknown action '%s'" % action)
        if not args.get("confirm"):
            return _err("PRECONDITION_FAILED", "battery_off requires confirm=true")
        if not args.get("reason"):
            return _err("INVALID_ARGUMENT", "battery_off requires 'reason'")
        snap = self._client.snapshot()
        if not snap.get("fresh"):
            return _err("NO_FEEDBACK", "no fresh state; refuse battery_off")
        if self._is_moving(snap):
            return _err("PRECONDITION_FAILED", "robot must be static (stop_move/damp) before battery_off")
        try:
            result = _send_bms_off_once()
        except Exception as e:  # noqa: BLE001  # 含 STUB（无 robot_interface）的 ImportError
            return _err("EXEC_FAILED", "battery_off send failed: %s" % e)
        return _ok("battery_off", {"accepted": True, "send": result,
                                   "command_sent_at_ms": _ms(), "reason": args.get("reason")})


def make_plugin(plugin_config, namespace, executor, client):
    """main.py 装配入口。"""
    return Plugin(plugin_config, namespace, executor, client)
