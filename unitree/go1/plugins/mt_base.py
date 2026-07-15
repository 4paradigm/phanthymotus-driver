from __future__ import annotations

#!/usr/bin/env python3
"""
plugins/mt_base.py — MT 卡片公共层(返回包络、错误码、控制等级校验、状态卡基类)。

依据 MT《Go1 动作/运控能力卡片》§3 统一接口约定:
  • 控制卡成功:{ok, card, action, control_level, applied, timestamp_ms}
    失败:{ok:false, code, message}(不允许静默裁剪后继续)。
  • 状态卡:无业务输入,支持 start/stop/info;消息含 timestamp_ms/control_level/最新数据。
所有 MT 卡片通过本层与 go1_ctrl(唯一硬件入口)交互。
"""
import time

from plugins.base import BasePlugin, clamp  # noqa: F401  (clamp 供子类 import)

# 错误码(MT §3.1)
INVALID_ARGUMENT = "INVALID_ARGUMENT"
WRONG_CONTROL_LEVEL = "WRONG_CONTROL_LEVEL"
PRECONDITION_FAILED = "PRECONDITION_FAILED"
NO_FEEDBACK = "NO_FEEDBACK"
SAFETY_LIMIT = "SAFETY_LIMIT"
COMMUNICATION_ERROR = "COMMUNICATION_ERROR"


def now_ms() -> int:
    return int(time.time() * 1000)


class MtCard(BasePlugin):
    """MT 卡片基类。子类设 CARD;控制卡设 CONTROL_LEVEL(HIGHLEVEL|LOWLEVEL|ANY)。"""
    CARD = ""
    CONTROL_LEVEL = "ANY"

    def __init__(self, cfg, namespace, bridge, ctrl):
        self._cfg = cfg or {}
        self._ns = namespace
        self._bridge = bridge
        self._ctrl = ctrl

    # ── 返回包络 ────────────────────────────────────────────────────────────
    def _ok(self, action, applied=None):
        return {"ok": True, "card": self.CARD, "action": action,
                "control_level": self._ctrl.control_level,
                "applied": applied or {}, "timestamp_ms": now_ms()}

    def _err(self, code, message):
        return {"ok": False, "code": code, "message": message,
                "card": self.CARD, "timestamp_ms": now_ms()}

    # ── 前置校验 ────────────────────────────────────────────────────────────
    def _level_error(self):
        """卡片控制等级与 Driver 当前等级不符 → WRONG_CONTROL_LEVEL;否则 None。"""
        if self.CONTROL_LEVEL != "ANY" and self._ctrl.control_level != self.CONTROL_LEVEL:
            return self._err(WRONG_CONTROL_LEVEL,
                             "card '%s' needs %s but driver is %s"
                             % (self.CARD, self.CONTROL_LEVEL, self._ctrl.control_level))
        return None

    def _require_confirm(self, args, action):
        if args.get("confirm") is not True:
            return self._err(PRECONDITION_FAILED, "action '%s' requires confirm=true" % action)
        return None

    @staticmethod
    def _num(args, key):
        """取数值;缺失/非数返回 (None, err_msg)。"""
        v = args.get(key)
        if v is None:
            return None, "missing '%s'" % key
        try:
            return float(v), None
        except (TypeError, ValueError):
            return None, "'%s' must be a number" % key

    def _ranged(self, args, key, lo, hi):
        """取并校验范围;越界不静默裁剪,返回 (val, err_dict|None)。"""
        v, msg = self._num(args, key)
        if msg:
            return None, self._err(INVALID_ARGUMENT, msg)
        if v < lo or v > hi:
            return None, self._err(INVALID_ARGUMENT,
                                   "'%s'=%s out of range [%s, %s]" % (key, v, lo, hi))
        return v, None


# ══════════════════════════════════════════════════════════════════════════════
#  状态卡基类
# ══════════════════════════════════════════════════════════════════════════════
class MtStateCard(MtCard):
    """状态 sensor 卡:子类设 CARD/TOPIC/FORMAT/HZ,实现 _payload()。"""
    TOPIC = ""            # 相对路径,如 "state/imu"(会拼成 /{ns}/state/imu)
    FMT = "data/json"
    HZ = 10.0
    DESC = ""

    def __init__(self, cfg, namespace, bridge, ctrl):
        super().__init__(cfg, namespace, bridge, ctrl)
        self._topic = "/%s/%s" % (namespace, self.TOPIC)
        try:
            bridge.add_sensor("go1_%s" % self.CARD, self._topic, self.HZ, self._produce)
        except Exception:  # noqa: BLE001
            pass

    def _payload(self):
        raise NotImplementedError

    def _produce(self):
        # 连接态但无新包 → 不伪造时间戳,跳过本次发布(MT §3.2)
        if self._ctrl.is_connected() and not self._ctrl.state_fresh():
            return None
        d = self._payload()
        if d is None:
            return None
        d = dict(d)
        d["timestamp_ms"] = now_ms()
        d["control_level"] = self._ctrl.control_level
        return d

    def get_tool(self):
        # 纯状态卡:只出数据流(topic_out),**不暴露可执行 action** → core 不渲染"执行"按钮
        # (MT 要求:状态卡去掉执行按钮,只看数据流)。read/start/stop/info 仍可经 MCP 调用
        # (转发器就是直接 tools/call read,不走此 schema),仅是不在 UI 上给按钮。
        return {
            "name": self.CARD, "type": "sensor", "multiInstance": False,
            "readOnly": True,
            "description": self.DESC or ("Go1 %s state → %s" % (self.CARD, self._topic)),
            "inputSchema": {"type": "object", "properties": {}},
            "topic_out": [{"topic": self._topic, "format": self.FMT}],
        }

    def dispatch(self, action, args):
        if action == "read":
            d = self._produce()
            return d if d is not None else self._err(NO_FEEDBACK, "no fresh state packet")
        if action == "info":
            return {"state": "running", "control_level": self._ctrl.control_level,
                    "topic_out": [{"topic": self._topic, "format": self.FMT}]}
        if action in ("start", "stop"):
            return self._lifecycle(action)
        return self._err(INVALID_ARGUMENT, "unknown action '%s'" % action)
