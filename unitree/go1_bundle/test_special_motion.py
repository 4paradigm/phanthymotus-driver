"""
test_special_motion.py — Go1 官方特殊动作卡（jump_yaw_left / straight_hand）。

【未验收·test 前缀】本卡尚未在目标 Go1 实机固件上验证，故按 CONTRIBUTING.md §4 用 `test_`
前缀命名，实机验收量程+安全后再改名 special_motion 上架。

自包含：一张卡 = 一个文件。main.py 按 config.yaml 里的卡名自动 import 并 make_plugin()。
本卡把两个白名单特殊动作封装为**同步动作序列**（调用会阻塞至整套动作完成再返回完整结果）：
  1. set_posture(mode=1) 力控平衡站立，并读 snapshot()["mode"] 确认真的站到位——否则拒绝开火；
  2. set_posture(mode=10/11) 触发特殊动作，保持白名单时长并按 5Hz 采样 snapshot()["progress"]；
  3. set_posture(mode=1) 回位，确认落稳。
一次调用返回整套 go/no-go 结果（是否站稳/是否进入特殊模式/progress 采样/是否安全回位），
因为这是目标狗上的首次真实开火测试，调用方需要清晰的成败信号。

前置：狗须【已站立】（高层无法从地面扶起，需遥控扶起）。高风险动作须 confirm=true。
下发只用共享 client 已有的 set_posture()/snapshot() 原语（mode 10/11 见 go1_sdk_client
的 MODE_NAMES），不改动 client。
⚠️ 控制卡须上真机验证量程+安全后才能去掉 test_ 前缀上架（见 CONTRIBUTING.md §4）。
"""

from __future__ import annotations

import threading
import time

# ── 卡片元数据 ────────────────────────────────────────────────────────────────
CARD = "test_special_motion"     # 卡名 = MCP 工具名 = config.yaml key = 本文件名
TYPE = "actuator"
CONTROL_LEVEL = "HIGHLEVEL"
NODE = "go1_test_special_motion"  # 预留（本卡不发 topic）
DESC = (
    "Go1 官方特殊动作 (HIGHLEVEL·未验收) — jump_yaw_left / straight_hand。"
    "封装为同步动作序列：先进入 balance_stand(mode=1) 并确认站稳，再 set_posture(mode=10/11)，"
    "保持白名单时长后回到 mode=1。调用会阻塞整个动作过程并返回完整执行结果"
    "（是否站稳/是否进入特殊模式/progress 采样/是否安全回位）。动作时长由 config.yaml 白名单配置，"
    "须在目标 Go1 固件上实机验证。高风险动作须 confirm=true。前置：狗须【已站立】。"
)

# HighState.mode（Go1 legacy comm.h）：mode 1 = force_stand / balance stand。
M_FORCE_STAND = 1

# 两个白名单特殊动作对应的 HighCmd.mode 码（Go1 comm.h；与 go1_sdk_client.MODE_NAMES 一致）。
_SPECIAL_MOTION_MODES = {
    "jump_yaw_left": 10,   # jumpYaw    — 左向偏航跳
    "straight_hand": 11,   # straightHand
}

# config.yaml 未指定时的兜底保持时长（秒）。
_DEFAULT_DURATIONS = {
    "jump_yaw_left": 3.0,
    "straight_hand": 4.0,
}


def _ms() -> int:
    return int(time.time() * 1000)


def _err(code: str, message: str) -> dict:
    return {"ok": False, "code": code, "message": message}


class Plugin:
    """特殊动作控制卡：同步跑一整套白名单动作序列，串行互斥（一次只跑一个）。"""

    def __init__(self, plugin_config, namespace, executor, client):
        self._client = client
        self._lock = threading.Lock()
        self._running_seq = None
        self._seq_id = 0

        cfg = plugin_config or {}
        self._durations = {
            "jump_yaw_left": float(cfg.get("jump_yaw_left_duration_s",
                                           _DEFAULT_DURATIONS["jump_yaw_left"])),
            "straight_hand": float(cfg.get("straight_hand_duration_s",
                                           _DEFAULT_DURATIONS["straight_hand"])),
        }
        # 等待 mode 迁移（站到 balance_stand / 回位）的超时（秒）。
        self._stabilize_timeout = float(cfg.get("stabilize_timeout_s", 3.0))
        # 确认站稳后、开火前的小停顿（秒）。
        self._settle_s = float(cfg.get("stabilize_settle_s", 0.5))

    # ── 生命周期 ──
    def start(self):
        pass

    def stop(self):
        # 尽力：关停时把狗带回平衡站立。
        try:
            self._client.set_posture(M_FORCE_STAND)
        except Exception:  # noqa: BLE001
            pass

    # ── 工具声明 ──
    def get_tool(self):
        return {
            "name": CARD, "type": TYPE, "multiInstance": False, "description": DESC,
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
                    "stop":          {"params": [],
                                      "description": "立即回到平衡站立（打断/兜底）"},
                },
            },
        }

    # ── 分发（返回 plain dict；未知 action 返回 None）──
    def dispatch(self, action, args):
        if action == "start":
            return {"state": "ready"}
        if action == "info":
            return {"ok": True, "card": CARD, "action": "info", "control_level": CONTROL_LEVEL,
                    "applied": {"running_seq": self._running_seq}, "timestamp_ms": _ms()}
        if action == "stop":
            self._client.set_posture(M_FORCE_STAND)
            return {"ok": True, "card": CARD, "action": "stop", "control_level": CONTROL_LEVEL,
                    "applied": {"mode": M_FORCE_STAND}, "timestamp_ms": _ms()}

        if action not in _SPECIAL_MOTION_MODES:
            return None

        args = args or {}
        if not args.get("confirm"):
            return _err("PRECONDITION_FAILED", f"{action} requires confirm=true")

        # NO_FEEDBACK 守卫——没有新鲜状态（sport 通道未拿到 / 被别的客户端占用 / STUB）时绝不开火。
        snap = self._client.snapshot()
        if not snap.get("fresh"):
            return _err("NO_FEEDBACK",
                        "no fresh HighState (sport channel not acquired / STUB); refuse special motion")

        with self._lock:
            if self._running_seq is not None:
                return _err("PRECONDITION_FAILED",
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

    # ── internal ────────────────────────────────────────────────────────────

    def _current_mode(self):
        return self._client.snapshot().get("mode")

    def _current_progress(self) -> float:
        try:
            return round(float(self._client.snapshot().get("progress", 0.0)), 3)
        except Exception:  # noqa: BLE001
            return 0.0

    def _wait_mode(self, target: int, timeout: float) -> bool:
        """轮询 snapshot()["mode"] 直到等于 target 或超时。"""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self._current_mode() == target:
                return True
            time.sleep(0.05)
        return self._current_mode() == target

    def _hold_and_sample(self, mode_code: int, duration: float) -> list:
        """开火后持续保持 mode_code（每周期重发以刷住姿态命令），按 5Hz 采样 progress。"""
        samples = []
        start = time.monotonic()
        deadline = start + duration
        while time.monotonic() < deadline:
            # 重发保持特殊动作 mode（set_posture 是持续保持型，但重发可防被其它调用覆盖）。
            self._client.set_posture(mode_code)
            samples.append([round(time.monotonic() - start, 2), self._current_progress()])
            time.sleep(0.2)
        return samples

    def _run_sequence(self, action: str, mode_code: int,
                      duration: float, seq_id: int) -> dict:
        steps = {}
        client = self._client

        # 1. 进入 balance_stand 并确认到位。
        client.set_posture(M_FORCE_STAND)
        stood = self._wait_mode(M_FORCE_STAND, self._stabilize_timeout)
        steps["balance_stand_confirmed"] = stood
        if not stood:
            client.set_posture(M_FORCE_STAND)  # 继续命令一个安全 mode
            return {
                "ok": False, "code": "PRECONDITION_FAILED",
                "message": (f"robot did not reach balance_stand (mode=1) within "
                            f"{self._stabilize_timeout}s (current mode={self._current_mode()}); "
                            "special motion aborted before firing"),
                "card": CARD, "action": action, "control_level": CONTROL_LEVEL,
                "sequence_id": seq_id, "steps": steps, "timestamp_ms": _ms(),
            }
        time.sleep(self._settle_s)

        # 2. 开火，保持白名单时长并采样 progress。
        client.set_posture(mode_code)
        entered = self._wait_mode(mode_code, min(1.5, self._stabilize_timeout))
        steps["entered_special_mode"] = entered
        samples = self._hold_and_sample(mode_code, duration)
        steps["progress_samples"] = samples
        steps["progress_max"] = max((p for _, p in samples), default=0.0)

        # 3. 回到 balance_stand 并确认落稳。
        client.set_posture(M_FORCE_STAND)
        returned = self._wait_mode(M_FORCE_STAND, self._stabilize_timeout)
        steps["returned_to_stand"] = returned

        # 安全关键结果是稳定回到站立。entered / progress 原样上报，供调用方判断固件是否真的执行了动作。
        note = None
        if not entered and steps["progress_max"] == 0.0:
            note = ("HighState.mode never showed mode=%d and progress stayed 0 — "
                    "firmware may not have executed this motion (check Legged_sport "
                    "version / robot readiness)." % mode_code)

        return {
            "ok": bool(returned),
            "card": CARD, "action": action, "control_level": CONTROL_LEVEL,
            "sequence_id": seq_id,
            "mode": mode_code,
            "duration_s": duration,
            "state": "completed" if returned else "completed_with_warnings",
            "steps": steps,
            **({"note": note} if note else {}),
            "timestamp_ms": _ms(),
        }


def make_plugin(plugin_config, namespace, executor, client):
    """main.py 装配入口。"""
    return Plugin(plugin_config, namespace, executor, client)
