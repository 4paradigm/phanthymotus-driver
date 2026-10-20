"""Pure policy and one-use previews. This module never sends CAN commands."""

from dataclasses import dataclass
import math
import threading
import time
from uuid import uuid4


def mode_name(state):
    for prefix, name in (("STANDBY", "standby"), ("CAN_CTRL", "can"),
                         ("TEACHING_MODE", "teaching")):
        if state.control_mode.startswith(prefix):
            return name
    return "other"


def signature(state):
    return (state.control_mode, tuple(state.enabled), state.teach_status, state.error_code, state.motor_faults)


def teaching_stopped(state):
    return state.teach_status.startswith(("DISABLED", "STOP_RECORDING", "PAUSE_EXECUTION", "TERMINATE_EXECUTION"))


def healthy(state, teaching=False):
    # Recording/replaying/paused are activity states, not motor faults.
    normal = state.arm_status.startswith("NORMAL")
    activity = teaching and any(state.arm_status.startswith(p) for p in
                               ("TEACHING", "REPLAY", "TRAJECTORY", "RECORD"))
    if state.error_code or state.motor_faults or not (normal or activity):
        raise RuntimeError("机械臂有故障或状态异常，请先查看状态。")
    if any(state.enabled) and not all(state.enabled):
        raise RuntimeError("六轴只有部分使能，请先检查状态。")


def ready_to_move(state):
    healthy(state)
    if mode_name(state) != "can" or not all(state.enabled):
        raise RuntimeError("需要电脑控制且六轴已使能，请先检查模式并初始化。")
    if not state.teach_status.startswith(("DISABLED", "STOP_RECORDING", "TERMINATE_EXECUTION")):
        raise RuntimeError("请先停止示教录制或回放。")


def summary(state):
    mode = {"standby": "待机", "can": "电脑控制", "teaching": "拖动示教",
            "other": f"其它模式（{state.control_mode}）"}[mode_name(state)]
    enabled = "六轴已使能" if all(state.enabled) else "六轴已失能" if not any(state.enabled) else "部分使能"
    fault = "无故障" if not state.error_code and not state.motor_faults and (state.arm_status.startswith("NORMAL") or state.arm_status.startswith("TEACHING_")) else f"故障/异常：{state.arm_status} {state.error_code} {' '.join(state.motor_faults)}"
    return f"{mode}｜{enabled}｜{fault}"


def mode_plan(state, target, args, verified_teaching=False, stationary=False):
    if target not in {"standby", "can", "teaching"}:
        raise ValueError("target_mode 仅支持 standby / can / teaching")
    current = mode_name(state)
    missing = []
    try:
        healthy(state, teaching=current == "teaching")
    except RuntimeError as exc:
        missing.append(str(exc))
    guidance = False
    reminder = ""
    if target == "standby":
        reminder = "请先支撑机械臂并解除使能。" if any(state.enabled) else "待机不代表失能，请核对六轴状态。"
        if any(state.enabled):
            missing.append("六轴须全部失能；支撑后在机械臂卡片单独失能")
        if current not in {"standby", "can"}:
            missing.append("请先退出示教或其它控制模式")
    elif target == "can" and current != "teaching":
        reminder = "切换后仍未使能，请再执行初始化。"
        if current != "standby":
            missing.append("当前须为待机")
        if any(state.enabled):
            missing.append("六轴须全部失能")
        if args.get("other_controllers_stopped") is not True:
            missing.append("确认其它控制程序已停止")
    else:
        reminder = "扶稳后进入示教，轻推调整姿态。" if target == "teaching" else "先退出示教，再检查状态并初始化。"
        if target == "teaching":
            if current != "can" or not all(state.enabled):
                missing.append("当前须为电脑控制且六轴已使能")
            if not stationary:
                missing.append("机械臂须静止 0.5 秒")
            if args.get("workspace_clear") is not True:
                missing.append("确认活动空间清空")
        if args.get("arm_supported") is not True:
            missing.append("确认人员已扶稳机械臂")
        if not verified_teaching:
            guidance = True
            missing.append("实际固件的软件示教流程未验收，请使用机身按键")
    if not teaching_stopped(state):
        missing.append("请先停止示教录制或回放")
    if current == "other":
        missing.append("本版只显示此模式，不提供接管入口")
    return {"current_mode": current, "target_mode": target, "allowed": not missing,
            "missing_conditions": missing, "reminder": reminder,
            "button_guidance": guidance,
            "guidance": ("扶稳机械臂，按该固件手册使用机身示教按键；退出后重新读取状态。"
                         "若流程要求回零或重启，请单独安排现场操作。") if guidance else ""}


@dataclass
class Preview:
    key: str
    state: object
    created: float
    extra: object


class Previews:
    def __init__(self):
        self.entries = {}
        self.lock = threading.Lock()

    def invalidate(self):
        with self.lock:
            self.entries.clear()

    def put(self, key, state, extra=None):
        with self.lock:
            self.entries.clear()  # A new preview replaces every older target.
            token = uuid4().hex
            self.entries[token] = Preview(key, state, time.monotonic(), extra)
        return {"preview_id": token, "valid_for_seconds": 60}

    def consume(self, token, key, state, extra=None):
        with self.lock:
            if token is None and len(self.entries) == 1:
                token = next(iter(self.entries))
            preview = self.entries.pop(token, None)
        extra_matches = (preview is not None and (preview.extra == extra or
                        isinstance(preview.extra,tuple) and isinstance(extra,tuple)
                        and preview.extra[1:] == extra[1:] and abs(preview.extra[0]-extra[0]) <= .5))
        if (preview is None or preview.key != key or time.monotonic() - preview.created > 60
                or signature(preview.state) != signature(state)
                or len(state.joints_deg) != 6
                or any(not math.isfinite(b) or abs(a-b) > .5
                       for a, b in zip(preview.state.joints_deg, state.joints_deg))
                or not extra_matches):
            raise RuntimeError("预览已失效或已执行；请重新预览。")
