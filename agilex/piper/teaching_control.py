"""Guarded teaching actions added to an existing control card, not a new card.

Only firmware versions already verified in deployment configuration may send
the SDK record/end-record commands. No CAN_CTRL teaching value is invented.
"""
from dataclasses import asdict
import math
import time

from agilex.piper.control import healthy, mode_name, signature, teaching_stopped


TEACHING_ACTIONS = {"enter_teaching": "进入拖动示教", "exit_teaching": "退出拖动示教"}


def add_teaching_schema(definition):
    schema = definition["inputSchema"]
    prop = schema["properties"]["action"]
    for action, label in TEACHING_ACTIONS.items():
        if action not in prop["enum"]:
            prop["enum"].append(action)
            prop.setdefault("oneOf", []).append({"const": action, "title": label})
        schema.setdefault("x-action-params", {})[action] = {
            "params": ["teaching_supported", "other_controllers_stopped", "execute", "preview_id"]
                      + (["teaching_space_clear"] if action == "enter_teaching" else []),
            "description": label,
        }
        schema.setdefault("x-confirmation", {})[action] = {
            "preview": {"action": action, "execute": False}, "execute": {"execute": True},
            "message": "确认已扶稳机械臂且其他控制程序已停止；进入示教会开始录制，退出会结束录制。以设备反馈为准。",
        }
    for key, label in (("teaching_supported", "人员已扶稳机械臂"),
                       ("teaching_space_clear", "示教活动范围已清空"),
                       ("other_controllers_stopped", "其他控制程序已停止")):
        schema["properties"].setdefault(key, {"type": "boolean", "enum": [False, True], "default": False,
            "title": label, "oneOf": [{"const": False, "title": "未确认"}, {"const": True, "title": "已确认"}]})
    schema["properties"].setdefault("execute", {"type": "boolean", "default": False, "x-ui-hidden": True})
    schema["properties"].setdefault("preview_id", {"type": "string", "x-ui-hidden": True})
    definition["description"] = "机械臂控制：许可、待机/CAN、进入/退出示教、电机、关节、夹爪和停止；先预览再确认。"


def fresh(state):
    if (len(state.joints_deg) != 6 or len(state.enabled) != 6
            or not all(math.isfinite(v) for v in state.joints_deg)
            or not math.isfinite(state.feedback_time) or not 0 <= time.time()-state.feedback_time <= .5):
        raise RuntimeError("六轴反馈无效或超过 0.5 秒未更新")
    return state


def teaching_action(runtime, action, args):
    if action not in TEACHING_ACTIONS:
        raise ValueError("未知示教操作")
    allowed_keys = {"action", "_tool_name", "instance_id", "execute", "preview_id", "teaching_supported",
                    "teaching_space_clear", "other_controllers_stopped"}
    if set(args)-allowed_keys:
        raise ValueError("示教操作包含无关参数")
    for key in ("execute", "teaching_supported", "teaching_space_clear", "other_controllers_stopped"):
        if key in args and type(args[key]) is not bool:
            raise ValueError(f"{key} 必须为布尔值")
    if "preview_id" in args and not isinstance(args["preview_id"], str):
        raise ValueError("预览编号必须为文本")
    entering = action == "enter_teaching"
    previous = getattr(runtime.worker_context, "operation_object", None)
    runtime.worker_context.operation_object = "arm"
    try:
        with runtime.operation("arm_control", action):
            driver = runtime.connected()
            state = fresh(runtime.read_state())
            current = mode_name(state)
            healthy(state, teaching=current == "teaching")
            if (entering and current == "teaching" or not entering and current in {"can", "standby"}
                    and teaching_stopped(state)):
                runtime.previews.invalidate()
                return {"confirmed": True, "executed": False, "state": asdict(state),
                        "message": "已处于示教模式" if entering else "已退出示教；未切换模式或使能电机"}
            # Read the SDK firmware cache only. Do not query or approve a new
            # firmware implicitly, and do not change the configured allowlist.
            if runtime.firmware_version is None:
                runtime.firmware_version = driver.firmware(query=False)
            verified = runtime.verified_teaching()
            missing = []
            if args.get("teaching_supported") is not True:
                missing.append("请先扶稳机械臂并确认")
            if args.get("other_controllers_stopped") is not True:
                missing.append("确认其他控制程序已停止")
            if entering:
                if current != "can" or not all(state.enabled):
                    missing.append("进入示教前须为电脑控制且六轴已使能")
                if args.get("teaching_space_clear") is not True:
                    missing.append("确认示教活动范围清空")
                if not teaching_stopped(state):
                    missing.append("请先结束录制或回放")
                if verified and current == "can" and not driver.stationary():
                    missing.append("机械臂须静止 0.5 秒")
            else:
                if current != "teaching":
                    missing.append("当前不是拖动示教模式")
                if not (teaching_stopped(state) or state.teach_status.startswith("START_RECORD")):
                    missing.append("示教轨迹正在回放，请先在机身暂停或结束回放")
            if not verified:
                missing.append("当前固件的软件示教流程尚未验证，请使用机身示教按键")
            plan = {"allowed": not missing, "executed": False, "current_mode": current,
                    "operation": TEACHING_ACTIONS[action], "firmware": runtime.firmware_version,
                    "software_teaching_verified": verified, "missing_conditions": missing,
                    "button_guidance": not verified,
                    "message": "；".join(missing) if missing else "条件满足，请确认本次示教切换",
                    "guidance": "扶稳机械臂，按对应固件手册使用机身示教按键；随后在顶部监控核对姿态。" if not verified else ""}
            key = f"teaching:{action}:{runtime.firmware_version}"
            if missing:
                runtime.previews.invalidate()
                return plan
            if not args.get("execute", False):
                return {**plan, **runtime.previews.put(key, state)}
            runtime.require_motion_permission()
            runtime.previews.consume(args.get("preview_id") or None, key, state)
            try:
                with driver._send_lock:
                    driver._check_cancelled()
                    latest = fresh(driver.get_state())
                    if (signature(latest) != signature(state)
                            or any(abs(a-b) > .5 for a,b in zip(latest.joints_deg, state.joints_deg))):
                        raise RuntimeError("执行前状态已变化，请重新预览")
                    runtime.require_motion_permission()
                    sent_at = time.time()
                    driver.arm.MotionCtrl_1(grag_teach_ctrl=1 if entering else 2)
                result = driver._wait_confirmation(
                    lambda s: (mode_name(s) == "teaching" and s.teach_status.startswith("START_RECORD")) if entering else
                              (mode_name(s) in {"can", "standby"} and teaching_stopped(s)),
                    sent_at, "请核对示教模式反馈；未主动回零、切换 CAN 或使能电机。")
                return {**plan, **result, "executed": True}
            finally:
                runtime.previews.invalidate()
                runtime.last_signature = None
    finally:
        runtime.worker_context.operation_object = previous
