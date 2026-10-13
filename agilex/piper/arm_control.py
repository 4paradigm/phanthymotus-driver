"""One Chinese control card on the existing PiPER runtime.

The old ten-card implementation remains available for regression and backups.
Only arm_control and the optional camera are registered by this module.
"""
from dataclasses import asdict
import json
import math
import time

from common.vendor_runtime import action_schema, tool
from agilex.piper.cards import build_tools, choice
from agilex.piper.control import healthy, mode_name, ready_to_move, teaching_stopped
from agilex.piper.device import PiperPlugin
from agilex.piper.runtime import PiperRuntime, confirm, number


CONTROL = "arm_control"
JOG_ACTIONS = {"jog_positive": 1, "jog_negative": -1}
GRIPPER_ACTIONS = {"gripper_open": "open", "gripper_close": "close",
                   "gripper_set_width": "set_width"}


def control_tools(topic):
    legacy = {item["name"]: item for item in build_tools(topic)}
    properties = {}
    for item in legacy.values():
        properties.update({key: value for key, value in item["inputSchema"]["properties"].items()
                           if key != "action"})
    properties["step_deg"] = {"type": "number", "enum": [.5, 1, 2], "default": .5,
                               "title": "微动步长（°）"}
    actions = {
        "read": ([], "读取机械臂状态"),
        "allow_motion": ([], "开启本次操作许可"),
        "block_motion": ([], "关闭操作许可并停止动作"),
        "switch_mode": (["target_mode", "other_controllers_stopped"], "切换待机 / CAN"),
        "enable": (["object", "speed_pct"], "使能所选电机"),
        "disable": (["object", "supported"], "失能所选电机（先支撑）"),
        "jog_positive": (["joint", "step_deg", "speed_deg_s", "speed_pct"], "所选关节正向微动 +"),
        "jog_negative": (["joint", "step_deg", "speed_deg_s", "speed_pct"], "所选关节反向微动 −"),
        "move_joint": (["joint", "target_deg", "speed_deg_s", "speed_pct"], "所选关节绝对目标角度"),
        "gripper_open": (["torque_nm"], "夹爪张开"),
        "gripper_close": (["torque_nm"], "夹爪闭合"),
        "gripper_set_width": (["width_mm", "torque_nm"], "夹爪指定开度"),
        "halt": (["target"], "停止所选范围（保持电机力矩）"),
        "cancel_preview": ([], "取消当前预览"),
        "start": ([], "启动卡片；不连接、不使能、不移动"),
        "stop": ([], "停止卡片并关闭本次许可；不失能"),
        "info": ([], "查看控制卡和最近动作结果"),
    }
    properties["target_mode"] = choice("目标模式", [("standby", "待机"), ("can", "电脑控制（CAN）")], "can")
    used = {key for params, _ in actions.values() for key in params}
    used.update({"execute", "preview_id", "workspace_clear", "take_control", "allow_limit_adjustment"})
    properties = {key: value for key, value in properties.items() if key in used}
    schema = action_schema(actions, properties)
    schema["properties"]["action"].update(title="操作", default="read",
        oneOf=[{"const": action, "title": label} for action, (_, label) in actions.items()])
    schema.update({"additionalProperties": False, "x-manual-execution": True,
                   "x-resource": "arm", "x-is-dangerous": True,
                   "x-hooks": {hook: {"action": "halt", "params": {"target": "all"}}
                               for hook in ("on_interrupt_motion", "on_interrupt_all")}})
    guarded = {"switch_mode": "piper_can", "enable": "piper_enable", "move_joint": "piper_joint",
               **{action: "piper_joint" for action in JOG_ACTIONS},
               **{action: "piper_gripper" for action in GRIPPER_ACTIONS}}
    schema["x-confirmation"] = {}
    for action, source in guarded.items():
        source_action = (GRIPPER_ACTIONS[action] if action in GRIPPER_ACTIONS else
                         "move" if source == "piper_joint" else
                         "switch" if action == "switch_mode" else "enable")
        rule = legacy[source]["inputSchema"]["x-confirmation"][source_action]
        schema["x-confirmation"][action] = {
            **rule, "preview": {"action": action, "execute": False},
        }
        params = ["execute", "preview_id"]
        if action != "switch_mode":
            params += ["workspace_clear", "take_control"]
        if action == "enable":
            params += ["allow_limit_adjustment"]
        schema["x-action-params"][action]["params"] += params
    # Permit toggles are human menu operations, as in the legacy card contract.
    for action in ("allow_motion", "block_motion"):
        schema["x-action-params"].pop(action)
    schema["x-completion"] = {"actions": ["enable", "move_joint", *JOG_ACTIONS, *GRIPPER_ACTIONS], "timeout": 60}
    arm = tool(CONTROL, "actuator", "机械臂控制：许可、待机/CAN、电机、关节微动/绝对角度、夹爪和停止；先预览再确认。", schema)
    arm["title"] = "机械臂控制"
    return [arm, legacy["piper_photo"]]


class ArmControlPlugin(PiperPlugin):
    def get_tools(self):
        return control_tools(self.topic)

    def _validate(self, name, action, args):
        definition = next((item for item in self.get_tools() if item["name"] == name), None)
        if definition is None:
            raise ValueError("未知卡片；请刷新机械臂控制卡")
        properties = definition["inputSchema"]["properties"]
        if action not in properties["action"]["enum"]:
            raise ValueError("该卡片不支持此操作")
        for key, value in args.items():
            if key in {"_tool_name", "action", "instance_id"}:
                continue
            if key not in properties:
                raise ValueError(f"此操作不使用参数 {key}")
            prop = properties[key]
            if prop["type"] == "boolean" and type(value) is not bool:
                raise ValueError(f"{key} 必须是布尔值")
            if prop["type"] in {"number", "integer"}:
                number(args, key, None, prop.get("minimum", -math.inf), prop.get("maximum", math.inf),
                       integer=prop["type"] == "integer")
            if prop["type"] == "string" and not isinstance(value, str):
                raise ValueError(f"{key} 必须是文本")
            if "enum" in prop and value not in prop["enum"]:
                raise ValueError(f"{key} 的选项无效")

    @staticmethod
    def _object(action, args):
        if action in GRIPPER_ACTIONS:
            return "gripper"
        return args.get("object", "arm") if action in {"enable", "disable"} else "arm"

    def dispatch(self, action, args):
        args = dict(args)
        # The canvas serializes numeric enum selections as strings.
        if args.get("step_deg") in ("0.5", "1", "2"):
            args["step_deg"] = float(args["step_deg"])
        name = args.get("_tool_name", CONTROL)
        self._validate(name, action, args)
        long_action = (name == "piper_photo" and action in {"capture", "import_photo"}
                       or name == CONTROL and action in {"enable", "move_joint", *JOG_ACTIONS, *GRIPPER_ACTIONS}
                       and args.get("execute") is True)
        previous = getattr(self.worker_context, "operation_object", None)
        self.worker_context.operation_object = self._object(action, args)
        try:
            if long_action and self.config.get("async_actions", True):
                return self.run_async(name, action, args)
            return self.dispatch_sync(action, args)
        finally:
            self.worker_context.operation_object = previous

    def dispatch_sync(self, action, args):
        name = args.get("_tool_name", CONTROL)
        self._validate(name, action, args)
        if name == "piper_photo":
            return super().dispatch_sync(action, args)
        if action == "start":
            self.start(name)
            return {"state": "ready", "message": "机械臂控制卡已就绪；未连接、使能或移动"}
        if action == "stop":
            return {**self.request_stop(name), "note": "已请求取消并尽力保持；未释放电机力矩，请核对实际反馈"}
        if action == "info":
            snapshot = self.read_monitor_snapshot()
            return {"state": "idle" if self.closed or name in self.stopped_cards else "ready",
                    "snapshot": snapshot, "busy": snapshot["busy"],
                    "last_action": snapshot["last_actions"].get(name),
                    "motion_enabled": snapshot["motion_enabled"], "active_card": snapshot["active_card"],
                    "active_object": snapshot["active_object"]}
        if action == "halt":
            return self.halt(args.get("target", "all"))
        if action == "cancel_preview":
            self.previews.invalidate()
            return {"executed": False, "cancelled": True}
        previous = getattr(self.worker_context, "public_call", None)
        previous_object = getattr(self.worker_context, "operation_object", None)
        self.worker_context.public_call = (name, action)
        self.worker_context.operation_object = self._object(action, args)
        try:
            if action == "block_motion":
                return self._rewrite(self.motion_permission(action))
            if self.closed or name in self.stopped_cards:
                raise RuntimeError("机械臂控制卡已停止，请先启动卡片")
            if action == "read":
                return self.status()
            if action == "allow_motion":
                return self._rewrite(self.motion_permission(action))
            if action == "switch_mode":
                self._fresh_state()
                target = args.get("target_mode", "can")
                return self._rewrite(self.switch_card(target, args))
            if action == "enable":
                self._fresh_state()
                return self._rewrite(self.enable_card(args))
            if action == "disable":
                self._fresh_state()
                return self._rewrite(self.disable_card(args))
            if action in JOG_ACTIONS:
                return self._jog(action, args)
            if action == "move_joint":
                self._fresh_state()
                return PiperRuntime.dispatch_sync(self, action, {**args, "_tool_name": "piper_arm"})
            if action in GRIPPER_ACTIONS:
                self._fresh_state()
                grip = self.connected().get_gripper()
                if not grip.enabled:
                    raise RuntimeError("请在机械臂控制卡选择夹爪并使能，再操作开合")
                return self._rewrite(self.gripper_action(GRIPPER_ACTIONS[action], args))
            raise ValueError("未知控制操作")
        except (ValueError, RuntimeError) as exc:
            raise type(exc)(self._message(str(exc))) from exc
        finally:
            self.worker_context.public_call = previous
            self.worker_context.operation_object = previous_object

    def _fresh_state(self):
        state = self.read_state()
        if (not math.isfinite(state.feedback_time) or abs(time.time() - state.feedback_time) > .5
                or len(state.joints_deg) != 6 or not all(math.isfinite(v) for v in state.joints_deg)):
            raise RuntimeError("六轴反馈无效或超过 0.5 秒未更新")
        return state

    def _jog(self, action, args):
        joint = number(args, "joint", 1, 1, 6, integer=True)
        step = number(args, "step_deg", .5, .5, 2)
        if step not in (.5, 1, 2):
            raise ValueError("微动步长仅支持 0.5°、1°、2°")
        rate = number(args, "speed_deg_s", 1, .5, 5)
        speed = number(args, "speed_pct", 5, 1, 10, integer=True)
        execute = args.get("execute", False)
        if execute:
            self.motion_allowed(args)
            confirm(args, "take_control")
        with self.operation(CONTROL, action):
            state = self._fresh_state()
            ready_to_move(state)
            driver = self.connected()
            key = json.dumps([action, joint, step, rate, speed])
            if not execute:
                target = list(state.joints_deg)
                target[joint - 1] += JOG_ACTIONS[action] * step
                plan = driver.joint_plan(target, rate, speed)
                return {**plan, "executed": False, **self.previews.put(key, state, tuple(plan["target_deg"]))}
            # The preview owns the absolute target. Never add the delta again.
            with self.previews.lock:
                token = args.get("preview_id") or (next(iter(self.previews.entries)) if len(self.previews.entries) == 1 else None)
                entry = self.previews.entries.get(token)
                if entry is None or not isinstance(entry.extra, tuple) or len(entry.extra) != 6:
                    raise RuntimeError("预览已失效或已执行；请重新预览")
                target = entry.extra
            self.previews.consume(token, key, state, target)
            driver.joint_plan(target, rate, speed)
            return {"executed": True, "target_deg": target,
                    "state": asdict(driver._move_joint_target(target, rate, speed))}

    @staticmethod
    def _message(text):
        for old, new in (("停止单轴卡片", "停止机械臂控制卡"),
                         ("「机器人状态」", "「机械臂控制→读取状态」"),
                         ("「操作许可」", "「机械臂控制→开启操作许可」"),
                         ("「切换电脑控制」", "「机械臂控制→切换待机/CAN」"),
                         ("「电机使能」", "「机械臂控制→使能电机」"),
                         ("「电机失能」", "「机械臂控制→失能电机」"),
                         ("「停止动作」", "「机械臂控制→停止动作」")):
            text = text.replace(old, new)
        return text

    @classmethod
    def _rewrite(cls, result):
        if isinstance(result, str):
            return cls._message(result)
        if isinstance(result, dict):
            return {key: cls._rewrite(value) for key, value in result.items()}
        if isinstance(result, list):
            return [cls._rewrite(value) for value in result]
        return result

    def status(self):
        result = PiperRuntime.status(self)
        result["completed_actions"] = dict(self.last_actions)
        result["active_object"] = self.active_object
        result["next_step"] = "在机械臂控制卡分别检查许可、模式与使能，再预览关节或夹爪动作"
        return result

    def halt(self, target):
        with self.lifecycle:
            self.previews.invalidate()
            active = self.active_card
            affected = self.gate.locked() and (
                active == CONTROL and (target == "all" or self.active_object == target)
                or active == "piper_photo" and target == "all")
            if affected:
                self.cancel.set()
                if self.driver is not None and active != "piper_photo":
                    self.driver.cancel_motion()
        return {"state": "stopping" if affected else "idle", "operation_stopping": affected,
                "active_card": active, "active_object": self.active_object,
                "motor_torque_released": False,
                "message": "已请求取消并尽力保持；请在机械臂控制卡读取实际结果" if affected else "当前范围没有执行中的动作"}
