"""Dual configuration routes the existing ten cards and adds two paired cards."""
import copy
from pathlib import Path
import re
import threading

from agilex.piper.arm import PiperDriver
from agilex.piper.cards import build_tools, choice, flag
from agilex.piper.device import PiperPlugin
from agilex.piper.paired_motion import PairedMotion
from agilex.piper.teleop_bus import TeleopBus
from common.teleop_contract import STATE_TOPIC, COMMAND_FORMAT, FEEDBACK_FORMAT
from common.vendor_runtime import action_schema, tool

SIDES = ("left", "right")


def dual_tools(topic):
    result = build_tools(topic)
    side_property = choice("机械臂", [("left", "左臂"), ("right", "右臂"), ("all", "双臂（状态/停止）")], "left")
    for definition in result:
        if definition["name"] == "piper_photo":
            continue
        schema = definition["inputSchema"]
        schema["properties"]["arm_side"] = copy.deepcopy(side_property)
        # No backend default: an old single-arm canvas must select its intended arm.
        schema["properties"]["arm_side"].pop("default")
        for action, contract in schema["x-action-params"].items():
            if action not in {"start", "stop", "info"}:
                contract["params"] = ["arm_side", *contract["params"]]
    hidden = {"execute": flag("执行已确认的计划", True),
              "preview_id": {"type": "string", "title": "预览编号", "x-ui-hidden": True},
              "workspace_clear": flag("双臂空间清空且可及时停止", True),
              "take_control": flag("确认接管双臂", True)}
    lifecycle = {"start": ([], "准备卡片，不使能或移动"), "stop": ([], "停止并保持双臂，不失能"),
                 "info": ([], "读取会话、录制及完成结果")}
    for name, title, actions, properties in [
        ("piper_teleop", "双臂 VR 遥操", {"begin": (list(hidden), "预览并启用遥操会话"), "finish": ([], "结束遥操并保持")},
         {**hidden, "input_topic": {"type": "string", "title": "设备输入"}}),
        ("piper_motion", "动作录制与回放", {"record_start": ([], "开始记录双臂与夹爪轨迹"),
            "record_stop": ([], "结束录制并保存"), "list": ([], "列出录制文件"),
            "inspect": (["filename"], "检查轨迹文件"),
            "play": (["filename", *hidden], "预览并单次原速回放"), "halt": ([], "停止回放并保持")},
         {**hidden, "filename": {"type": "string", "title": "录制文件名"}}),
    ]:
        schema = action_schema({**actions, **lifecycle}, properties)
        schema["additionalProperties"] = False
        schema["x-manual-execution"] = True
        schema["x-resource"] = "arm"
        schema["x-is-dangerous"] = True
        schema["x-hooks"] = {"on_interrupt_motion": {"action": "stop"}}
        schema["properties"]["action"].update(title="操作", default=next(iter(actions)),
            oneOf=[{"const": a, "title": text} for a, (_, text) in {**actions, **lifecycle}.items()])
        action = "begin" if name == "piper_teleop" else "play"
        schema["x-confirmation"] = {action: {"preview": {"action": action, "execute": False},
            "execute": {"execute": True, "workspace_clear": True, "take_control": True},
            "message": "核对左右机械臂、夹爪、起始姿态和活动空间。确认其他控制程序已停止，并可立即停止双臂。"}}
        if name == "piper_teleop":
            schema["x-action-params"]["start"]["params"] = ["input_topic"]
        if name == "piper_motion":
            schema["x-completion"] = {"actions": ["play"], "timeout": 610}
        definition = tool(name, "actuator", title + "；不会自动切模式、使能或回起点。", schema,
                          topic_in=[{"format": COMMAND_FORMAT}] if name == "piper_teleop" else None,
                          topic_out=[{"topic": STATE_TOPIC, "format": FEEDBACK_FORMAT}] if name == "piper_teleop" else None)
        definition["title"] = title
        result.append(definition)
    return result


class DualPiperPlugin:
    def __init__(self, config, namespace="piper", ros2=None, *, driver_factory=PiperDriver, bus_factory=TeleopBus, **motion_options):
        definitions = config.get("arms")
        if not isinstance(definitions, dict) or set(definitions) != set(SIDES):
            raise ValueError("dual_mode_requires_left_and_right_arms")
        if any(not isinstance(definitions[s], dict) for s in SIDES):
            raise ValueError("invalid_arm_configuration")
        interfaces = [definitions[s].get("can_interface") for s in SIDES]
        if any(not isinstance(v, str) or not re.fullmatch(r"[A-Za-z0-9_-]{1,15}", v) for v in interfaces):
            raise ValueError("invalid_can_interface")
        if len(set(interfaces)) != 2:
            raise ValueError("duplicate_can_binding")
        if any(definitions[s].get("can_transport", config.get("can_transport", "socketcan")) != "socketcan" for s in SIDES):
            raise ValueError("dual_mode_requires_socketcan")
        self.arms = {}
        root = Path(config.get("data_dir", "/opt/phanthy-motus/data/piper"))
        for side in SIDES:
            child = {**config, **definitions[side], "data_dir": str(root if side == "left" else root / "right")}
            child.pop("arms", None)
            self.arms[side] = PiperPlugin(child, namespace if side == "left" else namespace + "_right", ros2, driver_factory)
            self.arms[side].motion_enabled = False
        self.motion = PairedMotion(self.arms, config.get("calibration_path"), root, **motion_options)
        self.ros2, self.bus_factory, self.bus = ros2, bus_factory, None
        self.input_topic = None
        self.lock = threading.RLock()
        self.stopped = set()
        self.closed = False

    def get_tools(self):
        definitions = dual_tools(self.arms["left"].topic)
        if self.input_topic:
            next(d for d in definitions if d["name"] == "piper_teleop")["topic_in"][0]["topic"] = self.input_topic
        return definitions

    def start(self):
        with self.lock:
            self.closed = False
            self.stopped.clear()
            for arm in self.arms.values():
                arm.start()

    def stop(self):
        with self.lock:
            self.closed = True
            self.motion.close()
            if self.bus is not None:
                self.bus.close()
                self.bus = None
            self.input_topic = None
            for arm in self.arms.values():
                arm.stop()

    def _bus(self):
        with self.lock:
            if not self.input_topic:
                raise RuntimeError("input_topic_required")
            if self.bus is None:
                if self.ros2 is None:
                    raise RuntimeError("teleop_requires_ros2_input")
                self.bus = self.bus_factory(self.ros2, self.motion, self.input_topic)
            if not self.bus.status()["running"]:
                raise RuntimeError("teleop_dds_unhealthy_restart_card")

    def _validate(self, name, action, args):
        definition = next((d for d in self.get_tools() if d["name"] == name), None)
        if definition is None:
            raise ValueError("unknown_piper_card")
        props = definition["inputSchema"]["properties"]
        if action not in props["action"]["enum"]:
            raise ValueError("invalid_piper_action")
        for key, value in args.items():
            if key in {"_tool_name", "action", "instance_id"}:
                continue
            if key not in props:
                raise ValueError("unknown_piper_argument_" + key)
            prop = props[key]
            if prop["type"] == "boolean" and type(value) is not bool:
                raise ValueError("invalid_" + key)
            if prop["type"] == "string" and not isinstance(value, str):
                raise ValueError("invalid_" + key)
            if "enum" in prop and value not in prop["enum"]:
                raise ValueError("invalid_" + key)
        if "input_topic" in args:
            topic = args["input_topic"]
            if len(topic) > 255 or not re.fullmatch(r"(?:/[A-Za-z_][A-Za-z0-9_]*)+", topic):
                raise ValueError("invalid_input_topic")

    def dispatch(self, action, args):
        name = args.get("_tool_name", "piper_status")
        if name == "piper_status":
            action = args.get("action", "read")
        self._validate(name, action, args)
        paired = name in {"piper_teleop", "piper_motion"}
        if action == "start" and paired:
            with self.lock:
                if self.closed:
                    raise RuntimeError("piper_driver_closed")
                self.stopped.discard(name)
                if name == "piper_teleop":
                    topic = args.get("input_topic")
                    if not topic:
                        self.motion.request_stop("input_topic_missing")
                        if self.bus is not None:
                            self.bus.close()
                            self.bus = None
                        self.input_topic = None
                        return {"state": "waiting_binding", "reason": "input_topic_required", "executed": False}
                    if self.bus is not None and topic != self.input_topic:
                        raise RuntimeError("stop_before_rebinding_input_topic")
                    self.input_topic = topic
                    self._bus()
            return {"state": "ready", "executed": False}
        if action == "stop" and paired:
            with self.lock:
                self.stopped.add(name)
                result = self.motion.request_stop("card_stopped")
                if name == "piper_teleop" and self.bus is not None:
                    self.bus.close()
                    self.bus = None
                if name == "piper_teleop":
                    self.input_topic = None
            return result
        if paired:
            if action == "info":
                return {**self.motion.status(),
                        "topic_in": [{"format": COMMAND_FORMAT, **({"topic": self.input_topic} if self.input_topic else {})}] if name == "piper_teleop" else [],
                        "topic_out": [{"topic": STATE_TOPIC, "format": FEEDBACK_FORMAT}] if name == "piper_teleop" else [],
                        "transport": self.bus.status() if self.bus else None}
            if action in {"finish", "halt"}:
                return self.motion.request_stop()
            if action == "record_stop":
                return self.motion.record_stop()
            if self.closed or name in self.stopped:
                raise RuntimeError("piper_card_stopped")
            if action == "begin":
                with self.lock:
                    if self.closed or name in self.stopped:
                        raise RuntimeError("piper_card_stopped")
                    self._bus()
                    return self.motion.prepare("teleop", args)
            if action == "play":
                with self.lock:
                    if self.closed or name in self.stopped:
                        raise RuntimeError("piper_card_stopped")
                    return self.motion.prepare("replay", args)
            if action == "record_start":
                return self.motion.record_start()
            if action == "list":
                return {"recordings": self.motion.recordings()}
            if action == "inspect":
                return self.motion.inspect_recording(args.get("filename"))
        side = args.get("arm_side")
        if side is None:
            if name == "piper_photo":
                side = "left"
            elif action in {"start", "stop", "info"}:
                side = "all"
            else:
                raise ValueError("select_left_or_right_arm")
        if side == "all" and not (action in {"start", "stop", "info"} or name == "piper_status" and action == "read"
                                  or name == "piper_stop" or name == "piper_permission" and action == "block"):
            raise ValueError("this_action_requires_one_arm")
        paired_stop = None
        if (name == "piper_stop" and action == "halt" or name == "piper_permission" and action == "block"
                or action == "stop" and name not in {"piper_photo", "piper_status"}):
            paired_stop = self.motion.request_stop("operator_stop" if action != "stop" else "card_stopped")
        forwarded = {k: v for k, v in args.items() if k != "arm_side"}
        selected = SIDES if side == "all" else (side,)
        if name == "piper_status" and action == "read":
            result = {}
            for s in selected:
                try:
                    result[s] = self.arms[s].dispatch(action, forwarded)
                except Exception as exc:
                    result[s] = {"feedback_fresh": False, "error": str(exc)}
                    self.motion.request_stop(s + "_feedback_failed")
            return {"arms": result, "paired_motion": self.motion.status()}
        result = {s: self.arms[s].dispatch(action, forwarded) for s in selected}
        if paired_stop is not None:
            return {"arms": result, "paired_motion": paired_stop}
        return {"arms": result} if side == "all" else {"arm_side": side, **result[side]}
