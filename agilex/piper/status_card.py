"""Separate manual status card; owns no hardware and no control actions."""
from common.vendor_runtime import action_schema, tool
from agilex.piper.monitor_bundle import read_passive_snapshot
from agilex.piper.monitor import monitor_frame
import time


class PiperStatusCard:
    def __init__(self, runtime):
        self.runtime = runtime
        self.running = True
        self.last_valid_time = None

    def get_tools(self):
        actions = {"read": ([], "读取状态"), "start": ([], "启动状态卡"),
                   "stop": ([], "停止状态卡"), "info": ([], "查看状态卡信息")}
        schema = action_schema(actions, {})
        schema.update({"additionalProperties": False, "x-manual-execution": True})
        schema["properties"]["action"].update(title="操作", default="read",
            oneOf=[{"const": key, "title": label} for key, (_, label) in actions.items()])
        definition = tool("piper_status", "resource", "单独读取模式、使能、角度、限位和夹爪状态；不使能、不移动。", schema)
        definition["title"] = "读取状态"
        return [definition]

    def start(self):
        self.running = True
        return self.info()

    def stop(self):
        self.running = False
        return self.info()

    def info(self):
        return {"state": "ready" if self.running else "idle", "topic_out": []}

    def dispatch(self, action, args):
        if set(args) - {"action", "instance_id", "_tool_name"}:
            raise ValueError("状态卡不接受控制参数")
        action = args.get("action", "read")
        if action in {"start", "stop", "info"}:
            return getattr(self, action)()
        if action != "read":
            raise ValueError("状态卡不支持控制动作")
        if not self.running:
            raise RuntimeError("请先启动状态卡")
        runtime = self.runtime
        # Manual read may establish the SAME connection, never a second SDK.
        # Do not use status/read_state: they invalidate previews/cancel moves.
        if not runtime.connection_lock.acquire(blocking=False):
            raise RuntimeError("连接正在变更，请稍后读取")
        try:
            if runtime.closed:
                raise RuntimeError("控制服务已停止，请先启动机械臂控制卡")
            runtime.connected()
            data = read_passive_snapshot(runtime)
        finally:
            runtime.connection_lock.release()
        _, parameters, self.last_valid_time = monitor_frame(data, now=time.time(),
            device_name=runtime.config.get("name", "机械臂 1"), last_valid_time=self.last_valid_time)
        return parameters


def split_status_card(plugins):
    plugins = list(plugins)
    runtime = next(p for p in plugins if hasattr(p, "read_monitor_snapshot") and hasattr(p, "connected"))
    original_tools = runtime.get_tools
    original_dispatch = runtime.dispatch
    def get_tools():
        from copy import deepcopy
        definitions = deepcopy(original_tools())
        for definition in definitions:
            if definition["name"] != "arm_control":
                continue
            schema = definition["inputSchema"]
            prop = schema["properties"]["action"]
            prop["enum"] = ["select_action", *[x for x in prop["enum"] if x != "read"]]
            choices = [x for x in prop.get("oneOf", []) if x["const"] != "read"]
            prop["oneOf"] = [{"const": "select_action", "title": "请选择控制动作"}, *choices]
            prop["default"] = "select_action"
            schema.get("x-action-params", {}).pop("read", None)
            schema.setdefault("x-action-params", {})["select_action"] = {"params": [], "description": "请选择控制动作"}
        return definitions
    def dispatch(action, args):
        if args.get("_tool_name") == "arm_control" and action == "select_action":
            return {"executed": False, "message": "请在操作下拉框选择要执行的控制动作。"}
        if args.get("_tool_name") == "arm_control" and action == "read":
            raise ValueError("请使用独立的「读取状态」卡")
        return original_dispatch(action, args)
    runtime.get_tools = get_tools
    runtime.dispatch = dispatch
    return [*plugins, PiperStatusCard(runtime)]
