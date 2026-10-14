"""Separate manual status card; owns no hardware and no control actions."""
from common.vendor_runtime import action_schema, tool
from agilex.piper.monitor_bundle import read_passive_snapshot
from agilex.piper.monitor import PiperMonitorPlugin


class PiperStatusCard:
    def __init__(self, runtime, monitor=None):
        self.runtime = runtime
        self.monitor = monitor or PiperMonitorPlugin(
            lambda: read_passive_snapshot(runtime),
            namespace=runtime.topic.rsplit("/", 1)[0],
            device_name=runtime.config.get("name", "机械臂 1"))

    def get_tools(self):
        actions = {"read": ([], "读取骨架位置"), "start": ([], "开始骨架监控"),
                   "stop": ([], "停止骨架监控"), "info": ([], "查看监控信息")}
        schema = action_schema(actions, {})
        schema.update({"additionalProperties": False, "x-manual-execution": True})
        schema["properties"]["action"].update(title="操作", default="read",
            oneOf=[{"const": key, "title": label} for key, (_, label) in actions.items()])
        definition = tool("piper_status", "sensor", "读取机械臂实测六轴位置，持续更新骨架；在顶部「监控」查看，不使能、不移动。",
                          schema, topic_out=self.monitor._topics()[:1])
        definition["title"] = "读取骨架位置"
        return [definition, self.monitor.get_tools()[1]]

    def start(self):
        return self.monitor.start()

    def stop(self):
        return self.monitor.stop()

    def info(self):
        info = self.monitor.info()
        info["topic_out"] = self.monitor._topics()[:1]
        return info

    def dispatch(self, action, args):
        if args.get("_tool_name") == "model":
            return self.monitor.dispatch(action, args)
        if set(args) - {"action", "instance_id", "_tool_name"}:
            raise ValueError("状态卡不接受控制参数")
        # DriverBundle removes action from sensor arguments before dispatch.
        action = args.get("action", "read" if action == "piper_status" else action)
        if action in {"start", "stop", "info"}:
            return getattr(self, action)()
        if action != "read":
            raise ValueError("状态卡不支持控制动作")
        runtime = self.runtime
        # Manual read may establish the SAME connection, never a second SDK.
        # Do not use status/read_state: they invalidate previews/cancel moves.
        if not runtime.connection_lock.acquire(blocking=False):
            raise RuntimeError("连接正在变更，请稍后读取")
        try:
            if runtime.closed:
                raise RuntimeError("控制服务已停止，请先启动机械臂控制卡")
            runtime.connected()
        finally:
            runtime.connection_lock.release()
        # Continuous sampling never connects. Only this explicit manual read
        # can initialize the shared connection, then all frames use its cache.
        return self.monitor.read()


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
    monitors = [p for p in plugins if isinstance(p, PiperMonitorPlugin)]
    if len(monitors) > 1:
        raise ValueError("骨架卡必须绑定唯一监控插件")
    monitor = monitors[0] if monitors else None
    return [*[p for p in plugins if p not in monitors], PiperStatusCard(runtime, monitor)]
