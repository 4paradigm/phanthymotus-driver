"""Automatic pose detector on the shared runtime; owns no control commands."""
from common.vendor_runtime import action_schema, tool
from agilex.piper.monitor_bundle import read_passive_snapshot
from agilex.piper.monitor import PiperMonitorPlugin
from agilex.piper.direct_control import direct_schema, install_direct_control
import threading


class PiperStatusCard:
    def __init__(self, runtime, monitor=None, *, auto_connect=False):
        self.runtime = runtime
        self.monitor = monitor or PiperMonitorPlugin(
            lambda: read_passive_snapshot(runtime),
            namespace=runtime.topic.rsplit("/", 1)[0],
            device_name=runtime.config.get("name", "机械臂 1"))
        self.auto_connect = auto_connect
        self._connection_stop = threading.Event()
        self._connection_thread = None
        self._connection_lock = threading.Lock()
        self._connection_error = None

    def get_tools(self):
        actions = {"start": ([], "开始骨架监控"),
                   "stop": ([], "停止骨架监控"), "info": ([], "查看监控信息")}
        schema = action_schema(actions, {})
        schema.update({"additionalProperties": False})
        schema["properties"]["action"].update(title="操作", default="info",
            oneOf=[{"const": key, "title": label} for key, (_, label) in actions.items()])
        # Core's sensor template always adds its broken detail-stream button.
        # Expose this read-only state as a resource instead: lifecycle and topic
        # publishing remain intact, and its renderer is only the monitor page.
        definition = tool("piper_status", "resource", "自动检测实测六轴位置并持续发布骨架；顶部「监控」显示，不使能、不移动。",
                          schema, topic_out=self.monitor._topics()[:1])
        definition["title"] = "骨架动作检测"
        return [definition, self.monitor.get_tools()[1]]

    def start(self):
        with self._connection_lock:
            if (self._connection_stop.is_set() and self._connection_thread is not None
                    and self._connection_thread.is_alive()):
                raise RuntimeError("只读连接仍在结束，请稍后重启检测")
            self.monitor.start()
            if self.auto_connect and self.runtime.driver is None:
                if self._connection_thread is None or not self._connection_thread.is_alive():
                    self._connection_stop.clear()
                    self._connection_thread = threading.Thread(target=self._connect_shared, daemon=True,
                                                               name="piper-pose-connection")
                    self._connection_thread.start()
        return self.info()

    def stop(self):
        with self._connection_lock:
            self._connection_stop.set()
            self.monitor.stop()
            thread = self._connection_thread
            if thread is not None:
                thread.join(timeout=2)
        return self.info()

    def info(self):
        info = self.monitor.info()
        info["topic_out"] = self.monitor._topics()[:1]
        info["connection_error"] = self._connection_error
        return info

    def _connect_shared(self):
        # The user requested automatic detection without a read button. Only
        # initialize the same shared CAN listener, outside the sampler/ROS timer.
        # Do not acquire the action lock, change permission, or send controls.
        runtime = self.runtime
        while not self._connection_stop.is_set():
            if not runtime.closed and not runtime.gate.locked() and runtime.connection_lock.acquire(blocking=False):
                try:
                    if not self._connection_stop.is_set():
                        runtime.connected()
                        self._connection_error = None
                        return
                except Exception as exc:
                    self._connection_error = str(exc).replace("\n", " ")[:200]
                finally:
                    runtime.connection_lock.release()
            self._connection_stop.wait(2)

    def dispatch(self, action, args):
        if args.get("_tool_name") == "model":
            return self.monitor.dispatch(action, args)
        if set(args) - {"action", "instance_id", "_tool_name"}:
            raise ValueError("状态卡不接受控制参数")
        # Resource calls keep action in args; preserve sensor-call compatibility.
        action = args.get("action", "info" if action == "piper_status" else action)
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
        # The sampler only reads cache; connection initialization is separate.
        return self.monitor.read()


def split_status_card(plugins):
    plugins = list(plugins)
    runtime = next(p for p in plugins if hasattr(p, "read_monitor_snapshot") and hasattr(p, "connected"))
    install_direct_control(runtime)
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
            direct_schema(definition)
        return definitions
    def dispatch(action, args):
        if args.get("_tool_name") == "arm_control" and action == "select_action":
            return {"executed": False, "message": "请在操作下拉框选择要执行的控制动作。"}
        if args.get("_tool_name") == "arm_control" and action == "read":
            raise ValueError("请使用独立的「读取状态」卡")
        if args.get("_tool_name") == "arm_control" and action in {"enter_teaching", "exit_teaching"}:
            raise ValueError("软件示教入口已移除，请使用机身示教按键")
        return original_dispatch(action, args)
    runtime.get_tools = get_tools
    runtime.dispatch = dispatch
    monitors = [p for p in plugins if isinstance(p, PiperMonitorPlugin)]
    if len(monitors) > 1:
        raise ValueError("骨架卡必须绑定唯一监控插件")
    monitor = monitors[0] if monitors else None
    return [*[p for p in plugins if p not in monitors], PiperStatusCard(runtime, monitor, auto_connect=True)]
