"""Stock PhanthyMotus cards for an isolated Teleopit simulation session.

Only the supervisor is called here. Teleopit, MuJoCo and PICO dependencies are
loaded by the worker process, never by the MCP server or the Core DDS context.
"""

from __future__ import annotations

import copy
import json
import threading
import time
from typing import Any

from common.vendor_runtime import action_schema, jsonable, tool
from manager import validate_options


OPTION_PROPERTIES = {
    "source": {
        "type": "string", "enum": ["bvh", "pico"], "default": "bvh",
        "description": "输入来源：BVH 动作回放，或安装 pico-bridge 的 PICO 全身追踪",
    },
    "bvh_path": {
        "type": "string", "default": "",
        "description": "Driver 主机上的 BVH 文件路径；留空使用已安装的官方示例",
    },
    "max_steps": {
        "type": "integer", "minimum": 0, "maximum": 180000, "default": 500,
        "description": "最大策略步数（50 Hz）；0 表示持续运行，直到点击停止",
    },
    "human_height": {
        "type": "number", "minimum": 0.8, "maximum": 2.5, "default": 1.75,
        "description": "操作者身高（米），用于动作重定向",
    },
    "render": {
        "type": "boolean", "default": True,
        "description": "生成 MuJoCo 仿真画面，由仿真预览卡显示",
    },
    "pico_advertise_host": {
        "type": "string", "default": "", "x-sensitive": True,
        "description": "PICO 可达的 Driver 主机局域网 IP；留空由 pico-bridge 自动选择",
    },
}


def _options(arguments: dict[str, Any]) -> dict[str, Any]:
    """Validate only user-editable options; server paths are not MCP arguments."""
    return validate_options({key: value for key, value in arguments.items()
                             if key not in {"instance_id", "_tool_name"}})


class CoreTopicPublisher:
    """JSON/JPEG publishers in the Core domain only; no robot DDS participant."""

    def __init__(self, namespace: str, core_domain_id: int = 42):
        import rclpy
        from rclpy.context import Context
        from rclpy.node import Node
        from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
        from sensor_msgs.msg import CompressedImage
        from std_msgs.msg import String

        self._rclpy = rclpy
        self._string_type = String
        self._image_type = CompressedImage
        self._context = Context()
        self._node = None
        try:
            rclpy.init(context=self._context, domain_id=int(core_domain_id))
            self._node = Node("teleopit_simulation", context=self._context)
            qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                             history=HistoryPolicy.KEEP_LAST, depth=1,
                             durability=DurabilityPolicy.VOLATILE)
            prefix = f"/{namespace}/teleopit"
            self._state = self._node.create_publisher(String, f"{prefix}/state", qos)
            self._preview = self._node.create_publisher(CompressedImage, f"{prefix}/preview", qos)
        except Exception:
            self.close()
            raise

    def publish_state(self, payload: dict[str, Any]) -> None:
        message = self._string_type()
        message.data = json.dumps(jsonable(payload), ensure_ascii=False, allow_nan=False)
        self._state.publish(message)

    def publish_preview(self, jpeg: bytes) -> None:
        message = self._image_type()
        message.header.stamp = self._node.get_clock().now().to_msg()
        message.header.frame_id = "teleopit_simulation"
        message.format = "jpeg"
        message.data = jpeg
        self._preview.publish(message)

    def close(self) -> None:
        if self._node is not None:
            self._node.destroy_node()
            self._node = None
        if self._rclpy.ok(context=self._context):
            self._rclpy.shutdown(context=self._context)


class TeleopitPlugin:
    """One simulation controller and two independently controlled sensor cards."""

    def __init__(self, config: dict, namespace: str, manager: Any, publisher: Any = None):
        self.manager = manager
        self.publisher = publisher
        self._lock = threading.RLock()
        self._ready = False
        self._stopping = False
        self._configured: dict[str, Any] = {}
        self._active = {"teleopit_state": False, "teleopit_preview": False}
        self._topics = {
            "teleopit_state": [{"topic": f"/{namespace}/teleopit/state", "format": "data/json"}],
            "teleopit_preview": [{"topic": f"/{namespace}/teleopit/preview", "format": "image/jpeg"}],
        }
        self._defaults = {key: value for key, value in config.get("teleopit", {}).items()
                          if key in OPTION_PROPERTIES and key != "pico_advertise_host"}
        self._publish_error: str | None = None
        self._closed = threading.Event()
        self._thread: threading.Thread | None = None

    def get_tools(self) -> list[dict]:
        properties = copy.deepcopy(OPTION_PROPERTIES)
        for name, value in self._defaults.items():
            properties[name]["default"] = value
        options = list(properties)
        actions = {
            "start": (options, "启用控制卡并保存配置，不启动仿真"),
            "run": (options, "启动 Teleopit G1 29 自由度 MuJoCo 仿真，绝不向真机输出"),
            "stop": ([], "停止仿真并取消正在进行的模型加载或 PICO 等待"),
            "pause": ([], "暂停当前仿真"),
            "resume": ([], "继续当前仿真"),
            "preflight": (options, "检查 Teleopit、模型、示例动作及输入依赖，不启动仿真"),
            "info": ([], "读取仿真状态、关节目标、延迟和错误原因"),
            "config": (options, "保存下一次运行的设置；不修改正在运行的会话"),
        }
        simulation = tool(
            "teleopit_sim", "actuator",
            "Teleopit 动作仿真：BVH/PICO 全身输入 → 重定向 → 策略推理 → MuJoCo。"
            "仅支持 G1 29 自由度仿真；不控制 G1 真机。启动卡片后点击 run。",
            action_schema(actions, properties),
        )
        simulation["inputSchema"]["additionalProperties"] = False
        simulation["configSchema"] = {
            "type": "object", "properties": copy.deepcopy(properties),
            "additionalProperties": False,
        }
        sensors = []
        for name, description in (
            ("teleopit_state", "Teleopit 仿真状态：会话、29 关节目标、实测仿真关节和处理延迟"),
            ("teleopit_preview", "Teleopit 仿真预览：在卡片内显示 MuJoCo JPEG 画面；需启用 render"),
        ):
            schema = action_schema({
                "start": ([], "开启本卡片数据推送，不启动仿真"),
                "stop": ([], "关闭本卡片数据推送，不停止仿真"),
                "info": ([], "读取推送状态和仿真信息"),
            }, {})
            schema["additionalProperties"] = False
            sensors.append(tool(name, "sensor", description, schema, topic_out=self._topics[name]))
        return [simulation, *sensors]

    def start(self) -> dict:
        # Driver startup is deliberately cheap and never calls manager.run().
        with self._lock:
            if self._closed.is_set():
                return {"state": "error", "error": "Driver 已关闭"}
            self._ready = True
            if self.publisher is not None and self._thread is None:
                self._thread = threading.Thread(target=self._publish_loop, daemon=True,
                                                name="teleopit-card-publisher")
                self._thread.start()
        return {"state": "ready", "hardware_output": False}

    def stop(self) -> dict:
        """Bundle shutdown, not a single card's stop action."""
        self._closed.set()
        with self._lock:
            self._ready = False
            self._active = {key: False for key in self._active}
            thread = self._thread
        result = self.manager.stop()
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=2)
        return result

    def _info(self) -> dict:
        info = dict(self.manager.info())
        with self._lock:
            info.update({
                "card_state": "ready" if self._ready else "idle",
                "configured_options": dict(self._configured),
                "streams": dict(self._active),
                "ros_available": self.publisher is not None,
                "publish_error": self._publish_error,
            })
        info["hardware_output"] = False
        info["profile"] = "g1_29_sim"
        return info

    def _sensor(self, name: str, action: str) -> dict | None:
        if action not in ("start", "stop", "info"):
            return None
        with self._lock:
            if action == "start":
                if self._closed.is_set():
                    return {"state": "error", "error": "Driver 已关闭"}
                if self.publisher is None:
                    return {"state": "unavailable", "ok": False,
                            "error": "ROS 2 推送不可用；请在已配置 Core DDS 的环境中启动 Driver。"
                                     "无 ROS 模式仍可通过控制卡 info 查询仿真。",
                            "topic_out": self._topics[name]}
                self._active[name] = True
            elif action == "stop":
                self._active[name] = False
            running = self._active[name]
        result = {"state": "running" if running else "idle", "topic_out": self._topics[name],
                  "hardware_output": False, "ros_available": self.publisher is not None}
        if action == "info":
            result["simulation"] = self._info()
            if name == "teleopit_preview":
                result["preview_available"] = self.manager.preview() is not None
        return result

    def dispatch(self, action: str, args: dict) -> dict | None:
        name = args.get("_tool_name", "teleopit_sim")
        if name in self._active:
            return self._sensor(name, action)
        if name != "teleopit_sim":
            return None
        if action in ("start", "config"):
            options = _options(args)
            with self._lock:
                if self._stopping:
                    return {"state": "stopping", "adapter_ok": False,
                            "error": "正在停止仿真，请等待停止完成"}
                self._configured.update(options)
                if action == "start":
                    if self._closed.is_set():
                        return {"state": "error", "error": "Driver 已关闭"}
                    self._ready = True
                return {"state": "ready" if self._ready else "idle", "adapter_ok": True,
                        "configured_options": dict(self._configured), "hardware_output": False}
        if action in ("run", "preflight"):
            overrides = _options(args)
            with self._lock:
                ready = self._ready and not self._stopping and not self._closed.is_set()
                options = {**self._configured, **overrides}
                if action == "run":
                    if not ready:
                        return {"state": "idle", "ok": False, "error": "请先启动 Teleopit 控制卡"}
                    # The manager only schedules work here. Hold the small
                    # lifecycle lock until scheduling, so stop cannot be
                    # overtaken by an already-admitted run request.
                    return self.manager.run(options)
            if action == "preflight":
                return self.manager.preflight(options)
        if action == "stop":
            with self._lock:
                self._ready = False
                self._stopping = True
            try:
                return self.manager.stop()
            finally:
                with self._lock:
                    self._stopping = False
        if action == "pause":
            return self.manager.pause()
        if action == "resume":
            return self.manager.resume()
        if action == "info":
            return self._info()
        return None

    def publish_once(self) -> None:
        """One producer → Core transport pass, also usable by deterministic tests."""
        with self._lock:
            active = dict(self._active)
        if self.publisher is None or self._closed.is_set():
            return
        if active["teleopit_state"]:
            payload = self._info()
            payload["published_at_ms"] = int(time.time() * 1000)
            self.publisher.publish_state(payload)
        if active["teleopit_preview"]:
            jpeg = self.manager.preview()
            if jpeg is not None:
                self.publisher.publish_preview(jpeg)

    def _publish_loop(self) -> None:
        while not self._closed.wait(0.2):
            try:
                self.publish_once()
                with self._lock:
                    self._publish_error = None
            except Exception as exc:
                with self._lock:
                    previous = self._publish_error
                    self._publish_error = str(exc)
                if previous != str(exc):
                    print(f"[teleopit] card publishing failed: {exc}", flush=True)
