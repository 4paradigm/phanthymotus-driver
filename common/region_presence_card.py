"""Reusable processor adapter: VOP JSON -> sparse region occupancy events."""
from __future__ import annotations

from dataclasses import asdict
import json
import threading
import time
import uuid

from common.region_presence import PresenceConfig, PresenceMonitor
from common.image_quality import QualityConfig, analyze_jpeg


class RegionPresencePlugin:
    PREFIX = "region_presence"

    def __init__(self, cfg, namespace, executor, camera=None, quality_config=None):
        self._camera = camera
        self._quality = QualityConfig().updated({k: v for k, v in (quality_config or {}).items() if k != "enabled"})
        self._config = PresenceConfig().updated({k: v for k, v in cfg.items() if k != "enabled"})
        self._namespace, self._executor = namespace.strip("/"), executor
        self._topic = f"/{self._namespace}/region_presence/events" if self._namespace else "/region_presence/events"
        self._lock, self._lifecycle = threading.RLock(), threading.RLock()
        self._clock = time.monotonic
        self._node = self._publish = None
        self._attached = self._running = False
        self._generation = self._revision = 0
        self._input = self._publish_error = ""
        self._session = uuid.uuid4().hex
        self._monitor = PresenceMonitor(self._config)

    def get_tool(self):
        defaults = asdict(PresenceConfig())
        props = {
            "target_name": {"type": "string", "title": "目标类别", "description": "VOP 词表中的准确类别名，如 person；不识别身份"},
            "min_confidence": {"type": "number", "minimum": 0, "maximum": 1, "title": "最低置信度"},
            **{k: {"type": "number", "minimum": -1, "maximum": 1,
                   "title": {"x_min": "区域左边界", "x_max": "区域右边界", "y_min": "区域上边界", "y_max": "区域下边界"}[k],
                   "description": "VOP 坐标：画面中心 0，左/上 -1，右/下 +1；按目标中心是否落入区域判断"}
               for k in ("x_min", "x_max", "y_min", "y_max")},
            "enter_hold_s": {"type": "number", "minimum": 0, "maximum": 60, "title": "出现确认时长（秒）"},
            "leave_hold_s": {"type": "number", "minimum": 0, "maximum": 60, "title": "离开确认时长（秒）"},
            "dwell_alert_s": {"type": "number", "minimum": 0, "maximum": 86400, "title": "连续在场提醒（秒，0 关闭）"},
            "capacity_limit": {"type": "integer", "minimum": 0, "maximum": 256, "title": "区域目标数量上限（0 关闭）"},
            "stale_after_s": {"type": "number", "minimum": .2, "maximum": 120, "title": "识别数据超时（秒）"}}
        props = {k: {**v, "default": defaults[k], "scope": "shared"} for k, v in props.items()}
        return {"name": self.PREFIX, "type": "processor", "multiInstance": False,
                "description": "区域目标在场：复用 VOP 检测，稳定判断出现/离开，变化时输出事件。不追踪身份，不累计进出人数；断流为未知。",
                "inputSchema": {"type": "object", "properties": {
                    "action": {"type": "string", "enum": ["start", "check", "target_guidance", "scan_guidance", "info", "stop", "config"]},
                    "input_topic": {"type": "string", "description": "连线解析的一路 VOP objects JSON Topic"},
                    **{k: {a: b for a, b in v.items() if a != "scope"} for k, v in props.items()}},
                    "required": ["action"], "x-action-params": {
                        "start": {"params": ["input_topic"], "description": "绑定一路 VOP data/json 检测流"},
                        "scan_guidance": {"params": [], "description": "查看 Bumi 彩色相机最新画面，给出二维码位置或亮度调整提示；无需启动 VOP"},
                        "target_guidance": {"params": [], "description": "读取 VOP 目标位置引导和本机彩色相机图像质量，返回可播报的调整提示"},
                        "check": {"params": [], "description": "读取区域在场状态；present=null 表示待确认、无数据或断流，不表示无人"},
                        "info": {"params": [], "description": "查看参数、输入输出与处理状态"},
                        "stop": {"params": [], "description": "停止本卡，不停止相机或 VOP"},
                        "config": {"params": list(props), "description": "原子更新区域与确认时长，重新等待稳定观测"}}},
                "configSchema": {"type": "object", "properties": props},
                "topic_in": [{"format": "data/json", "desc": "VOP objects（非 pose 或电量 JSON）"}],
                "topic_out": [{"topic": self._topic, "format": "data/json", "desc": "在场变化、停留达标、数量超限及恢复事件"}]}

    def start(self):
        return True  # Registration only; canvas start supplies the source.

    def _make_node(self, topic, generation):
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
        from std_msgs.msg import String
        node = Node(self.PREFIX, namespace="/" + self._namespace if self._namespace else "/")
        try:
            publisher = node.create_publisher(String, self._topic, 10)
            qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                             history=HistoryPolicy.KEEP_LAST, depth=1,
                             durability=DurabilityPolicy.VOLATILE)
            node.create_subscription(String, topic, lambda msg: self._on_message(msg, generation), qos)
            node.create_timer(.2, lambda: self._tick(generation))
            return node, lambda data: publisher.publish(String(data=json.dumps(data, ensure_ascii=False, allow_nan=False)))
        except Exception:
            node.destroy_node()
            raise

    def _emit(self, events):
        for event in events:
            payload = {**event, "event_id": f"{self._session}:{self._revision}:{event['sequence']}",
                       "timestamp": time.time(), "input_topic": self._input,
                       "config_revision": self._revision,
                       "freshness_basis": "local_receive_time_of_new_publication",
                       "capture_time_verified": False}
            try:
                self._publish(payload)
                self._publish_error = ""
            except Exception as error:
                self._publish_error = str(error)[:240]

    def _on_message(self, msg, generation):
        with self._lock:
            if not self._running or generation != self._generation:
                return
            try:
                if not isinstance(msg.data, str) or len(msg.data.encode("utf-8")) > 262144:
                    raise ValueError("VOP JSON must be text of at most 256 KiB")
                payload = json.loads(msg.data)
            except (ValueError, TypeError, AttributeError, RecursionError) as error:
                self._emit(self._monitor.unavailable("invalid_data", str(error)[:240]))
                return
            self._emit(self._monitor.ingest(payload, self._clock()))

    def _tick(self, generation):
        with self._lock:
            if self._running and generation == self._generation:
                self._emit(self._monitor.poll(self._clock()))

    def check(self):
        with self._lock:
            if self._running:
                self._emit(self._monitor.poll(self._clock()))
            result = self._monitor.snapshot(self._clock())
            if not self._running:
                result.update(state="idle", present=None)
            return {**result, "input_topic": self._input, "config_revision": self._revision}

    def info(self):
        with self._lock:
            return {"state": "running" if self._running else "idle", "input_topic": self._input,
                    "topic_in": [{"topic": self._input, "format": "data/json"}] if self._input else [],
                    "topic_out": self.get_tool()["topic_out"], "parameters": asdict(self._config),
                    "publish_error": self._publish_error, "observation": self.check()}

    def _start_stream(self, topic):
        if (not isinstance(topic, str) or not topic.startswith("/") or topic == "/"
                or any(c.isspace() for c in topic) or topic == self._topic):
            return {"state": "error", "message": "Connect exactly one VOP objects output (absolute input_topic)"}
        with self._lifecycle:
            if self._running and self._input == topic:
                return self.info()
            result = self.stop()
            if result["state"] != "idle":
                return result
            try:
                with self._lock:
                    self._generation += 1
                    self._monitor = PresenceMonitor(self._config)
                    self._session = uuid.uuid4().hex
                    self._input, self._publish_error = topic, ""
                    self._node, self._publish = self._make_node(topic, self._generation)
                self._attached = True
                if self._executor.add_node(self._node) is False:
                    raise RuntimeError("Executor refused the region_presence node")
                with self._lock:
                    self._running = True
                return self.info()
            except Exception as error:
                cleanup = self.stop()
                return {"state": "error", "message": f"Cannot start region_presence: {error}", "cleanup": cleanup}

    def stop(self):
        with self._lifecycle:
            with self._lock:
                if self._running:
                    self._emit(self._monitor.unavailable("idle", "Region monitor stopped"))
                self._running = False
                self._generation += 1
                self._publish = None
                self._monitor = PresenceMonitor(self._config)
            # Retain a failed cleanup handle; a new start must not leak it.
            try:
                if self._node is not None:
                    if self._attached:
                        self._executor.remove_node(self._node)
                        self._attached = False
                    self._node.destroy_node()
                    self._node = None
            except Exception as error:
                return {"state": "error", "message": f"Cannot release region_presence node: {error}"}
            return {"state": "idle"}

    def _analyze_frame(self, analyze, max_age):
        try:
            if self._camera is None or not self._camera.is_running():
                return {"state": "unavailable", "message": "相机未运行"}
            frame, sequence = self._camera.wait_for_color_frame(timeout_s=0)
            if not frame or self._clock() - frame["received_monotonic"] > max_age:
                return {"state": "unavailable", "message": "暂无新鲜画面"}
            return {**analyze(frame["data"]), "frame_sequence": sequence}
        except Exception as error:
            return {"state": "error", "message": f"无法分析画面：{error}"}

    def dispatch(self, action, args):
        if action == "scan_guidance":
            from common.region_presence import qr_guidance
            return self._analyze_frame(qr_guidance, 2)
        if action == "target_guidance":
            quality = self._analyze_frame(lambda data: analyze_jpeg(data, self._quality), self._quality.max_frame_age_s)
            observation = self.check()
            position = observation["position_guidance"]
            messages = {"underexposed": "画面偏暗，请增加照明。",
                        "overexposed": "画面过亮，请避开强光或反光。",
                        "low_sharpness": "画面细节偏少，请保持目标稳定并检查对焦。"}
            hints = [messages[issue] for issue in quality.get("issues", [])] + position["hints"]
            if position["state"] == "aligned" and quality.get("acceptable"):
                hints.append("目标已对齐，画面质量合适。")
            return {"position_guidance": position, "image_quality": quality, "hints": hints,
                    "target_name": observation["target_name"], "source_timestamp": observation["source_timestamp"]}
        if action == "start":
            topics = args.get("input_topics")
            if topics is not None:
                if not isinstance(topics, list) or len(topics) != 1:
                    return {"state": "error", "message": "Exactly one VOP input is supported"}
                if args.get("input_topic") and args["input_topic"] != topics[0]:
                    return {"state": "error", "message": "input_topic disagrees with input_topics"}
            return self._start_stream(topics[0] if topics else args.get("input_topic", ""))
        if action == "check":
            return self.check()
        if action == "info":
            return self.info()
        if action == "stop":
            return self.stop()
        if action == "config":
            changes = {k: v for k, v in args.items() if k not in ("_tool_name", "instance_id")}
            with self._lifecycle, self._lock:
                try:
                    config = self._config.updated(changes)
                except ValueError as error:
                    return {"state": "error", "message": str(error)}
                if self._running:
                    self._emit(self._monitor.unavailable("reconfigured", "Region settings changed"))
                self._config, self._revision = config, self._revision + 1
                self._monitor = PresenceMonitor(config)
                return {"state": "configured", "parameters": asdict(config)}
        return None
