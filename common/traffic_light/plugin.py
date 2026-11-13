"""VOP-style multi-instance processor. One VOP wire infers its existing JPEG wire."""
from copy import deepcopy
import json
import logging
import re
import threading
import uuid

from .processor import TrafficLightProcessor
from .schemas import Config

log = logging.getLogger(__name__)


def topics_for(input_topic):
    if (not isinstance(input_topic, str) or len(input_topic) > 512
            or not re.fullmatch(r"/(?:[A-Za-z_][A-Za-z0-9_]*/)*[A-Za-z_][A-Za-z0-9_]*", input_topic)
            or not input_topic.endswith("/objects") or len(input_topic) <= len("/objects")):
        raise ValueError("input_topic must be a VOP /.../objects topic")
    return input_topic[:-len("/objects")], input_topic + "/traffic_light"


class _Runtime:
    def __init__(self, executor, topic, config):
        from rclpy.node import Node

        rgb, output = topics_for(topic)
        self.topic, self.rgb, self.output = topic, rgb, output
        self.lock = threading.RLock()
        self.running = True
        self.disposal = None
        self.node = Node("traffic_light_" + uuid.uuid4().hex[:12])
        self.publisher = None
        self.executor = executor
        self.engine = TrafficLightProcessor(config,
            ros_clock=lambda: self.node.get_clock().now().nanoseconds,
            not_before_ns=self.node.get_clock().now().nanoseconds)
        self._published_state = None

    def start(self):
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
        from sensor_msgs.msg import CompressedImage
        from std_msgs.msg import String
        self.String = String
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=2,
                         durability=DurabilityPolicy.VOLATILE)
        try:
            self.publisher = self.node.create_publisher(String, self.output, qos)
            self.node.create_subscription(CompressedImage, self.rgb, self.image, qos)
            self.node.create_subscription(String, self.topic, self.detection, qos)
            self.node.create_timer(.2, self.tick)
            self.executor.add_node(self.node)
        except Exception:
            self.close()
            raise

    def publish(self, result):
        if self.running:
            self.publisher.publish(self.String(data=json.dumps(result, ensure_ascii=False, allow_nan=False)))
            self._published_state = result["state"]

    def image(self, msg):
        with self.lock:
            if not self.running:
                return
            try:
                stamp = msg.header.stamp
                for result in self.engine.image(msg.data, {"sec": stamp.sec, "nanosec": stamp.nanosec}):
                    self.publish(result)
            except Exception as exc:
                self.publish(self.engine.error("rgb_input_failed: " + str(exc)))

    def detection(self, msg):
        with self.lock:
            if not self.running:
                return
            try:
                if len(msg.data) > 262144:
                    raise ValueError("VOP JSON exceeds 256 KiB")
                result = self.engine.detection(json.loads(msg.data))
                self.publish(result)
            except Exception as exc:
                self.publish(self.engine.error("vop_input_failed: " + str(exc)))

    def tick(self):
        with self.lock:
            if self.running:
                result = self.engine.latest()
                if result["state"] != self._published_state:
                    self.publish(result)

    def close(self):
        from ._ros_lifecycle import dispose_node
        with self.lock:
            self.running = False
            # Callbacks already acquired the lock have finished; late callbacks no-op.
            self.disposal = dispose_node(self.executor, self.node, label="traffic_light",
                                         progress=self.disposal)
            self.engine.frames.clear()
            self.engine.pending.clear()
            self.engine.tracks.clear()
            self.engine.error("observation_stopped")
            return self.disposal.complete


class TrafficLightPlugin:
    PREFIX = "traffic_light"

    def __init__(self, plugin_cfg, namespace, executor):
        self._config = Config.from_dict(plugin_cfg)
        self._executor = executor
        self._instances = {}
        self._configs = {}
        self._bindings = {}
        self._errors = {}
        self._lock = threading.RLock()

    def start(self):
        """Bundle boot only; source subscriptions require Card dispatch(start)."""
        pass

    def stop(self):
        """Bundle shutdown releases every instance using the same Card path."""
        return self.shutdown()

    def get_tools(self):
        return [{"name": self.PREFIX, "type": "processor", "multiInstance": True,
                 "description": "机动车交通灯观察：逐目标灯色与圆灯/箭头分析，3/5 独立源帧确认；不控制机器人。",
                 "inputSchema": {"type": "object", "properties": {
                     "action": {"type": "string", "enum": ["start", "stop", "info", "latest", "config",
                                                                  "observe", "finish_observation"]},
                     "input_topic": {"type": "string", "description": "连接 VOP 的 /.../objects；RGB 自动推导"},
                     "instance_id": {"type": "string", "x-ui-hidden": True}},
                     "required": ["action"],
                     "x-hooks": {"on_interrupt_all": {"action": "stop"},
                                 "on_interrupt_motion": {"action": "stop"}},
                     "x-action-params": {
                         "observe": {"params": [], "description": "在画布已绑定的VOP源上开始/恢复观察；不启动相机，不控制身体"},
                         "finish_observation": {"params": [], "description": "结束本实例观察并释放订阅、缓存；保留画布输入绑定供复看"},
                         "latest": {"params": [], "description": "读取所有交通灯当前结果与运行信息；过期或不确定时明确未知"}}},
                 "configSchema": {"type": "object", "properties": {
                     "max_age_s": {"type": "number", "minimum": .05, "maximum": 5, "default": 1., "scope": "instance"},
                     "cache_size": {"type": "integer", "minimum": 2, "maximum": 180, "default": 64, "scope": "instance"},
                     "min_detection_confidence": {"type": "number", "minimum": 0, "maximum": 1, "default": .3, "scope": "instance"}}},
                 "topic_in": [{"format": "data/json", "desc": "VOP objects，必须包含源图像 stamp"}],
                 "topic_out": [{"format": "data/json", "desc": "逐信号灯和灯泡的观察结果"}]}]

    def dispatch(self, name, args):
        if not isinstance(args, dict):
            return {"state": "error", "error": "arguments must be an object"}
        action = args.get("action", name)
        with self._lock:
            instance_id = args.get("instance_id")
            if instance_id is not None and (not isinstance(instance_id, str) or len(instance_id) > 200):
                return {"state": "error", "error": "invalid instance_id"}
            if not instance_id:
                if len(self._instances) == 1 and action != "start":
                    instance_id = next(iter(self._instances))
                elif not self._instances and len(self._bindings) == 1 and action in ("observe", "latest", "finish_observation"):
                    instance_id = next(iter(self._bindings))
                elif len(self._instances) > 1 and action not in ("stop", "start"):
                    return {"state": "error", "error": "instance_id required with multiple instances"}
                else:
                    candidate = args.get("input_topic")
                    instance_id = candidate if isinstance(candidate, str) and candidate else "_default"
            current = self._instances.get(instance_id)
            try:
                if action == "observe":
                    # Core deliberately hides processor system actions from LLMs.
                    # This domain operation reuses the same lifecycle and only an
                    # existing canvas binding; it cannot invent a new camera source.
                    bound = self._bindings.get(instance_id)
                    if not bound:
                        raise ValueError("wire and start this traffic_light card on the canvas first")
                    args = {**args, "input_topic": bound}
                    action = "start"
                elif action == "finish_observation":
                    args = {**args, "instance_id": instance_id}
                    action = "stop"
                if action == "config":
                    if current:
                        raise ValueError("stop the instance before changing configuration")
                    changes = args.get("config", args)
                    if len(self._configs) >= 32 and instance_id not in self._configs:
                        raise ValueError("at most 32 instance configurations")
                    values = {**self._config.__dict__, **self._configs.get(instance_id, {}),
                              **{k: v for k, v in changes.items() if k in Config.__dataclass_fields__}}
                    self._configs[instance_id] = Config.from_dict(values).__dict__
                    return {"state": "idle", "config": deepcopy(self._configs[instance_id])}
                if action == "start":
                    inputs = args.get("input_topics") or []
                    if not isinstance(inputs, list) or len(inputs) > 1:
                        raise ValueError("connect exactly one VOP objects output")
                    topic = args.get("input_topic") or (inputs[0] if len(inputs) == 1 else None) or self._bindings.get(instance_id)
                    topics_for(topic)
                    if len(self._bindings) >= 32 and instance_id not in self._bindings:
                        raise ValueError("at most 32 saved traffic_light bindings")
                    self._bindings[instance_id] = topic
                    if current:
                        if current.topic != topic or not current.running:
                            raise ValueError("stop the existing binding before restart")
                        return self._info(current)
                    if len(self._instances) >= 8:
                        raise ValueError("at most eight traffic_light instances")
                    self._instances[instance_id] = current = _Runtime(
                        self._executor, topic, self._configs.get(instance_id, self._config.__dict__))
                    current.start()
                    self._errors.pop(instance_id, None)
                    return self._info(current)
                if action == "stop":
                    keys = [instance_id] if args.get("instance_id") else list(self._instances)
                    failures = []
                    for key in keys:
                        runtime = self._instances.get(key)
                        if runtime and runtime.close():
                            del self._instances[key]
                        elif runtime:
                            failures.append(key)
                    return {"state": "error" if failures else "idle", "cleanup_pending": failures}
                if action == "info":
                    if current:
                        return self._info(current)
                    topic = args.get("input_topic") or self._bindings.get(instance_id)
                    rgb, output = topics_for(topic) if topic else (None, None)
                    return {"state": "idle", "topic_in": ([{"topic": topic, "format": "data/json"}] if topic else []),
                            "topic_out": ([{"topic": output, "format": "data/json"}] if output else []),
                            "rgb_topic": rgb, "last_image_time": None, "last_detection_time": None,
                            "has_synchronized_frame": False, "error": self._errors.get(instance_id)}
                if action == "latest":
                    if current:
                        with current.lock:
                            return {**current.engine.latest(), "runtime": self._info(current)}
                    return {"state": "waiting", "timestamp": None, "frame_timestamp": None,
                            "signals": [], "reason": "instance_not_started",
                            "runtime": self.dispatch("info", {"instance_id": instance_id})}
            except Exception as exc:
                self._errors[instance_id] = str(exc)
                log.warning("traffic_light %s failed: %s", action, exc)
                return {"state": "error", "error": str(exc)}
            return None

    @staticmethod
    def _info(runtime):
        with runtime.lock:
            result = {"state": "running" if runtime.running else "stopping",
                    "topic_in": [{"topic": runtime.topic, "format": "data/json"}],
                    "topic_out": [{"topic": runtime.output, "format": "data/json"}],
                    "rgb_topic": runtime.rgb, **runtime.engine.info()}
            if runtime.disposal and not runtime.disposal.complete:
                result["error"] = runtime.disposal.error
            return result

    def shutdown(self):
        return self.dispatch("stop", {"action": "stop"})
