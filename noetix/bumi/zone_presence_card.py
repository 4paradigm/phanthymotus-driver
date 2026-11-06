"""ROS transport and platform lifecycle for the SDK-independent processor."""
import json
import threading

from zone_presence import Occupancy


class ZonePresencePlugin:
    def __init__(self, config, namespace, executor):
        self.executor = executor
        self.namespace = namespace
        self.node = None
        self.lock = threading.RLock()
        self.engine = Occupancy(**{k: v for k, v in config.items() if k != "enabled"})
        self.input_topic = None
        self.event_pub = None

    def get_tool(self):
        return {
            "name": "zone_presence", "type": "processor", "multiInstance": False,
            "description": "服务区域人物占用。机器人固定站立，按 VOP 检测框中心判断画面区域内人数、停留及进入离开事件。坐标左上(-1,-1)、右下(1,1)。",
            "inputSchema": {"type": "object", "properties": {
                "action": {"type": "string", "enum": ["start", "stop", "info", "read"]},
                "input_topic": {"type": "string"}}, "required": ["action"],
                "x-action-params": {"read": {"params": [], "description": "读取最新区域占用状态与持续时间。"}}},
            "configSchema": {"type": "object", "properties": {
                "region": {"type": "array", "items": {"type": "number"}, "minItems": 4, "maxItems": 4,
                           "default": [-.6, -.8, .6, 1], "description": "画面区域[left,top,right,bottom]，每项-1..1"},
                "enter_s": {"type": "number", "default": 2, "minimum": .1, "maximum": 60},
                "leave_s": {"type": "number", "default": 1, "minimum": .1, "maximum": 60},
                "confidence": {"type": "number", "default": .5, "minimum": 0, "maximum": 1}}},
            "topic_in": [{"format": "data/json", "desc": "VOP objects，position为中心相对坐标"}],
            "topic_out": [
                {"topic": f"/{self.namespace}/zone_presence/state", "format": "data/json", "msg_type": "std_msgs/msg/String"},
                {"topic": f"/{self.namespace}/zone_presence/events", "format": "data/json", "msg_type": "std_msgs/msg/String"}]}

    def start(self):
        pass  # The platform supplies its VOP topic through dispatch(start).

    def stop(self):
        with self.lock:
            if self.node is not None:
                self.executor.remove_node(self.node)
                self.node.destroy_node()
                self.node = None
            self.event_pub = None
            self.engine = Occupancy(region=self.engine.region, enter_s=self.engine.enter_s,
                                    leave_s=self.engine.leave_s, confidence=self.engine.confidence)

    def _publish(self, publisher, data):
        from std_msgs.msg import String
        message = String()
        message.data = json.dumps(data, ensure_ascii=False)
        publisher.publish(message)

    def dispatch(self, action, args):
        with self.lock:
            if action == "config":
                if self.node is not None:
                    return {"state": "error", "error": "Stop the Card before changing its region."}
                changes = {k: v for k, v in args.items() if k != "_tool_name"}
                try:
                    self.engine = Occupancy(**{"region": self.engine.region, "enter_s": self.engine.enter_s,
                                               "leave_s": self.engine.leave_s, "confidence": self.engine.confidence,
                                               **changes})
                except (ValueError, TypeError) as exc:
                    return {"state": "error", "error": str(exc)}
                return {"state": "configured"}
            if action == "stop":
                self.stop()
                return {"state": "stopped"}
            if action in ("info", "read"):
                state, event = self.engine.read()
                if event and self.event_pub is not None:
                    self._publish(self.event_pub, event)
                return {**state, "running": self.node is not None, "input_topic": self.input_topic}
            if action != "start":
                return None
            topic = args.get("input_topic")
            if not isinstance(topic, str) or not topic.startswith("/"):
                return {"state": "error", "error": "Connect a VOP output topic first."}
            if self.node is not None:
                return {"state": "running"} if topic == self.input_topic else {
                    "state": "error", "error": "Stop before changing input topic."}
            from rclpy.node import Node
            from rclpy.qos import QoSProfile, ReliabilityPolicy
            from std_msgs.msg import String
            node = Node("zone_presence")
            outputs = self.get_tool()["topic_out"]
            state_pub = node.create_publisher(String, outputs[0]["topic"], 10)
            event_pub = node.create_publisher(String, outputs[1]["topic"], 10)

            def receive(message):
                with self.lock:
                    try:
                        payload = json.loads(message.data)
                    except (ValueError, TypeError):
                        payload = None
                    event = self.engine.update(payload)
                    if event:
                        self._publish(event_pub, event)

            def tick():
                with self.lock:
                    state, event = self.engine.read()
                    self._publish(state_pub, state)
                    if event:
                        self._publish(event_pub, event)

            try:
                node.create_subscription(String, topic, receive,
                    QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
                node.create_timer(.2, tick)
                self.executor.add_node(node)
            except Exception:
                node.destroy_node()
                raise
            self.node, self.input_topic = node, topic
            self.event_pub = event_pub
            return {"state": "running", "input_topic": topic}
