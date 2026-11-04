"""Joint skeleton and safe-skin cards sharing one feedback stream."""

from __future__ import annotations

import json
import math
import threading
import time

from common.vendor_runtime import action_schema, tool

if __package__:
    from .feedback import FeedbackClient, positive_number
    from .model import JOINT_NAMES, model_resource, resolve_model
    from .control import PARTS, SafeSkinController
else:
    from feedback import FeedbackClient, positive_number
    from model import JOINT_NAMES, model_resource, resolve_model
    from control import PARTS, SafeSkinController


CARD_DESCRIPTIONS = {
    "joints": "CR3A 六轴实时骨架，固定直角连接保持真实关节坐标和六个可动轴",
    "safe_skin": "小臂及 J4/J5/J6 安全皮肤接近暂停标志、数据有效性和最近控制请求",
}


class DobotSensors:
    def __init__(self, config, namespace="dobot_cra", ros2=None, client=None, controller=None):
        self.client = client if client is not None else FeedbackClient(config)
        self.control = controller if controller is not None else SafeSkinController(self.client, config)
        self._hz = positive_number(config.get("publish_hz", 10), "publish_hz")
        if self._hz > 125:
            raise ValueError("publish_hz must not exceed the 125 Hz feedback rate")
        self._ros2 = ros2
        self._node = None
        self._publishers = {}
        self._message_type = None
        self._lifecycle = threading.RLock()
        prefix = namespace.strip("/") or "dobot_cra"
        self._topics = {
            name: [{"topic": f"/{prefix}/state/{name}",
                    "format": "sensor/skeleton" if name == "joints" else "data/json"}]
            for name in CARD_DESCRIPTIONS
        }

    def get_tools(self):
        definitions = [tool(name, "sensor", description, topic_out=self._topics[name])
                       for name, description in CARD_DESCRIPTIONS.items()]
        # The removed connection card's shared configuration now lives on joints.
        definitions[0]["configSchema"] = {
            "type": "object", "properties": {
                "controller_ip": {"type": "string", "description": "控制柜 IPv4 地址（LAN2 默认 192.168.200.1）",
                                  "scope": "shared", "x-sensitive": True},
                "feedback_port": {"type": "integer", "enum": [30004, 30005, 30006], "default": 30004,
                                  "description": "反馈端口；30006 自动使用至少 3 秒超时", "scope": "shared"},
            },
        }
        schema = action_schema({
            "info": ([], "查看连接、控制请求记录及反馈；不发送控制命令"),
            "request_control": (["confirm_change"], "显式请求 TCP 控制模式；仅下使能/下电时允许，不使能机器人"),
            "set_enabled": (["enabled", "confirm_change"], "提交安全皮肤总开关；关闭会移除皮肤接近保护，不能用于碰撞复位"),
            "set_sensitivity": (["part", "level", "confirm_change"], "提交指定部位灵敏度；0 关闭，1 低，2 中，3 高"),
        }, {
            "enabled": {"type": "boolean", "description": "安全皮肤总开关"},
            "part": {"type": "integer", "enum": [3, 4, 5, 6], "description": "3 小臂；4/5/6 为 J4/J5/J6"},
            "level": {"type": "integer", "enum": [0, 1, 2, 3], "description": "0 关闭；1 低；2 中；3 高"},
            "confirm_change": {"type": "boolean", "description": "确认本次安全设置变更；每次写请求必须为 true"},
        })
        schema["x-is-dangerous"] = True
        schema["x-resource"] = "arm"
        definitions.append(tool("safe_skin_control", "actuator", "安全皮肤 TCP 控制：总开关和分区灵敏度；队列提交不等于配置读回", schema))
        definitions.append(tool("model", "resource", "六轴 CR3A 直角连杆骨架 URDF，附加连接全部为固定关节"))
        return definitions

    def start(self):
        with self._lifecycle:
            if self._node is not None:
                return
            if self._ros2 is not None:
                from rclpy.node import Node
                from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
                from std_msgs.msg import String

                qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                                 history=HistoryPolicy.KEEP_LAST, depth=1,
                                 durability=DurabilityPolicy.VOLATILE)
                node = Node("dobot_cra_sensors", context=self._ros2.ctx_core)
                try:
                    self._publishers = {name: node.create_publisher(String, topics[0]["topic"], qos)
                                        for name, topics in self._topics.items()}
                    self._message_type = String
                    self._node = node
                    node.create_timer(1.0 / self._hz, self._publish)
                    self._ros2.executor_core.add_node(node)
                except Exception:
                    self._node = None
                    self._publishers = {}
                    node.destroy_node()
                    raise
            self.client.start()

    def stop(self):
        with self._lifecycle:
            node, self._node = self._node, None
            if node is not None:
                try:
                    self._ros2.executor_core.remove_node(node)
                finally:
                    node.destroy_node()
            self._publishers = {}
            try:
                self.control.close()
            finally:
                self.client.stop()

    def payload(self, name, snapshot=None):
        snapshot = self.client.snapshot() if snapshot is None else snapshot
        result = {key: value for key, value in snapshot.items() if key != "data"}
        result.update(timestamp_ms=int(time.time() * 1000), source="dobot_feedback", read_only=True)
        state = snapshot["data"]
        if name == "joints":
            skeleton = {**result, "format": "sensor/skeleton", "position_unit": "rad",
                        "joint_count": 6, "joints": [], "available": False}
            try:
                skeleton.update(resolve_model(snapshot, self.client.model_name))
            except ValueError as exc:
                return {**skeleton, "model_available": False, "model_error": str(exc)}
            skeleton["model_available"] = True
            if state is not None:
                joints = state["joint_state"]
                skeleton["available"] = True
                skeleton["joints"] = [
                    {"idx": i, "name": name, "q": math.radians(joints["q_actual"][i]),
                     "dq": math.radians(joints["qd_actual"][i]), "degree": joints["q_actual"][i]}
                    for i, name in enumerate(JOINT_NAMES)
                ]
            return skeleton
        if name == "safe_skin":
            # Feedback has proximity stops, not distance/configuration readback.
            regions = {
                name: {"part": part, "raw_flag": None if state is None else state[f"{name}_approach_state"],
                       "approach_paused": None if state is None else bool(state[f"{name}_approach_state"])}
                for part, name in PARTS.items()
            }
            keys = ("robot_mode", "robot_mode_name", "enable_status", "running_status", "error_status",
                    "collision_state", "safety_state", "safe_state", "current_command_id")
            return {
                **result, "available": state is not None, "regions": regions,
                "any_approach_paused": None if state is None else any(v["approach_paused"] for v in regions.values()),
                "robot_status": None if state is None else {key: state[key] for key in keys},
                "configuration_readback_available": False, "hardware_presence": "unknown",
                "enabled": None, "sensitivity": {name: None for name in PARTS.values()},
                "control": self.control.snapshot(), "reset_supported": False,
            }
        raise ValueError(f"unknown sensor: {name}")

    def _publish(self):
        with self._lifecycle:
            snapshot = self.client.snapshot()
            for name, publisher in self._publishers.items():
                message = self._message_type()
                message.data = json.dumps(self.payload(name, snapshot), ensure_ascii=False, allow_nan=False)
                publisher.publish(message)

    def dispatch(self, action, args):
        name = args.get("_tool_name", action)
        if name == "model":
            return model_resource(self.client.snapshot(), self.client.model_name)
        if name == "safe_skin_control":
            if action in ("info", name):
                return {"state": "ready", **self.control.snapshot(), "feedback": self.payload("safe_skin")}
            if action in ("start", "stop"):
                # Lifecycle does not claim control or cancel queued settings.
                if action == "stop":
                    self.control.close()
                return {"state": "ready" if action == "start" else "idle"}
            return self.control.execute(action, args)
        if name not in CARD_DESCRIPTIONS:
            return None
        if action == "start":
            if any(key in args for key in ("controller_ip", "feedback_port")):
                self.control.reconfigure(args.get("controller_ip", self.client.address),
                                         args.get("feedback_port", self.client.port))
            self.client.start()
            return {"state": "running", "topic_out": self._topics[name], **self.payload(name)}
        if action == "stop":
            return {"state": "idle"}
        if action in ("info", name):
            return {"topic_out": self._topics[name], **self.payload(name)}
        raise ValueError(f"unsupported sensor action: {action}")


def build_plugins(config, namespace, ros2):
    return [DobotSensors(config, namespace, ros2)]
