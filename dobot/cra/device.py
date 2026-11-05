"""Joint skeleton and angles sharing one receive-only feedback stream."""

from __future__ import annotations

import json
import math
import threading
import time

from common.vendor_runtime import action_schema, tool

if __package__:
    from .feedback import FeedbackClient, positive_number
    from .model import JOINT_NAMES, model_resource, resolve_model
    from .control import JOINT_LIMITS_DEG, RobotController
else:
    from feedback import FeedbackClient, positive_number
    from model import JOINT_NAMES, model_resource, resolve_model
    from control import JOINT_LIMITS_DEG, RobotController


CARD_DESCRIPTIONS = {
    "joints": "CR3A 六轴实时骨架及关节转角 JSON（度），固定直角连接保持真实关节坐标",
}


class DobotSensors:
    def __init__(self, config, namespace="dobot_cra", ros2=None, client=None, controller=None):
        self.client = client if client is not None else FeedbackClient(config)
        self.control = controller if controller is not None else RobotController(self.client, config)
        self._hz = positive_number(config.get("publish_hz", 30), "publish_hz")
        if self._hz > 125:
            raise ValueError("publish_hz must not exceed the 125 Hz feedback rate")
        self._ros2 = ros2
        self._node = None
        self._publishers = {}
        self._message_type = None
        self._lifecycle = threading.RLock()
        prefix = namespace.strip("/") or "dobot_cra"
        self._topics = {
            "joints": [
                {"topic": f"/{prefix}/state/joints", "format": "sensor/skeleton"},
                {"topic": f"/{prefix}/state/joint_angles", "format": "data/json"},
            ],
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
        angle_fields = [f"{joint}_deg" for joint in JOINT_NAMES]
        motion_schema = action_schema({
            "info": ([], "查看关节运动卡功能、参数和单位说明，不读取运行状态"),
            "move_j": ([*angle_fields, "velocity", "acceleration", "confirm_move"],
                       "状态检查通过后先 EnableSafeSkin(0)，再 ClearError，成功后检查使能，未使能则 EnableRobot 并等待确认，再执行绝对关节运动（度）；六个角度必填，速度/加速度留空为 10%"),
            "stop_motion": (["confirm_move"], "停止控制器运动队列/工程，包括点动；不下使能"),
            "pause": (["confirm_move"], "暂停当前运动队列/工程；不支持点动暂停"),
            "resume": (["confirm_move"], "先 EnableSafeSkin(0)，成功后检查使能，未使能则 EnableRobot 并等待确认，再继续暂停队列；不自动清错"),
            "start": ([], "启动反馈读取，不发送运动指令"),
            "stop": ([], "关闭共享命令连接，不发送停止运动指令"),
            "config": ([], "确认卡片配置；本卡与 joints 共用 IP/端口配置"),
        }, {
            **{field: {"type": "number", "minimum": lower, "maximum": upper,
                       "description": f"J{index} 绝对目标角度（度），范围 [{lower}, {upper}]，必填"}
               for index, (field, (lower, upper)) in enumerate(zip(angle_fields, JOINT_LIMITS_DEG), 1)},
            "velocity": {"type": "integer", "minimum": 1, "maximum": 100, "default": 10,
                         "description": "本条运动速度比例 1–100%，留空为 10%；实际速度还受全局速率影响"},
            "acceleration": {"type": "integer", "minimum": 1, "maximum": 100, "default": 10,
                             "description": "本条运动加速度比例 1–100%，留空为 10%"},
            "confirm_move": {"type": "boolean", "enum": [False, True], "default": False,
                             "oneOf": [{"const": False, "title": "未确认（不执行）"},
                                       {"const": True, "title": "确认执行本次运动操作"}],
                             "description": "本卡每次运动控制操作都需明确确认"},
        })
        motion_schema["x-is-dangerous"] = True
        motion_schema["x-resource"] = "arm"
        motion_tool = tool("joint_move", "actuator", "CR3A 绝对关节角度运动、停止、暂停与继续；角度单位为度", motion_schema)
        motion_tool["configSchema"] = definitions[0]["configSchema"]
        definitions.append(motion_tool)
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
                    self._publishers = {
                        (name, index): node.create_publisher(String, output["topic"], qos)
                        for name, topics in self._topics.items()
                        for index, output in enumerate(topics)
                    }
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
        state = snapshot["data"]
        result = {"timestamp_ms": int(time.time() * 1000), "available": state is not None}
        if name == "joint_angles":
            return {
                **result, **{
                    f"{joint}_deg": None if state is None else state["joint_state"]["q_actual"][index]
                    for index, joint in enumerate(JOINT_NAMES)
                },
            }
        if name == "joints":
            skeleton = {**result, "joints": [], "available": False}
            try:
                resolve_model(snapshot, self.client.model_name)
            except ValueError:
                return skeleton
            if state is not None:
                joints = state["joint_state"]
                skeleton["available"] = True
                skeleton["joints"] = [
                    {"name": name, "q": math.radians(joints["q_actual"][i])}
                    for i, name in enumerate(JOINT_NAMES)
                ]
            return skeleton
        raise ValueError(f"unknown sensor: {name}")

    def _publish(self):
        with self._lifecycle:
            snapshot = self.client.snapshot()
            for (name, index), publisher in self._publishers.items():
                message = self._message_type()
                output_name = "joint_angles" if name == "joints" and index == 1 else name
                message.data = json.dumps(self.payload(output_name, snapshot), ensure_ascii=False, allow_nan=False,
                                          indent=2 if output_name == "joint_angles" else None)
                publisher.publish(message)

    def dispatch(self, action, args):
        name = args.get("_tool_name", action)
        if name == "model":
            return model_resource(self.client.snapshot(), self.client.model_name)
        if action in ("info", name):
            # Core calls info for descriptions and topic discovery. Keep live
            # state exclusively in ROS outputs.
            return next((definition for definition in self.get_tools() if definition["name"] == name), None)
        if name == "joint_move":
            if action in ("config", "start") and any(key in args for key in ("controller_ip", "feedback_port")):
                self.control.reconfigure(args.get("controller_ip", self.client.address),
                                         args.get("feedback_port", self.client.port))
            if action in ("start", "stop", "config"):
                if action == "stop":
                    self.control.close()
                elif action == "start":
                    self.client.start()
                return {"state": {"start": "ready", "stop": "idle", "config": "configured"}[action]}
            return self.control.execute_motion(action, args)
        if name not in CARD_DESCRIPTIONS:
            return None
        if action in ("config", "start"):
            if any(key in args for key in ("controller_ip", "feedback_port")):
                self.control.reconfigure(args.get("controller_ip", self.client.address),
                                         args.get("feedback_port", self.client.port))
            if action == "start":
                self.client.start()
                return {"state": "running", "topic_out": self._topics[name], **self.payload(name)}
            return {"state": "configured", "topic_out": self._topics[name],
                    "controller_ip": self.client.address, "feedback_port": self.client.port}
        if action == "stop":
            return {"state": "idle"}
        raise ValueError(f"unsupported sensor action: {action}")


def build_plugins(config, namespace, ros2):
    return [DobotSensors(config, namespace, ros2)]
