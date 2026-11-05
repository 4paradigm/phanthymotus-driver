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
    from .control import PARTS, RobotController
else:
    from feedback import FeedbackClient, positive_number
    from model import JOINT_NAMES, model_resource, resolve_model
    from control import PARTS, RobotController


CARD_DESCRIPTIONS = {
    "joints": "CR3A 六轴实时骨架及关节转角 JSON（度/弧度），固定直角连接保持真实关节坐标",
}


class DobotSensors:
    def __init__(self, config, namespace="dobot_cra", ros2=None, client=None, controller=None):
        self.client = client if client is not None else FeedbackClient(config)
        self.control = controller if controller is not None else RobotController(self.client, config)
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
            "joints": [
                {"topic": f"/{prefix}/state/joints", "format": "sensor/skeleton"},
                {"topic": f"/{prefix}/state/joint_angles", "format": "data/json"},
            ],
            "safe_skin_control": [{"topic": f"/{prefix}/state/safe_skin", "format": "data/json"}],
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
            "start": ([], "启动实时触发状态输出，不发送设置命令"),
            "stop": ([], "关闭共享命令连接，不修改安全皮肤设置"),
            "config": ([], "确认卡片配置；共享 IP/端口通过 joints 配置"),
        }, {
            "enabled": {"type": "boolean", "description": "安全皮肤总开关"},
            "part": {"type": "integer", "enum": [3, 4, 5, 6], "description": "3 小臂；4/5/6 为 J4/J5/J6"},
            "level": {"type": "integer", "enum": [0, 1, 2, 3], "description": "0 关闭；1 低；2 中；3 高"},
            "confirm_change": {"type": "boolean", "description": "确认本次安全设置变更；每次写请求必须为 true"},
        })
        schema["x-is-dangerous"] = True
        schema["x-resource"] = "arm"
        definitions.append(tool("safe_skin_control", "actuator", "安全皮肤：实时触发 JSON 输出、总开关和分区灵敏度；队列提交不等于配置读回",
                                schema, topic_out=self._topics["safe_skin_control"]))
        system_schema = action_schema({
            "info": ([], "读取系统实时状态及本卡最近请求，不发送命令"),
            "get_errors": ([], "通过 SDK 查询控制器当前报警编号，不清除报警"),
            "request_control": (["confirm_change"], "请求 TCP 控制权；仅未上电/下使能时允许，不自动使能"),
            "power_on": (["confirm_change"], "给机械臂本体上电；等待反馈初始化完成后才能另行使能"),
            "enable": (["confirm_change"], "上使能，使用控制器已有负载设置，不清错或继续队列"),
            "disable": (["confirm_change"], "下使能；反馈过期时仍可尝试发送"),
            "clear_error": (["confirm_change"], "机器人静止时清除报警；不自动使能、复位皮肤或恢复运动"),
            "stop_motion": (["confirm_change"], "停止运动指令队列/工程；与卡片 stop 不同"),
            "pause": (["confirm_change"], "暂停正在执行的队列/工程；点动请使用 stop_motion"),
            "resume": (["confirm_change"], "显式继续已暂停队列；可能恢复机械臂运动"),
            "set_speed_factor": (["ratio", "confirm_change"], "设置全局速度比例，1–100%"),
            "start_drag": (["confirm_change"], "机器人静止且无报警/碰撞时进入关节拖拽示教"),
            "stop_drag": (["confirm_change"], "退出拖拽示教；反馈过期时仍可尝试发送"),
            "start": ([], "启动反馈读取，不发送控制命令"),
            "stop": ([], "关闭共享 Dashboard 连接；不会发送停止运动指令"),
            "config": ([], "确认卡片配置；共享 IP/端口通过 joints 配置"),
        }, {
            "ratio": {"type": "integer", "minimum": 1, "maximum": 100, "description": "全局速度百分比"},
            "confirm_change": {"type": "boolean", "description": "确认本次系统操作；写请求必须为 true"},
        })
        system_schema["x-is-dangerous"] = True
        system_schema["x-resource"] = "arm"
        definitions.append(tool("systemcontrol", "actuator", "系统状态、报警查询、TCP 控制权、本体上电、使能及运行控制", system_schema))
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
        result = {key: value for key, value in snapshot.items() if key != "data"}
        result.update(timestamp_ms=int(time.time() * 1000), source="dobot_feedback", read_only=True)
        state = snapshot["data"]
        if name == "joint_angles":
            angles = [] if state is None else list(state["joint_state"]["q_actual"])
            return {
                **result, "format": "data/json", "available": state is not None,
                "joint_count": 6, "joint_names": list(JOINT_NAMES),
                "angles_deg": angles, "angles_rad": [math.radians(angle) for angle in angles],
            }
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
        if name == "safe_skin_control":
            # Feedback has proximity stops, not distance/configuration readback.
            regions = {
                name: None if state is None else bool(state[f"{name}_approach_state"])
                for name in PARTS.values()
            }
            return {
                "timestamp_ms": result["timestamp_ms"], "available": state is not None,
                "any_approach_paused": None if state is None else any(regions.values()),
                "regions": regions,
            }
        raise ValueError(f"unknown sensor: {name}")

    def _publish(self):
        with self._lifecycle:
            snapshot = self.client.snapshot()
            for (name, index), publisher in self._publishers.items():
                message = self._message_type()
                output_name = "joint_angles" if name == "joints" and index == 1 else name
                message.data = json.dumps(self.payload(output_name, snapshot), ensure_ascii=False, allow_nan=False)
                publisher.publish(message)

    def dispatch(self, action, args):
        name = args.get("_tool_name", action)
        if name == "model":
            return model_resource(self.client.snapshot(), self.client.model_name)
        if name == "systemcontrol":
            if action in ("info", name):
                return {"state": "ready", **self.control.system_info()}
            if action in ("start", "stop", "config"):
                if action == "stop":
                    self.control.close()
                elif action == "start":
                    self.client.start()
                return {"state": {"start": "ready", "stop": "idle", "config": "configured"}[action]}
            return self.control.execute_system(action, args)
        if name == "safe_skin_control":
            if action in ("info", name):
                return {"state": "ready", "topic_out": self._topics[name],
                        **self.control.snapshot(), "feedback": self.payload(name)}
            if action in ("start", "stop", "config"):
                # Lifecycle does not claim control or cancel queued settings.
                if action == "stop":
                    self.control.close()
                elif action == "start":
                    self.client.start()
                return {"state": {"start": "ready", "stop": "idle", "config": "configured"}[action],
                        "topic_out": self._topics[name]}
            return self.control.execute(action, args)
        if name not in CARD_DESCRIPTIONS:
            return None
        if action in ("config", "start"):
            if any(key in args for key in ("controller_ip", "feedback_port")):
                self.control.reconfigure(args.get("controller_ip", self.client.address),
                                         args.get("feedback_port", self.client.port))
            if action == "start":
                self.client.start()
            return {"state": "configured" if action == "config" else "running",
                    "topic_out": self._topics[name], **self.payload(name)}
        if action == "stop":
            return {"state": "idle"}
        if action in ("info", name):
            return {"topic_out": self._topics[name], **self.payload(name)}
        raise ValueError(f"unsupported sensor action: {action}")


def build_plugins(config, namespace, ros2):
    return [DobotSensors(config, namespace, ros2)]
