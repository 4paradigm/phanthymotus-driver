"""RM75-style sensor cards backed by one receive-only Dobot SDK client."""

from __future__ import annotations

import json
import math
import threading
import time

from common.vendor_runtime import tool

if __package__:
    from .feedback import FeedbackClient, SDK_COMMIT, SDK_VERSION, positive_number
else:
    from feedback import FeedbackClient, SDK_COMMIT, SDK_VERSION, positive_number


CARD_DESCRIPTIONS = {
    "connection": "控制器反馈连接、数据新鲜度和重连状态（只读）",
    "joint_states": "六关节实际/目标角度、速度、电流和温度（只读）",
    "robot_info": "反馈报告的机器人类型码、六轴信息和人工配置的型号（只读）",
    "software_info": "驱动所用官方 SDK 版本及来源；反馈协议不提供控制器固件版本",
    "arm_all_state": "机械臂完整反馈：关节、TCP 位姿、负载、力和 IO（只读）",
    "controller_state": "使能、抱闸、运行、报警标志、电压和 IO 状态（只读）",
}


class DobotSensors:
    def __init__(self, config, namespace="dobot_cra", ros2=None, client=None):
        self.client = client if client is not None else FeedbackClient(config)
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
            name: [{"topic": f"/{prefix}/state/{name}", "format": "data/json"}]
            for name in CARD_DESCRIPTIONS
        }

    def get_tools(self):
        definitions = [
            tool(name, "sensor", description, topic_out=self._topics[name])
            for name, description in CARD_DESCRIPTIONS.items()
        ]
        definitions[0]["configSchema"] = {
            "type": "object",
            "properties": {
                "controller_ip": {
                    "type": "string", "description": "控制柜 IPv4 地址（LAN2 默认 192.168.200.1）",
                    "scope": "shared", "x-sensitive": True,
                },
                "feedback_port": {
                    "type": "integer", "enum": [30004, 30005, 30006], "default": 30004,
                    "description": "反馈端口：30004 为 8ms，30005 为 200ms，30006 为 1000ms",
                    "scope": "shared",
                },
            },
        }
        return definitions

    def start(self):
        with self._lifecycle:
            if self._node is not None:
                return
            if self._ros2 is not None:
                from rclpy.node import Node
                from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
                from std_msgs.msg import String

                qos = QoSProfile(
                    reliability=ReliabilityPolicy.BEST_EFFORT,
                    history=HistoryPolicy.KEEP_LAST, depth=1,
                    durability=DurabilityPolicy.VOLATILE,
                )
                node = Node("dobot_cra_sensors", context=self._ros2.ctx_core)
                try:
                    self._publishers = {
                        name: node.create_publisher(String, topics[0]["topic"], qos)
                        for name, topics in self._topics.items()
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
            self.client.stop()

    def payload(self, name, snapshot=None):
        snapshot = self.client.snapshot() if snapshot is None else snapshot
        health = {key: value for key, value in snapshot.items() if key != "data"}
        result = {
            **health, "timestamp_ms": int(time.time() * 1000),
            "source": "dobot_feedback", "read_only": True,
        }
        if name == "connection":
            return result
        if name == "software_info":
            return {
                **result, "source": "driver_metadata",
                "sdk_version": SDK_VERSION, "sdk_commit": SDK_COMMIT,
                "sdk_repository": "https://github.com/Dobot-Arm/TCP-IP-Python-V4",
                "feedback_protocol_bytes": 1440,
                "controller_firmware_version": None,
                "controller_firmware_version_available": False,
                "note": "Feedback does not report firmware version; check DobotStudio Pro.",
            }
        state = snapshot["data"]
        if state is None:
            return {**result, "available": False, "data": None}
        result["available"] = True
        if name == "robot_info":
            return {
                **result, "robot_type_code": state["robot_type"], "arm_dof": 6,
                "configured_model_name": self.client.model_name or None,
                "model_name_source": "operator_config" if self.client.model_name else None,
            }
        if name == "joint_states":
            joints = state["joint_state"]
            return {
                **result,
                "joint_names": [f"joint{i}" for i in range(1, 7)],
                "position": [math.radians(v) for v in joints["q_actual"]],
                "velocity": [math.radians(v) for v in joints["qd_actual"]],
                "position_unit": "rad", "velocity_unit": "rad/s",
                "raw_degree": joints["q_actual"],
                "target_degree": joints["q_target"],
                "raw_velocity_deg_s": joints["qd_actual"],
                "current_a": joints["i_actual"],
                "motor_temperature_c": joints["temperatures"],
                "brake_released": state["brake_released"],
            }
        if name == "arm_all_state":
            return {
                **result, "data": state,
                "units": {
                    "joint_position": "deg", "joint_velocity": "deg/s",
                    "tcp_position": "mm", "tcp_orientation": "deg",
                    "voltage": "V", "current": "A", "load": "kg",
                },
            }
        if name == "controller_state":
            keys = (
                "robot_mode", "robot_mode_name", "enable_status", "brake_status",
                "brake_released", "error_status", "running_status", "drag_status",
                "jog_status", "pause_cmd_flag", "run_queued_cmd", "collision_state",
                "auto_manual_mode", "safety_state", "safe_state", "safety_io_in",
                "safety_io_out", "digital_inputs", "digital_outputs", "voltage",
                "current", "speed_scaling", "current_command_id", "timestamp", "run_time",
            )
            return {
                **result, "data": {key: state[key] for key in keys},
                "alarm_codes_available": False,
                "note": "error_status is an alarm flag, not a GetErrorID alarm-code list.",
            }
        raise ValueError(f"unknown sensor: {name}")

    def _publish(self):
        # A single coherent snapshot for all cards; no network calls in ROS timers.
        with self._lifecycle:
            snapshot = self.client.snapshot()
            for name, publisher in self._publishers.items():
                message = self._message_type()
                message.data = json.dumps(self.payload(name, snapshot), ensure_ascii=False, allow_nan=False)
                publisher.publish(message)

    def dispatch(self, action, args):
        name = args.get("_tool_name", action)
        if name not in CARD_DESCRIPTIONS:
            return None
        if action == "start":
            # Shared configuration belongs to connection. Starting another card
            # cannot take ownership of or close the shared feedback socket.
            if name == "connection" and any(key in args for key in ("controller_ip", "feedback_port")):
                self.client.configure(
                    args.get("controller_ip", self.client.address),
                    args.get("feedback_port", self.client.port),
                )
            self.client.start()
            return {"state": "running", "topic_out": self._topics[name], **self.payload(name)}
        if action == "stop":
            # Always-on singleton sensors follow the platform lifecycle contract.
            return {"state": "idle"}
        if action in ("info", name):
            return {"topic_out": self._topics[name], **self.payload(name)}
        raise ValueError(f"unsupported sensor action: {action}")


def build_plugins(config, namespace, ros2):
    return [DobotSensors(config, namespace, ros2)]
