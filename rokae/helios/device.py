"""Motus joints sensor and its matching original ROKAE URDF resource."""

from __future__ import annotations

import json
import os
import threading
import time

from common.vendor_runtime import tool
from .feedback import FeedbackClient, ScopeFeedbackClient, positive_number, validate_url
from .model import fetch_model


class HeliosJoints:
    PREFIX = "joints"

    def __init__(self, config, namespace="rokae_helios", ros2=None, client=None, scope_client=None):
        self._config = config
        self._ros2 = ros2
        self._node = None
        self._publishers = []
        self._lock = threading.RLock()
        self._hz = positive_number(config.get("publish_hz", 10), "publish_hz")
        if self._hz > 30:
            raise ValueError("publish_hz must not exceed 30; iNex feedback is approximately 10 Hz")
        self.client = client or self._new_client(os.getenv("HELIOS_STATE_URL", config["state_url"]))
        self.scope_client = scope_client or self._new_client(self.client.url, scope=True)
        # One output only: the skeleton message already carries per-joint q, dq
        # and torques, so a separate status topic would duplicate it.
        self._topics = [{"topic": f"/{namespace}/state/joints", "format": "sensor/skeleton"}]
        self._model_lock = threading.Lock()
        self._model_url = None
        self._urdf = None

    def _new_client(self, url, *, scope=False):
        cls = ScopeFeedbackClient if scope else FeedbackClient
        return cls(url, stale_timeout_s=self._config.get("stale_timeout_s", 1),
                              reconnect_delay_s=self._config.get("reconnect_delay_s", 1),
                              connect_timeout_s=self._config.get("connect_timeout_s", 3))

    def get_tools(self):
        sensor = tool("joints", "sensor", "Helios-2.1 实时关节骨架：双臂、躯干、头部共 20 轴；每轴含角度(rad)、速度、电机反馈力矩与模型重力矩(Nm)；只读，约 10 Hz。",
                      topic_out=self._topics)
        sensor["configSchema"] = {"type": "object", "properties": {
            "state_url": {"type": "string", "scope": "shared", "x-sensitive": True,
                          "description": "原厂 iNex 状态 WebSocket 地址（留空使用部署配置）"},
        }}
        return [sensor, tool("model", "resource", "珞石 Helios-2.1 原厂 20 轴 URDF 骨架模型")]

    def start(self):
        with self._lock:
            if self._node is not None:
                return
            if self._ros2 is not None:
                from rclpy.node import Node
                from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
                from std_msgs.msg import String
                node = Node("helios_joints", context=self._ros2.ctx_core)
                try:
                    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                                     history=HistoryPolicy.KEEP_LAST, durability=DurabilityPolicy.VOLATILE)
                    self._publishers = [node.create_publisher(String, t["topic"], qos) for t in self._topics]
                    self._message_type = String
                    node.create_timer(1 / self._hz, self._publish)
                    self._ros2.executor_core.add_node(node)
                    self._node = node
                except Exception:
                    self._publishers = []
                    node.destroy_node()
                    raise
            self.client.start()
            self.scope_client.start()

    def stop(self):
        with self._lock:
            self.client.stop()
            self.scope_client.stop()
            if self._node is not None:
                self._ros2.executor_core.remove_node(self._node)
                self._node.destroy_node()
                self._node = None
            self._publishers = []

    def payload(self):
        snap = self.client.snapshot()
        scope = self.scope_client.snapshot()
        torques = {j["name"]: j for j in scope["torques"]} if scope["available"] and snap["available"] else {}
        joints = [{**j, **{k: v for k, v in torques.get(j["name"], {}).items() if k != "name"}}
                  for j in snap["joints"]]
        torque_status = {k: v for k, v in scope.items() if k != "torques"}
        torque_status.update({"unit": "Nm", "source_topic": "joint/scope",
                              "feedback_source": "motor_torque", "gravity_source": "robot_model",
                              "physical_validity_verified": False})
        message = {"online": "实时关节数据正常", "idle": "状态订阅已停止",
                   "reconnecting": "连接中或已断线，正在重连", "waiting": "已连接，等待有效关节数据",
                   "stale": "关节数据过期", "invalid": "收到无效关节数据"}[snap["state"]]
        return {**snap, "joints": joints, "torque_status": torque_status, "message": message,
                "timestamp_ms": int(time.time() * 1000), "angle_unit": "rad"}

    def _publish(self):
        with self._lock:
            for publisher in self._publishers:
                msg = self._message_type()
                msg.data = json.dumps(self.payload(), ensure_ascii=False, allow_nan=False)
                publisher.publish(msg)

    def dispatch(self, action, args):
        name = args.get("_tool_name", action)
        if name == "model":
            # A missing model must not prevent battery/joint subscriptions from starting.
            # Separate lock keeps network reads out of the sensor lifecycle lock.
            with self._model_lock:
                url = self.client.url
                if self._model_url != url:
                    model = fetch_model(url, self._config.get("connect_timeout_s", 3))
                    if self.client.url != url:
                        raise RuntimeError("state endpoint changed while reading model; retry")
                    self._urdf, self._model_url = model, url
                return {"urdf": self._urdf, "display_name": "珞石 Helios-2.1"}
        if name != "joints":
            return None
        with self._lock:
            if action in ("info", "joints"):
                return self.get_tools()[0]
            if action in ("start", "config"):
                url = args.get("state_url")
                if url and validate_url(url) != self.client.url:
                    replacement = self._new_client(url)
                    scope_replacement = self._new_client(url, scope=True)
                    self.client.stop()
                    self.scope_client.stop()
                    self.client = replacement
                    self.scope_client = scope_replacement
                    self.client.start()
                    self.scope_client.start()
                if action == "start":
                    self.client.start()
                    self.scope_client.start()
                return {"state": "running" if action == "start" else "configured", "topic_out": self._topics}
            if action == "stop":
                self.client.stop()
                self.scope_client.stop()
                self._publish()  # Explicit idle + empty joints, never an invented zero pose.
                return {"state": "idle", "topic_out": self._topics}
            raise ValueError(f"unsupported joints action: {action}")
