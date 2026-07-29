#!/usr/bin/env python3
"""天轶2.0 Inspire 灵巧手状态原子卡片。"""

import copy
import json
import threading
import time

from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import String


_RELIABLE_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    durability=DurabilityPolicy.VOLATILE,
)


def _empty_hand_state() -> dict:
    return {
        "name": [],
        "position": [],
        "velocity": [],
        "effort": [],
        "timestamp": None,
    }


class HandStatePlugin:
    """转发左右 Inspire 灵巧手的关节位置、速度和力反馈。"""

    def __init__(self, plugin_config: dict, namespace: str, ros2):
        del plugin_config
        self._topic = f"/{namespace}/state/hand"
        self._running = False
        self._subscriptions = []
        self._lock = threading.Lock()
        self._state = {
            "updated_at": None,
            "left": _empty_hand_state(),
            "right": _empty_hand_state(),
        }

        self._sub_node = Node(
            "tianyi2_hand_state_sub", context=ros2.ctx_tianyi
        )
        ros2.executor_tianyi.add_node(self._sub_node)

        self._pub_node = Node("tianyi2_hand_state_pub", context=ros2.ctx_core)
        ros2.executor_core.add_node(self._pub_node)
        self._pub = self._pub_node.create_publisher(
            String, self._topic, _RELIABLE_QOS
        )

    def get_tool(self) -> dict:
        return {
            "name": "hand_state",
            "type": "sensor",
            "description": (
                "天轶2.0 Inspire灵巧手状态 — "
                "左右手关节名称、位置、速度和力反馈"
            ),
            "inputSchema": {"type": "object", "properties": {}},
            "topic_out": [{"topic": self._topic, "format": "data/json"}],
        }

    def start(self) -> None:
        """开始转发状态；重复调用不会创建重复订阅。"""
        with self._lock:
            self._running = True
            if self._subscriptions:
                return

            try:
                from sensor_msgs.msg import JointState

                self._subscriptions = [
                    self._sub_node.create_subscription(
                        JointState,
                        "/inspire_hand/state/left_hand",
                        lambda msg: self._on_hand_state("left", msg),
                        _RELIABLE_QOS,
                    ),
                    self._sub_node.create_subscription(
                        JointState,
                        "/inspire_hand/state/right_hand",
                        lambda msg: self._on_hand_state("right", msg),
                        _RELIABLE_QOS,
                    ),
                ]
                print("[HandStatePlugin] subscriptions created")
            except ImportError as exc:
                self._running = False
                print(
                    f"[HandStatePlugin] WARNING: msg import failed ({exc}), "
                    "sensor is idle"
                )

    def stop(self) -> None:
        """暂停状态转发，保留最近一次双手状态供查询。"""
        with self._lock:
            self._running = False

    def _on_hand_state(self, side: str, msg) -> None:
        received_at = time.time()
        state = {
            "name": list(msg.name),
            "position": list(msg.position),
            "velocity": list(msg.velocity),
            "effort": list(msg.effort),
            "timestamp": {
                "sec": int(msg.header.stamp.sec),
                "nanosec": int(msg.header.stamp.nanosec),
                "received_at": received_at,
            },
        }

        with self._lock:
            if not self._running:
                return
            self._state[side] = state
            self._state["updated_at"] = received_at
            payload = copy.deepcopy(self._state)
            out = String()
            out.data = json.dumps(payload, ensure_ascii=False)
            self._pub.publish(out)

    def dispatch(self, action_or_tool: str, args: dict) -> dict:
        del args
        if action_or_tool == "hand_state":
            with self._lock:
                return copy.deepcopy(self._state)

        if action_or_tool == "start":
            self.start()
        elif action_or_tool == "stop":
            self.stop()
        elif action_or_tool != "info":
            return {"error": f"unknown action: {action_or_tool}"}

        return {
            "state": "running" if self._is_running() else "idle",
            "topic_out": [{"topic": self._topic, "format": "data/json"}],
        }

    def _is_running(self) -> bool:
        with self._lock:
            return self._running
