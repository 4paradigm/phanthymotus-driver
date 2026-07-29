#!/usr/bin/env python3
"""天轶2.0 遥控器状态原子卡片。"""

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

_KEY_EVENT_NAMES = {
    0: "none",
    1: "a_up",
    2: "a_down",
    3: "b_up",
    4: "b_down",
    5: "c_up",
    6: "c_down",
    7: "d_up",
    8: "d_down",
    9: "e_up",
    10: "e_mid",
    11: "e_down",
    12: "f_up",
    13: "f_mid",
    14: "f_down",
    15: "g_left",
    16: "g_mid",
    17: "g_right",
    18: "h_left",
    19: "h_mid",
    20: "h_right",
}


class RemoteControllerPlugin:
    """转发 SBUS 遥控器按键、拨杆和摇杆状态。"""

    def __init__(self, plugin_config: dict, namespace: str, ros2):
        del plugin_config
        self._topic = f"/{namespace}/state/remote_controller"
        self._running = False
        self._subscription = None
        self._latest = None
        self._lock = threading.Lock()

        self._sub_node = Node(
            "tianyi2_remote_controller_sub", context=ros2.ctx_tianyi
        )
        ros2.executor_tianyi.add_node(self._sub_node)

        self._pub_node = Node(
            "tianyi2_remote_controller_pub", context=ros2.ctx_core
        )
        ros2.executor_core.add_node(self._pub_node)
        self._pub = self._pub_node.create_publisher(
            String, self._topic, _RELIABLE_QOS
        )

    def get_tool(self) -> dict:
        return {
            "name": "remote_controller",
            "type": "sensor",
            "description": (
                "天轶2.0 SBUS遥控器状态 — "
                "按键事件、A-H拨杆状态及双摇杆位置"
            ),
            "inputSchema": {"type": "object", "properties": {}},
            "topic_out": [{"topic": self._topic, "format": "data/json"}],
        }

    def start(self) -> None:
        """开始转发事件；重复调用不会重复创建 ROS 订阅。"""
        with self._lock:
            self._running = True
            if self._subscription is not None:
                return

            try:
                from bodyctrl_msgs.msg import SbusData

                self._subscription = self._sub_node.create_subscription(
                    SbusData,
                    "/sbus_data/event",
                    self._on_remote_controller,
                    _RELIABLE_QOS,
                )
                print("[RemoteControllerPlugin] subscription created")
            except ImportError as exc:
                self._running = False
                warning = (
                    "[RemoteControllerPlugin] WARNING: "
                    f"msg import failed ({exc}), sensor is idle"
                )
                print(warning)

    def stop(self) -> None:
        """暂停事件转发，保留最近状态供查询。"""
        with self._lock:
            self._running = False

    def _on_remote_controller(self, msg) -> None:
        key_event_new = int(msg.key_event_new)
        key_event_old = int(msg.key_event_old)
        received_at = time.time()
        payload = {
            "header": {
                "stamp": {
                    "sec": int(msg.header.stamp.sec),
                    "nanosec": int(msg.header.stamp.nanosec),
                },
                "frame_id": msg.header.frame_id,
            },
            "received_at": received_at,
            "key_event_new": key_event_new,
            "key_event_new_name": _KEY_EVENT_NAMES.get(
                key_event_new, f"unknown_{key_event_new}"
            ),
            "key_event_old": key_event_old,
            "key_event_old_name": _KEY_EVENT_NAMES.get(
                key_event_old, f"unknown_{key_event_old}"
            ),
            "button_a": int(msg.button_a),
            "button_b": int(msg.button_b),
            "button_c": int(msg.button_c),
            "button_d": int(msg.button_d),
            "button_e": int(msg.button_e),
            "button_f": int(msg.button_f),
            "button_g": int(msg.button_g),
            "button_h": int(msg.button_h),
            "x1": float(msg.x1),
            "y1": float(msg.y1),
            "x2": float(msg.x2),
            "y2": float(msg.y2),
        }

        with self._lock:
            if not self._running:
                return
            self._latest = payload
            out = String()
            out.data = json.dumps(payload, ensure_ascii=False)
            self._pub.publish(out)

    def dispatch(self, action_or_tool: str, args: dict) -> dict:
        del args
        if action_or_tool == "remote_controller":
            with self._lock:
                if self._latest is None:
                    return {"state": "no_data"}
                return copy.deepcopy(self._latest)

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
