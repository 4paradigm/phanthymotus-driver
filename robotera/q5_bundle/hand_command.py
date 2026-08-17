"""Shared, single-publisher command path for the Q5 XHand Lite cards."""

from __future__ import annotations

import math
import threading

try:
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    from xbot_common_interfaces.msg import HybridJointCommand

    _HAS_ROS2 = True
    _QOS = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                      history=HistoryPolicy.KEEP_LAST, depth=1,
                      durability=DurabilityPolicy.VOLATILE)
except Exception:
    _HAS_ROS2 = False

TOPIC = "/hand_controller/commands"
NODE = "q5_hand_command"
HAND_VELOCITY_RADPS = 0.0
HAND_FEEDFORWARD = 350.0
HAND_KP = 100.0
HAND_KD = 0.0
HAND_JOINTS = (
    "left_hand_thumb_bend_joint", "left_hand_thumb_rota_joint1",
    "left_hand_index_joint1", "left_hand_mid_joint1",
    "left_hand_ring_joint1", "left_hand_pinky_joint1",
    "right_hand_thumb_bend_joint", "right_hand_thumb_rota_joint1",
    "right_hand_index_joint1", "right_hand_mid_joint1",
    "right_hand_ring_joint1", "right_hand_pinky_joint1",
)


def failure(code: str, message: str, **details) -> dict:
    return {"ok": False, "code": code, "message": message, "details": details}


def finite_number(value, field: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{field} must be a number")
    value = float(value)
    if not math.isfinite(value):
        raise ValueError(f"{field} must be finite")
    return value


class HandCommandRouter:
    """One publisher plus an in-process lease shared by hand control cards."""

    def __init__(self, executor):
        self._node = None
        self._pub = None
        self._lock = threading.Lock()
        self._owner = None
        if _HAS_ROS2 and executor is not None:
            try:
                self._node = Node(NODE)
                self._pub = self._node.create_publisher(HybridJointCommand, TOPIC, _QOS)
                executor.add_node(self._node)
            except Exception as e:
                print(f"[hand_command] ROS2 publisher unavailable: {e}", flush=True)
                self._node = None
                self._pub = None

    def status(self):
        competitors = []
        endpoint_query_available = self._node is not None
        if self._node is not None:
            try:
                competitors = [
                    {"node_name": endpoint.node_name, "node_namespace": endpoint.node_namespace}
                    for endpoint in self._node.get_publishers_info_by_topic(TOPIC)
                    if endpoint.node_name != NODE
                ]
            except Exception:
                endpoint_query_available = False
        with self._lock:
            owner = self._owner
        return {"ros_publisher_available": self._pub is not None,
                "endpoint_query_available": endpoint_query_available,
                "other_publishers": competitors,
                "active_owner": owner, "topic": TOPIC,
                "joint_model": "XHand Lite 12 joints"}

    def acquire(self, owner: str) -> bool:
        with self._lock:
            if self._owner not in (None, owner):
                return False
            self._owner = owner
            return True

    def release(self, owner: str):
        with self._lock:
            if self._owner == owner:
                self._owner = None

    def publish(self, positions: dict) -> bool:
        if self._node is None or self._pub is None or not positions:
            return False
        msg = HybridJointCommand()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.joint_name = list(positions)
        msg.position = [float(positions[name]) for name in msg.joint_name]
        count = len(msg.joint_name)
        # Match the vendor XHand Lite command contract: all command vectors
        # must be present and aligned with joint_name.
        msg.velocity = [HAND_VELOCITY_RADPS] * count
        msg.feedforward = [HAND_FEEDFORWARD] * count
        msg.kp = [HAND_KP] * count
        msg.kd = [HAND_KD] * count
        self._pub.publish(msg)
        return True


def get_router(client, executor) -> HandCommandRouter:
    router = getattr(client, "_q5_hand_command_router", None)
    if router is None:
        router = HandCommandRouter(executor)
        setattr(client, "_q5_hand_command_router", router)
    return router
