"""Shared, single-publisher command path for Q5 direct body cards."""

from __future__ import annotations

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

TOPIC = "/wr1_controller/commands"
NODE = "q5_body_command"
BODY_VELOCITY_RADPS = 0.0
BODY_FEEDFORWARD_NM = 0.0
BODY_KP = 85.0
BODY_KD = 20.0


class BodyCommandRouter:
    """One Q5 body publisher plus an in-process lease for arm/head cards."""

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
                print(f"[body_command] ROS2 publisher unavailable: {e}", flush=True)
                self._node = None
                self._pub = None

    def status(self):
        competitors = []
        same_name_count = 0
        endpoint_query_available = self._node is not None
        if self._node is not None:
            try:
                endpoints = self._node.get_publishers_info_by_topic(TOPIC)
                same_name_count = sum(1 for endpoint in endpoints if endpoint.node_name == NODE)
                competitors = [{"node_name": endpoint.node_name,
                                "node_namespace": endpoint.node_namespace}
                               for endpoint in endpoints if endpoint.node_name != NODE]
            except Exception:
                endpoint_query_available = False
        with self._lock:
            owner = self._owner
        return {"ros_publisher_available": self._pub is not None,
                "endpoint_query_available": endpoint_query_available,
                "other_publishers": competitors, "active_owner": owner,
                "same_name_publisher_count": same_name_count,
                "topic": TOPIC, "publisher_node": NODE}

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
        # The Q5 SDK example fills every HybridJointCommand vector. Leaving
        # these fields empty can make the vendor controller ignore a command.
        msg.velocity = [BODY_VELOCITY_RADPS] * count
        msg.feedforward = [BODY_FEEDFORWARD_NM] * count
        msg.kp = [BODY_KP] * count
        msg.kd = [BODY_KD] * count
        self._pub.publish(msg)
        return True


def get_router(client, executor) -> BodyCommandRouter:
    router = getattr(client, "_q5_body_command_router", None)
    if router is None:
        router = BodyCommandRouter(executor)
        setattr(client, "_q5_body_command_router", router)
    return router
