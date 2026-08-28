"""Q5 business-state card (read-only)."""

from __future__ import annotations

import json
import time

from sensor_contract import topic_out

try:
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
    from std_msgs.msg import String
    _HAS_ROS2 = True
    _QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                      history=HistoryPolicy.KEEP_LAST, depth=1,
                      durability=DurabilityPolicy.VOLATILE)
except Exception:
    _HAS_ROS2 = False

CARD = "robot_ready"
TYPE = "sensor"
TOPIC = "/{ns}/q5/robot_ready"
FMT = "data/json"
HZ = 2.0
NODE = "q5_robot_ready"
DESC = "Q5 当前业务状态：以 /xbot_state 的 READY/ACTIVE 等厂商状态为准"

STALE_THRESHOLD_MS = 5000
ROBOT_STATE_LABELS = {
    0: "INIT", 1: "SELF_TEST", 2: "IDLE", 3: "READY", 4: "ACTIVE",
    5: "SHUTDOWN", 6: "OTA", 7: "E_STOP", -1: "ERROR",
}
CONTROL_READY_STATES = {3, 4}


def build(snap: dict, lifecycle_state: str = "unknown", robot_status: dict | None = None) -> dict:
    """综合评估就绪状态。"""
    d = {
        "timestamp_ms": int(time.time() * 1000),
    }

    # 维度 1：消息新鲜度
    fresh = bool(snap.get("fresh", False))
    age_ms = snap.get("age_ms", -1)
    d["message_freshness"] = {
        "fresh": fresh,
        "age_ms": age_ms,
        "stale_threshold_ms": STALE_THRESHOLD_MS,
    }

    robot_status = robot_status or {"available": False, "fresh": False}
    robot_state = robot_status.get("state")
    robot_state_label = ROBOT_STATE_LABELS.get(robot_state, "UNKNOWN")
    robot_state_ready = (
        bool(robot_status.get("available", False))
        and bool(robot_status.get("fresh", False))
        and robot_state in CONTROL_READY_STATES
    )

    # This is the primary card state.  Do not substitute the ROS lifecycle
    # label here: lifecycle "active" and Q5 RobotStatus "READY"/"ACTIVE"
    # describe different state machines.
    d["robot_status"] = {
        "available": bool(robot_status.get("available", False)),
        "fresh": bool(robot_status.get("fresh", False)),
        "age_ms": robot_status.get("age_ms"),
        "state": robot_state,
        "state_label": robot_state_label,
        "ready": robot_state_ready,
        "message": robot_status.get("message"),
        "source": "/xbot_state",
    }
    d["robot_state"] = robot_state_label

    # Motion-manager lifecycle is an independent prerequisite used by control
    # cards.  It is deliberately named so it cannot be mistaken for the Q5
    # RobotStatus reported above.
    d["motion_manager_lifecycle"] = {
        "state": lifecycle_state,
        "active": lifecycle_state == "active",
        "source": "/motion_manager/get_state",
    }

    # Command authority belongs to the individual control cards and is not a
    # useful state reading for this card.
    motion_ready = fresh and lifecycle_state == "active"
    d["motion_ready"] = motion_ready
    d["ready"] = robot_state_ready
    d["ready_scope"] = "robot_status"
    d["available"] = bool(robot_status.get("available", False))

    if not robot_status.get("available", False) or not robot_status.get("fresh", False):
        d["message"] = "机器人当前状态未知：未收到新鲜 /xbot_state 消息"
    elif robot_state_ready:
        d["message"] = "机器人当前状态：%s；运动管理器生命周期：%s" % (
            robot_state_label, lifecycle_state.upper())
    else:
        d["message"] = "机器人当前状态：%s；运动管理器生命周期：%s" % (
            robot_state_label, lifecycle_state.upper())

    return d


class Plugin:
    """就绪状态卡插件。"""

    def __init__(self, plugin_config, namespace, executor, client):
        self._client = client
        self._topic = TOPIC.format(ns=namespace)
        self._node = None
        if _HAS_ROS2 and executor is not None:
            try:
                self._node = Node(NODE)
                self._pub = self._node.create_publisher(String, self._topic, _QOS)
                self._node.create_timer(1.0 / HZ, self._tick)
                executor.add_node(self._node)
                self._node.get_logger().info(f"q5 robot_ready -> {self._topic} @ {HZ}Hz")
            except Exception as e:
                print(f"[{CARD}] ROS2 发布不可用，退回 MCP 轮询: {e}", flush=True)
                self._node = None

    def _tick(self):
        try:
            m = String()
            m.data = json.dumps(build(self._client.snapshot(),
                                      self._client.get_lifecycle_state(),
                                      self._client.sensor_snapshot("robot_status")))
            self._pub.publish(m)
        except Exception as e:
            self._node.get_logger().error(f"publish {self._topic} error: {e}")

    def get_tool(self):
        desc = DESC + (f" -> {self._topic}" if self._node else " — poll via MCP action=info")
        return {
            "name": CARD,
            "type": TYPE,
            "multiInstance": False,
            "description": desc,
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {
                        "type": "string",
                        "enum": ["info", "start", "stop"],
                        "description": "读取状态或控制卡片生命周期",
                    },
                },
                "required": ["action"],
                "additionalProperties": False,
            },
            "topic_out": topic_out(self._topic, FMT),
        }

    def start(self):
        pass

    def stop(self):
        pass

    def dispatch(self, action, args):
        if action == "start":
            return {"state": "running"}
        if action == "stop":
            return {"state": "idle"}
        if action in ("info", "read", "get", CARD):
            return {
                "state": "running",
                "data": build(self._client.snapshot(),
                              self._client.get_lifecycle_state(),
                              self._client.sensor_snapshot("robot_status")),
                "topic_out": topic_out(self._topic, FMT),
            }
        return None


def make_plugin(plugin_config, namespace, executor, client):
    return Plugin(plugin_config, namespace, executor, client)
