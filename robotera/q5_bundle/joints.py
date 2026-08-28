"""Q5 real-time skeleton card backed by the complete JointState snapshot."""

from __future__ import annotations

import json
import time
import xml.etree.ElementTree as ET
from functools import lru_cache
from pathlib import Path

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

CARD = "joints"
MODEL = "model"
TYPE = "sensor"
TOPIC = "/{ns}/q5/joints"
FMT = "sensor/skeleton"
HZ = 10.0
NODE = "q5_joints"
DESC = "Q5 实时身体与双手骨架：由 /joint_states 驱动的实机 URDF 可视化"
MODEL_PATH = Path(__file__).parent / "resource" / "q5_model.urdf"

# The Q5 JointState stream omits ``rota`` from its two index-finger motor
# names.  The URDF uses the full kinematic names, which the skeleton contract
# requires verbatim.
_SKELETON_NAME_ALIASES = {
    "left_hand_index_joint1": "left_hand_index_rota_joint1",
    "right_hand_index_joint1": "right_hand_index_rota_joint1",
}

# XHand Lite publishes one actuator state per finger. The URDF contains the
# passive distal revolute joint separately, so mirror the actuator angle into
# that joint for a faithful curled-finger visualization.
_HAND_DISTAL_JOINTS = {
    "left_hand_index_rota_joint1": "left_hand_index_rota_joint2",
    "left_hand_mid_joint1": "left_hand_mid_joint2",
    "left_hand_ring_joint1": "left_hand_ring_joint2",
    "left_hand_pinky_joint1": "left_hand_pinky_joint2",
    "right_hand_index_rota_joint1": "right_hand_index_rota_joint2",
    "right_hand_mid_joint1": "right_hand_mid_joint2",
    "right_hand_ring_joint1": "right_hand_ring_joint2",
    "right_hand_pinky_joint1": "right_hand_pinky_joint2",
    "left_hand_thumb_rota_joint1": "left_hand_thumb_rota_joint2",
    "right_hand_thumb_rota_joint1": "right_hand_thumb_rota_joint2",
}


def _skeleton_urdf() -> str:
    """Return only the URDF kinematic tree understood by the skeleton viewer.

    The vendor file also embeds a ``ros2_control`` block.  Its actuator and
    transmission declarations repeat the same joint names but have no parent
    or child links.  That is valid for ROS control, but it corrupts browser
    kinematics when all ``<joint>`` elements are indexed by name.
    """
    root = ET.parse(MODEL_PATH).getroot()
    for element in list(root):
        if element.tag in {"mujoco", "ros2_control"}:
            root.remove(element)
    return ET.tostring(root, encoding="unicode")


@lru_cache(maxsize=1)
def _model_joint_indices() -> dict[str, int]:
    """Return the renderer's stable index for each kinematic URDF joint."""
    root = ET.fromstring(_skeleton_urdf())
    return {
        joint.get("name"): index
        for index, joint in enumerate(root.findall("joint"))
        if joint.get("name")
    }


@lru_cache(maxsize=1)
def _model_joint_limits() -> dict[str, tuple[float, float]]:
    root = ET.fromstring(_skeleton_urdf())
    limits = {}
    for joint in root.findall("joint"):
        limit = joint.find("limit")
        if limit is None or limit.get("lower") is None or limit.get("upper") is None:
            continue
        limits[joint.get("name")] = (float(limit.get("lower")), float(limit.get("upper")))
    return limits


def build(snap: dict) -> dict:
    positions = snap.get("joints", {})
    names = snap.get("joint_names", [])
    model_indices = _model_joint_indices()
    # The skeleton contract requires a stable index as well as the exact URDF
    # joint name.  Keep the incoming JointState order: it is the robot's
    # authoritative ordering and avoids reordering hand/body joints in the UI.
    joints = []
    for message_idx, name in enumerate(names):
        if name not in positions:
            continue
        # idx is the URDF/model index, not the arbitrary order of one
        # JointState message.  The latter changes between publishers and
        # causes renderers that use idx to apply angles to the wrong joints.
        model_name = _SKELETON_NAME_ALIASES.get(name, name)
        item = {"idx": model_indices.get(model_name, message_idx), "name": model_name, "q": positions[name]}
        if model_name != name:
            item["source_name"] = name
        if name in snap.get("velocities", {}):
            item["dq"] = snap["velocities"][name]
        if name in snap.get("efforts", {}):
            item["tau"] = snap["efforts"][name]
        joints.append(item)
        distal_name = _HAND_DISTAL_JOINTS.get(model_name)
        if distal_name and distal_name not in positions and distal_name in model_indices:
            lower, upper = _model_joint_limits().get(distal_name, (float("-inf"), float("inf")))
            distal_q = max(lower, min(upper, float(positions[name])))
            joints.append({"idx": model_indices[distal_name], "name": distal_name,
                           "q": distal_q, "source_name": name, "derived_from": model_name})
    return {
        "timestamp_ms": int(time.time() * 1000),
        "received_at_ms": snap.get("received_at_ms"),
        "message_timestamp_ms": snap.get("message_timestamp_ms"),
        "fresh": bool(snap.get("fresh", False)),
        "available": bool(snap.get("available", False)),
        "age_ms": snap.get("age_ms"),
        "stale": bool(snap.get("stale", False)),
        "format": FMT,
        "joints": joints,
        "joint_count": len(joints),
        "position_unit": snap.get("position_unit", "rad"),
        "source_topic": "/joint_states",
        "message": (
            None if snap.get("fresh", False)
            else "关节状态消息已过期" if snap.get("available", False)
            else "未收到 /joint_states 消息"
        ),
    }


class Plugin:
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
            except Exception as e:
                print(f"[{CARD}] ROS2 发布不可用，退回 MCP 轮询: {e}", flush=True)
                self._node = None

    def _tick(self):
        msg = String()
        msg.data = json.dumps(build(self._client.snapshot()), ensure_ascii=False)
        self._pub.publish(msg)

    def get_tools(self):
        return [
            {
                "name": CARD,
                "type": TYPE,
                "multiInstance": False,
                "description": DESC + (f" -> {self._topic}" if self._node else " — poll via MCP action=info"),
                "inputSchema": {
                    "type": "object",
                    "properties": {"action": {"type": "string", "enum": ["info", "start", "stop"]}},
                    "required": ["action"],
                    "additionalProperties": False,
                },
                "topic_out": topic_out(self._topic, FMT),
            },
            {
                "name": MODEL,
                "type": "resource",
                "multiInstance": False,
                "description": "Q5 身体骨架 URDF，用于实时 joints 3D 可视化",
                "inputSchema": {"type": "object", "properties": {}},
            },
        ]

    def start(self):
        pass

    def stop(self):
        pass

    def dispatch(self, action, args):
        if action == MODEL:
            if not MODEL_PATH.exists():
                return {"error": "Q5 visual URDF model not found"}
            return {
                "urdf": _skeleton_urdf(),
                "model": "RobotEra Q5",
                "geometry": "q5_wr1_lite_robot_description",
                "source_topic": "/robot_description",
                "mesh_package": "robot_control/description/wr1/lite/meshes",
            }
        if action == "start":
            return {"state": "running"}
        if action == "stop":
            return {"state": "idle"}
        if action in ("info", "read", "get", CARD):
            return {"state": "running", "data": build(self._client.snapshot()),
                    "topic_out": ([{"topic": self._topic, "format": FMT}] if self._node else [])}
        return None


def make_plugin(plugin_config, namespace, executor, client):
    return Plugin(plugin_config, namespace, executor, client)
