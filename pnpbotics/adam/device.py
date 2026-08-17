"""PNPbotics Adam driver — plugin classes.

Plugins:
  StatePlugin  — DDS rt/lowstate + rt/handstate → ROS2 skeleton/IMU/battery
  LocoPlugin   — gRPC locomotion control
  ArmPlugin    — ROS2 JointState upper body control
  HandPlugin   — DDS rt/handcmd finger control
  ModelPlugin  — URDF resource for 3D visualization
"""

import json
import io
import math
import os
import queue
import struct
import sys
import threading
import time
import zlib
from pathlib import Path

import numpy as np

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    from sensor_msgs.msg import JointState
    from std_msgs.msg import String

    HAS_ROS2 = True
except ImportError:
    HAS_ROS2 = False

try:
    from pndbotics_sdk_py.core.channel import (
        ChannelFactoryInitialize,
        ChannelPublisher,
        ChannelSubscriber,
    )
    from pndbotics_sdk_py.idl.pnd_adam.msg.dds_ import (
        LowState_,
        LowCmd_,
        HandCmd_,
        HandState_,
    )
    from pndbotics_sdk_py.idl.default import (
        pnd_adam_msg_dds__HandCmd_,
    )

    HAS_PND_SDK = True
except ImportError:
    HAS_PND_SDK = False


# ---------------------------------------------------------------------------
# Joint definitions per variant
# ---------------------------------------------------------------------------

ADAM_LITE_JOINTS = [
    "hipPitch_Left", "hipRoll_Left", "hipYaw_Left",
    "kneePitch_Left", "anklePitch_Left", "ankleRoll_Left",
    "hipPitch_Right", "hipRoll_Right", "hipYaw_Right",
    "kneePitch_Right", "anklePitch_Right", "ankleRoll_Right",
    "waistRoll", "waistPitch", "waistYaw",
    "shoulderPitch_Left", "shoulderRoll_Left", "shoulderYaw_Left", "elbow_Left",
    "shoulderPitch_Right", "shoulderRoll_Right", "shoulderYaw_Right", "elbow_Right",
]

ADAM_SP_JOINTS = [
    "hipPitch_Left", "hipRoll_Left", "hipYaw_Left",
    "kneePitch_Left", "anklePitch_Left", "ankleRoll_Left",
    "hipPitch_Right", "hipRoll_Right", "hipYaw_Right",
    "kneePitch_Right", "anklePitch_Right", "ankleRoll_Right",
    "waistRoll", "waistPitch", "waistYaw",
    "shoulderPitch_Left", "shoulderRoll_Left", "shoulderYaw_Left", "elbow_Left",
    "wristYaw_Left", "wristPitch_Left", "wristRoll_Left",
    "shoulderPitch_Right", "shoulderRoll_Right", "shoulderYaw_Right", "elbow_Right",
    "wristYaw_Right", "wristPitch_Right", "wristRoll_Right",
]

ADAM_PRO_JOINTS = [
    "hipPitch_Left", "hipRoll_Left", "hipYaw_Left",
    "kneePitch_Left", "anklePitch_Left", "ankleRoll_Left",
    "hipPitch_Right", "hipRoll_Right", "hipYaw_Right",
    "kneePitch_Right", "anklePitch_Right", "ankleRoll_Right",
    "waistRoll", "waistPitch", "waistYaw",
    "neckYaw", "neckPitch",
    "shoulderPitch_Left", "shoulderRoll_Left", "shoulderYaw_Left", "elbow_Left",
    "wristYaw_Left", "wristPitch_Left", "wristRoll_Left",
    "shoulderPitch_Right", "shoulderRoll_Right", "shoulderYaw_Right", "elbow_Right",
    "wristYaw_Right", "wristPitch_Right", "wristRoll_Right",
]

VARIANT_JOINTS = {
    "lite": ADAM_LITE_JOINTS,
    "sp": ADAM_SP_JOINTS,
    "pro": ADAM_PRO_JOINTS,
}

VARIANT_DOF = {"lite": 23, "sp": 29, "pro": 31}

# ROS2 JointState joint names for upper body control (used by ArmPlugin)
ROS2_UPPER_BODY_JOINTS = [
    "dof_pos/waistRoll", "dof_pos/waistPitch", "dof_pos/waistYaw",
    "dof_pos/shoulderPitch_Left", "dof_pos/shoulderRoll_Left",
    "dof_pos/shoulderYaw_Left", "dof_pos/elbow_Left",
    "dof_pos/wristYaw_Left", "dof_pos/wristPitch_Left", "dof_pos/wristRoll_Left",
    "dof_pos/shoulderPitch_Right", "dof_pos/shoulderRoll_Right",
    "dof_pos/shoulderYaw_Right", "dof_pos/elbow_Right",
    "dof_pos/wristYaw_Right", "dof_pos/wristPitch_Right", "dof_pos/wristRoll_Right",
    "root_pos/z",
    "dof_pos/hand_pinky_Left", "dof_pos/hand_ring_Left",
    "dof_pos/hand_middle_Left", "dof_pos/hand_index_Left",
    "dof_pos/hand_thumb_1_Left", "dof_pos/hand_thumb_2_Left",
    "dof_pos/hand_pinky_Right", "dof_pos/hand_ring_Right",
    "dof_pos/hand_middle_Right", "dof_pos/hand_index_Right",
    "dof_pos/hand_thumb_1_Right", "dof_pos/hand_thumb_2_Right",
]


def _best_effort_qos():
    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
    )


# ===========================================================================
# StatePlugin — subscribes DDS rt/lowstate, publishes to ROS2
# ===========================================================================

class _StatePublisherNode(Node):
    """ROS2 node that publishes skeleton, IMU, and battery data."""

    def __init__(self, namespace: str, variant: str, publish_rate_hz: float):
        super().__init__("adam_state_publisher")
        self._namespace = namespace
        self._variant = variant
        self._joints = VARIANT_JOINTS[variant]

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self._topic_skeleton = f"/{namespace}/state/joints"
        self._topic_imu = f"/{namespace}/state/imu"
        self._topic_battery = f"/{namespace}/state/battery"

        self._pub_skeleton = self.create_publisher(String, self._topic_skeleton, qos)
        self._pub_imu = self.create_publisher(String, self._topic_imu, qos)
        self._pub_battery = self.create_publisher(String, self._topic_battery, qos)

        self._latest_state = None
        self._latest_hand = None
        self._lock = threading.Lock()

        interval = 1.0 / publish_rate_hz
        self._timer = self.create_timer(interval, self._publish)

    def update_state(self, state):
        with self._lock:
            self._latest_state = state

    def update_hand(self, hand):
        with self._lock:
            self._latest_hand = hand

    def _publish(self):
        with self._lock:
            state = self._latest_state
            hand = self._latest_hand

        if state is None:
            return

        # Skeleton (joints)
        joints = []
        for idx, name in enumerate(self._joints):
            if idx < len(state.motor_state):
                joints.append({
                    "idx": idx,
                    "name": name,
                    "q": float(state.motor_state[idx].q),
                })
        msg = String()
        msg.data = json.dumps({"joints": joints})
        self._pub_skeleton.publish(msg)

        # IMU
        imu = state.imu_state
        imu_data = {
            "quaternion": list(imu.quaternion),
            "gyroscope": list(imu.gyroscope),
            "accelerometer": list(imu.accelerometer),
            "ypr": list(imu.ypr),
            "temperature": int(imu.temperature),
        }
        msg_imu = String()
        msg_imu.data = json.dumps(imu_data)
        self._pub_imu.publish(msg_imu)

        # Battery
        bat = state.battery_data
        bat_data = {
            "voltage": float(bat.voltage),
            "current": float(bat.current),
            "power": float(bat.power),
            "wh_accumulated": float(bat.wh_accumulated),
            "status": str(bat.status) if hasattr(bat, "status") else "unknown",
        }
        msg_bat = String()
        msg_bat.data = json.dumps(bat_data)
        self._pub_battery.publish(msg_bat)


class StatePlugin:
    """Subscribes DDS rt/lowstate and rt/handstate, publishes to ROS2."""

    PREFIX = "state"

    def __init__(self, plugin_config: dict, namespace: str, executor,
                 variant: str, dds_lowstate_sub=None, dds_handstate_sub=None, **kwargs):
        self._namespace = namespace
        self._variant = variant
        self._running = False

        rate = plugin_config.get("publish_rate_hz", 50)
        self._node = _StatePublisherNode(namespace, variant, rate)
        executor.add_node(self._node)

        # DDS subscribers (pre-created in main.py before rclpy.init to avoid conflict)
        self._lowstate_sub = dds_lowstate_sub
        self._handstate_sub = dds_handstate_sub

        # Start polling thread for DDS data
        if self._lowstate_sub or self._handstate_sub:
            self._poll_thread = threading.Thread(target=self._poll_dds, daemon=True)
            self._poll_thread.start()

    def _poll_dds(self):
        """Poll DDS subscribers in a background thread."""
        while True:
            if self._lowstate_sub:
                try:
                    msg = self._lowstate_sub.Read(timeout=1)
                    if msg:
                        self._node.update_state(msg)
                except Exception:
                    pass
            if self._handstate_sub:
                try:
                    msg = self._handstate_sub.Read(timeout=0)
                    if msg:
                        self._node.update_hand(msg)
                except Exception:
                    pass

    def get_tools(self) -> list:
        return [
            {
                "name": "joints",
                "type": "sensor",
                "description": "Adam joint state — real-time skeleton visualization",
                "inputSchema": {"type": "object", "properties": {}},
                "topic_out": [
                    {"topic": self._node._topic_skeleton, "format": "sensor/skeleton"}
                ],
            },
            {
                "name": "imu",
                "type": "sensor",
                "description": "Adam IMU — quaternion, gyroscope, accelerometer",
                "inputSchema": {"type": "object", "properties": {}},
                "topic_out": [
                    {"topic": self._node._topic_imu, "format": "data/json"}
                ],
            },
            {
                "name": "battery",
                "type": "sensor",
                "description": "Adam battery — voltage, current, power, status",
                "inputSchema": {"type": "object", "properties": {}},
                "topic_out": [
                    {"topic": self._node._topic_battery, "format": "data/json"}
                ],
            },
        ]

    def start(self):
        self._running = True

    def stop(self):
        self._running = False

    def dispatch(self, action: str, args: dict) -> dict:
        if action == "start":
            self._running = True
            return {"state": "running"}
        if action == "stop":
            self._running = False
            return {"state": "idle"}
        if action == "info":
            tool_name = args.get("_tool_name", "joints")
            if tool_name == "imu":
                return {"state": "running" if self._running else "idle",
                        "topic_out": [{"topic": self._node._topic_imu, "format": "data/json"}]}
            if tool_name == "battery":
                return {"state": "running" if self._running else "idle",
                        "topic_out": [{"topic": self._node._topic_battery, "format": "data/json"}]}
            return {"state": "running" if self._running else "idle",
                    "topic_out": [{"topic": self._node._topic_skeleton, "format": "sensor/skeleton"}]}
        return None


# ===========================================================================
# LocoPlugin — gRPC locomotion control
# ===========================================================================

class LocoPlugin:
    """High-level locomotion via gRPC on port 6666."""

    PREFIX = "loco"

    def __init__(self, plugin_config: dict, namespace: str, executor,
                 grpc_client, **kwargs):
        self._grpc = grpc_client
        self._namespace = namespace

    def get_tool(self) -> dict:
        return {
            "name": "loco",
            "type": "actuator",
            "description": "Adam locomotion — walk, turn, stop, gestures, mode switching",
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {
                        "type": "string",
                        "enum": [
                            "set_mode", "move", "stop", "stand_motion",
                            "stand_action", "stand_dynamic", "get_state",
                            "list_actions", "clear_error", "carry_box",
                        ],
                    },
                    "mode": {"type": "integer", "description": "Mode ID"},
                    "vx": {"type": "number", "description": "Forward velocity (m/s)"},
                    "vy": {"type": "number", "description": "Lateral velocity (m/s)"},
                    "vyaw": {"type": "number", "description": "Yaw angular velocity (rad/s)"},
                    "motion_id": {"type": "integer", "description": "Predefined motion ID"},
                    "action_id": {"type": "integer", "description": "Predefined action/gesture ID"},
                    "pitch": {"type": "number", "description": "Body pitch (rad)"},
                    "roll": {"type": "number", "description": "Body roll (rad)"},
                    "yaw": {"type": "number", "description": "Body yaw (rad)"},
                    "height": {"type": "number", "description": "Body height (m)"},
                    "enable": {"type": "boolean", "description": "Enable/disable flag"},
                },
                "required": ["action"],
                "x-action-params": {
                    "set_mode": {
                        "params": ["mode"],
                        "description": "Switch robot mode (e.g., stand, walk)",
                    },
                    "move": {
                        "params": ["vx", "vy", "vyaw"],
                        "description": "Walk with specified velocities",
                    },
                    "stop": {
                        "params": [],
                        "description": "Stop all movement",
                    },
                    "stand_motion": {
                        "params": ["motion_id"],
                        "description": "Execute predefined standing pose",
                    },
                    "stand_action": {
                        "params": ["action_id"],
                        "description": "Execute predefined gesture/action",
                    },
                    "stand_dynamic": {
                        "params": ["pitch", "roll", "yaw", "height"],
                        "description": "Adjust body orientation and height while standing",
                    },
                    "get_state": {
                        "params": [],
                        "description": "Query current robot state (mode, gait, battery)",
                    },
                    "list_actions": {
                        "params": [],
                        "description": "List available motions and actions",
                    },
                    "clear_error": {
                        "params": [],
                        "description": "Clear error state",
                    },
                    "carry_box": {
                        "params": ["enable"],
                        "description": "Enable/disable carry box mode",
                    },
                },
            },
        }

    def start(self):
        pass

    def stop(self):
        pass

    def dispatch(self, action: str, args: dict) -> dict:
        if action == "start":
            return {"state": "ready"}
        if action == "stop":
            return {"state": "idle"}
        if action == "set_mode":
            return self._grpc.set_mode(args.get("mode", 0))
        if action == "move":
            return self._grpc.set_speed(
                args.get("vx", 0.0), args.get("vy", 0.0), args.get("vyaw", 0.0)
            )
        if action == "stop_move" or action == "stop":
            return self._grpc.set_speed(0.0, 0.0, 0.0)
        if action == "stand_motion":
            return self._grpc.set_stand_motion(args.get("motion_id", 0))
        if action == "stand_action":
            return self._grpc.set_stand_action(args.get("action_id", 0))
        if action == "stand_dynamic":
            return self._grpc.set_stand_dynamic(
                pitch=args.get("pitch", 0.0),
                roll=args.get("roll", 0.0),
                yaw=args.get("yaw", 0.0),
                height=args.get("height", 0.0),
            )
        if action == "get_state":
            return self._grpc.get_robot_state()
        if action == "list_actions":
            return self._grpc.get_stand_list()
        if action == "clear_error":
            return self._grpc.set_error_clear()
        if action == "carry_box":
            return self._grpc.set_carry_box(args.get("enable", False))
        if action == "info":
            return {"state": "ready"}
        return None


# ===========================================================================
# ArmPlugin — ROS2 JointState upper body control
# ===========================================================================

class _ArmControlNode(Node):
    """ROS2 node that publishes JointState at 100Hz for upper body control."""

    def __init__(self, namespace: str, publish_rate_hz: float):
        super().__init__("adam_arm_controller")
        self._namespace = namespace

        self._pub = self.create_publisher(JointState, "joint_states", 10)
        self._joint_names = ROS2_UPPER_BODY_JOINTS
        self._positions = np.zeros(len(self._joint_names), dtype=np.float64)
        # Default height = 1.0m (standing), hands fully open = 1000
        self._positions[17] = 1.0  # root_pos/z
        self._positions[18:24] = 1000.0  # left hand fingers
        self._positions[24:30] = 1000.0  # right hand fingers

        self._active = False
        self._lock = threading.Lock()

        interval = 1.0 / publish_rate_hz
        self._timer = self.create_timer(interval, self._publish)

    def _publish(self):
        if not self._active:
            return
        with self._lock:
            positions = self._positions.copy()

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self._joint_names
        msg.position = positions.tolist()
        msg.velocity = [0.0] * len(self._joint_names)
        msg.effort = [0.0] * len(self._joint_names)
        self._pub.publish(msg)

    def set_joints(self, joints_dict: dict):
        """Set joint positions by name. Keys are short names like 'shoulderPitch_Left'."""
        with self._lock:
            for name, value in joints_dict.items():
                # Try to find matching joint
                full_name = f"dof_pos/{name}"
                if full_name in self._joint_names:
                    idx = self._joint_names.index(full_name)
                    self._positions[idx] = float(value)
                elif name in self._joint_names:
                    idx = self._joint_names.index(name)
                    self._positions[idx] = float(value)

    def set_height(self, z: float):
        z = max(0.6, min(1.0, z))
        with self._lock:
            self._positions[17] = z

    def zero_arms(self):
        with self._lock:
            self._positions[:17] = 0.0
            self._positions[17] = 1.0  # keep standing height


class ArmPlugin:
    """Upper body control via ROS2 JointState publishing at 100Hz."""

    PREFIX = "arm"

    def __init__(self, plugin_config: dict, namespace: str, executor,
                 grpc_client=None, **kwargs):
        self._namespace = namespace
        self._grpc = grpc_client

        rate = plugin_config.get("publish_rate_hz", 100)
        self._node = _ArmControlNode(namespace, rate)
        executor.add_node(self._node)

    def get_tool(self) -> dict:
        return {
            "name": "arm",
            "type": "actuator",
            "description": "Adam upper body — waist, arms, wrists via ROS2 JointState at 100Hz",
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {
                        "type": "string",
                        "enum": ["enable", "disable", "set_joints", "set_height", "zero"],
                    },
                    "joints": {
                        "type": "object",
                        "description": "Joint name → radian value pairs (e.g., {\"shoulderPitch_Left\": 0.5})",
                    },
                    "height": {
                        "type": "number",
                        "description": "Body height 0.6-1.0m",
                    },
                },
                "required": ["action"],
                "x-action-params": {
                    "enable": {
                        "params": [],
                        "description": "Activate upper body retarget mode (robot must be standing)",
                    },
                    "disable": {
                        "params": [],
                        "description": "Deactivate upper body retarget mode",
                    },
                    "set_joints": {
                        "params": ["joints"],
                        "description": "Set arm/waist joint angles in radians",
                    },
                    "set_height": {
                        "params": ["height"],
                        "description": "Set body height (0.6-1.0m)",
                    },
                    "zero": {
                        "params": [],
                        "description": "Reset all arm joints to zero (neutral position)",
                    },
                },
            },
        }

    def start(self):
        pass

    def stop(self):
        self._node._active = False

    def dispatch(self, action: str, args: dict) -> dict:
        if action == "start":
            return {"state": "ready"}
        if action == "stop":
            self._node._active = False
            return {"state": "idle"}
        if action == "enable":
            self._node._active = True
            return {"state": "active", "message": "Upper body retarget mode enabled"}
        if action == "disable":
            self._node._active = False
            return {"state": "idle", "message": "Upper body retarget mode disabled"}
        if action == "set_joints":
            joints = args.get("joints", {})
            self._node.set_joints(joints)
            return {"state": "active", "joints_set": len(joints)}
        if action == "set_height":
            h = args.get("height", 1.0)
            self._node.set_height(h)
            return {"state": "active", "height": h}
        if action == "zero":
            self._node.zero_arms()
            return {"state": "active", "message": "Arms zeroed"}
        if action == "info":
            return {"state": "active" if self._node._active else "idle"}
        return None


# ===========================================================================
# HandPlugin — DDS rt/handcmd finger control
# ===========================================================================

class HandPlugin:
    """Finger control via DDS rt/handcmd (PND hand: 0-1000, Inspire: 0-1800)."""

    PREFIX = "hand"

    def __init__(self, plugin_config: dict, namespace: str, executor,
                 dds_hand_pub=None, dds_hand_sub=None, **kwargs):
        self._namespace = namespace
        self._hand_type = plugin_config.get("hand_type", "pnd")
        self._max_val = 1000 if self._hand_type == "pnd" else 1800

        self._hand_pub = dds_hand_pub
        self._hand_sub = dds_hand_sub
        self._latest_hand_state = None
        self._lock = threading.Lock()

        # Poll hand state in background
        if self._hand_sub:
            threading.Thread(target=self._poll_hand, daemon=True).start()

    def _poll_hand(self):
        while True:
            try:
                msg = self._hand_sub.Read(timeout=1)
                if msg:
                    with self._lock:
                        self._latest_hand_state = msg
            except Exception:
                pass

    def get_tool(self) -> dict:
        return {
            "name": "hand",
            "type": "actuator",
            "description": f"Adam hand control — per-finger position (0=closed, {self._max_val}=open)",
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {
                        "type": "string",
                        "enum": ["open", "close", "set_fingers", "get_state"],
                    },
                    "left": {
                        "type": "array",
                        "items": {"type": "integer"},
                        "description": "6 values [pinky, ring, middle, index, thumb1, thumb2]",
                    },
                    "right": {
                        "type": "array",
                        "items": {"type": "integer"},
                        "description": "6 values [pinky, ring, middle, index, thumb1, thumb2]",
                    },
                },
                "required": ["action"],
                "x-action-params": {
                    "open": {
                        "params": [],
                        "description": "Open all fingers fully",
                    },
                    "close": {
                        "params": [],
                        "description": "Close all fingers (make fist)",
                    },
                    "set_fingers": {
                        "params": ["left", "right"],
                        "description": "Set individual finger positions",
                    },
                    "get_state": {
                        "params": [],
                        "description": "Read current finger positions",
                    },
                },
            },
        }

    def start(self):
        pass

    def stop(self):
        pass

    def _send_hand_cmd(self, positions: list):
        if not HAS_PND_SDK or self._hand_pub is None:
            return
        cmd = pnd_adam_msg_dds__HandCmd_()
        for i in range(min(12, len(positions))):
            cmd.position[i] = int(max(0, min(self._max_val, positions[i])))
        self._hand_pub.Write(cmd)

    def dispatch(self, action: str, args: dict) -> dict:
        if action == "start":
            return {"state": "ready"}
        if action == "stop":
            return {"state": "idle"}
        if action == "open":
            positions = [self._max_val] * 12
            self._send_hand_cmd(positions)
            return {"state": "done", "message": "All fingers opened"}
        if action == "close":
            positions = [0] * 12
            self._send_hand_cmd(positions)
            return {"state": "done", "message": "All fingers closed"}
        if action == "set_fingers":
            left = args.get("left", [self._max_val] * 6)
            right = args.get("right", [self._max_val] * 6)
            positions = list(left[:6]) + list(right[:6])
            # Pad if incomplete
            while len(positions) < 12:
                positions.append(self._max_val)
            self._send_hand_cmd(positions)
            return {"state": "done", "left": left[:6], "right": right[:6]}
        if action == "get_state":
            with self._lock:
                state = self._latest_hand_state
            if state is None:
                return {"state": "unknown", "message": "No hand state received yet"}
            return {
                "state": "ok",
                "left": list(state.position[:6]),
                "right": list(state.position[6:12]),
            }
        if action == "info":
            return {"state": "ready"}
        return None


# ---------------------------------------------------------------------------
# Local ZED Mini camera cards
# ---------------------------------------------------------------------------

class _LatestFrameQueue:
    """A bounded queue that keeps the newest frame and drops stale frames."""

    def __init__(self):
        self._queue = queue.Queue(maxsize=1)

    def put_latest(self, frame):
        while True:
            try:
                self._queue.put_nowait(frame)
                return
            except queue.Full:
                try:
                    self._queue.get_nowait()
                except queue.Empty:
                    continue

    def get(self, timeout):
        return self._queue.get(timeout=timeout)


class ZedCameraPlugin:
    """Publish the Adam ZED Mini through the local ZED Python SDK.

    The camera is physically attached to the Jetson running this container, so
    there is no reason to consume the separate ZED network-stream sender.  One
    capture thread owns the SDK camera, copies requested streams into bounded
    latest-frame queues, and dedicated workers perform the expensive encoding,
    conversion and ROS2 publication for RGB, depth and the optional point
    cloud.
    """

    PREFIX = "camera"

    _CARD_NAMES = (
        "camera_head",
        "camera_depth",
        "camera_pointcloud",
    )

    _FORMATS = {
        "camera_head": "image/jpeg",
        "camera_depth": "image/depth-zlib",
        "camera_pointcloud": "sensor/pointcloud",
    }

    def __init__(self, plugin_config, namespace, executor):
        self._config = dict(plugin_config or {})
        self._namespace = namespace
        self._topics = {
            "camera_head": f"/{namespace}/camera/head",
            "camera_depth": f"/{namespace}/camera/head/depth",
            "camera_pointcloud": f"/{namespace}/camera/head/points",
        }

        pointcloud_config = self._config.get("pointcloud", {})
        if not isinstance(pointcloud_config, dict):
            pointcloud_config = {}
        self._pointcloud_config = pointcloud_config
        self._pointcloud_enabled = bool(
            pointcloud_config.get(
                "enabled", self._config.get("pointcloud_enabled", False)))
        # The three cards share one ZED capture thread, but each card has its
        # own publication lifecycle.  RGB and depth are opt-in because they
        # are expensive image streams, matching the point-cloud card's
        # on-demand behaviour.
        self._card_enabled = {
            "camera_head": False,
            "camera_depth": False,
            "camera_pointcloud": self._pointcloud_enabled,
        }
        self._rgb_hz = max(1.0, min(float(self._config.get("rgb_hz", 15)), 30.0))
        self._depth_hz = max(1.0, min(float(self._config.get("depth_hz", 8)), 15.0))
        self._pointcloud_hz = max(
            0.2, min(float(pointcloud_config.get("hz", 2)), 10.0))
        self._jpeg_quality = max(
            20, min(int(self._config.get("jpeg_quality", 70)), 95))
        self._max_points = max(
            1000, min(int(pointcloud_config.get("max_points", 10000)), 40000))
        self._max_point_distance_m = max(
            1.0, min(float(pointcloud_config.get("max_distance_m", 8.0)), 30.0))
        mount_rotation = pointcloud_config.get("mount_rotation_deg", {})
        if not isinstance(mount_rotation, dict):
            mount_rotation = {}
        self._pointcloud_mount_rotation_deg = {
            axis: float(mount_rotation.get(axis, 0.0))
            for axis in ("x", "y", "z")
        }
        self._pointcloud_mount_rotation = self._rotation_matrix_xyz(
            *(math.radians(self._pointcloud_mount_rotation_deg[axis])
              for axis in ("x", "y", "z")))
        mount_translation = pointcloud_config.get("mount_translation_m", {})
        if not isinstance(mount_translation, dict):
            mount_translation = {}
        self._pointcloud_mount_translation_m = {
            axis: float(mount_translation.get(axis, 0.0))
            for axis in ("x", "y", "z")
        }
        self._resolution_name = str(self._config.get("resolution", "VGA")).upper()
        self._depth_mode_name = str(
            self._config.get("depth_mode", "PERFORMANCE")).upper()
        self._camera_fps = max(1, min(int(self._config.get("fps", 15)), 60))
        # The ZED Mini on Adam's head is physically mounted upside down.  Let
        # the SDK rotate the complete camera data path (RGB, depth and point
        # cloud) together so the three outputs remain pixel/geometry aligned.
        self._camera_flip = bool(self._config.get("camera_flip", True))

        self._running = False
        self._available = False
        self._camera = None
        self._capture_thread = None
        self._worker_threads = []
        self._frame_queues = {}
        self._stop_event = threading.Event()
        self._lifecycle_lock = threading.RLock()
        self._publish_locks = {
            "rgb": threading.Lock(),
            "depth": threading.Lock(),
            "pointcloud": threading.Lock(),
        }
        self._rgb_pub = None
        self._depth_pub = None
        self._pointcloud_pub = None
        self._lock = threading.Lock()
        self._state = {
            "state": "idle",
            "available": False,
            "source": "zed-sdk-local",
            "error": None,
            "pointcloud_enabled": self._pointcloud_enabled,
            "left_intrinsics": None,
            "right_intrinsics": None,
            "stereo_baseline_m": None,
            "stereo_translation_m": None,
        }

        self._pub_node = Node("adam_zed_camera")
        executor.add_node(self._pub_node)

    @staticmethod
    def _tool(name, description, topic, fmt, input_schema=None):
        return {
            "name": name,
            "type": "sensor",
            "multiInstance": False,
            "description": description,
            "inputSchema": input_schema or {"type": "object", "properties": {}},
            "topic_out": [{"topic": topic, "format": fmt}],
        }

    def get_tools(self):
        return [
            self._tool(
                "camera_head",
                "Adam ZED Mini left RGB image from the local Jetson ZED SDK",
                self._topics["camera_head"], self._FORMATS["camera_head"]),
            self._tool(
                "camera_depth",
                "Adam ZED Mini depth image, zlib-compressed little-endian uint16 millimetres",
                self._topics["camera_depth"], self._FORMATS["camera_depth"]),
            self._tool(
                "camera_pointcloud",
                "Adam ZED Mini XYZ point cloud for the Phanthymotus 3D renderer; runtime-toggleable",
                self._topics["camera_pointcloud"], self._FORMATS["camera_pointcloud"],
                {
                    "type": "object",
                    "properties": {
                        "action": {
                            "type": "string",
                            "enum": ["start", "stop", "info"],
                            "description": "Enable or disable point-cloud publishing without reopening the camera",
                        },
                    },
                }),
        ]

    def start(self):
        with self._lifecycle_lock:
            if self._running:
                if (self._capture_thread is None
                        or self._capture_thread.is_alive()):
                    return True
                # The capture thread died outside its normal error path.  Let
                # the cleanup below close any stale camera before restarting.
                self._running = False

            # A capture loop can stop unexpectedly after opening the camera.
            # Do not create a replacement thread until the old one and all of
            # its camera calls have definitely finished.
            if (self._capture_thread is not None
                    or self._worker_threads
                    or self._camera is not None):
                if not self.stop():
                    return False

            self._stop_event.clear()
            self._running = True
            try:
                from sensor_msgs.msg import CompressedImage
                from std_msgs.msg import UInt8MultiArray

                qos = _best_effort_qos()
                self._CompressedImage = CompressedImage
                self._UInt8MultiArray = UInt8MultiArray
                self._rgb_pub = self._pub_node.create_publisher(
                    CompressedImage, self._topics["camera_head"], qos)
                self._depth_pub = self._pub_node.create_publisher(
                    CompressedImage, self._topics["camera_depth"], qos)
                self._pointcloud_pub = self._pub_node.create_publisher(
                    UInt8MultiArray, self._topics["camera_pointcloud"], qos)
            except Exception as exc:
                self._running = False
                self._stop_event.set()
                self._destroy_publishers()
                self._set_error(f"ROS2 camera publisher setup failed: {exc}")
                return False

            self._frame_queues = {
                "rgb": _LatestFrameQueue(),
                "depth": _LatestFrameQueue(),
                "pointcloud": _LatestFrameQueue(),
            }
            self._worker_threads = [
                threading.Thread(
                    target=self._rgb_worker,
                    daemon=True,
                    name="adam_zed_rgb_worker"),
                threading.Thread(
                    target=self._depth_worker,
                    daemon=True,
                    name="adam_zed_depth_worker"),
                threading.Thread(
                    target=self._pointcloud_worker,
                    daemon=True,
                    name="adam_zed_pointcloud_worker"),
            ]
            for worker in self._worker_threads:
                worker.start()

            self._capture_thread = threading.Thread(
                target=self._capture_loop, daemon=True, name="adam_zed_capture")
            self._capture_thread.start()
            return True

    def _destroy_publishers(self):
        # Destroy each publisher under its own lock so RGB publication cannot
        # wait behind a slow depth or point-cloud publication.
        for attr, lock_name in (
                ("_rgb_pub", "rgb"),
                ("_depth_pub", "depth"),
                ("_pointcloud_pub", "pointcloud")):
            with self._publish_locks[lock_name]:
                publisher = getattr(self, attr, None)
                setattr(self, attr, None)
                if publisher is None:
                    continue
                try:
                    self._pub_node.destroy_publisher(publisher)
                except Exception:
                    pass

    def stop(self):
        with self._lifecycle_lock:
            self._running = False
            self._stop_event.set()

            # Closing before join is intentional: it gives a blocking SDK
            # grab() a chance to return so that the capture thread can exit.
            camera = self._camera
            if camera is not None:
                try:
                    camera.close()
                except Exception:
                    pass

            thread = self._capture_thread
            if thread is not None and thread is not threading.current_thread():
                thread.join(timeout=3.0)
            workers = list(self._worker_threads)
            for worker in workers:
                if worker is not threading.current_thread():
                    worker.join(timeout=3.0)

            # Never clear the thread handles or destroy publishers while the
            # capture or processing workers can still call publish().  A later
            # start() will retry this cleanup before creating replacement
            # threads.
            capture_alive = thread is not None and thread.is_alive()
            workers_alive = any(worker.is_alive() for worker in workers)
            if capture_alive or workers_alive:
                self._available = False
                self._set_error(
                    "ZED capture or processing thread did not stop within "
                    "3 seconds; camera publishers were kept alive")
                return False

            self._capture_thread = None
            self._worker_threads = []
            self._frame_queues = {}
            self._camera = None
            self._available = False
            with self._lock:
                self._state.update({
                    "state": "idle",
                    "available": False,
                    "pointcloud_enabled": self._pointcloud_enabled,
                })
            self._destroy_publishers()
            return True

    def _set_error(self, message):
        with self._lock:
            self._available = False
            self._state.update({
                "state": "error",
                "available": False,
                "error": str(message),
            })
        print(f"[ZedCameraPlugin] {message}", flush=True)

    @staticmethod
    def _enum_name(value):
        name = getattr(value, "name", None)
        return str(name if name is not None else value)

    @staticmethod
    def _resolution_dict(resolution):
        return {
            "width": int(getattr(resolution, "width", 0)),
            "height": int(getattr(resolution, "height", 0)),
        }

    @staticmethod
    def _float_list(value):
        try:
            return [float(item) for item in value]
        except (TypeError, ValueError):
            return []

    def _camera_metadata(self, sdk_camera_info, params):
        configuration = sdk_camera_info.camera_configuration
        calibration = configuration.calibration_parameters
        left = calibration.left_cam
        right = calibration.right_cam
        translation = calibration.stereo_transform.get_translation().get()
        return {
            "state": "running",
            "available": True,
            "connected": True,
            "source": "zed-sdk-local",
            "resolution": self._resolution_dict(configuration.resolution),
            "fps": int(configuration.fps),
            "depth_mode": self._enum_name(params.depth_mode),
            "coordinate_units": self._enum_name(params.coordinate_units),
            "left_intrinsics": {
                "fx": float(left.fx), "fy": float(left.fy),
                "cx": float(left.cx), "cy": float(left.cy),
                "distortion": self._float_list(left.disto),
            },
            "right_intrinsics": {
                "fx": float(right.fx), "fy": float(right.fy),
                "cx": float(right.cx), "cy": float(right.cy),
                "distortion": self._float_list(right.disto),
            },
            "stereo_baseline_m": float(calibration.get_camera_baseline()),
            "stereo_translation_m": self._float_list(translation),
            "error": None,
            "pointcloud_enabled": self._pointcloud_enabled,
        }

    @staticmethod
    def _load_zed_module():
        try:
            import pyzed.sl as sl
            return sl
        except ImportError as first_error:
            # The deployment mounts the host's architecture-specific pyzed
            # extension at /opt/pyzed instead of baking a licensed SDK into
            # the driver image.
            candidate = os.environ.get("ZED_PYTHON_PATH", "/opt/pyzed")
            if candidate:
                candidate_path = Path(candidate)
                search_paths = [candidate_path]
                # When the package directory itself is mounted at
                # /opt/pyzed, Python needs its parent (/opt) on sys.path.
                if (candidate_path / "__init__.py").exists() or list(candidate_path.glob("sl*.so")):
                    search_paths.append(candidate_path.parent)
                for search_path in reversed(search_paths):
                    if search_path.exists() and str(search_path) not in sys.path:
                        sys.path.insert(0, str(search_path))
            try:
                import pyzed.sl as sl
                return sl
            except ImportError as second_error:
                raise ImportError(
                    "pyzed.sl is unavailable; mount the Jetson ZED SDK and set "
                    "ZED_PYTHON_PATH (or PYTHONPATH) accordingly"
                ) from second_error
            except Exception:
                raise first_error

    def _capture_active(self):
        return self._running and not self._stop_event.is_set()

    @staticmethod
    def _advance_deadline(deadline, period, now):
        """Advance a stream on an absolute schedule without jitter drift."""
        if deadline <= 0.0:
            return now + period
        deadline += period
        if deadline <= now:
            return now + period
        return deadline

    def _publish_capture_message(self, publisher, message, card_name=None):
        """Publish while keeping teardown from racing the ROS call."""
        if publisher is None:
            return False
        lock_name = {
            "camera_head": "rgb",
            "camera_depth": "depth",
            "camera_pointcloud": "pointcloud",
        }.get(card_name, "rgb")
        with self._publish_locks[lock_name]:
            if not self._capture_active():
                return False
            if card_name is not None:
                with self._lock:
                    if not self._card_enabled.get(card_name, False):
                        return False
            publisher.publish(message)
            return True

    def _capture_loop(self):
        try:
            self._capture_loop_body()
        except Exception as exc:
            if self._capture_active():
                self._running = False
                self._set_error(f"ZED capture loop failed: {exc}")
        finally:
            self._available = False

    def _capture_loop_body(self):
        try:
            import numpy as np
            sl = self._load_zed_module()
        except Exception as exc:
            if not self._capture_active():
                return
            self._running = False
            self._set_error(f"local ZED SDK import failed: {exc}")
            return

        if not self._capture_active():
            return

        params = sl.InitParameters()
        params.camera_resolution = getattr(
            sl.RESOLUTION, self._resolution_name, sl.RESOLUTION.VGA)
        depth_mode = getattr(sl.DEPTH_MODE, self._depth_mode_name, None)
        if depth_mode is None:
            depth_mode = getattr(sl.DEPTH_MODE, "NEURAL_LIGHT", sl.DEPTH_MODE.PERFORMANCE)
        params.depth_mode = depth_mode
        params.camera_fps = self._camera_fps
        params.coordinate_units = sl.UNIT.METER
        if self._camera_flip:
            if hasattr(params, "camera_image_flip"):
                flip_modes = getattr(sl, "FLIP_MODE", None)
                params.camera_image_flip = getattr(flip_modes, "ON", 1)
            else:
                print(
                    "[ZedCameraPlugin] camera_flip requested but this ZED SDK "
                    "does not expose camera_image_flip",
                    flush=True,
                )
        if hasattr(params, "depth_maximum_distance"):
            params.depth_maximum_distance = self._max_point_distance_m

        if not self._capture_active():
            return

        camera = sl.Camera()
        try:
            status = camera.open(params)
        except Exception as exc:
            if not self._capture_active():
                try:
                    camera.close()
                except Exception:
                    pass
                return
            self._running = False
            self._set_error(f"ZED camera open failed: {exc}")
            try:
                camera.close()
            except Exception:
                pass
            return
        if status != sl.ERROR_CODE.SUCCESS:
            if not self._capture_active():
                try:
                    camera.close()
                except Exception:
                    pass
                return
            self._running = False
            self._set_error(f"ZED camera open failed: {status}")
            try:
                camera.close()
            except Exception:
                pass
            return

        # stop() may have been called while the SDK was opening the camera.
        # Never publish or enter grab() after that stop request.
        if not self._capture_active():
            try:
                camera.close()
            except Exception:
                pass
            return
        self._camera = camera
        self._available = True
        try:
            metadata = self._camera_metadata(
                camera.get_camera_information(), params)
        except Exception as exc:
            self._available = False
            if self._capture_active():
                self._running = False
                self._set_error(f"ZED camera metadata read failed: {exc}")
            try:
                camera.close()
            except Exception:
                pass
            return
        if not self._capture_active():
            return
        with self._lock:
            self._state = metadata

        runtime = sl.RuntimeParameters()
        image = sl.Mat()
        depth = sl.Mat()
        pointcloud = sl.Mat()
        next_rgb = 0.0
        next_depth = 0.0
        next_pointcloud = 0.0
        rgb_period = 1.0 / self._rgb_hz
        depth_period = 1.0 / self._depth_hz
        pointcloud_period = 1.0 / self._pointcloud_hz
        rgb_queue = self._frame_queues["rgb"]
        depth_queue = self._frame_queues["depth"]
        pointcloud_queue = self._frame_queues["pointcloud"]
        last_grab_error = None

        try:
            while self._capture_active():
                try:
                    status = camera.grab(runtime)
                except Exception as exc:
                    if not self._capture_active():
                        break
                    self._running = False
                    self._set_error(f"ZED grab failed: {exc}")
                    break
                if status != sl.ERROR_CODE.SUCCESS:
                    if not self._capture_active():
                        break
                    if status != last_grab_error:
                        print(f"[ZedCameraPlugin] grab status: {status}", flush=True)
                        last_grab_error = status
                    self._stop_event.wait(0.01)
                    continue
                last_grab_error = None
                if not self._capture_active():
                    break
                now = time.monotonic()

                with self._lock:
                    rgb_enabled = self._card_enabled["camera_head"]
                    depth_enabled = self._card_enabled["camera_depth"]
                    pointcloud_enabled = (
                        self._card_enabled["camera_pointcloud"]
                        and self._pointcloud_enabled)

                if rgb_enabled and now >= next_rgb and self._capture_active():
                    try:
                        camera.retrieve_image(image, sl.VIEW.LEFT, sl.MEM.CPU)
                        rgb_queue.put_latest(
                            np.array(image.get_data(), copy=True))
                    except Exception as exc:
                        if self._capture_active():
                            self._set_error(f"RGB capture failed: {exc}")
                    next_rgb = self._advance_deadline(
                        next_rgb, rgb_period, now)

                need_depth = depth_enabled and now >= next_depth
                need_pointcloud = pointcloud_enabled and now >= next_pointcloud

                if need_depth and self._capture_active():
                    try:
                        camera.retrieve_measure(depth, sl.MEASURE.DEPTH, sl.MEM.CPU)
                        depth_queue.put_latest(
                            np.array(depth.get_data(), copy=True))
                    except Exception as exc:
                        if self._capture_active():
                            self._set_error(f"depth capture failed: {exc}")
                    next_depth = self._advance_deadline(
                        next_depth, depth_period, now)

                if need_pointcloud and self._capture_active():
                    try:
                        camera.retrieve_measure(
                            pointcloud, sl.MEASURE.XYZRGBA, sl.MEM.CPU)
                        pointcloud_queue.put_latest(
                            np.array(pointcloud.get_data(), copy=True))
                    except Exception as exc:
                        if self._capture_active():
                            self._set_error(f"pointcloud capture failed: {exc}")
                    next_pointcloud = self._advance_deadline(
                        next_pointcloud, pointcloud_period, now)
        finally:
            self._available = False
            if self._running and not self._stop_event.is_set():
                self._running = False
                self._set_error("ZED capture loop stopped unexpectedly")

    def _rgb_worker(self):
        try:
            import numpy as np
            from PIL import Image as PillowImage
        except Exception as exc:
            if self._capture_active():
                self._set_error(f"RGB worker import failed: {exc}")
            return

        frame_queue = self._frame_queues["rgb"]
        while self._capture_active():
            try:
                image = frame_queue.get(timeout=0.1)
            except queue.Empty:
                continue
            if not self._capture_active():
                break
            with self._lock:
                if not self._card_enabled["camera_head"]:
                    continue
            try:
                jpeg = self._encode_jpeg(image, np, PillowImage)
                msg = self._CompressedImage()
                msg.format = "jpeg"
                msg.data = jpeg
                self._publish_capture_message(
                    self._rgb_pub, msg, "camera_head")
            except Exception as exc:
                if self._capture_active():
                    self._set_error(f"RGB processing failed: {exc}")

    def _depth_worker(self):
        try:
            import numpy as np
        except Exception as exc:
            if self._capture_active():
                self._set_error(f"depth worker import failed: {exc}")
            return

        frame_queue = self._frame_queues["depth"]
        while self._capture_active():
            try:
                depth = frame_queue.get(timeout=0.1)
            except queue.Empty:
                continue
            if not self._capture_active():
                break
            with self._lock:
                if not self._card_enabled["camera_depth"]:
                    continue
            try:
                depth_mm = self._normalize_depth(depth, np)
                msg = self._CompressedImage()
                msg.format = "16UC1; compressedDepth zlib"
                msg.data = zlib.compress(
                    depth_mm.astype("<u2", copy=False).tobytes(), level=1)
                self._publish_capture_message(
                    self._depth_pub, msg, "camera_depth")
            except Exception as exc:
                if self._capture_active():
                    self._set_error(f"depth processing failed: {exc}")

    def _pointcloud_worker(self):
        try:
            import numpy as np
        except Exception as exc:
            if self._capture_active():
                self._set_error(f"pointcloud worker import failed: {exc}")
            return

        frame_queue = self._frame_queues["pointcloud"]
        while self._capture_active():
            try:
                pointcloud = frame_queue.get(timeout=0.1)
            except queue.Empty:
                continue
            if not self._capture_active():
                break
            with self._lock:
                if not (self._card_enabled["camera_pointcloud"]
                        and self._pointcloud_enabled):
                    continue
            try:
                payload = self._pack_pointcloud(pointcloud, np)
                if payload is None:
                    continue
                msg = self._UInt8MultiArray()
                msg.data = list(payload)
                self._publish_capture_message(
                    self._pointcloud_pub, msg, "camera_pointcloud")
            except Exception as exc:
                if self._capture_active():
                    self._set_error(f"pointcloud processing failed: {exc}")

    def _encode_jpeg(self, image, np, pillow_image):
        if image.ndim == 3 and image.shape[2] >= 3:
            # ZED's default U8_C4 CPU image is BGRA.  The dashboard expects
            # ordinary RGB JPEG bytes, so drop alpha and reverse BGR->RGB.
            rgb = image[:, :, :3][:, :, ::-1]
        elif image.ndim == 2:
            rgb = np.repeat(image[:, :, None], 3, axis=2)
        else:
            raise ValueError(f"unexpected ZED image shape {image.shape}")
        output = io.BytesIO()
        pillow_image.fromarray(np.ascontiguousarray(rgb), "RGB").save(
            output, format="JPEG", quality=self._jpeg_quality, optimize=False)
        return output.getvalue()

    @staticmethod
    def _normalize_depth(depth, np):
        if depth.ndim == 3:
            depth = depth[:, :, 0]
        valid = np.isfinite(depth) & (depth > 0.0)
        millimetres = np.zeros(depth.shape, dtype=np.uint16)
        millimetres[valid] = np.clip(
            depth[valid] * 1000.0, 0.0, 65535.0).astype(np.uint16)

        # The stock Phanthymotus depth renderer consumes a fixed 640x480
        # matrix.  Crop the ZED 16:9 image centrally, then nearest-neighbour
        # resample without introducing an OpenCV dependency.
        height, width = millimetres.shape
        if width * 3 > height * 4:
            crop_width = max(1, (height * 4) // 3)
            left = max(0, (width - crop_width) // 2)
            millimetres = millimetres[:, left:left + crop_width]
        elif width * 3 < height * 4:
            crop_height = max(1, (width * 3) // 4)
            top = max(0, (height - crop_height) // 2)
            millimetres = millimetres[top:top + crop_height, :]
        source_height, source_width = millimetres.shape
        rows = (np.arange(480) * source_height / 480).astype(np.int64)
        cols = (np.arange(640) * source_width / 640).astype(np.int64)
        return millimetres[rows[:, None], cols[None, :]]

    @staticmethod
    def _rotation_matrix_xyz(x_rad, y_rad, z_rad):
        """Return a renderer-frame rotation that applies X, then Y, then Z."""
        sx, cx = math.sin(x_rad), math.cos(x_rad)
        sy, cy = math.sin(y_rad), math.cos(y_rad)
        sz, cz = math.sin(z_rad), math.cos(z_rad)
        # Column-vector convention: Rz @ Ry @ Rx.
        return (
            (cz * cy, cz * sy * sx - sz * cx, cz * sy * cx + sz * sx),
            (sz * cy, sz * sy * sx + cz * cx, sz * sy * cx - cz * sx),
            (-sy, cy * sx, cy * cx),
        )

    def _pack_pointcloud(self, points, np):
        if points.ndim != 3 or points.shape[2] < 3:
            raise ValueError(f"unexpected ZED point-cloud shape {points.shape}")
        xyz = points[:, :, :3].reshape(-1, 3)
        valid = np.isfinite(xyz).all(axis=1)
        valid &= xyz[:, 2] > 0.05
        valid &= xyz[:, 2] <= self._max_point_distance_m
        xyz = xyz[valid]
        if xyz.size == 0:
            return None
        if xyz.shape[0] > self._max_points:
            stride = int(math.ceil(xyz.shape[0] / self._max_points))
            xyz = xyz[::stride][:self._max_points]

        # First express the optical ZED frame in the renderer's default frame:
        # (right, down, forward) -> (right, up, backward).  Correct the fixed
        # camera mounting angle in that frame, then invert the renderer's
        # configurable default mapping display=(packed_y,-packed_z,-packed_x).
        rotation = self._pointcloud_mount_rotation
        display = np.empty_like(xyz, dtype="<f4")
        camera_x = xyz[:, 0]
        camera_y = -xyz[:, 1]
        camera_z = -xyz[:, 2]
        display[:, 0] = (
            rotation[0][0] * camera_x
            + rotation[0][1] * camera_y
            + rotation[0][2] * camera_z)
        display[:, 1] = (
            rotation[1][0] * camera_x
            + rotation[1][1] * camera_y
            + rotation[1][2] * camera_z)
        display[:, 2] = (
            rotation[2][0] * camera_x
            + rotation[2][1] * camera_y
            + rotation[2][2] * camera_z)
        display[:, 0] += self._pointcloud_mount_translation_m["x"]
        display[:, 1] += self._pointcloud_mount_translation_m["y"]
        display[:, 2] += self._pointcloud_mount_translation_m["z"]
        packed_xyz = np.empty((xyz.shape[0], 3), dtype="<f4")
        packed_xyz[:, 0] = -display[:, 2]
        packed_xyz[:, 1] = display[:, 0]
        packed_xyz[:, 2] = -display[:, 1]
        return struct.pack("<II", 12, int(packed_xyz.shape[0])) + packed_xyz.tobytes()

    def _card_state(self, tool_name):
        with self._lock:
            enabled = self._card_enabled[tool_name]
            state = self._state.get("state", "idle")
            running = self._running

        if tool_name == "camera_pointcloud" and not enabled:
            return "disabled"
        if not enabled:
            return "idle"
        # start() launches the SDK capture loop asynchronously.  Report the
        # card as running during that short opening window; an SDK failure is
        # reported asynchronously through the shared error state.
        if state == "idle" and running:
            return "running"
        return state

    def _card_response(self, tool_name):
        response = {
            "state": self._card_state(tool_name),
            "topic_out": [{
                "topic": self._topics[tool_name],
                "format": self._FORMATS[tool_name],
            }],
        }
        if tool_name == "camera_pointcloud":
            response["pointcloud_enabled"] = self._pointcloud_enabled
        return response

    def _stop_if_no_cards_enabled(self):
        with self._lock:
            should_stop = not any(self._card_enabled.values())
        if should_stop:
            self.stop()

    def dispatch(self, action, args):
        tool_name = args.get("_tool_name", action)
        if tool_name not in self._CARD_NAMES:
            return {"state": self._state.get("state", "idle")}

        if tool_name == "camera_pointcloud" and action in ("start", "enable"):
            with self._lock:
                self._card_enabled[tool_name] = True
                self._pointcloud_enabled = True
                self._state["pointcloud_enabled"] = True
            if not self._running:
                self.start()
            return self._card_response(tool_name)

        if tool_name == "camera_pointcloud" and action in ("stop", "disable"):
            with self._lock:
                self._card_enabled[tool_name] = False
                self._pointcloud_enabled = False
                self._state["pointcloud_enabled"] = False
            self._stop_if_no_cards_enabled()
            return self._card_response(tool_name)

        if action == "start":
            with self._lock:
                self._card_enabled[tool_name] = True
            if not self._running:
                self.start()
            return self._card_response(tool_name)

        if action == "stop":
            with self._lock:
                self._card_enabled[tool_name] = False
            self._stop_if_no_cards_enabled()
            return self._card_response(tool_name)

        if action in ("info", tool_name):
            return self._card_response(tool_name)
        return {"state": self._state.get("state", "idle")}


# ---------------------------------------------------------------------------
# Resource card and bundle
# ---------------------------------------------------------------------------

class ModelPlugin:
    """Returns URDF for 3D skeleton visualization on dashboard."""

    PREFIX = "model"

    # Map variant to available URDF file (repo only has lite, sp, standard)
    _VARIANT_URDF = {
        "lite": "adam_lite.urdf",
        "sp": "adam_sp.urdf",
        "pro": "adam_pro.urdf",       # adam_standard used as fallback for pro
        "standard": "adam_pro.urdf",  # adam_standard stored as adam_pro
    }

    def __init__(self, plugin_config: dict, namespace: str, executor,
                 variant: str, **kwargs):
        self._variant = variant
        self._namespace = namespace
        # Resolve URDF file path
        urdf_name = self._VARIANT_URDF.get(variant, f"adam_{variant}.urdf")
        self._urdf_path = Path(__file__).parent / "resource" / urdf_name

    def get_tool(self) -> dict:
        return {
            "name": "model",
            "type": "resource",
            "description": f"Adam {self._variant} URDF model for 3D visualization",
            "inputSchema": {"type": "object", "properties": {}},
        }

    def start(self):
        pass

    def stop(self):
        pass

    def dispatch(self, action: str, args: dict) -> dict:
        if action == "start":
            return {"state": "running"}
        if action == "stop":
            return {"state": "idle"}
        # Return URDF content
        if self._urdf_path.exists():
            return {"urdf": self._urdf_path.read_text()}
        # Try any available URDF as fallback
        resource_dir = Path(__file__).parent / "resource"
        urdfs = list(resource_dir.glob("adam_*.urdf"))
        if urdfs:
            return {"urdf": urdfs[0].read_text(), "note": f"Fallback URDF ({urdfs[0].name})"}
        return {"error": f"No URDF found for variant '{self._variant}'"}


# ===========================================================================
# AdamDeviceBundle — aggregates all plugins
# ===========================================================================

class AdamDeviceBundle:
    """Loads and manages all Adam plugins based on config."""

    def __init__(self, config: dict, namespace: str, executor, grpc_client,
                 dds_lowstate_sub=None, dds_handstate_sub=None,
                 dds_hand_pub=None, dds_hand_sub=None):
        self._plugins = []
        self._tool_map = {}  # tool_name → plugin

        variant = config.get("variant", "sp")
        plugins_cfg = config.get("plugins", {})

        # StatePlugin
        if plugins_cfg.get("state", {}).get("enabled", True):
            p = StatePlugin(
                plugins_cfg.get("state", {}), namespace, executor,
                variant=variant,
                dds_lowstate_sub=dds_lowstate_sub,
                dds_handstate_sub=dds_handstate_sub,
            )
            self._plugins.append(p)

        # LocoPlugin
        if plugins_cfg.get("loco", {}).get("enabled", True):
            p = LocoPlugin(
                plugins_cfg.get("loco", {}), namespace, executor,
                grpc_client=grpc_client,
            )
            self._plugins.append(p)

        # CameraPlugin
        if plugins_cfg.get("camera", {}).get("enabled", False):
            p = ZedCameraPlugin(
                plugins_cfg.get("camera", {}), namespace, executor)
            self._plugins.append(p)

        # ArmPlugin
        if plugins_cfg.get("arm", {}).get("enabled", True):
            p = ArmPlugin(
                plugins_cfg.get("arm", {}), namespace, executor,
                grpc_client=grpc_client,
            )
            self._plugins.append(p)

        # HandPlugin
        if plugins_cfg.get("hand", {}).get("enabled", True):
            p = HandPlugin(plugins_cfg.get("hand", {}), namespace, executor,
                           dds_hand_pub=dds_hand_pub, dds_hand_sub=dds_hand_sub)
            self._plugins.append(p)

        # ModelPlugin
        if plugins_cfg.get("model", {}).get("enabled", True):
            p = ModelPlugin(
                plugins_cfg.get("model", {}), namespace, executor,
                variant=variant,
            )
            self._plugins.append(p)

        # Build tool map
        for plugin in self._plugins:
            if hasattr(plugin, "get_tools"):
                for tool in plugin.get_tools():
                    self._tool_map[tool["name"]] = plugin
            elif hasattr(plugin, "get_tool"):
                tool = plugin.get_tool()
                self._tool_map[tool["name"]] = plugin

    def start_all(self):
        for p in self._plugins:
            p.start()

    def stop_all(self):
        for p in self._plugins:
            p.stop()

    def get_all_tools(self) -> list:
        tools = []
        for plugin in self._plugins:
            if hasattr(plugin, "get_tools"):
                tools.extend(plugin.get_tools())
            elif hasattr(plugin, "get_tool"):
                tools.append(plugin.get_tool())
        return tools

    def dispatch(self, tool_name: str, args: dict) -> dict:
        plugin = self._tool_map.get(tool_name)
        if plugin is None:
            return {"error": f"Unknown tool: {tool_name}"}
        action = args.pop("action", tool_name)
        args["_tool_name"] = tool_name
        result = plugin.dispatch(action, args)
        if result is None:
            return {"error": f"Unknown action '{action}' for tool '{tool_name}'"}
        return result
