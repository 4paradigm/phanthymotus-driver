"""PNPbotics Adam driver — plugin classes.

Plugins:
  StatePlugin  — DDS rt/lowstate + rt/handstate → ROS2 skeleton/IMU/battery
  LocoPlugin   — gRPC locomotion control
  ArmPlugin    — ROS2 JointState upper body control
  HandPlugin   — DDS rt/handcmd finger control
  ModelPlugin  — URDF resource for 3D visualization
"""

import json
import threading
import time
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


# ===========================================================================
# ModelPlugin — URDF resource
# ===========================================================================

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
