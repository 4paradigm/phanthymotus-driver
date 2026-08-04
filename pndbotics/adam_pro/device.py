#!/usr/bin/env python3
"""Adam Pro plugins using PNDbotics' official pnd_adam ROS2 messages."""

from __future__ import annotations

import json
import math
import threading
import time

from common.vendor_runtime import action_schema, jsonable, tool


JOINT_NAMES = (
    "left_hip_pitch", "left_hip_roll", "left_hip_yaw", "left_knee", "left_ankle_pitch", "left_ankle_roll",
    "right_hip_pitch", "right_hip_roll", "right_hip_yaw", "right_knee", "right_ankle_pitch", "right_ankle_roll",
    "waist_yaw", "waist_roll", "waist_pitch",
    "left_shoulder_pitch", "left_shoulder_roll", "left_shoulder_yaw", "left_elbow", "left_wrist_roll", "left_wrist_pitch", "left_wrist_yaw",
    "right_shoulder_pitch", "right_shoulder_roll", "right_shoulder_yaw", "right_elbow", "right_wrist_roll", "right_wrist_pitch", "right_wrist_yaw",
    "head_yaw", "head_pitch",
)
JOINT_INDEX = {name: index for index, name in enumerate(JOINT_NAMES)}
KP = (305, 700, 405, 305, 30, 0, 305, 700, 405, 305, 30, 0, 205, 405, 405, 18, 9, 9, 9, 9, 9, 9, 18, 9, 9, 9, 9, 9, 9, 40, 40)
KD = (6.1, 30, 6.1, 6.1, 2.25, .25, 6.1, 30, 6.1, 6.1, 2.25, .25, 4.1, 6.1, 6.1, .9, .9, .9, .9, .9, .9, .9, .9, .9, .9, .9, .9, .9, .9, 1, 1)
GROUPS = {
    "legs": tuple(range(12)), "waist": (12, 13, 14), "left_arm": tuple(range(15, 22)),
    "right_arm": tuple(range(22, 29)), "arms": tuple(range(15, 29)), "head": (29, 30),
    "upper_body": tuple(range(12, 31)), "all": tuple(range(31)),
}


class AdamNodes:
    def __init__(self, config, namespace, ros2):
        from pnd_adam.msg import HandCmd, LowCmd, LowState
        from rclpy.node import Node
        from sensor_msgs.msg import Image
        from std_msgs.msg import String

        self.config = config
        self.robot = Node("adam_pro_driver_robot", context=ros2.ctx_robot)
        self.core = Node("adam_pro_driver_core", context=ros2.ctx_core)
        ros2.executor_robot.add_node(self.robot); ros2.executor_core.add_node(self.core)
        self.low_state = None; self.state_lock = threading.Lock()
        self.low_pub = self.robot.create_publisher(LowCmd, config["topics"]["low_command"], 10)
        self.hand_pub = self.robot.create_publisher(HandCmd, config["topics"]["hand_command"], 10)
        self.state_pub = self.core.create_publisher(String, f"/{namespace}/adam_pro/state", 10)
        self.robot.create_subscription(LowState, config["topics"]["low_state"], self._on_state, 10)
        self.camera_topic = f"/{namespace}/adam_pro/camera"
        self.camera_pub = self.core.create_publisher(Image, self.camera_topic, 2)
        self.robot.create_subscription(Image, config["topics"]["camera"], self.camera_pub.publish, 2)
        self.closed = False

    def _on_state(self, msg):
        from std_msgs.msg import String
        with self.state_lock: self.low_state = msg
        out = String(); out.data = json.dumps(self.snapshot(), ensure_ascii=False); self.state_pub.publish(out)

    def snapshot(self):
        with self.state_lock:
            msg = self.low_state
            if msg is None: return {"state": "waiting", "joints": []}
            motors = list(msg.motor_state)
            return {
                "state": "connected", "mode": int(msg.mode_pr), "tick": int(msg.tick),
                "imu": jsonable(msg.imu_state), "battery": jsonable(msg.battery_data),
                "wireless_remote": list(msg.wireless_remote),
                "joints": [
                    {"name": name, **jsonable(motors[index])}
                    for index, name in enumerate(JOINT_NAMES) if index < len(motors)
                ],
            }

    def positions(self):
        with self.state_lock:
            if self.low_state is None or len(self.low_state.motor_state) < 31: return None
            return [float(motor.q) for motor in self.low_state.motor_state[:31]]

    def close(self):
        if self.closed: return
        self.closed = True; self.robot.destroy_node(); self.core.destroy_node()


class AdamStatePlugin:
    def __init__(self, nodes, namespace): self.nodes = nodes; self.topic = f"/{namespace}/adam_pro/state"
    def get_tools(self):
        return [
            tool("state", "sensor", "Adam Pro 31-motor, IMU, wireless and battery state", topic_out=[{"topic": self.topic, "format": "data/json"}]),
            tool("camera", "sensor", "Adam Pro ZED camera ROS2 stream", topic_out=[{"topic": self.nodes.camera_topic, "format": "video/raw"}]),
            tool("joint_groups", "sensor", "Adam Pro named joint groups"),
            tool("capabilities", "sensor", "Adam Pro driver capability discovery"),
            tool("ros_graph", "sensor", "Live Adam Pro ROS2 graph"),
        ]
    def start(self): pass
    def stop(self): self.nodes.close()
    def dispatch(self, action, args):
        name = args.get("_tool_name")
        if action == "info":
            if name == "state": return {"state": "ready", "topic_out": [{"topic": self.topic, "format": "data/json"}]}
            if name == "camera": return {"state": "ready", "topic_out": [{"topic": self.nodes.camera_topic, "format": "video/raw"}]}
            return {"state": "ready"}
        if action == "start": return {"state": "running"}
        if action == "stop": return {"state": "idle"}
        if name == "state": return self.nodes.snapshot()
        if name == "camera": return {"state": "running", "topic": self.nodes.camera_topic}
        if name == "joint_groups": return {"names": list(JOINT_NAMES), "groups": {key: [JOINT_NAMES[i] for i in value] for key, value in GROUPS.items()}}
        if name == "capabilities": return {"model": "PNDbotics Adam Pro", "body_dof": 31, "hand_dof": 12, "control_hz": 400, "capabilities": ["low_level_joint", "hand", "trajectory", "choreography", "camera", "imu", "battery", "teleoperation_compatible", "mujoco_compatible"]}
        if name == "ros_graph": return {"topics": self.nodes.robot.get_topic_names_and_types(), "services": self.nodes.robot.get_service_names_and_types(), "nodes": self.nodes.robot.get_node_names_and_namespaces()}
        return None


class AdamLowLevelPlugin:
    def __init__(self, nodes, config):
        self.nodes = nodes; self.rate = float(config["control"].get("rate_hz", 400.0))
        self.cancel = threading.Event(); self.thread = None; self.status = {"state": "idle"}

    def get_tool(self):
        return tool("joints", "actuator", "Adam Pro 31-motor low-level position/velocity/torque control", action_schema(
            {
                "move": (["targets", "duration", "kp_scale", "kd_scale"], "Move named Adam Pro joints with interpolation"),
                "command": (["positions", "velocities", "torques", "kp", "kd", "mode"], "Publish one complete 31-motor command"),
                "hold": (["duration"], "Hold the current 31-motor pose"),
                "disable": (["group"], "Disable selected motor group"),
                "sequence": (["steps", "repeat"], "Execute an arbitrary multi-step full-body sequence"),
                "cancel": ([], "Cancel active Adam Pro trajectory or sequence"),
                "status": ([], "Get low-level control status"),
            },
            {
                "targets": {"type": "object", "additionalProperties": {"type": "number"}}, "duration": {"type": "number"},
                "kp_scale": {"type": "number"}, "kd_scale": {"type": "number"},
                "positions": {"type": "array", "items": {"type": "number"}, "minItems": 31, "maxItems": 31},
                "velocities": {"type": "array", "items": {"type": "number"}}, "torques": {"type": "array", "items": {"type": "number"}},
                "kp": {"type": "array", "items": {"type": "number"}}, "kd": {"type": "array", "items": {"type": "number"}}, "mode": {"type": "integer"},
                "group": {"type": "string", "enum": list(GROUPS)}, "steps": {"type": "array", "items": {"type": "object"}}, "repeat": {"type": "integer"},
            },
        ))
    def start(self): pass
    def stop(self): self.cancel.set()

    def _publish(self, positions, velocities=None, torques=None, kp=None, kd=None, mode=1):
        from pnd_adam.msg import LowCmd, MotorCmd
        if len(positions) != 31: raise ValueError("Adam Pro positions must contain 31 values")
        velocities = velocities or [0.0] * 31; torques = torques or [0.0] * 31; kp = kp or list(KP); kd = kd or list(KD)
        for label, values in (("velocities", velocities), ("torques", torques), ("kp", kp), ("kd", kd)):
            if len(values) != 31: raise ValueError(f"Adam Pro {label} must contain 31 values")
        msg = LowCmd(); msg.mode_pr = int(mode); msg.motor_cmd = []
        for i in range(31):
            motor = MotorCmd(); motor.mode = int(mode); motor.q = float(positions[i]); motor.dq = float(velocities[i]); motor.tau = float(torques[i]); motor.kp = float(kp[i]); motor.kd = float(kd[i]); motor.ki = 0.0
            msg.motor_cmd.append(motor)
        self.nodes.low_pub.publish(msg)

    def _move_blocking(self, targets, duration, kp_scale=1.0, kd_scale=1.0):
        start = self.nodes.positions()
        if start is None: raise RuntimeError("No Adam Pro lowstate received")
        end = list(start)
        for name, value in targets.items():
            if name not in JOINT_INDEX: raise ValueError(f"Unknown Adam Pro joint: {name}")
            end[JOINT_INDEX[name]] = float(value)
        began = time.monotonic(); dt = 1.0 / max(1.0, self.rate)
        while not self.cancel.is_set():
            ratio = min(1.0, (time.monotonic() - began) / max(0.01, duration))
            smooth = ratio * ratio * (3.0 - 2.0 * ratio)
            self._publish([a + (b - a) * smooth for a, b in zip(start, end)], kp=[v * kp_scale for v in KP], kd=[v * kd_scale for v in KD])
            self.status = {"state": "running", "progress": ratio, "targets": targets}
            if ratio >= 1.0: break
            time.sleep(dt)

    def _launch(self, runner):
        if self.thread and self.thread.is_alive(): raise RuntimeError("Adam Pro low-level controller is busy")
        self.cancel.clear()
        def guarded():
            try: runner(); self.status = {"state": "cancelled" if self.cancel.is_set() else "completed"}
            except Exception as exc: self.status = {"state": "failed", "error": str(exc)}
        self.thread = threading.Thread(target=guarded, daemon=True); self.thread.start()

    def dispatch(self, action, args):
        if action == "start": return {"state": "ready"}
        if action == "stop": self.cancel.set(); return {"state": "idle"}
        if action == "cancel": self.cancel.set(); return {"state": "cancelled"}
        if action == "status": return dict(self.status)
        if action == "command":
            self._publish(args["positions"], args.get("velocities"), args.get("torques"), args.get("kp"), args.get("kd"), args.get("mode", 1)); return {"state": "published"}
        if action == "disable":
            current = self.nodes.positions() or [0.0] * 31; indices = GROUPS[str(args.get("group", "all"))]
            from pnd_adam.msg import LowCmd, MotorCmd
            msg = LowCmd(); msg.motor_cmd = []
            for i in range(31):
                motor = MotorCmd(); motor.mode = 0 if i in indices else 1; motor.q = current[i]; motor.kp = 0.0 if i in indices else float(KP[i]); motor.kd = float(KD[i]); msg.motor_cmd.append(motor)
            self.nodes.low_pub.publish(msg); return {"state": "published", "disabled": [JOINT_NAMES[i] for i in indices]}
        if action in ("move", "hold"):
            targets = args.get("targets", {}) if action == "move" else {name: pos for name, pos in zip(JOINT_NAMES, self.nodes.positions() or [])}
            self._launch(lambda: self._move_blocking(targets, float(args.get("duration", 3.0)), float(args.get("kp_scale", 1.0)), float(args.get("kd_scale", 1.0))))
            return {"state": "running", "targets": targets}
        if action == "sequence":
            steps = list(args.get("steps", [])); repeat = max(1, int(args.get("repeat", 1)))
            def run():
                for _ in range(repeat):
                    for step in steps:
                        if self.cancel.is_set(): return
                        self._move_blocking(step.get("targets", {}), float(step.get("duration", 1.0)), float(step.get("kp_scale", 1.0)), float(step.get("kd_scale", 1.0)))
            self._launch(run); return {"state": "running", "steps": len(steps), "repeat": repeat}
        return None


class AdamHandPlugin:
    def __init__(self, nodes): self.nodes = nodes
    def get_tool(self):
        return tool("hand", "actuator", "Adam Pro dual 6-DOF dexterous-hand control", action_schema(
            {"open": (["side"], "Open Adam Pro hand"), "close": (["side", "strength"], "Close Adam Pro hand"), "set": (["positions"], "Set all 12 finger positions")},
            {"side": {"type": "string", "enum": ["left", "right", "both"]}, "strength": {"type": "number"}, "positions": {"type": "array", "items": {"type": "integer"}, "minItems": 12, "maxItems": 12}},
        ))
    def start(self): pass
    def stop(self): pass
    def _send(self, values):
        from pnd_adam.msg import HandCmd
        if len(values) != 12: raise ValueError("Adam Pro hand command must contain 12 positions")
        msg = HandCmd(); msg.position = [max(0, min(1000, int(v))) for v in values]; self.nodes.hand_pub.publish(msg)
        return {"state": "published", "positions": list(msg.position)}
    def dispatch(self, action, args):
        if action == "start": return {"state": "ready"}
        if action == "stop": return {"state": "idle"}
        if action == "set": return self._send(args["positions"])
        if action in ("open", "close"):
            side = str(args.get("side", "both")); strength = max(0.0, min(1.0, float(args.get("strength", 1.0))))
            values = [1000] * 12; target = 1000 if action == "open" else int(1000 * (1.0 - strength))
            spans = range(12) if side == "both" else (range(6) if side == "left" else range(6, 12))
            for index in spans: values[index] = target
            return self._send(values)
        return None


class AdamChoreographyPlugin:
    PRESETS = {
        "wave": [
            ({"right_shoulder_pitch": -0.4, "right_shoulder_roll": -1.1, "right_elbow": -1.2}, 1.2),
            ({"right_wrist_roll": 0.8}, .4), ({"right_wrist_roll": -0.8}, .4), ({"right_wrist_roll": 0.8}, .4),
        ],
        "bow": [({"waist_pitch": 0.45, "head_pitch": 0.25}, 1.0), ({"waist_pitch": 0.0, "head_pitch": 0.0}, 1.0)],
        "upper_body_dance": [
            ({"waist_yaw": .35, "left_shoulder_roll": 1.0, "right_shoulder_roll": -1.0}, .8),
            ({"waist_yaw": -.35, "left_shoulder_roll": .2, "right_shoulder_roll": -.2, "left_elbow": -1.0, "right_elbow": -1.0}, .8),
            ({"waist_yaw": 0.0, "left_shoulder_roll": 0.0, "right_shoulder_roll": 0.0, "left_elbow": 0.0, "right_elbow": 0.0}, .8),
        ],
    }
    def __init__(self, low): self.low = low
    def get_tool(self):
        return tool("choreography", "actuator", "Adam Pro gestures and full custom choreography", action_schema(
            {"list": ([], "List built-in Adam Pro gestures"), "play": (["name", "repeat"], "Play a built-in gesture"), "custom": (["steps", "repeat"], "Play custom full-body choreography"), "stop": ([], "Stop choreography")},
            {"name": {"type": "string", "enum": list(self.PRESETS)}, "steps": {"type": "array", "items": {"type": "object"}}, "repeat": {"type": "integer"}},
        ))
    def start(self): pass
    def stop(self): self.low.cancel.set()
    def dispatch(self, action, args):
        if action == "start": return {"state": "ready"}
        if action in ("stop",): self.low.cancel.set(); return {"state": "stopped"}
        if action == "list": return {"gestures": list(self.PRESETS)}
        if action == "play":
            steps = [{"targets": targets, "duration": duration} for targets, duration in self.PRESETS[str(args["name"])]]
            return self.low.dispatch("sequence", {"steps": steps, "repeat": args.get("repeat", 1)})
        if action == "custom": return self.low.dispatch("sequence", args)
        return None


def build_plugins(config, namespace, ros2):
    nodes = AdamNodes(config, namespace, ros2); low = AdamLowLevelPlugin(nodes, config)
    return [AdamStatePlugin(nodes, namespace), low, AdamHandPlugin(nodes), AdamChoreographyPlugin(low)]
