#!/usr/bin/env python3
"""
drivers/booster/k1/device.py — Booster K1 设备插件。

K1 只有一个 SDK 入口：boosteros.robots.booster.BoosterRobot（官方要求单例，见 main.py）。
每个插件构造时接收同一个 `robot` 实例，而不是像 unitree/g1 那样各自持有独立 SDK client。

`robot` 为 None 表示 main.py 未能连上真机/虚拟机（LocoClientInitError 或本环境本来就没有
K1 可连）——所有插件必须优雅降级：sensor 插件跳过订阅（topic 不发数据），actuator 插件在
dispatch 里对每个真实动作 action 返回 {"state": "error", "error": "robot not connected"}，
而不是抛异常把 MCP handler 打挂。

插件：
  StatePlugin    (sensor + resource) — imu/battery/joints(skeleton)/odom/fall_down_state + URDF
  CameraPlugin   (sensor)            — RGB 图像
  LocoPlugin     (actuator)          — 行走模式/步态/速度/头部朝向/里程计复位
  UpperBodyPlugin(actuator)          — 上身自定义关节控制（walk 模式下接管头+双臂）
  ActionPlugin   (actuator, ACP)     — 预定义动作 / 起身 / 轨迹回放，异步任务走 ACP
  AudioPlugin    (actuator + sensor) — 音频播放（ACP）、系统音量、录音
  SpeechPlugin   (processor, 可选)   — 语音识别 / 对话 / 音色列表 (boosteros[brain])
  DetectionPlugin(processor, 可选)   — 目标检测 (boosteros[brain])
"""

import json
import os as _os
import ssl as _ssl
import threading
import time
import urllib.request as _urllib
from pathlib import Path
from uuid import uuid4

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage

_LOW_LAT_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=200,
    durability=DurabilityPolicy.VOLATILE,
)


def _not_connected() -> dict:
    return {"state": "error", "error": "robot not connected"}


def _normalize_joint_key(name: str) -> str:
    return "".join(ch for ch in name.lower() if ch.isalnum())


def _load_urdf_joint_name_map() -> dict:
    """normalized-name -> exact URDF <joint name="..."> string.

    boosteros' get_joint_states() names (e.g. "AAHead_Yaw") and the
    booster_assets K1_22dof.urdf joint names (e.g. "aahead_yaw_joint") use
    different casing/suffix conventions from two independently generated
    artifacts — confirmed by spot-checking booster_assets against the SDK
    quick-start example, where they didn't even agree with each other on a
    couple of head joints. Match by a case/underscore/suffix-insensitive key
    instead of hardcoding a guessed table; unmapped joints fall back to their
    raw boosteros name and simply won't render until this is re-verified
    against a real K1.
    """
    urdf_path = Path(__file__).parent / "resource" / "k1_model.urdf"
    mapping: dict = {}
    try:
        import xml.etree.ElementTree as ET
        root = ET.parse(urdf_path).getroot()
        for joint in root.findall("joint"):
            name = joint.get("name", "")
            key = _normalize_joint_key(name.removesuffix("_joint"))
            mapping[key] = name
    except Exception:
        pass
    return mapping


def _acp_callback(action_id: str, status: str, result: dict, tool: str) -> None:
    """POST action completion to Agent Core. See README_dev.md § Action Completion Protocol."""
    agent_core_url = _os.environ.get("AGENT_CORE_URL", "https://localhost:15678")
    ctx = _ssl.create_default_context()
    ctx.check_hostname = False
    ctx.verify_mode = _ssl.CERT_NONE
    payload = json.dumps({
        "action_id": action_id,
        "status": status,       # "completed" | "error" | "cancelled"
        "result": result,
        "tool": tool,
        "ts": time.time(),
    }).encode()
    try:
        req = _urllib.Request(
            f"{agent_core_url}/api/acp/complete",
            data=payload,
            headers={"Content-Type": "application/json"},
            method="POST",
        )
        _urllib.urlopen(req, timeout=5, context=ctx)
    except Exception as e:
        import sys
        print(f"[ACP] callback failed for {action_id}: {e}", file=sys.stderr)


_TASK_STATUS_TO_ACP = {
    "SUCCEEDED": "completed",
    "FAILED": "error",
    "CANCELLED": "cancelled",
}


def _bind_task_acp(handle, tool: str) -> str:
    """Register handle.add_done_callback() -> ACP completion POST. Returns action_id (= trace_id)."""
    action_id = handle.trace_id

    def _on_done(h):
        status = _TASK_STATUS_TO_ACP.get(str(h.status), "error")
        result = {}
        if h.error is not None:
            result["error"] = str(h.error)
        _acp_callback(action_id, status, result, tool)

    handle.add_done_callback(_on_done)
    return action_id


# ── StatePlugin (sensor + resource) ──────────────────────────────────────────

class _K1StateNode(Node):
    """Bridges boosteros push/poll telemetry onto ROS2 topics for Agent Core's ros2_bridge."""

    _JOINTS_INTERVAL = 0.1  # 10 Hz poll (boosteros has no subscribe_joint_states)

    def __init__(self, robot, imu_topic, battery_topic, joints_topic, odom_topic, fall_topic):
        super().__init__("k1_state")
        self._robot = robot
        self._imu_pub     = self.create_publisher(String, imu_topic,     _LOW_LAT_QOS)
        self._battery_pub = self.create_publisher(String, battery_topic, _LOW_LAT_QOS)
        self._joints_pub  = self.create_publisher(String, joints_topic,  _LOW_LAT_QOS)
        self._odom_pub    = self.create_publisher(String, odom_topic,    _LOW_LAT_QOS)
        self._fall_pub    = self.create_publisher(String, fall_topic,    _LOW_LAT_QOS)

        self._lock = threading.Lock()
        self._last_imu: dict = {}
        self._last_battery: dict = {}
        self._last_fall: dict = {}
        self._last_imu_quat = [1.0, 0.0, 0.0, 0.0]
        self._subs = []
        self._urdf_joint_map = _load_urdf_joint_name_map()

        if robot is None:
            self.get_logger().warn("K1StateNode: robot not connected, topics will stay empty")
            return

        try:
            self._subs.append(robot.subscribe_imu(self._on_imu))
            self._subs.append(robot.subscribe_battery(self._on_battery))
            self._subs.append(robot.subscribe_odom(self._on_odom))
            self._subs.append(robot.subscribe_fall_down_state(self._on_fall))
        except Exception as e:
            self.get_logger().warn(f"K1StateNode: subscribe failed: {e}")

        # No subscribe_joint_states in boosteros — poll get_joint_states() instead.
        self.create_timer(self._JOINTS_INTERVAL, self._poll_joints)

    def _on_imu(self, imu) -> None:
        # boosteros IMUState.orientation is [x,y,z,w]; skeleton spec wants [w,x,y,z].
        quat = list(imu.orientation) if imu.orientation is not None else None
        imu_quat = [quat[3], quat[0], quat[1], quat[2]] if quat else [1.0, 0.0, 0.0, 0.0]
        data = {
            "linear_acceleration": list(imu.linear_acceleration),
            "angular_velocity": list(imu.angular_velocity),
            "rpy": list(imu.rpy) if imu.rpy is not None else None,
            "timestamp": imu.timestamp,
        }
        with self._lock:
            self._last_imu = data
            self._last_imu_quat = imu_quat
        out = String()
        out.data = json.dumps(data)
        self._imu_pub.publish(out)

    def _on_battery(self, battery) -> None:
        data = {
            "percentage": getattr(battery, "percentage", None),
            "voltage": getattr(battery, "voltage", None),
            "current": getattr(battery, "current", None),
            "charging": getattr(battery, "charging", None),
            "timestamp": getattr(battery, "timestamp", None),
        }
        with self._lock:
            self._last_battery = data
        out = String()
        out.data = json.dumps(data)
        self._battery_pub.publish(out)

    def _on_odom(self, odom) -> None:
        out = String()
        out.data = json.dumps({
            "position": list(getattr(odom, "position", [])),
            "orientation": list(getattr(odom, "orientation", [])),
            "linear_velocity": list(getattr(odom, "linear_velocity", [])),
            "angular_velocity": list(getattr(odom, "angular_velocity", [])),
            "timestamp": getattr(odom, "timestamp", None),
        })
        self._odom_pub.publish(out)

    def _on_fall(self, fall) -> None:
        data = {"fallen": getattr(fall, "fallen", None), "timestamp": getattr(fall, "timestamp", None)}
        with self._lock:
            self._last_fall = data
        out = String()
        out.data = json.dumps(data)
        self._fall_pub.publish(out)

    def _poll_joints(self) -> None:
        try:
            states = self._robot.get_joint_states()
        except Exception:
            return
        joints = [
            {"idx": i, "name": self._urdf_joint_map.get(_normalize_joint_key(j.name), j.name),
             "q": round(float(j.position), 4),
             "dq": round(float(j.velocity), 4) if j.velocity is not None else 0.0,
             "tau": round(float(j.effort), 3) if j.effort is not None else 0.0}
            for i, j in enumerate(states.joints)
        ]
        with self._lock:
            imu_quat = list(self._last_imu_quat)
        out = String()
        out.data = json.dumps({"joints": joints, "imu_quat": imu_quat})
        self._joints_pub.publish(out)

    def snapshot(self, key: str) -> dict:
        with self._lock:
            return dict({"imu": self._last_imu, "battery": self._last_battery,
                         "fall_down_state": self._last_fall}.get(key, {}))


class StatePlugin:
    PREFIX = "state"

    def __init__(self, plugin_config: dict, namespace: str, executor, robot):
        self._robot = robot
        self._imu_topic     = f"/{namespace}/state/imu"
        self._battery_topic = f"/{namespace}/state/battery"
        self._joints_topic  = f"/{namespace}/state/joints"
        self._odom_topic    = f"/{namespace}/state/odom"
        self._fall_topic    = f"/{namespace}/state/fall_down_state"
        self._node = _K1StateNode(robot, self._imu_topic, self._battery_topic,
                                   self._joints_topic, self._odom_topic, self._fall_topic)
        executor.add_node(self._node)

    def get_tools(self) -> list:
        return [self._imu_tool(), self._battery_tool(), self._joints_tool(),
                self._odom_tool(), self._fall_tool(), self._model_tool()]

    def _imu_tool(self) -> dict:
        return {
            "name": "imu", "type": "sensor", "multiInstance": False,
            "description": f"K1 IMU — linear acceleration, angular velocity, rpy. Publishes to {self._imu_topic}",
            "inputSchema": {"type": "object", "properties": {}},
            "topic_out": [{"topic": self._imu_topic, "format": "data/json"}],
        }

    def _battery_tool(self) -> dict:
        return {
            "name": "battery", "type": "sensor", "multiInstance": False,
            "description": f"K1 battery — percentage, voltage, current, charging state. Publishes to {self._battery_topic}",
            "inputSchema": {"type": "object", "properties": {}},
            "topic_out": [{"topic": self._battery_topic, "format": "data/json"}],
        }

    def _joints_tool(self) -> dict:
        return {
            "name": "joints", "type": "sensor", "multiInstance": False,
            "description": f"K1 joint states — 22 motors (neck + arms + legs), position(q)/velocity(dq)/effort(tau) at 10Hz. Publishes to {self._joints_topic}",
            "inputSchema": {"type": "object", "properties": {}},
            "topic_out": [{"topic": self._joints_topic, "format": "sensor/skeleton"}],
        }

    def _odom_tool(self) -> dict:
        return {
            "name": "odom", "type": "sensor", "multiInstance": False,
            "description": f"K1 odometry — position/orientation/velocity. Publishes to {self._odom_topic}",
            "inputSchema": {"type": "object", "properties": {}},
            "topic_out": [{"topic": self._odom_topic, "format": "data/json"}],
        }

    def _fall_tool(self) -> dict:
        return {
            "name": "fall_down_state", "type": "sensor", "multiInstance": False,
            "description": f"K1 fall-detection state. Publishes to {self._fall_topic}",
            "inputSchema": {"type": "object", "properties": {}},
            "topic_out": [{"topic": self._fall_topic, "format": "data/json"}],
        }

    def _model_tool(self) -> dict:
        return {
            "name": "model", "type": "resource", "multiInstance": False,
            "description": "K1 robot URDF model (22DOF) for 3D skeleton visualization",
            "inputSchema": {"type": "object", "properties": {}},
        }

    def start(self) -> None:
        pass  # subscriptions/timer started in __init__

    def stop(self) -> None:
        pass

    def dispatch(self, action: str, args: dict) -> dict | None:
        if action == "start":
            return {"state": "running"}
        if action == "stop":
            return {"state": "idle"}
        if action == "info":
            tool_name = args.get('_tool_name', '')
            topic_map = {
                'imu':             (self._imu_topic,     'data/json'),
                'battery':         (self._battery_topic, 'data/json'),
                'joints':          (self._joints_topic,  'sensor/skeleton'),
                'odom':            (self._odom_topic,    'data/json'),
                'fall_down_state': (self._fall_topic,    'data/json'),
            }
            if tool_name in topic_map:
                topic, fmt = topic_map[tool_name]
                return {"state": "running", "topic_out": [{"topic": topic, "format": fmt}]}
            return {"state": "running"}
        if action == "model":
            urdf_path = Path(__file__).parent / "resource" / "k1_model.urdf"
            if urdf_path.exists():
                return {"urdf": urdf_path.read_text()}
            return {"error": "URDF model file not found"}
        return None


# ── CameraPlugin (sensor) ─────────────────────────────────────────────────────

class _K1CameraNode(Node):
    def __init__(self, robot, color_topic: str):
        super().__init__("k1_camera")
        self._pub = self.create_publisher(CompressedImage, color_topic, _LOW_LAT_QOS)
        self._sub = None
        if robot is None:
            self.get_logger().warn("K1CameraNode: robot not connected, no image will publish")
            return
        try:
            self._sub = robot.subscribe_image(self._on_image, img_type="rgb")
        except Exception as e:
            self.get_logger().warn(f"K1CameraNode: subscribe_image failed: {e}")

    def _on_image(self, image) -> None:
        import cv2
        import numpy as np
        try:
            if hasattr(image, "format"):  # already CompressedImage (e.g. jpeg)
                jpeg_bytes = image.to_bytes()
            else:
                arr = image.to_numpy()
                ok, buf = cv2.imencode(".jpg", arr)
                if not ok:
                    return
                jpeg_bytes = buf.tobytes()
        except Exception:
            return
        msg = CompressedImage()
        msg.format = "jpeg"
        msg.data = jpeg_bytes
        self._pub.publish(msg)


class CameraPlugin:
    PREFIX = "camera"

    def __init__(self, plugin_config: dict, namespace: str, executor, robot):
        self._color_topic = f"/{namespace}/camera/rgb"
        self._node = _K1CameraNode(robot, self._color_topic)
        executor.add_node(self._node)

    def get_tools(self) -> list:
        return [self._color_tool()]

    def _color_tool(self) -> dict:
        return {
            "name": "camera", "type": "sensor", "multiInstance": False,
            "description": f"K1 head camera — JPEG frames. Publishes to {self._color_topic}",
            "inputSchema": {"type": "object", "properties": {}},
            "topic_out": [{"topic": self._color_topic, "format": "image/jpeg"}],
        }

    def start(self) -> None:
        pass

    def stop(self) -> None:
        pass

    def dispatch(self, action: str, args: dict) -> dict | None:
        if action == "start":
            return {"state": "running"}
        if action == "stop":
            return {"state": "idle"}
        if action == "info":
            return {"state": "running", "topic_out": [{"topic": self._color_topic, "format": "image/jpeg"}]}
        return None


# ── LocoPlugin (actuator) ─────────────────────────────────────────────────────

class LocoPlugin:
    PREFIX = "loco"

    def __init__(self, plugin_config: dict, namespace: str, executor, robot):
        self._robot = robot

    def get_tool(self) -> dict:
        return {
            "name": "loco", "type": "actuator", "multiInstance": False,
            "description": "K1 locomotion — mode/gait switching, planar velocity, head orientation, odometry reset",
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {
                        "type": "string",
                        "enum": ["move", "stop", "set_mode", "set_gait", "get_mode",
                                  "list_gaits", "set_head_angle", "reset_odom"],
                        "description": "Action to perform",
                    },
                    "vx":    {"type": "number", "description": "Forward velocity m/s, positive = forward"},
                    "vy":    {"type": "number", "description": "Lateral velocity m/s, positive = left"},
                    "vyaw":  {"type": "number", "description": "Yaw rate rad/s, positive = counter-clockwise"},
                    "mode":  {"type": "string", "description": "Target mode, e.g. 'walk', 'prepare', 'custom'"},
                    "gait":  {"type": "string", "description": "Target gait name (see list_gaits)"},
                    "pitch": {"type": "number", "description": "Head pitch rad, positive = down"},
                    "yaw":   {"type": "number", "description": "Head yaw rad, positive = left"},
                },
                "required": ["action"],
                "x-action-params": {
                    "move":            {"params": ["vx", "vy", "vyaw"], "description": "Move with planar velocities (robot must be in 'walk' mode)"},
                    "stop":            {"params": [],                  "description": "Zero all planar velocity"},
                    "set_mode":        {"params": ["mode"],            "description": "Switch robot mode (walk / prepare / custom / ...)"},
                    "set_gait":        {"params": ["gait"],            "description": "Switch gait"},
                    "get_mode":        {"params": [],                  "description": "Get current mode"},
                    "list_gaits":      {"params": [],                  "description": "List supported gaits"},
                    "set_head_angle":  {"params": ["pitch", "yaw"],    "description": "Set head pitch/yaw (robot must be in 'walk' mode)"},
                    "reset_odom":      {"params": [],                  "description": "Reset odometry to origin"},
                },
            },
        }

    def start(self) -> None:
        pass

    def stop(self) -> None:
        if self._robot is not None:
            try:
                self._robot.set_velocity(0.0, 0.0, 0.0)
            except Exception:
                pass

    def dispatch(self, action: str, args: dict) -> dict | None:
        if action == "start":
            return {"state": "ready"}
        if action == "stop":
            return {"state": "idle"}
        if self._robot is None:
            return _not_connected()
        try:
            if action == "move":
                vx, vy, vyaw = float(args.get("vx", 0.0)), float(args.get("vy", 0.0)), float(args.get("vyaw", 0.0))
                self._robot.set_velocity(vx, vy, vyaw)
                return {"state": "moving", "vx": vx, "vy": vy, "vyaw": vyaw}
            if action == "stop":
                self._robot.set_velocity(0.0, 0.0, 0.0)
                return {"state": "stopped"}
            if action == "set_mode":
                mode = args.get("mode", "")
                self._robot.set_mode(mode)
                return {"state": "ok", "mode": mode}
            if action == "set_gait":
                gait = args.get("gait", "")
                self._robot.set_gait(gait)
                return {"state": "ok", "gait": gait}
            if action == "get_mode":
                return {"mode": self._robot.get_mode()}
            if action == "list_gaits":
                return {"gaits": list(self._robot.list_gaits())}
            if action == "set_head_angle":
                pitch, yaw = float(args.get("pitch", 0.0)), float(args.get("yaw", 0.0))
                self._robot.set_head_angle(pitch=pitch, yaw=yaw)
                return {"state": "ok", "pitch": pitch, "yaw": yaw}
            if action == "reset_odom":
                self._robot.reset_odom()
                return {"state": "ok"}
        except Exception as e:
            return {"state": "error", "error": str(e)}
        return None


# ── UpperBodyPlugin (actuator) ────────────────────────────────────────────────

class UpperBodyPlugin:
    PREFIX = "upper_body"

    def __init__(self, plugin_config: dict, namespace: str, executor, robot):
        self._robot = robot
        self._enabled = False

    def get_tool(self) -> dict:
        return {
            "name": "upper_body", "type": "actuator", "multiInstance": False,
            "description": (
                "K1 upper-body custom control — in 'walk' mode, hand control of head + both "
                "arms (first 10 joints) to the caller while legs keep walking under the system"
            ),
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {
                        "type": "string",
                        "enum": ["enable", "disable", "set_joints"],
                        "description": "Action to perform",
                    },
                    "joints": {
                        "type": "array",
                        "description": "Joint commands: [{name, position, kp, kd}], names from state/joints",
                        "items": {
                            "type": "object",
                            "properties": {
                                "name": {"type": "string"},
                                "position": {"type": "number", "description": "Target position, rad"},
                                "kp": {"type": "number", "description": "Position gain"},
                                "kd": {"type": "number", "description": "Velocity gain"},
                            },
                            "required": ["name", "position"],
                        },
                    },
                },
                "required": ["action"],
                "x-action-params": {
                    "enable":     {"params": [],         "description": "Take over head + arm control (must be in 'walk' mode)"},
                    "disable":    {"params": [],         "description": "Return upper-body control to the system"},
                    "set_joints": {"params": ["joints"], "description": "Send target position/kp/kd for the upper-body joints"},
                },
            },
        }

    def start(self) -> None:
        pass

    def stop(self) -> None:
        if self._robot is not None and self._enabled:
            try:
                self._robot.upper_body_control(False)
            except Exception:
                pass
        self._enabled = False

    def dispatch(self, action: str, args: dict) -> dict | None:
        if action == "start":
            return {"state": "ready"}
        if action == "stop":
            self.stop()
            return {"state": "idle"}
        if self._robot is None:
            return _not_connected()
        try:
            if action == "enable":
                self._robot.upper_body_control(True)
                self._enabled = True
                return {"state": "ok", "upper_body_control": True}
            if action == "disable":
                self._robot.upper_body_control(False)
                self._enabled = False
                return {"state": "ok", "upper_body_control": False}
            if action == "set_joints":
                from boosteros.types import JointCommand
                commands = [
                    JointCommand(name=j["name"], position=float(j["position"]),
                                 kp=float(j.get("kp", 20.0)), kd=float(j.get("kd", 1.0)))
                    for j in args.get("joints", [])
                ]
                self._robot.set_joints(commands)
                return {"state": "ok", "count": len(commands)}
        except Exception as e:
            return {"state": "error", "error": str(e)}
        return None


# ── ActionPlugin (actuator, ACP) ──────────────────────────────────────────────

class ActionPlugin:
    PREFIX = "action"

    def __init__(self, plugin_config: dict, namespace: str, executor, robot):
        self._robot = robot
        self._handles: dict = {}  # trace_id -> TaskHandle

    def get_tool(self) -> dict:
        return {
            "name": "action", "type": "actuator", "multiInstance": False,
            "description": "K1 predefined actions, get-up, and trajectory playback — long-running, tracked via ACP",
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {
                        "type": "string",
                        "enum": ["list_actions", "do_action", "get_up", "execute_trajectory", "cancel"],
                        "description": "Action to perform",
                    },
                    "action_id":       {"type": "string", "description": "Predefined action ID from list_actions"},
                    "trajectory_path": {"type": "string", "description": "Path to a .btraj trajectory file, already present on the robot's filesystem"},
                    "task_trace_id":   {"type": "string", "description": "trace_id of a running task to cancel"},
                },
                "required": ["action"],
                "x-action-params": {
                    "list_actions":       {"params": [],                  "description": "List predefined action IDs supported by this robot"},
                    "do_action":          {"params": ["action_id"],       "description": "Asynchronously perform a predefined action"},
                    "get_up":             {"params": [],                  "description": "Asynchronously trigger the get-up routine"},
                    "execute_trajectory": {"params": ["trajectory_path"], "description": "Asynchronously replay a recorded joint trajectory (.btraj)"},
                    "cancel":             {"params": ["task_trace_id"],   "description": "Cancel a running action/get-up/trajectory task"},
                },
                "x-completion": {
                    "actions": ["do_action", "get_up", "execute_trajectory"],
                    "timeout": 60,
                },
            },
        }

    def start(self) -> None:
        pass

    def stop(self) -> None:
        for handle in list(self._handles.values()):
            try:
                handle.cancel()
            except Exception:
                pass

    def dispatch(self, action: str, args: dict) -> dict | None:
        if action == "start":
            return {"state": "ready"}
        if action == "stop":
            self.stop()
            return {"state": "idle"}
        if self._robot is None:
            return _not_connected()
        try:
            if action == "list_actions":
                return {"actions": [a.id if hasattr(a, "id") else str(a) for a in self._robot.list_actions()]}
            if action == "do_action":
                handle = self._robot.do_action(args["action_id"])
                action_id = _bind_task_acp(handle, self.PREFIX)
                self._handles[action_id] = handle
                return {"state": "running", "action_id": action_id}
            if action == "get_up":
                handle = self._robot.get_up()
                action_id = _bind_task_acp(handle, self.PREFIX)
                self._handles[action_id] = handle
                return {"state": "running", "action_id": action_id}
            if action == "execute_trajectory":
                from boosteros.types import TrajectoryData
                trajectory = TrajectoryData.load(args["trajectory_path"])
                handle = self._robot.execute_trajectory(trajectory)
                action_id = _bind_task_acp(handle, self.PREFIX)
                self._handles[action_id] = handle
                return {"state": "running", "action_id": action_id}
            if action == "cancel":
                handle = self._handles.get(args.get("task_trace_id", ""))
                if handle is None:
                    return {"state": "error", "error": "unknown task_trace_id"}
                return {"state": "ok", "cancelled": bool(handle.cancel())}
        except Exception as e:
            return {"state": "error", "error": str(e)}
        return None


# ── AudioPlugin (actuator + sensor) ───────────────────────────────────────────

class AudioPlugin:
    PREFIX = "audio"

    def __init__(self, plugin_config: dict, namespace: str, executor, robot):
        self._robot = robot
        self._handles: dict = {}

    def get_tool(self) -> dict:
        return {
            "name": "audio", "type": "actuator", "multiInstance": False,
            "description": "K1 audio — play a local sound file, system volume, mic recording",
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {
                        "type": "string",
                        "enum": ["play_sound", "get_volume", "set_volume",
                                  "start_recording", "stop_recording", "is_recording"],
                        "description": "Action to perform",
                    },
                    "audio_path": {"type": "string", "description": "Path to a .wav/.mp3/.pcm file already on the robot's filesystem"},
                    "volume":     {"type": "number", "description": "Volume 0.0-1.0"},
                },
                "required": ["action"],
                "x-action-params": {
                    "play_sound":      {"params": ["audio_path", "volume"], "description": "Asynchronously play a local audio file"},
                    "get_volume":      {"params": [],                       "description": "Get system output volume"},
                    "set_volume":      {"params": ["volume"],               "description": "Set system output volume (0.0-1.0)"},
                    "start_recording": {"params": [],                       "description": "Start microphone recording"},
                    "stop_recording":  {"params": [],                       "description": "Stop recording and return the captured audio"},
                    "is_recording":    {"params": [],                       "description": "Whether a recording is currently in progress"},
                },
                "x-completion": {
                    "actions": ["play_sound"],
                    "timeout": 60,
                },
            },
        }

    def start(self) -> None:
        pass

    def stop(self) -> None:
        for handle in list(self._handles.values()):
            try:
                handle.cancel()
            except Exception:
                pass

    def dispatch(self, action: str, args: dict) -> dict | None:
        if action == "start":
            return {"state": "ready"}
        if action == "stop":
            return {"state": "idle"}
        if self._robot is None:
            return _not_connected()
        try:
            if action == "play_sound":
                volume = args.get("volume")
                handle = self._robot.play_sound(args["audio_path"], volume=float(volume) if volume is not None else None)
                action_id = _bind_task_acp(handle, self.PREFIX)
                self._handles[action_id] = handle
                return {"state": "playing", "action_id": action_id}
            if action == "get_volume":
                return {"volume": self._robot.audio_manager.get_system_volume()}
            if action == "set_volume":
                volume = float(args.get("volume", 0.5))
                self._robot.audio_manager.set_system_volume(volume)
                return {"state": "ok", "volume": volume}
            if action == "start_recording":
                self._robot.audio_manager.start_recording()
                return {"state": "recording"}
            if action == "stop_recording":
                audio = self._robot.audio_manager.stop_recording()
                duration = getattr(getattr(audio, "duration", None), "seconds", None)
                return {"state": "ok", "duration_s": duration}
            if action == "is_recording":
                return {"recording": bool(self._robot.audio_manager.is_recording())}
        except Exception as e:
            return {"state": "error", "error": str(e)}
        return None


# ── SpeechPlugin (processor, optional — requires boosteros[brain]) ───────────

class SpeechPlugin:
    PREFIX = "speech"

    def __init__(self, plugin_config: dict, namespace: str, executor, robot):
        self._robot = robot

    def get_tool(self) -> dict:
        return {
            "name": "speech", "type": "processor", "multiInstance": False,
            "description": "K1 brain speech — chat and available voices (requires boosteros[brain])",
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {"type": "string", "enum": ["chat", "list_voices"], "description": "Action to perform"},
                    "text":   {"type": "string", "description": "User text to send to the chat interface"},
                },
                "required": ["action"],
                "x-action-params": {
                    "chat":        {"params": ["text"], "description": "Send text to the robot's brain chat interface"},
                    "list_voices": {"params": [],       "description": "List available TTS voices"},
                },
            },
        }

    def start(self) -> None:
        pass

    def stop(self) -> None:
        pass

    def dispatch(self, action: str, args: dict) -> dict | None:
        if action == "start":
            return {"state": "ready"}
        if action == "stop":
            return {"state": "idle"}
        if self._robot is None:
            return _not_connected()
        try:
            if action == "chat":
                return {"reply": self._robot.speech.chat(args.get("text", ""))}
            if action == "list_voices":
                return {"voices": list(self._robot.speech.list_voices())}
        except Exception as e:
            return {"state": "error", "error": str(e)}
        return None


# ── DetectionPlugin (processor, optional — requires boosteros[brain]) ────────

class DetectionPlugin:
    PREFIX = "detection"

    def __init__(self, plugin_config: dict, namespace: str, executor, robot):
        self._robot = robot

    def get_tool(self) -> dict:
        return {
            "name": "detection", "type": "processor", "multiInstance": False,
            "description": "K1 brain vision detection (requires boosteros[brain])",
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {"type": "string", "enum": ["list_models", "load_model", "detect"], "description": "Action to perform"},
                    "model":  {"type": "string", "description": "Model name from list_models"},
                },
                "required": ["action"],
                "x-action-params": {
                    "list_models": {"params": [],        "description": "List available detection models"},
                    "load_model":  {"params": ["model"], "description": "Load a detection model by name"},
                    "detect":      {"params": [],        "description": "Run detection on the current camera frame"},
                },
            },
        }

    def start(self) -> None:
        pass

    def stop(self) -> None:
        pass

    def dispatch(self, action: str, args: dict) -> dict | None:
        if action == "start":
            return {"state": "ready"}
        if action == "stop":
            return {"state": "idle"}
        if self._robot is None:
            return _not_connected()
        try:
            if action == "list_models":
                return {"models": list(self._robot.detection.list_models())}
            if action == "load_model":
                self._robot.detection.load_model(args["model"])
                return {"state": "ok", "model": args["model"]}
            if action == "detect":
                image = self._robot.get_image(img_type="rgb")
                result = self._robot.detection.detect(image)
                return {"detections": [d.__dict__ if hasattr(d, "__dict__") else str(d) for d in result]}
        except Exception as e:
            return {"state": "error", "error": str(e)}
        return None
