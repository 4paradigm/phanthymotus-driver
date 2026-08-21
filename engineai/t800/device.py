#!/usr/bin/env python3
"""EngineAI T800 ROS2 and Native SDK plugins.

Robot-facing traffic uses ROS domain 69.  Normalized dashboard streams are
republished on ROS domain 42.  All control interfaces exposed by EngineAI's
community protocol are represented, including the high-rate joint paths.
"""

from __future__ import annotations

import json
import math
import numbers
import os
import queue
import re
import sqlite3
import subprocess
import threading
import time
from collections import deque
from dataclasses import asdict
from pathlib import Path
from uuid import uuid4

import numpy as np
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

# Open3D is optional; fall back to numpy-only PCD writing when unavailable.
try:
    import open3d as o3d

    _HAS_OPEN3D = True
except ImportError:
    _HAS_OPEN3D = False

from control import (
    LED_MODES,
    MOTION_STATES,
    WALK_MOTION_STATES,
    T800_JOINT_GROUPS,
    T800_JOINT_INDEX,
    T800_JOINT_NAMES,
    RepeatingCommand,
    action_schema,
    array_property,
    clamp,
    float_list,
    joint_payload,
    list_or_default,
    optional_floats,
    resample_joint_trajectory,
    sensor_action_schema,
    sensor_tool,
    validate_joint_indices,
    validate_joint_positions,
    validate_parallel_arrays,
)
from native_sdk import NativeSdkManager


_BEST_EFFORT = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=3,
    durability=DurabilityPolicy.VOLATILE,
)
_RELIABLE = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    durability=DurabilityPolicy.VOLATILE,
)
_RELIABLE_ONE = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    durability=DurabilityPolicy.VOLATILE,
)
_AUDIO_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=200,
    durability=DurabilityPolicy.VOLATILE,
)


def _t800_acp_notify(action_id: str, status: str, result: dict, tool: str) -> None:
    """Post asynchronous actuator completion to Agent Core."""
    import ssl
    import urllib.request

    agent_core_url = os.environ.get("AGENT_CORE_URL", "https://localhost:15678")
    context = ssl.create_default_context()
    context.check_hostname = False
    context.verify_mode = ssl.CERT_NONE
    payload = json.dumps({
        "action_id": action_id,
        "status": status,
        "result": result,
        "tool": tool,
        "ts": time.time(),
    }).encode()
    try:
        request = urllib.request.Request(
            f"{agent_core_url}/api/acp/complete",
            data=payload,
            headers={"Content-Type": "application/json"},
            method="POST",
        )
        urllib.request.urlopen(request, timeout=5, context=context)
    except Exception as exc:
        print(f"[motion_recorder] ACP notify failed for {action_id}: {exc}", flush=True)

_LIFECYCLE_ACTIONS = {
    "start": ([], "启动卡片数据流"),
    "info": ([], "返回卡片状态和实际输出 topic"),
    "stop": ([], "停止卡片数据流"),
}


def _with_lifecycle(actions: dict[str, tuple[list[str], str]]) -> dict[str, tuple[list[str], str]]:
    return {**_LIFECYCLE_ACTIONS, **actions}


def _with_topic_out(payload: dict, topic: str, fmt: str = "data/json") -> dict:
    return {**payload, "topic_out": [{"topic": topic, "format": fmt}]}


def _now_ms() -> int:
    return int(time.time() * 1000)


_GAMEPAD_BUTTON_NAMES = {
    0: "LB", 1: "RB", 2: "A", 3: "B", 4: "X", 5: "Y",
    6: "BACK", 7: "START", 8: "CROSS_X_UP", 9: "CROSS_X_DOWN",
    10: "CROSS_Y_LEFT", 11: "CROSS_Y_RIGHT",
}

_GAMEPAD_ACTIONS = {
    frozenset({"LB", "START"}): "idle",
    frozenset({"LB", "RB"}): "passive",
    frozenset({"LB", "A"}): "stand",
    frozenset({"LB", "B"}): "walk",
    frozenset({"RB", "B"}): "dance",
    frozenset({"START", "CROSS_X_UP"}): "get_up",
    frozenset({"START", "CROSS_X_DOWN"}): "lie_down",
}


def _pressed_gamepad_buttons(digital_states) -> list[str]:
    states = [] if digital_states is None else digital_states
    return [
        _GAMEPAD_BUTTON_NAMES.get(index, str(index))
        for index, value in enumerate(states)
        if int(value) != 0
    ]


def _gamepad_control_source(msg) -> str:
    # Movement speed is derived from the GamepadKeys analog stream. On T800 this
    # can be produced by the physical joystick or a virtual/software sender; the
    # card exposes hardware_connected separately instead of guessing UI origin.
    return "gamepad_analog"


def _motion_direction(stick_x: float, stick_y: float, yaw_x: float = 0.0) -> str:
    parts: list[str] = []
    if abs(stick_y) >= 0.10:
        parts.append("forward" if stick_y > 0 else "backward")
    if abs(stick_x) >= 0.10:
        parts.append("right" if stick_x > 0 else "left")
    if not parts and abs(yaw_x) >= 0.10:
        parts.append("turn_right" if yaw_x > 0 else "turn_left")
    return "_".join(parts) if parts else "none"


def _normalize_motion_action(name: str) -> str:
    value = str(name or "").strip()
    lowered = value.lower()
    if not lowered or lowered == "unknown":
        return "unknown"
    aliases = (
        ("stand", ("stand", "pd_stand", "stance")),
        ("sit", ("sit", "sitting", "seated", "seat", "squat")),
        ("punch", ("punch", "boxing", "box", "fight", "fist", "打拳")),
        ("dance", ("dance",)),
        ("walk", ("walk", "loco")),
        ("get_up", ("get_up", "getup")),
        ("lie_down", ("lie_down", "liedown", "supine")),
        ("idle", ("idle",)),
        ("passive", ("passive", "damping")),
    )
    for normalized, tokens in aliases:
        if any(token in lowered for token in tokens):
            return normalized
    return value


def _json_message(payload: dict) -> String:
    def scalar_default(value):
        item = getattr(value, "item", None)
        if callable(item):
            return item()
        if isinstance(value, numbers.Integral):
            return int(value)
        if isinstance(value, numbers.Real):
            return float(value)
        if hasattr(value, "__float__"):
            return float(value)
        if hasattr(value, "__int__"):
            return int(value)
        raise TypeError(f"Object of type {type(value).__name__} is not JSON serializable")

    msg = String()
    msg.data = json.dumps(
        payload,
        separators=(",", ":"),
        ensure_ascii=False,
        default=scalar_default,
    )
    return msg


def _graph_items(node, method: str) -> list:
    callback = getattr(node, method, None)
    if callback is None:
        return []
    try:
        return list(callback())
    except Exception:
        return []


class StatePlugin:
    """Bridge all public T800 feedback interfaces to Agent Core."""

    _STREAMS = {
        "joints": ("state/joints", "sensor/skeleton", "T800 25 关节位置、速度和力矩"),
        "imu": ("state/imu", "data/json", "T800 IMU 姿态、欧拉角、角速度和线加速度"),
        "battery": ("state/battery", "data/json", "T800 电源使能、电量、电压、电流和错误码"),
        "motor_health": ("state/motor_health", "data/json", "T800 电机温度、电压、电流、掉线与错误码"),
        "motor_state": ("state/motor_state", "data/json", "T800 Native SDK 原始电机位置、速度和力矩"),
        "motor_command": ("state/motor_command", "data/json", "T800 Native SDK 原始电机控制命令"),
        "joint_command_feedback": ("state/joint_command_feedback", "data/json", "T800 Native SDK 最近关节控制命令反馈"),
        "gamepad": ("state/gamepad", "data/json", "T800 遥控器连接、按键和摇杆状态"),
        "motion_state": ("state/motion", "data/json", "T800 当前运动状态和允许转换状态"),
        "driver_health": ("state/driver_health", "data/json", "T800 driver 各数据源连接与新鲜度"),
    }
    _DERIVED_STREAMS = {
        "robot_snapshot": ("state/robot_snapshot", "T800 运动、关节、IMU、电源和电机状态聚合快照"),
        "fault_summary": ("state/fault_summary", "T800 电机、电源、温度和通信故障摘要"),
        "stability": ("state/stability", "基于 IMU 的机身倾斜、角速度和跌倒风险估计"),
        "joint_groups": ("model/joint_groups", "T800 腿、躯干、双臂和头部关节分组"),
        "capabilities": ("model/capabilities", "T800 Driver 原生接口、高阶动作和限制说明"),
        "ros_graph": ("state/ros_graph", "实时发现 T800 ROS2 节点、topic、service 和固件扩展接口"),
        "mainboard": ("state/mainboard", "展示 T800 主控板关联的温度、电源与电机诊断摘要"),
    }
    _MAINBOARD_KEYWORDS = ("mainboard", "main_board", "board", "thermal", "temperature", "fan", "diagnostic")
    _MAINBOARD_STRONG_KEYWORDS = ("mainboard", "main_board", "board")

    def __init__(self, config: dict, namespace: str, ros2, motion_events=None):
        self._config = config
        self._ns = namespace
        self._ros2 = ros2
        self._topics = config["topics"]
        self._motion_events = motion_events
        self._timeout = float(config["ros"].get("source_timeout_sec", 1.0))
        self._running = False
        self._lock = threading.RLock()
        self._cache: dict[str, dict] = {}
        self._updated: dict[str, float] = {}
        self._last_joint_positions = [0.0] * len(T800_JOINT_NAMES)
        self._current_motion = ""
        self._available_motions: list[str] = []

        self._sub_node = Node("t800_state_sub", context=ros2.ctx_robot)
        self._pub_node = Node("t800_state_pub", context=ros2.ctx_core)
        ros2.executor_robot.add_node(self._sub_node)
        ros2.executor_core.add_node(self._pub_node)
        self._publishers = {
            name: self._pub_node.create_publisher(
                String, f"/{namespace}/{relative_topic}", _BEST_EFFORT
            )
            for name, (relative_topic, _, _) in self._STREAMS.items()
        }
        self._derived_publishers = {
            name: self._pub_node.create_publisher(String, f"/{namespace}/{relative_topic}", _BEST_EFFORT)
            for name, (relative_topic, _) in self._DERIVED_STREAMS.items()
        }
        self._urdf_path = Path(__file__).parent / "resource" / "serial_t800.urdf"

    def get_tools(self) -> list[dict]:
        tools = [
            sensor_tool(name, description, f"/{self._ns}/{relative}", fmt)
            for name, (relative, fmt, description) in self._STREAMS.items()
        ]
        tools.append(
            {
                "name": "model",
                "type": "resource",
                "description": "EngineAI T800 25DOF URDF 骨架模型",
                "inputSchema": {"type": "object", "properties": {}},
            }
        )
        tools.extend(
            sensor_tool(name, description, f"/{self._ns}/{relative}", "data/json")
            for name, (relative, description) in self._DERIVED_STREAMS.items()
        )
        return tools

    def start(self) -> None:
        if self._running:
            return
        from interface_protocol.msg import (
            GamepadKeys,
            ImuInfo,
            JointCommand,
            JointState,
            MotionState,
            MotorDebug,
            PowerInfo,
        )

        self._running = True
        self._sub_node.create_subscription(
            JointState, self._topics["joint_state"], self._on_joints, _BEST_EFFORT
        )
        self._sub_node.create_subscription(
            ImuInfo, self._topics["imu"], self._on_imu, _BEST_EFFORT
        )
        self._sub_node.create_subscription(
            GamepadKeys, self._topics["gamepad"], self._on_gamepad, _BEST_EFFORT
        )
        self._sub_node.create_subscription(
            MotorDebug, self._topics["motor_debug"], self._on_motor_debug, _BEST_EFFORT
        )
        self._sub_node.create_subscription(
            JointState, self._topics["motor_state"], self._on_motor_state, _BEST_EFFORT
        )
        self._sub_node.create_subscription(
            JointCommand, self._topics["motor_command"], self._on_motor_command, _BEST_EFFORT
        )
        self._sub_node.create_subscription(
            JointCommand, self._topics["joint_command_feedback"], self._on_joint_command_feedback, _BEST_EFFORT
        )
        self._sub_node.create_subscription(
            PowerInfo, self._topics["power"], self._on_power, _BEST_EFFORT
        )
        self._sub_node.create_subscription(
            MotionState, self._topics["motion_state"], self._on_motion, _BEST_EFFORT
        )
        self._timer = self._pub_node.create_timer(0.05, self._publish_tick)

    def stop(self) -> None:
        self._running = False

    def current_motion(self) -> tuple[str, list[str]]:
        with self._lock:
            return self._current_motion, list(self._available_motions)

    def joint_positions(self) -> list[float]:
        with self._lock:
            return list(self._last_joint_positions)

    def dispatch(self, action_or_tool: str, args: dict) -> dict:
        if action_or_tool == "model":
            try:
                return {"urdf": self._urdf_path.read_text(encoding="utf-8")}
            except FileNotFoundError:
                return {"error": "T800 URDF not found"}
        if action_or_tool in self._DERIVED_STREAMS:
            return self._derived_snapshot(action_or_tool)
        if action_or_tool in self._STREAMS:
            return self._snapshot(action_or_tool)
        if action_or_tool == "status":
            name = args.get("_tool_name", "driver_health")
            if name in self._DERIVED_STREAMS:
                return self._derived_snapshot(name)
            if name in self._STREAMS:
                return self._snapshot(name)
        if action_or_tool == "start":
            return {"state": "running"}
        if action_or_tool == "stop":
            return {"state": "idle"}
        if action_or_tool == "info":
            name = args.get("_tool_name", "driver_health")
            if name in self._STREAMS:
                relative, fmt, _ = self._STREAMS[name]
                return _with_topic_out({"state": "running"}, f"/{self._ns}/{relative}", fmt)
            if name in self._DERIVED_STREAMS:
                relative, _ = self._DERIVED_STREAMS[name]
                return _with_topic_out({"state": "running"}, f"/{self._ns}/{relative}")
            return {"state": "running"}
        return {"error": f"unknown state action: {action_or_tool}"}

    def _derived_snapshot(self, name: str) -> dict:
        if name == "robot_snapshot":
            return {
                "motion": self._snapshot("motion_state"),
                "joints": self._snapshot("joints"),
                "imu": self._snapshot("imu"),
                "battery": self._snapshot("battery"),
                "motor_health": self._snapshot("motor_health"),
                "timestamp_ms": _now_ms(),
            }
        if name == "fault_summary":
            motor = self._snapshot("motor_health")
            battery = self._snapshot("battery")
            offline = [index for index, value in enumerate(motor.get("offline", [])) if value]
            disabled = [index for index, value in enumerate(motor.get("enabled", [])) if not value]
            motor_errors = [
                {"joint_index": index, "code": int(code)}
                for index, code in enumerate(motor.get("error_code", []))
                if int(code) != 0
            ]
            temperatures = list(motor.get("motor_temperature_c", []))
            hot = [
                {"joint_index": index, "temperature_c": float(value)}
                for index, value in enumerate(temperatures)
                if float(value) >= float(self._config["control"].get("motor_warning_temperature_c", 70.0))
            ]
            power_error = int(battery.get("error_code", 0) or 0)
            stale = bool(motor.get("stale", True) or battery.get("stale", True))
            issues = len(offline) + len(motor_errors) + len(hot) + int(power_error != 0)
            return {
                "state": "unknown" if stale else ("fault" if issues else "ok"),
                "offline_joints": offline,
                "disabled_joints": disabled,
                "motor_errors": motor_errors,
                "hot_motors": hot,
                "power_error_code": power_error,
                "source_stale": stale,
                "timestamp_ms": _now_ms(),
            }
        if name == "stability":
            imu = self._snapshot("imu")
            rpy = list(imu.get("rpy_rad", []))
            angular = list(imu.get("angular_velocity_rad_s", []))
            if len(rpy) < 2 or len(angular) < 3:
                return {"state": "no_data", "stale": True, "timestamp_ms": _now_ms()}
            roll, pitch = float(rpy[0]), float(rpy[1])
            angular_speed = math.sqrt(sum(float(value) ** 2 for value in angular[:3]))
            tilt = max(abs(roll), abs(pitch))
            fall_tilt = float(self._config["control"].get("fall_tilt_rad", 0.9))
            warn_tilt = float(self._config["control"].get("tilt_warning_rad", 0.45))
            fall_rate = float(self._config["control"].get("fall_angular_speed_rad_s", 3.0))
            state = "fall_risk" if tilt >= fall_tilt or angular_speed >= fall_rate else (
                "tilted" if tilt >= warn_tilt else "stable"
            )
            return {
                "state": state,
                "roll_rad": roll,
                "pitch_rad": pitch,
                "tilt_rad": tilt,
                "angular_speed_rad_s": angular_speed,
                "source_stale": bool(imu.get("stale", True)),
                "timestamp_ms": _now_ms(),
            }
        if name == "joint_groups":
            return {
                "groups": {
                    group: [{"index": index, "name": T800_JOINT_NAMES[index]} for index in indices]
                    for group, indices in T800_JOINT_GROUPS.items()
                },
                "timestamp_ms": _now_ms(),
            }
        if name == "capabilities":
            return {
                "robot": "EngineAI T800 Development Edition",
                "dof": len(T800_JOINT_NAMES),
                "native_motion_states": list(MOTION_STATES),
                "control": [
                    "body_velocity", "open_loop_displacement", "open_loop_turn", "open_loop_arc",
                    "motion_fsm", "joint_plan", "joint_override", "joint_bridge", "native_node_control",
                    "gesture_sequences", "dance", "virtual_gamepad", "soft_emergency_stop",
                    "motor_power", "led", "tts", "ros_graph_discovery",
                ],
                "feedback": list(self._STREAMS) + [
                    "joint_plan_state", "heartbeat_status",
                    "motion_command_trace", "native_interface_probe",
                    "motion_events", "mainboard",
                ],
                "limitations": [
                    "no odometry topic: displacement/turn/arc are time-integrated open-loop estimates",
                    "no public dexterous-hand interface in the referenced T800 protocol",
                ],
                "timestamp_ms": _now_ms(),
            }
        if name == "ros_graph":
            def graph(method: str) -> list:
                callback = getattr(self._sub_node, method, None)
                if callback is None:
                    return []
                try:
                    return callback()
                except Exception:
                    return []

            topics = [
                {"name": topic, "types": list(types)}
                for topic, types in graph("get_topic_names_and_types")
            ]
            services = [
                {"name": service, "types": list(types)}
                for service, types in graph("get_service_names_and_types")
            ]
            nodes = [
                {"name": node, "namespace": namespace}
                for node, namespace in graph("get_node_names_and_namespaces")
            ]
            configured = set(self._topics.values())
            return {
                "state": "available" if topics or services or nodes else "no_data",
                "nodes": nodes,
                "topics": topics,
                "services": services,
                "unmapped_topics": [item for item in topics if item["name"] not in configured],
                "timestamp_ms": _now_ms(),
            }
        if name == "mainboard":
            return self._mainboard_snapshot()
        return {"error": f"unknown derived state: {name}"}

    def _mainboard_snapshot(self) -> dict:
        motor = self._snapshot("motor_health")
        power = self._snapshot("battery")
        if "mos_temperature_c" in motor or "voltage_v" in power:
            return self._mainboard_hardware_snapshot(motor, power)

        candidates = self._mainboard_candidates()
        strong = [
            item for item in candidates
            if self._is_strong_mainboard_name(item["name"])
        ]
        state = "source_discovered" if strong else "no_data_source"
        source = strong[0] if strong else None
        return {
            "state": state,
            "ok": False,
            "source": source,
            "discovered_candidates": candidates,
            "message": (
                "mainboard candidate topic discovered; add a typed adapter after confirming the message contract"
                if source else "No confirmed T800 mainboard telemetry topic or API was found."
            ),
            "search_keywords": list(self._MAINBOARD_KEYWORDS),
            "timestamp_ms": _now_ms(),
        }

    def _mainboard_hardware_snapshot(self, motor: dict, power: dict) -> dict:
        mos_temperatures = [float(value) for value in motor.get("mos_temperature_c", [])]
        motor_errors = [int(value) for value in motor.get("error_code", [])]
        offline = [int(value) for value in motor.get("offline", [])]
        disabled = [0 if bool(value) else 1 for value in motor.get("enabled", [])]

        board_temperature = max(mos_temperatures) if mos_temperatures else None
        warning_temperature = float(self._config["control"].get("motor_warning_temperature_c", 70.0))
        temperature_warning = board_temperature is not None and board_temperature >= warning_temperature
        power_error = int(power.get("error_code", 0) or 0)
        motor_error_count = sum(1 for value in motor_errors if value != 0)
        offline_count = sum(1 for value in offline if value != 0)
        disabled_count = sum(1 for value in disabled if value != 0)
        stale = bool(motor.get("stale", True) or power.get("stale", True))

        if stale:
            state = "stale"
        elif power_error or motor_error_count or offline_count or disabled_count:
            state = "error"
        elif temperature_warning:
            state = "warning"
        else:
            state = "ok"

        ages = [
            value for value in (motor.get("age_sec"), power.get("age_sec"))
            if value is not None
        ]
        message_parts = []
        if stale:
            message_parts.append("hardware telemetry is stale")
        if temperature_warning:
            message_parts.append(f"board temperature >= {warning_temperature:.0f} C")
        if power_error:
            message_parts.append(f"power error code {power_error}")
        if motor_error_count:
            message_parts.append(f"{motor_error_count} motor driver errors")
        if offline_count:
            message_parts.append(f"{offline_count} offline motor drivers")
        if disabled_count:
            message_parts.append(f"{disabled_count} disabled motor drivers")
        message = "; ".join(message_parts) if message_parts else "hardware telemetry normal"

        return {
            "state": state,
            "ok": state == "ok",
            "temperature": [] if board_temperature is None else [round(board_temperature, 1)],
            "power": {
                "enabled": power.get("enabled"),
                "battery_percentage": None if power.get("percentage") is None else round(float(power["percentage"]), 1),
                "input_voltage_v": None if power.get("voltage_v") is None else round(float(power["voltage_v"]), 2),
                "current_a": None if power.get("current_a") is None else round(float(power["current_a"]), 2),
                "current_limit_a": None if power.get("current_limit_a") is None else round(float(power["current_limit_a"]), 1),
            },
            "diagnostics": {
                "error_code": power_error,
                "motor_error_count": motor_error_count,
                "offline_count": offline_count,
                "disabled_count": disabled_count,
                "message": message,
            },
            "age_sec": round(max(ages), 1) if ages else None,
            "timestamp_ms": _now_ms(),
        }

    def _mainboard_candidates(self) -> list[dict]:
        candidates = []
        configured = set(self._topics.values())
        for name, types in _graph_items(self._sub_node, "get_topic_names_and_types"):
            if not self._is_mainboard_candidate(name, types):
                continue
            candidates.append({
                "kind": "topic",
                "name": name,
                "message_types": list(types),
                "configured": name in configured,
            })
        for name, types in _graph_items(self._sub_node, "get_service_names_and_types"):
            if not self._is_mainboard_candidate(name, types):
                continue
            candidates.append({
                "kind": "service",
                "name": name,
                "message_types": list(types),
                "configured": False,
            })
        return candidates

    @classmethod
    def _is_mainboard_candidate(cls, name: str, interface_types: list[str]) -> bool:
        if cls._is_strong_mainboard_name(name):
            return True
        if any(cls._is_strong_mainboard_name(interface_type) for interface_type in interface_types):
            return True
        lowered = " ".join([name, *interface_types]).lower()
        return any(
            keyword in lowered
            for keyword in ("thermal", "temperature", "fan", "diagnostic")
        )

    @staticmethod
    def _is_strong_mainboard_name(name: str) -> bool:
        lowered = name.lower().replace("-", "_")
        if "mainboard" in lowered or "main_board" in lowered:
            return True
        tokenized = lowered
        for separator in ("/", ".", "_", ":"):
            tokenized = tokenized.replace(separator, " ")
        return "board" in tokenized.split()

    def _set(self, name: str, payload: dict) -> None:
        with self._lock:
            payload["timestamp_ms"] = _now_ms()
            self._cache[name] = payload
            self._updated[name] = time.monotonic()

    def _snapshot(self, name: str) -> dict:
        if name == "driver_health":
            return self._health()
        with self._lock:
            payload = dict(self._cache.get(name, {"state": "no_data"}))
            updated = self._updated.get(name)
        payload["age_sec"] = None if updated is None else max(0.0, time.monotonic() - updated)
        payload["stale"] = updated is None or payload["age_sec"] > self._timeout
        return payload

    def _health(self) -> dict:
        now = time.monotonic()
        with self._lock:
            sources = {
                name: {
                    "connected": name in self._updated,
                    "age_sec": None if name not in self._updated else max(0.0, now - self._updated[name]),
                }
                for name in self._STREAMS
                if name != "driver_health"
            }
        for value in sources.values():
            value["stale"] = value["age_sec"] is None or value["age_sec"] > self._timeout
        connected = sum(1 for value in sources.values() if value["connected"])
        fresh = sum(1 for value in sources.values() if value["connected"] and not value["stale"])
        if fresh:
            state = "running"
        elif connected:
            state = "degraded"
        else:
            state = "waiting"
        return {
            "state": state,
            "connected_sources": connected,
            "fresh_sources": fresh,
            "sources": sources,
            "robot_domain_id": self._config["ros"]["robot_domain_id"],
            "core_domain_id": self._config["ros"]["core_domain_id"],
            "timestamp_ms": _now_ms(),
        }

    def _on_joints(self, msg) -> None:
        payload = joint_payload(msg.position, msg.velocity, msg.torque)
        with self._lock:
            self._last_joint_positions[: len(msg.position)] = list(msg.position)
        self._set("joints", payload)

    def _on_imu(self, msg) -> None:
        self._set(
            "imu",
            {
                "quaternion_wxyz": [msg.quaternion.w, msg.quaternion.x, msg.quaternion.y, msg.quaternion.z],
                "rpy_rad": [msg.rpy.x, msg.rpy.y, msg.rpy.z],
                "linear_acceleration_m_s2": [
                    msg.linear_acceleration.x,
                    msg.linear_acceleration.y,
                    msg.linear_acceleration.z,
                ],
                "angular_velocity_rad_s": [
                    msg.angular_velocity.x,
                    msg.angular_velocity.y,
                    msg.angular_velocity.z,
                ],
            },
        )

    def _on_gamepad(self, msg) -> None:
        self._set(
            "gamepad",
            {
                "hardware_connected": bool(msg.hardware_connected),
                "digital_states": list(msg.digital_states),
                "analog_states": list(msg.analog_states),
            },
        )

    def _on_motor_debug(self, msg) -> None:
        self._set(
            "motor_health",
            {
                "mos_temperature_c": list(msg.mos_temperature),
                "motor_temperature_c": list(msg.motor_temperature),
                "voltage_v": list(msg.voltage),
                "current_a": list(msg.current),
                "error_code": list(msg.error_code),
                "offline": list(msg.offline),
                "enabled": list(msg.enable),
            },
        )

    def _on_power(self, msg) -> None:
        self._set(
            "battery",
            {
                "enabled": bool(msg.enable),
                "percentage": float(msg.percentage),
                "voltage_v": float(msg.voltage),
                "current_a": float(msg.current),
                "current_limit_a": float(msg.current_limit),
                "error_code": int(msg.error_code),
            },
        )

    def _on_motor_state(self, msg) -> None:
        self._set(
            "motor_state",
            {"position_rad": list(msg.position), "velocity_rad_s": list(msg.velocity), "torque_nm": list(msg.torque)},
        )

    def _on_joint_command_feedback(self, msg) -> None:
        self._set("joint_command_feedback", self._joint_command_payload(msg))

    def _on_motor_command(self, msg) -> None:
        self._set("motor_command", self._joint_command_payload(msg))

    @staticmethod
    def _joint_command_payload(msg) -> dict:
        return {
            "position_rad": list(msg.position),
            "velocity_rad_s": list(msg.velocity),
            "feed_forward_torque_nm": list(msg.feed_forward_torque),
            "torque_nm": list(msg.torque),
            "stiffness": list(msg.stiffness),
            "damping": list(msg.damping),
            "parallel_parser_type": int(msg.parallel_parser_type),
        }

    def _on_motion(self, msg) -> None:
        payload = {
            "current_motion_task": msg.current_motion_task,
            "available_transition_motions": list(msg.available_transition_motions),
        }
        with self._lock:
            self._current_motion = msg.current_motion_task
            self._available_motions = list(msg.available_transition_motions)
        self._set("motion_state", payload)

    def _publish_tick(self) -> None:
        if not self._running:
            return
        tick = getattr(self, "_tick", 0) + 1
        self._tick = tick
        schedules = {
            "joints": 1,
            "imu": 1,
            "motion_state": 4,
            "gamepad": 2,
            "motor_health": 4,
            "motor_state": 4,
            "motor_command": 4,
            "joint_command_feedback": 4,
            "battery": 20,
            "driver_health": 20,
        }
        for name, divisor in schedules.items():
            if tick % divisor:
                continue
            if name != "driver_health" and name not in self._cache:
                continue
            self._publishers[name].publish(_json_message(self._snapshot(name)))
        if tick % 20 == 0:
            for name, publisher in self._derived_publishers.items():
                publisher.publish(_json_message(self._derived_snapshot(name)))


class HeartbeatStatusPlugin:
    def __init__(self, config: dict, namespace: str, ros2):
        self._config = config
        self._ns = namespace
        self._timeout = float(config["ros"].get("source_timeout_sec", 1.0))
        self._node = Node("t800_heartbeat_status", context=ros2.ctx_robot)
        self._pub_node = Node("t800_heartbeat_status_pub", context=ros2.ctx_core)
        ros2.executor_robot.add_node(self._node)
        ros2.executor_core.add_node(self._pub_node)
        self._publisher = self._pub_node.create_publisher(
            String, f"/{namespace}/state/heartbeat_status", _RELIABLE
        )
        self._lock = threading.RLock()
        self._latest: dict | None = None
        self._updated: float | None = None

    def get_tool(self) -> dict:
        return {
            "name": "heartbeat_status",
            "type": "sensor",
            "multiInstance": False,
            "readOnly": True,
            "description": "显示 T800 ROS2 节点心跳、健康状态和数据新鲜度",
            "inputSchema": action_schema(
                _with_lifecycle({
                    "status": ([], "返回适合画布验收的简洁心跳状态"),
                    "debug": ([], "返回原始心跳字段，供研发排查"),
                }),
                {},
                "心跳查询动作",
            ),
            "topic_out": [{"topic": f"/{self._ns}/state/heartbeat_status", "format": "data/json"}],
        }

    def start(self) -> None:
        from interface_protocol.msg import Heartbeat

        self._node.create_subscription(
            Heartbeat, self._config["topics"]["heartbeat"], self._on_heartbeat, _BEST_EFFORT
        )
        self._pub_node.create_timer(0.2, self._publish)

    def stop(self) -> None:
        pass

    def dispatch(self, action: str, args: dict) -> dict:
        if action == "info":
            return _with_topic_out(self._snapshot(), f"/{self._ns}/state/heartbeat_status")
        if action in ("heartbeat_status", "status", "start"):
            return self._snapshot()
        if action == "debug":
            return self._debug_snapshot()
        if action == "stop":
            return {"state": "idle"}
        return {"error": f"unknown heartbeat action: {action}"}

    def _on_heartbeat(self, msg) -> None:
        with self._lock:
            self._latest = {
                "node_name": str(msg.node_name),
                "node_status": str(msg.node_status),
                "startup_timestamp": int(msg.startup_timestamp),
                "error_code": int(msg.error_code),
                "error_message": str(msg.error_message),
            }
            self._updated = time.monotonic()

    def _snapshot(self) -> dict:
        with self._lock:
            latest = dict(self._latest or {})
            updated = self._updated
        age_sec = None if updated is None else max(0.0, time.monotonic() - updated)
        stale = updated is None or age_sec > self._timeout
        if updated is None:
            return {
                "state": "no_data",
                "health": "unknown",
                "node": None,
                "message": "no heartbeat data",
                "age_sec": None,
                "timestamp_ms": _now_ms(),
            }
        error_code = int(latest.get("error_code", 0))
        message = str(latest.get("error_message") or latest.get("node_status") or "ok")
        return {
            "state": "stale" if stale else "running",
            "health": "error" if error_code else "ok",
            "node": latest.get("node_name"),
            "message": message,
            "age_sec": round(age_sec, 1),
            "timestamp_ms": _now_ms(),
        }

    def _debug_snapshot(self) -> dict:
        with self._lock:
            latest = dict(self._latest or {})
            updated = self._updated
        age_sec = None if updated is None else max(0.0, time.monotonic() - updated)
        stale = updated is None or age_sec > self._timeout
        if updated is None:
            return {
                "state": "no_data", "node_name": None, "node_status": None,
                "startup_timestamp": None, "error_code": None, "error_message": None,
                "age_sec": None, "stale": True, "timestamp_ms": _now_ms(),
            }
        latest.update({"state": "stale" if stale else "running", "age_sec": age_sec,
                       "stale": stale, "timestamp_ms": _now_ms()})
        return latest

    def _publish(self) -> None:
        self._publisher.publish(_json_message(self._snapshot()))

class MotionCommandTracePlugin:
    def __init__(self, config: dict, namespace: str, ros2):
        self._config = config
        self._ns = namespace
        capacity = max(1, int(config.get("diagnostics", {}).get("command_trace_capacity", 20)))
        self._node = Node("t800_motion_command_trace", context=ros2.ctx_robot)
        self._pub_node = Node("t800_motion_command_trace_pub", context=ros2.ctx_core)
        ros2.executor_robot.add_node(self._node)
        ros2.executor_core.add_node(self._pub_node)
        self._publisher = self._pub_node.create_publisher(
            String, f"/{namespace}/state/motion_command_trace", _RELIABLE
        )
        self._lock = threading.RLock()
        self._velocity_commands = deque(maxlen=capacity)
        self._motion_requests = deque(maxlen=capacity)
        self._gamepad_inputs = deque(maxlen=capacity)
        self._odometry_samples = deque(maxlen=capacity)
        self._velocity_count = 0
        self._motion_count = 0
        self._gamepad_count = 0
        self._odometry_count = 0
        self._velocity_updated: float | None = None
        self._motion_updated: float | None = None
        self._gamepad_updated: float | None = None
        self._odometry_updated: float | None = None

    def _max_reasonable_speed(self) -> float:
        control = self._config.get("control", {})
        max_vx = abs(float(control.get("max_vx", 3.0)))
        max_vy = abs(float(control.get("max_vy", 1.0)))
        # Leave a small margin for measured odometry noise while still rejecting
        # ODIN2-internal values that are not robot m/s velocity on T800.
        return max(1.0, math.hypot(max_vx, max_vy) * 1.5)

    def _is_valid_speed(self, speed: float) -> bool:
        return math.isfinite(speed) and 0.0 <= speed <= self._max_reasonable_speed()

    def get_tool(self) -> dict:
        return {
            "name": "motion_command_trace",
            "type": "sensor",
            "multiInstance": False,
            "readOnly": True,
            "description": "输出 T800 当前速度，兼容手柄、驱动指令和里程计来源",
            "inputSchema": action_schema(
                _with_lifecycle({
                    "status": ([], "返回适合画布验收的简洁速度状态"),
                    "debug": ([], "返回最近命令、手柄、里程计原始摘要，供研发排查"),
                }),
                {},
                "运动速度查询",
            ),
            "topic_out": [{"topic": f"/{self._ns}/state/motion_command_trace", "format": "data/json"}],
        }

    def start(self) -> None:
        from interface_protocol.msg import BodyVelCmd, GamepadKeys, MotionStateRequest
        from nav_msgs.msg import Odometry

        topics = self._config["topics"]
        self._node.create_subscription(
            BodyVelCmd, topics["body_velocity"], self._on_velocity, _RELIABLE
        )
        self._node.create_subscription(
            MotionStateRequest, topics["motion_request"], self._on_motion_request, _RELIABLE_ONE
        )
        self._node.create_subscription(
            GamepadKeys, topics["gamepad"], self._on_gamepad, _BEST_EFFORT
        )
        odometry_topic = str(topics.get("odometry", "")).strip()
        if odometry_topic:
            self._node.create_subscription(
                Odometry, odometry_topic, self._on_odometry, _BEST_EFFORT
            )
        self._pub_node.create_timer(0.2, self._publish)

    def stop(self) -> None:
        pass

    def dispatch(self, action: str, args: dict) -> dict:
        if action == "info":
            return _with_topic_out(self._snapshot(), f"/{self._ns}/state/motion_command_trace")
        if action in ("motion_command_trace", "status", "start"):
            return self._snapshot()
        if action == "debug":
            return self._debug_snapshot()
        if action == "stop":
            return {"state": "idle"}
        return {"error": f"unknown motion trace action: {action}"}

    def _on_velocity(self, msg) -> None:
        entry = {
            "linear_velocity": [float(value) for value in msg.linear_velocity],
            "yaw_velocity": float(msg.yaw_velocity),
            "timestamp_ms": _now_ms(),
        }
        with self._lock:
            self._velocity_commands.append(entry)
            self._velocity_count += 1
            self._velocity_updated = time.monotonic()

    def _on_motion_request(self, msg) -> None:
        entry = {"target_motion_name": str(msg.target_motion_name), "timestamp_ms": _now_ms()}
        with self._lock:
            self._motion_requests.append(entry)
            self._motion_count += 1
            self._motion_updated = time.monotonic()

    def _on_gamepad(self, msg) -> None:
        analog = [float(value) for value in msg.analog_states]
        max_vx = abs(float(self._config.get("control", {}).get("max_vx", 3.0)))
        max_vy = abs(float(self._config.get("control", {}).get("max_vy", 1.0)))
        stick_x = analog[2] if len(analog) > 2 else 0.0
        stick_y = analog[3] if len(analog) > 3 else 0.0
        yaw_x = analog[4] if len(analog) > 4 else 0.0
        estimated_speed = math.hypot(stick_y * max_vx, stick_x * max_vy)
        entry = {
            "hardware_connected": bool(msg.hardware_connected),
            "control_source": _gamepad_control_source(msg),
            "digital_pressed": [index for index, value in enumerate(msg.digital_states) if int(value) != 0],
            "buttons": _pressed_gamepad_buttons(msg.digital_states),
            "analog_states": analog,
            "left_stick": {"x": stick_x if len(analog) > 2 else None, "y": stick_y if len(analog) > 3 else None},
            "right_stick": {"x": analog[4] if len(analog) > 4 else None, "y": analog[5] if len(analog) > 5 else None},
            "direction": _motion_direction(stick_x, stick_y, yaw_x),
            "estimated_speed_m_s": estimated_speed,
            "timestamp_ms": _now_ms(),
        }
        with self._lock:
            self._gamepad_inputs.append(entry)
            self._gamepad_count += 1
            self._gamepad_updated = time.monotonic()

    def _on_odometry(self, msg) -> None:
        linear = msg.twist.twist.linear
        angular = msg.twist.twist.angular
        vx, vy, vz = float(linear.x), float(linear.y), float(linear.z)
        speed = math.sqrt(vx * vx + vy * vy + vz * vz)
        entry = {
            "frame_id": str(getattr(msg.header, "frame_id", "")),
            "child_frame_id": str(getattr(msg, "child_frame_id", "")),
            "linear_velocity": {"x": vx, "y": vy, "z": vz},
            "speed_m_s": speed,
            "valid": self._is_valid_speed(speed),
            "angular_velocity": {"x": float(angular.x), "y": float(angular.y), "z": float(angular.z)},
            "yaw_rate_rad_s": float(angular.z),
            "timestamp_ms": _now_ms(),
        }
        with self._lock:
            self._odometry_samples.append(entry)
            self._odometry_count += 1
            self._odometry_updated = time.monotonic()

    def _snapshot(self) -> dict:
        now = time.monotonic()
        with self._lock:
            velocity = self._velocity_commands[-1] if self._velocity_commands else None
            gamepad = self._gamepad_inputs[-1] if self._gamepad_inputs else None
            odometry = self._odometry_samples[-1] if self._odometry_samples else None
            velocity_updated = self._velocity_updated
            gamepad_updated = self._gamepad_updated
            odometry_updated = self._odometry_updated
        timeout_sec = float(self._config["ros"].get("source_timeout_sec", 1.0))
        odometry_age = None if odometry_updated is None else max(0.0, now - odometry_updated)
        velocity_age = None if velocity_updated is None else max(0.0, now - velocity_updated)
        gamepad_age = None if gamepad_updated is None else max(0.0, now - gamepad_updated)

        source = "none"
        speed = 0.0
        age_sec = None
        odometry_valid = bool(odometry and odometry.get("valid", False))
        odometry_fresh = odometry_valid and odometry_age is not None and odometry_age <= timeout_sec
        velocity_fresh = velocity is not None and velocity_age is not None and velocity_age <= timeout_sec
        use_odometry = odometry_fresh or (
            odometry_valid
            and not velocity_fresh
            and (velocity_age is None or (odometry_age is not None and odometry_age <= velocity_age))
        )
        if use_odometry:
            source = "odometry"
            speed = float(odometry.get("speed_m_s", 0.0))
            age_sec = odometry_age
        elif velocity is not None:
            source = "body_velocity_command"
            values = [float(value) for value in velocity.get("linear_velocity", [])]
            speed = math.sqrt(sum(value * value for value in values))
            age_sec = velocity_age
        elif gamepad is not None:
            source = str(gamepad.get("control_source", "gamepad"))
            speed = float(gamepad.get("estimated_speed_m_s", 0.0))
            age_sec = gamepad_age

        stale = age_sec is None or age_sec > timeout_sec
        speed_rounded = round(speed, 2)
        motion_state = "moving" if speed_rounded >= 0.03 else "stopped"
        state = "no_data" if source == "none" else ("stale" if stale else "running")
        return {
            "state": state,
            "speed": f"{speed_rounded:.2f} m/s",
            "motion_state": motion_state,
            "source": source,
            "direction": "none" if gamepad is None else str(gamepad.get("direction", "none")),
            "gamepad_connected": None if gamepad is None else bool(gamepad.get("hardware_connected")),
            "age_sec": None if age_sec is None else round(age_sec, 1),
            "timestamp_ms": _now_ms(),
        }

    def _debug_snapshot(self) -> dict:
        now = time.monotonic()
        with self._lock:
            velocity = list(self._velocity_commands)
            motions = list(self._motion_requests)
            gamepad = list(self._gamepad_inputs)
            odometry = list(self._odometry_samples)
            velocity_updated = self._velocity_updated
            motion_updated = self._motion_updated
            gamepad_updated = self._gamepad_updated
            odometry_updated = self._odometry_updated
            counts = {
                "body_velocity": self._velocity_count,
                "motion_request": self._motion_count,
                "gamepad": self._gamepad_count,
                "odometry": self._odometry_count,
            }
        ages = {
            "body_velocity": None if velocity_updated is None else max(0.0, now - velocity_updated),
            "motion_request": None if motion_updated is None else max(0.0, now - motion_updated),
            "gamepad": None if gamepad_updated is None else max(0.0, now - gamepad_updated),
            "odometry": None if odometry_updated is None else max(0.0, now - odometry_updated),
        }
        return {
            "state": "running" if velocity or motions or gamepad or odometry else "no_data",
            "velocity_commands": velocity,
            "motion_requests": motions,
            "gamepad_inputs": gamepad,
            "odometry_samples": odometry,
            "latest_velocity_command": velocity[-1] if velocity else None,
            "latest_motion_request": motions[-1] if motions else None,
            "latest_gamepad_input": gamepad[-1] if gamepad else None,
            "latest_measured_velocity": odometry[-1] if odometry else None,
            "last_seen_age_sec": ages,
            "command_count": {**counts, "total": sum(counts.values())},
            "timestamp_ms": _now_ms(),
        }

    def _publish(self) -> None:
        self._publisher.publish(_json_message(self._snapshot()))

class MotionEventsPlugin:
    """Read-only event timeline for T800 motion-related MCP calls and feedback."""

    _MOTION_TOOLS = {
        "loco",
        "motion_mode",
        "dance",
        "joint_plan",
        "joint_plan_state",
        "gesture",
        "joint_override",
        "joint_bridge",
        "motor_power",
        "native_node_control",
        "virtual_gamepad",
        "safety",
        "motion_state",
    }

    def __init__(self, config: dict, namespace: str, ros2):
        self._config = config
        self._ns = namespace
        capacity = int(config.get("diagnostics", {}).get("motion_events_capacity", 100))
        self._capacity = max(1, min(capacity, 1000))
        self._robot_node = Node("t800_motion_events_sub", context=ros2.ctx_robot)
        self._node = Node("t800_motion_events_pub", context=ros2.ctx_core)
        ros2.executor_robot.add_node(self._robot_node)
        ros2.executor_core.add_node(self._node)
        self._publisher = self._node.create_publisher(
            String, f"/{namespace}/state/motion_events", _RELIABLE
        )
        self._lock = threading.RLock()
        self._events = deque(maxlen=self._capacity)
        self._sequence = 0
        self._moving = False
        self._latest_speed = 0.0
        self._latest_speed_source = "none"
        self._latest_speed_updated: float | None = None
        self._latest_action = "none"
        self._latest_buttons: list[str] = []
        self._latest_control_source = "none"
        self._latest_direction = "none"
        self._current_motion_state = "unknown"
        self._last_motion_state = "unknown"
        self._last_gamepad_signature = ""
        self._motion_start_threshold = 0.05
        self._motion_stop_threshold = 0.03

    def _max_reasonable_speed(self) -> float:
        control = self._config.get("control", {})
        max_vx = abs(float(control.get("max_vx", 3.0)))
        max_vy = abs(float(control.get("max_vy", 1.0)))
        return max(1.0, math.hypot(max_vx, max_vy) * 1.5)

    def _classify_gamepad_action(self, buttons: list[str], speed: float) -> str:
        button_set = frozenset(buttons)
        if button_set in _GAMEPAD_ACTIONS:
            return _GAMEPAD_ACTIONS[button_set]
        if speed >= self._motion_start_threshold:
            return "move"
        if buttons:
            return "buttons:" + "+".join(buttons)
        return "none"

    def get_tool(self) -> dict:
        return {
            "name": "motion_events",
            "type": "sensor",
            "multiInstance": False,
            "readOnly": True,
            "description": "T800 motion event stream — motion state changes and motion command lifecycle events",
            "inputSchema": action_schema(
                _with_lifecycle({
                    "status": ([], "返回最新事件摘要"),
                    "debug": (["limit", "since_event_id", "source_tool", "severity"], "返回筛选后的事件时间线"),
                }),
                {
                    "limit": {
                        "type": "integer",
                        "minimum": 1,
                        "maximum": 200,
                        "description": "最多返回事件条数",
                    },
                    "since_event_id": {
                        "type": "string",
                        "description": "只返回该事件之后的新事件",
                    },
                    "source_tool": {
                        "type": "string",
                        "description": "按来源 tool 过滤",
                    },
                    "severity": {
                        "type": "string",
                        "enum": ["debug", "info", "warning", "error"],
                        "description": "按事件级别过滤",
                    },
                },
                "运动事件查询动作",
            ),
            "topic_out": [{"topic": f"/{self._ns}/state/motion_events", "format": "data/json"}],
        }

    def start(self) -> None:
        from interface_protocol.msg import BodyVelCmd, GamepadKeys, MotionState, MotionStateRequest

        topics = self._config.get("topics", {})
        body_velocity_topic = str(topics.get("body_velocity", "")).strip()
        if body_velocity_topic:
            self._robot_node.create_subscription(
                BodyVelCmd, body_velocity_topic, self._on_velocity, _RELIABLE
            )
        motion_request_topic = str(topics.get("motion_request", "")).strip()
        if motion_request_topic:
            self._robot_node.create_subscription(
                MotionStateRequest, motion_request_topic, self._on_motion_request, _RELIABLE_ONE
            )
        gamepad_topic = str(topics.get("gamepad", "")).strip()
        if gamepad_topic:
            self._robot_node.create_subscription(
                GamepadKeys, gamepad_topic, self._on_gamepad, _BEST_EFFORT
            )
        motion_state_topic = str(topics.get("motion_state", "")).strip()
        if motion_state_topic:
            self._robot_node.create_subscription(
                MotionState, motion_state_topic, self._on_motion_state, _BEST_EFFORT
            )
        odometry_topic = str(self._config.get("topics", {}).get("odometry", "")).strip()
        if odometry_topic:
            from nav_msgs.msg import Odometry

            self._robot_node.create_subscription(
                Odometry, odometry_topic, self._on_odometry, _BEST_EFFORT
            )
        self._node.create_timer(0.2, self._publish)

    def stop(self) -> None:
        pass

    def dispatch(self, action: str, args: dict) -> dict:
        if action == "info":
            return _with_topic_out(self._summary_snapshot(), f"/{self._ns}/state/motion_events")
        if action in ("motion_events", "status", "start"):
            return self._summary_snapshot()
        if action in ("debug", "list"):
            return self._debug_snapshot(args)
        if action == "stop":
            return {"state": "idle"}
        return {"error": f"unknown motion events action: {action}"}

    def _on_velocity(self, msg) -> None:
        values = [float(value) for value in msg.linear_velocity]
        speed = math.sqrt(sum(value * value for value in values))
        self._handle_speed_sample(
            speed,
            source_tool="motion_command_trace",
            source_kind="body_velocity_command",
            detail={"linear_velocity": values, "yaw_velocity": float(msg.yaw_velocity)},
        )

    def _on_motion_request(self, msg) -> None:
        motion_name = str(msg.target_motion_name)
        action = _normalize_motion_action(motion_name)
        with self._lock:
            self._latest_action = action
            self._latest_control_source = "motion_request"
            self._latest_direction = "none"
        self.record_event(
            source_tool="motion_command_trace",
            event_type="motion_request",
            severity="info",
            phase="requested",
            summary=f"motion request: {action} ({motion_name})",
            detail={
                "action": action,
                "target_motion_name": motion_name,
                "source": self._config.get("topics", {}).get("motion_request"),
            },
        )

    def _on_gamepad(self, msg) -> None:
        analog = [float(value) for value in msg.analog_states]
        max_vx = abs(float(self._config.get("control", {}).get("max_vx", 3.0)))
        max_vy = abs(float(self._config.get("control", {}).get("max_vy", 1.0)))
        stick_x = analog[2] if len(analog) > 2 else 0.0
        stick_y = analog[3] if len(analog) > 3 else 0.0
        yaw_x = analog[4] if len(analog) > 4 else 0.0
        speed = math.hypot(stick_y * max_vx, stick_x * max_vy)
        buttons = _pressed_gamepad_buttons(msg.digital_states)
        control_source = _gamepad_control_source(msg)
        direction = _motion_direction(stick_x, stick_y, yaw_x)
        action = self._classify_gamepad_action(buttons, speed)
        signature = f"{control_source}|{action}|{direction}|{','.join(buttons)}|{round(speed, 2):.2f}"
        should_record_input = (bool(buttons) or speed >= self._motion_start_threshold) and signature != self._last_gamepad_signature
        with self._lock:
            self._latest_action = action
            self._latest_buttons = list(buttons)
            self._latest_control_source = control_source
            self._latest_direction = direction
            self._last_gamepad_signature = signature
        if should_record_input:
            self.record_event(
                source_tool="motion_command_trace",
                event_type="gamepad_action",
                severity="info",
                phase="input",
                summary=f"{control_source} action: {action}, {direction}, {speed:.2f} m/s",
                detail={
                    "action": action,
                    "buttons": buttons,
                    "control_source": control_source,
                    "direction": direction,
                    "speed": f"{speed:.2f} m/s",
                    "source": self._config.get("topics", {}).get("gamepad"),
                },
            )
        self._handle_speed_sample(
            speed,
            source_tool="motion_command_trace",
            source_kind="gamepad",
            detail={
                "hardware_connected": bool(msg.hardware_connected),
                "action": action,
                "buttons": buttons,
                "control_source": control_source,
                "direction": direction,
                "left_stick": {"x": stick_x if len(analog) > 2 else None, "y": stick_y if len(analog) > 3 else None},
                "source": self._config.get("topics", {}).get("gamepad"),
            },
        )

    def _on_motion_state(self, msg) -> None:
        current = str(getattr(msg, "current_motion_task", "") or "unknown")
        action = _normalize_motion_action(current)
        with self._lock:
            previous = self._current_motion_state
            self._last_motion_state = previous
            self._current_motion_state = current
            self._latest_action = action
            self._latest_control_source = "motion_state"
            self._latest_direction = "none"
        if current and current != "unknown" and current != previous:
            self.record_event(
                source_tool="motion_state",
                event_type="motion_state_changed",
                severity="info",
                phase="confirmed",
                summary=f"motion state: {action} ({current})",
                detail={"action": action, "previous": previous, "current": current},
            )

    def _on_odometry(self, msg) -> None:
        linear = msg.twist.twist.linear
        vx, vy, vz = float(linear.x), float(linear.y), float(linear.z)
        speed = math.sqrt(vx * vx + vy * vy + vz * vz)
        if not math.isfinite(speed) or speed > self._max_reasonable_speed():
            return
        self._handle_speed_sample(
            speed,
            source_tool="motion_command_trace",
            source_kind="odometry",
            detail={
                "linear_velocity": {"x": vx, "y": vy, "z": vz},
                "source": self._config.get("topics", {}).get("odometry"),
            },
        )

    def _handle_speed_sample(self, speed: float, *, source_tool: str, source_kind: str, detail: dict) -> None:
        if not math.isfinite(speed) or speed < 0.0 or speed > self._max_reasonable_speed():
            return
        with self._lock:
            moving = self._moving
            if not moving and speed >= self._motion_start_threshold:
                self._moving = True
                event = "motion_start"
            elif moving and speed <= self._motion_stop_threshold:
                self._moving = False
                event = "motion_stop"
            else:
                event = ""
            self._latest_speed = speed
            self._latest_speed_source = source_kind
            self._latest_speed_updated = time.monotonic()
            if source_kind != "gamepad":
                self._latest_control_source = source_kind
                self._latest_direction = "none"
        if not event:
            return
        event_phase = "running" if event == "motion_start" else "completed"
        self.record_event(
            source_tool=source_tool,
            event_type=event,
            severity="info",
            phase=event_phase,
            summary=f"{event} detected from {source_kind} speed {speed:.2f} m/s",
            detail={"speed": f"{speed:.2f} m/s", "source_kind": source_kind, **detail},
        )

    def record_tool_call(self, tool_name: str, action: str, args: dict, result: dict) -> None:
        if tool_name not in self._MOTION_TOOLS:
            return
        event_type, severity, phase = self._classify(tool_name, action, result)
        self.record_event(
            source_tool=tool_name,
            event_type=event_type,
            severity=severity,
            phase=phase,
            summary=self._summary(tool_name, action, result),
            detail={
                "action": action,
                "arguments": self._compact_value(args),
                "result": self._compact_value(result),
            },
        )

    def record_exception(self, tool_name: str, action: str, args: dict, exc: Exception) -> None:
        if tool_name not in self._MOTION_TOOLS:
            return
        self.record_event(
            source_tool=tool_name,
            event_type="dispatch_exception",
            severity="error",
            phase="failed",
            summary=f"{tool_name}.{action} raised {type(exc).__name__}",
            detail={"action": action, "arguments": self._compact_value(args), "error": str(exc)},
        )

    def record_event(
        self,
        *,
        source_tool: str,
        event_type: str,
        severity: str,
        phase: str,
        summary: str,
        detail: dict | None = None,
    ) -> dict:
        with self._lock:
            self._sequence += 1
            event_id = f"t800-motion-{self._sequence:06d}"
            timestamp_ms = _now_ms()
            event = {
                "type": event_type,
                "timestamp": round(timestamp_ms / 1000.0, 3),
                "event_id": event_id,
                "timestamp_ms": timestamp_ms,
                "source_tool": source_tool,
                "event_type": event_type,
                "severity": severity,
                "phase": phase,
                "summary": summary,
                "detail": detail or {},
            }
            self._events.append(event)
            return dict(event)

    def _summary_snapshot(self) -> dict:
        now = time.monotonic()
        with self._lock:
            latest_event = dict(self._events[-1]) if self._events else None
            events = [dict(event) for event in self._events]
            total_recorded = self._sequence
            moving = self._moving
            latest_speed = self._latest_speed
            latest_speed_source = self._latest_speed_source
            latest_speed_updated = self._latest_speed_updated
            latest_action = self._latest_action
            latest_buttons = list(self._latest_buttons)
            latest_control_source = self._latest_control_source
            latest_direction = self._latest_direction
            current_motion_state = self._current_motion_state
        speed_age = None if latest_speed_updated is None else max(0.0, now - latest_speed_updated)
        display_action = latest_action
        display_control_source = latest_control_source
        if display_action == "none" and current_motion_state not in ("", "unknown"):
            display_action = _normalize_motion_action(current_motion_state)
            display_control_source = "motion_state"
        return {
            "state": "running" if latest_event or latest_speed_updated is not None else "no_data",
            "motion_state": "moving" if moving else "stopped",
            "speed": f"{round(latest_speed, 2):.2f} m/s",
            "speed_source": latest_speed_source,
            "control_source": display_control_source,
            "action": display_action,
            "direction": latest_direction,
            "buttons": latest_buttons,
            "current_motion_state": current_motion_state,
            "event": None if latest_event is None else latest_event.get("type"),
            "summary": None if latest_event is None else latest_event.get("summary"),
            "event_id": None if latest_event is None else latest_event.get("event_id"),
            "event_count": total_recorded,
        }

    def _debug_snapshot(self, args: dict | None = None) -> dict:
        args = args or {}
        limit = int(args.get("limit", 50))
        limit = max(1, min(limit, 200))
        since_event_id = str(args.get("since_event_id", "") or "")
        source_tool = str(args.get("source_tool", "") or "")
        severity = str(args.get("severity", "") or "")
        with self._lock:
            events = [dict(event) for event in self._events]
            total_buffered = len(self._events)
            latest_event_id = events[-1]["event_id"] if events else None
            total_recorded = self._sequence
        if since_event_id:
            events = self._after_event(events, since_event_id)
        if source_tool:
            events = [event for event in events if event["source_tool"] == source_tool]
        if severity:
            events = [event for event in events if event["severity"] == severity]
        events = events[-limit:]
        warning_count = sum(1 for event in events if event["severity"] == "warning")
        error_count = sum(1 for event in events if event["severity"] == "error")
        return {
            "state": "running" if events else "no_data",
            "ok": True,
            "robot": "t800",
            "tool": "motion_events",
            "events": events,
            "summary": {
                "capacity": self._capacity,
                "total_recorded": total_recorded,
                "total_buffered": total_buffered,
                "returned": len(events),
                "latest_event_id": latest_event_id,
                "warning_count": warning_count,
                "error_count": error_count,
            },
            "timestamp_ms": _now_ms(),
        }

    @staticmethod
    def _after_event(events: list[dict], since_event_id: str) -> list[dict]:
        for index, event in enumerate(events):
            if event["event_id"] == since_event_id:
                return events[index + 1:]
        return events

    @staticmethod
    def _classify(tool_name: str, action: str, result: dict) -> tuple[str, str, str]:
        if "error" in result:
            return "command_rejected", "warning", "rejected"
        state = str(result.get("state", "completed"))
        if state in ("timeout", "error", "failed"):
            return "command_failed", "error", "failed"
        if state == "rejected":
            return "command_rejected", "warning", "rejected"
        if action in ("status", "info", "start") or tool_name in ("motion_state", "joint_plan_state"):
            return "status_read", "debug", "completed"
        if action in ("stop", "stop_move", "stop_dance", "stop_gesture", "release", "stop_command", "soft_stop"):
            return "motion_stop", "info", "completed"
        if tool_name == "safety":
            return "safety_request", "warning", "dispatch"
        return "command_requested", "info", "dispatch"

    @staticmethod
    def _summary(tool_name: str, action: str, result: dict) -> str:
        if "error" in result:
            return f"{tool_name}.{action} rejected: {result['error']}"
        state = result.get("state", "completed")
        target = result.get("target") or result.get("target_motion") or result.get("node_name")
        suffix = f" -> {target}" if target else ""
        return f"{tool_name}.{action} {state}{suffix}"

    @classmethod
    def _compact_value(cls, value):
        if isinstance(value, dict):
            return {
                str(key): cls._compact_value(inner)
                for key, inner in value.items()
                if not str(key).startswith("_")
            }
        if isinstance(value, (list, tuple)):
            if len(value) > 8:
                return {"length": len(value), "preview": [cls._compact_value(item) for item in value[:8]]}
            return [cls._compact_value(item) for item in value]
        if isinstance(value, (str, int, float, bool)) or value is None:
            return value
        return str(value)

    def _publish(self) -> None:
        self._publisher.publish(_json_message(self._summary_snapshot()))

class NativeInterfaceProbePlugin:
    _NAME_KEYWORDS = ("engineai", "motion", "hardware")
    _PRIORITY_RANK = {"high": 3, "medium": 2, "low": 1}

    def __init__(self, config: dict, namespace: str, ros2):
        self._config = config
        self._ns = namespace
        self._node = Node("t800_native_interface_probe", context=ros2.ctx_robot)
        self._pub_node = Node("t800_native_interface_probe_pub", context=ros2.ctx_core)
        ros2.executor_robot.add_node(self._node)
        ros2.executor_core.add_node(self._pub_node)
        self._publisher = self._pub_node.create_publisher(
            String, f"/{namespace}/state/native_interface_probe", _RELIABLE
        )
        self._latest_scan: dict | None = None
        self._last_scan_at = 0.0

    def get_tool(self) -> dict:
        return {
            "name": "native_interface_probe",
            "type": "sensor",
            "multiInstance": False,
            "readOnly": True,
            "description": "扫描 T800 ROS 图，汇总已映射与未映射原生接口",
            "inputSchema": action_schema(
                _with_lifecycle({
                    "scan": ([], "扫描 ROS2 topic/service 并输出适合画布验收的摘要"),
                    "debug": ([], "输出完整 topic/service/mapped/unmapped 清单，供研发排查"),
                }),
                {},
                "原生接口探测动作",
            ),
            "topic_out": [{"topic": f"/{self._ns}/state/native_interface_probe", "format": "data/json"}],
        }

    def start(self) -> None:
        self._pub_node.create_timer(1.0, self._publish)

    def stop(self) -> None:
        pass

    def dispatch(self, action: str, args: dict) -> dict:
        if action == "info":
            return _with_topic_out(self._scan(), f"/{self._ns}/state/native_interface_probe")
        if action in ("native_interface_probe", "scan", "status", "start"):
            return self._scan()
        if action == "debug":
            return self._scan(debug=True)
        if action == "stop":
            return {"state": "idle"}
        return {"error": f"unknown native interface probe action: {action}"}

    def _scan(self, debug: bool = False) -> dict:
        topics = self._graph("get_topic_names_and_types")
        services = self._graph("get_service_names_and_types")
        configured_topics = set(self._config.get("topics", {}).values())
        configured_services = set(self._config.get("services", {}).values())
        mapped, unmapped = [], []
        for name, types in topics:
            for message_type in types:
                if not self._is_relevant(name, message_type):
                    continue
                candidate = {
                    "kind": "topic", "name": name, "message_type": message_type,
                    "publishers": self._count("count_publishers", name),
                    "subscribers": self._count("count_subscribers", name),
                    "suggested_priority": self._priority(name, message_type),
                }
                (mapped if name in configured_topics else unmapped).append(candidate)
        for name, types in services:
            for service_type in types:
                if not self._is_relevant(name, service_type):
                    continue
                candidate = {
                    "kind": "service", "name": name, "message_type": service_type,
                    "servers": self._count("count_services", name),
                    "clients": self._count("count_clients", name),
                    "suggested_priority": self._priority(name, service_type),
                }
                (mapped if name in configured_services else unmapped).append(candidate)
        result = {
            "state": "available" if topics or services else "no_data",
            "topics": [{"name": name, "types": list(types)} for name, types in topics],
            "services": [{"name": name, "types": list(types)} for name, types in services],
            "mapped": mapped,
            "unmapped_candidates": unmapped,
            "topic_count": len(topics),
            "service_count": len(services),
            "timestamp_ms": _now_ms(),
        }
        compact = self._compact_scan(result)
        self._latest_scan = compact
        self._last_scan_at = time.monotonic()
        return result if debug else compact

    @classmethod
    def _compact_scan(cls, result: dict) -> dict:
        unmapped = sorted(
            result.get("unmapped_candidates", []),
            key=lambda item: cls._PRIORITY_RANK.get(str(item.get("suggested_priority")), 0),
            reverse=True,
        )
        high_priority = [
            {
                "kind": item.get("kind"),
                "name": item.get("name"),
                "type": item.get("message_type"),
                "priority": item.get("suggested_priority"),
            }
            for item in unmapped[:5]
        ]
        return {
            "state": result.get("state"),
            "topic_count": result.get("topic_count", 0),
            "service_count": result.get("service_count", 0),
            "mapped_count": len(result.get("mapped", [])),
            "unmapped_count": len(result.get("unmapped_candidates", [])),
            "top_unmapped": high_priority,
            "summary_text": (
                f"{result.get('topic_count', 0)} topics, "
                f"{result.get('service_count', 0)} services, "
                f"{len(result.get('unmapped_candidates', []))} unmapped"
            ),
            "timestamp_ms": result.get("timestamp_ms", _now_ms()),
        }

    def _publish(self) -> None:
        payload = self._latest_scan
        if payload is None or time.monotonic() - self._last_scan_at > 5.0:
            payload = self._scan()
        self._publisher.publish(_json_message(payload))

    def _graph(self, method: str) -> list:
        callback = getattr(self._node, method, None)
        if callback is None:
            return []
        try:
            return list(callback())
        except Exception:
            return []

    def _count(self, method: str, name: str) -> int:
        callback = getattr(self._node, method, None)
        if callback is None:
            return 0
        try:
            return int(callback(name))
        except Exception:
            return 0

    @classmethod
    def _is_relevant(cls, name: str, interface_type: str) -> bool:
        lowered_name = name.lower()
        lowered_type = interface_type.lower()
        return "interface_protocol" in lowered_type or any(
            keyword in lowered_name or keyword in lowered_type for keyword in cls._NAME_KEYWORDS
        )

    @staticmethod
    def _priority(name: str, interface_type: str) -> str:
        lowered_type = interface_type.lower()
        lowered_name = name.lower()
        if "interface_protocol" in lowered_type:
            return "high"
        if "motion" in lowered_name or "hardware" in lowered_name:
            return "medium"
        return "low"

class LocomotionPlugin:
    STOP_PRIORITY = 0

    def __init__(self, config: dict, namespace: str, ros2, state: StatePlugin):
        self._config = config
        self._state = state
        self._node = Node("t800_locomotion", context=ros2.ctx_robot)
        ros2.executor_robot.add_node(self._node)
        self._publisher = None
        self._watchdog_timer = None
        limits = config.get("control", {})
        self._limits = (
            abs(float(limits.get("max_vx", 1.0))),
            abs(float(limits.get("max_vy", 1.0))),
            abs(float(limits.get("max_vyaw", 1.0))),
        )
        self._stream = RepeatingCommand(
            self._publish_payload,
            self._publish_zero,
            rate_hz=float(limits.get("velocity_rate_hz", 100)),
        )
        self._watchdog_period = float(limits.get("stream_watchdog_period_sec", 0.5))
        self._stream_gap_limit_sec = max(
            5.0 / float(limits.get("velocity_rate_hz", 100)), 0.5
        )

    def get_tool(self) -> dict:
        return {
            "name": "loco",
            "type": "actuator",
            "multiInstance": False,
            "description": "T800 全向速度控制，支持定时和持续运动",
            "inputSchema": action_schema(
                {
                    "move": (["vx", "vy", "vyaw", "duration", "force"], "按速度移动；duration=-1 持续到 stop_move"),
                    "move_displacement": (["x_m", "y_m", "speed_m_s", "force"], "按时间积分估算相对位移（开环，无里程计反馈）"),
                    "turn_angle": (["angle_rad", "angular_speed_rad_s", "force"], "按时间积分估算原地转角（开环）"),
                    "arc": (["radius_m", "angle_rad", "linear_speed_m_s", "force"], "按给定半径和角度走圆弧（开环）"),
                    "stop_move": ([], "立即发布零速度并停止刷新"),
                    "status": ([], "查询速度控制刷新状态"),
                },
                {
                    "vx": {"type": "number", "description": "前向速度 m/s"},
                    "vy": {"type": "number", "description": "侧向速度 m/s"},
                    "vyaw": {"type": "number", "description": "偏航角速度 rad/s"},
                    "duration": {"type": "number", "description": "秒；-1=持续，0=停止"},
                    "force": {
                        "type": "boolean",
                        "description": "忽略必须处于 rl_basic/lower_body_balance 的状态门禁",
                    },
                    "x_m": {"type": "number", "description": "机身坐标系前向位移，米"},
                    "y_m": {"type": "number", "description": "机身坐标系侧向位移，米"},
                    "speed_m_s": {"type": "number", "description": "平移速度绝对值，m/s"},
                    "angle_rad": {"type": "number", "description": "偏航角或圆弧夹角，rad"},
                    "angular_speed_rad_s": {"type": "number", "description": "角速度绝对值，rad/s"},
                    "radius_m": {"type": "number", "description": "圆弧半径绝对值，米"},
                    "linear_speed_m_s": {"type": "number", "description": "圆弧线速度，m/s；负数为后退"},
                },
                "运动动作",
                completion=(["move"], 60),
            ),
        }

    def start(self) -> None:
        from interface_protocol.msg import BodyVelCmd

        self._message_type = BodyVelCmd
        self._publisher = self._node.create_publisher(
            BodyVelCmd, self._config["topics"]["body_velocity"], _RELIABLE
        )
        self._watchdog_timer = self._node.create_timer(
            self._watchdog_period, self._stream_health_check
        )

    def _stream_health_check(self):
        """流发布看门狗：active 且超过 gap 上限无发布 → 停流归零"""
        s = self._stream.snapshot()
        if s.active and s.last_publish_at is not None:
            gap = time.monotonic() - s.last_publish_at
            if gap > self._stream_gap_limit_sec:
                self._stream.stop()
                self._publish_zero()

    def stop(self) -> None:
        self._stream.stop()

    def dispatch(self, action: str, args: dict) -> dict:
        try:
            return self._dispatch(action, args)
        except Exception:
            self._stream.stop()
            self._publish_zero()
            raise

    def _dispatch(self, action: str, args: dict) -> dict:
        if action == "start":
            return {"state": "ready"}
        if action == "stop":
            self.stop()
            return {"state": "idle"}
        if action == "status" or action == "info":
            return {"state": "ready", "stream": asdict(self._stream.snapshot())}
        if action == "stop_move":
            stopped = self._stream.stop()
            self._publish_zero()
            return {"state": "stopped", "was_active": stopped}
        if action not in ("move", "move_displacement", "turn_angle", "arc"):
            return {"error": f"unknown locomotion action: {action}"}

        motion, _ = self._state.current_motion()
        if motion not in WALK_MOTION_STATES and not bool(args.get("force", False)):
            return {
                "error": (
                    f"move requires motion state in {WALK_MOTION_STATES} "
                    f"(current: {motion or 'unknown'})"
                )
            }
        open_loop = action != "move"
        if action == "move":
            vx = clamp(args.get("vx", 0), -self._limits[0], self._limits[0])
            vy = clamp(args.get("vy", 0), -self._limits[1], self._limits[1])
            vyaw = clamp(args.get("vyaw", 0), -self._limits[2], self._limits[2])
            duration = float(args.get("duration", 1.0))
        elif action == "move_displacement":
            x_m = float(args.get("x_m", 0.0))
            y_m = float(args.get("y_m", 0.0))
            distance = math.hypot(x_m, y_m)
            if not math.isfinite(distance) or distance == 0:
                return {"error": "x_m and y_m must define a non-zero finite displacement"}
            speed = clamp(abs(args.get("speed_m_s", 0.3)), 0.01, math.hypot(*self._limits[:2]))
            duration = max(
                distance / speed,
                abs(x_m) / self._limits[0] if x_m else 0.0,
                abs(y_m) / self._limits[1] if y_m else 0.0,
            )
            vx = x_m / duration
            vy = y_m / duration
            vyaw = 0.0
        elif action == "turn_angle":
            angle = float(args.get("angle_rad", 0.0))
            if not math.isfinite(angle) or angle == 0:
                return {"error": "angle_rad must be non-zero and finite"}
            speed = clamp(abs(args.get("angular_speed_rad_s", 0.5)), 0.01, self._limits[2])
            vyaw = math.copysign(speed, angle)
            duration = abs(angle) / speed
            vx = vy = 0.0
        else:
            radius = abs(float(args.get("radius_m", 0.0)))
            angle = float(args.get("angle_rad", 0.0))
            linear = float(args.get("linear_speed_m_s", 0.3))
            if not all(math.isfinite(value) for value in (radius, angle, linear)) or radius <= 0 or angle == 0 or linear == 0:
                return {"error": "radius_m, angle_rad and linear_speed_m_s must be finite and non-zero"}
            requested_vx = clamp(linear, -self._limits[0], self._limits[0])
            angular_speed = min(abs(requested_vx) / radius, self._limits[2])
            vx = math.copysign(angular_speed * radius, requested_vx)
            vyaw = math.copysign(angular_speed, angle)
            duration = abs(angle) / angular_speed
            vy = 0.0
        snapshot = self._stream.start({"vx": vx, "vy": vy, "vyaw": vyaw}, duration)
        return {"state": "running" if duration else "stopped", "vx": vx, "vy": vy, "vyaw": vyaw,
                "duration": duration, "open_loop": open_loop, "stream": asdict(snapshot)}

    def _publish_payload(self, payload: dict) -> None:
        if self._publisher is None:
            raise RuntimeError("locomotion publisher is not initialized")
        msg = self._message_type()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.header.frame_id = "body"
        msg.linear_velocity = [payload["vx"], payload["vy"]]
        msg.yaw_velocity = payload["vyaw"]
        self._publisher.publish(msg)

    def _publish_zero(self) -> None:
        if self._publisher is not None:
            self._publish_payload({"vx": 0.0, "vy": 0.0, "vyaw": 0.0})


class MotionModePlugin:
    _SHORTCUTS = {
        "idle": "idle",
        "passive": "passive",
        "stand": "pd_stand",
        "walk": "rl_basic",
        "dance": "dance",
        "get_up": "rl_mimic_supine_to_stance",
        "lie_down": "rl_mimic_stance_to_supine",
    }
    def __init__(self, config: dict, namespace: str, ros2, state: StatePlugin):
        self._config = config
        self._state = state
        self._node = Node("t800_motion_mode", context=ros2.ctx_robot)
        ros2.executor_robot.add_node(self._node)
        self._publisher = None

    def get_tool(self) -> dict:
        return {
            "name": "motion_mode",
            "type": "actuator",
            "multiInstance": False,
            "description": "T800 运动状态机切换，包含站立、行走、舞蹈、起身、躺下及桥接模式",
            "inputSchema": action_schema(
                {"switch": (["target", "force", "wait"], "请求切换到目标 Native SDK motion state"),
                 **{name: (["force", "wait"], f"快捷切换到 {target}") for name, target in self._SHORTCUTS.items()},
                 "status": ([], "查询当前和可转换状态")},
                {
                    "target": {"type": "string", "description": "目标 motion state；支持固件返回的自定义状态名"},
                    "force": {"type": "boolean", "description": "目标不在 available transitions 时仍发送"},
                    "wait": {"type": "boolean", "description": "等待状态反馈，默认 true"},
                },
                "状态机动作",
            ),
        }

    def start(self) -> None:
        from interface_protocol.msg import MotionStateRequest

        self._message_type = MotionStateRequest
        self._publisher = self._node.create_publisher(
            MotionStateRequest, self._config["topics"]["motion_request"], _RELIABLE_ONE
        )

    def stop(self) -> None:
        pass

    def dispatch(self, action: str, args: dict) -> dict:
        current, available = self._state.current_motion()
        if action in ("start", "info", "status"):
            return {"state": "ready", "current": current, "available": available}
        if action == "stop":
            return {"state": "idle"}
        if action in self._SHORTCUTS:
            args = dict(args)
            args["target"] = self._SHORTCUTS[action]
            action = "switch"
        if action != "switch":
            return {"error": f"unknown motion mode action: {action}"}
        target = str(args.get("target", ""))
        if not target:
            return {"error": "target motion is required"}
        if available and target not in available and not bool(args.get("force", False)):
            return {"error": f"{target} is not available from {current}", "available": available}
        msg = self._message_type()
        msg.target_motion_name = target
        self._publisher.publish(msg)
        if not bool(args.get("wait", True)):
            return {"state": "requested", "target": target, "previous": current}
        deadline = time.monotonic() + float(self._config["control"]["mode_transition_timeout_sec"])
        while time.monotonic() < deadline:
            current, available = self._state.current_motion()
            if current == target:
                return {"state": "completed", "current": current, "available": available}
            time.sleep(0.05)
        return {"state": "timeout", "target": target, "current": current, "available": available}


class DancePlugin:
    """Discoverable dance facade over Native SDK motion states."""

    def __init__(self, motion_mode: MotionModePlugin, state: StatePlugin):
        self._motion_mode = motion_mode
        self._state = state

    def get_tool(self) -> dict:
        return {
            "name": "dance",
            "type": "actuator",
            "multiInstance": False,
            "description": "T800 整机舞蹈发现、播放、停止和状态；官方公开基线内置一套 dance 策略/轨迹",
            "inputSchema": action_schema(
                {
                    "list": ([], "列出官方内置和固件动态发现的舞蹈 motion states"),
                    "play": (["name", "force", "wait"], "播放指定舞蹈，默认 dance"),
                    "stop_dance": (["target", "force", "wait"], "停止舞蹈并切换到 walk 或指定状态"),
                    "status": ([], "查询当前是否处于舞蹈状态"),
                },
                {
                    "name": {"type": "string", "description": "舞蹈 motion state 名，默认 dance"},
                    "target": {"type": "string", "description": "停止后的状态，默认 walk"},
                    "force": {"type": "boolean"},
                    "wait": {"type": "boolean"},
                },
                "舞蹈动作",
            ),
        }

    def start(self) -> None:
        pass

    def stop(self) -> None:
        pass

    def dispatch(self, action: str, args: dict) -> dict:
        current, available = self._state.current_motion()
        detected = sorted({name for name in available if "dance" in name.lower()} | {"dance"})
        if action in ("start", "info", "list"):
            return {
                "state": "ready",
                "dances": detected,
                "built_in": [{"name": "dance", "policy": "dance.mnn", "trajectory": "dance.npz"}],
                "selector_available": len(detected) > 1,
            }
        if action == "status":
            return {"state": "playing" if "dance" in current.lower() else "idle", "current": current,
                    "dances": detected}
        if action == "stop":
            return {"state": "idle"}
        if action == "play":
            target = str(args.get("name", "dance"))
        elif action == "stop_dance":
            target = str(args.get("target", "rl_basic"))
        else:
            return {"error": f"unknown dance action: {action}"}
        forwarded = dict(args)
        forwarded["target"] = target
        return self._motion_mode.dispatch("switch", forwarded)


class JointPlanPlugin:
    _LIMIT_MARGIN_RAD = 0.0
    _PRESETS = {
        "shake_hand": {
            "indices": list(range(12, 25)),
            "positions": [0.0, 0.024, 0.081, -0.001, -0.069, 0.0, -0.47, 0.255, 0.161, -0.731, 0.028, 0.0, 0.0],
            "duration": 2.0,
        },
        "wave_hands": {
            "indices": list(range(12, 25)),
            "positions": [0.0, -1.29568, 1.17971, 0.0757227, -1.06603, -0.0989933,
                          -0.0211716, -0.322156, 0.0440607, -0.0871668, 0.0196457, 0.0, 0.0],
            "duration": 2.0,
        },
    }

    def __init__(self, config: dict, namespace: str, ros2, state: StatePlugin | None = None):
        self._config = config
        self._ns = namespace
        self._sub_node = Node("t800_joint_plan_state", context=ros2.ctx_robot)
        self._pub_node = Node("t800_joint_plan_core", context=ros2.ctx_core)
        ros2.executor_robot.add_node(self._sub_node)
        ros2.executor_core.add_node(self._pub_node)
        self._publisher = None
        self._state_lock = threading.RLock()
        self._state_changed = threading.Condition(self._state_lock)
        self._last_state = {"state": "no_data"}
        self._executing_requests: set[int] = set()
        self._request_id = 0
        self._state_type = None
        self._state = state
        self._core_topic = f"/{namespace}/state/joint_plan"
        self._core_pub = self._pub_node.create_publisher(String, self._core_topic, _BEST_EFFORT)

    def get_tools(self) -> list[dict]:
        return [
            {
                "name": "joint_plan",
                "type": "actuator",
                "multiInstance": False,
                "description": "T800 任意关节轨迹规划、取消、复位与官方预置动作",
                "inputSchema": action_schema(
                    {
                        "plan": (["joint_indices", "target_positions", "target_velocities", "duration",
                                  "stiffness", "damping", "gravity_compensation"], "规划并执行任意关节目标"),
                        "plan_named": (["joint_names", "target_positions", "target_velocities", "duration",
                                        "stiffness", "damping", "gravity_compensation"], "按关节名称规划动作"),
                        "head_pose": (["pitch_rad", "yaw_rad", "duration"], "控制头部俯仰和偏航"),
                        "arm_pose": (["side", "target_positions", "duration"], "控制左臂或右臂 5 个关节"),
                        "hold_current": (["duration"], "以当前 25 关节位置创建保持规划"),
                        "cancel": (["request_id"], "取消指定或最近的关节规划"),
                        "reset": ([], "复位到默认姿态"),
                        "preset": (["preset"], "执行官方 T800 上肢预置动作"),
                        "status": ([], "查询规划器状态与进度"),
                    },
                    {
                        "joint_indices": array_property("关节索引 0..24", item_type="integer"),
                        "joint_names": array_property("T800 关节名称", item_type="string"),
                        "target_positions": array_property("目标弧度，与 joint_indices 等长"),
                        "target_velocities": array_property("目标速度，可留空"),
                        "duration": {"type": "number", "description": "执行时间，秒"},
                        "stiffness": array_property("刚度，可留空"),
                        "damping": array_property("阻尼，可留空"),
                        "gravity_compensation": {"type": "boolean"},
                        "request_id": {"type": "integer"},
                        "preset": {"type": "string", "enum": list(self._PRESETS)},
                        "pitch_rad": {"type": "number"},
                        "yaw_rad": {"type": "number"},
                        "side": {"type": "string", "enum": ["left", "right"]},
                    },
                    "关节规划动作",
                ),
            },
            sensor_tool("joint_plan_state", "T800 关节规划器 request、状态与进度", self._core_topic, "data/json"),
        ]

    def start(self) -> None:
        from interface_protocol.msg import JointMotionPlanRequest, JointMotionPlanState

        self._request_type = JointMotionPlanRequest
        self._state_type = JointMotionPlanState
        self._publisher = self._sub_node.create_publisher(
            JointMotionPlanRequest, self._config["topics"]["joint_plan_request"], _RELIABLE_ONE
        )
        self._sub_node.create_subscription(
            JointMotionPlanState, self._config["topics"]["joint_plan_state"], self._on_state, _BEST_EFFORT
        )

    def stop(self) -> None:
        pass

    def dispatch(self, action: str, args: dict) -> dict:
        if action == "start":
            return {"state": "running" if args.get("_tool_name") == "joint_plan_state" else "ready"}
        if action in ("info", "status", "joint_plan_state"):
            with self._state_lock:
                snapshot = dict(self._last_state)
            if args.get("_tool_name") == "joint_plan_state" or action == "joint_plan_state":
                return _with_topic_out(snapshot, self._core_topic)
            return snapshot
        if action == "stop":
            return {"state": "idle"}
        if action == "reset":
            return self._publish_request("reset", {})
        if action == "cancel":
            return self._publish_request("cancel", args)
        if action == "preset":
            preset = self._PRESETS.get(str(args.get("preset", "")))
            if preset is None:
                return {"error": "unknown joint preset"}
            args = {
                "joint_indices": preset["indices"],
                "target_positions": preset["positions"],
                "duration": preset["duration"],
                "gravity_compensation": True,
            }
            return self._publish_request("plan", args)
        if action == "plan_named":
            names = args.get("joint_names")
            if not isinstance(names, (list, tuple)) or not names:
                return {"error": "joint_names must be a non-empty array"}
            unknown = [str(name) for name in names if str(name) not in T800_JOINT_INDEX]
            if unknown:
                return {"error": f"unknown joint names: {unknown}"}
            args = dict(args)
            args["joint_indices"] = [T800_JOINT_INDEX[str(name)] for name in names]
            return self._publish_request("plan", args)
        if action == "head_pose":
            return self._publish_request("plan", {
                "joint_indices": list(T800_JOINT_GROUPS["head"]),
                "target_positions": [float(args.get("pitch_rad", 0.0)), float(args.get("yaw_rad", 0.0))],
                "duration": args.get("duration", 1.0),
                "gravity_compensation": True,
            })
        if action == "arm_pose":
            side = str(args.get("side", ""))
            if side not in ("left", "right"):
                return {"error": "side must be left or right"}
            return self._publish_request("plan", {
                "joint_indices": list(T800_JOINT_GROUPS[f"{side}_arm"]),
                "target_positions": args.get("target_positions"),
                "duration": args.get("duration", 1.5),
                "gravity_compensation": True,
            })
        if action == "hold_current":
            if self._state is None:
                return {"error": "joint state is unavailable"}
            return self._publish_request("plan", {
                "joint_indices": list(T800_JOINT_GROUPS["all"]),
                "target_positions": self._state.joint_positions(),
                "duration": args.get("duration", 0.5),
                "gravity_compensation": True,
            })
        if action == "plan":
            return self._publish_request("plan", args)
        return {"error": f"unknown joint plan action: {action}"}

    def current_motion(self) -> tuple[str, list[str]]:
        if self._state is None:
            return "", []
        return self._state.current_motion()

    def wait_until_idle(
        self,
        timeout: float,
        cancel_event: threading.Event,
        *,
        minimum_request_id: int | None = None,
    ) -> dict:
        """Wait for an IDLE planner state without inventing time-based progress."""
        if self._state_type is None:
            raise RuntimeError("joint planner is not started")
        deadline = time.monotonic() + float(timeout)
        idle = int(self._state_type.IDLE)
        with self._state_changed:
            while True:
                if cancel_event.is_set():
                    raise RuntimeError("gesture cancelled")
                state = dict(self._last_state)
                request_id = state.get("request_id")
                # The official planner may not publish until it receives its
                # first request. Allow that first request, then require an
                # observed state for every subsequent transition.
                if request_id is None and minimum_request_id is None:
                    return state
                if (
                    request_id is not None
                    and int(state.get("status", -1)) == idle
                    and (minimum_request_id is None or int(request_id) >= minimum_request_id)
                ):
                    return state
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise TimeoutError("joint planner did not become IDLE")
                self._state_changed.wait(timeout=min(remaining, 0.2))

    def wait_for_request(
        self,
        request_id: int,
        timeout: float,
        cancel_event: threading.Event,
    ) -> dict:
        """Require the planner to execute and finish the exact request."""
        if self._state_type is None:
            raise RuntimeError("joint planner is not started")
        target = int(request_id)
        deadline = time.monotonic() + float(timeout)
        idle = int(self._state_type.IDLE)
        with self._state_changed:
            while True:
                if cancel_event.is_set():
                    raise RuntimeError("gesture cancelled")
                state = dict(self._last_state)
                current_id = state.get("request_id")
                status = int(state.get("status", -1))
                if (
                    current_id is not None
                    and int(current_id) == target
                    and target in self._executing_requests
                    and status == idle
                ):
                    self._executing_requests.discard(target)
                    return state
                if current_id is not None and int(current_id) > target:
                    raise RuntimeError(f"joint planner request {target} was superseded")
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise TimeoutError(f"joint planner did not complete request {target}")
                self._state_changed.wait(timeout=min(remaining, 0.2))

    def _next_request_id(self) -> int:
        with self._state_lock:
            self._request_id += 1
            return self._request_id

    def _publish_request(self, action: str, args: dict) -> dict:
        msg = self._request_type()
        if action == "cancel":
            msg.request_id = int(args.get("request_id", self._request_id))
            msg.request_type = self._request_type.REQUEST_CANCEL
        else:
            msg.request_id = self._next_request_id()
            msg.request_type = (
                self._request_type.REQUEST_RESET if action == "reset" else self._request_type.REQUEST_PLAN_EXECUTE
            )
        if action == "plan":
            indices, positions = validate_joint_positions(
                args.get("joint_indices"),
                args.get("target_positions"),
                limit_margin_rad=self._LIMIT_MARGIN_RAD,
            )
            velocities = optional_floats(args, "target_velocities", len(indices))
            stiffness = optional_floats(args, "stiffness", len(indices))
            damping = optional_floats(args, "damping", len(indices))
            msg.use_gravity_compensation = bool(args.get("gravity_compensation", True))
            msg.joint_indices = indices
            msg.target_positions = positions
            msg.target_velocities = velocities
            msg.execution_time = clamp(args.get("duration", 2.0), 0.05, 120.0)
            msg.stiffness = stiffness
            msg.damping = damping
        else:
            msg.use_gravity_compensation = False
            msg.joint_indices = []
            msg.target_positions = []
            msg.target_velocities = []
            msg.execution_time = 0.0
            msg.stiffness = []
            msg.damping = []
        self._publisher.publish(msg)
        return {"state": "requested", "request_id": msg.request_id, "request_type": int(msg.request_type)}

    def _on_state(self, msg) -> None:
        payload = {
            "request_id": int(msg.request_id),
            "status": int(msg.status),
            "progress": float(msg.progress),
            "timestamp_ms": _now_ms(),
        }
        with self._state_changed:
            self._request_id = max(self._request_id, int(msg.request_id))
            self._last_state = payload
            if self._state_type is not None and int(msg.status) == int(self._state_type.EXECUTING):
                self._executing_requests.add(int(msg.request_id))
            self._state_changed.notify_all()
        self._core_pub.publish(_json_message(payload))


class GesturePlugin:
    """Planner-synchronised upper-body gestures validated against URDF limits."""

    _INDICES = list(range(12, 25))
    _LIMIT_MARGIN_RAD = 0.02
    _READY_TIMEOUT_SEC = 10.0
    _STEP_TIMEOUT_SEC = 15.0
    _COOLDOWN_SEC = 3.0
    _ACP_TIMEOUT_SEC = 300.0
    _ACP_CALLBACK_TIMEOUT_SEC = 5.0
    _ACP_SAFETY_MARGIN_SEC = 5.0
    _MAX_SEQUENCE_STEPS = 16
    _NEUTRAL = [0.0, 0.028, 0.084, -0.001, -0.066, 0.0, 0.024, -0.081, 0.001, -0.069, 0.0, 0.0, 0.0]
    _RAISED = [0.0, -1.29568, 1.17971, 0.0757227, -1.06603, -0.0989933,
               -0.0211716, -0.322156, 0.0440607, -0.0871668, 0.0196457, 0.0, 0.0]
    _WAVE = [0.0, -1.07786, 1.13928, 0.177577, -1.83356, -0.0875483,
             -0.0211716, -0.322156, 0.0440607, -0.0871668, 0.0196457, 0.0, 0.0]
    _HAND_EXTENDED = [0.0, 0.024, 0.081, -0.001, -0.069, 0.0,
                      -0.47, 0.255, 0.161, -0.731, 0.028, 0.0, 0.0]
    _HAND_WITHDRAWN = [0.0, 0.024, 0.081, -0.001, -0.069, 0.0,
                       0.028, -0.084, 0.001, -0.066, 0.0, 0.0, 0.0]
    _SHAKE_STIFFNESS = [400.0, 40.0, 40.0, 20.0, 40.0, 20.0,
                        40.0, 40.0, 20.0, 40.0, 20.0, 100.0, 100.0]
    _SHAKE_DAMPING = [3.0, 1.0, 1.0, 1.0, 1.0, 1.0,
                      1.0, 1.0, 1.0, 1.0, 1.0, 3.0, 3.0]
    _GESTURES = {
        "wave_hands": [
            {"name": "neutral", "positions": _NEUTRAL, "duration": 1.0},
            {"name": "raise_hand", "positions": _RAISED, "duration": 2.0},
            {"name": "wave_left", "positions": _WAVE, "duration": 0.3},
            {"name": "wave_right", "positions": _RAISED, "duration": 0.3},
            {"name": "wave_left_again", "positions": _WAVE, "duration": 0.3},
            {"name": "wave_right_again", "positions": _RAISED, "duration": 0.3},
            {"name": "wave_finish", "positions": _WAVE, "duration": 0.3},
            {"name": "return_to_neutral", "positions": _NEUTRAL, "duration": 2.0},
        ],
        "shake_hand": [
            {
                "name": "extend_right_hand",
                "positions": _HAND_EXTENDED,
                "duration": 2.0,
                "hold_after_sec": 2.0,
                "stiffness": _SHAKE_STIFFNESS,
                "damping": _SHAKE_DAMPING,
            },
            {
                "name": "withdraw_right_hand",
                "positions": _HAND_WITHDRAWN,
                "duration": 2.0,
                "stiffness": _SHAKE_STIFFNESS,
                "damping": _SHAKE_DAMPING,
            },
        ],
    }

    def __init__(self, joint_plan: JointPlanPlugin):
        self._joint_plan = joint_plan
        self._lock = threading.Lock()
        self._cancel = threading.Event()
        self._thread: threading.Thread | None = None
        self._last_finished_at: float | None = None
        self._status = {
            "state": "idle", "gesture": None, "step": 0, "step_name": None,
            "total_steps": 0, "request_id": None, "error": None,
        }

    def get_tool(self) -> dict:
        actions = _with_lifecycle({
            "list": ([], "列出内置手势及步数"),
            "play": (["name", "repetitions", "reset_after", "force"], "异步播放一次实机验证手势"),
            "sequence": (["steps", "reset_after", "force"], "异步执行经过关节限位校验的自定义序列"),
            "stop_gesture": (["reset_after"], "取消当前步骤并停止手势"),
            "status": ([], "查询手势、步骤和错误"),
        })
        actions["stop"] = (["reset_after"], "停止当前手势并进入空闲状态")
        schema = action_schema(
            actions,
            {
                "name": {"type": "string", "enum": list(self._GESTURES)},
                "repetitions": {"type": "integer", "minimum": 1, "maximum": 1,
                                "description": "安全限制：内置手势每次只执行一次"},
                "reset_after": {"type": "boolean"},
                "force": {"type": "boolean", "description": "忽略 lower_body_balance 状态门禁"},
                "steps": {
                    "type": "array",
                    "description": "每步支持 joint_indices 或 joint_names、target_positions、duration、hold_after_sec、stiffness、damping",
                    "maxItems": self._MAX_SEQUENCE_STEPS,
                    "items": {"type": "object"},
                },
            },
            "手势动作",
        )
        schema["x-completion"] = {
            "actions": ["play", "sequence"],
            "timeout": int(self._ACP_TIMEOUT_SEC),
        }
        return {
            "name": "gesture",
            "type": "actuator",
            "multiInstance": False,
            "description": "T800 完整多步手势编排；内置官方挥手和握手序列，也支持任意关节动作队列",
            "inputSchema": schema,
        }

    def start(self) -> None:
        pass

    def stop(self) -> None:
        self._stop(reset_after=False)

    def dispatch(self, action: str, args: dict) -> dict:
        if action in ("start", "info", "list"):
            return {
                "state": "ready",
                "gestures": [
                    {"name": name, "steps": len(steps), "source": "T800 real-device validated trajectory"}
                    for name, steps in self._GESTURES.items()
                ],
                "custom_sequence": True,
                "safety": {
                    "required_motion_state": "lower_body_balance",
                    "position_limit_margin_rad": self._LIMIT_MARGIN_RAD,
                    "planner_state_synchronised": True,
                    "cooldown_sec": self._COOLDOWN_SEC,
                    "maximum_repetitions": 1,
                    "maximum_sequence_steps": self._MAX_SEQUENCE_STEPS,
                    "acp_timeout_sec": self._ACP_TIMEOUT_SEC,
                },
            }
        if action == "status":
            with self._lock:
                result = dict(self._status)
                if self._last_finished_at is None:
                    result["cooldown_remaining_sec"] = 0.0
                else:
                    result["cooldown_remaining_sec"] = max(
                        0.0, self._COOLDOWN_SEC - (time.monotonic() - self._last_finished_at)
                    )
                return result
        if action == "stop":
            self._stop(reset_after=bool(args.get("reset_after", False)))
            return {"state": "idle"}
        if action == "stop_gesture":
            return self._stop(reset_after=bool(args.get("reset_after", False)))
        if action in ("play", "sequence") and bool(args.get("wait", False)):
            return {
                "error": "wait=true is not supported for asynchronous gesture actions; "
                         "use action_id completion or status instead"
            }
        if action == "play":
            name = str(args.get("name", ""))
            if name not in self._GESTURES:
                return {"error": f"unknown gesture: {name}"}
            repetitions = int(args.get("repetitions", 1))
            if repetitions != 1:
                return {"error": "repetitions must be 1 for thermal safety"}
            with self._lock:
                if (
                    self._last_finished_at is not None
                    and time.monotonic() - self._last_finished_at < self._COOLDOWN_SEC
                ):
                    return {"error": "gesture cooldown is active"}
            steps = self._official_steps(name)
            label = name
        elif action == "sequence":
            steps = args.get("steps")
            if not isinstance(steps, list) or not steps:
                return {"error": "steps must be a non-empty array"}
            if len(steps) > self._MAX_SEQUENCE_STEPS:
                return {"error": f"steps cannot contain more than {self._MAX_SEQUENCE_STEPS} items"}
            if any(not isinstance(step, dict) for step in steps):
                return {"error": "every gesture step must be an object"}
            steps = [dict(step) for step in steps]
            label = "custom"
        else:
            return {"error": f"unknown gesture action: {action}"}
        motion, _available_motions = self._joint_plan.current_motion()
        if motion != "lower_body_balance" and not bool(args.get("force", False)):
            return {
                "error": "gesture requires motion state 'lower_body_balance' "
                         f"(current: {motion or 'unknown'})"
            }
        reset_after = bool(args.get("reset_after", True))
        try:
            steps = self._prepare_steps(steps)
            self._validate_completion_budget(steps, reset_after=reset_after)
        except (TypeError, ValueError) as exc:
            return {"error": str(exc)}
        return self._start_sequence(
            label,
            steps,
            reset_after=reset_after,
        )

    def _official_steps(self, name: str) -> list[dict]:
        steps = []
        for definition in self._GESTURES[name]:
            step = {
                "name": definition["name"],
                "joint_indices": list(self._INDICES),
                "target_positions": list(definition["positions"]),
                "duration": definition["duration"],
                "hold_after_sec": definition.get("hold_after_sec", 0.0),
                "stiffness": list(definition.get("stiffness", [])),
                "damping": list(definition.get("damping", [])),
                "gravity_compensation": True,
            }
            steps.append(step)
        return steps

    def _prepare_steps(self, steps: list[dict]) -> list[dict]:
        prepared = []
        for offset, source in enumerate(steps, start=1):
            step = dict(source)
            if "joint_names" in step:
                names = step.get("joint_names")
                if not isinstance(names, (list, tuple)) or not names:
                    raise ValueError("joint_names must be a non-empty array")
                unknown = [str(name) for name in names if str(name) not in T800_JOINT_INDEX]
                if unknown:
                    raise ValueError(f"unknown joint names: {unknown}")
                indices = [T800_JOINT_INDEX[str(name)] for name in names]
            else:
                indices = step.get("joint_indices")
            validated_indices, positions = validate_joint_positions(
                indices,
                step.get("target_positions"),
                limit_margin_rad=self._LIMIT_MARGIN_RAD,
            )
            duration = float(step.get("duration", 2.0))
            if not math.isfinite(duration) or not 0.05 <= duration <= 120.0:
                raise ValueError(f"gesture step {offset} duration must be between 0.05 and 120 seconds")
            hold_after = float(step.get("hold_after_sec", 0.0))
            if not math.isfinite(hold_after) or not 0.0 <= hold_after <= 30.0:
                raise ValueError(f"gesture step {offset} hold_after_sec must be between 0 and 30 seconds")
            count = len(validated_indices)
            step["target_positions"] = positions
            step["duration"] = duration
            step["hold_after_sec"] = hold_after
            step["stiffness"] = optional_floats(step, "stiffness", count)
            step["damping"] = optional_floats(step, "damping", count)
            step["gravity_compensation"] = bool(step.get("gravity_compensation", True))
            step["name"] = str(step.get("name", f"step_{offset}"))
            if "joint_names" not in step:
                step["joint_indices"] = validated_indices
            prepared.append(step)
        return prepared

    def _validate_completion_budget(self, steps: list[dict], *, reset_after: bool) -> None:
        worst_case = self._READY_TIMEOUT_SEC + self._ACP_CALLBACK_TIMEOUT_SEC
        worst_case += self._ACP_SAFETY_MARGIN_SEC
        for step in steps:
            worst_case += max(
                self._STEP_TIMEOUT_SEC,
                float(step["duration"]) + 5.0,
            )
            worst_case += float(step.get("hold_after_sec", 0.0))
        if reset_after:
            worst_case += self._STEP_TIMEOUT_SEC
        if worst_case > self._ACP_TIMEOUT_SEC:
            raise ValueError(
                f"gesture worst-case runtime {worst_case:.1f}s exceeds "
                f"ACP completion timeout {self._ACP_TIMEOUT_SEC:.0f}s"
            )

    def _start_sequence(self, label: str, steps: list[dict], *, reset_after: bool) -> dict:
        with self._lock:
            if self._thread is not None and self._thread.is_alive():
                current = dict(self._status)
                current.pop("action_id", None)
                return {**current, "error": "another gesture sequence is already running"}
            from uuid import uuid4

            action_id = f"t800_gesture_{uuid4().hex[:12]}"
            self._cancel = threading.Event()
            self._status = {
                "state": "running", "gesture": label, "step": 0,
                "step_name": None, "total_steps": len(steps),
                "request_id": None, "action_id": action_id, "error": None,
            }

        def run() -> None:
            request_id = None
            try:
                self._joint_plan.wait_until_idle(
                    self._READY_TIMEOUT_SEC, self._cancel
                )
                for index, step in enumerate(steps, start=1):
                    if self._cancel.is_set():
                        raise RuntimeError("gesture cancelled")
                    with self._lock:
                        self._status["step"] = index
                        self._status["step_name"] = step["name"]
                    action = "plan_named" if "joint_names" in step else "plan"
                    result = self._joint_plan.dispatch(action, step)
                    if "error" in result:
                        raise ValueError(result["error"])
                    request_id = int(result["request_id"])
                    with self._lock:
                        self._status["request_id"] = request_id
                    self._joint_plan.wait_for_request(
                        request_id,
                        max(self._STEP_TIMEOUT_SEC, float(step["duration"]) + 5.0),
                        self._cancel,
                    )
                    hold_after = float(step.get("hold_after_sec", 0.0))
                    if hold_after > 0 and self._cancel.wait(hold_after):
                        raise RuntimeError("gesture cancelled")
                if not self._cancel.is_set() and reset_after:
                    result = self._joint_plan.dispatch("reset", {})
                    if "error" in result:
                        raise ValueError(result["error"])
                    request_id = int(result["request_id"])
                    with self._lock:
                        self._status["request_id"] = request_id
                        self._status["step_name"] = "reset"
                    self._joint_plan.wait_for_request(
                        request_id,
                        self._STEP_TIMEOUT_SEC,
                        self._cancel,
                    )
                with self._lock:
                    if self._cancel.is_set() or self._status.get("state") == "cancelled":
                        self._status["state"] = "cancelled"
                        self._status["error"] = ""
                    else:
                        self._status["state"] = "completed"
            except Exception as exc:
                cancelled = self._cancel.is_set()
                if request_id is not None:
                    self._joint_plan.dispatch("cancel", {"request_id": request_id})
                with self._lock:
                    self._status["state"] = "cancelled" if cancelled else "error"
                    self._status["error"] = "" if cancelled else str(exc)
            finally:
                with self._lock:
                    self._last_finished_at = time.monotonic()
                    final_status = str(self._status.get("state", "error"))
                    final_result = {
                        "gesture": label,
                        "step": self._status.get("step"),
                        "step_name": self._status.get("step_name"),
                        "total_steps": self._status.get("total_steps"),
                        "request_id": self._status.get("request_id"),
                        "error": self._status.get("error"),
                    }
                if action_id is not None:
                    self._acp_notify(action_id, final_status, final_result)

        thread = threading.Thread(target=run, daemon=True, name="t800-gesture-sequence")
        with self._lock:
            self._thread = thread
        thread.start()
        return {
            "state": "running",
            "gesture": label,
            "total_steps": len(steps),
            "action_id": action_id,
        }

    def _stop(self, *, reset_after: bool) -> dict:
        with self._lock:
            active = (
                self._status.get("state") == "running"
                and self._thread is not None
                and self._thread.is_alive()
            )
            if active:
                self._cancel.set()
                self._status["state"] = "cancelled"
                self._status["error"] = ""
                request_id = self._status.get("request_id")
            else:
                request_id = None
            result = dict(self._status)
        if request_id is not None:
            self._joint_plan.dispatch("cancel", {"request_id": request_id})
        if reset_after:
            self._joint_plan.dispatch("reset", {})
        return result

    @staticmethod
    def _acp_notify(action_id: str, status: str, result: dict) -> None:
        """Report asynchronous gesture completion to Agent Core."""
        import json as _json
        import os as _os
        import ssl as _ssl
        import urllib.parse as _urlparse
        import urllib.request as _urllib

        agent_core_url = _os.environ.get("AGENT_CORE_URL", "https://localhost:15678")
        ca_cert = _os.environ.get("AGENT_CORE_CA_CERT")
        try:
            parsed_url = _urlparse.urlparse(agent_core_url)
            if parsed_url.scheme not in ("http", "https"):
                raise ValueError("AGENT_CORE_URL must use http or https")
            if (
                parsed_url.scheme == "http"
                and parsed_url.hostname not in ("localhost", "127.0.0.1", "::1")
            ):
                raise ValueError("unencrypted AGENT_CORE_URL is only allowed on loopback")
            context = _ssl.create_default_context(cafile=ca_cert or None)
            payload = _json.dumps({
                "action_id": action_id,
                "status": status,
                "result": result,
                "tool": "gesture",
                "ts": time.time(),
            }).encode()
            request = _urllib.Request(
                f"{agent_core_url}/api/acp/complete",
                data=payload,
                headers={"Content-Type": "application/json"},
                method="POST",
            )
            _urllib.urlopen(
                request,
                timeout=GesturePlugin._ACP_CALLBACK_TIMEOUT_SEC,
                context=context,
            )
        except Exception as exc:
            print(f"[gesture] ACP callback failed for {action_id}: {exc}", flush=True)


class _JointStreamBase:
    def stop(self) -> None:
        self._stream.stop()

    def _status(self) -> dict:
        return {"state": "ready", "stream": asdict(self._stream.snapshot())}


class JointOverridePlugin(_JointStreamBase):
    def __init__(self, config: dict, namespace: str, ros2, state: StatePlugin):
        self._config = config
        self._state = state
        self._node = Node("t800_joint_override", context=ros2.ctx_robot)
        ros2.executor_robot.add_node(self._node)
        self._publisher = None
        self._last_indices: list[int] = []
        self._stream = RepeatingCommand(
            self._publish,
            self._publish_release,
            rate_hz=float(config["control"]["override_rate_hz"]),
        )

    def get_tool(self) -> dict:
        return {
            "name": "joint_override",
            "type": "actuator",
            "description": "T800 特定关节高频覆盖控制；支持持续保持和释放",
            "inputSchema": action_schema(
                {"command": (["joint_indices", "position", "velocity", "feed_forward_torque", "torque",
                              "stiffness", "damping", "weight", "duration", "force"], "以高频流覆盖指定关节"),
                 "release": ([], "释放关节覆盖"), "status": ([], "查询覆盖流状态")},
                {
                    "joint_indices": array_property("关节索引 0..24", item_type="integer"),
                    "position": array_property("目标位置 rad"),
                    "velocity": array_property("目标速度 rad/s"),
                    "feed_forward_torque": array_property("前馈力矩 Nm"),
                    "torque": array_property("附加力矩 Nm"),
                    "stiffness": array_property("刚度"),
                    "damping": array_property("阻尼"),
                    "weight": {"type": "number", "description": "覆盖权重 0..1"},
                    "duration": {"type": "number", "description": "秒；-1 持续"},
                    "force": {"type": "boolean", "description": "忽略 lower_body_balance 状态门禁"},
                },
                "覆盖控制动作",
            ),
        }

    def start(self) -> None:
        from interface_protocol.msg import JointOverrideCommand

        self._message_type = JointOverrideCommand
        self._publisher = self._node.create_publisher(
            JointOverrideCommand, self._config["topics"]["joint_override"], _RELIABLE
        )

    def dispatch(self, action: str, args: dict) -> dict:
        if action in ("start", "info", "status"):
            return self._status()
        if action in ("stop", "release"):
            self.stop()
            self._publish_release()
            return {"state": "released" if action == "release" else "idle"}
        if action != "command":
            return {"error": f"unknown joint override action: {action}"}
        motion, _ = self._state.current_motion()
        if motion != "lower_body_balance" and not bool(args.get("force", False)):
            return {"error": f"joint override requires lower_body_balance (current: {motion or 'unknown'})"}
        indices = validate_joint_indices(args.get("joint_indices"))
        size = len(indices)
        position = float_list(args.get("position"), "position", size=size)
        payload = {
            "indices": indices,
            "position": position,
            "velocity": optional_floats(args, "velocity", size),
            "feed_forward_torque": optional_floats(args, "feed_forward_torque", size),
            "torque": optional_floats(args, "torque", size),
            "stiffness": optional_floats(args, "stiffness", size),
            "damping": optional_floats(args, "damping", size),
            "weight": clamp(args.get("weight", 1.0), 0.0, 1.0),
        }
        self._last_indices = indices
        duration = float(args.get("duration", 1.0))
        return {"state": "running", "stream": asdict(self._stream.start(payload, duration)), "duration": duration}

    def _publish(self, payload: dict) -> None:
        msg = self._message_type()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.weight = payload["weight"]
        msg.joint_indices = payload["indices"]
        size = len(payload["indices"])
        msg.position = payload["position"]
        msg.velocity = list_or_default(payload["velocity"], size)
        msg.feed_forward_torque = list_or_default(payload["feed_forward_torque"], size)
        msg.torque = list_or_default(payload["torque"], size)
        msg.stiffness = list_or_default(payload["stiffness"], size)
        msg.damping = list_or_default(payload["damping"], size)
        self._publisher.publish(msg)

    def _publish_release(self) -> None:
        if self._publisher is None or not self._last_indices:
            return
        size = len(self._last_indices)
        self._publish({"weight": 0.0, "indices": self._last_indices, "position": [0.0] * size,
                       "velocity": [], "feed_forward_torque": [], "torque": [], "stiffness": [], "damping": []})


class JointBridgePlugin(_JointStreamBase):
    def __init__(self, config: dict, namespace: str, ros2, state: StatePlugin):
        self._config = config
        self._state = state
        self._node = Node("t800_joint_bridge", context=ros2.ctx_robot)
        ros2.executor_robot.add_node(self._node)
        self._publisher = None
        self._stream = RepeatingCommand(
            self._publish,
            self._publish_damping,
            rate_hz=float(config["control"]["low_level_rate_hz"]),
        )

    def get_tool(self) -> dict:
        return {
            "name": "joint_bridge",
            "type": "actuator",
            "description": "T800 25DOF 低层关节命令流，最高 500Hz",
            "inputSchema": action_schema(
                {"command": (["position", "velocity", "feed_forward_torque", "torque", "stiffness", "damping",
                              "parallel_parser_type", "duration", "force"], "向全部25关节发送底层命令"),
                 "stop_command": ([], "停止命令流并发送阻尼保持"), "status": ([], "查询底层命令流")},
                {
                    "position": array_property("25个关节位置 rad"),
                    "velocity": array_property("25个关节速度 rad/s"),
                    "feed_forward_torque": array_property("25个前馈力矩 Nm"),
                    "torque": array_property("25个力矩 Nm"),
                    "stiffness": array_property("25个刚度"),
                    "damping": array_property("25个阻尼"),
                    "parallel_parser_type": {"type": "integer", "enum": [0, 1]},
                    "duration": {"type": "number", "description": "秒；-1 持续"},
                    "force": {"type": "boolean", "description": "忽略 joint_bridge 状态门禁"},
                },
                "低层关节动作",
            ),
        }

    def start(self) -> None:
        from interface_protocol.msg import JointCommand

        self._message_type = JointCommand
        self._publisher = self._node.create_publisher(
            JointCommand, self._config["topics"]["joint_command"], _BEST_EFFORT
        )

    def dispatch(self, action: str, args: dict) -> dict:
        if action in ("start", "info", "status"):
            return self._status()
        if action in ("stop", "stop_command"):
            self.stop()
            self._publish_damping()
            return {"state": "stopped" if action == "stop_command" else "idle"}
        if action != "command":
            return {"error": f"unknown joint bridge action: {action}"}
        motion, _ = self._state.current_motion()
        if motion != "joint_bridge" and not bool(args.get("force", False)):
            return {"error": f"joint bridge requires joint_bridge state (current: {motion or 'unknown'})"}
        size = len(T800_JOINT_NAMES)
        parser_type = int(args.get("parallel_parser_type", 0))
        if parser_type not in (0, 1):
            return {"error": "parallel_parser_type must be 0 (classic) or 1 (RL)"}
        payload = {
            "position": float_list(args.get("position"), "position", size=size),
            "velocity": optional_floats(args, "velocity", size),
            "feed_forward_torque": optional_floats(args, "feed_forward_torque", size),
            "torque": optional_floats(args, "torque", size),
            "stiffness": optional_floats(args, "stiffness", size),
            "damping": optional_floats(args, "damping", size),
            "parallel_parser_type": parser_type,
        }
        duration = float(args.get("duration", 1.0))
        return {"state": "running", "stream": asdict(self._stream.start(payload, duration)), "duration": duration}

    def _publish(self, payload: dict) -> None:
        size = len(T800_JOINT_NAMES)
        msg = self._message_type()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.position = payload["position"]
        msg.velocity = list_or_default(payload["velocity"], size)
        msg.feed_forward_torque = list_or_default(payload["feed_forward_torque"], size)
        msg.torque = list_or_default(payload["torque"], size)
        msg.stiffness = list_or_default(payload["stiffness"], size)
        msg.damping = list_or_default(payload["damping"], size)
        msg.parallel_parser_type = payload["parallel_parser_type"]
        self._publisher.publish(msg)

    def _publish_damping(self) -> None:
        if self._publisher is None:
            return
        size = len(T800_JOINT_NAMES)
        self._publish({"position": self._state.joint_positions(), "velocity": [], "feed_forward_torque": [],
                       "torque": [], "stiffness": [0.0] * size, "damping": [1.0] * size,
                       "parallel_parser_type": 0})


class LedPlugin:
    def __init__(self, config: dict, namespace: str, ros2):
        self._config = config
        self._node = Node("t800_led", context=ros2.ctx_robot)
        ros2.executor_robot.add_node(self._node)
        self._publisher = None

    def get_tool(self) -> dict:
        return {
            "name": "led",
            "type": "actuator",
            "description": "T800 头灯、胸灯和膝灯的官方灯效控制",
            "inputSchema": {"type": "object", "properties": {
                "mode": {"type": "string", "enum": list(LED_MODES)}}, "required": ["mode"]},
        }

    def start(self) -> None:
        from interface_protocol.msg import LedControl
        self._message_type = LedControl
        self._publisher = self._node.create_publisher(LedControl, self._config["topics"]["led"], _RELIABLE)

    def stop(self) -> None:
        pass

    def dispatch(self, action: str, args: dict) -> dict:
        if action in ("start", "info"):
            return {"state": "ready", "modes": list(LED_MODES)}
        if action == "stop":
            return {"state": "idle"}
        mode = str(args.get("mode", action))
        if mode not in LED_MODES:
            return {"error": f"unknown LED mode: {mode}"}
        msg = self._message_type()
        msg.color = LED_MODES[mode]
        self._publisher.publish(msg)
        return {"state": "set", "mode": mode, "value": LED_MODES[mode]}


class TtsPlugin:
    def __init__(self, config: dict, namespace: str, ros2):
        self._config = config
        self._node = Node("t800_tts", context=ros2.ctx_robot)
        ros2.executor_robot.add_node(self._node)
        self._publisher = None

    def get_tool(self) -> dict:
        return {
            "name": "tts",
            "type": "actuator",
            "description": "T800 Native SDK TTS 消息接口；topic 可通过 config.yaml 校准",
            "inputSchema": {"type": "object", "properties": {
                "text": {"type": "string"}, "language": {"type": "string"},
                "speaker": {"type": "string"}, "rate": {"type": "integer", "minimum": 50, "maximum": 300}},
                "required": ["text"]},
        }

    def start(self) -> None:
        from interface_protocol.msg import Tts
        self._message_type = Tts
        self._publisher = self._node.create_publisher(Tts, self._config["topics"]["tts"], _RELIABLE)

    def stop(self) -> None:
        pass

    def dispatch(self, action: str, args: dict) -> dict:
        if action in ("start", "info"):
            return {"state": "ready", "topic": self._config["topics"]["tts"]}
        if action == "stop":
            return {"state": "idle"}
        text = str(args.get("text", "")).strip()
        if not text:
            return {"error": "text is required"}
        msg = self._message_type()
        msg.text = text
        msg.language = str(args.get("language", "zh"))
        msg.speaker = str(args.get("speaker", "default"))
        msg.rate = int(clamp(args.get("rate", 150), 50, 300))
        self._publisher.publish(msg)
        return {"state": "published", "characters": len(text), "language": msg.language,
                "speaker": msg.speaker, "rate": msg.rate}


class MotorPowerPlugin:
    def __init__(self, config: dict, namespace: str, ros2):
        self._config = config
        self._node = Node("t800_motor_power", context=ros2.ctx_robot)
        ros2.executor_robot.add_node(self._node)
        self._client = None

    def get_tool(self) -> dict:
        return {
            "name": "motor_power",
            "type": "actuator",
            "description": "T800 电机使能/失能服务（高风险底层能力）",
            "inputSchema": {
                "type": "object",
                "properties": {"action": {"type": "string", "enum": ["enable", "disable"]}},
                "required": ["action"],
            },
        }

    def start(self) -> None:
        from interface_protocol.srv import EnableMotor

        self._service_type = EnableMotor
        self._client = self._node.create_client(EnableMotor, self._config["services"]["enable_motor"])

    def stop(self) -> None:
        pass

    def dispatch(self, action: str, args: dict) -> dict:
        if action in ("start", "info"):
            available = bool(self._client and self._client.service_is_ready())
            if action == "start" and not available:
                return {"state": "error", "message": "motor enable service is unavailable",
                        "service": self._config["services"]["enable_motor"], "available": False}
            return {"state": "ready", "service": self._config["services"]["enable_motor"],
                    "available": available}
        if action == "stop":
            return {"state": "idle"}
        if action not in ("enable", "disable"):
            return {"error": f"unknown motor power action: {action}"}
        if not self._client.wait_for_service(timeout_sec=1.0):
            return {"error": "motor enable service is unavailable"}
        request = self._service_type.Request()
        request.enable = action == "enable"
        future = self._client.call_async(request)
        deadline = time.monotonic() + 5.0
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.02)
        if not future.done():
            return {"state": "timeout", "enabled": request.enable}
        response = future.result()
        return {"state": "completed" if response.success else "rejected", "enabled": request.enable,
                "success": bool(response.success), "message": response.message}


class NativeSdkPlugin:
    def __init__(self, config: dict, namespace: str, ros2):
        self._config = config
        self._manager = NativeSdkManager(config)

    def get_tool(self) -> dict:
        return self._manager.tool()

    def start(self) -> None:
        if bool(self._config.get("autostart", False)):
            self._manager.start()

    def stop(self) -> None:
        if bool(self._config.get("stop_on_exit", False)):
            self._manager.stop()

    def dispatch(self, action: str, args: dict) -> dict:
        if action == "info":
            action = "status"
        return self._manager.dispatch(action)


class NativeNodeControlPlugin:
    """Control Native SDK LogicNode instances through its official manager topic."""

    def __init__(self, config: dict, namespace: str, ros2):
        self._config = config
        self._node = Node("t800_native_node_control", context=ros2.ctx_robot)
        ros2.executor_robot.add_node(self._node)
        self._publisher = None

    def get_tool(self) -> dict:
        return {
            "name": "native_node_control",
            "type": "actuator",
            "multiInstance": False,
            "description": "通过 Native SDK ManagerNode 动态启动或停止已注册 LogicNode",
            "inputSchema": action_schema(
                {
                    "start_node": (["node_name"], "启动已注册的 Native SDK LogicNode"),
                    "stop_node": (["node_name"], "停止已注册的 Native SDK LogicNode"),
                    "status": ([], "返回控制 topic；Native SDK 当前协议不提供节点清单反馈"),
                },
                {"node_name": {"type": "string", "description": "Native SDK 注册节点名"}},
                "Native 节点动作",
            ),
        }

    def start(self) -> None:
        from interface_protocol.msg import NodeControl

        self._message_type = NodeControl
        self._publisher = self._node.create_publisher(
            NodeControl, self._config["topics"]["native_node_control"], _RELIABLE_ONE
        )

    def stop(self) -> None:
        pass

    def dispatch(self, action: str, args: dict) -> dict:
        if action in ("start", "info", "status"):
            return {
                "state": "ready",
                "topic": self._config["topics"]["native_node_control"],
                "feedback_available": False,
            }
        if action == "stop":
            return {"state": "idle"}
        if action not in ("start_node", "stop_node"):
            return {"error": f"unknown native node action: {action}"}
        node_name = str(args.get("node_name", "")).strip()
        if not node_name:
            return {"error": "node_name is required"}
        msg = self._message_type()
        msg.node_name = node_name
        msg.command = action == "start_node"
        self._publisher.publish(msg)
        return {
            "state": "requested",
            "node_name": node_name,
            "command": "start" if msg.command else "stop",
            "acknowledged": False,
        }


class SafetyControlPlugin:
    """One-call stop/recovery primitives composed from public ROS2 commands."""

    def __init__(self, config: dict, namespace: str, ros2, state: StatePlugin):
        self._config = config
        self._state = state
        self._node = Node("t800_safety_control", context=ros2.ctx_robot)
        ros2.executor_robot.add_node(self._node)
        self._controls: list = []

    def set_controls(self, controls: list) -> None:
        self._controls = list(controls)

    def get_tool(self) -> dict:
        return {
            "name": "safety",
            "type": "actuator",
            "multiInstance": False,
            "description": "停止运动流、释放覆盖、发送关节阻尼并请求 passive/idle 的组合控制",
            "inputSchema": action_schema(
                {
                    "soft_stop": ([], "发布零机身速度，不切换状态"),
                    "emergency_passive": ([], "零速度、释放覆盖、关节阻尼并请求 passive"),
                    "idle": ([], "零速度并请求 idle"),
                    "stand": ([], "请求 pd_stand；不会自动判定现场是否可安全站立"),
                    "status": ([], "查询当前 motion state"),
                },
                {},
                "组合安全动作",
            ),
        }

    def start(self) -> None:
        from interface_protocol.msg import BodyVelCmd, JointCommand, JointOverrideCommand, MotionStateRequest

        self._body_type = BodyVelCmd
        self._joint_type = JointCommand
        self._override_type = JointOverrideCommand
        self._motion_type = MotionStateRequest
        topics = self._config["topics"]
        self._body_pub = self._node.create_publisher(BodyVelCmd, topics["body_velocity"], _RELIABLE)
        self._joint_pub = self._node.create_publisher(JointCommand, topics["joint_command"], _BEST_EFFORT)
        self._override_pub = self._node.create_publisher(
            JointOverrideCommand, topics["joint_override"], _RELIABLE
        )
        self._motion_pub = self._node.create_publisher(
            MotionStateRequest, topics["motion_request"], _RELIABLE_ONE
        )

    def stop(self) -> None:
        pass

    def dispatch(self, action: str, args: dict) -> dict:
        current, available = self._state.current_motion()
        if action in ("start", "info", "status"):
            return {"state": "ready", "current_motion": current, "available": available}
        if action == "stop":
            return {"state": "idle"}
        if action not in ("soft_stop", "emergency_passive", "idle", "stand"):
            return {"error": f"unknown safety action: {action}"}
        stopped_streams = []
        for control in self._controls:
            if action == "soft_stop" and not isinstance(control, LocomotionPlugin):
                continue
            control.stop()
            stopped_streams.append(type(control).__name__)
        self._publish_zero_velocity()
        if action == "soft_stop":
            return {"state": "stopped", "motion_request": None, "stopped_streams": stopped_streams}
        if action == "emergency_passive":
            self._publish_override_release()
            self._publish_joint_damping()
            target = "passive"
        else:
            target = "idle" if action == "idle" else "pd_stand"
        request = self._motion_type()
        request.target_motion_name = target
        self._motion_pub.publish(request)
        return {"state": "requested", "previous_motion": current, "target_motion": target,
                "stopped_streams": stopped_streams}

    def _publish_zero_velocity(self) -> None:
        msg = self._body_type()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.header.frame_id = "body"
        msg.linear_velocity = [0.0, 0.0]
        msg.yaw_velocity = 0.0
        self._body_pub.publish(msg)

    def _publish_override_release(self) -> None:
        msg = self._override_type()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.weight = 0.0
        msg.joint_indices = list(range(len(T800_JOINT_NAMES)))
        msg.position = self._state.joint_positions()
        msg.velocity = [0.0] * len(T800_JOINT_NAMES)
        msg.feed_forward_torque = [0.0] * len(T800_JOINT_NAMES)
        msg.torque = [0.0] * len(T800_JOINT_NAMES)
        msg.stiffness = [0.0] * len(T800_JOINT_NAMES)
        msg.damping = [0.0] * len(T800_JOINT_NAMES)
        self._override_pub.publish(msg)

    def _publish_joint_damping(self) -> None:
        msg = self._joint_type()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.position = self._state.joint_positions()
        msg.velocity = [0.0] * len(T800_JOINT_NAMES)
        msg.feed_forward_torque = [0.0] * len(T800_JOINT_NAMES)
        msg.torque = [0.0] * len(T800_JOINT_NAMES)
        msg.stiffness = [0.0] * len(T800_JOINT_NAMES)
        msg.damping = [1.0] * len(T800_JOINT_NAMES)
        msg.parallel_parser_type = 0
        self._joint_pub.publish(msg)


class MicPlugin:
    """Capture the T800 microphone through the vendor-owned PulseAudio server."""

    _CHUNK_SAMPLES = 512  # 16 kHz 下 1024 字节 = 512 samples
    _CHUNK_BYTES = 1024

    def __init__(self, config: dict, namespace: str, ros2):
        self._config = config
        self._ns = namespace
        self._topic = f"/{namespace}/mic/audio"
        self._node = Node("t800_mic", context=ros2.ctx_core)
        ros2.executor_core.add_node(self._node)
        self._publisher = None
        self._message_type = None
        self._header_type = None
        self._running = False
        self._process = None
        self._thread = None
        self._samples_published = 0
        self._last_error = ""

    def get_tool(self) -> dict:
        return sensor_tool(
            "mic",
            f"T800 内置麦克风，经 PulseAudio 采集 PCM-16 16kHz 单声道并发布到 {self._topic}",
            self._topic,
            "audio/pcm-16k",
        )

    def start(self) -> None:
        if self._running:
            return
        if self._publisher is None:
            from audio_msgs.msg import AudioChunk
            from std_msgs.msg import Header

            self._message_type = AudioChunk
            self._header_type = Header
            self._publisher = self._node.create_publisher(AudioChunk, self._topic, _BEST_EFFORT)

        try:
            self._check_pulse()
            self._process = self._spawn_capture()
        except Exception as exc:
            self._last_error = str(exc)
            raise RuntimeError(f"PulseAudio capture is unavailable: {exc}") from exc
        if self._process.poll() is not None:
            self._last_error = f"parec exited with code {self._process.returncode}"
            self._process = None
            raise RuntimeError(self._last_error)
        self._running = True
        self._thread = threading.Thread(target=self._capture_loop, daemon=True, name="t800-mic")
        self._thread.start()

    def _spawn_capture(self):
        return subprocess.Popen(
            ["parec", "--raw", "--format=s16le", "--rate=16000", "--channels=1",
             "--latency-msec=50"],
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            bufsize=0,
        )

    def _check_pulse(self) -> None:
        subprocess.run(["pactl", "info"], capture_output=True, text=True, timeout=3, check=True)

    def stop(self) -> None:
        self._running = False
        process = self._process
        self._process = None
        if process is not None and process.poll() is None:
            try:
                process.terminate()
                process.wait(timeout=1)
            except Exception:
                process.kill()
        thread = self._thread
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=1)
        self._thread = None

    def _capture_loop(self) -> None:
        pending = bytearray()
        process = self._process
        try:
            while self._running and process is not None and process.stdout is not None:
                data = process.stdout.read(self._CHUNK_BYTES - len(pending))
                if not data:
                    if process.poll() is not None:
                        break
                    continue
                pending.extend(data)
                if len(pending) == self._CHUNK_BYTES:
                    self._publish_chunk(bytes(pending))
                    pending.clear()
        except Exception as exc:  # noqa: BLE001
            self._last_error = str(exc)
        finally:
            if self._running and process is not None and process.poll() is not None:
                self._last_error = f"parec exited with code {process.returncode}"
                self._running = False

    def _publish_chunk(self, chunk: bytes) -> None:
        msg = self._message_type()
        msg.header = self._header_type()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.format = "audio/pcm-16k"
        msg.data = list(chunk)
        self._publisher.publish(msg)
        self._samples_published += len(chunk) // 2

    def dispatch(self, action: str, args: dict) -> dict:
        if action == "start":
            try:
                self.start()
            except RuntimeError as exc:
                message = f"mic capture failed: {exc}"
                return {"state": "error", "message": message, "error": message}
            if self._running:
                return {"state": "running", "topic_out": [{"topic": self._topic, "format": "audio/pcm-16k"}]}
            message = f"mic capture failed: {self._last_error or 'no audio device'}"
            return {"state": "error", "message": message, "error": message}
        if action == "stop":
            self.stop()
            return {"state": "idle"}
        if action in ("info", "status"):
            return {"state": "running" if self._running else "idle",
                    "topic_out": [{"topic": self._topic, "format": "audio/pcm-16k"}],
                    "samples_published": self._samples_published,
                    "last_error": self._last_error}
        return {"error": f"unknown mic action: {action}"}


class SpeakerPlugin:
    """Canvas PCM sink that plays through the T800 built-in speaker.

    按众擎飞书《ROS2 接口开发文档》第8章实现：回放走官方 ALSA 接口
    ``aplay``（8.2.2，``-t raw`` 从 stdin 流式播放 PCM-16 16 kHz 单声道），
    系统音量走官方 ``pactl`` 接口（8.2.3 / 8.3.1，0-100）。画布 / Agent
    Core 把音频文件与用户 mic 统一转成 ``audio/pcm-16k`` 块流（与 G1
    speaker 契约一致）发布到 topic_in，本卡写入 aplay stdin 经内置喇叭
    播放。
    """

    PREFIX = "speaker"
    _EOF_MAGIC = b"\x01\x00\xff\xff\x01\x00\xff\xff"  # G1 契约：utterance 结束标记

    def __init__(self, config: dict, namespace: str, ros2):
        self._node = Node("t800_speaker", context=ros2.ctx_core)
        ros2.executor_core.add_node(self._node)
        self._subscription = None
        self._input_topic = ""
        self._state = "idle"
        self._last_error = ""
        self._process = None
        # 实时音频滑动窗口：队列上限限制最大滞后（50 块 × 1024B ≈ 1.6s）。
        # remote_mic 等持续流若发布速率高于播放速率，满时丢最旧块而非新块，
        # 保证播放的是最新内容而非永远滞后的积压数据。
        self._queue = queue.Queue(maxsize=50)
        self._beep_queue = queue.Queue(maxsize=256)  # 开机音独立队列，与 live _queue 隔离
        self._thread = None
        self._running = False
        self._session = 0
        self._lifecycle_lock = threading.RLock()
        self._chunks_played = 0
        self._dropped = 0
        self._last_chunk_time = 0.0
        # 每个会话用独立 Event 标记 live PCM 已到达。它既终止开机音生产，
        # 也让播放器拒绝在 drain 竞态后迟到的开机音块。
        self._beep_cancel = threading.Event()

    def get_tool(self) -> dict:
        return {
            "name": "speaker",
            "type": "actuator",
            "multiInstance": False,
            "description": "T800 speaker — 订阅画布连接的 PCM-16k 音频流，经官方 ALSA aplay "
                           "接口流式播放到机器人喇叭；音量经官方 pactl 接口控制",
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {
                        "type": "string",
                        "enum": ["start", "stop", "info", "get_volume", "set_volume"],
                    },
                    "input_topic": {
                        "type": "string",
                        "description": "画布连接提供的 ROS2 PCM 音频 topic",
                    },
                    "volume": {
                        "type": "integer",
                        "minimum": 0,
                        "maximum": 100,
                        "description": "系统音量 0-100（官方 pactl 接口）",
                    },
                },
                "required": ["action"],
                "x-action-params": {
                    "start": {"params": ["input_topic"], "description": "订阅画布音频 topic 并开始流式播放"},
                    "stop": {"params": [], "description": "停止播放并释放订阅"},
                    "info": {"params": [], "description": "查询播放状态、缓冲与计数"},
                    "get_volume": {"params": [], "description": "查询机器人喇叭系统音量（官方 pactl 接口）"},
                    "set_volume": {"params": ["volume"], "description": "设置机器人喇叭系统音量 0-100（官方 pactl 接口）"},
                },
            },
            "topic_in": [{"format": "audio/pcm-16k"}],
        }

    def start(self) -> None:
        pass  # 播放经 dispatch(start) 按画布连接的 input_topic 启动

    def stop(self) -> None:
        with self._lifecycle_lock:
            self._stop_locked()

    def _stop_locked(self) -> None:
        self._running = False
        self._beep_cancel.set()
        self._session += 1  # 使旧播放线程失效，防止其状态更新覆盖新会话
        if self._subscription is not None:
            self._node.destroy_subscription(self._subscription)
            self._subscription = None
        while not self._queue.empty():
            try:
                self._queue.get_nowait()
            except queue.Empty:
                break
        while not self._beep_queue.empty():
            try:
                self._beep_queue.get_nowait()
            except queue.Empty:
                break
        process = self._process
        self._process = None
        if process is not None:
            try:
                if process.stdin is not None:
                    process.stdin.close()
            except Exception:
                pass  # 已退出的 aplay 可能已关闭/回收 stdin
            try:
                if process.poll() is None:
                    process.terminate()
                    process.wait(timeout=1)
            except Exception:
                try:
                    if process.poll() is None:
                        process.kill()
                except Exception:
                    pass  # stop 必须对已消失或已回收的进程幂等
        thread = self._thread
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=1)
        self._thread = None
        self._input_topic = ""
        self._state = "idle"

    def _spawn_player(self):
        # 官方 8.2.2：aplay 回放；-t raw 表示从 stdin 流式读原始 PCM。
        # 默认设备已路由到 PulseAudio（/etc/asound.conf），--buffer-time/
        # --period-time 压低 aplay 读前缓冲（默认 500ms 会让 remote_mic
        # 等实时音频源产生明显延迟），PulseAudio 侧延迟由 PULSE_LATENCY_MSEC 控制。
        return subprocess.Popen(
            ["aplay", "-q", "-t", "raw", "-f", "S16_LE", "-r", "16000", "-c", "1",
             "--buffer-time=100000", "--period-time=20000", "-"],
            stdin=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            bufsize=0,
        )

    def _check_pulse(self) -> None:
        subprocess.run(["pactl", "info"], capture_output=True, text=True, timeout=3, check=True)

    def _run_command(self, command: list[str]) -> str:
        result = subprocess.run(command, capture_output=True, text=True, timeout=10, check=True)
        return result.stdout.strip()

    def _get_volume(self) -> int:
        out = self._run_command(["pactl", "get-sink-volume", "@DEFAULT_SINK@"])
        match = re.search(r"(\d+)%", out)
        if match is None:
            raise RuntimeError(f"cannot parse pactl volume output: {out!r}")
        return int(match.group(1))

    def _set_volume(self, volume) -> int:
        value = int(clamp(volume, 0, 100))
        self._run_command(["pactl", "set-sink-volume", "@DEFAULT_SINK@", f"{value}%"])
        return value

    def _start_play(self, topic: str) -> dict:
        with self._lifecycle_lock:
            return self._start_play_locked(topic)

    def _start_play_locked(self, topic: str) -> dict:
        self._stop_locked()
        from audio_msgs.msg import AudioChunk
        # 每个播放会话拥有独立队列。旧回调/播放线程/开机音线程即使在
        # stop() 后才恢复执行，也只能访问旧队列，不能污染或消费新会话。
        self._queue = queue.Queue(maxsize=50)
        self._beep_queue = queue.Queue(maxsize=256)
        self._beep_cancel = threading.Event()
        live_queue = self._queue
        beep_queue = self._beep_queue
        beep_cancel = self._beep_cancel
        session = self._session  # stop() 已递增,捕获当前会话
        try:
            self._check_pulse()
            self._run_command(["pactl", "set-sink-mute", "@DEFAULT_SINK@", "0"])
            self._process = self._spawn_player()
            self._input_topic = topic
            self._subscription = self._node.create_subscription(
                AudioChunk, topic,
                lambda msg: self._on_chunk(
                    msg, session, live_queue, beep_queue, beep_cancel
                ),
                _AUDIO_QOS,
            )
        except Exception as exc:
            self._stop_locked()
            self._last_error = str(exc)
            return {"state": "error", "message": f"speaker playback failed: {exc}"}
        if self._process is None or self._process.poll() is not None:
            code = self._process.returncode if self._process is not None else "unknown"
            self._last_error = f"aplay exited with code {code}"
            self._stop_locked()
            return {"state": "error", "message": self._last_error}
        self._running = True
        self._state = "ready"
        process = self._process  # 捕获当前进程供播放线程独占持有
        self._thread = threading.Thread(
            target=self._play_loop,
            args=(session, process, live_queue, beep_queue, beep_cancel),
            daemon=True,
            name="t800-speaker",
        )
        self._thread.start()
        # 开机音在订阅和播放线程就绪后异步入队——不阻塞 dispatch(start)，
        # 首个 live PCM 到达时会原子取消并抢占剩余开机音。
        self._enqueue_startup_sound(session, beep_queue, beep_cancel)
        return {"state": "ready", "topic_in": [{"topic": topic, "format": "audio/pcm-16k"}]}

    def _enqueue_startup_sound(
        self,
        session: int | None = None,
        beep_queue: queue.Queue | None = None,
        beep_cancel: threading.Event | None = None,
    ) -> None:
        """把 startup_beep.pcm 分块异步推入 _beep_queue。

        使用独立 _beep_queue 与实时音频 _queue 隔离，防止
        drain/drop 误操作实时 PCM。_play_loop 始终优先实时队列；首个
        live chunk 到达时设置本会话的取消事件，_drain_beep() 只清空
        _beep_queue。
        """
        import pathlib

        pcm_path = pathlib.Path(__file__).parent / "resource" / "startup_beep.pcm"
        try:
            pcm = pcm_path.read_bytes()
        except (OSError, IOError):
            return
        if not pcm:
            return
        # 正常路径由 _start_play_locked 显式传入会话资源；默认值只供直接
        # 调用兼容。线程不能在真正获得调度时再读取“当前会话”。
        if session is None:
            session = self._session
        if beep_queue is None:
            beep_queue = self._beep_queue
        if beep_cancel is None:
            beep_cancel = self._beep_cancel
        threading.Thread(
            target=self._enqueue_beep_blocks,
            args=(pcm, session, beep_queue, beep_cancel),
            daemon=True,
            name="t800-beep-enqueue",
        ).start()

    def _enqueue_beep_blocks(
        self,
        pcm: bytes,
        session: int,
        beep_queue: queue.Queue,
        beep_cancel: threading.Event,
    ) -> None:
        """后台线程：分块阻塞入队开机音到 _beep_queue。

        用独立 _beep_queue（maxsize=256=整块开机音），队列容量
        足以容纳整段开机音，不会因满而丢块。当 stop() 或首个
        live chunk 到达时检查标志退出。
        """
        chunk_size = 1024
        for offset in range(0, len(pcm), chunk_size):
            if self._session != session:
                return  # 新会话已开始，旧开机音线程退出
            if beep_cancel.is_set():
                return  # live chunk 已到达
            block = pcm[offset:offset + chunk_size]
            beep_queue.put(block)  # 队列容量充足，阻塞等待消耗

    def _drain_beep(self, beep_queue: queue.Queue | None = None) -> None:
        """清空 _beep_queue 中剩余的开机音块——首个 live chunk 到达时调用。

        只清空独立开机音 _beep_queue，不影响 _queue 中的 live PCM。
        """
        if beep_queue is None:
            beep_queue = self._beep_queue
        drained = 0
        while drained < 256:  # 安全上限（_beep_queue maxsize）
            try:
                beep_queue.get_nowait()
                drained += 1
            except queue.Empty:
                break

    def _on_chunk(
        self,
        msg,
        session: int | None = None,
        live_queue: queue.Queue | None = None,
        beep_queue: queue.Queue | None = None,
        beep_cancel: threading.Event | None = None,
    ) -> None:
        # 丢弃不属于当前会话的回调:stop() 后残留的 ROS 回调若在新会话
        # 播放器已就绪后执行,会把旧 topic 的 PCM 入队泄漏到新连接。
        if session is not None and session != self._session:
            return
        if not self._running:
            return
        pcm = bytes(msg.data)
        if pcm == self._EOF_MAGIC or not pcm:
            return
        if getattr(msg, "format", "audio/pcm-16k") not in ("audio/pcm-16k", "pcm_16k_16bit_mono"):
            self._last_error = f"unsupported audio format: {msg.format}"
            return
        if live_queue is None:
            live_queue = self._queue
        if beep_queue is None:
            beep_queue = self._beep_queue
        if beep_cancel is None:
            beep_cancel = self._beep_cancel
        # 首个 live chunk 到达时清空队列中剩余开机音块——避免
        # 8 秒 beep 排在实时音频流前面造成明显延迟。
        if not beep_cancel.is_set():
            # 先设置取消标记，再 drain/入 live 队列。播放器观察到标记后
            # 会拒绝任何已取出或在 drain 之后迟到的 beep 块。
            beep_cancel.set()
            self._drain_beep(beep_queue)
        try:
            live_queue.put_nowait(pcm)
            self._last_chunk_time = time.monotonic()
        except queue.Full:
            # 滑动窗口：丢弃最旧的块，腾出位置放新块——持续实时流下
            # 播放始终跟随最新输入，而不是永远滞后的积压音频。
            try:
                live_queue.get_nowait()
            except queue.Empty:
                pass
            try:
                live_queue.put_nowait(pcm)
            except queue.Full:
                self._last_error = "speaker buffer full; audio chunk dropped"
                return
            self._last_chunk_time = time.monotonic()
            self._dropped += 1

    def _play_loop(
        self,
        session: int,
        process: subprocess.Popen | None,
        live_queue: queue.Queue,
        beep_queue: queue.Queue,
        beep_cancel: threading.Event,
    ) -> None:
        # 会话隔离：stop() 会递增 _session 并关闭 aplay stdin，旧线程随后
        # 在写失败/队列超时处退出，其状态更新不得覆盖新会话。
        # 每个线程独占持有自己的 process 引用，防止旧线程在 stop/start
        # 之后写入新会话的 aplay stdin。
        while self._running and self._session == session:
            # live PCM 始终优先。首个 live chunk 设置 beep_cancel 后，已取出
            # 或 drain 之后迟到的 beep 块也会在写 aplay 前被丢弃。
            from_beep = False
            try:
                pcm = live_queue.get_nowait()
            except queue.Empty:
                pcm = None
                if not beep_cancel.is_set():
                    try:
                        pcm = beep_queue.get_nowait()
                        from_beep = True
                    except queue.Empty:
                        pass
                if pcm is None:
                    try:
                        pcm = live_queue.get(timeout=0.1)
                    except queue.Empty:
                        if self._session != session:
                            return
                        if self._state == "playing" and time.monotonic() - self._last_chunk_time >= 0.3:
                            self._state = "ready"
                        continue
            if from_beep and beep_cancel.is_set():
                continue
            # 队列 get() 与写入之间必须重新检查 session：
            # stop() 仅 join 旧线程 1 秒，阻塞在 get() 上的旧线程醒来后
            # 若不重检 session，会把旧队列残留 PCM 写到新会话的 aplay stdin。
            if self._session != session:
                return
            try:
                if process is None or process.poll() is not None or process.stdin is None:
                    raise RuntimeError("aplay is not running")
                process.stdin.write(pcm)
                process.stdin.flush()
                self._chunks_played += 1
                self._state = "playing"
            except Exception as exc:
                if self._session == session:
                    self._last_error = str(exc)
                    self._state = "error"
                    self._running = False
                break

    def dispatch(self, action: str, args: dict) -> dict:
        if action in ("start", "play"):
            topic = str(args.get("input_topic", "")).strip()
            if not topic:
                return {"state": "error", "message": "Missing input_topic", "error": "Missing input_topic"}
            return self._start_play(topic)
        if action == "stop":
            self.stop()
            return {"state": "idle"}
        if action == "get_volume":
            try:
                return {"state": self._state, "volume": self._get_volume()}
            except Exception as exc:
                return {"state": self._state, "error": f"get_volume failed: {exc}"}
        if action == "set_volume":
            try:
                return {"state": self._state, "volume": self._set_volume(args.get("volume", 100))}
            except Exception as exc:
                return {"state": self._state, "error": f"set_volume failed: {exc}"}
        if action == "info":
            topic_in = ([{"topic": self._input_topic, "format": "audio/pcm-16k"}]
                        if self._input_topic else [{"format": "audio/pcm-16k"}])
            return {"state": self._state, "topic_in": topic_in,
                    "buffer_chunks": self._queue.qsize(), "chunks_played": self._chunks_played,
                    "dropped_old": self._dropped,
                    "last_error": self._last_error}
        return {"error": f"unknown speaker action: {action}"}


class VisionPlugin:
    """T800-Odin2 激光雷达相机视觉数据桥接（飞书文档 7.2 节）。

    Subscribes to the Odin2 raw/SLAM point clouds, stereo compressed images
    and the calibrated ``32FC1`` depth image published by the official
    ``pcd2depth_ros2_node`` on the Orin board.  It republishes normalized
    streams on domain 42 for Agent Core and the dashboard renderers
    (``sensor/pointcloud``, ``image/jpeg``, ``image/depth-z16``).  Topic names
    follow the per-device prefix ``/{topic_prefix}/{model}/device{N}/`` and
    must be calibrated against ``ros_graph`` on the real robot.
    """

    _SOURCES = ("raw", "slam")
    _TOOL_NAMES = ("pointcloud", "camera", "depth")
    _DEPTH_WIDTH = 640
    _DEPTH_HEIGHT = 480

    def __init__(self, config: dict, namespace: str, ros2):
        self._config = config
        self._ns = namespace
        self._topics = config["topics"]
        vision_config = config.get("plugins", {}).get("vision", {}) or {}
        self._source = vision_config.get("source", "raw")
        if self._source not in self._SOURCES:
            self._source = "raw"
        self._cloud_topic = f"/{namespace}/vision/cloud"
        self._cam_left_topic = f"/{namespace}/vision/camera_left"
        self._cam_right_topic = f"/{namespace}/vision/camera_right"
        self._depth_topic = f"/{namespace}/vision/depth"
        self._sub_node = Node("t800_vision_sub", context=ros2.ctx_robot)
        self._pub_node = Node("t800_vision_pub", context=ros2.ctx_core)
        ros2.executor_robot.add_node(self._sub_node)
        ros2.executor_core.add_node(self._pub_node)
        self._running = False
        self._initialized = False
        self._enabled_tools: set[str] = set()
        self._lock = threading.RLock()
        self._frames = {"pointcloud": 0, "camera_left": 0, "camera_right": 0, "depth": 0}

    def get_tools(self) -> list[dict]:
        return [self._cloud_tool(), self._camera_tool(), self._depth_tool()]

    def _cloud_tool(self) -> dict:
        tool = sensor_tool(
            "pointcloud",
            f"T800-Odin2 {self._source} 点云转发（256×192）；二进制 [uint32 point_step][uint32 total_points]"
            f"[PointCloud2 bytes]，发布到 {self._cloud_topic}",
            self._cloud_topic,
            "sensor/pointcloud",
        )
        schema = action_schema(
            _with_lifecycle({
                "status": ([], "返回点云流状态"),
                "select_source": (["source"], "切换 Odin2 raw 或 SLAM 点云源"),
            }),
            {"source": {"type": "string", "enum": list(self._SOURCES)}},
            "点云生命周期和数据源选择",
        )
        schema.pop("required", None)
        tool["inputSchema"] = schema
        return tool

    def _camera_tool(self) -> dict:
        return {
            "name": "camera",
            "type": "sensor",
            "multiInstance": False,
            "readOnly": True,
            "description": f"T800-Odin2 双目 JPEG 图像转发，发布到 {self._cam_left_topic}（左）和"
                           f" {self._cam_right_topic}（右）",
            "inputSchema": sensor_action_schema(),
            "topic_out": [
                {"topic": self._cam_left_topic, "format": "image/jpeg"},
                {"topic": self._cam_right_topic, "format": "image/jpeg"},
            ],
        }

    def _depth_tool(self) -> dict:
        return sensor_tool(
            "depth",
            f"T800-Odin2 官方标定深度图（640×480，毫米 16UC1），发布到 {self._depth_topic}",
            self._depth_topic,
            "image/depth-z16",
        )

    def start(self) -> None:
        if self._running:
            return
        if self._initialized:
            self._enabled_tools.update(self._TOOL_NAMES)
            self._running = True
            return
        import array as _array
        import struct as _struct
        import numpy as _np
        from sensor_msgs.msg import CompressedImage, Image, PointCloud2
        from std_msgs.msg import UInt8MultiArray

        self._running = True
        self._struct = _struct
        self._np = _np
        self._image_type = Image
        self._array = _array
        self._multi_type = UInt8MultiArray
        self._cloud_pub = self._pub_node.create_publisher(UInt8MultiArray, self._cloud_topic, _BEST_EFFORT)
        self._cam_left_pub = self._pub_node.create_publisher(CompressedImage, self._cam_left_topic, _BEST_EFFORT)
        self._cam_right_pub = self._pub_node.create_publisher(CompressedImage, self._cam_right_topic, _BEST_EFFORT)
        self._depth_pub = self._pub_node.create_publisher(Image, self._depth_topic, _BEST_EFFORT)
        self._sub_node.create_subscription(
            PointCloud2, self._topics["vision_cloud_raw"], self._on_cloud_raw, _BEST_EFFORT
        )
        self._sub_node.create_subscription(
            PointCloud2, self._topics["vision_cloud_slam"], self._on_cloud_slam, _BEST_EFFORT
        )
        self._sub_node.create_subscription(
            CompressedImage, self._topics["vision_camera_left"], self._on_camera_left, _BEST_EFFORT
        )
        self._sub_node.create_subscription(
            CompressedImage, self._topics["vision_camera_right"], self._on_camera_right, _BEST_EFFORT
        )
        self._sub_node.create_subscription(
            Image, self._topics["vision_depth"], self._on_depth, _RELIABLE
        )
        self._initialized = True
        self._enabled_tools.update(self._TOOL_NAMES)

    def stop(self) -> None:
        self._running = False
        self._enabled_tools.clear()

    def _on_cloud_raw(self, msg) -> None:
        self._on_cloud(msg, "raw")

    def _on_cloud_slam(self, msg) -> None:
        self._on_cloud(msg, "slam")

    def _on_cloud(self, msg, source: str) -> None:
        if not self._running or "pointcloud" not in self._enabled_tools or source != self._source:
            return
        data = bytes(msg.data)
        if not data:
            return
        point_step = int(msg.point_step) or 1
        header = self._struct.pack("<II", point_step, len(data) // point_step)
        buf = bytearray(8 + len(data))
        buf[:8] = header
        buf[8:] = data
        z_field = next(
            (field for field in getattr(msg, "fields", []) if field.name == "z"),
            None,
        )
        if z_field is not None and int(getattr(z_field, "datatype", 7)) == 7 \
                and int(z_field.offset) + 4 <= point_step:
            # 仅当 z 是 FLOAT32 时取反;其他 datatype 或大端布局跳过,避免损坏
            self._negate_z_inplace(buf, 8, point_step, int(z_field.offset),
                                   is_bigendian=bool(getattr(msg, "is_bigendian", False)))
        out = self._multi_type()
        out.data = self._array.array("B", buf)
        self._cloud_pub.publish(out)
        self._frames["pointcloud"] += 1

    @staticmethod
    def _negate_z_inplace(buf: bytearray, start: int, point_step: int, z_offset: int,
                          is_bigendian: bool = False) -> None:
        import numpy as np

        total = (len(buf) - start) // point_step
        if total <= 0:
            return
        if is_bigendian:
            return  # 大端布局与 <f4 取反不兼容,跳过(透传原数据)避免损坏
        raw = np.frombuffer(buf, dtype=np.uint8, count=total * point_step, offset=start)
        raw = raw.reshape(total, point_step)
        # 注意：不能用 ravel()——不连续视图上 ravel 会拷贝，原地乘法写不回去
        z_values = raw[:, z_offset:z_offset + 4].view("<f4")
        z_values *= -1

    def _on_camera_left(self, msg) -> None:
        if not self._running or "camera" not in self._enabled_tools:
            return
        self._cam_left_pub.publish(msg)
        self._frames["camera_left"] += 1

    def _on_camera_right(self, msg) -> None:
        if not self._running or "camera" not in self._enabled_tools:
            return
        self._cam_right_pub.publish(msg)
        self._frames["camera_right"] += 1

    def _on_depth(self, msg) -> None:
        if not self._running or "depth" not in self._enabled_tools:
            return
        width = int(msg.width)
        height = int(msg.height)
        encoding = str(msg.encoding).upper()
        if encoding == "32FC1":
            item_size = 4
            dtype = "f4"
        elif encoding in ("16UC1", "MONO16"):
            item_size = 2
            dtype = "u2"
        else:
            return
        row_step = int(msg.step)
        if width <= 0 or height <= 0 or row_step < width * item_size:
            return
        data = memoryview(msg.data)
        required = (height - 1) * row_step + width * item_size
        if len(data) < required:
            return
        byte_order = ">" if bool(msg.is_bigendian) else "<"
        depth = self._np.ndarray(
            shape=(height, width),
            dtype=self._np.dtype(f"{byte_order}{dtype}"),
            buffer=data,
            strides=(row_step, item_size),
        )

        if encoding == "32FC1":
            # The vendor node has already transformed the lidar points into
            # the camera frame and publishes optical-axis depth in metres.
            valid = self._np.isfinite(depth) & (depth > 0.0)
            depth_mm = self._np.zeros((height, width), dtype="<u2")
            depth_mm[valid] = self._np.clip(
                self._np.rint(depth[valid] * 1000.0), 1, 65535
            ).astype("<u2")
        else:
            depth_mm = depth.astype("<u2", copy=True)

        # Agent Core's depth renderer consumes the Image payload without its
        # ROS metadata and therefore requires a fixed 640x480 uint16 buffer.
        target_width = self._DEPTH_WIDTH
        target_height = self._DEPTH_HEIGHT
        if width * target_height > height * target_width:
            crop_width = max(1, height * target_width // target_height)
            x0 = (width - crop_width) // 2
            depth_mm = depth_mm[:, x0:x0 + crop_width]
        elif width * target_height < height * target_width:
            crop_height = max(1, width * target_height // target_width)
            y0 = (height - crop_height) // 2
            depth_mm = depth_mm[y0:y0 + crop_height, :]
        source_height, source_width = depth_mm.shape
        rows = self._np.arange(target_height) * source_height // target_height
        cols = self._np.arange(target_width) * source_width // target_width
        depth_mm = depth_mm[rows[:, None], cols[None, :]].astype("<u2", copy=False)

        out = self._image_type()
        out.header = msg.header
        out.height = target_height
        out.width = target_width
        out.encoding = "16UC1"
        out.is_bigendian = False
        out.step = target_width * 2
        out.data = depth_mm.tobytes(order="C")
        self._depth_pub.publish(out)
        self._frames["depth"] += 1

    def dispatch(self, action: str, args: dict) -> dict:
        tool_name = str(args.get("_tool_name", ""))
        if not tool_name and action in self._TOOL_NAMES:
            tool_name = action
        if tool_name not in self._TOOL_NAMES:
            tool_name = ""

        if action == "start":
            was_initialized = self._initialized
            if not was_initialized:
                self.start()
            if tool_name and not was_initialized:
                # Direct use outside the bundle initializes all publishers and
                # subscriptions once, but only activates the requested card.
                self._enabled_tools = {tool_name}
            elif tool_name:
                self._enabled_tools.add(tool_name)
            else:
                self._enabled_tools.update(self._TOOL_NAMES)
            self._running = bool(self._enabled_tools)
        elif action == "stop":
            if tool_name:
                self._enabled_tools.discard(tool_name)
                self._running = bool(self._enabled_tools)
            else:
                self.stop()
        if action in ("start", "stop", "info", "status", "pointcloud", "camera", "depth"):
            topic_out_by_tool = {
                "pointcloud": [{"topic": self._cloud_topic, "format": "sensor/pointcloud"}],
                "camera": [
                    {"topic": self._cam_left_topic, "format": "image/jpeg"},
                    {"topic": self._cam_right_topic, "format": "image/jpeg"},
                ],
                "depth": [{"topic": self._depth_topic, "format": "image/depth-z16"}],
            }
            selected_tools = (tool_name,) if tool_name else self._TOOL_NAMES
            topic_out = [topic for name in selected_tools for topic in topic_out_by_tool[name]]
            is_running = (
                tool_name in self._enabled_tools if tool_name else bool(self._enabled_tools)
            )
            return {"state": "running" if is_running else "idle",
                    "source": self._source,
                    "topic_out": topic_out,
                    "frames": dict(self._frames)}
        if action == "select_source":
            source = str(args.get("source", "")).strip()
            if source not in self._SOURCES:
                raise ValueError(f"invalid pointcloud source: {source}; expected {'|'.join(self._SOURCES)}")
            with self._lock:
                self._source = source
            return {"state": "running" if self._running else "idle", "source": source}
        return {"error": f"unknown vision action: {action}"}


# ── Mapping (Odin2 odometry + point cloud) ───────────────────────────────────

class _MappingDB:
    """SQLite storage for saved maps (name, pcd path, point count)."""

    def __init__(self, db_path: str):
        os.makedirs(os.path.dirname(db_path) if os.path.dirname(db_path) else ".", exist_ok=True)
        self._conn = sqlite3.connect(db_path, check_same_thread=False)
        self._conn.row_factory = sqlite3.Row
        self._migrate()

    def _migrate(self) -> None:
        self._conn.executescript(
            """
            CREATE TABLE IF NOT EXISTS maps (
                name        TEXT PRIMARY KEY,
                pcd_path    TEXT NOT NULL,
                point_count INTEGER DEFAULT 0,
                created_at  REAL DEFAULT (strftime('%s','now'))
            );
            CREATE TABLE IF NOT EXISTS poi (
                id          INTEGER PRIMARY KEY AUTOINCREMENT,
                name        TEXT NOT NULL,
                description TEXT DEFAULT '',
                x           REAL NOT NULL,
                y           REAL NOT NULL,
                z           REAL DEFAULT 0,
                yaw         REAL DEFAULT 0,
                map_name    TEXT NOT NULL,
                created_at  REAL DEFAULT (strftime('%s','now')),
                UNIQUE(name, map_name)
            );
            CREATE TABLE IF NOT EXISTS state (
                key        TEXT PRIMARY KEY,
                value      TEXT,
                updated_at REAL DEFAULT (strftime('%s','now'))
            );
            """
        )
        # Add cloud_path column to maps if missing (legacy DB compatibility)
        try:
            self._conn.execute("ALTER TABLE maps ADD COLUMN cloud_path TEXT")
        except sqlite3.OperationalError:
            pass  # column already exists
        self._conn.commit()

    def add_map(self, name: str, pcd_path: str, point_count: int) -> None:
        self._conn.execute(
            "INSERT OR REPLACE INTO maps (name, pcd_path, point_count) VALUES (?, ?, ?)",
            (name, pcd_path, point_count),
        )
        self._conn.commit()

    def get_map(self, name: str) -> dict | None:
        row = self._conn.execute("SELECT * FROM maps WHERE name = ?", (name,)).fetchone()
        return dict(row) if row else None

    def list_maps(self) -> list[dict]:
        rows = self._conn.execute(
            "SELECT name, pcd_path, point_count, created_at FROM maps ORDER BY created_at DESC"
        ).fetchall()
        return [dict(r) for r in rows]

    def delete_map(self, name: str) -> bool:
        map_info = self.get_map(name)
        if not map_info:
            return False
        try:
            os.remove(map_info["pcd_path"])
        except OSError:
            pass
        self._conn.execute("DELETE FROM maps WHERE name = ?", (name,))
        self._conn.execute("DELETE FROM poi WHERE map_name = ?", (name,))
        self._conn.commit()
        return True

    def add_poi(
        self,
        name: str,
        x: float,
        y: float,
        z: float,
        yaw: float,
        map_name: str,
        description: str = "",
    ) -> None:
        self._conn.execute(
            "INSERT OR REPLACE INTO poi (name, description, x, y, z, yaw, map_name) "
            "VALUES (?, ?, ?, ?, ?, ?, ?)",
            (name, description, x, y, z, yaw, map_name),
        )
        self._conn.commit()

    def delete_poi(self, name: str, map_name: str) -> bool:
        cur = self._conn.execute(
            "DELETE FROM poi WHERE name = ? AND map_name = ?", (name, map_name)
        )
        self._conn.commit()
        return cur.rowcount > 0

    def list_pois(self, map_name: str) -> list[dict]:
        rows = self._conn.execute(
            "SELECT name, description, x, y, z, yaw, map_name, created_at "
            "FROM poi WHERE map_name = ? ORDER BY name",
            (map_name,),
        ).fetchall()
        return [dict(r) for r in rows]

    def find_poi(self, query: str, map_name: str) -> dict | None:
        row = self._conn.execute(
            "SELECT name, description, x, y, z, yaw, map_name "
            "FROM poi WHERE map_name = ? AND name LIKE ?",
            (map_name, f"%{query}%"),
        ).fetchone()
        return dict(row) if row else None

    def set_state(self, key: str, value: str | None) -> None:
        if value is None:
            self._conn.execute("DELETE FROM state WHERE key = ?", (key,))
        else:
            self._conn.execute(
                "INSERT OR REPLACE INTO state (key, value, updated_at) "
                "VALUES (?, ?, strftime('%s','now'))",
                (key, value),
            )
        self._conn.commit()

    def get_state(self, key: str) -> str | None:
        row = self._conn.execute(
            "SELECT value FROM state WHERE key = ?", (key,)
        ).fetchone()
        return row["value"] if row else None

    def set_cloud_path(self, map_name: str, cloud_path: str) -> None:
        self._conn.execute(
            "UPDATE maps SET cloud_path = ? WHERE name = ?", (cloud_path, map_name)
        )
        self._conn.commit()

    def list_maps_with_pois(self) -> list[dict]:
        maps = self.list_maps()
        for m in maps:
            m["tags"] = [p["name"] for p in self.list_pois(m["name"])]
        return maps


class ControlledSpatialPlugin:
    """人工遥控建图与语义标记（controlled_spatial），与 unitree/g1 接口对齐。

    通过 Odin2 里程计位姿累积激光点云生成 .pcd 地图，支持开始/停止/取消建图、
    状态查询、地图管理、语义标记（tag）管理和已保存地图加载。
    暂不包含 navigation（navigate_to_tag / navigate_to_pose）。
    """

    def __init__(self, plugin_config: dict, namespace: str, ros2):
        self._config = plugin_config
        self._ns = namespace
        self._ros2 = ros2

        self._map_save_dir = plugin_config.get("map_save_dir", "/opt/phanthy-motus/data/maps")
        db_path = plugin_config.get("db_path", "/opt/phanthy-motus/data/mapping.db")
        self._voxel_size = float(plugin_config.get("voxel_size", 0.05))
        self._max_points = int(plugin_config.get("max_points", 5_000_000))
        self._odometry_topic = plugin_config.get(
            "odometry_topic", "/manifold/ODIN2/device0/odometry"
        )
        self._pointcloud_topic = plugin_config.get(
            "pointcloud_topic", "/manifold/ODIN2/device0/cloud/slam"
        )

        self._db = _MappingDB(db_path)

        self._is_mapping = False
        self._current_map: str | None = None
        self._active_map: str | None = None  # 当前活动地图（建图中或已加载）
        self._loaded_points: np.ndarray | None = None  # load_map 加载的点云（可选）
        self._current_pose: dict | None = None
        self._global_points: np.ndarray | None = None
        self._start_time: float | None = None
        self._frame_count = 0
        self._lock = threading.Lock()

        # 启动时从 DB 恢复上次的 active_map（持久化）
        saved_active = self._db.get_state("active_map")
        if saved_active and self._db.get_map(saved_active):
            self._active_map = saved_active
            print(f"[controlled_spatial] restored active_map='{saved_active}' from DB", flush=True)

        self._node = Node("t800_mapping", context=ros2.ctx_robot)
        ros2.executor_robot.add_node(self._node)
        self._odom_sub = None
        self._cloud_sub = None

        self._create_odom_subscription()

        print(
            f"[controlled_spatial] ready (odom={self._odometry_topic}, cloud={self._pointcloud_topic}, "
            f"open3d={_HAS_OPEN3D})",
            flush=True,
        )

    def get_tool(self) -> dict:
        return {
            "name": "controlled_spatial",
            "type": "actuator",
            "multiInstance": False,
            "description": (
                "T800 人工控制建图与语义标记卡片（与 unitree/g1 controlled_spatial 对齐）。"
                "人工遥控行走时按 Odin2 里程计位姿累积点云，生成并保存 .pcd 地图。"
                "支持开始/停止/取消建图、状态查询、地图管理、语义标记（tag）管理和已保存地图加载。"
            ),
            "inputSchema": action_schema(
                {
                    "start_mapping": (["map_name", "overwrite"], "开始建图，用遥控器或 loco 控制机器人行走。若同名地图已存在需 overwrite=true 才允许覆盖"),
                    "stop_mapping": ([], "停止建图，体素下采样后保存 .pcd 地图"),
                    "cancel_mapping": ([], "取消建图，丢弃已累积点云，不保存"),
                    "mapping_status": ([], "查询当前建图状态、实时位姿和已累积点数"),
                    "list_maps": ([], "列出所有已保存地图"),
                    "delete_map": (["map_name"], "删除指定地图及 .pcd 文件"),
                    "tag_place": (["name", "description"], "在当前位置打语义标记，关联到活动地图"),
                    "untag_place": (["name"], "删除指定名称的位置标记"),
                    "list_tags": ([], "列出当前活动地图的所有标记，含相对机器人的距离和方位"),
                    "load_map": (["map_name"], "加载已保存地图为活动地图（不做重定位，需手动将机器人放置在地图原点附近）"),
                },
                {
                    "map_name": {"type": "string", "description": "地图名称"},
                    "overwrite": {"type": "boolean", "description": "地图已存在时是否覆盖（默认 false，防止误操作覆盖已有地图）", "default": False},
                    "name": {"type": "string", "description": "标记名称"},
                    "description": {"type": "string", "description": "标记描述（可选）"},
                },
                "建图动作",
            ),
        }

    def start(self) -> None:
        pass

    def stop(self) -> None:
        if self._is_mapping:
            self._destroy_cloud_subscription()
            print("[controlled_spatial] driver shutdown, mapping stopped without saving", flush=True)
        self._destroy_odom_subscription()
        try:
            self._ros2.executor_robot.remove_node(self._node)
            self._node.destroy_node()
        except Exception:
            pass

    def dispatch(self, action: str, args: dict) -> dict:
        if action in ("start", "info"):
            return {"state": "ready"}
        if action == "stop":
            return {"state": "idle"}
        if action == "start_mapping":
            return self._start_mapping(args.get("map_name", ""), args.get("overwrite", False))
        if action == "stop_mapping":
            return self._stop_mapping()
        if action == "cancel_mapping":
            return self._cancel_mapping()
        if action == "mapping_status":
            return self._mapping_status()
        if action == "list_maps":
            return self._list_maps()
        if action == "delete_map":
            return self._delete_map(args.get("map_name", ""))
        if action == "tag_place":
            return self._tag_place(args.get("name", ""), args.get("description", ""))
        if action == "untag_place":
            return self._untag_place(args.get("name", ""))
        if action == "list_tags":
            return self._list_tags()
        if action == "load_map":
            return self._load_map(args.get("map_name", ""))
        return {"error": f"unknown controlled_spatial action: {action}"}

    # ── Action handlers ──────────────────────────────────────────────

    def _start_mapping(self, map_name: str, overwrite: bool = False) -> dict:
        map_name = str(map_name).strip()
        if not map_name:
            return {"error": "map_name is required"}
        with self._lock:
            if self._is_mapping:
                return {"error": f"already mapping '{self._current_map}'", "current_map": self._current_map}

        existing = self._db.get_map(map_name)
        if existing and not overwrite:
            return {
                "error": f"map '{map_name}' already exists. Use overwrite=true to overwrite, or choose a different name.",
                "existing_map": {
                    "name": existing["name"],
                    "point_count": existing.get("point_count", 0),
                    "created_at": existing.get("created_at"),
                },
            }
        if existing:
            print(f"[controlled_spatial] overwriting existing map '{map_name}' (overwrite=true)", flush=True)

        try:
            self._create_cloud_subscription()
        except Exception as exc:
            return {"error": f"failed to create ROS2 subscriptions: {exc}"}

        with self._lock:
            self._is_mapping = True
            self._current_map = map_name
            self._active_map = map_name
            self._current_pose = None
            self._global_points = None
            self._start_time = time.monotonic()
            self._frame_count = 0

        try:
            self._db.set_state("map_status", "mapping")
            self._db.set_state("active_map", map_name)
        except Exception as exc:
            print(f"[controlled_spatial] failed to persist map_status: {exc}", flush=True)

        print(f"[controlled_spatial] started mapping '{map_name}'", flush=True)
        return {
            "state": "mapping",
            "map_name": map_name,
            "message": "建图已开始，请用 virtual_gamepad 或 loco 控制机器人行走",
        }

    def _stop_mapping(self) -> dict:
        with self._lock:
            if not self._is_mapping:
                return {"error": "no active mapping session"}
            map_name = self._current_map
            start_time = self._start_time
            points = self._global_points

        self._destroy_cloud_subscription()
        elapsed = time.monotonic() - start_time if start_time else 0.0
        result = self._save_map(map_name, points)

        with self._lock:
            self._is_mapping = False
            self._current_map = None
            self._global_points = None
            self._current_pose = None
            self._start_time = None
            self._frame_count = 0
            # _active_map 保留为 map_name：地图仍处于活动状态，可继续 tag_place

        try:
            self._db.set_state("map_status", "idle")
        except Exception as exc:
            print(f"[controlled_spatial] failed to persist map_status: {exc}", flush=True)

        if "error" in result:
            print(f"[controlled_spatial] save failed for '{map_name}': {result['error']}", flush=True)
            return result

        result["elapsed_time"] = round(elapsed, 2)
        print(
            f"[controlled_spatial] saved '{map_name}': {result['point_count']} points "
            f"({elapsed:.1f}s) -> {result['pcd_path']}",
            flush=True,
        )
        return result

    def _cancel_mapping(self) -> dict:
        with self._lock:
            if not self._is_mapping:
                return {"error": "no active mapping session"}
            map_name = self._current_map

        self._destroy_cloud_subscription()

        with self._lock:
            self._is_mapping = False
            self._current_map = None
            self._global_points = None
            self._current_pose = None
            self._start_time = None
            self._frame_count = 0
            self._active_map = None

        try:
            self._db.set_state("map_status", "idle")
            self._db.set_state("active_map", None)
        except Exception as exc:
            print(f"[controlled_spatial] failed to persist map_status: {exc}", flush=True)

        print(f"[controlled_spatial] cancelled mapping '{map_name}'", flush=True)
        return {"state": "cancelled", "map_name": map_name}

    def _mapping_status(self) -> dict:
        with self._lock:
            active_map = self._active_map
            if not self._is_mapping:
                return {"state": "idle", "is_mapping": False, "active_map": active_map}
            pose = dict(self._current_pose) if self._current_pose else None
            point_count = int(len(self._global_points)) if self._global_points is not None else 0
            elapsed = time.monotonic() - self._start_time if self._start_time else 0.0
            return {
                "state": "mapping",
                "is_mapping": True,
                "current_map": self._current_map,
                "active_map": active_map,
                "current_pose": pose,
                "point_count": point_count,
                "frame_count": self._frame_count,
                "elapsed_time": round(elapsed, 2),
            }

    def _list_maps(self) -> dict:
        try:
            return {"maps": self._db.list_maps()}
        except Exception as exc:
            return {"error": f"failed to list maps: {exc}"}

    def _delete_map(self, map_name: str) -> dict:
        map_name = str(map_name).strip()
        if not map_name:
            return {"error": "map_name is required"}
        with self._lock:
            if self._is_mapping and self._current_map == map_name:
                return {"error": f"cannot delete active map '{map_name}'. Stop or cancel mapping first."}
        try:
            if self._db.delete_map(map_name):
                with self._lock:
                    if self._active_map == map_name:
                        self._active_map = None
                return {"state": "deleted", "map_name": map_name}
            return {"error": f"map '{map_name}' not found"}
        except Exception as exc:
            return {"error": f"failed to delete map: {exc}"}

    # ── Tag / map-load handlers ──────────────────────────────────────

    def _tag_place(self, name: str, description: str = "") -> dict:
        name = str(name).strip()
        if not name:
            return {"error": "name is required"}
        pose = self._get_pose()
        if pose is None:
            return {"error": "no odometry pose available yet"}
        active_map = self._active_map
        if not active_map:
            return {"error": "no active map; call start_mapping or load_map first"}
        self._db.add_poi(
            name=name,
            x=pose["x"],
            y=pose["y"],
            z=pose.get("z", 0.0),
            yaw=pose.get("yaw", 0.0),
            map_name=active_map,
            description=description,
        )
        print(f"[controlled_spatial] tagged place '{name}' on map '{active_map}'", flush=True)
        return {"status": "tagged", "name": name, "pose": pose, "map": active_map}

    def _untag_place(self, name: str) -> dict:
        name = str(name).strip()
        if not name:
            return {"error": "name is required"}
        active_map = self._active_map
        if not active_map:
            return {"error": "no active map; call start_mapping or load_map first"}
        if self._db.delete_poi(name, active_map):
            print(f"[controlled_spatial] untagged place '{name}' from map '{active_map}'", flush=True)
            return {"status": "deleted", "name": name}
        return {"error": f"tag '{name}' not found in map '{active_map}'"}

    def _list_tags(self) -> dict:
        active_map = self._active_map
        if not active_map:
            return {"error": "no active map; call start_mapping or load_map first"}
        pois = self._db.list_pois(active_map)
        pose = self._get_pose()
        tags = []
        for poi in pois:
            entry = {
                "name": poi["name"],
                "description": poi["description"],
                "x": poi["x"],
                "y": poi["y"],
                "z": poi["z"],
                "yaw": poi["yaw"],
            }
            if pose:
                dx = poi["x"] - pose["x"]
                dy = poi["y"] - pose["y"]
                dist = math.sqrt(dx * dx + dy * dy)
                # Rotate target vector into robot frame
                cos_yaw = math.cos(-pose["yaw"])
                sin_yaw = math.sin(-pose["yaw"])
                rx = dx * cos_yaw - dy * sin_yaw
                ry = dx * sin_yaw + dy * cos_yaw
                entry["distance"] = round(dist, 2)
                entry["bearing"] = self._bearing_label(rx, ry)
            tags.append(entry)
        return {"tags": tags, "map": active_map}

    def _load_map(self, map_name: str) -> dict:
        map_name = str(map_name).strip()
        if not map_name:
            return {"error": "map_name is required"}
        if self._is_mapping:
            return {"error": "cannot load map while mapping; call stop_mapping first"}
        map_info = self._db.get_map(map_name)
        if not map_info:
            return {"error": f"map '{map_name}' not found"}
        self._active_map = map_name
        self._loaded_points = None  # 第一期不加载点云到内存，仅设置活动地图
        self._db.set_state("active_map", map_name)
        print(f"[controlled_spatial] loaded map '{map_name}' as active", flush=True)
        return {
            "status": "loaded",
            "map_name": map_name,
            "pcd_path": map_info["pcd_path"],
            "note": "T800 无 SLAM 重定位服务，请手动将机器人放置在地图原点附近",
        }

    # ── Helpers ──────────────────────────────────────────────────────

    def _get_pose(self) -> dict | None:
        """Thread-safe deep copy of the current odometry pose."""
        with self._lock:
            return dict(self._current_pose) if self._current_pose else None

    @staticmethod
    def _bearing_label(dx: float, dy: float) -> str:
        """Convert delta (x=forward, y=left) to 8-direction bearing label."""
        angle = math.atan2(dy, dx)  # radians, 0=forward, pi/2=left
        deg = math.degrees(angle)
        if -22.5 <= deg < 22.5:
            return "front"
        elif 22.5 <= deg < 67.5:
            return "left_front"
        elif 67.5 <= deg < 112.5:
            return "left"
        elif 112.5 <= deg < 157.5:
            return "left_behind"
        elif -67.5 <= deg < -22.5:
            return "right_front"
        elif -112.5 <= deg < -67.5:
            return "right"
        elif -157.5 <= deg < -112.5:
            return "right_behind"
        else:
            return "behind"

    # ── ROS2 subscriptions ───────────────────────────────────────────

    def _create_odom_subscription(self) -> None:
        from nav_msgs.msg import Odometry

        self._odom_sub = self._node.create_subscription(
            Odometry, self._odometry_topic, self._on_odometry, _BEST_EFFORT
        )

    def _create_cloud_subscription(self) -> None:
        from sensor_msgs.msg import PointCloud2

        self._cloud_sub = self._node.create_subscription(
            PointCloud2, self._pointcloud_topic, self._on_pointcloud, _BEST_EFFORT
        )

    def _destroy_cloud_subscription(self) -> None:
        if self._cloud_sub is not None:
            try:
                self._node.destroy_subscription(self._cloud_sub)
            except Exception:
                pass
            self._cloud_sub = None

    def _destroy_odom_subscription(self) -> None:
        if self._odom_sub is not None:
            try:
                self._node.destroy_subscription(self._odom_sub)
            except Exception:
                pass
            self._odom_sub = None

    # ── Callbacks ────────────────────────────────────────────────────

    def _on_odometry(self, msg) -> None:
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        yaw = self._quaternion_to_yaw(orientation.x, orientation.y, orientation.z, orientation.w)
        with self._lock:
            self._current_pose = {
                "x": float(position.x),
                "y": float(position.y),
                "z": float(position.z),
                "qx": float(orientation.x),
                "qy": float(orientation.y),
                "qz": float(orientation.z),
                "qw": float(orientation.w),
                "yaw": round(yaw, 4),
            }

    def _on_pointcloud(self, msg) -> None:
        with self._lock:
            if not self._is_mapping:
                return
            if self._current_pose is None:
                return
            if self._global_points is not None and len(self._global_points) >= self._max_points:
                return
            pose = dict(self._current_pose)

        try:
            points = self._parse_pointcloud2(msg)
        except Exception as exc:
            print(f"[controlled_spatial] pointcloud parse error: {exc}", flush=True)
            return

        if points is None or len(points) == 0:
            return

        transformed = self._transform_points(points, pose)

        with self._lock:
            if self._global_points is None:
                self._global_points = transformed
            else:
                self._global_points = np.vstack([self._global_points, transformed])
            self._frame_count += 1

    # ── Point cloud helpers ──────────────────────────────────────────

    @staticmethod
    def _parse_pointcloud2(msg) -> np.ndarray | None:
        fields = {f.name: f for f in msg.fields}
        if not all(name in fields for name in ("x", "y", "z")):
            return None

        x_off = fields["x"].offset
        y_off = fields["y"].offset
        z_off = fields["z"].offset
        point_step = msg.point_step
        num_points = msg.width * msg.height

        if num_points == 0 or point_step == 0:
            return None

        raw = np.frombuffer(msg.data, dtype=np.uint8)
        raw = raw[: num_points * point_step].reshape(num_points, point_step)

        x = np.frombuffer(raw[:, x_off : x_off + 4].tobytes(), dtype=np.float32)
        y = np.frombuffer(raw[:, y_off : y_off + 4].tobytes(), dtype=np.float32)
        z = np.frombuffer(raw[:, z_off : z_off + 4].tobytes(), dtype=np.float32)

        points = np.column_stack([x, y, z]).astype(np.float64)
        valid = np.isfinite(points).all(axis=1)
        return points[valid]

    @staticmethod
    def _transform_points(points: np.ndarray, pose: dict) -> np.ndarray:
        qx, qy, qz, qw = pose["qx"], pose["qy"], pose["qz"], pose["qw"]
        # Normalize quaternion to avoid scale drift in the rotation matrix.
        qnorm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
        if qnorm > 0:
            qx, qy, qz, qw = qx / qnorm, qy / qnorm, qz / qnorm, qw / qnorm
        R = np.array(
            [
                [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
                [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
                [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)],
            ]
        )
        rotated = points @ R.T
        rotated[:, 0] += pose["x"]
        rotated[:, 1] += pose["y"]
        rotated[:, 2] += pose["z"]
        return rotated

    @staticmethod
    def _quaternion_to_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
        return math.atan2(
            2 * (qw * qz + qx * qy),
            1 - 2 * (qy * qy + qz * qz),
        )

    # ── Save map ─────────────────────────────────────────────────────

    def _save_map(self, map_name: str, points: np.ndarray | None) -> dict:
        if points is None or len(points) == 0:
            return {"error": "no point cloud data to save"}

        os.makedirs(self._map_save_dir, exist_ok=True)
        pcd_path = os.path.join(self._map_save_dir, f"{map_name}.pcd")

        try:
            points = self._voxel_downsample(points, self._voxel_size)
        except Exception as exc:
            print(f"[controlled_spatial] voxel downsample failed, saving raw: {exc}", flush=True)

        point_count = int(len(points))

        try:
            if _HAS_OPEN3D:
                pcd = o3d.geometry.PointCloud()
                pcd.points = o3d.utility.Vector3dVector(points)
                o3d.io.write_point_cloud(pcd_path, pcd)
            else:
                self._write_pcd_ascii(pcd_path, points)
        except Exception as exc:
            return {"error": f"failed to write PCD: {exc}"}

        try:
            self._db.add_map(map_name, pcd_path, point_count)
        except Exception as exc:
            return {"error": f"failed to update database: {exc}", "pcd_path": pcd_path, "point_count": point_count}

        return {"state": "saved", "map_name": map_name, "pcd_path": pcd_path, "point_count": point_count}

    @staticmethod
    def _voxel_downsample(points: np.ndarray, voxel_size: float) -> np.ndarray:
        if voxel_size <= 0:
            return points
        voxel_idx = np.floor(points / voxel_size).astype(np.int64)
        _, unique_idx = np.unique(voxel_idx, axis=0, return_index=True)
        return points[unique_idx]

    @staticmethod
    def _write_pcd_ascii(path: str, points: np.ndarray) -> None:
        n = len(points)
        header = (
            "# .PCD v0.7 - Point Cloud Data file format\n"
            "VERSION 0.7\n"
            "FIELDS x y z\n"
            "SIZE 4 4 4\n"
            "TYPE F F F\n"
            "COUNT 1 1 1\n"
            f"WIDTH {n}\n"
            "HEIGHT 1\n"
            "VIEWPOINT 0 0 0 1 0 0 0\n"
            f"POINTS {n}\n"
            "DATA ascii\n"
        )
        with open(path, "w") as f:
            f.write(header)
            for p in points:
                f.write(f"{p[0]:.6f} {p[1]:.6f} {p[2]:.6f}\n")


GAIT_PROFILES: dict[str, dict] = {
    "basic": {
        "motion_states": ("walk", "rl_basic"),
        "description": "基础行走（自动适配新版 rl_basic 与旧版 walk 状态名）",
    },
    "balanced": {
        "motion_states": ("lower_body_balance",),
        "description": "下肢平衡步态",
    },
}


class GaitPlugin:
    """Version-adaptive gait selector over the public motion-state API.

    The public T800 Native SDK stores walking policy settings under
    ``assets/config/t800/.../*.yaml``.  It does not define the historical
    ``config/rl_basic/gait.json`` contract or fields such as ``step_height``.
    Exposing those guessed files produced successful-looking writes that the
    robot never consumed.  This plugin only uses the documented ROS motion
    state interface and resolves the ``rl_basic``/``walk`` version difference
    from the robot's advertised transitions.
    """

    PREFIX = "gait"

    def __init__(self, config: dict, motion_mode: "MotionModePlugin", state: StatePlugin):
        self._motion_mode = motion_mode
        self._state = state
        plugin_config = config.get("plugins", {}).get("gait", {})
        configured_basic = plugin_config.get("basic_motion_states", ["walk", "rl_basic"])
        if not isinstance(configured_basic, (list, tuple)) or not configured_basic:
            configured_basic = ["walk", "rl_basic"]
        basic_states = tuple(str(item) for item in configured_basic if str(item))
        self._profiles = {
            name: {
                **definition,
                "motion_states": basic_states if name == "basic" else definition["motion_states"],
            }
            for name, definition in GAIT_PROFILES.items()
        }

    def get_tool(self) -> dict:
        return {
            "name": self.PREFIX,
            "type": "actuator",
            "multiInstance": False,
            "description": (
                "T800 步态选择，通过官方 motion state 接口切换基础行走、"
                "下肢平衡步态；会按固件返回的 available transitions 判定可用性。"
            ),
            "inputSchema": action_schema(
                {
                    "start": ([], "启动卡片（actuator 生命周期）"),
                    "stop": ([], "停止卡片（不强制改变机器人状态）"),
                    "info": ([], "查询当前步态与可用性"),
                    "status": ([], "查询当前步态与可用性"),
                    "list": ([], "列出步态档位与当前固件的可用性"),
                    "select": (["gait", "force", "wait"], "选择步态档位"),
                },
                {
                    "gait": {
                        "type": "string",
                        "enum": list(self._profiles),
                        "description": "步态档位",
                    },
                    "force": {
                        "type": "boolean",
                        "description": "固件未声明该转换时仍发送（默认 false）",
                    },
                    "wait": {
                        "type": "boolean",
                        "description": "等待 motion state 反馈（默认 true）",
                    },
                },
                "步态动作",
            ),
        }

    def start(self) -> None:
        pass

    def stop(self) -> None:
        pass

    def dispatch(self, action: str, args: dict) -> dict:
        if action == "start":
            status = self._status()
            status["gait_state"] = status.pop("state")
            status["state"] = "ready"
            return status
        if action in ("info", "status"):
            return self._status()
        if action == "stop":
            return {"state": "idle"}
        if action == "list":
            return self._list_profiles()
        if action == "select":
            return self._select(args)
        return {"error": f"unknown gait action: {action}"}

    def _resolve(self, profile: str, current: str, available: list[str]) -> str | None:
        candidates = self._profiles[profile]["motion_states"]
        if current in candidates:
            return current
        for candidate in candidates:
            if candidate in available:
                return candidate
        return None

    def _profile_for_motion(self, motion: str) -> str | None:
        for name, definition in self._profiles.items():
            if motion in definition["motion_states"]:
                return name
        return None

    def _status(self) -> dict:
        current, available = self._state.current_motion()
        profile = self._profile_for_motion(current)
        return {
            "state": "active" if profile else "inactive",
            "gait": profile,
            "motion_state": current,
            "available_motion_states": available,
            "available_gaits": [
                name
                for name in self._profiles
                if self._resolve(name, current, available) is not None
            ],
        }

    def _list_profiles(self) -> dict:
        current, available = self._state.current_motion()
        profiles = []
        for name, definition in self._profiles.items():
            resolved = self._resolve(name, current, available)
            profiles.append({
                "name": name,
                "description": definition["description"],
                "motion_states": list(definition["motion_states"]),
                "resolved_motion_state": resolved,
                "available": resolved is not None,
                "active": current in definition["motion_states"],
            })
        return {
            "state": "ready",
            "current_motion_state": current,
            "available_motion_states": available,
            "profiles": profiles,
        }

    def _select(self, args: dict) -> dict:
        profile = str(args.get("gait", ""))
        if profile not in self._profiles:
            return {"error": f"unknown gait: {profile}", "gaits": list(self._profiles)}
        current, available = self._state.current_motion()
        force = bool(args.get("force", False))
        target = self._resolve(profile, current, available)
        if target is None:
            if not force:
                return {
                    "error": f"gait {profile} is not available from {current or 'unknown'}",
                    "available_motion_states": available,
                    "candidate_motion_states": list(self._profiles[profile]["motion_states"]),
                }
            target = self._profiles[profile]["motion_states"][0]

        if current == target:
            return {
                "state": "completed",
                "gait": profile,
                "current": current,
                "available": available,
            }

        result = self._motion_mode.dispatch("switch", {
            "target": target,
            "force": force,
            "wait": bool(args.get("wait", True)),
        })
        return {"gait": profile, "motion_state": target, **result}


class MotionRecorderPlugin:
    """Record, save, and replay T800 joint trajectories.

    Recording captures joint positions from the robot's joint state topic
    at a configurable rate. Playback resamples the recorded upper-body path
    and publishes it continuously through the official joint override topic.

    ⚠  Playback requires the robot to be in ``lower_body_balance`` mode.
    """

    PREFIX = "motion_recorder"

    # Default recording rate (Hz) — 20 Hz captures 50ms intervals, enough
    # for smooth playback without overwhelming the trajectory buffer.
    _DEFAULT_RECORD_HZ = 20.0
    _MAX_FRAMES = 6000  # 5 minutes at 20 Hz
    _DEFAULT_PLAYBACK_STIFFNESS = (
        200.0, 30.0, 30.0, 15.0, 30.0, 15.0,
        40.0, 40.0, 20.0, 40.0, 20.0, 100.0, 100.0,
    )
    _DEFAULT_PLAYBACK_DAMPING = (
        3.0, 1.0, 1.0, 1.0, 1.0, 1.0,
        1.0, 1.0, 1.0, 1.0, 1.0, 3.0, 3.0,
    )

    def __init__(self, config: dict, namespace: str, ros2):
        self._config = config
        self._ns = namespace
        self._node = None
        self._ros2 = ros2

        # Recording state
        self._record_hz = float(
            config.get("plugins", {})
            .get("motion_recorder", {})
            .get("record_hz", self._DEFAULT_RECORD_HZ)
        )
        if not math.isfinite(self._record_hz) or self._record_hz <= 0:
            raise ValueError("motion_recorder.record_hz must be a positive finite number")
        self._record_period_sec = 1.0 / self._record_hz
        recorder_config = config.get("plugins", {}).get("motion_recorder", {})
        self._upper_indices = tuple(T800_JOINT_GROUPS["upper_body"])
        self._playback_rate_hz = clamp(
            recorder_config.get("playback_rate_hz", 100.0), 50.0, 200.0
        )
        self._entry_blend_sec = clamp(
            recorder_config.get("entry_blend_sec", 0.5), 0.1, 3.0
        )
        self._reset_timeout_sec = clamp(
            recorder_config.get("reset_timeout_sec", 15.0), 1.0, 120.0
        )
        self._playback_stiffness = float_list(
            recorder_config.get("playback_stiffness", self._DEFAULT_PLAYBACK_STIFFNESS),
            "playback_stiffness",
            size=len(self._upper_indices),
        )
        self._playback_damping = float_list(
            recorder_config.get("playback_damping", self._DEFAULT_PLAYBACK_DAMPING),
            "playback_damping",
            size=len(self._upper_indices),
        )
        record_dir = (
            config.get("plugins", {})
            .get("motion_recorder", {})
            .get("recordings_dir", "")
        )
        self._recordings_dir = Path(
            record_dir if record_dir else
            os.path.join(os.path.dirname(__file__), RECORDINGS_DIR)
        )
        self._recordings_dir.mkdir(parents=True, exist_ok=True)

        self._lock = threading.RLock()
        self._frames: list[dict] = []
        self._recording = False
        self._record_thread: threading.Thread | None = None
        self._record_stop = threading.Event()
        self._record_label = ""
        self._record_start_ms = 0
        self._record_session = 0
        self._last_sample_at: float | None = None
        self._last_recording: dict | None = None
        self._joint_state_sub = None
        self._state = None
        self._motion_mode = None
        self._needs_reset = False
        self._reset_request_id: int | None = None
        self._reset_action_id: str | None = None
        self._last_reset: dict | None = None

        # Playback state
        self._playback_thread: threading.Thread | None = None
        self._playback_stop = threading.Event()
        self._playing = False
        self._playback_label = ""
        self._playback_frame = 0
        self._playback_action_id: str | None = None
        self._last_playback_error: str | None = None
        self._override_publisher = None
        self._override_message_type = None
        self._acp_notify = _t800_acp_notify

        # Latest joint state cache
        self._latest_joint_positions: list[float] | None = None

    def get_tool(self) -> dict:
        input_schema = action_schema(
            {
                "record_start": (["label", "duration"], "开始录制关节轨迹"),
                "record_stop": ([], "停止录制并自动保存"),
                "play": (["name", "speed_scale"], "100Hz 平滑回放指定录制文件"),
                "stop_playback": ([], "停止回放"),
                "reset": ([], "恢复 lower_body_balance 默认姿态，完成后允许下一次录制"),
                "save": (["name", "label"], "将当前录制保存到文件"),
                "load": (["name"], "从文件加载录制到内存"),
                "list": ([], "列出所有已保存的录制文件"),
                "delete": (["name"], "删除指定录制文件"),
                "status": ([], "查询录制/回放状态"),
                "info": ([], "同 status"),
            },
            {
                "label": {
                    "type": "string",
                    "description": "录制或保存的标签名",
                },
                "name": {
                    "type": "string",
                    "description": "录制文件名（不含 .json）",
                },
                "duration": {
                    "type": "number",
                    "description": "自动停止时间（秒），0=持续录制直到手动停止",
                },
                "speed_scale": {
                    "type": "number",
                    "minimum": 0.1,
                    "maximum": 5.0,
                    "description": "回放速度倍率，1.0=原速",
                },
            },
            "运动录制/回放动作",
        )
        input_schema["x-completion"] = {
            "actions": ["play", "reset"],
            "timeout": 3600,
        }
        return {
            "name": self.PREFIX,
            "type": "actuator",
            "multiInstance": False,
            "description": (
                "T800 全身运动录制与回放 — 录制关节轨迹到文件，"
                "以 100Hz 平滑回放已录制的上肢轨迹。支持录制/停止/回放/复位/管理。"
                "录制与回放均需 lower_body_balance；每段动作后需复位才可继续录制。"
            ),
            "inputSchema": input_schema,
        }

    def start(self) -> None:
        from rclpy.node import Node
        from interface_protocol.msg import JointOverrideCommand, JointState

        self._node = Node("t800_motion_recorder", context=self._ros2.ctx_robot)
        self._ros2.executor_robot.add_node(self._node)

        # Subscribe to joint state for recording
        topic = self._config.get("topics", {}).get("joint_state", "/hardware/joint_state")
        self._joint_state_sub = self._node.create_subscription(
            JointState, topic, self._on_joint_state,
            self._best_effort_qos(),
        )
        self._override_message_type = JointOverrideCommand
        self._override_publisher = self._node.create_publisher(
            JointOverrideCommand,
            self._config.get("topics", {}).get(
                "joint_override", "/motion/joint_override_command"
            ),
            _RELIABLE,
        )

    def _best_effort_qos(self):
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
        return QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=3,
            durability=DurabilityPolicy.VOLATILE,
        )

    def stop(self) -> None:
        self._finalize_recording(stop_reason="shutdown")
        self._stop_playback()
        with self._lock:
            reset_action_id = self._reset_action_id
            reset_request_id = self._reset_request_id
            self._reset_action_id = None
            self._reset_request_id = None
        if reset_action_id:
            self._acp_notify(reset_action_id, "cancelled", {
                "request_id": reset_request_id,
                "reason": "plugin_stopped",
            }, self.PREFIX)
        if self._node:
            try:
                self._node.destroy_node()
            except Exception:
                pass

    # ── ROS2 callbacks ────────────────────────────────────────────────────────

    def _on_joint_state(self, msg) -> None:
        positions = [float(p) for p in msg.position]
        velocities = [float(v) for v in msg.velocity]
        sampled_at = time.monotonic()
        with self._lock:
            self._latest_joint_positions = list(positions)
            recording = self._recording
        if recording and self._lower_body_balance_error("recording"):
            self._finalize_recording(stop_reason="motion_state_changed")
            return
        with self._lock:
            if not self._recording or len(self._frames) >= self._MAX_FRAMES:
                return
            if (
                self._last_sample_at is not None
                and sampled_at - self._last_sample_at < self._record_period_sec
            ):
                return
            self._last_sample_at = sampled_at
            self._frames.append({
                "timestamp": _now_ms() - self._record_start_ms,
                "positions": positions,
                "velocities": velocities,
            })

    # ── Dispatch ──────────────────────────────────────────────────────────────

    def dispatch(self, action: str, args: dict) -> dict:
        if action == "start":
            status = self._status()
            status["activity_state"] = status.pop("state")
            status["state"] = "ready"
            return status
        if action in ("info", "status"):
            return self._status()
        if action == "stop":
            recording_result = self._finalize_recording(stop_reason="lifecycle")
            self._playback_stop.set()
            result = {"state": "idle"}
            if recording_result.get("state") == "saved":
                result["recording_result"] = recording_result
            return result
        if action == "record_start":
            with self._lock:
                if self._recording:
                    return {
                        "state": "recording",
                        "recording": True,
                        "already_recording": True,
                        "label": self._record_label,
                        "frames": len(self._frames),
                        "session_id": self._record_session,
                    }
                if self._playing:
                    return {"error": "cannot record while playing"}
            return self._record_start(args)
        if action == "record_stop":
            return self._record_stop_action()
        if action == "play":
            return self._play(args)
        if action == "stop_playback":
            return self._stop_playback()
        if action == "reset":
            return self._reset_pose()
        if action == "save":
            return self._save(args)
        if action == "load":
            return self._load(args)
        if action == "list":
            return self._list_recordings()
        if action == "delete":
            return self._delete(args)
        return {"error": f"unknown motion_recorder action: {action}"}

    # ── Recording ─────────────────────────────────────────────────────────────

    def _record_start(self, args: dict) -> dict:
        gate_error = self._lower_body_balance_error("recording")
        if gate_error:
            return gate_error
        self._refresh_reset_state()
        with self._lock:
            if self._needs_reset:
                reason = (
                    "reset is still in progress"
                    if self._reset_request_id is not None
                    else "reset required before starting the next recording"
                )
                return {"error": reason, "needs_reset": True}
        try:
            duration = float(args.get("duration", 0.0))
        except (TypeError, ValueError):
            return {"error": "duration must be a non-negative finite number"}
        if not math.isfinite(duration) or duration < 0:
            return {"error": "duration must be a non-negative finite number"}

        with self._lock:
            if self._recording:
                return {
                    "state": "recording",
                    "recording": True,
                    "already_recording": True,
                    "label": self._record_label,
                    "frames": len(self._frames),
                    "session_id": self._record_session,
                }
            self._frames = []
            self._recording = True
            label = str(args.get("label", "")).strip()
            self._record_label = label or f"recording_{int(time.time())}"
            self._record_start_ms = _now_ms()
            self._last_sample_at = None
            self._record_session += 1
            session_id = self._record_session
            self._record_stop.clear()

        # Auto-stop timer if duration specified
        if duration > 0:
            def auto_stop():
                if not self._record_stop.wait(duration):
                    self._finalize_recording(
                        expected_session=session_id,
                        stop_reason="duration",
                    )
            self._record_thread = threading.Thread(
                target=auto_stop,
                daemon=True,
                name=f"t800-motion-record-{session_id}",
            )
            self._record_thread.start()

        return {
            "state": "recording",
            "recording": True,
            "label": self._record_label,
            "duration": duration if duration > 0 else "unlimited",
            "record_hz": self._record_hz,
            "session_id": session_id,
            "joint_data_available": self._latest_joint_positions is not None,
            "note": "recording started; use record_stop to stop and save",
        }

    def _record_stop_action(self) -> dict:
        return self._finalize_recording(stop_reason="manual")

    def _finalize_recording(
        self,
        *,
        expected_session: int | None = None,
        stop_reason: str,
    ) -> dict:
        with self._lock:
            if not self._recording or (
                expected_session is not None
                and expected_session != self._record_session
            ):
                return {
                    "state": "idle",
                    "recording": False,
                    "already_stopped": True,
                    "last_recording": dict(self._last_recording) if self._last_recording else None,
                }
            self._recording = False
            self._record_stop.set()
            frames = list(self._frames)
            label = self._record_label
            session_id = self._record_session
            if frames:
                self._needs_reset = True
                self._reset_request_id = None

        # Auto-save
        safe_name = re.sub(r"[^A-Za-z0-9_.-]+", "_", label).strip("._")
        safe_name = safe_name or f"recording_{int(time.time())}"
        save_path = self._recordings_dir / f"{safe_name}.json"
        frame_count = len(frames)
        metadata = {
            "label": label,
            "frames": frame_count,
            "duration_ms": frames[-1]["timestamp"] if frames else 0,
            "recorded_at": _now_ms(),
            "record_hz": self._record_hz,
            "stop_reason": stop_reason,
            "session_id": session_id,
        }
        recording = {
            "metadata": metadata,
            "frames": frames,
        }
        try:
            save_path.write_text(
                json.dumps(recording, ensure_ascii=False, indent=2),
                encoding="utf-8",
            )
        except OSError as exc:
            result = {
                "state": "error",
                "recording": False,
                "error": f"save failed: {exc}",
                "label": label,
                "frames": frame_count,
                "stop_reason": stop_reason,
                "session_id": session_id,
            }
            with self._lock:
                self._last_recording = dict(result)
            return result

        result = {
            "state": "saved",
            "recording": False,
            "label": label,
            "frames": frame_count,
            "duration_ms": metadata["duration_ms"],
            "file": str(save_path),
            "stop_reason": stop_reason,
            "session_id": session_id,
        }
        with self._lock:
            self._last_recording = dict(result)
        return result

    # ── Playback ──────────────────────────────────────────────────────────────

    def _play(self, args: dict) -> dict:
        gate_error = self._lower_body_balance_error("playback")
        if gate_error:
            return gate_error
        with self._lock:
            if self._playing:
                return {"error": "already playing; stop first"}
            if self._recording:
                return {"error": "cannot play while recording"}

        name = str(args.get("name", ""))
        if not name:
            # Try to use current buffer
            with self._lock:
                if not self._frames:
                    return {"error": "no recording loaded; specify name or record first"}
                frames = list(self._frames)
                label = "buffer"
        else:
            safe_name = name.replace(" ", "_").replace("/", "_")
            file_path = self._recordings_dir / f"{safe_name}.json"
            if not file_path.exists():
                return {"error": f"recording not found: {name}"}
            try:
                data = json.loads(file_path.read_text(encoding="utf-8"))
                frames = data.get("frames", [])
                label = data.get("metadata", {}).get("label", name)
            except (json.JSONDecodeError, OSError) as exc:
                return {"error": f"failed to load recording: {exc}"}

        if not frames:
            return {"error": "recording is empty"}

        speed_scale = clamp(float(args.get("speed_scale", 1.0)), 0.1, 5.0)
        try:
            with self._lock:
                current_positions = list(self._latest_joint_positions or [])
            samples = resample_joint_trajectory(
                frames,
                joint_indices=self._upper_indices,
                current_positions=current_positions,
                playback_rate_hz=self._playback_rate_hz,
                speed_scale=speed_scale,
                entry_blend_sec=self._entry_blend_sec,
            )
        except (TypeError, ValueError) as exc:
            return {"error": f"invalid recording: {exc}"}
        self._playback_stop.clear()
        action_id = f"t800_motion_play_{uuid4().hex[:12]}"

        with self._lock:
            self._playing = True
            self._playback_label = label
            self._playback_frame = 0
            self._playback_action_id = action_id
            self._last_playback_error = None

        def run():
            published = 0
            try:
                period = 1.0 / self._playback_rate_hz
                deadline = time.monotonic()
                for i, (position, velocity) in enumerate(samples):
                    if self._playback_stop.is_set():
                        break
                    mode_error = self._lower_body_balance_error("playback")
                    if mode_error:
                        raise RuntimeError(mode_error["error"])
                    now = time.monotonic()
                    if now - deadline > period:
                        deadline = now
                    wait_time = deadline - time.monotonic()
                    if wait_time > 0 and self._playback_stop.wait(wait_time):
                        break

                    with self._lock:
                        self._playback_frame = min(
                            len(frames) - 1,
                            int(i * len(frames) / max(1, len(samples))),
                        )
                        self._playback_label = label
                    self._publish_override(position, velocity, weight=1.0)
                    published += 1
                    deadline += period
            except Exception as exc:
                with self._lock:
                    self._last_playback_error = str(exc)
            finally:
                try:
                    self._publish_override_release()
                finally:
                    with self._lock:
                        if published:
                            self._needs_reset = True
                            self._reset_request_id = None
                        self._playing = False
                        self._playback_label = ""
                        error = self._last_playback_error
                        if self._playback_action_id == action_id:
                            self._playback_action_id = None
                    completion = (
                        "cancelled"
                        if self._playback_stop.is_set()
                        else ("error" if error else "completed")
                    )
                    self._acp_notify(action_id, completion, {
                        "label": label,
                        "frames": len(frames),
                        "samples_published": published,
                        "error": error,
                    }, self.PREFIX)

        self._playback_thread = threading.Thread(target=run, daemon=True, name="t800-motion-playback")
        self._playback_thread.start()

        return {
            "state": "playing",
            "action_id": action_id,
            "label": label,
            "frames": len(frames),
            "samples": len(samples),
            "playback_rate_hz": self._playback_rate_hz,
            "entry_blend_sec": self._entry_blend_sec,
            "control_path": "joint_override",
            "speed_scale": speed_scale,
            "estimated_duration_s": round(
                self._entry_blend_sec
                + (frames[-1]["timestamp"] - frames[0]["timestamp"]) / 1000.0 / speed_scale,
                1,
            ),
        }

    def _publish_override(self, position: list[float], velocity: list[float], *, weight: float) -> None:
        if self._override_publisher is None or self._override_message_type is None:
            raise RuntimeError("joint override publisher is not initialized")
        msg = self._override_message_type()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.header.frame_id = ""
        msg.weight = float(weight)
        msg.joint_indices = list(self._upper_indices)
        msg.position = list(position)
        msg.velocity = list(velocity)
        msg.feed_forward_torque = [0.0] * len(self._upper_indices)
        msg.torque = [0.0] * len(self._upper_indices)
        msg.stiffness = list(self._playback_stiffness)
        msg.damping = list(self._playback_damping)
        self._override_publisher.publish(msg)

    def _publish_override_release(self) -> None:
        if self._override_publisher is None:
            return
        zeros = [0.0] * len(self._upper_indices)
        try:
            self._publish_override(zeros, zeros, weight=0.0)
        except Exception as exc:
            with self._lock:
                self._last_playback_error = str(exc)

    def _stop_playback(self) -> dict:
        self._playback_stop.set()
        with self._lock:
            was_playing = self._playing
            frame = self._playback_frame
            thread = self._playback_thread
        if (
            thread is not None
            and thread is not threading.current_thread()
            and thread.is_alive()
        ):
            thread.join(timeout=1.0)
        with self._lock:
            self._playing = False
        self._publish_override_release()
        return {
            "state": "stopped" if was_playing else "idle",
            "frames_played": frame,
        }

    # ── Save/Load ─────────────────────────────────────────────────────────────

    def _save(self, args: dict) -> dict:
        name = str(args.get("name", ""))
        if not name:
            return {"error": "name is required"}
        label = str(args.get("label", name))
        safe_name = name.replace(" ", "_").replace("/", "_")
        file_path = self._recordings_dir / f"{safe_name}.json"

        with self._lock:
            if not self._frames:
                return {"error": "no frames in buffer to save"}
            frames = list(self._frames)
            metadata = {
                "label": label,
                "frames": len(frames),
                "duration_ms": frames[-1]["timestamp"] if frames else 0,
                "recorded_at": _now_ms(),
            }

        recording = {"metadata": metadata, "frames": frames}
        try:
            file_path.write_text(json.dumps(recording, ensure_ascii=False, indent=2), encoding="utf-8")
        except OSError as exc:
            return {"error": f"save failed: {exc}"}

        return {
            "state": "saved",
            "name": name,
            "file": str(file_path),
            "frames": len(frames),
            "duration_ms": metadata["duration_ms"],
        }

    def _load(self, args: dict) -> dict:
        name = str(args.get("name", ""))
        if not name:
            return {"error": "name is required"}
        safe_name = name.replace(" ", "_").replace("/", "_")
        file_path = self._recordings_dir / f"{safe_name}.json"
        if not file_path.exists():
            return {"error": f"recording not found: {name}"}

        try:
            data = json.loads(file_path.read_text(encoding="utf-8"))
            frames = data.get("frames", [])
            meta = data.get("metadata", {})
        except (json.JSONDecodeError, OSError) as exc:
            return {"error": f"load failed: {exc}"}

        with self._lock:
            self._frames = list(frames)
            self._record_label = meta.get("label", name)

        return {
            "state": "loaded",
            "name": name,
            "label": meta.get("label", name),
            "frames": len(frames),
            "duration_ms": meta.get("duration_ms", 0),
        }

    def _list_recordings(self) -> dict:
        recordings = []
        for fpath in sorted(self._recordings_dir.glob("*.json")):
            try:
                data = json.loads(fpath.read_text(encoding="utf-8"))
                meta = data.get("metadata", {})
                recordings.append({
                    "name": fpath.stem,
                    "label": meta.get("label", fpath.stem),
                    "frames": meta.get("frames", len(data.get("frames", []))),
                    "duration_ms": meta.get("duration_ms", 0),
                    "file": str(fpath),
                })
            except (json.JSONDecodeError, OSError):
                recordings.append({
                    "name": fpath.stem,
                    "error": "corrupt file",
                    "file": str(fpath),
                })

        return {
            "state": "ready",
            "recordings": recordings,
            "count": len(recordings),
            "directory": str(self._recordings_dir),
        }

    def _delete(self, args: dict) -> dict:
        name = str(args.get("name", ""))
        if not name:
            return {"error": "name is required"}
        safe_name = name.replace(" ", "_").replace("/", "_")
        file_path = self._recordings_dir / f"{safe_name}.json"
        if not file_path.exists():
            return {"error": f"recording not found: {name}"}
        try:
            file_path.unlink()
        except OSError as exc:
            return {"error": f"delete failed: {exc}"}
        return {"state": "deleted", "name": name}

    # ── Status ────────────────────────────────────────────────────────────────

    def _status(self) -> dict:
        self._refresh_reset_state()
        with self._lock:
            elapsed_ms = (
                max(0, _now_ms() - self._record_start_ms)
                if self._recording else 0
            )
            return {
                "state": "recording" if self._recording else ("playing" if self._playing else "idle"),
                "recording": self._recording,
                "playing": self._playing,
                "buffer_frames": len(self._frames),
                "record_label": self._record_label if self._recording else "",
                "record_elapsed_ms": elapsed_ms,
                "record_hz": self._record_hz,
                "record_session_id": self._record_session if self._recording else None,
                "playback_label": self._playback_label if self._playing else "",
                "playback_frame": self._playback_frame if self._playing else 0,
                "playback_error": self._last_playback_error,
                "recordings_dir": str(self._recordings_dir),
                "joint_data_available": self._latest_joint_positions is not None,
                "last_recording": dict(self._last_recording) if self._last_recording else None,
                "needs_reset": self._needs_reset,
                "reset_pending": self._reset_request_id is not None,
                "last_reset": dict(self._last_reset) if self._last_reset else None,
            }

    # ── Plugin compatibility ──────────────────────────────────────────────────

    def set_joint_plan(self, joint_plan):
        """Inject the JointPlanPlugin reference for reset."""
        self._joint_plan = joint_plan

    def set_reset_controls(self, state, motion_mode) -> None:
        """Inject motion-state controls used by reset and the recording gate."""
        self._state = state
        self._motion_mode = motion_mode

    def _lower_body_balance_error(self, operation: str) -> dict | None:
        if self._state is None:
            return {
                "error": f"{operation} blocked: motion state is unavailable",
                "current_motion_state": None,
            }
        current, _ = self._state.current_motion()
        if current == "lower_body_balance":
            return None
        return {
            "error": (
                f"{operation} requires lower_body_balance "
                f"(current: {current or 'unknown'}); run reset first"
            ),
            "current_motion_state": current,
        }

    def _reset_pose(self) -> dict:
        with self._lock:
            if self._recording:
                return {"error": "stop recording before reset"}
            if self._reset_request_id is not None and self._last_reset is not None:
                return {**self._last_reset, "already_resetting": True}
        joint_plan = getattr(self, "_joint_plan", None)
        if joint_plan is None or self._state is None or self._motion_mode is None:
            return {"error": "reset controls are unavailable"}

        # Release any active upper-body override before changing the robot FSM.
        self._stop_playback()
        current, available = self._state.current_motion()
        if current != "lower_body_balance":
            if "lower_body_balance" not in available:
                return {
                    "error": (
                        "lower_body_balance is not available from "
                        f"{current or 'unknown'}"
                    ),
                    "available": available,
                }
            mode_result = self._motion_mode.dispatch("switch", {
                "target": "lower_body_balance",
                "force": False,
                "wait": True,
            })
            current, _ = self._state.current_motion()
            if current != "lower_body_balance":
                return {
                    "error": "failed to enter lower_body_balance before reset",
                    "mode_result": mode_result,
                }

        result = joint_plan.dispatch("reset", {})
        if "error" in result:
            return result
        request_id = result.get("request_id")
        if request_id is None:
            return {"error": "joint planner reset did not return request_id"}
        action_id = f"t800_motion_reset_{uuid4().hex[:12]}"
        with self._lock:
            self._needs_reset = True
            self._reset_request_id = int(request_id)
            self._reset_action_id = action_id
            self._last_reset = {
                "state": "resetting",
                "request_id": self._reset_request_id,
                "action_id": action_id,
                "motion_state": current,
            }
            response = dict(self._last_reset)
        threading.Thread(
            target=self._monitor_reset,
            args=(int(request_id), action_id),
            daemon=True,
            name="t800-motion-reset-monitor",
        ).start()
        return response

    def _refresh_reset_state(self) -> None:
        with self._lock:
            request_id = self._reset_request_id
        joint_plan = getattr(self, "_joint_plan", None)
        if request_id is None or joint_plan is None:
            return
        status = joint_plan.dispatch("status", {})
        if (
            int(status.get("request_id", -1)) == request_id
            and int(status.get("status", -1)) == 1
        ):
            completed_action_id = None
            with self._lock:
                if self._reset_request_id == request_id:
                    completed_action_id = self._reset_action_id
                    self._reset_request_id = None
                    self._reset_action_id = None
                    self._needs_reset = False
                    self._last_reset = {
                        "state": "completed",
                        "request_id": request_id,
                        "action_id": completed_action_id,
                        "motion_state": "lower_body_balance",
                    }
            if completed_action_id:
                self._acp_notify(completed_action_id, "completed", {
                    "request_id": request_id,
                    "motion_state": "lower_body_balance",
                }, self.PREFIX)

    def _monitor_reset(self, request_id: int, action_id: str) -> None:
        deadline = time.monotonic() + self._reset_timeout_sec
        while time.monotonic() < deadline:
            with self._lock:
                if (
                    self._reset_request_id != request_id
                    or self._reset_action_id != action_id
                ):
                    return
            self._refresh_reset_state()
            time.sleep(0.05)
        with self._lock:
            if (
                self._reset_request_id != request_id
                or self._reset_action_id != action_id
            ):
                return
            self._reset_request_id = None
            self._reset_action_id = None
            self._needs_reset = True
            self._last_reset = {
                "state": "error",
                "request_id": request_id,
                "action_id": action_id,
                "motion_state": "lower_body_balance",
                "error": "reset timeout",
            }
        self._acp_notify(action_id, "error", {
            "request_id": request_id,
            "error": "reset timeout",
        }, self.PREFIX)
