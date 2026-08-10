#!/usr/bin/env python3
"""EngineAI T800 ROS2 and Native SDK plugins.

Robot-facing traffic uses ROS domain 69.  Normalized dashboard streams are
republished on ROS domain 42.  All control interfaces exposed by EngineAI's
community protocol are represented, including the high-rate joint paths.
"""

from __future__ import annotations

import json
import math
import threading
import time
from dataclasses import asdict
from pathlib import Path

from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from control import (
    LED_MODES,
    MOTION_STATES,
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
    sensor_tool,
    validate_joint_indices,
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


def _now_ms() -> int:
    return int(time.time() * 1000)


def _json_message(payload: dict) -> String:
    msg = String()
    msg.data = json.dumps(payload, separators=(",", ":"), ensure_ascii=False)
    return msg


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
    }

    def __init__(self, config: dict, namespace: str, ros2):
        self._config = config
        self._ns = namespace
        self._ros2 = ros2
        self._topics = config["topics"]
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
        if action_or_tool == "start":
            return {"state": "running"}
        if action_or_tool == "stop":
            return {"state": "idle"}
        if action_or_tool == "info":
            name = args.get("_tool_name", "driver_health")
            if name not in self._STREAMS:
                return {"state": "running"}
            relative, fmt, _ = self._STREAMS[name]
            return {
                "state": "running",
                "topic_out": [{"topic": f"/{self._ns}/{relative}", "format": fmt}],
            }
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
                "feedback": list(self._STREAMS) + ["joint_plan_state"],
                "limitations": [
                    "no odometry topic: displacement/turn/arc are time-integrated open-loop estimates",
                    "no public camera/lidar/dexterous-hand interface in the referenced T800 protocol",
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
        return {"error": f"unknown derived state: {name}"}

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
        return {
            "state": "running" if any(v["connected"] for v in sources.values()) else "waiting",
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


class LocomotionPlugin:
    def __init__(self, config: dict, namespace: str, ros2, state: StatePlugin):
        self._config = config
        self._state = state
        self._node = Node("t800_locomotion", context=ros2.ctx_robot)
        ros2.executor_robot.add_node(self._node)
        self._publisher = None
        limits = config["control"]
        self._limits = (
            float(limits["max_vx"]),
            float(limits["max_vy"]),
            float(limits["max_vyaw"]),
        )
        self._stream = RepeatingCommand(
            self._publish_payload,
            self._publish_zero,
            rate_hz=float(limits["velocity_rate_hz"]),
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
                    "force": {"type": "boolean", "description": "忽略当前必须为 walk 的状态门禁"},
                    "x_m": {"type": "number", "description": "机身坐标系前向位移，米"},
                    "y_m": {"type": "number", "description": "机身坐标系侧向位移，米"},
                    "speed_m_s": {"type": "number", "description": "平移速度绝对值，m/s"},
                    "angle_rad": {"type": "number", "description": "偏航角或圆弧夹角，rad"},
                    "angular_speed_rad_s": {"type": "number", "description": "角速度绝对值，rad/s"},
                    "radius_m": {"type": "number", "description": "圆弧半径绝对值，米"},
                    "linear_speed_m_s": {"type": "number", "description": "圆弧线速度，m/s；负数为后退"},
                },
                "运动动作",
            ),
        }

    def start(self) -> None:
        from interface_protocol.msg import BodyVelCmd

        self._message_type = BodyVelCmd
        self._publisher = self._node.create_publisher(
            BodyVelCmd, self._config["topics"]["body_velocity"], _RELIABLE
        )

    def stop(self) -> None:
        self._stream.stop()

    def dispatch(self, action: str, args: dict) -> dict:
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
        if motion != "walk" and not bool(args.get("force", False)):
            return {"error": f"move requires motion state 'walk' (current: {motion or 'unknown'})"}
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
        "walk": "walk",
        "dance": "dance",
        "get_up": "supine_to_stance",
        "lie_down": "stance_to_supine",
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
            target = str(args.get("target", "walk"))
        else:
            return {"error": f"unknown dance action: {action}"}
        forwarded = dict(args)
        forwarded["target"] = target
        return self._motion_mode.dispatch("switch", forwarded)


class JointPlanPlugin:
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
        self._state_lock = threading.Lock()
        self._last_state = {"state": "no_data"}
        self._request_id = 0
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
                return dict(self._last_state)
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
            indices = validate_joint_indices(args.get("joint_indices"))
            positions = float_list(args.get("target_positions"), "target_positions", size=len(indices))
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
        with self._state_lock:
            self._request_id = max(self._request_id, int(msg.request_id))
            self._last_state = payload
        self._core_pub.publish(_json_message(payload))


class GesturePlugin:
    """Multi-step gesture choreography using the official joint planner."""

    _INDICES = list(range(12, 25))
    _NEUTRAL = [0.0, 0.028, 0.084, -0.001, -0.066, 0.0, 0.024, -0.081, 0.001, -0.069, 0.0, 0.0, 0.0]
    _RAISED = [0.0, -1.29568, 1.17971, 0.0757227, -1.06603, -0.0989933,
               -0.0211716, -0.322156, 0.0440607, -0.0871668, 0.0196457, 0.0, 0.0]
    _WAVE = [0.0, -1.07786, 1.13928, 0.177577, -1.83356, -0.0875483,
             -0.0211716, -0.322156, 0.0440607, -0.0871668, 0.0196457, 0.0, 0.0]
    _HAND_EXTENDED = [0.0, 0.024, 0.081, -0.001, -0.069, 0.0,
                      -0.47, 0.255, 0.161, -0.731, 0.028, 0.0, 0.0]
    _HAND_WITHDRAWN = [0.0, 0.024, 0.081, -0.001, -0.069, 0.0,
                       0.028, -0.084, 0.001, -0.066, 0.0, 0.0, 0.0]
    _BASE_STIFFNESS = [200.0, 30.0, 30.0, 15.0, 30.0, 15.0,
                       40.0, 40.0, 20.0, 40.0, 20.0, 100.0, 100.0]
    _WAVE_STIFFNESS = [500.0, 30.0, 30.0, 15.0, 30.0, 15.0,
                       40.0, 40.0, 20.0, 40.0, 20.0, 100.0, 100.0]
    _SHAKE_STIFFNESS = [400.0, 40.0, 40.0, 20.0, 40.0, 20.0,
                        40.0, 40.0, 20.0, 40.0, 20.0, 100.0, 100.0]
    _BASE_DAMPING = [3.0, 1.0, 1.0, 1.0, 1.0, 1.0,
                     1.0, 1.0, 1.0, 1.0, 1.0, 3.0, 3.0]
    _GESTURES = {
        "wave_hands": [
            (_NEUTRAL, 1.0, _BASE_STIFFNESS), (_RAISED, 2.0, _BASE_STIFFNESS),
            (_WAVE, 0.3, _WAVE_STIFFNESS), (_RAISED, 0.3, _WAVE_STIFFNESS),
            (_WAVE, 0.3, _WAVE_STIFFNESS), (_RAISED, 0.3, _WAVE_STIFFNESS),
            (_WAVE, 0.3, _BASE_STIFFNESS),
        ],
        "shake_hand": [
            (_HAND_EXTENDED, 2.0, _SHAKE_STIFFNESS),
            (_HAND_WITHDRAWN, 2.0, _SHAKE_STIFFNESS),
        ],
    }

    def __init__(self, joint_plan: JointPlanPlugin):
        self._joint_plan = joint_plan
        self._lock = threading.Lock()
        self._cancel = threading.Event()
        self._thread: threading.Thread | None = None
        self._status = {"state": "idle", "gesture": None, "step": 0, "total_steps": 0}

    def get_tool(self) -> dict:
        return {
            "name": "gesture",
            "type": "actuator",
            "multiInstance": False,
            "description": "T800 完整多步手势编排；内置官方挥手和握手序列，也支持任意关节动作队列",
            "inputSchema": action_schema(
                {
                    "list": ([], "列出内置手势及步数"),
                    "play": (["name", "repetitions", "reset_after", "wait"], "播放内置完整手势"),
                    "sequence": (["steps", "reset_after", "wait"], "执行自定义多步关节规划序列"),
                    "stop_gesture": (["reset_after"], "取消当前步骤并停止手势"),
                    "status": ([], "查询手势、步骤和错误"),
                },
                {
                    "name": {"type": "string", "enum": list(self._GESTURES)},
                    "repetitions": {"type": "integer", "minimum": 1, "maximum": 20},
                    "reset_after": {"type": "boolean"},
                    "wait": {"type": "boolean", "description": "等待整套动作完成"},
                    "steps": {
                        "type": "array",
                        "description": "每步支持 joint_indices 或 joint_names、target_positions、duration、stiffness、damping",
                        "items": {"type": "object"},
                    },
                },
                "手势动作",
            ),
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
                    {"name": name, "steps": len(steps), "source": "EngineAI T800 official example"}
                    for name, steps in self._GESTURES.items()
                ],
                "custom_sequence": True,
            }
        if action == "status":
            with self._lock:
                return dict(self._status)
        if action in ("stop", "stop_gesture"):
            return self._stop(reset_after=bool(args.get("reset_after", False)))
        if action == "play":
            name = str(args.get("name", ""))
            if name not in self._GESTURES:
                return {"error": f"unknown gesture: {name}"}
            repetitions = int(args.get("repetitions", 1))
            if repetitions < 1 or repetitions > 20:
                return {"error": "repetitions must be between 1 and 20"}
            steps = []
            for _ in range(repetitions):
                steps.extend(self._official_steps(name))
            label = name
        elif action == "sequence":
            steps = args.get("steps")
            if not isinstance(steps, list) or not steps:
                return {"error": "steps must be a non-empty array"}
            steps = [dict(step) for step in steps]
            label = "custom"
        else:
            return {"error": f"unknown gesture action: {action}"}
        return self._start_sequence(
            label,
            steps,
            reset_after=bool(args.get("reset_after", True)),
            wait=bool(args.get("wait", False)),
        )

    def _official_steps(self, name: str) -> list[dict]:
        steps = []
        for positions, duration, stiffness in self._GESTURES[name]:
            steps.append({
                "joint_indices": list(self._INDICES),
                "target_positions": list(positions),
                "duration": duration,
                "stiffness": list(stiffness),
                "damping": list(self._BASE_DAMPING),
                "gravity_compensation": True,
            })
        return steps

    def _start_sequence(self, label: str, steps: list[dict], *, reset_after: bool, wait: bool) -> dict:
        with self._lock:
            if self._thread is not None and self._thread.is_alive():
                return {"error": "another gesture sequence is already running", **self._status}
            self._cancel = threading.Event()
            self._status = {"state": "running", "gesture": label, "step": 0,
                            "total_steps": len(steps), "error": None}

        def run() -> None:
            try:
                for index, step in enumerate(steps, start=1):
                    if self._cancel.is_set():
                        break
                    with self._lock:
                        self._status["step"] = index
                    action = "plan_named" if "joint_names" in step else "plan"
                    result = self._joint_plan.dispatch(action, step)
                    if "error" in result:
                        raise ValueError(result["error"])
                    with self._lock:
                        self._status["request_id"] = result.get("request_id")
                    duration = clamp(step.get("duration", 2.0), 0.05, 120.0)
                    if self._cancel.wait(duration):
                        break
                if not self._cancel.is_set() and reset_after:
                    result = self._joint_plan.dispatch("reset", {})
                    with self._lock:
                        self._status["request_id"] = result.get("request_id")
                with self._lock:
                    self._status["state"] = "cancelled" if self._cancel.is_set() else "completed"
            except Exception as exc:
                with self._lock:
                    self._status["state"] = "error"
                    self._status["error"] = str(exc)

        thread = threading.Thread(target=run, daemon=True, name="t800-gesture-sequence")
        with self._lock:
            self._thread = thread
        thread.start()
        if wait:
            thread.join()
            with self._lock:
                return dict(self._status)
        return {"state": "running", "gesture": label, "total_steps": len(steps)}

    def _stop(self, *, reset_after: bool) -> dict:
        self._cancel.set()
        with self._lock:
            request_id = self._status.get("request_id")
        if request_id is not None:
            self._joint_plan.dispatch("cancel", {"request_id": request_id})
        if reset_after:
            self._joint_plan.dispatch("reset", {})
        with self._lock:
            self._status["state"] = "cancelled"
            return dict(self._status)


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
            return {"state": "ready", "service": self._config["services"]["enable_motor"],
                    "available": bool(self._client and self._client.service_is_ready())}
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
