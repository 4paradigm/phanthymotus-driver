"""Q5 direct base-drive velocity control card.

This card only publishes finite-duration TwistStamped commands. It does not
perform the Q5 ready/zero/lift_up/activate sequence.
"""

from __future__ import annotations

import math
import threading
import time

from control_contract import q5_active_status, q5_is_control_ready

try:
    from geometry_msgs.msg import TwistStamped
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

    _HAS_ROS2 = True
    # The verified Q5 base controller subscribes with RELIABLE QoS. A
    # BEST_EFFORT publisher is incompatible with that endpoint and silently
    # drops every velocity command.
    _QOS = QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        durability=DurabilityPolicy.VOLATILE,
    )
except Exception:
    _HAS_ROS2 = False

CARD = "base_drive"
TYPE = "actuator"
TOPIC = "/wr1_base_drive_controller/cmd_vel"
NODE = "q5_base_drive"
DESC = "Q5 底盘速度控制：前进、后退、左转、右转与高级速度组合；每次动作自动停车"


def _failure(code: str, message: str, **details) -> dict:
    return {
        "ok": False,
        "code": code,
        "message": message,
        "details": details,
    }


def _number(value, field: str):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{field} must be a number")
    value = float(value)
    if not math.isfinite(value):
        raise ValueError(f"{field} must be finite")
    return value


class Plugin:
    def __init__(self, plugin_config, namespace, executor, client):
        self._client = client
        self._max_linear = float(plugin_config.get("max_linear_x_mps", 0.20))
        self._max_angular = float(plugin_config.get("max_angular_z_radps", 0.40))
        self._max_duration = float(plugin_config.get("max_duration_s", 2.0))
        self._publish_rate = float(plugin_config.get("publish_rate_hz", 10.0))
        self._stop_repetitions = int(plugin_config.get("stop_repetitions", 3))
        self._node = None
        self._pub = None
        self._lock = threading.Lock()
        self._motion_stop = None
        self._motion_thread = None
        self._active_command = None

        if min(self._max_linear, self._max_angular, self._max_duration, self._publish_rate) <= 0:
            raise ValueError("base_drive limits and publish_rate_hz must be positive")
        if self._stop_repetitions < 1:
            raise ValueError("base_drive stop_repetitions must be at least 1")

        if _HAS_ROS2 and executor is not None:
            try:
                self._node = Node(NODE)
                self._pub = self._node.create_publisher(TwistStamped, TOPIC, _QOS)
                executor.add_node(self._node)
            except Exception as e:
                print(f"[{CARD}] ROS2 publisher unavailable: {e}", flush=True)
                self._node = None
                self._pub = None

    def get_tool(self):
        return {
            "name": CARD,
            "type": TYPE,
            "multiInstance": False,
            "description": DESC,
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {
                        "type": "string",
                        "enum": ["start", "forward", "backward", "turn_left", "turn_right", "move", "cancel", "info"],
                        "oneOf": [
                            {"const": "start", "title": "检查控制条件"},
                            {"const": "forward", "title": "前进"},
                            {"const": "backward", "title": "后退"},
                            {"const": "turn_left", "title": "原地左转"},
                            {"const": "turn_right", "title": "原地右转"},
                            {"const": "move", "title": "高级：组合速度"},
                            {"const": "cancel", "title": "立即停止"},
                            {"const": "info", "title": "查看状态"},
                        ],
                        "description": "方向动作到时自动停止；停止会立即重复发送零速度。",
                    },
                    "speed_mps": {
                        "type": "number", "title": "移动速度 (m/s)", "minimum": 0.01,
                        "maximum": self._max_linear, "multipleOf": 0.01,
                        "default": min(0.10, self._max_linear),
                        "description": f"范围[0.01,{self._max_linear:g}]m/s",
                    },
                    "turn_speed_radps": {
                        "type": "number", "title": "转向速度 (rad/s)", "minimum": 0.01,
                        "maximum": self._max_angular, "multipleOf": 0.01,
                        "default": min(0.20, self._max_angular),
                        "description": f"范围[0.01,{self._max_angular:g}]rad/s",
                    },
                    "linear_x": {
                        "type": "number",
                        "title": "前后速度 (m/s)", "minimum": -self._max_linear,
                        "maximum": self._max_linear, "multipleOf": 0.01, "default": 0.10,
                        "description": f"范围[-{self._max_linear:g},{self._max_linear:g}]m/s",
                    },
                    "angular_z": {
                        "type": "number",
                        "title": "转向速度 (rad/s)", "minimum": -self._max_angular,
                        "maximum": self._max_angular, "multipleOf": 0.01, "default": 0.0,
                        "description": f"范围[-{self._max_angular:g},{self._max_angular:g}]rad/s",
                    },
                    "duration_s": {
                        "type": "number",
                        "title": "持续时间 (秒)", "minimum": 0.1, "maximum": self._max_duration,
                        "multipleOf": 0.1, "default": min(0.5, self._max_duration),
                        "description": f"范围[0.1,{self._max_duration:g}]秒",
                    },
                },
                "required": ["action"],
                "additionalProperties": False,
                "x-action-params": {
                    "start": {"params": [], "description": "检查控制锁、发布者冲突和当前限制。"},
                    "forward": {"params": ["speed_mps", "duration_s"], "description": "以设定速度直线前进，到时自动停车。"},
                    "backward": {"params": ["speed_mps", "duration_s"], "description": "以设定速度直线后退，到时自动停车。"},
                    "turn_left": {"params": ["turn_speed_radps", "duration_s"], "description": "以设定速度原地左转，到时自动停车。"},
                    "turn_right": {"params": ["turn_speed_radps", "duration_s"], "description": "以设定速度原地右转，到时自动停车。"},
                    "move": {"params": ["linear_x", "angular_z", "duration_s"], "description": "高级模式：同时设置前后与转向速度，到时自动停车。"},
                    "cancel": {"params": [], "description": "立即发送零速度。"},
                    "info": {"params": [], "description": "查看当前命令和安全条件。"},
                },
            },
        }

    def _control_status(self) -> dict:
        competing_publishers = []
        endpoint_query_available = self._node is not None
        if self._node is not None:
            try:
                competing_publishers = [
                    {"node_name": endpoint.node_name, "node_namespace": endpoint.node_namespace}
                    for endpoint in self._node.get_publishers_info_by_topic(TOPIC)
                    if endpoint.node_name != NODE
                ]
            except Exception:
                endpoint_query_available = False
        return {
            "ros_publisher_available": self._pub is not None,
            "endpoint_query_available": endpoint_query_available,
            # Q5 vendor confirmation: direct external velocity publishing is
            # supported. These nodes are reported for diagnosis, not treated
            # as a software ownership lock.
            "other_publishers": competing_publishers,
            "control_mode": "direct_velocity_interface",
            "q5_fsm": q5_active_status(self._client),
            "topic": TOPIC,
            "limits": {
                "max_linear_x_mps": self._max_linear,
                "max_angular_z_radps": self._max_angular,
                "max_duration_s": self._max_duration,
            },
        }

    def _publish(self, linear_x: float, angular_z: float) -> bool:
        if self._pub is None or self._node is None:
            return False
        msg = TwistStamped()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.twist.linear.x = linear_x
        msg.twist.angular.z = angular_z
        self._pub.publish(msg)
        return True

    def _publish_stop(self):
        published = False
        for index in range(self._stop_repetitions):
            published = self._publish(0.0, 0.0) or published
            if index + 1 < self._stop_repetitions:
                time.sleep(1.0 / self._publish_rate)
        return published

    def _run_motion(self, stop_event, linear_x: float, angular_z: float, duration_s: float):
        deadline = time.monotonic() + duration_s
        try:
            while not stop_event.is_set() and time.monotonic() < deadline:
                self._publish(linear_x, angular_z)
                stop_event.wait(1.0 / self._publish_rate)
        finally:
            self._publish_stop()
            with self._lock:
                if self._motion_stop is stop_event:
                    self._motion_stop = None
                    self._motion_thread = None
                    self._active_command = None

    def _stop_motion(self, reason: str) -> dict:
        with self._lock:
            stop_event = self._motion_stop
            motion_thread = self._motion_thread
            self._motion_stop = None
            self._motion_thread = None
            self._active_command = None
        if stop_event is not None:
            stop_event.set()
        published = self._publish_stop()
        if motion_thread is not None and motion_thread is not threading.current_thread():
            motion_thread.join(timeout=1.0)
        if not published:
            return _failure("ROS_UNAVAILABLE", "Cannot publish Q5 zero velocity", reason=reason)
        return {"ok": True, "state": "stopped", "reason": reason, "zero_velocity_repetitions": self._stop_repetitions}

    def _validate_move(self, args: dict):
        status = self._control_status()
        if not status["ros_publisher_available"]:
            return _failure("ROS_UNAVAILABLE", "Q5 TwistStamped publisher is unavailable", status=status)
        lifecycle_state = self._client.get_lifecycle_state()
        if lifecycle_state != "active":
            return _failure("LIFECYCLE_NOT_ACTIVE", "Q5 motion_manager must be active before base control",
                            status={**status, "lifecycle_state": lifecycle_state})
        q5_ready, q5_status = q5_is_control_ready(self._client)
        if not q5_ready:
            return _failure("Q5_FSM_NOT_READY", "Q5 /xbot_state must be fresh and READY or ACTIVE before base control",
                            status={**status, "q5_fsm": q5_status})
        if not self._client.snapshot().get("fresh", False):
            return _failure("JOINT_STATE_UNAVAILABLE", "Refusing motion without fresh /joint_states")
        try:
            linear_x = _number(args.get("linear_x"), "linear_x")
            angular_z = _number(args.get("angular_z"), "angular_z")
            duration_s = _number(args.get("duration_s"), "duration_s")
        except ValueError as e:
            return _failure("INVALID_ARGUMENT", str(e))
        if linear_x == 0.0 and angular_z == 0.0:
            return _failure("INVALID_ARGUMENT", "Use action=stop for zero velocity")
        if abs(linear_x) > self._max_linear or abs(angular_z) > self._max_angular:
            return _failure("LIMIT_EXCEEDED", "Requested velocity exceeds configured deployment guardrails", limits=status["limits"])
        if not 0.0 < duration_s <= self._max_duration:
            return _failure("INVALID_ARGUMENT", "duration_s is outside the configured safe interval", max_duration_s=self._max_duration)
        return linear_x, angular_z, duration_s

    def _directional_args(self, action: str, args: dict):
        try:
            duration_s = _number(args.get("duration_s"), "duration_s")
            if action in ("forward", "backward"):
                speed = _number(args.get("speed_mps"), "speed_mps")
                if not 0.0 < speed <= self._max_linear:
                    return _failure("LIMIT_EXCEEDED", "speed_mps is outside the configured base-drive limit",
                                    max_linear_x_mps=self._max_linear)
                return {"linear_x": speed if action == "forward" else -speed, "angular_z": 0.0,
                        "duration_s": duration_s}
            speed = _number(args.get("turn_speed_radps"), "turn_speed_radps")
            if not 0.0 < speed <= self._max_angular:
                return _failure("LIMIT_EXCEEDED", "turn_speed_radps is outside the configured base-drive limit",
                                max_angular_z_radps=self._max_angular)
            return {"linear_x": 0.0, "angular_z": speed if action == "turn_left" else -speed,
                    "duration_s": duration_s}
        except ValueError as e:
            return _failure("INVALID_ARGUMENT", str(e))

    def start(self):
        pass

    def stop(self):
        self._stop_motion("driver_shutdown")

    def dispatch(self, action, args):
        if action == "start":
            return {"state": "ready", "safety": self._control_status()}
        if action == "info":
            with self._lock:
                active = dict(self._active_command) if self._active_command else None
            return {"ok": True, "state": "moving" if active else "idle", "active_command": active,
                    "safety": self._control_status()}
        if action in ("cancel", "stop"):
            return self._stop_motion("command")
        if action not in ("forward", "backward", "turn_left", "turn_right", "move"):
            return None

        move_args = self._directional_args(action, args) if action != "move" else args
        if isinstance(move_args, dict) and move_args.get("ok") is False:
            return move_args
        command = self._validate_move(move_args)
        if isinstance(command, dict):
            return command
        linear_x, angular_z, duration_s = command
        with self._lock:
            if self._motion_thread is not None and self._motion_thread.is_alive():
                return _failure("MOTION_IN_PROGRESS", "A Q5 base command is already active; call stop before moving again")
            stop_event = threading.Event()
            self._motion_stop = stop_event
            self._active_command = {
                "action": action,
                "linear_x": linear_x,
                "angular_z": angular_z,
                "duration_s": duration_s,
                "started_at_ms": int(time.time() * 1000),
            }
            self._motion_thread = threading.Thread(
                target=self._run_motion,
                args=(stop_event, linear_x, angular_z, duration_s),
                daemon=True,
                name="q5_base_drive",
            )
            self._motion_thread.start()
        return {"ok": True, "state": "moving", "command": dict(self._active_command),
                "stops_automatically": True}


def make_plugin(plugin_config, namespace, executor, client):
    return Plugin(plugin_config, namespace, executor, client)
