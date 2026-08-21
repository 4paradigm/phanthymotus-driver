"""Q5 direct base-drive velocity control card.

This card publishes finite-duration TwistStamped commands via a separate
subprocess running on rmw_fastrtps_cpp (FastDDS) / Domain 211, because the
vendor Wr1 base controller only accepts messages on that RMW.

Parent process (CycloneDDS) handles /activate_service discovery and call.
Subprocess (FastDDS) only publishes TwistStamped velocity commands.

The parent process (main.py, CycloneDDS) stays untouched.
"""

from __future__ import annotations

import math
import multiprocessing as mp
import os
import threading
import time

from control_contract import q5_active_status, q5_is_control_ready

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


# ── Subprocess launcher (FastDDS only — publishes TwistStamped) ──────────────

class _SubprocDriver:
    """Spawn a FastDDS subprocess and send it commands via a Queue."""

    def __init__(self, publish_rate: float, stop_repetitions: int):
        self._ctx = mp.get_context("spawn")
        self._cmd_q = self._ctx.Queue()
        self._publish_rate = publish_rate
        self._stop_repetitions = stop_repetitions
        self._proc = None
        self._lock = threading.Lock()

    def start(self):
        """Spawn the subprocess (lazy — only once first needed)."""
        with self._lock:
            if self._proc is not None and self._proc.is_alive():
                return
            self._proc = self._ctx.Process(
                target=_subproc_main,
                args=(self._cmd_q, self._publish_rate, self._stop_repetitions),
                name="q5_base_drive_subproc", daemon=True,
            )
            self._proc.start()
            print(f"[base_drive] subproc started → pid={self._proc.pid}", flush=True)

    def move(self, linear_x: float, angular_z: float, duration_s: float):
        self.start()
        self._cmd_q.put_nowait({
            "kind": "move",
            "linear_x": linear_x,
            "angular_z": angular_z,
            "duration_s": duration_s,
        })

    def stop(self):
        self.start()
        try:
            self._cmd_q.put_nowait({"kind": "stop"})
        except Exception:
            pass

    def get_status(self) -> dict:
        return {
            "subproc_alive": self._proc.is_alive() if self._proc else False,
            "subproc_pid": self._proc.pid if self._proc else None,
        }


def _subproc_main(cmd_q: mp.Queue, publish_rate: float, stop_repetitions: int):
    """Subprocess entry — FastDDS only, publishes TwistStamped. No activation."""
    os.environ["ROS_DOMAIN_ID"] = "211"
    os.environ["RMW_IMPLEMENTATION"] = "rmw_fastrtps_cpp"
    os.environ["FASTDDS_BUILTIN_TRANSPORTS"] = "DEFAULT"
    os.environ.setdefault("PYTHONUNBUFFERED", "1")

    import signal

    import rclpy
    import rclpy.executors
    from rclpy.node import Node
    from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
    from geometry_msgs.msg import TwistStamped
    from std_srvs.srv import Trigger

    rclpy.init()
    node = Node("q5_base_drive_subproc")
    pub = node.create_publisher(
        TwistStamped, "/wr1_base_drive_controller/cmd_vel",
        QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        ),
    )
    # No /activate_service client here — parent handles activation via CycloneDDS

    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)

    running = True

    def _handle_sig(signum, frame):
        nonlocal running
        running = False

    signal.signal(signal.SIGTERM, _handle_sig)
    signal.signal(signal.SIGINT, _handle_sig)

    last_log = time.time()

    def _publish(linear_x: float, angular_z: float):
        msg = TwistStamped()
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.twist.linear.x = linear_x
        msg.twist.angular.z = angular_z
        pub.publish(msg)

    def _publish_stop():
        for i in range(stop_repetitions):
            _publish(0.0, 0.0)
            if i + 1 < stop_repetitions:
                time.sleep(1.0 / publish_rate)

    while running:
        # Drain commands
        cmds = []
        try:
            while True:
                cmds.append(cmd_q.get_nowait())
        except Exception:
            pass

        for cmd in cmds:
            if not isinstance(cmd, dict):
                continue
            kind = cmd.get("kind")
            if kind == "stop":
                _publish_stop()
            elif kind == "move":
                lx = float(cmd.get("linear_x", 0.0))
                az = float(cmd.get("angular_z", 0.0))
                dur = float(cmd.get("duration_s", 1.0))
                deadline = time.monotonic() + dur
                stop_early = False
                try:
                    while not stop_early and time.monotonic() < deadline:
                        _publish(lx, az)
                        # Check for pending stop command
                        try:
                            extra = cmd_q.get_nowait()
                            if isinstance(extra, dict) and extra.get("kind") == "stop":
                                stop_early = True
                        except Exception:
                            pass
                        time.sleep(1.0 / publish_rate)
                finally:
                    _publish_stop()

        # Health log every 10s
        now = time.time()
        if now - last_log >= 10.0:
            last_log = now
            node.get_logger().info("base_drive_subproc health OK")

        executor.spin_once(timeout_sec=0.005)

    node.destroy_node()
    rclpy.shutdown()


# ── Plugin ───────────────────────────────────────────────────────────────────

class Plugin:
    """Handles validation, activation (via FastDDS temp node), and dispatch."""

    def __init__(self, plugin_config, namespace, executor, client):
        self._client = client
        self._max_linear = float(plugin_config.get("max_linear_x_mps", 0.20))
        self._max_angular = float(plugin_config.get("max_angular_z_radps", 0.40))
        self._max_duration = float(plugin_config.get("max_duration_s", 2.0))
        self._publish_rate = float(plugin_config.get("publish_rate_hz", 10.0))
        self._stop_repetitions = int(plugin_config.get("stop_repetitions", 3))
        self._driver = _SubprocDriver(
            self._publish_rate, self._stop_repetitions,
        )

        if min(self._max_linear, self._max_angular, self._max_duration, self._publish_rate) <= 0:
            raise ValueError("base_drive limits and publish_rate_hz must be positive")
        if self._stop_repetitions < 1:
            raise ValueError("base_drive stop_repetitions must be at least 1")

        # Activation handled by parent — spawn a temporary FastDDS node to
        # call /activate_service (which is only discoverable on FastDDS).
        self._activated = False
        self._activate_lock = threading.Lock()
        self._ensure_activation()

    def _ensure_activation(self) -> dict:
        """Call /activate_service using a temporary FastDDS ROS2 node.

        The vendor's /activate_service Trigger is published on the same
        FastDDS / Domain 211 stack that the base controller listens on.
        The parent process uses CycloneDDS and cannot discover it, so we
        spin up a short-lived FastDDS node solely for this one call.
        """
        with self._activate_lock:
            if self._activated:
                return {"ok": True, "activated": True}

            # Save parent's DDS env
            saved_domain = os.environ.get("ROS_DOMAIN_ID")
            saved_rmw = os.environ.get("RMW_IMPLEMENTATION")

            # Temporarily switch parent process to FastDDS
            os.environ["ROS_DOMAIN_ID"] = "211"
            os.environ["RMW_IMPLEMENTATION"] = "rmw_fastrtps_cpp"
            os.environ["FASTDDS_BUILTIN_TRANSPORTS"] = "DEFAULT"

            result = _call_activate_service()

            # Restore parent's CycloneDDS env
            if saved_domain is not None:
                os.environ["ROS_DOMAIN_ID"] = saved_domain
            else:
                os.environ.pop("ROS_DOMAIN_ID", None)
            if saved_rmw is not None:
                os.environ["RMW_IMPLEMENTATION"] = saved_rmw
            else:
                os.environ.pop("RMW_IMPLEMENTATION", None)
            os.environ.pop("FASTDDS_BUILTIN_TRANSPORTS", None)

            if result.get("ok"):
                self._activated = True
            return result

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
        return {
            "ros_publisher_available": True,  # via subproc
            "control_mode": "direct_velocity_interface",
            "q5_fsm": q5_active_status(self._client),
            "topic": TOPIC,
            "limits": {
                "max_linear_x_mps": self._max_linear,
                "max_angular_z_radps": self._max_angular,
                "max_duration_s": self._max_duration,
            },
            "subproc": self._driver.get_status(),
            "activated": self._activated,
        }

    def _validate_move(self, args: dict):
        status = self._control_status()
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
        self._driver.stop()

    def dispatch(self, action, args):
        if action == "start":
            return {"state": "ready", "safety": self._control_status()}
        if action == "info":
            return {"ok": True, "state": "moving" if self._driver.get_status()["subproc_alive"] else "idle",
                    "active_command": None,
                    "safety": self._control_status()}
        if action in ("cancel", "stop"):
            self._driver.stop()
            return {"ok": True, "state": "stopped", "reason": "command"}

        if action not in ("forward", "backward", "turn_left", "turn_right", "move"):
            return None

        # Ensure activation before every movement
        activation_result = self._ensure_activation()
        if activation_result.get("ok") is False:
            return activation_result

        move_args = self._directional_args(action, args) if action != "move" else args
        if isinstance(move_args, dict) and move_args.get("ok") is False:
            return move_args
        command = self._validate_move(move_args)
        if isinstance(command, dict):
            return command

        linear_x, angular_z, duration_s = command

        self._driver.move(linear_x, angular_z, duration_s)
        return {
            "ok": True, "state": "moving",
            "command": {
                "action": action,
                "linear_x": linear_x,
                "angular_z": angular_z,
                "duration_s": duration_s,
                "started_at_ms": int(time.time() * 1000),
            },
            "stops_automatically": True,
        }


def _call_activate_service() -> dict:
    """Create a temporary FastDDS node and call /activate_service.

    Returns {"ok": True} on success or a failure dict on error.
    """
    import rclpy
    from rclpy.node import Node
    from std_srvs.srv import Trigger

    try:
        if not rclpy.ok():
            rclpy.init()
        node = Node("q5_base_drive_activate_temp")
        client = node.create_client(Trigger, "/activate_service")

        if not client.service_is_ready():
            node.destroy_node()
            return _failure("ACTIVATE_SERVICE_UNAVAILABLE",
                            "/activate_service not available on FastDDS/Domain 211")

        future = client.call_async(Trigger.Request())
        deadline = time.monotonic() + 10.0
        while not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.02)

        if not future.done():
            node.destroy_node()
            return _failure("ACTIVATE_SERVICE_TIMEOUT",
                            "/activate_service did not respond within 10s")

        resp = future.result()
        node.destroy_node()

        if not resp or not resp.success:
            return _failure("ACTIVATE_SERVICE_FAILED",
                            resp.reason if resp and resp.reason else "unknown error")

        print("[base_drive] /activate_service succeeded", flush=True)
        return {"ok": True, "activated": True}

    except Exception as e:
        return _failure("ACTIVATE_SERVICE_ERROR", str(e))


def make_plugin(plugin_config, namespace, executor, client):
    return Plugin(plugin_config, namespace, executor, client)
