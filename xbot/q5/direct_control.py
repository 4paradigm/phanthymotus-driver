"""Q5 direct-control cards consolidated from the reference implementation.

This module contains the base, arm, hand, hand-gesture, and head cards.
Shared command routers, control-state checks, and URDF limits remain in their
small dedicated support modules so all cards use a single publisher per bus.
"""

from __future__ import annotations

import math
import threading
import time

from geometry_msgs.msg import TwistStamped
from rclpy.action import ActionClient
from rclpy.node import Node
from std_srvs.srv import Trigger
from xbot_common_interfaces.action import SimpleActions
from xbot_common_interfaces.srv import DynamicLaunch
from body_command import get_router as _get_body_router
from hand_command import HAND_JOINTS, failure as _hand_failure, finite_number as _hand_finite_number, get_router as _get_hand_router
from control_contract import q5_active_status, q5_is_control_ready
from joint_limits import JOINT_LIMITS, limits_for

try:
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    _RELIABLE_QOS = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                               history=HistoryPolicy.KEEP_LAST, depth=1,
                               durability=DurabilityPolicy.VOLATILE)
except Exception:
    _RELIABLE_QOS = 1



# ── Consolidated from base_drive.py ─────────────────────────────

"""Q5 direct base-drive velocity control card.

This card only publishes finite-duration TwistStamped commands. It does not
perform the Q5 ready/zero/lift_up/activate sequence.
"""




BASE_CARD = "base_drive"
BASE_TYPE = "actuator"
BASE_TOPIC = "/wr1_base_drive_controller/cmd_vel"
BASE_NODE = "q5_base_drive"
BASE_DESC = "Q5 底盘速度控制：前进、后退、左转、右转与高级速度组合；每次动作自动停车"


def _base_failure(code: str, message: str, **details) -> dict:
    return {
        "ok": False,
        "code": code,
        "message": message,
        "details": details,
    }


def _base_number(value, field: str):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{field} must be a number")
    value = float(value)
    if not math.isfinite(value):
        raise ValueError(f"{field} must be finite")
    return value


class BaseDrivePlugin:
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

        if True and executor is not None:
            try:
                self._node = Node(BASE_NODE)
                self._pub = self._node.create_publisher(TwistStamped, BASE_TOPIC, _RELIABLE_QOS)
                executor.add_node(self._node)
            except Exception as e:
                print(f"[{BASE_CARD}] ROS2 publisher unavailable: {e}", flush=True)
                self._node = None
                self._pub = None

    def get_tool(self):
        return {
            "name": BASE_CARD,
            "type": BASE_TYPE,
            "multiInstance": False,
            "description": BASE_DESC,
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
                    for endpoint in self._node.get_publishers_info_by_topic(BASE_TOPIC)
                    if endpoint.node_name != BASE_NODE
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
            "topic": BASE_TOPIC,
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
            return _base_failure("ROS_UNAVAILABLE", "Cannot publish Q5 zero velocity", reason=reason)
        return {"ok": True, "state": "stopped", "reason": reason, "zero_velocity_repetitions": self._stop_repetitions}

    def _validate_move(self, args: dict):
        status = self._control_status()
        if not status["ros_publisher_available"]:
            return _base_failure("ROS_UNAVAILABLE", "Q5 TwistStamped publisher is unavailable", status=status)
        lifecycle_state = self._client.get_lifecycle_state()
        if lifecycle_state != "active":
            return _base_failure("LIFECYCLE_NOT_ACTIVE", "Q5 motion_manager must be active before base control",
                                 status={**status, "lifecycle_state": lifecycle_state})
        q5_ready, q5_status = q5_is_control_ready(self._client)
        if not q5_ready:
            return _base_failure("Q5_FSM_NOT_READY", "Q5 /xbot_state must be fresh and READY or ACTIVE before base control",
                            status={**status, "q5_fsm": q5_status})
        if not self._client.snapshot().get("fresh", False):
            return _base_failure("JOINT_STATE_UNAVAILABLE", "Refusing motion without fresh /joint_states")
        try:
            linear_x = _base_number(args.get("linear_x"), "linear_x")
            angular_z = _base_number(args.get("angular_z"), "angular_z")
            duration_s = _base_number(args.get("duration_s"), "duration_s")
        except ValueError as e:
            return _base_failure("INVALID_ARGUMENT", str(e))
        if linear_x == 0.0 and angular_z == 0.0:
            return _base_failure("INVALID_ARGUMENT", "Use action=stop for zero velocity")
        if abs(linear_x) > self._max_linear or abs(angular_z) > self._max_angular:
            return _base_failure("LIMIT_EXCEEDED", "Requested velocity exceeds configured deployment guardrails", limits=status["limits"])
        if not 0.0 < duration_s <= self._max_duration:
            return _base_failure("INVALID_ARGUMENT", "duration_s is outside the configured safe interval", max_duration_s=self._max_duration)
        return linear_x, angular_z, duration_s

    def _directional_args(self, action: str, args: dict):
        try:
            duration_s = _base_number(args.get("duration_s"), "duration_s")
            if action in ("forward", "backward"):
                speed = _base_number(args.get("speed_mps"), "speed_mps")
                if not 0.0 < speed <= self._max_linear:
                    return _base_failure("LIMIT_EXCEEDED", "speed_mps is outside the configured base-drive limit",
                                    max_linear_x_mps=self._max_linear)
                return {"linear_x": speed if action == "forward" else -speed, "angular_z": 0.0,
                        "duration_s": duration_s}
            speed = _base_number(args.get("turn_speed_radps"), "turn_speed_radps")
            if not 0.0 < speed <= self._max_angular:
                return _base_failure("LIMIT_EXCEEDED", "turn_speed_radps is outside the configured base-drive limit",
                                max_angular_z_radps=self._max_angular)
            return {"linear_x": 0.0, "angular_z": speed if action == "turn_left" else -speed,
                    "duration_s": duration_s}
        except ValueError as e:
            return _base_failure("INVALID_ARGUMENT", str(e))

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
                return _base_failure("MOTION_IN_PROGRESS", "A Q5 base command is already active; call stop before moving again")
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




# ── Consolidated from arm_control.py ─────────────────────────────

"""Direct absolute Q5 arm joint-position control card.

The card accepts one absolute target for one allowlisted body joint. Targets
are validated against the bundled URDF limits before interpolation.
"""




ARM_CARD = "arm_control"
ARM_TYPE = "actuator"
ARM_TOPIC = "/wr1_controller/commands"
ARM_JOINTS = (
    "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_arm_yaw_joint",
    "left_elbow_pitch_joint", "left_elbow_yaw_joint", "left_wrist_pitch_joint",
    "left_wrist_roll_joint", "right_shoulder_pitch_joint", "right_shoulder_roll_joint",
    "right_arm_yaw_joint", "right_elbow_pitch_joint", "right_elbow_yaw_joint",
    "right_wrist_pitch_joint", "right_wrist_roll_joint",
)
ARM_JOINT_LABELS = {
    "left_shoulder_pitch_joint": "左肩俯仰", "left_shoulder_roll_joint": "左肩横滚",
    "left_arm_yaw_joint": "左上臂偏航", "left_elbow_pitch_joint": "左肘俯仰",
    "left_elbow_yaw_joint": "左肘偏航", "left_wrist_pitch_joint": "左腕俯仰",
    "left_wrist_roll_joint": "左腕旋转", "right_shoulder_pitch_joint": "右肩俯仰",
    "right_shoulder_roll_joint": "右肩横滚", "right_arm_yaw_joint": "右上臂偏航",
    "right_elbow_pitch_joint": "右肘俯仰", "right_elbow_yaw_joint": "右肘偏航",
    "right_wrist_pitch_joint": "右腕俯仰", "right_wrist_roll_joint": "右腕旋转",
}


def _arm_limit_summary() -> str:
    """Human-readable limits for clients that do not render JSON Schema allOf."""
    return "; ".join(
        f"{ARM_JOINT_LABELS[name]} {JOINT_LIMITS[name][0]:g}~{JOINT_LIMITS[name][1]:g} rad"
        for name in ARM_JOINTS
    )


ARM_DESC = (
    "关节绝对角度范围：" + _arm_limit_summary() + "。"
    "Q5 手臂单关节位置控制；target_position_rad 是绝对角度（不是增量），"
    "先执行 prepare_position_control。每步最多 0.010 rad、20 Hz，最大约 0.20 rad/s。"
)


def _arm_failure(code: str, message: str, **details) -> dict:
    return {"ok": False, "code": code, "message": message, "details": details}


def _arm_number(value, field: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{field} must be a number")
    value = float(value)
    if not math.isfinite(value):
        raise ValueError(f"{field} must be finite")
    return value


class ArmControlPlugin:
    def __init__(self, plugin_config, namespace, executor, client):
        self._client = client
        self._router = _get_body_router(client, executor)
        self._max_step = float(plugin_config.get("max_step_rad", 0.010))
        self._publish_rate = float(plugin_config.get("publish_rate_hz", 20.0))
        self._hold_repetitions = int(plugin_config.get("hold_repetitions", 3))
        if min(self._max_step, self._publish_rate) <= 0:
            raise ValueError("arm_control limits and publish rate must be positive")
        if self._hold_repetitions < 1:
            raise ValueError("arm_control hold_repetitions must be at least 1")

        self._lock = threading.Lock()
        self._motion_stop = None
        self._motion_thread = None
        self._active_command = None
        self._mode_node = Node("q5_arm_control_mode")
        executor.add_node(self._mode_node)
        self._dynamic = self._mode_node.create_client(DynamicLaunch, "/dynamic_launch")
        self._ready = self._mode_node.create_client(Trigger, "/ready_service")
        self._activate = self._mode_node.create_client(Trigger, "/activate_service")
        self._actions = ActionClient(self._mode_node, SimpleActions, "/simple_actions")
        self._prepared = False
        client.direct_control_prepared = False

    def get_tool(self):
        limit_rules = [
            {"if": {"properties": {"joint_name": {"const": name}}, "required": ["joint_name"]},
             "then": {"properties": {"target_position_rad": {"minimum": lower, "maximum": upper}}}}
            for name, (lower, upper) in ((name, JOINT_LIMITS[name]) for name in ARM_JOINTS)
        ]
        return {
            "name": ARM_CARD,
            "type": ARM_TYPE,
            "multiInstance": False,
            "description": ARM_DESC,
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {"type": "string", "enum": ["start", "prepare_position_control", "move", "cancel", "info"], "oneOf": [
                        {"const": "start", "title": "检查连接状态"},
                        {"const": "prepare_position_control", "title": "准备位置直控"},
                        {"const": "move", "title": "设置单关节角度"},
                        {"const": "cancel", "title": "取消并保持当前角度"},
                        {"const": "info", "title": "查看状态"},
                    ]},
                    "joint_name": {"type": "string", "title": "目标关节", "enum": list(ARM_JOINTS), "oneOf": [
                        {"const": name, "title": ARM_JOINT_LABELS[name]} for name in ARM_JOINTS
                    ]},
                    "target_position_rad": {"type": "number",
                                             "title": "范围 (rad)：" + _arm_limit_summary() + "；目标绝对角度",
                                             "multipleOf": 0.005,
                                             "description": "范围 (rad)：" + _arm_limit_summary() + "；绝对目标角度，不是相对位移；超限会被拒绝。"},
                },
                "required": ["action"],
                "additionalProperties": False,
                "allOf": limit_rules,
                "x-action-params": {
                    "start": {"params": [], "description": "检查 ROS 连接和机器人状态。"},
                    "prepare_position_control": {"params": [], "description": "执行厂商 DynamicLaunch(pos)、READY、初始姿态、抬臂和ACTIVE流程。"},
                    "move": {"params": ["joint_name", "target_position_rad"], "description": "设置单个关节的绝对角度；最大速度约 0.20 rad/s。范围：" + _arm_limit_summary()},
                    "cancel": {"params": [], "description": "取消微调，并保持当前关节角度。"},
                    "info": {"params": [], "description": "查看当前运动和安全条件。"},
                },
            },
        }

    def _safety(self) -> dict:
        status = self._router.status()
        status.update({
            "control_mode": "direct_joint_position",
            "command_message": "xbot_common_interfaces/msg/HybridJointCommand",
            "lifecycle_state": self._client.get_lifecycle_state(),
            "joint_state_fresh": bool(self._client.snapshot().get("fresh", False)),
            "q5_fsm": q5_active_status(self._client),
            "limits": {"max_step_rad": self._max_step,
                       "joint_position_limits": limits_for(ARM_JOINTS),
                       "joint_names_source": "q5_model.urdf"},
        })
        return status

    def _publish(self, joint_name: str, position: float) -> bool:
        return self._router.publish({joint_name: position})

    def _hold_position(self, joint_name: str, position: float | None) -> bool:
        if position is None:
            return False
        published = False
        for index in range(self._hold_repetitions):
            published = self._publish(joint_name, float(position)) or published
            if index + 1 < self._hold_repetitions:
                time.sleep(1.0 / self._publish_rate)
        return published

    def _hold_current(self, joint_name: str) -> bool:
        snap = self._client.snapshot()
        position = snap.get("joints", {}).get(joint_name)
        return self._hold_position(joint_name, position) if snap.get("fresh") else False

    def _run_move(self, stop_event, joint_name: str, current: float, target: float, duration_s: float):
        steps = max(
            int(math.ceil(abs(target - current) / self._max_step)),
            int(math.ceil(duration_s * self._publish_rate)),
            1,
        )
        try:
            for index in range(1, steps + 1):
                if stop_event.is_set():
                    break
                position = current + (target - current) * (index / steps)
                self._publish(joint_name, position)
                stop_event.wait(duration_s / steps)
        finally:
            # The joint-state stream may still contain the pre-command angle
            # when the final interpolation point is sent. Hold the target on
            # successful completion; cancellation continues to hold feedback.
            self._hold_position(joint_name, target) if not stop_event.is_set() else self._hold_current(joint_name)
            self._router.release(ARM_CARD)
            with self._lock:
                if self._motion_stop is stop_event:
                    self._motion_stop = None
                    self._motion_thread = None
                    self._active_command = None

    def _stop(self, reason: str) -> dict:
        with self._lock:
            stop_event = self._motion_stop
            motion_thread = self._motion_thread
            active = dict(self._active_command) if self._active_command else None
            self._motion_stop = None
            self._motion_thread = None
            self._active_command = None
        if stop_event is not None:
            stop_event.set()
        held = bool(active and self._hold_current(active["joint_name"]))
        if motion_thread is not None and motion_thread is not threading.current_thread():
            motion_thread.join(timeout=1.0)
        return {"ok": True, "state": "stopped", "reason": reason,
                "hold_command_published": held}

    def _validate_move(self, args):
        joint_name = args.get("joint_name")
        if joint_name not in ARM_JOINTS:
            return _arm_failure("JOINT_NOT_ALLOWED", "Joint is not in the Q5 arm allowlist")
        status = self._safety()
        if not status["ros_publisher_available"]:
            return _arm_failure("ROS_UNAVAILABLE", "Q5 arm command publisher is unavailable", status=status)
        if status["same_name_publisher_count"] > 1:
            return _arm_failure(
                "DUPLICATE_BODY_PUBLISHER",
                "Refusing arm motion: multiple q5_body_command publishers are active on /wr1_controller/commands",
                status=status,
            )
        # Head control uses this same body router and works alongside the
        # vendor MPC endpoint. ROS graph discovery only proves an endpoint
        # exists, not that it is actively emitting commands, so report it in
        # `info` but do not reject a bounded single-joint interpolation here.
        if not bool(getattr(self._client, "direct_control_prepared", False)):
            return _arm_failure(
                "DIRECT_CONTROL_NOT_PREPARED",
                "Run arm_control action=prepare_position_control first; vendor position-control sequence has not completed",
                status=status,
            )
        # Direct HybridJointCommand control is owned by the vendor body
        # controller after arm_control preparation completes. motion_manager is a
        # separate lifecycle node and may legitimately remain inactive.
        q5_ready, q5_status = q5_is_control_ready(self._client)
        if not q5_ready:
            return _arm_failure("Q5_FSM_NOT_READY", "Q5 /xbot_state must be fresh and READY or ACTIVE before arm control",
                            status={**status, "q5_fsm": q5_status})
        snap = self._client.snapshot()
        if not snap.get("fresh"):
            return _arm_failure("JOINT_STATE_UNAVAILABLE", "Refusing arm control without fresh /joint_states")
        current = snap.get("joints", {}).get(joint_name)
        if current is None:
            return _arm_failure("JOINT_UNAVAILABLE", "Requested arm joint is absent from /joint_states", joint_name=joint_name)
        try:
            target = _arm_number(args.get("target_position_rad"), "target_position_rad")
        except ValueError as e:
            return _arm_failure("INVALID_ARGUMENT", str(e))
        lower, upper = JOINT_LIMITS.get(joint_name, (None, None))
        if lower is None or target < lower or target > upper:
            return _arm_failure("LIMIT_EXCEEDED", "target_position_rad is outside the joint safety limits",
                            joint_name=joint_name, min_rad=lower, max_rad=upper,
                            target_position_rad=target)
        # A legal full-range move must not turn into a fast jump. The existing
        # max step and publication rate bound interpolation speed instead.
        duration_s = max(0.5, abs(target - float(current)) / (self._max_step * self._publish_rate))
        return joint_name, float(current), target, duration_s

    def start(self):
        return {"state": "ready" if self._router.status()["ros_publisher_available"] else "unavailable"}

    @staticmethod
    def _wait_future(future, timeout):
        deadline = time.monotonic() + timeout
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.02)
        return future.result() if future.done() else None

    def _prepare_position_control(self):
        steps = []
        if not self._dynamic.wait_for_service(timeout_sec=10.0):
            return _arm_failure("DYNAMIC_LAUNCH_UNAVAILABLE", "Q5 /dynamic_launch is unavailable", steps=steps)
        req = DynamicLaunch.Request()
        req.app_name, req.sync_control, req.launch_mode = "", False, "pos"
        response = self._wait_future(self._dynamic.call_async(req), 15.0)
        steps.append({"step": "dynamic_launch_pos", "success": bool(response and response.success), "message": getattr(response, "message", "timeout") if response else "timeout"})
        if not response or not response.success:
            return _arm_failure("DYNAMIC_LAUNCH_FAILED", "Q5 position launch failed", steps=steps)
        if not self._ready.wait_for_service(timeout_sec=10.0):
            return _arm_failure("READY_SERVICE_UNAVAILABLE", "Q5 /ready_service is unavailable", steps=steps)
        response = self._wait_future(self._ready.call_async(Trigger.Request()), 30.0)
        steps.append({"step": "ready_service", "success": bool(response and response.success), "message": getattr(response, "message", "timeout") if response else "timeout"})
        if not response or not response.success:
            return _arm_failure("READY_FAILED", "Q5 ready initialization failed", steps=steps)
        if not self._actions.wait_for_server(timeout_sec=5.0):
            return _arm_failure("SIMPLE_ACTIONS_UNAVAILABLE", "Q5 /simple_actions is unavailable", steps=steps)
        for name in ("initpose_handsdown", "lift_up"):
            goal = SimpleActions.Goal()
            goal.action_name = name
            goal.time_cost = 4.0
            handle = self._wait_future(self._actions.send_goal_async(goal), 8.0)
            result = self._wait_future(handle.get_result_async(), 35.0) if handle and handle.accepted else None
            success = bool(result and getattr(result.result, "result", 2) == 0)
            steps.append({"step": name, "success": success, "message": getattr(result.result, "message", "timeout") if result else "timeout"})
            if not success:
                return _arm_failure("SIMPLE_ACTION_FAILED", f"Q5 action {name} failed", steps=steps)
        if not self._activate.wait_for_service(timeout_sec=10.0):
            return _arm_failure("ACTIVATE_SERVICE_UNAVAILABLE", "Q5 /activate_service is unavailable", steps=steps)
        response = self._wait_future(self._activate.call_async(Trigger.Request()), 15.0)
        success = bool(response and response.success)
        steps.append({"step": "activate_service", "success": success, "message": getattr(response, "message", "timeout") if response else "timeout"})
        self._prepared = success
        self._client.direct_control_prepared = success
        return {"ok": success, "state": "active" if success else "failed", "steps": steps}

    def stop(self):
        self._stop("driver_shutdown")

    def dispatch(self, action, args):
        if action == "start":
            return {**self.start(), "safety": self._safety()}
        if action == "prepare_position_control":
            return self._prepare_position_control()
        if action in ("cancel", "stop"):
            return self._stop("command")
        if action == "info":
            with self._lock:
                active = dict(self._active_command) if self._active_command else None
            return {"ok": True, "state": "moving" if active else "idle", "active_command": active,
                    "safety": self._safety()}
        if action != "move":
            return None

        command = self._validate_move(args)
        if isinstance(command, dict):
            return command
        joint_name, current, target, duration_s = command
        if not self._router.acquire(ARM_CARD):
            return _arm_failure("COMMAND_IN_PROGRESS", "Another Q5 body card currently owns the command publisher",
                            status=self._router.status())
        with self._lock:
            if self._motion_thread is not None and self._motion_thread.is_alive():
                self._router.release(ARM_CARD)
                return _arm_failure("MOTION_IN_PROGRESS", "An arm movement is already active; call stop before another move")
            stop_event = threading.Event()
            self._motion_stop = stop_event
            self._active_command = {
                "joint_name": joint_name, "start_position_rad": current,
                "target_position_rad": target, "duration_s": duration_s,
                "started_at_ms": int(time.time() * 1000),
            }
            self._motion_thread = threading.Thread(
                target=self._run_move,
                args=(stop_event, joint_name, current, target, duration_s),
                daemon=True,
                name="q5_arm_control",
            )
            self._motion_thread.start()
        return {"ok": True, "state": "moving", "command": dict(self._active_command),
                "stops_by_holding_current_position": True}




# ── Consolidated from hand_control.py ─────────────────────────────

"""Direct, joint-level XHand Lite control card.

The card accepts one or more named finger targets, interpolated from the live
joint state. It intentionally does not expose gain, torque, or velocity arrays.
"""




HAND_CONTROL_CARD = "hand_control"
HAND_CONTROL_TYPE = "actuator"
HAND_CONTROL_DESC = "Q5 XHand Lite 完整关节控制：可同时设置任意左右手手指目标"

HAND_SIDES = ("left", "right", "both")
FINGERS = ("thumb", "index", "middle", "ring", "pinky")
FINGER_LABELS = {
    "thumb": "拇指",
    "index": "食指", "middle": "中指", "ring": "无名指", "pinky": "小指",
}
FINGER_JOINT_SUFFIXES = {
    "thumb": ("hand_thumb_bend_joint", "hand_thumb_rota_joint1"),
    "index": ("hand_index_joint1",),
    "middle": ("hand_mid_joint1",),
    "ring": ("hand_ring_joint1",),
    "pinky": ("hand_pinky_joint1",),
}


class HandControlPlugin:
    def __init__(self, plugin_config, namespace, executor, client):
        self._client = client
        self._router = _get_hand_router(client, executor)
        self._min_position = float(plugin_config.get("min_position_rad", 0.0))
        self._max_position = float(plugin_config.get("max_position_rad", 1.0))
        self._max_step = float(plugin_config.get("max_step_rad", 0.04))
        self._max_duration = float(plugin_config.get("max_duration_s", 2.0))
        self._publish_rate = float(plugin_config.get("publish_rate_hz", 20.0))
        self._hold_repetitions = int(plugin_config.get("hold_repetitions", 3))
        if not (self._min_position < self._max_position and min(self._max_step, self._max_duration, self._publish_rate) > 0 and self._hold_repetitions >= 1):
            raise ValueError("hand_control limits must be positive and position bounds ordered")
        self._lock = threading.Lock()
        self._motion_stop = None
        self._motion_thread = None
        self._active_command = None

    def get_tool(self):
        target = {"type": "object", "properties": {
            "joint_name": {"type": "string", "enum": list(HAND_JOINTS), "description": "XHand Lite 执行关节名称"},
            "position_rad": {"type": "number", "minimum": self._min_position, "maximum": self._max_position,
                             "description": f"范围[{self._min_position:g},{self._max_position:g}]rad"},
        }, "required": ["joint_name", "position_rad"], "additionalProperties": False}
        return {"name": HAND_CONTROL_CARD, "type": HAND_CONTROL_TYPE, "multiInstance": False, "description": HAND_CONTROL_DESC,
                "inputSchema": {"type": "object", "properties": {
                    "action": {"type": "string", "enum": ["start", "open_hand", "close_hand", "set_hand", "set_finger", "set", "cancel", "info"], "oneOf": [
                        {"const": "start", "title": "检查连接状态"},
                        {"const": "open_hand", "title": "张开整手"},
                        {"const": "close_hand", "title": "合拢整手"},
                        {"const": "set_hand", "title": "设置整手弯曲程度"},
                        {"const": "set_finger", "title": "设置单指弯曲程度"},
                        {"const": "set", "title": "高级：指定关节目标"},
                        {"const": "cancel", "title": "取消并保持"},
                        {"const": "info", "title": "查看状态"},
                    ]},
                    "side": {"type": "string", "title": "执行侧", "enum": list(HAND_SIDES), "oneOf": [
                        {"const": "left", "title": "左手"}, {"const": "right", "title": "右手"},
                        {"const": "both", "title": "双手"},
                    ], "default": "both", "description": "先选左/右/双手。"},
                    "finger": {"type": "string", "title": "手指", "enum": list(FINGERS), "oneOf": [
                        {"const": name, "title": FINGER_LABELS[name]} for name in FINGERS
                    ], "description": "选thumb可同时设弯曲和旋转。"},
                    "curl_rad": {"type": "number", "title": "弯曲角度(rad)", "minimum": self._min_position,
                                 "maximum": self._max_position, "multipleOf": 0.01,
                                 "default": min(0.20, self._max_position),
                                 "description": f"范围[{self._min_position:g},{self._max_position:g}]rad"},
                    "rotation_rad": {"type": "number", "title": "拇指旋转(rad)",
                                     "minimum": self._min_position, "maximum": self._max_position,
                                     "multipleOf": 0.01,
                                     "description": f"范围[{self._min_position:g},{self._max_position:g}]rad"},
                    "targets": {"type": "array", "title": "关节目标", "items": target,
                                "minItems": 1, "maxItems": len(HAND_JOINTS), "x-widget": "json",
                                "x-example": '[{"joint_name":"left_hand_index_joint1","position_rad":0.10}]',
                                "description": "每项：关节名+目标角度。"},
                }, "required": ["action"], "additionalProperties": False,
                "x-action-params": {
                    "start": {"params": [], "description": "检查 ROS 连接和机器人状态。"},
                    "open_hand": {"params": ["side"], "description": "将选中手完整张开到 0 rad。"},
                    "close_hand": {"params": ["side"], "description": f"将选中手完整合拢到 {self._max_position:g} rad。"},
                    "set_hand": {"params": ["side", "curl_rad"], "description": "为整只手设定统一的弯曲程度。"},
                    "set_finger": {"params": ["side", "finger", "curl_rad", "rotation_rad"], "description": "thumb可填弯曲+旋转"},
                    "set": {"params": ["targets"], "description": "高级模式：用 JSON 指定多个关节的绝对目标。"},
                    "cancel": {"params": [], "description": "取消当前插补，并保持当前位置。"},
                    "info": {"params": [], "description": "查看运动状态与安全条件。"},
                }}}

    def _safety(self):
        status = self._router.status()
        status.update({"control_mode": "direct_joint_position",
                       "command_message": "xbot_common_interfaces/msg/HybridJointCommand",
                       "lifecycle_state": self._client.get_lifecycle_state(),
                       "joint_state_fresh": bool(self._client.snapshot().get("fresh", False)),
                       "q5_fsm": q5_active_status(self._client),
                       "limits": {"min_position_rad": self._min_position, "max_position_rad": self._max_position,
                                  "max_step_rad": self._max_step, "max_duration_s": self._max_duration,
                                  "vendor_certified": False}})
        return status

    def _allowed(self, args):
        status = self._safety()
        if not status["ros_publisher_available"]:
            return _hand_failure("ROS_UNAVAILABLE", "Q5 hand command publisher is unavailable", status=status)
        if status["lifecycle_state"] != "active":
            return _hand_failure("LIFECYCLE_NOT_ACTIVE", "Q5 motion_manager must be active before hand control", status=status)
        q5_ready, q5_status = q5_is_control_ready(self._client)
        if not q5_ready:
            return _hand_failure("Q5_FSM_NOT_READY", "Q5 /xbot_state must be fresh and READY or ACTIVE before hand control",
                           status={**status, "q5_fsm": q5_status})
        if not status["joint_state_fresh"]:
            return _hand_failure("JOINT_STATE_UNAVAILABLE", "Refusing hand control without fresh /joint_states", status=status)
        return status

    def _targets(self, args):
        raw_targets = args.get("targets")
        if not isinstance(raw_targets, list) or not raw_targets:
            return _hand_failure("INVALID_ARGUMENT", "targets must be a non-empty list")
        targets = {}
        try:
            for item in raw_targets:
                if not isinstance(item, dict):
                    raise ValueError("each target must be an object")
                name = item.get("joint_name")
                if name not in HAND_JOINTS:
                    raise ValueError("joint_name is not an allowed XHand Lite joint")
                if name in targets:
                    raise ValueError(f"duplicate target for {name}")
                value = _hand_finite_number(item.get("position_rad"), "position_rad")
                if not self._min_position <= value <= self._max_position:
                    raise ValueError(f"position_rad for {name} is outside the configured guardrail")
                targets[name] = value
            duration_value = args.get("duration_s")
            duration = _hand_finite_number(
                min(0.5, self._max_duration) if duration_value is None else duration_value,
                "duration_s",
            )
        except ValueError as e:
            return _hand_failure("INVALID_ARGUMENT", str(e))
        if not 0.0 < duration <= self._max_duration:
            return _hand_failure("INVALID_ARGUMENT", "duration_s is outside the configured safe interval", max_duration_s=self._max_duration)
        current = self._client.snapshot().get("joints", {})
        missing = [name for name in targets if name not in current]
        if missing:
            return _hand_failure("JOINT_UNAVAILABLE", "Requested hand joint is absent from /joint_states", missing_joints=missing)
        # Large target changes are safe to accept here because _run() always
        # interpolates them into max_step_rad-sized position updates.
        return {"current": {name: float(current[name]) for name in targets}, "targets": targets, "duration_s": duration}

    def _profile_targets(self, action, args):
        side = args.get("side", "both")
        if side not in HAND_SIDES:
            return _hand_failure("INVALID_ARGUMENT", "side must be left, right, or both")
        if action in ("open_hand", "close_hand"):
            # Named open/close operations target the complete hand range. The
            # worker interpolates the change into bounded position steps.
            curl = None
        else:
            try:
                curl = _hand_finite_number(args.get("curl_rad"), "curl_rad")
            except ValueError as e:
                return _hand_failure("INVALID_ARGUMENT", str(e))
            if not self._min_position <= curl <= self._max_position:
                return _hand_failure("INVALID_ARGUMENT", "curl_rad is outside the configured guardrail")

        fingers = FINGERS
        if action == "set_finger":
            finger = args.get("finger")
            if finger not in FINGERS:
                return _hand_failure("INVALID_ARGUMENT", "finger无效")
            fingers = (finger,)
        current = self._client.snapshot().get("joints", {})
        targets = []
        for hand in ("left", "right"):
            if side not in (hand, "both"):
                continue
            for finger in fingers:
                suffixes = FINGER_JOINT_SUFFIXES[finger]
                if action == "set_finger" and finger == "thumb":
                    rotation_value = args.get("rotation_rad", current.get(f"{hand}_hand_thumb_rota_joint1"))
                    try:
                        rotation = _hand_finite_number(rotation_value, "rotation_rad")
                    except ValueError as e:
                        return _hand_failure("INVALID_ARGUMENT", str(e))
                    if not self._min_position <= rotation <= self._max_position:
                        return _hand_failure("INVALID_ARGUMENT", "旋转超范围")
                    suffixes = ("hand_thumb_bend_joint", "hand_thumb_rota_joint1")
                for index, suffix in enumerate(suffixes):
                    name = f"{hand}_{suffix}"
                    if curl is None:
                        if name not in current:
                            return _hand_failure("JOINT_UNAVAILABLE", "Requested hand joint is absent from /joint_states",
                                           missing_joints=[name])
                        value = self._min_position if action == "open_hand" else self._max_position
                    else:
                        value = curl
                        if action == "set_finger" and finger == "thumb" and index == 1:
                            value = rotation
                    targets.append({"joint_name": name, "position_rad": value})
        return self._targets({"targets": targets, "duration_s": args.get("duration_s")})

    def _start_motion(self, command, source):
        if not self._router.acquire(HAND_CONTROL_CARD):
            return _hand_failure("COMMAND_IN_PROGRESS", "Another Q5 hand card currently owns the command publisher", status=self._router.status())
        with self._lock:
            if self._motion_thread is not None and self._motion_thread.is_alive():
                self._router.release(HAND_CONTROL_CARD)
                return _hand_failure("MOTION_IN_PROGRESS", "A hand command is already active; call stop before another command")
            event = threading.Event()
            self._motion_stop = event
            self._active_command = {"source": source, "targets_rad": dict(command["targets"]),
                                    "duration_s": command["duration_s"], "started_at_ms": int(time.time() * 1000)}
            self._motion_thread = threading.Thread(target=self._run, args=(event, command), daemon=True, name="q5_hand_control")
            self._motion_thread.start()
        return {"ok": True, "state": "moving", "command": dict(self._active_command), "stops_by_holding_current_position": True}

    def _run(self, stop_event, command):
        current, targets, duration = command["current"], command["targets"], command["duration_s"]
        steps = max(int(math.ceil(duration * self._publish_rate)),
                    max(int(math.ceil(abs(targets[name] - current[name]) / self._max_step)) for name in targets), 1)
        try:
            for index in range(1, steps + 1):
                if stop_event.is_set():
                    break
                position = {name: current[name] + (targets[name] - current[name]) * index / steps for name in targets}
                self._router.publish(position)
                stop_event.wait(duration / steps)
        finally:
            self._router.release(HAND_CONTROL_CARD)
            with self._lock:
                if self._motion_stop is stop_event:
                    self._motion_stop = self._motion_thread = self._active_command = None

    def _hold_current(self):
        snap = self._client.snapshot()
        positions = snap.get("joints", {})
        if not snap.get("fresh") or any(name not in positions for name in HAND_JOINTS):
            return False
        if not self._router.acquire(HAND_CONTROL_CARD):
            return False
        try:
            published = False
            for _ in range(self._hold_repetitions):
                published = self._router.publish({name: float(positions[name]) for name in HAND_JOINTS}) or published
                time.sleep(1.0 / self._publish_rate)
            return published
        finally:
            self._router.release(HAND_CONTROL_CARD)

    def _stop(self, reason):
        with self._lock:
            event, thread, active = self._motion_stop, self._motion_thread, self._active_command
        if event is not None:
            event.set()
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=1.0)
        held = self._hold_current() if active else False
        return {"ok": True, "state": "stopped", "reason": reason, "hold_current_published": held}

    def dispatch(self, action, args):
        if action == "start":
            return {"state": "ready" if self._router.status()["ros_publisher_available"] else "unavailable", "safety": self._safety()}
        if action == "info":
            with self._lock:
                active = dict(self._active_command) if self._active_command else None
            return {"ok": True, "state": "moving" if active else "idle", "active_command": active, "safety": self._safety()}
        if action in ("cancel", "stop"):
            return self._stop("command")
        if action == "hold":
            allowed = self._allowed(args)
            if allowed.get("ok") is False:
                return allowed
            return {"ok": self._hold_current(), "state": "held"}
        if action not in ("set", "open_hand", "close_hand", "set_hand", "set_finger"):
            return None
        allowed = self._allowed(args)
        if allowed.get("ok") is False:
            return allowed
        command = self._targets(args) if action == "set" else self._profile_targets(action, args)
        if command.get("ok") is False:
            return command
        return self._start_motion(command, action)

    def stop(self):
        self._stop("driver_shutdown")




# ── Consolidated from hand_gesture.py ─────────────────────────────

"""Direct XHand Lite preset-gesture card.

Gesture values are deployment presets, not vendor-certified grasp limits. The
card delegates execution to ``hand_control`` so both cards share one direct
publisher and one in-process command lease.
"""


CARD = "hand_gesture"
TYPE = "actuator"
DESC = "Q5 XHand Lite 预设手势：张手、握拳、指向、捏取及常用手势"


def _side_pose(left_values, right_values, side):
    positions = {}
    if side in ("left", "both"):
        positions.update(left_values)
    if side in ("right", "both"):
        positions.update(right_values)
    return positions


def _paired(left):
    return left, {name.replace("left_", "right_", 1): value for name, value in left.items()}


def _preset(left_values):
    left, right = _paired(left_values)
    return {"left": left, "right": right}


PRESETS = {
    "open_hand": _preset({
        "left_hand_thumb_bend_joint": 0.0, "left_hand_thumb_rota_joint1": 0.0,
        "left_hand_index_joint1": 0.0, "left_hand_mid_joint1": 0.0,
        "left_hand_ring_joint1": 0.0, "left_hand_pinky_joint1": 0.0,
    }),
    "light_grip": _preset({
        "left_hand_thumb_bend_joint": 0.35, "left_hand_thumb_rota_joint1": 0.35,
        "left_hand_index_joint1": 0.45, "left_hand_mid_joint1": 0.45,
        "left_hand_ring_joint1": 0.45, "left_hand_pinky_joint1": 0.45,
    }),
    "closed_fist": _preset({
        "left_hand_thumb_bend_joint": 1.0, "left_hand_thumb_rota_joint1": 1.0,
        "left_hand_index_joint1": 1.0, "left_hand_mid_joint1": 1.0,
        "left_hand_ring_joint1": 1.0, "left_hand_pinky_joint1": 1.0,
    }),
    "point": _preset({
        "left_hand_thumb_bend_joint": 0.60, "left_hand_thumb_rota_joint1": 0.60,
        "left_hand_index_joint1": 0.0, "left_hand_mid_joint1": 1.0,
        "left_hand_ring_joint1": 1.0, "left_hand_pinky_joint1": 1.0,
    }),
    "pinch": _preset({
        "left_hand_thumb_bend_joint": 0.75, "left_hand_thumb_rota_joint1": 0.75,
        "left_hand_index_joint1": 0.75, "left_hand_mid_joint1": 0.10,
        "left_hand_ring_joint1": 0.10, "left_hand_pinky_joint1": 0.10,
    }),
    "victory": _preset({
        "left_hand_thumb_bend_joint": 0.30, "left_hand_thumb_rota_joint1": 0.30,
        "left_hand_index_joint1": 0.0, "left_hand_mid_joint1": 0.0,
        "left_hand_ring_joint1": 1.0, "left_hand_pinky_joint1": 1.0,
    }),
    "thumbs_up": _preset({
        "left_hand_thumb_bend_joint": 0.0, "left_hand_thumb_rota_joint1": 0.0,
        "left_hand_index_joint1": 1.0, "left_hand_mid_joint1": 1.0,
        "left_hand_ring_joint1": 1.0, "left_hand_pinky_joint1": 1.0,
    }),
    "ok_sign": _preset({
        "left_hand_thumb_bend_joint": 0.75, "left_hand_thumb_rota_joint1": 0.75,
        "left_hand_index_joint1": 0.75, "left_hand_mid_joint1": 0.0,
        "left_hand_ring_joint1": 0.0, "left_hand_pinky_joint1": 0.0,
    }),
    "three": _preset({
        "left_hand_thumb_bend_joint": 0.0, "left_hand_thumb_rota_joint1": 0.30,
        "left_hand_index_joint1": 0.0, "left_hand_mid_joint1": 0.0,
        "left_hand_ring_joint1": 0.0, "left_hand_pinky_joint1": 1.0,
    }),
    "rock": _preset({
        "left_hand_thumb_bend_joint": 0.0, "left_hand_thumb_rota_joint1": 0.30,
        "left_hand_index_joint1": 0.0, "left_hand_mid_joint1": 1.0,
        "left_hand_ring_joint1": 1.0, "left_hand_pinky_joint1": 0.0,
    }),
}
GESTURE_LABELS = {
    "open_hand": "张手", "light_grip": "轻握", "closed_fist": "完全握拳",
    "point": "指向", "pinch": "捏取", "victory": "胜利手势",
    "thumbs_up": "点赞", "ok_sign": "OK 手势", "three": "比三", "rock": "摇滚手势",
}


class HandGesturePlugin:
    def __init__(self, plugin_config, namespace, executor, client):
        # Reuse the complete command card's safety validation and command path.
        control_config = dict(plugin_config)
        control_config.setdefault("max_step_rad", 0.04)
        control_config.setdefault("min_position_rad", 0.0)
        control_config.setdefault("max_position_rad", 1.0)
        control_config.setdefault("hold_repetitions", 3)
        self._control = HandControlPlugin(control_config, namespace, executor, client)

    def get_tool(self):
        return {"name": CARD, "type": TYPE, "multiInstance": False, "description": DESC,
                "inputSchema": {"type": "object", "properties": {
                    "action": {"type": "string", "enum": ["start", *PRESETS, "cancel", "info"], "oneOf": [
                        {"const": "start", "title": "检查连接状态"},
                        *[{"const": name, "title": label} for name, label in GESTURE_LABELS.items()],
                        {"const": "cancel", "title": "取消并保持"},
                        {"const": "info", "title": "查看状态"},
                    ]},
                    "side": {"type": "string", "title": "执行侧", "enum": ["left", "right", "both"], "oneOf": [
                        {"const": "left", "title": "左手"}, {"const": "right", "title": "右手"},
                        {"const": "both", "title": "双手"},
                    ], "default": "both"},
                }, "required": ["action"], "additionalProperties": False,
                "x-action-params": {
                    "start": {"params": [], "description": "检查 ROS 连接和机器人状态。"},
                    **{name: {"params": ["side"], "description": f"对指定手执行{GESTURE_LABELS[name]}预设。"}
                       for name in PRESETS},
                    "cancel": {"params": [], "description": "取消当前手势，并保持当前位置。"},
                    "info": {"params": [], "description": "查看运动状态与安全条件。"},
                }}}

    def dispatch(self, action, args):
        if action in ("start", "info"):
            return self._control.dispatch(action, args)
        if action in ("cancel", "stop"):
            return self._control.dispatch("stop", args)
        if action not in PRESETS:
            return None
        gesture, side = action, args.get("side", "both")
        if side not in ("left", "right", "both"):
            return {"ok": False, "code": "INVALID_ARGUMENT", "message": "side must be left, right, or both", "details": {}}
        positions = _side_pose(PRESETS[gesture]["left"], PRESETS[gesture]["right"], side)
        command_args = {"targets": [{"joint_name": name, "position_rad": position} for name, position in positions.items()]}
        result = self._control.dispatch("set", command_args)
        if result.get("ok"):
            result["gesture"] = gesture
            result["side"] = side
            result["preset_vendor_certified"] = False
        return result

    def stop(self):
        self._control.stop()




# ── Consolidated from head_control.py ─────────────────────────────

"""Direct absolute neck position control for the Q5 model."""




HEAD_CARD = "head_control"
HEAD_TYPE = "actuator"
HEAD_TOPIC = "/wr1_controller/commands"
HEAD_JOINTS = ("neck_yaw_joint", "neck_pitch_joint")
HEAD_ACTIONS = {
    "neck_yaw": {
        "joint_name": "neck_yaw_joint", "title": "偏航：左右转头",
        "description": "范围[-0.79,0.79]rad；正负按坐标系。",
    },
    "neck_pitch": {
        "joint_name": "neck_pitch_joint", "title": "俯仰：抬头/低头",
        "description": "范围[-0.26,0.70]rad；正抬头负低头。",
    },
}
HEAD_DESC = "Q5 头部控制：偏航（左右转头）和俯仰（抬头/低头）"


def _head_failure(code, message, **details):
    return {"ok": False, "code": code, "message": message, "details": details}


def _head_number(value, field):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(float(value)):
        raise ValueError(f"{field} must be a finite number")
    return float(value)


class HeadControlPlugin:
    def __init__(self, plugin_config, namespace, executor, client):
        self._client = client
        self._router = _get_body_router(client, executor)
        self._max_step = float(plugin_config.get("max_step_rad", 0.025))
        self._rate = float(plugin_config.get("publish_rate_hz", 20.0))
        self._hold_repetitions = int(plugin_config.get("hold_repetitions", 3))
        if min(self._max_step, self._rate) <= 0 or self._hold_repetitions < 1:
            raise ValueError("head_control limits and rate must be positive")
        self._lock = threading.Lock()
        self._stop_event = self._thread = self._active = None

    def get_tool(self):
        limit_rules = []
        for action, detail in HEAD_ACTIONS.items():
            lower, upper = JOINT_LIMITS[detail["joint_name"]]
            limit_rules.append({
                "if": {"properties": {"action": {"const": action}}, "required": ["action"]},
                "then": {"properties": {"target_position_rad": {"minimum": lower, "maximum": upper}}},
            })
        return {"name": HEAD_CARD, "type": HEAD_TYPE, "multiInstance": False, "description": HEAD_DESC,
                "inputSchema": {"type": "object", "properties": {
                    "action": {"type": "string", "enum": ["start", *HEAD_ACTIONS, "cancel", "info"], "oneOf": [
                        {"const": "start", "title": "检查连接状态"},
                        *[{"const": action, "title": detail["title"], "description": detail["description"]}
                          for action, detail in HEAD_ACTIONS.items()],
                        {"const": "cancel", "title": "取消并保持"},
                        {"const": "info", "title": "查看状态"},
                    ]},
                    "target_position_rad": {"type": "number", "title": "目标角度 (rad)",
                                             "multipleOf": 0.005,
                                             "description": "先看范围，再填目标角度。"},
                }, "required": ["action"], "additionalProperties": False, "allOf": limit_rules,
                "x-action-params": {
                    "start": {"params": [], "description": "检查 ROS 连接和机器人状态。"},
                    **{action: {"params": ["target_position_rad"], "description": detail["description"]}
                       for action, detail in HEAD_ACTIONS.items()},
                    "cancel": {"params": [], "description": "取消当前微调，并保持当前位置。"},
                    "info": {"params": [], "description": "查看运动状态与安全条件。"},
                }}}

    def _safety(self):
        status = self._router.status()
        status.update({"control_mode": "direct_joint_position",
                "command_message": "xbot_common_interfaces/msg/HybridJointCommand",
                "lifecycle_state": self._client.get_lifecycle_state(),
                "joint_state_fresh": bool(self._client.snapshot().get("fresh", False)), "topic": HEAD_TOPIC,
                "q5_fsm": q5_active_status(self._client),
                "joints": list(HEAD_JOINTS), "joint_names_source": "q5_model.urdf",
                "limits": limits_for(HEAD_JOINTS)})
        return status

    def _allowed(self, args, joint_name=None):
        status = self._safety()
        if not status["ros_publisher_available"]:
            return _head_failure("ROS_UNAVAILABLE", "Q5 body command publisher is unavailable", status=status)
        if status["lifecycle_state"] != "active" or not status["joint_state_fresh"]:
            return _head_failure("ROBOT_NOT_READY", "Q5 must be active with fresh /joint_states before head control", status=status)
        q5_ready, q5_status = q5_is_control_ready(self._client)
        if not q5_ready:
            return _head_failure("Q5_FSM_NOT_READY", "Q5 /xbot_state must be fresh and READY or ACTIVE before head control",
                            status={**status, "q5_fsm": q5_status})
        if joint_name is not None and joint_name not in self._client.snapshot().get("joints", {}):
            return _head_failure("HEAD_MODEL_MISMATCH", "Configured neck joint is absent from /joint_states", joint_name=joint_name)
        return status

    def _publish(self, name, position):
        return self._router.publish({name: position})

    def _hold_position(self, name, position):
        if position is None:
            return False
        published = False
        for _ in range(self._hold_repetitions):
            published = self._publish(name, float(position)) or published
            time.sleep(1.0 / self._rate)
        return published

    def _hold_current(self, name):
        snap = self._client.snapshot()
        value = snap.get("joints", {}).get(name)
        if not snap.get("fresh") or value is None:
            return False
        return self._hold_position(name, value)

    def _run(self, event, name, current, target, duration):
        steps = max(int(math.ceil(abs(target - current) / self._max_step)), int(math.ceil(duration * self._rate)), 1)
        try:
            for index in range(1, steps + 1):
                if event.is_set():
                    break
                self._publish(name, current + (target - current) * index / steps)
                event.wait(duration / steps)
        finally:
            # Joint feedback can lag the last command. Reusing it after a
            # successful move sends the neck back to its start angle.
            self._hold_position(name, target) if not event.is_set() else self._hold_current(name)
            self._router.release(HEAD_CARD)
            with self._lock:
                if self._stop_event is event:
                    self._stop_event = self._thread = self._active = None

    def _stop(self, reason):
        with self._lock:
            event, thread, active = self._stop_event, self._thread, self._active
        if event is not None:
            event.set()
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=1.0)
        # The active worker sends its final hold while it still owns the shared
        # publisher, then releases the lease in _run().
        return {"ok": True, "state": "stopped", "reason": reason,
                "hold_command_attempted": bool(active)}

    def dispatch(self, action, args):
        if action == "start":
            return {"state": "ready" if self._router.status()["ros_publisher_available"] else "unavailable", "safety": self._safety()}
        if action == "info":
            with self._lock:
                active = dict(self._active) if self._active else None
            return {"ok": True, "state": "moving" if active else "idle", "active_command": active, "safety": self._safety()}
        if action in ("cancel", "stop"):
            return self._stop("command")
        detail = HEAD_ACTIONS.get(action)
        if detail is None:
            return None
        name = detail["joint_name"]
        allowed = self._allowed(args, name)
        if allowed.get("ok") is False:
            return allowed
        try:
            target = _head_number(args.get("target_position_rad"), "target_position_rad")
        except ValueError as e:
            return _head_failure("INVALID_ARGUMENT", str(e))
        lower, upper = JOINT_LIMITS.get(name, (None, None))
        if lower is None or target < lower or target > upper:
            return _head_failure("LIMIT_EXCEEDED", "target_position_rad is outside the joint safety limits",
                            joint_name=name, min_rad=lower, max_rad=upper, target_position_rad=target)
        current = float(self._client.snapshot()["joints"][name])
        # Retain the established per-sample interpolation bound. A large but
        # legal target takes longer; it is not sent as a fast jump.
        duration = max(0.5, abs(target - current) / (self._max_step * self._rate))
        if not self._router.acquire(HEAD_CARD):
            return _head_failure("COMMAND_IN_PROGRESS", "Another Q5 body card currently owns the command publisher",
                            status=self._router.status())
        with self._lock:
            if self._thread is not None and self._thread.is_alive():
                self._router.release(HEAD_CARD)
                return _head_failure("MOTION_IN_PROGRESS", "A head adjustment is already active; call stop first")
            event = threading.Event()
            self._stop_event = event
            self._active = {"joint_name": name, "start_position_rad": current, "target_position_rad": target, "duration_s": duration, "started_at_ms": int(time.time() * 1000)}
            self._thread = threading.Thread(target=self._run, args=(event, name, current, target, duration), daemon=True, name="q5_head_control")
            self._thread.start()
        return {"ok": True, "state": "moving", "head_action": action,
                "joint_name": name, "command": dict(self._active),
                "stops_by_holding_current_position": True}

    def stop(self):
        self._stop("driver_shutdown")
