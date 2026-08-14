"""Direct absolute Q5 arm joint-position control card.

The card accepts one absolute target for one allowlisted body joint. Targets
are validated against the bundled URDF limits before interpolation.
"""

from __future__ import annotations

import math
import threading
import time

from body_command import get_router
from control_contract import q5_active_status, q5_is_control_ready
from joint_limits import JOINT_LIMITS, limits_for

CARD = "arm_control"
TYPE = "actuator"
TOPIC = "/wr1_controller/commands"
DESC = "Q5 手臂控制：将单个关节设置到指定安全角度"
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


def _failure(code: str, message: str, **details) -> dict:
    return {"ok": False, "code": code, "message": message, "details": details}


def _number(value, field: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{field} must be a number")
    value = float(value)
    if not math.isfinite(value):
        raise ValueError(f"{field} must be finite")
    return value


def _field_name(joint_name: str) -> str:
    return f"{joint_name.removesuffix('_joint')}_rad"


class Plugin:
    def __init__(self, plugin_config, namespace, executor, client):
        self._client = client
        # Hardware output is opt-in. The deployment config explicitly enables
        # this card after the operator has verified the Q5 command interface.
        self._hardware_enabled = bool(plugin_config.get("hardware_enable", False))
        self._router = get_router(client, executor) if self._hardware_enabled else None
        self._max_step = float(plugin_config.get("max_step_rad", 0.010))
        # Q5's vendor example moves an arm in 0.01-rad samples spaced 0.3 s
        # apart. This is deliberately far slower than neck interpolation.
        self._publish_rate = float(plugin_config.get("publish_rate_hz", 1.0 / 0.3))
        self._max_target_delta = float(plugin_config.get("max_target_delta_rad", 0.050))
        self._hold_repetitions = int(plugin_config.get("hold_repetitions", 3))
        self._settle_tolerance = float(plugin_config.get("settle_tolerance_rad", 0.035))
        self._settle_timeout = float(plugin_config.get("settle_timeout_s", 1.5))
        if min(self._max_step, self._publish_rate, self._max_target_delta) <= 0:
            raise ValueError("arm_control limits and publish rate must be positive")
        if self._hold_repetitions < 1:
            raise ValueError("arm_control hold_repetitions must be at least 1")
        if self._settle_tolerance <= 0 or self._settle_timeout <= 0:
            raise ValueError("arm_control settling limits must be positive")

        self._lock = threading.Lock()
        self._motion_stop = None
        self._motion_thread = None
        self._active_command = None

    def get_tool(self):
        action_details = {
            name: {"joint_name": name, "field": _field_name(name),
                   "title": ARM_JOINT_LABELS[name], "limits": JOINT_LIMITS[name]}
            for name in ARM_JOINTS
        }
        position_fields = {
            detail["field"]: {
                "type": "number", "title": "目标角度 (rad)", "multipleOf": 0.005,
                "minimum": detail["limits"][0], "maximum": detail["limits"][1],
                "description": (f"范围[{detail['limits'][0]:g},{detail['limits'][1]:g}]rad；"
                                f"单次差不超过{self._max_target_delta:g}rad"),
            }
            for detail in action_details.values()
        }
        return {
            "name": CARD,
            "type": TYPE,
            "multiInstance": False,
            "description": DESC,
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {"type": "string", "enum": ["start", *action_details, "cancel", "info"], "oneOf": [
                        {"const": "start", "title": "检查连接状态"},
                        *[{"const": name, "title": detail["title"]}
                          for name, detail in action_details.items()],
                        {"const": "cancel", "title": "取消并保持当前角度"},
                        {"const": "info", "title": "查看状态"},
                    ]},
                    **position_fields,
                },
                "required": ["action"],
                "additionalProperties": False,
                "x-action-params": {
                    "start": {"params": [], "description": "检查 ROS 连接和机器人状态。"},
                    **{name: {"params": [detail["field"]],
                                "description": (f"{detail['title']}；范围[{detail['limits'][0]:g},{detail['limits'][1]:g}]rad；"
                                                f"单次差不超过{self._max_target_delta:g}rad")}
                       for name, detail in action_details.items()},
                    "cancel": {"params": [], "description": "取消微调，并保持当前关节角度。"},
                    "info": {"params": [], "description": "查看当前运动和安全条件。"},
                },
            },
        }

    def _safety(self) -> dict:
        status = self._router.status() if self._router is not None else {
            "ros_publisher_available": False,
            "endpoint_query_available": False,
            "other_publishers": [],
            "active_owner": None,
            "topic": TOPIC,
            "publisher_node": None,
        }
        status.update({
            "hardware_enabled": self._hardware_enabled,
            "control_mode": "direct_joint_position",
            "command_message": "xbot_common_interfaces/msg/HybridJointCommand",
            "lifecycle_state": self._client.get_lifecycle_state(),
            "joint_state_fresh": bool(self._client.snapshot().get("fresh", False)),
            "q5_fsm": q5_active_status(self._client),
            "limits": {"max_step_rad": self._max_step,
                       "max_target_delta_rad": self._max_target_delta,
                       "joint_position_limits": limits_for(ARM_JOINTS),
                       "settle_tolerance_rad": self._settle_tolerance,
                       "settle_timeout_s": self._settle_timeout,
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

    def _hold_target_until_settled(self, stop_event, joint_name: str, target: float) -> bool:
        """Keep the target authoritative while the physical arm catches up.

        Arm feedback reaches its target later than the neck.  Releasing the
        direct publisher after only a few packets lets the resident controller
        immediately restore its earlier target, which looks like a shake.
        """
        deadline = time.monotonic() + self._settle_timeout
        published = False
        while not stop_event.is_set() and time.monotonic() < deadline:
            ready, _ = q5_is_control_ready(self._client)
            if self._client.get_lifecycle_state() != "active" or not ready:
                return published
            published = self._publish(joint_name, target) or published
            snap = self._client.snapshot()
            measured = snap.get("joints", {}).get(joint_name)
            if snap.get("fresh") and measured is not None and abs(float(measured) - target) <= self._settle_tolerance:
                self._hold_position(joint_name, target)
                return True
            stop_event.wait(1.0 / self._publish_rate)
        return published

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
                ready, _ = q5_is_control_ready(self._client)
                if self._client.get_lifecycle_state() != "active" or not ready:
                    stop_event.set()
                    break
                position = current + (target - current) * (index / steps)
                self._publish(joint_name, position)
                stop_event.wait(duration_s / steps)
        finally:
            # The joint-state stream may still contain the pre-command angle
            # when the final interpolation point is sent. Hold the target on
            # successful completion; cancellation continues to hold feedback.
            if not stop_event.is_set():
                self._hold_target_until_settled(stop_event, joint_name, target)
            elif q5_is_control_ready(self._client)[0] and self._client.get_lifecycle_state() == "active":
                self._hold_current(joint_name)
            self._router.release(CARD)
            with self._lock:
                if self._motion_stop is stop_event:
                    self._motion_stop = None
                    self._motion_thread = None
                    self._active_command = None

    def _stop(self, reason: str) -> dict:
        if self._router is None:
            return {"ok": True, "state": "idle", "reason": reason,
                    "message": "arm_control disabled; no Q5 command was sent"}
        with self._lock:
            stop_event = self._motion_stop
            motion_thread = self._motion_thread
            active = dict(self._active_command) if self._active_command else None
            self._motion_stop = None
            self._motion_thread = None
            self._active_command = None
        if stop_event is not None:
            stop_event.set()
        can_hold = (q5_is_control_ready(self._client)[0]
                    and self._client.get_lifecycle_state() == "active")
        held = bool(active and can_hold and self._hold_current(active["joint_name"]))
        if motion_thread is not None and motion_thread is not threading.current_thread():
            motion_thread.join(timeout=1.0)
        return {"ok": True, "state": "stopped", "reason": reason,
                "hold_command_published": held}

    def _validate_move(self, joint_name: str, target_value):
        if not self._hardware_enabled:
            return _failure(
                "ARM_CONTROL_DISABLED",
                "Q5 arm control is disabled by configuration",
                status=self._safety(),
            )
        status = self._safety()
        if not status["ros_publisher_available"]:
            return _failure("ROS_UNAVAILABLE", "Q5 arm command publisher is unavailable", status=status)
        # /wr1_controller/commands has a single downstream controller, not a
        # command arbiter.  A resident publisher such as mpc_policy_node can
        # keep sending a full-body target while this card sends one arm target.
        # That produces alternating setpoints on the same joint and has caused
        # the Q5 joint protection to fault.  The operator must use the vendor
        # handoff procedure before direct arm commands are allowed.
        if status.get("other_publishers", []):
            return _failure(
                "BODY_COMMAND_CONFLICT",
                "External ROS publisher owns /wr1_controller/commands; complete the vendor MPC handoff before arm control",
                status=status,
            )
        if status["lifecycle_state"] != "active":
            return _failure("LIFECYCLE_NOT_ACTIVE", "Q5 motion_manager must be active before arm control", status=status)
        q5_ready, q5_status = q5_is_control_ready(self._client)
        if not q5_ready:
            return _failure("Q5_FSM_NOT_READY", "Q5 /xbot_state must be fresh and READY or ACTIVE before arm control",
                            status={**status, "q5_fsm": q5_status})
        snap = self._client.snapshot()
        if not snap.get("fresh"):
            return _failure("JOINT_STATE_UNAVAILABLE", "Refusing arm control without fresh /joint_states")
        current = snap.get("joints", {}).get(joint_name)
        if current is None:
            return _failure("JOINT_UNAVAILABLE", "Requested arm joint is absent from /joint_states", joint_name=joint_name)
        try:
            target = _number(target_value, _field_name(joint_name))
        except ValueError as e:
            return _failure("INVALID_ARGUMENT", str(e))
        lower, upper = JOINT_LIMITS.get(joint_name, (None, None))
        if lower is None or target < lower or target > upper:
            return _failure("LIMIT_EXCEEDED", "target is outside the joint safety limits",
                            joint_name=joint_name, min_rad=lower, max_rad=upper,
                            target_position_rad=target)
        delta = target - float(current)
        if abs(delta) > self._max_target_delta:
            return _failure(
                "TARGET_DELTA_EXCEEDED",
                "Target is too far from the current arm position; Q5 requires a small incremental arm adjustment",
                joint_name=joint_name,
                current_position_rad=float(current),
                target_position_rad=target,
                max_target_delta_rad=self._max_target_delta,
            )
        # A legal full-range move must not turn into a fast jump. The existing
        # max step and publication rate bound interpolation speed instead.
        duration_s = max(1.0 / self._publish_rate, abs(delta) / (self._max_step * self._publish_rate))
        return joint_name, float(current), target, duration_s

    def start(self):
        if not self._hardware_enabled:
            return {"state": "disabled", "message": "arm_control disabled; no Q5 command publisher created"}
        return {"state": "ready" if self._router.status()["ros_publisher_available"] else "unavailable"}

    def stop(self):
        self._stop("driver_shutdown")

    def dispatch(self, action, args):
        if action == "start":
            return {**self.start(), "safety": self._safety()}
        if action in ("cancel", "stop"):
            return self._stop("command")
        if action == "info":
            with self._lock:
                active = dict(self._active_command) if self._active_command else None
            return {"ok": True, "state": "moving" if active else ("disabled" if not self._hardware_enabled else "idle"), "active_command": active,
                    "safety": self._safety()}
        if action not in ARM_JOINTS:
            return None

        command = self._validate_move(action, args.get(_field_name(action)))
        if isinstance(command, dict):
            return command
        joint_name, current, target, duration_s = command
        if not self._router.acquire(CARD):
            return _failure("COMMAND_IN_PROGRESS", "Another Q5 body card currently owns the command publisher",
                            status=self._router.status())
        with self._lock:
            if self._motion_thread is not None and self._motion_thread.is_alive():
                self._router.release(CARD)
                return _failure("MOTION_IN_PROGRESS", "An arm movement is already active; call stop before another move")
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


def make_plugin(plugin_config, namespace, executor, client):
    return Plugin(plugin_config, namespace, executor, client)
