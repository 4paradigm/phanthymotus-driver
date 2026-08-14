"""Q5 position-control mode and direct arm-control cards."""

from __future__ import annotations

import math
import threading
import time

from rclpy.action import ActionClient
from rclpy.node import Node
from std_srvs.srv import Trigger
from xbot_common_interfaces.action import SimpleActions
from xbot_common_interfaces.srv import DynamicLaunch

from body_command import get_router as _get_body_router
from control_contract import q5_active_status, q5_is_control_ready
from joint_limits import JOINT_LIMITS, limits_for


# ── Arm control ──────────────────────────────────────────────────────────────

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


def _arm_field_name(joint_name: str) -> str:
    return f"{joint_name.removesuffix('_joint')}_rad"


def _arm_limit_summary() -> str:
    """Human-readable limits for clients that do not render JSON Schema allOf."""
    return "; ".join(
        f"{ARM_JOINT_LABELS[name]} {JOINT_LIMITS[name][0]:g}~{JOINT_LIMITS[name][1]:g} rad"
        for name in ARM_JOINTS
    )


ARM_DESC = (
    "关节绝对角度范围：" + _arm_limit_summary() + "。"
    "Q5 手臂单关节位置控制；每个关节动作使用自己的绝对角度参数（不是增量），"
    "需由 q5_control_mode 完成位置直控准备后使用。每步最多 0.010 rad、20 Hz，最大约 0.20 rad/s。"
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


class Q5ControlModePlugin:
    """Vendor Q5 control-mode transitions, kept separate from joint movement."""

    def __init__(self, plugin_config, namespace, executor, client):
        del plugin_config, namespace
        self._client = client
        self._node = Node("q5_control_mode")
        executor.add_node(self._node)
        self._dynamic = self._node.create_client(DynamicLaunch, "/dynamic_launch")
        self._ready = self._node.create_client(Trigger, "/ready_service")
        self._activate = self._node.create_client(Trigger, "/activate_service")
        self._actions = ActionClient(self._node, SimpleActions, "/simple_actions")
        # Q5 exposes no service that tells us which DynamicLaunch mode owns
        # ACTIVE. Do not infer direct-position ownership from ACTIVE alone.
        self._client.q5_position_control_prepared = False

    def get_tool(self):
        return {
            "name": "q5_control_mode", "type": "actuator", "multiInstance": False,
            "description": "Q5 模式切换：位置直控准备、READY 与 ACTIVE。完整准备会实际垂手并抬臂。",
            "inputSchema": {"type": "object", "properties": {
                "action": {"type": "string", "enum": ["start", "prepare_position_control", "ready", "active", "info"], "oneOf": [
                    {"const": "start", "title": "检查模式服务"},
                    {"const": "prepare_position_control", "title": "准备位置直控"},
                    {"const": "ready", "title": "切换 READY"},
                    {"const": "active", "title": "切换 ACTIVE"},
                    {"const": "info", "title": "查看模式状态"},
                ]},
            }, "required": ["action"], "additionalProperties": False,
            "x-action-params": {
                "start": {"params": [], "description": "检查 Q5 模式服务是否可用。"},
                "prepare_position_control": {"params": [], "description": "依次执行 pos、READY、垂手、抬臂、ACTIVE；供 arm_control 位置直控使用。"},
                "ready": {"params": [], "description": "启动 pos 并切到 READY；不会执行垂手或抬臂，且不会解锁 arm_control。"},
                "active": {"params": [], "description": "调用厂商 activate_service。仅在此前完整准备未失效时解锁 arm_control。"},
                "info": {"params": [], "description": "查看 Q5 FSM 与本驱动的位置直控准备状态。"},
            }},
        }

    @staticmethod
    def _wait_future(future, timeout):
        deadline = time.monotonic() + timeout
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.02)
        return future.result() if future.done() else None

    def _status(self):
        return {
            "q5_fsm": q5_active_status(self._client),
            "position_control_prepared": bool(getattr(self._client, "q5_position_control_prepared", False)),
            "services": {
                "dynamic_launch": self._dynamic.service_is_ready(),
                "ready_service": self._ready.service_is_ready(),
                "activate_service": self._activate.service_is_ready(),
                "simple_actions": self._actions.server_is_ready(),
            },
        }

    def _failure(self, code, message, **details):
        return {"ok": False, "code": code, "message": message, "details": details}

    def _launch_pos(self, steps):
        if not self._dynamic.wait_for_service(timeout_sec=10.0):
            return self._failure("DYNAMIC_LAUNCH_UNAVAILABLE", "Q5 /dynamic_launch is unavailable", steps=steps)
        request = DynamicLaunch.Request()
        request.app_name, request.sync_control, request.launch_mode = "", False, "pos"
        response = self._wait_future(self._dynamic.call_async(request), 15.0)
        success = bool(response and response.success)
        steps.append({"step": "dynamic_launch_pos", "success": success,
                      "message": getattr(response, "message", "timeout") if response else "timeout"})
        return None if success else self._failure("DYNAMIC_LAUNCH_FAILED", "Q5 position launch failed", steps=steps)

    def _to_ready(self, steps):
        error = self._launch_pos(steps)
        if error:
            return error
        if not self._ready.wait_for_service(timeout_sec=10.0):
            return self._failure("READY_SERVICE_UNAVAILABLE", "Q5 /ready_service is unavailable", steps=steps)
        response = self._wait_future(self._ready.call_async(Trigger.Request()), 30.0)
        success = bool(response and response.success)
        steps.append({"step": "ready_service", "success": success,
                      "message": getattr(response, "message", "timeout") if response else "timeout"})
        return None if success else self._failure("READY_FAILED", "Q5 ready initialization failed", steps=steps)

    def _to_active(self, steps):
        if not self._activate.wait_for_service(timeout_sec=10.0):
            return self._failure("ACTIVATE_SERVICE_UNAVAILABLE", "Q5 /activate_service is unavailable", steps=steps)
        response = self._wait_future(self._activate.call_async(Trigger.Request()), 15.0)
        success = bool(response and response.success)
        steps.append({"step": "activate_service", "success": success,
                      "message": getattr(response, "message", "timeout") if response else "timeout"})
        return None if success else self._failure("ACTIVATE_FAILED", "Q5 activation failed", steps=steps)

    def _prepare(self):
        steps = []
        self._client.q5_position_control_prepared = False
        error = self._to_ready(steps)
        if error:
            return error
        if not self._actions.wait_for_server(timeout_sec=5.0):
            return self._failure("SIMPLE_ACTIONS_UNAVAILABLE", "Q5 /simple_actions is unavailable", steps=steps)
        for name in ("initpose_handsdown", "lift_up"):
            goal = SimpleActions.Goal()
            goal.action_name, goal.time_cost = name, 4.0
            handle = self._wait_future(self._actions.send_goal_async(goal), 8.0)
            result = self._wait_future(handle.get_result_async(), 35.0) if handle and handle.accepted else None
            success = bool(result and getattr(result.result, "result", 2) == 0)
            steps.append({"step": name, "success": success,
                          "message": getattr(result.result, "message", "timeout") if result else "timeout"})
            if not success:
                return self._failure("SIMPLE_ACTION_FAILED", f"Q5 action {name} failed", steps=steps)
        error = self._to_active(steps)
        if error:
            return error
        self._client.q5_position_control_prepared = True
        return {"ok": True, "state": "active", "position_control_prepared": True, "steps": steps}

    def start(self):
        return {"state": "ready", "status": self._status()}

    def stop(self):
        pass

    def dispatch(self, action, args):
        del args
        if action in ("start", "info"):
            return {"ok": True, "state": "ready", "status": self._status()}
        if action == "prepare_position_control":
            return self._prepare()
        if action == "ready":
            self._client.q5_position_control_prepared = False
            steps = []
            error = self._to_ready(steps)
            return error or {"ok": True, "state": "ready", "position_control_prepared": False, "steps": steps}
        if action == "active":
            steps = []
            error = self._to_active(steps)
            return error or {"ok": True, "state": "active",
                             "position_control_prepared": bool(getattr(self._client, "q5_position_control_prepared", False)),
                             "steps": steps}
        return None


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

    def get_tool(self):
        action_details = {
            name: {"field": _arm_field_name(name), "title": ARM_JOINT_LABELS[name],
                   "limits": JOINT_LIMITS[name]}
            for name in ARM_JOINTS
        }
        position_fields = {
            detail["field"]: {
                "type": "number", "title": "目标绝对角度 (rad)", "multipleOf": 0.005,
                "minimum": detail["limits"][0], "maximum": detail["limits"][1],
                "description": f"范围[{detail['limits'][0]:g},{detail['limits'][1]:g}]rad；绝对角度，不是相对位移。",
            }
            for detail in action_details.values()
        }
        return {
            "name": ARM_CARD,
            "type": ARM_TYPE,
            "multiInstance": False,
            "description": ARM_DESC,
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
                                "description": f"{detail['title']}；范围[{detail['limits'][0]:g},{detail['limits'][1]:g}]rad；最大速度约 0.20 rad/s。"}
                       for name, detail in action_details.items()},
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

    def _validate_move(self, joint_name: str, target_value):
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
        if not bool(getattr(self._client, "q5_position_control_prepared", False)):
            return _arm_failure(
                "DIRECT_CONTROL_NOT_PREPARED",
                "Run q5_control_mode action=prepare_position_control first; vendor position-control sequence has not completed in this driver session",
                status=status,
            )
        # Direct HybridJointCommand control is owned by the vendor body
        # controller after arm_control preparation completes. motion_manager is a
        # separate lifecycle node and may legitimately remain inactive.
        q5_ready, q5_status = q5_is_control_ready(self._client)
        if not q5_ready or q5_status.get("state") != 4:
            return _arm_failure("Q5_FSM_NOT_ACTIVE", "Q5 must remain fresh and ACTIVE after position-control preparation before arm control",
                            status={**status, "q5_fsm": q5_status})
        snap = self._client.snapshot()
        if not snap.get("fresh"):
            return _arm_failure("JOINT_STATE_UNAVAILABLE", "Refusing arm control without fresh /joint_states")
        current = snap.get("joints", {}).get(joint_name)
        if current is None:
            return _arm_failure("JOINT_UNAVAILABLE", "Requested arm joint is absent from /joint_states", joint_name=joint_name)
        try:
            target = _arm_number(target_value, _arm_field_name(joint_name))
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
            return {"ok": True, "state": "moving" if active else "idle", "active_command": active,
                    "safety": self._safety()}
        if action not in ARM_JOINTS:
            return None

        command = self._validate_move(action, args.get(_arm_field_name(action)))
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
