"""Direct absolute neck position control for the Q5 model."""

from __future__ import annotations

import math
import threading
import time

from body_command import get_router
from control_contract import q5_active_status, q5_is_control_ready
from joint_limits import JOINT_LIMITS, limits_for

CARD = "head_control"
TYPE = "actuator"
TOPIC = "/wr1_controller/commands"
HEAD_JOINTS = ("neck_yaw_joint", "neck_pitch_joint")
HEAD_ACTIONS = {
    "neck_yaw": {
        "joint_name": "neck_yaw_joint", "title": "偏航：左右转头",
        "field": "neck_yaw_rad",
        "description": "范围[-0.79,0.79]rad；正负按坐标系。",
    },
    "neck_pitch": {
        "joint_name": "neck_pitch_joint", "title": "俯仰：抬头/低头",
        "field": "neck_pitch_rad",
        "description": "范围[-0.26,0.70]rad；正抬头负低头。",
    },
}
DESC = "Q5 头部控制：偏航（左右转头）和俯仰（抬头/低头）"


def _failure(code, message, **details):
    return {"ok": False, "code": code, "message": message, "details": details}


def _number(value, field):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(float(value)):
        raise ValueError(f"{field} must be a finite number")
    return float(value)


class Plugin:
    def __init__(self, plugin_config, namespace, executor, client):
        self._client = client
        self._router = get_router(client, executor)
        self._max_step = float(plugin_config.get("max_step_rad", 0.025))
        self._rate = float(plugin_config.get("publish_rate_hz", 20.0))
        self._hold_repetitions = int(plugin_config.get("hold_repetitions", 3))
        if min(self._max_step, self._rate) <= 0 or self._hold_repetitions < 1:
            raise ValueError("head_control limits and rate must be positive")
        self._lock = threading.Lock()
        self._stop_event = self._thread = self._active = None

    def get_tool(self):
        position_fields = {}
        for detail in HEAD_ACTIONS.values():
            lower, upper = JOINT_LIMITS[detail["joint_name"]]
            position_fields[detail["field"]] = {
                "type": "number", "title": f"目标角度 (rad)",
                "minimum": lower, "maximum": upper, "multipleOf": 0.005,
                "description": detail["description"],
            }
        return {"name": CARD, "type": TYPE, "multiInstance": False, "description": DESC,
                "inputSchema": {"type": "object", "properties": {
                    "action": {"type": "string", "enum": ["start", *HEAD_ACTIONS, "cancel", "info"], "oneOf": [
                        {"const": "start", "title": "检查连接状态"},
                        *[{"const": action, "title": detail["title"], "description": detail["description"]}
                          for action, detail in HEAD_ACTIONS.items()],
                        {"const": "cancel", "title": "取消并保持"},
                        {"const": "info", "title": "查看状态"},
                    ]},
                    **position_fields,
                }, "required": ["action"], "additionalProperties": False,
                "x-action-params": {
                    "start": {"params": [], "description": "检查 ROS 连接和机器人状态。"},
                    **{action: {"params": [detail["field"]], "description": detail["description"]}
                       for action, detail in HEAD_ACTIONS.items()},
                    "cancel": {"params": [], "description": "取消当前微调，并保持当前位置。"},
                    "info": {"params": [], "description": "查看运动状态与安全条件。"},
                }}}

    def _safety(self):
        status = self._router.status()
        status.update({"control_mode": "direct_joint_position",
                "command_message": "xbot_common_interfaces/msg/HybridJointCommand",
                "lifecycle_state": self._client.get_lifecycle_state(),
                "joint_state_fresh": bool(self._client.snapshot().get("fresh", False)), "topic": TOPIC,
                "q5_fsm": q5_active_status(self._client),
                "joints": list(HEAD_JOINTS), "joint_names_source": "q5_model.urdf",
                "limits": limits_for(HEAD_JOINTS)})
        return status

    def _allowed(self, args, joint_name=None):
        status = self._safety()
        if not status["ros_publisher_available"]:
            return _failure("ROS_UNAVAILABLE", "Q5 body command publisher is unavailable", status=status)
        if status["lifecycle_state"] != "active" or not status["joint_state_fresh"]:
            return _failure("ROBOT_NOT_READY", "Q5 must be active with fresh /joint_states before head control", status=status)
        q5_ready, q5_status = q5_is_control_ready(self._client)
        if not q5_ready:
            return _failure("Q5_FSM_NOT_READY", "Q5 /xbot_state must be fresh and READY or ACTIVE before head control",
                            status={**status, "q5_fsm": q5_status})
        if joint_name is not None and joint_name not in self._client.snapshot().get("joints", {}):
            return _failure("HEAD_MODEL_MISMATCH", "Configured neck joint is absent from /joint_states", joint_name=joint_name)
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
            self._router.release(CARD)
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
            target = _number(args.get(detail["field"]), detail["field"])
        except ValueError as e:
            return _failure("INVALID_ARGUMENT", str(e))
        lower, upper = JOINT_LIMITS.get(name, (None, None))
        if lower is None or target < lower or target > upper:
            return _failure("LIMIT_EXCEEDED", "target is outside the joint safety limits",
                            joint_name=name, min_rad=lower, max_rad=upper, target_position_rad=target)
        current = float(self._client.snapshot()["joints"][name])
        # Retain the established per-sample interpolation bound. A large but
        # legal target takes longer; it is not sent as a fast jump.
        duration = max(0.5, abs(target - current) / (self._max_step * self._rate))
        if not self._router.acquire(CARD):
            return _failure("COMMAND_IN_PROGRESS", "Another Q5 body card currently owns the command publisher",
                            status=self._router.status())
        with self._lock:
            if self._thread is not None and self._thread.is_alive():
                self._router.release(CARD)
                return _failure("MOTION_IN_PROGRESS", "A head adjustment is already active; call stop first")
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


def make_plugin(plugin_config, namespace, executor, client):
    return Plugin(plugin_config, namespace, executor, client)
