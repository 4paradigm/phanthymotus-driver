"""Direct absolute waist position control for the Q5 model."""

from __future__ import annotations

import math
import threading
import time

from body_command import get_router
from control_contract import q5_active_status, q5_is_control_ready
from joint_limits import JOINT_LIMITS, limits_for
from q5_acp import notify as _acp_notify

try:
    from legacy_direct_control import PositionControlPreparer
except ImportError:
    PositionControlPreparer = None

CARD = "waist_control"
TYPE = "actuator"
TOPIC = "/wr1_controller/commands"
WAIST_JOINTS = ("waist_yaw_joint",)
WAIST_ACTIONS = {
    "waist_yaw": {
        "joint_name": "waist_yaw_joint", "title": "偏航：左右扭腰",
        "field": "waist_yaw_rad",
        "description": "范围[-1.57,1.57]rad；正负按坐标系。",
    },
}
DESC = "Q5 腰部控制：偏航（左右扭腰）"


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
            raise ValueError("waist_control limits and rate must be positive")
        self._lock = threading.Lock()
        self._stop_event = self._thread = self._active = None

        # Embedded position-control preparer so waist_control can self-prepare.
        self._preparer = None
        if PositionControlPreparer is not None:
            try:
                self._preparer = PositionControlPreparer(plugin_config, namespace, executor, client)
            except Exception:
                self._preparer = None

    def _ensure_prepared(self) -> dict | None:
        """Auto-prepare position control if not already prepared.

        Returns None on success (or already prepared), or an error dict on failure.
        """
        if bool(getattr(self._client, "q5_position_control_prepared", False)):
            return None
        if self._preparer is None:
            return _failure("CONTROL_MODE_UNAVAILABLE",
                            "Position-control preparer is not initialized; cannot auto-prepare")
        print("[waist_control] position control not prepared, auto-preparing...")
        result = self._preparer._prepare()
        if isinstance(result, dict) and result.get("ok") is False:
            return result
        if not bool(getattr(self._client, "q5_position_control_prepared", False)):
            return _failure("PREPARE_FAILED", "Auto-prepare did not set q5_position_control_prepared",
                            prepare_result=result)
        return None

    def _reset(self, args):
        """Interpolate all waist joints back to 0 rad."""
        allowed = self._allowed(args)
        if allowed.get("ok") is False:
            return allowed
        snap = self._client.snapshot()
        if not snap.get("fresh"):
            return _failure("JOINT_STATE_UNAVAILABLE", "No fresh /joint_states for reset")
        targets = {}
        for name in WAIST_JOINTS:
            v = snap.get("joints", {}).get(name)
            if v is not None:
                targets[name] = 0.0
        if not targets:
            return _failure("WAIST_MODEL_MISMATCH", "No waist joints found in /joint_states")
        if not self._router.acquire(CARD):
            return _failure("COMMAND_IN_PROGRESS", "Another Q5 body card currently owns the command publisher")
        action_id = f"waist_control_reset_{int(time.time()*1000)}"
        # Use the first joint's delta for duration estimation
        first_name = WAIST_JOINTS[0]
        current = float(snap["joints"][first_name])
        duration = max(0.5, abs(current) / (self._max_step * self._rate))
        with self._lock:
            if self._thread is not None and self._thread.is_alive():
                self._router.release(CARD)
                return _failure("MOTION_IN_PROGRESS", "A waist adjustment is already active; call stop first")
            event = threading.Event()
            self._stop_event = event
            self._active = {"joint_name": "all", "start_position_rad": current,
                            "target_position_rad": 0.0, "duration_s": duration,
                            "started_at_ms": int(time.time() * 1000)}
            self._thread = threading.Thread(
                target=self._run_reset, args=(event, targets, snap, duration, action_id),
                daemon=True, name="q5_waist_reset")
            self._thread.start()
        return {"ok": True, "state": "moving", "waist_action": "reset",
                "command": dict(self._active), "action_id": action_id}

    def _run_reset(self, event, targets, snap, duration, action_id):
        """Interpolate all waist joints from current to 0 rad."""
        cancelled = True
        try:
            currents = {}
            for name in WAIST_JOINTS:
                v = snap.get("joints", {}).get(name)
                if v is not None:
                    currents[name] = float(v)
            steps = max(
                max(int(math.ceil(abs(currents[n] - 0.0) / self._max_step)) for n in currents) if currents else 1,
                int(math.ceil(duration * self._rate)),
                1,
            )
            for index in range(1, steps + 1):
                if event.is_set():
                    break
                for name in currents:
                    self._publish(name, currents[name] * (1.0 - index / steps))
                event.wait(duration / steps)
            cancelled = event.is_set()
        finally:
            if not event.is_set():
                for name in targets:
                    self._hold_position(name, 0.0)
            else:
                for name in WAIST_JOINTS:
                    self._hold_current(name)
            self._router.release(CARD)
            with self._lock:
                if self._stop_event is event:
                    self._stop_event = self._thread = self._active = None
            if action_id:
                if cancelled:
                    _acp_notify(action_id, "cancelled", {"joints": list(targets.keys())}, CARD)
                else:
                    _acp_notify(action_id, "completed", {"joints": list(targets.keys()), "target_rad": 0.0}, CARD)

    def get_tool(self):
        position_fields = {}
        for detail in WAIST_ACTIONS.values():
            lower, upper = JOINT_LIMITS[detail["joint_name"]]
            position_fields[detail["field"]] = {
                "type": "number", "title": f"目标角度 (rad)",
                "minimum": lower, "maximum": upper, "multipleOf": 0.005,
                "description": detail["description"],
            }
        return {"name": CARD, "type": TYPE, "multiInstance": False, "description": DESC,
                "inputSchema": {"type": "object", "properties": {
                    "action": {"type": "string", "enum": ["start", *WAIST_ACTIONS, "reset", "cancel", "stop", "info"], "oneOf": [
                        {"const": "start", "title": "检查连接状态"},
                        *[{"const": action, "title": detail["title"], "description": detail["description"]}
                          for action, detail in WAIST_ACTIONS.items()],
                        {"const": "reset", "title": "归零"},
                        {"const": "cancel", "title": "取消并保持"},
                        {"const": "stop", "title": "停止并保持当前位置"},
                        {"const": "info", "title": "查看状态"},
                    ]},
                    **position_fields,
                }, "required": ["action"], "additionalProperties": False,
                "x-action-params": {
                    "start": {"params": [], "description": "检查 ROS 连接和机器人状态。"},
                    **{action: {"params": [detail["field"]], "description": detail["description"]}
                       for action, detail in WAIST_ACTIONS.items()},
                    "reset": {"params": [], "description": "归零：将腰部 yaw 关节插补回 0 rad。"},
                    "cancel": {"params": [], "description": "取消当前微调，并保持当前位置。"},
                    "stop": {"params": [], "description": "停止当前运动并保持当前位置。"},
                    "info": {"params": [], "description": "查看运动状态与安全条件。"},
                },
                "x-completion": {
                    "actions": [*WAIST_ACTIONS.keys(), "reset"],
                    "timeout": 15,
                },
                # Waist yaw. Independent of legs, arms and the speaker.
                "x-resource": "waist",
            }}

    def _safety(self):
        status = self._router.status()
        status.update({"control_mode": "direct_joint_position",
                "command_message": "xbot_common_interfaces/msg/HybridJointCommand",
                "lifecycle_state": self._client.get_lifecycle_state(),
                "joint_state_fresh": bool(self._client.snapshot().get("fresh", False)), "topic": TOPIC,
                "q5_fsm": q5_active_status(self._client),
                "joints": list(WAIST_JOINTS), "joint_names_source": "q5_model.urdf",
                "limits": limits_for(WAIST_JOINTS)})
        return status

    def _allowed(self, args, joint_name=None):
        status = self._safety()
        if not status["ros_publisher_available"]:
            return _failure("ROS_UNAVAILABLE", "Q5 body command publisher is unavailable", status=status)
        if status["lifecycle_state"] != "active" or not status["joint_state_fresh"]:
            return _failure("ROBOT_NOT_READY", "Q5 must be active with fresh /joint_states before waist control", status=status)
        q5_ready, q5_status = q5_is_control_ready(self._client)
        if not q5_ready:
            return _failure("Q5_FSM_NOT_READY", "Q5 /xbot_state must be fresh and READY or ACTIVE before waist control",
                            status={**status, "q5_fsm": q5_status})
        # Auto-prepare: if position control is not prepared, prepare it now.
        prep_error = self._ensure_prepared()
        if prep_error is not None:
            return prep_error
        if joint_name is not None and joint_name not in self._client.snapshot().get("joints", {}):
            return _failure("WAIST_MODEL_MISMATCH", "Configured waist joint is absent from /joint_states", joint_name=joint_name)
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

    def _run(self, event, name, current, target, duration, action_id=None):
        steps = max(int(math.ceil(abs(target - current) / self._max_step)), int(math.ceil(duration * self._rate)), 1)
        cancelled = True
        try:
            for index in range(1, steps + 1):
                if event.is_set():
                    break
                self._publish(name, current + (target - current) * index / steps)
                event.wait(duration / steps)
            cancelled = event.is_set()
        finally:
            self._hold_position(name, target) if not event.is_set() else self._hold_current(name)
            self._router.release(CARD)
            with self._lock:
                if self._stop_event is event:
                    self._stop_event = self._thread = self._active = None
            if action_id:
                if cancelled:
                    _acp_notify(action_id, "cancelled", {"joint_name": name}, CARD)
                else:
                    _acp_notify(action_id, "completed", {"joint_name": name, "target_rad": target}, CARD)

    def _stop(self, reason):
        with self._lock:
            event, thread, active = self._stop_event, self._thread, self._active
        if event is not None:
            event.set()
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=1.0)
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
        if action == "reset":
            return self._reset(args)
        detail = WAIST_ACTIONS.get(action)
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
        duration = max(0.5, abs(target - current) / (self._max_step * self._rate))
        if not self._router.acquire(CARD):
            return _failure("COMMAND_IN_PROGRESS", "Another Q5 body card currently owns the command publisher",
                            status=self._router.status())
        with self._lock:
            if self._thread is not None and self._thread.is_alive():
                self._router.release(CARD)
                return _failure("MOTION_IN_PROGRESS", "A waist adjustment is already active; call stop first")
            event = threading.Event()
            action_id = f"waist_control_{action}_{int(time.time()*1000)}"
            self._stop_event = event
            self._active = {"joint_name": name, "start_position_rad": current, "target_position_rad": target, "duration_s": duration, "started_at_ms": int(time.time() * 1000)}
            self._thread = threading.Thread(target=self._run, args=(event, name, current, target, duration, action_id), daemon=True, name="q5_waist_control")
            self._thread.start()
        return {"ok": True, "state": "moving", "waist_action": action,
                "joint_name": name, "command": dict(self._active),
                "action_id": action_id,
                "stops_by_holding_current_position": True}

    def stop(self):
        self._stop("driver_shutdown")


def make_plugin(plugin_config, namespace, executor, client):
    return Plugin(plugin_config, namespace, executor, client)
