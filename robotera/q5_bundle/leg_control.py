"""Q5 leg control card — wheel-legged joints (hip/knee/ankle) + leg wheel arms.

Direct absolute position control of the leg joints and the drv_hang (wheel
arm) joints over the same vendor command channel as waist/arm_control.
Safe semantic presets plus a whitelisted single-joint ``set`` action.  All
targets are validated against the URDF joint limits and executed as small
incremental interpolation steps (never a fast jump); cancel keeps the robot
at the last commanded position.
"""

from __future__ import annotations

import math
import threading
import time

from body_command import get_router
from control_contract import q5_active_status, q5_is_control_ready
from joint_limits import JOINT_LIMITS, limits_for

CARD = "leg_control"
TYPE = "actuator"
TOPIC = "/wr1_controller/commands"

# Wheel-legged joints present in the bundled URDF (resource/q5_model.urdf).
LEG_JOINTS = ("hip_joint", "knee_joint", "ankle_joint",
              "left_drv_hang_joint", "right_drv_hang_joint")

# Safe semantic presets (rad).  Values stay inside the URDF limits; presets
# are deliberately conservative for the wheel-legged balance model.
LEG_PRESETS = {
    "stand": {
        "hip_joint": 0.0, "knee_joint": 0.0, "ankle_joint": 0.0,
        "left_drv_hang_joint": 0.0, "right_drv_hang_joint": 0.0,
    },
    "knee_bend": {"knee_joint": -0.30},
    "hang_up": {"left_drv_hang_joint": 0.30, "right_drv_hang_joint": 0.30},
    "hang_down": {"left_drv_hang_joint": -0.30, "right_drv_hang_joint": -0.30},
}
PRESET_LABELS = {
    "stand": "站立位（各关节回 URDF 零位）",
    "knee_bend": "微蹲（膝微屈 -0.30 rad）",
    "hang_up": "轮腿摆臂抬升（+0.30 rad）",
    "hang_down": "轮腿摆臂下降（-0.30 rad）",
}
DESC = "Q5 腿部控制：髋/膝/踝 + 轮腿摆臂关节（预设姿态 + 白名单关节自由控制）"


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
        self._max_step = float(plugin_config.get("max_step_rad", 0.02))
        self._rate = float(plugin_config.get("publish_rate_hz", 20.0))
        self._hold_repetitions = int(plugin_config.get("hold_repetitions", 3))
        if min(self._max_step, self._rate) <= 0 or self._hold_repetitions < 1:
            raise ValueError("leg_control limits and rate must be positive")
        self._lock = threading.Lock()
        self._stop_event = self._thread = self._active = None

    def get_tool(self):
        return {"name": CARD, "type": TYPE, "multiInstance": False, "description": DESC,
                "inputSchema": {"type": "object", "properties": {
                    "action": {"type": "string", "enum": ["start", *LEG_PRESETS, "set", "cancel", "info"], "oneOf": [
                        {"const": "start", "title": "检查连接状态"},
                        *[{"const": name, "title": label} for name, label in PRESET_LABELS.items()],
                        {"const": "set", "title": "自由控制"},
                        {"const": "cancel", "title": "取消并保持"},
                        {"const": "info", "title": "查看状态"},
                    ]},
                    "joint_name": {"type": "string", "title": "关节名", "enum": list(LEG_JOINTS), "oneOf": [
                        *[{"const": n, "title": n} for n in LEG_JOINTS],
                    ]},
                    "position_rad": {"type": "number", "title": "目标角度 (rad)", "multipleOf": 0.005,
                                     "description": "目标绝对角度，须在关节限位内"},
                }, "required": ["action"], "additionalProperties": False,
                "x-action-params": {
                    "start": {"params": [], "description": "检查 ROS 连接和机器人状态。"},
                    **{name: {"params": [], "description": label} for name, label in PRESET_LABELS.items()},
                    "set": {"params": ["joint_name", "position_rad"],
                            "description": "对白名单关节执行增量位置控制（限位内）。"},
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
                "joints": list(LEG_JOINTS), "joint_names_source": "q5_model.urdf",
                "limits": limits_for(LEG_JOINTS)})
        return status

    def _allowed(self, targets):
        status = self._safety()
        if not status["ros_publisher_available"]:
            return _failure("ROS_UNAVAILABLE", "Q5 body command publisher is unavailable", status=status)
        if status["lifecycle_state"] != "active" or not status["joint_state_fresh"]:
            return _failure("ROBOT_NOT_READY", "Q5 must be active with fresh /joint_states before leg control", status=status)
        q5_ready, q5_status = q5_is_control_ready(self._client)
        if not q5_ready:
            return _failure("Q5_FSM_NOT_READY", "Q5 /xbot_state must be fresh and READY or ACTIVE before leg control",
                            status={**status, "q5_fsm": q5_status})
        missing = [n for n in targets if n not in self._client.snapshot().get("joints", {})]
        if missing:
            return _failure("LEG_MODEL_MISMATCH", "Configured leg joint absent from /joint_states", missing=missing)
        bad = []
        for n, target in targets.items():
            lower, upper = JOINT_LIMITS.get(n, (None, None))
            if lower is None or target < lower or target > upper:
                bad.append({"joint_name": n, "min_rad": lower, "max_rad": upper, "target_position_rad": target})
        if bad:
            return _failure("LIMIT_EXCEEDED", "target outside the joint safety limits", violations=bad)
        return {"ok": True, "status": status}

    def _publish(self, positions):
        return self._router.publish(positions)

    def _hold_position(self, positions):
        published = False
        for _ in range(self._hold_repetitions):
            published = self._publish(positions) or published
            time.sleep(1.0 / self._rate)
        return published

    def _hold_current(self, names):
        snap = self._client.snapshot()
        if not snap.get("fresh"):
            return False
        joints = snap.get("joints", {})
        return self._hold_position({n: joints[n] for n in names if n in joints})

    def _run(self, event, targets, duration):
        snap = self._client.snapshot()
        joints = snap.get("joints", {})
        currents = {n: float(joints[n]) for n in targets if n in joints}
        max_delta = max((abs(targets[n] - currents[n]) for n in currents), default=0.0)
        steps = max(int(math.ceil(max_delta / self._max_step)), int(math.ceil(duration * self._rate)), 1)
        try:
            for index in range(1, steps + 1):
                if event.is_set():
                    break
                frac = index / steps
                self._publish({n: currents[n] + (targets[n] - currents[n]) * frac for n in currents})
                event.wait(duration / steps)
        finally:
            # Joint feedback can lag the last command; reuse the final target
            # instead of sending the joints back to their start angles.
            if not event.is_set():
                self._hold_position(targets)
            else:
                self._hold_current(list(targets))
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
        return {"ok": True, "state": "stopped", "reason": reason,
                "hold_command_attempted": bool(active)}

    def _launch(self, targets, action_name):
        allowed = self._allowed(targets)
        if allowed.get("ok") is False:
            return allowed
        snap = self._client.snapshot()
        joints = snap.get("joints", {})
        max_delta = max((abs(targets[n] - float(joints[n])) for n in targets if n in joints), default=0.0)
        duration = max(0.5, max_delta / (self._max_step * self._rate))
        if not self._router.acquire(CARD):
            return _failure("COMMAND_IN_PROGRESS", "Another Q5 body card currently owns the command publisher",
                            status=self._router.status())
        with self._lock:
            if self._thread is not None and self._thread.is_alive():
                self._router.release(CARD)
                return _failure("MOTION_IN_PROGRESS", "A leg move is already active; call stop first")
            event = threading.Event()
            self._stop_event = event
            self._active = {"action": action_name, "targets": targets, "duration_s": duration,
                            "started_at_ms": int(time.time() * 1000)}
            self._thread = threading.Thread(target=self._run, args=(event, targets, duration),
                                            daemon=True, name="q5_leg_control")
            self._thread.start()
        return {"ok": True, "state": "moving", "leg_action": action_name,
                "command": dict(self._active), "stops_by_holding_current_position": True}

    def dispatch(self, action, args):
        if action == "start":
            return {"state": "ready" if self._router.status()["ros_publisher_available"] else "unavailable",
                    "safety": self._safety()}
        if action == "info":
            with self._lock:
                active = dict(self._active) if self._active else None
            return {"ok": True, "state": "moving" if active else "idle",
                    "active_command": active, "safety": self._safety()}
        if action in ("cancel", "stop"):
            return self._stop("command")
        if action in LEG_PRESETS:
            targets = dict(LEG_PRESETS[action])
            violations = []
            for n, t in targets.items():
                lower, upper = JOINT_LIMITS.get(n, (None, None))
                if lower is None or t < lower or t > upper:
                    violations.append(n)
            if violations:
                return _failure("PRESET_OUT_OF_LIMITS", "preset target exceeds URDF limits", joints=violations)
            return self._launch(targets, action)
        if action == "set":
            name = args.get("joint_name", "")
            if name not in LEG_JOINTS:
                return _failure("INVALID_JOINT", "joint_name must be a whitelisted leg joint",
                                whitelist=list(LEG_JOINTS))
            try:
                target = _number(args.get("position_rad"), "position_rad")
            except ValueError as e:
                return _failure("INVALID_ARGUMENT", str(e))
            return self._launch({name: target}, "set")
        return None

    def stop(self):
        self._stop("driver_shutdown")


def make_plugin(plugin_config, namespace, executor, client):
    return Plugin(plugin_config, namespace, executor, client)
