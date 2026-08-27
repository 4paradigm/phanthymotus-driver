"""Q5 leg joint control card — set hip/knee/ankle angles together (degrees) or zero.

Three-joint absolute position control over the same vendor command channel as
arm_control.  ``set`` takes up to three target angles in degrees, each validated
against its URDF joint limits (out-of-range is rejected, never clamped); omitted
joints keep their current angle, so the card supports both single-joint and
three-joint simultaneous motion in one HybridJointCommand.  ``zero`` returns all
three joints to 0°.  Motion runs as incremental interpolation (never a fast
jump) under the shared BodyCommandRouter lease, and the card reports
asynchronous completion to Agent Core via the ACP endpoint so long motions are
not mistaken for instantly-completed actions.
"""

from __future__ import annotations

import json
import math
import os
import ssl
import threading
import time
import urllib.request

from body_command import get_router
from control_contract import q5_active_status, q5_is_control_ready
from joint_limits import JOINT_LIMITS
from rclpy.node import Node
from std_srvs.srv import Trigger

CARD = "leg_control"
TYPE = "actuator"
TOPIC = "/wr1_controller/commands"

# Canonical joint order for the leg: hip -> knee -> ankle (both legs follow).
JOINTS = ("hip_joint", "knee_joint", "ankle_joint")
# Public parameter field -> joint name mapping (fields are kept short for the canvas).
_FIELDS = {"hip_deg": "hip_joint", "knee_deg": "knee_joint", "ankle_deg": "ankle_joint"}
_LABELS = {"hip_joint": "髋", "knee_joint": "膝", "ankle_joint": "踝"}

_LIMITS = {}
for _j in JOINTS:
    _lim = JOINT_LIMITS.get(_j)
    if _lim is None:
        raise RuntimeError(f"{CARD}: joint {_j} missing from URDF joint limits")
    _LIMITS[_j] = _lim

_DEG = {j: (round(_LIMITS[j][0] * 180.0 / math.pi, 1), round(_LIMITS[j][1] * 180.0 / math.pi, 1)) for j in JOINTS}
_RANGE_STR = "、".join(f"{_LABELS[j]}{_DEG[j][0]}°~{_DEG[j][1]}°" for j in JOINTS)
DESC = f"Q5 腿部控制：set 设置髋/膝/踝角度(°)，支持三关节同时动；zero 三关节归零。范围 {_RANGE_STR}"


def _failure(code, message, **details):
    return {"ok": False, "code": code, "message": message, "details": details}


def _number(value, field):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(float(value)):
        raise ValueError(f"{field} must be a finite number")
    return float(value)


def _acp_notify(action_id, status, result, tool=""):
    """POST action completion to Agent Core (module-level ACP helper)."""
    agent_core_url = os.environ.get("AGENT_CORE_URL", "https://localhost:15678")
    ctx = ssl.create_default_context()
    ctx.check_hostname = False
    ctx.verify_mode = ssl.CERT_NONE
    payload = json.dumps({
        "action_id": action_id,
        "status": status,
        "result": result,
        "tool": tool,
        "ts": time.time(),
    }).encode()
    try:
        req = urllib.request.Request(
            f"{agent_core_url}/api/acp/complete",
            data=payload,
            headers={"Content-Type": "application/json"},
            method="POST",
        )
        urllib.request.urlopen(req, timeout=5, context=ctx)
    except Exception:
        pass  # ACP failure must not block joint motion


class Plugin:
    def __init__(self, plugin_config, namespace, executor, client):
        self._client = client
        self._router = get_router(client, executor)
        self._node = Node(f"q5_{CARD}")
        executor.add_node(self._node)
        self._activate_client = None
        self._max_step = float(plugin_config.get("max_step_rad", 0.02))
        self._rate = float(plugin_config.get("publish_rate_hz", 20.0))
        self._hold_repetitions = int(plugin_config.get("hold_repetitions", 3))
        if min(self._max_step, self._rate) <= 0 or self._hold_repetitions < 1:
            raise ValueError(f"{CARD} limits and rate must be positive")
        self._lock = threading.Lock()
        self._stop_event = self._thread = self._active = None

    def get_tool(self):
        props = {
            "action": {"type": "string", "enum": ["set", "zero"], "oneOf": [
                {"const": "set", "title": "设角度"},
                {"const": "zero", "title": "归零"},
            ]},
        }
        for field, joint in _FIELDS.items():
            props[field] = {"type": "number", "title": f"{_LABELS[joint]}关节角度 (°)",
                            "minimum": _DEG[joint][0], "maximum": _DEG[joint][1],
                            "description": f"{_LABELS[joint]}关节目标角度（度），范围 {_DEG[joint][0]}°~{_DEG[joint][1]}°，省略则保持当前"}
        return {"name": CARD, "type": TYPE, "multiInstance": False, "description": DESC,
                "inputSchema": {"type": "object", "properties": props,
                                "required": ["action"], "additionalProperties": False,
                                "x-action-params": {
                                    "set": {"params": list(_FIELDS),
                                            "description": "同时设置腿部三关节角度（度），省略的关节保持当前角度。"},
                                    "zero": {"params": [], "description": "髋/膝/踝三关节归零（回到 0°）。"},
                                },
                                "x-completion": {
                                    "actions": ["set", "zero"],
                                    "timeout": 30,
                                }}}

    @staticmethod
    def _wait_future(future, timeout):
        deadline = time.monotonic() + timeout
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.02)
        return future.result() if future.done() else None

    def _ensure_active(self):
        """Bring Q5 from READY to ACTIVE before motion; None on success else failure dict."""
        fsm = q5_active_status(self._client)
        if fsm.get("state_label") == "ACTIVE":
            return None
        if self._activate_client is None:
            self._activate_client = self._node.create_client(Trigger, "/activate_service")
        if not self._activate_client.wait_for_service(timeout_sec=5.0):
            return _failure("ACTIVATE_UNAVAILABLE", "Q5 /activate_service is unavailable", q5_fsm=fsm)
        future = self._activate_client.call_async(Trigger.Request())
        response = self._wait_future(future, 15.0)
        if response is None or not response.success:
            return _failure("ACTIVATE_FAILED", "Q5 activate_service did not confirm activation",
                            message=getattr(response, "message", "timeout") if response else "timeout")
        # Wait for FSM ACTIVE and fresh /joint_states (feedback resumes after activation).
        for _ in range(20):
            time.sleep(0.5)
            fsm = q5_active_status(self._client)
            fresh = bool(self._client.snapshot().get("fresh", False))
            if fsm.get("state_label") == "ACTIVE" and fresh:
                return None
        return _failure("ACTIVATE_TIMEOUT", "Q5 did not reach ACTIVE with fresh /joint_states",
                        q5_fsm=fsm)

    def _safety(self):
        status = self._router.status()
        status.update({"control_mode": "direct_joint_position",
                "command_message": "xbot_common_interfaces/msg/HybridJointCommand",
                "lifecycle_state": self._client.get_lifecycle_state(),
                "joint_state_fresh": bool(self._client.snapshot().get("fresh", False)), "topic": TOPIC,
                "q5_fsm": q5_active_status(self._client),
                "joints": list(JOINTS),
                "limits_deg": {j: {"min_deg": _DEG[j][0], "max_deg": _DEG[j][1]} for j in JOINTS}})
        return status

    def _allowed(self, targets):
        status = self._safety()
        if not status["ros_publisher_available"]:
            return _failure("ROS_UNAVAILABLE", "Q5 body command publisher is unavailable", status=status)
        if status["lifecycle_state"] != "active" or not status["joint_state_fresh"]:
            return _failure("ROBOT_NOT_READY", "Q5 must be active with fresh /joint_states before leg control",
                            status=status)
        q5_ready, q5_status = q5_is_control_ready(self._client)
        if not q5_ready:
            return _failure("Q5_FSM_NOT_READY", "Q5 /xbot_state must be fresh and READY or ACTIVE before leg control",
                            status={**status, "q5_fsm": q5_status})
        snap_joints = self._client.snapshot().get("joints", {})
        for joint, target in targets.items():
            if joint not in snap_joints:
                return _failure("JOINT_MODEL_MISMATCH", f"{joint} absent from /joint_states", joint_name=joint)
            if target < _LIMITS[joint][0] or target > _LIMITS[joint][1]:
                return _failure("LIMIT_EXCEEDED", f"{joint} target angle out of range",
                                joint_name=joint, min_deg=_DEG[joint][0], max_deg=_DEG[joint][1],
                                target_deg=round(target * 180.0 / math.pi, 1))
        return {"ok": True, "status": status}

    def _publish(self, positions):
        # One HybridJointCommand carrying all commanded joints at once.
        return self._router.publish(positions)

    def _hold_position(self, positions):
        published = False
        for _ in range(self._hold_repetitions):
            published = self._publish(positions) or published
            time.sleep(1.0 / self._rate)
        return published

    def _hold_current(self):
        snap = self._client.snapshot()
        if not snap.get("fresh"):
            return False
        positions = {}
        for joint in JOINTS:
            value = snap.get("joints", {}).get(joint)
            if value is None:
                return False
            positions[joint] = float(value)
        return self._hold_position(positions)

    def _run(self, event, current, targets, duration, action_id, targets_deg):
        steps = max(max(int(math.ceil(abs(targets[j] - current[j]) / self._max_step)) for j in JOINTS),
                    int(math.ceil(duration * self._rate)), 1)
        try:
            for index in range(1, steps + 1):
                if event.is_set():
                    break
                self._publish({j: current[j] + (targets[j] - current[j]) * index / steps for j in JOINTS})
                event.wait(duration / steps)
        except Exception:
            pass
        finally:
            # Joint feedback can lag the last command; reuse the final targets
            # instead of sending the joints back to their start angles.
            if not event.is_set():
                self._hold_position(targets)
            else:
                self._hold_current()
            if action_id:
                if event.is_set():
                    _acp_notify(action_id, "cancelled", {"joints": list(JOINTS)}, CARD)
                else:
                    _acp_notify(action_id, "completed",
                                {"joints": list(JOINTS), "target_positions_deg": targets_deg}, CARD)
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

    def _launch(self, targets, action_name, targets_deg, action_id):
        # READY -> ACTIVE automatically before motion (joint feedback only
        # exists in ACTIVE; without it the incremental move cannot start).
        activate_error = self._ensure_active()
        if activate_error is not None:
            return activate_error
        # Fill omitted joints with their current angles so the interpolation
        # always drives the full leg without moving unspecified joints.
        snap = self._client.snapshot()
        current = {}
        for joint in JOINTS:
            if joint in targets:
                current[joint] = float(snap["joints"][joint])
            else:
                current[joint] = targets[joint] = float(snap["joints"][joint])
        allowed = self._allowed(targets)
        if allowed.get("ok") is False:
            return allowed
        duration = max(0.5, max(abs(targets[j] - current[j]) for j in JOINTS) / (self._max_step * self._rate))
        if not self._router.acquire(CARD):
            return _failure("COMMAND_IN_PROGRESS", "Another Q5 body card currently owns the command publisher",
                            status=self._router.status())
        with self._lock:
            if self._thread is not None and self._thread.is_alive():
                self._router.release(CARD)
                return _failure("MOTION_IN_PROGRESS", "A leg move is already active; call stop first")
            event = threading.Event()
            self._stop_event = event
            self._active = {"action": action_name, "joints": list(JOINTS), "action_id": action_id,
                            "start_positions_deg": {j: round(current[j] * 180.0 / math.pi, 1) for j in JOINTS},
                            "target_positions_deg": targets_deg,
                            "duration_s": duration, "started_at_ms": int(time.time() * 1000)}
            self._thread = threading.Thread(target=self._run, args=(event, current, targets, duration,
                                                                    action_id, targets_deg),
                                            daemon=True, name=f"q5_{CARD}")
            self._thread.start()
        return {"ok": True, "state": "moving", "joints": list(JOINTS),
                "target_positions_deg": targets_deg, "action_id": action_id,
                "command": dict(self._active),
                "stops_by_holding_current_position": True}

    def dispatch(self, action, args):
        if action in ("start", "info"):
            # Hidden lifecycle actions: not exposed in the enum (canvas shows
            # only set/zero) but required by the canvas startup sequence.
            return {"state": "ready" if self._router.status()["ros_publisher_available"] else "unavailable",
                    "safety": self._safety()}
        if action == "set":
            targets = {}
            targets_deg = {}
            for field, joint in _FIELDS.items():
                if args.get(field) is not None:
                    try:
                        deg = _number(args.get(field), field)
                    except ValueError as e:
                        return _failure("INVALID_ARGUMENT", str(e))
                    targets[joint] = math.radians(deg)
                    targets_deg[joint] = round(deg, 1)
            if not targets:
                return _failure("INVALID_ARGUMENT", "at least one of hip_deg/knee_deg/ankle_deg is required")
            return self._launch(targets, "set", targets_deg, f"{CARD}_set_{int(time.time() * 1000)}")
        if action == "zero":
            targets = {j: 0.0 for j in JOINTS}
            targets_deg = {j: 0.0 for j in JOINTS}
            return self._launch(targets, "zero", targets_deg, f"{CARD}_zero_{int(time.time() * 1000)}")
        if action in ("cancel", "stop"):
            return self._stop("command")
        return None

    def stop(self):
        self._stop("driver_shutdown")


def make_plugin(plugin_config, namespace, executor, client):
    return Plugin(plugin_config, namespace, executor, client)
