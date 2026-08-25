"""Q5 hip joint control card — set angle (degrees) or zero.

Single-joint absolute position control over the same vendor command channel
as arm_control.  ``set`` takes a target angle in degrees and is validated
against the URDF joint limits (out-of-range is rejected, never clamped);
``zero`` returns the joint to 0°.  Motion runs as incremental interpolation
(never a fast jump) under the shared BodyCommandRouter lease, and the card
reports asynchronous completion to Agent Core via the ACP endpoint so long
motions are not mistaken for instantly-completed actions.
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

CARD = "ankle_control"
TYPE = "actuator"
TOPIC = "/wr1_controller/commands"
JOINT = "ankle_joint"

_LIMITS = JOINT_LIMITS.get(JOINT)
if _LIMITS is None:
    raise RuntimeError(f"{CARD}: joint {JOINT} missing from URDF joint limits")
LOWER_DEG = round(_LIMITS[0] * 180.0 / math.pi, 1)
UPPER_DEG = round(_LIMITS[1] * 180.0 / math.pi, 1)
DESC = f"Q5 踝关节控制：set 设角度(°)，zero 归零。范围 {LOWER_DEG}° ~ {UPPER_DEG}°"


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
        return {"name": CARD, "type": TYPE, "multiInstance": False, "description": DESC,
                "inputSchema": {"type": "object", "properties": {
                    "action": {"type": "string", "enum": ["set", "zero"], "oneOf": [
                        {"const": "set", "title": "设角度"},
                        {"const": "zero", "title": "归零"},
                    ]},
                    "position_deg": {"type": "number", "title": "目标角度 (°)",
                                     "minimum": LOWER_DEG, "maximum": UPPER_DEG,
                                     "description": f"目标角度（度），范围 {LOWER_DEG}° ~ {UPPER_DEG}°"},
                }, "required": ["action"], "additionalProperties": False,
                "x-action-params": {
                    "set": {"params": ["position_deg"],
                            "description": f"设置踝关节角度（度），须在 {LOWER_DEG}°~{UPPER_DEG}° 范围内。"},
                    "zero": {"params": [], "description": "踝关节归零（回到 0°）。"},
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
                "joint": JOINT,
                "limit_deg": {"min_deg": LOWER_DEG, "max_deg": UPPER_DEG}})
        return status

    def _allowed(self, target_rad):
        status = self._safety()
        if not status["ros_publisher_available"]:
            return _failure("ROS_UNAVAILABLE", "Q5 body command publisher is unavailable", status=status)
        if status["lifecycle_state"] != "active" or not status["joint_state_fresh"]:
            return _failure("ROBOT_NOT_READY", "Q5 must be active with fresh /joint_states before ankle control",
                            status=status)
        q5_ready, q5_status = q5_is_control_ready(self._client)
        if not q5_ready:
            return _failure("Q5_FSM_NOT_READY", "Q5 /xbot_state must be fresh and READY or ACTIVE before ankle control",
                            status={**status, "q5_fsm": q5_status})
        if JOINT not in self._client.snapshot().get("joints", {}):
            return _failure("JOINT_MODEL_MISMATCH", "ankle_joint absent from /joint_states", joint_name=JOINT)
        if target_rad < _LIMITS[0] or target_rad > _LIMITS[1]:
            return _failure("LIMIT_EXCEEDED", f"target angle out of range {LOWER_DEG}°~{UPPER_DEG}°",
                            joint_name=JOINT, min_deg=LOWER_DEG, max_deg=UPPER_DEG,
                            target_deg=round(target_rad * 180.0 / math.pi, 1))
        return {"ok": True, "status": status}

    def _publish(self, position):
        return self._router.publish({JOINT: position})

    def _hold_position(self, position):
        published = False
        for _ in range(self._hold_repetitions):
            published = self._publish(position) or published
            time.sleep(1.0 / self._rate)
        return published

    def _hold_current(self):
        snap = self._client.snapshot()
        if not snap.get("fresh"):
            return False
        value = snap.get("joints", {}).get(JOINT)
        if value is None:
            return False
        return self._hold_position(float(value))

    def _run(self, event, current, target, duration, action_id, target_deg):
        steps = max(int(math.ceil(abs(target - current) / self._max_step)), int(math.ceil(duration * self._rate)), 1)
        try:
            for index in range(1, steps + 1):
                if event.is_set():
                    break
                self._publish(current + (target - current) * index / steps)
                event.wait(duration / steps)
        except Exception:
            pass
        finally:
            # Joint feedback can lag the last command; reuse the final target
            # instead of sending the joint back to its start angle.
            if not event.is_set():
                self._hold_position(target)
            else:
                self._hold_current()
            if action_id:
                if event.is_set():
                    _acp_notify(action_id, "cancelled", {"joint": JOINT}, CARD)
                else:
                    _acp_notify(action_id, "completed",
                                {"joint": JOINT, "target_position_deg": target_deg}, CARD)
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

    def _launch(self, target_rad, action_name, target_deg, action_id):
        # READY -> ACTIVE automatically before motion (joint feedback only
        # exists in ACTIVE; without it the incremental move cannot start).
        activate_error = self._ensure_active()
        if activate_error is not None:
            return activate_error
        allowed = self._allowed(target_rad)
        if allowed.get("ok") is False:
            return allowed
        snap = self._client.snapshot()
        current = float(snap["joints"][JOINT])
        duration = max(0.5, abs(target_rad - current) / (self._max_step * self._rate))
        if not self._router.acquire(CARD):
            return _failure("COMMAND_IN_PROGRESS", "Another Q5 body card currently owns the command publisher",
                            status=self._router.status())
        with self._lock:
            if self._thread is not None and self._thread.is_alive():
                self._router.release(CARD)
                return _failure("MOTION_IN_PROGRESS", "A ankle move is already active; call stop first")
            event = threading.Event()
            self._stop_event = event
            self._active = {"action": action_name, "joint": JOINT, "action_id": action_id,
                            "start_position_deg": round(current * 180.0 / math.pi, 1),
                            "target_position_deg": target_deg,
                            "duration_s": duration, "started_at_ms": int(time.time() * 1000)}
            self._thread = threading.Thread(target=self._run, args=(event, current, target_rad, duration,
                                                                    action_id, target_deg),
                                            daemon=True, name=f"q5_{CARD}")
            self._thread.start()
        return {"ok": True, "state": "moving", "joint": JOINT,
                "target_position_deg": target_deg, "action_id": action_id,
                "command": dict(self._active),
                "stops_by_holding_current_position": True}

    def dispatch(self, action, args):
        if action in ("start", "info"):
            # Hidden lifecycle actions: not exposed in the enum (canvas shows
            # only set/zero) but required by the canvas startup sequence.
            return {"state": "ready" if self._router.status()["ros_publisher_available"] else "unavailable",
                    "safety": self._safety()}
        if action == "set":
            try:
                deg = _number(args.get("position_deg"), "position_deg")
            except ValueError as e:
                return _failure("INVALID_ARGUMENT", str(e))
            return self._launch(math.radians(deg), "set", round(deg, 1),
                                f"{CARD}_set_{int(time.time() * 1000)}")
        if action == "zero":
            return self._launch(0.0, "zero", 0.0, f"{CARD}_zero_{int(time.time() * 1000)}")
        if action in ("cancel", "stop"):
            return self._stop("command")
        return None

    def stop(self):
        self._stop("driver_shutdown")


def make_plugin(plugin_config, namespace, executor, client):
    return Plugin(plugin_config, namespace, executor, client)
