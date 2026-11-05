# -*- coding: utf-8 -*-
# arm_gesture - Q5 semantic arm gestures (composes arm_control low-level joint positioning)

from __future__ import annotations

import math
import threading
import time
import json
import urllib.request
import ssl
import os

try:
    from arm_control import ArmControlPlugin
except ImportError:
    ArmControlPlugin = None

from body_command import get_router as _get_body_router, BodyCommandRouter
from control_contract import q5_active_status, q5_is_control_ready
from joint_limits import JOINT_LIMITS, limits_for

_Q5_ARM_JOINTS = (
    "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_arm_yaw_joint",
    "left_elbow_pitch_joint", "left_elbow_yaw_joint", "left_wrist_pitch_joint",
    "left_wrist_roll_joint", "right_shoulder_pitch_joint", "right_shoulder_roll_joint",
    "right_arm_yaw_joint", "right_elbow_pitch_joint", "right_elbow_yaw_joint",
    "right_wrist_pitch_joint", "right_wrist_roll_joint",
)

CARD = "arm_gesture"
TYPE = "actuator"
TOPIC = "/{ns}/q5/arm_gesture"
FMT = "data/json"
HZ = 2.0
NODE = "q5_arm_gesture"
DESC = "Q5 semantic arm gestures: salute, welcome, raise, shake hands, high five, arms out with pose holding, and reset; driven by arm_control absolute joint interpolation."

# ── Joint definitions ────────────────────────────────────────────────────────

ARM_JOINTS_LEFT = (
    "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_arm_yaw_joint",
    "left_elbow_pitch_joint", "left_elbow_yaw_joint", "left_wrist_pitch_joint",
    "left_wrist_roll_joint",
)
ARM_JOINTS_RIGHT = (
    "right_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_arm_yaw_joint",
    "right_elbow_pitch_joint", "right_elbow_yaw_joint", "right_wrist_pitch_joint",
    "right_wrist_roll_joint",
)
ARM_JOINT_NAMES = ARM_JOINTS_LEFT + ARM_JOINTS_RIGHT

# Mirrored index: shoulder_roll(1), arm_yaw(2), elbow_yaw(4), wrist_roll(6) flip sign
_MIRROR_MASK = [1, 2, 4, 6]

# Gestures that stay at their target instead of returning to neutral.
_HOLD_GESTURES = {"arms_out"}

# ── Gesture definitions (degrees) ────────────────────────────────────────────

_GESTURES_DEG = {
    "salute":          [10, 90, 60, -110, 50, 0, 0],
    "welcome":         [10, 65, 75, -100, 0, 0, 0],
    "raise":           [-10, 95, 0, -15, 0, 0, 0],
    "shake_hands":     [-45, -10, -15, -20, 0, 0, 0],
    "high_five":       [-40, 40, -20, -80, 0, 0, 50],
    # Horizontal arms: shoulder roll +90 degrees on the left, mirrored on the right,
    # with straight elbows. Hold the target pose; use reset to lower the arms.
    "arms_out":        [0, 90, 0, 0, 0, 0, 0],
}

# ── Helpers ──────────────────────────────────────────────────────────────────

_GESTURE_LABELS = {
    "salute": "Salute", "welcome": "Welcome", "raise": "Raise arm",
    "shake_hands": "Shake hands", "high_five": "High five", "arms_out": "Arms out (hold)",
}

# Neutral pose: shoulder_pitch -30° (~-0.52 rad) so arms rest naturally in
# front of body. Positive shoulder_pitch rotates arms backward on Q5.
_NEUTRAL_DEG = [-30, 0, 0, 0, 0, 0, 0]


def _deg2rad(deg: float) -> float:
    return deg * math.pi / 180.0


def _rad2deg(rad: float) -> float:
    return rad * 180.0 / math.pi


def _mirror_deg(pose: list[float]) -> list[float]:
    """Return a right-arm mirror of a left-arm degree pose."""
    mirrored = list(pose)
    for i in _MIRROR_MASK:
        mirrored[i] = -mirrored[i]
    return mirrored


def _gesture_to_positions(gesture: str, side: str, deg_pose: list[float]) -> dict:
    """Map a 7-element degree list to {joint_name: position_rad} for *side*."""
    if side == "right":
        deg_list = _mirror_deg(deg_pose)
        joints = ARM_JOINTS_RIGHT
    else:
        deg_list = deg_pose
        joints = ARM_JOINTS_LEFT
    return {joints[i]: _deg2rad(deg_list[i]) for i in range(7)}


def _mirrored_positions(gesture: str, side: str, deg_pose: list[float]) -> dict:
    """Build a side-aware {joint_name: rad} dict, mirroring for right arm."""
    result = {}
    if side in ("left", "both"):
        result.update(_gesture_to_positions(gesture, "left", deg_pose))
    if side in ("right", "both"):
        result.update(_gesture_to_positions(gesture, "right", deg_pose))
    return result


def _clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


def _failure(code: str, message: str, **details) -> dict:
    return {"ok": False, "code": code, "message": message, "details": details}


def _acp_notify(action_id: str, status: str, result: dict, tool: str = ""):
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
        pass  # ACP failure must not block gesture execution


# ── Frame builder ────────────────────────────────────────────────────────────

def _build_frames(gesture: str, cycles: int) -> list:
    """Return a list of (pose_deg, hold_seconds, transition_ratio) tuples.

    transition_ratio scales the interpolation time for each frame:
      < 1.0 → faster transition, creates natural blend with next frame
      == 1.0 → full-speed transition with explicit hold

    No separate PREPARE frame — the gesture target is reached in a single
    smooth interpolation from the current pose, eliminating the "two-step"
    feel reported in review.
    """
    frames: list[tuple[list[float], float, float]] = []

    target = _GESTURES_DEG[gesture]

    # Hold gestures stop at the target: no return-to-neutral frame.
    if gesture in _HOLD_GESTURES:
        return [(target, 0.3, 1.0)]

    # Single smooth transition to the gesture target
    if gesture == "salute":
        frames.append((target, 0.4, 0.85))
    else:
        frames.append((target, 0.3, 0.80))

    if gesture == "shake_hands":
        for i in range(cycles * 2):
            pose = list(target)
            pose[3] = -13 if i % 2 == 0 else -27
            frames.append((pose, 0.0, 0.65))
    elif gesture == "welcome":
        for i in range(cycles * 2):
            pose = list(target)
            pose[3] = -110 if i % 2 == 0 else -90
            frames.append((pose, 0.0, 0.65))

    # Return to neutral: full-speed transition with hold
    frames.append((_NEUTRAL_DEG, 1.0, 1.0))
    return frames


# ── Pose validation ─────────────────────────────────────────────────────────

def _validate_pose_rad(positions: dict) -> list[str]:
    """Check *positions* against URDF-derived JOINT_LIMITS. Returns violation list."""
    violations = []
    for name, rad in positions.items():
        lim = JOINT_LIMITS.get(name)
        if lim is None:
            continue  # non-arm joint, skip
        lo, hi = lim
        if rad < lo - 1e-6:
            violations.append(f"{name}: {rad:.4f} < {lo:.4f}")
        if rad > hi + 1e-6:
            violations.append(f"{name}: {rad:.4f} > {hi:.4f}")
    return violations


# ── Plugin ───────────────────────────────────────────────────────────────────

class Plugin:
    """Semantic arm-gesture actuator built on arm_control / BodyCommandRouter."""

    def __init__(self, plugin_config: dict, namespace: str, executor, client):
        self._client = client
        self._namespace = namespace

        # Delegate low-level joint control to arm_control when available.
        self._arm_control: ArmControlPlugin | None = None
        if ArmControlPlugin is not None:
            ctrl_cfg = dict(plugin_config)
            ctrl_cfg.setdefault("max_step_rad", 0.010)
            ctrl_cfg.setdefault("publish_rate_hz", 3.33)
            ctrl_cfg.setdefault("hold_repetitions", 3)
            try:
                self._arm_control = ArmControlPlugin(ctrl_cfg, namespace, executor, client)
            except Exception:
                self._arm_control = None

        # Shared body publisher (single-router pattern).
        self._router: BodyCommandRouter = _get_body_router(client, executor)

        # Motion parameters.
        self._max_step = float(plugin_config.get("max_step_rad", 0.010))
        self._publish_rate = float(plugin_config.get("publish_rate_hz", 3.33))
        self._hold_repetitions = int(plugin_config.get("hold_repetitions", 3))
        self._settle_tolerance = float(plugin_config.get("settle_tolerance_rad", 0.02))

        # Thread-safe state.
        self._lock = threading.Lock()
        self._stop_event: threading.Event | None = None
        self._motion_thread: threading.Thread | None = None
        self._status: dict = {"state": "idle", "gesture": None, "updated_at_ms": int(time.time() * 1000)}

    # ── Tool definition ──────────────────────────────────────────────────

    def get_tool(self) -> dict:
        actions = ["salute", "welcome", "raise", "shake_hands", "high_five", "arms_out", "reset",
                   "cancel", "stop", "start", "info"]
        one_of_actions = [
            {"const": "start", "title": "Check connection"},
            *[{"const": name, "title": label} for name, label in _GESTURE_LABELS.items()],
            {"const": "reset", "title": "Reset"},
            {"const": "cancel", "title": "Cancel and hold"},
            {"const": "stop", "title": "Stop"},
            {"const": "info", "title": "View status"},
        ]
        return {
            "name": CARD,
            "type": TYPE,
            "multiInstance": False,
            "description": DESC,
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {"type": "string", "enum": actions, "oneOf": one_of_actions},
                    "side": {
                        "type": "string", "title": "Arm side", "enum": ["left", "right", "both"],
                        "default": "right",
                        "oneOf": [
                            {"const": "left", "title": "Left arm"},
                            {"const": "right", "title": "Right arm"},
                            {"const": "both", "title": "Both arms"},
                        ],
                        "description": "Select the arm or arms to move.",
                    },
                    "salute_side": {
                        "type": "string", "title": "Salute side", "enum": ["left", "right"],
                        "default": "right",
                        "oneOf": [
                            {"const": "left", "title": "Left-arm salute"},
                            {"const": "right", "title": "Right-arm salute"},
                        ],
                        "description": "Salute supports one arm at a time; defaults to the right arm.",
                    },
                    "cycles": {
                        "type": "integer", "title": "Cycles",
                        "minimum": 1, "maximum": 5, "default": 2,
                        "description": "Number of shake-hands or welcome oscillation cycles, from 1 to 5.",
                    },
                    "speed": {
                        "type": "number", "title": "Joint speed (rad/s)",
                        "minimum": 0.2, "maximum": 1.5, "default": 0.8,
                        "description": "Joint interpolation speed in rad/s, from 0.2 to 1.5; defaults to 0.8.",
                    },
                },
                "required": ["action"],
                "additionalProperties": False,
                "x-action-params": {
                    "salute": {"params": ["salute_side", "speed"], "description": "Raise the forearm toward the forehead, pause, then return to neutral."},
                    "welcome": {"params": ["side", "cycles", "speed"], "description": "Raise a hand beside the body, wave, then return to neutral."},
                    "raise": {"params": ["side", "speed"], "description": "Raise the arm above the head, then return to neutral."},
                    "shake_hands": {"params": ["side", "cycles", "speed"], "description": "Extend a hand forward, gently oscillate it for a handshake, then return to neutral."},
                    "high_five": {"params": ["side", "speed"], "description": "Extend a hand in front of the body near shoulder height for a high five, then return to neutral."},
                    "arms_out": {"params": ["side", "speed"], "description": "Extend the arm or arms sideways to horizontal and hold; use reset to lower them."},
                    "reset": {"params": ["speed"], "description": "Cancel the sequence and return both arms to neutral."},
                    "cancel": {"params": [], "description": "Cancel remaining motion frames and hold the current position."},
                    "stop": {"params": [], "description": "Stop the current gesture and hold the current position."},
                    "start": {"params": [], "description": "Check ROS connectivity and robot status."},
                    "info": {"params": [], "description": "View the current motion status and safety conditions."},
                },
                "x-completion": {
                    "actions": ["salute", "welcome", "raise", "shake_hands", "high_five", "arms_out", "reset"],
                    "timeout": 60,
                },
                # Both arms, deliberately. `side` picks left/right/both per call, but
                # x-resource is static per schema, so claiming only one would let a
                # `side="both"` gesture run concurrently with a single-arm one on the
                # same joints. Over-serialising left against right is the cheap
                # mistake here; the other direction moves real hardware twice.
                "x-resource": ["arm_l", "arm_r"],
            },
        }

    # ── Lifecycle ─────────────────────────────────────────────────────────

    def start(self) -> dict:
        if self._arm_control is not None:
            return self._arm_control.start()
        return {"state": "ready" if self._router is not None else "unavailable"}

    def stop(self) -> dict:
        self._stop("driver_shutdown")
        if self._arm_control is not None:
            self._arm_control.stop()
        return {"state": "idle"}

    # ── Safety ────────────────────────────────────────────────────────────

    def _safety(self) -> dict:
        router_status = self._router.status()
        status = {
            "ros_publisher_available": router_status["ros_publisher_available"],
            "other_publishers": router_status["other_publishers"],
            "same_name_publisher_count": router_status.get("same_name_publisher_count", 0),
            "lifecycle_state": self._client.get_lifecycle_state(),
            "joint_state_fresh": bool(self._client.snapshot().get("fresh", False)),
            "q5_fsm": q5_active_status(self._client),
            "position_control_prepared": bool(getattr(self._client, "q5_position_control_prepared", False)),
            "limits": {"max_step_rad": self._max_step,
                        "publish_rate_hz": self._publish_rate,
                        "hold_repetitions": self._hold_repetitions},
        }
        if self._arm_control is not None:
            arm_safety = self._arm_control._safety()
            status.update(arm_safety)
        return status

    def _validate_run(self, gesture: str, side: str) -> dict | None:
        """Pre-flight checks. Returns None on success or an error dict.
        If gesture is empty string, skip gesture-name validation but still
        check side, publisher, lifecycle, and Q5 FSM state."""
        if gesture and gesture not in _GESTURES_DEG:
            return _failure("UNKNOWN_GESTURE", f"Unknown gesture: {gesture}")
        if side not in ("left", "right", "both"):
            return _failure("INVALID_ARGUMENT", "side must be left, right, or both")

        status = self._safety()

        if not status["ros_publisher_available"]:
            return _failure("ROS_UNAVAILABLE", "Q5 arm command publisher is unavailable", status=status)

        if status["lifecycle_state"] != "active":
            return _failure("LIFECYCLE_NOT_ACTIVE", "Q5 motion_manager must be active", status=status)

        ready, q5_fsm = q5_is_control_ready(self._client)
        if not ready:
            return _failure("Q5_FSM_NOT_READY", "Q5 /xbot_state must be fresh and READY or ACTIVE",
                            status={**status, "q5_fsm": q5_fsm})

        # Auto-prepare: delegate to arm_control's preparation path.
        if self._arm_control is not None:
            prepare_error = self._arm_control._ensure_prepared()
            if prepare_error:
                return {**prepare_error, "status": status}

        if not status["joint_state_fresh"]:
            return _failure("JOINT_STATE_UNAVAILABLE", "Refusing gesture without fresh /joint_states",
                            status=status)

        return None

    # ── Publish & hold ────────────────────────────────────────────────────

    def _publish_positions(self, positions: dict) -> bool:
        """Publish a position dict via the shared BodyCommandRouter."""
        return self._router.publish(positions)

    def _hold_positions(self, positions: dict) -> bool:
        """Hold positions for self._hold_repetitions at self._publish_rate."""
        published = False
        for _ in range(self._hold_repetitions):
            published = self._publish_positions(positions) or published
            time.sleep(1.0 / self._publish_rate)
        return published

    def _hold_current(self) -> dict:
        """Snapshot and hold all left/right arm joints at their current positions."""
        snap = self._client.snapshot()
        if not snap.get("fresh"):
            return {}
        joints = snap.get("joints", {})
        current: dict[str, float] = {}
        for name in ARM_JOINT_NAMES:
            v = joints.get(name)
            if v is not None:
                current[name] = float(v)
        if current:
            self._hold_positions(current)
        return current

    # ── Move worker ───────────────────────────────────────────────────────

    def _run_move(self, stop_event: threading.Event, frames, speed: float, side: str,
                  gesture: str, action_id: str | None):
        """Interpolate through gesture frames, honoring stop_event and commanded side.

        transition_ratio scales the interpolation time per frame:
          ratio < 1.0 → shorter transition, blends naturally into next frame
          ratio == 1.0 → full-speed transition with explicit hold
        """
        cancelled = True  # default: cancelled on any error/exception
        acquired = False
        try:
            previous_positions = dict(self._hold_current())  # start from current pose

            # Acquire the router once for the entire gesture so no other body
            # card can interleave commands between frames.
            acquired = self._router.acquire(CARD)
            if not acquired:
                return

            for frame_deg, hold_s, transition_ratio in frames:
                if stop_event.is_set():
                    break

                positions = _mirrored_positions("_current", side, frame_deg)
                if stop_event.is_set():
                    break

                # Validate pose against limits
                violations = _validate_pose_rad(positions)
                if violations:
                    print(f"[arm_gesture] Pose violation, stopping: {violations}")
                    break

                # Compute transition duration from max joint delta
                max_delta_rad = 0.0
                for name in ARM_JOINT_NAMES:
                    if name in positions and name in previous_positions:
                        delta = abs(positions[name] - previous_positions[name])
                        max_delta_rad = max(max_delta_rad, delta)

                # transition_ratio scales the interpolation time for natural blending
                transition_s = (max_delta_rad / speed if speed > 0 else 0.5) * transition_ratio
                transition_s = max(0.05, transition_s)  # minimum transition time

                # Interpolate
                steps = max(
                    int(math.ceil(max_delta_rad / self._max_step)),
                    int(math.ceil(transition_s * self._publish_rate)),
                    1,
                )

                for step in range(1, steps + 1):
                    if stop_event.is_set():
                        break
                    t = step / steps
                    interp_positions = {}
                    for name in ARM_JOINT_NAMES:
                        if name in positions and name in previous_positions:
                            prev = previous_positions[name]
                            tgt = positions[name]
                            interp_positions[name] = prev + (tgt - prev) * t
                    if interp_positions:
                        self._publish_positions(interp_positions)
                    stop_event.wait(transition_s / steps)

                if stop_event.is_set():
                    self._hold_current()
                else:
                    self._hold_positions(positions)

                previous_positions = {
                    name: positions[name] for name in ARM_JOINT_NAMES
                    if name in positions
                }

                # Hold for the specified duration
                if hold_s > 0 and not stop_event.is_set():
                    stop_event.wait(hold_s)
            cancelled = False  # all frames completed successfully
        except Exception:
            pass
        finally:
            if acquired:
                self._router.release(CARD)
            # ACP completion callback
            if action_id:
                if cancelled and not stop_event.is_set():
                    _acp_notify(action_id, "error", {"gesture": gesture, "error": "unexpected_failure"}, CARD)
                elif stop_event.is_set():
                    _acp_notify(action_id, "cancelled", {"gesture": gesture, "side": side}, CARD)
                else:
                    _acp_notify(action_id, "completed", {"gesture": gesture, "side": side}, CARD)

            with self._lock:
                if self._status.get("state") != "error":
                    self._status["state"] = "idle"
                self._stop_event = None
                self._motion_thread = None

    # ── Stop / Cancel ─────────────────────────────────────────────────────

    def _stop(self, reason: str) -> dict:
        with self._lock:
            stop_event = self._stop_event
            motion_thread = self._motion_thread
            self._stop_event = None
            self._motion_thread = None

        if stop_event is not None:
            stop_event.set()
        if motion_thread is not None and motion_thread is not threading.current_thread():
            motion_thread.join(timeout=1.0)

        held = self._hold_current()
        self._status = {"state": "stopped", "gesture": self._status.get("gesture"),
                        "updated_at_ms": int(time.time() * 1000), "reason": reason}
        return {"ok": True, "state": "stopped", "reason": reason,
                "hold_command_published": bool(held)}

    # ── Dispatch ──────────────────────────────────────────────────────────

    def dispatch(self, action: str, args: dict) -> dict:
        if action == "info":
            result = {"state": "ready" if self._router is not None else "unavailable",
                      "safety": self._safety()}
            with self._lock:
                result["status"] = dict(self._status)
            return result

        if action == "start":
            if self._arm_control is not None:
                return self._arm_control.dispatch(action, args)
            return {"state": "ready" if self._router is not None else "unavailable",
                    "safety": self._safety()}

        if action == "cancel":
            return self._stop("command")

        if action == "stop":
            return self.stop()

        if action == "reset":
            speed = _clamp(args.get("speed", 0.8), 0.2, 1.5)
            side = "both"
            check = self._validate_run("", side)
            if isinstance(check, dict):
                return check
            need_cancel = False
            with self._lock:
                if self._motion_thread is not None and self._motion_thread.is_alive():
                    need_cancel = True
            if need_cancel:
                self._stop("preempt")
            neutral = _mirrored_positions("_neutral", "both", _NEUTRAL_DEG)
            violations = _validate_pose_rad(neutral)
            if violations:
                return _failure("LIMIT_EXCEEDED", "Neutral pose out of range", violations=violations)
            # Run reset as async worker with ACP completion
            action_id = f"arm_gesture_reset_{int(time.time()*1000)}"
            frames = [(_NEUTRAL_DEG, 1.0, 1.0)]
            stop_event = threading.Event()
            with self._lock:
                self._stop_event = stop_event
                self._motion_thread = threading.Thread(
                    target=self._run_move,
                    args=(stop_event, frames, speed, side, "reset", action_id),
                    daemon=True,
                    name="q5_arm_gesture_reset",
                )
                self._motion_thread.start()
            self._status = {"state": "running", "gesture": "reset", "side": side,
                            "updated_at_ms": int(time.time() * 1000)}
            return {"ok": True, "state": "running", "gesture": "reset",
                    "side": side, "action_id": action_id}

        # Gesture actions
        if action not in _GESTURES_DEG:
            return None

        # Resolve side: salute uses salute_side, others use side
        if action == "salute":
            side = args.get("salute_side", args.get("side", "right"))
        else:
            side = args.get("side", "right")

        if side == "both" and action == "salute":
            return _failure("UNSAFE_BILATERAL_SALUTE",
                            "salute only supports one arm at a time to avoid head/arm interference")

        speed = _clamp(args.get("speed", 0.8), 0.2, 1.5)
        cycles = int(_clamp(args.get("cycles", 2), 1, 5))

        # Validate pre-flight
        check = self._validate_run(action, side)
        if isinstance(check, dict):
            return check

        # Check for concurrent motion
        with self._lock:
            if self._motion_thread is not None and self._motion_thread.is_alive():
                return _failure("MOTION_IN_PROGRESS",
                                "An arm gesture is already active; call cancel before starting another")

        # Build frames
        frames = _build_frames(action, cycles)

        # Validate all frames
        for frame_deg, _, _ in frames:
            positions = _mirrored_positions(action, side, frame_deg)
            violations = _validate_pose_rad(positions)
            if violations:
                return _failure("ARM_POSE_OUT_OF_RANGE",
                                "Semantic arm pose exceeds Q5 joint limits",
                                gesture=action, violations=violations)

        # ACP: generate action_id and completion callback for long gestures
        action_id = None
        if action in _GESTURES_DEG:
            action_id = f"arm_gesture_{action}_{int(time.time()*1000)}"

        # Start motion thread
        stop_event = threading.Event()
        with self._lock:
            self._stop_event = stop_event
            self._motion_thread = threading.Thread(
                target=self._run_move,
                args=(stop_event, frames, speed, side, action, action_id),
                daemon=True,
                name="q5_arm_gesture",
            )
            self._motion_thread.start()

        self._status = {"state": "running", "gesture": action, "side": side,
                        "cycles": cycles, "speed": speed,
                        "updated_at_ms": int(time.time() * 1000)}

        return {"ok": True, "state": "running", "gesture": action,
                "side": side, "cycles": cycles, "speed": speed,
                "action_id": action_id}


def make_plugin(plugin_config, namespace, executor, client):
    return Plugin(plugin_config, namespace, executor, client)
