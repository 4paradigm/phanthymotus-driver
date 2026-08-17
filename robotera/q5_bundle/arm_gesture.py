# -*- coding: utf-8 -*-
# arm_gesture —— Q5 手臂语义手势 (composes arm_control low-level joint positioning)

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
DESC = "Q5 手臂语义手势：敬礼、欢迎、举手、握手、击掌及归零，由 arm_control 绝对关节插补驱动"

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

# Mirrored index: shoulder_roll(1), arm_yaw(2), elbow_yaw(4) flip sign
_MIRROR_MASK = [1, 2, 4]

# ── Gesture definitions (degrees) ────────────────────────────────────────────

_GESTURES_DEG = {
    "salute":          [-10, 90, 60, -110, 50, 0, 0],
    "welcome":         [-10, 65, 75, -100, 0, 0, 0],
    "raise":           [0, 103, 0, -15, 0, 0, 0],
    "shake_hands":     [-55, 15, 5, -35, 0, 0, 0],
    "high_five":       [-40, 40, -20, -80, 0, 0, 50],
}

_PREPARE_DEG = {
    "salute":          [-10, 40, 35, -45, 25, 0, 0],
    "welcome":         [-10, 45, 45, -60, 0, 0, 0],
    "raise":           [0, 75, 0, -30, 0, 0, 0],
    "shake_hands":     [-30, 10, 0, -20, 0, 0, 0],
    "high_five":       [-25, 25, -10, -45, 0, 0, 10],
}

# ── Helpers ──────────────────────────────────────────────────────────────────

_GESTURE_LABELS = {
    "salute": "敬礼", "welcome": "欢迎", "raise": "举手",
    "shake_hands": "握手", "high_five": "击掌",
}


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
    """Return a list of (pose_deg, hold_seconds, transition_ratio) tuples."""
    frames: list[tuple[list[float], float, float]] = []

    prepare = _PREPARE_DEG.get(gesture, _GESTURES_DEG[gesture])
    target = _GESTURES_DEG[gesture]

    frames.append((prepare, 0.25, 0.90))

    if gesture == "salute":
        frames.append((target, 1.1, 1.0))
    else:
        frames.append((target, 0.8, 0.90))

    if gesture == "shake_hands":
        for i in range(cycles * 2):
            pose = list(target)
            pose[3] = -28 if i % 2 == 0 else -42
            frames.append((pose, 0.30, 0.85))
    elif gesture == "welcome":
        for i in range(cycles * 2):
            pose = list(target)
            pose[3] = -110 if i % 2 == 0 else -90
            frames.append((pose, 0.35, 0.85))

    frames.append(([0.0] * 7, 1.0, 1.0))
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
            ctrl_cfg.setdefault("publish_rate_hz", 20.0)
            ctrl_cfg.setdefault("hold_repetitions", 3)
            try:
                self._arm_control = ArmControlPlugin(ctrl_cfg, namespace, executor, client)
            except Exception:
                self._arm_control = None

        # Shared body publisher (single-router pattern).
        self._router: BodyCommandRouter = _get_body_router(client, executor)

        # Motion parameters.
        self._max_step = float(plugin_config.get("max_step_rad", 0.010))
        self._publish_rate = float(plugin_config.get("publish_rate_hz", 20.0))
        self._hold_repetitions = int(plugin_config.get("hold_repetitions", 3))
        self._settle_tolerance = float(plugin_config.get("settle_tolerance_rad", 0.02))

        # Thread-safe state.
        self._lock = threading.Lock()
        self._stop_event: threading.Event | None = None
        self._motion_thread: threading.Thread | None = None
        self._status: dict = {"state": "idle", "gesture": None, "updated_at_ms": int(time.time() * 1000)}

    # ── Tool definition ──────────────────────────────────────────────────

    def get_tool(self) -> dict:
        actions = ["salute", "welcome", "raise", "shake_hands", "high_five", "reset",
                    "cancel", "stop", "start", "info"]
        one_of_actions = [
            {"const": "start", "title": "检查连接状态"},
            *[{"const": name, "title": label} for name, label in _GESTURE_LABELS.items()],
            {"const": "reset", "title": "归零"},
            {"const": "cancel", "title": "取消并保持"},
            {"const": "stop", "title": "停止并归零"},
            {"const": "info", "title": "查看状态"},
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
                        "type": "string", "title": "执行侧", "enum": ["left", "right", "both"],
                        "default": "right",
                        "oneOf": [
                            {"const": "left", "title": "左臂"},
                            {"const": "right", "title": "右臂"},
                            {"const": "both", "title": "双臂"},
                        ],
                        "description": "执行手臂选择。",
                    },
                    "salute_side": {
                        "type": "string", "title": "敬礼侧", "enum": ["left", "right"],
                        "default": "right",
                        "oneOf": [
                            {"const": "left", "title": "左臂敬礼"},
                            {"const": "right", "title": "右臂敬礼"},
                        ],
                        "description": "敬礼只支持单臂，默认右臂。",
                    },
                    "cycles": {
                        "type": "integer", "title": "循环次数",
                        "minimum": 1, "maximum": 5, "default": 2,
                        "description": "握手/欢迎的摆动循环次数 [1,5]。",
                    },
                    "speed": {
                        "type": "number", "title": "关节速度(rad/s)",
                        "minimum": 0.2, "maximum": 1.5, "default": 0.5,
                        "description": "关节插补速度，范围[0.2,1.5]，默认0.5。",
                    },
                },
                "required": ["action"],
                "additionalProperties": False,
                "x-action-params": {
                    "salute": {"params": ["salute_side", "speed"], "description": "抬起小臂、将手靠近额侧、停留后回正"},
                    "welcome": {"params": ["side", "cycles", "speed"], "description": "在身体侧上方抬起手掌并左右摆动后回正"},
                    "raise": {"params": ["side", "speed"], "description": "将手臂高举到头部上方后回正"},
                    "shake_hands": {"params": ["side", "cycles", "speed"], "description": "向前伸手并轻柔上下摆动，做出握手动作"},
                    "high_five": {"params": ["side", "speed"], "description": "将手掌伸到身体前方并保持在肩部附近，做出击掌等待姿势"},
                    "reset": {"params": ["speed"], "description": "取消序列并回到中性姿态"},
                    "cancel": {"params": [], "description": "取消尚未发送的后续动作帧，并保持当前位置"},
                    "stop": {"params": [], "description": "停止当前手势并回到中性姿态（归零）"},
                    "start": {"params": [], "description": "检查 ROS 连接和机器人状态"},
                    "info": {"params": [], "description": "查看当前运动和安全条件"},
                },
                "x-completion": {
                    "actions": ["salute", "welcome", "raise", "shake_hands", "high_five"],
                    "timeout": 60,
                },
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

        if status.get("same_name_publisher_count", 0) > 1:
            return _failure("DUPLICATE_BODY_PUBLISHER",
                            "Multiple q5_body_command publishers detected on /wr1_controller/commands",
                            status=status)

        if status["lifecycle_state"] != "active":
            return _failure("LIFECYCLE_NOT_ACTIVE", "Q5 motion_manager must be active", status=status)

        ready, q5_fsm = q5_is_control_ready(self._client)
        if not ready:
            return _failure("Q5_FSM_NOT_READY", "Q5 /xbot_state must be fresh and READY or ACTIVE",
                            status={**status, "q5_fsm": q5_fsm})

        # Check position-control preparation (same gate as arm_control).
        if self._arm_control is not None:
            arm_ctrl_prep = self._arm_control._client.q5_position_control_prepared
            if not arm_ctrl_prep:
                return _failure(
                    "DIRECT_CONTROL_NOT_PREPARED",
                    "Run q5_control_mode action=prepare_position_control first",
                    status=status,
                )

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
        """Interpolate through gesture frames, honoring stop_event and commanded side."""
        cancelled = True  # default: cancelled on any error/exception
        try:
            previous_positions = dict(self._hold_current())  # start from current pose

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

                transition_s = max_delta_rad / speed if speed > 0 else 0.5
                delay = max(0.12, transition_s * transition_ratio) + hold_s

                # Interpolate
                steps = max(
                    int(math.ceil(max_delta_rad / self._max_step)),
                    int(math.ceil(transition_s * self._publish_rate)),
                    1,
                )

                acquired = self._router.acquire(CARD)
                if not acquired:
                    break

                try:
                    for step in range(1, steps + 1):
                        if stop_event.is_set():
                            break
                        t = step / steps
                        for name in ARM_JOINT_NAMES:
                            if name in positions and name in previous_positions:
                                prev = previous_positions[name]
                                tgt = positions[name]
                                interp = prev + (tgt - prev) * t
                                self._publish_positions({name: interp})
                        stop_event.wait(transition_s / steps)
                finally:
                    if stop_event.is_set():
                        self._hold_current()
                    else:
                        self._hold_positions(positions)
                    self._router.release(CARD)

                previous_positions = {
                    name: positions[name] for name in ARM_JOINT_NAMES
                    if name in positions
                }

                if not stop_event.wait(delay):
                    pass  # normal hold completed
            cancelled = False  # all frames completed successfully
        except Exception:
            pass
        finally:
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
        if action in ("start", "info"):
            if self._arm_control is not None:
                return self._arm_control.dispatch(action, args)
            result = {"state": "ready" if self._router is not None else "unavailable",
                      "safety": self._safety()}
            if action == "info":
                with self._lock:
                    result["status"] = dict(self._status)
            return result

        if action == "cancel":
            return self._stop("command")

        if action == "stop":
            return self.stop()

        if action == "reset":
            speed = _clamp(args.get("speed", 0.5), 0.2, 1.5)
            # reset bypasses gesture validation; check side/robot safety instead
            side = "right"
            check = self._validate_run("", side)
            if isinstance(check, dict):
                return check
            # Check for active motion outside the lock to avoid deadlock:
            # _stop() also acquires self._lock, so we peek first then call
            # _stop() only if needed (and _stop() will re-acquire the lock).
            need_cancel = False
            with self._lock:
                if self._motion_thread is not None and self._motion_thread.is_alive():
                    need_cancel = True
            if need_cancel:
                self._stop("preempt")
            # Publish neutral to all arm joints
            neutral = {name: 0.0 for name in ARM_JOINT_NAMES}
            violations = _validate_pose_rad(neutral)
            if violations:
                return _failure("LIMIT_EXCEEDED", "Neutral pose out of range", violations=violations)
            acquired = self._router.acquire(CARD)
            if not acquired:
                return _failure("COMMAND_IN_PROGRESS", "Another arm card owns the command path")
            try:
                self._hold_positions(neutral)
            finally:
                self._router.release(CARD)
            self._status = {"state": "idle", "gesture": "reset",
                            "updated_at_ms": int(time.time() * 1000)}
            return {"ok": True, "state": "stopped", "gesture": "reset"}

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

        speed = _clamp(args.get("speed", 0.5), 0.2, 1.5)
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
