# -*- coding: utf-8 -*-
# boos_taunt —— Q5 倒彩嘲讽手势 (composes arm_control low-level joint positioning)

from __future__ import annotations

import math
import threading
import time

try:
    from arm_control import ArmControlPlugin
except ImportError:
    ArmControlPlugin = None

from body_command import get_router as _get_body_router, BodyCommandRouter
from control_contract import q5_active_status, q5_is_control_ready
from joint_limits import JOINT_LIMITS
from q5_acp import notify as _acp_notify

CARD = "boos_taunt"
TYPE = "actuator"
TOPIC = "/{ns}/q5/boos_taunt"
FMT = "data/json"
HZ = 2.0
NODE = "q5_boos_taunt"
DESC = "Q5 倒彩嘲讽手势：双臂展开后左右轻摆，保持后回到中性姿态，由 arm_control 绝对关节插补驱动"

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

# Neutral pose: shoulder_pitch -30° (~-0.52 rad) so arms rest naturally in
# front of body. Positive shoulder_pitch rotates arms backward on Q5.
_NEUTRAL_DEG = [-30, 0, 0, 0, 0, 0, 0]

# Boos pose: both arms spread wide and slightly raised (shoulder_roll lifts
# arms sideways away from the torso; mirrored for the right arm).
_BOOS_SPREAD_DEG = [-10, 70, 0, -30, 0, 0, 0]
# Light sway offset applied to shoulder_roll on alternating cycles.
_BOOS_SWAY_DELTA_DEG = 12.0

_MOTION_LABELS = {"boos_wave": "倒彩嘲讽"}


# ── Helpers ──────────────────────────────────────────────────────────────────

def _deg2rad(deg: float) -> float:
    return deg * math.pi / 180.0


def _mirror_deg(pose: list[float]) -> list[float]:
    """Return a right-arm mirror of a left-arm degree pose."""
    mirrored = list(pose)
    for i in _MIRROR_MASK:
        mirrored[i] = -mirrored[i]
    return mirrored


def _pose_to_positions(deg_pose: list[float]) -> dict:
    """Map a 7-element left-arm degree list to both arms {joint_name: rad}."""
    result = {}
    for i, name in enumerate(ARM_JOINTS_LEFT):
        result[name] = _deg2rad(deg_pose[i])
    mirrored = _mirror_deg(deg_pose)
    for i, name in enumerate(ARM_JOINTS_RIGHT):
        result[name] = _deg2rad(mirrored[i])
    return result


def _clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


def _failure(code: str, message: str, **details) -> dict:
    return {"ok": False, "code": code, "message": message, "details": details}


# ── Frame builder ────────────────────────────────────────────────────────────

def _build_frames(cycles: int) -> list:
    """Return (pose_deg, hold_seconds, transition_ratio) frames for boos_wave.

    Sequence: spread both arms → alternate light left/right sway → hold →
    return to neutral. transition_ratio scales interpolation time per frame
    (< 1.0 blends naturally into the next frame).
    """
    frames: list[tuple[list[float], float, float]] = []

    # pose1: spread both arms
    frames.append((_BOOS_SPREAD_DEG, 0.3, 0.85))

    # pose2: sway — alternate shoulder_roll offset each cycle
    for i in range(cycles * 2):
        pose = list(_BOOS_SPREAD_DEG)
        delta = _BOOS_SWAY_DELTA_DEG if i % 2 == 0 else -_BOOS_SWAY_DELTA_DEG
        pose[1] = _BOOS_SPREAD_DEG[1] + delta
        frames.append((pose, 0.0, 0.65))

    # pose3: hold the spread pose
    frames.append((_BOOS_SPREAD_DEG, 0.5, 1.0))

    # pose4: return to neutral, full-speed transition with hold
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

class BoosTauntPlugin:
    """Boos & taunt semantic actuator built on arm_control / BodyCommandRouter."""

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

        # Thread-safe state.
        self._lock = threading.Lock()
        self._stop_event: threading.Event | None = None
        self._motion_thread: threading.Thread | None = None
        self._status: dict = {"state": "idle", "motion": None,
                              "updated_at_ms": int(time.time() * 1000)}

    # ── Tool definition ──────────────────────────────────────────────────

    def get_tool(self) -> dict:
        actions = ["start", "info", "execute", "cancel"]
        one_of_actions = [
            {"const": "start", "title": "检查连接状态"},
            {"const": "info", "title": "查看状态"},
            {"const": "execute", "title": "执行倒彩嘲讽"},
            {"const": "cancel", "title": "取消并保持"},
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
                    "motion": {
                        "type": "string", "title": "动作",
                        "enum": ["boos_wave"], "default": "boos_wave",
                        "oneOf": [{"const": "boos_wave", "title": "倒彩挥手"}],
                        "description": "第一版仅支持 boos_wave：双臂展开、左右轻摆、保持后回正。",
                    },
                    "cycles": {
                        "type": "integer", "title": "摆动次数",
                        "minimum": 1, "maximum": 5, "default": 2,
                        "description": "双臂左右轻摆的循环次数 [1,5]。",
                    },
                    "speed": {
                        "type": "number", "title": "关节速度(rad/s)",
                        "minimum": 0.2, "maximum": 1.5, "default": 0.8,
                        "description": "关节插补速度，范围[0.2,1.5]，默认0.8。",
                    },
                },
                "required": ["action"],
                "additionalProperties": False,
                "x-action-params": {
                    "start": {"params": [], "description": "检查 ROS 连接和机器人状态"},
                    "info": {"params": [], "description": "查看当前运动和安全条件"},
                    "execute": {"params": ["motion", "cycles", "speed"],
                                "description": "执行一次完整倒彩嘲讽：双臂展开→左右轻摆→保持→回正"},
                    "cancel": {"params": [], "description": "取消尚未发送的后续动作帧，并保持当前位置"},
                },
                "x-completion": {
                    "actions": ["execute"],
                    "timeout": 60,
                },
                # Both arms always: boos_wave drives left and right together, so
                # the card must claim both resource channels for ACP scheduling.
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

    def _validate_run(self) -> dict | None:
        """Pre-flight checks. Returns None on success or an error dict."""
        status = self._safety()

        if not status["ros_publisher_available"]:
            return _failure("ROS_UNAVAILABLE", "Q5 arm command publisher is unavailable", status=status)

        if status["lifecycle_state"] != "active":
            return _failure("LIFECYCLE_NOT_ACTIVE", "Q5 motion_manager must be active", status=status)

        # Gate on shared control contract: /xbot_state must be fresh and the FSM
        # must be READY(3) or ACTIVE(4).
        ready, q5_fsm = q5_is_control_ready(self._client)
        if not ready:
            return _failure("Q5_FSM_NOT_READY", "Q5 /xbot_state must be fresh and READY or ACTIVE",
                            status={**status, "q5_fsm": q5_fsm})

        # Auto-prepare: delegate to arm_control's vendor preparation path.
        if self._arm_control is not None:
            prepare_error = self._arm_control._ensure_prepared()
            if prepare_error:
                return {**prepare_error, "status": status}

        if not status["joint_state_fresh"]:
            return _failure("JOINT_STATE_UNAVAILABLE", "Refusing motion without fresh /joint_states",
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

    def _run_move(self, stop_event: threading.Event, frames, speed: float,
                  motion: str, action_id: str | None):
        """Interpolate through boos frames, honoring stop_event.

        transition_ratio scales the interpolation time per frame:
          ratio < 1.0 → shorter transition, blends naturally into next frame
          ratio == 1.0 → full-speed transition with explicit hold
        """
        cancelled = True  # default: cancelled on any error/exception
        acquired = False
        try:
            previous_positions = dict(self._hold_current())  # start from current pose

            # Acquire the router once for the entire motion so no other body
            # card can interleave commands between frames.
            acquired = self._router.acquire(CARD)
            if not acquired:
                return

            for frame_deg, hold_s, transition_ratio in frames:
                if stop_event.is_set():
                    break

                positions = _pose_to_positions(frame_deg)

                # Validate pose against limits
                violations = _validate_pose_rad(positions)
                if violations:
                    print(f"[{CARD}] Pose violation, stopping: {violations}")
                    break

                # Compute transition duration from max joint delta
                max_delta_rad = 0.0
                for name in ARM_JOINT_NAMES:
                    if name in positions and name in previous_positions:
                        delta = abs(positions[name] - previous_positions[name])
                        max_delta_rad = max(max_delta_rad, delta)

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
                    _acp_notify(action_id, "error", {"motion": motion, "error": "unexpected_failure"}, CARD)
                elif stop_event.is_set():
                    _acp_notify(action_id, "cancelled", {"motion": motion}, CARD)
                else:
                    _acp_notify(action_id, "completed", {"motion": motion}, CARD)

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
        self._status = {"state": "stopped", "motion": self._status.get("motion"),
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

        if action != "execute":
            return _failure("UNKNOWN_ACTION", f"Unknown action: {action}",
                            supported=["start", "info", "execute", "cancel"])

        motion = args.get("motion", "boos_wave")
        if motion != "boos_wave":
            return _failure("UNKNOWN_MOTION", f"Unknown motion: {motion}",
                            supported=["boos_wave"])

        speed = _clamp(args.get("speed", 0.8), 0.2, 1.5)
        cycles = int(_clamp(args.get("cycles", 2), 1, 5))

        # Pre-flight: ROS publisher, lifecycle, Q5 FSM ready, vendor prep, fresh joints.
        check = self._validate_run()
        if isinstance(check, dict):
            return check

        # Motion guard: reject concurrent execution.
        with self._lock:
            if self._motion_thread is not None and self._motion_thread.is_alive():
                return _failure("MOTION_IN_PROGRESS",
                                "A boos_taunt motion is already active; call cancel before starting another")

        # Build frames and validate every frame against joint limits.
        frames = _build_frames(cycles)
        for frame_deg, _, _ in frames:
            positions = _pose_to_positions(frame_deg)
            violations = _validate_pose_rad(positions)
            if violations:
                return _failure("ARM_POSE_OUT_OF_RANGE",
                                "Boos pose exceeds Q5 joint limits",
                                motion=motion, violations=violations)

        # ACP: generate action_id for async completion callback.
        action_id = f"boos_taunt_{motion}_{int(time.time()*1000)}"

        # Start motion thread.
        stop_event = threading.Event()
        with self._lock:
            self._stop_event = stop_event
            self._motion_thread = threading.Thread(
                target=self._run_move,
                args=(stop_event, frames, speed, motion, action_id),
                daemon=True,
                name="q5_boos_taunt",
            )
            self._motion_thread.start()

        self._status = {"state": "running", "motion": motion,
                        "cycles": cycles, "speed": speed,
                        "updated_at_ms": int(time.time() * 1000)}

        return {"ok": True, "state": "running", "motion": motion,
                "cycles": cycles, "speed": speed,
                "action_id": action_id}


def make_plugin(plugin_config, namespace, executor, client):
    return BoosTauntPlugin(plugin_config, namespace, executor, client)
