# -*- coding: utf-8 -*-
# waist_gesture —— Q5 腰部语义手势 (composes waist_control low-level joint positioning)

from __future__ import annotations

import math
import threading
import time

from body_command import get_router as _get_body_router, BodyCommandRouter
from control_contract import q5_active_status, q5_is_control_ready
from joint_limits import JOINT_LIMITS

try:
    from legacy_direct_control import Q5ControlModePlugin
except ImportError:
    Q5ControlModePlugin = None

CARD = "waist_gesture"
TYPE = "actuator"
TOPIC = "/{ns}/q5/waist_gesture"
FMT = "data/json"
HZ = 2.0
NODE = "q5_waist_gesture"
DESC = "Q5 腰部语义手势：扭腰致意、左右摆腰、画圈，由 waist_control 绝对关节插补驱动"

WAIST_JOINT = "waist_yaw_joint"

# ── Gesture definitions (degrees) ────────────────────────────────────────────
# waist_yaw range: [-90, 90] degrees (URDF: -1.57~1.57 rad)
_GESTURES_DEG = {
    "bow":           [0],     # 先回正再轻微前倾（用waist_yaw=0表示归正配合头部即可）
    "twist_left":    [-30],   # 向左扭腰
    "twist_right":   [30],    # 向右扭腰
    "swing":         [0],     # 左右摆腰（动态循环）
    "circle":        [0],     # 画圈（动态循环）
}

_GESTURE_LABELS = {
    "bow": "扭腰致意", "twist_left": "左扭腰", "twist_right": "右扭腰",
    "swing": "左右摆腰", "circle": "画圈扭腰",
}

_NEUTRAL_DEG = [0]


def _deg2rad(deg: float) -> float:
    return deg * math.pi / 180.0


def _clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


def _failure(code: str, message: str, **details) -> dict:
    return {"ok": False, "code": code, "message": message, "details": details}


# ── Frame builder ────────────────────────────────────────────────────────────

def _build_frames(gesture: str, cycles: int) -> list:
    """Return a list of (pose_deg_list, hold_seconds, transition_ratio) tuples."""
    frames: list[tuple[list[float], float, float]] = []
    target = _GESTURES_DEG[gesture]

    if gesture == "bow":
        # 轻微扭腰致意：先到 -15° 再回正
        frames.append(([-15], 0.3, 0.9))
        frames.append(([15], 0.3, 0.9))
        frames.append((_NEUTRAL_DEG, 0.5, 1.0))
    elif gesture in ("twist_left", "twist_right"):
        frames.append((target, 0.8, 0.9))
        frames.append((_NEUTRAL_DEG, 0.5, 1.0))
    elif gesture == "swing":
        # 左右摆腰 cycles 次
        for i in range(cycles * 2):
            angle = -30 if i % 2 == 0 else 30
            frames.append(([angle], 0.35, 0.85))
        frames.append((_NEUTRAL_DEG, 0.5, 1.0))
    elif gesture == "circle":
        # 画圈：分8个相位点
        for i in range(8):
            angle = int(30 * math.sin(2 * math.pi * i / 8))
            frames.append(([angle], 0.2, 0.9))
        frames.append((_NEUTRAL_DEG, 0.5, 1.0))
    else:
        frames.append((target, 0.8, 0.9))
        frames.append((_NEUTRAL_DEG, 0.5, 1.0))

    return frames


# ── Plugin ───────────────────────────────────────────────────────────────────

class Plugin:
    """Semantic waist-gesture actuator built on BodyCommandRouter."""

    def __init__(self, plugin_config: dict, namespace: str, executor, client):
        self._client = client
        self._namespace = namespace

        # Embedded Q5 control-mode helper so waist_gesture can self-prepare
        self._control_mode: Q5ControlModePlugin | None = None
        if Q5ControlModePlugin is not None:
            try:
                self._control_mode = Q5ControlModePlugin(plugin_config, namespace, executor, client)
            except Exception:
                self._control_mode = None

        # Shared body publisher (single-router pattern).
        self._router: BodyCommandRouter = _get_body_router(client, executor)

        # Motion parameters.
        self._max_step = float(plugin_config.get("max_step_rad", 0.025))
        self._publish_rate = float(plugin_config.get("publish_rate_hz", 20.0))
        self._hold_repetitions = int(plugin_config.get("hold_repetitions", 3))

        # Thread-safe state.
        self._lock = threading.Lock()
        self._stop_event: threading.Event | None = None
        self._motion_thread: threading.Thread | None = None
        self._status: dict = {"state": "idle", "gesture": None, "updated_at_ms": int(time.time() * 1000)}

    # ── Tool definition ──────────────────────────────────────────────────

    def get_tool(self) -> dict:
        actions = ["bow", "twist_left", "twist_right", "swing", "circle", "reset",
                    "cancel", "stop", "start", "prepare", "info"]
        one_of_actions = [
            {"const": "start", "title": "检查连接状态"},
            *[{"const": name, "title": label} for name, label in _GESTURE_LABELS.items()],
            {"const": "reset", "title": "归零"},
            {"const": "cancel", "title": "取消并保持"},
            {"const": "stop", "title": "停止并归零"},
            {"const": "prepare", "title": "准备位置直控"},
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
                    "cycles": {
                        "type": "integer", "title": "循环次数",
                        "minimum": 1, "maximum": 5, "default": 2,
                        "description": "摆腰/画圈的循环次数 [1,5]。",
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
                    "bow": {"params": ["speed"], "description": "轻微扭腰致意"},
                    "twist_left": {"params": ["speed"], "description": "向左扭腰后回正"},
                    "twist_right": {"params": ["speed"], "description": "向右扭腰后回正"},
                    "swing": {"params": ["cycles", "speed"], "description": "左右摆腰循环后回正"},
                    "circle": {"params": ["cycles", "speed"], "description": "腰部画圈运动后回正"},
                    "reset": {"params": ["speed"], "description": "取消序列并回到中性姿态"},
                    "cancel": {"params": [], "description": "取消尚未发送的后续动作帧，并保持当前位置"},
                    "stop": {"params": [], "description": "停止当前手势并回到中性姿态（归零）"},
                    "prepare": {"params": [], "description": "执行位置直控准备：pos→READY→垂手→抬臂→ACTIVE，解锁控制"},
                    "start": {"params": [], "description": "检查 ROS 连接和机器人状态"},
                    "info": {"params": [], "description": "查看当前运动和安全条件"},
                },
                "x-completion": {
                    "actions": ["bow", "twist_left", "twist_right", "swing", "circle"],
                    "timeout": 60,
                },
            },
        }

    # ── Lifecycle ─────────────────────────────────────────────────────────

    def start(self) -> dict:
        return {"state": "ready" if self._router is not None else "unavailable"}

    def stop(self) -> dict:
        self._stop("driver_shutdown")
        return {"state": "idle"}

    def prepare(self) -> dict:
        """Delegate to embedded Q5ControlModePlugin for position-control preparation."""
        if self._control_mode is None:
            return _failure("CONTROL_MODE_UNAVAILABLE",
                            "Q5 control-mode helper is not initialized")
        return self._control_mode.dispatch("prepare_position_control", {})

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
        return status

    def _validate_run(self, gesture: str) -> dict | None:
        """Pre-flight checks. Returns None on success or an error dict."""
        if gesture and gesture not in _GESTURES_DEG:
            return _failure("UNKNOWN_GESTURE", f"Unknown gesture: {gesture}")

        status = self._safety()

        if not status["ros_publisher_available"]:
            return _failure("ROS_UNAVAILABLE", "Q5 body command publisher is unavailable", status=status)

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

        if not getattr(self._client, "q5_position_control_prepared", False):
            return _failure(
                "DIRECT_CONTROL_NOT_PREPARED",
                "Run waist_gesture action=prepare first",
                status=status,
            )

        if not status["joint_state_fresh"]:
            return _failure("JOINT_STATE_UNAVAILABLE", "Refusing gesture without fresh /joint_states",
                            status=status)

        return None

    # ── Publish & hold ────────────────────────────────────────────────────

    def _publish_positions(self, positions: dict) -> bool:
        return self._router.publish(positions)

    def _hold_positions(self, positions: dict) -> bool:
        published = False
        for _ in range(self._hold_repetitions):
            published = self._publish_positions(positions) or published
            time.sleep(1.0 / self._publish_rate)
        return published

    def _hold_current(self) -> dict:
        snap = self._client.snapshot()
        if not snap.get("fresh"):
            return {}
        joints = snap.get("joints", {})
        v = joints.get(WAIST_JOINT)
        if v is not None:
            current = {WAIST_JOINT: float(v)}
            self._hold_positions(current)
            return current
        return {}

    # ── Move worker ───────────────────────────────────────────────────────

    def _run_move(self, stop_event: threading.Event, frames, speed: float,
                  gesture: str, action_id: str | None):
        """Interpolate through gesture frames, honoring stop_event."""
        cancelled = True
        try:
            previous_positions = dict(self._hold_current())

            for frame_deg, hold_s, transition_ratio in frames:
                if stop_event.is_set():
                    break

                target_rad = _deg2rad(frame_deg[0])
                positions = {WAIST_JOINT: target_rad}

                # Validate against limits
                lo, hi = JOINT_LIMITS.get(WAIST_JOINT, (-1.57, 1.57))
                if target_rad < lo - 1e-6 or target_rad > hi + 1e-6:
                    print(f"[waist_gesture] Pose violation: {target_rad:.4f} outside [{lo:.4f},{hi:.4f}]")
                    break

                prev_val = previous_positions.get(WAIST_JOINT, 0.0)
                max_delta_rad = abs(target_rad - prev_val)

                transition_s = max_delta_rad / speed if speed > 0 else 0.5
                delay = max(0.12, transition_s * transition_ratio) + hold_s

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
                        interp = prev_val + (target_rad - prev_val) * t
                        self._publish_positions({WAIST_JOINT: interp})
                        stop_event.wait(transition_s / steps)
                finally:
                    if stop_event.is_set():
                        self._hold_current()
                    else:
                        self._hold_positions(positions)
                    self._router.release(CARD)

                previous_positions = {WAIST_JOINT: target_rad}

                if not stop_event.wait(delay):
                    pass
            cancelled = False
        except Exception:
            pass
        finally:
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
            return {"state": "ready" if self._router is not None else "unavailable",
                    "safety": self._safety()}

        if action == "cancel":
            return self._stop("command")

        if action == "stop":
            return self.stop()

        if action == "prepare":
            return self.prepare()

        if action == "reset":
            speed = _clamp(args.get("speed", 0.8), 0.2, 1.5)
            check = self._validate_run("")
            if isinstance(check, dict):
                return check
            need_cancel = False
            with self._lock:
                if self._motion_thread is not None and self._motion_thread.is_alive():
                    need_cancel = True
            if need_cancel:
                self._stop("preempt")
            neutral = {WAIST_JOINT: _deg2rad(_NEUTRAL_DEG[0])}
            acquired = self._router.acquire(CARD)
            if not acquired:
                return _failure("COMMAND_IN_PROGRESS", "Another card owns the command path")
            try:
                snap = self._client.snapshot()
                current = {WAIST_JOINT: 0.0}
                if snap.get("fresh"):
                    v = snap.get("joints", {}).get(WAIST_JOINT)
                    if v is not None:
                        current[WAIST_JOINT] = float(v)
                max_delta = abs(neutral[WAIST_JOINT] - current[WAIST_JOINT])
                transition_s = max_delta / speed if speed > 0 else 1.0
                steps = max(int(math.ceil(max_delta / self._max_step)),
                           int(math.ceil(transition_s * self._publish_rate)), 1)
                for step in range(1, steps + 1):
                    t = step / steps
                    interp = current[WAIST_JOINT] + (neutral[WAIST_JOINT] - current[WAIST_JOINT]) * t
                    self._publish_positions({WAIST_JOINT: interp})
                    time.sleep(transition_s / steps)
                self._hold_positions(neutral)
            finally:
                self._router.release(CARD)
            self._status = {"state": "idle", "gesture": "reset",
                            "updated_at_ms": int(time.time() * 1000)}
            return {"ok": True, "state": "stopped", "gesture": "reset"}

        # Gesture actions
        if action not in _GESTURES_DEG:
            return None

        speed = _clamp(args.get("speed", 0.8), 0.2, 1.5)
        cycles = int(_clamp(args.get("cycles", 2), 1, 5))

        # Validate pre-flight
        check = self._validate_run(action)
        if isinstance(check, dict):
            return check

        # Check for concurrent motion
        with self._lock:
            if self._motion_thread is not None and self._motion_thread.is_alive():
                return _failure("MOTION_IN_PROGRESS",
                                "A waist gesture is already active; call cancel before starting another")

        # Build frames
        frames = _build_frames(action, cycles)

        # Start motion thread
        stop_event = threading.Event()
        action_id = f"waist_gesture_{action}_{int(time.time()*1000)}"
        with self._lock:
            self._stop_event = stop_event
            self._motion_thread = threading.Thread(
                target=self._run_move,
                args=(stop_event, frames, speed, action, action_id),
                daemon=True,
                name="q5_waist_gesture",
            )
            self._motion_thread.start()

        self._status = {"state": "running", "gesture": action,
                        "cycles": cycles, "speed": speed,
                        "updated_at_ms": int(time.time() * 1000)}

        return {"ok": True, "state": "running", "gesture": action,
                "cycles": cycles, "speed": speed,
                "action_id": action_id}


def make_plugin(plugin_config, namespace, executor, client):
    return Plugin(plugin_config, namespace, executor, client)
