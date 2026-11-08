#!/usr/bin/env python3
"""Gentle in-place spin card for the Noetix Bumi bundle.

The card turns the robot in place by a whole number of revolutions (1-5) in a
chosen direction.  It is deliberately slow: angular velocity is capped well
below the chassis full scale, ramps in over 0.6 s, decelerates near the target
and finishes with short IMU-corrected pulses instead of a hard stop.

The module keeps ROS and the vendor SDK out of module scope: the plugin receives
the HighController (and optionally the LocoPlugin for arbitration) and the pure
helpers in this file can be tested without rclpy or the robot.
"""

from __future__ import annotations

import json
import math
import os
import ssl
import threading
import time
import urllib.request
import uuid

# Bumi's measured full-scale yaw rate at a normalised vyaw of 1.0 (README).
_FULL_SCALE_WZ_RAD_S = 0.83
_MAX_TURNS = 5
_DEFAULT_MAX_VYAW = 0.55
_MIN_MAX_VYAW = 0.45
_MAX_MAX_VYAW = 0.80
# Above the ~0.40 normalised stiction boundary, so the final pulses move the
# robot instead of being absorbed by the legs.
_PULSE_VYAW = 0.45
_COARSE_STOP_DEG = 8.0
_FINE_TOLERANCE_DEG = 3.0
_ACCEL_S = 0.6
_CONTROL_PERIOD_S = 0.02
_SETTLE_S = 0.40
_MAX_PULSES = 15
_MODE_CHECK_S = 0.5
_NO_PROGRESS_S = 4.0
_ACP_TIMEOUT_S = 5.0


class SpinInputError(ValueError):
    """The spin request itself is invalid or unsafe."""


def wrap180(deg: float) -> float:
    """Normalise an angle in degrees to [-180, 180)."""
    return ((deg + 180.0) % 360.0) - 180.0


def validate_turns(value) -> int:
    """Return an integer 1-5, or raise SpinInputError.

    Accepts an integer, an integral float and a decimal string; booleans and
    fractional values are rejected because they cannot mean "a number of turns".
    """
    if isinstance(value, bool) or value is None:
        raise SpinInputError("turns 必须是 1–5 的整数")
    if isinstance(value, float):
        if not math.isfinite(value) or not value.is_integer():
            raise SpinInputError("turns 必须是 1–5 的整数")
        value = int(value)
    elif isinstance(value, str):
        text = value.strip()
        if not text.isdigit():
            raise SpinInputError("turns 必须是 1–5 的整数")
        value = int(text)
    elif not isinstance(value, int):
        raise SpinInputError("turns 必须是 1–5 的整数")
    if not 1 <= value <= _MAX_TURNS:
        raise SpinInputError(f"turns 必须在 1–{_MAX_TURNS} 圈之间，当前为 {value}")
    return value


def normalize_direction(value):
    """Map a direction label to ``clockwise`` or ``counterclockwise``."""
    if not isinstance(value, str):
        return None
    text = value.strip().lower()
    if text in ("clockwise", "cw", "right", "顺时针", "顺"):
        return "clockwise"
    if text in ("counterclockwise", "counter-clockwise", "ccw", "left",
                "逆时针", "逆"):
        return "counterclockwise"
    return None


def spin_target_deg(turns: int, direction: str) -> float:
    """Signed target angle: clockwise is right/negative, CCW is left/positive."""
    if direction == "clockwise":
        return -abs(turns) * 360.0
    return abs(turns) * 360.0


def speed_for_remaining(remaining_deg: float, max_vyaw: float) -> float:
    """Coarse angular speed for the remaining angle.

    Full capped speed far from the target, a gentle linear taper inside 25
    degrees, then a stop for the final-correct pulses.
    """
    remaining = abs(remaining_deg)
    if remaining > 25.0:
        return max_vyaw
    if remaining > _COARSE_STOP_DEG:
        return max(_PULSE_VYAW, max_vyaw * remaining / 25.0)
    return 0.0


def pulse_duration(remaining_deg: float) -> float:
    """Short correction pulse, scaled by the remaining angle."""
    return min(0.25, max(0.08, 0.03 * abs(remaining_deg)))


def expected_duration_s(turns: int, max_vyaw: float) -> float:
    deg_per_s = max_vyaw * _FULL_SCALE_WZ_RAD_S * 180.0 / math.pi
    return abs(turns) * 360.0 / deg_per_s + 12.0


class SpinPlugin:
    """MCP card: ``spin`` — turn Bumi in place slowly by 1-5 revolutions."""

    PREFIX = "spin"

    def __init__(self, plugin_config: dict, namespace: str, executor,
                 high_ctrl, loco_plugin=None):
        plugin_config = plugin_config or {}
        self._namespace = namespace
        self._high_ctrl = high_ctrl
        self._loco = loco_plugin
        try:
            configured = float(plugin_config.get("max_vyaw", _DEFAULT_MAX_VYAW))
        except (TypeError, ValueError):
            configured = _DEFAULT_MAX_VYAW
        if not math.isfinite(configured):
            configured = _DEFAULT_MAX_VYAW
        self._max_vyaw = min(_MAX_MAX_VYAW, max(_MIN_MAX_VYAW, configured))
        self._active = None
        self._thread = None
        self._last = None
        self._lock = threading.RLock()
        self._cmd = None

    # ── Card definition ──────────────────────────────────────────────────────

    def get_tool(self) -> dict:
        return {
            "name": "spin",
            "type": "actuator",
            "multiInstance": False,
            "description": (
                "让 Bumi 原地缓慢转圈。输入圈数（1–5）并选择顺时针或逆时针；"
                "卡片用 IMU 闭环控制，起步平滑、接近目标自动减速，最后用小幅修正停准。"
                "执行前必须确认机器人已稳定站立并处于行走模式，地面平坦防滑，"
                "周围至少 3m×3m 无人无障碍物；旋转过程中不要靠近或推拉机器人。"
            ),
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {
                        "type": "string",
                        "enum": ["spin", "stop", "info", "start"],
                        "description": ("spin=开始转圈；stop=立即停止；"
                                        "info=查看状态；start=检查控制器是否就绪。"),
                    },
                    "turns": {
                        "type": "integer", "minimum": 1, "maximum": _MAX_TURNS,
                        "description": "转圈圈数，1–5 的整数。",
                    },
                    "direction": {
                        "type": "string",
                        "enum": ["clockwise", "counterclockwise"],
                        "default": "clockwise",
                        "description": ("clockwise=顺时针（向右转），"
                                        "counterclockwise=逆时针（向左转）。"),
                    },
                },
                "required": ["action"],
                "additionalProperties": False,
                "x-action-params": {
                    "spin": {"params": ["turns", "direction"],
                             "description": ("缓慢原地转 turns 圈：clockwise 顺时针、"
                                             "counterclockwise 逆时针。最大 5 圈。")},
                    "stop": {"params": [],
                             "description": "立即停止转圈并发送零速度。"},
                    "info": {"params": [],
                             "description": "返回当前状态、最近一次结果和速度上限。"},
                    "start": {"params": [],
                              "description": "检查 HighController 是否就绪，不移动。"},
                },
                "x-hooks": {"on_interrupt_motion": {"action": "stop"}},
                "x-completion": {"actions": ["spin"], "timeout": 180},
            },
            "topic_out": [],
        }

    def get_tools(self) -> list:
        return [self.get_tool()]

    # ── Lifecycle ────────────────────────────────────────────────────────────

    def start(self):
        if self._high_ctrl is None:
            return {"state": "unavailable", "message": "HighController 未初始化"}
        return {"state": "ready"}

    def stop(self):
        with self._lock:
            record = self._active
            thread = self._thread
        if record is None:
            return {"state": "idle", "stopped": False}
        record["stop_event"].set()
        if (thread is not None and thread.is_alive()
                and thread is not threading.current_thread()):
            thread.join(timeout=8)
        with self._lock:
            still_running = self._active is record
        if still_running:
            try:
                self._publish(0.0)
            except Exception:
                pass
        return {"state": "idle", "stopped": True}

    def dispatch(self, action: str, args: dict):
        args = args or {}
        if action == "spin":
            return self._start_spin(args)
        if action == "stop":
            return self.stop()
        if action == "info":
            return self._info()
        if action == "start":
            return self.start()
        return None

    # ── Start / status ───────────────────────────────────────────────────────

    def _start_spin(self, args: dict) -> dict:
        try:
            turns = validate_turns(args.get("turns"))
            raw_direction = args.get("direction", "clockwise")
            direction = normalize_direction(raw_direction)
            if direction is None:
                raise SpinInputError(
                    "direction 只能是 clockwise 或 counterclockwise")
        except SpinInputError as exc:
            return {"state": "error", "code": "INVALID_PARAMETERS",
                    "error": str(exc)}

        try:
            self._preflight()
        except SpinInputError as exc:
            return {"state": "error", "error": str(exc)}

        with self._lock:
            if self._active is not None:
                return {"state": "error",
                        "error": "机器人正在转圈，请等待完成或先 stop"}
            target = spin_target_deg(turns, direction)
            record = {
                "action_id": f"bumi_spin_{uuid.uuid4().hex}",
                "turns": turns,
                "direction": direction,
                "target_deg": target,
                "started_at": time.time(),
                "stop_event": threading.Event(),
            }
            thread = threading.Thread(target=self._run_spin, args=(record,),
                                      daemon=True, name="bumi_spin")
            self._active = record
            self._thread = thread
            thread.start()

        return {
            "state": "running",
            "action_id": record["action_id"],
            "turns": turns,
            "direction": direction,
            "target_deg": target,
            "max_vyaw": self._max_vyaw,
            "expected_duration_s": round(expected_duration_s(turns, self._max_vyaw), 1),
        }

    def _info(self) -> dict:
        with self._lock:
            active = None if self._active is None else {
                "action_id": self._active["action_id"],
                "turns": self._active["turns"],
                "direction": self._active["direction"],
                "started_at": self._active["started_at"],
            }
            last = dict(self._last) if self._last else None
        mode = None
        if self._high_ctrl is not None:
            try:
                mode = int(self._high_ctrl.get_mode())
            except Exception:
                mode = None
        return {
            "state": "running" if active else "idle",
            "max_turns": _MAX_TURNS,
            "max_vyaw": self._max_vyaw,
            "workmode": mode,
            "active": active,
            "last": last,
        }

    # ── Worker ───────────────────────────────────────────────────────────────

    def _run_spin(self, record: dict) -> None:
        outcome = {"state": "error", "error": "spin did not start"}
        status = "error"
        reserved = False
        owns_chassis = False
        try:
            reserved = self._reserve_loco()
            self._prepare(reserved)
            owns_chassis = True
            outcome = self._rotate(record)
            status = {"done": "completed", "stopped": "cancelled"}.get(
                outcome["state"], "error")
        except Exception as exc:
            outcome = {"state": "error",
                       "error": f"{type(exc).__name__}: {exc}"}
        finally:
            if reserved or owns_chassis:
                for _ in range(3):
                    try:
                        self._publish(0.0)
                        break
                    except Exception:
                        time.sleep(0.05)
            if owns_chassis:
                self._release_chassis()
            if reserved:
                self._release_loco()
            result = {
                **outcome,
                "action": "spin",
                "turns": record["turns"],
                "direction": record["direction"],
                "target_deg": record["target_deg"],
                "duration_s": round(time.time() - record["started_at"], 1),
            }
            with self._lock:
                if self._active is record:
                    self._active = None
                    self._thread = None
                self._last = result
            self._notify_completion(record["action_id"], status, result)

    def _prepare(self, reserved: bool = False) -> None:
        self._require_walking()
        if self._loco is not None:
            if not reserved and self._loco.is_moving():
                raise SpinInputError(
                    "机器人正在执行其他移动动作，请等待完成或先停止后再转圈")
            # An explicit human/LLM action outranks a streaming policy.
            self._loco._preempt_servo("spin.spin")
            self._loco._set_servo_velocity_owner(True)

    def _reserve_loco(self) -> bool:
        """Reserve the chassis so loco's explicit actions refuse to start."""
        if self._loco is None:
            return False
        with self._loco._rotate_lock:
            if self._loco._body_action:
                raise SpinInputError(
                    f"底盘已被 {self._loco._body_action} 占用，请稍后再试")
            if self._loco._rotate_state["state"] == "running":
                raise SpinInputError("机器人正在执行目标移动或旋转，请先停止")
            self._loco._body_action = "spin.spin"
        return True

    def _release_loco(self) -> None:
        if self._loco is None:
            return
        with self._loco._rotate_lock:
            if self._loco._body_action == "spin.spin":
                self._loco._body_action = None

    def _release_chassis(self) -> None:
        if self._loco is not None:
            try:
                self._loco._set_servo_velocity_owner(False)
            except Exception as exc:
                print(f"[spin] releasing velocity owner failed: {exc}", flush=True)

    def _require_walking(self) -> None:
        if self._high_ctrl is None:
            raise SpinInputError("HighController 未初始化，无法转圈")
        mode = int(self._high_ctrl.get_mode())
        if mode == 26:
            raise SpinInputError("机器人处于保护模式，无法转圈；请先重启机器人")
        if mode != 2:
            raise SpinInputError(
                f"转圈需要机器人处于行走模式（workmode=2），当前为 {mode}。"
                "请先让机器人稳定站立并进入行走模式。")

    def _preflight(self) -> None:
        self._require_walking()
        if self._loco is not None and self._loco.is_moving():
            raise SpinInputError(
                "机器人正在执行其他移动动作，请等待完成或先停止后再转圈")

    def _rotate(self, record: dict) -> dict:
        target = float(record["target_deg"])
        deadline = time.monotonic() + record["turns"] * 25.0 + 25.0
        started = time.monotonic()
        next_mode_check = started + _MODE_CHECK_S
        traveled = 0.0
        last_yaw = self._yaw_deg()
        progress_ref = 0.0
        progress_at = started

        while True:
            if record["stop_event"].is_set():
                return self._stopped_result(target, traveled)
            now = time.monotonic()
            if now > deadline:
                raise RuntimeError("转圈超时，已停止；请检查机器人状态")
            traveled, last_yaw = self._advance(traveled, last_yaw)
            remaining = target - traveled
            if abs(traveled - progress_ref) >= 0.5:
                progress_ref = traveled
                progress_at = now
            if abs(remaining) > _COARSE_STOP_DEG and now - progress_at > _NO_PROGRESS_S:
                raise RuntimeError(
                    "机器人没有转动：可能未站立、地面打滑或被阻挡，已停止")
            if abs(remaining) <= _COARSE_STOP_DEG:
                break
            if now >= next_mode_check:
                self._require_walking()
                next_mode_check = now + _MODE_CHECK_S
            speed = speed_for_remaining(abs(remaining), self._max_vyaw)
            ramp = min(1.0, (now - started) / _ACCEL_S)
            self._publish(math.copysign(speed * ramp, remaining))
            time.sleep(_CONTROL_PERIOD_S)

        # Coarse stop, let the body settle, then correct with short pulses.
        self._publish(0.0)
        time.sleep(_SETTLE_S)
        for _ in range(_MAX_PULSES):
            if record["stop_event"].is_set():
                return self._stopped_result(target, traveled)
            traveled, last_yaw = self._advance(traveled, last_yaw)
            remaining = target - traveled
            if abs(remaining) <= _FINE_TOLERANCE_DEG:
                break
            duration = pulse_duration(abs(remaining))
            self._publish(math.copysign(_PULSE_VYAW, remaining))
            time.sleep(duration)
            self._publish(0.0)
            time.sleep(_SETTLE_S)

        traveled, _ = self._advance(traveled, last_yaw)
        error = target - traveled
        return {
            "state": "done",
            "traveled_deg": round(traveled, 1),
            "error_deg": round(error, 1),
            "within_tolerance": abs(error) <= _FINE_TOLERANCE_DEG,
        }

    def _stopped_result(self, target: float, traveled: float) -> dict:
        return {
            "state": "stopped",
            "traveled_deg": round(traveled, 1),
            "error_deg": round(target - traveled, 1),
        }

    # ── Sensors / actuation ──────────────────────────────────────────────────

    def _advance(self, traveled: float, last_yaw: float):
        yaw = self._yaw_deg()
        delta = wrap180(yaw - last_yaw)
        # At <=0.8 rad/s and 50 Hz a real step is <1 degree; a larger jump is
        # an IMU glitch, not motion.
        if abs(delta) <= 60.0:
            traveled += delta
        return traveled, yaw

    def _yaw_deg(self) -> float:
        imu = self._high_ctrl.get_imu_data()
        x, y, z, w = (float(value) for value in imu.ori)
        norm = math.hypot(x, y, z, w)
        if not math.isfinite(norm) or norm == 0.0:
            raise RuntimeError("IMU 四元数无效，无法读取朝向")
        x, y, z, w = x / norm, y / norm, z / norm, w / norm
        return math.degrees(math.atan2(2.0 * (w * z + x * y),
                                       1.0 - 2.0 * (y * y + z * z)))

    def _publish(self, vyaw: float) -> None:
        if self._loco is not None:
            self._loco.publish_velocity(0.0, 0.0, float(vyaw))
            return
        if self._high_ctrl is None:
            raise RuntimeError("HighController 未初始化")
        self._high_ctrl.publish_cmd(0.0, 0.0, float(vyaw), self._default_cmd(), 0)

    def _default_cmd(self):
        if self._cmd is None:
            from highcontrol_py import ControlCmd
            self._cmd = ControlCmd.DEFAULT
        return self._cmd

    # ── ACP completion ───────────────────────────────────────────────────────

    @staticmethod
    def _notify_completion(action_id: str, status: str, result: dict) -> None:
        try:
            url = os.environ.get("AGENT_CORE_URL",
                                 "https://localhost:15678").rstrip("/")
            payload = json.dumps({
                "action_id": action_id,
                "status": status,
                "result": result,
                "tool": "spin",
                "ts": time.time(),
            }).encode()
            request = urllib.request.Request(
                f"{url}/api/acp/complete", data=payload,
                headers={"Content-Type": "application/json"}, method="POST")
            context = (ssl._create_unverified_context()
                       if url.startswith("https://") else None)
            with urllib.request.urlopen(request, timeout=_ACP_TIMEOUT_S,
                                        context=context) as response:
                response.read()
        except Exception as exc:
            print(f"[spin] ACP callback failed for {action_id}: {exc}",
                  flush=True)
