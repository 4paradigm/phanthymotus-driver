"""Standalone Bumi card for exiting disabled mode."""

from __future__ import annotations

import threading
import time

_WORKMODE_NAMES = {0: "enabled", 1: "ready", 2: "walking", 5: "dance",
                   8: "greet", 9: "shake", 10: "cheer", 11: "start_teach",
                   12: "end_teach", 14: "save_teach_1", 23: "play_teach",
                   26: "protection", 27: "fall_to_stand", 28: "stand_to_fall",
                   29: "save_teach_2", 30: "disabled", 31: "dance1",
                   32: "dance2", 33: "tear"}

_MAX_FEEDBACK_AGE_MS = 1000


class FeedbackUnavailable(Exception):
    pass


def _get_control_cmd(name):
    from highcontrol_py import ControlCmd
    return getattr(ControlCmd, name)


def _get_default_cmd():
    return _get_control_cmd("DEFAULT")


class MotorEnablePlugin:
    PREFIX = "motor_enable"

    def __init__(self, plugin_config: dict, namespace: str, executor, high_ctrl, *, chassis_owner):
        self._high_ctrl = high_ctrl
        self._owner = chassis_owner
        self._action_lock = threading.Lock()
        self._cancel_lock = threading.Lock()
        self._enable_cancel = threading.Event()
        self._feedback_lock = threading.Lock()
        self._feedback = None

    def _observer(self):
        with self._feedback_lock:
            if self._feedback is None:
                try:
                    from highcontrol_py import RobotStatusObserver
                    self._feedback = RobotStatusObserver()
                except Exception as exc:
                    raise FeedbackUnavailable(f"状态订阅不可用: {exc}") from exc
            return self._feedback

    def _snapshot(self):
        try:
            return self._observer().snapshot()
        except Exception as exc:
            raise FeedbackUnavailable(f"无法读取状态订阅: {exc}") from exc

    @staticmethod
    def _recent(snapshot):
        age = snapshot.get("age_ms")
        return snapshot.get("workmode") is not None and age is not None and 0 <= age <= _MAX_FEEDBACK_AGE_MS

    def require_recent_mode(self):
        snapshot = self._snapshot()
        if not self._recent(snapshot):
            raise FeedbackUnavailable("最近 1 秒未收到机器人状态，无法安全发送 START。")
        return int(snapshot["workmode"])

    def mark(self):
        return self._observer().mark()

    def _reconcile(self, snapshot):
        marker = self._owner._motor_enable_marker
        received = snapshot.get("received_ns")
        if (self._owner._motor_enable_pending and marker is not None and received is not None
                and received > marker and self._recent(snapshot)
                and snapshot.get("workmode") == self._owner._mode_toggle_target):
            self._owner._enable_unconfirmed = False
            self._owner._motor_enable_pending = False

    def start(self) -> None:
        try:
            self._observer()
        except FeedbackUnavailable as exc:
            print(f"[motor_enable] {exc}", flush=True)

    def stop(self) -> None:
        with self._cancel_lock:
            self._enable_cancel.set()

    def get_tool(self) -> dict:
        return {
            "name": "motor_enable", "type": "actuator", "multiInstance": False,
            "description": "退出 Bumi 失能模式。使能可能移动关节；操作前将机器人仰面平躺并清空周围区域。不能解除 protection(26)，也不会自动起身或行走。",
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {"type": "string", "enum": ["enable", "status", "info", "start", "stop"]},
                    "confirm": {"type": "boolean", "description": "确认机器人仰面平躺，关节活动范围安全且其他运动命令已停止。"},
                },
                "required": ["action"],
                "x-action-params": {
                    "enable": {"params": ["confirm"], "required": ["confirm"], "description": "确认安全放置后退出失能；使能前将机器人仰面平躺并确保关节活动范围安全。"},
                    "status": {"params": [], "description": "查询订阅到的工作模式、接收时间和未确认状态。"},
                    "info": {"params": [], "description": "只读查询当前模式及上次使能是否仍未确认。"},
                    "start": {"params": [], "description": "将卡片置为就绪，不发送使能指令。"},
                    "stop": {"params": [], "description": "取消等待使能反馈，不发送失能指令，也不会撤回已发送的使能指令。"},
                },
                "x-is-dangerous": True,
                "x-resource": ["base"],
            },
            "topic_out": [],
        }

    def dispatch(self, action: str, args: dict) -> dict | None:
        if action == "start":
            self.start()
            return {"state": "ready", "command_sent": False}
        if action == "stop":
            self.stop()
            return {"state": "idle", "command_sent": False,
                    "message": "Observation cancelled; an already sent enable command cannot be undone."}
        if action not in {"status", "info", "enable"}:
            return None
        if self._high_ctrl is None:
            return {"state": "error", "command_sent": False, "error": "Robot controller unavailable"}
        if action in {"status", "info"}:
            try:
                with self._owner._rotate_lock:
                    snapshot = self._snapshot()
                    if not self._owner._body_action:
                        self._reconcile(snapshot)
                    unconfirmed = self._owner._enable_unconfirmed
            except FeedbackUnavailable as exc:
                return {"state": "error", "command_sent": False, "code": "FEEDBACK_UNAVAILABLE", "error": str(exc)}
            mode = snapshot.get("workmode")
            return {"state": "completed", "command_sent": False,
                    "workmode": mode, "workmode_name": _WORKMODE_NAMES.get(mode, "unknown"),
                    "enable_unconfirmed": unconfirmed,
                    "feedback_freshness": ("recent" if self._recent(snapshot) else
                                           "unavailable" if snapshot.get("workmode") is None else "stale"),
                    "feedback_age_ms": snapshot.get("age_ms"),
                    "feedback_sequence": snapshot.get("sequence"),
                    "message": "时间为本机收到状态的时间；不代表机器人源时间、命令回执或关节已停止。"}
        if args.get("confirm") is not True:
            return {"state": "error", "command_sent": False, "code": "CONFIRM_REQUIRED",
                    "error": "Confirm safe face-up placement and a clear joint movement area before enabling."}
        # Admission and cancellation initialization are one operation with
        # respect to stop(), including bundle shutdown. Do not hold this lock
        # during chassis reservation, publishing or the feedback wait.
        with self._cancel_lock:
            if not self._action_lock.acquire(blocking=False):
                return {"state": "error", "command_sent": False, "code": "ENABLE_BUSY",
                        "error": "A motor mode request is already active."}
            self._enable_cancel.clear()
        try:
            if not self._owner.reserve_motor_enable():
                return {"state": "error", "command_sent": False, "code": "CHASSIS_BUSY",
                        "error": "Another chassis action is active."}
            try:
                self._owner.pause_servo_for_motor_enable()
                return self._enable_motors()
            finally:
                try:
                    self._owner._set_servo_velocity_owner(False)
                finally:
                    self._owner.release_motor_enable()
        finally:
            self._action_lock.release()

    def _enable_motors(self) -> dict:
        attempted = False
        release_attempted = False
        default_count = 0
        mode = None
        try:
            target_mode = 0
            snapshot = self._snapshot()
            if self._owner._enable_unconfirmed:
                self._reconcile(snapshot)
                if self._owner._enable_unconfirmed:
                    return {"state": "error", "command_sent": False, "code": "ENABLE_UNCONFIRMED",
                            "retry_available": False,
                            "error": "上次 START 的结果尚未确认，不会再次发送。请检查机器人及控制链路，并遵循厂商确认的恢复流程。"}
            mode = self.require_recent_mode()
            if mode == target_mode:
                return {"state": "completed", "command_sent": False, "confirmed": True,
                        "workmode": mode, "workmode_name": _WORKMODE_NAMES[mode],
                        "feedback_freshness": "recent", "confirmation_scope": "locally_received_workmode",
                        "message": "最近收到的状态显示已使能；不能确认关节是否已停止。"}
            if mode != 30:
                return self._enable_mode_error(mode)
            print(f"[motor_enable] start_requested workmode={mode}", flush=True)
            try:
                source_mode = mode
                mode = self._owner.publish_mode_toggle(self._enable_cancel, source_mode, target_mode, self)
            finally:
                attempted = self._owner._enable_unconfirmed
            if mode is None:
                return {"state": "cancelled", "command_sent": False}
            if mode != source_mode:
                return self._enable_mode_error(mode)
            # Release the event even if stop arrives after START. Never toggle
            # START again to cancel or compensate for missing feedback.
            print("[motor_enable] START published to SDK; delivery unknown", flush=True)
            time.sleep(0.01)
            deadline = time.monotonic() + 4.0
            while True:
                # Follow the vendor example's DEFAULT refresh while this card
                # owns the chassis; never repeat the START edge.
                release_attempted = True
                self._owner._publish_cmd(0, 0, 0, _get_default_cmd(), 0)
                default_count += 1
                snapshot = self._snapshot()
                mode = snapshot.get("workmode")
                self._reconcile(snapshot)
                if not self._owner._enable_unconfirmed:
                    print(f"[motor_enable] target_observed workmode={mode} defaults={default_count}", flush=True)
                    return {"state": "completed", "command_sent": True, "confirmed": True,
                            "workmode": mode, "workmode_name": _WORKMODE_NAMES[mode],
                            "feedback_freshness": "recent", "confirmation_scope": "locally_received_workmode_after_start",
                            "delivery": "unknown",
                            "message": "发送后收到使能状态；不能证明命令送达或关节已停止。"}
                marker = self._owner._motor_enable_marker
                received = snapshot.get("received_ns")
                if (self._recent(snapshot) and received is not None and marker is not None
                        and received > marker and mode not in (source_mode, target_mode)):
                    print(f"[motor_enable] mode_changed workmode={mode} defaults={default_count}", flush=True)
                    return self._enable_mode_error(mode, command_sent=True)
                if self._enable_cancel.is_set():
                    print(f"[motor_enable] cancelled workmode={mode} defaults={default_count}", flush=True)
                    return {"state": "cancelled", "command_sent": True, "confirmed": False,
                            "message": "Stopped waiting; enable may still take effect. Inspect motion_state."}
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    print(f"[motor_enable] timeout workmode={mode} defaults={default_count}", flush=True)
                    return {"state": "error", "command_sent": True, "confirmed": False,
                            "code": "ENABLE_TIMEOUT", "workmode": mode,
                            "retry_available": False, "feedback_freshness": "recent" if self._recent(snapshot) else "stale",
                            "feedback_age_ms": snapshot.get("age_ms"),
                            "delivery": "unknown",
                            "error": "等待使能反馈超时（4 秒），未自动重发 START。",
                            "message": "机器人实际状态尚未确认。请检查机器人及控制链路，并遵循厂商确认的恢复流程。"}
                self._enable_cancel.wait(min(0.01, remaining))
        except FeedbackUnavailable as exc:
            if attempted and not release_attempted:
                try:
                    self._owner._publish_cmd(0, 0, 0, _get_default_cmd(), 0)
                except Exception:
                    pass
            return {"state": "error", "command_sent": None if attempted else False,
                    "command_attempted": attempted, "confirmed": False,
                    "code": "FEEDBACK_UNAVAILABLE", "retry_available": False, "error": str(exc)}
        except Exception as exc:
            # The SDK may have accepted START before raising; try to release it
            # once, but never issue another START on an uncertain outcome.
            if attempted and not release_attempted:
                try:
                    self._owner._publish_cmd(0, 0, 0, _get_default_cmd(), 0)
                    default_count += 1
                except Exception:
                    pass
            print(f"[motor_enable] sdk_error attempted={attempted} defaults={default_count} "
                  f"error_type={type(exc).__name__}", flush=True)
            return {"state": "error", "command_sent": None if attempted else False,
                    "command_attempted": attempted, "confirmed": False, "delivery": "unknown",
                    "code": "ENABLE_SDK_ERROR", "error": str(exc),
                    "message": "Inspect motion_state before further commands; delivery may be uncertain."}

    @staticmethod
    def _enable_mode_error(mode: int, command_sent: bool = False) -> dict:
        return {"state": "error", "command_sent": command_sent, "confirmed": False,
                "code": "PROTECTION_ACTIVE" if mode == 26 else "INVALID_WORKMODE",
                "workmode": mode, "workmode_name": _WORKMODE_NAMES.get(mode, "unknown"),
                "error": ("Protection cannot be cleared by motor_enable. Inspect the robot and follow the vendor recovery procedure."
                          if mode == 26 else "Current mode is unsupported or changed; no further mode command was sent.")}
