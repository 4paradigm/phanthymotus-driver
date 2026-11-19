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


def _get_control_cmd(name):
    from highcontrol_py import ControlCmd
    return getattr(ControlCmd, name)


def _get_default_cmd():
    return _get_control_cmd("DEFAULT")


class MotorEnablePlugin:
    def __init__(self, plugin_config: dict, namespace: str, executor, high_ctrl, *, chassis_owner):
        self._high_ctrl = high_ctrl
        self._owner = chassis_owner
        self._action_lock = threading.Lock()
        self._enable_cancel = threading.Event()

    def start(self) -> None:
        pass

    def stop(self) -> None:
        self._enable_cancel.set()

    def get_tool(self) -> dict:
        return {
            "name": "motor_enable", "type": "actuator", "multiInstance": False,
            "description": "退出 Bumi 失能模式。使能可能移动关节；操作前将机器人仰面平躺并清空周围区域。不能解除 protection(26)，也不会自动起身或行走。",
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {"type": "string", "enum": ["enable", "info", "start", "stop"]},
                    "confirm": {"type": "boolean", "description": "确认机器人仰面平躺，关节活动范围安全且其他运动命令已停止。"},
                },
                "required": ["action"],
                "x-action-params": {
                    "enable": {"params": ["confirm"], "required": ["confirm"], "description": "确认安全放置后退出失能；使能前将机器人仰面平躺并确保关节活动范围安全。"},
                    "info": {"params": [], "description": "只读查询当前模式及上次使能是否仍未确认。"},
                },
                "x-is-dangerous": True,
                "x-resource": ["base"],
            },
            "topic_out": [],
        }

    def dispatch(self, action: str, args: dict) -> dict:
        if action == "start":
            return {"state": "ready", "command_sent": False}
        if action == "stop":
            self._enable_cancel.set()
            return {"state": "idle", "command_sent": False,
                    "message": "Observation cancelled; an already sent enable command cannot be undone."}
        if action not in {"info", "enable"}:
            return {"state": "error", "command_sent": False, "error": "Unknown motor_enable action"}
        if self._high_ctrl is None:
            return {"state": "error", "command_sent": False, "error": "Robot controller unavailable"}
        if action == "info":
            try:
                mode = int(self._high_ctrl.get_mode())
            except Exception as exc:
                return {"state": "error", "command_sent": False, "error": str(exc)}
            return {"state": "completed", "command_sent": False,
                    "workmode": mode, "workmode_name": _WORKMODE_NAMES.get(mode, "unknown"),
                    "enable_unconfirmed": self._owner._enable_unconfirmed}
        if args.get("confirm") is not True:
            return {"state": "error", "command_sent": False, "code": "CONFIRM_REQUIRED",
                    "error": "Confirm safe face-up placement and a clear joint movement area before enabling."}
        if not self._action_lock.acquire(blocking=False):
            return {"state": "error", "command_sent": False, "code": "ENABLE_BUSY",
                    "error": "A motor mode request is already active."}
        try:
            self._enable_cancel.clear()
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
        mode = None
        try:
            mode = int(self._high_ctrl.get_mode())
            target_mode = 0
            if self._owner._enable_unconfirmed:
                if mode == self._owner._mode_toggle_target:
                    self._owner._enable_unconfirmed = False
                else:
                    return {"state": "error", "command_sent": False, "code": "ENABLE_UNCONFIRMED",
                            "error": "Previous START delivery is uncertain. Inspect the robot and motion_state; START will not be resent."}
            if mode == target_mode:
                return {"state": "completed", "command_sent": False, "confirmed": True,
                        "workmode": mode, "workmode_name": _WORKMODE_NAMES[mode],
                        "message": "Already in the requested mode."}
            if mode != 30:
                return self._enable_mode_error(mode)
            try:
                source_mode = mode
                mode = self._owner.publish_mode_toggle(self._enable_cancel, source_mode, target_mode)
            finally:
                attempted = self._owner._enable_unconfirmed
            if mode is None:
                return {"state": "cancelled", "command_sent": False}
            if mode != source_mode:
                return self._enable_mode_error(mode)
            # Release the event even if stop arrives after START. Never toggle
            # START again to cancel or compensate for missing feedback.
            time.sleep(0.01)
            self._owner._publish_cmd(0, 0, 0, _get_default_cmd(), 0)
            deadline = time.monotonic() + 3.0
            while True:
                mode = int(self._high_ctrl.get_mode())
                if mode == target_mode:
                    self._owner._enable_unconfirmed = False
                    return {"state": "completed", "command_sent": True, "confirmed": True,
                            "workmode": mode, "workmode_name": _WORKMODE_NAMES[mode],
                            "message": "Requested mode observed; physical motion or settling is not confirmed."}
                if mode != source_mode:
                    return self._enable_mode_error(mode, command_sent=True)
                if self._enable_cancel.is_set():
                    return {"state": "cancelled", "command_sent": True, "confirmed": False,
                            "message": "Stopped waiting; enable may still take effect. Inspect motion_state."}
                if time.monotonic() >= deadline:
                    return {"state": "error", "command_sent": True, "confirmed": False,
                            "code": "ENABLE_TIMEOUT", "workmode": mode,
                            "error": "Requested mode was not observed within 3 seconds. START was not retried."}
                self._enable_cancel.wait(0.05)
        except Exception as exc:
            return {"state": "error", "command_sent": None if attempted else False,
                    "command_attempted": attempted, "confirmed": False,
                    "code": "ENABLE_SDK_ERROR", "error": str(exc),
                    "message": "Inspect motion_state before further commands; delivery may be uncertain."}

    @staticmethod
    def _enable_mode_error(mode: int, command_sent: bool = False) -> dict:
        return {"state": "error", "command_sent": command_sent, "confirmed": False,
                "code": "PROTECTION_ACTIVE" if mode == 26 else "INVALID_WORKMODE",
                "workmode": mode, "workmode_name": _WORKMODE_NAMES.get(mode, "unknown"),
                "error": ("Protection cannot be cleared by motor_enable. Inspect the robot and follow the vendor recovery procedure."
                          if mode == 26 else "Current mode is unsupported or changed; no further mode command was sent.")}
