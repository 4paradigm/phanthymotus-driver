"""Bumi walking-mode selection through the official App's TCP protocol.

The SDK is used only for feedback. Its RUN command remains unverified and is
never sent. The card stays disabled by default pending an on-site switch test.
"""

from __future__ import annotations

import math
import threading
import time

from walking_mode_client import MODEL_CODES, WalkingModeClient, discover_host


WALKING_MODELS = tuple(MODEL_CODES)


def _timeout(config, key, default):
    raw = config.get(key, default)
    value = float(raw)
    if isinstance(raw, bool) or not math.isfinite(value) or not 0.1 <= value <= 15:
        raise ValueError(f"{key} must be a finite number between 0.1 and 15 seconds")
    return value


class WalkingModePlugin:
    """Change the mode only while this card owns the idle chassis."""

    def __init__(self, plugin_config: dict, namespace: str, executor, high_ctrl,
                 loco_plugin=None):
        self._high_ctrl = high_ctrl
        self._loco = loco_plugin
        self._host = plugin_config.get("app_host") or None
        self._source_ip = plugin_config.get("discovery_source_ip", "192.168.55.102")
        self._query_timeout = _timeout(plugin_config, "query_timeout_s", 1.0)
        self._discovery_timeout = _timeout(plugin_config, "discovery_timeout_s", 3.0)
        self._verify_timeout = _timeout(plugin_config, "verify_timeout_s", 4.0)
        self._cancel = threading.Event()
        self._dispatch_lock = threading.Lock()

    def get_tool(self) -> dict:
        return {
            "name": "walking_mode",
            "type": "actuator",
            "multiInstance": False,
            "description": (
                "Bumi 行走模式切换（待现场验收）：lower_body 为仅腿部行走，"
                "whole_body 为全身行走。请在机器人平稳站立、停止移动后使用。"
                "切换后确认模式反馈；不会自动起身或开始行走。"
            ),
            "inputSchema": {
                "type": "object",
                "properties": {
                    "walking_model": {
                        "type": "string",
                        "enum": list(WALKING_MODELS),
                        "description": (
                            "lower_body=仅腿部行走；whole_body=全身行走。"
                            "相同模式不重复发送切换命令。"
                        ),
                    },
                },
                "required": ["walking_model"],
            },
            "topic_out": [],
        }

    def start(self) -> None:
        self._cancel.clear()

    def stop(self) -> None:
        self._cancel.set()

    def dispatch(self, action: str, args: dict) -> dict | None:
        if action == "stop":
            self.stop()
            return {"state": "idle", "command_sent": False}
        if action == "start":
            self.start()
            return {"state": "ready", "command_sent": False}
        if action == "info":
            return {
                "state": "idle" if self._cancel.is_set() else "ready",
                "transport": "noetix_app_tcp",
                "hardware_switch_verified": False,
                "command_sent": False,
            }
        # With no `action` field, BumiDeviceBundle uses the tool name.
        if action != "walking_mode":
            return None
        model = args.get("walking_model")
        if not isinstance(model, str) or model not in WALKING_MODELS:
            return {
                "state": "error",
                "code": "invalid_walking_model",
                "command_sent": False,
                "error": "walking_model 必须为 lower_body 或 whole_body。",
            }
        if self._cancel.is_set():
            return self._error("stopped", model, "卡片已停止，请先启动。")
        if self._loco is None:
            return self._error("missing_arbitration", model, "行走模式切换需要启用 loco。")
        if not self._dispatch_lock.acquire(blocking=False):
            return self._error("busy", model, "已有模式切换正在执行。")
        try:
            with self._loco.reserve_walking_mode():
                return self._switch(model)
        except (OSError, TypeError, ValueError, RuntimeError) as exc:
            return self._error("preflight_failed", model, str(exc))
        finally:
            self._dispatch_lock.release()

    @staticmethod
    def _error(code, model, message, **details):
        return {"state": "error", "code": code, "requested_walking_model": model,
                "command_sent": False, "mode_verified": False, "error": message,
                **details}

    def _stationary(self):
        joints = self._high_ctrl.get_joint_state()
        if len(joints) != 21:
            raise ValueError("无法取得全部 21 个关节状态，未发送切换命令。")
        velocities = [float(joint.vel) for joint in joints]
        if any(not math.isfinite(v) or abs(v) >= 0.15 for v in velocities):
            raise ValueError("机器人关节尚未静止，请停止移动并平稳站立后重试。")

    def _switch(self, model):
        wanted = MODEL_CODES[model]
        sent = attempted = preempted = False
        app_mode = sdk_mode = None
        try:
            sdk_mode = int(self._high_ctrl.get_mode())
            if sdk_mode not in (2, 3):
                return self._error("invalid_workmode", model, "仅允许在两种行走模式之间切换。",
                                   observed_workmode=sdk_mode)
            host = self._host or discover_host(self._source_ip, self._discovery_timeout)
            with WalkingModeClient(host, timeout=self._query_timeout) as client:
                app_mode = client.get_status()
                sdk_mode = int(self._high_ctrl.get_mode())
                if app_mode != sdk_mode or app_mode not in (2, 3):
                    return self._error("feedback_mismatch", model, "App 与 SDK 模式不一致，未发送切换命令。",
                                       app_workmode=app_mode, observed_workmode=sdk_mode)
                if sdk_mode == wanted:
                    return {"state": "completed", "walking_model": model,
                            "observed_workmode": sdk_mode, "app_workmode": app_mode,
                            "command_sent": False, "mode_verified": True}
                preempted = self._loco._preempt_servo("walking_mode")
                self._stationary()
                if int(self._high_ctrl.get_mode()) != sdk_mode:
                    return self._error("workmode_changed", model, "检查期间模式已变化，未发送切换命令。",
                                       preempted_loco_servo=preempted)
                if self._cancel.is_set():
                    return self._error("cancelled", model, "切换已取消。")
                attempted = True
                client.set_walking_model(model)
                sent = True
                deadline = time.monotonic() + self._verify_timeout
                confirmed = 0
                while time.monotonic() < deadline and not self._cancel.is_set():
                    app_mode = client.get_status()
                    sdk_mode = int(self._high_ctrl.get_mode())
                    if 26 in (app_mode, sdk_mode):
                        return self._error("protection", model, "机器人进入保护模式，请现场检查。",
                                           command_sent=True, observed_workmode=sdk_mode,
                                           app_workmode=app_mode, preempted_loco_servo=preempted)
                    confirmed = confirmed + 1 if app_mode == sdk_mode == wanted else 0
                    if confirmed >= 2:
                        return {"state": "completed", "walking_model": model,
                                "observed_workmode": sdk_mode, "app_workmode": app_mode,
                                "command_sent": True, "mode_verified": True,
                                "preempted_loco_servo": preempted}
                    self._cancel.wait(0.1)
            return self._error("cancelled" if self._cancel.is_set() else "mode_not_confirmed",
                               model, "未确认目标模式；未重发或自动回退。", command_sent=sent,
                               observed_workmode=sdk_mode, app_workmode=app_mode,
                               preempted_loco_servo=preempted)
        except (OSError, TypeError, ValueError, RuntimeError) as exc:
            return self._error("mode_switch_failed", model, str(exc), command_sent=sent,
                               command_may_have_been_sent=attempted,
                               observed_workmode=sdk_mode, app_workmode=app_mode,
                               preempted_loco_servo=preempted)
