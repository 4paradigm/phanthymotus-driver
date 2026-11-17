"""Shared walking-mode backend for the Bumi locomotion cards.

The SDK is used only for feedback. Its RUN command remains unverified and is
never sent. Registration and startup only prepare the backend.
"""

from __future__ import annotations

import math
import threading
import time
import uuid

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

    PREFIX = "walking_mode"

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
        self._state_lock = threading.Lock()
        self._stopped = False
        self._worker = None

    def add_to_tool(self, tool: dict) -> None:
        """Extend a fresh tool declaration with an independent action."""
        schema = tool["inputSchema"]
        schema["properties"]["action"]["enum"].append(self.PREFIX)
        schema["properties"]["walking_model"] = self.get_tool()["inputSchema"][
            "properties"]["walking_model"]
        schema["x-action-params"][self.PREFIX] = {
            "params": ["walking_model"],
            "required": ["walking_model"],
            "description": (
                "切换行走模式，lower_body=仅腿部、whole_body=全身。"
                "请先停止移动并确认机器人平稳站立。"
            ),
        }
        schema.setdefault("allOf", []).append({
            "if": {"properties": {"action": {"const": self.PREFIX}}},
            "then": {"required": ["walking_model"]},
        })
        completion = schema.setdefault("x-completion", {"actions": [], "timeout": 0})
        completion["actions"].append(self.PREFIX)
        completion["timeout"] = max(completion["timeout"], self._completion_timeout())

    def _completion_timeout(self):
        return math.ceil(self._discovery_timeout + 6 * self._query_timeout
                         + self._verify_timeout + 5)

    def get_tool(self) -> dict:
        return {
            "name": "walking_mode",
            "type": "actuator",
            "multiInstance": False,
            "description": (
                "Bumi 行走模式切换：lower_body 为仅腿部行走，"
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
        with self._state_lock:
            self._stopped = False

    def cancel_switch(self) -> None:
        with self._state_lock:
            self._cancel.set()

    def stop(self) -> None:
        with self._state_lock:
            self._stopped = True
            self._cancel.set()
            worker = self._worker
        if worker is not None and worker is not threading.current_thread():
            worker.join(timeout=6)

    def dispatch(self, action: str, args: dict, *, tool_name=None, notify=None) -> dict | None:
        if action == "stop":
            self.stop()
            return {"state": "idle", "command_sent": False}
        if action == "start":
            self.start()
            return {"state": "ready", "command_sent": False}
        if action == "info":
            return {
                "state": "idle" if self._stopped else "ready",
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
        if self._loco is None:
            return self._error("missing_arbitration", model, "行走模式切换需要启用 loco。")
        # Serialize acceptance with stop/cancel, including starting the worker.
        # Reserve the chassis before returning running to either parent card.
        with self._state_lock:
            if self._stopped:
                return self._error("stopped", model, "卡片已停止，请先启动。")
            if not self._dispatch_lock.acquire(blocking=False):
                return self._error("busy", model, "已有模式切换正在执行。")
            self._cancel.clear()
            try:
                reservation = self._loco.reserve_walking_mode()
                reservation.__enter__()
            except Exception as exc:
                self._dispatch_lock.release()
                return self._error("preflight_failed", model, str(exc))

            if notify is not None:
                action_id = f"bumi_walking_mode_{uuid.uuid4().hex}"
                worker = threading.Thread(
                    target=self._switch_worker,
                    args=(model, reservation, action_id, tool_name, notify),
                    daemon=True, name="bumi_walking_mode")
                self._worker = worker
                try:
                    worker.start()
                except Exception as exc:
                    self._worker = None
                    self._release(reservation)
                    return self._error("worker_start_failed", model, str(exc))
                return {"state": "running", "action_id": action_id,
                        "requested_walking_model": model,
                        "expected_duration_s": self._completion_timeout()}

        try:
            return self._switch(model)
        except (OSError, TypeError, ValueError, RuntimeError) as exc:
            return self._error("mode_switch_failed", model, str(exc))
        finally:
            self._release(reservation)

    def _release(self, reservation):
        try:
            reservation.__exit__(None, None, None)
        finally:
            self._dispatch_lock.release()

    def _switch_worker(self, model, reservation, action_id, tool_name, notify):
        try:
            result = self._switch(model)
        except Exception as exc:
            result = self._error("mode_switch_failed", model, str(exc),
                                 command_may_have_been_sent=True)
        finally:
            self._release(reservation)
        result["action"] = self.PREFIX
        status = "completed" if result.get("state") == "completed" else "error"
        if result.get("code") in ("cancelled", "stopped"):
            status = "cancelled"
        notify(action_id, status, result, tool_name)

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
            if self._cancel.is_set():
                return self._error("cancelled", model, "切换已取消。")
            sdk_mode = int(self._high_ctrl.get_mode())
            if sdk_mode not in (2, 3):
                return self._error("invalid_workmode", model, "仅允许在两种行走模式之间切换。",
                                   observed_workmode=sdk_mode)
            host = self._host or discover_host(self._source_ip, self._discovery_timeout)
            with WalkingModeClient(host, timeout=self._query_timeout) as client:
                app_mode = client.get_status()
                sdk_mode = int(self._high_ctrl.get_mode())
                if self._cancel.is_set():
                    return self._error("cancelled", model, "切换已取消。")
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
                # A stop takes this lock before braking, so a pending TCP
                # write finishes (or is cancelled) before stop returns.
                with self._state_lock:
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
