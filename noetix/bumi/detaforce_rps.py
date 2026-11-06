"""DetaForce basic-action Card and its official taught-recording backend.

No LowController, joint streaming, automatic standing, or invented recording
numbers. Hardware execution requires explicit enablement and six separately
verified taught clips. A workmode change is not proof of gesture completion.
"""

from __future__ import annotations

from copy import deepcopy
import json
import os
import secrets
import ssl
import threading
import time
import urllib.request

from detaforce_motion import INVITE_TEXT, LABELS


TOOL = "detaforce_rps"
PLAYBACK_POLL_S = .05
CONFIG_PROPERTIES = {
    "hardware_enabled": {"type": "boolean", "default": False,
                         "description": "仅在示教动作已逐项验收、机器人稳定站立且有人监护时开启。"},
    "playback_timeout_s": {"type": "number", "default": 60, "minimum": 5, "maximum": 120,
                           "description": "示教播放状态监控上限（秒）。超时退出本卡播放并回报 ACP 错误。"},
    **{f"{name}_recording_id": {
        "type": "integer", "minimum": -1, "maximum": 65535, "default": -1,
        "description": f"{label}的厂家示教编号；-1=未绑定，不发送动作。"}
       for name, label in LABELS.items()},
}


def _notify_completion(action_id, status, result, tool):
    """Use the local ACP endpoint and payload of the existing Bumi cards."""
    try:
        url = os.environ.get("AGENT_CORE_URL", "https://localhost:15678").rstrip("/")
        payload = json.dumps({"action_id": action_id, "status": status,
                              "result": result, "tool": tool, "ts": time.time()}).encode()
        request = urllib.request.Request(url+"/api/acp/complete", data=payload,
                                         headers={"Content-Type": "application/json"}, method="POST")
        context = ssl._create_unverified_context() if url.startswith("https://") else None
        with urllib.request.urlopen(request, timeout=5, context=context) as response:
            response.read()
        return True
    except Exception as exc:
        print(f"[Bumi ACP] callback failed for {action_id}: {exc}", flush=True)
        return False


def tool_spec(*, simulation=False):
    spec = {
        "name": TOOL, "type": "actuator", "multiInstance": False,
        "description": (
            "Bumi 三角洲猜拳基础动作：单手/双手向面部招手两次、双臂交替转动、猛攻（双拳前后交替）、鼠鼠双手举高、花来（面前举拳、肯定下压、回举）。"
            "仅提供动作原语，不识别人类出拳、不判断胜负。Bumi 无手指，末端形状固定。"
            + ("当前为 Gazebo 运动学仿真，不会控制真机。" if simulation else
               "真机通过已绑定的厂家示教回放执行；默认关闭，不直接下发自编关节轨迹。")
        ),
        "inputSchema": {
            "type": "object", "additionalProperties": False,
            "properties": {"action": {"type": "string",
                                       "enum": [*LABELS, "status", "stop_action"]}},
            "required": ["action"],
            "x-action-params": {
                **{name: {"params": [], "description": label + (
                    "。前臂向面部收回招手两次，语音由现有 tts/speaker 卡组合播放 " + INVITE_TEXT if name.startswith("invite") else "。")}
                   for name, label in LABELS.items()},
                "status": {"params": [], "description": "读取本卡动作状态；不将接收命令或模式退出伪报为动作完成。"},
                "stop_action": {"params": [], "description": "停止本卡持有的动作，不停止其他卡片发起的任务。"},
            },
            "x-hooks": {"on_interrupt_motion": {"action": "stop_action"}},
        },
        "topic_out": [],
    }
    if not simulation:
        spec["inputSchema"]["x-completion"] = {"actions": list(LABELS), "timeout": 150}
        spec["configSchema"] = {"type": "object", "additionalProperties": False,
                                "properties": deepcopy(CONFIG_PROPERTIES)}
    return spec


class DetaforceRpsPlugin:
    def __init__(self, plugin_config, loco_plugin, high_ctrl, *, completion_callback=None, clock=None):
        self._loco = loco_plugin
        self._ctrl = high_ctrl
        self._lock = threading.RLock()
        self._notify = completion_callback or _notify_completion
        self._clock = clock or time.monotonic
        self._parameters = {name: value["default"] for name, value in CONFIG_PROPERTIES.items()}
        self._validate_config({k: v for k, v in plugin_config.items() if k != "enabled"})
        self._active = None
        self._worker = None
        self._fault = None
        self._cancel = threading.Event()
        self._last = {"state": "idle", "command_sent": False}

    def _validate_config(self, changes):
        if not isinstance(changes, dict) or set(changes) - set(CONFIG_PROPERTIES):
            raise ValueError("Unknown DetaForce configuration parameter")
        for name, value in changes.items():
            if name == "hardware_enabled":
                if type(value) is not bool:
                    raise ValueError("hardware_enabled must be boolean")
            elif name == "playback_timeout_s":
                if type(value) not in (int, float) or not 5 <= value <= 120:
                    raise ValueError("playback_timeout_s must be finite and in 5..120")
            elif type(value) is not int or not -1 <= value <= 65535:
                raise ValueError(name + " must be an integer in -1..65535")
        self._parameters.update(changes)

    def get_tool(self):
        return tool_spec()

    def start(self):
        # Bundle boot and canvas start must never cause motion.
        return {"state": "ready"}

    def stop(self):
        with self._lock:
            worker = self._worker
            result = self._stop_action()
        if worker and worker is not threading.current_thread():
            worker.join(timeout=6)
        return {**result, "state": "idle", "action_status": result,
                "command_sent": bool(result.get("command_sent") or
                                     result.get("stop_result", {}).get("command_sent"))}

    def _status(self):
        if self._active and "terminal_status" not in self._active:
            mode = int(self._ctrl.get_mode())
            if mode == 23:
                self._active["playback_started_verified"] = True
            started = self._active["playback_started_verified"]
            self._last = {**self._active, "workmode": mode,
                          "state": "running" if mode == 23 else
                                   "playback_exited" if mode == 2 and started else
                                   "start_unconfirmed" if mode == 2 else "interrupted",
                          "motion_completion_verified": False}
        return {**deepcopy(self._last), "backend": "official_action_recording",
                "hardware_enabled": self._parameters["hardware_enabled"],
                "safety_fault": self._fault,
                "parameters": self._parameters.copy(),
                "recording_bound": {name: self._parameters[f"{name}_recording_id"] >= 0 for name in LABELS},
                "physical_interaction_validated": False,
                "completion_note": "playback_exited 仅表示观察到退出播放模式，动作形态和结束姿态仍需现场核验。"}

    def _stop_action(self):
        if not self._active:
            return {"state": "idle", "command_sent": False,
                    "message": "This Card owns no playback; no other task was stopped."}
        if "terminal_status" in self._active:
            return deepcopy(self._last)
        self._cancel.set()
        result = self._exit_playback()
        self._active["stop_result"] = deepcopy(result)
        return {"state": "stopped" if result.get("confirmed") else "error",
                "action_id": self._active["action_id"], "stop_result": result,
                "motion_completion_verified": False}

    def _exit_playback(self):
        """Only exit a playback mode held by this Card's active action."""
        try:
            mode = int(self._ctrl.get_mode())
            if mode == 23:
                result = self._loco.dispatch("stop_playback", {"_tool_name": "action_recording"})
                if not isinstance(result, dict):
                    return {"state": "error", "confirmed": False, "error": "Invalid stop response"}
                return result
            if mode == 2 and self._active and not self._active["playback_started_verified"]:
                return {"state": "error", "command_sent": False, "confirmed": False,
                        "workmode": mode, "error": "Playback start was never observed; inspect the robot."}
            return {"state": "idle" if mode == 2 else "error", "command_sent": False,
                    "confirmed": mode == 2, "workmode": mode}
        except Exception as exc:
            return {"state": "error", "confirmed": False, "error": str(exc)}

    def _monitor_playback(self, record, cancel, deadline):
        status = "error"
        result = {"state": "error", "error": "Playback monitor ended unexpectedly"}
        try:
            while True:
                with self._lock:
                    observation = self._status()
                    if cancel.is_set():
                        stop = record.get("stop_result", {})
                        status = "cancelled" if stop.get("confirmed") else "error"
                        result = {**observation, "state": "stopped" if status == "cancelled" else "error",
                                  "stop_result": deepcopy(stop)}
                        break
                    mode = observation["workmode"]
                    if mode == 2 and record["playback_started_verified"]:
                        status = "completed"
                        result = {**observation, "state": "playback_exited",
                                  "completion_basis": "sdk_workmode_exit"}
                        break
                    if mode not in (2, 23):
                        result = {**observation, "state": "error",
                                  "error": f"Playback interrupted by workmode {mode}"}
                        break
                    if self._clock() >= deadline:
                        result = {**observation, "state": "timeout",
                                  "error": "Playback mode did not exit before its timeout",
                                  "stop_result": self._exit_playback()}
                        break
                cancel.wait(PLAYBACK_POLL_S)
        except Exception as exc:
            with self._lock:
                result = {"state": "error", "error": str(exc),
                          "stop_result": self._exit_playback()}
        finally:
            with self._lock:
                claimed = self._active is record and "terminal_status" not in record
                if claimed:
                    record["terminal_status"] = status
                    if status == "error" and (not record["playback_started_verified"] or
                                              not result.get("stop_result", {}).get("confirmed", True)):
                        self._fault = "Playback state uncertain; inspect the robot before restarting the driver."
                    self._last = {**record, **result, "terminal_status": status,
                                  "motion_completion_verified": False,
                                  "physical_interaction_validated": False,
                                  "completion_callback_attempted": True}
                    payload = deepcopy(self._last)
            if claimed:
                try:
                    notified = self._notify(record["action_id"], status, payload, TOOL)
                except Exception as exc:
                    notified = False
                    with self._lock:
                        self._last["completion_callback_error"] = str(exc)
                finally:
                    with self._lock:
                        self._last["completion_notified"] = bool(notified)
                        self._active = None
                        self._worker = None

    def dispatch(self, action, args):
        if action == "start":
            return self.start()
        if action == "stop":
            return self.stop()
        if action == "stop_action":
            result = self.stop()
            return {**result, "state": "error"} if result["action_status"]["state"] == "error" else result
        with self._lock:
            if action in ("info", "status"):
                return self._status()
            if action == "config":
                if self._active:
                    return {"state": "error", "error": "Stop this Card's playback before changing bindings."}
                try:
                    self._validate_config({k: v for k, v in args.items() if k != "_tool_name"})
                except ValueError as exc:
                    return {"state": "error", "error": str(exc)}
                return {"state": "configured", "parameters": self._parameters.copy()}
            if action not in LABELS:
                return None
            if set(args) - {"_tool_name"}:
                return {"state": "error", "command_sent": False, "error": "Basic actions accept no parameters."}
            if self._active:
                return {"state": "error", "command_sent": False, "error": "This Card already owns a playback."}
            if self._fault:
                return {"state": "error", "command_sent": False, "error": self._fault}
            recording_id = self._parameters[f"{action}_recording_id"]
            if not self._parameters["hardware_enabled"] or recording_id < 0:
                return {"state": "unavailable", "command_sent": False,
                        "error": "Hardware playback is disabled or this action has no verified recording binding."}
            if self._loco is None or self._ctrl is None:
                return {"state": "unavailable", "command_sent": False, "error": "Official Bumi playback backend unavailable."}
            mode = int(self._ctrl.get_mode())
            if mode != 2 or self._loco.is_moving():
                return {"state": "error", "command_sent": False,
                        "error": "Requires stationary walking mode (2); this Card does not stand or prepare the robot."}
            result = self._loco.dispatch("play_recording", {
                "_tool_name": "action_recording", "recording_id": recording_id})
            if result.get("command_sent"):
                run_id = secrets.token_hex(16)
                self._cancel = threading.Event()
                self._active = {"run_id": run_id, "action_id": f"bumi_detaforce_rps_{run_id}", "action": action,
                                "recording_id": recording_id, "command_sent": True,
                                "playback_started_verified": result.get("workmode") == 23,
                                "playback_result": deepcopy(result)}
                record = self._active
                try:
                    deadline = self._clock()+self._parameters["playback_timeout_s"]
                    response = {**self._status(), "expected_duration_s": self._parameters["playback_timeout_s"]}
                    self._worker = threading.Thread(target=self._monitor_playback,
                                                    args=(record, self._cancel, deadline), daemon=True,
                                                    name="bumi_detaforce_playback")
                    self._worker.start()
                except Exception as exc:
                    stop = self._exit_playback()
                    if not stop.get("confirmed"):
                        self._fault = "Playback state uncertain; inspect the robot before restarting the driver."
                    self._active = None
                    self._worker = None
                    response = {"state": "error", "error": str(exc), "command_sent": True,
                                "stop_result": stop}
            else:
                response = {"state": "error", "command_sent": result.get("command_sent", False),
                            "error": "Official backend did not confirm entry into playback mode.",
                            "playback_result": result}
            if action.startswith("invite"):
                response["suggested_speech"] = INVITE_TEXT
            return response
