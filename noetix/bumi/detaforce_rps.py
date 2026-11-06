"""DetaForce basic-action Card and its official taught-recording backend.

No LowController, joint streaming, automatic standing, or invented recording
numbers. Hardware execution requires explicit enablement and six separately
verified taught clips. A workmode change is not proof of gesture completion.
"""

from __future__ import annotations

from copy import deepcopy
import secrets
import threading

from detaforce_motion import INVITE_TEXT, LABELS


TOOL = "detaforce_rps"
CONFIG_PROPERTIES = {
    "hardware_enabled": {"type": "boolean", "default": False,
                         "description": "仅在示教动作已逐项验收、机器人稳定站立且有人监护时开启。"},
    **{f"{name}_recording_id": {
        "type": "integer", "minimum": -1, "maximum": 65535, "default": -1,
        "description": f"{label}的厂家示教编号；-1=未绑定，不发送动作。"}
       for name, label in LABELS.items()},
}


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
        spec["configSchema"] = {"type": "object", "additionalProperties": False,
                                "properties": deepcopy(CONFIG_PROPERTIES)}
    return spec


class DetaforceRpsPlugin:
    def __init__(self, plugin_config, loco_plugin, high_ctrl):
        self._loco = loco_plugin
        self._ctrl = high_ctrl
        self._lock = threading.RLock()
        self._parameters = {name: value["default"] for name, value in CONFIG_PROPERTIES.items()}
        self._validate_config({k: v for k, v in plugin_config.items() if k != "enabled"})
        self._active = None
        self._last = {"state": "idle", "command_sent": False}

    def _validate_config(self, changes):
        if not isinstance(changes, dict) or set(changes) - set(CONFIG_PROPERTIES):
            raise ValueError("Unknown DetaForce configuration parameter")
        for name, value in changes.items():
            if name == "hardware_enabled":
                if type(value) is not bool:
                    raise ValueError("hardware_enabled must be boolean")
            elif type(value) is not int or not -1 <= value <= 65535:
                raise ValueError(name + " must be an integer in -1..65535")
        self._parameters.update(changes)

    def get_tool(self):
        return tool_spec()

    def start(self):
        # Bundle boot and canvas start must never cause motion.
        pass

    def stop(self):
        with self._lock:
            return self._stop_action()

    def _status(self):
        if self._active:
            mode = int(self._ctrl.get_mode())
            if mode == 23:
                self._active["playback_started_verified"] = True
            started = self._active["playback_started_verified"]
            self._last = {**self._active, "workmode": mode,
                          "state": "running" if mode == 23 else
                                   "playback_exited" if mode == 2 and started else
                                   "start_unconfirmed" if mode == 2 else "interrupted",
                          "motion_completion_verified": False}
            if mode != 23 and (started or mode != 2):
                self._active = None
        return {**deepcopy(self._last), "backend": "official_action_recording",
                "hardware_enabled": self._parameters["hardware_enabled"],
                "parameters": self._parameters.copy(),
                "recording_bound": {name: self._parameters[f"{name}_recording_id"] >= 0 for name in LABELS},
                "physical_interaction_validated": False,
                "completion_note": "playback_exited 仅表示观察到退出播放模式，动作形态和结束姿态仍需现场核验。"}

    def _stop_action(self):
        if not self._active:
            return {"state": "idle", "command_sent": False,
                    "message": "This Card owns no playback; no other task was stopped."}
        self._status()
        if not self._active:
            return self._status()
        if int(self._ctrl.get_mode()) != 23:
            return {**self._status(), "command_sent": False,
                    "error": "Playback start is unconfirmed; check the robot before retrying. No WALK was sent."}
        result = self._loco.dispatch("stop_playback", {"_tool_name": "action_recording"})
        if result.get("confirmed") and result.get("workmode") == 2:
            self._last = {**self._active, "state": "stopped", "stop_result": deepcopy(result),
                          "motion_completion_verified": False}
            self._active = None
        return {**self._status(), "stop_result": result}

    def dispatch(self, action, args):
        with self._lock:
            if action in ("info", "start", "status"):
                return self._status()
            if action in ("stop", "stop_action"):
                return self._stop_action()
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
            self._status()
            if self._active:
                return {"state": "error", "command_sent": False, "error": "This Card already owns a playback."}
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
                self._active = {"run_id": secrets.token_hex(8), "action": action,
                                "recording_id": recording_id, "command_sent": True,
                                "playback_started_verified": result.get("workmode") == 23,
                                "playback_result": deepcopy(result)}
                response = self._status()
            else:
                response = {"state": "error", "command_sent": result.get("command_sent", False),
                            "error": "Official backend did not confirm entry into playback mode.",
                            "playback_result": result}
            if action.startswith("invite"):
                response["suggested_speech"] = INVITE_TEXT
            return response
