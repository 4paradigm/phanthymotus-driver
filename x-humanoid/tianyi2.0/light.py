"""Tianyi 2.0 Pro system-light actuator card."""

import math
import threading

from rclpy.node import Node

from device import _RELIABLE_QOS


class LightPlugin:
    """Tianyi semantic system-light control with controller-domain cleanup."""
    _commands = {
        "blue_standby": 99,
        "blue_breathing": 20,
        "white": 310,
        "rainbow": 312,
        "warning": 12,
        "error": 10,
    }
    # Some Tianyi light domains ignore standby while active, so each exposed
    # effect has an explicit exit command used on timeout and before a switch.
    _exit_actions = {
        "blue_breathing": "service_ready",
        "warning": "warning_clear",
        "error": "error_clear",
        "white": "chat_quit",
        "rainbow": "chat_quit",
    }
    _internal_commands = {
        "service_ready": 22,
        "warning_clear": 13,
        "error_clear": 11,
        "chat_quit": 300,
    }

    def __init__(self, plugin_config, namespace, ros2):
        self._pub_node = Node("tianyi2_light_pub", context=ros2.ctx_tianyi)
        ros2.executor_tianyi.add_node(self._pub_node)
        self._pub = None
        self._timer = None
        self._timer_lock = threading.Lock()
        self._timer_generation = 0
        self._active_action = None

    def get_tool(self):
        return {
            "name": "light",
            "type": "actuator",
            "description": "天轶2.0 系统状态灯效。duration 未填写时默认持续 5 秒，-1 表示常亮。",
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {"type": "string", "enum": list(self._commands)},
                    "duration": {
                        "type": "number",
                        "default": 5,
                        "description": "灯效持续时间（秒）。未填写默认 5 秒；-1 表示常亮；正数到期后发送对应结束命令。",
                    },
                },
                "required": ["action"],
                "x-action-params": {
                    action: {"params": ["duration"], "description": action}
                    for action in self._commands
                },
                "x-hooks": {
                    "on_hearing":  {"action": "blue_breathing", "params": {"duration": -1}},
                    "on_thinking": {"action": "rainbow",        "params": {"duration": -1}},
                    "on_speaking": {"action": "white",          "params": {"duration": -1}},
                    "on_idle":     {"action": "blue_standby",   "params": {"duration": -1}},
                    "on_error":    {"action": "error",          "params": {"duration": 5}},
                },
            },
        }

    def start(self):
        from bodyctrl_msgs.msg import LightCtrl
        self._pub = self._pub_node.create_publisher(LightCtrl, "/xsys/light/ctrl", _RELIABLE_QOS)

    def stop(self):
        with self._timer_lock:
            self._timer_generation += 1
            if self._timer is not None:
                self._timer.cancel()
                self._timer = None

    def _publish(self, action):
        from bodyctrl_msgs.msg import LightCtrl
        command = self._commands.get(action, self._internal_commands.get(action))
        if command is None:
            raise ValueError(f"no command defined for light action: {action}")
        msg = LightCtrl()
        msg.cmd = int(command)
        msg.caller_id = "phanthy-motus"
        msg.caller_msg = f"Agent Core: {action}"
        self._pub.publish(msg)

    def _clear_active_effect(self):
        if self._active_action is None:
            return
        exit_action = self._exit_actions.get(self._active_action)
        if exit_action is not None:
            self._publish(exit_action)
        self._active_action = None

    def _finish_timed_effect(self, generation):
        with self._timer_lock:
            if generation != self._timer_generation:
                return
            self._timer = None
            self._clear_active_effect()

    def dispatch(self, action, args):
        if action not in self._commands:
            return {"error": f"unknown action: {action}"}
        raw_duration = args.get("duration", 5)
        try:
            duration = float(raw_duration)
        except (TypeError, ValueError):
            return {"error": "duration must be -1 or a positive number of seconds"}
        if isinstance(raw_duration, bool) or not math.isfinite(duration) or (duration != -1 and duration <= 0):
            return {"error": "duration must be -1 or a positive number of seconds"}

        with self._timer_lock:
            self._timer_generation += 1
            generation = self._timer_generation
            if self._timer is not None:
                self._timer.cancel()
                self._timer = None
            self._clear_active_effect()
            self._publish(action)
            self._active_action = None if action == "blue_standby" else action
            if duration != -1:
                self._timer = threading.Timer(duration, self._finish_timed_effect, args=(generation,))
                self._timer.daemon = True
                self._timer.start()

        return {"state": action, "duration": -1 if duration == -1 else duration}
