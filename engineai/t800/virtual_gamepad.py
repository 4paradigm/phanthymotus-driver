#!/usr/bin/env python3
"""Headless client for EngineAI Native SDK's official LCM virtual gamepad."""

from __future__ import annotations

import struct
import time
from dataclasses import asdict

from control import RepeatingCommand, action_schema, array_property, clamp


BUTTONS = {
    "LB": 0,
    "RB": 1,
    "A": 2,
    "B": 3,
    "X": 4,
    "Y": 5,
    "BACK": 6,
    "START": 7,
    "CROSS_X_UP": 8,
    "CROSS_X_DOWN": 9,
    "CROSS_Y_LEFT": 10,
    "CROSS_Y_RIGHT": 11,
}

ANALOGS = {
    "LT": 0,
    "RT": 1,
    "LEFT_STICK_X": 2,
    "LEFT_STICK_Y": 3,
    "RIGHT_STICK_X": 4,
    "RIGHT_STICK_Y": 5,
}

MOTION_MACROS = {
    "idle": ["LB", "START"],
    "passive": ["LB", "RB"],
    "stand": ["LB", "A"],
    "walk": ["LB", "B"],
    "dance": ["RB", "B"],
    "get_up": ["START", "CROSS_X_UP"],
    "lie_down": ["START", "CROSS_X_DOWN"],
}

# LCM fingerprint generated from EngineAI's GamepadKeys schema:
# int64 timestamp, int32 digital_states[12], double analog_states[6].
_GAMEPAD_FINGERPRINT = 0xAD95CC1F0C874EE5


def encode_gamepad(digital_states: list[int], analog_states: list[float], timestamp_us: int | None = None) -> bytes:
    """Encode one EngineAI lcm_data.GamepadKeys packet."""

    if len(digital_states) != 12:
        raise ValueError("digital_states must contain exactly 12 values")
    if len(analog_states) != 6:
        raise ValueError("analog_states must contain exactly 6 values")
    digital = [int(value) for value in digital_states]
    if any(value not in (0, 1) for value in digital):
        raise ValueError("digital_states values must be 0 or 1")
    analog = [clamp(value, -1.0, 1.0) for value in analog_states]
    timestamp = int(time.time() * 1_000_000) if timestamp_us is None else int(timestamp_us)
    return struct.pack(">Qq12i6d", _GAMEPAD_FINGERPRINT, timestamp, *digital, *analog)


class VirtualGamepadPlugin:
    """Publish buttons and sticks to Native SDK's input-command arbiter."""

    def __init__(self, config: dict, namespace: str, ros2):
        self._config = config
        self._url = str(config.get("url", "udpm://239.255.76.67:7667?ttl=1"))
        self._channel = str(config.get("channel", "virtual_gamepad/gamepad_keys"))
        self._lcm = None
        self._error = None
        self._last_command = {"digital_states": [0] * 12, "analog_states": [0.0] * 6}
        self._stream = RepeatingCommand(
            self._publish,
            self._publish_release,
            rate_hz=float(config.get("rate_hz", 20.0)),
        )

    def get_tool(self) -> dict:
        return {
            "name": "virtual_gamepad",
            "type": "actuator",
            "multiInstance": False,
            "description": "T800 Native SDK LCM 虚拟手柄：12 个按键、6 个模拟量和官方运动组合键",
            "inputSchema": action_schema(
                {
                    "command": (["buttons", "analogs", "duration"], "持续发送按键和摇杆状态"),
                    "press": (["buttons", "hold_seconds"], "按住一个或多个按键后自动释放"),
                    "sticks": (["left_x", "left_y", "right_x", "right_y", "lt", "rt", "duration"],
                               "持续发送双摇杆和扳机值"),
                    "macro": (["macro", "hold_seconds"], "发送 Native SDK 官方状态切换组合键"),
                    "release": ([], "释放全部虚拟按键和模拟量"),
                    "status": ([], "查询 LCM、映射和发送流状态"),
                },
                {
                    "buttons": array_property("按键名，可组合", item_type="string"),
                    "analogs": array_property("6 个模拟量：[LT, RT, LX, LY, RX, RY]，范围 -1..1"),
                    "duration": {"type": "number", "description": "秒；-1 持续发送"},
                    "hold_seconds": {"type": "number", "description": "按键保持时间，默认 0.5 秒"},
                    "macro": {"type": "string", "enum": list(MOTION_MACROS)},
                    **{name.lower(): {"type": "number", "minimum": -1, "maximum": 1}
                       for name in ("LEFT_X", "LEFT_Y", "RIGHT_X", "RIGHT_Y", "LT", "RT")},
                },
                "虚拟手柄动作",
            ),
        }

    def start(self) -> None:
        try:
            import lcm

            self._lcm = lcm.LCM(self._url)
            self._error = None
        except Exception as exc:
            self._lcm = None
            self._error = str(exc)
            raise RuntimeError(f"LCM virtual gamepad initialization failed: {exc}") from exc

    def stop(self) -> None:
        self._stream.stop()
        self._publish_release()

    def dispatch(self, action: str, args: dict) -> dict:
        if action in ("start", "info", "status"):
            return self._status()
        if action == "stop":
            self.stop()
            return {"state": "idle"}
        if action == "release":
            was_active = self._stream.stop()
            self._publish_release()
            return {"state": "released", "was_active": was_active}
        if self._lcm is None:
            return {"error": "LCM virtual gamepad is unavailable", "detail": self._error, "url": self._url}

        try:
            if action == "macro":
                name = str(args.get("macro", ""))
                if name not in MOTION_MACROS:
                    return {"error": f"unknown virtual gamepad macro: {name}"}
                command = self._make_command(MOTION_MACROS[name], [])
                duration = clamp(args.get("hold_seconds", 0.5), 0.05, 5.0)
            elif action == "press":
                command = self._make_command(args.get("buttons", []), [])
                duration = clamp(args.get("hold_seconds", 0.5), 0.05, 30.0)
            elif action == "sticks":
                analogs = [
                    args.get("lt", 0.0), args.get("rt", 0.0),
                    args.get("left_x", 0.0), args.get("left_y", 0.0),
                    args.get("right_x", 0.0), args.get("right_y", 0.0),
                ]
                command = self._make_command([], analogs)
                duration = float(args.get("duration", 1.0))
            elif action == "command":
                command = self._make_command(args.get("buttons", []), args.get("analogs", []))
                duration = float(args.get("duration", 1.0))
            else:
                return {"error": f"unknown virtual gamepad action: {action}"}
            snapshot = self._stream.start(command, duration)
            return {
                "state": "running" if duration else "released",
                "buttons": command["buttons"],
                "analogs": command["analog_states"],
                "duration": duration,
                "stream": asdict(snapshot),
            }
        except (TypeError, ValueError) as exc:
            return {"error": str(exc)}

    def _make_command(self, buttons, analogs) -> dict:
        if not isinstance(buttons, (list, tuple)):
            raise ValueError("buttons must be an array")
        names = [str(name).upper() for name in buttons]
        unknown = [name for name in names if name not in BUTTONS]
        if unknown:
            raise ValueError(f"unknown virtual gamepad buttons: {unknown}")
        digital = [0] * 12
        for name in names:
            digital[BUTTONS[name]] = 1
        if analogs in (None, []):
            analog = [0.0] * 6
        else:
            if not isinstance(analogs, (list, tuple)) or len(analogs) != 6:
                raise ValueError("analogs must contain exactly 6 values")
            analog = [clamp(value, -1.0, 1.0) for value in analogs]
        return {"buttons": names, "digital_states": digital, "analog_states": analog}

    def _publish(self, command: dict) -> None:
        if self._lcm is None:
            return
        self._last_command = {
            "digital_states": list(command["digital_states"]),
            "analog_states": list(command["analog_states"]),
        }
        self._lcm.publish(
            self._channel,
            encode_gamepad(command["digital_states"], command["analog_states"]),
        )

    def _publish_release(self) -> None:
        if self._lcm is None:
            return
        released = {"buttons": [], "digital_states": [0] * 12, "analog_states": [0.0] * 6}
        self._publish(released)

    def _status(self) -> dict:
        return {
            "state": "ready" if self._lcm is not None else "unavailable",
            "url": self._url,
            "channel": self._channel,
            "error": self._error,
            "buttons": list(BUTTONS),
            "analogs": list(ANALOGS),
            "macros": dict(MOTION_MACROS),
            "last_command": dict(self._last_command),
            "stream": asdict(self._stream.snapshot()),
        }
