"""Runtime controls for the G1 RealSense color sensor.

Only the camera capture process owns the SDK sensor. The card sends requests
to that process through a pipe; importing this module never opens hardware.
"""

from __future__ import annotations


FIELDS = {
    "auto_exposure": "enable_auto_exposure",
    "exposure": "exposure",
    "gain": "gain",
    "auto_white_balance": "enable_auto_white_balance",
    "white_balance": "white_balance",
    "brightness": "brightness",
}
BOOLEAN_FIELDS = {"auto_exposure", "auto_white_balance"}
MANUAL_MODE = {"exposure": "auto_exposure", "gain": "auto_exposure",
               "white_balance": "auto_white_balance"}


class RealSenseSettingsOps:
    """Read and change supported options on the already open color sensor."""

    def __init__(self, sensor, rs):
        self.sensor = sensor
        self.options = {
            name: getattr(rs.option, sdk_name)
            for name, sdk_name in FIELDS.items()
            if hasattr(rs.option, sdk_name)
            and sensor.supports(getattr(rs.option, sdk_name))
        }
        self.initial = self.read()

    def read(self):
        values = {}
        ranges = {}
        for name, option in self.options.items():
            value = self.sensor.get_option(option)
            values[name] = bool(value) if name in BOOLEAN_FIELDS else value
            option_range = self.sensor.get_option_range(option)
            ranges[name] = {"min": option_range.min, "max": option_range.max,
                            "step": option_range.step, "default": option_range.default,
                            "read_only": self.sensor.is_option_read_only(option)}
        return {"values": values, "ranges": ranges}

    def _write(self, name, value):
        self.sensor.set_option(self.options[name], float(value))

    def set(self, requested):
        if len(requested) != 1:
            raise ValueError("set requires exactly one setting")
        name, value = next(iter(requested.items()))
        current = self.read()
        if name not in self.options:
            raise ValueError(f"{name} is not supported by this color sensor")
        bounds = current["ranges"][name]
        if bounds["read_only"]:
            raise ValueError(f"{name} is read-only")
        if name in BOOLEAN_FIELDS:
            if type(value) is not bool:
                raise ValueError(f"{name} must be a boolean")
        elif (type(value) not in (int, float) or not bounds["min"] <= value <= bounds["max"]):
            raise ValueError(f"{name} must be a number in [{bounds['min']}, {bounds['max']}]")

        auto_name = MANUAL_MODE.get(name)
        if auto_name and auto_name in self.options:
            previous_auto = current["values"][auto_name]
            self._write(auto_name, False)
            try:
                self._write(name, value)
            except Exception:
                try:
                    self._write(auto_name, previous_auto)
                except Exception:
                    pass
                raise
        else:
            self._write(name, value)
        actual = self.read()
        if name in BOOLEAN_FIELDS and actual["values"][name] != value:
            raise RuntimeError(f"{name} readback differs from requested value")
        return {"success": True, "action": "set", "requested": requested,
                "actual": actual, "initial": self.initial}

    def reset(self):
        """Restore settings read at camera startup, with automatic modes last."""
        initial = self.initial["values"]
        for name in ("brightness", "exposure", "gain", "white_balance"):
            if name in self.options and not self.sensor.is_option_read_only(self.options[name]):
                if name in MANUAL_MODE and initial.get(MANUAL_MODE[name], False):
                    continue  # auto mode owns this changing value
                if name in MANUAL_MODE and MANUAL_MODE[name] in self.options:
                    self._write(MANUAL_MODE[name], False)
                self._write(name, initial[name])
        for name in ("auto_exposure", "auto_white_balance"):
            if name in self.options and not self.sensor.is_option_read_only(self.options[name]):
                self._write(name, initial[name])
        return {"success": True, "action": "reset", "actual": self.read(),
                "initial": self.initial}


class CameraSettingsPlugin:
    PREFIX = "camera_settings"

    def __init__(self, camera):
        self.camera = camera

    def get_tool(self):
        return {
            "name": self.PREFIX, "type": "actuator", "multiInstance": False,
            "description": ("G1 RGB 相机参数：先用 get 查看当前值和合法范围；"
                            "set 每次只填一个参数；reset 恢复本次相机启动时的设置。"),
            "inputSchema": {
                "type": "object", "required": ["action"], "additionalProperties": False,
                "properties": {
                    "action": {"type": "string", "enum": ["get", "set", "reset", "info"],
                               "description": "先读取，再单项设置，最后恢复。", "oneOf": [
                                   {"const": "get", "title": "读取当前设置"},
                                   {"const": "set", "title": "修改一个参数"},
                                   {"const": "reset", "title": "恢复启动时设置"},
                                   {"const": "info", "title": "查看设置说明"},
                               ]},
                    "auto_exposure": {"type": "boolean", "title": "自动曝光",
                                      "description": "true 自动调亮度；false 固定曝光。"},
                    "exposure": {"type": "number", "title": "曝光",
                                 "description": "曝光值；越大通常越亮，设置时关闭自动曝光。"},
                    "gain": {"type": "number", "title": "增益",
                             "description": "放大信号；越大通常越亮、噪点越多，设置时关闭自动曝光。"},
                    "auto_white_balance": {"type": "boolean", "title": "自动白平衡",
                                          "description": "true 自动校正色彩；false 固定色温。"},
                    "white_balance": {"type": "number", "title": "白平衡色温",
                                      "description": "色温值（K）；校正偏色，设置时关闭自动白平衡。"},
                    "brightness": {"type": "number", "title": "亮度",
                                   "description": "图像亮度；正数更亮，负数更暗。"},
                },
                "x-action-params": {
                    "get": {"params": [], "description": "读取当前值、支持项和合法范围。"},
                    "info": {"params": [], "description": "同 get，读取设置详情。"},
                    "set": {"params": list(FIELDS),
                            "description": "六个输入框只填一个；曝光、增益会关闭自动曝光，色温会关闭自动白平衡。"},
                    "reset": {"params": [], "description": "恢复本次相机启动时读取的设置。"},
                },
            },
        }

    def start(self):
        pass

    def stop(self):
        pass

    def dispatch(self, action, args):
        if action == "start":
            return {"state": "ready"}
        if action == "stop":
            return {"state": "idle"}
        if action in ("get", "info", "reset"):
            return self.camera.request_settings("get" if action == "info" else action)
        if action != "set":
            return None
        requested = {name: args[name] for name in FIELDS if name in args}
        if len(requested) != 1:
            return {"success": False, "error": "set requires exactly one setting"}
        return self.camera.request_settings("set", requested)
