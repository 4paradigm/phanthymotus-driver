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
            "description": ("G1 RealSense RGB camera settings. Read current auto modes and "
                            "option ranges, adjust one setting, or restore startup values. "
                            "The camera remains in automatic mode until explicitly changed."),
            "inputSchema": {
                "type": "object", "required": ["action"], "additionalProperties": False,
                "properties": {
                    "action": {"type": "string", "enum": ["get", "set", "reset", "info"]},
                    "auto_exposure": {"type": "boolean"},
                    "exposure": {"type": "number"},
                    "gain": {"type": "number"},
                    "auto_white_balance": {"type": "boolean"},
                    "white_balance": {"type": "number"},
                    "brightness": {"type": "number"},
                },
                "x-action-params": {
                    "get": {"params": []}, "info": {"params": []},
                    "set": {"params": list(FIELDS),
                            "description": "Set exactly one supported field; manual exposure, gain, or white balance disables its auto mode"},
                    "reset": {"params": [], "description": "Restore values read when this camera started"},
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
