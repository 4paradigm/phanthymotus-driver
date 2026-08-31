"""MAVLink transport and MCP-facing tools for Qianjiao 2.0 Pro."""
from __future__ import annotations

import threading
import time
from typing import Any

try:
    from pymavlink import mavutil
except ImportError:  # importable in development without the optional SDK
    mavutil = None

UINT16_MAX = 65535
CHANNELS = ("heave", "pitch", "forward", "yaw", "lateral", "roll")
COMMAND_LONG = 76
MAV_CMD_COMPONENT_ARM_DISARM = 400
MAV_MODE_FLAG_SAFETY_ARMED = 128


def _pwm(value: Any) -> int:
    value = float(value)
    if not -1.0 <= value <= 1.0:
        raise ValueError("axis values must be in [-1, 1]")
    return int(round(1500 + value * 400))


class MockLink:
    def __init__(self):
        self.armed = False
        self.last_rc = [1500] * 6
        self.last_command: dict[str, Any] | None = None
        self.last_heartbeat = time.monotonic()

    def heartbeat(self):
        self.last_heartbeat = time.monotonic()


class QianjiaoDevice:
    def __init__(self, cfg: dict):
        self.cfg = cfg
        self.target_ip = str(cfg.get("target_ip", "192.168.2.1"))
        self.target_port = int(cfg.get("target_port", 14550))
        self.timeout = float(cfg.get("heartbeat_timeout", 3.0))
        self.rate = max(0.2, float(cfg.get("heartbeat_rate", 1.0)))
        self.mock = bool(cfg.get("mock", False))
        self.link: Any = MockLink() if self.mock else None
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._lock = threading.RLock()
        self._last_heartbeat = 0.0
        self._armed = False
        self._last_error: str | None = None

    def start(self):
        if self._thread and self._thread.is_alive():
            return
        if not self.mock:
            if mavutil is None:
                raise RuntimeError("pymavlink is required for live mode")
            # udp: creates a bidirectional socket and sends to the configured ROV.
            self.link = mavutil.mavlink_connection(
                f"udp:{self.target_ip}:{self.target_port}",
                source_system=int(self.cfg.get("source_system", 255)),
                source_component=int(self.cfg.get("source_component", 190)),
                mavlink20=False,
                dialect="ardupilotmega",
            )
        self._stop.clear()
        self._thread = threading.Thread(target=self._loop, daemon=True, name="qianjiao-mavlink")
        self._thread.start()

    def stop(self):
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=2)
        if self.link is not None and hasattr(self.link, "close"):
            self.link.close()

    def _loop(self):
        while not self._stop.is_set():
            try:
                if self.mock:
                    self.link.heartbeat()
                else:
                    self.link.mav.heartbeat_send(11, 3, 0, 0, 4)
                    msg = self.link.recv_match(blocking=False)
                    while msg is not None:
                        if msg.get_type() == "HEARTBEAT":
                            self._last_heartbeat = time.monotonic()
                            self._armed = bool(int(getattr(msg, "base_mode", 0)) & MAV_MODE_FLAG_SAFETY_ARMED)
                        msg = self.link.recv_match(blocking=False)
            except Exception as exc:
                self._last_error = str(exc)
            self._stop.wait(1.0 / self.rate)

    def _connected(self) -> bool:
        last = self.link.last_heartbeat if self.mock else self._last_heartbeat
        return last > 0 and time.monotonic() - last <= self.timeout

    def status(self) -> dict:
        armed = bool(self.link.armed) if self.mock else self._armed
        return {
            "connected": self._connected(),
            "armed": armed,
            "heartbeat_age": None if not self._connected() else round(time.monotonic() - (self.link.last_heartbeat if self.mock else self._last_heartbeat), 3),
            "transport": "mock" if self.mock else "mavlink-udp-v1",
            "endpoint": f"{self.target_ip}:{self.target_port}",
            "last_error": self._last_error,
        }

    def _require_connected(self):
        if not self._connected():
            raise RuntimeError("ROV heartbeat not received (connection timeout)")

    def arm(self, armed: bool) -> dict:
        self._require_connected()
        with self._lock:
            if self.mock:
                self.link.armed = armed
                self.link.last_command = {"command": MAV_CMD_COMPONENT_ARM_DISARM, "param1": int(armed)}
            else:
                self.link.mav.command_long_send(1, 1, MAV_CMD_COMPONENT_ARM_DISARM, 0, int(armed), 0, 0, 0, 0, 0, 0)
            return {"state": "armed" if armed else "disarmed", "command": MAV_CMD_COMPONENT_ARM_DISARM}

    def move(self, values: dict) -> dict:
        self._require_connected()
        if not self.mock and not self._is_armed_from_heartbeat():
            raise RuntimeError("ROV is not armed; call arm first")
        pwm = [_pwm(values.get(axis, 0.0)) for axis in CHANNELS]
        with self._lock:
            if self.mock:
                self.link.last_rc = pwm
            else:
                # MAVLink channels are 1-indexed; the vendor maps roll to
                # channel 7 and leaves channel 6 unused.
                channels = pwm[:5] + [UINT16_MAX, pwm[5], UINT16_MAX]
                args = [1, 1] + channels + [UINT16_MAX] * 10
                self.link.mav.rc_channels_override_send(*args)
        return {"state": "moving", "channels": dict(zip(CHANNELS, pwm))}

    def stop_motion(self) -> dict:
        return self.move({axis: 0 for axis in CHANNELS}) if self._connected() and (self.mock or self._is_armed_from_heartbeat()) else {"state": "stopped", "channels": {axis: 1500 for axis in CHANNELS}}

    def _is_armed_from_heartbeat(self) -> bool:
        return self._armed

    def get_tools(self):
        return [
            {"name": "rov_status", "type": "sensor", "description": "潜蛟 MAVLink 连接、心跳和解锁状态", "topic_out": [{"topic": "/qianjiao2_pro/status", "format": "data/json"}], "inputSchema": {"type": "object", "properties": {"action": {"type": "string", "enum": ["info", "start", "stop"]}}}},
            {"name": "rov_control", "type": "actuator", "description": "潜蛟 2.0 Pro 解锁及 6DOF 运动控制", "inputSchema": {"type": "object", "properties": {"action": {"type": "string", "enum": ["arm", "disarm", "move", "stop"]}, "heave": {"type": "number", "minimum": -1, "maximum": 1}, "pitch": {"type": "number", "minimum": -1, "maximum": 1}, "forward": {"type": "number", "minimum": -1, "maximum": 1}, "yaw": {"type": "number", "minimum": -1, "maximum": 1}, "lateral": {"type": "number", "minimum": -1, "maximum": 1}, "roll": {"type": "number", "minimum": -1, "maximum": 1}}, "required": ["action"]}},
        ]

    def dispatch(self, tool: str, args: dict) -> dict:
        action = args.get("action", "info")
        if tool == "rov_status":
            if action == "start": self.start()
            elif action == "stop": self.stop()
            return self.status()
        if action == "arm": return self.arm(True)
        if action == "disarm": return self.arm(False)
        if action == "move": return self.move(args)
        if action == "stop": return self.stop_motion()
        raise ValueError(f"unsupported action: {action}")
