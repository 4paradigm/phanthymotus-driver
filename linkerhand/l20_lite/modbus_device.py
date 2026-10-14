"""PhanthyMotus hand card for the USB-RS485 L20 Lite."""

from __future__ import annotations

import threading
import time
import re

from common.vendor_runtime import action_schema, tool
from modbus_client import JOINT_COUNT, L20LiteModbusClient, _bytes

JOINT_NAMES = [
    "thumb_root", "thumb_sway", "index_root", "middle_root", "ring_root",
    "pinky_root", "index_sway", "middle_sway", "ring_sway", "thumb_roll",
]

# Open pose was captured from the connected RIGHT hand. The fist pose is the
# user's requested target, not a hardware-verified reachable pose. Values are
# 0..255 position units, not degrees; config.yaml can override either pose.
RIGHT_OPEN_POSE = [254, 254, 254, 254, 254, 254, 127, 126, 127, 254]
RIGHT_FIST_POSE = [93, 255, 0, 0, 0, 0, 128, 128, 128, 255]


def _named(values: list[int]) -> dict[str, int]:
    return dict(zip(JOINT_NAMES, values))


def _positions_from_card(value) -> list[int]:
    """Accept a 10-value list or the CSV text emitted by the canvas form."""
    if isinstance(value, str):
        parts = re.split(r"[\s,]+", value.strip())
        if not parts or any(not re.fullmatch(r"\d+", part) for part in parts):
            raise ValueError("positions must be 10 comma-separated integers in 0..255")
        value = [int(part) for part in parts]
    return _bytes(value)


class L20LiteConnection:
    """Open the serial device on first use, never on server startup."""

    def __init__(self, config: dict):
        self.port = str(config.get("serial_port", "/dev/ttyUSB0"))
        self.side = str(config.get("side", "right")).lower()
        if self.side not in ("right", "left"):
            raise ValueError("side must be right or left")
        self.baudrate = int(config.get("baudrate", 115200))
        self.timeout_s = float(config.get("response_timeout_s", 0.5))
        self._client: L20LiteModbusClient | None = None
        self._lock = threading.Lock()

    def get(self) -> L20LiteModbusClient:
        with self._lock:
            if self._client is None:
                self._client = L20LiteModbusClient(
                    self.port, self.side, baudrate=self.baudrate,
                    timeout_s=self.timeout_s,
                )
            return self._client

    def close(self) -> None:
        with self._lock:
            client, self._client = self._client, None
        if client is not None:
            client.close()


class HandPlugin:
    def __init__(self, connection: L20LiteConnection, config: dict):
        self.connection = connection
        if connection.side != "right" and ("open_pose" not in config or "fist_pose" not in config):
            raise ValueError("left-hand open_pose and fist_pose must be configured explicitly")
        self.open_pose = _bytes(config.get("open_pose", RIGHT_OPEN_POSE))
        self.fist_pose = _bytes(config.get("fist_pose", RIGHT_FIST_POSE))
        self.motion_speed = self._setting(config.get("motion_speed", 60), "motion_speed")
        self.motion_torque = self._setting(config.get("motion_torque", 100), "motion_torque")
        self.motion_timeout_s = float(config.get("motion_timeout_s", 8.0))
        self.position_tolerance = int(config.get("position_tolerance", 6))
        self.poll_interval_s = float(config.get("poll_interval_s", 0.2))
        if not 0.5 <= self.motion_timeout_s <= 30:
            raise ValueError("motion_timeout_s must be in 0.5..30 seconds")
        if not 0 <= self.position_tolerance <= 20:
            raise ValueError("position_tolerance must be in 0..20")
        if not 0.02 <= self.poll_interval_s <= 1:
            raise ValueError("poll_interval_s must be in 0.02..1 seconds")
        self._motion_lock = threading.RLock()
        self._stop_requested = threading.Event()

    @staticmethod
    def _setting(value, name: str) -> int:
        if isinstance(value, bool) or not isinstance(value, int) or not 1 <= value <= 255:
            raise ValueError(f"{name} must be an integer in 1..255")
        return value

    def get_tool(self) -> dict:
        definition = tool(
            "hand", "actuator",
            "LinkerHand L20 Lite USB-RS485/Modbus 控制。动作会驱动真实机械手；执行前需获得用户确认。",
            action_schema(
                {
                    "get_state": ([], "读取 10 个关节位置和速度"),
                    "open": ([], "展开到已验证的右手姿态"),
                    "close": ([], "握拳到用户指定的右手姿态（尚未实机验证）"),
                    "set_joint": (["joint", "position"], "设置单关节位置 0..255"),
                    "set_positions": (["positions"], "按关节顺序设置 10 个位置；画布输入逗号分隔值"),
                    "set_speed": (["speed"], "配置后续动作速度，不立即驱动"),
                    "set_torque": (["torque"], "配置后续动作力矩，不立即驱动"),
                    "release": ([], "立即将所有关节力矩设为 0"),
                },
                {
                    "joint": {"type": "string", "enum": JOINT_NAMES},
                    "position": {"type": "integer", "minimum": 0, "maximum": 255},
                    "positions": {"type": "string", "description": "10 个位置，逗号分隔，如 254,254,234,254,254,254,127,126,127,254"},
                    "speed": {"type": "integer", "minimum": 1, "maximum": 255},
                    "torque": {"type": "integer", "minimum": 1, "maximum": 255},
                },
            ),
        )
        definition["title"] = "L20 Lite 灵巧手"
        return definition

    def start(self) -> None:
        pass

    def stop(self) -> None:
        self._stop_requested.set()
        try:
            with self._motion_lock:
                # Shutdown must not turn a disconnected device on.
                if self.connection._client is not None:
                    self.connection._client.set_torques([0] * JOINT_COUNT)
        finally:
            self.connection.close()

    def _move(self, target: list[int], action: str) -> dict:
        with self._motion_lock:
            self._stop_requested.clear()
            client = self.connection.get()
            current = client.get_positions()  # fail closed if no valid reply
            if all(abs(a - b) <= self.position_tolerance for a, b in zip(current, target)):
                return {"state": "already_at_target", "action": action, "positions": _named(current)}
            energized = False
            try:
                client.set_speeds([self.motion_speed] * JOINT_COUNT)
                energized = True
                client.set_torques([self.motion_torque] * JOINT_COUNT)
                halfway = [(a + b) // 2 for a, b in zip(current, target)]
                client.set_positions(halfway)
                time.sleep(0.8)
                if self._stop_requested.is_set():
                    raise InterruptedError("hand motion stopped")
                client.set_positions(target)
                deadline = time.monotonic() + self.motion_timeout_s
                measured = current
                while time.monotonic() < deadline:
                    if self._stop_requested.wait(self.poll_interval_s):
                        raise InterruptedError("hand motion stopped")
                    measured = client.get_positions()
                    if all(abs(a - b) <= self.position_tolerance for a, b in zip(measured, target)):
                        return {
                            "state": "completed", "action": action, "side": self.connection.side,
                            "requested_positions": _named(target), "positions": _named(measured),
                            "speed": self.motion_speed, "torque": self.motion_torque,
                        }
                client.set_torques([0] * JOINT_COUNT)
                return {
                    "state": "released_timeout", "action": action, "side": self.connection.side,
                    "requested_positions": _named(target), "positions": _named(measured),
                    "torque_released": True,
                }
            except Exception:
                if energized:
                    try:
                        client.set_torques([0] * JOINT_COUNT)
                    except Exception:
                        pass
                raise

    def dispatch(self, action: str, args: dict) -> dict | None:
        if action == "start":
            # Canvas lifecycle marker; opening a card must not move the hand.
            return {"state": "ready"}
        if action == "info":
            return {
                "description": f"L20 Lite {self.connection.side} USB-RS485/Modbus 控制卡片",
                "side": self.connection.side,
                "serial_port": self.connection.port,
                "joint_names": JOINT_NAMES,
            }
        if action == "get_state":
            with self._motion_lock:
                client = self.connection.get()
                positions = client.get_positions()
                try:
                    speeds = _named(client.get_speeds())
                except (TimeoutError, ValueError, RuntimeError) as exc:
                    speeds = None
                    speed_error = str(exc)
            result = {
                "state": "connected", "side": self.connection.side,
                "serial_port": self.connection.port, "positions": _named(positions),
                "speeds": speeds, "timestamp_ms": int(time.time() * 1000),
            }
            if speeds is None:
                result["speed_error"] = speed_error
            return result
        if action == "open":
            return self._move(self.open_pose, "open")
        if action == "close":
            return self._move(self.fist_pose, "close")
        if action == "set_joint":
            joint = args.get("joint")
            if joint not in JOINT_NAMES:
                raise ValueError(f"unknown joint: {joint}")
            value = _bytes([args.get("position")], expected=1)[0]
            with self._motion_lock:
                target = self.connection.get().get_positions()
                target[JOINT_NAMES.index(joint)] = value
                return self._move(target, "set_joint")
        if action == "set_positions":
            return self._move(_positions_from_card(args.get("positions")), "set_positions")
        if action == "set_speed":
            speed = self._setting(args.get("speed"), "speed")
            with self._motion_lock:
                self.motion_speed = speed
            return {"state": "configured", "speed": speed, "applied": False}
        if action == "set_torque":
            torque = self._setting(args.get("torque"), "torque")
            with self._motion_lock:
                self.motion_torque = torque
            return {"state": "configured", "torque": torque, "applied": False}
        if action in ("stop", "release"):
            self._stop_requested.set()
            with self._motion_lock:
                # Canvas stop may arrive before the serial device was ever used.
                client = self.connection._client
                if client is None:
                    return {"state": "idle" if action == "stop" else "released", "torque_released": True}
                client.set_torques([0] * JOINT_COUNT)
                positions = client.get_positions()
            return {"state": "idle" if action == "stop" else "released", "positions": _named(positions), "torque_released": True}
        return None


def build_plugins(config: dict, _namespace: str, _ros2) -> list:
    connection = L20LiteConnection(config)
    hand_config = config.get("plugins", {}).get("hand", {})
    return [HandPlugin(connection, hand_config)] if hand_config.get("enabled", True) else []
