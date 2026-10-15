"""PhanthyMotus hand card for the USB-RS485 L20 Lite."""

from __future__ import annotations

import threading
import time
import json
import os
import ssl
import urllib.request
from uuid import uuid4

from common.vendor_runtime import action_schema, tool
from modbus_client import JOINT_COUNT, L20LiteModbusClient, _bytes

JOINT_NAMES = [
    "thumb_root", "thumb_sway", "index_root", "middle_root", "ring_root",
    "pinky_root", "index_sway", "middle_sway", "ring_sway", "thumb_roll",
]
MOTION_ACTIONS = ("open", "close", "set_positions")
STANDARD_JOINT_NAMES = [
    "thumb_flex", "thumb_sway", "index_flex", "middle_flex", "ring_flex",
    "pinky_flex", "index_sway", "middle_sway", "ring_sway", "thumb_roll",
]
CHINESE_JOINT_NAMES = [
    "拇指弯曲", "拇指侧摆", "食指弯曲", "中指弯曲", "无名指弯曲",
    "小指弯曲", "食指侧摆", "中指侧摆", "无名指侧摆", "拇指旋转",
]
JOINT_FIELDS = {name: index for index, name in enumerate(STANDARD_JOINT_NAMES)}
STRUCTURED_POSITION_FIELDS = [
    "thumb_flex", "index_flex", "middle_flex", "ring_flex", "pinky_flex",
    "thumb_sway", "thumb_roll", "index_sway", "middle_sway", "ring_sway",
]
SPEED_RATIOS = {f"{ratio:.1f}x": ratio for ratio in (1.0, 1.5, 2.0, 2.5, 3.0)}

# These are reference values for this installation, not universal defaults.
# Each side must have its own explicit calibration in the deployment config.
# Values are 0..255 position units, not degrees.
RIGHT_OPEN_POSE = [254, 254, 254, 254, 254, 254, 127, 126, 127, 254]
RIGHT_FIST_POSE = [93, 255, 0, 0, 0, 0, 128, 128, 128, 255]


def _named(values: list[int]) -> dict[str, int]:
    return dict(zip(JOINT_NAMES, values))


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

    def get(self, side: str | None = None) -> L20LiteModbusClient:
        target_side = side or self.side
        if target_side not in ("left", "right"):
            raise ValueError("side must be left or right")
        with self._lock:
            if self._client is not None and self._client.side != target_side:
                self._client.close()
                self._client = None
            if self._client is None:
                self._client = L20LiteModbusClient(
                    self.port, target_side, baudrate=self.baudrate,
                    timeout_s=self.timeout_s,
                )
            return self._client

    def detect_sides(self) -> list[str]:
        """Probe only the two known slave IDs using read-only position frames."""
        with self._lock:
            client = self._client
        if client is None:
            client = self.get()
        found = []
        try:
            for side in ("right", "left"):
                try:
                    client.probe_positions(side)
                except TimeoutError:
                    continue
                found.append(side)
        except Exception:
            # A bad frame or disconnected adapter is not evidence of absence.
            self.close()
            raise
        if not found:
            # A USB adapter may have been unplugged and reinserted at this path.
            # Do not keep a stale file descriptor across the next probe.
            self.close()
        return found

    def close(self) -> None:
        with self._lock:
            client, self._client = self._client, None
        if client is not None:
            client.close()


class HandPlugin:
    PREFIX = "hand"

    def __init__(self, connection: L20LiteConnection, config: dict):
        self.connection = connection
        if connection.side == "right" and "fist_pose" not in config:
            raise ValueError("fist_pose must be explicitly calibrated and configured before enabling close")
        if connection.side == "right" and "open_pose" not in config:
            raise ValueError("open_pose must be explicitly calibrated and configured before enabling open")
        if connection.side == "left" and not {"left_open_pose", "left_fist_pose"} <= config.keys():
            raise ValueError("left-hand left_open_pose and left_fist_pose must be configured explicitly")
        self.poses_by_side = {}
        if {"open_pose", "fist_pose"} <= config.keys():
            self.poses_by_side["right"] = {
                "open": _bytes(config["open_pose"]),
                "close": _bytes(config["fist_pose"]),
            }
        if {"left_open_pose", "left_fist_pose"} <= config.keys():
            self.poses_by_side["left"] = {
                "open": _bytes(config["left_open_pose"]),
                "close": _bytes(config["left_fist_pose"]),
            }
        self.auto_detect_gestures = config.get("auto_detect_gestures", False)
        if not isinstance(self.auto_detect_gestures, bool):
            raise ValueError("auto_detect_gestures must be a boolean")
        self.motion_speed = self._setting(config.get("motion_speed", 60), "motion_speed")
        self.motion_torque = self._setting(config.get("motion_torque", 100), "motion_torque")
        self.initial_motion_torque = self.motion_torque
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
        self._state_lock = threading.Lock()
        self._active: dict | None = None
        self._last_result: dict | None = None
        self._completion_delivery_failed = False
        self._detected_sides: list[str] = []
        self._detection_error: str | None = None

    @staticmethod
    def _setting(value, name: str) -> int:
        if isinstance(value, bool) or not isinstance(value, int) or not 1 <= value <= 255:
            raise ValueError(f"{name} must be an integer in 1..255")
        return value

    def _motion_options(self, args: dict) -> tuple[int, bool]:
        ratio = args.get("speed_ratio", "1.0x")
        if not isinstance(ratio, str) or ratio not in SPEED_RATIOS:
            raise ValueError("speed_ratio must be one of 1.0x, 1.5x, 2.0x, 2.5x, 3.0x")
        hold = args.get("hold_torque", True)
        if isinstance(hold, str) and hold.lower() in ("true", "false"):
            hold = hold.lower() == "true"
        if not isinstance(hold, bool):
            raise ValueError("hold_torque must be a boolean")
        return min(255, max(1, round(self.motion_speed * SPEED_RATIOS[ratio]))), hold

    def get_tool(self) -> dict:
        schema = action_schema(
            {
                "info": ([], "只读检测已连接的左右手及数量，不驱动手指"),
                "get_state": ([], "读取 10 个关节位置和速度"),
                "open": (["speed_ratio", "hold_torque", "confirm_motion"], "展开到已校准姿态；可选倍速和到位泄力；每次运动需确认"),
                "close": (["speed_ratio", "hold_torque", "confirm_motion"], "握拳到已校准姿态；可选倍速和到位泄力；每次运动需确认"),
                "set_positions": ([*STRUCTURED_POSITION_FIELDS, "speed_ratio", "hold_torque", "confirm_motion"], "按需填写各关节位置；留空保持原位；每次运动需确认"),
                "set_torque": (["torque"], f"配置后续动作力矩；初始值 {self.initial_motion_torque}，重启后恢复；不立即驱动"),
                "release": ([], "立即将所有关节力矩设为 0"),
            },
            {
                **{
                    name: {"type": "integer", "minimum": 0, "maximum": 255,
                           "description": f"{CHINESE_JOINT_NAMES[JOINT_FIELDS[name]]}，留空保持当前位置"}
                    for name in STRUCTURED_POSITION_FIELDS
                },
                "speed_ratio": {"type": "string", "enum": list(SPEED_RATIOS),
                                "description": "相对当前设置速度的倍率，最高 255"},
                "hold_torque": {"type": "boolean", "enum": [True, False],
                                "description": "默认 true 保持原有行为；false 在到位后释放力矩"},
                "torque": {"type": "integer", "minimum": 1, "maximum": 255,
                           "default": self.initial_motion_torque,
                           "description": f"初始值 {self.initial_motion_torque}；仅影响后续动作"},
                "confirm_motion": {"type": "boolean", "const": True, "description": "确认手附近无人和障碍物后，每次运动显式输入 true"},
            },
        )
        schema["allOf"] = [{
            "if": {"properties": {"action": {"enum": list(MOTION_ACTIONS)}}},
            "then": {"required": ["confirm_motion"]},
        }]
        schema["x-is-dangerous"] = True
        schema["x-completion"] = {
            # Allow the motion deadline plus callback retries and Core registration.
            "actions": list(MOTION_ACTIONS), "timeout": int(self.motion_timeout_s) + 25,
        }
        schema["x-hooks"] = {"on_interrupt_motion": {"action": "stop"}}
        definition = tool(
            "hand", "actuator",
            "LinkerHand L20 Lite USB-RS485/Modbus 控制。每次运动必须显式传 confirm_motion=true。",
            schema,
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

    def _detect_sides(self) -> list[str]:
        try:
            sides = self.connection.detect_sides()
        except Exception as exc:
            with self._state_lock:
                self._detected_sides = []
                self._detection_error = str(exc)
            raise
        with self._state_lock:
            self._detected_sides = sides
            self._detection_error = None
        return sides

    def _move(self, target: list[int], action: str, *, side: str | None = None,
              speed: int | None = None, hold_torque: bool = True) -> dict:
        with self._motion_lock:
            if self._stop_requested.is_set():
                raise InterruptedError("hand motion stopped")
            client = self.connection.get(side) if side is not None else self.connection.get()
            active_side = side or self.connection.side
            actual_speed = self.motion_speed if speed is None else speed
            current = client.get_positions()  # fail closed if no valid reply
            if all(abs(a - b) <= self.position_tolerance for a, b in zip(current, target)):
                if not hold_torque:
                    client.set_torques([0] * JOINT_COUNT)
                return {"state": "already_at_target", "action": action, "side": active_side,
                        "positions": _named(current), "torque_released": not hold_torque}
            if self._stop_requested.is_set():
                raise InterruptedError("hand motion stopped")
            energized = False
            try:
                client.set_speeds([actual_speed] * JOINT_COUNT)
                energized = True
                client.set_torques([self.motion_torque] * JOINT_COUNT)
                if self._stop_requested.is_set():
                    raise InterruptedError("hand motion stopped")
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
                        if not hold_torque:
                            client.set_torques([0] * JOINT_COUNT)
                        return {
                            "state": "completed", "action": action, "side": active_side,
                            "requested_positions": _named(target), "positions": _named(measured),
                            "speed": actual_speed, "torque": self.motion_torque if hold_torque else 0,
                            "torque_released": not hold_torque,
                        }
                client.set_torques([0] * JOINT_COUNT)
                return {
                    "state": "released_timeout", "action": action, "side": active_side,
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

    def _acp_callback(self, action_id: str, status: str, result: dict) -> None:
        """Deliver one terminal result; retry only the notification, never motion."""
        url = os.environ.get("AGENT_CORE_URL", "https://localhost:15678").rstrip("/")
        context = ssl.create_default_context(cafile=os.environ.get("AGENT_CORE_CA_CERT") or None)
        if not os.environ.get("AGENT_CORE_CA_CERT") and url.startswith(
            ("https://localhost:", "https://127.0.0.1:")
        ):
            context.check_hostname = False
            context.verify_mode = ssl.CERT_NONE
        payload = json.dumps({
            "action_id": action_id, "status": status, "result": result,
            "tool": self.PREFIX, "ts": time.time(),
        }, ensure_ascii=False).encode()
        error = None
        for attempt in range(3):
            try:
                request = urllib.request.Request(
                    url + "/api/acp/complete", data=payload,
                    headers={"Content-Type": "application/json"}, method="POST",
                )
                with urllib.request.urlopen(request, timeout=5, context=context) as response:
                    acknowledgement = json.load(response)
                if acknowledgement.get("ok") is not True or acknowledgement.get("action_id") != action_id:
                    raise RuntimeError("Agent Core did not acknowledge action_id")
                return
            except Exception as exc:
                error = exc
                if attempt < 2:
                    time.sleep(0.5)
        raise RuntimeError(f"ACP completion delivery failed: {error}")

    def _target_for_action(self, active: dict, client: L20LiteModbusClient,
                           side: str) -> list[int]:
        if active["action"] == "set_positions":
            target = client.get_positions()
            for index, value in active["joint_positions"].items():
                target[index] = value
            return target
        return list(self.poses_by_side[side][active["action"]])

    def _run_motion(self, active: dict) -> None:
        action = active["action"]
        try:
            with self._motion_lock:
                if self._stop_requested.is_set():
                    raise InterruptedError("hand motion stopped")
                side = None
                if self.auto_detect_gestures:
                    sides = self._detect_sides()
                    if len(sides) != 1:
                        raise RuntimeError(f"expected exactly one hand, detected {sides}")
                    side = sides[0]
                if self._stop_requested.is_set():
                    raise InterruptedError("hand motion stopped")
                active_side = side or self.connection.side
                if active_side not in self.poses_by_side:
                    raise RuntimeError(f"{active_side}-hand motion requires explicit calibrated open and fist poses")
                client = self.connection.get(side) if side is not None else self.connection.get()
                target = self._target_for_action(active, client, active_side)
                result = self._move(target, action, side=side, speed=active["speed"],
                                    hold_torque=active["hold_torque"])
            status = "error" if result["state"] == "released_timeout" else "completed"
        except InterruptedError as exc:
            status, result = "cancelled", {"state": "cancelled", "message": str(exc)}
        except Exception as exc:
            status, result = "error", {"state": "error", "message": str(exc)}
        terminal = {"action_id": active["action_id"], "status": status, "result": result}
        with self._state_lock:
            self._last_result = terminal
        try:
            self._acp_callback(active["action_id"], status, result)
        except Exception as exc:
            # Fail closed: Core may still hold a pending barrier. Do not admit
            # another motion until the failed notification is investigated.
            with self._state_lock:
                self._completion_delivery_failed = True
                terminal["callback_error"] = str(exc)
            print(f"[ACP] {active['action_id']}: {exc}", flush=True)
        else:
            with self._state_lock:
                if self._active is active:
                    self._active = None

    def _start_motion(self, action: str, args: dict) -> dict:
        # Validate all user input before accepting an ACP action or opening serial.
        speed, hold_torque = self._motion_options(args)
        active = {"action": action, "action_id": f"hand_{action}_{uuid4().hex}",
                  "speed": speed, "hold_torque": hold_torque}
        if action == "set_positions":
            if args.get("positions") not in (None, ""):
                raise ValueError("positions CSV is no longer supported; use named joint fields")
            joint_positions = {}
            for field in STRUCTURED_POSITION_FIELDS:
                value = args.get(field)
                if value is not None and value != "":
                    joint_positions[JOINT_FIELDS[field]] = _bytes([value], expected=1)[0]
            if not joint_positions:
                raise ValueError("at least one joint position must be specified")
            active["joint_positions"] = joint_positions
        with self._state_lock:
            if self._completion_delivery_failed:
                raise RuntimeError("ACP completion delivery failed; restart driver after checking Core")
            if self._active is not None:
                raise RuntimeError("another hand motion is active")
            self._stop_requested.clear()
            self._active = active
            try:
                threading.Thread(target=self._run_motion, args=(active,),
                                 name=active["action_id"], daemon=True).start()
            except Exception:
                self._active = None
                raise
        return {"state": "running", "action_id": active["action_id"]}

    def dispatch(self, action: str, args: dict) -> dict | None:
        if action in MOTION_ACTIONS and args.get("confirm_motion") is not True:
            raise ValueError("confirm_motion must be true for every movement request")
        if action == "start":
            # Canvas lifecycle marker; opening a card must not move the hand.
            return {"state": "ready"}
        if action == "info":
            with self._state_lock:
                active_id = self._active["action_id"] if self._active else None
            if self.auto_detect_gestures and active_id is None:
                try:
                    with self._motion_lock:
                        self._detect_sides()
                except Exception:
                    pass  # Surface the detection error in info; never guess a side.
            with self._state_lock:
                last_result = self._last_result
                detected_sides = list(self._detected_sides)
                detection_error = self._detection_error
            detected_side = detected_sides[0] if len(detected_sides) == 1 else None
            label = {"left": "左手", "right": "右手"}.get(
                detected_side, "未检测到手" if not detected_sides else "同时检测到左右手",
            )
            return {
                "description": f"L20 Lite {label} USB-RS485/Modbus 控制卡片" if self.auto_detect_gestures
                               else f"L20 Lite {self.connection.side} USB-RS485/Modbus 控制卡片",
                "side": detected_side if self.auto_detect_gestures else self.connection.side,
                "detected_sides": detected_sides if self.auto_detect_gestures else [self.connection.side],
                "detected_count": len(detected_sides) if self.auto_detect_gestures else 1,
                "detection_error": detection_error if self.auto_detect_gestures else None,
                "serial_port": self.connection.port,
                "joint_names": JOINT_NAMES,
                "active_action_id": active_id,
                "last_result": last_result,
            }
        if action == "get_state":
            with self._motion_lock:
                if self.auto_detect_gestures:
                    sides = self._detect_sides()
                    if len(sides) != 1:
                        raise RuntimeError(f"expected exactly one hand, detected {sides}")
                    side = sides[0]
                else:
                    side = self.connection.side
                client = self.connection.get(side)
                positions = client.get_positions()
                try:
                    speeds = _named(client.get_speeds())
                except (TimeoutError, ValueError, RuntimeError) as exc:
                    speeds = None
                    speed_error = str(exc)
            result = {
                "state": "connected", "side": side,
                "serial_port": self.connection.port, "positions": _named(positions),
                "speeds": speeds, "timestamp_ms": int(time.time() * 1000),
            }
            if speeds is None:
                result["speed_error"] = speed_error
            return result
        if action in MOTION_ACTIONS:
            return self._start_motion(action, args)
        if action == "set_torque":
            torque = self._setting(args.get("torque"), "torque")
            with self._motion_lock:
                self.motion_torque = torque
            return {"state": "configured", "torque": torque, "applied": False}
        if action in ("stop", "release"):
            self._stop_requested.set()
            with self._motion_lock:
                # Canvas stop may arrive before the serial device was ever used.
                if action == "release" and self.auto_detect_gestures:
                    sides = self._detect_sides()
                    if len(sides) != 1:
                        raise RuntimeError(f"expected exactly one hand, detected {sides}")
                    client = self.connection.get(sides[0])
                else:
                    client = self.connection._client
                if client is None:
                    return {"state": "idle" if action == "stop" else "released", "torque_released": True}
                client.set_torques([0] * JOINT_COUNT)
                positions = client.get_positions()
            return {"state": "idle" if action == "stop" else "released", "side": client.side,
                    "positions": _named(positions), "torque_released": True}
        return None


def build_plugins(config: dict, _namespace: str, _ros2) -> list:
    connection = L20LiteConnection(config)
    hand_config = config.get("plugins", {}).get("hand", {})
    return [HandPlugin(connection, hand_config)] if hand_config.get("enabled", True) else []
