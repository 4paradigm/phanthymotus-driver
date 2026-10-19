"""Separate, confirmation-gated L20 Lite gesture card.

Pose values and collision-avoidance phases come from the colleague's
2026-10-19 Modbus prototype. This plugin deliberately does not modify hand.
"""

from __future__ import annotations

import json
import os
import ssl
import threading
import time
import urllib.request
from uuid import uuid4

from common.vendor_runtime import action_schema, tool
from modbus_client import JOINT_COUNT, _bytes
from modbus_device import L20LiteConnection, SPEED_RATIOS, _named


ACTIONS = ("ok_gesture", "thumb_up_gesture", "v_sign_gesture", "number", "grasp", "custom_grasp")
NUMBER_POSES = {
    0: [93, 255, 0, 0, 0, 0, 128, 128, 128, 255],
    1: [0, 255, 255, 0, 0, 0, 128, 128, 128, 255],
    2: [0, 255, 255, 255, 0, 0, 128, 128, 128, 255],
    3: [0, 255, 255, 255, 255, 0, 128, 128, 128, 255],
    4: [0, 255, 255, 255, 255, 255, 128, 128, 128, 255],
    5: [255, 255, 255, 255, 255, 255, 128, 128, 128, 255],
    6: [255, 255, 0, 0, 0, 255, 128, 128, 128, 255],
    7: [131, 32, 110, 110, 0, 0, 128, 128, 128, 218],
    8: [255, 255, 255, 0, 0, 0, 128, 128, 128, 255],
    9: [100, 81, 142, 0, 0, 0, 128, 128, 128, 172],
}
DEFAULT_POSES = {
    "ok_gesture": [132, 28, 123, 255, 255, 255, 128, 128, 128, 255],
    "thumb_up_gesture": [255, 255, 0, 0, 0, 0, 128, 128, 128, 255],
    "v_sign_gesture": [0, 255, 255, 255, 0, 0, 128, 128, 128, 255],
    "grasp_prep": [255, 0, 255, 255, 255, 255, 128, 128, 128, 255],
    "grasp_target": [0, 0, 0, 0, 0, 0, 128, 128, 128, 255],
}


class GesturePlugin:
    PREFIX = "hand_gesture"

    def __init__(self, hand, config: dict):
        self.hand = hand  # Reuse the existing read-only discovery and RS-485 bus lock.
        self.config = config
        self._lock = threading.RLock()
        self._bindings: dict[str, str] = {}
        self._connections: dict[str, L20LiteConnection] = {}
        self._active: dict | None = None
        self._stop = threading.Event()
        self._delivery_failed = False
        self._last_result: dict | None = None
        self._calibration = {}
        for side in ("left", "right"):
            poses = config.get(f"{side}_poses")
            if poses is None:
                continue
            missing = set(DEFAULT_POSES) - set(poses)
            if missing:
                raise ValueError(f"{side}_poses missing {sorted(missing)}")
            self._calibration[side] = {name: _bytes(poses[name]) for name in DEFAULT_POSES}
            numbers = config.get(f"{side}_number_poses")
            if numbers is None:
                raise ValueError(f"{side}_number_poses is required")
            if set(map(int, numbers)) != set(NUMBER_POSES):
                raise ValueError(f"{side}_number_poses must contain 0..9")
            self._calibration[side]["number"] = {int(n): _bytes(pose) for n, pose in numbers.items()}
        self.speed = self._setting(config.get("motion_speed", 85), "motion_speed")
        self.number_speed = self._setting(config.get("number_speed", config.get("count_speed", 85)), "number_speed")
        self.torque = self._setting(config.get("motion_torque", 100), "motion_torque")
        self.thumb_speed_scaling = config.get("thumb_speed_scaling", True)
        if not isinstance(self.thumb_speed_scaling, bool):
            raise ValueError("thumb_speed_scaling must be boolean")
        self.timeout = float(config.get("motion_timeout_s", 8.0))
        self.tolerance = int(config.get("position_tolerance", 6))
        self.poll = float(config.get("poll_interval_s", 0.2))
        if not 0.5 <= self.timeout <= 30 or not 0.02 <= self.poll <= 1 or not 0 <= self.tolerance <= 20:
            raise ValueError("invalid gesture motion timing or tolerance")

    @staticmethod
    def _setting(value, name):
        if isinstance(value, bool) or not isinstance(value, int) or not 1 <= value <= 255:
            raise ValueError(f"{name} must be an integer in 1..255")
        return value

    def get_tool(self) -> dict:
        schema = action_schema({
            "info": ([], "只读检测左右手及当前卡片绑定状态"),
            "ok_gesture": (["speed_ratio", "hold_torque", "confirm_motion"], "OK 手势：拇指与食指捏合，其余手指伸展"),
            "thumb_up_gesture": (["speed_ratio", "hold_torque", "confirm_motion"], "点赞手势：拇指伸展，四指收拢"),
            "v_sign_gesture": (["speed_ratio", "hold_torque", "confirm_motion"], "V 字手势：食指和中指伸展"),
            "number": (["number", "speed_ratio", "hold_torque", "confirm_motion"], "选择 0–9 中的一个数字，执行对应的数字手势"),
            "grasp": (["strength", "speed_ratio", "hold_torque", "confirm_motion"], "抓握：soft/standard/firm 力矩预设；仅用位置停滞估计接触，无触觉传感器"),
            "custom_grasp": (["torque", "speed_ratio", "hold_torque", "confirm_motion"], "抓握：指定 1–255 原始力矩上限；仅用位置停滞估计接触"),
            "config": (["bound_side"], "将这张卡片绑定到左手或右手；保存配置不会运动"),
        }, {
            "bound_side": {"type": "string", "enum": ["left", "right"]},
            "instance_id": {"type": "string"},
            "speed_ratio": {"type": "string", "enum": list(SPEED_RATIOS), "description": "速度倍率，最大 255"},
            "hold_torque": {"type": "boolean", "description": "到位后保留力矩；手势和数字默认 false，抓握默认 true"},
            "number": {"type": "integer", "enum": list(range(10)),
                       "description": "选择数字 0–9，执行该数字对应的手势"},
            "strength": {"type": "string", "enum": ["soft", "standard", "firm"], "default": "standard",
                         "description": "对应原始力矩上限 30/85/160；不保证对任何物体的夹持力"},
            "torque": {"type": "integer", "minimum": 1, "maximum": 255,
                       "description": "设备原始扭矩上限，不是牛顿或防夹保证"},
            "confirm_motion": {"type": "boolean", "const": True,
                               "description": "确认手附近无人和障碍物后，本次请求显式传入布尔值 true"},
        })
        schema["allOf"] = [
            {"if": {"properties": {"action": {"enum": list(ACTIONS)}}},
             "then": {"required": ["confirm_motion"]}},
            {"if": {"properties": {"action": {"const": "number"}}},
             "then": {"required": ["number"]}},
        ]
        schema["x-is-dangerous"] = True
        # Keep a completion barrier through any thumb/index clearance phases.
        schema["x-completion"] = {"actions": list(ACTIONS), "timeout": 300}
        schema["x-hooks"] = {"on_interrupt_motion": {"action": "stop"}}
        definition = tool(self.PREFIX, "actuator", "L20 Lite 独立手势卡片，动作需要安全确认", schema)
        definition["title"] = "L20 Lite 手势"
        definition["multiInstance"] = True
        definition["configSchema"] = {"type": "object", "properties": {
            "bound_side": {"type": "string", "enum": ["left", "right"], "scope": "instance",
                           "title": "绑定灵巧手", "description": "选择本卡片控制的左手或右手"}}}
        return definition

    def start(self):
        pass

    def stop(self):
        self._stop.set()
        with self.hand._bus_lock:
            for connection in self._connections.values():
                if connection._client is not None:
                    connection._client.set_torques([0] * JOINT_COUNT)
                connection.close()

    def _connection(self, instance_id, side, port):
        with self._lock:
            connection = self._connections.get(instance_id)
            if connection is not None and (connection.port != port or connection.side != side):
                connection.close()
                connection = None
            if connection is None:
                connection = L20LiteConnection({**self.hand.config, "side": side, "serial_port": port})
                self._connections[instance_id] = connection
            return connection

    def _check_stop(self):
        if self._stop.is_set():
            raise InterruptedError("gesture motion stopped")

    def _wait(self, seconds):
        if self._stop.wait(seconds):
            raise InterruptedError("gesture motion stopped")

    def _speed(self, ratio, base):
        if ratio not in SPEED_RATIOS:
            raise ValueError("speed_ratio must be 1.0x, 1.5x, 2.0x, 2.5x or 3.0x")
        return min(255, round(base * SPEED_RATIOS[ratio]))

    def _joint_speeds(self, base):
        speeds = [base] * JOINT_COUNT
        if self.thumb_speed_scaling:
            speeds[0] = max(1, round(base * 29.11 / 63.11))
            speeds[1] = max(1, round(base * 29.11 / 63.21))
            speeds[9] = max(1, round(base * 29.11 / 50.90))
        return speeds

    def _prepare(self, args, action):
        if args.get("confirm_motion") is not True:
            raise ValueError("confirm_motion must be true for every movement request")
        base = self.number_speed if action == "number" else self.speed
        speed = self._speed(args.get("speed_ratio", "1.0x"), base)
        hold = args.get("hold_torque", action in ("grasp", "custom_grasp"))
        if not isinstance(hold, bool):
            raise ValueError("hold_torque must be boolean")
        options = {"speed": speed, "hold": hold}
        if action == "number":
            number = args.get("number")
            # The canvas renders enum values in <select>, whose .value is
            # always a string even when the schema declares an integer.
            if isinstance(number, str) and len(number) == 1 and number in "0123456789":
                number = int(number)
            if type(number) is not int or not 0 <= number <= 9:
                raise ValueError("number must be a single digit in 0..9")
            options["number"] = number
        if action == "grasp":
            strength = args.get("strength", "standard")
            if strength not in ("soft", "standard", "firm"):
                raise ValueError("strength must be soft, standard or firm")
            options["strength"] = strength
            torque, base_speed = {"soft": (30, 45), "standard": (85, 85), "firm": (160, 110)}[strength]
            options["torque"] = torque
            options["speed"] = self._speed(args.get("speed_ratio", "1.0x"), base_speed)
        if action == "custom_grasp":
            torque = self._setting(args.get("torque"), "torque")
            options["torque"] = torque
            base_speed = 45 if torque <= 35 else 65 if torque <= 70 else 85
            options["speed"] = self._speed(args.get("speed_ratio", "1.0x"), base_speed)
        return options

    def _drive_and_wait(self, client, target, speed, *, tolerance=None, timeout=None):
        """Command a pose; fail closed if the verified position is not reached."""
        self._check_stop()
        client.set_positions(target)
        deadline = time.monotonic() + (self.timeout if timeout is None else timeout)
        measured = client.get_positions()
        limit = self.tolerance if tolerance is None else tolerance
        while True:
            self._check_stop()
            if all(abs(a - b) <= limit for a, b in zip(measured, target)):
                return measured
            if time.monotonic() >= deadline:
                raise TimeoutError("gesture target position not reached")
            self._wait(min(self.poll, 0.06))
            measured = client.get_positions()

    def _clearance(self, client, target, speed, poses):
        """Port the prototype's thumb/index clearance sequences."""
        current = client.get_positions()
        if target == poses["ok_gesture"]:
            if current[1] < 60 and current[0] < 200:
                lift = list(current)
                lift[0] = 255
                self._drive_and_wait(client, lift, speed, tolerance=20)
            prep = list(target)
            prep[0] = 255
            self._drive_and_wait(client, prep, speed, tolerance=20)
        elif target[1] < 60 and target[0] >= 100 and target[2] >= 90:
            prep = list(target)
            prep[0] = 240
            self._drive_and_wait(client, prep, speed, tolerance=20)
        elif current[1] < 60 and current[0] < 180 and current[2] >= 70 and target[0] >= 200 and target[1] >= 180:
            lift = list(current)
            lift[0] = 255
            self._drive_and_wait(client, lift, speed, tolerance=20)
        elif current[0] < 70 and current[2] > 40 and target[0] >= 80 and target[2] <= 40:
            prep = list(current)
            prep[0] = 93
            self._drive_and_wait(client, prep, speed, tolerance=20)
        elif current[0] >= 70 and current[2] < 100 and target[0] <= 50 and target[2] >= 100:
            prep = list(target)
            prep[0] = current[0]
            self._drive_and_wait(client, prep, speed, tolerance=20)
        elif current[0] < 120 and current[2] < 120 and target[0] > 180 and target[2] > 180:
            prep = list(target)
            prep[0] = current[0]
            self._drive_and_wait(client, prep, speed, tolerance=20)

    def _pose(self, client, target, speed, poses):
        self._clearance(client, target, speed, poses)
        return self._drive_and_wait(client, target, speed)

    def _grasp(self, client, poses, options):
        speed, torque = options["speed"], options["torque"]
        client.set_speeds(self._joint_speeds(speed))
        client.set_torques([85] * JOINT_COUNT)
        current = client.get_positions()
        if current[0] < 120 and current[2] < 120:
            free = list(current)
            for index in (2, 3, 4, 5):
                free[index] = 255
            self._drive_and_wait(client, free, speed, tolerance=20)
        prep = poses["grasp_prep"]
        self._pose(client, prep, speed, poses)
        client.set_torques([torque] * JOINT_COUNT)
        target = poses["grasp_target"]
        client.set_positions(target)
        deadline = time.monotonic() + max(self.timeout, 6 * 85 / speed)
        previous = client.get_positions()
        stable = 0
        contact_estimated = False
        while time.monotonic() < deadline:
            self._wait(min(self.poll, 0.06))
            measured = client.get_positions()
            if all(abs(a - b) <= 15 for a, b in zip(measured, target)):
                break
            stable = stable + 1 if max(abs(a - b) for a, b in zip(measured, previous)) <= 2 else 0
            if stable >= (3 if torque <= 35 else 4):
                contact_estimated = True
                break
            previous = measured
        else:
            raise TimeoutError("grasp did not reach target or settle")
        measured = client.get_positions()
        client.set_positions(measured)  # Stop driving toward a blocked target.
        return {"positions": _named(measured), "contact_estimated": contact_estimated,
                "contact_method": "position_stall_only", "torque": torque, "speed": speed}

    def _execute(self, active):
        action, side, port, options = active["action"], active["side"], active["port"], active["options"]
        try:
            with self.hand._bus_lock:
                self._check_stop()
                found = self.hand._discover()
                if found.get(side) != port:
                    raise RuntimeError(f"{side} hand moved or disconnected before motion")
                connection = self._connection(active["instance_id"], side, port)
                client = connection.get(side)
                client.probe_positions(side)
                poses = self._calibration[side]
                try:
                    if action in ("grasp", "custom_grasp"):
                        result = self._grasp(client, poses, options)
                    else:
                        client.set_speeds(self._joint_speeds(options["speed"]))
                        client.set_torques([self.torque] * JOINT_COUNT)
                        if action == "number":
                            number = options["number"]
                            position = self._pose(client, poses["number"][number], options["speed"], poses)
                            result = {"number": number, "positions": _named(position),
                                      "speed": options["speed"]}
                        else:
                            position = self._pose(client, poses[action], options["speed"], poses)
                            result = {"positions": _named(position), "speed": options["speed"]}
                    if not options["hold"]:
                        client.set_torques([0] * JOINT_COUNT)
                    result.update(state="completed", action=action, side=side,
                                  torque_released=not options["hold"])
                except Exception:
                    try:
                        client.set_torques([0] * JOINT_COUNT)
                    except Exception:
                        pass
                    raise
            status = "completed"
        except InterruptedError as exc:
            status, result = "cancelled", {"state": "cancelled", "message": str(exc)}
        except Exception as exc:
            status, result = "error", {"state": "error", "message": str(exc)}
        terminal = {"action_id": active["action_id"], "status": status, "result": result}
        with self._lock:
            self._last_result = terminal
        try:
            self._acp_callback(active["action_id"], status, result)
        except Exception as exc:
            with self._lock:
                self._delivery_failed = True
                terminal["callback_error"] = str(exc)
            print(f"[ACP] {active['action_id']}: {exc}", flush=True)
        else:
            with self._lock:
                if self._active is active:
                    self._active = None

    def _acp_callback(self, action_id, status, result):
        url = os.environ.get("AGENT_CORE_URL", "https://localhost:15678").rstrip("/")
        context = ssl.create_default_context(cafile=os.environ.get("AGENT_CORE_CA_CERT") or None)
        if not os.environ.get("AGENT_CORE_CA_CERT") and url.startswith(("https://localhost:", "https://127.0.0.1:")):
            context.check_hostname = False
            context.verify_mode = ssl.CERT_NONE
        payload = json.dumps({"action_id": action_id, "status": status, "result": result,
                              "tool": self.PREFIX, "ts": time.time()}, ensure_ascii=False).encode()
        for attempt in range(3):
            try:
                request = urllib.request.Request(url + "/api/acp/complete", data=payload,
                                                 headers={"Content-Type": "application/json"}, method="POST")
                with urllib.request.urlopen(request, timeout=5, context=context) as response:
                    acknowledgement = json.load(response)
                if acknowledgement.get("ok") is not True or acknowledgement.get("action_id") != action_id:
                    raise RuntimeError("Agent Core did not acknowledge action_id")
                return
            except Exception:
                if attempt == 2:
                    raise
                time.sleep(0.5)

    def dispatch(self, action, args):
        instance_id = args.get("instance_id") or ""
        if action == "config":
            side = args.get("bound_side")
            if not isinstance(instance_id, str) or not instance_id or side not in ("left", "right"):
                raise ValueError("instance_id and bound_side (left/right) are required")
            with self._lock:
                if self._active is not None:
                    raise RuntimeError("cannot change binding during gesture motion")
                if any(owner != instance_id and bound == side for owner, bound in self._bindings.items()):
                    raise ValueError(f"{side} hand is already bound to another gesture card")
                self._bindings[instance_id] = side
            return {"state": "configured", "instance_id": instance_id, "bound_side": side}
        if action == "stop":
            with self._lock:
                if self._active is not None and self._active["instance_id"] != instance_id:
                    return {"state": "idle", "torque_released": False}
            self._stop.set()
            with self.hand._bus_lock:
                connection = self._connections.get(instance_id)
                if connection is not None and connection._client is not None:
                    connection._client.set_torques([0] * JOINT_COUNT)
            return {"state": "idle", "torque_released": True}
        if action == "info":
            found = self.hand._discover()
            side = self._bindings.get(instance_id)
            return {"state": "connected" if side in found else "unbound" if side is None else "offline",
                    "bound_side": side, "detected_sides": list(found), "detected_count": len(found),
                    "serial_port": found.get(side), "calibrated_sides": list(self._calibration),
                    "active_action_id": self._active["action_id"] if self._active else None,
                    "last_result": self._last_result}
        if action == "start":
            return {"state": "ready" if self._bindings.get(instance_id) else "unbound",
                    "bound_side": self._bindings.get(instance_id)}
        if action not in ACTIONS:
            return None
        options = self._prepare(args, action)  # Validate before opening serial.
        side = self._bindings.get(instance_id)
        found = self.hand._discover()
        if side is None:
            if len(found) != 1:
                raise RuntimeError("select left or right in instance config when zero or two hands are detected")
            side = next(iter(found))
        if side not in self._calibration:
            raise RuntimeError(f"{side} hand has no explicitly calibrated gesture poses")
        port = found.get(side)
        if port is None:
            raise RuntimeError(f"{side} hand is not connected")
        active = {"action": action, "action_id": f"hand_gesture_{action}_{uuid4().hex}",
                  "instance_id": instance_id, "side": side, "port": port, "options": options}
        with self._lock:
            if self._delivery_failed:
                raise RuntimeError("ACP completion delivery failed; inspect Core and restart driver")
            if self._active is not None:
                raise RuntimeError("another gesture motion is active")
            self._stop.clear()
            self._active = active
            try:
                threading.Thread(target=self._execute, args=(active,), name=active["action_id"], daemon=True).start()
            except Exception:
                self._active = None
                raise
        return {"state": "running", "action_id": active["action_id"]}
