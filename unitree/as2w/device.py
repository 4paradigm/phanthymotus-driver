"""Unitree AS2W driver plugins (official AS2 SDK SportClient)."""
import copy
import json
import math
import struct
import threading
import time

from std_msgs.msg import String
from unitree_sdk2py.core.channel import ChannelSubscriber
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowState_, SportModeState_


def _number(value, default=0.0):
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def _bounded_argument(args, name, minimum, maximum):
    value = args.get(name, 0)
    if isinstance(value, bool):
        raise ValueError(f"{name} must be a number")
    try:
        value = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name} must be a number") from exc
    if not math.isfinite(value) or value < minimum or value > maximum:
        raise ValueError(f"{name} must be finite and between {minimum} and {maximum}")
    return value


def _integer_argument(args, name, allowed):
    value = args.get(name, 0)
    if isinstance(value, bool) or not isinstance(value, int) or value not in allowed:
        choices = ", ".join(str(item) for item in sorted(allowed))
        raise ValueError(f"{name} must be one of: {choices}")
    return value


_REMOTE_BUTTONS_BYTE2 = (
    ("LT", 5), ("RT", 4), ("back", 3), ("start", 2), ("LB", 1), ("RB", 0),
)
_REMOTE_BUTTONS_BYTE3 = (
    ("left", 7), ("down", 6), ("right", 5), ("up", 4),
    ("Y", 3), ("X", 2), ("B", 1), ("A", 0),
)
_REMOTE_AXIS_DEADZONE = 0.1
_REMOTE_STALE_AFTER = 0.5


def _parse_wireless_remote(raw):
    """Decode AS2W LowState_.wireless_remote using the vendored AS2 SDK layout."""
    if raw is None or len(raw) < 24:
        return {"available": False, "fresh": False}

    buttons = {
        name: bool(int(raw[byte_index]) >> bit & 1)
        for byte_index, definitions in ((2, _REMOTE_BUTTONS_BYTE2), (3, _REMOTE_BUTTONS_BYTE3))
        for name, bit in definitions
    }
    axes = {
        name: round(struct.unpack("f", bytes(raw[offset:offset + 4]))[0], 4)
        for name, offset in (("lx", 4), ("rx", 8), ("ry", 12), ("ly", 20))
    }
    return {
        "available": True,
        "fresh": True,
        "buttons": buttons,
        "axes": axes,
        "active": any(buttons.values()) or any(
            abs(value) > _REMOTE_AXIS_DEADZONE for value in axes.values()
        ),
    }


class _StateNode:
    _REMOTE_INTERVAL = 0.1

    def __init__(self, namespace, executor):
        from rclpy.node import Node
        self.node = Node("as2w_state")
        self.imu = self.node.create_publisher(String, f"/{namespace}/state/imu", 10)
        self.joints = self.node.create_publisher(String, f"/{namespace}/state/joints", 10)
        self.joint_state = self.node.create_publisher(String, f"/{namespace}/state/joint_state", 10)
        self.battery = self.node.create_publisher(String, f"/{namespace}/state/battery", 10)
        self.remote_controller = self.node.create_publisher(
            String, f"/{namespace}/state/remote_controller", 10
        )
        self.loco = self.node.create_publisher(String, f"/{namespace}/loco/state", 10)
        self._last_remote_time = 0.0
        self._last_remote = None
        self._remote_lock = threading.Lock()
        self._low = ChannelSubscriber("rt/lowstate", LowState_)
        self._sport = ChannelSubscriber("rt/sportmodestate", SportModeState_)
        self._low.Init(self._on_low, 10)
        self._sport.Init(self._on_sport, 10)
        executor.add_node(self.node)

    def close(self):
        for subscriber in (self._low, self._sport):
            try:
                subscriber.Close()
            except Exception:
                pass
        self.node.destroy_node()

    def _publish(self, publisher, value):
        message = String()
        message.data = json.dumps(value, separators=(",", ":"))
        publisher.publish(message)

    def _on_low(self, msg):
        imu = getattr(msg, "imu_state", getattr(msg, "imu", None))
        if imu is not None:
            self._publish(self.imu, {"quaternion": list(getattr(imu, "quaternion", [])),
                                     "gyroscope": list(getattr(imu, "gyroscope", [])),
                                     "accelerometer": list(getattr(imu, "accelerometer", [])),
                                     "rpy": list(getattr(imu, "rpy", []))})
        motors = getattr(msg, "motor_state", getattr(msg, "motor_states", []))
        motors = list(motors)[:len(_AS2_JOINT_NAMES)]
        states = [{"idx": i, "q": _number(getattr(m, "q", 0)),
                   "dq": _number(getattr(m, "dq", 0)),
                   "tau": _number(getattr(m, "tau_est", getattr(m, "tau", 0))),
                   "temperature": int(getattr(m, "temperature", 0))} for i, m in enumerate(motors)]
        self._publish(self.joint_state, {"joint_states": states})
        self._publish(self.joints, {"joints": [{"idx": s["idx"], "name": _AS2_JOINT_NAMES[s["idx"]], "q": s["q"]}
                                               for s in states[:len(_AS2_JOINT_NAMES)]],
                                    "imu_quat": list(getattr(imu, "quaternion", [])) if imu else []})
        bms = getattr(msg, "bms_state", None)
        if bms is not None:
            self._publish(self.battery, {"soc": int(getattr(bms, "soc", 0)),
                                         "current": _number(getattr(bms, "current", 0)),
                                         "cycle": int(getattr(bms, "cycle", 0))})
        now = time.monotonic()
        if self._last_remote is None or now - self._last_remote_time >= self._REMOTE_INTERVAL:
            remote = _parse_wireless_remote(getattr(msg, "wireless_remote", None))
            remote["timestamp_ms"] = int(time.time() * 1000)
            with self._remote_lock:
                self._last_remote_time = now
                self._last_remote = remote
            self._publish(self.remote_controller, remote)

    def _on_sport(self, msg):
        imu = getattr(msg, "imu_state", None)
        self._publish(self.loco, {"mode": int(getattr(msg, "mode", 0)),
                                  "velocity": list(getattr(msg, "velocity", [])),
                                  "position": list(getattr(msg, "position", [])),
                                  "body_height": _number(getattr(msg, "body_height", 0)),
                                  "imu_rpy": list(getattr(imu, "rpy", [])) if imu else []})

    @property
    def last_remote(self):
        with self._remote_lock:
            remote = copy.deepcopy(self._last_remote)
            last_remote_time = self._last_remote_time
        if remote is not None and remote["available"]:
            remote["fresh"] = time.monotonic() - last_remote_time <= _REMOTE_STALE_AFTER
        return remote


class StatePlugin:
    PREFIX = "state"
    def __init__(self, config, namespace, executor):
        self._namespace = namespace
        self._state = _StateNode(namespace, executor)
    def get_tools(self):
        specs = (("imu", "state/imu", "data/json", "AS2 IMU state"),
                 ("joints", "state/joints", "sensor/skeleton", "AS2W 16-joint skeleton for model animation"),
                 ("joint_state", "state/joint_state", "data/json", "AS2 raw motor position, velocity, torque, and temperature"),
                 ("battery", "state/battery", "data/json", "AS2 BMS state"),
                 ("loco_state", "loco/state", "data/json", "AS2 high-level locomotion state"),
                 ("remote_controller", "state/remote_controller", "data/json",
                  "AS2W wireless remote controller: 14 buttons and 4 axes"))
        return [{"name": name, "type": "sensor", "multiInstance": False, "description": desc,
                 "inputSchema": {"type": "object", "properties": {}},
                 "topic_out": [{"topic": f"/{self._namespace}/{path}", "format": fmt}]}
                for name, path, fmt, desc in specs]
    def start(self): pass
    def stop(self): self._state.close()
    def dispatch(self, action, args):
        if action == "stop":
            return {"state": "idle"}
        if action == "read" and args.get("_tool_name") == "remote_controller":
            return {"state": "running", "data": self._state.last_remote or {"available": False}}
        if action == "info":
            tool_name = args.get("_tool_name")
        elif action in ("imu", "joints", "joint_state", "battery", "loco_state", "remote_controller"):
            tool_name = action
        else:
            tool_name = None
        if tool_name:
            tool = next((item for item in self.get_tools() if item["name"] == tool_name), None)
            if tool is not None:
                return {"state": "running", "topic_out": tool["topic_out"]}
        if action in ("start", "info"):
            return {"state": "running"}
        return None


class LocoPlugin:
    PREFIX = "loco"
    _ROLL_LIMITS = (-0.2, 0.2)
    _PITCH_LIMITS = (-0.3, 0.3)
    _YAW_LIMITS = (-0.3, 0.3)
    _BODY_HEIGHT_LIMITS = (-0.3, 0.3)
    _BODY_POSITION_LIMITS = (-0.2, 0.2)
    _GAIT_TYPES = {0}
    def __init__(self, config, namespace, executor, proxy):
        self.proxy = proxy
        self._lock = threading.Lock()
        self._stop = None
        self._stop_external_motion = None
        self._motion_executor = None
    def set_external_motion_stop(self, callback):
        self._stop_external_motion = callback
    def set_motion_executor(self, executor):
        self._motion_executor = executor
        self._stop_external_motion = executor.stop
    def get_tool(self):
        actions = ["move", "timed_move", "stop_move", "stand_up", "stand_down", "balance_stand", "recovery_stand", "damp", "euler", "speed_level", "body_height", "body_position", "switch_gait", "switch_joystick", "left_side_gait", "right_side_gait", "auto_recovery", "get_state"]
        return {"name": "loco", "type": "actuator", "multiInstance": False,
                "description": "Unitree AS2 locomotion via SportClient", "inputSchema": {"type": "object", "properties": {
                    "action": {"type": "string", "enum": actions},
                    "vx": {"type": "number", "minimum": -1.5, "maximum": 1.5},
                    "vy": {"type": "number", "minimum": -1.0, "maximum": 1.0},
                    "vyaw": {"type": "number", "minimum": -2.0, "maximum": 2.0},
                    "duration": {"type": "number", "minimum": -1, "maximum": 30},
                    "roll": {"type": "number", "minimum": -0.2, "maximum": 0.2},
                    "pitch": {"type": "number", "minimum": -0.3, "maximum": 0.3},
                    "yaw": {"type": "number", "minimum": -0.3, "maximum": 0.3},
                    "level": {"type": "integer", "enum": [-1, 0, 1]},
                    "height": {"type": "number", "minimum": -0.3, "maximum": 0.3},
                    "x": {"type": "number", "minimum": -0.2, "maximum": 0.2},
                    "y": {"type": "number", "minimum": -0.2, "maximum": 0.2},
                    "z": {"type": "number", "minimum": -0.2, "maximum": 0.2},
                    "flag": {"type": "boolean"}}, "required": ["action"],
                "x-completion": {"actions": ["timed_move"], "timeout": 40},
                "x-action-params": {
                    "move": {"params": ["vx", "vy", "vyaw", "duration"], "description": "Set velocity once; duration=-1 streams continuously and duration=0 stops. Use timed_move for a finite action."},
                    "timed_move": {"params": ["vx", "vy", "vyaw", "duration"], "required": ["duration"], "description": "Run a finite, cancellable velocity action for more than 0 and at most 30 seconds."},
                    "stop_move": {"params": [], "description": "Stop movement."},
                    "stand_up": {"params": [], "description": "Stand up."}, "stand_down": {"params": [], "description": "Stand down."},
                    "balance_stand": {"params": [], "description": "Balance stand."}, "recovery_stand": {"params": [], "description": "Recovery stand."},
                    "damp": {"params": [], "description": "Damp motors."}, "euler": {"params": ["roll", "pitch", "yaw"], "description": "Set body attitude."},
                    "speed_level": {"params": ["level"], "description": "Set speed level."}, "body_height": {"params": ["height"], "description": "Set body height."},
                    "body_position": {"params": ["x", "y", "z", "yaw"], "description": "Set body position."}, "switch_gait": {"params": ["level"], "description": "Switch gait."},
                    "switch_joystick": {"params": ["flag"], "description": "Enable or disable joystick."}, "left_side_gait": {"params": ["flag"], "description": "Enable left-side gait."},
                    "right_side_gait": {"params": ["flag"], "description": "Enable right-side gait."}, "auto_recovery": {"params": ["flag"], "description": "Enable or disable auto recovery."},
                    "get_state": {"params": [], "description": "Read sport state."}}}}
    def start(self): pass
    def stop(self):
        if self._motion_executor:
            self._motion_executor.stop(self.PREFIX)
        self._stop_continuous()
        self.proxy.StopMove()
    def interrupt_motion(self):
        if self._motion_executor:
            self._motion_executor.stop(self.PREFIX)
        self._stop_continuous()
        return self.proxy.StopMove()
    def _stop_continuous(self):
        event = self._stop
        self._stop = None
        if event: event.set()
    def _continuous(self, vx, vy, yaw):
        self._stop_continuous(); event = threading.Event(); self._stop = event
        def run():
            while not event.is_set():
                if self.proxy.Move(vx, vy, yaw) != 0: break
                event.wait(.1)
        threading.Thread(target=run, daemon=True).start()
    def _timed_move(self, vx, vy, yaw, duration):
        if self._motion_executor is None:
            return {"error": "timed motion executor is unavailable", "code": "UNAVAILABLE"}
        def worker(stop_event):
            started = time.monotonic()
            samples = 0
            while not stop_event.is_set():
                elapsed = time.monotonic() - started
                if elapsed >= duration:
                    break
                ret = self.proxy.Move(vx, vy, yaw)
                if ret != 0:
                    return {"state": "failed", "ret": ret, "samples": samples}
                samples += 1
                stop_event.wait(0.1)
            return {"state": "cancelled" if stop_event.is_set() else "completed",
                    "samples": samples, "duration": round(time.monotonic() - started, 3),
                    "vx": vx, "vy": vy, "vyaw": yaw}
        result = self._motion_executor.start(self.PREFIX, worker)
        if "error" not in result:
            result.update({"vx": vx, "vy": vy, "vyaw": yaw, "duration": duration})
        return result
    def dispatch(self, action, args):
        if action in ("start", "info"): return {"state": "ready"}
        if action == "stop":
            if self._stop_external_motion:
                self._stop_external_motion()
            self._stop_continuous()
            return {"state": "idle", "ret": self.proxy.StopMove()}
        if action != "get_state" and self._stop_external_motion:
            self._stop_external_motion()
        if action in ("move", "timed_move"):
            try:
                vx = _bounded_argument(args, "vx", -1.5, 1.5)
                vy = _bounded_argument(args, "vy", -1.0, 1.0)
                yaw = _bounded_argument(args, "vyaw", -2.0, 2.0)
                duration = args.get("duration")
                if duration is not None:
                    duration = _bounded_argument(args, "duration", -1.0, 30.0)
            except ValueError as exc:
                return {"error": str(exc), "code": "INVALID_ARGUMENT"}
            if action == "timed_move":
                if duration is None or duration <= 0:
                    return {"error": "timed_move requires duration greater than 0",
                            "code": "INVALID_ARGUMENT"}
                self._stop_continuous()
                return self._timed_move(vx, vy, yaw, duration)
            if duration is None: return {"ret": self.proxy.Move(vx, vy, yaw), "vx": vx, "vy": vy, "vyaw": yaw}
            if duration == -1:
                self._continuous(vx, vy, yaw); return {"ret": 0, "status": "running", "duration": -1}
            if duration < 0:
                return {"error": "duration must be -1, 0, or positive", "code": "INVALID_ARGUMENT"}
            if duration == 0:
                return {"state": "idle", "ret": self.proxy.StopMove(), "duration": 0}
            return {"error": "positive duration requires action=timed_move",
                    "code": "INVALID_ARGUMENT"}
        if action == "stop_move": self._stop_continuous(); return {"ret": self.proxy.StopMove()}
        methods = {"stand_up": "StandUp", "stand_down": "StandDown", "balance_stand": "BalanceStand", "recovery_stand": "RecoveryStand", "damp": "Damp"}
        if action in methods: return {"ret": getattr(self.proxy, methods[action])()}
        try:
            if action == "euler":
                roll = _bounded_argument(args, "roll", *self._ROLL_LIMITS)
                pitch = _bounded_argument(args, "pitch", *self._PITCH_LIMITS)
                yaw = _bounded_argument(args, "yaw", *self._YAW_LIMITS)
                return {"ret": self.proxy.Euler(roll, pitch, yaw)}
            if action == "speed_level":
                level = _integer_argument(args, "level", {-1, 0, 1})
                return {"ret": self.proxy.SpeedLevel(level)}
            if action == "body_height":
                height = _bounded_argument(args, "height", *self._BODY_HEIGHT_LIMITS)
                return {"ret": self.proxy.BodyHeight(height)}
            if action == "body_position":
                x = _bounded_argument(args, "x", *self._BODY_POSITION_LIMITS)
                y = _bounded_argument(args, "y", *self._BODY_POSITION_LIMITS)
                z = _bounded_argument(args, "z", *self._BODY_POSITION_LIMITS)
                yaw = _bounded_argument(args, "yaw", *self._YAW_LIMITS)
                return {"ret": self.proxy.BodyPosition(x, y, z, yaw)}
            if action == "switch_gait":
                level = _integer_argument(args, "level", self._GAIT_TYPES)
                return {"ret": self.proxy.SwitchGait(level)}
        except ValueError as exc:
            return {"error": str(exc), "code": "INVALID_ARGUMENT"}
        if action == "auto_recovery": return {"ret": self.proxy.SetAutoRecovery(1 if args.get("flag", True) else 0)}
        if action == "switch_joystick": return {"ret": self.proxy.SwitchJoystick(1 if args.get("flag", True) else 0)}
        if action == "left_side_gait": return {"ret": self.proxy.LeftSideGait(1 if args.get("flag", True) else 0)}
        if action == "right_side_gait": return {"ret": self.proxy.RightSideGait(1 if args.get("flag", True) else 0)}
        if action == "get_state":
            code, state = self.proxy.GetState()
            return {"ret": code, "state": state}
        return None


class SpecialActionPlugin:
    """AS2W-specific discrete motions provided by the official SportClient."""
    PREFIX = "special_action"

    def __init__(self, config, namespace, executor, proxy, prepare_motion=None):
        self.proxy = proxy
        self._prepare_motion = prepare_motion

    def get_tool(self):
        actions = ["front_flip", "back_flip", "handstand", "biped_stand"]
        return {"name": "special_action", "type": "actuator", "multiInstance": False,
                "description": "AS2W discrete acrobatic motions via the official SportClient. Requires a clear safety area.",
                "inputSchema": {"type": "object", "properties": {
                    "action": {"type": "string", "enum": actions},
                    "enter": {"type": "boolean", "description": "Enter or exit a sustained posture."},
                    "confirm": {"type": "boolean", "description": "Required true for hazardous motions."}},
                    "required": ["action"],
                    "x-is-dangerous": True,
                    "x-action-params": {
                        "front_flip": {"params": ["confirm"], "description": "DANGEROUS forward flip; requires confirm=true."},
                        "back_flip": {"params": ["confirm"], "description": "DANGEROUS backward flip; requires confirm=true."},
                        "handstand": {"params": ["enter", "confirm"], "description": "DANGEROUS handstand; requires confirm=true."},
                        "biped_stand": {"params": ["enter", "confirm"], "description": "DANGEROUS biped stand; requires confirm=true."}}}}

    def start(self): pass
    def stop(self): pass

    def dispatch(self, action, args):
        if action in ("start", "info"): return {"state": "ready"}
        if action == "stop": return {"state": "idle"}
        if action in ("front_flip", "back_flip", "handstand", "biped_stand") and not args.get("confirm", False):
            return {"error": "special action requires confirm=true"}
        if action in ("front_flip", "back_flip", "handstand", "biped_stand") and self._prepare_motion:
            self._prepare_motion()
        if action == "front_flip": return {"ret": self.proxy.FrontFlip()}
        if action == "back_flip": return {"ret": self.proxy.BackFlip()}
        if action == "handstand": return {"ret": self.proxy.HandStand(1 if args.get("enter", True) else 0)}
        if action == "biped_stand": return {"ret": self.proxy.BipedStand(1 if args.get("enter", True) else 0)}
        return None


_AS2_JOINT_NAMES = [
    "FR_hip", "FR_thigh", "FR_calf", "FR_foot",
    "FL_hip", "FL_thigh", "FL_calf", "FL_foot",
    "RR_hip", "RR_thigh", "RR_calf", "RR_foot",
    "RL_hip", "RL_thigh", "RL_calf", "RL_foot",
]
