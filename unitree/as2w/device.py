"""Unitree AS2W driver plugins (official AS2 SDK SportClient)."""
import json
import math
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


class _StateNode:
    def __init__(self, namespace, executor):
        from rclpy.node import Node
        self.node = Node("as2w_state")
        self.imu = self.node.create_publisher(String, f"/{namespace}/state/imu", 10)
        self.joints = self.node.create_publisher(String, f"/{namespace}/state/joints", 10)
        self.joint_state = self.node.create_publisher(String, f"/{namespace}/state/joint_state", 10)
        self.battery = self.node.create_publisher(String, f"/{namespace}/state/battery", 10)
        self.loco = self.node.create_publisher(String, f"/{namespace}/loco/state", 10)
        self._low = ChannelSubscriber("rt/lowstate", LowState_)
        self._sport = ChannelSubscriber("rt/sportmodestate", SportModeState_)
        self._low.Init(self._on_low, 10)
        self._sport.Init(self._on_sport, 10)
        executor.add_node(self.node)

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

    def _on_sport(self, msg):
        imu = getattr(msg, "imu_state", None)
        self._publish(self.loco, {"mode": int(getattr(msg, "mode", 0)),
                                  "velocity": list(getattr(msg, "velocity", [])),
                                  "position": list(getattr(msg, "position", [])),
                                  "body_height": _number(getattr(msg, "body_height", 0)),
                                  "imu_rpy": list(getattr(imu, "rpy", [])) if imu else []})


class StatePlugin:
    PREFIX = "state"
    def __init__(self, config, namespace, executor):
        self._namespace = namespace
        self._state = _StateNode(namespace, executor)
    def get_tools(self):
        specs = (("imu", "state/imu", "data/json", "AS2 IMU state"),
                 ("joints", "state/joints", "sensor/skeleton", "AS2 12-joint skeleton for model animation"),
                 ("joint_state", "state/joint_state", "data/json", "AS2 raw motor position, velocity, torque, and temperature"),
                 ("battery", "state/battery", "data/json", "AS2 BMS state"),
                 ("loco_state", "loco/state", "data/json", "AS2 high-level locomotion state"))
        return [{"name": name, "type": "sensor", "multiInstance": False, "description": desc,
                 "inputSchema": {"type": "object", "properties": {}},
                 "topic_out": [{"topic": f"/{self._namespace}/{path}", "format": fmt}]}
                for name, path, fmt, desc in specs]
    def start(self): pass
    def stop(self): pass
    def dispatch(self, action, args):
        return {"state": "running"} if action in ("start", "info", "imu", "joints", "joint_state", "battery", "loco_state") else ({"state": "idle"} if action == "stop" else None)


class LocoPlugin:
    PREFIX = "loco"
    def __init__(self, config, namespace, executor, proxy):
        self.proxy = proxy
        self._lock = threading.Lock()
        self._stop = None
    def get_tool(self):
        actions = ["move", "stop_move", "stand_up", "stand_down", "balance_stand", "recovery_stand", "damp", "euler", "speed_level", "body_height", "body_position", "switch_gait", "switch_joystick", "left_side_gait", "right_side_gait", "auto_recovery", "get_state"]
        return {"name": "loco", "type": "actuator", "multiInstance": False,
                "description": "Unitree AS2 locomotion via SportClient", "inputSchema": {"type": "object", "properties": {
                    "action": {"type": "string", "enum": actions}, "vx": {"type": "number"}, "vy": {"type": "number"}, "vyaw": {"type": "number"},
                    "duration": {"type": "number"}, "roll": {"type": "number"}, "pitch": {"type": "number"}, "yaw": {"type": "number"},
                    "level": {"type": "integer"}, "height": {"type": "number"}, "x": {"type": "number"}, "y": {"type": "number"}, "z": {"type": "number"}, "flag": {"type": "boolean"}}, "required": ["action"]}}
    def start(self): pass
    def stop(self):
        self._stop_continuous()
        self.proxy.StopMove()
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
    def dispatch(self, action, args):
        if action in ("start", "info"): return {"state": "ready"}
        if action == "stop": return {"state": "idle"}
        if action == "move":
            vx, vy, yaw = max(-1.5, min(1.5, float(args.get("vx", 0)))), max(-1, min(1, float(args.get("vy", 0)))), max(-2, min(2, float(args.get("vyaw", 0))))
            duration = args.get("duration")
            if duration is None: return {"ret": self.proxy.Move(vx, vy, yaw), "vx": vx, "vy": vy, "vyaw": yaw}
            duration = float(duration)
            if duration == -1:
                self._continuous(vx, vy, yaw); return {"ret": 0, "status": "running", "duration": -1}
            if duration < 0: return {"ret": -1, "message": "duration must be -1, 0, or positive"}
            self._stop_continuous(); ret = self.proxy.Move(vx, vy, yaw); time.sleep(duration); self.proxy.StopMove(); return {"ret": ret, "duration": duration}
        if action == "stop_move": self._stop_continuous(); return {"ret": self.proxy.StopMove()}
        methods = {"stand_up": "StandUp", "stand_down": "StandDown", "balance_stand": "BalanceStand", "recovery_stand": "RecoveryStand", "damp": "Damp"}
        if action in methods: return {"ret": getattr(self.proxy, methods[action])()}
        if action == "euler": return {"ret": self.proxy.Euler(float(args.get("roll", 0)), float(args.get("pitch", 0)), float(args.get("yaw", 0)))}
        if action == "speed_level": return {"ret": self.proxy.SpeedLevel(max(-1, min(1, int(args.get("level", 0)))))}
        if action == "body_height": return {"ret": self.proxy.BodyHeight(float(args.get("height", 0)))}
        if action == "body_position": return {"ret": self.proxy.BodyPosition(float(args.get("x", 0)), float(args.get("y", 0)), float(args.get("z", 0)), float(args.get("yaw", 0)))}
        if action == "switch_gait": return {"ret": self.proxy.SwitchGait(int(args.get("level", 0)))}
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

    def __init__(self, config, namespace, executor, proxy):
        self.proxy = proxy

    def get_tool(self):
        actions = ["front_flip", "back_flip", "handstand", "biped_stand"]
        return {"name": "special_action", "type": "actuator", "multiInstance": False,
                "description": "AS2W discrete acrobatic motions via the official SportClient. Requires a clear safety area.",
                "inputSchema": {"type": "object", "properties": {
                    "action": {"type": "string", "enum": actions},
                    "enter": {"type": "boolean", "description": "Enter or exit a sustained posture."}},
                    "required": ["action"]}}

    def start(self): pass
    def stop(self): pass

    def dispatch(self, action, args):
        if action in ("start", "info"): return {"state": "ready"}
        if action == "stop": return {"state": "idle"}
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
