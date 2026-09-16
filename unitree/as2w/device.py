"""Unitree AS2W driver plugins (official AS2 SDK SportClient)."""
import json
import math
import threading
import time
from uuid import uuid4

from std_msgs.msg import String
from unitree_sdk2py.core.channel import ChannelSubscriber
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import BmsState_, LowState_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_


def _number(value, default=0.0):
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def _values(value):
    try:
        return list(value)
    except TypeError:
        return [value]


def _acp_notify(action_id, status, result):
    import os, ssl, urllib.request
    payload = json.dumps({"action_id": action_id, "status": status,
                          "result": result, "tool": "loco", "ts": time.time()}).encode()
    try:
        request = urllib.request.Request(f"{os.environ.get('AGENT_CORE_URL', 'https://localhost:15678')}/api/acp/complete",
            data=payload, headers={"Content-Type": "application/json"}, method="POST")
        urllib.request.urlopen(request, timeout=5, context=ssl._create_unverified_context())
    except Exception as exc:
        print(f"[loco] ACP callback failed for {action_id}: {exc}", flush=True)


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
        self._bms = ChannelSubscriber("rt/lf/bmsstate", BmsState_)
        self._sport = ChannelSubscriber("rt/sportmodestate", SportModeState_)
        self._low.Init(self._on_low, 10)
        self._bms.Init(self._on_bms, 10)
        self._sport.Init(self._on_sport, 10)
        executor.add_node(self.node)

    def close(self):
        for subscriber in (self._low, self._bms, self._sport):
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
                   "temperature": _values(getattr(m, "temperature", []))} for i, m in enumerate(motors)]
        self._publish(self.joint_state, {"joint_states": states})
        self._publish(self.joints, {"joints": [{"idx": s["idx"], "name": _AS2_JOINT_NAMES[s["idx"]], "q": s["q"]}
                                               for s in states[:len(_AS2_JOINT_NAMES)]],
                                    "imu_quat": list(getattr(imu, "quaternion", [])) if imu else []})
    def _on_bms(self, bms):
        self._publish(self.battery, {"soc": int(getattr(bms, "soc", 0)),
                                     "current": _number(getattr(bms, "current", 0)),
                                     "cycle": int(getattr(bms, "cycle", 0)),
                                     "temperature": _values(getattr(bms, "temperature", []))})

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
                 ("joints", "state/joints", "sensor/skeleton", "AS2W 16-joint skeleton for model animation"),
                 ("joint_state", "state/joint_state", "data/json", "AS2 raw motor position, velocity, torque, and temperature"),
                 ("battery", "state/battery", "data/json", "AS2 BMS state"),
                 ("loco_state", "loco/state", "data/json", "AS2 high-level locomotion state"))
        return [{"name": name, "type": "sensor", "multiInstance": False, "description": desc,
                 "inputSchema": {"type": "object", "properties": {}},
                 "topic_out": [{"topic": f"/{self._namespace}/{path}", "format": fmt}]}
                for name, path, fmt, desc in specs]
    def start(self): pass
    def stop(self): self._state.close()
    def dispatch(self, action, args):
        if action == "info":
            name = args.get("_tool_name")
            paths = {"imu": ("state/imu", "data/json"),
                     "joints": ("state/joints", "sensor/skeleton"),
                     "joint_state": ("state/joint_state", "data/json"),
                     "battery": ("state/battery", "data/json"),
                     "loco_state": ("loco/state", "data/json")}
            if name in paths:
                path, fmt = paths[name]
                return {"state": "running", "topic_out": [{"topic": f"/{self._namespace}/{path}", "format": fmt}]}
            return {"state": "running"}
        if action in ("imu", "joints", "joint_state", "battery", "loco_state"):
            path = {"imu": "state/imu", "joints": "state/joints", "joint_state": "state/joint_state", "battery": "state/battery", "loco_state": "loco/state"}[action]
            fmt = "sensor/skeleton" if action == "joints" else "data/json"
            return {"state": "running", "topic_out": [{"topic": f"/{self._namespace}/{path}", "format": fmt}]}
        return {"state": "running"} if action in ("start", "info") else ({"state": "idle"} if action == "stop" else None)


class LocoPlugin:
    PREFIX = "loco"
    def __init__(self, config, namespace, executor, proxy):
        self.proxy = proxy
        self._lock = threading.Lock()
        self._stop = None
    def get_tool(self):
        actions = ["move", "stop_move", "stand_up", "stand_down", "balance_stand", "recovery_stand", "damp", "euler", "speed_level", "body_height", "body_position", "switch_joystick", "left_side_gait", "right_side_gait", "auto_recovery", "get_state"]
        return {"name": "loco", "type": "actuator", "multiInstance": False,
                "description": "AS2W locomotion. Velocity is clamped to vx [-1.5, 1.5] m/s, vy [-1, 1] m/s, yaw [-2, 2] rad/s. Stand actions are accepted first and report completion through ACP.", "inputSchema": {"type": "object", "properties": {
                    "action": {"type": "string", "enum": actions, "description": "Locomotion action"}, "vx": {"type": "number", "description": "Forward velocity m/s [-1.5, 1.5]"}, "vy": {"type": "number", "description": "Lateral velocity m/s [-1, 1]"}, "vyaw": {"type": "number", "description": "Yaw velocity rad/s [-2, 2]"},
                    "duration": {"type": "number", "minimum": -1, "maximum": 30, "description": "Seconds; -1 continues until stop_move"}, "roll": {"type": "number", "description": "Body roll radians"}, "pitch": {"type": "number", "description": "Body pitch radians"}, "yaw": {"type": "number", "description": "Body yaw radians"},
                    "speed_preset": {"type": "string", "enum": ["slow", "normal", "fast"], "description": "Speed limiter preset"}, "height": {"type": "number", "description": "Body height offset"}, "x": {"type": "number", "description": "Body X offset"}, "y": {"type": "number", "description": "Body Y offset"}, "z": {"type": "number", "description": "Body Z offset"}, "flag": {"type": "boolean", "description": "Enable or disable the selected feature"}}, "required": ["action"],
                "x-completion": {"actions": ["stand_up", "stand_down", "balance_stand", "recovery_stand"], "timeout": 20},
                "x-action-params": {
                    "move": {"params": ["vx", "vy", "vyaw", "duration"], "description": "Move with optional duration (-1 for continuous)."},
                    "stop_move": {"params": [], "description": "Stop movement."},
                    "stand_up": {"params": [], "description": "Stand up."}, "stand_down": {"params": [], "description": "Stand down."},
                    "balance_stand": {"params": [], "description": "Balance stand."}, "recovery_stand": {"params": [], "description": "Recovery stand."},
                    "damp": {"params": [], "description": "Damp motors."}, "euler": {"params": ["roll", "pitch", "yaw"], "description": "Set body attitude."},
                    "speed_level": {"params": ["speed_preset"], "description": "Set speed limiter: slow, normal, or fast."}, "body_height": {"params": ["height"], "description": "Set body height offset."},
                    "body_position": {"params": ["x", "y", "z", "yaw"], "description": "Set body position offset."},
                    "switch_joystick": {"params": ["flag"], "description": "Enable or disable joystick."}, "left_side_gait": {"params": ["flag"], "description": "Enable left-side gait."},
                    "right_side_gait": {"params": ["flag"], "description": "Enable right-side gait."}, "auto_recovery": {"params": ["flag"], "description": "Enable or disable auto recovery."},
                    "get_state": {"params": [], "description": "Read sport state."}}}}
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
    def _await_posture(self, action_id, action, expected_damping):
        deadline = time.monotonic() + 20
        while time.monotonic() < deadline:
            code, state = self.proxy.GetState()
            fsm = str(state.get("fsm_id", "")) if code == 0 else ""
            if fsm and (fsm == "0") == expected_damping:
                _acp_notify(action_id, "completed", {"action": action, "state": state})
                return
            time.sleep(.25)
        _acp_notify(action_id, "error", {"action": action, "error": "controller state did not reach the expected posture within 20 seconds"})
    def dispatch(self, action, args):
        if action in ("start", "info"): return {"state": "ready"}
        if action == "stop":
            self._stop_continuous()
            return {"state": "idle", "ret": self.proxy.StopMove()}
        if action == "move":
            vx, vy, yaw = max(-1.5, min(1.5, float(args.get("vx", 0)))), max(-1, min(1, float(args.get("vy", 0)))), max(-2, min(2, float(args.get("vyaw", 0))))
            duration = args.get("duration")
            if duration is None: return {"ret": self.proxy.Move(vx, vy, yaw), "vx": vx, "vy": vy, "vyaw": yaw}
            duration = float(duration)
            if not math.isfinite(duration) or duration > 30:
                return {"ret": -1, "message": "duration must be at most 30 seconds"}
            if duration == -1:
                self._continuous(vx, vy, yaw); return {"ret": 0, "status": "running", "duration": -1}
            if duration < 0: return {"ret": -1, "message": "duration must be -1, 0, or positive"}
            self._stop_continuous(); ret = self.proxy.Move(vx, vy, yaw); time.sleep(duration); self.proxy.StopMove(); return {"ret": ret, "duration": duration}
        if action == "stop_move": self._stop_continuous(); return {"ret": self.proxy.StopMove()}
        methods = {"stand_up": ("StandUp", False), "stand_down": ("StandDown", True), "balance_stand": ("BalanceStand", False), "recovery_stand": ("RecoveryStand", False)}
        if action in methods:
            method, expected_damping = methods[action]
            ret = getattr(self.proxy, method)()
            if ret != 0: return {"ret": ret, "accepted": False, "action": action}
            action_id = f"as2w_loco_{uuid4().hex[:8]}"
            threading.Thread(target=self._await_posture, args=(action_id, action, expected_damping), daemon=True).start()
            return {"ret": 0, "accepted": True, "status": "running", "action": action, "action_id": action_id}
        if action == "damp": return {"ret": self.proxy.Damp(), "accepted": True, "action": action}
        if action == "euler": return {"ret": self.proxy.Euler(float(args.get("roll", 0)), float(args.get("pitch", 0)), float(args.get("yaw", 0)))}
        if action == "speed_level":
            preset = args.get("speed_preset", "normal")
            if preset not in {"slow", "normal", "fast"}: return {"ret": -1, "error": "speed_preset must be slow, normal, or fast"}
            return {"ret": self.proxy.SpeedLevel({"slow": -1, "normal": 0, "fast": 1}[preset]), "speed_preset": preset}
        if action == "body_height": return {"ret": self.proxy.BodyHeight(float(args.get("height", 0)))}
        if action == "body_position": return {"ret": self.proxy.BodyPosition(float(args.get("x", 0)), float(args.get("y", 0)), float(args.get("z", 0)), float(args.get("yaw", 0)))}
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
