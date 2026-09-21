"""Unitree As2W driver plugins (official AS2 SDK SportClient)."""
import json
import math
import threading
import time
from uuid import uuid4

from audio_msgs.msg import AudioChunk
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
from unitree_sdk2py.core.channel import ChannelSubscriber
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import BmsState_, LowState_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import AudioData_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_


_LOW_LAT_QOS = None
_CAMERA_QOS = None
try:
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    _LOW_LAT_QOS = QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        durability=DurabilityPolicy.VOLATILE,
    )
    _CAMERA_QOS = QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        durability=DurabilityPolicy.VOLATILE,
    )
except ImportError:
    # Unit tests load the card contracts without a ROS installation.
    pass


_STATE_PUBLISH_HZ = 30.0


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
        self.imu = self.node.create_publisher(String, f"/{namespace}/state/imu", _LOW_LAT_QOS or 1)
        self.joints = self.node.create_publisher(String, f"/{namespace}/state/joints", _LOW_LAT_QOS or 1)
        self.joint_state = self.node.create_publisher(String, f"/{namespace}/state/joint_state", _LOW_LAT_QOS or 1)
        self.battery = self.node.create_publisher(String, f"/{namespace}/state/battery", _LOW_LAT_QOS or 1)
        self.loco = self.node.create_publisher(String, f"/{namespace}/loco/state", _LOW_LAT_QOS or 1)
        self._low = ChannelSubscriber("rt/lowstate", LowState_)
        self._bms = ChannelSubscriber("rt/lf/bmsstate", BmsState_)
        # AS2/As2W's official sport-state example uses the lf namespace.
        self._sport = ChannelSubscriber("rt/lf/sportmodestate", SportModeState_)
        # DDS callbacks must do almost no work.  In particular, JSON encoding
        # and ROS publication from the callback can make the callback queue
        # fall behind the robot's LowState stream.  Keep only the newest
        # object and let a small publisher worker consume it.  This gives the
        # frontend the newest pose instead of an old queue of poses.
        self._latest_lock = threading.Lock()
        self._latest_low = None
        self._latest_bms = None
        self._latest_sport = None
        self._low_generation = 0
        self._bms_generation = 0
        self._sport_generation = 0
        self._published_low_generation = -1
        self._published_bms_generation = -1
        self._published_sport_generation = -1
        self._stop_event = threading.Event()
        self._publisher_thread = threading.Thread(
            target=self._publish_loop, daemon=True, name="as2w-state-publish")
        self._low.Init(self._on_low, 1)
        self._bms.Init(self._on_bms, 1)
        self._sport.Init(self._on_sport, 1)
        executor.add_node(self.node)
        self._publisher_thread.start()

    def close(self):
        for subscriber in (self._low, self._bms, self._sport):
            try:
                subscriber.Close()
            except Exception:
                pass
        self._stop_event.set()
        publisher_thread = getattr(self, "_publisher_thread", None)
        if publisher_thread is not None:
            publisher_thread.join(timeout=1.0)
        self.node.destroy_node()

    def _publish(self, publisher, value):
        message = String()
        message.data = json.dumps(value, separators=(",", ":"))
        publisher.publish(message)

    @staticmethod
    def _flat(prefix, values):
        return {f"{prefix}_{i}": float(value) for i, value in enumerate(values)}

    def _publish_loop(self):
        wait_for = 1.0 / _STATE_PUBLISH_HZ
        while not self._stop_event.is_set():
            with self._latest_lock:
                low = self._latest_low
                bms = self._latest_bms
                sport = self._latest_sport
                low_generation = self._low_generation
                bms_generation = self._bms_generation
                sport_generation = self._sport_generation
            if low is not None and low_generation != self._published_low_generation:
                self._publish_low(low)
                self._published_low_generation = low_generation
            if bms is not None and bms_generation != self._published_bms_generation:
                self._publish_bms(bms)
                self._published_bms_generation = bms_generation
            if sport is not None and sport_generation != self._published_sport_generation:
                self._publish_sport(sport)
                self._published_sport_generation = sport_generation
            self._stop_event.wait(wait_for)

    def _on_low(self, msg):
        latest_lock = getattr(self, "_latest_lock", None)
        if latest_lock is None:
            self._publish_low(msg)
            return
        with latest_lock:
            self._latest_low = msg
            self._low_generation += 1
        # The test harness constructs this object without __init__.  Keep the
        # helper useful there while the real node always uses the worker.
        if getattr(self, "_publisher_thread", None) is None:
            self._publish_low(msg)

    def _publish_low(self, msg):
        imu = getattr(msg, "imu_state", getattr(msg, "imu", None))
        if imu is not None:
            imu_data = {}
            for key in ("quaternion", "gyroscope", "accelerometer", "rpy"):
                imu_data.update(self._flat(key, getattr(imu, key, [])))
            self._publish(self.imu, imu_data)
        motors = getattr(msg, "motor_state", getattr(msg, "motor_states", []))
        # AS2W publishes a fixed 35-slot array, but only the first 12 slots are
        # leg motors.  The remaining slots are reserved and contain zeros.
        motors = list(motors)[:len(_AS2_JOINT_NAMES)]
        states = [{"idx": i, "q": _number(getattr(m, "q", 0)),
                   "dq": _number(getattr(m, "dq", 0)),
                   "tau": _number(getattr(m, "tau_est", getattr(m, "tau", 0))),
                   "temperature": _values(getattr(m, "temperature", []))} for i, m in enumerate(motors)]
        joint_data = {}
        for state in states:
            name = _AS2_JOINT_NAMES[state["idx"]]
            joint_data[f"{name}_q"] = state["q"]
            joint_data[f"{name}_dq"] = state["dq"]
            joint_data[f"{name}_tau"] = state["tau"]
            for index, temperature in enumerate(state["temperature"]):
                joint_data[f"{name}_temperature_{index}"] = float(temperature)
        self._publish(self.joint_state, joint_data)
        skeleton = [{"idx": s["idx"], "name": _AS2_JOINT_NAMES[s["idx"]], "q": s["q"],
                     "dq": s["dq"], "tau": s["tau"],
                     "temperature": s["temperature"]}
                    for s in states[:len(_AS2_JOINT_NAMES)]]
        # Keep the established Go2/G1 sensor/skeleton contract.  The frontend
        # expects these two top-level fields and does not consume joint_count.
        self._publish(self.joints, {"joints": skeleton,
                                    "imu_quat": list(getattr(imu, "quaternion", [])) if imu else []})

    def _on_bms(self, bms):
        latest_lock = getattr(self, "_latest_lock", None)
        if latest_lock is None:
            self._publish_bms(bms)
            return
        with latest_lock:
            self._latest_bms = bms
            self._bms_generation += 1
        if getattr(self, "_publisher_thread", None) is None:
            self._publish_bms(bms)

    def _publish_bms(self, bms):
        # Unitree BmsState_.current is milliamps (mA).
        current_ma = _number(getattr(bms, "current", 0))
        battery = {"soc": int(getattr(bms, "soc", 0)),
                   "current_ma": current_ma,
                   "cycle": int(getattr(bms, "cycle", 0))}
        battery.update(self._flat("temperature", getattr(bms, "temperature", [])))
        self._publish(self.battery, battery)

    def _on_sport(self, msg):
        latest_lock = getattr(self, "_latest_lock", None)
        if latest_lock is None:
            self._publish_sport(msg)
            return
        with latest_lock:
            self._latest_sport = msg
            self._sport_generation += 1
        if getattr(self, "_publisher_thread", None) is None:
            self._publish_sport(msg)

    def _publish_sport(self, msg):
        loco = {"mode": int(getattr(msg, "mode", 0)),
                "body_height": _number(getattr(msg, "body_height", 0))}
        loco.update(self._flat("velocity", getattr(msg, "velocity", [])))
        loco.update(self._flat("position", getattr(msg, "position", [])))
        self._publish(self.loco, loco)


class StatePlugin:
    PREFIX = "state"
    def __init__(self, config, namespace, executor):
        self._namespace = namespace
        self._executor = executor
        self._state = _StateNode(namespace, executor)
    def get_tools(self):
        specs = (("imu", "state/imu", "data/json", "As2W IMU state"),
                 ("joints", "state/joints", "sensor/skeleton", "As2W 12-joint leg skeleton for model animation"),
                 ("joint_state", "state/joint_state", "data/json", "As2W raw motor position, velocity, torque, and temperature"),
                 ("battery", "state/battery", "data/json", "As2W BMS state; current_ma is mA"),
                 ("loco_state", "loco/state", "data/json", "As2W high-level locomotion state"))
        return [{"name": name, "type": "sensor", "multiInstance": False, "description": desc,
                 "inputSchema": {"type": "object", "properties": {}},
                 "topic_out": [{"topic": f"/{self._namespace}/{path}", "format": fmt}]}
                for name, path, fmt, desc in specs]
    def start(self):
        # Sensor cards share one LowState subscription.  Recreate it when a
        # dashboard stopped the card instead of claiming a dead stream is live.
        if self._state is None:
            self._state = _StateNode(self._namespace, self._executor)

    def stop(self):
        if self._state is not None:
            self._state.close()
            self._state = None
    def dispatch(self, action, args):
        if action == "start":
            self.start()
            return {"state": "running"}
        if action == "stop":
            self.stop()
            return {"state": "idle"}
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
        return {"state": "running"} if action == "info" else None


class LocoPlugin:
    PREFIX = "loco"
    def __init__(self, config, namespace, executor, proxy):
        self.proxy = proxy
        self._lock = threading.Lock()
        self._stop = None
        self._transition_stop = None

    _STANDING = {"STAND_UP", "BALANCE_STAND", "RECOVERY_STAND", "STANDING"}
    # AS2 reports fsm_id=0/fsm_name=PASSIVE while the body is not yet in the
    # balance controller.  It is ambiguous from the name alone whether the
    # operator has just stood the robot up, so a move request starts the
    # balance transition in the background and lets the firmware reject it if
    # the posture is actually unsafe.
    _BALANCE_REQUIRED = {"STAND_UP", "PASSIVE", "STAND", "STANDING"}
    _MOVING = {"WALK", "WALKING", "RUN", "RUNNING", "MOVE", "MOVING",
               "REGULAR_WALK", "REGULAR_RUN"}
    _DOWN = {"STAND_DOWN", "DAMPING", "LYING", "FALL", "FALLEN",
             "SQUAT"}

    @classmethod
    def _is_moving(cls, state):
        return state in cls._MOVING or "WALK" in state or "RUN" in state or "MOVE" in state

    def _read_state(self):
        result = self.proxy.GetState()
        if not isinstance(result, tuple) or len(result) != 2:
            return None, None, {"ret": result if isinstance(result, int) else 3104,
                                 "current_state": "UNKNOWN",
                                 "error": "Unable to read robot locomotion state",
                                 "reason": "SportClient.GetState did not return a state",
                                 "suggested_actions": ["get_state"]}
        code, state = result
        if code != 0 or not isinstance(state, dict):
            return None, state if isinstance(state, dict) else {}, {
                "ret": code, "current_state": "UNKNOWN",
                "error": "Unable to read robot locomotion state",
                "reason": "SportClient.GetState failed; refusing an unsafe transition",
                "suggested_actions": ["get_state"]}
        name = str(state.get("fsm_name", "")).strip().upper()
        if not name:
            return None, state, {
                "ret": -1, "current_state": "UNKNOWN",
                "error": "Robot returned no locomotion state",
                "reason": "The current FSM state is unknown; refusing the action",
                "suggested_actions": ["get_state"]}
        return name, state, None

    @staticmethod
    def _not_allowed(action, state, reason, suggested):
        return {"ret": -1, "accepted": False, "action": action,
                "current_state": state or "UNKNOWN", "error": "Action cannot be executed",
                "reason": reason, "suggested_actions": suggested}

    @classmethod
    def _rpc_rejected(cls, action, state, ret, reason, suggested):
        result = cls._not_allowed(action, state, reason, suggested)
        result["ret"] = ret
        result["rpc_ret"] = ret
        return result

    def _cancel_transition(self):
        with self._lock:
            event = self._transition_stop
            self._transition_stop = None
        if event is not None:
            event.set()

    def _finish_transition(self, stop_event):
        with self._lock:
            if self._transition_stop is stop_event:
                self._transition_stop = None

    def _transition_to_balance_and_move(self, action_id, vx, vy, yaw, duration, stop_event):
        deadline = time.monotonic() + 12.0
        last_state = "UNKNOWN"
        last_move_ret = None
        while time.monotonic() < deadline:
            if stop_event.is_set():
                self._finish_transition(stop_event)
                _acp_notify(action_id, "cancelled", {
                    "action": "move", "reason": "move transition was cancelled"})
                return
            name, state, error = self._read_state()
            last_state = name or last_state
            if name == "BALANCE_STAND":
                return self._run_move_after_transition(action_id, vx, vy, yaw, duration, stop_event)
            if error:
                self._finish_transition(stop_event)
                _acp_notify(action_id, "error", error)
                return
            # Some AS2 firmware keeps GetState at PASSIVE for a short time
            # after BalanceStand.  Do not wait forever for a state label that
            # is lagging behind the command path: probe Move in the worker and
            # continue as soon as the sport service accepts it.
            last_move_ret = self.proxy.Move(vx, vy, yaw)
            if last_move_ret == 0:
                return self._run_move_after_transition(
                    action_id, vx, vy, yaw, duration, stop_event,
                    move_already_sent=True)
            time.sleep(0.15)
        _acp_notify(action_id, "error", self._rpc_rejected(
            "move", last_state, last_move_ret if last_move_ret is not None else -1,
            "The automatic balance-stand transition did not reach a state that accepts Move",
            ["get_state", "balance_stand", "stand_up", "stop_move"]))
        self._finish_transition(stop_event)

    def _run_move_after_transition(self, action_id, vx, vy, yaw, duration,
                                   stop_event, move_already_sent=False):
        if stop_event.is_set():
            self._finish_transition(stop_event)
            _acp_notify(action_id, "cancelled", {
                "action": "move", "reason": "move transition was cancelled"})
            return
        ret = 0 if move_already_sent else self.proxy.Move(vx, vy, yaw)
        if ret != 0:
            self._finish_transition(stop_event)
            _acp_notify(action_id, "error", {"ret": ret, "accepted": False,
                "action": "move", "current_state": "BALANCE_STAND",
                "error": "Move was rejected after balance stand",
                "reason": "The sport controller refused the velocity command",
                "suggested_actions": ["get_state", "stop_move"]})
            return
        if duration is None:
            self._finish_transition(stop_event)
            _acp_notify(action_id, "completed", {
                "action": "move", "ret": 0, "current_state": "BALANCE_STAND"})
            return
        if duration == -1:
            with self._lock:
                self._stop = stop_event
            while not stop_event.is_set():
                if self.proxy.Move(vx, vy, yaw) != 0:
                    break
                stop_event.wait(0.1)
            self.proxy.StopMove()
            with self._lock:
                if self._stop is stop_event:
                    self._stop = None
            self._finish_transition(stop_event)
            return
        deadline = time.monotonic() + duration
        try:
            while time.monotonic() < deadline:
                time.sleep(min(0.1, deadline - time.monotonic()))
                if time.monotonic() < deadline:
                    ret = self.proxy.Move(vx, vy, yaw)
                    if ret != 0:
                        break
        finally:
            self.proxy.StopMove()
        _acp_notify(action_id, "completed" if ret == 0 else "error",
                    {"action": "move", "ret": ret, "duration": duration})
        self._finish_transition(stop_event)

    def _move_error_for_state(self, state):
        if state in self._DOWN:
            return self._not_allowed("move", state,
                "The robot is not standing, so the controller rejects Move",
                ["stand_up", "recovery_stand"])
        return self._not_allowed("move", state,
            "The robot is in a transition, special motion, or fault state",
            ["stop_move", "recovery_stand", "get_state"])
    def get_tool(self):
        actions = ["move", "stop_move", "stand_up", "stand_down", "balance_stand", "recovery_stand", "damp", "euler", "speed_level", "body_height", "body_position", "switch_joystick", "left_side_gait", "right_side_gait", "auto_recovery", "get_state"]
        return {"name": "loco", "type": "actuator", "multiInstance": False,
                "description": "As2W locomotion. Velocity is clamped to vx [-1.5, 1.5] m/s, vy [-1, 1] m/s, yaw [-2, 2] rad/s. Stand actions are accepted first and report completion through ACP.", "inputSchema": {"type": "object", "properties": {
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
        self._cancel_transition()
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
    def _await_posture(self, action_id, action, expected_name):
        deadline = time.monotonic() + 20
        left_old_state = False
        matches = 0
        while time.monotonic() < deadline:
            result = self.proxy.GetState()
            if not isinstance(result, tuple) or len(result) != 2:
                _acp_notify(action_id, "error", {
                    "ret": result if isinstance(result, int) else 3104,
                    "action": action,
                    "error": "Unable to read robot locomotion state",
                    "reason": "The controller did not return an FSM state while waiting for the transition",
                    "suggested_actions": ["get_state", "retry"]})
                return
            code, state = result
            if code != 0 or not isinstance(state, dict):
                _acp_notify(action_id, "error", {
                    "ret": code,
                    "current_state": "UNKNOWN",
                    "action": action,
                    "error": "Unable to read robot locomotion state",
                    "reason": "The controller returned an invalid FSM state while waiting for the transition",
                    "suggested_actions": ["get_state", "retry"]})
                return
            name = str(state.get("fsm_name", "")).upper()
            if name and name != "DAMPING":
                left_old_state = True
            if left_old_state and name == expected_name.upper():
                matches += 1
            else:
                matches = 0
            if matches >= 2:
                _acp_notify(action_id, "completed", {"action": action, "state": state})
                return
            time.sleep(.25)
        _acp_notify(action_id, "error", {"action": action,
                    "current_state": name or "UNKNOWN",
                    "error": "controller state did not reach the expected posture within 20 seconds",
                    "reason": f"The controller did not enter {expected_name}",
                    "suggested_actions": ["get_state", "stop_move", "recovery_stand"]})
    def dispatch(self, action, args):
        if action in ("start", "info"): return {"state": "ready"}
        if action == "stop":
            self._cancel_transition()
            self._stop_continuous()
            return {"state": "idle", "ret": self.proxy.StopMove()}
        if action == "move":
            vx, vy, yaw = max(-1.5, min(1.5, float(args.get("vx", 0)))), max(-1, min(1, float(args.get("vy", 0)))), max(-2, min(2, float(args.get("vyaw", 0))))
            duration = args.get("duration")
            if duration is not None:
                duration = float(duration)
                if not math.isfinite(duration) or duration > 30:
                    return {"ret": -1, "error": "duration must be at most 30 seconds"}
                if duration < 0 and duration != -1:
                    return {"ret": -1, "error": "duration must be -1, 0, or positive"}
            state_name, state, state_error = self._read_state()
            if state_error:
                return {**state_error, "action": "move"}
            if state_name in self._BALANCE_REQUIRED:
                self._cancel_transition()
                ret = self.proxy.BalanceStand()
                if ret != 0:
                    return self._rpc_rejected("move", state_name, ret,
                        "The robot is not currently accepting the automatic balance-stand transition",
                        ["get_state", "stand_up", "balance_stand", "recovery_stand"])
                action_id = f"as2w_loco_{uuid4().hex[:8]}"
                transition_stop = threading.Event()
                with self._lock:
                    self._transition_stop = transition_stop
                threading.Thread(target=self._transition_to_balance_and_move,
                    args=(action_id, vx, vy, yaw, duration, transition_stop), daemon=True,
                    name="as2w-loco-balance-move").start()
                return {"ret": 0, "accepted": True, "status": "running",
                        "action": "move", "transition": "balance_stand",
                        "action_id": action_id, "current_state": state_name}
            if state_name not in self._STANDING and not self._is_moving(state_name):
                return self._move_error_for_state(state_name)
            self._cancel_transition()
            if duration is None:
                self._stop_continuous()
                ret = self.proxy.Move(vx, vy, yaw)
                return {"ret": ret, "accepted": ret == 0, "action": "move",
                        "current_state": state_name, "vx": vx, "vy": vy, "vyaw": yaw,
                        **({} if ret == 0 else {"error": "Sport controller rejected Move",
                          "reason": "The robot is standing but the velocity command was refused",
                          "suggested_actions": ["get_state", "stop_move"]})}
            duration = float(duration)
            if duration == -1:
                self._continuous(vx, vy, yaw); return {"ret": 0, "status": "running", "duration": -1}
            if duration < 0: return {"ret": -1, "message": "duration must be -1, 0, or positive"}
            self._stop_continuous(); ret = self.proxy.Move(vx, vy, yaw); time.sleep(duration); self.proxy.StopMove(); return {"ret": ret, "duration": duration}
        if action == "stop_move":
            self._cancel_transition()
            self._stop_continuous()
            ret = self.proxy.StopMove()
            return {"ret": ret, "accepted": ret == 0, "action": action,
                    **({} if ret == 0 else {"error": "StopMove was rejected",
                      "reason": "The controller is not accepting stop commands",
                      "suggested_actions": ["get_state", "recovery_stand"]})}
        methods = {"stand_up": ("StandUp", "STAND_UP"), "stand_down": ("StandDown", "STAND_DOWN"), "balance_stand": ("BalanceStand", "BALANCE_STAND"), "recovery_stand": ("RecoveryStand", "RECOVERY_STAND")}
        if action in methods:
            self._cancel_transition()
            state_name, state, state_error = self._read_state()
            if state_error:
                return {**state_error, "action": action}
            if action == "stand_up" and self._is_moving(state_name):
                return self._not_allowed(action, state_name,
                    "StandUp cannot be issued while the robot is walking",
                    ["stop_move", "stand_up"])
            if action == "stand_up" and state_name in self._STANDING:
                return self._not_allowed(action, state_name,
                    "The robot is already standing; use balance_stand or move",
                    ["balance_stand", "move", "stand_down"])
            if action == "stand_down" and self._is_moving(state_name):
                return self._not_allowed(action, state_name,
                    "StandDown cannot be issued while the robot is walking",
                    ["stop_move", "stand_down"])
            if action == "stand_down" and state_name in self._DOWN:
                return self._not_allowed(action, state_name,
                    "The robot is already down or damping",
                    ["stand_up", "recovery_stand"])
            if action == "balance_stand" and state_name in self._DOWN:
                return self._not_allowed(action, state_name,
                    "BalanceStand requires the robot to be standing or in passive mode first",
                    ["stand_up", "recovery_stand"])
            if action == "recovery_stand" and state_name not in {"FALL", "FALLEN", "STAND_DOWN", "DAMPING"}:
                return self._not_allowed(action, state_name,
                    "RecoveryStand is only valid from a fallen or down posture",
                    ["stand_down", "damp", "recovery_stand"])
            if action in {"balance_stand", "recovery_stand"} and self._is_moving(state_name):
                return self._not_allowed(action, state_name,
                    "Posture transition cannot be issued while the robot is moving",
                    ["stop_move", action])
            method, expected_name = methods[action]
            ret = getattr(self.proxy, method)()
            if ret != 0:
                return self._rpc_rejected(action, state_name, ret,
                    "SportClient rejected the posture transition",
                    ["get_state", "stop_move", "stand_up", "recovery_stand"])
            action_id = f"as2w_loco_{uuid4().hex[:8]}"
            threading.Thread(target=self._await_posture, args=(action_id, action, expected_name), daemon=True).start()
            return {"ret": 0, "accepted": True, "status": "running", "action": action, "action_id": action_id}
        if action == "damp":
            self._cancel_transition()
            state_name, state, state_error = self._read_state()
            if state_error:
                return {**state_error, "action": action}
            if self._is_moving(state_name):
                return self._not_allowed(action, state_name,
                    "Damp is refused while the robot is walking or running",
                    ["stop_move", "damp"])
            ret = self.proxy.Damp()
            return {"ret": ret, "accepted": ret == 0, "action": action,
                    "current_state": state_name,
                    **({} if ret == 0 else {"error": "SportClient rejected the action",
                      "reason": "The controller refused damping from the current posture",
                      "suggested_actions": ["get_state", "stop_move", "recovery_stand"],
                      "rpc_ret": ret})}
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
            result = self.proxy.GetState()
            if isinstance(result, tuple) and len(result) == 2:
                code, state = result
                return {"ret": code, "state": state}
            return {"ret": result if isinstance(result, int) else 3104,
                    "state": {}, "error": "Unable to read robot locomotion state",
                    "reason": "SportClient.GetState did not return a state",
                    "suggested_actions": ["retry", "recovery_stand"]}
        return None


class SpecialActionPlugin:
    """As2W-specific discrete motions provided by the official SportClient."""
    PREFIX = "special_motion"

    def __init__(self, config, namespace, executor, proxy):
        self.proxy = proxy

    def get_tool(self):
        actions = ["front_flip", "back_flip", "handstand", "biped_stand"]
        return {"name": "special_motion", "type": "actuator", "multiInstance": False,
                "description": "As2W discrete acrobatic motions via the official SportClient. Requires a clear safety area.",
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
        methods = {
            "front_flip": lambda: self.proxy.FrontFlip(),
            "back_flip": lambda: self.proxy.BackFlip(),
            "handstand": lambda: self.proxy.HandStand(1 if args.get("enter", True) else 0),
            "biped_stand": lambda: self.proxy.BipedStand(1 if args.get("enter", True) else 0),
        }
        if action in methods:
            state_result = self.proxy.GetState()
            if not isinstance(state_result, tuple) or len(state_result) != 2:
                return {"ret": state_result if isinstance(state_result, int) else 3104,
                        "accepted": False, "action": action, "current_state": "UNKNOWN",
                        "error": "Unable to read robot locomotion state",
                        "reason": "The current FSM state is unknown; refusing a dangerous motion",
                        "suggested_actions": ["get_state", "stand_up"]}
            code, state = state_result
            state_name = str(state.get("fsm_name", "")).strip().upper() if isinstance(state, dict) else ""
            if code != 0 or not state_name:
                return {"ret": code, "accepted": False, "action": action,
                        "current_state": state_name or "UNKNOWN",
                        "error": "Unable to read robot locomotion state",
                        "reason": "The current FSM state is unknown; refusing a dangerous motion",
                        "suggested_actions": ["get_state", "stand_up"]}
            if state_name not in {"STAND_UP", "BALANCE_STAND", "RECOVERY_STAND"}:
                return {"ret": -1, "accepted": False, "action": action,
                        "current_state": state_name,
                        "error": "Special motion cannot be executed from the current state",
                        "reason": "The robot must be standing and balanced before a special motion",
                        "suggested_actions": ["stand_up", "balance_stand", "recovery_stand"]}
            ret = methods[action]()
            return {"ret": ret, "accepted": ret == 0, "action": action,
                    "current_state": state_name,
                    **({} if ret == 0 else {"error": "SportClient rejected the special action",
                      "reason": "The controller refused the special motion from the current posture",
                      "suggested_actions": ["get_state", "recovery_stand"]})}
        return None


_AS2_JOINT_NAMES = [
    "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
    "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
    "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
    "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint",
]


MIC_AUDIO_FORMAT = "audio/pcm-16k"
SPEAKER_APP_NAME = "as2w_speaker"
SPEAKER_BLOCK_BYTES = 3200  # 100 ms at 16 kHz, 16-bit, mono.
SPEAKER_QUEUE_BLOCKS = 8  # Keep the live stream below 800 ms of queued audio.


def _audio_chunk(payload):
    """Build the common ROS audio message from a DDS byte sequence."""
    message = AudioChunk()
    message.format = MIC_AUDIO_FORMAT
    message.data = list(bytes(payload))
    return message


class _MicNode:
    """Republish AS2's robot microphone DDS stream as AudioChunk messages."""

    def __init__(self, topic, config=None):
        from rclpy.node import Node
        self.node = Node("as2w_mic")
        self.topic = topic
        self.publisher = self.node.create_publisher(AudioChunk, topic, _LOW_LAT_QOS)
        self.subscriber = None
        self._subscribers = []
        self.state = "idle"
        self.packet_count = 0
        self.last_packet_ts = 0.0
        self.last_error = None
        self._last_dds_packet_ts = 0.0
        self._config = config or {}
        self._alsa_thread = None
        self._alsa_stop = threading.Event()
        self._publish_lock = threading.Lock()
        self._publish_buffer = bytearray()
        self.backend = "dds"

    def start(self):
        if self.state == "running":
            return self.topic
        topics = self._config.get(
            "dds_topics", ["rt/audiosender", "rt/lf/audiosender", "rt/audio"])
        if isinstance(topics, str):
            topics = [topics]
        self._subscribers = []
        for topic in topics:
            try:
                subscriber = ChannelSubscriber(topic, AudioData_)
                subscriber.Init(lambda msg, source=topic: self._on_audio(msg, source), 1)
                self._subscribers.append(subscriber)
            except Exception:
                continue
        self.subscriber = self._subscribers[0] if self._subscribers else None
        with self._publish_lock:
            self._publish_buffer.clear()
        self._active_dds_topic = None
        self.state = "running"
        if self._config.get("backend", "auto") in ("auto", "alsa"):
            self._alsa_thread = threading.Thread(target=self._alsa_fallback,
                                                 daemon=True, name="as2w-mic-alsa")
            self._alsa_stop.clear()
            self._alsa_thread.start()
        return self.topic

    def stop(self):
        for subscriber in self._subscribers:
            try:
                subscriber.Close()
            except Exception:
                pass
        self._subscribers = []
        self.subscriber = None
        self._alsa_stop.set()
        if self._alsa_thread is not None:
            self._alsa_thread.join(timeout=1)
            self._alsa_thread = None
        self.state = "idle"

    def _publish_pcm(self, payload):
        with self._publish_lock:
            self._publish_buffer.extend(payload)
            while len(self._publish_buffer) >= 1024:
                chunk = bytes(self._publish_buffer[:1024])
                del self._publish_buffer[:1024]
                self.publisher.publish(_audio_chunk(chunk))
                self.packet_count += 1

    def _on_audio(self, msg, source=None):
        try:
            payload = bytes(getattr(msg, "data", []))
        except (TypeError, ValueError) as exc:
            self.last_error = f"invalid audio packet from {source}: {str(exc)[:120]}"
            return
        if not payload:
            return
        # Subscribe to the firmware names used by different AS2 images, but
        # publish from only one active source at a time if both are bridged.
        now = time.monotonic()
        active = getattr(self, "_active_dds_topic", None)
        if active is not None and source != active and now - self.last_packet_ts < 1.0:
            return
        self._active_dds_topic = source
        if self.backend == "alsa":
            # Drop a partial fallback frame before resuming the firmware
            # stream; never mix samples from two capture backends.
            with self._publish_lock:
                self._publish_buffer.clear()
        self._publish_pcm(payload)
        self.last_packet_ts = now
        self._last_dds_packet_ts = now
        self.backend = "dds"
        self.last_error = None

    def _alsa_fallback(self):
        # AS2 firmware may advertise rt/audiosender without publishing it
        # until its voice capture service is enabled. Use the board capture
        # device in that case so the mic card remains useful on this hardware.
        try:
            import alsaaudio
            configured = self._config.get("alsa_device", "auto")
            if configured != "auto":
                devices = [configured]
            else:
                try:
                    devices = list(alsaaudio.pcms(alsaaudio.PCM_CAPTURE))
                except Exception:
                    devices = []
                devices += ["default", "plughw:1,0", "plughw:1,1", "hw:1,0", "hw:1,1"]
            devices = list(dict.fromkeys(devices))
            pcm = None
            sample_rate = 16000
            channels = 1
            for device in devices:
                for rate, channel_count in ((16000, 1), (48000, 1), (48000, 2), (44100, 1)):
                    candidate = None
                    try:
                        candidate = alsaaudio.PCM(alsaaudio.PCM_CAPTURE, alsaaudio.PCM_NONBLOCK,
                                                  device=device)
                        candidate.setchannels(channel_count)
                        candidate.setrate(rate)
                        candidate.setformat(alsaaudio.PCM_FORMAT_S16_LE)
                        candidate.setperiodsize(512 if rate == 16000 else 1024)
                        pcm = candidate
                        sample_rate, channels = rate, channel_count
                        break
                    except Exception:
                        if candidate is not None:
                            try:
                                candidate.close()
                            except Exception:
                                pass
                if pcm is not None:
                    break
            if pcm is None:
                self.last_error = "no usable ALSA capture device"
                return
        except Exception as exc:
            self.last_error = f"ALSA fallback unavailable: {str(exc)[:120]}"
            return
        deadline = time.monotonic() + float(self._config.get("dds_grace_s", 2.0))
        import audioop
        audio_state = None
        try:
            while not self._alsa_stop.is_set():
                try:
                    length, data = pcm.read()
                except Exception:
                    # Non-blocking ALSA reports an empty period as EAGAIN on
                    # some board images; keep polling without killing mic.
                    self._alsa_stop.wait(0.01)
                    continue
                if length <= 0 or not data:
                    self._alsa_stop.wait(0.01)
                    continue
                if time.monotonic() < deadline:
                    continue
                if time.monotonic() - self._last_dds_packet_ts < 1.0:
                    # Prefer the firmware stream whenever it is alive. Keep
                    # the ALSA device open so fallback resumes if DDS stops.
                    continue
                if self.backend == "dds":
                    # DDS has gone quiet. Discard its incomplete frame once
                    # before switching to ALSA so the PCM streams do not mix.
                    with self._publish_lock:
                        self._publish_buffer.clear()
                    self.backend = "alsa"
                if channels == 2:
                    # Downmix little-endian signed stereo to mono.
                    data = audioop.tomono(data, 2, 0.5, 0.5)
                if sample_rate != 16000:
                    data, audio_state = audioop.ratecv(data, 2, 1, sample_rate, 16000, audio_state)
                self._publish_pcm(data)
                self.last_packet_ts = time.monotonic()
                self.backend = "alsa"
                self.last_error = None
        finally:
            try:
                pcm.close()
            except Exception:
                pass


class MicPlugin:
    PREFIX = "mic"

    def __init__(self, config, namespace, executor):
        self._topic = f"/{namespace}/mic/audio"
        self._node = _MicNode(self._topic, config)
        executor.add_node(self._node.node)

    def get_tool(self):
        return {"name": "mic", "type": "sensor", "multiInstance": False,
                "description": f"As2W microphone DDS stream as PCM 16kHz/16bit/mono: {self._topic}",
                "inputSchema": {"type": "object", "properties": {}},
                "topic_out": [{"topic": self._topic, "format": "audio/pcm-16k"}]}

    def start(self):
        self._node.start()

    def stop(self):
        self._node.stop()

    def dispatch(self, action, args):
        if action in ("start", "mic"):
            self._node.start()
            return {"state": "running", "topic": self._topic}
        if action == "stop":
            self._node.stop()
            return {"state": "idle"}
        if action == "info":
            age = None
            if self._node.last_packet_ts:
                age = max(0.0, time.monotonic() - self._node.last_packet_ts)
            return {"state": self._node.state,
                    "topic_out": [{"topic": self._topic, "format": "audio/pcm-16k"}],
                    "packets": self._node.packet_count,
                    "backend": self._node.backend,
                    "packet_age_s": age,
                    "dds_topics": list(self._node._config.get(
                        "dds_topics", ["rt/audiosender", "rt/lf/audiosender", "rt/audio"])),
                    "error": getattr(self._node, "last_error", None)}
        return None


class _SpeakerNode:
    """Subscribe to AudioChunk and stream bounded PCM blocks through AudioClient."""

    def __init__(self, audio_client):
        from rclpy.node import Node
        import queue
        self.node = Node("as2w_speaker")
        self._client = audio_client
        self._queue = queue.Queue(maxsize=SPEAKER_QUEUE_BLOCKS)
        self._subscription = None
        self._stop_event = threading.Event()
        self._thread = None
        self.topic = None
        self.state = "idle"
        self.blocks_sent = 0
        self._next_play_time = 0.0
        self._last_play_error = 0.0
        self.last_play_error = None

    def start(self, topic):
        if self._thread is not None and self._thread.is_alive():
            if self.topic == topic:
                return topic
            self.stop()
            if self._thread is not None and self._thread.is_alive():
                raise RuntimeError("speaker worker is still stopping; retry start after it exits")
        if self._subscription is not None:
            if self.topic == topic:
                return topic
            self.stop()
            if self._thread is not None and self._thread.is_alive():
                raise RuntimeError("speaker worker is still stopping; retry start after it exits")
        try:
            # AudioClient keeps a stream session on the robot. Clear a stale
            # session left by a previous container before accepting new audio.
            self._client.Audio_PlayStop(SPEAKER_APP_NAME)
        except Exception:
            pass
        self.topic = topic
        self._subscription = self.node.create_subscription(
            AudioChunk, topic, self._on_chunk, _LOW_LAT_QOS)
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._drain, daemon=True, name="as2w-speaker")
        self._thread.start()
        self.state = "ready"
        return topic

    def stop(self):
        import queue
        if self._subscription is not None:
            try:
                self.node.destroy_subscription(self._subscription)
            except Exception:
                pass
            self._subscription = None
        self._stop_event.set()
        try:
            self._queue.put_nowait(None)
        except queue.Full:
            self._clear_queue()
            self._queue.put_nowait(None)
        thread = self._thread
        if thread is not None:
            thread.join(timeout=5)
            if not thread.is_alive():
                self._thread = None
        self._clear_queue()
        self._next_play_time = 0.0
        try:
            self._client.Audio_PlayStop(SPEAKER_APP_NAME)
        except Exception:
            pass
        self.state = "idle"

    def _clear_queue(self):
        import queue
        while True:
            try:
                self._queue.get_nowait()
            except queue.Empty:
                return

    def _on_chunk(self, msg):
        import queue
        payload = bytes(getattr(msg, "data", []))
        if payload:
            try:
                self._queue.put_nowait(payload)
            except queue.Full:
                # The source is live audio; preserving old audio would make
                # latency grow without bound. Drop the oldest block instead.
                try:
                    self._queue.get_nowait()
                except queue.Empty:
                    pass
                try:
                    self._queue.put_nowait(payload)
                except queue.Full:
                    return
            self.state = "playing"

    def _drain(self):
        import queue
        merged = bytearray()
        while not self._stop_event.is_set():
            try:
                item = self._queue.get(timeout=0.1)
            except queue.Empty:
                if merged:
                    self._play_block(bytes(merged))
                    merged.clear()
                continue
            if item is None:
                break
            merged.extend(item)
            if len(merged) >= SPEAKER_BLOCK_BYTES:
                self._play_block(bytes(merged))
                merged.clear()
        if merged and not self._stop_event.is_set():
            self._play_block(bytes(merged))
        if self.state == "playing":
            self.state = "ready"

    def _play_block(self, payload):
        started = time.monotonic()
        try:
            result = self._client.Audio_PlayStream(SPEAKER_APP_NAME, "0", payload)
            if isinstance(result, tuple) and len(result) == 2:
                code, detail = result
            else:
                code, detail = result, None
            if code != 0:
                self._record_play_error(code, detail)
                return result
            self.last_play_error = None
            self.blocks_sent += 1
        except Exception as exc:
            self._record_play_error("exception", str(exc))
            return None
        # Keep at most 240 ms of audio ahead of the robot decoder.  Without a
        # cumulative deadline, fast RPC responses can overrun the firmware's
        # stream buffer on longer utterances.
        duration = len(payload) / 32000.0
        self._next_play_time = max(getattr(self, "_next_play_time", 0.0), started) + duration
        wait_for = self._next_play_time - 0.24 - time.monotonic()
        if wait_for > 0:
            self._stop_event.wait(wait_for)
        return result

    def _record_play_error(self, code, detail):
        self.last_play_error = {"code": code, "detail": str(detail)[:160]}
        now = time.monotonic()
        previous = getattr(self, "_last_play_error", 0.0)
        if now - previous >= 10.0:
            print(f"[speaker] PlayStream failed: code={code} detail={str(detail)[:160]}", flush=True)
            self._last_play_error = now


class SpeakerPlugin:
    PREFIX = "speaker"

    def __init__(self, config, namespace, executor, audio_client):
        self._node = _SpeakerNode(audio_client)
        configured_topic = (config or {}).get("input_topic")
        if configured_topic:
            self._input_topic = str(configured_topic).replace("{namespace}", namespace)
        else:
            # A speaker card must be startable without a hand-written MCP
            # argument.  External producers can publish AudioChunk messages
            # here; an explicit input_topic still overrides this default.
            self._input_topic = f"/{namespace}/speaker/audio"
        executor.add_node(self._node.node)

    def get_tool(self):
        input_topic = getattr(self, "_input_topic", "/speaker/audio")
        return {"name": "speaker", "type": "actuator", "multiInstance": False,
                "description": f"As2W speaker: subscribes to PCM 16kHz/16bit/mono AudioChunk stream. Default input topic: {input_topic}",
                "inputSchema": {"type": "object", "properties": {
                    "action": {"type": "string", "enum": ["start", "stop", "info", "get_volume", "set_volume"]},
                    "input_topic": {"type": "string", "description": f"Optional ROS2 AudioChunk topic; defaults to {input_topic}"},
                    "volume": {"type": "integer", "minimum": 0, "maximum": 100}},
                    "required": ["action"],
                    "x-action-params": {
                    "start": {"params": [], "description": "Start playback on the configured AudioChunk topic; input_topic may override it."},
                    "stop": {"params": [], "description": "Stop playback and clear buffered audio."},
                    "get_volume": {"params": [], "description": "Read the current volume."},
                    "set_volume": {"params": ["volume"], "description": "Set volume from 0 to 100."}}},
                "topic_in": [{"topic": input_topic, "format": "audio/pcm-16k"}]}

    def start(self):
        pass

    def stop(self):
        self._node.stop()

    def dispatch(self, action, args):
        if action in ("start", "play", "speaker"):
            topic = args.get("input_topic") or args.get("topic_in") or getattr(self, "_input_topic", "/speaker/audio")
            if isinstance(topic, dict):
                topic = topic.get("topic")
            elif isinstance(topic, (list, tuple)):
                topic = topic[0] if topic else None
            if not topic:
                topic = getattr(self, "_input_topic", "/speaker/audio")
            try:
                started_topic = self._node.start(topic)
            except RuntimeError as exc:
                return {"state": self._node.state, "accepted": False,
                        "error": str(exc),
                        "reason": "The previous playback RPC is still in flight",
                        "suggested_actions": ["stop", "retry"]}
            return {"state": "ready", "topic": started_topic,
                    "input_topic": started_topic}
        if action == "stop":
            self._node.stop()
            return {"state": "idle"}
        if action == "info":
            return {"state": self._node.state, "topic": self._node.topic,
                    "blocks_sent": self._node.blocks_sent,
                    "last_error": self._node.last_play_error}
        if action == "get_volume":
            result = self._node._client.Audio_GetVolume()
            if isinstance(result, tuple) and len(result) == 2:
                code, volume = result
            else:
                code, volume = result, None
            return {"ret": code, "volume": volume}
        if action == "set_volume":
            volume = max(0, min(100, int(args.get("volume", 50))))
            return {"ret": self._node._client.Audio_SetVolume(volume), "volume": volume}
        return None


class _CameraRgbNode:
    """Poll AS2 videohub snapshots and publish JPEG CompressedImage frames."""

    def __init__(self, topic, proxy, fps=5.0):
        from rclpy.node import Node
        self.node = Node("as2w_camera_rgb")
        self.topic = topic
        self.proxy = proxy
        self.publisher = self.node.create_publisher(CompressedImage, topic, _CAMERA_QOS)
        self.period = 1.0 / max(0.5, min(15.0, float(fps)))
        self._stop_event = threading.Event()
        self._thread = None
        self.state = "idle"
        self.frames = 0
        self.last_frame_ts = 0.0
        self.last_error = None

    def start(self):
        if self._thread is not None and self._thread.is_alive():
            return
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._loop, daemon=True, name="as2w-camera-rgb")
        self._thread.start()
        self.state = "running"

    def stop(self):
        self._stop_event.set()
        thread = self._thread
        if thread is not None:
            thread.join(timeout=6)
            if not thread.is_alive():
                self._thread = None
        self.state = "idle"

    def _loop(self):
        while not self._stop_event.is_set():
            try:
                result = self.proxy.Video_GetImageSample()
            except Exception as exc:
                self.last_error = str(exc)
                self._stop_event.wait(self.period)
                continue
            if self._stop_event.is_set():
                break
            code, payload = result if isinstance(result, tuple) and len(result) == 2 else (3104, None)
            if code == 0 and payload:
                message = CompressedImage()
                message.header.stamp = self.node.get_clock().now().to_msg()
                message.format = "jpeg"
                message.data = list(payload)
                self.publisher.publish(message)
                self.frames += 1
                self.last_frame_ts = time.monotonic()
                self.last_error = None
            elif code != 0:
                self.last_error = f"videohub returned {code}"
            self._stop_event.wait(self.period)


class CameraPlugin:
    PREFIX = "camera_rgb"

    def __init__(self, config, namespace, executor, proxy):
        self._topic = f"/{namespace}/camera/rgb"
        self._node = _CameraRgbNode(self._topic, proxy, config.get("fps", 5))
        executor.add_node(self._node.node)

    def get_tool(self):
        return {"name": "camera_rgb", "type": "sensor", "multiInstance": False,
                "description": f"AS2 videohub RGB JPEG stream at up to 5 FPS: {self._topic}",
                "inputSchema": {"type": "object", "properties": {}},
                "topic_out": [{"topic": self._topic, "format": "image/jpeg"}]}

    def start(self):
        self._node.start()

    def stop(self):
        self._node.stop()

    def dispatch(self, action, args):
        if action in ("start", "camera_rgb"):
            self._node.start()
            return {"state": "running", "topic": self._topic}
        if action == "stop":
            self._node.stop()
            return {"state": "idle"}
        if action == "info":
            return {"state": self._node.state, "frames": self._node.frames,
                    "last_error": self._node.last_error,
                    "topic_out": [{"topic": self._topic, "format": "image/jpeg"}]}
        return None


class LedPlugin:
    PREFIX = "led"

    def __init__(self, config, namespace, executor, proxy):
        self._proxy = proxy
        self._color = [0, 0, 0]
        self._keepalive_stop = threading.Event()
        self._keepalive_thread = None

    def get_tool(self):
        return {"name": "led", "type": "actuator", "multiInstance": False,
                "description": "AS2 RGB LED. The selected color is refreshed periodically because AS2 firmware may time out LED state.",
                "inputSchema": {"type": "object", "properties": {
                    "action": {"type": "string", "enum": ["set_color", "off", "info"]},
                    "red": {"type": "integer", "minimum": 0, "maximum": 255, "description": "Red channel 0-255."},
                    "green": {"type": "integer", "minimum": 0, "maximum": 255, "description": "Green channel 0-255."},
                    "blue": {"type": "integer", "minimum": 0, "maximum": 255, "description": "Blue channel 0-255."}},
                    "required": ["action"],
                    "x-action-params": {
                        "set_color": {"params": ["red", "green", "blue"], "description": "Set RGB color."},
                        "off": {"params": [], "description": "Turn the LED off."},
                        "info": {"params": [], "description": "Read the selected color."}}}}

    def start(self):
        # Do not send a black LED command every 0.7 seconds during bundle
        # startup.  Apart from being unnecessary, that used to occupy the
        # shared voice RPC worker and made live speaker audio appear silent.
        if not any(self._color):
            return
        self._keepalive_stop.clear()
        if self._keepalive_thread is None or not self._keepalive_thread.is_alive():
            self._keepalive_thread = threading.Thread(target=self._keepalive,
                                                       daemon=True, name="as2w-led")
            self._keepalive_thread.start()

    def stop(self):
        self._keepalive_stop.set()
        if self._keepalive_thread is not None:
            self._keepalive_thread.join(timeout=1)
            self._keepalive_thread = None

    def _keepalive(self):
        while not self._keepalive_stop.is_set():
            color = tuple(self._color)
            try:
                self._proxy.Audio_LedControl(*color)
            except Exception as exc:
                now = time.monotonic()
                previous = getattr(self, "_last_error", 0.0)
                if now - previous >= 10.0:
                    print(f"[led] refresh failed: {str(exc)[:160]}", flush=True)
                    self._last_error = now
            self._keepalive_stop.wait(0.7)

    def dispatch(self, action, args):
        if action in ("start", "info"):
            return {"state": "ready", "color": list(self._color)}
        if action == "stop":
            self.stop()
            return {"state": "idle"}
        if action == "off":
            color = (0, 0, 0)
        elif action == "set_color":
            color = tuple(max(0, min(255, int(args.get(key, 0))))
                          for key in ("red", "green", "blue"))
        else:
            return None
        self._color = list(color)
        if action == "off":
            self.stop()
        result = self._proxy.Audio_LedControl(*color)
        if action != "off" and result == 0:
            self.start()
        return {"ret": result, "color": list(color)}
