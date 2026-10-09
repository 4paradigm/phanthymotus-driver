"""Unitree As2W driver plugins (official AS2 SDK SportClient)."""
import copy
import json
import math
import threading
import time
from uuid import uuid4

from std_msgs.msg import String
from unitree_sdk2py.core.channel import ChannelSubscriber
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import BmsState_, LowState_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_

try:
    from odom_specs import OdomAdapter
except ModuleNotFoundError as exc:
    if exc.name != "odom_specs":
        raise
    from unitree.as2w.odom_specs import OdomAdapter


_LOW_LAT_QOS = None
try:
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    _LOW_LAT_QOS = QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        durability=DurabilityPolicy.VOLATILE,
    )
except ImportError:
    # Unit tests load the card contracts without a ROS installation.
    pass


_STATE_PUBLISH_HZ = 60.0


def _number(value, default=0.0):
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def _finite_number(value):
    """A missing/invalid measurement must never be reported as stopped."""
    if isinstance(value, bool):
        return None
    try:
        result = float(value)
        return result if math.isfinite(result) else None
    except (TypeError, ValueError, OverflowError):
        return None


def _values(value):
    try:
        return list(value)
    except TypeError:
        return [value]


def _acp_notify(action_id, status, result, tool="loco"):
    import os, ssl, urllib.request
    endpoint = f"{os.environ.get('AGENT_CORE_URL', 'https://localhost:15678')}/api/acp/complete"
    payload = json.dumps({"action_id": action_id, "status": status,
                          "result": result, "tool": tool, "ts": time.time()}).encode()
    try:
        request = urllib.request.Request(endpoint,
            data=payload, headers={"Content-Type": "application/json"}, method="POST")
        with urllib.request.urlopen(
                request, timeout=5,
                context=ssl._create_unverified_context()) as response:
            print(f"[{tool}] ACP complete action_id={action_id} status={status} http={response.status}", flush=True)
    except Exception as exc:
        print(f"[{tool}] ACP callback failed action_id={action_id} status={status} endpoint={endpoint} error={str(exc)[:180]}", flush=True)


class _StateNode:
    def __init__(self, namespace, executor, odom_config=None):
        from rclpy.node import Node
        self._odom_adapter = OdomAdapter(odom_config)
        self.node = Node("as2w_state")
        self.imu = self.node.create_publisher(String, f"/{namespace}/state/imu", _LOW_LAT_QOS or 1)
        self.joints = self.node.create_publisher(String, f"/{namespace}/state/joints", _LOW_LAT_QOS or 1)
        self.joint_state = self.node.create_publisher(String, f"/{namespace}/state/joint_state", _LOW_LAT_QOS or 1)
        self.battery = self.node.create_publisher(String, f"/{namespace}/state/battery", _LOW_LAT_QOS or 1)
        self.loco = self.node.create_publisher(String, f"/{namespace}/loco/state", _LOW_LAT_QOS or 1)
        self.odom = self.node.create_publisher(String, f"/{namespace}/state/odom", _LOW_LAT_QOS or 1)
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
        self._latest_sport_received_ms = None
        self._latest_sport_received_monotonic = None
        self._sport_stream_id = uuid4().hex
        self._last_odom_sample = None
        self._low_generation = 0
        self._bms_generation = 0
        self._sport_generation = 0
        self._published_low_generation = -1
        self._published_bms_generation = -1
        self._published_sport_generation = -1
        self._stop_event = threading.Event()
        self._publisher_thread = threading.Thread(
            target=self._publish_loop, daemon=True, name="as2w-state-publish")
        # queueLen=1 in the bundled SDK rejects new samples when full; it is
        # not a latest-value queue. LowState is high-rate, so consume it
        # directly and let the callback replace the cached pointer.
        self._low.Init(self._on_low, 0)
        self._bms.Init(self._on_bms, 0)
        self._sport.Init(self._on_sport, 0)
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
                sport_received_ms = self._latest_sport_received_ms
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
                self._publish_sport(sport, received_ms=sport_received_ms)
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
        # Timestamp at DDS receipt, not when the publisher thread gets CPU.
        # Otherwise a queued old sample would look like a current measurement.
        received_ms = int(time.time() * 1000)
        received_monotonic = time.monotonic()
        latest_lock = getattr(self, "_latest_lock", None)
        if latest_lock is None:
            self._publish_sport(msg, received_ms=received_ms)
            return
        with latest_lock:
            self._latest_sport = msg
            self._latest_sport_received_ms = received_ms
            self._latest_sport_received_monotonic = received_monotonic
            self._sport_generation += 1
        if getattr(self, "_publisher_thread", None) is None:
            self._publish_sport(msg, received_ms=received_ms)

    def motion_snapshot(self):
        with self._latest_lock:
            msg = self._latest_sport
            if msg is None or self._stop_event.is_set():
                return None
            try:
                velocity = [_finite_number(v) for v in msg.velocity]
            except (AttributeError, TypeError):
                velocity = None
            return {"velocity": velocity,
                    "yaw_speed": _finite_number(getattr(msg, "yaw_speed", None)),
                    "received_monotonic": self._latest_sport_received_monotonic,
                    "timestamp": (self._latest_sport_received_ms / 1000.0 if self._latest_sport_received_ms is not None else None),
                    "stream_id": self._sport_stream_id,
                    "generation": self._sport_generation}

    def _publish_sport(self, msg, received_ms=None):
        loco = {"mode": int(getattr(msg, "mode", 0)),
                "body_height": _number(getattr(msg, "body_height", 0)),
                "yaw_speed": _finite_number(getattr(msg, "yaw_speed", None)),
                "timestamp": received_ms / 1000.0 if received_ms is not None else None}
        loco.update(self._flat("velocity", getattr(msg, "velocity", [])))
        loco.update(self._flat("position", getattr(msg, "position", [])))
        self._publish(self.loco, loco)
        # Retain the established raw topic. This second port is the explicit
        # contract for consumers that compare commanded and measured motion.
        if getattr(self, "odom", None) is not None:
            sample = self._odom_adapter.sample(
                msg, received_ms=(int(time.time() * 1000)
                                  if received_ms is None else received_ms))
            self._publish(self.odom, sample)
            with self._latest_lock:
                self._last_odom_sample = sample

    def odom_status(self):
        with self._latest_lock:
            sample = self._last_odom_sample
            received_at = self._latest_sport_received_monotonic
            received_samples = self._sport_generation
        age_ms = (int(time.time() * 1000) - sample["stamp_ms"]
                  if sample is not None else None)
        receive_age_ms = (max(0, int((time.monotonic() - received_at) * 1000))
                          if received_at is not None else None)
        fresh = (age_ms is not None and receive_age_ms is not None
                 and 0 <= age_ms <= self._odom_adapter.max_age_ms
                 and receive_age_ms <= self._odom_adapter.max_age_ms)
        return {"received_samples": received_samples,
                "sample_age_ms": age_ms, "receive_age_ms": receive_age_ms,
                "fresh": fresh, "max_age_ms": self._odom_adapter.max_age_ms,
                "stamp_source": (sample["vendor"]["stamp_source"]
                                 if sample is not None else None)}

    def odom_snapshot(self):
        """Return a detached sample without renewing its measurement stamp."""
        with self._latest_lock:
            return copy.deepcopy(self._last_odom_sample)


class StatePlugin:
    PREFIX = "state"
    def __init__(self, config, namespace, executor):
        self._namespace = namespace
        self._executor = executor
        self._odom_config = config.get("odom", {})
        self._odom_adapter = OdomAdapter(self._odom_config)
        self._state = self._create_node()

    def _create_node(self):
        config = getattr(self, "_odom_config", {})
        if config:
            return _StateNode(self._namespace, self._executor, odom_config=config)
        return _StateNode(self._namespace, self._executor)

    def _odom_interface(self):
        adapter = getattr(self, "_odom_adapter", None) or OdomAdapter()
        return adapter.interface(publish_hz=_STATE_PUBLISH_HZ)

    def odom_snapshot(self):
        # A concurrent stop may clear self._state; retain one local reference.
        state = self._state
        return state.odom_snapshot() if state is not None else None

    def motion_snapshot(self):
        # Resolve on every call: stopping/starting sensor cards replaces _state.
        state = self._state
        return state.motion_snapshot() if state is not None else None

    def get_tools(self):
        specs = (("imu", "state/imu", "data/json", "As2W IMU state"),
                 ("joints", "state/joints", "sensor/skeleton", "As2W 12-joint leg skeleton for model animation"),
                 ("joint_state", "state/joint_state", "data/json", "As2W raw motor position, velocity, torque, and temperature"),
                 ("battery", "state/battery", "data/json", "As2W BMS state; current_ma is mA"),
                 ("loco_state", "loco/state", "data/json", "As2W high-level locomotion state"))
        tools = []
        for name, path, fmt, desc in specs:
            tool = {"name": name, "type": "sensor", "multiInstance": False,
                    "description": desc,
                    "inputSchema": {"type": "object", "properties": {}},
                    "topic_out": [{"topic": f"/{self._namespace}/{path}", "format": fmt}]}
            if name == "loco_state":
                tool["topic_out"].append(
                    {"topic": f"/{self._namespace}/state/odom", "format": "state/odom"})
                tool["odom_interface"] = self._odom_interface()
                tool["description"] += (
                    "; also motus.odom/1. Unverified velocity axes are null; no pose is claimed")
            tools.append(tool)
        return tools
    def start(self):
        # Sensor cards share one LowState subscription.  Recreate it when a
        # dashboard stopped the card instead of claiming a dead stream is live.
        if self._state is None:
            self._state = self._create_node()

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
        if action == "info" or action in ("imu", "joints", "joint_state", "battery", "loco_state"):
            name = args.get("_tool_name") if action == "info" else action
            state = getattr(self, "_state", True)
            result = {"state": "running" if state is not None else "idle"}
            tool = next((t for t in self.get_tools() if t["name"] == name), None)
            if tool is not None:
                result["topic_out"] = tool["topic_out"]
            if name == "loco_state":
                result["odom_interface"] = self._odom_interface()
                result["odom_status"] = (state.odom_status()
                    if hasattr(state, "odom_status") else {
                        "received_samples": 0, "sample_age_ms": None,
                        "receive_age_ms": None, "fresh": False,
                        "max_age_ms": self._odom_interface()["vendor"]["max_age_ms"],
                        "stamp_source": None})
            return result
        return None


class LocoPlugin:
    PREFIX = "loco"
    def __init__(self, config, namespace, executor, proxy, motion_snapshot=None):
        self.proxy = proxy
        self._motion_snapshot = motion_snapshot
        self._lock = threading.Lock()
        self._stop = None
        self._transition_stop = None

    def is_moving(self):
        with self._lock:
            active = any(event is not None and not event.is_set()
                         for event in (self._stop, self._transition_stop))
        checker = getattr(self.proxy, "legacy_motion_active", None)
        return active or (bool(checker()) if callable(checker) else False)

    _STANDING = {"STAND_UP", "BALANCE_STAND", "RECOVERY_STAND", "STANDING",
                 "AI_STAND_UP", "AI_BALANCE_STAND", "AI_RECOVERY_STAND"}
    # AS2 reports AI_STAND_UP after StandUp. It is already a usable standing
    # posture; calling BalanceStand here creates an unnecessary mode change.
    _BALANCE_REQUIRED = {"PASSIVE", "STAND"}
    _MOVING = {"WALK", "WALKING", "RUN", "RUNNING", "MOVE", "MOVING",
               "REGULAR_WALK", "REGULAR_RUN", "AI_FREE_WALK", "AI_WALK",
               "AI_RUN"}
    _DOWN = {"STAND_DOWN", "DAMPING", "LYING", "FALL", "FALLEN",
             "SQUAT", "AI_STAND_DOWN", "AI_DAMPING", "AI_FALL",
             "AI_FALLEN"}

    @classmethod
    def _is_moving(cls, state):
        return (state in cls._MOVING or "WALK" in state or "RUN" in state or
                "MOVE" in state or state.endswith("FREE_WALK"))

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

    def _motion_status(self):
        try:
            sample = self._motion_snapshot() if self._motion_snapshot else None
        except Exception:
            sample = None
        sample = sample if isinstance(sample, dict) else {}
        received = _finite_number(sample.get("received_monotonic"))
        velocity = sample.get("velocity")
        velocity = ([_finite_number(v) for v in velocity]
                    if isinstance(velocity, (list, tuple)) else [])
        yaw_speed = _finite_number(sample.get("yaw_speed"))
        age = time.monotonic() - received if received is not None else None
        valid = len(velocity) == 3 and None not in velocity and yaw_speed is not None
        fresh = age is not None and 0 <= age <= 0.5
        return {"valid": valid, "fresh": fresh, "age_sec": age,
                "velocity": velocity, "yaw_speed": yaw_speed,
                "timestamp": sample.get("timestamp"),
                "generation": sample.get("generation"),
                "stationary_sample": (math.hypot(*velocity) <= 0.03 and abs(yaw_speed) <= 0.05
                                      if valid and fresh else None)}

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
            if name in {"BALANCE_STAND", "AI_BALANCE_STAND"}:
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

    def _start_balance_move(self, state_name, vx, vy, yaw, duration):
        """Start the hidden AS2 balance transition and return an ACP action."""
        self._cancel_transition()
        balance_ret = self.proxy.BalanceStand()
        if balance_ret != 0:
            return None, self._rpc_rejected(
                "move", state_name, balance_ret,
                "Move was refused and the automatic balance-stand transition was also refused",
                ["get_state", "stand_up", "balance_stand", "recovery_stand"])
        action_id = f"as2w_loco_{uuid4().hex[:8]}"
        transition_stop = threading.Event()
        with self._lock:
            self._transition_stop = transition_stop
        threading.Thread(
            target=self._transition_to_balance_and_move,
            args=(action_id, vx, vy, yaw, duration, transition_stop),
            daemon=True, name="as2w-loco-balance-move").start()
        return action_id, {
            "ret": 0, "accepted": True, "status": "running",
            "action": "move", "transition": "balance_stand",
            "action_id": action_id, "current_state": state_name}

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
            move_ret = 0
            move_error = None
            try:
                while not stop_event.is_set():
                    try:
                        move_ret = self.proxy.Move(vx, vy, yaw)
                    except Exception as exc:
                        move_ret = 3104
                        move_error = f"{type(exc).__name__}: {str(exc)[:160]}"
                    if move_ret != 0:
                        break
                    stop_event.wait(0.1)
            finally:
                try:
                    self.proxy.StopMove()
                except Exception:
                    pass
                with self._lock:
                    if self._stop is stop_event:
                        self._stop = None
                self._finish_transition(stop_event)
            if move_ret != 0 and not stop_event.is_set():
                result = {
                    "action": "move",
                    "ret": move_ret,
                    "rpc_ret": move_ret,
                    "current_state": "BALANCE_STAND",
                    "error": "Continuous Move failed",
                    "reason": "The sport controller stopped accepting the velocity command",
                    "suggested_actions": ["get_state", "stop_move"],
                }
                if move_error:
                    result["rpc_error"] = move_error
                _acp_notify(action_id, "error", result)
            else:
                _acp_notify(action_id, "cancelled", {
                    "action": "move",
                    "ret": 0,
                    "duration": -1,
                    "reason": "Continuous move stopped by stop_move or card shutdown",
                })
            return
        deadline = time.monotonic() + duration
        stop_ret = 0
        stop_error = None
        try:
            while time.monotonic() < deadline:
                remaining = deadline - time.monotonic()
                if stop_event.wait(min(0.1, remaining)):
                    _acp_notify(action_id, "cancelled", {
                        "action": "move", "duration": duration,
                        "reason": "Timed move stopped by stop_move or card shutdown"})
                    return
                if time.monotonic() < deadline:
                    ret = self.proxy.Move(vx, vy, yaw)
                    if ret != 0:
                        _acp_notify(action_id, "error", {
                            "action": "move", "ret": ret, "rpc_ret": ret,
                            "duration": duration,
                            "error": "Timed Move failed",
                            "reason": "The sport controller stopped accepting the velocity command",
                            "suggested_actions": ["get_state", "stop_move"]})
                        return
        finally:
            try:
                stop_ret = self.proxy.StopMove()
            except Exception as exc:
                stop_ret = 3104
                stop_error = f"{type(exc).__name__}: {str(exc)[:160]}"
            finally:
                self._finish_transition(stop_event)
        if stop_ret != 0:
            result = {"action": "move", "ret": stop_ret, "rpc_ret": stop_ret,
                      "duration": duration, "error": "Timed Move final stop failed",
                      "reason": "Requested duration elapsed, but StopMove was not accepted",
                      "suggested_actions": ["get_state", "stop_move"]}
            if stop_error:
                result["rpc_error"] = stop_error
            _acp_notify(action_id, "error", result)
            return
        _acp_notify(action_id, "completed",
                    {"action": "move", "ret": 0, "duration": duration,
                     "stop_accepted": True, "stopped_confirmed": False,
                     "reason": "requested duration elapsed; stop command accepted, physical stop not yet verified"})

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
                "description": "As2W locomotion. move uses vx forward/back m/s, vy lateral m/s, vyaw rotation rad/s, and duration seconds (-1 means continue until stop_move). stand_up/stand_down change posture; balance_stand enables active balance; damp releases motor torque; recovery_stand is for fallen/down posture; speed_level accepts slow/normal/fast; body_height/body_position/euler are direct controller offsets. The flag actions are explicitly documented below.", "inputSchema": {"type": "object", "properties": {
                    "action": {"type": "string", "enum": actions, "description": "Locomotion action"}, "vx": {"type": "number", "description": "Forward velocity m/s [-1.5, 1.5]"}, "vy": {"type": "number", "description": "Lateral velocity m/s [-1, 1]"}, "vyaw": {"type": "number", "description": "Yaw velocity rad/s [-2, 2]"},
                    "duration": {"type": "number", "minimum": -1, "maximum": 30, "description": "Seconds; -1 continues until stop_move"}, "roll": {"type": "number", "description": "Body roll radians"}, "pitch": {"type": "number", "description": "Body pitch radians"}, "yaw": {"type": "number", "description": "Body yaw radians"},
                    "speed_preset": {"type": "string", "enum": ["slow", "normal", "fast"], "description": "Speed limiter preset"}, "height": {"type": "number", "description": "Body height offset"}, "x": {"type": "number", "description": "Body X offset"}, "y": {"type": "number", "description": "Body Y offset"}, "z": {"type": "number", "description": "Body Z offset"}, "flag": {"type": "boolean", "description": "Used by four switch actions: true enables/enters and false disables/exits."}}, "required": ["action"],
                "x-completion": {"actions": ["move", "stop_move", "stand_up", "stand_down", "balance_stand", "recovery_stand"], "timeout": 45},
                "x-action-params": {
                    "move": {"params": ["vx", "vy", "vyaw", "duration"], "description": "Move with optional duration (-1 for continuous)."},
                    "stop_move": {"params": [], "description": "Stop movement."},
                    "stand_up": {"params": [], "description": "Stand up."}, "stand_down": {"params": [], "description": "Stand down."},
                    "balance_stand": {"params": [], "description": "Balance stand."}, "recovery_stand": {"params": [], "description": "Recovery stand."},
                    "damp": {"params": [], "description": "Damp motors."}, "euler": {"params": ["roll", "pitch", "yaw"], "description": "Set body attitude."},
                    "speed_level": {"params": ["speed_preset"], "description": "Set speed limiter: slow, normal, or fast."}, "body_height": {"params": ["height"], "description": "Set body height offset."},
                    "body_position": {"params": ["x", "y", "z", "yaw"], "description": "Set body position offset."},
                    "switch_joystick": {"params": ["flag"], "description": "true hands control to the wireless joystick; false disables it."}, "left_side_gait": {"params": ["flag"], "description": "Enter or exit left-side gait; true enters, false exits."},
                    "right_side_gait": {"params": ["flag"], "description": "Enter or exit right-side gait; true enters, false exits."}, "auto_recovery": {"params": ["flag"], "description": "Automatic fall recovery; true enables, false disables."},
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
            expected_states = {expected_name.upper(), f"AI_{expected_name.upper()}"}
            if left_old_state and name in expected_states:
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
                    "suggested_actions": ["get_state", "stop_move", "recovery_stand"],
                    "observed_state": state})
    def dispatch(self, action, args):
        if action in ("start", "info"): return {"state": "ready"}
        if action == "stop":
            self._cancel_transition()
            self._stop_continuous()
            return {"state": "idle", "ret": self.proxy.StopMove()}
        if action == "move":
            vx, vy, yaw = max(-1.5, min(1.5, float(args.get("vx", 0)))), max(-1, min(1, float(args.get("vy", 0)))), max(-2, min(2, float(args.get("vyaw", 0))))
            duration = args.get("duration")
            if duration == 0:
                duration = None
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
                _, result = self._start_balance_move(
                    state_name, vx, vy, yaw, duration)
                return result
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
                self._cancel_transition()
                self._stop_continuous()
                action_id = f"as2w_loco_{uuid4().hex[:8]}"
                stop_event = threading.Event()
                with self._lock:
                    self._stop = stop_event
                ret = self.proxy.Move(vx, vy, yaw)
                if ret != 0:
                    with self._lock:
                        if self._stop is stop_event:
                            self._stop = None
                    if state_name in self._STANDING:
                        _, result = self._start_balance_move(
                            state_name, vx, vy, yaw, duration)
                        return result
                    return {"ret": ret, "rpc_ret": ret, "accepted": False,
                            "action": "move", "current_state": state_name,
                            "error": "Continuous Move was rejected",
                            "reason": "The sport controller refused the velocity command",
                            "suggested_actions": ["get_state", "stop_move"]}
                threading.Thread(target=self._run_continuous_move,
                                 args=(action_id, vx, vy, yaw, stop_event),
                                 daemon=True, name="as2w-loco-continuous-move").start()
                return {"ret": 0, "accepted": True, "status": "running",
                        "action": "move", "action_id": action_id,
                        "current_state": state_name, "duration": -1}
            if duration < 0: return {"ret": -1, "message": "duration must be -1, 0, or positive"}
            self._stop_continuous()
            ret = self.proxy.Move(vx, vy, yaw)
            if ret != 0:
                if state_name in self._STANDING:
                    _, result = self._start_balance_move(
                        state_name, vx, vy, yaw, duration)
                    return result
                return {"ret": ret, "rpc_ret": ret, "accepted": False,
                        "action": "move", "current_state": state_name,
                        "error": "Timed Move was rejected",
                        "reason": "The sport controller refused the velocity command",
                        "suggested_actions": ["get_state", "stop_move"]}
            action_id = f"as2w_loco_{uuid4().hex[:8]}"
            stop_event = threading.Event()
            with self._lock:
                self._transition_stop = stop_event
            threading.Thread(target=self._run_timed_move,
                             args=(action_id, vx, vy, yaw, duration, stop_event),
                             daemon=True, name="as2w-loco-timed-move").start()
            return {"ret": 0, "accepted": True, "status": "running",
                    "action": "move", "action_id": action_id,
                    "current_state": state_name, "duration": duration}
        if action == "stop_move":
            self._cancel_transition()
            self._stop_continuous()
            action_id = f"as2w_loco_{uuid4().hex[:8]}"
            threading.Thread(target=self._stop_move_worker,
                             args=(action_id,), daemon=True,
                             name="as2w-loco-stop-move").start()
            return {"ret": 0, "accepted": True, "status": "stopping",
                    "action": action, "action_id": action_id}
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
            if action == "recovery_stand" and state_name not in {
                    "FALL", "FALLEN", "STAND_DOWN", "DAMPING",
                    "AI_FALL", "AI_FALLEN", "AI_STAND_DOWN", "AI_DAMPING"}:
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
                return {"ret": code, "state": state, "motion": self._motion_status()}
            return {"ret": result if isinstance(result, int) else 3104,
                    "state": {}, "error": "Unable to read robot locomotion state",
                    "reason": "SportClient.GetState did not return a state",
                    "suggested_actions": ["retry", "recovery_stand"]}
        return None

    def _run_timed_move(self, action_id, vx, vy, yaw, duration, stop_event):
        self._run_move_after_transition(action_id, vx, vy, yaw, duration,
                                        stop_event, move_already_sent=True)

    def _run_continuous_move(self, action_id, vx, vy, yaw, stop_event):
        try:
            while not stop_event.wait(0.1):
                ret = self.proxy.Move(vx, vy, yaw)
                if ret != 0:
                    _acp_notify(action_id, "error", {
                        "action": "move", "ret": ret, "rpc_ret": ret,
                        "error": "Continuous Move failed",
                        "reason": "The sport controller stopped accepting the velocity command",
                        "suggested_actions": ["get_state", "stop_move"]})
                    return
        finally:
            with self._lock:
                if self._stop is stop_event:
                    self._stop = None
            self._finish_transition(stop_event)
        _acp_notify(action_id, "cancelled", {
            "action": "move", "ret": 0, "duration": -1,
            "reason": "Continuous move stopped by stop_move or card shutdown"})

    def _await_stopped(self, action_id, requested_at):
        # GetState reports the selected controller mode, not measured movement.
        # AI_FREE_WALK can remain selected while all measured speeds are zero.
        try:
            name, state, state_error = self._read_state()
        except Exception as exc:
            name, state = None, None
            state_error = {"reason": f"{type(exc).__name__}: {str(exc)[:160]}"}
        diagnostic = {"current_state": name or "UNKNOWN"}
        if state is not None:
            diagnostic["state"] = state
        if state_error:
            diagnostic["state_error"] = state_error
        # Diagnostic RPC latency must not consume the physical confirmation
        # window: GetState can be slow while sport telemetry remains healthy.
        deadline = time.monotonic() + 5.0
        stable_since = None
        stable_samples = 0
        last_key = None
        last_received = None
        reason = "No fresh motion telemetry available"
        while time.monotonic() < deadline:
            try:
                sample = self._motion_snapshot() if self._motion_snapshot else None
            except Exception:
                sample = None
            sample = sample if isinstance(sample, dict) else {}
            received = _finite_number(sample.get("received_monotonic"))
            yaw_speed = _finite_number(sample.get("yaw_speed"))
            generation = sample.get("generation")
            raw_velocity = sample.get("velocity")
            velocity = ([_finite_number(v) for v in raw_velocity]
                        if isinstance(raw_velocity, (list, tuple)) else [])
            now = time.monotonic()
            valid = (received is not None and received > requested_at
                     and 0 <= now - received <= 0.5
                     and type(generation) is int and generation > 0
                     and len(velocity) == 3 and None not in velocity
                     and yaw_speed is not None)
            if not valid:
                stable_since, stable_samples = None, 0
                reason = "Motion telemetry is missing, invalid, stale, or predates the stop command"
            else:
                key = (sample.get("stream_id"), generation)
                # The same cached frame must not count as multiple observations.
                if key != last_key:
                    if (last_key is not None and (
                            key[0] != last_key[0] or generation <= last_key[1]
                            or received <= last_received
                            or received - last_received > 0.5)):
                        stable_since, stable_samples = None, 0
                    if last_received is not None and received <= last_received:
                        stable_since, stable_samples = None, 0
                        reason = "Motion telemetry did not advance in time"
                    elif math.hypot(*velocity) > 0.03 or abs(yaw_speed) > 0.05:
                        stable_since, stable_samples = None, 0
                        reason = "Measured linear or yaw speed remains above the stop threshold"
                    else:
                        if stable_since is None:
                            stable_since = received
                        stable_samples += 1
                        reason = "Waiting for distinct stable motion samples"
                        if stable_samples >= 3 and received - stable_since >= 0.3:
                            print(f"[loco] stop verified action_id={action_id} "
                                  f"linear_mps={math.hypot(*velocity):.6f} "
                                  f"yaw_rad_s={yaw_speed:.6f} samples={stable_samples} "
                                  f"stable_sec={received - stable_since:.3f}", flush=True)
                            _acp_notify(action_id, "completed", {
                                "action": "stop_move", "ret": 0, **diagnostic,
                                "stopped_confirmed": True,
                                "velocity": velocity, "yaw_speed": yaw_speed,
                                "sample_age_sec": now - received,
                                "stable_samples": stable_samples,
                                "stable_duration_sec": received - stable_since,
                                "reason": "Fresh linear and yaw measurements confirm the robot stopped"})
                            return
                    last_key, last_received = key, received
            time.sleep(0.1)
        _acp_notify(action_id, "error", {
            "action": "stop_move", "ret": 0, **diagnostic,
            "stopped_confirmed": False,
            "error": "StopMove was accepted but physical stop could not be confirmed",
            "reason": reason,
            "suggested_actions": ["get_state", "retry_stop"]})

    def _stop_move_worker(self, action_id):
        requested_at = time.monotonic()
        try:
            ret = self.proxy.StopMove()
        except Exception as exc:
            _acp_notify(action_id, "error", {
                "action": "stop_move", "ret": 3104,
                "error": "StopMove RPC failed",
                "reason": f"The stop command could not reach the sport controller: {type(exc).__name__}: {str(exc)[:160]}",
                "suggested_actions": ["get_state", "retry_stop"]})
            return
        if ret != 0:
            _acp_notify(action_id, "error", {
                "action": "stop_move", "ret": ret, "rpc_ret": ret,
                "error": "StopMove was rejected",
                "reason": "SportClient did not accept the stop command",
                "suggested_actions": ["get_state", "retry_stop"]})
            return
        self._await_stopped(action_id, requested_at)


class SpecialMotionPlugin:
    """As2W-specific discrete motions provided by the official SportClient."""
    PREFIX = "special_motion"

    def __init__(self, config, namespace, executor, proxy):
        self.proxy = proxy
        self._active_posture = None
        self._state_lock = threading.Lock()
        self._motion_lock = threading.Lock()
        self._motion_generation = 0

    def is_moving(self):
        with self._state_lock:
            return self._motion_lock.locked() or self._active_posture is not None

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
                    "x-completion": {"actions": actions, "timeout": 45},
                    "x-action-params": {
                        "front_flip": {"params": ["confirm"], "description": "DANGEROUS forward flip; requires confirm=true."},
                        "back_flip": {"params": ["confirm"], "description": "DANGEROUS backward flip; requires confirm=true."},
                        "handstand": {"params": ["enter", "confirm"], "description": "DANGEROUS handstand; requires confirm=true."},
                        "biped_stand": {"params": ["enter", "confirm"], "description": "DANGEROUS biped stand; requires confirm=true."}}}}

    def start(self): pass
    def stop(self):
        with self._state_lock:
            # Invalidate an enter operation before inspecting active posture.
            # If its vendor RPC is still blocked, the worker observes the new
            # generation after the RPC returns and sends the matching exit.
            self._motion_generation += 1
            posture = self._active_posture
        ret = 0
        if posture == "handstand":
            ret = self.proxy.HandStand(0)
        elif posture == "biped_stand":
            ret = self.proxy.BipedStand(0)
        if ret == 0:
            with self._state_lock:
                if self._active_posture == posture:
                    self._active_posture = None
        return ret

    def _run_motion(self, action_id, action, enter, generation):
        result = {"action": action}
        if action in ("handstand", "biped_stand"):
            result["enter"] = enter
        ret = None
        failure = None
        try:
            if action == "front_flip":
                ret = self.proxy.FrontFlip()
            elif action == "back_flip":
                ret = self.proxy.BackFlip()
            elif action == "handstand":
                ret = self.proxy.HandStand(1 if enter else 0)
            else:
                ret = self.proxy.BipedStand(1 if enter else 0)
            result["ret"] = ret
        except Exception as exc:
            failure = exc
            result["error"] = f"{type(exc).__name__}: {exc}"

        sustained = action in ("handstand", "biped_stand")
        cancelled = False
        if sustained and enter:
            with self._state_lock:
                cancelled = generation != self._motion_generation
                if ret == 0 and not cancelled:
                    self._active_posture = action if enter else None
        elif sustained and ret == 0:
            with self._state_lock:
                self._active_posture = None

        if cancelled:
            try:
                cancel_ret = (
                    self.proxy.HandStand(0) if action == "handstand"
                    else self.proxy.BipedStand(0)
                )
                result["cancel_ret"] = cancel_ret
                result["cancelled"] = True
                if cancel_ret == 0:
                    status = "cancelled"
                else:
                    status = "error"
                    result["error"] = (
                        "stop requested while entering posture, but exit returned {}"
                        .format(cancel_ret)
                    )
            except Exception as exc:
                status = "error"
                result["cancelled"] = True
                result["error"] = (
                    "stop requested while entering posture, but exit failed: "
                    f"{type(exc).__name__}: {exc}"
                )
        elif failure is not None:
            status = "error"
        else:
            status = "completed" if ret == 0 else "error"

        self._motion_lock.release()
        _acp_notify(action_id, status, result, tool="special_motion")

    def _start_motion(self, action, args):
        if not self._motion_lock.acquire(blocking=False):
            return {"error": "another special motion is still running"}
        action_id = f"as2w_special_motion_{uuid4().hex[:8]}"
        enter = bool(args.get("enter", True))
        with self._state_lock:
            self._motion_generation += 1
            generation = self._motion_generation
        try:
            threading.Thread(target=self._run_motion,
                             args=(action_id, action, enter, generation),
                             daemon=True).start()
        except Exception:
            self._motion_lock.release()
            raise
        return {"accepted": True, "status": "running", "action": action,
                "action_id": action_id}

    def _validate_motion_state(self, action, enter):
        # Exiting a sustained posture is a recovery operation and must remain
        # available even when the firmware reports the posture itself rather
        # than a normal standing state.
        if action in ("handstand", "biped_stand") and not enter:
            return None
        try:
            state_result = self.proxy.GetState()
        except Exception as exc:
            return {"ret": 3104, "accepted": False, "action": action,
                    "current_state": "UNKNOWN",
                    "error": "Unable to read robot locomotion state",
                    "reason": f"GetState failed: {type(exc).__name__}: {str(exc)[:160]}",
                    "suggested_actions": ["get_state", "stand_up"]}
        if not isinstance(state_result, tuple) or len(state_result) != 2:
            return {"ret": state_result if isinstance(state_result, int) else 3104,
                    "accepted": False, "action": action,
                    "current_state": "UNKNOWN",
                    "error": "Unable to read robot locomotion state",
                    "reason": "The current FSM state is unknown; refusing a dangerous motion",
                    "suggested_actions": ["get_state", "stand_up"]}
        code, state = state_result
        state_name = (str(state.get("fsm_name", "")).strip().upper()
                      if isinstance(state, dict) else "")
        if code != 0 or not state_name:
            return {"ret": code, "accepted": False, "action": action,
                    "current_state": state_name or "UNKNOWN",
                    "error": "Unable to read robot locomotion state",
                    "reason": "The current FSM state is unknown; refusing a dangerous motion",
                    "suggested_actions": ["get_state", "stand_up"]}
        allowed = {"STAND_UP", "BALANCE_STAND", "RECOVERY_STAND",
                   "AI_STAND_UP", "AI_BALANCE_STAND", "AI_RECOVERY_STAND"}
        if state_name not in allowed:
            return {"ret": -1, "accepted": False, "action": action,
                    "current_state": state_name,
                    "error": "Special motion cannot be executed from the current state",
                    "reason": "The robot must be standing and balanced before a special motion",
                    "suggested_actions": ["stand_up", "balance_stand", "recovery_stand"]}
        return None

    def dispatch(self, action, args):
        if action in ("start", "info"): return {"state": "ready"}
        if action == "stop":
            return {"state": "idle", "ret": self.stop()}
        motions = ("front_flip", "back_flip", "handstand", "biped_stand")
        if action in motions and args.get("confirm") is not True:
            return {
                "ok": False,
                "code": "INVALID_ARGUMENT",
                "error": "special motion requires boolean confirm=true",
            }
        if action in motions:
            state_error = self._validate_motion_state(
                action, bool(args.get("enter", True)))
            if state_error is not None:
                return state_error
            return self._start_motion(action, args)
        return None


# Import compatibility for deployments that imported the old Python class.
# Only the ``special_motion`` tool is advertised by the bundle.
SpecialActionPlugin = SpecialMotionPlugin


_AS2_JOINT_NAMES = [
    "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
    "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
    "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
    "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint",
]


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
                    "red": {"type": "integer", "minimum": 0, "maximum": 255},
                    "green": {"type": "integer", "minimum": 0, "maximum": 255},
                    "blue": {"type": "integer", "minimum": 0, "maximum": 255}},
                    "required": ["action"],
                    "x-action-params": {
                        "set_color": {"params": ["red", "green", "blue"], "description": "Set RGB color."},
                        "off": {"params": [], "description": "Turn the LED off."},
                        "info": {"params": [], "description": "Read the selected color."}}}}

    def start(self):
        if not any(self._color):
            return
        self._keepalive_stop.clear()
        if self._keepalive_thread is None or not self._keepalive_thread.is_alive():
            self._keepalive_thread = threading.Thread(
                target=self._keepalive, daemon=True, name="as2w-led")
            self._keepalive_thread.start()

    def stop(self):
        self._keepalive_stop.set()
        if self._keepalive_thread is not None:
            self._keepalive_thread.join(timeout=1)
            self._keepalive_thread = None

    def _keepalive(self):
        while not self._keepalive_stop.is_set():
            try:
                self._proxy.Audio_LedControl(*tuple(self._color))
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
