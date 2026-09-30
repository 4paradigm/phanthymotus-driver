"""Unitree As2W driver plugins (official AS2 SDK SportClient)."""
import json
import math
import queue
import socket
import struct
import threading
import multiprocessing
import time
from uuid import uuid4

from audio_msgs.msg import AudioChunk
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
from unitree_sdk2py.core.channel import ChannelSubscriber
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import BmsState_, LowState_
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


_STATE_PUBLISH_HZ = 60.0
_AS2_SPORT_MODE_NAMES = {
    # SportModeState.mode is numeric high-level mode, not SportClient's FSM.
    0: "IDLE_DEFAULT_STAND", 1: "BALANCE_STAND", 2: "POSE",
    3: "LOCOMOTION", 4: "RESERVE", 5: "LIE_DOWN", 6: "JOINT_LOCK",
    7: "DAMPING", 8: "RECOVERY_STAND", 9: "RESERVE_2", 10: "SIT",
    11: "FRONT_FLIP", 12: "FRONT_JUMP", 13: "FRONT_POUNCE",
}
# A2/AS2 BodyHeight is an absolute controller target.  Do not apply the
# Go2-relative [-0.18, 0.03] mapping here: the AS2 SDK example itself calls
# BodyHeight(0.18), and the URDF leg chain places the high body target near
# 0.48 m (two 0.212 m vertical leg links plus approximately 0.055 m wheel
# radius and base/foot offsets; the 0.1054 m hip offset is lateral).
_BODY_HEIGHT_MIN_M = 0.17
_BODY_HEIGHT_MAX_M = 0.50
_MIC_GROUP = "239.168.123.161"
_MIC_PORT = 5555
_MIC_CHUNK_BYTES = 1024


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
    endpoint = f"{os.environ.get('AGENT_CORE_URL', 'https://localhost:15678')}/api/acp/complete"
    payload = json.dumps({"action_id": action_id, "status": status,
                          "result": result, "tool": "loco", "ts": time.time()}).encode()
    try:
        request = urllib.request.Request(endpoint,
            data=payload, headers={"Content-Type": "application/json"}, method="POST")
        with urllib.request.urlopen(request, timeout=5, context=ssl._create_unverified_context()) as response:
            print(f"[loco] ACP complete action_id={action_id} status={status} http={response.status}", flush=True)
    except Exception as exc:
        print(f"[loco] ACP callback failed action_id={action_id} status={status} endpoint={endpoint} error={str(exc)[:180]}", flush=True)


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
        # The four wheel joints are kinematic joints, not low-state motors.
        # The skeleton renderer still needs them to traverse the URDF from a
        # calf link to its wheel; omitting them makes the lower leg disappear.
        for index, name in enumerate(("FR_foot_joint", "FL_foot_joint",
                                      "RR_foot_joint", "RL_foot_joint"),
                                     start=len(skeleton)):
            skeleton.append({"idx": index, "name": name, "q": 0.0,
                             "dq": 0.0, "tau": 0.0, "temperature": [],
                             "virtual": True})
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
        mode = int(getattr(msg, "mode", 0))
        body_height = _number(getattr(msg, "body_height", 0))
        mode_name = _AS2_SPORT_MODE_NAMES.get(mode, f"MODE_{mode}")
        body_height_valid = body_height != 0.0
        loco = {"mode": mode,
                "mode_name": mode_name,
                "mode_name_source": "SportModeState.mode",
                "body_height": body_height,
                "body_height_m": body_height if body_height_valid else None,
                "body_height_valid": body_height_valid,
                "body_height_status": "reported" if body_height_valid else "unavailable",
                "body_height_source": "rt/lf/sportmodestate.body_height"}
        loco.update(self._flat("velocity", getattr(msg, "velocity", [])))
        loco.update(self._flat("position", getattr(msg, "position", [])))
        loco["velocity_mps"] = list(getattr(msg, "velocity", []))
        loco["position_m"] = list(getattr(msg, "position", []))
        loco["yaw_speed_rad_s"] = _number(getattr(msg, "yaw_speed", 0))
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
        self._move_thread = None
        self._transition_stop = None
        self._transition_thread = None
        self._state_override = None
        self._state_override_until = 0.0
        # AS2 firmware may keep reporting AI_FREE_WALK after StopMove and a
        # successful BalanceStand.  This is a controller-label lag, not proof
        # that velocity is still active.  Keep the accepted terminal posture
        # as a state anchor until a new motion/posture command supersedes it.
        self._state_hint = None

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
        raw_name = str(state.get("fsm_name", "")).strip().upper()
        name = raw_name
        if (raw_name in {"AI_FREE_WALK", "FREE_WALK"} and
                (self._state_hint == "BALANCE_STAND" or
                 (self._state_override == "BALANCE_STAND" and
                  time.monotonic() < self._state_override_until))):
            state = dict(state)
            state["raw_fsm_name"] = raw_name
            state["fsm_name"] = "BALANCE_STAND"
            state["state_source"] = "stop_move_balance_stand_confirmation"
            name = "BALANCE_STAND"
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
            thread = self._transition_thread
            self._transition_stop = None
        if event is not None:
            event.set()
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=8.0)

    def _finish_transition(self, stop_event):
        with self._lock:
            if self._transition_stop is stop_event:
                self._transition_stop = None
            if self._transition_thread is threading.current_thread():
                self._transition_thread = None

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
        transition_thread = threading.Thread(
            target=self._transition_to_balance_and_move,
            args=(action_id, vx, vy, yaw, duration, transition_stop),
            daemon=True, name="as2w-loco-balance-move")
        with self._lock:
            self._transition_thread = transition_thread
        transition_thread.start()
        return action_id, {"ret": 0, "accepted": True, "status": "running",
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
                self._move_thread = threading.current_thread()
            move_ret = 0
            move_error = None
            while not stop_event.is_set():
                try:
                    move_ret = self.proxy.Move(vx, vy, yaw)
                except Exception as exc:
                    move_ret = 3104
                    move_error = f"{type(exc).__name__}: {str(exc)[:160]}"
                if move_ret != 0:
                    break
                stop_event.wait(0.1)
            stop_error = None
            try:
                self.proxy.StopMove()
            except Exception as exc:
                stop_error = f"{type(exc).__name__}: {str(exc)[:160]}"
            finally:
                with self._lock:
                    if self._stop is stop_event:
                        self._stop = None
                    if self._move_thread is threading.current_thread():
                        self._move_thread = None
                self._finish_transition(stop_event)
            if stop_error and not stop_event.is_set():
                _acp_notify(action_id, "error", {
                    "action": "move", "ret": 3104,
                    "error": "Continuous Move cleanup failed",
                    "reason": f"StopMove failed after the velocity loop ended: {stop_error}",
                    "suggested_actions": ["stop_move", "get_state"]})
            elif move_ret != 0 and not stop_event.is_set():
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
        cancelled = False
        move_failure = None
        while time.monotonic() < deadline:
            remaining = deadline - time.monotonic()
            if stop_event.wait(min(0.1, remaining)):
                cancelled = True
                break
            if time.monotonic() < deadline:
                try:
                    ret = self.proxy.Move(vx, vy, yaw)
                except Exception as exc:
                    move_failure = {"ret": 3104, "rpc_error": f"{type(exc).__name__}: {str(exc)[:160]}"}
                    break
                if ret != 0:
                    move_failure = {"ret": ret, "rpc_ret": ret}
                    break
        stop_error = None
        try:
            self.proxy.StopMove()
        except Exception as exc:
            stop_error = f"{type(exc).__name__}: {str(exc)[:160]}"
        finally:
            self._finish_transition(stop_event)
        if stop_error:
            _acp_notify(action_id, "error", {
                "action": "move", "ret": 3104,
                "duration": duration, "error": "Timed Move cleanup failed",
                "reason": f"StopMove failed after the timed command: {stop_error}",
                "suggested_actions": ["stop_move", "get_state"]})
        elif move_failure:
            _acp_notify(action_id, "error", {
                "action": "move", "duration": duration,
                "error": "Timed Move failed",
                "reason": "The sport controller stopped accepting the velocity command",
                "suggested_actions": ["get_state", "stop_move"], **move_failure})
        elif cancelled:
            _acp_notify(action_id, "cancelled", {
                "action": "move", "duration": duration,
                "reason": "Timed move stopped by stop_move or card shutdown"})
        else:
            self._await_stopped(
                action_id, "move",
                {"duration": duration,
                 "reason": "requested duration elapsed and walking FSM stabilized"})

    def _move_error_for_state(self, state):
        if state in self._DOWN:
            return self._not_allowed("move", state,
                "The robot is not standing, so the controller rejects Move",
                ["stand_up", "recovery_stand"])
        return self._not_allowed("move", state,
            "The robot is in a transition, special motion, or fault state",
            ["stop_move", "recovery_stand", "get_state"])
    def get_tool(self):
        actions = ["move", "stop_move", "stand_up", "stand_down", "balance_stand", "recovery_stand", "speed_level", "body_height", "body_position", "switch_joystick", "left_side_gait", "right_side_gait", "auto_recovery", "get_state"]
        return {"name": "loco", "type": "actuator", "multiInstance": False,
                "description": "As2W locomotion. move uses signed vx forward/back m/s, vy lateral m/s, vyaw rotation degrees/s (converted to radians for the SDK), and duration seconds (-1 means continue until stop_move). A positive duration stops internally when its timer expires; stop_move is only for an active duration=-1 move. stand_up/stand_down change posture; balance_stand enables active balance; recovery_stand is for fallen/down posture; body_height is an absolute AS2 target height in meters and is passed directly to the SDK; body_position is a direct controller offset. The flag actions are explicitly documented below.", "inputSchema": {"type": "object", "properties": {
                    "action": {"type": "string", "enum": actions, "description": "Locomotion action"}, "vx": {"type": "number", "description": "Forward velocity m/s [-1.5, 1.5]"}, "vy": {"type": "number", "description": "Lateral velocity m/s [-1, 1]"}, "vyaw": {"type": "number", "description": "Yaw velocity in degrees/s [-120, 120]; converted to radians/s for Unitree SDK"},
                    "duration": {"type": "number", "minimum": -1, "maximum": 30, "description": "Seconds; -1 continues until stop_move"}, "roll": {"type": "number", "description": "Body roll radians"}, "pitch": {"type": "number", "description": "Body pitch radians"}, "yaw": {"type": "number", "description": "Body yaw radians"},
                    "speed_preset": {"type": "string", "enum": ["slow", "normal", "fast"], "description": "Speed limiter preset"}, "height": {"type": "number", "minimum": 0.17, "maximum": 0.50, "description": "Absolute AS2 body height in meters"}, "x": {"type": "number", "description": "Body X offset"}, "y": {"type": "number", "description": "Body Y offset"}, "z": {"type": "number", "description": "Body Z offset"}, "flag": {"type": "boolean", "description": "Used by four switch actions: true enables/enters and false disables/exits."}}, "required": ["action"],
                "x-completion": {"actions": ["move", "stop_move", "stand_up", "stand_down", "balance_stand", "recovery_stand"], "timeout": 45},
                "x-action-params": {
                    "move": {"params": ["vx", "vy", "vyaw", "duration"], "description": "Move with optional duration (-1 for continuous)."},
                    "stop_move": {"params": [], "description": "Stop movement."},
                    "stand_up": {"params": [], "description": "Stand up."}, "stand_down": {"params": [], "description": "Stand down."},
                    "balance_stand": {"params": [], "description": "Balance stand."}, "recovery_stand": {"params": [], "description": "Recovery stand."},
                    "speed_level": {"params": ["speed_preset"], "description": "Set speed limiter: slow, normal, or fast."}, "body_height": {"params": ["height"], "description": "Set AS2 absolute body height target in meters; supported range is 0.17-0.50 m. Around 0.48 m includes the URDF high-stand leg geometry and wheel radius; the value is passed directly to the AS2 SDK."},
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
        with self._lock:
            event = self._stop
            thread = self._move_thread
            self._stop = None
        if event:
            event.set()
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=8.0)
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
            # AS2 prefixes sport FSM names with AI_ (for example
            # AI_STAND_UP/AI_STAND_DOWN).  The public action names use the
            # shorter names, so compare both forms rather than reporting a
            # false timeout after the robot has completed the transition.
            expected_names = {expected_name.upper(), "AI_" + expected_name.upper()}
            if left_old_state and name in expected_names:
                matches += 1
            else:
                matches = 0
            if matches >= 2:
                _acp_notify(action_id, "completed", {
                    "action": action, "ret": 0, "state": state,
                    "final_state": name,
                    "reason": "SportClient posture transition completed"})
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
            vx = max(-1.5, min(1.5, float(args.get("vx", 0))))
            vy = max(-1, min(1, float(args.get("vy", 0))))
            # Keep the public MCP unit in degrees/s and convert at the single
            # boundary where commands enter Unitree's radian-based SDK.
            vyaw_deg = max(-120.0, min(120.0, float(args.get("vyaw", 0))))
            yaw = math.radians(vyaw_deg)
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
                _, result = self._start_balance_move(state_name, vx, vy, yaw, duration)
                return result
            if state_name not in self._STANDING and not self._is_moving(state_name):
                return self._move_error_for_state(state_name)
            self._cancel_transition()
            self._state_override = None
            self._state_override_until = 0.0
            # A new velocity command makes the previous terminal posture hint
            # invalid.  The hint is deliberately cleared only after the
            # current state has been read, so the command can use it as the
            # gate for body_height/stand transitions immediately beforehand.
            self._state_hint = None
            if duration is None:
                self._stop_continuous()
                ret = self.proxy.Move(vx, vy, yaw)
                return {"ret": ret, "accepted": ret == 0, "action": "move",
                        "current_state": state_name, "vx": vx, "vy": vy,
                        "vyaw": vyaw_deg, "vyaw_sdk_rad": yaw,
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
                        _, result = self._start_balance_move(state_name, vx, vy, yaw, duration)
                        return result
                    return {"ret": ret, "rpc_ret": ret, "accepted": False,
                            "action": "move", "current_state": state_name,
                            "error": "Continuous Move was rejected",
                            "reason": "The sport controller refused the velocity command",
                            "suggested_actions": ["get_state", "stop_move"]}
                move_thread = threading.Thread(
                    target=self._run_continuous_move,
                    args=(action_id, vx, vy, yaw, stop_event),
                    daemon=True, name="as2w-loco-continuous-move")
                with self._lock:
                    self._move_thread = move_thread
                move_thread.start()
                return {"ret": 0, "accepted": True, "status": "running",
                        "action": "move", "action_id": action_id,
                        "current_state": state_name, "duration": -1}
            if duration < 0: return {"ret": -1, "message": "duration must be -1, 0, or positive"}
            self._stop_continuous()
            ret = self.proxy.Move(vx, vy, yaw)
            if ret != 0:
                if state_name in self._STANDING:
                    _, result = self._start_balance_move(state_name, vx, vy, yaw, duration)
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
            transition_thread = threading.Thread(
                target=self._run_timed_move,
                args=(action_id, vx, vy, yaw, duration, stop_event),
                daemon=True, name="as2w-loco-timed-move")
            with self._lock:
                self._transition_thread = transition_thread
            transition_thread.start()
            return {"ret": 0, "accepted": True, "status": "running",
                    "action": "move", "action_id": action_id,
                    "current_state": state_name, "duration": duration}
        if action == "stop_move":
            with self._lock:
                continuous_active = self._stop is not None or self._move_thread is not None
            if not continuous_active:
                return {"ret": -1, "accepted": False, "action": action,
                        "error": "No continuous move is active",
                        "reason": "stop_move is only the terminator for move with duration=-1",
                        "suggested_actions": ["move", "get_state"]}
            self._cancel_transition()
            self._stop_continuous()
            action_id = f"as2w_loco_{uuid4().hex[:8]}"
            # StopMove can block behind a firmware/RPC timeout.  It must not
            # hold the MCP request open; ACP owns the asynchronous result.
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
                action_id = f"as2w_loco_{uuid4().hex[:8]}"
                method, expected_name = methods[action]
                threading.Thread(target=self._stop_then_posture,
                                 args=(action_id, action, method, expected_name, state_name),
                                 daemon=True, name="as2w-loco-stop-then-stand-down").start()
                return {"ret": 0, "accepted": True, "status": "running",
                        "action": action, "action_id": action_id,
                        "current_state": state_name,
                        "transition": "stop_move_then_stand_down"}
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
                    ["stand_down", "recovery_stand"])
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
        if action == "speed_level":
            preset = args.get("speed_preset", "normal")
            if preset not in {"slow", "normal", "fast"}: return {"ret": -1, "error": "speed_preset must be slow, normal, or fast"}
            return {"ret": self.proxy.SpeedLevel({"slow": -1, "normal": 0, "fast": 1}[preset]), "speed_preset": preset}
        if action == "body_height":
            if "height" not in args:
                return {"ret": -1, "accepted": False, "action": action,
                        "error": "height is required",
                        "reason": "BodyHeight needs an absolute target in meters; zero is not used as an implicit default",
                        "suggested_actions": ["stand_up", "balance_stand"]}
            try:
                height = float(args["height"])
            except (TypeError, ValueError):
                return {"ret": -1, "accepted": False, "action": action,
                        "error": "height must be a number", "suggested_actions": ["get_state"]}
            if not math.isfinite(height) or not _BODY_HEIGHT_MIN_M <= height <= _BODY_HEIGHT_MAX_M:
                return {"ret": -1, "accepted": False, "action": action,
                        "error": f"height must be between {_BODY_HEIGHT_MIN_M:.2f} and {_BODY_HEIGHT_MAX_M:.2f} meters",
                        "suggested_actions": ["get_state"]}
            state_name, state, state_error = self._read_state()
            if state_error:
                return {**state_error, "action": action, "height_m": height}
            if state_name in self._DOWN or self._is_moving(state_name):
                return self._not_allowed(
                    action, state_name,
                    "BodyHeight requires a standing, non-walking posture",
                    ["stand_up", "balance_stand", "stop_move"])
            # AS2/A2's BodyHeight argument is the target height itself.  This
            # differs from the relative-offset convention used by some Go2
            # SDKs and is why 0.35 must reach the SDK as 0.35, not as zero.
            sdk_height = height
            ret = self.proxy.BodyHeight(sdk_height)
            result = {"ret": ret, "accepted": ret == 0, "action": action,
                      "height_m": height, "sdk_height_m": sdk_height,
                      "current_state": state_name}
            if ret != 0:
                result.update({"rpc_ret": ret,
                               "error": "SportClient rejected BodyHeight",
                               "reason": "The controller did not accept the target height",
                               "suggested_actions": ["balance_stand", "get_state"]})
            return result
        if action == "body_position": return {"ret": self.proxy.BodyPosition(float(args.get("x", 0)), float(args.get("y", 0)), float(args.get("z", 0)), float(args.get("yaw", 0)))}
        if action == "auto_recovery": return {"ret": self.proxy.SetAutoRecovery(1 if args.get("flag", True) else 0)}
        if action == "switch_joystick": return {"ret": self.proxy.SwitchJoystick(1 if args.get("flag", True) else 0)}
        if action == "left_side_gait": return {"ret": self.proxy.LeftSideGait(1 if args.get("flag", True) else 0)}
        if action == "right_side_gait": return {"ret": self.proxy.RightSideGait(1 if args.get("flag", True) else 0)}
        if action == "get_state":
            state_name, state, state_error = self._read_state()
            if state_error:
                return {**state_error, "action": action, "state": state or {}}
            return {"ret": 0, "state": state, "current_state": state_name,
                    "state_source": state.get("state_source", "sport_client.get_state")}
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
                if self._move_thread is threading.current_thread():
                    self._move_thread = None
            self._finish_transition(stop_event)
        _acp_notify(action_id, "cancelled", {
            "action": "move", "ret": 0, "duration": -1,
            "reason": "Continuous move stopped by stop_move or card shutdown"})

    def _await_stopped(self, action_id, completed_action="stop_move", completion_extra=None):
        """Leave the AS2 walking FSM before reporting an action terminal state."""
        deadline = time.monotonic() + 2.0
        last_state = "UNKNOWN"
        balance_requested = False
        balance_ret = None
        while time.monotonic() < deadline:
            name, state, error = self._read_state()
            if error:
                _acp_notify(action_id, "error", {
                    **error, "action": completed_action,
                    "reason": "StopMove was accepted, but state confirmation failed"})
                return
            last_state = name
            if not self._is_moving(name):
                result = {"action": completed_action, "ret": 0,
                          "state": state, "reason": "controller left walking state"}
                if completion_extra:
                    result.update(completion_extra)
                _acp_notify(action_id, "completed", result)
                return
            if not balance_requested:
                # On AS2, StopMove may stop the velocity command while the FSM
                # remains AI_FREE_WALK. BalanceStand is the documented safe
                # bridge back to a posture from which StandDown is accepted.
                balance_requested = True
                balance_ret = self.proxy.BalanceStand()
                print(f"[loco] stop stabilization action_id={action_id} state={name} BalanceStand ret={balance_ret}", flush=True)
                if balance_ret != 0:
                    break
                self._state_override = "BALANCE_STAND"
                self._state_override_until = time.monotonic() + 10.0
                self._state_hint = "BALANCE_STAND"
            time.sleep(0.1)
        # AS2W firmware can keep GetState().fsm_name at AI_FREE_WALK after
        # StopMove has already stopped the velocity command. BalanceStand is
        # the accepted controller-side normalization; do not report a false
        # terminal failure solely because that label is stale. The state is
        # surfaced explicitly so callers can decide whether to issue the next
        # posture command.
        if balance_ret == 0:
            result = {"action": completed_action, "ret": 0,
                      "current_state": last_state, "balance_ret": 0,
                      "state_stale": True,
                      "reason": "StopMove and BalanceStand were accepted; AS2 GetState still reports a stale walking label"}
            if completion_extra:
                result.update(completion_extra)
            _acp_notify(action_id, "completed", result)
            return
        _acp_notify(action_id, "error", {
            "action": completed_action, "ret": 0, "current_state": last_state,
            "balance_ret": balance_ret,
            "error": "StopMove was accepted but balance normalization failed",
            "reason": "The controller did not accept BalanceStand after StopMove",
            "suggested_actions": ["get_state", "retry_stop", "balance_stand"]})

    def _stop_move_worker(self, action_id):
        try:
            # The sport RPC worker is serialized.  Do not send StopMove while
            # an old Move loop can still enqueue another command; that race
            # was leaving AS2 in AI_FREE_WALK after an apparently successful
            # stop.
            self._cancel_transition()
            self._stop_continuous()
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
        self._await_stopped(action_id)

    def _stop_then_posture(self, action_id, action, method, expected_name, state_name):
        """Serialize a safe StopMove before a posture command from AI walk."""
        self._cancel_transition()
        self._stop_continuous()
        try:
            stop_ret = self.proxy.StopMove()
        except Exception as exc:
            _acp_notify(action_id, "error", {
                "action": action, "ret": 3104, "current_state": state_name,
                "error": "StopMove RPC failed before posture transition",
                "reason": f"Could not stop the active velocity command: {type(exc).__name__}: {str(exc)[:160]}",
                "suggested_actions": ["get_state", "retry_stop"]})
            return
        if stop_ret != 0:
            _acp_notify(action_id, "error", self._rpc_rejected(
                action, state_name, stop_ret,
                "StopMove was rejected, so the posture transition was not sent",
                ["get_state", "stop_move", action]))
            return
        # GetState may retain AI_FREE_WALK while velocity is already zero.  Do
        # not use that label as a second command gate: the official posture
        # RPC is the authority on whether StandDown is now acceptable.
        time.sleep(0.2)
        try:
            ret = getattr(self.proxy, method)()
        except Exception as exc:
            _acp_notify(action_id, "error", {
                "action": action, "ret": 3104, "current_state": state_name,
                "stop_ret": stop_ret, "error": "Posture RPC failed after StopMove",
                "reason": f"{method} could not reach the sport controller: {type(exc).__name__}: {str(exc)[:160]}",
                "suggested_actions": ["get_state", "retry_stop", action]})
            return
        if ret != 0:
            _acp_notify(action_id, "error", self._rpc_rejected(
                action, state_name, ret,
                f"{method} was rejected after StopMove; the controller still considers the posture unsafe",
                ["get_state", "retry_stop", "balance_stand", action]))
            return
        self._await_posture(action_id, action, expected_name)


class SpecialActionPlugin:
    """As2W-specific discrete motions provided by the official SportClient."""
    PREFIX = "special_motion"

    def __init__(self, config, namespace, executor, proxy):
        self.proxy = proxy

    def get_tool(self):
        actions = ["front_flip", "back_flip", "handstand", "biped_stand"]
        return {"name": "special_motion", "type": "actuator", "multiInstance": False,
                "description": "AS2 special motions. Use only with a clear safety area and the firmware-required posture: front_flip/back_flip are one-shot flips; handstand/biped_stand enter or exit a sustained posture. confirm=true is mandatory.",
                "inputSchema": {"type": "object", "properties": {
                    "action": {"type": "string", "enum": actions},
                    "enter": {"type": "boolean", "description": "Only for handstand/biped_stand: true enters, false exits; omit for flips."},
                    "confirm": {"type": "boolean", "description": "Safety acknowledgement; must be true."}},
                    "required": ["action"],
                    "x-is-dangerous": True,
                    "x-completion": {"actions": actions, "timeout": 45},
                    "x-action-params": {
                        "front_flip": {"params": ["confirm"], "description": "One-shot forward flip; normally requires BALANCE_STAND and confirm=true."},
                        "back_flip": {"params": ["confirm"], "description": "One-shot backward flip; normally requires BALANCE_STAND and confirm=true."},
                        "handstand": {"params": ["enter", "confirm"], "description": "Enter/exit handstand with enter=true/false and confirm=true."},
                        "biped_stand": {"params": ["enter", "confirm"], "description": "Enter/exit biped stand with enter=true/false and confirm=true."}}}}

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
            action_id = f"as2w_special_{uuid4().hex[:8]}"
            try:
                ret = methods[action]()
            except Exception as exc:
                ret = 3104
                error = f"{type(exc).__name__}: {str(exc)[:160]}"
            else:
                error = None
            result = {"ret": ret, "accepted": ret == 0, "action": action,
                      "current_state": state_name, "action_id": action_id}
            if ret == 0:
                # SportClient's special-motion calls are synchronous from the
                # SDK's perspective.  Still publish a terminal ACP event so
                # callers do not wait forever merely because this is a
                # one-shot action.
                _acp_notify(action_id, "completed", {"action": action,
                             "ret": 0, "current_state": state_name})
            else:
                result.update({"error": "SportClient rejected the special action",
                    "reason": "The controller refused the special motion from the current posture",
                    "suggested_actions": ["get_state", "recovery_stand"],
                    **({"rpc_error": error} if error else {})})
                _acp_notify(action_id, "error", result)
            return result
        return None


_AS2_JOINT_NAMES = [
    "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
    "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
    "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
    "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint",
]


MIC_AUDIO_FORMAT = "audio/pcm-16k"
_AUDIO_FORMAT_ALIASES = {MIC_AUDIO_FORMAT, "pcm_16k_16bit_mono"}
SPEAKER_APP_NAME = "as2w_speaker"
_AUDIO_EOF_MAGIC = b"\x01\x00\xff\xff\x01\x00\xff\xff"
SPEAKER_BLOCK_BYTES = 9600  # 300 ms at 16 kHz, 16-bit, mono; AS2 voice startup needs a full frame.
SPEAKER_QUEUE_BLOCKS = 8  # Keep the live stream below 800 ms of queued audio.
_SPEAKER_EOF = object()
_SPEAKER_MAX_LEAD_S = 0.0


def _pcm_bytes(values):
    """Convert ROS int8/uint8 sequences without changing PCM bit patterns."""
    if isinstance(values, (bytes, bytearray, memoryview)):
        return bytes(values)
    return bytes(int(value) & 0xff for value in values)


def _audio_chunk(payload):
    """Build the common ROS audio message from a DDS byte sequence."""
    message = AudioChunk()
    message.format = MIC_AUDIO_FORMAT
    message.data = list(bytes(payload))
    return message


def _resolved_mic_config(config, interface):
    """Bind robot audio multicast to the same interface as Unitree DDS."""
    resolved = dict(config or {})
    if not resolved.get("multicast_interface") and interface and interface != "(auto)":
        resolved["multicast_interface"] = interface
    return resolved


class _MicNode:
    """Republish AS2's robot-body audio multicast as AudioChunk messages."""

    def __init__(self, topic, config=None):
        from rclpy.node import Node
        self.node = Node("as2w_mic")
        self.topic = topic
        self.publisher = self.node.create_publisher(AudioChunk, topic, _LOW_LAT_QOS)
        self.state = "idle"
        self.packet_count = 0
        self.last_packet_ts = 0.0
        self.last_error = None
        self._config = config or {}
        self._socket = None
        self._capture_thread = None
        self._publish_lock = threading.Lock()
        self._publish_buffer = bytearray()
        self.backend = "robot_multicast"
        self.node.create_timer(10.0, self._report)

    def _report(self):
        interface = self._config.get("multicast_interface") or "(default-route)"
        if self.packet_count:
            self.node.get_logger().info(
                f"As2W mic packets={self.packet_count} interface={interface} "
                f"packet_age_s={time.monotonic() - self.last_packet_ts:.2f}")
        else:
            self.node.get_logger().warning(
                f"As2W mic has received no multicast packets on interface={interface} "
                f"group={self._config.get('multicast_group', _MIC_GROUP)}:{self._config.get('multicast_port', _MIC_PORT)}")

    def start(self):
        if self.state == "running":
            return self.topic
        with self._publish_lock:
            self._publish_buffer.clear()
        group = self._config.get("multicast_group", _MIC_GROUP)
        port = int(self._config.get("multicast_port", _MIC_PORT))
        sock = None
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            sock.bind(("", port))
            interface = self._config.get("multicast_interface", "")
            if interface:
                if interface == "(auto)":
                    interface = ""
                else:
                    index = socket.if_nametoindex(interface)
                    # Linux's ip_mreqn form uses the interface index, so the
                    # membership never follows the host default route.
                    membership = struct.pack(
                        "=4s4si", socket.inet_aton(group),
                        socket.inet_aton("0.0.0.0"), index)
            if not interface:
                membership = struct.pack("4s4s", socket.inet_aton(group),
                                         socket.inet_aton("0.0.0.0"))
            sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, membership)
        except (OSError, ValueError) as exc:
            if sock is not None:
                sock.close()
            self.state = "error"
            self.last_error = f"robot microphone multicast unavailable: {str(exc)[:160]}"
            return self.topic
        sock.settimeout(0.5)
        self._socket = sock
        self.state = "running"
        self._capture_thread = threading.Thread(target=self._capture_loop,
                                                 daemon=True, name="as2w-mic-udp")
        self._capture_thread.start()
        return self.topic

    def stop(self):
        sock = self._socket
        self._socket = None
        if sock is not None:
            try:
                sock.close()
            except Exception:
                pass
        if self._capture_thread is not None:
            self._capture_thread.join(timeout=1)
            self._capture_thread = None
        self.state = "idle"

    def _capture_loop(self):
        while self._socket is not None:
            try:
                payload, _ = self._socket.recvfrom(65535)
            except socket.timeout:
                continue
            except OSError:
                break
            if not payload:
                continue
            self._publish_pcm(payload)
            self.last_packet_ts = time.monotonic()
            self.backend = "robot_multicast"
            self.last_error = None

    def _publish_pcm(self, payload):
        with self._publish_lock:
            self._publish_buffer.extend(payload)
            while len(self._publish_buffer) >= _MIC_CHUNK_BYTES:
                chunk = bytes(self._publish_buffer[:_MIC_CHUNK_BYTES])
                del self._publish_buffer[:_MIC_CHUNK_BYTES]
                self.publisher.publish(_audio_chunk(chunk))
                self.packet_count += 1

class MicPlugin:
    PREFIX = "mic"

    def __init__(self, config, namespace, executor, interface=""):
        self._topic = f"/{namespace}/mic/audio"
        mic_config = _resolved_mic_config(config, interface)
        self._node = _MicNode(self._topic, mic_config)
        executor.add_node(self._node.node)

    def get_tool(self):
        return {"name": "mic", "type": "sensor", "multiInstance": False,
                "description": f"As2W robot-body microphone multicast as PCM 16kHz/16bit/mono: {self._topic}",
                "inputSchema": {"type": "object", "properties": {}},
                "topic_out": [{"topic": self._topic, "format": "audio/pcm-16k"}]}

    def start(self):
        self._node.start()

    def stop(self):
        self._node.stop()

    def dispatch(self, action, args):
        if action in ("start", "mic"):
            self._node.start()
            result = {"state": self._node.state, "topic": self._topic}
            if self._node.last_error:
                result["error"] = self._node.last_error
            return result
        if action == "stop":
            self.stop()
            return {"state": "idle"}
        if action == "info":
            age = None
            if self._node.last_packet_ts:
                age = max(0.0, time.monotonic() - self._node.last_packet_ts)
            return {"state": self._node.state,
                    "topic_out": [{"topic": self._topic, "format": "audio/pcm-16k"}],
                    "packets": self._node.packet_count,
                    "backend": self._node.backend,
                    "multicast": {"group": self._node._config.get("multicast_group", _MIC_GROUP),
                                   "port": int(self._node._config.get("multicast_port", _MIC_PORT)),
                                   "interface": self._node._config.get("multicast_interface", "auto")},
                    "packet_age_s": age,
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
        self.blocks_received = 0
        self.last_chunk_ts = 0.0
        self.last_play_ts = 0.0
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
        print(f"[speaker] subscribed topic={topic} format={MIC_AUDIO_FORMAT}; waiting for AudioChunk", flush=True)
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
        payload = _pcm_bytes(getattr(msg, "data", []))
        fmt = str(getattr(msg, "format", "") or "")
        if fmt and fmt not in _AUDIO_FORMAT_ALIASES:
            self._record_play_error("format", f"unsupported AudioChunk format {fmt[:80]}")
            return
        if payload:
            self.blocks_received = getattr(self, "blocks_received", 0) + 1
            self.last_chunk_ts = time.monotonic()
            # TTS publishes this short marker at utterance boundaries. It is
            # control data, never PCM, and must not be sent to the robot DAC.
            item = _SPEAKER_EOF if payload == _AUDIO_EOF_MAGIC else payload
            try:
                self._queue.put_nowait(item)
            except queue.Full:
                # The source is live audio; preserving old audio would make
                # latency grow without bound. Drop the oldest block instead.
                try:
                    self._queue.get_nowait()
                except queue.Empty:
                    pass
                try:
                    self._queue.put_nowait(item)
                except queue.Full:
                    return
            self.state = "playing"

    def _drain(self):
        import queue
        merged = bytearray()
        idle_polls = 0
        while not self._stop_event.is_set():
            try:
                item = self._queue.get(timeout=0.1)
            except queue.Empty:
                idle_polls += 1
                while len(merged) >= SPEAKER_BLOCK_BYTES:
                    self._play_block(bytes(merged[:SPEAKER_BLOCK_BYTES]))
                    del merged[:SPEAKER_BLOCK_BYTES]
                if merged and idle_polls >= 2:
                    self._play_block(bytes(merged))
                    merged.clear()
                    idle_polls = 0
                continue
            if item is None:
                break
            if item is _SPEAKER_EOF:
                while len(merged) >= SPEAKER_BLOCK_BYTES:
                    self._play_block(bytes(merged[:SPEAKER_BLOCK_BYTES]))
                    del merged[:SPEAKER_BLOCK_BYTES]
                if merged:
                    self._play_block(bytes(merged))
                    merged.clear()
                self._next_play_time = 0.0
                continue
            idle_polls = 0
            merged.extend(item)
            while len(merged) >= SPEAKER_BLOCK_BYTES:
                self._play_block(bytes(merged[:SPEAKER_BLOCK_BYTES]))
                del merged[:SPEAKER_BLOCK_BYTES]
        if not self._stop_event.is_set():
            while len(merged) >= SPEAKER_BLOCK_BYTES:
                self._play_block(bytes(merged[:SPEAKER_BLOCK_BYTES]))
                del merged[:SPEAKER_BLOCK_BYTES]
            if merged:
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
            self.last_play_ts = time.monotonic()
        except Exception as exc:
            self._record_play_error("exception", str(exc))
            return None
        # Pace at the audio timeline. Sending a whole merged block faster than
        # real time can overrun the AS2 voice buffer and distort the opening
        # frames or later syllables.
        duration = len(payload) / 32000.0
        self._next_play_time = max(getattr(self, "_next_play_time", 0.0), started) + duration
        wait_for = self._next_play_time - _SPEAKER_MAX_LEAD_S - time.monotonic()
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
        executor.add_node(self._node.node)

    def get_tool(self):
        input_topic = getattr(self, "_input_topic", None)
        topic_in = ([{"topic": input_topic, "format": "audio/pcm-16k"}]
                    if input_topic else [{"format": "audio/pcm-16k"}])
        return {"name": "speaker", "type": "actuator", "multiInstance": False,
                "description": "As2W speaker: subscribes to an explicitly connected PCM 16kHz/16bit/mono AudioChunk stream; it has no default input topic.",
                "inputSchema": {"type": "object", "properties": {
                    "action": {"type": "string", "enum": ["start", "stop", "info", "get_volume", "set_volume"]},
                    "input_topic": {"type": "string", "description": "ROS2 AudioChunk topic to subscribe for playback; required by start."},
                    "volume": {"type": "integer", "minimum": 0, "maximum": 100}},
                    "required": ["action"],
                    "x-action-params": {
                    "start": {"params": ["input_topic"], "description": "Start playback on the explicitly connected AudioChunk topic."},
                    "stop": {"params": [], "description": "Stop playback and clear buffered audio."},
                    "get_volume": {"params": [], "description": "Read the current volume."},
                    "set_volume": {"params": ["volume"], "description": "Set volume from 0 to 100."}}},
                "topic_in": topic_in}

    def start(self):
        # The canvas supplies the upstream topic when playback is started.
        # Do not subscribe to a hard-coded topic at bundle startup.
        return None

    def stop(self):
        self._node.stop()

    def dispatch(self, action, args):
        if action in ("start", "play", "speaker"):
            topic = args.get("input_topic") or args.get("topic_in") or getattr(self, "_input_topic", None)
            if isinstance(topic, dict):
                topic = topic.get("topic")
            elif isinstance(topic, (list, tuple)):
                topic = topic[0] if topic else None
            if not topic:
                return {"state": self._node.state, "accepted": False,
                        "error": "Missing input_topic",
                        "reason": "Speaker playback requires an explicitly connected AudioChunk topic",
                        "suggested_actions": ["start with input_topic"]}
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
            self.stop()
            return {"state": "idle"}
        if action == "info":
            return {"state": self._node.state, "topic": self._node.topic,
                    "blocks_received": self._node.blocks_received,
                    "blocks_sent": self._node.blocks_sent,
                    "chunk_age_s": (max(0.0, time.monotonic() - self._node.last_chunk_ts)
                                    if self._node.last_chunk_ts else None),
                    "last_play_age_s": (max(0.0, time.monotonic() - self._node.last_play_ts)
                                        if self._node.last_play_ts else None),
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
        self.fps = max(0.5, min(15.0, float(fps)))
        self.period = 1.0 / self.fps
        self._stop_event = threading.Event()
        self._thread = None
        self._publish_thread = None
        self._frame_queue = queue.Queue(maxsize=1)
        self.state = "idle"
        self.frames = 0
        self.last_frame_ts = 0.0
        self.last_frame_interval_s = None
        self.last_rpc_s = None
        self.last_error = None

    def start(self):
        if self._thread is not None and self._thread.is_alive():
            return
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._loop, daemon=True, name="as2w-camera-rgb")
        self._publish_thread = threading.Thread(
            target=self._publish_loop, daemon=True, name="as2w-camera-rgb-publish")
        self._thread.start()
        self._publish_thread.start()
        self.state = "running"

    def stop(self):
        self._stop_event.set()
        thread = self._thread
        if thread is not None:
            thread.join(timeout=6)
            if not thread.is_alive():
                self._thread = None
        publish_thread = self._publish_thread
        if publish_thread is not None:
            publish_thread.join(timeout=1)
            if not publish_thread.is_alive():
                self._publish_thread = None
        self.state = "idle"

    def _loop(self):
        while not self._stop_event.is_set():
            try:
                rpc_started = time.monotonic()
                result = self.proxy.Video_GetImageSample()
                self.last_rpc_s = time.monotonic() - rpc_started
            except Exception as exc:
                self.last_error = str(exc)
                self._stop_event.wait(self.period)
                continue
            code, payload = result if isinstance(result, tuple) and len(result) == 2 else (3104, None)
            if code == 0 and payload:
                try:
                    self._frame_queue.put_nowait(bytes(payload))
                except queue.Full:
                    try:
                        self._frame_queue.get_nowait()
                    except queue.Empty:
                        pass
                    try:
                        self._frame_queue.put_nowait(bytes(payload))
                    except queue.Full:
                        pass
                self.last_error = None
            elif code != 0:
                self.last_error = f"videohub returned {code}"
            self._stop_event.wait(self.period)

    def _publish_loop(self):
        while not self._stop_event.is_set():
            try:
                payload = self._frame_queue.get(timeout=0.5)
            except queue.Empty:
                continue
            message = CompressedImage()
            message.header.stamp = self.node.get_clock().now().to_msg()
            message.format = "jpeg"
            message.data = payload
            self.publisher.publish(message)
            self.frames += 1
            now = time.monotonic()
            if self.last_frame_ts:
                self.last_frame_interval_s = now - self.last_frame_ts
            self.last_frame_ts = now


class CameraPlugin:
    PREFIX = "camera_rgb"

    def __init__(self, config, namespace, executor, proxy, interface=""):
        self._topic = f"/{namespace}/camera/rgb"
        self._process_mode = bool(config.get("process", False))
        self._interface = interface
        self._proxy = proxy
        self._node = None if self._process_mode else _CameraRgbNode(self._topic, proxy, config.get("fps", 5))
        self._process = None
        self._process_stop = None
        self._fps = self._node.fps if self._node is not None else max(0.5, min(15.0, float(config.get("fps", 5))))
        if self._process_mode:
            self._fps = max(0.5, min(15.0, float(config.get("fps", 5))))
            self._start_process()
        else:
            executor.add_node(self._node.node)

    def _start_process(self):
        context = multiprocessing.get_context("spawn")
        stop_event = context.Event()
        self._process_stop = stop_event
        self._process = context.Process(
            target=_run_camera_process,
            args=(self._topic, self._fps, self._interface, stop_event),
            # The camera worker creates RpcProxy's own client workers. Python
            # forbids daemon processes from creating children; the shared
            # stop event lets it close those workers before process exit.
            daemon=False, name="as2w-camera-rgb-process")
        self._process.start()

    def get_tool(self):
        return {"name": "camera_rgb", "type": "sensor", "multiInstance": False,
                "description": f"AS2 videohub RGB JPEG stream in an isolated process at up to {getattr(self, '_fps', 5.0):g} FPS (firmware RPC latency may reduce the effective rate; implementation cap 15 FPS): {self._topic}",
                "inputSchema": {"type": "object", "properties": {}},
                "topic_out": [{"topic": self._topic, "format": "image/jpeg"}]}

    def start(self):
        if self._process_mode:
            if self._process is None or not self._process.is_alive():
                self._start_process()
        else:
            self._node.start()

    def stop(self):
        if self._process_mode:
            process = self._process
            self._process = None
            stop_event = self._process_stop
            self._process_stop = None
            if process is not None:
                if stop_event is not None:
                    stop_event.set()
                process.join(timeout=8)
                if process.is_alive():
                    print("[camera_rgb] graceful worker stop timed out; terminating process", flush=True)
                    process.terminate()
                    process.join(timeout=3)
        else:
            self._node.stop()

    def dispatch(self, action, args):
        if action in ("start", "camera_rgb"):
            self.start()
            return {"state": "running", "topic": self._topic}
        if action == "stop":
            self.stop()
            return {"state": "idle"}
        if action == "info":
            if self._process_mode:
                return {"state": "running" if self._process and self._process.is_alive() else "idle",
                        "process_pid": self._process.pid if self._process else None,
                        "topic_out": [{"topic": self._topic, "format": "image/jpeg"}]}
            return {"state": self._node.state, "frames": self._node.frames,
                    "last_frame_interval_s": self._node.last_frame_interval_s,
                    "last_rpc_s": self._node.last_rpc_s,
                    "last_error": self._node.last_error,
                    "topic_out": [{"topic": self._topic, "format": "image/jpeg"}]}
        return None


def _run_camera_process(topic, fps, interface, stop_event):
    from sensor_worker import run_camera
    run_camera(topic, fps, interface, stop_event)


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

    def start(self, allow_black=False):
        # Do not send a black LED command every 0.7 seconds during bundle
        # startup.  Apart from being unnecessary, that used to occupy the
        # shared voice RPC worker and made live speaker audio appear silent.
        if not allow_black and not any(self._color):
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
        if action == "start":
            self.start()
            return {"state": "ready", "color": list(self._color)}
        if action == "info":
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
        result = self._proxy.Audio_LedControl(*color)
        if result == 0:
            self.start(allow_black=action == "off")
        return {"ret": result, "color": list(color)}
