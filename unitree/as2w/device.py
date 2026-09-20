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
        depth=200,
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
        # AS2/As2W's official sport-state example uses the lf namespace.
        self._sport = ChannelSubscriber("rt/lf/sportmodestate", SportModeState_)
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

    @staticmethod
    def _flat(prefix, values):
        return {f"{prefix}_{i}": float(value) for i, value in enumerate(values)}

    def _on_low(self, msg):
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
        # Unitree BmsState_.current is milliamps (mA).
        current_ma = _number(getattr(bms, "current", 0))
        battery = {"soc": int(getattr(bms, "soc", 0)),
                   "current_ma": current_ma,
                   "cycle": int(getattr(bms, "cycle", 0))}
        battery.update(self._flat("temperature", getattr(bms, "temperature", [])))
        self._publish(self.battery, battery)

    def _on_sport(self, msg):
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
            code, state = self.proxy.GetState()
            name = str(state.get("fsm_name", "")) if code == 0 else ""
            if name and name != "ai_damping":
                left_old_state = True
            if left_old_state and name == expected_name:
                matches += 1
            else:
                matches = 0
            if matches >= 2:
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
        methods = {"stand_up": ("StandUp", "ai_stand_up"), "stand_down": ("StandDown", "ai_stand_down"), "balance_stand": ("BalanceStand", "ai_balance_stand"), "recovery_stand": ("RecoveryStand", "ai_recovery_stand")}
        if action in methods:
            method, expected_name = methods[action]
            ret = getattr(self.proxy, method)()
            if ret != 0: return {"ret": ret, "accepted": False, "action": action}
            action_id = f"as2w_loco_{uuid4().hex[:8]}"
            threading.Thread(target=self._await_posture, args=(action_id, action, expected_name), daemon=True).start()
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
    """As2W-specific discrete motions provided by the official SportClient."""
    PREFIX = "special_action"

    def __init__(self, config, namespace, executor, proxy):
        self.proxy = proxy

    def get_tool(self):
        actions = ["front_flip", "back_flip", "handstand", "biped_stand"]
        return {"name": "special_action", "type": "actuator", "multiInstance": False,
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
        if action == "front_flip": return {"ret": self.proxy.FrontFlip()}
        if action == "back_flip": return {"ret": self.proxy.BackFlip()}
        if action == "handstand": return {"ret": self.proxy.HandStand(1 if args.get("enter", True) else 0)}
        if action == "biped_stand": return {"ret": self.proxy.BipedStand(1 if args.get("enter", True) else 0)}
        return None


_AS2_JOINT_NAMES = [
    "FR_hip", "FR_thigh", "FR_calf",
    "FL_hip", "FL_thigh", "FL_calf",
    "RR_hip", "RR_thigh", "RR_calf",
    "RL_hip", "RL_thigh", "RL_calf",
]


MIC_AUDIO_FORMAT = "pcm_16k_16bit_mono"
SPEAKER_APP_NAME = "as2w_speaker"
SPEAKER_BLOCK_BYTES = 9600  # 300 ms at 16 kHz, 16-bit, mono.


def _audio_chunk(payload):
    """Build the common ROS audio message from a DDS byte sequence."""
    message = AudioChunk()
    message.format = MIC_AUDIO_FORMAT
    message.data = list(bytes(payload))
    return message


class _MicNode:
    """Republish AS2's robot microphone DDS stream as AudioChunk messages."""

    def __init__(self, topic):
        from rclpy.node import Node
        self.node = Node("as2w_mic")
        self.topic = topic
        self.publisher = self.node.create_publisher(AudioChunk, topic, _LOW_LAT_QOS)
        self.subscriber = None
        self.state = "idle"
        self.packet_count = 0
        self.last_packet_ts = 0.0

    def start(self):
        if self.subscriber is not None:
            return self.topic
        self.subscriber = ChannelSubscriber("rt/audiosender", AudioData_)
        self.subscriber.Init(self._on_audio, 10)
        self.state = "running"
        return self.topic

    def stop(self):
        if self.subscriber is not None:
            try:
                self.subscriber.Close()
            except Exception:
                pass
            self.subscriber = None
        self.state = "idle"

    def _on_audio(self, msg):
        payload = bytes(getattr(msg, "data", []))
        if not payload:
            return
        self.publisher.publish(_audio_chunk(payload))
        self.packet_count += 1
        self.last_packet_ts = time.monotonic()


class MicPlugin:
    PREFIX = "mic"

    def __init__(self, config, namespace, executor):
        self._topic = f"/{namespace}/mic/audio"
        self._node = _MicNode(self._topic)
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
        if action == "start":
            self._node.start()
            return {"state": "running", "topic": self._topic}
        if action == "stop":
            self._node.stop()
            return {"state": "idle"}
        if action == "info":
            return {"state": self._node.state,
                    "topic_out": [{"topic": self._topic, "format": "audio/pcm-16k"}],
                    "packets": self._node.packet_count}
        return None


class _SpeakerNode:
    """Subscribe to AudioChunk and stream bounded PCM blocks through AudioClient."""

    def __init__(self, audio_client):
        from rclpy.node import Node
        import queue
        self.node = Node("as2w_speaker")
        self._client = audio_client
        self._queue = queue.Queue()
        self._subscription = None
        self._stop_event = threading.Event()
        self._thread = None
        self.topic = None
        self.state = "idle"
        self.blocks_sent = 0
        self._next_play_time = 0.0

    def start(self, topic):
        if self._subscription is not None:
            if self.topic == topic:
                return topic
            self.stop()
        self.topic = topic
        self._subscription = self.node.create_subscription(
            AudioChunk, topic, self._on_chunk, _LOW_LAT_QOS)
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._drain, daemon=True, name="as2w-speaker")
        self._thread.start()
        self.state = "ready"
        return topic

    def stop(self):
        if self._subscription is not None:
            try:
                self.node.destroy_subscription(self._subscription)
            except Exception:
                pass
            self._subscription = None
        self._stop_event.set()
        self._queue.put(None)
        if self._thread is not None:
            self._thread.join(timeout=2)
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
        payload = bytes(getattr(msg, "data", []))
        if payload:
            self._queue.put(payload)
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
            self.blocks_sent += 1
        except Exception as exc:
            print(f"[speaker] PlayStream failed: {exc}", flush=True)
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


class SpeakerPlugin:
    PREFIX = "speaker"

    def __init__(self, config, namespace, executor, audio_client):
        self._node = _SpeakerNode(audio_client)
        executor.add_node(self._node.node)

    def get_tool(self):
        return {"name": "speaker", "type": "actuator", "multiInstance": False,
                "description": "As2W speaker: subscribes to PCM 16kHz/16bit/mono AudioChunk stream",
                "inputSchema": {"type": "object", "properties": {
                    "action": {"type": "string", "enum": ["start", "stop", "info", "get_volume", "set_volume"]},
                    "input_topic": {"type": "string", "description": "ROS2 AudioChunk topic"},
                    "volume": {"type": "integer", "minimum": 0, "maximum": 100}},
                    "required": ["action"]},
                "topic_in": [{"format": "audio/pcm-16k"}]}

    def start(self):
        pass

    def stop(self):
        self._node.stop()

    def dispatch(self, action, args):
        if action in ("start", "play"):
            topic = args.get("input_topic") or args.get("topic_in")
            if not topic:
                return {"error": "Missing input_topic"}
            return {"state": "ready", "topic": self._node.start(topic)}
        if action == "stop":
            self._node.stop()
            return {"state": "idle"}
        if action == "info":
            return {"state": self._node.state, "topic": self._node.topic,
                    "blocks_sent": self._node.blocks_sent}
        if action == "get_volume":
            code, volume = self._node._client.Audio_GetVolume()
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
        if self._thread is not None:
            self._thread.join(timeout=3)
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
                "inputSchema": {"type": "object", "properties": {
                    "action": {"type": "string", "enum": ["snapshot", "info"]}}},
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
        if action == "snapshot":
            return {"state": self._node.state, "frames": self._node.frames,
                    "topic_out": [{"topic": self._topic, "format": "image/jpeg"}]}
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

    def get_tool(self):
        return {"name": "led", "type": "actuator", "multiInstance": False,
                "description": "AS2 RGB LED control through the voice service",
                "inputSchema": {"type": "object", "properties": {
                    "action": {"type": "string", "enum": ["set_color", "off", "info"]},
                    "red": {"type": "integer", "minimum": 0, "maximum": 255},
                    "green": {"type": "integer", "minimum": 0, "maximum": 255},
                    "blue": {"type": "integer", "minimum": 0, "maximum": 255}},
                    "required": ["action"]}}

    def start(self):
        pass

    def stop(self):
        pass

    def dispatch(self, action, args):
        if action in ("start", "info"):
            return {"state": "ready", "color": list(self._color)}
        if action == "stop":
            return {"state": "idle"}
        if action == "off":
            color = (0, 0, 0)
        elif action == "set_color":
            color = tuple(max(0, min(255, int(args.get(key, 0))))
                          for key in ("red", "green", "blue"))
        else:
            return None
        result = self._proxy.Audio_LedControl(*color)
        self._color = list(color)
        return {"ret": result, "color": list(color)}
