"""Unitree As2W driver plugins (official AS2 SDK SportClient)."""
import json
import math
import struct
import threading
import time
from uuid import uuid4

from std_msgs.msg import String
from unitree_sdk2py.core.channel import ChannelSubscriber
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import BmsState_, LowState_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_


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
_REMOTE_PUBLISH_HZ = 10.0
_REMOTE_FRESH_S = 0.5
_REMOTE_DEADZONE = 0.1
_REMOTE_FRAME_HEADER = b"\x55\x51"
_BODY_HEIGHT_MIN_M = 0.17
_BODY_HEIGHT_MAX_M = 0.50


def _unavailable_remote():
    return {"available": False, "fresh": False,
            "control_level": "LOWLEVEL"}


def _parse_wireless_remote(raw):
    """Decode Unitree's little-endian 24-byte controller prefix."""
    try:
        data = bytes(raw)
    except (TypeError, ValueError):
        return _unavailable_remote()
    if len(data) < 24:
        return _unavailable_remote()
    # A connected AS2W controller produces Unitree frames beginning 55 51,
    # including the neutral frame whose buttons and axes are all zero. Some
    # firmware paths expose a correctly sized but entirely zero-filled array
    # when no controller payload is available; length alone must not report
    # that placeholder as an online remote.
    if data[:2] != _REMOTE_FRAME_HEADER:
        return _unavailable_remote()
    try:
        buttons = {
            "LT": bool(data[2] & (1 << 5)),
            "RT": bool(data[2] & (1 << 4)),
            "start": bool(data[2] & (1 << 2)),
            "LB": bool(data[2] & (1 << 1)),
            "RB": bool(data[2] & (1 << 0)),
            "left": bool(data[3] & (1 << 7)),
            "down": bool(data[3] & (1 << 6)),
            "right": bool(data[3] & (1 << 5)),
            "up": bool(data[3] & (1 << 4)),
            "Y": bool(data[3] & (1 << 3)),
            "X": bool(data[3] & (1 << 2)),
            "B": bool(data[3] & (1 << 1)),
            "A": bool(data[3] & (1 << 0)),
        }
        axes = {
            "lx": round(struct.unpack("<f", data[4:8])[0], 4),
            "rx": round(struct.unpack("<f", data[8:12])[0], 4),
            "ry": round(struct.unpack("<f", data[12:16])[0], 4),
            "ly": round(struct.unpack("<f", data[20:24])[0], 4),
        }
    except (IndexError, struct.error):
        return _unavailable_remote()
    if not all(math.isfinite(value) for value in axes.values()):
        return _unavailable_remote()
    # float32(0.1) is slightly greater than Python's binary64 0.1. Keep the
    # configured boundary itself inside the deadzone despite that conversion.
    active = any(buttons.values()) or any(
        abs(value) > _REMOTE_DEADZONE + 1e-6 for value in axes.values())
    return {
        "available": True,
        "fresh": True,
        "control_level": "LOWLEVEL",
        "buttons": buttons,
        "axes": axes,
        "active": active,
        "timestamp_ms": int(time.time() * 1000),
    }


def _number(value, default=0.0):
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def _finite(value):
    """Return one finite measured value, otherwise ``None``."""
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    return result if math.isfinite(result) else None


def _timespec_ms(stamp):
    """Convert Unitree's TimeSpec to epoch milliseconds when usable."""
    try:
        sec = int(stamp.sec)
        nanosec = int(stamp.nanosec)
    except (AttributeError, TypeError, ValueError):
        return None
    if sec <= 0 and nanosec <= 0:
        return None
    return sec * 1000 + nanosec // 1_000_000


def _quaternion_yaw(quaternion):
    """Return yaw from Unitree's ``[w, x, y, z]`` quaternion."""
    try:
        w, x, y, z = (float(value) for value in quaternion)
    except (TypeError, ValueError):
        return None
    if not all(math.isfinite(value) for value in (w, x, y, z)):
        return None
    norm = math.sqrt(w * w + x * x + y * y + z * z)
    if norm <= 1e-9:
        return None
    w, x, y, z = w / norm, x / norm, y / norm, z / norm
    return math.atan2(2.0 * (w * z + x * y),
                      1.0 - 2.0 * (y * y + z * z))


def _invalid_argument(message, field=None):
    """Return the card-level rejection expected by direct MCP callers."""
    result = {
        "ok": False,
        "ret": -1,
        "accepted": False,
        "code": "INVALID_ARGUMENT",
        "error": message,
    }
    if field:
        result["field"] = field
    return result


def _finite_argument(args, name, default=0.0):
    """Read one JSON number without accepting booleans, strings, NaN, or inf."""
    value = args.get(name, default)
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None, _invalid_argument(
            f"{name} must be a finite number", name)
    number = float(value)
    if not math.isfinite(number):
        return None, _invalid_argument(
            f"{name} must be a finite number", name)
    return number, None


def _boolean_argument(args, name, default=True):
    """Read one literal JSON boolean instead of relying on Python truthiness."""
    value = args.get(name, default)
    if value is not True and value is not False:
        return None, _invalid_argument(f"{name} must be a boolean", name)
    return value, None


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
    _ODOM_INTERVAL = 0.1

    def __init__(self, namespace, executor):
        from rclpy.node import Node
        self.node = Node("as2w_state")
        self.imu = self.node.create_publisher(String, f"/{namespace}/state/imu", _LOW_LAT_QOS or 1)
        self.joints = self.node.create_publisher(String, f"/{namespace}/state/joints", _LOW_LAT_QOS or 1)
        self.joint_state = self.node.create_publisher(String, f"/{namespace}/state/joint_state", _LOW_LAT_QOS or 1)
        self.battery = self.node.create_publisher(String, f"/{namespace}/state/battery", _LOW_LAT_QOS or 1)
        self.loco = self.node.create_publisher(String, f"/{namespace}/loco/state", _LOW_LAT_QOS or 1)
        self.odom = self.node.create_publisher(String, f"/{namespace}/state/odom", _LOW_LAT_QOS or 1)
        self._odom_lock = threading.Lock()
        self._odom_burst = []
        self._odom_burst_size = 0
        self._odom_stamp_ms = 0
        self._odom_stamp_provenance = {}
        self._last_odom_time = 0.0
        self.remote = self.node.create_publisher(
            String, f"/{namespace}/state/remote_controller", _LOW_LAT_QOS or 1)
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
        self._remote_lock = threading.Lock()
        self._remote = _unavailable_remote()
        self._remote_received_mono = None
        self._remote_enabled = True
        self._remote_last_publish_mono = 0.0
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

    @staticmethod
    def _xyz(prefix, values, invert_xy=False):
        """Name a three-vector and optionally normalise the planar signs."""
        values = list(values or [])
        result = {}
        for index, axis in enumerate(("x", "y", "z")):
            value = (_number(values[index], None)
                     if index < len(values) else None)
            if invert_xy and index < 2 and value is not None:
                value = -value
                # Do not expose negative zero after changing coordinate signs.
                if value == 0.0:
                    value = 0.0
            result[f"{prefix}_{axis}"] = value
        return result

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
            self._maybe_publish_odom()
            self._maybe_publish_remote()
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
        if hasattr(self, "remote"):
            self._cache_remote(getattr(msg, "wireless_remote", None))
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

    def _cache_remote(self, raw):
        parsed = _parse_wireless_remote(raw)
        with self._remote_lock:
            self._remote = parsed
            self._remote_received_mono = (
                time.monotonic() if parsed.get("available") else None)

    def remote_snapshot(self):
        with self._remote_lock:
            snapshot = dict(self._remote)
            received = self._remote_received_mono
        snapshot["fresh"] = bool(
            snapshot.get("available") and received is not None
            and time.monotonic() - received <= _REMOTE_FRESH_S)
        return snapshot

    def set_remote_enabled(self, enabled):
        with self._remote_lock:
            self._remote_enabled = bool(enabled)

    def remote_enabled(self):
        with self._remote_lock:
            return self._remote_enabled

    def _maybe_publish_remote(self):
        now = time.monotonic()
        with self._remote_lock:
            if (not self._remote_enabled
                    or now - self._remote_last_publish_mono
                    < 1.0 / _REMOTE_PUBLISH_HZ):
                return
            self._remote_last_publish_mono = now
        self._publish(self.remote, self.remote_snapshot())

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
        # Accumulate every vendor reading before the latest-value cache drops
        # older samples. Odom failures are isolated from the legacy state path.
        try:
            self._accumulate_odom(msg)
        except Exception:
            pass
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
        velocity = list(getattr(msg, "velocity", []))
        position = list(getattr(msg, "position", []))
        # These vendor vectors are in the odometry world frame. Keep their
        # measured signs here; state/odom below performs the body conversion.
        velocity_xyz = self._xyz("velocity", velocity)
        position_xyz = self._xyz("position", position)
        # AS2-W firmware leaves SportModeState.body_height at zero. Hardware
        # observation shows the actual body height in position.z, spanning
        # approximately [0, 0.4] m, so publish that single source consistently.
        body_height = position_xyz["position_z"]
        loco = {
            "mode": int(getattr(msg, "mode", 0)),
            "body_height": body_height,
        }
        loco.update(velocity_xyz)
        loco.update(position_xyz)
        self._publish(self.loco, loco)

    def _maybe_publish_odom(self):
        """Drain the measurement window on fixed 10 Hz due times."""
        if getattr(self, "odom", None) is None:
            return
        try:
            now = time.monotonic()
            due = self._last_odom_time + self._ODOM_INTERVAL
            if now < due:
                return
            self._last_odom_time = (
                due if now - due < self._ODOM_INTERVAL else now)
            with self._latest_lock:
                sport = self._latest_sport
            if sport is not None:
                self._publish_odom(sport)
        except Exception:
            # The original loco/state output must remain available even if the
            # standard odometry adapter sees a malformed vendor message.
            pass

    def _accumulate_odom(self, msg):
        """Convert one world-frame vendor reading into body-frame twist."""
        if getattr(self, "odom", None) is None:
            return
        from common.odom import resolve_stamp_ms

        velocity = list(getattr(msg, "velocity", []) or [])
        vx_world = _finite(velocity[0]) if len(velocity) > 0 else None
        vy_world = _finite(velocity[1]) if len(velocity) > 1 else None
        imu = getattr(msg, "imu_state", None)
        yaw = _quaternion_yaw(getattr(imu, "quaternion", None))
        if yaw is None or vx_world is None or vy_world is None:
            vx_body = vy_body = None
        else:
            cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
            vx_body = _finite(cos_yaw * vx_world + sin_yaw * vy_world)
            vy_body = _finite(-sin_yaw * vx_world + cos_yaw * vy_world)
        gyroscope = list(getattr(imu, "gyroscope", []) or [])
        row = [
            vx_body, vy_body,
            None, None, None,
            _finite(gyroscope[2]) if len(gyroscope) > 2 else None,
        ]
        stamp_ms, provenance = resolve_stamp_ms(
            vendor_ms=_timespec_ms(getattr(msg, "stamp", None)),
            received_ms=int(time.time() * 1000),
        )
        with self._odom_lock:
            self._odom_burst.append(row)
            self._odom_stamp_ms = stamp_ms
            self._odom_stamp_provenance = provenance

    def _publish_odom(self, msg):
        """Publish the finite per-axis mean without inventing measurements."""
        from common.odom import build_sample, mean_twist

        with self._odom_lock:
            burst, self._odom_burst = self._odom_burst, []
            self._odom_burst_size = len(burst)
            stamp_ms = self._odom_stamp_ms
            provenance = self._odom_stamp_provenance
        if not burst:
            stamp_ms = int(time.time() * 1000)
            provenance = {
                "stamp_source": "published",
                "stamp_reason": "no readable vendor reading in window",
            }
        twist = [_finite(value) for value in mean_twist(burst)]
        position = list(getattr(msg, "position", []) or [])
        body_height = (_finite(position[2]) if len(position) > 2 else None)
        sample = build_sample(
            stamp_ms=stamp_ms,
            twist=twist,
            vendor={
                "mode": int(getattr(msg, "mode", 0)),
                "gait_type": int(getattr(msg, "gait_type", 0)),
                "body_height": body_height,
                "samples": len(burst),
                **provenance,
            },
        )
        self._publish(self.odom, sample)

    def odom_health(self):
        with self._odom_lock:
            return {
                "odom_burst_samples": self._odom_burst_size,
                "odom_measured": (
                    "SportModeState velocity is odometry-world frame and is "
                    "rotated into body frame with each reading's quaternion; "
                    "wz comes from imu_state.gyroscope[2]."),
                **self._odom_stamp_provenance,
            }


class StatePlugin:
    PREFIX = "state"
    ODOM_AXES = ("vx", "vy", "wz")
    def __init__(self, config, namespace, executor):
        self._namespace = namespace
        self._executor = executor
        self._state = _StateNode(namespace, executor)
    def get_tools(self):
        specs = (("imu", "state/imu", "data/json", "As2W IMU state"),
                 ("joints", "state/joints", "sensor/skeleton", "As2W 12-joint leg skeleton for model animation"),
                 ("joint_state", "state/joint_state", "data/json", "As2W raw motor position, velocity, torque, and temperature"),
                 ("battery", "state/battery", "data/json", "As2W BMS state; current_ma is mA"),
                 ("loco_state", "loco/state", "data/json", "As2W high-level locomotion state"),
                 ("remote_controller", "state/remote_controller", "data/json",
                  "Read-only Unitree remote buttons and joystick state"))
        tools = [{"name": name, "type": "sensor", "multiInstance": False,
                  "description": desc,
                  "inputSchema": {"type": "object", "properties": {}},
                  "topic_out": [{"topic": f"/{self._namespace}/{path}",
                                 "format": fmt}]}
                 for name, path, fmt, desc in specs]
        loco_state = next(tool for tool in tools if tool["name"] == "loco_state")
        loco_state["description"] = (
            "As2W high-level locomotion state. velocity_x/y and position_x/y "
            "retain the vendor odometry-world axes; +Z is up. position_z and "
            "body_height are 0 m when lying down and approximately 0.4 m when "
            "standing. The state/odom output converts velocity to body frame, "
            "where +X is forward and +Y is left, for navigation.")
        loco_state["topic_out"] = self._loco_state_topics()
        remote = next(tool for tool in tools
                      if tool["name"] == "remote_controller")
        remote["inputSchema"] = {
            "type": "object",
            "properties": {
                "action": {"type": "string", "enum": [
                    "start", "stop", "info", "read"]},
            },
            "required": ["action"],
            "additionalProperties": False,
        }
        return tools

    def _loco_state_topics(self):
        return [
            {"topic": f"/{self._namespace}/loco/state", "format": "data/json"},
            {"topic": f"/{self._namespace}/state/odom", "format": "state/odom"},
        ]

    @staticmethod
    def _odom_interface():
        from common.odom import build_interface

        return build_interface(
            provides=StatePlugin.ODOM_AXES,
            rate_hz=10,
            # Wheel/leg dead reckoning is useful as measured velocity but not as
            # an absolute pose source.
            pose_drift="unbounded",
        )

    def _loco_state_info(self):
        result = {
            "state": "running",
            "topic_out": self._loco_state_topics(),
            "odom_interface": self._odom_interface(),
        }
        state = getattr(self, "_state", None)
        if state is not None:
            try:
                result.update(state.odom_health())
            except Exception:
                pass
        return result
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
        name = args.get("_tool_name")
        if name == "remote_controller":
            if action == "start":
                if self._state is None:
                    self.start()
                self._state.set_remote_enabled(True)
                return {"state": "running"}
            if action == "stop":
                # The LowState subscriber is shared with IMU/joints/battery.
                # Only disable this card's ROS publication.
                if self._state is not None:
                    self._state.set_remote_enabled(False)
                return {"state": "idle"}
            if action == "read":
                return (self._state.remote_snapshot() if self._state is not None
                        else _unavailable_remote())
            if action == "info":
                running = (self._state is not None
                           and self._state.remote_enabled())
                return {
                    "state": "running" if running else "idle",
                    "topic_out": [{
                        "topic": f"/{self._namespace}/state/remote_controller",
                        "format": "data/json",
                    }],
                }
        if action == "start":
            self.start()
            if name == "loco_state":
                return self._loco_state_info()
            return {"state": "running"}
        if action == "stop":
            self.stop()
            return {"state": "idle"}
        if action == "info":
            paths = {"imu": ("state/imu", "data/json"),
                     "joints": ("state/joints", "sensor/skeleton"),
                     "joint_state": ("state/joint_state", "data/json"),
                     "battery": ("state/battery", "data/json"),
                     "loco_state": ("loco/state", "data/json"),
                     "remote_controller": ("state/remote_controller", "data/json")}
            if name in paths:
                path, fmt = paths[name]
                if name == "loco_state":
                    return self._loco_state_info()
                return {"state": "running", "topic_out": [{"topic": f"/{self._namespace}/{path}", "format": fmt}]}
            return {"state": "running"}
        if action in ("imu", "joints", "joint_state", "battery", "loco_state", "remote_controller"):
            if action == "loco_state":
                return self._loco_state_info()
            path = {"imu": "state/imu", "joints": "state/joints", "joint_state": "state/joint_state", "battery": "state/battery", "loco_state": "loco/state", "remote_controller": "state/remote_controller"}[action]
            fmt = "sensor/skeleton" if action == "joints" else "data/json"
            return {"state": "running", "topic_out": [{"topic": f"/{self._namespace}/{path}", "format": fmt}]}
        return {"state": "running"} if action == "info" else None


class LocoPlugin:
    PREFIX = "loco"
    def __init__(self, config, namespace, executor, proxy):
        self.proxy = proxy
        self._lock = threading.Lock()
        self.motion_lock = threading.RLock()
        self._stop = None
        self._transition_stop = None
        self._servo = None
        self._special_motion = None
        self._controlled_spatial = None
        self._moving = False
        self._moving_owner = None
        self._state_override = None
        self._state_override_until = 0.0
        # Some AS2 firmware keeps reporting AI_FREE_WALK after both StopMove
        # and BalanceStand have succeeded. Preserve that accepted posture as a
        # posture anchor until a later motion or posture command supersedes it.
        self._state_hint = None

    def attach_servo(self, servo):
        """Attach the continuous command card that shares this chassis."""
        self._servo = servo

    def attach_special_motion(self, plugin):
        self._special_motion = plugin

    def attach_controlled_spatial(self, plugin):
        self._controlled_spatial = plugin

    def _other_chassis_owner(self):
        if (self._special_motion is not None
                and self._special_motion.is_running()):
            return "special_motion"
        if (self._controlled_spatial is not None
                and self._controlled_spatial.is_navigating()):
            return "controlled_spatial"
        return ""

    def is_moving(self):
        """Whether this call-shaped card currently owns an ongoing movement."""
        with self._lock:
            return (self._moving or self._stop is not None
                    or self._transition_stop is not None)

    def _preempt_servo(self, reason):
        if self._servo is None:
            return False
        return bool(self._servo.pause_for_explicit_command(reason))

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
            self._transition_stop = None
        if event is not None:
            event.set()

    def _finish_transition(self, stop_event):
        with self._lock:
            if self._transition_stop is stop_event:
                self._transition_stop = None

    def _worker_move(self, stop_event, vx, vy, yaw):
        with self.motion_lock:
            if stop_event.is_set() or self._moving_owner is not stop_event:
                return 0
            return self.proxy.Move(vx, vy, yaw)

    def _release_move(self, stop_event, stop=False):
        # A replaced worker may finish after the next command has started.
        # Only the current owner may stop or release that command.
        with self.motion_lock:
            if self._moving_owner is not stop_event:
                return
            try:
                if stop:
                    self.proxy.StopMove()
            except Exception:
                pass
            finally:
                self._moving = False
                self._moving_owner = None
                with self._lock:
                    if self._stop is stop_event:
                        self._stop = None
                    if self._transition_stop is stop_event:
                        self._transition_stop = None

    def _transition_to_balance_and_move(self, action_id, vx, vy, yaw, duration, stop_event):
        deadline = time.monotonic() + 12.0
        last_state = "UNKNOWN"
        last_move_ret = None
        while time.monotonic() < deadline:
            if stop_event.is_set():
                self._finish_transition(stop_event)
                self._release_move(stop_event)
                _acp_notify(action_id, "cancelled", {
                    "action": "move", "reason": "move transition was cancelled"})
                return
            name, state, error = self._read_state()
            last_state = name or last_state
            if name in {"BALANCE_STAND", "AI_BALANCE_STAND"}:
                return self._run_move_after_transition(action_id, vx, vy, yaw, duration, stop_event)
            if error:
                self._finish_transition(stop_event)
                self._release_move(stop_event)
                _acp_notify(action_id, "error", error)
                return
            # Some AS2 firmware keeps GetState at PASSIVE for a short time
            # after BalanceStand.  Do not wait forever for a state label that
            # is lagging behind the command path: probe Move in the worker and
            # continue as soon as the sport service accepts it.
            last_move_ret = self._worker_move(stop_event, vx, vy, yaw)
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
        self._release_move(stop_event)

    def _start_balance_move(self, state_name, vx, vy, yaw, duration):
        """Start the hidden AS2 balance transition and return an ACP action."""
        self._cancel_transition()
        self._stop_continuous()
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
            self._moving = True
            self._moving_owner = transition_stop
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
            self._release_move(stop_event)
            _acp_notify(action_id, "cancelled", {
                "action": "move", "reason": "move transition was cancelled"})
            return
        ret = 0 if move_already_sent else self._worker_move(stop_event, vx, vy, yaw)
        if ret != 0:
            self._finish_transition(stop_event)
            self._release_move(stop_event)
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
            with self.motion_lock:
                if stop_event.is_set() or self._moving_owner is not stop_event:
                    _acp_notify(action_id, "cancelled", {
                        "action": "move", "reason": "Move was replaced before continuous execution"})
                    return
                with self._lock:
                    self._stop = stop_event
            move_ret = 0
            move_error = None
            try:
                while not stop_event.is_set():
                    try:
                        move_ret = self._worker_move(stop_event, vx, vy, yaw)
                    except Exception as exc:
                        move_ret = 3104
                        move_error = f"{type(exc).__name__}: {str(exc)[:160]}"
                    if move_ret != 0:
                        break
                    stop_event.wait(0.1)
            finally:
                self._release_move(stop_event, stop=True)
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
        try:
            while time.monotonic() < deadline:
                remaining = deadline - time.monotonic()
                if stop_event.wait(min(0.1, remaining)):
                    _acp_notify(action_id, "cancelled", {
                        "action": "move", "duration": duration,
                        "reason": "Timed move stopped by stop_move or card shutdown"})
                    return
                if time.monotonic() < deadline:
                    ret = self._worker_move(stop_event, vx, vy, yaw)
                    if ret != 0:
                        _acp_notify(action_id, "error", {
                            "action": "move", "ret": ret, "rpc_ret": ret,
                            "duration": duration,
                            "error": "Timed Move failed",
                            "reason": "The sport controller stopped accepting the velocity command",
                            "suggested_actions": ["get_state", "stop_move"]})
                        return
        finally:
            self._release_move(stop_event, stop=True)
        _acp_notify(action_id, "completed",
                    {"action": "move", "ret": 0, "duration": duration,
                     "reason": "requested duration elapsed"})

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
                "description": "As2W locomotion. move uses signed vx forward/back m/s, vy lateral m/s, vyaw rotation degrees/s (converted to radians for the SDK), and duration seconds (-1 means continue until stop_move). stand_up/stand_down change posture; balance_stand enables active balance; recovery_stand is for fallen/down posture; body_height is an absolute AS2 target height in meters. Damp and Euler remain internal SDK operations and are not exposed by this card.", "inputSchema": {"type": "object", "properties": {
                    "action": {"type": "string", "enum": actions, "description": "Locomotion action"}, "vx": {"type": "number", "description": "Forward velocity m/s [-1.5, 1.5]"}, "vy": {"type": "number", "description": "Lateral velocity m/s [-1, 1]"}, "vyaw": {"type": "number", "description": "Yaw velocity in degrees/s [-120, 120]; converted to radians/s for Unitree SDK"},
                    "duration": {"type": "number", "minimum": -1, "maximum": 30, "description": "Seconds; -1 continues until stop_move"}, "yaw": {"type": "number", "description": "Body yaw radians"},
                    "speed_preset": {"type": "string", "enum": ["slow", "normal", "fast"], "description": "Speed limiter preset"}, "height": {"type": "number", "minimum": 0.17, "maximum": 0.50, "description": "Absolute AS2 body height in meters"}, "x": {"type": "number", "description": "Body X offset"}, "y": {"type": "number", "description": "Body Y offset"}, "z": {"type": "number", "description": "Body Z offset"}, "flag": {"type": "boolean", "description": "Used by four switch actions: true enables/enters and false disables/exits."}}, "required": ["action"],
                "x-completion": {"actions": ["move", "stop_move", "stand_up", "stand_down", "balance_stand", "recovery_stand"], "timeout": 45},
                "x-action-params": {
                    "move": {"params": ["vx", "vy", "vyaw", "duration"], "description": "Move with optional duration (-1 for continuous)."},
                    "stop_move": {"params": [], "description": "Stop movement."},
                    "stand_up": {"params": [], "description": "Stand up."}, "stand_down": {"params": [], "description": "Stand down."},
                    "balance_stand": {"params": [], "description": "Balance stand."}, "recovery_stand": {"params": [], "description": "Recovery stand."},
                    "speed_level": {"params": ["speed_preset"], "description": "Set speed limiter: slow, normal, or fast."}, "body_height": {"params": ["height"], "description": "Set the absolute AS2 body height target in meters (0.17-0.50)."},
                    "body_position": {"params": ["x", "y", "z", "yaw"], "description": "Set body position offset."},
                    "switch_joystick": {"params": ["flag"], "description": "true hands control to the wireless joystick; false disables it."}, "left_side_gait": {"params": ["flag"], "description": "Enter or exit left-side gait; true enters, false exits."},
                    "right_side_gait": {"params": ["flag"], "description": "Enter or exit right-side gait; true enters, false exits."}, "auto_recovery": {"params": ["flag"], "description": "Automatic fall recovery; true enables, false disables."},
                    "get_state": {"params": [], "description": "Read sport state."}}}}
    def start(self): pass
    def stop(self):
        with self.motion_lock:
            self._preempt_servo("loco.shutdown")
            self._cancel_transition()
            self._stop_continuous()
            self.proxy.StopMove()
            self._moving = False
            self._moving_owner = None
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
        with self.motion_lock:
            return self._dispatch(action, args)

    def _dispatch(self, action, args):
        if action in ("start", "info"): return {"state": "ready"}
        if action == "stop":
            self._preempt_servo("loco.stop")
            self._cancel_transition()
            self._stop_continuous()
            ret = self.proxy.StopMove()
            self._moving = False
            self._moving_owner = None
            return {"state": "idle", "ret": ret}
        if action != "get_state":
            owner = self._other_chassis_owner()
            if owner:
                return {"ret": -1, "accepted": False, "action": action,
                        "error": "Chassis is owned by another controller",
                        "reason": f"{owner} 正在占用底盘",
                        "suggested_actions": ["get_state"]}
        if action == "move":
            values = {}
            for name in ("vx", "vy", "vyaw"):
                values[name], error = _finite_argument(args, name)
                if error:
                    return error
            vx = max(-1.5, min(1.5, values["vx"]))
            vy = max(-1.0, min(1.0, values["vy"]))
            # PR #308 keeps the call-shaped loco API in degrees/s. The
            # continuous navigation card remains motus.control/1 radians/s.
            vyaw_deg = max(-120.0, min(120.0, values["vyaw"]))
            yaw = math.radians(vyaw_deg)
            duration = args.get("duration")
            if duration is not None:
                duration, error = _finite_argument(args, "duration")
                if error:
                    return error
                if duration == 0:
                    duration = None
            if duration is not None:
                if duration > 30:
                    return _invalid_argument(
                        "duration must be at most 30 seconds", "duration")
                if duration < 0 and duration != -1:
                    return _invalid_argument(
                        "duration must be -1, 0, or positive", "duration")
            # Validation comes first: a malformed request must not pause a
            # healthy navigation stream.  Once accepted, the explicit command
            # wins before either writer touches the chassis.
            preempted_servo = self._preempt_servo("loco.move")
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
            self._state_override = None
            self._state_override_until = 0.0
            self._state_hint = None
            if duration is None:
                self._stop_continuous()
                ret = self.proxy.Move(vx, vy, yaw)
                self._moving = ret == 0
                self._moving_owner = object() if ret == 0 else None
                return {"ret": ret, "accepted": ret == 0, "action": "move",
                        "current_state": state_name, "vx": vx, "vy": vy,
                        "vyaw": vyaw_deg, "vyaw_sdk_rad": yaw,
                        **({"preempted_loco_servo": True} if preempted_servo else {}),
                        **({} if ret == 0 else {"error": "Sport controller rejected Move",
                          "reason": "The robot is standing but the velocity command was refused",
                          "suggested_actions": ["get_state", "stop_move"]})}
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
                self._moving = True
                self._moving_owner = stop_event
                threading.Thread(target=self._run_continuous_move,
                                 args=(action_id, vx, vy, yaw, stop_event),
                                 daemon=True, name="as2w-loco-continuous-move").start()
                return {"ret": 0, "accepted": True, "status": "running",
                        "action": "move", "action_id": action_id,
                        "current_state": state_name, "duration": -1}
            if duration < 0:
                return _invalid_argument(
                    "duration must be -1, 0, or positive", "duration")
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
            self._moving = True
            action_id = f"as2w_loco_{uuid4().hex[:8]}"
            stop_event = threading.Event()
            with self._lock:
                self._transition_stop = stop_event
                self._moving_owner = stop_event
            threading.Thread(target=self._run_timed_move,
                             args=(action_id, vx, vy, yaw, duration, stop_event),
                             daemon=True, name="as2w-loco-timed-move").start()
            return {"ret": 0, "accepted": True, "status": "running",
                    "action": "move", "action_id": action_id,
                    "current_state": state_name, "duration": duration}
        if action == "stop_move":
            preempted_servo = self._preempt_servo("loco.stop_move")
            self._cancel_transition()
            self._stop_continuous()
            action_id = f"as2w_loco_{uuid4().hex[:8]}"
            self._moving = True
            self._moving_owner = action_id
            threading.Thread(target=self._stop_move_worker,
                             args=(action_id, action_id), daemon=True,
                             name="as2w-loco-stop-move").start()
            return {"ret": 0, "accepted": True, "status": "stopping",
                    "action": action, "action_id": action_id,
                    **({"preempted_loco_servo": True}
                       if preempted_servo else {})}
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
                threading.Thread(
                    target=self._stop_then_posture,
                    args=(action_id, action, method, expected_name, state_name),
                    daemon=True,
                    name="as2w-loco-stop-then-stand-down",
                ).start()
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
            if action == "recovery_stand" and state_name not in {
                    "FALL", "FALLEN", "STAND_DOWN", "DAMPING",
                    "AI_FALL", "AI_FALLEN", "AI_STAND_DOWN", "AI_DAMPING"}:
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
                        "reason": "BodyHeight needs an absolute target in meters",
                        "suggested_actions": ["stand_up", "balance_stand"]}
            height, error = _finite_argument(args, "height")
            if error:
                return error
            if not _BODY_HEIGHT_MIN_M <= height <= _BODY_HEIGHT_MAX_M:
                return {"ret": -1, "accepted": False, "action": action,
                        "error": (
                            f"height must be between {_BODY_HEIGHT_MIN_M:.2f} "
                            f"and {_BODY_HEIGHT_MAX_M:.2f} meters"),
                        "suggested_actions": ["get_state"]}
            state_name, state, state_error = self._read_state()
            if state_error:
                return {**state_error, "action": action, "height_m": height}
            if state_name in self._DOWN or self._is_moving(state_name):
                return self._not_allowed(
                    action, state_name,
                    "BodyHeight requires a standing, non-walking posture",
                    ["stand_up", "balance_stand", "stop_move"])
            ret = self.proxy.BodyHeight(height)
            result = {"ret": ret, "accepted": ret == 0, "action": action,
                      "height_m": height, "sdk_height_m": height,
                      "current_state": state_name}
            if ret != 0:
                result.update({"rpc_ret": ret,
                               "error": "SportClient rejected BodyHeight",
                               "reason": "The controller did not accept the target height",
                               "suggested_actions": ["balance_stand", "get_state"]})
            return result
        if action == "body_position":
            values = {}
            for name in ("x", "y", "z", "yaw"):
                values[name], error = _finite_argument(args, name)
                if error:
                    return error
            return {"ret": self.proxy.BodyPosition(
                values["x"], values["y"], values["z"], values["yaw"])}
        flag_methods = {
            "auto_recovery": "SetAutoRecovery",
            "switch_joystick": "SwitchJoystick",
            "left_side_gait": "LeftSideGait",
            "right_side_gait": "RightSideGait",
        }
        if action in flag_methods:
            flag, error = _boolean_argument(args, "flag")
            if error:
                return error
            return {"ret": getattr(self.proxy, flag_methods[action])(
                1 if flag else 0)}
        if action == "get_state":
            state_name, state, state_error = self._read_state()
            if state_error:
                return {**state_error, "action": action, "state": state or {}}
            return {"ret": 0, "state": state, "current_state": state_name,
                    "state_source": state.get(
                        "state_source", "sport_client.get_state")}
        return None

    def _run_timed_move(self, action_id, vx, vy, yaw, duration, stop_event):
        self._run_move_after_transition(action_id, vx, vy, yaw, duration,
                                        stop_event, move_already_sent=True)

    def _run_continuous_move(self, action_id, vx, vy, yaw, stop_event):
        try:
            while not stop_event.wait(0.1):
                ret = self._worker_move(stop_event, vx, vy, yaw)
                if ret != 0:
                    _acp_notify(action_id, "error", {
                        "action": "move", "ret": ret, "rpc_ret": ret,
                        "error": "Continuous Move failed",
                        "reason": "The sport controller stopped accepting the velocity command",
                        "suggested_actions": ["get_state", "stop_move"]})
                    return
        finally:
            self._release_move(stop_event, stop=True)
        _acp_notify(action_id, "cancelled", {
            "action": "move", "ret": 0, "duration": -1,
            "reason": "Continuous move stopped by stop_move or card shutdown"})

    def _await_stopped(self, action_id, completed_action="stop_move",
                       completion_extra=None):
        """Normalize AS2's stale walking label after an accepted stop."""
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
                          "state": state,
                          "reason": "controller left walking state"}
                if completion_extra:
                    result.update(completion_extra)
                _acp_notify(action_id, "completed", result)
                return
            if not balance_requested:
                balance_requested = True
                balance_ret = self.proxy.BalanceStand()
                print(
                    f"[loco] stop stabilization action_id={action_id} "
                    f"state={name} BalanceStand ret={balance_ret}",
                    flush=True,
                )
                if balance_ret != 0:
                    break
                self._state_override = "BALANCE_STAND"
                self._state_override_until = time.monotonic() + 10.0
                self._state_hint = "BALANCE_STAND"
            time.sleep(0.1)
        if balance_ret == 0:
            result = {"action": completed_action, "ret": 0,
                      "current_state": last_state, "balance_ret": 0,
                      "state_stale": True,
                      "reason": (
                          "StopMove and BalanceStand were accepted; AS2 "
                          "GetState still reports a stale walking label")}
            if completion_extra:
                result.update(completion_extra)
            _acp_notify(action_id, "completed", result)
            return
        _acp_notify(action_id, "error", {
            "action": completed_action, "ret": 0,
            "current_state": last_state, "balance_ret": balance_ret,
            "error": "StopMove was accepted but balance normalization failed",
            "reason": (
                "The controller did not accept BalanceStand after StopMove"),
            "suggested_actions": [
                "get_state", "retry_stop", "balance_stand"]})

    def _stop_then_posture(self, action_id, action, method, expected_name,
                           state_name):
        """Serialize StopMove before a posture command from a walking FSM."""
        try:
            with self.motion_lock:
                self._cancel_transition()
                self._stop_continuous()
                stop_ret = self.proxy.StopMove()
                if stop_ret == 0:
                    self._moving = False
                    self._moving_owner = None
        except Exception as exc:
            _acp_notify(action_id, "error", {
                "action": action, "ret": 3104,
                "current_state": state_name,
                "error": "StopMove RPC failed before posture transition",
                "reason": (
                    "Could not stop the active velocity command: "
                    f"{type(exc).__name__}: {str(exc)[:160]}"),
                "suggested_actions": ["get_state", "retry_stop"]})
            return
        if stop_ret != 0:
            _acp_notify(action_id, "error", self._rpc_rejected(
                action, state_name, stop_ret,
                "StopMove was rejected, so the posture transition was not sent",
                ["get_state", "stop_move", action]))
            return
        time.sleep(0.2)
        try:
            with self.motion_lock:
                ret = getattr(self.proxy, method)()
        except Exception as exc:
            _acp_notify(action_id, "error", {
                "action": action, "ret": 3104,
                "current_state": state_name, "stop_ret": stop_ret,
                "error": "Posture RPC failed after StopMove",
                "reason": (
                    f"{method} could not reach the sport controller: "
                    f"{type(exc).__name__}: {str(exc)[:160]}"),
                "suggested_actions": ["get_state", "retry_stop", action]})
            return
        if ret != 0:
            _acp_notify(action_id, "error", self._rpc_rejected(
                action, state_name, ret,
                f"{method} was rejected after StopMove",
                ["get_state", "retry_stop", "balance_stand", action]))
            return
        self._state_hint = None
        self._state_override = None
        self._state_override_until = 0.0
        self._await_posture(action_id, action, expected_name)

    def _stop_move_worker(self, action_id, owner=None):
        try:
            with self.motion_lock:
                if owner is not None and self._moving_owner != owner:
                    _acp_notify(action_id, "cancelled", {
                        "action": "stop_move", "reason": "Replaced by a newer loco command"})
                    return
                ret = self.proxy.StopMove()
                if ret == 0:
                    self._moving = False
                    self._moving_owner = None
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


class SpecialMotionPlugin:
    """As2W-specific discrete motions provided by the official SportClient."""
    PREFIX = "special_motion"

    def __init__(self, config, namespace, executor, proxy):
        self.proxy = proxy
        self._active_posture = None
        self._state_lock = threading.Lock()
        self._motion_lock = threading.Lock()
        self._motion_generation = 0
        self._response_events = {}
        self._response_lock = threading.Lock()
        self._servo = None
        self._loco = None
        self._controlled_spatial = None
        self._chassis_lock = threading.RLock()

    def attach_servo(self, servo):
        self._servo = servo
        if self._loco is None:
            self._chassis_lock = servo._lock

    def attach_loco(self, loco):
        self._loco = loco
        self._chassis_lock = loco.motion_lock

    def attach_controlled_spatial(self, plugin):
        self._controlled_spatial = plugin

    def is_running(self):
        with self._state_lock:
            active_posture = self._active_posture is not None
        return self._motion_lock.locked() or active_posture

    def _other_chassis_owner(self):
        if self._loco is not None and self._loco.is_moving():
            return "loco"
        if (self._controlled_spatial is not None
                and self._controlled_spatial.is_navigating()):
            return "controlled_spatial"
        return ""

    def _preempt_servo(self, reason):
        if self._servo is None:
            return False
        return bool(self._servo.pause_for_explicit_command(reason))

    def get_tool(self):
        actions = ["front_flip", "back_flip", "handstand", "biped_stand"]
        return {"name": "special_motion", "type": "actuator", "multiInstance": False,
                "description": "As2W discrete acrobatic motions via the official SportClient. The robot must be standing and balanced, the area must be clear, and confirm must be the literal boolean true. Flips are one-shot actions; handstand and biped_stand are sustained postures that support enter=false to exit.",
                "inputSchema": {"type": "object", "properties": {
                    "action": {"type": "string", "enum": actions},
                    "enter": {"type": "boolean", "description": "For handstand/biped_stand: true enters and false exits the sustained posture."},
                    "confirm": {"type": "boolean", "description": "Must be the literal boolean true for every hazardous motion."}},
                    "required": ["action"],
                    "x-is-dangerous": True,
                    "x-completion": {"actions": actions, "timeout": 45},
                    "x-action-params": {
                        "front_flip": {"params": ["confirm"], "description": "DANGEROUS forward flip; requires confirm=true."},
                        "back_flip": {"params": ["confirm"], "description": "DANGEROUS backward flip; requires confirm=true."},
                        "handstand": {"params": ["enter", "confirm"], "description": "DANGEROUS sustained handstand; confirm=true is required and enter=false exits."},
                        "biped_stand": {"params": ["enter", "confirm"], "description": "DANGEROUS sustained biped stand; confirm=true is required and enter=false exits."}}}}

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

    def _run_motion(self, action_id, action, enter, generation,
                    response_event):
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
            if status == "completed":
                result["completion_scope"] = "sport_client_rpc_accepted"

        self._motion_lock.release()
        # ACP completion must not overtake the MCP response that gives the
        # caller this action_id. The HTTP/SSE handler releases this barrier in
        # a finally block after attempting to send that response.
        response_event.wait()
        _acp_notify(action_id, status, result, tool="special_motion")

    def response_sent(self, action_id):
        """Release a terminal ACP event after its MCP action response."""
        with self._response_lock:
            response_event = self._response_events.pop(action_id, None)
        if response_event is not None:
            response_event.set()

    def _start_motion(self, action, args):
        if not self._motion_lock.acquire(blocking=False):
            return {"error": "another special motion is still running"}
        action_id = f"as2w_special_motion_{uuid4().hex[:8]}"
        enter = bool(args.get("enter", True))
        with self._state_lock:
            self._motion_generation += 1
            generation = self._motion_generation
        response_event = threading.Event()
        with self._response_lock:
            self._response_events[action_id] = response_event
        try:
            threading.Thread(target=self._run_motion,
                             args=(action_id, action, enter, generation,
                                   response_event),
                             daemon=True).start()
        except Exception:
            with self._response_lock:
                self._response_events.pop(action_id, None)
            self._motion_lock.release()
            raise
        return {"accepted": True, "status": "running", "queued": True,
                "action": action, "action_id": action_id}

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
            with self._chassis_lock:
                owner = self._other_chassis_owner()
                if owner:
                    return {"ret": -1, "accepted": False, "action": action,
                            "error": "Chassis is owned by another controller",
                            "reason": f"{owner} 正在占用底盘"}
                # Reserve the shared handover while stopping the stream and
                # acquiring this card's own motion slot.
                self._preempt_servo(f"special_motion.{action}")
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
                    "red": {"type": "integer", "minimum": 0, "maximum": 255, "description": "Red channel 0-255."},
                    "green": {"type": "integer", "minimum": 0, "maximum": 255, "description": "Green channel 0-255."},
                    "blue": {"type": "integer", "minimum": 0, "maximum": 255, "description": "Blue channel 0-255."}},
                    "required": ["action"],
                    "x-action-params": {
                        "set_color": {"params": ["red", "green", "blue"], "description": "Set RGB color."},
                        "off": {"params": [], "description": "Turn the LED off."},
                        "info": {"params": [], "description": "Read the selected color."}}}}

    def start(self, allow_black=False):
        # Avoid occupying the audio RPC worker with black refreshes during
        # bundle startup. Explicit off is different: it must keep black alive.
        if not allow_black and not any(self._color):
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
            # AS2 firmware can restore its default LED after a timeout. Black
            # therefore needs the same refresh loop as visible colors.
            self.start(allow_black=action == "off")
        return {"ret": result, "color": list(color)}
