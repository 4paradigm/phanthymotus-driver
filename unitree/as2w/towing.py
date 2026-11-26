"""AS2W towing control and its driver integration."""
from collections import deque
from collections.abc import Mapping
from dataclasses import asdict, dataclass
import json
import logging
import math
from types import SimpleNamespace
import threading
import time


LOG = logging.getLogger(__name__)
MAX_SPEED_MPS = 0.5
COMMAND_EPS = 1e-6


def _number(value):
    if type(value) not in (int, float):
        return None
    try:
        return float(value) if math.isfinite(value) else None
    except OverflowError:
        return None


@dataclass(frozen=True)
class TowingConfig:
    enabled: bool = False
    loop_hz: float = 20
    publish_hz: float = 5
    max_speed_mps: float = MAX_SPEED_MPS
    min_speed_mps: float = 0.05
    drag_warn_ratio: float = 0.25
    drag_stop_ratio: float = 0.6
    stuck_speed_eps_mps: float = 0.02
    stuck_hold_ms: float = 800
    detach_drop_ratio: float = 0.5
    detach_accel_mps2: float = 0.8
    slope_warn_deg: float = 8.0
    freshness_timeout_ms: float = 300
    loop_timeout_ms: float = 500
    zero_frame_count: int = 5

    @classmethod
    def validate(cls, raw):
        if not isinstance(raw, Mapping):
            return None, "towing"
        bounds = {
            "loop_hz": (0, 100), "publish_hz": (0, 100),
            "max_speed_mps": (0, MAX_SPEED_MPS), "min_speed_mps": (0, MAX_SPEED_MPS),
            "drag_warn_ratio": (0, 1), "drag_stop_ratio": (0, 1),
            "stuck_speed_eps_mps": (0, MAX_SPEED_MPS), "stuck_hold_ms": (0, 60000),
            "detach_drop_ratio": (0, 2), "detach_accel_mps2": (0, math.inf),
            "slope_warn_deg": (0, 90), "freshness_timeout_ms": (0, 60000),
            "loop_timeout_ms": (0, 60000),
        }
        defaults = asdict(cls())
        for field in raw:
            if field not in defaults:
                return None, field
        values = {**defaults, **raw}
        if type(values["enabled"]) is not bool:
            return None, "enabled"
        for field, (low, high) in bounds.items():
            value = _number(values[field])
            if value is None or not low < value <= high:
                return None, field
        frames = values["zero_frame_count"]
        if type(frames) is not int or not 1 <= frames <= 100:
            return None, "zero_frame_count"
        config = cls(**values)
        if config.min_speed_mps > config.max_speed_mps:
            return None, "min_speed_mps"
        if config.drag_warn_ratio >= config.drag_stop_ratio:
            return None, "drag_warn_ratio"
        if config.stuck_speed_eps_mps * 3 >= config.max_speed_mps:
            return None, "stuck_speed_eps_mps"
        if config.publish_hz > config.loop_hz:
            return None, "publish_hz"
        if config.loop_timeout_ms <= 1000 / config.loop_hz:
            return None, "loop_timeout_ms"
        return config, None


class SpeedChannel:
    """Shares the chassis admission lock and checks existing owner reservations."""

    def __init__(self, lock=None):
        self.lock = lock if lock is not None else threading.RLock()
        self._claimant = None
        self._unconfirmed_stop = None
        self._uncertain = set()
        self._existing = {}

    @property
    def owner(self):
        if self._claimant is not None:
            return "towing"
        return "towing_fault" if self._unconfirmed_stop is not None else None

    def observe(self, owner, active):
        self._existing[owner] = active

    def acquire(self, claimant):
        with self.lock:
            if self.owner is not None:
                return self.owner
            for name, active in self._existing.items():
                if name in self._uncertain or active():
                    return name
            self._claimant = claimant
            return None

    def retry_stop(self, claimant):
        with self.lock:
            if self._unconfirmed_stop is not claimant:
                return False
            self._claimant = claimant
            self._unconfirmed_stop = None
            return True

    def release(self, claimant, *, stop_unconfirmed=False):
        with self.lock:
            if self._claimant is claimant:
                self._unconfirmed_stop = claimant if stop_unconfirmed else None
                self._claimant = None


class _ChassisProxy:
    """Apply admission at the last SDK write for legacy velocity owners."""

    def __init__(self, proxy, channel, name):
        self._proxy, self._channel, self._name = proxy, channel, name

    def __getattr__(self, name):
        return getattr(self._proxy, name)

    def Move(self, *args):
        return self._send("Move", *args)

    def StopMove(self):
        return self._send("StopMove")

    def GetState(self):
        with self._channel.lock:
            if self._channel.owner:
                return 3104, {}
            return self._proxy.GetState()

    def _send(self, method, *args):
        channel = self._channel
        with channel.lock:
            if channel.owner:
                return 3104
            try:
                ret = getattr(self._proxy, method)(*args)
            except Exception:
                channel._uncertain.add(self._name)
                raise
            if method == "StopMove" and ret == 0:
                channel._uncertain.clear()
            elif method == "StopMove" or ret == 3104:
                channel._uncertain.add(self._name)
            return ret


def _derated_speed(config, cruise, ratio):
    fraction = (ratio - config.drag_warn_ratio) / (config.drag_stop_ratio - config.drag_warn_ratio)
    floor = min(cruise, config.min_speed_mps)
    return cruise - (cruise - floor) * max(0.0, min(1.0, fraction))


def _drag_ratio(command, measured):
    return max(-1.0, min(1.0, (command - measured) / max(command, COMMAND_EPS)))


def _trend(current, previous):
    if previous is None or current is None:
        return None
    if math.isclose(current, previous, abs_tol=1e-9):
        return "steady"
    return "rising" if current > previous else "falling"


class _TowingCard:
    def __init__(self, config, namespace, proxy, speed_channel, *,
                 clock=time.monotonic, wait=time.sleep):
        self.topic = f"/{namespace}/state/towing"
        self._raw_config = config
        self._proxy, self._channel = proxy, speed_channel
        self._clock, self._wait = clock, wait
        self._lock = threading.RLock()
        self._feedback_lock = threading.Lock()
        self._speed = self._imu = self._current = None
        self._publisher = self._node = None
        self._message = SimpleNamespace
        self._closed = False
        self._config = TowingConfig()
        self._state = "idle"
        self._errors = {}
        self._last_error_log = {}
        self._events = deque(maxlen=16)
        self._reset()

    def _reset(self):
        self._load = None
        self._busy_owner = None
        self._cruise = self._cmd = 0.0
        self._last_tick = self._last_publish = None
        self._startup_until = None
        self._stuck_since = None
        self._last_speed = self._last_imu = self._last_current = None
        self._ratio = self._drag_trend = self._slope = None
        self._current_trend = None
        self._limited_by = self._error = None
        self._events.clear()
        self._flags = dict(stuck=False, detached=False, speed_channel_busy=False)

    def tool(self):
        return {
            "name": "towing", "type": "actuator", "multiInstance": False,
            "description": "AS2W straight-line towing with drag derating and automatic stop.",
            "inputSchema": {"type": "object", "properties": {
                "action": {"type": "string", "enum": ["start", "stop", "info"]},
                "load_class": {"type": "string", "minLength": 1, "maxLength": 64},
                "vx": {"type": "number", "exclusiveMinimum": 0, "maximum": MAX_SPEED_MPS,
                       "description": "Requested forward cruise speed in m/s, within configured min/max"},
            }, "required": ["action"]},
            "topic_out": [{"topic": self.topic, "format": "data/json"}],
            "x-action-params": {
                "start": {"params": ["load_class", "vx"], "description": "Arm and start towing"},
                "stop": {"params": [], "description": "Send zero speed and release the chassis"},
                "info": {"params": [], "description": "Read towing state and recent events"},
            },
        }

    def bind(self, node, message_type=SimpleNamespace):
        with self._lock:
            if self._node is not None or self._closed:
                raise RuntimeError("towing cannot bind to this node")
            self._publisher = node.create_publisher(message_type, self.topic, 1)
            self._node, self._message = node, message_type

    def detach(self, node):
        with self._lock:
            if self._node is node and self._publisher is not None:
                self._finish()
                publisher = self._publisher
                self._node = self._publisher = None
                node.destroy_publisher(publisher)

    def shutdown(self):
        with self._lock:
            self._closed = True
            self._finish()

    def stop(self):
        with self._lock:
            self._finish()
            return self._status(self._clock())

    def on_speed(self, received, vx, wz):
        with self._feedback_lock:
            self._speed = (received, _number(vx), _number(wz))

    def on_imu(self, received, pitch_deg):
        with self._feedback_lock:
            self._imu = (received, _number(pitch_deg))

    def on_current(self, received, reported_ma):
        with self._feedback_lock:
            self._current = (received, _number(reported_ma))

    def _feedback(self, now):
        max_age = self._config.freshness_timeout_ms / 1000
        with self._feedback_lock:
            now = max(now, self._clock())
            def fresh(sample):
                return (sample if sample and 0 <= now - sample[0] <= max_age
                        and all(v is not None for v in sample[1:]) else None)
            return fresh(self._speed), fresh(self._imu), fresh(self._current)

    def _status(self, now):
        speed, imu, current = self._feedback(now)
        return dict(
            ts_mono=now, state=self._state,
            owner=self._channel.owner or self._busy_owner, load_class=self._load,
            cmd=dict(vx=self._cmd, wz=0.0),
            meas=dict(vx=speed[1] if speed else None, wz=speed[2] if speed else None),
            drag_ratio=self._ratio, drag_trend=self._drag_trend,
            slope_deg=self._slope if imu else None, limited_by=self._limited_by,
            flags=dict(self._flags), events=[dict(event) for event in self._events],
            fresh=bool(speed and imu), error=self._error,
            slope_warning=bool(imu and self._slope is not None
                               and abs(self._slope) > self._config.slope_warn_deg),
            bms_reported_ma=current[1] if current else None,
            bms_current_magnitude_trend=self._current_trend if current else None,
            electrical_power_w=None, traction_force_n=None, error_counts=dict(self._errors),
        )

    def _record_error(self, kind, detail):
        now = self._clock()
        self._errors[kind] = self._errors.get(kind, 0) + 1
        if now - self._last_error_log.get(kind, -math.inf) >= 10:
            self._last_error_log[kind] = now
            LOG.warning("AS2W towing %s: %s", kind, detail)

    def _publish(self, now):
        if self._publisher is None:
            return
        self._last_publish = now
        try:
            message = self._message()
            message.data = json.dumps(self._status(now), allow_nan=False, separators=(",", ":"))
            self._publisher.publish(message)
        except Exception as exc:
            self._record_error("publish", exc)

    def _transition(self, state):
        self._state = state
        self._publish(self._clock())

    def _reject(self, error, code, **details):
        self._error = error
        self._record_error(code, error)
        self._transition("idle")
        return dict(error=error, code=code, **details)

    def dispatch(self, action, args):
        if args.get("_tool_name") != "towing":
            return None
        if action == "start":
            return self.start(args.get("load_class"), args.get("vx"))
        if action == "stop":
            return self.stop()
        if action == "info":
            with self._lock:
                return self._status(self._clock())
        return {"error": "unknown towing action", "code": "INVALID_ARGUMENT"}

    def start(self, load_class, vx):
        with self._lock:
            if self._closed:
                return {"error": "towing unavailable", "code": "UNAVAILABLE"}
            if self._state != "idle":
                return {"error": "speed channel busy", "owner": "towing"}
            if self._channel.owner == "towing_fault":
                return {"error": "speed channel busy", "owner": "towing_fault"}
            self._reset()
            self._transition("arming")
            config, field = TowingConfig.validate(self._raw_config)
            if field is not None:
                return self._reject("invalid towing configuration", "INVALID_CONFIG", field=field)
            self._config = config
            if not config.enabled:
                return self._reject("towing is disabled", "DISABLED")
            vx = _number(vx)
            if vx is None or not config.min_speed_mps <= vx <= config.max_speed_mps:
                return self._reject("invalid cruise speed", "INVALID_ARGUMENT", field="vx")
            if not isinstance(load_class, str) or not 1 <= len(load_class.strip()) <= 64:
                return self._reject("invalid load class", "INVALID_ARGUMENT", field="load_class")
            try:
                owner = self._channel.acquire(self)
            except Exception as exc:
                return self._reject(str(exc), "CHANNEL_ERROR")
            if owner:
                self._busy_owner = owner
                self._flags["speed_channel_busy"] = True
                self._reject("speed channel busy", "CHANNEL_BUSY")
                return {"error": "speed channel busy", "owner": owner}

            now = self._clock()
            speed, imu, _ = self._feedback(now)
            if speed is None or imu is None or self._publisher is None:
                self._channel.release(self)
                return self._reject("fresh speed/IMU and publisher required", "NOT_READY")
            self._load, self._cruise = load_class.strip(), vx
            self._last_tick = now
            if abs(speed[1]) <= config.stuck_speed_eps_mps:
                self._startup_until = now + config.stuck_hold_ms / 1000
                if vx > 3 * config.stuck_speed_eps_mps:
                    self._stuck_since = now
            self._slope = imu[1]
            self._transition("towing")
            self._last_speed = speed
            self._last_imu = imu[0]
            self._ratio = _drag_ratio(vx, speed[1])
            if not self._send(vx):
                self._finish("fault", "initial Move failed")
            return self._status(self._clock())

    def _send(self, vx):
        value = _number(vx)
        self._cmd = max(0.0, min(value if value is not None else 0.0,
                                 self._config.max_speed_mps, MAX_SPEED_MPS))
        try:
            ret = self._sport_call("Move", self._cmd, 0.0, 0.0)
            if ret == 0:
                return True
            self._record_error("Move", f"returned {ret}")
        except Exception as exc:
            self._record_error("Move", exc)
        return False

    def _sport_call(self, method, *args):
        bounded_call = getattr(type(self._proxy), "towing_call", None)
        if bounded_call is None:
            return getattr(self._proxy, method)(*args)
        timeout = min(1 / self._config.loop_hz, self._config.loop_timeout_ms / 2000)
        return bounded_call(self._proxy, method, timeout, *args)

    def _finish(self, event=None, detail=None):
        if self._state != "towing":
            if not self._channel.retry_stop(self):
                return
            self._error = None
        if event:
            self._events.append(dict(type=event, ts_mono=self._clock(), detail=detail))
            if event == "fault":
                self._error = detail
                self._record_error("fault", detail)
            else:
                self._flags[event] = True
            self._limited_by = event
        self._transition("stopping")
        stopped = False
        try:
            accepted = True
            for index in range(self._config.zero_frame_count):
                if index:
                    self._wait(1 / self._config.loop_hz)
                accepted = self._send(0.0) and accepted
            stopped = accepted
            if not accepted:
                self._error = "zero velocity sequence was not acknowledged"
                self._events.append(dict(type="fault", ts_mono=self._clock(), detail=self._error))
                try:
                    ret = self._sport_call("StopMove")
                    stopped = ret == 0
                    if not stopped:
                        self._record_error("StopMove", f"returned {ret}")
                except Exception as exc:
                    self._record_error("StopMove", exc)
        finally:
            self._channel.release(self, stop_unconfirmed=not stopped)
            self._transition("idle")

    def tick(self):
        with self._lock:
            if self._state != "towing":
                return
            try:
                self._tick(self._clock())
            except Exception as exc:
                self._finish("fault", f"towing tick failed: {exc}")

    def _tick(self, now):
        config = self._config
        elapsed = now - self._last_tick
        self._last_tick = now
        if elapsed > config.loop_timeout_ms / 1000:
            self._finish("fault", "loop timeout")
            return
        speed, imu, current = self._feedback(now)
        if speed is None or imu is None:
            self._finish("fault", "speed or IMU feedback timeout")
            return

        old_slope_warning = abs(self._slope) > config.slope_warn_deg
        old_imu_time = self._last_imu
        if old_imu_time is None or imu[0] > old_imu_time:
            interval = elapsed if old_imu_time is None else imu[0] - old_imu_time
            self._slope += -math.expm1(-interval) * (imu[1] - self._slope)
            self._last_imu = imu[0]
        slope_changed = old_slope_warning != (abs(self._slope) > config.slope_warn_deg)
        if current and (self._last_current is None or current[0] > self._last_current[0]):
            self._current_trend = _trend(abs(current[1]), abs(self._last_current[1])
                                         if self._last_current else None)
            self._last_current = current
        vx = speed[1]
        was_starting = self._startup_until is not None
        holding = vx <= config.stuck_speed_eps_mps and self._cmd > 3 * config.stuck_speed_eps_mps
        if holding and self._stuck_since is None:
            self._stuck_since = now
        elif not holding:
            self._stuck_since = None
        held = self._stuck_since is not None and now - self._stuck_since >= config.stuck_hold_ms / 1000

        new_speed = self._last_speed is None or speed[0] > self._last_speed[0]
        if new_speed:
            ratio = _drag_ratio(self._cmd, vx)
            previous_ratio, self._ratio = self._ratio, ratio
            self._drag_trend = _trend(ratio, previous_ratio)
            prior = self._last_speed
            self._last_speed = speed
            if (prior is not None and not was_starting
                    and previous_ratio - ratio > config.detach_drop_ratio
                    and _drag_ratio(self._cmd, prior[1]) - ratio > config.detach_drop_ratio
                    and vx - prior[1] > config.detach_accel_mps2 * (speed[0] - prior[0])):
                self._finish("detached", f"drag ratio fell from {previous_ratio:.3f} to {ratio:.3f}")
                return
        if was_starting and (now >= self._startup_until or self._ratio < config.drag_stop_ratio):
            self._startup_until = None
        starting = self._startup_until is not None
        if held or (not starting and self._ratio >= config.drag_stop_ratio):
            self._finish("stuck", f"ratio {self._ratio:.3f}")
            return

        previous_limit = self._limited_by
        warning = not starting and self._ratio >= config.drag_warn_ratio
        vx_next = self._cruise
        if warning:
            vx_next = _derated_speed(config, self._cruise, self._ratio)
            if previous_limit:
                vx_next = min(self._cmd, vx_next)
        elif previous_limit:
            # Restoring cruise immediately can recreate the same drag on the
            # next tick. Increase only when the measured speed leaves headroom.
            recovery = vx / (1 - config.drag_warn_ratio / 2)
            vx_next = min(self._cruise, max(self._cmd, recovery))
        derating = warning or vx_next < self._cruise
        self._limited_by = "drag_derate" if derating else None
        if not self._send(vx_next):
            self._finish("fault", "Move failed")
            return
        finished = self._clock()
        if finished - now > config.loop_timeout_ms / 1000:
            self._finish("fault", "loop timeout")
            return
        if derating and previous_limit != self._limited_by:
            self._events.append(dict(type="derate", ts_mono=finished, detail=f"ratio {self._ratio:.3f}"))
        if (previous_limit != self._limited_by or slope_changed
                or finished - self._last_publish >= 1 / config.publish_hz):
            self._publish(finished)

    def loop_delay(self):
        with self._lock:
            if self._state != "towing":
                return 0.05
            return max(0.0, 1 / self._config.loop_hz - (self._clock() - self._last_tick))


def _vector(values, count):
    if values is None or len(values) < count:
        return None
    result = tuple(_number(value) for value in values[:count])
    return result if all(value is not None for value in result) else None


class TowingPlugin:
    """DDS and ROS adapters around the standalone controller."""

    def __init__(self, cfg, namespace, proxy, dds_ready, loco, servo, special, spatial):
        config = cfg.get("towing", {})
        self._configured, _ = TowingConfig.validate(config)
        self._dds_ready = dds_ready
        self._node = None
        self._subscribers = []
        self._done = threading.Event()
        self._wake = threading.Event()
        self._thread = None
        self._lifecycle = threading.RLock()

        # Existing owner probes are used solely for admission. Feedback and
        # commands go directly to DDS and the shared SportClient proxy.
        lock = loco.motion_lock if loco is not None else servo._lock if servo is not None else None
        channel = SpeedChannel(lock)
        if self._configured is not None and self._configured.enabled and dds_ready:
            for plugin, lock_name, owner, probe in (
                (loco, "motion_lock", "loco", "owns_speed_channel"),
                (servo, "_lock", "loco_servo", "owns_speed_channel"),
                (special, "_chassis_lock", "special_motion", "is_running"),
                (spatial, "_chassis_lock", "controlled_spatial", "is_navigating"),
            ):
                if plugin is not None:
                    setattr(plugin, lock_name, channel.lock)
                    plugin._towing_channel = channel
                    channel.observe(owner, getattr(plugin, probe))
            if loco is not None:
                loco.proxy = _ChassisProxy(loco.proxy, channel, "loco")
            if servo is not None:
                servo._proxy = _ChassisProxy(servo._proxy, channel, "loco_servo")
        self._card = _TowingCard(config, namespace, proxy, channel)

    def get_tools(self):
        return [self._card.tool()]

    def start(self):
        with self._lifecycle:
            if (self._done.is_set() or self._thread is not None or not self._dds_ready
                    or self._configured is None or not self._configured.enabled):
                return
            try:
                from rclpy.node import Node
                from std_msgs.msg import String
                from unitree_sdk2py.core.channel import ChannelSubscriber
                from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_
                from unitree_sdk2py.idl.unitree_hg.msg.dds_ import BmsState_, LowState_

                self._node = Node("as2w_towing")
                self._card.bind(self._node, String)
                for topic, kind, callback in (
                    ("rt/lf/sportmodestate", SportModeState_, self._on_sport),
                    ("rt/lowstate", LowState_, self._on_low),
                    ("rt/lf/bmsstate", BmsState_, self._on_bms),
                ):
                    subscriber = ChannelSubscriber(topic, kind)
                    self._subscribers.append(subscriber)
                    if subscriber.Init(callback, 0) is False:
                        raise RuntimeError(f"could not subscribe to {topic}")
                self._thread = threading.Thread(
                    target=self._run, daemon=True, name="as2w-towing")
                self._thread.start()
            except Exception as exc:
                LOG.warning("AS2W towing feedback setup failed: %s", exc)
                self.stop()

    def stop(self):
        with self._lifecycle:
            self._done.set()
            self._wake.set()
            self._card.shutdown()
            if self._thread is not None and self._thread.is_alive():
                self._thread.join()
            for subscriber in self._subscribers:
                try:
                    subscriber.Close()
                except Exception as exc:
                    LOG.warning("AS2W towing DDS cleanup failed: %s", exc)
            self._subscribers.clear()
            if self._node is not None:
                self._card.detach(self._node)
                self._node.destroy_node()
                self._node = None

    def dispatch(self, action, args):
        result = self._card.dispatch(action, args)
        if action == "start":
            self._wake.set()
        return result

    def _run(self):
        while not self._done.is_set():
            if self._wake.wait(self._card.loop_delay()):
                self._wake.clear()
                continue
            if not self._done.is_set():
                self._card.tick()

    def _on_low(self, message):
        now = time.monotonic()
        rpy = _vector(getattr(getattr(message, "imu_state", None), "rpy", None), 3)
        self._card.on_imu(now, math.degrees(rpy[1]) if rpy and abs(rpy[1]) <= math.pi else None)

    def _on_sport(self, message):
        now = time.monotonic()
        velocity = _vector(getattr(message, "velocity", None), 2)
        imu = getattr(message, "imu_state", None)
        quaternion = _vector(getattr(imu, "quaternion", None), 4)
        gyro = _vector(getattr(imu, "gyroscope", None), 3)
        vx = None
        if velocity and quaternion and (norm := math.hypot(*quaternion)):
            w, x, y, z = (value / norm for value in quaternion)
            yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
            vx = math.cos(yaw) * velocity[0] + math.sin(yaw) * velocity[1]
        self._card.on_speed(now, vx, gyro[2] if gyro else None)

    def _on_bms(self, message):
        self._card.on_current(time.monotonic(), getattr(message, "current", None))
