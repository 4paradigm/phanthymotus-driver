"""Tianyi body-frame velocity stream: domain 42 JSON -> Slamware SDK.

No motion on bundle startup. Only MCP start attaches the input. No REST
MoveByAction conversion: it cannot represent a simultaneous vx/wz command.
"""

from __future__ import annotations

import copy
import json
import math
import threading
import time

from common.control import ControlSink, parse_descriptor
from slamtec_velocity import (ChassisBusy, CommandLeaseExpired,
                              SlamtecVelocityBackend, StaleVelocityCommand)

INPUT_TOPIC = "/actucore/navi/cmd"
WATCHDOG_MS = 500
MAX_OBS_AGE_MS = 1000
CANCEL_AFTER_MS = 1000
FOLLOW_KEYS = ("wz_limit",)
SPEED_PRESETS = {"slow": 0.15, "normal": 0.30, "fast": 0.50}
FOOTPRINT_HALF_WIDTH_M = 0.275  # Measured 0.55 m across, arms at rest.
TOGGLES = {"dry_run": True, "rotate_only": False, "require_standing": False}
CONFIG_KEYS = (*TOGGLES, "speed_preset", "min_vx",
               "linear_acceleration", "angular_acceleration", *FOLLOW_KEYS)


class _PostureRefused(ValueError):
    pass


class _LatestControlInbox:
    """Short receiver lock, independent of the lock held during SDK requests.

    Keep one latest motion and one priority zero command. The control worker
    consumes them; the bridge reader never waits for a vendor call.
    """

    def __init__(self):
        self._lock = threading.Lock()
        self._active = False
        self._generation = 0
        self._motion = self._zero = None
        self._received = self._coalesced = self._cleared = self._processed = 0
        self._last_received_at = None
        self._last_gap_ms = self._last_wait_ms = None

    def activate(self, generation, active):
        with self._lock:
            self._cleared += int(self._motion is not None) + int(self._zero is not None)
            self._motion = self._zero = None
            self._generation, self._active = generation, active
            self._last_received_at = None
            self._last_gap_ms = None

    def put(self, message, generation, received_at=None):
        if received_at is None:
            received_at = (time.monotonic(), int(time.time() * 1000))
        try:
            payload = json.loads(message.data)
            values = payload.get("values") if isinstance(payload, dict) else None
            zero = (isinstance(values, list) and len(values) == 6 and
                    all(isinstance(v, (int, float)) and not isinstance(v, bool) and v == 0
                        for v in values))
        except (ValueError, TypeError, AttributeError):
            zero = False  # The consumer still validates/counts malformed input.
        with self._lock:
            if not self._active or generation is not None and generation != self._generation:
                return False
            now = received_at[0]
            self._last_gap_ms = (round((now - self._last_received_at) * 1000, 1)
                                 if self._last_received_at is not None else None)
            self._last_received_at = now
            self._received += 1
            entry = (message, self._generation, *received_at)
            if zero:
                self._coalesced += int(self._motion is not None) + int(self._zero is not None)
                self._motion, self._zero = None, entry
            else:
                self._coalesced += int(self._motion is not None)
                self._motion = entry
            return True

    def take(self, allow_motion=True, *, newer_than=None):
        with self._lock:
            if not self._active:
                return None
            if self._zero is not None:
                entry, self._zero = self._zero, None
            elif allow_motion:
                if newer_than is not None and self._motion is not None:
                    # A confirmed stop may have blocked long enough for a new
                    # frame to supersede the selected motion. Compare and pop
                    # atomically; leave other streams/sequences for the sink's
                    # normal arbitration. A pending zero always takes priority.
                    try:
                        candidate = json.loads(self._motion[0].data)
                        source = newer_than.get("source")
                        priority = candidate.get("priority", 0)
                        seq, previous_seq = candidate.get("seq"), newer_than.get("seq")
                        if not (isinstance(source, str) and source and
                                candidate.get("source") == source and
                                isinstance(priority, (int, float)) and not isinstance(priority, bool) and
                                priority == newer_than.get("priority", 0) and
                                isinstance(seq, int) and not isinstance(seq, bool) and
                                isinstance(previous_seq, int) and not isinstance(previous_seq, bool) and
                                seq > previous_seq):
                            return None
                    except (ValueError, TypeError, AttributeError):
                        return None
                entry, self._motion = self._motion, None
            else:
                return None
            if entry is None:
                return None
            self._processed += 1
            self._last_wait_ms = round((time.monotonic() - entry[2]) * 1000, 1)
            return entry[0], entry[1], entry[2:]

    def info(self):
        with self._lock:
            return {"active": self._active, "received": self._received,
                    "processed": self._processed, "coalesced": self._coalesced,
                    "cleared": self._cleared,
                    "pending": int(self._motion is not None) + int(self._zero is not None),
                    "last_receive_gap_ms": self._last_gap_ms, "last_wait_ms": self._last_wait_ms}

    def has_priority_zero(self):
        with self._lock:
            return self._active and self._zero is not None


def _validated_min_vx(value, max_vx):
    message = f"loco_servo.min_vx must be finite and between 0 and max_vx ({max_vx:g})"
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(message)
    try:
        valid = math.isfinite(value) and 0 <= value <= max_vx
    except OverflowError:
        valid = False
    if not valid:
        raise ValueError(message)
    return float(value)


def _validated_wz_limit(value, max_wz):
    message = f"loco_servo.wz_limit must be finite and in (0, max_wz ({max_wz:g})]"
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(message)
    try:
        valid = math.isfinite(value) and 0 < value <= max_wz
    except OverflowError:
        valid = False
    if not valid:
        raise ValueError(message)
    return float(value)


def _validated_speed_preset(value, max_vx):
    choices = ", ".join(SPEED_PRESETS)
    if not isinstance(value, str) or value not in SPEED_PRESETS:
        raise ValueError(f"loco_servo.speed_preset must be one of: {choices}")
    # A deployment may set a lower native ceiling. Presets can never widen it;
    # this keeps older/custom max_vx configurations safe and truthful.
    return value, min(SPEED_PRESETS[value], max_vx)


def _step_limits(hz, linear_acceleration, angular_acceleration):
    for name, value in (("expected_hz", hz), ("linear_acceleration", linear_acceleration),
                        ("angular_acceleration", angular_acceleration)):
        try:
            valid = (not isinstance(value, bool) and isinstance(value, (int, float)) and
                     math.isfinite(value) and value > 0)
        except OverflowError:
            valid = False
        if not valid:
            raise ValueError(f"loco_servo.{name} must be a positive finite number")
    linear_step, angular_step = linear_acceleration / hz, angular_acceleration / hz
    if not all(math.isfinite(value) and value > 0 for value in (linear_step, angular_step)):
        raise ValueError("loco_servo per-frame velocity steps must be positive and finite")
    # The middle axes are pinned at zero by their hard bounds; the descriptor
    # still requires a positive step limit for every dimension.
    return [linear_step, 1e-6, 1e-6, 1e-6, 1e-6, angular_step]


def build_descriptor(hz=10.0, max_vx=0.3, max_wz=1.0, min_vx=0.0,
                     linear_acceleration=0.3, angular_acceleration=2.0,
                     response=None):
    min_vx = _validated_min_vx(min_vx, max_vx)
    descriptor = {
        "control_interface": "motus.control/1", "mode": "twist", "dof": 6,
        "joint_names": ["vx", "vy", "vz", "wx", "wy", "wz"],
        "frame": "base_link",
        "units": {"linear": "m/s", "angular": "rad/s", "time": "s"},
        "limits": {"lower": [-max_vx, 0, 0, 0, 0, -max_wz],
                   "upper": [max_vx, 0, 0, 0, 0, max_wz],
                   "min_magnitude": [min_vx, 0, 0, 0, 0, 0],
                   "max_delta_per_step": _step_limits(hz, linear_acceleration, angular_acceleration),
                   "zero_immediate": True},
        "groups": [{"name": "base", "offset": 0, "count": 6,
                    "unit": "m/s", "resource": "base"}],
        "rate": {"max_hz": 20, "expected_hz": hz,
                 "watchdog_ms": WATCHDOG_MS, "max_obs_age_ms": MAX_OBS_AGE_MS,
                 "cancel_after_ms": CANCEL_AFTER_MS},
        "force_torque": None,
        # Static outside width measured on the physical Tianyi in its normal
        # navigation posture. Navi adds its own dynamic clearance margin; this
        # declaration must remain the robot envelope rather than baking that
        # policy-specific margin in twice.
        "footprint": {"half_width": FOOTPRINT_HALF_WIDTH_M,
                      "source": "measured"},
    }
    if response is not None:
        descriptor["response"] = copy.deepcopy(response)
    return descriptor


class LocoServoPlugin:
    def __init__(self, config, namespace, ros2, conflict=None):
        self._ros2 = ros2
        self._backend_config = dict(config)
        self._conflict = conflict or (lambda: "")
        self._lock = threading.RLock()
        self._stop_lock = threading.Lock()
        self._hz = self._positive(config, "expected_hz", 10.0)
        if self._hz > 20:
            raise ValueError("loco_servo.expected_hz must be <= 20")
        self._accel = (self._positive(config, "linear_acceleration", 0.3),
                       self._positive(config, "angular_acceleration", 2.0))
        self._max_vx = self._positive(config, "max_vx", 0.5)
        # The native worker enforces this independently. Pass the normalized
        # ceiling even when the deployment omitted max_vx, so fast mode cannot
        # be accepted by Python and then rejected by the SDK boundary.
        self._backend_config["max_vx"] = self._max_vx
        self._speed_preset, self._vx_limit = _validated_speed_preset(
            config.get("speed_preset", "normal"), self._max_vx)
        self._descriptor = build_descriptor(
            self._hz, self._vx_limit,
            self._positive(config, "max_wz", 1.0), config.get("min_vx", 0.0),
            *self._accel, response=config.get("response"))
        self._min_vx = self._descriptor["limits"]["min_magnitude"][0]
        self._parsed_descriptor = parse_descriptor(self._descriptor)
        # Apply the configured yaw cap before the shared clamp; the descriptor
        # navi reads still declares the chassis's original operating limits.
        self._max_wz = self._descriptor["limits"]["upper"][5]
        self._wz_limit = _validated_wz_limit(config.get("wz_limit", self._max_wz), self._max_wz)
        self._settings = dict(TOGGLES)
        for key in TOGGLES:
            if key in config:
                if not isinstance(config[key], bool):
                    raise ValueError(f"loco_servo.{key} must be boolean")
                self._settings[key] = config[key]
        self._standing_topic = str(config.get("standing_topic", "")).strip()
        self._standing_timeout = self._positive(config, "standing_timeout", 1.0)
        self._standing = None
        self._standing_at = None
        self._nodes = []
        self._controller_thread = None
        self._controller_stop = threading.Event()
        self._controller_wake = threading.Event()
        self._controller_failed = False
        self._closing = False
        self._backend = None
        self._sink = None
        self._running = self._paused = False
        self._yielded = False
        self._generation = 0
        self._input_topic = ""
        self._barrier_ms = 0
        self._last_output = [0.0] * 6
        self._last_output_at = None
        self._last_valid_at = None
        self._command_deadline_ms = 0
        self._command_cancel_deadline_ms = 0
        self._lease_expiry_ms = 0
        self._cancel_deadline_ms = 0
        self._soft_held = False
        self._last_preserved_action = False
        self._command_requested_values = [0.0] * 6
        self._zero_pending = False
        self._last = None
        self._last_timing = None
        self._counts = {key: 0 for key in
                        ("received", "accepted", "published", "dry_run_commands", "soft_holds",
                         "chassis_busy", "rejected", "dropped", "holds", "publish_errors",
                         "refreshed_after_stop")}
        self._inbox = _LatestControlInbox()

    @staticmethod
    def _positive(config, key, default):
        value = config.get(key, default)
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ValueError(f"loco_servo.{key} must be a positive finite number")
        try:
            valid = math.isfinite(value) and value > 0
        except OverflowError:
            valid = False
        if not valid:
            raise ValueError(f"loco_servo.{key} must be a positive finite number")
        return float(value)

    def get_tool(self):
        tool = {
            "name": "loco_servo", "type": "actuator", "multiInstance": False,
            "description": "天轶底盘连续速度控制：接收 navi 的速度流，通过思岚 SDK 执行 vx/wz；默认空跑。",
            "inputSchema": {
                "type": "object", "required": ["action"],
                "properties": {
                    "action": {"type": "string", "enum":
                               ["start", "stop", "pause", "resume", "info", "config"]},
                    "input_topic": {"type": "string", "default": INPUT_TOPIC}},
                "x-resource": ["base"], "x-is-dangerous": True,
                "x-hooks": {"on_interrupt_motion": {"action": "pause"},
                            "on_interrupt_all": {"action": "pause"}},
                "x-action-params": {
                    "start": {"params": ["input_topic", *CONFIG_KEYS],
                              "description": "应用本次填写的配置后订阅速度话题；省略的配置保持当前值，准备接收新的导航指令"},
                    "stop": {"params": [], "description": "停止底盘并释放速度订阅"},
                    "pause": {"params": [], "description": "停止底盘，保持订阅"},
                    "resume": {"params": [], "description": "恢复接收新的速度指令"},
                    "info": {"params": [], "description": "查看状态、动作空间及执行诊断"},
                    "config": {"params": list(CONFIG_KEYS),
                               "description": "更新空跑、执行限制及标称加速度；设置变化前先停车"}}},
            "topic_in": [{"topic": INPUT_TOPIC, "format": "control/velocity",
                          "desc": "motus.control/1 twist [vx,vy,vz,wx,wy,wz]"}],
            "configSchema": {"type": "object", "properties": {
                "speed_preset": {"type": "string", "default": self._speed_preset,
                                  "enum": list(SPEED_PRESETS),
                                  "description": "直行速度档位：slow=0.15、normal=0.30、fast=0.50 m/s。只改变 vx 上限，不放大转向；切档会先停车，随后需重启 navi 读取新上限。"},
                "linear_acceleration": {"type": "number", "default": self._accel[0],
                                        "exclusiveMinimum": 0,
                                        "description": "直行标称加速度（m/s²），默认 0.3；按 expected_hz 换算为每帧速度步长，由共享 ControlSink 钳制。显式零速度立即停车。"},
                "angular_acceleration": {"type": "number", "default": self._accel[1],
                                         "exclusiveMinimum": 0,
                                         "description": "转向标称加速度（rad/s²），默认 2.0；按 expected_hz 换算为每帧速度步长，由共享 ControlSink 钳制。显式零速度立即停车。"},
                "wz_limit": {"type": "number", "default": self._wz_limit, "exclusiveMinimum": 0,
                             "maximum": self._max_wz,
                             "description": f"转向限幅（rad/s），默认 {self._max_wz:g}（不额外限制）；超出的转向指令截到该值执行，不拒收。不改变向 navi 声明的速度上限。"},
                "min_vx": {"type": "number", "default": self._min_vx, "minimum": 0.0,
                           "maximum": self._descriptor["limits"]["upper"][0],
                           "description": "实测最小稳定直行速度（m/s），仅向规划器声明，不在执行端强行抬高速度；0 不声明。修改后需重新启动 navi 读取。"},
                "dry_run": {"type": "boolean", "default": self._settings["dry_run"],
                            "description": "空跑：照常接收、检查、计数，但不创建 SDK 运动动作。第一次上机先保持开启，检查它想发什么。"},
                "rotate_only": {"type": "boolean", "default": self._settings["rotate_only"],
                                "description": "只转不走：vx / vy 在到达底盘前被清零，保留 vyaw。"},
                "require_standing": {"type": "boolean", "default": self._settings["require_standing"],
                                     "description": "要求有效的站立反馈才执行指令。天轶需配置 standing_topic（域0 Bool）；开启后缺失、过期或 false 均拒绝运动。"}}},
        }
        # Agent Core constructs action-specific functions from x-action-params;
        # configSchema alone does not advertise a dispatchable config action.
        # Copy the constraints/descriptions, but never copy form defaults into
        # the action schema. The canvas saves the gear form first and sends it
        # as ``action: config`` before ``start``. A stale action-schema default
        # (notably dry_run=true from deployment YAML) would otherwise be filled
        # into start and overwrite the value the user just saved in the gear.
        action_settings = copy.deepcopy(tool["configSchema"]["properties"])
        for field in action_settings.values():
            field.pop("default", None)
        tool["inputSchema"]["properties"].update(action_settings)
        return tool

    def start(self):
        """Bundle lazy-start hook, intentionally inert."""

    def stop(self):
        # Never join under _lock: the worker may already be waiting to acquire
        # it. _closing prevents start from replacing resources during the join.
        with self._stop_lock:
            with self._lock:
                self._inbox.activate(self._generation, False)
                self._paused = True
                self._hold()
                if self._zero_pending:
                    return {"state": "error", "error": "SDK stop unconfirmed; card remains paused"}
                self._closing = True
                self._running = False
                self._generation += 1
                self._controller_stop.set()
                self._controller_wake.set()
                worker = self._controller_thread
            if worker is not None:
                if worker is threading.current_thread():
                    return {"state": "error", "error": "control worker cannot join itself; retry stop externally"}
                worker.join(timeout=1.0)
                if worker.is_alive():
                    return {"state": "error", "error": "control worker has not stopped; restart blocked"}
            with self._lock:
                self._controller_thread = None
                self._sink = None
                self._close()
                self._yielded = False
                self._paused = False
                self._closing = False
                return {"state": "idle"}

    def dispatch(self, action, args):
        if action == "stop":
            return self.stop()  # In particular, do not retain an outer RLock.
        with self._lock:
            if action == "info":
                return self._info()
            if action == "config":
                return self._configure(args)
            if action == "start":
                return self._start(args)
            if action in ("pause", "resume"):
                if not self._running:
                    return {"state": "idle"}
                if action == "resume":
                    if self._controller_failed or (
                            self._controller_thread is not None and not self._controller_thread.is_alive()):
                        return {"state": "error", "error": "control worker stopped; stop/start the card"}
                    problem = self._conflict()
                    if problem:
                        return {"state": "error", "error": problem}
                    self._yielded = False
                self._inbox.activate(self._generation, False)
                self._paused = action == "pause"
                self._hold()
                if self._zero_pending:
                    self._paused = True
                    return {"state": "error", "error": "SDK stop unconfirmed; card remains paused"}
                if action == "resume" and not self._settings["dry_run"]:
                    try:
                        self._backend.connect()
                    except Exception as exc:
                        self._paused = True
                        return {"state": "error", "error": str(exc)}
                self._barrier_ms = int(time.time() * 1000)
                self._inbox.activate(self._generation, not self._paused)
                self._controller_wake.set()
                return self._info()
        return None

    def _start(self, args):
        if self._closing:
            return {"state": "error", "error": "previous control worker is stopping; retry stop before restart"}
        topic = args.get("input_topic")
        if not topic:
            topics = args.get("input_topics")
            topic = topics[0] if isinstance(topics, list) and topics else INPUT_TOPIC
        if not isinstance(topic, str) or not topic.strip().startswith("/"):
            return {"state": "error", "error": "input_topic must be an absolute ROS topic"}
        if self._running:
            if self._input_topic != topic.strip():
                return {"state": "error", "error": "stop before changing input_topic"}
            if any(key in args for key in CONFIG_KEYS):
                result = self._configure(args)
                if not result.get("ok"):
                    return result
            return self._info()
        problem = self._conflict()
        if problem:
            return {"state": "error", "error": problem}
        # Some canvas clients send configuration on start rather than in a
        # preceding config call. Honour explicit values (including false)
        # before any SDK connection or subscription is opened.
        if any(key in args for key in CONFIG_KEYS):
            result = self._configure(args)
            if not result.get("ok"):
                return result
        self._generation += 1
        generation = self._generation
        self._yielded = False
        self._input_topic = topic.strip()
        self._barrier_ms = int(time.time() * 1000)
        self._sink = ControlSink(self._parsed_descriptor, self._apply,
                                 on_watchdog=self._soft_hold, on_abort=self._soft_hold)
        self._sink.rebase([0.0] * 6)
        try:
            self._open(self._input_topic, generation)
        except Exception as exc:
            self._close()
            self._sink = None
            return {"state": "error", "error": f"loco_servo start failed: {exc}"}
        self._running = True
        self._paused = False
        self._inbox.activate(self._generation, True)
        try:
            self._start_controller(generation)
        except Exception as exc:
            self._running = False
            self._inbox.activate(self._generation, False)
            self._controller_thread = None
            self._close()
            self._sink = None
            return {"state": "error", "error": f"control worker start failed: {exc}"}
        return self._info()

    def _start_controller(self, generation):
        stop, wake = threading.Event(), threading.Event()
        self._controller_failed = False
        self._controller_stop, self._controller_wake = stop, wake
        worker = threading.Thread(target=self._control_loop,
                                  args=(generation, stop, wake),
                                  name=f"tianyi-loco-control-{generation}", daemon=True)
        self._controller_thread = worker
        worker.start()

    def _control_loop(self, generation, stop, wake):
        try:
            while not stop.is_set():
                # Clear before processing: an arrival during a blocked SDK
                # call stays signalled and is consumed on the very next pass.
                wake.clear()
                with self._lock:
                    if not self._running or generation != self._generation:
                        return
                self._tick(generation)
                if stop.is_set():
                    return
                with self._lock:
                    delay = None if self._paused and not self._zero_pending else 0.02
                    # Active watchdog/standing checks remain bounded; a fully
                    # paused worker sleeps until resume or stop wakes it.
                    if not self._paused and self._inbox.info()["pending"]:
                        due_in = (self._last_output_at + 1 / 20 - time.monotonic()
                                  if self._last_output_at is not None else 0)
                        delay = max(0, min(delay, due_in))
                wake.wait(delay)
        except Exception as exc:
            with self._lock:
                if generation == self._generation and self._running:
                    self._controller_failed = True
                    self._paused = True
                    self._inbox.activate(generation, False)
                    self._hold()
                    self._record("ERROR", f"control worker failed: {exc}")

    def _open(self, topic, generation):
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
        from std_msgs.msg import String, Bool

        # Node identity/context is what bridge_integration uses for domain 42.
        core = Node("tianyi2_loco_servo_in", context=self._ros2.ctx_core)
        self._nodes.append((core, self._ros2.executor_core))
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE)
        core.create_subscription(String, topic,
                                 lambda msg: self._on_message(msg, generation), qos)
        self._ros2.executor_core.add_node(core)
        self._backend = SlamtecVelocityBackend(self._backend_config)
        if not self._settings["dry_run"]:
            self._backend.connect()  # Read-only: no action or heartbeat until a valid nonzero command.
        if self._standing_topic:
            body = Node("tianyi2_loco_servo_standing", context=self._ros2.ctx_tianyi)
            self._nodes.append((body, self._ros2.executor_tianyi))
            body.create_subscription(
                Bool, self._standing_topic,
                lambda msg: self._on_standing(msg, generation), qos)
            self._ros2.executor_tianyi.add_node(body)

    def _close(self):
        self._inbox.activate(self._generation, False)
        self._controller_stop.set()
        self._controller_wake.set()
        if self._backend is not None:
            self._backend.close()
            self._backend = None
        for node, executor in reversed(self._nodes):
            try:
                executor.remove_node(node)
            except Exception:
                pass
            try:
                node.destroy_node()
            except Exception as exc:
                self._record("ERROR", f"node teardown failed: {exc}")
        self._nodes.clear()
        self._standing = self._standing_at = None

    def _on_standing(self, message, generation):
        with self._lock:
            if generation != self._generation:
                return
            self._standing = message.data if isinstance(message.data, bool) else None
            self._standing_at = time.monotonic()
            self._controller_wake.set()

    def _standing_problem(self):
        if not self._settings["require_standing"]:
            return ""
        if not self._standing_topic:
            return "require_standing enabled but no verified standing_topic configured"
        if self._standing_at is None or time.monotonic() - self._standing_at > self._standing_timeout:
            return "standing feedback missing or stale"
        return "" if self._standing is True else "standing feedback is not true"

    def _record(self, verdict, reason="", **extra):
        self._last = {"verdict": verdict, "reason": reason, **extra}

    def _on_message(self, message, generation=None, received_at=None):
        # No plugin/SDK lock here: waiting on it would block the socket reader
        # and accumulate already-aging commands in the Unix socket buffer.
        if self._inbox.put(message, generation, received_at):
            self._controller_wake.set()

    def _process_message(self, message, generation=None, received_at=None):
        with self._lock:
            if not self._running or self._paused or (
                    generation is not None and generation != self._generation):
                return
            self._counts["received"] += 1
            try:
                command = json.loads(message.data)
                problem = self._validate(command)
                if problem:
                    self._counts["rejected"] += 1
                    self._record("REJECTED", problem)
                    return
                if command["stamp_ms"] <= self._barrier_ms:
                    self._counts["dropped"] += 1
                    self._record("DROPPED", "command predates start/resume/config")
                    return
                # Limit SDK apply STARTS, not time since its last reply. At
                # 10Hz a 70ms SDK call leaves only 30ms after acknowledgement;
                # treating that as a >20Hz stream drops every other command.
                if any(command["values"]) and self._last_output_at is not None and (
                        time.monotonic() - self._last_output_at < 1 / 20):
                    self._counts["dropped"] += 1
                    self._record("DROPPED", "command rate exceeds 20 Hz")
                    return
                started = time.monotonic()
                wall_ms = int(time.time() * 1000)
                received_mono = received_at[0] if received_at is not None else started
                # TTL and observation age are admission checks at the sink
                # (stale input is dropped without stopping current motion, as
                # on R1). Once admitted, motion follows only the receive
                # watchdog. An obs_stamp_ms + 500 execution deadline left a
                # 400ms-old detection <100ms of lease at 10Hz: every frame
                # soft-zeroed and the shared clamp restarted from 0, capping
                # output at one step (0.03 m/s, 0.2 rad/s). IPC/SDK waits
                # spend this original receive budget; nothing renews it.
                receive_remaining_ms = math.floor(
                    WATCHDOG_MS - max(0.0, started - received_mono) * 1000)
                self._command_deadline_ms = wall_ms + receive_remaining_ms
                self._command_cancel_deadline_ms = wall_ms + math.floor(
                    CANCEL_AFTER_MS - max(0.0, started - received_mono) * 1000)
                self._last_timing = {
                    "seq": command.get("seq"), "ttl_ms": command.get("ttl_ms"),
                    "execution_deadline_basis": "receive_watchdog",
                    "receive_watchdog_ms": WATCHDOG_MS,
                    "cancel_after_ms": CANCEL_AFTER_MS,
                    "receive_age_ms": (received_at[1] - command["stamp_ms"]
                                       if received_at is not None else None),
                    "local_wait_ms": (round((started - received_at[0]) * 1000, 1)
                                      if received_at is not None else None),
                    "command_age_ms": wall_ms - command["stamp_ms"],
                    "observation_age_ms": wall_ms - command.get("obs_stamp_ms", command["stamp_ms"]),
                    "remaining_before_apply_ms": self._command_deadline_ms - wall_ms,
                    "cancel_remaining_before_apply_ms": self._command_cancel_deadline_ms - wall_ms,
                }
                self._command_requested_values = list(command["values"])
                # Shape the turn of a non-zero command. _command_requested_values
                # keeps navi's original values, so a shaped all-zero output
                # zeroes velocity but keeps the action instead of cancelling.
                shaped, turn = self._shape_turn(command)
                self._last_timing["turn"] = turn
                if shaped != command["values"]:
                    command = dict(command, values=shaped)
                # The native deadline independently zeroes the old command.
                # Synchronize only the physical baseline, never seq/holder or
                # accepted deadlines, before the shared sink clamps a new one.
                if self._lease_expiry_ms and wall_ms >= self._lease_expiry_ms:
                    self._sink.rebase([0.0] * 6)
                try:
                    outcome = self._sink.submit(command)
                finally:
                    self._last_timing.update(
                        apply_round_trip_ms=round((time.monotonic() - started) * 1000, 1),
                        remaining_after_apply_ms=self._command_deadline_ms - int(time.time() * 1000))
                if outcome.applied:
                    # rotate_only changes what reached the chassis. The next
                    # shared clamp must start at that actual output.
                    self._sink.rebase(self._last_output)
                    self._counts["accepted"] += 1
                    if any(self._last_output) or self._last_preserved_action:
                        self._last_valid_at = received_mono
                        self._lease_expiry_ms = self._command_deadline_ms
                        self._cancel_deadline_ms = self._command_cancel_deadline_ms
                        self._soft_held = False
                    else:
                        # No action existed to preserve (for example an idle
                        # zero).  Do not arm a cancellation timer for a vendor
                        # action that was never created.
                        self._last_valid_at = None
                        self._lease_expiry_ms = self._cancel_deadline_ms = 0
                        self._soft_held = False
                    self._record(outcome.verdict.name, outcome.reason,
                                 requested=list(command["values"]),
                                 output=list(self._last_output),
                                 dry_run=self._settings["dry_run"])
                else:
                    self._counts["dropped" if outcome.verdict.name == "DROPPED" else "rejected"] += 1
                    self._record(outcome.verdict.name, outcome.reason)
            except _PostureRefused as exc:
                self._counts["rejected"] += 1
                self._record("REFUSED", str(exc))
            except (StaleVelocityCommand, CommandLeaseExpired) as exc:
                self._counts["dropped"] += 1
                if (isinstance(exc, StaleVelocityCommand) and
                        getattr(exc, "zero_velocity_confirmed", False) is True):
                    # A fresh candidate can expire inside its SDK call. Native
                    # then acknowledges a real zero even when the previous
                    # accepted motion deadline is still in the future. Rebase
                    # to that actual output, without adopting the rejected
                    # candidate's receive/cancel deadlines or issuing a STOP.
                    self._last_output = [0.0] * 6
                    self._last_output_at = None
                    self._last_preserved_action = False
                    self._lease_expiry_ms = 0
                    self._soft_held = True
                    self._sink.rebase(self._last_output)
                # Rejection itself issues no stop and adopts no new lease.
                # The ordinary tick retains the accepted cancellation deadline,
                # independently of this rejected packet.
                self._record("DROPPED", str(exc))
            except ChassisBusy as exc:
                # Native BUSY is acknowledged before this worker creates an
                # action, starts a heartbeat or writes velocity. Do not fault,
                # stop or preempt the foreign owner; consume only this frame.
                # A later fresh navi frame retries through the same worker.
                self._counts["chassis_busy"] += 1
                self._counts["holds"] += 1
                self._last_output = [0.0] * 6
                self._last_output_at = None
                self._last_valid_at = None
                self._lease_expiry_ms = self._cancel_deadline_ms = 0
                self._soft_held = False
                self._last_preserved_action = False
                self._sink.rebase(self._last_output)
                self._record("HOLD", f"{exc}; waiting for chassis to become idle")
            except (ValueError, TypeError, KeyError, OverflowError) as exc:
                self._counts["rejected"] += 1
                self._record("REJECTED", str(exc))
            except Exception as exc:
                self._counts["publish_errors"] += 1
                self._paused = True
                self._hold()
                self._record("ERROR", str(exc))

    def _validate(self, command):
        if not isinstance(command, dict):
            return "command must be an object"
        if command.get("frame", "base_link") not in ("base_link", "body"):
            return "only body/base_link velocity commands are supported"
        # The shared sink performs schema, shape, seq, TTL and source arbitration.
        # Guard NaN/Inf before its comparisons, and reject unsupported axes even
        # in rotate_only mode: silently accepting a strafe is misleading.
        values = command.get("values")
        if not isinstance(values, list) or len(values) != 6:
            return "values must contain six numbers"
        for i, value in enumerate(values):
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
                return "values must contain finite numbers"
            if not self._descriptor["limits"]["lower"][i] <= value <= self._descriptor["limits"]["upper"][i]:
                return f"axis {i} is unsupported or exceeds configured velocity limits"
        now = int(time.time() * 1000)
        for key in ("stamp_ms", "ttl_ms", "priority", "obs_stamp_ms"):
            value = command.get(key, 0)
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
                return f"{key} must be finite"
        if command.get("stamp_ms", 0) > now + 100:
            return "command timestamp is in the future"
        if command.get("obs_stamp_ms", 0) > now + 100:
            return "observation timestamp is in the future"
        return ""

    def _native_expiry_update(self):
        return (not self._settings["dry_run"] and
                getattr(self._backend, "supports_atomic_expiry_update", False) is True)

    def _native_staged_stop(self):
        return (not self._settings["dry_run"] and
                getattr(self._backend, "supports_staged_stop", False) is True)

    def _shape_turn(self, command):
        """Use navi's yaw request directly, subject only to the configured cap."""
        values = list(command["values"])
        requested = values[5]
        turn = {"requested_wz": requested, "wz_limit": self._wz_limit}
        if not any(values):
            turn["shaped_wz"] = requested
            return values, turn
        values[5] = max(-self._wz_limit, min(self._wz_limit, values[5]))
        turn["shaped_wz"] = round(values[5], 4)
        return values, turn

    def _apply(self, values, _gripper=None):
        # Run after sink freshness/shape/arbitration checks: invalid input must
        # neither renew motion nor issue a stop against an otherwise live lease.
        requested = list(self._command_requested_values)
        if self._settings["rotate_only"]:
            requested[0] = requested[1] = 0.0
        motion_requested = any(requested)
        now_ms = time.time() * 1000
        # A zero in the velocity stream is a velocity update, not a lifecycle
        # command.  Tianyi's vendor STOP releases the Slamware action, so keep
        # an action we already own until the independent cancel_after deadline.
        # An idle zero must
        # remain pure no-op: it must neither create nor cancel an action.
        if (not motion_requested and self._cancel_deadline_ms
                and now_ms >= self._cancel_deadline_ms):
            # The action lease has already ended.  A late zero must not make us
            # forget a still-owned SDK action without cancelling it.
            self._hold()
            if self._zero_pending:
                # STOP is a lifecycle boundary, not a best-effort velocity
                # write.  Keep the retry latched and fault the card if the SDK
                # did not confirm it; otherwise the code below would clear the
                # latch while the previous action could still be moving.
                raise RuntimeError("SDK stop unconfirmed")
        preserve_zero = bool(
            not motion_requested and self._cancel_deadline_ms
            and now_ms < self._cancel_deadline_ms)
        if motion_requested and self._command_deadline_ms <= now_ms:
            raise CommandLeaseExpired("receive watchdog expired before apply")
        if motion_requested:
            problem = self._standing_problem()
            if problem:
                self._hold()
                raise _PostureRefused(problem)
        output = list(values)
        if self._settings["rotate_only"]:
            output[0] = output[1] = 0.0
        motion_expired = bool(motion_requested and self._lease_expiry_ms and
                              now_ms >= self._lease_expiry_ms)
        if motion_expired and not self._native_staged_stop() and not self._native_expiry_update():
            # Compatibility with old workers that cancel on motion expiry.
            # This is deliberately after the sink's admission checks: stale or
            # outranked input must never trigger a stop or renew any deadline.
            self._hold()
            if self._zero_pending:
                raise RuntimeError("SDK stop unconfirmed")
            if self._inbox.has_priority_zero():
                raise CommandLeaseExpired("priority zero arrived during legacy action stop")
            if self._command_deadline_ms <= time.time() * 1000:
                raise CommandLeaseExpired("command expired during legacy action stop")
        replace_expired = bool(motion_expired and not self._native_staged_stop() and
                               self._native_expiry_update() and
                               self._lease_expiry_ms and
                               now_ms >= self._lease_expiry_ms)
        now = time.monotonic()
        # ControlSink is the sole step limiter. Its values are already clamped
        # by the advertised fixed per-frame contract, including immediate zero.
        preserve_action = bool(
            preserve_zero or motion_requested and not any(output))
        if self._settings["dry_run"]:
            self._counts["dry_run_commands"] += 1
        elif any(output) or preserve_action:
            self._publish(output, replace_expired=replace_expired,
                          preserve_action=preserve_action)
        self._last_preserved_action = preserve_action
        self._last_output = output
        self._last_output_at = now
        self._zero_pending = False

    def _publish(self, values, *, replace_expired=False, preserve_action=False):
        if self._backend is None:
            raise RuntimeError("Slamware SDK backend unavailable")
        try:
            if any(values) or preserve_action:
                options = {"cancel_deadline_ms": self._command_cancel_deadline_ms}
                if replace_expired:
                    options["replace_expired"] = True
                if preserve_action:
                    options["preserve_action"] = True
                self._backend.write(values, self._command_deadline_ms, **options)
            else:
                self._backend.stop()
        except (ChassisBusy, CommandLeaseExpired):
            raise
        except Exception as exc:
            raise RuntimeError(f"Slamware SDK write failed: {exc}") from exc
        self._counts["published"] += 1

    def _soft_hold(self, *_args):
        """Zero the live action once; retain its original cancellation deadline."""
        if self._yielded or self._soft_held or not self._cancel_deadline_ms:
            return
        if time.time() * 1000 >= self._cancel_deadline_ms:
            self._hold()
            self._record("HOLD", f"no valid command for {CANCEL_AFTER_MS} ms; action cancelled")
            return
        if self._backend is not None and not self._settings["dry_run"]:
            try:
                self._backend.soft_stop(self._cancel_deadline_ms)
                self._counts["published"] += 1
            except Exception as exc:
                self._counts["publish_errors"] += 1
                self._paused = True
                self._hold()
                self._record("ERROR", f"soft zero velocity failed: {exc}")
                return
        self._counts["soft_holds"] += 1
        self._last_output = [0.0] * 6
        self._last_output_at = None
        self._last_preserved_action = False
        self._lease_expiry_ms = 0
        self._soft_held = True
        if self._sink is not None:
            self._sink.rebase(self._last_output)
        self._record("SOFT_HOLD", "receive watchdog expired; action retained")

    def _hold(self, *_args):
        self._counts["holds"] += 1
        if self._yielded:
            self._zero_pending = False
        elif self._backend is not None and not self._settings["dry_run"]:
            try:
                self._publish([0.0] * 6)
                self._zero_pending = False
            except Exception as exc:
                self._zero_pending = True
                self._counts["publish_errors"] += 1
                self._record("ERROR", f"zero velocity publish failed: {exc}")
        self._last_output = [0.0] * 6
        self._last_output_at = None
        self._last_valid_at = None
        self._lease_expiry_ms = 0
        self._cancel_deadline_ms = 0
        self._soft_held = False
        self._last_preserved_action = False
        if self._sink is not None and not self._zero_pending:
            self._sink.rebase(self._last_output)

    def _tick(self, generation=None):
        with self._lock:
            if not self._running or generation is not None and generation != self._generation:
                return
            if self._zero_pending:
                self._hold()
                if self._zero_pending:
                    self._paused = True
            if self._paused:
                self._inbox.activate(self._generation, False)
                return
            # A burst after transport jitter is not proof that Navi computed
            # commands above 20Hz. Keep its latest motion until the next apply
            # slot rather than throwing it away. Zero commands never wait.
            motion_due = self._last_output_at is None or (
                time.monotonic() - self._last_output_at >= 1 / 20)
            pending = self._inbox.take(allow_motion=motion_due)
            if pending is not None:
                self._process_message(*pending)
                if self._paused:
                    self._inbox.activate(self._generation, False)
                    return
            if self._standing_problem() and any(self._last_output):
                self._hold()
                self._record("REFUSED", self._standing_problem())
            if (self._cancel_deadline_ms and time.time() * 1000 >= self._cancel_deadline_ms or
                    self._last_valid_at is not None and time.monotonic() - self._last_valid_at >= CANCEL_AFTER_MS / 1000):
                self._hold()
                self._record("HOLD", f"no valid command for {CANCEL_AFTER_MS} ms; action cancelled")
            elif (self._lease_expiry_ms and time.time() * 1000 >= self._lease_expiry_ms or
                  self._last_valid_at is not None and time.monotonic() - self._last_valid_at >= WATCHDOG_MS / 1000):
                self._soft_hold()
            self._sink.tick()

    def _configure(self, args):
        updates = {key: args[key] for key in TOGGLES if key in args}
        if any(not isinstance(value, bool) for value in updates.values()):
            return {"state": "error", "error": "configuration toggles must be boolean"}
        try:
            acceleration = (self._positive(args, "linear_acceleration", self._accel[0]),
                            self._positive(args, "angular_acceleration", self._accel[1]))
            speed_preset, vx_limit = _validated_speed_preset(
                args.get("speed_preset", self._speed_preset), self._max_vx)
            min_vx = _validated_min_vx(args.get("min_vx", self._min_vx),
                                       vx_limit)
            wz_limit = _validated_wz_limit(args.get("wz_limit", self._wz_limit), self._max_wz)
            descriptor = copy.deepcopy(self._descriptor)
            descriptor["limits"]["lower"][0] = -vx_limit
            descriptor["limits"]["upper"][0] = vx_limit
            descriptor["limits"]["min_magnitude"][0] = min_vx
            descriptor["limits"]["max_delta_per_step"] = _step_limits(self._hz, *acceleration)
            parsed_descriptor = parse_descriptor(descriptor)
        except ValueError as exc:
            return {"state": "error", "error": str(exc)}
        # The turn cap only affects later commands: no stop required.
        # Legacy yaw_latency_gain settings are intentionally ignored; no
        # constructor, start or config path can re-enable compensation.
        self._wz_limit = wz_limit
        if (speed_preset != self._speed_preset or min_vx != self._min_vx or
                acceleration != self._accel or
                any(self._settings[key] != value for key, value in updates.items())):
            self._inbox.activate(self._generation, False)
            # Stop under the OLD mode, especially before enabling dry_run.
            if self._running:
                self._hold()
                if self._zero_pending:
                    self._paused = True
                    return {"state": "error", "error": "SDK stop unconfirmed; settings unchanged"}
                if not updates.get("dry_run", self._settings["dry_run"]):
                    try:
                        self._backend.connect()
                    except Exception as exc:
                        self._inbox.activate(self._generation, not self._paused and not self._yielded)
                        return {"state": "error", "error": str(exc), "dry_run": self._settings["dry_run"]}
            if self._sink is not None:
                self._sink.reconfigure(parsed_descriptor)
            self._settings.update(updates)
            self._speed_preset = speed_preset
            self._vx_limit = vx_limit
            self._accel = acceleration
            # Keep the existing sink and its source/sequence arbitration;
            # reconfigure only the declared limits and their shared clamp.
            self._min_vx = min_vx
            self._descriptor = descriptor
            self._parsed_descriptor = parsed_descriptor
            self._barrier_ms = int(time.time() * 1000)
            self._inbox.activate(self._generation,
                                 self._running and not self._paused and not self._yielded)
        return {"ok": True, **self._info()}

    def reserves_chassis(self):
        """Local stream reservation; this does not assert SDK ownership."""
        with self._lock:
            return (self._running and not self._paused and not self._yielded
                    and not self._closing)

    def pause_for_explicit_command(self, reason=""):
        with self._lock:
            if self._running:
                self._inbox.activate(self._generation, False)
                self._paused = True
                self._hold()
                if self._zero_pending:
                    return False
                # After the handover even zero commands could interrupt the
                # new owner. Stop/config/timers must cease all base writes.
                self._yielded = True
                self._record("PAUSED", reason)
            return True

    def _info(self):
        interface = copy.deepcopy(self._descriptor)
        interface.update({key: self._settings[key] for key in ("dry_run", "rotate_only")})
        return {
            "state": "stopping" if self._closing else "paused" if self._running and self._paused else "running" if self._running else "idle",
            "input": self._input_topic, "speed_preset": self._speed_preset,
            "vx_limit": self._vx_limit, "min_vx": self._min_vx, **self._settings,
            "linear_acceleration": self._accel[0], "angular_acceleration": self._accel[1],
            "wz_limit": self._wz_limit,
            "yielded_to_explicit_command": self._yielded,
            "chassis_reserved": self.reserves_chassis(),
            "control_interface": interface,
            "counts": dict(self._counts), "last": copy.deepcopy(self._last),
            "input_queue": self._inbox.info(),
            "dispatch": {"mode": "dedicated_worker", "watchdog_poll_ms": 20,
                         "step_limiter": "shared_control_sink", "step_basis": "per_command",
                         "execution_deadline_basis": "receive_watchdog",
                         "receive_watchdog_ms": WATCHDOG_MS,
                         "cancel_after_ms": CANCEL_AFTER_MS,
                         "staged_stop": self._native_staged_stop(),
                         "max_apply_hz": 20, "worker_failed": self._controller_failed,
                         "expiry_update": ("native_staged" if self._native_staged_stop() else
                                           "native_atomic" if self._native_expiry_update() else "stop_then_set"),
                         "worker_alive": bool(
                             self._controller_thread and self._controller_thread.is_alive())},
            "last_command_timing": copy.deepcopy(self._last_timing),
            "output": list(self._last_output), "output_topic": None,
            "backend": self._backend.info() if self._backend is not None else
                       {"type": "slamware_sdk", "connected": False, "state": "disconnected"},
            "standing_topic": self._standing_topic,
            "posture_problem": self._standing_problem(),
            "acceleration_limits": {"linear_m_s2": self._accel[0],
                                    "angular_rad_s2": self._accel[1]},
            "command_age_ms": (int((time.monotonic() - self._last_valid_at) * 1000)
                               if self._last_valid_at is not None else None),
        }
