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
from slamtec_velocity import CommandLeaseExpired, SlamtecVelocityBackend

INPUT_TOPIC = "/actucore/navi/cmd"
WATCHDOG_MS = 300
TOGGLES = {"dry_run": True, "rotate_only": False, "require_standing": False}


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

    def put(self, message, generation):
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


def build_descriptor(hz=10.0, max_vx=0.3, max_wz=1.0):
    return {
        "control_interface": "motus.control/1", "mode": "twist", "dof": 6,
        "joint_names": ["vx", "vy", "vz", "wx", "wy", "wz"],
        "frame": "base_link",
        "units": {"linear": "m/s", "angular": "rad/s", "time": "s"},
        "limits": {"lower": [-max_vx, 0, 0, 0, 0, -max_wz],
                   "upper": [max_vx, 0, 0, 0, 0, max_wz]},
        "groups": [{"name": "base", "offset": 0, "count": 6,
                    "unit": "m/s", "resource": "base"}],
        "rate": {"max_hz": 20, "expected_hz": hz,
                 "watchdog_ms": WATCHDOG_MS, "max_obs_age_ms": 500},
        "force_torque": None,
    }


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
        self._descriptor = build_descriptor(
            self._hz, self._positive(config, "max_vx", 0.3),
            self._positive(config, "max_wz", 1.0))
        self._parsed_descriptor = parse_descriptor(self._descriptor)
        self._accel = (self._positive(config, "linear_acceleration", 0.3),
                       self._positive(config, "angular_acceleration", 0.6))
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
        self._lease_expiry_ms = 0
        self._zero_pending = False
        self._last = None
        self._last_timing = None
        self._counts = {key: 0 for key in
                        ("received", "accepted", "published", "dry_run_commands",
                         "rejected", "dropped", "holds", "publish_errors", "refreshed_after_stop")}
        self._inbox = _LatestControlInbox()

    @staticmethod
    def _positive(config, key, default):
        value = config.get(key, default)
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ValueError(f"loco_servo.{key} must be a positive finite number")
        if not math.isfinite(value) or value <= 0:
            raise ValueError(f"loco_servo.{key} must be a positive finite number")
        return float(value)

    def get_tool(self):
        return {
            "name": "loco_servo", "type": "actuator", "multiInstance": False,
            "description": "天轶底盘连续速度控制：接收 navi 的速度流，通过思岚 SDK 执行 vx/wz；默认空跑。",
            "inputSchema": {
                "type": "object", "required": ["action"],
                "properties": {
                    "action": {"type": "string", "enum":
                               ["start", "stop", "pause", "resume", "info"]},
                    "input_topic": {"type": "string", "default": INPUT_TOPIC}},
                "x-resource": ["base"], "x-is-dangerous": True,
                "x-hooks": {"on_interrupt_motion": {"action": "pause"},
                            "on_interrupt_all": {"action": "pause"}},
                "x-action-params": {
                    "pause": {"params": [], "description": "停止底盘，保持订阅"},
                    "resume": {"params": [], "description": "恢复接收新的速度指令"}}},
            "topic_in": [{"topic": INPUT_TOPIC, "format": "control/velocity",
                          "desc": "motus.control/1 twist [vx,vy,vz,wx,wy,wz]"}],
            "configSchema": {"type": "object", "properties": {
                "dry_run": {"type": "boolean", "default": True,
                            "description": "空跑：照常接收、检查、计数，但不创建 SDK 运动动作。第一次上机先保持开启，检查它想发什么。"},
                "rotate_only": {"type": "boolean", "default": False,
                                "description": "只转不走：vx / vy 在到达底盘前被清零，保留 vyaw。"},
                "require_standing": {"type": "boolean", "default": False,
                                     "description": "要求有效的站立反馈才执行指令。天轶需配置 standing_topic（域0 Bool）；开启后缺失、过期或 false 均拒绝运动。"}}},
        }

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
            return self._info()
        problem = self._conflict()
        if problem:
            return {"state": "error", "error": problem}
        self._generation += 1
        generation = self._generation
        self._yielded = False
        self._input_topic = topic.strip()
        self._barrier_ms = int(time.time() * 1000)
        self._sink = ControlSink(self._parsed_descriptor, self._apply,
                                 on_watchdog=self._hold, on_abort=self._hold)
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

    def _on_message(self, message, generation=None):
        # No plugin/SDK lock here: waiting on it would block the socket reader
        # and accumulate already-aging commands in the Unix socket buffer.
        if self._inbox.put(message, generation):
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
                if any(command["values"]):
                    if (self._lease_expiry_ms and time.time() * 1000 >= self._lease_expiry_ms and
                            not self._native_expiry_update()):
                        self._hold()  # Native lease may already have stopped the chassis.
                        if self._zero_pending:
                            raise RuntimeError("SDK stop unconfirmed")
                        pending = self._inbox.take(newer_than=command)
                        if pending is not None:
                            self._counts["dropped"] += 1
                            self._counts["refreshed_after_stop"] += 1
                            # Revalidate the replacement, including the original
                            # TTL, observation age, seq and source arbitration.
                            # _hold cleared the lease, so this refresh is once.
                            self._process_message(*pending)
                            return
                    problem = self._standing_problem()
                    if problem:
                        self._hold()
                        self._counts["rejected"] += 1
                        self._record("REFUSED", problem)
                        return
                self._command_deadline_ms = min(
                    command["stamp_ms"] + command.get("ttl_ms", 0),
                    command.get("obs_stamp_ms", command["stamp_ms"]) + 500,
                    int(time.time() * 1000) + WATCHDOG_MS)
                started = time.monotonic()
                wall_ms = int(time.time() * 1000)
                self._last_timing = {
                    "seq": command.get("seq"), "ttl_ms": command.get("ttl_ms"),
                    "receive_age_ms": (received_at[1] - command["stamp_ms"]
                                       if received_at is not None else None),
                    "local_wait_ms": (round((started - received_at[0]) * 1000, 1)
                                      if received_at is not None else None),
                    "command_age_ms": wall_ms - command["stamp_ms"],
                    "observation_age_ms": wall_ms - command.get("obs_stamp_ms", command["stamp_ms"]),
                    "remaining_before_apply_ms": self._command_deadline_ms - wall_ms,
                }
                try:
                    outcome = self._sink.submit(command)
                finally:
                    self._last_timing.update(
                        apply_round_trip_ms=round((time.monotonic() - started) * 1000, 1),
                        remaining_after_apply_ms=self._command_deadline_ms - int(time.time() * 1000))
                if outcome.applied:
                    self._counts["accepted"] += 1
                    self._last_valid_at = time.monotonic()
                    self._lease_expiry_ms = self._command_deadline_ms
                    self._record(outcome.verdict.name, outcome.reason,
                                 requested=list(command["values"]),
                                 output=list(self._last_output),
                                 dry_run=self._settings["dry_run"])
                else:
                    self._counts["dropped" if outcome.verdict.name == "DROPPED" else "rejected"] += 1
                    self._record(outcome.verdict.name, outcome.reason)
            except CommandLeaseExpired as exc:
                self._counts["dropped"] += 1
                self._hold()
                if self._zero_pending:
                    self._paused = True  # Preserve the stop error from _hold.
                else:
                    self._record("DROPPED", str(exc))
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

    def _apply(self, values, _gripper=None):
        target = list(values)
        if self._settings["rotate_only"]:
            target[0] = target[1] = 0.0
        replace_expired = bool(any(target) and self._native_expiry_update() and
                               self._lease_expiry_ms and
                               time.time() * 1000 >= self._lease_expiry_ms)
        if replace_expired:
            # The native UPDATE confirms any expired old action's stop before
            # starting this fresh one. Restart the ramp locally, but do not
            # spend a separate Python STOP/ACK round trip before the UPDATE.
            self._last_output = [0.0] * 6
            self._last_output_at = None
        now = time.monotonic()
        dt = min(1 / self._hz, max(0, now - self._last_output_at)) if self._last_output_at is not None else 1 / self._hz
        output = [0.0] * 6
        for axis, accel in ((0, self._accel[0]), (5, self._accel[1])):
            # Explicit zero bypasses acceleration limiting: stop means stop.
            if target[axis]:
                step = accel * dt
                output[axis] = max(self._last_output[axis] - step,
                                   min(target[axis], self._last_output[axis] + step))
        if self._settings["dry_run"]:
            self._counts["dry_run_commands"] += 1
        else:
            self._publish(output, replace_expired=replace_expired)
        self._last_output = output
        self._last_output_at = now
        self._zero_pending = False

    def _publish(self, values, *, replace_expired=False):
        if self._backend is None:
            raise RuntimeError("Slamware SDK backend unavailable")
        try:
            if any(values):
                if replace_expired:
                    self._backend.write(values, self._command_deadline_ms, replace_expired=True)
                else:
                    self._backend.write(values, self._command_deadline_ms)
            else:
                self._backend.stop()
        except CommandLeaseExpired:
            raise
        except Exception as exc:
            raise RuntimeError(f"Slamware SDK write failed: {exc}") from exc
        self._counts["published"] += 1

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
            if self._lease_expiry_ms and time.time() * 1000 >= self._lease_expiry_ms:
                self._hold()
                self._record("HOLD", "command TTL or observation lease expired")
            if self._last_valid_at is not None and time.monotonic() - self._last_valid_at >= WATCHDOG_MS / 1000:
                self._hold()
                self._record("HOLD", "no valid command for 300 ms")
            self._sink.tick()

    def _configure(self, args):
        updates = {key: args[key] for key in TOGGLES if key in args}
        if any(not isinstance(value, bool) for value in updates.values()):
            return {"state": "error", "error": "configuration toggles must be boolean"}
        if any(self._settings[key] != value for key, value in updates.items()):
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
            self._settings.update(updates)
            self._barrier_ms = int(time.time() * 1000)
            self._inbox.activate(self._generation,
                                 self._running and not self._paused and not self._yielded)
        return {"ok": True, **self._info()}

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
            "input": self._input_topic, **self._settings,
            "yielded_to_explicit_command": self._yielded,
            "control_interface": interface,
            "counts": dict(self._counts), "last": copy.deepcopy(self._last),
            "input_queue": self._inbox.info(),
            "dispatch": {"mode": "dedicated_worker", "watchdog_poll_ms": 20,
                         "max_apply_hz": 20, "worker_failed": self._controller_failed,
                         "expiry_update": "native_atomic" if self._native_expiry_update() else "stop_then_set",
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
