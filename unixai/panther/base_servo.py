"""Continuous domain-42 motus.control/1 input -> Panther body-frame Twist.

Only the isolated receiver uses FastDDS. Validation, expiry and zero-output
watchdog run on a separate local thread, independent of ROS executor progress.
Bundle startup never attaches the input or reserves the chassis.
"""
from __future__ import annotations

import copy
import json
import math
import multiprocessing as mp
import os
import queue
import re
import threading
import time

from common.control import ControlSink, parse_descriptor

INPUT_TOPIC = "/actucore/navi/cmd"
MAX_HZ = 20.
WATCHDOG_MS = 300
MAX_OBS_AGE_MS = 500
MAX_TTL_MS = 500
ZERO = (0.,) * 6


def descriptor(config, drive_config):
    def positive(key, default):
        value = config.get(key, default)
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
            raise ValueError(f"base_servo.{key} must be positive and finite")
        return float(value)
    hz = positive("expected_hz", 10.)
    drive_linear = drive_config["max_linear_mps"]
    drive_angular = math.radians(drive_config["max_angular_deg_s"])
    vx = positive("max_linear_mps", drive_linear)
    wz = positive("max_angular_rad_s", drive_angular)
    if hz > MAX_HZ or vx > drive_linear or wz > drive_angular:
        raise ValueError("base_servo rate/speed exceeds Panther base limits")
    linear_step = positive("linear_acceleration_mps2", .3) / hz
    angular_step = positive("angular_acceleration_rad_s2", .6) / hz
    result = {"control_interface": "motus.control/1", "mode": "twist", "dof": 6,
              "joint_names": ["vx", "vy", "vz", "wx", "wy", "wz"],
              "units": {"linear": "m/s", "angular": "rad/s", "time": "s"},
              "frame": "base_link", "force_torque": None,
              "limits": {"lower": [-vx, -vx, 0., 0., 0., -wz],
                         "upper": [vx, vx, 0., 0., 0., wz],
                         "max_delta_per_step": [linear_step, linear_step, 1e-6, 1e-6, 1e-6, angular_step],
                         "zero_immediate": True},
              "groups": [{"name": "base", "offset": 0, "count": 6, "unit": "m/s", "resource": "base"}],
              "rate": {"max_hz": MAX_HZ, "expected_hz": hz, "watchdog_ms": WATCHDOG_MS,
                       "max_obs_age_ms": MAX_OBS_AGE_MS}}
    if config.get("footprint"):
        footprint = copy.deepcopy(config["footprint"])
        if not isinstance(footprint.get("half_width"), (int, float)) or not math.isfinite(footprint["half_width"]) or footprint["half_width"] <= 0:
            raise ValueError("base_servo footprint half_width must be positive finite metres")
        result["footprint"] = footprint
    parse_descriptor(result)
    return result


def _receive(topic, domain, profile, inbox, ready, closing):
    from common import logsafe
    logsafe.install(check_fd=False)
    os.environ["ROS_DOMAIN_ID"] = str(domain)
    os.environ["RMW_IMPLEMENTATION"] = "rmw_fastrtps_cpp"
    os.environ["FASTRTPS_DEFAULT_PROFILES_FILE"] = profile
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from std_msgs.msg import String
    rclpy.init(domain_id=domain)
    node = Node("panther_base_servo_input", enable_rosout=False, start_parameter_services=False)
    def received(message):
        if len(message.data) > 65536:
            return
        try:
            item = (json.loads(message.data), time.monotonic())
        except (ValueError, TypeError):
            return
        try:
            inbox.put_nowait(item)
        except queue.Full:
            try:
                inbox.get_nowait()
            except queue.Empty:
                pass
            try:
                inbox.put_nowait(item)
            except queue.Full:
                pass
    node.create_subscription(String, topic, received, qos_profile_sensor_data)
    ready.set()
    try:
        while not closing.is_set():
            rclpy.spin_once(node, timeout_sec=.02)
    finally:
        node.destroy_node()
        rclpy.shutdown()


class BaseServoPlugin:
    def __init__(self, device, config):
        self.device = device
        self._lock = device._base_gate
        self._lifecycle_lock = threading.RLock()
        self._raw = descriptor(config, device._cfg["plugins"]["base_drive"])
        self._descriptor = parse_descriptor(self._raw)
        self._dry_run = config.get("dry_run", False)
        self._rotate_only = config.get("rotate_only", False)
        if type(self._dry_run) is not bool or type(self._rotate_only) is not bool:
            raise ValueError("base_servo toggles must be booleans")
        self._ctx = mp.get_context("spawn")
        self._process = self._inbox = self._closing = None
        self._thread = None
        self._thread_end = threading.Event()
        self._stopped = threading.Event()
        self._stopped.set()
        self._running = self._paused = self._reserved = False
        self._sink = None
        self._input = ""
        self._values = ZERO
        self._lease_until = 0.
        self._arm_time = self._arm_wall = 0.
        self._last_publish = self._last_apply = 0.
        self._candidate_lease = 0.
        self._pending = None
        self._zero_count = 0
        self._stop_confirmed = True
        self._error = ""
        self._last_command = None
        self._applied = self._holds = self._rejected = 0

    def get_tool(self):
        toggles = {key: {"type": "boolean", "default": getattr(self, "_" + key)}
                   for key in ("dry_run", "rotate_only")}
        return {"name": "base_servo", "type": "actuator", "multiInstance": False,
                "description": "Panther continuous body velocity from ActuCore navi; explicit start, expiry and zero-output watchdog",
                "inputSchema": {"type": "object", "required": ["action"], "additionalProperties": False,
                    "properties": {"action": {"type": "string", "enum": ["start", "stop", "pause", "resume", "info", "config"]},
                                   "input_topic": {"type": "string", "default": INPUT_TOPIC}, **toggles},
                    "x-action-params": {"start": {"params": ["input_topic"]},
                        **{a: {"params": []} for a in ("stop", "pause", "resume", "info")},
                        "config": {"params": list(toggles)}},
                    "x-resource": "base", "x-hooks": {
                        "on_interrupt_motion": {"action": "pause"}, "on_interrupt_all": {"action": "pause"}}},
                "configSchema": {"type": "object", "properties": toggles, "additionalProperties": False},
                "topic_in": [{"format": "control/velocity", "default_topic": INPUT_TOPIC}],
                "control_interface": copy.deepcopy(self._raw)}

    def start(self):
        pass  # DriverBundle.start_all must never arm continuous motion.

    def stop(self):
        return self.dispatch("stop", {})

    def dispatch(self, action, args):
        with self._lifecycle_lock:
            return self._dispatch(action, args)

    def _dispatch(self, action, args):
        try:
            if action == "start":
                return self._start(args)
            if action in ("stop", "pause"):
                return self._halt(stop=action == "stop")
            with self._lock:
                if action == "resume":
                    if not self._running:
                        raise RuntimeError("start base_servo before resume")
                    if not self._stop_confirmed:
                        raise RuntimeError("previous zero-output stop is unconfirmed")
                    self.device.base_check_available("base_servo")
                    self.device.servo_prepare(self._dry_run)
                    if self._process is None or not self._process.is_alive():
                        raise RuntimeError("control input receiver exited; stop and start again")
                    self._arm()
                elif action == "config":
                    if self._reserved:
                        raise RuntimeError("pause/stop and confirm zero output before configuration")
                    updates = {key: args[key] for key in ("dry_run", "rotate_only") if key in args}
                    if any(type(v) is not bool for v in updates.values()):
                        raise ValueError("dry_run and rotate_only must be booleans")
                    for key, value in updates.items():
                        setattr(self, "_" + key, value)
                elif action != "info":
                    return {"state": "error", "error": "unknown action"}
                return self.info()
        except Exception as exc:
            return {"state": "error", "error": str(exc), "stop_confirmed": self._stop_confirmed}

    def _arm(self):
        self._sink = ControlSink(self._descriptor, self._apply,
                                 on_watchdog=self._hold, on_abort=self._hold)
        self._sink.rebase(ZERO)
        self._paused = False
        self._reserved = True
        self._pending = None
        self._values = ZERO
        self._lease_until = 0.
        self._arm_time, self._arm_wall = time.monotonic(), time.time() * 1000
        self._zero_count = 0
        self._stop_confirmed = False
        self._error = ""
        self._stopped.clear()

    def _start(self, args):
        topic = args.get("input_topic") or INPUT_TOPIC
        if not isinstance(topic, str) or not re.fullmatch(r"/[A-Za-z_][A-Za-z0-9_/]*", topic) or "//" in topic or topic.endswith('/'):
            raise ValueError("input_topic must be an absolute ROS topic")
        with self._lock:
            if self._running:
                if topic != self._input:
                    raise RuntimeError("stop before changing the input topic")
                return self.info()
            if self._reserved:
                raise RuntimeError("previous stop is unconfirmed")
            self.device.base_check_available("base_servo")
            self.device.servo_prepare(self._dry_run)
            self._arm()
            self._running = True
            self._input = topic
            self._inbox = self._ctx.Queue(maxsize=1)
            self._closing = self._ctx.Event()
            ready = self._ctx.Event()
            self._process = self._ctx.Process(target=_receive,
                args=(topic, int(os.environ.get("AGENT_CORE_ROS_DOMAIN_ID", "42")),
                      "/opt/phanthy-motus/dds-local.xml", self._inbox, ready, self._closing), daemon=True)
            self._thread_end.clear()
            self._thread = threading.Thread(target=self._loop, daemon=True, name="panther_base_servo")
            try:
                self._process.start()
                self._thread.start()
            except Exception:
                self._running = False
                self._paused = True
                self._reserved = False  # No output has been applied during setup.
                self._closing.set()
                self._stopped.set()
                if self._process.pid is not None and self._process.is_alive():
                    self._process.terminate()
                    self._process.join(timeout=2)
                raise
        if not ready.wait(8) or not self._process.is_alive():
            self._halt(stop=True)
            raise RuntimeError("control input receiver did not start")
        return self.info()

    def _halt(self, stop=False):
        with self._lock:
            self._paused = True
            self._pending = None
            self._hold()
            if stop:
                self._running = False
                if self._closing is not None:
                    self._closing.set()
            process = self._process if stop else None
        # The independent output thread must keep running while we await zeros.
        self._stopped.wait(1.)
        if process is not None:
            process.join(timeout=2)
            if process.is_alive():
                process.terminate()
                process.join(timeout=2)
            if self._thread is not None and self._stopped.is_set():
                self._thread.join(timeout=2)
            with self._lock:
                self._process = None
                if self._inbox is not None:
                    self._inbox.close()
                    self._inbox = None
        with self._lock:
            return self.info()

    def pause_for_explicit_command(self):
        if self._reserved:
            self._paused = True
            self._hold()
            # Called under the shared chassis gate. Emit a bounded zero burst
            # synchronously; the normal output thread cannot take this lock.
            for _ in range(3):
                self._zero()
                if self._stop_confirmed:
                    break
                time.sleep(.05)
            if not self._stop_confirmed:
                raise RuntimeError("base_servo stop is unconfirmed; explicit motion refused")

    def _apply(self, values, _gripper=None):
        values = tuple(values)
        if self._rotate_only:
            values = (0., 0., 0., 0., 0., values[5])
        matched = self._dry_run or self.device.servo_publish(values)
        if any(values) and not matched:
            raise RuntimeError("nonzero stream output has no matched subscriber")
        self._values = values
        self._lease_until = self._candidate_lease
        self._last_publish = self._last_apply = time.monotonic()
        self._zero_count = 0 if any(values) or not matched else self._zero_count + 1
        self._stop_confirmed = self._zero_count >= 3
        self._applied += 1

    def _hold(self):
        self._values = ZERO
        self._lease_until = 0.
        self._holds += 1
        if self._sink is not None:
            self._sink.rebase(ZERO)

    def _zero(self):
        try:
            matched = self._dry_run or self.device.servo_publish(ZERO)
            self._zero_count = self._zero_count + 1 if matched else 0
            if not matched:
                self._stop_confirmed = False
            self._last_publish = time.monotonic()
            if self._zero_count >= 3:
                self._stop_confirmed = True
                if self._paused or not self._running:
                    self._reserved = False
                    self._stopped.set()
        except Exception as exc:
            self._zero_count = 0
            self._stop_confirmed = False
            self._error = str(exc)

    def _accept(self, command, received):
        now, wall = time.monotonic(), time.time() * 1000
        try:
            if not isinstance(command, dict):
                raise ValueError("command must be an object")
            stamp, ttl, obs = (command.get(key) for key in ("stamp_ms", "ttl_ms", "obs_stamp_ms"))
            if any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) for v in (stamp, ttl, obs)):
                raise ValueError("finite stamp_ms, ttl_ms and obs_stamp_ms are required")
            if received < self._arm_time or stamp < self._arm_wall:
                raise ValueError("command predates the current start/resume")
            if stamp > wall + 50 or obs > wall + 50 or not 0 < ttl <= MAX_TTL_MS:
                raise ValueError("future timestamp or ttl_ms outside (0, 500]")
            lease_ms = min(WATCHDOG_MS, stamp + ttl - wall, obs + MAX_OBS_AGE_MS - wall)
            if lease_ms <= 0:
                raise ValueError("expired command or observation")
            values = command.get("values")
            if isinstance(values, (list, tuple)) and len(values) == 6 and any(values[i] != 0 for i in (2, 3, 4)):
                raise ValueError("vz, wx and wy are unsupported and must be zero")
            self._candidate_lease = now + lease_ms / 1000
            outcome = self._sink.submit(command)
            self._last_command = {"verdict": outcome.verdict.value, "reason": outcome.reason}
            if not outcome.applied:
                self._rejected += 1
        except (ValueError, TypeError) as exc:
            self._rejected += 1
            self._last_command = {"verdict": "REJECTED", "reason": str(exc)}

    def _step(self):
        with self._lock:
            now = time.monotonic()
            if self._running and not self._paused:
                if self._process is not None and not self._process.is_alive():
                    self._error = "control input receiver exited"
                    self._paused = True
                    self._hold()
                else:
                    if self._inbox is not None:
                        try:
                            self._pending = self._inbox.get_nowait()
                        except queue.Empty:
                            pass
                    if self._pending is not None:
                        cmd, received = self._pending
                        explicit_zero = isinstance(cmd, dict) and cmd.get("values") == list(ZERO)
                        if explicit_zero or now - self._last_apply >= 1 / MAX_HZ:
                            self._pending = None
                            self._accept(cmd, received)
                    self._sink.tick()
            if self._reserved and now - self._last_publish >= 1 / MAX_HZ:
                if self._running and not self._paused and self._lease_until > now and any(self._values):
                    if not self._dry_run:
                        self.device.servo_publish(self._values)
                    self._last_publish = now
                else:
                    if any(self._values):
                        self._hold()
                    self._zero()

    def _loop(self):
        while not self._thread_end.wait(.01):
            try:
                self._step()
            except Exception as exc:
                with self._lock:
                    self._error = str(exc)
                    self._paused = True
                    self._hold()
            with self._lock:
                if not self._running and not self._reserved:
                    return

    def info(self):
        with self._lock:
            state = ("stop_unconfirmed" if self._reserved and self._paused and not self._stop_confirmed else
                     "paused" if self._running and self._paused else "running" if self._running else "idle")
            return {"state": state, "input_topic": self._input, "dry_run": self._dry_run,
                    "rotate_only": self._rotate_only, "reserved": self._reserved,
                    "stop_confirmed": self._stop_confirmed, "error": self._error,
                    "applied": self._applied, "holds": self._holds, "rejected": self._rejected,
                    "last_command": self._last_command, "sink": self._sink.stats() if self._sink else None,
                    "control_interface": copy.deepcopy(self._raw)}
