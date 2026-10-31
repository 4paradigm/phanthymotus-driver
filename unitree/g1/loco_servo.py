#!/usr/bin/env python3
"""G1 motus.control/1 twist stream; direct SDK path, not SmartMotion safety.

ControlSink checks the stream contract, freshness and declared limits. It does
not provide SmartMotion obstacle/collision checks. No cross-card
arbitration is performed. The operator must avoid competing chassis writers.
"""
from __future__ import annotations

import json
import math
import threading
import time

from common.control import ControlSink, parse_descriptor

AXIS_NAMES = ["vx", "vy", "vz", "wx", "wy", "wz"]
DEFAULT_EXPECTED_HZ = 10.0
MAX_HZ = 20.0
WATCHDOG_MS = 300
MAX_OBS_AGE_MS = 500
# G1 device.py describes these velocity ranges. Step limits are conservative
# software choices, not measured G1 acceleration or deadband specifications.
STANDING_FSMS = frozenset((500, 501, 801, 802))
FSM_CACHE_S = 1.0


def build_descriptor(expected_hz=DEFAULT_EXPECTED_HZ):
    return {
        "control_interface": "motus.control/1", "mode": "twist", "dof": 6,
        "joint_names": list(AXIS_NAMES),
        "units": {"linear": "m/s", "angular": "rad/s", "time": "s"},
        "limits": {"lower": [-1., -1., 0., 0., 0., -2.],
                   "upper": [1., 1., 0., 0., 0., 2.],
                   "max_delta_per_step": [0.15, 0.15, 1e-6, 1e-6, 1e-6, 0.30]},
        "groups": [{"name": "base_linear", "offset": 0, "count": 3,
                    "unit": "m/s", "resource": "base"},
                   {"name": "base_angular", "offset": 3, "count": 3,
                    "unit": "rad/s", "resource": "base"}],
        "frame": "base_link",
        "rate": {"max_hz": MAX_HZ, "expected_hz": expected_hz,
                 "watchdog_ms": WATCHDOG_MS, "max_obs_age_ms": MAX_OBS_AGE_MS},
        "force_torque": None,
    }


class LocoServoPlugin:
    PREFIX = "locoservo"
    _TOGGLES = ("dry_run", "rotate_only", "require_standing")

    def __init__(self, plugin_config, namespace, executor, transport, fsm_reader):
        config = plugin_config or {}
        self._ns, self._executor, self._transport = namespace, executor, transport
        self._fsm_reader = fsm_reader
        self._command = None
        self._stop_requested = False
        self._physical_dirty = False
        self._expected_hz = float(config.get("expected_hz", DEFAULT_EXPECTED_HZ))
        if not math.isfinite(self._expected_hz) or not 1000 / WATCHDOG_MS < self._expected_hz <= MAX_HZ:
            raise ValueError(f"loco_servo.expected_hz must be in ({1000 / WATCHDOG_MS}, {MAX_HZ}]")
        self._dry_run = bool(config.get("dry_run", True))
        self._rotate_only = bool(config.get("rotate_only", False))
        self._require_standing = bool(config.get("require_standing", True))
        self._descriptor_raw = build_descriptor(self._expected_hz)
        self._descriptor = parse_descriptor(self._descriptor_raw)
        self._lock = threading.RLock()
        self._sink = self._sub_node = self._ticker = None
        self._running = self._paused = False
        self._auto_resume = False
        self._manual_pause = False
        self._shutdown = False
        self._input_topic = ""
        self._last_command = None
        self._stop_failed = False
        self._submitted_values = self._effective_values = None
        self._applied = self._holds = self._refused = self._sdk_errors = 0
        self._suppressed = 0
        self._last_ret = 0
        self._last_error = self._fsm_problem = ""
        self._fsm_checked_at = float("-inf")

    def get_tool(self):
        return {"name": "loco_servo", "type": "actuator", "multiInstance": False,
                "description": "G1 连续速度控制，直发 SDK；不经过 SmartMotion，无避障或跨卡互斥。",
                "inputSchema": {"type": "object", "properties": {
                    "action": {"type": "string", "enum": ["start", "stop", "pause", "resume", "info"]},
                    "input_topic": {"type": "string", "description": "control/velocity 指令流话题"}},
                    "required": ["action"],
                    "x-action-params": {"pause": {"params": [], "description": "停止底盘并保留订阅"},
                                        "resume": {"params": [], "description": "继续执行"}},
                    "x-hooks": {"on_interrupt_motion": {"action": "pause"},
                                "on_interrupt_all": {"action": "pause"}},
                    "x-is-dangerous": True},
                "topic_in": [{"format": "control/velocity", "desc": "motus.control/1 twist [vx,vy,vz,wx,wy,wz]"}],
                "configSchema": {"type": "object", "properties": {
                    "dry_run": {"type": "boolean", "default": True, "description": "空跑，不发送 SDK 指令"},
                    "rotate_only": {"type": "boolean", "default": False, "description": "只转向，抑制平移"},
                    "require_standing": {"type": "boolean", "default": True, "description": "每条指令检查 G1 运控 FSM"}}}}

    def dispatch(self, action, args):
        if action == "start": return self._start(args)
        if action == "stop": return self._stop()
        if action == "pause": return self._halt(True)
        if action == "resume": return self._halt(False)
        if action == "info": return self._info()
        if action == "config": return self._config(args)
        return None

    def start(self):
        pass  # Bundle start must not subscribe or move.

    def stop(self):
        # Seal the bundle lifecycle before waiting, so an in-flight HTTP start
        # cannot restart this card after shutdown has passed it.
        with self._lock:
            self._shutdown = True
        result = self._stop()
        if result.get("state") == "stop_pending":
            # Give the dedicated writer and SDK response a bounded opportunity
            # before the process exits; timeout still cannot prove standstill.
            self._transport.wait_stop(1.5)
            result = self._stop()
        if result.get("state") not in ("stopped", "idle"):
            raise RuntimeError(result.get("error") or "stop not confirmed by SDK")
        if result.get("error"):
            raise RuntimeError(result["error"])
        return result

    def _transport_status(self):
        status = self._transport.status()
        if status.get("paused") or status.get("stop_status") != "ok":
            self._paused = True
        if status.get("last_error"):
            self._last_error = str(status["last_error"])
        return status

    def _config(self, args):
        with self._lock:
            if args.get("dry_run") is True and not self._dry_run and self._running:
                if not self._stop_requested:
                    self._hold()
                status = self._transport_status()
                if self._stop_failed or status.get("stop_status") != "ok":
                    return {"ok": False, "state": "stop_failed" if self._stop_failed else "stop_" + status.get("stop_status", "failed"),
                            "error": self._last_error or "stop not confirmed by SDK"}
            if args.get("dry_run") is True and not self._dry_run:
                self._physical_dirty = False
            for key in self._TOGGLES:
                if key in args:
                    setattr(self, "_" + key, bool(args[key]))
            return {"ok": True, "state": self._state(),
                    **{key: getattr(self, "_" + key) for key in self._TOGGLES}}

    def _state(self):
        status = self._transport_status()
        if self._stop_failed:
            return "stop_failed"
        if status.get("stop_status") != "ok":
            return "stop_" + status.get("stop_status", "failed")
        return "paused" if self._running and self._paused else "running" if self._running else "idle"

    def _start(self, args):
        topic = (args.get("input_topic") or "").strip()
        if not topic:
            topic = ((args.get("input_topics") or [""])[0] or "").strip()
        if not topic:
            return {"state": "error", "message": "缺少 input_topic，请连接 control/velocity"}
        if self._executor is None:
            return {"state": "error", "message": "没有 ROS 上下文，无法订阅"}
        with self._lock:
            if self._shutdown:
                return {"state": "error", "message": "驱动正在关闭"}
            if self._running:
                return {"state": "error", "message": f"已经在运行（{self._input_topic}）"}
            if not self._transport.resume():
                return {"state": "error", "error": "transport stop not confirmed"}
            self._stop_requested = False
            self._sink = ControlSink(self._descriptor, self._apply,
                                     on_watchdog=self._watchdog_hold, on_abort=self._hold)
            self._auto_resume = False
            self._manual_pause = False
            self._running, self._paused, self._input_topic = True, False, topic
            self._applied = self._holds = 0
            try:
                self._open(topic)
            except Exception as exc:
                self._running = False
                self._sink = None
                self._close_node()
                return {"state": "error", "message": f"启动失败: {exc}"}
            return {"state": "running", "input": topic, "dry_run": self._dry_run,
                    "rotate_only": self._rotate_only, "control_interface": self._interface()}

    def _halt(self, halted):
        with self._lock:
            if self._shutdown and not halted:
                return {"state": "error", "message": "驱动正在关闭"}
            if not self._running:
                return {"state": "idle", "message": "卡片未在运行"}
            status = self._transport_status()
            if halted:
                self._manual_pause = True
                self._auto_resume = False
                if (not self._stop_requested or self._stop_failed
                        or status.get("stop_status") in ("failed", "pending")):
                    self._hold()
                return {"state": self._state(), "input": self._input_topic,
                        "stop_ack": self._transport.status().get("stop_status")}
            if self._stop_failed or not self._transport.resume():
                return {"state": "error", "input": self._input_topic,
                        "error": self._last_error or "stop not confirmed by SDK"}
            self._paused = False
            self._manual_pause = False
            self._auto_resume = False
            self._stop_requested = False
            self._last_error = ""
            return {"state": "running", "input": self._input_topic}

    def _stop(self):
        with self._lock:
            if not self._running:
                return {"state": "idle"}
            status = self._transport_status()
            if not self._stop_requested or self._stop_failed or status.get("stop_status") == "failed":
                self._hold()
                status = self._transport_status()
            if self._stop_failed or not self._stop_requested:
                return {"state": "stop_failed", "error": self._last_error}
            if status.get("stop_status") != "ok":
                return {"state": "stop_" + status.get("stop_status", "failed"),
                        "error": self._last_error or "stop not confirmed by SDK"}
            self._physical_dirty = False
            self._running = self._paused = False
            self._sink = None
            self._close_node()
            return {"state": "stopped", "stop_ack": "ok", "physical_stop_confirmed": False}

    def _interface(self):
        out = dict(self._descriptor_raw)
        if self._dry_run: out["dry_run"] = True
        if self._rotate_only: out["rotate_only"] = True
        return out

    def _info(self):
        with self._lock:
            return {"state": self._state(), "input": self._input_topic,
                    "dry_run": self._dry_run, "rotate_only": self._rotate_only,
                    "require_standing": self._require_standing,
                    "suppressed_translations": self._suppressed, "applied": self._applied,
                    "holds": self._holds, "refused": self._refused, "sdk_errors": self._sdk_errors,
                    "last_ret": self._last_ret, "last_error": self._last_error,
                    "posture_problem": self._fsm_problem, "last": self._last_command,
                    "transport": self._transport.status(), "physical_stop_confirmed": False,
                    "control_interface": self._interface()}

    def _posture_problem(self):
        if not self._require_standing:
            return ""
        try:
            fsm = self._fsm_reader()
            stamp = fsm.get("ts")
            if (isinstance(stamp, bool) or not isinstance(stamp, (int, float))
                    or not math.isfinite(stamp) or not 0 <= time.time() - stamp < FSM_CACHE_S):
                self._fsm_problem = "FSM 状态缺失或过期"
            elif type(fsm.get("fsm_id")) is not int or fsm["fsm_id"] not in STANDING_FSMS:
                self._fsm_problem = f"G1 FSM={fsm.get('fsm_id')}，不是主运控状态"
            else:
                self._fsm_problem = ""
        except Exception as exc:
            self._fsm_problem = f"读不到 FSM 状态（{exc}）"
        return self._fsm_problem

    def _open(self, topic):
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
        from std_msgs.msg import String
        node = Node(f"g1_loco_servo_{abs(hash(topic)) % 100000}")
        self._sub_node = node  # visible for partial setup cleanup
        qos = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                         reliability=ReliabilityPolicy.BEST_EFFORT)
        node.create_subscription(String, topic, self._on_message, qos)
        self._ticker = node.create_timer(WATCHDOG_MS / 2000., self._tick)
        self._executor.add_node(node)

    def _close_node(self):
        node, ticker = self._sub_node, self._ticker
        self._sub_node = self._ticker = None
        if node is None: return
        if ticker is not None:
            try: node.destroy_timer(ticker)
            except Exception: pass
        try: self._executor.remove_node(node)
        except Exception: pass
        try: node.destroy_node()
        except Exception: pass

    def _on_message(self, message):
        with self._lock:
            status = self._transport_status()
            auto = (not self._manual_pause and not self._stop_failed
                    and status.get("stop_status") == "ok"
                    and (self._auto_resume or status.get("stop_reason") == "watchdog"))
            if self._shutdown or not self._running or self._sink is None or (self._paused and not auto): return
            if self._paused and auto:
                self._sink._last_values = (0.,) * 6
            try:
                command = json.loads(message.data)
            except (ValueError, TypeError):
                return
            self._command = command
            self._submitted_values = command.get("values") if isinstance(command, dict) else None
            self._effective_values = None
            try:
                outcome = self._sink.submit(command)
                if self._effective_values is not None:
                    # The sink stores its clamped output after apply; retain what was
                    # actually sent so subsequent steps cannot resurrect zero axes.
                    self._sink._last_values = self._effective_values
            except Exception as exc:
                self._last_command = {"verdict": "ERROR", "reason": str(exc), "at": time.time()}
                self._hold()
                return
            finally:
                self._submitted_values = None
                self._command = None
            self._last_command = {"verdict": str(getattr(outcome.verdict, "name", outcome.verdict)),
                                  "reason": outcome.reason, "at": time.time()}

    def _tick(self):
        with self._lock:
            if self._running and self._state() == "running" and self._sink is not None:
                self._sink.tick()

    def _apply(self, values, _gripper=None):
        # apply is reached only after contract, freshness and arbitration checks.
        # An explicit zero must not be turned into motion by step clamping.
        values = list(values)
        if self._submitted_values is not None:
            for i in (0, 1, 5):
                if self._submitted_values[i] == 0:
                    values[i] = 0.
        vx, vy, wz = float(values[0]), float(values[1]), float(values[5])
        if self._rotate_only and (vx or vy):
            self._suppressed += 1
            vx = vy = 0.
        values[0], values[1], values[5] = vx, vy, wz
        problem = self._posture_problem()
        if problem:
            self._refused += 1
            raise RuntimeError(problem)
        if self._paused:
            if not self._transport.resume():
                raise RuntimeError("transport stop not confirmed")
            self._paused = False
            self._auto_resume = False
            self._stop_requested = False
        if self._dry_run:
            self._effective_values = tuple(values)
            self._applied += 1
            return
        # Translate the earliest wall-clock expiry into a monotonic deadline,
        # after all admission checks and immediately before enqueueing.
        cmd = self._command
        wall = time.time() * 1000
        mono = time.monotonic()
        # Permit small cross-host clock skew, not arbitrarily future-dated input.
        if any(isinstance(cmd.get(key), (int, float)) and cmd[key] > wall + 100
               for key in ("stamp_ms", "obs_stamp_ms")):
            raise RuntimeError("command timestamp is too far in the future")
        expiry = cmd["stamp_ms"] + cmd["ttl_ms"]
        if cmd.get("obs_stamp_ms") is not None:
            expiry = min(expiry, cmd["obs_stamp_ms"] + MAX_OBS_AGE_MS)
        deadline = mono + (expiry - wall) / 1000
        if deadline <= time.monotonic():
            raise RuntimeError("command expired before transport submission")
        try:
            # G1's SDK uses a one-second zero command for StopMove; subsecond
            # durations are not reliable. Neither value proves crash-time stop.
            if not self._transport.submit_velocity(vx, vy, wz, 1.0, deadline=deadline):
                raise RuntimeError("transport rejected velocity (paused, failed or expired)")
        except Exception as exc:
            self._sdk_errors += 1
            self._last_error = str(exc)
            raise
        self._physical_dirty = True
        self._effective_values = tuple(values)
        self._applied += 1  # queued, not SDK ACK or physical motion
        self._last_error = ""

    def _watchdog_hold(self, *_args):
        result = self._hold()
        self._auto_resume = result
        return result

    def _hold(self, *_args):
        self._auto_resume = False
        self._holds += 1
        self._paused = True
        if self._sink is not None:
            self._sink._last_values = (0.,) * 6
        try:
            if not self._physical_dirty:
                self._stop_requested = True
                return True
            if not self._transport.pause():
                raise RuntimeError("transport refused stop request")
            self._stop_requested = True
            self._stop_failed = False
            return True
        except Exception as exc:
            self._stop_failed = True
            self._sdk_errors += 1
            self._last_error = str(exc)
            return False
