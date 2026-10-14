#!/usr/bin/env python3
"""Continuous ``motus.control/1`` twist input for the Unitree AS2W chassis.

``loco`` is call-shaped: one MCP call asks the robot to move.  Visual
navigation is stream-shaped and publishes a fresh velocity command every tick.
This card is that data-plane sink.  It deliberately shares the same RpcProxy as
``loco`` and explicitly arbitrates with it, because both ultimately call the
same SportClient.Move service.
"""

from __future__ import annotations

import json
import math
import threading
import time

from common.control import ControlSink, parse_descriptor


VX_LIMIT = 1.5
VY_LIMIT = 1.0
WZ_LIMIT = 2.0

# Conservative command-step limits.  For twist these are acceleration limits,
# not joint velocity limits.  AS2W's actual no-motion thresholds have not been
# measured, so no ``min_magnitude`` is declared yet; copying R1's foot-gait
# thresholds onto a wheel-legged chassis would be a false hardware claim.
VX_STEP = 0.15
VY_STEP = 0.15
WZ_STEP = 0.30
PINNED_STEP = 1e-6

DEFAULT_EXPECTED_HZ = 10.0
MAX_HZ = 20.0
WATCHDOG_MS = 300
MAX_OBS_AGE_MS = 500
FSM_CACHE_S = 1.0

STANDING_STATES = {
    "STAND_UP", "BALANCE_STAND", "RECOVERY_STAND", "STANDING",
    "AI_STAND_UP", "AI_BALANCE_STAND", "AI_RECOVERY_STAND",
}
MOVING_STATES = {
    "WALK", "WALKING", "RUN", "RUNNING", "MOVE", "MOVING",
    "REGULAR_WALK", "REGULAR_RUN", "AI_FREE_WALK", "AI_WALK", "AI_RUN",
}

AXIS_NAMES = ["vx", "vy", "vz", "wx", "wy", "wz"]


def build_descriptor(expected_hz=DEFAULT_EXPECTED_HZ):
    """Return AS2W's bounded continuous chassis action space."""
    return {
        "control_interface": "motus.control/1",
        "mode": "twist",
        "dof": 6,
        "joint_names": list(AXIS_NAMES),
        "units": {"linear": "m/s", "angular": "rad/s", "time": "s"},
        "limits": {
            "lower": [-VX_LIMIT, -VY_LIMIT, 0.0, 0.0, 0.0, -WZ_LIMIT],
            "upper": [VX_LIMIT, VY_LIMIT, 0.0, 0.0, 0.0, WZ_LIMIT],
            "max_delta_per_step": [VX_STEP, VY_STEP, PINNED_STEP,
                                   PINNED_STEP, PINNED_STEP, WZ_STEP],
        },
        "groups": [{"name": "base", "offset": 0, "count": 6,
                    "unit": "m/s", "resource": "base"}],
        "frame": "base_link",
        "rate": {
            "max_hz": MAX_HZ,
            "expected_hz": float(expected_hz),
            "watchdog_ms": WATCHDOG_MS,
            "max_obs_age_ms": MAX_OBS_AGE_MS,
        },
        "force_torque": None,
    }


class LocoServoPlugin:
    """Subscribe to one velocity stream and safely apply it to SportClient."""

    PREFIX = "locoservo"

    def __init__(self, plugin_config, namespace, executor, proxy,
                 loco_plugin=None):
        config = plugin_config or {}
        self._namespace = namespace
        self._executor = executor
        self._proxy = proxy
        self._loco = loco_plugin
        self._expected_hz = float(
            config.get("expected_hz", DEFAULT_EXPECTED_HZ))
        if not 0 < self._expected_hz <= MAX_HZ:
            raise ValueError(
                f"loco_servo.expected_hz must be in (0, {MAX_HZ}]")

        self._dry_run = bool(config.get("dry_run", False))
        self._rotate_only = bool(config.get("rotate_only", False))
        self._require_standing = bool(config.get("require_standing", True))
        self._descriptor_raw = build_descriptor(self._expected_hz)
        self._descriptor = parse_descriptor(self._descriptor_raw)

        self._lock = (loco_plugin.motion_lock if loco_plugin is not None
                      else threading.RLock())
        self._sink = None
        self._node = None
        self._ticker = None
        self._input_topic = ""
        self._running = False
        self._paused = False
        self._applied = 0
        self._holds = 0
        self._refused = 0
        self._sdk_errors = 0
        self._last_ret = 0
        self._last_command = None
        self._posture_error = ""
        self._posture_checked_at = float("-inf")

        if loco_plugin is not None:
            loco_plugin.attach_servo(self)

    def get_tool(self):
        return {
            "name": "loco_servo",
            "type": "actuator",
            "multiInstance": False,
            "description": (
                "AS2W continuous chassis velocity input for navigation and "
                "execution policies. Consumes motus.control/1 twist commands."),
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {"type": "string", "enum": [
                        "start", "stop", "pause", "resume", "info", "config"]},
                    "input_topic": {"type": "string"},
                },
                "required": ["action"],
                "x-action-params": {
                    "pause": {"params": [], "description": "Stop and pause"},
                    "resume": {"params": [], "description": "Resume input"},
                },
                "x-hooks": {
                    "on_interrupt_motion": {"action": "pause"},
                    "on_interrupt_all": {"action": "pause"},
                },
                "x-is-dangerous": True,
                "x-resource": ["base"],
            },
            "configSchema": {
                "type": "object",
                "properties": {
                    "dry_run": {"type": "boolean", "default": False,
                                "description": "Validate commands without moving"},
                    "rotate_only": {"type": "boolean", "default": False,
                                    "description": "Suppress vx/vy; keep yaw"},
                    "require_standing": {"type": "boolean", "default": True,
                                         "description": "Require a standing/walking FSM"},
                },
            },
            "topic_in": [{
                "format": "control/velocity",
                "desc": "motus.control/1 twist [vx,vy,vz,wx,wy,wz]",
            }],
        }

    def dispatch(self, action, args):
        if action == "start":
            return self._start(args)
        if action == "stop":
            return self._stop()
        if action == "pause":
            return self._halt(True)
        if action == "resume":
            return self._halt(False)
        if action == "info":
            return self._info()
        if action == "config":
            return self._config(args)
        return None

    def start(self):
        # Bundle startup must never make the chassis move.  Canvas start supplies
        # an input topic and is the deliberate action that opens the stream.
        pass

    def stop(self):
        self._stop()

    def _interface(self):
        out = dict(self._descriptor_raw)
        if self._dry_run:
            out["dry_run"] = True
        if self._rotate_only:
            out["rotate_only"] = True
        return out

    def _start(self, args):
        with self._lock:
            return self._start_locked(args)

    def _start_locked(self, args):
        topic = str(args.get("input_topic") or "").strip()
        if not topic:
            topics = args.get("input_topics") or []
            topic = str(topics[0] if topics else "").strip()
        if not topic:
            return {"state": "error", "message":
                    "缺少 input_topic，请连接 control/velocity 输出"}
        if self._loco is not None and self._loco.is_moving():
            return {"state": "error", "message":
                    "loco 正在占用底盘，请先调用 stop_move"}
        if self._executor is None:
            return {"state": "error", "message": "没有 ROS 上下文，无法订阅"}

        sink = ControlSink(self._descriptor, self._apply,
                           on_watchdog=self._hold, on_abort=self._hold)
        with self._lock:
            if self._running:
                return {"state": "error",
                        "message": f"已经在运行（{self._input_topic}）"}
            self._sink = sink
            self._input_topic = topic
            self._running = True
            self._paused = False
            self._posture_checked_at = float("-inf")
        try:
            self._open(topic)
        except Exception as exc:
            with self._lock:
                self._running = False
                self._sink = None
                self._input_topic = ""
            return {"state": "error", "message": f"启动失败: {exc}"}
        return {"state": "running", "input": topic,
                "control_interface": self._interface()}

    def _stop(self):
        with self._lock:
            if not self._running:
                return {"state": "idle"}
            if not self._paused:
                self._hold()
            self._running = False
            self._paused = False
            self._sink = None
            node, ticker = self._node, self._ticker
            self._node = None
            self._ticker = None
        if node is not None and ticker is not None:
            try:
                node.destroy_timer(ticker)
            except Exception:
                pass
        if node is not None:
            try:
                self._executor.remove_node(node)
                node.destroy_node()
            except Exception:
                pass
        return {"state": "stopped"}

    def _halt(self, paused):
        with self._lock:
            if not self._running:
                return {"state": "idle"}
            if not paused and self._loco is not None and self._loco.is_moving():
                return {"state": "error", "message":
                        "loco 正在占用底盘，请先调用 stop_move"}
            self._paused = bool(paused)
            if paused:
                self._hold()
            else:
                self._posture_checked_at = float("-inf")
        return {"state": "paused" if paused else "running",
                "input": self._input_topic}

    def pause_for_explicit_command(self, reason=""):
        with self._lock:
            if not self._running or self._paused:
                return False
            self._paused = True
            self._hold()
        print(f"[loco_servo] paused by explicit command: {reason}", flush=True)
        return True

    def _config(self, args):
        with self._lock:
            if (args.get("dry_run") is True and not self._dry_run
                    and self._running and not self._paused):
                self._hold()
            self._posture_checked_at = float("-inf")
            for name in ("dry_run", "rotate_only", "require_standing"):
                if name in args:
                    setattr(self, f"_{name}", bool(args[name]))
            state = ("paused" if self._running and self._paused else
                     "running" if self._running else "idle")
        return {"ok": True, "state": state,
                "dry_run": self._dry_run,
                "rotate_only": self._rotate_only,
                "require_standing": self._require_standing}

    def _info(self):
        with self._lock:
            return {
                "state": ("paused" if self._running and self._paused else
                          "running" if self._running else "idle"),
                "input": self._input_topic,
                "dry_run": self._dry_run,
                "rotate_only": self._rotate_only,
                "require_standing": self._require_standing,
                "applied": self._applied,
                "holds": self._holds,
                "refused": self._refused,
                "sdk_errors": self._sdk_errors,
                "last_ret": self._last_ret,
                "posture_problem": self._posture_error,
                "last": self._last_command,
                "control_interface": self._interface(),
            }

    def _open(self, topic):
        from rclpy.node import Node
        from std_msgs.msg import String

        node = Node(f"as2w_loco_servo_{abs(hash(topic)) % 100000}")
        try:
            node.create_subscription(String, topic, self._on_message, 1)
            ticker = node.create_timer(WATCHDOG_MS / 2000.0, self._tick)
            self._executor.add_node(node)
        except Exception:
            try:
                self._executor.remove_node(node)
            finally:
                node.destroy_node()
            raise
        self._ticker = ticker
        self._node = node

    def _on_message(self, message):
        # Serialize submit/apply, watchdog, pause and shutdown. A callback that
        # started before preemption must finish before the explicit command.
        with self._lock:
            sink = self._sink
            if sink is None or not self._running or self._paused:
                return
            try:
                command = json.loads(message.data)
            except (TypeError, ValueError):
                return
            values = command.get("values") if isinstance(command, dict) else None
            if isinstance(values, (list, tuple)) and any(
                    isinstance(value, (int, float)) and not math.isfinite(value)
                    for value in values):
                self._last_command = {"verdict": "REJECTED",
                                      "reason": "values must be finite",
                                      "at": time.time()}
                return
            self._last_command = None
            outcome = sink.submit(command)
            if self._last_command is None:
                self._last_command = {
                    "verdict": str(getattr(outcome.verdict, "name", outcome.verdict)),
                    "reason": outcome.reason,
                    "at": time.time(),
                }

    def _tick(self):
        with self._lock:
            if self._sink is not None and self._running and not self._paused:
                self._sink.tick()

    def _posture_problem(self):
        if not self._require_standing:
            return ""
        now = time.monotonic()
        if now - self._posture_checked_at < FSM_CACHE_S:
            return self._posture_error
        self._posture_checked_at = now
        try:
            code, state = self._proxy.GetState()
        except Exception as exc:
            self._posture_error = f"读不到运动状态（{exc}）"
            return self._posture_error
        if code != 0 or not isinstance(state, dict):
            self._posture_error = f"读不到运动状态（code={code}）"
            return self._posture_error
        name = str(state.get("fsm_name", "")).strip().upper()
        standing = (name in STANDING_STATES or name in MOVING_STATES or
                    "WALK" in name or "RUN" in name or "MOVE" in name or
                    name.endswith("FREE_WALK"))
        self._posture_error = ("" if standing else
                               f"机器人当前状态 {name or 'UNKNOWN'} 不允许移动")
        return self._posture_error

    def _apply(self, values, _gripper=None):
        with self._lock:
            if not self._running or self._paused:
                return
            vx, vy, wz = float(values[0]), float(values[1]), float(values[5])
            if self._rotate_only:
                vx = vy = 0.0
            problem = self._posture_problem()
            if problem:
                self._refused += 1
                self._last_command = {"verdict": "REFUSED", "reason": problem,
                                      "at": time.time()}
                self._hold()
                return
            if self._dry_run:
                self._applied += 1
                return
            try:
                ret = self._proxy.Move(vx, vy, wz)
            except Exception as exc:
                ret = 3104
                problem = f"Move RPC failed: {exc}"
            self._last_ret = ret
            if ret != 0:
                self._sdk_errors += 1
                self._last_command = {"verdict": "SDK_ERROR",
                                      "reason": problem or f"Move returned {ret}",
                                      "at": time.time()}
                self._hold()
            else:
                self._applied += 1
                self._sdk_errors = 0

    def _hold(self, *_args):
        with self._lock:
            # A paused policy no longer owns the chassis. Its watchdog or
            # shutdown must not stop a subsequent explicit loco movement.
            if self._loco is not None and self._loco.is_moving():
                return
            self._holds += 1
            if self._dry_run:
                return
            try:
                ret = self._proxy.StopMove()
                if ret != 0:
                    self._last_ret = ret
                    self._sdk_errors += 1
            except Exception as exc:
                self._last_ret = 3104
                self._sdk_errors += 1
                print(f"[loco_servo] StopMove failed: {exc}", flush=True)
