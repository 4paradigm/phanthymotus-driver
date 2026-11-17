"""Continuous ``motus.control/1`` velocity input for the S10 Pro."""

from __future__ import annotations

import copy
import json
import math
import os
import threading
import time

from common.control import ControlSink, parse_descriptor
from domain_bridge import CoreSubscriber

AXES = ["vx", "vy", "vz", "wx", "wy", "wz"]
VX_LIMIT, VY_LIMIT, WZ_LIMIT = 1.67, 0.4, 1.0
SPEED_PRESETS = {"slow": .15, "normal": .30, "fast": .50}
DEFAULT_SETTINGS = {"dry_run": True, "rotate_only": False, "speed_preset": "normal",
                    "min_vx": 0., "linear_acceleration": .3,
                    "angular_acceleration": 2., "wz_limit": WZ_LIMIT}


def _validated_settings(values):
    unknown = set(values) - set(DEFAULT_SETTINGS)
    if unknown:
        raise ValueError("unsupported loco_servo setting: " + ", ".join(sorted(unknown)))
    result = {**DEFAULT_SETTINGS, **values}
    for key in ("dry_run", "rotate_only"):
        if type(result[key]) is not bool:
            raise ValueError(f"{key} must be a boolean")
    if result["speed_preset"] not in SPEED_PRESETS:
        raise ValueError("speed_preset must be slow, normal or fast")
    for key in ("min_vx", "linear_acceleration", "angular_acceleration", "wz_limit"):
        value = result[key]
        try:
            valid = (not isinstance(value, bool) and isinstance(value, (int, float))
                     and math.isfinite(value) and (value >= 0 if key == "min_vx" else value > 0))
        except OverflowError:
            valid = False
        if not valid:
            raise ValueError(f"{key} must be a finite {'nonnegative' if key == 'min_vx' else 'positive'} number")
        result[key] = float(value)
    if result["min_vx"] > min(SPEED_PRESETS[result["speed_preset"]], VX_LIMIT):
        raise ValueError("min_vx must not exceed the selected forward-speed limit")
    if result["wz_limit"] > WZ_LIMIT:
        raise ValueError("wz_limit must not exceed the chassis yaw limit")
    return result


def build_descriptor(expected_hz=10.0, settings=None):
    settings = _validated_settings(settings or {})
    vx = min(SPEED_PRESETS[settings["speed_preset"]], VX_LIMIT)
    linear_step = settings["linear_acceleration"] / expected_hz
    angular_step = settings["angular_acceleration"] / expected_hz
    return {
        "control_interface": "motus.control/1", "mode": "twist", "dof": 6,
        "joint_names": AXES, "units": {"linear": "m/s", "angular": "rad/s", "time": "s"},
        "limits": {"lower": [-vx, -VY_LIMIT, 0, 0, 0, -WZ_LIMIT],
                   "upper": [vx, VY_LIMIT, 0, 0, 0, WZ_LIMIT],
                   "min_magnitude": [settings["min_vx"], 0, 0, 0, 0, 0],
                   "zero_immediate": True,
                   "max_delta_per_step": [linear_step, linear_step, 1e-6, 1e-6, 1e-6, angular_step]},
        "frame": "base_link",
        "footprint": {"half_width": 0.35, "source": "vendor-spec"},
        "rate": {"max_hz": 20.0, "expected_hz": float(expected_hz), "watchdog_ms": 300, "max_obs_age_ms": 500},
        "force_torque": None,
    }


class _VelocitySink(ControlSink):
    """S10-only velocity stops and confirmed-output slew baseline.

    Leave the shared sink and all other robots unchanged. Only the velocity
    baseline is rebased; sequence, arbitration and watchdog state survive.
    """
    def rebase(self, actual_values):
        self._last_values = tuple(float(value) for value in actual_values)

    def _clamp_step(self, values):
        limited, names = super()._clamp_step(values)
        return (tuple(0. if requested == 0 else value
                      for requested, value in zip(values, limited)),
                [name for name in names if values[AXES.index(name)] != 0])


class S10LocoServoPlugin:
    PREFIX = "loco_servo"

    def __init__(self, native, ros2=None, motion=None, config=None):
        self.native, self.ros2, self.motion = native, ros2, motion
        self.navigation = None
        self._motion_enabled = False
        self._settings = _validated_settings(config or {})
        self._descriptor_raw = build_descriptor(settings=self._settings)
        self._descriptor = parse_descriptor(self._descriptor_raw)
        self._lock = threading.RLock()
        self._lifecycle = threading.RLock()
        self._sink = None
        self._node = None
        self._timer = None
        self._running = False
        self._paused = False
        self._topic = ""
        self._subscriber = None
        self._reader = None
        self._reader_stop = threading.Event()
        self._dry_run = self._settings["dry_run"]
        self._barrier_ms = 0
        self._last_output = [0.] * 6
        self._dry_run_commands = 0
        self._last = None
        self._applied = 0
        self._refused = 0
        self._posture_refused = 0
        self._stop_pending = False
        self._stop_error = None

    def get_tool(self):
        properties = {
            "dry_run": {"type": "boolean", "description": "空跑：检查、整形并计数，不切换厂家模式或发送速度。"},
            "rotate_only": {"type": "boolean", "description": "只转不走：平移清零，保留转向。"},
            "speed_preset": {"type": "string", "enum": list(SPEED_PRESETS),
                             "description": "前后速度上限：slow=0.15、normal=0.30、fast=0.50 m/s；不放大指令，不改变横移/转向上限。修改后重启 navi 读取新上限。"},
            "min_vx": {"type": "number", "minimum": 0, "maximum": self._descriptor_raw["limits"]["upper"][0],
                       "description": "实测最小稳定前后速度，仅向规划器声明，不抬高执行速度；0 表示不声明。"},
            "linear_acceleration": {"type": "number", "exclusiveMinimum": 0,
                                    "description": "平移标称加速度（m/s²），按10Hz换算每帧步长；零速度立即停车。"},
            "angular_acceleration": {"type": "number", "exclusiveMinimum": 0,
                                     "description": "转向标称加速度（rad/s²），按10Hz换算每帧步长。"},
            "wz_limit": {"type": "number", "exclusiveMinimum": 0, "maximum": WZ_LIMIT,
                         "description": "执行端转向限幅（rad/s）；不改变向规划器声明的厂家转向上限。"},
        }
        return {"name": self.PREFIX, "type": "actuator", "multiInstance": False,
                "description": "山猫 S10 Pro 连续速度控制：速度档位、空跑及执行限制，默认空跑。",
                "inputSchema": {"type": "object", "properties": {
                    "action": {"type": "string", "enum": ["start", "stop", "pause", "resume", "info", "config"]},
                    "input_topic": {"type": "string"}, "confirm": {"type": "boolean"},
                    **copy.deepcopy(properties)}, "required": ["action"], "x-is-dangerous": True,
                    "x-action-params": {"start": {"params": ["input_topic", *properties]}, "stop": {"params": []},
                        "pause": {"params": []}, "resume": {"params": []}, "info": {"params": []},
                        "config": {"params": [*properties, "confirm"]}}, "x-resource": "base",
                    "x-hooks": {"on_interrupt_motion": {"action": "pause"},
                                "on_interrupt_all": {"action": "pause"}}},
                "configSchema": {"type": "object", "properties": {
                    key: {**prop, "default": self._dry_run if key == "dry_run" else self._settings[key],
                          "scope": "instance"} for key, prop in properties.items()}},
                "topic_in": [{"format": "control/velocity", "desc": "motus.control/1 twist [vx,vy,vz,wx,wy,wz]"}]}

    def start(self):
        pass

    def stop(self):
        self.dispatch("stop", {})

    def _refresh_motion_permission(self):
        value = os.environ.get("S10_MOTION_ENABLED", "0")
        self._motion_enabled = value == "1"
        return self._motion_enabled, value in {"0", "1"}

    def dispatch(self, action, args):
        if action in ("start", "stop", "pause", "resume", "config"):
            with self._lifecycle:
                if action == "start": return self._start(args)
                if action == "stop": return self._stop()
                if action == "pause": return self._halt(True)
                if action == "resume": return self._halt(False)
                return self._configure(args)
        if action == "info":
            with self._lock:
                return {"state": "paused" if self._paused else "running" if self._running else "idle",
                        "motion_enabled": self._refresh_motion_permission()[0], "input": self._topic,
                        **self._settings, "dry_run": self._dry_run,
                        "vx_limit": self._descriptor_raw["limits"]["upper"][0],
                        "applied": self._applied, "refused": self._refused,
                        "dry_run_commands": self._dry_run_commands, "last_command": copy.deepcopy(self._last),
                        "stop_pending": self._stop_pending, "stop_error": self._stop_error,
                        "control_interface": copy.deepcopy(self._descriptor_raw)}
        return None

    def _configure(self, args):
        try:
            nested = args.get("config", {})
            if not isinstance(nested, dict):
                raise ValueError("config must be an object")
            changes = {**nested, **{k: v for k, v in args.items()
                       if k not in {"config", "confirm", "instance_id", "_tool_name"}}}
            with self._lock:
                candidate = _validated_settings({**self._settings, "dry_run": self._dry_run, **changes})
                if not candidate["dry_run"]:
                    enabled, valid = self._refresh_motion_permission()
                    if not valid or not enabled:
                        return {"ok": False, "code": "INVALID_MOTION_PERMISSION" if not valid else "MOTION_DISABLED",
                                "message": "S10_MOTION_ENABLED must be 1 for real chassis control"}
                    if args.get("confirm") is False:
                        return {"ok": False, "code": "CONFIRM_REQUIRED", "message": "real chassis control was not confirmed"}
                if candidate == {**self._settings, "dry_run": self._dry_run}:
                    return {"ok": True, **candidate}
                raw = build_descriptor(settings=candidate)
                parsed = parse_descriptor(raw)
                if self._running or self._stop_pending:
                    if not self._hold():
                        self._paused = True
                        raise RuntimeError("native stop unconfirmed; settings remain unchanged")
                    if self._running and self._dry_run and not candidate["dry_run"]:
                        self._prepare_motion()
                self._settings, self._dry_run = candidate, candidate["dry_run"]
                self._descriptor_raw, self._descriptor = raw, parsed
                if self._running:
                    self._new_sink()
                self._barrier_ms = int(time.time() * 1000)
                return {"ok": True, **candidate}
        except (ValueError, TypeError, RuntimeError, OSError) as exc:
            return {"ok": False, "code": "INVALID_ARGUMENT", "message": str(exc)}

    def _prepare_motion(self):
        if self.motion is not None and self.motion.is_moving():
            raise ValueError("loco 正在控制底盘，请先停止")
        if self.navigation is not None and self.navigation.is_moving():
            raise ValueError("navigation 正在控制底盘，请先停止")
        ensure_plaintext = getattr(self.native, "ensure_plaintext_transport", None)
        if ensure_plaintext is not None:
            ensure_plaintext()
        for message_type, command, items in (
            (0x00100002, 0x00500002, {"Mode": 1}),
            (0x00100001, 0x00300002, {"GaitParam": 0x3002}),
        ):
            result = self.native.send(message_type, command, items)
            if isinstance(result, dict) and result.get("ErrorCode", 0) not in (0, "0"):
                raise RuntimeError(f"native command rejected: {result['ErrorCode']}")

    def _new_sink(self):
        self._sink = _VelocitySink(self._descriptor, self._apply, on_watchdog=self._hold, on_abort=self._hold)
        self._last_output = [0.] * 6
        self._sink.rebase(self._last_output)

    def _start(self, args):
        if not self._running and self._subscriber is not None:
            stopped = self._stop()
            if stopped["state"] == "error":
                return stopped
        topic = str(args.get("input_topic") or "").strip()
        if not topic: return {"state": "error", "message": "缺少 input_topic，请连接 control/velocity 输出"}
        if self.ros2 is None:
            return {"state": "error", "message": "没有 ROS runtime，无法接入 control/velocity"}
        if self.motion is not None and self.motion.is_moving():
            return {"state": "error", "message": "loco 正在控制底盘，请先停止"}
        if self.navigation is not None and self.navigation.is_moving():
            return {"state": "error", "message": "navigation 正在控制底盘，请先停止"}
        profile = os.environ.get("S10_CORE_FASTDDS_PROFILE", "/tmp/s10-pro-core-fastdds.xml")
        with self._lock:
            if self._running: return {"state": "error", "message": "loco_servo 已在运行"}
            config = self._configure({k: v for k, v in args.items() if k not in {"input_topic", "input_topics"}})
            if not config.get("ok"):
                return {**config, "state": "error"}
            if not self._dry_run:
                try:
                    self._prepare_motion()
                except Exception as exc:
                    return {"state": "error", "message": f"启动导航模式失败: {exc}"}
            self._new_sink()
            self._barrier_ms = int(time.time() * 1000)
            self._topic, self._running, self._paused = topic, True, False
            self._reader_stop.clear()
            try:
                self._subscriber = CoreSubscriber(profile, topic)
                self._reader = threading.Thread(target=self._read_loop, daemon=True, name="s10-loco-servo-reader")
                self._reader.start()
            except Exception as exc:
                self._running = False
                self._sink = None
                self._topic = ""
                return {"state": "error", "message": f"启动 control/velocity 订阅失败: {exc}"}
        return {"state": "ready", "input": topic, "control_interface": self._descriptor_raw}

    def _stop(self):
        with self._lock:
            if (self._running or self._stop_pending) and not self._hold():
                self._paused = True
                return {"state": "error", "message": "native stop unconfirmed; stream remains paused"}
            self._running = self._paused = False
            subscriber, self._subscriber = self._subscriber, None
            reader, self._reader = self._reader, None
            self._reader_stop.set()
            self._sink = None; self._topic = ""
        if subscriber: subscriber.stop()
        if reader and reader is not threading.current_thread(): reader.join(timeout=1)
        return {"state": "idle"}

    def _halt(self, paused):
        with self._lock:
            if not self._running: return {"state": "idle"}
            self._paused = bool(paused)
            if not self._hold():
                self._paused = True
                return {"state": "error", "message": "native stop unconfirmed; stream remains paused"}
            self._barrier_ms = int(time.time() * 1000)
        return {"state": "paused" if paused else "running", "input": self._topic}

    def pause_for_explicit_command(self, _reason=""):
        with self._lock:
            if not self._running or self._paused: return False
            self._paused = True
            self._hold()
            self._barrier_ms = int(time.time() * 1000)
        return True

    def attach_navigation(self, plugin):
        self.navigation = plugin

    def _on_message(self, message):
        with self._lock:
            if self._running and not self._paused and self._sink:
                try: self._submit(json.loads(message.data))
                except (TypeError, ValueError): self._refused += 1

    def _submit(self, command):
        if isinstance(command, dict):
            stamp = command.get("stamp_ms")
            if isinstance(stamp, (int, float)) and stamp <= self._barrier_ms:
                self._last = {"verdict": "DROPPED", "reason": "command predates start/resume/config"}
                return
        sink, refused = self._sink, self._refused
        outcome = sink.submit(command)
        if self._sink is sink:
            sink.rebase(self._last_output)
        self._last = {"verdict": outcome.verdict.name if self._refused == refused else "REFUSED",
                      "reason": outcome.reason, "output": list(self._last_output),
                      "dry_run": self._dry_run}

    def _read_loop(self):
        while not self._reader_stop.wait(0.05):
            if self._stop_pending:
                self._tick()
                continue
            if not self._running:
                return
            subscriber = self._subscriber
            if subscriber is None:
                return
            if not subscriber._process.is_alive():
                with self._lock:
                    if self._subscriber is subscriber:
                        self._refused += 1
                        self._disarm()
                continue
            payload = subscriber.latest()
            if payload is not None:
                with self._lock:
                    if self._running and not self._paused and self._sink:
                        try: self._submit(json.loads(payload))
                        except (TypeError, ValueError): self._refused += 1
                        except Exception as exc:
                            # Keep the stop retry loop alive after a native send
                            # fault has already disarmed command admission.
                            self._last = {"verdict": "ERROR", "reason": str(exc)}
            self._tick()

    def _tick(self):
        with self._lock:
            if self._stop_pending:
                if self._hold() and not self._running:
                    self._reader_stop.set()
            elif self._running and not self._paused and self._sink:
                self._sink.tick()

    def _apply(self, values, _gripper=None):
        values = list(values)
        if self._settings["rotate_only"]:
            values[0] = values[1] = 0.
        values[5] = max(-self._settings["wz_limit"], min(self._settings["wz_limit"], values[5]))
        standing_problem = getattr(self.motion, "standing_problem", None)
        if standing_problem is not None:
            problem = standing_problem()
            if problem:
                self._refused += 1
                self._posture_refused += 1
                if self._posture_refused == 1 or self._posture_refused % 100 == 0:
                    print(f"[loco_servo] 拒绝速度命令（第 {self._posture_refused} 条）：{problem}",
                          flush=True)
                self._hold()
                return
        self._posture_refused = 0
        if self._dry_run:
            self._last_output = values
            self._applied += 1
            self._dry_run_commands += 1
            return
        enabled, valid = self._refresh_motion_permission()
        if not valid or not enabled:
            self._refused += 1
            self._disarm()
            return
        try:
            ret = self.native.send(0x00100001, 0x00110002,
                                   {"X": float(values[0]), "Y": float(values[1]), "Z": 0.0,
                                    "Roll": 0.0, "Pitch": 0.0, "Yaw": float(values[5])})
            if isinstance(ret, dict) and ret.get("ErrorCode", 0) not in (0, "0"):
                raise RuntimeError(f"native velocity command rejected: {ret.get('ErrorCode')}")
            self._last_output = values
            self._applied += 1
        except Exception:
            self._refused += 1
            # A failed send cannot satisfy ControlSink's watchdog. Hold the
            # chassis and disarm the stream before another non-zero command.
            self._disarm()
            raise

    def _disarm(self):
        with self._lock:
            self._running, self._paused = False, True
            self._hold()
            self._sink = None
            if not self._stop_pending:
                self._reader_stop.set()

    def _hold(self, *_args):
        if not self._dry_run or self._stop_pending:
            try:
                result = self.native.send(0x00100001, 0x00110002,
                    {"X": 0.0, "Y": 0.0, "Z": 0.0, "Roll": 0.0, "Pitch": 0.0, "Yaw": 0.0})
                if isinstance(result, dict) and result.get("ErrorCode", 0) not in (0, "0"):
                    raise RuntimeError(f"native stop rejected: {result['ErrorCode']}")
            except Exception as exc:
                self._stop_pending, self._stop_error = True, str(exc)
                self._paused = True
                return False
        self._stop_pending, self._stop_error = False, None
        self._last_output = [0.] * 6
        if self._sink is not None:
            self._sink.rebase(self._last_output)
        return True
