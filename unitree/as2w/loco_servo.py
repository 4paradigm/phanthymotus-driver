"""Continuous motus.control/1 twist input for the Unitree As2W chassis."""

from __future__ import annotations

import json
import math
import threading
import time

from common.control import ControlSink, parse_descriptor

AXIS_NAMES = ["vx", "vy", "vz", "wx", "wy", "wz"]
DOF = 6
DEFAULT_EXPECTED_HZ = 10.0
MAX_HZ = 20.0
WATCHDOG_MS = 300
MAX_OBS_AGE_MS = 500
PINNED_STEP = 1e-6

# Conservative initial operating envelope. These are policy limits, not
# measured hardware maxima; the SDK's broader limits remain enforced by loco.
DEFAULT_MAX_VX = 0.4
DEFAULT_MAX_VY = 0.25
DEFAULT_MAX_WZ = 0.5
DEFAULT_ACCEL_VX = 0.15
DEFAULT_ACCEL_VY = 0.1
DEFAULT_ACCEL_WZ = 0.2
STANDING_STATES = {
    "STAND_UP", "BALANCE_STAND", "RECOVERY_STAND", "STANDING",
    "AI_STAND_UP", "AI_BALANCE_STAND", "AI_RECOVERY_STAND",
}
MOVING_STATES = {
    "WALK", "WALKING", "RUN", "RUNNING", "MOVE", "MOVING",
    "REGULAR_WALK", "REGULAR_RUN", "AI_FREE_WALK", "AI_WALK", "AI_RUN",
}


def build_descriptor(config: dict | None = None) -> dict:
    config = config or {}
    max_vx = float(config.get("max_vx", DEFAULT_MAX_VX))
    max_vy = float(config.get("max_vy", DEFAULT_MAX_VY))
    max_wz = float(config.get("max_wz", DEFAULT_MAX_WZ))
    accel_vx = float(config.get("accel_vx", DEFAULT_ACCEL_VX))
    accel_vy = float(config.get("accel_vy", DEFAULT_ACCEL_VY))
    accel_wz = float(config.get("accel_wz", DEFAULT_ACCEL_WZ))
    expected_hz = float(config.get("expected_hz", DEFAULT_EXPECTED_HZ))

    for name, value in (
        ("max_vx", max_vx), ("max_vy", max_vy), ("max_wz", max_wz),
        ("accel_vx", accel_vx), ("accel_vy", accel_vy),
        ("accel_wz", accel_wz),
    ):
        if not math.isfinite(value) or not 0 < value:
            raise ValueError(f"loco_servo.{name} must be positive")
    if max_vx > 1.5 or max_vy > 1.0 or max_wz > 2.0:
        raise ValueError("loco_servo velocity limits exceed the As2W loco tool limits")
    if not 0 < expected_hz <= MAX_HZ:
        raise ValueError(f"loco_servo.expected_hz must be in (0, {MAX_HZ}]")

    raw = {
        "control_interface": "motus.control/1",
        "mode": "twist",
        "dof": DOF,
        "joint_names": list(AXIS_NAMES),
        "units": {"linear": "m/s", "angular": "rad/s", "time": "s"},
        "limits": {
            "lower": [-max_vx, -max_vy, 0.0, 0.0, 0.0, -max_wz],
            "upper": [max_vx, max_vy, 0.0, 0.0, 0.0, max_wz],
            "max_delta_per_step": [
                accel_vx, accel_vy, PINNED_STEP, PINNED_STEP, PINNED_STEP,
                accel_wz,
            ],
            "min_magnitude": [0.0] * DOF,
        },
        "groups": [{"name": "base", "offset": 0, "count": DOF,
                    "unit": "m/s", "resource": "base"}],
        "frame": "base_link",
        "rate": {
            "max_hz": MAX_HZ,
            "expected_hz": expected_hz,
            "watchdog_ms": WATCHDOG_MS,
            "max_obs_age_ms": MAX_OBS_AGE_MS,
        },
        "force_torque": None,
    }
    parse_descriptor(raw)
    return raw


class LocoServoPlugin:
    """Consume one velocity stream and map the planar twist to As2W Move."""

    PREFIX = "locoservo"

    def __init__(self, config: dict, namespace: str, executor, proxy,
                 loco_plugin=None, special_motion_plugin=None):
        self._config = dict(config or {})
        self._namespace = namespace
        self._executor = executor
        self._proxy = proxy
        self._loco = loco_plugin
        self._special_motion = special_motion_plugin
        self._dry_run = bool(self._config.get("dry_run", True))
        self._require_standing = bool(self._config.get("require_standing", True))
        self._descriptor_raw = build_descriptor(self._config)
        self._descriptor = parse_descriptor(self._descriptor_raw)
        self._lock = threading.RLock()
        self._sink = None
        self._node = None
        self._timer = None
        self._input_topic = ""
        self._running = False
        self._paused = False
        self._last_command = None
        self._last_fsm_check = float("-inf")
        self._fsm_problem = ""
        self._applied = 0
        self._refused = 0
        self._sdk_errors = 0
        self._holds = 0
        self._last_ret = 0
        if self._loco is not None:
            self._loco.attach_servo(self)

    def get_tool(self) -> dict:
        return {
            "name": "loco_servo",
            "type": "actuator",
            "multiInstance": False,
            "description": (
                "AS2W 连续底盘速度控制：接收 motus.control/1 的 6 维 twist "
                "数据流，并映射为底盘 Move(vx, vy, vyaw)。"
            ),
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {"type": "string",
                               "enum": ["start", "stop", "pause", "resume",
                                        "info", "config"]},
                    "input_topic": {"type": "string",
                                    "description": "control/velocity 指令话题"},
                    "dry_run": {"type": "boolean"},
                    "require_standing": {"type": "boolean"},
                },
                "required": ["action"],
                "x-action-params": {
                    "pause": {"params": [],
                              "description": "立即调用 StopMove 并暂停流控"},
                    "resume": {"params": [], "description": "恢复接收速度指令"},
                    "config": {"params": ["dry_run", "require_standing"],
                               "description": "更新空跑和站立状态保护设置"},
                },
                "x-hooks": {"on_interrupt_motion": {"action": "pause"},
                            "on_interrupt_all": {"action": "pause"}},
                "x-is-dangerous": True,
                "x-resource": ["base"],
            },
            "topic_in": [{"format": "control/velocity",
                          "desc": "motus.control/1 twist [vx, vy, vz, wx, wy, wz]"}],
            "configSchema": {
                "type": "object",
                "properties": {
                    "dry_run": {
                        "type": "boolean",
                        "default": True,
                        "description": "空跑模式：检查并记录指令，但不调用电机。"
                                       "确认方向和速度安全后才显式关闭。",
                    },
                    "require_standing": {
                        "type": "boolean",
                        "default": True,
                        "description": "仅在 SportClient 报告机器人已站立或行走时执行。",
                    },
                },
            },
        }

    def start(self):
        # A restarted driver must never resume a prior motion stream by itself.
        return None

    def stop(self):
        return self._stop()

    def dispatch(self, action: str, args: dict):
        if action == "start":
            return self._start(args)
        if action == "stop":
            return self._stop()
        if action == "pause":
            return self._pause()
        if action == "resume":
            return self._resume()
        if action == "info":
            return self._info()
        if action == "config":
            return self._configure(args)
        return None

    def _start(self, args: dict) -> dict:
        topic = (args.get("input_topic") or "").strip()
        if not topic:
            topics = args.get("input_topics") or []
            topic = (topics[0] or "").strip() if topics else ""
        if not topic:
            return {"state": "error",
                    "message": "缺少 input_topic；请连接 control/velocity 输出"}
        if self._executor is None:
            return {"state": "error", "message": "ROS executor 不可用"}
        if self._loco is not None and self._loco.is_moving():
            return {"state": "error",
                    "message": "loco 正在控制底盘；先调用 loco.stop_move 再启动流控"}
        if (self._special_motion is not None
                and self._special_motion.is_running()):
            return {"state": "error",
                    "message": "special_motion 动作正在执行；完成后再启动流控"}
        with self._lock:
            if self._running:
                return {"state": "error",
                        "message": f"loco_servo 已运行（{self._input_topic}）"}
            self._sink = ControlSink(self._descriptor, self._apply,
                                     on_watchdog=self._hold,
                                     on_abort=self._hold)
            self._input_topic = topic
            self._running = True
            self._paused = False
        try:
            self._open(topic)
        except Exception as exc:
            with self._lock:
                self._running = False
                self._sink = None
                self._input_topic = ""
            return {"state": "error",
                    "message": f"loco_servo 订阅启动失败：{type(exc).__name__}: {exc}"}
        return {"state": "running", "input": topic, "dry_run": self._dry_run,
                "control_interface": self._interface()}

    def _open(self, topic: str) -> None:
        from rclpy.node import Node
        from std_msgs.msg import String

        node = Node(f"as2w_loco_servo_{abs(hash(topic)) % 100000}")
        try:
            node.create_subscription(String, topic, self._on_message, 1)
            timer = node.create_timer(WATCHDOG_MS / 2000.0, self._tick)
            self._executor.add_node(node)
        except Exception:
            node.destroy_node()
            raise
        with self._lock:
            self._node = node
            self._timer = timer

    def _on_message(self, message) -> None:
        try:
            command = json.loads(message.data)
        except (TypeError, json.JSONDecodeError) as exc:
            self._record("REJECTED", f"invalid JSON: {exc}")
            return
        if not isinstance(command, dict):
            self._record("REJECTED", "control message must be a JSON object")
            return
        with self._lock:
            sink = self._sink
            if sink is None or self._paused:
                return
            if not self._dry_run and self._require_standing:
                problem = self._posture_problem()
                if problem:
                    self._refused += 1
                    self._record("REFUSED", problem)
                    self._hold()
                    return
            outcome = sink.submit(command)
            if outcome is not None:
                self._record(getattr(outcome.verdict, "name", str(outcome.verdict)),
                             getattr(outcome, "reason", ""))

    def _tick(self) -> None:
        with self._lock:
            sink = self._sink
            if sink is not None and not self._paused:
                outcome = sink.tick()
                if outcome is not None:
                    self._record(
                        getattr(outcome.verdict, "name", str(outcome.verdict)),
                        getattr(outcome, "reason", ""))

    def _apply(self, values, _gripper=None) -> None:
        with self._lock:
            vx, vy, vyaw = float(values[0]), float(values[1]), float(values[5])
            self._applied += 1
            if self._dry_run:
                if self._applied == 1 or self._applied % 100 == 0:
                    print(f"[loco_servo] DRY RUN Move #{self._applied} "
                          f"(vx={vx:+.3f}, vy={vy:+.3f}, "
                          f"vyaw={vyaw:+.3f})", flush=True)
                return
            try:
                ret = self._proxy.Move(vx, vy, vyaw)
            except Exception as exc:
                self._sdk_errors += 1
                self._last_ret = 3104
                self._record("SDK_ERROR", f"{type(exc).__name__}: {exc}")
                self._hold()
                return
            self._last_ret = ret
            if ret != 0:
                self._sdk_errors += 1
                self._record("SDK_ERROR", f"Move returned {ret}")
                self._hold()
            else:
                self._sdk_errors = 0

    def _hold(self, *_args) -> None:
        with self._lock:
            self._holds += 1
            if self._dry_run:
                print("[loco_servo] DRY RUN StopMove()", flush=True)
                return
            try:
                ret = self._proxy.StopMove()
            except Exception as exc:
                self._record("STOP_ERROR", f"{type(exc).__name__}: {exc}")
                print(f"[loco_servo] StopMove failed: {type(exc).__name__}: {exc}",
                      flush=True)
                return
            self._last_ret = ret
            if ret != 0:
                self._record("STOP_ERROR", f"StopMove returned {ret}")
                print(f"[loco_servo] StopMove rejected: ret={ret}", flush=True)
            elif self._loco is not None:
                self._loco.servo_stopped()

    def _posture_problem(self) -> str:
        now = time.monotonic()
        if now - self._last_fsm_check < 0.5:
            return self._fsm_problem
        self._last_fsm_check = now
        try:
            result = self._proxy.GetState()
        except Exception as exc:
            self._fsm_problem = f"无法读取底盘状态：{type(exc).__name__}: {exc}"
            return self._fsm_problem
        if not isinstance(result, tuple) or len(result) != 2:
            self._fsm_problem = "SportClient.GetState 返回格式无效"
            return self._fsm_problem
        code, state = result
        name = str(state.get("fsm_name", "")).strip().upper() if isinstance(state, dict) else ""
        if code != 0 or not name:
            self._fsm_problem = "无法确认机器人姿态，拒绝速度指令"
        elif name not in STANDING_STATES and name not in MOVING_STATES:
            self._fsm_problem = f"机器人状态 {name} 不允许底盘速度控制"
        else:
            self._fsm_problem = ""
        return self._fsm_problem

    def _record(self, verdict: str, reason: str) -> None:
        self._last_command = {"verdict": str(verdict), "reason": reason,
                             "at": time.time()}

    def _pause(self) -> dict:
        with self._lock:
            if not self._running:
                return {"state": "idle", "message": "loco_servo 未运行"}
            self._paused = True
        self._hold()
        return {"state": "paused", "input": self._input_topic}

    def _resume(self) -> dict:
        with self._lock:
            if not self._running:
                return {"state": "idle", "message": "loco_servo 未运行"}
            self._paused = False
        return {"state": "running", "input": self._input_topic}

    def pause_for_explicit_command(self, reason: str = "") -> bool:
        with self._lock:
            if not self._running or self._paused:
                return False
            self._paused = True
        print(f"[loco_servo] paused by loco command{': ' + reason if reason else ''}",
              flush=True)
        self._hold()
        return True

    def _stop(self) -> dict:
        with self._lock:
            if not self._running:
                return {"state": "idle"}
            self._running = False
            self._paused = False
            node, timer = self._node, self._timer
            self._node = None
            self._timer = None
            self._sink = None
        self._hold()
        if timer is not None and node is not None:
            node.destroy_timer(timer)
        if node is not None:
            self._executor.remove_node(node)
            node.destroy_node()
        return {"state": "stopped"}

    def _configure(self, args: dict) -> dict:
        with self._lock:
            if args.get("dry_run") is False and self._dry_run:
                print("[loco_servo] dry_run disabled; live chassis control is enabled",
                      flush=True)
            elif args.get("dry_run") is True and not self._dry_run:
                self._hold()
            if "dry_run" in args:
                self._dry_run = bool(args["dry_run"])
            if "require_standing" in args:
                self._require_standing = bool(args["require_standing"])
            return {"ok": True, **self._info()}

    def _interface(self) -> dict:
        descriptor = dict(self._descriptor_raw)
        if self._dry_run:
            descriptor["dry_run"] = True
        return descriptor

    def _info(self) -> dict:
        with self._lock:
            state = ("paused" if self._running and self._paused else
                     "running" if self._running else "idle")
            topic = self._input_topic
        return {
            "state": state,
            "input": topic,
            "dry_run": self._dry_run,
            "require_standing": self._require_standing,
            "applied": self._applied,
            "refused": self._refused,
            "sdk_errors": self._sdk_errors,
            "holds": self._holds,
            "last_ret": self._last_ret,
            "last": self._last_command,
            "posture_problem": self._fsm_problem,
            "control_interface": self._interface(),
        }
