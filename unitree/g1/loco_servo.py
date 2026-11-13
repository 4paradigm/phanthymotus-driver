#!/usr/bin/env python3
"""G1 motus.control/1 twist stream with the R1 Move/StopMove semantics.

ControlSink handles freshness, limits and a 300ms software watchdog. Each live
command calls the shared RPC client's continuous Move (SDK duration 864000s).
StopMove is best effort: a failed stop does not latch or prevent the next valid
command. This is not SmartMotion, obstacle avoidance, or cross-card arbitration.
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

    def __init__(self, plugin_config, namespace, executor, loco_client, fsm_reader):
        config = plugin_config or {}
        self._ns, self._executor, self._client = namespace, executor, loco_client
        self._fsm_reader = fsm_reader
        self._expected_hz = float(config.get("expected_hz", DEFAULT_EXPECTED_HZ))
        if not math.isfinite(self._expected_hz) or not 0 < self._expected_hz <= MAX_HZ:
            raise ValueError(f"loco_servo.expected_hz must be in (0, {MAX_HZ}]")
        self._dry_run = bool(config.get("dry_run", True))
        self._rotate_only = bool(config.get("rotate_only", False))
        self._require_standing = bool(config.get("require_standing", True))
        self._descriptor_raw = build_descriptor(self._expected_hz)
        self._descriptor = parse_descriptor(self._descriptor_raw)
        self._lock = threading.RLock()
        self._sink = self._sub_node = self._ticker = None
        self._running = self._paused = False
        self._shutdown = self._stopping = False
        self._input_topic = ""
        self._last_command = None
        self._applied = self._holds = self._refused = self._sdk_errors = 0
        self._suppressed = 0
        self._last_ret = 0
        self._fsm_problem = ""

    def get_tool(self):
        return {"name": "loco_servo", "type": "actuator", "multiInstance": False,
                "description": "G1 连续速度控制，直发 SDK；不经过 SmartMotion，无避障或跨卡互斥。",
                "inputSchema": {"type": "object", "properties": {
                    "action": {"type": "string", "enum": ["start", "stop", "pause", "resume", "info"]},
                    "input_topic": {"type": "string", "description": "control/velocity 指令流话题"}},
                    "required": ["action"],
                    "x-action-params": {"pause": {"params": [], "description": "请求停止底盘并保留订阅"},
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
        with self._lock:
            self._shutdown = True
        return self._stop()

    def _config(self, args):
        with self._lock:
            for key in self._TOGGLES:
                if key in args:
                    setattr(self, "_" + key, bool(args[key]))
            return {"ok": True, "state": self._state(),
                    **{key: getattr(self, "_" + key) for key in self._TOGGLES}}

    def _state(self):
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
            if self._shutdown or self._stopping:
                return {"state": "error", "message": "驱动正在关闭或停止"}
            if self._running:
                return {"state": "error", "message": f"已经在运行（{self._input_topic}）"}
            self._sink = ControlSink(self._descriptor, self._apply,
                                     on_watchdog=self._hold, on_abort=self._hold)
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
            self._paused = bool(halted)
        if halted:
            self._hold()
        return {"state": "paused" if halted else "running", "input": self._input_topic,
                "physical_stop_confirmed": False}

    def _stop(self):
        with self._lock:
            if self._stopping:
                return {"state": "error", "message": "停止调用仍在进行，物理停止未知"}
            if not self._running:
                return {"state": "idle"}
            self._stopping = True
            self._running = self._paused = False
            self._sink = None
            node, ticker = self._sub_node, self._ticker
            self._sub_node = self._ticker = None
        try:
            self._hold()
        finally:
            self._close_node(node, ticker)
            with self._lock:
                self._stopping = False
        return {"state": "stopped", "physical_stop_confirmed": False}

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
                    "last_ret": self._last_ret, "posture_problem": self._fsm_problem,
                    "last": self._last_command, "physical_stop_confirmed": False,
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
        self._sub_node = node
        qos = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                         reliability=ReliabilityPolicy.BEST_EFFORT)
        node.create_subscription(String, topic, self._on_message, qos)
        self._ticker = node.create_timer(WATCHDOG_MS / 2000., self._tick)
        self._executor.add_node(node)

    def _close_node(self, node=None, ticker=None):
        if node is None:
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
            sink = self._sink
            if self._shutdown or not self._running or sink is None or self._paused:
                return
        try:
            command = json.loads(message.data)
        except (ValueError, TypeError):
            return
        outcome = sink.submit(command)
        self._last_command = {"verdict": str(getattr(outcome.verdict, "name", outcome.verdict)),
                              "reason": outcome.reason, "at": time.time()}

    def _tick(self):
        with self._lock:
            sink = self._sink if self._running and not self._paused else None
        if sink is not None:
            sink.tick()

    def _apply(self, values, _gripper=None):
        with self._lock:
            if self._shutdown or not self._running or self._paused:
                return
        vx, vy, wz = float(values[0]), float(values[1]), float(values[5])
        if self._rotate_only and (vx or vy):
            self._suppressed += 1
            vx = vy = 0.
        problem = self._posture_problem()
        if problem:
            self._refused += 1
            return
        self._refused = 0
        self._applied += 1
        if self._dry_run:
            return
        ret = self._client.Move(vx, vy, wz, True)
        if ret != 0:
            self._sdk_errors += 1
            self._last_ret = ret
            if self._sdk_errors == 1 or self._sdk_errors % 100 == 0:
                print(f"[loco_servo] Move returned ret={ret}; execution unconfirmed", flush=True)
        else:
            self._sdk_errors = 0
            self._last_ret = 0

    def _hold(self, *_args):
        # Like R1, watchdog/abort do not pause or latch; a valid command resumes.
        self._holds += 1
        if self._dry_run:
            return
        try:
            self._client.StopMove()
        except Exception as exc:
            print(f"[loco_servo] StopMove failed: {exc}", flush=True)
