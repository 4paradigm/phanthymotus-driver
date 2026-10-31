"""Continuous ``motus.control/1`` velocity input for the S10 Pro."""

from __future__ import annotations

import json
import os
import threading
import time

from common.control import ControlSink, parse_descriptor
from domain_bridge import CoreSubscriber

AXES = ["vx", "vy", "vz", "wx", "wy", "wz"]
VX_LIMIT, VY_LIMIT, WZ_LIMIT = 1.67, 0.4, 1.0


def build_descriptor(expected_hz=10.0):
    return {
        "control_interface": "motus.control/1", "mode": "twist", "dof": 6,
        "joint_names": AXES, "units": {"linear": "m/s", "angular": "rad/s", "time": "s"},
        "limits": {"lower": [-VX_LIMIT, -VY_LIMIT, 0, 0, 0, -WZ_LIMIT],
                   "upper": [VX_LIMIT, VY_LIMIT, 0, 0, 0, WZ_LIMIT],
                   "max_delta_per_step": [0.25, 0.15, 1e-6, 1e-6, 1e-6, 0.2]},
        "groups": [{"name": "base", "offset": 0, "count": 6, "unit": "m/s", "resource": "base"}],
        "frame": "base_link",
        "rate": {"max_hz": 20.0, "expected_hz": float(expected_hz), "watchdog_ms": 300, "max_obs_age_ms": 500},
        "force_torque": None,
    }


class S10LocoServoPlugin:
    def __init__(self, native, ros2=None, motion=None):
        self.native, self.ros2, self.motion = native, ros2, motion
        # Match the repository-wide motion-permission contract: images are
        # read-only unless the deployment explicitly arms physical motion.
        self._motion_enabled = os.environ.get("S10_MOTION_ENABLED", "0") == "1"
        self._descriptor_raw = build_descriptor()
        self._descriptor = parse_descriptor(self._descriptor_raw)
        self._lock = threading.RLock()
        self._sink = None
        self._node = None
        self._timer = None
        self._running = False
        self._paused = False
        self._topic = ""
        self._subscriber = None
        self._reader = None
        self._reader_stop = threading.Event()
        self._dry_run = True
        self._last = None
        self._applied = 0
        self._refused = 0

    def get_tool(self):
        return {"name": "loco_servo", "type": "actuator", "multiInstance": False,
                "description": "S10 Pro continuous body velocity input for visual navigation.",
                "inputSchema": {"type": "object", "properties": {
                    "action": {"type": "string", "enum": ["start", "stop", "pause", "resume", "info", "config"]},
                    "input_topic": {"type": "string"}, "dry_run": {"type": "boolean"},
                    "confirm": {"type": "boolean"}}, "required": ["action"],
                    "x-is-dangerous": True,
                    "x-action-params": {"start": {"params": ["input_topic"]}, "stop": {"params": []},
                        "pause": {"params": []}, "resume": {"params": []}, "info": {"params": []},
                        "config": {"params": ["dry_run", "confirm"]}},
                    "x-resource": ["base"]},
                "topic_in": [{"format": "control/velocity", "desc": "motus.control/1 twist [vx,vy,vz,wx,wy,wz]"}]}

    def start(self):
        pass

    def stop(self):
        self.dispatch("stop", {})

    def dispatch(self, action, args):
        if action == "start": return self._start(args)
        if action == "stop": return self._stop()
        if action == "pause": return self._halt(True)
        if action == "resume": return self._halt(False)
        if action == "info": return {"state": "paused" if self._paused else "running" if self._running else "idle",
                                      "input": self._topic, "dry_run": self._dry_run,
                                      "motion_enabled": self._motion_enabled,
                                      "applied": self._applied, "refused": self._refused,
                                      "control_interface": self._descriptor_raw}
        if action == "config":
            if "dry_run" in args and type(args["dry_run"]) is not bool:
                return {"ok": False, "code": "INVALID_ARGUMENT", "message": "dry_run must be a boolean"}
            if args.get("dry_run") is False and self._dry_run and args.get("confirm") is not True:
                return {"ok": False, "code": "CONFIRM_REQUIRED", "message": "启用实时底盘控制需要 confirm=true"}
            if args.get("dry_run") is True and not self._dry_run and self._running:
                self._hold()
            if "dry_run" in args: self._dry_run = args["dry_run"]
            return {"ok": True, "dry_run": self._dry_run}
        raise ValueError(f"unsupported loco_servo action: {action}")

    def _start(self, args):
        topic = str(args.get("input_topic") or "").strip()
        if not topic: return {"state": "error", "message": "缺少 input_topic，请连接 control/velocity 输出"}
        if not self._motion_enabled:
            return {"state": "error", "message": "S10_MOTION_ENABLED is not set; this driver is read-only"}
        if self.ros2 is None:
            return {"state": "error", "message": "没有 ROS runtime，无法接入 control/velocity"}
        if self.motion is not None and self.motion.is_moving():
            return {"state": "error", "message": "loco 正在控制底盘，请先停止"}
        profile = os.environ.get("S10_CORE_FASTDDS_PROFILE", "/tmp/s10-pro-core-fastdds.xml")
        with self._lock:
            if self._running: return {"state": "error", "message": "loco_servo 已在运行"}
            self._sink = ControlSink(self._descriptor, self._apply, on_watchdog=self._hold, on_abort=self._hold)
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
        return {"state": "running", "input": topic, "control_interface": self._descriptor_raw}

    def _stop(self):
        with self._lock:
            if self._running: self._hold()
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
            if paused: self._hold()
        return {"state": "paused" if paused else "running", "input": self._topic}

    def pause_for_explicit_command(self, _reason=""):
        with self._lock:
            if not self._running or self._paused: return False
            self._paused = True
            self._hold()
        return True

    def _on_message(self, message):
        with self._lock:
            if self._running and not self._paused and self._sink:
                try: self._last = self._sink.submit(json.loads(message.data))
                except (TypeError, ValueError): self._refused += 1

    def _read_loop(self):
        while not self._reader_stop.wait(0.05):
            subscriber = self._subscriber
            if subscriber is None:
                return
            if not subscriber._process.is_alive():
                # The reader drives the watchdog. Once its subprocess is gone,
                # no later tick can revoke the last non-zero native velocity.
                # Hold before changing _running so a concurrent start cannot
                # open a fresh stream between failure detection and the stop.
                self._reader_stop.set()
                self._hold()
                with self._lock:
                    if self._subscriber is subscriber:
                        self._running = False
                        self._paused = True
                        self._sink = None
                        self._refused += 1
                return
            payload = subscriber.latest()
            if payload is not None:
                with self._lock:
                    if self._running and not self._paused and self._sink:
                        try: self._last = self._sink.submit(json.loads(payload))
                        except (TypeError, ValueError): self._refused += 1
            self._tick()

    def _tick(self):
        with self._lock:
            if self._running and not self._paused and self._sink: self._sink.tick()

    def _apply(self, values, _gripper=None):
        if self._dry_run: self._applied += 1; return
        try:
            ret = self.native.send(0x00100001, 0x00110002,
                                   {"X": float(values[0]), "Y": float(values[1]), "Z": 0.0,
                                    "Roll": 0.0, "Pitch": 0.0, "Yaw": float(values[5])})
            if isinstance(ret, dict) and ret.get("ErrorCode", 0) not in (0, "0"): self._refused += 1
            else: self._applied += 1
        except Exception: self._refused += 1

    def _hold(self, *_args):
        if not self._dry_run:
            try: self.native.send(0x00100001, 0x00110002,
                                  {"X": 0.0, "Y": 0.0, "Z": 0.0, "Roll": 0.0, "Pitch": 0.0, "Yaw": 0.0})
            except Exception: pass
