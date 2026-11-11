"""motus.control/1 twist consumer for ActuCore navi (SI units)."""

import json
import math

from common.control import ControlSink, parse_descriptor
from common.vendor_runtime import tool

WATCHDOG_MS = 300


def build_descriptor(config):
    linear = float(config.get("max_linear_m_s", 1.0))
    angular = math.radians(float(config.get("max_angular_deg_s", 60)))
    if not math.isfinite(linear) or not 0 < linear <= 1.3:
        raise ValueError("servo linear limit must be in (0, 1.3] m/s")
    if not math.isfinite(angular) or not 0 < angular <= 2:
        raise ValueError("servo angular limit must be in (0, 2] rad/s")
    descriptor = {
        "control_interface": "motus.control/1", "mode": "twist", "dof": 6,
        "joint_names": ["vx", "vy", "vz", "wx", "wy", "wz"],
        "units": {"linear": "m/s", "angular": "rad/s", "time": "s"},
        "limits": {"lower": [-linear, 0, 0, 0, 0, -angular],
                   "upper": [linear, 0, 0, 0, 0, angular],
                   "max_delta_per_step": [0.1, 1e-6, 1e-6, 1e-6, 1e-6, 0.2]},
        "frame": "base_link",
        "groups": [{"name": "base", "offset": 0, "count": 6,
                    "unit": "m/s", "resource": "base"}],
        "rate": {"max_hz": 50, "expected_hz": 10,
                 "watchdog_ms": WATCHDOG_MS, "max_obs_age_ms": 500},
        "force_torque": None,
    }
    parse_descriptor(descriptor)
    return descriptor


class BunkerServo:
    """All methods and callbacks serialize with the device's motion lock."""

    def __init__(self, device):
        self.device = device
        self.descriptor = build_descriptor(device.config)
        self.running = False
        self.paused = False
        self.topic = ""
        self.subscription = None
        self.sink = None
        self.last_error = ""

    def get_tool(self):
        return tool("bunker_servo", "actuator", "Continuous base control from ActuCore navi",
            {"type": "object", "properties": {
                "action": {"type": "string", "enum": ["start", "stop", "pause", "resume", "info"]},
                "input_topic": {"type": "string"}}, "required": ["action"],
             "x-action-params": {"pause": {"params": [], "description": "Stop and pause navigation"},
                                 "resume": {"params": [], "description": "Resume navigation"}},
             "x-hooks": {"on_interrupt_motion": {"action": "pause"},
                         "on_interrupt_all": {"action": "pause"}},
             "x-is-dangerous": True, "x-resource": ["base"]},
            topic_in=[{"format": "control/velocity", "desc": "motus.control/1 twist, m/s and rad/s"}])

    def _new_sink(self):
        return ControlSink(self.descriptor, self._apply,
                           on_watchdog=self.device.can.stop_motion,
                           on_abort=self.device.can.stop_motion)

    def _apply(self, values, _gripper):
        # set_target revalidates fresh chassis state and critical fault flags.
        self.device.can.set_target(values[0], values[5], lease_s=WATCHDOG_MS / 1000)

    def on_command(self, message):
        with self.device._motion_lock:
            if not self.running or self.paused or self.device._active_move_id is not None:
                return
            try:
                self.sink.submit(json.loads(message.data))
            except Exception as exc:
                # No message-derived strings or per-message log writes.
                self.last_error = type(exc).__name__
                self.paused = True
                self.device.can.stop_motion()

    def tick(self):
        with self.device._motion_lock:
            if self.running and not self.paused:
                self.sink.tick()

    def pause_locked(self):
        if self.running:
            self.paused = True
            if self.device._active_move_id is None:
                self.device.can.stop_motion()

    def dispatch(self, action, args):
        with self.device._motion_lock:
            if action == "info":
                return {"state": "paused" if self.paused else "running" if self.running else "idle",
                        "input_topic": self.topic, "control_interface": self.descriptor,
                        "last_error": self.last_error,
                        "sink": self.sink.stats() if self.sink else {}}
            if action == "stop":
                if self.subscription is not None:
                    self.device.bridge.unbind_control(self.subscription)
                self.subscription = None
                if self.running and self.device._active_move_id is None:
                    self.device.can.stop_motion()
                self.running = self.paused = False
                self.sink = None
                self.topic = ""
                return {"state": "idle"}
            if action == "pause":
                self.pause_locked()
                return {"state": "paused" if self.running else "idle"}
            if action not in ("start", "resume"):
                raise ValueError("unknown bunker_servo action")
            if not self.device._motion_enabled:
                raise RuntimeError("movement is disabled by BUNKER_MOTION_ENABLED=0")
            if self.device._active_move_id is not None:
                raise RuntimeError("base_drive.move is running")
            if action == "resume" and not self.running:
                raise ValueError("start bunker_servo with an input_topic first")
            if action == "start":
                topic = args.get("input_topic")
                if not isinstance(topic, str) or not topic.strip():
                    raise ValueError("input_topic is required")
                if self.running:
                    raise RuntimeError("bunker_servo is already started")
            sink = self._new_sink()
            # Validate/bind the ROS topic before any chassis side effect. A
            # callback cannot apply a target until this locked start completes.
            subscription = (self.device.bridge.bind_control(topic, self.on_command)
                            if action == "start" else None)
            try:
                self.device.can.request_control_mode()
                self.device.can.validate_target(0.0, 0.0)
                self.device.can.stop_motion()
            except Exception:
                if subscription is not None:
                    self.device.bridge.unbind_control(subscription)
                raise
            if action == "start":
                self.subscription = subscription
                self.topic = topic
            self.sink = sink
            self.running = True
            self.paused = False
            self.last_error = ""
            return {"state": "running", "input_topic": self.topic,
                    "control_interface": self.descriptor}
