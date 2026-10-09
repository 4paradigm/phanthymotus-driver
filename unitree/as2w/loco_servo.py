"""AS2W motus.control/1 twist card. SDK writes live in one bounded worker."""
from __future__ import annotations

import json
import threading

from common.control import parse_descriptor
from as2w_control import AXES, VelocityController, finite


def build_descriptor(config=None):
    cfg = config or {}

    def number(key, default, lower, upper):
        value = cfg.get(key, default)
        if not finite(value) or not lower <= value <= upper:
            raise ValueError(f"{key} must be finite in [{lower}, {upper}]")
        return float(value)

    hz = number("expected_hz", 10.0, 1, 20)
    # These are commissioning policy ceilings, not measured hardware limits.
    vx = number("vx_limit", 0.30, 0.01, 1.5)
    vy = number("vy_limit", 0.20, 0.01, 1.0)
    wz = number("wz_limit", 0.50, 0.01, 2.0)
    acceleration = [number("linear_acceleration", 0.30, 0.01, 2.0)] * 2 + [0.0] * 3 + [
        number("angular_acceleration", 0.50, 0.01, 4.0)]
    descriptor = {
        "control_interface": "motus.control/1", "mode": "twist", "dof": 6,
        "joint_names": list(AXES), "frame": "base_link",
        "units": {"linear": "m/s", "angular": "rad/s", "time": "s"},
        "limits": {"lower": [-vx, -vy, 0.0, 0.0, 0.0, -wz],
                   "upper": [vx, vy, 0.0, 0.0, 0.0, wz]},
        "groups": [{"name": "base", "offset": 0, "count": 6,
                    "unit": "m/s", "resource": "base"}],
        "rate": {"max_hz": 20.0, "expected_hz": hz,
                 "watchdog_ms": int(number("watchdog_ms", 300, 100, 1000)),
                 "max_obs_age_ms": int(number("max_obs_age_ms", 500, 50, 1000))},
        "force_torque": None,
        "acceleration_limits": acceleration,
        "calibration": {"axes_verified": False, "deadband": "unknown",
                        "footprint": "unknown", "limits_source": "commissioning-policy"},
    }
    footprint = cfg.get("footprint")
    if footprint is not None:
        if not isinstance(footprint, dict) or footprint.get("source") not in ("measured", "vendor-spec"):
            raise ValueError("footprint requires measured or vendor-spec provenance")
        for key in ("half_width", "front", "rear", "height"):
            if not finite(footprint.get(key)) or footprint[key] <= 0:
                raise ValueError("footprint requires positive finite " + key)
        descriptor["footprint"] = dict(footprint)
        descriptor["calibration"]["footprint"] = footprint["source"]
    parse_descriptor(descriptor)
    return descriptor


class LocoServoPlugin:
    PREFIX = "locoservo"

    def __init__(self, config, namespace, executor, control_client,
                 loco_plugin=None, state_provider=None, odom_provider=None):
        self._config = dict(config or {})
        for key in ("dry_run", "rotate_only"):
            if key in self._config and not isinstance(self._config[key], bool):
                raise ValueError(key + " must be boolean")
        self._namespace, self._executor = namespace, executor
        self._loco = loco_plugin
        self._descriptor = build_descriptor(self._config)
        self._controller = VelocityController(
            control_client, self._descriptor,
            dry_run=self._config.get("dry_run", True),
            rotate_only=self._config.get("rotate_only", False),
            state_provider=state_provider, odom_provider=odom_provider,
            conflict=self._chassis_conflict)
        self._node = None
        self._topic = ""
        self._lock = threading.RLock()
        self._last_rejection = ""
        if loco_plugin and callable(getattr(loco_plugin, "attach_servo", None)):
            loco_plugin.attach_servo(self)

    def get_tool(self):
        actions = ["start", "stop", "pause", "resume", "reset_fault", "info"]
        return {"name": "loco_servo", "type": "actuator", "multiInstance": False,
                "description": "As2W continuous body velocity stream. Defaults to dry_run; "
                               "real control requires explicit configuration and a ready standing chassis.",
                "inputSchema": {"type": "object", "required": ["action"], "properties": {
                    "action": {"type": "string", "enum": actions},
                    "input_topic": {"type": "string"}},
                    "x-action-params": {
                        "pause": {"params": [], "description": "Stop and pause the navigation stream."},
                        "resume": {"params": [], "description": "Reacquire control and accept only fresh commands."},
                        "reset_fault": {"params": [], "description": "Retry stopping, acknowledge a latched fault and remain paused."}},
                    "x-hooks": {"on_interrupt_motion": {"action": "pause"},
                                "on_interrupt_all": {"action": "pause"}},
                    "x-is-dangerous": True, "x-resource": ["base"]},
                "topic_in": [{"format": "control/velocity", "desc": "motus.control/1 twist [vx,vy,vz,wx,wy,wz]"}],
                "configSchema": {"type": "object", "properties": {
                    "dry_run": {"type": "boolean", "default": True,
                                "description": "Validate without moving. Entering dry_run first stops real motion; resume is explicit."},
                    "rotate_only": {"type": "boolean", "default": False,
                                    "description": "Suppress translation; change only while paused."}}}}

    def _chassis_conflict(self):
        return bool(self._loco and callable(getattr(self._loco, "is_moving", None))
                    and self._loco.is_moving())

    def start(self):
        # Bundle construction is not permission to consume a command topic.
        pass

    def stop(self):
        result = self._controller.close()
        self._disconnect()
        return result

    def dispatch(self, action, args):
        if action == "info":
            return self._info()
        if action == "config":
            unknown = set(args) - {"dry_run", "rotate_only", "_tool_name"}
            if unknown:
                return {"ok": False, "state": "error", "error": "unknown config fields: " + ", ".join(sorted(unknown))}
            result = self._controller.configure(**{k: args[k] for k in ("dry_run", "rotate_only") if k in args})
            return {**self._info(), **result}
        if action == "start":
            return self._start(args)
        if action == "pause":
            return self._controller.pause(timeout=1.5)
        if action == "resume":
            if self._node is None:
                return {"ok": False, "state": "error", "error": "connect a control topic before resume"}
            return self._controller.activate()
        if action == "reset_fault":
            return self._controller.reset_fault()
        if action == "stop":
            result = self._controller.pause("card stopped", timeout=1.5)
            self._disconnect()
            return {**result, "state": "idle" if result.get("ok") else "error"}
        return None

    def _start(self, args):
        topic = args.get("input_topic")
        if not topic:
            topics = args.get("input_topics") or []
            topic = topics[0] if isinstance(topics, list) and topics else ""
        if not isinstance(topic, str) or not topic.strip():
            return {"ok": False, "state": "error", "error": "input_topic is required"}
        if self._executor is None:
            return {"ok": False, "state": "error", "error": "ROS executor unavailable"}
        with self._lock:
            if self._node is not None:
                return {"ok": False, "state": "error", "error": "already connected; stop before rewiring"}
            try:
                from rclpy.node import Node
                from std_msgs.msg import String
                node = Node("as2w_loco_servo")
                node.create_subscription(String, topic.strip(), self._on_message, 1)
                self._executor.add_node(node)
                self._node, self._topic = node, topic.strip()
            except Exception as exc:
                if 'node' in locals():
                    node.destroy_node()
                return {"ok": False, "state": "error", "error": f"subscription failed: {exc}"}
            result = self._controller.activate()
            if not result.get("ok"):
                self._disconnect()
                return result
            return {**self._info(), **result}

    def _disconnect(self):
        with self._lock:
            node, self._node = self._node, None
            self._topic = ""
        if node is not None:
            try:
                self._executor.remove_node(node)
            finally:
                node.destroy_node()

    def _on_message(self, message):
        try:
            if len(message.data) > 16384:
                raise ValueError("command exceeds 16 KiB")
            command = json.loads(message.data)
        except (ValueError, TypeError, AttributeError) as exc:
            self._last_rejection = "invalid control JSON: " + str(exc)
            return
        result = self._controller.submit(command)
        if not result["ok"]:
            self._last_rejection = result["reason"]

    def pause_for_explicit_command(self, reason="explicit chassis action", timeout=1.5):
        return self._controller.pause(reason, timeout=timeout)

    def owns_chassis(self):
        return self._controller.info()["owner_acquired"]

    def is_running(self):
        return self._controller.info()["state"] == "running"

    def _info(self):
        status = self._controller.info()
        descriptor = dict(self._descriptor)
        descriptor["dry_run"] = status["dry_run"]
        descriptor["rotate_only"] = status["rotate_only"]
        degraded = ["AS2W velocity axes and deadband have not been measured by this driver.",
                    "SDK acknowledgement does not verify physical stopping; firmware link-loss behaviour is unverified."]
        if "footprint" not in descriptor:
            degraded.append("AS2W footprint is undeclared; upstream fallback geometry is not a measured clearance.")
        return {**status, "input": self._topic, "connected": self._node is not None,
                "control_interface": descriptor, "degraded": degraded,
                "last_rejection": self._last_rejection}
